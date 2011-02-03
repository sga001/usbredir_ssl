/* usbredirhost.c usb network redirection usb host code.

   Copyright 2010-2011 Red Hat, Inc.

   Red Hat Authors:
   Hans de Goede <hdegoede@redhat.com>

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "usbredirhost.h"

#define MAX_ENDPOINTS  32
#define MAX_INTERFACES 32 /* Max 32 endpoints and thus interfaces */
#define CTRL_TIMEOUT 5000 /* USB specifies a 5 second max timeout */
#define ISO_TIMEOUT  1000

#define MAX_ISO_TRANSFER_COUNT       16
#define MAX_ISO_PACKETS_PER_TRANSFER 32
/* Special iso_packet_idx value indicating a submitted transfer */
#define ISO_SUBMITTED_IDX            -1

/* Macros to go from an endpoint address to an index for our ep array */
#define EP2I(ep_address) (((ep_address & 0x80) >> 3) | (ep_address & 0x0f))
#define I2EP(i) (((i & 0x10) << 3) | (i & 0x0f))

struct usbredirtransfer {
    struct usbredirhost *host;        /* Back pointer to the the redirhost */
    struct libusb_transfer *transfer; /* Back pointer to the libusb transfer */
    uint32_t id;
    int iso_packet_idx;
    union {
        struct usb_redir_control_packet_header control_header;
        struct usb_redir_bulk_packet_header bulk_header;
        struct usb_redir_iso_packet_header iso_header;
    };
    struct usbredirtransfer *next;
    struct usbredirtransfer *prev;
};

struct usbredirhost_ep {
    uint8_t type;
    uint8_t interface; /* bInterfaceNumber this ep belongs to */
    uint8_t iso_started;
    uint8_t iso_pkts_per_transfer;
    uint8_t iso_transfer_count;
    int iso_out_idx;
    int max_packetsize;
    struct usbredirtransfer *iso_transfer[MAX_ISO_TRANSFER_COUNT];
};

struct usbredirhost {
    struct usbredirparser *parser;
    usbredirparser_log log_func;
    usbredirparser_read read_func;
    usbredirparser_write write_func;
    void *func_priv;
    libusb_device *dev;
    libusb_device_handle *handle;
    struct libusb_config_descriptor *config;
    int active_config;
    int claimed;
    struct usbredirhost_ep endpoint[MAX_ENDPOINTS];
    uint8_t driver_detached[MAX_INTERFACES];
    uint8_t alt_setting[MAX_INTERFACES];
    struct usbredirtransfer transfers_head;
};

static void va_log(struct usbredirhost *host, int verbose,
    const char *fmt, ...)
{
    char buf[512];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    
    host->log_func(host->func_priv, verbose, buf);
}

#define ERROR(...)   va_log(host, usbredirparser_error, \
                            "usbredirhost error: " __VA_ARGS__)
#define WARNING(...) va_log(host, usbredirparser_warning, \
                            "usbredirhost warning: " __VA_ARGS__)
#define INFO(...)    va_log(host, usbredirparser_info, \
                            "usbredirhost info: " __VA_ARGS__)

static void usbredirhost_reset(void *priv, uint32_t id);
static void usbredirhost_set_configuration(void *priv, uint32_t id,
    struct usb_redir_set_configuration_header *set_configuration);
static void usbredirhost_get_configuration(void *priv, uint32_t id);
static void usbredirhost_set_alt_setting(void *priv, uint32_t id,
    struct usb_redir_set_alt_setting_header *set_alt_setting);
static void usbredirhost_get_alt_setting(void *priv, uint32_t id,
    struct usb_redir_get_alt_setting_header *get_alt_setting);
static void usbredirhost_start_iso_stream(void *priv, uint32_t id,
    struct usb_redir_start_iso_stream_header *start_iso_stream);
static void usbredirhost_stop_iso_stream(void *priv, uint32_t id,
    struct usb_redir_stop_iso_stream_header *stop_iso_stream);
static void usbredirhost_alloc_bulk_streams(void *priv, uint32_t id,
    struct usb_redir_alloc_bulk_streams_header *alloc_bulk_streams);
static void usbredirhost_free_bulk_streams(void *priv, uint32_t id,
    struct usb_redir_free_bulk_streams_header *free_bulk_streams);
static void usbredirhost_cancel_data_packet(void *priv, uint32_t id);
static void usbredirhost_control_packet(void *priv, uint32_t id,
    struct usb_redir_control_packet_header *control_header,
    uint8_t *data, int data_len);
static void usbredirhost_bulk_packet(void *priv, uint32_t id,
    struct usb_redir_bulk_packet_header *bulk_header,
    uint8_t *data, int data_len);
static void usbredirhost_iso_packet(void *priv, uint32_t id,
    struct usb_redir_iso_packet_header *iso_header,
    uint8_t *data, int data_len);

static void usbredirhost_cancel_iso_stream(struct usbredirhost *host,
    uint8_t ep, int free);

static void usbredirhost_log(void *priv, int level, const char *msg)
{
    struct usbredirhost *host = priv;

    host->log_func(host->func_priv, level, msg);
}

static int usbredirhost_read(void *priv, uint8_t *data, int count)
{
    struct usbredirhost *host = priv;

    return host->read_func(host->func_priv, data, count);
}

static int usbredirhost_write(void *priv, uint8_t *data, int count)
{
    struct usbredirhost *host = priv;

    return host->write_func(host->func_priv, data, count);
}

/* One function to convert either a transfer status code, or a libusb error
   code to a usb_redir status. We handle both in one conversion function so
   that we can pass error codes as status codes to the completion handler
   in case of submission error (the codes don't overlap), using the completion
   handler to report back the status and cleanup as it would on completion of
   a successfully submitted transfer. */
static int libusb_status_or_error_to_redir_status(int status)
{
    switch (status) {
        case LIBUSB_TRANSFER_COMPLETED:
            return usb_redir_success;
        case LIBUSB_TRANSFER_ERROR:
            return usb_redir_ioerror;
        case LIBUSB_TRANSFER_TIMED_OUT:
            return usb_redir_timeout;
        case LIBUSB_TRANSFER_CANCELLED:
            return usb_redir_cancelled;
        case LIBUSB_TRANSFER_STALL:
            return usb_redir_stall;
        case LIBUSB_TRANSFER_NO_DEVICE:
            return usb_redir_disconnected;
        case LIBUSB_TRANSFER_OVERFLOW:
            return usb_redir_ioerror;

        case LIBUSB_ERROR_INVALID_PARAM: 
            return usb_redir_inval;
        case LIBUSB_ERROR_NO_DEVICE:
            return usb_redir_disconnected;
        case LIBUSB_ERROR_TIMEOUT:
            return usb_redir_timeout;
        default:
            return usb_redir_ioerror;
    }
}

static void usbredirhost_parse_config(struct usbredirhost *host)
{
    int i, j;
    const struct libusb_interface_descriptor *intf_desc;
    struct usb_redir_ep_info_header ep_info;
    uint8_t ep_address;

    for (i = 0; i < MAX_ENDPOINTS; i++) {
        if ((i & 0x0f) == 0) {
            host->endpoint[i].type = usb_redir_type_control;
        } else {
            host->endpoint[i].type = usb_redir_type_invalid;
        }
        host->endpoint[i].interface = 0;
    }

    for (i = 0; i < host->config->bNumInterfaces; i++) {
        intf_desc =
            &host->config->interface[i].altsetting[host->alt_setting[i]];
        for (j = 0; j < intf_desc->bNumEndpoints; j++) {
            ep_address = intf_desc->endpoint[j].bEndpointAddress;
            host->endpoint[EP2I(ep_address)].type =
                intf_desc->endpoint[j].bmAttributes &
                    LIBUSB_TRANSFER_TYPE_MASK;
            host->endpoint[EP2I(ep_address)].interface =
                intf_desc->bInterfaceNumber;
            host->endpoint[EP2I(ep_address)].max_packetsize =
                libusb_get_max_iso_packet_size(host->dev, ep_address);
        }
    }

    for (i = 0; i < MAX_ENDPOINTS; i++) {
        ep_info.type[i] = host->endpoint[i].type;
        ep_info.interface[i] = host->endpoint[i].interface;
    }
    usbredirparser_send_ep_info(host->parser, &ep_info);
}

static int usbredirhost_claim(struct usbredirhost *host)
{
    int i, n, r, ret = usb_redir_success;

    if (host->config) {
        libusb_free_config_descriptor(host->config);
        host->config = NULL;
    }

    r = libusb_get_config_descriptor_by_value(host->dev, host->active_config,
                                              &host->config);
    if (r < 0) {
        ERROR("could not get descriptors for configuration %d: %d",
              host->active_config, r);
        return libusb_status_or_error_to_redir_status(r);
    }
    if (host->config->bNumInterfaces > MAX_INTERFACES) {
        ERROR("usb decriptor has too much intefaces (%d > %d)",
              (int)host->config->bNumInterfaces, MAX_INTERFACES);
        return usb_redir_ioerror;
    }

    /* All interfaces begin alt setting 0 when (re)claimed */
    memset(host->alt_setting, 0, MAX_INTERFACES);

    for (i = 0; i < host->config->bNumInterfaces; i++) {
        n = host->config->interface[i].altsetting[0].bInterfaceNumber;

        r = libusb_detach_kernel_driver(host->handle, n);
        if (r < 0 && r != LIBUSB_ERROR_NOT_FOUND) {
            ERROR("could not detach driver from interface %d (configuration %d): %d",
                  n, host->active_config, r);
            ret = libusb_status_or_error_to_redir_status(r);
            goto error;
        }
        /* Note indexed by i not n !! (too ensure we don't go out of bound) */
        host->driver_detached[i] = (r != LIBUSB_ERROR_NOT_FOUND);

        r = libusb_claim_interface(host->handle, n);
        if (r < 0) {
            ERROR("could not claim interface %d (configuration %d): %d",
                  n, host->active_config, r);
            ret = libusb_status_or_error_to_redir_status(r);
            goto error;
        }
    }

    usbredirhost_parse_config(host);
    host->claimed = 1;
    return ret;

error:
    if (host->driver_detached[i]) {
        libusb_attach_kernel_driver(host->handle, n);
    }
    for (i--; i >= 0; i--) {
        n = host->config->interface[i].altsetting[0].bInterfaceNumber;

        libusb_release_interface(host->handle, n);

        if (host->driver_detached[i]) {
            r = libusb_attach_kernel_driver(host->handle, n);
            if (r == 0) {
                host->driver_detached[i] = 0;
            }
        }
    }
    return ret;
}

static int usbredirhost_release(struct usbredirhost *host)
{
    int i, n, r, ret = usb_redir_success;

    if (!host->claimed) {
        return usb_redir_success;
    }

    for (i = 0; i < host->config->bNumInterfaces; i++) {
        n = host->config->interface[i].altsetting[0].bInterfaceNumber;

        r = libusb_release_interface(host->handle, n);
        if (r < 0 && r != LIBUSB_ERROR_NOT_FOUND) {
            ERROR("could not release interface %d (configuration %d): %d",
                  n, host->active_config, r);
            ret = usb_redir_ioerror;
        }

        if (host->driver_detached[i]) {
            r = libusb_attach_kernel_driver(host->handle, n);
            if (r < 0) {
                ERROR("could not re-attach driver to interface %d (configuration %d): %d",
                      n, host->active_config, r);
                ret = usb_redir_ioerror;
            }
            if (r == 0) {
                host->driver_detached[i] = 0;
            }
        }
    }

    host->claimed = 0;
    return ret;
}

struct usbredirhost *usbredirhost_open(libusb_device_handle *usb_dev_handle,
    usbredirparser_log log_func,
    usbredirparser_read  read_guest_data_func,
    usbredirparser_write write_guest_data_func,
    void *func_priv, const char *version)
{
    struct usbredirhost *host;
    int r;

    host = calloc(1, sizeof(*host));
    if (!host) {
        log_func(func_priv, usbredirparser_error,
            "usbredirhost error: Out of memory allocating usbredirhost");
        return NULL;
    }

    host->dev = libusb_get_device(usb_dev_handle);
    host->handle = usb_dev_handle;
    host->log_func = log_func;
    host->read_func = read_guest_data_func;
    host->write_func = write_guest_data_func;
    host->func_priv = func_priv;
    host->parser = usbredirparser_create(usbredirhost_log,
                                         usbredirhost_read,
                                         usbredirhost_write,
                                         NULL, /* report ep types */
                                         usbredirhost_reset,
                                         NULL, /* reset status */
                                         usbredirhost_set_configuration,
                                         usbredirhost_get_configuration,
                                         NULL, /* config status */
                                         usbredirhost_set_alt_setting,
                                         usbredirhost_get_alt_setting,
                                         NULL, /* alt status */
                                         usbredirhost_start_iso_stream,
                                         usbredirhost_stop_iso_stream,
                                         NULL, /* iso status */
                                         usbredirhost_alloc_bulk_streams,
                                         usbredirhost_free_bulk_streams,
                                         NULL, /* bulk streams status */
                                         usbredirhost_cancel_data_packet,
                                         usbredirhost_control_packet,
                                         usbredirhost_bulk_packet,
                                         usbredirhost_iso_packet,
                                         host, version, NULL, 0,
                                         usbredirparser_fl_usb_host);
    if (!host->parser) {
        /* usbredirparser_create will have logged the error */
        usbredirhost_close(host);
        return NULL;
    }

    r = libusb_get_configuration(host->handle, &host->active_config);
    if (r < 0) {
        ERROR("could not get active configuration: %d", r);
        usbredirhost_close(host);
        return NULL;
    }

    if (usbredirhost_claim(host) != usb_redir_success) {
        usbredirhost_close(host);
        return NULL;
    }

    return host;
}

void usbredirhost_close(struct usbredirhost *host)
{
    if (!host) {
        return;
    }

    if (host->claimed) {
        usbredirhost_release(host);
    }
    if (host->config) {
        libusb_free_config_descriptor(host->config);
    }
    if (host->parser) {
        usbredirparser_destroy(host->parser);
    }
    if (host->handle) {
        libusb_close(host->handle);
    }
    free(host);
}

int usbredirhost_read_guest_data(struct usbredirhost *host)
{
    return usbredirparser_do_read(host->parser);
}

int usbredirhost_has_data_to_write(struct usbredirhost *host)
{
    return usbredirparser_has_data_to_write(host->parser);
}

int usbredirhost_write_guest_data(struct usbredirhost *host)
{
    return usbredirparser_do_write(host->parser);
}

/**************************************************************************/

static struct usbredirtransfer *usbredirhost_alloc_transfer(
    struct usbredirhost *host, int iso_packets)
{
    struct usbredirtransfer *redir_transfer;
    struct libusb_transfer *libusb_transfer;

    redir_transfer  = calloc(1, sizeof(*redir_transfer));
    libusb_transfer = libusb_alloc_transfer(iso_packets);
    if (!redir_transfer || !libusb_transfer) {
        ERROR("out of memory allocating usb transfer, dropping packet");
        free(redir_transfer);
        libusb_free_transfer(libusb_transfer);
        return NULL;
    }
    redir_transfer->host       = host;
    redir_transfer->transfer   = libusb_transfer;
    libusb_transfer->user_data = redir_transfer;

    return redir_transfer;
}

static void usbredirhost_free_transfer(struct usbredirtransfer *transfer)
{
    if (!transfer)
        return;

    free(transfer->transfer->buffer);
    libusb_free_transfer(transfer->transfer);
    free(transfer);
}

static void usbredirhost_add_transfer(struct usbredirhost *host,
    struct usbredirtransfer *new_transfer)
{
    struct usbredirtransfer *transfer = &host->transfers_head;

    while (transfer->next) {
        transfer = transfer->next;
    }

    new_transfer->prev = transfer;
    transfer->next = new_transfer;
}

static void usbredirhost_remove_and_free_transfer(
    struct usbredirtransfer *transfer)
{
    if (transfer->next)
        transfer->next->prev = transfer->prev;
    if (transfer->prev)
        transfer->prev->next = transfer->next;

    usbredirhost_free_transfer(transfer);
}

static struct usbredirtransfer *usbredirhost_find_transfer_by_id(
    struct usbredirhost *host, uint32_t id)
{
    struct usbredirtransfer *transfer = &host->transfers_head;

    while (transfer->next) {
        if (transfer->id == id) {
            return transfer;
        }
        transfer = transfer->next;
    }
    return NULL;
}

static void usbredirhost_cancel_pending_urbs(struct usbredirhost *host)
{
    struct usbredirtransfer *transfer = &host->transfers_head;

    while (transfer->next) {
        libusb_cancel_transfer(transfer->transfer);
        transfer = transfer->next;
    }
}

static void usbredirhost_cancel_pending_urbs_on_ep(
    struct usbredirhost *host, uint8_t ep)
{
    struct usbredirtransfer *transfer = &host->transfers_head;

    while (transfer->next) {
        if (transfer->transfer->endpoint == ep) {
            libusb_cancel_transfer(transfer->transfer);
        }
        transfer = transfer->next;
    }
}

static int usbredirhost_bInterfaceNumber_to_index(
    struct usbredirhost *host, uint8_t bInterfaceNumber)
{
    int i, n;

    for (i = 0; i < host->config->bNumInterfaces; i++) {
        n = host->config->interface[i].altsetting[0].bInterfaceNumber;
        if (n == bInterfaceNumber) {
            return i;
        }
    }

    ERROR("invalid bNumInterface: %d\n", (int)bInterfaceNumber);
    return -1;
}

static void usbredirhost_send_iso_status(struct usbredirhost *host,
    uint32_t id, uint8_t endpoint, uint8_t status)
{
    struct usb_redir_iso_stream_status_header header;

    header.endpoint = endpoint;
    header.status = status;
    usbredirparser_send_iso_stream_status(host->parser, id, &header);
}

static int usbredirhost_submit_iso_transfer(struct usbredirhost *host,
    struct usbredirtransfer *transfer)
{
    int r;

    r = libusb_submit_transfer(transfer->transfer);
    if (r < 0) {
        uint8_t ep = transfer->transfer->endpoint;
        ERROR("submitting iso transfer on ep %02x: %d, stopping stream",
              (unsigned int)ep, r);
        usbredirhost_cancel_iso_stream(host, ep, 1);
        return libusb_status_or_error_to_redir_status(r);
    }

    transfer->iso_packet_idx = ISO_SUBMITTED_IDX;
    return usb_redir_success;
}

/* Returns 1 if the status is ok, 0 if something is wrong with the packet */
static int usbredirhost_handle_iso_status(struct usbredirhost *host,
    uint32_t id, uint8_t ep, int r)
{
    int i, status;

    switch (r) {
    case LIBUSB_TRANSFER_COMPLETED:
        return 1;
    case LIBUSB_TRANSFER_CANCELLED:
        /* Stream was intentionally stopped */
        return 0;
    case LIBUSB_TRANSFER_STALL:
        /* Uhoh, Cancel the stream (but don't free it), clear stall
           and in case of an input stream resubmit the transfers */
        WARNING("iso stream on endpoint %02X stalled, clearing stall",
                (unsigned int)ep);
        usbredirhost_cancel_iso_stream(host, ep, 0);
        r = libusb_clear_halt(host->handle, ep);
        if (r < 0) {
            usbredirhost_send_iso_status(host, id, ep,
                                  libusb_status_or_error_to_redir_status(r));
            return 0;
        }
        if (ep & LIBUSB_ENDPOINT_IN) {
            for (i = 0; i < host->endpoint[EP2I(ep)].iso_transfer_count; i++) {
                host->endpoint[EP2I(ep)].iso_transfer[i]->id =
                    i * host->endpoint[EP2I(ep)].iso_pkts_per_transfer;
                status = usbredirhost_submit_iso_transfer(host,
                                    host->endpoint[EP2I(ep)].iso_transfer[i]);
                if (status != usb_redir_success) {
                    usbredirhost_send_iso_status(host, id, ep, status);
                    return 0;
                }
            }
            host->endpoint[EP2I(ep)].iso_started = 1;
        }
        return 0;
    default:
        ERROR("iso stream error on endpoint %02X: %d", (unsigned int)ep, r);
        usbredirhost_send_iso_status(host, id, ep,
                                   libusb_status_or_error_to_redir_status(r));
        return 0;
    }
}

static void usbredirhost_iso_packet_complete(
    struct libusb_transfer *libusb_transfer)
{
    struct usbredirtransfer *transfer = libusb_transfer->user_data;
    struct usbredirhost *host = transfer->host;
    uint8_t ep = libusb_transfer->endpoint;
    int i, status;

    /* Mark transfer completed (iow not submitted) */
    transfer->iso_packet_idx = 0;

    /* Check overal transfer status */
    if (!usbredirhost_handle_iso_status(host, transfer->id, ep,
                                        libusb_transfer->status)) {
        return;
    }

    /* Check per packet status and send ok input packets to usb-guest */
    for (i = 0; i < libusb_transfer->num_iso_packets; i++) {
        if (!usbredirhost_handle_iso_status(host, transfer->id, ep,
                               libusb_transfer->iso_packet_desc[i].status)) {
            return;
        }
        if (ep & LIBUSB_ENDPOINT_IN) {
            struct usb_redir_iso_packet_header iso_header;

            iso_header.endpoint = ep;
            iso_header.status   = usb_redir_success;
            iso_header.length   =
                libusb_transfer->iso_packet_desc[i].actual_length;

            usbredirparser_send_iso_packet(host->parser, transfer->id,
                           &iso_header,
                           libusb_get_iso_packet_buffer(libusb_transfer, i),
                           libusb_transfer->iso_packet_desc[i].actual_length);
            transfer->id++;
        }
    }

    /* And for input transfers resubmit the transfer (output transfers
       get resubmitted when they have all their packets filled with data) */
    if (ep & LIBUSB_ENDPOINT_IN) {
        transfer->id += (host->endpoint[EP2I(ep)].iso_transfer_count - 1) *
                        libusb_transfer->num_iso_packets;
        status = usbredirhost_submit_iso_transfer(host, transfer);
        if (status != usb_redir_success) {
            usbredirhost_send_iso_status(host, transfer->id, ep, status);
        }
    }
}

static int usbredirhost_alloc_iso_stream(struct usbredirhost *host,
    uint8_t ep, uint8_t pkts_per_transfer, uint8_t transfer_count)
{
    int i, buf_size;
    unsigned char *buffer;

    if (host->endpoint[EP2I(ep)].type != LIBUSB_TRANSFER_TYPE_ISOCHRONOUS) {
        ERROR("start iso stream on non iso endpoint");
        return usb_redir_inval;
    }

    if (   pkts_per_transfer < 1 ||
           pkts_per_transfer > MAX_ISO_PACKETS_PER_TRANSFER ||
           transfer_count < 1 ||
           transfer_count > MAX_ISO_TRANSFER_COUNT) {
        ERROR("start iso stream pkts_per_urb or no_urbs invalid");
        return usb_redir_inval;
    }

    for (i = 0; i < transfer_count; i++) {
        host->endpoint[EP2I(ep)].iso_transfer[i] =
            usbredirhost_alloc_transfer(host, pkts_per_transfer);
        if (!host->endpoint[EP2I(ep)].iso_transfer[i]) {
            goto alloc_error;
        }

        buf_size = host->endpoint[EP2I(ep)].max_packetsize * pkts_per_transfer;
        buffer = malloc(buf_size);
        if (!buffer) {
            goto alloc_error;
        }
        libusb_fill_iso_transfer(
            host->endpoint[EP2I(ep)].iso_transfer[i]->transfer, host->handle,
            ep, buffer, buf_size, pkts_per_transfer,
            usbredirhost_iso_packet_complete,
            host->endpoint[EP2I(ep)].iso_transfer[i], ISO_TIMEOUT);
        libusb_set_iso_packet_lengths(
            host->endpoint[EP2I(ep)].iso_transfer[i]->transfer,
            host->endpoint[EP2I(ep)].max_packetsize);
    }
    host->endpoint[EP2I(ep)].iso_out_idx = 0;
    host->endpoint[EP2I(ep)].iso_pkts_per_transfer = pkts_per_transfer;
    host->endpoint[EP2I(ep)].iso_transfer_count = transfer_count;

    return usb_redir_success;

alloc_error:
    ERROR("out of memory allocating iso stream buffers");
    do {
        usbredirhost_free_transfer(host->endpoint[EP2I(ep)].iso_transfer[i]);
        host->endpoint[EP2I(ep)].iso_transfer[i] = NULL;
        i--;
    } while (i >= 0);
    return usb_redir_ioerror;
}

static void usbredirhost_cancel_iso_stream(struct usbredirhost *host,
    uint8_t ep, int do_free)
{
    int i;
    struct usbredirtransfer *transfer;

    for (i = 0; i < host->endpoint[EP2I(ep)].iso_transfer_count; i++) {
        transfer = host->endpoint[EP2I(ep)].iso_transfer[i];
        if (transfer->iso_packet_idx == ISO_SUBMITTED_IDX) {
            libusb_cancel_transfer(transfer->transfer);
        }
        if (do_free) {
            if (transfer->iso_packet_idx == ISO_SUBMITTED_IDX) {
                /* Tell libusb to free the buffer and transfer when the
                   transfer has completed (or was successfully cancelled) */
                transfer->transfer->flags = LIBUSB_TRANSFER_FREE_TRANSFER |
                                            LIBUSB_TRANSFER_FREE_BUFFER;
                /* Free our transfer struct (and only our struct) now */
                free(transfer);
            } else {
                usbredirhost_free_transfer(transfer);
            }
            host->endpoint[EP2I(ep)].iso_transfer[i] = NULL;
        }
    }
    host->endpoint[EP2I(ep)].iso_out_idx = 0;
    host->endpoint[EP2I(ep)].iso_started = 0;
    if (do_free) {
        host->endpoint[EP2I(ep)].iso_pkts_per_transfer = 0;
        host->endpoint[EP2I(ep)].iso_transfer_count = 0;
    }
}

/**************************************************************************/

static void usbredirhost_reset(void *priv, uint32_t id)
{
    struct usbredirhost *host = priv;
    struct usb_redir_reset_status_header status = { usb_redir_disconnected };
    int r;

    r = libusb_reset_device(host->handle);
    if (r < 0) {
        ERROR("resetting device: %d", r);
        goto exit;
    }
    /* Resetting releases the interfaces and re-attaches the driver */
    memset(host->driver_detached, 0, sizeof(host->driver_detached));
    host->claimed = 0;

    r = libusb_get_configuration(host->handle, &host->active_config);
    if (r < 0) {
        ERROR("could not get active configuration: %d", r);
        goto exit;
    }

    if (usbredirhost_claim(host) == usb_redir_success) {
        status.status = usb_redir_success;
    }
exit:
    usbredirparser_send_reset_status(host->parser, id, &status);
}

static void usbredirhost_set_configuration(void *priv, uint32_t id,
    struct usb_redir_set_configuration_header *set_config)
{
    struct usbredirhost *host = priv;
    int i, r;
    struct usb_redir_configuration_status_header status = {
        .status = usb_redir_success,
    };

    if (set_config->configuration == host->active_config) {
        goto exit;
    }

    for (i = 0; i < MAX_ENDPOINTS; i++) {
        if (host->endpoint[i].type == LIBUSB_TRANSFER_TYPE_ISOCHRONOUS) {
            usbredirhost_cancel_iso_stream(host, I2EP(i), 1);
        }
    }
    usbredirhost_cancel_pending_urbs(host);

    status.status = usbredirhost_release(host);
    if (status.status != usb_redir_success) {
        goto exit;
    }

    r = libusb_set_configuration(host->handle, set_config->configuration);
    if (r < 0) {
        ERROR("could not set active configuration to %d: %d",
              (int)set_config->configuration, r);
        status.status = usb_redir_ioerror;
        goto exit;
    }

    host->active_config = set_config->configuration;
    status.status = usbredirhost_claim(host);

exit:
    status.configuration = host->active_config;
    usbredirparser_send_configuration_status(host->parser, id, &status);
}

static void usbredirhost_get_configuration(void *priv, uint32_t id)
{
    struct usbredirhost *host = priv;
    struct usb_redir_configuration_status_header status;

    status.status = usb_redir_success;
    status.configuration = host->active_config;
    usbredirparser_send_configuration_status(host->parser, id, &status);
}

static void usbredirhost_set_alt_setting(void *priv, uint32_t id,
    struct usb_redir_set_alt_setting_header *set_alt_setting)
{
    struct usbredirhost *host = priv;
    int i, j, r;
    const struct libusb_interface_descriptor *intf_desc;
    struct usb_redir_alt_setting_status_header status = {
        .status = usb_redir_success,
    };

    i = usbredirhost_bInterfaceNumber_to_index(host,
                                               set_alt_setting->interface);
    if (i == -1) {
        status.status = usb_redir_inval;
        status.alt = -1;
        goto exit_unknown_interface;
    }

    intf_desc = &host->config->interface[i].altsetting[host->alt_setting[i]];
    for (j = 0; j < intf_desc->bNumEndpoints; j++) {
        uint8_t ep = intf_desc->endpoint[j].bEndpointAddress;
        if (host->endpoint[EP2I(ep)].type == LIBUSB_TRANSFER_TYPE_ISOCHRONOUS) {
            usbredirhost_cancel_iso_stream(host, ep, 1);
        } else {
            usbredirhost_cancel_pending_urbs_on_ep(host, ep);
        }
    }

    r = libusb_set_interface_alt_setting(host->handle,
                                         set_alt_setting->interface,
                                         set_alt_setting->alt);
    if (r < 0) {
        ERROR("could not set alt setting for interface %d to %d: %d",
              (int)set_alt_setting->interface, (int)set_alt_setting->alt, r);
        status.status = libusb_status_or_error_to_redir_status(r);
        goto exit;
    }
    host->alt_setting[i] = set_alt_setting->alt;
    usbredirhost_parse_config(host);

exit:
    status.alt = host->alt_setting[i];
exit_unknown_interface:
    status.interface = set_alt_setting->interface;
    usbredirparser_send_alt_setting_status(host->parser, id, &status);
}

static void usbredirhost_get_alt_setting(void *priv, uint32_t id,
    struct usb_redir_get_alt_setting_header *get_alt_setting)
{
    struct usbredirhost *host = priv;
    struct usb_redir_alt_setting_status_header status;
    int i;

    i = usbredirhost_bInterfaceNumber_to_index(host,
                                               get_alt_setting->interface);
    if (i >= 0) {
        status.status = usb_redir_success;
        status.alt = host->alt_setting[i];
    } else {
        status.status = usb_redir_inval;
        status.alt = -1;
    }

    status.interface = get_alt_setting->interface;
    usbredirparser_send_alt_setting_status(host->parser, id, &status);
}

static void usbredirhost_start_iso_stream(void *priv, uint32_t id,
    struct usb_redir_start_iso_stream_header *start_iso_stream)
{
    struct usbredirhost *host = priv;
    int i, status;
    uint8_t ep = start_iso_stream->endpoint;

    status = usbredirhost_alloc_iso_stream(host, ep,
                   start_iso_stream->pkts_per_urb, start_iso_stream->no_urbs);
    if (status != usb_redir_success) {
        usbredirhost_send_iso_status(host, id, ep, status);
        return;
    }

    /* For input endpoints submit the transfers now */
    if (start_iso_stream->endpoint & LIBUSB_ENDPOINT_IN) {
        for (i = 0; i < host->endpoint[EP2I(ep)].iso_transfer_count; i++) {
            host->endpoint[EP2I(ep)].iso_transfer[i]->id =
                i * host->endpoint[EP2I(ep)].iso_pkts_per_transfer;
            status = usbredirhost_submit_iso_transfer(host,
                         host->endpoint[EP2I(ep)].iso_transfer[i]);
            if (status != usb_redir_success) {
                usbredirhost_send_iso_status(host, id, ep, status);
                return;
            }
        }
        host->endpoint[EP2I(ep)].iso_started = 1;
    }
    usbredirhost_send_iso_status(host, id, ep, usb_redir_success);
}

static void usbredirhost_stop_iso_stream(void *priv, uint32_t id,
    struct usb_redir_stop_iso_stream_header *stop_iso_stream)
{
    struct usbredirhost *host = priv;
    uint8_t ep = stop_iso_stream->endpoint;

    usbredirhost_cancel_iso_stream(host, ep, 1);
    usbredirhost_send_iso_status(host, id, ep, usb_redir_success);
}

static void usbredirhost_alloc_bulk_streams(void *priv, uint32_t id,
    struct usb_redir_alloc_bulk_streams_header *alloc_bulk_streams)
{
    struct usbredirhost *host = priv;
}

static void usbredirhost_free_bulk_streams(void *priv, uint32_t id,
    struct usb_redir_free_bulk_streams_header *free_bulk_streams)
{
    struct usbredirhost *host = priv;
}

static void usbredirhost_cancel_data_packet(void *priv, uint32_t id)
{
    struct usbredirhost *host = priv;
    struct usbredirtransfer *transfer;

    transfer = usbredirhost_find_transfer_by_id(host, id);
    if (!transfer) {
        /* This is not an error, the transfer may have completed by the time
           we receive the cancel */
        return;
    }
    libusb_cancel_transfer(transfer->transfer);
}

static void usbredirhost_control_packet_complete(
    struct libusb_transfer *libusb_transfer)
{
    struct usb_redir_control_packet_header control_header;
    struct usbredirtransfer *transfer = libusb_transfer->user_data;
    struct usbredirhost *host = transfer->host;

    control_header = transfer->control_header;
    control_header.status = libusb_status_or_error_to_redir_status(
                                                  libusb_transfer->status);
    control_header.length = libusb_transfer->actual_length;

    if (control_header.endpoint & LIBUSB_ENDPOINT_IN) {
        usbredirparser_send_control_packet(host->parser, transfer->id,
                                           &control_header,
                                           libusb_transfer->buffer +
                                               LIBUSB_CONTROL_SETUP_SIZE,
                                           libusb_transfer->actual_length);
    } else {
        usbredirparser_send_control_packet(host->parser, transfer->id,
                                           &control_header, NULL, 0);
    }

    usbredirhost_remove_and_free_transfer(transfer);
}

static void usbredirhost_control_packet(void *priv, uint32_t id,
    struct usb_redir_control_packet_header *control_header,
    uint8_t *data, int data_len)
{
    struct usbredirhost *host = priv;
    struct usbredirtransfer *transfer;
    unsigned char *buffer;
    int r;

    /* Verify data_len */
    if (control_header->endpoint & LIBUSB_ENDPOINT_IN) {
        if (data || data_len) {
            ERROR("data len non zero for control input packet");
            control_header->status = usb_redir_inval;
            control_header->length = 0;
            usbredirparser_send_control_packet(host->parser, id, 
                                               control_header, NULL, 0);
            return;
        }
    } else {
        if (data_len != control_header->length) {
            ERROR("data len: %d != header len: %d for control packet",
                  data_len, control_header->length);
            control_header->status = usb_redir_inval;
            control_header->length = 0;
            usbredirparser_send_control_packet(host->parser, id,
                                               control_header, NULL, 0);
            return;
        }
    }

    buffer = malloc(LIBUSB_CONTROL_SETUP_SIZE + control_header->length);
    if (!buffer) {
        ERROR("out of memory allocating transfer buffer, dropping packet");
        return;
    }

    transfer = usbredirhost_alloc_transfer(host, 0);
    if (!transfer) {
        free(buffer);
        return;
    }

    libusb_fill_control_setup(buffer,
                              control_header->requesttype,
                              control_header->request,
                              control_header->value,
                              control_header->index,
                              control_header->length);

    if (!(control_header->endpoint & LIBUSB_ENDPOINT_IN)) {
        memcpy(buffer + LIBUSB_CONTROL_SETUP_SIZE, data, data_len);
        free(data);
    }

    libusb_fill_control_transfer(transfer->transfer, host->handle, buffer,
                                 usbredirhost_control_packet_complete,
                                 transfer, CTRL_TIMEOUT);
    transfer->id = id;
    transfer->control_header = *control_header;

    usbredirhost_add_transfer(host, transfer);

    r = libusb_submit_transfer(transfer->transfer);
    if (r < 0) {
        ERROR("submitting control transfer on ep %02X: %d",
              (unsigned int)control_header->endpoint, r);
        transfer->transfer->actual_length = 0;
        transfer->transfer->status = r;
        usbredirhost_control_packet_complete(transfer->transfer);
    }
}

static void usbredirhost_bulk_packet(void *priv, uint32_t id,
    struct usb_redir_bulk_packet_header *bulk_header,
    uint8_t *data, int data_len)
{
    struct usbredirhost *host = priv;
}

static void usbredirhost_iso_packet(void *priv, uint32_t id,
    struct usb_redir_iso_packet_header *iso_header,
    uint8_t *data, int data_len)
{
    struct usbredirhost *host = priv;
    uint8_t ep = iso_header->endpoint;
    struct usbredirtransfer *transfer;
    int i, j, status;

    if (host->endpoint[EP2I(ep)].type != LIBUSB_TRANSFER_TYPE_ISOCHRONOUS) {
        ERROR("received iso packet for non iso endpoint");
        usbredirhost_send_iso_status(host, id, ep, usb_redir_inval);
        return;
    }

    if (ep & LIBUSB_ENDPOINT_IN) {
        ERROR("received iso packet for an iso input endpoint");
        usbredirhost_send_iso_status(host, id, ep, usb_redir_inval);
        return;
    }

    if (host->endpoint[EP2I(ep)].iso_transfer_count == 0) {
        ERROR("received iso out packet for non started iso stream");
        usbredirhost_send_iso_status(host, id, ep, usb_redir_inval);
        return;
    }

    if (data_len > host->endpoint[EP2I(ep)].max_packetsize) {
        ERROR("received iso out packet is larger than wMaxPacketSize");
        usbredirhost_send_iso_status(host, id, ep, usb_redir_inval);
        return;
    }

    i = host->endpoint[EP2I(ep)].iso_out_idx;
    transfer = host->endpoint[EP2I(ep)].iso_transfer[i];
    j = transfer->iso_packet_idx;
    if (j == ISO_SUBMITTED_IDX) {
        WARNING("overflow of iso out queue on ep: %02X, dropping packet",
                (unsigned int)ep);
        return;
    }

    /* Store the id of the first packet in the urb */
    if (j == 0) {
        transfer->id = id;
    }
    memcpy(libusb_get_iso_packet_buffer(transfer->transfer, j),
           data, data_len);
    transfer->transfer->iso_packet_desc[j].length = data_len;

    j++;
    transfer->iso_packet_idx = j;
    if (j == host->endpoint[EP2I(ep)].iso_pkts_per_transfer) {
        i = (i + 1) % host->endpoint[EP2I(ep)].iso_transfer_count;
        host->endpoint[EP2I(ep)].iso_out_idx = i;
        j = 0;
    }

    if (host->endpoint[EP2I(ep)].iso_started) {
        if (transfer->iso_packet_idx ==
                host->endpoint[EP2I(ep)].iso_pkts_per_transfer) {
            status = usbredirhost_submit_iso_transfer(host, transfer);
            if (status != usb_redir_success) {
                usbredirhost_send_iso_status(host, id, ep, status);
                return;
            }
        }
    } else {
        /* We've not started the stream (submitted some transfers) yet,
           do so once we have half our buffers filled */
        int available = i * host->endpoint[EP2I(ep)].iso_pkts_per_transfer + j;
        int needed = (host->endpoint[EP2I(ep)].iso_pkts_per_transfer *
                      host->endpoint[EP2I(ep)].iso_transfer_count) / 2;
        if (available == needed) {
            for (i = 0; i < host->endpoint[EP2I(ep)].iso_transfer_count / 2;
                    i++) {
                status = usbredirhost_submit_iso_transfer(host,
                                    host->endpoint[EP2I(ep)].iso_transfer[i]);
                if (status != usb_redir_success) {
                    usbredirhost_send_iso_status(host, id, ep, status);
                    return;
                }
            }
            host->endpoint[EP2I(ep)].iso_started = 1;
        }
    }
}
