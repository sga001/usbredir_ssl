/* usbredirparser.c usb redirection protocol parser

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
#include "usbredirparser.h"

struct usbredirparser_buf {
    uint8_t *buf;
    int pos;
    int len;

    struct usbredirparser_buf *next;
};

struct usbredirparser {
    usbredirparser_log log_func;
    usbredirparser_read read_func;
    usbredirparser_write write_func;
    usbredirparser_report_ep_types report_ep_types_func;
    usbredirparser_reset reset_func;
    usbredirparser_reset_status reset_status_func;
    usbredirparser_set_configuration set_configuration_func;
    usbredirparser_get_configuration get_configuration_func;
    usbredirparser_configuration_status configuration_status_func;
    usbredirparser_set_alt_setting set_alt_setting_func;
    usbredirparser_get_alt_setting get_alt_setting_func;
    usbredirparser_alt_setting_status alt_setting_status_func;
    usbredirparser_start_iso_stream start_iso_stream_func;
    usbredirparser_stop_iso_stream stop_iso_stream_func;
    usbredirparser_iso_stream_status iso_stream_status_func;
    usbredirparser_alloc_bulk_streams alloc_bulk_streams_func;
    usbredirparser_free_bulk_streams free_bulk_streams_func;
    usbredirparser_bulk_streams_status bulk_streams_status_func;
    usbredirparser_cancel_data_packet cancel_data_packet_func;
    usbredirparser_control_packet control_packet_func;
    usbredirparser_bulk_packet bulk_packet_func;
    usbredirparser_iso_packet iso_packet_func;
    void *func_priv;
    int flags;

    uint32_t our_caps[USB_REDIR_CAPS_SIZE];
    uint32_t peer_caps[USB_REDIR_CAPS_SIZE];

    struct usb_redir_header header;
    uint8_t type_header[256];
    int header_read;
    int type_header_len;
    int type_header_read;
    uint8_t *data;
    int data_len;
    int data_read;
    int to_skip;
    struct usbredirparser_buf *write_buf;
};

static void va_log(struct usbredirparser *parser, int verbose,
    const char *fmt, ...)
{
    char buf[512];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    
    parser->log_func(parser->func_priv, verbose, buf);
}

#define ERROR(...)   va_log(parser, usbredirparser_error, \
                            "usbredirparser error: " __VA_ARGS__)
#define WARNING(...) va_log(parser, usbredirparser_warning, \
                            "usbredirparser warning: " __VA_ARGS__)
#define INFO(...)    va_log(parser, usbredirparser_info, \
                            "usbredirparser info: " __VA_ARGS__)

static void usbredirparser_queue(struct usbredirparser *parser, uint32_t type,
    uint32_t id, void *type_header_in, uint8_t *data_in, int data_len);

struct usbredirparser *usbredirparser_create(
    usbredirparser_log log_func,
    usbredirparser_read read_func,
    usbredirparser_write write_func,
    usbredirparser_report_ep_types report_ep_types_func,
    usbredirparser_reset reset_func,
    usbredirparser_reset_status reset_status_func,
    usbredirparser_set_configuration set_configuration_func,
    usbredirparser_get_configuration get_configuration_func,
    usbredirparser_configuration_status configuration_status_func,
    usbredirparser_set_alt_setting set_alt_setting_func,
    usbredirparser_get_alt_setting get_alt_setting_func,
    usbredirparser_alt_setting_status alt_setting_status_func,
    usbredirparser_start_iso_stream start_iso_stream_func,
    usbredirparser_stop_iso_stream stop_iso_stream_func,
    usbredirparser_iso_stream_status iso_stream_status_func,
    usbredirparser_alloc_bulk_streams alloc_bulk_streams_func,
    usbredirparser_free_bulk_streams free_bulk_streams_func,
    usbredirparser_bulk_streams_status bulk_streams_status_func,
    usbredirparser_cancel_data_packet cancel_data_packet_func,
    usbredirparser_control_packet control_packet_func,
    usbredirparser_bulk_packet bulk_packet_func,
    usbredirparser_iso_packet iso_packet_func,
    void *func_priv,
    const char *version, uint32_t *caps, int caps_len, int flags)
{
    struct usbredirparser *parser;
    struct usb_redir_hello_header hello;

    parser = calloc(1, sizeof(*parser));
    if (!parser) {
        log_func(func_priv, usbredirparser_error,
            "usbredirparser error: Out of memory allocating usbredirparser");
        return NULL;
    }

    parser->log_func = log_func;
    parser->read_func = read_func;
    parser->write_func = write_func;
    parser->report_ep_types_func = report_ep_types_func;
    parser->reset_func = reset_func;
    parser->reset_status_func = reset_status_func;
    parser->set_configuration_func = set_configuration_func;
    parser->get_configuration_func = get_configuration_func;
    parser->configuration_status_func = configuration_status_func;
    parser->set_alt_setting_func = set_alt_setting_func;
    parser->get_alt_setting_func = get_alt_setting_func;
    parser->alt_setting_status_func = alt_setting_status_func;
    parser->start_iso_stream_func = start_iso_stream_func;
    parser->stop_iso_stream_func = stop_iso_stream_func;
    parser->iso_stream_status_func = iso_stream_status_func;
    parser->alloc_bulk_streams_func = alloc_bulk_streams_func;
    parser->free_bulk_streams_func = free_bulk_streams_func;
    parser->bulk_streams_status_func = bulk_streams_status_func;
    parser->cancel_data_packet_func = cancel_data_packet_func;
    parser->control_packet_func = control_packet_func;
    parser->bulk_packet_func = bulk_packet_func;
    parser->iso_packet_func = iso_packet_func;
    parser->func_priv = func_priv;
    parser->flags = flags;

    snprintf(hello.version, sizeof(hello.version), "%s", version);
    if (caps_len > USB_REDIR_CAPS_SIZE) {
        caps_len = USB_REDIR_CAPS_SIZE;
    }
    memcpy(parser->our_caps, caps, caps_len * sizeof(uint32_t));
    usbredirparser_queue(parser, usb_redir_hello, 0, &hello,
                         (uint8_t *)parser->our_caps,
                         USB_REDIR_CAPS_SIZE * sizeof(uint32_t));

    return parser;
}

void usbredirparser_destroy(struct usbredirparser *parser)
{
    struct usbredirparser_buf *wbuf, *next_wbuf;

    wbuf = parser->write_buf;
    while (wbuf) {
        next_wbuf = wbuf->next;
        free(wbuf->buf);
        free(wbuf);
        wbuf = next_wbuf;
    }

    free(parser);
}

int usbredirparser_peer_has_cap(struct usbredirparser *parser, int cap)
{
    if (cap / 32 >= USB_REDIR_CAPS_SIZE) {
        ERROR("request for out of bounds cap: %d", cap);
        return 0;
    }
    if ((parser->peer_caps[cap / 32]) & (1 << (cap % 32))) {
        return 1;
    } else {
        return 0;
    }
}

static void usbredirparser_handle_hello(struct usbredirparser *parser,
    struct usb_redir_hello_header *hello, uint8_t *data, int data_len)
{
    int i;
    char buf[64];
    uint32_t *peer_caps = (uint32_t *)data;

    /* In case hello->version is not 0 terminated (which would be a protocol
       violation)_ */
    snprintf(buf, sizeof(buf), "%s", hello->version);
    INFO("Peer version: %s", buf);

    memset(parser->peer_caps, 0, sizeof(parser->peer_caps));
    if (data_len > sizeof(parser->peer_caps)) {
        data_len = sizeof(parser->peer_caps);
    }
    for (i = 0; i < data_len / sizeof(uint32_t); i++) {
        parser->peer_caps[i] = peer_caps[i];
    }
    free(data);
}

static int usbredirparser_get_type_header_len(struct usbredirparser *parser,
    int32_t type, int send)
{
    int command_for_host = 0;
    
    if (parser->flags & usbredirparser_fl_usb_host) {
        command_for_host = 1;
    }
    if (send) {
        command_for_host = !command_for_host;
    }

    switch (type) {
    case usb_redir_hello:
        return sizeof(struct usb_redir_hello_header);
    case usb_redir_report_ep_types:
        if (!command_for_host) {
            return sizeof(struct usb_redir_report_ep_types_header);
        } else {
            return -1;
        }
    case usb_redir_reset:
        if (command_for_host) {
            return 0; /* No packet type specific header */
        } else {
            return -1;
        }
    case usb_redir_reset_status:
        if (!command_for_host) {
            return sizeof(struct usb_redir_reset_status_header);
        } else {
            return -1;
        }
    case usb_redir_set_configuration:
        if (command_for_host) {
            return sizeof(struct usb_redir_set_configuration_header);
        } else {
            return -1; /* Should never be send to a guest */
        }
    case usb_redir_get_configuration:
        if (command_for_host) {
            return 0; /* No packet type specific header */
        } else {
            return -1;
        }
    case usb_redir_configuration_status:
        if (!command_for_host) {
            return sizeof(struct usb_redir_configuration_status_header);
        } else {
            return -1;
        }
    case usb_redir_set_alt_setting:
        if (command_for_host) {
            return sizeof(struct usb_redir_set_alt_setting_header);
        } else {
            return -1;
        }
    case usb_redir_get_alt_setting:
        if (command_for_host) {
            return sizeof(struct usb_redir_get_alt_setting_header);
        } else {
            return -1;
        }
    case usb_redir_alt_setting_status:
        if (!command_for_host) {
            return sizeof(struct usb_redir_alt_setting_status_header);
        } else {
            return -1;
        }
    case usb_redir_start_iso_stream:
        if (command_for_host) {
            return sizeof(struct usb_redir_start_iso_stream_header);
        } else {
            return -1;
        }
    case usb_redir_stop_iso_stream:
        if (command_for_host) {
            return sizeof(struct usb_redir_stop_iso_stream_header);
        } else {
            return -1;
        }
    case usb_redir_iso_stream_status:
        if (!command_for_host) {
            return sizeof(struct usb_redir_iso_stream_status_header);
        } else {
            return -1;
        }
    case usb_redir_alloc_bulk_streams:
        if (command_for_host) {
            return sizeof(struct usb_redir_alloc_bulk_streams_header);
        } else {
            return -1;
        }
    case usb_redir_free_bulk_streams:
        if (command_for_host) {
            return sizeof(struct usb_redir_free_bulk_streams_header);
        } else {
            return -1;
        }
    case usb_redir_bulk_streams_status:
        if (!command_for_host) {
            return sizeof(struct usb_redir_bulk_streams_status_header);
        } else {
            return -1;
        }
    case usb_redir_cancel_data_packet:
        if (command_for_host) {
            return 0; /* No packet type specific header */
        } else {
            return -1;
        }
    case usb_redir_control_packet:
        return sizeof(struct usb_redir_control_packet_header);
    case usb_redir_bulk_packet:
        return sizeof(struct usb_redir_bulk_packet_header);
    case usb_redir_iso_packet:
        return sizeof(struct usb_redir_iso_packet_header);
    default:
        return -1;
    }
}

static int usbredirparser_expect_extra_data(struct usbredirparser *parser)
{
    switch (parser->header.type) {
    case usb_redir_hello: /* For the variable length capabilities array */
    case usb_redir_control_packet:
    case usb_redir_bulk_packet:
    case usb_redir_iso_packet:
        return 1;
    default:
        return 0;
    }
}

static void usbredirparser_call_type_func(struct usbredirparser *parser)
{
    switch (parser->header.type) {
    case usb_redir_hello:
        usbredirparser_handle_hello(parser,
            (struct usb_redir_hello_header *)parser->type_header,
            parser->data, parser->data_len);
        break;
    case usb_redir_report_ep_types:
        parser->report_ep_types_func(parser->func_priv,
            (struct usb_redir_report_ep_types_header *)parser->type_header);
        break;
    case usb_redir_reset:
        parser->reset_func(parser->func_priv, parser->header.id);
        break;
    case usb_redir_reset_status:
        parser->reset_status_func(parser->func_priv, parser->header.id,
            (struct usb_redir_reset_status_header *)parser->type_header);
        break;
    case usb_redir_set_configuration:
        parser->set_configuration_func(parser->func_priv, parser->header.id,
            (struct usb_redir_set_configuration_header *)parser->type_header);
        break;
    case usb_redir_get_configuration:
        parser->get_configuration_func(parser->func_priv, parser->header.id);
        break;
    case usb_redir_configuration_status:
        parser->configuration_status_func(parser->func_priv, parser->header.id,
            (struct usb_redir_configuration_status_header *)parser->type_header);
        break;
    case usb_redir_set_alt_setting:
        parser->set_alt_setting_func(parser->func_priv, parser->header.id,
            (struct usb_redir_set_alt_setting_header *)parser->type_header);
        break;
    case usb_redir_get_alt_setting:
        parser->get_alt_setting_func(parser->func_priv, parser->header.id,
            (struct usb_redir_get_alt_setting_header *)parser->type_header);
        break;
    case usb_redir_alt_setting_status:
        parser->alt_setting_status_func(parser->func_priv, parser->header.id,
            (struct usb_redir_alt_setting_status_header *)parser->type_header);
        break;
    case usb_redir_start_iso_stream:
        parser->start_iso_stream_func(parser->func_priv, parser->header.id,
            (struct usb_redir_start_iso_stream_header *)parser->type_header);
        break;
    case usb_redir_stop_iso_stream:
        parser->stop_iso_stream_func(parser->func_priv, parser->header.id,
            (struct usb_redir_stop_iso_stream_header *)parser->type_header);
        break;
    case usb_redir_iso_stream_status:
        parser->iso_stream_status_func(parser->func_priv, parser->header.id,
            (struct usb_redir_iso_stream_status_header *)parser->type_header);
        break;
    case usb_redir_alloc_bulk_streams:
        parser->alloc_bulk_streams_func(parser->func_priv, parser->header.id,
            (struct usb_redir_alloc_bulk_streams_header *)parser->type_header);
        break;
    case usb_redir_free_bulk_streams:
        parser->free_bulk_streams_func(parser->func_priv, parser->header.id,
            (struct usb_redir_free_bulk_streams_header *)parser->type_header);
        break;
    case usb_redir_bulk_streams_status:
        parser->bulk_streams_status_func(parser->func_priv, parser->header.id,
            (struct usb_redir_bulk_streams_status_header *)parser->type_header);
        break;
    case usb_redir_cancel_data_packet:
        parser->cancel_data_packet_func(parser->func_priv, parser->header.id);
        break;
    case usb_redir_control_packet:
        parser->control_packet_func(parser->func_priv, parser->header.id,
            (struct usb_redir_control_packet_header *)parser->type_header,
            parser->data, parser->data_len);
        break;
    case usb_redir_bulk_packet:
        parser->bulk_packet_func(parser->func_priv, parser->header.id,
            (struct usb_redir_bulk_packet_header *)parser->type_header,
            parser->data, parser->data_len);
        break;
    case usb_redir_iso_packet:
        parser->iso_packet_func(parser->func_priv, parser->header.id,
            (struct usb_redir_iso_packet_header *)parser->type_header,
            parser->data, parser->data_len);
        break;
    }
}

int usbredirparser_do_read(struct usbredirparser *parser)
{
    int r, type_header_len, data_len;
    uint8_t *dest;

    /* Skip forward to next packet (only used in error conditions) */
    while (parser->to_skip > 0) {
        uint8_t buf[65536];
        r = (parser->to_skip > sizeof(buf)) ? sizeof(buf) : parser->to_skip;
        r = parser->read_func(parser->func_priv, buf, r);
        if (r <= 0)
            return r;
        parser->to_skip -= r;
        if (parser->to_skip == 0)
            parser->header_read = 0;
    }

    /* Consume data until read would block or returns an error */
    while (1) {
        if (parser->header_read < sizeof(parser->header)) {
            r = sizeof(parser->header) - parser->header_read;
            dest = (uint8_t *)&parser->header + parser->header_read;
        } else if (parser->type_header_read < parser->type_header_len) {
            r = parser->type_header_len - parser->type_header_read;
            dest = parser->type_header + parser->type_header_read;
        } else {
            r = parser->data_len - parser->data_read;
            dest = parser->data + parser->data_read;
        }

        if (r > 0) {
            r = parser->read_func(parser->func_priv, dest, r);
            if (r <= 0) {
                return r;
            }
        }

        if (parser->header_read < sizeof(parser->header)) {
            parser->header_read += r;
            if (parser->header_read == sizeof(parser->header)) {
                type_header_len =
                    usbredirparser_get_type_header_len(parser,
                                                       parser->header.type, 0);
                if (type_header_len < 0) {
                    ERROR("invalid usb-redir packet type: %u",
                          parser->header.type);
                    parser->to_skip = parser->header.length;
                    return -2;
                }
                /* This should never happen */
                if (type_header_len > sizeof(parser->type_header)) {
                    ERROR("type specific header buffer too small, please report!!");
                    parser->to_skip = parser->header.length;
                    return -2;
                }
                if (parser->header.length < type_header_len ||
                    (parser->header.length > type_header_len &&
                     !usbredirparser_expect_extra_data(parser))) {
                    ERROR("invalid packet length: %u", parser->header.length);
                    parser->to_skip = parser->header.length;
                    return -2;
                }
                data_len = parser->header.length - type_header_len;
                if (data_len) {
                    parser->data = malloc(data_len);
                    if (!parser->data) {
                        ERROR("Out of memory allocating data buffer");
                        parser->to_skip = parser->header.length;
                        return -2;
                    }
                }
                parser->type_header_len = type_header_len;
                parser->data_len = data_len;
            }
        } else if (parser->type_header_read < parser->type_header_len) {
            parser->type_header_read += r;
        } else {
            parser->data_read += r;
            if (parser->data_read == parser->data_len) {
                usbredirparser_call_type_func(parser);
                parser->header_read = 0;
                parser->type_header_len  = 0;
                parser->type_header_read = 0;
                parser->data_len  = 0;
                parser->data_read = 0;
                parser->data = NULL;
            }
        }
    }
}

int usbredirparser_has_data_to_write(struct usbredirparser *parser)
{
    return parser->write_buf != NULL;
}

int usbredirparser_do_write(struct usbredirparser *parser)
{
    int w;
    struct usbredirparser_buf* wbuf;

    for (;;) {    
        wbuf = parser->write_buf;
        if (!wbuf)
            return 0;

        w = wbuf->len - wbuf->pos;
        w = parser->write_func(parser->func_priv, wbuf->buf + wbuf->pos, w);
        if (w <= 0)
            return w;

        wbuf->pos += w;
        if (wbuf->pos == wbuf->len) {
            parser->write_buf = wbuf->next;
            free(wbuf->buf);
            free(wbuf);
        }
    }
}

static void usbredirparser_queue(struct usbredirparser *parser, uint32_t type,
    uint32_t id, void *type_header_in, uint8_t *data_in, int data_len)
{
    uint8_t *buf, *type_header_out, *data_out;
    struct usb_redir_header *header;
    struct usbredirparser_buf *wbuf, *new_wbuf;
    int type_header_len;

    type_header_len = usbredirparser_get_type_header_len(parser, type, 1);
    if (type_header_len < 0) { /* This should never happen */
        ERROR("packet type unknown with internal call, please report!!");
        return;
    }

    new_wbuf = calloc(1, sizeof(*new_wbuf));
    buf = malloc(sizeof(*header) + type_header_len + data_len);
    if (!new_wbuf || !buf) {
        ERROR("Out of memory allocating buffer to send packet, dropping!");
        free(new_wbuf); free(buf);
        return;
    }

    new_wbuf->buf = buf;
    new_wbuf->len = sizeof(*header) + type_header_len + data_len;

    header = (struct usb_redir_header *)buf;
    type_header_out = buf + sizeof(*header);
    data_out = type_header_out + type_header_len;

    header->type   = type;
    header->id     = id;
    header->length = type_header_len + data_len;
    memcpy(type_header_out, type_header_in, type_header_len);
    memcpy(data_out, data_in, data_len);

    if (!parser->write_buf) {
        parser->write_buf = new_wbuf;
        return;
    }

    /* maybe we should limit the write_buf stack depth ? */
    wbuf = parser->write_buf;
    while (wbuf->next)
        wbuf = wbuf->next;

    wbuf->next = new_wbuf;
}

void usbredirparser_send_report_ep_types(struct usbredirparser *parser,
    struct usb_redir_report_ep_types_header *report_ep_types)
{
    usbredirparser_queue(parser, usb_redir_report_ep_types, 0,
                         report_ep_types, NULL, 0);
}

void usbredirparser_send_reset(struct usbredirparser *parser, uint32_t id)
{
    usbredirparser_queue(parser, usb_redir_reset, id, NULL, NULL, 0);
}

void usbredirparser_send_reset_status(struct usbredirparser *parser,
    uint32_t id, struct usb_redir_reset_status_header *reset_status)
{
    usbredirparser_queue(parser, usb_redir_reset_status, id,
                         reset_status, NULL, 0);
}

void usbredirparser_send_set_configuration(struct usbredirparser *parser,
    uint32_t id,
    struct usb_redir_set_configuration_header *set_configuration)
{
    usbredirparser_queue(parser, usb_redir_set_configuration, id,
                         set_configuration, NULL, 0);
}

void usbredirparser_send_get_configuration(struct usbredirparser *parser,
    uint32_t id)
{
    usbredirparser_queue(parser, usb_redir_get_configuration, id,
                         NULL, NULL, 0);
}

void usbredirparser_send_configuration_status(struct usbredirparser *parser,
    uint32_t id,
    struct usb_redir_configuration_status_header *configuration_status)
{
    usbredirparser_queue(parser, usb_redir_configuration_status, id,
                         configuration_status, NULL, 0);
}

void usbredirparser_send_set_alt_setting(struct usbredirparser *parser,
    uint32_t id,
    struct usb_redir_set_alt_setting_header *set_alt_setting)
{
    usbredirparser_queue(parser, usb_redir_set_alt_setting, id,
                         set_alt_setting, NULL, 0);
}

void usbredirparser_send_get_alt_setting(struct usbredirparser *parser,
    uint32_t id,
    struct usb_redir_get_alt_setting_header *get_alt_setting)
{
    usbredirparser_queue(parser, usb_redir_get_alt_setting, id,
                         get_alt_setting, NULL, 0);
}

void usbredirparser_send_alt_setting_status(struct usbredirparser *parser,
    uint32_t id,
    struct usb_redir_alt_setting_status_header *alt_setting_status)
{
    usbredirparser_queue(parser, usb_redir_alt_setting_status, id,
                         alt_setting_status, NULL, 0);
}

void usbredirparser_send_start_iso_stream(struct usbredirparser *parser,
    uint32_t id,
    struct usb_redir_start_iso_stream_header *start_iso_stream)
{
    usbredirparser_queue(parser, usb_redir_start_iso_stream, id,
                         start_iso_stream, NULL, 0);
}

void usbredirparser_send_stop_iso_stream(struct usbredirparser *parser,
    uint32_t id,
    struct usb_redir_stop_iso_stream_header *stop_iso_stream)
{
    usbredirparser_queue(parser, usb_redir_stop_iso_stream, id,
                         stop_iso_stream, NULL, 0);
}

void usbredirparser_send_iso_stream_status(struct usbredirparser *parser,
    uint32_t id,
    struct usb_redir_iso_stream_status_header *iso_stream_status)
{
    usbredirparser_queue(parser, usb_redir_iso_stream_status, id,
                         iso_stream_status, NULL, 0);
}

void usbredirparser_send_alloc_bulk_streams(struct usbredirparser *parser,
    uint32_t id,
    struct usb_redir_alloc_bulk_streams_header *alloc_bulk_streams)
{
    usbredirparser_queue(parser, usb_redir_alloc_bulk_streams, id,
                         alloc_bulk_streams, NULL, 0);
}

void usbredirparser_send_free_bulk_streams(struct usbredirparser *parser,
    uint32_t id,
    struct usb_redir_free_bulk_streams_header *free_bulk_streams)
{
    usbredirparser_queue(parser, usb_redir_free_bulk_streams, id,
                         free_bulk_streams, NULL, 0);
}

void usbredirparser_send_bulk_streams_status(struct usbredirparser *parser,
    uint32_t id,
    struct usb_redir_bulk_streams_status_header *bulk_streams_status)
{
    usbredirparser_queue(parser, usb_redir_bulk_streams_status, id,
                         bulk_streams_status, NULL, 0);
}

void usbredirparser_send_cancel_data_packet(struct usbredirparser *parser,
    uint32_t id)
{
    usbredirparser_queue(parser, usb_redir_cancel_data_packet, id,
                         NULL, NULL, 0);
}

/* Data packets: */
void usbredirparser_send_control_packet(struct usbredirparser *parser,
    uint32_t id,
    struct usb_redir_control_packet_header *control_header,
    uint8_t *data, int data_len)
{
    usbredirparser_queue(parser, usb_redir_control_packet, id, control_header,
                         data, data_len);
}

void usbredirparser_send_bulk_packet(struct usbredirparser *parser,
    uint32_t id,
    struct usb_redir_bulk_packet_header *bulk_header,
    uint8_t *data, int data_len)
{
    usbredirparser_queue(parser, usb_redir_bulk_packet, id, bulk_header,
                         data, data_len);
}

void usbredirparser_send_iso_packet(struct usbredirparser *parser,
    uint32_t id,
    struct usb_redir_iso_packet_header *iso_header,
    uint8_t *data, int data_len)
{
    usbredirparser_queue(parser, usb_redir_iso_packet, id, iso_header,
                         data, data_len);
}