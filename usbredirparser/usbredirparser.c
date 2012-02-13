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
#include "config.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "usbredirproto-compat.h"
#include "usbredirparser.h"
#include "usbredirfilter.h"

/* Locking convenience macros */
#define LOCK(parser) \
    do { \
        if ((parser)->lock) \
            (parser)->callb.lock_func((parser)->lock); \
    } while (0)

#define UNLOCK(parser) \
    do { \
        if ((parser)->lock) \
            (parser)->callb.unlock_func((parser)->lock); \
    } while (0)

struct usbredirparser_buf {
    uint8_t *buf;
    int pos;
    int len;

    struct usbredirparser_buf *next;
};

struct usbredirparser_priv {
    struct usbredirparser callb;
    int flags;

    uint32_t our_caps[USB_REDIR_CAPS_SIZE];
    uint32_t peer_caps[USB_REDIR_CAPS_SIZE];

    void *lock;

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

static void va_log(struct usbredirparser_priv *parser, int verbose,
    const char *fmt, ...)
{
    char buf[512];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    
    parser->callb.log_func(parser->callb.priv, verbose, buf);
}

#define ERROR(...)   va_log(parser, usbredirparser_error, \
                            "usbredirparser error: " __VA_ARGS__)
#define WARNING(...) va_log(parser, usbredirparser_warning, \
                            "usbredirparser warning: " __VA_ARGS__)
#define INFO(...)    va_log(parser, usbredirparser_info, \
                            "usbredirparser info: " __VA_ARGS__)

static void usbredirparser_queue(struct usbredirparser *parser, uint32_t type,
    uint32_t id, void *type_header_in, uint8_t *data_in, int data_len);

struct usbredirparser *usbredirparser_create(void)
{
    return calloc(1, sizeof(struct usbredirparser_priv));
}

void usbredirparser_init(struct usbredirparser *parser_pub,
    const char *version, uint32_t *caps, int caps_len, int flags)
{
    struct usbredirparser_priv *parser =
        (struct usbredirparser_priv *)parser_pub;
    struct usb_redir_hello_header hello;

    parser->flags = flags;
    if (parser->callb.alloc_lock_func) {
        parser->lock = parser->callb.alloc_lock_func();
    }

    snprintf(hello.version, sizeof(hello.version), "%s", version);
    if (caps_len > USB_REDIR_CAPS_SIZE) {
        caps_len = USB_REDIR_CAPS_SIZE;
    }
    memcpy(parser->our_caps, caps, caps_len * sizeof(uint32_t));
    usbredirparser_queue(parser_pub, usb_redir_hello, 0, &hello,
                         (uint8_t *)parser->our_caps,
                         USB_REDIR_CAPS_SIZE * sizeof(uint32_t));
}

void usbredirparser_destroy(struct usbredirparser *parser_pub)
{
    struct usbredirparser_priv *parser =
        (struct usbredirparser_priv *)parser_pub;
    struct usbredirparser_buf *wbuf, *next_wbuf;

    wbuf = parser->write_buf;
    while (wbuf) {
        next_wbuf = wbuf->next;
        free(wbuf->buf);
        free(wbuf);
        wbuf = next_wbuf;
    }

    if (parser->lock)
        parser->callb.free_lock_func(parser->lock);

    free(parser);
}

void usbredirparser_caps_set_cap(uint32_t *caps, int cap)
{
    caps[cap / 32] |= 1 << (cap % 32);
}

int usbredirparser_peer_has_cap(struct usbredirparser *parser_pub, int cap)
{
    struct usbredirparser_priv *parser =
        (struct usbredirparser_priv *)parser_pub;
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

int usbredirparser_have_cap(struct usbredirparser *parser_pub, int cap)
{
    struct usbredirparser_priv *parser =
        (struct usbredirparser_priv *)parser_pub;
    if ((parser->our_caps[cap / 32]) & (1 << (cap % 32))) {
        return 1;
    } else {
        return 0;
    }
}

static void usbredirparser_handle_hello(struct usbredirparser_priv *parser,
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

    /* Added in 0.3.2, so no guarantee it is there */
    if (parser->callb.hello_func)
        parser->callb.hello_func(parser->callb.priv, hello);
}

static int usbredirparser_get_type_header_len(
    struct usbredirparser *parser_pub, int32_t type, int send)
{
    struct usbredirparser_priv *parser =
        (struct usbredirparser_priv *)parser_pub;
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
    case usb_redir_device_connect:
        if (!command_for_host) {
            if (usbredirparser_have_cap(parser_pub,
                                    usb_redir_cap_connect_device_version) &&
                usbredirparser_peer_has_cap(parser_pub,
                                    usb_redir_cap_connect_device_version)) {
                return sizeof(struct usb_redir_device_connect_header);
            } else {
                return sizeof(struct usb_redir_device_connect_header_no_device_version);
            }
        } else {
            return -1;
        }
    case usb_redir_device_disconnect:
        if (!command_for_host) {
            return 0;
        } else {
            return -1;
        }
    case usb_redir_reset:
        if (command_for_host) {
            return 0; /* No packet type specific header */
        } else {
            return -1;
        }
    case usb_redir_interface_info:
        if (!command_for_host) {
            return sizeof(struct usb_redir_interface_info_header);
        } else {
            return -1;
        }
    case usb_redir_ep_info:
        if (!command_for_host) {
            return sizeof(struct usb_redir_ep_info_header);
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
    case usb_redir_start_interrupt_receiving:
        if (command_for_host) {
            return sizeof(struct usb_redir_start_interrupt_receiving_header);
        } else {
            return -1;
        }
    case usb_redir_stop_interrupt_receiving:
        if (command_for_host) {
            return sizeof(struct usb_redir_stop_interrupt_receiving_header);
        } else {
            return -1;
        }
    case usb_redir_interrupt_receiving_status:
        if (!command_for_host) {
            return sizeof(struct usb_redir_interrupt_receiving_status_header);
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
    case usb_redir_filter_reject:
        if (command_for_host) {
            return 0;
        } else {
            return -1;
        }
    case usb_redir_filter_filter:
        return 0;
    case usb_redir_control_packet:
        return sizeof(struct usb_redir_control_packet_header);
    case usb_redir_bulk_packet:
        return sizeof(struct usb_redir_bulk_packet_header);
    case usb_redir_iso_packet:
        return sizeof(struct usb_redir_iso_packet_header);
    case usb_redir_interrupt_packet:
        return sizeof(struct usb_redir_interrupt_packet_header);
    default:
        return -1;
    }
}

/* Note this function only checks if extra data is allowed for the
   packet type being read at all, a check if it is actually allowed
   given the direction of the packet + ep is done in _erify_type_header */
static int usbredirparser_expect_extra_data(struct usbredirparser_priv *parser)
{
    switch (parser->header.type) {
    case usb_redir_hello: /* For the variable length capabilities array */
    case usb_redir_control_packet:
    case usb_redir_bulk_packet:
    case usb_redir_iso_packet:
    case usb_redir_interrupt_packet:
    case usb_redir_filter_filter:
        return 1;
    default:
        return 0;
    }
}

static int usbredirparser_verify_type_header(
    struct usbredirparser *parser_pub,
    int32_t type, void *header, uint8_t *data, int data_len, int send)
{
    struct usbredirparser_priv *parser =
        (struct usbredirparser_priv *)parser_pub;
    int command_for_host = 0, expect_extra_data = 0;
    int length = 0, ep = -1;

    if (parser->flags & usbredirparser_fl_usb_host) {
        command_for_host = 1;
    }
    if (send) {
        command_for_host = !command_for_host;
    }

    switch (type) {
    case usb_redir_interface_info: {
        struct usb_redir_interface_info_header *intf_info = header;

        if (intf_info->interface_count > 32) {
            ERROR("interface_count > 32");
            return 0;
        }
        break;
    }
    case usb_redir_filter_reject:
        if ((send && !usbredirparser_peer_has_cap(parser_pub,
                                             usb_redir_cap_filter)) ||
            (!send && !usbredirparser_have_cap(parser_pub,
                                             usb_redir_cap_filter))) {
            ERROR("filter_reject without cap_filter");
            return 0;
        }
        break;
    case usb_redir_filter_filter:
        if ((send && !usbredirparser_peer_has_cap(parser_pub,
                                             usb_redir_cap_filter)) ||
            (!send && !usbredirparser_have_cap(parser_pub,
                                             usb_redir_cap_filter))) {
            ERROR("filter_filter without cap_filter");
            return 0;
        }
        if (data_len < 1) {
            ERROR("filter_filter without data");
            return 0;
        }
        if (data[data_len - 1] != 0) {
            ERROR("non 0 terminated filter_filter data");
            return 0;
        }
        break;
    case usb_redir_control_packet:
        length = ((struct usb_redir_control_packet_header *)header)->length;
        ep = ((struct usb_redir_control_packet_header *)header)->endpoint;
        break;
    case usb_redir_bulk_packet:
        length = ((struct usb_redir_bulk_packet_header *)header)->length;
        ep = ((struct usb_redir_bulk_packet_header *)header)->endpoint;
        break;
    case usb_redir_iso_packet:
        length = ((struct usb_redir_iso_packet_header *)header)->length;
        ep = ((struct usb_redir_iso_packet_header *)header)->endpoint;
        break;
    case usb_redir_interrupt_packet:
        length = ((struct usb_redir_interrupt_packet_header *)header)->length;
        ep = ((struct usb_redir_interrupt_packet_header *)header)->endpoint;
        break;
    }

    if (ep != -1) {
        if (((ep & 0x80) && !command_for_host) ||
            (!(ep & 0x80) && command_for_host)) {
            expect_extra_data = 1;
        }
        if (expect_extra_data) {
            if (data_len != length) {
                ERROR("data len %d != header len %d ep %02X",
                      data_len, length, ep);
                return 0;
            }
        } else {
            if (data || data_len) {
                ERROR("unexpected extra data ep %02X", ep);
                return 0;
            }
            switch (type) {
            case usb_redir_iso_packet:
                ERROR("iso packet send in wrong direction");
                return 0;
            case usb_redir_interrupt_packet:
                if (command_for_host) {
                    ERROR("interrupt packet send in wrong direction");
                    return 0;
                }
                break;
            }
        }
    }

    return 1; /* Verify ok */
}

static void usbredirparser_call_type_func(struct usbredirparser_priv *parser)
{
    switch (parser->header.type) {
    case usb_redir_hello:
        usbredirparser_handle_hello(parser,
            (struct usb_redir_hello_header *)parser->type_header,
            parser->data, parser->data_len);
        break;
    case usb_redir_device_connect:
        parser->callb.device_connect_func(parser->callb.priv,
            (struct usb_redir_device_connect_header *)parser->type_header);
        break;
    case usb_redir_device_disconnect:
        parser->callb.device_disconnect_func(parser->callb.priv);
        break;
    case usb_redir_reset:
        parser->callb.reset_func(parser->callb.priv);
        break;
    case usb_redir_interface_info:
        parser->callb.interface_info_func(parser->callb.priv,
            (struct usb_redir_interface_info_header *)parser->type_header);
        break;
    case usb_redir_ep_info:
        parser->callb.ep_info_func(parser->callb.priv,
            (struct usb_redir_ep_info_header *)parser->type_header);
        break;
    case usb_redir_set_configuration:
        parser->callb.set_configuration_func(parser->callb.priv,
            parser->header.id,
            (struct usb_redir_set_configuration_header *)parser->type_header);
        break;
    case usb_redir_get_configuration:
        parser->callb.get_configuration_func(parser->callb.priv,
            parser->header.id);
        break;
    case usb_redir_configuration_status:
        parser->callb.configuration_status_func(parser->callb.priv,
            parser->header.id, (struct usb_redir_configuration_status_header *)
            parser->type_header);
        break;
    case usb_redir_set_alt_setting:
        parser->callb.set_alt_setting_func(parser->callb.priv,
            parser->header.id,
            (struct usb_redir_set_alt_setting_header *)parser->type_header);
        break;
    case usb_redir_get_alt_setting:
        parser->callb.get_alt_setting_func(parser->callb.priv,
            parser->header.id,
            (struct usb_redir_get_alt_setting_header *)parser->type_header);
        break;
    case usb_redir_alt_setting_status:
        parser->callb.alt_setting_status_func(parser->callb.priv,
            parser->header.id,
            (struct usb_redir_alt_setting_status_header *)parser->type_header);
        break;
    case usb_redir_start_iso_stream:
        parser->callb.start_iso_stream_func(parser->callb.priv,
            parser->header.id,
            (struct usb_redir_start_iso_stream_header *)parser->type_header);
        break;
    case usb_redir_stop_iso_stream:
        parser->callb.stop_iso_stream_func(parser->callb.priv,
            parser->header.id,
            (struct usb_redir_stop_iso_stream_header *)parser->type_header);
        break;
    case usb_redir_iso_stream_status:
        parser->callb.iso_stream_status_func(parser->callb.priv,
            parser->header.id,
            (struct usb_redir_iso_stream_status_header *)parser->type_header);
        break;
    case usb_redir_start_interrupt_receiving:
        parser->callb.start_interrupt_receiving_func(parser->callb.priv,
            parser->header.id,
            (struct usb_redir_start_interrupt_receiving_header *)
            parser->type_header);
        break;
    case usb_redir_stop_interrupt_receiving:
        parser->callb.stop_interrupt_receiving_func(parser->callb.priv,
            parser->header.id,
            (struct usb_redir_stop_interrupt_receiving_header *)
            parser->type_header);
        break;
    case usb_redir_interrupt_receiving_status:
        parser->callb.interrupt_receiving_status_func(parser->callb.priv,
            parser->header.id,
            (struct usb_redir_interrupt_receiving_status_header *)
            parser->type_header);
        break;
    case usb_redir_alloc_bulk_streams:
        parser->callb.alloc_bulk_streams_func(parser->callb.priv,
            parser->header.id,
            (struct usb_redir_alloc_bulk_streams_header *)parser->type_header);
        break;
    case usb_redir_free_bulk_streams:
        parser->callb.free_bulk_streams_func(parser->callb.priv,
            parser->header.id,
            (struct usb_redir_free_bulk_streams_header *)parser->type_header);
        break;
    case usb_redir_bulk_streams_status:
        parser->callb.bulk_streams_status_func(parser->callb.priv,
            parser->header.id, (struct usb_redir_bulk_streams_status_header *)
            parser->type_header);
        break;
    case usb_redir_cancel_data_packet:
        parser->callb.cancel_data_packet_func(parser->callb.priv,
            parser->header.id);
        break;
    case usb_redir_filter_reject:
        parser->callb.filter_reject_func(parser->callb.priv);
        break;
    case usb_redir_filter_filter: {
        struct usbredirfilter_rule *rules;
        int r, count;

        r = usbredirfilter_string_to_rules(parser->data, ",", "|",
                                           &rules, &count);
        if (r) {
            ERROR("parsing filter (5d), ignoring filter message", r);
            break;
        }
        parser->callb.filter_filter_func(parser->callb.priv, rules, count);
        break;
    }
    case usb_redir_control_packet:
        parser->callb.control_packet_func(parser->callb.priv,
            parser->header.id,
            (struct usb_redir_control_packet_header *)parser->type_header,
            parser->data, parser->data_len);
        break;
    case usb_redir_bulk_packet:
        parser->callb.bulk_packet_func(parser->callb.priv,
            parser->header.id,
            (struct usb_redir_bulk_packet_header *)parser->type_header,
            parser->data, parser->data_len);
        break;
    case usb_redir_iso_packet:
        parser->callb.iso_packet_func(parser->callb.priv, parser->header.id,
            (struct usb_redir_iso_packet_header *)parser->type_header,
            parser->data, parser->data_len);
        break;
    case usb_redir_interrupt_packet:
        parser->callb.interrupt_packet_func(parser->callb.priv,
            parser->header.id,
            (struct usb_redir_interrupt_packet_header *)parser->type_header,
            parser->data, parser->data_len);
        break;
    }
}

int usbredirparser_do_read(struct usbredirparser *parser_pub)
{
    struct usbredirparser_priv *parser =
        (struct usbredirparser_priv *)parser_pub;
    int r, type_header_len, data_len;
    uint8_t *dest;

    /* Skip forward to next packet (only used in error conditions) */
    while (parser->to_skip > 0) {
        uint8_t buf[65536];
        r = (parser->to_skip > sizeof(buf)) ? sizeof(buf) : parser->to_skip;
        r = parser->callb.read_func(parser->callb.priv, buf, r);
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
            r = parser->callb.read_func(parser->callb.priv, dest, r);
            if (r <= 0) {
                return r;
            }
        }

        if (parser->header_read < sizeof(parser->header)) {
            parser->header_read += r;
            if (parser->header_read == sizeof(parser->header)) {
                type_header_len =
                    usbredirparser_get_type_header_len(parser_pub,
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
                r = usbredirparser_verify_type_header(parser_pub,
                         parser->header.type, parser->type_header,
                         parser->data, parser->data_len, 0);
                if (r)
                    usbredirparser_call_type_func(parser);
                parser->header_read = 0;
                parser->type_header_len  = 0;
                parser->type_header_read = 0;
                parser->data_len  = 0;
                parser->data_read = 0;
                parser->data = NULL;
                if (!r)
                    return -2;
            }
        }
    }
}

int usbredirparser_has_data_to_write(struct usbredirparser *parser_pub)
{
    struct usbredirparser_priv *parser =
        (struct usbredirparser_priv *)parser_pub;
    return parser->write_buf != NULL;
}

int usbredirparser_do_write(struct usbredirparser *parser_pub)
{
    struct usbredirparser_priv *parser =
        (struct usbredirparser_priv *)parser_pub;
    struct usbredirparser_buf* wbuf;
    int w, ret = 0;

    LOCK(parser);
    for (;;) {    
        wbuf = parser->write_buf;
        if (!wbuf)
            break;

        w = wbuf->len - wbuf->pos;
        w = parser->callb.write_func(parser->callb.priv,
                                     wbuf->buf + wbuf->pos, w);
        if (w <= 0) {
            ret = -1;
            break;
        }

        /* See usbredirparser_write documentation */
        if ((parser->flags & usbredirparser_fl_write_cb_owns_buffer) &&
                w != wbuf->len)
            abort();

        wbuf->pos += w;
        if (wbuf->pos == wbuf->len) {
            parser->write_buf = wbuf->next;
            if (!(parser->flags & usbredirparser_fl_write_cb_owns_buffer))
                free(wbuf->buf);
            free(wbuf);
        }
    }
    UNLOCK(parser);
    return ret;
}

void usbredirparser_free_write_buffer(struct usbredirparser *parser,
    uint8_t *data)
{
    free(data);
}

void usbredirparser_free_packet_data(struct usbredirparser *parser,
    uint8_t *data)
{
    free(data);
}

static void usbredirparser_queue(struct usbredirparser *parser_pub,
    uint32_t type, uint32_t id, void *type_header_in,
    uint8_t *data_in, int data_len)
{
    struct usbredirparser_priv *parser =
        (struct usbredirparser_priv *)parser_pub;
    uint8_t *buf, *type_header_out, *data_out;
    struct usb_redir_header *header;
    struct usbredirparser_buf *wbuf, *new_wbuf;
    int type_header_len;

    type_header_len = usbredirparser_get_type_header_len(parser_pub, type, 1);
    if (type_header_len < 0) { /* This should never happen */
        ERROR("packet type unknown with internal call, please report!!");
        return;
    }

    if (!usbredirparser_verify_type_header(parser_pub, type, type_header_in,
                                           data_in, data_len, 1)) {
        ERROR("usbredirparser_send_* call invalid params, please report!!");
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

    LOCK(parser);
    if (!parser->write_buf) {
        parser->write_buf = new_wbuf;
    } else {
        /* maybe we should limit the write_buf stack depth ? */
        wbuf = parser->write_buf;
        while (wbuf->next)
            wbuf = wbuf->next;

        wbuf->next = new_wbuf;
    }
    UNLOCK(parser);
}

void usbredirparser_send_device_connect(struct usbredirparser *parser,
    struct usb_redir_device_connect_header *device_connect)
{
    usbredirparser_queue(parser, usb_redir_device_connect, 0, device_connect,
                         NULL, 0);
}

void usbredirparser_send_device_disconnect(struct usbredirparser *parser)
{
    usbredirparser_queue(parser, usb_redir_device_disconnect, 0, NULL,
                         NULL, 0);
}

void usbredirparser_send_reset(struct usbredirparser *parser)
{
    usbredirparser_queue(parser, usb_redir_reset, 0, NULL, NULL, 0);
}

void usbredirparser_send_interface_info(struct usbredirparser *parser,
    struct usb_redir_interface_info_header *interface_info)
{
    usbredirparser_queue(parser, usb_redir_interface_info, 0, interface_info,
                         NULL, 0);
}

void usbredirparser_send_ep_info(struct usbredirparser *parser,
    struct usb_redir_ep_info_header *ep_info)
{
    usbredirparser_queue(parser, usb_redir_ep_info, 0, ep_info, NULL, 0);
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

void usbredirparser_send_start_interrupt_receiving(struct usbredirparser *parser,
    uint32_t id,
    struct usb_redir_start_interrupt_receiving_header *start_interrupt_receiving)
{
    usbredirparser_queue(parser, usb_redir_start_interrupt_receiving, id,
                         start_interrupt_receiving, NULL, 0);
}

void usbredirparser_send_stop_interrupt_receiving(struct usbredirparser *parser,
    uint32_t id,
    struct usb_redir_stop_interrupt_receiving_header *stop_interrupt_receiving)
{
    usbredirparser_queue(parser, usb_redir_stop_interrupt_receiving, id,
                         stop_interrupt_receiving, NULL, 0);
}

void usbredirparser_send_interrupt_receiving_status(struct usbredirparser *parser,
    uint32_t id,
    struct usb_redir_interrupt_receiving_status_header *interrupt_receiving_status)
{
    usbredirparser_queue(parser, usb_redir_interrupt_receiving_status, id,
                         interrupt_receiving_status, NULL, 0);
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

void usbredirparser_send_filter_reject(struct usbredirparser *parser)
{
    if (!usbredirparser_peer_has_cap(parser, usb_redir_cap_filter))
        return;

    usbredirparser_queue(parser, usb_redir_filter_reject, 0, NULL, NULL, 0);
}

void usbredirparser_send_filter_filter(struct usbredirparser *parser_pub,
    struct usbredirfilter_rule *rules, int rules_count)
{
    struct usbredirparser_priv *parser =
        (struct usbredirparser_priv *)parser_pub;
    char *str;

    if (!usbredirparser_peer_has_cap(parser_pub, usb_redir_cap_filter))
        return;

    str = usbredirfilter_rules_to_string(rules, rules_count, ",", "|");
    if (!str) {
        ERROR("creating filter string, not sending filter");
        return;
    }
    usbredirparser_queue(parser_pub, usb_redir_filter_filter, 0, NULL,
                         str, strlen(str) + 1);
    free(str);
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

void usbredirparser_send_interrupt_packet(struct usbredirparser *parser,
    uint32_t id,
    struct usb_redir_interrupt_packet_header *interrupt_header,
    uint8_t *data, int data_len)
{
    usbredirparser_queue(parser, usb_redir_interrupt_packet, id,
                         interrupt_header, data, data_len);
}
