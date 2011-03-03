/* usbredirproto.h usb redirection protocol definitions

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
#ifndef __USBREDIRPROTO_H
#define __USBREDIRPROTO_H

/* PACK macros borrowed from spice-protocol */
#ifdef __GNUC__

#define ATTR_PACKED __attribute__ ((__packed__))

#ifdef __MINGW32__
#pragma pack(push,1)
#endif

#else

#pragma pack(push)
#pragma pack(1)
#define ATTR_PACKED
#pragma warning(disable:4200)
#pragma warning(disable:4103)

#endif

#include <stdint.h>

enum {
    usb_redir_success,
    usb_redir_cancelled,    /* The transfer was cancelled */
    usb_redir_inval,        /* Invalid packet type / length / ep, etc. */
    usb_redir_ioerror,      /* IO error */
    usb_redir_stall,        /* Stalled */
    usb_redir_timeout,      /* Request timed out */
    usb_redir_disconnected, /* The device has been disconnected */
};

enum {
    /* Note these 4 match the usb spec! */
    usb_redir_type_control,
    usb_redir_type_iso,
    usb_redir_type_bulk,
    usb_redir_type_interrupt,
    usb_redir_type_invalid = 255
};

enum {
    usb_redir_speed_low,
    usb_redir_speed_full,
    usb_redir_speed_high,
    usb_redir_speed_super,
    usb_redir_speed_unknown = 255
};

enum {
    /* Control packets */
    usb_redir_hello,
    usb_redir_device_info,
    usb_redir_ep_info,
    usb_redir_device_disconnected,
    usb_redir_reset,
    usb_redir_reset_status,
    usb_redir_set_configuration,
    usb_redir_get_configuration,
    usb_redir_configuration_status,
    usb_redir_set_alt_setting,
    usb_redir_get_alt_setting,
    usb_redir_alt_setting_status,
    usb_redir_start_iso_stream,
    usb_redir_stop_iso_stream,
    usb_redir_iso_stream_status,
    usb_redir_start_interrupt_receiving,
    usb_redir_stop_interrupt_receiving,
    usb_redir_interrupt_receiving_status,
    usb_redir_alloc_bulk_streams,
    usb_redir_free_bulk_streams,
    usb_redir_bulk_streams_status,
    usb_redir_cancel_data_packet,
    /* Data packets */
    usb_redir_control_packet = 100,
    usb_redir_bulk_packet,
    usb_redir_iso_packet,
    usb_redir_interrupt_packet,
};

enum {
    usb_redir_cap_bulk_streams,
};
/* Number of uint32_t-s needed to hold all (known) capabilities */
#define USB_REDIR_CAPS_SIZE 1

struct usb_redir_header {
    uint32_t type;
    uint32_t length;
    uint32_t id;  
} ATTR_PACKED;

struct usb_redir_hello_header {
    char     version[64];
    uint32_t capabilities[0];
} ATTR_PACKED;

struct usb_redir_device_info_header {
    uint8_t speed;
} ATTR_PACKED;

struct usb_redir_ep_info_header {
    uint8_t type[32];
    uint8_t interval[32];
    uint8_t interface[32];
} ATTR_PACKED;

struct usb_redir_reset_status_header {
    uint8_t status;
} ATTR_PACKED;

struct usb_redir_set_configuration_header {
    uint8_t configuration;
} ATTR_PACKED;

struct usb_redir_configuration_status_header {
    uint8_t status;
    uint8_t configuration;
} ATTR_PACKED;

struct usb_redir_set_alt_setting_header {
    uint8_t interface;
    uint8_t alt;
} ATTR_PACKED;

struct usb_redir_get_alt_setting_header {
    uint8_t interface;
} ATTR_PACKED;

struct usb_redir_alt_setting_status_header {
    uint8_t status;
    uint8_t interface;
    uint8_t alt;
} ATTR_PACKED;

struct usb_redir_start_iso_stream_header {
    uint8_t endpoint;
    uint8_t pkts_per_urb;
    uint8_t no_urbs;
} ATTR_PACKED;

struct usb_redir_stop_iso_stream_header {
    uint8_t endpoint;
} ATTR_PACKED;

struct usb_redir_iso_stream_status_header {
    uint8_t status;
    uint8_t endpoint;
} ATTR_PACKED;

struct usb_redir_start_interrupt_receiving_header {
    uint8_t endpoint;
} ATTR_PACKED;

struct usb_redir_stop_interrupt_receiving_header {
    uint8_t endpoint;
} ATTR_PACKED;

struct usb_redir_interrupt_receiving_status_header {
    uint8_t status;
    uint8_t endpoint;
} ATTR_PACKED;

struct usb_redir_alloc_bulk_streams_header {
    uint8_t endpoint;
    uint8_t no_streams;
} ATTR_PACKED;

struct usb_redir_free_bulk_streams_header {
    uint8_t endpoint;
} ATTR_PACKED;

struct usb_redir_bulk_streams_status_header {
    uint8_t status;
    uint8_t endpoint;
    uint8_t no_streams;
} ATTR_PACKED;

struct usb_redir_control_packet_header {
    uint8_t endpoint;
    uint8_t request;
    uint8_t requesttype;
    uint8_t status;
    uint16_t value;
    uint16_t index;
    uint16_t length; 
} ATTR_PACKED;

struct usb_redir_bulk_packet_header {
    uint8_t endpoint;
    uint8_t status;
    uint16_t length;
    uint32_t stream_id;
} ATTR_PACKED;

struct usb_redir_iso_packet_header {
    uint8_t endpoint;
    uint8_t status;
    uint16_t length;
} ATTR_PACKED;

struct usb_redir_interrupt_packet_header {
    uint8_t endpoint;
    uint8_t status;
    uint16_t length;
} ATTR_PACKED;

#undef ATTR_PACKED

#if defined(__MINGW32__) || !defined(__GNUC__)
#pragma pack(pop)
#endif

#endif
