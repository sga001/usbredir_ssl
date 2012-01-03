/* usbredirhost.c usb network redirection usb host code header

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
#ifndef __USBREDIRHOST_H
#define __USBREDIRHOST_H

#include <libusb.h>
#include "usbredirparser.h"
#include "usbredirfilter.h"

struct usbredirhost;

typedef void (*usbredirhost_flush_writes)(void *priv);

/* This function *takes ownership of* the passed in libusb_device_handle
   and sends the initial usb_redir_hello packet to the usb-guest.

   log_func is called by the usbredirhost to log various messages

   read_guest_data_func / write_guest_data_func are called by the
   usbredirhost to read/write data from/to the usb-guest.

   This function returns a pointer to the created usbredirhost object on
   success, or NULL on failure. Note that the passed in libusb_device_handle
   is closed on failure.

   Note:
   1) Both the usbredirtransport_log and the usbredirtransport_write
      callbacks may get called before this function completes.
   2) It is the responsibility of the code instantiating the usbredirhost
      to make sure that libusb_handle_events gets called (using the
      libusb_context from the passed in libusb_device_handle) when there are
      events waiting on the filedescriptors libusb_get_pollfds returns
   3) usbredirhost is *not* multithread safe it is the callers responsibility
      to make sure that no usbredirhost methods are called at the same time
      as libusb_handle_events
   4) usbredirhost is re-entrant safe, that is, it is ok to have one thread
      per usbredirhost without locking between the threads as long as each
      thread has its own libusb_context. IOW usbredirhost does not use
      static or global variables.
*/

enum {
    usbredirhost_fl_write_cb_owns_buffer = 0x01, /* See usbredirparser.h */
};

struct usbredirhost *usbredirhost_open(
    libusb_context *usb_ctx,
    libusb_device_handle *usb_dev_handle,
    usbredirparser_log log_func,
    usbredirparser_read  read_guest_data_func,
    usbredirparser_write write_guest_data_func,
    void *func_priv, const char *version, int verbose, int flags);

/* See README.multi-thread */
struct usbredirhost *usbredirhost_open_full(
    libusb_context *usb_ctx,
    libusb_device_handle *usb_dev_handle,
    usbredirparser_log log_func,
    usbredirparser_read  read_guest_data_func,
    usbredirparser_write write_guest_data_func,
    usbredirhost_flush_writes flush_writes_func,
    usbredirparser_alloc_lock alloc_lock_func,
    usbredirparser_lock lock_func,
    usbredirparser_unlock unlock_func,
    usbredirparser_free_lock free_lock_func,
    void *func_priv, const char *version, int verbose, int flags);

/* Close the usbredirhost, returning control of the device back to any
   host kernel drivers for it, freeing any allocated memory, etc.

   Note this function calls libusb_handle_events to "reap" cancelled
   urbs before closing the libusb device handle. This means that if you
   are using the same libusb context for other purposes your transfer complete
   callbacks may get called! */
void usbredirhost_close(struct usbredirhost *host);

/* Call this whenever there is data ready for the usbredirhost to read from
   the usb-guest
   returns 0 on success, -1 if a read error happened, -2 if a parse error
   happened. If a read error happened this function will continue where it
   left of the last time on the next call. If a parse error happened it will
   skip to the next packet (*) on the next call.
   *) As determined by the faulty's package headers length field */
int usbredirhost_read_guest_data(struct usbredirhost *host);

/* If this returns true there is data queued to write to the usb-guest */
int usbredirhost_has_data_to_write(struct usbredirhost *host);

/* Call this when usbredirhost_has_data_to_write returns true
   returns 0 on success, -1 if a write error happened.
   If a write error happened, this function will retry writing any queued data
   on the next call, and will continue doing so until it has succeeded! */
int usbredirhost_write_guest_data(struct usbredirhost *host);

/* When passing the usbredirhost_fl_write_cb_owns_buffer flag to
   usbredirhost_open, this function must be called to free the data buffer
   passed to write_guest_data_func when done with this buffer. */
void usbredirhost_free_write_buffer(struct usbredirhost *host, uint8_t *data);

/* Get device and config descriptors from the USB device dev, and call
   usbredirfilter_check with the passed in filter rules and the needed info
   from the descriptors, flags gets passed to usbredirfilter_check unmodified.
   See the documentation of usbredirfilter_check for more details.

   Return value: -EIO or -ENOMEM when getting the descriptors fails, otherwise
       it returns the return value of the usbredirfilter_check call. */
int usbredirhost_check_device_filter(struct usbredirfilter_rule *rules,
    int rules_count, libusb_device *dev, int flags);

#endif
