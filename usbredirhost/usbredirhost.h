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

struct usbredirhost;

/* This function *takes ownership of* the passed in libusb_device_handle
   and sends the initial usb_redir_hello packet to the usb-guest.

   log_func is called by the usbredirhost to log various messages

   read_guest_data_func / write_guest_data_func are called by the
   usbredirhost to read/write data from/to the usb-guest.

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
struct usbredirhost *usbredirhost_open(libusb_device_handle *usb_dev_handle,
    usbredirparser_log log_func,
    usbredirparser_read  read_guest_data_func,
    usbredirparser_write write_guest_data_func,
    void *func_priv, const char *version, int verbose);

/* Close the usbredirhost, returning control of the device back to any
   host kernel drivers for it, freeing any allocated memory, etc. */
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

#endif
