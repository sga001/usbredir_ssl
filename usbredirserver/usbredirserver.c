/* usbredirserver.c simple usb network redirection tcp/ip server (host).

   Copyright 2010-2011 Red Hat, Inc.

   Red Hat Authors:
   Hans de Goede <hdegoede@redhat.com>

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public
   License as published by the Free Software Foundation; either
   version 2 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this library; if not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <getopt.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/time.h>
#include <netdb.h>
#include "usbredirhost.h"

#define VERSION "usbredirserver " USBREDIR_VERSION

static int verbose = usbredirparser_info;
static int client_fd, running = 1;
static libusb_context *ctx;
static struct usbredirhost *host;

static const struct option longopts[] = {
    { "port", required_argument, NULL, 'p' },
    { "verbose", required_argument, NULL, 'v' },
    { "help", no_argument, NULL, 'h' },
    { NULL, 0, NULL, 0 }
};

static void usbredirserver_log(void *priv, int level, const char *msg)
{
    if (level <= verbose)
        fprintf(stderr, "%s\n", msg);
}

static int usbredirserver_read(void *priv, uint8_t *data, int count)
{
    int r = read(client_fd, data, count);
    if (r < 0) {
        if (errno == EAGAIN)
            return 0;
        return -1;
    }
    if (r == 0) { /* Client disconnected */
        close(client_fd);
        client_fd = -1;
    }
    return r;
}

static int usbredirserver_write(void *priv, uint8_t *data, int count)
{
    int r = write(client_fd, data, count);
    if (r < 0) {
        if (errno == EAGAIN)
            return 0;
        if (errno == EPIPE) { /* Client disconnected */
            close(client_fd);
            client_fd = -1;
            return 0;
        }
        return -1;
    }
    return r;
}

static void usage(int exit_code, char *argv0)
{
    fprintf(exit_code? stderr:stdout,
        "Usage: %s [-p|--port <port>] [-v|--verbose <0-5>] <usbbus-usbaddr|vendorid:prodid>\n",
        argv0);
    exit(exit_code);
}

static void invalid_usb_device_id(char *usb_device_id, char *argv0)
{
    fprintf(stderr, "Invalid usb device identifier: %s\n", usb_device_id);
    usage(1, argv0);
}

static void run_main_loop(void)
{
    const struct libusb_pollfd **pollfds = NULL;
    fd_set readfds, writefds;
    int i, n, nfds;
    struct timeval timeout, *timeout_p;

    while (running && client_fd != -1) {
        FD_ZERO(&readfds);
        FD_ZERO(&writefds);

        FD_SET(client_fd, &readfds);
        if (usbredirhost_has_data_to_write(host)) {
            FD_SET(client_fd, &writefds);
        }
        nfds = client_fd + 1;

        free(pollfds);
        pollfds = libusb_get_pollfds(ctx);
        for (i = 0; pollfds && pollfds[i]; i++) {
            if (pollfds[i]->events & POLLIN) {
                FD_SET(pollfds[i]->fd, &readfds);
            }
            if (pollfds[i]->events & POLLOUT) {
                FD_SET(pollfds[i]->fd, &writefds);
            }
            if (pollfds[i]->fd >= nfds)
                nfds = pollfds[i]->fd + 1;
        }

        if (libusb_get_next_timeout(ctx, &timeout) == 1) {
            timeout_p = &timeout;
        } else {
            timeout_p = NULL;
        }
        n = select(nfds, &readfds, &writefds, NULL, timeout_p);
        if (n == -1) {
            if (errno == EINTR) {
                continue;
            }
            perror("select");
            break;
        }
        memset(&timeout, 0, sizeof(timeout));
        if (n == 0) {
            libusb_handle_events_timeout(ctx, &timeout);
            continue;
        }

        if (FD_ISSET(client_fd, &readfds)) {
            if (usbredirhost_read_guest_data(host)) {
                break;
            }
        }
        if (FD_ISSET(client_fd, &writefds)) {
            if (usbredirhost_write_guest_data(host)) {
                break;
            }
        }

        for (i = 0; pollfds && pollfds[i]; i++) {
            if (FD_ISSET(pollfds[i]->fd, &readfds) ||
                FD_ISSET(pollfds[i]->fd, &writefds)) {
                libusb_handle_events_timeout(ctx, &timeout);
                break;
            }
        }
    }
    if (client_fd != -1) { /* Broken out of the loop because of an error ? */
        close(client_fd);
        client_fd = -1;
    }
    free(pollfds);
}

static void quit_handler(int sig)
{
    running = 0;
}

int main(int argc, char *argv[])
{
    int o, server_fd = -1;
    char *endptr, *delim;
    int port       = 4000;
    int usbbus     = -1;
    int usbaddr    = -1;
    int usbvendor  = -1;
    int usbproduct = -1;
    struct addrinfo *r, *res, hints;
    struct sigaction act;
    char port_str[16];
    libusb_device_handle *handle;

    while ((o = getopt_long(argc, argv, "hp:", longopts, NULL)) != -1) {
        switch (o) {
        case 'p':
            port = strtol(optarg, &endptr, 10);
            if (*endptr != '\0') {
                fprintf(stderr, "Inalid value for --port: '%s'\n", optarg);
                usage(1, argv[0]);
            }
            break;
        case 'v':
            verbose = strtol(optarg, &endptr, 10);
            if (*endptr != '\0') {
                fprintf(stderr, "Inalid value for --verbose: '%s'\n", optarg);
                usage(1, argv[0]);
            }
            break;
        case '?':
        case 'h':
            usage(o == '?', argv[0]);
            break;
        }
    }
    if (optind == argc) {
        fprintf(stderr, "Missing usb device identifier argument\n");
        usage(1, argv[0]);
    }
    delim = strchr(argv[optind], '-');
    if (delim && delim[1]) {
        usbbus = strtol(argv[optind], &endptr, 10);
        if (*endptr != '-') {
            invalid_usb_device_id(argv[optind], argv[0]);
        }
        usbaddr = strtol(delim + 1, &endptr, 10);
        if (*endptr != '\0') {
            invalid_usb_device_id(argv[optind], argv[0]);
        }
    } else {
        delim = strchr(argv[optind], ':');
        if (!delim || !delim[1]) {
            invalid_usb_device_id(argv[optind], argv[0]);
        }
        usbvendor = strtol(argv[optind], &endptr, 16);
        if (*endptr != ':') {
            invalid_usb_device_id(argv[optind], argv[0]);
        }
        usbproduct = strtol(delim + 1, &endptr, 16);
        if (*endptr != '\0') {
            invalid_usb_device_id(argv[optind], argv[0]);
        }
    }
    optind++;
    if (optind != argc) {
        fprintf(stderr, "Excess non option arguments\n");
        usage(1, argv[0]);
    }

    memset(&act, 0, sizeof(act));
    act.sa_handler = quit_handler;
    sigaction(SIGINT, &act, NULL);
    sigaction(SIGHUP, &act, NULL);
    sigaction(SIGTERM, &act, NULL);
    sigaction(SIGQUIT, &act, NULL);

    if (libusb_init(&ctx)) {
        fprintf(stderr, "Could not init libusb\n");
        exit(1);
    }

    libusb_set_debug(ctx, verbose);

    memset(&hints, 0, sizeof(hints));
    hints.ai_flags = AI_ADDRCONFIG | AI_NUMERICSERV | AI_PASSIVE;
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    sprintf(port_str, "%d", port);
    if (getaddrinfo(NULL, port_str, &hints, &res) != 0) {
        perror("getaddrinfo");
        exit(1);
    }

    for (r = res; r != NULL; r = r->ai_next) {
        server_fd = socket(r->ai_family, r->ai_socktype, r->ai_protocol);
        if (server_fd == -1)
            continue;

        if (bind(server_fd, r->ai_addr, r->ai_addrlen) == 0)
            break;

        close(server_fd);
    }
    freeaddrinfo(res);

    if (r == NULL) {
        fprintf(stderr, "Could not bind to port: %s\n", port_str);
        exit(1);
    }

    if (listen(server_fd, 1)) {
        perror("listen");
        exit(1);
    }

    while (running) {
        client_fd = accept(server_fd, NULL, 0);
        if (client_fd == -1) {
            if (errno == EINTR) {
                continue;
            }
            perror("accept");
            break;
        }

        fcntl(client_fd, F_SETFL,
              (long)fcntl(client_fd, F_GETFL) | O_NONBLOCK);

        /* Try to find the specified usb device */
        if (usbvendor != -1) {
            handle = libusb_open_device_with_vid_pid(ctx, usbvendor,
                                                     usbproduct);
            if (!handle) {
                fprintf(stderr,
                    "Could not open an usb-device with vid:pid %04x:%04x\n",
                    usbvendor, usbproduct);
                exit(1);
            }
        } else {
            /* FIXME */
            fprintf(stderr, "bus-addr usb device indentification is not implemented\n");
            exit(1);
        }
        if (!handle) {
            close(client_fd);
            continue;
        }

        host = usbredirhost_open(ctx, handle, usbredirserver_log,
                                 usbredirserver_read, usbredirserver_write,
                                 NULL, VERSION, verbose);
        if (!host)
            exit(1);
        run_main_loop();
        usbredirhost_close(host);
    }

    close(server_fd);
    libusb_exit(ctx);
    exit(0);
}
