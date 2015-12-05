/* usbredirserver.c simple usb network redirection tcp/ip server (host).

   Copyright 2010-2011 Red Hat, Inc.

   Red Hat Authors:
   Hans de Goede <hdegoede@redhat.com>

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public
   License as published by the Free Software Foundation; either
   version 2 of the License, or (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this library; if not, see <http://www.gnu.org/licenses/>.
*/

#include "config.h"

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
#include <netinet/in.h>
#include "usbredirhost.h"


#define SERVER_VERSION "usbredirserver " PACKAGE_VERSION

#include <stdbool.h>
#include <openssl/rand.h>
#include <openssl/ssl.h>
#include <openssl/x509.h>

static SSL_CTX *ssl_ctx = NULL;
static SSL *ssl_conn = NULL;
static bool use_ssl = false;
#define ONERR(C) do { if (C) { return -1; } } while (0)
#define EVSSL_CA_CERT "./keys/ca_cert.pem"
#define EVSSL_VERIFY_DEPTH 9
#define EVSSL_CIPHERS "ECDHE-RSA-AES256-GCM-SHA384:ECDHE-RSA-AES256-SHA384:ECDHE-RSA-AES128-GCM-SHA256:ECDHE-RSA-AES128-SHA256:ECDHE-RSA-RC4-SHA:ECDHE-RSA-AES256-SHA:HIGH:!aNULL:!eNULL:!EXP:!LOW:!MEDIUM:!MD5"
#define EVSSL_SRV_KEY "./keys/server_key.pem"
#define EVSSL_SRV_CERT "./keys/server_cert.pem"
#define EVSSL_CLT_NAME "evssl_client"
#define EVSSL_NAMEBUF_LEN 16

static int verbose = usbredirparser_info;
static int client_fd, running = 1;
static libusb_context *ctx;
static struct usbredirhost *host;

static const struct option longopts[] = {
    { "port", required_argument, NULL, 'p' },
    { "verbose", required_argument, NULL, 'v' },
    { "ssl", no_argument, NULL, 's' },
    { "help", no_argument, NULL, 'h' },
    { NULL, 0, NULL, 0 }
};

static int init_ssl_ctx(void) {
    SSL_load_error_strings();
    SSL_library_init();

    ONERR((ssl_ctx = SSL_CTX_new(SSLv23_server_method())) == NULL);

    // ssl general options
    SSL_CTX_set_options(ssl_ctx, SSL_OP_NO_SSLv2);
    SSL_CTX_set_options(ssl_ctx, SSL_OP_NO_SSLv3);
    SSL_CTX_set_options(ssl_ctx, SSL_OP_NO_TLSv1);
    SSL_CTX_set_options(ssl_ctx, SSL_OP_NO_TLSv1_1);
    SSL_CTX_clear_options(ssl_ctx, SSL_OP_NO_TLSv1_2);
    ONERR(SSL_CTX_set_cipher_list(ssl_ctx, EVSSL_CIPHERS) != 1);

    // load server certificate
    ONERR(SSL_CTX_use_certificate_chain_file(ssl_ctx, EVSSL_SRV_CERT) != 1);
    ONERR(SSL_CTX_use_PrivateKey_file(ssl_ctx, EVSSL_SRV_KEY, SSL_FILETYPE_PEM) != 1);
    ONERR(SSL_CTX_check_private_key(ssl_ctx) != 1);

    // force client to present certificate
    SSL_CTX_set_verify(ssl_ctx, SSL_VERIFY_PEER | SSL_VERIFY_FAIL_IF_NO_PEER_CERT, NULL);
    SSL_CTX_set_verify_depth(ssl_ctx, EVSSL_VERIFY_DEPTH);
    ONERR(SSL_CTX_load_verify_locations(ssl_ctx, EVSSL_CA_CERT, NULL) != 1);

    unsigned char sid[SSL_MAX_SSL_SESSION_ID_LENGTH];
    RAND_bytes(sid, SSL_MAX_SSL_SESSION_ID_LENGTH);
    ONERR(SSL_CTX_set_session_id_context(ssl_ctx, sid, sizeof(sid)) != 1);

    return 0;
}

static int setup_ssl_connection(void) {
    char cname[EVSSL_NAMEBUF_LEN];
    X509 *cert = NULL;
    X509_NAME *sname = NULL;

    // accept the connection
    ONERR((ssl_conn = SSL_new(ssl_ctx)) == NULL);
    ONERR(SSL_set_rfd(ssl_conn, client_fd) != 1);
    ONERR(SSL_set_wfd(ssl_conn, client_fd) != 1);
    ONERR(SSL_accept(ssl_conn) != 1);

    // check the certificate
    ONERR((cert = SSL_get_peer_certificate(ssl_conn)) == NULL);
    ONERR((sname = X509_get_subject_name(cert)) == NULL);
    X509_NAME_get_text_by_NID(sname, NID_commonName, cname, EVSSL_NAMEBUF_LEN);
    ONERR(strcmp(cname, EVSSL_CLT_NAME) != 0);

    return 0;
}

static void usbredirserver_log(void *priv, int level, const char *msg)
{
    if (level <= verbose)
        fprintf(stderr, "%s\n", msg);
}

static int usbredirserver_ssl_read(void *priv, uint8_t *data, int count) {
    int r = SSL_read(ssl_conn, data, count);
    // error reading
    if (r < 0) {
        int e = SSL_get_error(ssl_conn, r);
        if (e == SSL_ERROR_WANT_READ || e == SSL_ERROR_WANT_WRITE || e == SSL_ERROR_NONE) {
            return 0;
        } else if (e == SSL_ERROR_ZERO_RETURN) {
            goto ssl_read_client_hangup;
        } else {
            return -1;
        }
    }

    // nothing read; probably the other side closed connection
    if (r == 0 && SSL_get_error(ssl_conn, r) == SSL_ERROR_ZERO_RETURN) {
ssl_read_client_hangup:
        SSL_shutdown(ssl_conn);
        SSL_free(ssl_conn);
        ssl_conn = NULL;
        close(client_fd);
        client_fd = -1;
    }

    return r;
}

static int usbredirserver_ssl_write(void *priv, uint8_t *data, int count) {
    int r = SSL_write(ssl_conn, data, count);
    // error writing
    if (r < 0) {
        int e = SSL_get_error(ssl_conn, r);
        if (e == SSL_ERROR_WANT_READ || e == SSL_ERROR_WANT_WRITE || e == SSL_ERROR_NONE) {
            return 0;
        } else if (e == SSL_ERROR_ZERO_RETURN) {
            goto ssl_write_client_hangup;
        } else {
            return -1;
        }
    }

    // nothing written; probably the other side closed the connection
    if (r == 0 && SSL_get_error(ssl_conn, r) == SSL_ERROR_ZERO_RETURN) {
ssl_write_client_hangup:
        SSL_shutdown(ssl_conn);
        SSL_free(ssl_conn);
        ssl_conn = NULL;
        close(client_fd);
        client_fd = -1;
    }

    return r;
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
        /* usbredirhost_read_guest_data may have detected client disconnect */
        if (client_fd == -1)
            break;

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
    int o, flags, server_fd = -1;
    char *endptr, *delim;
    int port       = 4000;
    int usbbus     = -1;
    int usbaddr    = -1;
    int usbvendor  = -1;
    int usbproduct = -1;
    int on = 1;
    struct sockaddr_in6 serveraddr;
    struct sigaction act;
    libusb_device_handle *handle = NULL;

    while ((o = getopt_long(argc, argv, "hp:v:", longopts, NULL)) != -1) {
        switch (o) {
        case 'p':
            port = strtol(optarg, &endptr, 10);
            if (*endptr != '\0') {
                fprintf(stderr, "Invalid value for --port: '%s'\n", optarg);
                usage(1, argv[0]);
            }
            break;
        case 'v':
            verbose = strtol(optarg, &endptr, 10);
            if (*endptr != '\0') {
                fprintf(stderr, "Invalid value for --verbose: '%s'\n", optarg);
                usage(1, argv[0]);
            }
            break;
        case 's':
            use_ssl = true;
            if (init_ssl_ctx() < 0) {
                fprintf(stderr, "Error initializing SSL! Aborting.\n");
                exit(-1);
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

    server_fd = socket(AF_INET6, SOCK_STREAM, 0);
    if (server_fd == -1) {
        perror("Error creating ipv6 socket");
        exit(1);
    }

    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on))) {
        perror("Error setsockopt(SO_REUSEADDR) failed");
        exit(1);
    }
                                                      
    memset(&serveraddr, 0, sizeof(serveraddr));
    serveraddr.sin6_family = AF_INET6;
    serveraddr.sin6_port   = htons(port);
    serveraddr.sin6_addr   = in6addr_any;

    if (bind(server_fd, (struct sockaddr *)&serveraddr, sizeof(serveraddr))) {
        fprintf(stderr, "Error binding port %d: %s\n", port, strerror(errno));
        exit(1);
    }

    if (listen(server_fd, 1)) {
        perror("Error listening");
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

        flags = fcntl(client_fd, F_GETFL);
        if (flags == -1) {
            perror("fcntl F_GETFL");
            break;
        }
        flags = fcntl(client_fd, F_SETFL, flags | O_NONBLOCK);
        if (flags == -1) {
            perror("fcntl F_SETFL O_NONBLOCK");
            break;
        }

        // set up ssl connection
        if (use_ssl && (setup_ssl_connection() < 0)) {
            fprintf(stderr, "Error setting up SSL connection. Aborting.\n");
            break;
        }

        /* Try to find the specified usb device */
        if (usbvendor != -1) {
            handle = libusb_open_device_with_vid_pid(ctx, usbvendor,
                                                     usbproduct);
            if (!handle) {
                fprintf(stderr,
                    "Could not open an usb-device with vid:pid %04x:%04x\n",
                    usbvendor, usbproduct);
            }
        } else {
            libusb_device **list = NULL;
            ssize_t i, n;

            n = libusb_get_device_list(ctx, &list);
            for (i = 0; i < n; i++) {
                if (libusb_get_bus_number(list[i]) == usbbus &&
                        libusb_get_device_address(list[i]) == usbaddr)
                    break;
            }
            if (i < n) {
                if (libusb_open(list[i], &handle) != 0) {
                    fprintf(stderr,
                        "Could not open usb-device at bus-addr %d-%d\n",
                        usbbus, usbaddr);
                }
            } else {
                fprintf(stderr,
                    "Could not find an usb-device at bus-addr %d-%d\n",
                    usbbus, usbaddr);
            }
            libusb_free_device_list(list, 1);
        }
        if (!handle) {
            close(client_fd);
            continue;
        }

        host = usbredirhost_open(ctx, handle, usbredirserver_log,
                                 use_ssl ? usbredirserver_ssl_read : usbredirserver_read,
                                 use_ssl ? usbredirserver_ssl_write : usbredirserver_write,
                                 NULL, SERVER_VERSION, verbose, 0);
        if (!host)
            exit(1);
        run_main_loop();
        usbredirhost_close(host);
        handle = NULL;
    }

    close(server_fd);
    if (ssl_conn != NULL) {
        SSL_free(ssl_conn);
        ssl_conn = NULL;
    }
    libusb_exit(ctx);
    exit(0);
}
