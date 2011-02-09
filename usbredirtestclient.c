/* usbredirtestclient.c simple usb network redirection test client (guest).

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
#include "usbredirparser.h"

/* Macros to go from an endpoint address to an index for our ep array */
#define EP2I(ep_address) (((ep_address & 0x80) >> 3) | (ep_address & 0x0f))
#define I2EP(i) (((i & 0x10) << 3) | (i & 0x0f))

#ifndef VERSION
#define VERSION "usbredirtestclient 0.0"
#endif

static void usbredirtestclient_device_info(void *priv,
    struct usb_redir_device_info_header *device_info);
static void usbredirtestclient_ep_info(void *priv,
    struct usb_redir_ep_info_header *ep_info);
static void usbredirtestclient_reset_status(void *priv, uint32_t id,
    struct usb_redir_reset_status_header *reset_status);
static void usbredirtestclient_configuration_status(void *priv, uint32_t id,
    struct usb_redir_configuration_status_header *configuration_status);
static void usbredirtestclient_alt_setting_status(void *priv, uint32_t id,
    struct usb_redir_alt_setting_status_header *alt_setting_status);
static void usbredirtestclient_iso_stream_status(void *priv, uint32_t id,
    struct usb_redir_iso_stream_status_header *iso_stream_status);
static void usbredirtestclient_bulk_streams_status(void *priv, uint32_t id,
    struct usb_redir_bulk_streams_status_header *bulk_streams_status);
static void usbredirtestclient_control_packet(void *priv, uint32_t id,
    struct usb_redir_control_packet_header *control_header,
    uint8_t *data, int data_len);
static void usbredirtestclient_bulk_packet(void *priv, uint32_t id,
    struct usb_redir_bulk_packet_header *bulk_header,
    uint8_t *data, int data_len);
static void usbredirtestclient_iso_packet(void *priv, uint32_t id,
    struct usb_redir_iso_packet_header *iso_header,
    uint8_t *data, int data_len);

/* id's for all the test commands we send */
enum {
   reset_id,
   get_config_id,
   set_config_id,
   get_alt_id,
   set_alt_id,
   first_cmdline_id
};

static int verbose = usbredirparser_info; /* 2 */
static int client_fd, running = 1;
static struct usbredirparser *parser;
static int id = first_cmdline_id;

static const struct option longopts[] = {
    { "port", required_argument, NULL, 'p' },
    { "verbose", required_argument, NULL, 'v' },
    { "help", no_argument, NULL, 'h' },
    { NULL, 0, NULL, 0 }
};

static void usbredirtestclient_log(void *priv, int level, const char *msg)
{
    if (level <= verbose)
        fprintf(stderr, "%s\n", msg);
}

static int usbredirtestclient_read(void *priv, uint8_t *data, int count)
{
    int r = read(client_fd, data, count);
    if (r < 0) {
        if (errno == EAGAIN)
            return 0;
        return -1;
    }
    if (r == 0) { /* Server disconnected */
        close(client_fd);
        client_fd = -1;
    }
    return r;
}

static int usbredirtestclient_write(void *priv, uint8_t *data, int count)
{
    int r = write(client_fd, data, count);
    if (r < 0) {
        if (errno == EAGAIN)
            return 0;
        if (errno == EPIPE) { /* Server disconnected */
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
        "Usage: %s [-p|--port <port>] [-v|--verbose <0-3>] <server>\n",
        argv0);
    exit(exit_code);
}

static void run_main_loop(void)
{
    fd_set readfds, writefds;
    int n, nfds;

    while (running && client_fd != -1) {
        FD_ZERO(&readfds);
        FD_ZERO(&writefds);

        FD_SET(client_fd, &readfds);
        if (usbredirparser_has_data_to_write(parser)) {
            FD_SET(client_fd, &writefds);
        }
        nfds = client_fd + 1;

        n = select(nfds, &readfds, &writefds, NULL, NULL);
        if (n == -1) {
            if (errno == EINTR) {
                continue;
            }
            perror("select");
            break;
        }

        if (FD_ISSET(client_fd, &readfds)) {
            if (usbredirparser_do_read(parser)) {
                break;
            }
        }
        if (FD_ISSET(client_fd, &writefds)) {
            if (usbredirparser_do_write(parser)) {
                break;
            }
        }
    }
    if (client_fd != -1) { /* Broken out of the loop because of an error ? */
        close(client_fd);
        client_fd = -1;
    }
}

static void quit_handler(int sig)
{
    running = 0;
}

int main(int argc, char *argv[])
{
    int o;
    char *endptr, *server;
    struct addrinfo *r, *res, hints;
    struct sigaction act;
    char port_str[16];
    int port = 4000;

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
        fprintf(stderr, "Missing server argument\n");
        usage(1, argv[0]);
    }
    server = argv[optind];
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

    memset(&hints, 0, sizeof(hints));
    hints.ai_flags = AI_ADDRCONFIG | AI_NUMERICSERV;
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    sprintf(port_str, "%d", port);
    if (getaddrinfo(server, port_str, &hints, &res) != 0) {
        perror("getaddrinfo");
        exit(1);
    }

    for (r = res; r != NULL; r = r->ai_next) {
        client_fd = socket(r->ai_family, r->ai_socktype, r->ai_protocol);
        if (client_fd == -1)
            continue;

        if (connect(client_fd, r->ai_addr, r->ai_addrlen) == 0)
            break;

        close(client_fd);
    }
    freeaddrinfo(res);

    if (r == NULL) {
        fprintf(stderr, "Could not connect to: [%s]:%s\n", server, port_str);
        exit(1);
    }

    fcntl(client_fd, F_SETFL, (long)fcntl(client_fd, F_GETFL) | O_NONBLOCK);
    parser = usbredirparser_create(usbredirtestclient_log,
                                   usbredirtestclient_read,
                                   usbredirtestclient_write,
                                   usbredirtestclient_device_info,
                                   usbredirtestclient_ep_info,
                                   NULL, /* reset */
                                   usbredirtestclient_reset_status,
                                   NULL, /* set config */
                                   NULL, /* get config */
                                   usbredirtestclient_configuration_status,
                                   NULL, /* set alt */
                                   NULL, /* get alt */
                                   usbredirtestclient_alt_setting_status,
                                   NULL, /* start iso */
                                   NULL, /* stop iso */
                                   usbredirtestclient_iso_stream_status,
                                   NULL, /* alloc bulk streams */
                                   NULL, /* free bulk streams */
                                   usbredirtestclient_bulk_streams_status,
                                   NULL, /* cancel packet */
                                   usbredirtestclient_control_packet,
                                   usbredirtestclient_bulk_packet,
                                   usbredirtestclient_iso_packet,
                                   NULL, VERSION, NULL, 0, 0);
    if (!parser) {
        exit(1);
    }

    /* Queue a reset, the other test commands will be send in response
       to the status packets of previous commands */
    usbredirparser_send_reset(parser, reset_id);

    run_main_loop();

    exit(0);
}

static void usbredirtestclient_cmdline_help(void)
{
    printf("Avaiable commands:\n"
        "ctrl <endpoint> <request> <request_type> <value> <index> <length> [data]\n"
        "quit\n"
        "help\n");
}

static int usbredirtestclient_cmdline_ctrl(void)
{
    struct usb_redir_control_packet_header control_header;
    char *arg, *endptr = NULL;
    uint8_t *data = NULL;
    int data_len;

    arg = strtok(NULL, " \t\n");
    if (arg) {
        control_header.endpoint = strtol(arg, &endptr, 0);
    }
    if (!arg || *endptr != '\0') {
        printf("Missing or invalid endpoint\n");
        return 0;
    }

    arg = strtok(NULL, " \t\n");
    if (arg) {
        control_header.request = strtol(arg, &endptr, 0);
    }
    if (!arg || *endptr != '\0') {
        printf("Missing or invalid request\n");
        return 0;
    }

    arg = strtok(NULL, " \t\n");
    if (arg) {
        control_header.requesttype = strtol(arg, &endptr, 0);
    }
    if (!arg || *endptr != '\0') {
        printf("Missing or invalid request type\n");
        return 0;
    }

    arg = strtok(NULL, " \t\n");
    if (arg) {
        control_header.value = strtol(arg, &endptr, 0);
    }
    if (!arg || *endptr != '\0') {
        printf("Missing or invalid value\n");
        return 0;
    }

    arg = strtok(NULL, " \t\n");
    if (arg) {
        control_header.index = strtol(arg, &endptr, 0);
    }
    if (!arg || *endptr != '\0') {
        printf("Missing or invalid index\n");
        return 0;
    }

    arg = strtok(NULL, " \t\n");
    if (arg) {
        control_header.length = strtol(arg, &endptr, 0);
    }
    if (!arg || *endptr != '\0') {
        printf("Missing or invalid length\n");
        return 0;
    }

    if (!(control_header.endpoint & 0x80)) {    
        int i;

        data = malloc(control_header.length);
        if (!data) {
            fprintf(stderr, "Out of memory!\n");
            close(client_fd);
            client_fd = -1;
            return 0;
        }

        for (i = 0; i < control_header.length; i++) {
            arg = strtok(NULL, " \t\n");
            if (arg) {
                data[i] = strtol(arg, &endptr, 0);
            }
            if (!arg || *endptr != '\0') {
                printf("Missing or invalid data byte(s)\n");
                return 0;
            }
        }
        data_len = control_header.length;
    } else {
        data_len = 0;
    }
    usbredirparser_send_control_packet(parser, id, &control_header,
                                       data, data_len);
    printf("Send control packet with id: %u\n", id);
    id++;
    return 1;
}

static void usbredirtestclient_cmdline_parse(void)
{
    char buf[128];
    char *cmd;

    while (running && client_fd != -1) {
        printf("> ");
        if (!fgets(buf, sizeof(buf), stdin)) {
            close(client_fd);
            client_fd = -1;
            return;
        }

        cmd = strtok(buf, " \t\n");
        if (!cmd)
            continue;

        if (!strcmp(cmd, "help")) {
            usbredirtestclient_cmdline_help();
        } else if (!strcmp(cmd, "quit")) {
            close(client_fd);
            client_fd = -1;
            return;
        } else if (!strcmp(cmd, "ctrl")) {
            if (usbredirtestclient_cmdline_ctrl()) {
                return; /* Run main loop until an answer is received */
            }
        } else {
            printf("unknown command: '%s', type 'help' for help\n", cmd);
        }
    }
}

static void usbredirtestclient_device_info(void *priv,
    struct usb_redir_device_info_header *device_info)
{
    printf("device info, speed: %s\n", device_info->slow ? "lo" : "full");
}

static void usbredirtestclient_ep_info(void *priv,
    struct usb_redir_ep_info_header *ep_info)
{
    int i;

    for (i = 0; i < 32; i++) {
       if (ep_info->type[i] != usb_redir_type_invalid) {
           printf("endpoint: %02X, type: %d, interval: %d, interface: %d\n",
                  I2EP(i), (int)ep_info->type[i], (int)ep_info->interval[i],
                  (int)ep_info->interface[i]);
       }
    }
}

static void usbredirtestclient_reset_status(void *priv, uint32_t id,
    struct usb_redir_reset_status_header *reset_status)
{
    switch (id) {
    case reset_id:
        printf("Reset status: %d\n", reset_status->status);
        usbredirparser_send_get_configuration(parser, get_config_id);
        break;
    default:
        fprintf(stderr, "Unexpected reset status packet, id: %d\n", id);
    }
}

static void usbredirtestclient_configuration_status(void *priv, uint32_t id,
    struct usb_redir_configuration_status_header *config_status)
{
    struct usb_redir_set_configuration_header set_config;
    struct usb_redir_get_alt_setting_header get_alt;

    switch (id) {
    case get_config_id:
        printf("Get config: %d, status: %d\n", config_status->configuration,
               config_status->status);
        set_config.configuration = config_status->configuration;
        usbredirparser_send_set_configuration(parser, set_config_id,
                                              &set_config);
        break;
    case set_config_id:
        printf("Set config: %d, status: %d\n", config_status->configuration,
               config_status->status);
        get_alt.interface = 0; /* Assume the device has an interface 0 */
        usbredirparser_send_get_alt_setting(parser, get_alt_id, &get_alt);
        break;
    default:
        fprintf(stderr, "Unexpected configuration status packet, id: %d\n",
                id);
    }
}

static void usbredirtestclient_alt_setting_status(void *priv, uint32_t id,
    struct usb_redir_alt_setting_status_header *alt_setting_status)
{
    struct usb_redir_set_alt_setting_header set_alt;

    switch (id) {
    case get_alt_id:
        printf("Get alt: %d, interface: %d, status: %d\n",
               alt_setting_status->alt, alt_setting_status->interface,
               alt_setting_status->status);
        set_alt.interface = alt_setting_status->interface;
        set_alt.alt = alt_setting_status->alt;
        usbredirparser_send_set_alt_setting(parser, set_alt_id, &set_alt);
        break;
    case set_alt_id:
        printf("Set alt: %d, interface: %d, status: %d\n",
               alt_setting_status->alt, alt_setting_status->interface,
               alt_setting_status->status);
        /* Auto tests done, go interactive */
        usbredirtestclient_cmdline_parse();
        break;
    default:
        fprintf(stderr, "Unexpected alt status packet, id: %d\n", id);
    }
}

static void usbredirtestclient_iso_stream_status(void *priv, uint32_t id,
    struct usb_redir_iso_stream_status_header *iso_stream_status)
{
}

static void usbredirtestclient_bulk_streams_status(void *priv, uint32_t id,
    struct usb_redir_bulk_streams_status_header *bulk_streams_status)
{
}

static void usbredirtestclient_control_packet(void *priv, uint32_t id,
    struct usb_redir_control_packet_header *control_header,
    uint8_t *data, int data_len)
{
    int i;
    printf("Control packet id: %u, status: %d", id, control_header->status);

    if (data_len) {
        printf(", data:");
    }
    for (i = 0; i < data_len; i++) {
        printf(" %02X", (unsigned int)data[i]);
    }
    printf("\n");

    /* Ask what to send next */
    usbredirtestclient_cmdline_parse();
}

static void usbredirtestclient_bulk_packet(void *priv, uint32_t id,
    struct usb_redir_bulk_packet_header *bulk_header,
    uint8_t *data, int data_len)
{
}

static void usbredirtestclient_iso_packet(void *priv, uint32_t id,
    struct usb_redir_iso_packet_header *iso_header,
    uint8_t *data, int data_len)
{
}
