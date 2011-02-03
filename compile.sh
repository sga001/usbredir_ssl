#!/bin/sh
set -e

gcc -g -O2 -Wall -I/usr/include/libusb-1.0 -o usbredirserver \
    usbredirserver.c usbredirhost.c usbredirparser.c -lusb-1.0

gcc -O2 -Wall -o usbredirtestclient usbredirtestclient.c usbredirparser.c
