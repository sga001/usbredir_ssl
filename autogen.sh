#!/bin/sh

autoreconf -fi
if [ -z "$NOCONFIGURE" ]; then
    ./configure $@
fi
