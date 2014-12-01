#!/bin/sh

srcdir=`dirname $0`
test -z "$srcdir" && srcdir=.

(
    cd "$srcdir"
    autoreconf -fi
)

test -n "$NOCONFIGURE" || "$srcdir/configure" "$@"
