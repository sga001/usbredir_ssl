lib_LTLIBRARIES = libusbredirparser.la

libusbredirparser_la_SOURCES = usbredirparser.c
libusbredirparser_ladir = $(includedir)
libusbredirparser_la_HEADERS = usbredirparser.h  usbredirproto.h
libusbredirparser_la_LDFLAGS = -version-info $(LIBUSBREDIRPARSER_SO_VERSION) \
                               -no-undefined

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = libusbredirparser.pc

EXTRA_DIST = libusbredirparser.pc
