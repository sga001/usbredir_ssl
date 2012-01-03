lib_LTLIBRARIES = libusbredirhost.la

libusbredirhost_la_SOURCES = usbredirhost.c
libusbredirhost_ladir = $(includedir)
libusbredirhost_la_HEADERS = usbredirhost.h
libusbredirhost_la_CFLAGS = $(LIBUSB_CFLAGS) -I$(top_srcdir)/usbredirparser
libusbredirhost_la_LIBADD = $(LIBUSB_LIBS) \
                            $(top_builddir)/usbredirparser/libusbredirparser.la
libusbredirhost_la_LDFLAGS = -version-info $(LIBUSBREDIRHOST_SO_VERSION) \
                             -no-undefined

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = libusbredirhost.pc
