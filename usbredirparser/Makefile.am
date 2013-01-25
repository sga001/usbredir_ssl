lib_LTLIBRARIES = libusbredirparser.la

libusbredirparser_la_SOURCES = usbredirparser.c usbredirfilter.c usbredirproto-compat.h
libusbredirparser_ladir = $(includedir)
libusbredirparser_la_HEADERS = usbredirparser.h usbredirfilter.h usbredirproto.h
libusbredirparser_la_LDFLAGS = -version-info $(LIBUSBREDIRPARSER_SO_VERSION) \
                               -no-undefined \
                               -export-symbols-regex '^usbredir'

if OS_WIN32
libusbredirparser_la_SOURCES += strtok_r.c strtok_r.h
endif

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = libusbredirparser-0.5.pc
