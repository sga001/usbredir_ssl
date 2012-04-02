sbin_PROGRAMS = usbredirserver

usbredirserver_SOURCES = usbredirserver.c
usbredirserver_LDADD = $(LIBUSB_LIBS) \
                       $(top_builddir)/usbredirhost/libusbredirhost.la
usbredirserver_CFLAGS = $(LIBUSB_CFLAGS) \
                        -I$(top_srcdir)/usbredirhost \
                        -I$(top_srcdir)/usbredirparser

dist_man_MANS = usbredirserver.1
