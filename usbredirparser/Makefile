USBREDIRPARSER_OBJS = usbredirparser.o
TARGETS             = $(USBREDIRPARSER_LIB) libusbredirparser.pc
INCLUDES            = usbredirproto.h usbredirparser.h

ifeq ($(LINKTYPE),static)
USBREDIRPARSER_LIB  = libusbredirparser.a
else
USBREDIRPARSER_LIB  = libusbredirparser.so
override CPPFLAGS  += -fPIC
endif

LIB_RELEASE = 0

all: $(TARGETS)

-include $(USBREDIRPARSER_OBJS:.o=.d)

$(USBREDIRPARSER_LIB): $(USBREDIRPARSER_OBJS)

libusbredirparser.pc:
	@echo prefix=$(PREFIX) > libusbredirparser.pc
	@echo libdir=$(LIBDIR) >> libusbredirparser.pc
	@echo >> libusbredirparser.pc
	@echo 'Name: libusbredirparser' >> libusbredirparser.pc
	@echo 'Description: usbredirparser library' >> libusbredirparser.pc
	@echo 'Version: '$(USBREDIR_VERSION) >> libusbredirparser.pc
	@echo 'Libs: -L$${libdir} -lusbredirparser' >> libusbredirparser.pc
	@echo 'Cflags: -I$${prefix}/include' >> libusbredirparser.pc

install: all
	mkdir -p $(DESTDIR)$(PREFIX)/include
	install -p -m 644 $(INCLUDES) $(DESTDIR)$(PREFIX)/include
	mkdir -p $(DESTDIR)$(LIBDIR)
ifeq ($(LINKTYPE),static)
	install -m 644 $(USBREDIRPARSER_LIB) $(DESTDIR)$(LIBDIR)
else
	install -m 755 $(USBREDIRPARSER_LIB).$(LIB_RELEASE) $(DESTDIR)$(LIBDIR)
	cd $(DESTDIR)$(LIBDIR) && \
	  ln -f -s $(USBREDIRPARSER_LIB).$(LIB_RELEASE) $(USBREDIRPARSER_LIB)
endif
	mkdir -p $(DESTDIR)$(LIBDIR)/pkgconfig
	install -m 644 libusbredirparser.pc $(DESTDIR)$(LIBDIR)/pkgconfig

include ../Make.rules
