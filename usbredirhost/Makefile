override CPPFLAGS    += $(shell pkg-config --cflags libusb-1.0)

LIBS_libusbredirhost  = $(shell pkg-config --libs libusb-1.0)

USBREDIRHOST_OBJS     = usbredirhost.o
USBREDIRHOST_DEPS     = $(USBREDIRHOST_OBJS)
TARGETS               = $(USBREDIRHOST_LIB) libusbredirhost.pc
INCLUDES              = usbredirhost.h

ifeq ($(LINKTYPE),static)
USBREDIRHOST_LIB      = libusbredirhost.a
else
USBREDIRHOST_LIB      = libusbredirhost.so
USBREDIRHOST_DEPS    += ../usbredirparser/libusbredirparser.so
override CPPFLAGS    += -fPIC
endif

all: $(TARGETS)

-include $(USBREDIRHOST_OBJS:.o=.d)

$(USBREDIRHOST_LIB): $(USBREDIRHOST_DEPS)

libusbredirhost.pc:
	@echo prefix=$(PREFIX) > libusbredirhost.pc
	@echo libdir=$(LIBDIR) >> libusbredirhost.pc
	@echo >> libusbredirhost.pc
	@echo 'Name: libusbredirhost' >> libusbredirhost.pc
	@echo 'Description: usbredirhost library' >> libusbredirhost.pc
	@echo 'Version: '$(USBREDIR_VERSION) >> libusbredirhost.pc
	@echo 'Requires.private: libusbredirparser' >> libusbredirhost.pc
	@echo 'Libs: -L$${libdir} -lusbredirhost' >> libusbredirhost.pc
	@echo 'Libs.private: -lusb-1.0' >> libusbredirhost.pc
	@echo 'Cflags: -I$${prefix}/include' >> libusbredirhost.pc

install: all
	mkdir -p $(DESTDIR)$(PREFIX)/include
	install -p -m 644 $(INCLUDES) $(DESTDIR)$(PREFIX)/include
	mkdir -p $(DESTDIR)$(LIBDIR)
ifeq ($(LINKTYPE),static)
	install -m 644 $(USBREDIRHOST_LIB) $(DESTDIR)$(LIBDIR)
else
	install -m 755 $(USBREDIRHOST_LIB).$(LIB_RELEASE) $(DESTDIR)$(LIBDIR)
	cd $(DESTDIR)$(LIBDIR) && \
	  ln -f -s $(USBREDIRHOST_LIB).$(LIB_RELEASE) $(USBREDIRHOST_LIB)
endif
	mkdir -p $(DESTDIR)$(LIBDIR)/pkgconfig
	install -m 644 libusbredirhost.pc $(DESTDIR)$(LIBDIR)/pkgconfig

include ../Make.rules
