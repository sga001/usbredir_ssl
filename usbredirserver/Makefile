override CPPFLAGS    += $(shell pkg-config --cflags libusb-1.0)
LIBS                  = $(shell pkg-config --libs libusb-1.0)

USBREDIRSERVER_OBJS   = usbredirserver.o
TARGETS               = usbredirserver

all: $(TARGETS)

-include *.d

usbredirserver: $(USBREDIRSERVER_OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS) -lusbredirhost

install: all
	mkdir -p $(DESTDIR)$(SBINDIR)
	install -m 755 usbredirserver $(DESTDIR)$(SBINDIR)

include ../Make.rules
