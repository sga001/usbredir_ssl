override CPPFLAGS    += $(shell pkg-config --cflags libusb-1.0)
LIBS                  = $(shell pkg-config --libs libusb-1.0)

USBREDIRSERVER_OBJS   = usbredirserver.o
TARGETS               = usbredirserver

all: $(TARGETS)

-include *.d

usbredirserver: $(USBREDIRSERVER_OBJS)
# rpath-link so that usbredirparser (needed by usbredirhost) can be found
# when building without having a previous version installed
	$(CC) $(LDFLAGS) -Wl,-rpath-link,../usbredirparser -o $@ $^ $(LIBS) -lusbredirhost

install: all
	mkdir -p $(DESTDIR)$(SBINDIR)
	install -m 755 usbredirserver $(DESTDIR)$(SBINDIR)

include ../Make.rules
