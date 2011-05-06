USBREDIRTESTCLIENT_OBJS = usbredirtestclient.o
TARGETS                 = usbredirtestclient

all: $(TARGETS)

-include *.d

usbredirtestclient: $(USBREDIRTESTCLIENT_OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS) -lusbredirparser

install:

include ../Make.rules
