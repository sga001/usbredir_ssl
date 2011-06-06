SUBDIRS = usbredirparser usbredirhost usbredirserver usbredirtestclient

all install clean::
	for i in $(SUBDIRS); do \
		$(MAKE) -C $$i $@ || exit 1; \
	done

include Make.rules
