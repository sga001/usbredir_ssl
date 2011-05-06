SUBDIRS = usbredirparser usbredirhost usbredirserver usbredirtestclient

all install clean::
	for i in $(SUBDIRS); do \
		$(MAKE) -C $$i $@; \
	done

include Make.rules
