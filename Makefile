SUBDIRS = usbredirparser usbredirhost usbredirserver usbredirtestclient

all install clean::
	for i in $(SUBDIRS); do \
		$(MAKE) -C $$i $@ || exit 1; \
	done

tag:
	@git tag -a -m "Tag as usbredir-$(USBREDIR_VERSION)" usbredir-$(USBREDIR_VERSION)
	@echo "Tagged as usbredir-$(USBREDIR_VERSION)"

archive-no-tag:
	@git archive --format=tar --prefix=usbredir-$(USBREDIR_VERSION)/ usbredir-$(USBREDIR_VERSION) > usbredir-$(USBREDIR_VERSION).tar
	@bzip2 -f usbredir-$(USBREDIR_VERSION).tar

archive: clean tag archive-no-tag

include Make.rules
