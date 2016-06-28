SUBDIRS = app cascoda openthread platform
CLEANSUBDIRS = $(SUBDIRS)
LDFLAGS = -pthread

.PHONY: subdirs $(SUBDIRS)
.PHONY: clean $(SUBCLEAN)

subdirs: $(SUBDIRS)

$(SUBDIRS):
	$(MAKE) -C $@

app: cascoda openthread platform

clean: $(CLEANSUBDIRS)

$(CLEANSUBDIRS):
	$(MAKE) -C $@ clean