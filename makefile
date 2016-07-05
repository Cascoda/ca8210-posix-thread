SUBDIRS = app cascoda openthread platform
CLEANSUBDIRS := $(addsuffix .clean,$(SUBDIRS))
CFLAGS = -g
LDFLAGS = -pthread

.PHONY: subdirs $(SUBDIRS) clean $(SUBCLEAN)

subdirs: $(SUBDIRS)

$(SUBDIRS):
	$(MAKE) -C $@

app: cascoda openthread platform

clean: $(CLEANSUBDIRS)

$(CLEANSUBDIRS):
	$(MAKE) -C $(subst .clean,,$@) clean
