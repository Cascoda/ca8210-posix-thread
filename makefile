SUBDIRS = app ca8210-kernel-exchange openthread platform
CLEANSUBDIRS := $(addsuffix .clean,$(SUBDIRS))
CFLAGS = -g
LDFLAGS = -pthread

.PHONY: subdirs $(SUBDIRS) clean $(SUBCLEAN)

subdirs: $(SUBDIRS)

$(SUBDIRS):
	$(MAKE) -C $@

app: ca8210-kernel-exchange openthread platform

clean: $(CLEANSUBDIRS)

$(CLEANSUBDIRS):
	$(MAKE) -C $(subst .clean,,$@) clean
