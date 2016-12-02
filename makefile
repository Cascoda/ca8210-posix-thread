SUBDIRS = example ca8210-kernel-exchange openthread platform
CLEANSUBDIRS := $(addsuffix .clean,$(SUBDIRS))
CFLAGS = -g
LDFLAGS = -pthread

.PHONY: subdirs $(SUBDIRS) clean $(SUBCLEAN) full-lib

subdirs: $(SUBDIRS)

$(SUBDIRS):
	$(MAKE) -C $@

example: full-lib

clean: $(CLEANSUBDIRS)

full-lib: ca8210-kernel-exchange openthread platform
	$(AR) -M < libscript.mri

$(CLEANSUBDIRS):
	$(MAKE) -C $(subst .clean,,$@) clean
