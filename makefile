SUBDIRS = ca821x-posix openthread platform
CLEANSUBDIRS := $(addsuffix .clean,$(SUBDIRS)) example.clean
CFLAGS = -g
LDFLAGS = -pthread

.PHONY: subdirs $(SUBDIRS) clean $(SUBCLEAN) full-lib example default

default: example

subdirs: $(SUBDIRS)

$(SUBDIRS):
	$(MAKE) -C $@

example: full-lib
	$(MAKE) -C $@ clean
	$(MAKE) -C $@

clean: $(CLEANSUBDIRS)

full-lib: ca821x-posix openthread platform
	$(AR) -M < libscript.mri

$(CLEANSUBDIRS):
	$(MAKE) -C $(subst .clean,,$@) clean
