SUBDIRS = ca8210-kernel-exchange ca8210-usb-exchange openthread platform
CLEANSUBDIRS := $(addsuffix .clean,$(SUBDIRS)) example
CFLAGS = -g
LDFLAGS = -pthread
EXCHANGE = kernel

.PHONY: subdirs $(SUBDIRS) clean $(SUBCLEAN) full-lib example default

default: example

subdirs: $(SUBDIRS)

$(SUBDIRS):
	$(MAKE) -C $@ EXCHANGE=$(EXCHANGE)

example: full-lib
	$(MAKE) -C $@ EXCHANGE=$(EXCHANGE)

clean: $(CLEANSUBDIRS)

full-lib: ca8210-kernel-exchange ca8210-usb-exchange openthread platform
	$(AR) -M < libscript.mri

$(CLEANSUBDIRS):
	$(MAKE) -C $(subst .clean,,$@) clean
