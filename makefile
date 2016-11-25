SUBDIRS = example ca8210-kernel-exchange openthread platform
CLEANSUBDIRS := $(addsuffix .clean,$(SUBDIRS))
CFLAGS = -g
LDFLAGS = -pthread

LIBPACK = openthread/src/core/libopenthread.a ca8210-kernel-exchange/libca8210.a platform/libotplatposixca8210.a openthread/third_party/mbedtls/libmbedcrypto.a
LIBPACKED = libotposixca8210.a

.PHONY: subdirs $(SUBDIRS) clean $(SUBCLEAN) full-lib

subdirs: $(SUBDIRS)

$(SUBDIRS):
	$(MAKE) -C $@

example: full-lib

clean: $(CLEANSUBDIRS)

full-lib: ca8210-kernel-exchange openthread platform
	ar cqT  $(LIBPACKED) $(LIBPACK) && echo -e 'create $(LIBPACKED)\naddlib $(LIBPACKED)\nsave\nend' | ar -M

$(CLEANSUBDIRS):
	$(MAKE) -C $(subst .clean,,$@) clean
