SUBDIRS = app cascoda openthread platform

.PHONY: subdirs $(SUBDIRS)

subdirs: $(SUBDIRS)

$(SUBDIRS):
        $(MAKE) -C $@

app: cascoda openthread platform