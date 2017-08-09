TARGET_MAIN = cliapp
TARGET_MT = mtExample
EXCHANGE = kernel
LIBDIRS = ../openthread/src/cli/ ../ ../openthread/third_party/mbedtls/
LIBSEARCH := $(addprefix -L,$(LIBDIRS))
LIBS = -lm -lopenthread-cli-ftd -lotposixca8210-$(EXCHANGE)-ftd -lstdc++ -lpthread
INCLUDEDIRS := ./\
	../openthread/include/\
	../openthread/include/openthread/\
	../openthread/src/core/\
	./include/\
	../platform/\
	../ca8210-$(EXCHANGE)-exchange/cascoda-api/include/\
	../ca8210-$(EXCHANGE)-exchange/
INCLUDEDIRCMD := $(addprefix -I,$(INCLUDEDIRS)) 
CFLAGS := $(INCLUDEDIRCMD) -g -Wall -pthread

.PHONY: default all clean

default: $(TARGET_MAIN) $(TARGET_MT)
all: default

OBJECTS_MAIN = main.o
OBJECTS_MT = mainMultithread.o
HEADERS := $(foreach DIR,$(INCLUDEDIRS),$(notdir $(wildcard $(DIR)*.h)))

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

.PRECIOUS: $(TARGET_MAIN) $(TARGET_MT) $(OBJECTS_MAIN) $(OBJECTS_MT)

$(TARGET_MAIN): $(OBJECTS_MAIN)
	$(CC) $(OBJECTS_MAIN) -Wall $(LIBSEARCH) $(LIBS) -o $@
	
$(TARGET_MT): $(OBJECTS_MT)
	$(CC) $(OBJECTS_MT) -Wall $(LIBSEARCH) $(LIBS) -o $@

clean:
	-rm -f *.o
	-rm -f $(TARGET_MT)
	-rm -f $(TARGET_MAIN)
