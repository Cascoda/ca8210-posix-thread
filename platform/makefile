TARGET = libotplatposixca8210.a
EXCHANGE = kernel
LIBS = -lm
INCLUDEDIRS := ./\
	../openthread/include/openthread/platform/\
	../openthread/include/openthread/\
	../openthread/include/\
	../ca8210-kernel-exchange/ca821x-api/include/\
	../ca8210-kernel-exchange/\
	../openthread/src/core/\
	../openthread/src/core/common/
INCLUDEDIRCMD := $(addprefix -I,$(INCLUDEDIRS)) 
SOURCEDIR = ./
CFLAGS := $(INCLUDEDIRCMD) -g -Wall -std=c99

ifeq ($(EXCHANGE), "usb")
CFLAGS := -DUSE_USB_EXCHANGE 
endif

.PHONY: default all clean

default: $(TARGET)	

all: default

OBJECTS = $(patsubst %.c, %.o, $(wildcard $(SOURCEDIR)*.c))
HEADERS := $(foreach DIR,$(INCLUDEDIRS),$(notdir $(wildcard $(DIR)*.h)))
HEADERS := $(HEADERS) $(foreach DIR,$(INCLUDEDIRS),$(notdir $(wildcard $(DIR)*.hpp)))

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

.PRECIOUS: $(TARGET) $(OBJECTS)

$(TARGET): $(OBJECTS)
	$(AR) rcs $(TARGET) $^

clean:
	-rm -f *.o
	-rm -f $(TARGET)

