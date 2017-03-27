TARGET = cliapp
LIBDIRS = ../openthread/src/cli/ ../ ../openthread/third_party/mbedtls/
LIBSEARCH := $(addprefix -L,$(LIBDIRS))
LIBS = -lm -lotposixca8210-ftd -lopenthread-cli -lstdc++ -lpthread
CC = gcc
INCLUDEDIRS := ./\
	../openthread/include/\
	../openthread/include/openthread/\
	../openthread/src/core/\
	./include/\
	../platform/\
	../ca8210-kernel-exchange/cascoda-api/include/\
	../ca8210-kernel-exchange/
INCLUDEDIRCMD := $(addprefix -I,$(INCLUDEDIRS)) 
CFLAGS := $(INCLUDEDIRCMD) -g -Wall -pthread

.PHONY: default all clean

default: $(TARGET)
all: default

OBJECTS = $(patsubst %.c, %.o, $(wildcard *.c))
HEADERS := $(foreach DIR,$(INCLUDEDIRS),$(notdir $(wildcard $(DIR)*.h)))

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

.PRECIOUS: $(TARGET) $(OBJECTS)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -Wall $(LIBSEARCH) $(LIBS) -o $@

clean:
	-rm -f *.o
	-rm -f $(TARGET)
