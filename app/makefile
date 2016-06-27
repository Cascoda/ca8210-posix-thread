TARGET = cliapp
LIBDIRS = ../openthread/src/cli/ ../openthread/src/core/ ../cascoda/ ../platform/ ../openthread/third_party/mbedtls/
LIBSEARCH := $(addprefix -L,$(LIBDIRS))
LIBS = -lm -lcascoda -lopenthread -lopenthread-cli -lraspcascoda -lmbedcrypto -lstdc++
CC = gcc
INCLUDEDIRS := ./\
	../openthread/include/cli/\
	../openthread/include/\
	./include/\
	../platform/
INCLUDEDIRCMD := $(addprefix -I,$(INCLUDEDIRS)) 
CFLAGS := $(INCLUDEDIRCMD) -g -Wall

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
