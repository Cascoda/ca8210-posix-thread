TARGET = cliapp
LIBDIRS = ../openthread/src/cli/ ../openthread/src/core/ ../cascoda/ ../platform/
LIBSEARCH = foreach(CMD,$(LIBDIRS),$(addprefix -L,$(CMD)))
LIBS = -lm -lcascoda -lopenthread -lopenthread-cli -lraspcascoda
CC = gcc
CFLAGS = -g -Wall

.PHONY: default all clean

default: $(TARGET)
all: default

OBJECTS = $(patsubst %.c, %.o, $(wildcard *.c))
HEADERS = $(wildcard include/*.h) \
			$(wildcard ../openthread/include/*.h) \
			$(wildcard ../openthread/include/cli/*.h) \
			$(wildcard ../platform/*.h)

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

.PRECIOUS: $(TARGET) $(OBJECTS)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -Wall $(LIBSEARCH) $(LIBS) -o $@

clean:
	-rm -f *.o
	-rm -f $(TARGET)
