TARGET = cascoda.a
LIBS = -lm
CC = gcc
CFLAGS = -g -Wall

.PHONY: default all clean

default: $(TARGET)
all: default

OBJECTS = source/$(patsubst %.c, %.o, $(wildcard *.c))
HEADERS = include/$(wildcard *.h)

%.o: %.c $(HEADERS)
    $(CC) $(CFLAGS) -c $< -o $@

.PRECIOUS: $(TARGET) $(OBJECTS)

$(TARGET): $(OBJECTS)
    ar rcs $(TARGET) $^

clean:
    -rm -f *.o
    -rm -f $(TARGET)