TARGET = cascoda.a
LIBS = -lm
CC = gcc
CFLAGS = -g -Wall
INCLUDEDIR = include/
SOURCEDIR = source/

.PHONY: default all clean

default: $(TARGET)
all: default

OBJECTS = $(addprefix $(SOURCEDIR), $(patsubst %.c, %.o, $(wildcard *.c)))
HEADERS = $(addprefix $(INCLUDEDIR), $(wildcard *.h))

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

.PRECIOUS: $(TARGET) $(OBJECTS)

$(TARGET): $(OBJECTS)
	ar rcs $(TARGET) $^

clean:
	-rm -f *.o
	-rm -f $(TARGET)