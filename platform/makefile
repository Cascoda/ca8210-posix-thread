TARGET = libraspcascoda.a
LIBS = -lm
CC = gcc
CFLAGS = -g -Wall
INCLUDEDIRS = ./ \
			../openthread/include/platform/ \
			../openthread/include/cli/ \
			../openthread/include/crypto/ \
			../openthread/include/ \
			../cascoda/include/
SOURCEDIR = ./

.PHONY: default all clean

default: $(TARGET)
all: default

OBJECTS = $(patsubst %.c, %.o, $(wildcard $(addprefix $(SOURCEDIR),*.c)))
HEADERS = foreach(DIR,$(SOURCEDIR),$(wildcard $(addprefix $(DIR),*.h)))

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

.PRECIOUS: $(TARGET) $(OBJECTS)

$(TARGET): $(OBJECTS)
	ar rcs $(TARGET) $^

clean:
	-rm -f *.o
	-rm -f $(TARGET)