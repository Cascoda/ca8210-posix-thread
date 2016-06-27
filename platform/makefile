TARGET = libraspcascoda.a
LIBS = -lm
CC = gcc
INCLUDEDIRS := ./\
	../openthread/include/platform/\
	../openthread/include/cli/\
	../openthread/include/crypto/\
	../openthread/include/\
	../cascoda/include/\
	../openthread/src/core/\
	../openthread/src/core/common/
INCLUDEDIRCMD := $(addprefix -I,$(INCLUDEDIRS)) 
SOURCEDIR = ./
CFLAGS := $(INCLUDEDIRCMD) -g -Wall

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
	ar rcs $(TARGET) $^

clean:
	-rm -f *.o
	-rm -f $(TARGET)
