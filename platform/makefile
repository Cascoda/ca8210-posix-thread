TARGET = libotplatposixca8210.a
LIBS = -lm
CC = gcc
INCLUDEDIRS := ./\
	../openthread/include/platform/\
	../openthread/include/cli/\
	../openthread/include/crypto/\
	../openthread/include/\
	../ca8210-kernel-exchange/cascoda-api/include/\
	../ca8210-kernel-exchange/\
	../openthread/src/core/\
	../openthread/src/core/common/
INCLUDEDIRCMD := $(addprefix -I,$(INCLUDEDIRS)) 
SOURCEDIR = ./
CFLAGS := $(INCLUDEDIRCMD) -g -Wall -std=c99

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

