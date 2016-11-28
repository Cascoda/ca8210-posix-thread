make \
PREFIX=arm-linux-gnueabi- \
CC=$(PREFIX)cc \
LD=$(PREFIX)ld \
CXX=$(PREFIX)g++ \
STRIP=$(PREFIX)strip \
AR=$(PREFIX)ar \
RANLIB=$(PREFIX)ranlib \
AS=$(PREFIX)as \
NM=$(PREFIX)nm \
OBJCOPY=$(PREFIX)objcopy \
OBJDUMP=$(PREFIX)objdump
