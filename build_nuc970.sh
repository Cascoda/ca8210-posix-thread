#!/bin/bash
dir=$(cd -P -- "$(dirname -- "$0")" && pwd -P)
cd $dir

make CC=arm-linux-gcc \
LD=arm-linux-ld \
CXX=arm-linux-g++ \
STRIP=arm-linux-strip \
AR=arm-linux-ar \
RANLIB=arm-linux-ranlib \
AS=arm-linux-as \
NM=arm-linux-nm \
OBJCOPY=arm-linux-objcopy \
OBJDUMP=arm-linux-objdump
