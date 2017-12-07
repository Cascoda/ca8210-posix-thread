#!/bin/bash
dir=$(cd -P -- "$(dirname -- "$0")" && pwd -P)

cd openthread/
make distclean
./bootstrap
./configure --enable-cli-app=all --enable-application-coap --enable-joiner --enable-commissioner --enable-ftd
cd ../
