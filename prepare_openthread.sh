#!/bin/bash
dir=$(cd -P -- "$(dirname -- "$0")" && pwd -P)
cd $dir

cd openthread/
make distclean
./bootstrap
./configure --enable-cli-app=all --enable-application-coap --enable-joiner --enable-commissioner --enable-tmf-proxy --enable-mac-filter --enable-cert-log --enable-dhcp6-client --enable-dhcp6-server --enable-border-router
cd ../
