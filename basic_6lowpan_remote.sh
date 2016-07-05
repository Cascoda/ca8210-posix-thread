#!/bin/bash

# set panid
ifconfig wpan0 down
echo "setting panid"
iwpan dev wpan0 set pan_id 0xca5c
sleep 1 

# configure 6LoWPAN interface
ifconfig lowpan0
echo "attaching lowpan0 to wpan0"
ip link add link wpan0 name lowpan0 type lowpan
sleep 1

# configure IP address
echo "setting ip address to ca5c:0da0::1"
ip addr add 2001:db8:ca5c:0da0::2/64 dev lowpan0
sleep 1

# bring up the interfaces
echo "bringing up interfaces"
ifconfig wpan0 up
sleep 1
ifconfig lowpan0 up && \
sleep 1

echo "ready for ping"