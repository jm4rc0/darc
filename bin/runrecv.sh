#!/bin/bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
source /Canary/etc/bashrc
source /rtc/etc/rtc.bashrc
while [ 1 ]; do recvStream.py -hts ; sleep 1 ; done;
