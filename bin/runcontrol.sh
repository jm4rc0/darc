#!/bin/sh
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
source /Canary/etc/bashrc
source /rtc/etc/rtc.bashrc
echo $PATH
echo $LD_LIBRARY_PATH
#wait for network to initialise...
sleep 10
while [ 1 ];do control.py ; sleep 1 ; done ;
