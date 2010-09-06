#!/bin/sh
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
source /Canary/etc/bashrc
source /rtc/etc/rtc.bashrc
screen -d -m runcontrol.sh
screen -d -m runsend.sh
screen -d -m
