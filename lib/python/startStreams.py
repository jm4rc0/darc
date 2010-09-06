#!/usr/bin/env python
"""Code to start streams... looks at /dev/shm to see which streams are available, and then starts a sendStream program for each.
"""
import sys
import os
import string
import subprocess
import time
def getStreams(prefix):#Thos is also used by control.py
    """Streams are labelled in shm with name prefix+rtc*Buf where * is the stream name."""
    files=os.listdir("/dev/shm")
    start=prefix+"rtc"
    lstart=len(start)
    s=[]
    for file in files:
        if file[:lstart]==start and file[-3:]=="Buf":
            s.append(file)
    return s



if __name__=="__main__":
    affin=None
    prio=None
    shmprefix=""
    host=None
    port=None
    cmdList=["sendStream.py"]
    for arg in sys.argv[1:]:
        if arg[:2]=="-p":#port
            cmdList.append(arg)
        elif arg[:2]=="-h":#host
            cmdList.append(arg)
        elif arg[:2]=="-a":#affinity
            cmdList.append(arg)
        elif arg[:2]=="-i":#importance
            cmdList.append(arg)
        elif arg[:2]=="-s":#shm prefix
            cmdList.append(arg)
            shmprefix=arg[2:]
        elif arg[:2]=="-d":#debug
            cmdList.append(arg)
        else:
            print "Usage: %s -pport -hhost -aaffinity -ipriority -sshmprefix"%sys.argv[0]
    streams=getStreams(shmprefix)
    print streams
    #plist=[]
    pdir={}
    for stream in streams:
        cmd=cmdList+[stream[len(shmprefix):]]
        print "Executing %s"%string.join(cmd," ")
        #plist.append(subprocess.Popen(cmd))
        pdir[stream]=subprocess.Popen(cmd)

    try:
        while 1:
            for k in pdir.keys():
                if pdir[k].poll()!=None:#terminated
                    del(pdir[k])
            streams=getStreams(shmprefix)
            keys=pdir.keys()
            for stream in streams:
                if stream not in keys:
                    cmd=cmdList+[stream[len(shmprefix):]]
                    print "Executing %s"%string.join(cmd," ")
                    pdir[stream]=subprocess.Popen(cmd)
            time.sleep(1)
    except:
        print sys.exc_info()
        print "Keyboard interrupt - killing streams"
        for k in pdir.keys():
            p=pdir[k]
            print "Killing",p,p.pid
            p.kill()

#    try:
#        for p in plist:
#            p.wait()
#    except KeyboardInterrupt:
#        print "Keyboard interrupt - killing streams"
#        for p in plist:
#            print "Killing",p,p.pid
#            p.kill()
