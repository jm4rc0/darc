#!/usr/bin/env python
#darc, the Durham Adaptive optics Real-time Controller.
#Copyright (C) 2010 Alastair Basden.

#This program is free software: you can redistribute it and/or modify
#it under the terms of the GNU Affero General Public License as
#published by the Free Software Foundation, either version 3 of the
#License, or (at your option) any later version.

#This program is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU Affero General Public License for more details.

#You should have received a copy of the GNU Affero General Public License
#along with this program.  If not, see <http://www.gnu.org/licenses/>.
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

def watchShm(prefix):
    import pyinotify
    import select
    wm=pyinotify.WatchManager()
    mask=pyinotify.IN_DELETE | pyinotify.IN_CREATE
    wm.add_watch("/dev/shm",mask)
    class PTmp(pyinotify.ProcessEvent):
        def process_IN_CREATE(self, event):
            print "Create: %s" %  os.path.join(event.path, event.name)
        def process_IN_DELETE(self, event):
            print "Remove: %s" %  os.path.join(event.path, event.name)
    PTmpInst=PTmp()
    notifier=pyinotify.Notifier(wm,PTmpInst)
    #notifier.loop()
    while 1:
        rtr=select.select([wm.get_fd()],[],[])[0]
        print rtr
        for r in rtr:
            if r==wm.get_fd():
                # if notifier.check_events():#not needed - select does this...
                notifier.read_events()
                ev=notifier.process_events()#this calls the PTmp methods...

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
