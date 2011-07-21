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

import pyinotify
class WatchStreams(pyinotify.ProcessEvent):
    def __init__(self,prefix="",addcb=None,remcb=None,watchdir="/dev/shm"):
        self.prefix=prefix
        self.start=prefix+"rtc"
        self.startlen=len(self.start)
        self.addcb=addcb
        self.remcb=remcb
        self.mywm=pyinotify.WatchManager()
        self.mywm.add_watch(watchdir,pyinotify.IN_DELETE | pyinotify.IN_CREATE)
        self.notifier=pyinotify.Notifier(self.mywm,self)
    def process_IN_CREATE(self, event):
        if event.name[:self.startlen]==self.start and event.name[-3:]=="Buf":
            if self.addcb!=None:
                self.addcb(event.name)
            else:
                print "Got stream %s"%event.name
    def process_IN_DELETE(self, event):
        if event.name[:self.startlen]==self.start and event.name[-3:]=="Buf":
            if self.remcb!=None:
                self.remcb(event.name)
            else:
                print "Stream removed %s"%event.name
    def myfd(self):
        return self.mywm.get_fd()
    def readEvents(self):#this is blocking, if no events have occurred.
        self.notifier.read_events()
    def processEvents(self):
        self.notifier.process_events()
    def loop(self):#an example of how to use it...
        import select
        while 1:
            rtr=select.select([self.myfd()],[],[])[0]
            self.readEvents()
            self.processEvents()


class WatchDir(pyinotify.ProcessEvent):
    def __init__(self,watchdir="/dev/shm",prefix="",postfix="",addcb=None,remcb=None,modcb=None):
        self.prefix=prefix
        self.postfix=postfix
        self.prelen=len(prefix)
        self.postlen=len(postfix)
        self.addcb=addcb
        self.remcb=remcb
        self.modcb=modcb
        self.mywm=pyinotify.WatchManager()
        flags=pyinotify.IN_DELETE | pyinotify.IN_CREATE
        if modcb!=None:
            flags|=pyinotify.IN_MODIFY
        self.mywm.add_watch(watchdir,flags)
        self.notifier=pyinotify.Notifier(self.mywm,self)

    def addWatchFile(self,fname,cb=None):
        self.mywm.add_watch(fname,pyinotify.IN_MODIFY)

    def process_IN_MODIFY(self,event):
        if event.name[:self.prelen]==self.prefix and (self.postlen==0 or event.name[-self.postlen:]==self.postfix):
            if self.modcb!=None:
                self.modcb(event.name)
            else:
                print "Modified file '%s' (%s)"%(event.name,str(event))
    def process_IN_CREATE(self, event):
        if event.name[:self.prelen]==self.prefix and (self.postlen==0 or event.name[-self.postlen:]==self.postfix):
            if self.addcb!=None:
                self.addcb(event.name)
            else:
                print "Got stream '%s' (%s)"%(event.name,str(event))
    def process_IN_DELETE(self, event):
        if event.name[:self.prelen]==self.prefix and (self.postlen==0 or event.name[-self.postlen:]==self.postfix):
            if self.remcb!=None:
                self.remcb(event.name)
            else:
                print "Stream removed %s"%event.name
    def myfd(self):
        return self.mywm.get_fd()
    def readEvents(self):#this is blocking, if no events have occurred.
        self.notifier.read_events()
    def processEvents(self):
        self.notifier.process_events()
    def loop(self):#an example of how to use it...
        import select
        while 1:
            rtr=select.select([self.myfd()],[],[])[0]
            self.readEvents()
            self.processEvents()
    def handle(self,source=None,cond=None):
        self.readEvents()
        self.processEvents()
        return True


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
