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
import select
import traceback
import time
import stat
import os
import threading
import socket
class logread:
    def __init__(self,name=None,txtlim=1024*80,tag="",callback=None,sleeptime=5):
        if name==None:
            name="/dev/shm/%sstdout0"%tag
        self.name=name
        self.go=1
        self.txt=""
        self.txtlim=txtlim
        self.lock=threading.Lock()
        #self.tag=tag
        self.callback=callback
        self.fd=None
        self.sock=None
        self.sleep=0
        self.sleeptime=sleeptime
    def loop(self):
        if os.stat(self.name).st_size<self.txtlim:
            #open the prev logfile...
            if os.path.exists(self.name):
                self.txt=open(self.name).read()
                if len(self.txt)>self.txtlim:
                    self.txt=self.txt[-self.txtlim:]
                if self.sock!=None:#send initial stuff to the socket
                    self.sock.send(self.txt)
        while self.go:
            while self.sleep:
                time.sleep(self.sleeptime*2)
                #if self.name=="/dev/shm/stdout0":
                #    print "logread sleep"
            #if self.name=="/dev/shm/stdout0":
            #    print "logread awake %s"%self.name
            while self.fd==None and self.go==1:
                try:#wait for the file to exist...
                    self.fd=open(self.name)#"/dev/shm/%sstdout0"%self.tag)
                except:
                    time.sleep(self.sleeptime*2)
            if self.go==0:
                break
            ctime=os.fstat(self.fd.fileno()).st_ctime
            data=self.fd.read()
            if len(data)>0:
                #print "logread got:"
                #print data
                if self.callback!=None:
                    if self.callback(data)==1:
                        self.go=0
                self.lock.acquire()
                self.txt+=data
                if len(self.txt)>self.txtlim:
                    self.txt=self.txt[-self.txtlim:]
                self.lock.release()
                #data=data.split("\n")
                #print "got data",len(data)
                #for d in data:
                #    if d!="reconFrameFinished" and len(data)>0:
                #        print d
            else:
                try:
                    ntime=os.stat(self.name).st_ctime#"/dev/shm/%sstdout0"%self.tag).st_ctime
                except:
                    self.fd.close()
                    self.fd=None
                    ntime=ctime
                    print "Couldn't stat %s"%self.name#/dev/shm/%sstdout0"%self.tag
                if ntime==ctime:
                    #print "sleep at %s (tell=%d)"%(time.strftime("%H%M%S"),self.fd.tell())
                    time.sleep(self.sleeptime)
                else:#new creation time... so reopen
                    #print "reopen"
                    try:
                        self.fd=open(self.name)#"/dev/shm/%sstdout0"%self.tag)
                        ctime=os.fstat(self.fd.fileno()).st_ctime
                    except:
                        self.fd=None

    def getTxt(self):
        self.lock.acquire()
        t=self.txt
        self.lock.release()
        return t

    def launch(self):
        self.thread=threading.Thread(target=self.loop)
        self.thread.setDaemon(1)
        self.thread.start()

    def connect(self,host,port):
        """Connect to host,port and send log data there"""
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.sock.connect((host,port))
        print "logread connected to %s %d"%(host,port)
        self.usercallback=self.callback
        self.callback=self.sockcallback
        txt=self.getTxt()
        if len(txt)>self.txtlim:
            txt=txt[-self.txtlim:]
        self.sock.send(txt)
    def sockcallback(self,data):
        err=0
        if self.usercallback!=None:
            if self.usercallback(data)==1:
                err=1
        if err==0:
            try:
                self.sock.send(data)
            except:
                self.sock.close()
                self.sock=None
                if self.usercallback!=None:
                    self.callback=self.usercallback
                else:
                    err=1
                    self.callback=None
        return err
    def pr(self,data):
        print data
        return 0

if __name__=="__main__":
    import sys
    print sys.argv
    name=None
    txtlim=1024*80
    tag=""
    sleeptime=5
    host=None
    port=None
    if len(sys.argv)>1:
        name=sys.argv[1]
    if len(sys.argv)>2:
        txtlim=int(sys.argv[2])
    if len(sys.argv)>3:
        sleeptime=float(sys.argv[3])
    if len(sys.argv)>4:
        host=sys.argv[4]
    if len(sys.argv)>5:
        port=int(sys.argv[5])
    l=logread(name=name,txtlim=txtlim,tag=tag,sleeptime=sleeptime)
    if host!=None:
        l.connect(host,port)
    else:
        l.callback=l.pr
    l.loop()
