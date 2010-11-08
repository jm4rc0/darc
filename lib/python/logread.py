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
class logread:
    def __init__(self,name=None,txtlim=1024*80,tag="",callback=None):
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
        self.sleep=0
    def loop(self):

        while self.go:
            while self.sleep:
                time.sleep(10)
                #if self.name=="/dev/shm/stdout0":
                #    print "logread sleep"
            #if self.name=="/dev/shm/stdout0":
            #    print "logread awake %s"%self.name
            while self.fd==None and self.go==1:
                try:#wait for the file to exist...
                    self.fd=open(self.name)#"/dev/shm/%sstdout0"%self.tag)
                except:
                    time.sleep(10)
            if self.go==0:
                break
            ctime=os.fstat(self.fd.fileno()).st_ctime
            data=self.fd.read()
            if len(data)>0:
                #print "logread got:"
                #print data
                if self.callback!=None:
                    self.callback(data)
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
                    time.sleep(5)
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

if __name__=="__main__":
    l=logread()
    l.loop()
