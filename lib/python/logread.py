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

from startStreams import WatchDir
class logread:
    def __init__(self,name=None,txtlim=1024*80,tag="",callback=None,sleeptime=5,watchedDir="/dev/shm/"):
        self.prefix=tag
        self.watchedDir=watchedDir
        self.nameDict={}
        self.wm=WatchDir(self.watchedDir,self.prefix+"rtc","Stdout0",self.logAdded,self.logRemoved,self.logModified)
        if name==None:#want to subscribe to all current and future logs...
            #name="/dev/shm/%srtcStdout0"%tag
            self.openNewLogs=1
        else:
            self.openNewLogs=0
            if type(name)==type(""):
                if self.watchedDir==name[:len(self.watchedDir)]:
                    name=name[len(self.watchedDir):]
                self.nameDict={name:None}
            elif type(name) in [type(()), type([])]:
                for n in name:
                    if n[:len(self.watchedDir)]==self.watchedDir:
                        n=n[len(self.watchedDir):]
                    self.nameDict[n]=None
        self.go=1
        self.txt=""
        self.txtlim=txtlim
        self.lock=threading.Lock()
        self.callback=callback
        self.fd=None
        self.sock=None
        self.sleep=0
        self.sleeptime=sleeptime
        self.tagrtc=tag+"rtc"
        self.lentagrtc=len(self.tagrtc)

    
    def logAdded(self,fname):
        """Called when a new logfile is added, or when logrotation is done"""
        if self.openNewLogs:
            if self.nameDict.has_key(fname) and self.nameDict[fname]!=None:
                self.savedTxt+=self.nameDict[fname].read()
            try:
                self.nameDict[fname]=open(self.watchedDir+fname)
            except:
                pass
        else:#only reopen if its one we're watching...
            if self.nameDict.has_key(fname):
                if self.nameDict[fname]!=None:
                    self.savedTxt+=self.nameDict[fname].read()
                self.nameDict[fname]=open(self.watchedDir+fname)
    def logRemoved(self,fname):
        #Only remove the log if we're opening new logs - otherwise we'll lose track of what we want to be logging...
        if self.openNewLogs:
            if self.nameDict.has_key(fname):
                del(self.nameDict[fname])
    def logModified(self,fname):
        if self.nameDict.has_key(fname) and self.nameDict[fname]!=None:
            data=self.nameDict[fname].read()
            if len(data)>0:
                if self.callback!=None:
                    if self.callback(data)==1:
                        self.go=0
                self.lock.acquire()
                self.txt+=data
                if len(self.txt)>self.txtlim:
                    self.txt=self.txt[-self.txtlim:]
                self.lock.release()
            else:
                pass


            
    def addNewLogs(self):
        if self.openNewLogs:
            files=os.listdir(self.watchedDir)
            for f in self.nameDict.keys():
                if f not in files:
                    del(self.nameDict[f])
            names=self.nameDict.keys()
            for f in files:
                if f[:self.lentagrtc]==self.tagrtc and f[-7:]=="Stdout0" and f not in names:
                    self.nameDict[f]=None



    def loop(self):
        self.addNewLogs()
        #first open the logs, and read the streams...
        for name in self.nameDict.keys():
            if os.path.exists(self.watchedDir+name):
                fd=open(self.watchedDir+name)
                self.nameDict[name]=fd
                if os.fstat(fd.fileno()).st_size<self.txtlim:
                    txt=fd.read()
                else:
                    fd.seek(-self.txtlim,2)
                    txt=fd.read()
                if len(txt)>0:
                    self.txt+="******* %s ********\n%s"%(name,txt)
        
        if len(self.txt)>0 and self.sock!=None:#send initial stuff to the socket
            self.sock.send(self.txt)
        while self.go:
            rtr=[self.wm.myfd()]
            #for key in self.nameDict.keys():
            #    if self.nameDict[key]!=None:
            #        rtr.append(self.nameDict[key])
            rtr,rtw,err=select.select(rtr,[],[])
            # print rtr
            # for f in self.nameDict.keys():
            #     fd=self.nameDict[f]
            #     if fd in rtr:#read the log...
            #         print "Reading..."
            #         data=fd.read()
            #         if len(data)>0:
            #             if self.callback!=None:
            #                 if self.callback(data)==1:
            #                     self.go=0
            #             self.lock.acquire()
            #             self.txt+=data
            #             if len(self.txt)>self.txtlim:
            #                 self.txt=self.txt[-self.txtlim:]
            #             self.lock.release()
            #         else:
            #             pass
            #now see if any new logs have been added...
            if self.wm.myfd() in rtr:
                self.savedTxt=""
                self.wm.handle()#add the files
                if len(self.savedTxt)>0:
                    if self.callback!=None:
                        if self.callback(self.savedTxt)==1:
                            self.go=0




            # while self.sleep:
            #     time.sleep(self.sleeptime*2)
            # while self.go:
            #     if self.checkNew:
            #         self.addNewLogs()
            #     cnt=0
            #     for f in self.nameDict.keys():
            #         fd=self.nameDict[f]
            #         if fd==None:
            #             try:
            #                 fd==open(f)
            #                 self.nameDict[f]=fd
            #             except:
            #                 pass
            #         if fd!=None:
            #             cnt+=1
            #             ctime=os.fstat(fd.fileno()).st_ctime
            #             data=fd.read()
                        

            #     if cnt==0:
            #         time.sleep(self.sleeptime*2)
            # while self.fd==None and self.go==1:
            #     try:#wait for the file to exist...
            #         self.fd=open(self.nameList[0])#"/dev/shm/%sstdout0"%self.tag)
            #     except:
            #         time.sleep(self.sleeptime*2)
            # if self.go==0:
            #     break
            # ctime=os.fstat(self.fd.fileno()).st_ctime
            # data=self.fd.read()
            # if len(data)>0:
            #     if self.callback!=None:
            #         if self.callback(data)==1:
            #             self.go=0
            #     self.lock.acquire()
            #     self.txt+=data
            #     if len(self.txt)>self.txtlim:
            #         self.txt=self.txt[-self.txtlim:]
            #     self.lock.release()
            # else:
            #     try:
            #         ntime=os.stat(self.nameList[0]).st_ctime#"/dev/shm/%sstdout0"%self.tag).st_ctime
            #     except:
            #         self.fd.close()
            #         self.fd=None
            #         ntime=ctime
            #         print "Couldn't stat %s"%self.nameList[0]
            #     if ntime==ctime:
            #         #print "sleep at %s (tell=%d)"%(time.strftime("%H%M%S"),self.fd.tell())
            #         time.sleep(self.sleeptime)
            #     else:#new creation time... so reopen
            #         #print "reopen"
            #         try:
            #             self.fd=open(self.nameList[0])
            #             ctime=os.fstat(self.fd.fileno()).st_ctime
            #         except:
            #             self.fd=None

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
        #print "logread connected to %s %d"%(host,port)
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
    #print sys.argv
    name=None
    txtlim=1024*80
    tag=""
    sleeptime=5
    host=None
    port=None
    if len(sys.argv)>1:
        name=sys.argv[1]
        if name=="ALL":
            name=None
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
