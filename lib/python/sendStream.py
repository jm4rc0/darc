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

import time
import buffer
import sys
import socket
import serialise
import numpy
#import recvStream
import Saver
class Send:
    def __init__(self,host,port,shmname,prefix,debug,log,raw,connect=1,attempts=-1,startWithLatest=0,decimate=1):
        self.host=host
        self.port=port
        self.startWithLatest=startWithLatest
        self.shmname=shmname
        self.prefix=prefix
        self.shmOpen=0
        self.sock=None
        self.circbuf=None
        self.debug=debug
        self.raw=raw#if 1, send in raw mode, not serialise.
        self.saver=None
        self.attempts=attempts
        self.cumfreq=0
        self.decimate=decimate
        if log:
            self.saver=Saver.Saver(shmname+".log")
            
        self.openSHM()
        if connect:
            self.connectSock()
        self.go=1
    def connectSock(self):
        self.sock=None
        while self.sock==None and self.attempts!=0:
            if self.attempts>0:
                self.attempts-=1
            self.sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            try:
                self.sock.connect((self.host,self.port))
                print "Connected to Receiver"
            except:
                print "Couldn't connect to receiver on %s %d"%(self.host,self.port)
                self.sock=None
                time.sleep(1)
        if self.sock==None:
            sys.exit(0)
        serialise.Send(["name",self.prefix+self.shmname],self.sock)
        print "name sent %s"%(self.prefix+self.shmname)
        if self.raw:#inform that we'll be sending raw data...
            print "Sending raw flag"
            serialise.Send(["raw","raw",self.prefix+self.shmname],self.sock)

    def openSHM(self):
        self.shmOpen=0
        while self.shmOpen==0:
            try:
                self.circbuf=buffer.Circular("/"+self.prefix+self.shmname,raw=self.raw)
                self.shmOpen=1
            except:
                print "Failed to open /dev/shm/%s%s"%(self.prefix,self.shmname)
                self.circbuf=None
                time.sleep(1)
        print "/dev/shm/%s%s opened"%(self.prefix,self.shmname)

    def checkSHM(self):
        """Check to see whether open shm is same as newly opened shm.
        Return 1 on failure - ie if not the same, or if not openable.
        """
        if self.circbuf==None:
            return True
        try:
            shm=numpy.memmap("/dev/shm/"+self.prefix+self.shmname,"b","r",shape=(buffer.getCircHeaderSize(),))
        except:#shm doesn't exist any more.
            print "Error opening shm"
            return True
        if self.circbuf.compare(shm):
            #They are the same.
            return False
        return True
        

    def loop(self):
        """Waits for data, and sends it over socket."""
        #first, open the buffer...
        self.cumfreq=self.decimate
        if self.startWithLatest:
            ret=self.circbuf.getLatest()
        while self.go:
            #wait for data to be ready
            #print "Waiting for data"
            ret=self.circbuf.getNextFrame(timeout=10,copy=1)
            if self.debug:
                if ret==None:
                    print "No data yet %s"%(self.prefix+self.shmname)
                else:
                    print "Got data %s"%(self.prefix+self.shmname)
                    
            #if key=="rtcTimeBuf":
            #    ret=b.data[:,0],ret[1],ret[2]

            #How can I tell if a timeout occurred because rtc is dead/restarted or because its paused/not sending?
            #First, open the shm in a new array, and look at header.  If header is same as existing, don't do anything else.  Otherwise, reopen and reinitialise the circular buffer.
            if ret==None:
                if self.checkSHM():#returns 1 on failure...
                    print "Reopening SHM"
                    self.openSHM()
                    ret=self.circbuf.getNextFrame(timeout=10,copy=1)
                else:
                    #shm still valid - probably timeout occurred, meaning RTC still dead, or just not producing this stream.
                    pass
            #Check to see if we're lagging behind the RTC - if so, send the latest frame...
            lw=self.circbuf.lastWritten[0]
            if lw>=0:
                diff=lw-self.circbuf.lastReceived
                if diff<0:
                    diff+=self.circbuf.nstore[0]
                if diff>self.circbuf.nstore[0]*.75:
                    print "Sending of %s lagging - skipping %d frames"%(self.prefix+self.shmname,diff-1)
                    ret=self.circbuf.get(lw,copy=1)

            if ret!=None:
                cbfreq=int(self.circbuf.freq[0])
                if cbfreq<1:
                    cbfreq=1
                self.cumfreq+=cbfreq
                self.cumfreq-=self.cumfreq%cbfreq
                if self.cumfreq>=self.decimate:#so now send the data
                    self.cumfreq=0

                    #data,timestamp,frameno=ret
                    #print "got data at %s %s %s"%(str(timestamp),str(data.shape),str(data.dtype.char))
                    if self.saver!=None:
                        if self.raw:
                            self.saver.writeRaw(ret)
                        else:
                            data,ftime,fno=ret
                            self.saver.write(data,ftime,fno)

                    #send the data
                    #print "sending",self.sock
                    if self.sock!=None:
                        try:
                            if self.debug:
                                print "Sending %s"%(self.prefix+self.shmname)
                            if self.raw:
                                try:
                                    self.sock.sendall(ret)
                                except:
                                    print "Error in sendStream with raw stream - sock.sendall failed - couldn't send raw data"
                                    raise
                            else:
                                serialise.Send(["data",self.prefix+self.shmname,ret],self.sock)
                        except:
                            #self.connectSock()
                            print "error in serialise.Send - exiting - finishing sending of %s"%(self.prefix+self.shmname)
                            self.go=0
                            #raise
        if self.saver!=None:
            self.saver.close()
if __name__=="__main__":
    setprio=0
    affin=0x7fffffff
    prio=0
    shmprefix=""
    host=socket.gethostname()
    port=4243
    debug=0
    log=0
    raw=0
    connect=1
    tries=-1
    startWithLatest=0
    decimate=1
    for arg in sys.argv[1:]:
        if arg[:2]=="-p":
            port=int(arg[2:])
        elif arg[:2]=="-h":
            host=arg[2:]
        elif arg[:2]=="-a":#affinity
            affin=int(arg[2:])
            setprio=1
        elif arg[:2]=="-i":#importance
            prio=int(arg[2:])
            setprio=1
        elif arg[:2]=="-s":#shm prefix
            shmprefix=arg[2:]
        elif arg[:2]=="-v":#debug
            debug=1
        elif arg[:2]=="-d":#decimate
            decimate=int(arg[2:])
        elif arg[:2]=="-l":#log locally (into streamname.log)
            log=1
        elif arg[:2]=="-r":#send in raw mode...
            raw=1
        elif arg[:2]=="-c":#don't connect
            connect=0
        elif arg[:2]=="-t":#number of connection tries...
            tries=int(arg[2:])
        elif arg[:2]=="-n":
            startWithLatest=1
        else:
            streamname=arg#eg rtcPxlBuf
    if setprio:
        try:
            import utils
            utils.setAffinityAndPriority(affin,prio)
        except:
            print "Error setting affinity and priority to %x and %d"%(affin,prio)
            
    s=Send(host,port,streamname,shmprefix,debug,log,raw,connect,attempts=tries,startWithLatest=startWithLatest,decimate=decimate)
    s.loop()
