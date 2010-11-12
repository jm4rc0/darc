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

import os
import time
import numpy

import buffer
import controlCorba

class nodeStream:
    """A class which receives a darc telemetry stream, and places it in a shm circular buffer array on local node"""
    def __init__(self,name,prefix="",myhostname=None,testprefix=""):
        """Initialise the object.  name is the name of the stream, prefix is the darc prefix.  myhostname can be an IP address (but will be found automagically on linux), and testprefix can be used if you want to add an additional prefix to the shm buffer."""
        self.name=name
        self.decimate=100
        self.prefix=prefix
        self.testprefix=testprefix
        self.nstore=100
        self.myhostname=myhostname
        #self.lock=lock.lock()
        self.ctrl=controlCorba.controlClient(debug=0,controlName=prefix)
        while self.ctrl.obj==None:
            self.ctrl=controlCorba.controlClient(debug=0,controlName=prefix)
            if self.ctrl.obj==None:
                time.sleep(1)
        self.cb=None
        self.timeDataLastRequested=0
        if self.checkSHM():
            #The assumption here is that this shm is old and not written by anyone else...
            self.cb=buffer.Circular("/"+self.testprefix+self.prefix+self.name)
            self.cb.lastWritten[0]=-1


    def run(self):
        self.blockCallback=controlCorba.blockCallback([self.prefix+self.name],-1,callback=self.callback)
        self.r=self.ctrl.Subscribe([self.prefix+self.name],self.blockCallback.call,decimate=self.decimate,host=self.myhostname,verbose=0,sendFromHead=0,startthread=0,timeout=0.2,timeoutFunc=self.setDecimate)
        #self.r.d.sockConn.verbose=1
        self.r.d.loop()#enter the main loop.

        #self.ctrl.GetStreamBlock([self.name],-1,decimate=self.decimate,callback=self.callback,printstatus=0)
        #while 1:
            #check decimate rate... if it has changed, do soemthing.
        #    self.setDecimate()
        #    time.sleep(0.2)

    def setDecimate(self):
        #self.lock.acquire()
        if self.cb!=None and self.cb.freq[0]!=self.decimate:
            self.decimate=int(self.cb.freq[0])
            self.timeDataLastRequested=0
            print "Setting decimate to %d"%self.decimate
            self.r.d.sockConn.userSelList[0].sendall(numpy.array([self.decimate]).astype(numpy.int32).tostring())
        #self.lock.release()

    def checkSHM(self):
        """If the shm exists, just open this and use it."""
        return os.path.exists("/dev/shm/%s%s%s"%(self.testprefix,self.prefix,self.name))

    def callback(self,data):
        """data[0]=="data",data[1]==name,data[2]==data,timestamp,fno
        """
        d=data[2][0]
        timestamp=data[2][1]
        frameno=data[2][2]
        setdec=0
        print "Got data for frame ",frameno
        #Put the data into the shm array.
        if self.cb==None:
            #open the buffer to write into
            print "nodestream creating circular buffer %s: shape %s, type %s"%(self.testprefix+self.prefix+self.name,str(d.shape),d.dtype.char)
            self.cb=buffer.Circular("/"+self.testprefix+self.prefix+self.name,owner=1,dims=d.shape,dtype=d.dtype.char,nstore=self.nstore)
            #self.owner=0#so that won't be removed if this process quits.
        #check the shape of the buffer
        if self.cb.getShape()!=d.shape or self.cb.dtype[0]!=d.dtype.char:
            print "nodestream reshaping"
            self.cb.reshape(d.shape,d.dtype.char)
        #and write the data.

        if self.cb.circsignal[0]!=0:
            self.timeDataLastRequested=timestamp
            self.cb.circsignal[0]=0
        elif self.timeDataLastRequested>0 and timestamp-self.timeDataLastRequested>60:
            #data hasn't been requested for last minute
            self.timeDataLastRequested=timestamp
            #Turn off the decimation of sender.
            print "No requests for last minute - nodestream turning off."
            self.cb.freq[0]=0
        elif self.timeDataLastRequested==0:
            self.timeDataLastRequested=timestamp
        self.setDecimate()
        self.cb.add(d,timestamp,frameno)
        return 0
if __name__=="__main__":
    import sys
    name=sys.argv[1]
    prefix=""
    testprefix=""
    if len(sys.argv)>2:
        prefix=sys.argv[2]
    if len(sys.argv)>3:
        testprefix=sys.argv[3]
    ns=nodeStream(name,prefix,testprefix=testprefix)
    ns.run()
