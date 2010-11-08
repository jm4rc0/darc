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
"""Code to produce a dummy camera image and send over a socket.
"""
import SockConn
import time
import numpy
import sys
import FITS

class socketCam:
    def __init__(self,host,port,fname,stime):
        self.port=port
        self.host=host
        self.fname=fname
        self.data=FITS.Read(fname)[1].astype(numpy.uint16)
        self.dpos=0
        self.fno=0
        self.stime=stime
        self.globals=globals()
        self.hdr=numpy.zeros((2,),numpy.uint32)
        self.npxls=reduce(lambda x,y:x*y,self.data.shape[1:])
        self.hdr[0]=(0x55<<24) | self.npxls
        print "Header:",hex(self.hdr[0])
        hz=0
        if self.stime>0:
            hz=1./stime
        print "sleep time: %g (%g Hz)"%(self.stime,hz)
        print "Data: %s %s"%(self.data.shape,self.data.dtype.char)
        self.sockConn=SockConn.SockConn(self.port,host=self.host,globals=self.globals,startThread=1,listenSTDIN=0,connectFunc=self.newConnection)


    def newConnection(self,s,suberr=0):
        print "New connection"


    def main(self):
        print "Entering main loop"
        while 1:
            d=self.data[self.dpos]
            self.hdr[1]=self.fno
            for s in self.sockConn.selIn:
                if s!=self.sockConn.lsock:
                    #print "Sending",s,self.fno,self.hdr
                    try:
                        s.sendall(self.hdr.tostring())
                    except:
                        print "Error sending"
                    try:
                        s.sendall(d)
                    except:
                        print "Error sending2"
            self.fno+=1
            self.dpos=self.fno%self.data.shape[0]
            time.sleep(self.stime)
            
if __name__=="__main__":
    if len(sys.argv)!=5:
        print "Usage: %s host port fname stime"%sys.argv[0]
        sys.exit(0)
    host=sys.argv[1]
    port=int(sys.argv[2])
    fname=sys.argv[3]
    stime=float(sys.argv[4])
    sc=socketCam(host,port,fname,stime)
    sc.main()
                
             
