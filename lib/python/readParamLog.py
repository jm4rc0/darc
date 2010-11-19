import numpy
import buffer
import serialise

class readParamLog:
    """Used for reading a parameter log as created by
    darcmagic param -file=param.ser
    """
    def __init__(self,fname,bufsize=64*1024*1024):
        self.fname=fname
        self.lastfno=0
        self.lasttstamp=0.
        self.fd=open(self.fname)
        self.buf=buffer.Buffer(None,size=bufsize)
        self.savedData=None
    def getState(self,fno=None,tstamp=None):
        """Get the state of the rtc at frame number fno, or at time tstamp"""
        if fno==None and tstamp==None:
            raise Exception("getState must specify a frame or time at which to get the state")
        if fno!=None:
            if self.lastfno>fno:
                #Have to reset and start from the start
                print "Rewinding"
                self.fd.close()
                self.fd=open(self.fname)
            elif self.lastfno==fno:
                return self.buf.copy()
        elif tstamp!=None:
            if self.lasttstamp>tstamp:
                #rewind
                print "Rewinding"
                self.fd.close()
                self.fd=open(self.fname)
            elif self.lasttstamp==tstamp:
                return self.buf.copy()
        finished=0
        self.lasttstamp=tstamp
        self.lastfno=fno
        while finished==0:
            if self.savedData!=None:
                data=self.savedData
                self.savedData=None
            else:
                data=serialise.ReadMessage(self.fd)
            if data==None:
                print "Got to end of stored parameters"
                finished=1
            else:
                if (fno!=None and data[0]>fno) or (tstamp!=None and data[1]>tstamp):
                    print "Got to position"
                    finished=1
                    self.savedData=data
                    if self.lasttstamp==None:
                        self.laststamp=data[1]
                    if self.lastfno==None:
                        self.lastfno=data[0]
                else:
                    #put the data into the buffer
                    vdict=data[2]
                    for k in vdict.keys():
                        self.buf.set(k,vdict[k][0],comment=vdict[k][1])
                    self.buf.set("switchTime",data[1])
                    self.buf.set("frameno",data[0])

        return self.buf.copy()

    def getNextChange(self):
        """Get timestamp and frame number and data of next switch"""
        if self.savedData==None:
            self.savedData=serialise.ReadMessage(self.fd)
        return self.savedData#fno,time,dictionary of {key:(value,comment)}

    def advance(self):
        """Move to the next buffer swap"""
        if self.savedData==None:
            self.savedData=serialise.ReadMessage(self.fd)
        data=self.savedData
        self.savedData=None
        vdict=data[2]
        for k in vdict.keys():
            self.buf.set(k,vdict[k][0],comment=vdict[k][1])
        self.buf.set("switchTime",data[1])
        self.buf.set("frameno",data[0])
        return self.buf.copy()
