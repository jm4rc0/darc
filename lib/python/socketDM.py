"""Code to produce a dummy camera image and send over a socket.
"""
import SockConn
import time
import numpy
import sys
import FITS

class socketCam:
    def __init__(self,host,port,nact):
        self.port=port
        self.host=host
        self.nact=nact
        self.globals=globals()
        self.sockConn=SockConn.SockConn(self.port,host=self.host,globals=self.globals,startThread=0,listenSTDIN=0,connectFunc=self.newConnection)
        self.sockConn.readsock=self.readsock


    def newConnection(self,s,suberr=0):
        print "New connection"

    def readsock(self,s):
        data=""
        try:
            nr=0
            nn=8+2*self.nact
            while nr<nn:
                tmp=s.recv(nn-nr)
                if tmp=="":#socket closed?
                    print "Socket closed?"
                    raise Exception("Sock closed")

                data+=tmp
                nr=len(data)
        except:
            print "Error reading socket"
            print sys.exc_info()
            return -1
        hdr=numpy.fromstring(data[:8],dtype=numpy.uint32)
        data=numpy.fromstring(data[8:],dtype=numpy.uint16)
        if hdr[0]!=((0x5555<<16) | self.nact):
            print "Wrong header received",hex(hdr[0]),hdr[1]
        #print data,hdr[1]
        print hdr[1]

            
if __name__=="__main__":
    if len(sys.argv)!=4:
        print "Usage: %s host port nact"%sys.argv[0]
        sys.exit(0)
    host=sys.argv[1]
    port=int(sys.argv[2])
    nact=int(sys.argv[3])
    sc=socketCam(host,port,nact)
    sc.sockConn.loop()
                
             
