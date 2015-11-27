"""A simple client for darccontrolc.  
Not recommended unless you have a specific reason to use darccontrolc."""
import os
import subprocess
import time
import socket
import numpy
import darc
import buffer
import FITS
#From darccontrolc:
DARCSET=1
DARCGET=2
DARCDECIMATE=3
DARCSENDER=4
DARCSTREAM=5
DARCSTOP=6
DARCCONTROLSTOP=7
DARCINIT=8
DARCINITFILE=9

class DarcCClient:
    def __init__(self,host,port,prefix=""):
        self.prefix=prefix
        self.host=host
        self.port=port
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.sock.connect((host,port))

    def recvall(self,nbytes):
        ngot=0
        data=""
        while ngot<nbytes:
            tmp=self.sock.recv(nbytes-ngot)
            if tmp>0:
                data+=tmp
                ngot+=len(tmp)
            else:
                raise Exception("Error: got 0 bytes from sock.recv (total received is %d)"%ngot)
        return data
    def checkReply(self):
        """Reads 8 bytes from the socket, and checks they are an end of message packet"""
        err=0
        endtxt=self.sock.recv(8)
        end=numpy.fromstring(endtxt,dtype=numpy.int32)
        if end[0]!=0x55555555:
            err=1
            print "Warning: end of reply not received - flushing"
            print "TODO: Flush"
        elif end[1]&0xffff!=0:
            err=1
            print "Error message in reply to msg %d receivved: %d"%(end[1]>>16,end[1]&0xffff)
        return err

    def Set(self,name,value,doswitch=1):
        #send: namelen,name,nbytes,dtype,doswitch
        self.sock.send(numpy.array([0x55555555,DARCSET,len(name)]).astype(numpy.int32))
        self.sock.send(name)
        if value is None:
            dtype='n'
            nbytes=0
        elif type(value)==type(""):
            dtype='s'
            nbytes=len(value)
        elif type(value)==numpy.ndarray:
            dtype=value.dtype.char
            nbytes=value.itemsize*value.size
        elif type(value)==type(0):
            dtype='i'
            nbytes=4
            value=numpy.array([value]).astype(numpy.int32)
        elif type(value)==type(0.):
            dtype='f'
            nbytes=4
            value=numpy.array([value]).astype(numpy.float32)
        else:
            print "data of type %s not yet handled in darccClient.py"%str(type(value))
        self.sock.send(numpy.array([nbytes&0xffffffff,nbytes>>32,ord(dtype),doswitch]).astype(numpy.int32))
        if nbytes>0:
            self.sock.sendall(value)
        self.checkReply()

    def Get(self,name):
        
        self.sock.send(numpy.array([0x55555555,DARCGET,len(name)]).astype(numpy.int32))
        self.sock.send(name)
        hdrtxt=self.sock.recv(52)
        hdr=numpy.fromstring(hdrtxt,numpy.int32)
        nameGot=hdrtxt[:16]
        nbytes=hdr[4]
        dtype=chr(hdr[5])
        ndim=hdr[6]
        dims=hdr[7:13]
        print nameGot,nbytes,dtype,ndim,dims
        #receive the data
        if nbytes>0:
            datatxt=self.recvall(nbytes)
        #and receive the "finished" message...
        self.checkReply()
        if dtype=='s':
            data=datatxt.strip("\0")
        elif dtype in ['f','i','h','I','H','b','B']:
            data=numpy.fromstring(datatxt,dtype)
            data.shape=dims[:ndim]
            if data.size==1:#1 value, not an array.
                data=data[0]
        elif dtype=='n':
            data=None
        else:
            print "Unrecognised dtype %s"%dtype
        return data

    def GetStream(self,name,block=1,forcewrite=1):
        self.sock.send(numpy.array([0x55555555,DARCSTREAM,len(name)]).astype(numpy.int32))
        self.sock.send(name)
        self.sock.send(numpy.array([block|(forcewrite<<1)]).astype(numpy.int32))
        #Now receive size, dtype, data.
        size=self.sock.recv(4)
        size=numpy.fromstring(size,dtype=numpy.int32)[0]
        print "Receiving data of size %d bytes"%size
        if size==0:
            data=None
            frameno=0
            frametime=0.
        else:
            data=self.recvall(size)#this is the header and the data.
            frameno=numpy.fromstring(data[:4],dtype=numpy.uint32)[0]
            frametime=numpy.fromstring(data[4:12],dtype=numpy.float64)[0]
            dtype=data[12]
            data=data[28:]
            print dtype
            if dtype!='s':
                data=numpy.fromstring(data,dtype=dtype)
        self.checkReply()
        return data,frameno,frametime

    def GetStatus(self):
        txt=self.GetStream("rtcStatusBuf")[0]
        if txt==None:
            raise Exception("Unable to get rtcStatusBuf")
        else:
            txt=txt.tostring().strip("\0")
        return txt
    def SetDecimation(self,stream,dec):
        self.sock.send(numpy.array([0x55555555,DARCDECIMATE,len(stream)]).astype(numpy.int32))
        self.sock.send(stream)
        self.sock.send(numpy.array([dec]).astype(numpy.int32))
        self.checkReply()



    def StartReceiver(self,name,datasize=None,affin=0x7fffffff,prio=0,sendFromHead=1,outputname=None,nstore=10,port=4262,readFrom=0,readTo=-1,readStep=1):
        """Starts a receiver locally.  This then receives data from the RTC and writes it to a local shared memory circular buffer, which other local clients can then read.  name here includes prefix, but this shouldn't be sent to receiver.
        """
        if outputname==None:
            outputname=name
        if datasize==None:
            #work out the size of the data...
            data=self.GetStream(name)[0]
            datasize=(data.size*data.itemsize+32)*nstore+buffer.getCircHeaderSize()

        plist=["receiver","-p%d"%port,"-a%d"%affin,"-i%d"%prio,"-n%d"%datasize,"-o/%s"%outputname,name[len(self.prefix):],"-q"]
        if self.prefix!="":
            plist.append("-s%s"%self.prefix)
        if os.path.exists("/dev/shm/%s"%outputname):
            raise Exception("local /dev/shm/%s already exists"%outputname)
        p=subprocess.Popen(plist)
        #Now wait for it to bind, and get the bound port
        cnt=0
        while cnt<100 and not os.path.exists("/dev/shm/%s"%outputname):
            cnt+=1
            time.sleep(0.05)
        if cnt==100:
            p.terminate()
            p.wait()
            raise Exception("Local /dev/shm/%s not found"%outputname)
        cb=buffer.Circular("/%s"%(outputname))
        cnt=0
        s=None
        while cnt<100 and s==None:
            s=cb.getLatest()
            if s==None:
                time.sleep(0.01)
                cnt+=1
        if s!=None:
            port=int(s[0][0])
        else:
            p.terminate()
            p.wait()
            raise Exception("Unable to determine port of receiver")
        print "Local receiver has started and is listening on port %d"%port
        return port


        #Now start the sender, and we're done...
        hostlist=string.join([x[1] for x in getNetworkInterfaces()],",")
        reset=0#don't want to reset the stream...
        self.obj.StartStream(sdata([name]),hostlist,port,decimation,sendFromHead,"name",reset,readFrom,readTo,readStep)


    def StartStream(self,stream,dec=None):
        if dec!=None:
            self.SetDecimation(stream,dec)
        #Start a receiver and get port number.
        port=self.startReceiver(stream)
        #Get the correct client IP - should be on same network as self.host
        IPlist=[x[1] for x in darc.getNetworkInterfaces()]
        sendto=None
        for ip in IPlist:
            if ip.split(".")[:3]==self.host.split(".")[:3]:
                sendto=ip
        if sendto==None:
            raise Exception("Could not find local IP to which %s can connect"%self.host)
        ippart=map(int,sendto.split("."))
        receiverip=ippart[0]+(ippart[1]<<8)+(ippart[2]<<16)+(ippart[3]<<24)
        #Start the sender.
        self.sock.send(numpy.array([0x55555555,DARCSENDER,len(stream)]).astype(numpy.int32))
        self.sock.send(stream)
        self.sock.send(numpy.array([receiverip,port]).astype(numpy.int32))
        self.checkReply()

    def StopDarc(self):
        self.sock.send(numpy.array([0x55555555,DARCSTOP]).astype(numpy.int32))
        self.checkReply()

    def StopControl(self):
        self.sock.send(numpy.array([0x55555555,DARCCONTROLSTOP]).astype(numpy.int32))
        
    def DarcInit(self,data):
        """data can be a FITS filename, or the contents of the FITS file"""
        if os.path.exists(data):
            data=FITS.Read(data)[1]
        nbytes=data.size*data.itemsize
        self.sock.send(numpy.array([0x55555555,DARCINIT,nbytes&0xffffffff,nbytes>>32]).astype(numpy.int32))
        self.sock.sendall(data)
        self.checkReply()
    def DarcInitFile(self,fname):
        """fname is the filename of a FITS config file held locally to darc."""
        self.sock.send(numpy.array([0x55555555,DARCINITFILE,len(fname)]).astype(numpy.int32))
        self.sock.sendall(fname)
        self.checkReply()
        
    def CreateConfig(self,pyconfig,outname=None,size=None,nhdr=None):
        """Creates a FITS config file from a python config.
        Fills in the missing values.
        """
        if outname==None:
            outname=pyconfig[:-2]+"fits"
        buf=buffer.loadBufFromPyConfig(pyconfig,self.prefix,size,nhdr)
        buf.save(outname)

