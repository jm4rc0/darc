import numpy
import socket
HSIZE=32#in circ.h
class DataReceiver:
    def __init__(self,host="127.0.0.1",port=7777):#,stream,prefix=""):
        """Starts a server that can listen to a darc sender stream.  
        It is not advisable to use this class.  
        However, it can be useful in rare circumstances.
        
        Once initialised, call the loop() method.  Then locally on the darc machine start a sender with something like:
        sender -p7777 -h10.0.1.91  -d10 -r -n mainrtcCentBuf -R
        where the port number (7777) is should match the DataReceiver().port value.  
        """
        
        #self.stream=stream
        #self.prefix=prefix
        self.go=1
        self.port=port
        self.host=host
        self.listenSocket()
        
    def processData(self,data):
        print "Overload this function to do something with data (returning False to end processing, True to continue)"
        print data.dtype,data.shape
        return True
    
    def readData(self):
        err=0
        n=0
        data=""
        while err==0 and n<HSIZE:
            tmp=self.sock.recv(HSIZE-n)
            nrec=len(tmp)
            if nrec>0:
                n+=nrec
                data+=tmp
            else:
                print "Error getting data"
                print n
                self.go=0
                return None
        #now compare header with previous to get dtype etc.
        hdr=numpy.fromstring(data,dtype=numpy.uint8)
        ihdr=hdr.view(numpy.int32)
        data=None
        if ihdr[0]==28:#28 bytes following
            if hdr[4]==0x55 and hdr[5]==0x55:#we're being resent size/shape
                self.nd=hdr[6]
                self.dtype=chr(hdr[7])
                self.dims=ihdr[2:2+self.nd]
        else:#otherwise ignore it - probably sender testing the connection.
            dsize=ihdr[0]-HSIZE+4
            n=0
            data=""
            while err==0 and n<dsize:
                tmp=self.sock.recv(dsize-n)
                rec=len(tmp)
                if rec>0:
                    n+=rec
                    data+=tmp
                else:
                    print "Error reading socket"
                    self.go=0
                    return None
                
            data=numpy.fromstring(data,dtype=self.dtype)
            try:
                data.shape=self.dims
            except:
                print data.shape,self.dims,self.dtype
                raise
        return data
        
    def listenSocket(self):
        self.sock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        while 1:
            try:
                self.sock.bind((self.host,self.port))
            except:
                print "Couldn't bind to %s on port %d - trying again"%(self.host,self.port)
                self.port+=1
            else:
                break
        print "Bound to port %d"%self.port
        self.sock.listen(1)
        #Now start a sender on darc.
        #d=darc.Control(self.prefix)
        #d.StartSender(self.stream,host,self.port)
        
    def loop(self):
        err=0
        sock,addr=self.sock.accept()
        print "Accepting client from %s"%str(addr)
        self.sock=sock
        while self.go:
            while err==0 and self.go:
                data=self.readData()
                if data is not None:
                    if not self.processData(data):
                        self.go=0
        self.sock.close()
