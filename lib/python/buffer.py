try:
    import utils
except:
    print "WARNING - buffer.py cannot import utils"
import numpy
import time,os#,stat
#import threading
class Buffer:
    """This needs to be a large shared memory region, so that values can get updated by other processes.
    This stores everything in a large buffer.
    First the header, which contains max nhdr entries, each of which comprises of name(31), type(1), start(4), nbytes(4),ndim(4),shape(24),lcomment(4) total 72
    
    """
    def __init__(self,shmname,owner=0,size=64*1024*1024,nhdr=128):
        self.shmname=shmname
        self.owner=owner
        self.nhdr=nhdr
        if shmname!=None:
            mode="w+"
            if owner==0:#get the buffer size...
                mode="r+"
                size=os.stat("/dev/shm"+shmname).st_size#[stat.ST_SIZE]
                print "Opening buffer of size %d bytes"%size
            #self.buffer=utils.open((size,),"c",shmname,owner)
            self.arr=numpy.memmap("/dev/shm"+shmname,"c",mode,shape=(size,))
            #buffer has a info header with hdrsize(4),nhdr(4),flags(4),mutexsize(4),condsize(4),mutex(N),cond(N)
            hdrsize=int(self.arr[:4].view(numpy.int32)[0])
            #pointer to the main part of the array.
            self.buffer=self.arr[hdrsize:]
            #get the number of entries.
            self.nhdr=int(self.arr[4:8].view(numpy.int32)[0])
            msize=int(self.arr[12:16].view(numpy.int32)[0])
            self.flags=self.arr[8:12].view(numpy.int32)
            csize=int(self.arr[16:20].view(numpy.int32)[0])
            #get the memory occupied by the condition variable and mutex.
            self.condmutex=self.arr[20:20+msize]
            self.cond=self.arr[20+msize:20+msize+csize]
            #self.semid=utils.newsemid("/dev/shm"+shmname,98,1,1,owner)
        else:
            self.arr=None
            self.buffer=numpy.zeros((size,),"c")
            self.condmutex=None
            self.cond=None
        self.bufferSize=size
        self.align=8#align data to 8 byte boundaries...
        self.hdrsize=72
        self.header=self.buffer[:nhdr*self.hdrsize]
        #self.header.shape=(nhdr,self.hdrsize)
        #self.labels=self.header[:,:27]#name of variable
        #self.type=self.header[:,27]#type of variable
        #self.start=self.header[:,28:32].view("i")#starting index in array
        #self.nbytes=self.header[:,32:36].view("i")#number of bytes occupoed
        #self.ndim=self.header[:,36:40].view("i")#dimensionality
        #self.shape=self.header[:,40:64].view("i")#dimensions (max 6)
        self.labels=self.header[:31*nhdr]
        
        self.labels.shape=(nhdr,31)
        self.blabels=self.labels.view("b")
        self.type=self.header[31*nhdr:32*nhdr]
        self.start=self.header[32*nhdr:36*nhdr].view("i")
        self.nbytes=self.header[36*nhdr:40*nhdr].view("i")
        self.ndim=self.header[40*nhdr:44*nhdr].view("i")
        self.shape=self.header[44*nhdr:68*nhdr].view("i")
        self.shape.shape=(nhdr,6)
        self.lcomment=self.header[68*nhdr:72*nhdr].view("i")
        self.tmpname=numpy.zeros((31,),"c")
        if owner:
            self.buffer.view("b")[:]=0
            #utils.initSemaphore(self.semid,0,0)#initially, things can write into the buffer.
            raise Exception("Cannot yet own the param Buffer from python - need to implement mutex/cond initialisation")
            #self.initialise()
            pass

    def __del__(self):
        if self.shmname!=None:
            if self.owner:
                #utils.semdel(self.semid)
                raise Exception("Need to destroy cond and mutex in buffer.py")
            #utils.unmap(self.buffer)#dodgy?
        self.buffer=None
        if self.owner:
            #utils.unlink(self.shmname)
            os.unlink("/dev/shm"+self.shmname)



    def freezeContents(self):
        """Call this so that nothing can (legally) write into the buffer"""
        #utils.initSemaphore(self.semid,0,1)#initialise so that something can block on it waiting for a zero.
        raise Exception("freezeContents not yetimplemented")

    def unfreezeContents(self):
        """Call this so that things can (legally) write into the buffer"""
        #utils.initSemaphore(self.semid,0,0)#initialise so that something can unblock on it waiting for a zero.
        raise Exception("unfreezeContents not yetimplemented")

    def getNEntries(self):
        """Get the number of entries..."""
        i=0
        while i<self.nhdr and self.blabels[i,0]!=0:
            i+=1
        return i
        
    def getLabels(self):
        l=[]
        n=self.getNEntries()
        for i in range(n):
            l.append(self.labels[i].tostring().strip("\0"))
        return l
    
    def get(self,name,copy=0):
        #print "Get %s"%name
        name=name[:31]
        i=self.getIndex(name)
        val=self.makeval(i)
        if copy and type(val)==numpy.ndarray:
            val=val.copy()
        return val

    def remove(self,name):
        """Remove an entry from the param buffer"""
        name=name[:31]
        indx=self.getIndex(name,raiseerror=0)
        if indx==None:
            return None
        val=self.makeval(indx)
        if type(val)==numpy.ndarray:
            val=val.copy()
        self.blabels[indx:-1]=self.blabels[indx+1:]
        self.blabels[-1]=0
        self.type[indx:-1]=self.type[indx+1:]
        self.start[indx:-1]=self.start[indx+1:]
        self.nbytes[indx:-1]=self.nbytes[indx+1:]
        self.ndim[indx:-1]=self.ndim[indx+1:]
        self.shape[indx:-1]=self.shape[indx+1:]
        self.lcomment[indx:-1]=self.lcomment[indx+1:]
        return val

    def getComment(self,name):
        name=name[:31]
        i=self.getIndex(name)
        if self.lcomment[i]>0:
            return self.buffer[self.start[i]+self.nbytes[i]:self.start[i]+self.nbytes[i]+self.lcomment[i]].tostring()
        else:
            return ""

    def getIndex(self,name,raiseerror=1):
        name=name[:31]
        l=len(name)
        self.tmpname[:l]=name
        self.tmpname.view("b")[l:]=0
        for i in range(self.nhdr):
            if numpy.alltrue(self.labels[i]==self.tmpname):
                return i
        if raiseerror:
            raise Exception("name %s not found in buffer"%name)
        return None
    def makeval(self,indx):
        if self.type[indx] in numpy.typecodes["All"]:
            val=self.buffer[self.start[indx]:self.start[indx]+self.nbytes[indx]]        
            val=val.view(self.type[indx])
            if self.ndim[indx]>0:
                val.shape=self.shape[indx,:self.ndim[indx]]
                if type(val)!=numpy.ndarray:
                    val=numpy.array(val)#could have been memmap.
            else:#not an array... single value only.
                val=val[0]
        elif self.type[indx]=='n':#None...
            val=None
        elif self.type[indx]=='s':#string
            
            val=self.buffer[self.start[indx]:self.start[indx]+self.nbytes[indx]]        
            val=val.tostring()
            val=val.strip("\0")
        else:
            print "Unrecognised type in buffer.makeval() %s"%self.type[indx]
        return val

    def setControl(self,name,val):
        """same as set() but used for switchRequested, so done in c (acts on current buffer - must write with minimal delay)"""
        #print "TODO: buffer.setControl() coded in C"#actually - does it need doing in c?  If ignoreLoc, then the actuat write to the buffer will be very quick.
        self.set(name,val,ignoreLock=1)

    def set(self,name,val,ignoreLock=0,comment=""):
        if name in ["switchRequested"]:
            #print "ERROR (TODO - decide)? Buffer.set cannot be used for switchRequested, use buffer.setControl instead (done in c rather than python)"
            pass
        if ignoreLock==0 and self.shmname!=None:
            #utils.semop(self.semid,0,0)#wait for the buffer to be unfrozen.
            #Check the freeze bit - if set, block on the condition variable.
            if self.arr!=None: 
                while int(self.arr[8:12].view(numpy.int32)[0])==1:#
                    # buffer is currently frozen - wait for it to unblock
                    utils.pthread_mutex_lock(self.condmutex)
                    utils.pthread_cond_timedwait(self.cond,self.condmutex,1.0,1)
                    utils.pthread_mutex_unlock(self.condmutex)
        if type(comment)==type(""):
            lcom=len(comment)
        else:
            lcom=0
        name=name[:31]
        if type(val) in [type(0)]:
            val=numpy.array([val]).astype(numpy.int32)[0]
        elif type(val) in [type(0.)]:
            val=numpy.array([val]).astype(numpy.float32)[0]

        t1=type(numpy.array([0]).astype("i").view("i")[0])#this is necessary for 32 bit machines, because in 32bit numpy screws up, so that numpy.int32 type is for "l", with nothing corresponding to "i".
        t2=type(numpy.array([0]).astype("l").view("l")[0])
        if type(val) in [numpy.ndarray,numpy.float64,numpy.int32,numpy.int64,numpy.float32,numpy.int16,t1,t2]:
            bytes=val.size*val.itemsize
            shape=val.shape
            dtype=val.dtype.char
            if val.dtype==numpy.int32:
                dtype="i"#fix for 32 bit machines...
            val=val.ravel().view("c")
        elif type(val)==type(""):
            if len(val)==0:
                val="\0"
            elif val[-1]!="\0":
                val+="\0"
            bytes=len(val)
            dtype='s'
            shape=()
        elif type(val)==type(None):
            bytes=0
            dtype='n'
            shape=()
        else:
            #self.unfreezeContents()
            raise Exception("Type %s not known for %s (numpy.int32 is %s,%d)"%(type(val),name,numpy.int32,type(val)==numpy.int32))
        indx=self.getIndex(name,raiseerror=0)
        if indx==None:#insert a new value.
            indx=self.newEntry(name)
            if indx==None:
                #self.unfreezeContents()
                raise Exception("buffer.set Unable to create new entry %s"%name)
            #print "Adding new buffer entry %s"%name
        if self.nbytes[indx]+self.lcomment[indx]<bytes+lcom:#there is no space for it at current location...
            start=self.getSpace(bytes+lcom)
            if start==None:
                #self.unfreezeContents()
                raise Exception("buffer.set No space left in buffer %s size %d+%d"%(name,bytes,lcom))
            #print "Entry %s moving to location %d (size %d+%d)"%(name,start,bytes,lcom)
            self.start[indx]=start
        if bytes>0:
            self.buffer[self.start[indx]:self.start[indx]+bytes]=val
        if lcom>0:
            self.buffer[self.start[indx]+bytes:self.start[indx]+bytes+lcom]=comment
        self.lcomment[indx]=lcom
        self.nbytes[indx]=bytes
        self.ndim[indx]=len(shape)
        self.shape[indx,:self.ndim[indx]]=shape
        self.type[indx]=dtype
        #self.unfreezeContents()

    def newEntry(self,name):
        """Get the index for a new entry"""
        name=name[:31]
        n=self.getNEntries()
        #print "%d entries"%n
        if n>=self.nhdr:
            return None
        indx=n
        self.nbytes[indx]=0
        l=len(name)
        self.labels[indx,:l]=name
        return indx

    def getSpace(self,bytes):
        """find space in the buffer for this many bytes..."""
        if bytes==0:
            return 0
        s=(int(self.nhdr*self.hdrsize+self.align-1)/self.align)*self.align
        e=s+bytes
        found=0
        nEntries=self.getNEntries()
        while found==0 and e<=self.bufferSize:
            found=1
            for i in range(nEntries):
                if self.nbytes[i]+self.lcomment[i]>0:
                    if (self.start[i]<=s and self.start[i]+self.nbytes[i]+self.lcomment[i]>s) or (self.start[i]<e and self.start[i]+self.nbytes[i]+self.lcomment[i]>=e) or (s<self.start[i] and e>self.start[i]):
                        found=0
                        s=(int(self.start[i]+self.nbytes[i]+self.lcomment[i]+self.align-1)/self.align)*self.align
                        e=s+bytes
        if found==0:
            return None
        return s

    def getMem(self):
        """Finds how much space the buffer is using from start to finish (i.e. including any free space inbetween)"""
        n=self.getNEntries()
        mem=0
        for i in range(n):
            mem=max(mem,self.start[i]+self.nbytes[i]+self.lcomment[i])
        return mem
        
##     def initialise(self):
##         """Place the memory into its initial state..."""
##         nacts=54
##         self.buffer.view("b")[:]=0
##         control={
##             "switchRequested":0,#this is the only item in a currently active buffer that can be changed...
##             "DMgain":0.25,
##             "staticTerm":None,
##             "refSlopes":None,
##             "kalmanCoeffs":None,
##             "dmControlState":0,
##             "reconmx":None,
##             "dmPause":0,
##             "reconMode":"closedLoop",
##             "applyPxlCalibration":0,
##             "centroidMode":"CPU",#whether data is from cameras or from WPU.
##             "centroidAlgorithm":"wcog",
##             "windowMode":"basic",
##             "windowMap":None,
##             "maxActuatorsSaturated":10,
##             "applyAntiWindup":0,
##             "tipTiltGain":0.5,
##             "laserStabilisationGain":0.1,
##             "thresholdAlgorithm":1,
##             "acquireMode":"frame",#frame, pixel or subaps, depending on what we should wait for...
##             "reconstructMode":"simple",#simple (matrix vector only), other...
##             "v0":0.,#v0 from the tomograhpcic algorithm in openloop (see spec)
##             "E":None,#E from the tomo algo in openloop (see spec).
##             "clip":1,
##             "bleedGain":None,#a gain for the piston bleed...
##             "midRangeValue":0.,#midrange actuator value used in actuator bleed
##             "gain":numpy.zeros((nacts,),numpy.float32),#the actual gains for each actuator...
##             "sendRawPxlData":0,
##             "rawPxlDatgaSubSampleRate":10,
##             "sendCalibratedPxlData":0,
##             "calibratedPxlDataSubSampleRate":10,
##             "sendCentroidData":0,
##             "centroidDataSubSampleRate":10,
##             "sendMirrorDemands":0,
##             "mirrorDemandsSubSampleRate":10,
##             "sendRegularStatusPackets":0,
##             "regularStatusPacketsSubSampleRate":10,
##             "ncam":1,
##             "nsuby":8,
##             "nsubx":8,
##             "npxly":64,
##             "npxlx":64,
##             "pxlCnt":None,#array of number of pixels to wait for next subap to have arrived.
##             "subapLocation":None,#array of ncam,nsuby,nsubx,4, holding ystart,yend,xstart,xend for each subap.
            
##             }
##         control["gain"][:2]=control["tipTiltGain"]
##         control["gain"][2:]=control["DMgain"]

##         for key in control.keys():
##             self.set(key,control[key])

##     def __getattr__(self,key):
##         return self.get(key)
def getAlign():
    # warning - don't change this from 8 without looking at the computation of nstore in self.reshape()
    return 8
def getHeaderSize():
    align=getAlign()
    return ((8+4+4+4+2+1+1+6*4+align-1)/align)*align #header contains buffer size (int64), last written to (int32), freq (int32), nstore (int32), forcewriteall(int8),ndim (int8),  dtype (int8), forcewrite (int8), shape (6*int32)

class Circular:
    """A class to implement a circular buffer.  Only the owner ever writes to this buffer, except for the freq entry.
    upon initialisation, the buffer size is set.

    """
    def __init__(self,shmname,owner=0,dims=None,dtype=None,nstore=None,raw=0):
        self.align=getAlign()
        
        self.shmname=shmname
        self.owner=owner
        self.freqcnt=0
        self.framecnt=0
        self.dtypeSave=None
        self.ndimSave=None
        self.shapeArrSave=None
        self.nstoreSave=None
        self.raw=raw
        self.lastReceived=-1#last frame received...
        self.lastReceivedFrame=-1
        if owner==1:
            raise Exception("Not yet implemented without semaphores")
            self.hdrsize=getHeaderSize()
            if dims==None or dtype==None or nstore==None:
                raise Exception("Owner of circular buffer must specify dims, dtype and nstore")
            #self.timesize=((8*nstore+self.align-1)/self.align)*self.align#size of the timestamp (float 64 for each entry)
            #self.frameNoSize=((4*nstore+self.align-1)/self.align)*self.align#size of the frameno (int32 for each entry)
            #self.dtype=dtype
            #self.dims=dims
            self.datasize=reduce(lambda x,y:x*y,dims)
            self.frameSize=((self.datasize*numpy.zeros((1,),dtype).itemsize+32+self.align-1)/self.align)*self.align
            self.size=self.hdrsize+self.frameSize*nstore#self.frameNoSize+self.timesize+nstore*self.datasize*numpy.zeros((1,),dtype).itemsize
            mode="w+"
        else:
            #first open the header to see what size it is, then open the full array.
            #buf=utils.open((self.hdrsize,),"b",shmname,owner)
            mode="r+"
            #print "todo: use a stat to get circ buf shm size"
            #buf=numpy.memmap("/dev/shm"+shmname,"b","r",shape=(8,))
            #self.size=int(buf[0:8].view(numpy.int64)[0])
            self.size=os.stat("/dev/shm"+shmname).st_size

            
            #utils.unmap(buf)

        #self.buffer=utils.open((self.size,),"b",shmname,owner)
            
        self.buffer=numpy.memmap("/dev/shm"+shmname,"b",mode,shape=(self.size,))
        #now get the hdrsize
        self.hdrsize=int(self.buffer[48:52].view(numpy.int32)[0])
        msize=int(self.buffer[52:56].view(numpy.int32)[0])
        csize=int(self.buffer[56:60].view(numpy.int32)[0])
        self.condmutex=self.buffer[60:60+msize]
        self.cond=self.buffer[60+msize:60+msize+csize]
        #self.semid=utils.newsemid("/dev/shm"+shmname,98,1,1,owner)

        #get the header arrays
        self.bufsize=self.buffer[0:8].view(numpy.int64)
        self.lastWritten=self.buffer[8:12].view("i")
        self.freq=self.buffer[12:16].view("i")
        self.nstore=self.buffer[16:20].view("i")
        self.forcewriteall=self.buffer[20:21]
        self.ndim=self.buffer[21:22]#.view("h")
        self.dtype=self.buffer[22:23].view("c")
        self.forcewrite=self.buffer[23:24]
        self.shapeArr=self.buffer[24:48].view("i")

        if owner:
            raise Exception("Can't be owner in buffer.py class Circular - implement if needed")
            self.buffer[:]=0
            self.bufsize[0]=self.size#this should never change now...
            self.lastWritten[0]=-1
            self.freq[0]=0
            self.nstore[0]=nstore
            self.ndim[0]=len(dims)
            self.dtype[0]=dtype
            self.forcewrite[0]=0
            self.shapeArr[:]=-1
            self.shapeArr[:self.ndim[0]]=dims
            utils.initSemaphore(self.semid,0,1)#initialise so that something can block on it waiting for a zero.

        self.makeDataArrays()

        #if owner==0:
        #    print "Got: %s %s %d"%(self.dtype[0],str(self.shapeArr[:self.ndim[0]]),self.nstore[0])

    
    def compare(self,arr):
        """Compare the header of self with arr.  Return True if they are the same.  Doesn't compare lastWritten, because this may update during the compare...
        """
        #print "Comparing",arr[:8],self.buffer[:8],arr[12:self.hdrsize],self.buffer[12:self.hdrsize]
        if numpy.alltrue(arr[:8]==self.buffer[:8]) and numpy.alltrue(arr[12:self.hdrsize]==self.buffer[12:self.hdrsize]):
            return True
        return False


    def __del__(self):
        if self.owner:
            utils.semdel(self.semid)
        #utils.unmap(self.buffer)#dodgy?
        self.buffer=None
        if self.owner:
            #utils.unlink(self.shmname)
            os.unlink("/dev/shm"+self.shmname)
    def setForceWrite(self,val=1):
        """Called to force a write to the buffer, even if one is not due (ie not decimated yet)
        """
        self.forcewrite[0]=val

    def setForceWriteAll(self,val=1):
        """If this is set in the first buffer written (rtcPxlBuf), then the RTC will set forceWrite in all other buffers.
        """
        self.forcewriteall[0]=val

    def add(self,data):
        """Should be called for owner only.  data is an array or a list of arrays
        Note - this is untested on 32 bit machines... (int32 dtype on these mathines is l instead of the expected i)"""
        raise Exception("FUnciton add not implemented")
        self.freqcnt+=1
        self.framecnt+=1
        if (self.freq[0]>0 and self.freqcnt>=self.freq[0]) or self.forcewrite[0]!=0:#add to the buffer
            if self.forcewrite[0]>0:
                self.forcewrite[0]-=1
            self.freqcnt=0
            if type(data)==type([]):
                size=0
                dtype=data[0].dtype.char
                for d in data:
                    size+=d.size
                    if dtype!=d.dtype.char:
                        raise Exception("Not all data in list are same type in circular buffer")
                if self.ndim[0]!=1 or self.shapeArr[0]!=size or dtype!=self.dtype[0]:
                    raise Exception("Data list sizes/types don't match for circular buffer %s %s %s %s %s"%(str(self.ndim[0]),str(self.shapeArr[0]),str(size),str(dtype),str(self.dtype[0])))
            else:
                if data.shape!=tuple(self.shapeArr[:self.ndim[0]]) or data.dtype.char!=self.dtype[0]:
                    raise Exception("Data sizes/types don't match for circular buffer %s %s %s %s"%(str(data.shape),str(self.shapeArr),str(data.dtype.char),self.dtype))
            indx=self.lastWritten[0]+1
            if indx==self.nstore[0]:
                indx=0
            if type(data)==type([]):
                f=0
                for d in data:
                    t=f+d.size
                    self.data[indx,f:t]=d.ravel()
                    f=t
            else:
                self.data[indx]=data
            self.timestamp[indx]=time.time()
            self.datasizearr[indx]=self.datasize*self.elsize+32-4
            self.datatype[indx]=self.dtype[0]
            self.frameNo[indx]=self.framecnt
            #now update the lastWritten flag.
            self.lastWritten[0]=indx
            #and signal that there is a new entry...
            #utils.semop(self.semid,0,-1)#decrease to unblock waiting process
            utils.initSemaphore(self.semid,0,0)#reinitialise
            utils.initSemaphore(self.semid,0,1)#reinitialise

    def reshape(self,dims,dtype):
        """Should be called for owner only.  Reshape the array.  The shared memory array (self.buffer) remains the same, so readers don't have to reopen it."""
        self.ndim[0]=len(dims)
        self.dtype[0]=dtype
        self.shapeArr[:]=-1
        self.shapeArr[:len(dims)]=dims

        self.lastWritten[0]=-1
        #now work out nstore...
        databytes=(int(reduce(lambda x,y:x*y,dims)*numpy.zeros((1,),dtype).itemsize+32+self.align-1)/self.align)*self.align
        nstore=int(self.size-self.hdrsize)/int(databytes)#+8+4)
        print "nstore now %d"%nstore
        self.nstore[0]=nstore
        self.makeDataArrays()
        
    def makeDataArrays(self):
        remade=0
        if self.nstoreSave!=self.nstore[0] or self.ndimSave!=self.ndim[0] or (not numpy.alltrue(self.shapeArrSave==self.shapeArr[:self.ndim[0]])) or self.dtypeSave!=self.dtype[0]:
            #something changed, so re-make the data arrays...
            #print "Remaking dataarray"
            remade=1
            self.nstoreSave=self.nstore[0]
            self.ndimSave=self.ndim[0]
            self.shapeArrSave=self.shapeArr[:self.ndim[0]].copy()
            self.dtypeSave=self.dtype[0]
            #self.timesize=((8*self.nstore[0]+self.align-1)/self.align)*self.align#size of the timestamp (float 64 for each entry)
            #self.frameNoSize=((4*self.nstore[0]+self.align-1)/self.align)*self.align#size of the framestamp (int32 for each entry)
            self.datasize=reduce(lambda x,y:x*y,self.shapeArr[:self.ndim[0]])#*self.nstore[0]
            self.elsize=numpy.zeros((1,),self.dtype[0]).itemsize
            self.frameSize=(int(self.datasize*self.elsize+32+self.align-1)/self.align)*self.align
            d=self.buffer[self.hdrsize:self.hdrsize+self.frameSize*self.nstore[0]]
            t=d.view("d")
            t.shape=self.nstore[0],t.shape[0]/self.nstore[0]
            self.timestamp=t[:,1]
            r=d.view("b")
            r.shape=self.nstore[0],r.shape[0]/self.nstore[0]
            self.rawdata=r[:,:32+self.datasize*self.elsize]
            t=d.view("c")
            t.shape=self.nstore[0],t.shape[0]/self.nstore[0]
            self.datatype=t[:,16]
            s=d.view("i")
            s.shape=self.nstore[0],s.shape[0]/self.nstore[0]
            self.datasizearr=s[:,0]
            f=d.view("i")
            f.shape=self.nstore[0],f.shape[0]/self.nstore[0]
            self.frameNo=f[:,1]
            d=d.view(self.dtype[0])
            d.shape=self.nstore[0],d.shape[0]/self.nstore[0]
            start=32/d.itemsize
            self.data=d[:,start:start+self.datasize]
            #print d.shape,d.dtype,self.data.shape,self.nstore,self.shapeArr[:self.ndim[0]]
            self.data.shape=(self.nstore[0],)+tuple(self.shapeArr[:self.ndim[0]])
            #print d.shape,self.hdrsize,self.frameSize,self.nstore[0],self.buffer.shape,self.datasize,d.dtype.char,self.dtype[0]
            #d.shape=self.nstore[0],self.frameSize
            #self.data=self.buffer[self.hdrsize+self.frameNoSize+self.timesize:self.hdrsize+self.frameNoSize+self.timesize+self.datasize*numpy.zeros((1,),self.dtype[0]).itemsize].view(self.dtype[0])
            #self.timestamp=d[:,8:16].view("d")
            #self.timestamp=self.buffer[self.hdrsize+self.frameNoSize:self.hdrsize+self.frameNoSize+self.timesize].view("d")
            #self.frameNo=d[:,4:8].view("i")
            #self.frameNo=self.buffer[self.hdrsize:self.hdrsize+self.frameNoSize].view("i")
            #self.data=d[:,16:16+self.datasize].view(self.dtype[0])
            #self.data.shape=(self.nstore[0],)+tuple(self.shapeArr[:self.ndim[0]])
        return remade
    def getShape(self):
        return tuple(self.shapeArr[:self.ndim[0]])

        
    def getLatest(self):
        """Get the latest one..."""
        indx=int(self.lastWritten[0])
        return self.get(indx)

    def get(self,indx,copy=0):
        self.lastReceived=int(indx)
        if indx<0:
            self.lastReceivedFrame=-1
            return None
        #check arrays are the right shape...
        self.makeDataArrays()
            
        self.lastReceivedFrame=int(self.frameNo[indx])
        if self.raw==0:
            if copy:
                return numpy.array(self.data[indx]),self.timestamp[indx],self.frameNo[indx]
            else:
                return self.data[indx],self.timestamp[indx],self.frameNo[indx]
        else:
            if copy:
                return numpy.array(self.rawdata[indx])
            else:
                return self.rawdata[indx]
    def getNext(self):
        """Wait for the next one to be available (which is current if this hasn't previously been asked for).  Actually, the logic here doesn't seem to work...  Not used."""
        raise Exception("Function getNext not implemented")
        utils.semop(self.semid,0,0)#wait for a zero...
        utils.initSemaphore(self.semid,0,1)#reinitialise
        
        return self.getLatest()

    def getNextFrame(self,timeout=0.,copy=0,retry=1):
        """Look at lastReceived and lastWritten, and then send if not equal, otherwise wait...
        But - what to do if the buffer has just been reshaped and written to, so that lastWritten==0?  This typically might happen in the case of rtcGenericBuf.
        """
        data=None
        while data==None:
            if self.makeDataArrays():#have been remade...
                self.lastReceived=-1
                self.lastReceivedFrame=-1
            lw=int(self.lastWritten[0])
            lwf=int(self.frameNo[lw])
            #print "getNextFrame:",lw,self.lastReceived,data,lwf,self.lastReceivedFrame
            if lw<0:#no frame written yet - so block, waiting.
                pass
            elif lw==self.lastReceived:
                #If the buffer has been reshaped, this might be the case even if new data has arrived.  So, we should also check the frame numbers here.  Also if the buffer has wrapped round...
                #print self.lastReceivedFrame,lwf,lw,self.lastReceived
                if self.lastReceivedFrame!=lwf:
                    #frame numbers different so data is new.
                    #No change to lastReceived...
                    self.lastReceivedFrame=int(lwf)
                    if self.raw==0:
                        if copy:
                            data=numpy.array(self.data[self.lastReceived]),self.timestamp[self.lastReceived],lwf
                        else:
                            data=self.data[self.lastReceived],self.timestamp[self.lastReceived],lwf
                    else:
                        if copy:
                            data=numpy.array(self.rawdata[self.lastReceived])
                        else:
                            data=self.rawdata[self.lastReceived]

            else:
                #self.makeDataArrays()
                self.lastReceived=(self.lastReceived+1)%self.nstore[0]
                self.lastReceivedFrame=int(self.frameNo[self.lastReceived])
                if self.raw==0:
                    if copy:
                        data=numpy.array(self.data[self.lastReceived]),self.timestamp[self.lastReceived],self.frameNo[self.lastReceived]
                    else:
                        data=self.data[self.lastReceived],self.timestamp[self.lastReceived],self.frameNo[self.lastReceived]
                else:
                    if copy:
                        data=numpy.array(self.rawdata[self.lastReceived])
                    else:
                        data=self.rawdata[self.lastReceived]
            if data==None:
                try:
                    #print "Waiting timeout %g %d %d"%(timeout,self.lastReceived,lw)
                    utils.pthread_mutex_lock(self.condmutex)
                    if timeout==0:
                        utils.pthread_cond_wait(self.cond,self.condmutex)
                        timeup=0
                    else:
                        timeup=utils.pthread_cond_timedwait(self.cond,self.condmutex,timeout,1)
                    utils.pthread_mutex_unlock(self.condmutex)
                    #timeup=utils.semop(self.semid,0,0,timeout)#wait for a zero.
                    #print "got, timeup=%g %d %d %d"%(timeup,self.lastReceived,lw,threading.activeCount())
                    
                    if timeup==0:
                        if self.makeDataArrays():#has been remade...
                            self.lastReceived=-1
                        self.lastReceived=(self.lastReceived+1)%int(self.nstore[0])
                        self.lastReceivedFrame=int(self.frameNo[self.lastReceived])
                        if self.raw==0:
                            if copy:
                                data=numpy.array(self.data[self.lastReceived]),self.timestamp[self.lastReceived],self.frameNo[self.lastReceived]
                            else:
                                data=self.data[self.lastReceived],self.timestamp[self.lastReceived],self.frameNo[self.lastReceived]
                        else:
                            if copy:
                                data=numpy.array(self.rawdata[self.lastReceived])
                            else:
                                data=self.rawdata[self.lastReceived]
                except:
                    data=None
                    raise
                if timeup:
                    #print "timeout - retrying %d"%retry,self.lastReceived,self.lastReceivedFrame,self.lastWritten,self.frameNo
                    data=None
                    if retry==0:
                        break
                    elif retry>0:
                        retry-=1
        #print "Returning %d %d"%(self.lastReceived,self.lastWritten[0]),type(self.lastReceived)
        return data

if __name__=="__main__":
    import sys
    b=Buffer(sys.argv[1])
    labels=b.getLabels()
    for l in labels:
        v=b.get(l)
        print l,v
    b.set("test",99)
