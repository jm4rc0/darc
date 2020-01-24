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
try:
    import utils
except:
    print "WARNING - buffer.py cannot import utils"
import numpy
import time,os#,stat
#import threading
import FITS
import traceback
def loadBuf(fname,hdu=0):
    data=FITS.Read(fname)[hdu*2+1]
    b=Buffer(None,size=data.size)
    b.assign(data)
    return b

def loadBufFromPyConfig(configFile,prefix="",size=None,nhdr=None,check=1):
    """Set contents of the buffer using the config.py file"""
    d={"control":{},"prefix":prefix,"numpy":numpy}
    execfile(configFile,d)
    control=d["control"]
    if d.has_key("comments"):
        comments=d["comments"]
    else:
        comments={}
    if not control.has_key("configfile"):
        control["configfile"]=configFile
    #invent values...
    import Check
    checkval=Check.Check()
    try:
        checkval.inventValues(control,comments)
    except:
        print "Unable to invent missing values..."
        raise
    if control.has_key("switchRequested"):
        control["switchRequested"]=0
    

    if size==None:
        nbytes=0
        for key in control.keys():
            val=control[key]
            if type(val)==numpy.ndarray:
                nbytes+=val.size*val.itemsize
            elif type(val)==type(""):
                nbytes+=len(val)
            else:
                nbytes+=8#could be something else, but assume this for now.
            nbytes+=len(comments.get(key,""))

        nbytes*=2#allow some space for padding...
        nbytes+=60
        nbytes+=(16+4+4+4+4+24+4)*len(control)
    else:
        nbytes=size
    if nhdr==None:
        nhdr=len(control)+10
        print "Using buffer of size %d bytes with %d entries"%(nbytes,nhdr)
    b=Buffer(None,size=nbytes,nhdr=nhdr)
    b.setControl("switchRequested",0)

    failed=[]
    for key in control.keys():
        try:
            if check:
                control[key]=checkval.valid(key,control[key],b)
            b.set(key,control[key],comment=comments.get(key,""))
            checkval.setDependencies(key,b)
        except:
            failed.append(key)
    while len(failed)>0:
        f=[]
        for key in failed:
            try:
                if check:
                    control[key]=checkval.valid(key,control[key],b)
                b.set(key,control[key],comment=comments.get(key,""))
                checkval.setDependencies(key,b)
            except:
                f.append(key)
        if len(f)==len(failed):#failed to add new ones...
            print "Failed to initialise buffer:"
            print f
            for key in f:
                try:
                    if check:
                        control[key]=checkval.valid(key,control[key],b)
                    b.set(key,control[key],comment=comments.get(key,""))
                    checkval.setDependencies(key,b)
                except:
                    print key
                    traceback.print_exc()

            raise Exception("Failed to initialise buffer")
        failed=f



    return b

    

class Buffer:
    """This needs to be a large shared memory region, so that values can get updated by other processes.
    This stores everything in a large buffer.
  //buffer has a header with hdrsize(4),nhdr(4),flags(4),mutexsize(4),condsize(4),mutex(N),cond(N),spare bytes for alignment purposes.
  pb->hdr=(int*)pb->arr;
  pb->hdr[0]=4+4+4+4+4+sizeof(pthread_cond_t)+sizeof(pthread_mutex_t);
  //just make sure that buf (&pb->arr[pb->hdr[0]]) is 16 byte aligned
  pb->hdr[0]+=(16-((((unsigned long)pb->arr)+pb->hdr[0])&0xf))%16;


    First the header, which contains max nhdr entries, each of which comprises of name(16), type(1), start(4), nbytes(4),ndim(4),shape(24),lcomment(4) total 60
    


    """
    def __init__(self,shmname,create=0,size=64*1024*1024,nhdr=128):
        self.shmname=shmname
        self.unlinkOnDel=0
        self.create=0
        #self.nhdr=nhdr
        if shmname!=None:
            mode="w+"
            if create:
                if os.path.exists("/dev/shm"+shmname):
                    create=0
                else:
                    #create the shm and initialise it.
                    self.arr=numpy.memmap("/dev/shm"+shmname,"c",mode,shape=(size,))
                    self.arr.view("b")[:]=0
                    #write the hdrsize, number of entries etc.
                    self.arr[4:8].view(numpy.int32)[0]=nhdr
                    msize,csize=utils.pthread_sizeof_mutexcond()
                    self.arr[12:16].view(numpy.int32)[0]=msize
                    self.arr[16:20].view(numpy.int32)[0]=csize
                    utils.pthread_cond_init(self.arr[20+msize:20+msize+csize],1)
                    utils.pthread_mutex_init(self.arr[20:20+msize],1)
                    hdrsize=4+4+4+4+4+msize+csize
                    #make it nicely aligned.
                    hdrsize+=(16-((self.arr.__array_interface__["data"][0]+hdrsize)&0xf))%16
                    self.arr[:4].view(numpy.int32)[0]=hdrsize
            if create==0:#get the buffer size...
                mode="r+"
                size=os.stat("/dev/shm"+shmname).st_size#[stat.ST_SIZE]
                print "Opening buffer of size %d bytes"%size
                self.arr=numpy.memmap("/dev/shm"+shmname,"c",mode,shape=(size,))
            #buffer has a info header with hdrsize(4),nhdr(4),flags(4),mutexsize(4),condsize(4),mutex(N),cond(N)
            hdrsize=int(self.arr[:4].view(numpy.int32)[0])
            while hdrsize==0:
                print "Waiting to get hdrsize of parambuf"
                hdrsize=int(self.arr[:4].view(numpy.int32)[0]) 
                time.sleep(0.1)
            #pointer to the main part of the array.
            print "Got buffer header size of %d"%hdrsize
            self.buffer=self.arr[hdrsize:]
            #get the number of entries.
            self.nhdr=self.arr[4:8].view(numpy.int32)
            while self.nhdr[0]==0:
                print "Waiting to get number of header entries..."
                #self.nhdr[0]=int(self.arr[4:8].view(numpy.int32)[0])
                time.sleep(0.1)
            print "Got max number of entries (nhdr) %d"%self.nhdr[0]
            msize=int(self.arr[12:16].view(numpy.int32)[0])
            self.flags=self.arr[8:12].view(numpy.int32)
            csize=int(self.arr[16:20].view(numpy.int32)[0])
            while msize==0 or csize==0:
                msize=int(self.arr[12:16].view(numpy.int32)[0])
                csize=int(self.arr[16:20].view(numpy.int32)[0])
                print "Waiting to get mutex/cond size"
                time.sleep(0.1)
            #get the memory occupied by the condition variable and mutex.
            self.condmutex=self.arr[20:20+msize]
            self.cond=self.arr[20+msize:20+msize+csize]
            #self.semid=utils.newsemid("/dev/shm"+shmname,98,1,1,owner)
        else:
            hdrsize=20
            self.arr=numpy.zeros((max(size,hdrsize),),"c")
            self.arr[:4].view(numpy.int32)[0]=hdrsize
            self.arr[4:8].view(numpy.int32)[0]=nhdr
            self.nhdr=self.arr[4:8].view(numpy.int32)
            self.flags=self.arr[8:12].view(numpy.int32)
            self.buffer=self.arr[hdrsize:]
            self.condmutex=None
            self.cond=None
        self.arrhdrsize=self.arr[:4].view(numpy.int32)
        self.bufferSize=size
        self.align=64#align data to this byte boundaries...
        self.hdrsize=57#the size of an individual entry in the header.  See setNhdr() for why...
        self.setNhdr(self.nhdr[0])
        self.tmpname=numpy.zeros((16,),"c")
        self.create=create

    def __del__(self):
        if self.shmname!=None:
            if self.create:
                #utils.semdel(self.semid)
                if self.unlinkOnDel:
                    #os.unlink("/dev/shm"+self.shmname)
                    raise Exception("Need to destroy cond and mutex in buffer.py")

            #utils.unmap(self.buffer)#dodgy?
        self.buffer=None
    def assign(self,arr):
        arr=arr.view('c')
        if self.arr.size>arr.size:
            self.arr[:arr.size]=arr
        elif self.arr.size==arr.size:
            self.arr[:]=arr
        else:
            raise Exception("Array too large to assign")
        hdrsize=int(arr[:4].view(numpy.int32)[0])
        self.buffer=self.arr[hdrsize:]
        self.setNhdr()
        
    def save(self,fname):
        """Save the buffer (so that it can be loaded with loadBuf)"""
        FITS.Write(self.arr,fname)


    def setNhdr(self,nhdr=None):
        if nhdr==None:
            nhdr=self.nhdr[0]
        else:
            self.nhdr[0]=nhdr
        self.header=self.buffer[:self.nhdr[0]*self.hdrsize]
        self.labels=self.header[:16*self.nhdr[0]]
        self.labels.shape=(self.nhdr[0],16)
        self.blabels=self.labels.view("b")
        self.type=self.header[16*self.nhdr[0]:17*self.nhdr[0]]
        self.start=self.header[17*self.nhdr[0]:21*self.nhdr[0]].view("i")
        self.nbytes=self.header[21*self.nhdr[0]:25*self.nhdr[0]].view("i")
        self.ndim=self.header[25*self.nhdr[0]:29*self.nhdr[0]].view("i")
        self.shape=self.header[29*self.nhdr[0]:53*self.nhdr[0]].view("i")
        self.shape.shape=(self.nhdr[0],6)
        self.lcomment=self.header[53*self.nhdr[0]:57*self.nhdr[0]].view("i")


    def copy(self):
        b=Buffer(None,size=self.bufferSize,nhdr=self.nhdr[0])
        b.assign(self.arr)
        return b


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
        while i<self.nhdr[0] and self.blabels[i,0]!=0:
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
        name=name[:16]
        i=self.getIndex(name)
        val=self.makeval(i)
        if copy and type(val)==numpy.ndarray:
            val=val.copy()
        return val

    def swap(self,n1,n2,raiseError=1):
        """Swap the names of two values.  Must be of same size and type.
        """
        indx1=self.getIndex(n1)
        indx2=self.getIndex(n2)
        if self.type[indx1]==self.type[indx2] and self.nbytes[indx1]==self.nbytes[indx2]:
            #entries can be swapped
            tmp=self.labels[indx1].copy()
            self.labels[indx1]=self.labels[indx2]
            self.labels[indx2]=tmp
            rt=0
        else:
            if raiseError:
                raise Exception("Entries %s and %s cannont be swapped (%s, %s, %d, %d"%(n1,n2,self.type[indx1],self.type[indx2],self.nbytes[indx1],self.nbytes[indx2]))
            rt=1
        return rt

    def remove(self,name):
        """Remove an entry from the param buffer"""
        name=name[:16]
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
        name=name[:16]
        i=self.getIndex(name)
        if self.lcomment[i]>0:
            return self.buffer[self.start[i]+self.nbytes[i]:self.start[i]+self.nbytes[i]+self.lcomment[i]].tostring()
        else:
            return ""

    def getIndex(self,name,raiseerror=1):
        name=name[:16]
        l=len(name)
        self.tmpname[:l]=name
        self.tmpname.view("b")[l:]=0
        for i in range(self.nhdr[0]):
            if numpy.alltrue(self.labels[i]==self.tmpname):
                return i
        if raiseerror:
            raise Exception("name %s not found in buffer"%name)
        return None
    def makeval(self,indx):
        if self.type[indx] in numpy.typecodes["All"]+'c':
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
        self.set(name,val,ignoreLock=1)

    def prepareVal(self,val):
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
            rt=val,bytes,dtype,shape
        elif type(val)==type(""):
            if len(val)==0:
                val="\0"
            elif val[-1]!="\0":
                val+="\0"
            bytes=len(val)
            dtype='s'
            shape=()
            rt=val,bytes,dtype,shape
        elif type(val)==type(None):
            bytes=0
            dtype='n'
            shape=()
            rt=val,bytes,dtype,shape
        else:
            rt=None
        return rt
    def set(self,name,val,ignoreLock=0,comment=""):
        if name in ["switchRequested"]:
            pass
        if ignoreLock==0 and self.shmname!=None:
            #utils.semop(self.semid,0,0)#wait for the buffer to be unfrozen.
            #Check the freeze bit - if set, block on the condition variable.
            if self.arr is not None: 
                while int(self.arr[8:12].view(numpy.int32)[0])==1:#
                    # buffer is currently frozen - wait for it to unblock
                    print "Waiting for buffer to unfreeze in buffer.py set(%s)"%name
                    try:
                        utils.pthread_mutex_lock_cond_wait(self.cond,self.condmutex,1.0,1)
                    except:
                        print "Error in pthread_mutex_lock_cond_wait - in buffer.set - continuing"
                    #utils.pthread_mutex_lock(self.condmutex)
                    #try:
                    #    utils.pthread_cond_timedwait(self.cond,self.condmutex,1.0,1)
                    #except:
                    #    print "Error in utils.pthread_cond_timedwait in buffer.set - continuing"
                    #utils.pthread_mutex_unlock(self.condmutex)
        if type(comment)==type(""):
            lcom=len(comment)
        else:
            lcom=0
        name=name[:16]

        rt=self.prepareVal(val)
        if rt==None:
            raise Exception("Type %s not known for %s (numpy.int32 is %s,%d)"%(type(val),name,numpy.int32,type(val)==numpy.int32))
        val,bytes,dtype,shape=rt
        indx=self.getIndex(name,raiseerror=0)
        if indx==None:#insert a new value.
            indx=self.newEntry(name)
            if indx==None:
                #self.unfreezeContents()
                print "Entries:",self.nhdr[0],self.getNEntries()
                msg="buffer.set Unable to create new entry %s"%name
                if self.nhdr[0]==self.getNEntries():
                    print "No free buffer entries.  Try running darccontrol with --nhdr=X where X is something larger than %d"%self.nhdr[0]
                    msg+=".  No free buffer entries.  Try running darccontrol with --nhdr=X where X is something larger than %d"%self.nhdr[0]
                raise Exception(msg)
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
        name=name[:16]
        n=self.getNEntries()
        #print "%d entries"%n
        if n>=self.nhdr[0]:
            return None
        indx=n
        self.nbytes[indx]=0
        l=len(name)
        self.blabels[indx,l:]=0
        self.labels[indx,:l]=name
        return indx

    def getSpace(self,bytes):
        """find space in the buffer for this many bytes..."""
        if bytes==0:
            return 0
        s=(int(self.nhdr[0]*self.hdrsize+self.align-1)/self.align)*self.align
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

    def getMem(self,includeArrHdrSize=0,includeNames=1):
        """Finds how much space the buffer is using from start to finish (i.e. including any free space inbetween)"""
        n=self.getNEntries()
        if includeNames:
            mem=(int(self.nhdr[0]*self.hdrsize+self.align-1)/self.align)*self.align
        else:#only want the size of the actual data.
            mem=0
        for i in range(n):
            mem=max(mem,self.start[i]+self.nbytes[i]+self.lcomment[i])
        if includeArrHdrSize:
            mem+=self.arrhdrsize[0]
        return mem


class BufferSequence:
    """A class to implement creating a buffer sequence for the rtc.
    """
    def __init__(self):
        self.buf={}
        self.add("bufferUseSeq",1,0,inactive=1)

    def add(self,name,value,iteration,inactive=0,singleIter=0):
        """Uses temporary working space to create the sequence"""
        iteration=int(iteration)
        if not self.buf.has_key(iteration):
            self.buf[iteration]={}
        d=self.buf[iteration]
        d[name]=(value,inactive)
        if singleIter:
            raise Exception("singleIter not yet implemented")
        if singleIter:
            #Now switch back to the previous value.
            iteration+=1
            if not self.buf.has_key(iteration):
                self.buf[iteration]={}
            d=self.buf[iteration]
            d[name]=("PREVIOUSVALUE",inactive)#not yet known...
        
    def makeBuffer(self,niter=None,checkbuf=None):
        """Convert the working space into a buffer that will be understood by the rtc
        If niter!=None, and is greater than the largest iteration specified in add(), this will be the number of iterations before the buffer wraps around.
        If check!=None should be a Buffer instance containing the values to check against (e.g. copied from the rtc)
"""
        b=Buffer(None,nhdr=0,size=0)
        hdrsize=b.hdrsize
        if checkbuf!=None:
            import Check
            c=Check.Check()
            
        keys=self.buf.keys()
        keys.sort()
        hdr=numpy.zeros((4,),numpy.int32)
        txt=""

        for i in range(len(keys)):
            k=keys[i]
            buf=self.buf[k]
            nhdr=len(buf)
            #compute the size of these entries.
            size=hdrsize*nhdr+b.arrhdrsize[0]
            names=buf.keys()
            names.sort()
            for name in names:
                val,which=buf[name]
                if val=="PREVIOUSVALUE":
                    raise Exception("PREVIOUS VALUE NOT YET IMPLEMENTED")
                if checkbuf!=None:
                    val=c.valid(name,val,checkbuf)
                rt=b.prepareVal(val)
                if rt==None:
                    raise Exception("Unknown type %s for %s"%(type(val),name))
                sval,bytes,dtype,shape=rt
                size+=bytes+b.align
                                    
            pbuf=Buffer(None,size=size,nhdr=nhdr)
            #this invalidates the dimension stuff, but for this thats okay.
            for name in names:#buf.keys():
                val,which=buf[name]
                if checkbuf!=None:
                    val=c.valid(name,val,checkbuf)
                pbuf.set(name,val,comment="1"*which)#zero length comment->active buffer...
                indx=pbuf.getIndex(name)
            #Now get the buffer as string...
            if k==keys[-1]:#last entry...
                if niter!=None and niter>k:
                    rpt=niter-k
                else:
                    rpt=1
            else:
                rpt=keys[i+1]-k
            hdr[0]=hdr.size*hdr.itemsize
            hdr[1]=nhdr
            hdr[2]=rpt
            hdr[3]=hdr[0]+pbuf.getMem()
            txt+=hdr.tostring()
            txt+=pbuf.buffer[:pbuf.getMem()].tostring()
        return numpy.fromstring(txt,dtype='b')

    def decodeBuffer(self,arr):
        offset=0
        pdict={}
        iteration=0
        hdrsize=Buffer(None,nhdr=0,size=0).hdrsize
        while offset<arr.size:
            hsize=int(arr[offset:offset+4].view(numpy.int32)[0])
            iarr=arr[offset:offset+hsize].view(numpy.int32)
            barr=arr[offset+iarr[0]:offset+iarr[3]]
            b=Buffer(None,size=barr.size+hdrsize,nhdr=iarr[1])
            b.buffer[:barr.size*barr.itemsize]=barr.view('c')
            d={}
            for l in b.getLabels():
                d[l]=(b.get(l),int(b.getComment(l)!=""))
            pdict[iteration]=d#arr[offset:offset+iarr[3]]
            iteration+=iarr[2]
            offset+=iarr[3]
        return pdict,iteration
    def decodeHeaders(self,arr):
        offset=0
        iteration=0
        l=[]
        while offset<arr.size:
            iarr=arr[offset:offset+16].view(numpy.int32)
            l.append(iarr)
            offset+=iarr[3]
        return l

def getAlign():#for the circular buffers.
    # warning - don't change this from 8 without looking at the computation of nstore in self.reshape()
    return 8
def getCircHeaderSize():
    align=getAlign()
    return ((8+4+4+4+2+1+1+4+2*4+4+8+4+4+4+4+4+utils.pthread_sizeof_mutexcond()[0]+utils.pthread_sizeof_mutexcond()[1]+align-1)/align)*align #header contains buffer size (int64), last written to (int32), freq (int32), nstore (int32), forcewriteall(int8),ndim (int8),  dtype (int8), forcewrite (int8), shape (6*int32) pid (int32), circhdrsize (int32) circsignal (int8), 3 spare (int24), mutex size(int32), cond size(int32), mutex, cond

class Circular:
    """A class to implement a circular buffer.  Only the owner ever writes to this buffer, except for the freq entry.
    upon initialisation, the buffer size is set.

    """
    def __init__(self,shmname,owner=0,dims=None,dtype=None,nstore=None,raw=0,dirname="/dev/shm"):
        self.align=getAlign()
        
        self.shmname=shmname
        self.dirname=dirname
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
            self.hdrsize=getCircHeaderSize()
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
            self.size=os.stat(self.dirname+shmname).st_size
            if self.size==0:
                time.sleep(0.1)
                self.size=os.stat(self.dirname+shmname).st_size
            if self.size==0:
                raise Exception("Zero size array %s"%shmname)
            
            #utils.unmap(buf)

        #self.buffer=utils.open((self.size,),"b",shmname,owner)
            
        self.buffer=numpy.memmap(self.dirname+shmname,"b",mode,shape=(self.size,))
        #now get the hdrsize
        self.circsignal=self.buffer[56:57]
        if owner:
            self.buffer[:]=0
            self.buffer[60:68].view(numpy.int32)[:]=utils.pthread_sizeof_mutexcond()
            self.buffer[52:56].view(numpy.int32)[0]=self.hdrsize
            self.buffer[48:52].view(numpy.int32)[0]=os.getpid()
        else:
            #if not the owner, wait until the buffer has been initialised properly... one of the last things to be done is that the writer will set ndim...
            i=0
            while i<1000 and int(self.buffer[21:22])==0:
                i+=1
                time.sleep(0.001)
            if i==1000:
                print "ERROR - buffer ndim not initialised - buffer %s, ndim=%d"%(shmname,int(self.buffer[21:22]))
                raise Exception("ERROR - buffer ndim not initialised - buffer %s, ndim=%d"%(shmname,int(self.buffer[21:22])))

            self.hdrsize=int(self.buffer[52:56].view(numpy.int32)[0])
        msize=int(self.buffer[60:64].view(numpy.int32)[0])
        self.futex=self.buffer[64:64+msize]
        self.ownerPid=self.buffer[48:52].view(numpy.int32)
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
            self.bufsize[0]=self.size#this should never change now...
            self.lastWritten[0]=-1
            self.freq[0]=0
            self.nstore[0]=nstore
            self.ndim[0]=len(dims)
            self.dtype[0]=dtype
            self.forcewrite[0]=0
            self.circsignal[0]=0
            self.shapeArr[:]=-1
            self.shapeArr[:self.ndim[0]]=dims
            utils.darc_futex_init(self.futex)
            #utils.initSemaphore(self.semid,0,1)#initialise so that something can block on it waiting for a zero.
        else:
            #If is possible that the array isn't initialised yet - which might manifest itself with ndim==0.  So wait to see if this is the case.
            n=0
            while self.ndim[0]==0 and n<100:
                n+=1
                time.sleep(0.01)
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
        #print "Circular.__del__ Deleting buffer object %s"%self.shmname
        #utils.unmap(self.buffer)#dodgy?
        #self.buffer.close()#I think this unmaps it.
        self.buffer=None
        if self.owner:
            #utils.unlink(self.shmname)
            os.unlink(self.dirname+self.shmname)
    def setForceWrite(self,val=1):
        """Called to force a write to the buffer, even if one is not due (ie not decimated yet)
        """
        self.forcewrite[0]=val

    def setForceWriteAll(self,val=1):
        """If this is set in the first buffer written (rtcPxlBuf), then the RTC will set forceWrite in all other buffers.
        """
        self.forcewriteall[0]=val

    def add(self,data,timestamp,frameno):
        """Should be called for owner only.  data is an array or a list of arrays
        Note - this is untested on 32 bit machines... (int32 dtype on these mathines is l instead of the expected i)"""
        if (self.freq[0]>0 and frameno%self.freq[0]==0) or self.forcewrite[0]!=0:#add to the buffer
            if self.forcewrite[0]>0:
                self.forcewrite[0]-=1
            indx=self.lastWritten[0]+1
            if(indx>=self.nstore[0]):
                indx=0
            self.data[indx]=data.view(self.data.dtype.char)
            self.datasizearr[indx]=self.datasize*self.elsize+32-4
            self.frameNo[indx]=frameno
            self.timestamp[indx]=timestamp
            self.datatype[indx]=data.dtype.char
            self.lastWritten[0]=indx
            utils.darc_futex_broadcast(self.futex)
        return 0

            # self.freqcnt=0
            # if type(data)==type([]):
            #     size=0
            #     dtype=data[0].dtype.char
            #     for d in data:
            #         size+=d.size
            #         if dtype!=d.dtype.char:
            #             raise Exception("Not all data in list are same type in circular buffer")
            #     if self.ndim[0]!=1 or self.shapeArr[0]!=size or dtype!=self.dtype[0]:
            #         raise Exception("Data list sizes/types don't match for circular buffer %s %s %s %s %s"%(str(self.ndim[0]),str(self.shapeArr[0]),str(size),str(dtype),str(self.dtype[0])))
            # else:
            #     if data.shape!=tuple(self.shapeArr[:self.ndim[0]]) or data.dtype.char!=self.dtype[0]:
            #         raise Exception("Data sizes/types don't match for circular buffer %s %s %s %s"%(str(data.shape),str(self.shapeArr),str(data.dtype.char),self.dtype))
            # indx=self.lastWritten[0]+1
            # if indx==self.nstore[0]:
            #     indx=0
            # if type(data)==type([]):
            #     f=0
            #     for d in data:
            #         t=f+d.size
            #         self.data[indx,f:t]=d.ravel()
            #         f=t
            # else:
            #     self.data[indx]=data
            # self.timestamp[indx]=time.time()
            # self.datasizearr[indx]=self.datasize*self.elsize+32-4
            # self.datatype[indx]=self.dtype[0]
            # self.frameNo[indx]=self.framecnt
            # #now update the lastWritten flag.
            # self.lastWritten[0]=indx
            # #and signal that there is a new entry...
            # #utils.semop(self.semid,0,-1)#decrease to unblock waiting process
            # utils.initSemaphore(self.semid,0,0)#reinitialise
            # utils.initSemaphore(self.semid,0,1)#reinitialise

    def reshape(self,dims,dtype):
        """Should be called for owner only.  Reshape the array.  The shared memory array (self.buffer) remains the same, so readers don't have to reopen it."""
        self.ndim[0]=len(dims)
        self.dtype[0]=dtype
        self.shapeArr[:]=-1
        self.shapeArr[:len(dims)]=dims

        self.lastWritten[0]=-1
        #now work out nstore...
        databytes=(int(reduce(lambda x,y:x*y,dims)*numpy.zeros((1,),dtype).itemsize+32+self.align-1)//self.align)*self.align
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
            self.frameSize=(int(self.datasize*self.elsize+32+self.align-1)//self.align)*self.align
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
        self.circsignal[0]=1#signal that we are using the buffer
            
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
        #print "getnextfrmae"
        data=None
        self.circsignal[0]=1#signal that we are using the buffer
        while data==None:
            if self.makeDataArrays():#have been remade...
                self.lastReceived=-1
                self.lastReceivedFrame=-1
                #print "made"

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
                    try:
                        if timeout:
                            timeup=utils.darc_futex_timedwait(self.futex,timeout)
                        else:
                            timeup=utils.darc_futex_wait(self.futex)
                    except:
                        print "Error in utils.darc_futex_timedwait in buffer.getNextFrame - continuing"
                        timeup=1
                    #utils.pthread_mutex_lock(self.condmutex)
                    if self.buffer[0:8].view(numpy.int64)==0:
                        raise Exception("Circlar buffer size is zero - probably means buffer is no longer in existance: %s"%self.shmname)
                    #try:
                    #    if timeout==0:
                    #        #print "cond_wait"
                    #        utils.pthread_cond_wait(self.cond,self.condmutex)
                    #        #print "waited"
                    #        timeup=0
                    #    else:
                    #        timeup=utils.pthread_cond_timedwait(self.cond,self.condmutex,timeout,1)
                    #except:
                    #    print "Error in utils.pthread_cond_(timed)wait in buffer.getNextFrame - continuing"
                    #utils.pthread_mutex_unlock(self.condmutex)
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
                    #print "timeout - retrying %d"%retry,self.lastReceived,self.lastReceivedFrame,self.lastWritten,self.frameNo,timeup,timeout
                    data=None
                    if retry==0 and timeout!=0.:#changed from timeout==0 to != on 16th Dec 2014.
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
