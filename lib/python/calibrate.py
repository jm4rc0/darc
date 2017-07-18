import time
import numpy
import darc
import FITS
def applyBrightest(img,ub,subapLocation):
    subapLocation.shape=subapLocation.size//6,6
    if type(ub)==type(0):
        ub=numpy.ones(subapLocation.shape[0],numpy.int32)*ub
    for i in range(subapLocation.shape[0]):
        s=subapLocation[i]
        if s[2]!=0 and s[5]!=0:
            im=img[s[0]:s[1]:s[2],s[3]:s[4]:s[5]]
            tmp=im.copy().ravel()
            tmp.sort()
            if ub[i]!=0:
                j=tmp.size-numpy.abs(ub[i])
                thr=tmp[j]
                if ub[i]<0:
                    thr2=thr
                    while thr2==thr and j>1:
                        j-=1
                        thr2=tmp[j]
                    im[:]=numpy.where(im<thr,0,im-thr2)
                else:
                    im[:]=numpy.where(im<thr,0,im)
    return img

def makeSubapLocation(nsubx,nsuby,xoff,yoff,xstep,ystep):
    sl=numpy.zeros((nsuby,nsubx,6),numpy.int32)
    for i in range(nsuby):
        for j in range(nsubx):
            sl[i,j]=(yoff+i*ystep,yoff+(i+1)*ystep,1,xoff+j*xstep,xoff+(j+1)*xstep,1)
    sl.shape=sl.size//6,6
    return sl

def makeSlopes(img,ncam,nsub,npxlx,npxly,sf,sl,camlist=None):
    """Need to subtract 0.5 from the results if a correlation image."""
    if camlist==None:
        camlist=range(ncam)
    elif type(camlist)!=type([]):
        camlist=[camlist]
    sl.shape=sl.size//6,6
    slopes=numpy.zeros((sf.sum()*2),numpy.float32)
    soff=0
    start=0
    pos=0
    npxl=numpy.array(npxlx)*numpy.array(npxly)
    for i in range(ncam):
        thisimg=img[start:start+npxl[i]]
        thisimg.shape=npxly[i],npxlx[i]
        for j in range(nsub[i]):
            if sf[soff+j]:
                if i in camlist:
                    s=sl[soff+j]
                    im=thisimg[s[0]:s[1]:s[2],s[3]:s[4]:s[5]]
                    tot=im.sum()
                    if tot>0:
                        x=(im.sum(0)*numpy.arange(im.shape[1])).sum()
                        y=(im.sum(1)*numpy.arange(im.shape[0])).sum()
                        x/=tot
                        y/=tot
                    else:
                        x=0
                        y=0
                    slopes[pos*2]=x-im.shape[1]//2+0.5#Is this a bug?  If tot==0, will result in slopes being 0.5-im.shape
                    slopes[pos*2+1]=y-im.shape[0]//2+0.5
                pos+=1
        soff+=nsub[i]
        start+=npxl[i]
    return slopes


class DMInteraction:
    def __init__(self,nactList=None,prefix=""):
        """nactList: List of number of actuators of each DM.
        prefix: darc prefix
        """
        if nactList is None:
            d=darc.Control(prefix)
            nactList=[d.Get("nacts")]
            print "Assuming 1 DM"
        if type(nactList)==type(0):
            nactList=[nactList]
            print "Assuming 1 DM"
        self.nactList=nactList
        self.prefix=prefix

    def pokeSimple(self,dmNo,pokeval,actNos=None,nAv=10,setInDarc=0,cond=0.01,delay=0):
        """
        Performs a basic push-pull poke.
        More complications could poke patterns of actuators, e.g.a Hadamard matrix, sinusoids, etc.

        Inefficient implementation - in a real system, you probably want to set actuators to a 2D array, with dimensions X,nacts.  Darc will then play this sequence, repeating when it gets to the end.  You can then record a sequence of slopes."""
        print "Assuming system is in calibration mode (light sources etc) with valid reference slopes.  actuators should be set to mid-range."
        print "Assuming response of system is fast enough that no delay is required between poking and recording slopes.  Note - in simulation, this may not be the case"
        d=darc.Control(self.prefix)
        nslopes=d.Get("subapFlag").sum()*2
        nacts=self.nactList[dmNo]
        pmx=numpy.zeros((nacts,nslopes),numpy.float32)
        d.Set("addActuators",0)
        actuators=d.Get("actuators")
        offset=sum(self.nactList[:dmNo])
        if actNos is None:
            actNos = range(nacts) # all actuators
        for i in range(nacts):
            if not i in actNos:
                continue 
            print "Poking %d/%d"%(i,nacts)
            #push
            actuators[i+offset]+=pokeval
            d.Set("actuators",actuators)
            #record
            if delay!=0:
                time.sleep(delay)
            sl=d.SumData("rtcCentBuf",nAv)[0]/nAv/pokeval
            #pull
            actuators[i+offset]-=2*pokeval
            d.Set("actuators",actuators)
            #record
            if delay!=0:
                time.sleep(delay)
            sl2=d.SumData("rtcCentBuf",nAv)[0]/nAv/pokeval
            #reset
            actuators[i+offset]+=pokeval
            #store
            pmx[i]=(sl-sl2)/2.
        d.Set("actuators",actuators)
        if setInDarc:
            rmx=self.reconstruct(pmx,cond)
            fullrmx=d.Get("rmx")
            fullrmx[offset:offset+nacts]=rmx
            print "Setting rmx in darc"
            d.Set("rmx",fullrmx)
        return pmx

    def takeRefSlopes(self,nAv=10,setInDarc=0):
        d=darc.Control(self.prefix)
        ref=d.Get("refCentroids")
        d.Set("refCentroids",None)
        time.sleep(1)
        sl=d.SumData("rtcCentBuf",nAv)[0]/nAv
        if setInDarc:
            d.Set("refCentroids",sl)
        else:#reset back to what they were.
            d.Set("refCentroids",ref)
        return sl

    def reconstruct(self,pmx,cond=0.01,scale=None):
        """If scale is a list of the DM scales (i.e. the pokeval scales), then will scale the matrix by these before inverting, and rescale afterwards"""
        ndm=len(self.nactList)
        if scale!=None:
            offset=0
            for i in range(ndm):
                pmx[offset:offset+self.nactList[i]]*=scale[i]
                offset+=self.nactList[i]
        rmx=-numpy.linalg.pinv(pmx,cond).T.copy()
        if scale!=None:
            offset=0
            for i in range(ndm):
                rmx[offset:offset+self.nactList[i]]*=scale[i]
                offset+=self.nactList[i]
            
        return rmx

    def pokeAll(self,pokevalList,nAv=10,setInDarc=0,cond=0.01,scale=None,delay=0):
        """Very simple, does a global least squares fit to produce control matrix, if required.
        If scale==1, will scale poke matrix and rmx by pokevalList."""
        ndm=len(self.nactList)
        d=darc.Control(self.prefix)
        pmx=d.Get("rmx")
        offset=0
        for i in range(ndm):
            nact=self.nactList[i]
            dmpmx=self.pokeSimple(i,pokevalList[i],nAv=nAv,delay=delay)
            pmx[offset:offset+nact]=dmpmx
            offset+=nact
        if setInDarc:
            if scale==0:
                scale=None
            if scale!=None:
                scale=pokevalList
            rmx=self.reconstruct(pmx,cond,scale=scale)
            d.Set("rmx",rmx)
        return pmx

    def openLoop(self,prefix=""):
        d=darc.Control(prefix)
        d.Set("addActuators",0)

    def closeLoop(self,prefix=""):
        d=darc.Control(prefix)
        d.Set("addActuators",1)


    def measureLinearity(self,actno,actmin,actmax,nsteps=50,nAv=1,delay=0.5):
        d=darc.Control(self.prefix)
        actuators=d.Get("actuators")
        nslopes=d.Get("subapFlag").sum()*2
        orig=actuators[actno]
        step=(actmax-actmin)/nsteps
        res=numpy.zeros((nsteps,nslopes),numpy.float32)
        for i in range(nsteps):
            actuators[actno]=actmin+i*step
            d.Set("actuators",actuators)
            if delay!=0:
                time.sleep(delay)
            sl=d.SumData("rtcCentBuf",nAv)[0]/nAv
            res[i]=sl
        actuators[actno]=orig
        d.Set("actuators",actuators)
        return res,numpy.arange(nsteps)*step+actmin

    def measureLatency(self,actno,amp,nsteps,nrecord):
        d=darc.Control(self.prefix)
        acts=d.Get("actuators")
        acts2=numpy.zeros((nsteps,acts.size),numpy.float32)
        acts2[:]=acts.ravel()
        acts2[:,actno]+=amp*numpy.sin(numpy.arange(nsteps)/float(nsteps)*2*numpy.pi)
        d.Set("actuators",acts2)
        data=d.GetStreamBlock(["rtcCentBuf","rtcActuatorBuf"],nrecord,asArray=1)
        d.Set("actuators",acts)
        return data
