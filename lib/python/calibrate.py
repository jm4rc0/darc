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
            if nAv==1:
                sl=d.GetStream("rtcCentBuf")[0]/pokeval
            else:
                sl=d.SumData("rtcCentBuf",nAv)[0]/nAv/pokeval
            #pull
            actuators[i+offset]-=2*pokeval
            d.Set("actuators",actuators)
            #record
            if delay!=0:
                time.sleep(delay)
            if nAv==1:
                sl2=d.GetStream("rtcCentBuf")[0]/pokeval
            else:
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
        s=data["rtcCentBuf"][0]
        a=data["rtcActuatorBuf"][0]
        delay=self.computeSineDelay([a],[s],actno,sindx=None)

        return delay,data

    
    def pokeSine(self,nFrames,pokeVal=1000.,dmNo=0,baseFreq=5,nrec=2,fname="sinePoke.fits",fno=-20,order=None,nRepeats=1,repeatIfErr=10):
        """nFrames - number of iterations over which to poke.
        baseFreq - minimum number of sine waves to fit within nFrames.
        nrec - number of cycles to record (for averaging purposes)
        fno - number of frames to allow for sync purposes.  Make this more negative for faster systems (or closer to zero for simulation!).
        order - if None, will increase the freq of neighbouring actuators in a linear fashion.  Otherwise, can be the index order in which this should be done.
        nRepeats - number of times to repeat the recording, with an offset added to the frequency of each actuator each time (so that different dynamics can be explored).
        repeatIfErr - number of times to repeat if an error is obtained, before giving up.  If -ve, will repeat indefinitely

        Use processSine() function below to process this data...
        """
        if order is not None:
            raise Exception("order not yet implemented - sorry!")
        d=darc.Control(self.prefix)
        nslopes=d.Get("subapFlag").sum()*2
        nacts=d.Get("nacts")#self.nactList[dmNo]
        actOrig=d.Get("actuators")
        pokeArr=numpy.zeros((nFrames,nacts),numpy.float32)
        nactDm=self.nactList[dmNo]
        offset=sum(self.nactList[:dmNo])
        writeMode="w"
        freqOffset=0.
        dataList=[]
        errList=[]
        extraHeader=["POKEVAL = %d"%pokeVal,"NFRAMES = %d"%nFrames,"DMNUMBER= %d"%dmNo,"BASEFREQ= %d"%baseFreq,"NRECORD = %d"%nrec]
        for rpt in range(nRepeats):
            err=0
            pokeArr[:]=actOrig
            for i in range(nactDm):
                freq=baseFreq+(freqOffset+i)%nactDm
                pokeArr[:,offset+i]+=numpy.sin(numpy.arange(nFrames)/float(nFrames)*2.*numpy.pi*freq)*pokeVal
            d.Set("actuators",pokeArr)
            time.sleep(1.)
            takeData=repeatIfErr
            if takeData==0:
                takeData=1
            while takeData:
                err=0
                data=d.GetStreamBlock(["rtcActuatorBuf","rtcCentBuf"],nFrames*nrec,asArray=1,fno=fno)
                #check the data is okay.
                for key in ["rtcCentBuf","rtcActuatorBuf"]:
                    f=data[key][2]
                    if not numpy.alltrue((f[1:]-f[:-1])==1):
                        print "Cycle %d: Some frames missing from %s"%(rpt,key)
                        err=1
                if not numpy.alltrue(data["rtcCentBuf"][2]==data["rtcActuatorBuf"][2]):
                    allframenumbersequal=0
                    print "Cycle %d: actuator and slope frame numbers don't agree"%rpt
                    err=1
                allframetimesequal=1
                for key in ["rtcCentBuf","rtcActuatorBuf"]:
                    ftime=data[key][1]
                    fdiff=ftime[1:]-ftime[:-1]
                    if not numpy.alltrue(fdiff<=numpy.median(fdiff)*3):
                        allframetimesequal=0
                        err=1
                        print "Cycle %d: Not all frame times within 3x median frame time"%rpt
                if err==0:#got data okay
                    takeData=0
                elif takeData>1:
                    print "Error in data - repeating acquisition (another %d times to try)"%takeData
                    takeData-=1
                elif takeData==1:
                    takeData=0
                    if repeatIfErr==0:
                        print "Error in data - continuing anyway"
                    else:
                        print "Error in data - cannot acquire (is your network fast enough?  Is the frame rate too high?)"
                else:
                    print "Error in data - repeating acquisition (will continue until successful...!)"
            if repeatIfErr!=0 and err!=0:
                raise Exception("Unable to capture data")
            #subtract the actuator offset (midrange)
            data["rtcActuatorBuf"][0]-=actOrig
            dataList.append(data)
            if fname is not None:
                FITS.Write(data["rtcCentBuf"][0],fname,writeMode=writeMode,extraHeader=extraHeader)
                writeMode="a"
                extraHeader=None
                FITS.Write(data["rtcCentBuf"][1],fname,writeMode="a")#times
                FITS.Write(data["rtcCentBuf"][2],fname,writeMode="a")#frameno
                FITS.Write(data["rtcActuatorBuf"][0],fname,writeMode="a")
                FITS.Write(data["rtcActuatorBuf"][1],fname,writeMode="a")
                FITS.Write(data["rtcActuatorBuf"][2],fname,writeMode="a")
            errList.append(err)
            freqOffset+=nactDm//nRepeats
        d.Set("actuators",actOrig)
        return dataList,errList


    def loadSineData(self,fname,stdThresh=0.25):
        """
        fname is the name of a file that has been created by e.g. doSinePokeGTC()
        stdThresh is the threshold above which slopes are ignored if their std is higher than the max std * thresh.  This allows us to automatically cut out slopes the don't have light."""
        sList=[]
        vList=[]
        data=FITS.Read(fname)
        for i in range(len(data)//12):
            s=data[1+i*12]#the slopes
            vmes=data[7+i*12]#the actuators (sinusoidal)
            if stdThresh>0:
                #define a mask of valid subaps based on those with low rms.
                sstd=s.std(0)#get standard deviation of each subap
                maxstd=sstd.max()#find the max...
                valid=numpy.where(sstd<maxstd*stdThresh,1,0)
                s*=valid
            sList.append(s)
            vList.append(vmes)
        return vList,sList

    def rollSineData(self,vList,sList):
        """Roll the data so that the phase becomes zero
        vList is the list of actuator arrays (i.e. as output from loadSineData).
        sList is the list of slope arrays."""
        v2=[]
        s2=[]
        for i in range(len(vList)):
            v=vList[i]
            s=sList[i]
            vgrad=(v[1:]-v[:-1]).sum(1)
            maxpos=vgrad.argmax()
            v=numpy.roll(v,-maxpos-1,0)
            s=numpy.roll(s,-maxpos-3,0)
            s2.append(s-s.mean(0))
            v2.append(v-v.mean(0))
        return v2,s2

    def makeSinePmx(self,vList,sList,pokeVal):
        """compute an interaction matrix.  pokeVal is the amplitude of the sine waves used for poking.  Returns a list of matrics, one for each sine-sequence."""
        pmxList=[]
        for i in range(len(vList)):
            pmx=numpy.dot(sList[i].T.astype("d"),vList[i].astype("d")).T/float(vList[i].shape[0])*2/(pokeVal**2)
            pmxList.append(pmx)
        return pmxList

    def makeSineRmx(self,pmxList,rcond,thresh=0.25):
        """rcond is the conditioning value.
        thresh is a threshold as a fraction of the std, below which the pmx is set to zero.
        """
        rmxList=[]
        for pmx in pmxList:
            if thresh!=0:
                pmx=numpy.where(numpy.abs(pmx)<pmx.std()*thresh,0,pmx)
            rmxList.append(-numpy.linalg.pinv(pmx,rcond).T.copy())
        return rmxList

    def computeSineDelay(self,vList,sList,vindx,sindx=None):
        """Computes frame delay between vList and sList.  """
        vt=vList[0][:,vindx]
        fv=numpy.fft.fft(vt)[:vt.shape[0]//2]
        vpeak=numpy.absolute(fv).argmax()
        if sindx is None:
            fs=numpy.fft.fft(sList[0],axis=0)
            sindx=numpy.argmax(numpy.absolute(fs[vpeak]))
            print "Got slope index at %d"%sindx
            fs=fs[:,sindx]
        else:
            st=sList[0][:,sindx]
            fs=numpy.fft.fft(st)[:st.shape[0]//2]
        speak=numpy.absolute(fs).argmax()
        if(vpeak!=speak):
            print "WARNING: FFT peaks not at same position %d %d"%(vpeak,speak)
            return None
        vphase=numpy.arctan2(fv.imag[vpeak],fv.real[vpeak])
        sphase=numpy.arctan2(fs.imag[speak],fs.real[speak])
        phaseDiff=vphase-sphase
        print "%f radians, %f cycles"%(phaseDiff,phaseDiff/2./numpy.pi)
        nsamp=vList[0].shape[0]
        ncycles=vpeak
        framesPerCycle=nsamp/ncycles
        framesDelay=framesPerCycle*phaseDiff/2./numpy.pi
        print "Delay: %f frames"%framesDelay
        return framesDelay#if +ve, need to roll slopes -ve or acts +ve.

    def computeSineDelays(self,vList,sList):
        """Compute mean frame delay between slopes and actuators."""
        delayList=[]
        for indx in range(vList[0].shape[1]):
            delay=self.computeSineDelay(vList,sList,indx)
            if delay!=None:
                delayList.append(delay)
        delays=numpy.array(delayList)
        indx=numpy.where(numpy.abs(delays-numpy.median(delays))<0.5)
        meandelay=delays[indx].mean()
        stddelay=delays[indx].std(ddof=1)
        print "Mean delay (ignoring outliers) %g +- %g frames"%(meandelay,stddelay)
        return delays,meandelay,stddelay#if +ve, need to roll slopes -ve or acts +ve.

    def shiftSineActuators(self,vList,delay):
        """shift actuators by non-integer delay - given by the return from computeSineDelays()"""
        delay1=int(numpy.floor(delay))
        delay2=int(numpy.ceil(delay))
        frac=delay%1

        outList=[]
        for s in vList:
            s1=numpy.roll(s,delay1,0)
            s2=numpy.roll(s,delay2,0)
            snew=s1*(1-frac)+s2*(frac)
            outList.append(snew)
        return outList

    def shiftSineSlopes(self,sList,delay):
        """shift slopes by non-integer delay as given by computeSineDelays().  Either use this or shiftSineActuators, but not both...  and looking at created interaction matres, there appears to be very little difference between which one to choose."""
        return self.shiftSineActuators(sList,-delay)


    def processSine(self,fname,rcond=0.1,stdThresh=0.25,pokeVal=1000,pmxThresh=0.5):
        """Uses the file created by pokeSine() to compute the rmx.
        fname - the file containing the slopes and actuator values.
        rcond - conditioning value for pseudo-inverse of interaction matrix.
        stdThresh - used to define which are active sub-apertures (the larger this value, the more sub-apertures will be used).
        pokeVal - the amplitude used during poking.
        pmxThresh - a threshold used for cleaning the interaction matrix (larger values will result in more of the pmx being set to zero)."""
        vList,sList=self.loadSineData(fname,stdThresh)
        vList,sList=self.rollSineData(vList,sList)
        delays,meandelay,stddelay=self.computeSineDelays(vList,sList)
        vList=self.shiftSineActuators(vList,meandelay)
        pmxList=self.makeSinePmx(vList,sList,pokeVal)
        rmxList=self.makeSineRmx(pmxList,rcond,pmxThresh)
        rmx=numpy.array(rmxList).mean(0)
        return rmx,pmxList,rmxList

