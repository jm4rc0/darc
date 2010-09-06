#This is a configuration file for CANARY.
#Aim to fill up the control dictionary with values to be used in the RTCS.

#import correlation
import FITS
import tel
import numpy
NCAMERAS=4#1, 2, 3, 4.  This is the number of physical cameras

nacts=54#97#54#+256
ncam=(int(NCAMERAS)+1)/2
camPerGrab=numpy.ones((ncam,),"i")*2
if NCAMERAS%2==1:
    camPerGrab[-1]=1
ncamThreads=numpy.ones((ncam,),numpy.int32)*1
npxly=numpy.zeros((ncam,),numpy.int32)
npxly[:]=128
npxlx=npxly.copy()*camPerGrab
nsuby=npxly.copy()
nsuby[:]=7
nsubx=nsuby.copy()*camPerGrab
nsubaps=(nsuby*nsubx).sum()
individualSubapFlag=tel.Pupil(7,3.5,1,7).subflag.astype("i")
subapFlag=numpy.zeros(((nsuby*nsubx).sum(),),"i")
for i in range(ncam):
    tmp=subapFlag[(nsuby[:i]*nsubx[:i]).sum():(nsuby[:i+1]*nsubx[:i+1]).sum()]
    tmp.shape=nsuby[i],nsubx[i]
    for j in range(camPerGrab[i]):
        tmp[:,j::camPerGrab[i]]=individualSubapFlag
#ncents=nsubaps*2
ncents=subapFlag.sum()*2
npxls=(npxly*npxlx).sum()

fakeCCDImage=None#(numpy.random.random((npxls,))*20).astype("i")
#camimg=(numpy.random.random((10,npxls))*20).astype(numpy.int16)

bgImage=None#FITS.Read("shimgb1stripped_bg.fits")[1].astype("f")#numpy.zeros((npxls,),"f")
darkNoise=None#FITS.Read("shimgb1stripped_dm.fits")[1].astype("f")
flatField=None#FITS.Read("shimgb1stripped_ff.fits")[1].astype("f")
#indx=0
#nx=npxlx/nsubx
#ny=npxly/nsuby
#correlationPSF=numpy.zeros((npxls,),numpy.float32)


subapLocation=numpy.zeros((nsubaps,6),"i")
nsubapsCum=numpy.zeros((ncam+1,),numpy.int32)
ncentsCum=numpy.zeros((ncam+1,),numpy.int32)
for i in range(ncam):
    nsubapsCum[i+1]=nsubapsCum[i]+nsuby[i]*nsubx[i]
    ncentsCum[i+1]=ncentsCum[i]+subapFlag[nsubapsCum[i]:nsubapsCum[i+1]].sum()*2

# now set up a default subap location array...
#this defines the location of the subapertures.
subx=(npxlx-16*camPerGrab)/nsubx
suby=(npxly-16)/nsuby
xoff=[8]*ncam
yoff=[8]*ncam
for k in range(ncam):
    nc=camPerGrab[k]
    for i in range(nsuby[k]):
        for j in range(nsubx[k]):
            indx=nsubapsCum[k]+i*nsubx[k]+j
            if subapFlag[indx]:
                subapLocation[indx]=(yoff[k]+i*suby[k],yoff[k]+i*suby[k]+suby[k],1,xoff[k]*nc+j/nc*subx[k]*nc+j%nc,xoff[k]*nc+j/nc*subx[k]*nc+subx[k]*nc+j%nc,nc)

pxlCnt=numpy.zeros((nsubaps,),"i")
# set up the pxlCnt array - number of pixels to wait until each subap is ready.  Here assume identical for each camera.
for k in range(ncam):
    # tot=0#reset for each camera
    for i in range(nsuby[k]):
        for j in range(nsubx[k]):
            indx=nsubapsCum[k]+i*nsubx[k]+j
            n=(subapLocation[indx,1]-1)*npxlx[k]+subapLocation[indx,4]
            pxlCnt[indx]=n
pxlCnt[-5]=128*256
#pxlCnt[-6]=128*256
pxlCnt[nsubaps/2-5]=128*256
#pxlCnt[nsubaps/2-6]=128*256

#The params are dependent on the interface library used.
cameraParams=numpy.zeros((5*ncam+2,),numpy.int32)
cameraParams[0::5]=128*8#blocksize
cameraParams[1::5]=1000#timeout/ms
cameraParams[2::5]=range(ncam)#port
cameraParams[3::5]=0xffff#thread affinity
cameraParams[4::5]=2#thread priority
cameraParams[-2]=0#resync
cameraParams[-1]=1#wpu correction
cameraParams=numpy.fromstring("/home/ali/replay_damien.fits",dtype="i")

centroiderParams=numpy.zeros((5*ncam,),numpy.int32)
centroiderParams[0::5]=18#blocksize
centroiderParams[1::5]=1000#timeout/ms
centroiderParams[2::5]=range(ncam)#port
centroiderParams[3::5]=-1#thread affinity
centroiderParams[4::5]=1#thread priority
rmx=numpy.zeros((nacts,ncents)).astype("f")#FITS.Read("rmxRTC.fits")[1].transpose().astype("f")

mirrorParams=numpy.zeros((4,),"i")
mirrorParams[0]=1000#timeout/ms
mirrorParams[1]=2#port
mirrorParams[2]=-1#thread affinity
mirrorParams[3]=3#thread prioirty

#Now describe the DM - this is for the GUI only, not the RTC.
#The format is: ndms, N for each DM, actuator numbers...
#Where ndms is the number of DMs, N is the number of linear actuators for each, and the actuator numbers are then an array of size NxN with entries -1 for unused actuators, or the actuator number that will set this actuator in the DMC array.

dmDescription=numpy.zeros((8*8+2*2+2+1,),numpy.int16)
dmDescription[0]=2#1 DM
dmDescription[1]=2#1st DM has 2 linear actuators
dmDescription[2]=8#1st DM has nacts linear actuators
tmp=dmDescription[3:7]
tmp[:]=-1
tmp[:2]=[52,53]#tip/tilt
tmp=dmDescription[7:]
tmp[:]=-1
tmp.shape=8,8
dmflag=tel.Pupil(8,4,0).fn.ravel()
numpy.put(tmp,dmflag.nonzero()[0],numpy.arange(52))



control={
    "switchRequested":0,#this is the only item in a currently active buffer that can be changed...
    "pause":0,
    "go":1,
    "maxClipped":nacts,
    "refCentroids":None,
    "centroidMode":"CoG",#whether data is from cameras or from WPU.
    "windowMode":"basic",
    "thresholdAlgorithm":1,
    "reconstructMode":"simple",#simple (matrix vector only), truth or open
    "centroidWeighting":None,
    "v0":numpy.ones((nacts,),"f")*32768,#v0 from the tomograhpcic algorithm in openloop (see spec)
    "bleedGain":0.0,#0.05,#a gain for the piston bleed...
    "actMax":numpy.ones((nacts,),numpy.uint16)*65535,#4095,#max actuator value
    "actMin":numpy.zeros((nacts,),numpy.uint16),#4095,#max actuator value
    "nacts":nacts,
    "ncam":ncam,
    "nsuby":nsuby,
    "nsubx":nsubx,
    "npxly":npxly,
    "npxlx":npxlx,
    "ncamThreads":ncamThreads,
    "pxlCnt":pxlCnt,
    "subapLocation":subapLocation,
    "bgImage":bgImage,
    "darkNoise":darkNoise,
    "closeLoop":1,
    "flatField":flatField,#numpy.random.random((npxls,)).astype("f"),
    "thresholdValue":0.,#could also be an array.
    "powerFactor":1.,#raise pixel values to this power.
    "subapFlag":subapFlag,
    "fakeCCDImage":fakeCCDImage,
    "printTime":0,#whether to print time/Hz
    "rmx":rmx,#numpy.random.random((nacts,ncents)).astype("f"),
    "gain":numpy.ones((nacts,),"f"),
    "E":numpy.zeros((nacts,nacts),"f"),#E from the tomoalgo in openloop.
    "threadAffinity":None,
    "threadPriority":numpy.ones((ncamThreads.sum()+1,),numpy.int32)*10,
    "delay":100000,
    "clearErrors":0,
    "camerasOpen":1,
    "camerasFraming":1,
    "cameraName":"libcamfile.so",#"camfile",
    "cameraParams":cameraParams,
    "mirrorName":"libmirrorSL240.so",
    "mirrorParams":mirrorParams,
    "mirrorOpen":0,
    "frameno":0,
    "switchTime":numpy.zeros((1,),"d")[0],
    "adaptiveWinGain":0.5,
    "correlationThresholdType":0,
    "correlationThreshold":0.,
    "fftCorrelationPattern":None,#correlation.transformPSF(correlationPSF,ncam,npxlx,npxly,nsubx,nsuby,subapLocation),
#    "correlationPSF":correlationPSF,
    "nsubapsTogether":1,
    "nsteps":0,
    "addActuators":0,
    "actuators":None,#(numpy.random.random((3,52))*1000).astype("H"),#None,#an array of actuator values.
    "actSequence":None,#numpy.ones((3,),"i")*1000,
    "recordCents":0,
    "pxlWeight":None,
    "averageImg":0,
    "centroidersOpen":0,
    "centroidersFraming":0,
    "centroidersParams":centroiderParams,
    "centroidersName":"sl240centroider",
    "actuatorMask":None,
    "dmDescription":dmDescription,
    "averageCent":0,
    "centCalData":None,
    "centCalBounds":None,
    "centCalSteps":None,
    "figureOpen":0,
    "figureName":"figureSL240",
    "figureParams":numpy.array([1000,3,0xffff,2]).astype("i"),#timeout,port,affinity,priority
    "reconName":"libreconmvm.so",
    "fluxThreshold":0,
    "printUnused":1,
    "useBrightest":0,
    "figureGain":1,
    "decayFactor":None,#used in libreconmvm.so
    "reconlibOpen":1,
    "maxAdapOffset":0,
    "version":" "*120,
    #"lastActs":numpy.zeros((nacts,),numpy.uint16),
    }

#control["pxlCnt"][-3:]=npxls#not necessary, but means the RTC reads in all of the pixels... so that the display shows whole image
