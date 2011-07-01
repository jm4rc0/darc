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
import correlation,FITS
import tel
import numpy
nacts=54#97#54#+256
ncam=1
ncamThreads=numpy.ones((ncam,),numpy.int32)*2
npxly=numpy.zeros((ncam,),numpy.int32)
npxly[:]=144#300#480
npxlx=npxly.copy()
npxlx[:]=144#400#640
nsuby=npxly.copy()
nsuby[:]=12#15
#nsuby[4:]=16
nsubx=nsuby.copy()
nsubaps=(nsuby*nsubx).sum()
#subapFlag=tel.Pupil(15*16,15*8,8,15).subflag.astype("i").ravel()#numpy.ones((nsubaps,),"i")
subapFlag=tel.Pupil(12*12,12*6,6,12).subflag.astype("i").ravel()#numpy.ones((nsubaps,),"i")

#ncents=nsubaps*2
ncents=subapFlag.sum()*2
npxls=(npxly*npxlx).sum()

fakeCCDImage=None#(numpy.random.random((npxls,))*20).astype("i")
#camimg=(numpy.random.random((10,npxls))*20).astype(numpy.int16)

bgImage=None#FITS.Read("shimgb1stripped_bg.fits")[1].astype("f")#numpy.zeros((npxls,),"f")
darkNoise=None#FITS.Read("shimgb1stripped_dm.fits")[1].astype("f")
flatField=None#FITS.Read("shimgb1stripped_ff.fits")[1].astype("f")

subapLocation=numpy.zeros((nsubaps,6),"i")
nsubaps=nsuby*nsubx#cumulative subap
nsubapsCum=numpy.zeros((ncam+1,),numpy.int32)
ncentsCum=numpy.zeros((ncam+1,),numpy.int32)
for i in range(ncam):
    nsubapsCum[i+1]=nsubapsCum[i]+nsubaps[i]
    ncentsCum[i+1]=ncentsCum[i]+subapFlag[nsubapsCum[i]:nsubapsCum[i+1]].sum()*2
#kalmanPhaseSize=nacts#assume single layer turbulence...
#HinfT=numpy.random.random((ncents,kalmanPhaseSize*3)).astype("f")-0.5
#kalmanHinfDM=numpy.random.random((kalmanPhaseSize*3,kalmanPhaseSize)).astype("f")-0.5
#Atur=numpy.random.random((kalmanPhaseSize,kalmanPhaseSize)).astype("f")-0.5
#invN=numpy.random.random((nacts,kalmanPhaseSize)).astype("f")

# now set up a default subap location array...
subx=(npxlx-0)/nsubx
suby=(npxly-0)/nsuby
for k in range(ncam):
    for i in range(nsuby[k]):
        for j in range(nsubx[k]):
            indx=nsubapsCum[k]+i*nsubx[k]+j
            if subapFlag[indx]:
                subapLocation[indx]=(0+i*suby[k],0+i*suby[k]+suby[k],1,0+j*subx[k],0+j*subx[k]+subx[k],1)

cameraName="/Canary/rtc/libjaicam.so"
cameraParams=numpy.zeros((18,),numpy.int32)
cameraParams[0]=2#bytes per pixel
cameraParams[1]=1500#timeout/ms
cameraParams[2]=1#thread affinity el size
cameraParams[3]=59#thread priority
cameraParams[4]=40#xoffset
cameraParams[5]=8#yoffset
cameraParams[6]=10000#exposure time in lines
cameraParams[7]=4#scan mode, 0, 1,2,3,4.
cameraParams[8]=200#1024#timerDelayRaw
cameraParams[9]=16465#4096#timerDurationRaw
cameraParams[10]=7#300#timerGranularity 1=1kHz, 3=500Hz, 7=250Hz.
cameraParams[11]=0#testmode(set to 1 to just test the rate at which images are obtained - they are then never passed to the rtc).
cameraParams[12]=4#max waiting frames queued before start throwing them away.
cameraParams[13]=1#internal trigger
cameraParams[14]=0xffff#thread affinity
cameraParams[15]=0#print pulnix node names... (parameters)
cameraParams[16]=-1#offset channel A (-1 to not set)
cameraParams[17]=-1#offset channel B
rmx=numpy.random.random((nacts,ncents)).astype("f")#FITS.Read("rmxRTC.fits")[1].transpose().astype("f")
gainRmxT=rmx.transpose().copy()

mirrorName="libmirrorPdAO32.so"
mirrorParams=numpy.zeros((3,),"i")
mirrorParams[0]=1#thread affinity element size
mirrorParams[1]=59#thread priority
mirrorParams[2]=-1#thread affinity.

actInit=numpy.ones((96,),numpy.uint16)*32768
#Tip/Tilt drive is differential. Max differential voltage is 13V.
#DAC outputs range from -10V (for 0x0) to +10V (for 0x10000); 
#however outputs must be limited to +/- 6.5V for the TT outputs
#because of above max differential limit.
#+6.5V ~ 0x8000 + (0x8000 * 6.5/10) = 0x8000 + 0x5334
#-6.5V ~ 0x8000 - (0x8000 * 6.5/10) = 0x8000 - 0x5334


#Mapping to the DAC card
actMapping=numpy.arange((nacts+2)).astype("i")
actMapping[52]=80#tip+ - was 64
actMapping[53]=81#tip-
actMapping[54]=82#tilt+
actMapping[55]=83#tilt-
#Mapping from the RTCS actuators
figureActSource=numpy.arange((nacts+2)).astype("i")
figureActSource[52]=52
figureActSource[53]=52
figureActSource[54]=53
figureActSource[55]=53
#Scaling of received actuators
figureActScale=numpy.ones((nacts+2,),numpy.float32)
figureActScale[52]=21300./32768
figureActScale[53]=-21300./32768
figureActScale[54]=21300./32768
figureActScale[55]=-21300./32768
#Offsetting of received actuators
figureActOffset=numpy.zeros((nacts+2,),numpy.float32)
figureActOffset[52]=32768-21300
figureActOffset[53]=32768+21300
figureActOffset[54]=32768-21300
figureActOffset[55]=32768+21300
#The equation to set DAC value actMapping[i] is
# act[actSource[i]] * actScale[i] + actOffset[i]


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
    #"DMgain":0.25,
    #"staticTerm":None,
    "maxClipped":nacts,
    "refCentroids":None,
    #"dmControlState":0,
    #"gainReconmxT":None,#numpy.random.random((ncents,nacts)).astype("f"),#reconstructor with each row i multiplied by gain[i].
    #"dmPause":0,
    #"reconMode":"closedLoop",
    #"applyPxlCalibration":0,
    "centroidMode":"CoG",#whether data is from cameras or from WPU.
    #"centroidAlgorithm":"wcog",
    "windowMode":"basic",
    #"windowMap":None,
    #"maxActuatorsSaturated":10,
    #"applyAntiWindup":0,
    #"tipTiltGain":0.5,
    #"laserStabilisationGain":0.1,
    "thresholdAlgo":1,
    #"acquireMode":"frame",#frame, pixel or subaps, depending on what we should wait for...
    "reconstructMode":"simple",#simple (matrix vector only), truth or open
    "centroidWeight":None,
    "v0":numpy.zeros((nacts,),"f"),#v0 from the tomograhpcic algorithm in openloop (see spec)
    #"gainE":None,#numpy.random.random((nacts,nacts)).astype("f"),#E from the tomo algo in openloop (see spec) with each row i multiplied by 1-gain[i]
    #"clip":1,#use actMax instead
    "bleedGain":0.0,#0.05,#a gain for the piston bleed...
    #"midRangeValue":2048,#midrange actuator value used in actuator bleed
    "actMax":numpy.ones((nacts,),numpy.uint16)*65535,#4095,#max actuator value
    "actMin":numpy.zeros((nacts,),numpy.uint16),#4095,#max actuator value
    #"gain":numpy.zeros((nacts,),numpy.float32),#the actual gains for each actuator...
    "nacts":nacts,
    "ncam":ncam,
    "nsub":nsuby*nsubx,
    #"nsubx":nsubx,
    "npxly":npxly,
    "npxlx":npxlx,
    "ncamThreads":ncamThreads,
    #"pxlCnt":numpy.zeros((ncam,nsuby,nsubx),"i"),#array of number of pixels to wait for next subap to have arrived.
    "pxlCnt":numpy.zeros((nsubaps,),"i"),
    #"subapLocation":numpy.zeros((ncam,nsuby,nsubx,4),"i"),#array of ncam,nsuby,nsubx,4, holding ystart,yend,xstart,xend for each subap.
    "subapLocation":subapLocation,
    #"bgImage":numpy.zeros((ncam,npxly,npxlx),"f"),#an array, same size as image
    "bgImage":bgImage,
    "darkNoise":darkNoise,
    "closeLoop":1,
    #"flatField":numpy.ones((ncam,npxly,npxlx),"f"),#an array same size as image.
    "flatField":flatField,#numpy.random.random((npxls,)).astype("f"),
    "thresholdValue":0.,
    "powerFactor":1.,#raise pixel values to this power.
    "subapFlag":subapFlag,
    #"randomCCDImage":0,#whether to have a fake CCD image...
    "usingDMC":0,#whether using DMC
    #"kalmanHinfT":HinfT,#Hinfinity, transposed...
    #"kalmanHinfDM":kalmanHinfDM,
    #"kalmanPhaseSize":kalmanPhaseSize,
    #"kalmanAtur":Atur,
    #"kalmanReset":0,
    #"kalmanInvN":invN,
    #"kalmanUsed":0,#whether to use Kalman...
    "fakeCCDImage":fakeCCDImage,
    "printTime":0,#whether to print time/Hz
    "rmx":rmx,#numpy.random.random((nacts,ncents)).astype("f"),
    "gain":numpy.ones((nacts,),"f"),
    "E":numpy.zeros((nacts,nacts),"f"),#E from the tomoalgo in openloop.
    "threadAffinity":None,
    "threadPriority":numpy.ones((ncamThreads.sum()+1,),numpy.int32)*60,
    "delay":0,
    "clearErrors":0,
    "camerasOpen":1,
    "camerasFraming":1,
    #"cameraParams":None,
    #"cameraName":"andorpci",
    "cameraName":cameraName,
    "cameraParams":cameraParams,
    "mirrorName":mirrorName,
    "mirrorParams":mirrorParams,
    "actInit":actInit,
    "actMapping":actMapping,
    "actSource":figureActSource,
    "actScale":figureActScale,
    "actOffset":figureActOffset,
    "mirrorOpen":0,
    "frameno":0,
    "switchTime":numpy.zeros((1,),"d")[0],
    "adaptiveWinGain":0.5,
    "nsubapsTogether":1,
    "nsteps":0,
    "addActuators":0,
    "actuators":None,#(numpy.random.random((3,52))*1000).astype("H"),#None,#an array of actuator values.
    "actSequence":None,#numpy.ones((3,),"i")*1000,
    "recordCents":0,
    "pxlWeight":None,
    "averageImg":0,
    "actuatorMask":None,
    "dmDescription":dmDescription,
    "averageCent":0,
    "centCalData":None,
    "centCalBounds":None,
    "centCalSteps":None,
    "figureOpen":1,
    "figureName":"libfigureSL240SCPassThrough.so",
    "figureParams":numpy.array([1000,0,1,50,0,0xffff]).astype("i"),#timeout,port,Naffinity,priority,debug,affin
    "reconName":"libreconmvm.so",
    "fluxThreshold":0,
    "printUnused":1,
    "useBrightest":0,
    "figureGain":1,
    "decayFactor":None,#used in libreconmvm.so
    "reconlibOpen":1,
    "figureDebug":0,
    }
#set the gain array
#control["gain"][:2]=0.5
# Note, gain is NOT used by the reconstructor - here, we assume that rows of the reconstructor have already been multiplied by the appropriate gain.  Similarly, the rows of E have been multiplied by 1-gain.  This multiplication is handled transparently by the GUI.

# set up the pxlCnt array - number of pixels to wait until each subap is ready.  Here assume identical for each camera.
for k in range(ncam):
    # tot=0#reset for each camera
    for i in range(nsuby[k]):
        for j in range(nsubx[k]):
            indx=nsubapsCum[k]+i*nsubx[k]+j
            n=(subapLocation[indx,1]-1)*npxlx[k]+subapLocation[indx,4]
            control["pxlCnt"][indx]=n
#control["pxlCnt"][-3:]=npxls#not necessary, but means the RTC reads in all of the pixels... so that the display shows whole image
