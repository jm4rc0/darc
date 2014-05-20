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
nacts=52#97#54#+256
ncam=1
ncamThreads=numpy.ones((ncam,),numpy.int32)*8
npxly=numpy.zeros((ncam,),numpy.int32)
npxly[:]=128
npxlx=npxly.copy()
nsub=npxly.copy()
nsub[:]=49
#nsuby[4:]=16
#nsubx=nsuby.copy()
nsubaps=nsub.sum()#(nsuby*nsubx).sum()
subapFlag=tel.Pupil(7*16,7*8,8,7).subflag.astype("i").ravel()#numpy.ones((nsubaps,),"i")

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


#FITS.Write(camimg,"camImage.fits")#file used when reading from file,
subapLocation=numpy.zeros((nsubaps,6),"i")
#nsubaps=nsuby*nsubx#cumulative subap
nsubapsCum=numpy.zeros((ncam+1,),numpy.int32)
ncentsCum=numpy.zeros((ncam+1,),numpy.int32)
for i in range(ncam):
    nsubapsCum[i+1]=nsubapsCum[i]+nsub[i]
    ncentsCum[i+1]=ncentsCum[i]+subapFlag[nsubapsCum[i]:nsubapsCum[i+1]].sum()*2
kalmanPhaseSize=nacts#assume single layer turbulence...
HinfT=numpy.random.random((ncents,kalmanPhaseSize*3)).astype("f")-0.5
kalmanHinfDM=numpy.random.random((kalmanPhaseSize*3,kalmanPhaseSize)).astype("f")-0.5
Atur=numpy.random.random((kalmanPhaseSize,kalmanPhaseSize)).astype("f")-0.5
invN=numpy.random.random((nacts,kalmanPhaseSize)).astype("f")

# now set up a default subap location array...
subx=(npxlx-16)/numpy.sqrt(nsub)
suby=(npxly-16)/numpy.sqrt(nsub)
for k in range(ncam):
    for i in range(nsub[k]):
        #for j in range(nsubx[k]):
        indx=nsubapsCum[k]+i#*nsubx[k]+j
        ny=numpy.sqrt(nsub[k])
        if subapFlag[indx]:
            subapLocation[indx]=(8+(i//ny)*suby[k],8+(i//ny)*suby[k]+suby[k],1,8+(i%ny)*subx[k],8+(i%ny)*subx[k]+subx[k],1)

cameraParams=numpy.zeros((7+7,),numpy.int32)
cameraParams[0]=1#threadAffinElSize
cameraParams[1]=128*8#blocksize
cameraParams[2]=1000#timeout/ms
cameraParams[3]=2#port
cameraParams[4]=1#thread priority
cameraParams[5]=0#reorder
cameraParams[6]=0#testLastPixel
cameraParams[7]=0xffff#thread affinity
cameraParams[8]=0#resync
cameraParams[9]=1#wpu corrction
cameraParams[10]=2#nframes to skip
cameraParams[11]=0#pxlRowStartSkipThreshold
cameraParams[12]=0#pxlRowEndInsertThreshold
cameraParams[13]=0#record timestamp.
rmx=numpy.random.random((nacts,ncents)).astype("f")#FITS.Read("rmxRTC.fits")[1].transpose().astype("f")
gainRmxT=rmx.transpose().copy()

mirrorParams=numpy.zeros((5,),"i")
mirrorParams[0]=1000#timeout/ms
mirrorParams[1]=1#port
mirrorParams[2]=1#thread affinity el size
mirrorParams[3]=1#thread prioirty
mirrorParams[4]=-1#thread affinity

#Now describe the DM - this is for the GUI only, not the RTC.
#The format is: ndms, N for each DM, actuator numbers...
#Where ndms is the number of DMs, N is the number of linear actuators for each, and the actuator numbers are then an array of size NxN with entries -1 for unused actuators, or the actuator number that will set this actuator in the DMC array.
dmDescription=numpy.zeros((8*8+1+1,),numpy.int16)
dmDescription[0]=1#1 DM
dmDescription[1]=8#1st DM has nacts linear actuators
tmp=dmDescription[2:]
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
    "nsub":nsub,
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
    "kalmanHinfT":HinfT,#Hinfinity, transposed...
    "kalmanHinfDM":kalmanHinfDM,
    "kalmanPhaseSize":kalmanPhaseSize,
    "kalmanAtur":Atur,
    "kalmanReset":0,
    "kalmanInvN":invN,
    "kalmanUsed":0,#whether to use Kalman...
    "fakeCCDImage":fakeCCDImage,
    "printTime":0,#whether to print time/Hz
    "rmx":rmx,#numpy.random.random((nacts,ncents)).astype("f"),
    "gain":numpy.ones((nacts,),"f"),
    "E":numpy.zeros((nacts,nacts),"f"),#E from the tomoalgo in openloop.
    "threadAffinity":None,
    "threadPriority":numpy.ones((9,),numpy.int32)*10,
    "delay":0,
    "clearErrors":0,
    "camerasOpen":1,
    "cameraName":"libsl240Int32cam.so",#"camfile",
    "cameraParams":cameraParams,
    "mirrorName":"libmirrorSL240.so",
    "mirrorParams":mirrorParams,
    "mirrorOpen":0,
    "frameno":0,
    "switchTime":numpy.zeros((1,),"d")[0],
    "adaptiveWinGain":0.5,
    "corrThreshType":0,
    "corrThresh":0.,
    "corrFFTPattern":None,#correlation.transformPSF(correlationPSF,ncam,npxlx,npxly,nsubx,nsuby,subapLocation),
#    "correlationPSF":correlationPSF,
    "nsubapsTogether":1,
    "nsteps":0,
    "addActuators":0,
    "actuators":None,#(numpy.random.random((3,52))*1000).astype("H"),#None,#an array of actuator values.
    "actSequence":None,#numpy.ones((3,),"i")*1000,
    "recordCents":0,
    "pxlWeight":None,
    "averageImg":0,
    "slopeOpen":1,
    "slopeParams":None,
    "slopeName":"librtcslope.so",
    "actuatorMask":None,
    "dmDescription":dmDescription,
    "averageCent":0,
    "centCalData":None,
    "centCalBounds":None,
    "centCalSteps":None,
    "figureOpen":0,
    "figureName":"libfigureSL240.so",
    "figureParams":numpy.array([1000,0,1,1,0,0xffff]).astype("i"),#timeout,port,affinity,priority
    "reconName":"libreconmvm.so",
    "fluxThreshold":0,
    "printUnused":1,
    "useBrightest":0,
    "figureGain":1,
    "decayFactor":None,#used in libreconmvm.so
    "reconlibOpen":1,
    "maxAdapOffset":0,
    "version":" "*120,
    }
#set the gain array
#control["gain"][:2]=0.5
# Note, gain is NOT used by the reconstructor - here, we assume that rows of the reconstructor have already been multiplied by the appropriate gain.  Similarly, the rows of E have been multiplied by 1-gain.  This multiplication is handled transparently by the GUI.

# set up the pxlCnt array - number of pixels to wait until each subap is ready.  Here assume identical for each camera.
for k in range(ncam):
    # tot=0#reset for each camera
    for i in range(nsub[k]):
        #for j in range(nsubx[k]):
        indx=nsubapsCum[k]+i#*nsubx[k]+j
        n=(subapLocation[indx,1]-1)*npxlx[k]+subapLocation[indx,4]
        control["pxlCnt"][indx]=n
#control["pxlCnt"][-3:]=npxls#not necessary, but means the RTC reads in all of the pixels... so that the display shows whole image
