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

#A configuration file for use with the uEye USB camera.
import FITS
import tel
import numpy
nacts=4#97#54#+256
ncam=1
ncamThreads=numpy.ones((ncam,),numpy.int32)*1
npxly=numpy.zeros((ncam,),numpy.int32)
npxly[:]=480
npxlx=npxly.copy()
npxlx[:]=640
nsuby=npxly.copy()
nsuby[:]=16
#nsuby[4:]=16
nsubx=nsuby.copy()
nsub=nsubx*nsuby
nsubaps=(nsuby*nsubx).sum()
subapFlag=numpy.ones((nsubaps,),"i")#tel.Pupil(7*16,7*8,8,7).subflag.astype("i").ravel()#numpy.ones((nsubaps,),"i")

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
nsubaps=nsuby*nsubx#cumulative subap
for i in range(nsuby[0]):
    for j in range(nsubx[0]):
        subapLocation[i*nsubx[0]+j]=(i*30,(i+1)*30,1,j*30+80,(j+1)*30+80,1)

cameraParams=numpy.array([0,0,640,480,30]).astype(numpy.int32)#xpos,ypos,width,height,frame rate
rmx=numpy.zeros((nacts,ncents),"f")

#devname="/dev/ttyUSB4\0"
mirrorParams=numpy.zeros((5,),"i")
mirrorParams[0]=1#affin el size
mirrorParams[1]=1#prio
mirrorParams[2]=-1#thread affinity
mirrorParams[3:]=numpy.fromstring("BEL111\0\0",dtype="i")#serial number of the mirror.  Need to export ACECFG to point to the directory holding the ALPAO config files.

pxlCnt=numpy.zeros((nsubaps,),numpy.int32)

for i in range(nsub[0]):
    n=(subapLocation[i,1]-1)*npxlx[0]+subapLocation[i,4]
    pxlCnt[i]=n

creepAbstats=numpy.random.random(241).astype("f")
creepMean=numpy.random.random(241).astype("f")
creepTime=time.time()
creepMode=1


control={
    "switchRequested":0,#this is the only item in a currently active buffer that can be changed...
    "pause":0,
    "go":1,
    #"DMgain":0.25,
    #"staticTerm":None,
    "maxClipped":nacts,
    "refCentroids":None,
     "centroidMode":"CoG",#whether data is from cameras or from WPU.
     "windowMode":"basic",
     "thresholdAlgo":0,
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
    "pxlCnt":pxlCnt,
    "subapLocation":subapLocation,
    "bgImage":bgImage,
    "darkNoise":darkNoise,
    "closeLoop":1,
    "flatField":flatField,#numpy.random.random((npxls,)).astype("f"),
    "thresholdValue":0.,
    "powerFactor":1.,#raise pixel values to this power.
    "subapFlag":subapFlag,
    "fakeCCDImage":fakeCCDImage,
    "printTime":0,#whether to print time/Hz
    "rmx":rmx,#numpy.random.random((nacts,ncents)).astype("f"),
    "gain":numpy.ones((nacts,),"f"),
    "E":numpy.zeros((nacts,nacts),"f"),#E from the tomoalgo in openloop.
    "threadAffinity":None,
    "threadPriority":numpy.ones((ncamThreads.sum()+1,),numpy.int32)*10,
    "delay":0,
    "clearErrors":0,
    "camerasOpen":1,
    "cameraName":"libcamuEyeUSB.so",#"libsl240Int32cam.so",#"camfile",
    "cameraParams":cameraParams,
    "mirrorName":"libmirrorAlpaoSDK.so",
    "mirrorParams":mirrorParams,
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
    "averageCent":0,
    "centCalData":None,
    "centCalBounds":None,
    "centCalSteps":None,
    "figureOpen":0,
    "figureName":"libfigureSL240.so",
    "figureParams":None,
    "reconName":"libreconmvm.so",
    "fluxThreshold":0,
    "printUnused":1,
    "useBrightest":0,
    "figureGain":1,
    "decayFactor":None,#used in libreconmvm.so
    "reconlibOpen":1,
    "maxAdapOffset":0,
    "mirrorStep":0,
    "mirrorSteps":numpy.zeros((nacts,),numpy.int32),
    "mirrorUpdate":0,
    "mirrorReset":0,
    "mirrorGetPos":0,
    "mirrorDoMidRange":0,
    "mirrorMidRange":numpy.ones((nacts,),numpy.int32)*500,
    "uEyeFrameRate":30.,#in Hz
    "uEyeExpTime":0.,#in ms, or 0 for 1/framerate.
    "uEyeNFrames":1,
    "uEyeGrabMode":0,
    "uEyeGain":0,
    "uEyeBoostGain":0,
    "uEyeActualExp":0.,#numpy.zeros((1,),numpy.float64),
    "uEyeBlackLevel":0,
    "uEyePixelClock":30,
    "creepMean":creepMean,
    "creepAbstats":creepAbstats,
    "creepMode":creepMode,
    "creepTime":numpy.array([creepTime])[0],
    }
