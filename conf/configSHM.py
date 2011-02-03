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
#This is a configuration file for CANARY.
#Aim to fill up the control dictionary with values to be used in the RTCS.

#This should be run with 3 different prefixes simultaneuosly.  i.e.:
#control.py configSHM.py
#control.py configSHM.py -s1
#control.py configSHM.py -s11

import FITS
import tel
import numpy

nacts=54#97#54#+256
ncam=1
ncamThreads=numpy.ones((ncam,),numpy.int32)*1
npxly=numpy.zeros((ncam,),numpy.int32)
npxly[:]=128
npxlx=npxly.copy()
nsuby=npxlx.copy()
nsuby[:]=7#for config purposes only... not sent to rtc
nsubx=nsuby.copy()
nsub=nsubx*nsuby#This is used by rtc.
nsubaps=nsub.sum()#(nsuby*nsubx).sum()
subapFlag=tel.Pupil(7,3.5,1,7).subflag.astype("i").ravel()
ncents=subapFlag.sum()*2
npxls=(npxly*npxlx).sum()

fakeCCDImage=None#(numpy.random.random((npxls,))*20).astype("i")

bgImage=None#FITS.Read("shimgb1stripped_bg.fits")[1].astype("f")#numpy.zeros((npxls,),"f")
darkNoise=None#FITS.Read("shimgb1stripped_dm.fits")[1].astype("f")
flatField=None#FITS.Read("shimgb1stripped_ff.fits")[1].astype("f")

subapLocation=numpy.zeros((nsubaps,6),"i")
nsubapsCum=numpy.zeros((ncam+1,),numpy.int32)
ncentsCum=numpy.zeros((ncam+1,),numpy.int32)
nsubapsCum[1]=nsub[0]
ncentsCum[1]=subapFlag.sum()*2

# now set up a default subap location array...
#this defines the location of the subapertures.
subx=16
suby=16
xoff=8
yoff=8
for i in range(nsuby[0]):
    for j in range(nsubx[0]):
        indx=i*nsubx[0]+j
        if subapFlag[indx]:
            subapLocation[indx]=(yoff+i*suby,yoff+i*suby+suby,1,xoff+j*subx,xoff+j*subx+subx,1)

pxlCnt=numpy.zeros((nsubaps,),"i")
# set up the pxlCnt array - number of pixels to wait until each subap is ready.  Here assume identical for each camera.
# tot=0#reset for each camera
for i in range(nsub[0]):
    pxlCnt[i]=(subapLocation[i,1]-1)*npxlx[0]+subapLocation[i,4]
pxlCnt[-3]=128*128

#The params are dependent on the interface library used.
cameraParams=numpy.fromstring("/home/ali/replay_1cam.fits\0\0",dtype="i")


rmx=numpy.zeros((nacts,ncents)).astype("f")#FITS.Read("rmxRTC.fits")[1].transpose().astype("f")

if len(prefix)>0:#one of the processing instances
    mirrorParams=numpy.zeros((2,),"i")
    mirrorParams[0]=2000#timeout/ms
    mirrorParams[1]=1#as float
    mirrorName="libmirrorSHM.so"
    mirrorOpen=1
    camerasOpen=1
    centOpen=1
    calOpen=1
    reconOpen=1
    reconParams=None
    reconName="libreconmvm.so"
    delay=10000*len(prefix)
else:#the async part that brings it all together
    mirrorName="test"
    mirrorOpen=0
    camerasOpen=0
    centOpen=0
    calOpen=0
    mirrorParams=None
    reconOpen=1
    reconName="libreconAsync.so"
    reconParams=numpy.zeros((6,),"i")
    reconParams[0]=2#nclients
    reconParams[1]=4340#port
    reconParams[2]=-1#affinity
    reconParams[3]=-4#priority
    reconParams[4]=3000#timeout in ms.
    reconParams[5]=0#overwrite flag
    delay=0

#Now describe the DM - this is for the GUI only, not the RTC.
#The format is: ndms, N for each DM, actuator numbers...
#Where ndms is the number of DMs, N is the number of linear actuators for each, and the actuator numbers are then an array of size NxN with entries -1 for unused actuators, or the actuator number that will set this actuator in the DMC array.

dmDescription=numpy.zeros((8*8+2*2+2+1,),numpy.int16)
dmDescription[0]=2#2 DMs
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
    "thresholdAlgo":1,
    "reconstructMode":"simple",#simple (matrix vector only), truth or open
    "centroidWeight":None,
    "v0":numpy.ones((nacts,),"f")*32768,#v0 from the tomograhpcic algorithm in openloop (see spec)
    "bleedGain":0.0,#0.05,#a gain for the piston bleed...
    "actMax":numpy.ones((nacts,),numpy.uint16)*65535,#4095,#max actuator value
    "actMin":numpy.zeros((nacts,),numpy.uint16),#4095,#max actuator value
    "nacts":nacts,
    "ncam":ncam,
    "nsub":nsub,
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
    "delay":delay,
    "clearErrors":0,
    "camerasOpen":camerasOpen,
    "cameraName":"libcamfile.so",#"camfile",
    "cameraParams":cameraParams,
    "mirrorName":mirrorName,
    "mirrorParams":mirrorParams,
    "mirrorOpen":mirrorOpen,
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
    "slopeOpen":centOpen,
    "slopeParams":None,
    "slopeName":"librtcslope.so",
    "actuatorMask":None,
    "dmDescription":dmDescription,
    "averageCent":0,
    "calibrateOpen":calOpen,
    "calibrateName":"librtccalibrate.so",
    "calibrateParams":None,
    "corrPSF":None,
    "centCalData":None,
    "centCalBounds":None,
    "centCalSteps":None,
    "figureOpen":0,
    "figureName":"figureSL240",
    "figureParams":numpy.array([1000,3,0xffff,2]).astype("i"),#timeout,port,affinity,priority
    "reconParams":reconParams,
    "reconName":reconName,
    "fluxThreshold":0,
    "printUnused":1,
    "useBrightest":0,
    "figureGain":1,
    "decayFactor":None,#used in libreconmvm.so
    "reconlibOpen":reconOpen,
    "maxAdapOffset":0,
    "version":" "*120,
    }

if len(prefix)==0:#
    asyncNames=numpy.zeros((5,),"c")
    asyncNames[0]='1'
    asyncNames[2:4]='11'
    control["asyncCombines"]=[-1,-1]
    control["asyncInitState"]=None
    control["asyncNames"]=asyncNames
    control["asyncOffsets"]=None
    control["asyncReset"]=0
    control["asyncScales"]=None
    control["asyncStarts"]=[0,0]
    control["asyncUpdates"]=[1,1]
    control["asyncTypes"]=[1,1]#both socket.
