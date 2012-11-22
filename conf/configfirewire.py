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
nacts=140#97#54#+256
ncam=1
ncamThreads=numpy.ones((ncam,),numpy.int32)*1
npxly=numpy.zeros((ncam,),numpy.int32)
npxly[:]=480
npxlx=npxly.copy()
npxlx[:]=640
nsuby=npxly.copy()
nsuby[:]=11
#nsuby[4:]=16
nsubx=nsuby.copy()
nsub=nsubx*nsuby
nsubaps=(nsuby*nsubx).sum()
#subapFlag=numpy.ones((nsubaps,),"i")
subapFlag=tel.Pupil(11,11/2.,1,11).subflag.astype("i").ravel()#numpy.ones((nsubaps,),"i")

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
nsubapsCum=numpy.zeros((ncam+1,),numpy.int32)
ncentsCum=numpy.zeros((ncam+1,),numpy.int32)
for i in range(ncam):
    nsubapsCum[i+1]=nsubapsCum[i]+nsubaps[i]
    ncentsCum[i+1]=ncentsCum[i]+subapFlag[nsubapsCum[i]:nsubapsCum[i+1]].sum()*2
for i in range(nsubaps):
    subapLocation[i]=((i//11)*40+20,(i//11+1)*40+20,1,(i%11)*40+100,(i%11+1)*40+100,1)
#guid for red camera is 2892819690320999
#guid for fire-i camera is 582164335728668360
try:
    a=prefix
except:
    prefix="main"

if prefix=="main":
    vmode=69#for the unibrain fire-i
    fr=36#30Hz
else:
    vmode=70#for the red camera
    fr=35#15Hz
cameraParams=numpy.array([0,0,1,vmode,-1,0,0,0,0,fr,2]).astype(numpy.int32)#guid,guid,print,vidmode(-1 or 69(8bit)/70(16bit) probably),color mode when vidmode==-1 (8=352, 16=357 or -1), width(640), height(480), offx (0), offy (0), framerate (36==30Hz, 35, -1==15Hz), [ISO speed (-1==don't set, 0=100, 1=200,2=400 (probably want -1 or 2), 3=800,4=1600,5=3200.)]
lval=cameraParams[:2].view(numpy.uint64)
if prefix=="main":
    lval[0]=582164335728668360#to select fire-i camera
else:
    lval[0]=2892819690320999#to select red camera

rmx=numpy.zeros((nacts,ncents),"f")

#devname="/dev/ttyUSB4\0"
mirrorParams=numpy.zeros((3,),"i")
#mirrorParams.view("c")[:len(devname)]=devname
mirrorParams[0]=1
mirrorParams[1]=1
mirrorParams[2]=-1
pxlCnt=numpy.zeros((nsubaps,),numpy.int32)

for k in range(ncam):
    # tot=0#reset for each camera
    for i in range(nsub[k]):
        #for j in range(nsubx[k]):
        indx=nsubapsCum[k]+i#*nsubx[k]+j
        n=(subapLocation[indx,1]-1)*npxlx[k]+subapLocation[indx,4]
        pxlCnt[indx]=n

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
    "cameraName":"libcamfirewire.so",#"libsl240Int32cam.so",#"camfile",
    "cameraParams":cameraParams,
    "mirrorName":"libmirrorBMMMulti.so",
    "mirrorParams":mirrorParams,
    "mirrorOpen":0,
    "frameno":0,
    "switchTime":numpy.zeros((1,),"d")[0],
    "adaptiveWinGain":0.5,
    "nsubapsTogether":1,
    "nsteps":0,
    "addActuators":0,
    "actuators":numpy.ones((nacts,),numpy.float32)*32768,#None,#(numpy.random.random((3,52))*1000).astype("H"),#None,#an array of actuator values.
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
#    "mirrorStep":0,
#    "mirrorSteps":numpy.zeros((nacts,),numpy.int32),
#    "mirrorUpdate":0,
#    "mirrorReset":0,
#    "mirrorGetPos":0,
#    "mirrorDoMidRange":0,
#    "mirrorMidRange":numpy.ones((nacts,),numpy.int32)*500,
    "actInit":None,
    "actMapping":None,
    "actOffset":None,
    "actScale":None,
    "actSource":None,
    }
