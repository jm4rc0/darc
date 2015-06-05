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

#This is for andor PCI configuration.  Requires libandorcam.so (which is part of darc)

import FITS
import tel
import numpy
nacts=52#97#54#+256
ncam=2
ncamThreads=numpy.ones((ncam,),numpy.int32)*1
npxly=numpy.zeros((ncam,),numpy.int32)
npxly[:]=128
npxlx=npxly.copy()
nsub=npxly.copy()
nsub[:]=49
#nsuby[4:]=16
#nsubx=nsuby.copy()
nsubaps=nsub.sum()#(nsuby*nsubx).sum()
isubapFlag=tel.Pupil(7*16,7*8,8,7).subflag.astype("i").ravel()#numpy.ones((nsubaps,),"i")
subapFlag=numpy.zeros(2*7*7,"i")
subapFlag[:7*7]=isubapFlag
subapFlag[7*7:]=isubapFlag

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

cameraParams=numpy.zeros((2,),numpy.int32)#xpos and ypos

rmx=numpy.random.random((nacts,ncents)).astype("f")#FITS.Read("rmxRTC.fits")[1].transpose().astype("f")

mirrorParams=numpy.zeros((5,),"i")
mirrorParams[0]=1000#timeout/ms
mirrorParams[1]=1#port
mirrorParams[2]=1#thread affinity el size
mirrorParams[3]=1#thread prioirty
mirrorParams[4]=-1#thread affinity

pxlCnt=numpy.zeros((nsubaps,),"i")
# set up the pxlCnt array - number of pixels to wait until each subap is ready.  Here assume identical for each camera.
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
    "maxClipped":nacts,
    "refCentroids":None,
    "centroidMode":"CoG",#whether data is from cameras or from WPU.
    "windowMode":"basic",
    "thresholdAlgo":1,
    "reconstructMode":"simple",#simple (matrix vector only), truth or open
    "centroidWeight":None,
    "v0":numpy.zeros((nacts,),"f"),#v0 from the tomograhpcic algorithm in openloop (see spec)
    "bleedGain":0.0,#0.05,#a gain for the piston bleed...
    "actMax":numpy.ones((nacts,),numpy.uint16)*65535,#4095,#max actuator value
    "actMin":numpy.zeros((nacts,),numpy.uint16),#4095,#max actuator value
    "nacts":nacts,
    "ncam":ncam,
    "nsub":nsub,
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
    "threadPriority":numpy.ones((1+ncamThreads.sum(),),numpy.int32)*10,
    "delay":0,
    "clearErrors":0,
    "camerasOpen":1,
    "cameraName":"libandorcam.so",#"camfile",
    "cameraParams":cameraParams,
    "mirrorName":"libmirrorSL240.so",
    "mirrorParams":mirrorParams,
    "mirrorOpen":0,
    "frameno":0,
    "switchTime":numpy.zeros((1,),"d")[0],
    "adaptiveWinGain":0.5,
    "corrThreshType":0,
    "corrThresh":0.,
    "corrFFTPattern":None,
    "nsubapsTogether":1,
    "nsteps":0,
    "addActuators":0,
    "actuators":None,
    "actSequence":None,
    "recordCents":0,
    "pxlWeight":None,
    "averageImg":0,
    "slopeOpen":1,
    "slopeParams":None,
    "slopeName":"librtcslope.so",
    "actuatorMask":None,
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
#    "andorChangeClamp":0,
#    "andorClamp":0,
#    "andorCoolerOn":1,
#    "andorEmAdvanced":1,
#    "andorEmGain":0,
#    "andorEmMode":1,
    "andorExpTime":0.01,
#    "andorFanMode":0,
#    "andorFastExtTrig":1,
#    "andorHSSpeed":0,
#    "andorOutputAmp":1,
#    "andorOutputType":1,
#    "andorPreAmp":0,
#    "andorTemperature":-70,
    "andorTrigMode":0,#0 for internal, 1 for external.  Note, if internal, a andorExpTime should also be specified >0, otherwise it seems to not work properly.
#    "andorVSSpeed":1,
#    "andorVSamp":1,
    }

