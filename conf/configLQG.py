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
import FITS
import tel
import numpy
nacts=52#97#54#+256
ncam=1
ncamThreads=numpy.ones((ncam,),numpy.int32)*8
npxly=numpy.zeros((ncam,),numpy.int32)
npxly[:]=128
npxlx=npxly.copy()
nsuby=npxly.copy()
nsuby[:]=7
#nsuby[4:]=16
nsubx=nsuby.copy()
nsub=nsubx*nsuby
nsubaps=(nsuby*nsubx).sum()
subapFlag=tel.Pupil(7*16,7*8,8,7).subflag.astype("i").ravel()#numpy.ones((nsubaps,),"i")

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
lqgPhaseSize=430#assume single layer turbulence...
lqgActSize=nacts



lqgAHwfs=numpy.zeros((lqgPhaseSize*2,lqgPhaseSize),numpy.float32)
lqgAtur=numpy.zeros((lqgPhaseSize,lqgPhaseSize),numpy.float32)
lqgHT=numpy.zeros((ncents,lqgPhaseSize*2),numpy.float32)
lqgHdm=numpy.zeros((lqgPhaseSize*2,lqgActSize),numpy.float32)
lqgInvN=numpy.zeros((lqgActSize,lqgPhaseSize),numpy.float32)
lqgInvNHT=numpy.dot(lqgInvN,lqgHT[:,:lqgPhaseSize].T).T.copy()

# now set up a default subap location array...
subx=(npxlx-16)/nsubx
suby=(npxly-16)/nsuby
for k in range(ncam):
    for i in range(nsuby[k]):
        for j in range(nsubx[k]):
            indx=nsubapsCum[k]+i*nsubx[k]+j
            if subapFlag[indx]:
                subapLocation[indx]=(8+i*suby[k],8+i*suby[k]+suby[k],1,8+j*subx[k],8+j*subx[k]+subx[k],1)

cameraParams=numpy.zeros((7,),numpy.int32)
cameraParams[0]=1#affin el size
cameraParams[1]=128*8#blocksize
cameraParams[2]=1000#timeout/ms
cameraParams[3]=0#port
cameraParams[5]=0#reorder
cameraParams[6]=0xffff#thread affinity
cameraParams[4]=1#thread priority
rmx=numpy.random.random((52,72)).astype("f")#FITS.Read("rmxRTC.fits")[1].transpose().astype("f")
gainRmxT=rmx.transpose().copy()

mirrorParams=numpy.zeros((5,),"i")
mirrorParams[0]=1000#timeout/ms
mirrorParams[1]=1#port
mirrorParams[2]=1#thread affinity el size
mirrorParams[4]=-1#thread affinity
mirrorParams[3]=1#thread prioirty


pxlCnt=numpy.zeros((nsubaps,),"i")
# set up the pxlCnt array - number of pixels to wait until each subap is ready.  Here assume identical for each camera.
for k in range(ncam):
    # tot=0#reset for each camera
    for i in range(nsuby[k]):
        for j in range(nsubx[k]):
            indx=nsubapsCum[k]+i*nsubx[k]+j
            n=(subapLocation[indx,1]-1)*npxlx[k]+subapLocation[indx,4]
            pxlCnt[indx]=n

control={
    "switchRequested":0,#this is the only item in a currently active buffer that can be changed...
    "pause":0,
    "go":1,
    "maxClipped":nacts,
    "refCentroids":None,
    "centroidMode":"CoG",
    "windowMode":"basic",
    "thresholdAlgo":1,
    "centroidWeight":None,
    "v0":numpy.zeros((nacts,),"f"),#v0 - mean mirror position
    "bleedGain":0.0,#0.05,#a gain for the piston bleed...
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
    "thresholdValue":200.,
    "powerFactor":1.,#raise pixel values to this power.
    "subapFlag":subapFlag,
    "lqgPhaseSize":lqgPhaseSize,
    "lqgActSize":lqgActSize,
    "lqgAHwfs":lqgAHwfs,
    "lqgAtur":lqgAtur,
    "lqgHT":lqgHT,
    "lqgHdm":lqgHdm,
    "lqgInvN":lqgInvN,
    "lqgInvNHT":lqgInvNHT,
    "fakeCCDImage":fakeCCDImage,
    "printTime":0,#whether to print time/Hz
    "threadAffinity":None,
    "threadPriority":numpy.ones((9,),numpy.int32)*10,
    "delay":50000,
    "clearErrors":0,
    "camerasOpen":0,
    "cameraName":"sl240Int32cam",#"camfile",
    "cameraParams":cameraParams,
    "mirrorName":"librtcmirror.so",
    "mirrorParams":mirrorParams,
    "mirrorOpen":0,
    "frameno":0,
    "switchTime":numpy.zeros((1,),"d")[0],
    "adaptiveWinGain":0.5,
    "nsubapsTogether":1,
    "nsteps":0,
    "addActuators":0,
    "actuators":None,
    "actSequence":None,
    "recordCents":0,
    "pxlWeight":None,
    "averageImg":0,
    "actuatorMask":None,
    "averageCent":0,
    "centCalData":None,
    "centCalBounds":None,
    "centCalSteps":None,
    "figureOpen":0,
    "figureName":"figureSL240",
    "figureParams":None,
    "reconName":"libreconLQG.so",
    }
#set the gain array
#control["gain"][:2]=0.5
# Note, gain is NOT used by the reconstructor - here, we assume that rows of the reconstructor have already been multiplied by the appropriate gain.  Similarly, the rows of E have been multiplied by 1-gain.  This multiplication is handled transparently by the GUI.

#control["pxlCnt"][-3:]=npxls#not necessary, but means the RTC reads in all of the pixels... so that the display shows whole image
