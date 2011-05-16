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
ncam=4#8 cameras... 4 7x7, 4 15x15
ncamThreads=numpy.ones((ncam,),numpy.int32)
ncamThreads[:2]=3
ncamThreads[2:]=1

npxly=numpy.zeros((ncam,),numpy.int32)
npxly[:]=128
npxlx=npxly.copy()
npxlx[:]=256
nsuby=npxly.copy()
nsuby[:2]=15
nsuby[2:]=7
#nsuby[4:]=16
nsubx=nsuby.copy()
nsubx[:2]=30
nsubx[2:]=14
nsub=nsubx*nsuby
nsubaps=(nsuby*nsubx).sum()
subapFlagNGS=tel.Pupil(7*16,7*8,8,7).subflag.astype("i")#.ravel()#numpy.ones((nsubaps,),"i")
subapFlagLGS=tel.Pupil(15*8,15*4,8,15).subflag.astype("i")#.ravel()#numpy.ones((nsubaps,),"i")
subapFlag=numpy.zeros((7*7*4+15*15*4,),"i")
tmp=subapFlag[:15*30]
tmp.shape=15,30
tmp[:,::2]=subapFlagLGS
tmp[:,1::2]=subapFlagLGS
tmp=subapFlag[15*30:2*15*30]
tmp.shape=15,30
tmp[:,::2]=subapFlagLGS
tmp[:,1::2]=subapFlagLGS
tmp=subapFlag[2*15*30:2*15*30+7*14]
tmp.shape=7,14
tmp[:,::2]=subapFlagNGS
tmp[:,1::2]=subapFlagNGS
tmp=subapFlag[2*15*30+7*14:]
tmp.shape=7,14
tmp[:,::2]=subapFlagNGS
tmp[:,1::2]=subapFlagNGS

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
#nsubaps=nsuby*nsubx#cumulative subap
nsubapsCum=numpy.zeros((ncam+1,),numpy.int32)
ncentsCum=numpy.zeros((ncam+1,),numpy.int32)
for i in range(ncam):
    nsubapsCum[i+1]=nsubapsCum[i]+nsuby[i]*nsubx[i]
    ncentsCum[i+1]=ncentsCum[i]+subapFlag[nsubapsCum[i]:nsubapsCum[i+1]].sum()*2

# now set up a default subap location array...
subx=[8,8,16,16]#pixels per subap(npxlx-16)/nsubx
suby=[8,8,16,16]#pixels per subap(npxly-16)/nsuby
xoff=[4,4,8,8]
yoff=[4,4,8,8]
for k in range(ncam):
    for i in range(nsuby[k]):
        for j in range(nsubx[k]):
            indx=nsubapsCum[k]+i*nsubx[k]+j
            if subapFlag[indx]:
                subapLocation[indx]=(yoff[k]+i*suby[k],yoff[k]+i*suby[k]+suby[k],1,xoff[k]*2+j/2*subx[k]*2+j%2,xoff[k]*2+j/2*subx[k]*2+subx[k]*2+j%2,2)

pxlCnt=numpy.zeros((nsubaps,),"i")
# set up the pxlCnt array - number of pixels to wait until each subap is ready.  Here assume identical for each camera.
for k in range(ncam):
    # tot=0#reset for each camera
    for i in range(nsuby[k]):
        for j in range(nsubx[k]):
            indx=nsubapsCum[k]+i*nsubx[k]+j
            n=(subapLocation[indx,1]-1)*npxlx[k]+subapLocation[indx,4]
            pxlCnt[indx]=n
cameraParams=numpy.zeros((25,),numpy.int32)
cameraParams[0]=1#affin el size
cameraParams[1::6]=128*8#blocksize
cameraParams[2::6]=1000#timeout/ms
cameraParams[3::6]=range(4)#port
cameraParams[6::6]=0xffff#thread affinity
cameraParams[4::6]=1#thread priority
cameraParams[5::6]=0#reorder

mirrorParams=numpy.zeros((5,),"i")
mirrorParams[0]=1000#timeout/ms
mirrorParams[1]=1#port
mirrorParams[2]=1#thread affinity el size
mirrorParams[4]=-1#thread affinity
mirrorParams[3]=1#thread prioirty

#Now describe the DM - this is for the GUI only, not the RTC.
#The format is: ndms, N for each DM, actuator numbers...
#Where ndms is the number of DMs, N is the number of linear actuators for each, and the actuator numbers are then an array of size NxN with entries -1 for unused actuators, or the actuator number that will set this actuator in the DMC array.
nacts=2#tip-tilt
dmDescription=numpy.zeros((2*2+32*32+8*8+3+1,),numpy.int16)
dmDescription[:]=-1
dmDescription[0]=3#1 DM
dmDescription[1]=2#1st DM has nacts linear actuators
dmDescription[2]=8#1st DM has nacts linear actuators
dmDescription[3]=32#1st DM has nacts linear actuators
dmDescription[4]=0
dmDescription[5]=1
#[6] and [7] not used (keep as -1)
tmp=dmDescription[8:8+8*8]
tmp.shape=8,8
dmflag=tel.Pupil(8,4,0).fn.ravel()
n=dmflag.sum()
numpy.put(tmp,dmflag.nonzero()[0],numpy.arange(n)+nacts)
nacts+=n
tmp=dmDescription[8*8+8:]
tmp.shape=32,32
dmflag=tel.Pupil(32,16,0).fn.ravel()
n=dmflag.sum()
numpy.put(tmp,dmflag.nonzero()[0],numpy.arange(n)+nacts)
nacts+=n

print "nacts: %d"%nacts
kalmanPhaseSize=nacts#assume single layer turbulence...
HinfT=numpy.random.random((ncents,kalmanPhaseSize*3)).astype("f")-0.5
kalmanHinfDM=numpy.random.random((kalmanPhaseSize*3,kalmanPhaseSize)).astype("f")-0.5
Atur=numpy.random.random((kalmanPhaseSize,kalmanPhaseSize)).astype("f")-0.5
invN=numpy.random.random((nacts,kalmanPhaseSize)).astype("f")

rmx=numpy.random.random((nacts,ncents)).astype("f")#FITS.Read("rmxRTC.fits")[1].transpose().astype("f")
gainRmxT=rmx.transpose().copy()


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
    "nacts":nacts,
    "ncam":ncam,
    "nsub":nsub,
#    "nsubx":nsubx,
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
    "threadPriority":numpy.ones((ncamThreads.sum()+1,),numpy.int32)*10,
    "delay":10000,
    "clearErrors":0,
    "camerasOpen":0,
    "camerasFraming":0,
    "cameraName":"sl240Int32cam",#"camfile",
    "cameraParams":cameraParams,
    "mirrorName":"dmcSL240mirror",
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
    "dmDescription":dmDescription,
    "averageCent":0,
    "centCalData":None,
    "centCalBounds":None,
    "centCalSteps":None,
    "figureOpen":0,
    "figureName":"figureSL240",
    "figureParams":None,
    }
#set the gain array
#control["gain"][:2]=0.5
# Note, gain is NOT used by the reconstructor - here, we assume that rows of the reconstructor have already been multiplied by the appropriate gain.  Similarly, the rows of E have been multiplied by 1-gain.  This multiplication is handled transparently by the GUI.

#control["pxlCnt"][-3:]=npxls#not necessary, but means the RTC reads in all of the pixels... so that the display shows whole image
