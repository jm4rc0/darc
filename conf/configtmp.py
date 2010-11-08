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
ncamThreads=numpy.ones((ncam,),numpy.int32)*1
npxly=numpy.zeros((ncam,),numpy.int32)
npxly[:]=128
npxlx=npxly.copy()
nsuby=npxly.copy()
nsuby[:]=7
#nsuby[4:]=16
nsubx=nsuby.copy()
nsubaps=(nsuby*nsubx).sum()
subapFlag=tel.Pupil(7*16,7*8,8,7).subflag.astype("i").ravel()#numpy.ones((nsubaps,),"i")

#ncents=nsubaps*2
ncents=subapFlag.sum()*2
npxls=(npxly*npxlx).sum()

fakeCCDImage=None#(numpy.random.random((npxls,))*20).astype("i")
#camimg=(numpy.random.random((10,npxls))*20).astype(numpy.int16)

bgImage=FITS.Read("shimgb1stripped_bg.fits")[1].astype("f")#numpy.zeros((npxls,),"f")
darkNoise=FITS.Read("shimgb1stripped_dm.fits")[1].astype("f")
flatField=FITS.Read("shimgb1stripped_ff.fits")[1].astype("f")
#indx=0
#nx=npxlx/nsubx
#ny=npxly/nsuby
#correlationPSF=numpy.zeros((npxls,),numpy.float32)

#for i in range(ncam):
if 0:
    bgtmp=bgImage[indx:npxlx[i]*npxly[i]+indx]
    bgtmp.shape=npxly[i],npxlx[i]
    bgtmp[:]=numpy.arange(npxlx[i])/(npxlx[i]-1.)*30
    darktmp=darkNoise[indx:npxlx[i]*npxly[i]+indx]
    darktmp.shape=npxly[i],npxlx[i]
    darktmp[:]=numpy.arange(npxlx[i])/(npxlx[i]-1.)*5
    fftmp=flatField[indx:npxlx[i]*npxly[i]+indx]
    fftmp.shape=npxly[i],npxlx[i]
    fftmp[:]=(numpy.cos(numpy.arange(npxlx[i])/(npxlx[i]-1.)*20)+5)*2
    tmp=fakeCCDImage[indx:npxlx[i]*npxly[i]+indx]
    tmp.shape=npxly[i],npxlx[i]
    tmp[ny[i]/2-1:npxly[i]:ny[i],nx[i]/2-1:npxlx[i]:nx[i]]=100
    tmp[ny[i]/2:npxly[i]:ny[i],nx[i]/2-1:npxlx[i]:nx[i]]=100
    tmp[ny[i]/2-1:npxly[i]:ny[i],nx[i]/2:npxlx[i]:nx[i]]=100
    tmp[ny[i]/2:npxly[i]:ny[i],nx[i]/2:npxlx[i]:nx[i]]=100
    tmp*=fftmp
    tmp+=bgtmp
    tmp=camimg[:,indx:npxlx[i]*npxly[i]+indx]
    tmp.shape=tmp.shape[0],npxly[i],npxlx[i]
    tmp[:,ny[i]/2-1:npxly[i]:ny[i],nx[i]/2-1:npxlx[i]:nx[i]]=100
    tmp[:,ny[i]/2:npxly[i]:ny[i],nx[i]/2-1:npxlx[i]:nx[i]]=100
    tmp[:,ny[i]/2-1:npxly[i]:ny[i],nx[i]/2:npxlx[i]:nx[i]]=100
    tmp[:,ny[i]/2:npxly[i]:ny[i],nx[i]/2:npxlx[i]:nx[i]]=100
    tmp*=fftmp
    tmp+=bgtmp

    #corrImg=correlationPSF[indx:indx+npxlx[i]*npxly[i]]
    #corrImg.shape=npxly[i],npxlx[i]
    #corrImg[ny[i]/2-1:npxly[i]:ny[i],nx[i]/2-1:npxlx[i]:nx[i]]=1
    #corrImg[ny[i]/2:npxly[i]:ny[i],nx[i]/2-1:npxlx[i]:nx[i]]=1
    #corrImg[ny[i]/2-1:npxly[i]:ny[i],nx[i]/2:npxlx[i]:nx[i]]=1
    #corrImg[ny[i]/2:npxly[i]:ny[i],nx[i]/2:npxlx[i]:nx[i]]=1
    
    #indx+=npxlx[i]*npxly[i]

#FITS.Write(camimg,"camImage.fits")#file used when reading from file,
subapLocation=numpy.zeros((nsubaps,6),"i")
nsubaps=nsuby*nsubx#cumulative subap
nsubapsCum=numpy.zeros((ncam+1,),numpy.int32)
ncentsCum=numpy.zeros((ncam+1,),numpy.int32)
for i in range(ncam):
    nsubapsCum[i+1]=nsubapsCum[i]+nsubaps[i]
    ncentsCum[i+1]=ncentsCum[i]+subapFlag[nsubapsCum[i]:nsubapsCum[i+1]].sum()*2
kalmanPhaseSize=nacts#assume single layer turbulence...
HinfT=numpy.random.random((ncents,kalmanPhaseSize*3)).astype("f")-0.5
kalmanHinfDM=numpy.random.random((kalmanPhaseSize*3,kalmanPhaseSize)).astype("f")-0.5
Atur=numpy.random.random((kalmanPhaseSize,kalmanPhaseSize)).astype("f")-0.5
invN=numpy.random.random((nacts,kalmanPhaseSize)).astype("f")

# now set up a default subap location array...
subx=(npxlx-16)/nsubx
suby=(npxly-16)/nsuby
for k in range(ncam):
    for i in range(nsuby[k]):
        for j in range(nsubx[k]):
            indx=nsubapsCum[k]+i*nsubx[k]+j
            if subapFlag[indx]:
                subapLocation[indx]=(8+i*suby[k],8+i*suby[k]+suby[k],1,8+j*subx[k],8+j*subx[k]+subx[k],1)

cameraParams=numpy.zeros((2,),numpy.int32)
cameraParams[0]=5554#port
cameraParams[1:]=numpy.fromstring("m912",dtype=numpy.int32)
centroiderParams=numpy.zeros((5,),numpy.int32)
centroiderParams[0]=18#blocksize
centroiderParams[1]=1000#timeout/ms
centroiderParams[2]=2#port
centroiderParams[3]=-1#thread affinity
centroiderParams[4]=1#thread priority
rmx=FITS.Read("rmxRTC.fits")[1].transpose().astype("f")
gainRmxT=rmx.transpose().copy()

mirrorParams=numpy.zeros((2,),"i")
mirrorParams[0]=5553#timeout/ms
mirrorParams[1:]=numpy.fromstring("m912",dtype=numpy.int32)

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
    "gainReconmxT":None,#numpy.random.random((ncents,nacts)).astype("f"),#reconstructor with each row i multiplied by gain[i].
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
    "thresholdAlgorithm":1,
    #"acquireMode":"frame",#frame, pixel or subaps, depending on what we should wait for...
    "reconstructMode":"simple",#simple (matrix vector only), truth or open
    "centroidWeighting":None,
    "v0":numpy.zeros((nacts,),"f"),#v0 from the tomograhpcic algorithm in openloop (see spec)
    "gainE":None,#numpy.random.random((nacts,nacts)).astype("f"),#E from the tomo algo in openloop (see spec) with each row i multiplied by 1-gain[i]
    #"clip":1,#use actMax instead
    "bleedGain":0.0,#0.05,#a gain for the piston bleed...
    "midRangeValue":2048,#midrange actuator value used in actuator bleed
    "actMax":65535,#4095,#max actuator value
    "actMin":0,#4095,#max actuator value
    #"gain":numpy.zeros((nacts,),numpy.float32),#the actual gains for each actuator...
    "nacts":nacts,
    "ncam":ncam,
    "nsuby":nsuby,
    "nsubx":nsubx,
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
    "thresholdValue":200.,
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
    "threadPriority":None,
    "delay":0,
    "clearErrors":0,
    "camerasOpen":0,
    "camerasFraming":0,
    #"cameraParams":None,
    #"cameraName":"andorpci",
    "cameraName":"camsocket",#"camfile",
    "cameraParams":cameraParams,
    "mirrorName":"dmcSocket",
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
