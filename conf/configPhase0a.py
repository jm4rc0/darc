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
nacts=54#+256
ncam=1
ncamThreads=numpy.ones((ncam,),numpy.int32)*1
npxly=numpy.zeros((ncam,),numpy.int32)
npxly[:]=128
npxlx=npxly.copy()
nsuby=npxly.copy()
nsuby[:]=8
#nsuby[4:]=16
nsubx=nsuby.copy()
nsubaps=(nsuby*nsubx).sum()
subapFlag=numpy.ones((nsubaps,),"i")
indx=0
for i in range(ncam):
    tmp=subapFlag[indx:nsubx[i]*nsuby[i]+indx]
    tmp.shape=nsuby[i],nsubx[i]
    tmp[0,0]=0
    tmp[0,1]=0
    tmp[1,0]=0
    tmp[0,-1]=0
    tmp[0,-2]=0
    tmp[1,-1]=0
    tmp[-1,0]=0
    tmp[-1,1]=0
    tmp[-2,0]=0
    tmp[-1,-1]=0
    tmp[-1,-2]=0
    tmp[-2,-1]=0
    indx+=nsubx[i]*nsuby[i]

#ncents=nsubaps*2
ncents=subapFlag.sum()*2
npxls=(npxly*npxlx).sum()

fakeCCDImage=(numpy.random.random((npxls,))*20).astype("i")
bgImage=numpy.zeros((npxls,),"f")
flatField=numpy.zeros((npxls,),"f")
indx=0
nx=npxlx/nsubx
ny=npxly/nsuby
for i in range(ncam):
    bgtmp=bgImage[indx:npxlx[i]*npxly[i]+indx]
    bgtmp.shape=npxly[i],npxlx[i]
    bgtmp[:]=numpy.arange(npxlx[i])/(npxlx[i]-1.)*30
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
    indx+=npxlx[i]*npxly[i]

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
    "centroidMode":"CPU",#whether data is from cameras or from WPU.
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
    "v0":numpy.random.random((nacts,)).astype("f"),#v0 from the tomograhpcic algorithm in openloop (see spec)
    "gainE":None,#numpy.random.random((nacts,nacts)).astype("f"),#E from the tomo algo in openloop (see spec) with each row i multiplied by 1-gain[i]
    #"clip":1,#use actMax instead
    "bleedGain":0.0,#0.05,#a gain for the piston bleed...
    "midRangeValue":2048,#midrange actuator value used in actuator bleed
    "actMax":0,#4095,#max actuator value
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
    #"flatField":numpy.ones((ncam,npxly,npxlx),"f"),#an array same size as image.
    "flatField":flatField,#numpy.random.random((npxls,)).astype("f"),
    "thresholdValue":20.,
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
    "fakeCCDImage":None,
    "actuators":None,#an array of actuator values.
    "printTime":0,#whether to print time/Hz
    "rmx":numpy.random.random((nacts,ncents)).astype("f"),
    "gain":numpy.ones((nacts,),"f")*0.25,
    "E":numpy.random.random((nacts,nacts)).astype("f"),#E from the tomoalgo in openloop.
    "threadAffinity":None,
    "threadPriority":None,
    "delay":0,
    "clearErrors":0,
    "camerasOpen":0,
    "camerasFraming":0,
    "cameraParams":None,
    "cameraName":"andorpci",
    "mirrorName":"dmcSocket",
    "mirrorOpen":0,
    }
#set the gain array
control["gain"][:2]=0.5
# Note, gain is NOT used by the reconstructor - here, we assume that rows of the reconstructor have already been multiplied by the appropriate gain.  Similarly, the rows of E have been multiplied by 1-gain.  This multiplication is handled transparently by the GUI.

# now set up a default subap location array...
subx=npxlx/nsubx
suby=npxly/nsuby
for k in range(ncam):
    for i in range(nsuby[k]):
        for j in range(nsubx[k]):
            indx=nsubapsCum[k]+i*nsubx[k]+j
            subapLocation[indx]=(i*suby[k],i*suby[k]+suby[k],1,j*subx[k],j*subx[k]+subx[k],1)
# set up the pxlCnt array - number of pixels to wait until each subap is ready.  Here assume identical for each camera.
for k in range(ncam):
    # tot=0#reset for each camera
    for i in range(nsuby[k]):
        for j in range(nsubx[k]):
            indx=nsubapsCum[k]+i*nsubx[k]+j
            n=(subapLocation[indx,1]-1)*npxlx[k]+subapLocation[indx,4]
            control["pxlCnt"][indx]=n
