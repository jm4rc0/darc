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

#import correlation
import FITS
import tel
import numpy
NNGSCAM=4#1, 2, 3, 4.  This is the number of physical cameras
NLGSCAM=1
NCAMERAS=NNGSCAM+NLGSCAM
nacts=56#97#54#+256
ncam=(int(NNGSCAM)+1)//2+NLGSCAM
camPerGrab=numpy.ones((ncam,),"i")
camPerGrab[:NNGSCAM//2]=2
<<<<<<< HEAD:conf/configCanaryBKalman.py
ncamThreads=numpy.ones((ncam,),numpy.int32)*4
=======
ncamThreads=numpy.ones((ncam,),numpy.int32)*8
threadPriority=numpy.ones((ncamThreads.sum()+1,),numpy.int32)*10
>>>>>>> 979771a8903c4f9967995be82138fb0459bedf72:conf/configCanaryBKalman.py
npxly=numpy.zeros((ncam,),numpy.int32)
npxly[:]=128
npxlx=npxly.copy()*camPerGrab
nsuby=npxlx.copy()
nsuby[:]=7#for config purposes only... not sent to rtc
nsuby[ncam-NLGSCAM:]=14#The LGS (4 WFS on 1 detector)
nsubx=nsuby.copy()*camPerGrab#for config purposes - not sent to rtc
nsub=nsubx*nsuby#This is used by rtc.
nsubaps=nsub.sum()#(nsuby*nsubx).sum()
individualSubapFlag=tel.Pupil(7,3.5,1,7).subflag.astype("i")
lgsSubapFlag=numpy.zeros((14,14),"i")
#lgsSubapFlag[:7,:7]=individualSubapFlag
#lgsSubapFlag[:7,7:]=individualSubapFlag
#lgsSubapFlag[7:,:7]=individualSubapFlag
#lgsSubapFlag[7:,7:]=individualSubapFlag
for i in range(4):#interleave sub-apertures from each LGS WFS image
    #wfs1,wfs2,wfs1,wfs2...wfs3,wfs4,wfs3,wfs4...next row:wfs1,wfs2,wfs1,wfs2...wfs3,wfs3,wfs3,wfs...next row: etc
    lgsSubapFlag[i//2::2,i%2::2]=individualSubapFlag
subapFlag=numpy.zeros((nsubaps,),"i")
for i in range(ncam-NLGSCAM):
    tmp=subapFlag[nsub[:i].sum():nsub[:i+1].sum()]
    tmp.shape=nsuby[i],nsubx[i]
    for j in range(camPerGrab[i]):
        tmp[:,j::camPerGrab[i]]=individualSubapFlag
#now for the lgs
for i in range(ncam-NLGSCAM,ncam):
    tmp=subapFlag[nsub[:i].sum():nsub[:i+1].sum()]
    tmp.shape=nsuby[i],nsubx[i]
    tmp[:]=lgsSubapFlag
#ncents=nsubaps*2
ncents=subapFlag.sum()*2
npxls=(npxly*npxlx).sum()

print "ncents: %d"%ncents
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
nsubapsCum=numpy.zeros((ncam+1,),numpy.int32)
ncentsCum=numpy.zeros((ncam+1,),numpy.int32)
for i in range(ncam):
    nsubapsCum[i+1]=nsubapsCum[i]+nsub[i]
    ncentsCum[i+1]=ncentsCum[i]+subapFlag[nsubapsCum[i]:nsubapsCum[i+1]].sum()*2

# now set up a default subap location array...
#this defines the location of the subapertures.
subx=(npxlx-16*camPerGrab)/nsubx
suby=(npxly-16)/nsuby
xoff=[8]*(ncam-NLGSCAM)+[4]*NLGSCAM
yoff=[8]*(ncam-NLGSCAM)+[4]*NLGSCAM
for k in range(ncam-NLGSCAM):
    nc=camPerGrab[k]
    for i in range(nsuby[k]):
        for j in range(nsubx[k]):
            indx=nsubapsCum[k]+i*nsubx[k]+j
            if subapFlag[indx]:
                subapLocation[indx]=(yoff[k]+i*suby[k],yoff[k]+i*suby[k]+suby[k],1,xoff[k]*nc+j/nc*subx[k]*nc+j%nc,xoff[k]*nc+j/nc*subx[k]*nc+subx[k]*nc+j%nc,nc)
#and now the LGS subapLocation

adapWinShiftCnt=numpy.ones((nsubaps,2),"i")
#now, set [:,0] to -1 for subaps in the top half of the LGS camera. (Split frame readout).

#These are row interleaved, and also position interleaved.
for k in range(ncam-NLGSCAM,ncam):
    for i in range(nsuby[k]):
        for j in range(nsubx[k]):
            indx=nsubapsCum[k]+i*nsubx[k]+j
            ii=i//2
            jj=j//2
            if subapFlag[indx]:
                if i%2==0:#a LHS WFS image
                    if j%2==0:#bottom left
                        subapLocation[indx]=(yoff[k]+ii*suby[k],yoff[k]+(ii+1)*suby[k],1,xoff[k]+jj*subx[k],xoff[k]+(jj+1)*subx[k],1)
                    else:#top left
                        subapLocation[indx]=(npxly[k]-(yoff[k]+(ii+1)*suby[k]),npxly[k]-(yoff[k]+ii*suby[k]),1,xoff[k]+jj*subx[k],xoff[k]+(jj+1)*subx[k],1)
                        adapWinShiftCnt[indx,0]=-1
                else:#a RHS WFS image
                    if j%2==0:#bottom right
                        subapLocation[indx]=(yoff[k]+ii*suby[k],yoff[k]+(ii+1)*suby[k],1,xoff[k]+npxlx[k]//2+jj*subx[k],xoff[k]+npxlx[k]//2+(jj+1)*subx[k],1)
                    else:#top right
                        subapLocation[indx]=(npxly[k]-(yoff[k]+(ii+1)*suby[k]),npxly[k]-(yoff[k]+ii*suby[k]),1,xoff[k]+npxlx[k]//2+jj*subx[k],xoff[k]+npxlx[k]//2+(jj+1)*subx[k],1)
                        adapWinShiftCnt[indx,0]=-1



pxlCnt=numpy.zeros((nsubaps,),"i")
# set up the pxlCnt array - number of pixels to wait until each subap is ready.  Here assume identical for each camera.
for k in range(ncam-NLGSCAM):
    # tot=0#reset for each camera
    for i in range(nsub[k]):
        indx=nsubapsCum[k]+i
        n=(subapLocation[indx,1]-1)*npxlx[k]+subapLocation[indx,4]
        pxlCnt[indx]=n

for k in range(ncam-NLGSCAM,ncam):
    for i in range(nsuby[k]):
        for j in range(nsubx[k]):
            indx=nsubapsCum[k]+i*nsubx[k]+j
            if j%2==0:#bottom half
                pxlCnt[indx]=((subapLocation[indx,1]-1)*npxlx[k]+subapLocation[indx,4])*2
            else:#top half
                pxlCnt[indx]=pxlCnt[indx-1]
#pxlCnt[-5]=128*256
#pxlCnt[-6]=128*256
#pxlCnt[nsubaps/2-5]=128*256
#pxlCnt[nsubaps/2-6]=128*256

#The params are dependent on the interface library used.
cameraParams=numpy.zeros((6*ncam+3,),numpy.int32)
cameraParams[0:6*ncam:6]=128*8#blocksize
cameraParams[1:6*ncam:6]=1000#timeout/ms
cameraParams[2:6*ncam:6]=range(ncam)#port
cameraParams[3:6*ncam:6]=0xffff#thread affinity
cameraParams[4:6*ncam:6]=2#thread priority
cameraParams[5:6*(ncam-NLGSCAM):6]=0#reorder
cameraParams[5+6*(ncam-NLGSCAM):6*ncam:6]=1#reorder
cameraParams[-3]=0#resync
cameraParams[-2]=1#wpu correction
cameraParams[-1]=2#number of frames to skip after short (truncated) frame.

centroiderParams=None
kalmanPhaseSize=1084#2164
rmx=numpy.zeros((nacts,ncents)).astype("f")#FITS.Read("rmxRTC.fits")[1].transpose().astype("f")

mirrorParams=numpy.zeros((4,),"i")
mirrorParams[0]=1000#timeout/ms
mirrorParams[1]=3#port
mirrorParams[2]=-1#thread affinity
mirrorParams[3]=3#thread prioirty

#Now describe the DM - this is for the GUI only, not the RTC.
#The format is: ndms, N for each DM, actuator numbers...
#Where ndms is the number of DMs, N is the number of linear actuators for each, and the actuator numbers are then an array of size NxN with entries -1 for unused actuators, or the actuator number that will set this actuator in the DMC array.

dmDescription=numpy.zeros((8*8+2*2+2*2+3+1,),numpy.int16)
dmDescription[0]=3#3 DMs
dmDescription[1]=2#1st DM has 2 linear actuators
dmDescription[2]=2#2nd DM has 2 linear actuators
dmDescription[3]=8#3rd DM has nacts linear actuators
tmp=dmDescription[4:8]
tmp[:]=-1
tmp[:2]=[52,53]#tip/tilt
tmp=dmDescription[8:12]
tmp[:]=-1
tmp[:2]=54,55
tmp=dmDescription[12:]
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
    #"rmx":rmx,#numpy.random.random((nacts,ncents)).astype("f"),
    "gain":numpy.ones((nacts,),"f"),
    "E":numpy.zeros((nacts,nacts),"f"),#E from the tomoalgo in openloop.
    "threadAffinity":None,
    "threadPriority":threadPriority,
    "delay":0,
    "clearErrors":0,
    "camerasOpen":0,
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
    "slopeParams":centroiderParams,
    "slopeName":"librtcslope.so",
    "actuatorMask":None,
    "dmDescription":dmDescription,
    "averageCent":0,
    "calibrateOpen":1,
    "calibrateName":"librtccalibrate.so",
    "calibrateParams":None,
    "corrPSF":None,
    "centCalData":None,
    "centCalBounds":None,
    "centCalSteps":None,
    "figureOpen":0,
    "figureName":"figureSL240",
    "figureParams":numpy.array([1000,3,0xffff,2]).astype("i"),#timeout,port,affinity,priority
    "reconName":"libreconKalman.so",
    "kalmanPhaseSize":kalmanPhaseSize,
    "kalmanHinfDM":numpy.random.random((kalmanPhaseSize*3,kalmanPhaseSize,)).astype("f"),
    "kalmanHinfT":numpy.random.random((ncents,kalmanPhaseSize*3)).astype("f"),
    "kalmanInvN":numpy.random.random((nacts,kalmanPhaseSize,)).astype("f"),
    "kalmanAtur":numpy.random.random((kalmanPhaseSize,kalmanPhaseSize,)).astype("f"),
    "fluxThreshold":0,
    "printUnused":1,
    "useBrightest":0,
    "figureGain":1,
    "decayFactor":None,#used in libreconmvm.so
    "reconlibOpen":1,
    "maxAdapOffset":0,
    "version":" "*120,
    "adapWinShiftCnt":adapWinShiftCnt,
    }

