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
import os
#Set up some basic parameters
nacts=241
ncam=1 # Number of WFSs
ncamThreads=numpy.ones((ncam,),numpy.int32)*8 #Number of threads to use
npxly=numpy.ones((ncam,),numpy.int32)*128#detector size
npxlx=npxly.copy()
nsuby=numpy.ones((ncam,),numpy.int32)*7#number of sub-apertures
nsubx=nsuby.copy()
nsub=nsuby*nsubx
nsubtot=nsub.sum()
subapFlag=numpy.array([tel.Pupil(7*16,7*8,8,7).subflag.astype("i")]*ncam).ravel()#Generate an map of used sub-apertures

ncents=subapFlag.sum()*2#number of slope measurements (x and y)
npxls=(npxly*npxlx).sum()

#Calibration data
bgImage=numpy.ones(128*128*ncam)*12#None#FITS.Read("shimgb1stripped_bg.fits")[1]
darkNoise=None#FITS.Read("shimgb1stripped_dm.fits")[1].astype("f")
flatField=None#FITS.Read("shimgb1stripped_ff.fits")[1].astype("f")

nsubaps=nsuby*nsubx#cumulative subap
nsubapsCum=numpy.zeros((ncam+1,),numpy.int32)
ncentsCum=numpy.zeros((ncam+1,),numpy.int32)
for i in range(ncam):
    nsubapsCum[i+1]=nsubapsCum[i]+nsubaps[i]
    ncentsCum[i+1]=ncentsCum[i]+subapFlag[nsubapsCum[i]:nsubapsCum[i+1]].sum()*2

# now set up a default subap location array... (define where the subaps are)
#There are 6 entries for each sub-aperture, which define:
#ystart, yend, ystep, xstart, xend, xstep.

#The steps are usually 1, unless you have interleaved pixels from
#multiple cameras.  The starts and ends define the pixels at which the
#sub-aperture starts and ends.

#For more complicated geometries, it is also possible to assign to
#sub-apertures on a per-pixel basis.  However, not all dynamic
#libraries support this.

subapLocation=numpy.zeros((nsubtot,6),"i")
subx=(npxlx-16)/nsubx
suby=(npxly-16)/nsuby
for k in range(ncam):
    for i in range(nsuby[k]):
        for j in range(nsubx[k]):
            indx=nsubapsCum[k]+i*nsubx[k]+j
            if subapFlag[indx]:
                subapLocation[indx]=(8+i*suby[k],8+i*suby[k]+suby[k],1,8+j*subx[k],8+j*subx[k]+subx[k],1)

# set up the pxlCnt array - number of pixels to wait until each subap is ready.  Here assume identical for each camera.
pxlCnt=numpy.zeros((nsubtot,),"i")
for k in range(ncam):
    # tot=0#reset for each camera
    for i in range(nsuby[k]):
        for j in range(nsubx[k]):
            indx=nsubapsCum[k]+i*nsubx[k]+j
            n=(subapLocation[indx,1]-1)*npxlx[k]+subapLocation[indx,4]
            pxlCnt[indx]=n
#pxlCnt[-3:]=npxls#not necessary, but means the RTC reads in all of the pixels... so that the display shows whole image

#create a reconstruction matrix
rmx=numpy.zeros((nacts,ncents)),"f")#FITS.Read("rmxRTC.fits")[1].transpose().astype("f")


#Parameters passed to the dynamic libraries upon loading.  These will vary depending on what library is in use.
fname="/opt/darc/test/img3x128x128.fits"
if not os.path.exists(fname):
    fname="/rtc/test/img3x128x128.fits"
while len(fname)%4!=0:#zero pad to it fits into 32bit int array
    fname+="\0"
cameraParams=numpy.fromstring(fname,dtype="i")
slopeParams=None
mirrorParams=numpy.zeros((5,),"i")
mirrorParams[0]=1#affin el size
mirrorParams[1]=1#prio
mirrorParams[2]=-1#thread affinity
mirrorParams[3:]=numpy.fromstring("BEL111\0\0",dtype="i")




#Now populate the control structure - this is what gets used.

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
    "actMax":numpy.ones((nacts,),numpy.float32)*65535,#4095,#max actuator value
    "actMin":numpy.zeros((nacts,),numpy.float32),#4095,#max actuator value
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
    "thresholdValue":1.0,
    "powerFactor":1.,#raise pixel values to this power.
    "subapFlag":subapFlag,
    "fakeCCDImage":None,#(numpy.random.random((npxls,))*20).astype("f"),
    "printTime":0,#whether to print time/Hz
    "rmx":rmx,#numpy.random.random((nacts,ncents)).astype("f"),
    "gain":numpy.ones((nacts,),"f"),
    "E":numpy.zeros((nacts,nacts),"f"),#E from the tomoalgo in openloop.
    "threadAffinity":None,
    "threadPriority":numpy.ones((ncamThreads.sum()+1,),numpy.int32)*10,
    "delay":100000,#will usually be zero (except if you want to set your own frame rate, e.g. if loading images from a file)
    "clearErrors":0,
    "camerasOpen":1,
    "cameraName":"libcamfile.so",#"libsl240Int32cam.so",#"camfile",
    "cameraParams":cameraParams,
    "mirrorName":"libmirrorAlpaoSdk.so",
    "mirrorParams":mirrorParams,
    "mirrorOpen":1,
    "frameno":0,
    "switchTime":numpy.zeros((1,),"d")[0],
    "adaptiveWinGain":0.5,
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
    "maxAdapOffset":10,
    "noPrePostThread":0,
    }

