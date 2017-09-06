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
#This is a configuration file for the Xeon Phi Knights Landing testing.
#Aim to fill up the control dictionary with values to be used in the RTCS.

#Best for phi: nsub=74, 54 threads, prio=39,99,99,99,99...

#import correlation
import sys
import string
import FITS
import tel
import numpy
numpy.set_printoptions(linewidth=500)
NCAMERAS=1   #1, 2, 3, 4.  This is the number of physical cameras
ncam=NCAMERAS#(int(NCAMERAS)+1)/2

nsub=74 #should be 74 for E-ELT...

nthreads = 56 #total number of threads, maximum 62 for now
nn=nthreads//ncam #threads per camera
ncamThreads=numpy.ones((ncam,),numpy.int32)*nn
nthreads = ncamThreads.sum() # new total number of threads, multiple of ncam

noPrePostThread=0
threadPriority=97*numpy.ones((nthreads+1,),numpy.uint32)
#for i in range(nthreads): # use to set individual priorities
#	threadPriority[i+1] = (100-nthreads)+i
#threadPriority=numpy.arange(nthreads+1)+40
threadPriority[0]=98   # first thread 39 priority
print threadPriority


threadAffElSize = 4
threadAffinity=numpy.zeros(((1+nthreads)*threadAffElSize),dtype=numpy.uint32)

mcpu = 64
for i in range(0,nthreads):
    if (i<74):
        j=i+mcpu-nthreads
        threadAffinity[(i)*threadAffElSize+(j)/32]=1<<(((j)%32))
    else:
        print "too many threads.."
        exit(0)

threadAffinity[threadAffElSize:] = threadAffinity[:-threadAffElSize]
threadAffinity[:threadAffElSize] = 0
threadAffinity[(mcpu-nthreads-1-2)/32] = 1<<(((mcpu-nthreads-1-2)%32)) # control thread affinity

camAffin = numpy.zeros(2,dtype=numpy.uint32)
camAffin[(mcpu-nthreads-1)/32] = 1<<(((mcpu-nthreads-1)%32)) #1 thread
camAffin[1-(mcpu-nthreads-1)/32] = 0
print "cam affinity", [bin(i) for i in camAffin]

# print out threadAfinity in binary form for verification
for i in range(nthreads+1):
   print i,[bin(j) for j in threadAffinity[i*threadAffElSize:(i+1)*threadAffElSize]]

npxl=6 # for square subaps
xnpxl=12 # width for rectangular subaps
ynpxl=3 #height for rectangular subaps
nact=(nsub+1)
#print "nact = ", nact
nacts=tel.Pupil(nact,nact/2.,2).fn.astype("i").sum()
print "nacts1",nacts
nacts=16*((nacts+15)/16)
#nacts=10000
print "nacts2",nacts
camPerGrab=numpy.ones((ncam,),"i")
#if NCAMERAS%2==1:
#    camPerGrab[-1]=1
npxly=numpy.zeros((ncam,),numpy.int32)

npxly[:]=nsub*npxl #square subaps
npxly[:]=nsub*ynpxl #rectangular subaps
npxly[:]=260 #manually set for 10G camera

npxlx=npxly.copy() #square subaps
npxlx[:]=nsub*xnpxl #rectangular subaps
npxlx[:]=896 #manually set for 10G camera

nsuby=npxly.copy()
nsuby[:]=nsub
nsubx=nsuby.copy()
nsubaps=(nsuby*nsubx).sum()
individualSubapFlag=tel.Pupil(nsub,nsub/2.,2.7*nsub/40.,nsub).subflag.astype("i")#gives 1240 used subaps
subapFlag=numpy.zeros(((nsuby*nsubx).sum(),),"i")
for i in range(ncam):
    tmp=subapFlag[(nsuby[:i]*nsubx[:i]).sum():(nsuby[:i+1]*nsubx[:i+1]).sum()]
    tmp.shape=nsuby[i],nsubx[i]
    tmp[:]=individualSubapFlag
#ncents=nsubaps*2
ncents=subapFlag.sum()*2
npxls=(npxly*npxlx).sum()

nvSubaps = subapFlag.sum() # number of valid subaps

fakeCCDImage=None#(numpy.random.random((npxls,))*20).astype("i")
#camimg=(numpy.random.random((10,npxls))*20).astype(numpy.int16)

bgImage=numpy.ones((npxls,),"f")#None#FITS.Read("shimgb1stripped_bg.fits")[1].astype("f")#numpy.zeros((npxls,),"f")
darkNoise=None#FITS.Read("shimgb1stripped_dm.fits")[1].astype("f")
flatField=numpy.random.random(npxls).astype("f")*0.1+0.95###None#FITS.Read("shimgb1stripped_ff.fits")[1].astype("f")
#indx=0
#nx=npxlx/nsubx
#ny=npxly/nsuby
#correlationPSF=numpy.zeros((npxls,),numpy.float32)


subapLocation=numpy.zeros((nsubaps,6),"i")
nsubapsCum=numpy.zeros((ncam+1,),numpy.int32)
nvSubapsCum=numpy.zeros((ncam+1,),numpy.int32)
ncentsCum=numpy.zeros((ncam+1,),numpy.int32)
for i in range(ncam):
    nsubapsCum[i+1]=nsubapsCum[i]+nsuby[i]*nsubx[i]
    nvSubapsCum[i+1]=nvSubapsCum[i]+nvSubaps
    ncentsCum[i+1]=ncentsCum[i]+subapFlag[nsubapsCum[i]:nsubapsCum[i+1]].sum()*2

# now set up a default subap location array...
#this defines the location of the subapertures.

subx=[npxl]*ncam#(npxlx-16*camPerGrab)/nsubx  #square subaps
suby=[npxl]*ncam#(npxly-16)/nsuby             #square subaps
subx=[xnpxl]*ncam                             #rectangular subaps
suby=[ynpxl]*ncam                             #rectangular subaps

xoff=[0]*ncam
yoff=[0]*ncam
for k in range(ncam):
    nc=camPerGrab[k]
    for i in range(nsuby[k]):
        for j in range(nsubx[k]):
            indx=nsubapsCum[k]+i*nsubx[k]+j
            if subapFlag[indx]:
                subapLocation[indx]=(yoff[k]+i*suby[k],yoff[k]+i*suby[k]+suby[k],1,xoff[k]*nc+j/nc*subx[k]*nc+j%nc,xoff[k]*nc+j/nc*subx[k]*nc+subx[k]*nc+j%nc,nc)
#print "subapLocation",subapLocation


pxlCnt=numpy.zeros((nsubaps,),"i")
subapAllocation=numpy.zeros((nsubaps,),"i")-1
subapMult = 8  #multiple of subaps per thread
subapsPerThread = ((nvSubaps//ncam))/(nthreads//ncam)
print "subapsPerThread = ",subapsPerThread
subapsPerThread = ((nvSubaps//ncam)+(nthreads//ncam)-1)/(nthreads//ncam) #make sure it's rounded up for number of threads
print "subapsPerThread = ",subapsPerThread
subapsPerThread = subapMult*((subapsPerThread+subapMult-1)/subapMult) # round up to subapMult
print "rounded subapsPerThread = ",subapsPerThread

print "valid subaps = ", nvSubaps
print "total subaps = ", nsubaps
print "\"max subaps\" = ",nthreads*subapsPerThread # max subaps if all threads processed subapsPerThread subaps, must be >=nvSubaps

threadCount = 0
firstIndex = 0
j = 0
for k in range(ncam):
    firstIndex = 0
    for i in range(nsubaps/ncam):
      if subapFlag[i]==1:
        subapAllocation[k*nsubaps/ncam+i] = threadCount
        pxlCnt[k*nsubaps/ncam+firstIndex:k*nsubaps/ncam+i+1] = (npxlx[k]*suby[k])*((i+nsub)/nsub)
        j+=1
      if j>=subapsPerThread:
          threadCount+=1
          firstIndex = i+1
          j=0
    j=0
    threadCount+=1


for i in range(ncam*nsub):
    print '[%s]' % (' '.join('%05s' % j for j in pxlCnt[i*nsub:(i+1)*nsub]))
for i in range(ncam*nsub):
    print '[%s]' % (' '.join('%02s' % j for j in subapAllocation[i*nsub:(i+1)*nsub]))

threadCounts = numpy.zeros(nthreads+1,dtype='i')
for i in range(len(subapAllocation)):
    if subapFlag[i]:
        threadCounts[subapAllocation[i]]+=1
for j in range(nthreads):
	print j, threadCounts[j]
# for verification of subapAllocation, print the array
#for i in range(ncam*nsub):
#    print '[%s]' % (' '.join('%02s' % j for j in subapAllocation[i*nsub:(i+1)*nsub]))

# for verification of subapsPerThread, print the number of subaps processed by each thread
#threadCounts = numpy.zeros(nthreads+1,dtype='i')
#for i in range(len(subapAllocation)):
#    if subapFlag[i]:
#        threadCounts[subapAllocation[i]]+=1
#print "thread\tsubaps"
#for j in range(nthreads):
#    print j, "\t",threadCounts[j]

#The params are dependent on the interface library used.
#rams[-1]=1#wpu correction


fname="/home/djenkins/git/darc/test/img3x{0}x{1}.fits".format(npxly,npxlx)

#option to save a random array
#arr=(numpy.random.random((3,npxly,npxlx))*1000).astype(numpy.uint16)
#FITS.Write(arr,fname)

#if not os.path.exists(fname):
    #fname="/rtc/test/img3x128x128.fits"

while len(fname)%4!=0:#zero pad to it fits into 32bit int array
    fname+="\0"
fname+="1111"


camerasOpen = 1

##### For aravis camera ######
cameraName = "libcamAravis.so"
camList=["EVT-20007"][:ncam]
camNames=string.join(camList,";")#"Imperx, inc.-110323;Imperx, inc.-110324"
print camNames
while len(camNames)%4!=0:
    camNames+="\0"
namelen=len(camNames)
cameraParams=numpy.zeros((10*ncam+4+(namelen+3)//4,),numpy.int32)
cameraParams[0:ncam]=8#8 bpp
cameraParams[ncam:2*ncam]=65536#block size - 32 rows in this case
cameraParams[2*ncam:3*ncam]=0#x offset
cameraParams[3*ncam:4*ncam]=0#y offset
cameraParams[4*ncam:5*ncam]=npxlx#camnpxlx
cameraParams[5*ncam:6*ncam]=npxly#camnpxly
cameraParams[6*ncam:7*ncam]=0#byteswap
cameraParams[7*ncam:8*ncam]=0#reorder
cameraParams[8*ncam:9*ncam]=99#t50#priority
cameraParams[9*ncam]=2#affin el size
cameraParams[9*ncam+1:9*ncam+2]=camAffin[0]#0x200#0xfc0fc0#affinity
cameraParams[9*ncam+2:10*ncam+2]=camAffin[1]#camAffin#0x200#0xfc0fc0#affinity
cameraParams[10*ncam+2]=namelen#number of bytes for the name.
cameraParams[10*ncam+3:10*ncam+3+(namelen+3)//4].view("c")[:]=camNames
cameraParams[10*ncam+3+(namelen+3)//4]=0#record timestamp
print 10*ncam+4+(namelen+3)//4,cameraParams

camCommand="FrameRate=20;"
#camCommand=None

##############################

camerasOpen = 0
#~ cameraName = ''

centroiderParams=numpy.zeros((5*ncam,),numpy.int32)
centroiderParams[0::5]=18#blocksize
centroiderParams[1::5]=1000#timeout/ms
centroiderParams[2::5]=range(ncam)#port
centroiderParams[3::5]=-1#thread affinity
centroiderParams[4::5]=1#thread priority
rmx=numpy.zeros((nacts,ncents)).astype("f")#FITS.Read("rmxRTC.fits")[1].transpose().astype("f")
print rmx.shape
mirrorParams=numpy.zeros((4,),"i")
mirrorParams[0]=1000#timeout/ms
mirrorParams[1]=2#port
mirrorParams[2]=-1#thread affinity
mirrorParams[3]=1#thread prioirty

#Now describe the DM - this is for the GUI only, not the RTC.
#The format is: ndms, N for each DM, actuator numbers...
#Where ndms is the number of DMs, N is the number of linear actuators for each, and the actuator numbers are then an array of size NxN with entries -1 for unused actuators, or the actuator number that will set this actuator in the DMC array.

dmDescription=numpy.zeros((17*17+1+1,),numpy.int16)
dmDescription[0]=1#1 DM
dmDescription[1]=17#1st DM has 2 linear actuators
tmp=dmDescription[2:]
tmp[:]=-1
tmp.shape=17,17
dmflag=tel.Pupil(17,17/2.,1).fn.ravel()
numpy.put(tmp,dmflag.nonzero()[0],numpy.arange(nacts))

# setup for treeAdd functionality

######## tree add config for 2->2->2 ###########
nlayers = numpy.array([5,],dtype="i")[0]
nparts = numpy.array([23,],dtype="i")[0]        #0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22
barrierWaits64 = numpy.array([4,4,4,4,4,4,4,4,4,4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 2, 2, 2],dtype="i")
barrierWaits = numpy.zeros_like(barrierWaits64)
threadArray = numpy.array([
     0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9,10,10,10,10,11,11,11,11,12,12,12,12,13,13,13,13,14,14,14,14,15,15,15,15,
    16,-2,-2,-2,16,-2,-2,-2,16,-2,-2,-2,16,-2,-2,-2,17,-2,-2,-2,17,-2,-2,-2,17,-2,-2,-2,17,-2,-2,-2,18,-2,-2,-2,18,-2,-2,-2,18,-2,-2,-2,18,-2,-2,-2,19,-2,-2,-2,19,-2,-2,-2,19,-2,-2,-2,19,-2,-2,-2,
    20,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,20,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,21,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,21,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,
    22,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,22,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,
    -1,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,
],dtype='i')

if nthreads==1:
    threadArray[0]=-1
if nthreads<5:
    threadArray[64]=-1
if nthreads<17:
    threadArray[128]=-1
if nthreads<33:
    threadArray[256]=-1


######### tree add config for 8->4->2 add #########
#~ nlayers = numpy.array([4,],dtype="i")[0]
#~ nparts = numpy.array([11,],dtype="i")[0]
#~ barrierWaits64 = numpy.array([8,8,8,8,8,8,8,8,4,4,2],dtype='i')
#~ barrierWaits = numpy.zeros_like(barrierWaits64)
#~ threadArray = numpy.array([
     #~ 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7,
     #~ 8,-2,-2,-2,-2,-2,-2,-2, 8,-2,-2,-2,-2,-2,-2,-2, 8,-2,-2,-2,-2,-2,-2,-2, 8,-2,-2,-2,-2,-2,-2,-2, 9,-2,-2,-2,-2,-2,-2,-2, 9,-2,-2,-2,-2,-2,-2,-2, 9,-2,-2,-2,-2,-2,-2,-2, 9,-2,-2,-2,-2,-2,-2,-2,
    #~ 10,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,10,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,
    #~ -1,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,
#~ ],dtype='i')
#~ if nthreads==1:
    #~ threadArray[0]=-1
#~ if nthreads<9:
    #~ threadArray[64]=-1
#~ if nthreads<33:
    #~ threadArray[128]=-1


partArray = numpy.zeros(nthreads*nlayers,dtype="i")
for i in range(nlayers):
    for j in range(nthreads):
        partArray[i*nthreads+j] = threadArray[i*64+j];
        if threadArray[i*64+j]>-1:
            barrierWaits[threadArray[i*64+j]]+=1



control={
    "switchRequested":0,#this is the only item in a currently active buffer that can be changed...
    "pause":0,
    "go":1,
    "maxClipped":nacts,
    "refCentroids":numpy.random.random(ncents).astype("f")*0.1-0.05,#None,
    "centroidMode":"CoG",#whether data is from cameras or from WPU.
    "windowMode":"basic",
    "thresholdAlgo":1,
    "reconstructMode":"truth",#simple (matrix vector only), truth or open
    "centroidWeight":None,
    "v0":None,#numpy.ones((nacts,),"f")*32768,#v0 from the tomograhpcic algorithm in openloop (see spec)
    "bleedGain":0.0,#0.05,#a gain for the piston bleed...
    "actMax":numpy.ones((nacts,),numpy.uint16)*65535,#4095,#max actuator value
    "actMin":numpy.zeros((nacts,),numpy.uint16),#4095,#max actuator value
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
    "thresholdValue":0.,#could also be an array.
    "powerFactor":1.,#raise pixel values to this power.
    "subapFlag":subapFlag,
    "fakeCCDImage":fakeCCDImage,
    "printTime":0,#whether to print time/Hz
    "rmx":rmx,#numpy.random.random((nacts,ncents)).astype("f"),
    "gain":numpy.ones((nacts,),"f"),
    "E":None,#numpy.zeros((nacts,nacts),"f"),#E from the tomoalgo in openloop.
    "threadAffElSize":threadAffElSize,
    "threadAffinity":threadAffinity,
    "threadPriority":threadPriority,
    "delay":0,
    "clearErrors":0,
    "camerasOpen":camerasOpen,
    "camerasFraming":0,
    "cameraName":cameraName,#"libcamfile.so",#"libsl240Int32cam.so",#"camfile",
    "cameraParams":cameraParams,
    "mirrorName":"libmirrorSL240.so",
    "mirrorParams":mirrorParams,
    "mirrorOpen":0,
    "frameno":0,
    "switchTime":numpy.zeros((1,),"d")[0],
    "adaptiveWinGain":0.5,
    "nsubapsTogether":1,
    "nsteps":0,
    "addActuators":1,
    "actuators":numpy.ones((nacts),"f")*32768,#None,#(numpy.random.random((3,52))*1000).astype("H"),#None,#an array of actuator values.
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
    "figureParams":numpy.array([1000,3,0xffff,1]).astype("i"),#timeout,port,affinity,priority
    "reconName":"libreconmvm.so",
    "fluxThreshold":0,
    "printUnused":1,
    "useBrightest":0,
    "figureGain":1,
    "decayFactor":None,#used in libreconmvm.so
    "reconlibOpen":1,
    "maxAdapOffset":0,
    "version":" "*120,
    "noPrePostThread":noPrePostThread,
    #"noPrePostThread":1,
    "calibrateOpen":1,
    "slopeOpen":1,
    "subapAllocation":subapAllocation,
    "partArray":partArray,
    "barrierWaits":barrierWaits,
    "nparts":nparts,
    "nlayers":nlayers,
    }

#control["pxlCnt"][-3:]=npxls#not necessary, but means the RTC reads in all of the pixels... so that the display shows whole image
if camCommand!=None:
    for i in range(ncam):
        control["aravisCmd%d"%i]=camCommand
