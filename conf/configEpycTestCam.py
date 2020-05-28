#!/usr/bin/python
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

#Best for phi: 54 threads, prio=39,99,99,99,99...
from __future__ import print_function
#import correlation
import os
import sys
import string
import FITS
import tel
import numa
import numpy
import socket
hostName = socket.gethostname()

import inspect
import imp

import multiprocessing
mcpu = multiprocessing.cpu_count()
filepath = os.path.join(os.path.dirname(inspect.stack()[0][1]),"darcEPYC.py")
darcEPYC = imp.load_source('darcEPYC',filepath)

darcEPYC.printon = 1
printDEBUG = darcEPYC.printDEBUG

# numpy.set_printoptions(linewidth=500)

macAddrsMul=  darcEPYC.macAddrsMul
macAddrsSim=  darcEPYC.macAddrsSim
macAddrsUni=  darcEPYC.macAddrsUni

camMulticast=True
NCAMERAS=1   #1, 2, 3, 4.  This is the number of physical cameras
ncam=NCAMERAS#(int(NCAMERAS)+1)/2

nsub=80 #should be 80 for E-ELT...

npxl = 800//nsub

 #total number of threads, maximum 58 for 64 core machine, 6 cores free for OS and main, camera and mirror threads
# nn=nthreads//ncam #threads per camera
# ncamThreads=numpy.ones((ncam,),numpy.int32)*nn
# nthreads = ncamThreads.sum() # new total number of threads, multiple of ncam

noPrePostThread=0

# threadinfo = darcEPYC.getThreadInfo(nthreads)

# globals().update(threadinfo)

# npxl=10 # for square subaps
xnpxl=npxl # width for rectangular subaps
ynpxl=npxl #height for rectangular subaps
# nact=(nsub+1)
#print "nact = ", nact
# nacts=tel.Pupil(nact,nact/2.,2).fn.astype("i").sum() #gives 5160
# printDEBUG("nacts1",nacts)
# nacts=16*((nacts+15)/16) # round to 5168
nacts=5318 #manually set to m4 size + m5
# nactsM4=5316#tel.Pupil(75,75/2.*1.013,2).fn.astype("i").sum()#5800
# nactsM5=2
# nactsLGS=12
# nactsDM1 = 500
# nactsDM2 = 500
# nacts = nactsM4 + nactsM5 + nactsLGS + nactsDM1 + nactsDM2
# nacts=5170
# nacts=16*((nacts+15)/16)
printDEBUG("nacts",nacts)
camPerGrab=numpy.ones((ncam,),"i")
#if NCAMERAS%2==1:
#    camPerGrab[-1]=1
npxly=numpy.zeros((ncam,),numpy.int32)

# npxly[:]=nsub*npxl #square subaps
# npxly[:]=nsub*ynpxl #rectangular subaps
# npxly[:]=720 #manually set for 10G camera
npxly[:]=800 #manually set for 10G camera

npxlx=npxly.copy() #square subaps
# npxlx[:]=nsub*xnpxl #rectangular subaps
# npxlx[:]=240 #manually set for 10G camera
npxlx[:]=800 #manually set for 10G camera

nsuby=npxly.copy()
nsuby[:]=nsub
nsubx=nsuby.copy()
nsubaps=(nsuby*nsubx).sum()
individualSubapFlag=tel.Pupil(nsub,nsub/2.,nsub/2./3.5,nsub).subflag.astype("i")#gives 1240 used subaps
printDEBUG("subap flag shape: ",individualSubapFlag.shape)

# individualSubapFlag[:,::2]=0
# individualSubapFlag[::2,:]=0

subapFlag=numpy.zeros(((nsuby*nsubx).sum(),),"i")
for i in range(ncam):
    tmp=subapFlag[(nsuby[:i]*nsubx[:i]).sum():(nsuby[:i+1]*nsubx[:i+1]).sum()]
    tmp.shape=nsuby[i],nsubx[i]
    tmp[:]=individualSubapFlag
#ncents=nsubaps*2
ncents=subapFlag.sum()*2
npxls=(npxly*npxlx).sum()

# nthreads = 28

# nvSubaps = subapFlag.sum() # number of valid subaps
# printDEBUG("no. of valid subaps = ",nvSubaps)
# subapAllocation,nthreads = darcEPYC.getSubapAllocation(nsub,ncam,subapFlag,nthreads)

nthreads = 54
nn=(nthreads)//ncam #threads per camera
ncamThreads=numpy.ones((ncam,),numpy.int32)*nn
nthreads = ncamThreads.sum() # new total number of threads, multiple of ncam

threadPriority=99*numpy.ones((nthreads+1,),numpy.uint32)
# for i in range(nthreads): # use to set individual priorities
# 	threadPriority[i+1] = 97
#threadPriority=numpy.arange(nthreads+1)+40
threadPriority[0]=99   # first thread 39 priority
printDEBUG(threadPriority)

threadAffElSize = 2
threadAffinity=numpy.zeros(((1+nthreads)*threadAffElSize),dtype=numpy.uint32)

camAffin = numpy.zeros(1,dtype=numpy.uint32)
mirAffin = numpy.zeros(2,dtype=numpy.uint32)
camAffin[0]=1<<8 #1 thread
mirAffin[0]=1<<12

corelist = [1,2,3,5,6,7,9,10,11]+range(13,32)+[33,34,35,37,38,39,41,42,43]+range(45,64)
# corelist = [i//3 for i in range(3,192,4)]
print(len(corelist))

# threadAffinity[:] = [1<<i for i in corelist[:nthreads+1]]

for i in range(nthreads):
    j = corelist[i]
    threadAffinity[(i)*threadAffElSize+j//32]=1<<(j%32)
# for i in range(0,nthreads):
#     print(i)
#     if (i<mcpu-2):
#         j=1+i if i<3 else 1+i+1
#         threadAffinity[(i)*threadAffElSize]=1<<j
#     else:
#         printDEBUG("too many threads..")
#         exit(0)

threadAffinity[threadAffElSize:] = threadAffinity[:-threadAffElSize]
threadAffinity[:threadAffElSize] = 0
threadAffinity[0] = 1<<4 # control thread affinity

fakeCCDImage=None#(numpy.random.random((npxls,))*20).astype("i")
#camimg=(numpy.random.random((10,npxls))*20).astype(numpy.int16)

bgImage=None#numpy.ones((npxls,),"f")#None#FITS.Read("shimgb1stripped_bg.fits")[1].astype("f")#numpy.zeros((npxls,),"f")
darkNoise=None#numpy.random.random(npxls).astype("f")#None#FITS.Read("shimgb1stripped_dm.fits")[1].astype("f")
flatField=None#numpy.random.random(npxls).astype("f")*0.1+0.95###None#FITS.Read("shimgb1stripped_ff.fits")[1].astype("f")
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
    nvSubapsCum[i+1]=nvSubapsCum[i]+subapFlag[nsubapsCum[i]:nsubapsCum[i+1]].sum()
    ncentsCum[i+1]=ncentsCum[i]+subapFlag[nsubapsCum[i]:nsubapsCum[i+1]].sum()*2

# now set up a default subap location array...
#this defines the location of the subapertures.

# subx=[npxl]*ncam#(npxlx-16*camPerGrab)/nsubx  #square subaps
# suby=[npxl]*ncam#(npxly-16)/nsuby             #square subaps
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
# printDEBUG("subapLocation",subapLocation)


# pxlCnt=numpy.zeros((nsubaps,),"i")
# subapAllocation=numpy.zeros((nsubaps,),"i")-1
# subapMult = 8  #multiple of subaps per thread
# subapsPerThread = ((nvSubaps//ncam))/(nthreads//ncam)
# printDEBUG("subapsPerThread = ",subapsPerThread)
# subapsPerThread = ((nvSubaps//ncam)+(nthreads//ncam)-1)/(nthreads//ncam) #make sure it's rounded up for number of threads
# printDEBUG("subapsPerThread = ",subapsPerThread)
# subapsPerThread = subapMult*((subapsPerThread+subapMult-1)/subapMult) # round up to subapMult
# printDEBUG("rounded subapsPerThread = ",subapsPerThread)

# printDEBUG("valid subaps = ", nvSubaps)
# printDEBUG("total subaps = ", nsubaps)
# printDEBUG("\"max subaps\" = ",nthreads*subapsPerThread) # max subaps if all threads processed subapsPerThread subaps, must be >=nvSubaps

# threadCount = 0
# firstIndex = 0
# j = 0
# mult0 = 10#12
# mult  = mult0
# for k in range(ncam):
#     firstIndex = 0
#     for i in range(nsubaps/ncam):
#       if subapFlag[i]==1:
#         subapAllocation[k*nsubaps/ncam+i] = threadCount
#         pxlCnt[k*nsubaps/ncam+firstIndex:k*nsubaps/ncam+i+1] = (npxlx[k]*suby[k])*((i+nsub)/nsub)
#         j+=1
#       if j>=(subapsPerThread+mult*subapMult):
#           threadCount+=1
#           firstIndex = i+1
#           j=0
#           mult=int(mult0-threadCount*2.*mult0/(nthreads))
#     j=0
#     threadCount+=1
# pxlCnt=numpy.zeros((nsubaps,),"i")
# for i in range(len(subapAllocation)):
#     thread = subapAllocation[i]
#     flag = subapFlag[i]
#     if flag==0:
#         pxlCnt[i]=0
#     else:
#         pxlCnt[i] = ((numpy.where(subapAllocation==thread)[0][-1]+nsub)/nsub)*(npxlx[k]*suby[k])


pxlCnt=numpy.zeros((nsubaps,),"i")
subapAllocation=numpy.zeros((nsubaps,),"i")-1
for i in range(nsub):
    for j in range(nsub):
        if subapFlag[i*nsub+j]:
            pxlCnt[i*nsub+j] = (i+1)*(npxlx[k]*(suby[k]))
            subapAllocation[i*nsub+j]=i%nthreads
# subapAllocation=None


# pxlCnt[:] = 800*800
# if darcEPYC.printon:
#     for i in range(ncam*nsub):
#         printDEBUG('[%s]' % (' '.join('%05s' % j for j in pxlCnt[i*nsub:(i+1)*nsub])))
#     for i in range(ncam*nsub):
#         printDEBUG('[%s],' % (' '.join('%02s,' % j for j in subapAllocation[i*nsub:(i+1)*nsub])))
#     printDEBUG("printed the allocation")
# threadCounts = numpy.zeros(nthreads+1,dtype='i')
# for i in range(len(subapAllocation)):
#     if subapFlag[i]:
#         threadCounts[subapAllocation[i]]+=1
# if darcEPYC.printon:
#     for i in range(nthreads):
#         try:
#             j = numpy.where(subapAllocation==i)[0][-1]
#             printDEBUG("thread {}, last subap={}, waiting for {} pixels".format(i,j,pxlCnt[j]))
#         except Exception:
#             printDEBUG("No more pixels")
#     # print thread affinity and subaps per thread
#     printDEBUG("thread\t\tcore\tsubaps\tbinary affin")
#     printDEBUG("______________________________________________")
#     bs = [bin(i) for i in mirAffin]
#     for x in range(len(bs)):
#             b = bs[x]
#             l = [k for k in len(b)-numpy.array([j for j in range(len(b)) if b[j]=='1'])]
#             if len(l):
#                 printDEBUG("mirror\t\t", x*32+l[0]-1,"\t\t", [bin(j) for j in mirAffin])

#     bs = [bin(i) for i in camAffin]
#     for x in range(len(bs)):
#             b = bs[x]
#             l = [k for k in len(b)-numpy.array([j for j in range(len(b)) if b[j]=='1'])]
#             if len(l):
#                 printDEBUG("camera\t\t", x*32+l[0]-1,"\t\t", [bin(j) for j in camAffin])

#     # print out threadAfinity in binary form for verification
#     for i in range(nthreads+1):
#         #print i,[bin(j) for j in threadAffinity[i*threadAffElSize:(i+1)*threadAffElSize]]
#         bs = [bin(j) for j in threadAffinity[i*threadAffElSize:(i+1)*threadAffElSize]]
#         for x in range(len(bs)):
#             b = bs[x]
#             l = [k for k in len(b)-numpy.array([j for j in range(len(b)) if b[j]=='1'])]
#             if len(l):
#                 printDEBUG("recon:{}".format(i) if i>0 else "darc\t","\t", x*32+l[0]-1,"\t", threadCounts[i-1] if i>0 else "","\t", [bin(j) for j in threadAffinity[i*threadAffElSize:(i+1)*threadAffElSize]])


# for verification of subapAllocation, print the array
# for i in range(ncam*nsub):
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

# fits file for camfile cam lib
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
# cameraName = "libcamAravis.so"
# # camList=["EVT-20007"][:ncam]
# camList=["Aravis-david-{}Cam".format(os.environ['HOSTNAME'])][:ncam]
# camNames=string.join(camList,";")#"Imperx, inc.-110323;Imperx, inc.-110324"
# printDEBUG(camNames)
# while len(camNames)%4!=0:
#     camNames+="\0"
# namelen=len(camNames)
# cameraParams=numpy.zeros((10*ncam+4+(namelen+3)//4,),numpy.int32)
# cameraParams[0:ncam]=16#8 bpp
# cameraParams[ncam:2*ncam]=65536#block size - 32 rows in this case
# cameraParams[2*ncam:3*ncam]=0#x offset
# cameraParams[3*ncam:4*ncam]=0#y offset
# cameraParams[4*ncam:5*ncam]=npxlx#camnpxlx
# cameraParams[5*ncam:6*ncam]=npxly#camnpxly
# cameraParams[6*ncam:7*ncam]=0#byteswap
# cameraParams[7*ncam:8*ncam]=0#reorder
# cameraParams[8*ncam:9*ncam]=99#t50#priority
# cameraParams[9*ncam]=2#affin el size
# cameraParams[9*ncam+1:9*ncam+2]=camAffin[0]#0x200#0xfc0fc0#affinity
# cameraParams[9*ncam+2:10*ncam+2]=camAffin[1]#camAffin#0x200#0xfc0fc0#affinity
# cameraParams[10*ncam+2]=namelen#number of bytes for the name.
# cameraParams[10*ncam+3:10*ncam+3+(namelen+3)//4].view("c")[:]=camNames
# cameraParams[10*ncam+3+(namelen+3)//4]=1#record timestamp
# printDEBUG(10*ncam+4+(namelen+3)//4,cameraParams)

# camCommand="AcquisitionFrameRate=30;"
# camCommand="FrameRate=30;"
# camCommand=None

##############################

##### For lvsmSim camera ######
cameraName = "libcamrtdnpPacketSocket.so"
camAffElSz = 1
# cameraParams=numpy.zeros((15*ncam+1+camAffElSz*ncam+1,),dtype="i")
cameraParams=numpy.zeros((15*ncam+2+camAffElSz*ncam+1,),dtype="i")
cameraParams[0*ncam:1*ncam]=2#bytes per pixel
cameraParams[1*ncam:2*ncam]=range(9000,9000+ncam)#port
if camMulticast:
    cameraParams[2*ncam:3*ncam]=macAddrsMul[hostName][2]#host interface to bind to (can be got in python3 using socket.if_nametoindex("eth0"))
    cameraParams[3*ncam:4*ncam]=socket.htonl(0x01005e00)#multicast mac address first 4 bytes
    cameraParams[4*ncam:5*ncam]=socket.htonl(0x00fa1111)#multicast mac address next 2 bytes and a flag to turn on multicasting.
else:
    cameraParams[2*ncam:3*ncam]=macAddrsUni[hostName][2]#host interface to bind to (can be got in python3 using socket.if_nametoindex("eth0"))
    cameraParams[3*ncam:4*ncam]=socket.htonl(0x00000000)#multicast mac address first 4 bytes
    cameraParams[4*ncam:5*ncam]=socket.htonl(0x00000000)#multicast mac address next 2 bytes and a flag to turn on multicasting.
#To use multicasting, do something like:
#cameraParams[3*ncam:5*ncam]=socket.htonl(0xaabbccdd),socket.htonl(0xeeff1)#(note the 1 after ff).
#For a multicast IP a.b.c.d, the MAC address is basically given as:
#01:00:50|(a>>4):b&0x7fffffff:c:d

cameraParams[5*ncam:6*ncam]=8000#blocksize (unused)
cameraParams[6*ncam:7*ncam]=0#reorder
cameraParams[7*ncam:8*ncam]=10#topicId
cameraParams[8*ncam:9*ncam]=99#componentId
cameraParams[9*ncam:10*ncam]=0#application tag     3c:fd:fe:a4:9b:18
if camMulticast:
    cameraParams[10*ncam:12*ncam]=socket.htonl(0x3cfdfea4),socket.htonl(0x9b180000)#source mac address e.g. socket.This would be, for a mac address of aa:bb:cc:dd:ee:ff, socket.htonl(0xaabbccdd),socket.htonl(0xeeff0000)
    cameraParams[12*ncam:14*ncam]=socket.htonl(macAddrsMul[hostName][0]),socket.htonl(macAddrsMul[hostName][1])#my mac address e.g. socket.This would be, for a mac address of aa:bb:cc:dd:ee:ff, socket.htonl(0xaabbccdd),socket.htonl(0xeeff0000)
else:
    cameraParams[10*ncam:12*ncam]=socket.htonl(macAddrsSim[hostName][0]),socket.htonl(macAddrsSim[hostName][1])#source mac address e.g. socket.This would be, for a mac address of aa:bb:cc:dd:ee:ff, socket.htonl(0xaabbccdd),socket.htonl(0xeeff0000)
    cameraParams[12*ncam:14*ncam]=socket.htonl(macAddrsUni[hostName][0]),socket.htonl(macAddrsUni[hostName][1])#my mac address e.g. socket.This would be, for a mac address of aa:bb:cc:dd:ee:ff, socket.htonl(0xaabbccdd),socket.htonl(0xeeff0000)
cameraParams[14*ncam:15*ncam]=98#priority of receiving threads
cameraParams[15*ncam:15*ncam+1]=1#affin el size
cameraParams[15*ncam+1:15*ncam+1+camAffElSz*ncam]=camAffin[0]#thread affinity
cameraParams[15*ncam+1+camAffElSz*ncam:15*ncam+1+camAffElSz*ncam+1]=1#record timestamp
cameraParams[15*ncam+1+camAffElSz*ncam+1:15*ncam+1+camAffElSz*ncam+2]=2#4#2#4#4#bytespp

camCommand=None
camerasOpen = 1
camerasOpen = 0
#~ cameraName = ''

centroiderParams=numpy.zeros((5*ncam,),numpy.int32)
centroiderParams[0::5]=18#blocksize
centroiderParams[1::5]=1000#timeout/ms
centroiderParams[2::5]=list(range(ncam))#port
centroiderParams[3::5]=-1#thread affinity
centroiderParams[4::5]=1#thread priority
rmx=numpy.zeros((nacts,ncents)).astype("f")#FITS.Read("rmxRTC.fits")[1].transpose().astype("f")
gain=numpy.ones((nacts,),"f")
printDEBUG(rmx.shape)

# mirrorName="libmirrorSendTime.so"
# mirrorOpen=1

# addr="172.27.2.80"#IP address of node with master on it (to which packets should be sent).
# # addr="127.0.0.1"#loopback for testing
# addr+="\0"
# while len(addr)%4!=0:
#     addr+="\0"

# mirrorParams=numpy.zeros((1+len(addr)//4,),"i")
# mirrorParams[0]=8812#port to send data to.
# mirrorParams[1:]=numpy.fromstring(addr,dtype="i")

mirrorName="libmirrorUDP.so"
mirrorOpen=1
addr="225.0.0.251;172.27.2.110"#IP address of node with master on it (to which packets should be sent).
addr+="\0"
while len(addr)%4!=0:
    addr+="\0"

mirrorParams=numpy.zeros((8+len(addr)//4,),"i")
mirrorParams[0]=9000-128#payload - I think MTU size-28 (for the IP/UDP headers), but maybe needs to be smaller.  Maybe should be -68 to be sure.
mirrorParams[1]=8800#port to send data to.
mirrorParams[2]=2#thread affinity el size
mirrorParams[3]=99#thread prioirty
mirrorParams[4]=mirAffin[0]#thread affinity
mirrorParams[5]=mirAffin[1]#thread affinity
mirrorParams[6]=0#don't send the prefix.
mirrorParams[7]=1#asfloat
mirrorParams[8:]=numpy.fromstring(addr,dtype="i")

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


# treeAdd = darcEPYC.getTreeParams(nthreads,[2,3],0)
# treeAdd = darcEPYC.getTreeParams(nthreads,[2,2,2,3],0)
treeAdd = darcEPYC.getTreeParams(nthreads,[2],0)
# treeAdd["treeNlayers"]=1
# treeAdd["treeNparts"]=1
# treeAdd={}
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
    "gain":gain,
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
    "mirrorName":mirrorName,
    "mirrorParams":mirrorParams,
    "mirrorOpen":mirrorOpen,
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
    "calibrateOpen":1,
    "slopeOpen":1,
    "subapAllocation":subapAllocation,
    }

control.update(treeAdd)
# exit(0)
"""With subapAllocation: partial rmx on each numa node"""
nthr=ncamThreads.sum()
nnodes=numa.get_max_node()+1
#specify which numa nodes the threads are closest to:
threadToNuma=numpy.zeros(nthr,numpy.int32)#all node 0 for now...!
#Use numactl --hardware to get some of this information... however, note this isn't foolproof, e.g. for the Xeon Phi and other systems with specialised memory.
#Also assumes that the threadAffinity in the config file is set to correspond to this as well.

threadToNuma[:]=(numpy.arange(nthr)+2)//4
threadToNuma[:]=(numpy.array(corelist[:nthr])%32)//4
print("numanodes = ",threadToNuma)
control["threadToNuma"] = threadToNuma

sf=subapFlag
nsub=sf.size
print("nsub = ",nsub)
sa=subapAllocation
gainReconmxT = rmx.transpose().astype(numpy.float32)

for i in range(nnodes):
    control["numa%d"%i]={}

nacts=gain.shape[0]
for i in range(nacts):
    gainReconmxT[:,i]*=gain[i]

thrSubapCnt=numpy.zeros(nthr,numpy.int32)
rmxPart=[]
for i in range(nthr):
    rmxPart.append([])
indx=0
for i in range(nsub):
    if sf[i]:
        thrSubapCnt[sa[i]]+=1
        rmxPart[sa[i]].append(gainReconmxT[indx])
        rmxPart[sa[i]].append(gainReconmxT[indx+1])
        indx+=2
for i in range(nthr):
    r=numpy.array(rmxPart[i])
    print("Thread %d rmx shape %s, dtype %s"%(i,str(r.shape),r.dtype))
    control["numa%d"%threadToNuma[i]]["gainReconmxT%d"%i]=r



# nthr=control.get("ncamThreads").sum()
# print("nthreads =", nthr)
# nnodes=numa.get_max_node()+1
# print("nnodes =", nnodes)
# threadToNuma=numpy.zeros(nthr,numpy.int32)
# sf=control.get("subapFlag")
# nsub=sf.size
# print("nsub =", nsub)
# sa=numpy.zeros(nsub,numpy.int32)
# #divide all threads equally....
# sa[:]=nthr-1
# start=0
# for i in range(nnodes):
#     control["numa%d"%i]={}
# for i in range(nthr):
#     end=start+nsub/nthr
#     sa[start:end]=i
#     start=end
# control["threadToNuma"]=threadToNuma
# gainReconmxT=control["rmx"].transpose().astype(numpy.float32)
# g=control["gain"]
# nacts=g.shape[0]
# for i in range(nacts):
#     gainReconmxT[:,i]*=g[i]#shape nslope,nact
# thrSubapCnt=numpy.zeros(nthr,numpy.int32)
# rmxPart=[]
# for i in range(nthr):
#     rmxPart.append([])
# indx=0
# for i in range(nsub):
#     if sf[i]:
#         thrSubapCnt[sa[i]]+=1
#         rmxPart[sa[i]].append(gainReconmxT[indx])
#         rmxPart[sa[i]].append(gainReconmxT[indx+1])
#         indx+=2
# for i in range(nthr):
#     r=numpy.array(rmxPart[i])
#     print("Thread %d node %d rmx shape %s, dtype %s"%(i,threadToNuma[i],str(r.shape),r.dtype))
#     #numaList[threadToNuma[i]]["gainReconmxT%d"%i]=r
#     #d.SetNuma("gainReconmxT%d"%i,r,int(threadToNuma[i]),swap=1)
#     control["numa%d"%threadToNuma[i]]["gainReconmxT%d"%i]=r
# control["subapAllocation"]=sa
# #for i in range(nnodes):
# #    control["numa%d"%i]=numaList[i]