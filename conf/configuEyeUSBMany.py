#darc, the Durham Adaptive optics Real-time Controller.
#Copyright (C) 2013 Alastair Basden.

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

# This is a configuration file for DARC
# Aim to fill up the control dictionary with values to be used in the RTCS.
#
# The target is configuring just N uEye cameras (camuEyeManyUSB) using the
# prefix as the key for defining N e.g. prefix=ueye6 implies ncam=6.
#
# NAB
# 2014/Nov/12

import string
import numpy
import sys
import re
if 'prefix' not in dir(): prefix="ueye"
print("{0:s}::{1:s}".format(sys.argv[0],prefix))

match=re.search( r"ueye(\d*)", prefix )
if match==None or match.groups()[0]=='':
   ncam=1
   print("{0:s}::assuming one available camera".format(sys.argv[0]))
else:
   ncam=int(match.groups()[0])

print("{0:s}::specifying {1:d} cameras".format(sys.argv[0],ncam))

ncamThreads,nacts=numpy.array([1]*ncam),4 ## REQUIRED, 4==dummy c.f. nsub=1
npxly=numpy.zeros((ncam,),numpy.int32)
npxly[:]=480
npxlx=npxly.copy()
npxlx[:]=640
nsuby=npxlx.copy()
nsuby[:]=1
nsubx=nsuby.copy()
nsub=nsubx*nsuby ## REQUIRED #This is used by rtc.
##nsubaps=nsub.sum()#(nsuby*nsubx).sum()
##print(nsubaps)
##individualSubapFlag=tel.Pupil(30,15,2,30).subflag.astype("i")
##subapFlag=numpy.ones((nsubaps,),"i")
##for i in range(ncam):
##    tmp=subapFlag[nsub[:i].sum():nsub[:i+1].sum()]
##    tmp.shape=nsuby[i],nsubx[i]
##    tmp[:]=individualSubapFlag
##ncents=subapFlag.sum()*2
npxls=(npxly*npxlx).sum()
print("{0:s}::specifying {1:d} pixels".format(sys.argv[0],npxls))

##fakeCCDImage=None#(numpy.random.random((npxls,))*20).astype("i")
##bgImage=None   #FITS.Read("shimgb1stripped_bg.fits")[1].astype("f")#numpy.zeros((npxls,),"f")
##darkNoise=None #FITS.Read("shimgb1stripped_dm.fits")[1].astype("f")
##flatField=None #FITS.Read("shimgb1stripped_ff.fits")[1].astype("f")

##subapLocation=numpy.zeros((nsubaps,6),"i")
##nsubapsCum=numpy.zeros((ncam+1,),numpy.int32)
##ncentsCum=numpy.zeros((ncam+1,),numpy.int32)
##for i in range(ncam):
##    nsubapsCum[i+1]=nsubapsCum[i]+nsub[i]
##    ncentsCum[i+1]=ncentsCum[i]+subapFlag[nsubapsCum[i]:nsubapsCum[i+1]].sum()*2

### now set up a default subap location array...
###this defines the location of the subapertures.
##subx=(npxlx)/nsubx
##suby=(npxly)/nsuby
##xoff=[0]*ncam
##yoff=[0]*ncam
##for k in range(ncam):
##    for i in range(nsuby[k]):
##        for j in range(nsubx[k]):
##            indx=nsubapsCum[k]+i*nsubx[k]+j
##            subapLocation[indx]=(yoff[k]+i*suby[k],yoff[k]+i*suby[k]+suby[k],1,xoff[k]+j*subx[k],xoff[k]+j*subx[k]+subx[k],1)

##pxlCnt=numpy.zeros((nsubaps,),"i")
### set up the pxlCnt array - number of pixels to wait until each subap is ready.  Here assume identical for each camera.
##for k in range(ncam):
##    for i in range(nsub[k]):
##        indx=nsubapsCum[k]+i
##        n=subapLocation[indx,1]*npxlx[k] # whole rows together...
##        pxlCnt[indx]=n

##rmx=numpy.zeros((nacts,ncents)).astype("f")

"""
  Parameters are:
  HID (0=first available camera)
  xpos (0?) 
  ypos (0?) 
  width (640) 
  height (480)
  framerate (30?)
  binning (1=no binning,2=2x binning etc..)

"""
cameraParams=numpy.zeros([7*ncam],numpy.int32)
cameraParams[0*ncam:1*ncam]=0    # HID
cameraParams[1*ncam:2*ncam]=0    # xpos
cameraParams[2*ncam:2*ncam]=0    # ypos
cameraParams[3*ncam:4*ncam]=640  # height
cameraParams[4*ncam:5*ncam]=480  # width
cameraParams[5*ncam:6*ncam]=30   # framerate
cameraParams[6*ncam:7*ncam]=1    # binning


cameraParamsForControlStems=( # make sure alphabetical order
  "uEyeActExp",
  "uEyeBlackLvl",
  "uEyeBoostG",
  "uEyeExpTime",
  "uEyeFrameRte",
  "uEyeGain",
  "uEyeGrabMode",
  "uEyeNFrames",
  "uEyePxlClock",
)
cameraParamsForControlDefaults={
  "uEyeActExp"    : 33, # ?? set to zero
  "uEyeBlackLvl"  : 0,  # 0 or 1
  "uEyeBoostG"    : 0,  # 0 or 1
  "uEyeExpTime"   : 30.0, # [ms] 0=1/framerate, 'f'
  "uEyeFrameRte"  : 30.0,# [Hz] ,'f'
  "uEyeGain"      : 0,  # 0--100
# from uEye.h
#        CAPTMODE_FREERUN                    = 0x01,
#        CAPTMODE_SINGLE                     = 0x02,
#        // software trigger modes
#        CAPTMODE_TRIGGER_SOFT_SINGLE        = 0x10,
#        CAPTMODE_TRIGGER_SOFT_CONTINUOUS    = 0x20,
#        // hardware trigger modes
#        CAPTMODE_TRIGGER_HW_SINGLE          = 0x00100,
#        CAPTMODE_TRIGGER_HW_CONTINUOUS      = 0x00200
  "uEyeGrabMode"  : 1,
  "uEyeNFrames"   : 1,  # >=1
  "uEyePxlClock"  : 30, # [Mhz], 5--30 (ish)
}
cameraParamsForControl={}
for i in range(ncam):
   for k in cameraParamsForControlStems:
      cameraParamsForControl["{0:s}{1:03d}".format(k,i)]=cameraParamsForControlDefaults[k]


control={
#    "actMax":numpy.ones((nacts,),numpy.uint16)*65535,
#    "actMin":numpy.zeros((nacts,),numpy.uint16),
#    "actSequence":None,
#    "actuatorMask":None,
    "actuators":None,
#    "adaptiveGroup":[1,2],
#    "adaptiveWinGain":0.5,
#    "addActuators":0,
    "calibrateOpen":0,
#    "calibrateName":"librtccalibrate.so",
#    "calibrateParams":None,
#    "averageImg":0,
#    "bgImage":bgImage,
#    "bleedGain":0.0,
    
    "camerasOpen":1,
    "cameraName":"libcamuEyeManyUSB.so",
    "cameraParams":cameraParams,
    "camerasFraming":ncam,
    "ncam":ncam,
    "ncamThreads":ncamThreads,
    "npxlx":npxlx,
    "npxly":npxly,
    "clearErrors":0,
    "closeLoop":1,
    "darkNoise":None,
    "decayFactor":None,
    "delay":0,
##    "E":numpy.zeros((nacts,nacts),"f"),#E from the tomoalgo in openloop.
    "fakeCCDImage":None,
    
    "figureOpen":0,
#    "figureGain":1,
#    "figureName":"figureSL240",
#    "figureParams":None,
#    "flatField":flatField,#numpy.random.random((npxls,)).astype("f"),
#    "fluxThreshold":0,
#    "frameno":0,
#    "gain":numpy.ones((nacts,),"f"),
#    "go":1,
#    "maxAdapOffset":0,
#    "maxClipped":nacts,
    
    "mirrorOpen":0,
#    "mirrorName":"libmirror.so",
#    "mirrorParams":None,
    "nacts":nacts, ## REQUIRED
#    "nsteps":0,
#    "nsubapsTogether":1,
    "nsub":nsub, ## REQUIRED
#    "pause":0,
#    "powerFactor":1.,#raise pixel values to this power.
#    "printTime":0,#whether to print time/Hz
#    "printUnused":1,
#    "pxlCnt":pxlCnt,
#    "pxlWeight":None,
    
    "reconlibOpen":0,
#    "reconName":"libreconmvm.so",
#    "reconstructMode":"simple",
#    "recordCents":0,
#    "rmx":rmx,
    "v0":numpy.zeros((nacts,),"f")*32768, ## REQUIRED
    
    "slopeOpen":0,
#    "slopeName":"librtcslope.so",
#    "slopeParams":None,
#    "subapFlag":subapFlag,
#    "subapLocation":subapLocation,
#    "corrFFTPattern":None,
#    "corrPSF":None,
#    "corrThresh":0.,
#    "corrThreshType":0,
#    "centCalBounds":None,
#    "centCalData":None,
#    "centCalSteps":None,
#    "centroidMode":"CoG",
#    "centroidWeight":None,
    "refCentroids":None, ## REQUIRED
#    "averageCent":0,
#    "thresholdAlgo":1,
#    "thresholdValue":0.,#could also be an array.
#    "useBrightest":0,
    
#    "switchRequested":0, # this is the only item in a currently active buffer
#                         # that can be changed...
#    "switchTime":numpy.zeros((1,),"d")[0],
    "threadAffinity":None,
    "threadPriority":numpy.ones((ncamThreads.sum()+1,),numpy.int32)*10,
    "version":" "*120,
    "windowMode":"basic",
    }
for key in cameraParamsForControl:
   control[key]=cameraParamsForControl[key]
   print("{0:s}::Added key to control '{1:s}'".format(sys.argv[0],key))
