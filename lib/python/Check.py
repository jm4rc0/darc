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
import numpy
import FITS
import os
import traceback
class Check:
    def checkNoneOrArray(self,val,size,dtype):
        if type(val)==type([]):
            val=numpy.array(val)
        elif type(val)==type(""):
            if os.path.exists(val):
                print "Loading %s"%val
                val=FITS.Read(val)[1]
            else:
                print "File %s not found"%val
                raise Exception("File %s not found"%val)
        if val==None:
            pass
        elif type(val)==numpy.ndarray:
            val=val.astype(dtype)
            if size!=None:
                val.shape=size,
        else:
            raise Exception("checkNoneOrArray size requested %d"%size)
        return val
    def checkArray(self,val,shape,dtype,raiseShape=0):
        if type(val)==type([]):
            val=numpy.array(val)
        elif type(val)==type(""):
            if os.path.exists(val):
                print "Loading %s"%val
                val=FITS.Read(val)[1]
            else:
                raise Exception("File not found %s"%val)
        if type(val)==numpy.ndarray:
            if dtype!=None:
                val=val.astype(dtype)
            if shape!=None:
                if type(shape)!=type(()):
                    shape=(shape,)
                if val.shape!=shape:
                    print "Warning - shape not quite right?"
                    if raiseShape:
                        raise Exception("checkArray shape")
                try:
                    val.shape=shape
                except:
                    print val.shape,shape
                    raise
        else:
            raise Exception("checkArray")
        return val
    def checkDouble(self,val):
        val=float(val)
        val=numpy.array([val]).astype("d")
        return val

    def checkNoneOrFloat(self,val):
        if val==None:
            pass
        else:
            val=float(val)
        return val
    def checkFlag(self,val):
        val=int(val)
        if val!=0 and val!=1:
            raise Exception("checkFlag")
        return val


    def valid(self,label,val,buf):
        """Checks a value is valid.  buf is the buffer that contains all the other parameters"""
        #buf=self.guibuf
        if label=="reconstructMode":
            if(val not in ["simple","truth","open","offset"]):
                raise Exception(label)
        elif label=="windowMode":
            if val not in ["basic","adaptive","global"]:
                raise Exception(label)
        elif label in ["cameraName","mirrorName","comment","slopeName","figureName","version"]:
            if type(val)!=type(""):
                raise Exception(label)
        elif label in ["reconName","calibrateName","bufferName"]:
            if type(val) not in [type(""),type(None)]:
                raise Exception(label)
        elif label=="centroidMode":
            if val not in ["WPU","CoG","Gaussian","CorrelationCoG","CorrelationGaussian"]:
                raise Exception(label)
        elif label in ["cameraParams","mirrorParams","slopeParams","figureParams","reconParams","calibrateParams","bufferParams"]:
            if type(val)==type(""):
                l=4-len(val)%4
                if l<4:
                    val+="\0"*l
                val=numpy.fromstring(val,dtype=numpy.int32)

            if type(val)==type([]):
                val=numpy.array(val)
            if type(val)!=type(None) and type(val)!=numpy.ndarray:
                print "ERROR in val for %s: %s"%(label,str(val))
                raise Exception(label)
        elif label in ["closeLoop","nacts","thresholdAlgo","delay","maxClipped","camerasFraming","camerasOpen","mirrorOpen","clearErrors","frameno","corrThreshType","nsubapsTogether","nsteps","addActuators","recordCents","averageImg","averageCent","kalmanPhaseSize","figureOpen","printUnused","reconlibOpen","currentErrors","xenicsExposure","calibrateOpen","iterSource","bufferOpen","bufferUseSeq","subapLocType","noPrePostThread","asyncReset","openLoopIfClip"]:
            val=int(val)
        elif label in ["dmDescription"]:
            if val.dtype.char!="h":
                raise Exception("dmDescription type not int16")
        elif label in ["actuators"]:
            val=self.checkNoneOrArray(val,None,"f")
            if val!=None:
                if val.size%buf.get("nacts")==0:
                    # shape okay... multiple of nacts.
                    pass
                else:
                    raise Exception("actuators should be array (nacts,) or (X,nacts)")
        elif label in ["actMax","actMin"]:
            nact=buf.get("nacts")
            try:
                val=self.checkArray(val,nact,None)
            except:
                print "WARNING (Check.py) - actMax/actMin as int now depreciated... this may not work (depending on which version of RTC you're running).  The error was"
                #traceback.print_exc()
                #val=int(val)
                print "Continuing... using %s"%str(val)
        #elif label in["lastActs"]:
        #    nact=buf.get("nacts")
        #    val=self.checkNoneOrArray(val,nact,"H")
        elif label in ["actInit"]:
            val=self.checkNoneOrArray(val,None,"H")
        elif label in ["actMapping"]:
            val=self.checkNoneOrArray(val,None,"i")
        elif label in ["figureActSource"]:
            val=self.checkNoneOrArray(val,None,"i")
        elif label in ["figureActScale","figureActOffset","actScale","actOffset"]:
            val=self.checkNoneOrArray(val,None,"f")
        elif label in ["actuatorMask"]:
            val=self.checkNoneOrArray(val,None,"f")
            if val!=None:
                if val.size==buf.get("nacts"):
                    # shape okay... multiple of nacts.
                    pass
                else:
                    raise Exception("actuatorMask should be array (nacts,)")
        elif label in ["actSequence"]:
            actuators=buf.get("actuators")
            if actuators==None:
                size=None
            else:
                size=actuators.size/buf.get("nacts")
            val=self.checkNoneOrArray(val,size,"i")
        elif label in ["bgImage","flatField","darkNoise","pxlWeight","calmult","calsub","calthr"]:
            val=self.checkNoneOrArray(val,(buf.get("npxlx")*buf.get("npxly")).sum(),"f")
        elif label in ["thresholdValue"]:
            if type(val)==type(""):
                if os.path.exists(val):
                    val=FITS.Read(val)[1]
                else:
                    val=eval(val)
            if type(val)==numpy.ndarray:
                npxls=(buf.get("npxlx")*buf.get("npxly")).sum()
                if val.size==npxls:
                    val=self.checkArray(val,(npxls,),"f")
                else:
                    val=self.checkArray(val,(buf.get("nsub").sum(),),"f")
            else:
                try:
                    val=float(val)
                except:
                    print "thresholdValue: %s"%str(type(val))
                    print "thresholdValue should be float or array of floats"
                    raise
        elif label in ["useBrightest"]:
            if type(val)==type(""):
                if os.path.exists(val):
                    val=FITS.Read(val)[1]
                else:
                    val=eval(val)
            if type(val)==numpy.ndarray:
                val=self.checkArray(val,(buf.get("nsub").sum(),),"i")
            else:
                try:
                    val=int(val)
                except:
                    print "%s: %s"%(label,str(type(val)))
                    print "%s should be int or array of ints size equal to total number of subapertures (valid and invalid)"%label
                    raise
        elif label in ["fluxThreshold"]:
            if type(val)==type(""):
                if os.path.exists(val):
                    val=FITS.Read(val)[1]
                else:
                    val=eval(val)
            if type(val)==numpy.ndarray:
                val=self.checkArray(val,buf.get("subapFlag").sum(),"f")
            else:
                try:
                    val=float(val)
                except:
                    print "%s: %s"%(label,str(type(val)))
                    print "%s should be float or array of floats size equal to tutal number of used subapertures"
                    raise

        elif label in ["maxAdapOffset"]:
            if type(val)==type(""):
                if os.path.exists(val):
                    val=FITS.Read(val)[1]
                else:
                    val=eval(val)
            if type(val)==numpy.ndarray:
                val=self.checkArray(val,buf.get("subapFlag").sum(),"i")
            else:
                try:
                    val=int(val)
                except:
                    print "maxAdapOffset",val
                    print "maxAdapOffset should be int or array of ints of size equal to number of valid subaps %s"%str(type(val))
                    raise
        elif label in ["bleedGain","powerFactor","adaptiveWinGain","corrThresh","figureGain"]:
            val=float(val)
        elif label in ["switchTime"]:
            val=self.checkDouble(val)
        elif label in ["fakeCCDImage"]:
            val=self.checkNoneOrArray(val,(buf.get("npxlx")*buf.get("npxly")).sum(),"f")
        elif label in ["centroidWeight"]:
            val=self.checkNoneOrFloat(val)
        elif label in ["gainE","E"]:
            val=self.checkArray(val,(buf.get("nacts"),buf.get("nacts")),"f")
        elif label in ["gainReconmxT"]:
            val=self.checkArray(val,(buf.get("subapFlag").sum()*2,buf.get("nacts")),"f")
        elif label in ["kalmanAtur","kalmanInvN"]:
            val=self.checkArray(val,(buf.get("kalmanPhaseSize"),buf.get("kalmanPhaseSize")),"f")
        elif label in ["kalmanHinfDM"]:
            val=self.checkArray(val,(buf.get("kalmanPhaseSize")*3,buf.get("kalmanPhaseSize")),"f")
        elif label in ["kalmanHinfT"]:
            val=self.checkArray(val,(buf.get("subapFlag").sum()*2,buf.get("kalmanPhaseSize")*3),"f")
        elif label in ["kalmanReset","kalmanUsed","printTime","usingDMC","go","pause","switchRequested","startCamerasFraming","stopCamerasFraming","openCameras","closeCameras","centFraming","slopeOpen"]:
            val=self.checkFlag(val)
        elif label in ["nsub","ncamThreads","npxlx","npxly"]:
            val=self.checkArray(val,buf.get("ncam"),"i")
        elif label in ["pxlCnt","subapFlag"]:
            val=self.checkArray(val,buf.get("nsub").sum(),"i")
        elif label in ["refCentroids"]:
            val=self.checkNoneOrArray(val,buf.get("subapFlag").sum()*2,"f")
        elif label in ["centCalBounds"]:
            val=self.checkNoneOrArray(val,buf.get("subapFlag").sum()*2*2,"f")
        elif label in ["centCalSteps","centCalData"]:
            ncents=buf.get("subapFlag").sum()*2
            val=self.checkNoneOrArray(val,None,"f")
            if val!=None:
                nsteps=val.size/ncents
                if val.size!=nsteps*ncents:
                    raise Exception("%s wrong shape - should be multiple of %d, is %d"%(label,ncents,val.size))
        elif label in ["subapLocation"]:
            slt=buf.get("subapLocType")
            if slt==0:
                val=self.checkArray(val,(buf.get("nsub").sum(),6),"i")
            else:
                val=self.checkArray(val,None,"i")
                n=val.size//buf.get("nsub").sum()#get the size and test again
                val=self.checkArray(val,(buf.get("nsub").sum(),n),"i")
        elif label in ["subapAllocation"]:
            val=self.checkNoneOrArray(val,buf.get("nsub").sum(),"i")
        elif label in ["v0","gain"]:
            val=self.checkArray(val,buf.get("nacts"),"f")
        elif label in ["asyncInitState","asyncScales","asyncOffsets"]:
            val=self.checkNoneOrArray(val,buf.get("nacts"),"f")
        elif label in ["asyncCombines","asyncUpdates","asyncStarts"]:
            val=self.checkNoneOrArray(val,None,"i")
        elif label in ["decayFactor"]:
            val=self.checkNoneOrArray(val,buf.get("nacts"),"f")
        elif label in ["rmx"]:
            val=self.checkArray(val,(buf.get("nacts"),buf.get("subapFlag").sum()*2),"f",raiseShape=1)
        elif label in ["ncam"]:
            val=int(val)
            if val<1:
                raise Exception("Illegal ncam")
        elif label in ["threadAffinity","threadPriority"]:
            val=self.checkNoneOrArray(val,buf.get("ncamThreads").sum()+1,"i")
        elif label in ["corrFFTPattern","corrPSF"]:
            val=self.checkNoneOrArray(val,(buf.get("npxlx")*buf.get("npxly")).sum(),"f")
        elif label in ["adaptiveGroup"]:
            val=self.checkNoneOrArray(val,(buf.get("subapFlag").sum(),),"i")
        elif label in ["asyncNames"]:
            pass#no checking needed...
        else:
            print "Unchecked parameter %s"%label
                                      
        return val
