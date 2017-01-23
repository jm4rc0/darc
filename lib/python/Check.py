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
try:#A custom checking library can be provided, not part of darc, which has a valid(label,val,buf) method that checks a parameter, and raises an exception if it is not valid.
    import CustomCheck
except:
    CustomCheck=None
    print "Unable to load CustomCheck module - continuing"

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
        if val is None:
            pass
        elif type(val)==numpy.ndarray:
            val=val.astype(dtype)
            if size is not None:
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
            if dtype is not None:
                val=val.astype(dtype)
            if shape is not None:
                if type(shape)!=type(()):
                    shape=(shape,)
                if val.shape!=shape:
                    print "Warning - shape not quite right (expecting %s, got %s)?"%(str(shape),str(val.shape))
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
        if val is None:
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
        elif label in ["cameraName","mirrorName","comment","slopeName","figureName","version","configfile"]:
            if type(val)!=type(""):
                raise Exception(label)
        elif label in ["reconName","calibrateName","bufferName"]:
            if type(val) not in [type(""),type(None)]:
                raise Exception(label)
        elif label=="centroidMode":
            if type(val)==numpy.ndarray:
                nsubaps=buf.get("nsub").sum()
                try:
                    val=self.checkArray(val,nsubaps,"i")
                except:
                    print "centroidMode array wrong"
                    traceback.print_exc()
                    raise
            elif val not in ["WPU","CoG","Gaussian","CorrelationCoG","CorrelationGaussian",0,1,2,3,4,5]:
                print "centroidMode not correct (%s)"%str(val)
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
        elif label in ["closeLoop","nacts","thresholdAlgo","delay","maxClipped","camerasFraming","camerasOpen","mirrorOpen","clearErrors","frameno","corrThreshType","nsubapsTogether","nsteps","addActuators","recordCents","averageImg","averageCent","kalmanPhaseSize","figureOpen","printUnused","reconlibOpen","currentErrors","xenicsExposure","calibrateOpen","iterSource","bufferOpen","bufferUseSeq","subapLocType","noPrePostThread","asyncReset","openLoopIfClip","threadAffElSize","mirrorStep","mirrorUpdate","mirrorReset","mirrorGetPos","mirrorDoMidRange","lqgPhaseSize","lqgActSize"]:
            val=int(val)
        elif label in ["dmDescription"]:
            if val.dtype.char!="h":
                raise Exception("dmDescription type not int16")
        elif label in ["actuators"]:
            val=self.checkNoneOrArray(val,None,"f")
            if val is not None:
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
            if val is not None:
                if val.size==buf.get("nacts"):
                    # shape okay... multiple of nacts.
                    pass
                else:
                    raise Exception("actuatorMask should be array (nacts,)")
        elif label in ["actSequence"]:
            actuators=buf.get("actuators")
            if actuators is None:
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
        elif label in ["powerFactor","adaptiveWinGain","corrThresh","figureGain","uEyeFrameRate","uEyeExpTime"]:
            val=float(val)
        elif label=="bleedGain":
            if type(val)==numpy.ndarray:
                if val.dtype.char!='f':
                    val=val.astype('f')
            else:
                val=float(val)
        elif label=="bleedGroups":
            val=self.checkNoneOrArray(val,buf.get("nacts"),"i")
        elif label in ["switchTime"]:
            val=self.checkDouble(val)
        elif label in ["fakeCCDImage"]:
            val=self.checkNoneOrArray(val,(buf.get("npxlx")*buf.get("npxly")).sum(),"f")
        elif label in ["centroidWeight"]:
            val=self.checkNoneOrFloat(val)
        elif label in ["gainE","E"]:
            if val is None:
                pass
            else:
                val=self.checkArray(val,(buf.get("nacts"),buf.get("nacts")),"f")
        elif label in ["gainReconmxT"]:
            val=self.checkArray(val,(buf.get("subapFlag").sum()*2,buf.get("nacts")),"f")
        elif label in ["kalmanAtur"]:
            val=self.checkArray(val,(buf.get("kalmanPhaseSize"),buf.get("kalmanPhaseSize")),"f")
        elif label in ["kalmanInvN"]:
            val=self.checkNoneOrArray(val,buf.get("nacts")*buf.get("kalmanPhaseSize"),"f")
            if val is not None:#now check shape
                val=self.checkArray(val,(buf.get("nacts"),buf.get("kalmanPhaseSize")),"f")
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
            if val is not None:
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
        elif label in ["gain"]:
            val=self.checkArray(val,buf.get("nacts"),"f")
        elif label in ["v0"]:
            val=self.checkNoneOrArray(val,buf.get("nacts"),"f")
        elif label in ["asyncInitState","asyncScales","asyncOffsets"]:
            val=self.checkNoneOrArray(val,buf.get("nacts"),"f")
        elif label in ["asyncCombines","asyncUpdates","asyncStarts","asyncTypes"]:
            val=self.checkNoneOrArray(val,None,"i")
        elif label in ["decayFactor"]:
            val=self.checkNoneOrArray(val,buf.get("nacts"),"f")
        elif label in ["rmx"]:
            if val is None and buf.get("reconName") not in ["libreconpcg.so","libreconneural.so","libreconLQG.so","libreconcure.so"]:
                raise Exception("rmx is None")
            elif val is not None:
                val=self.checkArray(val,(buf.get("nacts"),buf.get("subapFlag").sum()*2),"f",raiseShape=1)
        elif label in ["slopeSumMatrix"]:
            val=self.checkNoneOrArray(val,None,"f")
            if (val is not None) and val.size%buf.get("nacts")!=0:
                raise Exception("slopeSumMatrix wrong size")
        elif label in ["slopeSumGroup"]:
            val=self.checkNoneOrArray(val,buf.get("subapFlag").sum()*2,"i")
            if (val is not None) and numpy.max(val)+1!=(buf.get("slopeSumMatrix").size/buf.get("nacts")):
                raise Exception("Groupings in slopeSumGroup not consistent with size of slopeSumMatrix")
        elif label in ["ncam"]:
            val=int(val)
            if val<1:
                raise Exception("Illegal ncam")
        elif label in ["threadAffinity"]:
            if val is None:
                pass
            elif type(val)==numpy.ndarray:
                if val.dtype!="i":
                    val=val.astype("i")
                if val.size%(buf.get("ncamThreads").sum()+1)!=0:
                    raise Exception("threadAffinity error (size not multiple of %d)"%(buf.get("ncamThreads").sum()+1))
            else:
                raise Exception("threadAffinity error (should be an array, or None)")
        elif label in ["threadPriority"]:
            val=self.checkNoneOrArray(val,buf.get("ncamThreads").sum()+1,"i")
        elif label in ["corrFFTPattern","corrPSF"]:
            if type(val)==numpy.ndarray:
                val=val.astype(numpy.float32)
            elif val is not None:
                raise Exception("corrFFTPattern error")
            #val=self.checkNoneOrArray(val,(buf.get("npxlx")*buf.get("npxly")).sum(),"f")
        elif label in ["adaptiveGroup"]:
            val=self.checkNoneOrArray(val,buf.get("subapFlag").sum(),"i")
        elif label in ["asyncNames"]:
            pass#no checking needed...
        elif label in ["adapWinShiftCnt"]:
            val=self.checkNoneOrArray(val,buf.get("nsub").sum()*2,"i")
        elif label in ["centIndexArray"]:
            if type(val)==type([]):
                val=numpy.array(val)
            elif type(val)==type(""):
                if os.path.exists(val):
                    print "Loading %s"%val
                    val=FITS.Read(val)[1]
                else:
                    print "File %s not found"%val
                    raise Exception("File %s not found"%val)
            if val is None:
                pass
            elif type(val)==numpy.ndarray:
                val=val.astype("f")
                fft=buf.get("corrFFTPattern",None)
                if fft is None:
                    npxls=(buf.get("npxlx")*buf.get("npxly")).sum()
                else:
                    npxls=fft.size
                if val.size not in [npxls,npxls*2,npxls*3,npxls*4]:
                    raise Exception("centIndexArray wrong size")
            else:
                raise Exception("centIndexArray")
        elif label=="actsToSend":
            val=self.checkNoneOrArray(val,None,"i")
        elif label in ["mirrorSteps","mirrorMidRange"]:#used for mirrorLLS.c
            val=self.checkArray(val,buf.get("nacts"),"i")
        elif label in ["lqgAHwfs"]:
            val=self.checkArray(val,(buf.get("lqgPhaseSize")*2,buf.get("lqgPhaseSize")),"f",raiseShape=1)
        elif label in ["lqgAtur"]:
            val=self.checkArray(val,(buf.get("lqgPhaseSize"),buf.get("lqgPhaseSize")),"f",raiseShape=1)
        elif label in ["lqgHT"]:
            val=self.checkArray(val,(buf.get("subapFlag").sum()*2,2*buf.get("lqgPhaseSize")),"f",raiseShape=1)
        elif label in ["lqgHdm"]:
            try:
                val=self.checkArray(val,(2*buf.get("lqgPhaseSize"),buf.get("nacts")),"f",raiseShape=1)
            except:
                val=self.checkArray(val,(2,2*buf.get("lqgPhaseSize"),buf.get("nacts")),"f",raiseShape=1)
                
        elif label in ["lqgInvN"]:
            try:
                val=self.checkArray(val,(buf.get("nacts"),buf.get("lqgPhaseSize")),"f",raiseShape=1)
            except:
                val=self.checkArray(val,(2,buf.get("nacts"),buf.get("lqgPhaseSize")),"f",raiseShape=1)

        elif label in ["lqgInvNHT"]:
            val=self.checkArray(val,(buf.get("subapFlag").sum()*2,buf.get("nacts")),"f",raiseShape=1)

        elif CustomCheck is not None:
            val=CustomCheck.valid(label,val,buf)
        else:
            print "Unchecked parameter %s"%label
                                      
        return val


    def setDependencies(self,name,b):
        """Value name has just changed in the buffer,  This will require some other things updating.
        """
        paramChangedDict={}
        if name in ["bgImage","flatField","darkNoise","pxlWeight","thresholdValue","thresholdAlgo","subapLocation","subapFlag","npxlx","npxly","nsub","nsuby","calsub","calmult","calthr"]:
            #update calsub, calmult, calthr
            try:
                ff=b.get("flatField")
                bg=b.get("bgImage")
                dn=b.get("darkNoise")
                wt=b.get("pxlWeight")
                th=b.get("thresholdValue")
                ta=b.get("thresholdAlgo")
                sl=b.get("subapLocation")
                sf=b.get("subapFlag")
                st=b.get("subapLocType")
                npxlx=b.get("npxlx")
                npxly=b.get("npxly")
                #nsuby=b.get("nsuby")
                nsub=b.get("nsub")
                ncam=b.get("ncam")
            except:#buffer probably not filled yet...
                return
            if ff is not None:ff=ff.copy()
            if bg is not None:bg=bg.copy()
            if dn is not None:dn=dn.copy()
            if wt is not None:wt=wt.copy()
            if type(th)==numpy.ndarray:th=th.copy()
            npxls=(npxlx*npxly).sum()
            if ta==2:#add threshold to background then set thresh to zero
                #note this altered background is only used here for calcs.
                if type(th)==numpy.ndarray:#value per subap
                    if th.size==npxls:#value per pixel
                        if bg is None:
                            bg=th.copy()
                        else:
                            bg+=th
                    else:
                        if bg is None:
                            bg=numpy.zeros((npxls),numpy.float32)
                        nsubapsCum=0
                        npxlcum=0
                        pos=0
                        for k in range(ncam):
                            bb=bg[npxlcum:npxlcum+npxlx[k]*npxly[k]]
                            bb.shape=npxly[k],npxlx[k]
                            for i in range(nsub[k]):
                                s=sl[pos]
                                if sf[pos]!=0:#subap used
                                    if st==0:
                                        bb[s[0]:s[1]:s[2],s[3]:s[4]:s[5]]+=th[pos]
                                    else:
                                        for i in range(sl.shape[0]):
                                            if sl[i]==-1:
                                                break
                                            bb[sl[i]]=th[pos]
                                pos+=1
                            nsubapsCum+=nsub[k]
                            npxlcum+=npxly[k]*npxlx[k]
                else:
                    if (bg is None) and th!=0:
                        bg=numpy.zeros((npxls),numpy.float32)
                    if bg is not None:
                        bg[:]+=th
                        
                calthr=numpy.zeros((npxls),numpy.float32)
            elif ta==1:
                #multiply threshold by weight
                if type(th)==numpy.ndarray:
                    if th.size==npxls: #threshold per pixel
                        if wt is None:
                            calthr=th
                        else:
                            calthr=th*wt
                    else:#threshold per subap
                        calthr=numpy.zeros((npxls),numpy.float32)
                        if wt is None:
                            wtt=numpy.ones((npxls),numpy.float32)
                        else:
                            wtt=wt
                        #now multiply threshold by weight.
                        nsubapsCum=0
                        npxlcum=0
                        pos=0
                        for k in range(ncam):
                            ct=calthr[npxlcum:npxlcum+npxlx[k]*npxly[k]]
                            ct.shape=npxly[k],npxlx[k]
                            w=wtt[npxlcum:npxlcum+npxlx[k]*npxly[k]]
                            w.shape=npxly[k],npxlx[k]
                            for i in range(nsub[k]):
                                s=sl[pos]
                                if sf[pos]!=0:#subap used
                                    if st==0:
                                        ct[s[0]:s[1]:s[2],s[3]:s[4]:s[5]]=th[pos]*w[s[0]:s[1]:s[2],s[3]:s[4]:s[5]]
                                    else:
                                        for i in range(sl.shape[0]):
                                            if sl[i]==-1:
                                                break
                                            ct[sl[i]]=th[pos]*w[sl[i]]
                                pos+=1
                            nsubapsCum+=nsub[k]
                            npxlcum+=npxly[k]*npxlx[k]
                else:#single threshold value
                    if wt is None:
                        calthr=numpy.zeros((npxls),numpy.float32)
                        calthr[:]=th
                    else:
                        calthr=wt*th
            else:
                calthr=None
            if ff is None:
                if wt is None:
                    calmult=None
                else:
                    calmult=wt
            else:
                if wt is None:
                    calmult=ff
                else:
                    calmult=ff*wt
            #calsub should equal (dn*ff+bg)*wt
            if dn is None:
                if bg is None:
                    calsub=None
                else:
                    if wt is None:
                        calsub=bg
                    else:
                        calsub=bg*wt
            else:
                if ff is None:
                    calsub=dn
                else:
                    calsub=ff*dn
                if bg is not None:
                    calsub+=bg
                if wt is not None:
                    calsub*=wt
            if calsub is not None:calsub=calsub.astype(numpy.float32)
            if calmult is not None:calmult=calmult.astype(numpy.float32)
            if calthr is not None:calthr=calthr.astype(numpy.float32)
            paramChangedDict["calsub"]=(calsub,"")
            paramChangedDict["calmult"]=(calmult,"")
            paramChangedDict["calthr"]=(calthr,"")
            b.set("calsub",calsub)
            b.set("calmult",calmult)
            b.set("calthr",calthr)



        if name in ["gain","E","rmx","gainE","gainReconmxT","decayFactor"]:
            #now update the gainE and gainReconmxT.
            try:
                rmx=b.get("rmx")
                e=b.get("E")
                g=b.get("gain")
                d=b.get("decayFactor")
            except:
                return
            if rmx is not None:
                rmxt=rmx.transpose().astype(numpy.float32)
                nacts=g.shape[0]
                for i in range(nacts):
                    rmxt[:,i]*=g[i]
                #rmxt=rmxt.astype(numpy.float32)
                paramChangedDict["gainReconmxT"]=(rmxt,"")
                b.set("gainReconmxT",rmxt)
            if e is not None:
                gainE=e.copy()
                if d is None:
                    d=1-g
                    print "Computing gainE from 1-g"
                else:
                    print "Computing gainE from decayFactor"
                for i in range(nacts):
                    gainE[i]*=d[i]#1-g[i]
                gainE=gainE.astype(numpy.float32)
            else:
                gainE=None
            paramChangedDict["gainE"]=(gainE,"")
            b.set("gainE",gainE)
        return paramChangedDict

    def checkAdd(self,d,label,val,comments):
        """If label not in d, add it"""
        if not d.has_key(label):
            print "Making default value for %s equal to %s"%(label,str(val))
            d[label]=val
            comments[label]="Value guessed during initialisation"
    def inventValues(self,c,comments):
        """If the user doesn't have everything it needs, we make it up here, and warn the user
        """
        self.checkAdd(c,"ncam",1,comments)
        self.checkAdd(c,"nacts",52,comments)
        self.checkAdd(c,"nsub",[49],comments)
        #self.checkAdd(c,"nsuby",[7],comments)
        self.checkAdd(c,"npxlx",[128],comments)
        self.checkAdd(c,"npxly",[128],comments)
        self.checkAdd(c,"refCentroids",None,comments)
        self.checkAdd(c,"subapLocType",0,comments)
        nsub=numpy.array(c["nsub"])
        #nsuby=numpy.array(c["nsuby"])
        npxlx=numpy.array(c["npxlx"])
        npxly=numpy.array(c["npxly"])
        nsubaps=nsub.sum()
        if not c.has_key("subapFlag"):
            subapFlag=numpy.ones((nsubaps,),numpy.int32)
            self.checkAdd(c,"subapFlag",subapFlag,comments)
        subapFlag=c["subapFlag"]
        nsubapsCum=numpy.zeros((c["ncam"]+1,),numpy.int32)
        nsubapsUsed=numpy.zeros((c["ncam"],),numpy.int32)
        for i in range(c["ncam"]):
            nsubapsCum[i+1]=nsubapsCum[i]+nsub[i]
            nsubapsUsed=c["subapFlag"][nsubapsCum[i]:nsubapsCum[i+1]].sum()
        if not c.has_key("subapLocation"):
            if c["subapLocType"]==0:
                c["subapLocation"]=numpy.zeros((nsubaps,6),numpy.int32)
                c["subapLocation"]=self.computeFillingSubapLocation(c)
            else:#give enough pixels to entirely use the ccd.
                maxpxls=numpy.max((npxlx*npxly+nsubapsUsed-1)/nsubapsUsed)
                c["subapLocation"]=numpy.zeros((nsubaps,maxpxls))
                c["subapLocation"]=self.computeFillingSubapLocation(c)
            self.checkAdd(c,"subapLocation",c["subapLocation"],comments)
        subapLocation=c["subapLocation"]
        if not c.has_key("pxlCnt"):
            pxlcnt=numpy.zeros((nsubaps,),"i")
            # set up the pxlCnt array - number of pixels to wait until each subap is ready.  Here assume identical for each camera.
            if c["subapLocType"]==0:
                for k in range(c["ncam"]):
                    # tot=0#reset for each camera
                    for i in range(nsub[k]):
                        indx=nsubapsCum[k]+i
                        n=(subapLocation[indx,1]-1)*npxlx[k]+subapLocation[indx,4]
                        pxlcnt[indx]=n
            else:
                subapLocation.shape=nsubaps,subapLocation.size/nsubaps
                for i in range(nsub.sum()):
                    pxlcnt[i]=numpy.max(subapLocation[i])
            c["pxlCnt"]=pxlcnt

        self.checkAdd(c,"pxlCnt",c["pxlCnt"],comments)
        self.checkAdd(c,"bgImage",None,comments)
        self.checkAdd(c,"darkNoise",None,comments)
        self.checkAdd(c,"flatField",None,comments)
        self.checkAdd(c,"thresholdAlgo",0,comments)
        self.checkAdd(c,"thresholdValue",0,comments)
        self.checkAdd(c,"powerFactor",1,comments)
        self.checkAdd(c,"centroidWeight",None,comments)
        self.checkAdd(c,"windowMode","basic",comments)
        self.checkAdd(c,"go",1,comments)
        self.checkAdd(c,"centroidMode","CoG",comments)
        self.checkAdd(c,"pause",0,comments)
        self.checkAdd(c,"printTime",0,comments)
        self.checkAdd(c,"ncamThreads",[1],comments)
        self.checkAdd(c,"switchRequested",0,comments)
        self.checkAdd(c,"actuators",None,comments)
        self.checkAdd(c,"fakeCCDImage",None,comments)
        self.checkAdd(c,"threadAffinity",None,comments)
        self.checkAdd(c,"threadPriority",None,comments)
        self.checkAdd(c,"delay",0,comments)
        self.checkAdd(c,"maxClipped",c["nacts"],comments)
        self.checkAdd(c,"clearErrors",0,comments)
        self.checkAdd(c,"camerasOpen",0,comments)
        #self.checkAdd(c,"camerasFraming",0,comments)
        self.checkAdd(c,"cameraParams",[],comments)
        self.checkAdd(c,"cameraName","none",comments)
        self.checkAdd(c,"mirrorOpen",0,comments)
        self.checkAdd(c,"mirrorName","none",comments)
        self.checkAdd(c,"frameno",0,comments)
        self.checkAdd(c,"switchTime",0,comments)
        self.checkAdd(c,"adaptiveWinGain",0.,comments)
        self.checkAdd(c,"corrThreshType",0,comments)
        self.checkAdd(c,"corrThresh",0,comments)
        self.checkAdd(c,"corrFFTPattern",None,comments)
        #self.checkAdd(c,"nsubapsTogether",1,comments)
        self.checkAdd(c,"nsteps",0,comments)
        self.checkAdd(c,"closeLoop",1,comments)
        self.checkAdd(c,"mirrorParams",None,comments)
        self.checkAdd(c,"addActuators",0,comments)
        self.checkAdd(c,"actSequence",None,comments)
        self.checkAdd(c,"recordCents",0,comments)
        self.checkAdd(c,"pxlWeight",None,comments)
        self.checkAdd(c,"averageImg",0,comments)
        self.checkAdd(c,"slopeOpen",1,comments)
#        self.checkAdd(c,"slopeFraming",0,comments)
        self.checkAdd(c,"slopeParams",None,comments)
        self.checkAdd(c,"slopeName","librtcslope.so",comments)
        self.checkAdd(c,"actuatorMask",None,comments)
        self.checkAdd(c,"averageCent",0,comments)
        #self.checkAdd(c,"calmult",None)
        #self.checkAdd(c,"calsub",None)
        #self.checkAdd(c,"calthr",None)
        self.checkAdd(c,"centCalData",None,comments)
        self.checkAdd(c,"centCalBounds",None,comments)
        self.checkAdd(c,"centCalSteps",None,comments)
        self.checkAdd(c,"figureOpen",0,comments)
        self.checkAdd(c,"figureName","none",comments)
        self.checkAdd(c,"figureParams",None,comments)
        self.checkAdd(c,"reconName","libreconmvm.so",comments)
        self.checkAdd(c,"fluxThreshold",0,comments)
        self.checkAdd(c,"printUnused",1,comments)
        self.checkAdd(c,"useBrightest",0,comments)
        self.checkAdd(c,"figureGain",1.,comments)
        self.checkAdd(c,"reconlibOpen",1,comments)
        self.checkAdd(c,"maxAdapOffset",0,comments)
        #self.checkAdd(c,"lastActs",None,comments)
        self.checkAdd(c,"version"," "*120,comments)
        self.checkAdd(c,"currentErrors",0,comments)
        self.checkAdd(c,"actOffset",None,comments)
        self.checkAdd(c,"actScale",None,comments)
        self.checkAdd(c,"actsToSend",None,comments)
        self.checkAdd(c,"reconParams",None,comments)
        self.checkAdd(c,"adaptiveGroup",None,comments)
        self.checkAdd(c,"calibrateName","librtccalibrate.so",comments)
        self.checkAdd(c,"calibrateParams",None,comments)
        self.checkAdd(c,"calibrateOpen",1,comments)
        self.checkAdd(c,"iterSource",0,comments)
        self.checkAdd(c,"bufferName","librtcbuffer.so",comments)
        self.checkAdd(c,"bufferParams",None,comments)
        self.checkAdd(c,"bufferOpen",0,comments)
        self.checkAdd(c,"bufferUseSeq",0,comments)
        self.checkAdd(c,"noPrePostThread",0,comments)
        self.checkAdd(c,"subapAllocation",None,comments)
        self.checkAdd(c,"decayFactor",None,comments)
        self.checkAdd(c,"openLoopIfClip",0,comments)
        self.checkAdd(c,"adapWinShiftCnt",None,comments)
        self.checkAdd(c,"slopeSumMatrix",None,comments)
        self.checkAdd(c,"slopeSumGroup",None,comments)
        self.checkAdd(c,"centIndexArray",None,comments)
        self.checkAdd(c,"threadAffElSize",(os.sysconf("SC_NPROCESSORS_ONLN")+31)//32,comments)
        self.checkAdd(c,"adapResetCount",0,comments)
        self.checkAdd(c,"bleedGain",0.,comments)
        self.checkAdd(c,"bleedGroups",None,comments)



    def computeFillingSubapLocation(self,b=None):
        """Compute a subapLocation (and pxlcnt) such that all pixels are in a subap.
        Used during initialisation.
        """
        print "computing filling subap location"
        if b is None:
            b=self.getInactiveBuffer()
        subapLocation=b.get("subapLocation").copy()
        subapFlag=b.get("subapFlag")
        ncam=b.get("ncam")
        npxlx=b.get("npxlx")
        npxly=b.get("npxly")
        nsub=b.get("nsub")
        subapLocationType=b.get("subapLocType")
        #nsuby=b.get("nsuby")
        subcum=0
        if subapLocationType==0:
            for i in range(ncam):
                sf=subapFlag[subcum:subcum+nsub[i]]
                sl=subapLocation[subcum:subcum+nsub[i]]
                #sf.shape=nsuby[i],nsubx[i]
                sl.shape=nsub[i],6
                #Compute the number of rows and columns that have a value in them.
                #ncols=(sf.sum(0)>0).sum()
                sfsum=sf.sum()
                nrows=int(numpy.sqrt(sfsum))
                ndone=0
                pos=0
                for j in range(nrows):
                    ncols=int((sfsum-ndone)/(nrows-j))
                    pxldone=0
                    for k in range(ncols):
                        npxls=(npxlx[i]-pxldone)/(ncols-k)
                        if sf[pos]:
                            sl[pos]=[ndone,ncols,1,pxldone,npxls,1]
                        pos+=1
                        pxldone+=npxls
                    ndone+=ncols
                subcum+=nsub[i]#*nsuby[i]
        else:
            subapLocation[:]=-1
            for i in range(ncam):
                sf=subapFlag[subcum:subcum+nsub[i]]
                sl=subapLocation[subcum:subcum+nsub[i]]
                sl.shape=nsub[i],sl.size/nsub[i]
                n=sl.shape[1]
                nused=sf.sum()
                pxl=0
                for j in range(nsub[i]):
                    k=0
                    while k<n and pxl<npxlx[i]*npxly[i]:
                        sl[j,k]=pxl
                        pxl+=1
                        k+=1
        return subapLocation
