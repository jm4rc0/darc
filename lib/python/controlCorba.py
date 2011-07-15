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
#!/usr/bin/env python
CVSID="$Id$"
import sys
from omniORB import CORBA, PortableServer
import CosNaming
#import RTC, RTC__POA
import numpy
import string
import time
import control_idl
import threading
import os
import select
import traceback
import subprocess
import socket
import thread
import recvStream
import Saver
import FITS
import buffer
#import startStreams
class dummyControl:
    def __init__(self,a=None,b=None,c=None,d=None):
        print a,b,c,d
        self.errList=[]

    def __call__(self,a=None,b=None,c=None,d=None):
        print a,b,c,d
        self.errList=[]
    def getStreams(self):
        return ["calPxl","cents"]

# Define an implementation of the Control interface
class Control_i (control_idl._0_RTC__POA.Control):
    def __init__(self,c=None,l=None):
        """c is the instance of the control object
        l is a lock that should be obtained before calling an operation.
        """
        if l==None:
            l=threading.Lock()
        self.l=l
        if c==None:
            c=dummyControl()
        self.c=c
        self.endPipe=os.pipe()
    def initialise(self,c,l):
        self.c=c
        self.l=l
        self.c.sockConn.selIn.append(self.endPipe[0])
    def echoString(self, mesg):
        #self.l.acquire()
        #print "echoString() called with message:", mesg
        #self.l.release()
        return mesg
    def AverageImage(self,nframes,whole):
        """Acquire a calibrated image averaged over nframes.  Return the image to the user.

        Note, and pixel calibration specified in the GUI will be applied here.
        If whole is set, it will compute subapLocation such that the whole image is calibrated.  Note - this will mess up centroid measurements, so the loop is opened.  At the end, things are placed back into their existing state.

        """
        self.l.acquire()
        try:
            rt=self.c.acquireImage(nframes,whole)
            rt=control_idl._0_RTC.Control.FDATA(rt.size,rt.tostring())
        except:
            self.l.release()
            raise
        self.l.release()
        return rt
    def AverageCentroids(self,nframes):
        """Acquire averaged centroids over nframes, return image to user.
        """
        self.l.acquire()
        try:
            rt=self.c.acquireCents(nframes)
            rt=control_idl._0_RTC.Control.FDATA(rt.size,rt.tostring())
        except:
            self.l.release()
            raise
        
        self.l.release()
        return rt

    def WFacqBckgrd(self):
        """Acquire a background image, and set it as such in the RTC.  Return the image to the user.
        Note, the dark noise and flatfield have already been applied to this image.
        """
        self.l.acquire()
        try:
            rt=self.c.acquireBackground()
            rt=control_idl._0_RTC.Control.FDATA(rt.size,rt.tostring())
        except:
            self.l.release()
            raise

        self.l.release()
        return rt
    # def WFsetRefSlope(self,fdata):
    #     self.l.acquire()
    #     try:
    #         data=numpy.fromstring(fdata.data,numpy.float32)
    #         self.c.set("refCentroids",data,comment="set by corba %s"%time.strftime("%y/%m/%d %H:%M:%S"),copyFirst=1,update=1)
    #     except:
    #         self.l.release()
    #         raise

    #     self.l.release()
    #     return 0
    # def WFsetBckgrd(self,fdata):
    #     self.l.acquire()
    #     try:
    #         data=numpy.fromstring(fdata.data,numpy.float32)
    #         self.c.setBackground(data)
    #     except:
    #         self.l.release()
    #         raise

    #     self.l.release()
    #     return 0
    # def WFsetThreshold(self, thresh):
    #     self.l.acquire()
    #     try:
    #         self.c.setThreshold(thresh)
    #     except:
    #         self.l.release()
    #         raise

    #     self.l.release()
    #     return 0
    # def WFsetGain(self,fdata):#flat field...
    #     self.l.acquire()
    #     try:
    #         data=numpy.fromstring(fdata.data,numpy.float32)
    #         self.c.setFlatfield(data)
    #     except:
    #         self.l.release()
    #         raise

    #     self.l.release()
    #     return 0
    # def CsetGain(self,fdata):
    #     self.l.acquire()
    #     try:
    #         gain=numpy.fromstring(fdata.data,numpy.float32)
    #         self.c.setGain(gain)
    #     except:
    #         self.l.release()
    #         raise

    #     self.l.release()
    #     return 0
    # def CsetMirrorDAC(self,vmin,vmax):
    #     self.l.acquire()
    #     try:
    #         self.c.setActBounds(vmin,vmax)
    #     except:
    #         self.l.release()
    #         raise

    #     self.l.release()
    #     return 0
    # def CsetMirror(self,fdata):#n==no of actuators
    #     self.l.acquire()
    #     #set all actuators
    #     try:
    #         data=numpy.fromstring(fdata.data,numpy.float32)
    #         if data.shape[0]==0:
    #             data=None
    #         self.c.setActuators(data)
    #     except:
    #         self.l.release()
    #         raise

    #     self.l.release()
    #     return 0
    # def CsetActuator(self,mode,act,v):
    #     self.l.acquire()
    #     try:
    #         self.c.setActuator(mode,act,v)
    #     except:
    #         self.l.release()
    #         raise

    #     self.l.release()
    #     return 0
    # def SetKalman(self,atur,hinft,invn,hinfdm):
    #     self.l.acquire()
    #     try:
    #         atur=convert(atur)
    #         hinft=convert(hinft)
    #         invn=convert(invn)
    #         hinfdm=convert(hinfdm)
    #         self.c.setKalman(atur,hinft,invn,hinfdm)
    #     except:
    #         self.l.release()
    #         raise

    #     self.l.release()
    #     return 0

    # def CcloseLoop(self,mode,rmxfdata, vmaxDM,vmaxTT,n0, sensib, tau, modeVlimit, maxSubApOff, saturationIntensity):
    #     self.l.acquire()
    #     #continue update of mirror actuators
    #     #Here, mode is ignored.
    #     #So is vmax
    #     #Infact, everyhting is ignored except rmx
    #     try:
    #         rmx=numpy.fromstring(rmxfdata,numpy.float32)
    #         self.c.closeLoop(rmx)
    #     except:
    #         self.l.release()
    #         raise
       
    #     self.l.release()
    #     return 0
    # def CopenLoop(self,val):
    #     self.l.acquire()
    #     try:
    #         self.c.openLoop(val)
    #     except:
    #         self.l.release()
    #         raise
    #     self.l.release()
    #     return 0
    def CdoInteractM(self,timeDelayMirror,vMirror,frameNoMirror, cycleNoMirror, timeDelayTT, vTT, frameNoTT, cycleNoTT, abMotPt, abMotPup):
        """Compute a poke matrix.  Actually, this just creates the matrix to do the matrix."""
        self.l.acquire()
        vMirror=decode(vMirror)
        #First, work out what values need to go into the matrix...
#        nSkip=timeDelayMirror/2#assume 500Hz = 2ms per iter, so no of iters is timeDelayMirror/2
        try:
            nSkip=timeDelayMirror
            #offset=self.c.getActiveBuffer().get("midRangeValue")#32768
            offset=self.c.getActiveBuffer().get("v0")
            arr,steps=self.c.preparePokeMatrix(nSkip,frameNoMirror,offset,vMirror)
            #and now make it.
            pmx=self.c.makePokeMatrix(arr,steps)#probably shape=4*nacts,ncents
            pmx=pmx[1::2]#ignore the bits where the mirror was getting into shape.
            #now take the differential
            if type(vMirror)==type(numpy.ndarray):
                vMirror.shape=vMirror.size/nacts,nacts
                vdiff=vMirror[0]+vMirror[1]
            else:
                vdiff=2.*vMirror
            pmx=((pmx[::2]-pmx[1::2]).T/vdiff).T
            #print pmx
            fdata=control_idl._0_RTC.Control.FDATA(pmx.size,pmx.tostring())
        except:
            self.l.release()
            raise
            
        self.l.release()
        return fdata
    def CmakeInteractM(self,data,steps,lock=1):
        if lock:
            self.l.acquire()
        try:
            retcorba=0
            if type(data)!=numpy.ndarray:
                retcorba=1
                data=numpy.fromstring(data.data,numpy.float)
            if type(steps)!=numpy.ndarray:
                retcorba=1
                steps=numpy.fromstring(steps.data,numpy.int32)
            print "acts:",data,"seq:",steps
            pmx=self.c.makePokeMatrix(data,steps)
            if retcorba:#make it into a CORBA object.
                pmx=control_idl._0_RTC.Control.FDATA(pmx.size,pmx.tostring())
        except:
            if lock:
                self.l.release()
            raise
        if lock:
            self.l.release()
        return pmx

    # def CsetCommandM(self,fdata,fdata_dm,fdata_tX, fdata_tY):#n== no of centroids * no of actuators, dm, tX, tY are size [6]
    #     self.l.acquire()
    #     try:
    #         rmx=convert(fdata)
    #         self.c.setRMX(rmx)
    #     except:
    #         self.l.release()
    #         raise
    #     self.l.release()
    #     return 0
    # def CcalcOffset(self,modeLoop,setMode, saveMode, frameno, modeCtrl, vmaxMirror, vmaxTT, n0, sensib, tau, modeVlimit, maxSubApOff, saturationIntensity, abMotPt, abPupPt):
    #     self.l.acquire()
    #     print "Not yet implemented"
    #     self.l.release()
    #     return 0
    # def CresetLoop(self):
    #     self.l.acquire()
    #     try:
    #         self.c.resetLoop()
    #     except:
    #         self.l.release()
    #         raise
    #     self.l.release()
    #     return 0
    def RTCinit(self,fname):
        self.l.acquire()
        try:
            fname=decode(fname)#can be a filename (.py or .fits) or an array.
            rt=0
            try:
                self.c.RTCInit(fname)
            except Exception,ex:
                rt=-1
                print ex
                print sys.exc_info()
                traceback.print_exc()
                print "Error in RTCinit"
        except:
            self.l.release()
            raise
        self.l.release()
        if rt:
            raise Exception("Error in RTCinit")
        return rt
    # def RTCcanaryInit(self,config):#config is the parameter filename
    #     self.l.acquire()
    #     print config
    #     self.l.release()
    #     return 0
    def ControlHalt(self,stopRTC):
        """Halt RTC and control object"""
        self.l.acquire()
        try:
            print "Halting..."
            rt=0
            try:
                self.c.stop(stopRTC)
            except:
                rt=1
                traceback.print_exc()
            os.write(self.endPipe[1],"E")
        except:
            self.l.release()
            raise
        self.l.release()
        return rt
    def RTChalt(self):
        """Halt just RTC"""
        self.l.acquire()
        try:
            print "Halting..."
            stopControl=0#self.c.rtcStopped
            rt=0
            try:
                self.c.stop(stopControl=stopControl)
            except:
                rt=1
            if stopControl:
                os.write(self.endPipe[1],"E")
        except:
            self.l.release()
            raise
            
        self.l.release()
        return rt
    def SetRTCDecimation(self,key,val):
        self.l.acquire()
        try:
            self.c.setRTCDecimation(key,val)
        except:
            self.l.release()
            raise
        self.l.release()
        return 0
    def RemoveError(self,err):
        self.l.acquire()
        try:
            self.c.removeError(err)#If err=="", removes all of them.
        except:
            self.l.release()
            raise
        self.l.release()
        return 0
    def GetInactiveBuffer(self):
        self.l.acquire()
        try:
            buf=self.c.getInactiveBuffer()
            if buf!=None:
                buf=buf.arr.view('b')[:buf.getMem(1)].tostring()
            else:
                buf=""
            rt=control_idl._0_RTC.Control.BDATA(len(buf),buf)
        except:
            self.l.release()
            raise
        self.l.release()
        return rt
    def Set(self,names,vals,comments,doSwitch,check,copy):
        self.l.acquire()
        #b=self.c.getInactiveBuffer()
        try:
            errList=[]
            #print names
            for i in range(names.n):
                name=names.data[i]
                #print name,vals,i
                val=decode(vals.data[i])
                if len(comments.data)>i:
                    comment=comments.data[i]
                else:
                    comment=""

                try:
                    self.c.set(name,val,comment=comment,check=check)
                except:
                    errList.append(name)
            if doSwitch:
                self.c.setSwitchRequested(wait=1)
                if copy:
                    self.c.copyToInactive()
            rt=control_idl._0_RTC.Control.SDATA(len(errList),errList)
        except:
            self.l.release()
            raise
        self.l.release()
        return rt
    def RequestParamSwitch(self,wait):
        self.l.acquire()
        try:
            self.c.setSwitchRequested(wait=wait)
        except:
            self.l.release()
            raise
        self.l.release()
        return 0
    def CopyToInactive(self):
        self.l.acquire()
        try:
            self.c.copyToInactive()
        except:
            self.l.release()
            raise
        self.l.release()
        return 0
    def GetActiveBufferArray(self):
        self.l.acquire()
        try:
            buf=self.c.getActiveBufferArray()
            if buf!=None:
                buf=buf.tostring()
            else:
                buf=""
            rt=control_idl._0_RTC.Control.BDATA(len(buf),buf)
        except:
            self.l.release()
            traceback.print_exc()
            raise
        self.l.release()
        return rt
    def TogglePause(self,p):
        self.l.acquire()
        try:
            p=self.c.togglePause(p)
        except:
            self.l.release()
            raise
        self.l.release()
        return p
    def GetStreams(self):
        self.l.acquire()
        try:
            streams=self.c.getStreams()
            rt=control_idl._0_RTC.Control.SDATA(len(streams),streams)
        except:
            self.l.release()
            raise
        self.l.release()
        return rt
    # def Subscribe(self,stream,decimate):
    #     self.l.acquire()
    #     #probably too complicated to do is here - so really, clients should subscribe via the dataswitch.
    #     self.l.release()
    #     return 0
    def ReleaseLock(self):
        """Can be used in emergencies"""
        self.l.release()
        return 0

    def Execute(self,cmd,vals):
        """cmd is a string, vals is type SVDATA"""
        dataList=[]
        for val in vals.data:
            dataList.append(decode(val))
        d={}
        g=globals()
        g["dataList"]=dataList
        g["self"]=self
        exec cmd in g,d
        data=None
        if d.has_key("data"):
            data=d["data"]
        if type(data)!=type([]):
            data=[data]
        rt=encode(data)
        print rt
        return rt
    def GetErrors(self):
        """Retrieve the error list..."""
        data=control_idl._0_RTC.Control.SDATA(len(self.c.errList),self.c.errList)
        return data

    def PublishParams(self):
        """Publish the parameter buffer"""
        self.c.publishParams()
        return 0

    def GetControlHost(self):
        """Can be used if dataSwitch is not being used... - tells client where to connect..."""
        txt="%s %d"%(self.c.host,self.c.port)
        return txt

    def GetActuators(self,retcorba=1):
        self.l.acquire()
        try:
            acts=self.c.getActuators()
            if retcorba:#make it into a CORBA object.
                acts=control_idl._0_RTC.Control.UHDATA(acts.size,acts.tostring())
        except:
            self.l.release()
            raise
        self.l.release()
        return acts

    def Get(self,name):
        self.l.acquire()
        try:
            val=self.c.getActiveBuffer().get(name)
            val=encode(val)
        except:
            self.l.release()
            raise
        self.l.release()
        return val

    def GetComment(self,name):
        self.l.acquire()
        try:
            com=self.c.getActiveBuffer().getComment(name)
        except:
            self.l.release()
            raise
        self.l.release()
        return com

    def SetCloseLoop(self,p):
        self.l.acquire()
        try:
            p=self.c.setCloseLoop(p)
        except:
            self.l.release()
            raise
        self.l.release()
        return p

    def GetStatus(self):
        self.l.acquire()
        try:
            s=self.c.getStatus()
        except:
            self.l.release()
            raise
        self.l.release()
        if s==None:
            s="Unable to get status"
        return s
    def GetStream(self,name,latest,wholeBuffer):
        self.l.acquire()
        try:
            arr=self.c.getStream(name,latest,wholeBuffer=wholeBuffer)#arr==data,time,fno
        except:
            self.l.release()
            raise
        self.l.release()
        if type(arr)!=type(None):
            arr=list(arr)
        arr=encode(arr)
        return arr
    def GetVersion(self):
        self.l.acquire()
        try:
            v=self.c.getVersion()
        except:
            self.l.release()
            raise
        self.l.release()
        return v+"\nremote controlCorba.py version:"+CVSID
    def SetDecimation(self,name,d1,d2,log,fname):
        self.l.acquire()
        try:
            self.c.setDecimation(name,d1,d2,log,fname)
        except:
            self.l.release()
            raise
        self.l.release()
        return 0
    def Remove(self,name,returnval,doSwitch):#remove a value
        self.l.acquire()
        try:
            rt=self.c.remove(name,doSwitch=doSwitch)
        except:
            self.l.release()
            raise
        self.l.release()
        if returnval:
            rt=encode(rt)
        else:
            rt=encode(None)
        return rt
    def GetDecimation(self):#get the decimation values
        self.l.acquire()
        rt=[]
        try:
            d=self.c.getRTCDecimation()
            for k in d.keys():
                rt.append(k)
                rt.append(d[k])
        except:
            self.l.release()
            raise
        self.l.release()
        rt=encode(rt)
        return rt
    def GetNControlThreads(self):
        l=threading.enumerate()
        n=len(l)
        print n
        try:
            print l
        except:
            print "Error printing threads... (caught)"
        return n


    def ComputeIP(self,hostlist):
        """Match my IP with the IPs in the list, to get the closest match"""
        if len(hostlist)==1:# and hostlist[0]!="127.0.0.1":
            return hostlist[0]#use this address
        host=None
        myIPs=[x[1] for x in getNetworkInterfaces()]#If this fails, you may be on a mac?  If so, you need to define your host in whatever calls this method.
        # Compare myIPs with hostlist to see whether we are on the same network.  If not, then try sending to 1 of them.
        best=0
        besthost=None
        for hh in hostlist:
            h=hh.split(".")
            if hh!="127.0.0.1":
                for me in myIPs:
                    me=me.split(".")
                    match=0
                    for i in range(4):
                        if int(me[i])==int(h[i]):
                            match+=1
                        else:
                            break#not same network
                    if match>best:
                        best=match
                        besthost=hh

        if best==4:#ip address matches, so localhost...
            host="127.0.0.1"
        elif best>0:
            host=besthost
        if host==None:
            print "Could not work out which host to connect to from %s where my interfaces are %s"%(str(hostlist),str(myIPs))
        return host

    def StartStream(self, names,host,port,decimate,sendFromHead,header,reset,readFrom,readTo,readStep):
        """decimate can be -1 or 0 which means don't change...
        names should include shmPrefix, if any.
        host can be an ip address, or many ip addresses comma separated.
        If many, compare with our IP addresses to work out which is best to 
        send to.
        If headedr=="no", no header sent, if "name", name is send, otherwise, a full serialised header is sent.
        """
        self.l.acquire()
        try:
            #First try to work out where to send the data.
            hostlist=host.split(",")
            host=self.ComputeIP(hostlist)
                # myIPs=[x[1] for x in getNetworkInterfaces()]#If this fails, you may be on a mac?  If so, you need to define your host in whatever calls this method.
                # # Compare myIPs with hostlist to see whether we are on the same network.  If not, then try sending to 1 of them.
                # best=0
                # besthost=None
                # for hh in hostlist:
                #     h=hh.split(".")
                #     if hh!="127.0.0.1":
                #         for me in myIPs:
                #             me=me.split(".")
                #             match=0
                #             for i in range(4):
                #                 if int(me[i])==int(h[i]):
                #                     match+=1
                #                 else:
                #                     break#not same network
                #             if match>best:
                #                 best=match
                #                 besthost=hh
                
                # if best==4:#ip address matches, so localhost...
                #     host="127.0.0.1"
                # elif best>0:
                #     host=besthost
            decorig={}
            for i in range(names.n):
                name=names.data[i]
                decorig[name]=self.c.getRTCDecimation(name)[name]
            plist=[]
            dec=decimate
            if dec<=0:
                dec=1
            for i in range(names.n):
                name=names.data[i]
                #process="sendStream.py"
                process="sender"
                arglist=["-p%d"%port,"-h%s"%host,"-t1","-i1","-r","-n","-d%d"%dec,name]
                if sendFromHead:
                    arglist.append("-f")
                if header=="no":
                    arglist.append("-R")
                elif header=="name":
                    arglist.append("-R1")
                else:#send a serialised header
                    pass
                if readFrom>0:
                    arglist.append("-F%d"%readFrom)
                if readTo>0:
                    arglist.append("-T%d"%readTo)
                if readStep>1:
                    arglist.append("-S%d"%readStep)
                #print [process]+arglist
                #No need to specify -sPREFIX since name already includes this
                try:
                    p=subprocess.Popen([process]+arglist,stdout=sys.stdout)
                except:
                    if os.path.exists("./"+process):
                        print "Warning - %s not found on path - trying ./%s"%(process,process)
                        p=subprocess.Popen(["./"+process]+arglist,stdout=sys.stdout)
                    
                plist.append(p)
            #time.sleep(3)#give enough time for the process to start and connect...
            if decimate>0:#now start it going.
                #allow time for subprocesses to start...
                for i in range(names.n):
                    name=names.data[i]
                    if decorig[name]==0:
                        self.c.setDecimation(name,decimate)
                    elif decorig[name]>decimate:
                        self.c.setDecimation(name,hcf(decorig[name],decimate))
                    elif decimate%decorig[name]!=0:
                        self.c.setDecimation(name,hcf(decorig[name],decimate))
            else:#if there are streams not switched on, turn them on...
                for k in decorig.keys():
                    if decorig[k]==0:
                        self.c.setDecimation(k,1)
            #now, start a thread to join on plist, and reset decimates...
            if reset:
                thread.start_new_thread(self.resetDecimates,(plist,decorig))
        except:
            self.l.release()
            raise
        self.l.release()
        return 0
    
    def GetLabels(self):
        self.l.acquire()
        try:
            data=sdata(self.c.getLabels())
        except:
            self.l.release()
            raise
        self.l.release()
        return data
    
    def WaitParamChange(self,timeout):
        #print "calling waitParamChange"
        self.c.waitParamChange(timeout)
        #print "waitParamChange returned"
        return 0

    def resetDecimates(self,plist=None,decorig=None):
        if plist!=None:
            for p in plist:
                p.wait()
        if decorig!=None:
            print "Resetting decimates to",decorig
            for name in decorig.keys():
                if type(decorig[name])==type({}):
                    self.c.setDecimation(name,decorig[name][name])
                else:
                    self.c.setDecimation(name,decorig[name])

    def GetLog(self):
        self.l.acquire()
        try:
            data=self.c.getLog()
        except:
            self.l.release()
            raise
        self.l.release()
        return data
    def GetLogFiles(self):
        self.l.acquire()
        try:
            data=self.c.getLogfiles()
        except:
            self.l.release()
            raise
        self.l.release()
        return string.join(data,",")
    def StartLogStream(self,hostlist,port,name,limit,sleeptime):
        self.l.acquire()
        try:
            host=self.ComputeIP(hostlist.split(","))
            arglist=[name,"%d"%limit,"%g"%sleeptime,host,"%d"%port]
            process="logread.py"
            try:
                p=subprocess.Popen([process]+arglist)
            except:
                if os.path.exists("./"+process):
                    print "Warning %s not found on path - using ./%s"%(process,process)
                    p=subprocess.Popen(["./"+process]+arglist)
                else:
                    raise
        except:
            self.l.release()
            raise
        self.l.release()
        return 0
        
    def CalibrateWholeImage(self,copy):
        """Rearranges subaps so that whole image is read out"""
        self.l.acquire()
        try:
            self.c.copyToInactive()
            data=self.c.computeFillingSubapLocation(updateRTC=1)
            self.c.togglePause(0,wait=1)#unpause, and wait for the buffer to swap.
            if copy:
                self.c.copyToInactive()
        except:
            self.l.release()
            raise
        self.l.release()
        return encode(data)
    def RestorePartialImageCalibration(self):
        """Restores subaps to what they should be."""
        self.l.acquire()
        try:
            self.c.revertSavedState()
        except:
            self.l.release()
            raise
        self.l.release()
        return 0
    def Transfer(self,data,fname):
        data=decode(data)
        try:
            open(fname,"w").write(data)
        except:
            print "Error writing file %s"%fname
            raise
        return 0
    def Swap(self,n1,n2):
        self.l.acquire()
        try:
            self.c.swap(n1,n2)
        except:
            self.l.release()
            raise
        self.l.release()
        return 0

    def WakeLogs(self,flag):
        self.l.acquire()
        try:
            self.c.wakeLogs(flag)
        except:
            self.l.release()
            raise
        self.l.release()
        return 0

    def ConnectParamSubscriber(self,hostlist,port,names):
        self.l.acquire()
        host=self.ComputeIP(hostlist.split(","))
        try:
            self.c.connectParamSubscriber(host,port,decode(names))
        except:
            self.l.release()
            raise
        self.l.release()
        return 0

    def StartSummer(self,stream,nsum,decimation,affin,prio,fromHead,startWithLatest,rolling,dtype,outputname,nstore):
        self.l.acquire()
        if outputname=="":
            outputname=None
        try:
            name=self.c.startSummer(stream,nsum,decimation,affin,prio,fromHead,startWithLatest,rolling,dtype,outputname,nstore)
        except:
            self.l.release()
            raise
        self.l.release()
        return name

    def StopSummer(self,name):
        self.l.acquire()
        try:
            self.c.stopSummer(name)
        except:
            self.l.release()
            raise
        self.l.release()
        return 0
    def GetSummerList(self):
        self.l.acquire()
        try:
            lst=self.c.getSummerList()
            rt=sdata(lst)
        except:
            self.l.release()
            raise
        self.l.release()
        return rt



    def StartSplitter(self,stream,readfrom,readto,readstep,affin,prio,fromHead,outputname,nstore):
        self.l.acquire()
        if outputname=="":
            outputname=None
        try:
            name=self.c.startSplitter(stream,readfrom,readto,readstep,affin,prio,fromHead,outputname,nstore)
        except:
            self.l.release()
            raise
        self.l.release()
        return name

    def StopSplitter(self,name):
        self.l.acquire()
        try:
            self.c.stopSplitter(name)
        except:
            self.l.release()
            raise
        self.l.release()
        return 0
    def GetSplitterList(self):
        self.l.acquire()
        try:
            lst=self.c.getSplitterList()
            rt=sdata(lst)
        except:
            self.l.release()
            raise
        self.l.release()
        return rt




    def SumData(self,stream,nsum,dtype):
        self.l.acquire()
        try:
            arr=self.c.sumData(stream,nsum,dtype)#arr==data,time,fno
        except:
            self.l.release()
            raise
        self.l.release()
        if type(arr)!=type(None):
            arr=list(arr)
        if arr==None:
            print "Error in SumData %s - returned None"%str(stream)
            arr=[arr]
        arr=encode(arr)

        return arr

    def WatchParam(self,tag,paramList,timeout):
        paramList=decode(paramList)
        if tag==0:#first time - create a new tag
            self.l.acquire()
            tag=self.c.newParamTag()
            self.l.release()
        print "Tag %d Watching %s"%(tag,str(paramList))
        changed=self.c.watchParam(tag,paramList,timeout)#this is blocking
        rt=encode([tag]+changed)
        return rt

def convert(data):
    """Convert an array into the CORBA type (FDATA, HDATA, IDATA or BDATA) or the reverse.
    """
    if type(data)==numpy.ndarray:
        if data.dtype.char=="f":
            data=control_idl._0_RTC.Control.FDATA(data.size,data.tostring())
        elif data.dtype.char=="d":
            data=control_idl._0_RTC.Control.DDATA(data.size,data.tostring())
        elif data.dtype.char=="h":
            data=control_idl._0_RTC.Control.HDATA(data.size,data.tostring())
        elif data.dtype.char=="H":
            data=control_idl._0_RTC.Control.UHDATA(data.size,data.tostring())
        elif data.itemsize==4 and data.dtype.char in ["i","l"]:
            data=control_idl._0_RTC.Control.IDATA(data.size,data.astype(numpy.int32).tostring())
        elif data.dtype.char=="b":
            data=control_idl._0_RTC.Control.BDATA(data.size,data.tostring())
        else:
            raise Exception("convert type %s not yet implemented"%data.dtype.char)
    elif isinstance(data,control_idl._0_RTC.Control.FDATA):
        data=numpy.fromstring(data.data,numpy.float32)
    elif isinstance(data,control_idl._0_RTC.Control.DDATA):
        data=numpy.fromstring(data.data,numpy.float64)
    elif isinstance(data,control_idl._0_RTC.Control.BDATA):
        data=numpy.fromstring(data.data,numpy.int8)
    elif isinstance(data,control_idl._0_RTC.Control.HDATA):
        data=numpy.fromstring(data.data,numpy.int16)
    elif isinstance(data,control_idl._0_RTC.Control.UHDATA):
        data=numpy.fromstring(data.data,numpy.uint16)
    elif isinstance(data,control_idl._0_RTC.Control.IDATA):
        data=numpy.fromstring(data.data,numpy.int32)

    return data
def encode(val):
    """Convert into a CORBA SERVAL type"""
    rt=None
    dtype="n"
    nd=0
    dims=[]
    size=0
    data=""
    if val==None:
        pass
    elif type(val) in [numpy.ndarray,numpy.memmap]:
        if val.dtype.char=="d":
            val=val.astype("f")
        if val.dtype.char in ["f","b","B","h","i","H","d"] or (val.dtype.char=="l" and val.itemsize==4):
            dtype=val.dtype.char
            if dtype=="l" and val.itemsize==4:
                dtype="i"
            nd=len(val.shape)
            dims=val.shape
            size=val.size*val.itemsize
            data=val.tostring()
        else:
            raise Exception("numpy type %s not convertable"%val.dtype.char)
    elif type(val) in [type(0),numpy.int32]:
        dtype="i"
        size=4
        data=numpy.array(val).astype(numpy.int32).tostring()
    elif type(val) in [type(0.),numpy.float32]:
        dtype="f"
        size=4
        data=numpy.array(val).astype(numpy.float32).tostring()
    elif type(val) in [numpy.float64]:
        dtype="d"
        size=8
        data=numpy.array(val).astype(numpy.float64).tostring()
    elif type(val)==type(""):
        dtype="s"
        size=len(val)
        data=val
    elif type(val)==type([]):
        vlist=[]
        for v in val:
            vlist.append(encode(v))
        rt=control_idl._0_RTC.Control.SVDATA(len(vlist),vlist)    
    elif type(val) in [type(numpy.arange(1).view("i")[0])]:
        #numpy bugfix on 32 bit platforms
        dtype="i"
        size=4
        data=numpy.array(val).astype(numpy.int32).tostring()
    else:
        raise Exception("dtype %s not yet convertable"%str(type(val)))
    if rt==None:
        rt=control_idl._0_RTC.Control.SERVAL(dtype,nd,dims,size,data)
    return rt


def decode(val):
    """Convert from CORAB SERVAL type or SVDATA to python"""
    if isinstance(val,control_idl._0_RTC.Control.SVDATA):
        rt=[]
        for v in val.data:
            rt.append(decode(v))
    elif isinstance(val,control_idl._0_RTC.Control.FDATA):
        rt=numpy.fromstring(val.data,numpy.float32)
    elif isinstance(val,control_idl._0_RTC.Control.DDATA):
        rt=numpy.fromstring(val.data,numpy.float64)
    elif isinstance(val,control_idl._0_RTC.Control.BDATA):
        rt=numpy.fromstring(val.data,numpy.int8)
    elif isinstance(val,control_idl._0_RTC.Control.HDATA):
        rt=numpy.fromstring(val.data,numpy.int16)
    elif isinstance(val,control_idl._0_RTC.Control.UHDATA):
        rt=numpy.fromstring(val.data,numpy.uint16)
    elif isinstance(val,control_idl._0_RTC.Control.IDATA):
        rt=numpy.fromstring(val.data,numpy.int32)
    elif isinstance(val,control_idl._0_RTC.Control.SDATA):
        rt=val.data
    elif val.dtype=="n":
        rt=None
    elif val.dtype in ["f","b","B","h","i","H","d"]:
        rt=numpy.fromstring(val.data,val.dtype)
        #print rt.size,rt.dtype.char,len(val.data)
        if val.nd==0:
            if val.dtype in ["f","d"]:
                rt=float(rt[0])
            else:
                rt=int(rt[0])
        else:
            rt.shape=val.dims[:val.nd]
    elif val.dtype=="s":
        rt=val.data
    else:
        raise Exception("dtype %s not yet implemented"%val.dtype)
    return rt

def sdata(val):
    if type(val)!=type([]):
        if type(val)==type(""):
            val=[val]
        else:
            raise Exception("controlCorba.sdata() must take string or list of")
    rt=control_idl._0_RTC.Control.SDATA(len(val),val)
    return rt

def hcf(no1,no2):  
    while no1!=no2:  
        if no1>no2:  
            no1-=no2  
        else:
            no2-=no1  
    return no1  
class controlClient:
    """Used eg by the GUI"""
    def __init__(self,controlName="",debug=0,orb=None):
        if "Control" not in controlName:
            self.prefix=controlName
            controlName=controlName+"Control"
        else:
            #depreciated.
            self.prefix=controlName[:-7]
        self.debug=debug
        if orb==None:
            orb=CORBA.ORB_init(sys.argv,CORBA.ORB_ID)
        self.orb=orb
        self.obj=None
        self.connectControl(controlName)
    def connectControl(self,controlName="Control"):
        if self.debug:
            print "Attemptint to connect to rtc Control Corba"
        self.obj=None
        # Initialise the ORB
        #orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)
        orb=self.orb
        # Obtain a reference to the root naming context
        obj = orb.resolve_initial_references("NameService")
        try:
            rootContext = obj._narrow(CosNaming.NamingContext)
        except:
            print "Unable to connect to nameservice"
            return False
        if rootContext is None:
            print "Failed to narrow the root naming context"
        # Resolve the name "rtcServer.my_context/Control.Object"
        name = [CosNaming.NameComponent("rtcServer", "my_context"),
                CosNaming.NameComponent(controlName, "Object")]
        try:
            obj = rootContext.resolve(name)
        except CosNaming.NamingContext.NotFound, ex:
            print "Name not found"
        else:
            # Narrow the object to an RTC::Control
            self.obj = obj._narrow(control_idl._0_RTC.Control)
        if self.obj is None:
            print "Object reference is not an RTC::Control - not connected"
        else:
            if self.debug:
                print "Connected to rtc Control Corba",self.obj
            # Invoke the echoString operation
            message = "Hello from Python"
            try:
                result = self.obj.echoString(message)
            except:
                result="nothing (failed)"
                traceback.print_exc()
                print "EchoString failed - continuing but not connected"
                self.obj=None
            if self.debug:
                print "I said '%s'. The object said '%s'." % (message,result)
        #fdata=control_idl._0_RTC.Control.FDATA(10,numpy.arange(10).astype("f").tostring())
        #self.obj.WFsetRefSlope(fdata)
        return self.obj!=None
    def Set(self,name,val,com="",swap=1,check=1,copy=1):
        return self.set(name,val,com,swap,check,copy)

    def set(self,name,val,com="",swap=1,check=1,copy=1):
        if type(name)==type(""):
            val=[val]#single value only.
        rt=self.obj.Set(sdata(name),encode(val),sdata(com),swap,check,copy)
        return decode(rt)
        #elif type(name)==type([]):
        #    self.obj.Set(sdata(name),encode(val),sdata(com),swap,check)
    def RTCinit(self,fname):
        self.obj.RTCinit(encode(fname))
    def ControlHalt(self,stopRTC=1):
        self.obj.ControlHalt(stopRTC)
    def RTChalt(self):
        self.obj.RTChalt()
    # def WFsetBckgrd(self,fdata):
    #     self.obj.WFsetBckgrd(convert(fdata.astype(numpy.float32)))
    # def WFsetGain(self,fdata):
    #     self.obj.WFsetGain(convert(fdata.astype(numpy.float32)))
    # def WFsetRefSlope(self,fdata):
    #     self.obj.WFsetRefSlope(convert(fdata.astype(numpy.float32)))
    # def WFsetThreshold(self,f):
    #     self.obj.WFsetThreshold(float(f))
    # def CopenLoop(self,i):
    #     self.obj.CopenLoop(int(i))
    # def CsetMirror(self,uhdata):
    #     self.obj.CsetMirror(convert(uhdata.astype(numpy.uint16)))
    # def SetKalman(self,d1,d2,d3,d4):
    #     self.obj.SetKalman(convert(d1.astype(numpy.float32)),convert(d2.astype(numpy.float32)),convert(d3.astype(numpy.float32)),convert(d4.astype(numpy.float32)))
    def SetDecimation(self,name,d1,d2=1,log=0,fname="",remote=1,local=1):
        if remote:#set remote decimate (if it exists)
            self.obj.SetDecimation(name,d1,d2,log,fname)
        if local:#set local decimate (a receiver writing to shm) if it exists
            try:
                cb=buffer.Circular("/"+name)
            except:
                cb=None
            if cb!=None:
                cb.freq[0]=d1

    def Get(self,name):
        return decode(self.obj.Get(name))
    def AverageImage(self,n,whole=0):
        img=self.obj.AverageImage(n,whole)
        img=numpy.fromstring(img.data,numpy.float32)
        return img
    def AverageCentroids(self,n):
        img=self.obj.AverageCentroids(n)
        img=numpy.fromstring(img.data,numpy.float32)
        return img
    def GetComment(self,name):
        c=self.obj.GetComment(name)
        return c
    def GetStream(self,name,latest=0,wholeBuffer=0):
        """Get a single frame of data from a given stream"""
        data=self.obj.GetStream(name,latest,wholeBuffer)
        return decode(data)#returns data,time,fno
    def GetVersion(self):
        data=self.obj.GetVersion()
        data+="\nlocal controlCorba.py version:"+CVSID
        return data
    def localRead(self,name,callback,lock,done,decimate,sendFromHead,resetDecimate,readFrom,readTo,readStep):
        import buffer
        buf=buffer.Circular("/"+name)#name includes prefix
        decorig=int(buf.freq[0])
        d=None
        if decorig==0:
            d=decimate
        elif decorig>decimate:
            d=hcf(decorig,decimate)
        elif decimate%decorig!=0:
            d=hcf(decorig,decimate)
        if d!=None:
            buf.freq[0]=d
        else:
            resetDecimate=0#no need to reset because we didn't set.
        go=1
        cumfreq=decimate
        buf.getLatest()
        while go:
            data=buf.getNextFrame()
            lw=int(buf.lastWritten[0])
            if lw>=0:
                diff=lw-buf.lastReceived
                if diff<0:
                    diff+=buf.nstore[0]
                if diff>buf.nstore[0]*0.75:
                    print "Sending of %s lagging locally, skipping %d frames"%(name,diff-1)
                    data=buf.get(lw)
            if data!=None:
                freq=int(buf.freq[0])
                if freq<1:
                    freq=1
                cumfreq+=freq
                cumfreq-=cumfreq%freq
                if cumfreq>=decimate:#so now send the data
                    cumfreq=0
                    if decimate%freq==0:#synchronise frame numbers
                        cumfreq=data[1]%decimate
                    if sendFromHead==1 and lw>0 and freq!=1:
                        data=buf.get(lw)
                else:
                    data=None
            #print "Got next"
            if data!=None:#convert from memmap to array.. subsample if neccesary
                if readFrom>0 or readTo>0 or readStep>1:
                    if readTo==-1:
                        readToTmp=data[0].size
                    else:
                        readToTmp=readTo
                    data=(numpy.array(data[0][readFrom:readToTmp:readStep]),data[1],data[2])
                else:
                    data=(numpy.array(data[0]),data[1],data[2])
            #print data
                lock.acquire()#only 1 can call the callback at once.
                #print "Got lock"
                try:
                    if callback(["data",name,data])!=0:
                        #print "Ending"
                        done[0]+=1
                        go=0
                    lock.release()
                except:
                    go=0
                    lock.release()
                    raise
            #print "Released go=%d"%go
        #print "localRead thread finishing"
        if resetDecimate:
            buf.freq[0]=decorig

    def Subscribe(self,namelist,callback,decimate=None,host=None,verbose=0,sendFromHead=0,startthread=1,timeout=None,timeoutFunc=None,resetDecimate=1,readFrom=0,readTo=-1,readStep=1):
        """Subscribe to the streams in namelist, for nframes starting at fno calling callback when data is received.
        if decimate is set, sets decimate of all frames to this.
        callback should accept 1 argument, which is ["data",streamname,(data,frame time, frame number)]
        If callback returns 1, the connection will be closed.
        """
        if host==None:
            # get a list of network interfaces
            host=[x[1] for x in getNetworkInterfaces()]
        if type(namelist)!=type([]):
            namelist=[namelist]
        #c=threadCallback(callback)#justs makes sure the callback is called in a threadsafe way... actually - I think we're okay...
        r=recvStream.Receiver(len(namelist),callback,host,bindto="",start=startthread,verbose=verbose,timeout=timeout,timeoutFunc=timeoutFunc)#this starts running in a new thread... and finishes when everything has connected and disconnected.

        d=decimate
        if decimate==None:
            d=-1
        if type(r.hostList)==type([]):
            hostlist=string.join(r.hostList,",")
        else:
            hostlist=r.hostList
        self.obj.StartStream(sdata(namelist),hostlist,r.port,d,sendFromHead,"",resetDecimate,readFrom,readTo,readStep)
        return r
    def GetStreamBlock(self,namelist,nframes,fno=None,callback=None,decimate=None,flysave=None,block=0,returnData=None,verbose=0,myhostname=None,printstatus=1,sendFromHead=0,asfits=0,localbuffer=1,returnthreadlist=0,resetDecimate=1,readFrom=0,readTo=-1,readStep=1,nstoreLocal=100):
        """Get nframes of data from the streams in namelist.  If callback is specified, this function returns immediately, and calls callback whenever a new frame arrives.  If callback not specified, this function blocks until all data has been received.  It then returns a dictionary with keys equal to entries in namelist, and values equal to a list of (data,frametime, framenumber) with one list entry for each requested frame.
        callback should accept a argument, which is ["data",streamname,(data,frame time, frame number)].  If callback returns 1, assumes that won't want to continue and closes the connection.  Or, if in raw mode, ["raw",streamname,datastr] where datastr is 4 bytes of size, 4 bytes of frameno, 8 bytes of time, 1 bytes dtype, 7 bytes spare then the data
        flysave, if not None will cause frames to be saved on the fly... it can be a string, dictionary or list.
        Waits until frame number >=fno before starting.
        if decimate is set, sets decimate of all streams to this.
        """
        if type(namelist)!=type([]):
            namelist=[namelist]
        if len(namelist)==0:
            return {}
        cb=blockCallback(namelist,nframes,callback,fno,flysave,returnData,asfits=asfits)#namelist should include the shmPrefix here
        if localbuffer==0:#get data over a socket...
            r=self.Subscribe(namelist,cb.call,decimate=decimate,host=myhostname,verbose=verbose,sendFromHead=sendFromHead,resetDecimate=resetDecimate,readFrom=readFrom,readTo=readTo,readStep=readStep)#but here, namelist shouldn't include the shm prefix - which is wrong - so need to make changes so that it does...
            rt=r
            if ((callback==None and flysave==None) or block==1):
                try:
                    #block until all frames received...
                    got=False
                    cnt=0
                    if decimate==None:
                        dec=1
                    else:
                        dec=abs(decimate)
                    next=5+2*nframes/150.*dec#number of seconds that we expect to take getting data plus 5 seconds grace period
                    if next<1:
                        next=1
                    next=int(next)
                    while got==False:
                        got=cb.lock.acquire(0)
                        if got==False:
                            cnt+=1
                            if cnt==next:
                                if printstatus:
                                    print "%s Streams so far: %s.  Still waiting for %s to finish (frames still to go: %s, got %s).  %d still left to connect (to %s)"%(time.strftime("%H:%M:%S"),str(r.d.streamList),str(r.d.sockStreamDict.values()),str(cb.nframe),str(cb.nframeRec),r.d.nconnect,r.hostList)
                                next*=2
                            time.sleep(1)
                    cb.lock.release()
                    rt=cb.data
                except KeyboardInterrupt:
                    # wait for datawriting to stop...
                    r.d.sockConn.go=0
                    r.thread.join(1)#wait for finish...
                    if asfits:
                    # finalise the FITS files.
                        for k in cb.saver.keys():
                            cb.saver[k].fitsFinalise()
                    raise
        else:#using a local shm circular buffer to get the data from.
            #i.e. we are on the rtc, or receiver is running
            if decimate==None:
                decimate=1
            #Now read all the stuff...
            decorig=self.GetDecimation(remote=0)["local"]
            rtcreset={}
            rtcdec=self.GetDecimation(local=0)
            outputnameList=[]
            if decimate>0:#now start it going.
                for name in namelist:
                    pname=name[:-3]+"f%dt%ds%dBuf"%(readFrom,readTo,readStep)
                    if decorig.has_key(name) or decorig.has_key(pname):#the local receiver exists.  But we should check the shm owner pid, to see if the owner still exists...
                        if decorig.has_key(pname):
                            outputnameList.append((pname,1))
                        else:
                            outputnameList.append((name,0))
                        #Now, when localRead() is called, it will sort out the local decimation.  But, here, we should also sort out the RTC decimation.
                        d=None
                        if rtcdec.has_key(name):
                            if rtcdec[name]==0:
                                d=decimate
                            elif rtcdec[name]>decimate:
                                d=hcf(rtcdec[name],decimate)
                            elif decimate%rtcdec[name]!=0:
                                d=hcf(rtcdec[name],decimate)
                            if d!=None:
                                rtcreset[name]=rtcdec[name]
                                self.SetDecimation(name,d,local=0)
                    else:#have to start the receiver...
                        if readFrom>0 or readTo>0 or readStep>1:
                            outputname=pname
                            outputnameList.append((outputname,1))
                        else:
                            outputname=name
                            outputnameList.append((outputname,0))
                        print "Starting receiver %s into %s"%(name,outputname)
                        self.StartReceiver(name,decimate,sendFromHead=sendFromHead,outputname=outputname,nstore=nstoreLocal,port=4262,readFrom=readFrom,readTo=readTo,readStep=readStep)


            else:#if there are streams not switched on, turn them on...
                #Not sure why we do this...?  Turns everything on!
                print "AREYOUSURE (controlCorba?)"
                for k in decorig.keys():
                    if decorig[k]==0:
                        self.SetDecimation(k,1)




            #Need a thread for each circular buffer
            tlist=[]
            lock=threading.Lock()
            done=numpy.zeros((1,),numpy.int32)
            for name,isPartial in outputnameList:
                if isPartial:#the buffer we're reading from has already done the sub-sampling
                    tlist.append(threading.Thread(target=self.localRead,args=(name,cb.call,lock,done,decimate,sendFromHead,resetDecimate,0,-1,1)))
                else:#need to subsample this buffer
                    tlist.append(threading.Thread(target=self.localRead,args=(name,cb.call,lock,done,decimate,sendFromHead,resetDecimate,readFrom,readTo,readStep)))

                tlist[-1].daemon=True
                tlist[-1].start()

            if ((callback==None and flysave==None) or block==1):
                cnt=0
                dec=abs(decimate)
                next=5+2*nframes/150.*dec#number of seconds that we expect to take getting data plus 5 seconds grace period
                if next<1:
                    next=1
                next=int(next)
                try:
                    for t in tlist:#wait for all to finish
                        done=0
                        while done==0:
                            t.join(1)
                            if not t.isAlive():
                                done=1
                            else:
                                cnt+=1
                                if cnt==next:
                                    if printstatus:
                                        print "%s Streams: %s. still to go %s got %s"%(time.strftime("%y%m%d %H%M%S"),namelist,str(cb.nframe),str(cb.nframeRec))
                                    next*=2
                except KeyboardInterrupt:
                    cb.err=1
                    #and now reset the decimations.
                    raise
                # and now reset the decimations.
                if resetDecimate:
                    for name in rtcreset.keys():
                        self.SetDecimation(name,rtcreset[name],local=0)
                        

            rt=cb.data
            if returnthreadlist:
                rt=tlist
        return rt
    def GetLabels(self):
        labels=self.obj.GetLabels()
        return decode(labels)
    def WaitParamChange(self,timeout):
        if timeout==None:
            timeout=-1
        return self.obj.WaitParamChange(timeout)
    def AutoPoke(self,framesIgnore,framesApply,voltChange):
        pmx=self.obj.CdoInteractM(framesIgnore,encode(voltChange),framesApply,0,0,0,0,0,0,0)
        pmx=numpy.fromstring(pmx.data,numpy.float32)
        nacts=self.Get("nacts")
        pmx.shape=nacts,pmx.shape[0]/nacts
        return pmx
    def Poke(self,arr,steps):
        arr=convert(arr.astype(numpy.float32))
        steps=convert(steps.astype(numpy.int32))
        pmx=self.obj.CmakeInteractM(arr,steps)
        pmx=decode(pmx)
        nacts=self.Get("nacts")
        pmx.shape=nacts,pmx.shape[0]/nacts
        return pmx
    def GetLog(self):
        txt=self.obj.GetLog()
        return txt
    def GetLogFiles(self):
        txt=self.obj.GetLogFiles()
        return txt.split(",")
    def CalibrateWholeImage(self,copy=1):
        subloc=self.obj.CalibrateWholeImage(copy)
        return decode(subloc)
    def RestorePartialImageCalibration(self):
        rt=self.obj.RestorePartialImageCalibration()
        return rt
    def Transfer(self,data,fname):
        return self.obj.Transfer(encode(data),fname)
    def ReleaseLock(self):
        return self.obj.ReleaseLock()
    def Execute(self,cmd,vals=[]):
        data=self.obj.Execute(cmd,encode(vals))
        data=decode(data)
        return data
    def TogglePause(self,p):
        data=self.obj.TogglePause(p)
        return data
    def Remove(self,name,returnval=1,doSwitch=1):
        data=self.obj.Remove(name,returnval,doSwitch)
        data=decode(data)
        return data
    def GetDecimation(self,remote=1,local=1):
        d={}
        if remote:
            data=self.obj.GetDecimation()
            data=decode(data)
            for i in range(0,len(data),2):
                d[data[i]]=data[i+1]
        if local:
            loc={}
            files=os.listdir("/dev/shm")
            start=self.prefix+"rtc"
            lstart=len(start)
            s=[]
            for f in files:
                if f[:lstart]==start and f[-3:]=="Buf":
                    s.append(f)
            streams=s
            #streams=startStreams.getStreams(self.prefix)
            for stream in streams:
                try:
                    cb=buffer.Circular("/"+stream)
                    if os.path.exists("/proc/%d"%cb.ownerPid[0]):
                        #owner of this stream exists
                        loc[stream]=int(cb.freq[0])
                    else:#no owner - so remove the shm
                        try:
                            os.unlink("/dev/shm/"+stream)
                        except:
                            pass
                    
                except:
                    pass
            d["local"]=loc
        return d
    def Swap(self,n1,n2):
        self.obj.Swap(n1,n2)
    def WakeLogs(self,flag):
        self.obj.WakeLogs(flag)
    def GetActiveBufferArray(self):
        buf=numpy.fromstring(self.obj.GetActiveBufferArray().data,'c')
        return buf

    def ConnectParamSubscriber(self,host,port,names):
        self.obj.ConnectParamSubscriber(host,port,sdata(names))
    def SubscribeParams(self,params,callback=None,savefd=None,host=None):
        """callback(fno,tme,paramDict)"""
        import serialise
        if host==None:
            host=[x[1] for x in getNetworkInterfaces()]
            #host=socket.gethostbyaddr(socket.gethostname())[2][0]
            #if host=="127.0.0.1":
            #    print "Warning - got localhost as hostname"
        s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        bound=0
        port=4242
        while bound==0:
            try:
                s.bind(("",port))
            except:
                print "Couldn't bind to port %d.  "%(port)
                port+=1
                time.sleep(0.5)
                #raise
            else:
                bound=1
        if bound:
            s.listen(1)
        go=1
        while go==1:
            err=0
            if type(host)==type([]):
                host=string.join(host,",")

            self.ConnectParamSubscriber(host,port,params)
            conn,raddr=s.accept()
            print "accepted %s"%str(raddr)
            #First, before doing anything else, we read the current buffer, to get current state of the system...
            if savefd!=None:
                bufarr=self.GetActiveBufferArray()
                import buffer
                buf=buffer.Buffer(None,size=bufarr.size*bufarr.itemsize)
                buf.assign(bufarr)#arr[:]=bufarr
                #buf.setNhdr(buf.nhdr[0])
                d={}
                for l in buf.getLabels():
                    d[l]=(buf.get(l),buf.getComment(l))
                try:
                    fno=buf.get("frameno")
                except:
                    fno=0
                try:
                    tme=buf.get("switchTime")
                except:
                    tme=0.
                savefd.write(serialise.Serialise([fno,tme,d]))
                print "Saved initial state"
            
            while err==0 and go==1:
                try:
                    msg=serialise.ReadMessage(conn)
                except KeyboardInterrupt:
                    go=0
                except:
                    print "Error in ReadMessage"
                    traceback.print_exc()
                    err=1
                if msg==None:
                    go=0
                if err==0 and go==1 and msg[0]=="params":
                    if savefd!=None:
                        savefd.write(serialise.Serialise(msg[1:]))
                    if callback!=None:
                        if callback(msg[1],msg[2],msg[3])!=0:
                            go=0
            conn.close()
        s.close()
    def StartLogStream(self,host,port,name="/dev/shm/rtcStdout0",limit=1024*80,sleeptime=5.):
        self.obj.StartLogStream(host,port,name,limit,float(sleeptime))

    def SubscribeLog(self,rtclog=1,ctrllog=1,alllog=1,host=None,limit=1024*80,sleeptime=5,callback=None,specificFile=None):
        
        s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        bound=0
        port=4242
        while bound==0:
            try:
                s.bind(("",port))
            except:
                print "Couldn't bind to port %d.  "%(port)
                port+=1
                time.sleep(0.5)
                #raise
            else:
                bound=1
                print "Bound to port %d"%port
        if bound:
            s.listen(rtclog+ctrllog)

            if host==None:
                host=string.join([x[1] for x in getNetworkInterfaces()],",")
            connList=[]
            if specificFile!=None:
                self.StartLogStream(host,port,"/dev/shm/%s"%specificFile,limit,sleeptime)
                conn,raddr=s.accept()
                connList.append(conn)
                print "Accepted logs %s"%str(raddr)
            elif alllog:
                self.StartLogStream(host,port,"ALL",limit,sleeptime)
                conn,raddr=s.accept()
                connList.append(conn)
                print "accepted logs %s"%str(raddr)
            else:
                if rtclog:
                    self.StartLogStream(host,port,"/dev/shm/%srtcStdout0"%self.prefix,limit,sleeptime)
                    conn,raddr=s.accept()
                    connList.append(conn)
                    print "accepted rtclog %s"%str(raddr)
                if ctrllog:
                    self.StartLogStream(host,port,"/dev/shm/%srtcCtrlStdout0"%self.prefix,limit,sleeptime)
                    conn,raddr=s.accept()
                    connList.append(conn)
                    print "accepted ctrllog %s"%str(raddr)
            s.close()
            #now just read the sockets...
            while len(connList)>0:
                rtr,rtw,err=select.select(connList,[],[])
                for r in rtr:
                    data=r.recv(4096)
                    if len(data)==0:
                        connList.remove(r)
                        print "Connection closed"
                    else:
                        print data
                        if callback!=None:
                            if callback(data)==1:
                                connList.remove(r)


    def RemoveError(self,err):
        self.obj.RemoveError(err)

    def GetErrors(self):
        return decode(self.obj.GetErrors())

    def StartSummer(self,stream,nsum,decimation=1,affin=0x7fffffff,prio=0,fromHead=1,startWithLatest=1,rolling=0,dtype='n',outputname=None,nstore=10):
        if outputname==None:
            outputname=""
        data=self.obj.StartSummer(stream,nsum,decimation,affin,prio,fromHead,startWithLatest,rolling,dtype,outputname,nstore)
        return data

    def StopSummer(self,name):
        self.obj.StopSummer(name)

    def GetSummerList(self):
        lst=decode(self.obj.GetSummerList())
        return lst


    def StartSplitter(self,stream,readfrom=0,readto=-1,readstep=1,affin=0x7fffffff,prio=0,fromHead=0,outputname=None,nstore=-1):
        if outputname==None:
            outputname=""
        data=self.obj.StartSplitter(stream,readfrom,readto,readstep,affin,prio,fromHead,outputname,nstore)
        return data

    def StopSplitter(self,name):
        self.obj.StopSplitter(name)

    def GetSplitterList(self):
        lst=decode(self.obj.GetSplitterList())
        return lst


    def SumData(self,stream,n,dtype="n",setdec=1):
        #Summing is done on the RTC.  So, need to change the decimate there.
        decorig=None
        #if setdec:
        #    try:
        #        decorig=self.GetDecimation(local=0)[self.prefix+stream]
        #    except:
        #        pass
        #    self.SetDecimation(self.prefix+stream,1,local=0)
        data=self.obj.SumData(stream,n,dtype)
        if decorig!=None:
            self.SetDecimation(self.prefix+stream,decorig,local=0)
        data=decode(data)
        return data

    def StartReceiver(self,name,decimation,datasize=None,affin=0x7fffffff,prio=0,sendFromHead=1,outputname=None,nstore=10,port=4262,readFrom=0,readTo=-1,readStep=1):
        """Starts a receiver locally.  This then receives data from the RTC and writes it to a local shared memory circular buffer, which other local clients can then read.  name here includes prefix, but this shouldn't be sent to receiver.
        """
        if outputname==None:
            outputname=name
        if datasize==None:
            #work out the size of the data...
            data=self.GetStream(name)[0]
            datasize=(data.size*data.itemsize+32)*nstore+buffer.getHeaderSize()

        plist=["receiver","-p%d"%port,"-a%d"%affin,"-i%d"%prio,"-n%d"%datasize,"-o/%s"%outputname,name[len(self.prefix):]]
        if self.prefix!="":
            plist.append("-s%s"%self.prefix)
        if os.path.exists("/dev/shm/%s"%outputname):
            raise Exception("local /dev/shm/%s already exists"%outputname)
        p=subprocess.Popen(plist)
        #Now wait for it to bind, and get the bound port
        cnt=0
        while cnt<100 and not os.path.exists("/dev/shm/%s"%outputname):
            cnt+=1
            time.sleep(0.01)
        if cnt==100:
            p.terminate()
            p.wait()
            raise Exception("Local /dev/shm/%s not found"%outputname)
        cb=buffer.Circular("/%s"%(outputname))
        cnt=0
        s=None
        while cnt<100 and s==None:
            s=cb.getLatest()
            if s==None:
                time.sleep(0.01)
                cnt+=1
        if s!=None:
            port=int(s[0][0])
        else:
            p.terminate()
            p.wait()
            raise Exception("Unable to determine port of receiver")
        print "Local receiver has started and is listening on port %d"%port
        #Now start the sender, and we're done...
        hostlist=string.join([x[1] for x in getNetworkInterfaces()],",")
        reset=0#don't want to reset the stream...
        self.obj.StartStream(sdata([name]),hostlist,port,decimation,sendFromHead,"name",reset,readFrom,readTo,readStep)


    def StopReceiver(self,name):
        pass
    def GetReceiverList(self):
        pass

    def WatchParam(self,tag,paramList,timeout=-1):
        plist=sdata(paramList)
        changed=self.obj.WatchParam(tag,plist,float(timeout))
        changed=decode(changed)
        tag=changed.pop(0)
        return tag,changed
        

class threadCallback:
    def __init__(self,callback):
        self.callback=callback
        self.lock=threading.Lock()
    def call(self,data):
        self.lock.acquire()
        try:
            self.callback(data)
        except:
            self.lock.release()
            raise
        self.lock.release()

class blockCallback:
    def __init__(self,namelist,nframes,callback=None,fno=None,flysave=None,returnData=None,asfits=0):
        self.nframe={}
        self.nframeRec={}
        self.data={}
        self.asfits=asfits
        self.connected={}
        if type(namelist)!=type([]):
            namelist=[namelist]
        self.namelist=namelist
        for n in namelist:
            self.nframe[n]=nframes
            self.nframeRec[n]=0
            self.data[n]=[]
            self.connected[n]=0
        self.callback=callback
        self.err=0
        self.lock=threading.Lock()
        self.lock.acquire()
        self.incrementalFno=0
        if fno!=None and fno<0:
            fno=-fno
            self.incrementalFno=1
            #want to wait for all streams to connect.  Then wait fno many frames before starting collecting.
        self.fno=fno
        self.returnData=returnData
        if returnData==None and (callback!=None or flysave!=None):
            self.returnData=0
        else:
            if returnData==None:
                returnData=1
            self.returnData=returnData
        self.saver={}
        self.flysave=flysave
        if flysave!=None:
            if type(flysave)==type(""):
                if len(namelist)==1:
                    self.flysave={namelist[0]:flysave}
                else:
                    self.flysave={}
                    for n in namelist:
                        if asfits:
                            self.flysave[n]=flysize+n+".fits"
                        else:
                            self.flysave[n]=flysave+n+".log"
            elif type(flysave)==type([]):
                self.flysave={}
                for i in range(len(namelist)):
                    self.flysave[namelist[i]]=flysave[i]
            elif type(flysave)==type({}):
                #assume a key equal to all entries in namelist.
                k=self.flysave.keys()
                for n in namelist:
                    if n not in k:
                        if asfits:
                            self.flysave[n]=n
                        else:
                            self.flysave[n]=n+".log" 

    def call(self,data):
        if self.err:
            return 1
        rt=0
        process=0
        if data[0]=="data":#data contains ["data",streamname,(data,frametime,frame number)]
            name=data[1]
            process=1
            datafno=data[2][2]
        elif data[0]=="raw":#data contains ["raw",streamname,datastring]
            #datastring is 4 bytes of size, 4 bytes of frameno, 8 bytes of time, 1 bytes dtype, 15 bytes spare then the data
            name=data[1]
            #print "raw",name
            if numpy.fromstring(data[2][0:4],dtype=numpy.int32)[0]>28:
                #this means no data (header only) if sizze==28.
                datafno=numpy.fromstring(data[2][4:8],dtype=numpy.uint32)
                datatime=numpy.fromstring(data[2][8:16],dtype=numpy.float64)
                thedata=numpy.fromstring(data[2][32:],dtype=data[2][16])
                data=["data",name,(thedata,datatime,datafno)]
                process=1
        if process:
            #print data[2][0].shape
            if self.nframe.has_key(name) and self.nframe[name]!=0:
                if self.incrementalFno:#want to start at frame number + fno
                    self.connected[name]=1
                    #Now check that all have connected...
                    allconnected=1
                    for k in self.connected.keys():
                        if self.connected[k]==0:
                            allconnected=0
                            break
                    if allconnected:
                        self.incrementalFno=0
                        self.fno+=datafno#data[2][2]
                #print self.incrementalFno,self.fno,datafno
                if self.incrementalFno==0 and (self.fno==None or datafno>=self.fno):
                    if self.nframe[name]>0:
                        self.nframe[name]-=1
                    self.nframeRec[name]+=1
                    if self.flysave!=None and self.flysave[name]!=None:
                        self.savecallback(data)
                    if self.callback!=None:
                        rt=self.callback(data)
                    if self.returnData:
                        self.data[name].append(data[2])
                    release=1
                    #if all frames done, we can release...
                    for n in self.nframe.keys():
                        if self.nframe[n]!=0:
                            release=0
                            break
                        else:#no frames left to do - close the save file, if open
                            saver=self.saver.get(data[1],None)
                            if saver!=None:
                                saver.close()
                                del(self.saver[data[1]])
                    if release or rt:
                        self.lock.release()
            else:
                #print "Not expecting stream %s (expecting %s)"%(name,str(self.nframe.keys()))
                rt=1
        #print "done"
        return rt

    def savecallback(self,data):
        """Can be used to save frames as they arrive...
        """
        saver=self.saver.get(data[1],None)
        if saver==None:
            saver=Saver.Saver(self.flysave[data[1]],"w+")
            self.saver[data[1]]=saver
        saver.write(data[2][0],data[2][1],data[2][2])
        return 0
        


def initialiseServer(c=None,l=None,block=0,controlName="Control"):
    """c is the control object
    l is a threading.Lock object (or soemthing with acquire and release methods
    block is whether to block here, or return.
    """
    # Initialise the ORB and find the root POA
    orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)
    poa = orb.resolve_initial_references("RootPOA")
    # Create an instance of Control_i and a Control object reference
    ei = Control_i(c,l)
    eo = ei._this()
    # Obtain a reference to the root naming context
    obj         = orb.resolve_initial_references("NameService")
    try:
        rootContext = obj._narrow(CosNaming.NamingContext)
    except:
        print "Unable to connect the nameservice"
        return None
    if rootContext is None:
        print "Failed to narrow the root naming context"
        sys.exit(1)
    # Bind a context named "rtcServer.my_context" to the root context
    name = [CosNaming.NameComponent("rtcServer", "my_context")]
    try:
        rtcServerContext = rootContext.bind_new_context(name)
        print "New rtcServer context bound"
    except CosNaming.NamingContext.AlreadyBound, ex:
        print "RtcControl context already exists"
        obj = rootContext.resolve(name)
        rtcServerContext = obj._narrow(CosNaming.NamingContext)
        if rtcServerContext is None:
            print "rtcServer.mycontext exists but is not a NamingContext"
            sys.exit(1)
    # Bind the Control object to the rtcServer context
    name = [CosNaming.NameComponent(controlName, "Object")]
    try:
        rtcServerContext.bind(name, eo)
        print "New Control object bound"
    except CosNaming.NamingContext.AlreadyBound:
        rtcServerContext.rebind(name, eo)
        print "Control binding already existed -- rebound"
    # Activate the POA
    poaManager = poa._get_the_POAManager()
    poaManager.activate()
    if block:
        # Block for ever (or until the ORB is shut down)
        orb.run()
    return ei

def getNetworkInterfaces():
    """
    Used to get a list of the up interfaces and associated IP addresses
    on this machine (linux only).

    Returns:
        List of interface tuples.  Each tuple consists of
        (interface name, interface IP)
    """

    if os.environ.has_key("MYIPFORDARC"):
        return [os.environ["MYIPFORDARC"]]
    import fcntl
    import array
    import struct
    import socket
    import platform
    SIOCGIFCONF = 0x8912
    MAXBYTES = 8096

    arch = platform.architecture()[0]

    # I really don't know what to call these right now
    var1 = -1
    var2 = -1
    if arch == '32bit':
        var1 = 32
        var2 = 32
    elif arch == '64bit':
        var1 = 16
        var2 = 40
    else:
        print "Unable to obtain list of IP addresses.  You could try setting environment variable MYIPFORDARC = you IP address (probably you are on a mac?)"
        raise OSError("Unknown architecture: %s" % arch)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    names = array.array('B', '\0' * MAXBYTES)
    outbytes = struct.unpack('iL', fcntl.ioctl(sock.fileno(),SIOCGIFCONF,struct.pack('iL', MAXBYTES, names.buffer_info()[0])))[0]
    namestr = names.tostring()
    return [(namestr[i:i+var1].split('\0', 1)[0], socket.inet_ntoa(namestr[i+20:i+24])) for i in xrange(0, outbytes, var2)]



if __name__=="__main__":
    controlName="Control"
    for arg in sys.argv[1:]:
        if arg[:2]=="-s":#shmname
            controlName=arg[2:]+controlName
    
    initialiseServer(block=1,controlName=controlName)

