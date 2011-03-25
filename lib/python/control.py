#!/usr/bin/env python
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
CVSID="$Id$"
import numpy
import time
import os
#import thread
import sys
import socket
import threading
import traceback
import buffer
import SockConn
import utils
import serialise
import controlCorba
import Check
import startStreams
import subprocess
import logread
import stdoutlog

DataSwitch=None
# try:
#     import DataSwitch
# except:
#     DataSwitch=None
#     traceback.print_exc()
#     print "DATASWITCH not imported - will not talk to telemetry server - check python path"
# try:
#     class config_handler_i(DataSwitch.DataSwitchModule__POA.ConfigSubscriber,DataSwitch.Subscriber):
#         def __init__(self,dataswitch,msg="config",callback=None):
#             DataSwitch.Subscriber.__init__(self, dataswitch)
#             self.msg=msg
#             self.callback=callback
#         def update(self, status):
#             #print self.msg,status
#             if self.callback!=None:
#                 self.callback(self.msg,status)
# except:
#     pass
PS=None
# if DataSwitch==None:
#     try:
#         import PS
#     except:
#         print "Couldn't import PS"
#         PS=None

if PS!=None:
    import PSuser


class Control:
    """A class for controlling the RTC.  The GUI connects to this object.
    Opens a socket and listens for the GUI...
    """
    def __init__(self,globals):
        global DataSwitch
        global PS
        self.globals=globals
        self.circgo=1
        self.go=1
        self.nodarc=0
        self.coremain=None
        self.serverObject=None
        self.pipeDict={}
        self.paramChangedDict={}
        self.paramChangedSubscribers={"tag":{}}
        self.summerDict={}
        self.errList=[]
        self.rtcStopped=0
        self.dsConfig=None
        self.reconnectRunning=0
        self.check=Check.Check()
        self.port=4242
        self.host=socket.gethostname()
        self.configFile=None
        self.rtcDecimation={}
        self.circBufDict={}
        self.streamList=[]
        self.savedState={}
        self.shmPrefix=""
        self.sockConn=None
        self.logread=None
        self.ctrllogread=None
        self.readStreams=0
        self.conditionVar=threading.Condition()
        self.redirectdarc=1
        self.redirectcontrol=1
        self.paramTagRef=1
        self.bufsize=None
        self.nhdr=None
        affin=0x7fffffff
        uselock=1
        prio=0
        for arg in sys.argv[1:]:
            if arg[:2]=="-p":#port
                self.port=int(arg[2:])
                print "Using port %d"%self.port
            elif arg[:2]=="-h":#host
                self.host=arg[2:]
                print "Using host %s"%self.host
            elif arg[:2]=="-s":#shm prefix
                self.shmPrefix=arg[2:]
            elif arg[:2]=="-d":#dataswitch
                DataSwitch=None#don't connect...
                PS=None
                #self.readStreams=1
            elif arg[:2]=="-a":#affinity
                affin=int(arg[2:])#thread affinity mask
            elif arg[:2]=="-i":#importance... 
                prio=int(arg[2:])
            elif arg[:2]=="-l":#use lock
                uselock=0
            elif arg[:2]=="-o":#don't redirect output of darcmain
                self.redirectdarc=0
                self.redirectcontrol=0
            elif arg[:2]=="-q":#redirecting my output
                self.redirectcontrol=0
            elif arg[:2]=="--":
                if arg[2:9]=="prefix=":
                    self.shmPrefix=arg[9:]
            elif arg[:2]=="-b":
                self.bufsize=int(arg[2:])
            elif arg[:2]=="-e":
                self.nhdr=int(arg[2:])
            elif arg[:2]=="-n":
                self.nodarc=1#no instance of darc - just the shm buffer.
            else:
                self.configFile=arg
                print "Using config file %s"%self.configFile
        print "prefix %s"%self.shmPrefix
        if self.redirectcontrol:
            print "Redirecting control output"
            sys.stdout=stdoutlog.Stdoutlog("/dev/shm/%sctrlout"%self.shmPrefix)
            sys.stderr=sys.stdout
        if uselock:
            self.lock=threading.Lock()
        else:
            self.lock=dummyLock()
        if DataSwitch==None and PS==None:
            self.readStreams=1

        if prio!=0 or affin!=0x7fffffff:
            utils.setAffinityAndPriority(affin,prio)
        if DataSwitch!=None:
            self.connectDataSwitch()
        self.PSClient=None
        self.DecimateHandlerObject=None
        if PS!=None:
            self.PSConnect()

        #Start the RTC if needed... and the sendStreams
        self.initialiseRTC()
        #and now set a watch on the directory to be alerted if new streams appear
        self.watcher=startStreams.WatchStreams(self.shmPrefix,self.streamCreated,self.streamRemoved)#start watching /dev/shm/
        #get a list of current streams
        slist=startStreams.getStreams(self.shmPrefix)
        for s in slist:
            self.streamCreated(s)

        self.wakePipeEnd,self.wakePipe=os.pipe()#a pipe used to wake up the select call...


        self.sockConn=SockConn.SockConn(self.port,host=self.host,globals=self.globals,startThread=0,listenSTDIN=0,userSelList=self.pipeDict.keys()+[self.wakePipeEnd,self.watcher.myfd()],userSelCmd=self.handlePipe,connectFunc=self.newConnection,lock=self.lock)
        if self.sockConn.bound==0:
            print "Exiting..."
            sys.exit(0)
        self.port=self.sockConn.port
        self.logread=logread.logread(name="/dev/shm/%sstdout0"%self.shmPrefix,callback=self.logreadCallback)
        self.logread.sleep=1
        self.logread.launch()
        #thread.start_new_thread(self.watchStreamThread,())
        if self.redirectcontrol:
            self.ctrllogread=logread.logread(name="/dev/shm/%sctrlout0"%self.shmPrefix,callback=self.ctrllogreadCallback)
            self.ctrllogread.sleep=1
            self.ctrllogread.launch()

    def logreadCallback(self,txt):
        """publish the new log txt to any subscribers"""
        if DataSwitch==None and PS==None:
            l=[]
            serialise.SerialiseToList(["data","%srtclog"%self.shmPrefix,(txt,0,0)],l)
            for sock in self.sockConn.selIn:#infoDict["subscribers"].keys():
                #check that the socket is still open...
                if type(sock)==type(0) or type(sock._sock)==socket._closedsocket:#socket has been closed.
                    #print "Removing closed socket from subscribe list"
                    #del(infoDict["subscribers"][sock])
                    pass
                else:
                    if sock not in [self.sockConn.lsock,sys.stdin,self.sockConn.fsock]+self.sockConn.userSelList:#a client
                        if self.sockConn.writeDict.has_key(sock):
                            self.sockConn.writeDict[sock]+=l
                        else:
                            self.sockConn.writeDict[sock]=l
                        os.write(self.wakePipe,"a")
        else:
            if self.PSClient!=None:
                a=PS.DataStream(1,0.,"s",(len(txt),),[],txt)
                self.PSClient.publishDataStream(a,"rtcLog")
        return 0
    def ctrllogreadCallback(self,txt):
        """publish the new log txt to any subscribers"""
        if DataSwitch==None and PS==None:
            l=[]
            serialise.SerialiseToList(["data","%sctrllog"%self.shmPrefix,(txt,0,0)],l)
            for sock in self.sockConn.selIn:#infoDict["subscribers"].keys():
                #check that the socket is still open...
                if type(sock)==type(0) or type(sock._sock)==socket._closedsocket:#socket has been closed.
                    #print "Removing closed socket from subscribe list"
                    #del(infoDict["subscribers"][sock])
                    pass
                else:
                    if sock not in [self.sockConn.lsock,sys.stdin,self.sockConn.fsock]+self.sockConn.userSelList:#a client
                        if self.sockConn.writeDict.has_key(sock):
                            self.sockConn.writeDict[sock]+=l
                        else:
                            self.sockConn.writeDict[sock]=l
                        os.write(self.wakePipe,"a")
        else:
            if self.PSClient!=None:
                a=PS.DataStream(1,0.,"s",(len(txt),),[],txt)
                self.PSClient.publishDataStream(a,"ctrlLog")
        return 0
    def wakeLogs(self,wake):
        self.logread.sleep=wake
        if self.redirectcontrol:
            self.ctrllogread.sleep=wake


    def initialiseRTC(self,startCirc=1):
        self.circgo=1
        self.rtcStopped=0
        canstart=1
        corestarted=0
        cnt=10
        while cnt>0:
            try:#attempt to open existing...
                self.bufferList=[buffer.Buffer("/%srtcParam1"%self.shmPrefix),buffer.Buffer("/%srtcParam2"%self.shmPrefix)]
                self.rtcStarted=1
            except:
                self.bufferList=None
                self.rtcStarted=0
            if self.rtcStarted==0 and self.nodarc==1:
                #need to create the buffer ourselves...
                self.rtcStarted=1
                corestarted=1
                if self.bufsize==None:
                    bufsize=64*1024*1024
                else:
                    bufsize=self.bufsize
                if self.nhdr==None:
                    nhdr=128
                else:
                    nhdr=self.nhdr
                self.bufferList=[buffer.Buffer("/%srtcParam1"%self.shmPrefix,create=1,size=bufsize,nhdr=nhdr)]
                if os.path.exists("/dev/shm/%srtcParam2"%self.shmPrefix):
                    os.unlink("/dev/shm/%srtcParam2"%self.shmPrefix)
                os.link("/dev/shm/%srtcParam1"%self.shmPrefix,"/dev/shm/%srtcParam2"%self.shmPrefix)
                self.bufferList.append(buffer.Buffer("/%srtcParam2"%self.shmPrefix))
                
            if self.rtcStarted==0:#RTC not started - start it...
                if canstart:
                    if self.coremain!=None:
                        if self.coremain.poll()==None:
                            #still running...
                            self.coremain.terminate()
                            time.sleep(1)
                            if self.coremain.poll()==None:
                                self.coremain.kill()
                    print "Starting RTC"
                    plist=["darcmain","-i"]
                    if self.nhdr!=None:
                        plist.append("-e%d"%self.nhdr)
                    if self.bufsize!=None:
                        plist.append("-b%d"%self.bufsize)
                    if self.redirectdarc==1:
                        plist.append("-r")#redirect to /dev/shm/stdout0
                    if len(self.shmPrefix)>0:
                        plist.append("-s%s"%self.shmPrefix)
                        
                    try:
                        self.coremain=subprocess.Popen(plist)
                    except:
                        print "Failed to execute %s"%str(plist)
                        plist[0]="./darcmain"
                        try:
                            self.coremain=subprocess.Popen(plist)
                            print "Started %s"%str(plist)
                        except:
                            print "Failed to execute %s - please start it yourself"%str(plist)
                        
                    #if os.system("coremain -1 -1 %s &"%self.shmPrefix):
                    #    #Hmm - actually because of teh &, the last line always returns 0#.
                    #    print "Couldn't start coremain - trying ./coremain"
                    #    os.system("./coremain -1 -1 %s &"%self.shmPrefix)
                    corestarted=1
                    canstart=0
            else:
                #if block==0 or self.rtcStarted==1:
                break
            cnt-=1
            print "Waiting for RTC buffers"
            time.sleep(1)
        if self.bufferList==None:
            raise Exception("Failed to initialise RTC")
        #now initialise /rtcParam2 and send a switchRequested...
        print "RTCSTarted %d %d"%(corestarted,self.rtcStarted)
        if self.rtcStarted:
            if (self.bufferList[0].getMem()==0 and self.bufferList[1].getMem()==0):
                startCirc=1#buffer currently empty - so start circular buffers.
            bufno=self.bufferList.index(self.getInactiveBuffer())
            print "Inactive buffer %d"%bufno
            b=self.bufferList[bufno]#1]
            #semval=utils.getSemValue(b.semid,0);
            frozen=b.flags[0]&0x1
            print b.getLabels(),frozen
            if len(b.getLabels())==0 and frozen==0:#time to initialise the buffer (RTC probably waiting).
                #If no config file, then maybe wait for one to be sent via CORBA...
                if self.configFile==None:
                    #raise Exception("No config file specified")
                    print "No config file specified"
                    if corestarted:#buffer will be empty... so tell dataswitch
                        self.publishParams()
                else:
                    print "Loading rtc config"
                    startCirc=1
                    #bufno=self.bufferList.index(self.getInactiveBuffer())
                    self.initialiseBuffer(bufno,self.configFile)

        #Now that the RTC has started, open the circular buffers.
        if startCirc or corestarted:
            #self.circBufDict={}
            for k in self.pipeDict.keys():
                if k in self.sockConn.selIn:
                    self.sockConn.selIn.remove(k)
                if k in self.sockConn.userSelList:
                    self.sockConn.userSelList.remove(k)
            #self.pipeDict={}
            self.streamList=[]
            #time.sleep(1)#give the RTC time to create its circular buffers.
            #self.streamList=startStreams.getStreams(self.shmPrefix)#["rtcPxlBuf","rtcCalPxlBuf","rtcCentBuf","rtcMirrorBuf","rtcActuatorBuf","rtcStatusBuf","rtcTimeBuf","rtcErrorBuf","rtcSubLocBuf","rtcCorrBuf","rtcGenericBuf","rtcFluxBuf"]
            #print "Got streams: %s"%self.streamList
            #for key in self.streamList:
            #    r,w,infoDict=self.createCircBufThread(key,self.circBufDict)#self.circBufDict[key])
            #    self.pipeDict[r]=(w,key,self.circBufDict,infoDict)
            #if self.sockConn!=None:
            #    self.sockConn.userSelList+=self.pipeDict.keys()
            #    self.sockConn.selIn+=self.pipeDic.tkeys()
                #Somehow need to wake sockConn from the select loop...
            #    os.write(self.wakePipe,"a")

        #request update of decimates...
        # if DataSwitch!=None and self.serverObject==None:
        #     self.connectDataSwitch()
        # if self.serverObject!=None:#this will cause decimate rates to be updated
        #     try:
        #         self.serverObject.requestConfig()
        #     except:
        #         self.serverObject=None
        #         traceback.print_exc()
        #     if self.serverObject==None:#try to connect.
        #         self.connectDataSwitch()
        #         if self.serverObject!=None:
        #             try:
        #                 self.serverObject.requestConfig()
        #             except:
        #                 self.serverObject=None
        #                 traceback.print_exc()
                    
        # else:
        #     #for k in self.rtcDecimation.keys():
        #     #    self.setRTCDecimation(k,self.rtcDecimation[k])
        #     pass
        # if PS!=None:
        #     if self.PSClient==None:
        #         self.PSConnect()#this requests the decimates (or at least subscribes to them - which means they should be sent).
        #     else:#we need to request the decimates...
        #         try:
        #             PS.addSubscriber(self.PSname,self.DecimateHandlerObject,"Decimates")#subscribe to the decimate values...
        #         except:
        #             self.PSClient=None
        #             traceback.print_exc()
        #         if self.PSClient==None:
        #             self.PSConnect()
        #             if self.PSClient!=None:
        #                 try:
        #                     PS.addSubscriber(self.PSname,self.DecimateHandlerObject,"Decimates")#subscribe to the decimate values...
        #                 except:
        #                     self.PSClient=None
        #                     traceback.print_exc()
            
    def RTCInit(self,config):
        """config can be a filename (fits or py), or a numpy array containing the buffer.
        """
        #If there is an existing RTC, kill it.
        if self.bufferList==None:
            try:#attempt to open existing...
                self.bufferList=[buffer.Buffer("/%srtcParam1"%self.shmPrefix),buffer.Buffer("/%srtcParam2"%self.shmPrefix)]
            except:
                self.bufferList=None
        
        if self.bufferList==None or (self.bufferList[0].getMem()==0 and self.bufferList[1].getMem()==0) or self.rtcStopped==1:
            print "Not stopping RTC",self.bufferList,self.rtcStopped#hasn't yet started
            if self.bufferList!=None:
                print "buffer memory:",self.bufferList[0].getMem(),self.bufferList[1].getMem()
            startCirc=0
        else:
            startCirc=1
            print "Stopping existing RTC"
            self.stop(stopControl=0)
            time.sleep(1)
        #Then start a new one...

        #os.system("coremain -1 -1 %s &"%self.shmPrefix)
        print "Initialising RTC"
        self.configFile=config
        self.initialiseRTC(startCirc)#wait for the rtcParam buffers to be created...
        
        #bufno=self.bufferList.index(self.getInactiveBuffer())
        #self.initialiseBuffer(bufno,config)
        return 0
            
    # def watchStreamThread(self):
    #     """Polls the list of available streams, and starts a new circular buffer thread if one becomes available"""
    #     while self.go:
    #         time.sleep(1)
    #         try:
    #             sl=startStreams.getStreams(self.shmPrefix)#["rtcPxlBuf","rtcCalPxlBuf","rtcCentBuf","rtcMirrorBuf","rtcActuatorBuf","rtcStatusBuf","rtcTimeBuf","rtcErrorBuf","rtcSubLocBuf","rtcCorrBuf","rtcGenericBuf","rtcFluxBuf"]
    #             sock=[]
    #             for s in sl:
    #                 if s not in self.streamList:
    #                     print "Got stream: %s"%s
    #                     #Now remove othre streams of same name...
    #                     for k in self.pipeDict.keys():
    #                         if self.pipeDict[k][1]==s:
    #                             del(self.pipeDict[k])
    #                     r,w,infoDict=self.createCircBufThread(s,self.circBufDict)#self.circBufDict[key])
    #                     #and add this new stream.
    #                     self.pipeDict[r]=(w,s,self.circBufDict,infoDict)
    #                     sock.append(r)
    #             self.streamList=sl
    #             if self.sockConn!=None:
    #                 for s in sock:
    #                     if s not in self.sockConn.userSelList:
    #                         self.sockConn.userSelList.append(s)
    #                     if s not in self.sockConn.selIn:
    #                         self.sockConn.selIn.append(s)
    #                 #Somehow need to wake sockConn from the select loop...
    #                 os.write(self.wakePipe,"a")

    #         except:
    #             print "Error in watchStreamThread... (ignored)"
    #             traceback.print_exc()

    def streamCreated(self,name):
        #print "File /dev/shm/%s has appeared"%name
        try:
            if name not in self.streamList:
                print "Got stream: %s"%name
                #now remove other streams of same name
                for k in self.pipeDict.keys():
                    if self.pipeDict[k][1]==name:
                        del(self.pipeDict[k])
                self.streamList.append(name)
                if self.circBufDict.has_key(name):
                    del(self.circBufDict[name])
                self.circBufDict[name]=buffer.Circular("/"+name)
                if "rtcErrorBuf" in name:
                    r,w,infoDict=self.createCircBufThread(name,self.circBufDict)
                    # add this new stream
                    self.pipeDict[r]=(w,name,self.circBufDict,infoDict)
                    if self.sockConn!=None:
                        if r not in self.sockConn.userSelList:
                            self.sockConn.userSelList.append(r)
                        if r not in self.sockConn.selIn:
                            self.sockConn.selIn.append(r)
                #now inform anything doing paramWatch...
                t1=time.time()
                for tag in self.paramChangedSubscribers["tag"].keys():
                    mysub=self.paramChangedSubscribers["tag"][tag]
                    mysub.cond.acquire()#get the lock
                    try:
                        mysub.streamsChanged=1
                        mysub.cond.notify()
                        mysub.notifytime=t1
                        mysub.notifycnt+=1
                        if mysub.notifycnt>10 and mysub.notifytime-mysub.time>60:
                            # Client not responding - so remove
                            del(self.paramChangedSubscribers["tag"][tag])
                            print "Removing param notification %d"%tag
                    except:
                        mysub.cond.release()
                        raise
                    mysub.cond.release()


        except:
            print "Error in streamCreated for %s (ignored)"%name
            traceback.print_exc()
    def streamRemoved(self,name):
        print "Removed %s"%name
        if self.circBufDict.has_key(name):
            del(self.circBufDict[name])
        if name in self.streamList:
            self.streamList.remove(name)
            for sock in self.pipeDict.keys():
                if self.pipeDict[sock][1]==name:
                    del(self.pipeDict[sock])
                    if self.sockConn!=None:
                        if sock in self.sockConn.userSelList:
                            self.sockConn.userSelList.remove(sock)
                        if sock in self.sockConn.selIn:
                            self.sockConn.selIn.remove(sock)
            if self.circBufDict.has_key(name):
                del(self.circBufDict[name])

    def getStreams(self):
        """Return a list of streams"""
        return self.streamList

    def newConnection(self,s,suberr=1):
        """Someone else has just connected"""
        #print "Subscribing new connection to rtcErrorBuf"
        if suberr:
            self.subscribe(s,self.shmPrefix+'rtcErrorBuf',1)#subscribe them to the error messages
            for err in self.errList:
                #try:
                #    #print "Sending %s"%str(err)
                #    serialise.Send(["data","rtcErrorBuf",(err,0,0)],s)
                #except:
                #    print "Failed to send existing error %s to new connection"%err
                l=[]
                serialise.SerialiseToList(["data",self.shmPrefix+"rtcErrorBuf",(err,0,0)],l)
                if self.sockConn.writeDict.has_key(s):
                    self.sockConn.writeDict[s]+=l
                else:
                    self.sockConn.writeDict[s]=l
                if self.logread!=None:
                    l=[]
                    serialise.SerialiseToList(["data","%srtclog"%self.shmPrefix,(self.logread.getTxt(),0,0)],l)
                    self.sockConn.writeDict[s]+=l
                if self.ctrllogread!=None:
                    l=[]
                    serialise.SerialiseToList(["data","%sctrllog"%self.shmPrefix,(self.ctrllogread.getTxt(),0,0)],l)
                    self.sockConn.writeDict[s]+=l

        #inform of the current RTC decimation rates...
        #self.setRTCDecimation(None,None)
        #print "Decimation sent"
        
    def handlePipe(self,s):
        """A read on one of the pipes - which means that circ buffer data is ready.
        """
        if s==self.watcher.myfd():#new stream created, or one deleted.
            self.watcher.readEvents()
            self.watcher.processEvents()#this will call streamCreated or streamRemoved.
        else:
            try:
                txt=os.read(s,1)
            except:
                print "Error reading pipe..."
                return False
            if txt=="" or txt=="c":
                print "Closing pipe"
                try:
                    os.close(s)
                except:
                    print "Close pipe failed - ignoring"
                return False
            if self.pipeDict.has_key(s):
                w,key,circBufDict,infoDict=self.pipeDict[s]
            elif s==self.wakePipeEnd:
                return True
            else:
                print "Key for pipe not found - removing."
                return False
            #now send the data to the client...
            remlist=[]
            #print "handlepipe"
            if key==self.shmPrefix+"rtcErrorBuf":
                if infoDict["data"]==None:
                    return False
                err=infoDict["data"].tostring()
                try:
                    indx=err.index('\0')
                    err=err[:indx]
                except:
                    pass
                print "Appending %s error list %s"%(err,self.errList)
                self.errList.append(err)
                #print "Appended data",infoDict["data"]
            #First send to the data switch, and then see if any clients are attached.
            #self.sendDataSwitch(infoDict,circBufDict)

            for sock in infoDict["subscribers"].keys():#self.sockConn.selIn:
                #check that the socket is still open...
                #if type(sock)==type(""):#probably the data switch...
                #    pass
                if type(sock._sock)==socket._closedsocket:#socket has been closed.
                    print "Removing closed socket from subscribe list"
                    del(infoDict["subscribers"][sock])
                else:
                    sub=infoDict["subscribers"][sock]
                    sub.cumfreq+=circBufDict[key].freq[0]
                    if sub.cumfreq>=sub.freq:
                        sub.cumfreq=0
                        if sock not in [self.sockConn.lsock,sys.stdin,self.sockConn.fsock]+self.sockConn.userSelList:#a client
                            # print "Sending %s to client %s"%(key,sock)
                            #try:
                            #    serialise.Send(["data",key,(infoDict["data"],infoDict["timestamp"],infoDict["frameno"])],sock)
                            #except:
                            #    print "Serialise.send failed sending %s to %s in control.py"%(key,sock)
                            #    traceback.print_exc()
                            #    remlist.append(sock)
                            l=[]
                            serialise.SerialiseToList(["data",key,(infoDict["data"],infoDict["timestamp"],infoDict["frameno"])],l)
                            if self.sockConn.writeDict.has_key(sock):
                                self.sockConn.writeDict[sock]+=l
                            else:
                                self.sockConn.writeDict[sock]=l
            for sock in remlist:
                try:
                    self.sockConn.selIn.remove(sock)
                except:
                    print "unable to remove %s - not in list"%str(sock)
                self.subscribe(sock,key,0)
            try:
                os.write(w,"o")
            except:
                print "Error writing pipe"

    def PSDecimateHandler(self,d={}):
        """This is called when decimate values have been changed.
        """
        print "Decimates",d
        if self.dsConfig==None:
            self.dsConfig=PSuser.DecimateConfig()
        for k in d.keys():
            found=0
            for dce in self.dsConfig.generic:
                if dce.name==k:#update the decimate
                    dce.decimate1=int(d[k])
                    found=1
                    break
            if found==0:
                self.dsConfig.generic.append(PSuser.DecimateConfigEntry(name=k,decimate1=d[k]))
        self.dsConfigCallback(None,self.dsConfig)


    def PSConnect(self):
        print "Attempting to connect to PS"
        self.PSname=PSuser.DATAOBJ#"rtc"
        if self.shmPrefix!="":
            self.PSname=self.shmPrefix
        try:
            self.PSClient=PS.getPS(self.PSname)
            self.DecimateHandlerObject=PSuser.DictionaryHandler(self.PSDecimateHandler)
            PS.addSubscriber(self.PSname,self.DecimateHandlerObject,"Decimates")#subscribe to the decimate values...
        except:
            traceback.print_exc()
            print "Couldn't connect to PS name %s"%self.PSname
            self.PSClient=None
        if self.PSClient!=None:
            print "Connected to PS %s"%self.PSname
        else:
            self.DecimateHandlerObject=None
            if self.reconnectRunning==0:
                print "Starting reconnect thread"
                thread.start_new_thread(self.reconnectPS,())
    def reconnectPS(self):
        self.reconnectRunning=1
        while self.PSClient==None:
            time.sleep(10)
            if self.PSClient==None:
                self.PSConnect()
        self.reconnectRunning=0
    def connectDataSwitch(self):
        print "Attempting to connect to DataSwitch"
        try:
            self.serverObject=DataSwitch.connect()#"DataSwitch")
        except:
            print "Couldn't connectot  DataSwitch"
            self.serverObject=None
        if self.serverObject!=None:
            print "subscribing to config"
            try:
                #self.serverObject[0].subscribeConfig(config_handler_i(callback=self.dsConfigCallback)._this())
                ch=config_handler_i(self.serverObject,callback=self.dsConfigCallback)
                print "made config handler"
                t=ch._this()
                print "got this"
                self.serverObject.subscribeConfig(t)
                print "subscribed"
            except:
                print "Couldn't subscribe to config - disconnecting from dataswitch"
                print sys.exc_info()
                self.serverObject=None
        if self.serverObject!=None:
            print "Connected to dataswitch"
        else:
            print "Failed to connect to dataswitch"
            if self.reconnectRunning==0:
                print "Starting reconnect thread"
                thread.start_new_thread(self.reconnectDS,())
    def reconnectDS(self):
        """Attempt to connect to the dataswitch.
        """
        self.reconnectRunning=1
        while self.serverObject==None:
            time.sleep(10)
            if self.serverObject==None:
                self.connectDataSwitch()
        self.reconnectRunning=0
            
    def dsConfigCallback(self,msg,config):
        """Callback called when config on the dataswitch changes.
        config.generic is a list of objects, o, which have:
        o.name - string
        o.decimate1 - int
        o.decimate2 - int
        o.logfile - string
        o.log - bool
        The only thing we care about here is decimate1.
        """
        print "dsConfigCallback",msg,config,self.streamList
        self.dsConfig=config
        update=0
        haserr=0
        for obj in config.generic:
            if obj.name in self.streamList:
                #this is the config object for this stream...
                #Set the decimate (nothing done if already at this value)
                self.setRTCDecimation(obj.name,obj.decimate1)
            if obj.name=="%srtcErrorBuf"%self.shmPrefix:
                haserr=1
        if haserr==0:#no config for errors - put it in...
            if DataSwitch!=None:
                gc=DataSwitch.DataSwitchModule.GenericConfig("%srtcErrorBuf"%self.shmPrefix,1l,1l,"%srtcErrorBuf.log"%self.shmPrefix,0)
            else:
                gc=PSuser.DecimateConfigEntry("%srtcErrorBuf"%self.shmPrefix,1l,1l,"%srtcErrorBuf.log"%self.shmPrefix,0)
            self.dsConfig.generic.append(gc)
            print self.dsConfig
            print "Adding %srtcErrorBuf to dataswitch config."%self.shmPrefix
            if DataSwitch!=None and self.serverObject==None:
                self.connectDataSwitch()
            if self.serverObject!=None:
                try:
                    self.serverObject.publishConfig(self.dsConfig)
                except:
                    self.serverObject=None
                    traceback.print_exc()
                if self.serverObject==None:#retry to connect....
                    self.connectDataSwitch()
                    if self.serverObject!=None:
                        try:
                            self.serverObject.publishConfig(self.dsConfig)
                        except:
                            self.serverObject=None
                            traceback.print_exc()
            if PS!=None and self.PSClient==None:
                self.PSConnect()
            if self.PSClient!=None:
                try:
                    self.PSClient.publishDictionaryStream(PS.DictionaryStream([x.name for x in self.dsConfig.generic],[str(x.decimate1) for x in self.dsConfig.generic]),"Decimates")
                except:
                    self.PSClient=None
                    traceback.print_exc()
                if self.PSClient==None:#try to reconnect
                    self.PSConnect()
                    if self.PSClient!=None:
                        try:
                            self.PSClient.publishDictionaryStream(PS.DictionaryStream([x.name for x in self.dsConfig.generic],[str(x.decimate1) for x in self.dsConfig.generic]),"Decimates")
                        except:
                            self.PSClient=None
                            traceback.print_exc()

                    
            print "Added %srtcErrorBuf to dataswitch config"%self.shmPrefix

    def removeError(self,err):
        errno=0
        if err==None or len(err)==0:#remove all errors
            self.errList=[]
            errno=0x7fffffff
        elif err in self.errList:
            self.errList.remove(err)
        else:
            print "error %s not found in control ErrorList: %s"%(err,str(self.errList))
        #errno=None#1,2,4,8,16,32,etc
        if err=="Maximum clipping exceeded":
            errno=1
        elif err=="Error in parameter buffer":
            errno=2
        elif err=="Error opening camera":
            pass#no error needs clearing in rtc
        elif err=="Error opening mirror":
            pass#no error needs clearing in rtc
        elif err=="Error framing camera":
            pass#no error needs clearing in rtc
        elif err=="Error - camera frames not in sync":
            errno=4
        elif err=="Error - getting camera pixels" or err=="Error - getting camera centroids":
            errno=8
        elif err=="Slope outside calibration range":
            errno=16
        if errno!=0:
            self.set("clearErrors",errno,update=1)
            #b=self.getInactiveBuffer()
            #b.set('clearErrors',errno)
            #self.setSwitchRequested(wait=1)
            #self.copyToInactive()

    def createCircBufThread(self,key,circBufDict):
        """Create a thread which blocks on the circular buffer, waiting for data to become ready, and then writes to a pipe when it is.
        b is the circular buffer object in question.
        """
        infoDict={"data":None,"timestamp":0.,"buffer":None,"frameno":0,"subscribers":{},"key":key}
        r2,w=os.pipe()
        infoDict["rpipe"]=r2
        infoDict["wpipe2"]=w
        r,w2=os.pipe()
        infoDict["wpipe"]=w2
        infoDict["rpipe2"]=r
        t=threading.Thread(target=self.circBufThread,args=(infoDict,circBufDict,key))
        t.daemon=True
        t.start()
        #thread.start_new_thread(self.circBufThread,(infoDict,circBufDict,key))
        #wait for the thread to have attempted to open the circular buffer...
        tmp=os.read(infoDict["rpipe2"],1)
        return r,w,infoDict
        
    def circBufThread(self,infoDict,circBufDict,key):
        """Run as a separate thread, waiting for data from the circular buffer"""
        #first, open the buffer...
        done=0
        firsttime=1
        while done==0:
            try:
                #circBufDict[key]=buffer.Circular("/"+self.shmPrefix+key)
                mybuf=buffer.Circular("/"+key)
                circBufDict[key]=mybuf
                done=1
            except:
                time.sleep(0.01)
            if firsttime:
                firsttime=0
                msg="%s"%done
                os.write(infoDict["wpipe"],msg[:1])
        infoDict["buffer"]=mybuf#circBufDict[key]
        print "Got circular buffer %s"%(key)
        #infoDict=infoDict[0]#unpack the tuple
        #b=infoDict["buffer"]
        wpipe=infoDict["wpipe"]
        rpipe=infoDict["rpipe"]
        if mybuf.freq[0]!=0:#circBufDict[key].freq[0]!=0:
            self.rtcDecimation[key]=int(mybuf.freq[0])#circBufDict[key].freq[0])
            print "Got decimation %d for %s"%(self.rtcDecimation[key],key)
        elif self.rtcDecimation.has_key(key):
            #circBufDict[key].freq[0]=self.rtcDecimation[key]
            mybuf.freq[0]=self.rtcDecimation[key]
        try:
            while self.circgo:
                #wait for data to be ready
                #print "Waiting for data",b
                if self.readStreams or key==self.shmPrefix+"rtcErrorBuf":
                    ret=mybuf.getNextFrame()#this blocks - and if the buffer is removed, blocks forever...
                    if key==self.shmPrefix+"rtcTimeBuf":
                        ret=mybuf.data[:,0],ret[1],ret[2]
                    if ret==None:
                        print "Got None"
                    else:
                        data,timestamp,frameno=ret
                        #print "got data at %s %s %s"%(str(timestamp),str(data.shape),str(data.dtype.char))
                        #store the data
                        if type(infoDict["data"])==numpy.ndarray and infoDict["data"].shape==data.shape and infoDict["data"].dtype.char==data.dtype.char:
                            #print "putting"
                            infoDict["data"][:]=data
                        else:
                            #print "copying"
                            infoDict["data"]=data.copy()
                        infoDict["timestamp"]=timestamp
                        infoDict["frameno"]=frameno
                        #alert the main thread
                        os.write(wpipe,"n")
                        #wait for response
                        os.read(rpipe,1)
                else:
                    time.sleep(1)
        except:
            print "Traceback from thread %s"%key
            traceback.print_exc()
        print "Ending circBufThread %s"%key
        try:
            self.streamList.remove(key)
        except:
            print "Unable to remove %s from streamList %s"%(key,self.streamList)
        #os.write(wpipe,"c")
        os.close(wpipe)
        os.close(rpipe)
        os.close(infoDict["wpipe2"])
        os.close(infoDict["rpipe2"])
        print "Pipes closed for %s"%key
        
    def subscribe(self,sock,key,freq):
        """freq is the desired frequency (decimate rate)
        sock can be the socket or "dataswitch" (though the later is now obsolete)
        """
        done=0
        for k in self.pipeDict.keys():
            infoDict=self.pipeDict[k][3]
            if infoDict["key"]==key:
                #print infoDict["subscribers"]
                if freq<=0:#remove ourself from the subscribe list
                    if sock in infoDict["subscribers"].keys():
                        del(infoDict["subscribers"][sock])
                    if len(infoDict["subscribers"])==0 and key!=self.shmPrefix+"rtcTimeBuf":#stop the circ buffer
                        #self.setRTCDecimation(key,0)
                        pass#100918 - no longer turn off the decimation
                else:
                    if sock not in infoDict["subscribers"].keys():
                        infoDict["subscribers"][sock]=Subscriber(sock,freq)
                    else:#update the frequency
                        sub=infoDict["subscribers"][sock]
                        sub.freq=freq
                    if self.circBufDict.has_key(key):
                        if self.circBufDict[key].freq[0]==0:#if not already reading...
                            if key==self.shmPrefix+"rtcTimeBuf":
                                freq=1
                            self.setRTCDecimation(key,freq)
                    else:
                        print "Circ buf not yet started - not setting decimate"
                done=1
                break
        if done==0:
            print "Unable to find stream %s"%key

    def setRTCDecimation(self,key,freq):
        """Set the RTC decimate function for key to freq"""
        update=0
        #print "setRTCDecimation %s %d"%(key,freq)
        if key!=None:
            self.rtcDecimation[key]=freq
            if not self.circBufDict.has_key(key):
                print "CircBufDict doesn't have key %s"%(key)
            else:
                if self.circBufDict[key].freq[0]!=freq:
                    print "Setting decimate %s to %s"%(key,str(freq))
                    if type(freq)==type(0):
                        self.circBufDict[key].freq[0]=freq
                    elif type(freq)==type({}):
                        self.circBufDict[key].freq[0]=freq[key]
                    else:
                        print "Dont know type of freq: %s"%str(freq)
                        raise Exception("Dont know type of freq: %s"%str(freq))
                    update=1
        else:
            update=1
        #and now update any subscribers that this change has been made...
        if update:
            d={}
            for key in self.circBufDict.keys():
                d[key]=self.circBufDict[key].freq[0]
            for s in self.sockConn.selIn:
                if s not in [self.sockConn.lsock,sys.stdin,self.sockConn.fsock]+self.sockConn.userSelList:#a client
                    #try:
                    #    serialise.Send(["data","rtcDecimateVals",d],s)
                    #except:
                    #    print "Failed to send RTC decimation to sock %s"%s
                    l=[]
                    serialise.SerialiseToList(["data",self.shmPrefix+"rtcDecimateVals",d],l)
                    if self.sockConn.writeDict.has_key(s):
                        self.sockConn.writeDict[s]+=l
                    else:
                        self.sockConn.writeDict[s]=l
                    #print "Added to writeDict"
    def getRTCDecimation(self,key=None):
        """Get the RTC decimate value"""
        #key=self.shmPrefix+key
        if key==None:
            key=self.circBufDict.keys()
        elif type(key)!=type([]):
            key=[key]
        rt={}
        for k in key:
            rt[k]=int(self.circBufDict[k].freq[0])
            self.rtcDecimation[k]=rt[k]
        return rt#int(self.circBufDict[key].freq[0])

    def setDecimation(self,name,d1,d2=None,log=0,fname=""):
        """Called by corba... allow the user to turn streams on and off
        name should include shmPrefix
        """
        #name=self.shmPrefix+name
        # if d2==None:
        #     d2=d1
        self.setRTCDecimation(name,d1)
        # if self.dsConfig!=None:
        #     update=0
        #     found=0
        #     for obj in self.dsConfig.generic:#first see if in the config
        #         if obj.name==name:#self.shmPrefix+name:
        #             found=1
        #             if obj.decimate1!=d1 or obj.decimate2!=d2 or obj.log!=log or obj.logFile!=fname:
        #                 obj.decimate1=d1
        #                 obj.decimate2=d2
        #                 obj.log=log
        #                 obj.logFile=fname
        #                 update=1
        #     if found==0:#not in config, so add it.
        #         if DataSwitch!=None:
        #             gc=DataSwitch.DataSwitchModule.GenericConfig(name,long(d1),long(d2),fname,log)
        #         else:
        #             gc=PSuser.DecimateConfigEntry(name,long(d1),long(d2),fname,log)
        #         self.dsConfig.generic.append(gc)
        #     if DataSwitch!=None and self.serverObject==None:
        #         self.connectDataSwitch()
        #     if self.serverObject!=None:
        #         try:
        #             self.serverObject.publishConfig(self.dsConfig)
        #         except:
        #             self.serverObject=None
        #             traceback.print_exc()
        #         if self.serverObject==None:#retry to connect....
        #             self.connectDataSwitch()
        #             if self.serverObject!=None:
        #                 try:
        #                     self.serverObject.publishConfig(self.dsConfig)
        #                 except:
        #                     self.serverObject=None
        #                     traceback.print_exc()
                                
        #     if PS!=None and self.PSClient==None:
        #         self.PSConnect()
        #     if self.PSClient!=None:
        #         try:
        #             self.PSClient.publishDictionaryStream(PS.DictionaryStream([x.name for x in self.dsConfig.generic],[str(x.decimate1) for x in self.dsConfig.generic]),"Decimates")
        #         except:
        #             self.PSClient=None
        #             traceback.print_exc()
        #         if self.PSClient==None:#try to reconnect
        #             self.PSConnect()
        #             if self.PSClient!=None:
        #                 try:
        #                     self.PSClient.publishDictionaryStream(PS.DictionaryStream([x.name for x in self.dsConfig.generic],[str(x.decimate1) for x in self.dsConfig.generic]),"Decimates")
        #                 except:
        #                     self.PSClient=None
        #                     traceback.print_exc()




    def loop(self):
        """Does nothing"""
        self.sockConn.loop()

    def swap(self,n1,n2):
        b=self.getInactiveBuffer()
        if b==None:
            raise Exception("No inactive buffer found")
        b.swap(n1,n2)

    def getActiveBufferArray(self):
        """Returns the buffer array of the active buffer..."""
        b=self.getActiveBuffer()
        buf=None
        if b!=None:
            try:
                sr=b.get("switchRequested")
            except:
                sr=0
            if sr:#the buffer is about to swap, so get the other one.
                b=self.bufferList[1-self.bufferList.index(b)]
            else:
                b=self.getActiveBuffer()#just incase the swap has been done before we checked.
            buf=b.arr.view('b')[:b.getMem(1)]
        return buf

    def getActiveBuffer(self):
        """Return the buffer that is currently active"""

        if self.bufferList==None:
            return None
        if self.bufferList[0].flags[0]&0x1==1:
            return self.bufferList[0]
        elif self.bufferList[1].flags[0]&0x1==1:
            return self.bufferList[1]
        else:
            if self.nodarc:#not runnign darc - probably only one buffer anyway
                return self.bufferList[0]
            print "No active buffer",self.bufferList[0].flags,self.bufferList[1].flags
            return None
        # if utils.getSemValue(self.bufferList[0].semid,0)==1:
        #     return self.bufferList[0]
        # elif utils.getSemValue(self.bufferList[1].semid,0)==1:
        #     return self.bufferList[1]
        # else:
        #     print "No active buffer"
        #     return None

    def getInactiveBuffer(self):
        """Return the buffer that can be safely written to"""
        if self.bufferList==None:
            print "No buffers in getInactiveBuffer"
            return None
        if self.bufferList[0].flags[0]&0x1==0:
            return self.bufferList[0]
        elif self.bufferList[1].flags[0]&0x1==0:
            return self.bufferList[1]
        else:
            if self.nodarc:
                return self.bufferList[0]
            print "No inactive buffer"
            return None


    def getLabels(self):
        b=self.getActiveBuffer()
        return b.getLabels()

    def copyToInactive(self):
        """Copy from active to inactive buffer.  Here, we assume that the active buffer is correct, so can just copy the buffer contents without any checking."""
        ac=self.getActiveBuffer()
        inac=self.getInactiveBuffer()
        #Note - we don't copy the buffer header.
        inac.buffer.view("b")[:]=ac.buffer.view("b")

    def stop(self,stopRTC=1,stopControl=1):
        if stopRTC:
            self.circgo=0#stop the circular buffer threads...
            buflist=self.bufferList
            if self.bufferList!=None:
                try:
                    b=self.getInactiveBuffer()
                except:
                    b=None
                if b!=None:
                    active=self.bufferList[0]
                    if b==active:
                        active=self.bufferList[1]
                    b.set("go",0)
                    self.paramChangedDict["go"]=(0,"")
                    self.setSwitchRequested(preservePause=0)
                    active.set("go",0,ignoreLock=1)
                self.bufferList=None
            self.rtcStopped=1
            self.publishParams()
            # remove the shm
            print "Removing SHM"
            if os.path.exists("/dev/shm/%srtcParam1"%self.shmPrefix):
                os.unlink("/dev/shm/%srtcParam1"%self.shmPrefix)
            if os.path.exists("/dev/shm/%srtcParam2"%self.shmPrefix):
                os.unlink("/dev/shm/%srtcParam2"%self.shmPrefix)
            d=os.listdir("/dev/shm")
            for f in d:
                if f.startswith("%srtc"%self.shmPrefix) and f.endswith("Buf"):
                    os.unlink("/dev/shm/%s"%f)
            print self.circBufDict
            for k in self.pipeDict.keys():
                if k in self.sockConn.selIn:
                    self.sockConn.selIn.remove(k)
                if k in self.sockConn.userSelList:
                    self.sockConn.userSelList.remove(k)
            #self.pipeDict={}

            #No need to remove mutexes and condition variables created by the rtc - it should remove them itself - for the param buffers and for the circular buffers - when it removes the shm
            #self.circBufDict={}
        if stopControl:
            self.sockConn.endLoop()
            self.go=0
            
            
    def togglePause(self,p=1,wait=0):
        b=self.getInactiveBuffer()
        #b2=self.getActiveBuffer()
        b.set("pause",p)
        self.paramChangedDict["pause"]=(p,"")
        self.setSwitchRequested(preservePause=0,wait=wait)
        #print "done switch"
        #b2.set("pause",p)#this blocks until we are allowed to write to buffer.
        #print "set spare buffer"
        #for b in self.bufferList:
        #    b.setControl("pause",p)
        return p

    def setCloseLoop(self,p=1,wait=1):
        self.set("closeLoop",p,update=1,wait=wait)
        #b=self.getInactiveBuffer()
        #b.set("closeLoop",p)
        #self.setSwitchRequested(wait=wait)
        #self.copyToInactive()
        return p

    def connectParamSubscriber(self,host,port,plist):
        """Connect to a client who wants to be notified of param changes"""
        sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        try:
            sock.connect((host,port))
            print "Connected to param subscriber"
        except:
            print "Couldn't connect to param subscriber on %s %d"%(host,port)
            sock.close()
            sock=None
            raise Exception("Couldn't connect to param subscriber on %s %d"%(host,port))
        if sock!=None:
            self.paramChangedSubscribers[sock]=plist

    def informParamSubscribers(self,pdict=None):
        """Users can subscribe to be notified if parameters change.  
        This informs them."""
        frameno=0
        switchTime=0.
        if pdict==None:
            pdict=self.paramChangedDict
            self.paramChangedDict={}#reset for next time.
            try:
                switchiter=self.getActiveBuffer().get("frameno")
                switchTime=float(self.getActiveBuffer().get("switchTime"))
            except:
                if self.nodarc==0:
                    print "Unable to get switch iteration"
        changed=pdict.keys()
        for s in self.paramChangedSubscribers.keys():
            if s!="tag":#these are the socket based subscribers
                if self.paramChangedSubscribers[s]==None or len(self.paramChangedSubscribers[s])==0:#send everything.
                    try:
                        serialise.Send(["params",switchiter,switchTime,pdict],s)
                    except:
                        print "Error sending params to %s - closing connection"%str(s)
                        try:
                            s.close()
                        except:
                            pass
                        del(self.paramChangedSubscribers[s])
                else:#only send what the subscriber wants to see.
                    d={}
                    for l in self.paramChangedSubscribers[s]:
                        if l in changed:
                            d[l]=pdict[l]
                    if len(d)>0:
                        try:
                            serialise.Send(["params",switchiter,switchTime,d],s)
                        except:
                            print "Error sending params to %s: closing connection"%str(s)
                            try:
                                s.close()
                            except:
                                pass
                            del(self.paramChangedSubscribers[s])
        t1=time.time()
        for tag in self.paramChangedSubscribers["tag"].keys():
            changed=pdict.keys()
            mysub=self.paramChangedSubscribers["tag"][tag]
            mysub.cond.acquire()#get the lock
            try:
                if mysub.params==None or len(mysub.params)==0:#inform about everything.
                    #Each tag has a condition variable associated with it - so here, we put whatever has changed into a dict and signal the condition variable.
                    #The thread that is then awoken will clear the dictionary and return.
                    #now append the changed variables
                    for c in changed:
                        if c not in mysub.changed:
                            mysub.changed.append(c)
                    mysub.cond.notify()
                    mysub.notifytime=t1
                    mysub.notifycnt+=1
                else:#inform only about specific params
                    cnt=0
                    for c in mysub.params:
                        #c is what we are listening for
                        if (c not in mysub.changed) and (c in changed):
                            mysub.changed.append(c)
                            cnt+=1
                    if cnt>0:#time to wake the process
                        mysub.cond.notify()
                        mysub.notifytime=t1
                        mysub.notifycnt+=1
                if mysub.notifycnt>10 and mysub.notifytime-mysub.time>60:
                    #Client hasn't responded for more than 60s - so remove
                    del(self.paramChangedSubscribers["tag"][tag])
                    print "Removing param notification %d"%tag
            except:
                mysub.cond.release()
                raise
            mysub.cond.release()
                    
    def newParamTag(self):
        while self.paramChangedSubscribers["tag"].has_key(self.paramTagRef):
            self.paramTagRef+=1
        tag=self.paramTagRef
        self.paramTagRef+=1
        return tag

    def watchParam(self,tag,paramList=None,timeout=None):
        """If paramList==[], subscribe to all params.
        If paramList==None, use the same subscription as last time (or everything if this is the first time).
        It will return when a parameter changes
        It will also return if the streams change.
        """
        if not self.paramChangedSubscribers["tag"].has_key(tag):
            if paramList==None:
                paramList=[]#everything
            print "Creating subscribeObject"
            self.paramChangedSubscribers["tag"][tag]=subscribeObject(paramList)
            paramList=None
        mysub=self.paramChangedSubscribers["tag"][tag]
        changed=[]
        mysub.cond.acquire()
        try:
            mysub.time=time.time()#the time last requested
            mysub.notifycnt=0
            if paramList!=None:
                mysub.params=paramList
            if mysub.streamsChanged:#streams have changed so return immediately
                changed=[]
                mysub.streamsChanged=0
            elif len(mysub.changed)>0:
                #can return instantly...
                changed=mysub.changed
                mysub.changed=[]
            else:#need to wait for a change
                if timeout<0:
                    timeout=None
                mysub.cond.wait(timeout)
                if mysub.streamsChanged:
                    changed=[]
                    mysub.streamsChanged=0
                else:
                    changed=mysub.changed
                    mysub.changed=[]
        except:
            mysub.cond.release()
            raise
        mysub.cond.release()
        return changed



    def setSwitchRequested(self,preservePause=1,wait=0):
        if preservePause:
            try:
                p=self.getActiveBuffer().get("pause")
            except:
                p=1
            self.getInactiveBuffer().set("pause",p)
        active=self.getActiveBuffer()
        inactive=self.getInactiveBuffer()
        active.setControl('switchRequested',1)
        if wait:#we wait until the switch has completed.
            while active.flags[0]&1==1:
                print "Waiting for inactive buffer"
                utils.pthread_mutex_lock(active.condmutex)
                try:
                    t=utils.pthread_cond_timedwait(active.cond,active.condmutex,1,1)
                except:
                    traceback.print_exc()
                    print "Error in utils.pthread_cond_timedwait - continuing"
                utils.pthread_mutex_unlock(active.condmutex)
                if t:
                    print "Timeout while waiting - active flag now %d, inactive %d"%(inactive.flags[0]&1,active.flags[0]&1)
            print "Got inactive buffer"
            #utils.semop(active.semid,0,0)#wait for the buffer to be unfrozen.
            #while active==self.getActiveBuffer():
            #    time.sleep(0.05)
        self.informParamSubscribers()
        if preservePause:
            self.publishParams()

    def publishParams(self):
        """Send current active param buf to the dataswitch"""
        if DataSwitch!=None and self.serverObject==None:
            self.connectDataSwitch()
        if PS!=None and self.PSClient==None:
            self.PSConnect()

        if self.serverObject!=None or self.PSClient!=None:#Need to publish the parameter buffer.
            print "Publishing param buffer"
            b=self.getActiveBuffer()
            try:
                ftime=float(b.get("switchTime"))
                fno=int(b.get("frameno"))
            except:
                ftime=0.
                fno=0
            if b==None:#no active buffer...
                arr=numpy.zeros((0,),'b')
            else:
                arr=numpy.array(b.arr.view('b')[:b.getMem(1)])
            if DataSwitch!=None:
                a=DataSwitch.DataSwitchModule.Generic(1,arr.dtype.char,fno,ftime,1,arr.shape,arr.size,arr.tostring())
                try:
                    self.serverObject.publishGeneric(a,self.shmPrefix+"rtcParam")
                except:
                    self.serverObject=None
                    traceback.print_exc()
                if self.serverObject==None:#retry to connect.
                    self.connectDataSwitch()
                    if self.serverObject!=None:
                        try:
                            self.serverObject.publishGeneric(a,self.shmPrefix+"rtcParam")
                        except:
                            self.serverObject=None
                            traceback.print_exc()
            else:
                a=PS.DataStream(fno,ftime,arr.dtype.char,arr.shape,[],arr.tostring())
                try:
                    self.PSClient.publishDataStream(a,self.shmPrefix+"rtcParam")
                except:
                    self.PSClient=None
                    traceback.print_exc()
                if self.PSClient==None:#try to reconnect
                    self.PSConnect()
                    if self.PSClient!=None:
                        try:
                            self.PSClient.publishDataStream(a,self.shmPrefix+"rtcParam")
                        except:
                            self.PSClient=None
                            traceback.print_exc()
        self.conditionVar.acquire()
        self.conditionVar.notifyAll()
        self.conditionVar.release()




    def waitParamChange(self,timeout,paramList=None):
        """This is called without the corba lock"""
        self.conditionVar.acquire()
        if timeout<0:
            timeout=None
        self.conditionVar.wait(timeout)
        self.conditionVar.release()
        
    def remove(self,name,doSwitch=1,copyFirst=1,wait=1):
        """Removes a parameter from the inactive buffer"""
        if copyFirst:
            self.copyToInactive()
        b=self.getInactiveBuffer()
        val=b.remove(name)
        if doSwitch:
            self.setSwitchRequested(wait=wait)
            if wait:
                self.copyToInactive()
        return val

    def set(self,name,val,comment="",inactive=1,check=1,copyFirst=0,update=0,wait=1,usrbuffer=None):
        """Sets a value in a buffer.  
        Note, doesn't do a buffer swap unless update==1.
        If check is set, will check that the value is consistent with what is expected eg size and dtype etc (but not explicit values).
        if usrbuffer!=None, this buffer is used... this buffer is also used for checking...
        """
        if usrbuffer==None:
            buf=self.getActiveBuffer()
            if buf==None:#no active buffer... ???
                print "Set didn't get active buffer for %s"%name
                check=0
            if inactive:
                if copyFirst:
                    self.copyToInactive()
                b=self.getInactiveBuffer()
            else:
                b=self.getActiveBuffer()
        else:
            b=usrbuffer
            buf=usrbuffer
        if check:
            val=self.check.valid(name,val,buf)
        if inactive:
            self.paramChangedDict[name]=(val,comment)
        else:#making a change to active buffer - so tell any listeners...
            self.informParamSubscribers({name:(val,comment)})

        b.set(name,val,comment=comment)
        #if name in ["bgImage","flatField","darkNoise","pxlWeight","rmx","gain","E","thresholdValue","thresholdAlgorithm","subapLocation","subapFlag"]:
        try:
            self.setDependencies(name,b)
        except:
            print "Error setting dependencies"
            traceback.print_exc()
            raise
        if update==1 and inactive==1:
            self.setSwitchRequested(wait=wait)
            if wait:
                self.copyToInactive()


    def getLog(self,getdarc=1,getctrl=1,maxlen=0,minlen=0):
        if getdarc:
            txt=open("/dev/shm/%sstdout0"%self.shmPrefix).read()
            if len(txt)<minlen:#log rotation
                if os.path.exists("/dev/shm/%sstdout1"%self.shmPrefix):
                    txt=open("/dev/shm/%sstdout1"%self.shmPrefix).read()+txt
            if maxlen>0:
                txt=txt[-maxlen:]
            
        else:
            txt=""
        if getctrl:
            if os.path.exists("/dev/shm/%sctrlout0"%self.shmPrefix):
                txt+="\n********* CONTROL OUTPUT *********\n\n"
                tmp=open("/dev/shm/%sctrlout0"%self.shmPrefix).read()
                if len(tmp)<minlen:#log rotation
                    if os.path.exists("/dev/shm/%sctrlout1"%self.shmPrefix):
                        tmp=open("/dev/shm/%sctrlout1"%self.shmPrefix).read()+tmp
                if maxlen>0:
                    tmp=tmp[-maxlen:]
                txt+=tmp
        return txt
    def setDependencies(self,name,b):
        """Value name has just changed in the buffer,  This will require some other things updating.
        """
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
            if ff!=None:ff=ff.copy()
            if bg!=None:bg=bg.copy()
            if dn!=None:dn=dn.copy()
            if wt!=None:wt=wt.copy()
            if type(th)==numpy.ndarray:th=th.copy()
            npxls=(npxlx*npxly).sum()
            if ta==2:#add threshold to background then set thresh to zero
                #note this altered background is only used here for calcs.
                if type(th)==numpy.ndarray:#value per subap
                    if th.size==npxls:#value per pixel
                        if bg==None:
                            bg=th.copy()
                        else:
                            bg+=th
                    else:
                        if bg==None:
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
                    if bg==None and th!=0:
                        bg=numpy.zeros((npxls),numpy.float32)
                    if bg!=None:
                        bg[:]+=th
                        
                calthr=numpy.zeros((npxls),numpy.float32)
            elif ta==1:
                #multiply threshold by weight
                if type(th)==numpy.ndarray:
                    if th.size==npxls: #threshold per pixel
                        if wt==None:
                            calthr=th
                        else:
                            calthr=th*wt
                    else:#threshold per subap
                        calthr=numpy.zeros((npxls),numpy.float32)
                        if wt==None:
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
                    if wt==None:
                        calthr=numpy.zeros((npxls),numpy.float32)
                        calthr[:]=th
                    else:
                        calthr=wt*th
            else:
                calthr=None
            if ff==None:
                if wt==None:
                    calmult=None
                else:
                    calmult=wt
            else:
                if wt==None:
                    calmult=ff
                else:
                    calmult=ff*wt
            #calsub should equal (dn*ff+bg)*wt
            if dn==None:
                if bg==None:
                    calsub=None
                else:
                    if wt==None:
                        calsub=bg
                    else:
                        calsub=bg*wt
            else:
                if ff==None:
                    calsub=dn
                else:
                    calsub=ff*dn
                if bg!=None:
                    calsub+=bg
                if wt!=None:
                    calsub*=wt
            if calsub!=None:calsub=calsub.astype(numpy.float32)
            if calmult!=None:calmult=calmult.astype(numpy.float32)
            if calthr!=None:calthr=calthr.astype(numpy.float32)
            self.paramChangedDict["calsub"]=(calsub,"")
            self.paramChangedDict["calmult"]=(calmult,"")
            self.paramChangedDict["calthr"]=(calthr,"")
            b.set("calsub",calsub)
            b.set("calmult",calmult)
            b.set("calthr",calthr)



        if name in ["gain","E","rmx","gainE","gainReconmxT"]:
            #now update the gainE and gainReconmxT.
            try:
                rmx=b.get("rmx")
                e=b.get("E")
                g=b.get("gain")
            except:
                return

            rmxt=rmx.transpose().copy()
            nacts=g.shape[0]
            for i in range(nacts):
                rmxt[:,i]*=g[i]
            rmxt=rmxt.astype(numpy.float32)
            self.paramChangedDict["gainReconmxT"]=(rmxt,"")
            b.set("gainReconmxT",rmxt)
            
            gainE=e.copy()
            for i in range(nacts):
                gainE[i]*=1-g[i]
            gainE=gainE.astype(numpy.float32)
            self.paramChangedDict["gainE"]=(gainE,"")
            b.set("gainE",gainE)



    def acquireImage(self,nframes,whole=0):
        """Acquire an averaged image.  Return the image to the user.
        If whole is set, it will compute subapLocation such that the whole image is calibrated.  Note - this will mess up centroid measurements, so the loop is opened.  At the end, things are placed back into their existing state.

        """
        self.copyToInactive()
        b=self.getInactiveBuffer()
        self.set("averageImg",nframes,comment="Acquiring image")
        if whole:
            self.computeFillingSubapLocation(updateRTC=1)

        #dec=self.getRTCDecimation("rtcCalPxlBuf")
        #if dec==0:
        #    self.setRTCDecimation("rtcCalPxlBuf",10)
        #Now get the latest averaged image.
        #cb=self.circBufDict["rtcGenericBuf"]
        cb=buffer.Circular("/"+self.shmPrefix+"rtcGenericBuf")
        cb.getLatest()#update counters...
        cb.setForceWrite()

        self.togglePause(0,wait=1)#unpause, and wait for the buffer to swap.
        #The RTC is now averaging...
        print "Getting latest rtcGenericBuf frame"
        img,timestamp,frameno=cb.getNextFrame()
        print "Got img:",img.shape
        if whole:
            self.revertSavedState()
        img=numpy.array(img).astype(numpy.float32)
        return img

    def computeFillingSubapLocation(self,updateRTC=0,b=None):
        """Compute a subapLocation (and pxlcnt) such that all pixels are in a subap.
        If updateRTC is set, this is then sent to the RTC and the loop is opened.
        Typically, this will be used when acquiring calibration images if you want to acquire the whole CCD image - eg to create a background image...
        """
        print "computing filling subap location"
        if b==None:
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
                        npxls=(npxlx-pxldone)/(ncols-k)
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
        if updateRTC:
            self.savedState={}
            self.savedState["subapLocation"]=(b.get("subapLocation").copy(),b.getComment("subapLocation"))
            self.savedState["closeLoop"]=(b.get("closeLoop"),b.getComment("closeLoop"))
            self.savedState["pxlCnt"]=(b.get("pxlCnt").copy(),b.getComment("pxlCnt"))
            pxlCnt=b.get("pxlCnt").copy()
            nsubapsCum=0
            if subapLocationType==0:
                for k in range(ncam):
                    for i in range(nsub[k]):
                        indx=nsubapsCum+i
                        n=(subapLocation[indx,1]-1)*npxlx[k]+subapLocation[indx,4]
                        pxlCnt[indx]=n
                    nsubapsCum+=nsub[k]
            else:
                subapLocation.shape=nsub.sum(),subapLocation.size/nsub.sum()
                for i in range(subapLocation.shape[0]):
                    pxlCnt[i]=numpy.max(subapLocation.shape[i])
            self.set("subapLocation",subapLocation,comment="Set for acquiring image")
            self.set("pxlCnt",pxlCnt,comment="Set for acquiring image")
            self.set("closeLoop",0,comment="Set for acquiring image")
        return subapLocation
    def revertSavedState(self):
        print "Reverting to saved state"
        self.copyToInactive()
        for key in self.savedState.keys():
            self.set(key,self.savedState[key][0],comment=self.savedState[key][1])
        self.savedState={}
        self.setSwitchRequested(wait=1)
        self.copyToInactive()

    def acquireCents(self,nframes):
        """Acquire centroids.  Return the image to the user.
        """
        self.copyToInactive()
        b=self.getInactiveBuffer()
        self.set("averageCent",nframes,comment="Acquiring centroids")
        #dec=self.getRTCDecimation("rtcCalPxlBuf")
        #if dec==0:
        #    self.setRTCDecimation("rtcCalPxlBuf",10)
        #Now get the latest averaged image.
        #cb=self.circBufDict["rtcGenericBuf"]
        cb=buffer.Circular("/"+self.shmPrefix+"rtcGenericBuf")
        cb.getLatest()#update counters...
        cb.setForceWrite()
        print cb.lastReceived,cb.lastWritten
        self.togglePause(0,wait=1)#unpause, and wait for the buffer to swap.
        #The RTC is now averaging...
        print "Getting latest rtcGenericBuf frame"
        img,timestamp,frameno=cb.getNextFrame(timeout=10,retry=-1)
        print "Got cents:",img.shape,frameno
        print cb.lastReceived,cb.lastWritten

        img=numpy.array(img).astype(numpy.float32)
        print img[0]
        return img


    def acquireBackground(self):
        """Acquire a background image, and set it as such in the RTC.  Return the image to the user.
        Note, the dark noise and flatfield have already been applied to this image.
        """
        #Set threshold, powerfactor and pixel weighting to have no effect.
        #Set the background map to zeros.
        #Take an image, and retrieve the calPxlBuf, averaged over a number of frames.  Actually, no - don't do the averaging.
        #This is then the background
        #Set the background
        #Set threshold, powerfactor and pixel weighting to what they were
        #Return the background.
        self.copyToInactive()
        b=self.getInactiveBuffer()
        bg=b.get("bgImage")
        thr=b.get("thresholdAlgo")
        thrcom=b.getComment("thresholdAlgo")
        pow=b.get("powerFactor")
        powcom=b.getComment("powerFactor")
        pxlweight=b.get("pxlWeight")
        pxlweightcom=b.getComment("pxlWeight")
        self.set("thresholdAlgo",0,comment="Acquiring background")
        self.set("powerFactor",1.,comment="Acquiring background")
        self.set("pxlWeight",None,comment="Acquiring background")
        if bg!=None:
            bg[:]=0
            print bg.shape,type(bg)
            self.set("bgImage",bg,comment="Acquiring background")
        self.set("averageImg",10,comment="Acquiring background")
        #dec=self.getRTCDecimation("rtcCalPxlBuf")
        #if dec==0:
        #    self.setRTCDecimation("rtcCalPxlBuf",10)
        #Now get the latest averaged image.
        #cb=self.circBufDict["rtcGenericBuf"]
        cb=buffer.Circular("/"+self.shmPrefix+"rtcGenericBuf")
        cb.getLatest()#update counters...
        cb.setForceWrite()

        self.togglePause(0,wait=1)#unpause, and wait for the buffer to swap.
        #The RTC is now averaging...
        print "Getting latest rtcGenericBuf frame"
        img,timestamp,frameno=cb.getNextFrame()
        print "Got img:",img.shape

        img=numpy.array(img).astype(numpy.float32)
        self.copyToInactive()
        self.set("bgImage",img,comment="Acquired %s"%time.strftime("%y/%m/%d %H:%M:%S"),)
        self.set("thresholdAlgo",thr,comment=thrcom)
        self.set("powerFactor",pow,comment=powcom)
        self.set("pxlWeight",pxlweight,comment=pxlweightcom)
        self.setSwitchRequested(wait=1)
        self.copyToInactive()
        return img

    def acquireBackgroundOld(self):
        """Acquire a background image, and set it as such in the RTC.  Return the image to the user.
        Note, the dark noise and flatfield have already been applied to this image.
        """
        #Set threshold, powerfactor and pixel weighting to have no effect.
        #Set the background map to zeros.
        #Take an image, and retrieve the calPxlBuf, averaged over a number of frames.  Actually, no - don't do the averaging.
        #This is then the background
        #Set the background
        #Set threshold, powerfactor and pixel weighting to what they were
        #Return the background.
        self.copyToInactive()
        b=self.getInactiveBuffer()
        bg=b.get("bgImage")
        thr=b.get("thresholdAlgo")
        thrcom=b.getComment("thresholdAlgo")
        pow=b.get("powerFactor")
        powcom=b.getComment("powerFactor")
        #wei=b.get("")#PIXEL weighting not currently implemented.
        self.set("thresholdAlgo",0,comment="Acquiring background")
        self.set("powerFactor",1.,comment="Acquiring background")
        #self.set("pxlWeighting",XXX,comment="Acquiring background")
        bg[:]=0
        self.set("bgImage",bg,comment="Acquiring background")
        #dec=self.getRTCDecimation("rtcCalPxlBuf")
        #if dec==0:
        #    self.setRTCDecimation("rtcCalPxlBuf",10)
        self.togglePause(0,wait=1)#unpause, and wait for the buffer to swap.
        b=None#no longer the inactive buffer...
        self.copyToInactive()
        #Now get the latest image.
        #cb=self.circBufDict["rtcCalPxlBuf"]
        cb=buffer.Circular("/"+self.shmPrefix+"rtcCalPxlBuf")
        cb.setForceWrite()
        fno2=-1
        latest=cb.getLatest()
        if latest==None:
            fno=-1
        else:
            data,t,fno=latest
        for i in range(10):#wait at most a second for new data
            cb.setForceWrite()
            time.sleep(0.1)
            latest=cb.getLatest()
            if latest!=None:
                data,t,fno2=latest
            if fno2!=fno:
                break
        if fno2==fno:
            raise Exception("Circular buffer calPxlBuf not updating - cannot get bgImage")
        #if dec==0:
        #    self.setRTCDecimation("rtcCalPxlBuf",0)
        data=data.astype(numpy.float32)
        self.set("bgImage",data,comment="Acquired %s"%time.strftime("%y/%m/%d %H:%M:%S"))
        self.set("thresholdAlgo",thr,comment=thrcom)
        self.set("powerFactor",pow,comment=powcom)
        #self.set("pxlWeight",wei,comment=weicom)
        self.setSwitchRequested(wait=1)
        self.copyToInactive()
        

    def setBackground(self,bg):
        self.set("bgImage",bg,comment="set by setBackground %s"%time.strftime("%y/%m/%d %H:%M:%S"),copyFirst=1,update=1)

    def setThreshold(self,thresh):
        self.set("thresholdValue",thresh,comment="set by setthreshold %s"%time.strftime("%y/%m/%d %H:%M:%S"),copyFirst=1,update=1)

    def setFlatfield(self,ff):
        self.set("flatField",ff,comment="set by setFlatfield %s"%time.strftime("%y/%m/%d %H:%M:%S"),copyFirst=1,update=1)

    def setGain(self,gain):
        self.copyToInactive()
        self.set("gain",gain,comment="set by setGain %s"%time.strftime("%y/%m/%d %H:%M:%S"))
        b=self.getInactiveBuffer()
        #print "updating gainReconmxT, gainE"
        rmxt=b.get("rmx").transpose().copy()
        if rmxt.shape[1]==gain.shape[0]:
            for i in range(gain.shape[0]):
                rmxt[:,i]*=gain[i]
            self.set("gainReconmxT",rmxt,comment="set when gain changed by setGain %s"%time.strftime("%y/%m/%d %H:%M:%S"))
        gainE=b.get("E").copy()
        if gainE.shape[0]==gain.shape[0]:
            for i in range(gain.shape[0]):
                gainE[i]*=1-gain[i]
            self.set("gainE",gainE,comment="set when gain changed by setGain %s"%time.strftime("%y/%m/%d %H:%M:%S"))
        self.setSwitchRequested(wait=1)#unpause, and wait for the buffer to swap.
        self.copyToInactive()
        

    def setActBounds(self,vmin,vmax):
        self.copyToInactive()
        self.set("actMin",vmin,comment="set by setActBounds %s"%time.strftime("%y/%m/%d %H:%M:%S"))
        self.set("actMax",vmax,comment="set by setActBounds %s"%time.strftime("%y/%m/%d %H:%M:%S"))
        self.setSwitchRequested(wait=1)
        self.copyToInactive()

    def setActuators(self,acts):
        """set all actuators"""
        self.copyToInactive()
        self.set("actuators",acts,comment="set by setActuators %s"%time.strftime("%y/%m/%d %H:%M:%S"))
        self.setSwitchRequested(wait=1)
        self.copyToInactive()



    def setActuator(self,mode,act,v):
        """set individual actuator."""
        #set an individual actuator
        #First get the current actuator values.
        #Subscribe to rtcActuatorBuf
        #cb=self.circBufDict["rtcActuatorBuf"]
        cb=buffer.Circular("/"+self.shmPrefix+"rtcActuatorBuf")
        cb.setForceWrite()
        acts=None
        for i in range(10):
            latest=cb.getLatest()
            if latest==None:
                time.sleep(0.1)
            else:
                acts,t,fno=latest
                break
        if acts==None:
            raise Exception("Unable to get actuators")
        if mode=="set":
            acts[act]=v
        else:#add
            acts[act]+=v
        self.set("actuators",acts,comment="set by setActuator %s"%time.strftime("%y/%m/%d %H:%M:%S"))
        self.setSwitchRequested(wait=1)
        self.copyToInactive()

    def getActuators(self):
        #cb=self.circBufDict["rtcActuatorBuf"]
        cb=buffer.Circular("/"+self.shmPrefix+"rtcActuatorBuf")
        cb.setForceWrite()
        acts=None
        for i in range(10):
            latest=cb.getLatest()
            if latest==None:
                time.sleep(0.1)
            else:
                acts,t,fno=latest
                break
        if acts==None:
            raise Exception("Unable to get actuators")
        return acts

    def getStatus(self):
        #print "get status"
        s=self.getStream("%srtcStatusBuf"%self.shmPrefix)

        #cb=buffer.Circular("/%srtcStatusBuf"%self.shmPrefix)
        #cb.setForceWrite()
        #s=None
        #for i in range(10):#attempt 10 times.
        #    latest=cb.getLatest()
        #    if latest==None:
        #        time.sleep(0.1)
        #    else:
        #        s,t,fno=latest
        #        break
        if s!=None:
            s=s[0].tostring()
            s=s[:s.index("\0")]
        return s
    def getStream(self,name,latest=0,retry=0,wholeBuffer=0):
        """name should include shmprefix"""
        print "getStream %s"%name
        cb=buffer.Circular("/%s"%(name))
        #print "Got buffer"
        s=cb.getLatest()
        #print "Got latest"
        #s,t,fno=cb.getLatest()
        if latest==0 or (retry==1 and s==None):
            cb.setForceWrite()
            s=None
            #time.sleep(0.1)
            for i in range(10):#attempt 10 times
                #latest=cb.getLatest()
                #print "GetNext"
                latest=cb.getNextFrame(timeout=1.)
                #print "GotNext"
                if latest==None:
                    time.sleep(0.1)
                else:
                    #s,t,fno=latest
                    s=latest
                    break
        #print "Here"
        if wholeBuffer:#return the whole buffer (e.g. for rtcTimeBuf)
            s=cb.data,s[1],s[2]
        return s#None, or (data,time,fno)

    def getVersion(self):
        version="control.py version: "+CVSID+"\ndarccore version:"
        try:
            b=self.getActiveBuffer()
            version+=b.get("version")
        except:
            version+="darccore version unknown"
        return version

    def startSummer(self,stream,nsum,decimation=1,affin=0x7fffffff,prio=0,fromHead=1,startWithLatest=1,rolling=0,dtype="n",outputname=None,nstore=10):
        if rolling:
            rtxt="Rolling"
        else:
            rtxt="Summed"
        if fromHead:
            htxt="Head"
        else:
            htxt="Tail"
        if outputname==None:
            outputname="%s%s%s%d%c%sBuf"%(self.shmPrefix,stream,rtxt,nsum,dtype,htxt)
        if os.path.exists("/dev/shm/%s"%outputname):
            raise Exception("Stream for %s already exists"%outputname)
        dec=self.getRTCDecimation(self.shmPrefix+stream)[self.shmPrefix+stream]
        if dec==0:
            print "Setting decimation of %s to 1"%(self.shmPrefix+stream)
            self.setRTCDecimation(self.shmPrefix+stream,1)
        plist=["summer","-d%d"%decimation,"-a%d"%affin,"-i%d"%prio,"-n%d"%nsum,"-t%c"%dtype,"-o/%s"%outputname,"-S%d"%nstore,stream]
        if startWithLatest:
            plist.append("-l")
        if fromHead:
            plist.append("-h")
        if rolling:
            plist.append("-r")
        if len(self.shmPrefix)>0:
            plist.append("-s%s"%self.shmPrefix)
        if self.summerDict.has_key(outputname):
            self.summerDict[outputname].terminate()
            self.summerDict[outputname].wait()
        self.summerDict[outputname]=subprocess.Popen(plist)
        return outputname

    def stopSummer(self,name):
        if self.summerDict.has_key(name):
            self.summerDict[name].terminate()
            self.summerDict[name].wait()
            del(self.summerDict[name])
        if os.path.exists("/dev/shm/%s"%name):
            print "Manually removing /dev/shm/%s"%name
            os.unlink("/dev/shm/%s"%name)

    def getSummerList(self):
        return self.summerDict.keys()

    def sumData(self,stream,nsum,dtype):
        #first look to see if a summer stream exists already
        outname=None
        create=0
        #Set the decimate of the stream...
        

        for rs in ["Rolling","Summed"]:
            for ht in ["Head","Tail"]:
                tmp="%s%s%s%d%c%sBuf"%(self.shmPrefix,stream,rs,nsum,dtype,ht)
                if os.path.exists("/dev/shm/%s"%tmp):
                    outname=tmp
                    break
        if outname==None:
            outname="%s%s%s%d%c%sBuf"%(self.shmPrefix,stream,"Tmp",nsum,dtype,"Head")
            create=1
        else:
            print "Getting summed data from %s"%outname

        dec=self.getRTCDecimation(self.shmPrefix+stream)[self.shmPrefix+stream]
        if dec==0:
            print "Setting decimation of %s to 1"%(self.shmPrefix+stream)
            self.setRTCDecimation(self.shmPrefix+stream,1)
        p=None
        try:
            if create:
                plist=["summer","-d1","-l","-h","-1","-n%d"%nsum,"-t%c"%dtype,"-o/%s"%outname,"-S2",stream]
                if len(self.shmPrefix)>0:
                    plist.append("-s%s"%self.shmPrefix)

                # start the summing process going
                print "Starting summer for %s %s"%(outname,str(plist))
                p=subprocess.Popen(plist)
                #Wait for the stream to appear...
                n=0
                while n<1000 and not os.path.exists("/dev/shm/%s"%outname):
                    n+=1
                    time.sleep(0.01)
                if n==1000:
                    print "ERROR - stream %s did not appear"%outname
                    p.terminate()
                    raise Exception("Error - stream %s did not appear"%outname)
            # now get the stream.
            data=self.getStream(outname,latest=1,retry=1)
            if create:
                print "Terminating summer for %s"%outname
                p.terminate()#the process will then remove its shm entry.
                p.wait()
            if dec==0:
                print "Setting decimation of %s to 0"%(self.shmPrefix+stream)
                self.setRTCDecimation(self.shmPrefix+stream,0)
        except:#catch any exceptions and stop the process...
            if p!=None:
                p.terminate()
            raise
        return data

    def closeLoop(self,rmx):
        self.copyToInactive()
        self.set("closeLoop",0,comment="set by closeLoop %s"%time.strftime("%y/%m/%d %H:%M:%S"))
        self.setSwitchRequested(wait=1)
        self.copyToInactive()
        b=self.getInactiveBuffer()
        gain=b.get("gain")
        rmx.shape=gain.shape[0],rmx.shape[0]/gain.shape[0]
        self.set("rmx",rmx,comment="set by closeLoop %s"%time.strftime("%y/%m/%d %H:%M:%S"))
        rmxt=rmx.transpose().copy()
        #if rmxt.shape[1]==gain.shape[0]:
        for i in range(rmxt.shape[1]):
            rmxt[:,i]*=gain[i]
        self.set("gainReconmxT",rmxt,comment="set by closeLoop %s"%time.strftime("%y/%m/%d %H:%M:%S"))
        self.set("closeLoop",1,comment="set by closeLoop %s"%time.strftime("%y/%m/%d %H:%M:%S"))
        self.setSwitchRequested(wait=1)
        self.copyToInactive()

    def openLoop(self,val=1):
        #freeze update of mirror actuators
        self.copyToInactive()
        self.set("closeLoop",1-val,comment="set by openLoop %s"%time.strftime("%y/%m/%d %H:%M:%S"))
        self.setSwitchRequested(wait=1)
        self.copyToInactive()
        

    def preparePokeMatrix(self,nSkip,nAverage,offset,vMirror):
        self.copyToInactive()
        b=self.getInactiveBuffer()
        nacts=b.get("nacts")
        arr=numpy.zeros((nacts*4,nacts),numpy.uint16)
        steps=numpy.zeros((nacts*4,),numpy.int32)
        steps[::2]=nSkip
        steps[1::2]=nAverage
        arr[:]=offset
        print "Making poke matrix",offset,vMirror
        vm=vMirror
        if type(vMirror)==type(numpy.ndarray):
            vMirror.shape=vMirror.size/nacts,nacts
            vm=vMirror[0]
        arr[::4]+=numpy.identity(nacts)*vm#make shape
        arr[1::4]+=numpy.identity(nacts)*vm#take data
        if type(vMirror)==type(numpy.ndarray):
            vm=vMirror[-1]
        arr[2::4]-=numpy.identity(nacts)*vm#make shape
        arr[3::4]-=numpy.identity(nacts)*vm#take data
        return arr,steps

    def makePokeMatrix(self,data,steps):
        self.copyToInactive()
        b=self.getInactiveBuffer()
        nacts=b.get("nacts")
        ncents=b.get("subapFlag").sum()*2
        if len(data.shape)==1:
            data.shape=data.shape[0]/nacts,nacts
        if type(steps)!=numpy.ndarray or steps.shape[0]!=data.shape[0]:#same number of steps for each
            steps=numpy.ones((data.shape[0],),numpy.int32)*steps
        #Now do the poking.
        #cb=self.circBufDict["rtcGenericBuf"]
        cb=buffer.Circular("/"+self.shmPrefix+"rtcGenericBuf")
        latest=cb.getLatest()#non-blocking - just updates the internal counters
        #if latest!=None:
        #    prevFrame=latest[2]
        cb.setForceWrite()

        self.set("actuators",data,copyFirst=1)
        self.set("addActuators",0)
        self.set("actuatorMask",None)
        self.set("actSequence",steps,check=0)
        self.set("recordCents",1,check=0)
        self.setSwitchRequested(wait=1)
        #The RTC will now be poking.  So wait for the genericbuf to have 
        #the poke matrix...
        print "Getting latest %srtcGenericBuf frame"%self.shmPrefix
        pmx,timestamp,frameno=cb.getNextFrame()
        print "Got pmx:",pmx.shape,data.shape[0],ncents
        pmx.shape=data.shape[0],ncents
        self.set("actuators",None,copyFirst=1)
        self.set("actSequence",None)
        self.set("recordCents",0)
        self.setSwitchRequested(wait=1)
        self.copyToInactive()
        return pmx

    def makePokeMatrixOld(self,data,steps):
        self.copyToInactive()
        b=self.getInactiveBuffer()
        nacts=b.get("nacts")
        ncents=b.get("subapFlag").sum()*2
        if len(data.shape)==1:
            data.shape=data.shape[0]/nacts,nacts
        if type(steps)!=numpy.ndarray or steps.shape[0]!=data.shape[0]:#same number of steps for each
            steps=numpy.ones((data.shape[0],),numpy.int32)*steps
        #Now do the poking.
        pmx=numpy.zeros((data.shape[0],ncents),numpy.float32)
        #cb=self.circBufDict["rtcCentBuf"]
        cb=buffer.Circular("/"+self.shmPrefix+"rtcGenericBuf")
        fnolast=-1
        for i in range(data.shape[0]):
            self.set("actuators",data[i])
            self.setSwitchRequested(wait=1)
            for j in range(steps[i]):#average this many centroid measurements
                cb.setForceWrite()
                for l in range(10):#a few attempts to get a new frame
                    latest=cb.getLatest()
                    if latest!=None and latest[2]!=fnolast:
                        break
                    time.sleep(0.1)
                if latest==None or latest[2]==fnolast:
                    raise Exception("Didnt get circ buf centroids")
                cents,t,fnolast=latest
                #print cents.shape,t,fnolast,pmx.shape
                pmx[i]+=cents
            pmx[i]/=steps[i]
        self.set("actuators",None)
        self.setSwitchRequested(wait=1)
        return pmx
        
    
    def setKalman(self,atur,hinft,invn,hinfdm):
        self.copyToInactive()
        self.set("kalmanHinfT",hinft,comment="set by setKalman %s"%time.strftime("%y/%m/%d %H:%M:%S"))
        self.set("kalmanHinfDM",hinfdm,comment="set by setKalman %s"%time.strftime("%y/%m/%d %H:%M:%S"))
        self.set("kalmanAtur",atur,comment="set by setKalman %s"%time.strftime("%y/%m/%d %H:%M:%S"))
        self.set("kalmanInvN",invn,comment="set by setKalman %s"%time.strftime("%y/%m/%d %H:%M:%S"))
        self.set("kalmanUsed",1,comment="set by setKalman %s"%time.strftime("%y/%m/%d %H:%M:%S"))
        self.setSwitchRequested(wait=1)
        self.copyToInactive()
        return 0
    def setRMX(self,rmx):
        self.copyToInactive()
        b=self.getInactiveBuffer()
        gain=b.get("gain")
        rmx.shape=gain.shape[0],rmx.shape[0]/gain.shape[0]
        self.set("rmx",rmx,comment="set by setRMX %s"%time.strftime("%y/%m/%d %H:%M:%S"))
        rmxt=rmx.transpose().copy()
        #if rmxt.shape[1]==gain.shape[0]:
        for i in range(rmxt.shape[1]):
            rmxt[:,i]*=gain[i]
        self.set("gainReconmxT",rmxt,comment="set by setRMX %s"%time.strftime("%y/%m/%d %H:%M:%S"))
        self.setSwitchRequested(wait=1)
        self.copyToInactive()
        

    def resetLoop(self):
        self.copyToInactive()
        self.set("closeLoop",0,comment="set by resetLoop %s"%time.strftime("%y/%m/%d %H:%M:%S"))
        self.setSwitchRequested(wait=1)
        self.copyToInactive()
        self.set("closeLoop",1,comment="set by resetLoop %s"%time.strftime("%y/%m/%d %H:%M:%S"))
        self.setSwitchRequested(wait=1)
        self.copyToInactive()

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
                c["subapLocation"]=self.computeFillingSubapLocation(updateRTC=0,buf=c)
            else:#give enough pixels to entirely use the ccd.
                maxpxls=numpy.max((npxlx*npxly+nsubapsUsed-1)/nsubapsUsed)
                c["subapLocation"]=numpy.zeros((nsubaps,maxpxls))
                c["subapLocation"]=self.computeFillingSubapLocation(updateRTC=0,buf=c)
            self.checkAdd(c,"subapLocation",c["subapLocation"],comments)
            # # now set up a default subap location array...
            # ystep=1#numpy.array([1,1])
            # xstep=1#numpy.array([2,1])
            # xin=0#number of pixels in from edge of image that first subap starts
            # yin=0
            # subx=(npxlx-2*xin*xstep)/nsubx*xstep
            # suby=(npxly-2*yin*ystep)/nsuby*ystep
            # subapLocation=numpy.zeros((nsubaps,6),numpy.int32)
            # for k in range(c["ncam"]):
            #     for i in range(nsuby[k]):
            #         for j in range(nsubx[k]):
            #             indx=nsubapsCum[k]+i*nsubx[k]+j
            #             if subapFlag[indx]:
            #                 subapLocation[indx]=(yin*ystep+i*suby[k],yin*ystep+i*suby[k]+suby[k],ystep,xin*xstep+j%xstep+(j/xstep)*subx[k],xin*xstep+j%xstep+(j/xstep)*subx[k]+subx[k],xstep)
            # self.checkAdd(c,"subapLocation",subapLocation,comments)
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
        self.checkAdd(c,"nsubapsTogether",1,comments)
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

    def initialiseBuffer(self,nb,configFile):
        """fill buffers with sensible values
        Place the memory into its initial state...
        Could eg get these values from a param file or something.
        """
        print "Initialising buffer %d (config file=%s)"%(nb,str(configFile))

        bufDone=0
        buf=self.bufferList[nb]
        buf.buffer.view("b")[:]=0#empty the buffer.
        control={}
        comments={}
        if type(configFile)==numpy.ndarray:
            b=buffer.Buffer(None)
            b.buffer.view(configFile.dtype)[:configFile.size]=configFile
            labels=b.getLabels()
            for label in labels:
                val=b.get(label)
                com=b.getComment(label)
                control[label]=val
                comments[label]=com
            #buffer.buffer.view(configFile.dtype)[:configFile.size]=configFile
            #bufDone=1
        elif type(configFile)==type("") and configFile[-3:]==".py":
            print "Executing config file %s..."%configFile
            #control={}
            d={"control":{},"prefix":self.shmPrefix,"numpy":numpy}
            #execfile(configFile,globals())
            #global control#gives syntax warning, but is required to work!
            execfile(configFile,d)
            control=d["control"]
            if d.has_key("comments"):
                comments=d["comments"]
            if not control.has_key("configfile"):
                control["configfile"]=configFile
        elif type(configFile)==type("") and configFile[-5:]==".fits":
            tmp=FITS.Read(configFile)[1]
            #buffer.buffer.view(tmp.dtype)[:tmp.size]=tmp
            #bufDone=1
            b=buffer.Buffer(None)
            #b.buffer.view(tmp.dtype)[:tmp.size]=tmp
            b.assign(tmp)
            labels=b.getLabels()
            for label in labels:
                val=b.get(label)
                com=b.getComment(label)
                control[label]=val
                comments[label]=com
            if not control.has_key("configfile"):
                control["configfile"]=configFile
        elif type(configFile)==type(""):#assume that this is the config text...
            d={"control":{},"prefix":self.shmPrefix,"numpy":numpy}
            try:
                exec configFile in d#globals()
                control=d["control"]
                if d.has_key("comments"):
                    comments=d["comments"]
            except:
                print "Error doing configuration"
                traceback.print_exc()
                raise
        else:
            raise Exception("Buffer not initialised")
        if bufDone==0:
            try:
                self.inventValues(control,comments)
            except:
                traceback.print_exc()
                print "Unable to invent values..."
            buf.setControl("switchRequested",0)
            if control.has_key("switchRequested"):
                del(control["switchRequested"])
            failed=[]
            for key in control.keys():
                #buffer.set(key,control[key])
                try:
                    self.set(key,control[key],comment=comments.get(key,""),usrbuffer=buf)
                except:
                    failed.append(key)
            while len(failed)>0:
                f=[]
                for key in failed:
                    try:
                        self.set(key,control[key],comment=comments.get(key,""),usrbuffer=buf)
                    except:
                        f.append(key)
                if len(f)==len(failed):#failed to add new ones...
                    print "Failed to initialise buffer:"
                    print f
                    for key in f:
                        try:
                            self.set(key,control[key],comment=comments.get(key,""),usrbuffer=buf)
                        except:
                            print key
                            traceback.print_exc()

                    raise Exception("Failed to initialise buffer")
                failed=f
            #Is this necessary?
            #buf.freezeContents()
        self.bufferList[1-nb].setControl("switchRequested",1)
        b=self.bufferList[1-nb]
        i=0
        while b.flags[0]&0x1==1 and self.nodarc==0:
            utils.pthread_mutex_lock(b.condmutex)
            try:
                t=utils.pthread_cond_timedwait(b.cond,b.condmutex,10.,1)
            except:
                traceback.print_exc()
                print "Error in utils.pthread_cond_timedwait - continuing"
            utils.pthread_mutex_unlock(b.condmutex)
            i+=1
            if t:
                print "Waiting for switch to complete - timed out - retrying"
                if i>=5:
                    raise Exception("Failed to start RTC")


        print "Switch completed - copying buffer"
        #now copy the buffer...
        self.bufferList[1-nb].buffer[:]=self.bufferList[nb].buffer
        self.informParamSubscribers()
        self.publishParams()






class dummyLock:
    def __init__(self):
        pass
    def acquire(self):
        pass
    def release(self):
        pass
class subscribeObject:#used when subscribing to parameter changes
    def __init__(self,params):
        self.params=params
        self.cond=threading.Condition()
        self.changed=[]
        self.time=time.time()
        self.notifytime=0
        self.notifycnt=0
        self.streamsChanged=0

class Subscriber:
    def __init__(self,sock,freq):
        """Sock is the socket, freq is the desired frequency (ie the decimate factor)
        cumfreq is the cumulative decimate, used when the rtc decimate is less than the requested decimate.
        """
        self.sock=sock
        self.freq=freq
        self.cumfreq=freq


if __name__=="__main__":
    controlName="Control"
    uselock=1
    for arg in sys.argv[1:]:
        if arg[:2]=="-s":#the shm prefix...
            controlName=arg[2:]+controlName
        elif arg[:2]=="--":
            if arg[2:9]=="prefix=":
                controlName=arg[9:]+controlName

    ei=None
    while ei==None:
        ei=controlCorba.initialiseServer(controlName=controlName)#this is called here to remove any corba stuff from argv.
        if ei==None:
            time.sleep(1)
    c=Control(globals())
    if ei!=None:
        ei.initialise(c,c.lock)
        
    try:
        c.loop()
    except:
        traceback.print_exc()
    print "Ending"
    if c.logread!=None:
        c.logread.go=0
    if c.ctrllogread!=None:
        c.ctrllogread.go=0
