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
import darc
import Check
import startStreams
import subprocess
import logread
import stdoutlog
import argparse 
try:
    import numa
except:
    print "python numa library not imported."


class Control:
    """A class for controlling the RTC.  The GUI connects to this object.
    Opens a socket and listens for the GUI...
    """
    def __init__(self,globals, options=None):
        #global DataSwitch
        #global PS
        self.globals=globals
        self.circgo=1
        self.go=1
        self.nodarc=0
        self.coremain=None
        #self.serverObject=None
        self.pipeDict={}
        self.paramChangedDict={}
        self.paramChangedSubscribers={"tag":{}}
        self.summerDict={}
        self.splitterDict={}
        self.binnerDict={}
        self.errList=[]
        self.rtcStopped=0
        self.dsConfig=None
        self.reconnectRunning=0
        self.check=Check.Check()
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
        self.numaSize=0
        self.circBufMaxMemSize=None
        self.nstoreDict={}
        affin=0x7fffffff
        self.darcaffinity=0xffffffffffffffff
        uselock=1
        prio=0
        if options is None:
            self.port=4242
            self.host=socket.gethostname()
            self.configFile=None
        else:
            self.port = options.port
            self.host = options.host
            affin = options.affin
            self.darcaffinity=int(options.darcaffinity,16)
            prio = options.prio

            if options.lock is False:
                uselock=0
            if options.output:
                self.redirectdarc=0
                self.redirectcontrol=0
            if options.routput:
                self.redirectcontrol=0
            if options.prefix!=None:
                self.shmPrefix = options.prefix
            if options.buffersize!=None:
                self.bufsize = int(eval(options.buffersize))
            if options.nhdr!=None:
                self.nhdr = int(options.nhdr)
            if options.nodarc:
                self.nodarc = 1
            if options.nstore is not None:
                for stream in options.nstore:
                    self.nstoreDict[stream[0]] = int(stream[1])
            if options.circBufMemSize!=None:
                self.circBufMaxMemSize=eval(options.circBufMemSize)
            if options.numaSize!=None:
                self.numaSize=eval(options.numaSize)
            self.configFile = options.configfile

        if self.redirectcontrol:
            print "Redirecting control output"
            sys.stdout=stdoutlog.Stdoutlog("/dev/shm/%srtcCtrlStdout"%self.shmPrefix)
            sys.stderr=sys.stdout
        if uselock:
            self.lock=threading.Lock()
        else:
            self.lock=dummyLock()
        self.readStreams=1

        if prio!=0 or affin!=0x7fffffff:
            utils.setAffinityAndPriority(affin,prio)
        self.DecimateHandlerObject=None
        self.numaNodes=0
        self.numaBufferList=[]

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
        self.logread=logread.logread(name="/dev/shm/%srtcStdout0"%self.shmPrefix,callback=self.logreadCallback)
        self.logread.sleep=1
        self.logread.launch()
        #thread.start_new_thread(self.watchStreamThread,())
        if self.redirectcontrol:
            self.ctrllogread=logread.logread(name="/dev/shm/%srtcCtrlStdout0"%self.shmPrefix,callback=self.ctrllogreadCallback)
            self.ctrllogread.sleep=1
            self.ctrllogread.launch()

    def logreadCallback(self,txt):
        """publish the new log txt to any subscribers"""
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
        return 0
    def ctrllogreadCallback(self,txt):
        """publish the new log txt to any subscribers"""
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
                    #print "Starting RTC"
                    plist=["darcmain","-i"]
                    if self.darcaffinity!=0xffffffffffffffff:
                        plist.append("-I%#x"%self.darcaffinity)
                    if self.nhdr!=None:
                        plist.append("-e%d"%self.nhdr)
                    if self.bufsize!=None:
                        plist.append("-b%d"%self.bufsize)
                    if self.circBufMaxMemSize!=None:
                        plist.append("-m%d"%self.circBufMaxMemSize)
                    if self.redirectdarc==1:
                        plist.append("-r")#redirect to /dev/shm/stdout0
                    if len(self.shmPrefix)>0:
                        plist.append("-s%s"%self.shmPrefix)
                    if self.numaSize!=0:
                        plist.append("-N%d"%self.numaSize)
                    for st in self.nstoreDict.keys():#circular buffer sizes
                        plist+=["-c",st,"%d"%self.nstoreDict[st]]
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
        if self.numaSize!=0:#check for numa-aware buffers...
            if numa.available()==False:
                raise Exception("Numa specified but not available")
            self.numaNodes=numa.get_max_node()+1
            print "self.numaNodes: %d"%self.numaNodes
            self.numaBufferList=[]
            for i in range(self.numaNodes):
                self.numaBufferList.append(buffer.Buffer("/%srtcParam1Numa%d"%(self.shmPrefix,i)))
                self.numaBufferList.append(buffer.Buffer("/%srtcParam2Numa%d"%(self.shmPrefix,i)))
            print self.numaBufferList
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
                    try:
                        self.initialiseBuffer(bufno,self.configFile)
                    except:
                        #failed to initialise, so quit it.
                        if self.coremain!=None:
                            self.stop(stopControl=0)
                        raise

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
        return 0
            

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
                cb=buffer.Circular("/"+name)
                #self.lock.acquire()
                try:
                    if self.circBufDict.has_key(name):
                        del(self.circBufDict[name])
                    self.circBufDict[name]=cb
                except:
                    pass
                #self.lock.release()
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
        self.lock.acquire()#we grab the lock here so that none of the corba threads will try to access the circBufDict entry after its been removed.
        try:
            if self.circBufDict.has_key(name):
                del(self.circBufDict[name])
        except:
            pass
        self.lock.release()
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
            

    def getStreams(self):
        """Return a list of streams"""
        return self.streamList

    def newConnection(self,s,suberr=1):
        """Someone else has just connected"""
        #print "Subscribing new connection to rtcErrorBuf"
        if suberr:
            self.subscribe(s,self.shmPrefix+'rtcErrorBuf',1)#subscribe them to the error messages
            for err in self.errList:
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
                if infoDict["data"] is None:
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


    def removeError(self,err):
        errno=0
        if err==None or len(err)==0:#remove all errors
            self.errList=[]
            errno=0x7fffffff
        elif err in self.errList:
            self.errList.remove(err)
        else:
            print "warning %s not found in control ErrorList: %s"%(err,str(self.errList))
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
                #self.lock.acquire()
                try:
                    circBufDict[key]=mybuf
                except:
                    pass
                #self.lock.release()
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
        self.setRTCDecimation(name,d1)


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

    def getNumaLabels(self,node):
        b=self.getActiveBuffer()
        if b is self.bufferList[0]:
            bufno=0
        else:
            bufno=1
        b=self.numaBufferList[2*node+bufno]
        return b.getLabels()

    
    def copyToInactive(self):
        """Copy from active to inactive buffer.  Here, we assume that the active buffer is correct, so can just copy the buffer contents without any checking."""
        ac=self.getActiveBuffer()
        inac=self.getInactiveBuffer()
        if ac is self.bufferList[0]:
            bufno=0
        else:
            bufno=1
        #Note - we don't copy the buffer header.
        #Also note - this is quite bad for hammering memory bandwidth!
        inac.buffer.view("b")[:]=ac.buffer.view("b")
        if self.numaSize!=0:
            #also copy the numa nodes.
            for i in range(self.numaNodes):
                ac=self.numaBufferList[2*i+bufno]
                inac=self.numaBufferList[2*i+1-bufno]
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
                try:
                    os.unlink("/dev/shm/%srtcParam1"%self.shmPrefix)
                except:
                    print "Failed to unlink %srtcParam1"%self.shmPrefix
            if os.path.exists("/dev/shm/%srtcParam2"%self.shmPrefix):
                try:
                    os.unlink("/dev/shm/%srtcParam2"%self.shmPrefix)
                except:
                    print "Failed to unlink %srtcParam2"%self.shmPrefix
            d=os.listdir("/dev/shm")
            if self.coremain!=None:
                print "Waiting for darcmain to finish..."
                for i in range(10):
                    if self.coremain.poll()!=None:
                        break
                    time.sleep(1)
                if self.coremain.poll()==None:
                    print "Terminating darcmain"
                    self.coremain.terminate()
                    time.sleep(1)
                    if self.coremain.poll()==None:
                        self.coremain.kill()
                    self.coremain.wait()
                    self.coremain=None
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
        b.set("pause",p)
        self.paramChangedDict["pause"]=(p,"")
        self.setSwitchRequested(preservePause=0,wait=wait)
        return p

    def setCloseLoop(self,p=1,wait=1):
        self.set("closeLoop",p,update=1,wait=wait)
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
                try:
                    t=utils.pthread_mutex_lock_cond_wait(active.cond,active.condmutex,1,1)
                except:
                    print "Error in utils.pthread_mutex_lock_cond_wait - continuing"
                if t:
                    print "Timeout while waiting - active flag now %d, inactive %d"%(inactive.flags[0]&1,active.flags[0]&1)
            print "Got inactive buffer"
        self.informParamSubscribers()
        if preservePause:
            self.publishParams()

    def publishParams(self):
        """Send current active param buf to the dataswitch"""
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

    def get(self,name):
        buf=self.getActiveBuffer()
        got=0
        try:
            val=buf.get(name)
            got=1
        except:
            pass
        if got==0:
            if buf is self.bufferList[0]:
                bufno=0
            else:
                bufno=1
            for b in self.numaBufferList[bufno::2]:
                try:
                    val=b.get(name)
                    got=1
                except:
                    pass
                else:
                    break
        if got==0:
            raise Exception("Value %s not found"%name)
        return val

    def getNuma(self,name,node):
        buf=self.getActiveBuffer()
        if buf is self.bufferList[0]:
            bufno=0
        else:
            bufno=1
        buf=self.numaBufferList[2*node+bufno]
        val=buf.get(name)
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
        elif inactive==0:#we've just written to the active buffer, so copy to inactive to keep it in sync.
            self.copyToInactive()

    def setNuma(self,name,val,node,comment="",inactive=1,check=1,update=0,wait=1):
        """Sets a value in a buffer in a Numa node.  
        Note, doesn't do a buffer swap unless update==1.
        If check is set, will check that the value is consistent with what is expected eg size and dtype etc (but not explicit values).
        if usrbuffer!=None, this buffer is used... this buffer is also used for checking...
        """
        #first, find out which is the active main buffer, which then tells us which Numa pair to use.
        buf=self.getActiveBuffer()
        if buf is None:#no active buffer... ???
            raise Exception("Didn't get active buffer for %s"%name)
        if buf is self.bufferList[0]:
            bufno=0
        else:
            bufno=1

        if inactive:
            b=self.numaBufferList[2*node+1-bufno]
        else:
            b=self.numaBufferList[2*node+bufno]
        if check:#check should probably be performed using the main buffer (since there won't be much in the numa buffers - just part of a control matrix, etc).
            val=self.check.valid(name,val,buf)
        if inactive:
            self.paramChangedDict[name]=(val,comment)
        else:#making a change to active buffer - so tell any listeners...
            self.informParamSubscribers({name:(val,comment)})
        b.set(name,val,comment=comment)
        print "setNuma - not setting dependencies (not expected to be any)"
        #try:
        #    self.setDependencies(name,b)
        #except:
        #    print "Error setting dependencies"
        #    traceback.print_exc()
        #    raise
        if update==1 and inactive==1:
            self.setSwitchRequested(wait=wait)
            if wait:
                self.copyToInactive()
        elif inactive==0:#we've just written to the active buffer, so copy to inactive to keep it in sync.
            self.copyToInactive()


    def getLog(self,getdarc=1,getctrl=1,getall=1,maxlen=0,minlen=0):
        if getdarc:
            txt=open("/dev/shm/%srtcStdout0"%self.shmPrefix).read()
            if len(txt)<minlen:#log rotation
                if os.path.exists("/dev/shm/%srtcStdout1"%self.shmPrefix):
                    txt=open("/dev/shm/%srtcStdout1"%self.shmPrefix).read()+txt
            if maxlen>0:
                txt=txt[-maxlen:]
            
        else:
            txt=""
        if getctrl:
            if os.path.exists("/dev/shm/%srtcCtrlStdout0"%self.shmPrefix):
                txt+="\n********* CONTROL OUTPUT *********\n\n"
                tmp=open("/dev/shm/%srtcCtrlStdout0"%self.shmPrefix).read()
                if len(tmp)<minlen:#log rotation
                    if os.path.exists("/dev/shm/%srtcCtrlStdout1"%self.shmPrefix):
                        tmp=open("/dev/shm/%srtcCtrlStdout1"%self.shmPrefix).read()+tmp
                if maxlen>0:
                    tmp=tmp[-maxlen:]
                txt+=tmp
        if getall:
            files=os.listdir("/dev/shm")
            for f in files:
                if f[:len(self.shmPrefix+"rtc")]==self.shmPrefix+"rtc" and f[-7:]=="Stdout0" and f!=self.shmPrefix+"rtcStdout0" and f!=self.shmPrefix+"rtcCtrlStdout0":
                    #a log file...
                    txt+="\n********* %s ************\n\n"%f
                    tmp=open("/dev/shm/%s"%f).read()
                    if len(tmp)<minlen:#log rotation
                        if os.path.exists("/dev/shm/%s1"%f[:-1]):
                            tmp=open("/dev/shm/%s1"%f[:-1]).read()+tmp
                    if maxlen>0:
                        tmp=tmp[-maxlen:]
                    txt+=tmp
        return txt
    
    def setDependencies(self,name,b):
        """Value name has just changed in the buffer,  This will require some other things updating.
        """
        changed=self.check.setDependencies(name,b)
        if changed!=None:
            for key in changed.keys():
                self.paramChangedDict[key]=changed[key]


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



    def setActuator(self,mode,act,v):
        """set individual actuator."""
        #set an individual actuator
        #First get the current actuator values.
        #Subscribe to rtcActuatorBuf
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
        if s!=None:
            s=darc.statusBufToString(s[0])
            s=s[:s.index("\0")]
        return s
    def getStream(self,name,latest=0,retry=0,wholeBuffer=0,timeout=1.,retries=10):
        """name should include shmprefix"""
        print "getStream %s"%name
        cb=buffer.Circular("/%s"%(name))
        #print "Got buffer"
        cb.setForceWrite()
        s=cb.getLatest()
        #print "Got latest"
        #s,t,fno=cb.getLatest()
        if latest==0 or (retry==1 and s==None):
            cb.setForceWrite()
            s=None
            #time.sleep(0.1)
            for i in range(retries):#attempt 10 times
                #latest=cb.getLatest()
                #print "GetNext"
                latest=cb.getNextFrame(timeout=timeout)
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

    def startSummer(self,stream,nsum,decimation=1,affin=0x7fffffff,prio=0,fromHead=1,startWithLatest=1,rolling=0,dtype="n",outputname=None,nstore=10,sumsquare=0):
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
        if self.redirectcontrol:
            plist.append("-q")
        if len(self.shmPrefix)>0:
            plist.append("-s%s"%self.shmPrefix)
        if sumsquare:
            plist.append("-2")
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
        s=self.summerDict.keys()
        files=os.listdir("/dev/shm")
        for f in files:
            if f not in s:
                if f.startswith(self.shmPrefix+"rtc") and f.endswith("Buf"):
                    if (("Summed" in f) or ("Rolling" in f)) and (("Head" in f) or ("Tail" in f)):
                        s.append(f)
        return s

    def startSplitter(self,stream,readfrom=0,readto=-1,readstep=1,readblock=1,affin=0x7fffffff,prio=0,fromHead=1,outputname=None,nstore=-1):
        if fromHead:
            htxt="Head"
        else:
            htxt="Tail"
        if outputname==None:
            outputname="%s%sf%dt%dj%db%d%sBuf"%(self.shmPrefix,stream,readfrom,readto,readstep,readblock,htxt)
        if os.path.exists("/dev/shm/%s"%outputname):
            raise Exception("Stream for %s already exists"%outputname)
        plist=["splitter","-a%d"%affin,"-i%d"%prio,"-o/%s"%outputname,"-S%d"%nstore,"-F%d"%readfrom,"-T%d"%readto,"-J%d"%readstep,"-b%d"%readblock,stream]
        if fromHead:
            plist.append("-h")
        if self.redirectcontrol:
            plist.append("-q")
        if len(self.shmPrefix)>0:
            plist.append("-s%s"%self.shmPrefix)
        if self.splitterDict.has_key(outputname):
            self.splitterDict[outputname].terminate()
            self.splitterDict[outputname].wait()
        self.splitterDict[outputname]=subprocess.Popen(plist)
        return outputname

    def stopSplitter(self,name):
        if self.splitterDict.has_key(name):
            self.splitterDict[name].terminate()
            self.splitterDict[name].wait()
            del(self.splitterDict[name])
        if os.path.exists("/dev/shm/%s"%name):
            print "Manually removing /dev/shm/%s"%name
            os.unlink("/dev/shm/%s"%name)

    def getSplitterList(self):
        s=self.splitterDict.keys()
        files=os.listdir("/dev/shm")
        for f in files:
            if f not in s:
                if f.startswith(self.shmPrefix+"rtc") and f.endswith("Buf"):
                    if ("Buff" in f) and (("Head" in f) or ("Tail" in f)):
                        s.append(f)
        return s


    def startBinner(self,stream,nx,ny=1,readfrom=0,readto=-1,stride=-1,dtype='n',decimation=1,affin=0x7fffffff,prio=0,fromHead=1,outputname=None,nstore=-1):
        if fromHead:
            htxt="Head"
        else:
            htxt="Tail"
        if outputname==None:
            outputname="%s%sBinnerf%dt%dx%dy%dj%d%c%sBuf"%(self.shmPrefix,stream,readfrom,readto,nx,ny,stride,dtype,htxt)
        if os.path.exists("/dev/shm/%s"%outputname):
            raise Exception("Stream for %s already exists"%outputname)
        plist=["binner","-a%d"%affin,"-i%d"%prio,"-o/%s"%outputname,"-S%d"%nstore,"-F%d"%readfrom,"-T%d"%readto,"-J%d"%stride,"-x%d"%nx,"-y%d"%ny,"-t%c"%dtype,stream]
        if fromHead:
            plist.append("-h")
        if self.redirectcontrol:
            plist.append("-q")
        if len(self.shmPrefix)>0:
            plist.append("-s%s"%self.shmPrefix)
        if self.binnerDict.has_key(outputname):
            self.binnerDict[outputname].terminate()
            self.binnerDict[outputname].wait()
        self.binnerDict[outputname]=subprocess.Popen(plist)
        return outputname

    def stopBinner(self,name):
        if self.binnerDict.has_key(name):
            self.binnerDict[name].terminate()
            self.binnerDict[name].wait()
            del(self.binnerDict[name])
        if os.path.exists("/dev/shm/%s"%name):
            print "Manually removing /dev/shm/%s"%name
            os.unlink("/dev/shm/%s"%name)

    def getBinnerList(self):
        s=self.binnerDict.keys()
        files=os.listdir("/dev/shm")
        for f in files:
            if f not in s:
                if f.startswith(self.shmPrefix+"rtc") and f.endswith("Buf"):
                    if ("BufBinnerf" in f) and (("Head" in f) or ("Tail" in f)):
                        s.append(f)
        return s
    


    def startTemporarySummer(self,stream,nsum,dtype,sumsquare=0):
        """Starts a summer for 1 sum."""
        i=0
        p=None
        fname="/dev/shm/%s%sSummedTmp%d%cHead%dBuf"%(self.shmPrefix,stream,nsum,dtype,i)
        while os.path.exists(fname) or (sumsquare==1 and os.path.exists(fname+"2Buf")):
            i+=1
            fname="/dev/shm/%s%sSummedTmp%d%cHead%dBuf"%(self.shmPrefix,stream,nsum,dtype,i)
        outname=fname[9:]
        print "Setting decimation of %s to 1"%(self.shmPrefix+stream)
        self.setRTCDecimation(self.shmPrefix+stream,1)
        p=None
        data=None
        try:
            plist=["summer","-d1","-l","-h","-1","-n%d"%nsum,"-t%c"%dtype,"-o/%s"%outname,"-S2",stream]
            if len(self.shmPrefix)>0:
                plist.append("-s%s"%self.shmPrefix)
            if self.redirectcontrol:
                plist.append("-q")
            if sumsquare:
                plist.append("-2")
            # start the summing process going
            print " ".join(plist)
            #print "Starting summer for %s"%(str(plist))
            p=subprocess.Popen(plist,close_fds=True)
            #Wait for the stream to appear...
            n=0
            while n<200 and not os.path.exists("/dev/shm/%s"%outname):
                n+=1
                time.sleep(0.01)
            if n==200:
                print "ERROR - stream %s did not appear"%outname
                p.terminate()
                raise Exception("Error - stream %s did not appear"%outname)
        except:
            if p!=None:
                p.terminate()
            raise
        return outname,p

    def sumData(self,stream,nsum,dtype,sumsquare=0):
        #first look to see if a summer stream exists already
        outname=None
        out2name=None
        create=0
        data2=None
        #Set the decimate of the stream...
        

        for rs in ["Rolling","Summed"]:
            for ht in ["Head","Tail"]:
                tmp="%s%s%s%d%c%sBuf"%(self.shmPrefix,stream,rs,nsum,dtype,ht)
                if os.path.exists("/dev/shm/%s"%tmp):
                    if sumsquare:
                        tmp2="%s%s2%s%d%c%sBuf"%(self.shmPrefix,stream,rs,nsum,dtype,ht)
                        if os.path.exists("/dev/shm/%s"%tmp2):
                            out2name=tmp2
                        elif os.path.exists("/dev/shm/%s2Buf"%tmp):
                            out2name=tmp+"2Buf"
                    outname=tmp
                    break
        if sumsquare==1 and out2name==None and outname!=None:
            #current summer does not square
            i=0
            while os.path.exists("%s%s%d%s%d%c%sTmpBuf"%(self.shmPrefix,stream,i,"Tmp",nsum,dtype,"Head")):
                i+=1
            outname="%s%s%d%s%d%c%sTmpBuf"%(self.shmPrefix,stream,i,"Tmp",nsum,dtype,"Head")
            out2name=outname+"2Buf"
            create=1
        elif outname==None:
            outname="%s%s%s%d%c%sBuf"%(self.shmPrefix,stream,"Tmp",nsum,dtype,"Head")
            out2name=outname+"2Buf"
            create=1
        else:
            print "Getting summed data from %s"%outname

        dec=self.getRTCDecimation(self.shmPrefix+stream)[self.shmPrefix+stream]
        if dec!=1:#==0:
            print "Setting decimation of %s to 1"%(self.shmPrefix+stream)
            self.setRTCDecimation(self.shmPrefix+stream,1)
        p=None
        data=None
        try:
            if create:
                plist=["summer","-d1","-l","-h","-1","-n%d"%nsum,"-t%c"%dtype,"-o/%s"%outname,"-S2",stream]
                if len(self.shmPrefix)>0:
                    plist.append("-s%s"%self.shmPrefix)
                if self.redirectcontrol:
                    plist.append("-q")
                if sumsquare:
                    plist.append("-2")
                # start the summing process going
                print "Starting summer for %s %s"%(outname,str(plist))
                p=subprocess.Popen(plist,close_fds=True)
                #Wait for the stream to appear...
                n=0
                while n<1000 and not os.path.exists("/dev/shm/%s"%outname):
                    n+=1
                    time.sleep(0.01)
                if n==1000:
                    print "ERROR - stream %s did not appear"%outname
                    p.terminate()
                    raise Exception("Error - stream %s did not appear"%outname)
            #estimate how long it will take...
            try:
                status=self.getStream(self.shmPrefix+"rtcStatusBuf")
                if status!=None:
                    status=darc.parseStatusBuf(status[0])[0]
                    Hz=1./status["frametime"]
            except:
                Hz=100.
                traceback.print_exc()
                print "Continuing assuming 100Hz"
            timeout=nsum/Hz*2
            if timeout<1:
                timeout=1.
                #the total wait time is 10x timeout since will retry 10 times.
            # now get the stream.
            data=self.getStream(outname,latest=1,retry=1,timeout=timeout)
            if sumsquare:
                data2=self.getStream(out2name,latest=1,retry=1,timeout=timeout)
            if data==None:
                print "Hmm - didn't get data for %s timeout %g"%(outname,timeout)
            if create:
                print "Terminating summer for %s"%outname
                try:
                    p.terminate()#the process will then remove its shm entry.
                    p.wait()
                except:
                    traceback.print_exc()
                    print "Couldn't terminate process - not found - continuing..."
                p=None
            if dec!=1:#==0:
                print "Setting decimation of %s to %d"%(self.shmPrefix+stream,dec)
                self.setRTCDecimation(self.shmPrefix+stream,dec)
        except:#catch any exceptions and stop the process...
            if p!=None:
                p.terminate()
            raise
        if data2!=None:
            return [data[0],data2[0],data[1],data[2]]
        else:
            return data

    def getLogfiles(self):
        files=os.listdir("/dev/shm")
        flist=[]
        for f in files:
            if f.startswith(self.shmPrefix+"rtc") and f.endswith("Stdout0"):
                flist.append(f)
        return flist

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
        cb=None
        if os.path.exists("/dev/shm/%srtcGenericBuf"%self.shmPrefix):
            cb=buffer.Circular("/"+self.shmPrefix+"rtcGenericBuf")
            latest=cb.getLatest()#non-blocking - just updates the internal counters
            cb.setForceWrite()

        self.set("actuators",data,copyFirst=1)
        self.set("addActuators",0)
        self.set("actuatorMask",None)
        self.set("actSequence",steps,check=0)
        self.set("recordCents",1,check=0)
        self.setSwitchRequested(wait=1)
        #The RTC will now be poking.  So wait for the genericbuf to have 
        #the poke matrix...
        cnt=1
        i=1
        if cb==None:
            #wait for rtcGenericBuf
            while not os.path.exists("/dev/shm/%srtcGenericBuf"%self.shmPrefix):
                i+=1
                if i==cnt:
                    print "Waiting for /dev/shm/%srtcGenericBuf"%self.shmPrefix
                    cnt*=2
                time.sleep(1)
            cb=buffer.Circular("/"+self.shmPrefix+"rtcGenericBuf")
            time.sleep(0.2)#wait for it to be written...
            print "Getting latest %srtcGenericBuf frame"%self.shmPrefix
            pmx,timestamp,frameno=cb.getNextFrame()
        else:
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
            #b.buffer.view(configFile.dtype)[:configFile.size]=configFile
            b.assign(configFile)
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
                self.check.inventValues(control,comments)
            except:
                traceback.print_exc()
                print "Unable to invent values..."
            buf.setControl("switchRequested",0)
            if control.has_key("switchRequested"):
                del(control["switchRequested"])
            failed=[]
            numaList=[]
            for key in control.keys():
                #buffer.set(key,control[key])
                if key.startswith("numa"):
                    numaList.append(key)
                else:
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
            for key in numaList:
                node=int(key[4:])
                print "Setting parameters for numa node %d"%node
                numaData=control[key]
                for name in numaData.keys():
                    self.setNuma(name,numaData[name],node,comment="",inactive=1,check=1,update=0,wait=0)
                
            #Is this necessary?
            #buf.freezeContents()
        self.bufferList[1-nb].setControl("switchRequested",1)
        b=self.bufferList[1-nb]
        i=0
        while b.flags[0]&0x1==1 and self.nodarc==0:
            try:
                t=utils.pthread_mutex_lock_cond_wait(b.cond,b.condmutex,10.,1)
            except:
                print "Error in utils.pthread_mutex_lock_cond_wait - continuing"
            #utils.pthread_mutex_lock(b.condmutex)
            #try:
            #    t=utils.pthread_cond_timedwait(b.cond,b.condmutex,10.,1)
            #except:
            #    traceback.print_exc()
            #    print "Error in utils.pthread_cond_timedwait - continuing"
            #utils.pthread_mutex_unlock(b.condmutex)
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
    usage = '''
    darccontrol [options]

    darccontrol --configfile <input file name> --prefix <instance prefix name>
    
    Example:
    darccontrol pulnixConfigFile.py  -s pulnixcamera
    darccontrol -f pulnixConfigFile.py  -s pulnixcamera

    note:
    * more information, please use --help
    '''
    parser = argparse.ArgumentParser(usage=usage,conflict_handler='resolve')
    parser.add_argument('-s', '--prefix', dest='prefix', type=str, help='Prefix name to handle instace', default=None)
    parser.add_argument('-p', '--port', dest='port', type=int, help='Port to be used', default=4242)
    parser.add_argument('-H', '--host', dest='host', type=str, help='Host to be used', default=socket.gethostname())
    parser.add_argument('-d', '--dataswitch', dest='dataswitch', action='store_true', help='Host to be used', default=False)
    parser.add_argument('-I','--affinity',dest='darcaffinity',type=str,help='Thread affinity for darc core, default -1',default="0xffffffffffffffff")
    parser.add_argument('-a', '--affin', dest='affin', type=int, help='Number of affinity to be used,  default 0x7fffffff', default=0x7fffffff)
    parser.add_argument('-i', '--prio', dest='prio', type=int, help='Thread priority (importance) to be used', default=0)
    parser.add_argument('-l', '--lock', dest='lock', action='store_false', help='Use lock,  default True', default=True)
    parser.add_argument('-o', '--output', dest='output', action='store_true', help='Don\'t redirect the output of darcmain', default=False)
    parser.add_argument('-q', '--routput', dest='routput', action='store_true', help='Redirect the output of darcmain', default=False)
    parser.add_argument('-b', '--buffersize', dest='buffersize', type=str,  help='Set buffer size',  default=None)
    parser.add_argument('-e', '--nhdr', dest='nhdr',  type=str,  help='NHDR size',  default=None)
    parser.add_argument('-n', '--nodarc', dest='nodarc', action='store_true', help='No instance of darc - just the shm buffer.', default=False)
    parser.add_argument('-f', '--configfile', dest='configfile', type=str, help='Configuration file path', default=None)
    parser.add_argument('-c', '--nstore', dest='nstore', type=str,  nargs=2,  action='append',  help='stream name and number of circular buffer entries', default=None)
    parser.add_argument('-m', '--circBufMemSize', dest='circBufMemSize', type=str, help='Memory for circular buffers', default=None)
    parser.add_argument('-C', '--cleanstart', dest='cleanStart', action='store_true', help='Remove /dev/shm/*rtcParam[1,2] befpre starting', default=False)
    parser.add_argument('-N', '--numaSize', dest='numaSize', type=str,help='numa memory buffer size',default=None)
    (options, unknown) = parser.parse_known_args()
    controlName="Control"

    if options.prefix is None:
        options.prefix=""
    controlName = options.prefix + controlName
    if options.cleanStart:
        yn=raw_input("Remove /dev/shm/%srtcParam1 and 2? [y]/n "%options.prefix)
        if yn in ["","y","yes","Y","Yes","YES"]:
            try:
                os.unlink("/dev/shm/%srtcParam1"%options.prefix)
            except:
                pass
            try:
                os.unlink("/dev/shm/%srtcParam2"%options.prefix)
            except:
                pass
        yn=raw_input("Killall instances of darc (regardless of prefix)? [y]/n ")
        if yn in ["","y","yes","Y","Yes","YES"]:
            os.system("killall darcmain")
        yn=raw_input("Exit?  Yes to exit, no to continue.  [y]/n ")
        if yn in ["","y","yes","Y","Yes","YES"]:
            sys.exit(0)
        
    if len(unknown)>0:
        options.configfile = unknown[0]

    ei=None
    while ei==None:
        ei=darc.initialiseServer(controlName=controlName)#this is called here to remove any corba stuff from argv.
        if ei==None:
            time.sleep(1)
    c=Control(globals(), options)
    if ei!=None:
        ei.initialise(c,c.lock)
    try:
        c.loop()
    except KeyboardInterrupt:
        traceback.print_exc()
        msg="died with keyboard interrupt"
    except:
        traceback.print_exc()
        msg="died"
    else:
        msg="finished"

    print "Ending - control for darc has %s, darcmain may still be running"%msg
    if c.logread!=None:
        c.logread.go=0
    if c.ctrllogread!=None:
        c.ctrllogread.go=0
    darc.unbind(controlName)
    if msg=="died with keyboard interrupt":
        sys.__stdout__.write("\nEnding - control for darc has %s, darcmain may still be running.\nIf you wanted to stop darc, please use darcmagic stop -c in future.\n(Note - this will only work now if you restart darccontrol).\n"%msg)
    elif msg=="died":
        sys.__stdout__.write("\nEnding - control for darc has %s, darcmain may still be running.\nIf this was an unintentional crash, you can restart the control object using\ndarccontrol which should not affect operation of the real-time part of darc\n."%msg)
