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
"""This is a client of the RTCS.  It receives data from the RTCS and 
CORBA-ises it, and sends it onto the dataswitch.
It also does logging of data, and things like that.
It runs on the DataSwitch workstation.
Now that the dataswitch isn't used - this receives data, and saves to disk.
"""
#import buffer
import numpy
#import mmap
import time
import SockConn
try:
    import utils
except:
    #print "Unable to import utils"
    utils=None

import serialise
import os,threading,sys,socket,thread
import select
import Saver
#import traceback
DataSwitch=None
#try:
#    import DataSwitch
#except:
#    #print "Unable to import DataSwitch - not using."
#    DataSwitch=None
#if DataSwitch!=None:
#    print "Imported DataSwitch"
import FITS

# if DataSwitch !=None:
#     class config_handler_i(DataSwitch.DataSwitchModule__POA.ConfigSubscriber,DataSwitch.Subscriber):
#         def __init__(self,dataswitch,msg="config",callback=None):
#             DataSwitch.Subscriber.__init__(self, dataswitch)
#             self.msg=msg
#             self.callback=callback
#         def update(self, status):
#             #print self.msg,status
#             if self.callback!=None:
#                 self.callback(self.msg,status)

class GenericItem:
    def __init__(self,name,d1,d2,logFile,log):
        self.name=name
        self.decimate1=d1
        self.decimate2=d2
        self.log=log
        self.logFile=logFile

class Config:
    def __init__(self):
        self.generic=[]

#First connect to the rtcs.
#Then connect to the dataswitch.
#Subscribe to the config (tells us what to log, and how to decimate).
class dc:
    def __init__(self,argv,lport=4243,lhost=None,nconnect=-1,callback=None,closeOnFail=0,verbose=1,timeout=None,timeoutFunc=None):
        """port, host, number of connections to allow.
        callback can be a callback called when data arrives, accepting ["data",streamname,(data,frametime,framenumber)]
        
        """
        #self.host="m912"#host to connect to
        #self.port=4242#port to connect to (to get list of streams)
        self.verbose=verbose
        self.lport=lport#port to listen on (to receive streams)
        if lhost==None:
            lhost=socket.gethostname()
        self.lhost=lhost#host to listen on
        self.dsConfig=None#store the dataswitch config object.
        self.sendIndx=0
        self.serverObject=None
        self.userCallback=callback
        self.closeOnFail=closeOnFail
        #self.dsLock=threading.Lock()
        setprio=0
        affin=0x7fffffff
        prio=0
        self.dropFrames=0
        self.defaultSave=0
        self.rawdict={}
        self.sockStreamDict={}#associate sockets with streams...
        self.debug=0
        for arg in argv:
            #if arg[:2]=="-p":
            #    self.port=int(arg[2:])
            #    print "Using port %d"%self.port
            #elif arg[:2]=="-h":
            #    self.host=arg[2:]
            #    print "Using host %s"%self.host
            if arg[:2]=="-p":
                self.lport=int(arg[2:])
                print "Listening on port %d"%self.lport
            elif arg[:2]=="-h":
                self.lhost=arg[2:]
                print "Listening on %s"%self.lhost
            elif arg[:2]=="-a":#affinity
                affin=int(arg[2:])
                setprio=1
                if utils==None:
                    print "Unable to set affinity"
            elif arg[:2]=="-i":#importance
                prio=int(arg[2:])
                setprio=1
                if utils==None:
                    print "Unable to set priority"
            elif arg[:2]=="-d":
                self.dropFrames=1
            elif arg[:2]=="-s":#save by default for streams not in config
                self.defaultSave=1
            elif arg[:2]=="-v":
                self.debug=1
                print "Debug mode"
            elif arg[:2]=="-n":
                nconnect=int(arg[2:])
        self.nconnect=nconnect

        if setprio and utils!=None:
            utils.setAffinityAndPriority(affin,prio)
        self.savedTag=2**30#note tag can be anything - int, string etc.
        self.dataProcessDict={}
        self.dataForDS={}
        self.updateDecimateFromRTC=0
        self.clientSockList=[]#streams that are connected to us.
        self.addCallback("streamList",self.getStreams)
        self.streamList=[]
        #self.connectRTC(self.host,self.port)
        self.decDict=None
        if DataSwitch!=None:
            self.connectDataSwitch()
        self.saveStream={}
        self.rtcDecimate={}
        self.dsDecimate={}
        self.cumfreq={}
        #self.getStreams()
        self.globals=globals
        self.conndata={}#None
        self.recDataList=[]
        self.addCallback("rtcDecimateVals",self.receiveDecFromRTC)
        self.sockConn=SockConn.SockConn(self.lport,host=self.lhost,globals=self.globals,startThread=0,listenSTDIN=0,userSelList=[],userSelCmd=self.handleData,connectFunc=self.newConnection,verbose=verbose,timeout=timeout,timeoutFunc=timeoutFunc)
        if self.sockConn.bound==0:
            print "Not bound - Exiting..."
            sys.exit(0)

    def receiveDecFromRTC(self,data):
        print "receiveDecFromRTC"
        if self.updateDecimateFromRTC:
            self.updateDecimateFromRTC=0
            decDict=data[2]
            if self.dsConfig==None:
                self.decDict=decDict
            else:
                self.decDict=None
                clist=self.dsConfig.generic
                update=0
                for c in clist:
                    if decDict.has_key(c.name):
                        if c.decimate1!=decDict[c.name]:
                            c.decimate1=decDict[c.name]
                            update=1
                if update and self.serverObject!=None:
                    self.serverObject.publishConfig(self.dsConfig)


    def newConnection(self,s):
        """Someone has connected to us..."""
        self.clientSockList.append(s)
        self.sockConn.userSelList.append(s)
        self.rawdict[s]=(0,None)
        if self.nconnect>0:
            self.nconnect-=1
            if self.nconnect==0:#stop listening for new connections...
                if self.verbose:
                    print "Closing listening socket..."
                self.sockConn.selIn.remove(self.sockConn.lsock)
                self.sockConn.lsock.close()
        #nofd=0
        #for i in range(3,1024):
        #    try:
        #        os.fstat(i)
        #        nofd+=1
        #    except:
        #        pass
        #print "Total of %d fds"%nofd


    def process(self,sock):#,readSock=1):
        """Read from the socket... if we don't get a full serialise packet, wait.
        If a full packet is ready, act on it.
        """
        try:
            self.conndata[sock],valid=serialise.Recv(sock,self.conndata.get(sock))
        except:
            valid=0
            del(self.conndata[sock])#=None
            sock.close()
            if self.sockStreamDict.has_key(sock):
                del(self.sockStreamDict[sock])
            #self.conn=None
            sock=None
            print "Disconnected"
        if valid:
            data=self.conndata[sock]
            self.conndata[sock]=None
            data=serialise.Deserialise(data)[0]
            if type(data)==type(None):
                #self.conn=None
                if self.sockStreamDict.has_key(sock):
                    del(self.sockStreamDict[sock])
                sock=None
                print "Disconnected"
            else:
                self.recDataList.append(data)
        remList=[]
        #print "recDataList:",self.recDataList
        for data in self.recDataList:
            tag=(data[1])
            if self.dataProcessDict.has_key(tag):
                if self.dataProcessDict[tag](data)==1:#failed command
                    if self.closeOnFail:
                        sock.close()
                        if self.sockStreamDict.has_key(sock):
                            del(self.sockStreamDict[sock])

                        sock=None
                        if self.verbose:
                            print "dataProcessDict[%s] failed - disconnecting"%tag
                remList.append(data)
        for d in remList:
            self.recDataList.remove(d)
        #return remList
        return sock
    
    def handleData(self,source,condition=None):
        """This is called by SockConn when the select call has data"""
        if self.rawdict[source][0]==0:
            source=self.process(source)
            if source==None:#connection has been closed
                if self.verbose:
                    print "Connection closed remotely"
            if len(self.recDataList)>0:
                if self.verbose:
                    print "Unhandled things:"
            for data in self.recDataList:#unhandled things
                if data[0]=="name":
                    self.addStream(data[1])
                    self.sockStreamDict[source]=data[1]
                elif data[0]=="raw":#raw data will be coming from this socket in future.
                    if self.verbose:
                        print "Entering raw mode for %s (sock %s)"%(data[2],str(source))
                    self.rawdict[source]=(1,data[2])
                else:
                    for d in data:
                        if type(d)==type({}):
                            for key in d.keys():
                                print key,":\n",d[key]
                        elif type(d)==socket.SocketType:
                            print "Socket",d.getpeername()
                        else:
                            print d
            self.recDataList=[]
            return source!=None
        else:#raw data...
            source=self.processRaw(source,self.rawdict[source][1])
            #if source==None:#connection has benn closed or the handleRawPxl function failed - eg because user closed the socket for some reason.
                #print "Raw connection closed remotely"
                
            return source!=None
    def processRaw(self,sock,stream):
        try:
            self.conndata[sock],valid=self.recvRaw(sock,self.conndata.get(sock))
        except:
            valid=0
            del(self.conndata[sock])
            try:
                sock.close()
            except:
                print "couldn't close sock %s"%sock
                sock=None
            if self.sockStreamDict.has_key(sock):
                del(self.sockStreamDict[sock])
            sock=None
            print "Raw disconnected"
        if valid:
            data=self.conndata[sock]
            self.conndata[sock]=None
            if self.handleRawPxl(stream,data)!=0:
                if self.closeOnFail:
                    #print "processRaw closing socket"
                    sock.close()
                    if self.sockStreamDict.has_key(sock):
                        del(self.sockStreamDict[sock])

                    sock=None
                    if self.verbose:
                        print "handleRawPxl failed: %s"%stream

        return sock
        
    def recvRaw(self,sock,data):
        """Receive non-blocking from a socket... adding the result to data.  Then return (data,valid) with valid==1 if the data is a complete message ready for deserialisation.
        """
        if data==None:
            data=""
        #print "serialise.Recv datalen=%d"%len(data)
        datawaiting=1
        while datawaiting:
            rtr,rtw,err=select.select([sock],[],[],0.)
            if len(rtr)>0:#sock has some data...
                dlen=len(data)
                if dlen<4:
                    length=4#get the 4 byte header first...
                else:#work out how much data we need in total...
                    length=numpy.fromstring(data[:4],dtype=numpy.int32)+4#5+DecodeHeader(data[:5])[2]
                if dlen==length:
                    datawaiting=0
                else:
                    newdata=sock.recv(length-dlen)
                    if len(newdata)==0:#nothing received - must be error...
                        print "sock.recv didn't - datalen: %d left %d"%(dlen,length-dlen)
                        raise Exception("Failed to receive data")
                    data+=newdata
            else:
                datawaiting=0
        #now see if the data is complete:
        dlen=len(data)
        if dlen>=4 and dlen==4+numpy.fromstring(data[:4],dtype=numpy.int32):#DecodeHeader(data[:5])[2]:
            valid=1
        else:
            valid=0
        #print "done Recv, len %d, valid=%d"%(len(data),valid)
        return data,valid



    def connectRTC(self,host,port):
        self.conn=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        try:
            self.conn.connect((host,port))
            print "Connected to RTC"
        except:
            print "Couldn't connect to RTC"
            self.conn=None
        return self.conn

    def connectDataSwitch(self):
        if DataSwitch==None:
            self.serverObject=None
            return
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
    def dsConfigCallback(self,msg,config):
        """Callback called when config on the dataswitch changes.
        config.generic is a list of objects, o, which have:
        o.name - string
        o.decimate1 - int
        o.decimate2 - int
        o.logfile - string
        o.log - bool
        """
        print "dsConfigCallback",msg,config
        self.dsConfig=config
        update=0
        for obj in config.generic:
            if obj.name in self.streamList:
                #this is the config object for this stream...
                self.rtcDecimate[obj.name]=obj.decimate1
                self.dsDecimate[obj.name]=obj.decimate2
                #self.execute("c.setRTCDecimation('%s',%d);c.subscribe(sock,'%s',%d)"%(obj.name,obj.decimate1,obj.name,obj.decimate1))#dataclient needs data at decimate1. It then decimates this by decimate2/decimate1 before sending to dataswitch.
                if not self.saveStream.has_key(obj.name):
                    self.saveStream[obj.name]=None
                if obj.log:#logging...
                    if self.saveStream[obj.name]==None:#not currently logging
                        print "Start logging"
                        self.saveStream[obj.name]=Saver.Saver(obj.logFile)
                else:#not logging
                    if self.saveStream[obj.name]!=None:#need to finish the save
                        print "Finishing logging"
                        self.saveStream[obj.name].close()
                        self.saveStream[obj.name]=None
            if self.decDict!=None:
                if self.decDict.has_key(obj.name):
                    if obj.decimate1!=self.decDict[obj.name]:
                        obj.decimate1=self.decDict[obj.name]
                        update=1
        self.decDict=None
        if update and self.serverObject!=None:
            self.serverObject.publishConfig(self.dsConfig)

    def addStream(self,s):
        if self.verbose:
            print "addStream",s
        if s not in self.streamList:
            self.streamList.append(s)
        self.cumfreq[s]=0
        updateConfig=0
        if self.dsConfig==None:#no config object from dataswitch yet.
            if DataSwitch!=None:
                self.dsConfig=DataSwitch.DataSwitchModule.Config([])#,[],[])
            else:
                self.dsConfig=Config()
            updateConfig=-1#we don't want to update the config - let someone else do this.
        if s not in [x.name for x in self.dsConfig.generic]:
            #this stream not yet in dataswitch config...
            self.rtcDecimate[s]=1
            self.dsDecimate[s]=1
            if self.defaultSave:
                self.saveStream[s]=Saver.Saver(s+".log")
            else:
                self.saveStream[s]=None
            if DataSwitch!=None:
                gc=DataSwitch.DataSwitchModule.GenericConfig(s,0,0,s+".log",self.defaultSave)
            else:
                gc=GenericItem(s,0,0,s+".log",self.defaultSave)
            self.dsConfig.generic.append(gc)
            updateConfig+=1
        else:#this stream is in the dataswitch config...
            for obj in self.dsConfig.generic:
                if obj.name==s:
                    self.rtcDecimate[s]=obj.decimate1
                    self.dsDecimate[s]=obj.decimate2
                    if not self.saveStream.has_key(s):
                        self.saveStream[s]=None
                    if obj.log:#logging...
                        if self.saveStream[s]==None:
                            self.saveStream[s]=Saver.Saver(obj.logFile)
                    else:
                        if self.saveStream[s]!=None:
                            self.saveStream[s].close()
                            self.saveStream[s]=None
                    break
        self.addCallback(s,self.handlePxl)
        if updateConfig==1 and self.serverObject!=None:
            self.serverObject.publishConfig(self.dsConfig)
            
        if DataSwitch!=None:
            print "Informing dataswitch about %s"%s#create an empty stream.
            a=DataSwitch.DataSwitchModule.Generic(1,"i",0,0.,1,[0],0,"")
            if self.serverObject!=None:
                self.serverObject.publishGeneric(a,s)
            print "Done informing"

    def getStreams(self,data=None):
        if data==None:
            self.execute("s=c.getStreams()","s","streamList")
        else:
            #the list of streams has arrived...
            configChanged=0
            if self.dsConfig==None:#no config object from dataswitch yet.
                if DataSwitch!=None:
                    self.dsConfig=DataSwitch.DataSwitchModule.Config([])#,[],[])
                else:
                    self.dsConfig=Config()
                configChanged=1

            self.streamList=data[2]["s"]
            print "stream list:",self.streamList
            for s in self.streamList:
                self.cumfreq[s]=0
                if s not in [x.name for x in self.dsConfig.generic]:
                    #this stream not yet in dataswitch config...
                    self.rtcDecimate[s]=1
                    self.dsDecimate[s]=1
                    self.saveStream[s]=None
                    configChanged=1
                    if DataSwitch!=None:
                        gc=DataSwitch.DataSwitchModule.GenericConfig(s,0,0,s+".log",0)
                    else:
                        gc=GenericItem(s,0,0,s+".log",0)

                    self.dsConfig.generic.append(gc)
                else:#this stream is in the dataswitch config...
                    for obj in self.dsConfig.generic:
                        if obj.name==s:
                            self.rtcDecimate[s]=obj.decimate1
                            self.dsDecimate[s]=obj.decimate2
                            if not self.saveStream.has_key(s):
                                self.saveStream[s]=None
                            if obj.log:#logging...
                                if self.saveStream[s]==None:
                                    self.saveStream[s]=Saver.Saver(obj.logFile)
                            else:
                                if self.saveStream[s]!=None:
                                    self.saveStream[s].close()
                                    self.saveStream[s]=None
                            break

                self.addCallback(s,self.handlePxl)
                if DataSwitch!=None and self.serverObject!=None:
                    print "Informing dataswitch about %s"%s
                    a=DataSwitch.DataSwitchModule.Generic(1,"i",0,0.,1,[0],0,"")
                    self.serverObject.publishGeneric(a,s)
                    print "Done informing"
            if configChanged:#we have updated the config here...
                if self.serverObject!=None:
                    self.serverObject.publishConfig(self.dsConfig)
            #self.dsConfigCallback("dummy",dummyConfig())

    def execute(self,cmd,rt=None,tag=None,data=None):
        """Cause a command to be executed on the server.
        cmd is the python string to be exec'd, rt is the name of the variable to be returned if desired, tag is optional, and data is any data that should be sent with the command, and when the cmd is exec'd, data is referred to as "remoteData".  e.g. your cmd could be something like: "localArray[:]=remoteData" which would copy the remoteData value(s) into localArray on the server."""
        if self.conn!=None:
            if tag==None:
                tag=self.getTag()
            lst=["now",cmd,rt,tag,data]
            try:
                serialise.Send(lst,self.conn)
            except:
                self.conn=None
                raise
        else:
            tag=None
        return tag
    def getTag(self):
        self.savedTag+=1
        if self.savedTag==2**31:
            self.savedTag=2**30
        return str(self.savedTag)

    def addCallback(self,tag,method):
        self.dataProcessDict[tag]=method

    def endLoop(self):
        self.sockConn.endLoop()
    def loop(self):
        """probably start this in a new thread... listens on the socket, and
        does stuff depending on what it gets"""
        if self.verbose:
            print "Entering loop"
        self.sockConn.loop()

    def handleRawPxl(self,name,data):
        """name is stream name, data is a string - the buffer received."""
        if self.debug:
            print "handleRawPxl %s"%name
        if len(data)==32:
            #A header only - don't bother doing anything with it.
            #It probably contains info about the stream, but we don't need to know.
            return 0
        sfile=self.saveStream.get(name,None)
        if sfile!=None:
            sfile.writeRaw(data)
        #now - should I do something with it?
        if self.userCallback!=None:
            rt=self.userCallback(["raw",name,data])
        return rt
    def handlePxl(self,info):
        """info[1] is the stream name.
        info[0]=="data"
        info[2] contains data, frame time, frame number.
        Pixels are coming from the RTC socket, so we save them, and send them onto the dataswitch.
        """
        #info[0] should equal "data"
        rt=0
        if info[0]=="data":
            name=info[1]
            if self.debug:
                print "handlePxl %s"%name
            data,ftime,fno=info[2][:3]
            #print "handlepxl",info[0],name,fno
            sfile=self.saveStream.get(name,None)
            if sfile!=None:
                sfile.write(data,ftime,fno)
            if self.userCallback!=None:
                rt=self.userCallback(info)
            self.cumfreq[name]+=self.rtcDecimate[name]
            if self.dsDecimate[name]>0 and self.cumfreq[name]>=self.dsDecimate[name]:#reached the decimate...
                self.cumfreq[name]=0#so send to the dataswitch
                #Sending to the dataswitch can take a while - the dataswitch is slow.  Servicing the sockets, and saving the data is much more important.
                #So, we save the data here, and then when there is nothing to do (ie no sockets with data), we write to the dataswitch.
                if self.dataForDS.get(name)!=None:
                    print "Warning - data for %s frame %d not sent to dataswitch (new has arrived)"%(name,self.dataForDS[name][2])
                    pass
                self.dataForDS[name]=data,ftime,fno
                if self.debug:
                    print "Got data for %s"%name
                if self.dropFrames:
                    self.trySendToDataSwitch()
                else:
                    self.sendAllToDataSwitch()
        return rt
    def sendAllToDataSwitch(self):
        keys=self.dataForDS.keys()
        for k in keys:
            if self.dataForDS[k]!=None:
                self.sendToDataSwitch(k,self.dataForDS[k])
                self.dataForDS[k]=None

    def trySendToDataSwitch(self):
        """If there is nothing incoming on the socket, then we send some data to dataswitch."""
        keys=self.dataForDS.keys()
        l=len(keys)
        if self.sendIndx>l:
            self.sendIndx=0
        while self.sockConn.getNoWaiting()==0:
            i=0
            while i<l:
                name=keys[(i+self.sendIndx)%l]
                i+=1
                if self.dataForDS[name]!=None:
                    self.sendToDataSwitch(name,self.dataForDS[name])
                    self.dataForDS[name]=None
                    break#only write 1 object, before seeing if anything is waiting on sockets...
            self.sendIndx=(self.sendIndx+i)%l

#            for name in self.dataForDS.keys():
#                if self.dataForDS[name]!=None:
#                    self.sendToDataSwitch(name,self.dataForDS[name])
#                    self.dataForDS[name]=None
#                    break#only write 1 object, before seeing if anything is waiting on sockets...

    def sendToDataSwitch(self,name,data):
        #self.dsLock.acquire()
        try:
            if self.serverObject==None:
                self.connectDataSwitch()
            if self.serverObject!=None:
                data,ftime,fno=data[:3]
                a=DataSwitch.DataSwitchModule.Generic(1,data.dtype.char, int(fno),float(ftime),len(data.shape),data.shape, data.size*data.itemsize, data.tostring())
                try:
                    #self.serverObject[0].publishGeneric(a, name)
                    #print "publishGeneric..."
                    self.serverObject.publishGeneric(a, name)
                    if self.debug:
                        print "publish %s done"%name
                    #print "publishGeneric done"
                except:
                    print "Unable to publish to dataswitch"
                    raise
                    #self.connectDataSwitch()
            else:
                if self.debug:
                    print "Not able to connect to dataswitch"
        except:
            #self.dsLock.release()
            raise
        else:
            #self.dsLock.release()
            pass
        #print "handlepxl DONE"
class dummyConfigObj:
    def __init__(self,name):
        self.name=name
        self.decimate1=10
        self.decimate2=20
        self.log=0
        self.logfile=""
class dummyConfig:
    def __init__(self):
        self.generic=[dummyConfigObj("rtcCalPxlBuf"),dummyConfigObj("rtcActuatorBuf"),dummyConfigObj("rtcMirrorBuf")]

class Receiver:
    """A class that listens for things to connect and then calls callback."""
    def __init__(self,nconnect,callback,hostList,bindto=None,start=1,verbose=1,timeout=None,timeoutFunc=None):
        
        if bindto==None:#bindto can be "" which will bind to all interfaces.
            bindto=hostList
            if type(bindto)==type([]):
                bindto=bindto[0]
                print "Binding to",bingto
        self.d=dc([],lhost=bindto,nconnect=nconnect,callback=callback,closeOnFail=1,verbose=verbose,timeout=timeout,timeoutFunc=timeoutFunc)
        self.port=self.d.sockConn.port
        self.hostList=hostList#self.d.sockConn.host
        if start:
            self.thread=threading.Thread(target=self.d.loop)
            self.thread.start()
            #thread.start_new_thread(self.d.loop,())#this thread will finish when everything has connected and unconnected...


if __name__=="__main__":
    d=dc(sys.argv[1:])
    d.loop()
