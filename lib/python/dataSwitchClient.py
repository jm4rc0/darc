"""This is imported and used by rtcgui.py
"""
import numpy
import DataSwitch
import CORBA
import sys
#class publishers_handler_i(DataSwitch.DataSwitchModule__POA.PublishersSubscriber):
class publishers_handler_i(DataSwitch.DataSwitchModule__POA.PublishersSubscriber,DataSwitch.Subscriber):
    def __init__(self,dataswitch,msg="publishers",callback=None):
        DataSwitch.Subscriber.__init__(self, dataswitch)
        self.msg=msg
        self.callback=callback
    def update(self, status):
        print self.msg,status
        if self.callback!=None:
            self.callback(self.msg,status)

class config_handler_i(DataSwitch.DataSwitchModule__POA.ConfigSubscriber,DataSwitch.Subscriber):
    def __init__(self,dataswitch,msg="config",callback=None):
        DataSwitch.Subscriber.__init__(self, dataswitch)
        self.msg=msg
        self.callback=callback
    def update(self, status):
        print self.msg,status
        if self.callback!=None:
            self.callback(self.msg,status)

class generic_handler_i(DataSwitch.DataSwitchModule__POA.GenericSubscriber,DataSwitch.Subscriber):
    def __init__(self,dataswitch,msg="generic",callback=None):
        DataSwitch.Subscriber.__init__(self, dataswitch)
        self.msg=msg
        self.callback=callback
        print "Subscribing",msg
    def update(self, status):
        #status has status.frameNo,status.dataType,status.len,status.timestamp,status.data
        if self.callback!=None:
            self.callback(self.msg,status)

class DataSwitchClient:
    def __init__(self,streamListCallback=None,configCallback=None):
        """streamListCallback - a function to call when stream list changes.  Args are a list of new streams, and a list of removed streams.
        """
        self.streamList=[]
        self.streamListCallback=streamListCallback
        self.configCallback=configCallback
        self.pubHandler=None
        self.confHandler=None
        self.serverObj=None
        self.handlerDict={}
        self.connectDataSwitch()
    def connectDataSwitch(self):#,p,h):
        self.subscribedList=[]
        print "Attempting to connect to dataswitch..."
        try:
            self.serverObj = DataSwitch.connect()#"DataSwitch")
        except:
            print "Unable to connect to dataswitch"
        #print self.serverObj
        re=False
        try:
            #self.serverObj[0].subscribePublishers(publishers_handler_i(callback=self.publishersCallback)._this())
            #self.serverObj[0].subscribeConfig(config_handler_i(callback=self.configChanged)._this())
            if self.pubHandler==None:
                self.pubHandler=publishers_handler_i(self.serverObj,callback=self.publishersCallback)
            self.serverObj.subscribePublishers(self.pubHandler._this())
            if self.confHandler==None:
                self.confHandler=config_handler_i(self.serverObj,callback=self.configChanged)
            self.serverObj.subscribeConfig(self.confHandler._this())
            print "subscribed to pubishers and config"
        except CORBA.COMM_FAILURE, ex:
            print ex
            self.serverObj=None
            print "CORBA.COMM_FAILURE in connectDataSwitch"
        except CORBA.TRANSIENT, ex:
            print ex
            self.serverObj=None
            print "CORBA.Transient error in connectDataSwitch"
        except:
            print "Unable to subscribe to dataswitch"
            print sys.exc_info()
            self.serverObj=None
        #if self.serverObj==None:
        #    gobject.timeout_add(1000,self.connectDataSwitch)
        print self.serverObj
        if self.serverObj!=None:
            print "Connected to dataswitch"
        return False

    def publishersCallback(self,msg,status):
        remList=[]
        for s in self.streamList:
            if s not in status.streams:
                remList.append(s)
                print "stream %s removed"%s
        self.streamList=status.streams[:]
        if self.streamListCallback!=None:
            self.streamListCallback(self.streamList,remList)

    def configChanged(self,msg,config):
        print "config",msg,config
        if self.configCallback!=None:
            self.configCallback(msg,config)

    def subscribe(self,stream,dec,callback):
        retry=0
        print "subscribing to %s dec %d"%(stream,dec)
        if self.serverObj==None:
            print "Connecting to dataswitch..."
            self.connectDataSwitch()
        try:
            #self.serverObj[0].subscribeGeneric(generic_handler_i(msg=stream,callback=callback)._this(),stream,dec)
            if not self.handlerDict.has_key(stream):
                self.handlerDict[stream]=generic_handler_i(self.serverObj,msg=stream,callback=callback)
                
            self.serverObj.subscribeGeneric(self.handlerDict[stream]._this(),stream,dec)
        except:
            print "Attempting to connect to dataswitch"
            self.connectDataSwitch()
            retry=1
        if retry and self.serverObj!=None:
            #self.serverObj[0].subscribeGeneric(generic_handler_i(msg=stream,callback=callback)._this(),stream,dec)
            if not self.handlerDict.has_key(stream):
                self.handlerDict[stream]=generic_handler_i(self.serverObj,msg=stream,callback=callback)
            self.serverObj.subscribeGeneric(self.handlerDict[stream]._this(),stream,dec)
