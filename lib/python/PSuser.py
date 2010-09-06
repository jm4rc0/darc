import PS
import rtc.Command
DATAOBJ=rtc.Command.DATA_OBJ
class DictionaryHandler(PS.DictionaryStreamHandler):
    def __init__(self,callback=None):
        PS.DictionaryStreamHandler.__init__(self, DATAOBJ)
        self.dataList=[]
        self.callback=callback

    def log(self, msg):
        PS.log(self.getPS(), msg, 'ExampleHandler')

    def update(self, data):
        l=len(data.name)
        if l!=len(data.value):
            print "Error - Dictionary unbalanced"
        d={}
        for i in range(l):
            d[data.name[i]]=data.value[i]
        if self.callback!=None:
            self.callback(d)
        else:
            print d

#             dl=[]
#             remlist=[]
#             for i in range(l):
#                 if data.value[i]=="data":
#                     dl.append(data.name)
#             for s in self.dataList:
#                 if s not in dl:
#                     remlist.append(s)
#                     print "data stream %s removed"%s
#             self.dataList=dl
#             if self.callback!=None:
#                 self.callback(self.dataList,remlist)

class DataHandler(PS.DataStreamHandler):
    def __init__(self,stream,callback=None):
        PS.DataStreamHandler.__init__(self, DATAOBJ)
        self.stream=stream
        self.dataList=[]
        self.callback=callback

    def log(self, msg):
        PS.log(self.getPS(), msg, 'ExampleHandler')

    def update(self, data):
        #print data#data.count,data.time,data.dataType,data.dims,data.params,data.data,data.valid(==1)
        if self.callback!=None:
            self.callback(self.stream,data)
        else:
            print self.stream,data



class DecimateConfigEntry:
    def __init__(self,name="",decimate1=0,decimate2=0,logFile=None,log=0):
        self.name=name
        self.decimate1=decimate1
        self.decimate2=decimate2
        self.log=log
        if logFile==None:
            if len(name)==0:
                name="log"
            self.logFile=name+".fits"
        else:
            self.logFile=logFile

class DecimateConfig:
    def __init__(self):
        self.generic=[]#a list of DecimateConfigEntry objects.
