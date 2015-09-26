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
import xml.parsers.expat
class parseXml:
    def __init__(self,txt=""):
        self.p = xml.parsers.expat.ParserCreate()
        self.p.StartElementHandler = self.startElement
        self.p.EndElementHandler = self.endElement
        self.p.CharacterDataHandler = self.charData
        self.p.returns_unicode=0
        self.initialise()
        self.parse(txt)
    def initialise(self):
        self.plotList=[]
        self.intagList=[]
        self.storedTxt=None
    def parse(self,txt):
        if len(txt.strip())>0:
            self.p.Parse(txt)

    def startElement(self,name,attrs):
        self.intagList.append(name)
        #if len(self.intagList)>2 and self.intagList[-2]=="simSetup" and self.intagList[-3]=="aosim":
        if "displayset" in self.intagList:
            self.storedTxt=""
            if name=="plot":
                if attrs.has_key("size"):
                    attrs["size"]=eval(attrs["size"])
                if attrs.has_key("pos"):
                    attrs["pos"]=eval(attrs["pos"])
                if attrs.has_key("show"):
                    attrs["show"]=int(attrs["show"])
                if attrs.has_key("sub"):
                    attrs["sub"]=eval(attrs["sub"])
                if attrs.has_key("tbVal"):
                    attrs["tbVal"]=eval(attrs["tbVal"])
                else:
                    attrs["tbVal"]=(0,0,0)
                attrs["initcode"]=""
                attrs["mangle"]=""
                attrs["sub"]=[]
                attrs["paramsub"]=[]
                self.plotList.append(attrs)
            elif name=="mangle":
                if "plot" in self.intagList:
                    pass
            elif name=="sub":
                if "plot" in self.intagList:
                    pass
            elif name=="initcode":
                if "plot" in self.intagList:
                    pass
            elif name=="paramsub":
                if "plot" in self.intagList:
                    o=self.plotList[-1]
                    o["paramsub"].append(attrs["sub"])
    def charData(self,data):
        self.storedTxt+=data
    def endElement(self,name):
        n=self.intagList.pop()
        if n!=name:
            print "Parse error - muddled up somehow %s %s"%(n,name)
            raise Exception("XML parse error")
        if name=="mangle":
            o=self.plotList[-1]
            o["mangle"]=self.storedTxt
        elif name=="sub":
            o=self.plotList[-1]
            o["sub"]=eval(self.storedTxt)
        elif name=="initcode":
            o=self.plotList[-1]
            o["initcode"]=self.storedTxt

    def getPlots(self,group=None):
        """Group can be something that defines this group of plots... lor None."""
        p=[]
        for attr in self.plotList:
            for i in range(len(attr["sub"])):
                s=attr["sub"][i]
                if len(s)==3:#need to add the change flag - set to 1.
                    attr["sub"][i]=(s[0],s[1],s[2],1)
            p.append([attr.get("pos"),attr.get("size"),attr["show"],attr["mangle"],attr["sub"],attr["tbVal"],group,attr["initcode"],attr["paramsub"]])
        return p
