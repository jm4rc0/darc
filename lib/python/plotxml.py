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
                self.plotList.append(attrs)
            elif name=="mangle":
                if "plot" in self.intagList:
                    pass
            elif name=="sub":
                if "plot" in self.intagList:
                    pass
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

    def getPlots(self,group=None):
        """Group can be something that defines this group of plots... lor None."""
        p=[]
        for attr in self.plotList:
            for i in range(len(attr["sub"])):
                s=attr["sub"][i]
                if len(s)==3:#need to add the change flag - set to 1.
                    attr["sub"][i]=(s[0],s[1],s[2],1)
            p.append([attr["pos"],attr["size"],attr["show"],attr["mangle"],attr["sub"],attr["tbVal"],group])
        return p
