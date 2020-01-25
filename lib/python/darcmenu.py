import sys
import traceback
import numpy
import urwid
import darc


class ExitEdit(urwid.Edit):
    """Closes self on return or escape"""
    def __init__(self,txt,parent,edittxt="",exitOnEnter=1):
        super(ExitEdit,self).__init__(txt,edittxt)
        self.parent=parent
        self.orig=edittxt
        self.exitOnEnter=exitOnEnter
    def keypress(self,size,key):
        key=super(ExitEdit,self).keypress(size,key)
        if key=="enter":
            if self.exitOnEnter:
                indx=self.parent.index(self)-1
                if indx<0:
                    indx=0
                self.parent.remove(self)
                self.parent.set_focus(indx)
            urwid.emit_signal(self,"change",self,self.get_edit_text(),1)
        elif key=="esc":
            self.set_edit_text(self.orig)
            indx=self.parent.index(self)-1
            if indx<0:
                indx=0
            self.parent.remove(self)
            self.parent.set_focus(indx)
    def connect(self,what,func):
        urwid.connect_signal(self,what,func)


class EndEdit(urwid.Edit):
    """Emits change on return"""
    def __init__(self,txt,parent,edittxt=""):
        super(EndEdit,self).__init__(txt,edittxt)
        self.parent=parent
        self.orig=edittxt
    def keypress(self,size,key):
        #key=super(EndEdit,self).keypress(size,key)
        if key=="enter":
            urwid.emit_signal(self,"change",self,self.get_edit_text(),1)
        elif key=="esc":
            self.set_edit_text(self.orig)
            urwid.emit_signal(self,"change",self,self.orig,2)
        else:
            return super(EndEdit,self).keypress(size,key)
    def connect(self,what,func):
        urwid.connect_signal(self,what,func)

class SelText(urwid.Edit):
    def __init__(self,txt,edittxt=""):
        super(SelText,self).__init__(txt)

    def keypress(self,size,key):
        if key=="enter":
            urwid.emit_signal(self,"change",self,self.get_edit_text(),1)
        elif key=="esc":
            urwid.emit_signal(self,"change",self,self.get_edit_text(),2)
        else:
            return super(SelText,self).keypress(size,key)

    def connect(self,what,func):
        urwid.connect_signal(self,what,func)

class Menu:
    def __init__(self,prefix=""):
        self.prefix=prefix
        self.preprefix=""#to revert back to on escape.
        self.widgets=self.display()
        self.remList=[]
    def display(self,wid=None):

        if wid==None:
            #display main menu
            rt=urwid.ListBox(urwid.SimpleFocusListWalker([]))
            b=urwid.Button("Prefix: %s"%self.prefix)
            urwid.connect_signal(b,"click",self.buttonClick)
            rt.body.append(b)
            b=urwid.Button("Set")
            self.SetButton=b
            urwid.connect_signal(b,"click",self.buttonClick)
            rt.body.append(b)
            b=urwid.Button("Get")
            self.GetButton=b
            urwid.connect_signal(b,"click",self.buttonClick)
            rt.body.append(b)
            b=urwid.Button("Status")
            self.StatusButton=b
            urwid.connect_signal(b,"click",self.buttonClick)
            rt.body.append(b)
            b=urwid.Button("Telemetry")
            self.TelemetryButton=b
            urwid.connect_signal(b,"click",self.buttonClick)
            rt.body.append(b)
            b=urwid.Button("Quit")
            urwid.connect_signal(b,"click",self.quit)
            rt.body.append(b)

            #rt=urwid.ListBox(urwid.SimpleFocusListWalker([urwid.Button("Prefix: %s"%self.prefix),urwid.Button("Set"),urwid.Button("Get"),urwid.Button("Status"),urwid.Button("Telemetry")]))
        else:#display submenu
            print "Here"

        return rt

    def quit(self,e):
        raise urwid.ExitMainLoop()

    def prefixChange(self,e,txt,final=0):
        self.prefix=txt
        self.widgets.body[0].set_label("Prefix: %s"%self.prefix)
    # def cancelPrefixChange(self,e):
    #     self.prefix=self.preprefix
    #     self.widgets.body[0].set_label("Prefix: %s"%self.prefix)
        

    # def endPrefix(self,e,click=None):
    #     for r in self.remList:
    #         if r in self.widgets.body:
    #             self.widgets.body.remove(r)
    #     self.remList=[]

    def getChanged(self,e,txt=None,final=0):
        if final==1:#user has pressed return...
            try:
                d=darc.Control(self.prefix)
                if len(txt)==0:
                    labels=d.GetLabels()
                    labels.sort()
                    txt=""
                    for l in labels:
                        data=d.Get(l)
                        txt+=l+": "
                        if type(data)==numpy.ndarray and data.size>100:
                            txt+="Array dtype %s shape %s\n"%(data.dtype.char,data.shape)
                        else:
                            txt+=str(data)+"\n"
                    else:
                        print txt


                    
                else:
                    txt=str(d.Get(txt))
            except:
                txt=traceback.format_exc()
            lab=urwid.Text(txt)
            indx=self.widgets.body.index(self.GetButton)+2
            self.widgets.body.insert(indx,lab)
        elif final==2:
            indx=self.widgets.body.index(e)
            while type(self.widgets.body[indx])!=urwid.Button:
                self.widgets.body.remove(self.widgets.body[indx])

    def buttonClick(self,but=None):
        #self.prefix+=str(but)
        if but.get_label().startswith("Prefix:"):
            e=ExitEdit("Prefix:",self.widgets.body,self.prefix)
            indx=self.widgets.body.index(but)+1
            self.widgets.body.insert(indx,e)
            self.widgets.set_focus(indx)
            urwid.connect_signal(e,"change",self.prefixChange)
            #urwid.connect_signal(e,"escape",self.cancelPrefixChange)
            self.preprefix=self.prefix
            #e=urwid.Edit("New prefix: ")
            #self.remList.append(e)
            #self.widgets.body.insert(1,e)
            #self.widgets.set_focus(1)
            #urwid.connect_signal(e,"change",self.prefixChange)
            #urwid.connect_signal(self.widgets.body,"modified",self.focusChange)
        elif but.get_label()=="Set":
            indx=self.widgets.body.index(but)+1
            typ=type(self.widgets.body[indx])
            if typ!=urwid.Button:#already something here, so remove it
                while type(self.widgets.body[indx])!=urwid.Button:
                    self.widgets.body.remove(self.widgets.body[indx])
            else:
                e=EndEdit("Parameter:",self.widgets.body)
                self.widgets.body.insert(indx,e)
                self.widgets.set_focus(indx)
                #e.connect("change",self.setValue)
                e2=EndEdit("Value:",self.widgets.body)
                self.widgets.body.insert(indx+1,e2)
                e2.connect("change",self.setValue)
        elif but.get_label()=="TestGet":
            e=ExitEdit("Parameter to get (empty for all):",self.widgets.body)
            indx=self.widgets.body.index(but)+1
            self.widgets.body.insert(indx,e)
            self.widgets.set_focus(indx)
            e.connect("change",self.getChanged)
        elif but.get_label()=="Get":
            indx=self.widgets.body.index(but)+1
            typ=type(self.widgets.body[indx])
            if typ==EndEdit or typ==urwid.Text:
                while type(self.widgets.body[indx])!=urwid.Button:
                    #button clicked - so remove the edit/param...
                    self.widgets.body.remove(self.widgets.body[indx])
            else:
                e=EndEdit("Parameter to get (empty for all):",self.widgets.body)
                self.widgets.body.insert(indx,e)
                self.widgets.set_focus(indx)
                urwid.connect_signal(e,"change",self.getChanged)
        elif but.get_label()=="Status":
            self.getStatus()
        elif but.get_label()=="Telemetry":
            self.getTelemetry()

    def getStatus(self):
        try:
            d=darc.Control(self.prefix)
            txt=darc.statusBufToString(d.GetStream("rtcStatusBuf")[0]).strip("\0")
        except:
            txt=traceback.format_exc()
        indx=self.widgets.body.index(self.StatusButton)+1
        if type(self.widgets.body[indx])==SelText:
            self.widgets.body[indx].set_caption(txt)
        else:
            si=SelText(txt,"")
            si.connect("change",self.removeStatus)
            self.widgets.body.insert(indx,si)
    def removeStatus(self,e,txt,final=0):
            self.widgets.body.remove(e)

    def setValue(self,e,txt,final=0):
        if final==1:#return pressed
            indx=self.widgets.body.index(self.SetButton)
            name=self.widgets.body[indx+1].get_edit_text()
            value=self.widgets.body[indx+2].get_edit_text()
            w=self.widgets.body[indx+3]
            try:
                d=darc.Control(self.prefix)
                err=d.Set(name,eval(value))
            except:
                txt=traceback.format_exc()
            else:
                if len(err)==0:
                    txt="Set %s"%name
                else:
                    txt="Error setting %s"%name
            w=self.widgets.body[indx+3]
            if type(w)==urwid.Button:
                w=urwid.Text(txt)
                self.widgets.body.insert(indx+3,w)
            else:
                w.set_text(txt)
        elif final==2:#escape pressed:
            indx=self.widgets.body.index(self.SetButton)
            self.widgets.set_focus(indx)
    def focusChange(self,e=None):
        #urwid.disconnect_signal(self.widgets.body,"modified",self.focusChange)
        self.prefixChange(None,str(self.remList))
        for r in self.remList:
            if r in self.widgets.body:
                self.widgets.body.remove(r)#self.widgets.body[1])
        self.remList=[]
    def unhandledInput(self,key):
        if key=="enter":
            for r in self.remList:
                if r in self.widgets.body:
                    self.widgets.body.remove(r)#elf.widgets.body[1])
            self.remList=[]

    def getTelemetry(self):
        indx=self.widgets.body.index(self.TelemetryButton)+1
        if type(self.widgets.body[indx])==urwid.Button:
            #get telemetry...
            dec=None
            txt=""
            try:
                d=darc.Control(self.prefix)
                dec=d.GetDecimation()
            except:
                txt=traceback.format_exc()
            if dec==None:
                t=urwid.Text(txt)
                self.widgets.body.insert(indx,t)
            else:
                plist=[]
                for i in range(3):
                    plist.append(urwid.Pile([]))
                cols=urwid.Columns(plist,1)

        else:#finish telemetry
            while type(self.widgets.body[indx])!=urwid.Button:
                self.widgets.body.remove(self.widgets.body[indx])



if __name__=="__main__":
    prefix=""
    for a in sys.argv[1:]:
        if a.startswith("--prefix="):
            prefix=a[9:]
    
m=Menu(prefix)
urwid.MainLoop(m.widgets,unhandled_input=m.unhandledInput).run()
