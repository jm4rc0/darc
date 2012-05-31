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
import os
if os.environ.has_key("OS") and os.environ["OS"]=="Windows_NT":
    devshm="c:/RTC/shm/"
else:
    import serialise
    devshm="/dev/shm/"

"""
show how to add a matplotlib FigureCanvasGTK or FigureCanvasGTKAgg widget and
a toolbar to a gtk.Window
"""
import traceback
#import Numeric,RandomArray
import numpy,numpy.random
#from matplotlib.axes import Subplot
from matplotlib.figure import Figure
#from matplotlib.numerix import arange, sin, pi
import thread
import time
import FITS
# uncomment to select /GTK/GTKAgg/GTKCairo
from matplotlib.backends.backend_gtk import FigureCanvasGTK as FigureCanvas
#from matplotlib.backends.backend_gtkagg import FigureCanvasGTKAgg as FigureCanvas
#from matplotlib.backends.backend_gtkcairo import FigureCanvasGTKCairo as FigureCanvas

# or NavigationToolbarfor classic
from matplotlib.backends.backend_gtk import NavigationToolbar2GTK as NavigationToolbar
#from matplotlib.backends.backend_gtkagg import NavigationToolbar2GTKAgg as NavigationToolbar

    


from matplotlib.figure import FigureImage
#import pylab
import matplotlib.cm as colour
#from matplotlib import interactive
#interactive(True)
import gtk,gobject
import pango
import sys
import subprocess
import threading
import serialise
import socket
import plotxml
from startStreams import WatchDir
#from gui.myFileSelection.myFileSelection import myFileSelection



## def mysave(toolbar=None,button=None,c=None):
##         print "mypylabsave"
##         print toolbar.canvas,dir(toolbar.canvas)
##         print type(toolbar.canvas._pixmap)
##         data=toolbar.canvas.get_data()
##         print type(data)
##         fn=myFileSelection("Open an xml parameter file",self.oldfilename,parent=self.gladetree.get_widget("window1")).fname
##         if fn!=None:
##             if fn[-5:] not in [".fits",".FITS"]:
##                 print "Only saving as FITS is supported"
##             else:
##                 util.FITS.Write(self.data,fn)
##                 print "Data saved as %s"%fn

## NavigationToolbar.save_figure=mysave


        
X=numpy.random.random((20,20)).astype("f")
class myToolbar:
    def __init__(self,plotfn=None,label="",loadFunc=None,streamName=""):
        """plotfn is a function to call to replot..."""
        self.data=None
        self.label=label
        l=label
        if len(label)>0:
            l=label+", "
        self.hostnametxt=" [%s%s]"%(l,os.environ.get("HOSTNAME","unknown host"))
        self.loadFunc=loadFunc
        if plotfn!=None:
            self.replot=plotfn
        else:
            self.replot=self.dummyreplot
        self.autoscale=1
        self.freeze=0
        self.logx=0
        self.tbList=[]
        self.tbVal=[]
        self.store={}#can be used by mangle to save inbetween plots...
        self.stream={}#can be uused to store dtaa from all streams.
        self.streamName=streamName
        self.streamTime={}#stores tuples of fno,ftime for each stream
        self.streamTimeTxt=""#text info to show...
        self.subapLocation=None#needed for centroid overlays.
        self.npxlx=None#needed for centroid overlays.
        self.npxly=None#needed for centroid overlays.
        self.nsub=None#needed for centroid overlays.
        #self.nsuby=None#needed for centroid overlays.
        self.subscribeDict={}
        self.subapFlag=None#needed for centroid overlays.
        self.dataCopy=None
        self.mangleTxt=""
        self.mangleTxtDefault=""
        self.scale=[0,1]
        self.toolbar=gtk.VBox()
        self.hbox=gtk.HBox()
        self.hbox2=gtk.HBox()
        #self.tooltips=gtk.Tooltips()
        self.reprbutton=gtk.Button("Repr")
        self.reprbutton.connect("clicked",self.repr)
        #self.tooltips.set_tip(self.reprbutton,"Textual representation")
        self.reprbutton.set_tooltip_text("Textual representation")
        self.savebutton=gtk.Button("Save")
        self.savebutton.connect("clicked",self.savePlot)
        #self.tooltips.set_tip(self.savebutton,"Save unmodified as FITS")
        self.savebutton.set_tooltip_text("Save configuration (as xml) or data (unmodified as FITS)")
        self.loadbutton=gtk.Button("Load")
        self.loadbutton.connect("clicked",self.loadPlot)
        #self.tooltips.set_tip(self.loadbutton,"Load a FITS file to replace current")
        self.loadbutton.set_tooltip_text("Load a xml configuration or data FITS file to replace current")
        
        self.freezebutton=gtk.CheckButton("Freeze")
        self.freezebutton.set_active(self.freeze)
        self.freezebutton.connect("toggled",self.togglefreeze,None)
        #self.tooltips.set_tip(self.freezebutton,"freeze display")
        self.freezebutton.set_tooltip_text("freeze display")
        self.autobutton=gtk.CheckButton("Scaling")
        self.autobutton.set_active(self.autoscale)
        self.autobutton.connect("toggled",self.toggleAuto,None)
        #self.tooltips.set_tip(self.autobutton,"autoscale data")
        self.autobutton.set_tooltip_text("autoscale data")
        self.scaleMinEntry=gtk.Entry()
        self.scaleMinEntry.connect("focus-out-event",self.rescale,"min")
        self.scaleMinEntry.connect("activate",self.rescale,"min")
        self.scaleMinEntry.set_width_chars(8)
        #self.tooltips.set_tip(self.scaleMinEntry,"Minimum value to clip when not autoscaling")
        self.scaleMinEntry.set_tooltip_text("Minimum value to clip when not autoscaling")
        self.scaleMaxEntry=gtk.Entry()
        self.scaleMaxEntry.connect("focus-out-event",self.rescale,"max")
        self.scaleMaxEntry.connect("activate",self.rescale,"max")
        self.scaleMaxEntry.set_width_chars(8)
        self.scaleMaxEntry.set_tooltip_text("Maximum value to clip when not autoscaling")
        self.logxbutton=gtk.CheckButton("Logx")
        self.logxbutton.set_active(self.logx)
        self.logxbutton.connect("toggled",self.togglelogx,None)
        self.logxbutton.set_tooltip_text("Logaritm of x axis for 1d plots")
        self.dataMangleEntry=gtk.TextView()#gtk.Entry()
        self.dataMangleEntry.connect("focus-out-event",self.dataMangle,None)
        #self.dataMangleEntry.connect("activate",self.dataMangle,None)
        self.dataMangleEntry.set_tooltip_text("Formatting to perform on data prior to plotting, e.g. data=numpy.log(data) (this gets exec'd).  You can also use this to create an overlay, e.g. overlay=numpy.zeros((10,10,4));overlay[::4,::4,::3]=1 to create an overlay of red dots.")
        self.scrollMangle=gtk.ScrolledWindow()
        self.scrollMangle.add(self.dataMangleEntry)
        self.hbox.pack_start(self.freezebutton)
        self.hbox.pack_start(self.reprbutton)
        self.hbox.pack_start(self.savebutton)
        self.hbox.pack_start(self.loadbutton)
        self.hbox.pack_start(self.autobutton)
        self.hbox.pack_start(self.scaleMinEntry)
        self.hbox.pack_start(self.scaleMaxEntry)
        self.hbox.pack_start(self.logxbutton)
        self.toolbar.pack_start(self.hbox,False,False)
        #self.hbox2.pack_start(self.dataMangleEntry)
        self.hbox2.pack_start(self.scrollMangle,expand=True,fill=True)
        self.toolbar.pack_start(self.hbox2,expand=True,fill=True)
        self.toolbar.show_all()
    def dummyreplot(self):
        print "Replot data... (doing nowt)"
    def toggleAuto(self,w,data=None):
        self.autoscale=self.autobutton.get_active()
        self.replot()

    def rescale(self,w,e,data=None):
        if data==None:
            data=e
        if data=="min":
            indx=0
        else:
            indx=1
        try:
            self.scale[indx]=float(w.get_text())
        except:
            pass
        self.replot()
                
    def togglefreeze(self,w,data=None):
        self.freeze=self.freezebutton.get_active()
        if not self.freeze:
            self.replot()
    def togglelogx(self,w,data=None):
        self.logx=self.logxbutton.get_active()
        self.replot()
    def dataMangle(self,w,e=None,data=None):
        #txt=w.get_text().strip()
        buf=w.get_buffer()
        txt=buf.get_text(buf.get_start_iter(),buf.get_end_iter())
        if self.mangleTxt!=txt:
            self.mangleTxt=txt
            self.replot()
    def makeArr(self,arr,shape,dtype):
        """This is used if you want to plot a history.
        eg in mangle you would put something like:
        store=makeArr(store,(1000,),"f");store[:999]=store[1:];store[999]=data[1];data=store
        """
        if arr==None or arr.shape!=shape or arr.dtype.char!=dtype:
            arr=numpy.zeros(shape,dtype)
        return arr

    def setUserButtons(self,tbVal,tbNames):
        pass
    def prepare(self,data,dim=2,overlay=None,arrows=None,axis=None,plottype=None):
        self.origData=data
        title=self.streamName+self.hostnametxt
        streamTimeTxt=self.streamTimeTxt
        freeze=self.freeze
        colour=None
        text=None
        fount=None
        fast=0
        if self.freeze==0:
            if type(data)!=numpy.ndarray:
                data=numpy.array([data])
            if self.data==None or type(self.data)==type("") or self.data.shape!=data.shape or self.data.dtype.char!=data.dtype.char:
                self.data=data.copy()
            else:
                self.data[:]=data
            data=self.data
            if len(self.mangleTxt)>0:
                mangleTxt=self.mangleTxt
            else:
                mangleTxt=self.mangleTxtDefault
            if len(mangleTxt)>0:
                d={"data":data,"numpy":numpy,"overlay":overlay,"store":self.store,"makeArr":self.makeArr,"title":self.streamName,"stream":self.stream,"streamTime":self.streamTime,"streamTimeTxt":self.streamTimeTxt,"subapLocation":self.subapLocation,"freeze":0,"tbVal":self.tbVal[:],"debug":0,"dim":dim,"arrows":arrows,"npxlx":self.npxlx,"npxly":self.npxly,"nsub":self.nsub,"subapFlag":self.subapFlag,"quit":0,"colour":colour,"text":None,"axis":axis,"plottype":plottype,"fount":None}
                try:
                    exec mangleTxt in d
                    data=d["data"]#the new data... after mangling.
                    overlay=d["overlay"]#the new overlay
                    self.store=d["store"]
                    title=d["title"]
                    if title==self.streamName:#unchanged
                        title+=self.hostnametxt
                    streamTimeTxt=d["streamTimeTxt"]
                    freeze=d["freeze"]
                    tbNames=d.get("tbNames")#could be None
                    tbVal=d.get("tbVal")
                    fount=d.get("fount")
                    fast=d.get("fast",0)
                    self.setUserButtons(tbVal,tbNames)
                    #if d.has_key("tbNames") and type(d["tbNames"])==type([]):
                    #    for i in range(min(len(self.tbList),len(d["tbNames"])):
                    #        self.tbList[i].set_label(d["tbNames"][i])
                    dim=d["dim"]
                    if dim==None:
                        dim=2
                    if dim>2:
                        dim=2
                    if type(data)==numpy.ndarray:
                        dim=min(dim,len(data.shape))
                    arrows=d["arrows"]
                    colour=d["colour"]
                    text=d["text"]
                    if type(text)==type(""):
                        text=[[text,0,0]]
                    elif type(text) in [type([]),type(())]:
                        if len(text)>0:
                            if type(text[0])==type(""):
                                text=[text]
                    #if title==None:
                    #    title=self.stream
                    axis=d["axis"]
                    plottype=d["plottype"]
                except:
                    if d["debug"]:
                        print sys.exc_info()
                        traceback.print_exc()
                if d["quit"]:
                    sys.exit(0)

            if type(data)==type(""):
                pass
            else:
                if freeze==0:
                    if dim==2:#see if dimensions have changed...
                        dim=min(2,len(data.shape))
                    if self.logx==1 and dim==2:
                        #take log of the data...
                        m=numpy.min(data.ravel())
                        if m<0:
                            data+=0.1-m#now ranges from 0 upwards
                        elif m==0.:#get rid of any zeros...
                            data+=0.1
                        data=numpy.log(data)
                    if self.autoscale:
                        tmp=data.flat
                        #if data.flags.contiguous:
                        #    tmp=data.flat
                        #else:
                        #    tmp=numpy.array(data).flat
                        self.scale[0]=numpy.min(tmp)
                        self.scale[1]=numpy.max(tmp)
                        self.scaleMinEntry.set_text("%.4g"%(self.scale[0]))
                        self.scaleMaxEntry.set_text("%.4g"%(self.scale[1]))
                    if dim==1:
                        data[:]=numpy.where(data<self.scale[0],self.scale[0],data)
                        data[:]=numpy.where(data>self.scale[1],self.scale[1],data)
        
            
            if freeze==0 and type(overlay)==numpy.ndarray and len(overlay.shape)==3 and overlay.shape[2]==4 and len(data.shape)==2:
                # an overlay has been used...
                pass
            else:
                overlay=None
        self.data=data
        return freeze,self.logx,data,self.scale,overlay,title,streamTimeTxt,dim,arrows,colour,text,axis,plottype,fount,fast
##     def mysave(self,toolbar=None,button=None,c=None):
##         print "mypylabsave"
##         print a,b,c
##         fn=myFileSelection("Open an xml parameter file",self.oldfilename,parent=self.gladetree.get_widget("window1")).fname
##         if fn!=None:
##             if fn[-5:] not in [".fits",".FITS"]:
##                 print "Only saving as FITS is supported"
##             else:
##                 util.FITS.Write(self.data,fn)
##                 print "Data saved as %s"%fn

    def repr(self,w=None,a=None):
        Repr(self.data,self.label)

    def fileaddtimestamp(self,w,f=None):
        #print w,f
        f.selection_entry.insert_text(time.strftime("%y%m%d-%H%M%S"),f.selection_entry.get_position())

    def savePlot(self,w=None,a=None):
        f=gtk.FileSelection("Save as FITS or xml")
        #f.complete("*.fits")
        #f.ok_button.parent.pack_
        b=f.add_button("Insert timestamp",999)
        b.connect("clicked",self.fileaddtimestamp,f)
        b.parent.reorder_child(b,0)
        b=f.add_button("Save unformatted",998)
        b.connect("clicked",self.filesaveorig,f)
        b.parent.reorder_child(b,1)
        #print b.parent
        f.set_modal(1)
        f.set_position(gtk.WIN_POS_MOUSE)
        f.connect("destroy",self.filecancel,f)
        f.ok_button.connect("clicked",self.filesave,f)
        f.cancel_button.connect("clicked",self.filecancel,f)
        f.show_all()

    def loadPlot(self,w=None,a=None):
        f=gtk.FileSelection("Load FITS file/xml file")
        #f.complete("*.fits")
        f.set_modal(1)
        f.set_position(gtk.WIN_POS_MOUSE)
        f.connect("destroy",self.filecancel,f)
        f.ok_button.connect("clicked",self.fileload,f)
        f.cancel_button.connect("clicked",self.filecancel,f)
        f.show_all()

    def filesaveorig(self,w,f):
        fname=f.get_filename()#selection_entry.get_text()
        self.filecancel(w,f)
        print "Saving shape %s, dtype %s"%(str(self.origData.shape),str(self.origData.dtype.char))
        FITS.Write(self.origData,fname)
    def filesave(self,w,f):
        fname=f.get_filename()#selection_entry.get_text()
        self.filecancel(w,f)
        if fname[-5:]==".fits":
            print "Saving shape %s, dtype %s"%(str(self.data.shape),str(self.data.dtype.char))
            FITS.Write(self.data,fname)
        elif fname[-4:]==".xml":
            print "Saving config"
            pos=self.savebutton.get_toplevel().get_position()
            size=self.savebutton.get_toplevel().get_size()
            vis=0
            tbVal=self.tbVal
            mangleTxt=self.mangleTxt
            subscribeList=[]
            for key in self.subscribeDict.keys():
                subscribeList.append((key,self.subscribeDict[key][0],self.subscribeDict[key][1]))
            if os.path.exists(fname):
                plotList=plotxml.parseXml(open(fname).read()).getPlots()
            else:
                plotList=[]
            plotList.append([pos,size,vis,mangleTxt,subscribeList,tbVal,None])
            txt='<displayset date="%s">\n'%time.strftime("%y/%m/%d %H:%M:%D")
            for data in plotList:
                txt+='<plot pos="%s" size="%s" show="%d" tbVal="%s">\n<mangle>%s</mangle>\n<sub>%s</sub>\n</plot>\n'%(str(data[0]),str(data[1]),data[2],str(tuple(data[5])),data[3],str(data[4]))
            txt+="</displayset>\n"
            open(fname,"w").write(txt)

            
        else:
            print "Nothing saved"


    def fileload(self,w,f):
        fname=f.get_filename()#selection_entry.get_text()
        self.filecancel(w,f)
        if self.loadFunc!=None:
            try:
                self.loadFunc(fname,reposition=0)
            except:
                traceback.print_exc()


    def filecancel(self,w,f):
        f.set_modal(0)
        f.destroy()
    
class Repr:
    def __init__(self,data,label="Text representation"):
        self.win=gtk.Window()
        self.win.connect("destroy",self.quit)
        self.win.set_default_size(640,480)
        self.win.set_title(label)
        self.label=label
        self.sw=gtk.ScrolledWindow()
        self.win.add(self.sw)
        po=numpy.get_printoptions()["threshold"]
        numpy.set_printoptions(threshold=2**31)
        if type(data)==numpy.ndarray:
            st="Array shape=%s, dtype=%s\n"%(str(data.shape),str(data.dtype.char))
        else:
            st=""
        st+=str(data)
        numpy.set_printoptions(threshold=po)
        self.l=gtk.Label(st)
        self.sw.add_with_viewport(self.l)
        self.win.show_all()
    def quit(self,w,a=None):
        w.destroy()


class circToolbar(myToolbar):
    def __init__(self,plotfn=None,label=""):
        myToolbar.__init__(self,plotfn=plotfn,label=label)
        #self.hbox.remove(self.reprbutton)
        #del(self.reprbutton)
        #self.gobutton=gtk.ToggleButton("Go")
        #self.gobutton.connect("clicked",self.gocirc)
        #self.tooltips.set_tip(self.gobutton,"Retrieve from the circular buffer")
        #self.freqspin=gtk.SpinButton()
        #self.freqspin.set_range(0,1000000)
        #self.freqspin.set_value(100)
        #self.tooltips.set_tip(self.freqspin,"Frequency with which buffer entries are retrieved")
        #self.freqspin.connect("value-changed",self.gocirc)
        #self.hbox.pack_start(self.gobutton)
        #self.hbox.pack_start(self.freqspin)
        #self.hbox.reorder_child(self.gobutton,0)
        #self.hbox.reorder_child(self.freqspin,1)
        self.frameWidget=gtk.Label()
        self.hbox2.pack_start(self.frameWidget)
        self.initialised=0

    def initialise(self,execute,toggle):
        """execute is a method called to execute the command"""
        self.initialised=1
        self.execute=execute
        self.toggle=toggle

        
    def gocirc(self,w,a=None):
        if self.initialised:
            if self.gobutton.get_active():
                freq=int(self.freqspin.get_text())
                #self.execute("c.circBufDict['%s'].freq[0]=%d"%(self.label,freq),tag=self.label)
                self.execute("c.subscribe(sock,'%s',%d)"%(self.label,freq),tag=self.label)

            else:
                #self.execute("c.circBufDict['%s'].freq[0]=0"%(self.label),tag=self.label)
                self.execute("c.subscribe(sock,'%s',0)"%(self.label),tag=self.label)

    def stop(self,w=None,a=None):
        #print "STOP"
        #self.gobutton.set_active(0)
        #self.toggle.set_active(0)
        pass

    def __del__(self):
        """Turn off the buffer..."""
        print "Destroying circ %s"%self.label
        self.stop()
        
class circTxtToolbar(myToolbar):
    def __init__(self,plotfn=None,label=""):
        myToolbar.__init__(self,plotfn=plotfn,label=label)
        self.hbox.remove(self.reprbutton)
        self.hbox.remove(self.savebutton)
        self.hbox.remove(self.loadbutton)
        self.hbox.remove(self.autobutton)
        self.hbox.remove(self.scaleMaxEntry)
        self.hbox.remove(self.scaleMinEntry)
        self.hbox.remove(self.logxbutton)
        del(self.reprbutton)
        del(self.savebutton)
        del(self.loadbutton)
        del(self.autobutton)
        del(self.scaleMinEntry)
        del(self.scaleMaxEntry)
        del(self.logxbutton)
        #self.gobutton=gtk.ToggleButton("Go")
        #self.gobutton.connect("clicked",self.gocirc)
        #self.tooltips.set_tip(self.gobutton,"Retrieve from the circular buffer")
        #self.freqspin=gtk.SpinButton()
        #self.freqspin.set_range(0,1000000)
        #self.freqspin.set_value(100)
        #self.tooltips.set_tip(self.freqspin,"Frequency with which buffer entries are retrieved")
        #self.freqspin.connect("value-changed",self.gocirc)
        #self.hbox.pack_start(self.gobutton)
        #self.hbox.pack_start(self.freqspin)
        #self.hbox.reorder_child(self.gobutton,0)
        #self.hbox.reorder_child(self.freqspin,1)
        self.frameWidget=gtk.Label()
        self.hbox.pack_start(self.frameWidget)
        #self.hbox2.remove(self.dataMangleEntry)
        self.hbox2.remove(self.scrollMangle)
        self.toolbar.remove(self.hbox2)
        del(self.dataMangleEntry)
        del(self.scrollMangle)
        del(self.hbox2)
        self.initialised=0

    def initialise(self,execute,toggle):
        """execute is a method called to execute the command"""
        self.initialised=1
        self.execute=execute
        self.toggle=toggle

        
    def gocirc(self,w,a=None):
        if self.initialised:
            if self.gobutton.get_active():
                freq=int(self.freqspin.get_text())
                #self.execute("c.circBufDict['%s'].freq[0]=%d"%(self.label,freq),tag=self.label)
                self.execute("c.subscribe(sock,'%s',%d)"%(self.label,freq),tag=self.label)

            else:
                #self.execute("c.circBufDict['%s'].freq[0]=0"%(self.label),tag=self.label)
                self.execute("c.subscribe(sock,'%s',0)"%(self.label),tag=self.label)

    def stop(self,w=None,a=None):
        #print "STOP"
        #self.gobutton.set_active(0)
        #self.toggle.set_active(0)
        pass
    
    def __del__(self):
        """Turn off the buffer..."""
        print "Destroying circ %s"%self.label
        self.stop()
        
class plot:
    """Note, currently, this cant be used interactively - because Gtk has to be running...."""
    def __init__(self,window=None,startGtk=0,dims=None,label="darcplot [%s]"%os.environ.get("HOSTNAME","unknown host"),usrtoolbar=None,loadFunc=None,loadFuncArgs=(),subplot=(1,1,1),deactivatefn=None,quitGtk=0,scrollWin=0):
        """If specified, usrtoolbar should be a class constructor, for a class containing: initial args of plotfn, label, a toolbar object which is the widget to be added to the vbox, and a prepare method which returns freeze,logscale,data,scale and has args data,dim

        """
        self.dims=dims
        self.data=numpy.zeros((7,15),numpy.float32)
        self.data[1:6,1]=1
        self.data[2,4]=1
        self.data[3,3]=1
        self.data[4,2]=1
        self.data[1:6,5]=1
        self.data[1:4,7]=1
        self.data[1:4,9]=1
        self.data[1,8]=1
        self.data[3,8]=1
        self.data[1:4,11]=1
        self.data[1:4,13]=1
        self.data[3,12]=1
        self.plot1dAxis=None
        self.line1d=None
        self.image2d=None
        self.plottype=None
        self.scatcol='b'
        self.overlay=None
        self.userLoadFunc=loadFunc
        self.loadFuncArgs=loadFuncArgs
        self.deactivatefn=deactivatefn#this can be set by the caller, eg to turn off buttons...
        self.zoom=1
        self.zoomx=0.
        self.zoomy=0.
        
        if window==None:
            self.win = gtk.Window()
            self.win.connect("destroy-event", self.quit)
            self.win.connect("delete-event", self.quit)
            self.win.set_default_size(400,400)
            self.win.set_title(label)
            self.settitle=1
        else:
            self.settitle=0
            self.win=window
        self.label=label
        self.cmap=colour.gray
        self.interpolation="nearest"#see pylab documantation for others.


        self.vpane=gtk.VPaned()
        self.win.add(self.vpane)
        self.vbox = gtk.VBox()
        self.vboxPlot = gtk.VBox()
        #self.win.add(self.vbox)
        #self.vpane.set_position(100)
        self.vpane.connect("button_press_event",self.buttonPress)
        self.vbox.connect("button_press_event",self.buttonPress)
        self.vboxPlot.connect("button_press_event",self.buttonPress)



        #self.vbox = gtk.VBox()
        #self.win.add(self.vbox)
        #self.vbox.connect("button_press_event",self.buttonPress)
        self.txtPlot=gtk.Label("")
        self.txtPlotBox=gtk.EventBox()
        self.txtPlotBox.add(self.txtPlot)
        self.image=gtk.Image()
        self.imageEvent=gtk.EventBox()
        self.imageEvent.add(self.image)
        self.pixbuf=None
        self.pixbufImg=None
        self.lay=gtk.Layout()
        self.lay.put(self.imageEvent,0,0)
        self.txtPlotBox.connect("button_press_event",self.buttonPress)
        self.vboxPlot.pack_start(self.txtPlotBox)
        self.vboxPlot.pack_start(self.lay)#self.image)
        self.lay.connect("size-allocate",self.changeSize)
        self.imageEvent.connect("button_press_event",self.buttonPress)
        self.imageEvent.connect("scroll_event",self.zoomImage)
        self.fig=Figure(dpi=50)
        #self.fig=Figure(figsize=(4,4), dpi=50)
        self.ax=self.fig.add_subplot(*subplot)
        self.fig.subplots_adjust(right=0.99,left=0.08,bottom=0.05,top=0.99)
        #self.ax.imshow(self.data,interpolation=self.interpolation)
        #print type(fig),dir(ax),dir(fig)
        self.canvas = FigureCanvas(self.fig)  # a gtk.DrawingArea
        self.vboxPlot.pack_start(self.canvas)
        self.vpane.pack1(self.vboxPlot,resize=True)
        if scrollWin:
            self.scroll=gtk.ScrolledWindow()
            self.scroll.set_policy(gtk.POLICY_AUTOMATIC,gtk.POLICY_AUTOMATIC)
            self.scroll.add_with_viewport(self.vbox)
            self.vpane.pack2(self.scroll,resize=False)
        else:
            self.scroll=None
            self.vpane.pack2(self.vbox,resize=False)
        self.toolbar = NavigationToolbar(self.canvas, self.win)
        self.vbox.pack_start(self.toolbar, False, False)
        if usrtoolbar==None:
            self.mytoolbar=myToolbar(plotfn=self.plot,label=label,loadFunc=self.loadFunc,streamName=label)
        else:
            self.mytoolbar=usrtoolbar(plotfn=self.plot,label=label)
        #self.toolbar.save_figure=self.mytoolbar.mysave
        self.vbox.pack_start(self.mytoolbar.toolbar,True,True)
        self.win.show_all()
        self.txtPlot.hide()
        self.txtPlotBox.hide()
        self.image.hide()
        self.lay.hide()
        self.toolbar.hide()
        self.mytoolbar.toolbar.show()
        self.active=1#will be set to zero once quit or window closed.
        self.toolbarVisible=1
        self.startedGtk=0
        self.quitGtk=quitGtk
        self.update=0
        #self.plot()
        if startGtk==1 and gtk.main_level()==0:
            self.startedGtk=1
            thread.start_new_thread(gtk.main,())
            
    def quit(self,w=None,data=None):
        if self.deactivatefn!=None:
            d=self.deactivatefn
            self.deactivatefn=None
            d(self)
        self.active=0
        self.win.hide()
        self.win.destroy()
        if self.startedGtk:
            gtk.main_quit()
        if hasattr(self.mytoolbar,"stop"):
            self.mytoolbar.stop()
        #print "Quit...",self.quitGtk
        if self.quitGtk:
            #print "Plot Quitting"
            gtk.main_quit()

    def changeSize(self,w,rect):#called for image buffer - if quick display
        r=w.get_parent().get_allocation()
        w,h=r.width,r.height
        if self.pixbuf!=None and (self.pixbuf.get_width()!=w or self.pixbuf.get_height()!=h):
             self.pixbuf=self.pixbuf.scale_simple(w,h,gtk.gdk.INTERP_NEAREST)
            self.image.set_from_pixbuf(self.pixbuf)
            self.image.queue_draw()
            

    def newPalette(self,palette):
        if palette[-3:]==".gp":
            palette=palette[:-3]
        if palette in colour.datad.keys():
            self.cmap=getattr(colour,palette)
        else:
            print "Palette %s not regocnised"%str(palette)
            print colour.datad.keys()
        #self.plot()
    def newInterpolation(self,interp):
        if interp not in ["bicubic","bilinear","blackman100","blackman256","blackman64","nearest","sinc144","sinc64","spline16","spline36"]:
            print "Interpolation %s not recognised"%str(interp)
        else:
            self.interpolation=interp
        #self.plot()
        
    def buttonPress(self,w,e,data=None):
        """If the user right clicks, we show or hide the toolbar..."""
        rt=False
        if (type(e)==type(1) and e==3) or e.button==3:
            rt=True
            if self.toolbarVisible:
                self.toolbar.hide()
                self.mytoolbar.toolbar.hide()
                if self.scroll!=None:
                    self.scroll.hide()
                self.toolbarVisible=0
                self.vpane.set_position(-1)
            else:
                self.toolbar.show()
                self.mytoolbar.toolbar.show()
                if self.scroll!=None:
                    self.scroll.show()
                    size=self.win.get_size()[1]-140
                    if size<0:
                        size=1
                    self.vpane.set_position(size)

                self.toolbarVisible=1
        return rt
    def zoomImage(self,w,e,data=None):
        redist=0
        if e.direction==gtk.gdk.SCROLL_UP:
            r=w.get_parent().get_allocation()
            w,h=r.width,r.height
            #print "Zoom in",e.x,e.y,w,h,e.x/float(w),e.y/float(h)
            self.zoom*=2
            self.zoomx+=e.x/(w-1.)/(self.zoom/2.)-0.5/self.zoom
            self.zoomy+=e.y/(h-1.)/(self.zoom/2.)-0.5/self.zoom
            if self.zoomx<0:self.zoomx=0.
            if self.zoomy<0:self.zoomy=0.
            if self.zoomx>1-1./self.zoom:self.zoomx=1-1./self.zoom
            if self.zoomy>1-1./self.zoom:self.zoomy=1-1./self.zoom
            #print self.zoom,self.zoomx,self.zoomy
            redist=1
        elif e.direction==gtk.gdk.SCROLL_DOWN:
            r=w.get_parent().get_allocation()
            w,h=r.width,r.height
            #print "Zoom out",e.x,e.y,w,h
            self.zoom/=2
            if self.zoom<=1:
                self.zoom=1
                self.zoomx=0
                self.zoomy=0
            else:
                self.zoomx-=(1-e.x/(w-1.))/(2.*self.zoom)
                self.zoomy-=(1-e.y/(h-1.))/(2.*self.zoom)
                if self.zoomx<0:self.zoomx=0
                if self.zoomy<0:self.zoomy=0
                if self.zoomx>1-1./self.zoom:self.zoomx=1-1./self.zoom
                if self.zoomy>1-1./self.zoom:self.zoomy=1-1./self.zoom
            redist=1
        if redist:#redisplay...
            self.plot()
                
    def loadFunc(self,fname,reposition=1):
        if fname==None:
            print "Clearing plot"
            if self.userLoadFunc!=None:
                try:
                    self.userLoadFunc(None)
                except:
                    traceback.print_exc()
            self.mytoolbar.dataMangleEntry.get_buffer().set_text("")
            self.mytoolbar.mangleTxt=""
            self.mytoolbar.setUserButtons(())
            self.plottype=None
            self.mytoolbar.store={}
            self.overlay=None
            self.mytoolbar.stream={}#can be uused to store dtaa from all streams.
            self.mytoolbar.streamName=fname
            self.mytoolbar.streamTime={}#stores tuples of fno,ftime for each stream
            self.mytoolbar.streamTimeTxt=""#text info to show...
        elif fname[-5:]==".fits":
            data=FITS.Read(fname)[1]
            print "Loading shape %s, dtype %s"%(str(data.shape),str(data.dtype.char))
            self.plot(data)
            if self.userLoadFunc!=None:
                self.userLoadFunc(self.label,data,fname,*self.loadFuncArgs)
        elif fname[-4:]==".xml":
            #change the confuration.
            plotList=plotxml.parseXml(open(fname).read()).getPlots()
            if self.userLoadFunc!=None:#allow the function to select the one it wants, and do stuff... (subscribe etc).
                try:
                    theplot=self.userLoadFunc(plotList,*self.loadFuncArgs)
                except:
                    theplot=plotList[0]
                    traceback.print_exc()
            else:
                theplot=plotList[0]
            pos=theplot[0]
            size=theplot[1]
            show=theplot[2]
            mangle=theplot[3]
            sub=theplot[4]
            tbVal=theplot[5]
            self.mytoolbar.dataMangleEntry.get_buffer().set_text(mangle)
            self.mytoolbar.mangleTxt=mangle
            self.mytoolbar.setUserButtons(tbVal)
            self.plottype=None
            self.mytoolbar.store={}
            self.overlay=None
            self.mytoolbar.stream={}#can be uused to store dtaa from all streams.
            self.mytoolbar.streamName=fname
            self.mytoolbar.streamTime={}#stores tuples of fno,ftime for each stream
            self.mytoolbar.streamTimeTxt=""#text info to show...

            #if tbVal!=None:
            #    for i in range(len(tbVal)):
            #        self.mytoolbar.tbList[i].set_active(tbVal[i])
            if reposition:
                if size!=None:
                    self.win.set_default_size(size[0],size[1])
                    self.win.resize(size[0],size[1])
                if pos!=None:
                    self.win.move(pos[0],pos[1])

            

    def queuePlot(self,axis,overlay=None,arrows=None,clear=1):
        """puts a request to plot in the idle loop... (gives the rest of the
        gui a chance to update before plotting)
        """
        #print type(axis),self.data.shape
        #if type(axis)!=type(None):
        #    print axis.shape
        if self.update:
            ax=self.ax
            #t1=time.time()
            if hasattr(self.ax.xaxis,"callbacks"):
                try:
                    self.ax.xaxis.callbacks.callbacks=dict([(s,dict()) for s in self.ax.xaxis.callbacks.signals])#needed to fix a bug!
                    self.ax.yaxis.callbacks.callbacks=dict([(s,dict()) for s in self.ax.yaxis.callbacks.signals])#needed to fix a bug!
                except:
                    pass
            if clear:
                self.ax.cla()
            
            #self.ax.clear()
            #t2=time.time()
            #print "axclear time %g"%(t2-t1),self.ax,self.ax.plot,self.ax.xaxis.callbacks
            freeze,logscale,data,scale,overlay,title,streamTimeTxt,dims,arrows,colour,text,axis,self.plottype,fount,fast=self.mytoolbar.prepare(self.data,dim=self.dims,overlay=overlay,arrows=arrows,axis=axis,plottype=self.plottype)
            if colour!=None:
                self.newPalette(colour)
            if title!=None and self.settitle==1:
                self.win.set_title(title)
            if len(streamTimeTxt)>0:
                self.mytoolbar.frameWidget.set_text(streamTimeTxt)

            if type(data)!=numpy.ndarray:
                data=str(data)#force to a string.  we can then just print this.
            updateCanvas=0
            if type(data)==type(""):
                #self.ax.text(0,0,data)
                if freeze==0:
                    data=data.replace("\0","")
                    self.canvas.hide()
                    self.image.hide()
                    self.lay.hide()
                    if fount!=None:
                        self.txtPlot.modify_font(pango.FontDescription(fount))
                    self.txtPlot.set_text(data)
                    self.txtPlot.show()
                    self.txtPlotBox.show()
                    self.ax.annotate(data,xy=(10,10),xycoords="axes points")
            elif len(data.shape)==1 or dims==1:
                updateCanvas=1
                self.canvas.show()
                self.txtPlot.hide()
                self.txtPlotBox.hide()
                self.image.hide()
                self.lay.hide()
                #1D
                if len(data.shape)==1:
                    if freeze==0:
                        if type(axis)==type(None) or axis.shape[0]!=data.shape[0]:
                            if self.plot1dAxis==None or self.plot1dAxis.shape[0]<data.shape[0]:
                                self.plot1dAxis=numpy.arange(data.shape[0])+1
                            axis=self.plot1dAxis[:data.shape[0]]
                        if logscale:
                            try:
                                axis=numpy.log10(axis)
                            except:
                                print "Cannot take log"
                        #self.fig.axis([axis[0],axis[-1],scale[0],scale[1]])
                        #if self.line1d!=None and self.line1d.get_xdata().shape==axis.shape and max(self.line1d.get_ydata())==scale[1] and min(self.line1d.get_ydata())==scale[0]:
                        #    self.line1d.set_data(axis,data)
                        #else:
                        #    print "replot"
                        #    self.ax.cla()
                        if self.plottype=="scatter":
                            self.line1d=self.ax.scatter(axis,data,s=1,c=self.scatcol)
                        else:
                            self.line1d=self.ax.plot(axis,data)[0]
                else:#use first row of data for the x axis...
                    #axis=data[0]
                    #freeze,logscale,data,scale=self.mytoolbar.prepare(self.data,dim=1)
                    if freeze==0:
                        start=0
                        if type(axis)==type(None) or axis.shape[-1]!=data.shape[1] or (len(axis.shape)==2 and axis.shape[0]!=data.shape[0]):
                            #print "Using first row as axis"
                            #axis=numpy.arange(data.shape[0])+1
                            axis=data[0]#first rox of data is the axis.
                            start=1
                        if logscale:
                            try:
                                axis=numpy.log10(axis)
                            except:
                                print "Cannot take log"
                        #self.fig.axis([axis[0],axis[-1],scale[0],scale[1]])
                        try:
                            if len(axis.shape)==1:#single axis
                                if self.plottype=="scatter":
                                    for i in range(start,data.shape[0]):
                                        self.ax.scatter(axis,data[i],s=1,c=self.scatcol)
                                else:
                                    for i in range(start,data.shape[0]):
                                        self.ax.plot(axis,data[i])
                            else:#axis for each data
                                if self.plottype=="scatter":
                                    for i in range(start,data.shape[0]):
                                        self.ax.scatter(axis[i],data[i],s=1,c=self.scatcol)
                                else:
                                    for i in range(start,data.shape[0]):
                                        self.ax.plot(axis[i],data[i])
                                
                        except:
                            print "Error plotting data"
                            traceback.print_exc()
                if text!=None:
                    for t in text:
                        if len(t)>=5:
                            ax.text(t[1],t[2],t[0],color=t[3],size=t[4])
                        elif len(t)==4:
                            ax.text(t[1],t[2],t[0],color=t[3])
                        else:
                            ax.text(t[1],t[2],t[0])
            else:#2D
                if fast:
                    self.canvas.hide()
                    self.txtPlot.hide()
                    self.txtPlotBox.hide()
                    self.image.show()
                    self.lay.show()
                    if freeze==0:
                        if len(data.shape)==3 and data.shape[2]==3:#RGB format
                            pass
                        elif len(data.shape)!=2:#force to 2d
                            data=numpy.reshape(data,(reduce(lambda x,y:x*y,data.shape[:-1]),data.shape[-1]))
                        if self.zoom!=1:
                            zy=self.zoomy*data.shape[0]
                            zx=self.zoomx*data.shape[1]
                            data=data[data.shape[0]-(zy+data.shape[0]/self.zoom):data.shape[0]-zy,zx:zx+data.shape[1]/self.zoom]
                        mi=numpy.min(data)
                        data-=mi
                        ma=numpy.max(data)
                        if ma>0:
                            if data.dtype.char in ['f','d']:
                                data*=255./ma
                            else:
                                data=data*255./ma
                        if self.pixbufImg==None or self.pixbufImg.get_width()!=data.shape[1] or self.pixbufImg.get_height()!=data.shape[0]:
                            self.pixbufImg=gtk.gdk.Pixbuf(gtk.gdk.COLORSPACE_RGB,False,8,data.shape[1],data.shape[0])
                        d=self.pixbufImg.get_pixels_array()
                        if len(data.shape)==2:
                            for i in range(3):
                                d[:,:,i]=data[::-1]#invert the data
                        else:#3D
                            d[:]=data[::-1]#invert the data
                        r=self.lay.get_allocation()
                        w,h=r.width,r.height
                        if self.plottype=="bilinear":
                            interp=gtk.gdk.INTERP_BILINEAR
                        else:
                            interp=gtk.gdk.INTERP_NEAREST
                        if self.pixbuf!=None and self.pixbuf.get_width()==w and self.pixbuf.get_height()==h:#scale into existing pixbuf
                            self.pixbufImg.scale(self.pixbuf,0,0,w,h,0,0,w/float(data.shape[1]),h/float(data.shape[0]),interp)
                        else:#create new pixbuf
                            self.pixbuf=self.pixbufImg.scale_simple(w,h,interp)
                            self.image.set_from_pixbuf(self.pixbuf)
                        self.image.queue_draw()
                else:
                    updateCanvas=1
                    self.canvas.show()
                    self.txtPlot.hide()
                    self.txtPlotBox.hide()
                    self.image.hide()
                    self.lay.hide()
                    #freeze,logscale,data,scale=self.mytoolbar.prepare(self.data)
                    if freeze==0:
                        if self.plottype!=self.interpolation and self.plottype!=None and self.plottype!="scatter":
                            self.newInterpolation(self.plottype)
                        if len(data.shape)!=2:#force to 2d
                            data=numpy.reshape(data,(reduce(lambda x,y:x*y,data.shape[:-1]),data.shape[-1]))
                        self.image2d=ax.imshow(data,interpolation=self.interpolation,cmap=self.cmap,vmin=scale[0],vmax=scale[1],origin="lower",aspect="auto")
                        if overlay!=None:
                            ax.imshow(overlay,interpolation=self.interpolation,cmap=self.cmap,vmin=scale[0],vmax=scale[1],origin="lower",aspect="auto",extent=(-0.5,data.shape[1]-0.5,-0.5,data.shape[0]-0.5))
                        #arrows=[[10,10,500,50,{"head_width":10,"head_length":20,"head_width":10,"head_length":10,"length_includes_head":True,"fc":"red","ec":"green"}]]
                        if arrows!=None:
                            for a in arrows:
                                if len(a)>=4:
                                    if len(a)==5:
                                        args=a[4]
                                    else:
                                        args={}
                                    ax.arrow(a[0],a[1],a[2],a[3],**args)#head_width=10,head_length=10)
                        if text!=None:
                            for t in text:
                                if len(t)>=5:
                                    ax.text(t[1],t[2],t[0],color=t[3],size=t[4])
                                elif len(t)==4:
                                    ax.text(t[1],t[2],t[0],color=t[3])
                                else:
                                    ax.text(t[1],t[2],t[0])

            if freeze==0 and updateCanvas==1:
                try:
                    self.ax.draw()
                except:
                    pass
                #self.ax.update()
                self.canvas.draw()
                #self.canvas.queue_draw()
        self.update=0
        return False
    
    def plot(self,data=None,copy=0,axis=None,overlay=None,arrows=None,clear=1):
        """Plot new data... axis may be specified if 1d...
        overlay is an optional overlay...
        """
        if self.active==0:
            self.active=1
            self.win.show()
        self.overlay=overlay
        #if type(data)==numpy.ndarray:
        #    data=Numeric.array(data)
        if type(data)!=type(None):
            if copy:
                self.data=data.copy().astype("d")
            else:
                if not hasattr(data,"dtype"):#not an array or a numpy.float/int type.
                    self.data=numpy.array([data]).view("b")
                else:
                    self.data=data
                #elif data.dtype.char=="d":
                #    self.data=data
                #else:
                #    self.data=data.astype("d")
        else:#data==None?
            pass
            #if type(self.data)==numpy.ndarray:
            #    self.data=Numeric.array(self.data)
        self.update=1
        #print "plot"
        #print type(axis)
        #if type(axis)!=type(None):
        #    print axis.shape
        gobject.idle_add(self.queuePlot,axis,overlay,arrows,clear)
        return True

class plotTxt:
    def __init__(self,window=None,startGtk=0,dims=2,label="Window",usrtoolbar=None,loadFunc=None,loadFuncArgs=(),subplot=(1,1,1),deactivatefn=None):
        """
        This is a class for displaying text messages rather than images.
        If specified, usrtoolbar should be a class constructor, for a class containing: initial args of plotfn, label, a toolbar object which is the widget to be added to the vbox, and a prepare method which returns freeze,logscale,data,scale and has args data,dim

        """
        self.dims=dims
        self.data=numpy.zeros((7,15),numpy.float32)
        self.data[1:6,1]=1
        self.data[2,4]=1
        self.data[3,3]=1
        self.data[4,2]=1
        self.data[1:6,5]=1
        self.data[1:4,7]=1
        self.data[1:4,9]=1
        self.data[1,8]=1
        self.data[3,8]=1
        self.data[1:4,11]=1
        self.data[1:4,13]=1
        self.data[3,12]=1
        self.overlay=None
        self.userLoadFunc=loadFunc
        self.loadFuncArgs=loadFuncArgs
        self.deactivatefn=deactivatefn#this can be set by the caller, eg to turn off buttons...
        
        self.win = gtk.Window()
        self.win.connect("destroy", self.quit)
        self.win.set_default_size(400,100)
        self.win.set_title(label)
        self.label=label
        self.vpane=gtk.VPaned()
        self.win.add(self.vpane)
        self.vbox = gtk.VBox()
        #self.win.add(self.vbox)
        self.vpane.pack2(self.vbox,resize=True)
        self.vpane.connect("button_press_event",self.buttonPress)
        self.labelWidget=gtk.Label("")
        self.labelWidget.set_justify(gtk.JUSTIFY_CENTER)
        self.vpane.pack1(self.labelWidget,resize=True)
        #self.toolbar = NavigationToolbar(self.canvas, self.win)
        #self.vbox.pack_start(self.toolbar, False, False)
        if usrtoolbar==None:
            self.mytoolbar=myToolbar(plotfn=self.plot,label=label,loadFunc=self.loadFunc)
        else:
            self.mytoolbar=usrtoolbar(plotfn=self.plot,label=label)
        #self.toolbar.save_figure=self.mytoolbar.mysave
        self.vbox.pack_start(self.mytoolbar.toolbar,False,False)
        self.win.show_all()
        #self.toolbar.hide()
        self.mytoolbar.toolbar.show()
        self.active=1#will be set to zero once quit or window closed.
        self.toolbarVisible=1
        self.startedGtk=0
        self.update=0
        #self.plot()
        if startGtk==1 and gtk.main_level()==0:
            self.startedGtk=1
            thread.start_new_thread(gtk.main,())

    def quit(self,w=None,data=None):
        if self.deactivatefn!=None:
            d=self.deactivatefn
            self.deactivatefn=None
            d(self)
        self.active=0
        self.win.hide()
        self.win.destroy()
        if self.startedGtk:
            gtk.main_quit()
        if hasattr(self.mytoolbar,"stop"):
            self.mytoolbar.stop()
    def buttonPress(self,w,e,data=None):
        """If the user right clicks, we show or hide the toolbar..."""
        if e.button==3:
            if self.toolbarVisible:
                #self.toolbar.hide()
                self.mytoolbar.toolbar.hide()
                self.toolbarVisible=0
            else:
                #self.toolbar.show()
                self.mytoolbar.toolbar.show()
                self.toolbarVisible=1

        return True

    def plot(self,data=None,copy=0,axis=None,overlay=None):
        """Plot new data... axis may be specified if 1d...
        """
        if self.active==0:
            self.active=1
            self.win.show()
        #if type(data)==numpy.ndarray:
        #    data=Numeric.array(data)
        if type(data)==type(""):
            self.data=data
        elif type(data)==numpy.ndarray:
            self.data=data.view("b").tostring().strip().replace("\0"," ")
                #elif data.dtype.char=="d":
                #    self.data=data
                #else:
                #    self.data=data.astype("d")
        else:#data==None?
            pass
            #if type(self.data)==numpy.ndarray:
            #    self.data=Numeric.array(self.data)
        self.update=1
        #print "plot"
        #print type(axis)
        #if type(axis)!=type(None):
        #    print axis.shape
        self.labelWidget.set_text(self.data)


def randomise(w,data=None):
    print "Randomising"
    X[:]=numpy.random.random((20,20)).astype("f")
    #ax = fig.add_subplot(111)
    #print dir(ax),ax.figBottom
    #dr=dir(ax)
    #for d in dr:
    #    print d,"       ",getattr(ax,d)
    ax.clear()
    ax.imshow(X,interpolation="nearest",cmap=colour.gray)
    
    #ax.redraw_in_frame()
    ax.draw()
    #event = gtk.gdk.Event(gtk.gdk.EXPOSE)
    #print dir(event),dir(event.area),type(event.area),type(event.area.x),event.area.x,event.area.height
    #event.time = -1 # assign current time
    #canvas.emit("expose_event",event)
    #self.window.draw_drawable (self.style.fg_gc[self.state],self._pixmap, 0, 0, 0, 0, w, h)
    canvas.queue_draw()
    return True
def buttonPress(w,e,data=None):
    if e.button==3:
        print "Right clicked..."
        print dir(toolbar)
        print dir(toolbar.get_parent())
        print dir(toolbar.window)
        if gtk.Object.flags(toolbar)|gtk.VISIBLE:
            print "visible"
        print toolbar.get_child_visible()
        toolbar.hide()
    if e.button==2:
        print "middle clicked"
        ax.clear()
        ax.plot(numpy.arange(X.shape[1]),X[0])
        ax.draw()
        canvas.queue_draw()
    return True


class plotToolbar(myToolbar):
    def __init__(self,plotfn=None,label=""):
        myToolbar.__init__(self,plotfn=plotfn,label=label)
        vbox=gtk.VBox()
        hbox=gtk.HBox()
        self.confighbox=hbox
        self.tbHbox=gtk.HBox()
        self.tbList=[]
        self.tbVal=[]
        self.tbNames=[]
        b=gtk.Button("Spawn")
        b.set_tooltip_text("Start a new plot")
        hbox.pack_start(b)
        b.connect("clicked",self.spawn)
        b=gtk.Button("Activate")
        b.set_tooltip_text("Click to use the mangle text (actually, it will be used anyway, but this just gives people some reassurance)")
        hbox.pack_start(b)
        self.configdir=None
        self.comboList=gtk.ListStore(gobject.TYPE_STRING)
        self.combobox=gtk.ComboBox(self.comboList)
        cell = gtk.CellRendererText()
        self.combobox.pack_start(cell, True)
        self.combobox.add_attribute(cell, 'text', 0)
        self.combobox.connect("changed",self.comboChanged)
        self.filelist=[]
        #for i in range(3):#set up 3 user toggle buttons.
        #    self.tbList.append(gtk.ToggleButton("%d"%i))
        #    self.tbList[-1].connect("toggled",self.userButtonToggled,i)
        #    self.tbVal.append(0)
        #    self.tbHbox.pack_start(self.tbList[-1])
        self.frameWidget=gtk.Label()
        vbox.pack_start(hbox)
        vbox.pack_start(self.tbHbox)
        vbox.pack_start(self.frameWidget)
        self.hbox2.pack_start(vbox)
        self.hbox2.reorder_child(vbox,0)
        #self.hbox2.pack_start(self.frameWidget)
        self.initialised=0

        

    def initialise(self,subscribeAction,configdir=None):
        """execute is a method called to execute the command"""
        self.initialised=1
        self.showStreams=gtk.Button("Subscribe")
        self.hbox.pack_start(self.showStreams)
        self.showStreams.show()
        self.showStreams.connect("clicked",subscribeAction)
        self.configdir=configdir
        if self.configdir!=None:
            if len(self.configdir)==0:
                self.configdir="./"
            if self.configdir[-1]!='/':
                self.configdir+='/'
            sys.path.append(self.configdir)
            self.confighbox.pack_start(self.combobox)
            self.combobox.show()
            self.wm=WatchDir(self.configdir,"plot",".xml",self.comboUpdate,self.comboRemove)
            gobject.io_add_watch(self.wm.myfd(),gobject.IO_IN,self.wm.handle)
            self.comboPopped()
    def spawn(self,w=None,a=None):
        subprocess.Popen(sys.argv)

    def userButtonToggled(self,w,a=None):
        self.tbVal[a]=int(w.get_active())
        print self.tbVal
    def displayFileList(self,parentWin=None):
        w=gtk.Window()
        if parentWin!=None:
            w.set_transient_for(parentWin)
            w.set_position(gtk.WIN_POS_CENTER_ON_PARENT)
        w.connect("delete-event",w.destroy)
        w.set_title("Chose plot configuration")
        v=gtk.VBox()
        w.add(v)
        for f in self.filelist:
            if f!=None:
                b=gtk.Button(f)
                b.connect("clicked",self.comboChosen)
                v.add(b)
        w.show_all()
    def comboChosen(self,w):
        fname=self.configdir+w.get_child().get_text()
        print "loading",fname
        while len(self.tbVal)>0:
            self.removeUserButton()
        self.loadFunc(fname,reposition=0)
        win=w.get_parent().get_parent()
        win.destroy()
        
    def comboChanged(self,w,a=None):
        indx=w.get_active()
        if indx>=0:
            if self.filelist[indx]==None:
                while len(self.tbVal)>0:
                    self.removeUserButton()
                self.loadFunc(None)
            else:
                fname=self.configdir+self.filelist[indx]
                print "loading",fname
                while len(self.tbVal)>0:
                    self.removeUserButton()
                self.loadFunc(fname,reposition=0)
    def comboUpdate(self,fname):
        if fname not in self.filelist:
            self.filelist.append(fname)
            self.combobox.append_text(fname)


    def comboRemove(self,fname):
        if fname in self.filelist:
            indx=self.filelist.index(fname)
            self.combobox.remove_text(indx)
            self.filelist.pop(indx)

    def comboPopped(self,w=None):
        files=os.listdir(self.configdir)
        files.sort()
        self.filelist=[None]
        self.combobox.append_text("None")
        for fname in files:
            if fname[:4]=="plot" and fname[-4:]==".xml":
                self.filelist.append(fname)
                self.combobox.append_text(fname)
    def addUserButton(self,name=None,active=0):
        pos=len(self.tbList)
        if name==None:
            name="%d"%pos
        but=gtk.ToggleButton(name)
        but.set_active(active)
        but.connect("toggled",self.userButtonToggled,pos)
        self.tbList.append(but)
        self.tbVal.append(active)
        self.tbNames.append(name)
        self.tbHbox.pack_start(but)
        but.show()
    def removeUserButton(self):
        self.tbVal.pop()
        self.tbNames.pop()
        but=self.tbList.pop()
        self.tbHbox.remove(but)
    def setUserButtons(self,tbVal,names=None):
        if tbVal==self.tbVal and names==self.tbNames:
            return
        if tbVal==None:
            tbVal=[]
        if names!=None:
            l=len(names)
            ll=len(tbVal)
            if ll<l:
                tbVal=tbVal+[0]*(l-ll)
            else:
                tbVal=tbVal[:l]
        else:
            names=map(str,range(len(tbVal)))
            names[:len(self.tbNames)]=self.tbNames[:len(tbVal)]
        while len(self.tbVal)>len(tbVal):
            self.removeUserButton()

        l=len(self.tbList)
        for i in range(l):#for existing buttons
            self.tbList[i].set_active(tbVal[i])
            if names[i]!=self.tbNames[i]:
                self.tbList[i].set_label(names[i])
                self.tbNames[i]=names[i]
        for i in range(l,len(tbVal)):#and add new if required.
            self.addUserButton(names[i],tbVal[i])


class SubWid:
    """Class which shows a list of streams and allows the user to choose which ones should be subscribed too.
    """
    def __init__(self,win=None,parentSubscribe=None,parentWin=None,parentGrab=None,parentDec=None):
        """parentSubscribe is a method with args (stream, active flag, decimate,change other decimates flag) which is called when a stream subscription is changed
        """
        if win==None:
            win=gtk.Window()
        self.parentSubscribe=parentSubscribe
        self.parentGrab=parentGrab
        self.parentDec=parentDec
        self.win=win#gtk.Window()
        self.win.set_transient_for(parentWin)
        self.win.set_position(gtk.WIN_POS_CENTER_ON_PARENT)
        #self.win.connect("destroy", self.hide)
        self.win.connect("delete-event", self.hide)
        #self.win.set_default_size(400,100)
        self.win.set_title("Subscribe to...")
        self.table=gtk.Table(1,3)
        #self.hbox=gtk.HBox()
        #self.win.add(self.hbox)
        self.win.add(self.table)
        #self.win.set_modal(1)
        #self.vboxSub=gtk.VBox()#will hold the subscribe toggle buttons
        #self.hbox.pack_start(self.vboxSub)
        #self.vboxDec=gtk.VBox()#will hold the decimate rates
        #self.hbox.pack_start(self.vboxDec)
        #self.vboxChange=gtk.VBox()#will hold the change decimate rates flag
        #self.hbox.pack_start(self.vboxChange)
        #self.tooltips=gtk.Tooltips()



    def hide(self,w=None,data=None):
        self.win.hide()
        return True#False

    def show(self,streamDict,subscribeDict):
        """put stream list in and show...
        streamDict has entries (short description, long description)
        subscribeDict has an entry for each stream subscribed too, which is
        (sub,decimation) where sub is a flag whether subscribed to or not and 
        decimation is the decimation rate.
        decDict is dictionary of decimations as returned by controlClient().GetDecimation()
        """
        self.table.resize(len(streamDict),3)
        #for c in self.vboxSub.get_children():
        #    self.vboxSub.remove(c)
        #for c in self.vboxDec.get_children():
        #    self.vboxDec.remove(c)
        #for c in self.vboxChange.get_children():
        #    self.vboxChange.remove(c)
        #self.vboxSub.pack_start(gtk.Label("Stream "))
        #self.vboxDec.pack_start(gtk.Label("decimate"))
        #self.vboxChange.pack_start(gtk.Label(" "))
        self.table.foreach(self.table.remove)
        self.table.attach(gtk.Label("Stream "),0,1,0,1)
        self.table.attach(gtk.Label("decimate"),1,2,0,1)
        #self.table.attach(gtk.Label("d(local)"),3,4,0,1)
        #self.table.attach(gtk.Label("d(rtc)"),4,5,0,1)
        pos=1
        for s in streamDict.keys():
            short,lng=streamDict[s]
            t=gtk.ToggleButton(short)
            c=gtk.Button()
            #c.set_active(1)
            e=gtk.Entry()
            #e2=gtk.Entry()
            #e3=gtk.Entry()

            if len(short)!=3 and ("rtc" not in short or "Buf" not in short):
                t.set_sensitive(0)
                c.set_sensitive(0)
                e.set_sensitive(0)
                #e2.set_sensitive(0)
                #e3.set_sensitive(0)
            e.set_width_chars(4)
            #e2.set_width_chars(4)
            #e3.set_width_chars(4)
            t.set_tooltip_text(lng)
            e.set_tooltip_text("plot decimation factor")
            #e2.set_tooltip_text("local decimation factor")
            #e3.set_tooltip_text("RTC decimation factor")
            c.set_tooltip_text("Force local decimation rate to plot rate?")
            e.set_text("100")
            #e2.set_text("100")
            #e3.set_text("100")
            if subscribeDict.has_key(s):
                sub,dec=subscribeDict[s][:2]
                e.set_text("%d"%dec)#set decimation
                if sub:
                    t.set_active(1)#subscribe to it
                #if ch:
                #    c.set_active(1)
            #if decDict.has_key(s):
            #    e3.set_text("%d"%decDict[s])
            #if decDict.has_key("local") and decDict["local"].has_key(s):
            #    e2.set_text("%d"%decDict["local"][s])
            t.id=s
            args=(s,t,e,c)#,c,e2,e3)
            t.connect("toggled",self.substream,args)
            t.connect("button-press-event",self.substream,args)
            e.connect("activate",self.substream,args)
            e.connect("focus_out_event",self.substream,args)
            c.connect("clicked",self.substream,args)
            #e2.connect("activate",self.substream,args)
            #e2.connect("focus_out_event",self.substream,args)
            #e3.connect("activate",self.substream,args)
            #e3.connect("focus_out_event",self.substream,args)
            self.table.attach(t,0,1,pos,pos+1)
            self.table.attach(e,1,2,pos,pos+1)
            self.table.attach(c,2,3,pos,pos+1)
            #self.table.attach(e2,3,4,pos,pos+1)
            #self.table.attach(e3,4,5,pos,pos+1)
            #self.vboxSub.pack_start(t)
            #self.vboxDec.pack_start(e)
            #self.vboxChange.pack_start(c)
            pos+=1
        self.win.show_all()


    def substream(self,w,ev=None,a=None):
        """User has toggled the subscribe button or changed decimate rate"""
        grab=0
        if type(ev)==type(()):#e is the data
            s,t,e,c=ev#,c,e2,e3=ev
        else:#e is an event (from focus_out_event or pressed event)
            s,t,e,c=a#,c,e2,e3=a
            if type(w)==gtk.ToggleButton:
                grab=ev.button
        #print "substream",t,w
        #if type(w)==gtk.ToggleButton:
        #    t=w
        s=t.id
        active=int(t.get_active())
        try:
            dec=int(eval(e.get_text()))
        except:
            dec=100
            e.set_text("%d"%dec)
        #try:
        #    dec2=int(eval(e2.get_text()))
        #except:
        #    dec2=100
        #    e2.set_text("%d"%dec2)
        #try:
        #    dec3=int(eval(e3.get_text()))
        #except:
        #    dec3=100
        #    e3.set_text("%d"%dec3)

        #change=int(c.get_active())
        if type(w)==gtk.Button:
            #clicked the button to force local decimation rate
            if self.parentDec!=None:
                self.parentDec(s,dec)
        else:
            if grab==2 or grab==3:
                if self.parentGrab!=None:
                    self.parentGrab(s,latest=(grab==2))
            else:
                if self.parentSubscribe!=None:
                    self.parentSubscribe((s,active,dec))#,change,dec2,dec3))
        #self.show({3:("s3","l3"),4:("s4","l4")},{2:(1,20),3:(1,30)})

class PlotServer:
    def __init__(self,port,shmtag):
        self.port=port
        self.shmtag=shmtag
        #First, before doing anything else, connect to port.
        if self.makeConnection("localhost",port)==None:
            #connection failed, so exit...
            raise Exception("Couldn't connect to data server")
        #Now we're connected, wait for info to arrive...
        #Then once we have the stream list (which will come last), 
        #we can create the plot object and show it...
        subList=[]
        self.subapLocation=None#this is needed for plotting centroid overlays.  The centroid measurements are relative to this, rather than relative to the rtcSubLocBuf, which can change if in adaptive window mode.  So, the plotter needs a copy of the static subapLocation.
        self.subWid=None
        self.shmDict={}
        self.streamDict={}
        self.subscribeDict={}
        self.subscribeList=[]
        self.conndata=None
        self.mangle=""
        self.visible=1
        tbVal=None
        initDone=0
        size=pos=None
        while initDone==0:
            print "Plot reading message"
            msg=serialise.ReadMessage(self.conn)
            if msg==None:
                print "Plot read message failed - exiting"
                sys.exit(0)
            print "Plot got msg",msg
            if msg[0]=="state":
                pos=msg[1]
                size=msg[2]
                self.visible=msg[3]
                self.mangle=msg[4]
                subList=msg[5]#list of things subscribed to... list of tuples of (stream, subscribe flag, decimate,change other decimates flag)
                tbVal=msg[6]#buttons...
                
            elif msg[0]=="streams":
                self.addStream(msg[1])
                initDone=1
            elif msg[0]=="subapLocation":
                self.subapLocation=msg[1]
                self.npxlx=msg[2]
                self.npxly=msg[3]
                self.nsub=msg[4]
                #self.nsuby=msg[5]
                self.subapFlag=msg[5]
        self.subscribe(subList)
        self.plot=plot(quitGtk=1,usrtoolbar=plotToolbar)
        if self.visible==0:
            class dummy:
                button=3
            self.plot.buttonPress(None,dummy())
        self.plot.mytoolbar.dataMangleEntry.get_buffer().set_text(self.mangle)
        self.plot.mytoolbar.mangleTxt=self.mangle
            
        if size!=None:
            self.plot.win.set_default_size(size[0],size[1])
            self.plot.win.resize(size[0],size[1])
        if pos!=None:
            self.plot.win.move(pos[0],pos[1])
        self.plot.mytoolbar.setUserButtons(tbVal)
            #for i in range(min(len(self.plot.mytoolbar.tbList),len(tbVal))):
            #    v=tbVal[i]
            #    self.plot.mytoolbar.tbList[i].set_active(v)
        self.subWid=SubWid(gtk.Window(),self.subscribe,self.plot.win)
        if len(self.subscribeList)==0:    
            #need to pop up the subscribbe widget...
            #the user can then decide what to sub to.
            self.subWid.show(self.streamDict,self.subscribeDict)
        self.plot.mytoolbar.initialise(self.showStreams)

    def showStreams(self,w=None,a=None):
        if self.subWid!=None:
            self.subWid.show(self.streamDict,self.subscribeDict)

    def makeConnection(self,host,port):
        self.conn=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        try:
            self.conn.connect((host,port))
            self.sockID=gobject.io_add_watch(self.conn,gtk.gdk.INPUT_READ,self.handleData)
        except:
            print "Plotter error - couldn't connect to %s %d"%(host,port)
            self.conn=None
            raise
        return self.conn
    def subscribe(self,slist):
        """slist is a list of tuples of (stream,subscribe flag,decimate,change other decimates flag).
        """
        if type(slist)!=type([]):
            slist=[slist]
        for s in slist:
            serialise.Send(["sub",s],self.conn)
            if s[1]:#subscribe to it
                self.subscribeDict[s[0]]=s[1:]
                if s not in self.subscribeList:
                    self.subscribeList.append(s)
            else:
                if self.subscribeDict.has_key(s[0]):
                    del(self.subscribeDict[s[0]])
                if s in self.subscribeList:
                    self.subscribeList.remove(s)

    def addStream(self,streamList):
        """Add streams.
        streamList is a list of tuples containing:
        (stream name, short description, long description)"""
        if type(streamList)!=type([]):
            streamList=[streamList]
        for stream in streamList:
            self.streamDict[stream[0]]=stream[1:]
        if self.subWid!=None and self.subWid.win.get_property("visible"):
            self.showStreams()

    def delStream(self,stream):
        if self.streamDict.has_key(stream):
            del(self.streamDict[stream])
        if self.shmDict.has_key(stream):
            del(self.shmDict[stream])
        if self.subWid!=None and self.subWid.win.get_property("visible"):
            self.showStreams()
    def updateStream(self,stream,data=None):
        """new data has arrived for this stream... so plot it."""
        if stream in map(lambda x:x[0],self.subscribeList):#we're subscribed to it...
            if data==None:
                data,fno,ftime=self.readSHM(stream)
            else:
                data,ftime,fno=data
            if data!=None:
                if not self.plot.mytoolbar.streamTime.has_key(stream) or self.plot.mytoolbar.streamTime[stream]!=(fno,ftime):
                    #print "plotting...",fno,ftime
                    self.plot.mytoolbar.subapLocation=self.subapLocation
                    self.plot.mytoolbar.npxlx=self.npxlx
                    self.plot.mytoolbar.npxly=self.npxly
                    self.plot.mytoolbar.nsub=self.nsub
                    #self.plot.mytoolbar.nsuby=self.nsuby
                    self.plot.mytoolbar.subapFlag=self.subapFlag
                    
                    self.plot.mytoolbar.stream[stream]=data
                    self.plot.mytoolbar.streamName=stream
                    self.plot.mytoolbar.streamTime[stream]=fno,ftime
                    self.plot.mytoolbar.streamTimeTxt="%10d %9s%03d"%(fno,time.strftime("%H:%M:%S.",time.localtime(ftime)),(ftime%1)*1000)
                    if "rtcStatusBuf" in stream or stream=="Sta":
                        self.plot.mytoolbar.mangleTxtDefault="data=data.tostring()"
                    else:
                        self.plot.mytoolbar.mangleTxtDefault=""
                    self.plot.plot(data)
                else:
                    #print "not replotting"
                    pass
                # I dont think we need to do owt bout overlays - are created by mangle each time, and stored in store if necessary.
            
    def readSHM(self,stream):
        """The shared memory has a 64 byte header, and then the data.
        The header contains: dtype(1),nd(1),deadflag(1),writingflag(1),6xdimensions(24), frame number (4), frame time (8), 24 spare.
        If writing flag is set, don't bother doing anything - this means the next bit data is being written, and we'll be informed in due course.
        If deadflag is set, means the shm is no longer in use, and should be closed, and possibly repoened.
        IF nd or shape change, then deadflag will be set.  IE nd and shape are constant for the lifetime of the shm.
        """
        data=None
        ftime=0.
        fno=0
        shm=self.shmDict.get(stream)
        if shm==None:
            #try opening it...
            shm=self.openSHM(stream)
            if shm!=None:
                self.shmDict[stream]=shm
        if shm!=None:
            hdr,data=shm
            if hdr[2]==1:#dead flag - data no longer valid
                data=None
                del(self.shmDict[stream])
                shm=self.openSHM(stream)
                if shm!=None:#have reopened successfully.
                    hdr,data=shm
                    self.shmDict[stream]=hdr,data
            if hdr[3]==1:#writing flag - data invalid this time.
                data=None
        if data!=None:
            fno=hdr[:64].view(numpy.int32)[7]
            ftime=hdr[:64].view(numpy.float64)[4]
            data=numpy.array(data)#copy from shm to array...
        return data,fno,ftime

    def openSHM(self,stream):
        """Attempt to open the SHM array
        Return None, or (shm,reformatted data)
        """
        try:
            hdr=numpy.memmap(devshm+self.shmtag+stream,dtype=numpy.uint8,mode="r",shape=(64,))
            if hdr[2]==1:#dead flag
                hdr=None
        except:
            hdr=None
        data=None
        if hdr!=None and hdr[1]>0:
            size=reduce(lambda x,y:x*y,hdr.view(numpy.int32)[1:1+hdr[1]])
            if size>0:
                elsize={'f':4,'d':8,'i':4,'h':2,'b':1,'H':2}[hdr[0].view('c')]
                try:
                    data=numpy.memmap(devshm+self.shmtag+stream,dtype=numpy.uint8,mode="r",shape=(64+size*elsize,))
                    if data[2]==1:#dead flag
                        data=None
                except:
                    data=None
                if not numpy.alltrue(hdr[:3]==data[:3]) or not numpy.alltrue(hdr[4:28]==data[4:28]):
                    data=None
        del(hdr)
        if data!=None:
            d=data[64:].view(data[0].view('c'))
            d.shape=data[:64].view(numpy.int32)[1:1+data[1]]
            data=(data,d)
        return data

            
            
    def process(self):#,readSock=1):
        try:
            self.conndata,valid=serialise.Recv(self.conn,self.conndata)
        except:#disconnected
            valid=0
            self.conn=None
            self.conndata=None
        if valid:
            data=self.conndata
            self.conndata=None
            data=serialise.Deserialise(data)[0]
            if type(data)==type(None):
                self.conn=None#disconnected
            else:
                if data[0]=="new":#new stream
                    self.addStream(data[1])
                elif data[0]=="upd":#new data from a stream
                    self.updateStream(data[1])
                elif data[0]=="del":#remove stream
                    self.delStream(data[1])
                elif data[0]=="sav":#save state of plot... (send to GUI)
                    pos=self.plot.win.get_position()
                    size=self.plot.win.get_size()
                    serialise.Send(["sav",pos,size,self.plot.toolbarVisible,self.plot.mytoolbar.tbVal,self.plot.mytoolbar.mangleTxt,self.subscribeList],self.conn)
                elif data[0]=="end":
                    gtk.main_quit()
                elif data[0]=="sub":#a new subapLocation...
                    print "plot updating subaplocation"
                    self.subapLocation=data[1]
                    self.npxlx=data[2]
                    self.npxly=data[3]
                    self.nsub=data[4]
                    #self.nsuby=data[5]
                    self.subapFlag=data[5]
                elif data[0]=="dat":#this is only used on windoze where the shared memory stuff doesn't work - a quick fix - send the stream to each plot - even if they're not subscribed to it...
                    self.updateStream(data[1],data[2:5])
                    
    def handleData(self,source,condition):
        self.process()
        if self.conn==None:#connection has been closed
            print "Plotter connection closed remotely"
            if self.sockID!=None:
                gobject.source_remove(self.sockID)#gtk.input_remove(self.sockID)
            self.sockID=None
            gtk.main_quit()
        return self.conn!=None
class StdinServer:
    def __init__(self):
        self.go=1
        self.lastwin=0
        self.plotdict={}
        gobject.io_add_watch(sys.stdin,gobject.IO_IN,self.handleInput)
        gobject.io_add_watch(sys.stdin,gobject.IO_HUP,self.quit)
        self.data=None

    def quit(self,source,cbcondition,a=None):
        gtk.main_quit()
        return True
    def newplot(self):
        p=plot(usrtoolbar=plotToolbar)
        p.buttonPress(None,3)
        p.txtPlot.hide()
        p.txtPlotBox.hide()
        p.image.hide()
        return p
    
    def handleInput(self,source,cbcondition,a=None):
        data=serialise.ReadMessage(sys.stdin)
        if type(data)==type([]):
            if len(data)>1:
                win=data[0]
                data=data[1]
            else:
                win=self.lastwin
                data=data[0]
            if win==None:
                win=self.lastwin
            self.lastwin=win
            if not self.plotdict.has_key(win) or self.plotdict[win].active==0:
                self.plotdict[win]=self.newplot()
            p=self.plotdict[win]

            p.plot(data)
        return True
    def handleInputAsync(self,source,cbcondition,a=None):
        print "Handle input"
        #data=serialise.ReadMessage(sys.stdin)
        self.data,valid=serialise.Recv(source,self.data)
        if valid:
            data=serialise.Deserialise(self.data)[0]
            self.data=None
            if type(data)==type([]) and len(data)>0:
                data=data[0]
            self.p.plot(data)
        return True




class DarcReader:
    def __init__(self,streams,myhostname=None,prefix="",dec=25,configdir=None,withScroll=0):
        import controlCorba
        self.paramTag=0
        self.streams=[]
        self.prefix=prefix
        self.plotWaitingDict={}
        l=len(prefix)
        for s in streams:
            if l>0:
                if s[:l]==prefix:
                    self.streams.append(s)
                else:
                    self.streams.append(prefix+s)
            else:
                self.streams.append(s)
        streams=self.streams
        self.c=controlCorba.controlClient(controlName=prefix,debug=0)
        cnt=1
        while self.c.obj==None and cnt>0:
            cnt-=1
            time.sleep(1)
            self.c=controlCorba.controlClient(controlName=prefix,debug=0)
        self.p=plot(usrtoolbar=plotToolbar,quitGtk=1,loadFunc=self.loadFunc,scrollWin=withScroll,label=prefix)
        self.p.buttonPress(None,3)
        self.p.mytoolbar.loadFunc=self.p.loadFunc
        self.p.txtPlot.hide()
        self.p.txtPlotBox.hide()
        self.p.image.hide()
        try:
            self.p.mytoolbar.subapLocation=self.c.Get("subapLocation")
            self.p.mytoolbar.npxlx=self.c.Get("npxlx")
            self.p.mytoolbar.npxly=self.c.Get("npxly")
            self.p.mytoolbar.nsub=self.c.Get("nsub")
        #self.p.mytoolbar.nsuby=self.c.Get("nsuby")
            self.p.mytoolbar.subapFlag=self.c.Get("subapFlag")
        except:
            traceback.print_exc()
            print "Unable to get plot info - continuing"
        self.streamDict={}#Entries (short description, long description)
        #get the list of streams.
        try:
            keys=self.c.GetDecimation(local=0).keys()+self.c.GetDecimation(remote=0)['local'].keys()
            for k in keys:
                self.streamDict[k]=(k,k)
        except:
            traceback.print_exc()
            print "Unable to get stream list"
            
        self.subscribeDict={}#entry for each stream subscribed too, (sub,dec) where sub is a flag, whether subscribed or not and dec is the decimation
        for s in streams:
            self.subscribeDict[s]=(1,dec)
        self.p.mytoolbar.subscribeDict=self.subscribeDict
        self.subWid=SubWid(gtk.Window(),self.subscribe,self.p.win,self.grab,self.setLocalDec)
        self.threadNotNeededList=[]
        if len(streams)==0:    
            #need to pop up the subscribbe widget...
            #the user can then decide what to sub to.
            #self.subWid.show(self.streamDict,self.subscribeDict)
            if configdir==None:
                self.showStreams()
            else:#show a list of files.
                pass

            self.threadStreamDict={}
        else:
            self.threadStreamDict={}
            try:
                self.subscribe([(x,1,dec) for x in self.streams])
            except:
                self.showStreams()
                traceback.print_exc()
                print "Unable to subscribe - continuing..."
        self.p.mytoolbar.initialise(self.showStreams,configdir)
        if len(streams)==0 and configdir!=None:
            #show a list of the plotfiles available.
            self.p.mytoolbar.displayFileList(self.p.win)
        t=threading.Thread(target=self.paramThread)
        t.daemon=True
        t.start()


    def paramThread(self):
        """Watches params, and updates plots as necessary"""
        import controlCorba
        while 1:
            reconnect=0
            restart=0
            try:
                self.paramTag,changed=self.c.WatchParam(self.paramTag,["subapLocation","npxlx","npxly","nsub","subapFlag"])
                print "plot WatchParam %s"%str(changed)#Note, this also wakes up if the rtc is stopped - ie all params changed since it has stopped.
            except:
                time.sleep(1)
                traceback.print_exc()
                reconnect=1
            if reconnect==0:
                try:
                    if len(changed)==5:#everything changed - may have been a restart...
                        restart=1

                    if "subapLocation" in changed:
                        self.p.mytoolbar.subapLocation=self.c.Get("subapLocation")
                    if "npxlx" in changed:
                        self.p.mytoolbar.npxlx=self.c.Get("npxlx")
                    if "npxly" in changed:
                        self.p.mytoolbar.npxly=self.c.Get("npxly")
                    if "nsub" in changed:
                        self.p.mytoolbar.nsub=self.c.Get("nsub")
                    if "subapFlag" in changed:
                        self.p.mytoolbar.subapFlag=self.c.Get("subapFlag")
                    if restart:
                        #resubscribe to the data
                        slist=[]
                        for key in self.subscribeDict.keys():
                            slist.append((key,self.subscribeDict[key][0],self.subscribeDict[key][1]))
                        print "Resubscribing"
                        print slist
                        self.subscribe(slist)
                        keys=self.c.GetDecimation(local=0).keys()+self.c.GetDecimation(remote=0)['local'].keys()
                        for k in keys:
                            self.streamDict[k]=(k,k)


                except:
                    reconnect=1
                    traceback.print_exc()
            if reconnect:
                try:
                    self.c.echoString("Test")
                    reconnect=0
                except:
                    pass
            if reconnect:
                try:
                    self.c=controlCorba.controlClient(controlName=self.prefix,debug=0)
                    while self.c.obj==None:
                        time.sleep(1)
                        self.c=controlCorba.controlClient(controlName=self.prefix,debug=0)
                    if restart:
                        #resubscribe to the data
                        slist=[]
                        for key in self.subscribeDict.keys():
                            slist.append([key,self.subscribeDict[key][0],self.subscribeDict[key][1]])
                        print "Resubscribing after connection"
                        self.subscribe(slist)
                        keys=self.c.GetDecimation(local=0).keys()+self.c.GetDecimation(remote=0)['local'].keys()
                        for k in keys:
                            self.streamDict[k]=(k,k)

                except:
                    traceback.print_exc()
                                    
    def setLocalDec(self,stream,dec):
        import buffer
        cb=buffer.Circular("/"+stream)
        cb.freq[0]=dec

    def loadFunc(self,label,data=None,fname=None,args=None):
        if type(label)==type(""):
            #image data - do nothing
            theplot=None
        elif label==None:#want to unsubscribe from everything
            theplot=None
        else:
            #label is a plot list
            plotList=label
            #select the plot we want, return it, first subscribing as appropriate
            indx=0
            theplot=plotList[indx]
            sub=theplot[4]
            initcode=theplot[7]
            if len(initcode)>0:
                d={"darc":self.c,"numpy":numpy,"prefix":self.prefix}
                print "Executing plot initialisation code..."
                try:
                    exec initcode in d
                except:
                    traceback.print_exc()

            if type(sub)!=type([]):
                sub=[sub]
            csub=[]
            for s in sub:
                s=list(s)
                csub.append(s)
                if s[0][:len(self.prefix)]!=self.prefix:
                    s[0]=self.prefix+s[0]
            sub=csub
            slist=[x[0] for x in sub]
            #unsubscribe from everything we don't want...
            for s in self.threadStreamDict.keys():
                if s not in slist:
                    sub.append((s,0,0))
            print sub
            self.subscribe(sub)
            #self.showStreams()
        return theplot
    def grab(self,stream,latest=0,t=None):
        if t==None:#start the thread to grab data
            if type(stream)==type(""):
                t=threading.Thread(target=self.grab,args=())
                t._Thread__args=(stream,latest,t)
                t.start()
            else:#end the thread.
                t=stream[3]
                t.join()
                gobject.idle_add(self.doplot,stream[:3])
        else:#run the thread (grab the data)
                wholeBuffer=0
                if "rtcTimeBuf" in stream:
                    wholeBuffer=1
                data=self.c.GetStream(stream,latest=latest,wholeBuffer=wholeBuffer)
                if wholeBuffer:
                    data[0].shape=data[0].size
                gobject.idle_add(self.grab,["data",stream,data,t]) 
    def plotdata(self,data):
        rt=1-self.p.active
        if self.p.active:
            gtk.gdk.threads_enter()
            stream=data[1]
            #print threading.currentThread(),self.threadNotNeededList
            if threading.currentThread() in self.threadNotNeededList:
                self.threadNotNeededList.remove(threading.currentThread())
                rt=1
                #print "Thread not needed %s"%stream
            elif self.subscribeDict.has_key(stream) and self.subscribeDict[stream][0]==1:
                if self.plotWaitingDict.get(stream,None)==None:
                    gobject.idle_add(self.doplot,stream)#,data)
                self.plotWaitingDict[stream]=data
            else:
                rt=1#unsibscribe from this stream.
                print "Unsubscribing %s"%stream
            gtk.gdk.threads_leave()
        return rt

    def doplot(self,data):
        gtk.gdk.threads_enter()
        if type(data)==type(""):
            stream=data
            data=self.plotWaitingDict.get(data)
            self.plotWaitingDict[stream]=None
        if data!=None:
            stream=data[1]
            fno=data[2][2]
            ftime=data[2][1]
            data=data[2][0]
            #remove prefix - plot doesn't need to know about that...
            stream=stream[len(self.prefix):]

            self.p.mytoolbar.stream[stream]=data
            self.p.mytoolbar.streamName=stream
            self.p.mytoolbar.streamTime[stream]=fno,ftime
            self.p.mytoolbar.streamTimeTxt="%10d %9s%03d"%(fno,time.strftime("%H:%M:%S.",time.localtime(ftime)),(ftime%1)*1000)
            if "rtcStatusBuf" in stream or stream=="Sta":
                self.p.mytoolbar.mangleTxtDefault="data=data.tostring()"
            else:
                self.p.mytoolbar.mangleTxtDefault=""
            self.p.plot(data)
        gtk.gdk.threads_leave()
    def quit(self,source,cbcondition,a=None):
        gtk.main_quit()
        return True
    def showStreams(self,w=None,a=None):
        if self.subWid!=None:
            #should I update the list of streams here?
            orig=self.streamDict.keys()
            try:
                keys=self.c.GetDecimation(local=0).keys()+self.c.GetDecimation(remote=0)['local'].keys()
                for k in keys:
                    self.streamDict[k]=(k,k)
                for k in orig:
                    if k not in keys:
                        self.streamDict[k]=(k+" (dead?)",k+" (dead?)")
            except:
                print "Plot error updating stream list - may be out of date"
            self.subWid.show(self.streamDict,self.subscribeDict)#,self.c.GetDecimation())
        
    def subscribe(self,slist):
        """slist is a list of tuples of (stream,subscribe flag,decimate).
        """
        if type(slist)!=type([]):
            slist=[slist]
        #print slist
        for s in slist:
            if s[1]:#subscribe to it
                #But - what if we're already subscribed?  How do we remove the thread.
                if self.threadStreamDict.has_key(s[0]):
                    self.threadNotNeededList.append(self.threadStreamDict[s[0]])
                self.threadStreamDict[s[0]]=self.c.GetStreamBlock([s[0]],-1,callback=self.plotdata,decimate=s[2],sendFromHead=1,returnthreadlist=1,resetDecimate=0)[0]
                self.subscribeDict[s[0]]=s[1:]
            else:#unsubscribe
                if self.subscribeDict.has_key(s[0]):
                    del(self.subscribeDict[s[0]])
                if self.threadStreamDict.has_key(s[0]):
                    self.threadNotNeededList.append(self.threadStreamDict[s[0]])
                    del(self.threadStreamDict[s[0]])
        self.p.mytoolbar.subscribeDict=self.subscribeDict


if __name__=="__main__":
    if len(sys.argv)==2 and sys.argv[1]=="STDIN":#assume stdin has data, serialised.
        p=StdinServer()
        gtk.main()
    else:
        port=None
        # if len(sys.argv)==3:#could be port,shmtag for connecting to gui.
        #     try:
        #         port=int(sys.argv[1])
        #         shmtag=sys.argv[2].strip()
        #     except:
        #         port=None
        #     if port!=None:
        #         print "Will connect to port %d with shmtag %s"%(port,shmtag)
        #         p=PlotServer(port,shmtag)
        #         gtk.main()
        # if port==None:
        #     if len(sys.argv)==2 and not os.path.exists(sys.argv[1]):
        #         #assume sys.argv is the stream name_hostname_prefix_dec...
        #         info=sys.argv[1].split("_")
        #         streams=info[0].split(",")
        #         myhostname=None
        #         prefix=""
        #         dec=25
        #         if len(info)>1:
        #             myhostname=info[1]
        #         if len(info)>2:
        #             prefix=info[2]
        #         if len(info)>3:
        #             dec=int(info[3])
        #         if len(myhostname.strip())==0:
        #             myhostname=None
        #         print streams,myhostname,prefix,dec
        #         gtk.gdk.threads_init()
        #         d=DarcReader(streams,myhostname,prefix,dec)
        #         gtk.main()
        #     else:
                #What other options could we have?
                #Args could be: plotconfigfile hostname
                #Or could be streamname, decimate
                #Or could be streamname, decimate, prefix.
                #Or other options?
        # if len(sys.argv)>1 and os.path.exists(sys.argv[1]):
        #     #Assume this is an xml file describing the plot...
        #     dec=None
        #     myhostname=None
        #     prefix=""
        #     xml=sys.argv[1]
        #     indx=0
        #     if len(sys.argv)>2:
        #         indx=int(sys.argv[2])

        #     if len(sys.argv)>3:
        #         myhostname=sys.argv[3]
        #     if len(sys.argv)>4:
        #         prefix=sys.argv[4]
        #         try:
        #             dec=int(prefix)
        #             prefix=""
        #         except:
        #             pass
        #     if len(sys.argv)>5:
        #         dec=int(sys.argv[5])#overrides that in xml file.
        #     #Now read the xml file to find the streams and decimations.
        #     #And put the mangle text in place.
        #     #But don't use the window placement info - let WM do it.
        #     #Note - no ability to change decimations or streams.
        #     txt=open(xml).read()
        #     plotList=plotxml.parseXml(txt).getPlots()[indx]
        #     pos=plotList[0]
        #     size=plotList[1]
        #     show=plotList[2]
        #     mangle=plotList[3]
        #     sub=plotList[4]
        #     but=plotList[5]
        #     streams=[]
        #     dmin=0
        #     for s in sub:
        #         if "rtc" in s[0] and "Buf" in s[0]:
        #             streams.append(s[0])
        #             if s[2]>0 and (s[2]<dmin or dmin==0):
        #                 dmin=s[2]
        #     if dec==None:
        #         dec=dmin
        #     if dec==0:
        #         dec=250
        #     print streams,myhostname,prefix,dec
        #     gtk.gdk.threads_init()
        #     d=DarcReader(streams,myhostname,prefix,dec)
        #     #d.p.mytoolbar.mangleTxtDefault=mangle
        #     d.p.mytoolbar.dataMangleEntry.get_buffer().set_text(mangle)
        #     d.p.mytoolbar.mangleTxt=mangle
        #     d.p.mytoolbar.setUserButtons(but)
        #         #for i in range(len(but)):
        #         #    d.p.mytoolbar.tbList[i].set_active(but[i])
        #     if size!=None:
        #         d.p.win.set_default_size(size[0],size[1])
        #         d.p.win.resize(size[0],size[1])
        #     if pos!=None:
        #         d.p.win.move(pos[0],pos[1])

        #     gtk.main()
        # else:
        arglist=[]
        streams=[]
        dec=25
        prefix=""
        mangle=""
        configdir=None
        withScroll=0
        fname=None
        index=0
        #myhostname=None
        for arg in sys.argv[1:]:
            if arg[:2]=='-s':
                streams=arg[2:].split(",")
            elif arg[:2]=="-d":
                dec=int(arg[2:])
            elif arg[:2]=="-p":
                prefix=arg[2:]
            elif arg[:2]=="-m":
                mangle=arg[2:]
            elif "--prefix=" in arg:
                prefix=arg[9:]
            elif "-c" in arg:
                configdir=arg[2:]
            elif "--configdir=" in arg:
                configdir=arg[12:]
            elif "--with-scroll" in arg:
                withScroll=1
            elif arg[:2]=="-f":
                fname=arg[2:]#config file name
            elif arg[:2]=="-i":
                index=int(arg[2:])#index for plot in config file
            #elif arg[:2]=="-h":
            #    myhostname=arg[2:]
            else:
                arglist.append(arg)
        if len(arglist)>0:
            if os.path.exists(arglist[0]):#a config file
                fname=arglist.pop(0)
                if len(arglist)>0:
                    index=int(arglist.pop(0))
                #if len(arglist)>0:
                #    myhostname=arglist.pop(0)
                if len(arglist)>0:
                    if prefix=="":
                        prefix=arglist.pop(0)
                        try:
                            dec=int(prefix)
                            prefix=""
                        except:
                            pass
                if len(arglist)>0:
                    dec=int(arglist.pop(0))

            else:#or a list of streams
                streams+=arglist.pop(0).split(",")
                while len(arglist)>0:
                    arg=arglist.pop(0)
                    try:
                        dec=int(arg)
                    except:
                        if prefix=="":
                            prefix=arg
                        elif mangle=="":
                            mangle=arg
        gtk.gdk.threads_init()
        d=DarcReader(streams,None,prefix,dec,configdir,withScroll)
        if fname!=None:
            print "Loading %s"%fname
            d.p.loadFunc(fname)
            d.subWid.hide()
        elif mangle!=None:
            d.p.mytoolbar.dataMangleEntry.get_buffer().set_text(mangle)
            d.p.mytoolbar.mangleTxt=mangle
        gtk.main()


