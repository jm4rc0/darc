#!/usr/bin/env python
import sys
import socket
import numpy
import gtk
import gobject
import darc
class StreamTool:
    def __init__(self,prefix):
        self.win=gtk.Window()
        self.win.set_title("Stream tool on %s"%socket.gethostname())
        self.win.connect("delete-event",self.quit)
        vbox=gtk.VBox()
        self.win.add(vbox)
        self.prefix=prefix
        h=gtk.HBox()
        vbox.pack_start(h,False)
        h.pack_start(gtk.Label("Prefix:"),False)
        e=gtk.Entry()
        e.set_tooltip_text("darc prefix")
        e.set_width_chars(6)
        e.set_text(self.prefix)
        e.connect("focus-out-event",self.setPrefix)
        e.connect("activate",self.setPrefix)
        h.pack_start(e,False)
        b=gtk.Button("Get")
        b.set_tooltip_text("Get current summers")
        b.connect("clicked",self.getSumSplitBin)
        h.pack_start(b,False)

        self.ls=gtk.ListStore(gobject.TYPE_STRING)
        self.tv=gtk.TreeView(self.ls)
        rt=gtk.CellRendererText()
        col=gtk.TreeViewColumn("Summer",rt,text=0)
        col.set_sort_column_id(0)
        self.tv.append_column(col)
        #self.tv.connect("cursor-changed",self.selectSummer)
        vbox.pack_start(self.tv,False)
        b=gtk.Button("Remove")
        b.set_tooltip_text("Remove a summer")
        b.connect("clicked",self.removeSummer)
        vbox.pack_start(b,False)
        h=gtk.HBox()
        vbox.pack_start(h,False)
        b=gtk.Button("Add")
        b.connect("clicked",self.addSummer)
        b.set_tooltip_text("Start a new summer with the properties below")
        h.pack_start(b,False)
        e=gtk.Entry()
        e.set_width_chars(12)
        e.set_tooltip_text("The stream to sum")
        e.set_text("rtcPxlBuf")
        self.entryStream=e
        h.pack_start(e,False)
        e=gtk.Entry()
        e.set_width_chars(4)
        e.set_tooltip_text("Frames to sum")
        e.set_text("100")
        self.entryNframes=e
        h.pack_start(e,False)
        e=gtk.Entry()
        e.set_width_chars(1)
        e.set_tooltip_text("Data type")
        e.set_text("f")
        h.pack_start(e,False)
        self.entryDatatype=e
        h=gtk.HBox()
        vbox.pack_start(h,False)
        c=gtk.CheckButton("From head")
        c.set_tooltip_text("Check to send from head")
        c.set_active(True)
        h.pack_start(c,False)
        self.checkbuttonHead=c
        c=gtk.CheckButton("Rolling")
        c.set_tooltip_text("Check to start a rolling average")
        c.set_active(False)
        h.pack_start(c,False)
        self.checkbuttonRolling=c
        e=gtk.Entry()
        e.set_width_chars(3)
        e.set_tooltip_text("Number of frames in summed circular buffer")
        e.set_text("10")
        self.entryNstore=e
        h.pack_start(e,False)

        # Now the splitters
        self.lss=gtk.ListStore(gobject.TYPE_STRING)
        self.tvs=gtk.TreeView(self.lss)
        rt=gtk.CellRendererText()
        col=gtk.TreeViewColumn("Splitter",rt,text=0)
        col.set_sort_column_id(0)
        self.tvs.append_column(col)
        #self.tvs.connect("cursor-changed",self.selectSplitter)
        vbox.pack_start(self.tvs,False)
        b=gtk.Button("Remove")
        b.set_tooltip_text("Remove a splitter")
        b.connect("clicked",self.removeSplitter)
        vbox.pack_start(b,False)
        h=gtk.HBox()
        vbox.pack_start(h,False)
        b=gtk.Button("Add")
        b.connect("clicked",self.addSplitter)
        b.set_tooltip_text("Start a new splitter with the properties below")
        h.pack_start(b,False)
        e=gtk.Entry()
        e.set_width_chars(12)
        e.set_tooltip_text("The stream to sum")
        e.set_text("rtcPxlBuf")
        self.entrySplitStream=e
        h.pack_start(e,False)
        e=gtk.Entry()
        e.set_width_chars(5)
        e.set_tooltip_text("The starting index")
        e.set_text("0")
        self.entrySplitStart=e
        h.pack_start(e,False)
        e=gtk.Entry()
        e.set_width_chars(5)
        e.set_tooltip_text("The ending index")
        e.set_text("-1")
        self.entrySplitEnd=e
        h.pack_start(e,False)
        h=gtk.HBox()
        vbox.pack_start(h,False)
        e=gtk.Entry()
        e.set_width_chars(5)
        e.set_tooltip_text("The step size (row size if taking a sub-image)")
        e.set_text("2")
        self.entrySplitStep=e
        h.pack_start(e,False)
        e=gtk.Entry()
        e.set_width_chars(5)
        e.set_tooltip_text("The block size (this many elements read per step)")
        e.set_text("1")
        self.entrySplitBlock=e
        h.pack_start(e,False)
        c=gtk.CheckButton("From head")
        c.set_tooltip_text("Check to send from head")
        c.set_active(True)
        h.pack_start(c,False)
        self.checkbuttonSplitHead=c
        e=gtk.Entry()
        e.set_width_chars(3)
        e.set_tooltip_text("The circular buffer size")
        e.set_text("100")
        self.entrySplitNstore=e
        h.pack_start(e,False)
        
        #now the binners
        self.lsb=gtk.ListStore(gobject.TYPE_STRING)
        self.tvb=gtk.TreeView(self.lsb)
        rt=gtk.CellRendererText()
        col=gtk.TreeViewColumn("Binner",rt,text=0)
        col.set_sort_column_id(0)
        self.tvb.append_column(col)
        vbox.pack_start(self.tvb,False)
        b=gtk.Button("Remove")
        b.set_tooltip_text("Remove a binner")
        b.connect("clicked",self.removeBinner)
        vbox.pack_start(b,False)
        h=gtk.HBox()
        vbox.pack_start(h,False)
        b=gtk.Button("Add")
        b.connect("clicked",self.addBinner)
        b.set_tooltip_text("Start a new binner with the properties below")
        h.pack_start(b,False)
        e=gtk.Entry()
        e.set_width_chars(12)
        e.set_tooltip_text("The stream to bin")
        e.set_text("rtcPxlBuf")
        self.entryBinStream=e
        h.pack_start(e,False)
        e=gtk.Entry()
        e.set_width_chars(5)
        e.set_tooltip_text("The bin factor (x)")
        e.set_text("2")
        self.entryBinX=e
        h.pack_start(e,False)
        e=gtk.Entry()
        e.set_width_chars(5)
        e.set_tooltip_text("The bin factor (y)")
        e.set_text("1")
        self.entryBinY=e
        h.pack_start(e,False)
        e=gtk.Entry()
        e.set_width_chars(1)
        e.set_tooltip_text("The data type")
        e.set_text("n")
        self.entryBinDtype=e
        h.pack_start(e,False)

        
        e=gtk.Entry()
        e.set_width_chars(5)
        e.set_tooltip_text("The starting index")
        e.set_text("0")
        self.entryBinStart=e
        h.pack_start(e,False)
        e=gtk.Entry()
        e.set_width_chars(5)
        e.set_tooltip_text("The ending index")
        e.set_text("-1")
        self.entryBinEnd=e
        h.pack_start(e,False)
        h=gtk.HBox()
        vbox.pack_start(h,False)
        e=gtk.Entry()
        e.set_width_chars(5)
        e.set_tooltip_text("The stride size (row size if 2D data to be binned)")
        e.set_text("-1")
        self.entryBinStride=e
        h.pack_start(e,False)
        c=gtk.CheckButton("From head")
        c.set_tooltip_text("Check to send from head")
        c.set_active(True)
        h.pack_start(c,False)
        self.checkbuttonBinHead=c
        e=gtk.Entry()
        e.set_width_chars(3)
        e.set_tooltip_text("The circular buffer size")
        e.set_text("100")
        self.entryBinNstore=e
        h.pack_start(e,False)


        gobject.timeout_add(10,self.getSumSplitBin)
        self.win.show_all()
    def quit(self,w,a=None):
        gtk.main_quit()
    def setPrefix(self,e,a=None):
        self.prefix=e.get_text()
        self.getSumSplitBin()
    def getSumSplitBin(self,w=None,a=None):
        d=darc.Control(self.prefix)
        lst=d.GetSummerList()
        print lst
        lst.sort()
        self.ls.clear()
        for s in lst:
            self.ls.append([s])
        self.tv.show_all()

        lst=d.GetSplitterList()
        print lst
        lst.sort()
        self.lss.clear()
        for s in lst:
            self.lss.append([s])
        self.tvs.show_all()
        return False

        lst=d.GetBinnerList()
        print lst
        lst.sort()
        self.lsb.clear()
        for s in lst:
            self.lsb.append([s])
        self.tvb.show_all()
        return False

    def removeSummer(self,w,a=None):
        m=self.tv.get_model()
        c=self.tv.get_cursor()
        txt=m[c[0]][0]
        print "Removing",txt
        d=darc.Control(self.prefix)
        d.StopSummer(txt)
        self.getSumSplitBin()
    def addSummer(self,w,a=None):
        s=self.entryStream.get_text()
        n=int(self.entryNframes.get_text())
        f=self.entryDatatype.get_text()
        h=int(self.checkbuttonHead.get_active())
        r=int(self.checkbuttonRolling.get_active())
        ns=int(self.entryNstore.get_text())
        print "adding",s,n,f,h,r,ns
        d=darc.Control(self.prefix)
        d.StartSummer(s,n,fromHead=h,rolling=r,dtype=f,nstore=ns)
        self.getSumSplitBin()
    # def selectSplitter(self,w,row=None,col=None):
    #     m=w.get_model()
    #     c=w.get_cursor()
    #     txt=m[c[0]][0]
    #     print txt
    def removeSplitter(self,w,a=None):
        m=self.tvs.get_model()
        c=self.tvs.get_cursor()
        txt=m[c[0]][0]
        print "Removing",txt
        d=darc.Control(self.prefix)
        d.StopSplitter(txt)
        self.getSumSplitBin()
    def addSplitter(self,w,a=None):
        stream=self.entrySplitStream.get_text()
        s=int(self.entrySplitStart.get_text())
        e=int(self.entrySplitEnd.get_text())
        step=int(self.entrySplitStep.get_text())
        block=int(self.entrySplitBlock.get_text())
        h=int(self.checkbuttonSplitHead.get_active())
        ns=int(self.entrySplitNstore.get_text())
        print "adding",stream,s,e,step,block,h,ns
        d=darc.Control(self.prefix)
        name=d.StartSplitter(stream,s,e,step,block,fromHead=h,nstore=ns)
        self.getSumSplitBin()

    def removeBinner(self,w,a=None):
        m=self.tvb.get_model()
        c=self.tvb.get_cursor()
        txt=m[c[0]][0]
        print "Removing",txt
        d=darc.Control(self.prefix)
        d.StopBinner(txt)
        self.getSumSplitBin()
    def addBinner(self,w,a=None):
        stream=self.entryBinStream.get_text()
        s=int(self.entryBinStart.get_text())
        e=int(self.entryBinEnd.get_text())
        x=int(self.entryBinX.get_text())
        y=int(self.entryBinY.get_text())
        dt=(self.entryBinDtype.get_text())
        stride=int(self.entryBinStride.get_text())
        h=int(self.checkbuttonSplitHead.get_active())
        ns=int(self.entrySplitNstore.get_text())
        print "adding",stream,x,y,dt,s,e,stride,h,ns
        d=darc.Control(self.prefix)
        name=d.StartBinner(stream,x,y,s,e,stride,dt,fromHead=h,nstore=ns)
        self.getSumSplitBin()

        
if __name__=="__main__":
    prefix=""
    i=1
    while i<len(sys.argv):
        if sys.argv[i].startswith("--prefix="):
            prefix=sys.argv[i][9:]
        else:
            prefix=sys.argv[i]
        i+=1
    s=StreamTool(prefix)
    gtk.main()

