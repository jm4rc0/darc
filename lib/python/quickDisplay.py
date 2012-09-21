#!/usr/bin/python
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
import gtk
import numpy
import gobject
import FITS
import controlCorba
import sys
class QuickDisplay:
    def __init__(self,stream,dec,myhostname,cam,decorate=1,normalise=0):
        if stream in ["rtcPxlBuf","rtcCalPxlBuf"]:
            self.dim=2
        else:
            self.dim=1
        self.win=gtk.Window()
        #self.win.set_type_hint(gtk.gdk.WINDOW_TYPE_HINT_SPLASHSCREEN)
        self.win.set_default_size(300,300)
        self.w=self.h=300
        self.win.set_decorated(decorate)#remove the window frame
        self.cam=cam
        self.normalise=normalise
        self.img=gtk.Image()
        #self.img.set_alignment(0,0)
        #self.img.set_padding(0,0)
        self.pixbufImg=None
        #self.event=gtk.EventBox()
        #self.event.connect("button-press-event",self.click)
        #self.event.connect("button-release-event",self.unclick)
        #self.event.connect("motion-notify-event",self.motion)
        #self.event.connect("expose-event",self.expose)
        #box = gtk.ScrolledWindow()
        #box.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        #box.add_with_viewport(self.img)
        #v=box
        lay=gtk.Layout()
        lay.put(self.img,0,0)
        v=lay

        #v=gtk.Viewport()
        #v.add(self.img)

        #box.set_resize_mode(gtk.RESIZE_PARENT)
        if self.dim==2:
            #if cam==None:
            #    self.pixbuf=gtk.gdk.Pixbuf(gtk.gdk.COLORSPACE_RGB,False,8,512,512)
            #else:
            #    self.pixbuf=gtk.gdk.Pixbuf(gtk.gdk.COLORSPACE_RGB,False,8,256,256)
            self.pixbuf=gtk.gdk.Pixbuf(gtk.gdk.COLORSPACE_RGB,False,8,self.w-50,self.h-50)
        else:
            self.pixbuf=None
        self.c=controlCorba.controlClient(debug=0)
        self.r=self.c.GetStreamBlock(stream,-1,callback=self.recvdata,myhostname=myhostname,decimate=dec)
        self.queue=0
        #vbox.pack_start(self.event)
        self.win.add(v)#self.img)
        self.win.connect("delete-event",self.quit)
        lay.connect("size-allocate",self.changeSize,"size allocate")
        #self.img.connect("size-allocate",self.changeSize,"size allocate img")
        #self.img.connect("expose-event",self.reSize,"size allocate img")
        if self.dim==2:
            self.img.set_from_pixbuf(self.pixbuf)
            self.img.queue_draw()
            self.win.show_all()


    def changeSize(self,w,rect,b=None):
        #print w,rect,b
        print "changesize"
        #print self.img.get_parent(),self.img.get_parent().get_parent()
        #print self.img.get_parent().size_request(),rect,self.win.get_size(),self.img.get_parent().get_parent().get_parent()
        #rect.width,rect.height=w.get_parent().get_parent().get_parent().get_size()
        #rect.width-=4
        #rect.height-=4
        if type(w)==gtk.Image:
            rect.width,rect.height=w.get_parent().get_parent().get_size()
        elif type(w)==gtk.Layout:
            rect.width,rect.height=w.get_parent().get_size()
        
        if self.pixbuf.get_width()!=rect.width or self.pixbuf.get_height()!=rect.height:
            print "reshaping",rect.width,self.pixbuf.get_width()
            self.pixbuf=self.pixbuf.scale_simple(rect.width,rect.height,gtk.gdk.INTERP_NEAREST)
            #pb=gtk.gdk.Pixbuf(gtk.gdk.COLORSPACE_RGB,False,8,rect.width,rect.height)
            self.w=rect.width
            self.h=rect.height
            self.img.set_from_pixbuf(self.pixbuf)
            self.pixbuf=self.pixbuf
            #self.img.set_size_request(-1,-1)
            self.img.queue_draw()

    def recvdata(self,data):
        gtk.gdk.threads_enter()
        if self.queue==0:
            self.queue=1
            gobject.idle_add(self.showdata,data[2][0])
        gtk.gdk.threads_leave()
        return 0

    def showdata(self,data):
        #print "showdata"
        gtk.gdk.threads_enter()
        self.queue=0
        if data.dtype.char!='f':
            data=data.astype("f")
        #print self.img.size_request(),self.img.get_parent().size_request(),self.img.get_parent().get_size()
        if self.dim==2:
            if self.cam==None:
                data.shape=256,256
                if self.normalise:
                    for i in range(4):#for each camera
                        #d=data[(i//2)*128*256+i%2:(i//2+1)*128*256:2]
                        d=data[(i//2)*128:(i//2+1)*128,i%2::2]
                        mini=numpy.min(d)
                        d-=mini
                        maxi=numpy.max(d)
                        if maxi>0:
                            d*=255./maxi
                else:
                    mi=numpy.min(data)
                    data-=mi
                    ma=numpy.max(data)
                    if ma>0:
                        data*=255./ma
                d2=numpy.empty((256,256),data.dtype)
                for i in range(4):#for each camera
                    d2[(i//2)*128:(i//2+1)*128,(i%2)*128:(i%2+1)*128]=data[(i//2)*128:(i//2+1)*128,i%2::2]
                data=d2
            else:
                data.shape=256,256
                data=data[(cam//2)*128:(cam//2+1)*128,cam%2::2]
                mi=numpy.min(data)
                data-=mi
                ma=numpy.max(data)
                if ma>0:
                    data*=255./ma
            if self.pixbufImg==None or self.pixbufImg.get_width()!=data.shape[1] or self.pixbufImg.get_height()!=data.shape[0]:
                self.pixbufImg=gtk.gdk.Pixbuf(gtk.gdk.COLORSPACE_RGB,False,8,data.shape[1],data.shape[0])
            d=self.pixbufImg.get_pixels_array()
            if len(data.shape)==2:
                for i in range(3):
                    d[:,:,i]=data
            else:#3D
                d[:]=data
            if self.pixbuf.get_width()==self.w and self.pixbuf.get_height()==self.h:#scale into existing pixbuf
                self.pixbufImg.scale(self.pixbuf,0,0,self.w,self.h,0,0,self.w/float(data.shape[1]),self.h/float(data.shape[0]),gtk.gdk.INTERP_NEAREST)
            else:#create new pixbuf
                self.pixbuf=self.pixbufImg.scale_simple(self.w,self.h,gtk.gdk.INTERP_NEAREST)
                self.img.set_from_pixbuf(self.pixbuf)
            #if pb.get_width()==self.pixbuf.get_width() and pb.get_height()==self.pixbuf.get_height():
            #    #print "copying"
            #    self.pixbuf.get_pixels_array()[:]=pb.get_pixels_array()
            #else:
            #    #print "assigning"
            #    self.pixbuf=pb
            #    self.img.set_from_pixbuf(self.pixbuf)

        else:#1D data - line plot
            if self.pixbuf==None:
                self.pixbuf=gtk.gdk.Pixbuf(gtk.gdk.COLORSPACE_RGB,False,8,data.shape[0],100)
                self.img.set_from_pixbuf(self.pixbuf)
                self.arr=self.pixbuf.get_pixels_array()
                self.win.show_all()
            ma=numpy.max(data)
            mi=numpy.min(data)
            data-=mi
            if(ma-mi)>0:
                data*=99./(ma-mi)
            self.arr[:]=0xff
            for i in range(data.shape[0]):
                self.arr[data[i],i]=0

        self.img.queue_draw()
        gtk.gdk.threads_leave()

    def quit(self,w=None,a=None):
        gtk.main_quit()
        if self.r!=None and type(self.r)!=type({}):
            #print "calling endLoop"
            print self.r
            self.r.d.endLoop()

if __name__=="__main__":
    stream="rtcCalPxlBuf"
    dec=25
    myhostname="10.0.1.243"
    cam=None
    decorate=1
    normalise=0
    for arg in sys.argv[1:]:
        if arg[:2]=="-h":
            myhostname=arg[2:]
            if len(arg[2:])==0:
                print "Usage: %s -hIPaddr -dDecimation -f (no frame) -cCAM -n (normalise) -H (this help message"%sys.argv[0]
                sys.exit(0)

        elif arg[:2]=="-d":
            dec=int(arg[2:])
        elif arg[:2]=="-f":
            decorate=0
        elif arg[:2]=="-c":
            cam=int(arg[2:])
        elif arg[:2]=="-n":
            normalise=1
        elif arg[:2]=="-H":
            print "Usage: %s -hIPaddr -dDecimation -f (no frame) -cCAM -n (normalise) -H (this help message"%sys.argv[0]
            sys.exit(0)
        else:
            stream=arg
    gtk.gdk.threads_init()
                              
    img=QuickDisplay(stream,dec,myhostname,cam,decorate=decorate,normalise=normalise)
    gtk.main()
