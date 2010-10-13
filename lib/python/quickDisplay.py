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
        self.win.set_decorated(decorate)#remove the window frame
        self.cam=cam
        self.normalise=normalise
        self.img=gtk.Image()
        self.img.set_alignment(0,0)
        self.img.set_padding(0,0)
        #self.event=gtk.EventBox()
        #self.event.connect("button-press-event",self.click)
        #self.event.connect("button-release-event",self.unclick)
        #self.event.connect("motion-notify-event",self.motion)
        #self.event.connect("expose-event",self.expose)
        if self.dim==2:
            if cam==None:
                self.pixbuf=gtk.gdk.Pixbuf(gtk.gdk.COLORSPACE_RGB,False,8,512,512)
            else:
                self.pixbuf=gtk.gdk.Pixbuf(gtk.gdk.COLORSPACE_RGB,False,8,256,256)
        else:
            self.pixbuf=None
        self.c=controlCorba.controlClient(debug=0)
        self.c.GetStreamBlock(stream,-1,callback=self.recvdata,myhostname=myhostname,decimate=dec)
        self.queue=0
        #vbox.pack_start(self.event)
        self.win.add(self.img)
        self.win.connect("delete-event",self.quit)
        if self.dim==2:
            self.img.set_from_pixbuf(self.pixbuf)
            self.arr=self.pixbuf.get_pixels_array()
            self.img.queue_draw()
            self.win.show_all()
        #gobject.idle_add(self.newimg)
    # def expose(self,w=None,e=None):
    #     print "Expose"
    #     pass

    def recvdata(self,data):
        gtk.gdk.threads_enter()
        if self.queue==0:
            self.queue=1
            gobject.idle_add(self.showdata,data[2][0])
        gtk.gdk.threads_leave()
        return 0

    def showdata(self,data):
        gtk.gdk.threads_enter()
        self.queue=0
        if data.dtype.char!='f':
            data=data.astype("f")
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
                    
                for i in range(3):#rgb
                    for j in range(4):#expand to 4 pixels per pixel
                        for k in range(4):#for each camera
                            self.arr[(k//2)*256+j//2:(k//2+1)*256:2,(k%2)*256+j%2:(k%2+1)*256:2,i]=data[(k//2)*128:(k//2+1)*128,k%2::2]
#                        self.arr[j//2:256:2,256+j%2:512:2,i]=data[:128,1::2]#cam1
#                        self.arr[256+j//2:512:2,j%2:256:2,i]=data[128:,::2]#cam2
#                        self.arr[256+j//2:512:2,256+j%2:512:2,i]=data[128:,1::2]#cam3
            else:
                data.shape=256,256
                data=data[(cam//2)*128:(cam//2+1)*128,cam%2::2]
                mi=numpy.min(data)
                data-=mi
                ma=numpy.max(data)
                if ma>0:
                    data*=255./ma
                for i in range(3):#rgb
                    for j in range(4):#expand to 4 pixels per pixel
                        self.arr[j//2:256:2,j%2:256:2,i]=data
        else:
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
