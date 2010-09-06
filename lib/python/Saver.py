import numpy
#import mmap
import FITS
import sys
class Saver:
    """Class to implement saving of RTC streams"""
    def __init__(self,name,mode="a"):
        self.name=name
        self.fd=open(name,mode)
        self.info=numpy.zeros((8,),numpy.int32)
    def write(self,data,ftime,fno):
        self.info[0]=(self.info.size-1)*self.info.itemsize+data.size*data.itemsize#number of bytes to follow (excluding these 4)
        self.info[1]=fno
        self.info[2:4].view(numpy.float64)[0]=ftime
        self.info.view("c")[16]=data.dtype.char
        self.fd.write(self.info.data)
        self.fd.write(data.data)
    def writeRaw(self,data):#data should be of the correct format... ie same as that written by self.write()
        if type(data)==type(""):
            self.fd.write(data)
        else:
            self.fd.write(data.data)
    def close(self):
        self.fd.close()
    def read(self,readdata=1,ffrom=None,fto=None,tfrom=None,tto=None):
        data=[]
        frame=None
        while 1:
            hdr=self.fd.read(self.info.size*self.info.itemsize)
            if hdr=="":#end of file...
                return data
            elif len(hdr)<self.info.size*self.info.itemsize:
                print "Didn't read all of header"
                return data
            info=numpy.fromstring(hdr,numpy.int32)
            fno=int(info[1])
            ftime=float(info[2:4].view("d"))
            databytes=info[0]-(self.info.size-1)*self.info.itemsize
            fok=tok=0
            #print fno,ffrom,fto
            if (ffrom==None or fno>=ffrom) and (fto==None or fno<=fto):
                #print fno
                fok=1
            if (tfrom==None or ftime>=tfrom) and (tto==None or ftime<=tto):
                tok=1
            if readdata==1 and fok==1 and tok==1:
                frame=self.fd.read(databytes)
                if len(frame)!=databytes:
                    print "Didn't read all of frame"
                    return data
                frame=numpy.fromstring(frame,chr(info[4]))
                data.append((fno,ftime,frame))
            else:
                #skip the data.
                self.fd.seek(databytes-1,1)
                if self.fd.read(1)=="":#read the last byte to check we've not reached end of file.
                    print "Didn't read all of frame"
                    return data

                frame=None
    def tofits(self,fname,ffrom=None,fto=None,tfrom=None,tto=None):
        curshape=None
        curdtype=None
        fheader=None
        nentries=0
        tlist=[]
        flist=[]
        ffits=open(fname,"w")
        firstHeader=1
        while 1:
            hdr=self.fd.read(self.info.size*self.info.itemsize)
            if hdr=="":
                break
            elif len(hdr)<self.info.size*self.info.itemsize:
                print "Didn't read all of header"
                break
            info=numpy.fromstring(hdr,numpy.int32)
            fno=int(info[1])
            ftime=float(info[2:4].view("d"))
            databytes=info[0]-(self.info.size-1)*self.info.itemsize
            fok=tok=0
            if (ffrom==None or fno>=ffrom) and (fto==None or fno<=fto):
                #print fno
                fok=1
            if (tfrom==None or ftime>=tfrom) and (tto==None or ftime<=tto):
                tok=1
            if fok==1 and tok==1:
                frame=self.fd.read(databytes)
                if len(frame)!=databytes:
                    print "Didn't read all of frame"
                    break
                frame=numpy.fromstring(frame,chr(info[4])).byteswap()
                #can it be put into the existing HDU?  If not, finalise current, and start a new one.
                if curshape!=databytes or curdtype!=chr(info[4]):
                    #end the current HDU
                    FITS.End(ffits)
                    #Update FITS header
                    if fheader!=None:
                        FITS.updateLastAxis(None,nentries,fheader)
                        del(fheader)
                        #fheader.close()
                        fheader=None
                    #now write the frame number and time.
                    ffits.close()
                    if firstHeader==0:
                        FITS.Write(numpy.array(flist).astype("i"),fname,writeMode="a")
                        FITS.Write(numpy.array(tlist),fname,writeMode="a")
                    ffits=open(fname,"a+")
                    FITS.WriteHeader(ffits,[1,databytes/numpy.zeros((1,),chr(info[4])).itemsize],chr(info[4]),firstHeader=firstHeader)
                    ffits.flush()
                    firstHeader=0
                    fheader=numpy.memmap(fname,dtype="c",mode="r+",offset=ffits.tell()-2880)
                    flist=[]
                    tlist=[]
                    nentries=0
                    curshape=databytes
                    curdtype=chr(info[4])
                #now write the data
                ffits.write(frame)
                nentries+=1
                flist.append(fno)
                tlist.append(ftime)
            else:
                #skip the data
                self.fd.seek(databytes-1,1)
                if self.rd.read(1)=="":
                    print "Didn't read all of the frame"
                    break
        #now finalise the file.
        FITS.End(ffits)
        if fheader!=None:
            FITS.updateLastAxis(None,nentries,fheader)
            #fheader.close()
            del(fheader)
            fheader=None
        #now write the frame number and time.
        ffits.close()
        FITS.Write(numpy.array(flist).astype("i"),fname,writeMode="a")
        FITS.Write(numpy.array(tlist),fname,writeMode="a")

if __name__=="__main__":
    if len(sys.argv)>1:
        if sys.argv[1]=="convert":
            iname=sys.argv[2]
            oname=sys.argv[3]
            s=Saver(iname,"r")
            s.tofits(oname)
