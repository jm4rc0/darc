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
import numpy
def transformPSF(psf,ncam,npxlx,npxly,nsub,subapLocation,subflag,pad=None,savespace=0):
    """Function to transform psf into a form that will be usable by the RTC.
    psf is eg the LGS spot elongation pattern.
    ncam is number of cameras
    npxlx/y is array of size ncam with number of pixels for each camera
    nsubx/y is array size ncam with number of subaps for each camera
    subapLocation is array containing subap location details, one set of entries per subaperture.
    If pad is set, it will pad the fft pattern, and also return lots of other things required by darc.  If savespace is also set (as per default), the returned fft pattern subaperture locations will bear no resemblance to the wavefront sensors.
    """
    if psf==None:
        return None
    returnAll=1
    if pad==None:
        pad=0
        returnAll=0
    psfIn=psf.ravel()
    pxlFrom=0
    pxlOutFrom=0
    pos=0
    if pad:
        #work out how big the output needs to be.
        #if nsubx==None or nsuby==None:
        #    raise Exception("nsubx, nsuby required")
        if savespace:
            size=0
            nxout=[]
            nyout=[]
            subapLocOut=subapLocation.copy()
            #now we have to work out where to put the new correlation subaps.  Probably easiest just to put them all in one big long line.
            maxx=numpy.max((subapLocation[:,4]+2*pad*subapLocation[:,5]-subapLocation[:,3])/numpy.where(subapLocation[:,5]==0,1000,subapLocation[:,5]))
            ysizes=(subapLocation[:,1]+2*pad*subapLocation[:,2]-subapLocation[:,0])/numpy.where(subapLocation[:,2]==0,1000,subapLocation[:,2])
            toty=ysizes.sum()
            #subapLocOut[:,1::3]+=2*pad*subapLocOut[:,2::3]
            subapLocOut=numpy.zeros(subapLocation.shape,numpy.int32)
            suboff=0
            for i in range(ncam):
                #nx=npxlx[i]+nsubx[i]*2*pad
                #ny=npxly[i]+nsuby[i]*2*pad
                yindx=0
                for j in range(nsub[i]):
                    if subflag[suboff+j]:
                        subsizey=((subapLocation[suboff+j,1]-subapLocation[suboff+j,0])/subapLocation[suboff+j,2])+2*pad
                        subsizex=((subapLocation[suboff+j,4]-subapLocation[suboff+j,3])/subapLocation[suboff+j,5])+2*pad
                        subapLocOut[suboff+j]=(yindx,yindx+subsizey,1,0,subsizex,1)
                        yindx+=subsizey

                nx=numpy.max(subapLocOut[suboff:suboff+nsub[i],4])
                ny=yindx#numpy.max(subapLocOut[suboff:suboff+nsub[i],1])
                suboff+=nsub[i]
                nxout.append(nx)
                nyout.append(ny)
                size+=nx*ny
        else:#produce subap locations that try to look like the physical wfs.
            maxx=numpy.max((subapLocation[:,4]-subapLocation[:,3])/numpy.where(subapLocation[:,5]==0,1000,subapLocation[:,5]))
            maxy=numpy.max((subapLocation[:,1]-subapLocation[:,0])/numpy.where(subapLocation[:,2]==0,1000,subapLocation[:,2]))
            sx=(maxx+pad*2.)/maxx
            sy=(maxy+pad*2.)/maxy
            #print maxx,maxy,sx,sy
            sl2=subapLocation.copy()
            mod=(sl2[:,3]%numpy.where(sl2[:,5]==0,1000,sl2[:,5]))
            sl2[:,3]=(sl2[:,3]-mod)*sx+mod
            mod=(sl2[:,0]%numpy.where(sl2[:,2]==0,1000,sl2[:,2]))
            sl2[:,0]=(sl2[:,0]-mod)*sy+mod
            sl2[:,1]=sl2[:,0]+2*pad*sl2[:,2]+(subapLocation[:,1]-subapLocation[:,0])#//numpy.where(subapLocation[:,2]==0,1000,subapLocation[:,2])
            sl2[:,4]=sl2[:,3]+2*pad*sl2[:,5]+(subapLocation[:,4]-subapLocation[:,3])#//numpy.where(subapLocation[:,5]==0,1000,subapLocation[:,5])
            soff=0
            nxout=[]
            nyout=[]
            size=0
            for i in range(ncam):
                nx=numpy.max(sl2[soff:soff+nsub[i],4])
                ny=numpy.max(sl2[soff:soff+nsub[i],1])
                nxout.append(nx)
                nyout.append(ny)
                soff+=nsub[i]
                size+=nx*ny
            #now check for overlaps...
            tmp=numpy.zeros((size,),numpy.int32)
            soff=0
            poff=0
            sf=subflag
            for i in range(ncam):
                timg=tmp[poff:poff+nxout[i]*nyout[i]]
                timg.shape=nyout[i],nxout[i]
                for j in range(nsub[i]):
                    if sf[j+soff]:
                        s=sl2[j+soff]
                        timg[s[0]:s[1]:s[2],s[3]:s[4]:s[5]]+=1
                soff+=nsub[i]
                poff+=nxout[i]*nyout[i]
            if numpy.any(tmp>1):
                for i in range(6):
                    print sl2[:,i]
                raise Exception("Unable to calculate subaps...")
            subapLocOut=sl2
        psfOut=numpy.zeros((size,),numpy.float32)
    else:
        psfOut=numpy.zeros(psfIn.shape,numpy.float32)
        nxout=npxlx
        nyout=npxly
        subapLocOut=subapLocation
    pos=0
    for i in range(ncam):
        #get psf for this camera
        img=psfIn[pxlFrom:pxlFrom+npxlx[i]*npxly[i]]
        img.shape=npxly[i],npxlx[i]
        res=psfOut[pxlOutFrom:pxlOutFrom+nxout[i]*nyout[i]]
        res.shape=nyout[i],nxout[i]
        for j in range(nsub[i]):
            if subflag[pos]:
                loc=subapLocation[pos]
                locout=subapLocOut[pos]
                subpsf=img[loc[0]:loc[1]:loc[2],loc[3]:loc[4]:loc[5]]
                if pad:
                    sp=numpy.zeros((subpsf.shape[0]+2*pad,subpsf.shape[1]+2*pad),numpy.float32)
                    sp[pad:-pad,pad:-pad]=subpsf
                    subpsf=sp
                ff=r2hc(numpy.fft.fftshift(subpsf))
                # do the conjugate.
                mm=ff.shape[0]//2+1
                nn=ff.shape[1]//2+1
                ff[:mm,nn:]*=-1
                ff[mm:,:nn]*=-1
                # and now put it into the result.
                #print locout,ff.shape,res.shape
                res[locout[0]:locout[1]:locout[2],locout[3]:locout[4]:locout[5]]=ff


            pos+=1
        pxlFrom+=npxlx[i]*npxly[i]
        pxlOutFrom+=nxout[i]*nyout[i]
    if pad or returnAll:
        npxlout=numpy.array(nxout)*numpy.array(nyout)
        npxloutcum=[0]
        for np in npxlout:
            npxloutcum.append(np+npxloutcum[-1])
        return {"corrFFTPattern":psfOut,"corrSubapLoc":subapLocOut,"corrNpxlx":numpy.array(nxout).astype(numpy.int32),"corrNpxlCum":numpy.array(npxloutcum).astype(numpy.int32),"corrNpxly":numpy.array(nyout).astype(numpy.int32)}
    else:
        return psfOut
        
def untransformPSF(psf,ncam,npxlx,npxly,nsub,subapLocation,subflag):
    """Undoes transformPSF"""
    if psf==None:
        return None
    psfIn=psf.ravel()
    psfOut=numpy.zeros(psfIn.shape,numpy.float32)
    pxlFrom=0
    pos=0
    for i in range(ncam):
        #get psf for this camera
        img=psfIn[pxlFrom:pxlFrom+npxlx[i]*npxly[i]]
        img.shape=npxly[i],npxlx[i]
        res=psfOut[pxlFrom:pxlFrom+npxlx[i]*npxly[i]]
        res.shape=npxly[i],npxlx[i]
        for j in range(nsub[i]):
            if subflag[pos]:
                loc=subapLocation[pos]
                subpsf=img[loc[0]:loc[1]:loc[2],loc[3]:loc[4]:loc[5]]
                    # do the conjugate.
                mm=subpsf.shape[0]/2+1
                nn=subpsf.shape[1]/2+1
                subpsf[:mm,nn:]*=-1
                subpsf[mm:,:nn]*=-1
                ff=numpy.fft.fftshift(hc2r(subpsf))
                
                    #and now put it into the result.
                res[loc[0]:loc[1]:loc[2],loc[3]:loc[4]:loc[5]]=ff


            pos+=1
        pxlFrom+=npxlx[i]*npxly[i]
    return psfOut

def unpackImg(img,ncam,nsub,subflag,npxlx,npxly,sl,npxlx2,npxly2,sl2):
    """Takes img and unpacks subaps from sl to sl2, clipping if necessary."""
    npxl=npxlx*npxly
    npxl2=npxlx2*npxly2
    sl.shape=sl.size/6,6
    sl2.shape=sl2.size/6,6
    out=numpy.zeros(npxl2.sum(),numpy.float32)
    soff=0
    start=0
    start2=0
    for i in range(ncam):
        thisimg=img[start:start+npxl[i]]
        thisimg.shape=npxly[i],npxlx[i]
        thisout=out[start2:start2+npxl2[i]]
        thisout.shape=npxly2[i],npxlx2[i]
        #print start,start2
        for j in range(nsub[i]):
            if subflag[soff+j]:
                s=sl[soff+j]
                t=thisimg[s[0]:s[1]:s[2],s[3]:s[4]:s[5]]
                s2=sl2[soff+j]
                t2=thisout[s2[0]:s2[1]:s2[2],s2[3]:s2[4]:s2[5]]
                h1=t.shape
                h2=t2.shape
                if h2[0]<h1[0] and h2[1]<h1[1]:
                    dy=(h1[0]-h2[0])/2
                    dx=(h1[1]-h2[1])/2
                    t2[:]=t[dy:dy+h2[0],dx:dx+h2[1]]
                elif h2[0]>h1[0] and h2[1]>h1[1]:
                    dy=-(h1[0]-h2[0])/2
                    dx=-(h1[1]-h2[1])/2
                    t2[dy:dy+h1[0],dx:dx+h1[1]]=t
                elif h2[0]==h1[0] and h2[1]==h1[1]:#same shape
                    t2[:]=t
        soff+=nsub[i]
        start+=npxl[i]
        start2+=npxl2[i]
    return out

def makeCorrelation(psf,img,ncam,npxlx,npxly,nsub,subapLocation,subflag,pad=None,savespace=0):
    """Correlates 2 images with optional padding"""
    psfIn=psf.ravel()
    imgIn=img.ravel()
    pxlFrom=0
    pxlOutFrom=0
    pos=0
    if pad:
        #work out how big the output needs to be.
        #if nsubx==None or nsuby==None:
        #    raise Exception("nsubx, nsuby required")
        if savespace:
            size=0
            nxout=[]
            nyout=[]
            subapLocOut=subapLocation.copy()
            #now we have to work out where to put the new correlation subaps.  Probably easiest just to put them all in one big long line.
            maxx=numpy.max((subapLocation[:,4]+2*pad*subapLocation[:,5]-subapLocation[:,3])/numpy.where(subapLocation[:,5]==0,1000,subapLocation[:,5]))
            ysizes=(subapLocation[:,1]+2*pad*subapLocation[:,2]-subapLocation[:,0])/numpy.where(subapLocation[:,2]==0,1000,subapLocation[:,2])
            toty=ysizes.sum()
            #subapLocOut[:,1::3]+=2*pad*subapLocOut[:,2::3]
            subapLocOut=numpy.zeros(subapLocation.shape,numpy.int32)
            suboff=0
            for i in range(ncam):
                #nx=npxlx[i]+nsubx[i]*2*pad
                #ny=npxly[i]+nsuby[i]*2*pad
                yindx=0
                for j in range(nsub[i]):
                    if subflag[suboff+j]:
                        subsizey=((subapLocation[suboff+j,1]-subapLocation[suboff+j,0])/subapLocation[suboff+j,2])+2*pad
                        subsizex=((subapLocation[suboff+j,4]-subapLocation[suboff+j,3])/subapLocation[suboff+j,5])+2*pad
                        subapLocOut[suboff+j]=(yindx,yindx+subsizey,1,0,subsizex,1)
                        yindx+=subsizey

                nx=numpy.max(subapLocOut[suboff:suboff+nsub[i],4])
                ny=yindx#numpy.max(subapLocOut[suboff:suboff+nsub[i],1])
                suboff+=nsub[i]
                nxout.append(nx)
                nyout.append(ny)
                size+=nx*ny
        else:#produce subap locations that try to look like the physical wfs.
            maxx=numpy.max((subapLocation[:,4]-subapLocation[:,3])/numpy.where(subapLocation[:,5]==0,1000,subapLocation[:,5]))
            maxy=numpy.max((subapLocation[:,1]-subapLocation[:,0])/numpy.where(subapLocation[:,2]==0,1000,subapLocation[:,2]))
            sx=(maxx+pad*2.)/maxx
            sy=(maxy+pad*2.)/maxy
            #print maxx,maxy,sx,sy
            sl2=subapLocation.copy()
            mod=(sl2[:,3]%numpy.where(sl2[:,5]==0,1000,sl2[:,5]))
            sl2[:,3]=(sl2[:,3]-mod)*sx+mod
            mod=(sl2[:,0]%numpy.where(sl2[:,2]==0,1000,sl2[:,2]))
            sl2[:,0]=(sl2[:,0]-mod)*sx+mod
            sl2[:,1]=sl2[:,0]+2*pad*sl2[:,2]+(subapLocation[:,1]-subapLocation[:,0])#//numpy.where(subapLocation[:,2]==0,1000,subapLocation[:,2])
            sl2[:,4]=sl2[:,3]+2*pad*sl2[:,5]+(subapLocation[:,4]-subapLocation[:,3])#//numpy.where(subapLocation[:,5]==0,1000,subapLocation[:,5])
            soff=0
            nxout=[]
            nyout=[]
            size=0
            for i in range(ncam):
                nx=numpy.max(sl2[soff:soff+nsub[i],4])
                ny=numpy.max(sl2[soff:soff+nsub[i],1])
                nxout.append(nx)
                nyout.append(ny)
                soff+=nsub[i]
                size+=nx*ny
            #now check for overlaps...
            tmp=numpy.zeros((size,),numpy.int32)
            soff=0
            poff=0
            sf=subflag
            for i in range(ncam):
                timg=tmp[poff:poff+nxout[i]*nyout[i]]
                timg.shape=nyout[i],nxout[i]
                for j in range(nsub[i]):
                    if sf[j+soff]:
                        s=sl2[j+soff]
                        timg[s[0]:s[1]:s[2],s[3]:s[4]:s[5]]+=1
                soff+=nsub[i]
                poff+=nxout[i]*nyout[i]
            if numpy.any(tmp>1):
                for i in range(6):
                    print sl2[:,i]
                raise Exception("Unable to calculate subaps...")
            subapLocOut=sl2
        psfOut=numpy.zeros((size,),numpy.float32)
    else:
        psfOut=numpy.zeros(psfIn.shape,numpy.float32)
        nxout=npxlx
        nyout=npxly
        subapLocOut=subapLocation
    pos=0
    for i in range(ncam):
        #get psf for this camera
        img=psfIn[pxlFrom:pxlFrom+npxlx[i]*npxly[i]]
        img.shape=npxly[i],npxlx[i]
        thisimgIn=imgIn[pxlFrom:pxlFrom+npxlx[i]*npxly[i]]
        thisimgIn.shape=npxly[i],npxlx[i]
        res=psfOut[pxlOutFrom:pxlOutFrom+nxout[i]*nyout[i]]
        res.shape=nyout[i],nxout[i]
        for j in range(nsub[i]):
            if subflag[pos]:
                loc=subapLocation[pos]
                locout=subapLocOut[pos]
                subpsf=img[loc[0]:loc[1]:loc[2],loc[3]:loc[4]:loc[5]]
                subpsf2=thisimgIn[loc[0]:loc[1]:loc[2],loc[3]:loc[4]:loc[5]]
                if pad:
                    sp=numpy.zeros((subpsf.shape[0]+2*pad,subpsf.shape[1]+2*pad),numpy.float32)
                    sp[pad:-pad,pad:-pad]=subpsf
                    subpsf=sp
                    sp=numpy.zeros((subpsf2.shape[0]+2*pad,subpsf2.shape[1]+2*pad),numpy.float32)
                    sp[pad:-pad,pad:-pad]=subpsf2
                    subpsf2=sp
                ff=correlate(subpsf,subpsf2)
                res[locout[0]:locout[1]:locout[2],locout[3]:locout[4]:locout[5]]=ff


            pos+=1
        pxlFrom+=npxlx[i]*npxly[i]
        pxlOutFrom+=nxout[i]*nyout[i]
    npxlout=numpy.array(nxout)*numpy.array(nyout)
    npxloutcum=[0]
    for np in npxlout:
        npxloutcum.append(np+npxloutcum[-1])
    return {"correlation":psfOut,"corrSubapLoc":subapLocOut,"corrNpxlx":numpy.array(nxout).astype(numpy.int32),"corrNpxlCum":numpy.array(npxloutcum).astype(numpy.int32),"corrNpxly":numpy.array(nyout).astype(numpy.int32)}
    

def generateIdentityCorr(npxlx,npxly,nsub,sf,sl):
    """Place a dot at the centre of each subap."""
    img=numpy.zeros(((npxlx*npxly).sum(),),numpy.float32)
    start=0
    soffset=0
    for i in range(len(npxlx)):
        tmp=img[start:start+npxlx[i]*npxly[i]]
        tmp.shape=npxly[i],npxlx[i]
        for j in range(nsub[i]):
            if sf[soffset+j]:
                s=sl[soffset+j]
                tmp[(s[0:3].sum()-1)//2,(s[3:6].sum()-1)//2]=1
        soffset+=nsub[i]
        start+=npxlx[i]*npxly[i]
    return img



def r2hc(a):
    """FFT r to hc.
    """
    a=a.copy()
    for i in range(a.shape[0]):
        a[i]=r2hc1d(a[i])
    for i in range(a.shape[1]):
        a[:,i]=r2hc1d(a[:,i])
    return a


def r2hc1d(a):
    """1D version
    """
    res=numpy.fft.fft(a)
    b=a.copy()
    n=a.shape[0]
    b[:n/2+1]=res.real[:n/2+1]
    b[n/2+1:]=res.imag[(n+1)/2-1:0:-1]
    #b[n/2+1:]=res.imag[1:n/2]
    return b

def hc2r1d(a):
    b=numpy.zeros(a.shape,numpy.complex64)
    n=b.shape[0]
    b.real[:n/2+1]=a[:n/2+1]
    b.real[n/2+1:]=a[(n+1)/2-1:0:-1]
    b.imag[(n+1)/2-1:0:-1]=a[n/2+1:]
    b.imag[(n+0)/2+1:]=-a[(n+0)/2+1:]
    res=numpy.fft.ifft(b).real
    return res


def hc2r(a):
    """inverse FFT hc to r
    """
    a=a.copy()
    for i in range(a.shape[0]):
        a[i]=hc2r1d(a[i])
    for i in range(a.shape[1]):#I dont think it matters which axis you go over first.
        a[:,i]=hc2r1d(a[:,i])
    return a

def unpack(a):
    """a is the output from r2hc.  Here, we expand into a standard complex array, so that by inverting this, can test you understand what is going on.
    """
    res=numpy.zeros(a.shape,numpy.complex64)
    m=a.shape[0]
    n=a.shape[1]
    for i in range(1,(m+1)/2):
        res[i,0]=a[i,0]+1j*a[m-i,0]
        res[m-i,0]=a[i,0]-1j*a[m-i,0]
        if n%2==0:
            res[i,n/2]=a[i,n/2]+1j*a[m-i,n/2]
            res[m-i,n/2]=a[i,n/2]-1j*a[m-i,n/2]
        

        for j in range(1,(n+1)/2):
            if i==1:
                res[0,j]=a[0,j]+1j*a[0,n-j]
                res[0,n-j]=a[0,j]-1j*a[0,n-j]
                if m%2==0:
                    res[m/2,j]=a[m/2,j]+1j*a[m/2,n-j]
                    res[m/2,n-j]=a[m/2,j]-1j*a[m/2,n-j]
            res[i,j]=a[i,j]-a[m-i,n-j]+1j*(a[i,n-j]+a[m-i,j])
            res[m-i,n-j]=a[i,j]-a[m-i,n-j]-1j*(a[i,n-j]+a[m-i,j])
            res[i,n-j]=a[i,j]+a[m-i,n-j]+1j*(a[m-i,j]-a[i,n-j])
            res[m-i,j]=a[i,j]+a[m-i,n-j]-1j*(a[m-i,j]-a[i,n-j])
    res[0,0]=a[0,0]+0j
    if n%2==0:
        res[0,n/2]=a[0,n/2]+0j
    else:
        res[0,n/2]=a[0,n/2]+1j*a[0,n/2+1]
        
    if m%2==0:
        res[m/2,0]=a[m/2,0]+0j
    else:
        res[n/2,0]=a[n/2,0]+1j*a[n/2+1,0]
        
    if n%2==0 and m%2==0:
        res[m/2,n/2]=a[m/2,n/2]+0j
    return res

def pack(a):
    """Does the inverse of unpack - converts a 2D FFT to HC format."""
    res=numpy.zeros(a.shape,numpy.float32)
    m=a.shape[0]
    n=a.shape[1]
    for i in range(1,(m+1)/2):
        res[i,0]=a[i,0].real
        res[m-i,0]=a[i,0].imag
        if n%2==0:
            res[i,n/2]=a[i,n/2].real
            res[m-i,n/2]=a[i,n/2].imag

        for j in range(1,(n+1)/2):
            if i==1:
                res[0,j]=a[0,j].real
                res[0,n-j]=a[0,j].imag
                if m%2==0:
                    res[m/2,j]=a[m/2,j].real
                    res[m/2,n-j]=a[m/2,j].imag

            res[i,j]=(a[i,j]+a[i,n-j]).real/2
            res[m-i,n-j]=(a[i,n-j]-a[i,j]).real/2
            res[i,n-j]=(a[i,j]-a[i,n-j]).imag/2
            res[m-i,j]=(a[i,j]+a[i,n-j]).imag/2
    res[0,0]=a[0,0].real
    if n%2==0:
        res[0,n/2]=a[0,n/2].real
    else:
        res[0,n/2]=a[0,n/2].real
        res[0,n/2+1]=a[0,n/2].imag
        
    if m%2==0:
        res[m/2,0]=a[m/2,0].real
        if n%2==0:
            res[m/2,n/2]=a[m/2,n/2].real
    else:
        res[m/2,0]=a[m/2,0].real
        res[m/2+1,0]=a[m/2,0].imag
    return res

def correlate(a,b,pad=None):
    if pad!=None:
        m=a.shape[0]
        n=a.shape[1]
        if type(pad)==type(0):
            pad=pad,pad
        pads=2*pad[0]+m,2*pad[1]+n
        aa=numpy.zeros(pads,numpy.float32)
        aa[pad[0]:pad[0]+m,pad[1]:pad[1]+n]=a
        bb=numpy.zeros(pads,numpy.float32)
        bb[pad[0]:pad[0]+m,pad[1]:pad[1]+n]=b
        b=bb
        a=aa
    corr=numpy.fft.ifft2(numpy.fft.fft2(a)*numpy.conjugate(numpy.fft.fft2(numpy.fft.fftshift(b)))).real
    return corr

def testcorrelate(a,b,pad=None):
    """correlate a with b.  pad can be a zero padding size, either single value or y,x.
    pad is the number of zerod rows/cols to add to each size of the images.
    """
    #first do it the numpy way...
    m=a.shape[0]
    n=a.shape[1]
    if pad!=None:
        if type(pad)==type(0):
            pad=pad,pad
        pad=2*pad[0]+m,2*pad[1]+n
        aa=numpy.zeros(pad,numpy.float32)
        aa[(pad[0]-m)/2:(pad[0]-m)/2+m,(pad[1]-n)/2:(pad[0]-n)/2+n]=a
        bb=numpy.zeros(pad,numpy.float32)
        bb[(pad[0]-m)/2:(pad[0]-m)/2+m,(pad[1]-n)/2:(pad[0]-n)/2+n]=b
        b=bb
        a=aa
    m=a.shape[0]
    n=a.shape[1]
    corr=numpy.fft.ifft2(numpy.fft.fft2(a)*numpy.conjugate(numpy.fft.fft2(numpy.fft.fftshift(b)))).real
    #and now do it using half complex format.
    aa2=r2hc(numpy.fft.fftshift(b))
    #do the conjugate.
    mm=aa2.shape[0]/2+1
    nn=aa2.shape[1]/2+1
    aa2[:mm,nn:]*=-1
    aa2[mm:,:nn]*=-1

    aa1=r2hc(a)
    aa3=aa1*aa2
    #n=aa2.shape[0]
    #now multiply the two together
    res=numpy.zeros(a.shape,numpy.float32)
    res[0,0]=aa1[0,0]*aa2[0,0]
    if n%2==0:
        res[0,n/2]=aa1[0,n/2]*aa2[0,n/2]
    if m%2==0:
        res[m/2,0]=aa1[m/2,0]*aa2[m/2,0]
        if n%2==0:
            res[m/2,n/2]=aa1[m/2,n/2]*aa2[m/2,n/2]
    else:
        pass
        
    for i in range(1,(m+1)/2):
        #a10i=aa1[0,i]+1j*aa1[0,n-i]
        #a20i=aa2[0,i]+1j*aa2[0,n-i]
        #a30i=a10i*a20i
        #res[0,i]=a30i.real
        #res[0,n-i]=a30i.imag

        #a1i0=aa1[i,0]+1j*aa1[n-i,0]
        #a2i0=aa2[i,0]+1j*aa2[n-i,0]
        #a3i0=a1i0*a2i0
        #res[i,0]=a3i0.real
        #res[n-i,0]=a3i0.imag
        res[i,0]=aa1[i,0]*aa2[i,0]-aa1[m-i,0]*aa2[m-i,0]
        res[m-i,0]=aa1[i,0]*aa2[m-i,0]+aa1[m-i,0]*aa2[i,0]

        if n%2==0:
            #a1in=aa1[i,n/2]+1j*aa1[n-i,n/2]
            #a2in=aa2[i,n/2]+1j*aa2[n-i,n/2]
            #a3in=a1in*a2in
            #res[i,n/2]=a3in.real
            #res[n-i,n/2]=a3in.imag
            res[i,n/2]=aa1[i,n/2]*aa2[i,n/2]-aa1[m-i,n/2]*aa2[m-i,n/2]
            res[m-i,n/2]=aa1[i,n/2]*aa2[m-i,n/2]+aa1[m-i,n/2]*aa2[i,n/2]


        for j in range(1,(n+1)/2):
            if i==1:
                res[0,j]=aa1[0,j]*aa2[0,j]-aa1[0,n-j]*aa2[0,n-j]
                res[0,n-j]=aa1[0,j]*aa2[0,n-j]+aa1[0,n-j]*aa2[0,j]
                if m%2==0:
                    #a1ni=aa1[n/2,i]+1j*aa1[n/2,n-i]
                    #a2ni=aa2[n/2,i]+1j*aa2[n/2,n-i]
                    #a3ni=a1ni*a2ni
                    #res[n/2,i]=a3ni.real
                    #res[n/2,n-i]=a3ni.imag
                    res[m/2,j]=aa1[m/2,j]*aa2[m/2,j]-aa1[m/2,n-j]*aa2[m/2,n-j]
                    res[m/2,n-j]=aa1[m/2,j]*aa2[m/2,n-j]+aa1[m/2,n-j]*aa2[m/2,j]



            #first unpack hc
            #a1ij=aa1[i,j]-aa1[n-i,n-j]+1j*(aa1[i,n-j]+aa1[n-i,j])
            #a2ij=aa2[i,j]-aa2[n-i,n-j]+1j*(aa2[i,n-j]+aa2[n-i,j])

            #a1inmj=aa1[i,j]+aa1[n-i,n-j]+1j*(aa1[n-i,j]-aa1[i,n-j])
            #a2inmj=aa2[i,j]+aa2[n-i,n-j]+1j*(aa2[n-i,j]-aa2[i,n-j])
            #now multiply these unpacked values...
            #a3ij=a1ij*a2ij
            #a3inmj=a1inmj*a2inmj
            #and now repack the result.
            #res[i,j]=(a3ij+a3inmj).real/2
            #res[n-i,n-j]=(a3inmj-a3ij).real/2

            #res[i,n-j]=(a3ij-a3inmj).imag/2
            #res[n-i,j]=(a3ij+a3inmj).imag/2

            res[i,j]=aa3[i,j]+aa3[m-i,n-j]-aa3[i,n-j]-aa3[m-i,j]
            res[m-i,n-j]=aa1[i,j]*aa2[m-i,n-j]+aa1[m-i,n-j]*aa2[i,j]+aa1[m-i,j]*aa2[i,n-j]+aa1[i,n-j]*aa2[m-i,j]

            res[i,n-j]=aa1[i,j]*aa2[i,n-j]-aa1[m-i,n-j]*aa2[m-i,j]+aa1[i,n-j]*aa2[i,j]-aa1[m-i,j]*aa2[m-i,n-j]
            res[m-i,j]=aa1[i,j]*aa2[m-i,j]-aa1[m-i,n-j]*aa2[i,n-j]+aa1[m-i,j]*aa2[i,j]-aa1[i,n-j]*aa2[m-i,n-j]
            
    #and then hc2r.
    res=hc2r(res)
    #corr=r2hc(corr)
    return corr,res


## if __name__=="__main__":
##     import sys
##     n=16
##     m=16
##     if len(sys.argv)==3:
##         m=int(sys.argv[1])
##         n=int(sys.argv[2])
##     a=numpy.random.random((m,n)).astype("f")
##     a[3:5,4:6]=20.
##     b=numpy.zeros((m,n),"f")
##     b[m/2-1:m/2+1,n/2-1:n/2+1]=1.
##     #first as a test, write a.
##     open("sh.dat","w").write(a.tostring())
##     tmp=r2hc(a)

##     #corr=numpy.conjugate(numpy.fft.fft2(numpy.fft.fftshift(b)))
##     corr=r2hc(numpy.fft.fftshift(b))#how do I do the conjugate?
##     #Do the conjugation...
##     mm=corr.shape[0]/2+1
##     nn=corr.shape[1]/2+1
##     corr[:mm,nn:]*=-1
##     corr[mm:,:nn]*=-1
##     #Note, this is the same as doing:
##     #corr=r2hc(numpy.fft.ifft2(numpy.conjugate(numpy.fft.fft2(numpy.fft.fftshift(b)))).real)
##     open("corr.dat","w").write(corr.tostring())

##     #Now, do the convolution in python (for comparisons...).
##     c=correlate(a,b)

##     os.system("./corrfftw %d %d"%(m,n))

##     txt=open("ffta.dat").read()
##     ffta=numpy.fromstring(txt,numpy.float32)
##     ffta.shape=m,n

##     #This shows that r2hc works.... (ie my python version gives same as fftw)
##     print "checking r2hc",min(numpy.fabs(ffta.ravel())),max(numpy.fabs(ffta.ravel())),min(numpy.fabs(ffta-tmp).ravel()),max(numpy.fabs(ffta-tmp).ravel())

##     txt=open("fftab.dat").read()
##     fftab=numpy.fromstring(txt,numpy.float32)
##     fftab.shape=m,n

##     pyres=numpy.fft.ifft2(numpy.fft.fft2(a)*numpy.conjugate(numpy.fft.fft2(numpy.fft.fftshift(b)))).real

##     pyfftab=r2hc(pyres)
##     print "fftab diff",min(numpy.fabs(fftab.ravel())),max(numpy.fabs(fftab.ravel())),min(numpy.fabs(fftab-pyfftab).ravel()),max(numpy.fabs(fftab-pyfftab).ravel())
    

##     #and check the result...
##     pyres*=m*n

##     txt=open("res.dat").read()
##     res=numpy.fromstring(txt,numpy.float32)
##     res.shape=m,n

##     print "result diff",min(numpy.fabs(res.ravel())),max(numpy.fabs(res.ravel())),min(numpy.fabs(res-pyres).ravel()),max(numpy.fabs(res-pyres).ravel())

##     #Now compare res with pyres
##     #Fiddle round with creating corr until you get them matching...
