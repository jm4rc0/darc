import numpy
def applyBrightest(img,ub,subapLocation):
    subapLocation.shape=subapLocation.size//6,6
    if type(ub)==type(0):
        ub=numpy.ones(subapLocation.shape[0],numpy.int32)*ub
    for i in range(subapLocation.shape[0]):
        s=subapLocation[i]
        if s[2]!=0 and s[5]!=0:
            im=img[s[0]:s[1]:s[2],s[3]:s[4]:s[5]]
            tmp=im.copy().ravel()
            tmp.sort()
            if ub[i]!=0:
                j=tmp.size-numpy.abs(ub[i])
                thr=tmp[j]
                if ub[i]<0:
                    thr2=thr
                    while thr2==thr and j>1:
                        j-=1
                        thr2=tmp[j]
                    im[:]=numpy.where(im<thr,0,im-thr2)
                else:
                    im[:]=numpy.where(im<thr,0,im)
    return img

def makeSubapLocation(nsubx,nsuby,xoff,yoff,xstep,ystep):
    sl=numpy.zeros((nsuby,nsubx,6),numpy.int32)
    for i in range(nsuby):
        for j in range(nsubx):
            sl[i,j]=(yoff+i*ystep,yoff+(i+1)*ystep,1,xoff+j*xstep,xoff+(j+1)*xstep,1)
    sl.shape=sl.size//6,6
    return sl

def makeSlopes(img,ncam,nsub,npxlx,npxly,sf,sl):
    """Need to subtract 0.5 from the results if a correlation image."""
    sl.shape=sl.size//6,6
    slopes=numpy.zeros((sf.sum()*2),numpy.float32)
    soff=0
    start=0
    pos=0
    npxl=numpy.array(npxlx)*numpy.array(npxly)
    for i in range(ncam):
        thisimg=img[start:start+npxl[i]]
        thisimg.shape=npxly[i],npxlx[i]
        for j in range(nsub[i]):
            if sf[soff+j]:
                s=sl[soff+j]
                im=thisimg[s[0]:s[1]:s[2],s[3]:s[4]:s[5]]
                tot=im.sum()
                if tot>0:
                    x=(im.sum(0)*numpy.arange(im.shape[1])).sum()
                    y=(im.sum(1)*numpy.arange(im.shape[0])).sum()
                    x/=tot
                    y/=tot
                else:
                    x=0
                    y=0
                slopes[pos*2]=x-im.shape[1]//2+0.5
                slopes[pos*2+1]=y-im.shape[0]//2+0.5
                pos+=1
        soff+=nsub[i]
        start+=npxl[i]
    return slopes
