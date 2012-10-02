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
                j=tmp.size-numpy.abs(ub)
                thr=tmp[j]
                if ub[i]<0:
                    thr2=thr
                    while thr2==thr and j>0:
                        j-=1
                        thr2=tmp[j]
                    im[:]=numpy.where(im<thr,0,im-thr2)
                else:
                    im[:]=numpy.where(im<thr,0,im)
    return img
