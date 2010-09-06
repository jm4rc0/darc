import numpy
def createCentroidOverlay(cen,subapLocation,subflag,oversample=2,pattern=None,out=None,zeroOut=1,npxlx=None,npxly=None):
    """Create an array for putting centroid overlay on pixels.
    Note, streamDict has to have a key equal to Pxl or Cal which should be the image from the camera in question, shaped as a 2D array.
    It should also contain an array with key equal to "Cen", the centroids from this camera, and one with key equal "Sub", the subap locations for this camera."""
#     if npxlx==None or npxly==None:
#         if streamDict.has_key("Pxl"):
#             data=streamDict["Pxl"]
#         elif streamDict.has_key("Cal"):
#             data=streamDict["Cal"]
#         elif streamDict.has_key("rtcCalPxlBuf"):
#             data=streamDict["rtcCalPxlBuf"]
#         elif streamDict.has_key("rtcPxlBuf"):
#             data=streamDict["rtcPxlBuf"]
#         else:
#             print "ERROR - no raw or calibrated pixels found"
#             return None
#         npxly=numpy.array([data.shape[0]])
#         npxlx=numpy.array([data.shape[1]])
#         if cam!=0:
#             print "Error - no npxly/x array input, and cam>0"
#             return None
#     if streamDict.has_key("Cen"):
#         cen=streamDict["Cen"]
#     elif streamDict.has_key("rtcCentBuf"):
#         cen=streamDict["rtcCentBuf"]
#     else:
#         print "Error - centroids required"
#         return None
    #if not streamDict.has_key("Sub"):
    #    print "Error - subap locations required"
    #    return None
    #else:
    #    sub=streamDict["Sub"]
    subflag=subflag.ravel()
    if pattern==None:
        pattern=numpy.zeros((oversample,oversample,4),numpy.float32)
        pattern[:,:,1::2]=1
    ysize,xsize=pattern.shape[:2]
    #nsub=nsubx*nsuby
    sub=subapLocation#[nsub[:cam].sum():nsub[:cam+1].sum()]
    if len(sub.shape)==1:
        sub.shape=sub.shape[0]/6,6
    #sub=sub[nsub[:cam].sum():nsub[:cam+1].sum()]
    #cen=cen[nsub[:cam].sum()*2:nsub[:cam+1].sum()*2]
    centx=cen[::2]
    centy=cen[1::2]
    if out==None or out.shape!=(npxly*oversample,npxlx*oversample,4):
        print "Creating"
        overlay=numpy.zeros((npxly*oversample,npxlx*oversample,4),numpy.float32)
    else:
        overlay=out
        if zeroOut:
            overlay[:]=0
    pos=0
    for i in range(sub.shape[0]):
        sl=sub[i]*oversample
        sl[:3]/=sub[i,2]
        sl[3:6]/=sub[i,5]
        #if sl[2]!=0:#subap used...
        if subflag[i]:#subap used
            #find the centre of this subap, offset this by the x and y centroids, round to nearest int and this is the centroid location
            sl5=sl[5]
            sl2=sl[2]
            curnx=(sl[4]-sl[3])/sl5#nunber of used pixels in this subap
            curny=(sl[1]-sl[0])/sl2
            cx=curnx/2.-0.5
            cy=curny/2.-0.5
            cx+=centx[pos]
            cy+=centy[pos]
            cx=round(cx*sl[5])+sl[3]#/sl[5]
            cy=round(cy*sl[2])+sl[0]#/sl[2]
            overlay[cy-ysize/2:cy+(ysize+1)/2,cx-xsize/2:cx+(xsize+1)/2]=pattern
            pos+=1
    return overlay

def createSubapOverlay(sub,subflag,oversample=2,out=None,zeroOut=1,npxlx=None,npxly=None):
    """Create an array for putting a subap box overlay on pixels.
    The image and subap location must be specific for this camera.
    """
#     if npxlx==None or npxly==None:
#         if streamDict.has_key("Pxl"):
#             data=streamDict["Pxl"]
#         elif streamDict.has_key("Cal"):
#             data=streamDict["Cal"]
#         elif streamDict.has_key("rtcCalPxlBuf"):
#             data=streamDict["rtcCalPxlBuf"]
#         elif streamDict.has_key("rtcPxlBuf"):
#             data=streamDict["rtcPxlBuf"]
#         else:
#             print "ERROR - no raw or calibrated pixels found"
#             return None
#         npxly=numpy.array([data.shape[0]])
#         npxlx=numpy.array([data.shape[1]])
#         if cam!=0:
#             print "Error - no npxly/x array input, and cam>0"
#             return None

#     if streamDict.has_key("Sub"):
#         sub=streamDict["Sub"]
#     elif streamDict.has_key("rtcSubLocBuf"):
#         sub=streamDict["rtcSubLocBuf"]
#     else:
#         print "Error - subap locations required"
#         return None
    subflag=subflag.ravel()
    if len(sub.shape)==1:
        sub.shape=sub.shape[0]/6,6
    #nsub=nsubx*nsuby
    #sub=sub[nsub[:cam].sum():nsub[:cam+1].sum()]

    if out==None or out.shape!=(npxly*oversample,npxlx*oversample,4):
        overlay=numpy.zeros((npxly*oversample,npxlx*oversample,4),numpy.float32)
        print "creat"
    else:
        overlay=out
        if zeroOut:
            overlay[:]=0

    pos=0
    for i in range(sub.shape[0]):
        sl=sub[i]*oversample
        sl[:3]/=sub[i,2]
        sl[3:6]/=sub[i,5]
        #if sl[2]!=0:#subap is used
        if subflag[i]:
            overlay[sl[0]:sl[1]:sl[2],sl[3],2:]=1
            overlay[sl[0]:sl[1]:sl[2],sl[4]-1,2:]=1
            overlay[sl[0],sl[3]:sl[4]:sl[5],2:]=1
            overlay[sl[1]-1,sl[3]:sl[4]:sl[5],2:]=1
            #overlay[sl[0]:sl[1]:sl[2],sl[3],2:]=1
            #overlay[sl[0]:sl[1]:sl[2],sl[4]-1,2:]=1
            #overlay[sl[0],sl[3]:sl[4]:sl[5],2:]=1
            #overlay[sl[1]-1,sl[3]:sl[4]:sl[5],2:]=1
    return overlay

def createSubapCentOverlay(cen,sub,subapLocation,subflag,oversample=2,pattern=None,out=None,npxlx=None,npxly=None):
    """Create both subap and position overlays..."""
    o=createCentroidOverlay(cen,subapLocation,subflag,oversample=oversample,pattern=pattern,out=out,npxlx=npxlx,npxly=npxly)
    s=createSubapOverlay(sub,subflag,oversample,out,zeroOut=0,npxlx=npxlx,npxly=npxly)
    o[:,:,2]=numpy.where(s[:,:,2]!=0,s[:,:,2],o[:,:,2])
    o[:,:,3]+=s[:,:,3]
    o[:,:,3]=numpy.where(o[:,:,3]>1,1,o[:,:,3])
    return o

def makePattern(pattern,oversample=2):
    if pattern=="cross":
        pattern=numpy.zeros((oversample,oversample,4),numpy.float32)
        pattern[oversample/2:oversample/2+1,:,1::2]=1
        pattern[:,oversample/2:oversample/2+1,1::2]=1
    elif pattern=="egg":
        import tel
        pattern=numpy.zeros((oversample,oversample,4),numpy.float32)
        pattern[:,:,0]=tel.Pupil(oversample,oversample/2,0).fn
        pattern[:,:,1]=pattern[:,:,0]
        pattern[:,:,2]=tel.Pupil(oversample,oversample/4,0).fn
        #pattern[oversample*.25:oversample*.75,oversample*.25:oversample*.75,:3]=1
        pattern[:,:,3]=0.4
        
    elif pattern=="target":
        import tel
        pattern=numpy.zeros((oversample,oversample,4),numpy.float32)
        pattern[:,:,1]=1
        pattern[:,:,3]=tel.Pupil(oversample,oversample/2,oversample/2-1).fn
    else:
        pattern=numpy.zeros((oversample,oversample,4),numpy.float32)
        pattern[:,:,1::2]=1
    return pattern
def createCentroidArrows(cen,subapLocation,subflag,scale=1,npxlx=None,npxly=None,nsubx=None,nsuby=None,cam=0):
#     if streamDict.has_key("Cen"):
#         cen=streamDict["Cen"]
#     elif streamDict.has_key("rtcCentBuf"):
#         cen=streamDict["rtcCentBuf"]
#     else:
#         print "Error - centroids required"
#         return None
    subflag=subflag.ravel()
    sub=subapLocation
    if len(sub.shape)==1:
        sub.shape=sub.shape[0]/6,6

    #nsub=nsubx*nsuby
    #sub=sub[nsub[:cam].sum():nsub[:cam+1].sum()]
    #cen=cen[nsub[:cam].sum()*2:nsub[:cam+1].sum()*2]
    centx=cen[::2]
    centy=cen[1::2]
    cpos=0
    arrows=[]
    for i in range(sub.shape[0]):
        sl=sub[i]
        if subflag[i]!=0:#subap used...
            #find the centre of this subap, offset this by the x and y centroids, round to nearest int and this is the centroid location
            sl5=1
            sl2=1
            curnx=(sl[4]-sl[3])/sl[5]#nunber of used pixels in this subap
            curny=(sl[1]-sl[0])/sl[2]
            cx=curnx/2.#-0.5
            cy=curny/2.#-0.5
            headlen=2.
            hl=scale*numpy.sqrt(centx[cpos]*centx[cpos]+centy[cpos]*centy[cpos])
            if hl<headlen:
                headlen=hl/2.
            if hl!=0:
                arrows.append([cx+sl[3]/sl[5],cy+sl[0]/sl[2],centx[cpos]*scale,centy[cpos]*scale,{"ec":"red","fc":"red","head_width":headlen,"head_length":headlen,"length_includes_head":True}])
            cpos+=1
    return arrows

    
def createZernikeOverlay(actmap,pupfn=None,nmodes=None):
    #pupfn is same size as actmap
    import zernike
    if pupfn==None:
        import tel
        pupfn=tel.Pupil(actmap.shape[0],actmap.shape[0]/2,0).fn#(actmap!=0).astype(numpy.int32)
    if nmodes==None:
        nmodes=int((actmap.shape[0]+1)*(actmap.shape[0]+2)/2.)
    z=zernike.Zernike(pupfn,nmodes)
    coeff=z.calcZernikeCoeff(actmap)
    data=z.zernikeReconstruct(coeff,size=64)
    return data
def createLinearOverlay(acts,out):
    import pylab
    x=numpy.arange(acts.shape[1])/float(acts.shape[1]-1)
    xx=numpy.arange(out.shape[1])/float(out.shape[1]-1)
    for i in range(acts.shape[0]):
        out[i]=pylab.interp(xx,x,acts[i])
    for i in range(out.shape[1]):
        out[:,i]=pylab.interp(xx,x,out[:acts.shape[0],i])
    return out
def createSplineOverlay(acts,out):
    import scipy.interpolate
    x=numpy.arange(acts.shape[1])/float(acts.shape[1]-1)
    xx=numpy.arange(out.shape[1])/float(out.shape[1]-1)
    for i in range(acts.shape[0]):
        out[i]=scipy.interpolate.interp1d(x,acts[i],3)(xx)
    for i in range(out.shape[1]):
        out[:,i]=scipy.interpolate.interp1d(x,out[:acts.shape[0],i],3)(xx)
    return out
