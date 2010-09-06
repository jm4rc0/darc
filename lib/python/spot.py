import numpy
import zernike
def expandPoints(data,typ="circ",diam=9,sh=None):
    """Expand every point in data by typ which can be circ or cross, of diameter equal to diam.
    Alternatively, supply sh, of shape x,x,4 where x is odd.
    Other such functions could easily be written, eg different shapes, change colour, multi-coloured etc.
    """
    out=data.copy()
    if sh==None:
        sh=numpy.zeros((diam,diam,4),numpy.float32)
        if typ=="circ":
            sh[:,:,1]=zernike.Pupil(diam,diam/2.,0).fn
        elif typ=="cross":
            sh[diam/2,:,1]=1
            sh[:,diam/2,1]=1
        sh[:,:,3]=sh[:,:,1]
    else:
        diam=sh.shape[0]
        if sh.shape[1]!=diam or sh.shape[2]!=4:
            print "Error - sh must be shape x,x,4."
    for y in range(data.shape[0]):
        for x in range(data.shape[1]):
            if data[y,x,1]:
                out[y-diam/2:y+(diam+1)/2,x-diam/2:x+(diam+1)/2]=sh
    return out
