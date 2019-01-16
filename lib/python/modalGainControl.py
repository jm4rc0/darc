"""Computes modal gain control for a control matrix."""



def computeModalGainMx(gain,modalGainList,slopeToZernMx,rmx=None):
    """Computes:
    gainmx=(gI + z.T(g_m - gI)z)
    where gainmx has size nslopes,nslopes.
    z has size nmodes,nslopes
    g_m is the modalGainList of size nmodes (converted here to a diag matrix)
    gain is the overall system gain.

    If len(modalGainList)==0, this is equivalent to no modal gain control, 
    and will therefore just equal the gain.

    If rmx specified, then computes rmx=numpy.dot(rmx,gainmx)
    """
    modalGainList=numpy.array(modalGainList)
    nmodes=len(modalGainList)
    s2z=slopeToZernMx[:nmodes]
    nslopes=slopeToZernMx.shape[1]
    gI=-numpy.identity(nmodes)*gain
    gI.ravel()[::nmodes+1]+=modalGainList
    gainmx=gain*numpy.identity(nslopes) + numpy.dot(s2z.T,numpy.dot(gI,s2z))
    if rmx is not None:
        transpose=0
        if rmx.shape[1]!=gainmx.shape[0]:
            transpose=1
            rmx=rmx.T
        gainmx=numpy.dot(rmx,gainmx)
        if transpose:
            gainmx=gainmx.T
    return gainmx


if __name__=="__main__":
    import sys
    import FITS
    if len(sys.argv)<5:
        print("Usage: %s globalGain [modal,gain,list] output.fits slopeToZernMx.fits optionalRmx"%sys.argv[0])
        sys.exit(0)
    gain=sys.argv[1]
    modalGainList=eval(sys.argv[2])
    outMx=sys.argv[3]
    slopeToZernMx=FITS.Read(sys.argv[4])[1]
    if slopeToZernMx[0].std()==0:#first row all zeros - probably piston
        print("Stripping piston from %s"%sys.argv[4])
        slopeToZernMx=slopeToZernMx[1:]
    rmx=None
    if len(sys.argv)>5:
        rmx=FITS.Read(sys.argv[5])[1]
    gmx=computeModalGainMx(gain,modalGainList,slopeToZernMx,rmx)
    
    FITS.Write(gmx,outMx)
