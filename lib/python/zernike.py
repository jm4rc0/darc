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
def makeZernike(acts,nzern,mask):
    """Function (called from GUI in the mangle part) which can do a zernike decomposition of actuators.
    acts is a 1D array of the actuators.
    nzern is the number of zernikes to decompose (or a list of), so 3 or [0,1,2] would do piston, tip, tilt, while [3] would do focus only.
    mask defines the location of these actautors - a mask of 1s and 0s.
    """
    if type(nzern)==type(0):
        nzern=range(nzern)
    data=numpy.zeros(mask.shape,numpy.float32).ravel()
    m=mask.ravel()
    pos=0
    for i in range(m.shape[0]):
        if m[i]:
            data[i]=acts[pos]
            pos+=1
    data.shape=mask.shape
    z=Zernike(mask,nzern)
    coeff=z.calcZernikeCoeff(data)
    return coeff
def makeCircularGrid(nxpup,nypup=None,natype=numpy.float64,dosqrt=1):
    """
    returns a npup*npup numpy array with a grid of distance of pixels
    from the center of the screen

    Distance definition not suitable with FFT !!!

    default return type : Float64
    """
    if nypup==None:
        nypup=nxpup
    tabx=numpy.arange(nxpup)-float(nxpup/2.)+0.5 ##RWW's convention
    taby=numpy.arange(nypup)-float(nypup/2.)+0.5 ##RWW's convention
    grid=tabx[numpy.newaxis,:,]**2+taby[:,numpy.newaxis]**2
    if dosqrt:
        numpy.sqrt(grid,grid)
    return grid.astype(natype)

def makeThetaGrid(npup,natype=numpy.float32):
    """
    makeThetaGrid(dpix,natype)
    returns a dpix*dpix Numarray array with a grid of angles (in radians)
    from the center of the screen

    default return type : Float32
    """
    tabx=numpy.arange(npup)-float(npup/2.)+0.5 ##RWW's convention
    grid=numpy.arctan2(tabx[:,numpy.newaxis],tabx[numpy.newaxis,:,])
    return grid.astype(natype)

def nm(j,sign=0):
    """
    returns the [n,m] list giving the radial order n and azimutal order
    of the zernike polynomial of index j
    if sign is set, will also return a 1 for cos, -1 for sine or 0 when m==0.
    """
    n = int((-1.+numpy.sqrt(8*(j-1)+1))/2.)
    p = (j-(n*(n+1))/2.)
    k = n%2
    m = int((p+k)/2.)*2 - k
    if sign==0:
        return [n,m]
    else:#determine whether is sine or cos term.
        if m!=0:
            if j%2==0:
                s=1
            else:
                s=-1
            #nn,mm=nm(j-1)
            #if nn==n and mm==m:
            #    s=-1
            #else:
            #    s=1
        else:
            s=0
        return [n,m,s]


def fastZerCoeffs(n,m,natype=numpy.float64):
    """
    returns the array of the Knm(s) coefficients for the definition of
    Zernike polynomials (from Noll-1976)

    We use the following properties to improve computation speed :
    - K_mn(0) = n! / ((n+m)/2)! / ((n-m)/2)!
    - Knm(s+1) =  -Knm(s) * ((n+m)/2-s)*((n-m)/2-s)/(s+1)/(n-s) 
    """
    #result of the function
    result=numpy.zeros(n+1,natype)
    
    ## We first compute Knm(0), giving the coefficient of the highest polynomial
    st =  2 ## start index for dividing by ((n-m)/2)!
    coef = 1.00
    #print "fastZerCpeffs",int((n+m)/2.+1.5),n+1
    for i in range(int((n+m)/2.+1.5),n+1):
        ## we compute n! / ((n+m)/2)!
        if (st<=int(((n-m)/2.)+0.5) and (i%st==0)):
            j = i/float(st)
            st+=1
            coef *= j
        else:
            coef *= i

    ## We then divide by ((n-m)/2) ! (has already been partially done)
    #print "factorial from",st,int((n-m)/2.+1.5)
    for i in range(st,int((n-m)/2.+1.5)):
        coef /= float(i)
    
    ##We fill the array of coefficients
    result[n] = numpy.floor( coef + 0.5)  ## for K_nm(s=0), ie n=m

    ##We use the recurrent relation shown in the function documentation
    for i in range(1,int((n-m)/2.+1.5)):
      coef *= -((n+m)/2.-i+1)*((n-m)/2.-i+1)
      coef /= float(i)
      coef /= float((n-i+1))
      result[n-2*i] = numpy.floor( coef + 0.5 )

    return result
def Rnm(n,m,a,r,dosquare=1):
    """
    Returns the radial part of the Zernike polynomials of the Zernike
    polynomial with radial order n and azimutal order m

    n,m : radial and azimutal order
    a : array of Knm(s) coefficients given by fastZerCoeffs
    r : radial coordinate in the pupil squared (from util.tel.makeCircularGrid(dosqrt=0))

    Use of informations in section 5.3 of Numerical Recipes to accelerate
    the computataion of polynomials
    """
    if dosquare:
        if n>1 :
            r2 = r*r
    else:#r already squared (on large pupils, sqrting and then squaring again can lead to errors)
        if n>1 :
            r2=r
        r=numpy.sqrt(r)


    p=a[n]
    for i in range(n-2,m-1,-2):
        p=p*r2+a[i]#I think this is the part that causes numerical precision errors for large pupils...
  
    if m==0:
        return p
    elif m==1:
        p*=r
    elif m==2:
        p*=r2
    else:#This can be a problem in terms of numerical precision?  Probably not...
        p*=(r2**(m/2.))
  
    return p


class Zernike:
    """
    Class Zernike : new class to define Zernike Polynomials
    Assumes full circular apertures

    Fields :
    - jmax : index of the maximum zernike polynomial stored in the zern array, or a list of indexes to be computed.
    - npup : number of pixels of the zern array
    - zern : Numeric (Float32 or 64 or 128) array storing the Zernike Polynomials maps
             shape=(jmax,npup,npup) or (len(jmax),npup,npup)
    - invGeoCovMat : inverse of the geometric covariance matrix (used to give the
                     real expansion of the input phase)
    - pupfn : Ubyte array storing the circular binary array defining pupil function
    - puparea : area in PIXELS of the pupil function
    - idxPup : 1D array with the index of the pixels belonging to the pupil
    - natype : Numeric Data type of the zern array (Float32/64)
    - zern2D : Numeric Float32/64 array storing the zern cube
    as a 2D array (each 2D ZP is converted as a 1D array), with only the values belonging to the pupil
    Note, this module is approx 10x slower than cmod.zernike for creation of a single zernike on a large pupil when using float64.
    This module is 3.5x slower with float32 than with float64.
    Note, there are floating point precision issues even with float128.  For eg pupil=600, jmax=2016, this appears fuzzy if using float64.  Okay for float128, but then numpy.sum(zern[]*zern[]) should equal pup.sum, but doesn't.  Just be warned that high order zernikes may not be correct!
    """
    def __init__(self,pupil,jmax):
        """ Constructor for the Zernike class

        Parameters :
        - pupil : Pupil object storing pupil geometry
        - jmax : maximum index of the Zernike polynomial to store into the zern array, either a int, or a list of ints to be generated.
        - natype : output array data type (by default Float32):
        """
        
        natype=numpy.float32
        self.pupfn=pupil#numpy.array(pupil.copy())
        self.npup=self.pupfn.shape[0] ##number of pixels for one zernike polynomial
        if natype==numpy.float32 and self.npup>100:
            print "WARNING: zernikeMod - using float32 for large pupils can result in error."

        self.natype=natype ##type of output array
        
        if type(jmax)==type(1):
            jmax=range(jmax)
        self.zernExpand=None
        #jmax.sort()
        #print "Creation of Zernike Polynomials"
        self.createZernikeCube(jmax)
        self.zern2D=self.zern.view()
        self.zern2D.shape=(len(jmax),self.npup*self.npup)
        
    def createZernikeCube(self,jmax):
        """Fills the zern array (cube of Zernike Polynomials)
        jmax : maximum index of the Zernike Polynomial to store into zern
        """
        ## grid of distance to center normalised to 1,
        ## because ZP are defined on unit radius pupils
        rGrid=makeCircularGrid(self.npup,natype=self.natype,dosqrt=0)/self.npup**2*4
        #rGrid=rGrid.astype(self.natype) ##to keep the same datatype

        ##1D-index of pixels belonging to the pupil
        self.idxPup=idxPup=numpy.nonzero(self.pupfn.ravel())
        self.puparea=len(idxPup)

        ## grid of angles
        thetaGrid=makeThetaGrid(self.npup,natype=self.natype) ##grid of angles

        ##modification of jmax
        self.jmax=jmax
        ##we extract only the pixels corresponding to the pupil, to improve computation time
        try:
            rGridFlat=numpy.take(rGrid.ravel(),idxPup)
        except:
            rGridFlat=numpy.take(rGrid.flat,idxPup)
        thetaGridFlat=numpy.take(thetaGrid.ravel(),idxPup)

        ##we allocate the cube of Zernike Polynomials
        retShape=(len(jmax),self.npup,self.npup)
        self.zern=numpy.zeros(retShape,self.natype)

        ##we look for the radial order corresponding to jmax
        nmax=nm(max(jmax)+1)[0]
        ##print "nmax=%d" % nmax

        ##we put the piston
        if 0 in jmax:#piston required.
            self.zern[jmax.index(0),:,:,]=self.pupfn.copy().astype(self.natype)
        
        ##loop to fill the cube of zernike polynomials
        j=2

        ##we go through the radial orders 1 to nmax-1
        for n in range(1,nmax):
            ##computation of the starting azimutal order
            ## print "n=%d" % n
            if n%2:##n is odd : m starts at 1
                mstart=1
            else: ##n is even : mstart at 0
                mstart=0
            m=mstart      
            while (m<=n):
                ##we compute the radial part of the polynomial
                #Used to call Rnm here...
                ##print "m=%d" % m
                ##we make the other computations
                if m==0: ##azimutal order = 0 => no trigonometric part
                    ##we add one ZP when m=0
                    #print "j=%d" % j
                    if j-1 in jmax:
                        a=fastZerCoeffs(n,m,self.natype)
                        Z=Rnm(n,m,a,rGridFlat,dosquare=0)*numpy.array(numpy.sqrt(n+1))
                        #print j,n,m,a
                        numpy.put(self.zern[jmax.index(j-1),:,:,].ravel(),idxPup,Z)
                    j+=1
                else: ##azimutal order >0 : there is a trigonometric part
                    if ((j-1) in jmax) or (j in jmax):#generate the data...
                        a=fastZerCoeffs(n,m,self.natype)
                        Z=Rnm(n,m,a,rGridFlat,dosquare=0)*numpy.array(numpy.sqrt(n+1))
                        Z*=numpy.array(numpy.sqrt(2.))
                    ##we add the two zernike polynomials per azimutal order
                    for cnt in range(2):               
                        if j%2: ## j is odd : multiply by sin(m*theta)
                            #print "j=%d" % j
                            if j-1 in jmax:
                                #print j,n,m,a
                                numpy.put(self.zern[jmax.index(j-1),:,:,].ravel(),idxPup,Z*numpy.sin(m*thetaGridFlat))
                        else: ## j is even : multiply by cos(m*theta)
                            #print "j=%d" % j
                            if j-1 in jmax:
                                #print j,n,m,a
                                numpy.put(self.zern[jmax.index(j-1),:,:,].ravel(),idxPup,Z*numpy.cos(m*thetaGridFlat))
                        j+=1
                
                ##we increase the azimuthal order
                m+=2

        ##we do the last radial order
        n=nmax
        ##computation of the starting azimutal order
        ## print "n=%d" % n
        if n%2:##n is odd : m starts at 1
            mstart=1
        else: ##n is even : mstart at 0
            mstart=0
        m=mstart
        while (m<=n): ##we go through the azimutal orders
            ##we compute the radial part of the polynomial
            ##print "m=%d" % m
            #a=fastZerCoeffs(n,m)
            #Z=Rnm(n,m,a,rGridFlat,dosquare=0)*numpy.array(numpy.sqrt(n+1))
            ##we make the other computations
            if m==0: ##azimutal order = 0 => no trigonometric part
                if j>max(jmax)+1: ##we leave the while loop if jmax is reached
                    break
                else:
                    ##we add one ZP when m=0
                    #print "j=%d" % j
                    if j-1 in jmax:
                        a=fastZerCoeffs(n,m,self.natype)
                        Z=Rnm(n,m,a,rGridFlat,dosquare=0)*numpy.array(numpy.sqrt(n+1))
                        #print "zernikeMod put2",j,n,m,a
                        numpy.put(self.zern[jmax.index(j-1),:,:,].ravel(),idxPup,Z)
                    j+=1
            else: ##azimutal order >0 : there is a trigonometric part
                if ((j-1) in jmax) or j in jmax:
                    if j<=max(jmax)+1:
                        a=fastZerCoeffs(n,m,self.natype)
                        Z=Rnm(n,m,a,rGridFlat,dosquare=0)*numpy.array(numpy.sqrt(n+1))
                        Z*=numpy.array(numpy.sqrt(2.))
                ##we add the two zernike polynomials per azimutal order
                for cnt in range(2):
                    if j%2: ## j is odd : multiply by sin(m*theta)
                        if j>max(jmax)+1: ##we leave the current for loop if jmax is reached
                            break
                        else:
                            #print "Adding term in sinus"
                            #print "j=%d" % j
                            if j-1 in jmax:
                                #print j,n,m,a
                                numpy.put(self.zern[jmax.index(j-1),:,:,].ravel(),idxPup,Z*numpy.sin(m*thetaGridFlat))
                            j+=1
                    else: ## j is even : multiply by cos(m*theta)
                        if j>max(jmax)+1: ##we leave the while loop if jmax is reached
                            break
                        else:
                            #print "Adding term in cosinus"
                            #print "j=%d" % j
                            if j-1 in jmax:
                                #print j,n,m,a
                                numpy.put(self.zern[jmax.index(j-1),:,:,].ravel(),idxPup,Z*numpy.cos(m*thetaGridFlat))
                            j+=1
                               
                if (j>max(jmax)+1): ##we leave the while loop if jmax is reached
                    break


            ##we increment the azimutal order of 2
            m+=2
    
    def calcZernikeCoeff(self,data,useInvGeo=0):
        """Gives the coefficients a_i of the data projected over the Zernike polynomials 1 to self.jmax
        First computes the numeric scalar product b_i=1/puparea sum(data*self.zern[j,:,:,])
        Then computes the vector a=invGeoCovMat*b, thus taking into account the projection matrix
        """
        ##pupil's area in pixels
        area=self.puparea
        
        ##we compute the numeric scalar product
        col=(self.zern2D*data.ravel()).sum(axis=1)/float(area)#self.zern*data[na.newaxis:,];col=na.sum(col,axis=1);col=na.sum(col,axis=1)/area
        col=col.astype(self.natype)
        
        ##we do the product between the inverse of the geometric covariance function
        ##and the vect array
        ##as the geometric covariance matrix is symetric, its inverse is symetric
        ##we therefore use the function in the matrix package
        if useInvGeo:
            result=symmetricMatrixVectorMultiply(self.invGeoCovMat,col)
            # result=na.dot(self.invGeoCovMat,col)#agbhome
            result.astype(self.natype)
        else:
            result=col

        ##we return the result
        return result
    def zernikeReconstruct(self,coeff,out=None,size=None):
        if out==None:
            if size==None:
                size=self.npup
            out=numpy.zeros((size,size),numpy.float32)
        if out.shape[0]!=self.npup:
            if self.zernExpand==None or self.zernExpand.shape[1]!=out.shape[0]:
                import tel
                print "Computing zernikes"
                z=Zernike(tel.Pupil(out.shape[0],out.shape[0]/2,0).fn,coeff.shape[0])
                self.zernExpand=z.zern
            zern=self.zernExpand
        else:
            zern=self.zern
        for i in range(1,coeff.shape[0]):
            out+=coeff[i]*zern[i]
        return out
# class Pupil:#(user_array.container):#UserArray.UserArray):
#     """
#     Defines telescope pupil geometry

#     Class variables (important to simulation programmer):
#      - npup : number of pixels of the fn array
#      - r1 : radius in PIXELS of the primary mirror
#      - r2 : radius in PIXELS of the secondary mirror
#      - area : area in PIXELS of the pupil
#      - fn : numpy npup*npup array storing the pupil geometry
#     @cvar npup: number of pixels of the function array
#     @type npup: Int
#     @cvar area: Area in pixels of the pupil
#     @type area: Int
#     @cvar r1: Radius of primary mirror in Pixels
#     @type r1: Int
#     @cvar r2: Radius of secondary mirror in Pixels
#     @type r2: Int
    
#     """

#     def __init__(self,npup,r1=None,r2=0):#,nsubx=None,minarea=0.5,apoFunc=None,nAct=None,dmminarea=None):
#         """ Constructor for the Pupil class

#         Parameters: 
#          - r1 : radius in PIXELS of the primary mirror
#          - r2 : radius in PIXELS of the secondary mirror
#          - apoFunc : function defining pupil function in the case of apodised pupils
#         @param npup: number of pixels of the function array
#         @type npup: Int
#         @param apoFunc: Function determining pupil fucntion for apodised pupils
#         @type apoFunc: Function
#         @param r1: Radius of primary mirror in Pixels
#         @type r1: Int
#         @param r2: Radius of secondary mirror in Pixels
#         @type r2: Int
#         """
# ##         print "creating"
# ##         inarr=None
# ##         if type(npup)!=type(1):#assume its an array...
# ##             inarr=npup
# ##             npup=inarr.shape[0]
#         self.npup=npup
#         self.area=0.
#         if r1==None:
#             r1=npup/2
#         self.r1=r1
#         self.r2=r2
#         #self.nsubx=nsubx
#         #self.minarea=minarea
#         self.apoFunc=None#apoFunc
#         #if dmminarea==None:
#         #    self.dmminarea=minarea
#         #else:
#         #    self.dmminarea=dmminarea
#         ## we create a grid of x and y lines (to avoid for loops)
#         grid=makeCircularGrid(npup)

#         if type(self.apoFunc)==type(None):
#             self.fn=numpy.logical_and((grid<=r1),(grid>=r2))
#             self.area=numpy.sum(numpy.sum(self.fn))
#         elif type(self.apoFunc)==numpy.ndarray:#ArrayType:
#             self.fn=self.apoFunc*numpy.logical_and((grid<=r1),(grid>=r2))
#             self.area=numpy.sum(numpy.sum(self.fn))
#         else:
#             self.fn=self.apoFunc(grid)*numpy.logical_and((grid<=r1),(grid>=r2))
#             self.area=numpy.sum(numpy.sum(numpy.logical_and((grid<=r1),(grid>=r2))))
# ##         if type(inarr)!=type(None):
# ##             self.fn=inarr
#         #UserArray.UserArray.__init__(self,self.fn,copy=0)
#         #user_array.container.__init__(self,self.fn,copy=0)
#         #self.shape=self.fn.shape
#         self.sum=numpy.sum(numpy.sum(self.fn))
# ##         if nsubx!=None:
# ##             #if nAct==None:
# ##             nAct=nsubx+1
# ##             self.nAct=nAct
# ##             self.ndata=0
# ##             self.subflag=numpy.zeros((nsubx,nsubx),numpy.int32)
# ##             self.subarea=numpy.zeros((nsubx,nsubx),numpy.float64)
# ##             self.dmflag=numpy.zeros((nAct,nAct),numpy.int32)
# ##             n=npup/nsubx
# ##             self.pupsub=numpy.zeros((nsubx,nsubx,n,n),numpy.float64)
# ##             self.dmpupil=numpy.zeros((npup,npup),numpy.float64)
# ##             for i in range(nsubx):
# ##                 for j in range(nsubx):
# ##                     self.pupsub[i,j]=self.fn[i*n:(i+1)*n,j*n:(j+1)*n]
# ##                     self.subarea[i,j]=numpy.sum(numpy.sum(self.pupsub[i,j]))
# ##                     if self.subarea[i,j]>=minarea*n*n:#flag non-vignetted subaps
# ##                         self.subflag[i,j]=1
# ##                         self.ndata+=2#number of centroids that will be computed (note, 2== 1 for x, 1 for y).
# ##                     if self.subarea[i,j]>self.dmminarea*n*n:#this is only valid for nact==nsubx+1.
# ##                         self.dmflag[i,j]=self.dmflag[i+1,j]=self.dmflag[i,j+1]=self.dmflag[i+1,j+1]=1
# ##                         self.dmpupil[i*n:(i+1)*n,j*n:(j+1)*n]=1.
        

#     def _rc(self, a):
#         if len(numpy.shape(a)) == 0:
#             return a
#         else:
#             p=self.__class__(self.npup,self.r1,self.r2)#,self.nsubx,self.minarea,self.apoFunc)
#             p.fn=a
#             p.array=a
#             return p#self.__class__(a)


