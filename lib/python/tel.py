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
#$Id$
"""
Tel.py : functions and classes to define telescope pupil geometry
"""


#import UserArray
import numpy.lib.user_array as user_array
import numpy as na
import types

def makeCircularGrid(nxpup,nypup=None,natype=na.float64,dosqrt=1):
    """
    returns a npup*npup numpy array with a grid of distance of pixels
    from the center of the screen

    Distance definition not suitable with FFT !!!

    default return type : Float64
    """
    if nypup==None:
        nypup=nxpup
    tabx=na.arange(nxpup)-float(nxpup/2.)+0.5 ##RWW's convention
    taby=na.arange(nypup)-float(nypup/2.)+0.5 ##RWW's convention
    grid=tabx[na.newaxis,:,]**2+taby[:,na.newaxis]**2
    if dosqrt:
        na.sqrt(grid,grid)
    return grid.astype(natype)

### Anular pupil function #############################################
class Pupil(user_array.container):#UserArray.UserArray):
    """
    Defines telescope pupil geometry

    Class variables (important to simulation programmer):
     - npup : number of pixels of the fn array
     - r1 : radius in PIXELS of the primary mirror
     - r2 : radius in PIXELS of the secondary mirror
     - area : area in PIXELS of the pupil
     - fn : numpy npup*npup array storing the pupil geometry
    @cvar npup: number of pixels of the function array
    @type npup: Int
    @cvar area: Area in pixels of the pupil
    @type area: Int
    @cvar r1: Radius of primary mirror in Pixels
    @type r1: Int
    @cvar r2: Radius of secondary mirror in Pixels
    @type r2: Int
    
    """

    def __init__(self,npup,r1=None,r2=0,nsubx=None,minarea=0.5,apoFunc=None,nAct=None,dmminarea=None):
        """ Constructor for the Pupil class

        Parameters: 
         - r1 : radius in PIXELS of the primary mirror
         - r2 : radius in PIXELS of the secondary mirror
         - apoFunc : function defining pupil function in the case of apodised pupils
        @param npup: number of pixels of the function array
        @type npup: Int
        @param apoFunc: Function determining pupil fucntion for apodised pupils
        @type apoFunc: Function
        @param r1: Radius of primary mirror in Pixels
        @type r1: Int
        @param r2: Radius of secondary mirror in Pixels
        @type r2: Int
        """
##         print "creating"
##         inarr=None
##         if type(npup)!=type(1):#assume its an array...
##             inarr=npup
##             npup=inarr.shape[0]
        self.npup=npup
        self.area=0.
        if r1==None:
            r1=npup/2
        self.r1=r1
        self.r2=r2
        self.nsubx=nsubx
        self.minarea=minarea
        self.apoFunc=apoFunc
        if dmminarea==None:
            self.dmminarea=minarea
        else:
            self.dmminarea=dmminarea
        ## we create a grid of x and y lines (to avoid for loops)
        grid=makeCircularGrid(npup)

        if type(apoFunc)==types.NoneType:
            self.fn=na.logical_and((grid<=r1),(grid>=r2))
            self.area=na.sum(na.sum(self.fn))
        elif type(apoFunc)==na.ndarray:#ArrayType:
            self.fn=apoFunc*na.logical_and((grid<=r1),(grid>=r2))
            self.area=na.sum(na.sum(self.fn))
        else:
            self.fn=apoFunc(grid)*na.logical_and((grid<=r1),(grid>=r2))
            self.area=na.sum(na.sum(na.logical_and((grid<=r1),(grid>=r2))))
##         if type(inarr)!=type(None):
##             self.fn=inarr
        #UserArray.UserArray.__init__(self,self.fn,copy=0)
        user_array.container.__init__(self,self.fn,copy=0)
        #self.shape=self.fn.shape
        self.sum=na.sum(na.sum(self.fn))
        if nsubx!=None:
            #if nAct==None:
            nAct=nsubx+1
            self.nAct=nAct
            self.ndata=0
            self.subflag=na.zeros((nsubx,nsubx),na.int32)
            self.subarea=na.zeros((nsubx,nsubx),na.float64)
            self.dmflag=na.zeros((nAct,nAct),na.int32)
            n=npup/nsubx
            self.pupsub=na.zeros((nsubx,nsubx,n,n),na.float64)
            self.dmpupil=na.zeros((npup,npup),na.float64)
            for i in range(nsubx):
                for j in range(nsubx):
                    self.pupsub[i,j]=self.fn[i*n:(i+1)*n,j*n:(j+1)*n]
                    self.subarea[i,j]=na.sum(na.sum(self.pupsub[i,j]))
                    if self.subarea[i,j]>=minarea*n*n:#flag non-vignetted subaps
                        self.subflag[i,j]=1
                        self.ndata+=2#number of centroids that will be computed (note, 2== 1 for x, 1 for y).
                    if self.subarea[i,j]>self.dmminarea*n*n:#this is only valid for nact==nsubx+1.
                        self.dmflag[i,j]=self.dmflag[i+1,j]=self.dmflag[i,j+1]=self.dmflag[i+1,j+1]=1
                        self.dmpupil[i*n:(i+1)*n,j*n:(j+1)*n]=1.
        
    def getSubapFlag(self,nsubx,minarea=None):
        """Compute the subap flags for a given nsubx"""
        if minarea==None:
            minarea=self.minarea
        subflag=na.zeros((nsubx,nsubx),na.int32)
        n=self.npup/nsubx
        minarea*=n*n
        for i in range(nsubx):
            for j in range(nsubx):
                subarea=na.sum(na.sum(self.fn[i*n:(i+1)*n,j*n:(j+1)*n]))
                if subarea>=minarea:
                    subflag[i,j]=1
        return subflag
        
    def getSubarea(self,nsubx):
        """compute subarea"""
        subarea=na.zeros((nsubx,nsubx),na.float64)
        n=self.npup/nsubx
        #might be faster to use cmod.binimg here?
        for i in xrange(nsubx):
            for j in xrange(nsubx):
                subarea[i,j]=na.sum(self.fn[i*n:(i+1)*n,j*n:(j+1)*n])
        return subarea

    def _rc(self, a):
        if len(na.shape(a)) == 0:
            return a
        else:
            p=self.__class__(self.npup,self.r1,self.r2,self.nsubx,self.minarea,self.apoFunc)
            p.fn=a
            p.array=a
            return p#self.__class__(a)

    def perSubap(self,nsubx=None,vectorAlign=1):
        """Rearrange the data so that memory is sequential for subaps - this is used by the cell...
        If vectorAlign is set, it makes each row of the subap pupils to be 16 bytes in size."""
        if nsubx==None:
            nsubx=self.nsubx
        if nsubx==None:
            raise Exception("Error: Number of x subaps must be defined")
        nphs=self.npup/nsubx
        if vectorAlign:
            nphs_v=(nphs+3)&~3
        else:
            nphs_v=nphs
        a=na.zeros((nsubx,nsubx,nphs,nphs_v),na.float32)
        for i in range(nsubx):
            for j in range(nsubx):
                a[i,j,:,:nphs]=self.fn[i*nphs:(i+1)*nphs,j*nphs:(j+1)*nphs].astype(na.float32)
        return a

    def asDoubleSized(self):
        """return a pupil fn array double the size..."""
        return Pupil(self.npup*2,self.r1*2,self.r2*2,self.nsubx,self.minarea)
    
class RectangularPupil:
    """Defines telescope pupil geometry, can allow for oval pupil shape.  Note, the pupil is still circular, but the array on which it belongs is rectangular.
    Class variables (important to simulation programmer):
     - nxpup : number of pixels of the fn array in x direction
     - nypup : number of pixels of the fn array in x direction
     - r1 : radius in PIXELS of the primary mirror
     - r2 : radius in PIXELS of the secondary mirror
     - area : area in PIXELS of the pupil
     - fn : numpy npup*npup array storing the pupil geometry
    @cvar nxpup: number of pixels of the function array
    @type nxpup: Int
    @cvar nypup: number of pixels of the function array
    @type nypup: Int
    @cvar area: Area in pixels of the pupil
    @type area: Int
    @cvar r1: Radius of primary mirror in Pixels
    @type r1: Int
    @cvar r2: Radius of secondary mirror in Pixels
    @type r2: Int
    """
    def __init__(self,nxpup,nypup,r1,r2,apoFunc=None):
        """ Constructor for the Pupil class

        Parameters: 
         - r1 : radius in PIXELS of the primary mirror
         - r2 : radius in PIXELS of the secondary mirror
         - apoFunc : function defining pupil function in the case of apodised pupils
        @param nxpup: number of pixels of the function array
        @type nxpup: Int
        @param nypup: number of pixels of the function array
        @type nypup: Int
        @param apoFunc: Function determining pupil fucntion for apodised pupils
        @type apoFunc: Function
        @param r1: Radius of primary mirror in Pixels
        @type r1: Int
        @param r2: Radius of secondary mirror in Pixels
        @type r2: Int
        """
        self.nxpup=nxpup
        self.nypup=nypup
        self.area=0.
        self.r1=r1
        self.r2=r2

        ## we create a grid of x and y lines (to avoid for loops)
        grid=makeCircularGrid(nxpup,nypup)

        if type(apoFunc)==types.NoneType:
            self.fn=na.logical_and((grid<=r1),(grid>=r2))
            self.area=na.sum(na.sum(self.fn))
        else:
            self.fn=apoFunc(grid)*na.logical_and((grid<=r1),(grid>=r2))
            self.area=na.sum(na.sum(na.logical_and((grid<=r1),(grid>=r2))))
    

def rms(arr):
    """compute the RMS of an array..."""
    arr=na.array(arr).flat
    std=na.sqrt(na.average(arr*arr)-na.average(arr)**2)
    return std
