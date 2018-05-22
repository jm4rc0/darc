#darc, the Durham Adaptive optics Real-time Controller.
#Copyright (C) 2010 Alastair Basden.
#
#This program is free software: you can redistribute it and/or modify
#it under the terms of the GNU Affero General Public License as
#published by the Free Software Foundation, either version 3 of the
#License, or (at your option) any later version.
#
#This program is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU Affero General Public License for more details.
#
#You should have received a copy of the GNU Affero General Public License
#along with this program.  If not, see <http://www.gnu.org/licenses/>.
import numpy

def cog(npxlx,npxly,subapLocation):
    """Make a centIndexArray which will have the same result as a CoG."""
    nsub=subapLocation.size//6
    subapLocation.shape=nsub,6
    centIndexArray=numpy.zeros((npxly,npxlx,4),numpy.float32)
    centIndexArray[:,:,2:]=1
    for i in range(nsub):
        sl=subapLocation[i]
        ny=(sl[1]-sl[0])//sl[2]
        nx=(sl[4]-sl[3])//sl[5]
        if ny*nx>0:
            centIndexArray[sl[0]:sl[1]:sl[2],sl[3]:sl[4]:sl[5],0]=numpy.arange(ny)[:,None]-ny/2+0.5
            centIndexArray[sl[0]:sl[1]:sl[2],sl[3]:sl[4]:sl[5],1]=numpy.arange(nx)-nx/2+0.5
    return centIndexArray
