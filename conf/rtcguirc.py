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
"""A sample rtcguirc.py script - used to initialise the GUI."""
self.plotConfigDict["wfs image"]="tmp.xml"
self.centOverlayPattern=numpy.zeros((8,8,4),"f")
self.centOverlayPattern[:,:]=1
self.centOverlayPattern[2:6,2:6,2]=0
self.centOverlayPattern[0::7,0::7]=0
