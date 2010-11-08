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
import controlCorba
import FITS
#Create the corba client
c=controlCorba.controlClient()
def set(name,val,com="",swap=1,check=1):
    c.obj.Set(controlCorba.sdata(name),controlCorba.encode([val]),controlCorba.sdata(com),swap,check)

#Acquire a calibrated image, averaged over 10 frames
data=c.obj.AverageImage(10,0)

#Decode and reshape the return
data=controlCorba.decode(data)
data.shape=128,128

FITS.Write(data,"tmp.fits")
