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
"""A script to test the new calibration method...
This assumes that the RTC has been initialised in single camera mode.
Note, no cameras are actually required to be attached, since we use a fake image.

This script records slope measurements for various different threshold values and algorithms and pixel weightings.

It writes the results into a single FITS file, (default tmp.fits, can be specified on the command line), giving a FITS file with multiple header - data units.

"""
import controlCorba
import FITS
import sys
import time
file="tmp.fits"
if len(sys.argv)>1:
    file=sys.argv[1]

#Create the corba client
c=controlCorba.controlClient()

def set(name,val,com="",swap=1,check=1):
    c.obj.Set(controlCorba.sdata(name),controlCorba.encode([val]),controlCorba.sdata(com),swap,check)
    #time.sleep(0.01)

set("fakeCCDImage",FITS.Read("shimage.fits")[1][0])
FITS.Write(controlCorba.decode(c.obj.AverageCentroids(1)),file)

set("thresholdAlgorithm",2)
FITS.Write(controlCorba.decode(c.obj.AverageCentroids(1)),file,writeMode="a")

set("thresholdAlgorithm",0)
FITS.Write(controlCorba.decode(c.obj.AverageCentroids(1)),file,writeMode="a")

set("thresholdValue",0)
FITS.Write(controlCorba.decode(c.obj.AverageCentroids(1)),file,writeMode="a")

set("thresholdAlgorithm",1)
FITS.Write(controlCorba.decode(c.obj.AverageCentroids(1)),file,writeMode="a")

set("thresholdAlgorithm",2)
FITS.Write(controlCorba.decode(c.obj.AverageCentroids(1)),file,writeMode="a")

set("pxlWeight",FITS.Read("pxlweight.fits")[1])
FITS.Write(controlCorba.decode(c.obj.AverageCentroids(1)),file,writeMode="a")

set("thresholdAlgorithm",1)
FITS.Write(controlCorba.decode(c.obj.AverageCentroids(1)),file,writeMode="a")

set("thresholdAlgorithm",0)
FITS.Write(controlCorba.decode(c.obj.AverageCentroids(1)),file,writeMode="a")

set("thresholdValue",2)
FITS.Write(controlCorba.decode(c.obj.AverageCentroids(1)),file,writeMode="a")

set("thresholdAlgorithm",1)
FITS.Write(controlCorba.decode(c.obj.AverageCentroids(1)),file,writeMode="a")

set("thresholdAlgorithm",2)
FITS.Write(controlCorba.decode(c.obj.AverageCentroids(1)),file,writeMode="a")

set("pxlWeight",None)
FITS.Write(controlCorba.decode(c.obj.AverageCentroids(1)),file,writeMode="a")


print "done"
