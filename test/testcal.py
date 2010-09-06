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
