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
