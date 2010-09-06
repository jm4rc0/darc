"""A sample rtcguirc.py script - used to initialise the GUI."""
self.plotConfigDict["wfs image"]="tmp.xml"
self.centOverlayPattern=numpy.zeros((8,8,4),"f")
self.centOverlayPattern[:,:]=1
self.centOverlayPattern[2:6,2:6,2]=0
self.centOverlayPattern[0::7,0::7]=0
