import sys
import os
class Stdoutlog:
    def __init__(self,name,rotatelen=1048576,nrot=3):
        self.name=name
        self.rotatelen=rotatelen
        self.nrot=nrot
        self.names=[]
        for i in range(nrot+1):
            self.names.append("%s%d"%(name,i))
        self.fd=open(self.names[0],"a",1)
        self.fd.write("\n***** Log cycle starting *****\n\n")
    def write(self,txt):
        self.fd.write(txt)
        if self.rotatelen!=None:
            size=os.fstat(self.fd.fileno()).st_size
            if size>self.rotatelen:
                self.fd.write("LOGROTATE\n")
                for i in range(self.nrot,0,-1):
                    if os.path.exists(self.names[i-1]):
                        os.rename(self.names[i-1],self.names[i])
                self.fd=open(self.names[0],"w",1)
                self.fd.write("New log cycle\n")
    def fileno(self):
        return self.fd.fileno()
