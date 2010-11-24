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
                self.fd.close()
                for i in range(self.nrot,0,-1):
                    if os.path.exists(self.names[i-1]):
                        os.rename(self.names[i-1],self.names[i])
                self.fd=open(self.names[0],"w",1)
                self.fd.write("New log cycle\n")
        self.fd.flush()
    def fileno(self):
        return self.fd.fileno()
