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
#!/usr/bin/env python
CVSID="$Id$"
import sys
import threading
import traceback
import Pyro.core
import Pyro.naming
import controlVirtual

def encode(val,typ=None):
    return val
def decode(val,typ=None):
    return val

class ControlServer(controlVirtual.ControlServer,Pyro.core.ObjBase):
    def __init__(self,c=None,l=None):
        """c is the instance of the control object
        l is a lock that should be obtained before calling an operation.
        """
        Pyro.core.ObjBase.__init__(self)
        controlVirtual.ControlServer.__init__(self,c,l)
        self.encode=encode
        self.decode=decode




class Control(controlVirtual.Control):
    def __init__(self,controlName="",debug=0):
        if "Control" not in controlName:
            self.prefix=controlName
            controlName=controlName+"Control"
        else:
            #depreciated but still widely used.
            self.prefix=controlName[:-7]
        self.encode=encode
        self.decode=decode
        controlVirtual.Control.__init__(self,self.prefix)
        self.debug=debug
        self.obj=None
        uri="PYRONAME://darc%s"%controlName
        if self.debug:
            print "Attemptint to connect to darc Pyro with URI %s"%uri
        self.obj=Pyro.core.getProxyForURI(uri)
        if self.obj==None:
            print "Cannont find URI %s - not connected"%uri
        else:
            if self.debug:
                print "Connected to %s"%uri
            # Invoke the echoString operation
            message = "Hello from Python"
            try:
                result = self.obj.echoString(message)
            except:
                result="nothing (failed)"
                traceback.print_exc()
                print "EchoString failed - continuing but not connected"
                self.obj=None
            if self.debug:
                print "I said '%s'. The object said '%s'." % (message,result)
        #return self.obj!=None


def initialiseServer(c=None,l=None,block=0,controlName="Control"):
    """c is the control object
    l is a threading.Lock object (or soemthing with acquire and release methods
    block is whether to block here, or return.
    controlName is the darc prefix+Control.
    """
    print "Making Pyro daemon"
    daemon=Pyro.core.Daemon()
    print 'Locating Pyro name server'
    ns=Pyro.naming.NameServerLocator().getNS()
    print 'Found',ns
    daemon.useNameServer(ns)
    try:
        ns.unregister("darc%s"%controlName)
    except:
        pass

    print "Creating control object"
    ei = ControlServer(c,l)
    print "Registering darc object"
    uri=daemon.connect(ei,"darc%s"%controlName)
    print "Starting Pyro request loop"
    if block:
        # Block for ever (or until the pyro is shut down)
        daemon.requestLoop()
    else:
        t=threading.Thread(target=daemon.requestLoop)
        t.daemon=True
        t.start()
    return ei



if __name__=="__main__":
    controlName="Control"
    for arg in sys.argv[1:]:
        if arg[:2]=="-s":#shmname
            controlName=arg[2:]+controlName
    
    initialiseServer(block=1,controlName=controlName)

