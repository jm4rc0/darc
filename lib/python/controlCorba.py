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
import sys
import numpy
import traceback
import controlVirtual
from omniORB import CORBA, PortableServer
import CosNaming
import control_idl


def encode(val,typ=None):
    """Convert into a CORBA SERVAL type"""
    rt=None
    dtype="n"
    nd=0
    dims=[]
    size=0
    data=""
    if val is None:#note, val==None is depreciated in numpy 1.9 onwards
        pass
    elif typ is not None:
        if typ==[str]:
            return sdata(val)
        else:
            print "FIXME: encode: %s %s"%(str(typ),str(type(val)))
    if val is None:#note, val==None is depreciated in numpy 1.9 onwards
        pass
    elif type(val) in [numpy.ndarray,numpy.memmap]:
        #print "encode:",val.dtype.char
        if val.dtype.char=="d":
            print "Warning, converting float64 array to float32 in encode()"
            val=val.astype("f")
        if val.dtype.char in ["f","b","B","h","i","H","d"] or (val.dtype.char=="l" and val.itemsize==4):
            dtype=val.dtype.char
            if dtype=="l" and val.itemsize==4:
                dtype="i"
            nd=len(val.shape)
            dims=val.shape
            size=val.size*val.itemsize
            data=val.tostring()
        else:
            raise Exception("numpy type %s not convertable"%val.dtype.char)
    elif type(val) in [type(0),numpy.int32]:
        dtype="i"
        size=4
        data=numpy.array(val).astype(numpy.int32).tostring()
    elif type(val) in [type(0.),numpy.float32]:
        dtype="f"
        size=4
        data=numpy.array(val).astype(numpy.float32).tostring()
    elif type(val) in [numpy.float64]:
        dtype="d"
        size=8
        data=numpy.array(val).astype(numpy.float64).tostring()
    elif type(val)==type(""):
        dtype="s"
        size=len(val)
        data=val
    elif type(val)==type([]):
        vlist=[]
        for v in val:
            vlist.append(encode(v))
        rt=control_idl._0_RTC.Control.SVDATA(len(vlist),vlist)    
    elif type(val) in [type(numpy.arange(1).view("i")[0])]:
        #numpy bugfix on 32 bit platforms
        dtype="i"
        size=4
        data=numpy.array(val).astype(numpy.int32).tostring()
    else:
        raise Exception("dtype %s not yet convertable"%str(type(val)))
    if rt==None:
        #print "Encoding",dtype,nd,dims,size
        rt=control_idl._0_RTC.Control.SERVAL(dtype,nd,dims,size,data)
    return rt


def decode(val,typ=None):
    """Convert from CORAB SERVAL type or SVDATA to python"""
    if typ is not None:
        print "decode: %s %s"%(str(typ),str(type(val)))
    if isinstance(val,control_idl._0_RTC.Control.SVDATA):
        rt=[]
        for v in val.data:
            rt.append(decode(v))
    elif isinstance(val,control_idl._0_RTC.Control.FDATA):
        rt=numpy.fromstring(val.data,numpy.float32)
    elif isinstance(val,control_idl._0_RTC.Control.DDATA):
        rt=numpy.fromstring(val.data,numpy.float64)
    elif isinstance(val,control_idl._0_RTC.Control.BDATA):
        rt=numpy.fromstring(val.data,numpy.int8)
    elif isinstance(val,control_idl._0_RTC.Control.HDATA):
        rt=numpy.fromstring(val.data,numpy.int16)
    elif isinstance(val,control_idl._0_RTC.Control.UHDATA):
        rt=numpy.fromstring(val.data,numpy.uint16)
    elif isinstance(val,control_idl._0_RTC.Control.IDATA):
        rt=numpy.fromstring(val.data,numpy.int32)
    elif isinstance(val,control_idl._0_RTC.Control.SDATA):
        rt=val.data
    elif val.dtype=="n":
        rt=None
    elif val.dtype in ["f","b","B","h","i","H","d"]:
        rt=numpy.fromstring(val.data,val.dtype)
        #print rt.size,rt.dtype.char,len(val.data)
        if val.nd==0:
            if val.dtype in ["f"]:#,"d"]:
                rt=float(rt[0])
            elif val.dtype in ["d"]:
                rt=rt[0]#keep as numpy.float64 type (150209)
            else:
                rt=int(rt[0])
        else:
            rt.shape=val.dims[:val.nd]
    elif val.dtype=="s":
        rt=val.data
    else:
        raise Exception("dtype %s not yet implemented"%val.dtype)
    return rt

def sdata(val):
    if type(val)!=type([]):
        if type(val)==type(""):
            val=[val]
        else:
            raise Exception("controlCorba.sdata() must take string or list of")
    rt=control_idl._0_RTC.Control.SDATA(len(val),val)
    return rt

def hcf(no1,no2):  
    while no1!=no2:  
        if no1>no2:  
            no1-=no2  
        else:
            no2-=no1  
    return no1  






class ControlServer(controlVirtual.ControlServer,control_idl._0_RTC__POA.Control):
    def __init__(self,c=None,l=None):
        """c is the instance of the control object
        l is a lock that should be obtained before calling an operation.
        """
        #Pyro.core.ObjBase.__init__(self)
        controlVirtual.ControlServer.__init__(self,c,l)
        self.encode=encode
        self.decode=decode

class Control(controlVirtual.Control):
    def __init__(self,controlName="",debug=0,orb=None):
        if "Control" not in controlName:
            self.prefix=controlName
            controlName=controlName+"Control"
        else:
            #depreciated but still widely used.
            print "DEPRECIATION WARNING: don't add Control to prefix (controlCorba.py)"
            self.prefix=controlName[:-7]
        self.encode=encode
        self.decode=decode
        controlVirtual.Control.__init__(self,self.prefix)
        self.debug=debug
        if orb==None:
            orb=CORBA.ORB_init(sys.argv,CORBA.ORB_ID)
        self.orb=orb
        self.obj=None
        self.printcnt=0
        self.printat=1
        self.connectControl(controlName)

    def connectControl(self,controlName="Control"):
        if self.debug:
            print "Attemptint to connect to rtc Control Corba"
        self.obj=None
        # Initialise the ORB
        #orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)
        pr=0
        self.printcnt+=1
        if self.printcnt>=self.printat:
            self.printcnt=0
            self.printat*=2
            pr=1
        orb=self.orb
        # Obtain a reference to the root naming context
        obj = orb.resolve_initial_references("NameService")
        try:
            rootContext = obj._narrow(CosNaming.NamingContext)
        except:
            if pr:
                print "Unable to connect to nameservice"
            return False
        if rootContext is None:
            if pr:
                print "Failed to narrow the root naming context"
        # Resolve the name "darc.server/Control.Object"
        name = [CosNaming.NameComponent("darc", "server"),
                CosNaming.NameComponent(controlName, "Object")]
        try:
            obj = rootContext.resolve(name)
        except CosNaming.NamingContext.NotFound, ex:
            if pr:
                print "Name not found"
        else:
            # Narrow the object to an RTC::Control
            self.obj = obj._narrow(control_idl._0_RTC.Control)
        if self.obj is None:
            if pr:
                print "Object reference is not an RTC::Control - not connected"
        else:
            if self.debug:
                print "Connected to rtc Control Corba",self.obj
            self.printcnt=0
            self.printat=1
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
        return self.obj!=None




def initialiseServer(c=None,l=None,block=0,controlName="Control"):
    """c is the control object
    l is a threading.Lock object (or soemthing with acquire and release methods
    block is whether to block here, or return.
    """
    # Initialise the ORB and find the root POA
    orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)
    poa = orb.resolve_initial_references("RootPOA")
    # Create an instance of Control_i and a Control object reference
    ei = ControlServer(c,l)
    eo = ei._this()
    # Obtain a reference to the root naming context
    obj         = orb.resolve_initial_references("NameService")
    try:
        rootContext = obj._narrow(CosNaming.NamingContext)
    except:
        print "Unable to connect the nameservice"
        return None
    if rootContext is None:
        print "Failed to narrow the root naming context"
        sys.exit(1)
    # Bind a context named "darc.server" to the root context
    name = [CosNaming.NameComponent("darc", "server")]
    try:
        rtcServerContext = rootContext.bind_new_context(name)
        #print "New darc context bound"
    except CosNaming.NamingContext.AlreadyBound, ex:
        #print "RtcControl context already exists"
        obj = rootContext.resolve(name)
        rtcServerContext = obj._narrow(CosNaming.NamingContext)
        if rtcServerContext is None:
            print "darc.server exists but is not a NamingContext"
            sys.exit(1)
    # Bind the Control object to the darc context
    name = [CosNaming.NameComponent(controlName, "Object")]
    try:
        rtcServerContext.bind(name, eo)
        #print "New Control object bound"
    except CosNaming.NamingContext.AlreadyBound:
        rtcServerContext.rebind(name, eo)
        #print "Control binding already existed -- rebound"
    # Activate the POA
    poaManager = poa._get_the_POAManager()
    poaManager.activate()
    if block:
        # Block for ever (or until the ORB is shut down)
        orb.run()
    return ei

def unbind(controlName="Control"):
    # Initialise the ORB and find the root POA
    orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)
    poa = orb.resolve_initial_references("RootPOA")
    obj = orb.resolve_initial_references("NameService")
    try:
        rootContext = obj._narrow(CosNaming.NamingContext)
    except:
        print "Cannot unbind: Unable to connect the nameservice"
        return None
    if rootContext is None:
        print "Cannot unbind: Failed to narrow the root naming context"
        return None
    # Bind a context named "darc.server" to the root context
    name = [CosNaming.NameComponent("darc", "server")]
    try:
        rtcServerContext = rootContext.bind_new_context(name)
        print "New darc context bound"
    except CosNaming.NamingContext.AlreadyBound, ex:
        print "RtcControl context already exists"
        obj = rootContext.resolve(name)
        rtcServerContext = obj._narrow(CosNaming.NamingContext)
        if rtcServerContext is None:
            print "darc.server exists but is not a NamingContext"
            return None
    # Bind the Control object to the darc context
    name = [CosNaming.NameComponent(controlName, "Object")]
    try:
        rtcServerContext.unbind(name)
    except:
        print "Cannot unbind..."
        return None
    print "Unbound"
    return True

    



if __name__=="__main__":
    controlName="Control"
    for arg in sys.argv[1:]:
        if arg[:2]=="-s":#shmname
            controlName=arg[2:]+controlName
    
    initialiseServer(block=1,controlName=controlName)

        
