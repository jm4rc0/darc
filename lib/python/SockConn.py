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
#$Id$
import socket,serialise,types,os,select,cPickle,thread,time
import ConnObj
import sys
import traceback
class dummyLock:
    def acquire(self):
        pass
    def release(self):
        pass
class SockConn:
    
    """Class to open a listening socket, and start a select loop in a
    new thread.  Each simulation process will start a sockConn object.
    It opens a listening socket (if it cannot bind to the given port,
    it will simply try increasing port numbers until it succeeds), and
    enters a main loop (in a separate thread from the main process)
    waiting for clients to connect.  Once connected, clients (such as
    ConnObj objects) send instances of ConnMsg objects (actually
    ConnMsg.pickle() objects), which are used to instruct the main
    simulation process.
    
    A received command can be executed immediately (not advisable
    because the simulation will be at an unknown state), or at the end
    of an iteration cycle.  Additionally, the same command can be
    executed at the end of every iteration cycle if desired.  These
    commands have access to the entire simulation process, and so can
    be used to alter state, change parameters, or (more normally)
    return data to the client process (user).     

    Class variables (important to simulation programmer):
     - port - int, port number to listen on
     - host - string, hostname
     - globals - global dictionary
    Class variables (not important to simulation programmer):
     - go - int, determines whether to listen or not
     - fwd - tuple or None, whether a forwarding socket
     - lsock - socket object, listening socket
     - printmsg - int, whether to print messages
     - selIn - list, for select()
     - selOut - list, for select()
     - cmdList - list, of commands to be executed
     - rptCmdList - list, of commands to be executed every iteration
     - fsock - forwading socket

    @cvar port: port number to listen on
    @type port: int
    @cvar host: hostname
    @type host: String
    @cvar go: determines whether to listen or not
    @type go: Int
    @cvar fwd: whether a forwarding socket
    @type fwd: Tuple (hostname,port) or None
    @cvar lsock: listening socket
    @type lsock: socket.socket instance
    @cvar printmsg: whether to print messages
    @type printmsg: Int
    @cvar selIn: for select()
    @type selIn: List
    @cvar selOut: for select()
    @type selOut: List
    @cvar cmdList: commands to be executed
    @type cmdList: List
    @cvar rptCmdList: commands to be executed every iteration
    @type rptCmdList: List
    @cvar fsock: forwading socket
    @type fsock: socket.socket instance
    @cvar globals: global dictionary
    @type globals: Dict
    """
    def __init__(self, port, host=socket.gethostname(), fwd=None,globals=None,startThread=1,listenSTDIN=1,userSelList=[],userSelCmd=None,connectFunc=None,lock=dummyLock(),verbose=1,timeoutFunc=None,timeout=None):
        """Opens a listening port, and either acts on commands send, or if
        fwd is (host,port), forwards commands to this port (little used)
        @param port: Port number to listen on
        @type port: Int
        @param host: hostname
        @type host: String
        @param fwd: Forwarding socket information
        @type fwd: None or Tuple
        @param globals: Global dictionary
        @type globals: Dict
        @param startThread: Whether to start the listening loop
        @type startThread: Int
        @param listenSTDIN: whether to listen on stdin.
        @type listenSTDIN: Int
        @param userSelList: Optional list of sockets to listen on
        @type userSelList: List of ints
        @param userSelCmd: The command to call if one of the optional sockets has data waiting
        @type userSelCmd: Function
        """
        self.port=port
        self.host=host
        self.go=1
        self.lock=lock
        self.fwd=fwd
        self.userSelList=userSelList
        self.userSelCmd=userSelCmd
        self.timeout=timeout
        self.timeoutFunc=timeoutFunc
        self.lsock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.lsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
        self.printmsg=0
        self.bound=0
        self.verbose=verbose
        self.connectFunc=connectFunc
        self.writeDict={}#dictionary used to hold data to be written to the socket - for nonblocking writes... keys are the sockets...
        cnt=0
        while self.bound==0:
            try:
                self.lsock.bind((host,self.port))
            except:
                print "Couldn't bind to port %d on %s.  "%(self.port,host)
                self.port+=1
                time.sleep(0.5)
                #raise
            else:
                self.bound=1
                if self.verbose:
                    print "Bound to port %d on %s"%(self.port,host)
        if self.bound:
            self.lsock.listen(1)
            self.listenSTDIN=listenSTDIN
            self.selIn=[self.lsock]
            if self.listenSTDIN:
                self.selIn.append(sys.stdin)
            self.selIn+=self.userSelList
            self.selOut=[]
            self.cmdList=[]
            self.rptCmdList=[]
            self.fsock=None
            self.globals=globals
            if self.fwd!=None:
                self.fsock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.fsock.connect(self.fwd)
                self.selIn.append(self.fsock)
            if startThread:
                time.sleep(1)
                thread.start_new_thread(self.loop,())
    def setGlobals(self,globs):
        """sets the globals which are the objects we have access to...
        @param globs: typically globals()
        @type globs: Dict"""
        self.globals=globs
    def endLoop(self):
        """End the loop"""
        self.go=0
        
    def getNoWaiting(self):
        selOut=[]
        for k in self.writeDict.keys():
            if len(self.writeDict[k])>0:
                selOut.append(k)
        selErr=self.selIn+selOut
        rtr,rtw,err=select.select(self.selIn,selOut,selErr,0.)
        return len(rtr)+len(rtw)+len(err)

    def loop(self):
        """probably start this in a new thread... listens on the socket, and
        does stuff depending on what it gets"""
        while self.go:
            if self.fsock==-1:
                break
            selOut=[]
            for k in self.writeDict.keys():
                if len(self.writeDict[k])>0:
                    selOut.append(k)
            selErr=self.selIn+selOut
            #print self.selIn,selOut
            if len(selErr)==0:
                if self.verbose:
                    print "No connections any more... will exit"
                self.go=0
                break
            try:
                rtr,rtw,err=select.select(self.selIn,selOut,selErr,self.timeout)
            except KeyboardInterrupt:
                print "Keyboard interrupt in SockConn.loop"
                raise
            except:
                print "Error in select"
                print self.selIn
                traceback.print_exc()
                #Remove the offending sockets
                remlist=[]
                for s in self.selIn:
                    try:
                        rtr,rtw,err=select.select([s],[],[],0)
                    except:
                        remlist.append(s)
                print "Removing %s"%str(remlist)
                for s in remlist:
                    self.selIn.remove(s)
                    
                remlist=[]
                for s in selOut:
                    try:
                        rtr,rtw,err=select.select([],[s],[],0)
                    except:
                        remlist.append(s)
                print "Removing %s"%str(remlist)
                for s in remlist:
                    selOut.remove(s)
                    
                

                rtr=[]
                rtw=[]
                for s in self.selIn:
                    pass
            if self.timeout!=None and len(rtr)+len(rtw)+len(err)==0 and self.timeoutFunc!=None:
                self.timeoutFunc()
            for s in err:
                #remove from dict, and act on it...
                try:
                    self.close(s)
                except:
                    print "sockConn.py - error in self.close"
                    traceback.print_exc()
            for s in rtr:#ready to read the socket
                if s==self.lsock:#listening socket - new client...
                    if self.verbose:
                        print 'Accepting new client...'
                    conn,raddr=s.accept()
                    if self.verbose:
                        print "from %s"%str(raddr)
                    self.selIn.append(conn)
                    if self.connectFunc!=None:
                        self.connectFunc(conn)
                elif s==sys.stdin:
                    #print "Got from stdin"
                    try:
                        data=s.readline()[:-1]
                    except:
                        data="Error reading from stdin"
                    #print self.globals
                    try:
                        ft=self.globals["ctrl"].frametime
                        ti=self.globals["ctrl"].thisiter
                    except:
                        ti=0
                        ft=1.
                    if ft==0:
                        ft=1.
                    print "At iteration %d, taking %gs per frame (%g fps)"%(ti,ft,1./ft)
                    #print data,len(data)
                elif s==self.fsock:#forward data... (return to client)
                    data=s.recv(1024)
                    length=len(data)
                    if length==0:
                        self.close(s)
                    else:
                        for sk in self.selIn:
                            if sk!=self.lsock and sk!=self.fsock:
                                try:
                                    sk.sendall(data)
                                except:
                                    self.close(sk)
                elif s in self.userSelList:
                    if self.userSelCmd(s)==False:
                        if self.verbose:
                            print "SockConn removing %s"%str(s)
                        try:
                            self.selIn.remove(s)
                        except:
                            pass
                        #print "selIn now %s"%str(self.selIn)
                        try:
                            self.userSelList.remove(s)
                        except:
                            pass
                else:#readsock or forward data...
                    if self.fsock!=None:#forward the data...
                        data=s.recv(1024)
                        if len(data)==0:
                            self.close(s)
                        else:
                            try:
                                self.fsock.sendall(data)
                            except:
                                serialise.Send(["warning",None,"couldn't forward data"],s)
                    else:#readsock.
                        #print 'Reading socket...'
                        if self.readsock(s)==-1:#connection closed
                            try:
                                print 'Closing socket %s'%str(s.getpeername())
                            except:
                                print "Closing socket"
                            self.close(s)
            for s in rtw:
                #Now write to sockets that have data waiting to be written...
                if self.writeDict.has_key(s) and len(self.writeDict[s])>0:
                    data=self.writeDict[s].pop(0)
                    if len(data)>0:
                        #print type(data),s,len(data)
                        #sent=s.send(data)
                        #data=data[sent:]#remove the sent portion...
                        sent=1
                        while sent>0:#repeat while socket still working...
                            #print type(data)
                            try:
                                sent=s.send(data)
                            except:
                                sent=0
                            data=data[sent:]#remove the sent portion...
                            if len(data)==0:
                                if len(self.writeDict[s])>0:
                                    data=self.writeDict[s].pop(0)
                        if len(data)>0:#put back unsent data
                            self.writeDict[s].insert(0,data)
        if self.verbose:
            print "Loop ended in SockConn.py"
    def close(self,sock):
        """Close socket and stop listening on it
        @param sock: Socket
        @type sock: socket.socket instance
        """
        if type(sock)==type(0):
            try:
                os.close(sock)
            except:#closing failed... probably already closed.
                pass
        else:
            sock.close()
        try:
            self.selIn.remove(sock)
        except:
            pass
        if self.writeDict.has_key(sock):
            del(self.writeDict[sock])
        print "Closed ",sock

    def readsock(self,sock):
        """Reads the socket, obtaining 'data'.
        data.action will be a control word, e.g. 'cmd'
        If data.action=='now':
        data.command will be command to be exec'd.
        data.ret (if present) will be list of things to return.
        If data.action=='rpt':#repeat command
        data.command will be command to be exec'd.
        data.ret (if present) will be list of things to return.
        If data.action=='cmd':#command to execute during break...
        data.command will be command to be exec'd.
        data.ret (if present) will be list of things to return.
        If data.action=='del':#delete command from regular/cmd list
        this will be deleted...
        @param sock: Socket to read
        @type sock: socket.socket instance
        """
        try:
            tmp=serialise.ReadMessage(sock)
        except:
            print "Connection reset by peer."
            return -1
        if not tmp:
            return -1 #connection closed.
        #print "socketdata:",tmp
        data=ConnObj.ConnMsg(None,None)
        data.unpickle(tmp)
        #data=cPickle.loads(data[0])#unpickle a ConnMsg object.
        action=data.action
        tag=data.tag
        remoteData=data.remoteData
        print "got data: %s %s %s %s"%(str(data.action),str(data.command),str(data.tag),str(data.ret))
        #cmd=data.pop(0)
        if action=="now":
            if self.globals==None:
                self.cmdList.append([data,sock])
            else:
                self.execCmd(data,sock)
        elif action=="cmd":
            self.cmdList.append([data,sock])
        elif action[:3]=="rpt":
            freq=1
            if len(action)>3:
                freq=int(action[3:])
                if freq<1:
                    freq=1
            self.rptCmdList.append([data,sock,freq,0])
        elif action=="del":
            remlist=[]
            #print "Action del:",data,sock
            for cmd in self.rptCmdList:
                #print cmd
                try:
                    if (data.command=="" or data.command==None or cmd[0].command==data.command) and sock==cmd[1] and (tag==None or cmd[0].tag==tag):
                        #if cmd[:2]==[data,sock]:
                        remlist.append(cmd)
                except:
                    print "error deleting",data,sock,cmd
                    print data.command
            for cmd in remlist:
                print "Deleting action",cmd
                self.rptCmdList.remove(cmd)
            while [data,sock] in self.cmdList:
                print "Deleting action:",[data,sock]
                self.cmdList.remove([data,sock])
        elif action=="add":
            #prepend data.command to config.postList...
            print "SockConn - got data action add"
            if type(data.command)==type(()) and len(data.command)==2 and type(data.command[0])==type(""):
                if type(self.globals)!=type(None):
                    print "Adding to config.postList - %s"%data.command[0]
                    self.globals["ctrl"].config.postAdd(data.command)
                    print "Added post variable %s to config.postList"%data.command[0]
                else:
                    print "Cannot add post variable %s to config, SockConn has no globals"%data.command[0]
            else:
                print "SockConn - action add not received with non-valid data, should be tuple of (str,data)."
        else:
            print action,data
            serialise.Send(["warning",tag,"data not understood"],sock)

    def execCmd(self,data,sock):
        """Execute a command.
        @param data: The command to be executed
        @type data: ConnMsg instance
        @param sock: A socket
        @type sock: socket.socket instance
        @return:  The executed value
        @rtype: User defined
        """
        #command=data[0]
        command=data.command
        tag=data.tag
        remoteData=data.remoteData
        self.globals["remoteData"]=remoteData
        self.globals["sock"]=sock
        rtval=0
##         if len(data)>1:
##             rtObjects=data[1]
##             if type(rtObjects)!=types.ListType:
##                 rtObjects=[rtObjects]
##         else:
##             rtObjects=None
        rtObjects=data.ret
        d={}#newly created variables go in here...
        if self.printmsg:
            print "Acquiring lock"
        self.lock.acquire()
        if self.printmsg:
            print "Executing",command
        try:
            exec command in self.globals,d
        except Exception,msg:
            rtval=1
            print "Command Exec failed1:",command,msg
            print str(sys.exc_info()),str(sys.exc_info()[1].args)
            #print self.globals
            if sock!=None:
                try:
                    serialise.Send(["error",tag,"Command execution failed: %s %s"%(command,msg)],sock)
                except:
                    print "Serialise failed in SockConn.execCmd - couldn't send error message"
        except:
            rtval=1
            print "Command exec failed2:",command
            if sock!=None:
                try:
                    serialise.Send(["error",tag,"Command execution failed: %s"%(command)],sock)
                except:
                    print "Serialise failed in SockConn.execCmd - couldn't send error message"
        else:
            rt={}#will return rt to the user.
            l=0
            if rtObjects==None:#return all objects created...
                rtObjects=d.keys()
            elif type(rtObjects) not in [types.ListType,types.TupleType]:
                rtObjects=[rtObjects]
            for key in rtObjects:
                if d.has_key(key):
                    rt[key]=d[key]
                    l+=1
                elif key not in [""," "]:
                    print "key",key,"not found"
            #sock.send(serialise.Serialise(["data",rt]))
            if l>0:#don't reply if nothing to reply with!
                if sock!=None:
                    if self.printmsg:
                        print "Sending data over socket"
                    try:
                        serialise.Send(["data",tag,rt],sock)
                    except:
                        rtval=1
                        print "Serialise failed in SockConn.execCmd for tag %s with keys %s (error: %s %s)"%(str(tag),str(rt.keys()),str(sys.exc_info()),str(sys.exc_info()[1].args))
        self.lock.release()

        return rtval
    def doCmdList(self,thisiter):
        """This will be called at the end of every next() iteration, and executes everything waiting to be executed.
        @param thisiter:  Iteration number
        @type thisiter: Int
        """
        rlist=[]
        if self.globals!=None:
            for data,sock in self.cmdList:#only execute this once...
                self.execCmd(data,sock)
                rlist.append([data,sock])
            for r in rlist:#...so now remove from the list.
                try:
                    self.cmdList.remove(r)
                except:
                    pass
            rlist=[]
            for data in self.rptCmdList:#repeat these continually...
                freq=data[2]
                nextiter=data[3]
                if nextiter<=thisiter:
                    if self.execCmd(data[0],data[1])==1:#failed
                        rlist.append(data)
                    if freq>=0:
                        data[3]=thisiter+freq
                    else:#remove it from the list.
                        rlist.append(data)
            for r in rlist:
                try:
                    self.rptCmdList.remove(r)
                except:
                    pass
    def printList(self):
        """Prints the command list"""
        print "rpt command list:"
        for r in self.rptCmdList:
            print r


if __name__=="__main__":
    """Testing function"""
    import sys
    if len(sys.argv)==1:
        s=SockConn(9000)
        s.loop()
    elif sys.argv[1]=="f":
        s=SockConn(8000,fwd=(socket.gethostname(),9000))
        s.loop()

