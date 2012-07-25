import threading
import traceback
"""TODO:
A new way of stopping threads:
ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(t.ident),ctypes.py_object(ZeroDivisionError))

Or whatever error you like.
t is the thread object.

May not stop straight away, but will eventually.
"""


class QO:
    def __init__(self,tag,func,args,retfunc=None,cancelfunc=None,cancelargs=(),asThread=1,statusfunc=None,statusargs=()):
        self.tag=tag
        self.func=func
        self.args=args
        self.retfunc=retfunc
        self.cancelfunc=cancelfunc
        self.cancelargs=cancelargs
        self.asThread=asThread
        self.statusfunc=statusfunc
        self.statusargs=statusargs
    def __repr__(self):
        txt=str(self.func)+" tag %d"%self.tag
        if self.retfunc!=None:
            txt+=" return function %s"%str(self.retfunc)
        if self.cancelfunc!=None:
            txt+=" cancel function %s"%str(self.cancelfunc)
        if self.statusfunc!=None:
            txt+=" status function %s"%str(self.statusfunc)
            try:
                tmp=str(self.statusfunc(*self.statusargs))
            except:
                tmp="status func failed:\n%s"%traceback.format_exc()
            if len(tmp)>0:
                txt+="\n"+tmp
        return txt

class jobsched:
    def __init__(self):
        self.queueDict={}
        self.threadDict={}
        self.lock=threading.Lock()
        self.tag=1<<30

    def queue(self,func,args=(),retfunc=None,cancelfunc=None,cancelargs=(),asThread=1,queue="default",statusfunc=None,statusargs=()):
        """statusfunc returns a text string status when called
        cancelfunc can be called to tell the function to cancel itself
        """
        self.lock.acquire()
        self.tag+=1
        try:
            if not self.queueDict.has_key(queue):
                self.queueDict[queue]=[]
            self.queueDict[queue].append(QO(self.tag,func,args,retfunc,cancelfunc,cancelargs,asThread,statusfunc,statusargs))
            if len(self.queueDict[queue])==1:#nothing in queue, so start thread
                if self.threadDict.has_key(queue):
                    self.threadDict[queue].join()
                t=threading.Thread(target=self.processQueue,args=(queue,))
                t.daemon=1 
                t.start()
                self.threadDict[queue]=t
                
                
        except:
            self.lock.release()
            raise
        self.lock.release()
        return self.tag

    def processQueue(self,queue="default"):
        self.lock.acquire()
        try:
            while len(self.queueDict.get(queue,[]))>0:
                qObj=self.queueDict[queue][0]
                #release the lock and then run it...
                self.lock.release()
                try:
                    rt=qObj.func(*qObj.args)
                    if qObj.retfunc!=None:
                        qObj.retfunc(rt)
                except:
                    traceback.print_exc()
                    print "Error processing function in processQueue"
                self.lock.acquire()
                self.queueDict[queue].pop(0)
        except:
            self.lock.release()
            raise
        self.lock.release()
    def cancel(self,queue="default",tag=None):
        self.lock.acquire()
        try:
            if len(self.queueDict.get(queue,[]))>0:
                #there are jobs running...
                if tag==None or tag==self.queueDict[queue][0].tag:
                    # cancel the running job
                    if self.queueDict[queue][0].cancelfunc!=None:
                        cf=self.queueDict[queue][0].cancelfunc
                        ca=self.queueDict[queue][0].cancelargs
                        self.lock.release()
                        try:
                            cf(*ca)
                        except:
                            traceback.print_exc()
                            print "Cancellation function failed"
                        self.lock.acquire()
                    else:
                        print "No cancellation point"
                else:
                    #find the job...
                    for i in range(1,len(self.queueDict[queue])):
                        if self.queueDict[queue][i].tag==tag:
                            self.queueDict[queue].pop(i)
                            break
                    if i==len(self.queueDict[queue]):
                        raise Exception("Tag %d not found"%tag)
        except:
            self.lock.release()
            raise
        self.lock.release()
