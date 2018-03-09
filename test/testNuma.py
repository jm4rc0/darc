#Currently works with configcamfile.py, started with:
#darccontrol conf/configcamfile.py -o -N 1024*1024
import sys
import numpy
import numa
import darc

def setNuma():
    """Without subapAllocation... whole rmx on each numa node"""
    d=darc.Control()
    nthr=d.Get("ncamThreads").sum()
    nnodes=numa.get_max_node()+1
    #specify which numa nodes the threads are closest to:
    threadToNuma=numpy.zeros(nthr,numpy.int32)#all node 0 for now...!
    
    rmx=d.Get("gainReconmxT")
    d.Set("threadToNuma",threadToNuma)
    for i in range(nnodes):
        d.SetNuma("gainReconmxT%d"%i,rmx,i)
        

def setNuma2():
    """With subapAllocation: partial rmx on each numa node"""
    d=darc.Control()
    nthr=d.Get("ncamThreads").sum()
    nnodes=numa.get_max_node()+1
    #specify which numa nodes the threads are closest to:
    threadToNuma=numpy.zeros(nthr,numpy.int32)#all node 0 for now...!

    sf=d.Get("subapFlag")
    nsub=sf.size
    sa=numpy.zeros(nsub,numpy.int32)
    #divide all threads equally...
    sa[:]=nthr-1
    start=0
    for i in range(nthr):
        end=start+nsub/nthr
        sa[start:end]=i
        start=end
    print numpy.reshape(sa,(7,7))
    print numpy.reshape(sf,(7,7))
    
    d.Set("threadToNuma",threadToNuma,swap=1)
    rmx=d.Get("gainReconmxT")#nslope,nact
    thrSubapCnt=numpy.zeros(nthr,numpy.int32)
    rmxPart=[]
    for i in range(nthr):
        rmxPart.append([])
    indx=0
    for i in range(nsub):
        if sf[i]:
            thrSubapCnt[sa[i]]+=1
            rmxPart[sa[i]].append(rmx[indx])
            rmxPart[sa[i]].append(rmx[indx+1])
            indx+=2
    for i in range(nthr):
        r=numpy.array(rmxPart[i])
        print "Thread %d rmx shape %s, dtype %s"%(i,str(r.shape),r.dtype)
        d.SetNuma("gainReconmxT%d"%i,r,int(threadToNuma[i]),swap=1)
    d.Set("subapAllocation",sa,swap=1)

        
if __name__=="__main__":
    if len(sys.argv)==1:
        setNuma()
    else:
        setNuma2()
