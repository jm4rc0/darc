#Currently works with configScao80Numa.py, started with:
#darccontrol configScao80Numa.py -o -b 512*1024*1024 -N 512*1024*1024
import sys
import numpy
import numa
import darc


def setNuma(case="gig57"):
    """With subapAllocation: partial rmx on each numa node"""
    d=darc.Control()
    nthr=d.Get("ncamThreads").sum()
    nnodes=numa.get_max_node()+1
    #specify which numa nodes the threads are closest to:
    threadToNuma=numpy.zeros(nthr,numpy.int32)#all node 0 for now...!
    #Use numactl --hardware to get some of this information... however, note this isn't foolproof, e.g. for the Xeon Phi and other systems with specialised memory.
    #Also assumes that the threadAffinity in the config file is set to correspond to this as well.

    #gig57:
    if case=="gig57":
        if nthr!=16:
            raise Exception("Expecting 16 threads")
        threadToNuma[:8]=0
        threadToNuma[8:]=1
    elif case=="EPYC":
        if nthr!=32:
            raise Exception("Expecting 32 threads")
        threadToNuma[:]=numpy.arange(nthr)//4

    sf=d.Get("subapFlag")
    nsub=sf.size
    sa=d.Get("subapAllocation")
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
    d.Set("threadToNuma",threadToNuma,swap=1)


if __name__=="__main__":
    case=sys.argv[1]#e.g. gig57, EPYC, ...
    setNuma(case)