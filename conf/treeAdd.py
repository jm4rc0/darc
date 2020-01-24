import numpy

def treeAddInit(nthreads,groups=[2],addtype=0):
    ngroups = len(groups)
    nthreads1 = int(nthreads)
    nthreads = nthreads1
    tmp = nthreads
    cnt=0
    while(1):
        tmp = (tmp+groups[min(cnt,ngroups-1)]-1)/groups[min(cnt,ngroups-1)]
        cnt+=1
        if tmp==1:
            break
    printDEBUG("count = ",cnt)
    steps=cnt
    partNos = numpy.zeros((steps+1,nthreads),dtype='i')-2
    threadnos = numpy.arange(nthreads)
    nparts1 = (nthreads+groups[0]-1)//groups[0]
    nparts = nparts1
    i=0
    step=1
    start=0
    if addtype==0:
        while(i<steps):
            partNos[i,:nthreads] = threadnos[:nthreads]%nparts
            # partNos[i,start:] = threadnos[:nthreads]%nparts
            nthreads=nparts
            start+=nparts*(groups[min(i,ngroups-1)]-1)
            i+=1
            nparts = (nthreads+groups[min(i,ngroups-1)]-1)//groups[min(i,ngroups-1)]
    else:
        while(i<steps):
            partNos[i,::step] = threadnos[:nthreads]//groups[min(i,ngroups-1)]
            step*=groups[min(i,ngroups-1)]
            nthreads=nparts
            i+=1
            nparts = (nthreads+groups[min(i,ngroups-1)]-1)//groups[min(i,ngroups-1)]

    partNos[-1,0]=-1

    printDEBUG(partNos)

    barrierWaits = numpy.zeros(nthreads1,dtype='i')
    owners = numpy.zeros(nparts1,dtype='i')
    for i in range(nparts1):
        idx = numpy.where(partNos[0]==i)[0]
        owners[i]=idx[0]
        barrierWaits[idx[0]]=len(idx)
    for j in range(1,steps):
        for i in range(nparts1):
            idx = numpy.where(partNos[j]==i)[0]
            if len(idx)<=0:
                continue
            barrierWaits[owners[i]]+=(len(idx)-1)

    partArray = numpy.zeros(nthreads1*2,dtype='i')
    partArray[:nthreads1] = partNos[0,:]
    for i in range(nparts1):
        last_row = numpy.where(partNos==i)[0][-1]
        first_col = numpy.where(partNos[last_row]==i)[0][0]
        partArray[nthreads1+owners[i]] = partNos[last_row+1][first_col]
    partArray[nthreads1]=0

    barrierWaits[0]=0
    return partArray,barrierWaits,nparts1

def getTreeParams(nthreads,groups=[2],addtype=0):
    if nthreads<5:
        partArray=numpy.zeros(nthreads*2,dtype='i')
        barrierWaits=numpy.zeros(nthreads,dtype='i')
        nparts = 1
    else:
        partArray,barrierWaits,nparts=treeAddInit(nthreads,groups,addtype)

    printDEBUG(barrierWaits)

    print(nparts)

    for arr in [range(nthreads),partArray[:nthreads],barrierWaits,partArray[nthreads:]]:
        printDEBUG('[%s]' % (' '.join('%02s' % j for j in arr)))

    return {"treePartArray":partArray,
        "treeBarrierWaits":barrierWaits,
        "treeNparts":nparts,
        }

if __name__ =="__main__":
    getTreeParams(54,[3,2,3],0)
    getTreeParams(6,[2],0)

    nthreads = 64

    if nthreads==48:
        # treeAdd = getTreeParams(nthreads,[3,2,2,2,2],0)
        # treeAdd = getTreeParams(nthreads,[4,3,2,2],0)
        # treeAdd = getTreeParams(nthreads,[4,4,3],0)
        treeAdd = getTreeParams(nthreads,[6,4,2],0)
    elif nthreads==49:
        treeAdd = getTreeParams(nthreads,[7,7],0)
    elif nthreads==50:
        treeAdd = getTreeParams(nthreads,[5,5,2],0)
    elif nthreads==54:
    #     treeAdd = getTreeParams(nthreads,[3,2,3],0)
        treeAdd = getTreeParams(nthreads,[3,3,3,2],0)
        # treeAdd = getTreeParams(nthreads,[6,3,3],0)
        # treeAdd = getTreeParams(nthreads,[9,3,2],0)
    elif nthreads==56:
        treeAdd = getTreeParams(nthreads,[7,2,2,2],0)
        # treeAdd = getTreeParams(nthreads,[7,4,2],0)
    elif nthreads==60:
        # treeAdd = getTreeParams(nthreads,[5,3,2,2],0)
        # treeAdd = getTreeParams(nthreads,[5,4,3],0)
        treeAdd = getTreeParams(nthreads,[6,5,2],0)
    else:
        treeAdd = getTreeParams(nthreads,[2],0)
    # treeAdd = getTreeParams(nthreads,[nthreads//2],0)
    # treeAdd["treeNlayers"]=1
    # treeAdd["treeNparts"]=1
    # treeAdd={}
    control = treeAdd
    control['treeNparts'] = 0 # set this to turn off treeAdd
    control.update({
        'other_keys':"other_vals",
        })