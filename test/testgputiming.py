import os
import sys
nact=5672
ncen=10864
start=1
end=1025
if len(sys.argv)>1:
    ncen=int(sys.argv[1])
if len(sys.argv)>2:
    nact=int(sys.argv[2])
if len(sys.argv)>3:
    start=int(sys.argv[3])
if len(sys.argv)>4:
    end=int(sys.argv[4])
if end>1025:
    end=1025
if start<1:
    start=1
for i in range(start,end):
    print i
    os.system("./gpumvmgen %d %d %d -q >> gpumvmgentime%dx%d.csv"%(nact,ncen,i,nact,ncen))
