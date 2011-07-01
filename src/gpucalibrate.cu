/*
Ideas here for a gpu calibration module (which would also do slope measurement and reconstruction)

Work is split according to:
number of subaps processed together (e.g. a row of subaps)
Each subaperture is then divided into 
ny x nx threads which each do a portion of the calibration.  

Should I use cuda or opencl?  Probably opencl - for other device compatibility.

But - cuda seems to give better performance.

*/
#include <stdio.h>
#include <stdlib.h>
#include <cuda.h>

typedef struct{
  int *subapLocation;
  int cursubindx;
  int *subapFlag;
  int thresholdAlgo;
  float *calthr;
  float *calsub;
  float *calmult;
  float *subap;
  int npxlx;
  int npxlCum;
}CalStruct;

//kernel definition
__global__ void calibrateKernel(CalStruct *cstr){
  //nthreads - total number of threads running the problem.
  //nsubaps - the number of subaps shared between these threads.
  int i;
  //int nsubaps=blockDim.z;
  
  int subno=threadIdx.z;//get_local_id(0);//threadid[0];//the subap that this thread is processing.
  int yindx=threadIdx.y;//get_local_id(1);//threadid[1];//the start index
  //int nthreads=blockDim.y * blockDim.x *blockDim.z;
  int xindx=threadIdx.x;//get_local_id(1);//threadid[2];//x start index.
  int loc0,loc1,loc2,loc3,loc4,loc5;
  //int thrPerSubap=nthreads/nsubaps;
  float *calthr=cstr->calthr;
  float *calsub=cstr->calsub;
  float *calmult=cstr->calmult;
  int cnt;
  int pos;
  int pos2;
  int npxlx=cstr->npxlx;
  int npxlCum=cstr->npxlCum;
  int yrowPerThr;
  int xcolPerThr;
  int j;
  float *subap=&cstr->subap[(subno+cstr->cursubindx)*256];
  loc0=(cstr->subapLocation[(subno+cstr->cursubindx)*6+0]);
  loc1=(cstr->subapLocation[(subno+cstr->cursubindx)*6+1]);
  loc2=(cstr->subapLocation[(subno+cstr->cursubindx)*6+2]);
  loc3=(cstr->subapLocation[(subno+cstr->cursubindx)*6+3]);
  loc4=(cstr->subapLocation[(subno+cstr->cursubindx)*6+4]);
  loc5=(cstr->subapLocation[(subno+cstr->cursubindx)*6+5]);
  yrowPerThr=((loc1-loc0)/loc2+blockDim.y-1)/blockDim.y;
  xcolPerThr=((loc4-loc3)/loc5+blockDim.x-1)/blockDim.x;
  if(cstr->subapFlag[subno+cstr->cursubindx]!=0){
    if(calmult!=NULL && calsub!=NULL){
      if((cstr->thresholdAlgo==1 || cstr->thresholdAlgo==2) && calthr!=NULL){
	for(i=loc0+(yrowPerThr*yindx)*loc2; i<loc0+yrowPerThr*(yindx+1)*loc2 && i<loc1; i+=loc2){
	  cnt=(yrowPerThr*yindx+i)*npxlx+xcolPerThr*xindx;
	  pos=npxlCum+i*npxlx;
	  for(j=loc3+(xcolPerThr*xindx)*loc5;j<loc4 && j<loc3+xcolPerThr*(xindx+1)*loc5; j+=loc5){
	    pos2=pos+j;
	    subap[cnt]*=calmult[pos2];
	    subap[cnt]-=calsub[pos2];
	    if(subap[cnt]<calthr[pos2])
	      subap[cnt]=0;
	    cnt++;
	  }
	}
      }
    }
  }
}

/*
New thoughts (after mvm experience):
What is the best way of doing this?  Start with each thread doing 1 pixel
 - calmult, calsub, calthr.
 - then brightest pixel selection
 - then power factor.
 - then cxarr= pxl*x, cyarr=pxl*y
 - then need to sum these arrays (into shared memory)  How best to do this on a gpu?  How do GPUs handle reduce type functions?
 - then, onto slope values:  cx/=sum, cy/=sum, and this then gets put into the mvm.  

Q: Can the brightest pixel selection, and the summing be done well with one pixel per thread?  Or should we have multiple pixels per thread?  A whole subap per thread?  

Lets assume a high order ao system with at least 64x64 subaps.  Pixels for a row of subaps will probably arrive at the same time.  So, process all of these at once.  64 threads?  Or a few x 64 threads?  But, each thread would need to read its maps from a very different part of memory (non-contiguous), so memory reads would be slow.  Might be better to have one thread per pixel, and process these.  Then start a new kernel with one thread per subap which sums the pixels, computes the x and y slopes.  Then have a new kernel with one thread per actuator which does the mvm.  What would be best?  Might make a nice 4th year project.

 */

void calibrateHost(void){
  CalStruct *cstr;
  int nsubs=49;
  int npxlx=128;
  int npxly=128;
  int nsubx=7;
  //int nsuby=7;
  int i,nx,ny;
  cstr=(CalStruct *)calloc(sizeof(CalStruct),1);
  cstr->cursubindx=0;
  cstr->subapLocation=(int*)calloc(sizeof(int)*6,nsubs);
  cstr->subapFlag=(int*)calloc(sizeof(int),nsubs);
  cstr->thresholdAlgo=1;
  cstr->npxlx=npxlx;
  cstr->npxlCum=0;
  cstr->calthr=(float*)malloc(sizeof(float)*npxlx*npxly);
  cstr->calmult=(float*)malloc(sizeof(float)*npxlx*npxly);
  cstr->calsub=(float*)malloc(sizeof(float)*npxlx*npxly);
  cstr->subap=(float*)malloc(sizeof(float*)*nsubs*256);
  for(i=0; i<nsubs; i++){
    nx=i*nsubx;
    ny=i/nsubx;
    cstr->subapLocation[i*6+0]=8+ny*16;
    cstr->subapLocation[i*6+1]=8+ny*16+16;
    cstr->subapLocation[i*6+2]=1;
    cstr->subapLocation[i*6+3]=8+nx*16;
    cstr->subapLocation[i*6+4]=8+nx*16+16;
    cstr->subapLocation[i*6+5]=1;
    cstr->subapFlag[i]=1;
  }
  for(i=0; i<npxlx*npxly; i++){
    cstr->calmult[i]=1.;
    cstr->calthr[i]=10.;
    cstr->calsub[i]=2.;
  }

  //Now create the device struct and memory.
  CalStruct *dcstr;
  int *dsubapLocation;
  int *dsubapFlag;
  float *dcalthr;
  float *dcalsub;
  float *dcalmult;
  float *dsubap;

  cudaMalloc(&dcstr,sizeof(CalStruct));
  cudaMalloc(&dsubapLocation,sizeof(int)*6*nsubs);
  cudaMalloc(&dsubapFlag,sizeof(int)*nsubs);
  cudaMalloc(&dcalthr,sizeof(float)*npxlx*npxly);
  cudaMalloc(&dcalsub,sizeof(float)*npxlx*npxly);
  cudaMalloc(&dcalmult,sizeof(float)*npxlx*npxly);
  cudaMalloc(&dsubap,sizeof(float)*nsubs*256);
  //Now copy the data in.
  cudaMemcpy(dcalmult,cstr->calmult,sizeof(float)*npxlx*npxly,cudaMemcpyHostToDevice);
  cudaMemcpy(dcalsub,cstr->calsub,sizeof(float)*npxlx*npxly,cudaMemcpyHostToDevice);
  cudaMemcpy(dcalthr,cstr->calthr,sizeof(float)*npxlx*npxly,cudaMemcpyHostToDevice);
  cudaMemcpy(dsubapFlag,cstr->subapFlag,sizeof(int)*nsubs,cudaMemcpyHostToDevice);
  cudaMemcpy(dsubapLocation,cstr->subapLocation,sizeof(int)*6*nsubs,cudaMemcpyHostToDevice);
  //make a temporary cstr
  CalStruct *tcstr;
  tcstr=(CalStruct*)malloc(sizeof(CalStruct));
  memcpy(tcstr,cstr,sizeof(CalStruct));
  //insert the cuda pointers
  tcstr->calmult=dcalmult;
  tcstr->calthr=dcalthr;
  tcstr->calsub=dcalsub;
  tcstr->subapFlag=dsubapFlag;
  tcstr->subapLocation=dsubapLocation;
  tcstr->subap=dsubap;
  //Now copy this to the device
  cudaMemcpy(dcstr,tcstr,sizeof(CalStruct),cudaMemcpyHostToDevice);
  //And free the memory (wait for the copy to complete).
  cudaThreadSynchronize();
  free(tcstr);

  dim3 threadsPerBlock(16,7);//16 separate rows, 7 subaps.
  calibrateKernel<<<1,threadsPerBlock>>>(dcstr);
  //Now copy data back.
  cudaMemcpy(cstr->subap,dcstr->subap,sizeof(float)*nsubs*256,cudaMemcpyDeviceToHost);
  //And now compare results.
}
int main(void){
  int deviceCount=0;
  cuInit(0);
  cuDeviceGetCount(&deviceCount);
  if(deviceCount==0){
    printf("There is no device supporting CUDA.\n");
    exit(0);
  }else{
    printf("Found %d devices\n",deviceCount);
  }
  CUdevice cuDevice;
  cuDeviceGet(&cuDevice,0);
  CUcontext cuContext;
  cuCtxCreate(&cuContext,0,cuDevice);
  CUmodule cuModule;
  cuModuleLoad(&cuModule,"gpucalibrate.ptx");
  CUfunction calibrateK;
  cuModuleGetFunction(&calibrateK,cuModule,"calibrateKernel");
  //continue...
  calibrateHost();
  return 0;
}
