/*
Ideas here for a gpu calibration module (which would also do slope measurement and reconstruction)

Work is split according to:
number of subaps processed together (e.g. a row of subaps)
Each subaperture is then divided into 
ny x nx threads which each do a portion of the calibration.  

Should I use cuda or opencl?  Probably opencl - for other device compatibility.

But - cuda seems to give better performance.

*/

typedef struct{
  int *subapLocation;
  int cursubindx;
  int *subapFlag;
  int thresholdAlgo;
  float *calthr;
  float *calsub;
  float *calmult;
  float *subap;
}CalStruct;

//kernel definition
__global__ void calibrateKernel(CalStruct *cstr){
  //nthreads - total number of threads running the problem.
  //nsubaps - the number of subaps shared between these threads.
  int i;
  int nsubaps=blockDim.z;
  
  int subno=threadIdx.z;//get_local_id(0);//threadid[0];//the subap that this thread is processing.
  int yindx=threadIdx.y;//get_local_id(1);//threadid[1];//the start index
  int nthreads=blockDim.y * blockDim.x *blockDim.z;
  int xindx=threadIdx.x;//get_local_id(1);//threadid[2];//x start index.
  int *loc=&(cstr->subapLocation[(subno+cstr->cursubindx)*6]);
  //int thrPerSubap=nthreads/nsubaps;
  float *calthr=cstr->calthr;
  float *calsub=cstr->calsub;
  float *calmult=cstr->calmult;
  int cnt;
  int pos;
  int pos2;
  int npxlx=cstr->npxlx;
  int npxlCum=cstr->npxlCum;
  int yrowPerThr=((loc[1]-loc[0])/loc[2]+blockDim.y-1)/blockDim.y;
  int xcolPerThr=((loc[4]-loc[3])/loc[5]+blockDim.x-1)/blockDim.x;
  int j;
  float *subap=&cstr->subap[(subno+cstr->cursubindx)*256];
  if(cstr->subapFlag[subno+cstr->cursubindx]!=0){
    if(calmult!=NULL && calsub!=NULL){
      if((cstr->thresholdAlgo==1 || cstr->thresholdAlgo==2) && calthr!=NULL){
	for(i=loc[0]+(yrowPerThr*yindx)*loc[2]; i<loc[0]+yrowPerThr*(yindx+1)*loc[2] && i<loc[1]; i+=loc[2]){
	  cnt=(yrowPerThr*yindx+i)*npxlx+xcolPerThr*xindx;
	  pos=npxlCum+i*npxlx;
	  for(j=loc[3]+(xcolPerThr*xindx)*loc[5];j<loc[4] && j<loc[3]+xcolPerThr*(xindx+1)*loc[5]; j+=loc[5]){
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

void calibrateHost(void){
  CalStruct cstr;
  int nsubs=49;
  int npxlx=128;
  int npxly=128;
  int nsubx=7;
  int nsuby=7;
  int i,j;
  cstr=calloc(sizeof(CalStruct),1);
  cstr->cursubindx=0;
  cstr->subapLocation=calloc(sizeof(int)*6,nsubs);
  cstr->subapFlag=calloc(sizeof(int),nsubs);
  cstr->thresholdAlgo=1;
  cstr->calthr=malloc(sizeof(float)*npxlx*npxly);
  cstr->calmult=malloc(sizeof(float)*npxlx*npxly);
  cstr->calsub=malloc(sizeof(float)*npxlx*npxly);
  cstr->subap=malloc(sizeof(float*)*nsubs*256);
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
  cudaMalloc(&dcstr,sizeof(CalStruct));
  cudaMalloc(&dsubapLocation,sizeof(int)*6*nsubs);
  cudaMalloc(&dsubapFlag,sizeof(int)*nsubs);
  cudaMalloc(&dcalthr,sizeof(float)*npxlx*npxly);
  cudaMalloc(&dcalsub,sizeof(float)*npxlx*npxly);
  cudaMalloc(&dcalmult,sizeof(float)*npxlx*npxly);
  cudaMalloc(&subap,sizeof(float)*nsubs*256);
  //Now copy the data in.
  cudaMemcpy(dcalmult,calmult,sizeof(float)*npxlx*npxly,cudaMemcpyHostToDevice);
  cudaMemcpy(dcalsub,calsub,sizeof(float)*npxlx*npxly,cudaMemcpyHostToDevice);
  cudaMemcpy(dcalthr,calthr,sizeof(float)*npxlx*npxly,cudaMemcpyHostToDevice);
  cudaMemcpy(dsubapFlag,subapFlag,sizeof(int)*nsubs,cudaMemcpyHostToDevice);
  cudaMemcpy(dsubapLocation,subapLocation,sizeof(int)*6*nsubs,cudaMemcpyHostToDevice);
  //make a temporary cstr
  tcstr=malloc(sizeof(CalStruct));
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
  }
  CUdevice cuDevice;
  cuDeviceGet(&cuDevice,0);
  CuContext cuContext;
  cuCtxCreate(&cuContext,0,cuDevice);
  CUmodule cuModule;
  cuModuleLoad(&cuModule,"gpucalibrate.ptx");
  CUfunction calibrateK;
  cuModuleGetFunction(&calibrateK,cuModule,"calibrateKernel");
  //continue...
  calibrateHost();
  return 0;
}
