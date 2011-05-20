/*
Here, MVM for a specific defined size.  Peak performance allows about 425Hz, compared with cublas which gives 350Hz.  Bandwidth 100GB/s, with theoretical max of 144GB/s - so 70% of max.
*/
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
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

//use NUMTHREADS 224 for 3264, 416 for 10864, 160 for 2186
#define N 10864//3264//2186//5432//10864 //number of slopes 2x2x2x2x7x97
#define M  5672 //number of actuators 2x2x2x709

//#define USEBLOCK
#ifdef USEBLOCK
#define NCEN 32//number of slopes for a block to process 
#define NACT 32//number of acts for a block to partially process
#define Mmod (((M+NACT-1)/NACT)*NACT)
#define Nmod (((N+NCEN-1)/NCEN)*NCEN)
#define MBLOCK (Mmod/NACT)
#define NBLOCK (Nmod/NCEN)
#else
#define NUMTHREADS 416
#define ACTSPERTHREAD 1 //must be power of 2...
#define numBlocks ((((M+NUMTHREADS-1)/NUMTHREADS)+ACTSPERTHREAD-1)/ACTSPERTHREAD)
#define Mmod (numBlocks*NUMTHREADS*ACTSPERTHREAD)    //have to pad to multiple of NUMTHREADS so that 
#define Nmod N
#endif



 //__constant__ float constX[N];
#ifndef USEBLOCK
//kernel definition
 /*
__global__ void sgemvKernel1(float *a, float *x, float *y){
  //100Hz with NUMTHREADS=8
  int indx=threadIdx.x+blockIdx.x*NUMTHREADS;
  int i;
  float tmp=0;
  float *A;
  //first copy x to shared memory?  ~40kB...  Or register memory?
  //Then think about memory access of A. - should A be transposed?
  //Have each thread to multiple actuators
  //adjust shm size
  //Use float4 parameter.
  A=&a[N*indx];
#pragma unroll 112//note 112 is a divisor of 10864 (no of slopes)
  for(i=0;i<N;i++){
    tmp+=x[i]*A[i];
  }
  y[indx]=tmp;
}


__global__ void sgemvKernel2(float *a, float *x, float *y){
  //This is transposed.
  //306Hz with NUMTHREADS=64
  int indx=threadIdx.x+blockIdx.x*NUMTHREADS;
  int i;
  float tmp=0;
  float *A;
  int j=0;
  //This has A transposed
  A=&a[indx];
#pragma unroll 388//note 112 is a divisor of 10864 (no of slopes)
  for(i=0;i<N;i++){
    tmp+=x[i]*A[j];
    j+=M;
  }
  y[indx]=tmp;
  }*/


__global__ void sgemvKernel(const float* __restrict__ a,const float* __restrict__ x,float* __restrict__ y){
  //This is transposed.  And copys x to shm
  //425Hz with NUMTHREADS=416
  int indx=threadIdx.x+blockIdx.x*NUMTHREADS;
  int i;
  float tmp=0;
  const float *A;
  int j=0;
  __shared__ float xs[N];
  //copy x into shm
#pragma unroll
  for(i=threadIdx.x;i<N;i+=blockDim.x)
    xs[i]=x[i];
  
  __syncthreads();//not sure this is needed (results are correct without it) - but I think in theory it is...
  //This has A transposed
  A=&a[indx];
#pragma unroll 776//note 112 is a divisor of 10864 (no of slopes):2x2x2x2x7x97
  for(i=0;i<N;i++){
    tmp+=xs[i]*A[j];
    j+=M;
  }
  y[indx]=tmp;
}

__global__ void sgemvKernel8(const float* __restrict__ a,const float* __restrict__ x,float* __restrict__ y){
  //This is transposed.  And copys x to shm
  //And does 4 actuators per thread
  //418Hz with NUMTHREADS=448
  int indx=(threadIdx.x+blockIdx.x*NUMTHREADS)*ACTSPERTHREAD;
  int i;
  float tmp1=0,tmp2=0,tmp3=0,tmp4=0;
  const float *A;
  int j=0;
  __shared__ float xs[N];
  float xx;
  //copy x into shm
#pragma unroll
  for(i=threadIdx.x;i<N;i+=blockDim.x)
    xs[i]=x[i];
  
  // __syncthreads();
  //This has A transposed
  A=&a[indx];
#pragma unroll 388//note 112 is a divisor of 10864 (no of slopes):2x2x2x2x7x97
  for(i=0;i<N;i++){
    xx=xs[i];
    tmp1+=xx*A[j];
    tmp2+=xx*A[j+1];
    tmp3+=xx*A[j+2];
    tmp4+=xx*A[j+3];
    j+=M;
  }
  y[indx]=tmp1;
  y[indx+1]=tmp2;
  y[indx+2]=tmp3;
  y[indx+3]=tmp4;
}


__global__ void sgemvKernel7(float *a, float *x, float *y){
  //This is transposed.  And copys x to shm
  //401Hz with NUMTHREADS=448
  int indx=threadIdx.x+blockIdx.x*NUMTHREADS;
  int i;
  float xs1,xs2,xs3,xs4;
  float tmp1=0,tmp2=0,tmp3=0,tmp4=0;
  float *A;
  int j=0;
  __shared__ float xs[N];
  //copy x into shm
#pragma unroll
  for(i=threadIdx.x;i<N;i+=blockDim.x)
    xs[i]=x[i];
  
  // __syncthreads();
  //This has A transposed
  A=&a[indx];
#pragma unroll 181//note 112 is a divisor of 10864 (no of slopes):2x2x2x2x7x97
  for(i=0;i<=N-4;i+=4){
    xs1=xs[i];
    xs2=xs[i+1];
    xs3=xs[i+2];
    xs4=xs[i+3];
    tmp1+=xs1*A[j];
    j+=M;
    tmp2+=xs2*A[j];
    j+=M;
    tmp3+=xs3*A[j];
    j+=M;
    tmp4+=xs4*A[j];
    j+=M;
  }
  /*
#pragma unroll
  for(;i<N;){
    tmp1+=xs[i++]*A[j];
    j+=M;
    }*/
  y[indx]=tmp1+tmp2+tmp3+tmp4;
}


#endif
/*
__global__ void sgemvKernel7(float *a, float *y){
  //This is transposed.  Uses x in constant memory
  //388Hz with NUMTHREADS=512
  int indx=threadIdx.x+blockIdx.x*NUMTHREADS;
  int i;
  float tmp=0;
  float *A;
  int j=0;
  __shared__ float xs[N];
  //This has A transposed
  A=&a[indx];
#pragma unroll 388//note 112 is a divisor of 10864 (no of slopes):2x2x2x2x7x97
  for(i=0;i<N;i++){
    tmp+=constX[i]*A[j];
    j+=M;
  }
  y[indx]=tmp;
}
*/
#ifdef USEBLOCK
__global__ void sgemvKernel(float *a,float *x,float*ypartial){
  //This one, each thread partially computes a few actuators using a few slopes.
  __shared__ float tile[NACT*NCEN];
  __shared__ float X[NCEN];
  float *A=&a[(blockIdx.y*NACT)+(blockIdx.x*NCEN)*Mmod+threadIdx.x];
  float sum=0.;
  float *x0=&x[blockIdx.x*N/gridDim.x];
  int i;
  //Mmod and Nmod are M and N increased so that they are divisible by NACT/NCEN.
  //y has shape Mmod*Nmod/NCEN
  //Note - blockDim.x==NACT, blockDim.y==1 (so threadIdx.y=0)
  //Problem is then divided up into blocks with gridDim.x,y=Nmod/NCEN,Mmod/NACT
  //first copy part of the matrix and part-vector to shm
  for(i=threadIdx.x;i<NCEN;i+=blockDim.x)
    X[i]=x0[i];
#pragma unroll
  for(i=0;i<NCEN;i++)
    tile[i*NACT+threadIdx.x]=A[i*Mmod];
  //now do the partial mvm
#pragma unroll
  for(i=0;i<NCEN;i++)
    sum+=tile[threadIdx.x+i*NACT]*X[i];
  ypartial[(threadIdx.x+NACT*blockIdx.y)+blockIdx.x*Mmod]=sum;
}
//Now need (as part of previous kernel) to sum the ypartials.
__global__ void sgemvSumKernel(float *ypartial,float *y){
  int i;//
  y[threadIdx.x+blockIdx.x*blockDim.x]=0;
  for(i=0;i<Nmod/NCEN;i++)
    y[threadIdx.x+blockIdx.x*blockDim.x]+=ypartial[threadIdx.x+blockIdx.x*blockDim.x+Mmod*i];

}
#endif

#ifndef USEBLOCK
/*
__global__ void sgemvKernel3(float *a, float *x, float *y){
  //if iterate over ACTSPERTHREAD first best with actsPerThread==1 - (so same as kernel2)
  //But if iterate over N first, performance is reduced.
  //So, need to try with shared memory
  int indx=(threadIdx.x+blockIdx.x*NUMTHREADS)*ACTSPERTHREAD;
  int i;
  //float tmp=0;
  float tmp[ACTSPERTHREAD];
  float *A;
  int pos=0;
  int j;
  //#pragma unroll
  //for(j=0;j<ACTSPERTHREAD;j++){
    //This has A transposed
    A=&a[indx];
#pragma unroll 388//note 112 is a divisor of 10864 (no of slopes)
    for(i=0;i<N;i++){
#pragma unroll
      for(j=0;j<ACTSPERTHREAD;j++){
	tmp[j]+=x[i]*A[pos+j];
      }
      pos+=M;
    }
    for(j=0;j<ACTSPERTHREAD;j++)
      y[indx+j]=tmp[j];
    //tmp=0;
    //indx++;
    //pos=0;
    //}
}
__global__ void sgemvKernel5(float *a, float *x, float *y){
  //if iterate over ACTSPERTHREAD first best with actsPerThread==1 - (so same as kernel2)
  //So, need to try with shared memory
  int indx=(threadIdx.x+blockIdx.x*NUMTHREADS)*ACTSPERTHREAD;
  int i;
  float tmp=0;
  float *A;
  int pos=0;
  int j;
  __shared__ float xs[N];
  //copy x into shm
  for(i=threadIdx.x;i<N;i+=blockDim.x)
    for(j=0;j<ACTSPERTHREAD;j++)
      xs[i+j]=x[i+j];
  #pragma unroll
  for(j=0;j<ACTSPERTHREAD;j++){
    //This has A transposed
    A=&a[indx];
#pragma unroll 388//note 112 is a divisor of 10864 (no of slopes)
    for(i=0;i<N;i++){
      tmp+=xs[i]*A[pos];
      pos+=M;
    }
    y[indx]=tmp;
    tmp=0;
    indx++;
    pos=0;
  }
  }*/
#endif
  
void cmvm1(float *a,float *x,float *y){
  //non-transposed - use with 1
  int i,j;
  for(i=0;i<M;i++){
    y[i]=0;
    for(j=0;j<N;j++){
      y[i]+=x[j]*a[i*N+j];
    }
  }
}
void cmvm(float *a,float *x,float *y){
  //transposed - use with 2
  int i,j;
  for(i=0;i<M;i++){
    y[i]=0;
    for(j=0;j<N;j++){
      y[i]+=x[j]*a[i+j*M];
    }
  }
}


void doMvm(void){
  float *a,*Da,*x,*Dx,*y,*Dy,*cy;
#ifdef USEBLOCK
  float *Dypartial;
#endif
  int i,j,err=0;
  struct timeval t1,t2;
  double t;
#ifdef USEBLOCK
  dim3 numBlocks(NBLOCK,MBLOCK);
#endif
  printf("Calculation size %dx%d\n",N,Mmod);
  a=(float*)malloc(sizeof(float)*M*N);
  x=(float*)malloc(sizeof(float)*N);
  y=(float*)malloc(sizeof(float)*M);
  cy=(float*)malloc(sizeof(float)*M);
  cudaMalloc(&Da,sizeof(float)*Mmod*N);
  cudaMalloc(&Dx,sizeof(float)*N);
  cudaMalloc(&Dy,sizeof(float)*Mmod);
#ifdef USEBLOCK
  cudaMalloc(&Dypartial,sizeof(float)*Mmod*NBLOCK);
  StillNeedToSortOutMatrixCopyForThisCase();
#endif
  memset(y,0,sizeof(float)*M);
  cudaMemset(Dy,0,sizeof(float)*M);
  for(i=0;i<N;i++){
    x[i]=rand()/(float)RAND_MAX;
    for(j=0;j<M;j++){
      a[i*M+j]=rand()/(float)RAND_MAX;
    }
  }

  //Now copy the data in.
  cudaMemcpy(Da,a,sizeof(float)*M*N,cudaMemcpyHostToDevice);
  cudaMemcpy(Dx,x,sizeof(float)*N,cudaMemcpyHostToDevice);
  //cudaMemcpyToSymbol(constX,x,sizeof(float)*N);
  cudaThreadSynchronize();

  printf("Calling kernel\n");
#ifdef USEBLOCK
  sgemvKernel<<<numBlocks,NACT>>>(Da,Dx,Dypartial);
  sgemvSumKernel<<<MBLOCK,NACT>>>(Dy,Dypartial);
#else
  sgemvKernel<<<numBlocks,NUMTHREADS>>>(Da,Dx,Dy);
#endif
  cudaThreadSynchronize();
  gettimeofday(&t1,NULL);
  //a max of 1024 threads (same processor core).
  for(i=0;i<10;i++){
#ifdef USEBLOCK
    sgemvKernel<<<numBlocks,NACT>>>(Da,Dx,Dypartial);
    sgemvSumKernel<<<MBLOCK,NACT>>>(Dy,Dypartial);
#else
    sgemvKernel<<<numBlocks,NUMTHREADS>>>(Da,Dx,Dy);
#endif
    cudaThreadSynchronize();
  }
  gettimeofday(&t2,NULL);
  t=(t2.tv_sec-t1.tv_sec+1e-6*(t2.tv_usec-t1.tv_usec))/10;
  //Now copy data back.
  cudaMemcpy(y,Dy,sizeof(float)*M,cudaMemcpyDeviceToHost);
  //And now compare results.
  cudaThreadSynchronize();
  cmvm(a,x,cy);
  err=0;
  for(i=0;i<M;i++){
    if((cy[i]==0 && y[i]!=0) || fabsf((cy[i]-y[i])/cy[i])>1e-5){
      printf("%d %g %g\n",i,cy[i],y[i]);
      err++;
    }
  }
  i--;
  printf("%d %g %g\n",i,cy[i],y[i]);
  printf("err: %d\n",err);
  printf("kernel done in %gs (%gHz): %g GB/s (%g effective)\n",t,1/t,N*(Mmod+Mmod/NUMTHREADS)*4/t/1073741824,N*(M+1)*4/t/1073741824);//true rate includes the full (padded) matrix, and loading vector from global to shared memory a number of times.  Effective rate is unpadded matrix and single loading of vector.
}
int main(void){
  int deviceCount=0;
  int devID=0;
  int major, minor;
  char deviceName[100];
  if(cuInit(0)!=CUDA_SUCCESS){
    printf("cuInit error\n");
    exit(1);
  }
  cuDeviceGetCount(&deviceCount);
  if(deviceCount==0){
    printf("There is no device supporting CUDA.\n");
    exit(0);
  }else{
    printf("Found %d devices\n",deviceCount);
  }
  cuDeviceComputeCapability(&major, &minor, devID);
  cuDeviceGetName(deviceName, 256, devID);
  printf("Using Device %d: \"%s\" with Compute %d.%d capability\n", devID, deviceName, major, minor);
  
  CUdevice cuDevice;
  if(cuDeviceGet(&cuDevice,devID)!=CUDA_SUCCESS)
    printf("ERror cuDeviceGet\n");
  /*
  printf("Creating context\n");
  CUcontext cuContext;
  if(cuCtxCreate(&cuContext,0,cuDevice)!=CUDA_SUCCESS)
    printf("Error cuCtxCreate\n");
  CUmodule cuModule;
  printf("Loading module\n");
  if(cuModuleLoad(&cuModule,"gpumvm.ptx")!=CUDA_SUCCESS)
    printf("cuModuleLoad error\n");
  CUfunction sgemvFunc;
  printf("Getting function\n");
  if(cuModuleGetFunction(&sgemvFunc,cuModule,"sgemvKernel")!=CUDA_SUCCESS)
  printf("cuModuleGetFunction error\n");*/
  //continue...
  //  calibrateHost();
  //doesn't seem to make a difference cuCtxSetCacheConfig(CU_FUNC_CACHE_PREFER_SHARED);
  printf("Calling domvm\n");
  doMvm();
  return 0;
}
