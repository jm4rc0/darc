/*
Here, MVM for user specified sizes


*/
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <cuda.h>
#include "gpumvm.h"
//use NUMTHREADS 416 for 3264, 416 for 10864, 416 for 2186
//#define N 10864//3264//2186//5432//10864 //number of slopes 2x2x2x2x7x97
//#define M  5672 //number of actuators 2x2x2x709

#define MAXN 12000//max number of slopes that can be stored in shared memory - 48kB of shared memory exist, so this is a bit conservative - could be increase slightly to 12228, but this is safer(?).

//Note, the matrix should be in row-major format... i.e. actuators continous.  Which fortunately is what darc uses. 

extern "C" __global__ void sgemvKernelMN1M101(int M,int N,const float* __restrict__ a,const float* __restrict__ x,float* __restrict__ y){
  //This is transposed.  And copys x to shm
  //425Hz with NUMTHREADS=416
  //y=A.x
  int indx=threadIdx.x+blockIdx.x*blockDim.x;
  int i;
  float tmp=0;
  const float *A;
  int j=0;
  __shared__ float xs[MAXN];
  //copy x into shm
#pragma unroll
  for(i=threadIdx.x;i<N;i+=blockDim.x)
    xs[i]=x[i];
  
  __syncthreads();//not sure this is needed (results are correct without it) - but I think in theory it is...
  //This has A transposed
  A=&a[indx];
#pragma unroll 128 //note 112 is a divisor of 10864 (no of slopes):2x2x2x2x7x97
  for(i=0;i<N;i++){
    tmp+=xs[i]*A[j];
    j+=M;
  }
  y[indx]=tmp;
}
 
extern "C" __global__ void sgemvKernelMN1M111(int M,int N,const float* __restrict__ a,const float* __restrict__ x,float* __restrict__ y){
  //This is transposed.  And copys x to shm
  //425Hz with NUMTHREADS=416
  //y+=A.x
  int indx=threadIdx.x+blockIdx.x*blockDim.x;
  int i;
  float tmp=0;
  const float *A;
  int j=0;
  __shared__ float xs[MAXN];
  //copy x into shm
#pragma unroll
  for(i=threadIdx.x;i<N;i+=blockDim.x)
    xs[i]=x[i];
  
  __syncthreads();//not sure this is needed (results are correct without it) - but I think in theory it is...
  //This has A transposed
  A=&a[indx];
#pragma unroll 128 //note 112 is a divisor of 10864 (no of slopes):2x2x2x2x7x97
  for(i=0;i<N;i++){
    tmp+=xs[i]*A[j];
    j+=M;
  }
  y[indx]+=tmp;
}
 



extern "C" int gpusgemvMN1M101(int M,int N,float *Da,float *Dx,float *Dy,int numThreads){
  //Da should be of size Mmod x N where Mmod==numThreads*((M+numThreads-1)/numThreads).  Note, padding should all be at end, not at end of each row/col.
  //Dy should be of size Mmod.
  //Dx should be of size N.
  //numThreads is probably best as 416
  //y=A.x
  int numBlocks=((M+numThreads-1)/numThreads);
  sgemvKernelMN1M101<<<numBlocks,numThreads>>>(M,N,Da,Dx,Dy);
  return 0;
}
extern "C" int gpusgemvMN1M111(int M,int N,float *Da,float *Dx,float *Dy,int numThreads){
  //Da should be of size Mmod x N where Mmod==numThreads*((M+numThreads-1)/numThreads).  Note, padding should all be at end, not at end of each row/col.
  //Dy should be of size Mmod.
  //Dx should be of size N.
  //numThreads is probably best as 416
  //y+=A.x
  int numBlocks=((M+numThreads-1)/numThreads);
  sgemvKernelMN1M111<<<numBlocks,numThreads>>>(M,N,Da,Dx,Dy);
  return 0;
}

#ifndef GPUNOMAIN
void cmvm(int M,int N,float *a,float *x,float *y){
  //transposed
  int i,j;
  for(i=0;i<M;i++){
    y[i]=0;
    for(j=0;j<N;j++){
      y[i]+=x[j]*a[i+j*M];
    }
  }
}


void doMvm(int M,int N,int numThreads,int quiet){
  float *a,*Da,*x,*Dx,*y,*Dy,*cy;
  int i,j,err=0;
  struct timeval t1,t2;
  double t;
  int numBlocks=((M+numThreads-1)/numThreads);
  int Mmod=numBlocks*numThreads;
  if(quiet==0)
    printf("Calculation size %dx%d\n",N,Mmod);
  a=(float*)malloc(sizeof(float)*M*N);
  x=(float*)malloc(sizeof(float)*N);
  y=(float*)malloc(sizeof(float)*M);
  cy=(float*)malloc(sizeof(float)*M);
  cudaMalloc(&Da,sizeof(float)*Mmod*N);
  cudaMalloc(&Dx,sizeof(float)*N);
  cudaMalloc(&Dy,sizeof(float)*Mmod);
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
  if(quiet==0)
    printf("Calling kernel\n");
  sgemvKernelMN1M101<<<numBlocks,numThreads>>>(M,N,Da,Dx,Dy);
  cudaThreadSynchronize();
  gettimeofday(&t1,NULL);
  //a max of 1024 threads (same processor core).
  for(i=0;i<10;i++){
    sgemvKernelMN1M101<<<numBlocks,numThreads>>>(M,N,Da,Dx,Dy);
    cudaThreadSynchronize();
  }
  gettimeofday(&t2,NULL);
  t=(t2.tv_sec-t1.tv_sec+1e-6*(t2.tv_usec-t1.tv_usec))/10;
  //Now copy data back.
  cudaMemcpy(y,Dy,sizeof(float)*M,cudaMemcpyDeviceToHost);
  //And now compare results.
  cudaThreadSynchronize();
  cmvm(M,N,a,x,cy);
  err=0;
  for(i=0;i<M;i++){
    if((cy[i]==0 && y[i]!=0) || fabsf((cy[i]-y[i])/cy[i])>1e-5){
      printf("%d %g %g\n",i,cy[i],y[i]);
      err++;
    }
  }
  i--;
  if(quiet==0){
    printf("%d %g %g\n",i,cy[i],y[i]);
    printf("err: %d\n",err);
  }
  printf("%d %d %s%g %s %g %s %g %s %g %s %d\n",M,N,quiet==0?"kernel done in ":"",t,quiet==0?"s":"",1/t,quiet==0?"Hz":"",N*(Mmod+Mmod/numThreads)*4/t/1073741824,quiet==0?"GB/s":"",N*(M+1)*4/t/1073741824,quiet==0?"effective":"",numThreads);//true rate includes the full (padded) matrix, and loading vector from global to shared memory a number of times.  Effective rate is unpadded matrix and single loading of vector.
}
int main(int argc,char **argv){
  int deviceCount=0;
  int devID=0,i;
  int major, minor;
  char deviceName[100];
  int M,N,numThreads;
  int quiet=0;
  if(argc<4){
    printf("Usage: %s nActs nCents nthreads\ne.g. %s 5672 10864 416\n",argv[0],argv[0]);
    exit(0);
  }
  M=atoi(argv[1]);
  N=atoi(argv[2]);
  numThreads=atoi(argv[3]);
  for(i=4;i<argc;i++){
    if(argv[i][0]=='-'){
      if(argv[i][1]=='q'){
	quiet=1;
      }
    }
  }
    
  if(N>12000){
    printf("Unable to copy to SHM - N too large\n");
    exit(0);
  }
  if(cuInit(0)!=CUDA_SUCCESS){
    printf("cuInit error\n");
    exit(1);
  }
  cuDeviceGetCount(&deviceCount);
  if(deviceCount==0){
    printf("There is no device supporting CUDA.\n");
    exit(0);
  }else{
    if(quiet==0)
      printf("Found %d devices\n",deviceCount);
  }
  cuDeviceComputeCapability(&major, &minor, devID);
  cuDeviceGetName(deviceName, 256, devID);
  if(quiet==0)
    printf("Using Device %d: \"%s\" with Compute %d.%d capability\n", devID, deviceName, major, minor);
  
  CUdevice cuDevice;
  if(cuDeviceGet(&cuDevice,devID)!=CUDA_SUCCESS)
    printf("ERror cuDeviceGet\n");
  if(quiet==0)
    printf("Calling domvm\n");
  doMvm(M,N,numThreads,quiet);
  return 0;
}
#endif
