#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include <cublas.h>
#include <cuda_runtime.h>
//#define N 10864 //number of slopes
//#define M  5672 //number of actuators


//gcc -o cublasSgemvtest cublasSgemvtest.c -O3 -Wall -I/usr/local/cuda/include -L/usr/local/cuda/lib64 -lcublas


int main(int argc,char **argv){
  int devno;
  int i,j;
  struct timeval t1,t2;
  double t;
  int M=5672,N=10864;
  int quiet=0;
  if(argc<3){
    printf("Assuming Nacts %d, ncents %d\n",M,N);
  }else{
    M=atoi(argv[1]);
    N=atoi(argv[2]);
    if(argc>3){
      for(i=3;i<argc;i++){
	if(argv[i][0]=='-'){
	  if(argv[i][1]=='q'){
	    quiet=1;
	  }
	}
      }
    }
  }
  if(cudaSetDevice(0)!=cudaSuccess){
    printf("cudaSetDevice failed\n");
  }
  if(quiet==0)
    printf("Initialising CUBLAS\n");
  cudaError_t status;
  if((status=cublasInit())!=CUBLAS_STATUS_SUCCESS){
    printf("CUBLAS init error\n");
    return 1;
  }else{
    if(quiet==0)
      printf("CUBLAS initialised\n");
  }
  if(cudaGetDevice(&devno)!=cudaSuccess)
    printf("failed to get device number\n");
  if(devno!=0)
    printf("Note - device in use is not that requested\n");
  float *Da,*a,*Dx,*x,*Dy,*y;
  a=malloc(sizeof(float)*M*N);
  x=malloc(sizeof(float)*N);
  y=malloc(sizeof(float)*M);
  for(i=0;i<N;i++){
    x[i]=rand()/(float)RAND_MAX;
    for(j=0;j<M;j++)
      a[i*M+j]=rand()/(float)RAND_MAX;
  }
  if((status=cublasAlloc(M,sizeof(float),(void**)(&Dy)))!=CUBLAS_STATUS_SUCCESS){
    printf("device mem alloc error (cudmcommand\n");
  }
  if((status=cublasAlloc(N,sizeof(float),(void**)(&Dx)))!=CUBLAS_STATUS_SUCCESS){
    printf("device mem alloc error (cudmcnd\n");
  }
  if((status=cublasAlloc(M*N,sizeof(float),(void**)(&Da)))!=CUBLAS_STATUS_SUCCESS){
    printf("device mem alloc error (cuand\n");
  }
  if((status=cublasSetVector(M*N,sizeof(float),a,1,Da,1))!=CUBLAS_STATUS_SUCCESS){
    printf("device access error (write rmx)\n");
  }
  if((status = cublasSetVector(N, sizeof(float),x, 1, Dx, 1))!=CUBLAS_STATUS_SUCCESS){
    printf("!!!! device access error (write centroid vector)\n");
  }
  //warmup
  cublasSgemv('n',M,N,1.,Da,M,Dx, 1, 0.,Dy, 1);
  cudaThreadSynchronize();

  gettimeofday(&t1,NULL);
      //Do the GEMV., curmx==d_A
	//printf("cublasSgemv %d %d\n",rs->nacts,step);
  for(i=0;i<10;i++){
    cublasSgemv('n',M,N,1.,Da,M,Dx, 1, 0.,Dy, 1);
    cudaThreadSynchronize();
  }
  gettimeofday(&t2,NULL);

  if((status=cublasGetError())!=CUBLAS_STATUS_SUCCESS)
    printf("Error in cublasSgemv\n");
  t=(t2.tv_sec-t1.tv_sec+1e-6*(t2.tv_usec-t1.tv_usec))/10;
  printf("%d %d %g %s %g %s %g %s %g %s\n",M,N,t,quiet==0?"s":"",1/t,quiet==0?"Hz":"",N*(M+1)*4/t/1073741824,quiet==0?"GB/s":"",N*(M+1)*4/t/1073741824,quiet==0?"effective":"");
  return 0;
}
