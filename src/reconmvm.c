/*
darc, the Durham Adaptive optics Real-time Controller.
Copyright (C) 2010 Alastair Basden.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/**
   This is a library that can be used for a simple MVM.
*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#ifdef USEMKL
#include <mkl.h>
#include <mkl_types.h>
#include <mkl_blas.h>
#include <mkl_cblas.h>
#include <mkl_vml.h>
#elif defined(USECUDA)
#include <unistd.h>
#include <mqueue.h>
#ifdef MYCUBLAS
#include <cuda.h>
#include <cuda_runtime.h>
#include "gpumvm.h"
#else
#include <cublas.h>
#include <cuda_runtime.h>
#endif
#endif

#include "darc.h"
#include "agbcblas.h"

typedef enum{RECONMODE_SIMPLE,RECONMODE_TRUTH,RECONMODE_OPEN,RECONMODE_OFFSET}ReconModeType;

typedef enum{
  //#ifdef USETREEADD
  //#endif
  BLEEDGAIN,
  BLEEDGROUPS,
  DECAYFACTOR,
  GAINE,
  GAINE2,
  GAINRECONMXT,
  NACTS,
  NCAM,
  //#ifdef USETREEADD
  //#endif
  NSUB,
  //#ifdef USETREEADD
  //#endif
  RECONSTRUCTMODE,
  SUBAPALLOCATION,
  SUBAPFLAG,
  THREADTONUMA,
  TREEBARRIERWAITS,
  TREENPARTS,
  TREEPARTARRAY,
  V0,
  //Add more before this line.
  RECONNBUFFERVARIABLES//equal to number of entries in the enum
}RECONBUFFERVARIABLEINDX;

#define reconMakeNames() bufferMakeNames(RECONNBUFFERVARIABLES,"bleedGain","bleedGroups","decayFactor","gainE","gainE2","gainReconmxT","nacts","ncam","nsub","reconstructMode","subapAllocation","subapFlag","threadToNuma","treeBarrierWaits","treeNparts","treePartArray","v0")
//char *RECONPARAM[]={"gainReconmxT","reconstructMode","gainE","v0","bleedGain","decayFactor","nacts"};//,"midrange"};


typedef struct{
  /* treeAdd params */
  //#ifdef USETREEADD
  int *barrierWaits;
  volatile int *barrierFinished;
  int *partArray;
  int nparts;
  pthread_spinlock_t *partSpins;
  float **output;
    //#endif

  ReconModeType reconMode;
  float *gainE;
  float *gainE2;
  int polc; //1 for implicit polc, 2 for explicit polc
  float *polcCentroids;
  int polcCentroidsSize;
  int dmCommandArrSize;
#ifndef USECUDA
  float **dmCommandArr;
#endif
  float *rmxT;
  float **threadRmx;
  float *v0;
  float bleedGain;//OverNact;
  float *bleedGainArr;//OverNact;
  int *bleedGroupArr;
  int bleedGroups;
  float *bleedVal;
  int bleedValSize;
  //float midRangeTimesBleed;
  float *decayFactor;
  int nacts;
  int totCents;
#ifdef MYCUBLAS
  int nactsPadded;
  int newswap;
#endif
  int *subapAlloc;
  int *threadSubapCnt;
}ReconStructEntry;


typedef struct{
  ReconStructEntry rs[2];
  int buf;//current buffer being used
  int postbuf;//current buffer for post processing threads.
  //int swap;//set if need to change to the other buffer.
  // volatile int dmReady;  //drj 140422: No longer needed with darc_condwait
  volatile int polcCounter;
  float *latestDmCommand;
  float *latestDmCommand2;
  int latestDmCommandSize;
  int latestDmCommand2Size;
  darc_mutex_t dmMutex;
//   darc_cond_t dmCond;
  darc_condwait_t dmCondwait; //drj 140422: changed the darc_futex calls for darc_condwait calls...
  //int bufindx[RECONNBUFFERVARIABLES];
  circBuf *rtcErrorBuf;
  int nthreads;
  char *paramNames;
  int index[RECONNBUFFERVARIABLES];
  void *values[RECONNBUFFERVARIABLES];
  char dtype[RECONNBUFFERVARIABLES];
  int nbytes[RECONNBUFFERVARIABLES];
  arrayStruct *arr;
  int *threadToNumaList;
  int *centIndxTot;//only used for Numa.
#ifdef USECUDA
  //float *setDmCommand;
  char *mqname;
  char *mqnameFromGPU;
  int err;
  int go;
  int msgsize;
  float *initCommand;
  float *centroids;
  float *dmCommand;
  mqd_t mq;
  pthread_mutex_t cudamutex;
  pthread_cond_t cudacond;
  int retrievedDmCommand;
  pthread_t threadid;
  int cucentroidssize;
  int curmxsize;
  int deviceNo;
  unsigned int *threadAffinity;
  int threadAffinElSize;
  int threadPriority;
#ifdef MYCUBLAS
  int numThreadsPerBlock;
#endif
#endif
}ReconStruct;

#ifdef USECUDA
enum MESSAGES{INITFRAME=1,DOMVM,ENDFRAME,UPLOAD,CUDAEND};
#endif

void freeTreeAdd(ReconStructEntry *rs){
  int i;
  if(rs->partSpins!=NULL){
      free((void*)rs->partSpins);
    }
    if(rs->output!=NULL){
        for(i=1;i<rs->nparts;i++){
            if(rs->output[i]!=NULL){
              free(rs->output[i]);
            }
        }
        free(rs->output);
    }
    if(rs->barrierFinished!=NULL){
      free((void *)rs->barrierFinished);
    }
}
/**
   Called to free the reconstructor module when it is being closed.
*/
int reconClose(void **reconHandle){//reconHandle is &globals->reconStruct.
  ReconStruct *reconStruct=(ReconStruct*)*reconHandle;
  ReconStructEntry *rs;
#ifndef USECUDA
  int i;
#endif
  printf("Closing reconlibrary\n");
  if(reconStruct!=NULL){
    if(reconStruct->paramNames!=NULL)
      free(reconStruct->paramNames);
    darc_mutex_destroy(&reconStruct->dmMutex);
//     darc_cond_destroy(&reconStruct->dmCond);
    darc_condwait_destroy(&reconStruct->dmCondwait);
    rs=&reconStruct->rs[0];
    freeTreeAdd(rs);
    if(reconStruct->latestDmCommand!=NULL)
      free(reconStruct->latestDmCommand);
    if(reconStruct->latestDmCommand2!=NULL)
      free(reconStruct->latestDmCommand2);
#ifndef USECUDA
    if(rs->dmCommandArr!=NULL){
      for(i=0; i<reconStruct->nthreads; i++){
	if(rs->dmCommandArr[i]!=NULL)
	  free(rs->dmCommandArr[i]);
      }
      free(rs->dmCommandArr);
    }
#endif
#ifdef USECUDA
    if(reconStruct->threadAffinity!=NULL)
      free(reconStruct->threadAffinity);
    int msg=CUDAEND;
    reconStruct->go=0;
    pthread_mutex_lock(&reconStruct->cudamutex);
    if(mq_send(reconStruct->mq,(char*)&msg,sizeof(int),0)!=0){
      printf("Error in mq_send in reconClose()\n");
    }
    pthread_mutex_unlock(&reconStruct->cudamutex);

    if(reconStruct->threadid!=0){
      pthread_join(reconStruct->threadid,NULL);
      printf("joined reconcuda worker\n");
    }
    if(reconStruct->mqname!=NULL){
      if(mq_unlink(reconStruct->mqname)!=0){
	printf("mq_unlink error: %s\n",strerror(errno));
      }else{
	printf("mq unlinked %s\n",reconStruct->mqname);
      }
      free(reconStruct->mqname);
    }
    pthread_mutex_destroy(&reconStruct->cudamutex);
    pthread_cond_destroy(&reconStruct->cudacond);
#endif
    rs=&reconStruct->rs[1];
#ifndef USECUDA
    if(rs->dmCommandArr!=NULL){
      for(i=0; i<reconStruct->nthreads; i++){
	if(rs->dmCommandArr[i]!=NULL)
	  free(rs->dmCommandArr[i]);
      }
      free(rs->dmCommandArr);
    }
#endif
    free(reconStruct);
  }

  *reconHandle=NULL;
  printf("Finished reconClose\n");
  return 0;
}

#ifdef USECUDA


int reconSetThreadAffinityAndPriority(unsigned int *threadAffinity,int threadPriority,int threadAffinElSize){
  int i;
  cpu_set_t mask;
  int ncpu;
  struct sched_param param;
  printf("Getting CPUs\n");
  ncpu= sysconf(_SC_NPROCESSORS_ONLN);
  printf("Got %d CPUs\n",ncpu);
  CPU_ZERO(&mask);
  printf("Setting %d CPUs\n",ncpu);
  for(i=0; i<ncpu && i<threadAffinElSize*32; i++){
    if(((threadAffinity[i/32])>>(i%32))&1){
      CPU_SET(i,&mask);
    }
  }
  if(pthread_setaffinity_np(0,sizeof(cpu_set_t),&mask))
    printf("Error in sched_setaffinity: %s\n",strerror(errno));
  printf("Setting setparam\n");
  param.sched_priority=threadPriority;
  if(pthread_setschedparam(pthread_self(),SCHED_RR,&param))
    printf("error in pthread_setschedparam - maybe run as root?\n");
  return 0;
}


//with cuda only 1 thread can access the GPU (actually not strictly true, but simplifies stuff if you assume this).  So, use a mqueue implementation here.
void *reconWorker(void *reconHandle){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  ReconStructEntry *rs;
  int *msg;
  mqd_t mq;
  float *curmx=NULL;
  float *cucentroids=NULL;
  float *cudmCommand=NULL;
  float *dmCommandTmp=NULL;
  int step;
  int centindx;
  int deviceCount=0;
  rs=&reconStruct->rs[reconStruct->buf];
  if((mq=mq_open(reconStruct->mqname,O_RDONLY))==-1){
    printf("Error opening mqueue %s in reconmvm\n",reconStruct->mqname);
    reconStruct->err=1;
    return NULL;
  }
  if((msg=malloc(reconStruct->msgsize))==NULL){
    printf("Error allocating msg for mq in reconmvm\n");
    reconStruct->err=1;
    return NULL;
  }
  reconSetThreadAffinityAndPriority(reconStruct->threadAffinity,reconStruct->threadPriority,reconStruct->threadAffinElSize);
  printf("Initialising CUDA\n");
#ifdef MYCUBLAS
  int major, minor;
  char deviceName[64];
  CUdevice cuDevice;
  CUcontext cuContext;
  //CUmodule cuModule;
  //CUfunction sgemvKernel;
  if(cuInit(0)!=CUDA_SUCCESS){
    printf("cuInit error\n");
    return NULL;
  }else{
    printf("cuInit succeeded\n");
  }

  if(cuDeviceGetCount(&deviceCount)!=CUDA_SUCCESS || deviceCount==0){
    printf("There is no device supporting CUDA.\n");
    return NULL;
  }else{
    printf("Found %d devices\n",deviceCount);
  }
  if(reconStruct->deviceNo>=deviceCount || reconStruct->deviceNo<0){
    printf("Illegal CUDA device number requested - using 0\n");
    reconStruct->deviceNo=0;
  }
  cuDeviceComputeCapability(&major, &minor, reconStruct->deviceNo);
  cuDeviceGetName(deviceName, 64, reconStruct->deviceNo);
  printf("Using Device %d: \"%s\" with Compute %d.%d capability\n", reconStruct->deviceNo, deviceName, major, minor);
  if(cuDeviceGet(&cuDevice,reconStruct->deviceNo)!=CUDA_SUCCESS){
    printf("ERror cuDeviceGet\n");
    return NULL;
  }
  if(cuCtxCreate(&cuContext,0,cuDevice)!=CUDA_SUCCESS){
    printf("Error cuCtxCreate\n");
    return NULL;
  }
  /*if(cuModuleLoad(&cuModule,"mygpumvm.ptx")!=CUDA_SUCCESS){
    printf("Unable to load module mygpumvm.ptx\n");
    return NULL;
  }
  if(cuModuleGetFunction(&sgemvKernel,cuModule,"sgemvKernelMN1M111")!=CUDA_SUCCESS){
    printf("Error getting sgemvKernelMN1M111\n");
    return NULL;
    }*/
#else
  int devno;
  //cudaInit();
  cudaGetDeviceCount(&deviceCount);
  if(reconStruct->deviceNo>=deviceCount){
    printf("Illegal CUDA device number requested - using 0\n");
    reconStruct->deviceNo=0;
  }
  printf("Found %d devices - using %d\n",deviceCount,reconStruct->deviceNo);
  if(cudaSetDevice(reconStruct->deviceNo)!=cudaSuccess){
    printf("cudaSetDevice failed\n");
  }

  printf("Initialising CUBLAS\n");
  cudaError_t status;
  if((status=cublasInit())!=CUBLAS_STATUS_SUCCESS){
    printf("CUBLAS init error\n");
    return NULL;
  }else{
    printf("CUBLAS initialised\n");
  }
  if(cudaGetDevice(&devno)!=cudaSuccess)
    printf("failed to get device number\n");
  if(devno!=reconStruct->deviceNo)
    printf("Note - device in use is not that requested\n");
#endif//MYCUBLAS


  while(reconStruct->go){
    //first of all, read the queue until we receive an end of frame message.
    //Then, finish the GPU stuff and send a frame ended message.
    if(mq_receive(mq,(char*)msg,reconStruct->msgsize,NULL)==-1){
      printf("Error in mq_receive in reconmvm: %s",strerror(errno));
    }else{
      if(msg[0]==INITFRAME){//initialise the work vector to dmCommand
	//no longer used.
	//upload dmCommand
	//if((status=cublasSetVector(rs->nacts,sizeof(float),reconStruct->initCommand,1,cudmCommand,1))!=CUBLAS_STATUS_SUCCESS){
	//printf("cudamemset cublasSetVector failed  status=%d\n",(int)status);
	//}
      }else if(msg[0]==DOMVM){//do the mvm
	step=msg[1];//step;
	centindx=msg[2];//centindx;
	//copy the centroids to GPU cucentroids==d_B
	//printf("addr %p\n",&(rs->cucentroids[threadno][centindx]));

#ifdef MYCUBLAS
	//Maybe use cudaMemcpyAsync instead?  In which case, needs to be paged locked host memory.  Probably use cudaHostRegister() - so long as centroids are mem_aligned().
	//Also use streams.
	if(cudaMemcpy(&cucentroids[centindx],&reconStruct->centroids[centindx],sizeof(float)*step,cudaMemcpyHostToDevice)!=cudaSuccess)
	  printf("cudaMemcpy centroids error Pointers %p %p, size %d centindx %d\n",&(reconStruct->centroids[centindx]),&(cucentroids[centindx]),(int)(step*sizeof(float)),centindx);
	gpusgemvMN1M111(rs->nacts,step,&curmx[centindx*rs->nacts],&cucentroids[centindx],cudmCommand,reconStruct->numThreadsPerBlock);
	/*if(cuMemcpyHtoD(&cucentroids[centindx],&reconStruct->centroids[centindx],sizeof(float)*step)!=CUDA_SUCCESS)
	printf("cuMemcpyHtoD failed Pointers %p %p, size %d centindx %d\n",&(reconStruct->centroids[centindx]),&(cucentroids[centindx]),(int)(step*sizeof(float)),centindx);
	//args are M,N,a,x,y.
	if(cuFuncSetBlockShape(sgemvKernel,reconStruct->numThreadsPerBlock,1,1)!=CUDA_SUCCESS)
	  printf("cufuncSetBlockShape failed\n");

	int offset=0;
	cuParamSeti(sgemvKernel,offset,rs->nacts);
	offset+=sizeof(int);
	cuParamSeti(sgemvKernel,offset,step);
	offset+=sizeof(int);
	cuParamSetv(sgemvKernel,offset,);


	cuParamSetSize(sgemvKernel,offset);
	cuLaunchGrid(sgemvKernel,((M+reconStruct->numThreadsPerBlock-1)/reconStruct->numThreadsPerBlock),1);
	*/
#else
	error_t status;
	//printf("cublasSetVector(centroids) Pointers %p %p, size %d centindx %d nacts %d\n",&(reconStruct->centroids[centindx]),&(cucentroids[centindx]),(int)(step*sizeof(float)),centindx,rs->nacts);
	if((status = cublasSetVector(step, sizeof(float),&(reconStruct->centroids[centindx]), 1, &(cucentroids[centindx]), 1))!=CUBLAS_STATUS_SUCCESS){
	  printf("!!!! device access error (write centroid vector):error no=%d Pointers %p %p, size %d centindx %d\n",(int)status,&(reconStruct->centroids[centindx]),&(cucentroids[centindx]),(int)(step*sizeof(float)),centindx);
	}
	//Do the GEMV., curmx+=d_A
	//printf("cublasSgemv %d %d\n",rs->nacts,step);
	cublasSgemv('n',rs->nacts,step,1.,&(curmx[(centindx*rs->nacts)]), rs->nacts, &(cucentroids[centindx]), 1, 1.,cudmCommand, 1);
	if((status=cublasGetError())!=CUBLAS_STATUS_SUCCESS)
	  printf("Error in cublasSgemv\n");
#endif

      }else if(msg[0]==ENDFRAME){//end the frame - get the results.
	//printf("cuda endFrame\n");
	if(dmCommandTmp==NULL){
	  printf("Error - dmcomandTmp==NULL\n");
	}else{
#ifdef MYCUBLAS
	  if(cudaMemcpy(dmCommandTmp,cudmCommand,sizeof(float)*rs->nacts,cudaMemcpyDeviceToHost)!=cudaSuccess){
	    printf("cuda device access error cudaMemcpy dmcommand\n");
	  }

#else
	  if((status=cublasGetVector(rs->nacts, sizeof(float), cudmCommand,1,dmCommandTmp,1))!=CUBLAS_STATUS_SUCCESS){
	    printf ("cuda device access error (get dmCommand vector) %d %p\n",(int)status,cudmCommand);
	  }
#endif
	  //Now add dmCommandTmp to dmCommand
    #ifdef USEMKL
	  cblas_saxpy(rs->nacts,1.,dmCommandTmp,1.,reconStruct->dmCommand,1.0);
    #elif defined(USEAGBBLAS)
	  agb_cblas_saxpy111(rs->nacts,dmCommandTmp,reconStruct->dmCommand);
	  #else
    printf("Error: No cblas lib defined in Makefile\n");
    return 1;
    }
    #endif

	//And now clear cudmCommand
#ifdef MYCUBLAS
	cudaMemset(cudmCommand,0,sizeof(float)*rs->nacts);
#else
	cudaMemset(cudmCommand,0,sizeof(float)*rs->nacts);
#endif
	//and now tell the rtc that data has been received.
	//msg[0]=FRAMEENDED;
	//mq_send(mqFromGPU,msg,sizeof(int),0);
	pthread_mutex_lock(&reconStruct->cudamutex);
	reconStruct->retrievedDmCommand=1;
	pthread_cond_signal(&reconStruct->cudacond);
	pthread_mutex_unlock(&reconStruct->cudamutex);
      }else if(msg[0]==UPLOAD){//upload the rmx etc
	//printf("cuda UPLOAD\n");
	rs=&reconStruct->rs[reconStruct->buf];
	if(msg[1]==1){
	  if(dmCommandTmp!=NULL)
	    free(dmCommandTmp);
	  if(posix_memalign((void**)dmCommandTmp,ARRAYALIGN,sizeof(float)*rs->nacts)!=0){
	    printf("dmCommandTmp malloc failed\n");
	    reconStruct->err=-2;
	  }
#ifdef MYCUBLAS
	  if(cudmCommand!=NULL)
	    cudaFree(cudmCommand);
	  if(cudaMalloc((void**)&cudmCommand,rs->nactsPadded*sizeof(float))!=cudaSuccess){
	    printf("device mem alloc error (cudmcommand\n");
	    reconStruct->err=-2;
	  }
#else
	  cudaError_t status;
	  if(cudmCommand!=NULL)
	    cublasFree(cudmCommand);
	  if((status=cublasAlloc(rs->nacts,sizeof(float),(void**)(&cudmCommand)))!=CUBLAS_STATUS_SUCCESS){
	    printf("device mem alloc error (cudmcommand\n");
	    reconStruct->err=-2;
	  }
#endif
	}
	if(msg[2]==1){
#ifdef MYCUBLAS
	  if(curmx!=NULL)
	    cudaFree(curmx);
	  if(cudaMalloc((void**)&curmx,rs->nactsPadded*rs->totCents*sizeof(float))!=cudaSuccess){
	    printf("device mem alloc error (curmx)\n");
	    reconStruct->err=-4;
	    reconStruct->curmxsize=0;
	  }else{
	    reconStruct->curmxsize=rs->nacts*rs->totCents;
	  }
#else
	  if(curmx!=NULL)
	    cublasFree(curmx);
	  if((status=cublasAlloc(rs->nacts*rs->totCents,sizeof(float),(void**)&curmx))!=CUBLAS_STATUS_SUCCESS){
	    printf("device memory allocation error (curmx)\n");
	    reconStruct->err=-4;
	    reconStruct->curmxsize=0;
	  }else{
	    reconStruct->curmxsize=rs->nacts*rs->totCents;
	  }
#endif
	}
	if(msg[3]==1){
#ifdef MYCUBLAS
	  if(cucentroids!=NULL)
	    cudaFree(cucentroids);
	  if(cudaMalloc((void**)&cucentroids,rs->totCents*sizeof(float))!=cudaSuccess){
	    printf("device memory allocation error (cucentroids)\n");
	    reconStruct->err=-5;
	  }
	  if(reconStruct->err!=0){
	    reconStruct->cucentroidssize=0;
	  }else{
	    reconStruct->cucentroidssize=rs->totCents;
	  }
#else
	  if(cucentroids!=NULL)
	    cublasFree(cucentroids);
	  if((status=cublasAlloc(rs->totCents,sizeof(float),(void**)&cucentroids))!=CUBLAS_STATUS_SUCCESS){
	    printf("device memory allocation error (cucentroids)\n");
	    reconStruct->err=-5;
	  }
	  if(reconStruct->err!=0){
	    reconStruct->cucentroidssize=0;
	  }else{
	    reconStruct->cucentroidssize=rs->totCents;
	  }

#endif
	}
	//upload the rmx.
#ifdef MYCUBLAS
	if(cudaMemcpy(curmx,rs->rmxT,sizeof(float)*rs->totCents*rs->nacts,cudaMemcpyHostToDevice)!=cudaSuccess){
	  printf("device access error (write rmx)\n");
	  reconStruct->err=2;
	}
#else
	if((status=cublasSetVector(rs->totCents*rs->nacts,sizeof(float),rs->rmxT,1,curmx,1))!=CUBLAS_STATUS_SUCCESS){
	  printf("device access error (write rmx)\n");
	  reconStruct->err=2;
	}
#endif
	//printf("cuda pointers:%p %p %p %p\n",cudmCommand,dmCommandTmp,curmx,cucentroids);
      }else if(msg[0]==CUDAEND){
      }else{//unrecognised message
	printf("Message not recognised in reconmvm\n");
      }
    }

  }
#ifdef MYCUBLAS
  if(cudmCommand!=NULL)
    cudaFree(cudmCommand);
  if(curmx!=NULL)
    cudaFree(curmx);
  if(cucentroids!=NULL)
    cudaFree(cucentroids);
#else
  if(cudmCommand!=NULL)
    cublasFree(cudmCommand);
  if(curmx!=NULL)
    cublasFree(curmx);
  if(cucentroids!=NULL)
    cublasFree(cucentroids);
  printf("calling cublasShutdown\n");
  cublasShutdown();
#endif
  if(dmCommandTmp!=NULL)
    free(dmCommandTmp);
  return NULL;
}

#endif

void initTreeAdd(void *reconHandle){
    ReconStruct *reconStruct=(ReconStruct*)reconHandle;
    ReconStructEntry *rs=&reconStruct->rs[reconStruct->buf];

    int i,j;

    freeTreeAdd(rs);

    printf("reconmvm: initialising treeAdd components\n");

    if(posix_memalign((void**)&rs->partSpins,ARRAYALIGN,sizeof(pthread_spinlock_t)*rs->nparts)!=0){
      printf("posix_memalign failed for treeAdd partSpins\n");
    }
    if(posix_memalign((void**)&rs->output,ARRAYALIGN,sizeof(float*)*rs->nparts)!=0){
      printf("posix_memalign failed for treeAdd output\n");
    }
    if(posix_memalign((void**)&rs->barrierFinished,ARRAYALIGN,sizeof(volatile int)*rs->nparts)!=0){
      printf("posix_memalign failed for treeAdd barrierFinished\n");
    }

    for(i=0;i<rs->nparts;i++){
        pthread_spin_init(&rs->partSpins[i],0);
    }

    for(i=1;i<rs->nparts;i++){
        if(posix_memalign((void**)&rs->output[i],ARRAYALIGN,sizeof(float)*rs->nacts)!=0){
	  printf("posix_memalign failed for treeAdd output[%d]\n",i);
	}
        float *arr = rs->output[i];
        for(j=0;j<rs->nacts;j++){
            arr[j] = 0.0;
        }
    }
    rs->output[0]=reconStruct->arr->dmCommand;
    for(i=0;i<rs->nparts;i++){
        rs->barrierFinished[i]=0;
    }
    printf("reconmvm: initialised treeAdd components\n");
}


/**
   Called asynchronously, whenever new parameters are ready.
   Once this returns, a call to swap buffers will be issued.
   (actually, at the moment, this is called synchronously by first thread when a buffer swap is done).
*/
int reconNewParam(void *reconHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr,int totCents){
  int j=0,err=0,cnt=0;
  int nb;
  //globalStruct *globals=threadInfo->globals;
  //infoStruct *info=threadInfo->info;//totcents and nacts
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;//threadInfo->globals->reconStruct;
  //int *indx=reconStruct->bufindx;
  //Use the buffer not currently in use.
  ReconStructEntry *rs;
  RECONBUFFERVARIABLEINDX i;
  int nfound;
  int *nbytes=reconStruct->nbytes;
  void **values=reconStruct->values;
  char *dtype=reconStruct->dtype;
  reconStruct->arr=arr;
#ifdef USECUDA
  int msg[4];
  msg[1]=0;//resize flag
  msg[0]=UPLOAD;//the operation flag
#endif
  //swap the buffers...
  reconStruct->buf=1-reconStruct->buf;
  rs=&reconStruct->rs[reconStruct->buf];
  rs->totCents=totCents;
  nfound=bufferGetIndex(pbuf,RECONNBUFFERVARIABLES,reconStruct->paramNames,reconStruct->index,reconStruct->values,reconStruct->dtype,reconStruct->nbytes);
  if(nfound!=RECONNBUFFERVARIABLES){
    for(j=0; j<RECONNBUFFERVARIABLES; j++){
      if(reconStruct->index[j]<0 && j!=GAINE2 && j!=SUBAPALLOCATION && j!=THREADTONUMA && j!=TREENPARTS && j!=TREEBARRIERWAITS && j!=TREEPARTARRAY){
	printf("Missing %16s\n",&reconStruct->paramNames[j*BUFNAMESIZE]);
	err=-1;
      }
    }
  }
  if(err==0){
    i=NACTS;
    if(dtype[i]=='i' && nbytes[i]==4){
      rs->nacts=*((int*)values[i]);
    }else{
      printf("nacts error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"nacts error");
      err=NACTS;
    }
#ifdef USECUDA
#ifdef MYCUBLAS
    rs->nactsPadded=((rs->nacts+reconStruct->numThreadsPerBlock-1)/reconStruct->numThreadsPerBlock)*reconStruct->numThreadsPerBlock;
    rs->newswap=1;
#endif
#endif
    i=RECONSTRUCTMODE;
    nb=nbytes[i];
    if(dtype[i]=='s'){
      if(strncmp((char*)values[i],"simple",nb)==0){
	rs->reconMode=RECONMODE_SIMPLE;
      }else if(strncmp((char*)values[i],"truth",nb)==0){
	rs->reconMode=RECONMODE_TRUTH;
      }else if(strncmp((char*)values[i],"open",nb)==0){
	rs->reconMode=RECONMODE_OPEN;
      }else if(strncmp((char*)values[i],"offset",nb)==0){
	rs->reconMode=RECONMODE_OFFSET;
      }else{
	printf("reconstructMode not interpreted, assuming simple\n");
	rs->reconMode=RECONMODE_SIMPLE;
      }
    }else{
      printf("reconstructMode error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"reconstructMode error");
      err=RECONSTRUCTMODE;
    }
    i=GAINRECONMXT;
    if(dtype[i]=='f' && nbytes[i]==rs->totCents*rs->nacts*sizeof(float)){
      rs->rmxT=(float*)values[i];
    }else{
      printf("gainReconmxT error %c %d %d %d\n",dtype[i],nbytes[i],rs->totCents,rs->nacts);
      err=GAINRECONMXT;
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"gainReconmxT error");
    }
    i=GAINE;
    if(nbytes[i]==0 || rs->reconMode!=RECONMODE_OPEN){
      rs->gainE=NULL;
      rs->polc=0;
    }else if(dtype[i]=='f' && nbytes[i]==sizeof(float)*rs->nacts*rs->nacts){
      rs->gainE=(float*)values[i];
      rs->polc=1;//implicit polc
    }else if(dtype[i]=='f' && nbytes[i]==sizeof(float)*rs->nacts*rs->totCents){
      rs->gainE=(float*)values[i];
      rs->polc=2;//explicit polc
      if(rs->polcCentroidsSize<sizeof(float)*rs->totCents){
	if(rs->polcCentroids!=NULL){
	  free(rs->polcCentroids);
	}
	if(posix_memalign((void**)&rs->polcCentroids,ARRAYALIGN,sizeof(float)*rs->totCents)!=0){
	  printf("Error allocating recon polcCentroids memory\n");
	  err=-2;
	  rs->polcCentroidsSize=0;
	}else{
	  memset(rs->polcCentroids,0,sizeof(float)*rs->totCents);
	}
      }
    }else{
      rs->polc=0;
      rs->gainE=NULL;
      printf("gainE error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"gainE");
      err=GAINE;
    }
    i=GAINE2;
    if(reconStruct->index[i]>=0){
      if(nbytes[i]==0 || rs->reconMode!=RECONMODE_OPEN){
	rs->gainE2=NULL;
      }else if(dtype[i]=='f' && nbytes[i]==sizeof(float)*rs->nacts*rs->nacts && rs->polc==1){
	rs->gainE2=(float*)values[i];
      }else if(dtype[i]=='f' && nbytes[i]==sizeof(float)*rs->nacts*rs->totCents && rs->polc==2){
	rs->gainE2=(float*)values[i];
      }else{
	rs->gainE2=NULL;
	printf("gainE2 error\n");
	writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"gainE2");
	err=1;
      }
    }else{
      rs->gainE2=NULL;
    }
    i=V0;
    if(nbytes[i]==0)
      rs->v0=NULL;
    else if(dtype[i]=='f' && nbytes[i]==sizeof(float)*rs->nacts){
      rs->v0=(float*)values[i];
    }else{
      printf("v0 error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"v0 error");
      err=V0;
    }
    i=BLEEDGAIN;
    if(dtype[i]=='f'){
      if(nbytes[i]==sizeof(float)){
	rs->bleedGain=(*((float*)values[i]));///rs->nacts;
	rs->bleedGainArr=NULL;
	rs->bleedGroups=1;
      }else{
	rs->bleedGainArr=((float*)values[i]);
	rs->bleedGroups=nbytes[i]/sizeof(float);
      }
    }else{
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"bleedGain error");
      printf("bleedGain error\n");
      rs->bleedGainArr=NULL;
      rs->bleedGroups=1;
      err=BLEEDGAIN;
    }
    i=BLEEDGROUPS;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)*rs->nacts){
      rs->bleedGroupArr=(int*)values[i];
    }else{
      rs->bleedGroupArr=NULL;
      if(nbytes[i]!=0){
	printf("bleedGroups error\n");
	writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"bleedGroups error");
	err=BLEEDGROUPS;
      }
    }

    i=DECAYFACTOR;
    if(nbytes[i]==0){
      rs->decayFactor=NULL;
    }else if(dtype[i]=='f' && nbytes[i]==sizeof(float)*rs->nacts){
      rs->decayFactor=(float*)values[i];
    }else{
      rs->decayFactor=NULL;
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"decayFactor error");
      printf("decayFactor error\n");
      err=DECAYFACTOR;
    }
    /* get treeAdd params from the buffer*/
    //#ifdef USETREEADD
    cnt=(reconStruct->index[TREENPARTS]>=0) + (reconStruct->index[TREEBARRIERWAITS]>=0) + (reconStruct->index[TREEPARTARRAY]>=0);
    if(cnt==3){//all tree components present
      i=TREENPARTS;
      if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
	rs->nparts=*((int*)values[i]);
      }else{
	writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"nparts error");
	printf("nparts error\n");
	err=TREENPARTS;
      }
      i=TREEBARRIERWAITS;
      if(dtype[i]=='i' && nbytes[i]==sizeof(int)*(reconStruct->nthreads)){
	rs->barrierWaits=(int*)values[i];
      }else{
	rs->barrierWaits=NULL;
	writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"barrierWaits error");
	printf("barrierWaits error\n");
	err=TREEBARRIERWAITS;
      }
      i=TREEPARTARRAY;
      if(dtype[i]=='i' && nbytes[i]==sizeof(int)*reconStruct->nthreads*2){
	rs->partArray=(int*)values[i];
      }else{
	rs->partArray=NULL;
	writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"partArray error");
	printf("partArray error\n");
	err=TREEPARTARRAY;
      }
      if(rs->nparts<=0){
        printf("reconmvm: Warning - nparts <= 0, not using treeAdd\n");
        rs->nparts=0;
      }else{
        initTreeAdd(reconHandle);
      }
    }else if(cnt>0){
      printf("reconmvm: Warning - Some, but not all treeAdd components present...\n");
      rs->nparts=0;
    }else{//no treeAdd
      printf("reconmvm: Info - no treeAdd components present...\n");
      rs->nparts=0;
    }

    //#endif
  }
  //No need to get the lock here because this and newFrame() are called inside glob->libraryMutex.
  darc_condwait_setvalue(&reconStruct->dmCondwait,0);
  if(rs->dmCommandArrSize<sizeof(float)*rs->nacts){
    rs->dmCommandArrSize=sizeof(float)*rs->nacts;
#ifdef USECUDA
    if(err==0){
      msg[1]=1;//need to resize.
    }
#else
    for(j=0; j<reconStruct->nthreads; j++){
      if(rs->dmCommandArr[j]!=NULL)
	free(rs->dmCommandArr[j]);
      if(posix_memalign((void**)&rs->dmCommandArr[j],ARRAYALIGN,sizeof(float)*rs->nacts)!=0){
	printf("Error allocating recon dmCommand memory\n");
	err=-2;
	rs->dmCommandArrSize=0;
      }else{
        memset(rs->dmCommandArr[j],0,sizeof(float)*rs->nacts);
      }
    }
#endif
  }
  if(reconStruct->latestDmCommandSize<sizeof(float)*rs->nacts){
    reconStruct->latestDmCommandSize=sizeof(float)*rs->nacts;
    if(reconStruct->latestDmCommand!=NULL)
      free(reconStruct->latestDmCommand);
    if(posix_memalign((void**)&reconStruct->latestDmCommand,ARRAYALIGN,rs->nacts*sizeof(float))!=0){
      printf("Error allocating latestDmCommand memory\n");
      err=-3;
      reconStruct->latestDmCommandSize=0;
    }else{
      memset(reconStruct->latestDmCommand,0,rs->nacts*sizeof(float));
    }
  }
  if(rs->gainE2!=NULL){
    if(reconStruct->latestDmCommand2Size<sizeof(float)*rs->nacts){
      reconStruct->latestDmCommand2Size=sizeof(float)*rs->nacts;
      if(reconStruct->latestDmCommand2!=NULL)
	free(reconStruct->latestDmCommand2);
      if(posix_memalign((void**)&reconStruct->latestDmCommand2,ARRAYALIGN,rs->nacts*sizeof(float))!=0){
	printf("Error allocating latestDmCommand2 memory\n");
	err=-3;
	reconStruct->latestDmCommand2Size=0;
      }else{
        memset(reconStruct->latestDmCommand2,0,rs->nacts*sizeof(float));
      }
    }
  }
  if(err==0 && rs->bleedGroupArr!=NULL){
    nb=0;
    for(j=0;j<rs->nacts;j++){
      if(rs->bleedGroupArr[j]>nb)
	nb=rs->bleedGroupArr[j];
    }
    nb++;
    if(rs->bleedGroups>1 && nb>rs->bleedGroups){
      printf("Error - bleed groups not consistent with bleed gain\n");
      err=1;
    }else{
      rs->bleedGroups=nb;
    }
  }
  if(err==0){
    if(rs->bleedValSize<rs->bleedGroups){
      rs->bleedValSize=rs->bleedGroups;
      if(rs->bleedVal!=NULL)
	free(rs->bleedVal);
      if((rs->bleedVal=calloc(sizeof(float),rs->bleedGroups))==NULL){
	printf("error allocing bleedVal\n");
	rs->bleedValSize=0;
	err=1;
      }
    }
  }

  if(pbuf->nNumaNodes!=0 && reconStruct->index[THREADTONUMA]>=0 && dtype[THREADTONUMA]=='i' && nbytes[THREADTONUMA]==sizeof(int)*reconStruct->nthreads){
    //some numa aware buffers - search for partial reconstruction matrices here.
    //Need to work out now many subaps to be done by each thread... which then tells us the size of the relevant arrays to look out for.
    int ncam,nsubtot=0;
    int *nsub;
    char paramList[17];
    int indx;
    void *valptr;
    char dtp;
    int nb;
    int numaIndx;
    int *subapFlag;
    reconStruct->threadToNumaList=(int*)values[THREADTONUMA];
    ncam=*((int*)values[NCAM]);//number of cameras
    nsub=((int*)values[NSUB]);//array of number of subaps/camera.
    for(j=0;j<ncam;j++)
      nsubtot+=nsub[j];//total number of subaps (used and unused)
    rs->subapAlloc=NULL;
    if(reconStruct->index[SUBAPALLOCATION]>=0){//defined subaps
      if(nbytes[SUBAPALLOCATION]==sizeof(int)*nsubtot && dtype[SUBAPALLOCATION]=='i'){
	rs->subapAlloc=(int*)values[SUBAPALLOCATION];
      }
    }
    if(rs->subapAlloc!=NULL){//threads have defined subaps.
      subapFlag=(int*)values[SUBAPFLAG];
      memset(rs->threadSubapCnt,0,sizeof(int)*reconStruct->nthreads);
      for(j=0;j<nsubtot;j++){
	if(subapFlag[j] && rs->subapAlloc[j]>=0 && rs->subapAlloc[j]<reconStruct->nthreads)
	  rs->threadSubapCnt[rs->subapAlloc[j]]++;
      }
      for(j=0;j<reconStruct->nthreads;j++){//Get the relevant part of the RMX - there will be one per thread.
	snprintf(paramList,17,"gainReconmxT%d",j);//find the buffer for this thread:
	numaIndx=reconStruct->threadToNumaList[j];//get the node this thread is closest to.
	if(bufferGetIndex((paramBuf*)(pbuf->numaBufs[numaIndx]),1,paramList,&indx,&valptr,&dtp,&nb)==1){//found the part of the reconstruct for this thread.
	  if(dtp=='f' && nb==sizeof(float)*rs->nacts*rs->threadSubapCnt[j]*2){
	    rs->threadRmx[j]=(float*)valptr;
	  }else{
	    printf("Error - %s is wrong size or dtype (%c %d, should be %ld)\n",paramList,dtp,nb,sizeof(float)*rs->nacts*rs->threadSubapCnt[j]*2);
	    rs->threadRmx[j]=NULL;
	  }
	}else{
	  printf("Numa rmx for thread %d not found on node %d\n",j,numaIndx);
	  rs->threadRmx[j]=NULL;
	}
      }
    }else{//random allocation of threads to subaps.  But could still define a rmx per numa node, and allow the threads to read the closest one...
      for(j=0;j<reconStruct->nthreads;j++){
	numaIndx=reconStruct->threadToNumaList[j];//get the node this thread is
	snprintf(paramList,17,"gainReconmxT%d",numaIndx);//find the buffer for this thread:
	if(bufferGetIndex((paramBuf*)(pbuf->numaBufs[numaIndx]),1,paramList,&indx,&valptr,&dtp,&nb)==1){//found the part of the reconstruct for this thread.
	  if(dtp=='f' && nb==sizeof(float)*rs->nacts*rs->totCents){
	    rs->threadRmx[j]=(float*)valptr;
	  }else{
	    printf("Error - %s is wrong size or dtype (%c %d, should be %ld)\n",paramList,dtp,nb,sizeof(float)*rs->nacts*rs->totCents);
	    rs->threadRmx[j]=NULL;
	  }
	}else{
	  printf("Numa rmx for thread %d not found on node %d\n",j,numaIndx);
	  rs->threadRmx[j]=NULL;
	}
      }
    }
  }else{
    memset(rs->threadRmx,0,sizeof(float*)*reconStruct->nthreads);
  }



#ifdef USECUDA
  //  cudaError_t status;

  if(reconStruct->curmxsize<rs->nacts*rs->totCents){
    msg[2]=1;//realloc mvm
  }
  if(reconStruct->cucentroidssize<rs->totCents){
    msg[3]=1;//realloc centroids.
  }
  pthread_mutex_lock(&reconStruct->cudamutex);
  if(mq_send(reconStruct->mq,(char*)msg,sizeof(int)*4,0)!=0){
    printf("Error in mq_send in reconmvm\n");
  }
  pthread_mutex_unlock(&reconStruct->cudamutex);
#endif
  //reconStruct->swap=1;
  return err;
}


/**
   Initialise the reconstructor module
 */
int reconOpen(char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **reconHandle,int nthreads,unsigned int frameno,unsigned int **reconframeno,int *reconframenoSize,int totCents){
  //Sort through the parameter buffer, and get the things we need, and do
  //the allocations we need.
  ReconStruct *reconStruct;
  //ReconStructEntry *rs;
  int err=0;
#ifdef USECUDA
  mqd_t mq;
  struct mq_attr attr;

#endif
  if((reconStruct=calloc(sizeof(ReconStruct),1))==NULL){
    printf("Error allocating recon memory\n");
    *reconHandle=NULL;
    //threadInfo->globals->reconStruct=NULL;
    return 1;
  }
  //threadInfo->globals->reconStruct=(void*)reconStruct;
  *reconHandle=(void*)reconStruct;
  reconStruct->buf=1;
  reconStruct->arr=arr;
  reconStruct->nthreads=nthreads;//this doesn't change.
  reconStruct->rtcErrorBuf=rtcErrorBuf;
  reconStruct->paramNames=reconMakeNames();
#ifndef USECUDA
  if(posix_memalign((void**)&reconStruct->rs[0].dmCommandArr,ARRAYALIGN,sizeof(float*)*nthreads)!=0){
    printf("Error allocating recon memory[0]\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }else{
    memset(reconStruct->rs[0].dmCommandArr,0,sizeof(float*)*nthreads);
  }
  //if((reconStruct->rs[1].dmCommandArr=calloc(sizeof(float*),nthreads))==NULL){
  if(posix_memalign((void**)&reconStruct->rs[1].dmCommandArr,ARRAYALIGN,sizeof(float*)*nthreads)!=0){
    printf("Error allocating recon memory[1]\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }else{
    memset(reconStruct->rs[1].dmCommandArr,0,sizeof(float*)*nthreads);
  }
#endif
  reconStruct->rs[0].threadSubapCnt=calloc(sizeof(int),nthreads);
  reconStruct->rs[1].threadSubapCnt=calloc(sizeof(int),nthreads);
  reconStruct->rs[0].threadRmx=calloc(sizeof(float*),nthreads);
  reconStruct->rs[1].threadRmx=calloc(sizeof(float*),nthreads);
  reconStruct->centIndxTot=calloc(sizeof(int),nthreads);

#ifdef USECUDA

  if(n>0)
    reconStruct->deviceNo=args[0];
  if(n>1)
    reconStruct->threadPriority=args[1];
  if(n>2 && args[2]>0 && n>args[2]+2){
    reconStruct->threadAffinElSize=args[2];
    if((reconStruct->threadAffinity=calloc(sizeof(unsigned int),args[2]))==NULL){
      printf("Error allocing threadAffin in reconmvm\n");
      reconClose(reconHandle);
      *reconHandle=NULL;
      return 1;
    }
    memcpy(reconStruct->threadAffinity,&args[3],sizeof(unsigned int)*args[2]);
  }
#ifdef MYCUBLAS
  reconStruct->numThreadsPerBlock=416;//this should be optimised for your specific case
  if(n>2 && n>args[2]+3)
    reconStruct->numThreadsPerBlock=args[3+args[2]];
  printf("%d %d %d %u %d\n",reconStruct->deviceNo,reconStruct->threadPriority,reconStruct->threadAffinElSize,reconStruct->threadAffinity==NULL?0xffffffff:reconStruct->threadAffinity[0],reconStruct->numThreadsPerBlock);
#endif

  //create a message queue for talking to the cuda thread.
  if(prefix==NULL)
    err=asprintf(&reconStruct->mqname,"/rtcmqueue");
  else
    err=asprintf(&reconStruct->mqname,"/%srtcmqueue",prefix);
  if(err==-1){
    printf("Unable to create queue name in reconmvm\n");
    reconStruct->mqname=NULL;
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if((mq=mq_open(reconStruct->mqname,O_WRONLY|O_CREAT|O_EXCL,0x777,NULL))==-1){
    printf("Error creating mqueue %s in reconmvm: %s\n",reconStruct->mqname,strerror(errno));
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if(mq_getattr(mq,&attr)!=0){
    printf("Error getting mq_getattr in reconmvm: %s\n",strerror(errno));
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  reconStruct->msgsize=attr.mq_msgsize;
  reconStruct->go=1;
  printf("msgsize %d in reconmvm\n",(int)reconStruct->msgsize);
  reconStruct->mq=mq;
  pthread_create(&reconStruct->threadid,NULL,reconWorker,reconStruct);
  if(pthread_mutex_init(&reconStruct->cudamutex,NULL)){
    printf("Error init recon cudamutex\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if(pthread_cond_init(&reconStruct->cudacond,NULL)){
    printf("Error init recon cudacond\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }


#endif

  err=reconNewParam(*reconHandle,pbuf,frameno,arr,totCents);//this will change ->buf to 0.
  //rs->swap=0;//no - we don't need to swap.
  //rs=&reconStruct->rs[reconStruct->buf];
  if(err!=0){
    printf("Error in recon...\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  // the condition variable and mutex don't need to be buffer swaped...
  if(darc_mutex_init(&reconStruct->dmMutex,darc_mutex_init_NULL)){
    printf("Error init recon mutex\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
//   if(darc_cond_init(&reconStruct->dmCond,NULL)){
//     printf("Error init recon cond\n");
//     reconClose(reconHandle);
//     *reconHandle=NULL;
//     return 1;
//   }
  if(darc_condwait_init_tovalue(&reconStruct->dmCondwait,0)){
    printf("Error init recon condwait\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }

  return 0;
}


/**
   Called by single thread at the start of each frame.
   This thread is not a subap processing thread.
   If doing a param buffer swap, this is called after the swap has completed.
*/
int reconNewFrame(void *reconHandle,unsigned int frameno,double timestamp){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  ReconStructEntry *rs;
#if !defined(USEAGBBLAS) && !defined(USECUDA) || defined(USEMKL)
  CBLAS_ORDER order=CblasRowMajor;
  CBLAS_TRANSPOSE trans=CblasNoTrans;
  float alpha=1.,beta=0.;
  int inc=1;
#endif
  int i;
  float *dmCommand=reconStruct->arr->dmCommand;
  //if(reconStruct->swap){
  // reconStruct->swap=0;
  // reconStruct->buf=1-reconStruct->buf;
  //}
  rs=&reconStruct->rs[reconStruct->buf];
  //Now wake up the thread that does the initial processing...
  //No need to get a mutex here, because the preprocessing thread must be sleeping.
  //We do this processing in a separate thread because it has a while to complete, so we may as well let subap processing threads get on with their tasks...
  //pthread_cond_signal(&reconStruct->dmCond);
  if(rs->reconMode==RECONMODE_SIMPLE){//simple open loop
    //memset(p->dmCommand,0,sizeof(float)*p->nacts);
    if(rs->v0==NULL)
      memset(dmCommand,0,sizeof(float)*rs->nacts);
    else
      memcpy(dmCommand,rs->v0,sizeof(float)*rs->nacts);
  }else if(rs->reconMode==RECONMODE_TRUTH){//closed loop
    if(rs->decayFactor==NULL){
      memcpy(dmCommand,reconStruct->latestDmCommand,sizeof(float)*rs->nacts);
    }else{
      for(i=0; i<rs->nacts; i++){
	dmCommand[i]=rs->decayFactor[i]*reconStruct->latestDmCommand[i];
      }
    }
  }else if(rs->reconMode==RECONMODE_OPEN){//reconmode_open
    //If gainE matrices are defined, polc is done in reconStartFrame (shared between threads).
    if(rs->polc==0){//no POLC.
      memcpy(dmCommand,reconStruct->latestDmCommand,sizeof(float)*rs->nacts);
      if(rs->v0!=NULL){
#ifdef USEMKL
        cblas_saxpy(rs->nacts,1.,rs->v0,1,dmCommand,1);
#elif defined(USEAGBBLAS)
	agb_cblas_saxpy111(rs->nacts,rs->v0,dmCommand);
#else
        printf("Error: No cblas lib defined in Makefile\n");
        return 1;
#endif
      }
    }
    /*
    if(rs->gainE!=NULL){
#ifdef USEMKL
      cblas_sgemv(order,trans,rs->nacts,rs->nacts,alpha,rs->gainE,rs->nacts,reconStruct->latestDmCommand,inc,beta,dmCommand,inc);
#elif defined(USEAGBBLAS)
      agb_cblas_sgemvRowNN1N101(rs->nacts,rs->gainE,reconStruct->latestDmCommand,dmCommand);
#elif defined(USECUDA)
    //for now, since gainE isn't in gpu... note, if put in gpu - take care since gpu assumes column major... might need to set the transpose flag.
      agb_cblas_sgemvRowNN1N101(rs->nacts,rs->gainE,reconStruct->latestDmCommand,dmCommand);
#else
  printf("Error: No cblas lib defined in Makefile\n");
  return 1;
#endif
    }else{//gainE==NULL...
      memcpy(dmCommand,reconStruct->latestDmCommand,sizeof(float)*rs->nacts);
      //and add v0
      if(rs->v0!=NULL){
  #ifdef USEMKL
  cblas_saxpy(rs->nacts,1.,rs->v0,1.,dmCommand,1.0);
  #elif defined(USEAGBBLAS)
	agb_cblas_saxpy111(rs->nacts,rs->v0,dmCommand);
  #else
  printf("Error: No cblas lib defined in Makefile\n");
  return 1;
  #endif
      }
    }
    //For if polc frame delay >2...
    if(rs->gainE2!=NULL){
      //gainE already done, so need to add to dmCommand, rather than replace.
#ifdef USEMKL
      beta=1.;
      cblas_sgemv(order,trans,rs->nacts,rs->nacts,alpha,rs->gainE2,rs->nacts,reconStruct->latestDmCommand2,inc,beta,dmCommand,inc);
#elif defined(USEAGBBLAS)
      agb_cblas_sgemvRowMN1N111(rs->nacts,rs->nacts,rs->gainE2,reconStruct->latestDmCommand2,dmCommand);
#elif defined(USECUDA)
      //for now, since gainE isn't in gpu... note, if put in gpu - take care since gpu assumes column major... might need to set the transpose flag.
      agb_cblas_sgemvRowMN1N111(rs->nacts,rs->nacts,rs->gainE2,reconStruct->latestDmCommand2,dmCommand);
#else
      printf("Error: No cblas lib defined in Makefile\n");
      return 1;

#endif
    }
    */
  }else{//reconmode_offset
    if(rs->v0==NULL)
      memset(dmCommand,0,sizeof(float)*rs->nacts);
    else
      memcpy(dmCommand,rs->v0,sizeof(float)*rs->nacts);
  }
#ifdef USECUDA
  //CUDA calls can only be made by 1 thread, not this one, so have to inform the correct (subap-processing) thread that it needs to update.
  /*int msg[1];
    msg[0]=INITFRAME;//gpu needs to upload dmcommand.
  reconStruct->initCommand=dmCommand;
  pthread_mutex_lock(&reconStruct->cudamutex);
  if(mq_send(reconStruct->mq,(char*)msg,sizeof(int),0)!=0){
    printf("Error in mq_send in reconmvmcuda reconNewFrame()\n");
  }
  //and wait for the init to complete
  //mq_receive(reconStruct->mqFromGPU,msg,msgsize,NULL);//is this needed?
  pthread_mutex_unlock(&reconStruct->cudamutex);
  */  //reconStruct->setDmCommand=dmCommand;
#endif
  //set the DM arrays ready.
//   if(darc_mutex_lock(&reconStruct->dmMutex))
//     printf("darc_mutex_lock error in setDMArraysReady: %s\n",strerror(errno));
  // reconStruct->dmReady=1;
  //wake up any of the subap processing threads that are waiting.
//   darc_cond_broadcast(&reconStruct->dmCond);
  darc_condwait_broadcast_withvalue(&reconStruct->dmCondwait,1);
//   darc_mutex_unlock(&reconStruct->dmMutex);
  return 0;
}

/**
   Called once per thread at the start of each frame, possibly simultaneously.
*/
#if !defined(USECUDA)
int reconStartFrame(void *reconHandle,int cam,int threadno){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;//threadInfo->globals->reconStruct;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->buf];
  if(rs->polc==1){//implicit POLC
    //compute dmCommand[a:b] = gainE[a:b] . latestDmCommand
    int nPerThread=((rs->nacts+reconStruct->nthreads-1)/reconStruct->nthreads);
    int start=nPerThread*threadno;
    int end=start+nPerThread;
    float *dmCommand=reconStruct->arr->dmCommand;
    if(end>rs->nacts){
      end=rs->nacts;
      nPerThread=end-start;
    }
    if(start<rs->nacts){
#ifdef USEMKL
      CBLAS_ORDER order=CblasRowMajor;
      CBLAS_TRANSPOSE trans=CblasNoTrans;
      float alpha=1.,beta=0.;
      int inc=1;
      //printf("todo - reocnStartFrame - check sgemv call arg inputs\n");
      cblas_sgemv(order,trans,nPerThread,rs->nacts,alpha,&rs->gainE[start*rs->nacts],rs->nacts,reconStruct->latestDmCommand,inc,beta,&dmCommand[start],inc);
#elif defined(USEAGBBLAS)
      agb_cblas_sgemvRowMN1N101(nPerThread,rs->nacts,&rs->gainE[start*rs->nacts],reconStruct->latestDmCommand,&dmCommand[start]);
#elif defined(USECUDA)
      //for now, since gainE isn't in gpu... note, if put in gpu - take care since gpu assumes column major... might need to set the transpose flag.
      agb_cblas_sgemvRowMN1N101(nPerThread,rs->nacts,&rs->gainE[start*rs->nacts],reconStruct->latestDmCommand,&dmCommand[start]);
#endif
      if(rs->gainE2!=NULL){
      //gainE already done, so need to add to dmCommand, rather than replace.
#ifdef USEMKL
	beta=1.;
	cblas_sgemv(order,trans,nPerThread,rs->nacts,alpha,&rs->gainE2[start*rs->nacts],rs->nacts,reconStruct->latestDmCommand2,inc,beta,&dmCommand[start],inc);
#elif defined(USEAGBBLAS)
	agb_cblas_sgemvRowMN1N111(nPerThread,rs->nacts,&rs->gainE2[start*rs->nacts],reconStruct->latestDmCommand2,&dmCommand[start]);
#elif defined(USECUDA)
      //for now, since gainE isn't in gpu... note, if put in gpu - take care since gpu assumes column major... might need to set the transpose flag.
	agb_cblas_sgemvRowMN1N111(nPerThread,rs->nacts,&rs->gainE2[start*rs->nacts],reconStruct->latestDmCommand2,&dmCommand[start]);
#endif
      }
    }
  }else if(rs->polc==2){//explicit POLC...  this is a VERY stupid way of doing POLC.  Just inserted for tests to satisfy a referee for a publication.  This will also mess up adaptive windowing (since slopes are modified).
    //compute slopes[a:b] += gainE[a:b] . latestDmCommand
    float *centroids=rs->polcCentroids;
    int nPerThread=((rs->totCents+reconStruct->nthreads-1)/reconStruct->nthreads);
    int start=nPerThread*threadno;
    int end=start+nPerThread;
    if(end>rs->totCents){
      end=rs->totCents;
      nPerThread=end-start;
    }
    if(start<rs->totCents){
#ifdef USEMKL
      CBLAS_ORDER order=CblasRowMajor;
      CBLAS_TRANSPOSE trans=CblasNoTrans;
      float alpha=1.,beta=0.;
      int inc=1;
      //printf("todo - reocnStartFrame - check sgemv call arg inputs\n");
      cblas_sgemv(order,trans,nPerThread,rs->nacts,alpha,&rs->gainE[start*rs->nacts],rs->nacts,reconStruct->latestDmCommand,inc,beta,&centroids[start],inc);
#elif defined(USEAGBBLAS)
      agb_cblas_sgemvRowMN1N101(nPerThread,rs->nacts,&rs->gainE[start*rs->nacts],reconStruct->latestDmCommand,&centroids[start]);
#elif defined(USECUDA)
      //for now, since gainE isn't in gpu... note, if put in gpu - take care since gpu assumes column major... might need to set the transpose flag.
      agb_cblas_sgemvRowMN1N101(nPerThread,rs->nacts,&rs->gainE[start*rs->nacts],reconStruct->latestDmCommand,&centroids[start]);
#endif
      if(rs->gainE2!=NULL){
      //gainE already done, so need to add to dmCommand, rather than replace.
#ifdef USEMKL
	beta=1.;
	cblas_sgemv(order,trans,nPerThread,rs->nacts,alpha,&rs->gainE2[start*rs->nacts],rs->nacts,reconStruct->latestDmCommand2,inc,beta,&centroids[start],inc);
#elif defined(USEAGBBLAS)
	agb_cblas_sgemvRowMN1N111(nPerThread,rs->nacts,&rs->gainE2[start*rs->nacts],reconStruct->latestDmCommand2,&centroids[start]);
#elif defined(USECUDA)
      //for now, since gainE isn't in gpu... note, if put in gpu - take care since gpu assumes column major... might need to set the transpose flag.
	agb_cblas_sgemvRowMN1N111(nPerThread,rs->nacts,&rs->gainE2[start*rs->nacts],reconStruct->latestDmCommand2,&centroids[start]);
#endif
      }
    }
  }
  if(rs->polc!=0){
    //increment counter.  Possibly better to do this atomically using intrinsics.
    darc_mutex_lock(&reconStruct->dmMutex);
    reconStruct->polcCounter++;
    darc_mutex_unlock(&reconStruct->dmMutex);

  }

  memset((void*)(rs->dmCommandArr[threadno]),0,rs->nacts*sizeof(float));
  reconStruct->centIndxTot[threadno]=0;//only used for Numa.
  return 0;
}
#endif


/**
   Called multiple times by multiple threads, whenever new slope data is ready
   centroids may not be complete, and writing to dmCommand is not thread-safe without locking.
*/
int reconNewSlopes(void *reconHandle,int cam,int centindx,int threadno,int nsubapsDoing){
#if !defined(USEAGBBLAS) && !defined(USECUDA) || defined(USEMKL)
  CBLAS_ORDER order=CblasColMajor;
  CBLAS_TRANSPOSE trans=CblasNoTrans;
  float alpha=1.,beta=1.;
  int inc=1;
#endif
  int step;//number of rows to do in mmx...
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
#ifndef USECUDA
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->buf];
#endif
  float *centroids=reconStruct->arr->centroids;
  float *rmx=&(rs->rmxT[centindx*rs->nacts])
  //float *dmCommand=reconStruct->arr->dmCommand;
  //infoStruct *info=threadInfo->info;
  //globalStruct *glob=threadInfo->globals;
  //We assume that each row i of the reconstructor has already been multiplied by gain[i].
  //So, here we just do dmCommand+=rmx[:,n]*centx+rmx[:,n+1]*centy.
  dprintf("in partialReconstruct %d %d %d %p %p %p\n",rs->nacts,centindx,rs->totCents,centroids,rs->rmxT,rs->dmCommandArr[threadno]);
  step=2*nsubapsDoing;

  //do we need a numa matrix???
  if(rs->threadRmx[threadno]!=NULL){
    //is this a whole matrix (no subapAllocation), or just the relevant portion?
    if(rs->subapAlloc!=NULL){//threads have defined subaps.
      //centindx will only increase for a particular thread.  And since subapAllocation is defined, this will be known.  Therefore keep a note of where we are.
      rmx=&((rs->threadRmx[threadno])[reconStruct->centIndxTot[threadno]*rs->nacts]);
      reconStruct->centIndxTot[threadno]+=step;
    }else{//whole matrix, but in the correct Numa area.
      rmx=rs->threadRmx[threadno];
    }
  }
  if(rs->polc==2){//explicit polc.
    //wait until the POL slopes are ready, then add these to the centroids that we're investigating here.
    while(reconStruct->polcCounter<reconStruct->nthreads){//busy wait
    }
    int i;
    for(i=centindx;i<centindx+step;i++){
      centroids[i]+=rs->polcCentroids[i];
    }
  }
#ifdef USEMKL
  cblas_sgemv(order,trans,rs->nacts,step,alpha,rmx,rs->nacts,&(centroids[centindx]),inc,beta,rs->dmCommandArr[threadno],inc);
#elif defined(USEAGBBLAS)
#ifndef DUMMY
  #ifdef USEICC
  agb_cblas_32sgemvColMN1M111(rs->nacts,step,(void*)rmx,&(centroids[centindx]),rs->dmCommandArr[threadno]);
  #else
  agb_cblas_sgemvColMN1M111(rs->nacts,step,rmx,&(centroids[centindx]),rs->dmCommandArr[threadno]);
  #endif
#endif
#elif defined(USECUDA)
  //Need to wait here until the INITFRAME has been done...
#ifdef MYCUBLAS
  if(reconStruct->rs[reconStruct->buf].newswap)
    printf("MVM size %d %d\n",reconStruct->rs[reconStruct->buf].nacts,step);
#endif

  int msg[3];
  msg[0]=DOMVM;
  msg[1]=step;
  msg[2]=centindx;
  reconStruct->centroids=centroids;
  pthread_mutex_lock(&reconStruct->cudamutex);
  if(mq_send(reconStruct->mq,(char*)msg,sizeof(int)*3,0)!=0)
    printf("error in mq_send in reconNewSlopes\n");
  pthread_mutex_unlock(&reconStruct->cudamutex);
#else
  printf("Error: No cblas lib defined in Makefile\n");
  return 1;
#endif
  return 0;
}

/* treeAdd function */
void treeAdd(ReconStruct *reconStruct, int threadno){
    /* Tree vector add function */
    ReconStructEntry *rs = &reconStruct->rs[reconStruct->buf];
    /* initialise vars */

    int partNo1 = rs->partArray[threadno];

    pthread_spin_lock(&rs->partSpins[partNo1]);
    #ifdef USEMKL
    cblas_saxpy(rs->nacts,1.,rs->dmCommandArr[threadno],1,rs->output[partNo1],1);
    #else
    agb_cblas_saxpy111(rs->nacts,rs->dmCommandArr[threadno],rs->output[partNo1]);
    #endif
    rs->barrierFinished[partNo1]++;
    pthread_spin_unlock(&rs->partSpins[partNo1]);

    if(rs->barrierWaits[threadno]<=0){
      return;
    }

    int partNo2 = rs->partArray[reconStruct->nthreads+threadno];

    while(rs->barrierFinished[partNo1]<rs->barrierWaits[threadno] && rs->barrierFinished[partNo1]!=0){
      // busy wait
    }
    rs->barrierFinished[partNo1]=0;

    pthread_spin_lock(&rs->partSpins[partNo2]);
    #ifdef USEMKL
    cblas_saxpy(rs->nacts,1.,rs->output[partNo1],1,rs->output[partNo2],1);
    #else
    agb_cblas_saxpy111(rs->nacts,rs->output[partNo1],rs->output[partNo2]);
    #endif
    rs->barrierFinished[partNo2]++;
    pthread_spin_unlock(&rs->partSpins[partNo2]);
    memset(rs->output[partNo1],0,rs->nacts*sizeof(float));

    return;
}

/**
   Called once for each thread at the end of a frame
   Here we sum the individual dmCommands together to get the final one.
   centroids may not be complete, and writing to dmCommand is not thread-safe without locking.
*/
int reconEndFrame(void *reconHandle,int cam,int threadno,int err){
  //dmCommand=glob->arrays->dmCommand;
  //globalStruct *glob=threadInfo->globals;
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
#if !defined(USECUDA)
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->buf];
#endif
  if(rs->polc==1){
    //wait until the POL is all done in reconStartFrame.
    while(reconStruct->polcCounter<reconStruct->nthreads){
      //busy wait ...
    }

  }

  //#ifdef USETREEADD
  if(rs->nparts>0){
    // while(reconStruct->dmReady==0){  //DJ: removed the mutex and cond_wait above, busy wait on dmReady instead
    //   // busy wait ...
    // }
    darc_condwait_wait_ifvalue(&reconStruct->dmCondwait,0); // drj 140422: changed busy wait to futex condwait, busy waiting needs more careful consideration
    treeAdd(reconStruct,threadno);
  }else{//not using the butterfly treeAdd addition...
  //#else
#ifndef USECUDA
    float *dmCommand=reconStruct->arr->dmCommand;
#endif
//   if(darc_mutex_lock(&reconStruct->dmMutex))
//     printf("darc_mutex_lock error in copyThreadPhase: %s\n",strerror(errno));
//   if(reconStruct->dmReady==0)//wait for the precompute thread to finish (it will call setDMArraysReady when done)...
//     if(darc_cond_wait(&reconStruct->dmCond,&reconStruct->dmMutex))
//       printf("pthread_cond_wait error in copyThreadPhase: %s\n",strerror(errno));
    darc_condwait_wait_ifvalue(&reconStruct->dmCondwait,0);
    darc_mutex_lock(&reconStruct->dmMutex);
  //now add threadInfo->dmCommand to threadInfo->info->dmCommand.
#ifdef USEMKL
    cblas_saxpy(rs->nacts,1.,rs->dmCommandArr[threadno],1,dmCommand,1);
#elif defined(USEAGBBLAS)
    agb_cblas_saxpy111(rs->nacts,rs->dmCommandArr[threadno],dmCommand);
#elif defined(USECUDA)
#else
    printf("Error: No cblas lib defined in Makefile\n");
    return 1;
#endif

    darc_mutex_unlock(&reconStruct->dmMutex);
  //#endif
  }
  return 0;
}

/**
   Called by single thread per frame at the end of frame.
   This is called by a subaperture processing thread.
   The bare minimum should be placed here, as most processing should be done in the reconFrameFinished function instead, which doesn't hold up processing.
*/
int reconFrameFinishedSync(void *reconHandle,int err,int forcewrite){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  //float *centroids=reconStruct->arr->centroids;
  //float *dmCommand=reconStruct->arr->dmCommand;
#ifdef USECUDA
  int msg=ENDFRAME;
#ifdef MYCUBLAS
  reconStruct->rs[reconStruct->buf].newswap=0;
#endif
  reconStruct->dmCommand=reconStruct->arr->dmCommand;
  pthread_mutex_lock(&reconStruct->cudamutex);
  reconStruct->retrievedDmCommand=0;
  if(mq_send(reconStruct->mq,(char*)(&msg),sizeof(int),0)!=0)
    printf("Error in mq_send in reconFrameFinishedSync\n");
  pthread_mutex_unlock(&reconStruct->cudamutex);
#endif
  //if(pthread_mutex_lock(&reconStruct->dmMutex))
  //  printf("pthread_mutex_lock error in copyThreadPhase: %s\n",strerror(errno));
  //No need to get the lock here.
  reconStruct->polcCounter=0;
  darc_condwait_setvalue(&reconStruct->dmCondwait,0);
  //pthread_mutex_unlock(&reconStruct->dmMutex);
  reconStruct->postbuf=reconStruct->buf;
  return 0;
}
/**
   Called by single thread per frame - end of frame
   Do any post processing here.
   This it typically called by a non-subaperture processing thread.
   At the end of this method, dmCommand must be ready...
   Note, while this is running, subaperture processing of the next frame may start.
*/
int reconFrameFinished(void *reconHandle,int *err){//globalStruct *glob){
  //Note: dmCommand=glob->arrays->dmCommand.
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;//glob->reconStruct;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->postbuf];
  float *bleedVal=rs->bleedVal;
  int i,bleedGroup;
  float *dmCommand=reconStruct->arr->dmCommand;
#ifdef USECUDA
  pthread_mutex_lock(&reconStruct->cudamutex);
  if(reconStruct->retrievedDmCommand==0){
    pthread_cond_wait(&reconStruct->cudacond,&reconStruct->cudamutex);
  }
  pthread_mutex_unlock(&reconStruct->cudamutex);
  //mq_receive(reconStruct->mqFromGPU,msg,msgsize,NULL);//wait for dmCommand to be transferred from the GPU.
#endif
  if(rs->bleedGain!=0. || rs->bleedGainArr!=NULL){//compute the bleed value
    memset(bleedVal,0,sizeof(float)*rs->bleedGroups);
    if(rs->v0==NULL){
      for(i=0; i<rs->nacts; i++){
	if(rs->bleedGroupArr!=NULL)
	  bleedGroup=rs->bleedGroupArr[i];
	else
	  bleedGroup=0;
	bleedVal[bleedGroup]+=dmCommand[i];
      }
    }else{
      for(i=0; i<rs->nacts; i++){
	if(rs->bleedGroupArr!=NULL)
	  bleedGroup=rs->bleedGroupArr[i];
	else
	  bleedGroup=0;
	bleedVal[bleedGroup]+=dmCommand[i]-rs->v0[i];
      }
    }
    if(rs->bleedGainArr==NULL){
      for(i=0;i<rs->bleedGroups;i++)
	bleedVal[i]*=rs->bleedGain;
    }else{
      for(i=0;i<rs->bleedGroups;i++)
	bleedVal[i]*=rs->bleedGainArr[i];
    }
    for(i=0; i<rs->nacts; i++){
      if(rs->bleedGroupArr!=NULL)
	bleedGroup=rs->bleedGroupArr[i];
      else
	bleedGroup=0;
      dmCommand[i]-=bleedVal[bleedGroup];
    }
  }
  if(*err==0){
    if(rs->gainE2!=NULL && reconStruct->latestDmCommand2!=NULL)
      memcpy(reconStruct->latestDmCommand2,reconStruct->latestDmCommand,sizeof(float)*rs->nacts);
    memcpy(reconStruct->latestDmCommand,dmCommand,sizeof(float)*rs->nacts);
  }
  return 0;
}
/**
   Called by the single thread per frame, when the actuator values aren't being sent to the dm - so we need to reset ourselves.
   May not be called at all, if the loop is closed.
   Not called by a subap processing thread - subap processing for next frame have have started before this is called...
*/
int reconOpenLoop(void *reconHandle){//globalStruct *glob){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;//glob->reconStruct;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->postbuf];
  if(rs->v0==NULL)
    memset(reconStruct->latestDmCommand,0,sizeof(float)*rs->nacts);
  else
    memcpy(reconStruct->latestDmCommand,rs->v0,sizeof(float)*rs->nacts);
  if(rs->gainE2!=NULL && reconStruct->latestDmCommand2!=NULL)
    memset(reconStruct->latestDmCommand2,0,sizeof(float)*rs->nacts);
  return 0;

}

