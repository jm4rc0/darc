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
THIS IS DEPRECIATED - USE RECONMVM.C INSTREAD, DEFINING USECUBLAS.
/**
   This is a library that can be used for a simple MVM.
*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#ifdef USEAGBBLAS
#include "agbcblas.h"
#elif defined(USECUBLAS)
#include <cublas.h>
#include <cuda_runtime.h>
#include "agbcblas.h"
#else
#include <gsl/gsl_cblas.h>
typedef enum CBLAS_ORDER CBLAS_ORDER;
typedef enum CBLAS_TRANSPOSE CBLAS_TRANSPOSE;
#endif
#include "darc.h"
#include "circ.h"
#include "rtcrecon.h"

typedef enum{RECONMODE_SIMPLE,RECONMODE_TRUTH,RECONMODE_OPEN,RECONMODE_OFFSET}ReconModeType;

typedef enum{
  GAINRECONMXT,
  RECONSTRUCTMODE,
  GAINE,
  V0,
  BLEEDGAIN,
  DECAYFACTOR,
  //MIDRANGE,
  NACTS,
  //Add more before this line.
  RECONNBUFFERVARIABLES//equal to number of entries in the enum
}RECONBUFFERVARIABLEINDX;
char *RECONPARAM[]={"gainReconmxT","reconstructMode","gainE","v0","bleedGain","decayFactor","nacts"};//,"midrange"};


typedef struct{
  ReconModeType reconMode;
  float *gainE;
  int dmCommandArrSize;
  float *dmCommandArr;
  float *rmxT;
  float *v0;
  float bleedGainOverNact;
  //float midRangeTimesBleed;
  float *decayFactor;
  int nacts;
  int totCents;
#ifdef USECUBLAS
  float **cudmCommandArr;
  float *cudmCommand;
  float *curmx;
  float **cucentroids;
  int cucentroidssize;
  int curmxsize;
#endif
}ReconStructEntry;

typedef struct{
  ReconStructEntry rs[2];
  int buf;//current buffer being used
  int postbuf;//current buffer for post processing threads.
  //int swap;//set if need to change to the other buffer.
  int dmReady;
  float *latestDmCommand;
  int latestDmCommandSize;
  pthread_mutex_t dmMutex;
  pthread_cond_t dmCond;
  int bufindx[RECONNBUFFERVARIABLES];
  circBuf *rtcErrorBuf;
  int nthreads;
#ifdef USECUBLAS
  float *setDmCommand;
#endif
}ReconStruct;

/**
   Called to free the reconstructor module when it is being closed.
*/
int reconFree(void **reconHandle){//reconHandle is &globals->reconStruct.
  ReconStruct *reconStruct=(ReconStruct*)*reconHandle;
  ReconStructEntry *rs;
  printf("Closing reconlibrary\n");
  if(reconStruct!=NULL){
    pthread_mutex_destroy(&reconStruct->dmMutex);
    pthread_cond_destroy(&reconStruct->dmCond);
    rs=&reconStruct->rs[0];
    if(reconStruct->latestDmCommand!=NULL)
      free(reconStruct->latestDmCommand);
    if(rs->dmCommandArr!=NULL)
      free(rs->dmCommandArr);
#ifdef USECUBLAS
    if(rs->cudmCommand!=NULL)
      cublasFree(rs->cudmCommand);
    if(rs->cudmCommandArr!=NULL){
      int i;
      for(i=0; i<reconStruct->nthreads; i++){
	if(rs->cudmCommandArr[i]!=NULL)
	  cublasFree(rs->cudmCommandArr);
      }
      free(rs->cudmCommandArr);
    }
    if(rs->curmx!=NULL)
      cublasFree(rs->curmx);
    if(rs->cucentroids!=NULL){
      int i;
      for(i=0; i<reconStruct->nthreads; i++){
	if(rs->cucentroids[i]!=NULL)
	  cublasFree(rs->cucentroids[i]);
      }
      free(rs->cucentroids);
    }
#endif
    rs=&reconStruct->rs[1];
    if(rs->dmCommandArr!=NULL)
      free(rs->dmCommandArr);
#ifdef USECUBLAS
    if(rs->cudmCommand!=NULL)
      cublasFree(rs->cudmCommand);
    if(rs->cudmCommandArr!=NULL){
      int i;
      for(i=0; i<reconStruct->nthreads; i++){
	if(rs->cudmCommandArr[i]!=NULL)
	  cublasFree(rs->cudmCommandArr);
      }
      free(rs->cudmCommandArr);
    }
    if(rs->curmx!=NULL)
      cublasFree(rs->curmx);
    if(rs->cucentroids!=NULL){
      int i;
      for(i=0; i<reconStruct->nthreads; i++){
	if(rs->cucentroids[i]!=NULL)
	  cublasFree(rs->cucentroids[i]);
      }
      free(rs->cucentroids);
    }
#endif
    free(reconStruct);
  }
  *reconHandle=NULL;
  //threadInfo->globals->reconStruct=NULL;
#ifdef USECUBLAS
  printf("calling cublasShutdown\n");
  cublasShutdown();
#endif
  printf("Finished reconFree\n");
  return 0;
}

#ifdef USECUBLAS
//with cuda only 1 thread can access the GPU (actually not strictly true, but simplifies stuff if you assume this).  So, use a mqueue implementation here.
void *reconWorker(void *reconHandle){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  ReconStructEntry *rs;
  int i;
  int *msg;
  int msgsize;
  mqd_t mq;
  rs=&reconStruct->rs[reconStruct->buf];
  if((mq=mq_open(reconStruct->mqname,O_RDWR))==-1){
    printf("Error opening mqueue %s in reconmvm\n",reconStruct->mqname);
    reconStruct->err=1;
    return NULL;
  }
  if((msg=malloc(reconStruct->msgsize))==NULL){
    printf("Error allocating msg for mq in reconmvm\n");
    reconStruct->err=1;
    return NULL;
  }



  printf("Initialising CUDA\n");
  cudaError_t status;
  if((status=cublasInit())!=CUBLAS_STATUS_SUCCESS){
    printf("CUBLAS init error\n");
    return NULL;
  }else{
    printf("CUDA initialised\n");
  }


  while(reconStruct->go){
    //first of all, read the queue until we receive an end of frame message.
    //Then, finish the GPU stuff and send a frame ended message.
    if(mq_receive(mq,msg,bufsize,NULL)==-1){
      printf("Error in mq_receive in reconmvm: %s",strerror(errno));
    }else{
      if(msg[0]==INITFRAME){//initialise the work vector to dmCommand
	//upload dmCommand
	if((status=cublasSetVector(rs->nacts,sizeof(float),reconStruct->dmCommand,1,cudmCommand,1))!=CUBLAS_STATUS_SUCCESS){
	  printf("cudamemset cublasSetVector failed  status=%d\n",(int)status);
	}
	mq_send();//is this needed?  Possibly not.
      }else if(msg[0]==DOMVM){//do the mvm
	error_t status;
	step=msg[1];//step;
	centindx=msg[2];//centindx;
	//copy the centroids to GPU cucentroids==d_B
	//printf("addr %p\n",&(rs->cucentroids[threadno][centindx]));
	//printf("cublasSetVector(centroids)\n");
	if((status = cublasSetVector(step, sizeof(float),&(rs->centroids[centindx]), 1, &(cucentroids[centindx]), 1))!=CUBLAS_STATUS_SUCCESS){
	  printf("!!!! device access error (write centroid vector):error no=%d\n",(int)status);
	}
	//Do the GEMV., curmx==d_A
	//printf("cublasSgemv\n");
	cublasSgemv('n',rs->nacts,step,1.,&(curmx[(centindx*rs->nacts)]), rs->nacts, &(cucentroids[centindx]), 1, 1.,cudmCommand, 1);
	if((status=cublasGetError())!=CUBLAS_STATUS_SUCCESS)
	  printf("Error in cublasSgemv\n");


      }else if(msg[0]==ENDFRAME){//end the frame - get the results.
	if((status=cublasGetVector(rs->nacts, sizeof(float), cudmCommand,1,rs->dmCommand,1))!=CUBLAS_STATUS_SUCCESS){
	  printf ("cuda device access error (get dmCommand vector) %d\n",(int)status);
	}
	//and now tell the rtc that data has been received.
	msg[0]=FRAMEENDED;
	mq_send(mqFromGPU,msg,sizeof(int),0);
      }else if(msg[0]==UPLOAD){//upload the rmx etc
	if(msg[1]==1){
	  cudaError_t status;
	  int i;
	  if(rs->cudmCommand!=NULL)
	    cublasFree(rs->cudmCommand);
	  if((status=cublasAlloc(rs->nacts,sizeof(float),(void**)(&rs->cudmCommand)))!=CUBLAS_STATUS_SUCCESS){
	    printf("device mem alloc error (cudmcommand\n");
	    err=-2;
	  }
	  for(i=0; (err==0 && i<reconStruct->nthreads); i++){
	    if(rs->cudmCommandArr[i]!=NULL)
	      cublasFree(rs->cudmCommandArr[i]);
	    if((status=cublasAlloc(rs->nacts,sizeof(float),(void**)&rs->cudmCommandArr[i]))!=CUBLAS_STATUS_SUCCESS){
	      printf("device memory allocation error (cudmcommandArr[%d])\n",i);
	      err=-2;
	    }else{
	      if((status=cudaMemset(cudmCommand,0,rs->nacts*sizeof(float)))!=cudaSuccess){
		printf("device access error (memset cudmcommandarr[%d]) errno: %d\n",i,status);
		err=-2;
	      }
	    }
	    
	  }
	}
	if(msg[2]==1){
	  if(curmx!=NULL)
	    cublasFree(curmx);
	  if((status=cublasAlloc(rs->nacts*rs->totCents,sizeof(float),(void**)&curmx))!=CUBLAS_STATUS_SUCCESS){
	    printf("device memory allocation error (curmx)\n");
	    err=-4;
	    rs->curmxsize=0;
	  }else{
	    rs->curmxsize=rs->nacts*rs->totCents;
	  }
	}
	if(msg[3]==1){
	  if(cucentroids!=NULL)
	    cublasFree(cucentroids);
	  if((status=cublasAlloc(rs->totCents,sizeof(float),(void**)&cucentroids))!=CUBLAS_STATUS_SUCCESS){
	    printf("device memory allocation error (cucentroids[%d])\n",j);
	    err=-5;
	  }
	  if(err!=0){
	    cucentroidssize=0;
	  }else{
	    cucentroidssize=rs->totCents;
	  }
	}
	//upload the rmx.
	if((status=cublasSetVector(rs->totCents*rs->nacts,sizeof(float),rs->rmxT,1,curmx,1))!=CUBLAS_STATUS_SUCCESS){
	  printf("device access error (write rmx)\n");
	  err=2;
	}
      }else{//unrecognised message
	printf("Message not recognised in reconmvm\n");
      }
    }
					    
  }

}

#endif


/**
   Called asynchronously, whenever new parameters are ready.
   Once this returns, a call to swap buffers will be issued.
   (actually, at the moment, this is called synchronously by first thread when a buffer swap is done).
*/
int reconNewParam(char *buf,void *reconHandle,unsigned int frameno,arrayStruct *arr,int totCents){
  int j=0,err=0;
  int nbytes;
  //globalStruct *globals=threadInfo->globals;
  //infoStruct *info=threadInfo->info;//totcents and nacts
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;//threadInfo->globals->reconStruct;
  int *indx=reconStruct->bufindx;
  //Use the buffer not currently in use.
  ReconStructEntry *rs;
  RECONBUFFERVARIABLEINDX i;
#ifdef USECUBLAS
  int msg[4];
  msg[1]=0;//resize flag
  msg[0]=UPLOAD;//the operation flag
#endif
  //swap the buffers...
  reconStruct->buf=1-reconStruct->buf;
  rs=&reconStruct->rs[reconStruct->buf];
  rs->totCents=totCents;
  //rs->nacts=info->nacts;
  memset(indx,-1,sizeof(int)*RECONNBUFFERVARIABLES);

  //first run through the buffer getting the indexes of the params we need.
  while(j<NHDR && buf[j*31]!='\0'){
    if(strncmp(&buf[j*31],"reconstructMode",31)==0){
      indx[RECONSTRUCTMODE]=j;
    }else if(strncmp(&buf[j*31],"gainE",31)==0){
      indx[GAINE]=j;
    }else if(strncmp(&buf[j*31],"gainReconmxT",31)==0){
      indx[GAINRECONMXT]=j;
    }else if(strncmp(&buf[j*31],"v0",31)==0){
      indx[V0]=j;
      //}else if(strncmp(&buf[j*31],"midRangeValue",31)==0){
      //indx[MIDRANGE]=j;
    }else if(strncmp(&buf[j*31],"bleedGain",31)==0){
      indx[BLEEDGAIN]=j;
    }else if(strncmp(&buf[j*31],"decayFactor",31)==0){
      indx[DECAYFACTOR]=j;
    }else if(strncmp(&buf[j*31],"nacts",31)==0){
      indx[NACTS]=j;
    }
    j++;
  }
  for(j=0; j<RECONNBUFFERVARIABLES; j++){
    if(indx[j]==-1){
      //if(updateIndex){
      printf("ERROR buffer index %d\n",j);
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"Error in recon parameter buffer: %s",RECONPARAM[j]);
      //}
      err=-1;
    }
  }
  if(err==0){
    i=NACTS;
    if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==4){
      rs->nacts=*((int*)(buf+START[indx[i]]));
    }else{
      printf("nacts error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"nacts error");
      err=NACTS;
    }
    i=RECONSTRUCTMODE;
    nbytes=NBYTES[indx[i]];
    if(buf[NHDR*31+indx[i]]=='s'){
      if(strncmp(buf+START[indx[i]],"simple",nbytes)==0){
	rs->reconMode=RECONMODE_SIMPLE;
      }else if(strncmp(buf+START[indx[i]],"truth",nbytes)==0){
	rs->reconMode=RECONMODE_TRUTH;
      }else if(strncmp(buf+START[indx[i]],"open",nbytes)==0){
	rs->reconMode=RECONMODE_OPEN;
      }else if(strncmp(buf+START[indx[i]],"offset",nbytes)==0){
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
    if(buf[NHDR*31+indx[i]]=='f' && NBYTES[indx[i]]/sizeof(float)==rs->totCents*rs->nacts){
      rs->rmxT=(float*)(buf+START[indx[i]]);
    }else{
      printf("gainReconmxT error %c %d %d %d\n",buf[NHDR*31+indx[i]],NBYTES[indx[i]],rs->totCents,rs->nacts);
      err=GAINRECONMXT;
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"gainReconmxT error");
    }
    i=GAINE;
    if(buf[NHDR*31+indx[i]]=='f' && NBYTES[indx[i]]==sizeof(float)*rs->nacts*rs->nacts){
      rs->gainE=(float*)(buf+START[indx[i]]);
    }else{
      printf("gainE error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"gainE");
      err=GAINE;
    }
    i=V0;
    if(buf[NHDR*31+indx[i]]=='f' && NBYTES[indx[i]]==sizeof(float)*rs->nacts){
      rs->v0=(float*)(buf+START[indx[i]]);
    }else{
      printf("v0 error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"v0 error");
      err=V0;
    }
    i=BLEEDGAIN;
    if(buf[NHDR*31+indx[i]]=='f' && NBYTES[indx[i]]==sizeof(float)){
      rs->bleedGainOverNact=(*((float*)(buf+START[indx[i]])))/rs->nacts;
    }else{
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"bleedGain error");
      printf("bleedGain error\n");
      err=BLEEDGAIN;
    }
    i=DECAYFACTOR;
    if(NBYTES[indx[i]]==0){
      rs->decayFactor=NULL;
    }else if(buf[NHDR*31+indx[i]]=='f' && NBYTES[indx[i]]==sizeof(float)*rs->nacts){
      rs->decayFactor=(float*)(buf+START[indx[i]]);
    }else{
      rs->decayFactor=NULL;
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"decayFactor error");
      printf("decayFactor error\n");
      err=DECAYFACTOR;
    }
    /*
    i=MIDRANGE;
    if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
      rs->midRangeTimesBleed=(*((int*)(buf+START[indx[i]])))*rs->nacts*rs->bleedGainOverNact;
    }else{
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"midRangeValue error");
      printf("midrange error\n");
      err=MIDRANGE;
      }*/
  }

  reconStruct->dmReady=0;
  if(rs->dmCommandArrSize<sizeof(float)*rs->nacts*reconStruct->nthreads){
    rs->dmCommandArrSize=sizeof(float)*rs->nacts*reconStruct->nthreads;
    if(rs->dmCommandArr!=NULL)
      free(rs->dmCommandArr);
    if((rs->dmCommandArr=calloc(sizeof(float)*rs->nacts,reconStruct->nthreads))==NULL){
      printf("Error allocating recon dmCommand memory\n");
      err=-2;
      rs->dmCommandArrSize=0;
    }
#ifdef USECUBLAS
    if(err==0){
      msg[1]=1;//need to resize.
      
      /*cudaError_t status;
      int i;
      if(rs->cudmCommand!=NULL)
	cublasFree(rs->cudmCommand);
      if((status=cublasAlloc(rs->nacts,sizeof(float),(void**)(&rs->cudmCommand)))!=CUBLAS_STATUS_SUCCESS){
	printf("device mem alloc error (cudmcommand\n");
	err=-2;
      }
      for(i=0; (err==0 && i<reconStruct->nthreads); i++){
	if(rs->cudmCommandArr[i]!=NULL)
	  cublasFree(rs->cudmCommandArr[i]);
	if((status=cublasAlloc(rs->nacts,sizeof(float),(void**)&rs->cudmCommandArr[i]))!=CUBLAS_STATUS_SUCCESS){
	  printf("device memory allocation error (cudmcommandArr[%d])\n",i);
	  err=-2;
	}else{
	  if((status=cudaMemset(rs->cudmCommandArr[i],0,rs->nacts*sizeof(float)))!=cudaSuccess){
	    printf("device access error (memset cudmcommandarr[%d]) errno: %d\n",i,status);
	    err=-2;
	  }
	}

      }
      if(err!=0){
	rs->dmCommandArrSize=0;
	free(rs->dmCommandArr);
	rs->dmCommandArr=NULL;
	}*/
    }
#endif
  }
  if(reconStruct->latestDmCommandSize<sizeof(float)*rs->nacts){
    reconStruct->latestDmCommandSize=sizeof(float)*rs->nacts;
    if(reconStruct->latestDmCommand!=NULL)
      free(reconStruct->latestDmCommand);
    if((reconStruct->latestDmCommand=calloc(rs->nacts,sizeof(float)))==NULL){
      printf("Error allocating latestDmCommand memory\n");
      err=-3;
      reconStruct->latestDmCommandSize=0;
    }
  }

#ifdef USECUBLAS
  cudaError_t status;

  if(rs->curmxsize<rs->nacts*rs->totCents){
    msg[2]=1;//realloc mvm
    if(rs->curmx!=NULL)
      cublasFree(rs->curmx);
    if((status=cublasAlloc(rs->nacts*rs->totCents,sizeof(float),(void**)&rs->curmx))!=CUBLAS_STATUS_SUCCESS){
      printf("device memory allocation error (curmx)\n");
      err=-4;
      rs->curmxsize=0;
    }else{
      rs->curmxsize=rs->nacts*rs->totCents;
    }
  }
  if(rs->cucentroidssize<rs->totCents){
    msg[3]=1;//realloc centroids.
    for(j=0; (err==0 && j<reconStruct->nthreads); j++){
      if(rs->cucentroids[j]!=NULL)
	cublasFree(rs->cucentroids[j]);
      if((status=cublasAlloc(rs->totCents,sizeof(float),(void**)&rs->cucentroids[j]))!=CUBLAS_STATUS_SUCCESS){
	printf("device memory allocation error (cucentroids[%d])\n",j);
	err=-5;
      }
    }
    if(err!=0){
      rs->cucentroidssize=0;
    }else{
      rs->cucentroidssize=rs->totCents;
    }
  }
  //upload the rmx.
  if((status=cublasSetVector(rs->totCents*rs->nacts,sizeof(float),rs->rmxT,1,rs->curmx,1))!=CUBLAS_STATUS_SUCCESS){
    printf("device access error (write rmx)\n");
    err=2;
  }
  if(mq_send(reconStruct->mq,msg,sizeof(int)*4,0)!=0){
    printf("Error in mq_send in reconmvm\n");
  }
#endif
  //reconStruct->swap=1;
  return err;
}


/**
   Initialise the reconstructor module
 */
int reconOpen(char *name,int n,int *args,char *buf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **reconHandle,int nthreads,int frameno,int totCents){
  //Sort through the parameter buffer, and get the things we need, and do 
  //the allocations we need.
  ReconStruct *reconStruct;
  //ReconStructEntry *rs;
  int err=0;
#ifdef USECUBLAS
  if(nthreads!=1){
    printf("libreconmvmcuda.so does not currently work with >1 thread (a cuda problem)\n");
    *reconHandle=NULL;
    return 1;
  }
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
  reconStruct->nthreads=nthreads;//this doesn't change.
  reconStruct->rtcErrorBuf=rtcErrorBuf;
#ifdef USECUBLAS
  //create a message queue for talking to the cuda thread.
  if(asprintf(&reconStruct->mqname,"%smqueue",reconStruct->prefix)==-1){
    printf("Unable to create queue name in reconmvm\n");
    reconStruct->mqname=NULL;
    reconFree(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if((mq=mq_open(reconStruct->mqname,O_RDWR|O_CREAT|O_EXCL,0x777,NULL))==-1){
    printf("Error creating mqueue in reconmvm\n");
    reconFree(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if(mq_getattr(mq,&attr)!=0){
    printf("Error getting mq_getattr in reconmvm\n");
    reconFree(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  reconStruct->msgsize=attr.mq_msgsize;
  printf("msgsize %d in reconmvm\n",(int)reconStruct->msgsize);
  pthread_create(&threadid,NULL,reconWorker,reconStruct);
  /*
  printf("Initialising CUDA\n");
  cudaError_t status;
  if((status=cublasInit())!=CUBLAS_STATUS_SUCCESS){
    printf("CUBLAS init error\n");
    return 1;
  }else{
    printf("CUDA initialised\n");
    if((reconStruct->rs[0].cudmCommandArr=calloc(sizeof(float),nthreads))==NULL){
      printf("calloc error reconmvm\n");
      return 1;
    }

    if((reconStruct->rs[0].cucentroids=calloc(sizeof(float),nthreads))==NULL){
      printf("calloc error reconmvm\n");
      return 1;
    }
    if((reconStruct->rs[1].cudmCommandArr=calloc(sizeof(float),nthreads))==NULL){
      printf("calloc error reconmvm\n");
      return 1;
    }
    if((reconStruct->rs[1].cucentroids=calloc(sizeof(float),nthreads))==NULL){
      printf("calloc error reconmvm\n");
      return 1;
    }
    
  }*/
#endif

  err=reconNewParam(buf,*reconHandle,frameno,arr,totCents);//this will change ->buf to 0.
  //rs->swap=0;//no - we don't need to swap.
  //rs=&reconStruct->rs[reconStruct->buf];
  if(err!=0){
    printf("Error in recon...\n");
    reconFree(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  //the condition variable and mutex don't need to be buffer swaped...
  if(pthread_mutex_init(&reconStruct->dmMutex,NULL)){
    printf("Error init recon mutex\n");
    reconFree(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if(pthread_cond_init(&reconStruct->dmCond,NULL)){
    printf("Error init recon cond\n");
    reconFree(reconHandle);
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
int reconNewFrame(void *reconHandle,float *dmCommand){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  ReconStructEntry *rs;
#if !defined(USEAGBBLAS) && !defined(USECUBLAS)
  CBLAS_ORDER order=CblasRowMajor;
  CBLAS_TRANSPOSE trans=CblasNoTrans;
  float alpha=1.,beta=1.;
  int inc=1;
#endif
  int i;
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
    //initialise by copying v0 into the result.
    //This line removed 100528 after discussion with Eric Gendron.
    //memcpy(glob->arrays->dmCommand,rs->v0,sizeof(float)*rs->nacts);
    //beta=1.;
    //memset(glob->arrays->dmCommand,0,sizeof(float)*rs->nacts);
    //Now: dmcommand=v0+dot(gainE,latestDmCommand)
#ifdef USEAGBBLAS
    agb_cblas_sgemvRowNN1N101(rs->nacts,rs->gainE,reconStruct->latestDmCommand,dmCommand);
#elif defined(USECUBLAS)
    //for now, since gainE isn't in gpu... note, if put in gpu - take care since gpu assumes column major... might need to set the transpose flag.
    agb_cblas_sgemvRowNN1N101(rs->nacts,rs->gainE,reconStruct->latestDmCommand,dmCommand);
#else
    //beta=0.;
    cblas_sgemv(order,trans,rs->nacts,rs->nacts,alpha,rs->gainE,rs->nacts,reconStruct->latestDmCommand,inc,beta,dmCommand,inc);
#endif
  }else{//reconmode_offset
    memcpy(dmCommand,rs->v0,sizeof(float)*rs->nacts);
  }	
#ifdef USECUBLAS
  //CUDA calls can only be made by 1 thread, not this one, so have to inform the correct (subap-processing) thread that it needs to update.
  int msg[1];
  msg[0]=INITFRAME;//gpu needs to upload dmcommand.
  pthread_mutex_lock(&reconStruct->cudaMutex);
  mq_send(reconStruct->mq,msg,sizeof(int),0);
  //and wait for the init to complete
  //mq_receive(reconStruct->mqFromGPU,msg,msgsize,NULL);//is this needed?
  pthread_mutex_unlock(&reconStruct->cudaMutex);
  //reconStruct->setDmCommand=dmCommand;
#endif



  //set the DM arrays ready.
  if(pthread_mutex_lock(&reconStruct->dmMutex))
    printf("pthread_mutex_lock error in setDMArraysReady: %s\n",strerror(errno));
  reconStruct->dmReady=1;
  //wake up any of the subap processing threads that are waiting.
  pthread_cond_broadcast(&reconStruct->dmCond);
  pthread_mutex_unlock(&reconStruct->dmMutex);
  return 0;
}

/**
   Called once per thread at the start of each frame, possibly simultaneously.
*/
int reconStartFrame(void *reconHandle,int threadno){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;//threadInfo->globals->reconStruct;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->buf];
  //globalStruct *glob=threadInfo->globals;
#ifndef USECUBLAS
  memset((void*)(&rs->dmCommandArr[rs->nacts*threadno]),0,rs->nacts*sizeof(float));

#else
  /*
  cudaError_t status;
  if(reconStruct->setDmCommand!=NULL){
    if((status=cublasSetVector(rs->nacts,sizeof(float),reconStruct->setDmCommand,1,rs->cudmCommand,1))!=CUBLAS_STATUS_SUCCESS){
      printf("cudamemset cublasSetVector failed  status=%d\n",(int)status);
    }
    reconStruct->setDmCommand=NULL;

  }
  if((status=cudaMemset(rs->cudmCommandArr[threadno],0,rs->nacts*sizeof(float)))!=cudaSuccess){
    printf("cudamemset cudmCommandArr[%d] failed  status=%d\n",threadno,(int)status);
    }*/
  //cudaThreadSynchronize();
  
  
#endif


  return 0;
}

/**
   Called multiple times by multiple threads, whenever new slope data is ready
   centroids may not be complete, and writing to dmCommand is not thread-safe without locking.
*/
int reconNewSlopes(void *reconHandle,int centindx,int threadno,int nsubapsDoing,float *centroids,float *dmCommand){
#if !defined(USEAGBBLAS) && !defined(USECUBLAS)
  CBLAS_ORDER order=CblasColMajor;
  CBLAS_TRANSPOSE trans=CblasNoTrans;
  float alpha=1.,beta=1.;
  int inc=1;
#endif
  int step;//number of rows to do in mmx...
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->buf];
  //infoStruct *info=threadInfo->info;
  //globalStruct *glob=threadInfo->globals;
  //We assume that each row i of the reconstructor has already been multiplied by gain[i].  
  //So, here we just do dmCommand+=rmx[:,n]*centx+rmx[:,n+1]*centy.
  dprintf("in partialReconstruct %d %d %d %p %p %p\n",rs->nacts,centindx,rs->totCents,centroids,rs->rmxT,&rs->dmCommandArr[rs->nacts*threadno]);
  step=2*nsubapsDoing;
#ifdef USEAGBBLAS
  agb_cblas_sgemvColMN1M111(rs->nacts,step,&(rs->rmxT[centindx*rs->nacts]),&(centroids[centindx]),&rs->dmCommandArr[rs->nacts*threadno]);
#elif defined(USECUBLAS)
  int msg[3];
  msg[0]=DOMVM;
  msg[1]=step;
  msg[2]=centindx;
  pthread_mutex_lock(reconStruct->cudaMudex);
  mq_send(reconStruct->mq,msg,sizeof(int)*3,0);
  pthread_mutex_unlock(reconStruct->cudaMudex);
  /*
  error_t status;
  //copy the centroids to GPU cucentroids==d_B
  //printf("addr %p\n",&(rs->cucentroids[threadno][centindx]));
  //printf("cublasSetVector(centroids)\n");
  if((status = cublasSetVector(step, sizeof(float),&(centroids[centindx]), 1, &(rs->cucentroids[threadno][centindx]), 1))!=CUBLAS_STATUS_SUCCESS){
    printf("!!!! device access error (write centroid vector):error no=%d\n",(int)status);
    return 1;
  }
  //cudaThreadSynchronize();
  //Do the GEMV., curmx==d_A
  //printf("cublasSgemv\n");
  cublasSgemv('n',rs->nacts,step,1.,&(rs->curmx[(centindx*rs->nacts)]), rs->nacts, &(rs->cucentroids[threadno][centindx]), 1, 1.,rs->cudmCommandArr[threadno], 1);
  if((status=cublasGetError())!=CUBLAS_STATUS_SUCCESS)
    printf("Error in cublasSgemv\n");
  */
  //cudaThreadSynchronize();
  //and get the result... cudmCommandArr=d_C
  //if((status=cublasGetVector(rs->nacts, sizeof(float), &rs->cudmCommandArr[rs->nacts*threadno],1,&rs->dmCommandArr[rs->nacts*threadno],1))==CUBLAS_STATUS_SUCCESS){
  //printf ("!!!! device access error (write dm vector)\n");
  //return 1;
  //}
#else
  cblas_sgemv(order,trans,rs->nacts,step,alpha,&(rs->rmxT[centindx*rs->nacts]),rs->nacts,&(centroids[centindx]),inc,beta,&rs->dmCommandArr[rs->nacts*threadno],inc);
#endif
  return 0;
}

/**
   Called once for each thread at the end of a frame
   Here we sum the individual dmCommands together to get the final one.
   centroids may not be complete, and writing to dmCommand is not thread-safe without locking.
*/
int reconEndFrame(void *reconHandle,int threadno,float *centroids,float *dmCommand){
  //dmCommand=glob->arrays->dmCommand;
  //globalStruct *glob=threadInfo->globals;
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->buf];
  if(pthread_mutex_lock(&reconStruct->dmMutex))
    printf("pthread_mutex_lock error in copyThreadPhase: %s\n",strerror(errno));
  if(reconStruct->dmReady==0)//wait for the precompute thread to finish (it will call setDMArraysReady when done)...
    if(pthread_cond_wait(&reconStruct->dmCond,&reconStruct->dmMutex))
      printf("pthread_cond_wait error in copyThreadPhase: %s\n",strerror(errno));
  //now add threadInfo->dmCommand to threadInfo->info->dmCommand.
#ifdef USEAGBBLAS
  agb_cblas_saxpy111(rs->nacts,&rs->dmCommandArr[rs->nacts*threadno],dmCommand);
#elif defined(USECUBLAS)
  //Sums cudmCommand into dmCommand (ie from GPU to CPU operation)
  //printf("Summing %p\n",rs->cudmCommandArr[threadno]);
  /*cublasSaxpy(rs->nacts,1.,rs->cudmCommandArr[threadno],1,rs->cudmCommand,1);
  error_t status;
  if((status=cublasGetError())!=CUBLAS_STATUS_SUCCESS)
    printf("Error in cublasSaxpy\n");
  //printf("summed\n");
  //cudaThreadSynchronize();
  */
#else
  cblas_saxpy(rs->nacts,1.,&rs->dmCommandArr[rs->nacts*threadno],1,dmCommand,1);
#endif
  pthread_mutex_unlock(&reconStruct->dmMutex);
  return 0;
}

/**
   Called by single thread per frame at the end of frame.
   This is called by a subaperture processing thread.
   The bare minimum should be placed here, as most processing should be done in the reconFrameFinished function instead, which doesn't hold up processing.
*/
int reconFrameFinishedSync(void *reconHandle,float *centroids,float *dmCommand){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  //xxx do we really need to get the lock here?
#ifdef USECUBLAS
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->buf];
  error_t status;
  //and now get the vector.
  /*
  if((status=cublasGetVector(rs->nacts, sizeof(float), rs->cudmCommand,1,dmCommand,1))!=CUBLAS_STATUS_SUCCESS){
    printf ("cuda device access error (get dmCommand vector) %d\n",(int)status);
    }*/
  //cudaThreadSynchronize();
  //printf("dmCommand[0]=%g\n",dmCommand[0]);
  int msg=ENDFRAME;
  mq_send(reconStruct->mq,&msg,sizeof(int),0);
#endif
  if(pthread_mutex_lock(&reconStruct->dmMutex))
    printf("pthread_mutex_lock error in copyThreadPhase: %s\n",strerror(errno));
  reconStruct->dmReady=0;
  pthread_mutex_unlock(&reconStruct->dmMutex);
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
int reconFrameFinished(void *reconHandle,float *dmCommand){//globalStruct *glob){
  //Note: dmCommand=glob->arrays->dmCommand.
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;//glob->reconStruct;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->postbuf];
  float bleedVal=0.;
  int i;
#ifdef USECUBLAS
  mq_receive(reconStruct->mqFromGPU,msg,msgsize,NULL);//wait for dmCommand to be transferred from the GPU.
#endif
  if(rs->bleedGainOverNact!=0.){//compute the bleed value
    for(i=0; i<rs->nacts; i++){
      //bleedVal+=glob->arrays->dmCommand[i];
      bleedVal+=dmCommand[i]-rs->v0[i];
    }
    bleedVal*=rs->bleedGainOverNact;
    //bleedVal-=rs->midRangeTimesBleed;//Note - really midrange times bleed over nact... maybe this should be replaced by v0 - to allow a midrange value per actuator?
    for(i=0; i<rs->nacts; i++)
      dmCommand[i]-=bleedVal;
  }
  //bleedVal-=0.5;//do proper rounding...
  memcpy(reconStruct->latestDmCommand,dmCommand,sizeof(float)*rs->nacts);
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
  memcpy(reconStruct->latestDmCommand,rs->v0,sizeof(float)*rs->nacts);
  return 0;

}
 
