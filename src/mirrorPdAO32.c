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
   The code here is used to create a shared object library, which can then be swapped around depending on which mirrors/interfaces you have in use, ie you simple rename the mirror file you want to mirror.so (or better, change the soft link), and restart the coremain.

The library is written for a specific mirror configuration - ie in multiple mirror situations, the library is written to handle multiple mirrors, not a single mirror many times.

This library is written for the DAC card.  For use in a figure sensing RTC (or one which has the DAC card locally).

*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include "rtcmirror.h"
#include <time.h>
#include <pthread.h>
#ifndef NODM
#include "powerdaq.h"
#include "powerdaq32.h"
#endif
#include "darc.h"
#include "agbcblas.h"

#define errorChk(functionCall) {int error; if((error=functionCall)<0) { \
	                           fprintf(stderr, "Error %d at line %d in function call %s\n", error, __LINE__, #functionCall); \
	                           exit(EXIT_FAILURE);}}
typedef unsigned int uint32;
typedef enum _state
{
   closed,
   unconfigured,
   configured,
   running
} tState;

typedef enum{
  MIRRORACTCONTROLMX,
  MIRRORACTINIT,
  MIRRORACTMAPPING,
  MIRRORACTMAX,
  MIRRORACTMIN,
  MIRRORACTNEW,
  MIRRORACTOFFSET,
  MIRRORACTOSCARR,
  MIRRORACTOSCPERACT,
  MIRRORACTOSCTIME,
  MIRRORACTSCALE,
  MIRRORACTSOURCE,
  MIRRORNACTS,

  //Add more before this line.
  MIRRORNBUFFERVARIABLES//equal to number of entries in the enum
}MIRRORBUFFERVARIABLEINDX;

#define makeParamNames() bufferMakeNames(MIRRORNBUFFERVARIABLES,\
					 "actControlMx","actInit","actMapping","actMax","actMin","actNew","actOffset","actOscArr","actOscPerAct","actOscTime","actScale","actSource", "nacts")

//Need to add:  actControlMx, actNew.

typedef struct{
  int nacts;
  unsigned short *arr;
  uint32 arrsize;
  int open;
  int err;
  pthread_t threadid;
  pthread_cond_t cond;
  pthread_mutex_t m;
  int handle;
  unsigned int *threadAffinity;
  int threadAffinElSize;
  int threadPriority;
  tState state;//state of the acquisition session.
  int board;
  int adapterType;
  //MirrorStructBuffered msb[2];
  //int buf;
  //int swap;
  //int bufindx[MIRRORNBUFFERVARIABLES];
  circBuf *rtcActuatorBuf;
  circBuf *rtcErrorBuf;
  unsigned short *actInit;
  int initLen;
  unsigned short *actMin;
  unsigned short *actMax;
  int *actMapping;
  int actMappingSize;
  int actMappingLen;
  int *actSource;
  float *actScale;
  float *actOffset;
  float *oscillateArr;
  int nactsNew;
  float *actsNew;
  int actsNewSize;
  int *actControlMx;
  int oscillateIters;
  int oscillateArrSize;
  int oscillateSleepTime;
  unsigned short *arrPrev;
  char *paramNames;
  int index[MIRRORNBUFFERVARIABLES];
  void *values[MIRRORNBUFFERVARIABLES];
  char dtype[MIRRORNBUFFERVARIABLES];
  int nbytes[MIRRORNBUFFERVARIABLES];

}MirrorStruct;


#ifndef NODM
int InitSingleAO(MirrorStruct *pAoData){
   Adapter_Info adaptInfo;
   // get adapter type
   errorChk(_PdGetAdapterInfo(pAoData->board, &adaptInfo));
   pAoData->adapterType = adaptInfo.atType;
   if(pAoData->adapterType & atMF)
     printf("This is an MFx board\n");
   else
     printf("This is an AO32 board\n");
   pAoData->handle = PdAcquireSubsystem(pAoData->board, AnalogOut, 1);
   if(pAoData->handle < 0){
      printf("SingleAO: PdAcquireSubsystem failed\n");
      pAoData->state=closed;
      return 1;
   }
   pAoData->state = unconfigured;
   errorChk(_PdAOutReset(pAoData->handle));
   // need also to call this function if the board is a PD2-AO-xx
   if(pAoData->adapterType & atPD2AO){
      errorChk(_PdAO32Reset(pAoData->handle));
   }
   return 0;
}




void CleanUpSingleAO(MirrorStruct *pAoData){
   if(pAoData->state == running){
      pAoData->state = configured;
   }
   if(pAoData->state == configured){
     errorChk(_PdAOutReset(pAoData->handle));
     // need also to call this function if the board is a PD2-AO-xx
     if(pAoData->adapterType & atPD2AO){
       errorChk(_PdAO32Reset(pAoData->handle));
     }
     pAoData->state = unconfigured;
   }
   if(pAoData->handle > 0 && pAoData->state == unconfigured){
     errorChk(PdAcquireSubsystem(pAoData->handle, AnalogOut, 0));
   }
   pAoData->state = closed;
}
#endif

/**
   Free mirror/memory/sl240
*/
void mirrordofree(MirrorStruct *mirstr){
  //int i;
  if(mirstr!=NULL){
    if(mirstr->arr!=NULL)
      free(mirstr->arr);
    if(mirstr->arrPrev!=NULL)
      free(mirstr->arrPrev);
    /*for(i=0; i<2; i++){
      if(mirstr->msb[i].actMin!=NULL)free(mirstr->msb[i].actMin);
      if(mirstr->msb[i].actMax!=NULL)free(mirstr->msb[i].actMax);
      if(mirstr->msb[i].actMapping!=NULL)free(mirstr->msb[i].actMapping);
      }*/

#ifndef NODM
    CleanUpSingleAO(mirstr);
#endif
    pthread_cond_destroy(&mirstr->cond);
    pthread_mutex_destroy(&mirstr->m);
    free(mirstr);
  }
}

#ifdef NODM
int _PdAO32Write(int handle,int i,unsigned short val){
  return 0;
}
#endif


int mirrorsetThreadAffinity(unsigned int *threadAffinity,int threadPriority,int threadAffinElSize){
  int i;
  cpu_set_t mask;
  int ncpu;
  struct sched_param param;
  ncpu= sysconf(_SC_NPROCESSORS_ONLN);
  CPU_ZERO(&mask);
  for(i=0; i<ncpu && i<threadAffinElSize*32; i++){
    if(((threadAffinity[i/32])>>(i%32))&1){
      CPU_SET(i,&mask);
    }
  }
  //printf("Thread affinity %d\n",threadAffinity&0xff);
  if(sched_setaffinity(0,sizeof(cpu_set_t),&mask))
    printf("Error in sched_setaffinity: %s\n",strerror(errno));
  param.sched_priority=threadPriority;
  if(sched_setparam(0,&param)){
    printf("Error in sched_setparam: %s - probably need to run as root if this is important\n",strerror(errno));
  }
  if(sched_setscheduler(0,SCHED_RR,&param))
    printf("sched_setscheduler: %s - probably need to run as root if this is important\n",strerror(errno));
  if(pthread_setschedparam(pthread_self(),SCHED_RR,&param))
    printf("error in pthread_setschedparam - maybe run as root?\n");
  return 0;
}

/**
   The thread that does the work - copies actuators, and sends to the DAC
*/
void* worker(void *mirstrv){
  MirrorStruct *mirstr=(MirrorStruct*)mirstrv;
  int i,j;
  int val;
  struct timespec tme;
  int skip,step,nacts;
  mirrorsetThreadAffinity(mirstr->threadAffinity,mirstr->threadPriority,mirstr->threadAffinElSize);
  pthread_mutex_lock(&mirstr->m);
  if(mirstr->open && mirstr->actInit!=NULL){
    for(i=0; i<mirstr->initLen; i++){
      _PdAO32Write(mirstr->handle,i,mirstr->actInit[i]);
    }
  }
  
  while(mirstr->open){
    pthread_cond_wait(&mirstr->cond,&mirstr->m);//wait for actuators.
    if(mirstr->open){
      //Here, think about adding the option to oscillate the DM to the desired solution.  This would be using a pre-determined array.  The values to put on the DM would be something like:  exp(-t/5)*cos(t)*D/2+acts where t is 0,pi,2pi,3pi,..., and D is acts-acts_-1, i.e. the difference between last and current requested demands.  t could be of dimensions n, or n,nacts where n is the number of steps that you wish to take.  Would also need a sleep time, which would be something like frametime/2/n, allowing the update to be finished within half a frame time.  
      if(mirstr->actControlMx!=NULL){
	if(mirstr->actMapping==NULL)
	  nacts=mirstr->nactsNew;
	else
	  nacts=mirstr->actMappingLen;
      }else{
	if(mirstr->actMapping==NULL)
	  nacts=mirstr->nacts;
	else
	  nacts=mirstr->actMappingLen;
      }
	
      
      mirstr->err=0;
      if(mirstr->oscillateArr==NULL){
	if(mirstr->actMapping==NULL){
	  for(i=0; i<nacts; i++){
	    mirstr->err|=_PdAO32Write(mirstr->handle,i,mirstr->arr[i]);
	  }
	}else{
	  
	  for(i=0; i<nacts; i++){
	    mirstr->err|=_PdAO32Write(mirstr->handle,mirstr->actMapping[i],mirstr->arr[i]);
	  }
	}
      }else{//need to oscillate to the solution.
	clock_gettime(CLOCK_REALTIME,&tme);
	//nel=mirstr->actMapping==NULL?mirstr->nacts:mirstr->actMappingLen;
	for(j=0;j<mirstr->oscillateIters;j++){
	  if(mirstr->oscillateArrSize==mirstr->oscillateIters){//one only per timestep
	    skip=1;
	    step=0;
	  }else{//a value per actuator per timestep.
	    skip=nacts;
	    step=1;
	  }
	  if(mirstr->actMapping==NULL){
	    for(i=0;i<nacts;i++){
	      val=(int)(0.5+mirstr->arr[i]+mirstr->oscillateArr[j*skip+step*i]*((int)mirstr->arr[i]-(int)mirstr->arrPrev[i]));
	      if(val>mirstr->actMax[i])
		val=mirstr->actMax[i];
	      if(val<mirstr->actMin[i])
		val=mirstr->actMin[i];
	      mirstr->err|=_PdAO32Write(mirstr->handle,i,(unsigned short)val);
	    }
	  }else{
	    for(i=0; i<nacts; i++){
	      val=(int)(0.5+mirstr->arr[i]+mirstr->oscillateArr[j*skip+step*i]*((int)mirstr->arr[i]-(int)mirstr->arrPrev[i]));
	      if(val>mirstr->actMax[i])
		val=mirstr->actMax[i];
	      if(val<mirstr->actMin[i])
		val=mirstr->actMin[i];
	      mirstr->err|=_PdAO32Write(mirstr->handle,mirstr->actMapping[i],(unsigned short)val);
	    }
	  }
	  //wait before adjusting the mirror slightly.
	  tme.tv_nsec+=mirstr->oscillateSleepTime;
	  if(tme.tv_nsec>999999999){
	    tme.tv_sec++;
	    tme.tv_nsec-=1000000000;
	  }
	  clock_nanosleep(CLOCK_REALTIME,TIMER_ABSTIME,&tme,NULL);
	}
	//Store actuators for next iteration.
	memcpy(mirstr->arrPrev,mirstr->arr,sizeof(unsigned short)*nacts);
      }
    }
  }
  pthread_mutex_unlock(&mirstr->m);
  return NULL;
}

/**
   Open a camera of type name.  Args are passed in a float array of size n, which can be cast if necessary.  Any state data is returned in camHandle, which should be NULL if an error arises.
   pxlbuf is the array that should hold the data. The library is free to use the user provided version, or use its own version as necessary (ie a pointer to physical memory or whatever).  It is of size npxls*sizeof(short).
   ncam is number of cameras, which is the length of arrays pxlx and pxly, which contain the dimensions for each camera.
   Name is used if a library can support more than one camera.

*/

int mirrorOpen(char *name,int narg,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **mirrorHandle,int nacts,circBuf *rtcActuatorBuf,unsigned int frameno,unsigned int **mirrorframeno,int *mirrorframenoSize){
  int err;
  MirrorStruct *mirstr;
#ifndef NODM
  DWORD aoCfg;
#endif
  char *pn;
  printf("Initialising mirror %s\n",name);
  if((pn=makeParamNames())==NULL){
    printf("Error making paramList - please recode mirrorPdAO32.c\n");
    *mirrorHandle=NULL;
    return 1;
  }


  if((*mirrorHandle=malloc(sizeof(MirrorStruct)))==NULL){
    printf("couldn't malloc mirrorHandle\n");
    return 1;
  }
  mirstr=(MirrorStruct*)*mirrorHandle;
  memset(mirstr,0,sizeof(MirrorStruct));
  mirstr->paramNames=pn;
  mirstr->nacts=nacts;
  mirstr->rtcErrorBuf=rtcErrorBuf;
  mirstr->rtcActuatorBuf=rtcActuatorBuf;
  mirstr->arrsize=nacts*sizeof(unsigned short);
  if((mirstr->arr=malloc(mirstr->arrsize))==NULL){
    printf("couldn't malloc arr\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  if((mirstr->arrPrev=malloc(mirstr->arrsize))==NULL){
    printf("couldn't malloc arrPrev\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  memset(mirstr->arr,0,mirstr->arrsize);
  memset(mirstr->arrPrev,0,mirstr->arrsize);
  if(narg>2 && narg==2+args[0]){
    mirstr->threadAffinElSize=args[0];
    mirstr->threadPriority=args[1];
    mirstr->threadAffinity=(unsigned int*)&args[2];
  }else{
    printf("wrong number of args - should be Naffin, thread priority, thread affinity[Naffin]\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  if(mirstr->rtcActuatorBuf!=NULL && mirstr->rtcActuatorBuf->datasize!=mirstr->nacts*sizeof(unsigned short)){
    if(circReshape(mirstr->rtcActuatorBuf,1,&mirstr->nacts,'H')!=0){
      printf("Error reshaping rtcActuatorBuf\n");
    }
  }
  if(pthread_cond_init(&mirstr->cond,NULL)!=0){
    printf("Error initialising thread condition variable\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  //maybe think about having one per camera???
  if(pthread_mutex_init(&mirstr->m,NULL)!=0){
    printf("Error initialising mutex variable\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  //initialise acquisition session
#ifndef NODM
  if(InitSingleAO(mirstr)){//failed...
    printf("Failed to initSingleAO\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  // set configuration - _PdAOutReset is called inside _PdAOutSetCfg
  aoCfg = 0;
  errorChk(_PdAOutSetCfg(mirstr->handle, aoCfg, 0));
  mirstr->state = configured;
  //Start SW trigger
  errorChk(_PdAOutSwStartTrig(mirstr->handle));
  mirstr->state = running;
#endif
  mirstr->open=1;
  if((err=mirrorNewParam(*mirrorHandle,pbuf,frameno,arr))){
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  pthread_create(&mirstr->threadid,NULL,worker,mirstr);
  return 0;
}

/**
   Close a camera of type name.  Args are passed in the float array of size n, and state data is in camHandle, which should be freed and set to NULL before returning.
*/
int mirrorClose(void **mirrorHandle){
  MirrorStruct *mirstr=(MirrorStruct*)*mirrorHandle;
  printf("Closing mirror\n");
  if(mirstr!=NULL){
    pthread_mutex_lock(&mirstr->m);
    if(mirstr->paramNames!=NULL)
      free(mirstr->paramNames);
    mirstr->open=0;
    pthread_cond_signal(&mirstr->cond);//wake the thread.
    pthread_mutex_unlock(&mirstr->m);
    printf("Joining mirror worker thread\n");
    pthread_join(mirstr->threadid,NULL);//wait for worker thread to complete
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
  }
  printf("Mirror closed\n");
  return 0;
}
int mirrorSend(void *mirrorHandle,int n,float *data,unsigned int frameno,double timestamp,int err,int writeCirc){
  MirrorStruct *mirstr=(MirrorStruct*)mirrorHandle;
  int nclipped=0;
  int intDMCommand;
  int i,nacts;
  //MirrorStructBuffered *msb;
  if(err==0 && mirstr!=NULL && mirstr->open==1){
    //printf("Sending %d values to mirror\n",n);
    pthread_mutex_lock(&mirstr->m);
    /*if(mirstr->swap){
      mirstr->buf=1-mirstr->buf;
      mirstr->swap=0;
      }*/
    //msb=&mirstr->msb[mirstr->buf];
    err=mirstr->err;//get the error from the last time.  Even if there was an error, need to send new actuators, to wake up the thread... incase the error has gone away.
    //First, copy actuators.  Note, should n==mirstr->nacts.
    //Note, need to convert to uint16...
    if(mirstr->actControlMx!=NULL){
      //multiply acts by a matrix (sparse), to get a new set of acts.
      //This therefore allows to build up actuators that are a combination of other actuators.  e.g. a 3-actuator tiptilt mirror from 2x tiptilt signal.
      agb_cblas_sparse_csr_sgemvRowMN1N101(mirstr->nactsNew,mirstr->nacts,mirstr->actControlMx,data,mirstr->actsNew);
      data=mirstr->actsNew;
      //results placed in factsNew (of size nactsNew).
      if(mirstr->actMapping==NULL)
	nacts=mirstr->nactsNew;
      else
	nacts=mirstr->actMappingLen;
    }else{
      if(mirstr->actMapping==NULL)
	nacts=mirstr->nacts;
      else
	nacts=mirstr->actMappingLen;
    }
    

    if(mirstr->actMapping==NULL){
      for(i=0; i<nacts; i++){
	intDMCommand=(int)(data[i]+0.5);
	mirstr->arr[i]=(unsigned short)intDMCommand;
	if(intDMCommand<mirstr->actMin[i]){
	  nclipped++;
	  mirstr->arr[i]=mirstr->actMin[i];
	}
	if(intDMCommand>mirstr->actMax[i]){
	  nclipped++;
	  mirstr->arr[i]=mirstr->actMax[i];
	}
      }
    }else{//actMapping is specified...
      if(mirstr->actSource==NULL){
	if(mirstr->actScale==NULL){
	  if(mirstr->actOffset==NULL){
	    for(i=0; i<nacts; i++){
	      intDMCommand=(int)(data[i]+0.5);
	      mirstr->arr[i]=(unsigned short)intDMCommand;
	      if(intDMCommand<mirstr->actMin[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMin[i];
	      }
	      if(intDMCommand>mirstr->actMax[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMax[i];
	      }
	    }
	  }else{//actoffset defined.
	    for(i=0; i<nacts; i++){
	      intDMCommand=(int)(data[i]+mirstr->actOffset[i]+0.5);
	      mirstr->arr[i]=(unsigned short)intDMCommand;
	      if(intDMCommand<mirstr->actMin[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMin[i];
	      }
	      if(intDMCommand>mirstr->actMax[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMax[i];
	      }
	    }
	  }
	}else{//actscale defined
	  if(mirstr->actOffset==NULL){
	    for(i=0; i<nacts; i++){
	      intDMCommand=(int)(data[i]*mirstr->actScale[i]+0.5);
	      mirstr->arr[i]=(unsigned short)intDMCommand;
	      if(intDMCommand<mirstr->actMin[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMin[i];
	      }
	      if(intDMCommand>mirstr->actMax[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMax[i];
	      }
	    }
	  }else{//actScale and actoffset defined
	    for(i=0; i<nacts; i++){
	      intDMCommand=(int)(data[i]*mirstr->actScale[i]+mirstr->actOffset[i]+0.5);
	      mirstr->arr[i]=(unsigned short)intDMCommand;
	      if(intDMCommand<mirstr->actMin[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMin[i];
	      }
	      if(intDMCommand>mirstr->actMax[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMax[i];
	      }
	    }
	  }
	}
      }else{//actSource defined
	if(mirstr->actScale==NULL){
	  if(mirstr->actOffset==NULL){
	    for(i=0; i<nacts; i++){
	      intDMCommand=(int)(data[mirstr->actSource[i]]+0.5);
	      mirstr->arr[i]=(unsigned short)intDMCommand;
	      if(intDMCommand<mirstr->actMin[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMin[i];
	      }
	      if(intDMCommand>mirstr->actMax[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMax[i];
	      }
	    }
	  }else{//actSource and actoffset defined.
	    for(i=0; i<nacts; i++){
	      intDMCommand=(int)(data[mirstr->actSource[i]]+mirstr->actOffset[i]+0.5);
	      mirstr->arr[i]=(unsigned short)intDMCommand;
	      if(intDMCommand<mirstr->actMin[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMin[i];
	      }
	      if(intDMCommand>mirstr->actMax[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMax[i];
	      }
	    }
	  }
	}else{//actSource and actscale defined
	  if(mirstr->actOffset==NULL){
	    for(i=0; i<nacts; i++){
	      intDMCommand=(int)(data[mirstr->actSource[i]]*mirstr->actScale[i]+0.5);
	      mirstr->arr[i]=(unsigned short)intDMCommand;
	      if(intDMCommand<mirstr->actMin[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMin[i];
	      }
	      if(intDMCommand>mirstr->actMax[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMax[i];
	      }
	    }
	  }else{//actSource and actScale and actoffset defined
	    for(i=0; i<nacts; i++){
	      intDMCommand=(int)(data[mirstr->actSource[i]]*mirstr->actScale[i]+mirstr->actOffset[i]+0.5);
	      mirstr->arr[i]=(unsigned short)intDMCommand;
	      if(intDMCommand<mirstr->actMin[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMin[i];
	      }
	      if(intDMCommand>mirstr->actMax[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMax[i];
	      }
	    }
	  }
	}
      }
    }
    //memcpy(mirstr->arr,data,sizeof(unsigned short)*mirstr->nacts);
    //Wake up the thread.
    pthread_cond_signal(&mirstr->cond);
    pthread_mutex_unlock(&mirstr->m);
    if(writeCirc)
      circAddForce(mirstr->rtcActuatorBuf,mirstr->arr,timestamp,frameno);
  }else{
    err=1;
  }
  if(err)
    return -1;
  return nclipped;
}
/**
   This is called by a main processing thread - asynchronously with mirrorSend.
*/

int mirrorNewParam(void *mirrorHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr){
  MirrorStruct *mirstr=(MirrorStruct*)mirrorHandle;
  int err=0;
  //int got=0;
  //int dim;
  int nactsNew;
  //int bufno;
  //MirrorStructBuffered *msb;
  //int *indx=mirstr->bufindx;
  //MIRRORBUFFERVARIABLEINDX i;
  int j=0;
  int nfound;
  int *indx=mirstr->index;
  void **values=mirstr->values;
  char *dtype=mirstr->dtype;
  int *nbytes=mirstr->nbytes;
  int actControlMxSize;
  if(mirstr==NULL || mirstr->open==0){
    printf("Mirror not open\n");
    return 1;
  }
  //bufno=1-mirstr->buf;
  //msb=&mirstr->msb[bufno];

  nfound=bufferGetIndex(pbuf,MIRRORNBUFFERVARIABLES,mirstr->paramNames,indx,values,dtype,nbytes);
  if(nfound!=MIRRORNBUFFERVARIABLES){
    for(j=0; j<MIRRORNBUFFERVARIABLES; j++){
      if(indx[j]<0){
	printf("Missing %16s\n",&mirstr->paramNames[j*16]);
	if(j!=MIRRORACTOSCARR && j!=MIRRORACTOSCPERACT && j!=MIRRORACTOSCTIME){
	  writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"Error in mirror parameter buffer: %16s",&mirstr->paramNames[j*16]);
	  err=-1;
	}
      }
    }
  }
  if(err==0){
    pthread_mutex_lock(&mirstr->m);
    if(dtype[MIRRORNACTS]=='i' && nbytes[MIRRORNACTS]==sizeof(int)){
      if(mirstr->nacts!=*((int*)values[MIRRORNACTS])){
	printf("Error - nacts changed - please close and reopen mirror library\n");
	err=1;
      }
    }else{
      printf("mirrornacts error\n");
      writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"mirrornacts error");
      err=1;
    }
    if(nbytes[MIRRORACTNEW]==sizeof(int) && dtype[MIRRORACTNEW]=='i'){
      mirstr->nactsNew=*(int*)values[MIRRORACTNEW];
    }else{
      printf("Warning - actNew wrong size or type\n");
      err=1;
      mirstr->nactsNew=0;
    }

    if(nbytes[MIRRORACTCONTROLMX]==0 || mirstr->nactsNew==0){
      mirstr->actControlMx=NULL;
    }else if(dtype[MIRRORACTCONTROLMX]=='i'){
      mirstr->actControlMx=(int*)values[MIRRORACTCONTROLMX];
      actControlMxSize=nbytes[MIRRORACTCONTROLMX]/sizeof(int);
      if(actControlMxSize<mirstr->nactsNew || actControlMxSize!=mirstr->nactsNew+1+2*mirstr->actControlMx[mirstr->nactsNew]){
	printf("Warning - wrong size actControlMx\n");
	err=1;
	mirstr->actControlMx=NULL;
	actControlMxSize=0;
      }
      if(mirstr->actsNewSize<mirstr->nactsNew){
	if(mirstr->actsNew!=NULL)
	  free(mirstr->actsNew);
	if((mirstr->actsNew=malloc(mirstr->nactsNew*sizeof(float)))==NULL){
	  printf("Error allocing actNew\n");
	  err=1;
	  mirstr->actsNewSize=0;
	  mirstr->actControlMx=NULL;
	  actControlMxSize=0;
	}else{
	  mirstr->actsNewSize=mirstr->nactsNew;
	  memset(mirstr->actsNew,0,sizeof(float)*mirstr->nactsNew);
	}
      }
    }else{
      printf("Warning - bad actControlMx - should be int32 (and the mx values will be read as float in darc)\n");
      err=1;
    }

    if(nbytes[MIRRORACTMAPPING]==0){
      mirstr->actMapping=NULL;
    }else if(dtype[MIRRORACTMAPPING]=='i' && nbytes[MIRRORACTMAPPING]%sizeof(int)==0){
      mirstr->actMappingLen=nbytes[MIRRORACTMAPPING]/sizeof(int);
      mirstr->actMapping=(int*)values[MIRRORACTMAPPING];
    }else{
      printf("Warning - bad actuator mapping\n");
      mirstr->actMapping=NULL;
    }

    if(mirstr->actControlMx!=NULL){
      if(mirstr->actMapping==NULL)
	nactsNew=mirstr->nactsNew;
      else
	nactsNew=mirstr->actMappingLen;
    }else{
      if(mirstr->actMapping==NULL)
	nactsNew=mirstr->nacts;
      else
	nactsNew=mirstr->actMappingLen;
    }
    //dim=mirstr->actMapping==NULL?mirstr->nacts:mirstr->actMappingLen;


    if(mirstr->arrsize<nactsNew*sizeof(unsigned short)){//bytes[MIRRORACTMAPPING]){
      if(mirstr->arr!=NULL) 
	free(mirstr->arr);
      if((mirstr->arr=malloc(nactsNew*sizeof(unsigned short)))==NULL){//nbytes[MIRRORACTMAPPING]))==NULL){
	printf("Error allocating mirstr->arr\n");
	err=1;
	mirstr->arrsize=0;
      }else{
	mirstr->arrsize=nactsNew*sizeof(unsigned short);
	memset(mirstr->arr,0,nactsNew*sizeof(unsigned short));
      }
      if(mirstr->arrPrev!=NULL) 
	free(mirstr->arrPrev);
      if((mirstr->arrPrev=malloc(nactsNew*sizeof(unsigned short)))==NULL){
	printf("Error allocating mirstr->arrPrev\n");
	err=1;
	mirstr->arrsize=0;
      }else{
	//mirstr->arrsize=nbytes[MIRRORACTMAPPING];
	memset(mirstr->arrPrev,0,nactsNew*sizeof(unsigned short));
      }
    }

    if(mirstr->rtcActuatorBuf!=NULL && mirstr->rtcActuatorBuf->datasize!=nactsNew*sizeof(unsigned short)){
      if(circReshape(mirstr->rtcActuatorBuf,1,&nactsNew,'H')!=0){
	printf("Error reshaping rtcActuatorBuf\n");
      }
    }
    if(nbytes[MIRRORACTSOURCE]==0){
      mirstr->actSource=NULL;
    }else if(nbytes[MIRRORACTSOURCE]==nactsNew*sizeof(int) && dtype[MIRRORACTSOURCE]=='i'){
      mirstr->actSource=(int*)values[MIRRORACTSOURCE];
    }else{
      printf("actSource wrong\n");
      mirstr->actSource=NULL;
    }
    if(nbytes[MIRRORACTSCALE]==0){
      mirstr->actScale=NULL;
    }else if(nbytes[MIRRORACTSCALE]==nactsNew*sizeof(float) && dtype[MIRRORACTSCALE]=='f'){
      mirstr->actScale=(float*)values[MIRRORACTSCALE];
    }else{
      printf("actScale wrong\n");
      mirstr->actScale=NULL;
    }
    if(nbytes[MIRRORACTOFFSET]==0){
      mirstr->actOffset=NULL;
    }else if(nbytes[MIRRORACTOFFSET]==nactsNew*sizeof(float) && dtype[MIRRORACTOFFSET]=='f'){
      mirstr->actOffset=(float*)values[MIRRORACTOFFSET];
    }else{
      printf("actOffset wrong\n");
      mirstr->actOffset=NULL;
    }
    if(dtype[MIRRORACTMIN]=='H' && nbytes[MIRRORACTMIN]==sizeof(unsigned short)*nactsNew){
      mirstr->actMin=(unsigned short*)values[MIRRORACTMIN];
    }else{
      printf("mirrorActMin error\n");
      writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"mirrorActMin error");
      err=1;
    }
    if(dtype[MIRRORACTMAX]=='H' && nbytes[MIRRORACTMAX]==sizeof(unsigned short)*nactsNew){
      mirstr->actMax=(unsigned short*)values[MIRRORACTMAX];
    }else{
      printf("mirrorActMax error\n");
      writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"mirrorActMax error");
      err=1;
    }
    if(dtype[MIRRORACTINIT]=='H' && nbytes[MIRRORACTINIT]%sizeof(unsigned short)==0){
      mirstr->actInit=(unsigned short*)values[MIRRORACTINIT];
      mirstr->initLen=nbytes[MIRRORACTINIT]/sizeof(unsigned short);
    }else if(nbytes[MIRRORACTINIT]!=0){
      printf("actInit error\n");
      writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"actInit error");
      err=1;
      mirstr->actInit=NULL;
      mirstr->initLen=0;
    }else{
      mirstr->actInit=NULL;
      mirstr->initLen=0;
    }
    if(indx[MIRRORACTOSCARR]>=0){
      if(dtype[MIRRORACTOSCARR]=='f'){//is it 1D or 2D?
	if(nbytes[MIRRORACTOSCARR]%(nactsNew*sizeof(float))==0){//multiple of nacts, so probably 2D
	  if(indx[MIRRORACTOSCPERACT]>=0 && dtype[MIRRORACTOSCPERACT]=='i' && nbytes[MIRRORACTOSCPERACT]==sizeof(int) && *((int*)values[MIRRORACTOSCPERACT])==1){//2D
	    mirstr->oscillateArr=(float*)values[MIRRORACTOSCARR];
	    mirstr->oscillateIters=nbytes[MIRRORACTOSCARR]/sizeof(float)/nactsNew;
	    mirstr->oscillateArrSize=nbytes[MIRRORACTOSCARR]/sizeof(float);
	  }else{//1D
	    mirstr->oscillateArr=(float*)values[MIRRORACTOSCARR];
	    mirstr->oscillateIters=nbytes[MIRRORACTOSCARR]/sizeof(float);
	    mirstr->oscillateArrSize=nbytes[MIRRORACTOSCARR]/sizeof(float);
	  }
	}else if(nbytes[MIRRORACTOSCARR]%sizeof(float)==0){//1D
	  mirstr->oscillateArr=(float*)values[MIRRORACTOSCARR];
	  mirstr->oscillateIters=nbytes[MIRRORACTOSCARR]/sizeof(float);
	  mirstr->oscillateArrSize=nbytes[MIRRORACTOSCARR]/sizeof(float);
	}else{//wrong shape
	  printf("actOscArr error\n");
	  writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"actOscArr error");
	  mirstr->oscillateArr=NULL;
	  err=1;
	}
      }else{
	printf("actOscArr error\n");
	writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"actOscArr error");
	mirstr->oscillateArr=NULL;
	err=1;
      }
    }else{
      mirstr->oscillateArr=NULL;
    }
    if(indx[MIRRORACTOSCTIME]>=0){
      if(dtype[MIRRORACTOSCTIME]=='i' && nbytes[MIRRORACTOSCTIME]==sizeof(int)){
	mirstr->oscillateSleepTime=*((int*)values[MIRRORACTOSCTIME]);
      }else{
	printf("actOscTime error\n");
	writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"actOscTime error");
	mirstr->oscillateSleepTime=0;
	err=1;
      }	
    }else{
      mirstr->oscillateSleepTime=0;
    }
    pthread_mutex_unlock(&mirstr->m);
  }
  
  //mirstr->swap=1;
  return err;
}
