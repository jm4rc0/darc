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
#include "powerdaq.h"
#include "powerdaq32.h"

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
  int threadAffinity;
  int threadPriority;
  tState state;//state of the acquisition session.
  int board;
  int adapterType;
}MirrorStruct;

/**
   Find out if this SO library supports your mirror.
*/
int mirrorQuery(char *name){
  int rtval=0;
#ifdef OLD
  rtval=(strcmp(name,"dmcPdAO32mirror")!=0);
#endif
  return rtval;
}

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


/**
   Free mirror/memory/sl240
*/
void dofree(MirrorStruct *mirstr){
  if(mirstr!=NULL){
    if(mirstr->arr!=NULL)
      free(mirstr->arr);
    CleanUpSingleAO(mirstr);
    pthread_cond_destroy(&mirstr->cond);
    pthread_mutex_destroy(&mirstr->m);
    free(mirstr);
  }
}



int setThreadAffinityForDMC(int threadAffinity,int threadPriority){
  int i;
  cpu_set_t mask;
  int ncpu;
  struct sched_param param;
  ncpu= sysconf(_SC_NPROCESSORS_ONLN);
  CPU_ZERO(&mask);
  for(i=0; i<ncpu; i++){
    if(((threadAffinity)>>i)&1){
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
  int i;
  setThreadAffinityForDMC(mirstr->threadAffinity,mirstr->threadPriority);
  pthread_mutex_lock(&mirstr->m);
  while(mirstr->open){
    pthread_cond_wait(&mirstr->cond,&mirstr->m);//wait for actuators.
    if(mirstr->open){
      //Now send the header...
      mirstr->err=0;
      for(i=0; i<mirstr->nacts; i++){
	mirstr->err|=_PdAO32Write(mirstr->handle,i,mirstr->arr[i]);
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

int mirrorOpen(char *name,int narg,int *args,char *buf,circBuf *rtcErrorBuf,char *prefix,void **mirrorHandle,int nacts,circBuf *rtcActuatorBuf,unsigned int frameno){

  int err;
  MirrorStruct *mirstr;
  DWORD aoCfg;
  printf("Initialising mirror %s\n",name);
  if((err=mirrorQuery(name))){
    printf("Error - wrong mirror name\n");
    return 1;
  }
  if((*mirrorHandle=malloc(sizeof(MirrorStruct)))==NULL){
    printf("couldn't malloc mirrorHandle\n");
    return 1;
  }
  mirstr=(MirrorStruct*)*mirrorHandle;
  memset(mirstr,0,sizeof(MirrorStruct));
  mirstr->nacts=nacts;
  mirstr->arrsize=nacts*sizeof(unsigned short);
  if((mirstr->arr=malloc(mirstr->arrsize))==NULL){
    printf("couldn't malloc arr\n");
    dofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  memset(mirstr->arr,0,mirstr->arrsize);
  if(narg==2){
    mirstr->threadAffinity=args[0];
    mirstr->threadPriority=args[1];
  }else{
    printf("wrong number of args - should be timeout, fibrePort, thread affinity, thread priority\n");
    dofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }

  if(pthread_cond_init(&mirstr->cond,NULL)!=0){
    printf("Error initialising thread condition variable\n");
    dofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  //maybe think about having one per camera???
  if(pthread_mutex_init(&mirstr->m,NULL)!=0){
    printf("Error initialising mutex variable\n");
    dofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  //initialise acquisition session
  if(InitSingleAO(mirstr)){//failed...
    printf("Failed to initSingleAO\n");
    dofree(mirstr);
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

  mirstr->open=1;
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
    mirstr->open=0;
    pthread_cond_signal(&mirstr->cond);//wake the thread.
    pthread_mutex_unlock(&mirstr->m);
    pthread_join(mirstr->threadid,NULL);//wait for worker thread to complete
    dofree(mirstr);
    *mirrorHandle=NULL;
  }
  printf("Mirror closed\n");
  return 0;
}
int mirrorSend(void *mirrorHandle,int n,unsigned short *data,unsigned int frameno){
  MirrorStruct *mirstr=(MirrorStruct*)mirrorHandle;
  int err=0;
  if(mirstr!=NULL && mirstr->open==1){
    //printf("Sending %d values to mirror\n",n);
    pthread_mutex_lock(&mirstr->m);
    err=mirstr->err;//get the error from the last time.  Even if there was an error, need to send new actuators, to wake up the thread... incase the error has gone away.
    //First, copy actuators.  Note, should n==mirstr->nacts.
    memcpy(mirstr->arr,data,sizeof(unsigned short)*mirstr->nacts);
    //Wake up the thread.
    pthread_cond_signal(&mirstr->cond);
    pthread_mutex_unlock(&mirstr->m);
  }else{
    err=1;
  }
  return err;
}
