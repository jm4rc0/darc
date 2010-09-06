/**
   The code here is used to create a shared object library, which can then be swapped around depending on which mirrors/interfaces you have in use, ie you simple rename the mirror file you want to mirror.so (or better, change the soft link), and restart the coremain.

The library is written for a specific mirror configuration - ie in multiple mirror situations, the library is written to handle multiple mirrors, not a single mirror many times.
*/

#include <nslapi.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#ifndef OLD
#include "rtcmirror.h"
#endif
#include <time.h>
#include <pthread.h>
#define HDRSIZE 8 //the size of a WPU header - 4 bytes for frame no, 4 bytes for something else.

typedef struct{
  int nacts;
  unsigned short *arr;
  unsigned short *rtcacts;
  int frameno;
  uint32 arrsize;
  int open;
  int err;
  pthread_t threadid;
  pthread_cond_t cond;
  pthread_mutex_t m;
  nslDeviceInfo info;
  nslHandle handle;
  uint32 timeout;//in ms
  int fibrePort;//the port number on sl240 card.
  int sl240Opened;//sl240 has been opened okay?
  int threadAffinity;
  int threadPriority;
}MirrorStruct;

/**
   Find out if this SO library supports your mirror.
*/
int mirrorQuery(char *name){
  int rtval=0;
#ifdef OLD
  rtval=(strcmp(name,"dmcSL240mirror")!=0);
#endif
  return rtval;
}

/**
   Free mirror/memory/sl240
*/
void dofree(MirrorStruct *mirstr){
  if(mirstr!=NULL){
    if(mirstr->arr!=NULL)
      free(mirstr->arr);
    pthread_cond_destroy(&mirstr->cond);
    pthread_mutex_destroy(&mirstr->m);
    if(mirstr->sl240Opened==1){
      nslClose(&mirstr->handle);
    }
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
   The thread that does the work - copies actuators, and sends via SL240
*/
void* worker(void *mirstrv){
  MirrorStruct *mirstr=(MirrorStruct*)mirstrv;
  uint32 flagsIn,status,sofWord=0xa5a5a5a5,bytesXfered,flagsOut;
  setThreadAffinityForDMC(mirstr->threadAffinity,mirstr->threadPriority);
  pthread_mutex_lock(&mirstr->m);
  while(mirstr->open){
    pthread_cond_wait(&mirstr->cond,&mirstr->m);//wait for actuators.
    if(mirstr->open){
      //Now send the header...
      mirstr->err=0;
      flagsIn = NSL_DMA_USE_SYNCDV;
      status = nslSend(&mirstr->handle, (void *)&sofWord, sizeof(uint32), flagsIn, mirstr->timeout,&bytesXfered, &flagsOut);
      if (status == NSL_TIMEOUT) {
	printf("Tx timeout frame\n");
	mirstr->err=1;
      } else if (status == NSL_LINK_ERROR) {
	printf("Link error detected frame\n");
	mirstr->err=1;
      }else if(status==NSL_SUCCESS){
	if(sizeof(uint32)!=bytesXfered){
	  printf("%ld bytes requested, %d bytes sent\n",sizeof(uint32), bytesXfered);
	  mirstr->err=1;
	}
      }else{
	printf("error: %s\n", nslGetErrStr(status));
	mirstr->err=1;
      }
      //now send the data.
      flagsIn = 0;
      status = nslSend(&mirstr->handle,(void *)mirstr->arr,mirstr->arrsize,flagsIn,mirstr->timeout,&bytesXfered, &flagsOut);
      if (status == NSL_TIMEOUT) {
	printf("Tx acts timeout\n");
	mirstr->err=1;
      } else if (status == NSL_LINK_ERROR) {
	printf("Link error acts detected\n");
	mirstr->err=1;
      } else if (status == NSL_SUCCESS){
	if(mirstr->arrsize!=bytesXfered){
	  printf("%d bytes requested, %d bytes sent\n",mirstr->arrsize, bytesXfered);
	  mirstr->err=1;
	}
      } else {
	printf("%s\n", nslGetErrStr(status));
	mirstr->err=1;
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

#define RETERR dofree(mirstr);*mirrorHandle=NULL;return 1;
int mirrorOpen(char *name,int narg,int *args, int nacts,void **mirrorHandle,circBuf *rtcErrorBuf,circBuf *rtcActuatorBuf,unsigned int frameno,char *buf){
  int err;
  MirrorStruct *mirstr;
  uint32 status;
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
  //array has to be a whole number of int32 for the sl240, ie multiple of 4 bytes.
  mirstr->arrsize=(HDRSIZE+nacts*sizeof(unsigned short)+3)&(~0x3);
  if((mirstr->arr=malloc(mirstr->arrsize))==NULL){
    printf("couldn't malloc arr\n");
    dofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  memset(mirstr->arr,0,mirstr->arrsize);
  if(narg==4){
    mirstr->timeout=args[0];
    mirstr->fibrePort=args[1];
    mirstr->threadAffinity=args[2];
    mirstr->threadPriority=args[3];
  }else{
    printf("wrong number of args - should be timeout, fibrePort, thread affinity, thread priority\n");
    dofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }

  status = nslOpen(mirstr->fibrePort, &mirstr->handle);
  if (status != NSL_SUCCESS) {
    printf("Failed to open SL240: %s\n\n", nslGetErrStr(status));
    dofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  mirstr->sl240Opened=1;
  status = nslGetDeviceInfo(&mirstr->handle, &mirstr->info);
  if (status != NSL_SUCCESS) {
    printf("Failed to get SL240 device info: ");
    RETERR;
  }

  printf("SL240 Device info:\n==================\n");
  printf("Unit no.\t %d\n", mirstr->info.unitNum);
  printf("Board loc.\t %s\n", mirstr->info.boardLocationStr);
  printf("Serial no.\t 0x%x.%x\n",mirstr->info.serialNumH, mirstr->info.serialNumL);
  printf("Firmware rev.\t 0x%x\n", mirstr->info.revisionID);
  printf("Driver rev.\t %s\n", mirstr->info.driverRevisionStr);
  printf("Fifo size\t %dM\n", mirstr->info.popMemSize/0x100000);
  printf("Link speed\t %d MHz\n", mirstr->info.linkSpeed);
  printf("No. links\t %d\n\n\n", mirstr->info.numLinks);

  status = nslSetState(&mirstr->handle, NSL_EN_EWRAP, 0);
  if (status != NSL_SUCCESS)
    {RETERR;}
  status = nslSetState(&mirstr->handle,NSL_EN_RECEIVE, 1);
  if (status != NSL_SUCCESS)
    {RETERR;}
  status = nslSetState(&mirstr->handle,NSL_EN_RETRANSMIT, 0);
  if (status != NSL_SUCCESS)
    {RETERR;}
  status = nslSetState(&mirstr->handle,NSL_EN_CRC, 1);
  if (status != NSL_SUCCESS)
    {RETERR;}
  status = nslSetState(&mirstr->handle,NSL_EN_FLOW_CTRL, 0);
  if (status != NSL_SUCCESS)
    {RETERR;}
  status = nslSetState(&mirstr->handle,NSL_EN_LASER, 1);
  if (status != NSL_SUCCESS)
    {RETERR;}
  status = nslSetState(&mirstr->handle,NSL_EN_BYTE_SWAP, 0);
  if (status != NSL_SUCCESS)
    {RETERR;}
  status = nslSetState(&mirstr->handle,NSL_EN_WORD_SWAP, 0);
  if (status != NSL_SUCCESS)
    {RETERR;}
  status = nslSetState(&mirstr->handle, NSL_STOP_ON_LNK_ERR, 1);
  if (status != NSL_SUCCESS)
    {RETERR;}
  status = nslSetState(&mirstr->handle,NSL_EN_TRANSMIT, 1);
  if (status != NSL_SUCCESS)
    {RETERR;}

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
    memcpy(&mirstr->arr[HDRSIZE/sizeof(unsigned short)],data,sizeof(unsigned short)*mirstr->nacts);
    ((int*)mirstr->arr)[0]=frameno;
    //Wake up the thread.
    pthread_cond_signal(&mirstr->cond);
    pthread_mutex_unlock(&mirstr->m);
  }else{
    err=1;
  }
  return err;
}
