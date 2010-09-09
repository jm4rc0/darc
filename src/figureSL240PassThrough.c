/**
   The code here is used to create a shared object library, which can then be swapped around depending on which mirrors/interfaces you have in use, ie you simple rename the mirror file you want to mirror.so (or better, change the soft link), and restart the coremain.

A library for figure sensor input, which simply places the actuator demands straight onto the mirror.  This also updates the RTC with the current frame number from the actuator demands.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
//#include <signal.h>
#include <nslapi.h>
//#include <fxsl.h>
//#include <fxslapi.h>
#include "rtcfigure.h"
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

#define HDRSIZE 8 //the size of the SL240 header expected...
typedef struct{
  int open;
  int sl240Opened;
  pthread_mutex_t m;
  pthread_cond_t cond;
  float **actsRequired;//shared with the figure sensor RTC core.
  pthread_t threadid;
  unsigned short *acts;//temporary space for reading actuators into
  int nacts;
  nslHandle handle;
  nslDeviceInfo info;
  uint32 timeout;
  int fibrePort;
  int threadAffinity;
  int threadPriority;
  unsigned int *frameno
  char *arr;
  int arrsize;
  int mirhandle;
  tState state;//state of the acquisition session.
  int board;
  int adapterType;
  
}figureStruct;


void CleanUpSingleAO(figureStruct *pAoData){
   if(pAoData->state == running){
      pAoData->state = configured;
   }
   if(pAoData->state == configured){
     errorChk(_PdAOutReset(pAoData->mirhandle));
     // need also to call this function if the board is a PD2-AO-xx
     if(pAoData->adapterType & atPD2AO){
       errorChk(_PdAO32Reset(pAoData->mirhandle));
     }
     pAoData->state = unconfigured;
   }
   if(pAoData->mirhandle > 0 && pAoData->state == unconfigured){
     errorChk(PdAcquireSubsystem(pAoData->mirhandle, AnalogOut, 0));
   }
   pAoData->state = closed;
}

int figureDofree(void **figureHandle){
  figureStruct *f;
  printf("TODO - figureDofree\n");
  if(*figureHandle!=NULL){
    f=(figureStruct*)*figureHandle;
    if(f->sl240Opened)
      nslClose(&f->handle);
    f->sl240Opened=0;
    if(f->arr!=NULL)free(f->arr);
    f->arr=NULL;
    CleanUpSingleAO(mirstr);

    free(*figureHandle);
  }

  *figureHandle=NULL;
  return 0;
}

int InitSingleAO(figureStruct *f){
   Adapter_Info adaptInfo;
   // get adapter type
   errorChk(_PdGetAdapterInfo(f->board, &adaptInfo));
   f->adapterType = adaptInfo.atType;
   if(f->adapterType & atMF)
     printf("This is an MFx board\n");
   else
     printf("This is an AO32 board\n");
   f->mirhandle = PdAcquireSubsystem(f->board, AnalogOut, 1);
   if(f->handle < 0){
      printf("SingleAO: PdAcquireSubsystem failed\n");
      f->state=closed;
      return 1;
   }
   f->state = unconfigured;
   errorChk(_PdAOutReset(f->mirhandle));
   // need also to call this function if the board is a PD2-AO-xx
   if(f->adapterType & atPD2AO){
      errorChk(_PdAO32Reset(f->mirhandle));
   }
   return 0;
}


int figureGetActuators(figureStruct *f){
  //actuators should be placed into f->acts.
  //First get start of frame
  uint32 flagsIn;
  uint32 sofWord;
  uint32 bytesXfered;
  uint32 flagsOut;
  uint32 status;
  int err=0,done=0;
  int syncerrmsg=0;
  //nslSeq seq;
  flagsIn = NSL_DMA_USE_SYNCDV;
  while(done==0){
    status = nslRecv(&f->handle, (void *)&sofWord, sizeof(uint32), flagsIn, f->timeout, &bytesXfered, &flagsOut, NULL);
    if (status == NSL_SUCCESS) {
      if (flagsOut & NSL_DMA_USE_SYNCDV) {
	//printf("SYNC frame data = 0x%x bytes %d\n", sofWord,bytesXfered);
	done=1;
	err=0;
	//printf("New frame %d\n",cam);
      }else{//it may be here that the buffer is partially filled - in which case, we should continue reading 4 bytes for up to a full frame size, and see if we get the SYNC_DV.
	if((syncerrmsg%f->nacts)==0){
	  printf("WARNING: SYNCDV not set - figure library may be out of sync, sof=%#x bytes %d timeout %d\n",sofWord,bytesXfered,f->timeout);
	}
	syncerrmsg++;
	err=-1;
      }
    }else if(status!=NSL_TIMEOUT){
      printf("camerror: %s\n",nslGetErrStr(status));
      err=-1;
      done=1;
    }else{
      printf("Timeout waiting for new frame (figure library)\n");
      err=1;
      done=1;
    }
  }
  if(syncerrmsg>0 && err==0)//previously printed a sync warning... so now print an ok msg
    printf("Start of frame received okay after %d tries\n",syncerrmsg);
  if(err==0){
    //now read all the actuators.
    flagsIn=0;
    flagsOut=0;
    status = nslRecv(&f->handle, (void *)f->arr,f->arrsize, flagsIn, f->timeout,&bytesXfered, &flagsOut, NULL);
    if (status == NSL_TIMEOUT) {
      printf("Received timeout (figure library)\n");
      err=1;
    } else if (status == NSL_LINK_ERROR) {
      printf("Link error detected\n");
      err=-1;
    } else if (status == NSL_SUCCESS){
      if(f->arrsize!=bytesXfered){
	printf("%d bytes requested, got %d\n", f->arrsize,bytesXfered);
	err=-1;
      }else{
	//printf("got data okay\n");
      }
    }else{
      printf("%s\n", nslGetErrStr(status));
      err=-1;
    }
  }
  return err;
}
/*
int figureSubtractPiston(figureStruct *f){
  int i;
  float s=0.;
  for(i=0; i<f->nacts; i++)
    s+=f->acts[i];
  s/=f->nacts;
  for(i=0; i<f->nacts; i++)
    f->acts[i]-=s;
  return 0;
  }*/

#define MASKED_MODIFY(oldval, modval, mask) (((oldval) & ~(mask)) | ((modval) & (mask)))
/**
   This does the same as nslmon -u X --clrf
   Clears the receive fifo.
   Copied from the source code for nslmon.
*/

int figureClearReceiveBuffer(figureStruct *f){
  uint32 state;
  printf("clearing receive buffer (clrf)\n");
  state = nslReadCR(&f->handle, 0x8);
  state = MASKED_MODIFY(state, 0x2000, 0x00002000);
  
  nslWriteCR(&f->handle, 0x8, state);
  
  //usysMsTimeDelay(10);
  usleep(10000);
  
  state = MASKED_MODIFY(state, 0, 0x00002000);
  nslWriteCR(&f->handle, 0x8, state);
  printf("clearing receive buffer (clrf) DONE\n");
  return 0;
}

#undef MASKED_MODIFY


int figureSetThreadAffinityAndPriority(int threadAffinity,int threadPriority){
  int i;
  cpu_set_t mask;
  int ncpu;
  struct sched_param param;
  printf("Getting CPUs\n");
  ncpu= sysconf(_SC_NPROCESSORS_ONLN);
  printf("Got %d CPUs\n",ncpu);
  CPU_ZERO(&mask);
  printf("Setting %d CPUs\n",ncpu);
  for(i=0; i<ncpu; i++){
    if(((threadAffinity)>>i)&1){
      CPU_SET(i,&mask);
    }
  }
  printf("Thread affinity %d\n",threadAffinity&0xffff);
  if(sched_setaffinity(0,sizeof(cpu_set_t),&mask))
    printf("Error in sched_setaffinity: %s\n",strerror(errno));
  printf("Setting setparam\n");
  param.sched_priority=threadPriority;
  if(sched_setparam(0,&param)){
    printf("Error in sched_setparam: %s - probably need to run as root if this is important\n",strerror(errno));
  }
  return 0;
}

/**
   A thread started by figureOpen and stopped by figureClose, which get new actuator setpoints when they are ready, and copies them into actsRequired.
*/
void *figureWorker(void *ff){
  figureStruct *f=ff;
  int s;
  float pist;
  int i;
  figureSetThreadAffinityAndPriority(f->threadAffinity,f->threadPriority);
  while(f->open){
    //get the actuators from the actuator interface
    f->err=figureGetActuators(f);
    if(f->err==0){
      //write the actuators directly to the mirror.
      for(i=0; i<f->nacts; i++){
	f->err|=_PdAO32Write(f->mirhandle,i,f->acts[i]);
      }

      //And update the RTC actuator frame number.
      pthread_mutex_lock(&f->m);
      *(f->frameno)=((unsigned int*)f->arr)[0];//copy frame number
      //Note - since we're not providing the RTC with the actuator demands, we don't need to wake it up.
      //However, we do need to allocate the actsRequired array so that the RTC picks up the frame number.
      if(*(f->actsRequired)==NULL){
	if((*(f->actsRequired)=malloc(f->nacts*sizeof(float)))==NULL){
	  printf("Error actsRequired malloc\n");
	}else{
	  memset(*f->actsRequired,0,sizeof(float)*f->nacts);
	}
      }
      pthread_mutex_unlock(&f->m);


    }
  }
  //do some clearing up.
  pthread_mutex_lock(&f->m);
  if(*(f->actsRequired)!=NULL){
    free(*f->actsRequired);
    *f->actsRequired=NULL;
  }
  pthread_mutex_unlock(&f->m);
  return NULL;
}

/**
   Open a channel for reading actuator setpoints into this figure sensor.  Must be of type name.  Args are passed in an int array of size n, which can be cast if necessary.  Any state data is returned in figureHandle, which should be NULL if an error arises.
   Name is used if a library can support more than one camera, and to check that the currently compiled library is what you want it to be.
   The mutex should be obtained whenever new actuator setpoints arrive and are placed into actsRequired.  actsRequired should be allocated.
*/

int figureOpen(char *name,int n,int *args,char *buf,circBuf *rtcErrorBuf,char *prefix,void **figureHandle,int nacts,pthread_mutex_t m,pthread_cond_t cond,float **actsRequired,unsigned int *frameno){
  int err;
  figureStruct *f=NULL;
  uint32 status;
  printf("Initialising figure %s\n",name);
  if((*figureHandle=malloc(sizeof(figureStruct)))==NULL){
    printf("Error malloc figureHandle\n");
    err=1;
  }else{
    f=(figureStruct*)*figureHandle;
    memset(f,0,sizeof(figureStruct));
    if(n==4){
      f->timeout=args[0];
      f->fibrePort=args[1];
      f->threadAffinity=args[2];
      f->threadPriority=args[3];
    }else{
      printf("Wrong number of figure sensor library arguments - should be 4, was %d\n",n);
      err=1;
    }
  }
  if(err==0){
    f->m=m;
    f->cond=cond;
    f->actsRequired=actsRequired;
    f->nacts=nacts;
    f->frameno=frameno;

    //DMA array has to be a whole number of int32 for the sl240, ie multiple of 4 bytes.
    f->arrsize=(HDRSIZE+nacts*sizeof(unsigned short)+3)&(~0x3);
    if((f->arr=malloc(f->arrsize))==NULL){
      printf("couldn't malloc arr\n");
      err=1;
    }else{
      memset(f->arr,0,f->arrsize);
      f->acts=(unsigned short*)&(f->arr[HDRSIZE]);
    }
    status = nslOpen(f->fibrePort, &f->handle);
    if (status != NSL_SUCCESS) {
      printf("Failed to open figuresensor SL240 port %d: %s\n\n",f->fibrePort, nslGetErrStr(status));
      err=1;
    }else{
      f->sl240Opened=1;
      status = nslGetDeviceInfo(&f->handle, &f->info);
      if (status != NSL_SUCCESS) {
	printf("Failed to get SL240 device info: ");
	err=1;
      }else{
	printf("\n\nSL240 Device info:\n");
	printf("Unit no.\t %d\n", f->info.unitNum);
	printf("Board loc.\t %s\n", f->info.boardLocationStr);
	printf("Serial no.\t 0x%x.%x\n", f->info.serialNumH,f->info.serialNumL);
	printf("Firmware rev.\t 0x%x\n", f->info.revisionID);
	printf("Driver rev.\t %s\n", f->info.driverRevisionStr);
	printf("Fifo size\t %dM\n", f->info.popMemSize/0x100000);
	printf("Link speed\t %d MHz\n", f->info.linkSpeed);
	printf("No. links\t %d\n\n\n", f->info.numLinks);
	//set up card state.
	status = nslSetState(&f->handle, NSL_EN_EWRAP, 0);
	if (status != NSL_SUCCESS){
	  printf("%s\n",nslGetErrStr(status));err=1;}
	status = nslSetState(&f->handle,NSL_EN_RECEIVE, 1);
	if (status != NSL_SUCCESS){
	  printf("%s\n",nslGetErrStr(status));err=1;}
	status = nslSetState(&f->handle,NSL_EN_RETRANSMIT, 0);
	if (status != NSL_SUCCESS){
	  printf("%s\n",nslGetErrStr(status));err=1;}
	status = nslSetState(&f->handle,NSL_EN_CRC, 1);
	if (status != NSL_SUCCESS){
	  printf("%s\n",nslGetErrStr(status));err=1;}
	status = nslSetState(&f->handle,NSL_EN_FLOW_CTRL, 0);
	if (status != NSL_SUCCESS){
	  printf("%s\n",nslGetErrStr(status));err=1;}
	status = nslSetState(&f->handle,NSL_EN_LASER, 1);
	if (status != NSL_SUCCESS){
	  printf("%s\n",nslGetErrStr(status));err=1;}
	status = nslSetState(&f->handle,NSL_EN_BYTE_SWAP, 0);
	if (status != NSL_SUCCESS){
	  printf("%s\n",nslGetErrStr(status));err=1;}
	status = nslSetState(&f->handle,NSL_EN_WORD_SWAP, 0);
	if (status != NSL_SUCCESS){
	  printf("%s\n",nslGetErrStr(status));err=1;}
	status = nslSetState(&f->handle, NSL_STOP_ON_LNK_ERR, 1);
	if (status != NSL_SUCCESS){
	  printf("%s\n",nslGetErrStr(status));err=1;}
	status = nslSetState(&f->handle,NSL_EN_RECEIVE, 1);
	if (status != NSL_SUCCESS){
	  printf("%s\n",nslGetErrStr(status));err=1;}
	figureClearReceiveBuffer(f);
      }
    }
    f->open=1;
  }

  if(err==0){
    //initialise acquisition session
    if(InitSingleAO(f)){//failed...
      printf("Failed to initSingleAO\n");
      err=1;
    }else{
      // set configuration - _PdAOutReset is called inside _PdAOutSetCfg
      aoCfg = 0;
      errorChk(_PdAOutSetCfg(f->mirhandle, aoCfg, 0));
      f->state = configured;
      //Start SW trigger
      errorChk(_PdAOutSwStartTrig(f->mirhandle));
      f->state = running;
      pthread_create(&f->mirthreadid,NULL,mirworker,f);
    }

  }
  printf("done nsl\n");
  if(err==0 && pthread_create(&f->threadid,NULL,figureWorker,f)){
    printf("pthread_create figureWorker failed\n");
    err=1;
  }
  if(err){
    figureDofree(figureHandle);
  }
  return err;
}

/**
   Close a camera of type name.  Args are passed in the float array of size n, and state data is in camHandle, which should be freed and set to NULL before returning.
*/
int figureClose(void **figureHandle){
  figureStruct *f;
  printf("Closing figure\n");
  if(*figureHandle!=NULL){
    f=(figureStruct*)*figureHandle;
    f->open=0;
    //Wait for the thread to finish.
    pthread_join(f->threadid,NULL);
  }
  figureDofree(figureHandle);
  *figureHandle=NULL;
  return 0;
}
/**
New parameters ready - use if you need to...
*/
int figureNewParam(void *figureHandle,char *buf,unsigned int frameno){
  return 0;
}
