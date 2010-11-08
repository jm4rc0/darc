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

A library for figure sensor input, which simply places the actuator demands straight onto the mirror.  This also updates the RTC with the current frame number from the actuator demands.

For use with the single channel SL240 card (SC==single channel)
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>
//#include <nslapi.h>
#include <fxsl.h>
#include <fxslapi.h>
#include "rtcfigure.h"
//#include "powerdaq.h"
//#include "powerdaq32.h"

typedef unsigned int uint32;

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
  /*HANDLE*/int handle;//the sl240 handle - is defined as type int in fxsl.h but a name clash with powerdaq.h means have to define it here...
  fxsl_configstruct cfg;
  //nslHandle handle;
  //nslDeviceInfo info;
  uint32 timeout;
  uint32 fibrePort;
  int threadAffinity;
  int threadPriority;
  unsigned int *frameno;
  char *arr;
  int arrsize;
  int err;
  
}figureStruct;

char *
GetErrStr(fx_ulong status)
{
  char *p;
  switch (status) {
  case FXSL_SUCCESS: p = "Success";
    break;
  case FXSL_SYSTEM_ERROR: p = "System Error";
    break;
  case FXSL_BAD_PARAMETER: p = "Bad Parameter";
    break;
  case FXSL_LINK_ERROR: p = "Link Error";
    break;
  case FXSL_NO_DATA: p = "No Data";
    break;
  case FXSL_BUSY: p = "Busy";
    break;
  case FXSL_DRIVER_ERROR: p = "Driver Error";
    break;
  case FXSL_TIMEOUT: p = "Timeout";
    break;
/*   case FXSL_CALL_UNSUPPOPRTED: p = "Unsupported Call"; */
/*     break; */
  case FXSL_INSUFFICIENT_RESOURCES: p = "Insufficient Resources";
    break;
  case FXSL_CRC_ERROR: p = "CRC Error";
    break;
  case FXSL_FIFO_OVERFLOW: p = "Receive FIFO Overflow";
    break;
  default: p = "Unknown Error";
  }
  return (p);
}

int figureDofree(void **figureHandle){
  figureStruct *f;
  printf("TODO - figureDofree\n");
  if(*figureHandle!=NULL){
    f=(figureStruct*)*figureHandle;
    if(f->sl240Opened)
      fxsl_close(f->handle);
    f->sl240Opened=0;
    if(f->arr!=NULL)free(f->arr);
    f->arr=NULL;
    free(*figureHandle);
  }

  *figureHandle=NULL;
  return 0;
}

fx_ulong
sl240GetSOF(HANDLE handle, fx_long timeout,int attempts)
{
  fx_ulong status;
  fxsl_configstruct cfg;
  /* FXSL documentation recommends recv buffer
   * be at least 4 bytes larger than largest expected frame
   * when using SYNC */
  unsigned int sofWord;//[0x4000];  
  unsigned int i;
  fx_ulong flagsOut, bytesXfered;

  status = fxsl_get_config(handle, &cfg);
  if (status != FXSL_SUCCESS) {
    printf("Failed to read SL240 config\n");
  } else {
    cfg.convert_dsync = 1;
    status = fxsl_set_config(handle, &cfg);
    if (status != FXSL_SUCCESS) {
      printf("Failed to set SL240 config\n");
    } else {
      for (i = 0; i <attempts; i++) {
	flagsOut = FXSL_USE_SYNC_DVALID;
	status = fxsl_recv(handle, (void *)&sofWord, 0x4, &flagsOut, timeout, 
			   &bytesXfered);
	if (status==FXSL_SUCCESS && (flagsOut & FXSL_USE_SYNC_DVALID))
	  break;
	else if(status==FXSL_TIMEOUT){
	  printf("Timeout waiting for figure sensor SoF\n");
	  break;
	}else{
	  status = FXSL_SYSTEM_ERROR;
	}
      }
    }
  }
  return (status);
}

fx_ulong
sl240GetBytes(HANDLE handle, fx_ubyte *buffer, fx_ulong nBytes, fx_ulong timeout)
{
  fx_ulong status;
  fxsl_configstruct cfg;
  fx_ulong flagsOut, bytesXfered;


  status = fxsl_get_config(handle, &cfg);
  if (status != FXSL_SUCCESS) {
    printf("Failed to read SL240 config\n");
  } else {
    cfg.convert_dsync = 0;
    status = fxsl_set_config(handle, &cfg);
    if (status != FXSL_SUCCESS) {
      printf("Failed to set SL240 config\n");
    } else {
      flagsOut = 0;
      status = fxsl_recv(handle, buffer, nBytes, &flagsOut, timeout, 
			 &bytesXfered);
      if (status!=FXSL_SUCCESS || (bytesXfered != nBytes)) {
	printf("sl240GetBytes: %d bytes requested, %u bytes received, error %s\n",
	       nBytes, bytesXfered,GetErrStr(status));
	status = FXSL_SYSTEM_ERROR;
      }
    }
  }
  return (status);
}

int figureGetActuators(figureStruct *f){
  //actuators should be placed into f->acts.
  //First get start of frame
  //uint32 flagsIn;
  //uint32 sofWord;
  //uint32 bytesXfered;
  //uint32 flagsOut;
  uint32 status;
  int err=0;//,done=0;
  //  int syncerrmsg=0;
  //  int i;
  //nslSeq seq;
  //Read 4 bytes until get a start of frame, for at most nacts... after which, consider it an error.
  //for(i=0; i<f->nacts; i++){
  status=sl240GetSOF(f->handle,(fx_long)f->timeout,f->nacts+4);
  //  if(status==FXSL_SUCCESS){
  //  break;
  //}
  //}
  if(status!=FXSL_SUCCESS){
    printf("No Sync for figure sensor actuators\n");
    err=-1;
  }else{
    status=sl240GetBytes(f->handle,(fx_ubyte*)f->arr,f->arrsize,(fx_ulong)f->timeout);
    if(status!=FXSL_SUCCESS){
      printf("Error getting bytes\n");
      err=1;
    }
      
  }
  return err;
}
  /*  flagsIn = NSL_DMA_USE_SYNCDV;
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
  }*/
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

//#define MASKED_MODIFY(oldval, modval, mask) (((oldval) & ~(mask)) | ((modval) & (mask)))
/**
   This does the same as nslmon -u X --clrf
   Clears the receive fifo.
   Copied from the source code for nslmon.
*/
/*
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
*/
//#undef MASKED_MODIFY


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
      //Now sum actuators.
      s=0;
      for(i=0; i<f->nacts; i++){
	s+=(int)f->acts[i];
      }
      pist=(float)s/(float)f->nacts;
      //And update the RTC actuator frame number.
      pthread_mutex_lock(&f->m);
      *(f->frameno)=((unsigned int*)f->arr)[0];//copy frame number
      //printf("Sending actuators for frame %u (first actuator %d)\n",((unsigned int*)f->arr)[0],(int)f->acts[0]);
      //Note - since we're not providing the RTC with the actuator demands, we don't need to wake it up.
      //However, we do need to allocate the actsRequired array so that the RTC picks up the frame number.
      if(*(f->actsRequired)==NULL){
	if((*(f->actsRequired)=malloc(f->nacts*sizeof(float)))==NULL){
	  printf("Error actsRequired malloc\n");
	  f->err=1;
	}
      }
      if(*(f->actsRequired)!=NULL){
	//Copy the actuators, removing piston, and converting to float.
	for(i=0; i<f->nacts; i++){
	  (*f->actsRequired)[i]=(float)(f->acts[i])-pist;
	}
      }
      //wake up the rtc figure sensor thread so it can immediately send the new demands...
      pthread_cond_signal(&f->cond);
      pthread_mutex_unlock(&f->m);
      
    }else{
      printf("Error getting actuators\n");
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
   Set up the SL240 card (single channel).  EJY.
*/
fx_ulong
sl240Setup(HANDLE handle,fxsl_configstruct cfg)
{
  fx_ulong status;
  fx_ulong cr;
  //fxsl_configstruct cfg;

  /* Reset the board - write bit 1 of register 4 */
  status = fxsl_read_CR(handle, 0x4, &cr);
  if (status != FXSL_SUCCESS) {
    printf( "Failed to read SL240 control reg.: %s\n",
	    GetErrStr(status));
    return(status);
  }
  status = fxsl_write_CR(handle, 0x4, cr | 0x2);
  if (status!= FXSL_SUCCESS) {
    printf("Failed to reset SL240 board: %s\n",
	    GetErrStr(status));
    return(status);
  }

  status = fxsl_get_config(handle, &cfg);
  if (status != FXSL_SUCCESS) {
    printf("Failed to read SL240 config\n");
  } else {

    cfg.useCRC = 1;
    cfg.use_flow_control = 0;
    cfg.swapBytes = 0;
    cfg.halt_on_link_errors = 1;
    cfg.loopConfig = 0;
    status = fxsl_set_config(handle, &cfg);
    if (status != FXSL_SUCCESS) {
      printf("Failed to set SL240 config\n");
    }
  }
  if (status != FXSL_SUCCESS) 
    printf("sl240Setup: %s\n", GetErrStr(status));
  return status;
}

/**
   Open a channel for reading actuator setpoints into this figure sensor.  Must be of type name.  Args are passed in an int array of size n, which can be cast if necessary.  Any state data is returned in figureHandle, which should be NULL if an error arises.
   Name is used if a library can support more than one camera, and to check that the currently compiled library is what you want it to be.
   The mutex should be obtained whenever new actuator setpoints arrive and are placed into actsRequired.  actsRequired should be allocated.
*/

int figureOpen(char *name,int n,int *args,char *buf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **figureHandle,int nacts,pthread_mutex_t m,pthread_cond_t cond,float **actsRequired,unsigned int *frameno){
  int err=0;
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
    status = fxsl_open((fx_ulong)f->fibrePort,(int*) &f->handle);
    if (status != FXSL_SUCCESS) {
      printf("Failed to open figuresensor SL240 port %d: %s\n\n",f->fibrePort,GetErrStr(status));
      err=1;
      /*status = nslOpen(f->fibrePort, &f->handle);
    if (status != NSL_SUCCESS) {
      printf("Failed to open figuresensor SL240 port %d: %s\n\n",f->fibrePort, nslGetErrStr(status));
      err=1;*/
    }else{
      f->sl240Opened=1;
      status = fxsl_get_config(f->handle, &f->cfg);
      if (status != FXSL_SUCCESS) {
	printf("Failed to read SL240 config\n");
	err=1;
      /*
      status = nslGetDeviceInfo(&f->handle, &f->info);
      if (status != NSL_SUCCESS) {
	printf("Failed to get SL240 device info: ");
	err=1;*/
      }else{
	status=sl240Setup(f->handle,f->cfg);
	if (status != FXSL_SUCCESS) {
	  printf("sl240Setup failed: %s\n", GetErrStr(status));
	  fxsl_close(f->handle);
	  err=1;
	}
	printf("TODO - do we need to clear the buffers in sl240 card?\n");
	/*printf("\n\nSL240 Device info:\n");
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
	  figureClearReceiveBuffer(f);*/
      }
    }
    f->open=1;
  }
  printf("Initialised SL240, err=%d\n",err);
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
    printf("waiting for join\n");
    pthread_join(f->threadid,NULL);
    printf("joined\n");
  }
  figureDofree(figureHandle);
  *figureHandle=NULL;
  printf("figure closed\n");
  return 0;
}
/**
New parameters ready - use if you need to...
*/
int figureNewParam(void *figureHandle,char *buf,unsigned int frameno,arrayStruct *arr){
  return 0;
}
