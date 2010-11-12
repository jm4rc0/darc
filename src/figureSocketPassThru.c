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
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>
//#include <nslapi.h>
#include "rtcfigure.h"
#include "powerdaq.h"
#include "powerdaq32.h"
#include "darc.h"


typedef enum{
  ACTINIT,
  ACTMAPPING,
  ACTOFFSET,
  ACTSCALE,
  ACTSOURCE,
  FIGUREDEBUG,
  NBUFFERVARIABLES;
}figureNames;

#define makeParamNames() bufferMakeNames(NBUFFERVARIABLES,\
 "actInit","actMapping","actOffset","actScale","actSource","figureDebug" \
					 );

#define errorChk(functionCall) {int error; if((error=functionCall)<0) { \
	                           fprintf(stderr, "Error %d at line %d in function call %s\n", error, __LINE__, #functionCall); \
	                           exit(EXIT_FAILURE);}}
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
  pthread_mutex_t mInternal;
  pthread_cond_t cond;
  float **actsRequired;//shared with the figure sensor RTC core.
  pthread_t threadid;
  unsigned short *acts;//temporary space for reading actuators into
  int nacts;
  int handle;
  int socket;//the socket for listening on
  int hasclient;
  int client;//client socket
  unsigned short port;//the port to listen on.
  int timeout;
  int threadAffinity;
  int threadPriority;
  unsigned int *frameno;
  char *arr;
  int arrsize;
  int mirhandle;
  tState state;//state of the acquisition session.
  int board;
  int adapterType;
  int err;
  int debug;
  int *actMapping;//map actuator number to DAC number.
  int initLen;//number of entries in actInit.
  unsigned short *actInit;//only used when library is opened. Can be used to set default values to unused actuators.
  int actMappingLen;
  int *actSource;
  float *actScale;
  float *actOffset;
  int socketOpen;
  char *paramNames;
  int index[NBUFFERVARIABLES];
  void *values[NBUFFERVARIABLES];
  char dtype[NBUFFERVARIABLES];
  int nbytes[NBUFFERVARIABLES];

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
    if(f->socketOpen){
      close(f->socket);//close the socket
    }
    f->socketOpen=0;
    if(f->arr!=NULL)free(f->arr);
    f->arr=NULL;
    CleanUpSingleAO(f);
    //if(f->mInternal!=NULL){
      pthread_mutex_destroy(&f->mInternal);
      //}
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

int openSocket(figureStruct *f){
  //opens a listening socket
  //Create the socket and set up to accept connections.
  struct sockaddr_in name;
  int err=0;
  if((f->socket=socket(PF_INET,SOCK_STREAM,0))<0){
    printf("Error opening socket\n");
    err=1;
  }else{
    name.sin_family=AF_INET;
    name.sin_port=htons(f->port);
    name.sin_addr.s_addr=htonl(INADDR_ANY);
    if(bind(f->socket,(struct sockaddr*)&name,sizeof(name))<0){
      printf("Unable to bind\n");
      err=1;
      close(f->socket);
      f->socket=0;
    }
  }
  if(err==0){
    if(listen(f->socket,1)<0){
      printf("Failed to listen on port\n");
      err=1;
    }
  }
  if(err==0)
    f->socketOpen=1;
  return err;
  
}

int readSocket(figureStruct *f){
  //reads socket for data, returing 1 if timeout after a certain time.
  int n=0;
  int rec;
  int err=0;
  while(n<f->arrsize && err==0){
    rec=recv(f->client,&f->arr[n],f->arrsize-n,0);
    if(rec>0){
      n+=rec;
    }else if(n<0){
      printf("Error in readSocket in figureSocketPassThru\n");
      err=-1;
    }else{
      printf("End of file for socket in figureSocketPassThru?\n");
      err=-1;
    }
  }
  if(err==0){//check the header...
    if(((unsigned int*)f->arr)[0]!=((0x5555<<16)|f->nacts)){
      printf("Error in header - got %u\n",((unsigned int*)f->arr)[0]);
      err=1;
    }
  }
  return err;



}

int figureGetActuators(figureStruct *f){
  //Return -1 on error, +1 on timeout, 0 if valid.

  //actuators should be placed into f->acts.
  //First get start of frame
  int err=0;//,done=0;
  //  int syncerrmsg=0;
  //  int i;
  //nslSeq seq;
  //Read 4 bytes until get a start of frame, for at most nacts... after which, consider it an error.
  //for(i=0; i<f->nacts; i++){
  if((err=readSocket(f))!=0 && f->debug!=0)
    printf("Error reading socket\n");
  return err;
}


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
  if(sched_setscheduler(0,SCHED_RR,&param))
    printf("sched_setscheduler: %s - probably need to run as root if this is important\n",strerror(errno));
  if(pthread_setschedparam(pthread_self(),SCHED_RR,&param))
    printf("error in pthread_setschedparam - maybe run as root?\n");
  return 0;
}

/**
   A thread started by figureOpen and stopped by figureClose, which get new actuator setpoints when they are ready, and copies them into actsRequired.
*/
void *figureWorker(void *ff){
  figureStruct *f=ff;
  //int s;
  //float pist;
  int i;
  struct sockaddr_in clientname;
  size_t size;
  figureSetThreadAffinityAndPriority(f->threadAffinity,f->threadPriority);
  if(f->open && f->actInit!=NULL){
    pthread_mutex_lock(&f->mInternal);//lock it so that actMapping doesn't change.
    for(i=0; i<f->initLen; i++){
      _PdAO32Write(f->mirhandle,i,f->actInit[i]);
      
    }
    pthread_mutex_unlock(&f->mInternal);//lock it so that actMapping doesn't change.
  }
  while(f->open){
    //wait for someone to connect...
    if((f->client=accept(f->socket,(struct sockaddr*)&clientname,&size))<0){
      printf("Failed to accept on socket\n");
    }else{
      printf("Connected from %s port %hd\n",inet_ntoa(clientname.sin_addr),ntohs(clientname.sin_port));
      f->hasclient=1;
    }
    while(f->hasclient){

      //get the actuators from the actuator interface
      if((i=figureGetActuators(f))<0){
	close(f->client);
	printf("Error getting actuators for figure sensor - closing client and listening again\n");
	f->hasclient=0;
      }else if(i==1){
	if(f->debug)
	  printf("Error getting actuator demands\n");
      }else{
	if(f->err==0){
	  //write the actuators directly to the mirror.
	  pthread_mutex_lock(&f->mInternal);//lock it so that actMapping etc doesn't change.
	  if(f->actMapping==NULL){
	    for(i=0; i<f->nacts; i++){
	      f->err|=_PdAO32Write(f->mirhandle,i,f->acts[i]);
	    }
	  }else{//actMapping is specified.
	    if(f->actSource==NULL){
	      if(f->actScale==NULL){
		if(f->actOffset==NULL){
		  for(i=0; i<f->actMappingLen; i++){
		    f->err|=_PdAO32Write(f->mirhandle,f->actMapping[i],f->acts[i]);
		  }
		}else{//actoffset defined.
		  for(i=0; i<f->actMappingLen; i++){
		    f->err|=_PdAO32Write(f->mirhandle,f->actMapping[i],(unsigned short)(f->acts[i]+f->actOffset[i]));
		  }
		}
	      }else{//actscale defined
		if(f->actOffset==NULL){
		  for(i=0; i<f->actMappingLen; i++){
		    f->err|=_PdAO32Write(f->mirhandle,f->actMapping[i],(unsigned short)(f->acts[i]*f->actScale[i]));
		  }
		}else{//actScale and actoffset defined
		  for(i=0; i<f->actMappingLen; i++){
		    f->err|=_PdAO32Write(f->mirhandle,f->actMapping[i],(unsigned short)(f->acts[i]*f->actScale[i]+f->actOffset[i]));
		  }
		}
	      }
	    }else{//actSource defined
	      if(f->actScale==NULL){
		if(f->actOffset==NULL){
		  for(i=0; i<f->actMappingLen; i++){
		    f->err|=_PdAO32Write(f->mirhandle,f->actMapping[i],f->acts[f->actSource[i]]);
		  }
		}else{//actSource and actoffset defined.
		  for(i=0; i<f->actMappingLen; i++){
		    f->err|=_PdAO32Write(f->mirhandle,f->actMapping[i],(unsigned short)(f->acts[f->actSource[i]]+f->actOffset[i]));
		  }
		}
	      }else{//actSource and actscale defined
		if(f->actOffset==NULL){
		  for(i=0; i<f->actMappingLen; i++){
		    f->err|=_PdAO32Write(f->mirhandle,f->actMapping[i],(unsigned short)(f->acts[f->actSource[i]]*f->actScale[i]));
		  }
		}else{//actSource and actScale and actoffset defined
		  for(i=0; i<f->actMappingLen; i++){
		    f->err|=_PdAO32Write(f->mirhandle,f->actMapping[i],(unsigned short)(f->acts[f->actSource[i]]*f->actScale[i]+f->actOffset[i]));
		  }
		}
	      }
	    }
	  }
	  pthread_mutex_unlock(&f->mInternal);
	  if(f->err==0){
	    //And update the RTC actuator frame number.
	    pthread_mutex_lock(&f->m);
	    *(f->frameno)=((unsigned int*)f->arr)[1];//copy frame number
	    if(f->debug==1)
	      printf("Sending actuators for frame %u (first actuator %d)\n",((unsigned int*)f->arr)[1],(int)f->acts[0]);
	    else if(f->debug==2){
	      printf("Sending actuators for frame %u\n",((unsigned int*)f->arr)[1]);
	      for(i=0;i<f->nacts; i++){
		printf("0x%4x ",f->acts[i]);
		if((i&0x7)==0)
		  printf("\n");
	      }
	      printf("\n\n");
	    }else if(f->debug==3){
	      printf("Sending actuators for frame %u\n",((unsigned int*)f->arr)[1]);
	      for(i=0;i<f->nacts; i++){
		printf("%6d ",(int)f->acts[i]);
		if((i&0x7)==0)
		  printf("\n");
	      }
	      printf("\n\n");
	      
	    }else if(f->debug==4){
	      printf("Sending actuators for frame %u\n",((unsigned int*)f->arr)[1]);
	      if(f->actMapping==NULL){
		for(i=0;i<f->nacts;i++){
		  printf("[%d] 0x%4x, ",i,f->acts[i]);
		  if((i&0x7)==0)
		    printf("\n");
		}
	      }else{
		for(i=0;i<f->actMappingLen;i++){
		  if(f->actSource==NULL){
		    if(f->actScale==NULL){
		      if(f->actOffset==NULL){
			printf("[%d] 0x%4x, ",f->actMapping[i],f->acts[i]);
		      }else{
			printf("[%d] 0x%4x, ",f->actMapping[i],(unsigned short)(f->acts[i]+f->actOffset[i]));
		      }
		    }else{
		      if(f->actOffset==NULL){
			printf("[%d] 0x%4x, ",f->actMapping[i],(unsigned short)(f->acts[i]*f->actScale[i]));
		      }else{
			printf("[%d] 0x%4x, ",f->actMapping[i],(unsigned short)(f->acts[i]*f->actScale[i]+f->actOffset[i]));
		      }
		    }
		  }else{
		    if(f->actScale==NULL){
		      if(f->actOffset==NULL){
			printf("[%d] 0x%4x, ",f->actMapping[i],f->acts[f->actSource[i]]);
		      }else{
			printf("[%d] 0x%4x, ",f->actMapping[i],(unsigned short)(f->acts[f->actSource[i]]+f->actOffset[i]));
		      }
		    }else{
		      if(f->actOffset==NULL){
			printf("[%d] 0x%4x, ",f->actMapping[i],(unsigned short)(f->acts[f->actSource[i]]*f->actScale[i]));
		      }else{
			printf("[%d 0x%4x, ",f->actMapping[i],(unsigned short)(f->acts[f->actSource[i]]*f->actScale[i]+f->actOffset[i]));
		      }
		    }
		  }
		  if((i&0x7)==0)
		    printf("\n");
		  
		}
		printf("\n\n");
	      }
	    }
	    //Note - since we're not providing the RTC with the actuator demands, we don't need to wake it up.
	    //However, we do need to allocate the actsRequired array so that the RTC picks up the frame number.
	    if(*(f->actsRequired)==NULL){
	      if((*(f->actsRequired)=malloc(f->nacts*sizeof(float)))==NULL){
		printf("Error actsRequired malloc\n");
		f->err=1;
	      }else{
		memset(*f->actsRequired,0,sizeof(float)*f->nacts);
	      }
	    }
	    pthread_mutex_unlock(&f->m);
	  }else{
	    printf("Error setting actuators\n");
	  }
	  
	}else{
	  printf("Error getting actuators\n");
	}
      }
    }
  }
  if(f->hasclient)
    close(f->client);
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

int figureOpen(char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **figureHandle,int nthreads,unsigned int frameno,unsigned int **figureframeno,int *figureframenoSize,int totCents,int nacts,pthread_mutex_t m,pthread_cond_t cond,float **actsRequired){
  int err=0;
  figureStruct *f=NULL;
  char *pn;
  printf("Initialising figure %s\n",name);
  if((pn=makeParamNames())==NULL){
    printf("Error making paramList - please recode figureSocketPassThru.c\n");
    *figureHandle=NULL;
    return 1;
  }

  if((*figureHandle=malloc(sizeof(figureStruct)))==NULL){
    printf("Error malloc figureHandle\n");
    err=1;
  }else{
    f=(figureStruct*)*figureHandle;
    memset(f,0,sizeof(figureStruct));
    f->paramNames=pn;
    if(n==5){
      f->timeout=args[0];
      f->port=args[1];
      f->threadAffinity=args[2];
      f->threadPriority=args[3];
      f->debug=args[4];
    }else{
      printf("Wrong number of figure sensor library arguments - should be 5, was %d\n",n);
      err=1;
    }
  }
  if(err==0){
    f->m=m;
    f->cond=cond;
    f->actsRequired=actsRequired;
    f->nacts=nacts;
    if(*figureframenoSize<1){
      if((*figureframeno=malloc(sizeof(int)))==NULL){
	printf("unable to malloc figureframeno\n");
	err=1;
      }else
	*figureframenoSize=1;
    }
    f->frameno=figureframeno;
    if(pthread_mutex_init(&f->mInternal,NULL)){
      printf("Error init figure internal mutex\n");
      err=1;
    }
    //DMA array has to be a whole number of int32 for the sl240, ie multiple of 4 bytes.
    f->arrsize=HDRSIZE+nacts*sizeof(unsigned short);
    if((f->arr=malloc(f->arrsize))==NULL){
      printf("couldn't malloc arr\n");
      err=1;
    }else{
      memset(f->arr,0,f->arrsize);
      f->acts=(unsigned short*)&(f->arr[HDRSIZE]);
    }
    if((err=openSocket(f))!=0){
      printf("Unable to open listening socket\n");
    }else{
      f->open=1;
    }
  }
  if(err==0){
    printf("Initialising DAC card\n");
    //initialise acquisition session
    if(InitSingleAO(f)){//failed...
      printf("Failed to initSingleAO\n");
      err=1;
    }else{
      // set configuration - _PdAOutReset is called inside _PdAOutSetCfg
      int aoCfg = 0;
      errorChk(_PdAOutSetCfg(f->mirhandle, aoCfg, 0));
      f->state = configured;
      //Start SW trigger
      errorChk(_PdAOutSwStartTrig(f->mirhandle));
      f->state = running;
    }

  }
  if(err==0)
    err=figureNewParam(*figureHandle,pbuf,frameno,arr);
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
    if(f->paramNames!=NULL)
      free(f->paramNames);
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
int figureNewParam(void *figureHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr){
  figureStruct *f=(figureStruct*)figureHandle;
  unsigned short *actInit;
  int initLen;
  int *actMapping;
  int *actSource;
  float *actScale,*actOffset;
  int actMappingLen=0,actSourceLen=0,actScaleLen=0,actOffsetLen=0;
  int *index=f->index;
  void **values=f->values;
  char *dtype=f->dtype;
  int *nbytes=f->nbytes;
  int nfound;
  int err=0;
  if(figureHandle!=NULL){
    actMapping=NULL;
    actSource=NULL;
    actScale=NULL;
    actOffset=NULL;
    actInit=NULL;
    initLen=0;
    nfound=bufferGetIndex(pbuf,NBUFFERVARIABLES,f->paramNames,index,values,dtype,nbytes);
    if(index[ACTMAPPING]>=0){
      if(nbytes[ACTMAPPING]==0){
	//nowt
      }else if(nbytes[ACTMAPPING]%sizeof(int)==0 && dtype[ACTMAPPING]=='i'){
	actMapping=(int*)values[ACTMAPPING];
	actMappingLen=nbytes[ACTMAPPING]/sizeof(int);
      }else{
	printf("Warning - bad actuator mapping (wrong size or type\n");
	err=1;
      }
    }else
      printf("actMapping for figure sensor library not found - continuing with linear mapping\n");

    if(index[ACTINIT]>=0){
      if(nbytes[ACTINIT]>0){
	if(dtype[ACTINIT]=='H' && nbytes[ACTINIT]%sizeof(unsigned short)==0){
	  actInit=(unsigned short*)values[ACTINIT];
	  initLen=nbytes[ACTINIT]/sizeof(unsigned short);
	}else{
	  printf("Warning - bad actInit values (wrong size or type)\n");
	  err=1;
	}
      }
    }else
      printf("actInit for figure sensor library not found - continuing\n");

    if(index[ACTSOURCE]>=0){
      if(nbytes[ACTSOURCE]==0){
      }else if(nbytes[ACTSOURCE]%sizeof(int)==0 && dtype[ACTSOURCE]=='i'){
	actSource=(int*)values[ACTSOURCE];
	actSourceLen=nbytes[ACTSOURCE]/sizeof(int);
      }else{
	printf("Warning - bad figureActSource (wrong size or type)\n");
	err=1;
      }
    }else
      printf("figureActSource for figure sensor library not found - continuing\n");

    if(index[ACTSCALE]>=0){
      if(nbytes[ACTSCALE]==0){
      }else if(nbytes[ACTSCALE]%sizeof(float)==0 && dtype[ACTSCALE]=='f'){
	actScale=(float*)values[ACTSCALE];
	actScaleLen=nbytes[ACTSCALE]/sizeof(float);
      }else{
	printf("Warning - bad figureActScale (wrong size or type)\n");
	err=1;
      }
    }else
      printf("figureActScale for figure sensor library not found - continuing\n");

    if(index[ACTOFFSET]>=0){
      if(nbytes[ACTOFFSET]==0){
      }else if(nbytes[ACTOFFSET]%sizeof(float)==0 && dtype[ACTOFFSET]=='f'){
	actOffset=(float*)values[ACTOFFSET];
	actOffsetLen=nbytes[ACTOFFSET]/sizeof(float);
      }else{
	printf("Warning - bad figureActOffset (wrong size or type)\n");
	err=1;
      }
    }else
      printf("figureActOffset for figure sensor library not found - continuing\n");

    if(index[FIGUREDEBUG]>=0){
      if(nbytes[FIGUREDEBUG]==sizeof(int) && dtype[FIGUREDEBUG]=='i'){
	f->debug=*((int*)values[FIGUREDEBUG]);
      }else{
	printf("Warning - figureDebug bad\n");
      }
    }else
      printf("figureDebug for figure sensor library not found - continuing\n");
    if(actSourceLen!=actMappingLen && actSourceLen!=0){
      printf("figureActSource wrong size\n");
      actSource=NULL;
      err=1;
    }
    if(actScaleLen!=actMappingLen && actScaleLen!=0){
      printf("figureActScale wrong size\n");
      actScale=NULL;
      err=1;
    }
    if(actOffsetLen!=actMappingLen && actOffsetLen!=0){
      printf("figureActOffset wrong size\n");
      actOffset=NULL;
      err=1;
    }
    if(actSource==NULL && actMappingLen!=f->nacts){
      printf("warning: actSource not defined, and actMapping not equal to nacts - ignoring actMapping\n");
      actMappingLen=0;
      actMapping=NULL;
    }

    pthread_mutex_lock(&f->mInternal);
    f->actMapping=actMapping;
    f->actMappingLen=actMappingLen;
    f->actInit=actInit;
    f->initLen=initLen;
    f->actSource=actSource;
    f->actScale=actScale;
    f->actOffset=actOffset;
    pthread_mutex_unlock(&f->mInternal);
  }

  return err;
}
