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

This library does 1 or more PdAO32 cards, and then 1 or more alpao cards.

NOTE: Need to edit alpao header file, asdkType.h
because this clashes with the PDAO32 card libraries.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include "rtcmirror.h"
#include <time.h>
#include <sys/time.h>
#include <pthread.h>
#ifndef NODM
#include "powerdaq.h"
#include "powerdaq32.h"
#ifndef NOALPAO
#include "asdkWrapper.h"//alpao library wrapper.
#else
typedef int asdkDM;
#endif
#else
typedef int asdkDM;
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
  MIRRORACTSCALE,
  MIRRORACTSOURCE,
  ACTUATORS,
  MIRRORCREEPABSTATS,
  MIRRORCREEPMEAN,
  MIRRORCREEPMODE,
  MIRRORCREEPTIME,
  MIRRORNACTS,

  //Add more before this line.
  MIRRORNBUFFERVARIABLES//equal to number of entries in the enum
}MIRRORBUFFERVARIABLEINDX;

#define makeParamNames() bufferMakeNames(MIRRORNBUFFERVARIABLES,\
					 "actControlMx","actInit","actMapping","actMax","actMin","actNew","actOffset","actScale","actSource", "actuators","creepAbstats","creepMean","creepMode","creepTime","nacts")

/*
Creep:

Need to run 
leakyaverage rtcActuatorBuf -smain -g0.00001 -t0.01
on darc.
This creates a new stream containing the averaged DM shape.

This stream should be added to the logger - so that if leakyaverage crashes, the mean shape will have been logged.


This particular script can be run at intervals to update the setpoint for the creep compensation.

It will set the meanDmShape array in darc, and this is then used for compensation.  


A = new static aberrations at t=0.   From actuators or refCentroids+rmx.

B = Known mean DM shape (from leakyaverage)

Creep compensation:
(B-A)*f(t) + A

f(t) obtained from experiment (Urban's values).

If the DM has been powered off, the mean powered off shape can be obtained using the ALPAO SDK: -dm.offsets() (this never changes - i.e. 241 hardcoded values).

So, the darc module requires:

B (array of 241 values)
A or None (in which case, actuators are used)
t (time at which A is valid from).
f (values and time, for interpolation) - actually, the function.
mode (if 0, don't apply, if 1, update every iteration, if 2, apply now and change to 3)

mode can then be set to 2 every few minutes or something.  e.g. when the loop is opened, etc.

Email from UB on 23rd Jan 2015:
f(t) : numpy.load("run18_correction_factor.pck")
   These points are for every 6 minutes, starting at 0.0 (obviously).


Shape when DM powered off:
numpy.load("metaData_000.pck")['dmValues']

f(t) can be well approximated using:

f(t) = (1-1/(t*0.8+1))*0.215

where t=numpy.arange(60) in 6 minute intervals...

i.e. for t in seconds, use:

f(t) = (1-1/(t/450+1))*0.215

Assume that A is already applied, and therefore, need an additional (B-A)*f(t) to be added.


*/





typedef struct{
  int nacts;
  unsigned short *arr;
  double *darr;
  uint32 darrsize;
  uint32 arrsize;
  int open;
  int err;
  pthread_t threadid[2];
  pthread_cond_t cond;
  pthread_cond_t cond2;
  pthread_mutex_t m;
  pthread_mutex_t m2;
  int *handle;
  asdkDM **alpaohandle;
  unsigned int *threadAffinity;
  int threadAffinElSize;
  int threadPriority;
  tState *state;//state of the acquisition session.
  //int board;
  int *adapterType;
  int *alpaoSerialNo;
  int nactsBoard;
  int nactsAlpao;
  //MirrorStructBuffered msb[2];
  //int buf;
  //int swap;
  //int bufindx[MIRRORNBUFFERVARIABLES];
  circBuf *rtcActuatorBuf;
  circBuf *rtcErrorBuf;
  unsigned short *actInit;
  int *nactInit;
  int initLen;
  float *actMin;
  float *actMax;
  int *actMapping;
  int actMappingSize;
  int actMappingLen;
  int *actSource;
  float *actScale;
  float *actOffset;
  int nactsNew;
  float *actsNew;
  int actsNewSize;
  int *actControlMx;
  int nboards;
  int nalpao;
  int *boardNumber;
  int *nactBoard;
  int totactBoard;
  float *actuators;
  float *creepMean;//the mean DM shape over the past few hours (from leakyaverage.c)
  float *creepAbstats;//current requested mean DM shape - if NULL, taken from actuators.  Could also be refCentroids.rmx.
  int creepMode;//0 == off, 1==apply every iter, 2==apply now (and set to 3).
  double creepTime;//time at which the creepMean was valid.
  char *paramNames;
  int index[MIRRORNBUFFERVARIABLES];
  void *values[MIRRORNBUFFERVARIABLES];
  char dtype[MIRRORNBUFFERVARIABLES];
  int nbytes[MIRRORNBUFFERVARIABLES];
  unsigned int *mirrorframeno;
}MirrorStruct;


#ifndef NODM
int InitSingleAO(MirrorStruct *pAoData,int board){
   Adapter_Info adaptInfo;
   // get adapter type
   errorChk(_PdGetAdapterInfo(pAoData->boardNumber[board], &adaptInfo));
   pAoData->adapterType[board] = adaptInfo.atType;
   if(pAoData->adapterType[board] & atMF)
     printf("This is an MFx board\n");
   else
     printf("This is an AO32 board %20s, S/N %20s\n",adaptInfo.lpBoardName,adaptInfo.lpSerialNum);
   //Use PdGetNumberAdapters() for number of boards...
   pAoData->handle[board] = PdAcquireSubsystem(pAoData->boardNumber[board], AnalogOut, 1);
   if(pAoData->handle[board] < 0){
     printf("SingleAO: PdAcquireSubsystem failed for board %d\n",board);
     pAoData->state[board]=closed;
     return 1;
   }
   pAoData->state[board] = unconfigured;
   errorChk(_PdAOutReset(pAoData->handle[board]));
   // need also to call this function if the board is a PD2-AO-xx
   if(pAoData->adapterType[board] & atPD2AO){
      errorChk(_PdAO32Reset(pAoData->handle[board]));
   }
   return 0;
}




void CleanUpSingleAO(MirrorStruct *pAoData,int board){
   if(pAoData->state[board] == running){
      pAoData->state[board] = configured;
   }
   if(pAoData->state[board] == configured){
     errorChk(_PdAOutReset(pAoData->handle[board]));
     // need also to call this function if the board is a PD2-AO-xx
     if(pAoData->adapterType[board] & atPD2AO){
       errorChk(_PdAO32Reset(pAoData->handle[board]));
     }
     pAoData->state[board] = unconfigured;
   }
   if(pAoData->handle[board] > 0 && pAoData->state[board] == unconfigured){
     errorChk(PdAcquireSubsystem(pAoData->handle[board], AnalogOut, 0));
   }
   pAoData->state[board] = closed;
}
#endif

int openAlpao(MirrorStruct *mirstr,int n){
  double nb=mirstr->nactBoard[n+mirstr->nboards];
  printf("Opening %s\n",(char*)&mirstr->alpaoSerialNo[n*2]);
#ifndef NODM
#ifndef NOALPAO
  if((mirstr->alpaohandle[n]=asdkInit((char*)&mirstr->alpaoSerialNo[n*2]))==NULL){
    UInt errorNo;
    char errMsg[80];
    asdkGetLastError(&errorNo,errMsg,80);
    printf("%s\n",errMsg);
    return 1;
  }
  if(asdkGet(mirstr->alpaohandle[n],"NbOfActuator",&nb)!=0){
    printf("Error getting number of actuators\n");
    return 1;
  }
#endif
#endif
  if(nb!=mirstr->nactBoard[n+mirstr->nboards]){
    printf("Error: Expecting %g actuators for DM, but nacts for this dm is %d\n",nb,mirstr->nactBoard[n+mirstr->nboards]);
    return 1;
  }
  return 0;
}


/**
   Free mirror/memory/sl240
*/
void mirrordofree(MirrorStruct *mirstr){
  if(mirstr!=NULL){
    if(mirstr->arr!=NULL)
      free(mirstr->arr);
    if(mirstr->darr!=NULL)
      free(mirstr->darr);
    /*for(i=0; i<2; i++){
      if(mirstr->msb[i].actMin!=NULL)free(mirstr->msb[i].actMin);
      if(mirstr->msb[i].actMax!=NULL)free(mirstr->msb[i].actMax);
      if(mirstr->msb[i].actMapping!=NULL)free(mirstr->msb[i].actMapping);
      }*/

#ifndef NODM
    int i;
    for(i=0;i<mirstr->nboards;i++)
      CleanUpSingleAO(mirstr,i);
#ifndef NOALPAO
    for(i=0;i>mirstr->nalpao;i++)
      asdkRelease(mirstr->alpaohandle[i]);
#endif
#endif
    free(mirstr->boardNumber);
    free(mirstr->actsNew);
    free(mirstr->adapterType);
    free(mirstr->state);
    free(mirstr->handle);
    free(mirstr->alpaohandle);
    pthread_cond_destroy(&mirstr->cond);
    pthread_cond_destroy(&mirstr->cond2);
    pthread_mutex_destroy(&mirstr->m);
    pthread_mutex_destroy(&mirstr->m2);
    free(mirstr);
  }
}


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


#ifdef NODM
int _PdAO32Write(int handle,int i,unsigned short val){
  return 0;
}
int asdkSend(asdkDM *dm,double *data){
  return 0;
}
typedef unsigned int UInt;
int asdkGetLastError(UInt *errorNo,char *errMsg,int len){
  return 0;
}
#endif
#ifdef NOALPAO
int asdkSend(asdkDM *dm,double *data){
  return 0;
}
typedef unsigned int UInt;
int asdkGetLastError(UInt *errorNo,char *errMsg,int len){
  return 0;
}
#endif


void* workerAlpao(void *mirstrv){
  MirrorStruct *mirstr=(MirrorStruct*)mirstrv;
  int i;//,j;
  int offset,nactInitTot=0;
  //int nacts;
  mirrorsetThreadAffinity(mirstr->threadAffinity,mirstr->threadPriority,mirstr->threadAffinElSize);
  pthread_mutex_lock(&mirstr->m2);
  if(mirstr->open && mirstr->actInit!=NULL){
    //now initialise the alpao...
    if(mirstr->initLen>nactInitTot){
      printf("WARNING: Not yet implemented - actInit for alpao (there is no reason to have this)\n");
    }
  }
  while(mirstr->open){
    pthread_cond_wait(&mirstr->cond2,&mirstr->m2);//wait for actuators.
    if(mirstr->open){
      mirstr->err=0;
      offset=0;
      if(mirstr->actMapping==NULL){
	//and now for the alpao...
	for(i=0;i<mirstr->nalpao;i++){
	  if(asdkSend(mirstr->alpaohandle[i],&mirstr->darr[offset])!=0){
	    UInt errorNo;
	    char errMsg[80];
	    asdkGetLastError(&errorNo,errMsg,80);
	    printf("%s\n",errMsg);
	    printf("Error: asdkSend function\n");
	  }
	  offset+=mirstr->nactBoard[i+mirstr->nboards];
	}
      }else{
	for(i=0;i<mirstr->nalpao;i++){
	  if(asdkSend(mirstr->alpaohandle[i],&mirstr->darr[offset])!=0){
	    UInt errorNo;
	    char errMsg[80];
	    asdkGetLastError(&errorNo,errMsg,80);
	    printf("%s\n",errMsg);
	    printf("Error: asdkSend function\n");
	  }
	  offset+=mirstr->nactBoard[i+mirstr->nboards];
	}
      }
      mirstr->mirrorframeno[1]++;//gives some indication as to whether we're sending to the dm at the AO frame rate (which won't be the case if asdkSend takes too long to complete).
    }
  }
  pthread_mutex_unlock(&mirstr->m2);
  return NULL;
}



/**
   The thread that does the work - copies actuators, and sends to the DAC
*/
void* worker(void *mirstrv){
  MirrorStruct *mirstr=(MirrorStruct*)mirstrv;
  int i;//,j;
  //int val;
  //struct timespec tme;
  //int skip,step;
  int dmno,offset,nactInitTot=0;
  //int nacts;
  mirrorsetThreadAffinity(mirstr->threadAffinity,mirstr->threadPriority,mirstr->threadAffinElSize);
  pthread_mutex_lock(&mirstr->m);
  if(mirstr->open && mirstr->actInit!=NULL){
    dmno=0;
    offset=0;
    for(i=0;i<mirstr->nboards;i++)
      nactInitTot+=mirstr->nactInit[i];
    printf("Init DM 0 (%d actuators)\n",mirstr->nactInit[dmno]);
    for(i=0; i<mirstr->initLen && i<nactInitTot; i++){
      if(mirstr->nactInit!=NULL && i-offset==mirstr->nactInit[dmno]){
	offset=i;
	dmno++;
	printf("Init DM %d (%d actuators)\n",dmno,mirstr->nactInit[dmno]);
      }
      _PdAO32Write(mirstr->handle[dmno],i-offset,mirstr->actInit[i]);
    }
  }
  
  while(mirstr->open){
    pthread_cond_wait(&mirstr->cond,&mirstr->m);//wait for actuators.
    if(mirstr->open){
      /*if(mirstr->actControlMx!=NULL){
	if(mirstr->actMapping==NULL)
	  nacts=mirstr->nactsNew;
	else
	  nacts=mirstr->actMappingLen;
      }else{
	if(mirstr->actMapping==NULL)
	  nacts=mirstr->nacts;
	else
	  nacts=mirstr->actMappingLen;
	  }*/
	
      
      mirstr->err=0;
      dmno=0;
      offset=0;

      if(mirstr->actMapping==NULL){
	for(i=0; i<mirstr->nactsBoard; i++){
	  if(i-offset==mirstr->nactBoard[dmno]){
	    offset=i;
	    dmno++;
	  }
	  mirstr->err|=_PdAO32Write(mirstr->handle[dmno],i-offset,mirstr->arr[i]);
	}
	//and now for the alpao...
	offset=0;
	for(i=0;i<mirstr->nalpao;i++){
	  if(asdkSend(mirstr->alpaohandle[i],&mirstr->darr[offset])!=0){
	    UInt errorNo;
	    char errMsg[80];
	    asdkGetLastError(&errorNo,errMsg,80);
	    printf("%s\n",errMsg);
	    printf("Error: asdkSend function\n");
	  }
	  offset+=mirstr->nactBoard[i+mirstr->nboards];
	}
      }else{
	for(i=0; i<mirstr->nactsBoard; i++){
	  if(i-offset==mirstr->nactBoard[dmno]){
	    offset=i;
	    dmno++;
	  }
	  mirstr->err|=_PdAO32Write(mirstr->handle[dmno],mirstr->actMapping[i],mirstr->arr[i]);
	}
	//and now for the alpao (no actmapping here)
	offset=0;
	for(i=0;i<mirstr->nalpao;i++){
	  if(asdkSend(mirstr->alpaohandle[i],&mirstr->darr[offset])!=0){
	    UInt errorNo;
	    char errMsg[80];
	    asdkGetLastError(&errorNo,errMsg,80);
	    printf("%s\n",errMsg);
	    printf("Error: asdkSend function\n");
	  }
	  offset+=mirstr->nactBoard[i+mirstr->nboards];
	}
      }
      mirstr->mirrorframeno[0]++;//gives some indication as to whether we're sending to the dm at the AO frame rate (which won't be the case if asdkSend takes too long to complete).
    }
  }
  pthread_mutex_unlock(&mirstr->m);
  return NULL;
}

/**
Open the DMs: PD32AO first, then Alpao.
*/

int mirrorOpen(char *name,int narg,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **mirrorHandle,int nacts,circBuf *rtcActuatorBuf,unsigned int frameno,unsigned int **mirrorframeno,int *mirrorframenoSize){
  int err;
  MirrorStruct *mirstr;
#ifndef NODM
  DWORD aoCfg;
#endif
  char *pn;
  int i;
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
  if(*mirrorframenoSize<sizeof(unsigned int)*2){
    if((*mirrorframeno=malloc(sizeof(unsigned int)*2))==NULL){
      printf("Unable to alloc mirrorframeno\n");
      mirrordofree(mirstr);
      *mirrorHandle=NULL;
    }
    *mirrorframenoSize=sizeof(unsigned int)*2;
  }
  mirstr->mirrorframeno=*mirrorframeno;
  mirstr->mirrorframeno[0]=0;//for the PDAO32s
  mirstr->mirrorframeno[1]=0;//for the ALPAOs
  if(narg>1 && narg>args[0]+4){
    mirstr->threadAffinElSize=args[0];
    mirstr->threadPriority=args[1];
    mirstr->threadAffinity=(unsigned int*)&args[2];
    mirstr->nboards=args[2+args[0]];//number of PD32AO boards
    mirstr->nalpao=args[3+args[0]];//number of ALPAO DMs
    if(narg>=4+args[0]+mirstr->nboards+mirstr->nalpao*2+(mirstr->nboards+mirstr->nalpao)*2){
      //alloc some memory to put it all in...
      if((mirstr->boardNumber=calloc(sizeof(int),mirstr->nboards+2*mirstr->nalpao+(mirstr->nboards+mirstr->nalpao)*2))==NULL){
	printf("Error allocing boardNumber\n");
	mirrordofree(mirstr);
	*mirrorHandle=NULL;
	return 1;
      }
      mirstr->alpaoSerialNo=&mirstr->boardNumber[mirstr->nboards];
      mirstr->nactBoard=&mirstr->boardNumber[mirstr->nboards+mirstr->nalpao*2];
      mirstr->nactInit=&mirstr->boardNumber[mirstr->nboards*2+mirstr->nalpao*3];
      memcpy(mirstr->boardNumber,&args[4+args[0]],sizeof(int)*(mirstr->nboards*3+mirstr->nalpao*4));
      for(i=0;i<mirstr->nalpao;i++){
	((char*)(&mirstr->alpaoSerialNo[i*2]))[7]='\0';//serial numbers should be 6 bytes long - so terminate just in case...
	printf("Using mirror %s\n",(char*)(&mirstr->alpaoSerialNo[i*2]));
      }
    }else{
      printf("Error: wrong number of args - should be Naffin, prio, affin[Naffin], nPD32AOboards,nalpao, boardNumber[board],boardNumber[board],...,serialNo[alpao(8 bytes)],serialNo[alpao(8bytes)], nacts[board/alpao],nacts[board/alpao],...nactInit[board/alpao],nactInit[board/alpao],..., \n(alpao serial numbers are 6 chars long, and here should be packed into 2 ints)");
      mirrordofree(mirstr);
      *mirrorHandle=NULL;
      return 1;
    }
  }else{
    printf("Error: wrong number of args - should be Naffin, prio, affin[Naffin], nPD32AOboards,nalpao, boardNumber[board],boardNumber[board],...,serialNo[alpao(8 bytes)],serialNo[alpao(8bytes)], nacts[board/alpao],nacts[board/alpao],...nactInit[board/alpao],nactInit[board/alpao],..., \n(alpao serial numbers are 6 chars long, and here should be packed into 2 ints)");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  if((mirstr->adapterType=calloc(sizeof(int),mirstr->nboards))==NULL || (mirstr->state=calloc(sizeof(tState),mirstr->nboards))==NULL || (mirstr->handle=calloc(sizeof(int),mirstr->nboards))==NULL || (mirstr->alpaohandle=calloc(sizeof(asdkDM*),mirstr->nalpao))==NULL){
    printf("Error allocing board arrays\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  for(i=0;i<mirstr->nboards;i++){
    mirstr->nactsBoard+=mirstr->nactBoard[i];
    printf("DM %d: %d\n",i,mirstr->nactBoard[i]);
  }
  for(i=mirstr->nboards;i<mirstr->nboards+mirstr->nalpao;i++){
    mirstr->nactsAlpao+=mirstr->nactBoard[i];
    printf("DM %d: %d\n",i,mirstr->nactBoard[i]);
  }
  mirstr->totactBoard=mirstr->nactsBoard+mirstr->nactsAlpao;

  mirstr->arrsize=mirstr->nactsBoard*sizeof(unsigned short);
  if((mirstr->arr=malloc(mirstr->arrsize))==NULL){
    printf("couldn't malloc arr\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  memset(mirstr->arr,0,mirstr->arrsize);
  mirstr->darrsize=mirstr->nactsAlpao*sizeof(double);
  if((mirstr->darr=malloc(mirstr->darrsize))==NULL){
    printf("couldn't malloc darr\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  memset(mirstr->darr,0,mirstr->darrsize);
  

  
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
  if(pthread_cond_init(&mirstr->cond2,NULL)!=0){
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
  if(pthread_mutex_init(&mirstr->m2,NULL)!=0){
    printf("Error initialising mutex variable\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  //initialise acquisition session
#ifndef NODM
  for(i=0;i<mirstr->nboards;i++){
    if(InitSingleAO(mirstr,i)){//failed...
      printf("Failed to initSingleAO\n");
      mirrordofree(mirstr);
      *mirrorHandle=NULL;
      return 1;
    }
    // set configuration - _PdAOutReset is called inside _PdAOutSetCfg
    aoCfg = 0;
    errorChk(_PdAOutSetCfg(mirstr->handle[i], aoCfg, 0));
    mirstr->state[i] = configured;
    //Start SW trigger
    errorChk(_PdAOutSwStartTrig(mirstr->handle[i]));
    mirstr->state[i] = running;
  }
#ifndef NOALPAO
  //and now alloc the alpaos...
  for(i=0;i<mirstr->nalpao;i++){
    if((err=openAlpao(mirstr,i))!=0){
      printf("Error: opening Alpao mirror[%d]\n",i);
      mirrordofree(mirstr);
      *mirrorHandle=NULL;
      return 1;
    }
  }
#endif
#endif
  mirstr->open=1;
  if((err=mirrorNewParam(*mirrorHandle,pbuf,frameno,arr))){
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  pthread_create(&mirstr->threadid[0],NULL,worker,mirstr);
  pthread_create(&mirstr->threadid[1],NULL,workerAlpao,mirstr);
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
    pthread_mutex_lock(&mirstr->m2);
    if(mirstr->paramNames!=NULL)
      free(mirstr->paramNames);
    mirstr->open=0;
    pthread_cond_signal(&mirstr->cond);//wake the thread.
    pthread_cond_signal(&mirstr->cond2);//wake the thread.
    pthread_mutex_unlock(&mirstr->m);
    pthread_mutex_unlock(&mirstr->m2);
    printf("Joining mirror worker thread\n");
    pthread_join(mirstr->threadid[0],NULL);//wait for worker thread to complete
    pthread_join(mirstr->threadid[1],NULL);//wait for worker thread to complete
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
  }
  printf("Mirror closed\n");
  return 0;
}

void applyCreep(MirrorStruct *mirstr,float *data,int nacts){
  //nacts is total number of actuators (including for the PdAO32 card).
  struct timeval t1;
  double tdiff;
  int i;
  float f;
  float *B=mirstr->creepMean;
  float *A=mirstr->creepAbstats;
  gettimeofday(&t1,NULL);
  tdiff=t1.tv_sec+t1.tv_usec*1e-6-mirstr->creepTime;
  if(tdiff<0){
    printf("Warning: Creep time differential < 0: Not applying\n");
    return;
  }
  f=(1-1/(tdiff/450+1))*0.215;
  printf("f: %g %d %g\n",f,mirstr->nactsAlpao,(B[0]-A[0])*f);
  for(i=0;i<mirstr->nactsAlpao;i++){
    data[i+nacts-mirstr->nactsAlpao]+=(B[i]-A[i])*f;
  }
  return;
}

int mirrorSend(void *mirrorHandle,int n,float *data,unsigned int frameno,double timestamp,int err,int writeCirc){
  MirrorStruct *mirstr=(MirrorStruct*)mirrorHandle;
  int nclipped=0;
  int intDMCommand;
  int i,nacts;
  int nactsBoard=mirstr->nactsBoard;
  float val;
  //MirrorStructBuffered *msb;
  if(err==0 && mirstr!=NULL && mirstr->open==1){
    //printf("Sending %d values to mirror\n",n);
    pthread_mutex_lock(&mirstr->m);
    pthread_mutex_lock(&mirstr->m2);
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
    
    if(mirstr->creepMode!=0){
      applyCreep(mirstr,data,nacts);
    }
    if(mirstr->actMapping==NULL){
      if(mirstr->actOffset==NULL){
	for(i=0; i<nactsBoard; i++){
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
	for(i=nactsBoard; i<nacts; i++){
	  val=data[i];
	  if(val<mirstr->actMin[i]){
	    nclipped++;
	    val=mirstr->actMin[i];
	  }
	  if(val>mirstr->actMax[i]){
	    nclipped++;
	    val=mirstr->actMax[i];
	  }
	  mirstr->darr[i-nactsBoard]=(double)val;
	}
      }else{//actOffset specified
	for(i=0; i<nactsBoard; i++){
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
	for(i=nactsBoard; i<nacts; i++){
	  val=data[i]+mirstr->actOffset[i];
	  if(val<mirstr->actMin[i]){
	    nclipped++;
	    val=mirstr->actMin[i];
	  }
	  if(val>mirstr->actMax[i]){
	    nclipped++;
	    val=mirstr->actMax[i];
	  }
	  mirstr->darr[i-nactsBoard]=(double)val;
	}
      }
    }else{//actMapping is specified...
      if(mirstr->actSource==NULL){
	if(mirstr->actScale==NULL){
	  if(mirstr->actOffset==NULL){
	    for(i=0; i<nactsBoard; i++){
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
	    for(i=nactsBoard; i<nacts; i++){
	      val=data[i];
	      if(val<mirstr->actMin[i]){
		nclipped++;
		val=mirstr->actMin[i];
	      }
	      if(val>mirstr->actMax[i]){
		nclipped++;
		val=mirstr->actMax[i];
	      }
	      mirstr->darr[i-nactsBoard]=(double)val;
	    }
	  }else{//actoffset defined.
	    for(i=0; i<nactsBoard; i++){
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
	    for(i=nactsBoard; i<nacts; i++){
	      val=data[i]+mirstr->actOffset[i];
	      if(val<mirstr->actMin[i]){
		nclipped++;
		val=mirstr->actMin[i];
	      }
	      if(val>mirstr->actMax[i]){
		nclipped++;
		val=mirstr->actMax[i];
	      }
	      mirstr->darr[i-nactsBoard]=(double)val;
	    }
	  }
	}else{//actscale defined
	  if(mirstr->actOffset==NULL){
	    for(i=0; i<nactsBoard; i++){
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
	    for(i=nactsBoard; i<nacts; i++){
	      val=(data[i]*mirstr->actScale[i]);
	      if(val<mirstr->actMin[i]){
		nclipped++;
		val=mirstr->actMin[i];
	      }
	      if(val>mirstr->actMax[i]){
		nclipped++;
		val=mirstr->actMax[i];
	      }
	      mirstr->darr[i-nactsBoard]=(double)val;
	    }
	  }else{//actScale and actoffset defined
	    for(i=0; i<nactsBoard; i++){
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
	    for(i=nactsBoard; i<nacts; i++){
	      val=(data[i]*mirstr->actScale[i]+mirstr->actOffset[i]);
	      if(val<mirstr->actMin[i]){
		nclipped++;
		val=mirstr->actMin[i];
	      }
	      if(val>mirstr->actMax[i]){
		nclipped++;
		val=mirstr->actMax[i];
	      }
	      mirstr->darr[i-nactsBoard]=(double)val;
	    }
	  }
	}
      }else{//actSource defined
	if(mirstr->actScale==NULL){
	  if(mirstr->actOffset==NULL){
	    for(i=0; i<nactsBoard; i++){
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
	    for(i=nactsBoard; i<nacts; i++){
	      val=(data[mirstr->actSource[i]]);
	      if(val<mirstr->actMin[i]){
		nclipped++;
		val=mirstr->actMin[i];
	      }
	      if(val>mirstr->actMax[i]){
		nclipped++;
		val=mirstr->actMax[i];
	      }
	      mirstr->darr[i-nactsBoard]=(double)val;
	    }
	  }else{//actSource and actoffset defined.
	    for(i=0; i<nactsBoard; i++){
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
	    for(i=nactsBoard; i<nacts; i++){
	      val=(data[mirstr->actSource[i]]+mirstr->actOffset[i]);
	      if(val<mirstr->actMin[i]){
		nclipped++;
		val=mirstr->actMin[i];
	      }
	      if(val>mirstr->actMax[i]){
		nclipped++;
		val=mirstr->actMax[i];
	      }
	      mirstr->darr[i-nactsBoard]=(double)val;
	    }
	  }
	}else{//actSource and actscale defined
	  if(mirstr->actOffset==NULL){
	    for(i=0; i<nactsBoard; i++){
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
	    for(i=nactsBoard; i<nacts; i++){
	      val=(data[mirstr->actSource[i]]*mirstr->actScale[i]);
	      if(val<mirstr->actMin[i]){
		nclipped++;
		val=mirstr->actMin[i];
	      }
	      if(val>mirstr->actMax[i]){
		nclipped++;
		val=mirstr->actMax[i];
	      }
	      mirstr->darr[i-nactsBoard]=(double)val;
	    }
	  }else{//actSource and actScale and actoffset defined
	    for(i=0; i<nactsBoard; i++){
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
	    for(i=nactsBoard; i<nacts; i++){
	      val=(data[mirstr->actSource[i]]*mirstr->actScale[i]+mirstr->actOffset[i]);
	      if(val<mirstr->actMin[i]){
		nclipped++;
		val=mirstr->actMin[i];
	      }
	      if(val>mirstr->actMax[i]){
		nclipped++;
		val=mirstr->actMax[i];
	      }
	      mirstr->darr[i-nactsBoard]=(double)val;
	    }
	  }
	}
      }
    }
    //memcpy(mirstr->arr,data,sizeof(unsigned short)*mirstr->nacts);
    //Wake up the thread.
    pthread_cond_signal(&mirstr->cond);
    pthread_cond_signal(&mirstr->cond2);
    pthread_mutex_unlock(&mirstr->m);
    pthread_mutex_unlock(&mirstr->m2);
    if(writeCirc){
      circInsert(mirstr->rtcActuatorBuf,mirstr->darr,sizeof(double)*mirstr->nactsAlpao,sizeof(unsigned short)*mirstr->nactsBoard);
      circAddSizeForce(mirstr->rtcActuatorBuf,mirstr->arr,sizeof(unsigned short)*mirstr->nactsBoard,0,timestamp,frameno);
    }
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
	if(j==MIRRORCREEPABSTATS || j==MIRRORCREEPMEAN || j==MIRRORCREEPMODE || j==MIRRORCREEPTIME){
	  //ok
	}else{
	  writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"Error in mirror parameter buffer: %16s",&mirstr->paramNames[j*16]);
	  err=-1;
	}
      }
    }
  }
  if(err==0){
    pthread_mutex_lock(&mirstr->m);
    pthread_mutex_lock(&mirstr->m2);
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
    if(nactsNew!=mirstr->totactBoard){
      printf("Error: number of actuators per board summed not equal to expected number of actuators (%d != %d)\n",mirstr->totactBoard,nactsNew);
      err=1;
    }
    //dim=mirstr->actMapping==NULL?mirstr->nacts:mirstr->actMappingLen;



    if(mirstr->rtcActuatorBuf!=NULL && mirstr->rtcActuatorBuf->datasize!=mirstr->nactsBoard*sizeof(unsigned short)+mirstr->nactsAlpao*sizeof(double)){
      int tmp=(mirstr->nactsBoard*sizeof(unsigned short)+mirstr->nactsAlpao*sizeof(double))/sizeof(unsigned short);
      if(circReshape(mirstr->rtcActuatorBuf,1,&tmp,'H')!=0){
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
    if(dtype[MIRRORACTMIN]=='f' && nbytes[MIRRORACTMIN]==sizeof(float)*nactsNew){
      mirstr->actMin=(float*)values[MIRRORACTMIN];
    }else{
      printf("mirrorActMin error\n");
      writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"mirrorActMin error");
      err=1;
    }
    if(dtype[MIRRORACTMAX]=='f' && nbytes[MIRRORACTMAX]==sizeof(float)*nactsNew){
      mirstr->actMax=(float*)values[MIRRORACTMAX];
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
    if(dtype[ACTUATORS]=='f' && nbytes[ACTUATORS]==sizeof(float)*mirstr->nacts){
      mirstr->actuators=(float*)values[ACTUATORS];
    }else{
      mirstr->actuators=NULL;
    }
    if(indx[MIRRORCREEPABSTATS]>=0){
      if(dtype[MIRRORCREEPABSTATS]=='f' && nbytes[MIRRORCREEPABSTATS]==sizeof(float)*mirstr->nactsAlpao){
	mirstr->creepAbstats=(float*)values[MIRRORCREEPABSTATS];
      }else if(nbytes[MIRRORCREEPABSTATS]==0){
	if(mirstr->actuators!=NULL){
	  mirstr->creepAbstats=&mirstr->actuators[mirstr->nacts-mirstr->nactsAlpao];
	}else{
	  mirstr->creepAbstats=NULL;
	}
      }else{
	mirstr->creepAbstats=NULL;
	writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"creepAbstats error");
	printf("creepAbstats error\n");
	err=1;
      }
    }else{
      mirstr->creepAbstats=NULL;
    }
    if(indx[MIRRORCREEPMEAN]>=0){
      if(dtype[MIRRORCREEPMEAN]=='f' && nbytes[MIRRORCREEPMEAN]==sizeof(float)*mirstr->nactsAlpao){
	mirstr->creepMean=(float*)values[MIRRORCREEPMEAN];
      }else if(nbytes[MIRRORCREEPMEAN]==0){
	mirstr->creepMean=NULL;
      }else{
	mirstr->creepMean=NULL;
	writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"creepMean error");
	printf("creepMean error\n");
	err=1;
      }
    }else{
      mirstr->creepMean=NULL;
    }
    if(indx[MIRRORCREEPMODE]>=0){
      if(dtype[MIRRORCREEPMODE]=='i' && nbytes[MIRRORCREEPMODE]==sizeof(int)){
	mirstr->creepMode=*(int*)values[MIRRORCREEPMODE];
	if(mirstr->creepMode==2)
	  *(int*)values[MIRRORCREEPMODE]=3;
	if(((mirstr->creepAbstats==NULL) || (mirstr->creepMean==NULL)) && (mirstr->creepMode!=0)){
	  mirstr->creepMode=0;
	  *(int*)values[MIRRORCREEPMODE]=0;
	  printf("Warning - setting creepMode to 0\n");
	}
      }else{
	mirstr->creepMode=0;
	writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"creepMode error");
	printf("creepMode error\n");
	err=1;
      }
    }else{
      mirstr->creepMode=0;
    }
    if(indx[MIRRORCREEPTIME]>=0){
      if(dtype[MIRRORCREEPTIME]=='d' && nbytes[MIRRORCREEPTIME]==sizeof(double)){
	mirstr->creepTime=*(double*)values[MIRRORCREEPTIME];
      }else{
	mirstr->creepTime=0;
	writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"creepTime error");
	printf("creepTime error (expect d, got %c)\n",dtype[MIRRORCREEPTIME]);
	err=1;
      }
    }else{
      mirstr->creepTime=0;
    }


    pthread_mutex_unlock(&mirstr->m);
    pthread_mutex_unlock(&mirstr->m2);
  }
  
  //mirstr->swap=1;
  return err;
}





