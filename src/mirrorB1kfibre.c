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

One or more Boston 1k DMs with fibre interface.

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
#ifndef NOBMM
#include "bmc_mdlib.h"//include the BMM library
#else
typedef int tE;
typedef unsigned int			tU32;	/* generic count/data (32b) */
typedef unsigned short			tU16;	/* generic data (16b) */
typedef unsigned char			tU8;	/* generic data (8b) */
typedef struct tBMClibStruct	*tBMC;	/* anonymous token for access */
typedef struct tBMClibStruct {
	} tBMClibStruct;

typedef struct {
	} tBMCHVAspec, *tBMCHVAsp;

typedef enum {
	kBMCEnoErr		= 0,				/* no error */
	} tBMCerrorEnum;
char *BMCgetErrStr(tE err){
  return NULL;
}
#endif

#include "darc.h"
#include "agbcblas.h"

typedef enum{
  MIRRORACTCONTROLMX,
  MIRRORACTMAPPING,
  MIRRORACTMAX,
  MIRRORACTMIN,
  MIRRORACTNEW,
  MIRRORACTOFFSET,
  MIRRORACTSCALE,
  MIRRORACTSOURCE,
  ACTUATORS,
  MIRRORNACTS,
  RECORDTIMESTAMP,
  //Add more before this line.
  MIRRORNBUFFERVARIABLES//equal to number of entries in the enum
}MIRRORBUFFERVARIABLEINDX;

#define makeParamNames() bufferMakeNames(MIRRORNBUFFERVARIABLES,\
					 "actControlMx","actMapping","actMax","actMin","actNew","actOffset","actScale","actSource", "actuators","nacts","recordTimestamp")





typedef struct{
  int nacts;
  unsigned short *arr;
  unsigned int arrsize;
  int open;
  int err;
  pthread_t threadid;
  pthread_cond_t cond;
  pthread_mutex_t m;
  tBMC *bmmhandle;
  tBMCHVAsp bmmInfo;   // BMM structure containing HVA data;
  unsigned int *threadAffinity;
  int threadAffinElSize;
  int threadPriority;
  int nactsBMM;
  circBuf *rtcActuatorBuf;
  circBuf *rtcErrorBuf;
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
  int nbmm;
  int *nactArr;
  float *actuators;
  char *paramNames;
  int index[MIRRORNBUFFERVARIABLES];
  void *values[MIRRORNBUFFERVARIABLES];
  char dtype[MIRRORNBUFFERVARIABLES];
  int nbytes[MIRRORNBUFFERVARIABLES];
  unsigned int *mirrorframeno;
  int recordTime;
}MirrorStruct;


int openBMM(MirrorStruct *mirstr,int n){
  tE rt;
  int orientation=0;//defined in bmc_mdlib.h
#ifndef NOBMM
#define HVAType kBMCT_KILO1024LL
  if((rt=BMCopen(n,&mirstr->bmmhandle[n]))!=kBMCEnoErr){
    printf("Failed to open BMM DM: %s\n",BMCgetErrStr(rt));
    return 1;
  }

  if((rt=BMCsetUpHVA(mirstr->bmmhandle[n],(tBMC_HVAType_enum)HVAType))!=kBMCEnoErr){
    printf("Failed to set HVA type: %s\n",BMCgetErrStr(rt));
    BMCclose(mirstr->bmmhandle[n]);
    mirstr->bmmhandle[n]=0;
    return 1;
  }
  if((rt=BMCretrieveHVAinfo(mirstr->bmmhandle[n],(tBMC_HVAType_enum)HVAType,&mirstr->bmmInfo))!=kBMCEnoErr){
    printf("Failed to get HVA info: %s\n",BMCgetErrStr(rt));
    BMCclose(mirstr->bmmhandle[n]);
    mirstr->bmmhandle[n]=0;
    return 1;
  }
  if((rt=BMCclearHVA(mirstr->bmmhandle[n]))!=kBMCEnoErr){
    printf("Failed to clear all channels: %s\n",BMCgetErrStr(rt));
    BMCclose(mirstr->bmmhandle[n]);
    mirstr->bmmhandle[n]=0;
    return 1;
  }
  if((rt=BMCsetDMorientation(mirstr->bmmhandle[n],(tBMC_HVAType_enum)HVAType,(tBMC_orientation_enum)orientation))!=kBMCEnoErr){
    printf("Failed to set orientation: %s\n",BMCgetErrStr(rt));
    BMCclose(mirstr->bmmhandle[n]);
    mirstr->bmmhandle[n]=0;
    return 1;
  }

#endif
  return 0;
}


/**
   Free mirror/memory/sl240
*/
void mirrordofree(MirrorStruct *mirstr){
  int i;
  if(mirstr!=NULL){
    if(mirstr->arr!=NULL)
      free(mirstr->arr);

#ifndef NOBMM
    for(i=0;i<mirstr->nbmm;i++){
      if(mirstr->bmmhandle[i]!=0)
	BMCclose(mirstr->bmmhandle[i]);
      mirstr->bmmhandle[i]=0;
    }
#endif
    free(mirstr->nactArr);
    free(mirstr->actsNew);
    free(mirstr->bmmhandle);
    pthread_cond_destroy(&mirstr->cond);
    pthread_mutex_destroy(&mirstr->m);
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

#ifdef NOBMM
int BMCburstHVA(tBMC bmc,tU32 count, tU16 *data){
  return 0;
}
#endif


void *workerBMM(void *mirstrv){
  MirrorStruct *mirstr=(MirrorStruct*)mirstrv;
  int i;//,j;
  int offset;
  struct timeval t1;//,t2;
  tE rt;
  mirrorsetThreadAffinity(mirstr->threadAffinity,mirstr->threadPriority,mirstr->threadAffinElSize);
  pthread_mutex_lock(&mirstr->m);
  while(mirstr->open){
    pthread_cond_wait(&mirstr->cond,&mirstr->m);//wait for actuators.
    if(mirstr->open){
      mirstr->err=0;//thread safety warning...!
      offset=0;
      if(mirstr->recordTime){
	gettimeofday(&t1,NULL);
	mirstr->mirrorframeno[0]=t1.tv_sec*1000000+t1.tv_usec;//gives some indicat
      }else
	mirstr->mirrorframeno[0]++;
      if(mirstr->actMapping==NULL){
	//and now for the alpao...
	for(i=0;i<mirstr->nbmm;i++){
	  if((rt=BMCburstHVA(mirstr->bmmhandle[i],mirstr->nactArr[i],&mirstr->arr[offset]))!=kBMCEnoErr){
	    printf("Error sending to bmm: %s\n",BMCgetErrStr(rt));
	  }
	  offset+=mirstr->nactArr[i];
	}
      }else{
	for(i=0;i<mirstr->nbmm;i++){
	  if((rt=BMCburstHVA(mirstr->bmmhandle[i],mirstr->nactArr[i],&mirstr->arr[offset]))!=kBMCEnoErr){
	    printf("Error sending to bmm : %s\n",BMCgetErrStr(rt));

	  }
	  offset+=mirstr->nactArr[i];
	}
      }
    }
  }
  pthread_mutex_unlock(&mirstr->m);
  
  return NULL;
}

/**
Open the DMs
*/

int mirrorOpen(char *name,int narg,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **mirrorHandle,int nacts,circBuf *rtcActuatorBuf,unsigned int frameno,unsigned int **mirrorframeno,int *mirrorframenoSize){
  int err;
  MirrorStruct *mirstr;
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
  if(*mirrorframenoSize<1){
    if((*mirrorframeno=malloc(sizeof(unsigned int)*1))==NULL){
      printf("Unable to alloc mirrorframeno\n");
      mirrordofree(mirstr);
      *mirrorHandle=NULL;
    }
    *mirrorframenoSize=1;
  }
  mirstr->mirrorframeno=*mirrorframeno;
  mirstr->mirrorframeno[0]=0;//for the BMMs
  if(narg>1 && narg>args[0]+3){
    mirstr->threadAffinElSize=args[0];
    mirstr->threadPriority=args[1];
    mirstr->threadAffinity=(unsigned int*)&args[2];
    mirstr->nbmm=args[2+args[0]];//number of BMM DMs
    if(narg>=3+args[0]+mirstr->nbmm){
      //alloc some memory to put it all in...
      if((mirstr->nactArr=calloc(sizeof(int),mirstr->nbmm))==NULL){
	printf("Error allocing nactArr\n");
	mirrordofree(mirstr);
	*mirrorHandle=NULL;
	return 1;
      }
      memcpy(mirstr->nactArr,&args[3+args[0]],sizeof(int)*(mirstr->nbmm));
    }else{
      printf("Error: wrong number of args - should be Naffin, prio, affin[Naffin], nbmm, nact[0],nact[1],...\n");
      mirrordofree(mirstr);
      *mirrorHandle=NULL;
      return 1;
    }
  }else{
      printf("Error: wrong number of args - should be Naffin, prio, affin[Naffin], nbmm, nact[0],nact[1],...\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  if((mirstr->bmmhandle=calloc(sizeof(tBMC),mirstr->nbmm))==NULL){
    printf("Error allocing board arrays\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  for(i=0;i<mirstr->nbmm;i++){
    mirstr->nactsBMM+=mirstr->nactArr[i];
    printf("DM %d: %d\n",i,mirstr->nactArr[i]);
  }

  mirstr->arrsize=(mirstr->nactsBMM)*sizeof(unsigned short);
  if((mirstr->arr=malloc(mirstr->arrsize))==NULL){
    printf("couldn't malloc arr\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  memset(mirstr->arr,0,mirstr->arrsize);
  

  
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
#ifndef NOBMM
  for(i=0;i<mirstr->nbmm;i++){
    if((err=openBMM(mirstr,i))!=0){
      printf("Error opening BMM[%d]\n",i);
      mirrordofree(mirstr);
      *mirrorHandle=NULL;
      return 1;
    }
  }
#endif
  mirstr->open=1;
  if((err=mirrorNewParam(*mirrorHandle,pbuf,frameno,arr))){
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  pthread_create(&mirstr->threadid,NULL,workerBMM,mirstr);
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
  int nactsBMM=mirstr->nactsBMM;
  //MirrorStructBuffered *msb;
  if(err==0 && mirstr!=NULL && mirstr->open==1){
    //printf("Sending %d values to mirror\n",n);
    pthread_mutex_lock(&mirstr->m);
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
      nactsBMM=nacts;
    }
    
    if(mirstr->actMapping==NULL){
      if(mirstr->actOffset==NULL){
	for(i=0; i<nactsBMM; i++){
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
      }else{//actOffset specified
	for(i=0; i<nactsBMM; i++){
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
    }else{//actMapping is specified...
      if(mirstr->actSource==NULL){
	if(mirstr->actScale==NULL){
	  if(mirstr->actOffset==NULL){
	    for(i=0; i<nactsBMM; i++){
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
	    for(i=0; i<nactsBMM; i++){
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
	    for(i=0; i<nactsBMM; i++){
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
	    for(i=0; i<nactsBMM; i++){
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
	    for(i=0; i<nactsBMM; i++){
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
	    for(i=0; i<nactsBMM; i++){
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
	    for(i=0; i<nactsBMM; i++){
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
	    for(i=0; i<nactsBMM; i++){
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
    if(writeCirc){
      circAddSizeForce(mirstr->rtcActuatorBuf,mirstr->arr,sizeof(unsigned short)*mirstr->nactsBMM,0,timestamp,frameno);
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
	if(j==RECORDTIMESTAMP){
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
    if(nactsNew!=mirstr->nactsBMM){
      printf("Error: number of actuators per board summed not equal to expected number of actuators (%d != %d)\n",mirstr->nactsBMM,nactsNew);
      err=1;
    }
    //dim=mirstr->actMapping==NULL?mirstr->nacts:mirstr->actMappingLen;



    if(mirstr->rtcActuatorBuf!=NULL && mirstr->rtcActuatorBuf->datasize!=mirstr->nactsBMM*sizeof(unsigned short)){
      if(circReshape(mirstr->rtcActuatorBuf,1,&mirstr->nactsBMM,'H')!=0){
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
    if(dtype[ACTUATORS]=='f' && nbytes[ACTUATORS]==sizeof(float)*mirstr->nacts){
      mirstr->actuators=(float*)values[ACTUATORS];
    }else{
      mirstr->actuators=NULL;
    }
    mirstr->recordTime=0;
    if(indx[RECORDTIMESTAMP]>=0){
      if(dtype[RECORDTIMESTAMP]=='i' && nbytes[RECORDTIMESTAMP]==sizeof(int)){
	mirstr->recordTime=*(int*)values[RECORDTIMESTAMP];
      }else{
	writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"mirrorRecordTime error");
	printf("mirrorRecordTime error\n");
      }
    }

    pthread_mutex_unlock(&mirstr->m);
  }
  
  //mirstr->swap=1;
  return err;
}





