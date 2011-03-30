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

This version copies the actuator demands to shared memory, which has been created by the receiver, and then writes to an mqueue which has also been created by the receiver.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <mqueue.h>
#include <sys/mman.h>
#include "rtcmirror.h"
#include <time.h>
#include <pthread.h>
#include "darc.h"
#define HDRSIZE 8 //the size of a WPU header - 4 bytes for frame no, 4 bytes for something else.


typedef enum{
  MIRRORACTMAX,
  MIRRORACTMIN,
  //MIRRORLASTACTS,
  MIRRORACTOFFSET,
  MIRRORACTSCALE,
  MIRRORNACTS,
  //Add more before this line.
  MIRRORNBUFFERVARIABLES//equal to number of entries in the enum
}MIRRORBUFFERVARIABLEINDX;

#define makeParamNames() bufferMakeNames(MIRRORNBUFFERVARIABLES,	\
    "actMax",								\
    "actMin",								\
    "actOffset",							\
    "actScale",								\
    "nacts"								\
					 )
//char *MIRRORPARAM[]={"nacts","actMin","actMax","actScale","actOffset"};//,"lastActs"};


typedef struct{
  unsigned short *actMin;
  unsigned short *actMax;
  int actMinSize;
  int actMaxSize;
  int actOffsetSize;
  int actScaleSize;
  float *actOffset;
  float *actOffsetArr;
  float *actScale;
  float *actScaleArr;
  //unsigned short *lastActs;
}MirrorStructBuffered;

typedef struct{
  char *paramNames;
  int nacts;
  unsigned short *arr;
  //int frameno;
  int arrsize;
  int open;
  int err;
  arrayStruct *arrStr;
  //pthread_t threadid;
  //pthread_cond_t cond;
  pthread_mutex_t m;
  int timeout;//in ms
  //int fibrePort;//the port number on sl240 card.
  //int threadAffinity;
  //int threadPriority;
  MirrorStructBuffered msb[2];
  int buf;
  int swap;
  //int bufindx[MIRRORNBUFFERVARIABLES];
  circBuf *rtcActuatorBuf;
  circBuf *rtcErrorBuf;
  //char *host;
  //unsigned short port;
  int index[MIRRORNBUFFERVARIABLES];
  void *values[MIRRORNBUFFERVARIABLES];
  char dtype[MIRRORNBUFFERVARIABLES];
  int nbytes[MIRRORNBUFFERVARIABLES];
  char *prefix;
  int sendPrefix;
  int asfloat;
  mqd_t mq;
  char *shmbuf;
}MirrorStruct;

/**
   Free mirror/memory/sl240
*/
void mirrordofree(MirrorStruct *mirstr){
  int i;
  if(mirstr!=NULL){
    //if(mirstr->arr!=NULL)
    // free(mirstr->arr);
    for(i=0; i<2; i++){
      if(mirstr->msb[i].actMin!=NULL)free(mirstr->msb[i].actMin);
      if(mirstr->msb[i].actMax!=NULL)free(mirstr->msb[i].actMax);
    }
    //pthread_cond_destroy(&mirstr->cond);
    pthread_mutex_destroy(&mirstr->m);
    if(mirstr->mq>0)
      mq_close(mirstr->mq);
    mirstr->mq=0;
    if(mirstr->shmbuf!=NULL)
      munmap(mirstr->shmbuf,sizeof(pthread_mutex_t)+HDRSIZE+sizeof(float)*mirstr->nacts);
    mirstr->shmbuf=NULL;
    free(mirstr);
  }
}

/*int setThreadAffinityForDMC(int threadAffinity,int threadPriority){
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
  }*/

/**
   The thread that does the work - copies actuators, and sends via SL240
*/
/*void* mirrorworker(void *mirstrv){
  MirrorStruct *mirstr=(MirrorStruct*)mirstrv;
  int n,totsent=0,err=0;
  setThreadAffinityForDMC(mirstr->threadAffinity,mirstr->threadPriority);
  pthread_mutex_lock(&mirstr->m);
  while(mirstr->open){
    pthread_cond_wait(&mirstr->cond,&mirstr->m);//wait for actuators.
    if(mirstr->open){
      //Now send the data...
      totsent=0;
      while(err==0 && totsent<mirstr->arrsize){
	n=send(mirstr->socket,&mirstr->arr[totsent],mirstr->arrsize-totsent,0);
	if(n<0){//error
	  err=-1;
	  printf("Error sending data\n");
	}else{
	  totsent+=n;
	}
      }
    }
  }
  pthread_mutex_unlock(&mirstr->m);
  return NULL;
  }*/

int openMirrorSHM(MirrorStruct *mirstr){
  int err=0;
  struct stat st;
  char name[80];
  int fd=0;
  strcpy(name,"/reconAsync");
  name[79]='\0';
  strncpy(&name[11],mirstr->prefix,68);

  //first open the queue... (it should already have been created)
  if((mirstr->mq=mq_open(name,O_WRONLY))==-1){
    printf("mq_open failed in mirrorSHM for %s: %s\n",name,strerror(errno));
    err=1;
  }else if((fd=shm_open(name,O_RDWR,0777))<0){//open shm
    printf("shm_open failed in reconAsync for %s\n",name);
    err=1;
  }else if(fstat(fd,&st)!=0){//get size
    printf("Failed to stat %s\n",name);
    err=1;
  }else if((int)st.st_size!=sizeof(pthread_mutex_t)+mirstr->arrsize){//check the size
    printf("Error - size of shm is wrong (%d, should be %d)\n",(int)st.st_size,(int)sizeof(pthread_mutex_t)+mirstr->arrsize);
    err=1;
  }else if((mirstr->shmbuf=mmap(0,(int)st.st_size,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0))==MAP_FAILED){//and open the shm - this contains a mutex, 8 byte header then the data.
    printf("mmap failed for %s: %s\n",name,strerror(errno));
    err=1;
    mirstr->shmbuf=NULL;
  }
  if(fd>0)
    close(fd);
  if(err){//close the queue
    if(mirstr->mq>0)
      mq_close(mirstr->mq);
    mirstr->mq=0;
  }
  return err;
}

/**
   Open a camera of type name.  Args are passed in a float array of size n, which can be cast if necessary.  Any state data is returned in camHandle, which should be NULL if an error arises.
   pxlbuf is the array that should hold the data. The library is free to use the user provided version, or use its own version as necessary (ie a pointer to physical memory or whatever).  It is of size npxls*sizeof(short).
   ncam is number of cameras, which is the length of arrays pxlx and pxly, which contain the dimensions for each camera.
   Name is used if a library can support more than one camera.
*/

#define RETERR mirrordofree(mirstr);*mirrorHandle=NULL;return 1;
int mirrorOpen(char *name,int narg,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **mirrorHandle,int nacts,circBuf *rtcActuatorBuf,unsigned int frameno, unsigned int **mirrorframeno,int *mirrorframenoSize){
  //int err;
  MirrorStruct *mirstr;
  int err;
  char *pn;
  printf("Initialising mirrorSHM %s\n",name);
  if((pn=makeParamNames())==NULL){
    printf("Error making paramList - please recode mirrorSHM.c\n");
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
  mirstr->prefix=prefix;
  mirstr->nacts=nacts;
  mirstr->rtcErrorBuf=rtcErrorBuf;
  mirstr->rtcActuatorBuf=rtcActuatorBuf;
  mirstr->arrStr=arr;
  if(narg>1){
    mirstr->timeout=args[0];
    mirstr->asfloat=args[1];
  }else{
    printf("wrong number of args - should be timeout, asfloat flag\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  mirstr->arrsize=HDRSIZE+nacts*(mirstr->asfloat?sizeof(float):sizeof(unsigned short));
  //if((mirstr->arr=malloc(mirstr->arrsize))==NULL){
  //  printf("couldn't malloc arr\n");
  //  mirrordofree(mirstr);
  //  *mirrorHandle=NULL;
  //  return 1;
  // }
  //memset(mirstr->arr,0,mirstr->arrsize);

  if((err=openMirrorSHM(mirstr))!=0){
    printf("error opening shm\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  mirstr->arr=(unsigned short*)&mirstr->shmbuf[sizeof(pthread_mutex_t)];

  /*if(pthread_cond_init(&mirstr->cond,NULL)!=0){
    printf("Error initialising thread condition variable\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
    }*/
  //maybe think about having one per camera???
  if(pthread_mutex_init(&mirstr->m,NULL)!=0){
    printf("Error initialising mutex variable\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  mirstr->open=1;
  if((err=mirrorNewParam(*mirrorHandle,pbuf,frameno,arr))){
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  //if(err==0)
  //  pthread_create(&mirstr->threadid,NULL,mirrorworker,mirstr);
  return err;
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
    //pthread_cond_signal(&mirstr->cond);//wake the thread.
    pthread_mutex_unlock(&mirstr->m);
    //pthread_join(mirstr->threadid,NULL);//wait for worker thread to complete
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
  }
  printf("Mirror closed\n");
  return 0;
}

/**
   Called asynchronously from the main subap processing threads.
*/
int mirrorSend(void *mirrorHandle,int n,float *data,unsigned int frameno,double timestamp,int err){
  MirrorStruct *mirstr=(MirrorStruct*)mirrorHandle;
  //int err=0;
  int nclipped=0;
  int intDMCommand;
  MirrorStructBuffered *msb;
  unsigned short *actsSent;
  float *factsSent;
  int i;
  char msg='\0';
  float val;
  struct timespec timeout;
  if(mirstr==NULL)
    return -1;
  if(mirstr!=NULL && mirstr->open==1 && err==0){
    pthread_mutex_lock(&mirstr->m);
    if(mirstr->open==0){
      pthread_mutex_unlock(&mirstr->m);
      return -1;
    }
    actsSent=&(mirstr->arr[HDRSIZE/sizeof(unsigned short)]);
    factsSent=&(((float*)mirstr->arr)[HDRSIZE/sizeof(float)]);
    //printf("Sending %d values to mirror\n",n);
    if(mirstr->swap){
      mirstr->buf=1-mirstr->buf;
      mirstr->swap=0;
    }
    msb=&mirstr->msb[mirstr->buf];
    err=mirstr->err;//get the error from the last time.  Even if there was an error, need to send new actuators, to wake up the thread... incase the error has gone away.
    //First, copy actuators.  Note, should n==mirstr->nacts.
    //we also do clipping etc here...
    //memcpy(&mirstr->arr[HDRSIZE/sizeof(unsigned short)],data,sizeof(unsigned short)*mirstr->nacts);
    //Now copy the data into shm...
    pthread_mutex_lock((pthread_mutex_t*)mirstr->shmbuf);


    for(i=0; i<mirstr->nacts; i++){
      val=data[i];
      if(msb->actScale!=NULL)
	val*=msb->actScale[i];
      if(msb->actOffset!=NULL)
	val+=msb->actOffset[i];
      if(mirstr->asfloat==0){
	//convert to int (with rounding)
	intDMCommand=(int)(val+0.5);
	actsSent[i]=(unsigned short)intDMCommand;
	if(intDMCommand<msb->actMin[i]){
	  nclipped++;
	  actsSent[i]=msb->actMin[i];
	}
	if(intDMCommand>msb->actMax[i]){
	  nclipped++;
	  actsSent[i]=msb->actMax[i];
	}
      }else{
	factsSent[i]=val;
	if(val<msb->actMin[i]){
	  nclipped++;
	  factsSent[i]=msb->actMin[i];
	}
	if(val>msb->actMax[i]){
	  nclipped++;
	  factsSent[i]=msb->actMax[i];
	}
      }
    }
    
    ((unsigned int*)mirstr->arr)[1]=frameno;
    ((unsigned int*)mirstr->arr)[0]=(0x5555<<(16+mirstr->asfloat))|mirstr->nacts;
    
    pthread_mutex_unlock((pthread_mutex_t*)mirstr->shmbuf);
    //And now write to the mqueue
    clock_gettime(CLOCK_REALTIME, &timeout);
    timeout.tv_sec+=mirstr->timeout/1000;
    timeout.tv_nsec+=(mirstr->timeout%1000)*1000000;
    if(timeout.tv_nsec>=1000000000){
      timeout.tv_sec++;
      timeout.tv_nsec-=1000000000;
    }
    if(mq_timedsend(mirstr->mq,&msg,0,0,&timeout)==-1){
      printf("Error in mirrorSHM mirrorSend sending to mq: %s\n",strerror(errno));
      err=1;
    }

    //pthread_cond_signal(&mirstr->cond);
    //printf("circadd %u %g\n",frameno,timestamp);
    circAdd(mirstr->rtcActuatorBuf,actsSent,timestamp,frameno);//actsSent);
    pthread_mutex_unlock(&mirstr->m);
    //if(msb->lastActs!=NULL)
    //  memcpy(msb->lastActs,actsSent,sizeof(unsigned short)*mirstr->nacts);

  }else{
    err=1;
    printf("Mirror library error frame %u - not sending\n",frameno);
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
  int dim;
  int bufno;
  MirrorStructBuffered *msb;
  //int *indx=mirstr->bufindx;
  //MIRRORBUFFERVARIABLEINDX i;
  int j=0;
  int nfound;
  //int nbytes;
  int *indx=mirstr->index;
  void **values=mirstr->values;
  char *dtype=mirstr->dtype;
  int *nbytes=mirstr->nbytes;
  if(mirstr==NULL || mirstr->open==0){
    printf("Mirror not open\n");
    return 1;
  }
  mirstr->arrStr=arr;
  bufno=1-mirstr->buf;
  msb=&mirstr->msb[bufno];
  //memset(indx,-1,sizeof(int)*MIRRORNBUFFERVARIABLES);
  nfound=bufferGetIndex(pbuf,MIRRORNBUFFERVARIABLES,mirstr->paramNames,indx,values,dtype,nbytes);
  if(nfound!=MIRRORNBUFFERVARIABLES){
    for(j=0; j<MIRRORNBUFFERVARIABLES; j++){
      if(indx[j]<0 && j!=MIRRORACTOFFSET && j!=MIRRORACTSCALE){
	printf("ERROR Missing %16s\n",&mirstr->paramNames[j*BUFNAMESIZE]);
	writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"Error in mirror parameter buffer: %16s",&mirstr->paramNames[j*BUFNAMESIZE]);
	err=-1;
      }
    }
  }
  if(err==0){
    if(nbytes[MIRRORNACTS]==sizeof(int) && dtype[MIRRORNACTS]=='i'){
      if(mirstr->nacts!=*((int*)(values[MIRRORNACTS]))){
	printf("Error - nacts changed - please close and reopen mirror library\n");
	err=MIRRORNACTS;
      }
    }else{
      printf("mirrornacts error\n");
      writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"mirrornacts error");
      err=MIRRORNACTS;
    }
    if(mirstr->rtcActuatorBuf!=NULL && mirstr->rtcActuatorBuf->datasize!=mirstr->nacts*(mirstr->asfloat?sizeof(float):sizeof(unsigned short))){
      dim=mirstr->nacts;
      if(circReshape(mirstr->rtcActuatorBuf,1,&dim,mirstr->asfloat?'f':'H')!=0){
	printf("Error reshaping rtcActuatorBuf\n");
      }
    }
    if(dtype[MIRRORACTMIN]=='H' && nbytes[MIRRORACTMIN]==sizeof(unsigned short)*mirstr->nacts){
      if(msb->actMinSize<mirstr->nacts){
	if(msb->actMin!=NULL)
	  free(msb->actMin);
	if((msb->actMin=malloc(sizeof(unsigned short)*mirstr->nacts))==NULL){
	  printf("Error allocating actMin\n");
	  msb->actMinSize=0;
	  writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,
		       "mirrorActMin malloc error");
	  err=MIRRORACTMIN;
	}else{
	  msb->actMinSize=mirstr->nacts;
	}
      }
      if(msb->actMin!=NULL)
	memcpy(msb->actMin,values[MIRRORACTMIN],
	       sizeof(unsigned short)*mirstr->nacts);
    }else{
      printf("mirrorActMin error\n");
      writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"mirrorActMin error");
      err=MIRRORACTMIN;
    }
    if(dtype[MIRRORACTMAX]=='H' && nbytes[MIRRORACTMAX]==sizeof(unsigned short)*mirstr->nacts){
      if(msb->actMaxSize<mirstr->nacts){
	if(msb->actMax!=NULL)
	  free(msb->actMax);
	if((msb->actMax=malloc(sizeof(unsigned short)*mirstr->nacts))==NULL){
	  printf("Error allocating actMax\n");
	  msb->actMaxSize=0;
	  writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,
		       "mirrorActMax malloc error");
	  err=MIRRORACTMAX;
	}else{
	  msb->actMaxSize=mirstr->nacts;
	}
      }
      if(msb->actMax!=NULL)
	memcpy(msb->actMax,values[MIRRORACTMAX],
	       sizeof(unsigned short)*mirstr->nacts);
    }else{
      printf("mirrorActMax error\n");
      writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"mirrorActMax error");
      err=MIRRORACTMAX;
    }
    if(indx[MIRRORACTSCALE]>=0){//parameter is in the buf.
      if(nbytes[MIRRORACTSCALE]==0){
	msb->actScale=NULL;
      }else if(dtype[MIRRORACTSCALE]=='f' && nbytes[MIRRORACTSCALE]==sizeof(float)*mirstr->nacts){
	if(msb->actScaleSize<mirstr->nacts){
	  if(msb->actScaleArr!=NULL)
	    free(msb->actScaleArr);
	  if((msb->actScaleArr=malloc(sizeof(float)*mirstr->nacts))==NULL){
	    printf("Error allocatring actScaleArr\n");
	    msb->actScaleSize=0;
	    writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"actScaleArr malloc error\n");
	    err=MIRRORACTSCALE;
	  }else{
	    msb->actScaleSize=mirstr->nacts;
	  }
	}
	msb->actScale=msb->actScaleArr;
	if(msb->actScale!=NULL)

	  memcpy(msb->actScale,values[MIRRORACTSCALE],sizeof(float)*mirstr->nacts);
      }else{
	printf("actScale error\n");
	writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"actScale error");
	err=MIRRORACTSCALE;
	msb->actScale=NULL;
      }
    }else{
      msb->actScale=NULL;
    }
    if(indx[MIRRORACTOFFSET]>=0){//parameter is in the buf.
      if(nbytes[MIRRORACTOFFSET]==0){
	msb->actOffset=NULL;
      }else if(dtype[MIRRORACTOFFSET]=='f' && nbytes[MIRRORACTOFFSET]==sizeof(float)*mirstr->nacts){
	if(msb->actOffsetSize<mirstr->nacts){
	  if(msb->actOffsetArr!=NULL)
	    free(msb->actOffsetArr);
	  if((msb->actOffsetArr=malloc(sizeof(float)*mirstr->nacts))==NULL){
	    printf("Error allocatring actOffsetArr\n");
	    msb->actOffsetSize=0;
	    writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"actOffsetArr malloc error\n");
	    err=MIRRORACTOFFSET;
	  }else{
	    msb->actOffsetSize=mirstr->nacts;
	  }
	}
	msb->actOffset=msb->actOffsetArr;
	if(msb->actOffset!=NULL)

	  memcpy(msb->actOffset,values[MIRRORACTOFFSET],sizeof(float)*mirstr->nacts);
      }else{
	printf("actOffset error\n");
	writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"actOffset error");
	err=MIRRORACTOFFSET;
	msb->actOffset=NULL;
      }
    }else{
      msb->actOffset=NULL;
    }
  }
  pthread_mutex_lock(&mirstr->m);
  mirstr->swap=1;
  pthread_mutex_unlock(&mirstr->m);
  return err;
}
