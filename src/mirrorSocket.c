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

Try:
echo 1 > /proc/sys/net/ipv4/tcp_low_latency
on both the sending and receiving ends
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include "rtcmirror.h"
#include <time.h>
#include <pthread.h>
#include "darc.h"
#include "agbcblas.h"
#define HDRSIZE 8 //the size of a WPU header - 4 bytes for frame no, 4 bytes for something else.


typedef enum{
  MIRRORACTCONTROLMX,
  MIRRORACTMAX,
  MIRRORACTMIN,
  MIRRORACTNEW,
  MIRRORACTOFFSET,
  MIRRORACTPOWER,
  MIRRORACTSCALE,
  MIRRORDELAY,
  MIRRORNACTS,
  //Add more before this line.
  MIRRORNBUFFERVARIABLES//equal to number of entries in the enum
}MIRRORBUFFERVARIABLEINDX;

#define makeParamNames() bufferMakeNames(MIRRORNBUFFERVARIABLES,	\
					 "actControlMx",		\
					 "actMax",			\
					 "actMin",			\
					 "actNew",			\
					 "actOffset",			\
					 "actPower",			\
					 "actScale",			\
					 "mirrorDelay",			\
					 "nacts"			\
					 )
//char *MIRRORPARAM[]={"nacts","actMin","actMax","actScale","actOffset"};//,"lastActs"};


typedef struct{
  float *actMin;
  float *actMax;
  int actMinSize;
  int actMaxSize;
  int actOffsetSize;
  int actPowerSize;
  int actScaleSize;
  float *actOffset;
  float *actOffsetArr;
  float *actPower;
  float *actPowerArr;
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
  pthread_t threadid;
  pthread_cond_t cond;
  pthread_mutex_t m;
  int timeout;//in ms
  int socket;
  unsigned int *threadAffinity;
  int threadAffinElSize;
  int threadPriority;
  MirrorStructBuffered msb[2];
  int buf;
  int swap;
  //int bufindx[MIRRORNBUFFERVARIABLES];
  circBuf *rtcActuatorBuf;
  circBuf *rtcErrorBuf;
  char *host;
  unsigned short port;
  int index[MIRRORNBUFFERVARIABLES];
  void *values[MIRRORNBUFFERVARIABLES];
  char dtype[MIRRORNBUFFERVARIABLES];
  int nbytes[MIRRORNBUFFERVARIABLES];
  char *prefix;
  int sendPrefix;
  int asfloat;
  float mirrorDelay;
  int nactsNew;
  float *actsNew;
  int actsNewSize;
  int *actControlMx;
  struct timespec nanodelay;
}MirrorStruct;

/**
   Free mirror/memory/sl240
*/
void mirrordofree(MirrorStruct *mirstr){
  int i;
  if(mirstr!=NULL){
    if(mirstr->arr!=NULL)
      free(mirstr->arr);
    for(i=0; i<2; i++){
      if(mirstr->msb[i].actMin!=NULL)free(mirstr->msb[i].actMin);
      if(mirstr->msb[i].actMax!=NULL)free(mirstr->msb[i].actMax);
    }
    pthread_cond_destroy(&mirstr->cond);
    pthread_mutex_destroy(&mirstr->m);
    if(mirstr->socket!=0){
      close(mirstr->socket);
    }
    free(mirstr);
  }
}

int setThreadAffinityForDMC(unsigned int *threadAffinity,int threadPriority,int threadAffinElSize){
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
  if(pthread_setaffinity_np(pthread_self(),sizeof(cpu_set_t),&mask))
    printf("Error in pthread_setaffinity_np: %s\n",strerror(errno));
  param.sched_priority=threadPriority;
  if(pthread_setschedparam(pthread_self(),SCHED_RR,&param))
    printf("error in pthread_setschedparam - maybe run as root?\n");
  return 0;
}

/**
   The thread that does the work - copies actuators, and sends via SL240
*/
void* mirrorworker(void *mirstrv){
  MirrorStruct *mirstr=(MirrorStruct*)mirstrv;
  int n,totsent=0,err=0;
  setThreadAffinityForDMC(mirstr->threadAffinity,mirstr->threadPriority,mirstr->threadAffinElSize);
  pthread_mutex_lock(&mirstr->m);
  while(mirstr->open){
    pthread_cond_wait(&mirstr->cond,&mirstr->m);//wait for actuators.
    if(mirstr->open){
      //Now send the data...
      totsent=0;
      if(mirstr->mirrorDelay!=0){
	nanosleep(&mirstr->nanodelay,NULL);
      }
      while(err==0 && totsent<mirstr->arrsize){
	n=send(mirstr->socket,&mirstr->arr[totsent],mirstr->arrsize-totsent,0);
	if(n<0){//error
	  err=-1;
	  printf("Error sending data: %s\n",strerror(errno));
	}else{
	  totsent+=n;
	}
      }
    }
  }
  pthread_mutex_unlock(&mirstr->m);
  return NULL;
}

int openMirrorSocket(MirrorStruct *mirstr){
  int err=0;
  struct sockaddr_in sin;
  struct hostent *host;
  int n;
  int flag=1;
  if((host=gethostbyname(mirstr->host))==NULL){
    printf("gethostbyname error %s\n",mirstr->host);
    err=1;
  }else{
    mirstr->socket=socket(PF_INET,SOCK_STREAM,0);
    //disable Nagle algorithm to improve real-time performance (reduce latency for small packets).
    if(setsockopt(mirstr->socket,IPPROTO_TCP,TCP_NODELAY,(void*)&flag,sizeof(int))<0){
      printf("Error in setsockopt in mirrorSocket\n");
    }
    memcpy(&sin.sin_addr.s_addr,host->h_addr,host->h_length);
    sin.sin_family=AF_INET;
    sin.sin_port=htons(mirstr->port);
    if(connect(mirstr->socket,(struct sockaddr*)&sin,sizeof(sin))<0){
      printf("Error connecting\n");
      close(mirstr->socket);
      mirstr->socket=0;
      err=1;
    }else{
      if(mirstr->sendPrefix){
	n=strlen(mirstr->prefix);
	if(send(mirstr->socket,&n,sizeof(int),0)==-1 || send(mirstr->socket,mirstr->prefix,n,0)==-1){
	  printf("Unable to send prefix - error\n");
	  err=1;
	  close(mirstr->socket);
	  mirstr->socket=0;
	}
      }
    }
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
  printf("Initialising mirrorSocket %s\n",name);
  if((pn=makeParamNames())==NULL){
    printf("Error making paramList - please recode mirrorSocket.c\n");
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
  if(narg>7){
    mirstr->timeout=args[0];
    mirstr->port=args[1];
    mirstr->threadAffinElSize=args[2];
    mirstr->threadPriority=args[3];
    mirstr->threadAffinity=(unsigned int*)&args[4];
    mirstr->sendPrefix=args[4+args[2]];
    mirstr->asfloat=args[5+args[2]];
    mirstr->host=strndup((char*)&(args[6+args[2]]),(narg-6-args[2])*sizeof(int));
    printf("Got host %s\n",mirstr->host);
  }else{
    printf("wrong number of args - should be timeout, port, Naffin, thread priority,thread affinity[Naffin], sendPrefix flag, asfloat flag, hostname (string)\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  mirstr->arrsize=HDRSIZE+nacts*(mirstr->asfloat?sizeof(float):sizeof(unsigned short));
  if((mirstr->arr=malloc(mirstr->arrsize))==NULL){
    printf("couldn't malloc arr\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  memset(mirstr->arr,0,mirstr->arrsize);

  if(*mirrorframenoSize==0){
    if((*mirrorframeno=malloc(sizeof(int)))==NULL){
      printf("Unable to malloc mirrorframeno\n");
    }else{
      *mirrorframenoSize=1;
    }
  }
  **mirrorframeno=0;

  if((err=openMirrorSocket(mirstr))!=0){
    printf("error opening socket\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
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
  mirstr->open=1;
  if((err=mirrorNewParam(*mirrorHandle,pbuf,frameno,arr))){
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  if(err==0)
    pthread_create(&mirstr->threadid,NULL,mirrorworker,mirstr);
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
    pthread_cond_signal(&mirstr->cond);//wake the thread.
    pthread_mutex_unlock(&mirstr->m);
    pthread_join(mirstr->threadid,NULL);//wait for worker thread to complete
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
  }
  printf("Mirror closed\n");
  return 0;
}

/**
   Called asynchronously from the main subap processing threads.
*/
int mirrorSend(void *mirrorHandle,int n,float *data,unsigned int frameno,double timestamp,int err,int writeCirc){
  MirrorStruct *mirstr=(MirrorStruct*)mirrorHandle;
  //int err=0;
  int nclipped=0;
  int intDMCommand;
  MirrorStructBuffered *msb;
  unsigned short *actsSent=&(((unsigned short*)mirstr->arr)[HDRSIZE/sizeof(unsigned short)]);
  float *factsSent=&(((float*)mirstr->arr)[HDRSIZE/sizeof(float)]);
  int i,nacts;
  float val;
  if(mirstr!=NULL && mirstr->open==1 && err==0){
    //printf("Sending %d values to mirror\n",n);
    pthread_mutex_lock(&mirstr->m);
    if(mirstr->swap){
      mirstr->buf=1-mirstr->buf;
      mirstr->swap=0;
    }
    msb=&mirstr->msb[mirstr->buf];
    err=mirstr->err;//get the error from the last time.  Even if there was an error, need to send new actuators, to wake up the thread... incase the error has gone away.
    //First, copy actuators.  Note, should n==mirstr->nacts.
    //we also do clipping etc here...
    //memcpy(&mirstr->arr[HDRSIZE/sizeof(unsigned short)],data,sizeof(unsigned short)*mirstr->nacts);
    if(mirstr->actControlMx!=NULL){
      //multiply acts by a matrix (sparse), to get a new set of acts.
      //This therefore allows to build up actuators that are a combination of other actuators.  e.g. a 3-actuator tiptilt mirror from 2x tiptilt signal.
      agb_cblas_sparse_csr_sgemvRowMN1N101(mirstr->nactsNew,mirstr->nacts,mirstr->actControlMx,data,mirstr->actsNew);
      data=mirstr->actsNew;
      //results placed in factsNew (of size nactsNew).
      nacts=mirstr->nactsNew;
    }else{
      nacts=mirstr->nacts;
    }

    for(i=0; i<nacts; i++){
      val=data[i];
      if(msb->actScale!=NULL)
	val*=msb->actScale[i];
      if(msb->actOffset!=NULL)
	val+=msb->actOffset[i];
      if(msb->actPower!=NULL){
	if(msb->actPower[i]==2)
	  val*=val;
	else
	  val=powf(val,msb->actPower[i]);
      }
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
    ((unsigned int*)mirstr->arr)[0]=(0x5555<<(16+mirstr->asfloat))|nacts;
    //Wake up the thread.
    pthread_cond_signal(&mirstr->cond);
    pthread_mutex_unlock(&mirstr->m);
    //printf("circadd %u %g\n",frameno,timestamp);
    if(writeCirc)
      circAddForce(mirstr->rtcActuatorBuf,actsSent,timestamp,frameno);//actsSent);
    //if(msb->lastActs!=NULL)
    //  memcpy(msb->lastActs,actsSent,sizeof(unsigned short)*nacts);

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
  int nactsNew;
  int actControlMxSize;
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
      if(indx[j]<0){
	if(j!=MIRRORACTOFFSET && j!=MIRRORACTSCALE && j!=MIRRORACTPOWER && j!=MIRRORDELAY && j!=MIRRORACTCONTROLMX && j!=MIRRORACTNEW){
	  printf("ERROR Missing %16s\n",&mirstr->paramNames[j*BUFNAMESIZE]);
	  writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"Error in mirror parameter buffer: %16s",&mirstr->paramNames[j*BUFNAMESIZE]);
	  err=-1;
	}else{
	  printf("Warning: Missing %16s\n",&mirstr->paramNames[j*BUFNAMESIZE]);
	}
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
    if(indx[MIRRORACTNEW]>=0){
      if(nbytes[MIRRORACTNEW]==sizeof(int) && dtype[MIRRORACTNEW]=='i'){
	mirstr->nactsNew=*(int*)values[MIRRORACTNEW];
      }else{
	printf("Warning - actNew wrong size or type\n");
	err=1;
	mirstr->nactsNew=0;
      }
    }else{
      mirstr->nactsNew=mirstr->nacts;
    }
    if(indx[MIRRORACTCONTROLMX]>=0){
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
    }else{
      mirstr->actControlMx=NULL;
    }
    if(mirstr->actControlMx==NULL)
      nactsNew=mirstr->nacts;
    else
      nactsNew=mirstr->nactsNew;


    //mirstr->arrsize=HDRSIZE+nacts*(mirstr->asfloat?sizeof(float):sizeof(unsigned short));

    if(mirstr->arrsize<HDRSIZE+nactsNew*(mirstr->asfloat?sizeof(float):sizeof(unsigned short))){//bytes[MIRRORACTMAPPING]){
      if(mirstr->arr!=NULL) 
	free(mirstr->arr);
      if((mirstr->arr=malloc(HDRSIZE+nactsNew*(mirstr->asfloat?sizeof(float):sizeof(unsigned short))))==NULL){//nbytes[MIRRORACTMAPPING]))==NULL){
	printf("Error allocating mirstr->arr\n");
	err=1;
	mirstr->arrsize=0;
      }else{
	mirstr->arrsize=HDRSIZE+nactsNew*(mirstr->asfloat?sizeof(float):sizeof(unsigned short));
	memset(mirstr->arr,0,mirstr->arrsize);
      }
    }

    if(mirstr->rtcActuatorBuf!=NULL && mirstr->rtcActuatorBuf->datasize!=nactsNew*(mirstr->asfloat?sizeof(float):sizeof(unsigned short))){
      dim=nactsNew;
      if(circReshape(mirstr->rtcActuatorBuf,1,&dim,mirstr->asfloat?'f':'H')!=0){
	printf("Error reshaping rtcActuatorBuf\n");
      }
    }

    if(dtype[MIRRORACTMIN]=='H' && nbytes[MIRRORACTMIN]>=sizeof(unsigned short)*nactsNew){
      if(msb->actMinSize<nactsNew){
	if(msb->actMin!=NULL)
	  free(msb->actMin);
	if((msb->actMin=malloc(sizeof(float)*nactsNew))==NULL){
	  printf("Error allocating actMin\n");
	  msb->actMinSize=0;
	  writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,
		       "mirrorActMin malloc error");
	  err=1;
	}else{
	  msb->actMinSize=nactsNew;
	}
      }
      if(msb->actMin!=NULL){
	for(j=0;j<nactsNew;j++)//convert from H to f
	  msb->actMin[j]=(float)((unsigned short*)(values[MIRRORACTMIN]))[j];
      }
    }else if(dtype[MIRRORACTMIN]=='f' && nbytes[MIRRORACTMIN]>=sizeof(float)*nactsNew){
      if(msb->actMinSize<nactsNew){
	if(msb->actMin!=NULL)
	  free(msb->actMin);
	if((msb->actMin=malloc(sizeof(float)*nactsNew))==NULL){
	  printf("Error allocating actMin\n");
	  msb->actMinSize=0;
	  writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,
		       "mirrorActMin malloc error");
	  err=1;
	}else{
	  msb->actMinSize=nactsNew;
	}
      }
      if(msb->actMin!=NULL){
	memcpy(msb->actMin,values[MIRRORACTMIN],sizeof(float)*nactsNew);
      }
    }else{
      printf("mirrorActMin error\n");
      writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"mirrorActMin error");
      err=1;
    }
    if(dtype[MIRRORACTMAX]=='H' && nbytes[MIRRORACTMAX]>=sizeof(unsigned short)*nactsNew){
      if(msb->actMaxSize<nactsNew){
	if(msb->actMax!=NULL)
	  free(msb->actMax);
	if((msb->actMax=malloc(sizeof(float)*nactsNew))==NULL){
	  printf("Error allocating actMax\n");
	  msb->actMaxSize=0;
	  writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,
		       "mirrorActMax malloc error");
	  err=1;
	}else{
	  msb->actMaxSize=nactsNew;
	}
      }
      if(msb->actMax!=NULL){
	for(j=0;j<nactsNew;j++)//convert from H to f
	  msb->actMax[j]=(float)((unsigned short*)(values[MIRRORACTMAX]))[j];
	//memcpy(msb->actMax,values[MIRRORACTMAX],
	//      sizeof(unsigned short)*mirstr->nacts);
      }
    }else if(dtype[MIRRORACTMAX]=='f' && nbytes[MIRRORACTMAX]>=sizeof(float)*nactsNew){
      if(msb->actMaxSize<nactsNew){
	if(msb->actMax!=NULL)
	  free(msb->actMax);
	if((msb->actMax=malloc(sizeof(float)*nactsNew))==NULL){
	  printf("Error allocating actMax\n");
	  msb->actMaxSize=0;
	  writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,
		       "mirrorActMax malloc error");
	  err=1;
	}else{
	  msb->actMaxSize=nactsNew;
	}
      }
      if(msb->actMax!=NULL){
	memcpy(msb->actMax,values[MIRRORACTMAX],sizeof(float)*nactsNew);
      }
    }else{
      printf("mirrorActMax error\n");
      writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"mirrorActMax error");
      err=1;
    }
    if(indx[MIRRORACTSCALE]>=0){//parameter is in the buf.
      if(nbytes[MIRRORACTSCALE]==0){
	msb->actScale=NULL;
      }else if(dtype[MIRRORACTSCALE]=='f' && nbytes[MIRRORACTSCALE]==sizeof(float)*nactsNew){
	if(msb->actScaleSize<nactsNew){
	  if(msb->actScaleArr!=NULL)
	    free(msb->actScaleArr);
	  if((msb->actScaleArr=malloc(sizeof(float)*nactsNew))==NULL){
	    printf("Error allocatring actScaleArr\n");
	    msb->actScaleSize=0;
	    writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"actScaleArr malloc error\n");
	    err=1;
	  }else{
	    msb->actScaleSize=nactsNew;
	  }
	}
	msb->actScale=msb->actScaleArr;
	if(msb->actScale!=NULL)

	  memcpy(msb->actScale,values[MIRRORACTSCALE],sizeof(float)*nactsNew);
      }else{
	printf("actScale error\n");
	writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"actScale error");
	err=1;
	msb->actScale=NULL;
      }
    }else{
      msb->actScale=NULL;
    }
    if(indx[MIRRORACTOFFSET]>=0){//parameter is in the buf.
      if(nbytes[MIRRORACTOFFSET]==0){
	msb->actOffset=NULL;
      }else if(dtype[MIRRORACTOFFSET]=='f' && nbytes[MIRRORACTOFFSET]>=sizeof(float)*nactsNew){//note, >= here because some mirror modules may create additional actuators (eg the mirrorPdao32 module), so this allows that to be ignored.
	if(msb->actOffsetSize<nactsNew){
	  if(msb->actOffsetArr!=NULL)
	    free(msb->actOffsetArr);
	  if((msb->actOffsetArr=malloc(sizeof(float)*nactsNew))==NULL){
	    printf("Error allocatring actOffsetArr\n");
	    msb->actOffsetSize=0;
	    writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"actOffsetArr malloc error\n");
	    err=1;
	  }else{
	    msb->actOffsetSize=nactsNew;
	  }
	}
	msb->actOffset=msb->actOffsetArr;
	if(msb->actOffset!=NULL)

	  memcpy(msb->actOffset,values[MIRRORACTOFFSET],sizeof(float)*nactsNew);
      }else{
	printf("actOffset error\n");
	writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"actOffset error");
	err=1;
	msb->actOffset=NULL;
      }
    }else{
      msb->actOffset=NULL;
    }
    if(indx[MIRRORACTPOWER]>=0){//parameter is in the buf.
      if(nbytes[MIRRORACTPOWER]==0){
	msb->actPower=NULL;
      }else if(dtype[MIRRORACTPOWER]=='f' && nbytes[MIRRORACTPOWER]==sizeof(float)*nactsNew){
	if(msb->actPowerSize<nactsNew){
	  if(msb->actPowerArr!=NULL)
	    free(msb->actPowerArr);
	  if((msb->actPowerArr=malloc(sizeof(float)*nactsNew))==NULL){
	    printf("Error allocatring actPowerArr\n");
	    msb->actPowerSize=0;
	    writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"actPowerArr malloc error\n");
	    err=1;
	  }else{
	    msb->actPowerSize=nactsNew;
	  }
	}
	msb->actPower=msb->actPowerArr;
	if(msb->actPower!=NULL)

	  memcpy(msb->actPower,values[MIRRORACTPOWER],sizeof(float)*nactsNew);
      }else{
	printf("actPower error\n");
	writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"actPower error");
	err=1;
	msb->actPower=NULL;
      }
    }else{
      msb->actPower=NULL;
    }
    mirstr->mirrorDelay=0;
    mirstr->nanodelay.tv_sec=0;
    mirstr->nanodelay.tv_nsec=0;
    if(indx[MIRRORDELAY]>=0){
      if(dtype[MIRRORDELAY]=='f' && nbytes[MIRRORDELAY]==sizeof(float)){
	mirstr->mirrorDelay=*((float*)values[MIRRORDELAY]);
	mirstr->nanodelay.tv_sec=(int)mirstr->mirrorDelay;
	mirstr->nanodelay.tv_nsec=(mirstr->mirrorDelay-mirstr->nanodelay.tv_sec)*1e9;
      }else{
	printf("Error in mirrorDelay\n");
	writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"mirrorDelay error\n");
	err=1;
      }
    }
	

  }
  pthread_mutex_lock(&mirstr->m);
  mirstr->swap=1;
  pthread_mutex_unlock(&mirstr->m);
  return err;
}
