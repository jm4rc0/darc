//Alpao mirror, using Ethernet interface adapter.
//Including creep compensation algorithms.

/*
Creep compensation:

1.  Need a record of the average DM shape over the last few(?) hours.
a.  Probably using a gain/decay factor - with a very low gain.
b.  What if DM is frozen (ie no actuator buffer active)?
c.  What about an initial starting state?
d.  How long should the average shape be averaged over?
e.g. if AO loop at 150Hz, create average shape by doing Av = (1-g)Av + gNew where g = 1e-6, or something.
f.  Update this mean shape every few (X) minutes.

This is done by leakyaverage.c

2.  Once have the average shape, then need the creep compensation.  Decays gradually towards the average shape.
a.  

This library uses the ALPAO SDK.
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
#include <netdb.h>
#include "rtcmirror.h"
#include <time.h>
#include <pthread.h>
#include "darc.h"
#include "agbcblas.h"
#ifndef NODM
#include "asdkWrapper.h"//alpao library wrapper.
#else
typedef int asdkDM;
#endif

#define HDRSIZE 0
typedef enum{
  MIRRORACTCONTROLMX,
  MIRRORACTMAX,
  MIRRORACTMIN,
  MIRRORACTNEW,
  MIRRORACTOFFSET,
  MIRRORACTSCALE,
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
					 "actScale",			\
					 "nacts"			\
					 )

typedef struct{
  float *actMin;
  float *actMax;
  int actMinSize;
  int actMaxSize;
  int actOffsetSize;
  int actScaleSize;
  float *actOffset;
  float *actOffsetArr;
  float *actScale;
  float *actScaleArr;
}MirrorStructBuffered;

typedef struct{
  char *paramNames;
  int nacts;
  double *arr;
  //int frameno;
  int arrsize;
  int open;
  int err;
  arrayStruct *arrStr;
  pthread_t threadid;
  pthread_cond_t cond;
  pthread_mutex_t m;
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
  int index[MIRRORNBUFFERVARIABLES];
  void *values[MIRRORNBUFFERVARIABLES];
  char dtype[MIRRORNBUFFERVARIABLES];
  int nbytes[MIRRORNBUFFERVARIABLES];
  char *prefix;
  int nactsNew;
  float *actsNew;
  int actsNewSize;
  int *actControlMx;
  asdkDM *handle;
  char *serialName;
  unsigned int *mirrorframeno;
}MirrorStruct;



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

int mirrorClose(void **mirrorHandle){
  MirrorStruct *mirstr=(MirrorStruct*)*mirrorHandle;
  printf("Closing mirror\n");
  if(mirstr!=NULL){
    pthread_mutex_lock(&mirstr->m);
    mirstr->open=0;
    pthread_cond_signal(&mirstr->cond);//wake the thread.
    pthread_mutex_unlock(&mirstr->m);
    pthread_join(mirstr->threadid,NULL);//wait for worker thread to complete
#ifndef NODM
    if(mirstr->handle!=NULL)
      asdkRelease(mirstr->handle);
#endif
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
  }
  printf("Mirror closed\n");
  return 0;
}


int setAlpaoThreadAffinity(unsigned int *threadAffinity,int threadPriority,int threadAffinElSize){
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

void* mirrorworker(void *mirstrv){
  MirrorStruct *mirstr=(MirrorStruct*)mirstrv;
  setAlpaoThreadAffinity(mirstr->threadAffinity,mirstr->threadPriority,mirstr->threadAffinElSize);
  pthread_mutex_lock(&mirstr->m);
  while(mirstr->open){
    pthread_cond_wait(&mirstr->cond,&mirstr->m);//wait for actuators.
    if(mirstr->open){
      //Now send the data...
#ifndef NODM
      if(asdkSend(mirstr->handle,mirstr->arr)!=0){
	UInt errorNo;
	char errMsg[80];
	asdkGetLastError(&errorNo,errMsg,80);
	printf("%s\n",errMsg);
	printf("Error: asdkSend function\n");
      }
#endif
      mirstr->mirrorframeno[0]++;//this gives some indication as to whether we're sending to the dm at the AO frame rate (which won't be the case if asdkSend takes too long to complete)
    }
  }
  pthread_mutex_unlock(&mirstr->m);
  return NULL;
}
int openAlpao(MirrorStruct *mirstr){
  double nb=mirstr->nacts;
#ifndef NODM
  if((mirstr->handle=asdkInit(mirstr->serialName))==NULL){
    UInt errorNo;
    char errMsg[80];
    asdkGetLastError(&errorNo,errMsg,80);
    printf("%s\n",errMsg);
    return 1;
  }
  if(asdkGet(mirstr->handle,"NbOfActuator",&nb)!=0){
    printf("Error getting number of actuators\n");
    return 1;
  }
#endif
  if(nb>mirstr->nacts){
    printf("Error: Expecting %g actuators for DM, but nacts is %d\n",nb,mirstr->nacts);
    return 1;
  }else if(nb<mirstr->nacts){
    printf("Warning: %d actuators specified, but DM only has %g\n",mirstr->nacts,nb);
  }
  return 0;
}


#define RETERR mirrordofree(mirstr);*mirrorHandle=NULL;return 1;
int mirrorOpen(char *name,int narg,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **mirrorHandle,int nacts,circBuf *rtcActuatorBuf,unsigned int frameno, unsigned int **mirrorframeno,int *mirrorframenoSize){
  //int err;
  MirrorStruct *mirstr;
  int err;
  char *pn;
  printf("Initialising mirrorSocket %s\n",name);
  if((pn=makeParamNames())==NULL){
    printf("Error making paramList - please recode mirrorAlpao.c\n");
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
  if(*mirrorframenoSize<sizeof(unsigned int)){
    if((*mirrorframeno=malloc(sizeof(unsigned int)))==NULL){
      printf("Unable to alloc mirrorframeno\n");
      mirrordofree(mirstr);
      *mirrorHandle=NULL;
    }
    *mirrorframenoSize=sizeof(unsigned int);
  }
  mirstr->mirrorframeno=*mirrorframeno;
  mirstr->mirrorframeno[0]=0;
  mirstr->arrStr=arr;
  if(narg>3){
    mirstr->threadAffinElSize=args[0];
    mirstr->threadPriority=args[1];
    mirstr->threadAffinity=(unsigned int*)&args[2];
    mirstr->serialName=strndup((char*)&(args[2+args[0]]),(narg-2-args[0])*sizeof(int));
    printf("Got alpao serial number %s\n",mirstr->serialName);//BEL111 for durham
  }else{
    printf("wrong number of args - should be Naffin, thread priority,thread affinity[Naffin],  serialName (string)\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  mirstr->arrsize=HDRSIZE+nacts*sizeof(double);
  if((mirstr->arr=malloc(mirstr->arrsize))==NULL){
    printf("couldn't malloc arr\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  memset(mirstr->arr,0,mirstr->arrsize);

  if((err=openAlpao(mirstr))!=0){
    printf("Error: opening Alpao mirror\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  if(pthread_cond_init(&mirstr->cond,NULL)!=0){
    printf("Error: initialising thread condition variable\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  //maybe think about having one per camera???
  if(pthread_mutex_init(&mirstr->m,NULL)!=0){
    printf("Error: initialising mutex variable\n");
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
   Called asynchronously from the main subap processing threads.
*/
int mirrorSend(void *mirrorHandle,int n,float *data,unsigned int frameno,double timestamp,int err,int writeCirc){
  MirrorStruct *mirstr=(MirrorStruct*)mirrorHandle;
  //int err=0;
  int nclipped=0;
  MirrorStructBuffered *msb;
  double *actsSent=&(((double*)mirstr->arr)[HDRSIZE/sizeof(double)]);
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
      //convert to double
      actsSent[i]=(double)val;
      if(val<msb->actMin[i]){
	nclipped++;
	actsSent[i]=msb->actMin[i];
      }
      if(val>msb->actMax[i]){
	nclipped++;
	actsSent[i]=msb->actMax[i];
      }
    }
    
    //((unsigned int*)mirstr->arr)[1]=frameno;
    //((unsigned int*)mirstr->arr)[0]=(0x5555<<(16+mirstr->asfloat))|nacts;
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
	printf("ERROR Missing %16s\n",&mirstr->paramNames[j*BUFNAMESIZE]);
	if(j!=MIRRORACTOFFSET && j!=MIRRORACTSCALE && j!=MIRRORACTCONTROLMX && j!=MIRRORACTNEW){
	  writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"Error in mirror parameter buffer: %16s",&mirstr->paramNames[j*BUFNAMESIZE]);
	  err=-1;
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


    if(mirstr->arrsize<HDRSIZE+nactsNew*sizeof(double)){//bytes[MIRRORACTMAPPING]){
      if(mirstr->arr!=NULL) 
	free(mirstr->arr);
      if((mirstr->arr=malloc(HDRSIZE+nactsNew*sizeof(double)))==NULL){//nbytes[MIRRORACTMAPPING]))==NULL){
	printf("Error allocating mirstr->arr\n");
	err=1;
	mirstr->arrsize=0;
      }else{
	mirstr->arrsize=HDRSIZE+nactsNew*sizeof(double);
	memset(mirstr->arr,0,mirstr->arrsize);
      }
    }

    if(mirstr->rtcActuatorBuf!=NULL && mirstr->rtcActuatorBuf->datasize!=nactsNew*sizeof(double)){
      dim=nactsNew;
      if(circReshape(mirstr->rtcActuatorBuf,1,&dim,'d')!=0){
	printf("Error reshaping rtcActuatorBuf\n");
      }
    }

    if(dtype[MIRRORACTMIN]=='f' && nbytes[MIRRORACTMIN]>=sizeof(float)*nactsNew){
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
    if(dtype[MIRRORACTMAX]=='f' && nbytes[MIRRORACTMAX]>=sizeof(float)*nactsNew){
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

  }
  pthread_mutex_lock(&mirstr->m);
  mirstr->swap=1;
  pthread_mutex_unlock(&mirstr->m);
  return err;
}
