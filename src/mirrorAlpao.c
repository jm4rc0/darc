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

This library does not use the ALPAO SDK - uses sockets only.  NOTE - Not yet complete...

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

#define HDRSIZE 8
typedef enum{
  MIRRORACTCONTROLMX,
  MIRRORACTMAX,
  MIRRORACTMIN,
  MIRRORACTNEW,
  MIRRORACTOFFSET,
  MIRRORACTPOWER,
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
					 "actPower",			\
					 "actScale",			\
					 "nacts"			\
					 )

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
  int nactsNew;
  float *actsNew;
  int actsNewSize;
  int *actControlMx;
  struct timespec nanodelay;
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
  int n,totsent=0,err=0;
  setAlpaoThreadAffinity(mirstr->threadAffinity,mirstr->threadPriority,mirstr->threadAffinElSize);
  pthread_mutex_lock(&mirstr->m);
  while(mirstr->open){
    pthread_cond_wait(&mirstr->cond,&mirstr->m);//wait for actuators.
    if(mirstr->open){
      //Now send the data...
      totsent=0;
      while(err==0 && totsent<mirstr->arrsize){
	n=sendto(mirstr->socket,&mirstr->arr[totsent],mirstr->arrsize-totsent,0,destaddr,sizeof(struct sockaddr_in));
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
int openAlpao(MirrorStruct mirstr){
  struct hostent *hp;
  if((mirstr->socket=socket(AF_INET,SOCK_DGRAM,0))<0){
    printf("Error: couldn't open alpao socket\n");
    return 1;
  }
  hp=gethostbyname(mirstr->host);
  if(!hp){
    printf("Error: Couldn't get IP address for %s\n",mirstr->host);
    close(mirstr->socket);
    mirstr->socket=0;
    return 1;
  }else{
    printf("IP: %d.%d.%d.%d\n",((unsigned char*)hp->h_addr_list[0])[0],((unsigned char*)hp->h_addr_list[0])[1],((unsigned char*)hp->h_addr_list[0])[2],((unsigned char*)hp->h_addr_list[0])[3]);
  }
  mirstr->ip=*((unsigned int*)hp->h_addr_list[0]);
  memset((char*)&mirstr->dmaddrstruct,0,sizeof(struct sockaddr_in));
  mirstr->dmaddrstruct.sin_family=AF_INET;
  mirstr->dmaddrstruct.sin_port=htons(port);


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
  mirstr->arrStr=arr;
  if(narg>5){
    mirstr->timeout=args[0];
    mirstr->port=args[1];
    mirstr->threadAffinElSize=args[2];
    mirstr->threadPriority=args[3];
    mirstr->threadAffinity=(unsigned int*)&args[4];
    mirstr->host=strndup((char*)&(args[4+args[2]]),(narg-4-args[2])*sizeof(int));
    printf("Got host %s\n",mirstr->host);
  }else{
    printf("wrong number of args - should be timeout, port, Naffin, thread priority,thread affinity[Naffin],  hostname (string)\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  mirstr->arrsize=HDRSIZE+nacts*sizeof(unsigned short);
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
