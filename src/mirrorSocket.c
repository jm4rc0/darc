/**
   The code here is used to create a shared object library, which can then be swapped around depending on which mirrors/interfaces you have in use, ie you simple rename the mirror file you want to mirror.so (or better, change the soft link), and restart the coremain.

The library is written for a specific mirror configuration - ie in multiple mirror situations, the library is written to handle multiple mirrors, not a single mirror many times.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include "rtcmirror.h"
#include <time.h>
#include <pthread.h>
#include "darc.h"
#define HDRSIZE 8 //the size of a WPU header - 4 bytes for frame no, 4 bytes for something else.


typedef enum{
  MIRRORNACTS,
  MIRRORACTMIN,
  MIRRORACTMAX,
  //MIRRORLASTACTS,
  MIRRORACTSCALE,
  MIRRORACTOFFSET,
  //Add more before this line.
  MIRRORNBUFFERVARIABLES//equal to number of entries in the enum
}MIRRORBUFFERVARIABLEINDX;
char *MIRRORPARAM[]={"nacts","actMin","actMax","actScale","actOffset"};//,"lastActs"};


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
  int nacts;
  unsigned short *arr;
  //int frameno;
  int arrsize;
  int open;
  int err;
  pthread_t threadid;
  pthread_cond_t cond;
  pthread_mutex_t m;
  int timeout;//in ms
  int fibrePort;//the port number on sl240 card.
  int socketOpened;//sl240 has been opened okay?
  int socket;
  int threadAffinity;
  int threadPriority;
  MirrorStructBuffered msb[2];
  int buf;
  int swap;
  int bufindx[MIRRORNBUFFERVARIABLES];
  circBuf *rtcActuatorBuf;
  circBuf *rtcErrorBuf;
  char *host;
  unsigned short port;
  int socketOpen;
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
    if(mirstr->socketOpened==1){
      close(mirstr->socket);
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
  int n,totsent=0,err=0;
  setThreadAffinityForDMC(mirstr->threadAffinity,mirstr->threadPriority);
  pthread_mutex_lock(&mirstr->m);
  while(mirstr->open){
    pthread_cond_wait(&mirstr->cond,&mirstr->m);//wait for actuators.
    if(mirstr->open){
      //Now send the data...
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
}

int openMirrorSocket(MirrorStruct *mirstr){
  int err=0;
  struct sockaddr_in sin;
  struct hostent *host;
  if((host=gethostbyname(mirstr->host))==NULL){
    printf("gethostbyname error\n");
    err=1;
  }else{
    mirstr->socket=socket(PF_INET,SOCK_STREAM,0);
    memcpy(&sin.sin_addr.s_addr,host->h_addr,host->h_length);
    sin.sin_family=AF_INET;
    sin.sin_port=htons(mirstr->port);
    if(connect(mirstr->socket,(struct sockaddr*)&sin,sizeof(sin))<0){
      printf("Error connecting\n");
      close(mirstr->socket);
      err=1;
    }
  }
  if(err==0)
    mirstr->socketOpen=1;
  return err;
}

/**
   Open a camera of type name.  Args are passed in a float array of size n, which can be cast if necessary.  Any state data is returned in camHandle, which should be NULL if an error arises.
   pxlbuf is the array that should hold the data. The library is free to use the user provided version, or use its own version as necessary (ie a pointer to physical memory or whatever).  It is of size npxls*sizeof(short).
   ncam is number of cameras, which is the length of arrays pxlx and pxly, which contain the dimensions for each camera.
   Name is used if a library can support more than one camera.
*/

#define RETERR mirrordofree(mirstr);*mirrorHandle=NULL;return 1;
int mirrorOpen(char *name,int narg,int *args, int nacts,void **mirrorHandle,circBuf *rtcErrorBuf,circBuf *rtcActuatorBuf,unsigned int frameno, char *buf){
  //int err;
  MirrorStruct *mirstr;
  int err;
  printf("Initialising mirrorSocket %s\n",name);
  if((*mirrorHandle=malloc(sizeof(MirrorStruct)))==NULL){
    printf("couldn't malloc mirrorHandle\n");
    return 1;
  }
  mirstr=(MirrorStruct*)*mirrorHandle;
  memset(mirstr,0,sizeof(MirrorStruct));
  mirstr->nacts=nacts;
  mirstr->rtcErrorBuf=rtcErrorBuf;
  mirstr->rtcActuatorBuf=rtcActuatorBuf;
  //array has to be a whole number of int32 for the sl240, ie multiple of 4 bytes.
  mirstr->arrsize=HDRSIZE+nacts*sizeof(unsigned short);
  if((mirstr->arr=malloc(mirstr->arrsize))==NULL){
    printf("couldn't malloc arr\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  memset(mirstr->arr,0,mirstr->arrsize);
  if(narg>4){
    mirstr->timeout=args[0];
    mirstr->port=args[1];
    mirstr->threadAffinity=args[2];
    mirstr->threadPriority=args[3];
    mirstr->host=strndup((char*)&(args[4]),(narg-4)*sizeof(int));
  }else{
    printf("wrong number of args - should be timeout, fibrePort, thread affinity, thread priority, hostname (string)\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
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
  if((err=mirrorNewParam(*mirrorHandle,buf,frameno))){
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  if(err==0)
    pthread_create(&mirstr->threadid,NULL,worker,mirstr);
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
int mirrorSend(void *mirrorHandle,int n,float *data,unsigned int frameno,double timestamp,int err){
  MirrorStruct *mirstr=(MirrorStruct*)mirrorHandle;
  //int err=0;
  int nclipped=0;
  int intDMCommand;
  MirrorStructBuffered *msb;
  unsigned short *actsSent=&(((unsigned short*)mirstr->arr)[HDRSIZE/sizeof(unsigned short)]);
  int i;
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
    for(i=0; i<mirstr->nacts; i++){
      val=data[i];
      if(msb->actScale!=NULL)
	val*=msb->actScale[i];
      if(msb->actOffset!=NULL)
	val+=msb->actOffset[i];
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
    }
    
    ((unsigned int*)mirstr->arr)[1]=frameno;
    ((unsigned int*)mirstr->arr)[0]=(0x5555<<16)|mirstr->nacts;
    //Wake up the thread.
    pthread_cond_signal(&mirstr->cond);
    pthread_mutex_unlock(&mirstr->m);
    //printf("circadd %u %g\n",frameno,timestamp);
    circAdd(mirstr->rtcActuatorBuf,actsSent,timestamp,frameno);//actsSent);
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

int mirrorNewParam(void *mirrorHandle,char *buf,unsigned int frameno){
  MirrorStruct *mirstr=(MirrorStruct*)mirrorHandle;
  int err=0;
  int got=0;
  int dim;
  int bufno;
  MirrorStructBuffered *msb;
  int *indx=mirstr->bufindx;
  MIRRORBUFFERVARIABLEINDX i;
  int j=0;
  int nbytes;
  if(mirstr==NULL || mirstr->open==0){
    printf("Mirror not open\n");
    return 1;
  }
  bufno=1-mirstr->buf;
  msb=&mirstr->msb[bufno];
  memset(indx,-1,sizeof(int)*MIRRORNBUFFERVARIABLES);

  //first run through the buffer getting the indexes of the params we need.
  while(j<NHDR && buf[j*31]!='\0' && got<MIRRORNBUFFERVARIABLES){
    if(strncmp(&buf[j*31],"nacts",31)==0){
      indx[MIRRORNACTS]=j;
      got++;
    }else if(strncmp(&buf[j*31],"actMin",31)==0){
      indx[MIRRORACTMIN]=j;
      got++;
    }else if(strncmp(&buf[j*31],"actMax",31)==0){
      indx[MIRRORACTMAX]=j;
      got++;
    }else if(strncmp(&buf[j*31],"actScale",31)==0){
      indx[MIRRORACTSCALE]=j;
      got++;
    }else if(strncmp(&buf[j*31],"actOffset",31)==0){
      indx[MIRRORACTOFFSET]=j;
      got++;
    }/*else if(strncmp(&buf[j*31],"lastActs",31)==0){
      indx[MIRRORLASTACTS]=j;
      got++;
      }*/
    j++;
  }
  for(j=0; j<MIRRORNBUFFERVARIABLES; j++){
    if(indx[j]==-1 && j!=MIRRORACTOFFSET && j!=MIRRORACTSCALE){
      //if(updateIndex){
      printf("ERROR buffer index %d\n",j);
      writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"Error in mirror parameter buffer: %s",MIRRORPARAM[j]);
      //}
      err=-1;
    }
  }
  if(err==0){
    i=MIRRORNACTS;
    if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
      if(mirstr->nacts!=*((int*)(buf+START[indx[i]]))){
	printf("Error - nacts changed - please close and reopen mirror library\n");
	err=MIRRORNACTS;
      }
    }else{
      printf("mirrornacts error\n");
      writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"mirrornacts error");
      err=MIRRORNACTS;
    }
    if(mirstr->rtcActuatorBuf!=NULL && mirstr->rtcActuatorBuf->datasize!=mirstr->nacts*sizeof(unsigned short)){
      dim=mirstr->nacts;
      if(circReshape(mirstr->rtcActuatorBuf,1,&dim,'H')!=0){
	printf("Error reshaping rtcActuatorBuf\n");
      }
    }
    i=MIRRORACTMIN;
    if(buf[NHDR*31+indx[i]]=='H' && NBYTES[indx[i]]==sizeof(unsigned short)*mirstr->nacts){
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
	memcpy(msb->actMin,buf+START[indx[i]],
	       sizeof(unsigned short)*mirstr->nacts);
    }else{
      printf("mirrorActMin error\n");
      writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"mirrorActMin error");
      err=MIRRORACTMIN;
    }
    i=MIRRORACTMAX;
    if(buf[NHDR*31+indx[i]]=='H' && NBYTES[indx[i]]==sizeof(unsigned short)*mirstr->nacts){
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
	memcpy(msb->actMax,buf+START[indx[i]],
	       sizeof(unsigned short)*mirstr->nacts);
    }else{
      printf("mirrorActMax error\n");
      writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"mirrorActMax error");
      err=MIRRORACTMAX;
    }
    i=MIRRORACTSCALE;
    if(indx[i]>=0){//parameter is in the buf.
      nbytes=NBYTES[indx[i]];
      if(nbytes==0){
	msb->actScale=NULL;
      }else if(buf[NHDR*31+indx[i]]=='f' && nbytes==sizeof(float)*mirstr->nacts){
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

	  memcpy(msb->actScale,buf+START[indx[i]],sizeof(float)*mirstr->nacts);
      }else{
	printf("actScale error\n");
	writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"actScale error");
	err=MIRRORACTSCALE;
	msb->actScale=NULL;
      }
    }else{
      msb->actScale=NULL;
    }
    i=MIRRORACTOFFSET;
    if(indx[i]>=0){//parameter is in the buf.
      nbytes=NBYTES[indx[i]];
      if(nbytes==0){
	msb->actOffset=NULL;
      }else if(buf[NHDR*31+indx[i]]=='f' && nbytes==sizeof(float)*mirstr->nacts){
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

	  memcpy(msb->actOffset,buf+START[indx[i]],sizeof(float)*mirstr->nacts);
      }else{
	printf("actOffset error\n");
	writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"actOffset error");
	err=MIRRORACTOFFSET;
	msb->actOffset=NULL;
      }
    }else{
      msb->actOffset=NULL;
    }
    /*i=MIRRORLASTACTS;
    if(NBYTES[indx[i]]==0){
      msb->lastActs=NULL;
    }else if(buf[NHDR*31+indx[i]]=='H' && NBYTES[indx[i]]==sizeof(unsigned short)*mirstr->nacts){
      msb->lastActs=((unsigned short*)(buf+START[indx[i]]));
    }else{
      msb->lastActs=NULL;
      printf("lastActs error\n");
      writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"lastActs error");
      }*/
  }
  pthread_mutex_lock(&mirstr->m);
  mirstr->swap=1;
  pthread_mutex_unlock(&mirstr->m);
  return err;
}
