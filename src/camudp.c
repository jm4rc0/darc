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
   The code here is used to create a shared object library, which can then be swapped around depending on which cameras you have in use, ie you simple rename the camera file you want to camera.so (or better, change the soft link), and restart the coremain.

The library is written for a specific camera configuration - ie in multiple camera situations, the library is written to handle multiple cameras, not a single camera many times.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include "darc.h"
#include "rtccamera.h"
#include "qsort.h"
#include "buffer.h"

#define UDP_DATA_PORT 0x6000
//we use 4 buffers (instead of double buffering).
#define NBUF 4
#define BUFMASK 0x3
/**
   The struct to hold info.
*/


typedef struct{
  int ncam;//no of cameras
  int npxls;//number of pixels in the frame (total for all cameras)
  volatile int transferframe;//the frame currently being passed into the RTC (piecewise)
  volatile int last;//the last frame past to the RTC
  volatile int latest;//the latest whole frame arrived
  volatile int curframe;//the current frame
  volatile int *pxlcnt;//number of pixels received for this frame and buffer
  //int pxlsRequested;//number of pixels requested by DMA for this frame
  pthread_mutex_t m;
  pthread_cond_t *cond;//sync between main RTC
  pthread_cond_t *cond2;//sync between main RTC
  pthread_cond_t thrcond;//sync between threads
  unsigned short **DMAbuf;//a buffer for receiving the DMA - NBUF*(sizeof(short)*npxls).  If a DMA requires a specific word alignment, we may need to reconsider this...
  int open;//set by RTC if the camera is open
  volatile int *waiting;//set by RTC if the RTC is waiting for pixels
  volatile int newframeAll;//set by RTC when a new frame is starting to be requested.
  volatile int *newframe;
  //int transferRequired;
  //int frameno;
  unsigned int thisiter;
  unsigned short *imgdata;
  int *pxlsTransferred;//number of pixels copied into the RTC memory.
  pthread_t *threadid;
  int *port;//the port number to bind to
  unsigned int *userFrameNo;//pointer to the RTC frame number... to be updated for new frame.
  int *setFrameNo;//tells thread to set the userFrameNo.
  void *thrStruct;//pointer to an array of threadStructs.
  int *npxlsArr;
  int *npxlsArrCum;
  int thrcnt;
  int *err;
  int *threadPriority;
  unsigned int *threadAffinity;
  int threadAffinElSize;
  int *reorder;//is pixel reordering required, and if so, which pattern?
  int **reorderBuf;//pixels for reordering.
  int *reorderIndx;
  int *reorderno;
  int nReorders;
  char *paramNames;
  int *index;
  int *nbytes;
  void **values;
  char *dtype;
  int *ntoread;//number of frames to read this iteration - usually 1, unless frame numbers are different (ie cameras have become out of sync).
  int resync;//if set, will attempt to resynchronise cameras that have different frame numbers, by reading more frames from this one (number of extra frames is equal to the value of resync
  int wpuCorrection;//whether to apply the correction if a camera is missing frames occasionally.
  int *readStarted;
  int *gotFirstPacket;//flag to whether syncdv has already been received while reading a truncated frame.
  int skipFrameAfterBad;//flag - whether to skip a frame after a bad frame.
  int testLastPixel;//value - if nonzero, and one of the last this many pixels pixel are non-zero, flags as a bad frame.  Assumes that at least one subap will require all ccd pixels to be read (set in the config file - though this may increase latency, if not all pixels required).
  int pxlRowStartSkipThreshold;//If a pixel at the start of a row falls below this threshold, then this pixel is discarded - meaning that all future pixels are shifted one to the left.  Any required padding will take this value.
  int pxlRowEndInsertThreshold;//If a pixel at the end of a row falls below this threshold, then an extra pixel is inserted here - meaning that all future pixels are shifted one to the right.
  int *pxlShift;//the total shift of pixels (-1 for pixel removed, +1 for pixel inserted).
  int *pxlx;
  circBuf *rtcErrorBuf;
  int *frameReady;
  int *sock;
  int *frameCnt;
  int *host;
}CamStruct;

typedef struct{
  CamStruct *camstr;
  int camNo;
}ThreadStruct;


void safefree(void *ptr){
  if(ptr!=NULL)
    free(ptr);
}

void dofree(CamStruct *camstr){
  int i;
  printf("dofree called\n");
  if(camstr!=NULL){
    if(camstr->DMAbuf!=NULL){
      for(i=0; i<camstr->ncam; i++){
	if(camstr->DMAbuf[i]!=NULL)
	  free(camstr->DMAbuf[i]);
      }
      free(camstr->DMAbuf);
    }
    for(i=0; i<camstr->ncam; i++){
      if(camstr->sock[i]>0)
	close(camstr->sock[i]);
      camstr->sock[i]=0;
      pthread_cond_destroy(&camstr->cond[i]);
      pthread_cond_destroy(&camstr->cond2[i]);
    }
    pthread_cond_destroy(&camstr->thrcond);
    pthread_mutex_destroy(&camstr->m);
    safefree(camstr->npxlsArr);
    safefree(camstr->npxlsArrCum);
    safefree((void*)camstr->pxlcnt);
    safefree(camstr->ntoread);
    safefree(camstr->readStarted);
    safefree((void*)camstr->waiting);
    safefree((void*)camstr->newframe);
    safefree(camstr->pxlsTransferred);
    safefree(camstr->setFrameNo);
    safefree(camstr->port);
    safefree(camstr->thrStruct);
    safefree(camstr->threadid);
    safefree(camstr->err);
    safefree(camstr->threadPriority);
    safefree(camstr->threadAffinity);
    safefree(camstr->reorder);
    safefree(camstr->reorderBuf);
    safefree(camstr->reorderno);
    safefree(camstr->reorderIndx);
    safefree(camstr->index);
    safefree(camstr->paramNames);
    safefree(camstr->nbytes);
    safefree(camstr->dtype);
    safefree(camstr->values);
    safefree(camstr->gotFirstPacket);
    safefree(camstr->pxlShift);
    safefree(camstr->pxlx);
    safefree(camstr->sock);
    safefree(camstr->frameCnt);
    safefree(camstr->host);
    free(camstr);
  }
}


int camSetThreadAffinityAndPriority(unsigned int *threadAffinity,int threadPriority,int threadAffinElSize){
  int i;
  cpu_set_t mask;
  int ncpu;
  struct sched_param param;
  printf("Getting CPUs\n");
  ncpu= sysconf(_SC_NPROCESSORS_ONLN);
  printf("Got %d CPUs\n",ncpu);
  CPU_ZERO(&mask);
  printf("Setting %d CPUs\n",ncpu);
  for(i=0; i<ncpu && i<threadAffinElSize*32; i++){
    if(((threadAffinity[i/32])>>(i%32))&1){
      CPU_SET(i,&mask);
    }
  }
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
   The threads that does the work...
   One per camera interface...
*/
void* worker(void *thrstrv){
  ThreadStruct *thrstr=(ThreadStruct*)thrstrv;
  CamStruct *camstr=thrstr->camstr;
  int cam=thrstr->camNo;
  int err,i;
  int nRead;
  char timebuf[80];
  time_t tval;
  char *udpBuf,*tmpc;
  struct sockaddr_in sendAddr;
  socklen_t otherAddrLen;
  int totLen,recvLen=0;
  unsigned short htonsUDP_DATA_PORT=htons(UDP_DATA_PORT);
  unsigned short sourcePort;
  int frameBytes=camstr->npxlsArr[cam]*sizeof(unsigned short);
  printf("Calling camSetThreadAffinityAndPriority\n");
  camSetThreadAffinityAndPriority(&camstr->threadAffinity[cam*camstr->threadAffinElSize],camstr->threadPriority[cam],camstr->threadAffinElSize);
  pthread_mutex_lock(&camstr->m);
  //Cameras are assumed to be synced, so always doing same frame. - actually, this may no longer be true (subkect to testing) - we may now be able to recover... uses ntoread...
  camstr->ntoread[cam]=1;
  if(camstr->thrcnt==0){//first camera
    camstr->transferframe=0;
    camstr->last=0;
    camstr->latest=0;
    camstr->curframe=0;
  }
  otherAddrLen=sizeof(sendAddr);
  //do we need to clear a buffer?  Prob not.
  while(camstr->open){//thread initialised...
    if(camstr->thrcnt==0){//first camera this frame...
      //camstr->thrcnt++;
      camstr->curframe++;
      for(i=0; i<camstr->ncam; i++){
	camstr->pxlcnt[NBUF*i+(camstr->curframe&BUFMASK)]=0;
      }
    }
    camstr->thrcnt++;
    //camstr->pxlsRequested=0;
    err=0;
    nRead=camstr->ntoread[cam];
    camstr->ntoread[cam]=1;
    if(nRead!=1)
      printf("nRead %d for cam %d\n",nRead,cam);
    camstr->readStarted[cam]=0;
    pthread_mutex_unlock(&camstr->m);
    while(err==0 && nRead>0){
      nRead--;
      totLen=0;//total bytes received for this frame
      //Here, we probably want to:
      //Read a UDP packet into its in-order position.  If it has arrived out of order, move it to the correct position.  For now, just assume it always arrives in order (of course check, but don't have code that corrects).
      //Store its frame number in a frameReceived array.
      //Then check the frameReceived array - if all have the same frame number, then the frame is finished, and set the frame finished flag.
      //If the frame number is different than currentFrameNo, and frameFinished flag is not set, then this is an error, and we need to flag it as such.  
      //But, what if frame rate is such that pixels are continuously arriving, with little/no gap between frames - could quite easily get udp packets out of order - in which case would be discarding frames unnecessarily.  Consider this if the problem arises, but not for now.
      //Read the start of frame...
      udpBuf=(char*)&camstr->DMAbuf[cam][(camstr->curframe&BUFMASK)*camstr->npxlsArr[cam]];
      if(camstr->gotFirstPacket[cam]){//already received the first packet (partial frame previous time) - so data already in the buffer.
	camstr->gotFirstPacket[cam]=0;
	//recvLen will be set correctly from previous frame.
      }else{
	while((sendAddr.sin_port!=htonsUDP_DATA_PORT) && (err==0)){
	  recvLen=recvfrom(camstr->sock[cam],udpBuf,camstr->npxlsArr[cam]*sizeof(short),0,(struct sockaddr*)&sendAddr,&otherAddrLen);
	  if(recvLen<0){
	    printf("UDP receiving error for cam %d\n",cam);
	    err=1;
	  }
	}
      }
      //have now received the first packet okay.
      if(err==0){
	if(nRead==0)
	  camstr->readStarted[cam]=1;
	camstr->frameCnt[NBUF*cam+(camstr->curframe&BUFMASK)]++;
	sourcePort=UDP_DATA_PORT;
	totLen+=recvLen;
	//now loop until read all the pixels.
	while(totLen<frameBytes && err==0){
	  sourcePort++;
	  recvLen=recvfrom(camstr->sock[cam],&udpBuf[totLen],camstr->npxlsArr[cam]*sizeof(unsigned short)-totLen,0,(struct sockaddr*)&sendAddr,&otherAddrLen);
	  if(recvLen<0){
	    printf("UDP receiving error for cam %d after %d bytes\n",cam,totLen);
	    err=1;
	  }else if(sendAddr.sin_port==htons(sourcePort)){//got some data ok
	    totLen+=recvLen;
	    if(nRead==0){//have just read part of the frame that is to be sent - so send data to the rtc.
	      pthread_mutex_lock(&camstr->m);
	      camstr->err[NBUF*cam+(camstr->curframe&BUFMASK)]=err;
	      camstr->pxlcnt[NBUF*cam+(camstr->curframe&BUFMASK)]=totLen/2;
	      if(camstr->waiting[cam]==1){
		//rtc waiting for pixels, so wake it up
		camstr->waiting[cam]=0;
		pthread_cond_broadcast(&camstr->cond[cam]);//signal should do.
	      }
	      pthread_mutex_unlock(&camstr->m);
		
	    }
	  }else{//skipped frame.
	    printf("cam %d missing packets (frame %d) - expected %#x, got %#x\n",cam,camstr->frameCnt[cam],(int)sourcePort,(int)ntohs(sendAddr.sin_port));
	    err=1;
	    if(sendAddr.sin_port==htonsUDP_DATA_PORT){
	      printf("Starting packet received\n");
	      //copy the new data into the next buffer, ready for next time
	      camstr->gotFirstPacket[cam]=1;
	      tmpc=(char*)&camstr->DMAbuf[cam][((camstr->curframe+1)&BUFMASK)*camstr->npxlsArr[cam]];
	      memcpy(tmpc,&udpBuf[totLen],recvLen);
	    }
	  }
	}
      }else{//error getting start of frame
	memset(udpBuf,0,sizeof(unsigned short)*camstr->npxlsArr[cam]);
	//camstr->frameCnt[NBUF*cam+(camstr->curframe&BUFMASK)]=0;
      }
    }
    pthread_mutex_lock(&camstr->m);
    if(err==0 && camstr->wpuCorrection!=0){//see whetehr other cams have started their frames - if not, treat as mismatched frame - which means an extra read for this camera.
      for(i=0;i<camstr->ncam;i++){
	if(camstr->readStarted[i]==0){//hasn't started the frame yet...
	  tval=time(NULL);
	  strftime(timebuf,80,"%y/%m/%d %H:%M:%S",localtime(&tval));
	  printf("Read of camera %d not yet started - probably frame missing (%s)\n",i,timebuf);
	  camstr->ntoread[cam]++;//this camera needs to read extra frame next time.
	  break;
	}
      }
    }
    camstr->err[NBUF*cam+(camstr->curframe&BUFMASK)]=err;
    if(err && camstr->waiting[cam]){//the rtc is waiting for newest pixels, so wake it up but an error has occurred.
      camstr->waiting[cam]=0;
      pthread_cond_broadcast(&camstr->cond[cam]);
    }
    camstr->thrcnt++;
    if(camstr->thrcnt==camstr->ncam*2){//this is the last thread... so wake the others up... threads increment this once at start, and once at end of each frame
      camstr->latest=camstr->curframe;
      //camstr->curframe++;
      camstr->thrcnt=0;
      if(camstr->skipFrameAfterBad>0){
	int readExtra=0;
	for(i=0; i<camstr->ncam; i++){
	  readExtra|=camstr->gotFirstPacket[i];//set if a partial frame received.
	}
	if(readExtra){
	  for(i=0; i<camstr->ncam; i++){
	    camstr->ntoread[i]+=camstr->skipFrameAfterBad;
	  }
	}
      }
      pthread_cond_broadcast(&camstr->thrcond);
    }else{
      //Threads should all wait here until all completed this frame...
      pthread_cond_wait(&camstr->thrcond,&camstr->m);
    }
    
  }
  pthread_mutex_unlock(&camstr->m);
  return 0;
}
      


/**
   Open a camera of type name.  Args are passed in a int32 array of size n, which can be cast if necessary.  Any state data is returned in camHandle, which should be NULL if an error arises.
   pxlbuf is the array that should hold the data. The library is free to use the user provided version, or use its own version as necessary (ie a pointer to physical memory or whatever).  It is of size npxls*sizeof(short).
   ncam is number of cameras, which is the length of arrays pxlx and pxly, which contain the dimensions for each camera.  Currently, ncam must equal 1.
   Name is used if a library can support more than one camera.
   frameno is a pointer to an array with a value for each camera in which the frame number should be placed.



*/


#define TEST(a) if((a)==NULL){printf("calloc error\n");dofree(camstr);*camHandle=NULL;return 1;}

int camOpen(char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **camHandle,int nthreads,unsigned int frameno,unsigned int **camframeno,int *camframenoSize,int npxls,int ncam,int *pxlx,int* pxly){
  CamStruct *camstr;
  int i,ngot,j,k;
  unsigned short *tmps;
  char **reorderCC;
  struct sockaddr_in servAddr;
  printf("Initialising camera %s\n",name);
  if((*camHandle=malloc(sizeof(CamStruct)))==NULL){
    printf("Couldn't malloc camera handle\n");
    return 1;
  }
  printf("Malloced camstr\n");
  memset(*camHandle,0,sizeof(CamStruct));
  camstr=(CamStruct*)*camHandle;
  if(arr->pxlbuftype!='H' || arr->pxlbufsSize!=sizeof(unsigned short)*npxls){
    //need to resize the pxlbufs...
    arr->pxlbufsSize=sizeof(unsigned short)*npxls;
    arr->pxlbuftype='H';
    arr->pxlbufelsize=sizeof(unsigned short);
    tmps=realloc(arr->pxlbufs,arr->pxlbufsSize);
    if(tmps==NULL){
      if(arr->pxlbufs!=NULL)
	free(arr->pxlbufs);
      printf("pxlbuf malloc error in camfile.\n");
      arr->pxlbufsSize=0;
      free(*camHandle);
      *camHandle=NULL;
      return 1;
    }
    arr->pxlbufs=tmps;
    memset(arr->pxlbufs,0,arr->pxlbufsSize);
  }
  if(n>0)
    camstr->threadAffinElSize=args[0];
  camstr->imgdata=arr->pxlbufs;
  //camstr->frameno=frameno;
  if(*camframenoSize<ncam){
    if(*camframeno!=NULL)
      free(*camframeno);
    if((*camframeno=malloc(sizeof(unsigned int)*ncam))==NULL){
      printf("Couldn't malloc camframeno\n");
      *camframenoSize=0;
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }else{
      *camframenoSize=ncam;
    }
  }
  camstr->userFrameNo=*camframeno;
  camstr->ncam=ncam;
  camstr->rtcErrorBuf=rtcErrorBuf;
  camstr->npxls=npxls;//*pxlx * *pxly;
  TEST(camstr->npxlsArr=calloc(ncam,sizeof(int)));
  TEST(camstr->npxlsArrCum=calloc((ncam+1),sizeof(int)));
  TEST(camstr->pxlcnt=calloc(ncam*NBUF,sizeof(int)));
  TEST(camstr->frameCnt=calloc(ncam*NBUF,sizeof(unsigned int)));
  TEST(camstr->ntoread=calloc(ncam,sizeof(int)));
  TEST(camstr->readStarted=calloc(ncam,sizeof(int)));
  TEST(camstr->waiting=calloc(ncam,sizeof(int)));
  TEST(camstr->newframe=calloc(ncam,sizeof(int)));
  TEST(camstr->pxlsTransferred=calloc(ncam,sizeof(int)));
  TEST(camstr->setFrameNo=calloc(ncam,sizeof(int)));
  TEST(camstr->port=calloc(ncam,sizeof(int)));
  TEST(camstr->err=calloc(ncam*NBUF,sizeof(int)));
  TEST(camstr->thrStruct=calloc(ncam,sizeof(ThreadStruct)));
  TEST(camstr->threadid=calloc(ncam,sizeof(pthread_t)));
  TEST(camstr->cond=calloc(ncam,sizeof(pthread_cond_t)));
  TEST(camstr->cond2=calloc(ncam,sizeof(pthread_cond_t)));
  TEST(camstr->threadAffinity=calloc(ncam*camstr->threadAffinElSize,sizeof(int)));
  TEST(camstr->threadPriority=calloc(ncam,sizeof(int)));
  TEST(camstr->reorder=calloc(ncam,sizeof(int)));
  TEST(camstr->reorderno=calloc(ncam,sizeof(int)));
  TEST(camstr->reorderIndx=calloc(ncam,sizeof(int)));
  TEST(camstr->reorderBuf=calloc(ncam,sizeof(int*)));
  TEST(camstr->gotFirstPacket=calloc(ncam,sizeof(int)));
  TEST(camstr->pxlShift=calloc(ncam*2,sizeof(int)));
  TEST(camstr->pxlx=calloc(ncam,sizeof(int)));
  TEST(camstr->frameReady=calloc(ncam,sizeof(int)));
  TEST(camstr->sock=calloc(ncam,sizeof(int)));
  TEST(camstr->host=calloc(ncam,sizeof(int)));

  camstr->npxlsArrCum[0]=0;
  printf("malloced things\n");
  for(i=0; i<ncam; i++){
    camstr->pxlx[i]=pxlx[i];
    camstr->npxlsArr[i]=pxlx[i]*pxly[i];
    camstr->npxlsArrCum[i+1]=camstr->npxlsArrCum[i]+camstr->npxlsArr[i];
  }
  if(n>=(4+args[0])*ncam+1 && n<=(4+args[0])*ncam+7){
    int j;
    for(i=0; i<ncam; i++){
      camstr->port[i]=args[i*(4+args[0])+1];//host
      camstr->port[i]=args[i*(4+args[0])+2];// port
      camstr->threadPriority[i]=args[i*(4+args[0])+3];//thread priority
      camstr->reorder[i]=args[i*(4+args[0])+4];//reorder pixels
      for(j=0;j<args[0];j++)
	camstr->threadAffinity[i*args[0]+j]=((unsigned int*)&args[i*(4+args[0])+5])[j];//thread affinity
    }
    if(n>=(4+args[0])*ncam+2){
      camstr->resync=args[(4+args[0])*ncam+1];
    }else{
      camstr->resync=10;
    }
    if(n>=(4+args[0])*ncam+3){
      camstr->wpuCorrection=args[(4+args[0])*ncam+2];
    }else{
      camstr->wpuCorrection=0;
    }
    if(n>=(4+args[0])*ncam+4){
      camstr->skipFrameAfterBad=args[(4+args[0])*ncam+3];
      printf("skipFrameAfterBad %d\n",camstr->skipFrameAfterBad);
    }else{
      camstr->skipFrameAfterBad=0;
    }
    if(n>=(4+args[0])*ncam+5){
      camstr->testLastPixel=args[(4+args[0])*ncam+4];
      printf("testLastPixel %d\n",camstr->testLastPixel);
    }else{
      camstr->testLastPixel=0;
    }
    if(n>=(4+args[0])*ncam+6){
      camstr->pxlRowStartSkipThreshold=args[(4+args[0])*ncam+5];
      printf("pxlRowStartSkipThreshold %d\n",camstr->pxlRowStartSkipThreshold);
    }else{
      camstr->pxlRowStartSkipThreshold=0;
    }
    if(n>=(4+args[0])*ncam+7){
      camstr->pxlRowEndInsertThreshold=args[(4+args[0])*ncam+6];
      printf("pxlRowEndInsertThreshold %d\n",camstr->pxlRowEndInsertThreshold);
    }else{
      camstr->pxlRowEndInsertThreshold=0;
    }
  }else{
    printf("wrong number of cmd args, should be Naffin,host, udpport, thread priority, reorder, thread affinity[Naffin]),( host,udpport,...) for each camera (ie (5+args[0])*ncam) + optional value, resync, equal to max number of frames to try to resync cameras with, plus other optional value wpuCorrection - whether to read extra frame if the WPU cameras get out of sync (ie if a camera doesn't produce a frame occasionally), and another optional flag, whether to skip a frame after a bad frame, and another optional flag - test last pixel (if non-zero, flags as a bad frame), and 2 more optional flags, pxlRowStartSkipThreshold, pxlRowEndInsertThreshold if doing a WPU correction based on dark column detection.\n");
    dofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  printf("got args\n");
  for(i=0; i<ncam; i++){
    printf("%d\n",camstr->port[i]);
  }
  //now need to prepare the parameter buffer names.
  ngot=0;
  for(i=0; i<ncam; i++){
    if(camstr->reorder[i]!=0){
      for(j=0;j<ngot;j++){//have we already got this reorder?
	if(camstr->reorderno[j]==camstr->reorder[i]){
	  break;
	}
      }
      if(j==ngot){//a new entry
	camstr->reorderno[j]=camstr->reorder[i];
	ngot++;
      }
    }
  }
  memset(camstr->reorderIndx,-1,sizeof(int)*ncam);
  if(ngot>0){
    TEST(reorderCC=calloc(ncam,sizeof(char*)));
    for(i=0; i<ngot; i++){
      reorderCC[i]=malloc(BUFNAMESIZE);
      snprintf(reorderCC[i],16,"camReorder%d",camstr->reorderno[i]);
    }
    //Now sort them...
#define islt(a,b) (strcmp((*a),(*b))<0)
    QSORT(char*,reorderCC,ngot,islt);
#undef islt
    //now capture the order
    for(i=0; i<ngot; i++){
      j=atoi(&reorderCC[i][10]);
      for(k=0;k<ncam;k++){
	if(camstr->reorder[k]==j){
	  camstr->reorderIndx[k]=i;
	}
      }
    }
    //now make the parameter buffer
    if((camstr->paramNames=calloc(ngot,BUFNAMESIZE))==NULL){
      printf("Failed to mallocparamNames in camudp\n");
      dofree(camstr);
      *camHandle=NULL;
      for(i=0; i<ngot; i++)
	free(reorderCC[i]);
      free(reorderCC);
      return 1;
    }
    for(i=0; i<ngot; i++){
      memcpy(&camstr->paramNames[i*BUFNAMESIZE],reorderCC[i],BUFNAMESIZE);
      printf("%16s\n",&camstr->paramNames[i*BUFNAMESIZE]);
      free(reorderCC[i]);
    }
    free(reorderCC);
    TEST(camstr->index=calloc(sizeof(int),ngot));
    TEST(camstr->values=calloc(sizeof(void*),ngot));
    TEST(camstr->dtype=calloc(sizeof(char),ngot));
    TEST(camstr->nbytes=calloc(sizeof(int),ngot));
  }
  camstr->nReorders=ngot;
  for(i=0; i<ncam; i++){
    if(pthread_cond_init(&camstr->cond[i],NULL)!=0){
      printf("Error initialising condition variable %d\n",i);
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }
    if(pthread_cond_init(&camstr->cond2[i],NULL)!=0){
      printf("Error initialising condition variable2 %d\n",i);
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }
  }
  if(pthread_cond_init(&camstr->thrcond,NULL)!=0){
    printf("Error initialising thread condition variable\n");
    dofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  //maybe think about having one per camera???
  if(pthread_mutex_init(&camstr->m,NULL)!=0){
    printf("Error initialising mutex variable\n");
    dofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  printf("done mutex\n");
  if((camstr->DMAbuf=malloc(ncam*sizeof(unsigned short*)))==NULL){
    printf("Couldn't allocate DMA buffer\n");
    dofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  printf("memset dmabuf\n");
  memset(camstr->DMAbuf,0,sizeof(unsigned short*)*ncam);
  printf("doingf dmabuf\n");
  for(i=0; i<ncam; i++){
    if((camstr->DMAbuf[i]=malloc((sizeof(unsigned short)*camstr->npxlsArr[i])*NBUF))==NULL){
      printf("Couldn't allocate DMA buffer %d\n",i);
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }
    printf("memset dmabuf...\n");
    memset(camstr->DMAbuf[i],0,(sizeof(unsigned short)*camstr->npxlsArr[i])*NBUF);
  }
  printf("done dmabuf\n");

  if(camNewParam(*camHandle,pbuf,frameno,arr)!=0){
    printf("Error in camOpen->newParam...\n");
    dofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  printf("Reorders:\n");
  for(i=0; i<ncam; i++)
    printf("%d %p\n",camstr->reorder[i],camstr->reorderBuf[i]);
  //Now do the UDP stuff...
  for(i=0; i<ncam; i++){
    if((camstr->sock[i]=socket(AF_INET, SOCK_DGRAM, 0))<0){
      printf("Cannot open socket for cam %d\n",i);
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }
    /* bind local server port */
    memset(&servAddr,0,sizeof(servAddr));
    servAddr.sin_family = AF_INET;
    servAddr.sin_addr.s_addr = camstr->host[i];//host is ip address as int in network order (python: numpy.fromstring(socket.inet_aton("10.0.1.2"),dtype=numpy.int32) for example
    servAddr.sin_port = htons((short)camstr->port[i]);
    if(bind (camstr->sock[i], (struct sockaddr *) &servAddr,sizeof(servAddr))<0){
      printf("Cannot bind port number %d (cam %d) \n",camstr->port[i],i);
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }
    printf("Cam %d bound to port %d\n",i,camstr->port[i]);
  }
  printf("done binding\n");
  camstr->open=1;
  for(i=0; i<ncam; i++){
    ((ThreadStruct*)camstr->thrStruct)[i].camNo=i;
    ((ThreadStruct*)camstr->thrStruct)[i].camstr=camstr;
    pthread_create(&camstr->threadid[i],NULL,worker,&((ThreadStruct*)camstr->thrStruct)[i]);
  }
  printf("created threads (%d)\n",ncam);

  return 0;
}


/**
   Close a camera of type name.  Args are passed in the int32 array of size n, and state data is in camHandle, which should be freed and set to NULL before returning.
*/
int camClose(void **camHandle){
  CamStruct *camstr;
  int i;
  printf("Closing camera\n");
  if(*camHandle==NULL)
    return 1;
  camstr=(CamStruct*)*camHandle;
  pthread_mutex_lock(&camstr->m);
  camstr->open=0;
  for(i=0; i<camstr->ncam; i++){
    pthread_cond_broadcast(&camstr->cond[i]);
  }
  pthread_mutex_unlock(&camstr->m);
  for(i=0; i<camstr->ncam; i++){
    pthread_join(camstr->threadid[i],NULL);//wait for worker thread to complete
  }
  dofree(camstr);
  *camHandle=NULL;
  printf("Camera closed\n");
  return 0;
}
/**
   New parameters in the buffer (optional)...
*/
int camNewParam(void *camHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr){
  //the only param needed is camReorder if reorder!=0.
  int i,j;
  CamStruct *camstr=(CamStruct*)camHandle;
  int nfound,err=0;
  if(camstr->nReorders>0){
    nfound=bufferGetIndex(pbuf,camstr->nReorders,camstr->paramNames,camstr->index,camstr->values,camstr->dtype,camstr->nbytes);
    memset(camstr->reorderBuf,0,camstr->ncam*sizeof(int*));
    for(i=0; i<camstr->nReorders; i++){
      if(camstr->index[i]>=0 && camstr->nbytes[i]>0){
	if(camstr->dtype[i]=='i'){
	  //for which camera(s) is this?
	  for(j=0; j<camstr->ncam; j++){
	    if(camstr->reorderIndx[j]==i){//a reorder for this camera
	      if(camstr->nbytes[i]==sizeof(int)*camstr->npxlsArr[j]){
		camstr->reorderBuf[j]=(int*)camstr->values[i];
	      }else{
		printf("Wrong size for camReorder\n");
		err=1;
	      }
	    }
	  }
	}else{
	  printf("Wrong dtype for camReorder\n");
	  err=1;
	}
	
      }
    }
  }
  return err;
}


/**
   Called when we're starting processing the next frame.  This doesn't actually wait for any pixels.
*/
int camNewFrameSync(void *camHandle,unsigned int thisiter,double starttime){
  //printf("camNewFrame\n");
  CamStruct *camstr;
  int i;
  int maxf;
  int extratoread;
  camstr=(CamStruct*)camHandle;
  if(camHandle==NULL){// || camstr->framing==0){
    //printf("called camNewFrame with camHandle==NULL\n");
    return 1;
  }
  pthread_mutex_lock(&camstr->m);
  camstr->thisiter=thisiter;
  //printf("New frame\n");
  camstr->newframeAll=1;
  for(i=0;i<camstr->ncam; i++)
    camstr->newframe[i]=1;
  camstr->last=camstr->transferframe;
  //printf("newframe\n");
  if(camstr->resync){//want to try to resynchronise cameras if get out of sync.
    //now check all frame numbers are equal - if not, read extra frames until they are.
    maxf=0;//first get the max frame number
    for(i=0; i<camstr->ncam; i++){
      if(camstr->userFrameNo[i]>maxf)
	maxf=camstr->userFrameNo[i];
    }
    //now work out how many extra frames are required.
    for(i=0; i<camstr->ncam; i++){
      if(camstr->userFrameNo[i]<maxf){
	extratoread=maxf-camstr->userFrameNo[i];
	if(extratoread>camstr->resync)
	  extratoread=camstr->resync;
	printf("Out of sync (%d %d): Camera %d reading %d frames extra\n",maxf,camstr->userFrameNo[i],i,extratoread);
	camstr->ntoread[i]+=extratoread;
      }
    }
  }
  pthread_mutex_unlock(&camstr->m);
  return 0;
}

/**
   Wait for the next n pixels of the current frame to arrive from camera cam.
   Note - this can be called by multiple threads at same time.  Need to make sure its thread safe.

*/
int camWaitPixels(int n,int cam,void *camHandle){
  //printf("camWaitPixels %d, camera %d\n",n,cam);
  CamStruct *camstr=(CamStruct*)camHandle;
  int rt=0;
  int i,j;
  //static struct timeval t1;
  //struct timeval t2;
  //struct timeval t3;
  //printf("camWaitPixels %d %d\n",n,cam);
  if(camHandle==NULL){//L || camstr->framing==0){
    //printf("called camWaitPixels with camHandle==NULL\n");
    return 1;
  }
  if(n<0)
    n=0;
  if(n>camstr->npxlsArr[cam])
    n=camstr->npxlsArr[cam];
  //printf("camWaitPixels\n");
  pthread_mutex_lock(&camstr->m);
  //printf("camWaitPixels got mutex, newframe=%d\n",camstr->newframe[cam]);
  if(camstr->newframe[cam]){//first thread for this camera after new frame...
    camstr->newframe[cam]=0;
    camstr->frameReady[cam]=0;
    camstr->pxlShift[cam*2]=0;
    camstr->pxlShift[cam*2+1]=0;
    camstr->setFrameNo[cam]=1;
    camstr->pxlsTransferred[cam]=0;
    while(camstr->last==camstr->curframe){
      //RTC has finished with the current frame - so wait until it has been read fully, and we've started reading the next frame.
      camstr->waiting[cam]=1;
      //printf("Waiting with last %d curframe %d (cam %d)\n",camstr->last,camstr->curframe,cam);
      pthread_cond_wait(&camstr->cond[cam],&camstr->m);
      //printf("NOW NOT Waiting with last %d curframe %d (cam %d)\n",camstr->last,camstr->curframe,cam);
      
    }
    if(camstr->newframeAll){//first thread overall after new frame started
      if(camstr->latest==camstr->last){//the latest whole frame received from the camera is the one already passed to the RTC.
	camstr->transferframe=camstr->curframe;
      }else{//use the most recently arrived whole frame...
	camstr->transferframe=camstr->latest;
      }
      camstr->newframeAll=0;
    }
    camstr->frameReady[cam]=1;
    pthread_cond_broadcast(&camstr->cond2[cam]);
  }else{
    //need to wait until camstr->last!=camstr->curframe, i.e. a new frame has started to be read.  Infact, wait until transferframe has been set.
    //This arises because multiple threads per camera can call camWaitPixels at same time, and while the first one to do this is waiting for the next frame to start, others can get here.  So, they must block.
    while(camstr->frameReady[cam]==0){
      pthread_cond_wait(&camstr->cond2[cam],&camstr->m);
    }
  }
  //if((cam==0 && n==30641) || (cam==1 && n==15320))
  //  printf("wait pixels cam %d n %d last %d latest %d curframe %d transferframe %d\n",cam,n,camstr->last,camstr->latest,camstr->curframe,camstr->transferframe);
  if(camstr->transferframe==camstr->curframe){//wait for the pixels to arrive
    //printf("current frame %d %d\n",n,camstr->pxlcnt[NBUF*cam+(camstr->transferframe&BUFMASK)]);
    while(camstr->pxlcnt[NBUF*cam+(camstr->transferframe&BUFMASK)]<n && rt==0){//wait for pixels to arrive
      camstr->waiting[cam]=1;
      //printf("Waiting for pixels %d %d\n",camstr->pxlcnt[NBUF*cam+(camstr->transferframe&BUFMASK)],n);
      pthread_cond_wait(&camstr->cond[cam],&camstr->m);
      //TODO - should rt be set regardless of whether waiting for frame to transfer?
      rt=camstr->err[NBUF*cam+(camstr->transferframe&BUFMASK)];
      //printf("woken %d rt %d\n",cam,rt);
    }
    //printf("got pixels (rt=%d)\n",rt);
  }
  if(camstr->setFrameNo[cam]){//save the frame counter...
    camstr->setFrameNo[cam]=0;
    camstr->userFrameNo[cam]=camstr->frameCnt[cam*NBUF+(camstr->transferframe&BUFMASK)];
    //printf("frameno %d\n",camstr->userFrameNo[cam]);
  }
  //now copy the data.
  if(n>camstr->pxlsTransferred[cam]){
    if(camstr->pxlRowStartSkipThreshold!=0 || camstr->pxlRowEndInsertThreshold!=0){
      int pxlno;
      if(camstr->reorder[cam]!=0){
	printf("Warning - pixel reordering not implemented yet with pxlRowStartSkipThreshold or pxlRowEndInsertThreshold - if you need this please recode camera interface\n");
      }
      for(i=camstr->pxlsTransferred[cam]; i<n; i++){
	pxlno=camstr->npxlsArrCum[cam]+i+camstr->pxlShift[cam*2+i%2];
	camstr->imgdata[pxlno]=(camstr->DMAbuf[cam][(camstr->transferframe&BUFMASK)*(camstr->npxlsArr[cam])+i]);
	//check the first pixel (of each camera - there are 2 cameras in each cam interface).
	if(camstr->pxlRowStartSkipThreshold!=0 && ((i+camstr->pxlShift[cam*2+i%2])%camstr->pxlx[cam])==i%2 && camstr->imgdata[pxlno]<camstr->pxlRowStartSkipThreshold){
	  camstr->pxlShift[cam*2+i%2]--;//if the dark pixel is in first colum, need to remove a pixel.
	}else if(camstr->pxlRowEndInsertThreshold!=0 && ((i+camstr->pxlShift[cam*2+i%2])%camstr->pxlx[cam])==camstr->pxlx[cam]-4+i%2 && camstr->imgdata[pxlno]<camstr->pxlRowEndInsertThreshold){//If the dark pixel is in the 2nd last column, need to add a pixel (it should fall in the last column with the andors).
	  camstr->pxlShift[cam*2+i%2]++;
	  camstr->imgdata[pxlno+1]=camstr->imgdata[pxlno];
	  camstr->imgdata[pxlno]=camstr->pxlRowEndInsertThreshold;
	}
      }
      if(n==camstr->npxlsArr[cam]){//requesting the last pixels...
	//so, if we've lost any pixels, insert the last ones with the pixel threshold value.
	for(i=n-camstr->pxlShift[cam*2]*2; i<n; i+=2){
	  camstr->imgdata[camstr->npxlsArrCum[cam]+i]=camstr->pxlRowStartSkipThreshold;
	}
	for(i=n-camstr->pxlShift[cam*2+1]*2+1; i<n; i+=2){
	  camstr->imgdata[camstr->npxlsArrCum[cam]+i]=camstr->pxlRowStartSkipThreshold;
	}
      }
    }else{
      if(camstr->reorder[cam]==0){//no pixel reordering
	for(i=camstr->pxlsTransferred[cam]; i<n; i++){
	  camstr->imgdata[camstr->npxlsArrCum[cam]+i]=(camstr->DMAbuf[cam][(camstr->transferframe&BUFMASK)*(camstr->npxlsArr[cam])+i]);
	}
      }else{
	if(camstr->reorderBuf[cam]!=NULL){//reordering based on parameter buffer
	  //Note - todo - need to create reorderbuf and use camNewParam()...
	  //Look for a parameter called "camReorder%d" where %d is reorder[cam]
	  for(i=camstr->pxlsTransferred[cam]; i<n; i++){
	    camstr->imgdata[camstr->reorderBuf[cam][camstr->npxlsArrCum[cam]+i]]=(camstr->DMAbuf[cam][(camstr->transferframe&BUFMASK)*(camstr->npxlsArr[cam])+i]);
	  }
	}else if(camstr->reorder[cam]==1){//this is a standard reorder for scimeasure CCID18 (CANARY LGS) specific reordering
	  for(i=camstr->pxlsTransferred[cam]; i<n; i++){
	    //i%2 specifies whether bottom or top half.
	    //(i//2)%8 specifies which quadrant it is in.
	    //i//16 specifies which pixel it is in the 64x16 pixel quadrant.
	    //row==i//256
	    //col=(i//16)%16
	    //Assumes a 128x128 pixel detector...
	    j=(i%2?127-i/256:i/256)*128+((i/2)%8)*16+(i/16)%16;
	    camstr->imgdata[camstr->npxlsArrCum[cam]+j]=camstr->DMAbuf[cam][(camstr->transferframe&BUFMASK)*(camstr->npxlsArr[cam])+i];
	  }
	}//else if(camstr->reorder[cam]==2){}//can specify other reorderings here
      }
    }
    //printf("pxlsTransferred[%d]=%d\n",cam,n);
    camstr->pxlsTransferred[cam]=n;
  }
  //Fix for a camera bug.  Test the last pixels to see if they are zero, if not, raise an error.  Note, only make this test if pxlRowStartSkipThreshold==0, because otherwise, we have already applied some sort of correction
  if(n==camstr->npxlsArr[cam] && camstr->testLastPixel!=0 && camstr->pxlRowStartSkipThreshold==0){
    for(i=camstr->npxlsArr[cam]-camstr->testLastPixel; i<camstr->npxlsArr[cam];i++){
      if(camstr->imgdata[camstr->npxlsArrCum[cam]+i]!=0){
	rt|=1;
	printf("non-zero final pixel - glitch?\n");
      }
    }
  }
  if(rt!=0){
    printf("camWaitPixels got err %d (cam %d) %ld %ld, frame[0] %d [%d] %d\n",rt,cam,(long)camstr->transferframe,(long)camstr->curframe,camstr->userFrameNo[0],cam,camstr->userFrameNo[cam]);
  }
  pthread_mutex_unlock(&camstr->m);
  return rt;
}
int camFrameFinishedSync(void *camHandle,int err,int forcewrite){//subap thread (once)
  int i;
  CamStruct *camstr=(CamStruct*)camHandle;
 
  for(i=1; i<camstr->ncam; i++){
    if(camstr->userFrameNo[0]!=camstr->userFrameNo[i]){

      writeErrorVA(camstr->rtcErrorBuf,CAMSYNCERROR,camstr->thisiter,"Error - camera frames not in sync");
      break;
    }
  }
  return 0;
}
