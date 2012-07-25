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
#ifndef NOSL240
#include <nslapi.h>
#else
typedef unsigned int uint32;
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
//#include <unistd.h>
#include <pthread.h>
#include "darc.h"
#include "rtccamera.h"
#include "qsort.h"
#include "buffer.h"

#define HDRSIZE 8 //the size of a WPU header - 4 bytes for frame no, 4 bytes for something else.

//we use 4 buffers (instead of double buffering).
#define NBUF 4
#define BUFMASK 0x3
/**
   The struct to hold info.
   If using multi cameras (ie multi SL240 cards or streams), would need to recode, so have multiple instances of this struct.
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
  int *blocksize;//number of pixels to transfer per DMA;
  int **DMAbuf;//a buffer for receiving the DMA - 4*(sizeof(short)*npxls+HDRSIZE).  If a DMA requires a specific word alignment, we may need to reconsider this...
  int open;//set by RTC if the camera is open
  int framing;//set by RTC if the camera is framing.
  volatile int *waiting;//set by RTC if the RTC is waiting for pixels
  volatile int newframeAll;//set by RTC when a new frame is starting to be requested.
  volatile int *newframe;
  //int transferRequired;
  //int frameno;
  unsigned int thisiter;
  unsigned short *imgdata;
  int *pxlsTransferred;//number of pixels copied into the RTC memory.
  pthread_t *threadid;
#ifndef NOSL240
  nslDeviceInfo info;
  nslHandle *handle;
#endif
  uint32 *timeout;//in ms
  int *fibrePort;//the port number on sl240 card.
  unsigned int *userFrameNo;//pointer to the RTC frame number... to be updated for new frame.
  int *setFrameNo;//tells thread to set the userFrameNo.
  void *thrStruct;//pointer to an array of threadStructs.
  int *npxlsArr;
  int *npxlsArrCum;
  int thrcnt;
  int *sl240Opened;//which cameras have been opened okay.
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
  int *gotsyncdv;//flag to whether syncdv has already been received while reading a truncated frame.
  int skipFrameAfterBad;//flag - whether to skip a frame after a bad frame.
  int testLastPixel;//value - if nonzero, and one of the last this many pixels pixel are non-zero, flags as a bad frame.  Assumes that at least one subap will require all ccd pixels to be read (set in the config file - though this may increase latency, if not all pixels required).
  int pxlRowStartSkipThreshold;//If a pixel at the start of a row falls below this threshold, then this pixel is discarded - meaning that all future pixels are shifted one to the left.  Any required padding will take this value.
  int pxlRowEndInsertThreshold;//If a pixel at the end of a row falls below this threshold, then an extra pixel is inserted here - meaning that all future pixels are shifted one to the right.
  int *pxlShift;//the total shift of pixels (-1 for pixel removed, +1 for pixel inserted).
  int *pxlx;
  circBuf *rtcErrorBuf;
  int *frameReady;
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
      pthread_cond_destroy(&camstr->cond[i]);
      pthread_cond_destroy(&camstr->cond2[i]);
    }
    pthread_cond_destroy(&camstr->thrcond);
    pthread_mutex_destroy(&camstr->m);
#ifndef NOSL240
    if(camstr->sl240Opened!=NULL){
      if(camstr->handle!=NULL){
	for(i=0; i<camstr->ncam; i++){
	  if(camstr->sl240Opened[i])
	    nslClose(&camstr->handle[i]);
	}
	free(camstr->handle);
	camstr->handle=NULL;
      }
      safefree(camstr->sl240Opened);
    }
    safefree(camstr->handle);//incase its not already freed.
#endif
    safefree(camstr->npxlsArr);
    safefree(camstr->npxlsArrCum);
    safefree(camstr->blocksize);
    safefree((void*)camstr->pxlcnt);
    safefree(camstr->ntoread);
    safefree(camstr->readStarted);
    safefree((void*)camstr->waiting);
    safefree((void*)camstr->newframe);
    safefree(camstr->pxlsTransferred);
    safefree(camstr->setFrameNo);
    safefree(camstr->timeout);
    safefree(camstr->fibrePort);
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
    safefree(camstr->gotsyncdv);
    safefree(camstr->pxlShift);
    safefree(camstr->pxlx);
    free(camstr);
  }
}


int setThreadAffinityAndPriority(unsigned int *threadAffinity,int threadPriority,int threadAffinElSize){
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
#define MASKED_MODIFY(oldval, modval, mask) (((oldval) & ~(mask)) | ((modval) & (mask)))
/**
   This does the same as nslmon -u X --clrf
   Clears the receive fifo.
   Copied from the source code for nslmon.
*/

int clearReceiveBuffer(CamStruct *camstr,int cam){
  uint32 state=0;
  printf("clearing receive buffer (clrf)\n");
#ifndef NOSL240
  state = nslReadCR(&camstr->handle[cam], 0x8);
#endif
  state = MASKED_MODIFY(state, 0x2000, 0x00002000);
  
#ifndef NOSL240
  nslWriteCR(&camstr->handle[cam], 0x8, state);
#endif  
  //usysMsTimeDelay(10);
  usleep(10000);
  
  state = MASKED_MODIFY(state, 0, 0x00002000);
#ifndef NOSL240
  nslWriteCR(&camstr->handle[cam], 0x8, state);
#endif
  printf("clearing receive buffer (clrf) DONE\n");
  return 0;
}

#undef MASKED_MODIFY


/**
   Start the DMA going
*/
int getData(CamStruct *camstr,int cam,int nbytes,int *dest){
  int rt=0;
#ifndef NOSL240
  uint32 flagsIn,bytesXfered,flagsOut,status;
  flagsIn = NSL_DMA_USE_SYNCDV;//0;
  //pthread_mutex_lock(&camstr->m);

  status = nslRecv(&camstr->handle[cam], (void *)dest, nbytes, flagsIn, camstr->timeout[cam],&bytesXfered, &flagsOut, NULL);
  //pthread_mutex_unlock(&camstr->m);
  if (status == NSL_TIMEOUT) {
    printf("Received timeout\n");
    rt=1;
  } else if (status == NSL_LINK_ERROR) {
    printf("Link error detected\n");
    rt=1;
  } else if (status == NSL_SUCCESS){
    if(flagsOut&NSL_DMA_USE_SYNCDV){
      printf("SYNCDV received while waiting for data - truncated frame (%d/%d bytes)\n",bytesXfered,nbytes);
      //So, have already got the sof for the next frame...
      camstr->gotsyncdv[cam]=1;
      rt=1;
    }else if(nbytes!=bytesXfered){
      printf("%d bytes requested, %d bytes received\n", nbytes, bytesXfered);
      rt=1;
    }else{
      //printf("got data okay\n");
    }
  }else{
    printf("%s\n", nslGetErrStr(status));
    rt=1;
  }
#endif
  return rt;
}
/**
   Wait for a DMA to complete
*/
int waitStartOfFrame(CamStruct *camstr,int cam){
  int rt=0,done=0;
#ifndef NOSL240
  int syncerrmsg=0;
  uint32 flagsIn;
  uint32 sofWord[1024];
  uint32 bytesXfered;
  uint32 flagsOut;
  uint32 status;
#endif
  int nbytes=0;
  //nslSeq seq;
  if(camstr->gotsyncdv[cam]){//have previously got the start of frame while reading a truncated frame...
    done=1;
    camstr->gotsyncdv[cam]=0;
    nbytes=sizeof(uint32);//so that the message isn't printed out below.
  }
#ifndef NOSL240
  flagsIn = NSL_DMA_USE_SYNCDV;
  while(done==0){
    //pthread_mutex_lock(&camstr->m);
    status = nslRecv(&camstr->handle[cam], (void *)sofWord, sizeof(uint32)*1024, flagsIn, camstr->timeout[cam], &bytesXfered, &flagsOut, NULL);
    //pthread_mutex_unlock(&camstr->m);
    if (status == NSL_SUCCESS) {
      if (flagsOut & NSL_DMA_USE_SYNCDV) {
	//printf("SYNC frame data = 0x%x bytes %d\n", sofWord,bytesXfered);
	done=1;
	rt=0;
	nbytes+=bytesXfered;
	
	//printf("New frame %d\n",cam);
      }else{//it may be here that the buffer is partially filled - in which case, we should continue reading 4 bytes for up to a full frame size, and see if we get the SYNC_DV.
	if(syncerrmsg==0){
	  printf("WARNING: SYNCDV not set - may be out of sync, sof[0]=%#x bytes %d timeout %d npxls %d cam %d\n",sofWord[0],bytesXfered,camstr->timeout[cam],camstr->npxlsArr[cam],cam);
	}
	syncerrmsg++;
	rt=1;
	nbytes+=bytesXfered;
      }
    }else if(status!=NSL_TIMEOUT){
      printf("camerror: %s\n",nslGetErrStr(status));
      rt=1;
      done=1;
    }else{
      printf("Timeout waiting for new frame (cam %d)\n",cam);
      rt=1;
      done=1;
    }
  }
  if((syncerrmsg>0 || nbytes!=sizeof(uint32)) && rt==0)//previously printed a sync warning... so now print an ok msg
    printf("Start of frame received okay for cam %d after %d tries (%d bytes, %d pixels)\n",cam,syncerrmsg,nbytes,nbytes/(int)sizeof(uint32));
#endif
  return rt;
}

/**
   The threads that does the work...
   One per camera interface...
*/
void* worker(void *thrstrv){
  ThreadStruct *thrstr=(ThreadStruct*)thrstrv;
  CamStruct *camstr=thrstr->camstr;
  int cam=thrstr->camNo;
  int req,extra,off,err,i;
  int nRead;
  int pxlcnt;
  char timebuf[80];
  time_t tval;
  printf("Calling setThreadAffinityAndPriority\n");
  setThreadAffinityAndPriority(&camstr->threadAffinity[cam*camstr->threadAffinElSize],camstr->threadPriority[cam],camstr->threadAffinElSize);
  pthread_mutex_lock(&camstr->m);
  while(camstr->open){//thread initialised...
    //Cameras are assumed to be synced, so always doing same frame. - actually, this may no longer be true (subkect to testing) - we may now be able to recover... uses ntoread...
    camstr->ntoread[cam]=1;
    if(camstr->thrcnt==0){//first frame...
      camstr->transferframe=0;
      camstr->last=0;
      camstr->latest=0;
      camstr->curframe=0;
    }
    clearReceiveBuffer(camstr,cam);
    while(camstr->framing){//camera is framing...
      if(camstr->thrcnt==0){//first frame...
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
	pxlcnt=0;
	//Read the start of frame...
	err=waitStartOfFrame(camstr,cam);
	//printf("waitstartofframe %d (err %d)\n",cam,err);
	if(err==0){
	  if(nRead==0)
	    camstr->readStarted[cam]=1;
	  //now loop until we've read all the pixels.
	  ((int*)(&camstr->DMAbuf[cam][(camstr->curframe&BUFMASK)*(camstr->npxlsArr[cam]+HDRSIZE/sizeof(unsigned int))]))[0]=0;//set frame counter to zero.
	  while(pxlcnt/*camstr->pxlcnt[NBUF*cam+(camstr->curframe&BUFMASK)]*/<camstr->npxlsArr[cam] && err==0){
	    req=camstr->npxlsArr[cam]-pxlcnt/*camstr->pxlcnt[NBUF*cam+(camstr->curframe&BUFMASK)]*/<camstr->blocksize[cam]?camstr->npxlsArr[cam]-pxlcnt/*camstr->pxlcnt[NBUF*cam+(camstr->curframe&BUFMASK)]*/:camstr->blocksize[cam];
	    if(pxlcnt/*camstr->pxlcnt[NBUF*cam+(camstr->curframe&BUFMASK)]*/==0){//first pixel - also transfer header.
	      extra=HDRSIZE;
	      off=0;
	    }else{
	      extra=0;
	      off=HDRSIZE/sizeof(int)+pxlcnt/*camstr->pxlcnt[NBUF*cam+(camstr->curframe&BUFMASK)]*/;
	    }
	    err=getData(camstr,cam,req*sizeof(int)+extra,&(camstr->DMAbuf[cam][(camstr->curframe&BUFMASK)*(camstr->npxlsArr[cam]+HDRSIZE/sizeof(int))+off]));
	    //if(err==0)
	    //  printf("getdata %d %d\n",camstr->DMAbuf[cam][(camstr->curframe&BUFMASK)*(camstr->npxlsArr[cam]+HDRSIZE/sizeof(short))+4],camstr->curframe);
	    //if(extra!=0){
	    //  printf("frameno %d cam %d\n",camstr->DMAbuf[cam][(camstr->curframe&BUFMASK)*(camstr->npxlsArr[cam]+HDRSIZE/sizeof(int))],cam);
	    //}
	    pxlcnt+=req;
	    if(nRead==0){//have just read part of the frame that is to be sent... none left to read this iteration - so send data to the RTC.
	      pthread_mutex_lock(&camstr->m);

	      camstr->err[NBUF*cam+(camstr->curframe&BUFMASK)]=err;
	      camstr->pxlcnt[NBUF*cam+(camstr->curframe&BUFMASK)]=pxlcnt;//+=req;
	      //printf("pxlcnt now %d, waiting %d\n",camstr->pxlcnt[cam],camstr->waiting[cam]);
	      if(camstr->waiting[cam]==1){//the RTC is waiting for the newest pixels, so wake it up.
		camstr->waiting[cam]=0;
		pthread_cond_broadcast(&camstr->cond[cam]);//signal should do.
	      }
	      pthread_mutex_unlock(&camstr->m);
	    }
	  }

	  //printf("frameno end %d cam %d\n",camstr->DMAbuf[cam][(camstr->curframe&BUFMASK)*(camstr->npxlsArr[cam]+HDRSIZE/sizeof(int))],cam);
	}else{//error getting start of frame
	  ((int*)(&camstr->DMAbuf[cam][(camstr->curframe&BUFMASK)*(camstr->npxlsArr[cam]+HDRSIZE/sizeof(int))]))[0]=0;//set frame counter to zero.
	  memset(&camstr->DMAbuf[cam][(camstr->curframe&BUFMASK)*(camstr->npxlsArr[cam]+HDRSIZE/sizeof(int))+HDRSIZE/sizeof(int)],0,sizeof(int)*camstr->npxlsArr[cam]);
	  //printf("memset dmabuf\n");
	}
      }
      pthread_mutex_lock(&camstr->m);
      if(err==0 && camstr->wpuCorrection!=0){//now see whether other cameras have started their frames - if not - treat it as a mismatched frame - so means an extra read for this camera.
	for(i=0; i<camstr->ncam; i++){
	  if(camstr->readStarted[i]==0){//hasn't started the frame yet...
	    tval=time(NULL);
	    strftime(timebuf,80,"%y/%m/%d %H:%M:%S",localtime(&tval));
	    printf("Read cam%d not yet started - prob frame missing (%s %u)\n",i,timebuf,camstr->thisiter);
	    camstr->ntoread[cam]++;//this camera needs to read an extra frame next time.
	    break;
	  }
	}
	
      }
      //printf("ntoread %d\n",camstr->ntoread[cam]);
      //camstr->ntoread[cam]-=camstr->resync;
      //camstr->ntoread[cam]=1;
      camstr->err[NBUF*cam+(camstr->curframe&BUFMASK)]=err;
      if(err && camstr->waiting[cam]){//the RTC is waiting for the newest pixels, so wake it up, but an error has occurred.
	camstr->waiting[cam]=0;
	pthread_cond_broadcast(&camstr->cond[cam]);
      }
      camstr->thrcnt++;
      //printf("thrcnt %d ncam %d\n",camstr->thrcnt,camstr->ncam);
      if(camstr->thrcnt==camstr->ncam*2){//this is the last thread... so wake the others up... threads increment this once at start, and once at end of each frame
	camstr->latest=camstr->curframe;
	//camstr->curframe++;
	camstr->thrcnt=0;
	if(camstr->skipFrameAfterBad>0){
	  int readExtra=0;
	  for(i=0; i<camstr->ncam; i++){
	    readExtra|=camstr->gotsyncdv[i];//set if a partial frame received.
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
    //no longer framing...
    if(camstr->open){
      pthread_cond_wait(&camstr->cond[cam],&camstr->m);
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

   This is for getting data from the SL240.
   args here currently contains the blocksize (int32) and other things
*/


#define TEST(a) if((a)==NULL){printf("calloc error\n");dofree(camstr);*camHandle=NULL;return 1;}

int camOpen(char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **camHandle,int nthreads,unsigned int frameno,unsigned int **camframeno,int *camframenoSize,int npxls,int ncam,int *pxlx,int* pxly){
  CamStruct *camstr;
#ifndef NOSL240
  uint32 status;
#endif
  int i,ngot,j,k;
  unsigned short *tmps;
  char **reorderCC;
  printf("Initialising camera %s\n",name);
  /*if(npxls&1){
    printf("Error - odd number of pixels not supported by SL240 card\n");
    return 1;
    }*/
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
  camstr->framing=1;
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
  TEST(camstr->blocksize=calloc(ncam,sizeof(int)));
  TEST(camstr->pxlcnt=calloc(ncam*NBUF,sizeof(int)));
  TEST(camstr->ntoread=calloc(ncam,sizeof(int)));
  TEST(camstr->readStarted=calloc(ncam,sizeof(int)));
  TEST(camstr->waiting=calloc(ncam,sizeof(int)));
  TEST(camstr->newframe=calloc(ncam,sizeof(int)));
  TEST(camstr->pxlsTransferred=calloc(ncam,sizeof(int)));
  TEST(camstr->setFrameNo=calloc(ncam,sizeof(int)));
  TEST(camstr->sl240Opened=calloc(ncam,sizeof(int)));
  TEST(camstr->timeout=calloc(ncam,sizeof(uint32)));
  TEST(camstr->fibrePort=calloc(ncam,sizeof(int)));
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
  TEST(camstr->gotsyncdv=calloc(ncam,sizeof(int)));
  TEST(camstr->pxlShift=calloc(ncam*2,sizeof(int)));
  TEST(camstr->pxlx=calloc(ncam,sizeof(int)));
  TEST(camstr->frameReady=calloc(ncam,sizeof(int)));

  camstr->npxlsArrCum[0]=0;
  printf("malloced things\n");
  for(i=0; i<ncam; i++){
    camstr->pxlx[i]=pxlx[i];
    camstr->npxlsArr[i]=pxlx[i]*pxly[i];
    camstr->npxlsArrCum[i+1]=camstr->npxlsArrCum[i]+camstr->npxlsArr[i];
    /*if(camstr->npxlsArr[i]&1){
      printf("Error - odd number of pixels not supported by SL240 card cam %d",i);
      dofree(camstr);
      *camHandle=NULL;
      return 1;
      }*/
  }
  if(n>=(5+args[0])*ncam+1 && n<=(5+args[0])*ncam+7){
    int j;
    for(i=0; i<ncam; i++){
      camstr->blocksize[i]=args[i*(5+args[0])+1];//blocksize in pixels.
      camstr->timeout[i]=args[i*(5+args[0])+2];//timeout in ms.
      camstr->fibrePort[i]=args[i*(5+args[0])+3];//fibre port
      camstr->threadPriority[i]=args[i*(5+args[0])+4];//thread priority
      camstr->reorder[i]=args[i*(5+args[0])+5];//reorder pixels
      for(j=0;j<args[0];j++)
	camstr->threadAffinity[i*args[0]+j]=((unsigned int*)&args[i*(5+args[0])+6])[j];//thread affinity
      /*
      if(camstr->blocksize[i]&1){
	camstr->blocksize[i]++;
	printf("Warning - SL240 needs to transfer an even number of pixels - increasing to %d\n",camstr->blocksize[i]);
      }
      */
    }
    if(n>=(5+args[0])*ncam+2){
      camstr->resync=args[(5+args[0])*ncam+1];
    }else{
      camstr->resync=10;
    }
    if(n>=(5+args[0])*ncam+3){
      camstr->wpuCorrection=args[(5+args[0])*ncam+2];
    }else{
      camstr->wpuCorrection=0;
    }
    if(n>=(5+args[0])*ncam+4){
      camstr->skipFrameAfterBad=args[(5+args[0])*ncam+3];
      printf("skipFrameAfterBad %d\n",camstr->skipFrameAfterBad);
    }else{
      camstr->skipFrameAfterBad=0;
    }
    if(n>=(5+args[0])*ncam+5){
      camstr->testLastPixel=args[(5+args[0])*ncam+4];
      printf("testLastPixel %d\n",camstr->testLastPixel);
    }else{
      camstr->testLastPixel=0;
    }
    if(n>=(5+args[0])*ncam+6){
      camstr->pxlRowStartSkipThreshold=args[(5+args[0])*ncam+5];
      printf("pxlRowStartSkipThreshold %d\n",camstr->pxlRowStartSkipThreshold);
    }else{
      camstr->pxlRowStartSkipThreshold=0;
    }
    if(n>=(5+args[0])*ncam+7){
      camstr->pxlRowEndInsertThreshold=args[(5+args[0])*ncam+6];
      printf("pxlRowEndInsertThreshold %d\n",camstr->pxlRowEndInsertThreshold);
    }else{
      camstr->pxlRowEndInsertThreshold=0;
    }
  }else{
    printf("wrong number of cmd args, should be Naffin, (blocksize, timeout, fibreport, thread priority, reorder, thread affinity[Naffin]),( blocksize,...) for each camera (ie (5+args[0])*ncam) + optional value, resync, equal to max number of frames to try to resync cameras with, plus other optional value wpuCorrection - whether to read extra frame if the WPU cameras get out of sync (ie if a camera doesn't produce a frame occasionally), and another optional flag, whether to skip a frame after a bad frame, and another optional flag - test last pixel (if non-zero, flags as a bad frame), and 2 more optional flags, pxlRowStartSkipThreshold, pxlRowEndInsertThreshold if doing a WPU correction based on dark column detection.\n");
    dofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  printf("got args\n");
  for(i=0; i<ncam; i++){
    printf("%d %d %d\n",camstr->blocksize[i],camstr->timeout[i],camstr->fibrePort[i]);
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
      printf("Failed to mallocparamNames in sl240Int32cam\n");
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
  if((camstr->DMAbuf=malloc(ncam*sizeof(int*)))==NULL){
    printf("Couldn't allocate DMA buffer\n");
    dofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  printf("memset dmabuf\n");
  memset(camstr->DMAbuf,0,sizeof(int*)*ncam);
  printf("doingf dmabuf\n");
  for(i=0; i<ncam; i++){
    if((camstr->DMAbuf[i]=malloc((sizeof(int)*camstr->npxlsArr[i]+HDRSIZE)*NBUF))==NULL){
      printf("Couldn't allocate DMA buffer %d\n",i);
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }
    printf("memset dmabuf...\n");
    memset(camstr->DMAbuf[i],0,(sizeof(int)*camstr->npxlsArr[i]+HDRSIZE)*NBUF);
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
#ifndef NOSL240
  camstr->handle=malloc(sizeof(nslHandle)*ncam);
  memset(camstr->handle,0,sizeof(nslHandle)*ncam);
  //Now do the SL240 stuff...
  for(i=0; i<ncam; i++){
    status = nslOpen(camstr->fibrePort[i], &camstr->handle[i]);
    if (status != NSL_SUCCESS) {
      printf("Failed to open SL240 port %d: %s\n\n",camstr->fibrePort[i], nslGetErrStr(status));
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }
    camstr->sl240Opened[i]=1;
    status = nslGetDeviceInfo(&camstr->handle[i], &camstr->info);
    if (status != NSL_SUCCESS) {
      printf("Failed to get SL240 device info: ");
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }
    printf("\n\nSL240 Device info:\n");
    printf("Unit no.\t %d\n", camstr->info.unitNum);
    printf("Board loc.\t %s\n", camstr->info.boardLocationStr);
    printf("Serial no.\t 0x%x.%x\n", camstr->info.serialNumH, camstr->info.serialNumL);
    printf("Firmware rev.\t 0x%x\n", camstr->info.revisionID);
    printf("Driver rev.\t %s\n", camstr->info.driverRevisionStr);
    printf("Fifo size\t %dM\n", camstr->info.popMemSize/0x100000);
    printf("Link speed\t %d MHz\n", camstr->info.linkSpeed);
    printf("No. links\t %d\n\n\n", camstr->info.numLinks);
    //set up card state.
    status = nslSetState(&camstr->handle[i], NSL_EN_EWRAP, 0);
    if (status != NSL_SUCCESS){
      printf("%s\n",nslGetErrStr(status));dofree(camstr);*camHandle=NULL;return 1;}
    status = nslSetState(&camstr->handle[i],NSL_EN_RECEIVE, 1);
    if (status != NSL_SUCCESS){
      printf("%s\n",nslGetErrStr(status));dofree(camstr);*camHandle=NULL;return 1;}
    status = nslSetState(&camstr->handle[i],NSL_EN_RETRANSMIT, 0);
    if (status != NSL_SUCCESS){
      printf("%s\n",nslGetErrStr(status));dofree(camstr);*camHandle=NULL;return 1;}
    status = nslSetState(&camstr->handle[i],NSL_EN_CRC, 1);
    if (status != NSL_SUCCESS){
      printf("%s\n",nslGetErrStr(status));dofree(camstr);*camHandle=NULL;return 1;}
    status = nslSetState(&camstr->handle[i],NSL_EN_FLOW_CTRL, 0);
    if (status != NSL_SUCCESS){
      printf("%s\n",nslGetErrStr(status));dofree(camstr);*camHandle=NULL;return 1;}
    status = nslSetState(&camstr->handle[i],NSL_EN_LASER, 1);
    if (status != NSL_SUCCESS){
      printf("%s\n",nslGetErrStr(status));dofree(camstr);*camHandle=NULL;return 1;}
    status = nslSetState(&camstr->handle[i],NSL_EN_BYTE_SWAP, 0);
    if (status != NSL_SUCCESS){
      printf("%s\n",nslGetErrStr(status));dofree(camstr);*camHandle=NULL;return 1;}
    status = nslSetState(&camstr->handle[i],NSL_EN_WORD_SWAP, 0);
    if (status != NSL_SUCCESS){
      printf("%s\n",nslGetErrStr(status));dofree(camstr);*camHandle=NULL;return 1;}
    status = nslSetState(&camstr->handle[i], NSL_STOP_ON_LNK_ERR, 1);
    if (status != NSL_SUCCESS){
      printf("%s\n",nslGetErrStr(status));dofree(camstr);*camHandle=NULL;return 1;}
    status = nslSetState(&camstr->handle[i],NSL_EN_RECEIVE, 1);
    if (status != NSL_SUCCESS){
      printf("%s\n",nslGetErrStr(status));dofree(camstr);*camHandle=NULL;return 1;}
    clearReceiveBuffer(camstr,i);

  }
  printf("done nsl\n");

#endif

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
  camstr->framing=0;
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
    camstr->userFrameNo[cam]=*((unsigned int*)(&(camstr->DMAbuf[cam][(camstr->transferframe&BUFMASK)*(camstr->npxlsArr[cam]+HDRSIZE/sizeof(int))])));
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
	camstr->imgdata[pxlno]=(unsigned short)(camstr->DMAbuf[cam][(camstr->transferframe&BUFMASK)*(camstr->npxlsArr[cam]+HDRSIZE/sizeof(int))+HDRSIZE/sizeof(int)+i]);
	//check the first pixel (of each camera - there are 2 cameras in each cam interface).
	if(camstr->pxlRowStartSkipThreshold!=0 && ((i+camstr->pxlShift[cam*2+i%2])%camstr->pxlx[cam])==i%2 && camstr->imgdata[pxlno]<camstr->pxlRowStartSkipThreshold){
	  camstr->pxlShift[cam*2+i%2]--;//if the dark pixel is in first colum, need to remove a pixel.
	  printf("Removing pixel at frame %u cam %d i %d\n",camstr->thisiter,cam,i);
	}else if(camstr->pxlRowEndInsertThreshold!=0 && ((i+camstr->pxlShift[cam*2+i%2])%camstr->pxlx[cam])==camstr->pxlx[cam]-4+i%2 && camstr->imgdata[pxlno]<camstr->pxlRowEndInsertThreshold){//If the dark pixel is in the 2nd last column, need to add a pixel (it should fall in the last column with the andors).
	  camstr->pxlShift[cam*2+i%2]++;
	  camstr->imgdata[pxlno+1]=camstr->imgdata[pxlno];
	  camstr->imgdata[pxlno]=camstr->pxlRowEndInsertThreshold;
	  printf("Inserting pixel at frame %u cam %d i %d\n",camstr->thisiter,cam,i);
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
	  camstr->imgdata[camstr->npxlsArrCum[cam]+i]=(unsigned short)(camstr->DMAbuf[cam][(camstr->transferframe&BUFMASK)*(camstr->npxlsArr[cam]+HDRSIZE/sizeof(int))+HDRSIZE/sizeof(int)+i]);
	}
      }else{
	if(camstr->reorderBuf[cam]!=NULL){//reordering based on parameter buffer
	  //Note - todo - need to create reorderbuf and use camNewParam()...
	  //Look for a parameter called "camReorder%d" where %d is reorder[cam]
	  for(i=camstr->pxlsTransferred[cam]; i<n; i++){
	    camstr->imgdata[camstr->reorderBuf[cam][camstr->npxlsArrCum[cam]+i]]=(unsigned short)(camstr->DMAbuf[cam][(camstr->transferframe&BUFMASK)*(camstr->npxlsArr[cam]+HDRSIZE/sizeof(int))+HDRSIZE/sizeof(int)+i]);
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
	    camstr->imgdata[camstr->npxlsArrCum[cam]+j]=(unsigned short)(camstr->DMAbuf[cam][(camstr->transferframe&BUFMASK)*(camstr->npxlsArr[cam]+HDRSIZE/sizeof(int))+HDRSIZE/sizeof(int)+i]);
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
