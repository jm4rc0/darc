/**
   The code here is used to create a shared object library, which can then be swapped around depending on which cameras you have in use, ie you simple rename the camera file you want to camera.so (or better, change the soft link), and restart the coremain.

The library is written for a specific camera configuration - ie in multiple camera situations, the library is written to handle multiple cameras, not a single camera many times.
*/
#include <nslapi.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rtccamera.h"
#include <time.h>
//#include <unistd.h>
#include <pthread.h>

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
  pthread_cond_t thrcond;//sync between threads
  int *blocksize;//number of pixels to transfer per DMA;
  short **DMAbuf;//a buffer for receiving the DMA - 4*(sizeof(short)*npxls+HDRSIZE).  If a DMA requires a specific word alignment, we may need to reconsider this...
  int open;//set by RTC if the camera is open
  int framing;//set by RTC if the camera is framing.
  volatile int *waiting;//set by RTC if the RTC is waiting for pixels
  volatile int newframeAll;//set by RTC when a new frame is starting to be requested.
  volatile int *newframe;
  //int transferRequired;
  //int frameno;
  short *imgdata;
  int *pxlsTransferred;//number of pixels copied into the RTC memory.
  pthread_t *threadid;
  nslDeviceInfo info;
  nslHandle *handle;
  uint32 *timeout;//in ms
  int *fibrePort;//the port number on sl240 card.
  int *userFrameNo;//pointer to the RTC frame number... to be updated for new frame.
  int *setFrameNo;//tells thread to set the userFrameNo.
  void *thrStruct;//pointer to an array of threadStructs.
  int *npxlsArr;
  int *npxlsArrCum;
  int thrcnt;
  int *sl240Opened;//which cameras have been opened okay.
  int *err;
}CamStruct;

typedef struct{
  CamStruct *camstr;
  int camNo;
}ThreadStruct;

/**
   Find out if this SO library supports your camera.

*/
int camQuery(char *name){
  //Note, the strings aren't necessarily null terminated...
  if(strcmp(name,"sl240cam")==0)
    return 0;
  return 1;
}

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
    }
    pthread_cond_destroy(&camstr->thrcond);
    pthread_mutex_destroy(&camstr->m);
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
    safefree(camstr->npxlsArr);
    safefree(camstr->npxlsArrCum);
    safefree(camstr->blocksize);
    safefree((void*)camstr->pxlcnt);
    safefree((void*)camstr->waiting);
    safefree((void*)camstr->newframe);
    safefree(camstr->pxlsTransferred);
    safefree(camstr->setFrameNo);
    safefree(camstr->timeout);
    safefree(camstr->fibrePort);
    safefree(camstr->thrStruct);
    safefree(camstr->threadid);
    safefree(camstr->err);
    free(camstr);
  }
}

/**
   Start the DMA going
*/
int getData(CamStruct *camstr,int cam,int nbytes,short *dest){
  uint32 flagsIn,bytesXfered,flagsOut,status;
  int rt=0;
  flagsIn = 0;
  status = nslRecv(&camstr->handle[cam], (void *)dest, nbytes, flagsIn, camstr->timeout[cam],&bytesXfered, &flagsOut, NULL);
  if (status == NSL_TIMEOUT) {
    printf("Received timeout\n");
    rt=1;
  } else if (status == NSL_LINK_ERROR) {
    printf("Link error detected\n");
    rt=-1;
  } else if (status == NSL_SUCCESS){
    if(nbytes!=bytesXfered){
      printf("%d bytes requested, %d bytes received\n", nbytes, bytesXfered);
      rt=-1;
    }else{
      //printf("got data okay\n");
    }
  }else{
    printf("%s\n", nslGetErrStr(status));
    rt=-1;
  }
  return rt;
}
/**
   Wait for a DMA to complete
*/
int waitStartOfFrame(CamStruct *camstr,int cam){
  uint32 flagsIn;
  uint32 sofWord;
  uint32 bytesXfered;
  uint32 flagsOut;
  uint32 status;
  int rt=0,done=0;
  //nslSeq seq;
  flagsIn = NSL_DMA_USE_SYNCDV;
  while(done==0){
    status = nslRecv(&camstr->handle[cam], (void *)&sofWord, sizeof(uint32), flagsIn, camstr->timeout[cam], &bytesXfered, &flagsOut, NULL);
    if (status == NSL_SUCCESS) {
      if (flagsOut & NSL_DMA_USE_SYNCDV) {
	//printf("SYNC frame data = 0x%x bytes %d\n", sofWord,bytesXfered);
	done=1;
	rt=0;
      }else{
	printf("WARNING: SYNCDV not set - may be out of sync, sof=%#x bytes %d timeout %d\n",sofWord,bytesXfered,camstr->timeout[cam]);
	rt=-1;
      }
    }else if(status!=NSL_TIMEOUT){
      printf("%s\n",nslGetErrStr(status));
      rt=-1;
      done=1;
    }else{
      printf("Timeout waiting for new frame\n");
      rt=1;
      done=1;
    }
  }
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
  pthread_mutex_lock(&camstr->m);
  while(camstr->open){//thread initialised...
    //Cameras are assumed to be synced, so always doing same frame.
    if(camstr->thrcnt==0){//first frame...
      camstr->transferframe=0;
      camstr->last=0;
      camstr->latest=0;
      camstr->curframe=0;
    }
    while(camstr->framing){//camera is framing...
      if(camstr->thrcnt==0){//first frame...
	camstr->thrcnt++;
	camstr->curframe++;
	for(i=0; i<camstr->ncam; i++){
	  camstr->pxlcnt[NBUF*cam+(camstr->curframe&BUFMASK)]=0;
	}
      }
      //camstr->pxlsRequested=0;
      pthread_mutex_unlock(&camstr->m);
      //Read the start of frame...
      if((err=waitStartOfFrame(camstr,cam))==0){
	//now loop until we've read all the pixels.
	((int*)(&camstr->DMAbuf[cam][(camstr->curframe&BUFMASK)*(camstr->npxlsArr[cam]+HDRSIZE/sizeof(short))]))[0]=0;//set frame counter to zero.
	while(camstr->pxlcnt[NBUF*cam+(camstr->curframe&BUFMASK)]<camstr->npxlsArr[cam] && err==0){
	  req=camstr->npxlsArr[cam]-camstr->pxlcnt[NBUF*cam+(camstr->curframe&BUFMASK)]<camstr->blocksize[cam]?camstr->npxlsArr[cam]-camstr->pxlcnt[NBUF*cam+(camstr->curframe&BUFMASK)]:camstr->blocksize[cam];
	  if(camstr->pxlcnt[NBUF*cam+(camstr->curframe&BUFMASK)]==0){//first pixel - also transfer header.
	    extra=HDRSIZE;
	    off=0;
	  }else{
	    extra=0;
	    off=HDRSIZE/sizeof(short)+camstr->pxlcnt[NBUF*cam+(camstr->curframe&BUFMASK)];
	  }
	  err=getData(camstr,cam,req*sizeof(short)+extra,&(camstr->DMAbuf[cam][(camstr->curframe&BUFMASK)*(camstr->npxlsArr[cam]+HDRSIZE/sizeof(short))+off]));
	  //if(err==0)
	  //  printf("getdata %d %d\n",camstr->DMAbuf[cam][(camstr->curframe&BUFMASK)*(camstr->npxlsArr[cam]+HDRSIZE/sizeof(short))+4],camstr->curframe);
	  pthread_mutex_lock(&camstr->m);
	  camstr->err[NBUF*cam+(camstr->curframe&BUFMASK)]=err;
	  camstr->pxlcnt[NBUF*cam+(camstr->curframe&BUFMASK)]+=req;
	  //printf("pxlcnt now %d, waiting %d\n",camstr->pxlcnt[cam],camstr->waiting[cam]);
	  if(camstr->waiting[cam]==1){//the RTC is waiting for the newest pixels, so wake it up.
	    camstr->waiting[cam]=0;
	    pthread_cond_signal(&camstr->cond[cam]);//signal should do.
	  }
	  pthread_mutex_unlock(&camstr->m);
	}
      }else{//error getting start of frame
	((int*)(&camstr->DMAbuf[cam][(camstr->curframe&BUFMASK)*(camstr->npxlsArr[cam]+HDRSIZE/sizeof(short))]))[0]=0;//set frame counter to zero.
	memset(&camstr->DMAbuf[cam][(camstr->curframe&BUFMASK)*(camstr->npxlsArr[cam]+HDRSIZE/sizeof(short))+HDRSIZE/sizeof(short)],0,sizeof(short)*camstr->npxlsArr[cam]);
      }
      pthread_mutex_lock(&camstr->m);
      camstr->err[NBUF*cam+(camstr->curframe&BUFMASK)]=err;
      if(err && camstr->waiting[cam]){//the RTC is waiting for the newest pixels, so wake it up, but an error has occurred.
	camstr->waiting[cam]=0;
	pthread_cond_signal(&camstr->cond[cam]);
      }
      camstr->thrcnt++;
      if(camstr->thrcnt==camstr->ncam*2){//this is the last thread... so wake the others up... threads increment this once at start, and once at end of each frame
	camstr->latest=camstr->curframe;
	camstr->thrcnt=0;
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
   args here currently contains the blocksize (int32)
*/


#define TEST(a) if((a)==NULL){printf("calloc error\n");dofree(camstr);*camHandle=NULL;return 1;}

int camOpen(char *name,int n,int *args,char *buf,circBuf *rtcErrorBuf,char *prefix,void **camHandle,int npxls,short *pxlbuf,int ncam,int *pxlx,int* pxly,int* frameno){
  CamStruct *camstr;
  uint32 status;
  int i;
  printf("Initialising camera %s\n",name);
  if(camQuery(name)){
    printf("Wrong camera type %s\n",name);
    return 1;
  }
  if(npxls&1){
    printf("Error - odd number of pixels not supported by SL240 card\n");
    return 1;
  }
  if((*camHandle=malloc(sizeof(CamStruct)))==NULL){
    printf("Couldn't malloc camera handle\n");
    return 1;
  }
  printf("Malloced camstr\n");
  memset(*camHandle,0,sizeof(CamStruct));
  camstr=(CamStruct*)*camHandle;
  camstr->imgdata=pxlbuf;
  camstr->userFrameNo=frameno;
  camstr->ncam=ncam;
  camstr->npxls=npxls;//*pxlx * *pxly;
  TEST(camstr->npxlsArr=calloc(ncam,sizeof(int)));
  TEST(camstr->npxlsArrCum=calloc((ncam+1),sizeof(int)));
  TEST(camstr->blocksize=calloc(ncam,sizeof(int)));
  TEST(camstr->pxlcnt=calloc(ncam*NBUF,sizeof(int)));
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
  camstr->npxlsArrCum[0]=0;
  printf("malloced things\n");
  for(i=0; i<ncam; i++){
    camstr->npxlsArr[i]=pxlx[i]*pxly[i];
    camstr->npxlsArrCum[i+1]=camstr->npxlsArrCum[i]+camstr->npxlsArr[i];
    if(camstr->npxlsArr[i]&1){
      printf("Error - odd number of pixels not supported by SL240 card cam %d",i);
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }
  }
  if(n==3*ncam){
    for(i=0; i<ncam; i++){
      camstr->blocksize[i]=args[i*3+0];//blocksize in pixels.
      camstr->timeout[i]=args[i*3+1];//timeout in ms.
      camstr->fibrePort[i]=args[i*3+2];//fibre port
      if(camstr->blocksize[i]&1){
	camstr->blocksize[i]++;
	printf("Warning - SL240 needs to transfer an even number of pixels - increasing to %d\n",camstr->blocksize[i]);
      }
    }
  }else{
    printf("wrong number of cmd args, should be blocksize, timeout, fibreport, blocksize,... for each camera (ie 3*ncam)\n");
    dofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  printf("got args\n");
  for(i=0; i<ncam; i++){
    printf("%d %d %d\n",camstr->blocksize[i],camstr->timeout[i],camstr->fibrePort[i]);
  }
  for(i=0; i<ncam; i++){
    if(pthread_cond_init(&camstr->cond[i],NULL)!=0){
      printf("Error initialising condition variable %d\n",i);
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
  if((camstr->DMAbuf=malloc(ncam*sizeof(short*)))==NULL){
    printf("Couldn't allocate DMA buffer\n");
    dofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  printf("memset dmabuf\n");
  memset(camstr->DMAbuf,0,sizeof(short*)*ncam);
  printf("doingf dmabuf\n");
  for(i=0; i<ncam; i++){
    if((camstr->DMAbuf[i]=malloc((sizeof(short)*camstr->npxlsArr[i]+HDRSIZE)*NBUF))==NULL){
      printf("Couldn't allocate DMA buffer %d\n",i);
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }
    printf("memset dmabuf...\n");
    memset(camstr->DMAbuf[i],0,(sizeof(short)*camstr->npxlsArr[i]+HDRSIZE)*NBUF);
  }
  printf("done dmabuf\n");
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
  }
  printf("done nsl\n");


  camstr->open=1;
  for(i=0; i<ncam; i++){
    ((ThreadStruct*)camstr->thrStruct)[i].camNo=i;
    ((ThreadStruct*)camstr->thrStruct)[i].camstr=camstr;
    pthread_create(&camstr->threadid[i],NULL,worker,&((ThreadStruct*)camstr->thrStruct)[i]);
  }
  printf("created threads\n");
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
    pthread_cond_signal(&camstr->cond[i]);
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
int camNewParam(void *camHandle,char *buf,unsigned int frameno){
  return 0;
}
/**
   Start the camera framing, using the args and camera handle data.
*/
int camStartFraming(int n,int *args,void *camHandle){
  CamStruct *camstr;
  int i;
  if(camHandle==NULL){
    printf("called camStartFraming with camHandle==NULL\n");
    return 1;
  }
  camstr=(CamStruct*)camHandle;
  pthread_mutex_lock(&camstr->m);
  camstr->framing=1;
  for(i=0; i<camstr->ncam; i++){
    pthread_cond_signal(&camstr->cond[i]);
  }
  pthread_mutex_unlock(&camstr->m);
  printf("Framing camera\n");
  return 0;
}
/**
   Stop the camera framing
*/
int camStopFraming(void *camHandle){
  CamStruct *camstr;
  if(camHandle==NULL){
    printf("called camStopFraming with camHandle==NULL\n");
    return 1;
  }
  camstr=(CamStruct*)camHandle;
  pthread_mutex_lock(&camstr->m);
  camstr->framing=0;
  pthread_mutex_unlock(&camstr->m);
  printf("Stopping framing\n");
  return 0;
}

/**
   Can be called to get the latest iamge taken by the camera
*/
int camGetLatest(void *camHandle){
  if(camHandle==NULL){
    printf("called camGetLatest with camHandle==NULL\n");
    return 1;
  }
  printf("Getting latest frame (TODO if required)\n");
  return 0;
}

/**
   Called when we're starting processing the next frame.  This doesn't actually wait for any pixels.
*/
int camNewFrame(void *camHandle){
  //printf("camNewFrame\n");
  CamStruct *camstr;
  int i;
  camstr=(CamStruct*)camHandle;
  if(camHandle==NULL || camstr->framing==0){
    //printf("called camNewFrame with camHandle==NULL\n");
    return 1;
  }
  pthread_mutex_lock(&camstr->m);
  camstr->newframeAll=1;
  for(i=0;i<camstr->ncam; i++)
    camstr->newframe[i]=1;
  camstr->last=camstr->transferframe;
  pthread_mutex_unlock(&camstr->m);
  return 0;
}

/**
   Wait for the next n pixels of the current frame to arrive from camera cam.
   Note - this can be called by multiple threads at same time.  Need to make sure its thread safe.
   Actually - this is only called by 1 thread per camera at a time.

*/
int camWaitPixels(int n,int cam,void *camHandle){
  //printf("camWaitPixels %d, camera %d\n",n,cam);
  CamStruct *camstr=(CamStruct*)camHandle;
  int rt=0;
  //static struct timeval t1;
  //struct timeval t2;
  //struct timeval t3;
  //printf("camWaitPixels\n");
  if(camHandle==NULL || camstr->framing==0){
    //printf("called camWaitPixels with camHandle==NULL\n");
    return 1;
  }
  if(n<0)
    n=0;
  if(n>camstr->npxls)
    n=camstr->npxls;
  //printf("camWaitPixels\n");
  pthread_mutex_lock(&camstr->m);
  //printf("camWaitPixels got mutex, newframe=%d\n",camstr->newframe[cam]);
  if(camstr->newframe[cam]){//first thread for this camera after new frame...
    camstr->newframe[cam]=0;
    camstr->setFrameNo[cam]=1;
    camstr->pxlsTransferred[cam]=0;
    if(camstr->newframeAll){//first thread overall after new frame started
      camstr->newframeAll=0;
      if(camstr->latest==camstr->last){//the latest whole frame received from the camera is the one already passed to the RTC.
	camstr->transferframe=camstr->curframe;
      }else{//use the most recently arrived whole frame...
	camstr->transferframe=camstr->latest;
      }
    }
  }
  if(camstr->transferframe==camstr->curframe){//wait for the pixels to arrive
    while(camstr->pxlcnt[NBUF*cam+(camstr->transferframe&BUFMASK)]<n && rt==0){//wait for pixels to arrive
      camstr->waiting[cam]=1;
      //printf("Waiting for pixels %d %d\n",camstr->pxlcnt[NBUF*cam+(camstr->transferframe&BUFMASK)],n);
      pthread_cond_wait(&camstr->cond[cam],&camstr->m);
      //printf("woken\n");
      rt=camstr->err[NBUF*cam+(camstr->transferframe&BUFMASK)];
    }
    //printf("got pixels (rt=%d)\n",rt);
  }
  if(camstr->setFrameNo[cam]){//save the frame counter...
    camstr->setFrameNo[cam]=0;
    camstr->userFrameNo[cam]=*((int*)(&camstr->DMAbuf[cam][(camstr->transferframe&BUFMASK)*(camstr->npxlsArr[cam]+HDRSIZE/sizeof(short))]));
  }
  //now copy the data.
  if(n>camstr->pxlsTransferred[cam]){
    memcpy(&camstr->imgdata[camstr->npxlsArrCum[cam]+camstr->pxlsTransferred[cam]],&camstr->DMAbuf[cam][(camstr->transferframe&BUFMASK)*(camstr->npxlsArr[cam]+HDRSIZE/sizeof(short))+HDRSIZE/sizeof(short)+camstr->pxlsTransferred[cam]],sizeof(short)*(n-camstr->pxlsTransferred[cam]));
    camstr->pxlsTransferred[cam]=n;
  }
  if(rt!=0)
    printf("camWaitPixels got err %d (cam %d) frame %ld %ld\n",rt,cam,(long)camstr->transferframe,(long)camstr->curframe);
  pthread_mutex_unlock(&camstr->m);
  return rt;
}
