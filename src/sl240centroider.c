/**
   The code here is used to create a shared object library, which can then be swapped around depending on which cameras you have in use, ie you simple rename the camera file you want to camera.so (or better, change the soft link), and restart the coremain.

The library is written for a specific camera configuration - ie in multiple camera situations, the library is written to handle multiple cameras, not a single camera many times.
*/
#include <nslapi.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include "rtccentroider.h"
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
  int ncents;//number of pixels in the frame (total for all cameras)
  volatile int transferframe;//the frame currently being passed into the RTC (piecewise)
  volatile int last;//the last frame past to the RTC
  volatile int latest;//the latest whole frame arrived
  volatile int curframe;//the current frame
  volatile int *centcnt;//number of pixels received for this frame and buffer
  //int pxlsRequested;//number of pixels requested by DMA for this frame
  pthread_mutex_t m;
  pthread_cond_t *cond;//sync between main RTC
  pthread_cond_t thrcond;//sync between threads
  int *blocksize;//number of pixels to transfer per DMA;
  float **DMAbuf;//a buffer for receiving the DMA - 4*(sizeof(short)*npxls+HDRSIZE).  If a DMA requires a specific word alignment, we may need to reconsider this...
  int open;//set by RTC if the camera is open
  int framing;//set by RTC if the camera is framing.
  volatile int *waiting;//set by RTC if the RTC is waiting for pixels
  volatile int newframeAll;//set by RTC when a new frame is starting to be requested.
  volatile int *newframe;
  //int transferRequired;
  //int frameno;
  float *centdata;//was imgdata
  int *centsTransferred;//number of pixels copied into the RTC memory.
  pthread_t *threadid;
  nslDeviceInfo info;
  nslHandle *handle;
  uint32 *timeout;//in ms
  int *fibrePort;//the port number on sl240 card.
  int *userFrameNo;//pointer to the RTC frame number... to be updated for new frame.
  int *setFrameNo;//tells thread to set the userFrameNo.
  void *thrStruct;//pointer to an array of threadStructs.
  int *ncentsArr;
  int *ncentsArrCum;
  int thrcnt;
  int *sl240Opened;//which cameras have been opened okay.
  int *err;
  int *threadPriority;
  int *threadAffinity;
}CentStruct;

typedef struct{
  CentStruct *camstr;
  int camNo;
}CentThreadStruct;


void centsafefree(void *ptr){
  if(ptr!=NULL)
    free(ptr);
}

void centdofree(CentStruct *camstr){
  int i;
  printf("centdofree called\n");
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
      centsafefree(camstr->sl240Opened);
    }
    centsafefree(camstr->handle);//incase its not already freed.
    //centsafefree(camstr->ncentsArr);
    centsafefree(camstr->ncentsArrCum);
    centsafefree(camstr->blocksize);
    centsafefree((void*)camstr->centcnt);
    centsafefree((void*)camstr->waiting);
    centsafefree((void*)camstr->newframe);
    centsafefree(camstr->centsTransferred);
    centsafefree(camstr->setFrameNo);
    centsafefree(camstr->timeout);
    centsafefree(camstr->fibrePort);
    centsafefree(camstr->thrStruct);
    centsafefree(camstr->threadid);
    centsafefree(camstr->err);
    centsafefree(camstr->threadPriority);
    centsafefree(camstr->threadAffinity);
    free(camstr);
  }
}


int centsetThreadAffinityAndPriority(int threadAffinity,int threadPriority){
  int i;
  cpu_set_t mask;
  int ncpu;
  struct sched_param param;
  printf("Getting CPUs\n");
  ncpu= sysconf(_SC_NPROCESSORS_ONLN);
  printf("Got %d CPUs\n",ncpu);
  CPU_ZERO(&mask);
  printf("Setting %d CPUs\n",ncpu);
  for(i=0; i<ncpu; i++){
    if(((threadAffinity)>>i)&1){
      CPU_SET(i,&mask);
    }
  }
  printf("Thread affinity %d\n",threadAffinity&0xffff);
  if(sched_setaffinity(0,sizeof(cpu_set_t),&mask))
    printf("Error in sched_setaffinity: %s\n",strerror(errno));
  printf("Setting setparam\n");
  param.sched_priority=threadPriority;
  if(sched_setparam(0,&param)){
    printf("Error in sched_setparam: %s - probably need to run as root if this is important\n",strerror(errno));
  }
  return 0;
}

#define MASKED_MODIFY(oldval, modval, mask) (((oldval) & ~(mask)) | ((modval) & (mask)))
/**
   This does the same as nslmon -u X --clrf
   Clears the receive fifo.
   Copied from the source code for nslmon.
*/
int centClearReceiveBuffer(CentStruct *camstr){
  uint32 state;
  printf("clearing receive buffer (clrf)\n");
  state = nslReadCR(camstr->handle, 0x8);
  state = MASKED_MODIFY(state, 0x2000, 0x00002000);
  
  nslWriteCR(camstr->handle, 0x8, state);
  
  //usysMsTimeDelay(10);
  usleep(10000);
  
  state = MASKED_MODIFY(state, 0, 0x00002000);
  nslWriteCR(camstr->handle, 0x8, state);
  printf("clearing receive buffer (clrf) DONE\n");
  return 0;
}
#undef MASKED_MODIFY


/**
   Start the DMA going
*/
int centgetData(CentStruct *camstr,int cam,int nbytes,float *dest){
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
int centwaitStartOfFrame(CentStruct *camstr,int cam){
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
void* centWorker(void *thrstrv){
  CentThreadStruct *thrstr=(CentThreadStruct*)thrstrv;
  CentStruct *camstr=thrstr->camstr;
  int cam=thrstr->camNo;
  int req,extra,off,err,i;
  printf("Calling centsetThreadAffinityAndPriority\n");
  centsetThreadAffinityAndPriority(camstr->threadAffinity[cam],camstr->threadPriority[cam]);
  pthread_mutex_lock(&camstr->m);
  while(camstr->open){//thread initialised...
    //Cameras are assumed to be synced, so always doing same frame.
    if(camstr->thrcnt==0){//first frame...
      camstr->transferframe=0;
      camstr->last=0;
      camstr->latest=0;
      camstr->curframe=0;
    }
    centClearReceiveBuffer(camstr);//clear the receive fifos
    while(camstr->framing){//camera is framing...
      if(camstr->thrcnt==0){//first frame...
	camstr->thrcnt++;
	camstr->curframe++;
	for(i=0; i<camstr->ncam; i++){
	  camstr->centcnt[NBUF*cam+(camstr->curframe&BUFMASK)]=0;
	}
      }
      //camstr->pxlsRequested=0;
      pthread_mutex_unlock(&camstr->m);
      //Read the start of frame...
      if((err=centwaitStartOfFrame(camstr,cam))==0){
	//now loop until we've read all the pixels.
	((int*)(&camstr->DMAbuf[cam][(camstr->curframe&BUFMASK)*(camstr->ncentsArr[cam]+HDRSIZE/sizeof(float))]))[0]=0;//set frame counter to zero.
	while(camstr->centcnt[NBUF*cam+(camstr->curframe&BUFMASK)]<camstr->ncentsArr[cam] && err==0){
	  req=camstr->ncentsArr[cam]-camstr->centcnt[NBUF*cam+(camstr->curframe&BUFMASK)]<camstr->blocksize[cam]?camstr->ncentsArr[cam]-camstr->centcnt[NBUF*cam+(camstr->curframe&BUFMASK)]:camstr->blocksize[cam];
	  if(camstr->centcnt[NBUF*cam+(camstr->curframe&BUFMASK)]==0){//first pixel - also transfer header.
	    extra=HDRSIZE;
	    off=0;
	  }else{
	    extra=0;
	    off=HDRSIZE/sizeof(float)+camstr->centcnt[NBUF*cam+(camstr->curframe&BUFMASK)];
	  }
	  err=centgetData(camstr,cam,req*sizeof(float)+extra,&(camstr->DMAbuf[cam][(camstr->curframe&BUFMASK)*(camstr->ncentsArr[cam]+HDRSIZE/sizeof(float))+off]));
	  //if(err==0)
	  //  printf("getdata %d %d\n",camstr->DMAbuf[cam][(camstr->curframe&BUFMASK)*(camstr->ncentsArr[cam]+HDRSIZE/sizeof(short))+4],camstr->curframe);
	  pthread_mutex_lock(&camstr->m);
	  camstr->err[NBUF*cam+(camstr->curframe&BUFMASK)]=err;
	  camstr->centcnt[NBUF*cam+(camstr->curframe&BUFMASK)]+=req;
	  //printf("centcnt now %d, waiting %d\n",camstr->centcnt[cam],camstr->waiting[cam]);
	  if(camstr->waiting[cam]==1){//the RTC is waiting for the newest pixels, so wake it up.
	    camstr->waiting[cam]=0;
	    pthread_cond_signal(&camstr->cond[cam]);//signal should do.
	  }
	  pthread_mutex_unlock(&camstr->m);
	}
      }else{//error getting start of frame
	((int*)(&camstr->DMAbuf[cam][(camstr->curframe&BUFMASK)*(camstr->ncentsArr[cam]+HDRSIZE/sizeof(float))]))[0]=0;//set frame counter to zero.
	memset(&camstr->DMAbuf[cam][(camstr->curframe&BUFMASK)*(camstr->ncentsArr[cam]+HDRSIZE/sizeof(float))+HDRSIZE/sizeof(float)],0,sizeof(float)*camstr->ncentsArr[cam]);
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
   Open a camera of type name.  Args are passed in a float array of size n, which can be cast if necessary.  Any state data is returned in camHandle, which should be NULL if an error arises.
   pxlbuf is the array that should hold the data. The library is free to use the user provided version, or use its own version as necessary (ie a pointer to physical memory or whatever).  It is of size ncents*sizeof(short).
   ncam is number of cameras, which is the length of arrays pxlx and pxly, which contain the dimensions for each camera.  Currently, ncam must equal 1.
   Name is used if a library can support more than one camera.
   frameno is a pointer to an array with a value for each camera in which the frame number should be placed.

   This is for getting data from the SL240.
   args here currently contains the blocksize (int32)
*/


#define TEST(a) if((a)==NULL){printf("calloc error\n");centdofree(camstr);*camHandle=NULL;return 1;}

int centOpen(char *name,int n,int *args,char *buf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **camHandle,int ncam,int *nsubs,int* frameno){
  CentStruct *camstr;
  uint32 status;
  int i;
  printf("Initialising centroid camera %s\n",name);
  if((*camHandle=malloc(sizeof(CentStruct)))==NULL){
    printf("Couldn't malloc camera handle\n");
    return 1;
  }
  printf("Malloced camstr\n");
  memset(*camHandle,0,sizeof(CentStruct));
  camstr=(CentStruct*)*camHandle;
  camstr->centdata=arr->wpucentroids;
  camstr->userFrameNo=frameno;
  camstr->ncam=ncam;
  //camstr->npxls=npxls;//*pxlx * *pxly;
  camstr->ncentsArr=nsubs;
  TEST(camstr->ncentsArrCum=calloc((ncam+1),sizeof(int)));
  TEST(camstr->blocksize=calloc(ncam,sizeof(int)));
  TEST(camstr->centcnt=calloc(ncam*NBUF,sizeof(int)));
  TEST(camstr->waiting=calloc(ncam,sizeof(int)));
  TEST(camstr->newframe=calloc(ncam,sizeof(int)));
  TEST(camstr->centsTransferred=calloc(ncam,sizeof(int)));
  TEST(camstr->setFrameNo=calloc(ncam,sizeof(int)));
  TEST(camstr->sl240Opened=calloc(ncam,sizeof(int)));
  TEST(camstr->timeout=calloc(ncam,sizeof(uint32)));
  TEST(camstr->fibrePort=calloc(ncam,sizeof(int)));
  TEST(camstr->err=calloc(ncam*NBUF,sizeof(int)));
  TEST(camstr->thrStruct=calloc(ncam,sizeof(CentThreadStruct)));
  TEST(camstr->threadid=calloc(ncam,sizeof(pthread_t)));
  TEST(camstr->cond=calloc(ncam,sizeof(pthread_cond_t)));
  TEST(camstr->threadAffinity=calloc(ncam,sizeof(int)));
  TEST(camstr->threadPriority=calloc(ncam,sizeof(int)));
  camstr->ncentsArrCum[0]=0;
  printf("malloced things\n");
  for(i=0; i<ncam; i++){
    //camstr->ncentsArr[i]=pxlx[i]*pxly[i];
    camstr->ncentsArrCum[i+1]=camstr->ncentsArrCum[i]+camstr->ncentsArr[i];
    /*
    if(camstr->ncentsArr[i]&1){
      printf("Error - odd number of pixels not supported by SL240 card cam %d",i);
      centdofree(camstr);
      *camHandle=NULL;
      return 1;
      }*/
  }
  camstr->ncents=camstr->ncentsArrCum[ncam];
  //Now sort out the args...
  if(n==5*ncam){
    for(i=0; i<ncam; i++){
      camstr->blocksize[i]=args[i*5+0];//blocksize in pixels.
      camstr->timeout[i]=args[i*5+1];//timeout in ms.
      camstr->fibrePort[i]=args[i*5+2];//fibre port
      camstr->threadAffinity[i]=args[i*5+3];//thread affinity
      camstr->threadPriority[i]=args[i*5+4];//thread priority
      /*
      if(camstr->blocksize[i]&1){
	camstr->blocksize[i]++;
	printf("Warning - SL240 needs to transfer an even number of pixels - increasing to %d\n",camstr->blocksize[i]);
	}*/
    }
  }else{
    printf("wrong number of cmd args, should be blocksize, timeout, fibreport, thread affinity, thread priority, blocksize,... for each camera (ie 5*ncam)\n");
    centdofree(camstr);
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
      centdofree(camstr);
      *camHandle=NULL;
      return 1;
    }
  }
  if(pthread_cond_init(&camstr->thrcond,NULL)!=0){
    printf("Error initialising thread condition variable\n");
    centdofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  //maybe think about having one per camera???
  if(pthread_mutex_init(&camstr->m,NULL)!=0){
    printf("Error initialising mutex variable\n");
    centdofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  printf("done mutex\n");
  if((camstr->DMAbuf=malloc(ncam*sizeof(float*)))==NULL){
    printf("Couldn't allocate DMA buffer\n");
    centdofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  printf("memset dmabuf\n");
  memset(camstr->DMAbuf,0,sizeof(float*)*ncam);
  printf("doingf dmabuf\n");
  for(i=0; i<ncam; i++){
    if((camstr->DMAbuf[i]=malloc((sizeof(float)*camstr->ncentsArr[i]+HDRSIZE)*NBUF))==NULL){
      printf("Couldn't allocate DMA buffer %d\n",i);
      centdofree(camstr);
      *camHandle=NULL;
      return 1;
    }
    printf("memset dmabuf...\n");
    memset(camstr->DMAbuf[i],0,(sizeof(float)*camstr->ncentsArr[i]+HDRSIZE)*NBUF);
  }
  printf("done dmabuf\n");
  camstr->handle=malloc(sizeof(nslHandle)*ncam);
  memset(camstr->handle,0,sizeof(nslHandle)*ncam);
  //Now do the SL240 stuff...
  for(i=0; i<ncam; i++){
    status = nslOpen(camstr->fibrePort[i], &camstr->handle[i]);
    if (status != NSL_SUCCESS) {
      printf("Failed to open SL240 port %d: %s\n\n",camstr->fibrePort[i], nslGetErrStr(status));
      centdofree(camstr);
      *camHandle=NULL;
      return 1;
    }
    camstr->sl240Opened[i]=1;
    status = nslGetDeviceInfo(&camstr->handle[i], &camstr->info);
    if (status != NSL_SUCCESS) {
      printf("Failed to get SL240 device info: ");
      centdofree(camstr);
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
      printf("%s\n",nslGetErrStr(status));centdofree(camstr);*camHandle=NULL;return 1;}
    status = nslSetState(&camstr->handle[i],NSL_EN_RECEIVE, 1);
    if (status != NSL_SUCCESS){
      printf("%s\n",nslGetErrStr(status));centdofree(camstr);*camHandle=NULL;return 1;}
    status = nslSetState(&camstr->handle[i],NSL_EN_RETRANSMIT, 0);
    if (status != NSL_SUCCESS){
      printf("%s\n",nslGetErrStr(status));centdofree(camstr);*camHandle=NULL;return 1;}
    status = nslSetState(&camstr->handle[i],NSL_EN_CRC, 1);
    if (status != NSL_SUCCESS){
      printf("%s\n",nslGetErrStr(status));centdofree(camstr);*camHandle=NULL;return 1;}
    status = nslSetState(&camstr->handle[i],NSL_EN_FLOW_CTRL, 0);
    if (status != NSL_SUCCESS){
      printf("%s\n",nslGetErrStr(status));centdofree(camstr);*camHandle=NULL;return 1;}
    status = nslSetState(&camstr->handle[i],NSL_EN_LASER, 1);
    if (status != NSL_SUCCESS){
      printf("%s\n",nslGetErrStr(status));centdofree(camstr);*camHandle=NULL;return 1;}
    status = nslSetState(&camstr->handle[i],NSL_EN_BYTE_SWAP, 0);
    if (status != NSL_SUCCESS){
      printf("%s\n",nslGetErrStr(status));centdofree(camstr);*camHandle=NULL;return 1;}
    status = nslSetState(&camstr->handle[i],NSL_EN_WORD_SWAP, 0);
    if (status != NSL_SUCCESS){
      printf("%s\n",nslGetErrStr(status));centdofree(camstr);*camHandle=NULL;return 1;}
    status = nslSetState(&camstr->handle[i], NSL_STOP_ON_LNK_ERR, 1);
    if (status != NSL_SUCCESS){
      printf("%s\n",nslGetErrStr(status));centdofree(camstr);*camHandle=NULL;return 1;}
    status = nslSetState(&camstr->handle[i],NSL_EN_RECEIVE, 1);
    if (status != NSL_SUCCESS){
      printf("%s\n",nslGetErrStr(status));centdofree(camstr);*camHandle=NULL;return 1;}
  }
  printf("done nsl\n");

  centClearReceiveBuffer(camstr);


  camstr->open=1;
  for(i=0; i<ncam; i++){
    ((CentThreadStruct*)camstr->thrStruct)[i].camNo=i;
    ((CentThreadStruct*)camstr->thrStruct)[i].camstr=camstr;
    pthread_create(&camstr->threadid[i],NULL,centWorker,&((CentThreadStruct*)camstr->thrStruct)[i]);
  }
  printf("created threads\n");
  return 0;
}


/**
   Close a camera of type name.  Args are passed in the float array of size n, and state data is in camHandle, which should be freed and set to NULL before returning.
*/
int centClose(void **camHandle){
  CentStruct *camstr;
  int i;
  printf("Closing centroid camera\n");
  if(*camHandle==NULL)
    return 1;
  camstr=(CentStruct*)*camHandle;
  pthread_mutex_lock(&camstr->m);
  camstr->open=0;
  camstr->framing=0;
  for(i=0; i<camstr->ncam; i++){
    pthread_cond_signal(&camstr->cond[i]);
  }
  pthread_mutex_unlock(&camstr->m);
  for(i=0; i<camstr->ncam; i++){
    pthread_join(camstr->threadid[i],NULL);//wait for centWorker thread to complete
  }
  centdofree(camstr);
  *camHandle=NULL;
  printf("Centroid camera closed\n");
  return 0;
}
int centNewParam(void *camHandle,char *buf,unsigned int frameno,arrayStruct *arr){
  CentStruct *camstr=(CentStruct*)camHandle;
  
  camstr->centdata=arr->wpucentroids;

  return 0;
}


/**
   Start the camera framing, using the args and camera handle data.
*/
int centStartFraming(int n,int *args,void *camHandle){
  CentStruct *camstr;
  int i;
  if(camHandle==NULL){
    printf("called centStartFraming with camHandle==NULL\n");
    return 1;
  }
  camstr=(CentStruct*)camHandle;
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
int centStopFraming(void *camHandle){
  CentStruct *camstr;
  if(camHandle==NULL){
    printf("called centStopFraming with camHandle==NULL\n");
    return 1;
  }
  camstr=(CentStruct*)camHandle;
  pthread_mutex_lock(&camstr->m);
  camstr->framing=0;
  pthread_mutex_unlock(&camstr->m);
  printf("Stopping framing\n");
  return 0;
}

/**
   Can be called to get the latest iamge taken by the camera
*/
int centGetLatest(void *camHandle){
  if(camHandle==NULL){
    printf("called centGetLatest with camHandle==NULL\n");
    return 1;
  }
  printf("Getting latest frame (TODO if required)\n");
  return 0;
}

/**
   Called when we're starting processing the next frame.  This doesn't actually wait for any pixels.
*/
int centNewFrame(void *camHandle){
  //printf("camNewFrame\n");
  CentStruct *camstr;
  int i;
  camstr=(CentStruct*)camHandle;
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
int centWaitPixels(int n,int cam,void *camHandle){
  //printf("camWaitPixels %d, camera %d\n",n,cam);
  CentStruct *camstr=(CentStruct*)camHandle;
  int rt=0;
  //int i;
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
  if(n>camstr->ncents)
    n=camstr->ncents;
  //printf("camWaitPixels\n");
  pthread_mutex_lock(&camstr->m);
  //printf("camWaitPixels got mutex, newframe=%d\n",camstr->newframe[cam]);
  if(camstr->newframe[cam]){//first thread for this camera after new frame...
    camstr->newframe[cam]=0;
    camstr->setFrameNo[cam]=1;
    camstr->centsTransferred[cam]=0;
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
    while(camstr->centcnt[NBUF*cam+(camstr->transferframe&BUFMASK)]<n && rt==0){//wait for pixels to arrive
      camstr->waiting[cam]=1;
      //printf("Waiting for pixels %d %d\n",camstr->centcnt[NBUF*cam+(camstr->transferframe&BUFMASK)],n);
      pthread_cond_wait(&camstr->cond[cam],&camstr->m);
      //printf("woken\n");
      rt=camstr->err[NBUF*cam+(camstr->transferframe&BUFMASK)];
    }
    //printf("got pixels (rt=%d)\n",rt);
  }
  if(camstr->setFrameNo[cam]){//save the frame counter...
    camstr->setFrameNo[cam]=0;
    camstr->userFrameNo[cam]=*((int*)(&(camstr->DMAbuf[cam][(camstr->transferframe&BUFMASK)*(camstr->ncentsArr[cam]+HDRSIZE/sizeof(float))])));
  }
  //now copy the data.
  if(n>camstr->centsTransferred[cam]){
    memcpy(&camstr->centdata[camstr->ncentsArrCum[cam]+camstr->centsTransferred[cam]],&camstr->DMAbuf[cam][(camstr->transferframe&BUFMASK)*(camstr->ncentsArr[cam]+HDRSIZE/sizeof(float))+HDRSIZE/sizeof(float)+camstr->centsTransferred[cam]],sizeof(float)*(n-camstr->centsTransferred[cam]));
    //for(i=camstr->centsTransferred[cam]; i<n; i++){
    //  camstr->centdata[camstr->ncentsArrCum[cam]+i]=(camstr->DMAbuf[cam][(camstr->transferframe&BUFMASK)*(camstr->ncentsArr[cam]+HDRSIZE/sizeof(float))+HDRSIZE/sizeof(float)+i]);
    //}
    camstr->centsTransferred[cam]=n;
  }
  if(rt!=0)
    printf("centWaitPixels got err %d (cam %d) frame %ld %ld\n",rt,cam,(long)camstr->transferframe,(long)camstr->curframe);
  pthread_mutex_unlock(&camstr->m);
  return rt;
}
