/**
This file does the bulk of processing for the RTC.
*/
#define USECOND
#define CVSID MAINCVSID"$Id$"
#define _GNU_SOURCE
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <stdlib.h>
#include <dlfcn.h>
#include <unistd.h>
#include <limits.h>
#include <sys/types.h>
#ifdef USECOND
#else
#include <sys/ipc.h>
#include <sys/sem.h>
#endif
#include <sys/time.h>
#include <sys/stat.h>
#include <assert.h>
#include <math.h>
#include <sys/mman.h>
#ifdef USEMKL
#include <mkl.h>
#include <mkl_types.h>
#include <mkl_blas.h>
#include <mkl_cblas.h>
#include <mkl_vml.h>
#elif defined(USEAGBBLAS)
#include "agbcblas.h"
#else//use GSL instead...
#include <gsl/gsl_cblas.h>
//typedef enum CBLAS_ORDER CBLAS_ORDER;
//typedef enum CBLAS_TRANSPOSE CBLAS_TRANSPOSE;
#endif
#include <pthread.h>
#include <fftw3.h>
#include <signal.h>
#include "circ.h"
#include "rtccamera.h"
#include "rtcmirror.h"
#define NEWRECON
#include "rtcrecon.h"
#include "darcNames.h"
#include "darc.h"
#include "qsort.h"
#include "buffer.h"
/*Threading/sync is as follows:

Have a number of threads per camera (may be a different number each).  
Each of these thread sets is then dealt with separately while doing the frame:
Each thread gets a camera mutex, and when got, waits for pixels from next subap to be ready.  Then proceeds to process this data after releasing the mutex.

The global sync is arranged around a double buffer system.

Each thread gets a global start mutex, and increments a counter.
If it is the first thread (over all, not per camera), it checks whether new memory needs allocating and if so does this.
The first thread also sees whether a buffer swap is necessary, and sets a flag.
Threads do a buffer swap here if necessary, if they are the first thread for each camera.
Each thread swaps its references to the double buffered parameters if needed.
The start mutex is then released.

At the end of a frame (not just a subap), each thread gets a global end mutex, and increments a counter.
If it is the last thread, it checks whether memory should be freed.  
Then release the end mutex.

All threads wait until all processing of a given frame has been completed.  They then proceed to the next frame.  However, post processing can continue (in a separate thread) while subap processing of the next frame commences.

TODO:
open_mp number of threads (for mkl blas).
Change number of pixels waiting for with adaptive windowing (since shifts windows about) - this has been done, subject to testing
*/



#ifdef DOTIMING
#define STARTTIMING struct timeval t1,t2;gettimeofday(&t1,NULL);
#define ENDTIMING(NAME)  gettimeofday(&t2,NULL);threadInfo->globals->NAME##TimeSum+=t2.tv_sec*1e6+t2.tv_usec-t1.tv_sec*1e6-t1.tv_usec;threadInfo->globals->NAME##TimeCnt++;
#define PRINTTIMING(NAME) printf(#NAME" total time %gs (%d calls)\n",threadInfo->globals->NAME##TimeSum/1e6,threadInfo->globals->NAME##TimeCnt);

#else
#define STARTTIMING
#define ENDTIMING(NAME)
#define PRINTTIMING(NAME)
#endif

/**
   Write an error to the circular buffer which will then wake clients.
 */
void writeErrorVA(circBuf *rtcErrorBuf,int errnum,int frameno,char *txt,...){
  //when start using this seriously, may need to think about thread locking.
  //errnum is -1 for irregular errors, or >=0 for errors that may repeat regularly and need to be cleared by the user before being sent again, such as clipping of too many actuators.  Actually, it might make sense to have all errors ones that need clearing?  Or maybe not.
  //Since this function can be called from anywhere, it needs to be thread safe.
  int l;
  int warn=0;
  struct timeval t1;
  pthread_mutex_t *mut=&rtcErrorBuf->mutex;
  va_list ap;
  char *tmp;
  if(pthread_mutex_lock(mut))
    printf("pthread_mutex_lock error in writeError: %s\n",strerror(errno));
  
  //printf("err %s %d %d\n",txt,errnum,rtcErrorBuf->errFlag);
  if(errnum>=0){
    if((((rtcErrorBuf->errFlag)>>errnum)&1)==0)
      warn=1;
    (rtcErrorBuf->errFlag)|=(1<<errnum);//set the error flag
  }else
    warn=1;
  if(warn){
    printf("warning\n");
    gettimeofday(&t1,NULL);
    va_start(ap,txt);
    if((l=vasprintf(&tmp,txt,ap))>0){
      if(l>1024){
	l=1024;
	tmp[1023]='\0';
      }
      circAddSize(rtcErrorBuf,tmp,strlen(tmp)+1,0,t1.tv_sec+t1.tv_usec*1e-6,frameno);
      free(tmp);
    }else{//error doing formatting... just print the raw string.
      l=strlen(txt)+1;
      if(l>1024){
	l=1024;
	txt[1023]='\0';
      }
      circAddSize(rtcErrorBuf,txt,strlen(txt)+1,0,t1.tv_sec+t1.tv_usec*1e-6,frameno);
    }
    va_end(ap);
  }
  pthread_mutex_unlock(mut);
}
void writeError(circBuf *rtcErrorBuf,char *txt,int errnum,int frameno){
  //when start using this seriously, may need to think about thread locking.
  //errnum is -1 for irregular errors, or >=0 for errors that may repeat regularly and need to be cleared by the user before being sent again, such as clipping of too many actuators.  Actually, it might make sense to have all errors ones that need clearing?  Or maybe not.
  //Since this function can be called from anywhere, it needs to be thread safe.
  int l=strlen(txt)+1;
  int warn=0;
  struct timeval t1;
  pthread_mutex_t *mut=&rtcErrorBuf->mutex;
  if(pthread_mutex_lock(mut))
    printf("pthread_mutex_lock error in writeError: %s\n",strerror(errno));
  
  if(l>1024){
    l=1024;
    txt[1023]='\0';
  }
  //printf("err %s %d %d\n",txt,errnum,rtcErrorBuf->errFlag);
  if(errnum>=0){
    if((((rtcErrorBuf->errFlag)>>errnum)&1)==0)
      warn=1;
    (rtcErrorBuf->errFlag)|=(1<<errnum);//set the error flag
  }else
    warn=1;
  if(warn){
    printf("warning\n");
    gettimeofday(&t1,NULL);
    circAddSize(rtcErrorBuf,txt,strlen(txt)+1,0,t1.tv_sec+t1.tv_usec*1e-6,frameno);
  }
  pthread_mutex_unlock(mut);
}


#ifdef USEMKL
#define vectorPowx vsPowx
#else
void vectorPowx(int n,float *in,float powerFactor,float *out){//a replacement for mkl function...
  int i;
  for(i=0; i<n; i++)
    out[i]=powf(in[i],powerFactor);
}
#endif



/**
   Wait until the pixel buffer and centroid arrays can be written too, ie the post compute thread has finished writing them to the circular buffers.
   @return 0
 */
int waitPxlBufAndCentroidReady(globalStruct *glob){
  if(pthread_mutex_lock(&glob->precomp->pxlcentMutex))
    printf("pthread_mutex_lock error in waitPxlBufAndCentroidReady: %s\n",strerror(errno));
  if(glob->precomp->pxlcentReady==0)
    pthread_cond_wait(&glob->precomp->pxlcentCond,&glob->precomp->pxlcentMutex);
  pthread_mutex_unlock(&glob->precomp->pxlcentMutex);
  return 0;

}
/**
   Called by the post compute thread once it has finished with the pixel and centroid buffers.
   @return 0
 */
int setPxlBufAndCentroidReady(globalStruct *glob){
  if(pthread_mutex_lock(&glob->precomp->pxlcentMutex))
    printf("pthread_mutex_lock error in setPxlBufAndCentroidReady: %s\n",strerror(errno));
  glob->precomp->pxlcentReady=1;
  pthread_cond_broadcast(&glob->precomp->pxlcentCond);
  pthread_mutex_unlock(&glob->precomp->pxlcentMutex);
  return 0;

}
/**
   Updates subap locations, when in adaptive windowing mode
   @return the number of extra pixels required to be read from the CCD for this new subap location to be valid.  This number may be -ve.
*/
int updateSubapLocation(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  int *loc=NULL,*rloc;
  int i,cnt=0,npxl=0,maxpxl=0,imax;
  //now update subapLocation for next time...
  imax=info->nsub-(threadInfo->cursubindx-info->subCumIndx);
  i=0;
  //for(i=0; i<info->nsubapsTogether && i<imax; i++){
  for(i=0; i<threadInfo->nsubapsProcessing; i++){
    if(info->subflag[i+threadInfo->cursubindx-info->subCumIndx]==1){
      //subap is valid.
      loc=&(info->subapLocation[(threadInfo->cursubindx+i)*6]);//adaptive.
      rloc=&(info->realSubapLocation[(threadInfo->cursubindx+i)*6]);//user defined
      loc[0]=rloc[0]+info->adaptiveCentPos[threadInfo->centindx+2*cnt+1]*rloc[2];
      loc[1]=rloc[1]+info->adaptiveCentPos[threadInfo->centindx+2*cnt+1]*rloc[2];
      loc[3]=rloc[3]+info->adaptiveCentPos[threadInfo->centindx+2*cnt]*rloc[5];
      loc[4]=rloc[4]+info->adaptiveCentPos[threadInfo->centindx+2*cnt]*rloc[5];
      loc[2]=rloc[2];
      loc[5]=rloc[5];
      //now move the centers if the window moves outside the CCD...
      while(loc[0]<0){
	loc[0]+=rloc[2];
	loc[1]+=rloc[2];
	info->adaptiveCentPos[threadInfo->centindx+2*cnt+1]++;
      }
      while(loc[1]>info->npxly){
	loc[1]-=rloc[2];
	loc[0]-=rloc[2];
	info->adaptiveCentPos[threadInfo->centindx+2*cnt+1]--;
      }
      while(loc[3]<0){
	loc[3]+=rloc[5];
	loc[4]+=rloc[5];
	info->adaptiveCentPos[threadInfo->centindx+2*cnt]++;
      }
      while(loc[4]>info->npxlx){
	loc[4]-=rloc[5];
	loc[3]-=rloc[5];
	info->adaptiveCentPos[threadInfo->centindx+2*cnt]--;
      }
      //calculate the new number of pixels required for this shifted subap.
      npxl=info->pxlCnt[threadInfo->cursubindx+i]+((loc[0]-rloc[0])/rloc[2])*info->npxlx+(loc[3]-rloc[3])/rloc[5];
      if(npxl>maxpxl)//and keep a note of the maximum number required.
	maxpxl=npxl;
      cnt++;//increment the number of subaps valid.
    }
  }

  //here, we calculate the number of extra pixels required for the last subap of this block.  
  if(maxpxl>info->npxlx*info->npxly)
    maxpxl=info->npxlx*info->npxly;
  return maxpxl;// ((loc[0]-rloc[0])/rloc[2])*info->npxlx+(loc[3]-rloc[3])/rloc[5];
}

/**
   Copy a subaperture from the camera DMA memory into thread workspace.
   @return 0
 */
int copySubap(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  int cnt=0;
  int i,j;
  int *loc;
  float *tmp;
  loc=&(info->subapLocation[threadInfo->cursubindx*6]);
  threadInfo->curnpxly=(loc[1]-loc[0])/loc[2];
  threadInfo->curnpxlx=(loc[4]-loc[3])/loc[5];
  threadInfo->curnpxl=threadInfo->curnpxly*threadInfo->curnpxlx;
  if(threadInfo->curnpxl>threadInfo->subapSize){
    threadInfo->subapSize=threadInfo->curnpxl;
    if((tmp=fftwf_malloc(sizeof(float)*threadInfo->subapSize))==NULL){//must be freed using fftwf_free.
      printf("subap re-malloc failed\n");
      exit(0);
    }
    fftwf_free(threadInfo->subap);
    threadInfo->subap=tmp;
    if((tmp=malloc(sizeof(float)*threadInfo->subapSize))==NULL){
      printf("sort re-malloc failed\n");
      exit(0);
    }
    free(threadInfo->sort);
    threadInfo->sort=tmp;
  }



  if(info->fakeCCDImage!=NULL){
    for(i=loc[0]; i<loc[1]; i+=loc[2]){
      for(j=loc[3]; j<loc[4]; j+=loc[5]){
	threadInfo->subap[cnt]=(float)info->fakeCCDImage[info->npxlCum+i*info->npxlx+j];
	cnt++;
      }
    }
  }else{
    for(i=loc[0]; i<loc[1]; i+=loc[2]){
      for(j=loc[3]; j<loc[4]; j+=loc[5]){
	threadInfo->subap[cnt]=(float)info->pxlbuf[info->npxlCum+i*info->npxlx+j];
	cnt++;
      }
    }
  }
  return 0;
}
/**
   waitPixels(cnt,threadInfo), which is called by only 1 thread (per camera) at any
   one time, is blocking, waiting until "cnt" camera pixels have
   arrived for the current frame.  This then means that the subap
   processing for this thread can then continue.
   The camWaitPixelsFn can return 1 on error, 0 if okay, or -1 if no pixels, but not an error (why this would be, I can't think).
   @return 0 unless an error has occurred.
 */
int waitPixels(int cnt,threadStruct *threadInfo){
  int rt=0;
  globalStruct *glob=threadInfo->globals;
  infoStruct *info=threadInfo->info;
  if(glob->cameraLib!=NULL){
    rt=(*glob->camWaitPixelsFn)(cnt,info->cam,glob->camHandle);
    //The frame number will have been set.  So, if this is the first thread for this camera, can store the cam frame number.  If the first thread overall, can store glob->thisiter.  Also need to check that all frame numbers from each camera are the same - raise error if not.
   
    pthread_mutex_lock(&glob->camMutex);
    if(glob->camReadCnt==0){//first thread overall
      glob->setFrameno=1;
    }
    if(glob->setFrameno){//need to set frame no
      if(info->cam==0){//first cam can set frameno
	glob->setFrameno=0;
	if(rt==0){
	  if(glob->camiter==glob->camframeno[0]){//it would seem that the camera isn't updating the iteration count.
	    glob->thisiter++;
	  }else{//camera is updating the iteration count (frameno)
	    glob->thisiter=glob->camframeno[0];
	    glob->camiter=glob->thisiter;
	  }
	}else{//we need to update here - failed or no camera library.
	  glob->thisiter++;
	}
      }else{//not first cam.
	//we're not the first cam, but the frameno hasn't yet been set, so we have nothing to check against.  Is there anything I can do here?  Don't think so really, just have to hope!!!
      }
    }else{//frameno has already been set, so we can check against it.
      if(rt==0 && glob->camiter!=glob->camframeno[info->cam]){
	writeErrorVA(glob->rtcErrorBuf,CAMSYNCERROR,glob->thisiter,"Error - camera frames not in sync");
      }
    }
    glob->camReadCnt++;
    pthread_mutex_unlock(&glob->camMutex);
  }
  return rt==1;
}



/**
   Called by each thread when they are ready to process the next subap.  They block here until pixels for this subap become available for them to process.
   @return int 0 on success, -1 if frame finished, 1 on error.
 */
int waitNextSubaps(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  int endFrame=0,i,cnt=0,npxls=0;
  //Threads run in order, and are returned a subap as it becomes available...
  //first block in the thread queue until it is my turn.
  dprintf("locking subapMutex\n");
  if(pthread_mutex_lock(&info->subapMutex))
    printf("pthread_mutex_lock error in waitNextSubaps: %s\n",strerror(errno));
  dprintf("locked subapMutex %d %p %d\n",info->subindx,info->subflag,info->id);
  //work out which is the next subap to do...
  //if(info->frameFinished[threadInfo->mybuf]==1){091109
  if(info->frameFinished==1){
    dprintf("frameFinished in waitNextSubaps: %d\n",info->frameFinished);//091109[threadInfo->mybuf]);
    pthread_mutex_unlock(&info->subapMutex);
    return -1;
  }

  while(cnt==0 && info->subindx<info->nsub){
    npxls=0;
    while(info->subflag[info->subindx]==0 && info->subindx<info->nsub)
      info->subindx++;//skip unused subaps...
    i=0;
    while((info->subflag[info->subindx+i]==0 || info->pxlCnt[info->subindx+info->subCumIndx]==info->pxlCnt[info->subindx+i+info->subCumIndx]) && i<info->nsub-info->subindx){//iterate over subaps that require same number of pixels... (this replaces the old nsubapsTogether, but is a bit better and doesn't assume a square subap geometry)

    //for(i=0; i<info->nsubapsTogether && i<info->nsub-info->subindx; i++){
      cnt+=info->subflag[info->subindx+i];
      //see how many pixels this subap requires..., store the max so far.
      if(info->subflag[info->subindx+i]==1 && info->pxlCnt[info->subindx+i+info->subCumIndx]>npxls)
	npxls=info->pxlCnt[info->subindx+i+info->subCumIndx];
      i++;
    }
    if(cnt==0)
      info->subindx+=i;//info->nsubapsTogether;
    
  }
  if(info->subindx>=info->nsub){
    info->subindx=0;
    info->centindx=0;
    endFrame=-1;
    info->frameFinished=1;//091109[threadInfo->mybuf]=1;//let others know we've read all subaps.
  }
  //}
  if(endFrame==0){
    //store the indx for this thread...
    threadInfo->centindx=info->centindx+info->centCumIndx;
    threadInfo->cursubindx=info->subindx+info->subCumIndx;
    threadInfo->nsubapsDoing=cnt;//the number of used subaps to do.
    threadInfo->nsubapsProcessing=i;//the number of subaps to do.
    //threadInfo->cursuby=info->subindx/info->nsubx;
    //threadInfo->cursubx=info->subindx%info->nsubx;
    //and increment subindx ready for the next thread.
    info->centindx+=2*cnt;//info->nsubapsTogether;//increase by number of used subaps by this thread.
    threadInfo->nsubs=info->centindx;
    info->subindx+=i;//info->nsubapsTogether;
    if(info->subindx>=info->nsub){
      info->subindx=0;
      info->centindx=0;
      //this thread has valid data, so still want to process it, butother threads shouldn;t as frame is finished.
      info->frameFinished=1;//091109[threadInfo->mybuf]=1;//let other threadsknow that we've read all the subaps...
    }
    dprintf("waiting pixels\n");
    //then wait until pixels become available...
    //Note, when using adaptive windowing, the number of pixels required changes, depending on window position.   We compute this here.
    if(info->windowMode==WINDOWMODE_ADAPTIVE || info->windowMode==WINDOWMODE_GLOBAL){
      npxls=updateSubapLocation(threadInfo);
    }
    endFrame=waitPixels(npxls,threadInfo);//info->pxlCnt[threadInfo->cursubindx+info->nsubapsTogether-1]+extrapxl,threadInfo);
    dprintf("waited pixels\n");
    if(endFrame){
      writeErrorVA(threadInfo->globals->rtcErrorBuf,CAMGETERROR,threadInfo->globals->thisiter,"Error - getting camera pixels");
    }
  }
  //then release mutex so that next thread can wait for pixels, and reset this sem, so that this thread will wait next time...
  pthread_mutex_unlock(&info->subapMutex);//unlock the mutex, allowing the next thread to continue
  dprintf("freed muted\n");
  return endFrame;
}

#ifdef USESLOWCALIBRATION
/**
   Does nothing
 */

int handleBadPxl(threadStruct *threadInfo){
  return 0;
}

/**
   Subtract a background
*/
int subtractBG(threadStruct *threadInfo){
  //Subtract the background...
  infoStruct *info=threadInfo->info;
  int *loc;
  int i;
  STARTTIMING;
  if(info->background!=NULL){
    loc=&(info->subapLocation[threadInfo->cursubindx*6]);
    for(i=0; i<threadInfo->curnpxly; i++){
#ifdef USEAGBBLAS
      agb_cblas_saxpy1(threadInfo->curnpxlx,-1.,&(info->background[(loc[0]+i*loc[2])*info->npxlx+loc[3]]),loc[5],&(threadInfo->subap[i*threadInfo->curnpxlx]));
#else
      cblas_saxpy(threadInfo->curnpxlx,-1.,&(info->background[(loc[0]+i*loc[2])*info->npxlx+loc[3]]),loc[5],&(threadInfo->subap[i*threadInfo->curnpxlx]),1);
#endif
    }
  }
  ENDTIMING(subtractBg);
  return 0;
}

/**
   Subtract dark noise
*/
int subtractDarkNoise(threadStruct *threadInfo){
  //Subtract the dark noise...
  infoStruct *info=threadInfo->info;
  int *loc;
  int i;
  //STARTTIMING;
  if(info->darkNoise!=NULL){
    loc=&(info->subapLocation[threadInfo->cursubindx*6]);
    for(i=0; i<threadInfo->curnpxly; i++){
#ifdef USEAGBBLAS
      agb_cblas_saxpy1(threadInfo->curnpxlx,-1.,&(info->darkNoise[(loc[0]+i*loc[2])*info->npxlx+loc[3]]),loc[5],&(threadInfo->subap[i*threadInfo->curnpxlx]));
#else
      cblas_saxpy(threadInfo->curnpxlx,-1.,&(info->darkNoise[(loc[0]+i*loc[2])*info->npxlx+loc[3]]),loc[5],&(threadInfo->subap[i*threadInfo->curnpxlx]),1);
#endif
    }
  }
  //ENDTIMING(subtractBg);
  return 0;
}

/**
   multiply each pixel by its flatfield value
*/
int applyFlatField(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  int *loc;
  int i,j;
  STARTTIMING;
  if(info->flatField!=NULL){
    loc=&(info->subapLocation[threadInfo->cursubindx*6]);
    for(i=0; i<threadInfo->curnpxly; i++){
      for(j=0; j<threadInfo->curnpxlx; j++){
	threadInfo->subap[i*threadInfo->curnpxlx+j]*=info->flatField[(loc[0]+i*loc[2])*info->npxlx+loc[3]+j*loc[5]];
      }
    }
  }
  ENDTIMING(applyFlatField);
  return 0;
}
/**
   Apply a threshold using 1 of two algorithms
*/
int applyThreshold(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  int i,j;
  float threshold;
  float *thresharr=NULL;
  int *loc;
  STARTTIMING;
  if(info->thresholdArr==NULL){
    threshold=info->threshold;
  }else if(info->thresholdArrType==0){//per subap
    threshold=info->thresholdArr[threadInfo->cursubindx];
  }else{//a per pxl threshold
    thresharr=&info->thresholdArr[info->npxlCum];
  }
  if(info->thresholdArrType==0){//single value for this subap
    if(info->thresholdAlgo==1){
      for(i=0; i<threadInfo->curnpxl; i++){
	if(threadInfo->subap[i]<threshold)
	  threadInfo->subap[i]=0;
      }
    }else if(info->thresholdAlgo==2){
      for(i=0; i<threadInfo->curnpxl; i++){
	if(threadInfo->subap[i]<threshold)
	  threadInfo->subap[i]=0;
	else
	  threadInfo->subap[i]-=threshold;
      }
    }
  }else{
    //value per pixel
    loc=&(info->subapLocation[threadInfo->cursubindx*6]);
    if(info->thresholdAlgo==1){
      for(i=0; i<threadInfo->curnpxly; i++){
	for(j=0; j<threadInfo->curnpxlx; j++){
	  if(threadInfo->subap[i*threadInfo->curnpxlx+j]<thresharr[(loc[0]+i*loc[2])*info->npxlx+loc[3]+j*loc[5]])
	    threadInfo->subap[i*threadInfo->curnpxlx+j]=0;
	}
      }
    }else{
      for(i=0; i<threadInfo->curnpxly; i++){
	for(j=0; j<threadInfo->curnpxlx; j++){
	  if(threadInfo->subap[i*threadInfo->curnpxlx+j]<thresharr[(loc[0]+i*loc[2])*info->npxlx+loc[3]+j*loc[5]])
	    threadInfo->subap[i*threadInfo->curnpxlx+j]=0;
	  else
	    threadInfo->subap[i*threadInfo->curnpxlx+j]-=thresharr[(loc[0]+i*loc[2])*info->npxlx+loc[3]+j*loc[5]];
	}
      }
    }
  }
  ENDTIMING(applyThreshold);
  return 0;
}
/**
   multiply each pixel by its weighting.
*/
int applyPxlWeighting(threadStruct *threadInfo){
  //If put anything here, need to update controlCorba.py so that when acquiring a background image, it sets pixel weighting to something correct... (1)
  infoStruct *info=threadInfo->info;
  int *loc;
  int i,j;
  if(info->pxlweight!=NULL){
    loc=&(info->subapLocation[threadInfo->cursubindx*6]);
    for(i=0; i<threadInfo->curnpxly; i++){
      for(j=0; j<threadInfo->curnpxlx; j++){
	threadInfo->subap[i*threadInfo->curnpxlx+j]*=info->pxlweight[(loc[0]+i*loc[2])*info->npxlx+loc[3]+j*loc[5]];
      }
    }
    
  }
  return 0;
}
#endif //USESLOWCALIBRATION

/**
   Raise each pixel to a given power
*/
int applyPowerFactor(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  if(info->powerFactor!=1.){
    vectorPowx(threadInfo->curnpxl,threadInfo->subap,info->powerFactor,threadInfo->subap);//an mkl function...

  }
  return 0;
}
int storeCalibratedSubap(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  int cnt=0;
  int i,j;
  int *loc;
  float *calpxlbuf=threadInfo->info->calpxlbuf;
  loc=&(info->subapLocation[threadInfo->cursubindx*6]);
  //printf("store %d %d\n",loc[0],loc[3]);
  for(i=loc[0]; i<loc[1]; i+=loc[2]){
    for(j=loc[3]; j<loc[4]; j+=loc[5]){
      calpxlbuf[info->npxlCum+i*info->npxlx+j]=threadInfo->subap[cnt];
      cnt++;
    }
  }
  return 0;
}

int storeCorrelationSubap(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  int cnt=0;
  int i,j;
  int *loc;
  float *corrbuf=threadInfo->info->corrbuf;
  loc=&(info->subapLocation[threadInfo->cursubindx*6]);

  for(i=loc[0]; i<loc[1]; i+=loc[2]){
    for(j=loc[3]; j<loc[4]; j+=loc[5]){
      corrbuf[info->npxlCum+i*info->npxlx+j]=threadInfo->subap[cnt];
      cnt++;
    }
  }
  return 0;
}


/**
   We only want to use the brightest N (=info->useBrightest) pixels - set the 
   rest to zero.
*/
int applyBrightest(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  int i;
  //int j;
  //float min=threadInfo->subap[0];
  //int n=info->useBrightest;
  //float v;
  float *sort=threadInfo->sort;
  float *subap=threadInfo->subap;
  float thr;
  int useBrightest;
  if(info->useBrightestArr!=NULL){
    useBrightest=info->useBrightestArr[threadInfo->cursubindx];
  }else{
    useBrightest=info->useBrightest;
  }
  if(useBrightest>=threadInfo->curnpxl)
    return 0;//want to allow more pixels than there are...
  //copy the first n pixels, and then sort this...
  //Actually - may as well copy the whole lot, and sort the whole lot... not a lot of difference in speed depending on how many pixels you want to keep, and this is more understandable and robust...
  memcpy(sort,subap,sizeof(float)*threadInfo->curnpxl);
#define cmp(a,b) ((*a)<(*b))
  QSORT(float,sort,threadInfo->curnpxl,cmp);
#undef cmp

  //The threshold to use is:
  thr=sort[threadInfo->curnpxl-useBrightest];
  for(i=0; i<threadInfo->curnpxl; i++){
    if(subap[i]<thr)
      subap[i]=0;
  }
  return 0;
}

/**
   Calibrate CCD pixels more quickly...  Here, to improve performace,
   we do some if tests outside of the main loops.  This means a bit
   more code, but should run faster.
*/
int subapPxlCalibration(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  int *loc;
  int i,j;
  int cnt=0;
  int pos,pos2;
  float *subap=threadInfo->subap;
  float *calmult=info->calmult;
  float *calthr=info->calthr;
  float *calsub=info->calsub;
  //STARTTIMING;
  loc=&(info->subapLocation[threadInfo->cursubindx*6]);
  if(calmult!=NULL && calsub!=NULL){
    if((info->thresholdAlgo==1 || info->thresholdAlgo==2) && calthr!=NULL){
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	pos=info->npxlCum+i*info->npxlx;
	for(j=loc[3];j<loc[4]; j+=loc[5]){
	  pos2=pos+j;
	  subap[cnt]*=calmult[pos2];
	  subap[cnt]-=calsub[pos2];
	  if(subap[cnt]<calthr[pos2])
	    subap[cnt]=0;
	  cnt++;
	}
      }
    }else{//no thresholding
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	pos=info->npxlCum+i*info->npxlx;
	for(j=loc[3];j<loc[4]; j+=loc[5]){
	  pos2=pos+j;
	  subap[cnt]*=calmult[pos2];
	  subap[cnt]-=calsub[pos2];
	  cnt++;
	}
      }
    }
  }else if(calmult!=NULL){
    if((info->thresholdAlgo==1 || info->thresholdAlgo==2) && calthr!=NULL){
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	pos=info->npxlCum+i*info->npxlx;
	for(j=loc[3];j<loc[4]; j+=loc[5]){
	  pos2=pos+j;
	  subap[cnt]*=calmult[pos2];
	  if(subap[cnt]<calthr[pos2])
	    subap[cnt]=0;
	  cnt++;
	}
      }
    }else{//no thresholding
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	pos=info->npxlCum+i*info->npxlx;
	for(j=loc[3];j<loc[4]; j+=loc[5]){
	  subap[cnt]*=calmult[pos+j];
	  cnt++;
	}
      }
    }
  }else if(calsub!=NULL){
    if((info->thresholdAlgo==1 || info->thresholdAlgo==2 ) && calthr!=NULL){
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	pos=info->npxlCum+i*info->npxlx;
	for(j=loc[3];j<loc[4]; j+=loc[5]){
	  pos2=pos+j;
	  subap[cnt]-=calsub[pos2];
	  if(subap[cnt]<calthr[pos2])
	    subap[cnt]=0;
	  cnt++;
	}
      }
    }else{//no thresholding
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	pos=info->npxlCum+i*info->npxlx;
	for(j=loc[3];j<loc[4]; j+=loc[5]){
	  subap[cnt]-=calsub[pos+j];
	  cnt++;
	}
      }
    }
  }else{//both are null...
    if((info->thresholdAlgo==1 || info->thresholdAlgo==2) && calthr!=NULL){
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	pos=info->npxlCum+i*info->npxlx;
	for(j=loc[3];j<loc[4]; j+=loc[5]){
	  if(subap[cnt]<calthr[pos+j])
	    subap[cnt]=0;
	  cnt++;
	}
      }
    }
  }
  if(info->useBrightest>0 || info->useBrightestArr!=NULL){//we only want to use brightest useBrightest pixels
    applyBrightest(threadInfo);

  }
  //Now do the powerfactor.
  applyPowerFactor(threadInfo);
  return 0;
}

#ifdef USESLOWCALIBRATION

/**
   Calibrates the CCD pixels
*/
int subapPxlCalibrationOld(threadStruct *threadInfo){
  STARTTIMING;
  handleBadPxl(threadInfo);
  subtractDarkNoise(threadInfo);
  applyFlatField(threadInfo);
  subtractBG(threadInfo);
  applyThreshold(threadInfo);
  applyPxlWeighting(threadInfo);
  if(threadInfo->info->useBrightest>0 || threadInfo->info->useBrightestArr!=NULL)
    applyBrightest(threadInfo);
  applyPowerFactor(threadInfo);
  ENDTIMING(subapPxlCalibration);
  return 0;
}
#endif //USESLOWCALIBRATION
/**
   Does nothing
*/
int applyCentWeighting(threadStruct *threadInfo){
  printf("centWeighting does nothing\n");
  return 0;
}

/**
   Calculates the adaptive windows for next time, and updates the current centroids to take into account the position of the current window.
*/
int calcAdaptiveWindow(threadStruct *threadInfo,float cx,float cy){
  infoStruct *info=threadInfo->info;
  //Now use these values to calculate the window position for next time.
  info->adaptiveWinPos[threadInfo->centindx]*=1-info->adaptiveWinGain;
  info->adaptiveWinPos[threadInfo->centindx+1]*=1-info->adaptiveWinGain;
  info->adaptiveWinPos[threadInfo->centindx]+=cx*info->adaptiveWinGain;
  info->adaptiveWinPos[threadInfo->centindx+1]+=cy*info->adaptiveWinGain;
  info->adaptiveCentPos[threadInfo->centindx]=(int)roundf(info->adaptiveWinPos[threadInfo->centindx]);
  info->adaptiveCentPos[threadInfo->centindx+1]=(int)roundf(info->adaptiveWinPos[threadInfo->centindx+1]);
  if(info->maxAdapOffset>0){
    if(info->adaptiveCentPos[threadInfo->centindx]>info->maxAdapOffset)
      info->adaptiveCentPos[threadInfo->centindx]=info->maxAdapOffset;
    else if(info->adaptiveCentPos[threadInfo->centindx]<-info->maxAdapOffset)
      info->adaptiveCentPos[threadInfo->centindx]=-info->maxAdapOffset;
    if(info->adaptiveCentPos[threadInfo->centindx+1]>info->maxAdapOffset)
      info->adaptiveCentPos[threadInfo->centindx+1]=info->maxAdapOffset;
    else if(info->adaptiveCentPos[threadInfo->centindx+1]<-info->maxAdapOffset)
      info->adaptiveCentPos[threadInfo->centindx+1]=-info->maxAdapOffset;
  }

  return 0;
}

/**
   Calculates the adaptive windows for next time, and updates the current centroids to take into account the position of the current window.  Here, the adaptive window moves with a global tip-tilt.
   This should be called by a single thread when all centroids have been computed.
*/
int calcGlobalAdaptiveWindow(infoStruct *info){
  float sumx=0.,sumy=0.;
  int i,group;
  //compute the sum of x and y centroids.
  if(info->adaptiveGroup==NULL || info->nAdaptiveGroups==1){
    for(i=0; i<info->totCents; i+=2){
      sumx+=info->centroids[i];
      sumy+=info->centroids[i+1];
    }
    sumx/=info->totCents/2.;
    sumy/=info->totCents/2.;
    //Now use these values to calculate the window position for next time.
    for(i=0; i<info->totCents; i+=2){
      info->adaptiveWinPos[i]*=1-info->adaptiveWinGain;
      info->adaptiveWinPos[i+1]*=1-info->adaptiveWinGain;
      info->adaptiveWinPos[i]+=sumx*info->adaptiveWinGain;
      info->adaptiveWinPos[i+1]+=sumy*info->adaptiveWinGain;
      info->adaptiveCentPos[i]=(int)roundf(info->adaptiveWinPos[i]);
      info->adaptiveCentPos[i+1]=(int)roundf(info->adaptiveWinPos[i+1]);
      if(info->maxAdapOffset>0){
	if(info->adaptiveCentPos[i]>info->maxAdapOffset)
	  info->adaptiveCentPos[i]=info->maxAdapOffset;
	else if(info->adaptiveCentPos[i]<-info->maxAdapOffset)
	  info->adaptiveCentPos[i]=-info->maxAdapOffset;
	if(info->adaptiveCentPos[i+1]>info->maxAdapOffset)
	  info->adaptiveCentPos[i+1]=info->maxAdapOffset;
	else if(info->adaptiveCentPos[i+1]<-info->maxAdapOffset)
	  info->adaptiveCentPos[i+1]=-info->maxAdapOffset;
      }
    }
  }else{
    if(info->groupSumX!=NULL)
      memset(info->groupSumX,0,sizeof(float)*info->nAdaptiveGroups);
    if(info->groupSumY!=NULL)
      memset(info->groupSumY,0,sizeof(float)*info->nAdaptiveGroups);
    if(info->groupSum!=NULL)
      memset(info->groupSum,0,sizeof(int)*info->nAdaptiveGroups);
    for(i=0; i<info->totCents; i+=2){
      group=info->adaptiveGroup[i/2];
      info->groupSumX[group]+=info->centroids[i];
      info->groupSumY[group]+=info->centroids[i+1];
      info->groupSum[group]++;
    }
    //now get the mean of each group.
    for(i=0; i<info->nAdaptiveGroups; i++){
      info->groupSumX[i]/=info->groupSum[i];
      info->groupSumY[i]/=info->groupSum[i];
    }
    //Now use these values to calculate the window position for next time.
    for(i=0; i<info->totCents; i+=2){
      group=info->adaptiveGroup[i/2];
      info->adaptiveWinPos[i]*=1-info->adaptiveWinGain;
      info->adaptiveWinPos[i+1]*=1-info->adaptiveWinGain;
      info->adaptiveWinPos[i]+=info->groupSumX[group]*info->adaptiveWinGain;
      info->adaptiveWinPos[i+1]+=info->groupSumY[group]*info->adaptiveWinGain;
      info->adaptiveCentPos[i]=(int)roundf(info->adaptiveWinPos[i]);
      info->adaptiveCentPos[i+1]=(int)roundf(info->adaptiveWinPos[i+1]);
      if(info->maxAdapOffset>0){
	if(info->adaptiveCentPos[i]>info->maxAdapOffset)
	  info->adaptiveCentPos[i]=info->maxAdapOffset;
	else if(info->adaptiveCentPos[i]<-info->maxAdapOffset)
	  info->adaptiveCentPos[i]=-info->maxAdapOffset;
	if(info->adaptiveCentPos[i+1]>info->maxAdapOffset)
	  info->adaptiveCentPos[i+1]=info->maxAdapOffset;
	else if(info->adaptiveCentPos[i+1]<-info->maxAdapOffset)
	  info->adaptiveCentPos[i+1]=-info->maxAdapOffset;
      }
    }
  }
  return 0;
}


//Define a function to allow easy indexing into the fftCorrelationPattern array...
#define B(y,x) info->fftCorrelationPattern[info->npxlCum+(loc[0]+y*loc[2])*info->npxlx+loc[3]+x*loc[5]]
/**
   Calculates the correlation of the spot with the reference.
   fftCorrelationPattern is distributed in memory as per subapLocation, and is
   equal to numpy.conjugate(numpy.fft.fft2(numpy.fft.fftshift(corr)))
   where corr is the reference spot pattern (ie what you are correlating to).
   Should be stored in half complex form (reals then imags)
*/
int calcCorrelation(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  globalStruct *glob=threadInfo->globals;
  int *loc;
  int i,j,n,m,neven,meven;
  float *a;
  float r1,r2,r3,r4,r5,r6,r7,r8;
  int *tmp;
  fftwf_plan fPlan=NULL,ifPlan=NULL;
  //This is how the plans should be created (elsewhere).  Will need a different plan for each different sized subap (see subapLocation).  
  //fftwPlan=fftwf_plan_r2r_2d(curnpxly,curnpxlx, double *in, double *out,FFTW_R2HC, FFTW_R2HC, FFTW_ESTIMATE);
  //ifftwPlan=fftwf_plan_r2r_2d(curnpxly,curnpxlx, double *in, double *out,FFTW_HC2R, FFTW_HC2R, FFTW_ESTIMATE);
  for(i=0; i<glob->fftIndexSize; i++){
    if(glob->fftIndex[i*2]==0 || glob->fftIndex[i*2+1]==0){
      break;
    }
    if(glob->fftIndex[i*2]==threadInfo->curnpxlx && glob->fftIndex[i*2+1]==threadInfo->curnpxly){
      fPlan=glob->fftPlanArray[i*2];
      ifPlan=glob->fftPlanArray[i*2+1];
      break;
    }
  }
  if(fPlan==NULL){//need to create the plan...
    //only allow 1 thread to create a plan at a time.
    pthread_mutex_lock(&threadInfo->globals->camMutex);//reuse camMutex...
    //now just check again... (another thread may have created it while we were waiting for the mutex)...
    for(i=0; i<glob->fftIndexSize; i++){
      if(glob->fftIndex[i*2]==0 || glob->fftIndex[i*2+1]==0){
	break;
      }
      if(glob->fftIndex[i*2]==threadInfo->curnpxlx && glob->fftIndex[i*2+1]==threadInfo->curnpxly){
	fPlan=glob->fftPlanArray[i*2];
	ifPlan=glob->fftPlanArray[i*2+1];
	break;
      }
    }
    if(fPlan==NULL){
      if(i==glob->fftIndexSize){//need to make the index larger...
	if((tmp=realloc(glob->fftIndex,sizeof(int)*2*(glob->fftIndexSize+16)))==NULL){
	  printf("realloc of fftIndex failed - exiting\n");
	  exit(1);
	}
	//fill the new stuff with zeros...
	glob->fftIndex=(int*)tmp;
	memset(&glob->fftIndex[i*2],0,sizeof(int)*2*16);
	if((tmp=realloc(glob->fftPlanArray,sizeof(fftwf_plan)*2*(glob->fftIndexSize+16)))==NULL){
	  printf("realloc of fftPlanArray failed - exiting\n");
	  exit(1);
	}
	glob->fftPlanArray=(fftwf_plan*)tmp;
	memset(&glob->fftPlanArray[i*2],0,sizeof(fftwf_plan)*2*16);
	glob->fftIndexSize+=16;
      }
      //now do the planning
      printf("Planning FFTs size %d x %d\n",threadInfo->curnpxly,threadInfo->curnpxlx);
      glob->fftPlanArray[i*2]=fftwf_plan_r2r_2d(threadInfo->curnpxly,threadInfo->curnpxlx,threadInfo->subap,threadInfo->subap,FFTW_R2HC, FFTW_R2HC, FFTW_ESTIMATE);
      glob->fftPlanArray[i*2+1]=fftwf_plan_r2r_2d(threadInfo->curnpxly,threadInfo->curnpxlx,threadInfo->subap,threadInfo->subap,FFTW_HC2R, FFTW_HC2R, FFTW_ESTIMATE);
      glob->fftIndex[i*2]=threadInfo->curnpxlx;
      glob->fftIndex[i*2+1]=threadInfo->curnpxly;
      fPlan=glob->fftPlanArray[i*2];
      ifPlan=glob->fftPlanArray[i*2+1];
      
    }
    pthread_mutex_unlock(&threadInfo->globals->camMutex);//reuse camMutex...

  }

  //FFT the SH image.
  fftwf_execute_r2r(fPlan,threadInfo->subap,threadInfo->subap);
  
  //Now multiply by the reference...
  //This is fairly complicated due to the half complex format.  If you need to edit this, make sure you know what you're doing.
  //Here, we want to use the real subap location rather than the moving one, because this image map in question (the fft'd psf) doesn't move around, and we're just using subap location for convenience rather than having to identify another way to specify it.
  loc=&(info->realSubapLocation[threadInfo->cursubindx*6]);
  a=threadInfo->subap;
  n=threadInfo->curnpxlx;
  m=threadInfo->curnpxly;


  a[0]*=B(0,0);
  neven=(n%2==0);
  meven=(m%2==0);
  if(neven){
    a[n/2]*=B(0,n/2);
  }
  if(meven){
    a[n*m/2]*=B(m/2,0);
    if(neven){
      a[n*m/2+n/2]*=B(m/2,n/2);
    }
  }
  for(i=1; i<(n+1)/2; i++){
    r1=a[i]*B(0,i)-a[n-i]*B(0,n-i);
    r2=a[i]*B(0,n-i)+a[n-i]*B(0,i);
    a[i]=r1;
    a[n-i]=r2;
    if(meven){
      r3=a[m/2*n+i]*B(m/2,i)-a[m/2*n+n-i]*B(m/2,n-i);
      r4=a[m/2*n+i]*B(m/2,n-i)+a[m/2*n+n-i]*B(m/2,i);
      a[m/2*n+i]=r3;
      a[m/2*n+n-i]=r4;
    }
  }
  
  for(i=1; i<(m+1)/2; i++){
    //do the 4 rows/cols that only require 2 values...
    r5=a[i*n]*B(i,0)-a[(m-i)*n]*B(m-i,0);
    r6=a[i*n]*B(m-i,0)+a[(m-i)*n]*B(i,0);
    a[i*n]=r5;
    a[(m-i)*n]=r6;
    if(neven){
      r7=a[i*n+n/2]*B(i,n/2)-a[(m-i)*n+n/2]*B(m-i,n/2);
      r8=a[i*n+n/2]*B(m-i,n/2)+a[(m-i)*n+n/2]*B(i,n/2);
      a[i*n+n/2]=r7;
      a[(m-i)*n+n/2]=r7;
    }
    
    for(j=1; j<(n+1)/2; j++){
      //and now loop over the rest.
      r1=a[i*n+j]*B(i,j)+a[(m-i)*n+n-j]*B(m-i,n-j)-a[i*n+n-j]*B(i,n-j)-a[(m-i)*n+j]*B(m-i,j);
      r2=a[i*n+j]*B(m-i,n-j)+a[(m-i)*n+n-j]*B(i,j)+a[(m-i)*n+j]*B(i,n-j)+a[i*n+n-j]*B(m-i,j);
      r3=a[i*n+j]*B(i,n-j)-a[(m-i)*n+n-j]*B(m-i,j)+a[i*n+n-j]*B(i,j)-a[(m-i)*n+j]*B(m-i,n-j);
      r4=a[i*n+j]*B(m-i,j)-a[(m-i)*n+n-j]*B(i,n-j)+a[(m-i)*n+j]*B(i,j)-a[i*n+n-j]*B(m-i,n-j);
      a[i*n+j]=r1;
      a[(m-i)*n+n-j]=r2;
      a[i*n+n-j]=r3;
      a[(m-i)*n+j]=r4;
    }
  }
  //and now do the inverse fft...
  fftwf_execute_r2r(ifPlan,threadInfo->subap,threadInfo->subap);
  return 0;
}

#undef B
/**
   Applies the chosen threshold algorithm to the correlation
   There are 4 possible thresholding algorithms.  The threshold is either a fixed value, or a fraction of the maximum value found in subap.  This threshold is then either subtracted with everything negative being zero'd, or anything below this threshold is zero'd.
*/
int thresholdCorrelation(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  int i;
  float thresh=0.;
  if(info->correlationThresholdType==CORR_ABS_SUB || info->correlationThresholdType==CORR_ABS_ZERO){
    thresh=info->correlationThreshold;
  }else if(info->correlationThresholdType==CORR_FRAC_SUB || info->correlationThresholdType==CORR_FRAC_ZERO){
    //first find the max.
    for(i=0; i<threadInfo->curnpxl; i++){
      if(thresh<threadInfo->subap[i])
	thresh=threadInfo->subap[i];
    }
    thresh*=info->correlationThreshold;
  }
  if(info->correlationThresholdType==CORR_ABS_SUB || info->correlationThresholdType==CORR_FRAC_SUB){
    for(i=0; i<threadInfo->curnpxl; i++){
      if(threadInfo->subap[i]<thresh)
	threadInfo->subap[i]=0;
      else
	threadInfo->subap[i]-=thresh;
    }
  }else if(info->correlationThresholdType==CORR_ABS_ZERO || info->correlationThresholdType==CORR_FRAC_ZERO){
    for(i=0; i<threadInfo->curnpxl; i++){
      if(threadInfo->subap[i]<thresh)
	threadInfo->subap[i]=0;
    }
  }
  return 0;
}

/**
   Apply a slope linearisation to the measured slope values.
   This has been taken from DASP, slightly modified for out of range values.
*/
int applySlopeLinearisation(threadStruct *threadInfo,float *cx, float *cy){
  infoStruct *info=threadInfo->info;
  int lb,ub;
  float *cd,*cs;
  int centIndx=threadInfo->centindx;
  int nsteps;
  int i;
  //threadInfo->centIndx and +1 is the current centroid.  Note, centIndx is only ever even.
  nsteps=info->centCalnsteps;
  //first do the X centroid.
  lb=info->centCalBounds[centIndx*2];
  ub=info->centCalBounds[centIndx*2+1];
  cd=&info->centCalData[centIndx*nsteps];
  cs=&info->centCalSteps[centIndx*nsteps];
  if(*cx>cd[lb] && *cx<cd[ub]){
    //lies within the boundries specified, so can be linearised.
    i=1+lb;
    while(*cx>cd[i])
      i++;
    *cx=(*cx-cd[i-1])*(cs[i]-cs[i-1])/(cd[i]-cd[i-1])+cs[i-1];
  }else{//otherwise, out of bounds, so just keep as is.
    writeError(threadInfo->globals->rtcErrorBuf,"Slope outside calibration range",SLOPELINERROR,threadInfo->globals->thisiter);
  }


  //then do the Y centroid.
  lb=info->centCalBounds[centIndx*2+2];
  ub=info->centCalBounds[centIndx*2+3];
  cd=&info->centCalData[(centIndx+1)*nsteps];
  cs=&info->centCalSteps[(centIndx+1)*nsteps];
  if(*cy>cd[lb] && *cy<cd[ub]){
    //lies within the boundries specified, so can be linearised.
    i=1+lb;
    while(*cy>cd[i])
      i++;
    *cy=(*cy-cd[i-1])*(cs[i]-cs[i-1])/(cd[i]-cd[i-1])+cs[i-1];
  }else{//otherwise, out of bounds, so just keep as is.
    writeError(threadInfo->globals->rtcErrorBuf,"Slope outside calibration range",SLOPELINERROR,threadInfo->globals->thisiter);
  }
  return 0;
}


/**
   Calculates the slope - currently, basic and adaptive windowing only, centre of gravity estimation (or weighted if weighting supplied, though the method for this hasn't been written yet).
*/
int calcCentroid(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  float sum=0.;
  int i,j;
  float cy,cx;
  float minflux;
  STARTTIMING;
  //If doing correlation centroiding, the idea would be to perform the correlation first here, including any flooring etc of the corelation image.  Then, can apply the chosen centroid algorithm to this here (ie CoG, WCoG etc).

  if(info->centWeighting!=NULL){
    applyCentWeighting(threadInfo);
  }
  cx=0.;
  cy=0.;
  if(threadInfo->globals->fluxThresholdArr!=NULL){
    minflux=threadInfo->globals->fluxThresholdArr[threadInfo->centindx/2];
  }else{
    minflux=threadInfo->globals->fluxThreshold;
  }
  //if(info->windowMode==WINDOWMODE_BASIC || info->windowMode==WINDOWMODE_ADAPTIVE){
  if(info->centroidMode==CENTROIDMODE_CORRELATIONCOG || info->centroidMode==CENTROIDMODE_CORRELATIONGAUSSIAN){
    //do the correlation...
    calcCorrelation(threadInfo);
    //here, before thresholding, should probably store this in a circular buffer that can be sent to user.  Or maybe, this is the calibrated image buffer.
    storeCorrelationSubap(threadInfo);
    thresholdCorrelation(threadInfo);
  }

  if(info->centroidMode==CENTROIDMODE_COG || info->centroidMode==CENTROIDMODE_CORRELATIONCOG){
    for(i=0; i<threadInfo->curnpxly; i++){
      for(j=0; j<threadInfo->curnpxlx; j++){
	sum+=threadInfo->subap[i*threadInfo->curnpxlx+j];
	cx+=j*threadInfo->subap[i*threadInfo->curnpxlx+j];
	cy+=i*threadInfo->subap[i*threadInfo->curnpxlx+j];
      }
    }
    if(sum>minflux){
      cy/=sum;
      cx/=sum;
      cy-=threadInfo->curnpxly/2.-0.5;
      cx-=threadInfo->curnpxlx/2.-0.5;
    }else{
      cy=0;
      cx=0;
    }

  }else if(info->centroidMode==CENTROIDMODE_GAUSSIAN || info->centroidMode==CENTROIDMODE_CORRELATIONGAUSSIAN){
    //do some sort of gaussian fit to the data...
    printf("TODOxxx - gaussian fit to the data to get centroid (not yet implemented)\n");
  }else{
    printf("centroid mode not yet implemented\n");
  }
  if(sum>minflux){
    if(info->windowMode==WINDOWMODE_ADAPTIVE){
      //add centroid offsets to get the overall location correct
      //(i.e. the distance from it's nominal centre).
      cx+=info->adaptiveCentPos[threadInfo->centindx];
      cy+=info->adaptiveCentPos[threadInfo->centindx+1];
      //and calculate adaptive window for next time.
      calcAdaptiveWindow(threadInfo,cx,cy);
    }else if(info->windowMode==WINDOWMODE_GLOBAL){//add the current subap offset here
      cx+=info->adaptiveCentPos[threadInfo->centindx];
      cy+=info->adaptiveCentPos[threadInfo->centindx+1];
    }
    if(info->centCalData!=NULL){//appy centroid linearisation...
      //Here, we apply the centroid linearisation.
      applySlopeLinearisation(threadInfo,&cx,&cy);
    }
    if(info->refCents!=NULL){//subtract reference centroids.
      cx-=info->refCents[threadInfo->centindx];
      cy-=info->refCents[threadInfo->centindx+1];
    }
  }
  info->centroids[threadInfo->centindx]=cx;
  info->centroids[threadInfo->centindx+1]=cy;
  info->flux[threadInfo->centindx/2]=sum;
  ENDTIMING(calcCentroid);
  return 0;
}



/**
   Called after processing of the actuator values has completed. 
   If Kalman filtering is being used, and sending to the DMC, and user actuators aren't specified, this sends the kalman phase to the DMC.  
   Otherwise, it sends actuators either to the DMC or direct to the mirror, depending on whether usingDMC is set or not.
   @return 0
*/
int sendActuators(PostComputeData *p,globalStruct *glob){
  //unsigned short *actsSent=p->actsSent;
  //float *latestDmCommand=p->latestDmCommand;
  float *dmCommand=p->dmCommand;
  int nacts=p->nacts;
  float *userActs;//=p->userActs;
  float *userActsMask=p->userActsMask;
  int addUserActs=p->addUserActs;
  int *userActSeq=p->userActSeq;
  int userActSeqLen=p->userActSeqLen;
  int record=*p->recordCents;
  float *tmp;
  int i,resetRecon;
  //STARTTIMING;
  dprintf("sendActautors %#x %#x %d\n",p->userActs,userActSeq,p->pmxSize);
  if(p->userActs!=NULL){
    if(record!=0 && (userActSeqLen>1 || userActSeq!=NULL)){
      //we're probably doing something like creating a poke matrix here.
      //So, want to take the latest centroids, and add them into the pmx array
      //at the correct point.
      if(p->pmxSize!=p->userActSeqLen*p->totCents*sizeof(float)){
	p->pmxSize=p->userActSeqLen*p->totCents*sizeof(float);
	dprintf("reallocing pmx\n");
	if((tmp=realloc(p->pmx,p->pmxSize))==NULL){
	  printf("couldn't allocate pmx\n");
	  p->pmxSize=0;
	  p->pmx=NULL;
	  //*p->recordCents=0;
	  record=0;
	}
	dprintf("reallocdone\n");
	p->pmx=tmp;
      }
      if(record){
	if(p->seqCnt==0){
	  //printf("%p %d/%d\n",p->pmx,p->actCnt,p->userActSeqLen);
	  memcpy(&p->pmx[p->actCnt*p->totCents],p->centroids,sizeof(float)*p->totCents);
	}else{
#ifdef USEAGBBLAS
	  agb_cblas_saxpy111(p->totCents,p->centroids,&p->pmx[p->actCnt*p->totCents]);
#else
	  cblas_saxpy(p->totCents,1.,p->centroids,1/*info->totCents*/,&p->pmx[p->actCnt*p->totCents],1);
#endif
	}
      }
    }
    if(userActSeq!=NULL){
      p->seqCnt++;
      if(p->seqCnt>=userActSeq[p->actCnt]){//got to the end of this actuators
	if(record!=0)
#ifdef USEAGBBLAS
	  agb_cblas_sscal1(p->totCents,1./userActSeq[p->actCnt],&p->pmx[p->actCnt*p->totCents]);//average the pmx.
#else
	cblas_sscal(p->totCents,1./userActSeq[p->actCnt],&p->pmx[p->actCnt*p->totCents],1);//average the pmx.
#endif
	p->actCnt++;
	p->seqCnt=0;
      }
    }else{//only once per frame... so increment.
      p->actCnt++;
    }
    if(p->actCnt>=userActSeqLen){//start the sequence again.
      //Note, we've probably been creating an interaction matrix here.  So, in that case, the interaction matrix will now be complete, and should be sent to the user.
      //printf("Reshaping\n");
      if(record!=0){
	if(record>0)
	  (*p->recordCents)--;
	i=p->totCents*p->userActSeqLen;
	circReshape(glob->rtcGenericBuf,1,&i,'f');
	circAdd(glob->rtcGenericBuf,p->pmx,p->timestamp,p->thisiter);
      }
      //printf("Reshaped\n");
      p->actCnt=0;
      p->seqCnt=0;
    }
    userActs=&p->userActs[p->actCnt*p->nacts];
  }else{
    userActs=NULL;
    p->actCnt=0;
    p->seqCnt=0;
  }
  pthread_mutex_lock(&glob->libraryMutex);
  if(glob->calibrateFrameFinishedFn!=NULL)
    (*p->calibrateFrameFinishedFn)(glob->calibrateHandle,p->pxlCentInputError);

  if(glob->reconLib!=NULL && glob->reconHandle!=NULL && p->reconFrameFinishedFn!=NULL){
    (*p->reconFrameFinishedFn)(glob->reconHandle,glob->arrays->dmCommand,p->pxlCentInputError);
  }
  pthread_mutex_unlock(&glob->libraryMutex);
  if(p->pxlCentInputError==0){
    //we send the dmCommand here, because sendActuators can alter this depending on the bleed gain... and we don't want that to be seen - this is the result of the reconstruction process only...
    //xxxMoveToCorrectPlace;
    circAdd(glob->rtcMirrorBuf,p->dmCommand,p->timestamp,p->thisiter);//actsSent);
  }
  resetRecon=0;
  if(userActs==NULL || addUserActs!=0){
    //Either no user actuators specified, or we're adding user actuators.
    //In either case, we want to put latestDmCommand into actsSent.
    if(p->actsRequired!=NULL){//add in the required actuators - this typically will be set if using as a figure sensore (should probably also use reconmode_TRUTH then too...
      pthread_mutex_lock(&p->actsRequiredMutex);
      if(p->actsRequired!=NULL){//further test required here incase the library has been closed while we were blocked waiting for it...
	//first, make a copy of dmCommand - this copy can then be used by the thread that is activated by the actuator demands library to produce the new values...
	memcpy(p->dmCommandSave,dmCommand,sizeof(float)*nacts);
	
	if(p->figureGainArr==NULL){
#ifdef USEAGBBLAS
	  agb_cblas_saxpy11(nacts,p->figureGain,p->actsRequired,dmCommand);//dmCommand=dmCommand + actsRequired*figureGain.
#else
	  cblas_saxpy(nacts,p->figureGain,p->actsRequired,1,dmCommand,1);//dmCommand=dmCommand + actsRequired*figureGain.
#endif
	}else{
	  for(i=0; i<p->nacts; i++){
	    dmCommand[i]+=p->figureGainArr[i]*p->actsRequired[i];
	  }
	}
	glob->figureFrame[1]=glob->figureFrame[0];//copy into the second buffer...
      }
      pthread_mutex_unlock(&p->actsRequiredMutex);
      
    }
    //we can now use dmCommand as a temporary buffer...
    //So, apply the bleed gain algorithm to dmCommand, add 0.5 to it and test for uint16 clipping, and then convert to uint16.
    //This way, we check correctly for uint16 overflow.
    //Note, only after doing bleeding and user actuators do we store the dmCommand into latestDmCommand
    //An update here - no point including the user actuators in latestDMCommand if we want to use them to apply perturbations to the DM - eg for testing the system.  So, instead compute the bleed algorithm, put this into latestDMCommand, and then apply the user actuators.
    
    

    if(userActsMask!=NULL){
      if(userActs==NULL){//apply mask
	for(i=0; i<nacts; i++){
	  dmCommand[i]*=userActsMask[i];
	}
      }else{//apply mask  and actuators
	for(i=0; i<nacts; i++){
	  dmCommand[i]*=userActsMask[i];
	  dmCommand[i]+=userActs[i];
	}
      }
    }else{//appyl actuators
      if(userActs!=NULL){//add the user actuators...
	for(i=0; i<nacts; i++){
	  dmCommand[i]+=userActs[i];
	}
      }
    }
  }else{//userActs is specified, and we're replacing, not adding.
    //memcpy(actsSent,userActs,sizeof(unsigned short)*nacts);
    memcpy(dmCommand,userActs,sizeof(float)*nacts);

    //It is necessary in this case to let the recon library know that the actuator values aren't being used, so that it can reset itself...
    resetRecon=1;
  }
  if(p->closeLoop){
    //send actuators direct to the mirror.
    pthread_mutex_lock(&glob->libraryMutex);
    if(p->mirrorHandle!=NULL && glob->mirrorLib!=NULL && p->mirrorSendFn!=NULL)
      //(*p->mirrorSendFn)(p->mirrorHandle,nacts,actsSent,p->thisiter);
      p->nclipped=(*p->mirrorSendFn)(p->mirrorHandle,nacts,dmCommand,p->thisiter,p->timestamp,p->pxlCentInputError);
    pthread_mutex_unlock(&glob->libraryMutex);
    /*
      }
      }*/
  }else{
    //todo - maybe set this to midrange or v0?
    //Npte - we will copy into latestDMCommand twice here.  But doesn't matter too much since we're not closing the loop, so the additional latency isn't important.
    //It is necessary in this case to let the recon library know that the actuator values aren't being used, so that it can reset itself...
    resetRecon=1;
  }
  if(resetRecon){
    pthread_mutex_lock(&glob->libraryMutex);
    if(p->calibrateOpenLoopFn!=NULL)
      (*p->calibrateOpenLoopFn)(glob->calibrateHandle);
    if(glob->reconLib!=NULL && glob->reconHandle!=NULL && p->reconOpenLoopFn!=NULL){
      (*p->reconOpenLoopFn)(glob->reconHandle);
    }
    pthread_mutex_unlock(&glob->libraryMutex);
  }
  dprintf("sendActautors done\n");
  //ENDTIMING(sendActuators);
  return 0;
}

/**
   This is run as a separate thread, and is used only in the case that this RTC is acting as a figure sensor, receiving DM demands.  The library responsible for receiving these demands should call pthread_cond_signal, which will wake up this thread, which will then add the new demands to the current dmCommandSave, and send this (in a threadsafe way) to the mirrorSendFn.
   Sit in a loop, waiting for new demands...
*/
int figureThread(PostComputeData *p){
  int nacts=p->nacts;
  float *userActs;//=p->userActs;
  float *userActsMask=p->userActsMask;
  int addUserActs=p->addUserActs;
  int i;
  float *dmCommand;
  printf("figure thread starting\n");
  pthread_mutex_lock(&p->actsRequiredMutex);
  pthread_cond_wait(&p->actsRequiredCond,&p->actsRequiredMutex);
  while(p->go){
    //wait for mirror demands to arrive.
    dmCommand=p->dmCommandFigure;
    nacts=p->nacts;
    if(p->userActs!=NULL)
      userActs=&p->userActs[p->actCnt*nacts];
    else
      userActs=NULL;//=p->userActs;
    userActsMask=p->userActsMask;
    addUserActs=p->addUserActs;
    if((userActs==NULL || addUserActs!=0) && p->actsRequired!=NULL && p->actsRequired!=NULL && dmCommand!=NULL && p->dmCommandSave!=NULL){
      //first, make a copy of the saved dmCommand.
      if(p->figureGainArr==NULL){
	for(i=0; i<nacts; i++){
	  dmCommand[i]=p->dmCommandSave[i]+p->actsRequired[i]*p->figureGain;
	}
      }else{
	for(i=0; i<nacts; i++){
	  dmCommand[i]=p->dmCommandSave[i]+p->actsRequired[i]*p->figureGainArr[i];
	}
      }
      if(userActsMask!=NULL){
	if(userActs==NULL){//apply mask
	  for(i=0; i<nacts; i++){
	    dmCommand[i]*=userActsMask[i];
	  }
	}else{//apply mask  and actuators
	  for(i=0; i<nacts; i++){
	    dmCommand[i]*=userActsMask[i];
	    dmCommand[i]+=userActs[i];
	  }
	}
      }else{//appyl actuators
	if(userActs!=NULL){//add the user actuators...
	  for(i=0; i<nacts; i++){
	    dmCommand[i]+=userActs[i];
	  }
	}
      } 
      if(p->closeLoop){
	pthread_mutex_lock(p->libraryMutex);
	if(p->mirrorHandle!=NULL && p->mirrorLib!=NULL && p->mirrorSendFn!=NULL)
	  p->nclipped=(*p->mirrorSendFn)(p->mirrorHandle,nacts,dmCommand,p->thisiter,p->timestamp,p->pxlCentInputError);
	pthread_mutex_unlock(p->libraryMutex);
      }
    }
    pthread_cond_wait(&p->actsRequiredCond,&p->actsRequiredMutex);
  }
  pthread_mutex_unlock(&p->actsRequiredMutex);
  printf("figureThread ending\n");
  return 0;
}

int writeStatusBuf(globalStruct *glob,int paused,int closeLoop){
  char tmp[80];
  int pos,i;
  memset(glob->statusBuf,0,STATUSBUFSIZE);
  pos=0;
  for(i=0; (i<glob->ncam && pos<80); i++){
    pos+=snprintf(&tmp[pos],79-pos,"%d ",glob->camframeno[i]);
  }
  for(i=0; (i<glob->ncam && pos<80); i++){
    pos+=snprintf(&tmp[pos],79-pos,"%d ",glob->centframeno[i]);
  }
  snprintf(glob->statusBuf,STATUSBUFSIZE,"%d+1 threads\nClipped: %d\nIteration: %d/%d\nMax time %gs at iter %d\nFrame time %gs (%gHz)\nFrame numbers %s\n%s\n%s\n%s\n%s\n%s\n%s\n%s\nFS:%u\n%s",glob->nthreads,glob->nclipped,glob->thisiter,glob->niters,glob->maxtime,glob->maxtimeiter,glob->frameTime,1/glob->frameTime,tmp,paused==0?"Running...":"Paused...",glob->camHandle==NULL?"No cam":"Cam open",glob->camFraming?"Cam framing":"Cam not framing",glob->mirrorHandle==NULL?"No mirror":"Mirror connected",glob->centHandle==NULL?"No external centroider":"External centroider",glob->calibrateHandle==NULL?"No calibration":"Calibration",glob->reconLib==NULL?"No reconstructor":"Reconstructor in use",glob->figureFrame[1],closeLoop?"Loop closed":"Loop open");
  return 0;
}

int setThreadAffinity(int *threadAffinityList,int *threadPriorityList,int n,int ncpu){
  int i;
  cpu_set_t mask;
  int threadAffinity;
  struct sched_param param;
  if(threadAffinityList==NULL)
    threadAffinity=-1;
  else
    threadAffinity=threadAffinityList[n];
  CPU_ZERO(&mask);
  for(i=0; i<ncpu; i++){
    if(((threadAffinity)>>i)&1){
      CPU_SET(i,&mask);
    }
  }
  //printf("Thread affinity %d\n",threadAffinity&0xff);
  if(sched_setaffinity(0,sizeof(cpu_set_t),&mask))
    printf("Error in sched_setaffinity: %s\n",strerror(errno));
  if(threadPriorityList==NULL)
    param.sched_priority=(n==0)+1;
  else
    param.sched_priority=threadPriorityList[n];
  //if(sched_setparam(0,&param)){
  if(sched_setscheduler(0,SCHED_RR,&param)){
    //printf("Error in sched_setparam: %s\n",strerror(errno));
  }
  if(pthread_setschedparam(pthread_self(),SCHED_RR,&param))
    printf("error in pthread_setschedparam - maybe run as root?\n");
  return 0;
}



/**
   Called when a buffer swap is required.  Reads the new buffer.
   Only the first thread needs to do this.
*/

int updateBufferIndex(threadStruct *threadInfo){//,int updateIndex){
  //Assumes the buffer is an array, with header then data.  Header contains:
  //name (16 bytes), type(1), startaddr(4), nbytes(4), ndim(4), shape(24), lcomment(4).  Here, we find name, check that type and nbytes match what we expect, and move the pointer to startaddr (which is index in bytes from start of array).

  //This is called only by the first thread for each camera.  The
  //first thread overall has updateIndex set.
  //BUFFERVARIABLEINDX i;
  //int j;//,k,offset;
  //char *buf=threadInfo->globals->buffer[threadInfo->globals->curBuf]->buf;
  //int *indx;
  //int nbytes;
  int i;
  int err=0;
  globalStruct *globals=threadInfo->globals;
  int nfound;
  dprintf("updating buffer index\n");//insz=%d\n",updateIndex);
  nfound=bufferGetIndex(globals->buffer[threadInfo->globals->curBuf],NBUFFERVARIABLES,globals->paramNames,globals->bufferHeaderIndex,globals->bufferValues,globals->bufferDtype,globals->bufferNbytes);
  if(nfound!=NBUFFERVARIABLES){
    err=1;
    printf("Didn't find all buffer entries:\n");
    for(i=0; i<NBUFFERVARIABLES;i++){
      if(globals->bufferHeaderIndex[i]<0)
	printf("Missing %16s\n",&globals->paramNames[i*BUFNAMESIZE]);
    }
  }
  /*
    indx=globals->bufferHeaderIndex;

    //if(updateIndex){//only the first thread needs to do this...
    memset(indx,-1,sizeof(int)*NBUFFERVARIABLES);
    //First get the indexes for each variable...
    j=0;
    while(j<NHDR && buf[j*16]!='\0'){
    if(strncmp(&buf[j*16],"ncam",16)==0){
    indx[NCAM]=j;
    }else if(strncmp(&buf[j*16],"nacts",16)==0){
    indx[NACTS]=j;
    }else if(strncmp(&buf[j*16],"nsub",16)==0){
    indx[NSUB]=j;
    //}else if(strncmp(&buf[j*16],"nsuby",16)==0){
    //indx[NSUBY]=j;
    }else if(strncmp(&buf[j*16],"npxlx",16)==0){
    indx[NPXLX]=j;
    }else if(strncmp(&buf[j*16],"npxly",16)==0){
    indx[NPXLY]=j;
    }else if(strncmp(&buf[j*16],"refCentroids",16)==0){
    indx[REFCENTROIDS]=j;
    }else if(strncmp(&buf[j*16],"subapLocation",16)==0){
    indx[SUBAPLOCATION]=j;
    }else if(strncmp(&buf[j*16],"bgImage",16)==0){
    indx[BGIMAGE]=j;
    }else if(strncmp(&buf[j*16],"darkNoise",16)==0){
    indx[DARKNOISE]=j;
    }else if(strncmp(&buf[j*16],"flatField",16)==0){
    indx[FLATFIELD]=j;
    }else if(strncmp(&buf[j*16],"thresholdAlgorithm",16)==0){
    indx[THRESHOLDALGORITHM]=j;
    }else if(strncmp(&buf[j*16],"thresholdValue",16)==0){
    indx[THRESHOLDVALUE]=j;
    }else if(strncmp(&buf[j*16],"powerFactor",16)==0){
    indx[POWERFACTOR]=j;
    }else if(strncmp(&buf[j*16],"centroidWeighting",16)==0){
    indx[CENTROIDWEIGHTING]=j;
    }else if(strncmp(&buf[j*16],"windowMode",16)==0){
    indx[WINDOWMODE]=j;
    }else if(strncmp(&buf[j*16],"subapFlag",16)==0){
    indx[SUBAPFLAG]=j;
    }else if(strncmp(&buf[j*16],"go",16)==0){
    indx[GO]=j;
    }else if(strncmp(&buf[j*16],"pxlCnt",16)==0){
    indx[PXLCNT]=j;
    }else if(strncmp(&buf[j*16],"centroidMode",16)==0){
    indx[CENTROIDMODE]=j;
    }else if(strncmp(&buf[j*16],"pause",16)==0){
    indx[PAUSE]=j;
    }else if(strncmp(&buf[j*16],"printTime",16)==0){
    indx[PRINTTIME]=j;
    }else if(strncmp(&buf[j*16],"ncamThreads",16)==0){
    indx[NCAMTHREADS]=j;
    }else if(strncmp(&buf[j*16],"switchRequested",16)==0){
    indx[SWITCHREQUESTED]=j;
    }else if(strncmp(&buf[j*16],"actuators",16)==0){
    indx[USERACTS]=j;
    }else if(strncmp(&buf[j*16],"fakeCCDImage",16)==0){
    indx[FAKECCDIMAGE]=j;
    }else if(strncmp(&buf[j*16],"threadAffinity",16)==0){
    indx[THREADAFFINITY]=j;
    }else if(strncmp(&buf[j*16],"threadPriority",16)==0){
    indx[THREADPRIORITY]=j;
    }else if(strncmp(&buf[j*16],"delay",16)==0){
    indx[DELAY]=j;
    }else if(strncmp(&buf[j*16],"maxClipped",16)==0){
    indx[MAXCLIPPED]=j;
    }else if(strncmp(&buf[j*16],"clearErrors",16)==0){
    indx[CLEARERRORS]=j;
    }else if(strncmp(&buf[j*16],"camerasOpen",16)==0){
    indx[CAMERASOPEN]=j;
    }else if(strncmp(&buf[j*16],"camerasFraming",16)==0){
    indx[CAMERASFRAMING]=j;
    }else if(strncmp(&buf[j*16],"cameraParams",16)==0){
    indx[CAMERAPARAMS]=j;
    }else if(strncmp(&buf[j*16],"cameraName",16)==0){
    indx[CAMERANAME]=j;
    }else if(strncmp(&buf[j*16],"mirrorOpen",16)==0){
    indx[MIRROROPEN]=j;
    }else if(strncmp(&buf[j*16],"mirrorName",16)==0){
    indx[MIRRORNAME]=j;
    }else if(strncmp(&buf[j*16],"frameno",16)==0){
    indx[FRAMENO]=j;
    }else if(strncmp(&buf[j*16],"switchTime",16)==0){
    indx[SWITCHTIME]=j;
    }else if(strncmp(&buf[j*16],"adaptiveWinGain",16)==0){
    indx[ADAPTIVEWINGAIN]=j;
    }else if(strncmp(&buf[j*16],"correlationThresholdType",16)==0){
    indx[CORRELATIONTHRESHOLDTYPE]=j;
    }else if(strncmp(&buf[j*16],"correlationThreshold",16)==0){
    indx[CORRELATIONTHRESHOLD]=j;
    }else if(strncmp(&buf[j*16],"fftCorrelationPattern",16)==0){
    indx[FFTCORRELATIONPATTERN]=j;
    //}else if(strncmp(&buf[j*16],"nsubapsTogether",16)==0){
    //indx[NSUBAPSTOGETHER]=j;
    }else if(strncmp(&buf[j*16],"nsteps",16)==0){
    indx[NSTEPS]=j;
    }else if(strncmp(&buf[j*16],"closeLoop",16)==0){
    indx[CLOSELOOP]=j;
    }else if(strncmp(&buf[j*16],"mirrorParams",16)==0){
    indx[MIRRORPARAMS]=j;
    }else if(strncmp(&buf[j*16],"addActuators",16)==0){
    indx[ADDUSERACTS]=j;
    }else if(strncmp(&buf[j*16],"actSequence",16)==0){
    indx[USERACTSEQ]=j;
    }else if(strncmp(&buf[j*16],"recordCents",16)==0){
    indx[RECORDCENTS]=j;
    }else if(strncmp(&buf[j*16],"pxlWeight",16)==0){
    indx[PXLWEIGHT]=j;
    }else if(strncmp(&buf[j*16],"averageImg",16)==0){
    indx[AVERAGEIMG]=j;
    }else if(strncmp(&buf[j*16],"centOpen",16)==0){
    indx[CENTOPEN]=j;
    }else if(strncmp(&buf[j*16],"centFraming",16)==0){
    indx[CENTFRAMING]=j;
    }else if(strncmp(&buf[j*16],"centParams",16)==0){
    indx[CENTPARAMS]=j;
    }else if(strncmp(&buf[j*16],"centName",16)==0){
    indx[CENTNAME]=j;
    }else if(strncmp(&buf[j*16],"actuatorMask",16)==0){
    indx[USERACTSMASK]=j;
    }else if(strncmp(&buf[j*16],"dmDescription",16)==0){
    //do nothing...
    }else if(strncmp(&buf[j*16],"averageCent",16)==0){
    indx[AVERAGECENT]=j;
    }else if(strncmp(&buf[j*16],"calmult",16)==0){
    indx[CALMULT]=j;
    }else if(strncmp(&buf[j*16],"calsub",16)==0){
    indx[CALSUB]=j;
    }else if(strncmp(&buf[j*16],"calthr",16)==0){
    indx[CALTHR]=j;
    }else if(strncmp(&buf[j*16],"centCalData",16)==0){
    indx[CENTCALDATA]=j;
    }else if(strncmp(&buf[j*16],"centCalBounds",16)==0){
    indx[CENTCALBOUNDS]=j;
    }else if(strncmp(&buf[j*16],"centCalSteps",16)==0){
    indx[CENTCALSTEPS]=j;
    }else if(strncmp(&buf[j*16],"figureOpen",16)==0){
    indx[FIGUREOPEN]=j;
    }else if(strncmp(&buf[j*16],"figureName",16)==0){
    indx[FIGURENAME]=j;
    }else if(strncmp(&buf[j*16],"figureParams",16)==0){
    indx[FIGUREPARAMS]=j;
    }else if(strncmp(&buf[j*16],"reconName",16)==0){
    indx[RECONNAME]=j;
    }else if(strncmp(&buf[j*16],"fluxThreshold",16)==0){
    indx[FLUXTHRESHOLD]=j;
    }else if(strncmp(&buf[j*16],"printUnused",16)==0){
    indx[PRINTUNUSED]=j;
    }else if(strncmp(&buf[j*16],"useBrightest",16)==0){
    indx[USEBRIGHTEST]=j;
    }else if(strncmp(&buf[j*16],"figureGain",16)==0){
    indx[FIGUREGAIN]=j;
    }else if(strncmp(&buf[j*16],"reconlibOpen",16)==0){
    indx[RECONLIBOPEN]=j;
    }else if(strncmp(&buf[j*16],"maxAdapOffset",16)==0){
    indx[MAXADAPOFFSET]=j;
    }else if(strncmp(&buf[j*16],"version",16)==0){
    indx[VERSION]=j;
    }else if(strncmp(&buf[j*16],"currentErrors",16)==0){
    indx[CURRENTERRORS]=j;
    }else if(strncmp(&buf[j*16],"reconParams",16)==0){
    indx[RECONPARAMS]=j;
    }else if(strncmp(&buf[j*16],"adaptiveWinGroup",16)==0){
    indx[ADAPTIVEGROUP]=j;
    }else if(strncmp(&buf[j*16],"comment",16)==0){
    //do nothing...
    }else{
    if(strncmp(&buf[j*16],"rmx",16)==0 || strncmp(&buf[j*16],"E",16)==0 || strncmp(&buf[j*16],"gain",16)==0){
    //do nothing
    }else if(globals->printUnused){
    printf("Ignoring: %s (used by .so interfaces?)\n",&buf[j*16]);
    }
    }
    j++;
    }
    //}
    for(j=0; j<NBUFFERVARIABLES; j++){
    if(indx[j]==-1){
    //if(updateIndex){
    printf("ERROR buffer index %d %s\n",j,paramNames[j]);
    writeError(globals->rtcErrorBuf,"Error in parameter buffer",PARAMERROR,globals->thisiter);
    //}
    err=-1;
    //just check that we've not been told to stop...
    if(indx[GO]!=-1){
    if(buf[NHDR*16+indx[GO]]=='i' && NBYTES[indx[GO]]==4){
    globals->go=*((int*)(buf+START[indx[GO]]));
    }//else{
    //err=i;
    // printf("go error\n");
    //}
    }
    }
    }*/
  //if(err)
  return err;
}


/**
   Called when a buffer swap is required.  Reads the new buffer.
*/
int updateBuffer(threadStruct *threadInfo,int updateIndex){
  //Assumes the buffer is an array, with header then data.  Header contains:
  //name (16 bytes), type(1), startaddr(4), nbytes(4), ndim(4), shape(24), lcomment(4).  Here, we find name, check that type and nbytes match what we expect, and move the pointer to startaddr (which is index in bytes from start of array).

  //This is called only by the first thread for each camera.  The
  //first thread overall has updateIndex set.
  BUFFERVARIABLEINDX i;
  int j;//,k,offset;
  int m;
  //char *buf=threadInfo->globals->buffer[threadInfo->globals->curBuf]->buf;
  int *indx;
  int nb;
  int err=0;
  //short *tmps;
  //float *tmpf;
  struct timeval t1;
  double timestamp;
  infoStruct *info=threadInfo->info;
  globalStruct *globals=threadInfo->globals;
  int nusedSubaps;
  void **values=globals->bufferValues;
  int *nbytes=globals->bufferNbytes;
  char *dtype=globals->bufferDtype;
  dprintf("updating buffer insz=%d\n",updateIndex);

  indx=globals->bufferHeaderIndex;

  //first get ncam... (for checking array sizes etc)
  i=NCAM;
  if(dtype[i]=='i' && nbytes[i]==4){
    if(info->ncam!=*((int*)values[i])){
      printf("Cannont change ncam\n");
      err=-2;
    }
  }else{
    printf("ncam error\n");
    err=-2;
  }

  //printf("ncam=%d\n",info->ncam);
  //now get nsubx etc...
  i=NACTS;
  if(dtype[i]=='i' && nbytes[i]==4){
    info->nacts=*((int*)values[i]);
  }else{
    printf("nacts error\n");
    err=NACTS;
  }
  i=NSUB;
  if(dtype[i]=='i' && nbytes[i]==4*info->ncam){
    info->nsubList=(int*)values[i];
    info->nsub=info->nsubList[info->cam];
  }else{
    printf("nsub error\n");
    err=NSUB;
  }
  //i=NSUBY;
  //if(dtype[i]=='i' && nbytes[i]==4*info->ncam){
  //	info->nsubyList=(int*)values[i];
  //	info->nsuby=info->nsubyList[info->cam];
  //}else{
  //  printf("nsuby error\n");
  //  err=NSUBY;
  //}
  i=NPXLX;
  if(dtype[i]=='i' && nbytes[i]==4*info->ncam){
    info->npxlxList=(int*)values[i];
    info->npxlx=info->npxlxList[info->cam];
  }else{
    printf("npxlx error\n");
    err=NPXLX;
  }
  i=NPXLY;
  if(dtype[i]=='i' && nbytes[i]==4*info->ncam){
    info->npxlyList=(int*)values[i];
    info->npxly=info->npxlyList[info->cam];
  }else{
    printf("npxly error\n");
    err=NPXLY;
  }
  
  //compute number of subaps/pixels up to this point...
  info->subCumIndx=0;
  info->npxlCum=0;
  for(j=0;j<info->cam; j++){
    info->subCumIndx+=info->nsubList[j];
    info->npxlCum+=info->npxlxList[j]*info->npxlyList[j];
  }
  //and the total number
  info->nsubaps=info->subCumIndx;
  info->totPxls=info->npxlCum;
  for(j=info->cam; j<info->ncam; j++){
    info->nsubaps+=info->nsubList[j];
    info->totPxls+=info->npxlxList[j]*info->npxlyList[j];
  }
  i=SUBAPFLAG;
  if(dtype[i]=='i' && nbytes[i]==sizeof(int)*info->nsubaps){
    info->subapFlagArr=(int*)values[i];
    info->subflag=&(info->subapFlagArr[info->subCumIndx]);
    //printf("subflag %p %p\n",info->subapFlagArr,info->subflag);
  }else{
    printf("subapFlag error\n");
    err=SUBAPFLAG;
  }
  //and work out the cumulative cent index...
  info->centCumIndx=0;//sum of all subaps used up to this camera.
  //info->centCumIndx=cblas_sasum(info->subCumIndx,info->subapFlagArr,1);
  for(j=0; j<info->subCumIndx; j++){//sum the subaps...
    info->centCumIndx+=info->subapFlagArr[j];
  }
  info->centCumIndx*=2;
  info->totCents=0;
  nusedSubaps=0;
  for(j=info->subCumIndx; j<info->nsubaps; j++)
    nusedSubaps+=info->subapFlagArr[j];
  info->totCents=2*nusedSubaps;
  info->totCents+=info->centCumIndx;//and now sum the rest of them

  i=REFCENTROIDS;
  nb=nbytes[i];
  if(nb==0){
    info->refCents=NULL;
  }else if(dtype[i]=='f' && nb==sizeof(float)*info->totCents){
    info->refCents=(float*)values[i];
  }else{
    printf("refCentroids error\n");
    err=REFCENTROIDS;
  }

  i=CLOSELOOP;
  if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
    info->closeLoop=*((int*)values[i]);
  }else{
    printf("closeLoop error\n");
    err=CLOSELOOP;
  }
  i=PAUSE;
  if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
    info->pause=*((int*)values[i]);
    globals->ppause=((int*)values[i]);
  }else{
    printf("pause error\n");
    globals->ppause=NULL;
    err=i;
  }
  i=PRINTTIME;
  if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
    info->printTime=*((int*)values[i]);
    globals->maxtime=0;
  }else{
    printf("printTime error\n");
    err=i;
  }
  
  i=SUBAPLOCATION;
  if(dtype[i]=='i' && nbytes[i]==info->nsubaps*6*sizeof(int)){
    info->realSubapLocation=(int*)values[i];
  }else{
    printf("subapLocation error\n");
    err=i;
  }
  i=PXLCNT;
  if(dtype[i]=='i' && nbytes[i]==info->nsubaps*sizeof(int)){
    info->pxlCnt=(int*)values[i];
  }else{
    printf("pxlCnt error\n");
    err=i;
  }
  i=BGIMAGE;
  nb=nbytes[i];
  if(nb==0)
    info->background=NULL;
  else if(dtype[i]=='f' && nb/sizeof(float)==info->totPxls){
    info->backgroundArr=(float*)values[i];
    info->background=&(info->backgroundArr[info->npxlCum]);
  }else{
    printf("bgImage error\n");
    err=i;
  }
  i=DARKNOISE;
  nb=nbytes[i];
  if(nb==0)
    info->darkNoise=NULL;
  else if(dtype[i]=='f' && nb/sizeof(float)==info->totPxls){
    info->darkNoiseArr=(float*)values[i];
    info->darkNoise=&(info->darkNoiseArr[info->npxlCum]);
  }else{
    printf("darkNoise error\n");
    err=i;
  }
  i=FLATFIELD;
  nb=nbytes[i];
  if(nb==0)
    info->flatField=NULL;
  else if(dtype[i]=='f' && nb/sizeof(float)==info->totPxls){
    info->flatFieldArr=(float*)values[i];
    info->flatField=&(info->flatFieldArr[info->npxlCum]);
  }else{
    printf("flatField error\n");
    err=i;
  }
  i=THRESHOLDALGO;
  if(dtype[i]=='i' && nbytes[i]==4){
    info->thresholdAlgo=*((int*)values[i]);
  }else{
    printf("thresholdAlgorithm error\n");
    err=i;
  }
  i=THRESHOLDVALUE;
  nb=nbytes[i];
  if((dtype[i])=='f'){
    if(nb==sizeof(float)){
      info->threshold=*((float*)values[i]);
      info->thresholdArr=NULL;
      info->thresholdArrType=0;
    }else if(nb==sizeof(float)*info->nsubaps){
      info->threshold=0;
      info->thresholdArr=(float*)values[i];
      info->thresholdArrType=0;
    }else if(nb==sizeof(float)*info->totPxls){
      info->threshold=0;
      info->thresholdArr=(float*)values[i];
      info->thresholdArrType=1;
    }else{
      err=i;
      printf("threshold Error\n");
    }
  }else{
    err=i;
    printf("threshold error\n");
  }
  i=POWERFACTOR;
  if(dtype[i]=='f' && nbytes[i]==4){
    info->powerFactor=*((float*)values[i]);
  }else{
    err=i;
    printf("powerFactor error\n");
  }
  i=CENTROIDWEIGHT;
  nb=nbytes[i];
  if(nb==0)
    info->centWeighting=NULL;
  else if(dtype[i]=='f' && nb==4){
    info->centWeighting=((float*)values[i]);
  }else{
    err=i;
    printf("centWeighting error\n");
  }
  i=WINDOWMODE;
  nb=nbytes[i];
  info->resetAdaptiveWindows=0;
  if(nb!=0 && dtype[i]=='s'){
    if(strncmp(values[i],"basic",nb)==0){
      info->windowMode=WINDOWMODE_BASIC;
    }else if(strncmp(values[i],"adaptive",nb)==0){
      if(info->windowMode!=WINDOWMODE_ADAPTIVE){
	info->windowMode=WINDOWMODE_ADAPTIVE;
	//if(updateIndex)
	info->resetAdaptiveWindows=1;//all threads can set this, but only the first overall will use it...
      }
    }else if(strncmp(values[i],"global",nb)==0){
      if(info->windowMode!=WINDOWMODE_GLOBAL){
	info->windowMode=WINDOWMODE_GLOBAL;
	//if(updateIndex)
	info->resetAdaptiveWindows=1;//all threads can set this, but only the first overall will use it...
      }
    }else{
      info->windowMode=WINDOWMODE_ERROR;
      printf("windowMode string error (unrecognised)\n");
    }
  }else{
    err=i;
    printf("windowMode error\n");
  }
  if(updateIndex){
    i=GO;
    if(dtype[i]=='i' && nbytes[i]==4){
      globals->go=*((int*)values[i]);
    }else{
      err=i;
      printf("go error\n");
    }
  }
  i=CENTROIDMODE;
  nb=nbytes[i];
  if(nb!=0 && dtype[i]=='s'){
    //info->useWPU=0;
    if(strncmp(values[i],"WPU",nb)==0){
      //info->useWPU=1;
      info->centroidMode=CENTROIDMODE_WPU;
    }else if(strncmp(values[i],"CoG",nb)==0){
      info->centroidMode=CENTROIDMODE_COG;
    }else if(strncmp(values[i],"Gaussian",nb)==0){
      info->centroidMode=CENTROIDMODE_GAUSSIAN;
    }else if(strncmp(values[i],"Correlation CoG",nb)==0){
      info->centroidMode=CENTROIDMODE_CORRELATIONCOG;
    }else if(strncmp(values[i],"Correlation gaussian",nb)==0){
      info->centroidMode=CENTROIDMODE_CORRELATIONGAUSSIAN;
    }else{
      info->centroidMode=CENTROIDMODE_ERROR;
      printf("Unrecognised centroidMode\n");
    }
  }else{
    err=i;
    printf("centroidMode error\n");
  }

  i=USERACTS;
  if(nbytes[i]==0){
    info->userActs=NULL;
    info->userActSeqLen=0;
  }else if(dtype[i]=='f' && nbytes[i]%(sizeof(float)*info->nacts)==0){
    info->userActs=((float*)values[i]);
    //get the number of actuator sequences...
    info->userActSeqLen=nbytes[i]/(sizeof(float)*info->nacts);
  }else{ 
    printf("userActs/actuators error %c %d %d\n",dtype[i],nbytes[i],(int)sizeof(float)*info->nacts);
    err=USERACTS;
  }
  i=USERACTSMASK;
  if(nbytes[i]==0){
    info->userActsMask=NULL;
  }else if(dtype[i]=='f' && nbytes[i]%(sizeof(float)*info->nacts)==0){
    info->userActsMask=((float*)values[i]);
  }else{ 
    printf("userActsMask/actuatorMask error %c %d %d\n",dtype[i],nbytes[i],(int)sizeof(float)*info->nacts);
    err=USERACTSMASK;
  }

  i=FAKECCDIMAGE;
  if(nbytes[i]==0){
    info->fakeCCDImage=NULL;
  }else if(dtype[i]=='i' && nbytes[i]==sizeof(int)*info->totPxls){
    info->fakeCCDImage=((int*)values[i]);
  }else{ 
    printf("fakeCCDImage error\n");
    err=FAKECCDIMAGE;
  }
  i=THREADAFFINITY;
  nb=nbytes[i];
  if(nb==0){
    info->threadAffinityList=NULL;
  }else if(dtype[i]=='i' && nbytes[i]==sizeof(int)*(globals->nthreads+1)){
    info->threadAffinityList=(int*)values[i];
  }else{
    printf("threadAffinity error\n");
    err=THREADAFFINITY;
  }
  i=THREADPRIORITY;
  nb=nbytes[i];
  if(nb==0){
    info->threadPriorityList=NULL;
  }else if(dtype[i]=='i' && nbytes[i]==sizeof(int)*(globals->nthreads+1)){
    info->threadPriorityList=(int*)values[i];
  }else{
    printf("threadPriority error\n");
    err=THREADPRIORITY;
  }
  i=DELAY;
  if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
    info->delay=*((int*)values[i]);
  }else{
    err=i;
    printf("delay error\n");
  }
  i=MAXCLIPPED;
  if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
    info->maxClipped=*((int*)values[i]);
  }else{
    err=i;
    printf("maxClipped error\n");
  }
  if(updateIndex){//only one thread needs to clear errors... and only one needs to copy the priority and affinity lists for use with the preprocessing thread.
    globals->threadPriorityList=info->threadPriorityList;
    globals->threadAffinityList=info->threadAffinityList;
    i=CLEARERRORS;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int) && globals->rtcErrorBuf!=NULL){
      globals->rtcErrorBuf->errFlag &= ~(*((int*)values[i]));
      *((int*)values[i])=0;
    }else{//don't set an error flag here, since its not fatal.
      printf("clearErrors not found - not clearing\n");
    }
    i=CURRENTERRORS;//note, this is only updated when buffer is swapped.
    if(dtype[i]=='i' && nbytes[i]==sizeof(int) && globals->rtcErrorBuf!=NULL){
      *((int*)values[i])=globals->rtcErrorBuf->errFlag;
    }else{
      printf("Unable to write current errors\n");
    }
  }

  i=CAMERASOPEN;
  if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
    info->camerasOpen=((int*)values[i]);
    //*((int*)values[i])=0;
  }else{
    err=i;
    printf("camerasOpen error\n");
  }
  i=CAMERASFRAMING;
  if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
    info->camerasFraming=((int*)values[i]);
    //*((int*)values[i])=0;
  }else{
    err=i;
    printf("camerasFraming error\n");
  }

  i=CAMERAPARAMS;
  nb=nbytes[i];
  if(nb==0){
    info->cameraParams=NULL;
    info->cameraParamsCnt=0;
  }else if(dtype[i]=='i'){
    info->cameraParams=((int*)values[i]);
    info->cameraParamsCnt=nb/sizeof(int);
  }else{
    err=i;
    printf("cameraParams error\n");
  }

  i=CENTOPEN;
  if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
    info->centroidersOpen=((int*)values[i]);
    //*((int*)values[i])=0;
  }else{
    err=i;
    printf("centroidersOpen error\n");
  }
  i=CENTFRAMING;
  if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
    info->centroidersFraming=((int*)values[i]);
    //*((int*)values[i])=0;
  }else{
    err=i;
    printf("centoidersFraming error\n");
  }

  i=CENTPARAMS;
  nb=nbytes[i];
  if(nb==0){
    info->centroidersParams=NULL;
    info->centroidersParamsCnt=0;
  }else if(dtype[i]=='i'){
    info->centroidersParams=((int*)values[i]);
    info->centroidersParamsCnt=nb/sizeof(int);
  }else{
    err=i;
    printf("centroidersParams error\n");
  }

  i=MIRRORPARAMS;
  nb=nbytes[i];
  if(nb==0){
    info->mirrorParams=NULL;
    info->mirrorParamsCnt=0;
  }else if(dtype[i]=='i'){
    info->mirrorParams=((int*)values[i]);
    info->mirrorParamsCnt=nb/sizeof(int);
  }else{
    err=i;
    printf("mirrorParams error\n");
  }
  i=MIRROROPEN;
  if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
    info->mirrorOpen=((int*)values[i]);
    //*((int*)values[i])=0;
  }else{
    err=i;
    printf("mirrorOpen error\n");
  }
  if(updateIndex){//only one thread needs to set framenumber and time...
    i=FRAMENO;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      *((int*)values[i])=globals->thisiter;
    }else{
      err=i;
      printf("frameno error\n");
    }
    i=SWITCHTIME;
    if(dtype[i]=='d' && nbytes[i]==sizeof(double)){
      gettimeofday(&t1,NULL);
      timestamp=t1.tv_sec+t1.tv_usec*1e-6;
      *((double*)values[i])=timestamp;
    }else{
      err=i;
      printf("switchTime error\n");
    }
  }
  i=ADAPTIVEWINGAIN;
  if(dtype[i]=='f' && nbytes[i]==sizeof(float)){
    info->adaptiveWinGain=*((float*)values[i]);
  }else{
    err=i;
    printf("adaptiveWinGain error\n");
  }
  i=CORRTHRESHTYPE;
  if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
    info->correlationThresholdType=*((int*)values[i]);
  }else{
    err=i;
    printf("correlationThresholdType error\n");
  }
  i=CORRTHRESH;
  if(dtype[i]=='f' && nbytes[i]==sizeof(float)){
    info->correlationThreshold=*((float*)values[i]);
  }else{
    err=i;
    printf("correlationThreshold error\n");
  }
  i=CORRFFTPATTERN;
  nb=nbytes[i];
  if(nb==0)
    info->fftCorrelationPattern=NULL;
  else if(dtype[i]=='f' && nb/sizeof(float)==info->totPxls){
    info->fftCorrelationPatternArr=(float*)values[i];
    info->fftCorrelationPattern=info->fftCorrelationPatternArr;//&(info->fftCorrelationPatternArr[info->npxlCum]);
  }else{
    printf("fftCorrelationPattern error\n");
    err=i;
  }
  //i=NSUBAPSTOGETHER;
  //if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
  //  info->nsubapsTogether=*((int*)values[i]);
  //}else{
  //  err=i;
  //  printf("nsubapsTogether error\n");
  //}
  i=NSTEPS;
  if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
    info->nsteps=*((int*)values[i]);
  }else{
    err=i;
    printf("nsteps error\n");
  }
  i=ADDUSERACTS;
  if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
    info->addUserActs=*((int*)values[i]);
  }else{ 
    printf("addUserActs/addActuators error\n");
    err=ADDUSERACTS;
  }
  i=USERACTSEQ;
  if(nbytes[i]==0){
    info->userActSeq=NULL;
  }else if(dtype[i]=='i' && nbytes[i]==sizeof(int)*info->userActSeqLen){
    info->userActSeq=((int*)values[i]);
  }else{ 
    printf("userActSeq/actSequence error %d %d\n",nbytes[i],(int)(info->userActSeqLen*sizeof(int)));
    err=USERACTSEQ;
  }
  i=RECORDCENTS;
  if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
    info->recordCents=((int*)values[i]);
  }else{ 
    printf("recordCents error\n");
    err=RECORDCENTS;
  }
  i=PXLWEIGHT;
  nb=nbytes[i];
  if(nb==0)
    info->pxlweight=NULL;
  else if(dtype[i]=='f' && nb==sizeof(float)*info->totPxls){
    info->pxlweightArr=(float*)values[i];
    info->pxlweight=&(info->pxlweightArr[info->npxlCum]);
  }else{
    printf("pxlweight error\n");
    err=i;
  }
  i=AVERAGEIMG;
  if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
    info->averageImg=((int*)values[i]);
    info->nAvImg=*info->averageImg;
  }else{ 
    printf("averageImg error\n");
    err=AVERAGEIMG;
  }
  i=AVERAGECENT;
  if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
    info->averageCent=((int*)values[i]);
    info->nAvCent=*info->averageCent;
  }else{ 
    printf("averageCent error\n");
    err=AVERAGECENT;
  }
  i=CALMULT;
  nb=nbytes[i];
  if(nb==0)
    info->calmult=NULL;
  else if(dtype[i]=='f' && nb/sizeof(float)==info->totPxls){
    info->calmult=(float*)values[i];
  }else{
    printf("calmult error\n");
    err=i;
  }
  i=CALSUB;
  nb=nbytes[i];
  if(nb==0)
    info->calsub=NULL;
  else if(dtype[i]=='f' && nb/sizeof(float)==info->totPxls){
    info->calsub=(float*)values[i];
  }else{
    printf("calsub error\n");
    err=i;
  }
  i=CALTHR;
  nb=nbytes[i];
  if(nb==0)
    info->calthr=NULL;
  else if(dtype[i]=='f' && nb/sizeof(float)==info->totPxls){
    info->calthr=(float*)values[i];
  }else{
    printf("calthr error\n");
    err=i;
  }
  i=CENTCALDATA;
  info->centCalnsteps=0;
  nb=nbytes[i];
  if(nb==0)
    info->centCalData=NULL;
  else if(dtype[i]=='f'){
    info->centCalnsteps=nb/sizeof(float)/info->totCents;
    if(nb==sizeof(float)*info->totCents*info->centCalnsteps)
      info->centCalData=(float*)values[i];
    else{
      printf("centCalData error\n");
      err=i;
      info->centCalnsteps=0;
    }
  }else{
    printf("centCalData error\n");
    err=i;
  }
  i=CENTCALSTEPS;
  nb=nbytes[i];
  if(nb==0 || info->centCalnsteps==0)
    info->centCalSteps=NULL;
  else if(dtype[i]=='f' && nb/sizeof(float)==info->totCents*info->centCalnsteps){
    info->centCalSteps=(float*)values[i];
  }else{
    printf("centCalSteps error\n");
    err=i;
  }
  i=CENTCALBOUNDS;
  nb=nbytes[i];
  if(nb==0 || info->centCalnsteps==0)
    info->centCalBounds=NULL;
  else if(dtype[i]=='i' && nb/sizeof(int)==2*info->totCents){
    info->centCalBounds=(int*)values[i];
  }else{
    printf("centCalBounds error\n");
    err=i;
  }
  if(info->centCalData==NULL || info->centCalSteps==NULL || info->centCalBounds==NULL){
    info->centCalData=NULL;
    info->centCalSteps=NULL;
    info->centCalBounds=NULL;
  }
  
  //This stuff is for when using as a figure sensor...
  i=FIGUREOPEN;
  if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
    info->figureOpen=((int*)values[i]);
    //*((int*)values[i])=0;
  }else{
    err=i;
    printf("figureOpen error\n");
  }
  i=FIGUREPARAMS;
  nb=nbytes[i];
  if(nb==0){
    info->figureParams=NULL;
    info->figureParamsCnt=0;
  }else if(dtype[i]=='i'){
    info->figureParams=((int*)values[i]);
    info->figureParamsCnt=nb/sizeof(int);
  }else{
    err=i;
    printf("figureParams error\n");
  }
  i=USEBRIGHTEST;
  nb=nbytes[i];
  if(dtype[i]=='i'){
    if(nb==sizeof(int)){
      info->useBrightest=*((int*)values[i]);
      info->useBrightestArr=NULL;
    }else if(nb==sizeof(int)*info->nsubaps){
      info->useBrightest=0;
      info->useBrightestArr=((int*)values[i]);
    }else{
      printf("useBrightest error\n");
      info->useBrightestArr=NULL;
      info->useBrightest=0;
      err=i;
    }
  }else{
    printf("useBrightest error\n");
    info->useBrightestArr=NULL;
    info->useBrightest=0;
    err=i;
  }
  i=FIGUREGAIN;
  nb=nbytes[i];
  if(nb==0){
    info->figureGain=1;
    info->figureGainArr=NULL;
  }else if(nb==sizeof(float) && dtype[i]=='f'){
    info->figureGain=*((float*)values[i]);
    info->figureGainArr=NULL;
  }else if(nb==sizeof(float)*info->nacts && dtype[i]=='f'){
    info->figureGain=0;
    info->figureGainArr=((float*)values[i]);
  }else{
    printf("figureGain error\n");
    err=i;
    info->figureGainArr=NULL;
  }
  i=RECONLIBOPEN;
  if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
    info->reconlibOpen=((int*)values[i]);
    //*((int*)values[i])=0;
  }else{
    err=i;
    printf("reconlibOpen error\n");
  }
  i=MAXADAPOFFSET;
  if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
    info->maxAdapOffset=*((int*)values[i]);
  }else{
    err=i;
    printf("maxAdapOffset error\n");
  }
  i=ADAPTIVEGROUP;
  nb=nbytes[i];
  if(nb==0){
    info->adaptiveGroup=NULL;
    info->nAdaptiveGroups=1;
  }else if(nb==sizeof(int)*info->totCents/2 && dtype[i]=='i'){
    info->adaptiveGroup=((int*)values[i]);
    m=0;
    for(j=0; j<info->totCents/2; j++){
      if(info->adaptiveGroup[j]>m)
	m=info->adaptiveGroup[j];
    }
    info->nAdaptiveGroups=m+1;
    if(info->adaptiveGroupSize<info->nAdaptiveGroups){//need to re malloc mem.
      if(info->groupSumX!=NULL)
	free(info->groupSumX);
      info->groupSumX=malloc(sizeof(float)*info->nAdaptiveGroups);
      if(info->groupSumY!=NULL)
	free(info->groupSumY);
      info->groupSumY=malloc(sizeof(float)*info->nAdaptiveGroups);
      if(info->groupSum!=NULL)
	free(info->groupSum);
      info->groupSum=malloc(sizeof(int)*info->nAdaptiveGroups);
      if(info->groupSumX==NULL || info->groupSumY==NULL || info->groupSum==NULL){
	printf("unable to malloc groupSum for adaptiveWinGroup\n");
	err=i;
	info->adaptiveGroupSize=0;
	info->adaptiveGroup=NULL;
	info->nAdaptiveGroups=1;
      }else{
	info->adaptiveGroupSize=info->nAdaptiveGroups;
      }
    }
  }else{
    printf("adaptiveWinGroup error: nbytes=%d should be %d type %c should be i\n",nb,sizeof(int)*info->totCents/2,dtype[i]);
    err=i;
    info->adaptiveGroup=NULL;
    info->nAdaptiveGroups=1;
  }
  if(updateIndex){//only 1 thread needs to do this one
    i=VERSION;
    nb=nbytes[i];
    if(nb>0 && dtype[i]=='s'){
      //write the versions into here.
      strncpy(values[i],CVSID,nb-1);
    }

    i=CAMERANAME;
    nb=nbytes[i];
    if(nb==0){
      globals->cameraName=NULL;
    }else if(dtype[i]=='s'){
      globals->cameraName=values[i];
    }else{
      printf("cameraName error\n");
      globals->cameraName=NULL;
      err=i;
    }
    i=CENTNAME;
    nb=nbytes[i];
    if(nb==0){
      globals->centName=NULL;
    }else if(dtype[i]=='s'){
      globals->centName=values[i];
    }else{
      printf("centroidersName error\n");
      globals->centName=NULL;
      err=i;
    }
    i=FIGURENAME;
    nb=nbytes[i];
    if(nb==0){
      globals->figureName=NULL;
    }else if(dtype[i]=='s'){
      globals->figureName=values[i];
    }else{
      printf("figureName error\n");
      globals->figureName=NULL;
      err=i;
    }

    i=MIRRORNAME;
    nb=nbytes[i];
    if(nb==0){
      globals->mirrorName=NULL;
    }else if(dtype[i]=='s'){
      globals->mirrorName=values[i];
    }else{
      printf("mirrorName error\n");
      globals->mirrorName=NULL;
      err=i;
    }
    i=RECONNAME;
    nb=nbytes[i];
    if(nb==0){
      globals->reconName=NULL;
    }else if(dtype[i]=='s'){
      globals->reconName=values[i];
    }else{
      printf("reconName error\n");
      globals->reconName=NULL;
      err=i;
    }
    i=RECONPARAMS;
    nb=nbytes[i];
    if(nb==0){
      globals->reconParams=NULL;
      globals->reconParamsCnt=0;
    }else if(dtype[i]=='i'){
      globals->reconParams=((int*)values[i]);
      globals->reconParamsCnt=nb/sizeof(int);
    }else{
      err=i;
      printf("reconParams error\n");
    }

    i=FLUXTHRESHOLD;
    nb=nbytes[i];
    if(dtype[i]=='f'){
      if(nb==sizeof(float)){
	globals->fluxThresholdArr=NULL;
	globals->fluxThreshold=*(float*)values[i];
      }else if(nb==sizeof(float)*info->totCents/2){
	globals->fluxThresholdArr=(float*)values[i];;
	globals->fluxThreshold=0.;
      }else{
	globals->fluxThresholdArr=NULL;
	printf("fluxThreshold error\n");
	err=i;
      }
    }else{
      globals->fluxThresholdArr=NULL;
      printf("fluxThreshold error\n");
      err=i;
    }
    i=PRINTUNUSED;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      globals->printUnused=*((int*)values[i]);
    }else{
      printf("printUnused error\n");
      err=i;
    }
  }
  return err;
}
/**
   Opens the dynamic library for image calibration...
*/
int updateCalibrateLibrary(threadStruct *threadInfo){
  globalStruct *glob=threadInfo->globals;
  infoStruct *info=threadInfo->info;
  int open=0,err=0,doneParams=0;
  if(glob->calibrateNameOpen==NULL){
    if(glob->calibrateName!=NULL && *info->calibratelibOpen==1){
      glob->calibrateNameOpen=strdup(glob->calibrateName);
      open=1;
    }
  }else{//already one open
    if(glob->calibrateName==NULL || *info->calibratelibOpen==0){
      free(glob->calibrateNameOpen);
      glob->calibrateNameOpen=NULL;
      *info->calibratelibOpen=0;
      open=1;
    }else{
      if(strcmp(glob->calibrateName,glob->calibrateNameOpen)){//different name...
	free(glob->calibrateNameOpen);
	glob->calibrateNameOpen=strdup(glob->calibrateName);
	open=1;
      }
    }
  }
  pthread_mutex_lock(&glob->calibrateMutex);
  if(open){
    //first close existing, if it is open...
    if(glob->calibrateLib!=NULL){
      //close existing library.
      (*glob->calibrateCloseFn)(&threadInfo->globals->calibrateHandle);
      if(dlclose(glob->calibrateLib)!=0){
	printf("Failed to close calibrate library - ignoring\n");
      }
      glob->calibrateLib=NULL;
    }
    //and then open the new one.
    if(glob->calibrateNameOpen!=NULL){
      if((glob->calibrateLib=dlopen(glob->calibrateNameOpen,RTLD_LAZY))==NULL){
	printf("Failed to open calibrate library %s: %s\n",glob->calibrateNameOpen,dlerror());
	err=1;
      }else{//now get the symbols...
	int nsym=0;
	if((*(void**)(&glob->calibrateOpenFn)=dlsym(glob->calibrateLib,"calibrateOpen"))==NULL){
	  printf("dlsym failed for calibrateOpen (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->calibrateCloseFn)=dlsym(glob->calibrateLib,"calibrateClose"))==NULL){
	  printf("dlsym failed for calibrateClose (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->calibrateNewParamFn)=dlsym(glob->calibrateLib,"calibrateNewParam"))==NULL){
	  printf("dlsym failed for calibrateNewParam (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->calibrateNewFrameFn)=dlsym(glob->calibrateLib,"calibrateNewFrame"))==NULL){
	  printf("dlsym failed for calibrateNewFrame (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->calibrateStartFrameFn)=dlsym(glob->calibrateLib,"calibrateStartFrame"))==NULL){
	  printf("dlsym failed for calibrateStartFrame (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->calibrateNewSubapFn)=dlsym(glob->calibrateLib,"calibrateNewSubap"))==NULL){
	  printf("dlsym failed for calibrateNewSubap (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->calibrateEndFrameFn)=dlsym(glob->calibrateLib,"calibrateEndFrame"))==NULL){
	  printf("dlsym failed for calibrateEndFrame (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->calibrateFrameFinishedFn)=dlsym(glob->calibrateLib,"calibrateFrameFinished"))==NULL){
	  printf("dlsym failed for calibrateFrameFinished (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->calibrateFrameFinishedSyncFn)=dlsym(glob->calibrateLib,"calibrateFrameFinishedSync"))==NULL){
	  printf("dlsym failed for calibrateFrameFinishedSync (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->calibrateOpenLoopFn)=dlsym(glob->calibrateLib,"calibrateOpenLoop"))==NULL){
	  printf("dlsym failed for calibrateOpenLoop (non-fatal)\n");
	}else{nsym++;}
	if(err!=0 || nsym==0){//close the dll... - either error, or no symbols found.
	  if(glob->calibrateLib!=NULL  && dlclose(glob->calibrateLib)!=0){
	    printf("Failed to close calibrate library - ignoring\n");
	  }
	  glob->calibrateLib=NULL;
	}
      }
      if(glob->calibrateLib!=NULL){//do initialisation...
	doneParams=1;//the init function will do parameters...
	if((err=(*glob->calibrateOpenFn)(glob->calibrateNameOpen,glob->calibrateParamsCnt,glob->calibrateParams,glob->buffer[glob->curBuf],glob->rtcErrorBuf,glob->shmPrefix,glob->arrays,&glob->calibrateHandle,glob->nthreads,glob->thisiter))){
	  printf("Error calling calibrateOpen function\n");
	  if(dlclose(glob->calibrateLib)!=0){
	    printf("Failed to close calibrate library - ignoring\n");
	  }
	  glob->calibrateLib=NULL;
	}
      }
      if(glob->calibrateLib==NULL){
	free(glob->calibrateNameOpen);
	glob->calibrateNameOpen=NULL;
      }
	  
    }
    if(glob->calibrateLib==NULL){
      *info->calibratelibOpen=0;
      glob->calibrateCloseFn=NULL;
      glob->calibrateOpenFn=NULL;
      glob->calibrateNewParamFn=NULL;
      glob->calibrateNewFrameFn=NULL;
      glob->calibrateStartFrameFn=NULL;
      glob->calibrateNewSubapFn=NULL;
      glob->calibrateEndFrameFn=NULL;
      glob->calibrateFrameFinishedFn=NULL;
      glob->calibrateFrameFinishedSyncFn=NULL;
      glob->calibrateOpenLoopFn=NULL;
    }
  }
  if(glob->calibrateLib!=NULL && doneParams==0)
    (*glob->calibrateNewParamFn)(threadInfo->globals->calibrateHandle,glob->buffer[glob->curBuf],glob->thisiter,glob->arrays);
  pthread_mutex_unlock(&glob->calibrateMutex);
  return err;



}

/**
   Opens the dynamic library for reconstructor...
*/
int updateReconLibrary(threadStruct *threadInfo){
  globalStruct *glob=threadInfo->globals;
  infoStruct *info=threadInfo->info;
  int open=0,err=0,doneParams=0;
  if(glob->reconNameOpen==NULL){
    if(glob->reconName!=NULL && *info->reconlibOpen==1){
      glob->reconNameOpen=strdup(glob->reconName);
      open=1;
    }
  }else{//already one open
    if(glob->reconName==NULL || *info->reconlibOpen==0){
      free(glob->reconNameOpen);
      glob->reconNameOpen=NULL;
      *info->reconlibOpen=0;
      open=1;
    }else{
      if(strcmp(glob->reconName,glob->reconNameOpen)){//different name...
	free(glob->reconNameOpen);
	glob->reconNameOpen=strdup(glob->reconName);
	open=1;
      }
    }
  }
  if(open){
    //first close existing, if it is open...
    if(glob->reconLib!=NULL){
      //close existing library.
      (*glob->reconCloseFn)(&threadInfo->globals->reconHandle);
      if(dlclose(glob->reconLib)!=0){
	printf("Failed to close recon library - ignoring\n");
      }
      glob->reconLib=NULL;
    }
    //and then open the new one.
    if(glob->reconNameOpen!=NULL){
      if((glob->reconLib=dlopen(glob->reconNameOpen,RTLD_LAZY))==NULL){
	printf("Failed to open recon library %s: %s\n",glob->reconNameOpen,dlerror());
	err=1;
      }else{//now get the symbols...
	if((*(void**)(&glob->reconOpenFn)=dlsym(glob->reconLib,"reconOpen"))==NULL){
	  printf("dlsym failed for reconOpen\n");
	  err=1;
	}
	if((*(void**)(&glob->reconCloseFn)=dlsym(glob->reconLib,"reconClose"))==NULL){
	  printf("dlsym failed for reconClose\n");
	  err=1;
	}
	if((*(void**)(&glob->reconNewParamFn)=dlsym(glob->reconLib,"reconNewParam"))==NULL){
	  printf("dlsym failed for reconNewParam\n");
	  err=1;
	}
	if((*(void**)(&glob->reconNewFrameFn)=dlsym(glob->reconLib,"reconNewFrame"))==NULL){
	  printf("dlsym failed for reconNewFrame\n");
	  err=1;
	}
	if((*(void**)(&glob->reconStartFrameFn)=dlsym(glob->reconLib,"reconStartFrame"))==NULL){
	  printf("dlsym failed for reconStartFrame\n");
	  err=1;
	}
	if((*(void**)(&glob->reconNewSlopesFn)=dlsym(glob->reconLib,"reconNewSlopes"))==NULL){
	  printf("dlsym failed for reconNewSlopes\n");
	  err=1;
	}
	if((*(void**)(&glob->reconEndFrameFn)=dlsym(glob->reconLib,"reconEndFrame"))==NULL){
	  printf("dlsym failed for reconEndFrame\n");
	  err=1;
	}
	if((*(void**)(&glob->reconFrameFinishedFn)=dlsym(glob->reconLib,"reconFrameFinished"))==NULL){
	  printf("dlsym failed for reconFrameFinished\n");
	  err=1;
	}
	if((*(void**)(&glob->reconFrameFinishedSyncFn)=dlsym(glob->reconLib,"reconFrameFinishedSync"))==NULL){
	  printf("dlsym failed for reconFrameFinishedSync\n");
	  err=1;
	}
	if((*(void**)(&glob->reconOpenLoopFn)=dlsym(glob->reconLib,"reconOpenLoop"))==NULL){
	  printf("dlsym failed for reconOpenLoop\n");
	  err=1;
	}
	if(err){//close the dll...
	  if(glob->reconLib!=NULL  && dlclose(glob->reconLib)!=0){
	    printf("Failed to close recon library - ignoring\n");
	  }
	  glob->reconLib=NULL;
	}
      }
      if(glob->reconLib!=NULL){//do initialisation...
	doneParams=1;//the init function will do parameters...
	if((err=(*glob->reconOpenFn)(glob->reconNameOpen,glob->reconParamsCnt,glob->reconParams,glob->buffer[glob->curBuf],glob->rtcErrorBuf,glob->shmPrefix,glob->arrays,&glob->reconHandle,glob->nthreads,glob->thisiter,threadInfo->info->totCents))){
	  printf("Error calling reconOpen function\n");
	  if(dlclose(glob->reconLib)!=0){
	    printf("Failed to close recon library - ignoring\n");
	  }
	  glob->reconLib=NULL;
	}
      }
      if(glob->reconLib==NULL){
	free(glob->reconNameOpen);
	glob->reconNameOpen=NULL;
      }
	  
    }
    if(glob->reconLib==NULL){
      *info->reconlibOpen=0;
      glob->reconCloseFn=NULL;
      glob->reconOpenFn=NULL;
      glob->reconNewParamFn=NULL;
      glob->reconNewFrameFn=NULL;
      glob->reconStartFrameFn=NULL;
      glob->reconNewSlopesFn=NULL;
      glob->reconEndFrameFn=NULL;
      glob->reconFrameFinishedFn=NULL;
      glob->reconFrameFinishedSyncFn=NULL;
      glob->reconOpenLoopFn=NULL;
    }
  }
  if(glob->reconLib!=NULL && doneParams==0)
    (*glob->reconNewParamFn)(threadInfo->globals->reconHandle,glob->buffer[glob->curBuf],glob->thisiter,glob->arrays,threadInfo->info->totCents);
  return err;
}


/**
   Opens the dynamic library, and then uses this to open the camera
*/
int openCameras(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  globalStruct *glob=threadInfo->globals;
  int cerr=0,open=0;
  int doneParam=0;
  if(glob->cameraNameOpen==NULL){
    if(glob->cameraName!=NULL && *info->camerasOpen==1){
      glob->cameraNameOpen=strdup(glob->cameraName);
      open=1;
    }
  }else{//already one open
    if(glob->cameraName==NULL || *info->camerasOpen==0){
      free(glob->cameraNameOpen);
      glob->cameraNameOpen=NULL;
      open=1;
    }else if(strcmp(glob->cameraName,glob->cameraNameOpen)){//different name...
      free(glob->cameraNameOpen);
      glob->cameraNameOpen=strdup(glob->cameraName);
      open=1;
    }
  }
  if(open){//first open the shared object library...
    if(glob->cameraLib!=NULL){//attempt to close existing open library...
      if(glob->camHandle!=NULL)
	(*glob->camCloseFn)(&glob->camHandle);
      glob->camHandle=NULL;
      if(dlclose(glob->cameraLib)!=0){
	printf("Failed to close camera dynamic library - ignoring\n");
      }
      glob->cameraLib=NULL;
    }
    if(glob->cameraNameOpen!=NULL){
      printf("Opening %s\n",glob->cameraNameOpen);
      if((glob->cameraLib=dlopen(glob->cameraNameOpen,RTLD_LAZY))==NULL){
	printf("Failed to open camera dynamic library %s: %s\n",glob->cameraNameOpen,dlerror());
	cerr=1;
      }else{
	printf("Getting symbols\n");
	//now get the symbols... we want camOpen,camClose,camStartFraming,camStopFraming,camNewFrame and camWaitPixels.
	if((*(void**)(&glob->camOpenFn)=dlsym(glob->cameraLib,"camOpen"))==NULL){
	  printf("dlsym failed for camOpen\n");
	  cerr=1;
	}
	if((*(void**)(&glob->camCloseFn)=dlsym(glob->cameraLib,"camClose"))==NULL){
	  printf("dlsym failed for camClose\n");
	  cerr=1;
	}
	if((*(void**)(&glob->camStartFramingFn)=dlsym(glob->cameraLib,"camStartFraming"))==NULL){
	  printf("dlsym failed for camStartFraming\n");
	  cerr=1;
	}
	if((*(void**)(&glob->camStopFramingFn)=dlsym(glob->cameraLib,"camStopFraming"))==NULL){
	  printf("dlsym failed for camStopFraming\n");
	  cerr=1;
	}
	if((*(void**)(&glob->camNewFrameFn)=dlsym(glob->cameraLib,"camNewFrame"))==NULL){
	  printf("dlsym failed for camNewFrame\n");
	  cerr=1;
	}
	if((*(void**)(&glob->camWaitPixelsFn)=dlsym(glob->cameraLib,"camWaitPixels"))==NULL){
	  printf("dlsym failed for camWaitPixels\n");
	  cerr=1;
	}
	if((*(void**)(&glob->camNewParamFn)=dlsym(glob->cameraLib,"camNewParam"))==NULL){
	  printf("camera library has now newParam function - continuing...\n");
	  //this is now an error, just a warning.
	}
	printf("Got symbols %d\n",cerr);
	if(cerr==1){//close the dll
	  if(glob->cameraLib!=NULL && dlclose(glob->cameraLib)!=0){
	    printf("Failed to close camera dynamic library - ignoring\n");
	  }
	  glob->cameraLib=NULL;
	}
      }
      if(glob->cameraLib!=NULL){//do the initialisation
	//doneParams=1;//the init function will do parameters(?)
	//if(glob->pxlbufs==NULL){
	//}
	printf("Initialising cam\n");
	if((cerr=(*glob->camOpenFn)(glob->cameraName,info->cameraParamsCnt,info->cameraParams,glob->buffer[glob->curBuf],glob->rtcErrorBuf,glob->shmPrefix,glob->arrays,&glob->camHandle,info->totPxls,glob->arrays->pxlbufs,info->ncam,info->npxlxList,info->npxlyList,glob->camframeno))!=0){//This is probably a blocking call - ie might wait until the camera reaches a certain temerature or whatever...
	  printf("Error calling camOpenFn\n");
	}
	doneParam=1;
	printf("done init %d\n",cerr);
      }
      if(cerr){
	writeError(glob->rtcErrorBuf,"Error opening camera",-1,glob->thisiter);
	*info->camerasOpen=0;//write to the parambuf
	*info->camerasFraming=0;//write to the parambuf
	if(glob->cameraLib!=NULL && dlclose(glob->cameraLib)!=0){
	  printf("Failed to close camera dynamic library - ignoring\n");
	}
	glob->cameraLib=NULL;
      }
      if(glob->cameraLib==NULL){
	free(glob->cameraNameOpen);
	glob->cameraNameOpen=NULL;
      }
    }
    if(glob->cameraLib==NULL){
      glob->camOpenFn=NULL;
      glob->camCloseFn=NULL;
      glob->camStartFramingFn=NULL;
      glob->camStopFramingFn=NULL;
      glob->camNewFrameFn=NULL;
      glob->camWaitPixelsFn=NULL;
      glob->camNewParamFn=NULL;
    }
  }
  if(glob->cameraLib!=NULL && doneParam==0 && glob->camNewParamFn!=NULL){
    (*glob->camNewParamFn)(glob->camHandle,glob->buffer[glob->curBuf],glob->thisiter,glob->arrays);
  }
  
  return cerr;
}
int openCentroiders(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  globalStruct *glob=threadInfo->globals;
  int cerr=0,open=0,offset,j,k;
  int doneParams=0;
  if(glob->centNameOpen==NULL){
    if(glob->centName!=NULL && *info->centroidersOpen==1){
      glob->centNameOpen=strdup(glob->centName);
      open=1;
    }
  }else{//already open
    if(glob->centName==NULL || *info->centroidersOpen==0){
      free(glob->centNameOpen);
      glob->centNameOpen=NULL;
      open=1;
    }else if(strcmp(glob->centName,glob->centNameOpen)){//different name
      free(glob->centNameOpen);
      glob->centNameOpen=strdup(glob->centName);
      open=1;
    }
  }
  if(open){
    if(glob->centLib!=NULL){//attempt to close existing
      if(glob->centHandle!=NULL){
	(*glob->centCloseFn)(&glob->centHandle);
      }
      glob->centHandle=NULL;
      if(dlclose(glob->centLib)!=0){
	printf("Failed to close cent dynamic library - ignoring\n");
      }
      glob->centLib=NULL;
    }
    if(glob->centNameOpen!=NULL){
      //Now open the new library
      if((glob->centLib=dlopen(glob->centNameOpen,RTLD_LAZY))==NULL){
	printf("Failed to open cent dynamic library %s: %s\n",glob->centNameOpen,dlerror());
	cerr=1;
      }else{
	//now get the symbols... we want camOpen,camClose,camStartFraming,camStopFraming,camNewFrame and camWaitPixels.
	if((*(void**)(&glob->centOpenFn)=dlsym(glob->centLib,"centOpen"))==NULL){
	  printf("dlsym failed for centOpen\n");
	  cerr=1;
	}
	if((*(void**)(&glob->centCloseFn)=dlsym(glob->centLib,"centClose"))==NULL){
	  printf("dlsym failed for centClose\n");
	  cerr=1;
	}
	if((*(void**)(&glob->centStartFramingFn)=dlsym(glob->centLib,"centStartFraming"))==NULL){
	  printf("dlsym failed for centStartFraming\n");
	  cerr=1;
	}
	if((*(void**)(&glob->centStopFramingFn)=dlsym(glob->centLib,"centStopFraming"))==NULL){
	  printf("dlsym failed for centStopFraming\n");
	  cerr=1;
	}
	if((*(void**)(&glob->centNewFrameFn)=dlsym(glob->centLib,"centNewFrame"))==NULL){
	  printf("dlsym failed for centNewFrame\n");
	  cerr=1;
	}
	if((*(void**)(&glob->centWaitPixelsFn)=dlsym(glob->centLib,"centWaitPixels"))==NULL){
	  printf("dlsym failed for centWaitPixels\n");
	  cerr=1;
	}
	if((*(void**)(&glob->centNewParamFn)=dlsym(glob->centLib,"centNewParam"))==NULL){
	  printf("centroid library has now centNewParam - continuing...\n");
	  //note - this is not an error just a warning.
	}
	if(cerr==1){//close the dll
	  if(glob->centLib!=NULL && dlclose(glob->centLib)!=0){
	    printf("Failed to close cent dynamic library - ignoring\n");
	  }
	  glob->centLib=NULL;
	}
      }
      if(glob->centLib!=NULL){//do the initialisation
	//work out number of centroids per camera...
	offset=0;
	for(j=0; j<info->ncam; j++){
	  glob->ncentsList[j]=0;
	  for(k=0; k<info->nsubList[j]; k++){
	    glob->ncentsList[j]+=info->subapFlagArr[k+offset];
	  }
	  offset+=k;
	  glob->ncentsList[j]*=2;
	}
	if(cerr==0){
	  doneParams=1;//the open function will do parameters...
	  cerr=(*glob->centOpenFn)(glob->centName,info->centroidersParamsCnt,info->centroidersParams,glob->buffer[glob->curBuf],glob->rtcErrorBuf,glob->shmPrefix,glob->arrays,&glob->centHandle,info->ncam,glob->ncentsList,glob->centframeno);//This is probably a blocking call - ie might wait until the camera reaches a certain temerature or whatever...
	}	
      }
      if(cerr){
	writeError(glob->rtcErrorBuf,"Error opening centroider",-1,glob->thisiter);
	*info->centroidersOpen=0;//write to the parambuf.
	*info->centroidersFraming=0;//write to the parambuf.
	if(glob->centLib!=NULL && dlclose(glob->centLib)!=0){
	  printf("Failed to close centroider dynamic library - ignoring\n");
	}
	glob->centLib=NULL;
      }
      if(glob->centLib==NULL){
	free(glob->centNameOpen);
	glob->centNameOpen=NULL;
      }
    }
    if(glob->centLib==NULL){
      glob->centOpenFn=NULL;
      glob->centCloseFn=NULL;
      glob->centStartFramingFn=NULL;
      glob->centStopFramingFn=NULL;
      glob->centNewFrameFn=NULL;
      glob->centWaitPixelsFn=NULL;
      glob->centNewParamFn=NULL;
    }
  }
  if(glob->centLib!=NULL && doneParams==0 && glob->centNewParamFn!=NULL){
    (*glob->centNewParamFn)(glob->centHandle,glob->buffer[glob->curBuf],glob->thisiter,glob->arrays);
  }
  return cerr;
}

/**
   Opens and closes the dynamic library and uses this to open the mirror
   Also updates the mirror with the new param buffer.
*/
int updateMirror(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  globalStruct *glob=threadInfo->globals;
  int cerr=0;
  int open=0;
  int doneParam=0;
  //first open the shared object library...
  if(glob->mirrorNameOpen==NULL){
    if(glob->mirrorName!=NULL && *info->mirrorOpen==1){
      glob->mirrorNameOpen=strdup(glob->mirrorName);
      open=1;
    }
  }else{//already one open
    if(*info->mirrorOpen==0 || glob->mirrorName==NULL){
      free(glob->mirrorNameOpen);
      glob->mirrorNameOpen=NULL;
      open=1;
    }else if(strcmp(glob->mirrorName,glob->mirrorNameOpen)){//different name...
      free(glob->mirrorNameOpen);
      glob->mirrorNameOpen=strdup(glob->mirrorName);
      open=1;
    }
  }
  if(open){
    //first close existing, if it is open...
    if(glob->mirrorLib!=NULL){
      //close existing library.
      (*glob->mirrorCloseFn)(&glob->mirrorHandle);
      if(dlclose(glob->mirrorLib)!=0){
	printf("Failed to close mirror library - ignoring\n");
      }
      glob->mirrorLib=NULL;
    }
    //and then open the new one.
    if(glob->mirrorNameOpen!=NULL){
      if((glob->mirrorLib=dlopen(glob->mirrorNameOpen,RTLD_LAZY))==NULL){
	printf("Failed to open mirror library %s: %s\n",glob->mirrorNameOpen,dlerror());
	cerr=1;
      }else{//now get the symbols...
	//now get the symbols... we want mirrorOpen,mirrorClose,mirrorNewParam and mirrorSend
	if((*(void**)(&glob->mirrorOpenFn)=dlsym(glob->mirrorLib,"mirrorOpen"))==NULL){
	  printf("dlsym failed for mirrorOpen\n");
	  cerr=1;
	}
	if((*(void**)(&glob->mirrorCloseFn)=dlsym(glob->mirrorLib,"mirrorClose"))==NULL){
	  printf("dlsym failed for mirrorClose\n");
	  cerr=1;
	}
	if((*(void**)(&glob->mirrorNewParamFn)=dlsym(glob->mirrorLib,"mirrorNewParam"))==NULL){
	  printf("dlsym failed for mirrorNewParam\n");
	  cerr=1;
	}
	if((*(void**)(&glob->mirrorSendFn)=dlsym(glob->mirrorLib,"mirrorSend"))==NULL){
	  printf("dlsym failed for mirrorSend\n");
	  cerr=1;
	}
	if(cerr==0){
	  doneParam=1;
	  cerr=(*glob->mirrorOpenFn)(glob->mirrorNameOpen,info->mirrorParamsCnt,info->mirrorParams,glob->buffer[glob->curBuf],glob->rtcErrorBuf,glob->shmPrefix,glob->arrays,&glob->mirrorHandle,info->nacts,glob->rtcActuatorBuf,glob->thisiter);
	  if(cerr){
	    writeError(glob->rtcErrorBuf,"Error opening mirror",-1,glob->thisiter);
	  }

	}
	if(cerr==1){//close the dll
	  if(dlclose(glob->mirrorLib)!=0){
	    printf("Failed to close mirror dynamic library - ignoring\n");
	  }
	  glob->mirrorLib=NULL;
	}
      }
    }
  }
  if(glob->mirrorLib!=NULL && doneParam==0 && glob->mirrorNewParamFn!=NULL){
    cerr=(*glob->mirrorNewParamFn)(glob->mirrorHandle,glob->buffer[glob->curBuf],glob->thisiter,glob->arrays);
    if(cerr){
      //close the library...
      printf("Error getting mirror parameters - closing mirror library\n");
      (*glob->mirrorCloseFn)(&glob->mirrorHandle);
      if(dlclose(glob->mirrorLib)!=0){
	printf("Failed to close mirror library - ignoring\n");
      }
      glob->mirrorLib=NULL;
    }
  }
  if(glob->mirrorLib==NULL){
    if(glob->mirrorNameOpen!=NULL){
      free(glob->mirrorNameOpen);
      glob->mirrorNameOpen=NULL;
    }
    *info->mirrorOpen=0;
    glob->mirrorNewParamFn=NULL;
    glob->mirrorCloseFn=NULL;
    glob->mirrorOpenFn=NULL;
    glob->mirrorSendFn=NULL;
  }
  glob->precomp->post.mirrorLib=glob->mirrorLib;
  return cerr;
}

/**
   Opens the dynamic library and uses this for figure sensing.
*/
int openFigure(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  globalStruct *glob=threadInfo->globals;
  int cerr=0,open=0;
  int doneParam=0;
  if(glob->figureNameOpen==NULL){
    if(glob->figureName!=NULL && *info->figureOpen==1){
      glob->figureNameOpen=strdup(glob->figureName);
      open=1;
    }
  }else{//already one open
    if(*info->figureOpen==0 || glob->figureName==NULL){
      free(glob->figureNameOpen);
      glob->figureNameOpen=NULL;
      open=1;
    }else if(strcmp(glob->figureName,glob->figureNameOpen)){//different name
      free(glob->figureNameOpen);
      glob->figureNameOpen=strdup(glob->figureName);
      open=1;
    }
  }
  if(open){
    //first close existign if it is open...
    if(glob->figureLib!=NULL){
      if(glob->figureHandle!=NULL)
	(*glob->figureCloseFn)(&glob->figureHandle);
      glob->figureHandle=NULL;
      if(dlclose(glob->figureLib)!=0){
	printf("Failed to close figure sensor dynamic library - ignoring: %s\n",dlerror());
      }
      glob->figureLib=NULL;
    }
    //and then open the new one
    if(glob->figureNameOpen!=NULL){
      if((glob->figureLib=dlopen(glob->figureNameOpen,RTLD_LAZY))==NULL){
	printf("Failed to open figure dynamic library %s: %s\n",glob->figureNameOpen,dlerror());
	cerr=1;
      }else{
	//now get the symbols... we want figureOpen,figureClose.
	if((*(void**)(&glob->figureOpenFn)=dlsym(glob->figureLib,"figureOpen"))==NULL){
	  printf("dlsym failed for figureOpen\n");
	  cerr=1;
	}
	if((*(void**)(&glob->figureCloseFn)=dlsym(glob->figureLib,"figureClose"))==NULL){
	  printf("dlsym failed for figureClose\n");
	  cerr=1;
	}
	if((*(void**)(&glob->figureNewParamFn)=dlsym(glob->figureLib,"figureNewParam"))==NULL){
	  printf("no figureNewParam function - ignoring\n");
	  //not an error - just a warning
	}
	if(cerr==1){//close the dll
	  if(glob->figureLib!=NULL && dlclose(glob->figureLib)!=0){
	    printf("Failed to close figure sensor library - ignoring\n");
	  }
	  glob->figureLib=NULL;
	}
      }
      if(glob->figureLib!=NULL){
	//do initialisation
	//this should start a thread that reads the actuator setpoints whenever they're available, and puts them into requiredActs, creating it if necessary.
	doneParam=1;
	cerr=(*glob->figureOpenFn)(glob->figureName,info->figureParamsCnt,info->figureParams,glob->buffer[glob->curBuf],glob->rtcErrorBuf,glob->shmPrefix,glob->arrays,&glob->figureHandle,info->nacts,glob->precomp->post.actsRequiredMutex,glob->precomp->post.actsRequiredCond,&(glob->precomp->post.actsRequired),glob->figureFrame);
      }
      if(cerr){
	writeError(glob->rtcErrorBuf,"Error opening figure sensor library",-1,glob->thisiter);
	*info->figureOpen=0;
	if(glob->figureLib!=NULL && dlclose(glob->figureLib)!=0){
	  printf("Failed to close figure sensor dynamic library - ignoring: %s\n",strerror(errno));
	}
	glob->figureLib=NULL;
      }
      if(glob->figureLib==NULL){
	free(glob->figureNameOpen);
	glob->figureNameOpen=NULL;
      }
    }
    if(glob->figureLib==NULL){
      glob->figureOpenFn=NULL;
      glob->figureCloseFn=NULL;
      glob->figureNewParamFn=NULL;
      *info->figureOpen=0;
    }
  }
  if(glob->figureLib!=NULL && doneParam==0 && glob->figureNewParamFn!=NULL){
    (*glob->figureNewParamFn)(glob->figureHandle,glob->buffer[glob->curBuf],glob->thisiter,glob->arrays);
  }
  return cerr;
}


/**
   This is called once by the first thread that does a buffer swap - it checks that arrays are of correct size for new buffer, and if not, reallocates them.
*/
int updateMemory(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  arrayStruct *glob=threadInfo->globals->arrays;//091109[threadInfo->mybuf];
  //arrayStruct *glob2=threadInfo->globals->arrays;//091109[1-threadInfo->mybuf];
  //globalStruct *globals=threadInfo->globals;
  int err=0;
  short *tmps;
  //now allocate an array large enough to hold all pixels (note - this may not be used, depending on the camera library...)
  if(glob->pxlbufsSize!=sizeof(short)*info->totPxls){
    glob->pxlbufsSize=sizeof(short)*info->totPxls;
    tmps=realloc(glob->pxlbufs,glob->pxlbufsSize);
    if(tmps==NULL){
      if(glob->pxlbufs!=NULL)
	free(glob->pxlbufs);
      printf("pxlbuf malloc error.\n");
      err=1;
      glob->pxlbufsSize=0;
    }
    glob->pxlbufs=tmps;
    memset(glob->pxlbufs,0,glob->pxlbufsSize);
  }
  
  
  if(glob->fluxSize<info->totCents/2){
    if(glob->flux!=NULL)
      free(glob->flux);
    glob->fluxSize=info->totCents/2;
    if((glob->flux=malloc(sizeof(float)*info->totCents/2))==NULL){
      printf("malloc of flux failed\n");
      glob->fluxSize=0;
      err=1;
    }
  }
  if(glob->centroidsSize<info->totCents){
    if(glob->centroids!=NULL)
      free(glob->centroids);
    if(glob->wpucentroids!=NULL)
      free(glob->wpucentroids);
    glob->centroidsSize=info->totCents;
    if((glob->centroids=malloc(sizeof(float)*info->totCents))==NULL){
      printf("malloc of centroids failed\n");
      err=1;
      glob->wpucentroids=NULL;
      glob->centroidsSize=0;
    }else{//malloced okay.
      if((glob->wpucentroids=malloc(sizeof(float)*info->totCents))==NULL){
	printf("malloc of wpucentroids failed\n");
	err=1;
	free(glob->centroids);
	glob->centroids=NULL;
	glob->centroidsSize=0;
      }
    }
  }
  if(glob->dmCommandSize<info->nacts){
    if(glob->dmCommand!=NULL)
      free(glob->dmCommand);
    if(glob->dmCommandSave!=NULL)
      free(glob->dmCommandSave);
    if(glob->dmCommandFigure!=NULL)
      free(glob->dmCommandFigure);

    glob->dmCommandSave=NULL;
    glob->dmCommandFigure=NULL;
    glob->dmCommandSize=info->nacts;
    if((glob->dmCommand=malloc(sizeof(float)*info->nacts))==NULL){
      printf("malloc of dmCommand failed\n");
      err=1;
      glob->dmCommandSize=0;
    }else{
      if((glob->dmCommandSave=malloc(sizeof(float)*info->nacts))==NULL){
	printf("malloc of dmCommandSave failed\n");
	err=1;
	if(glob->dmCommand!=NULL)
	  free(glob->dmCommand);
	glob->dmCommand=NULL;
	glob->dmCommandSize=0;
      }else{
	if((glob->dmCommandFigure=malloc(sizeof(float)*info->nacts))==NULL){
	  printf("malloc of dmCommandFigure failed\n");
	  err=1;
	  if(glob->dmCommand!=NULL)
	    free(glob->dmCommand);
	  if(glob->dmCommandSave!=NULL)
	    free(glob->dmCommandSave);
	  glob->dmCommand=NULL;
	  glob->dmCommandSize=0;
	}
      }
    }
  }

  if(glob->calpxlbufSize<info->totPxls){
    if(glob->calpxlbuf!=NULL)
      free(glob->calpxlbuf);
    glob->calpxlbufSize=info->totPxls;
    if((glob->calpxlbuf=malloc(sizeof(float)*info->totPxls))==NULL){
      printf("malloc of calpxlbuf failed\n");
      err=1;
      glob->calpxlbufSize=0;
    }else{
      memset(glob->calpxlbuf,0,sizeof(float)*info->totPxls);
    }
  }
  if(glob->corrbufSize<info->totPxls){
    if(glob->corrbuf!=NULL)
      free(glob->corrbuf);
    glob->corrbufSize=info->totPxls;
    if((glob->corrbuf=malloc(sizeof(float)*info->totPxls))==NULL){
      printf("malloc of corrbuf failed\n");
      err=1;
      glob->corrbufSize=0;
    }
  }
  if(info->windowMode==WINDOWMODE_ADAPTIVE || info->windowMode==WINDOWMODE_GLOBAL){
    if(glob->subapLocationSize<info->nsubaps*6){
      if(glob->subapLocation!=NULL)
	free(glob->subapLocation);
      glob->subapLocationSize=info->nsubaps*6;
      if((glob->subapLocation=malloc(glob->subapLocationSize*sizeof(int)))==NULL){
	printf("malloc of subapLocation failed\n");
	err=1;
	glob->subapLocationSize=0;
      }
      memcpy(glob->subapLocation,info->realSubapLocation,info->nsubaps*6*sizeof(int));
    }
    if(glob->adaptiveCentPosSize<info->totCents){
      if(glob->adaptiveCentPos!=NULL)
	free(glob->adaptiveCentPos);
      glob->adaptiveCentPosSize=info->totCents;
      if((glob->adaptiveCentPos=malloc(sizeof(int)*info->totCents))==NULL){
	printf("malloc of adaptiveCentPos failed\n");
	err=1;
	glob->adaptiveCentPosSize=0;
      }
    }
    if(glob->adaptiveWinPosSize<info->totCents){
      if(glob->adaptiveWinPos!=NULL)
	free(glob->adaptiveWinPos);
      glob->adaptiveWinPosSize=info->totCents;
      if((glob->adaptiveWinPos=malloc(sizeof(float)*info->totCents))==NULL){
	printf("malloc of adaptiveWinPos failed\n");
	err=1;
	glob->adaptiveWinPosSize=0;
      }
    }
    if(info->resetAdaptiveWindows){
      memset(glob->adaptiveCentPos,0,sizeof(int)*info->totCents);
      memset(glob->adaptiveWinPos,0,sizeof(float)*info->totCents);
      info->resetAdaptiveWindows=0;
    }
  }else{
    //don't bother doing anything... don't free it, cos might be used later.
  }
  if(err){
    threadInfo->info->pause=1;
    threadInfo->globals->buferr=1;
    if(threadInfo->globals->ppause!=NULL)
      *(threadInfo->globals->ppause)=1;
  }
  return err;
}

/**
   Called to initiate a buffer swap - do the per camera stuff.
*/
int doCamBufferSwap(threadStruct *threadInfo,int updateIndex){
  infoStruct *info;
  //Swap the buffers for this thread.
  int err=0;//,cerr=0;
  dprintf("doBufferSwap %d\n",updateIndex);
  //dprintf("Swapping info buffers to %d %d\n",threadInfo->infobuf->id,updateIndex);

  //If this is first thread for this camera, update the infoStruct buffer.
  info=threadInfo->info;
  //somewhere in updateBuffer, we are safe to release the startMutex to allow other threads to start their buffer swapping.
  err=updateBuffer(threadInfo,updateIndex);
  info=threadInfo->info;
  if(err!=0){
    threadInfo->info->pause=1;
    threadInfo->globals->buferr=1;
    if(threadInfo->globals->ppause!=NULL)
      *(threadInfo->globals->ppause)=1;
  }else{
    if(updateIndex){
      err=updateMemory(threadInfo);
    }
  }
  return err;
}
/**
   Called by first thread only.
*/
int openLibraries(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  int cerr=0;
  int err=0;
  //if(*info->camerasOpen==1 && threadInfo->globals->camHandle==NULL){//camera not yet open
  pthread_mutex_lock(&threadInfo->globals->libraryMutex);

  if(*info->camerasOpen==1){
    cerr=openCameras(threadInfo);
    err|=cerr;
  }
  if(*info->camerasFraming==1 && threadInfo->globals->camFraming==0){
    //camera not yet framing...
    if(threadInfo->globals->cameraLib!=NULL && *info->camerasOpen==1){
      cerr=(*threadInfo->globals->camStartFramingFn)(info->cameraParamsCnt,info->cameraParams,threadInfo->globals->camHandle);
    }else{
      cerr=1;
    }
    if(cerr==0)
      threadInfo->globals->camFraming=1;
    else{
      err|=cerr;
      threadInfo->globals->camFraming=0;
      writeError(threadInfo->globals->rtcErrorBuf,"Error framing camera",-1,threadInfo->globals->thisiter);
      *info->camerasFraming=0;
    }
  }
  if(*info->camerasFraming==0 && threadInfo->globals->camFraming==1){
    //stop the framing...
    if(threadInfo->globals->cameraLib!=NULL && *info->camerasOpen==1){
      (*threadInfo->globals->camStopFramingFn)(threadInfo->globals->camHandle);
    }
    threadInfo->globals->camFraming=0;
  }
  
  if(*info->camerasOpen==0){// && threadInfo->globals->camHandle!=NULL){
    //camera open, so close it
    cerr=openCameras(threadInfo);
  }
  if(*info->centroidersOpen==1){// && threadInfo->globals->centHandle==NULL){//cent camera not yet open
    cerr=openCentroiders(threadInfo);
    err|=cerr;
  }
  if(*info->centroidersFraming==1 && threadInfo->globals->centFraming==0){
    //centroid camera not yet framing...
    if(threadInfo->globals->centLib!=NULL && *info->centroidersOpen==1){
      cerr=(*threadInfo->globals->centStartFramingFn)(info->centroidersParamsCnt,info->centroidersParams,threadInfo->globals->centHandle);
    }else{
      cerr=1;
    }
    if(cerr==0)
      threadInfo->globals->centFraming=1;
    else{
      err|=cerr;
      threadInfo->globals->centFraming=0;
      writeError(threadInfo->globals->rtcErrorBuf,"Error framing centroider camera",-1,threadInfo->globals->thisiter);
      *info->centroidersFraming=0;
    }
  }
  if(*info->centroidersFraming==0 && threadInfo->globals->centFraming==1){
    //stop the framing...
    if(threadInfo->globals->centLib!=NULL && *info->centroidersOpen==1){
      (*threadInfo->globals->centStopFramingFn)(threadInfo->globals->centHandle);
    }
    threadInfo->globals->centFraming=0;
  }
  
  if(*info->centroidersOpen==0){// && threadInfo->globals->centHandle!=NULL){
    //camera open, so close it
    cerr=openCentroiders(threadInfo);


  }
  
  //if(*info->mirrorOpen==1){
  //do this every bufferswap, incase new params are read by the library...
  err|=updateMirror(threadInfo);

  //if(*info->figureOpen==1){// && threadInfo->globals->figureHandle==NULL){
  //connect to the figure sensor setpoint reading library.  This is used when this RTC is being used as a figure sensor.
  cerr=openFigure(threadInfo);
  err|=cerr;
  //Open the reconstructor library (if correct one not open) and give it the new parameters.
  err|=updateReconLibrary(threadInfo);
  pthread_mutex_unlock(&threadInfo->globals->libraryMutex);
  return err;
}
    
/**
   Each thread needs to update their priorities etc.
*/
int doThreadBufferSwap(threadStruct *threadInfo){
  infoStruct *info;
  //Swap the buffers for this thread.
  int err=0;//,cerr=0;
  info=threadInfo->info;

  setThreadAffinity(info->threadAffinityList,info->threadPriorityList,threadInfo->threadno+1,threadInfo->globals->ncpu);

  if(err){
    threadInfo->info->pause=1;
    threadInfo->globals->buferr=1;
    if(threadInfo->globals->ppause!=NULL)
      *(threadInfo->globals->ppause)=1;
  }
  return err;
}

/**
   This copies the global work array pointers to the pointers for this camera.
   This needs to be done after a call to updateMemory, once for each camera.
*/

int swapArrays(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  arrayStruct *glob=threadInfo->globals->arrays;//091109[threadInfo->mybuf];
  info->flux=glob->flux;
  info->centroids=glob->centroids;
  info->dmCommand=glob->dmCommand;
  info->calpxlbuf=glob->calpxlbuf;
  info->corrbuf=glob->corrbuf;
  if(glob->pxlbufs!=NULL)
    info->pxlbuf=glob->pxlbufs;
  if(info->windowMode==WINDOWMODE_ADAPTIVE || info->windowMode==WINDOWMODE_GLOBAL){
    info->subapLocation=glob->subapLocation;
    info->adaptiveCentPos=glob->adaptiveCentPos;
    info->adaptiveWinPos=glob->adaptiveWinPos;
  }else{
    info->subapLocation=info->realSubapLocation;
  }
  
  return 0;
}



/**
   Wait for the centroids to arrive.
   The centroids are put into globals->wpucentroids which is same as info->wpucentroids.
   Called to get (optional) slope input
*/
int waitCentroids(threadStruct *threadInfo){
  int rt=0;
  globalStruct *glob=threadInfo->globals;
  arrayStruct *arr=glob->arrays;
  if(glob->centLib!=NULL && glob->centWaitPixelsFn!=NULL){
    //This call should block until nsubs slope measurement have been received.
    //They will be in wpucentroids.
    //This function should return 1 on error, and 0 if slopes are valid.
    //Can return -1 if slopes are not valid, but this is not an error (e.g. you don't want to add anything this time).
    rt=(*glob->centWaitPixelsFn)(threadInfo->nsubs,threadInfo->info->cam,glob->centHandle);
    if(rt==0){
      //We then add these into our slope array.
      //centroids+=wpucentroids.
      agb_cblas_saxpy111(threadInfo->nsubapsDoing*2,&arr->wpucentroids[threadInfo->centindx],&arr->centroids[threadInfo->centindx]);
    }
    //The frame number will have been set.  So, if this is the first thread for this camera, can store the cam frame number.  If the first thread overall, can store glob->thisiter.  Also need to check that all frame numbers from each camera are the same - raise error if not.
    if(glob->cameraLib==NULL && rt==0){//Only update frame number if no camera library open.
      pthread_mutex_lock(&glob->camMutex);
      if(glob->camReadCnt==0){//first thread overall
	glob->setFrameno=1;
      }
      if(glob->setFrameno){//need to set frame no
	if(threadInfo->info->cam==0){//first cam can set frameno
	  glob->setFrameno=0;
	  if(rt==0){
	    if(glob->camiter==glob->centframeno[0]){//it would seem that the camera isn't updating the iteration count.
	      glob->thisiter++;
	    }else{//camera is updating the iteration count (frameno)
	      glob->thisiter=glob->centframeno[0];
	      glob->camiter=glob->thisiter;
	    }
	  }else{//we need to update here - failed or no camera library.
	    glob->thisiter++;
	  }
	}else{//not first cam.
	  //we're not the first cam, but the frameno hasn't yet been set, so we have nothing to check against.  Is there anything I can do here?  Don't think so really, just have to hope!!!
	}
      }else{//frameno has already been set, so we can check against it.
	if(rt==0 && glob->camiter!=glob->centframeno[threadInfo->info->cam]){
	  writeErrorVA(glob->rtcErrorBuf,CAMSYNCERROR,glob->thisiter,"Error - camera frames not in sync");
	}
      }
      glob->camReadCnt++;
      pthread_mutex_unlock(&glob->camMutex);
    }
  }else{//no centroider open
    if(glob->cameraLib==NULL){
      //need to update frame number, since no camera either...
      pthread_mutex_lock(&glob->camMutex);
      if(glob->camReadCnt==0){//first thread overall
	glob->thisiter++;
      }
      glob->camReadCnt++;
      pthread_mutex_unlock(&glob->camMutex);
    }
  }

  return (rt==1);
}


/**
   Called instead of waitNextSubaps when WPU is being used to compute slopes.
*/
/*
  int getNextCentroids(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  int endFrame=0;
  int cnt=0,i;
  //Threads run in order, and are returned a subap as it becomes available...
  //first block in the thread queue until it is my turn.
  dprintf("locking subapMutex\n");
  if(pthread_mutex_lock(&info->subapMutex))
  printf("pthread_mutex_lock error in getNextCentroid: %s\n",strerror(errno));
  dprintf("locked subapMutex %d %p %d\n",info->subindx,info->subflag,info->id);
  //if(info->frameFinished[threadInfo->mybuf]==1){091109
  if(info->frameFinished==1){
  dprintf("frameFinished in getNextCentroid: %d\n",info->frameFinished);//091109[threadInfo->mybuf]);
  pthread_mutex_unlock(&info->subapMutex);
  return -1;
  }
  //work out which is the next subap to do... and how many...
  while(cnt==0 && info->subindx<info->nsubx*info->nsuby){
  for(i=0; i<info->nsubapsTogether && i<info->nsubx*info->nsuby-info->subindx; i++){
  cnt+=info->subflag[info->subindx+i];
  }
  if(cnt==0)
  info->subindx+=info->nsubapsTogether;
  }
  if(info->subindx>=info->nsubx*info->nsuby){
  info->subindx=0;
  info->centindx=0;
  endFrame=-1;
  info->frameFinished=1;//091109[threadInfo->mybuf]=1;//let others know we've read all subaps.
  }
  if(endFrame==0){//some subaps to read...
  //store the indx for this thread...
  threadInfo->centindx=info->centindx+info->centCumIndx;
  threadInfo->cursubindx=info->subindx+info->subCumIndx;
  threadInfo->nsubapsDoing=cnt;
  info->centindx+=2*cnt;//info->nsubapsTogether;//increase by number of used subaps by this thread.
  threadInfo->nsubs=info->centindx;//this is the number of subaps that we need to have arrived for this camera before continuing.
  info->subindx+=info->nsubapsTogether;
  if(info->subindx>=info->nsubx*info->nsuby){
  info->subindx=0;
  info->centindx=0;
  //this thread has valid data, so still want to process it, butother threads shouldn;t as frame is finished.
  info->frameFinished=1;//091109[threadInfo->mybuf]=1;//let other threadsknow that we've read all the subaps...
  }
  endFrame=(waitCentroids(threadInfo->nsubs,threadInfo)!=0);
  //The frame number will have been set.  So, if this is the first thread for this camera, can store the cam frame number.  If the first thread overall, can store glob->thisiter.  Also need to check that all frame numbers from each camera are the same - raise error if not.
  if(endFrame==0){//waited successfully...
  //get the frame number...
  pthread_mutex_lock(&threadInfo->globals->camMutex);
  if(threadInfo->globals->camReadCnt==0){//first thread overall
  if(threadInfo->globals->camiter==threadInfo->globals->camframeno[info->cam]){//it would seem that the camera isn't updating the iteration count...
  threadInfo->globals->thisiter++;
  }else{//camera is updating the iteration count (frame number)
  threadInfo->globals->thisiter=threadInfo->globals->camframeno[info->cam];
  threadInfo->globals->camiter=threadInfo->globals->thisiter;
  }
  }else{//not first thread...
  if(threadInfo->globals->camiter!=threadInfo->globals->camframeno[info->cam]){
  writeError(threadInfo->globals->rtcErrorBuf,"Error - camera frames not in sync",CAMSYNCERROR,threadInfo->globals->thisiter);
  }
  }
  threadInfo->globals->camReadCnt++;
  pthread_mutex_unlock(&threadInfo->globals->camMutex);
  }else{
  writeError(threadInfo->globals->rtcErrorBuf,"Error - getting camera centroids",CAMGETERROR,threadInfo->globals->thisiter);

  }
  }
  //then release mutex so that next thread can wait for pixels, and reset this sem, so that this thread will wait next time...
  pthread_mutex_unlock(&info->subapMutex);//unlock the mutex, allowing the next thread to continue
  dprintf("freed muted\n");
  return endFrame;
  }

*/




/**
   Wake up the pre/post processing thread.
*/
int wakePrepareActuatorsThread(threadStruct *threadInfo){
  PreComputeData *p=threadInfo->globals->precomp;
  infoStruct *info=threadInfo->info;
  dprintf("wakePrepareActuatorsThread %p %p\n",p,info);
  p->nacts=info->nacts;
  p->totCents=info->totCents;
  //these are the ones that are written to by the threads and so must be synced
  p->dmCommand=info->dmCommand;
  if(pthread_mutex_lock(&p->prepMutex))
    printf("pthread_mutex_lock error in wakePrepareActuatorsThread2: %s\n",strerror(errno));
  if(p->readyToStart==-1)
    pthread_cond_signal(&p->prepCond);//wake the thread
  else
    p->readyToStart=1;
  pthread_mutex_unlock(&p->prepMutex);
  return 0;
}

/**
   Wakes up the pre/post processing thread.
*/
int endFrame(threadStruct *threadInfo){
  //At the end of the frame: (this only gets called by 1 thread)
  //update the circular buffers here, or as they become available?  Probably here for simplicity.
  PostComputeData *p=&threadInfo->globals->precomp->post;
  infoStruct *info=threadInfo->info;
  globalStruct *globals=threadInfo->globals;
  STARTTIMING;
  if(pthread_mutex_lock(&globals->precomp->pxlcentMutex))
    printf("pthread_mutex_lock error in endFrame: %s\n",strerror(errno));
  globals->precomp->pxlcentReady=0;
  pthread_mutex_unlock(&globals->precomp->pxlcentMutex);

  if(pthread_mutex_lock(&globals->precomp->postMutex))
    printf("pthread_mutex_lock error in endFrame2: %s\n",strerror(errno));
  //set up the correct pointers...
  p->closeLoop=info->closeLoop;
  p->userActs=info->userActs;
  p->userActsMask=info->userActsMask;
  p->addUserActs=info->addUserActs;
  p->userActSeq=info->userActSeq;
  p->userActSeqLen=info->userActSeqLen;
  p->recordCents=info->recordCents;
  p->totPxls=info->totPxls;
  p->nAvImg=info->nAvImg;
  p->averageImg=info->averageImg;
  p->nAvCent=info->nAvCent;
  p->averageCent=info->averageCent;
  p->nacts=info->nacts;
  p->totCents=info->totCents;
  p->maxClipped=info->maxClipped;
  //these are written to by the post compute thread...
  p->calpxlbuf=info->calpxlbuf;
  p->corrbuf=info->corrbuf;
  p->centroids=info->centroids;
  p->flux=info->flux;
  p->dmCommand=info->dmCommand;
  p->dmCommandSave=globals->arrays->dmCommandSave;
  p->figureGain=info->figureGain;
  p->figureGainArr=info->figureGainArr;
  p->dmCommandFigure=globals->arrays->dmCommandFigure;
  p->mirrorHandle=globals->mirrorHandle;
  p->mirrorSendFn=globals->mirrorSendFn;
  p->reconOpenLoopFn=globals->reconOpenLoopFn;
  p->calibrateOpenLoopFn=globals->calibrateOpenLoopFn;
  p->reconFrameFinishedFn=globals->reconFrameFinishedFn;
  p->calibrateFrameFinishedFn=globals->calibrateFrameFinishedFn;
  p->thisiter=globals->thisiter;
  p->windowMode=info->windowMode;
  p->subapLocation=info->subapLocation;
  p->pxlCentInputError=info->pxlCentInputError;
  if(threadInfo->info->windowMode==WINDOWMODE_GLOBAL)
    calcGlobalAdaptiveWindow(threadInfo->info);


  //gettimeofday(&thistime,NULL);
  //timestamp=thistime.tv_sec+thistime.tv_usec*1e-6;
  if(FORCEWRITEALL(globals->rtcPxlBuf)){
    FORCEWRITE(globals->rtcPxlBuf)=FORCEWRITEALL(globals->rtcPxlBuf);
    FORCEWRITE(globals->rtcStatusBuf)=FORCEWRITEALL(globals->rtcPxlBuf);
    FORCEWRITE(globals->rtcTimeBuf)=FORCEWRITEALL(globals->rtcPxlBuf);
    FORCEWRITE(globals->rtcCalPxlBuf)=FORCEWRITEALL(globals->rtcPxlBuf);
    FORCEWRITE(globals->rtcCorrBuf)=FORCEWRITEALL(globals->rtcPxlBuf);
    FORCEWRITE(globals->rtcCentBuf)=FORCEWRITEALL(globals->rtcPxlBuf);
    FORCEWRITE(globals->rtcFluxBuf)=FORCEWRITEALL(globals->rtcPxlBuf);
    FORCEWRITE(globals->rtcSubLocBuf)=FORCEWRITEALL(globals->rtcPxlBuf);
    FORCEWRITE(globals->rtcMirrorBuf)=FORCEWRITEALL(globals->rtcPxlBuf);
    FORCEWRITE(globals->rtcActuatorBuf)=FORCEWRITEALL(globals->rtcPxlBuf);
    FORCEWRITEALL(globals->rtcPxlBuf)=0;
  }

  //No need to get the libraryMutex here, since this is never called at the same time as the updateRecon method...
  if(globals->calibrateFrameFinishedSyncFn!=NULL)
    (*globals->calibrateFrameFinishedSyncFn)(globals->calibrateHandle,info->pxlCentInputError);
  if(globals->reconLib!=NULL && globals->reconHandle!=NULL)
    (*globals->reconFrameFinishedSyncFn)(globals->reconHandle,threadInfo->info->centroids,globals->arrays->dmCommand,info->pxlCentInputError);
  pthread_cond_signal(&globals->precomp->postCond);//wake the thread
  pthread_mutex_unlock(&globals->precomp->postMutex);
  ENDTIMING(endFrame);
  return 0;
}


/**
   Run by the pre/post processing thread - does global initialisations/finalisations for each frame, e.g. sending the actuator values etc
*/
int prepareActuators(globalStruct *glob){
  //This is run as a separate thread, which blocks until freed, waiting to be able to do its thing.
  //CBLAS_ORDER order=CblasRowMajor;
  PreComputeData *p;
  PostComputeData *pp;
  //CBLAS_TRANSPOSE trans=CblasNoTrans;
  //struct timeval t1;
  double timestamp;
  //float alpha=1.,beta=1.;
  //int inc=1;
  int sendAvCalPxl=0;
  int sendAvCent=0;
  float *tmpf;
  struct sched_param schedParam;
  //if(pthread_mutex_lock(&glob->precomp->prepMutex))
  //  printf("pthread_mutex_lock error in prepareActuators: %s\n",strerror(errno));
  if(pthread_mutex_lock(&glob->precomp->postMutex))
    printf("pthread_mutex_lock error in prepareActuators2: %s\n",strerror(errno));
  memset(&schedParam,0,sizeof(struct sched_param));
  schedParam.sched_priority=2;
  if(sched_setscheduler(0,SCHED_RR,&schedParam)!=0){
    printf("Error in prepareActuators sched_setscheduler %s\n",strerror(errno));
  }
  if(sched_setparam(0,&schedParam)!=0){
    printf("Error in prepareActuators sched_setparam %s\n",strerror(errno));
  }

  printf("Thread starting prepareActuators loop...\n");
  while(1){
    //wait until we're signalled to start (this releases the mutex while waiting, and regets it when signalled...
    if(pthread_mutex_lock(&glob->precomp->prepMutex))
      printf("pthread_mutex_lock error in prepareActuators: %s\n",strerror(errno));
    if(glob->precomp->readyToStart==0){
      glob->precomp->readyToStart=-1;//inform main threads that we're waiting
      if(pthread_cond_wait(&glob->precomp->prepCond,&glob->precomp->prepMutex))
	printf("pthread_cond_wait error in prepareActuators: %s\n",strerror(errno));
    }
    //now we're ready to start the preprocessing...
    glob->precomp->readyToStart=0;//reset for next time...
    pthread_mutex_unlock(&glob->precomp->prepMutex);

    if((glob->threadAffinityList==NULL && glob->prepThreadAffinity!=-1) || (glob->threadAffinityList!=NULL && glob->prepThreadAffinity!=glob->threadAffinityList[0]) || (glob->threadPriorityList==NULL && glob->prepThreadPriority!=2) || (glob->threadPriorityList!=NULL && glob->prepThreadPriority!=glob->threadPriorityList[0])){
      if(glob->threadAffinityList==NULL)
	glob->prepThreadAffinity=-1;
      else
	glob->prepThreadAffinity=glob->threadAffinityList[0];
      if(glob->threadPriorityList==NULL)
	glob->prepThreadPriority=2;
      else
	glob->prepThreadPriority=glob->threadPriorityList[0];
      setThreadAffinity(glob->threadAffinityList,glob->threadPriorityList,0,glob->ncpu);
    }
    //First we do initial preparation...
    p=glob->precomp;
    dprintf("Precomputing dmCommand\n");

    pthread_mutex_lock(&glob->libraryMutex);
    if(glob->reconLib!=NULL && glob->reconHandle!=NULL && glob->reconNewFrameFn!=NULL)//First do the recon library.
      (*glob->reconNewFrameFn)(glob->reconHandle,glob->arrays->dmCommand);
    pthread_mutex_unlock(&glob->libraryMutex);

    ///////////////////////////NOW END OF FRAME////////////////////////////////

    //and now we wait for the end of frame stuff...
    if(pthread_cond_wait(&p->postCond,&p->postMutex))
      printf("pthread_cond_wait error in prepareActuators2: %s\n",strerror(errno));
    dprintf("postcomputing, sending actuators etc\n");
    pp=&p->post;

    //Now send stuff to the circular buffers.
    //gettimeofday(&t1,NULL);
    //timestamp=t1.tv_sec+t1.tv_usec*1e-6;
    //pp->timestamp=timestamp;
    timestamp=pp->timestamp;
    sendActuators(pp,glob);//moved here on 100623.  Should reduce latency slightly.  May also improve jitter since other threads aren't progressing yet past the camera read... however, on multi-core, will slightly reduce the max frame rate, since the main threads now have to wait for the mirror data to be sent.

    if(pp->pxlCentInputError==0){//will be set if an error has occurred eg during reading of image...
      circAdd(glob->rtcCalPxlBuf,pp->calpxlbuf,timestamp,pp->thisiter);
      if(*pp->averageImg>0){
	if(*pp->averageImg==1)
	  sendAvCalPxl=1;
	(*pp->averageImg)--;
	if(pp->avCalPxlBufSize<pp->totPxls){
	  tmpf=realloc(pp->avCalPxlBuf,pp->totPxls*sizeof(float));
	  if(tmpf==NULL){
	    printf("Unable to realloc avCalPxlBuf\n");
	    if(pp->avCalPxlBuf!=NULL)
	      free(pp->avCalPxlBuf);
	    pp->avCalPxlBuf=NULL;
	    pp->avCalPxlBufSize=0;
	    pp->averageImg=0;
	    sendAvCalPxl=0;
	  }else{//realloced okay
	    pp->avCalPxlBuf=tmpf;
	    pp->avCalPxlBufSize=pp->totPxls;
	    memset(pp->avCalPxlBuf,0,pp->totPxls*sizeof(float));
	  }
 	}
	if(pp->avCalPxlBuf!=NULL){
#ifdef USEAGBBLAS
	  agb_cblas_saxpy111(pp->totPxls,pp->calpxlbuf,pp->avCalPxlBuf);
#else
	  cblas_saxpy(pp->totPxls,1.,pp->calpxlbuf,1,pp->avCalPxlBuf,1);
#endif
	}
      }
      if(*pp->averageCent>0){
	if(*pp->averageCent==1)
	  sendAvCent=1;
	(*pp->averageCent)--;
	if(pp->avCentBufSize<pp->totCents){
	  tmpf=realloc(pp->avCentBuf,pp->totCents*sizeof(float));
	  if(tmpf==NULL){
	    printf("Unable to realloc avCentBuf\n");
	    if(pp->avCentBuf!=NULL)
	      free(pp->avCentBuf);
	    pp->avCentBuf=NULL;
	    pp->avCentBufSize=0;
	    pp->averageCent=0;
	    sendAvCent=0;
	  }else{//realloced okay
	    pp->avCentBuf=tmpf;
	    pp->avCentBufSize=pp->totCents;
	    memset(pp->avCentBuf,0,pp->totCents*sizeof(float));
	  }
 	}
	if(pp->avCentBuf!=NULL){
	  printf("Adding centroids %g %g\n",pp->centroids[0],pp->avCentBuf[0]);
#ifdef USEAGBBLAS
	  agb_cblas_saxpy111(pp->totCents,pp->centroids,pp->avCentBuf);
#else
	  cblas_saxpy(pp->totCents,1.,pp->centroids,1,pp->avCentBuf,1);
#endif
	}
      }
      circAdd(glob->rtcPxlBuf,glob->arrays->pxlbufs,timestamp,pp->thisiter);
      //check that we are doing correlation??? Or just add it anyway.
      circAdd(glob->rtcCorrBuf,pp->corrbuf,timestamp,pp->thisiter);
      //calpxlbuf can now be written to by other threads
      memset(pp->calpxlbuf,0,pp->totPxls*sizeof(float));//clear the buffer for next time.
      circAdd(glob->rtcCentBuf,pp->centroids,timestamp,pp->thisiter);
      circAdd(glob->rtcFluxBuf,pp->flux,timestamp,pp->thisiter);
      //centroids can now be written to by other threads
      circAdd(glob->rtcSubLocBuf,pp->subapLocation,timestamp,pp->thisiter);

    }
    setPxlBufAndCentroidReady(glob);
    //sendActuators(pp,glob);//send moved 100623 slightly earlier.
    if(sendAvCalPxl){
      sendAvCalPxl=0;
      //average the image...
      printf("Averaging image\n");
#ifdef USEAGBBLAS
      agb_cblas_sscal1(pp->totPxls,1./pp->nAvImg,pp->avCalPxlBuf);
#else
      cblas_sscal(pp->totPxls,1./pp->nAvImg,pp->avCalPxlBuf,1);
#endif
      circReshape(glob->rtcGenericBuf,1,&pp->totPxls,'f');
      circAdd(glob->rtcGenericBuf,pp->avCalPxlBuf,timestamp,pp->thisiter);
      //reset ready for next time
      memset(pp->avCalPxlBuf,0,sizeof(float)*pp->totPxls);
      printf("averaging done\n");
    }
    if(sendAvCent){
      sendAvCent=0;
      //average the image...
      printf("Averaging cents %g %d %g\n",pp->avCentBuf[0],pp->nAvCent,1./pp->nAvCent);
#ifdef USEAGBBLAS
      agb_cblas_sscal1(pp->totCents,(float)(1./pp->nAvCent),pp->avCentBuf);
#else
      cblas_sscal(pp->totCents,(float)(1./pp->nAvCent),pp->avCentBuf,1);
#endif
      circReshape(glob->rtcGenericBuf,1,&pp->totCents,'f');
      circAdd(glob->rtcGenericBuf,pp->avCentBuf,timestamp,pp->thisiter);
      //reset ready for next time
      printf("averaging done %g\n",pp->avCentBuf[0]);
      memset(pp->avCentBuf,0,sizeof(float)*pp->totCents);
    }
      


    glob->nclipped=pp->nclipped;
    if(pp->nclipped>pp->maxClipped)
      writeError(glob->rtcErrorBuf,"Maximum clipping exceeded",CLIPERROR,pp->thisiter);

    writeStatusBuf(glob,0,pp->closeLoop);
    circAdd(glob->rtcStatusBuf,glob->statusBuf,timestamp,pp->thisiter);
    

  }
  //and now allow other things access to the buffer.
  //pthread_mutex_unlock(&p->prepMutex);
  pthread_mutex_unlock(&p->postMutex);
  return 0;
}

int getSwitchRequested(globalStruct *globals){
  //retrieves value of switchRequested in the buffer, and if set, clears it.
  int j=0;
  int sr=0;
  char *buf=globals->buffer[globals->curBuf]->buf;
  paramBuf *pbuf=globals->buffer[globals->curBuf];
  int index=globals->bufferHeaderIndex[SWITCHREQUESTED];
  int found=0;
  if(index>=0 && index<NBUFFERVARIABLES && strncmp(&buf[index*16],"switchRequested",16)==0){//found it...
    found=1;
    if(globals->bufferNbytes[index]==sizeof(int) && globals->bufferDtype[index]=='i'){
      sr=*((int*)globals->bufferValues[index]);
      if(sr==1)
	globals->switchRequestedPtr=globals->bufferValues[index];
    }else{
      printf("switchRequested error\n");
    }
  }
  if(found==0){
    while(j<BUFNHDR(pbuf) && buf[j*16]!='\0'){
      if(strncmp(&buf[j*16],"switchRequested",16)==0){
	if(pbuf->dtype[j]=='i' && pbuf->nbytes[j]==sizeof(int)){
	  sr=*((int*)BUFGETVALUE(pbuf,j));
	  if(sr==1)
	    globals->switchRequestedPtr=((int*)BUFGETVALUE(pbuf,j));
	}else{
	  printf("switchRequested error\n");
	}
	break;
      }
      j++;
    }
  }
  if(sr==0)
    globals->switchRequestedPtr=NULL;
  return sr;
}
#ifdef USECOND
int freezeParamBuf(paramBuf *b1,paramBuf *b2){
  //freezes current buf b1, unfreezes the other one, b2.
  b1->hdr[2]|=0x1;//set the freeze bit.
  b2->hdr[2]&=~0x1;//unset the freeze bit.
  pthread_cond_broadcast(b2->cond);//wake up anything waiting for b2.
  return 0;
}
#else
int freezeParamBuf(int semid1,int semid2){
  //freezes current buf, and unfreezes the other one.
  union semun argument;
  argument.val=1;
  if(semctl(semid1,0,SETVAL,argument)==-1){
    printf("semctl failed in freezeParamBuf: %s\n",strerror(errno));
  }
  argument.val=0;
  if(semctl(semid2,0,SETVAL,argument)==-1){
    printf("semctl failed in unfreezeParamBuf: %s\n",strerror(errno));
  }
  return 0;
}
#endif
int updateCircBufs(threadStruct *threadInfo){
  //update the sizes of circular buffers...
  //printf("Updating circ bufs\n");
  infoStruct *info=threadInfo->info;
  int dim,err=0;
  if(threadInfo->globals->rtcPxlBuf!=NULL && threadInfo->globals->rtcPxlBuf->datasize!=info->totPxls*sizeof(short)){
    dim=info->totPxls;
    if(circReshape(threadInfo->globals->rtcPxlBuf,1,&dim,'h')!=0){
      printf("Error reshaping rtcPxlBuf\n");
      err=1;
    }
  }
  if(threadInfo->globals->rtcCalPxlBuf!=NULL && threadInfo->globals->rtcCalPxlBuf->datasize!=info->totPxls*sizeof(float)){
    dim=info->totPxls;
    if(circReshape(threadInfo->globals->rtcCalPxlBuf,1,&dim,'f')!=0){
      printf("Error reshaping rtcCalPxlBuf\n");
      err=1;
    }
  }
  if(threadInfo->globals->rtcCorrBuf!=NULL && threadInfo->globals->rtcCorrBuf->datasize!=info->totPxls*sizeof(float)){
    dim=info->totPxls;
    if(circReshape(threadInfo->globals->rtcCorrBuf,1,&dim,'f')!=0){
      printf("Error reshaping rtcCorrBuf\n");
      err=1;
    }
  }

  if(threadInfo->globals->rtcCentBuf!=NULL && threadInfo->globals->rtcCentBuf->datasize!=info->totCents*sizeof(float)){
    dim=info->totCents;
    if(circReshape(threadInfo->globals->rtcCentBuf,1,&dim,'f')!=0){
      printf("Error reshaping rtcCentBuf\n");
      err=1;
    }
  }
  if(threadInfo->globals->rtcMirrorBuf!=NULL && threadInfo->globals->rtcMirrorBuf->datasize!=info->nacts*sizeof(float)){
    dim=info->nacts;
    if(circReshape(threadInfo->globals->rtcMirrorBuf,1,&dim,'f')!=0){
      printf("Error reshaping rtcMirrorBuf\n");
      err=1;
    }
  }
  if(threadInfo->globals->rtcStatusBuf!=NULL && threadInfo->globals->rtcStatusBuf->datasize!=STATUSBUFSIZE){
    dim=STATUSBUFSIZE;
    if(circReshape(threadInfo->globals->rtcStatusBuf,1,&dim,'b')!=0){
      printf("Error reshaping rtcStatusBuf\n");
      err=1;
    }
  }
  if(threadInfo->globals->rtcTimeBuf!=NULL && threadInfo->globals->rtcTimeBuf->datasize!=sizeof(double)){
    dim=1;
    if(circReshape(threadInfo->globals->rtcTimeBuf,1,&dim,'d')!=0){
      printf("Error reshaping rtcTimeBuf\n");
      err=1;
    }
  }
  if(threadInfo->globals->rtcErrorBuf!=NULL && threadInfo->globals->rtcErrorBuf->datasize!=ERRORBUFSIZE){
    dim=ERRORBUFSIZE;
    if(circReshape(threadInfo->globals->rtcErrorBuf,1,&dim,'b')!=0){
      printf("Error reshaping rtcErrorBuf\n");
      err=1;
    }
  }
  if(threadInfo->globals->rtcSubLocBuf!=NULL && threadInfo->globals->rtcSubLocBuf->datasize!=info->nsubaps*6*sizeof(int)){
    dim=info->nsubaps*6;
    if(circReshape(threadInfo->globals->rtcSubLocBuf,1,&dim,'i')!=0){
      printf("Error reshaping rtcSubLocBuf\n");
      err=1;
    }
  }
  if(threadInfo->globals->rtcFluxBuf!=NULL && threadInfo->globals->rtcFluxBuf->datasize!=info->totCents/2*sizeof(float)){
    dim=info->totCents/2;
    if(circReshape(threadInfo->globals->rtcFluxBuf,1,&dim,'f')!=0){
      printf("Error reshaping rtcFluxBuf\n");
      err=1;
    }
  }
  return err;
}

int createCircBufs(threadStruct *threadInfo){
  globalStruct *glob=threadInfo->globals;
  int dim;
  char *tmp;
  if(glob->rtcPxlBuf==NULL){
    if(asprintf(&tmp,"/%srtcPxlBuf",glob->shmPrefix)==-1)
      exit(1);
    glob->rtcPxlBuf=openCircBuf(tmp,1,&threadInfo->info->totPxls,'h',100);
    free(tmp);
  }
  if(glob->rtcCalPxlBuf==NULL){
    if(asprintf(&tmp,"/%srtcCalPxlBuf",glob->shmPrefix)==-1)
      exit(1);
    glob->rtcCalPxlBuf=openCircBuf(tmp,1,&threadInfo->info->totPxls,'f',100);
    free(tmp);
  }
  if(glob->rtcCorrBuf==NULL){
    if(asprintf(&tmp,"/%srtcCorrBuf",glob->shmPrefix)==-1)
      exit(1);
    glob->rtcCorrBuf=openCircBuf(tmp,1,&threadInfo->info->totPxls,'f',100);
    free(tmp);
  }
  if(glob->rtcCentBuf==NULL){
    if(asprintf(&tmp,"/%srtcCentBuf",glob->shmPrefix)==-1)
      exit(1);
    glob->rtcCentBuf=openCircBuf(tmp,1,&threadInfo->info->totCents,'f',100);
    free(tmp);
  }
  if(glob->rtcMirrorBuf==NULL){
    if(asprintf(&tmp,"/%srtcMirrorBuf",glob->shmPrefix)==-1)
      exit(1);
    glob->rtcMirrorBuf=openCircBuf(tmp,1,&threadInfo->info->nacts,'f',1000);
    free(tmp);
  }
  if(glob->rtcActuatorBuf==NULL){
    if(asprintf(&tmp,"/%srtcActuatorBuf",glob->shmPrefix)==-1)
      exit(1);
    glob->rtcActuatorBuf=openCircBuf(tmp,1,&threadInfo->info->nacts,'f',1000);
    free(tmp);
  }
  dim=STATUSBUFSIZE;
  if(glob->rtcStatusBuf==NULL){
    if(asprintf(&tmp,"/%srtcStatusBuf",glob->shmPrefix)==-1)
      exit(1);
    glob->rtcStatusBuf=openCircBuf(tmp,1,&dim,'b',1000);
    free(tmp);
  }
  dim=1;
  if(glob->rtcTimeBuf==NULL){
    if(asprintf(&tmp,"/%srtcTimeBuf",glob->shmPrefix)==-1)
      exit(1);
    glob->rtcTimeBuf=openCircBuf(tmp,1,&dim,'d',10000);
    free(tmp);
  }
  dim=ERRORBUFSIZE;
  if(glob->rtcErrorBuf==NULL){
    if(asprintf(&tmp,"/%srtcErrorBuf",glob->shmPrefix)==-1)
      exit(1);
    glob->rtcErrorBuf=openCircBuf(tmp,1,&dim,'b',100);
    free(tmp);
  }
  dim=threadInfo->info->nsubaps*6;
  if(glob->rtcSubLocBuf==NULL){
    if(asprintf(&tmp,"/%srtcSubLocBuf",glob->shmPrefix)==-1)
      exit(1);
    glob->rtcSubLocBuf=openCircBuf(tmp,1,&dim,'i',100);
    free(tmp);
  }
  if(glob->rtcGenericBuf==NULL){//can store eg poke matrix of a calibrated image...
    dim=threadInfo->info->totCents*4*threadInfo->info->nacts;//4 values per poke
    if(threadInfo->info->totPxls>dim)
      dim=threadInfo->info->totPxls;
    if(asprintf(&tmp,"/%srtcGenericBuf",glob->shmPrefix)==-1)
      exit(1);
    glob->rtcGenericBuf=openCircBuf(tmp,1,&dim,'f',4);
    free(tmp);
  }
  if(glob->rtcFluxBuf==NULL){
    if(asprintf(&tmp,"/%srtcFluxBuf",glob->shmPrefix)==-1)
      exit(1);
    dim=threadInfo->info->totCents/2;
    glob->rtcFluxBuf=openCircBuf(tmp,1,&dim,'f',100);
    free(tmp);
  }
  return 0;
}

/**
   Called each iteration, to do housekeeping (inform camera library, etc).
*/
int startNewFrame(threadStruct *threadInfo){
  //The first thread should tell the cameras that a new frame is starting.
  if(threadInfo->globals->cameraLib!=NULL)//tell the camera library that a new frame has started
    (*threadInfo->globals->camNewFrameFn)(threadInfo->globals->camHandle);
  if(threadInfo->globals->centLib!=NULL)//tell the centroid camera library that a new frame has started
    (*threadInfo->globals->centNewFrameFn)(threadInfo->globals->centHandle);
  if(threadInfo->globals->calibrateNewFrameFn!=NULL)//tell calibrate library that new frame has started
    (*threadInfo->globals->calibrateNewFrameFn)(threadInfo->globals->calibrateHandle);
  if(threadInfo->info->pause==0)
    wakePrepareActuatorsThread(threadInfo);
  else
    usleep(10000);
  if(threadInfo->info->delay!=0)
    usleep(threadInfo->info->delay);
  return 0;
}

/**
   This is the main processing function, which each thread calls.
   Threads then proceed to obtain pixels for a sub-aperture, process
   this, and build their part of the reconstruction.  Buffer
   switching, and circular buffer updating is also carried out here.
   @return 0
*/

int processFrame(threadStruct *threadInfo){
  //each thread runs this...
  int i,err=0;

  int niters;
  infoStruct *info=threadInfo->info;
  struct timeval thistime;
  double dtime;
  //int increment;
  int nsubapDone;
  struct sched_param schedParam;
  double timestamp=0;
  int cursubindx,centindx;//,doEndFrame;
  niters=threadInfo->globals->niters;
#ifdef USEMKL
  mkl_set_num_threads(1);
#endif
  //memset(&schedParam,0,sizeof(struct sched_param));
  schedParam.sched_priority=1;
  if(sched_setscheduler(0,SCHED_RR,&schedParam)!=0){
    printf("Error in sched_setscheduler %s\n",strerror(errno));
  }
  if(sched_setparam(0,&schedParam)!=0){
    printf("Error in sched_setparam %s\n",strerror(errno));
  }
  while(niters!=0 && threadInfo->globals->go==1){
    //doEndFrame=0;//to be set to one if the pause flag is set (eg due to nsteps reaching zero or something).
    //agbthreadInfo->mybuf=1-threadInfo->mybuf;//switch buffers.
    
    dprintf("process frame thread %d niters %d mybuf %d frameno %d\n",threadInfo->threadno,niters,/*threadInfo->mybuf*/0,threadInfo->frameno);
    
    if(threadInfo->globals->doswitch){//a switch has been requested.
      dprintf("doswitch %d\n",threadInfo->threadno);
      //get the global start mutex
      if(pthread_mutex_lock(&threadInfo->globals->startMutex))//this should be released as soon as we're allowed - as soon as the first thread has done some of the buffer swapping.
	printf("pthread_mutex_lock error in processFrame : %s\n",strerror(errno));
      //get the per-camera start mutex
      if(pthread_mutex_lock(&threadInfo->info->startInfoMutex))//this should be released as soon as the first thread for this camera has done its buffer swapping work.
	printf("pthread_mutex_lock startInfoMutex error in processFrame : %s\n",strerror(errno));
      threadInfo->globals->threadCount++;
      threadInfo->info->threadInfoCount++;//counter for this camera...
      if(threadInfo->globals->threadCount==1){//first thread overall...
	if(pthread_mutex_lock(&threadInfo->globals->startFirstMutex))//this should be released once the work of the setting up has finished... to stop other threads continuing... and once cameras have been signalled that a new frame has started etc...
	  printf("pthread_mutex_lock startFirstMutex error in processFrame : %s\n",strerror(errno));
	//if(getClearSwitchRequested(threadInfo)){//a new parameter buffer is ready
	//thread safety of curbuf is okay - since we wait for all threads to finish previous frame before getting here... and only 1 thread is here at once.
	threadInfo->globals->curBuf=1-threadInfo->globals->curBuf;
#ifdef USECOND
	freezeParamBuf(threadInfo->globals->buffer[threadInfo->globals->curBuf],threadInfo->globals->buffer[1-threadInfo->globals->curBuf]);
#else
	freezeParamBuf(threadInfo->globals->buffer[threadInfo->globals->curBuf]->semid,threadInfo->globals->buffer[1-threadInfo->globals->curBuf]->semid);
#endif
	threadInfo->globals->buferr=0;
	if(updateBufferIndex(threadInfo)!=0){//error... probably not all params present in buffer...
	  threadInfo->info->pause=1;
	  threadInfo->globals->buferr=1;
	  if(threadInfo->globals->ppause!=NULL)
	    *(threadInfo->globals->ppause)=1;
	  pthread_mutex_unlock(&threadInfo->globals->startMutex);
	}else{
	  //have updated the index, so now other threads can continue.  


	  //if(threadInfo->globals->reconLib!=NULL)
	  //  (*threadInfo->globals->reconNewFrameFn)(threadInfo);
	  //Can release the startMutex now.
	  pthread_mutex_unlock(&threadInfo->globals->startMutex);
	  if(doCamBufferSwap(threadInfo,1)==0 && doThreadBufferSwap(threadInfo)==0){//no error...
	    //updateMemory allocates memory if required (eg if size changed).
	    swapArrays(threadInfo);
	    if(threadInfo->frameno==0)
	      createCircBufs(threadInfo);
	    updateCircBufs(threadInfo);
	    if(openLibraries(threadInfo)){
	      printf("Error opening libraries\n");
	    }
	  }else{
	    printf("Error doing buffer swap or updating memory - pausing\n");
	    //pause will have been set in doBufferSwap.
	  }
	}
	startNewFrame(threadInfo);
	//and clear the switchRequested parameter in the SHM buffer...
	if(threadInfo->globals->switchRequestedPtr!=NULL)
	  *threadInfo->globals->switchRequestedPtr=0;
	//Now allow other threads to continue.
	pthread_mutex_unlock(&threadInfo->globals->startFirstMutex);
	//Now can release the startInfoMutex for other threads of this cam
	pthread_mutex_unlock(&threadInfo->info->startInfoMutex);
      }else if(threadInfo->info->threadInfoCount==1){//first thread for this camera...
	pthread_mutex_unlock(&threadInfo->globals->startMutex);
	//if(threadInfo->globals->doswitch){//do a switch
	if(threadInfo->globals->buferr==0 && doCamBufferSwap(threadInfo,0)==0 && doThreadBufferSwap(threadInfo)==0){//no error
	  //now wait for first thread overall to have updated the memory etc
	  if(pthread_mutex_lock(&threadInfo->globals->startFirstMutex))//this should be released once the work of the setting up has finished...
	    printf("pthread_mutex_lock startFirstMutex error in processFrame : %s\n",strerror(errno));
	  swapArrays(threadInfo);
	}else{
	  //now wait for first thread overall to have updated the memory etc
	  if(pthread_mutex_lock(&threadInfo->globals->startFirstMutex))//this should be released once the work of the setting up has finished...
	    printf("pthread_mutex_lock startFirstMutex error in processFrame : %s\n",strerror(errno));
	}
	pthread_mutex_unlock(&threadInfo->globals->startFirstMutex);
	//Now can release the startInfoMutex for other threads of this cam
	pthread_mutex_unlock(&threadInfo->info->startInfoMutex);
      }else{//not first thread
	pthread_mutex_unlock(&threadInfo->globals->startMutex);
	//Now can release the startInfoMutex for other threads of this cam
	pthread_mutex_unlock(&threadInfo->info->startInfoMutex);
	if(threadInfo->globals->buferr==0)
	  doThreadBufferSwap(threadInfo);
      }
    }

    info=threadInfo->info;
    if(threadInfo->globals->go==0)
      break;
    ////////////////////////////////////////////////////////////////
    //Now do the computation...
    nsubapDone=0;
    err=0;
    if(info->pause==0){
      dprintf("Doing computation %d %d\n",threadInfo->threadno,niters);

      //if(info->kalmanUsed==0){
      //no need to get the libraryMutex here...
      if(threadInfo->globals->calibrateStartFrameFn!=NULL)
	(*threadInfo->globals->calibrateStartFrameFn)(threadInfo->globals->calibrateHandle,threadInfo->info->cam,threadInfo->threadno);
      if(threadInfo->globals->reconLib!=NULL && threadInfo->globals->reconHandle!=NULL)//First do the recon library.
	(*threadInfo->globals->reconStartFrameFn)(threadInfo->globals->reconHandle,threadInfo->threadno);
      //memset(threadInfo->dmCommand,0,threadInfo->dmCommandSize);
      //}else
	//memset(threadInfo->Xpred,0,threadInfo->XpredSize);
      //while(info->frameFinished[threadInfo->mybuf]==0){091109
      while(info->frameFinished==0){
	dprintf("framefinished %d niter %d\n",info->frameFinished,niters);
	/*
	if(info->centroidMode==CENTROIDMODE_WPU){//using FPGA WPU for centroiding.
	  if(nsubapDone==0)
	    waitPxlBufAndCentroidReady(threadInfo->globals);//wait until the pxlbuf and centroids array have been finished by the previous endframe thread...
	  dprintf("getting wpu centroid\n");
	  if((err=getNextCentroids(threadInfo))==0){//got subap okay
	    if(info->refCents!=NULL){//subtract reference centroids.
	      printf("TODO - subtract refCents from WPU\n");
	    }
	    nsubapDone++;
	    dprintf("got wpu centroid\n");
	  }else{
	    dprintf("error getting subap %d\n",err);
	  }
	}else{//using CPU for centroiding.
	*/
	dprintf("getting subap %d\n",info->id);
	if((err=waitNextSubaps(threadInfo))==0){//got subap okay
	  //here we need to use nsubapsTogether
	  cursubindx=threadInfo->cursubindx;
	  centindx=threadInfo->centindx;
	  //for(i=0; i<info->nsubapsTogether; i++){
	  for(i=0; i<threadInfo->nsubapsProcessing; i++){
	    if(info->subflag[threadInfo->cursubindx-info->subCumIndx]==1){
	      //this subap is valid, so do stuff...
	      //and now copy the pixels into our float array...
	      dprintf("copying subap %d\n",endFrame);
	      if(threadInfo->globals->calibrateNewSubapFn!=NULL)
		err=(*threadInfo->globals->calibrateNewSubapFn)(threadInfo->globals->calibrateHandle,threadInfo->info->cam,threadInfo->threadno,threadInfo->cursubindx,&threadInfo->subap,&threadInfo->subapSize);
	      
	      
	      copySubap(threadInfo);
	      subapPxlCalibration(threadInfo);
	      if(err==0 && nsubapDone==0){
		dprintf("WaitPxlBufAndCentroidReady\n");
		waitPxlBufAndCentroidReady(threadInfo->globals);//wait until the pxlbuf and centroids array have been finished by the previous endframe thread...
	      }
	      dprintf("storeCalibratedSubap\n");
	      storeCalibratedSubap(threadInfo);
	      if(threadInfo->globals->centCalcFn!=NULL){
		err=(*threadInfo->globals->centCalcFn)(threadInfo->globals->centHandle,threadInfo->subap);
	      }
	      dprintf("calibrated\n");
	      calcCentroid(threadInfo);
	      dprintf("centroided\n");
	      nsubapDone++;
	      threadInfo->centindx+=2;
	    }
	    threadInfo->cursubindx++;
	  }
	  //reset these for the next stage (reconstruction)
	  threadInfo->cursubindx=cursubindx;
	  threadInfo->centindx=centindx;
	  //Now, we see if the centroider interface has anything to offer...
	  //i.e. a WPU or such.
	  //This blocks until enough slopes have been received by the interface, as defined by threadInfo->nsubs. (of course, if you don't want your interface to block, then just don't).
	  //Once they have been received by the interface, they are added into arrays->centroids, i.e. for i=0;i<nsubapsDoing; i++ centroids[centindx]+=newslopes[centindx]
	  if(err==0){
	    if((err=waitCentroids(threadInfo))!=0){//didn't subap okay
	      writeErrorVA(threadInfo->globals->rtcErrorBuf,SLOPEERROR,threadInfo->globals->thisiter,"Error getting slopes");
	    }
	  }
	  
        }else{//didn't get subap okay.
	  dprintf("error getting subap %d\n",err);
	}
	//}
	if(err==0){//otherwise, frame probably finished (-1) or error (1)...

	  dprintf("reconstructing...\n");
	  //partialReconstruct(threadInfo);
	  if(threadInfo->globals->reconLib!=NULL && threadInfo->globals->reconHandle!=NULL){
	    (*threadInfo->globals->reconNewSlopesFn)(threadInfo->globals->reconHandle,threadInfo->centindx,threadInfo->threadno,threadInfo->nsubapsDoing,threadInfo->info->centroids,threadInfo->globals->arrays->dmCommand);
	  }

	  dprintf("reconstructed\n");
	  //nsubapDone++;
	}else if(err==-1)//frame finished
	  err=0;
	else
	  threadInfo->err=1;
      }
      dprintf("Done computation %d %d\n",threadInfo->threadno,niters);
      //Each thread should then add its own dmCommand to the global dmCommand in
      //a thread safe way... First, need to wait for the initialisation of these arrays to be ready..
      //copyThreadPhase(threadInfo);//this gets the mutex...
      if(threadInfo->globals->calibrateEndFrameFn!=NULL)
	(*threadInfo->globals->calibrateEndFrameFn)(threadInfo->globals->calibrateHandle,threadInfo->info->cam,threadInfo->threadno,threadInfo->err);
      if(threadInfo->globals->reconLib!=NULL && threadInfo->globals->reconHandle!=NULL){
	(*threadInfo->globals->reconEndFrameFn)(threadInfo->globals->reconHandle,threadInfo->threadno,threadInfo->info->centroids,threadInfo->globals->arrays->dmCommand,threadInfo->err);
      }
    }else{
      dprintf("Paused %d %d\n",threadInfo->threadno,niters);
    }
    threadInfo->frameno++;
    //////////////////////////////////////////////////////////////////
    //wait for all threads to finish this frame
    if(pthread_mutex_lock(&threadInfo->globals->endMutex))//091109[threadInfo->mybuf]))
      printf("pthread_mutex_lock error in processFrame4:  %s\n",strerror(errno));
    threadInfo->globals->threadCountFinished++;//091109[threadInfo->mybuf]++;
    info->pxlCentInputError|=threadInfo->err;
    threadInfo->err=0;
    //The last thread for each camera has some tidying up to do...
    if(info->threadCountFinished==info->nthreads-1){
      threadInfo->info->threadInfoCount=0;
      info->threadCountFinished=0;
      info->subindx=0;
      info->centindx=0;
      dprintf("resetting info->threadCountFinished\n");
      info->frameFinished=0;//091109[threadInfo->mybuf]=0;
      threadInfo->globals->pxlCentInputError|=info->pxlCentInputError;
      info->pxlCentInputError=0;//reset for next time.
      if(info->pause==0){//not paused...
	if(info->nsteps>1)
	  info->nsteps--;
	else if(info->nsteps==1){
	  info->pause=1;
	  info->nsteps=0;
	  //doEndFrame=1;
	  if(threadInfo->globals->ppause!=NULL)
	    *(threadInfo->globals->ppause)=1;//set the shm paused flag
	}
      }
    }else{
      info->threadCountFinished++;
    }
    //now the last thread to finish overall has some stuff to do...
    //091109if(threadInfo->globals->threadCountFinished[threadInfo->mybuf]==threadInfo->globals->nthreads){//last thread to finish
    if(threadInfo->globals->threadCountFinished==threadInfo->globals->nthreads){//last thread to finish
      threadInfo->globals->doswitch=0;//091109[threadInfo->mybuf]=0;
      threadInfo->globals->threadCount=0;//091109[threadInfo->mybuf]=0;

      threadInfo->globals->threadCountFinished=0;//091109[threadInfo->mybuf]=0;
      threadInfo->globals->camReadCnt=0;
      threadInfo->info->pxlCentInputError=threadInfo->globals->pxlCentInputError;
      gettimeofday(&thistime,NULL);
      timestamp=thistime.tv_sec+thistime.tv_usec*1e-6;
      threadInfo->globals->precomp->post.timestamp=timestamp;
      if(threadInfo->info->pause==0){// || doEndFrame==1){
	endFrame(threadInfo);//not paused...
      }else{//paused
	threadInfo->globals->thisiter++;//have to increment this so that the frameno changes in the circular buffer, OTHERWISE, the buffer may not get written
	writeStatusBuf(threadInfo->globals,1,threadInfo->info->closeLoop);
	circAdd(threadInfo->globals->rtcStatusBuf,threadInfo->globals->statusBuf,timestamp,threadInfo->globals->thisiter);
      }
      threadInfo->info->pxlCentInputError=0;
      threadInfo->globals->pxlCentInputError=0;

      //timing info stuff...
      if(threadInfo->globals->starttime==0){//first one....tv_sec
	threadInfo->globals->starttime=timestamp;//thistime;
	dtime=0;
	circAdd(threadInfo->globals->rtcTimeBuf,&dtime,timestamp,threadInfo->globals->thisiter);
	//gettimeofday(&threadInfo->globals->starttime,NULL);
      }else{
	//gettimeofday(&thistime,NULL);
	dtime=timestamp-threadInfo->globals->starttime;//thistime.tv_sec*1e6+thistime.tv_usec-threadInfo->globals->starttime.tv_sec*1e6-threadInfo->globals->starttime.tv_usec;
	threadInfo->globals->frameTime=dtime;
	threadInfo->globals->sumtime+=dtime;
	threadInfo->globals->sum2time+=dtime*dtime;
	threadInfo->globals->ntime++;
	if(dtime>threadInfo->globals->maxtime && threadInfo->globals->thisiter>10){
	  threadInfo->globals->maxtime=dtime;
	  threadInfo->globals->maxtimeiter=threadInfo->globals->thisiter;
	}
	threadInfo->globals->starttime=timestamp;//thistime;
	if(threadInfo->info->printTime){
	  printf("%10fs\t%10fHz\t%10fs           \r",dtime,1/dtime,threadInfo->globals->maxtime);
	  fflush(NULL);
	}
	circAdd(threadInfo->globals->rtcTimeBuf,&dtime,timestamp,threadInfo->globals->thisiter);
	//threadInfo->globals->thisiter++;//This is now updated earlier, when first pixels arrive...
      }
      //Now, we can check to see if a buffer swap is required.  If not, then do the start of frame stuff here, if so, then set correct flags...
      if(getSwitchRequested(threadInfo->globals)){//a new parameter buffer is ready
	threadInfo->globals->doswitch=1;
      }else{//signal to cameras etc.
	startNewFrame(threadInfo);
      }

      //tell the other threads that we've finished.
      pthread_cond_broadcast(&(threadInfo->globals->frameRunningCond));//091109[threadInfo->mybuf]));

      
    }else{//wait for the last thread to finish
      //091109if(pthread_cond_wait(&(threadInfo->globals->frameRunningCond[threadInfo->mybuf]),&(threadInfo->globals->endMutex[threadInfo->mybuf])))
      if(pthread_cond_wait(&(threadInfo->globals->frameRunningCond),&(threadInfo->globals->endMutex)))
	printf("cond_wait error frameRunning\n");
    }
    pthread_mutex_unlock(&threadInfo->globals->endMutex);//091109[threadInfo->mybuf]);
    
    
    if(niters>0)
      niters--;
  }
  printf("processFrame ending - done thread %d\n",threadInfo->threadno);
  if(threadInfo->threadno==0){
    printf("mean time %gus (%d iters), stdev %gus, max %gus (iter %d)\n",threadInfo->globals->sumtime/threadInfo->globals->ntime,threadInfo->globals->ntime,sqrt(threadInfo->globals->sum2time/threadInfo->globals->ntime-(threadInfo->globals->sumtime/threadInfo->globals->ntime)*(threadInfo->globals->sumtime/threadInfo->globals->ntime)),threadInfo->globals->maxtime,threadInfo->globals->maxtimeiter);
    PRINTTIMING(endFrame);
    PRINTTIMING(partialReconstruct);
    PRINTTIMING(subtractBg);
    PRINTTIMING(calcCentroid);
    PRINTTIMING(applyFlatField);
    PRINTTIMING(applyThreshold);
    PRINTTIMING(subapPxlCalibration);
    PRINTTIMING(applyPhase);
    PRINTTIMING(sendActuators);
    
  }
  return 0;
}

void *startThreadFunc(void *t){
  processFrame((threadStruct*)t);
  return NULL;

}
void *runPrepareActuators(void *glob){
  prepareActuators((globalStruct*)glob);
  return NULL;
}
