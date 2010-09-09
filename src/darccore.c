/**
This file does the bulk of processing for the RTC.
*/
#define USECOND
#define CVSID MAINCVSID"$Id: darccore.c,v 1.41 2010/07/16 07:50:17 ali Exp $"
#ifdef FORPYTHON
#include <Python.h>
#include <numpy/arrayobject.h>
#else
#define _GNU_SOURCE//already defined in Python.h
#endif
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
#include "recon.h"
#include "darcNames.h"
#include "darc.h"
#include "qsort.h"

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

//this enum is used while reading the parameter buffer.
/*
typedef enum{NCAM,
	       NACTS,
	       NSUBX,
	       NSUBY,
	       NPXLX,
	       NPXLY,
	       KALMANPHASESIZE,
	       KALMANUSED,
	       REFCENTROIDS,
	       SUBAPLOCATION,
	       BGIMAGE,
	       DARKNOISE,
	       FLATFIELD,
	       THRESHOLDALGORITHM,
	       THRESHOLDVALUE,
	       POWERFACTOR,
	       CENTROIDWEIGHTING,
	       WINDOWMODE,
	       KALMANRESET,
	       USINGDMC,
	       KALMANHINFT,
	       KALMANHINFDM,
	       KALMANATUR,
	       KALMANINVN,
	       SUBAPFLAG,
	       GO,
	       PXLCNT,
	       CENTROIDMODE,
	       GAINRECONMXT,
	       RECONSTRUCTMODE,
	       GAINE,
	       V0,
	       BLEEDGAIN,
	       MIDRANGE,
	       ACTMAX,
	       PAUSE,
	       PRINTTIME,
	       NCAMTHREADS,
	       SWITCHREQUESTED,
	       USERACTS,
	       FAKECCDIMAGE,
	       THREADAFFINITY,
	       THREADPRIORITY,
	       DELAY,
	       MAXCLIPPED,
	       CLEARERRORS,
	       CAMERASOPEN,
	       CAMERASFRAMING,
	       CAMERAPARAMS,
	       CAMERANAME,
	       MIRROROPEN,
	       MIRRORNAME,
	       FRAMENO,//this one is readonly - used to signify at what frame the buffer was last swapped... useful for saving status.
	       SWITCHTIME,//readonly - the time at which the param buffer was last swapped - useful for saving status.
	       ADAPTIVEWINGAIN,
	       CORRELATIONTHRESHOLDTYPE,//fixed value of fraction of peak?
	       CORRELATIONTHRESHOLD,//the value/fraction to use.
	       FFTCORRELATIONPATTERN,//array holding the spot PSFs.
	       NSUBAPSTOGETHER,//no of subaps evalued by one thread at a time
	     NSTEPS,//number of iterations to do before pausing (<=0 for continuous).
	     ACTMIN,
	     CLOSELOOP,//whether to send to the mirror or not.
	     MIRRORPARAMS,
	     ADDUSERACTS,//whether to add userActs to the calculated actuators.
	     USERACTSEQ,//if useracts is a 2d array, the number of times to send each line... 
	     RECORDCENTS,//whether to record centroids when useracts!=NULL...
	     PXLWEIGHT,//pixel weighting applied before centroiding.
	     AVERAGEIMG,//how many frames of calpxl to average before sending to generic stream.
	     CENTOPEN,
	     CENTFRAMING,
	     CENTPARAMS,
	     CENTNAME,
	     USERACTSMASK,
	     //Add more before this line.
	     NBUFFERVARIABLES//equal to number of entries in the enum
}BUFFERVARIABLEINDX;*/

//currently, only basic windowing implemented (centroid algorithm)

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
   Add the phase reconstructed by a given thread into the global phase memory in a thread-safe way.
   @return 0
 */
/*
int copyThreadPhase(threadStruct *threadInfo){
  //wait until the DM/kalman arrays can be written to by the worker threads
  //ie until the precompute stuff has finished.
  //Then copys the per-thread info (dmCommand or Xpred into the global array.
  infoStruct *info=threadInfo->info;
  globalStruct *glob=threadInfo->globals;
  if(pthread_mutex_lock(&glob->precomp->dmMutex))
    printf("pthread_mutex_lock error in copyThreadPhase: %s\n",strerror(errno));
  if(glob->precomp->dmReady==0)//wait for the precompute thread to finish (it will call setDMArraysReady when done)...
    if(pthread_cond_wait(&glob->precomp->dmCond,&glob->precomp->dmMutex))
      printf("pthread_cond_wait error in copyThreadPhase: %s\n",strerror(errno));
  if(threadInfo->info->kalmanUsed==0){
    //now add threadInfo->dmCommand to threadInfo->info->dmCommand.
    cblas_saxpy(info->nacts,1.,threadInfo->dmCommand,1,info->dmCommand,1);
  }else{
    cblas_saxpy(info->kalmanPhaseSize*3,1.,threadInfo->Xpred,1,info->Xpred,1);
  }
  pthread_mutex_unlock(&glob->precomp->dmMutex);
  return 0;
}
*/
/**
   Unblock threads waiting to write to the dmCommand arrays - this is called by the precomputation thread
   @return 0
 */
/*
int setDMArraysReady(globalStruct *glob){
  //precompute stuff has now finished...
  if(pthread_mutex_lock(&glob->precomp->dmMutex))
    printf("pthread_mutex_lock error in setDMArraysReady: %s\n",strerror(errno));
  glob->precomp->dmReady=1;
  pthread_cond_broadcast(&glob->precomp->dmCond);
  pthread_mutex_unlock(&glob->precomp->dmMutex);
  return 0;
}
*/
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
  imax=info->nsubx*info->nsuby-(threadInfo->cursubindx-info->subCumIndx);
  for(i=0; i<info->nsubapsTogether && i<imax; i++){
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
    //if(loc[0]!=rloc[0] || loc[1]!=rloc[1] || loc[2]!=rloc[2] || loc[3]!=rloc[3] || loc[4]!=rloc[4] || loc[5]!=rloc[5]){
    //  printf("Updated %d %d %d %d %d %d\n",loc[0],loc[1],loc[2],loc[3],loc[4],loc[5]);
    //}
    //windows have now been computed for next time...
    //Calculate the number of extra pixels required from the standard position...
  //if(loc==NULL)//not sure this will ever occur... but it might if the loop is not evaluated ie cursubindx is >=nsubx*nsuby.
      // return 0;
  //here, we calculate the number of extra pixels required for the last subap of this block.  
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
	threadInfo->subap[cnt]=(float)info->pxlbuf[i*info->npxlx+j];
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
   @return 0 unless an error has occurred.
 */
int waitPixels(int cnt,threadStruct *threadInfo){
  int rt=0;
  if(threadInfo->globals->cameraLib!=NULL)
    rt=(*threadInfo->globals->camWaitPixelsFn)(cnt,threadInfo->info->cam,threadInfo->globals->camHandle);
  return rt;
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
  //if(info->subindx==0 && threadInfo->globals->cameraLib!=NULL){//tell the camera library that a new frame has started
  //  (*threadInfo->globals->camNewFrameFn)(threadInfo->globals->camHandle);
  //}
  //Here, if threads are doing 1 subap at a time, we move on til we get to the next subap being evaluated.  Otherwise, we need to count how many subaps will be valid for this one.
  /*
  if(info->nsubapsTogether==1){
    while(info->subflag[info->subindx]==0){
      dprintf("subflag[%d] %d\n",info->subindx,info->subflag[info->subindx]);
      info->subindx++;
      if(info->subindx>=info->nsubx*info->nsuby){
	info->subindx=0;
	info->centindx=0;
	endFrame=-1;
	info->frameFinished[threadInfo->mybuf]=1;//let other threadsknow that we've read all the subaps...
	break;
      }
    }
  }else{//doing more than 1 subap at once...
  */
  while(cnt==0 && info->subindx<info->nsubx*info->nsuby){
    npxls=0;
    for(i=0; i<info->nsubapsTogether && i<info->nsubx*info->nsuby-info->subindx; i++){
      cnt+=info->subflag[info->subindx+i];
      //see how many pixels this subap requires..., store the max so far.
      if(info->subflag[info->subindx+i]==1 && info->pxlCnt[info->subindx+i+info->subCumIndx]>npxls)
	npxls=info->pxlCnt[info->subindx+i+info->subCumIndx];
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
  //}
  if(endFrame==0){
    //store the indx for this thread...
    threadInfo->centindx=info->centindx+info->centCumIndx;
    threadInfo->cursubindx=info->subindx+info->subCumIndx;
    threadInfo->nsubapsDoing=cnt;
    //threadInfo->cursuby=info->subindx/info->nsubx;
    //threadInfo->cursubx=info->subindx%info->nsubx;
    //and increment subindx ready for the next thread.
    info->centindx+=2*cnt;//info->nsubapsTogether;//increase by number of used subaps by this thread.
    info->subindx+=info->nsubapsTogether;
    if(info->subindx>=info->nsubx*info->nsuby){
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
    endFrame=(waitPixels(npxls,threadInfo)!=0);//info->pxlCnt[threadInfo->cursubindx+info->nsubapsTogether-1]+extrapxl,threadInfo);
    dprintf("waited pixels\n");
    //The frame number will have been set.  So, if this is the first thread for this camera, can store the cam frame number.  If the first thread overall, can store glob->thisiter.  Also need to check that all frame numbers from each camera are the same - raise error if not.
    if(endFrame==0){
      pthread_mutex_lock(&threadInfo->globals->camMutex);
      if(threadInfo->globals->camReadCnt==0){//first thread overall
	threadInfo->globals->setFrameno=1;
      }
      if(threadInfo->globals->setFrameno){//need to set frame no
	if(info->cam==0){//first cam can set frameno
	  threadInfo->globals->setFrameno=0;
	  if(threadInfo->globals->camiter==threadInfo->globals->camframeno[0]){//it would seem that the camera isn't updating the iteration count.
	    threadInfo->globals->thisiter++;
	  }else{//camera is updating the iteration count (frameno)
	    threadInfo->globals->thisiter=threadInfo->globals->camframeno[0];
	    threadInfo->globals->camiter=threadInfo->globals->thisiter;
	  }
	}else{
	  //we're not the first cam, but the frameno hasn't yet been set, so we have nothing to check against.  Is there anything I can do here?  Don't think so really, just have to hope!!!
	}
      }else{//frameno has already been set, so we can check against it.
	if(threadInfo->globals->camiter!=threadInfo->globals->camframeno[info->cam]){
	  writeErrorVA(threadInfo->globals->rtcErrorBuf,CAMSYNCERROR,threadInfo->globals->thisiter,"Error - camera frames not in sync");
	}
      }
      /*
      if(threadInfo->globals->camReadCnt==0){//first thread overall
	if(threadInfo->globals->camiter==threadInfo->globals->camframeno[info->cam]){//it would seem that the camera isn't updating the iteration count...
	  threadInfo->globals->thisiter++;
	}else{//camera is updating the iteration count (frame number)
	  threadInfo->globals->thisiter=threadInfo->globals->camframeno[info->cam];
	  threadInfo->globals->camiter=threadInfo->globals->thisiter;
	}
      }else{//not first thread...
	if(threadInfo->globals->camiter!=threadInfo->globals->camframeno[info->cam]){
	  writeErrorVA(threadInfo->globals->rtcErrorBuf,CAMSYNCERROR,threadInfo->globals->thisiter,"Error - camera frames not in sync");
	}
      }
      */
      threadInfo->globals->camReadCnt++;
      pthread_mutex_unlock(&threadInfo->globals->camMutex);
    }else{
      writeErrorVA(threadInfo->globals->rtcErrorBuf,CAMGETERROR,threadInfo->globals->thisiter,"Error - getting camera pixels");
      threadInfo->globals->thisiter++;//have to increment iteration number, otherwise some of the circular buffers won't get written.

    }
  }
  //then release mutex so that next thread can wait for pixels, and reset this sem, so that this thread will wait next time...
  pthread_mutex_unlock(&info->subapMutex);//unlock the mutex, allowing the next thread to continue
  dprintf("freed muted\n");
  return endFrame;
}

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

/*
inline int qfind(float *a,int n,float x){
  int i=0;
  int mid;
  while(i<n){
    mid=(n+i)/2;
    if(x<a[mid])
      n=mid;
    else if(x>a[mid])
      i=mid+1;
    else
      return mid-1;
  }
  return (n+i)/2-1;
}
*/
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
  /*
  //Now sort the remaining pixels into this
  for(i=n; i<threadInfo->curnpxl; i++){
    if((v=threadInfo->subap[i])>sort[0]){
      //insert this value in...
      pos=qfind(sort,n,v);
      if(pos>0)
	memmove(sort,&sort[1],sizeof(float)*pos);
      sort[pos]=v;
    }
    }
    thr=sort[0];//the threshold
  */
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
  //STARTTIMING;
  loc=&(info->subapLocation[threadInfo->cursubindx*6]);
  if(info->calmult!=NULL && info->calsub!=NULL){
    if((info->thresholdAlgo==1 || info->thresholdAlgo==2) && info->calthr!=NULL){
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	for(j=loc[3];j<loc[4]; j+=loc[5]){
	  threadInfo->subap[cnt]*=info->calmult[info->npxlCum+i*info->npxlx+j];
	  threadInfo->subap[cnt]-=info->calsub[info->npxlCum+i*info->npxlx+j];
	  if(threadInfo->subap[cnt]<info->calthr[info->npxlCum+i*info->npxlx+j])
	    threadInfo->subap[cnt]=0;
	  cnt++;
	}
      }
    }else{//no thresholding
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	for(j=loc[3];j<loc[4]; j+=loc[5]){
	  threadInfo->subap[cnt]*=info->calmult[info->npxlCum+i*info->npxlx+j];
	  threadInfo->subap[cnt]-=info->calsub[info->npxlCum+i*info->npxlx+j];
	  cnt++;
	}
      }
    }
  }else if(info->calmult!=NULL){
    if((info->thresholdAlgo==1 || info->thresholdAlgo==2) && info->calthr!=NULL){
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	for(j=loc[3];j<loc[4]; j+=loc[5]){
	  threadInfo->subap[cnt]*=info->calmult[info->npxlCum+i*info->npxlx+j];
	  if(threadInfo->subap[cnt]<info->calthr[info->npxlCum+i*info->npxlx+j])
	    threadInfo->subap[cnt]=0;
	  cnt++;
	}
      }
    }else{//no thresholding
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	for(j=loc[3];j<loc[4]; j+=loc[5]){
	  threadInfo->subap[cnt]*=info->calmult[info->npxlCum+i*info->npxlx+j];
	  cnt++;
	}
      }
    }
  }else if(info->calsub!=NULL){
    if((info->thresholdAlgo==1 || info->thresholdAlgo==2 ) && info->calthr!=NULL){
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	for(j=loc[3];j<loc[4]; j+=loc[5]){
	  threadInfo->subap[cnt]-=info->calsub[info->npxlCum+i*info->npxlx+j];
	  if(threadInfo->subap[cnt]<info->calthr[info->npxlCum+i*info->npxlx+j])
	    threadInfo->subap[cnt]=0;
	  cnt++;
	}
      }
    }else{//no thresholding
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	for(j=loc[3];j<loc[4]; j+=loc[5]){
	  threadInfo->subap[cnt]-=info->calsub[info->npxlCum+i*info->npxlx+j];
	  cnt++;
	}
      }
    }
  }else{//both are null...
    if((info->thresholdAlgo==1 || info->thresholdAlgo==2) && info->calthr!=NULL){
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	for(j=loc[3];j<loc[4]; j+=loc[5]){
	  if(threadInfo->subap[cnt]<info->calthr[info->npxlCum+i*info->npxlx+j])
	    threadInfo->subap[cnt]=0;
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
int calcAdaptiveWindow(threadStruct *threadInfo,float *cx,float *cy){
  infoStruct *info=threadInfo->info;
  //add centroid offsets to get the overall location correct
  //(i.e. the distance from it's nominal centre).
  *cx+=info->adaptiveCentPos[threadInfo->centindx];
  *cy+=info->adaptiveCentPos[threadInfo->centindx+1];
  //Now use these values to calculate the window position for next time.
  info->adaptiveWinPos[threadInfo->centindx]*=1-info->adaptiveWinGain;
  info->adaptiveWinPos[threadInfo->centindx+1]*=1-info->adaptiveWinGain;
  info->adaptiveWinPos[threadInfo->centindx]+=*cx*info->adaptiveWinGain;
  info->adaptiveWinPos[threadInfo->centindx+1]+=*cy*info->adaptiveWinGain;
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
#define B(y,x) info->fftCorrelationPattern[(loc[0]+y*loc[2])*info->npxlx+loc[3]+x*loc[5]]
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
  /*
  if(threadInfo->globals->fftPlanningDone==0){//planning not yet done
    //Not safe to access fftPlanArray.
    pthread_mutex_lock(&threadInfo->globals->camMutex);//reuse camMutex...
    if(threadInfo->globals->fftPlanningDone==0){//planning still not yet done
      pthread_join(threadInfo->globals->fftPlanThreadid,NULL);
      threadInfo->globals->fftPlanningDone=1;
    }
    pthread_mutex_unlock(&threadInfo->globals->camMutex);
  }
  if(threadInfo->curnpxly>=threadInfo->globals->fftPlanArrayXsize || threadInfo->curnpxlx>=threadInfo->globals->fftPlanArrayXsize){//need to resize it...
    //max subap size has changed, so need to generate some more...
    pthread_mutex_lock(&threadInfo->globals->camMutex);//reuse camMutex...
    //just check again, now we're in the thread safe part
    if(threadInfo->curnpxly>=threadInfo->globals->fftPlanArrayXsize || threadInfo->curnpxlx>=threadInfo->globals->fftPlanArrayXsize){//need to resize it...
      //do the extra planning
    }
    pthread_mutex_unlock(&threadInfo->globals->camMutex);//reuse camMutex...
  }



  //First, get the plans for subaps of this size...
  fPlan=threadInfo->globals->fftPlanArray[threadInfo->curnpxly*threadInfo->globals->fftPlanArrayXsize+threadInfo->curnpxlx];//the forward plan
  ifPlan=threadInfo->globals->ifftPlanArray[threadInfo->curnpxly*threadInfo->globals->fftPlanArrayXsize+threadInfo->curnpxlx];//the inverse plan
  */
  //FFT the SH image.
  fftwf_execute_r2r(fPlan,threadInfo->subap,threadInfo->subap);
  
  //Now multiply by the reference...
  //This is fairly complicated due to the half complex format.  If you need to edit this, make sure you know what you're doing.
  //Here, we want to use the real subap location rather than the moving one, because this image map in question (the fft'd psf) doesn't move around, and we're just using subap location for convenience rather than having to identify another way to specify it.
  loc=&(info->realSubapLocation[threadInfo->cursubindx*6]);
  a=threadInfo->subap;
  n=threadInfo->curnpxlx;
  m=threadInfo->curnpxly;
  //for(i=0; i<threadInfo->curnpxly; i++){
  //  for(j=0; j<threadInfo->curnpxlx; j++){
  //    //may need to do something more complex with this multiplication - ie sort out the real and imaginary parts... or it may just work!
  //    threadInfo->subap[i*threadInfo->curnpxlx+j]*=info->fftCorrelationPattern[(loc[0]+i*loc[2])*info->npxlx+loc[3]+j*loc[5]];
  //  }
  //}

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
    /*
    sum=cblas_sasum(threadInfo->curnpxl,threadInfo->subap,1);
    if(sum>minflux){
      for(i=0; i<threadInfo->curnpxlx; i++){
	cx+=i*cblas_sasum(threadInfo->curnpxly,&(threadInfo->subap[i]),threadInfo->curnpxlx);
      }
      for(i=0; i<threadInfo->curnpxly; i++){
	cy+=i*cblas_sasum(threadInfo->curnpxlx,&(threadInfo->subap[i*threadInfo->curnpxlx]),1);
      }
      cy/=sum;
      cx/=sum;
      cy-=threadInfo->curnpxly/2.-0.5;
      cx-=threadInfo->curnpxlx/2.-0.5;
    }else{
    }*/
  }else if(info->centroidMode==CENTROIDMODE_GAUSSIAN || info->centroidMode==CENTROIDMODE_CORRELATIONGAUSSIAN){
    //do some sort of gaussian fit to the data...
    printf("TODOxxx - gaussian fit to the data to get centroid (not yet implemented)\n");
  }else{
    printf("centroid mode not yet implemented\n");
  }
  if(sum>minflux){
    if(info->centCalData!=NULL){//appy centroid linearisation...
      //Here, we apply the centroid linearisation.
      applySlopeLinearisation(threadInfo,&cx,&cy);
    }
    if(info->refCents!=NULL){//subtract reference centroids.
      cx-=info->refCents[threadInfo->centindx];
      cy-=info->refCents[threadInfo->centindx+1];
    }
    if(info->windowMode==WINDOWMODE_ADAPTIVE)
      calcAdaptiveWindow(threadInfo,&cx,&cy);
    else if(info->windowMode==WINDOWMODE_GLOBAL){//add the current subap offset here
      cx+=info->adaptiveCentPos[threadInfo->centindx];
      cy+=info->adaptiveCentPos[threadInfo->centindx+1];
    }
  }
  info->centroids[threadInfo->centindx]=cx;
  info->centroids[threadInfo->centindx+1]=cy;
  info->flux[threadInfo->centindx/2]=sum;
  ENDTIMING(calcCentroid);
  return 0;
}


/**
   Apply the kalman filtering algorithm using slopes from a single subap.  This is called once for each subap, building up the final result.
*/
/*
Has been moved into reconKalman.c
int partialKalman(threadStruct *threadInfo){
  CBLAS_ORDER order=CblasColMajor;
  CBLAS_TRANSPOSE trans=CblasNoTrans;
  //infoStruct *info=threadInfo->info;
  float alpha=1.,beta=1.;
  int inc=1;
  int step;//=2;
  //float cents[2];
  //float tmp;
  infoStruct *info=threadInfo->info;
  
  if(info->kalmanReset==0){
    //update...
    //apply Xpref+=Hinf[:,indx]*(centx-precompDMXpref[indx])+Hinf[:,indx+1]*(centx-precompDMXpref[indx+1])
    //Note, Hinf here is actually transposed...
    //cblas_saxpy(info->kalmanPhaseSize*3,info->centroids[threadInfo->centindx]-info->precompDMXpred[threadInfo->centindx],&(info->kalmanHinfT[threadInfo->centindx*3*threadInfo->info->kalmanPhaseSize]),1,threadInfo->Xpred,1);
    //cblas_saxpy(info->kalmanPhaseSize*3,info->centroids[threadInfo->centindx+1]-info->precompDMXpred[threadInfo->centindx+1],&(info->kalmanHinfT[(threadInfo->centindx+1)*3*threadInfo->info->kalmanPhaseSize]),1,threadInfo->Xpred,1);
    //cents[0]=info->centroids[threadInfo->centindx];//-info->precompDMXpred[threadInfo->centindx];
    //cents[1]=info->centroids[threadInfo->centindx+1];//-info->precompDMXpred[threadInfo->centindx+1];
    //tmp=threadInfo->Xpred[0];
    step=2*threadInfo->nsubapsDoing;
    cblas_sgemv(order,trans,info->kalmanPhaseSize*3,step,alpha,&(info->kalmanHinfT[threadInfo->centindx*3*info->kalmanPhaseSize]),info->kalmanPhaseSize*3,&(info->centroids[threadInfo->centindx]),inc,beta,threadInfo->Xpred,inc);
  }
  return 0;
}
*/
/**
   Apply part of the standard matrix vector multiplication algorithm, using slopes from a single subap.  This is called once for each subap, building up the final result, allowing a frame to be processed as it arrives, rather than having to wait for a whole frame to be present first - reduces latency.
 */
/*
int partialReconstruct(threadStruct *threadInfo){
  CBLAS_ORDER order=CblasColMajor;
  CBLAS_TRANSPOSE trans=CblasNoTrans;
  //infoStruct *info=threadInfo->info;
  float alpha=1.,beta=1.;
  int inc=1;
  int step;//=2;//number of rows to do in mmx...
  //int tmp;
  //int indx;
  //int domult=0;
  infoStruct *info=threadInfo->info;
  STARTTIMING;
  //getDMCommandBufferLock(threadInfo);//wait til we can write into the buffer...
  //pthread_mutex_lock(&threadInfo->globals->precomp->prepMutex);
  if(info->kalmanUsed){
    //the buffer should have been reset previously by the prepare function if kalmanReset==1...
    partialKalman(threadInfo);
  }else{
    //We assume that each row i of the reconstructor has already been multiplied by gain[i].  
    //So, here we just do dmCommand+=rmx[:,n]*centx+rmx[:,n+1]*centy.
    dprintf("in partialReconstruct %d %d %d %p %p %p\n",info->nacts,threadInfo->centindx,info->totCents,info->centroids,info->rmxT,threadInfo->dmCommand);
    step=2*threadInfo->nsubapsDoing;
    //todo("check that the step is okay here... and that nacts should be used, not 1, since its rmxT...");
    cblas_sgemv(order,trans,info->nacts,step,alpha,&(info->rmxT[threadInfo->centindx*info->nacts]),info->nacts,&(info->centroids[threadInfo->centindx]),inc,beta,threadInfo->dmCommand,inc);
  }
  ENDTIMING(partialReconstruct);
  return 0;
}
*/
/**
   Takes the kalman phase, and if not using the DMC, converts this to actuators values.
   @return 0

int applyPhase(PostComputeData *p){//this is called if kalmanUsed==1.
  CBLAS_ORDER order=CblasRowMajor;
  CBLAS_TRANSPOSE trans=CblasNoTrans;
  //infoStruct *info=threadInfo->info;
  float alpha=1.,beta=0.;
  int inc=1;
  //STARTTIMING;
  if(p->usingDMC==0)
    cblas_sgemv(order,trans,p->nacts,p->kalmanPhaseSize,alpha,p->kalmanInvN,p->kalmanPhaseSize,p->Xdmc,inc,beta,p->dmCommand,inc);
  //ENDTIMING(applyPhase);
  return 0;
  }*/

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
  //int nclipped=0;
  //int actMax=p->actMax;
  //int actMin=p->actMin;
  //float bleed=p->bleedGainOverNact;
  //float *v0=p->v0;
  //float midrange=p->midRangeTimesBleed;
  float *userActs;//=p->userActs;
  float *userActsMask=p->userActsMask;
  int addUserActs=p->addUserActs;
  int *userActSeq=p->userActSeq;
  int userActSeqLen=p->userActSeqLen;
  int record=*p->recordCents;
  //int seqCnt=p->seqCnt;
  //int actCnt=p->actCnt;
  //float bleedVal=0;
  //int intDMCommand;
  //int nsaturate=0;//uint16 overflow counter...
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
  /*if(pp->kalmanUsed==1 && pp->usingDMC==0){
  //memcpy(pp->Xdmc,pp->Xpred,sizeof(float)*pp->kalmanPhaseSize*3);
  //applyPhase(pp);
  beta=0;//compute the dmCommand from the kalman phase.
  cblas_sgemv(order,trans,pp->nacts,pp->kalmanPhaseSize,alpha,pp->kalmanInvN,pp->kalmanPhaseSize,pp->Xpred,inc,beta,pp->dmCommand,inc);
  }*/
  //Call the library function to finish reconstructor computation...
  //Result is put into dmCommand.
  
  
  pthread_mutex_lock(&glob->reconMutex);
  if(glob->reconLib!=NULL && glob->reconStruct!=NULL && p->reconFrameFinishedFn!=NULL){
    (*p->reconFrameFinishedFn)(glob->reconStruct,glob->arrays->dmCommand,p->pxlCentInputError);
  }
  pthread_mutex_unlock(&glob->reconMutex);
  if(p->pxlCentInputError==0){
    //we send the dmCommand here, because sendActuators can alter this depending on the bleed gain... and we don't want that to be seen - this is the result of the reconstruction process only...
    //xxxMoveToCorrectPlace;
    circAdd(glob->rtcMirrorBuf,p->dmCommand,p->timestamp,p->thisiter);//actsSent);
  }
  resetRecon=0;
  if(userActs==NULL || addUserActs!=0){
    //Either no user actuators specified, or we're adding user actuators.
    //In either case, we want to put latestDmCommand into actsSent.
    //if(p->kalmanUsed==1 && p->usingDMC==1){//send p->Xpred to the dmc...
    //  //do nothing here - send p->Xpred below...
    //}else{//otherwise, we are sending actuators rather than phase.
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
    
    
    /*
      if(bleed!=0.){//compute the bleed value
      for(i=0; i<nacts; i++){
      bleedVal+=dmCommand[i];
      }
      bleedVal*=bleed;
      bleedVal-=midrange;//Note - really midrange times bleed over nact... maybe this should be replaced by v0 - to allow a midrange value per actuator?
      for(i=0; i<nacts; i++)
      dmCommand[i]-=bleedVal;
      }
      //bleedVal-=0.5;//do proper rounding...
      memcpy(latestDmCommand,dmCommand,sizeof(float)*nacts);
    */
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
    //Now do clipping and integer conversion.
    //This should now be done in the dm library.
    /*
      for(i=0; i<nacts; i++){
      //dmCommand[i]-=bleedVal;
      intDMCommand=(int)(dmCommand[i]+0.5);
      actsSent[i]=(unsigned short)intDMCommand;
      if(intDMCommand<actMin){
      nclipped++;
      actsSent[i]=actMin;
      }
      if(intDMCommand>actMax){
      nclipped++;
      actsSent[i]=actMax;
      }
      }
      //memcpy(latestDmCommand,dmCommand,sizeof(float)*nacts);
      p->nclipped=nclipped;
    */
    //}
  }else{//userActs is specified, and we're replacing, not adding.
    //memcpy(actsSent,userActs,sizeof(unsigned short)*nacts);
    memcpy(dmCommand,userActs,sizeof(float)*nacts);
    /*for(i=0; i<nacts; i++){
      actsSent[i]=(unsigned short)(userActs[i]);
      if(actsSent[i]<actMin){
      nclipped++;
      actsSent[i]=actMin;
      }
      if(actsSent[i]>actMax){
      nclipped++;
      actsSent[i]=actMax;
      }
      }
      p->nclipped=nclipped;*/
    //should we do anything about latestDMCommand - eg set to midrange or v0?  Or to userActs?  
    //memcpy(latestDmCommand,v0,sizeof(float)*nacts);
    //It is necessary in this case to let the recon library know that the actuator values aren't being used, so that it can reset itself...
    resetRecon=1;
  }
  if(p->closeLoop){
    //The thing to do now is:
    //if kalmanUsed==1 and usingDMC==1 and userActs==NULL, do nothing.
    //Otherwise, send the actuators.
    /*
      if(p->kalmanUsed==1 && p->usingDMC==1){
      if(userActs==NULL){
      //send p->Xpred to the DMC.
      printf("todo - send p->Xpred\n");
      }else{//user actuators... being used.
      //But this is a problem - the DMC expects data size kalmanPhaseSize*3 but userActs is size nacts. How do we get round this?
      if(addUserActs!=0){
      //add actautors to Xpred
      printf("todo - send userActs+p->Xpred\n");
      }else{
      //send userActs...
      printf("todo - send userActs\n");
      }
      //add userActs send the p->Xpred to the DMC - however, the DMC will be expecting data of size kalmanPhaseSize*3, but userActs will be of size nacts... how do we get around this?
      }
      }else{
      if(p->usingDMC==1){
      //send actuators to the DMC
      printf("todo - send acts to DMC\n");
      }else{*/
    //send actuators direct to the mirror.
    pthread_mutex_lock(&glob->mirrorMutex);
    if(p->mirrorHandle!=NULL && glob->mirrorLib!=NULL && p->mirrorSendFn!=NULL)
      //(*p->mirrorSendFn)(p->mirrorHandle,nacts,actsSent,p->thisiter);
      p->nclipped=(*p->mirrorSendFn)(p->mirrorHandle,nacts,dmCommand,p->thisiter,p->timestamp,p->pxlCentInputError);
    pthread_mutex_unlock(&glob->mirrorMutex);
    /*
      }
      }*/
  }else{
    //todo - maybe set this to midrange or v0?
    //Npte - we will copy into latestDMCommand twice here.  But doesn't matter too much since we're not closing the loop, so the additional latency isn't important.
    //memcpy(latestDmCommand,v0,sizeof(float)*nacts);
    //It is necessary in this case to let the recon library know that the actuator values aren't being used, so that it can reset itself...
    resetRecon=1;
  }
  if(resetRecon){
    pthread_mutex_lock(&glob->reconMutex);
    if(glob->reconLib!=NULL && glob->reconStruct!=NULL && p->reconOpenLoopFn!=NULL){
      (*p->reconOpenLoopFn)(glob->reconStruct);
    }
    pthread_mutex_unlock(&glob->reconMutex);
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
      //memcpy(dmCommand,p->dmCommandSave,sizeof(float)*nacts);
      //cblas_saxpy(nacts,1.,p->actsRequired,1,dmCommand,1);//dmCommand=dmCommand + actsRequired.
      //glob->figureFrame[1]=glob->figureFrame[0];//copy into the second buffer...
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
	pthread_mutex_lock(p->mirrorMutex);
	if(p->mirrorHandle!=NULL && p->mirrorLib!=NULL && p->mirrorSendFn!=NULL)
	  p->nclipped=(*p->mirrorSendFn)(p->mirrorHandle,nacts,dmCommand,p->thisiter,p->timestamp,p->pxlCentInputError);
	pthread_mutex_unlock(p->mirrorMutex);
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
  snprintf(glob->statusBuf,STATUSBUFSIZE,"%d+1 threads\nClipped: %d\nIteration: %d/%d\nMax time %gs at iter %d\nFrame time %gs (%gHz)\nFrame numbers %s\n%s\n%s\n%s\n%s\n%s\n%s\nFS:%u\n%s",glob->nthreads,glob->nclipped,glob->thisiter,glob->niters,glob->maxtime,glob->maxtimeiter,glob->frameTime,1/glob->frameTime,tmp,paused==0?"Running...":"Paused...",glob->camHandle==NULL?"No cam":"Cam open",glob->camFraming?"Cam framing":"Cam not framing",glob->mirrorHandle==NULL?"No mirror":"Mirror connected",glob->centHandle==NULL?"No external centroider":"External centroider",glob->reconLib==NULL?"No reconstructor":"Reconstructor in use",glob->figureFrame[1],closeLoop?"Loop closed":"Loop open");
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



//#define HDRSIZE 64*72

/**
   Called when a buffer swap is required.  Reads the new buffer.
   Only the first thread needs to do this.
*/

int updateBufferIndex(threadStruct *threadInfo){//,int updateIndex){
  //Assumes the buffer is an array, with header then data.  Header contains:
  //name (31 bytes), type(1), startaddr(4), nbytes(4), ndim(4), shape(24), lcomment(4).  Here, we find name, check that type and nbytes match what we expect, and move the pointer to startaddr (which is index in bytes from start of array).

  //This is called only by the first thread for each camera.  The
  //first thread overall has updateIndex set.
  //BUFFERVARIABLEINDX i;
  int j;//,k,offset;
  char *buf=threadInfo->globals->buffer[threadInfo->globals->curBuf]->buf;
  int *indx;
  //int nbytes;
  int err=0;
  //short *tmps;
  //float *tmpf;
  //struct timeval t1;
  //double timestamp;
  //infoStruct *info=threadInfo->info;
  globalStruct *globals=threadInfo->globals;
  dprintf("updating buffer index\n");//insz=%d\n",updateIndex);
  indx=globals->bufferHeaderIndex;

  //if(updateIndex){//only the first thread needs to do this...
  memset(indx,-1,sizeof(int)*NBUFFERVARIABLES);
  //First get the indexes for each variable...
  j=0;
  while(j<NHDR && buf[j*31]!='\0'){
    if(strncmp(&buf[j*31],"ncam",31)==0){
      indx[NCAM]=j;
    }else if(strncmp(&buf[j*31],"nacts",31)==0){
      indx[NACTS]=j;
    }else if(strncmp(&buf[j*31],"nsubx",31)==0){
      indx[NSUBX]=j;
    }else if(strncmp(&buf[j*31],"nsuby",31)==0){
      indx[NSUBY]=j;
    }else if(strncmp(&buf[j*31],"npxlx",31)==0){
      indx[NPXLX]=j;
    }else if(strncmp(&buf[j*31],"npxly",31)==0){
      indx[NPXLY]=j;
    }else if(strncmp(&buf[j*31],"refCentroids",31)==0){
      indx[REFCENTROIDS]=j;
    }else if(strncmp(&buf[j*31],"subapLocation",31)==0){
      indx[SUBAPLOCATION]=j;
    }else if(strncmp(&buf[j*31],"bgImage",31)==0){
      indx[BGIMAGE]=j;
    }else if(strncmp(&buf[j*31],"darkNoise",31)==0){
      indx[DARKNOISE]=j;
    }else if(strncmp(&buf[j*31],"flatField",31)==0){
      indx[FLATFIELD]=j;
    }else if(strncmp(&buf[j*31],"thresholdAlgorithm",31)==0){
      indx[THRESHOLDALGORITHM]=j;
    }else if(strncmp(&buf[j*31],"thresholdValue",31)==0){
      indx[THRESHOLDVALUE]=j;
    }else if(strncmp(&buf[j*31],"powerFactor",31)==0){
      indx[POWERFACTOR]=j;
    }else if(strncmp(&buf[j*31],"centroidWeighting",31)==0){
      indx[CENTROIDWEIGHTING]=j;
    }else if(strncmp(&buf[j*31],"windowMode",31)==0){
      indx[WINDOWMODE]=j;
      /*      }else if(strncmp(&buf[j*31],"kalmanPhaseSize",31)==0){
	      indx[KALMANPHASESIZE]=j;
	      }else if(strncmp(&buf[j*31],"kalmanUsed",31)==0){
	      indx[KALMANUSED]=j;
	      }else if(strncmp(&buf[j*31],"kalmanReset",31)==0){
	      indx[KALMANRESET]=j;
	      }else if(strncmp(&buf[j*31],"usingDMC",31)==0){
	      indx[USINGDMC]=j;
	      }else if(strncmp(&buf[j*31],"kalmanHinfT",31)==0){
	      indx[KALMANHINFT]=j;
	      }else if(strncmp(&buf[j*31],"kalmanHinfDM",31)==0){
	      indx[KALMANHINFDM]=j;
	      }else if(strncmp(&buf[j*31],"kalmanAtur",31)==0){
	      indx[KALMANATUR]=j;
	      }else if(strncmp(&buf[j*31],"kalmanInvN",31)==0){
	      indx[KALMANINVN]=j;*/
    }else if(strncmp(&buf[j*31],"subapFlag",31)==0){
      indx[SUBAPFLAG]=j;
    }else if(strncmp(&buf[j*31],"go",31)==0){
      indx[GO]=j;
    }else if(strncmp(&buf[j*31],"pxlCnt",31)==0){
      indx[PXLCNT]=j;
    }else if(strncmp(&buf[j*31],"centroidMode",31)==0){
      indx[CENTROIDMODE]=j;
      //      }else if(strncmp(&buf[j*31],"gainReconmxT",31)==0){
      //indx[GAINRECONMXT]=j;
      //}else if(strncmp(&buf[j*31],"reconstructMode",31)==0){
      //indx[RECONSTRUCTMODE]=j;
      //}else if(strncmp(&buf[j*31],"gainE",31)==0){
      //indx[GAINE]=j;
      //}else if(strncmp(&buf[j*31],"v0",31)==0){
      //indx[V0]=j;
      //}else if(strncmp(&buf[j*31],"bleedGain",31)==0){
      //indx[BLEEDGAIN]=j;
      //}else if(strncmp(&buf[j*31],"midRangeValue",31)==0){
      //indx[MIDRANGE]=j;
      //}else if(strncmp(&buf[j*31],"actMax",31)==0){
      //indx[ACTMAX]=j;
    }else if(strncmp(&buf[j*31],"pause",31)==0){
      indx[PAUSE]=j;
    }else if(strncmp(&buf[j*31],"printTime",31)==0){
      indx[PRINTTIME]=j;
    }else if(strncmp(&buf[j*31],"ncamThreads",31)==0){
      indx[NCAMTHREADS]=j;
    }else if(strncmp(&buf[j*31],"switchRequested",31)==0){
      indx[SWITCHREQUESTED]=j;
    }else if(strncmp(&buf[j*31],"actuators",31)==0){
      indx[USERACTS]=j;
    }else if(strncmp(&buf[j*31],"fakeCCDImage",31)==0){
      indx[FAKECCDIMAGE]=j;
    }else if(strncmp(&buf[j*31],"threadAffinity",31)==0){
      indx[THREADAFFINITY]=j;
    }else if(strncmp(&buf[j*31],"threadPriority",31)==0){
      indx[THREADPRIORITY]=j;
    }else if(strncmp(&buf[j*31],"delay",31)==0){
      indx[DELAY]=j;
    }else if(strncmp(&buf[j*31],"maxClipped",31)==0){
      indx[MAXCLIPPED]=j;
    }else if(strncmp(&buf[j*31],"clearErrors",31)==0){
      indx[CLEARERRORS]=j;
      /*}else if(strncmp(&buf[j*31],"openCameras",31)==0){
	indx[OPENCAMERAS]=j;
	}else if(strncmp(&buf[j*31],"closeCameras",31)==0){
	indx[CLOSECAMERAS]=j;
	}else if(strncmp(&buf[j*31],"startCamerasFraming",31)==0){
	indx[STARTCAMERASFRAMING]=j;
	}else if(strncmp(&buf[j*31],"stopCamerasFraming",31)==0){
	indx[STOPCAMERASFRAMING]=j;*/
    }else if(strncmp(&buf[j*31],"camerasOpen",31)==0){
      indx[CAMERASOPEN]=j;
    }else if(strncmp(&buf[j*31],"camerasFraming",31)==0){
      indx[CAMERASFRAMING]=j;
    }else if(strncmp(&buf[j*31],"cameraParams",31)==0){
      indx[CAMERAPARAMS]=j;
    }else if(strncmp(&buf[j*31],"cameraName",31)==0){
      indx[CAMERANAME]=j;
    }else if(strncmp(&buf[j*31],"mirrorOpen",31)==0){
      indx[MIRROROPEN]=j;
    }else if(strncmp(&buf[j*31],"mirrorName",31)==0){
      indx[MIRRORNAME]=j;
    }else if(strncmp(&buf[j*31],"frameno",31)==0){
      indx[FRAMENO]=j;
    }else if(strncmp(&buf[j*31],"switchTime",31)==0){
      indx[SWITCHTIME]=j;
    }else if(strncmp(&buf[j*31],"adaptiveWinGain",31)==0){
      indx[ADAPTIVEWINGAIN]=j;
    }else if(strncmp(&buf[j*31],"correlationThresholdType",31)==0){
      indx[CORRELATIONTHRESHOLDTYPE]=j;
    }else if(strncmp(&buf[j*31],"correlationThreshold",31)==0){
      indx[CORRELATIONTHRESHOLD]=j;
    }else if(strncmp(&buf[j*31],"fftCorrelationPattern",31)==0){
      indx[FFTCORRELATIONPATTERN]=j;
    }else if(strncmp(&buf[j*31],"nsubapsTogether",31)==0){
      indx[NSUBAPSTOGETHER]=j;
    }else if(strncmp(&buf[j*31],"nsteps",31)==0){
      indx[NSTEPS]=j;
      //}else if(strncmp(&buf[j*31],"actMin",31)==0){
      //indx[ACTMIN]=j;
    }else if(strncmp(&buf[j*31],"closeLoop",31)==0){
      indx[CLOSELOOP]=j;
    }else if(strncmp(&buf[j*31],"mirrorParams",31)==0){
      indx[MIRRORPARAMS]=j;
    }else if(strncmp(&buf[j*31],"addActuators",31)==0){
      indx[ADDUSERACTS]=j;
    }else if(strncmp(&buf[j*31],"actSequence",31)==0){
      indx[USERACTSEQ]=j;
    }else if(strncmp(&buf[j*31],"recordCents",31)==0){
      indx[RECORDCENTS]=j;
    }else if(strncmp(&buf[j*31],"pxlWeight",31)==0){
      indx[PXLWEIGHT]=j;
    }else if(strncmp(&buf[j*31],"averageImg",31)==0){
      indx[AVERAGEIMG]=j;
    }else if(strncmp(&buf[j*31],"centroidersOpen",31)==0){
      indx[CENTOPEN]=j;
    }else if(strncmp(&buf[j*31],"centroidersFraming",31)==0){
      indx[CENTFRAMING]=j;
    }else if(strncmp(&buf[j*31],"centroidersParams",31)==0){
      indx[CENTPARAMS]=j;
    }else if(strncmp(&buf[j*31],"centroidersName",31)==0){
      indx[CENTNAME]=j;
    }else if(strncmp(&buf[j*31],"actuatorMask",31)==0){
      indx[USERACTSMASK]=j;
    }else if(strncmp(&buf[j*31],"dmDescription",31)==0){
      //do nothing...
    }else if(strncmp(&buf[j*31],"averageCent",31)==0){
      indx[AVERAGECENT]=j;
    }else if(strncmp(&buf[j*31],"calmult",31)==0){
      indx[CALMULT]=j;
    }else if(strncmp(&buf[j*31],"calsub",31)==0){
      indx[CALSUB]=j;
    }else if(strncmp(&buf[j*31],"calthr",31)==0){
      indx[CALTHR]=j;
    }else if(strncmp(&buf[j*31],"centCalData",31)==0){
      indx[CENTCALDATA]=j;
    }else if(strncmp(&buf[j*31],"centCalBounds",31)==0){
      indx[CENTCALBOUNDS]=j;
    }else if(strncmp(&buf[j*31],"centCalSteps",31)==0){
      indx[CENTCALSTEPS]=j;
    }else if(strncmp(&buf[j*31],"figureOpen",31)==0){
      indx[FIGUREOPEN]=j;
    }else if(strncmp(&buf[j*31],"figureName",31)==0){
      indx[FIGURENAME]=j;
    }else if(strncmp(&buf[j*31],"figureParams",31)==0){
      indx[FIGUREPARAMS]=j;
    }else if(strncmp(&buf[j*31],"reconName",31)==0){
      indx[RECONNAME]=j;
    }else if(strncmp(&buf[j*31],"fluxThreshold",31)==0){
      indx[FLUXTHRESHOLD]=j;
    }else if(strncmp(&buf[j*31],"printUnused",31)==0){
      indx[PRINTUNUSED]=j;
      }else if(strncmp(&buf[j*31],"useBrightest",31)==0){
	indx[USEBRIGHTEST]=j;
      }else if(strncmp(&buf[j*31],"figureGain",31)==0){
	indx[FIGUREGAIN]=j;
      }else if(strncmp(&buf[j*31],"reconlibOpen",31)==0){
	indx[RECONLIBOPEN]=j;
      }else if(strncmp(&buf[j*31],"maxAdapOffset",31)==0){
	indx[MAXADAPOFFSET]=j;
      }else if(strncmp(&buf[j*31],"version",31)==0){
	indx[VERSION]=j;
      }else if(strncmp(&buf[j*31],"currentErrors",31)==0){
	indx[CURRENTERRORS]=j;
      }else if(strncmp(&buf[j*31],"reconParams",31)==0){
	indx[RECONPARAMS]=j;
      }else if(strncmp(&buf[j*31],"adaptiveWinGroup",31)==0){
	indx[ADAPTIVEGROUP]=j;
      }else if(strncmp(&buf[j*31],"comment",31)==0){
	//do nothing...
      }else{
	if(strncmp(&buf[j*31],"rmx",31)==0 || strncmp(&buf[j*31],"E",31)==0 || strncmp(&buf[j*31],"gain",31)==0){
	  //do nothing
	}else if(globals->printUnused){
	  printf("Ignoring: %s (used by .so interfaces?)\n",&buf[j*31]);
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
    }
  }
  //if(err)
  return err;
}


/**
   Called when a buffer swap is required.  Reads the new buffer.
 */

int updateBuffer(threadStruct *threadInfo,int updateIndex){
  //Assumes the buffer is an array, with header then data.  Header contains:
  //name (31 bytes), type(1), startaddr(4), nbytes(4), ndim(4), shape(24), lcomment(4).  Here, we find name, check that type and nbytes match what we expect, and move the pointer to startaddr (which is index in bytes from start of array).

  //This is called only by the first thread for each camera.  The
  //first thread overall has updateIndex set.
  BUFFERVARIABLEINDX i;
  int j;//,k,offset;
  int m;
  char *buf=threadInfo->globals->buffer[threadInfo->globals->curBuf]->buf;
  int *indx;
  int nbytes;
  int err=0;
  //short *tmps;
  //float *tmpf;
  struct timeval t1;
  double timestamp;
  infoStruct *info=threadInfo->info;
  globalStruct *globals=threadInfo->globals;
  int nusedSubaps;
  dprintf("updating buffer insz=%d\n",updateIndex);

  indx=globals->bufferHeaderIndex;

  //first get ncam... (for checking array sizes etc)
  i=NCAM;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==4){
    if(info->ncam!=*((int*)(buf+START[indx[i]]))){
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
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==4){
	info->nacts=*((int*)(buf+START[indx[i]]));
  }else{
    printf("nacts error\n");
    err=NACTS;
  }
  i=NSUBX;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==4*info->ncam){
    info->nsubxList=(int*)(buf+START[indx[i]]);
    info->nsubx=info->nsubxList[info->cam];
  }else{
    printf("nsubx error\n");
    err=NSUBX;
  }
  i=NSUBY;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==4*info->ncam){
	info->nsubyList=(int*)(buf+START[indx[i]]);
	info->nsuby=info->nsubyList[info->cam];
  }else{
    printf("nsuby error\n");
    err=NSUBY;
  }
  i=NPXLX;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==4*info->ncam){
    info->npxlxList=(int*)(buf+START[indx[i]]);
    info->npxlx=info->npxlxList[info->cam];
  }else{
    printf("npxlx error\n");
    err=NPXLX;
  }
  i=NPXLY;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==4*info->ncam){
    info->npxlyList=(int*)(buf+START[indx[i]]);
    info->npxly=info->npxlyList[info->cam];
  }else{
    printf("npxly error\n");
    err=NPXLY;
  }
  
  //compute number of subaps/pixels up to this point...
  info->subCumIndx=0;
  info->npxlCum=0;
  for(j=0;j<info->cam; j++){
    info->subCumIndx+=info->nsubxList[j]*info->nsubyList[j];
    info->npxlCum+=info->npxlxList[j]*info->npxlyList[j];
  }
  //and the total number
  info->nsubaps=info->subCumIndx;
  info->totPxls=info->npxlCum;
  for(j=info->cam; j<info->ncam; j++){
    info->nsubaps+=info->nsubxList[j]*info->nsubyList[j];
    info->totPxls+=info->npxlxList[j]*info->npxlyList[j];
  }
  /*
  //now allocate an array large enough to hold all pixels (note - this may not be used, depending on the camera library...)
  if(updateIndex){
    if(globals->pxlbufsSize!=sizeof(short)*info->totPxls){
      globals->pxlbufsSize=sizeof(short)*info->totPxls;
      tmps=realloc(globals->pxlbufs,globals->pxlbufsSize);
      if(tmps==NULL){
	if(globals->pxlbufs!=NULL)
	  free(globals->pxlbufs);
	printf("pxlbuf malloc error.\n");
	err=-3;
	globals->pxlbufsSize=0;
      }
      globals->pxlbufs=tmps;
      memset(globals->pxlbufs,0,globals->pxlbufsSize);
    }
    }*/
  //index into the pxlbufs array to get pixel buffer for this camera.
  //This is now done in swaparrays...
  //if(globals->pxlbufs==NULL){
  //  err=-3;
  //}//else{
  //info->pxlbuf=&globals->pxlbufs[info->npxlCum];
  //}
  i=SUBAPFLAG;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)*info->nsubaps){
    info->subapFlagArr=(int*)(buf+START[indx[i]]);
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
  //printf("totCents: %d\n",info->totCents);
  //info->totCents+=cblas_sasum(info->nsubaps-info->subCumIndx,&(info->subapFlagArr[info->subCumIndx]),1);

  /*if(updateIndex){
    //work out number of centroids per camera...
    offset=0;
    //printf("Got centroids:");
    for(j=0; j<info->ncam; j++){
      globals->ncentsList[j]=0;
      for(k=0; k<info->nsubxList[j]*info->nsubyList[j]; k++){
	globals->ncentsList[j]+=info->subapFlagArr[k+offset];
      }
      offset+=k;
      globals->ncentsList[j]*=2;
      //printf(" %d",globals->ncentsList[j]);
      }*/
    //printf("\n");

    //allocate array to hold all pixels from WPU centroider... note this may not be used depending on centroider library.
    /*
    if(globals->centbufsSize!=sizeof(float)*info->totCents){
      globals->centbufsSize=sizeof(float)*info->totCents;
      tmpf=realloc(globals->centbufs,globals->centbufsSize);
      if(tmpf==NULL){
	if(globals->centbufs!=NULL)
	  free(globals->centbufs);
	printf("centbuf malloc error.\n");
	err=-3;
	globals->centbufsSize=0;
      }
      globals->centbufs=tmpf;
      memset(globals->centbufs,0,globals->centbufsSize);
    }*/
  //}
  //index into the centbufs array to get centroid buffer for this camera.
  /*
  if(globals->centbufs!=NULL){
    info->centbuf=&globals->centbufs[info->centCumIndx];
  }else{
    err=-3;
    }*/

  //and then get the rest
  /*
  i=KALMANPHASESIZE;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->kalmanPhaseSize=*((int*)(buf+START[indx[i]]));
  }else{
    printf("kalmanPhaseSize error\n");
    err=KALMANPHASESIZE;
  }
  i=KALMANUSED;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->kalmanUsed=*((int*)(buf+START[indx[i]]));
  }else{
    printf("kalmanUsed error\n");
    err=KALMANUSED;
    }*/
  /*
  i=RECONSTRUCTMODE;
  nbytes=NBYTES[indx[i]];
  if(buf[NHDR*31+indx[i]]=='s'){
    if(strncmp(buf+START[indx[i]],"simple",nbytes)==0){
      info->reconMode=RECONMODE_SIMPLE;
    }else if(strncmp(buf+START[indx[i]],"truth",nbytes)==0){
      info->reconMode=RECONMODE_TRUTH;
    }else if(strncmp(buf+START[indx[i]],"open",nbytes)==0){
      info->reconMode=RECONMODE_OPEN;
    }else if(strncmp(buf+START[indx[i]],"offset",nbytes)==0){
      info->reconMode=RECONMODE_OFFSET;
    }else{
      printf("reconstructMode not interpreted, assuming simple\n");
      info->reconMode=RECONMODE_SIMPLE;
    }
  }else{
    printf("reconstructMode error\n");
    err=RECONSTRUCTMODE;
  }
  */

  i=REFCENTROIDS;
  nbytes=NBYTES[indx[i]];
  if(nbytes==0){
    info->refCents=NULL;
  }else if(buf[NHDR*31+indx[i]]=='f' && nbytes==sizeof(float)*info->totCents){
    info->refCents=(float*)(buf+START[indx[i]]);
  }else{
    printf("refCentroids error\n");
    err=REFCENTROIDS;
  }
  /*
  i=GAINRECONMXT;
  if(buf[NHDR*31+indx[i]]=='f' && NBYTES[indx[i]]/sizeof(float)==info->totCents*info->nacts){
    info->rmxT=(float*)(buf+START[indx[i]]);
  }else{
    printf("gainReconmxT error %c %d %d %d\n",buf[NHDR*31+indx[i]],NBYTES[indx[i]],info->totCents,info->nacts);
    err=GAINRECONMXT;
  }
  i=GAINE;
  if(buf[NHDR*31+indx[i]]=='f' && NBYTES[indx[i]]==sizeof(float)*info->nacts*info->nacts){
    info->gainE=(float*)(buf+START[indx[i]]);
  }else{
    printf("gainE error\n");
    err=GAINE;
  }
  i=V0;
  if(buf[NHDR*31+indx[i]]=='f' && NBYTES[indx[i]]==sizeof(float)*info->nacts){
    info->v0=(float*)(buf+START[indx[i]]);
  }else{
    printf("v0 error\n");
    err=V0;
  }
  i=BLEEDGAIN;
  if(buf[NHDR*31+indx[i]]=='f' && NBYTES[indx[i]]==sizeof(float)){
    info->bleedGainOverNact=(*((float*)(buf+START[indx[i]])))/info->nacts;
  }else{
    printf("bleedGain error\n");
    err=BLEEDGAIN;
  }
  i=MIDRANGE;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->midRangeTimesBleed=(*((int*)(buf+START[indx[i]])))*info->nacts*info->bleedGainOverNact;
  }else{
    printf("midrange error\n");
    err=MIDRANGE;
  }
  */
  /*
  i=ACTMAX;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->actMax=*((int*)(buf+START[indx[i]]));
  }else{
    printf("actMax error\n");
    err=ACTMAX;
  }
  i=ACTMIN;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->actMin=*((int*)(buf+START[indx[i]]));
  }else{
    printf("actMin error\n");
    err=ACTMIN;
  }
  */
  i=CLOSELOOP;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->closeLoop=*((int*)(buf+START[indx[i]]));
  }else{
    printf("closeLoop error\n");
    err=CLOSELOOP;
  }
  i=PAUSE;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->pause=*((int*)(buf+START[indx[i]]));
    globals->ppause=((int*)(buf+START[indx[i]]));
  }else{
    printf("pause error\n");
    globals->ppause=NULL;
    err=i;
  }
  i=PRINTTIME;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->printTime=*((int*)(buf+START[indx[i]]));
    globals->maxtime=0;
  }else{
    printf("printTime error\n");
    err=i;
  }
  
  i=SUBAPLOCATION;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==info->nsubaps*6*sizeof(int)){
    info->realSubapLocation=(int*)(buf+START[indx[i]]);
  }else{
    printf("subapLocation error\n");
    err=i;
  }
  i=PXLCNT;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==info->nsubaps*sizeof(int)){
    info->pxlCnt=(int*)(buf+START[indx[i]]);
  }else{
    printf("pxlCnt error\n");
    err=i;
  }
  i=BGIMAGE;
  nbytes=NBYTES[indx[i]];
  if(nbytes==0)
    info->background=NULL;
  else if(buf[NHDR*31+indx[i]]=='f' && nbytes/sizeof(float)==info->totPxls){
    info->backgroundArr=(float*)(buf+START[indx[i]]);
    info->background=&(info->backgroundArr[info->npxlCum]);
  }else{
    printf("bgImage error\n");
    err=i;
  }
  i=DARKNOISE;
  nbytes=NBYTES[indx[i]];
  if(nbytes==0)
    info->darkNoise=NULL;
  else if(buf[NHDR*31+indx[i]]=='f' && nbytes/sizeof(float)==info->totPxls){
    info->darkNoiseArr=(float*)(buf+START[indx[i]]);
    info->darkNoise=&(info->darkNoiseArr[info->npxlCum]);
  }else{
    printf("darkNoise error\n");
    err=i;
  }
  i=FLATFIELD;
  nbytes=NBYTES[indx[i]];
  if(nbytes==0)
    info->flatField=NULL;
  else if(buf[NHDR*31+indx[i]]=='f' && nbytes/sizeof(float)==info->totPxls){
    info->flatFieldArr=(float*)(buf+START[indx[i]]);
    info->flatField=&(info->flatFieldArr[info->npxlCum]);
  }else{
    printf("flatField error\n");
    err=i;
  }
  i=THRESHOLDALGORITHM;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==4){
    info->thresholdAlgo=*((int*)(buf+START[indx[i]]));
  }else{
    printf("thresholdAlgorithm error\n");
    err=i;
  }
  i=THRESHOLDVALUE;
  nbytes=NBYTES[indx[i]];
  if((buf[NHDR*31+indx[i]])=='f'){
    if(nbytes==sizeof(float)){
      info->threshold=*((float*)(buf+START[indx[i]]));
      info->thresholdArr=NULL;
      info->thresholdArrType=0;
    }else if(nbytes==sizeof(float)*info->nsubaps){
      info->threshold=0;
      info->thresholdArr=(float*)(buf+START[indx[i]]);
      info->thresholdArrType=0;
    }else if(nbytes==sizeof(float)*info->totPxls){
      info->threshold=0;
      info->thresholdArr=(float*)(buf+START[indx[i]]);
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
  if(buf[NHDR*31+indx[i]]=='f' && NBYTES[indx[i]]==4){
    info->powerFactor=*((float*)(buf+START[indx[i]]));
  }else{
    err=i;
    printf("powerFactor error\n");
  }
  i=CENTROIDWEIGHTING;
  nbytes=NBYTES[indx[i]];
  if(nbytes==0)
    info->centWeighting=NULL;
  else if(buf[NHDR*31+indx[i]]=='f' && nbytes==4){
    info->centWeighting=((float*)(buf+START[indx[i]]));
  }else{
    err=i;
    printf("centWeighting error\n");
  }
  i=WINDOWMODE;
  nbytes=NBYTES[indx[i]];
  info->resetAdaptiveWindows=0;
  if(nbytes!=0 && buf[NHDR*31+indx[i]]=='s'){
    if(strncmp(buf+START[indx[i]],"basic",nbytes)==0){
      info->windowMode=WINDOWMODE_BASIC;
    }else if(strncmp(buf+START[indx[i]],"adaptive",nbytes)==0){
      if(info->windowMode!=WINDOWMODE_ADAPTIVE){
	info->windowMode=WINDOWMODE_ADAPTIVE;
	//if(updateIndex)
	info->resetAdaptiveWindows=1;//all threads can set this, but only the first overall will use it...
      }
    }else if(strncmp(buf+START[indx[i]],"global",nbytes)==0){
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
  i=GO;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==4){
    info->go=*((int*)(buf+START[indx[i]]));
  }else{
    err=i;
    printf("go error\n");
  }
  i=CENTROIDMODE;
  nbytes=NBYTES[indx[i]];
  if(nbytes!=0 && buf[NHDR*31+indx[i]]=='s'){
    //info->useWPU=0;
    if(strncmp(buf+START[indx[i]],"WPU",nbytes)==0){
      //info->useWPU=1;
      info->centroidMode=CENTROIDMODE_WPU;
    }else if(strncmp(buf+START[indx[i]],"CoG",nbytes)==0){
      info->centroidMode=CENTROIDMODE_COG;
    }else if(strncmp(buf+START[indx[i]],"Gaussian",nbytes)==0){
      info->centroidMode=CENTROIDMODE_GAUSSIAN;
    }else if(strncmp(buf+START[indx[i]],"Correlation CoG",nbytes)==0){
      info->centroidMode=CENTROIDMODE_CORRELATIONCOG;
    }else if(strncmp(buf+START[indx[i]],"Correlation gaussian",nbytes)==0){
      info->centroidMode=CENTROIDMODE_CORRELATIONGAUSSIAN;
    }else{
      info->centroidMode=CENTROIDMODE_ERROR;
      printf("Unrecognised centroidMode\n");
    }
  }else{
    err=i;
    printf("centroidMode error\n");
  }
  /*i=KALMANRESET;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==4){
    info->kalmanReset=*((int*)(buf+START[indx[i]]));
  }else{
    err=i;
    printf("kalmanReset error\n");
  }
  i=USINGDMC;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==4){
    info->usingDMC=*((int*)(buf+START[indx[i]]));
  }else{
    err=i;
    printf("usingDMC error\n");
  }
  i=KALMANHINFT;
  if(info->kalmanUsed==1){
    if(buf[NHDR*31+indx[i]]=='f' && NBYTES[indx[i]]==info->kalmanPhaseSize*3*info->totCents*sizeof(float)){
      info->kalmanHinfT=((float*)(buf+START[indx[i]]));
    }else{
      err=i;
      printf("kalmanHinfT error\n");
    }
  }
  i=KALMANHINFDM;
  if(info->kalmanUsed==1){
    if(buf[NHDR*31+indx[i]]=='f' && NBYTES[indx[i]]==info->kalmanPhaseSize*info->kalmanPhaseSize*3*sizeof(float)){
      info->kalmanHinfDM=((float*)(buf+START[indx[i]]));
    }else{
      err=i;
      printf("kalmanHinfDM error\n");
    }
  }
  i=KALMANATUR;
    if(info->kalmanUsed==1){
      if(buf[NHDR*31+indx[i]]=='f' && NBYTES[indx[i]]==info->kalmanPhaseSize*info->kalmanPhaseSize*sizeof(float)){
	info->kalmanAtur=((float*)(buf+START[indx[i]]));
      }else{
	err=i;
	printf("kalmanAtur error\n");
      }
    }
  i=KALMANINVN;
  if(info->kalmanUsed==1){
    if(buf[NHDR*31+indx[i]]=='f' && NBYTES[indx[i]]==info->nacts*info->kalmanPhaseSize*sizeof(float)){
      info->kalmanInvN=((float*)(buf+START[indx[i]]));
    }else{
      err=i;
      printf("kalmanInvN error\n");
    }
  }*/
  i=USERACTS;
  if(NBYTES[indx[i]]==0){
    info->userActs=NULL;
    info->userActSeqLen=0;
  }else if(buf[NHDR*31+indx[i]]=='f' && NBYTES[indx[i]]%(sizeof(float)*info->nacts)==0){
    info->userActs=((float*)(buf+START[indx[i]]));
    //get the number of actuator sequences...
    info->userActSeqLen=NBYTES[indx[i]]/(sizeof(float)*info->nacts);
  }else{ 
    printf("userActs/actuators error %c %d %d\n",buf[NHDR*31+indx[i]],NBYTES[indx[i]],(int)sizeof(float)*info->nacts);
    err=USERACTS;
  }
  i=USERACTSMASK;
  if(NBYTES[indx[i]]==0){
    info->userActsMask=NULL;
  }else if(buf[NHDR*31+indx[i]]=='f' && NBYTES[indx[i]]%(sizeof(float)*info->nacts)==0){
    info->userActsMask=((float*)(buf+START[indx[i]]));
  }else{ 
    printf("userActsMask/actuatorMask error %c %d %d\n",buf[NHDR*31+indx[i]],NBYTES[indx[i]],(int)sizeof(float)*info->nacts);
    err=USERACTSMASK;
  }

  i=FAKECCDIMAGE;
  if(NBYTES[indx[i]]==0){
    info->fakeCCDImage=NULL;
  }else if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)*info->totPxls){
    info->fakeCCDImage=((int*)(buf+START[indx[i]]));
  }else{ 
    printf("fakeCCDImage error\n");
    err=FAKECCDIMAGE;
  }
  i=THREADAFFINITY;
  nbytes=NBYTES[indx[i]];
  if(nbytes==0){
    info->threadAffinityList=NULL;
  }else if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)*(globals->nthreads+1)){
    info->threadAffinityList=(int*)(buf+START[indx[i]]);
  }else{
    printf("threadAffinity error\n");
    err=THREADAFFINITY;
  }
  i=THREADPRIORITY;
  nbytes=NBYTES[indx[i]];
  if(nbytes==0){
    info->threadPriorityList=NULL;
  }else if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)*(globals->nthreads+1)){
    info->threadPriorityList=(int*)(buf+START[indx[i]]);
  }else{
    printf("threadPriority error\n");
    err=THREADPRIORITY;
  }
  i=DELAY;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->delay=*((int*)(buf+START[indx[i]]));
  }else{
    err=i;
    printf("delay error\n");
  }
  i=MAXCLIPPED;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->maxClipped=*((int*)(buf+START[indx[i]]));
  }else{
    err=i;
    printf("maxClipped error\n");
  }
  if(updateIndex){//only one thread needs to clear errors... and only one needs to copy the priority and affinity lists for use with the preprocessing thread.
    globals->threadPriorityList=info->threadPriorityList;
    globals->threadAffinityList=info->threadAffinityList;
    i=CLEARERRORS;
    if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int) && globals->rtcErrorBuf!=NULL){
      globals->rtcErrorBuf->errFlag &= ~(*((int*)(buf+START[indx[i]])));
      *((int*)(buf+START[indx[i]]))=0;
    }else{//don't set an error flag here, since its not fatal.
      printf("clearErrors not found - not clearing\n");
    }
    i=CURRENTERRORS;//note, this is only updated when buffer is swapped.
    if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int) && globals->rtcErrorBuf!=NULL){
      *((int*)(buf+START[indx[i]]))=globals->rtcErrorBuf->errFlag;
    }else{
      printf("Unable to write current errors\n");
    }
  }
  /*i=OPENCAMERAS;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->openCameras=*((int*)(buf+START[indx[i]]));
    *((int*)(buf+START[indx[i]]))=0;
  }else{
    err=i;
    printf("openCameras error\n");
  }
  i=COSECAMERAS;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->closeCameras=*((int*)(buf+START[indx[i]]));
    *((int*)(buf+START[indx[i]]))=0;
  }else{
    err=i;
    printf("closeCameras error\n");
  }
  i=STARTCAMERASFRAMING;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->startCamerasFraming=*((int*)(buf+START[indx[i]]));
    *((int*)(buf+START[indx[i]]))=0;
  }else{
    err=i;
    printf("startCamerasFraming error\n");
  }
  i=STOPCAMERASFRAMING;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->stopCamerasFraming=*((int*)(buf+START[indx[i]]));
    *((int*)(buf+START[indx[i]]))=0;
  }else{
    err=i;
    printf("stopCamerasFraming error\n");
    }*/
  i=CAMERASOPEN;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->camerasOpen=((int*)(buf+START[indx[i]]));
    //*((int*)(buf+START[indx[i]]))=0;
  }else{
    err=i;
    printf("camerasOpen error\n");
  }
  i=CAMERASFRAMING;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->camerasFraming=((int*)(buf+START[indx[i]]));
    //*((int*)(buf+START[indx[i]]))=0;
  }else{
    err=i;
    printf("camerasFraming error\n");
  }

  i=CAMERAPARAMS;
  nbytes=NBYTES[indx[i]];
  if(nbytes==0){
    info->cameraParams=NULL;
    info->cameraParamsCnt=0;
  }else if(buf[NHDR*31+indx[i]]=='i'){
    info->cameraParams=((int*)(buf+START[indx[i]]));
    info->cameraParamsCnt=nbytes/sizeof(int);
  }else{
    err=i;
    printf("cameraParams error\n");
  }

  i=CENTOPEN;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->centroidersOpen=((int*)(buf+START[indx[i]]));
    //*((int*)(buf+START[indx[i]]))=0;
  }else{
    err=i;
    printf("centroidersOpen error\n");
  }
  i=CENTFRAMING;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->centroidersFraming=((int*)(buf+START[indx[i]]));
    //*((int*)(buf+START[indx[i]]))=0;
  }else{
    err=i;
    printf("centoidersFraming error\n");
  }

  i=CENTPARAMS;
  nbytes=NBYTES[indx[i]];
  if(nbytes==0){
    info->centroidersParams=NULL;
    info->centroidersParamsCnt=0;
  }else if(buf[NHDR*31+indx[i]]=='i'){
    info->centroidersParams=((int*)(buf+START[indx[i]]));
    info->centroidersParamsCnt=nbytes/sizeof(int);
  }else{
    err=i;
    printf("centroidersParams error\n");
  }

  i=MIRRORPARAMS;
  nbytes=NBYTES[indx[i]];
  if(nbytes==0){
    info->mirrorParams=NULL;
    info->mirrorParamsCnt=0;
  }else if(buf[NHDR*31+indx[i]]=='i'){
    info->mirrorParams=((int*)(buf+START[indx[i]]));
    info->mirrorParamsCnt=nbytes/sizeof(int);
  }else{
    err=i;
    printf("mirrorParams error\n");
  }
  i=MIRROROPEN;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->mirrorOpen=((int*)(buf+START[indx[i]]));
    //*((int*)(buf+START[indx[i]]))=0;
  }else{
    err=i;
    printf("mirrorOpen error\n");
  }
  if(updateIndex){//only one thread needs to set framenumber and time...
    i=FRAMENO;
    if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
      *((int*)(buf+START[indx[i]]))=globals->thisiter;
    }else{
      err=i;
      printf("frameno error\n");
    }
    i=SWITCHTIME;
    if(buf[NHDR*31+indx[i]]=='d' && NBYTES[indx[i]]==sizeof(double)){
      gettimeofday(&t1,NULL);
      timestamp=t1.tv_sec+t1.tv_usec*1e-6;
      *((double*)(buf+START[indx[i]]))=timestamp;
    }else{
      err=i;
      printf("switchTime error\n");
    }
  }
  i=ADAPTIVEWINGAIN;
  if(buf[NHDR*31+indx[i]]=='f' && NBYTES[indx[i]]==sizeof(float)){
    info->adaptiveWinGain=*((float*)(buf+START[indx[i]]));
  }else{
    err=i;
    printf("adaptiveWinGain error\n");
  }
  i=CORRELATIONTHRESHOLDTYPE;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->correlationThresholdType=*((int*)(buf+START[indx[i]]));
  }else{
    err=i;
    printf("correlationThresholdType error\n");
  }
  i=CORRELATIONTHRESHOLD;
  if(buf[NHDR*31+indx[i]]=='f' && NBYTES[indx[i]]==sizeof(float)){
    info->correlationThreshold=*((float*)(buf+START[indx[i]]));
  }else{
    err=i;
    printf("correlationThreshold error\n");
  }
  i=FFTCORRELATIONPATTERN;
  nbytes=NBYTES[indx[i]];
  if(nbytes==0)
    info->fftCorrelationPattern=NULL;
  else if(buf[NHDR*31+indx[i]]=='f' && nbytes/sizeof(float)==info->totPxls){
    info->fftCorrelationPatternArr=(float*)(buf+START[indx[i]]);
    info->fftCorrelationPattern=&(info->fftCorrelationPatternArr[info->npxlCum]);
  }else{
    printf("fftCorrelationPattern error\n");
    err=i;
  }
  i=NSUBAPSTOGETHER;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->nsubapsTogether=*((int*)(buf+START[indx[i]]));
  }else{
    err=i;
    printf("nsubapsTogether error\n");
  }
  i=NSTEPS;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->nsteps=*((int*)(buf+START[indx[i]]));
  }else{
    err=i;
    printf("nsteps error\n");
  }
  i=ADDUSERACTS;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->addUserActs=*((int*)(buf+START[indx[i]]));
  }else{ 
    printf("addUserActs/addActuators error\n");
    err=ADDUSERACTS;
  }
  i=USERACTSEQ;
  if(NBYTES[indx[i]]==0){
    info->userActSeq=NULL;
  }else if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)*info->userActSeqLen){
    info->userActSeq=((int*)(buf+START[indx[i]]));
  }else{ 
    printf("userActSeq/actSequence error %d %d\n",NBYTES[indx[i]],(int)(info->userActSeqLen*sizeof(int)));
    err=USERACTSEQ;
  }
  i=RECORDCENTS;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->recordCents=((int*)(buf+START[indx[i]]));
  }else{ 
    printf("recordCents error\n");
    err=RECORDCENTS;
  }
  i=PXLWEIGHT;
  nbytes=NBYTES[indx[i]];
  if(nbytes==0)
    info->pxlweight=NULL;
  else if(buf[NHDR*31+indx[i]]=='f' && nbytes==sizeof(float)*info->totPxls){
    info->pxlweightArr=(float*)(buf+START[indx[i]]);
    info->pxlweight=&(info->pxlweightArr[info->npxlCum]);
  }else{
    printf("pxlweight error\n");
    err=i;
  }
  i=AVERAGEIMG;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->averageImg=((int*)(buf+START[indx[i]]));
    info->nAvImg=*info->averageImg;
  }else{ 
    printf("averageImg error\n");
    err=AVERAGEIMG;
  }
  i=AVERAGECENT;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->averageCent=((int*)(buf+START[indx[i]]));
    info->nAvCent=*info->averageCent;
  }else{ 
    printf("averageCent error\n");
    err=AVERAGECENT;
  }
  i=CALMULT;
  nbytes=NBYTES[indx[i]];
  if(nbytes==0)
    info->calmult=NULL;
  else if(buf[NHDR*31+indx[i]]=='f' && nbytes/sizeof(float)==info->totPxls){
    info->calmult=(float*)(buf+START[indx[i]]);
  }else{
    printf("calmult error\n");
    err=i;
  }
  i=CALSUB;
  nbytes=NBYTES[indx[i]];
  if(nbytes==0)
    info->calsub=NULL;
  else if(buf[NHDR*31+indx[i]]=='f' && nbytes/sizeof(float)==info->totPxls){
    info->calsub=(float*)(buf+START[indx[i]]);
  }else{
    printf("calsub error\n");
    err=i;
  }
  i=CALTHR;
  nbytes=NBYTES[indx[i]];
  if(nbytes==0)
    info->calthr=NULL;
  else if(buf[NHDR*31+indx[i]]=='f' && nbytes/sizeof(float)==info->totPxls){
    info->calthr=(float*)(buf+START[indx[i]]);
  }else{
    printf("calthr error\n");
    err=i;
  }
  i=CENTCALDATA;
  info->centCalnsteps=0;
  nbytes=NBYTES[indx[i]];
  if(nbytes==0)
    info->centCalData=NULL;
  else if(buf[NHDR*31+indx[i]]=='f'){
    info->centCalnsteps=nbytes/sizeof(float)/info->totCents;
    if(nbytes==sizeof(float)*info->totCents*info->centCalnsteps)
      info->centCalData=(float*)(buf+START[indx[i]]);
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
  nbytes=NBYTES[indx[i]];
  if(nbytes==0 || info->centCalnsteps==0)
    info->centCalSteps=NULL;
  else if(buf[NHDR*31+indx[i]]=='f' && nbytes/sizeof(float)==info->totCents*info->centCalnsteps){
    info->centCalSteps=(float*)(buf+START[indx[i]]);
  }else{
    printf("centCalSteps error\n");
    err=i;
  }
  i=CENTCALBOUNDS;
  nbytes=NBYTES[indx[i]];
  if(nbytes==0 || info->centCalnsteps==0)
    info->centCalBounds=NULL;
  else if(buf[NHDR*31+indx[i]]=='i' && nbytes/sizeof(int)==2*info->totCents){
    info->centCalBounds=(int*)(buf+START[indx[i]]);
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
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->figureOpen=((int*)(buf+START[indx[i]]));
    //*((int*)(buf+START[indx[i]]))=0;
  }else{
    err=i;
    printf("figureOpen error\n");
  }
  i=FIGUREPARAMS;
  nbytes=NBYTES[indx[i]];
  if(nbytes==0){
    info->figureParams=NULL;
    info->figureParamsCnt=0;
  }else if(buf[NHDR*31+indx[i]]=='i'){
    info->figureParams=((int*)(buf+START[indx[i]]));
    info->figureParamsCnt=nbytes/sizeof(int);
  }else{
    err=i;
    printf("figureParams error\n");
  }
  i=USEBRIGHTEST;
  nbytes=NBYTES[indx[i]];
  if(buf[NHDR*31+indx[i]]=='i'){
    if(nbytes==sizeof(int)){
      info->useBrightest=*((int*)(buf+START[indx[i]]));
      info->useBrightestArr=NULL;
    }else if(nbytes==sizeof(int)*info->nsubaps){
      info->useBrightest=0;
      info->useBrightestArr=((int*)(buf+START[indx[i]]));
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
  nbytes=NBYTES[indx[i]];
  if(nbytes==0){
    info->figureGain=1;
    info->figureGainArr=NULL;
  }else if(nbytes==sizeof(float) && buf[NHDR*31+indx[i]]=='f'){
    info->figureGain=*((float*)(buf+START[indx[i]]));
    info->figureGainArr=NULL;
  }else if(nbytes==sizeof(float)*info->nacts && buf[NHDR*31+indx[i]]=='f'){
    info->figureGain=0;
    info->figureGainArr=((float*)(buf+START[indx[i]]));
  }else{
    printf("figureGain error\n");
    err=i;
    info->figureGainArr=NULL;
  }
  i=RECONLIBOPEN;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->reconlibOpen=((int*)(buf+START[indx[i]]));
    //*((int*)(buf+START[indx[i]]))=0;
  }else{
    err=i;
    printf("reconlibOpen error\n");
  }
  i=MAXADAPOFFSET;
  if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
    info->maxAdapOffset=*((int*)(buf+START[indx[i]]));
  }else{
    err=i;
    printf("maxAdapOffset error\n");
  }
  i=ADAPTIVEGROUP;
  nbytes=NBYTES[indx[i]];
  if(nbytes==0){
    info->adaptiveGroup=NULL;
    info->nAdaptiveGroups=1;
  }else if(nbytes==sizeof(int)*info->totCents/2 && buf[NHDR*31+indx[i]]=='i'){
    info->adaptiveGroup=((int*)(buf+START[indx[i]]));
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
    printf("adaptiveWinGroup error: nbytes=%d should be %d type %c should be i\n",nbytes,sizeof(int)*info->totCents/2,buf[NHDR*31+indx[i]]);
    err=i;
    info->adaptiveGroup=NULL;
    info->nAdaptiveGroups=1;
  }
  if(updateIndex){//only 1 thread needs to do this one
    i=VERSION;
    nbytes=NBYTES[indx[i]];
    if(nbytes>0 && buf[NHDR*31+indx[i]]=='s'){
      //write the versions into here.
      strncpy(buf+START[indx[i]],CVSID,nbytes-1);
    }

    i=CAMERANAME;
    nbytes=NBYTES[indx[i]];
    if(nbytes==0){
      globals->cameraName=NULL;
    }else if(buf[NHDR*31+indx[i]]=='s'){
      globals->cameraName=buf+START[indx[i]];
    }else{
      printf("cameraName error\n");
      globals->cameraName=NULL;
      err=i;
    }
    i=CENTNAME;
    nbytes=NBYTES[indx[i]];
    if(nbytes==0){
      globals->centName=NULL;
    }else if(buf[NHDR*31+indx[i]]=='s'){
      globals->centName=buf+START[indx[i]];
    }else{
      printf("centroidersName error\n");
      globals->centName=NULL;
      err=i;
    }
    i=FIGURENAME;
    nbytes=NBYTES[indx[i]];
    if(nbytes==0){
      globals->figureName=NULL;
    }else if(buf[NHDR*31+indx[i]]=='s'){
      globals->figureName=buf+START[indx[i]];
    }else{
      printf("figureName error\n");
      globals->figureName=NULL;
      err=i;
    }

    i=MIRRORNAME;
    nbytes=NBYTES[indx[i]];
    if(nbytes==0){
      globals->mirrorName=NULL;
    }else if(buf[NHDR*31+indx[i]]=='s'){
      globals->mirrorName=buf+START[indx[i]];
    }else{
      printf("mirrorName error\n");
      globals->mirrorName=NULL;
      err=i;
    }
    i=RECONNAME;
    nbytes=NBYTES[indx[i]];
    if(nbytes==0){
      globals->reconName=NULL;
    }else if(buf[NHDR*31+indx[i]]=='s'){
      globals->reconName=buf+START[indx[i]];
    }else{
      printf("reconName error\n");
      globals->reconName=NULL;
      err=i;
    }
    i=RECONPARAMS;
    nbytes=NBYTES[indx[i]];
    if(nbytes==0){
      globals->reconParams=NULL;
      globals->reconParamsCnt=0;
    }else if(buf[NHDR*31+indx[i]]=='i'){
      globals->reconParams=((int*)(buf+START[indx[i]]));
      globals->reconParamsCnt=nbytes/sizeof(int);
    }else{
      err=i;
      printf("reconParams error\n");
    }

    i=FLUXTHRESHOLD;
    nbytes=NBYTES[indx[i]];
    if(buf[NHDR*31+indx[i]]=='f'){
      if(nbytes==sizeof(float)){
	globals->fluxThresholdArr=NULL;
	globals->fluxThreshold=*(float*)(buf+START[indx[i]]);
      }else if(nbytes==sizeof(float)*info->totCents/2){
	globals->fluxThresholdArr=(float*)(buf+START[indx[i]]);;
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
    if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
      globals->printUnused=*((int*)(buf+START[indx[i]]));
    }else{
      printf("printUnused error\n");
      err=i;
    }
  }
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
  pthread_mutex_lock(&glob->reconMutex);
  if(open){
    //first close existing, if it is open...
    if(glob->reconLib!=NULL){
      //close existing library.
      (*glob->reconFreeFn)(&threadInfo->globals->reconStruct);
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
	if((*(void**)(&glob->reconInitFn)=dlsym(glob->reconLib,"reconInit"))==NULL){
	  printf("dlsym failed for reconInit\n");
	  err=1;
	}
	if((*(void**)(&glob->reconFreeFn)=dlsym(glob->reconLib,"reconFree"))==NULL){
	  printf("dlsym failed for reconFree\n");
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
	if((err=(*glob->reconInitFn)(glob->reconParamsCnt,glob->reconParams,glob->buffer[glob->curBuf]->buf,&threadInfo->globals->reconStruct,threadInfo->globals->nthreads,glob->thisiter,threadInfo->info->totCents,glob->rtcErrorBuf))){
	  printf("Error calling reconInit function\n");
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
      glob->reconFreeFn=NULL;
      glob->reconInitFn=NULL;
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
    (*glob->reconNewParamFn)(glob->buffer[glob->curBuf]->buf,threadInfo->globals->reconStruct,glob->thisiter,threadInfo->info->totCents);
  pthread_mutex_unlock(&glob->reconMutex);
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
	if((cerr=(*glob->camOpenFn)(glob->cameraName,info->cameraParamsCnt,info->cameraParams,&glob->camHandle,info->totPxls,glob->pxlbufs,info->ncam,info->npxlxList,info->npxlyList,glob->camframeno,glob->buffer[glob->curBuf]->buf,glob->rtcErrorBuf))!=0){//This is probably a blocking call - ie might wait until the camera reaches a certain temerature or whatever...
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
    (*glob->camNewParamFn)(glob->camHandle,glob->buffer[glob->curBuf]->buf,glob->thisiter);
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
	  for(k=0; k<info->nsubxList[j]*info->nsubyList[j]; k++){
	    glob->ncentsList[j]+=info->subapFlagArr[k+offset];
	  }
	  offset+=k;
	  glob->ncentsList[j]*=2;
	}
	if(cerr==0){
	  doneParams=1;//the open function will do parameters...
	  cerr=(*glob->centOpenFn)(glob->centName,info->centroidersParamsCnt,info->centroidersParams,&glob->centHandle,info->centroids,info->ncam,glob->ncentsList,glob->camframeno,glob->buffer[glob->curBuf]->buf,glob->rtcErrorBuf);//This is probably a blocking call - ie might wait until the camera reaches a certain temerature or whatever...
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
    (*glob->centNewParamFn)(glob->centHandle,glob->buffer[glob->curBuf]->buf,glob->thisiter);
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
  pthread_mutex_lock(&glob->mirrorMutex);
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
	  cerr=(*glob->mirrorOpenFn)(glob->mirrorNameOpen,info->mirrorParamsCnt,info->mirrorParams,info->nacts,&glob->mirrorHandle,glob->rtcErrorBuf,glob->rtcActuatorBuf,glob->thisiter,glob->buffer[glob->curBuf]->buf);
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
    cerr=(*glob->mirrorNewParamFn)(glob->mirrorHandle,glob->buffer[glob->curBuf]->buf,glob->thisiter);
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
  pthread_mutex_unlock(&glob->mirrorMutex);

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
	cerr=(*glob->figureOpenFn)(glob->figureName,info->figureParamsCnt,info->figureParams,info->nacts,glob->precomp->post.actsRequiredMutex,glob->precomp->post.actsRequiredCond,&(glob->precomp->post.actsRequired),glob->figureFrame,&glob->figureHandle,glob->buffer[glob->curBuf]->buf,glob->rtcErrorBuf);
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
    (*glob->figureNewParamFn)(glob->figureHandle,glob->buffer[glob->curBuf]->buf,glob->thisiter);
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
  globalStruct *globals=threadInfo->globals;
  int err=0;
  short *tmps;
  //now allocate an array large enough to hold all pixels (note - this may not be used, depending on the camera library...)
  if(globals->pxlbufsSize!=sizeof(short)*info->totPxls){
    globals->pxlbufsSize=sizeof(short)*info->totPxls;
    tmps=realloc(globals->pxlbufs,globals->pxlbufsSize);
    if(tmps==NULL){
      if(globals->pxlbufs!=NULL)
	free(globals->pxlbufs);
      printf("pxlbuf malloc error.\n");
      err=1;
      globals->pxlbufsSize=0;
    }
    globals->pxlbufs=tmps;
    memset(globals->pxlbufs,0,globals->pxlbufsSize);
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
  //For the centroids, we have to do something special.  The centroids array gets passed to librtccent.so, so can't have it changing every time a buffer is swapped.  Therefore, there is only 1 array, which is shared here, and referenced by both arrayStruct objects.  This kind of defeats the object of having the 2 arrayStruct objects/double buffering, but I think it is no longer necessary - one day, I might get round to rationalising it and removing it.  There MIGHT be a problem here if the number of centroids changes.  Certainly, the WPU library would need reopening.
  if(glob->centroidsSize<info->totCents){
    if(glob->centroids!=NULL)
      free(glob->centroids);
    glob->centroidsSize=info->totCents;
    if((glob->centroids=malloc(sizeof(float)*info->totCents))==NULL){
      printf("malloc of centroids failed\n");
      err=1;
      glob->centroidsSize=0;
    }
    //glob2->centroidsSize=glob->centroidsSize;
    //glob2->centroids=glob->centroids;
  }
  
  /*if(glob->precompDMXpredSize<info->totCents){
    if(glob->precompDMXpred!=NULL)
      free(glob->precompDMXpred);
    glob->precompDMXpredSize=info->totCents;
    if((glob->precompDMXpred=malloc(sizeof(float)*info->totCents))==NULL){
      printf("malloc of precompDMXpred failed\n");
      err=1;
      glob->precompDMXpredSize=0;
    }
    }*/

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
  /*
  if(glob->latestDmCommandSize<info->nacts){
    if(glob->latestDmCommand!=NULL)
      free(glob->latestDmCommand);
    glob->latestDmCommandSize=info->nacts;
    if((glob->latestDmCommand=malloc(sizeof(float)*info->nacts))==NULL){
      printf("malloc of latestDmCommand failed\n");
      err=1;
      glob->latestDmCommandSize=0;
    }
    }*/

  /*if(glob->actsSentSize<info->nacts){
    if(glob->actsSent!=NULL)
      free(glob->actsSent);
    glob->actsSentSize=info->nacts;
    if((glob->actsSent=malloc(sizeof(unsigned short)*info->nacts))==NULL){
      printf("malloc of actsSent failed\n");
      err=1;
      glob->actsSentSize=0;
    }
    }*/

  /*if(glob->XpredSize<info->kalmanPhaseSize*3){
    if(glob->Xpred!=NULL)
      free(glob->Xpred);
    glob->XpredSize=info->kalmanPhaseSize*3;
    if((glob->Xpred=malloc(sizeof(float)*info->kalmanPhaseSize*3))==NULL){
      printf("malloc of Xpred failed\n");
      err=1;
      glob->XpredSize=0;
    }
    }*/

  //if(glob->XdmcSize<info->kalmanPhaseSize*3){
  //  if(glob->Xdmc!=NULL)
  //    free(glob->Xdmc);
  //  glob->XdmcSize=info->kalmanPhaseSize*3;
  //  if((glob->Xdmc=malloc(sizeof(float)*info->kalmanPhaseSize*3))==NULL){
  //    printf("malloc of Xdmc failed\n");
  //    err=1;
  //    glob->XdmcSize=0;
  //  }
  //}
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

  //tmp=threadInfo->info;
  //threadInfo->info=threadInfo->infobuf;
  //threadInfo->infobuf=tmp;

  //If this is first thread for this camera, update the infoStruct buffer.
  info=threadInfo->info;
  //if(threadInfo->globals->updateBuf[info->cam]){//first thread for this camera
  //threadInfo->globals->updateBuf[info->cam]=0;
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
    //if(threadInfo->globals->cameraLib!=NULL){
    // (*threadInfo->globals->camCloseFn)(&threadInfo->globals->camHandle);
    //}
    //and now close the dynamic library...
    //if(dlclose(threadInfo->globals->cameraLib)!=0){
    // printf("Failed to close camera dynamic library - ignoring\n");
    //}
    //threadInfo->globals->cameraLib=NULL;
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

    /*if(threadInfo->globals->centLib!=NULL){
      (*threadInfo->globals->centCloseFn)(&threadInfo->globals->centHandle);
    }
    //and now close the dynamic library...
    if(dlclose(threadInfo->globals->centLib)!=0){
      printf("Failed to close centroider camera dynamic library - ignoring\n");
    }
    threadInfo->globals->centLib=NULL;*/
  }
  
  //if(*info->mirrorOpen==1){
  //do this every bufferswap, incase new params are read by the library...
  err|=updateMirror(threadInfo);
  /*if(threadInfo->globals->mirrorHandle==NULL){
  //connect to the mirror
  cerr=openMirror(threadInfo);
  }else{
  //already open - but is it the correct one...???
  if(threadInfo->globals->mirrorName==NULL){//library unspecified - so want to close it...
  
  }
  }*/
  //}
  /*
    if(*info->mirrorOpen==0 && threadInfo->globals->mirrorHandle!=NULL){
    //disconnect from the mirror
    if(threadInfo->globals->mirrorLib!=NULL)
    (*threadInfo->globals->mirrorCloseFn)(&threadInfo->globals->mirrorHandle);
    //and now close the dynamic library.
    if(dlclose(threadInfo->globals->mirrorLib)!=0){
    printf("Failed to close mirror dynamic library - ignoring\n");
    }
    threadInfo->globals->mirrorLib=NULL;
    if(threadInfo->globals->mirrorHandle!=NULL){
    printf("Error: mirrorHandle not NULL (%p) - setting to NULL\n",threadInfo->globals->mirrorHandle);
    threadInfo->globals->mirrorHandle=NULL;
    }
    }*/
  //if(*info->figureOpen==1){// && threadInfo->globals->figureHandle==NULL){
    //connect to the figure sensor setpoint reading library.  This is used when this RTC is being used as a figure sensor.
  cerr=openFigure(threadInfo);
  err|=cerr;
  //}
  /*
  if(*info->figureOpen==0 && threadInfo->globals->figureHandle!=NULL){
    //disconnect from the figure sensor library
    if(threadInfo->globals->figureLib!=NULL)
      (*threadInfo->globals->figureCloseFn)(&threadInfo->globals->figureHandle);
    //and now close the dynamic library.
    if(dlclose(threadInfo->globals->figureLib)!=0){
      printf("Failed to close figure sensor dynamic library - ignoring\n");
    }
    threadInfo->globals->figureLib=NULL;
    if(threadInfo->globals->figureHandle!=NULL){
      printf("Error: figureHandle not NULL (%p) - setting to NULL\n",threadInfo->globals->figureHandle);
      threadInfo->globals->figureHandle=NULL;
    }
    }*/
  //Open the reconstructor library (if correct one not open) and give it the new parameters.
  err|=updateReconLibrary(threadInfo);
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
  //and now malloc each threads dmCommand array.
  /*
  if(threadInfo->info->kalmanUsed==0){
    if(threadInfo->dmCommand==NULL || threadInfo->dmCommandSize<threadInfo->info->nacts*sizeof(float)){
      if(threadInfo->dmCommand!=NULL)
	free(threadInfo->dmCommand);
      threadInfo->dmCommandSize=sizeof(float)*threadInfo->info->nacts;
      if((threadInfo->dmCommand=malloc(threadInfo->dmCommandSize))==NULL){
	printf("threadInfo->dmCommand malloc error\n");
	err=1;
	threadInfo->dmCommandSize=0;
      }
    }
  }else{
    if(threadInfo->Xpred==NULL || threadInfo->XpredSize<threadInfo->info->kalmanPhaseSize*3*sizeof(float)){
      if(threadInfo->Xpred!=NULL)
	free(threadInfo->Xpred);
      threadInfo->XpredSize=sizeof(float)*threadInfo->info->kalmanPhaseSize*3;
      if((threadInfo->Xpred=malloc(threadInfo->XpredSize))==NULL){
	printf("threadInfo->Xpred malloc error\n");
	err=1;
	threadInfo->XpredSize=0;
      }
    }
    }*/
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
  //info->precompDMXpred=glob->precompDMXpred;
  info->dmCommand=glob->dmCommand;
  //info->latestDmCommand=glob->latestDmCommand;
  //info->actsSent=glob->actsSent;
  //info->Xpred=glob->Xpred;
  //info->Xdmc=glob->Xdmc;
  info->calpxlbuf=glob->calpxlbuf;
  info->corrbuf=glob->corrbuf;
  if(threadInfo->globals->pxlbufs!=NULL)
    info->pxlbuf=&threadInfo->globals->pxlbufs[info->npxlCum];
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
   The centroids are put into globals->centroids which is same as info->centroids.
*/
int waitCentroids(int cnt,threadStruct *threadInfo){
  int rt=0;
  if(threadInfo->globals->centLib!=NULL)
    rt=(*threadInfo->globals->centWaitPixelsFn)(cnt,threadInfo->info->cam,threadInfo->globals->centHandle);
  return rt;
}

/**
   Called instead of waitNextSubaps when WPU is being used to compute slopes.
 */
int getNextCentroids(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  int endFrame=0;
  int cnt=0,nsubs=0,i;
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
    nsubs=info->centindx;//this is the number of subaps that we need to have arrived for this camera before continuing.
    info->subindx+=info->nsubapsTogether;
    if(info->subindx>=info->nsubx*info->nsuby){
      info->subindx=0;
      info->centindx=0;
      //this thread has valid data, so still want to process it, butother threads shouldn;t as frame is finished.
      info->frameFinished=1;//091109[threadInfo->mybuf]=1;//let other threadsknow that we've read all the subaps...
    }
    endFrame=(waitCentroids(nsubs,threadInfo)!=0);
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






/*
int getNextCentroids(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  int endFrame=0;
  //Threads run in order, and are returned a subap as it becomes available...
  //first block in the thread queue until it is my turn.
  printf("TODO - nsubapsDone/nsubapsTogether in getNextCentroids\n");
  dprintf("locking subapMutex\n");
  if(pthread_mutex_lock(&info->subapMutex))
    printf("pthread_mutex_lock error in getNextCentroid: %s\n",strerror(errno));
  dprintf("locked subapMutex %d %p %d\n",info->subindx,info->subflag,info->id);
  if(info->frameFinished[threadInfo->mybuf]==1){
    dprintf("frameFinished in getNextCentroid: %d\n",info->frameFinished[threadInfo->mybuf]);
    pthread_mutex_unlock(&info->subapMutex);
    return -1;
  }
  //work out which is the next subap to do... and how many...
  while(info->subflag[info->subindx]==0){
    dprintf("subflag[%d] %d\n",info->subindx,info->subflag[info->subindx]);
    info->subindx++;
    if(info->subindx>=info->nsubx*info->nsuby){
      info->subindx=0;
      info->centindx=0;
      endFrame=-1;
      info->frameFinished[threadInfo->mybuf]=1;//let other threadsknow that we've read all the subaps...
      break;
    }
  }
  if(endFrame==0){
    //store the indx for this thread...
    threadInfo->centindx=info->centindx+info->centCumIndx;
    threadInfo->cursubindx=info->subindx+info->subCumIndx;
    //threadInfo->cursuby=info->subindx/info->nsubx;
    //threadInfo->cursubx=info->subindx%info->nsubx;
    //and increment subindx ready for the next thread.
    info->centindx+=2;
    info->subindx++;
    if(info->subindx>=info->nsubx*info->nsuby){
      info->subindx=0;
      info->centindx=0;
      //this thread has valid data, so still want to process it, butother threads shouldn;t as frame is finished.
      info->frameFinished[threadInfo->mybuf]=1;//let other threadsknow that we've read all the subaps...
    }
    dprintf("waiting centroids\n");
    //then wait until pixels become available...
    waitCentroids();
    dprintf("waited centroids\n");

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
  //p->reconMode=info->reconMode;
  p->nacts=info->nacts;
  //p->mybuf=threadInfo->mybuf;
  //p->gainE=info->gainE;
  //p->v0=info->v0;
  //p->kalmanUsed=info->kalmanUsed;
  //p->kalmanReset=info->kalmanReset;
  //p->kalmanPhaseSize=info->kalmanPhaseSize;
  p->totCents=info->totCents;
  //p->kalmanHinfDM=info->kalmanHinfDM;
  //these are the ones that are written to by the threads and so must be synced
  p->dmCommand=info->dmCommand;
  //p->dmCommandSave=threadInfo->globals->arrays->dmCommandSave;
  //p->latestDmCommand=info->latestDmCommand;
  //p->Xdmc=info->Xdmc;
  //p->Xpred=info->Xpred;
  //p->precompDMXpred=info->precompDMXpred;
  //set the dmReady flag (safely)
  //xxx this can be moved into the recon library...
  //if(pthread_mutex_lock(&p->dmMutex))
  //  printf("pthread_mutex_lock error in wakePrepareActuatorsThread: %s\n",strerror(errno));
  ////p->dmReady=0;
  //pthread_mutex_unlock(&p->dmMutex);
 
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
  STARTTIMING;
  if(pthread_mutex_lock(&threadInfo->globals->precomp->pxlcentMutex))
    printf("pthread_mutex_lock error in endFrame: %s\n",strerror(errno));
  threadInfo->globals->precomp->pxlcentReady=0;
  pthread_mutex_unlock(&threadInfo->globals->precomp->pxlcentMutex);

  if(pthread_mutex_lock(&threadInfo->globals->precomp->postMutex))
    printf("pthread_mutex_lock error in endFrame2: %s\n",strerror(errno));
  //set up the correct pointers...
  //p->kalmanAtur=info->kalmanAtur;
  //p->actMax=info->actMax;
  //p->actMin=info->actMin;
  p->closeLoop=info->closeLoop;
  //p->bleedGainOverNact=info->bleedGainOverNact;
  //p->midRangeTimesBleed=info->midRangeTimesBleed;
  p->userActs=info->userActs;
  p->userActsMask=info->userActsMask;
  p->addUserActs=info->addUserActs;
  p->userActSeq=info->userActSeq;
  p->userActSeqLen=info->userActSeqLen;
  p->recordCents=info->recordCents;
  p->totPxls=info->totPxls;
  p->nAvImg=info->nAvImg;
  //p->v0=info->v0;
  p->averageImg=info->averageImg;
  p->nAvCent=info->nAvCent;
  p->averageCent=info->averageCent;
  //p->usingDMC=info->usingDMC;
  //p->kalmanInvN=info->kalmanInvN;
  //p->actsSent=info->actsSent;
  p->nacts=info->nacts;
  p->totCents=info->totCents;
  p->maxClipped=info->maxClipped;
  //these are written to by the post compute thread...
  p->calpxlbuf=info->calpxlbuf;
  p->corrbuf=info->corrbuf;
  p->centroids=info->centroids;
  p->flux=info->flux;
  p->dmCommand=info->dmCommand;
  p->dmCommandSave=threadInfo->globals->arrays->dmCommandSave;
  p->figureGain=info->figureGain;
  p->figureGainArr=info->figureGainArr;
  p->dmCommandFigure=threadInfo->globals->arrays->dmCommandFigure;
  //p->latestDmCommand=info->latestDmCommand;
  //p->Xdmc=info->Xdmc;
  //p->Xpred=info->Xpred;
  //p->precompDMXpred=info->precompDMXpred;
  //p->kalmanUsed=info->kalmanUsed;
  //p->kalmanReset=info->kalmanReset;
  //p->kalmanPhaseSize=info->kalmanPhaseSize;
  p->mirrorHandle=threadInfo->globals->mirrorHandle;
  p->mirrorSendFn=threadInfo->globals->mirrorSendFn;
  p->reconOpenLoopFn=threadInfo->globals->reconOpenLoopFn;
  p->reconFrameFinishedFn=threadInfo->globals->reconFrameFinishedFn;
  p->thisiter=threadInfo->globals->thisiter;
  p->windowMode=info->windowMode;
  p->subapLocation=info->subapLocation;
  p->pxlCentInputError=info->pxlCentInputError;
  //p->adaptiveWinGain=info->adaptiveWinGain;
  //p->kalmanDM=info->kalmanDM;
  if(threadInfo->info->windowMode==WINDOWMODE_GLOBAL)
    calcGlobalAdaptiveWindow(threadInfo->info);


  //gettimeofday(&thistime,NULL);
  //timestamp=thistime.tv_sec+thistime.tv_usec*1e-6;
  if(FORCEWRITEALL(threadInfo->globals->rtcPxlBuf)){
    FORCEWRITE(threadInfo->globals->rtcPxlBuf)=FORCEWRITEALL(threadInfo->globals->rtcPxlBuf);
    FORCEWRITE(threadInfo->globals->rtcStatusBuf)=FORCEWRITEALL(threadInfo->globals->rtcPxlBuf);
    FORCEWRITE(threadInfo->globals->rtcTimeBuf)=FORCEWRITEALL(threadInfo->globals->rtcPxlBuf);
    FORCEWRITE(threadInfo->globals->rtcCalPxlBuf)=FORCEWRITEALL(threadInfo->globals->rtcPxlBuf);
    FORCEWRITE(threadInfo->globals->rtcCorrBuf)=FORCEWRITEALL(threadInfo->globals->rtcPxlBuf);
    FORCEWRITE(threadInfo->globals->rtcCentBuf)=FORCEWRITEALL(threadInfo->globals->rtcPxlBuf);
    FORCEWRITE(threadInfo->globals->rtcFluxBuf)=FORCEWRITEALL(threadInfo->globals->rtcPxlBuf);
    FORCEWRITE(threadInfo->globals->rtcSubLocBuf)=FORCEWRITEALL(threadInfo->globals->rtcPxlBuf);
    FORCEWRITE(threadInfo->globals->rtcMirrorBuf)=FORCEWRITEALL(threadInfo->globals->rtcPxlBuf);
    FORCEWRITE(threadInfo->globals->rtcActuatorBuf)=FORCEWRITEALL(threadInfo->globals->rtcPxlBuf);
    FORCEWRITEALL(threadInfo->globals->rtcPxlBuf)=0;
  }
  //circAddPartial(threadInfo->globals->rtcPxlBuf,info->pxlbuf,info->npxlCum*sizeof(short),info->npxlx*info->npxly*sizeof(short),timestamp,threadInfo->globals->thisiter);//if remove this, can also remove the previous gettimeofday.


  //No need to get the reconMutex here, since this is never called at the same time as the updateRecon method...
  if(threadInfo->globals->reconLib!=NULL && threadInfo->globals->reconStruct!=NULL)
    (*threadInfo->globals->reconFrameFinishedSyncFn)(threadInfo->globals->reconStruct,threadInfo->info->centroids,threadInfo->globals->arrays->dmCommand,info->pxlCentInputError);
  pthread_cond_signal(&threadInfo->globals->precomp->postCond);//wake the thread
  pthread_mutex_unlock(&threadInfo->globals->precomp->postMutex);
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
    //if(pthread_cond_wait(&glob->precomp->prepCond,&glob->precomp->prepMutex))
    //  printf("pthread_cond_wait error in prepareActuators: %s\n",strerror(errno));
    p=glob->precomp;
    dprintf("Precomputing dmCommand\n");
    /*if(p->kalmanUsed){
      if(p->kalmanReset==1){
	//memset(p->Xdmc,0,sizeof(float)*p->kalmanPhaseSize*3);
	memset(p->Xpred,0,sizeof(float)*p->kalmanPhaseSize*3);
	//memset(p->precompDMXpred,0,sizeof(float)*p->totCents);
      }else{
	//beta=0.;
      	//performn precomp=dot(DM,Xpred[phaseSize*2:])
	//cblas_sgemv(order,trans,p->kalmanPhaseSize*3,p->kalmanPhaseSize,alpha,p->kalmanHinfDM,p->kalmanPhaseSize,&(p->Xpred[p->kalmanPhaseSize*2]),inc,beta,p->precompDMXpred,inc);
	beta=-1.;
	cblas_sgemv(order,trans,p->kalmanPhaseSize*3,p->kalmanPhaseSize,alpha,p->kalmanHinfDM,p->kalmanPhaseSize,&(p->Xpred[p->kalmanPhaseSize*2]),inc,beta,p->Xpred,inc);
      }
      //setDMArraysReady(glob);
    }else{//not using kalman
    */
    pthread_mutex_lock(&glob->reconMutex);
    if(glob->reconLib!=NULL && glob->reconStruct!=NULL && glob->reconNewFrameFn!=NULL)//First do the recon library.
      (*glob->reconNewFrameFn)(glob->reconStruct,glob->arrays->dmCommand);
    pthread_mutex_unlock(&glob->reconMutex);
      /*
      if(p->reconMode==RECONMODE_SIMPLE){//simple open loop
	//memset(p->dmCommand,0,sizeof(float)*p->nacts);
	memcpy(p->dmCommand,p->v0,sizeof(float)*p->nacts);
      }else if(p->reconMode==RECONMODE_TRUTH){//closed loop
	memcpy(p->dmCommand,p->latestDmCommand,sizeof(float)*p->nacts);
      }else if(p->reconMode==RECONMODE_OPEN){//reconmode_open
	//initialise by copying v0 into the result.
	memcpy(p->dmCommand,p->v0,sizeof(float)*p->nacts);
	//Now: dmcommand=v0+dot(gainE,latestDmCommand)
	beta=1.;
	cblas_sgemv(order,trans,p->nacts,p->nacts,alpha,p->gainE,p->nacts,p->latestDmCommand,inc,beta,p->dmCommand,inc);
      }else{//reconmode_offset
	memcpy(p->dmCommand,p->v0,sizeof(float)*p->nacts);
      }	
      setDMArraysReady(glob);
      */
    //}
    //xpred can now be written to by other threads.
    //dmcommand can now be written to by other threads.
    //latestdmcommand can now be written to by other threads.
    //precomputedmxpred and xdmc can now be written.

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
      circAdd(glob->rtcPxlBuf,glob->pxlbufs,timestamp,pp->thisiter);
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
    //pthread_mutex_unlock(&threadInfo->globals->precomp->prepMutex);
    //freeDMCommandBufferLock(threadInfo);
    //and send the rest to the circular buffers...
    //if(pp->pxlCentInputError==0){
      //circAdd(glob->rtcActuatorBuf,pp->actsSent,timestamp,pp->thisiter);//actsSent);
    //}
    writeStatusBuf(glob,0,pp->closeLoop);
    circAdd(glob->rtcStatusBuf,glob->statusBuf,timestamp,pp->thisiter);
    

    /*if(pp->kalmanUsed==1 && pp->kalmanReset==0){//do the kalman predict step.
      memcpy(&(pp->Xpred[pp->kalmanPhaseSize*2]),&(pp->Xpred[pp->kalmanPhaseSize*1]),sizeof(float)*pp->kalmanPhaseSize);
      memcpy(&(pp->Xpred[pp->kalmanPhaseSize]),pp->Xpred,sizeof(float)*pp->kalmanPhaseSize);
      //Perform Xpred[:phaseSize]=dot(Atur,Xpred[:phaseSize])
      beta=0.;
      cblas_sgemv(order,trans,pp->kalmanPhaseSize,pp->kalmanPhaseSize,alpha,pp->kalmanAtur,pp->kalmanPhaseSize,&(pp->Xpred[pp->kalmanPhaseSize]),inc,beta,pp->Xpred,inc);
      }*/
  }
  //and now allow other things access to the buffer.
  //pthread_mutex_unlock(&p->prepMutex);
  pthread_mutex_unlock(&p->postMutex);
  return 0;
}
/*
int getClearSwitchRequested(globalStruct *globals){
  //retrieves value of switchRequested in the buffer, and if set, clears it.
  int j=0;
  int sr=0;
  char *buf=globals->buffer[globals->curBuf]->buf;
  while(j<NHDR && buf[j*31]!='\0'){
    if(strncmp(&buf[j*31],"switchRequested",31)==0){
      if(buf[NHDR*31+j]=='i' && NBYTES[j]==4){
	sr=*((int*)(buf+START[j]));
	*((int*)(buf+START[j]))=0;//clear the flag.
      }else{
	printf("switchRequested error\n");
      }
      break;
    }
  }
  return sr;
  }*/
int getSwitchRequested(globalStruct *globals){
  //retrieves value of switchRequested in the buffer, and if set, clears it.
  int j=0;
  int sr=0;
  char *buf=globals->buffer[globals->curBuf]->buf;
  while(j<NHDR && buf[j*31]!='\0'){
    if(strncmp(&buf[j*31],"switchRequested",31)==0){
      if(buf[NHDR*31+j]=='i' && NBYTES[j]==4){
	sr=*((int*)(buf+START[j]));
	//*((int*)(buf+START[j]))=0;//clear the flag.
	if(sr==1)
	  globals->switchRequestedPtr=((int*)(buf+START[j]));
      }else{
	printf("switchRequested error\n");
      }
      break;
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
  /*This should now be updated by the mirror library, if required.
  if(threadInfo->globals->rtcActuatorBuf!=NULL && threadInfo->globals->rtcActuatorBuf->datasize!=info->nacts*sizeof(unsigned short)){
    dim=info->nacts;
    if(circReshape(threadInfo->globals->rtcActuatorBuf,1,&dim,'H')!=0){
      printf("Error reshaping rtcActuatorBuf\n");
      err=1;
    }
    }*/
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
  while(niters!=0 && info->go==1){
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
    //if(threadInfo->globals->threadCount/*[threadInfo->mybuf]091109*/==threadInfo->globals->nthreads){//last thread...
    //  threadInfo->globals->doswitch=0;//091109[threadInfo->mybuf]=0;
    //  threadInfo->globals->threadCount=0;//091109[threadInfo->mybuf]=0;
      /*
      if(pthread_mutex_lock(&threadInfo->globals->bufMutex))
	printf("pthread_mutex_lock error in processFrame3: %s\n",strerror(errno));
      
      if(threadInfo->globals->bufInUse==1){
	threadInfo->globals->bufInUse=0;
	if(threadInfo->globals->bufBlocking){
	  pthread_cond_signal(&threadInfo->globals->bufCond);
	}
	threadInfo->globals->bufBlocking=0;
	}
      pthread_mutex_unlock(&threadInfo->globals->bufMutex);
      */
    //}
    //pthread_mutex_unlock(&threadInfo->globals->startMutex);//091109[threadInfo->mybuf]);


    info=threadInfo->info;
    if(info->go==0)
      break;
    ////////////////////////////////////////////////////////////////
    //Now do the computation...
    nsubapDone=0;
    err=0;
    if(info->pause==0){
      dprintf("Doing computation %d %d\n",threadInfo->threadno,niters);

      //if(info->kalmanUsed==0){
      //no need to get the reconMutex here...
      if(threadInfo->globals->reconLib!=NULL && threadInfo->globals->reconStruct!=NULL)//First do the recon library.
	(*threadInfo->globals->reconStartFrameFn)(threadInfo->globals->reconStruct,threadInfo->threadno);
      //memset(threadInfo->dmCommand,0,threadInfo->dmCommandSize);
      //}else
	//memset(threadInfo->Xpred,0,threadInfo->XpredSize);
      //while(info->frameFinished[threadInfo->mybuf]==0){091109
      while(info->frameFinished==0){
	dprintf("framefinished %d niter %d\n",info->frameFinished,niters);
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
	  dprintf("getting subap %d\n",info->id);
	  if((err=waitNextSubaps(threadInfo))==0){//got subap okay
	    //here we need to use nsubapsTogether
	    cursubindx=threadInfo->cursubindx;
	    centindx=threadInfo->centindx;
	    for(i=0; i<info->nsubapsTogether; i++){
	      if(info->subflag[threadInfo->cursubindx-info->subCumIndx]==1){
		//this subap is valid, so do stuff...
		//and now copy the pixels into our float array...
		dprintf("copying subap %d\n",endFrame);
		copySubap(threadInfo);
		dprintf("got subap\n");
		subapPxlCalibration(threadInfo);
		if(nsubapDone==0){
		  dprintf("WaitPxlBufAndCentroidReady\n");
		  waitPxlBufAndCentroidReady(threadInfo->globals);//wait until the pxlbuf and centroids array have been finished by the previous endframe thread...
		}
		dprintf("storeCalibratedSubap\n");
		storeCalibratedSubap(threadInfo);
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
	  }else{//didn't get subap okay.
	    dprintf("error getting subap %d\n",err);
	  }
	}
	if(err==0){//otherwise, frame probably finished (-1) or error (1)...
	  //if(nsubapDone==0){
	  //  waitDMArraysReady(threadInfo->globals);//wait until the prepareActuators part is ready - ie the dmCommand array has been initialised...
	  //  printf("dmArraysReady\n");
	  //}
	  dprintf("reconstructing...\n");
	  //partialReconstruct(threadInfo);
	  if(threadInfo->globals->reconLib!=NULL && threadInfo->globals->reconStruct!=NULL){
	    (*threadInfo->globals->reconNewSlopesFn)(threadInfo->globals->reconStruct,threadInfo->centindx,threadInfo->threadno,threadInfo->nsubapsDoing,threadInfo->info->centroids,threadInfo->globals->arrays->dmCommand);
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
      if(threadInfo->globals->reconLib!=NULL && threadInfo->globals->reconStruct!=NULL){
	(*threadInfo->globals->reconEndFrameFn)(threadInfo->globals->reconStruct,threadInfo->threadno,threadInfo->info->centroids,threadInfo->globals->arrays->dmCommand,threadInfo->err);
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
      //add this frame to the circular buffer.
      /*if(threadInfo->globals->threadCountFinished[threadInfo->mybuf]==1)
	increment=1;
      else if( threadInfo->globals->threadCountFinished[threadInfo->mybuf]==threadInfo->globals->nthreads)
	increment=-1;
      else
      increment=0;*/
      if(info->pause==0){//not paused...
	/*gettimeofday(&thistime,NULL);
	timestamp=thistime.tv_sec+thistime.tv_usec*1e-6;
	if(FORCEWRITEALL(threadInfo->globals->rtcPxlBuf)){
	  FORCEWRITE(threadInfo->globals->rtcPxlBuf)=FORCEWRITEALL(threadInfo->globals->rtcPxlBuf);
	  FORCEWRITE(threadInfo->globals->rtcStatusBuf)=FORCEWRITEALL(threadInfo->globals->rtcPxlBuf);
	  FORCEWRITE(threadInfo->globals->rtcTimeBuf)=FORCEWRITEALL(threadInfo->globals->rtcPxlBuf);
	  FORCEWRITE(threadInfo->globals->rtcCalPxlBuf)=FORCEWRITEALL(threadInfo->globals->rtcPxlBuf);
	  FORCEWRITE(threadInfo->globals->rtcCorrBuf)=FORCEWRITEALL(threadInfo->globals->rtcPxlBuf);
	  FORCEWRITE(threadInfo->globals->rtcCentBuf)=FORCEWRITEALL(threadInfo->globals->rtcPxlBuf);
	  FORCEWRITE(threadInfo->globals->rtcFluxBuf)=FORCEWRITEALL(threadInfo->globals->rtcPxlBuf);
	  FORCEWRITE(threadInfo->globals->rtcSubLocBuf)=FORCEWRITEALL(threadInfo->globals->rtcPxlBuf);
	  FORCEWRITE(threadInfo->globals->rtcMirrorBuf)=FORCEWRITEALL(threadInfo->globals->rtcPxlBuf);
	  FORCEWRITE(threadInfo->globals->rtcActuatorBuf)=FORCEWRITEALL(threadInfo->globals->rtcPxlBuf);
	  FORCEWRITEALL(threadInfo->globals->rtcPxlBuf)=0;
	}
	circAddPartial(threadInfo->globals->rtcPxlBuf,info->pxlbuf,info->npxlCum*sizeof(short),info->npxlx*info->npxly*sizeof(short),timestamp,threadInfo->globals->thisiter);//if remove this, can also remove the previous gettimeofday.
	*/
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
//Now the python call:  Initialise threads and start them running.



