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
This file does the bulk of processing for the RTC.
*/
#define USECOND
#define GITID "$Id$"   //: darccore.c,v 1.41 2010/07/16 07:50:17 ali Exp $"
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
    FREQ(rtcErrorBuf)=1;
    gettimeofday(&t1,NULL);
    va_start(ap,txt);
    if((l=vasprintf(&tmp,txt,ap))>0){
      if(l>1024){
	l=1024;
	tmp[1023]='\0';
      }
      printf("rtcErrorBuf: %sn",tmp);
      circAddSize(rtcErrorBuf,tmp,strlen(tmp)+1,0,t1.tv_sec+t1.tv_usec*1e-6,frameno);
      free(tmp);
    }else{//error doing formatting... just print the raw string.
      l=strlen(txt)+1;
      if(l>1024){
	l=1024;
	txt[1023]='\0';
      }
      printf("rtcErrorBuf: %s",txt);
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
    FREQ(rtcErrorBuf)=1;
    printf("rtcErrorBuf: %s\n",txt);
    gettimeofday(&t1,NULL);
    circAddSize(rtcErrorBuf,txt,strlen(txt)+1,0,t1.tv_sec+t1.tv_usec*1e-6,frameno);
  }
  pthread_mutex_unlock(mut);
}
/**
   This is called before the main processing threads do their jobs, so that they are forced to wait for post processing to finish such that the arrays can be rewritten.
   Note - *StartFrameFn and *NewFrameSyncFn will already have been called, and *NewFrameFn may or may not have been called.
*/
void waitForArraysReady(globalStruct *glob){
  if(!glob->precomp->post.noPrePostThread){
    if(glob->calCentReady==0){
      pthread_mutex_lock(&glob->calCentMutex);
      if(glob->calCentReady==0)
	pthread_cond_wait(&glob->calCentCond,&glob->calCentMutex);
      pthread_mutex_unlock(&glob->calCentMutex);
    }
  }
}
/**
   This is called by post processing thread, after it has finished with the contents of the arrays.
*/

void setArraysReady(globalStruct *glob){
  if(!glob->precomp->post.noPrePostThread){
    pthread_mutex_lock(&glob->calCentMutex);
    glob->calCentReady=1;
    pthread_cond_broadcast(&glob->calCentCond);
    pthread_mutex_unlock(&glob->calCentMutex);
  }
}


/**
   Computes number of pixels required to read out this subaperture, based on the realSubapLocation and subapLocation (which may be the same thing)..
*/

int computePixelsRequired(threadStruct *threadInfo){
  int i,npxl=0,maxpxl=0,j;
  infoStruct *info=threadInfo->info;
  globalStruct *glob=threadInfo->globals;
  int *loc=NULL,*rloc;
  int yscale=1,xscale=1;
  if(glob->subapLocationType==0){//yfrom,yto,ystep,xfrom,xto,xstep.
    if(info->subapLocation==glob->realSubapLocation){//not doing adaptive windowing...
      if(glob->subapLocationType==0){//yfrom,yto,ystep,xfrom,xto,xstep
	for(i=0; i<threadInfo->nsubapsProcessing; i++){
	  if(glob->subapFlagArr[i+threadInfo->cursubindx]==1){
	    //subap is valid.
	    npxl=glob->pxlCnt[threadInfo->cursubindx+i];
	    if(npxl>maxpxl)
	      maxpxl=npxl;
	  }
	}
      }
    }else{//adaptive windowing.
      for(i=0; i<threadInfo->nsubapsProcessing; i++){
	if(glob->subapFlagArr[i+threadInfo->cursubindx]==1){
	  //subap is valid.
	  loc=&(info->subapLocation[(threadInfo->cursubindx+i)*6]);//adaptive - but points to realSubapLocation if not using adaptive windowing.
	  rloc=&(glob->realSubapLocation[(threadInfo->cursubindx+i)*6]);//user defined
	  if(glob->adapWinShiftCnt!=NULL){//If we have a strange CCD architecture, e.g. split readout, it may be necessary for the pxlCnt to change in a non-standard way (e.g. reduce as loc increases for example) and so an optional scale factor (which in that case would be -1) can be multiplied in.
	    yscale=glob->adapWinShiftCnt[(threadInfo->cursubindx+i)*2];
	    xscale=glob->adapWinShiftCnt[(threadInfo->cursubindx+i)*2+1];
	  }
	  //calculate the new number of pixels required for this shifted subap.
	  npxl=glob->pxlCnt[threadInfo->cursubindx+i]+((loc[0]-rloc[0])/rloc[2])*info->npxlx*yscale+((loc[3]-rloc[3])/rloc[5])*xscale;
	  if(npxl>maxpxl)//and keep a note of the maximum number required.
	    maxpxl=npxl;
	}
      }
    }
  }else{//pixel numbers
    for(i=0; i<threadInfo->nsubapsProcessing; i++){
      if(glob->subapFlagArr[i+threadInfo->cursubindx]==1){
	//subap is valid
	loc=&(info->subapLocation[(threadInfo->cursubindx+i)*glob->maxPxlPerSubap]);
	rloc=&(glob->realSubapLocation[(threadInfo->cursubindx+i)*glob->maxPxlPerSubap]);
	//calculate the new number of pixels required for this shifted subap.
	maxpxl=0;
	for(j=0; j<glob->maxPxlPerSubap && loc[j]>=0; j++){
	  if(loc[j]>maxpxl)
	    maxpxl=loc[j];
	}
      }
    }
  }
  if(maxpxl>info->npxlx*info->npxly)
    maxpxl=info->npxlx*info->npxly;
  return maxpxl;
}

int waitPixels(int cnt,threadStruct *threadInfo){
  int rt=0;
  globalStruct *glob=threadInfo->globals;
  infoStruct *info=threadInfo->info;
  if(glob->camWaitPixelsFn!=NULL){
    rt=(*glob->camWaitPixelsFn)(cnt,info->cam,glob->camHandle);

  }
  return rt==1;
}



/**
   Called by each thread when they are ready to process the next subap.  They block here until pixels for this subap become available for them to process.
   @return int 0 on success, -1 if frame finished, 1 on error.
 */
int waitNextSubaps(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  globalStruct *glob=threadInfo->globals;
  int endFrame=0,i,cnt=0,npxls=0;
  int centindx,subindx,skip=0;
  if(glob->subapAllocationArr==NULL){
    //Any thread can process any subap for its camera.
    //Threads run in order, and are returned a subap as it becomes available...
    //first block in the thread queue until it is my turn.
    if(pthread_mutex_lock(&info->subapMutex))
      printf("pthread_mutex_lock error in waitNextSubaps: %s\n",strerror(errno));
    //work out which is the next subap to do...
    //if(info->frameFinished[threadInfo->mybuf]==1){091109
    if(info->frameFinished==1){
      pthread_mutex_unlock(&info->subapMutex);
      return -1;
    }
    
    while(cnt==0 && info->subindx<info->nsub){
      npxls=0;
      while(info->subindx<info->nsub && info->subflag[info->subindx]==0)
	info->subindx++;//skip unused subaps...
      i=0;
      while(i<info->nsub-info->subindx && (info->subflag[info->subindx+i]==0 || glob->pxlCnt[info->subindx+info->subCumIndx]==glob->pxlCnt[info->subindx+i+info->subCumIndx]) ){//iterate over subaps that require same number of pixels... (this replaces the old nsubapsTogether, but is a bit better and doesn't assume a square subap geometry)
	
	//for(i=0; i<info->nsubapsTogether && i<info->nsub-info->subindx; i++){
	cnt+=info->subflag[info->subindx+i];
	//see how many pixels this subap requires..., store the max so far.
	if(info->subflag[info->subindx+i]==1 && glob->pxlCnt[info->subindx+i+info->subCumIndx]>npxls)
	  npxls=glob->pxlCnt[info->subindx+i+info->subCumIndx];
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
      
      //if(info->windowMode==WINDOWMODE_ADAPTIVE || info->windowMode==WINDOWMODE_GLOBAL){
      //  npxls=updateSubapLocation(threadInfo);
      //}
      npxls=computePixelsRequired(threadInfo);
      endFrame=waitPixels(npxls,threadInfo);//info->pxlCnt[threadInfo->cursubindx+info->nsubapsTogether-1]+extrapxl,threadInfo);
      dprintf("waited pixels\n");
      if(endFrame){
	writeErrorVA(threadInfo->globals->rtcErrorBuf,CAMGETERROR,threadInfo->globals->thisiter,"Error - getting camera pixels");
      }
    }
    //then release mutex so that next thread can wait for pixels, and reset this sem, so that this thread will wait next time...
    pthread_mutex_unlock(&info->subapMutex);//unlock the mutex, allowing the next thread to continue
    dprintf("freed muted\n");
  }else{//threads are assigned the subaps that they should process.
    //So, no need to get a mutex here.
    //Note, need to set frameFinished when all threads have processed their subaps.
    subindx=threadInfo->cursubindx+threadInfo->nsubapsProcessing;
    centindx=threadInfo->centindx+threadInfo->nsubapsDoing*2;
    while(cnt==0 && subindx<info->nsub+info->subCumIndx){
      npxls=0;
      while(subindx<info->nsub+info->subCumIndx && (glob->subapAllocationArr[subindx]!=threadInfo->threadno || glob->subapFlagArr[subindx]==0)){
	subindx++;//skip unused subaps, or ones for other threads.
	skip+=glob->subapFlagArr[subindx];
      }
      i=0;
      while(i+subindx<info->nsub+info->subCumIndx && (glob->subapFlagArr[subindx+i]==0 || (glob->pxlCnt[subindx+i]==glob->pxlCnt[subindx] && glob->subapAllocationArr[subindx]==threadInfo->threadno))){
	cnt+=glob->subapFlagArr[subindx+i];
	if(glob->subapFlagArr[subindx+i]==1 && glob->pxlCnt[subindx+i]>npxls)
	  npxls=glob->pxlCnt[subindx+i];
	i++;
      }
      if(cnt==0)
	subindx+=i;
    }
    if(subindx>=info->nsub+info->subCumIndx){//got to end of processing
      //info->subindx=0;
      //info->centindx=0;
      endFrame=-1;
      threadInfo->frameFinished=1;
      threadInfo->cursubindx=info->subCumIndx;
      threadInfo->nsubapsProcessing=0;
      threadInfo->nsubapsDoing=0;
      threadInfo->centindx=info->centCumIndx;
    }else{
      threadInfo->centindx+=skip*2;//skip the number assigned to other threads
      threadInfo->cursubindx=subindx;
      threadInfo->nsubapsDoing=cnt;//no of used subaps to do.
      threadInfo->nsubapsProcessing=i;//no of subaps to do.
      threadInfo->nsubs=threadInfo->centindx+2*cnt-info->centCumIndx;
      if(subindx+i>info->nsub+info->subCumIndx){
	threadInfo->frameFinished=1;
      }
      npxls=computePixelsRequired(threadInfo);
      endFrame=waitPixels(npxls,threadInfo);
      if(endFrame)
	writeErrorVA(glob->rtcErrorBuf,CAMGETERROR,glob->thisiter,"Error - getting camera pixels");
    }
  }
  return endFrame;
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
  if(!p->noPrePostThread)
    pthread_mutex_lock(&glob->libraryMutex);
  if(glob->camFrameFinishedFn!=NULL)
    (*p->camFrameFinishedFn)(glob->camHandle,p->pxlCentInputError);
  if(glob->calibrateFrameFinishedFn!=NULL)
    (*p->calibrateFrameFinishedFn)(glob->calibrateHandle,p->pxlCentInputError);
  if(glob->centFrameFinishedFn!=NULL)
    (*p->centFrameFinishedFn)(glob->centHandle,p->pxlCentInputError);
  if(p->reconFrameFinishedFn!=NULL)
    (*p->reconFrameFinishedFn)(glob->reconHandle,/*glob->arrays->dmCommand,*/p->pxlCentInputError);
  if(!p->noPrePostThread)
    pthread_mutex_unlock(&glob->libraryMutex);
  if(p->pxlCentInputError==0){
    //we send the dmCommand here, because sendActuators can alter this depending on the bleed gain... and we don't want that to be seen - this is the result of the reconstruction process only...
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
	if(glob->figureframenoSize>0)
	  glob->figureFrame=glob->figureframeno[0];//copy into the second buffer...
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
  if(*p->closeLoop){
    //send actuators direct to the mirror.
    if(!p->noPrePostThread)
      pthread_mutex_lock(&glob->libraryMutex);
    if(p->mirrorHandle!=NULL && glob->mirrorLib!=NULL && p->mirrorSendFn!=NULL)
      //(*p->mirrorSendFn)(p->mirrorHandle,nacts,actsSent,p->thisiter);
      p->nclipped=(*p->mirrorSendFn)(p->mirrorHandle,nacts,dmCommand,p->thisiter,p->timestamp,p->pxlCentInputError);
    if(!p->noPrePostThread)
      pthread_mutex_unlock(&glob->libraryMutex);
    /*
      }
      }*/
  }else{
    //It is necessary in this case to let the recon library know that the actuator values aren't being used, so that it can reset itself...
    resetRecon=1;
  }
  if(resetRecon){
    if(!p->noPrePostThread)
      pthread_mutex_lock(&glob->libraryMutex);
    if(p->camOpenLoopFn!=NULL)
      (*p->camOpenLoopFn)(glob->camHandle);
    if(p->calibrateOpenLoopFn!=NULL)
      (*p->calibrateOpenLoopFn)(glob->calibrateHandle);
    if(p->centOpenLoopFn!=NULL)
      (*p->centOpenLoopFn)(glob->centHandle);
    if(p->reconOpenLoopFn!=NULL)
      (*p->reconOpenLoopFn)(glob->reconHandle);
    if(!p->noPrePostThread)
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
      if(*p->closeLoop){
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
  int pos,i;
  memset(glob->statusBuf,0,STATUSBUFSIZE);
  pos=snprintf(glob->statusBuf,STATUSBUFSIZE,"%d+1 threads\nClipped: %d\nIteration: %d/%d\nMax time %gs at iter %d\nFrame time %gs (%gHz)\n%s\nFS: %u\n%s",glob->nthreads,glob->nclipped,glob->thisiter,glob->niters,glob->maxtime,glob->maxtimeiter,glob->frameTime,1/glob->frameTime,paused==0?"Running...":"Paused...",glob->figureFrame,closeLoop?"Loop closed":"Loop open");
  pos+=snprintf(&glob->statusBuf[pos],STATUSBUFSIZE-pos-1,glob->camHandle==NULL?"\nNo cam:":"\nCam:");
  for(i=0; i<glob->camframenoSize && pos<STATUSBUFSIZE; i++){
    pos+=snprintf(&glob->statusBuf[pos],STATUSBUFSIZE-pos-1," %u",glob->camframeno[i]);
  }
  pos+=snprintf(&glob->statusBuf[pos],STATUSBUFSIZE-pos-1,glob->calibrateHandle==NULL?"\nNo calibration:":"\nCalibration:");
  for(i=0; i<glob->calframenoSize && pos<STATUSBUFSIZE; i++){
    pos+=snprintf(&glob->statusBuf[pos],STATUSBUFSIZE-pos-1," %u",glob->calframeno[i]);
  }
  pos+=snprintf(&glob->statusBuf[pos],STATUSBUFSIZE-pos-1,glob->centHandle==NULL?"\nNo centroider:":"\nCentroider:");
  for(i=0; i<glob->centframenoSize && pos<STATUSBUFSIZE; i++){
    pos+=snprintf(&glob->statusBuf[pos],STATUSBUFSIZE-pos-1," %u",glob->centframeno[i]);
  }
  pos+=snprintf(&glob->statusBuf[pos],STATUSBUFSIZE-pos-1,glob->reconLib==NULL?"\nNo reconstructor:":"\nReconstructor:");
  for(i=0; i<glob->reconframenoSize && pos<STATUSBUFSIZE; i++){
    pos+=snprintf(&glob->statusBuf[pos],STATUSBUFSIZE-pos-1," %u",glob->reconframeno[i]);
  }
  pos+=snprintf(&glob->statusBuf[pos],STATUSBUFSIZE-pos-1,glob->mirrorHandle==NULL?"\nNo mirror:":"\nMirror:");
  for(i=0; i<glob->mirrorframenoSize && pos<STATUSBUFSIZE; i++){
    pos+=snprintf(&glob->statusBuf[pos],STATUSBUFSIZE-pos-1," %u",glob->mirrorframeno[i]);
  }
  pos+=snprintf(&glob->statusBuf[pos],STATUSBUFSIZE-pos-1,glob->figureHandle==NULL?"\nNo figure:":"\nFigure:");
  for(i=0; i<glob->figureframenoSize && pos<STATUSBUFSIZE; i++){
    pos+=snprintf(&glob->statusBuf[pos],STATUSBUFSIZE-pos-1," %u",glob->figureframeno[i]);
  }
  pos+=snprintf(&glob->statusBuf[pos],STATUSBUFSIZE-pos-1,glob->bufferHandle==NULL?"\nNo buffer lib:":"\nBuffer lib:");
  for(i=0; i<glob->bufferframenoSize && pos<STATUSBUFSIZE; i++){
    pos+=snprintf(&glob->statusBuf[pos],STATUSBUFSIZE-pos-1," %u",glob->bufferframeno[i]);
  }


  return 0;
}

int setThreadAffinity(globalStruct *glob,int n,int ncpu){
  int i;
  cpu_set_t mask;
  int threadAffinity;
  int *threadAffinityList=glob->threadAffinityList;
  int *threadPriorityList=glob->threadPriorityList;
  int *threadAffinityListPrev=glob->threadAffinityListPrev;
  int *threadPriorityListPrev=glob->threadPriorityListPrev;
  struct sched_param param;
  if(threadAffinityList==NULL)
    threadAffinity=-1;
  else
    threadAffinity=threadAffinityList[n];
  if(threadAffinity!=threadAffinityListPrev[n]){//changed, so set it.
    CPU_ZERO(&mask);
    for(i=0; i<ncpu; i++){
      if(((threadAffinity)>>i)&1){
	CPU_SET(i,&mask);
      }
    }
    //printf("Thread affinity %d\n",threadAffinity&0xff);
    if(sched_setaffinity(0,sizeof(cpu_set_t),&mask))
      printf("Error in sched_setaffinity: %s\n",strerror(errno));
    threadAffinityListPrev[n]=threadAffinity;
  }
  if(threadPriorityList==NULL)
    param.sched_priority=(n==0)+1;
  else
    param.sched_priority=threadPriorityList[n];
  if(param.sched_priority!=threadPriorityListPrev[n]){//changed, so set it.
    //if(sched_setparam(0,&param)){
    if(sched_setscheduler(0,SCHED_RR,&param)){
      //printf("Error in sched_setparam: %s\n",strerror(errno));
    }
    if(pthread_setschedparam(pthread_self(),SCHED_RR,&param))
      printf("error in pthread_setschedparam - maybe run as root?\n");
    threadPriorityListPrev[n]=param.sched_priority;
  }
  return 0;
}



/**
   Called when a buffer swap is required.  Reads the new buffer.
   Only the first thread needs to do this.
*/

int updateBufferIndex(threadStruct *threadInfo,int warn){//,int updateIndex){
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
  if(nfound!=NBUFFERVARIABLES && warn==1){
    err=1;
    printf("Didn't find all buffer entries:\n");
    for(i=0; i<NBUFFERVARIABLES;i++){
      if(globals->bufferHeaderIndex[i]<0)
	printf("Missing %16s\n",&globals->paramNames[i*BUFNAMESIZE]);
    }
  }
  return err;
}


/**
   Called when a buffer swap is required.  Reads the new buffer.
*/
int updateBuffer(globalStruct *globals){
  //Assumes the buffer is an array, with header then data.  Header contains:
  //name (16 bytes), type(1), startaddr(4), nbytes(4), ndim(4), shape(24), lcomment(4).  Here, we find name, check that type and nbytes match what we expect, and move the pointer to startaddr (which is index in bytes from start of array).

  //This is called only by the first thread for each camera.  The
  //first thread overall has updateIndex set.
  BUFFERVARIABLEINDX i;
  int j;//,k,offset;
  //int m;
  //char *buf=threadInfo->globals->buffer[threadInfo->globals->curBuf]->buf;
  int *indx;
  int nb;
  int err=0;
  //short *tmps;
  //float *tmpf;
  struct timeval t1;
  double timestamp;
  //infoStruct *info=threadInfo->info;
  //globalStruct *globals=threadInfo->globals;
  void **values=globals->bufferValues;
  int *nbytes=globals->bufferNbytes;
  char *dtype=globals->bufferDtype;
  dprintf("updating buffer \n",);

  indx=globals->bufferHeaderIndex;

    //first get ncam... (for checking array sizes etc)
    i=NCAM;
    if(dtype[i]=='i' && nbytes[i]==4){
    if(globals->ncam!=*((int*)values[i])){
      printf("Cannont change ncam\n");
      err=-2;
    }
    }else{
      printf("ncam error\n");
      err=-2;
    }
    //now get nsubx etc...
    i=NACTS;
    if(dtype[i]=='i' && nbytes[i]==4){
      globals->nacts=*((int*)values[i]);
    }else{
      printf("nacts error\n");
      err=NACTS;
    }
    i=NSUB;
    if(dtype[i]=='i' && nbytes[i]==4*globals->ncam){
      globals->nsubList=(int*)values[i];
    }else{
      printf("nsub error\n");
      err=NSUB;
    }
    i=NPXLX;
    if(dtype[i]=='i' && nbytes[i]==4*globals->ncam){
      globals->npxlxList=(int*)values[i];
    }else{
      printf("npxlx error\n");
      err=NPXLX;
    }
    i=NPXLY;
    if(dtype[i]=='i' && nbytes[i]==4*globals->ncam){
      globals->npxlyList=(int*)values[i];
    }else{
      printf("npxly error\n");
      err=NPXLY;
    }
    globals->nsubaps=0;
    globals->totPxls=0;
    for(j=0; j<globals->ncam; j++){
      globals->nsubaps+=globals->nsubList[j];
      globals->totPxls+=globals->npxlxList[j]*globals->npxlyList[j];
    }
    i=SUBAPFLAG;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)*globals->nsubaps){
      globals->subapFlagArr=(int*)values[i];
    }else{
      printf("subapFlag error\n");
      err=SUBAPFLAG;
    }
    globals->totCents=0;
    for(j=0; j<globals->nsubaps; j++){
      globals->totCents+=globals->subapFlagArr[j];
    }
    globals->totCents*=2;
    i=CLOSELOOP;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      globals->closeLoop=((int*)values[i]);
    }else{
      printf("closeLoop error\n");
      err=CLOSELOOP;
    }
    i=OPENLOOPIFCLIP;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      globals->openLoopIfClipped=*((int*)values[i]);
    }else{
      printf("openLoopIfClip error\n");
      err=OPENLOOPIFCLIP;
    }
    i=GO;
    if(dtype[i]=='i' && nbytes[i]==4){
      globals->go=*((int*)values[i]);
    }else{
      err=i;
      printf("go error\n");
    }
    i=ACTUATORS;
    if(nbytes[i]==0){
      globals->userActs=NULL;
      globals->userActSeqLen=0;
    }else if(dtype[i]=='f' && nbytes[i]%(sizeof(float)*globals->nacts)==0){
      globals->userActs=((float*)values[i]);
      //get the number of actuator sequences...
      globals->userActSeqLen=nbytes[i]/(sizeof(float)*globals->nacts);
    }else{ 
      printf("userActs/actuators error %c %d %d\n",dtype[i],nbytes[i],(int)sizeof(float)*globals->nacts);
      err=ACTUATORS;
    }
    i=ACTUATORMASK;
    if(nbytes[i]==0){
      globals->userActsMask=NULL;
    }else if(dtype[i]=='f' && nbytes[i]%(sizeof(float)*globals->nacts)==0){
      globals->userActsMask=((float*)values[i]);
    }else{ 
      printf("userActsMask/actuatorMask error %c %d %d\n",dtype[i],nbytes[i],(int)sizeof(float)*globals->nacts);
      err=ACTUATORMASK;
    }
    i=PAUSE;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      globals->ppause=((int*)values[i]);
    }else{
      printf("pause error\n");
      globals->ppause=NULL;
      err=i;
    }
    i=PRINTTIME;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      globals->printTime=*((int*)values[i]);
      globals->maxtime=0;
    }else{
      printf("printTime error\n");
      err=i;
    }
    i=SUBAPLOCTYPE;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      globals->subapLocationType=*((int*)values[i]);
    }else{
      printf("subapLocType error\n");
      globals->subapLocationType=0;
      err=i;
    }
    i=SUBAPLOCATION;
    if(globals->subapLocationType==0 && dtype[i]=='i' && nbytes[i]==globals->nsubaps*6*sizeof(int)){
      globals->realSubapLocation=(int*)values[i];
      globals->maxPxlPerSubap=6;
    }else if(globals->subapLocationType==1 && dtype[i]=='i'){
      globals->maxPxlPerSubap=nbytes[i]/sizeof(int)/globals->nsubaps;
      if(nbytes[i]==globals->maxPxlPerSubap*sizeof(int)*globals->nsubaps){
	globals->realSubapLocation=(int*)values[i];
      }else{
	printf("subapLocation error (wrong size %d for subapLocationType==1)\n",nbytes[i]);
	err=i;
      }
    }else{
      printf("subapLocation error\n");
      err=i;
    }
    i=PXLCNT;
    if(dtype[i]=='i' && nbytes[i]==globals->nsubaps*sizeof(int)){
      globals->pxlCnt=(int*)values[i];
    }else{
      printf("pxlCnt error\n");
      err=i;
    }
    i=WINDOWMODE;
    nb=nbytes[i];
    //info->resetAdaptiveWindows=0;
    if(nb!=0 && dtype[i]=='s'){
      if(strncmp(values[i],"basic",nb)==0){
	globals->windowMode=WINDOWMODE_BASIC;
      }else if(strncmp(values[i],"adaptive",nb)==0){
	if(globals->windowMode!=WINDOWMODE_ADAPTIVE){
	  globals->windowMode=WINDOWMODE_ADAPTIVE;
	  //if(updateIndex)
	  //info->resetAdaptiveWindows=1;//all threads can set this, but only the first overall will use it...
	}
      }else if(strncmp(values[i],"global",nb)==0){
	if(globals->windowMode!=WINDOWMODE_GLOBAL){
	  globals->windowMode=WINDOWMODE_GLOBAL;
	  //if(updateIndex)
	  //info->resetAdaptiveWindows=1;//all threads can set this, but only the first overall will use it...
	}
      }else{
	globals->windowMode=WINDOWMODE_ERROR;
	printf("windowMode string error (unrecognised)\n");
      }
    }else{
      err=i;
      printf("windowMode error\n");
    }

    i=THREADAFFINITY;
    nb=nbytes[i];
    if(nb==0){
      globals->threadAffinityList=NULL;
    }else if(dtype[i]=='i' && nbytes[i]==sizeof(int)*(globals->nthreads+1)){
      globals->threadAffinityList=(int*)values[i];
    }else{
      printf("threadAffinity error\n");
      err=THREADAFFINITY;
    }
    i=THREADPRIORITY;
    nb=nbytes[i];
    if(nb==0){
      globals->threadPriorityList=NULL;
    }else if(dtype[i]=='i' && nbytes[i]==sizeof(int)*(globals->nthreads+1)){
      globals->threadPriorityList=(int*)values[i];
    }else{
      printf("threadPriority error\n");
      err=THREADPRIORITY;
    }
    i=DELAY;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      globals->delay=*((int*)values[i]);
    }else{
      err=i;
      printf("delay error\n");
    }
    i=MAXCLIPPED;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      globals->maxClipped=*((int*)values[i]);
    }else{
      err=i;
      printf("maxClipped error\n");
    }
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
  

    i=CAMERASOPEN;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      globals->camerasOpen=((int*)values[i]);
      //*((int*)values[i])=0;
    }else{
      err=i;
      printf("camerasOpen error\n");
    }
    i=CAMERAPARAMS;
    nb=nbytes[i];
    if(nb==0){
      globals->cameraParams=NULL;
      globals->cameraParamsCnt=0;
    }else if(dtype[i]=='i'){
      globals->cameraParams=((int*)values[i]);
      globals->cameraParamsCnt=nb/sizeof(int);
    }else{
      err=i;
      printf("cameraParams error\n");
    }
    
    i=SLOPEOPEN;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      globals->centOpen=((int*)values[i]);
      //*((int*)values[i])=0;
    }else{
      err=i;
      printf("centOpen error\n");
    }
    i=SLOPEPARAMS;
    nb=nbytes[i];
    if(nb==0){
      globals->centParams=NULL;
      globals->centParamsCnt=0;
    }else if(dtype[i]=='i'){
      globals->centParams=((int*)values[i]);
      globals->centParamsCnt=nb/sizeof(int);
    }else{
      err=i;
      printf("centParams error\n");
    }
    
    i=MIRRORPARAMS;
    nb=nbytes[i];
    if(nb==0){
      globals->mirrorParams=NULL;
      globals->mirrorParamsCnt=0;
    }else if(dtype[i]=='i'){
      globals->mirrorParams=((int*)values[i]);
      globals->mirrorParamsCnt=nb/sizeof(int);
    }else{
      err=i;
      printf("mirrorParams error\n");
    }
    i=MIRROROPEN;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      globals->mirrorOpen=((int*)values[i]);
      //*((int*)values[i])=0;
    }else{
      err=i;
      printf("mirrorOpen error\n");
    }

    //This stuff is for when using as a figure sensor...
    i=FIGUREOPEN;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      globals->figureOpen=((int*)values[i]);
      //*((int*)values[i])=0;
    }else{
      err=i;
      printf("figureOpen error\n");
    }
    i=FIGUREPARAMS;
    nb=nbytes[i];
    if(nb==0){
      globals->figureParams=NULL;
      globals->figureParamsCnt=0;
    }else if(dtype[i]=='i'){
      globals->figureParams=((int*)values[i]);
      globals->figureParamsCnt=nb/sizeof(int);
    }else{
      err=i;
      printf("figureParams error\n");
    }
    i=RECONLIBOPEN;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      globals->reconlibOpen=((int*)values[i]);
      //*((int*)values[i])=0;
    }else{
      err=i;
      printf("reconlibOpen error\n");
    }
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
  
    i=NSTEPS;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      globals->nsteps=*((int*)values[i]);
    }else{
      err=i;
      printf("nsteps error\n");
    }
    i=ADDACTUATORS;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      globals->addUserActs=*((int*)values[i]);
    }else{ 
      printf("addActuators error\n");
      err=ADDACTUATORS;
    }
    i=ACTSEQUENCE;
    if(nbytes[i]==0){
      globals->userActSeq=NULL;
    }else if(dtype[i]=='i' && nbytes[i]==sizeof(int)*globals->userActSeqLen){
      globals->userActSeq=((int*)values[i]);
    }else{ 
      printf("userActSeq/actSequence error %d %d\n",nbytes[i],(int)(globals->userActSeqLen*sizeof(int)));
      err=ACTSEQUENCE;
    }
    i=RECORDCENTS;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      globals->recordCents=((int*)values[i]);
    }else{ 
      printf("recordCents error\n");
      err=RECORDCENTS;
    }
    i=AVERAGEIMG;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      globals->averageImg=((int*)values[i]);
      globals->nAvImg=*globals->averageImg;
    }else{ 
      printf("averageImg error\n");
      err=AVERAGEIMG;
    }
    i=AVERAGECENT;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      globals->averageCent=((int*)values[i]);
      globals->nAvCent=*globals->averageCent;
    }else{ 
      printf("averageCent error\n");
      err=AVERAGECENT;
    }
    i=FIGUREGAIN;
    nb=nbytes[i];
    if(nb==0){
      globals->figureGain=1;
      globals->figureGainArr=NULL;
    }else if(nb==sizeof(float) && dtype[i]=='f'){
      globals->figureGain=*((float*)values[i]);
      globals->figureGainArr=NULL;
    }else if(nb==sizeof(float)*globals->nacts && dtype[i]=='f'){
      globals->figureGain=0;
      globals->figureGainArr=((float*)values[i]);
    }else{
      printf("figureGain error\n");
      err=i;
      globals->figureGainArr=NULL;
    }
    i=VERSION;
    nb=nbytes[i];
    if(nb>0 && dtype[i]=='s'){
      //write the versions into here.
      
      strncpy(values[i],globals->mainGITID,nb-1);
      //((char*)(values[i]))[nb-1]='\0';
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
    i=SLOPENAME;
    nb=nbytes[i];
    if(nb==0){
      globals->centName=NULL;
    }else if(dtype[i]=='s'){
      globals->centName=values[i];
    }else{
      printf("centName error\n");
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
    i=CALIBRATEOPEN;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      globals->calibrateOpen=((int*)values[i]);
      //*((int*)values[i])=0;
    }else{
      err=i;
      printf("calibrateOpen error\n");
    }

    i=CALIBRATENAME;
    nb=nbytes[i];
    if(nb==0){
      globals->calibrateName=NULL;
    }else if(dtype[i]=='s'){
      globals->calibrateName=values[i];
    }else{
      printf("calibrateName error\n");
      globals->calibrateName=NULL;
      err=i;
    }
    i=CALIBRATEPARAMS;
    nb=nbytes[i];
    if(nb==0){
      globals->calibrateParams=NULL;
      globals->calibrateParamsCnt=0;
    }else if(dtype[i]=='i'){
      globals->calibrateParams=((int*)values[i]);
      globals->calibrateParamsCnt=nb/sizeof(int);
    }else{
      err=i;
      printf("calibrateParams error\n");
    }

    i=BUFFEROPEN;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      globals->bufferlibOpen=((int*)values[i]);
      //*((int*)values[i])=0;
    }else{
      err=i;
      printf("bufferOpen error\n");
    }

    i=BUFFERNAME;
    nb=nbytes[i];
    if(nb==0){
      globals->bufferName=NULL;
    }else if(dtype[i]=='s'){
      globals->bufferName=values[i];
    }else{
      printf("bufferName error\n");
      globals->bufferName=NULL;
      err=i;
    }
    i=BUFFERPARAMS;
    nb=nbytes[i];
    if(nb==0){
      globals->bufferParams=NULL;
      globals->bufferParamsCnt=0;
    }else if(dtype[i]=='i'){
      globals->bufferParams=((int*)values[i]);
      globals->bufferParamsCnt=nb/sizeof(int);
    }else{
      err=i;
      printf("bufferParams error\n");
    }
    i=BUFFERUSESEQ;
    if(nbytes[i]==sizeof(int) && dtype[i]=='i'){
      globals->bufferUseSeq=*((int*)values[i]);
    }else{
      printf("bufferUseSeq error\n");
      err=i;
      globals->bufferUseSeq=0;
    }
    i=PRINTUNUSED;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      globals->printUnused=*((int*)values[i]);
    }else{
      printf("printUnused error\n");
      err=i;
    }
    i=ITERSOURCE;
    if(nbytes[i]==0){
      globals->itersource=0;
      globals->iterindex=0;
    }else if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      j=*((int*)values[i]);
      globals->itersource=j&0xffff;
      globals->iterindex=(j>>16)&0xffff;
    }else{
      globals->itersource=0;
      globals->iterindex=0;
      printf("warning - incorrect iterSource\n");
    }
    i=NOPREPOSTTHREAD;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      globals->noPrePostThread=*((int*)values[i]);
    }else{
      printf("warning - noPrePostThread incorrect\n");
      globals->noPrePostThread=0;
    }
    i=SUBAPALLOCATION;
    if(dtype[i]=='i' && nbytes[i]==globals->nsubaps*sizeof(int)){
      globals->subapAllocationArr=(int*)values[i];
    }else{
      if(nbytes[i]!=0){
	printf("Error - subapAllocation\n");
	err=i;
      }
      globals->subapAllocationArr=NULL;
    }
    i=ADAPWINSHIFTCNT;
    if(dtype[i]=='i' && nbytes[i]==globals->nsubaps*2*sizeof(int)){
      globals->adapWinShiftCnt=(int*)values[i];
    }else{
      if(nbytes[i]!=0){
	printf("Error - adapWinShiftCnt\n");
	err=i;
      }
      globals->adapWinShiftCnt=NULL;
    }
    return err;
}
void updateInfo(threadStruct *threadInfo){
  infoStruct *info=threadInfo->info;
  globalStruct *globals=threadInfo->globals;
  int j;
  info->pause=*globals->ppause;
  info->nsub=globals->nsubList[info->cam];
  info->npxlx=globals->npxlxList[info->cam];
  info->npxly=globals->npxlyList[info->cam]; 
  //compute number of subaps/pixels up to this point...
  info->subCumIndx=0;
  info->npxlCum=0;
  for(j=0;j<info->cam; j++){
    info->subCumIndx+=globals->nsubList[j];
    info->npxlCum+=globals->npxlxList[j]*globals->npxlyList[j];
  }
  //and the total number
  info->subflag=&(globals->subapFlagArr[info->subCumIndx]);

  //and work out the cumulative cent index...
  info->centCumIndx=0;//sum of all subaps used up to this camera.
  //info->centCumIndx=cblas_sasum(info->subCumIndx,info->subapFlagArr,1);
  for(j=0; j<info->subCumIndx; j++){//sum the subaps...
    info->centCumIndx+=globals->subapFlagArr[j];
  }
  info->centCumIndx*=2;
  
}

/**
   Opens the dynamic library for image calibration...
*/
int updateCalibrateLibrary(globalStruct *glob){
  int open=0,err=0,doneParams=0;
  if(glob->calibrateNameOpen==NULL){
    if(glob->calibrateName!=NULL && *glob->calibrateOpen==1){
      glob->calibrateNameOpen=strdup(glob->calibrateName);
      open=1;
    }
  }else{//already one open
    if(glob->calibrateName==NULL || *glob->calibrateOpen==0 || glob->go==0){
      free(glob->calibrateNameOpen);
      glob->calibrateNameOpen=NULL;
      *glob->calibrateOpen=0;
      open=1;
    }else{
      if(strcmp(glob->calibrateName,glob->calibrateNameOpen)){//different name...
	free(glob->calibrateNameOpen);
	glob->calibrateNameOpen=strdup(glob->calibrateName);
	open=1;
      }
    }
  }
  if(open){
    //first close existing, if it is open...
    if(glob->calibrateLib!=NULL){
      //close existing library.
      (*glob->calibrateCloseFn)(&glob->calibrateHandle);
      if(dlclose(glob->calibrateLib)!=0){
	printf("Failed to close calibrate library - ignoring\n");
      }
      glob->calibrateLib=NULL;
    }
    //and then open the new one.
    if(glob->calibrateNameOpen!=NULL && glob->go!=0){
      if((glob->calibrateLib=dlopen(glob->calibrateNameOpen,RTLD_LAZY))==NULL){
	printf("Failed to open calibrate library %s: %s\n",glob->calibrateNameOpen,dlerror());
	err=1;
      }else{//now get the symbols...
	int nsym=0;
	if((*(void**)(&glob->calibrateOpenFn)=dlsym(glob->calibrateLib,"calibrateOpen"))==NULL){
	  printf("dlsym failed for calibrateOpen\n");
	  err=1;
	}else{nsym++;}
	if((*(void**)(&glob->calibrateCloseFn)=dlsym(glob->calibrateLib,"calibrateClose"))==NULL){
	  printf("dlsym failed for calibrateClose\n");
	  err=1;
	}else{nsym++;}
	if((*(void**)(&glob->calibrateNewParamFn)=dlsym(glob->calibrateLib,"calibrateNewParam"))==NULL){
	  printf("dlsym failed for calibrateNewParam (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->calibrateNewFrameFn)=dlsym(glob->calibrateLib,"calibrateNewFrame"))==NULL){
	  printf("dlsym failed for calibrateNewFrame (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->calibrateNewFrameSyncFn)=dlsym(glob->calibrateLib,"calibrateNewFrameSync"))==NULL){
	  printf("dlsym failed for calibrateNewFrameSync (non-fatal)\n");
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
	if((*(void**)(&glob->calibrateCompleteFn)=dlsym(glob->calibrateLib,"calibrateComplete"))==NULL){
	  printf("dlsym failed for calibrateComplete (non-fatal)\n");
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
	if((err=(*glob->calibrateOpenFn)(glob->calibrateNameOpen,glob->calibrateParamsCnt,glob->calibrateParams,glob->buffer[glob->curBuf],glob->rtcErrorBuf,glob->shmPrefix,glob->arrays,&glob->calibrateHandle,glob->nthreads,glob->thisiter,&glob->calframeno,&glob->calframenoSize))){
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
      *glob->calibrateOpen=0;
      glob->calibrateCloseFn=NULL;
      glob->calibrateOpenFn=NULL;
      glob->calibrateNewParamFn=NULL;
      glob->calibrateNewFrameFn=NULL;
      glob->calibrateNewFrameSyncFn=NULL;
      glob->calibrateStartFrameFn=NULL;
      glob->calibrateNewSubapFn=NULL;
      glob->calibrateEndFrameFn=NULL;
      glob->calibrateFrameFinishedFn=NULL;
      glob->calibrateFrameFinishedSyncFn=NULL;
      glob->calibrateOpenLoopFn=NULL;
      glob->calibrateCompleteFn=NULL;
    }
  }
  if(err==0 && glob->calibrateNewParamFn!=NULL && doneParams==0)
    err=(*glob->calibrateNewParamFn)(glob->calibrateHandle,glob->buffer[glob->curBuf],glob->thisiter,glob->arrays);
  return err;



}


/**
   Opens the dynamic library for slope computation...
*/
int updateCentLibrary(globalStruct *glob){
  int open=0,err=0,doneParams=0;
  if(glob->centNameOpen==NULL){
    if(glob->centName!=NULL && *glob->centOpen==1){
      glob->centNameOpen=strdup(glob->centName);
      open=1;
    }
  }else{//already one open
    if(glob->centName==NULL || *glob->centOpen==0 || glob->go==0){
      free(glob->centNameOpen);
      glob->centNameOpen=NULL;
      *glob->centOpen=0;
      open=1;
    }else{
      if(strcmp(glob->centName,glob->centNameOpen)){//different name...
	free(glob->centNameOpen);
	glob->centNameOpen=strdup(glob->centName);
	open=1;
      }
    }
  }
  if(open){
    //first close existing, if it is open...
    if(glob->centLib!=NULL){
      //close existing library.
      if(glob->centCloseFn!=NULL)
	(*glob->centCloseFn)(&glob->centHandle);
      if(dlclose(glob->centLib)!=0){
	printf("Failed to close cent library - ignoring\n");
      }
      glob->centLib=NULL;
    }
    //and then open the new one.
    if(glob->centNameOpen!=NULL && glob->go!=0){
      if((glob->centLib=dlopen(glob->centNameOpen,RTLD_LAZY))==NULL){
	printf("Failed to open cent library %s: %s\n",glob->centNameOpen,dlerror());
	err=1;
      }else{//now get the symbols...
	int nsym=0;
	if((*(void**)(&glob->centOpenFn)=dlsym(glob->centLib,"slopeOpen"))==NULL){
	  printf("dlsym failed for slopeOpen\n");
	  err=1;
	}else{nsym++;}
	if((*(void**)(&glob->centCloseFn)=dlsym(glob->centLib,"slopeClose"))==NULL){
	  printf("dlsym failed for slopeClose\n");
	  err=1;
	}else{nsym++;}
	if((*(void**)(&glob->centNewParamFn)=dlsym(glob->centLib,"slopeNewParam"))==NULL){
	  printf("dlsym failed for slopeNewParam (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->centNewFrameFn)=dlsym(glob->centLib,"slopeNewFrame"))==NULL){
	  printf("dlsym failed for slopeNewFrame (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->centNewFrameSyncFn)=dlsym(glob->centLib,"slopeNewFrameSync"))==NULL){
	  printf("dlsym failed for slopeNewFrameSync (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->centStartFrameFn)=dlsym(glob->centLib,"slopeStartFrame"))==NULL){
	  printf("dlsym failed for slopeStartFrame (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->centCalcSlopeFn)=dlsym(glob->centLib,"slopeCalcSlope"))==NULL){
	  printf("dlsym failed for slopeCalcSlope (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->centEndFrameFn)=dlsym(glob->centLib,"slopeEndFrame"))==NULL){
	  printf("dlsym failed for slopeEndFrame (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->centFrameFinishedFn)=dlsym(glob->centLib,"slopeFrameFinished"))==NULL){
	  printf("dlsym failed for slopeFrameFinished (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->centFrameFinishedSyncFn)=dlsym(glob->centLib,"slopeFrameFinishedSync"))==NULL){
	  printf("dlsym failed for slopeFrameFinishedSync (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->centOpenLoopFn)=dlsym(glob->centLib,"slopeOpenLoop"))==NULL){
	  printf("dlsym failed for slopeOpenLoop (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->centCompleteFn)=dlsym(glob->centLib,"slopeComplete"))==NULL){
	  printf("dlsym failed for slopeComplete (non-fatal)\n");
	}else{nsym++;}
	if(err!=0 || nsym==0){//close the dll... - either error, or no symbols found.
	  if(glob->centLib!=NULL  && dlclose(glob->centLib)!=0){
	    printf("Failed to close slope library - ignoring\n");
	  }
	  glob->centLib=NULL;
	}
      }
      if(glob->centLib!=NULL){//do initialisation...
	doneParams=1;//the init function will do parameters...
	if((err=(*glob->centOpenFn)(glob->centNameOpen,glob->centParamsCnt,glob->centParams,glob->buffer[glob->curBuf],glob->rtcErrorBuf,glob->shmPrefix,glob->arrays,&glob->centHandle,glob->ncam,glob->nthreads,glob->thisiter,&glob->centframeno,&glob->centframenoSize,glob->totCents))){
	  printf("Error calling slopeOpen function\n");
	  if(dlclose(glob->centLib)!=0){
	    printf("Failed to close slope library - ignoring\n");
	  }
	  glob->centLib=NULL;
	}
      }
      if(glob->centLib==NULL){
	free(glob->centNameOpen);
	glob->centNameOpen=NULL;
      }
	  
    }
    if(glob->centLib==NULL){
      *glob->centOpen=0;
      glob->centCloseFn=NULL;
      glob->centOpenFn=NULL;
      glob->centNewParamFn=NULL;
      glob->centNewFrameFn=NULL;
      glob->centNewFrameSyncFn=NULL;
      glob->centStartFrameFn=NULL;
      glob->centCalcSlopeFn=NULL;
      glob->centEndFrameFn=NULL;
      glob->centFrameFinishedFn=NULL;
      glob->centFrameFinishedSyncFn=NULL;
      glob->centOpenLoopFn=NULL;
      glob->centCompleteFn=NULL;
    }
  }
  if(err==0 && glob->centNewParamFn!=NULL && doneParams==0)
    err=(*glob->centNewParamFn)(glob->centHandle,glob->buffer[glob->curBuf],glob->thisiter,glob->arrays,glob->totCents);
  return err;



}



/**
   Opens the dynamic library for reconstructor...
*/
int updateReconLibrary(globalStruct *glob){
  int open=0,err=0,doneParams=0;
  if(glob->reconNameOpen==NULL){
    if(glob->reconName!=NULL && *glob->reconlibOpen==1){
      glob->reconNameOpen=strdup(glob->reconName);
      open=1;
    }
  }else{//already one open
    if(glob->reconName==NULL || *glob->reconlibOpen==0 || glob->go==0){
      free(glob->reconNameOpen);
      glob->reconNameOpen=NULL;
      *glob->reconlibOpen=0;
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
      (*glob->reconCloseFn)(&glob->reconHandle);
      if(dlclose(glob->reconLib)!=0){
	printf("Failed to close recon library - ignoring\n");
      }
      glob->reconLib=NULL;
    }
    //and then open the new one.
    if(glob->reconNameOpen!=NULL && glob->go!=0){
      if((glob->reconLib=dlopen(glob->reconNameOpen,RTLD_LAZY))==NULL){
	printf("Failed to open recon library %s: %s\n",glob->reconNameOpen,dlerror());
	err=1;
      }else{//now get the symbols...
	int nsym=0;
	if((*(void**)(&glob->reconOpenFn)=dlsym(glob->reconLib,"reconOpen"))==NULL){
	  printf("dlsym failed for reconOpen\n");
	  err=1;
	}else{nsym++;}
	if((*(void**)(&glob->reconCloseFn)=dlsym(glob->reconLib,"reconClose"))==NULL){
	  printf("dlsym failed for reconClose\n");
	  err=1;
	}else{nsym++;}
	if((*(void**)(&glob->reconNewParamFn)=dlsym(glob->reconLib,"reconNewParam"))==NULL){
	  printf("dlsym failed for reconNewParam (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->reconNewFrameFn)=dlsym(glob->reconLib,"reconNewFrame"))==NULL){
	  printf("dlsym failed for reconNewFrame (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->reconNewFrameSyncFn)=dlsym(glob->reconLib,"reconNewFrameSync"))==NULL){
	  printf("dlsym failed for reconNewFrameSync (nonfatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->reconStartFrameFn)=dlsym(glob->reconLib,"reconStartFrame"))==NULL){
	  printf("dlsym failed for reconStartFrame (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->reconNewSlopesFn)=dlsym(glob->reconLib,"reconNewSlopes"))==NULL){
	  printf("dlsym failed for reconNewSlopes (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->reconEndFrameFn)=dlsym(glob->reconLib,"reconEndFrame"))==NULL){
	  printf("dlsym failed for reconEndFrame (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->reconFrameFinishedFn)=dlsym(glob->reconLib,"reconFrameFinished"))==NULL){
	  printf("dlsym failed for reconFrameFinished (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->reconFrameFinishedSyncFn)=dlsym(glob->reconLib,"reconFrameFinishedSync"))==NULL){
	  printf("dlsym failed for reconFrameFinishedSync (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->reconOpenLoopFn)=dlsym(glob->reconLib,"reconOpenLoop"))==NULL){
	  printf("dlsym failed for reconOpenLoop (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->reconCompleteFn)=dlsym(glob->reconLib,"reconComplete"))==NULL){
	  printf("dlsym failed for reconComplete (non-fatal)\n");
	}else{nsym++;}
	if(err!=0 || nsym==0){//close the dll...
	  if(glob->reconLib!=NULL  && dlclose(glob->reconLib)!=0){
	    printf("Failed to close recon library - ignoring\n");
	  }
	  glob->reconLib=NULL;
	}
      }
      if(glob->reconLib!=NULL){//do initialisation...
	doneParams=1;//the init function will do parameters...
	if((err=(*glob->reconOpenFn)(glob->reconNameOpen,glob->reconParamsCnt,glob->reconParams,glob->buffer[glob->curBuf],glob->rtcErrorBuf,glob->shmPrefix,glob->arrays,&glob->reconHandle,glob->nthreads,glob->thisiter,&glob->reconframeno,&glob->reconframenoSize,glob->totCents))){
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
      *glob->reconlibOpen=0;
      glob->reconCloseFn=NULL;
      glob->reconOpenFn=NULL;
      glob->reconNewParamFn=NULL;
      glob->reconNewFrameFn=NULL;
      glob->reconNewFrameSyncFn=NULL;
      glob->reconStartFrameFn=NULL;
      glob->reconNewSlopesFn=NULL;
      glob->reconEndFrameFn=NULL;
      glob->reconFrameFinishedFn=NULL;
      glob->reconFrameFinishedSyncFn=NULL;
      glob->reconOpenLoopFn=NULL;
      glob->reconCompleteFn=NULL;
    }
  }
  if(err==0 && glob->reconNewParamFn!=NULL && doneParams==0)
    err=(*glob->reconNewParamFn)(glob->reconHandle,glob->buffer[glob->curBuf],glob->thisiter,glob->arrays,glob->totCents);
  return err;
}


/**
   Opens the dynamic library for buffer...
*/
int updateBufferLibrary(globalStruct *glob){
  int open=0,err=0,doneParams=0;
  if(glob->bufferNameOpen==NULL){
    if(glob->bufferName!=NULL && *glob->bufferlibOpen==1){
      glob->bufferNameOpen=strdup(glob->bufferName);
      open=1;
    }
  }else{//already one open
    if(glob->bufferName==NULL || *glob->bufferlibOpen==0 || glob->go==0){
      free(glob->bufferNameOpen);
      glob->bufferNameOpen=NULL;
      *glob->bufferlibOpen=0;
      open=1;
    }else{
      if(strcmp(glob->bufferName,glob->bufferNameOpen)){//different name...
	free(glob->bufferNameOpen);
	glob->bufferNameOpen=strdup(glob->bufferName);
	open=1;
      }
    }
  }
  if(open){
    //first close existing, if it is open...
    if(glob->bufferLib!=NULL){
      //close existing library.
      (*glob->bufferCloseFn)(&glob->bufferHandle);
      if(dlclose(glob->bufferLib)!=0){
	printf("Failed to close buffer library - ignoring\n");
      }
      glob->bufferLib=NULL;
    }
    //and then open the new one.
    if(glob->bufferNameOpen!=NULL && glob->go!=0){
      if((glob->bufferLib=dlopen(glob->bufferNameOpen,RTLD_LAZY))==NULL){
	printf("Failed to open buffer library %s: %s\n",glob->bufferNameOpen,dlerror());
	err=1;
      }else{//now get the symbols...
	int nsym=0;
	if((*(void**)(&glob->bufferOpenFn)=dlsym(glob->bufferLib,"bufferOpen"))==NULL){
	  printf("dlsym failed for bufferOpen\n");
	  err=1;
	}else{nsym++;}
	if((*(void**)(&glob->bufferCloseFn)=dlsym(glob->bufferLib,"bufferClose"))==NULL){
	  printf("dlsym failed for bufferClose\n");
	  err=1;
	}else{nsym++;}
	if((*(void**)(&glob->bufferNewParamFn)=dlsym(glob->bufferLib,"bufferNewParam"))==NULL){
	  printf("dlsym failed for bufferNewParam (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->bufferUpdateFn)=dlsym(glob->bufferLib,"bufferUpdate"))==NULL){
	  printf("dlsym failed for bufferUpdate (non-fatal)\n");
	}else{nsym++;}
	if(err!=0 || nsym==0){//close the dll...
	  if(glob->bufferLib!=NULL  && dlclose(glob->bufferLib)!=0){
	    printf("Failed to close buffer library - ignoring\n");
	  }
	  glob->bufferLib=NULL;
	}
      }
      if(glob->bufferLib!=NULL){//do initialisation...
	doneParams=1;//the init function will do parameters...
	if((err=(*glob->bufferOpenFn)(glob->bufferNameOpen,glob->bufferParamsCnt,glob->bufferParams,glob->buffer[glob->curBuf],glob->rtcErrorBuf,glob->shmPrefix,glob->arrays,&glob->bufferHandle,glob->nthreads,glob->thisiter,&glob->bufferframeno,&glob->bufferframenoSize,glob->buffer[1-glob->curBuf]))){
	  printf("Error calling bufferOpen function\n");
	  if(dlclose(glob->bufferLib)!=0){
	    printf("Failed to close buffer library - ignoring\n");
	  }
	  glob->bufferLib=NULL;
	}
      }
      if(glob->bufferLib==NULL){
	free(glob->bufferNameOpen);
	glob->bufferNameOpen=NULL;
      }
	  
    }
    if(glob->bufferLib==NULL){
      *glob->bufferlibOpen=0;
      glob->bufferCloseFn=NULL;
      glob->bufferOpenFn=NULL;
      glob->bufferNewParamFn=NULL;
      glob->bufferUpdateFn=NULL;
    }
  }
  if(err==0 && glob->bufferNewParamFn!=NULL && doneParams==0)
    err=(*glob->bufferNewParamFn)(glob->bufferHandle,glob->buffer[glob->curBuf],glob->thisiter,glob->arrays,glob->buffer[1-glob->curBuf]);
  return err;
}




/**
   Opens the dynamic library, and then uses this to open the camera
*/
int updateCameraLibrary(globalStruct *glob){
  //globalStruct *glob=threadInfo->globals;
  int cerr=0,open=0;
  int doneParams=0;
  if(glob->cameraNameOpen==NULL){
    if(glob->cameraName!=NULL && *glob->camerasOpen==1){
      glob->cameraNameOpen=strdup(glob->cameraName);
      open=1;
    }
  }else{//already one open
    if(glob->cameraName==NULL || *glob->camerasOpen==0 || glob->go==0){
      free(glob->cameraNameOpen);
      glob->cameraNameOpen=NULL;
      *glob->camerasOpen=0;
      open=1;
    }else if(strcmp(glob->cameraName,glob->cameraNameOpen)){//different name...
      free(glob->cameraNameOpen);
      glob->cameraNameOpen=strdup(glob->cameraName);
      open=1;
    }
  }
  if(open){//first open the shared object library...
    if(glob->cameraLib!=NULL){//attempt to close existing open library...
      if(glob->camCloseFn!=NULL)
	(*glob->camCloseFn)(&glob->camHandle);
      glob->camHandle=NULL;
      if(dlclose(glob->cameraLib)!=0){
	printf("Failed to close camera dynamic library - ignoring\n");
      }
      glob->cameraLib=NULL;
    }
    if(glob->cameraNameOpen!=NULL && glob->go!=0){
      printf("Opening %s\n",glob->cameraNameOpen);
      if((glob->cameraLib=dlopen(glob->cameraNameOpen,RTLD_LAZY))==NULL){
	printf("Failed to open camera dynamic library %s: %s\n",glob->cameraNameOpen,dlerror());
	cerr=1;
      }else{
	int nsym=0;
	//now get the symbols... we want camOpen,camClose,camStartFraming,camStopFraming,camNewFrame and camWaitPixels.
	if((*(void**)(&glob->camOpenFn)=dlsym(glob->cameraLib,"camOpen"))==NULL){
	  printf("dlsym failed for camOpen\n");
	  cerr=1;
	}else{nsym++;}
	if((*(void**)(&glob->camCloseFn)=dlsym(glob->cameraLib,"camClose"))==NULL){
	  printf("dlsym failed for camClose\n");
	  cerr=1;
	}else{nsym++;}
	if((*(void**)(&glob->camNewFrameFn)=dlsym(glob->cameraLib,"camNewFrame"))==NULL){
	  printf("dlsym failed for camNewFrame (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->camNewFrameSyncFn)=dlsym(glob->cameraLib,"camNewFrameSync"))==NULL){
	  printf("dlsym failed for camNewFrameSync (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->camStartFrameFn)=dlsym(glob->cameraLib,"camStartFrame"))==NULL){
	  printf("dlsym failed for camStartFrame (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->camEndFrameFn)=dlsym(glob->cameraLib,"camEndFrame"))==NULL){
	  printf("dlsym failed for camEndFrame (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->camWaitPixelsFn)=dlsym(glob->cameraLib,"camWaitPixels"))==NULL){
	  printf("dlsym failed for camWaitPixels\n");
	}else{nsym++;}
	if((*(void**)(&glob->camNewParamFn)=dlsym(glob->cameraLib,"camNewParam"))==NULL){
	  printf("camera library has no newParam function - continuing...\n");
	  //this is now an error, just a warning.
	}else{nsym++;}
	if((*(void**)(&glob->camFrameFinishedFn)=dlsym(glob->cameraLib,"camFrameFinished"))==NULL){
	  printf("dlsym failed for camFrameFinished (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->camFrameFinishedSyncFn)=dlsym(glob->cameraLib,"camFrameFinishedSync"))==NULL){
	  printf("dlsym failed for camFrameFinishedSync (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->camOpenLoopFn)=dlsym(glob->cameraLib,"camOpenLoop"))==NULL){
	  printf("dlsym failed for camOpenLoop (non-fatal)\n");
	}else{nsym++;}
	if((*(void**)(&glob->camCompleteFn)=dlsym(glob->cameraLib,"camComplete"))==NULL){
	  printf("dlsym failed for camComplete (non-fatal)\n");
	}else{nsym++;}
	if(cerr!=0 || nsym==0){//close the dll
	  if(glob->cameraLib!=NULL && dlclose(glob->cameraLib)!=0){
	    printf("Failed to close camera library - ignoring\n");
	  }
	  glob->cameraLib=NULL;
	}
      }
      if(glob->cameraLib!=NULL){//do the initialisation
	doneParams=1;//the init function will do parameters(?)
	//if(glob->pxlbufs==NULL){
	//}
	if((cerr=(*glob->camOpenFn)(glob->cameraName,glob->cameraParamsCnt,glob->cameraParams,glob->buffer[glob->curBuf],glob->rtcErrorBuf,glob->shmPrefix,glob->arrays,&glob->camHandle,glob->nthreads,glob->thisiter,&glob->camframeno,&glob->camframenoSize,glob->totPxls,glob->ncam,glob->npxlxList,glob->npxlyList))!=0){//This is probably a blocking call - ie might wait until the camera reaches a certain temerature or whatever...
	  printf("Error calling camOpenFn\n");
	  if(dlclose(glob->cameraLib)!=0){
	    printf("Faild to close camera library - ignoring\n");
	  }
	  glob->cameraLib=NULL;
	}
      }
      if(glob->cameraLib==NULL){
	free(glob->cameraNameOpen);
	glob->cameraNameOpen=NULL;
      }
      if(cerr){
	writeError(glob->rtcErrorBuf,"Error opening camera",-1,glob->thisiter);
	*glob->camerasOpen=0;//write to the parambuf
      }
      if(glob->cameraLib==NULL){
	free(glob->cameraNameOpen);
	glob->cameraNameOpen=NULL;
      }
    }
    if(glob->cameraLib==NULL){
      *glob->camerasOpen=0;
      glob->camOpenFn=NULL;
      glob->camCloseFn=NULL;
      glob->camNewFrameFn=NULL;
      glob->camNewFrameSyncFn=NULL;
      glob->camWaitPixelsFn=NULL;
      glob->camNewParamFn=NULL;
      glob->camStartFrameFn=NULL;
      glob->camEndFrameFn=NULL;
      glob->camFrameFinishedFn=NULL;
      glob->camFrameFinishedSyncFn=NULL;
      glob->camOpenLoopFn=NULL;
      glob->camCompleteFn=NULL;
    }
  }
  if(cerr==0 && doneParams==0 && glob->camNewParamFn!=NULL){
    cerr=(*glob->camNewParamFn)(glob->camHandle,glob->buffer[glob->curBuf],glob->thisiter,glob->arrays);
  }
  
  return cerr;
}
/**
   Opens and closes the dynamic library and uses this to open the mirror
   Also updates the mirror with the new param buffer.
*/
int updateMirror(globalStruct *glob){
  //infoStruct *info=threadInfo->info;
  //globalStruct *glob=threadInfo->globals;
  int cerr=0;
  int open=0;
  int doneParam=0;
  //first open the shared object library...
  if(glob->mirrorNameOpen==NULL){
    if(glob->mirrorName!=NULL && *glob->mirrorOpen==1){
      glob->mirrorNameOpen=strdup(glob->mirrorName);
      open=1;
    }
  }else{//already one open
    if(*glob->mirrorOpen==0 || glob->mirrorName==NULL || glob->go==0){
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
    if(glob->mirrorNameOpen!=NULL && glob->go!=0){
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
	  cerr=(*glob->mirrorOpenFn)(glob->mirrorNameOpen,glob->mirrorParamsCnt,glob->mirrorParams,glob->buffer[glob->curBuf],glob->rtcErrorBuf,glob->shmPrefix,glob->arrays,&glob->mirrorHandle,glob->nacts,glob->rtcActuatorBuf,glob->thisiter,&glob->mirrorframeno,&glob->mirrorframenoSize);
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
  if(cerr==0 && glob->mirrorLib!=NULL && doneParam==0 && glob->mirrorNewParamFn!=NULL){
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
    *glob->mirrorOpen=0;
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
int openFigure(globalStruct *glob){
  //infoStruct *info=threadInfo->info;
  //globalStruct *glob=threadInfo->globals;
  int cerr=0,open=0;
  int doneParam=0;
  if(glob->figureNameOpen==NULL){
    if(glob->figureName!=NULL && *glob->figureOpen==1){
      glob->figureNameOpen=strdup(glob->figureName);
      open=1;
    }
  }else{//already one open
    if(*glob->figureOpen==0 || glob->figureName==NULL || glob->go==0){
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
    if(glob->figureNameOpen!=NULL && glob->go!=0){
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
	cerr=(*glob->figureOpenFn)(glob->figureName,glob->figureParamsCnt,glob->figureParams,glob->buffer[glob->curBuf],glob->rtcErrorBuf,glob->shmPrefix,glob->arrays,&glob->figureHandle,glob->nthreads,glob->thisiter,&glob->figureframeno,&glob->figureframenoSize,glob->totCents,glob->nacts,glob->precomp->post.actsRequiredMutex,glob->precomp->post.actsRequiredCond,&(glob->precomp->post.actsRequired));
      }
      if(cerr){
	writeError(glob->rtcErrorBuf,"Error opening figure sensor library",-1,glob->thisiter);
	*glob->figureOpen=0;
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
      *glob->figureOpen=0;
    }
  }
  if(cerr==0 && glob->figureLib!=NULL && doneParam==0 && glob->figureNewParamFn!=NULL){
    cerr=(*glob->figureNewParamFn)(glob->figureHandle,glob->buffer[glob->curBuf],glob->thisiter,glob->arrays);
  }
  return cerr;
}


/**
   This is called once by the first thread that does a buffer swap - it checks that arrays are of correct size for new buffer, and if not, reallocates them.
*/
int updateMemory(globalStruct *glob){
  //infoStruct *info=threadInfo->info;
  arrayStruct *arr=glob->arrays;//091109[threadInfo->mybuf];
  //arrayStruct *glob2=threadInfo->globals->arrays;//091109[1-threadInfo->mybuf];
  //globalStruct *glob=threadInfo->globals;
  int err=0;
  void *tmps;
  //now allocate an array large enough to hold all pixels (note - this may not be used, depending on the camera library...)
  if(arr->pxlbufelsize==0){
    arr->pxlbufelsize=sizeof(unsigned short);
    arr->pxlbuftype='H';
  }
  if(arr->pxlbufsSize!=arr->pxlbufelsize*glob->totPxls){
    arr->pxlbufsSize=arr->pxlbufelsize*glob->totPxls;
    tmps=realloc(arr->pxlbufs,arr->pxlbufsSize);
    if(tmps==NULL){
      if(arr->pxlbufs!=NULL)
	free(arr->pxlbufs);
      printf("pxlbuf malloc error.\n");
      err=1;
      arr->pxlbufsSize=0;
    }
    arr->pxlbufs=tmps;
    memset(arr->pxlbufs,0,arr->pxlbufsSize);
  }
  
  
  if(arr->fluxSize<glob->totCents/2){
    if(arr->flux!=NULL)
      free(arr->flux);
    arr->fluxSize=glob->totCents/2;
    if((arr->flux=malloc(sizeof(float)*glob->totCents/2))==NULL){
      printf("malloc of flux failed\n");
      arr->fluxSize=0;
      err=1;
    }
  }
  if(arr->centroidsSize<glob->totCents){
    if(arr->centroids!=NULL)
      free(arr->centroids);
    arr->centroidsSize=glob->totCents;
    if((arr->centroids=malloc(sizeof(float)*glob->totCents))==NULL){
      printf("malloc of centroids failed\n");
      err=1;
      arr->centroidsSize=0;
    }
  }
  if(arr->dmCommandSize<glob->nacts){
    if(arr->dmCommand!=NULL)
      free(arr->dmCommand);
    if(arr->dmCommandSave!=NULL)
      free(arr->dmCommandSave);
    if(arr->dmCommandFigure!=NULL)
      free(arr->dmCommandFigure);

    arr->dmCommandSave=NULL;
    arr->dmCommandFigure=NULL;
    arr->dmCommandSize=glob->nacts;
    if((arr->dmCommand=malloc(sizeof(float)*glob->nacts))==NULL){
      printf("malloc of dmCommand failed\n");
      err=1;
      arr->dmCommandSize=0;
    }else{
      if((arr->dmCommandSave=malloc(sizeof(float)*glob->nacts))==NULL){
	printf("malloc of dmCommandSave failed\n");
	err=1;
	if(arr->dmCommand!=NULL)
	  free(arr->dmCommand);
	arr->dmCommand=NULL;
	arr->dmCommandSize=0;
      }else{
	if((arr->dmCommandFigure=malloc(sizeof(float)*glob->nacts))==NULL){
	  printf("malloc of dmCommandFigure failed\n");
	  err=1;
	  if(arr->dmCommand!=NULL)
	    free(arr->dmCommand);
	  if(arr->dmCommandSave!=NULL)
	    free(arr->dmCommandSave);
	  arr->dmCommand=NULL;
	  arr->dmCommandSize=0;
	}
      }
    }
  }

  if(arr->calpxlbufSize<glob->totPxls){
    if(arr->calpxlbuf!=NULL)
      free(arr->calpxlbuf);
    arr->calpxlbufSize=glob->totPxls;
    if((arr->calpxlbuf=malloc(sizeof(float)*glob->totPxls))==NULL){
      printf("malloc of calpxlbuf failed\n");
      err=1;
      arr->calpxlbufSize=0;
    }else{
      memset(arr->calpxlbuf,0,sizeof(float)*glob->totPxls);
    }
  }
  /*
  if(arr->corrbufSize<info->totPxls){
    if(arr->corrbuf!=NULL)
      free(arr->corrbuf);
    arr->corrbufSize=info->totPxls;
    if((arr->corrbuf=malloc(sizeof(float)*info->totPxls))==NULL){
      printf("malloc of corrbuf failed\n");
      err=1;
      arr->corrbufSize=0;
    }
    }*/
  if(glob->windowMode==WINDOWMODE_ADAPTIVE || glob->windowMode==WINDOWMODE_GLOBAL){
    if(arr->subapLocationSize<glob->nsubaps*glob->maxPxlPerSubap){
      arr->subapLocation=glob->subapLocationMem;
      if(arr->subapLocation!=NULL)
	free(arr->subapLocation);
      arr->subapLocationSize=glob->nsubaps*glob->maxPxlPerSubap;
      if((arr->subapLocation=malloc(arr->subapLocationSize*sizeof(int)))==NULL){
	printf("malloc of subapLocation failed\n");
	err=1;
	glob->subapLocationMem=NULL;
	arr->subapLocationSize=0;
      }
      memcpy(arr->subapLocation,glob->realSubapLocation,glob->nsubaps*glob->maxPxlPerSubap*sizeof(int));
      glob->subapLocationMem=arr->subapLocation;
    }
  }else{//just point arr subaplocation to the real one... so that it is always valid, so that rtccalibrate.so objects can just use the arr->subapLocation.
    arr->subapLocation=glob->realSubapLocation;
  }
  if(err){
    glob->buferr=1;
    if(glob->ppause!=NULL)
      *(glob->ppause)=1;
  }
  return err;
}

/**
   Called by first thread only.
*/
int openLibraries(globalStruct *glob,int getlock){//threadStruct *threadInfo){
  //infoStruct *info=threadInfo->info;
  //globalStruct *glob=threadInfo->globals;
  //int cerr=0;
  int err=0;
  //if(*info->camerasOpen==1 && glob->camHandle==NULL){//camera not yet open
  if(getlock)
    pthread_mutex_lock(&glob->libraryMutex);

  err=updateCameraLibrary(glob);
  //do this every bufferswap, incase new params are read by the library...
  err|=updateMirror(glob);

  //if(*info->figureOpen==1){// && glob->figureHandle==NULL){
  //connect to the figure sensor setpoint reading library.  This is used when this RTC is being used as a figure sensor.
  err|=openFigure(glob);
  //err|=cerr;
  err|=updateCalibrateLibrary(glob);
  //err|=cerr;
  err|=updateCentLibrary(glob);
  //err|=cerr;
  //Open the reconstructor library (if correct one not open) and give it the new parameters.
  err|=updateReconLibrary(glob);
  err|=updateBufferLibrary(glob);
  if(getlock)
    pthread_mutex_unlock(&glob->libraryMutex);
  return err;
}
    
/**
   Each thread needs to update their priorities etc.
*/
void doThreadBufferSwap(threadStruct *threadInfo){
  //infoStruct *info;
  globalStruct *glob=threadInfo->globals;
  //Swap the buffers for this thread.
  //int err=0;//,cerr=0;
  //info=threadInfo->info;

  setThreadAffinity(glob,threadInfo->threadno+1,glob->ncpu);

}

/**
   This copies the global work array pointers to the pointers for this camera.
   This needs to be done after a call to updateMemory, once for each camera.
*/

int swapArrays(threadStruct *threadInfo){
  globalStruct *glob=threadInfo->globals;
  infoStruct *info=threadInfo->info;
  arrayStruct *arr=threadInfo->globals->arrays;//091109[threadInfo->mybuf];
 
  if(glob->windowMode==WINDOWMODE_ADAPTIVE || glob->windowMode==WINDOWMODE_GLOBAL){
    info->subapLocation=arr->subapLocation;
    //info->adaptiveCentPos=arr->adaptiveCentPos;
    //info->adaptiveWinPos=arr->adaptiveWinPos;
  }else{
    info->subapLocation=glob->realSubapLocation;
  }
  
  return 0;
}



/**
   Wait for the centroids to arrive.
   Called to get (optional) slope input
*/
int computeSlopes(threadStruct *threadInfo){
  int rt=0;
  globalStruct *glob=threadInfo->globals;
  //arrayStruct *arr=glob->arrays;
  if(glob->centLib!=NULL && glob->centCalcSlopeFn!=NULL){
    //This call should block until nsubs slope measurement have been received.
    //This function should return 1 on error, and 0 if slopes are valid.
    //Can return -1 if slopes are not valid, but this is not an error (e.g. you don't want to add anything this time).
    //was centWaitPixelsFn here - now renamed.
    //Note - nsubs is the total number of subaps that are needed for this to be computed, e.g. the total number that must have arrived from the WPU.
    rt=(*glob->centCalcSlopeFn)(glob->centHandle,threadInfo->info->cam,threadInfo->threadno,threadInfo->nsubs,threadInfo->subap,threadInfo->subapSize,threadInfo->cursubindx,threadInfo->centindx,threadInfo->curnpxlx,threadInfo->curnpxly);

  }
  return (rt==1);
}


void doPreProcessing(globalStruct *glob){
  if(!glob->noPrePostThread)
    pthread_mutex_lock(&glob->libraryMutex);
  if(glob->camNewFrameFn!=NULL)//tell cam library: new frame
    (*glob->camNewFrameFn)(glob->camHandle,glob->thisiter,glob->starttime);
  if(glob->calibrateNewFrameFn!=NULL)//tell calibrate library: new frame
    (*glob->calibrateNewFrameFn)(glob->calibrateHandle,glob->thisiter,glob->starttime);
  if(glob->centNewFrameFn!=NULL)//tell cent library: new frame
    (*glob->centNewFrameFn)(glob->centHandle,glob->thisiter,glob->starttime);
  if(glob->reconNewFrameFn!=NULL)//First do the recon library.
    (*glob->reconNewFrameFn)(glob->reconHandle/*,glob->arrays->dmCommand*/,glob->thisiter,glob->starttime);
  if(!glob->noPrePostThread)
    pthread_mutex_unlock(&glob->libraryMutex);
}

void doPostProcessing(globalStruct *glob){
  PostComputeData *pp=&(glob->precomp->post);
  int sendAvCalPxl=0;
  int sendAvCent=0;
  float *tmpf;
  double timestamp;
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
    //circAdd(glob->rtcCorrBuf,pp->corrbuf,timestamp,pp->thisiter);
    //calpxlbuf can now be written to by other threads
    memset(pp->calpxlbuf,0,pp->totPxls*sizeof(float));//clear the buffer for next time.
    circAdd(glob->rtcCentBuf,pp->centroids,timestamp,pp->thisiter);
    circAdd(glob->rtcFluxBuf,pp->flux,timestamp,pp->thisiter);
    //centroids can now be written to by other threads
    circAdd(glob->rtcSubLocBuf,pp->subapLocation,timestamp,pp->thisiter);
    
  }
  if(!pp->noPrePostThread)
    pthread_mutex_lock(&glob->libraryMutex);
  if(glob->camCompleteFn!=NULL)//tell cam library: new frame
    (*glob->camCompleteFn)(glob->camHandle);
  if(glob->calibrateCompleteFn!=NULL)//tell calibrate library: new frame
    (*glob->calibrateCompleteFn)(glob->calibrateHandle);
  if(glob->centCompleteFn!=NULL)//tell cent library: new frame
    (*glob->centCompleteFn)(glob->centHandle);
  if(glob->reconCompleteFn!=NULL)
    (*glob->reconCompleteFn)(glob->reconHandle);
  if(!pp->noPrePostThread)
    pthread_mutex_unlock(&glob->libraryMutex);
  
  
  
  setArraysReady(glob);
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
  if(pp->nclipped>pp->maxClipped){
    writeError(glob->rtcErrorBuf,"Maximum clipping exceeded",CLIPERROR,pp->thisiter);
    if(pp->openLoopIfClipped){
      *glob->closeLoop=0;
    }
  }
  
  writeStatusBuf(glob,0,*pp->closeLoop);
  circAdd(glob->rtcStatusBuf,glob->statusBuf,timestamp,pp->thisiter);
}  

/**
   Wake up the pre/post processing thread.
*/
int wakePrepareActuatorsThread(threadStruct *threadInfo){
  PreComputeData *p=threadInfo->globals->precomp;
  //infoStruct *info=threadInfo->info;
  globalStruct *glob=threadInfo->globals;
  dprintf("wakePrepareActuatorsThread %p %p\n",p,info);
  p->nacts=glob->nacts;
  p->totCents=threadInfo->globals->totCents;
  //these are the ones that are written to by the threads and so must be synced
  p->dmCommand=glob->arrays->dmCommand;
  if(!glob->noPrePostThread){
    if(pthread_mutex_lock(&p->prepMutex))
      printf("pthread_mutex_lock error in wakePrepareActuatorsThread2: %s\n",strerror(errno));
    if(p->readyToStart==-1)
      pthread_cond_signal(&p->prepCond);//wake the thread
    else
      p->readyToStart=1;
    pthread_mutex_unlock(&p->prepMutex);
  }else{
    doPreProcessing(threadInfo->globals);
  }
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
  arrayStruct *arr=globals->arrays;
  char fw;
  STARTTIMING;
  //no need to get the mutex to do this, because - the subap threads are blocked (one of them is running this method), so won't be looking at the calcentReady, and the pre/post processing thread is also blocked, or at least has passed the stage where it sets calcentReady.
  globals->calCentReady=0;
  /*
#ifdef NONMODULARCENTROIDER
  if(pthread_mutex_lock(&globals->precomp->pxlcentMutex))
    printf("pthread_mutex_lock error in endFrame: %s\n",strerror(errno));
  globals->precomp->pxlcentReady=0;
  pthread_mutex_unlock(&globals->precomp->pxlcentMutex);
#endif
  */
  if(!globals->noPrePostThread){
    if(pthread_mutex_lock(&globals->precomp->postMutex))
      printf("pthread_mutex_lock error in endFrame2: %s\n",strerror(errno));
  }
  //set up the correct pointers...
  p->noPrePostThread=globals->noPrePostThread;
  p->closeLoop=globals->closeLoop;
  p->openLoopIfClipped=globals->openLoopIfClipped;
  p->userActs=globals->userActs;
  p->userActsMask=globals->userActsMask;
  p->addUserActs=globals->addUserActs;
  p->userActSeq=globals->userActSeq;
  p->userActSeqLen=globals->userActSeqLen;
  p->recordCents=globals->recordCents;
  p->totPxls=globals->totPxls;
  p->nAvImg=globals->nAvImg;
  p->averageImg=globals->averageImg;
  p->nAvCent=globals->nAvCent;
  p->averageCent=globals->averageCent;
  p->nacts=globals->nacts;
  p->totCents=globals->totCents;
  p->maxClipped=globals->maxClipped;
  //these are written to by the post compute thread...
  p->calpxlbuf=arr->calpxlbuf;
  //p->corrbuf=info->corrbuf;
  p->centroids=arr->centroids;
  p->flux=arr->flux;
  p->dmCommand=arr->dmCommand;
  p->dmCommandSave=arr->dmCommandSave;
  p->figureGain=globals->figureGain;
  p->figureGainArr=globals->figureGainArr;
  p->dmCommandFigure=globals->arrays->dmCommandFigure;
  p->mirrorHandle=globals->mirrorHandle;
  p->mirrorSendFn=globals->mirrorSendFn;
  p->reconOpenLoopFn=globals->reconOpenLoopFn;
  p->calibrateOpenLoopFn=globals->calibrateOpenLoopFn;
  p->camOpenLoopFn=globals->camOpenLoopFn;
  p->centOpenLoopFn=globals->centOpenLoopFn;
  p->reconFrameFinishedFn=globals->reconFrameFinishedFn;
  p->calibrateFrameFinishedFn=globals->calibrateFrameFinishedFn;
  p->camFrameFinishedFn=globals->camFrameFinishedFn;
  p->centFrameFinishedFn=globals->centFrameFinishedFn;
  p->thisiter=globals->thisiter;
  p->windowMode=globals->windowMode;
  p->subapLocation=info->subapLocation;
  p->pxlCentInputError=info->pxlCentInputError;
  //if(threadInfo->info->windowMode==WINDOWMODE_GLOBAL)
  //  calcGlobalAdaptiveWindow(threadInfo->info);


  //gettimeofday(&thistime,NULL);
  //timestamp=thistime.tv_sec+thistime.tv_usec*1e-6;
  fw=FORCEWRITEALL(globals->rtcPxlBuf);
  if(fw){
    FORCEWRITE(globals->rtcPxlBuf)=fw;
    FORCEWRITE(globals->rtcStatusBuf)=fw;
    FORCEWRITE(globals->rtcTimeBuf)=fw;
    FORCEWRITE(globals->rtcCalPxlBuf)=fw;
    //FORCEWRITE(globals->rtcCorrBuf)=fw;
    FORCEWRITE(globals->rtcCentBuf)=fw;
    FORCEWRITE(globals->rtcFluxBuf)=fw;
    FORCEWRITE(globals->rtcSubLocBuf)=fw;
    FORCEWRITE(globals->rtcMirrorBuf)=fw;
    FORCEWRITE(globals->rtcActuatorBuf)=fw;
    FORCEWRITEALL(globals->rtcPxlBuf)=0;
  }

  //No need to get the libraryMutex here, since this is never called at the same time as the updateRecon method...
  if(globals->camFrameFinishedSyncFn!=NULL)
    (*globals->camFrameFinishedSyncFn)(globals->camHandle,info->pxlCentInputError,fw);
  if(globals->calibrateFrameFinishedSyncFn!=NULL)
    (*globals->calibrateFrameFinishedSyncFn)(globals->calibrateHandle,info->pxlCentInputError,fw);
  if(globals->centFrameFinishedSyncFn!=NULL)
    (*globals->centFrameFinishedSyncFn)(globals->centHandle,info->pxlCentInputError,fw);
  if(globals->reconFrameFinishedSyncFn!=NULL)
    (*globals->reconFrameFinishedSyncFn)(globals->reconHandle,/*threadInfo->info->centroids,globals->arrays->dmCommand,*/info->pxlCentInputError,fw);
  if(!globals->noPrePostThread){
    pthread_cond_signal(&globals->precomp->postCond);//wake the thread
    pthread_mutex_unlock(&globals->precomp->postMutex);
  }else{
    doPostProcessing(globals);
  }
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
  //CBLAS_TRANSPOSE trans=CblasNoTrans;
  //struct timeval t1;
  //float alpha=1.,beta=1.;
  //int inc=1;
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

    setThreadAffinity(glob,0,glob->ncpu);
    //}
    //First we do initial preparation...
    dprintf("Precomputing dmCommand\n");
    doPreProcessing(glob);

    ///////////////////////////NOW END OF FRAME////////////////////////////////

    //and now we wait for the end of frame stuff...
    p=glob->precomp;
    if(pthread_cond_wait(&p->postCond,&p->postMutex))
      printf("pthread_cond_wait error in prepareActuators2: %s\n",strerror(errno));
    dprintf("postcomputing, sending actuators etc\n");
    //pp=&p->post;

    //Now send stuff to the circular buffers.
    //gettimeofday(&t1,NULL);
    //timestamp=t1.tv_sec+t1.tv_usec*1e-6;
    //pp->timestamp=timestamp;
    doPostProcessing(glob);

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
  if(index>=0 && index<NBUFFERVARIABLES && strncmp(&buf[index*BUFNAMESIZE],"switchRequested",BUFNAMESIZE)==0){//found it...
    if(pbuf->dtype[j]=='i' && pbuf->nbytes[j]==sizeof(int)){
      sr=*((int*)BUFGETVALUE(pbuf,j));
      if(sr==1)
	globals->switchRequestedPtr=((int*)BUFGETVALUE(pbuf,j));
      found=1;
    }else{
      printf("switchRequested error %d %c\n",globals->bufferNbytes[index],globals->bufferDtype[index]);
    }
  }
  if(found==0){
    while(j<BUFNHDR(pbuf) && buf[j*BUFNAMESIZE]!='\0'){
      if(strncmp(&buf[j*BUFNAMESIZE],"switchRequested",BUFNAMESIZE)==0){
	if(pbuf->dtype[j]=='i' && pbuf->nbytes[j]==sizeof(int)){
	  sr=*((int*)BUFGETVALUE(pbuf,j));
	  if(sr==1)
	    globals->switchRequestedPtr=((int*)BUFGETVALUE(pbuf,j));
	}else{
	  printf("switchRequested error... %d %c\n",pbuf->nbytes[j],pbuf->dtype[j]);
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
  //infoStruct *info=threadInfo->info;
  globalStruct *glob=threadInfo->globals;
  int dim,err=0;
  if(glob->rtcPxlBuf!=NULL && (glob->rtcPxlBuf->datasize!=glob->totPxls*glob->arrays->pxlbufelsize || DTYPE(glob->rtcPxlBuf)!=glob->arrays->pxlbuftype)){
    dim=glob->totPxls;
    if(circReshape(glob->rtcPxlBuf,1,&dim,glob->arrays->pxlbuftype)!=0){
      printf("Error reshaping rtcPxlBuf\n");
      err=1;
    }
  }
  if(glob->rtcCalPxlBuf!=NULL && glob->rtcCalPxlBuf->datasize!=glob->totPxls*sizeof(float)){
    dim=glob->totPxls;
    if(circReshape(glob->rtcCalPxlBuf,1,&dim,'f')!=0){
      printf("Error reshaping rtcCalPxlBuf\n");
      err=1;
    }
  }
  if(glob->rtcCentBuf!=NULL && glob->rtcCentBuf->datasize!=glob->totCents*sizeof(float)){
    dim=glob->totCents;
    if(circReshape(glob->rtcCentBuf,1,&dim,'f')!=0){
      printf("Error reshaping rtcCentBuf\n");
      err=1;
    }
  }
  if(glob->rtcMirrorBuf!=NULL && glob->rtcMirrorBuf->datasize!=glob->nacts*sizeof(float)){
    dim=glob->nacts;
    if(circReshape(glob->rtcMirrorBuf,1,&dim,'f')!=0){
      printf("Error reshaping rtcMirrorBuf\n");
      err=1;
    }
  }
  if(glob->rtcStatusBuf!=NULL && glob->rtcStatusBuf->datasize!=STATUSBUFSIZE){
    dim=STATUSBUFSIZE;
    if(circReshape(glob->rtcStatusBuf,1,&dim,'b')!=0){
      printf("Error reshaping rtcStatusBuf\n");
      err=1;
    }
  }
  if(glob->rtcTimeBuf!=NULL && glob->rtcTimeBuf->datasize!=sizeof(double)){
    dim=1;
    if(circReshape(glob->rtcTimeBuf,1,&dim,'d')!=0){
      printf("Error reshaping rtcTimeBuf\n");
      err=1;
    }
  }
  if(glob->rtcErrorBuf!=NULL && glob->rtcErrorBuf->datasize!=ERRORBUFSIZE){
    dim=ERRORBUFSIZE;
    if(circReshape(glob->rtcErrorBuf,1,&dim,'b')!=0){
      printf("Error reshaping rtcErrorBuf\n");
      err=1;
    }
  }
  if(glob->rtcSubLocBuf!=NULL && glob->rtcSubLocBuf->datasize!=glob->nsubaps*glob->maxPxlPerSubap*sizeof(int)){
    dim=glob->nsubaps*glob->maxPxlPerSubap;
    if(circReshape(glob->rtcSubLocBuf,1,&dim,'i')!=0){
      printf("Error reshaping rtcSubLocBuf\n");
      err=1;
    }
  }
  if(glob->rtcFluxBuf!=NULL && glob->rtcFluxBuf->datasize!=glob->totCents/2*sizeof(float)){
    dim=glob->totCents/2;
    if(circReshape(glob->rtcFluxBuf,1,&dim,'f')!=0){
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
    glob->rtcPxlBuf=openCircBuf(tmp,1,&glob->totPxls,glob->arrays->pxlbuftype,100);
    free(tmp);
  }
  if(glob->rtcCalPxlBuf==NULL){
    if(asprintf(&tmp,"/%srtcCalPxlBuf",glob->shmPrefix)==-1)
      exit(1);
    glob->rtcCalPxlBuf=openCircBuf(tmp,1,&glob->totPxls,'f',100);
    free(tmp);
  }
  if(glob->rtcCentBuf==NULL){
    if(asprintf(&tmp,"/%srtcCentBuf",glob->shmPrefix)==-1)
      exit(1);
    glob->rtcCentBuf=openCircBuf(tmp,1,&threadInfo->globals->totCents,'f',100);
    free(tmp);
  }
  if(glob->rtcMirrorBuf==NULL){
    if(asprintf(&tmp,"/%srtcMirrorBuf",glob->shmPrefix)==-1)
      exit(1);
    glob->rtcMirrorBuf=openCircBuf(tmp,1,&glob->nacts,'f',1000);
    free(tmp);
  }
  if(glob->rtcActuatorBuf==NULL){
    if(asprintf(&tmp,"/%srtcActuatorBuf",glob->shmPrefix)==-1)
      exit(1);
    glob->rtcActuatorBuf=openCircBuf(tmp,1,&glob->nacts,'f',1000);
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
    FREQ(glob->rtcErrorBuf)=1;
    //Note - this is actaully probably opened in darcmain.c
    free(tmp);
  }
  dim=glob->nsubaps*glob->maxPxlPerSubap;
  if(glob->rtcSubLocBuf==NULL){
    if(asprintf(&tmp,"/%srtcSubLocBuf",glob->shmPrefix)==-1)
      exit(1);
    glob->rtcSubLocBuf=openCircBuf(tmp,1,&dim,'i',100);
    free(tmp);
  }
  if(glob->rtcGenericBuf==NULL){//can store eg poke matrix of a calibrated image...
    dim=threadInfo->globals->totCents*4*glob->nacts;//4 values per poke
    if(glob->totPxls>dim)
      dim=glob->totPxls;
    if(asprintf(&tmp,"/%srtcGenericBuf",glob->shmPrefix)==-1)
      exit(1);
    glob->rtcGenericBuf=openCircBuf(tmp,1,&dim,'f',4);
    free(tmp);
  }
  if(glob->rtcFluxBuf==NULL){
    if(asprintf(&tmp,"/%srtcFluxBuf",glob->shmPrefix)==-1)
      exit(1);
    dim=threadInfo->globals->totCents/2;
    glob->rtcFluxBuf=openCircBuf(tmp,1,&dim,'f',100);
    free(tmp);
  }
  return 0;
}

/**
   Called each iteration, to do housekeeping (inform camera library, etc).
*/
void setFrameno(globalStruct *glob){
  //increate the rtc iteration count.
  glob->myiter++;
  //Now set thisiter to wherever it should be set.
  switch(glob->itersource){
  case 1://camera source
    if(glob->iterindex<glob->camframenoSize)
      glob->thisiter=glob->camframeno[glob->iterindex];
    else{
      glob->thisiter=glob->myiter;
      writeErrorVA(glob->rtcErrorBuf,FRAMENOERROR,glob->thisiter,"Error - illegal frame number source");
    }
    break;
  case 2://calibration source
    if(glob->iterindex<glob->calframenoSize)
      glob->thisiter=glob->calframeno[glob->iterindex];
    else{
      glob->thisiter=glob->myiter;
      writeErrorVA(glob->rtcErrorBuf,FRAMENOERROR,glob->thisiter,"Error - illegal frame number source");
    }
    break;
  case 3://centroid source
    if(glob->iterindex<glob->centframenoSize)
      glob->thisiter=glob->centframeno[glob->iterindex];
    else{
      glob->thisiter=glob->myiter;
      writeErrorVA(glob->rtcErrorBuf,FRAMENOERROR,glob->thisiter,"Error - illegal frame number source");
    }
    break;
  case 4://recon source
    if(glob->iterindex<glob->reconframenoSize)
      glob->thisiter=glob->reconframeno[glob->iterindex];
    else{
      glob->thisiter=glob->myiter;
      writeErrorVA(glob->rtcErrorBuf,FRAMENOERROR,glob->thisiter,"Error - illegal frame number source");
    }
    break;
  case 5://mirror source
    if(glob->iterindex<glob->mirrorframenoSize)
      glob->thisiter=glob->mirrorframeno[glob->iterindex];
    else{
      glob->thisiter=glob->myiter;
      writeErrorVA(glob->rtcErrorBuf,FRAMENOERROR,glob->thisiter,"Error - illegal frame number source");
    }
    break;
  case 6://figure source
    if(glob->iterindex<glob->figureframenoSize)
      glob->thisiter=glob->figureframeno[glob->iterindex];
    else{
      glob->thisiter=glob->myiter;
      writeErrorVA(glob->rtcErrorBuf,FRAMENOERROR,glob->thisiter,"Error - illegal frame number source");
    }
    break;
  case 7://buffer source
    if(glob->iterindex<glob->bufferframenoSize)
      glob->thisiter=glob->bufferframeno[glob->iterindex];
    else{
      glob->thisiter=glob->myiter;
      writeErrorVA(glob->rtcErrorBuf,FRAMENOERROR,glob->thisiter,"Error - illegal frame number source");
    }
    break;
  default:
    glob->thisiter=glob->myiter;
    break;
  }
}

/**
   Called each iteration, to do housekeeping (inform camera library, etc).
*/
int startNewFrame(threadStruct *threadInfo){
  globalStruct *glob=threadInfo->globals;
  //The first thread should tell the cameras that a new frame is starting.
  if(glob->buferr==0 && glob->bufferUseSeq!=0 && glob->bufferUpdateFn!=NULL)//update any param sequences
    (*glob->bufferUpdateFn)(glob->bufferHandle);


  if(glob->camNewFrameSyncFn!=NULL)//tell the camera library that new frame has started
    (*glob->camNewFrameSyncFn)(glob->camHandle,glob->thisiter,glob->starttime);
  if(glob->calibrateNewFrameSyncFn!=NULL)
    (*glob->calibrateNewFrameSyncFn)(glob->calibrateHandle,glob->thisiter,glob->starttime);
  if(glob->centNewFrameSyncFn!=NULL)
    (*glob->centNewFrameSyncFn)(glob->centHandle,glob->thisiter,glob->starttime);
  if(glob->reconNewFrameSyncFn!=NULL)
    (*glob->reconNewFrameSyncFn)(glob->reconHandle,glob->thisiter,glob->starttime);
  if(threadInfo->info->pause==0)
    wakePrepareActuatorsThread(threadInfo);
  else
    usleep(10000);
  if(glob->delay!=0)
    usleep(glob->delay);
  return 0;
}

void setGITID(globalStruct *glob){
  char *s=glob->mainGITID;
  if(asprintf(&glob->mainGITID,"%s%s",s,GITID)<0)
    glob->mainGITID=s;
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
  globalStruct *glob=threadInfo->globals;
  struct timeval thistime;
  double dtime;
  //int increment;
  int nsubapDone;
  struct sched_param schedParam;
  double timestamp=0;
  int cursubindx,centindx;//,doEndFrame;
  niters=glob->niters;
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
  while(niters!=0 && glob->go==1){
    //doEndFrame=0;//to be set to one if the pause flag is set (eg due to nsteps reaching zero or something).
    //agbthreadInfo->mybuf=1-threadInfo->mybuf;//switch buffers.
    
    dprintf("process frame thread %d niters %d mybuf %d frameno %d\n",threadInfo->threadno,niters,/*threadInfo->mybuf*/0,glob->frameno);
    
    if(glob->doswitch){//a switch has been requested.
      dprintf("doswitch %d\n",threadInfo->threadno);
      //get the global start mutex
      if(pthread_mutex_lock(&glob->startMutex))//this should be released as soon as we're allowed - as soon as the first thread has done some of the buffer swapping.
	printf("pthread_mutex_lock error in processFrame : %s\n",strerror(errno));
      //get the per-camera start mutex
      if(pthread_mutex_lock(&threadInfo->info->startInfoMutex))//this should be released as soon as the first thread for this camera has done its buffer swapping work.
	printf("pthread_mutex_lock startInfoMutex error in processFrame : %s\n",strerror(errno));
      glob->threadCount++;
      threadInfo->info->threadInfoCount++;//counter for this camera...
      if(glob->threadCount==1){//first thread overall...
	if(pthread_mutex_lock(&glob->startFirstMutex))//this should be released once the work of the setting up has finished... to stop other threads continuing... and once cameras have been signalled that a new frame has started etc...
	  printf("pthread_mutex_lock startFirstMutex error in processFrame : %s\n",strerror(errno));
	//if(getClearSwitchRequested(threadInfo)){//a new parameter buffer is ready
	//thread safety of curbuf is okay - since we wait for all threads to finish previous frame before getting here... and only 1 thread is here at once.
	glob->curBuf=1-glob->curBuf;
#ifdef USECOND
	freezeParamBuf(glob->buffer[glob->curBuf],glob->buffer[1-glob->curBuf]);
#else
	freezeParamBuf(glob->buffer[glob->curBuf]->semid,glob->buffer[1-glob->curBuf]->semid);
#endif
	glob->buferr=0;
	if(updateBufferIndex(threadInfo,1)!=0){//error... probably not all params present in buffer...
	  threadInfo->info->pause=1;
	  glob->buferr=1;
	  if(glob->ppause!=NULL)
	    *(glob->ppause)=1;
	  pthread_mutex_unlock(&glob->startMutex);
	}else{
	  //have updated the index, so now other threads can continue.  


	  //if(glob->reconLib!=NULL)
	  //  (*glob->reconNewFrameFn)(threadInfo);
	  //Can release the startMutex now.
	  err=updateBuffer(glob);
	  if(err==0)
	    err=updateMemory(glob);
	  info=threadInfo->info;
	  if(err!=0){
	    info->pause=1;
	    glob->buferr=1;
	    if(glob->ppause!=NULL)
	      *(glob->ppause)=1;
	  }
	  setFrameno(glob);
	  //The global buffer has now been updated, so other threads can proceed...
	  pthread_mutex_unlock(&glob->startMutex);
	  if(err==0){
	    updateInfo(threadInfo);
	    doThreadBufferSwap(threadInfo);
	    //updateMemory allocates memory if required (eg if size changed).
	    swapArrays(threadInfo);
	    if(glob->myiter==1)//the very first iteration
	      createCircBufs(threadInfo);
	    updateCircBufs(threadInfo);
	    if(openLibraries(glob,1)){
	      printf("Error opening libraries or doing buffer swap - pausing\n");
	      threadInfo->info->pause=1;
	      glob->buferr=1;
	      if(glob->ppause!=NULL)
		*(glob->ppause)=1;
	    }
	  }
	}
	startNewFrame(threadInfo);//this should be done regardless of whether there is an error or not.
	//and clear the switchRequested parameter in the SHM buffer...
	if(glob->switchRequestedPtr!=NULL)
	  *glob->switchRequestedPtr=0;
	//Now allow other threads to continue.
	pthread_mutex_unlock(&glob->startFirstMutex);
	//Now can release the startInfoMutex for other threads of this cam
	pthread_mutex_unlock(&threadInfo->info->startInfoMutex);
      }else if(threadInfo->info->threadInfoCount==1){//first thread for this camera...
	//if(glob->buferr)
	//  threadInfo->info->pause=1;
	pthread_mutex_unlock(&glob->startMutex);
	//if(glob->doswitch){//do a switch
	if(glob->buferr==0){
	  updateInfo(threadInfo);
	  doThreadBufferSwap(threadInfo);
	  //now wait for first thread overall to have updated the memory etc
	  if(pthread_mutex_lock(&glob->startFirstMutex))//this should be released once the work of the setting up has finished...
	    printf("pthread_mutex_lock startFirstMutex error in processFrame : %s\n",strerror(errno));
	  swapArrays(threadInfo);
	}else{
	  //now wait for first thread overall to have updated the memory etc
	  threadInfo->info->pause=1;
	  if(pthread_mutex_lock(&glob->startFirstMutex))//this should be released once the work of the setting up has finished...
	    printf("pthread_mutex_lock startFirstMutex error in processFrame : %s\n",strerror(errno));
	}
	pthread_mutex_unlock(&glob->startFirstMutex);
	if(glob->buferr)
	  threadInfo->info->pause=1;
	//Now can release the startInfoMutex for other threads of this cam
	pthread_mutex_unlock(&threadInfo->info->startInfoMutex);
      }else{//not first thread
	pthread_mutex_unlock(&glob->startMutex);
	//Now can release the startInfoMutex for other threads of this cam
	pthread_mutex_unlock(&threadInfo->info->startInfoMutex);
	if(glob->buferr==0)
	  doThreadBufferSwap(threadInfo);
      }
    }

    info=threadInfo->info;
    if(glob->go==0){
      printf("Thread %d leaving loop\n",threadInfo->threadno);
      break;
    }
    ////////////////////////////////////////////////////////////////
    //Now do the computation...
    //nsubapDone=0;
    err=0;
    if(info->pause==0){
      dprintf("Doing computation %d %d\n",threadInfo->threadno,niters);

      //if(info->kalmanUsed==0){
      //no need to get the libraryMutex here...
      if(glob->camStartFrameFn!=NULL)
	(*glob->camStartFrameFn)(glob->camHandle,threadInfo->info->cam,threadInfo->threadno);
      if(glob->calibrateStartFrameFn!=NULL)
	(*glob->calibrateStartFrameFn)(glob->calibrateHandle,threadInfo->info->cam,threadInfo->threadno);
      if(glob->centStartFrameFn!=NULL)
	(*glob->centStartFrameFn)(glob->centHandle,threadInfo->info->cam,threadInfo->threadno);
      if(glob->reconStartFrameFn!=NULL)//First do the recon library.
	(*glob->reconStartFrameFn)(glob->reconHandle,threadInfo->info->cam,threadInfo->threadno);
      //memset(threadInfo->dmCommand,0,threadInfo->dmCommandSize);
      //}else
	//memset(threadInfo->Xpred,0,threadInfo->XpredSize);
      //while(info->frameFinished[threadInfo->mybuf]==0){091109
      nsubapDone=0;
      threadInfo->frameFinished=0;
      threadInfo->cursubindx=info->subCumIndx;
      threadInfo->nsubapsProcessing=0;
      threadInfo->nsubapsDoing=0;
      threadInfo->centindx=info->centCumIndx;
      while(info->frameFinished==0 && threadInfo->frameFinished==0){
	dprintf("framefinished %d niter %d\n",info->frameFinished,niters);
	dprintf("getting subap %d\n",info->id);
	if((err=waitNextSubaps(threadInfo))==0){//got subap okay
	  //here we need to use nsubapsTogether
	  cursubindx=threadInfo->cursubindx;
	  centindx=threadInfo->centindx;
	  if(nsubapDone==0){
	    nsubapDone=1;
	    waitForArraysReady(glob);//wait until the cal/cent newFrameFn()s have completed
	  }
	  

	  //for(i=0; i<info->nsubapsTogether; i++){
	  for(i=0; i<threadInfo->nsubapsProcessing && err==0; i++){
	    if(glob->subapFlagArr[threadInfo->cursubindx]==1){
	      //this subap is valid, so do stuff...
	      //and now copy the pixels into our float array...
	      dprintf("copying subap %d\n",endFrame);
	      if(glob->calibrateNewSubapFn!=NULL)
		err=(*glob->calibrateNewSubapFn)(glob->calibrateHandle,threadInfo->info->cam,threadInfo->threadno,threadInfo->cursubindx,&threadInfo->subap,&threadInfo->subapSize,&threadInfo->curnpxlx,&threadInfo->curnpxly);
	      if(err==0 && (err=computeSlopes(threadInfo))!=0)
		writeErrorVA(glob->rtcErrorBuf,SLOPEERROR,glob->thisiter,"Error getting slopes");
	      threadInfo->centindx+=2;
	    }
	    threadInfo->cursubindx++;
	  }
	  //reset these for the next stage (reconstruction)
	  threadInfo->cursubindx=cursubindx;
	  threadInfo->centindx=centindx;
        }else{//didn't get subap okay - probably no more to get, or cam error.
	  dprintf("error getting subap %d\n",err);
	}
	//}
	if(err==0){//otherwise, frame probably finished (-1) or error (1)...

	  dprintf("reconstructing...\n");
	  //partialReconstruct(threadInfo);
	  if(glob->reconNewSlopesFn!=NULL){
	    (*glob->reconNewSlopesFn)(glob->reconHandle,threadInfo->info->cam,threadInfo->centindx,threadInfo->threadno,threadInfo->nsubapsDoing);//,threadInfo->info->centroids,glob->arrays->dmCommand);
	  }

	  dprintf("reconstructed\n");
	  //nsubapDone++;
	}else if(err==-1)//frame finished
	  err=0;
	else
	  threadInfo->err=1;
      }
      if(nsubapDone==0){
	nsubapDone=1;
	waitForArraysReady(glob);//wait until the cal/cent newFrameFn()s have completed
      }
      dprintf("Done computation %d %d\n",threadInfo->threadno,niters);
      //Each thread should then add its own dmCommand to the global dmCommand in
      //a thread safe way... First, need to wait for the initialisation of these arrays to be ready..
      //copyThreadPhase(threadInfo);//this gets the mutex...
      if(glob->camEndFrameFn!=NULL)
	(*glob->camEndFrameFn)(glob->camHandle,threadInfo->info->cam,threadInfo->threadno,threadInfo->err);
      if(glob->calibrateEndFrameFn!=NULL)
	(*glob->calibrateEndFrameFn)(glob->calibrateHandle,threadInfo->info->cam,threadInfo->threadno,threadInfo->err);
      if(glob->centEndFrameFn!=NULL)
	(*glob->centEndFrameFn)(glob->centHandle,threadInfo->info->cam,threadInfo->threadno,threadInfo->err);
      if(glob->reconEndFrameFn!=NULL)
	(*glob->reconEndFrameFn)(glob->reconHandle,threadInfo->info->cam,threadInfo->threadno,threadInfo->err);//threadInfo->info->centroids,glob->arrays->dmCommand,threadInfo->err);
    }else{
      dprintf("Paused %d %d\n",threadInfo->threadno,niters);
    }
    //////////////////////////////////////////////////////////////////
    //wait for all threads to finish this frame
    if(pthread_mutex_lock(&glob->endMutex))//091109[threadInfo->mybuf]))
      printf("pthread_mutex_lock error in processFrame4:  %s\n",strerror(errno));
    glob->threadCountFinished++;//091109[threadInfo->mybuf]++;
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
      glob->pxlCentInputError|=info->pxlCentInputError;
      info->pxlCentInputError=0;//reset for next time.
      if(info->pause==0){//not paused...
	if(glob->nsteps>1)
	  glob->nsteps--;
	else if(glob->nsteps==1){
	  info->pause=1;
	  glob->nsteps=0;
	  //doEndFrame=1;
	  if(glob->ppause!=NULL)
	    *(glob->ppause)=1;//set the shm paused flag
	}
      }
    }else{
      info->threadCountFinished++;
    }
    //now the last thread to finish overall has some stuff to do...
    //091109if(glob->threadCountFinished[threadInfo->mybuf]==glob->nthreads){//last thread to finish
    if(glob->threadCountFinished==glob->nthreads){//last thread to finish
      glob->doswitch=0;//091109[threadInfo->mybuf]=0;
      glob->threadCount=0;//091109[threadInfo->mybuf]=0;

      glob->threadCountFinished=0;//091109[threadInfo->mybuf]=0;
      glob->camReadCnt=0;
      threadInfo->info->pxlCentInputError=glob->pxlCentInputError;
      gettimeofday(&thistime,NULL);
      timestamp=thistime.tv_sec+thistime.tv_usec*1e-6;
      glob->precomp->post.timestamp=timestamp;
      if(threadInfo->info->pause==0){// || doEndFrame==1){
	endFrame(threadInfo);//not paused...
      }else{//paused
	glob->thisiter++;//have to increment this so that the frameno changes in the circular buffer, OTHERWISE, the buffer may not get written
	//Note, myiter doesn't get incremented here, and neither do the .so library frameno's so, once unpaused, the thisiter value may decrease back to what it was.
	writeStatusBuf(glob,1,*glob->closeLoop);
	circAdd(glob->rtcStatusBuf,glob->statusBuf,timestamp,glob->thisiter);
      }
      threadInfo->info->pxlCentInputError=0;
      glob->pxlCentInputError=0;

      //timing info stuff...
      if(glob->starttime==0){//first one....tv_sec
	glob->starttime=timestamp;//thistime;
	dtime=0;
	circAdd(glob->rtcTimeBuf,&dtime,timestamp,glob->thisiter);
	//gettimeofday(&glob->starttime,NULL);
      }else{
	//gettimeofday(&thistime,NULL);
	dtime=timestamp-glob->starttime;//thistime.tv_sec*1e6+thistime.tv_usec-glob->starttime.tv_sec*1e6-glob->starttime.tv_usec;
	glob->frameTime=dtime;
	glob->sumtime+=dtime;
	glob->sum2time+=dtime*dtime;
	glob->ntime++;
	if(dtime>glob->maxtime && glob->thisiter>10){
	  glob->maxtime=dtime;
	  glob->maxtimeiter=glob->thisiter;
	}
	glob->starttime=timestamp;//thistime;
	if(glob->printTime){
	  printf("%10fs\t%10fHz\t%10fs           \r",dtime,1/dtime,glob->maxtime);
	  fflush(NULL);
	}
	circAdd(glob->rtcTimeBuf,&dtime,timestamp,glob->thisiter);
	//glob->thisiter++;//This is now updated earlier, when first pixels arrive...
      }
      //Now, we can check to see if a buffer swap is required.  If not, then do the start of frame stuff here, if so, then set correct flags...
      if(getSwitchRequested(glob)){//a new parameter buffer is ready
	glob->doswitch=1;
      }else{//signal to cameras etc.
	setFrameno(glob);
	startNewFrame(threadInfo);
      }

      //tell the other threads that we've finished.
      pthread_cond_broadcast(&(glob->frameRunningCond));//091109[threadInfo->mybuf]));

      
    }else{//wait for the last thread to finish
      //091109if(pthread_cond_wait(&(glob->frameRunningCond[threadInfo->mybuf]),&(glob->endMutex[threadInfo->mybuf])))
      if(pthread_cond_wait(&(glob->frameRunningCond),&(glob->endMutex)))
	printf("cond_wait error frameRunning\n");
    }
    pthread_mutex_unlock(&glob->endMutex);//091109[threadInfo->mybuf]);
    
    
    if(niters>0)
      niters--;
  }
  printf("processFrame ending - done thread %d\n",threadInfo->threadno);
  if(threadInfo->threadno==0){
    printf("mean time %gus (%d iters), stdev %gus, max %gus (iter %d)\n",glob->sumtime/glob->ntime,glob->ntime,sqrt(glob->sum2time/glob->ntime-(glob->sumtime/glob->ntime)*(glob->sumtime/glob->ntime)),glob->maxtime,glob->maxtimeiter);
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

