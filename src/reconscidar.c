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
   This is a library that can be used for on-line scidar processing.
Note - it cannot be used for closed loop control - does not compute actuators.

What it does:
Takes the first 2 subaps (which may be for different cameras).  These are the 2 scidar images.

FFTs both of these into half-complex format.

Stores img1 in a 3 element circular buffer.

Stores img2 in a 5 element circular buffer.

Takes the oldest entry of the 3 element buffer, and correlates this with all 5 entries of the other buffer.

Stores these 5 correlations in rtcScidarImgBuf.

Does a blob detection on these 5 images, to get wind velocities.  These are stored in rtcScidarVelocityBuf

On the 3rd image (the one created by correlation of same-time images) does a leaky-box integration, gets the radial profile in the separation direction of the stars, and normalises this to get the Cn2 profile.  This is stored in rtcScidarProfileBuf.



*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include "agbcblas.h"
#include "darc.h"
#include "rtcrecon.h"
#include "buffer.h"
typedef enum{RECONMODE_SIMPLE,RECONMODE_TRUTH,RECONMODE_OPEN,RECONMODE_OFFSET}ReconModeType;

typedef enum{
  BLEEDGAIN,
  BLEEDGROUPS,
  DECAYFACTOR,
  GAINE,
  GAINRECONMXT,
  NACTS,
  RECONSTRUCTMODE,
  V0,
  //Add more before this line.
  RECONNBUFFERVARIABLES//equal to number of entries in the enum
}RECONBUFFERVARIABLEINDX;

#define reconMakeNames() bufferMakeNames(RECONNBUFFERVARIABLES,"bleedGain","bleedGroups","decayFactor","gainE","gainReconmxT","nacts","reconstructMode","v0")



typedef struct{
  ReconModeType reconMode;
  float *gainE;
  int dmCommandArrSize;
  float **dmCommandArr;
  float *rmxT;
  float *v0;
  float bleedGain;//OverNact;
  float *bleedGainArr;//OverNact;
  int *bleedGroupArr;
  int bleedGroups;
  float *bleedVal;
  int bleedValSize;
  //float midRangeTimesBleed;
  float *decayFactor;
  int nacts;
  int totCents;
}ReconStructEntry;

typedef struct{
  ReconStructEntry rs[2];
  int buf;//current buffer being used
  int postbuf;//current buffer for post processing threads.
  //int swap;//set if need to change to the other buffer.
  int dmReady;
  float *latestDmCommand;
  int latestDmCommandSize;
  pthread_mutex_t dmMutex;
  pthread_cond_t dmCond;
  //int bufindx[RECONNBUFFERVARIABLES];
  circBuf *rtcErrorBuf;
  int nthreads;
  char *paramNames;
  int index[RECONNBUFFERVARIABLES];
  void *values[RECONNBUFFERVARIABLES];
  char dtype[RECONNBUFFERVARIABLES];
  int nbytes[RECONNBUFFERVARIABLES];
  arrayStruct *arr;
}ReconStruct;

/**
   Called to free the reconstructor module when it is being closed.
*/
int reconClose(void **reconHandle){//reconHandle is &globals->reconStruct.
  ReconStruct *reconStruct=(ReconStruct*)*reconHandle;
  ReconStructEntry *rs;
  int i;
  printf("Closing reconlibrary\n");
  if(reconStruct!=NULL){
    if(reconStruct->paramNames!=NULL)
      free(reconStruct->paramNames);
    pthread_mutex_destroy(&reconStruct->dmMutex);
    pthread_cond_destroy(&reconStruct->dmCond);
    rs=&reconStruct->rs[0];
    if(reconStruct->latestDmCommand!=NULL)
      free(reconStruct->latestDmCommand);
    if(rs->dmCommandArr!=NULL){
      for(i=0; i<reconStruct->nthreads; i++){
	if(rs->dmCommandArr[i]!=NULL)
	  free(rs->dmCommandArr[i]);
      }
      free(rs->dmCommandArr);
    }
    rs=&reconStruct->rs[1];
    if(rs->dmCommandArr!=NULL){
      for(i=0; i<reconStruct->nthreads; i++){
	if(rs->dmCommandArr[i]!=NULL)
	  free(rs->dmCommandArr[i]);
      }
      free(rs->dmCommandArr);
    }
    free(reconStruct);
  }

  *reconHandle=NULL;
  printf("Finished reconClose\n");
  return 0;
}



/**
   Called asynchronously, whenever new parameters are ready.
   Once this returns, a call to swap buffers will be issued.
   (actually, at the moment, this is called synchronously by first thread when a buffer swap is done).
*/
int reconNewParam(void *reconHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr,int totCents){
  int j=0,err=0;
  int nb;
  //globalStruct *globals=threadInfo->globals;
  //infoStruct *info=threadInfo->info;//totcents and nacts
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;//threadInfo->globals->reconStruct;
  //int *indx=reconStruct->bufindx;
  //Use the buffer not currently in use.
  ReconStructEntry *rs;
  RECONBUFFERVARIABLEINDX i;
  int nfound;
  int *nbytes=reconStruct->nbytes;
  void **values=reconStruct->values;
  char *dtype=reconStruct->dtype;
  reconStruct->arr=arr;
  //swap the buffers...
  reconStruct->buf=1-reconStruct->buf;
  rs=&reconStruct->rs[reconStruct->buf];
  rs->totCents=totCents;
  nfound=bufferGetIndex(pbuf,RECONNBUFFERVARIABLES,reconStruct->paramNames,reconStruct->index,reconStruct->values,reconStruct->dtype,reconStruct->nbytes);
  if(nfound!=RECONNBUFFERVARIABLES){
    err=-1;
    printf("Didn't get all buffer entries for recon module:\n");
    for(j=0; j<RECONNBUFFERVARIABLES; j++){
      if(reconStruct->index[j]<0)
	printf("Missing %16s\n",&reconStruct->paramNames[j*BUFNAMESIZE]);
    }
  }
  if(err==0){
    i=NACTS;
    if(dtype[i]=='i' && nbytes[i]==4){
      rs->nacts=*((int*)values[i]);
    }else{
      printf("nacts error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"nacts error");
      err=NACTS;
    }
    i=RECONSTRUCTMODE;
    nb=nbytes[i];
    if(dtype[i]=='s'){
      if(strncmp((char*)values[i],"simple",nb)==0){
	rs->reconMode=RECONMODE_SIMPLE;
      }else if(strncmp((char*)values[i],"truth",nb)==0){
	rs->reconMode=RECONMODE_TRUTH;
      }else if(strncmp((char*)values[i],"open",nb)==0){
	rs->reconMode=RECONMODE_OPEN;
      }else if(strncmp((char*)values[i],"offset",nb)==0){
	rs->reconMode=RECONMODE_OFFSET;
      }else{
	printf("reconstructMode not interpreted, assuming simple\n");
	rs->reconMode=RECONMODE_SIMPLE;
      }
    }else{
      printf("reconstructMode error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"reconstructMode error");
      err=RECONSTRUCTMODE;
    }
    i=GAINRECONMXT;
    if(dtype[i]=='f' && nbytes[i]/sizeof(float)==rs->totCents*rs->nacts){
      rs->rmxT=(float*)values[i];
    }else{
      printf("gainReconmxT error %c %d %d %d\n",dtype[i],nbytes[i],rs->totCents,rs->nacts);
      err=GAINRECONMXT;
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"gainReconmxT error");
    }
    i=GAINE;
    if(nbytes[i]==0)
      rs->gainE=NULL;
    else if(dtype[i]=='f' && nbytes[i]==sizeof(float)*rs->nacts*rs->nacts){
      rs->gainE=(float*)values[i];
    }else{
      rs->gainE=NULL;
      printf("gainE error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"gainE");
      err=GAINE;
    }
    i=V0;
    if(dtype[i]=='f' && nbytes[i]==sizeof(float)*rs->nacts){
      rs->v0=(float*)values[i];
    }else{
      printf("v0 error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"v0 error");
      err=V0;
    }
    i=BLEEDGAIN;
    if(dtype[i]=='f'){
      if(nbytes[i]==sizeof(float)){
	rs->bleedGain=(*((float*)values[i]));///rs->nacts;
	rs->bleedGainArr=NULL;
	rs->bleedGroups=1;
      }else{
	rs->bleedGainArr=((float*)values[i]);
	rs->bleedGroups=nbytes[i]/sizeof(float);
      }
    }else{
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"bleedGain error");
      printf("bleedGain error\n");
      rs->bleedGainArr=NULL;
      rs->bleedGroups=1;
      err=BLEEDGAIN;
    }
    i=BLEEDGROUPS;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)*rs->nacts){
      rs->bleedGroupArr=(int*)values[i];
    }else{
      rs->bleedGroupArr=NULL;
      if(nbytes[i]!=0){
	printf("bleedGroups error\n");
	writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"bleedGroups error");
	err=BLEEDGROUPS;
      }
    }

    i=DECAYFACTOR;
    if(nbytes[i]==0){
      rs->decayFactor=NULL;
    }else if(dtype[i]=='f' && nbytes[i]==sizeof(float)*rs->nacts){
      rs->decayFactor=(float*)values[i];
    }else{
      rs->decayFactor=NULL;
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"decayFactor error");
      printf("decayFactor error\n");
      err=DECAYFACTOR;
    }
    /*
    i=MIDRANGE;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      rs->midRangeTimesBleed=(*((int*)values[i]))*rs->nacts*rs->bleedGainOverNact;
    }else{
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"midRangeValue error");
      printf("midrange error\n");
      err=MIDRANGE;
      }*/
  }
  //No need to get the lock here because this and newFrame() are called inside glob->libraryMutex.
  reconStruct->dmReady=0;
  if(rs->dmCommandArrSize<sizeof(float)*rs->nacts){
    rs->dmCommandArrSize=sizeof(float)*rs->nacts;
    for(i=0; i<reconStruct->nthreads; i++){
      if(rs->dmCommandArr[i]!=NULL)
	free(rs->dmCommandArr[i]);
      if((rs->dmCommandArr[i]=calloc(sizeof(float),rs->nacts))==NULL){
	printf("Error allocating recon dmCommand memory\n");
	err=-2;
	rs->dmCommandArrSize=0;
      }
    }
  }
  if(reconStruct->latestDmCommandSize<sizeof(float)*rs->nacts){
    reconStruct->latestDmCommandSize=sizeof(float)*rs->nacts;
    if(reconStruct->latestDmCommand!=NULL)
      free(reconStruct->latestDmCommand);
    if((reconStruct->latestDmCommand=calloc(rs->nacts,sizeof(float)))==NULL){
      printf("Error allocating latestDmCommand memory\n");
      err=-3;
      reconStruct->latestDmCommandSize=0;
    }
  }
  if(err==0 && rs->bleedGroupArr!=NULL){
    nb=0;
    for(j=0;j<rs->nacts;j++){
      if(rs->bleedGroupArr[j]>nb)
	nb=rs->bleedGroupArr[j];
    }
    nb++;
    if(rs->bleedGroups>1 && nb>rs->bleedGroups){
      printf("Error - bleed groups not consistent with bleed gain\n");
      err=1;
    }else{
      rs->bleedGroups=nb;
    }
  }
  if(err==0){
    if(rs->bleedValSize<rs->bleedGroups){
      rs->bleedValSize=rs->bleedGroups;
      if(rs->bleedVal!=NULL)
	free(rs->bleedVal);
      if((rs->bleedVal=calloc(sizeof(float),rs->bleedGroups))==NULL){
	printf("error allocing bleedVal\n");
	rs->bleedValSize=0;
	err=1;
      }
    }
  }

  return err;
}


/**
   Initialise the reconstructor module
 */
int reconOpen(char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **reconHandle,int nthreads,unsigned int frameno,unsigned int **reconframeno,int *reconframenoSize,int totCents){
  //Sort through the parameter buffer, and get the things we need, and do 
  //the allocations we need.
  ReconStruct *reconStruct;
  //ReconStructEntry *rs;
  int err=0;
  if((reconStruct=calloc(sizeof(ReconStruct),1))==NULL){
    printf("Error allocating recon memory\n");
    *reconHandle=NULL;
    //threadInfo->globals->reconStruct=NULL;
    return 1;
  }
  //threadInfo->globals->reconStruct=(void*)reconStruct;
  *reconHandle=(void*)reconStruct;
  reconStruct->buf=1;
  reconStruct->arr=arr;
  reconStruct->nthreads=nthreads;//this doesn't change.
  reconStruct->rtcErrorBuf=rtcErrorBuf;
  reconStruct->paramNames=reconMakeNames();
  if((reconStruct->rs[0].dmCommandArr=calloc(sizeof(float*),nthreads))==NULL){
    printf("Error allocating recon memory[0]\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if((reconStruct->rs[1].dmCommandArr=calloc(sizeof(float*),nthreads))==NULL){
    printf("Error allocating recon memory[1]\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }

  err=reconNewParam(*reconHandle,pbuf,frameno,arr,totCents);//this will change ->buf to 0.
  //rs->swap=0;//no - we don't need to swap.
  //rs=&reconStruct->rs[reconStruct->buf];
  if(err!=0){
    printf("Error in recon...\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  //the condition variable and mutex don't need to be buffer swaped...
  if(pthread_mutex_init(&reconStruct->dmMutex,NULL)){
    printf("Error init recon mutex\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if(pthread_cond_init(&reconStruct->dmCond,NULL)){
    printf("Error init recon cond\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  return 0;
}



/**
   Called by single thread at the start of each frame.
   This thread is not a subap processing thread.
   If doing a param buffer swap, this is called after the swap has completed.
*/
int reconNewFrame(void *reconHandle,unsigned int frameno,double timestamp){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  ReconStructEntry *rs;
  int i;
  float *dmCommand=reconStruct->arr->dmCommand;
  //if(reconStruct->swap){
  // reconStruct->swap=0;
  // reconStruct->buf=1-reconStruct->buf;
  //}
  rs=&reconStruct->rs[reconStruct->buf];
  //Now wake up the thread that does the initial processing...
  //No need to get a mutex here, because the preprocessing thread must be sleeping.
  //We do this processing in a separate thread because it has a while to complete, so we may as well let subap processing threads get on with their tasks...
  //pthread_cond_signal(&reconStruct->dmCond);
  if(rs->reconMode==RECONMODE_SIMPLE){//simple open loop
    //memset(p->dmCommand,0,sizeof(float)*p->nacts);
    memcpy(dmCommand,rs->v0,sizeof(float)*rs->nacts);
 }else if(rs->reconMode==RECONMODE_TRUTH){//closed loop
    if(rs->decayFactor==NULL){
      memcpy(dmCommand,reconStruct->latestDmCommand,sizeof(float)*rs->nacts);
    }else{
      for(i=0; i<rs->nacts; i++){
	dmCommand[i]=rs->decayFactor[i]*reconStruct->latestDmCommand[i];
      }
    }
  }else if(rs->reconMode==RECONMODE_OPEN){//reconmode_open
    //initialise by copying v0 into the result.
    //This line removed 100528 after discussion with Eric Gendron.
    //memcpy(glob->arrays->dmCommand,rs->v0,sizeof(float)*rs->nacts);
    //beta=1.;
    //memset(glob->arrays->dmCommand,0,sizeof(float)*rs->nacts);
    //Now: dmcommand=v0+dot(gainE,latestDmCommand)
    if(rs->gainE!=NULL){
      agb_cblas_sgemvRowNN1N101(rs->nacts,rs->gainE,reconStruct->latestDmCommand,dmCommand);
    }else{//gainE==NULL...
      memcpy(dmCommand,reconStruct->latestDmCommand,sizeof(float)*rs->nacts);
      //and add v0
      agb_cblas_saxpy111(rs->nacts,rs->v0,dmCommand);
    }

  }else{//reconmode_offset
    memcpy(dmCommand,rs->v0,sizeof(float)*rs->nacts);
  }	
  //set the DM arrays ready.
  if(pthread_mutex_lock(&reconStruct->dmMutex))
    printf("pthread_mutex_lock error in setDMArraysReady: %s\n",strerror(errno));
  reconStruct->dmReady=1;
  //wake up any of the subap processing threads that are waiting.
  pthread_cond_broadcast(&reconStruct->dmCond);
  pthread_mutex_unlock(&reconStruct->dmMutex);
  return 0;
}

/**
   Called once per thread at the start of each frame, possibly simultaneously.
*/
int reconStartFrame(void *reconHandle,int cam,int threadno){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;//threadInfo->globals->reconStruct;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->buf];
  memset((void*)(rs->dmCommandArr[threadno]),0,rs->nacts*sizeof(float));
  return 0;
}


/**
   Called multiple times by multiple threads, whenever new slope data is ready
   centroids may not be complete, and writing to dmCommand is not thread-safe without locking.
*/
int reconNewSlopes(void *reconHandle,int cam,int centindx,int threadno,int nsubapsDoing){
  int step;//number of rows to do in mmx...
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->buf];
  float *centroids=reconStruct->arr->centroids;
  //Here, we want to FFT the first 2 subaps into Half Complex format.
  int usedsubap=centindx/2;
  int ndone=0;
  int sfno=0;
  int subapcnt=0;
  int cam=0;
  for(i=0;i<nsubapsDoing;i++){
    usedsubap=centindx/2+i;
    //find subap number.
    while(subapcnt<=usedsubap && sfno<reconStruct->totSubaps){
      subapcnt+=subapFlag[sfno];
      //also need to get camera number, and pixel offset of this camera.
      while(sfno>reconStruct->nsubCum[cam] && cam<ncam)
	cam++;
      sfno++;
    }
    sfno--;//this is the sub-aperture number
    if(sfno==reconStruct->subap1){
      //process...
      loc=&arr->subapLocation[sfno*6];
      calPxls=&arr->calpxlbuf[npxlCum[cam]];
      nsubx=(loc[4]-loc[3])/loc[5];
      nsuby=(loc[1]-loc[0])/loc[2];
      pos=0;
      for(i=loc[0];i<loc[1];i+=loc[2]){
	for(j=loc[0];j<loc[1];j+=loc[2]){
	  subap[pos]=calPxls[i*npxlx[cam]+j];
	  pos++;
	}
      }
      todo("check plan exists");
      dofft(subap,rstr->corrCircBuf[0][rstr->circBufPos[0]]);
      //Now compute correlation of oldest entry of circBuf0 with 3 oldest entries of circBuf1.
      for(i=0;i<3;i++){
	correlateScidar(rstr->corrCircBuf[0][(rstr->circBufPos[0]+1)%3],rstr->corrCircBuf[1][(rstr->circBufPos[1]-2-i)%5],rstr->outBuf[i+2]);
	//And integrate the output buffer.
	integrateScidar(rstr->outBuf[i+2],rstr->outBufIntegrated[i+2]);
      }
    }else if(sfno==reconStruct->subap2){
      //process...
      loc=&arr->subapLocation[sfno*6];
      calPxls=&arr->calpxlbuf[npxlCum[cam]];
      nsubx=(loc[4]-loc[3])/loc[5];
      nsuby=(loc[1]-loc[0])/loc[2];
      pos=0;
      //this one is mirrored.
      for(i=loc[0];i<loc[1];i+=loc[2]){
	for(j=loc[0];j<loc[1];j+=loc[2]){
	  subap[i*nsubx+nsubx-j-1]=calPxls[i*npxlx[camno]+j];
	  pos++;
	}
      }
      todo("check plan exists");
      dofft(subap,rstr->corrCircBuf[1][rstr->circBufPos[1]]);
      //Now compute correlation of oldest entry of circBuf0 with 2 newest entries of circBuf1 (which includes the one just done).
      for(i=0;i<2;i++){
	correlateScidar(rstr->corrCircBuf[0][(rstr->circBufPos[0]+1)%3],rstr->corrCircBuf[1][(rstr->circBufPos[1]-i)%5],rstr->outBuf[i]);
	//And integrate the output buffer.
	integrateScidar(rstr->outBuf[i],rstr->outBufIntegrated[i]);
      }
      
    }

  }
  return 0;
}

/**
   Called once for each thread at the end of a frame
*/
/*
int reconEndFrame(void *reconHandle,int cam,int threadno,int err){
  //dmCommand=glob->arrays->dmCommand;
  //globalStruct *glob=threadInfo->globals;
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->buf];
  float *dmCommand=reconStruct->arr->dmCommand;
  if(pthread_mutex_lock(&reconStruct->dmMutex))
    printf("pthread_mutex_lock error in copyThreadPhase: %s\n",strerror(errno));
  if(reconStruct->dmReady==0)//wait for the precompute thread to finish (it will call setDMArraysReady when done)...
    if(pthread_cond_wait(&reconStruct->dmCond,&reconStruct->dmMutex))
      printf("pthread_cond_wait error in copyThreadPhase: %s\n",strerror(errno));
  //now add threadInfo->dmCommand to threadInfo->info->dmCommand.
  agb_cblas_saxpy111(rs->nacts,rs->dmCommandArr[threadno],dmCommand);
  pthread_mutex_unlock(&reconStruct->dmMutex);
  return 0;
}
*/
/**
   Called by single thread per frame at the end of frame.
   This is called by a subaperture processing thread.
   The bare minimum should be placed here, as most processing should be done in the reconFrameFinished function instead, which doesn't hold up processing.
*/
/*
int reconFrameFinishedSync(void *reconHandle,int err,int forcewrite){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  //float *centroids=reconStruct->arr->centroids;
  //float *dmCommand=reconStruct->arr->dmCommand;
  reconStruct->dmReady=0;
  reconStruct->postbuf=reconStruct->buf;
  return 0;
}
*/
/**
   Called by single thread per frame - end of frame
   Do any post processing here.
   This it typically called by a non-subaperture processing thread.
   At the end of this method, dmCommand must be ready...
   Note, while this is running, subaperture processing of the next frame may start.

   Here, we compute Cn2 and velocities.
*/
int reconFrameFinished(void *reconHandle,int *err){//globalStruct *glob){
  //Note: dmCommand=glob->arrays->dmCommand.
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;//glob->reconStruct;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->postbuf];
  float *bleedVal=rs->bleedVal;
  int i,bleedGroup;
  float *dmCommand=reconStruct->arr->dmCommand;
  //move to next position of circular buffer, for next iteration.
  rstr->circBufPos[0]=(rstr->circBufPos[0]+1)%3;//3 entries long.
  rstr->circBufPos[1]=(rstr->circBufPos[1]+1)%5;//5 entries long.
  
  //Get Cn2 profile.
  for(i=0;i<npxly;i++)
    rstr->cn2[i]=rstr->outBufIntegrated[2][(npxly+i)*2*npxlx+npxlx];

  circAdd(rstr->rtcCn2Buf,rstr->cn2);
  circAdd(rstr->rtcScidarCorrBuf,rstr->outBuf);
  circAdd(rstr->rtcScidarIntegratedCorrBuf,rstr->outBufIntegrated);
  //Now blob detect.  On [2] (correlation of images from same time), they will be on the y axis, i.e. maxima of Cn2.
  for(i=0;i<rstr->nblobs;i++){
    getRemovePeak(rstr->cn2,&peakPos[2*2*rstr->nblobs+i*2+1]);
    peakPos[2*2*rstr->nblobs+i*2]=0;
  }
  for(j=0;j<5;j++){
    if(j==2)
      continue;
    for(i=0;i<rstr->nblobs;i++)
      getRemoveBlob(rstr->outBufIntegrated[2],&peakPos[j*2*rstr->nblobs+i*2],&peakPos[j*2*rstr->nblobs+i*2+1]);
  }
  //Now look for moving layers.
  //Here, we use [0-2], [1-3] and [2-4] and look for a line in any of these.  If found, this then becomes a moving layer.
  //For each of the layers in [2], need to look for a straight line with uniformly spaced points in the other [?].  This is then the layer moving, and the velocity is given by spacing and by angle of line.
  pos=0;
  for(h=0;h<3;h++){
    for(i=0;i<rstr->nblobs;i++){
      for(j=0;j<rstr->nblobs;j++){
	//get dist to first peak.
	dx=peakPos[h*2*rstr->nblobs+i*2]-peakPos[(h+1)*2*rstr->nblobs+j*2];
	dy=peakPos[h*2*rstr->nblobs+i*2+1]-peakPos[(h+1)*2*rstr->nblobs+j*2+1];
	//compute pos of next (if a real signal):
	posx=peakPos[h*2*rstr->nblobs+i*2]+dx*2;
	posy=peakPos[h*2*rstr->nblobs+i*2+1]+dy*2;
	//see if any are within range...
	for(k=0;k<rstr->nblobs;k++){
	  if(fabsf(posx-peakPos[(h+2)*2*rstr->nblobs+k*2])<maxDist && fabsf(posy-peakPos[(h+2)*2*rstr->nblobs+k*2+1])<maxDist){
	    //line of 3 found... so treat this as a signal.
	    //But - what do we do if others are also found?  Ignore for now - we'll just have space to store one spurious for each (ie 2*nblobs in total - though only nblobs are expected).
	    //Compute angle of line, and mean dx/dy
	    dx2=peakPos[h*2*rstr->nblobs+i*2]-peakPos[(h+2)*2*rstr->nblobs+k*2];
	    dy2=peakPos[h*2*rstr->nblobs+i*2+1]-peakPos[(h+2)*2*rstr->nblobs+k*2+1];
	    if(pos<rstr->nblobs*2){
	      v[pos]=sqrt(dx2*dx2+dy2*dy2);
	      theta[pos]=atan2(dy2,dx2);
	    }else{
	      printf("Ignoring speed %g, angle %g: Too many computed\n",sqrt(dx2*dx2+dy2*dy2),atan2(dy2,dx2));
	    }
	    pos++;
	  }
	}

      nline=2;
      for(k=0;k<5;k++){
	if(k==2||k==3)
	  continue;
	nline+=peakAt(peakPos[2*2*rstr->nblobs+i*2]+dx*(k-2),peakPos[2*2*rstr->nblobs+i*2+1]+dy*(k-2));
      }
      if(nline==5){//found 5 in a line - this must be a true velocity.
	
	continue;
      }


  if(rs->bleedGain!=0. || rs->bleedGainArr!=NULL){//compute the bleed value
    memset(bleedVal,0,sizeof(float)*rs->bleedGroups);
    for(i=0; i<rs->nacts; i++){
      if(rs->bleedGroupArr!=NULL)
	bleedGroup=rs->bleedGroupArr[i];
      else
	bleedGroup=0;
      bleedVal[bleedGroup]+=dmCommand[i]-rs->v0[i];
    }
    if(rs->bleedGainArr==NULL){
      for(i=0;i<rs->bleedGroups;i++)
	bleedVal[i]*=rs->bleedGain;
    }else{
      for(i=0;i<rs->bleedGroups;i++)
	bleedVal[i]*=rs->bleedGainArr[i];
    }
    for(i=0; i<rs->nacts; i++){
      if(rs->bleedGroupArr!=NULL)
	bleedGroup=rs->bleedGroupArr[i];
      else
	bleedGroup=0;
      dmCommand[i]-=bleedVal[bleedGroup];
    }
  }
  if(*err==0)
    memcpy(reconStruct->latestDmCommand,dmCommand,sizeof(float)*rs->nacts);
  return 0;
}
/**
   Called by the single thread per frame, when the actuator values aren't being sent to the dm - so we need to reset ourselves.
   May not be called at all, if the loop is closed.
   Not called by a subap processing thread - subap processing for next frame have have started before this is called...
   Can use this to reset the scidar?
*/
int reconOpenLoop(void *reconHandle){//globalStruct *glob){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;//glob->reconStruct;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->postbuf];
  //memcpy(reconStruct->latestDmCommand,rs->v0,sizeof(float)*rs->nacts);
  return 0;

}
 
