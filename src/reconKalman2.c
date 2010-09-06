/**
   This is a library that can be used for a simple MVM.
*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#ifdef USEAGBBLAS
#include "agbcblas.h"
#else
#include <gsl/gsl_cblas.h>
typedef enum CBLAS_ORDER CBLAS_ORDER;
typedef enum CBLAS_TRANSPOSE CBLAS_TRANSPOSE;
#endif

#include "darc.h"
#include "circ.h"
#define NEWRECON
#include "recon.h"


typedef enum{
  KALMANPHASESIZE,
  KALMANHINFT,
  KALMANHINFDM,
  KALMANATUR,
  KALMANINVN,
  V0,
  BLEEDGAIN,
  NACTS,
  //Add more before this line.
  RECONNBUFFERVARIABLES//equal to number of entries in the enum
}RECONBUFFERVARIABLEINDX;
char *RECONPARAM[]={"kalmanPhaseSize","kalmanHinfT","kalmanHinfDM","kalmanAtur","kalmanInvN","v0","bleedGain","nacts"};


typedef struct{
  int totCents;
  int kalmanPhaseSize;
  float *kalmanHinfT;
  float *kalmanHinfDM;
  float *kalmanAtur;
  float *kalmanInvN;
  float *XpredArr;
  int XpredArrSize;
  int XpredSize;
  float *v0;
  float bleedGainOverNact;
  //float midRangeTimesBleed;
  int nacts;
}ReconStructEntry;

typedef struct{
  ReconStructEntry rs[2];
  int buf;//current buffer being used
  int postbuf;//current buffer for post processing threads.
  //int swap;//set if need to change to the other buffer.
  int dmReady;
  float *Xpred;
  int XpredSize;
  pthread_mutex_t dmMutex;
  pthread_cond_t dmCond;
  int bufindx[RECONNBUFFERVARIABLES];
  circBuf *rtcErrorBuf;
  int nthreads;
}ReconStruct;

/**
   Called to free the reconstructor module when it is being closed.
*/
int reconFree(void **reconHandle){
  ReconStruct *reconStruct=(ReconStruct*)*reconHandle;
  ReconStructEntry *rs;
  printf("Closing kalman reconstruction library\n");
  if(reconStruct!=NULL){
    pthread_mutex_destroy(&reconStruct->dmMutex);
    pthread_cond_destroy(&reconStruct->dmCond);
    rs=&reconStruct->rs[0];
    if(reconStruct->Xpred!=NULL)
      free(reconStruct->Xpred);
    if(rs->XpredArr!=NULL)
      free(rs->XpredArr);
    rs=&reconStruct->rs[1];
    if(rs->XpredArr!=NULL)
      free(rs->XpredArr);
    free(reconStruct);
  }
  *reconHandle=NULL;
  printf("Finished reconFree\n");
  return 0;
}


/**
   Called asynchronously, whenever new parameters are ready.
   Once this returns, a call to swap buffers will be issued.
   (actually, at the moment, this is called synchronously by first thread when a buffer swap is done).
*/
int reconNewParam(char *buf,void *reconHandle,unsigned int frameno,int totCents){
  int j=0,err=0;
  int nbytes;
  //globalStruct *globals=threadInfo->globals;
  //infoStruct *info=threadInfo->info;
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;//threadInfo->globals->reconStruct;
  int *indx=reconStruct->bufindx;
  //Use the buffer not currently in use.
  ReconStructEntry *rs;
  RECONBUFFERVARIABLEINDX i;
  //swap the buffers...
  reconStruct->buf=1-reconStruct->buf;
  rs=&reconStruct->rs[reconStruct->buf];
  rs->totCents=totCents;
  memset(indx,-1,sizeof(int)*RECONNBUFFERVARIABLES);

  //first run through the buffer getting the indexes of the params we need.
  while(j<NHDR && buf[j*31]!='\0'){
    if(strncmp(&buf[j*31],"kalmanPhaseSize",31)==0){
      indx[KALMANPHASESIZE]=j;
    }else if(strncmp(&buf[j*31],"kalmanHinfT",31)==0){
      indx[KALMANHINFT]=j;
    }else if(strncmp(&buf[j*31],"kalmanHinfDM",31)==0){
      indx[KALMANHINFDM]=j;
    }else if(strncmp(&buf[j*31],"kalmanAtur",31)==0){
      indx[KALMANATUR]=j;
    }else if(strncmp(&buf[j*31],"kalmanInvN",31)==0){
      indx[KALMANINVN]=j;
    }else if(strncmp(&buf[j*31],"v0",31)==0){
      indx[V0]=j;
    }else if(strncmp(&buf[j*31],"bleedGain",31)==0){
      indx[BLEEDGAIN]=j;
    }else if(strncmp(&buf[j*31],"nacts",31)==0){
      indx[NACTS]=j;
    }
    j++;
  }
  for(j=0; j<RECONNBUFFERVARIABLES; j++){
    if(indx[j]==-1){
      //if(updateIndex){
      printf("ERROR buffer index %d\n",j);
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"Error in recon parameter buffer: %s",RECONPARAM[j]);
      //}
      err=-1;
    }
  }
  if(err==0){
    i=NACTS;
    if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==4){
      rs->nacts=*((int*)(buf+START[indx[i]]));
    }else{
      printf("nacts error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"nacts error");
      err=NACTS;
    }
    i=KALMANPHASESIZE;
    if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
      rs->kalmanPhaseSize=*((int*)(buf+START[indx[i]]));
    }else{
      printf("kalmanPhaseSize error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"kalmanPhaseSize error");
      err=KALMANPHASESIZE;
    }
    i=KALMANHINFT;
    if(buf[NHDR*31+indx[i]]=='f' && NBYTES[indx[i]]==rs->kalmanPhaseSize*3*rs->totCents*sizeof(float)){
      rs->kalmanHinfT=((float*)(buf+START[indx[i]]));
    }else{
      err=i;
      printf("kalmanHinfT error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"kalmanHinfT error");
    }
    i=KALMANHINFDM;
    if(buf[NHDR*31+indx[i]]=='f' && NBYTES[indx[i]]==rs->kalmanPhaseSize*rs->kalmanPhaseSize*3*sizeof(float)){
      rs->kalmanHinfDM=((float*)(buf+START[indx[i]]));
    }else{
      err=i;
      printf("kalmanHinfDM error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"kalmanHinfDM error");
    }
    i=KALMANATUR;
    if(buf[NHDR*31+indx[i]]=='f' && NBYTES[indx[i]]==rs->kalmanPhaseSize*rs->kalmanPhaseSize*sizeof(float)){
      rs->kalmanAtur=((float*)(buf+START[indx[i]]));
    }else{
      err=i;
      printf("kalmanAtur error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"kalmanAtur error");
    }
    i=KALMANINVN;
    nbytes=NBYTES[indx[i]];
    if(nbytes==0){
      rs->kalmanInvN=NULL;
    }else if(buf[NHDR*31+indx[i]]=='f' && NBYTES[indx[i]]==rs->nacts*rs->kalmanPhaseSize*sizeof(float)){
      rs->kalmanInvN=((float*)(buf+START[indx[i]]));
    }else{
      err=i;
      printf("kalmanInvN error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"kalmanInvN error");
    }
    i=V0;
    if(buf[NHDR*31+indx[i]]=='f' && NBYTES[indx[i]]==sizeof(float)*rs->nacts){
      rs->v0=(float*)(buf+START[indx[i]]);
    }else{
      printf("v0 error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"v0 error");
      err=V0;
    }
    i=BLEEDGAIN;
    if(buf[NHDR*31+indx[i]]=='f' && NBYTES[indx[i]]==sizeof(float)){
      rs->bleedGainOverNact=(*((float*)(buf+START[indx[i]])))/rs->nacts;
    }else{
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"bleedGain error");
      printf("bleedGain error\n");
      err=BLEEDGAIN;
    }
    /*
    i=MIDRANGE;
    if(buf[NHDR*31+indx[i]]=='i' && NBYTES[indx[i]]==sizeof(int)){
      rs->midRangeTimesBleed=(*((int*)(buf+START[indx[i]])))*rs->nacts*rs->bleedGainOverNact;
    }else{
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"midRangeValue error");
      printf("midrange error\n");
      err=MIDRANGE;
      }*/
  }



  if(reconStruct->XpredSize<rs->kalmanPhaseSize*3){
    if(reconStruct->Xpred!=NULL)
      free(reconStruct->Xpred);
    reconStruct->XpredSize=rs->kalmanPhaseSize*3;
    if((reconStruct->Xpred=calloc(reconStruct->XpredSize,sizeof(float)))==NULL){
      printf("malloc of Xpred failed\n");
      err=-3;
      reconStruct->XpredSize=0;
    }
  }
  
  if(rs->XpredArrSize<rs->kalmanPhaseSize*3*reconStruct->nthreads){
    if(rs->XpredArr!=NULL)
      free(rs->XpredArr);
    rs->XpredArrSize=rs->kalmanPhaseSize*3*reconStruct->nthreads;
    if((rs->XpredArr=calloc(rs->XpredArrSize,sizeof(float)))==NULL){
      printf("rs->XpredArr malloc error\n");
      err=-2;
      rs->XpredArrSize=0;
      rs->XpredSize=0;
    }else{
      rs->XpredSize=rs->kalmanPhaseSize*3;
    }
  }
  


  reconStruct->dmReady=0;

  /*
  if(rs->dmCommandArrSize<sizeof(float)*rs->nacts*reconStruct->nthreads){
    rs->dmCommandArrSize=sizeof(float)*rs->nacts*reconStruct->nthreads;
    if(rs->dmCommandArr!=NULL)
      free(rs->dmCommandArr);
    if((rs->dmCommandArr=calloc(sizeof(float)*rs->nacts,reconStruct->nthreads))==NULL){
      printf("Error allocating recon dmCommand memory\n");
      err=-2;
      rs->dmCommandArrSize=0;
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
    }*/
  //reconStruct->swap=1;
  return err;
}


/**
   Initialise the reconstructor module
 */
int reconInit(char *buf, void **reconHandle,int nthreads,int frameno,int totCents,circBuf *rtcErrorBuf){
  //Sort through the parameter buffer, and get the things we need, and do 
  //the allocations we need.
  ReconStruct *reconStruct;
  //ReconStructEntry *rs;
  int err=0;
  if((reconStruct=calloc(sizeof(ReconStruct),1))==NULL){
    printf("Error allocating recon memory\n");
    //threadInfo->globals->reconStruct=NULL;
    *reconHandle=NULL;
    return 1;
  }
  *reconHandle=(void*)reconStruct;
  reconStruct->buf=1;
  reconStruct->nthreads=nthreads;//this doesn't change.
  reconStruct->rtcErrorBuf=rtcErrorBuf;
  err=reconNewParam(buf,*reconHandle,frameno,totCents);//this will change ->buf to 0.
  //rs->swap=0;//no - we don't need to swap.
  //rs=&reconStruct->rs[reconStruct->buf];
  if(err!=0){
    printf("Error in recon...\n");
    reconFree(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  //the condition variable and mutex don't need to be buffer swaped...
  if(pthread_mutex_init(&reconStruct->dmMutex,NULL)){
    printf("Error init recon mutex\n");
    reconFree(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if(pthread_cond_init(&reconStruct->dmCond,NULL)){
    printf("Error init recon cond\n");
    reconFree(reconHandle);
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
int reconNewFrame(void *reconHandle,float *dmCommand){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  ReconStructEntry *rs;
#ifndef USEAGBBLAS
  CBLAS_ORDER order=CblasRowMajor;
  CBLAS_TRANSPOSE trans=CblasNoTrans;
  float alpha=1.,beta=1.;
  int inc=1;
#endif
  //if(reconStruct->swap){
  // reconStruct->swap=0;
  // reconStruct->buf=1-reconStruct->buf;
  //}
  rs=&reconStruct->rs[reconStruct->buf];
  //Now wake up the thread that does the initial processing...
  //No need to get a mutex here, because the preprocessing thread must be sleeping.
  //We do this processing in a separate thread because it has a while to complete, so we may as well let subap processing threads get on with their tasks...
  //pthread_cond_signal(&reconStruct->dmCond);
  //performn precomp=dot(DM,Xpred[phaseSize*2:])


  //first do some final post processing stuff...
  memcpy(&(reconStruct->Xpred[rs->kalmanPhaseSize*2]),&(reconStruct->Xpred[rs->kalmanPhaseSize*1]),sizeof(float)*rs->kalmanPhaseSize);
  memcpy(&(reconStruct->Xpred[rs->kalmanPhaseSize]),reconStruct->Xpred,sizeof(float)*rs->kalmanPhaseSize);
  //Perform Xpred[:phaseSize]=dot(Atur,Xpred[:phaseSize])
#ifdef USEAGBBLAS
  //alpha=1, beta=0
  agb_cblas_sgemvRowNN1N101(rs->kalmanPhaseSize,rs->kalmanAtur,&(reconStruct->Xpred[rs->kalmanPhaseSize]),reconStruct->Xpred);
  //alpha=1, beta=-1.
  agb_cblas_sgemvRowMN1N1m11(rs->kalmanPhaseSize*3,rs->kalmanPhaseSize,rs->kalmanHinfDM,&(reconStruct->Xpred[rs->kalmanPhaseSize*2]),reconStruct->Xpred);
  
#else
  beta=0.;
  cblas_sgemv(order,trans,rs->kalmanPhaseSize,rs->kalmanPhaseSize,alpha,rs->kalmanAtur,rs->kalmanPhaseSize,&(reconStruct->Xpred[rs->kalmanPhaseSize]),inc,beta,reconStruct->Xpred,inc);

  beta=-1.;
  cblas_sgemv(order,trans,rs->kalmanPhaseSize*3,rs->kalmanPhaseSize,alpha,rs->kalmanHinfDM,rs->kalmanPhaseSize,&(reconStruct->Xpred[rs->kalmanPhaseSize*2]),inc,beta,reconStruct->Xpred,inc);
#endif
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
int reconStartFrame(void *reconHandle,int threadno){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->buf];

  memset(&rs->XpredArr[threadno*rs->XpredSize],0,sizeof(float)*rs->XpredSize);
  return 0;
}

/**
   Called multiple times by multiple threads, whenever new slope data is ready
*/
int reconNewSlopes(void *reconHandle,int centindx,int threadno,int nsubapsDoing,float *centroids,float *dmCommand){
#ifndef USEAGBBLAS
  CBLAS_ORDER order=CblasColMajor;
  CBLAS_TRANSPOSE trans=CblasNoTrans;
  //infoStruct *info=threadInfo->info;
  float alpha=1.,beta=1.;
  int inc=1;
#endif
  int step;//=2;
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->buf];
  //infoStruct *info=threadInfo->info;
  step=2*nsubapsDoing;
#ifdef USEAGBBLAS
  agb_cblas_sgemvColMN1M111(rs->kalmanPhaseSize*3,step,&(rs->kalmanHinfT[centindx*3*rs->kalmanPhaseSize]),&(centroids[centindx]),&rs->XpredArr[threadno*rs->XpredSize]);
#else
  cblas_sgemv(order,trans,rs->kalmanPhaseSize*3,step,alpha,&(rs->kalmanHinfT[centindx*3*rs->kalmanPhaseSize]),rs->kalmanPhaseSize*3,&(centroids[centindx]),inc,beta,&rs->XpredArr[threadno*rs->XpredSize],inc);
#endif
  return 0;
}

/**
   Called once for each thread and the end of a frame
   Here we sum the individual dmCommands together to get the final one.
*/
int reconEndFrame(void *reconHandle,int threadno,float *centroids,float *dmCommand){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->buf];
  if(pthread_mutex_lock(&reconStruct->dmMutex))
    printf("pthread_mutex_lock error in copyThreadPhase: %s\n",strerror(errno));
  if(reconStruct->dmReady==0)//wait for the precompute thread to finish (it will call setDMArraysReady when done)...
    if(pthread_cond_wait(&reconStruct->dmCond,&reconStruct->dmMutex))
      printf("pthread_cond_wait error in copyThreadPhase: %s\n",strerror(errno));


  //now add threadInfo->dmCommand to threadInfo->info->dmCommand.
#ifdef USEAGBBLAS
  agb_cblas_saxpy111(rs->XpredSize,&rs->XpredArr[threadno*rs->XpredSize],reconStruct->Xpred);
#else
  cblas_saxpy(rs->XpredSize,1.,&rs->XpredArr[threadno*rs->XpredSize],1,reconStruct->Xpred,1);
#endif
  //cblas_saxpy(rs->nacts,1.,&rs->dmCommandArr[rs->nacts*threadInfo->threadno],1,glob->arrays->dmCommand,1);
  pthread_mutex_unlock(&reconStruct->dmMutex);
  return 0;
}

/**
   Called by single thread per frame at the end of frame.
   This is called by a subaperture processing thread.
   The bare minimum should be placed here, as most processing should be done in the reconFrameFinished function instead, which doesn't hold up processing.
*/
int reconFrameFinishedSync(void *reconHandle,float *centroids,float *dmCommand){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  //xxx do we really need to get the lock here?
  if(pthread_mutex_lock(&reconStruct->dmMutex))
    printf("pthread_mutex_lock error in copyThreadPhase: %s\n",strerror(errno));
  reconStruct->dmReady=0;
  pthread_mutex_unlock(&reconStruct->dmMutex);
  reconStruct->postbuf=reconStruct->buf;
  return 0;
}
/**
   Called by single thread per frame - end of frame
   Do any post processing here.  This is called after reconFrameFinishedSync.
   This it typically called by a non-subaperture processing thread.
   At the end of this method, dmCommand must be ready...
   Note, while this is running, subaperture processing of the next frame may start.
*/
int reconFrameFinished(void *reconHandle,float *dmCommand){//globalStruct *glob){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->postbuf];
  float bleedVal=0.;
  int i;
#ifndef USEAGBBLAS
  CBLAS_ORDER order=CblasRowMajor;
  CBLAS_TRANSPOSE trans=CblasNoTrans;
  float alpha=1.,beta=1.;
  int inc=1;
#endif
  printf("reconFrameFinished\n");
  if(rs->kalmanInvN!=NULL){
    //carry out dot(invN , Xpred) to get dmCommand...
    //If nacts!=kalmanPhaseSize, this allows us to convert to the correct size.
    //Otherwise, it allows us to perform an additional operation...
#ifdef USEAGBBLAS
    agb_cblas_sgemvRowMN1N101(rs->nacts,rs->kalmanPhaseSize,rs->kalmanInvN,reconStruct->Xpred,dmCommand);
#else
    beta=0;//compute the dmCommand from the kalman phase.
    cblas_sgemv(order,trans,rs->nacts,rs->kalmanPhaseSize,alpha,rs->kalmanInvN,rs->kalmanPhaseSize,reconStruct->Xpred,inc,beta,dmCommand,inc);
#endif
  }else{
    if(rs->nacts!=rs->kalmanPhaseSize){
      printf("Error - nacts!=kalmanPhaseSize...\n");
    }
    memcpy(dmCommand,reconStruct->Xpred,sizeof(float)*(rs->nacts<rs->kalmanPhaseSize?rs->nacts:rs->kalmanPhaseSize));
  }
  //Maybe this isn't necessary for kalman?
  if(rs->bleedGainOverNact!=0.){//compute the bleed value
    for(i=0; i<rs->nacts; i++){
      //bleedVal+=glob->arrays->dmCommand[i];
      bleedVal+=dmCommand[i]-rs->v0[i];
    }
    bleedVal*=rs->bleedGainOverNact;
    //bleedVal-=rs->midRangeTimesBleed;//Note - really midrange times bleed over nact... maybe this should be replaced by v0 - to allow a midrange value per actuator?
    for(i=0; i<rs->nacts; i++)
      dmCommand[i]-=bleedVal;
  }
  //The kalman equivalent of the following not required because we store it in Xpred (without the bleeding) anyway.
  //memcpy(reconStruct->latestDmCommand,glob->arrays->dmCommand,sizeof(float)*rs->nacts);

  //Final post processing - the predict step - is now done in reconNewFrame()
  return 0;
}
/**
   Called by the single thread per frame, when the actuator values aren't being sent to the dm - so we need to reset ourselves.
   May not be called at all, if the loop is closed.
   Not called by a subap processing thread - subap processing for next frame may have started before this is called...
   Shouldn't write to dmCommand, but reset library internals only.
*/
int reconOpenLoop(void *reconHandle){//globalStruct *glob){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  //ReconStructEntry *rs=&reconStruct->rs[reconStruct->postbuf];
  //memcpy(reconStruct->latestDmCommand,rs->v0,sizeof(float)*rs->nacts);
  memset(reconStruct->Xpred,0,sizeof(float)*reconStruct->XpredSize);
  return 0;

}




