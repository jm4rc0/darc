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
#include "rtcrecon.h"
#include "buffer.h"
//#define ASYNCKALMANINIT
typedef enum{
  BLEEDGAIN,
  KALMANATUR,
  KALMANHINFDM,
  KALMANHINFT,
  KALMANINVN,
  KALMANPHASESIZE,
  NACTS,
  V0,
  //Add more before this line.
  RECONNBUFFERVARIABLES//equal to number of entries in the enum
}RECONBUFFERVARIABLEINDX;

#define reconMakeNames() bufferMakeNames(RECONNBUFFERVARIABLES,"bleedGain","kalmanAtur","kalmanHinfDM","kalmanHinfT","kalmanInvN","kalmanPhaseSize","nacts","v0")
//char *RECONPARAM[]={"kalmanPhaseSize","kalmanHinfT","kalmanHinfDM","kalmanAtur","kalmanInvN","v0","bleedGain","nacts"};
#ifndef ASYNCKALMANINIT
typedef struct{
  int Atur;
  int AturStart;
  int AturSize;
  int HinfDM;
  int HinfDMStart;
  int HinfDMSize;
}DoStruct;
#endif

typedef struct{
  int totCents;
  int kalmanPhaseSize;
  float *kalmanHinfT;
  float *kalmanHinfDM;
  float *kalmanAtur;
  float *kalmanInvN;
  float **XpredArr;
  int XpredArrSize;
  int XpredSize;
  float *v0;
  float bleedGainOverNact;
  //float midRangeTimesBleed;
  int nacts;
#ifndef ASYNCKALMANINIT
  DoStruct *doS;
#endif
}ReconStructEntry;

typedef struct{
  ReconStructEntry rs[2];
  int buf;//current buffer being used
  int postbuf;//current buffer for post processing threads.
  //int swap;//set if need to change to the other buffer.
  int dmReady;
  float *Xpred;
  int XpredSize;
#ifndef ASYNCKALMANINIT
  float *Xpred1tmp;
#endif
  float *Xpred2tmp;
  pthread_mutex_t dmMutex;
  pthread_cond_t dmCond;
  //int bufindx[RECONNBUFFERVARIABLES];
  circBuf *rtcErrorBuf;
  int nthreads;
  arrayStruct *arr;
  int err;
  char *paramNames;
  int index[RECONNBUFFERVARIABLES];
  void *values[RECONNBUFFERVARIABLES];
  char dtype[RECONNBUFFERVARIABLES];
  int nbytes[RECONNBUFFERVARIABLES];
}ReconStruct;

/**
   Called to free the reconstructor module when it is being closed.
*/
int reconClose(void **reconHandle){
  ReconStruct *reconStruct=(ReconStruct*)*reconHandle;
  ReconStructEntry *rs;
  int i;
  printf("Closing kalman reconstruction library\n");
  if(reconStruct!=NULL){
    if(reconStruct->paramNames!=NULL)
      free(reconStruct->paramNames);
    pthread_mutex_destroy(&reconStruct->dmMutex);
    pthread_cond_destroy(&reconStruct->dmCond);
    if(reconStruct->Xpred!=NULL)
      free(reconStruct->Xpred);
    if(reconStruct->Xpred2tmp!=NULL)
      free(reconStruct->Xpred2tmp);
#ifndef ASYNCKALMANINIT
    if(reconStruct->Xpred1tmp!=NULL)
      free(reconStruct->Xpred1tmp);
#endif
    rs=&reconStruct->rs[0];
    if(rs->XpredArr!=NULL){
      for(i=0; i<reconStruct->nthreads;i++){
	if(rs->XpredArr[i]!=NULL)
	  free(rs->XpredArr[i]);
      }
      free(rs->XpredArr);
    }
#ifndef ASYNCKALMANINIT
    if(rs->doS!=NULL)
      free(rs->doS);
#endif
    rs=&reconStruct->rs[1];
    if(rs->XpredArr!=NULL){
      for(i=0; i<reconStruct->nthreads;i++){
	if(rs->XpredArr[i]!=NULL)
	  free(rs->XpredArr[i]);
      }
      free(rs->XpredArr);
    }
#ifndef ASYNCKALMANINIT
    if(rs->doS!=NULL)
      free(rs->doS);
#endif
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
  //infoStruct *info=threadInfo->info;
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;//threadInfo->globals->reconStruct;
  //int *indx=reconStruct->bufindx;
  //Use the buffer not currently in use.
  ReconStructEntry *rs;
  RECONBUFFERVARIABLEINDX i;
  int nfound;
#ifndef ASYNCKALMANINIT
  int halfthr;
#endif
  int *nbytes=reconStruct->nbytes;
  void **values=reconStruct->values;
  char *dtype=reconStruct->dtype;
  //swap the buffers...
  reconStruct->buf=1-reconStruct->buf;
  rs=&reconStruct->rs[reconStruct->buf];
  rs->totCents=totCents;
  reconStruct->arr=arr;
  nfound=bufferGetIndex(pbuf,RECONNBUFFERVARIABLES,reconStruct->paramNames,reconStruct->index,reconStruct->values,reconStruct->dtype,reconStruct->nbytes);
  if(nfound!=RECONNBUFFERVARIABLES){
    err=-1;
    printf("Didn't get all buffer entries for recon module:\n");
    for(j=0; j<RECONNBUFFERVARIABLES; j++){
      if(reconStruct->index[j]<0)
	printf("Missing %16s\n",&reconStruct->paramNames[j*BUFNAMESIZE]);
    }
  }

  /*
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
    }*/
  if(err==0){
    i=NACTS;
    if(dtype[i]=='i' && nbytes[i]==4){
      rs->nacts=*((int*)(values[i]));
    }else{
      printf("nacts error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"nacts error");
      err=1;
    }
    i=KALMANPHASESIZE;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      rs->kalmanPhaseSize=*((int*)(values[i]));
    }else{
      printf("kalmanPhaseSize error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"kalmanPhaseSize error");
      err=1;
    }
    i=KALMANHINFT;
    if(dtype[i]=='f' && nbytes[i]==rs->kalmanPhaseSize*3*rs->totCents*sizeof(float)){
      rs->kalmanHinfT=((float*)(values[i]));
    }else{
      err=1;
      printf("kalmanHinfT error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"kalmanHinfT error");
    }
    i=KALMANHINFDM;
    if(dtype[i]=='f' && nbytes[i]==rs->kalmanPhaseSize*rs->kalmanPhaseSize*3*sizeof(float)){
      rs->kalmanHinfDM=((float*)(values[i]));
    }else{
      err=1;
      printf("kalmanHinfDM error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"kalmanHinfDM error");
    }
    i=KALMANATUR;
    if(dtype[i]=='f' && nbytes[i]==rs->kalmanPhaseSize*rs->kalmanPhaseSize*sizeof(float)){
      rs->kalmanAtur=((float*)(values[i]));
    }else{
      err=1;
      printf("kalmanAtur error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"kalmanAtur error");
    }
    i=KALMANINVN;
    nb=nbytes[i];
    if(nb==0){
      rs->kalmanInvN=NULL;
    }else if(dtype[i]=='f' && nbytes[i]==rs->nacts*rs->kalmanPhaseSize*sizeof(float)){
      rs->kalmanInvN=((float*)(values[i]));
    }else{
      err=1;
      printf("kalmanInvN error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"kalmanInvN error");
    }
    i=V0;
    if(dtype[i]=='f' && nbytes[i]==sizeof(float)*rs->nacts){
      rs->v0=(float*)(values[i]);
    }else{
      printf("v0 error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"v0 error");
      err=1;
    }
    i=BLEEDGAIN;
    if(dtype[i]=='f' && nbytes[i]==sizeof(float)){
      rs->bleedGainOverNact=(*((float*)(values[i])))/rs->nacts;
    }else{
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"bleedGain error");
      printf("bleedGain error\n");
      err=1;
    }
    /*
    i=MIDRANGE;
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      rs->midRangeTimesBleed=(*((int*)(values[i])))*rs->nacts*rs->bleedGainOverNact;
    }else{
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"midRangeValue error");
      printf("midrange error\n");
      err=1;
      }*/
  }
  if(err==0){
    if(reconStruct->XpredSize<rs->kalmanPhaseSize*3){
      if(reconStruct->Xpred!=NULL)
	free(reconStruct->Xpred);
      if(reconStruct->Xpred2tmp!=NULL)
	free(reconStruct->Xpred2tmp);
#ifndef ASYNCKALMANINIT
      if(reconStruct->Xpred2tmp!=NULL)
	free(reconStruct->Xpred2tmp);
#endif
      reconStruct->XpredSize=rs->kalmanPhaseSize*3;
      if((reconStruct->Xpred=calloc(reconStruct->XpredSize,sizeof(float)))==NULL){
	printf("malloc of Xpred failed in reconKalman\n");
	err=-3;
	reconStruct->XpredSize=0;
      }else if((reconStruct->Xpred2tmp=calloc(rs->kalmanPhaseSize,sizeof(float)))==NULL){
	printf("malloc of Xpred2tmp failed in reconKalman\n");
	err=-3;
	reconStruct->XpredSize=0;
	free(reconStruct->Xpred);
	reconStruct->Xpred=NULL;
      }
#ifndef ASYNCKALMANINIT
      else if((reconStruct->Xpred1tmp=calloc(rs->kalmanPhaseSize,sizeof(float)))==NULL){
	printf("malloc of Xpred1tmp failed in reconKalman\n");
	err=-3;
	reconStruct->XpredSize=0;
	free(reconStruct->Xpred);
	free(reconStruct->Xpred2tmp);
	reconStruct->Xpred=NULL;
	reconStruct->Xpred2tmp=NULL;
      }
#endif
    }
    if(rs->XpredArrSize<rs->kalmanPhaseSize*3){
      int i;
      rs->XpredArrSize=rs->kalmanPhaseSize*3;
      for(i=0; i<reconStruct->nthreads; i++){
	if(rs->XpredArr[i]!=NULL)
	  free(rs->XpredArr[i]);
	if((rs->XpredArr[i]=calloc(rs->XpredArrSize,sizeof(float)))==NULL){
	  i--;
	  rs->XpredArrSize=0;
	  while(i>0){//free existing
	    if(rs->XpredArr[i]!=NULL)
	      free(rs->XpredArr[i]);
	    rs->XpredArr[i]=NULL;
	    i--;
	  }
	  printf("malloc of XpredArr failed in reconKalman\n");
	  err=-3;
	  break;
	}
      }
    }
#ifndef ASYNCKALMANINIT
    //work out which initialisation work the threads should do.
    //First half of threads to the Atur
    //Second half do the HinfDM.
    halfthr=(reconStruct->nthreads+1)/2;//compute half the threads, rounded up.
    rs->doS[0].AturStart=0;
    for(j=0;j<halfthr;j++){
      rs->doS[j].Atur=1;
      rs->doS[j].HinfDM=0;
      if(j>0)
	rs->doS[j].AturStart=rs->doS[j-1].AturStart+rs->doS[j-1].AturSize;
      rs->doS[j].AturSize=(rs->kalmanPhaseSize-rs->doS[j].AturStart)/(halfthr-j);
    }
    for(j=halfthr;j<reconStruct->nthreads;j++){
      rs->doS[j].Atur=0;
      rs->doS[j].HinfDM=1;
      if(j==halfthr)
	rs->doS[j].HinfDMStart=rs->kalmanPhaseSize;
      else
	rs->doS[j].HinfDMStart=rs->doS[j-1].HinfDMStart+rs->doS[j-1].HinfDMSize;
      rs->doS[j].HinfDMSize=(rs->kalmanPhaseSize*3-rs->doS[j].HinfDMStart)/(reconStruct->nthreads-j);
    }
    if(reconStruct->nthreads==1){//only 1 thread - has to do everything...
      rs->doS[0].HinfDM=1;
      rs->doS[0].HinfDMStart=rs->kalmanPhaseSize;
      rs->doS[0].HinfDMSize=rs->kalmanPhaseSize*2;
    }

#endif

  }

  /*
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
  */


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
int reconOpen(char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **reconHandle,int nthreads,unsigned int frameno,unsigned int **reconframeno,int *reconframenoSize,int totCents){
  //Sort through the parameter buffer, and get the things we need, and do 
  //the allocations we need.
  ReconStruct *reconStruct;
  //ReconStructEntry *rs;
  int err=0;
  int i;
  if((reconStruct=calloc(sizeof(ReconStruct),1))==NULL){
    printf("Error allocating recon memory\n");
    //threadInfo->globals->reconStruct=NULL;
    *reconHandle=NULL;
    return 1;
  }
  *reconHandle=(void*)reconStruct;
  reconStruct->buf=1;
  reconStruct->nthreads=nthreads;//this doesn't change.
  for(i=0; i<2; i++){
    if((reconStruct->rs[i].XpredArr=(float**)calloc(sizeof(float*),nthreads))==NULL){
      printf("Error in reconKalman allocating xpredArr\n");
      reconClose(reconHandle);
      *reconHandle=NULL;
      return 1;
    }
#ifndef ASYNCKALMANINIT
    if((reconStruct->rs[i].doS=(DoStruct*)calloc(sizeof(DoStruct),nthreads))==NULL){
      printf("Error in reconKalman alloocing doS\n");
      reconClose(reconHandle);
      *reconHandle=NULL;
      return 1;
    }
#endif
  }
  reconStruct->rtcErrorBuf=rtcErrorBuf;
  reconStruct->paramNames=reconMakeNames();
  reconStruct->arr=arr;
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
  //float dmCommand=reconStruct->arr->dmCommand;
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
  //Now wake up the thread that does the initial processing...
  //No need to get a mutex here, because the preprocessing thread must be sleeping.
  //We do this processing in a separate thread because it has a while to complete, so we may as well let subap processing threads get on with their tasks...
  //pthread_cond_signal(&reconStruct->dmCond);
  //performn precomp=dot(DM,Xpred[phaseSize*2:])


  //first do some final post processing stuff...
#ifdef ASYNCKALMANINIT
  ReconStructEntry *rs;
  rs=&reconStruct->rs[reconStruct->buf];
  if(reconStruct->err==0){//no error from previous frames...
    memcpy(reconStruct->Xpred2tmp,&reconStruct->Xpred[rs->kalmanPhaseSize*1],sizeof(float)*rs->kalmanPhaseSize);//copy what will be Xpred[2] to scratch
    memcpy(&(reconStruct->Xpred[rs->kalmanPhaseSize*2]),&(reconStruct->Xpred[rs->kalmanPhaseSize*1]),sizeof(float)*rs->kalmanPhaseSize);
    memcpy(&(reconStruct->Xpred[rs->kalmanPhaseSize]),reconStruct->Xpred,sizeof(float)*rs->kalmanPhaseSize);
    //Perform Xpred[:phaseSize]=dot(Atur,Xpred[:phaseSize])
#ifdef USEAGBBLAS
    //alpha=1, beta=0
    //Maybe some of this can be moved into reconStartFrame so that the work is split between threads?  But then would have to be very careful about synchronisation...
    agb_cblas_sgemvRowNN1N101(rs->kalmanPhaseSize,rs->kalmanAtur,&(reconStruct->Xpred[rs->kalmanPhaseSize]),reconStruct->Xpred);
    //alpha=1, beta=-1.
    agb_cblas_sgemvRowMN1N1m11(rs->kalmanPhaseSize*3,rs->kalmanPhaseSize,rs->kalmanHinfDM,reconStruct->Xpred2tmp,reconStruct->Xpred);
    
#else
    beta=0.;
    cblas_sgemv(order,trans,rs->kalmanPhaseSize,rs->kalmanPhaseSize,alpha,rs->kalmanAtur,rs->kalmanPhaseSize,&(reconStruct->Xpred[rs->kalmanPhaseSize]),inc,beta,reconStruct->Xpred,inc);
    
    beta=-1.;
    cblas_sgemv(order,trans,rs->kalmanPhaseSize*3,rs->kalmanPhaseSize,alpha,rs->kalmanHinfDM,rs->kalmanPhaseSize,reconStruct->Xpred2tmp,inc,beta,reconStruct->Xpred,inc);
#endif
  }
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


#ifndef ASYNCKALMANINIT
//called once each frame by a single subap processing thread.
//Copys the Xpred arrays about synchronously.
int reconNewFrameSync(void *reconHandle,unsigned int frameno,double timestamp){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  ReconStructEntry *rs;
  rs=&reconStruct->rs[reconStruct->buf];  
  if(reconStruct->err==0){//no error from previous frames...
    memcpy(reconStruct->Xpred2tmp,&reconStruct->Xpred[rs->kalmanPhaseSize*1],sizeof(float)*rs->kalmanPhaseSize);//copy what will be Xpred[2] to scratch
    memcpy(&(reconStruct->Xpred[rs->kalmanPhaseSize*2]),&(reconStruct->Xpred[rs->kalmanPhaseSize*1]),sizeof(float)*rs->kalmanPhaseSize);
    memcpy(&(reconStruct->Xpred[rs->kalmanPhaseSize]),reconStruct->Xpred,sizeof(float)*rs->kalmanPhaseSize);
    memcpy(reconStruct->Xpred1tmp,reconStruct->Xpred,sizeof(float)*rs->kalmanPhaseSize);//this is also scratch
  }
  return 0;
}
#endif

/**
   Called once per thread at the start of each frame, possibly simultaneously.
*/
int reconStartFrame(void *reconHandle,int cam,int threadno){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->buf];
  memset(rs->XpredArr[threadno],0,sizeof(float)*rs->XpredSize);
#ifndef ASYNCKALMANINIT
  if(reconStruct->err==0){//no error from previous frames...
    //memcpy(&(reconStruct->Xpred[rs->kalmanPhaseSize*2]),&(reconStruct->Xpred[rs->kalmanPhaseSize*1]),sizeof(float)*rs->kalmanPhaseSize);
    //memcpy(&(reconStruct->Xpred[rs->kalmanPhaseSize]),reconStruct->Xpred,sizeof(float)*rs->kalmanPhaseSize);
    //Perform Xpred[:phaseSize]=dot(Atur,Xpred[:phaseSize])
#ifdef USEAGBBLAS
    if(rs->doS[threadno].Atur){
      //computes Xpred[0]=Atur.Xpred[1]-HinfDM[0].Xpred[2]
      
      //alpha=1, beta=0
      agb_cblas_sgemvRowMN1N101(rs->doS[threadno].AturSize,rs->kalmanPhaseSize,&rs->kalmanAtur[rs->doS[threadno].AturStart*rs->kalmanPhaseSize],reconStruct->Xpred1tmp,&reconStruct->Xpred[rs->doS[threadno].AturStart]);
      //alpha=1, beta=-1.
      agb_cblas_sgemvRowMN1N1m11(rs->doS[threadno].AturSize,rs->kalmanPhaseSize,&rs->kalmanHinfDM[rs->doS[threadno].AturStart*rs->kalmanPhaseSize],reconStruct->Xpred2tmp,&reconStruct->Xpred[rs->doS[threadno].AturStart]);
    }
    if(rs->doS[threadno].HinfDM){
      //computes Xpred[1,2]-=HinfDM[1,2].Xpred[2]
      //alpha=1, beta=-1.
      agb_cblas_sgemvRowMN1N1m11(rs->doS[threadno].HinfDMSize,rs->kalmanPhaseSize,&rs->kalmanHinfDM[rs->doS[threadno].HinfDMStart*rs->kalmanPhaseSize],reconStruct->Xpred2tmp,&reconStruct->Xpred[rs->doS[threadno].HinfDMStart]);
    }
    
#else
    CBLAS_ORDER order=CblasRowMajor;
    CBLAS_TRANSPOSE trans=CblasNoTrans;
    float alpha=1.,beta=1.;
    int inc=1;
    beta=0.;
    printf("This won't work - needs updating (reconKalman.c)\n");
    cblas_sgemv(order,trans,rs->kalmanPhaseSize,rs->kalmanPhaseSize,alpha,rs->kalmanAtur,rs->kalmanPhaseSize,&(reconStruct->Xpred[rs->kalmanPhaseSize]),inc,beta,reconStruct->Xpred,inc);
    
    beta=-1.;
    cblas_sgemv(order,trans,rs->kalmanPhaseSize*3,rs->kalmanPhaseSize,alpha,rs->kalmanHinfDM,rs->kalmanPhaseSize,reconStruct->Xpred2tmp,inc,beta,reconStruct->Xpred,inc);
#endif
  }
#endif
  return 0;
}

/**
   Called multiple times by multiple threads, whenever new slope data is ready
*/
int reconNewSlopes(void *reconHandle,int cam,int centindx,int threadno,int nsubapsDoing){
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
  float *centroids=reconStruct->arr->centroids;
  //infoStruct *info=threadInfo->info;
  step=2*nsubapsDoing;
#ifdef USEAGBBLAS
  agb_cblas_sgemvColMN1M111(rs->kalmanPhaseSize*3,step,&(rs->kalmanHinfT[centindx*3*rs->kalmanPhaseSize]),&(centroids[centindx]),rs->XpredArr[threadno]);
#else
  cblas_sgemv(order,trans,rs->kalmanPhaseSize*3,step,alpha,&(rs->kalmanHinfT[centindx*3*rs->kalmanPhaseSize]),rs->kalmanPhaseSize*3,&(centroids[centindx]),inc,beta,rs->XpredArr[threadno],inc);
#endif
  return 0;
}

/**
   Called once for each thread and the end of a frame
   Here we sum the individual dmCommands together to get the final one.
*/
int reconEndFrame(void *reconHandle,int cam,int threadno,int err){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->buf];
  if(pthread_mutex_lock(&reconStruct->dmMutex))
    printf("pthread_mutex_lock error in copyThreadPhase: %s\n",strerror(errno));
  if(reconStruct->dmReady==0)//wait for the precompute thread to finish (it will call setDMArraysReady when done)...
    if(pthread_cond_wait(&reconStruct->dmCond,&reconStruct->dmMutex))
      printf("pthread_cond_wait error in copyThreadPhase: %s\n",strerror(errno));


  //now add threadInfo->dmCommand to threadInfo->info->dmCommand.
#ifdef USEAGBBLAS
  agb_cblas_saxpy111(rs->XpredSize,rs->XpredArr[threadno],reconStruct->Xpred);
#else
  cblas_saxpy(rs->XpredSize,1.,rs->XpredArr[threadno],1,reconStruct->Xpred,1);
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
int reconFrameFinishedSync(void *reconHandle,int err,int forcewrite){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
#ifndef ASYNCKALMANINIT
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->buf];
#endif
  //xxx do we really need to get the lock here?
  if(pthread_mutex_lock(&reconStruct->dmMutex))
    printf("pthread_mutex_lock error in copyThreadPhase: %s\n",strerror(errno));
  reconStruct->dmReady=0;
  pthread_mutex_unlock(&reconStruct->dmMutex);
  reconStruct->postbuf=reconStruct->buf;



#ifndef ASYNCKALMANINIT
  float *dmCommand=reconStruct->arr->dmCommand;
#ifndef USEAGBBLAS
  CBLAS_ORDER order=CblasRowMajor;
  CBLAS_TRANSPOSE trans=CblasNoTrans;
  float alpha=1.,beta=1.;
  int inc=1;
#endif
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


#endif//not ASYNCKALMANINIT
  return 0;
}
/**
   Called by single thread per frame - end of frame
   Do any post processing here.  This is called after reconFrameFinishedSync.
   This it typically called by a non-subaperture processing thread.
   At the end of this method, dmCommand must be ready...
   Note, while this is running, subaperture processing of the next frame may start.
*/
int reconFrameFinished(void *reconHandle,int err){//globalStruct *glob){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->postbuf];
  float bleedVal=0.;
  int i;
  float *dmCommand=reconStruct->arr->dmCommand;
#ifdef ASYNCKALMANINIT
#ifndef USEAGBBLAS
  CBLAS_ORDER order=CblasRowMajor;
  CBLAS_TRANSPOSE trans=CblasNoTrans;
  float alpha=1.,beta=1.;
  int inc=1;
#endif
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
#endif//ASYNCKALMANINIT
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
  reconStruct->err=err;
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




