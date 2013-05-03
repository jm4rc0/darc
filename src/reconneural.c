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
   This is a library that can be used for a neural network implementation.
*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
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
#include "rtcrecon.h"
#include "buffer.h"
typedef enum{RECONMODE_SIMPLE,RECONMODE_TRUTH,RECONMODE_OPEN,RECONMODE_OFFSET}ReconModeType;

typedef enum{
  ANNBIAS,
  ANNLAYERSIZE,
  ANNLAYERTYPE,
  //ANNNLAYERS,
  ANNOFFSET,
  ANNSCALE,
  ANNTYPEARRAY,
  ANNWEIGHTS,
  BLEEDGAIN,
  BLEEDGROUPS,
  DECAYFACTOR,
  GAINE,
  //GAINRECONMXT,
  NACTS,
  RECONSTRUCTMODE,
  RECORDLINEAR,
  V0,
  //Add more before this line.
  RECONNBUFFERVARIABLES//equal to number of entries in the enum
}RECONBUFFERVARIABLEINDX;

#define reconMakeNames() bufferMakeNames(RECONNBUFFERVARIABLES,"annBias","annLayerSize","annLayerType","annOffset","annScale","annTypeArray","annWeights","bleedGain","bleedGroups","decayFactor","gainE",/*"gainReconmxT",*/"nacts","reconstructMode","recordLinear","v0")


typedef struct{
  ReconModeType reconMode;
  float *gainE;
  //int dmCommandArrSize;
  //float **dmCommandArr;
  float *annTmp;
  int annTmpSize;
  int annNLayers;
  float *annBias;
  float *annTmpOut;
  int *annCum;
  int *annCum2;
  int annCumSize;
  int *annLayerSize;
  float *annWeights;
  float *annScale;
  float *annOffset;
  int *annLayerType;
  int *annTypeArray;
  //float *rmxT;
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
  float recordLinear;
  int isLinear;
  int isNotLinear;
}ReconStructEntry;

typedef struct{
  ReconStructEntry rs[2];
  int buf;//current buffer being used
  int postbuf;//current buffer for post processing threads.
  //int swap;//set if need to change to the other buffer.
  int dmReady;
  float *latestDmCommand;
  int latestDmCommandSize;
  float **annTmpArr;
  int annTmpArrSize;
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
  unsigned int *reconFrameno;
}ReconStruct;

/**
   Called to free the reconstructor module when it is being closed.
*/
int reconClose(void **reconHandle){//reconHandle is &globals->reconStruct.
  ReconStruct *reconStruct=(ReconStruct*)*reconHandle;
  ReconStructEntry *rs;
  int j;
  printf("Closing reconlibrary\n");
  if(reconStruct!=NULL){
    if(reconStruct->paramNames!=NULL)
      free(reconStruct->paramNames);
    pthread_mutex_destroy(&reconStruct->dmMutex);
    pthread_cond_destroy(&reconStruct->dmCond);
    if(reconStruct->latestDmCommand!=NULL)
      free(reconStruct->latestDmCommand);
    if(reconStruct->annTmpArr!=NULL){
      if(reconStruct->annTmpArr[0]!=NULL)
	free(reconStruct->annTmpArr[0]);
      free(reconStruct->annTmpArr);
    }
    for(j=0;j<2;j++){
      rs=&reconStruct->rs[j];
      if(rs->bleedVal!=NULL)free(rs->bleedVal);
      if(rs->annCum!=NULL)free(rs->annCum);
      if(rs->annTmp!=NULL)free(rs->annTmp);
      /*if(rs->dmCommandArr!=NULL){
	for(i=0; i<reconStruct->nthreads; i++){
	  if(rs->dmCommandArr[i]!=NULL)
	    free(rs->dmCommandArr[i]);
	}
	free(rs->dmCommandArr);
	}*/
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
  int nb,mx,totsize,mxsize;
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
    err=0;
    for(i=0;i<RECONNBUFFERVARIABLES;i++){
      if(reconStruct->index[i]<0 && (i!=RECORDLINEAR)){
	printf("Missing %16s\n",&reconStruct->paramNames[i*BUFNAMESIZE]);
	err=1;
      }
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
    /*i=GAINRECONMXT;
    if(dtype[i]=='f' && nbytes[i]/sizeof(float)==rs->totCents*rs->nacts){
      rs->rmxT=(float*)values[i];
    }else{
      printf("gainReconmxT error %c %d %d %d\n",dtype[i],nbytes[i],rs->totCents,rs->nacts);
      err=GAINRECONMXT;
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"gainReconmxT error");
      }*/
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
    /*i=ANNNLAYERS;
    if(nbytes[i]==sizeof(int) && dtype[i]=='i'){
      rs->annNLayers=*((int*)values[i]);
      if(rs->annNLayers<1){
	printf("Error - at least 1 layer (the output) must be defined\n");
	writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"Error - at least 1 layer (the output) must be defined");
	err=1;
	rs->annNLayers=0;
      }
    }else{
      printf("annNLayers error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"annNLayers error");
      err=1;
      rs->annNLayers=0;
      }*/

    i=ANNLAYERSIZE;
    rs->annNLayers=nbytes[i]/sizeof(int);
    if(rs->annNLayers<1){
      printf("Error - at least 1 layer (the output) must be defined\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"Error - at least 1 layer (the output) must be defined");
      err=1;
      rs->annNLayers=0;
    }
    if(nbytes[i]==rs->annNLayers*sizeof(int) && dtype[i]=='i'){
      rs->annLayerSize=((int*)values[i]);
      if(rs->annLayerSize[rs->annNLayers-1]!=rs->nacts){
	printf("Error - final layer must have size equal to nacts\n");
	err=1;
	writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"Final layer must have size equal to nacts");
	rs->annLayerSize=NULL;
      }
    }else{
      printf("annLayerSize error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"annLayerSize error");
      err=1;
      rs->annLayerSize=NULL;
    }
    if(err==0 && rs->annCumSize<rs->annNLayers+1){
      if(rs->annCum)free(rs->annCum);
      if((rs->annCum=malloc(sizeof(int)*2*(rs->annNLayers+1)))==NULL){
	printf("Error allocing annCum\n");
	err=1;
	rs->annCumSize=0;
	rs->annCum2=NULL;
      }else{
	rs->annCumSize=rs->annNLayers+1;
	rs->annCum2=&(rs->annCum[rs->annNLayers+1]);
      }
    }

    i=ANNLAYERTYPE;
    if(nbytes[i]==rs->annNLayers*sizeof(int) && dtype[i]=='i'){
      rs->annLayerType=((int*)values[i]);
    }else{
      printf("annLayerType error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"annLayerType error");
      err=1;
      rs->annLayerType=NULL;
    }
    rs->annBias=NULL;
    rs->annOffset=NULL;
    rs->annScale=NULL;
    rs->annTypeArray=NULL;
    rs->annWeights=NULL;
    if(err==0){
      totsize=0;
      mxsize=totCents*rs->annLayerSize[0];
      rs->annCum[0]=0;
      rs->annCum2[0]=0;
      rs->annCum2[1]=mxsize;
      for(j=0;j<rs->annNLayers;j++){
	totsize+=rs->annLayerSize[j];
	rs->annCum[j+1]=totsize;
	if(j<rs->annNLayers-1){
	  mxsize+=rs->annLayerSize[j]*rs->annLayerSize[j+1];
	  rs->annCum2[j+2]=mxsize;
	}
      }
      i=ANNBIAS;
      if(nbytes[i]==sizeof(float)*totsize && dtype[i]=='f'){
	rs->annBias=(float*)values[i];
      }else{
	printf("Error annBias\n");
	writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"annBias error");
	err=1;
      }
      i=ANNOFFSET;
      if(nbytes[i]==0){
	rs->annOffset=NULL;
      }else if(nbytes[i]==sizeof(float)*rs->nacts && dtype[i]=='f'){
	rs->annOffset=(float*)values[i];
      }else{
	printf("Error annOffset\n");
	writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"annOffset error");
	err=1;
      }
      i=ANNSCALE;
      if(nbytes[i]==0){
	rs->annScale=NULL;
      }else if(nbytes[i]==sizeof(float)*rs->nacts && dtype[i]=='f'){
	rs->annScale=(float*)values[i];
      }else{
	printf("Error annScale\n");
	writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"annScale error");
	err=1;
      }
      i=ANNWEIGHTS;
      if(nbytes[i]==sizeof(float)*mxsize && dtype[i]=='f'){
	rs->annWeights=(float*)values[i];
      }else{
	printf("Error annWeights\n");
	writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"annWeights error");
	err=1;
      }
      i=ANNTYPEARRAY;
      if(nbytes[i]==0){
	rs->annTypeArray=NULL;
      }else if((dtype[i]=='f') && (nbytes[i]==sizeof(int)*totsize)){
	rs->annTypeArray=(int*)values[i];
      }else{
	printf("Error annTypeArray\n");
	writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"annTypeArray error");
	err=1;
      }	
      i=RECORDLINEAR;
      rs->recordLinear=0.;
      if(reconStruct->index[i]>=0){
	if(dtype[i]=='f' && nbytes[i]==sizeof(float)){
	  rs->recordLinear=*(float*)values[i];
	}else{
	  printf("Error recordLinear\n");
	  writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"recordLinear error");
	  err=1;
	}
      }
    }   
    if(reconStruct->annTmpArrSize<sizeof(float)*rs->annLayerSize[0]){
      reconStruct->annTmpArrSize=sizeof(float)*rs->annLayerSize[0];
      if(reconStruct->annTmpArr[0]!=NULL)free(reconStruct->annTmpArr[0]);
      if((reconStruct->annTmpArr[0]=calloc(reconStruct->nthreads,reconStruct->annTmpArrSize))==NULL){
	printf("Error allocing annTmpArr\n");
	err=1;
	reconStruct->annTmpArrSize=0;
      }else{
	for(j=1;j<reconStruct->nthreads;j++){
	  reconStruct->annTmpArr[j]=&(reconStruct->annTmpArr[j-1][rs->annLayerSize[0]]);
	}
      }
    }
    mx=0;
    for(j=0;j<rs->annNLayers;j++){
      if(rs->annLayerSize[j]>mx)
	mx=rs->annLayerSize[j];
    }
    if(rs->annTmpSize<sizeof(float)*mx){//this should be sized to the maximum layer.
      rs->annTmpSize=sizeof(float)*mx;
      if(rs->annTmp!=NULL)free(rs->annTmp);
      if((rs->annTmp=malloc(rs->annTmpSize*2))==NULL){
	printf("Error allocing annTmp to size %d\n",rs->annTmpSize);
	err=1;
	rs->annTmpSize=0;
      }else
	rs->annTmpOut=&(rs->annTmp[mx]);
    }
  }
  //No need to get the lock here because this and newFrame() are called inside glob->libraryMutex.
  reconStruct->dmReady=0;
  /*if(rs->dmCommandArrSize<sizeof(float)*rs->nacts){
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
    }*/
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
  /*if((reconStruct->rs[0].dmCommandArr=calloc(sizeof(float*),nthreads))==NULL){
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
    }*/
  if((reconStruct->annTmpArr=calloc(sizeof(float*),nthreads))==NULL){
    printf("Error allocating reconann memory[0]\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  /*if((reconStruct->rs[1].annTmpArr=calloc(sizeof(float*),nthreads))==NULL){
    printf("Error allocating reconann memory[1]\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
    }*/
  if(*reconframenoSize<1){//Can use this to pass out the fraction of linear activated neurons.
    *reconframeno=malloc(sizeof(int));
    *reconframenoSize=(*reconframeno!=NULL);
    reconStruct->reconFrameno=*reconframeno;
    printf("reconFrameno %p\n",reconStruct->reconFrameno);
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
#if !defined(USEAGBBLAS)
  CBLAS_ORDER order=CblasRowMajor;
  CBLAS_TRANSPOSE trans=CblasNoTrans;
  float alpha=1.,beta=1.;
  int inc=1;
#endif
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
#ifdef USEAGBBLAS
      agb_cblas_sgemvRowNN1N101(rs->nacts,rs->gainE,reconStruct->latestDmCommand,dmCommand);
#else
    //beta=0.;
      cblas_sgemv(order,trans,rs->nacts,rs->nacts,alpha,rs->gainE,rs->nacts,reconStruct->latestDmCommand,inc,beta,dmCommand,inc);
#endif
    }else{//gainE==NULL...
      memcpy(dmCommand,reconStruct->latestDmCommand,sizeof(float)*rs->nacts);
      //and add v0
      agb_cblas_saxpy111(rs->nacts,rs->v0,dmCommand);
    }

  }else{//reconmode_offset
    memcpy(dmCommand,rs->v0,sizeof(float)*rs->nacts);
  }	
  memcpy(rs->annTmp,rs->annBias,sizeof(float)*rs->annLayerSize[0]);
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
  memset((void*)(reconStruct->annTmpArr[threadno]),0,rs->annLayerSize[0]*sizeof(float));
  return 0;
}


/**
   Called multiple times by multiple threads, whenever new slope data is ready
   centroids may not be complete, and writing to dmCommand is not thread-safe without locking.
*/
int reconNewSlopes(void *reconHandle,int cam,int centindx,int threadno,int nsubapsDoing){
#if !defined(USEAGBBLAS)
  CBLAS_ORDER order=CblasColMajor;
  CBLAS_TRANSPOSE trans=CblasNoTrans;
  float alpha=1.,beta=1.;
  int inc=1;
#endif
  int step;//number of rows to do in mmx...
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->buf];
  float *centroids=reconStruct->arr->centroids;
  //All we can do before having the first set of slopes, is multiply the first weighting matrix.

  //So, here we just do annTmpArr+=annWeights[0][:,n]*centx+annWeights[0][:,n+1]*centy.
  //Note, annTmpArr has previously been initialised with 0, and annTmp with the hidden layer bias.
  dprintf("in partialReconstruct %d %d %d %p %p %p\n",rs->annLayerSize[0],centindx,rs->totCents,centroids,rs->annWeights[0],reconStruct->annTmpArr[threadno]);
  step=2*nsubapsDoing;
#ifdef USEAGBBLAS
  agb_cblas_sgemvColMN1M111(rs->annLayerSize[0],step,&(rs->annWeights[centindx*rs->annLayerSize[0]]),&(centroids[centindx]),reconStruct->annTmpArr[threadno]);
#else
  cblas_sgemv(order,trans,rs->annLayerSize[0],step,alpha,&(rs->annWeights[centindx*rs->annLayerSize[0]]),rs->annLayerSize[0],&(centroids[centindx]),inc,beta,reconStruct->annTmpArr[threadno],inc);
#endif
  return 0;
}

/**
   Called once for each thread at the end of a frame
   Here we sum the individual dmCommands together to get the final one.
   centroids may not be complete, and writing to dmCommand is not thread-safe without locking.
*/
int reconEndFrame(void *reconHandle,int cam,int threadno,int err){
  //dmCommand=glob->arrays->dmCommand;
  //globalStruct *glob=threadInfo->globals;
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->buf];
  //float *dmCommand=reconStruct->arr->dmCommand;
  if(pthread_mutex_lock(&reconStruct->dmMutex))
    printf("pthread_mutex_lock error in copyThreadPhase: %s\n",strerror(errno));
  if(reconStruct->dmReady==0)//wait for the precompute thread to finish (it will call setDMArraysReady when done)...
    if(pthread_cond_wait(&reconStruct->dmCond,&reconStruct->dmMutex))
      printf("pthread_cond_wait error in copyThreadPhase: %s\n",strerror(errno));
  //now add threadInfo->dmCommand to threadInfo->info->dmCommand.
#ifdef USEAGBBLAS
  agb_cblas_saxpy111(rs->annLayerSize[0],reconStruct->annTmpArr[threadno],rs->annTmp);
#else
  cblas_saxpy(rs->annLayerSize[0],1.,reconStruct->annTmpArr[threadno],1,rs->annTmp,1);
#endif

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
  //No need to get the lock here.
  reconStruct->dmReady=0;
  //pthread_mutex_unlock(&reconStruct->dmMutex);
  reconStruct->postbuf=reconStruct->buf;
  return 0;
}

inline float tansig(float v){
  return (2/(1+expf(-v)))-1;
}

void activate(ReconStructEntry *rs,int layer,float *in,float *out){
  //in can point to out.
  int i;
  switch(rs->annLayerType[layer]){
  case 0://no activation
    if(out!=in)
      memcpy(out,in,sizeof(float)*rs->annLayerSize[layer]);
    break;
  case 1://tansig activation.
    if(rs->recordLinear!=0.){
      for(i=0;i<rs->annLayerSize[layer];i++){
	if(fabsf(in[i])<rs->recordLinear)
	  rs->isLinear++;
	else
	  rs->isNotLinear++;
	out[i]=tansig(in[i]);
      }
      
    }else{
      for(i=0;i<rs->annLayerSize[layer];i++){
	out[i]=tansig(in[i]);
      }
    }
    break;
  case -1://per neuron activation.
    if(rs->annTypeArray==NULL){
      printf("Type not defined - assuming linear\n");
    }else{
      for(i=0;i<rs->annLayerSize[layer];i++){
	switch(rs->annTypeArray[rs->annCum[layer]+i]){
	case 0://no activation
	  out[i]=in[i];
	  break;
	case 1://tansig
	  out[i]=tansig(in[i]);
	  break;
	default:
	  printf("Unknown activation function %d  for layer %d - assuming linear\n",rs->annTypeArray[rs->annCum[layer]+i],layer);
	  break;
	}
      }
    }
    break;
  default:
    printf("Unknown activation function %d for layer %d: Assuming linear\n",rs->annLayerType[layer],layer);
    break;
  }

}

/**
   Called by single thread per frame - end of frame
   Do any post processing here.
   This it typically called by a non-subaperture processing thread.
   At the end of this method, dmCommand must be ready...
   Note, while this is running, subaperture processing of the next frame may start.
*/
int reconFrameFinished(void *reconHandle,int err){//globalStruct *glob){
  //Note: dmCommand=glob->arrays->dmCommand.
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;//glob->reconStruct;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->postbuf];
  float *bleedVal=rs->bleedVal;
  int i,bleedGroup;
  float *dmCommand=reconStruct->arr->dmCommand;
#if !defined(USEAGBBLAS)
  CBLAS_ORDER order=CblasRowMajor;
  CBLAS_TRANSPOSE trans=CblasNoTrans;
  float alpha=1.,beta=1.;
  int inc=1;
#endif
  float *annTmp=rs->annTmp;
  int annNLayers=rs->annNLayers;
  float *annBias=rs->annBias;
  float *annTmpOut=rs->annTmpOut;
  int *annCum=rs->annCum;
  int *annCum2=rs->annCum2;
  int *annLayerSize=rs->annLayerSize;
  float *annWeights=rs->annWeights;
  float *annScale=rs->annScale;
  float *annOffset=rs->annOffset;
  int nacts=rs->nacts;
  //So far, we have multiplied input slopes with first weighting matrix, and added bias.  Results are in annTmp.
  //So, we need to activate this, and then continue for the other hidden layers.
  rs->isLinear=0;
  rs->isNotLinear=0;
  activate(rs,0,annTmp,annTmp);
  for(i=1;i<annNLayers;i++){
    //val=activate(dot(weight, annTmp) + bias)
    memcpy(annTmpOut,&annBias[annCum[i]],sizeof(float)*annLayerSize[i]);
#ifdef USEAGBBLAS
    agb_cblas_sgemvRowMN1N111(annLayerSize[i],annLayerSize[i-1],&annWeights[annCum2[i]],annTmp,annTmpOut);
    //sgemv(weights,annTmp,bias,annTmpOut);
#else
    //order,trans,rows,cols,alpha,weights,cols,x,inc,beta,y,inc
    cblas_sgemv(order,trans,annLayerSize[i],annLayerSize[i-1],alpha,&annWeights[annCum2[i]],annLayerSize[i-1],annTmp,inc,beta,annTmpOut,inc);
#endif
    activate(rs,i,annTmpOut,annTmp);
  }
  //Now scale the output.
  if(annScale==NULL){
    if(annOffset!=NULL){
      for(i=0;i<nacts;i++){
	dmCommand[i]+=annTmp[i]+annOffset[i];
      }
    }else{
      for(i=0;i<nacts;i++){
	dmCommand[i]+=annTmp[i];
      }
    }
  }else{
    if(annOffset!=NULL){
      for(i=0;i<nacts;i++){
	dmCommand[i]+=annTmp[i]*annScale[i]+annOffset[i];
      }
    }else{
      for(i=0;i<nacts;i++){
	dmCommand[i]+=annTmp[i]*annScale[i];
      }
    }
  }
  if(rs->bleedGain!=0. || rs->bleedGainArr!=NULL){//compute the bleed value
    memset(bleedVal,0,sizeof(float)*rs->bleedGroups);
    for(i=0; i<nacts; i++){
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
    for(i=0; i<nacts; i++){
      if(rs->bleedGroupArr!=NULL)
	bleedGroup=rs->bleedGroupArr[i];
      else
	bleedGroup=0;
      dmCommand[i]-=bleedVal[bleedGroup];
    }
  }
  if(reconStruct->reconFrameno!=NULL){
    rs->isNotLinear+=rs->isLinear;//get the total
    if(rs->isNotLinear!=0)
      reconStruct->reconFrameno[0]=(unsigned int)((rs->isLinear*100)/(rs->isNotLinear));
  }
  if(err==0)
    memcpy(reconStruct->latestDmCommand,dmCommand,sizeof(float)*nacts);
  return 0;
}
/**
   Called by the single thread per frame, when the actuator values aren't being sent to the dm - so we need to reset ourselves.
   May not be called at all, if the loop is closed.
   Not called by a subap processing thread - subap processing for next frame have have started before this is called...
*/
int reconOpenLoop(void *reconHandle){//globalStruct *glob){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;//glob->reconStruct;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->postbuf];
  memcpy(reconStruct->latestDmCommand,rs->v0,sizeof(float)*rs->nacts);
  return 0;

}
 
