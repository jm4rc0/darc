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
   This is a library that can be used for a PCG reconstruction

Parameters required are:
pcgA - typically 1/gain*numpy.dot(pmx.T,pmx).  Shape nacts,nacts.
pcgB - typically pmx.T, shape nacts,ncents.  Will be dotted with slopes to give the pcg b vector.
pcgMinIter
pcgMaxIter
pcgWarmStart - 1 or 0
pcgInit - NULL or a vector to start from (each iter if warmStart==0, otherwise, only when loop is closed).
pcgTolerance - tolerance
pcgPrecond - NULL, or matrix size nacts,nacts, used for preconditioning.  Note, in standard notation, this would be the inverse of a preconditioning matrix.

Operation:

As slopes arrive, dot with pcgB matrix, to give b.

Also need to precompute A.x - this can be done at start of frame.

Once all slopes have arrived, can compute rn, zn, pn and then start the iterations

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
  DECAYFACTOR,
  GAINE,
  NACTS,
  PCGA,
  PCGB,//col major - fortran contig.
  PCGINIT,
  PCGMAXITER,
  PCGMINITER,
  PCGNITERS,
  PCGPRECOND,
  PCGTOLERANCE,
  PCGWARMSTART,
  RECONSTRUCTMODE,
  V0,
  //Add more before this line.
  RECONNBUFFERVARIABLES//equal to number of entries in the enum
}RECONBUFFERVARIABLEINDX;

#define reconMakeNames() bufferMakeNames(RECONNBUFFERVARIABLES,"bleedGain","decayFactor","gainE","nacts","pcgA","pcgB","pcgInit","pcgMaxIter","pcgMinIter","pcgNIters","pcgPrecond","pcgTolerance","pcgWarmStart","reconstructMode","v0")



typedef struct{
  ReconModeType reconMode;
  float *gainE;
  int bArrSize;
  float **bArr;
  float *rmxT;
  float *v0;
  float bleedGainOverNact;
  float *decayFactor;
  int nacts;
  int nactsPrev;
  int totCents;
  int warmStart;
  float *pcgInit;
  float *pcgA;
  float *Ax;//for result of A.x
  int AxArrSize;
  float *b;
  float *zn;
  float *Ap;
  float *pn;
  float *iM;
  float *xn;
  float tol;
  int *niters;
  int miniter;
  int maxiter;
}ReconStructEntry;

typedef struct{
  ReconStructEntry rs[2];
  int doneFirstIter;
  float *pcgB;//only used by main processing threads, so doesnt need to be in rs
  int *AxStart;
  int *AxSize;
  int buf;//current buffer being used
  int postbuf;//current buffer for post processing threads.
  //int swap;//set if need to change to the other buffer.
  int dmReady;
  float *latestDmCommand;
  int latestDmCommandSize;
  pthread_mutex_t dmMutex;
  pthread_cond_t dmCond;
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
  int i,j;
  printf("Closing reconlibrary\n");
  if(reconStruct!=NULL){
    if(reconStruct->paramNames!=NULL)
      free(reconStruct->paramNames);
    pthread_mutex_destroy(&reconStruct->dmMutex);
    pthread_cond_destroy(&reconStruct->dmCond);
    if(reconStruct->latestDmCommand!=NULL)free(reconStruct->latestDmCommand);
    if(reconStruct->AxSize!=NULL)free(reconStruct->AxSize);
    if(reconStruct->AxStart!=NULL)free(reconStruct->AxStart);
    for(j=0;j<2;j++){
      rs=&reconStruct->rs[j];
      if(rs->bArr!=NULL){
	for(i=0; i<reconStruct->nthreads; i++){
	  if(rs->bArr[i]!=NULL)
	    free(rs->bArr[i]);
	}
	free(rs->bArr);
      }
      if(rs->pn!=NULL)free(rs->pn);
      if(rs->xn!=NULL)free(rs->xn);
      if(rs->Ap!=NULL)free(rs->Ap);
      if(rs->zn!=NULL)free(rs->zn);
      if(rs->b!=NULL)free(rs->b);
      if(rs->Ax!=NULL)free(rs->Ax);
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
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;//threadInfo->globals->reconStruct;
  ReconStructEntry *rs;
  RECONBUFFERVARIABLEINDX i;
  int nfound;
  int *nbytes=reconStruct->nbytes;
  void **values=reconStruct->values;
  char *dtype=reconStruct->dtype;
  int *index=reconStruct->index;
  reconStruct->arr=arr;
  //swap the buffers...
  reconStruct->buf=1-reconStruct->buf;
  rs=&reconStruct->rs[reconStruct->buf];
  rs->totCents=totCents;
  nfound=bufferGetIndex(pbuf,RECONNBUFFERVARIABLES,reconStruct->paramNames,reconStruct->index,reconStruct->values,reconStruct->dtype,reconStruct->nbytes);
  if(nfound!=RECONNBUFFERVARIABLES){
    printf("Didn't get all buffer entries for reconpcg module:\n");
    for(j=0; j<RECONNBUFFERVARIABLES; j++){
      if(reconStruct->index[j]<0)
	printf("Missing %16s\n",&reconStruct->paramNames[j*BUFNAMESIZE]);
    }
  }
  rs->nactsPrev=rs->nacts;
  i=NACTS;
  if(err==0 && index[i]>=0){
    if(dtype[i]=='i' && nbytes[i]==4){
      rs->nacts=*((int*)values[i]);
    }else{
      printf("nacts error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"nacts error");
      err=1;
    }
  }else
    err=1;

  i=RECONSTRUCTMODE;
  if(err==0 && index[i]>=0){
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
      err=1;
    }
  }else
    err=1;
  if(err==0){
    i=GAINE;
    if(index[i]>=0){
      if(index[i]>=0){
	if(dtype[i]=='f' && nbytes[i]==sizeof(float)*rs->nacts*rs->nacts){
	  rs->gainE=(float*)values[i];
	}else{
	  printf("gainE error\n");
	  writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"gainE");
	  err=1;
	  rs->gainE=NULL;
	}
      }else{
	rs->gainE=NULL;
      }
    }else{
      if(rs->reconMode==RECONMODE_OPEN)
	err=1;
      else
	printf("Ignoring missing gainE\n");
    }
  }
  i=V0;
  if(err==0 && index[i]>=0){
    if(dtype[i]=='f' && nbytes[i]==sizeof(float)*rs->nacts){
      rs->v0=(float*)values[i];
    }else{
      printf("v0 error\n");
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"v0 error");
      err=1;
    }
  }else
    err=1;
  i=BLEEDGAIN;
  if(err==0 && index[i]>=0){
    if(dtype[i]=='f' && nbytes[i]==sizeof(float)){
      rs->bleedGainOverNact=(*((float*)values[i]))/rs->nacts;
    }else{
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"bleedGain error");
      printf("bleedGain error\n");
      err=1;
    }
  }else
    err=1;
  i=DECAYFACTOR;
  if(err==0){
    if(index[i]>=0){
      if(nbytes[i]==0){
	rs->decayFactor=NULL;
      }else if(dtype[i]=='f' && nbytes[i]==sizeof(float)*rs->nacts){
	rs->decayFactor=(float*)values[i];
      }else{
	rs->decayFactor=NULL;
	writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"decayFactor error");
	printf("decayFactor error\n");
	err=1;
      }
    }else{
      rs->decayFactor=NULL;
      printf("Ignoring missing decay factor\n");
    }
  }
  i=PCGA;
  if(err==0 && index[i]>=0){
    if(nbytes[i]==sizeof(float)*rs->nacts*rs->nacts && dtype[i]=='f'){
      rs->pcgA=(float*)values[i];
    }else{
      rs->pcgA=NULL;
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"pcgA error");
      printf("pcgA error\n");
      err=1;
    }
  }else
    err=1;
  i=PCGB;//Must be col major - ie fortran contig
  if(err==0 && index[i]>=0){
    if(nbytes[i]==sizeof(float)*rs->nacts*rs->totCents && dtype[i]=='f'){
      reconStruct->pcgB=(float*)values[i];
    }else{
      reconStruct->pcgB=NULL;
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"pcgB error");
      printf("pcgB error\n");
      err=1;
    }
  }else
    err=1;
  i=PCGINIT;
  if(err==0){
    if(index[i]>=0){
      if(nbytes[i]==0){
	rs->pcgInit=NULL;
      }else if(nbytes[i]==sizeof(float)*rs->nacts && dtype[i]=='f'){
	rs->pcgInit=(float*)values[i];
      }else{
	rs->pcgInit=NULL;
	writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"pcgInit error");
	printf("pcgInit error\n");
	err=1;
      }
    }else{
      rs->pcgInit=NULL;
      printf("Ignoring missing pcgInit\n");
    }
  }
  i=PCGMAXITER;
  if(err==0 && index[i]>=0){
    if(nbytes[i]==sizeof(int) && dtype[i]=='i'){
      rs->maxiter=*(int*)values[i];
    }else{
      rs->maxiter=100;
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"pcgMaxIter error");
      printf("pcgMaxIter error\n");
      err=1;
    }
  }else
    err=1;
  i=PCGMINITER;
  if(err==0 && index[i]>=0){
    if(nbytes[i]==sizeof(int) && dtype[i]=='i'){
      rs->miniter=*(int*)values[i];
    }else{
      rs->miniter=1;
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"pcgMinIter error");
      printf("pcgMinIter error\n");
      err=1;
    }
  }else
    err=1;
  i=PCGNITERS;
  if(err==0){
    if(index[i]>=0){
      if(nbytes[i]==sizeof(int) && dtype[i]=='i'){
	rs->niters=(int*)values[i];
      }else{
	rs->niters=NULL;
	writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"pcgNIters error");
	printf("pcgNIters error\n");
	err=1;
      }
    }else{
      rs->niters=NULL;
      printf("Ignoring missing pcgNIters\n");
    }
  }
  i=PCGPRECOND;
  if(err==0){
    if(index[i]>=0){
      if(nbytes[i]==0)
	rs->iM=NULL;
      else if(nbytes[i]==sizeof(float)*rs->nacts*rs->nacts && dtype[i]=='f'){
	rs->iM=(float*)values[i];
      }else{
	rs->iM=NULL;
	writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"pcgPrecond error");
	printf("pcgPrecond error\n");
	err=1;
      }
    }else{
      rs->iM=NULL;
      printf("Ignoring missing pcgPrecond\n");
    }
  }
  i=PCGTOLERANCE;
  if(err==0 && index[i]>=0){
    if(nbytes[i]==sizeof(float) && dtype[i]=='f'){
      rs->tol=*(float*)values[i];
    }else{
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"pcgTolerance error");
      printf("pcgTolerance error\n");
      err=1;
    }
  }else
    err=1;
  i=PCGWARMSTART;
  if(err==0 && index[i]>=0){
    if(nbytes[i]==sizeof(int) && dtype[i]=='i'){
      rs->warmStart=*(int*)values[i];
    }else{
      rs->warmStart=1;
      writeErrorVA(reconStruct->rtcErrorBuf,-1,frameno,"pcgWarmStart error");
      printf("pcgWarmStart error\n");
      err=1;
    }
  }else
    err=1;
  //No need to get the lock here because this and newFrame() are called inside glob->libraryMutex.
  reconStruct->dmReady=0;
  if(rs->bArrSize<sizeof(float)*rs->nacts){
    rs->bArrSize=sizeof(float)*rs->nacts;
    if(rs->pn!=NULL)
      free(rs->pn);
    if((rs->pn=malloc(rs->bArrSize))==NULL){
      printf("Error allocating recon pn memory\n");
      err=-2;
      rs->bArrSize=0;
    }
    if(err==0){
      if(rs->Ap!=NULL)
	free(rs->Ap);
      if((rs->Ap=malloc(rs->bArrSize))==NULL){
	printf("Error allocating recon Ap memory\n");
	err=-2;
	rs->bArrSize=0;
      }
    }
    if(err==0){
      if(rs->zn!=NULL)
	free(rs->zn);
      if((rs->zn=malloc(rs->bArrSize))==NULL){
	printf("Error allocating recon zn memory\n");
	err=-2;
	rs->bArrSize=0;
      }
    }
    if(err==0){
      if(rs->xn!=NULL)
	free(rs->xn);
      if((rs->xn=malloc(rs->bArrSize))==NULL){
	printf("Error allocating recon xn memory\n");
	err=-2;
	rs->bArrSize=0;
      }
    }
    if(err==0){
      if(rs->b!=NULL)
	free(rs->b);
      if((rs->b=malloc(rs->bArrSize))==NULL){
	printf("Error allocating recon b memory\n");
	err=-2;
	rs->bArrSize=0;
      }else{
	for(i=0; i<reconStruct->nthreads; i++){
	  if(rs->bArr[i]!=NULL)
	    free(rs->bArr[i]);
	  if((rs->bArr[i]=calloc(sizeof(float),rs->nacts))==NULL){
	    printf("Error allocating recon dmCommand memory\n");
	    err=-2;
	    rs->bArrSize=0;
	  }
	}
      }
    }
  }
  if(rs->AxArrSize<sizeof(float)*rs->nacts){
    rs->AxArrSize=sizeof(float)*rs->nacts;
    if(rs->Ax!=NULL)
      free(rs->Ax);
    if((rs->Ax=malloc(rs->AxArrSize))==NULL){
      printf("Error allocatiing recon dmCommand memory\n");
      err=-2;
      rs->AxArrSize=0;
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
  if(rs->nactsPrev!=rs->nacts){
    reconStruct->AxStart[0]=0;
    reconStruct->AxSize[0]=rs->nacts/reconStruct->nthreads;
    for(i=1;i<reconStruct->nthreads;i++){
      reconStruct->AxStart[i]=reconStruct->AxStart[i-1]+reconStruct->AxSize[i-1];
      reconStruct->AxSize[i]=(rs->nacts-reconStruct->AxStart[i])/(reconStruct->nthreads-i);
    }
  }
  printf("pcg params err: %d\n",err);
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
  if((reconStruct->AxSize=malloc(sizeof(int)*nthreads))==NULL){
    reconClose(reconHandle);
    printf("Error allocating AxSize\n");
    *reconHandle=NULL;
    return 1;
  }
  if((reconStruct->AxStart=malloc(sizeof(int)*nthreads))==NULL){
    reconClose(reconHandle);
    printf("Error allocating AxStart\n");
    *reconHandle=NULL;
    return 1;
  }
  if((reconStruct->rs[0].bArr=calloc(sizeof(float*),nthreads))==NULL){
    printf("Error allocating recon memory[0]\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if((reconStruct->rs[1].bArr=calloc(sizeof(float*),nthreads))==NULL){
    printf("Error allocating recon memory[1]\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  err=reconNewParam(*reconHandle,pbuf,frameno,arr,totCents);//this will change ->buf to 0.
  //rs->swap=0;//no - we don't need to swap.
  //rs=&reconStruct->rs[reconStruct->buf];
  if(err!=0){
    printf("Error in reconpcg...\n");
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
    //Now: dmcommand=dot(gainE,latestDmCommand)
    agb_cblas_sgemvRowNN1N101(rs->nacts,rs->gainE,reconStruct->latestDmCommand,dmCommand);
  }else{//reconmode_offset
    memcpy(dmCommand,rs->v0,sizeof(float)*rs->nacts);
  }
  memset(rs->b,0,sizeof(float)*rs->nacts);
  if(rs->warmStart && reconStruct->doneFirstIter){
    //nothing - since the previous result is already in there.
  }else if(rs->pcgInit!=NULL){
    memcpy(rs->xn,rs->pcgInit,sizeof(float)*rs->nacts);
  }else{//start from zeros.
    memset(rs->xn,0,sizeof(float)*rs->nacts);
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

   Here, we pre-multiply A.x since we have time while waiting for pixels.
*/
int reconStartFrame(void *reconHandle,int cam,int threadno){
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;//threadInfo->globals->reconStruct;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->buf];
  memset((void*)(rs->bArr[threadno]),0,rs->nacts*sizeof(float));

  //Do partial multiply of A, xn using select rows of A and all of xn.
  if(rs->warmStart && reconStruct->doneFirstIter){
    agb_cblas_sgemvRowMN1N101(reconStruct->AxSize[threadno],rs->nacts,&(rs->pcgA[reconStruct->AxStart[threadno]*rs->nacts]),rs->xn,&rs->Ax[reconStruct->AxStart[threadno]]);

  }else if(rs->pcgInit!=NULL){
    agb_cblas_sgemvRowMN1N101(reconStruct->AxSize[threadno],rs->nacts,&(rs->pcgA[reconStruct->AxStart[threadno]*rs->nacts]),rs->pcgInit,&rs->Ax[reconStruct->AxStart[threadno]]);
  }else{
    //Ax not needed - would be all zeros.
  }

  return 0;
}


/**
   Called multiple times by multiple threads, whenever new slope data is ready
   centroids may not be complete, and writing to dmCommand is not thread-safe without locking.

   Compute pcgB . slopes
*/
int reconNewSlopes(void *reconHandle,int cam,int centindx,int threadno,int nsubapsDoing){
  int step;//number of rows to do in mmx...
  ReconStruct *reconStruct=(ReconStruct*)reconHandle;
  float *centroids=reconStruct->arr->centroids;
  ReconStructEntry *rs=&reconStruct->rs[reconStruct->buf];
  //So, here we just do b+=pcgB[:,n]*centx++pcgB[:,n+1]*centy.
  step=2*nsubapsDoing;
  agb_cblas_sgemvColMN1M111(rs->nacts,step,&(reconStruct->pcgB[centindx*rs->nacts]),&(centroids[centindx]),rs->bArr[threadno]);
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
  //now add bArr[threadno] to b.
  agb_cblas_saxpy111(rs->nacts,rs->bArr[threadno],rs->b);
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
  reconStruct->postbuf=reconStruct->buf;
  return 0;
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
  float bleedVal=0.;
  int i,k;
  float *dmCommand=reconStruct->arr->dmCommand;
  float *rn;
  float *zn;
  float rnpznp;
  float rnzn;
  float alpha,beta;
  //Here, we have to do the PCG.
  rn=rs->b;
  if((rs->warmStart && reconStruct->doneFirstIter) || rs->pcgInit!=NULL){
    //rn=b-Ax;
    agb_cblas_saxpym111(rs->nacts,rs->Ax,rn);
  }
  reconStruct->doneFirstIter=1;
  if(rs->iM!=NULL){//zn=iM.rn
    zn=rs->zn;
    agb_cblas_sgemvRowNN1N101(rs->nacts,rs->iM,rn,zn);
  }else
    zn=rn;
  memcpy(rs->pn,zn,sizeof(float)*rs->nacts);
  k=0;
  rnpznp=agb_cblas_sdot11(rs->nacts,rn,zn);
  while(1){
    //Ap=A.pn;
    agb_cblas_sgemvRowNN1N101(rs->nacts,rs->pcgA,rs->pn,rs->Ap);
    rnzn=rnpznp;
    alpha=rnzn/agb_cblas_sdot11(rs->nacts,rs->pn,rs->Ap);
    //xn+=alpha*pn;
    agb_cblas_saxpy11(rs->nacts,alpha,rs->pn,rs->xn);
    if(rs->maxiter>0 && k>=rs->maxiter)
      break;
    //rn-=alpha*Ap;
    agb_cblas_saxpym11(rs->nacts,alpha,rs->Ap,rn);
    if(rs->iM==NULL)
      zn=rn;
    else{//zn=iM.rn
      zn=rs->zn;
      agb_cblas_sgemvRowNN1N101(rs->nacts,rs->iM,rn,zn);
    }
    rnpznp=agb_cblas_sdot11(rs->nacts,rn,zn);
    if(k>rs->miniter && rnpznp<rs->tol)
      break;
    beta=rnpznp/rnzn;
    //pn=zn+beta*pn;
    agb_cblas_sscal1(rs->nacts,beta,rs->pn);
    agb_cblas_saxpy111(rs->nacts,zn,rs->pn);

    k++;
  }
  if(rs->niters!=NULL)
    *rs->niters=k;//write to the param buf
  //answer (phase) is in xn: Add this to dmCommand.
  agb_cblas_saxpy111(rs->nacts,rs->xn,dmCommand);


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
  //bleedVal-=0.5;//do proper rounding...
  if(err==0)
    memcpy(reconStruct->latestDmCommand,dmCommand,sizeof(float)*rs->nacts);
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
  reconStruct->doneFirstIter=0;
  //Some pcg stuff also required.
  return 0;

}
 
