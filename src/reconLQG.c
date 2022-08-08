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
LQG library.

Operations performed are as follows:

In reconStartFrame,
\phi^n_n = A.\phi_n +(A-Hinfwfs0).\phi_{n-1} - Hinfdm0 .u_{n-2} - (optional) Hinfdm0 .u_{n-1}
\phi^n_{n-1} = \phi_n - Hinfdm1 . u_{n-2} -Hinfwfs1.\phi_{n-1} - (optional) Hinfdm1 . u_{n-1}
u^n_{n-2} = u_{n-1}
u^n_{n-1} = N . \phi^n_n + (optional) N . \phi^n_{n-1}

Then in reconNewSlopes:
\phi^n_n += Hinf0 . s
\phi^n_{n-1} += Hinf1 . s
u^n_{n-1} += N . Hinf0 . s

u^n_{n-1} then gets sent to DM.

NewFrameSync:
X^n then gets copied to X.

*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <sys/mman.h>
#include "agbcblas.h"

#include "darc.h"
#include "circ.h"
#define NEWRECON
#include "rtcrecon.h"
#include "buffer.h"

#define LEAK_INTEGRATOR_FACTOR 0.95
//#define LEAK_INTEGRATOR_FACTOR 1.0

//p is lqgPhaseSize (430), a is lqgActSize (probably==nacts), s is no of slopes.
typedef enum{
  BLEEDGAIN,          
  KEEPLASTSTATE,
  LQGAHWFS,//shape 2p x p (equal to A2-HWFS0, -HWFS1)
  LQGACTSIZE, // a
  LQGATUR,//shape p x p
  LQGHT,//shape 2p x s, stored transposed.
  LQGHDM,//shape 2p x a or 2 x 2p x a (June 2013)
  LQGINVN,//shape a x p, or shape 2 x a x p (June 2014)
  LQGINVNHT,//shape a x s, stored transposed.
  LQGPHASESIZE, // p
  NACTS,
  V0,
  //Add more before this line.
  RECONNBUFFERVARIABLES//equal to number of entries in the enum
}RECONBUFFERVARIABLEINDX;

#define reconMakeNames() bufferMakeNames(RECONNBUFFERVARIABLES,"bleedGain","keepLastState","lqgAHwfs","lqgActSize","lqgAtur","lqgHT","lqgHdm","lqgInvN","lqgInvNHT","lqgPhaseSize","nacts","v0")
typedef struct{
  int phaseStart;
  int partPhaseSize;
}DoStruct;

typedef struct{
  //ReconStructEntry rs[2];
  //int buf;//current buffer being used
  //int postbuf;//current buffer for post processing threads.
  //int swap;//set if need to change to the other buffer.
  int dmReady;
  int loopOpen;
  int totCents;
  int nacts;
  DoStruct *doS;
  int lqgPhaseSize;
  int lqgActSize;
  float *lqgHT;
  float *lqgAHwfs;
  float *lqgAtur;
  float *lqgHdm2;
  float *lqgHdm1;
  float *lqgInvN;
  float *lqgInvN1;
  float *lqgInvNHT;
  int PhiSize;
  int USize;
  float *v0;
  float bleedGainOverNact;
  float *U[3];
  float *Phi[2];
  float *PhiNew[2];
  float **PhiNewPart;
  float **Upart;
  float *stateSave;
  int saveSize;
  int *clearPart;
  int keepLastState;
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
  char *prefix;
  circBuf *rtcLqgBuf;
  unsigned int frameno;
  double timestamp;
  float *circData;
  int circDataSize;
  int clearU0;
}ReconStruct;

/**
   Called to free the reconstructor module when it is being closed.
*/
int reconClose(void **reconHandle){
  ReconStruct *rs=(ReconStruct*)*reconHandle;
  int i;
  char *buf;
  printf("Closing LQG reconstruction library\n");
  if(rs!=NULL){
    if(rs->paramNames!=NULL)
      free(rs->paramNames);
    pthread_mutex_destroy(&rs->dmMutex);
    pthread_cond_destroy(&rs->dmCond);
    if(rs->U[0]!=NULL)free(rs->U[0]);
    if(rs->U[1]!=NULL)free(rs->U[1]);
    if(rs->U[2]!=NULL)free(rs->U[2]);
    if(rs->Phi[0]!=NULL)free(rs->Phi[0]);
    if(rs->Phi[1]!=NULL)free(rs->Phi[1]);
    if(rs->PhiNew[0]!=NULL)free(rs->PhiNew[0]);
    if(rs->PhiNew[1]!=NULL)free(rs->PhiNew[1]);
    
    if(rs->doS!=NULL)
      free(rs->doS);
    if(rs->clearPart!=NULL)
      free(rs->clearPart);
    if(rs->PhiNewPart!=NULL){
      for(i=0; i<rs->nthreads;i++){
	if(rs->PhiNewPart[i]!=NULL)
	  free(rs->PhiNewPart[i]);
      }
      free(rs->PhiNewPart);
    }
    if(rs->Upart!=NULL){
      for(i=0; i<rs->nthreads;i++){
	if(rs->Upart[i]!=NULL)
	  free(rs->Upart[i]);
      }
      free(rs->Upart);
    }
    if(asprintf(&buf,"/%srtcLqgBuf",rs->prefix)==-1){
      printf("unable to get name %srtcLqgBuf\n",rs->prefix);
    }else{
      if(shm_unlink(buf))
	printf("Unable to unlink /dev/shm%s\n",buf);
      free(buf);
    }
    free(rs);
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
  ReconStruct *rs=(ReconStruct*)reconHandle;//threadInfo->globals->reconStruct;
  RECONBUFFERVARIABLEINDX i;
  int *nbytes=rs->nbytes;
  void **values=rs->values;
  char *dtype=rs->dtype;
  int *index=rs->index;
  int dim;
  //swap the buffers...
  rs->totCents=totCents;
  rs->arr=arr;
  bufferGetIndex(pbuf,RECONNBUFFERVARIABLES,rs->paramNames,rs->index,rs->values,rs->dtype,rs->nbytes);
  /*
  BLEEDGAIN,
  LQGACTSIZE, // a
  LQGAHWFS,//shape 2p x p (equal to A2-HWFS0, -HWFS1)
  LQGATUR,//shape p x p
  LQGHT,//shape 2p x s, stored transposed.
  LQGHDM,//shape 2p x a or 2 x 2p x a (added June 2013)
  LQGINVN,//shape a x p or 2 x a x p (added June 2014)
  LQGINVNHT,//shape a x s, stored transposed, equal to N.H[0].
  LQGPHASESIZE, // p
  NACTS,
  V0,
  */
  i=NACTS;
  if(index[i]>=0){//has been found...
    if(dtype[i]=='i' && nbytes[i]==4){
      rs->nacts=*((int*)(values[i]));
    }else{
      printf("nacts error\n");
      writeErrorVA(rs->rtcErrorBuf,-1,frameno,"nacts error");
      err=1;
    }
  }else{
    err=1;
    printf("nacts not found\n");
    writeErrorVA(rs->rtcErrorBuf,-1,frameno,"nacts error");
  }
  i=LQGPHASESIZE;
  int previouslqgPhaseSize = rs->lqgPhaseSize;
  if(index[i]>=0){//found
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      rs->lqgPhaseSize=*((int*)(values[i]));
    }else{
      printf("lqgPhaseSize error\n");
      writeErrorVA(rs->rtcErrorBuf,-1,frameno,"lqgPhaseSize error");
      err=1;
    }
  }else{
    err=1;
    printf("lqgPhaseSize error\n");
    writeErrorVA(rs->rtcErrorBuf,-1,frameno,"lqgPhaseSize error");
  }
  i=LQGACTSIZE;
  if(index[i]>=0){//found
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      rs->lqgActSize=*((int*)(values[i]));
    }else{
      printf("lqgActSize error\n");
      writeErrorVA(rs->rtcErrorBuf,-1,frameno,"lqgActSize error");
      err=1;
    }
  }else{
    err=1;
    printf("lqgActSize error\n");
    writeErrorVA(rs->rtcErrorBuf,-1,frameno,"lqgActSize error");
  }
  i=LQGAHWFS;//shape 2p x p (equal to A2-HWFS0, -HWFS1)
  if(index[i]>=0){
    if(dtype[i]=='f' && nbytes[i]==rs->lqgPhaseSize*rs->lqgPhaseSize*2*sizeof(float)){
      rs->lqgAHwfs=((float*)(values[i]));
    }else{
      err=1;
      printf("lqgAHwfs error\n");
      writeErrorVA(rs->rtcErrorBuf,-1,frameno,"lqgAHwfs error");
    }
  }else{
    err=1;
    printf("lqgAHwfs error\n");
    writeErrorVA(rs->rtcErrorBuf,-1,frameno,"lqgAHwfs error");
  }
  i=LQGATUR;
  if(index[i]>=0 && dtype[i]=='f' && nbytes[i]==rs->lqgPhaseSize*rs->lqgPhaseSize*sizeof(float)){
    rs->lqgAtur=((float*)(values[i]));
  }else{
    err=1;
    printf("lqgAtur error\n");
    writeErrorVA(rs->rtcErrorBuf,-1,frameno,"lqgAtur error");
  }
  i=LQGHT;
  if(index[i]>=0 && dtype[i]=='f' && nbytes[i]==2*rs->lqgPhaseSize*rs->totCents*sizeof(float)){
    rs->lqgHT=(float*)(values[i]);
  }else{
    err=1;
    printf("lqgHT error\n");
    writeErrorVA(rs->rtcErrorBuf,-1,frameno,"lqgHT error");
  }
  i=LQGHDM;
  if(index[i]>=0 && dtype[i]=='f'){
    if(nbytes[i]==2*rs->lqgPhaseSize*rs->lqgActSize*sizeof(float)){
      rs->lqgHdm2=(float*)(values[i]);//phase B mode
      rs->lqgHdm1=NULL;
    }else if(nbytes[i]==2*rs->lqgPhaseSize*2*rs->lqgActSize*sizeof(float)){
      rs->lqgHdm1=(float*)(values[i]);//phase C mode
      rs->lqgHdm2=&(((float*)(values[i]))[2*rs->lqgPhaseSize*rs->lqgActSize]);
    }else{
      err=1;
      printf("lqgHdm error 2\n");
      writeErrorVA(rs->rtcErrorBuf,-1,frameno,"lqgHdm error 2");
    }
  }else{
    err=1;
    printf("lqgHdm error\n");
    writeErrorVA(rs->rtcErrorBuf,-1,frameno,"lqgHdm error");
  }
  i=LQGINVN;
  if(index[i]>=0 && dtype[i]=='f'){
    if(nbytes[i]==rs->lqgPhaseSize*rs->lqgActSize*sizeof(float)){
      rs->lqgInvN=(float*)(values[i]);
      rs->lqgInvN1=NULL;
    }else if(nbytes[i]==2*rs->lqgPhaseSize*rs->lqgActSize*sizeof(float)){
      rs->lqgInvN=(float*)(values[i]);
      rs->lqgInvN1=&(((float*)(values[i]))[rs->lqgPhaseSize*rs->lqgActSize]);
    }else{
      err=1;
      printf("lqgInvN error\n");
      rs->lqgInvN1=NULL;
      writeErrorVA(rs->rtcErrorBuf,-1,frameno,"lqgInvN error");
    }
  }else{
    err=1;
    printf("lqgInvN error\n");
    rs->lqgInvN1=NULL;
    writeErrorVA(rs->rtcErrorBuf,-1,frameno,"lqgInvN error");
  }
  i=LQGINVNHT;
  if(index[i]>=0 && dtype[i]=='f' && nbytes[i]==rs->totCents*rs->lqgActSize*sizeof(float)){
    rs->lqgInvNHT=(float*)(values[i]);
  }else{
    err=1;
    printf("lqgInvNHT error\n");
    writeErrorVA(rs->rtcErrorBuf,-1,frameno,"lqgInvNHT error");
  }
  i=V0;
  if(index[i]>=0){
    if(nbytes[i]==0){
      rs->v0=NULL;
    }else if(dtype[i]=='f' && nbytes[i]==sizeof(float)*rs->nacts){
      rs->v0=(float*)(values[i]);
    }else{
      printf("v0 error\n");
      writeErrorVA(rs->rtcErrorBuf,-1,frameno,"v0 error");
      err=1;
    }
  }else{
    printf("v0 error\n");
    writeErrorVA(rs->rtcErrorBuf,-1,frameno,"v0 error");
    err=1;
  }
  i=BLEEDGAIN;
  if(index[i]>=0 && dtype[i]=='f' && nbytes[i]==sizeof(float)){
    rs->bleedGainOverNact=(*((float*)(values[i])))/rs->nacts;
  }else{
    writeErrorVA(rs->rtcErrorBuf,-1,frameno,"bleedGain error");
    printf("bleedGain error\n");
    err=1;
  }
  
  i=KEEPLASTSTATE;
  if(index[i]>=0){//found
    if(dtype[i]=='i' && nbytes[i]==sizeof(int)){
      rs->keepLastState = *((int*)(values[i]));
      printf("keepLastState set to: %s\n", (rs->keepLastState)?"True":"False");
    }else{
      printf("keepLastState not set: using default: False\n");
    }
  }    

    if (err == 0) {
        // State reset: skip if Phi size doesn't change 
        // and param 'keepLastState' set to true
        if ( rs->keepLastState && (previouslqgPhaseSize == rs->lqgPhaseSize)) {            
            printf("keepLastState set to True and same lqgPhaseSize: keeping state!\n");            
        }else{
            printf("keepLastState set to False or different lqgPhaseSize: resetting state!\n");
        
            if (rs->PhiSize < rs->lqgPhaseSize) {
                rs->PhiSize = rs->lqgPhaseSize;
                if (rs->Phi[0] != NULL)free(rs->Phi[0]);
                if (rs->Phi[1] != NULL)free(rs->Phi[1]);
                if (rs->PhiNew[0] != NULL)free(rs->PhiNew[0]);
                if (rs->PhiNew[1] != NULL)free(rs->PhiNew[1]);
                rs->Phi[0] = calloc(rs->lqgPhaseSize, sizeof (float));
                rs->Phi[1] = calloc(rs->lqgPhaseSize, sizeof (float));
                rs->PhiNew[0] = calloc(rs->lqgPhaseSize, sizeof (float));
                rs->PhiNew[1] = calloc(rs->lqgPhaseSize, sizeof (float));
                if (rs->Phi[0] == NULL || rs->Phi[1] == NULL || rs->PhiNew[0] == NULL || rs->PhiNew[1] == NULL) {
                    printf("malloc of Phi/PhiNew failed in reconLQG\n");
                    err = -3;
                    rs->PhiSize = 0;
                } else {
                    for (i = 0; i < rs->nthreads; i++) {
                        if (rs->PhiNewPart[i] != NULL)free(rs->PhiNewPart[i]);
                        if ((rs->PhiNewPart[i] = calloc(rs->lqgPhaseSize * 2, sizeof (float))) == NULL)
                            err = -3;
                    }
                    if (err == -3) {
                        printf("malloc of PhiNewPart failed in reconLQG\n");
                        rs->PhiSize = 0;
                    }
                }
                
            }else{
                // Set the matrices to 0s
                printf( "Zeroing Phi, PhiNew and PhiNewPart matrices\n" );
                bzero(rs->Phi[0], rs->lqgPhaseSize * sizeof (float));
                bzero(rs->Phi[1], rs->lqgPhaseSize * sizeof (float));
                bzero(rs->PhiNew[0], rs->lqgPhaseSize * sizeof (float));
                bzero(rs->PhiNew[1], rs->lqgPhaseSize * sizeof (float));
                for (i = 0; i < rs->nthreads; i++) {
                    bzero(rs->PhiNewPart[i], rs->lqgPhaseSize * 2 * sizeof (float));
                }                
            }            
            
            if (err == 0 && rs->USize < rs->lqgActSize) {
                rs->USize = rs->lqgActSize;
                for (i=0; i<3;i++){
                    if (rs->U[i] != NULL)free(rs->U[i]);
                    rs->U[i] = NULL;
                }
//                if (rs->U[0] != NULL)free(rs->U[0]);
//                if (rs->U[1] != NULL)free(rs->U[1]);
//                if (rs->U[2] != NULL)free(rs->U[2]);
                
                if ((rs->U[0] = calloc(rs->lqgActSize, sizeof (float))) == NULL) {
                    printf("malloc of U[0] failed in reconLQG\n");
                    err = -3;
                    rs->USize = 0;
//                    rs->U[1] = NULL;
                } else if ((rs->U[1] = calloc(rs->lqgActSize, sizeof (float))) == NULL) {
                    printf("malloc of U[1] failed in reconLQG\n");
                    err = -3;
                    rs->USize = 0;
                    free(rs->U[0]);
                    rs->U[0] = NULL;
                } else if ((rs->U[2] = calloc(rs->lqgActSize, sizeof (float))) == NULL) {
                    printf("malloc of U[2] failed in reconLQG\n");
                    err = -3;
                    rs->USize = 0;
                    free(rs->U[0]);
                    free(rs->U[1]);
                    rs->U[0] = NULL;
                    rs->U[1] = NULL;
                } else {
                    for (i = 0; i < rs->nthreads; i++) {
                        if (rs->Upart[i] != NULL)free(rs->Upart[i]);
                        if ((rs->Upart[i] = calloc(rs->lqgActSize * 2, sizeof (float))) == NULL)
                            err = -3;
                    }
                    if (err == -3) {
                        printf("malloc of Upart failed in reconLQG\n");
                        rs->USize = 0;
                    }
                }
            }else{
                // Set the matrices to 0s
                printf( "Zeroing U[0,1,2] matrices\n" );
                for (i = 0; i < 3; i++) {
                    bzero(rs->U[i], rs->lqgActSize * sizeof (float));
                }
                for (i = 0; i < rs->nthreads; i++) {
                    bzero(rs->Upart[i], rs->lqgActSize * 2 * sizeof (float));
                }                
            }
        }
        
        // END State reset
        
        if (err == 0 && rs->saveSize < rs->lqgActSize * 3 + rs->lqgPhaseSize * 2) {
            if (rs->stateSave != NULL)
                free(rs->stateSave);
            rs->saveSize = rs->lqgPhaseSize * 2 + rs->lqgActSize * 3;
            if ((rs->stateSave = calloc(rs->saveSize, sizeof (float))) == NULL) {
                printf("malloc of stateSave failed in reconLQG\n");
                err = -3;
                rs->saveSize = 0;
            }
        }

        //work out which initialisation work the threads should do.
        rs->doS[0].phaseStart = 0;
        for (j = 0; j < rs->nthreads; j++) {
            if (j > 0)
                rs->doS[j].phaseStart = rs->doS[j - 1].phaseStart + rs->doS[j - 1].partPhaseSize;
            rs->doS[j].partPhaseSize = (rs->lqgPhaseSize - rs->doS[j].phaseStart) / (rs->nthreads - j);
            //printf("lqg thread %d: %d->%d\n",j,rs->doS[j].phaseStart,rs->doS[j].partPhaseSize+rs->doS[j].phaseStart);
        }
        dim = rs->lqgPhaseSize * 2 + rs->lqgActSize * 2;
        if (rs->rtcLqgBuf != NULL && rs->rtcLqgBuf->datasize != dim * sizeof (float)) {
            if (circReshape(rs->rtcLqgBuf, 1, &dim, 'f') != 0) {
                printf("Error reshaping rtcLqgBuf\n");
                err = 1;
            }
        }
    }

  rs->dmReady=0;
  return err;
}


/**
   Initialise the reconstructor module
 */
int reconOpen(char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **reconHandle,int nthreads,unsigned int frameno,unsigned int **reconframeno,int *reconframenoSize,int totCents){
  //Sort through the parameter buffer, and get the things we need, and do 
  //the allocations we need.
  ReconStruct *rs;
  int err=0;
  char *tmp;
  int dim;
  if((rs=calloc(sizeof(ReconStruct),1))==NULL){
    printf("Error allocating recon memory\n");
    //threadInfo->globals->rs=NULL;
    *reconHandle=NULL;
    return 1;
  }
  *reconHandle=(void*)rs;
  rs->prefix=prefix;
  //the condition variable and mutex don't need to be buffer swaped...
  if(pthread_mutex_init(&rs->dmMutex,NULL)){
    printf("Error init recon mutex\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if(pthread_cond_init(&rs->dmCond,NULL)){
    printf("Error init recon cond\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }

  rs->nthreads=nthreads;//this doesn't change.
  if((rs->clearPart=(int*)calloc(sizeof(int),nthreads))==NULL){
    printf("Error in reconLQG allocating clearPart\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if((rs->PhiNewPart=(float**)calloc(sizeof(float*),nthreads))==NULL){
    printf("Error in reconLQG allocating PhiNewPart\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if((rs->Upart=(float**)calloc(sizeof(float*),nthreads))==NULL){
    printf("Error in reconLQG allocating Upart\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if((rs->doS=(DoStruct*)calloc(sizeof(DoStruct),nthreads))==NULL){
    printf("Error in reconLQG alloocing doS\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  rs->rtcErrorBuf=rtcErrorBuf;
  rs->paramNames=reconMakeNames();
  rs->arr=arr;
  err=reconNewParam(*reconHandle,pbuf,frameno,arr,totCents);
  if(err!=0){
    printf("Error in recon...\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if(asprintf(&tmp,"/%srtcLqgBuf",prefix)==-1){
    printf("Error allocing string rtcLqgBuf\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  dim=rs->lqgPhaseSize*2+rs->lqgActSize*2;
  rs->rtcLqgBuf=openCircBuf(tmp,1,&dim,'f',10);
  printf("created lqgBuf at 0x%p\n",rs->rtcLqgBuf);
  free(tmp);
  return 0;
}



/**
   Called by single thread at the start of each frame.
   This thread is not a subap processing thread.
   If doing a param buffer swap, this is called after the swap has completed.
*/
int reconNewFrame(void *reconHandle,unsigned int frameno,double timestamp){
  ReconStruct *rs=(ReconStruct*)reconHandle;
  //float dmCommand=rs->arr->dmCommand;
  //if(rs->swap){
  // rs->swap=0;
  // rs->buf=1-rs->buf;
  //}
  //Now wake up the thread that does the initial processing...
  //No need to get a mutex here, because the preprocessing thread must be sleeping.
  //We do this processing in a separate thread because it has a while to complete, so we may as well let subap processing threads get on with their tasks...
  //pthread_cond_signal(&rs->dmCond);
  //performn precomp=dot(DM,Xpred[phaseSize*2:])


  //first do some final post processing stuff...
  //set the DM arrays ready.
  if(pthread_mutex_lock(&rs->dmMutex))
    printf("pthread_mutex_lock error in setDMArraysReady: %s\n",strerror(errno));
  rs->dmReady=1;
  //wake up any of the subap processing threads that are waiting.
  pthread_cond_broadcast(&rs->dmCond);
  pthread_mutex_unlock(&rs->dmMutex);
  return 0;
}


//called once each frame by a single subap processing thread.
//Copys the Xpred arrays about synchronously.
int reconNewFrameSync(void *reconHandle,unsigned int frameno,double timestamp){
  ReconStruct *rs=(ReconStruct*)reconHandle;
  rs->frameno=frameno;
  rs->timestamp=timestamp;
  if(rs->err==0){//no error from previous frames...
    memcpy(rs->Phi[0],rs->PhiNew[0],sizeof(float)*rs->lqgPhaseSize);
    memcpy(rs->Phi[1],rs->PhiNew[1],sizeof(float)*rs->lqgPhaseSize);
    //memcpy(rs->U[0],rs->UNew[0],sizeof(float)*rs->lqgActSize);
    //memcpy(rs->U[1],rs->UNew[1],sizeof(float)*rs->lqgActSize);
    memcpy(rs->PhiNew[1],rs->Phi[0],sizeof(float)*rs->lqgPhaseSize);
    memcpy(rs->U[2],rs->U[0],sizeof(float)*rs->lqgActSize);//copy to temporary, for later.
    //memset(rs->U[0],0,sizeof(float)*rs->lqgActSize);
    rs->clearU0=1;
  }
  return 0;
}

/**
   Called once per thread at the start of each frame, possibly simultaneously.
*/
int reconStartFrame(void *reconHandle,int cam,int threadno){
  ReconStruct *rs=(ReconStruct*)reconHandle;
  DoStruct doS=rs->doS[threadno];
  if(rs->err==0){//no error from previous frames...
    //computes PhiNew[0]=Atur.Phi[0]-Hdm2[0].U[1]+AHwfs[0].Phi[1] (optional:)-Hdm1[0].U[0]
    //PhiNew[1]=Phi[0]-Hdm2[1].U[1]+AHwfs[1].Phi[1] (optional:)-Hdm1[1].U[0]
    //U[1]=U[0]
    //U[0]=invN.PhiNew[0] + (optional) invN[1].PhiNew[1]
    //PhiNew[0] = Atur.Phi[0]     alpha=1, beta=0
    agb_cblas_sgemvRowMN1N101(doS.partPhaseSize,rs->lqgPhaseSize,&(rs->lqgAtur[doS.phaseStart*rs->lqgPhaseSize]),rs->Phi[0],&(rs->PhiNew[0][doS.phaseStart]));
    //PhiNew[0] -= Hdm2[0].U[1]    alpha=-1, beta=1.
    agb_cblas_sgemvRowMNm1N111(doS.partPhaseSize,rs->lqgActSize,&(rs->lqgHdm2[doS.phaseStart*rs->lqgActSize]),rs->U[1],&(rs->PhiNew[0][doS.phaseStart]));
    //PhiNew[0] -= Hdm1[0].U[0]    alpha=-1, beta=1.
    if(rs->lqgHdm1!=NULL)//phase C mode
      agb_cblas_sgemvRowMNm1N111(doS.partPhaseSize,rs->lqgActSize,&(rs->lqgHdm1[doS.phaseStart*rs->lqgActSize]),rs->U[2],&(rs->PhiNew[0][doS.phaseStart]));
    //PhiNew[0] += AHwfs[0].Phi[1]  alpha=1, beta=1.
    agb_cblas_sgemvRowMN1N111(doS.partPhaseSize,rs->lqgPhaseSize,&(rs->lqgAHwfs[doS.phaseStart*rs->lqgPhaseSize]),rs->Phi[1],&(rs->PhiNew[0][doS.phaseStart]));
    //PhiNew[1] has had Phi[0] copied into it during NewFrameSync.  So now:
    //PhiNew[1] -= Hdm2[1].U[1]   alpha=-1, beta=1
    agb_cblas_sgemvRowMNm1N111(doS.partPhaseSize,rs->lqgActSize,&(rs->lqgHdm2[(rs->lqgPhaseSize+doS.phaseStart)*rs->lqgActSize]),rs->U[1],&(rs->PhiNew[1][doS.phaseStart]));
    //PhiNew[1] -= Hdm1[1].U[0]   alpha=-1, beta=1
    if(rs->lqgHdm1!=NULL)//phase C mode
      agb_cblas_sgemvRowMNm1N111(doS.partPhaseSize,rs->lqgActSize,&(rs->lqgHdm1[(rs->lqgPhaseSize+doS.phaseStart)*rs->lqgActSize]),rs->U[2],&(rs->PhiNew[1][doS.phaseStart]));

    //PhiNew[1] += AHwfs[1].Phi[1]  alpha=1, beta=1.
    agb_cblas_sgemvRowMN1N111(doS.partPhaseSize,rs->lqgPhaseSize,&(rs->lqgAHwfs[(rs->lqgPhaseSize+doS.phaseStart)*rs->lqgPhaseSize]),rs->Phi[1],&(rs->PhiNew[1][doS.phaseStart]));
    
    //Now, U[0] = invN.PhiNew[0]
    //But since PhiNew[0] isn't complete (we don't know what state the other threads are in), we can only do part of this.  Which means we have to form a partial sum, to be added together later in FrameFinishedSync.
    agb_cblas_sgemvRowMN1L101(rs->lqgActSize,doS.partPhaseSize,&(rs->lqgInvN[doS.phaseStart]),rs->lqgPhaseSize,&(rs->PhiNew[0][doS.phaseStart]),rs->Upart[threadno]);
    //And optionally (June 2014), do U[0] += invN[1].PhiNew[1]
    if(rs->lqgInvN1!=NULL){
      agb_cblas_sgemvRowMN1L111(rs->lqgActSize,doS.partPhaseSize,&(rs->lqgInvN1[doS.phaseStart]),rs->lqgPhaseSize,&(rs->PhiNew[1][doS.phaseStart]),rs->Upart[threadno]);
    }
    //and initialise PhiNewPart:
    rs->clearPart[threadno]=1;
    //memset(rs->PhiNewPart[threadno],0,sizeof(float)*2*rs->lqgPhaseSize);
  }
  return 0;
}

/**
   Called multiple times by multiple threads, whenever new slope data is ready
*/
int reconNewSlopes(void *reconHandle,int cam,int centindx,int threadno,int nsubapsDoing){
  int step;//=2;
  ReconStruct *rs=(ReconStruct*)reconHandle;
  float *centroids=rs->arr->centroids;
  //Performs:  rs->Upart[threadno]+=invNH . s
  //And:  rs->PhiNewPart[threadno]+=H[0] . s
  //And:  &rs->PhiNewPart[threadno][lqgPhaseSize]+= H[1].s
  //though the previous 2 are performed as one operation.

  step=2*nsubapsDoing;
  agb_cblas_sgemvColMN1M111(rs->lqgActSize,step,&(rs->lqgInvNHT[centindx*rs->lqgActSize]),&(centroids[centindx]),rs->Upart[threadno]);
  if(rs->clearPart[threadno]){
    rs->clearPart[threadno]=0;
    agb_cblas_sgemvColMN1M101(rs->lqgPhaseSize*2,step,&(rs->lqgHT[centindx*2*rs->lqgPhaseSize]),&(centroids[centindx]),rs->PhiNewPart[threadno]);
  }else{
    agb_cblas_sgemvColMN1M111(rs->lqgPhaseSize*2,step,&(rs->lqgHT[centindx*2*rs->lqgPhaseSize]),&(centroids[centindx]),rs->PhiNewPart[threadno]);
  }
  return 0;
}

/**
   Called once for each thread at the end of a frame
   Here we sum the individual dmCommands together to get the final one.
*/
int reconEndFrame(void *reconHandle,int cam,int threadno,int err){
  ReconStruct *rs=(ReconStruct*)reconHandle;
  if(pthread_mutex_lock(&rs->dmMutex))
    printf("pthread_mutex_lock error in reconEndFrame: %s\n",strerror(errno));
  if(rs->dmReady==0)//wait for the precompute thread to finish
    if(pthread_cond_wait(&rs->dmCond,&rs->dmMutex))
      printf("pthread_cond_wait error in reconEndFrame: %s\n",strerror(errno));
  //now add threadInfo->dmCommand to threadInfo->info->dmCommand.
  //But there is a thread problem with adding to phiNew here, because all portions of phiNew may not yet be ready.
  //agb_cblas_saxpy111(rs->lqgPhaseSize,rs->PhiNewPart[threadno],rs->PhiNew[0]);
  //agb_cblas_saxpy111(rs->lqgPhaseSize,&(rs-Ph>iNewPart[threadno][rs->lqgPhaseSize]),rs->PhiNew[1]);
  if(rs->clearU0){
    rs->clearU0=0;
    memcpy(rs->U[0],rs->Upart[threadno],sizeof(float)*rs->lqgActSize);
  }else{
    agb_cblas_saxpy111(rs->lqgActSize,rs->Upart[threadno],rs->U[0]);
  }
  pthread_mutex_unlock(&rs->dmMutex);
  return 0;
}

/**
   Called by single thread per frame at the end of frame.
   This is called by a subaperture processing thread.
   The bare minimum should be placed here, as most processing should be done in the reconFrameFinished function instead, which doesn't hold up processing.
*/
int reconFrameFinishedSync(void *reconHandle,int err,int forcewrite){
  ReconStruct *rs=(ReconStruct*)reconHandle;
  //xxx do we really need to get the lock here?
  int lc=1;
  float *dmCommand=rs->arr->dmCommand;
  float bleedVal=0.;
  int i;
  if(pthread_mutex_lock(&rs->dmMutex))
    printf("pthread_mutex_lock error in copyThreadPhase: %s\n",strerror(errno));
  rs->dmReady=0;
  if(rs->loopOpen){//reset the internals - so that works when 
    lc=0;
    rs->loopOpen=0;
    //if(rs->v0==NULL){
    // memset(dmCommand,0,sizeof(float)*(rs->nacts<rs->lqgActSize?rs->nacts:rs->lqgActSize));
    //}else{
    // memcpy(dmCommand,rs->v0,sizeof(float)*(rs->nacts<rs->lqgActSize?rs->nacts:rs->lqgActSize));
    //}
  }
  pthread_mutex_unlock(&rs->dmMutex);
  //rs->postbuf=rs->buf;
  if(lc){
    memcpy(rs->U[1],rs->U[2],sizeof(float)*rs->lqgActSize);
    for(i=0;i<rs->nthreads;i++){
      agb_cblas_saxpy111(rs->lqgPhaseSize,rs->PhiNewPart[i],rs->PhiNew[0]);
      agb_cblas_saxpy111(rs->lqgPhaseSize,&(rs->PhiNewPart[i][rs->lqgPhaseSize]),rs->PhiNew[1]);
    }
    memcpy(dmCommand,rs->U[0],sizeof(float)*(rs->nacts<rs->lqgActSize?rs->nacts:rs->lqgActSize));
    //make a copy of the state vector for if the loop is opened by camera glitch...
    memcpy(rs->stateSave,rs->PhiNew[0],sizeof(float)*rs->lqgPhaseSize);
    memcpy(&rs->stateSave[rs->lqgPhaseSize],rs->PhiNew[1],sizeof(float)*rs->lqgPhaseSize);
    memcpy(&rs->stateSave[2*rs->lqgPhaseSize],rs->U[1],sizeof(float)*rs->lqgActSize);
    memcpy(&rs->stateSave[2*rs->lqgPhaseSize+rs->lqgActSize],rs->U[2],sizeof(float)*rs->lqgActSize);
    memcpy(&rs->stateSave[2*rs->lqgPhaseSize+rs->lqgActSize*2],rs->U[0],sizeof(float)*rs->lqgActSize);

      
    if(rs->bleedGainOverNact!=0.){//compute the bleed value
      if(rs->v0==NULL){
	for(i=0; i<rs->nacts; i++)
	  bleedVal+=dmCommand[i];
      }else{
	for(i=0; i<rs->nacts; i++)
	  bleedVal+=dmCommand[i]-rs->v0[i];
      }
      bleedVal*=rs->bleedGainOverNact;
      //bleedVal-=rs->midRangeTimesBleed;//Note - really midrange times bleed over nact... maybe this should be replaced by v0 - to allow a midrange value per actuator?
      for(i=0; i<rs->nacts; i++)
	dmCommand[i]-=bleedVal;
    }
    if(rs->nacts>54 && (dmCommand[54]>20000 || dmCommand[54]<-20000))
      printf("dmCommand[54] %g\n",dmCommand[54]);
  }else{
    //reset/initialise the LQG stuff.
    //printf("Copying U[0] to dmCommand (U[0][%d]=%g)\n",rs->nacts>54?54:0,rs->nacts>54?rs->U[0][54]:rs->U[0][0]);
    printf("resetting...\n");
    //Here, restore the state vector, with a slight decay.
    memcpy(dmCommand,rs->U[0],sizeof(float)*(rs->nacts<rs->lqgActSize?rs->nacts:rs->lqgActSize));
    if(rs->nacts>54 && (dmCommand[54]>20000 || dmCommand[54]<-20000))
      printf("DmCommand[54] %g\n",dmCommand[54]);
    //memset(rs->PhiNew[0],0,sizeof(float)*rs->lqgPhaseSize);//enabled June 2014... for phase C stuff.
    //memset(rs->PhiNew[1],0,sizeof(float)*rs->lqgPhaseSize);//enabled June 2014... for phase C stuff.
    ////memset(rs->U[0],0,sizeof(float)*rs->lqgActSize);//enabled June 2014... for phase C stuff.
    //memset(rs->U[1],0,sizeof(float)*rs->lqgActSize);//enabled June 2014... for phase C stuff.
    //memset(rs->U[2],0,sizeof(float)*rs->lqgActSize);
    //Here, restore the state vector, with a slight decay.
    for(i=0;i<rs->lqgPhaseSize;i++){
      rs->stateSave[i]*=LEAK_INTEGRATOR_FACTOR;
      rs->stateSave[i+rs->lqgPhaseSize]*=LEAK_INTEGRATOR_FACTOR;
      rs->PhiNew[0][i]=rs->stateSave[i];
      rs->PhiNew[1][i]=rs->stateSave[i+rs->lqgPhaseSize];
    }
    for(i=0;i<rs->lqgActSize;i++){
      rs->stateSave[i+rs->lqgPhaseSize*2]*=LEAK_INTEGRATOR_FACTOR;
      rs->stateSave[i+rs->lqgPhaseSize*2+rs->lqgActSize]*=LEAK_INTEGRATOR_FACTOR;
      rs->stateSave[i+rs->lqgPhaseSize*2+rs->lqgActSize*2]*=LEAK_INTEGRATOR_FACTOR;
      rs->U[1][i]=rs->stateSave[i+rs->lqgPhaseSize*2];
      rs->U[2][i]=rs->stateSave[i+rs->lqgPhaseSize*2+rs->lqgActSize];
      rs->U[0][i]=rs->stateSave[i+rs->lqgPhaseSize*2+rs->lqgActSize*2];
    }
  }
  //The lqg equivalent of the following not required because we store it in Xpred (without the bleeding) anyway.
  //memcpy(rs->latestDmCommand,glob->arrays->dmCommand,sizeof(float)*rs->nacts);
  rs->err=err;
  if(forcewrite)
    FORCEWRITE(rs->rtcLqgBuf)=forcewrite;
  if(circSetAddIfRequired(rs->rtcLqgBuf,rs->frameno)){
    if(rs->circDataSize<(rs->lqgPhaseSize+rs->lqgActSize)*2){
      if(rs->circData!=NULL)
	free(rs->circData);
      rs->circDataSize=(rs->lqgPhaseSize+rs->lqgActSize)*2;
      if((rs->circData=malloc(sizeof(float)*rs->circDataSize))==NULL){
	printf("Error allocating circData in lqg\n");
	rs->circDataSize=0;
      }
    }
    if(rs->circData!=NULL){
      memcpy(rs->circData,rs->PhiNew[0],sizeof(float)*rs->lqgPhaseSize);
      memcpy(&rs->circData[rs->lqgPhaseSize],rs->PhiNew[1],sizeof(float)*rs->lqgPhaseSize);
      memcpy(&rs->circData[2*rs->lqgPhaseSize],rs->U[0],sizeof(float)*rs->lqgActSize);
      memcpy(&rs->circData[2*rs->lqgPhaseSize+rs->lqgActSize],rs->U[1],sizeof(float)*rs->lqgActSize);
      circAdd(rs->rtcLqgBuf,rs->circData,rs->timestamp,rs->frameno);
    }
  } 
  
  return 0;
}
/**
   Called by single thread per frame - end of frame
   Do any post processing here.  This is called after reconFrameFinishedSync.
   This it typically called by a non-subaperture processing thread.
   At the end of this method, dmCommand must be ready...
   Note, while this is running, subaperture processing of the next frame may start.
*/
/*
int reconFrameFinished(void *reconHandle,int *err){
  ReconStruct *rs=(ReconStruct*)reconHandle;
  ReconStructEntry *rs=&rs->rs[rs->postbuf];
  float bleedVal=0.;
  int i;
  float *dmCommand=rs->arr->dmCommand;
  //Maybe this isn't necessary for kalman?
  if(rs->bleedGainOverNact!=0.){//compute the bleed value
    if(rs->v0==NULL){
      for(i=0; i<rs->nacts; i++)
	bleedVal+=dmCommand[i];
    }else{
      for(i=0; i<rs->nacts; i++)
	bleedVal+=dmCommand[i]-rs->v0[i];
    }
    bleedVal*=rs->bleedGainOverNact;
    //bleedVal-=rs->midRangeTimesBleed;//Note - really midrange times bleed over nact... maybe this should be replaced by v0 - to allow a midrange value per actuator?
    for(i=0; i<rs->nacts; i++)
      dmCommand[i]-=bleedVal;
  }
  //The lqg equivalent of the following not required because we store it in Xpred (without the bleeding) anyway.
  //memcpy(rs->latestDmCommand,glob->arrays->dmCommand,sizeof(float)*rs->nacts);
  rs->err=err;
  //Final post processing - the predict step - is now done in reconNewFrame()
  return 0;
}
*/
/**
   Called by the single thread per frame, when the actuator values aren't being sent to the dm - so we need to reset ourselves.
   May not be called at all, if the loop is closed.
   Not called by a subap processing thread - subap processing for next frame may have started before this is called...
   Shouldn't write to dmCommand, but reset library internals only.
   Called after reconFrameFinished.
*/
int reconOpenLoop(void *reconHandle){//globalStruct *glob){
  ReconStruct *rs=(ReconStruct*)reconHandle;
  //This thread locking almost certainly not required - at this point dmReady is zero, in which case, the frameFinishedSync function (that uses the loopOpen from the previous iteration) will not yet have been called.
  //if(pthread_mutex_lock(&rs->dmMutex))
  //printf("pthread_mutex_lock error in reconOpenLoop: %s\n",strerror(errno));
  rs->loopOpen=1;
  //pthread_mutex_unlock(&rs->dmMutex);
  return 0;

}




