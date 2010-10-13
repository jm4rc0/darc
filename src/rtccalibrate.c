#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include "qsort.h"
#include "rtccalibrate.h"


typedef struct{
  int curnpxly;
  int curnpxlx;
  int curnpxl;
  float *subap;
  float *sort;
  int subapSize;
  int cursubindx;
  //int npxlCum;
  //int npxlx;
  //int npxly;
  int cam;
}CalThreadStruct;

typedef struct{
  int *npxlx;
  int *npxly;
  int *npxlCum;
  float *calthr;
  float *calsub;
  float *calmult;
  float *fakeCCDImage;
  int useBrightest;
  int *useBrightestArr;
  float powerFactor;
  int thresholdAlgo;
  int totPxls;
  int nsubaps;
  int ncam;
  int *nsub;
  int finalise;
  int nthreads;
  circBuf *rtcErrorBuf;
  arrayStruct *arr;
  CalThreadStruct *tstr;
  char *paramNames;
}CalStruct;

typedef enum{
  CALMULT,
  CALSUB,
  CALTHR,
  FAKECCDIMAGE,
  NCAM,
  NCAMTHREADS,
  NPXLX,
  NPXLY,
  NSUB,
  POWERFACTOR,
  //SUBAPLOCATION,
  THRESHOLDALGO,
  USEBRIGHTEST,
  NBUFFERVARIABLES
}calibrateNames;

//char calibrateParamList[NBUFFERVARIABLES][16]={
#define makeParamNames() bufferMakeNames(NBUFFERVARIABLES,\
  "calmult",\
  "calsub",\
  "calthr",\
  "fakeCCDImage",\
  "ncam",\
  "ncamThreads"\
  "npxlx",\
  "npxly",\
  "nsub",\
  "powerFactor",\
  "thresholdAlgo",\
  "useBrightest"\
)


int copySubap(CalStruct *cstr,int cam,int threadno){
  CalThreadStruct *tstr=&cstr->tstr[threadno];
  int cnt=0;
  int i,j;
  int *loc;
  float *tmp;
  short *pxlbuf=&cstr->arr->pxlbufs[cstr->npxlCum[cam]];
  int npxlx=cstr->npxlx[cam];
  loc=&(cstr->arr->subapLocation[tstr->cursubindx*6]);
  tstr->curnpxly=(loc[1]-loc[0])/loc[2];
  tstr->curnpxlx=(loc[4]-loc[3])/loc[5];
  tstr->curnpxl=tstr->curnpxly*tstr->curnpxlx;
  if(tstr->curnpxl>tstr->subapSize){
    tstr->subapSize=tstr->curnpxl;
    //if((tmp=fftwf_malloc(sizeof(float)*tstr->subapSize))==NULL){//must be freed using fftwf_free.
    if((i=posix_memalign((void**)(&tmp),16,sizeof(float)*tstr->subapSize))!=0){//equivalent to fftwf_malloc... (kernel/kalloc.h in fftw source).
      tmp=NULL;
    
      printf("subap re-malloc failed thread %d, size %d\n",threadno,tstr->subapSize);
      exit(0);
    }
    //fftwf_free(tstr->subap);
    free(tstr->subap);
    tstr->subap=tmp;
    if((tmp=malloc(sizeof(float)*tstr->subapSize))==NULL){
      printf("sort re-malloc failed thread %d, size %d\n",threadno,tstr->subapSize);
      exit(0);
    }
    free(tstr->sort);
    tstr->sort=tmp;
  }
  if(cstr->fakeCCDImage!=NULL){
    for(i=loc[0]; i<loc[1]; i+=loc[2]){
      for(j=loc[3]; j<loc[4]; j+=loc[5]){
	tstr->subap[cnt]=(float)cstr->fakeCCDImage[cstr->npxlCum[cam]+i*cstr->npxlx[cam]+j];
	cnt++;
      }
    }
  }else{
    for(i=loc[0]; i<loc[1]; i+=loc[2]){
      for(j=loc[3]; j<loc[4]; j+=loc[5]){
	//tstr->subap[cnt]=(float)cstr->arr->pxlbuf[cstr->npxlCum[tstr->cam]+i*cstr->npxlx[tstr->cam]+j];
	tstr->subap[cnt]=(float)pxlbuf[i*npxlx+j];
	cnt++;
      }
    }
  }
  return 0;
}



/**
   We only want to use the brightest N (=info->useBrightest) pixels - set the 
   rest to zero.
*/
int applyBrightest(CalStruct *cstr,int threadno){
  CalThreadStruct *tstr=&cstr->tstr[threadno];
  int i;
  //int j;
  //float min=threadInfo->subap[0];
  //int n=info->useBrightest;
  //float v;
  float *sort=tstr->sort;
  float *subap=tstr->subap;
  float thr;
  int useBrightest;
  if(cstr->useBrightestArr!=NULL){
    useBrightest=cstr->useBrightestArr[tstr->cursubindx];
  }else{
    useBrightest=cstr->useBrightest;
  }
  if(useBrightest>=tstr->curnpxl)
    return 0;//want to allow more pixels than there are...
  //copy the first n pixels, and then sort this...
  //Actually - may as well copy the whole lot, and sort the whole lot... not a lot of difference in speed depending on how many pixels you want to keep, and this is more understandable and robust...
  memcpy(sort,subap,sizeof(float)*tstr->curnpxl);
#define cmp(a,b) ((*a)<(*b))
  QSORT(float,sort,tstr->curnpxl,cmp);
#undef cmp

  //The threshold to use is:
  thr=sort[tstr->curnpxl-useBrightest];
  for(i=0; i<tstr->curnpxl; i++){
    if(subap[i]<thr)
      subap[i]=0;
  }
  return 0;
}

inline void vectorPowx(int n,float *in,float powerFactor,float *out){//a replacement for mkl function...
  int i;
  for(i=0; i<n; i++)
    out[i]=powf(in[i],powerFactor);
}
/**
   Raise each pixel to a given power
*/
int applyPowerFactor(CalStruct *cstr,int threadno){
  CalThreadStruct *tstr=&cstr->tstr[threadno];
  if(cstr->powerFactor!=1.){
    vectorPowx(tstr->curnpxl,tstr->subap,cstr->powerFactor,tstr->subap);//an mkl function...
  }
  return 0;
}


/**
   Calibrate CCD pixels more quickly...  Here, to improve performace,
   we do some if tests outside of the main loops.  This means a bit
   more code, but should run faster.
*/
int subapPxlCalibration(CalStruct *cstr,int cam,int threadno){
  CalThreadStruct *tstr=&cstr->tstr[threadno];
  int *loc;
  int i,j;
  int cnt=0;
  int pos,pos2;
  float *subap=tstr->subap;
  float *calmult=cstr->calmult;
  float *calthr=cstr->calthr;
  float *calsub=cstr->calsub;
  int npxlx=cstr->npxlx[cam];
  int npxlCum=cstr->npxlCum[cam];
  //STARTTIMING;
  loc=&(cstr->arr->subapLocation[tstr->cursubindx*6]);
  if(calmult!=NULL && calsub!=NULL){
    if((cstr->thresholdAlgo==1 || cstr->thresholdAlgo==2) && calthr!=NULL){
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	pos=npxlCum+i*npxlx;
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
	pos=npxlCum+i*npxlx;
	for(j=loc[3];j<loc[4]; j+=loc[5]){
	  pos2=pos+j;
	  subap[cnt]*=calmult[pos2];
	  subap[cnt]-=calsub[pos2];
	  cnt++;
	}
      }
    }
  }else if(calmult!=NULL){
    if((cstr->thresholdAlgo==1 || cstr->thresholdAlgo==2) && calthr!=NULL){
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	pos=npxlCum+i*npxlx;
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
	pos=npxlCum+i*npxlx;
	for(j=loc[3];j<loc[4]; j+=loc[5]){
	  subap[cnt]*=calmult[pos+j];
	  cnt++;
	}
      }
    }
  }else if(calsub!=NULL){
    if((cstr->thresholdAlgo==1 || cstr->thresholdAlgo==2 ) && calthr!=NULL){
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	pos=npxlCum+i*npxlx;
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
	pos=npxlCum+i*npxlx;
	for(j=loc[3];j<loc[4]; j+=loc[5]){
	  subap[cnt]-=calsub[pos+j];
	  cnt++;
	}
      }
    }
  }else{//both are null...
    if((cstr->thresholdAlgo==1 || cstr->thresholdAlgo==2) && calthr!=NULL){
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	pos=npxlCum+i*npxlx;
	for(j=loc[3];j<loc[4]; j+=loc[5]){
	  if(subap[cnt]<calthr[pos+j])
	    subap[cnt]=0;
	  cnt++;
	}
      }
    }
  }
  if(cstr->useBrightest>0 || cstr->useBrightestArr!=NULL){//we only want to use brightest useBrightest pixels
    applyBrightest(cstr,threadno);

  }
  //Now do the powerfactor.
  applyPowerFactor(cstr,threadno);
  return 0;
}


int storeCalibratedSubap(CalStruct *cstr,int cam,int threadno){
  CalThreadStruct *tstr=&cstr->tstr[threadno];
  int cnt=0;
  int i,j;
  int *loc;
  float *calpxlbuf=cstr->arr->calpxlbuf;
  float *subap=tstr->subap;
  int indx;
  loc=&(cstr->arr->subapLocation[tstr->cursubindx*6]);
  //printf("store %d %d\n",loc[0],loc[3]);
  for(i=loc[0]; i<loc[1]; i+=loc[2]){
    indx=cstr->npxlCum[cam]+i*cstr->npxlx[cam];
    for(j=loc[3]; j<loc[4]; j+=loc[5]){
      calpxlbuf[indx+j]=subap[cnt];
      cnt++;
    }
  }
  return 0;
}

int calibrateClose(void **calibrateHandle){
  int i;
  CalStruct *cstr=(CalStruct*)*calibrateHandle;
  printf("Closing rtccalibrate library\n");
  if(cstr!=NULL){
    if(cstr->tstr!=NULL){
      for(i=0; i<cstr->nthreads; i++){
	if(cstr->tstr[i].subap!=NULL)
	  free(cstr->tstr[i].subap);
	if(cstr->tstr[i].sort!=NULL)
	  free(cstr->tstr[i].sort);
      }
      free(cstr->tstr);
    }
    free(cstr);
  }
  *calibrateHandle=NULL;
  return 0;
}
int calibrateNewParam(void *calibrateHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr){
  //Here,if we have any finalisation to do, should do it.
  CalStruct *cstr=(CalStruct*)calibrateHandle;
  int index[NBUFFERVARIABLES];
  void *values[NBUFFERVARIABLES];
  char dtype[NBUFFERVARIABLES];
  int nbytes[NBUFFERVARIABLES];
  int nfound;
  int err=0;
  int i;
  //swap the buffers...
  //cstr->buf=1-cstr->buf;
  if(cstr->finalise){
    cstr->finalise=0;
    //do anything to finalise previous frame.

  }
  nfound=bufferGetIndex(pbuf,NBUFFERVARIABLES,cstr->paramNames,index,values,dtype,nbytes);
  if(nfound!=NBUFFERVARIABLES){
    err=1;
    printf("Didn't get all buffer entries for calibrate module:\n");
    for(i=0;i<NBUFFERVARIABLES;i++){
      if(index[i]<0)
	printf("Missing %16s\n",&cstr->paramNames[i*BUFNAMESIZE]);
    }
  }else{
    if(nbytes[NCAM]==sizeof(int) && dtype[NCAM]=='i'){
      cstr->ncam=*((int*)values[NCAM]);
    }else{
      printf("ncam error\n");
      err=1;
    }
    if(nbytes[NSUB]==sizeof(int)*cstr->ncam && dtype[NSUB]=='i')
      cstr->nsub=(int*)values[NSUB];
    else{
      cstr->nsub=NULL;
      printf("nsub error\n");
      err=1;
    }
    if(nbytes[NPXLX]==sizeof(int)*cstr->ncam && dtype[NPXLX]=='i')
      cstr->npxlx=(int*)values[NPXLX];
    else{
      cstr->npxlx=NULL;
      printf("npxlx error\n");
      err=1;
    }
    if(nbytes[NPXLY]==sizeof(int)*cstr->ncam && dtype[NPXLY]=='i')
      cstr->npxly=(int*)values[NPXLY];
    else{
      cstr->npxly=NULL;
      printf("npxly error\n");
      err=1;
    }
    cstr->nsubaps=0;
    cstr->totPxls=0;
    if(err==0){
      for(i=0; i<cstr->ncam; i++){
	cstr->nsubaps+=cstr->nsub[i];
	cstr->totPxls+=cstr->npxlx[i]*cstr->npxly[i];
      }
    }
    /*if(dtype[SUBAPLOCATION]=='i' && nbytes[SUBAPLOCATION]==sizeof(int)*6*cstr->nsubaps)
      cstr->realSubapLocation=(int*)values[SUBAPLOCATION];
    else{
      cstr->realSubapLocation=NULL;
      printf("subapLocation error\n");
      err=1;
      }*/
    if(dtype[POWERFACTOR]=='f' && nbytes[POWERFACTOR]==sizeof(float))
      cstr->powerFactor=*((float*)values[POWERFACTOR]);
    else{
      printf("powerFactor error\n");
      err=1;
    }
    if(nbytes[FAKECCDIMAGE]==0){
      cstr->fakeCCDImage=NULL;
    }else if(dtype[FAKECCDIMAGE]=='f' && nbytes[FAKECCDIMAGE]==sizeof(float)*cstr->totPxls){
      cstr->fakeCCDImage=(float*)values[FAKECCDIMAGE];
    }else{
      cstr->fakeCCDImage=NULL;
      printf("fakeCCDImage error\n");
      err=1;
    }
    if(nbytes[CALMULT]==0)
      cstr->calmult=NULL;
    else if(dtype[CALMULT]=='f' && nbytes[CALMULT]==sizeof(float)*cstr->totPxls)
      cstr->calmult=(float*)values[CALMULT];
    else{
      cstr->calmult=NULL;
      printf("calmult error\n");
      err=1;
    }
    if(nbytes[CALSUB]==0)
      cstr->calsub=NULL;
    else if(dtype[CALSUB]=='f' && nbytes[CALSUB]==sizeof(float)*cstr->totPxls)
      cstr->calsub=(float*)values[CALSUB];
    else{
      cstr->calsub=NULL;
      printf("calsub error\n");
      err=1;
    }
    if(nbytes[CALTHR]==0)
      cstr->calthr=NULL;
    else if(dtype[CALTHR]=='f' && nbytes[CALTHR]==sizeof(float)*cstr->totPxls)
      cstr->calthr=(float*)values[CALTHR];
    else{
      cstr->calthr=NULL;
      printf("calthr error\n");
      err=1;
    }
    if(dtype[THRESHOLDALGO]=='i' && nbytes[THRESHOLDALGO]==sizeof(int)){
      cstr->thresholdAlgo=*((int*)values[THRESHOLDALGO]);
    }else{
      printf("thresholdAlgo error\n");
      err=1;
    }
    if(dtype[USEBRIGHTEST]=='i'){
      if(nbytes[USEBRIGHTEST]==sizeof(int)){
	cstr->useBrightest=*((int*)values[USEBRIGHTEST]);
	cstr->useBrightestArr=NULL;
      }else if(nbytes[USEBRIGHTEST]==sizeof(int)*cstr->nsubaps){
	cstr->useBrightest=0;
	cstr->useBrightestArr=(int*)values[USEBRIGHTEST];
      }else{
	cstr->useBrightest=0;
	cstr->useBrightestArr=NULL;
	printf("useBrightest error\n");
	err=1;
      }
    }else{
      printf("useBrightest error\n");
      err=1;
    }
  }
  return err;
}
int calibrateOpen(char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **calibrateHandle,int nthreads,int frameno){
  CalStruct *cstr;
  int err;
  char *pn;

  if((pn=makeParamNames())==NULL){
    printf("Error making paramList - please recode rtccalibrate.c\n");
    *calibrateHandle=NULL;
    return 1;
  }
  if((cstr=calloc(sizeof(CalStruct),1))==NULL){
    printf("Error allocating calibration memory\n");
    *calibrateHandle=NULL;
    //threadInfo->globals->reconStruct=NULL;
    return 1;
  }
  cstr->paramNames=pn;
  //threadInfo->globals->reconStruct=(void*)reconStruct;
  *calibrateHandle=(void*)cstr;
  //cstr->buf=1;
  cstr->nthreads=nthreads;//this doesn't change.
  if((cstr->tstr=calloc(sizeof(CalThreadStruct),nthreads))==NULL){
    printf("Error allocating CalThreadStruct\n");
    free(cstr);
    *calibrateHandle=NULL;
    return 1;
  }
  cstr->rtcErrorBuf=rtcErrorBuf;
  err=calibrateNewParam(pbuf,*calibrateHandle,frameno,arr);//this will change ->buf to 0.
  if(err!=0){
    printf("Error in calibrateOpen...\n");
    calibrateClose(calibrateHandle);
    *calibrateHandle=NULL;
    return 1;
  }
  return 0;
}
int calibrateNewFrame(void *calibrateHandle){//#subap thread (once)
  //Should do any finalisation, if it is needed, and has not been done by calibrateNewParam (if a buffer switch has been done).
  CalStruct *cstr=(CalStruct*)calibrateHandle;
  if(cstr->finalise){
    cstr->finalise=0;
    //do anything to finalise previous frame.
    printf("todo - calibrateNewFrameFn\n");
  }
  return 0;
}
//Uncomment if needed.
//int calibrateStartFrame(void *calibrateHandle,int cam,int threadno){//subap thread (once per thread)
//}

int calibrateNewSubap(void *calibrateHandle,int cam,int threadno,int cursubindx,float **subap,int *subapSize){//subap thread
  CalStruct *cstr=(CalStruct*)calibrateHandle;
  cstr->tstr[threadno].cursubindx=cursubindx;
  copySubap(cstr,cam,threadno);
  subapPxlCalibration(cstr,cam,threadno);
  //previously, we had to wait until the calpxlbuf had been copied into the circular buffer - but this now has been moved in to the main threads, so we know this will have occurred.
  storeCalibratedSubap(cstr,cam,threadno);
  *subap=cstr->tstr[threadno].subap;
  *subapSize=cstr->tstr[threadno].subapSize;
  cstr->finalise=1;
  return 0;
}
//uncomment if needed
/*
int calibrateEndFrame(void *calibrateHandle,int cam,int threadno,int err){//subap thread (once per thread)
}
int calibrateFrameFinishedSync(void *calibrateHandle,int err){//subap thread (once)
}
int calibrateFrameFinished(void *calibrateHandle,int err){//non-subap thread (once)
}
int calibrateOpenLoop(void *calibrateHandle){
 
}
*/
