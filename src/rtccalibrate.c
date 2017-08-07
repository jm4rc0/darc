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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#ifdef WITHSIM
#include <fftw3.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#endif
#include "darc.h"
#include "qsort.h"
#include "rtccalibrate.h"


typedef struct{
  int curnpxly;
  int curnpxlx;
  int curnpxl;
  float *subap;
  float *sort;
  int subapSize;
#ifndef OLDMULTINEWFN //this is now the usual case.
  int sortSize;
  float *subapArr;
  int *nproc;
  int nprocSize;
  float **subapHandle;
  int *subapSizeHandle;
#endif
  int cursubindx;
  //int npxlCum;
  //int npxlx;
  //int npxly;
#ifdef WITHSIM
  //  int cam;
  gsl_rng *gslRand;
  int simSubapSize;
  float *simSubap;
  int simulating;
#endif
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
  int useBrightAv;
  int *useBrightAvArr;
  float totVarMin;
  float *totVarMinArr;
  float tvmPrecision;
  float *tvmPrecisionArr;
  int tvmMaxIter;
  int *tvmMaxIterArr;
  float powerFactor;
  int thresholdAlgo;
  int totPxls;
  int nsubaps;
  int ncam;
  char *prefix;
  int *nsub;
  int *subapFlagArr;
  float subapImgGain;
  float *subapImgGainArr;
  float *integratedImg;
  int integratedImgSize;
  //int finalise;
  int nthreads;
  circBuf *rtcErrorBuf;
  arrayStruct *arr;
  CalThreadStruct **tstr;
  char *paramNames;
  //int calpxlbufReady;
  //pthread_mutex_t calmutex;
  //pthread_cond_t calcond;
  int addSubapToCalBuf;
  int *pxlMap;
#ifdef WITHSIM
  int simCorrThreshType;
  int *simCorrThreshTypeArr;
  float simCorrThresh;
  float *simCorrThreshArr;
  float *simFFTPattern;
  int simMode;
  int *simModeArr;
  int *simnpxlcum;
  int *simnpxlx;
  int simnstore;
  float simOffset;
  float *simOffsetArr;
  float simSkybg;
  float *simSkybgArr;
  int simPoisson;
  int *simPoissonArr;
  float simReadnoise;
  float *simReadnoiseArr;
  float simReadbg;
  float *simReadbgArr;
  float simScale;
  float *simScaleArr;
  float simSig;
  float *simSigArr;
  int *simSubapLoc;
  float *simcalthr;
  float *simcalsub;
  float *simcalmult;
  int simuseBrightest;
  int *simuseBrightestArr;
  int simuseBrightAv;
  int *simuseBrightAvArr;
  int simthresholdAlgo;
  circBuf *rtcSimRawBuf;
  circBuf *rtcSimPxlBuf;
  float *simPxlBuf;
  int simPxlBufSize;
  int fftIndexSize;
  int *fftIndex;
  fftwf_plan *fftPlanArray;//array holding all the fftw plans
  pthread_mutex_t fftcreateMutex;
  double timestamp;
  unsigned int frameno;
  int simAllImg;
#endif

}CalStruct;

typedef enum{
  ADDSUBAPTOCAL,//should the subap be added to rtcCalPxlBuf, or overwrite?
  CALMULT,
  CALSUB,
  CALTHR,
  FAKECCDIMAGE,
  IMGGAIN,
  NCAM,
  NCAMTHREADS,
  NPXLX,
  NPXLY,
  NSUB,
  POWERFACTOR,
  PXLMAP,//Map of valid pixels within a subap.  First nsub bytes are int32 which are the offsets, then for every offset, there is 2 int32 values (y,x), followed by y*x float32 values that are the map.  NOTE - this can be used for a windowing function for correlation!
  //SUBAPLOCATION,
#ifdef WITHSIM
  SIMALLIMG,
  SIMCALMULT,
  SIMCALSUB,
  SIMCALTHR,
  SIMCORRTHRTYPE,
  SIMCORRTHRESH,
  SIMFFTPATTERN,
  SIMMODE,
  //  SIMNPXLCUM,
  SIMNPXLCUM,
  SIMNPXLX,
  SIMNSTORE,
  SIMOFFSET,
  SIMPOISSON,
  SIMREADNOISE,
  SIMSCALE,
  SIMSIG,
  SIMSKYBG,
  SIMSUBAPLOC,
  SIMTHRESHALGO,
  SIMUBTHRAV,
  SIMUSEBRIGHT,
#endif
  SUBAPFLAG,
#ifdef WITHSIM
  SUBAPLOCATION,
#endif
  THRESHOLDALGO,
  TOTVARMIN,
  TVMMAXITER,
  TVMPRECISION,
  USEBRIGHTTHRAV,
  USEBRIGHTEST,
  NBUFFERVARIABLES
}calibrateNames;

//char calibrateParamList[NBUFFERVARIABLES][16]={
#ifdef WITHSIM
#define makeParamNames() bufferMakeNames(NBUFFERVARIABLES,\
					 "addSubapToCal", \
					 "calmult",	  \
					 "calsub",	  \
					 "calthr",	  \
					 "fakeCCDImage",  \
					 "imgGain",	  \
					 "ncam",	  \
					 "ncamThreads",	  \
					 "npxlx",	  \
					 "npxly",	  \
					 "nsub",		\
					 "powerFactor",		\
					 "pxlMap",		\
					 "simAllImg",		\
					 "simCalMult",		\
					 "simCalSub",		\
					 "simCalThr",		\
					 "simCorrThrType",	\
					 "simCorrThresh",	\
					 "simFFTPattern",	\
					 "simMode",		\
					 "simNpxlCum",		\
					 "simNpxlx",		\
					 "simNstore",		\
					 "simOffset",		\
					 "simPoisson",		\
					 "simReadnoise",	\
					 "simScale",		\
					 "simSig",		\
					 "simSkybg",		\
					 "simSubapLoc",		\
					 "simThreshAlgo",	\
					 "simUBThrAv",		\
					 "simUseBright",	\
					 "subapFlag",		\
					 "subapLocation",	\
					 "thresholdAlgo",	\
					 "totVarMin",		\
					 "tvmMaxIter",		\
					 "tvmPrecision",	\
					 "useBrightThrAv",	\
					 "useBrightest"		\
					 )
#else
#define makeParamNames() bufferMakeNames(NBUFFERVARIABLES,\
					 "addSubapToCal", \
					 "calmult",	  \
					 "calsub",	  \
					 "calthr",	  \
					 "fakeCCDImage",  \
					 "imgGain",	  \
					 "ncam",	  \
					 "ncamThreads",	  \
					 "npxlx",	  \
					 "npxly",	  \
					 "nsub",	  \
					 "powerFactor",	  \
					 "pxlMap",	  \
					 "subapFlag",	  \
					 "thresholdAlgo", \
					 "totVarMin",		\
					 "tvmMaxIter",		\
					 "tvmPrecision",	\
					 "useBrightThrAv",\
					 "useBrightest"  \
					 )

#endif




int copySubap(CalStruct *cstr,int cam,int threadno){
  CalThreadStruct *tstr=cstr->tstr[threadno];
  int cnt=0;
  int i,j;
  int *loc;
  unsigned short *Hpxlbuf;//=&cstr->arr->pxlbufs[cstr->npxlCum[cam]];
  short *hpxlbuf;
  float *fpxlbuf;
  int *ipxlbuf;
  char *cpxlbuf;
  unsigned char *Cpxlbuf;
  unsigned int *Ipxlbuf;
  float subapImgGain;
  int npxlx=cstr->npxlx[cam];
  loc=&(cstr->arr->subapLocation[tstr->cursubindx*6]);
#ifndef OLDMULTINEWFN //this is now the usual case.
  //already malloced - no need to do this.
#else
  tstr->curnpxly=(loc[1]-loc[0])/loc[2];
  tstr->curnpxlx=(loc[4]-loc[3])/loc[5];
  tstr->curnpxl=tstr->curnpxly*tstr->curnpxlx;
  if(tstr->curnpxl>tstr->subapSize){
    float *tmp;
    tstr->subapSize=tstr->curnpxl;
    //if((tmp=fftwf_malloc(sizeof(float)*tstr->subapSize))==NULL){//must be freed using fftwf_free.
    if((i=posix_memalign((void**)(&tmp),SUBAPALIGN,sizeof(float)*tstr->subapSize))!=0){//equivalent to fftwf_malloc... (kernel/kalloc.h in fftw source).
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
#endif
  if(cstr->fakeCCDImage!=NULL){
    for(i=loc[0]; i<loc[1]; i+=loc[2]){
      for(j=loc[3]; j<loc[4]; j+=loc[5]){
	tstr->subap[cnt]=(float)cstr->fakeCCDImage[cstr->npxlCum[cam]+i*cstr->npxlx[cam]+j];
	cnt++;
      }
    }
  }else{
    if(cstr->arr->pxlbuftype=='b'){
      cpxlbuf=&(((char*)cstr->arr->pxlbufs)[cstr->npxlCum[cam]]);
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	for(j=loc[3]; j<loc[4]; j+=loc[5]){
	  tstr->subap[cnt]=(float)cpxlbuf[i*npxlx+j];
	  cnt++;
	}
      }
    }else if(cstr->arr->pxlbuftype=='B'){
      Cpxlbuf=&(((unsigned char*)cstr->arr->pxlbufs)[cstr->npxlCum[cam]]);
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	for(j=loc[3]; j<loc[4]; j+=loc[5]){
	  tstr->subap[cnt]=(float)Cpxlbuf[i*npxlx+j];
	  cnt++;
	}
      }
    }else if(cstr->arr->pxlbuftype=='h'){
      hpxlbuf=&(((short*)cstr->arr->pxlbufs)[cstr->npxlCum[cam]]);
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	for(j=loc[3]; j<loc[4]; j+=loc[5]){
	  tstr->subap[cnt]=(float)hpxlbuf[i*npxlx+j];
	  cnt++;
	}
      }
    }else if(cstr->arr->pxlbuftype=='H'){
      Hpxlbuf=&(((unsigned short*)cstr->arr->pxlbufs)[cstr->npxlCum[cam]]);
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	for(j=loc[3]; j<loc[4]; j+=loc[5]){
	  tstr->subap[cnt]=(float)Hpxlbuf[i*npxlx+j];
	  cnt++;
	}
      }
    }else if(cstr->arr->pxlbuftype=='i'){
      ipxlbuf=&(((int*)cstr->arr->pxlbufs)[cstr->npxlCum[cam]]);
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	for(j=loc[3]; j<loc[4]; j+=loc[5]){
	  tstr->subap[cnt]=(float)ipxlbuf[i*npxlx+j];
	  cnt++;
	}
      }
    }else if(cstr->arr->pxlbuftype=='I'){
      Ipxlbuf=&(((unsigned int*)cstr->arr->pxlbufs)[cstr->npxlCum[cam]]);
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	for(j=loc[3]; j<loc[4]; j+=loc[5]){
	  tstr->subap[cnt]=(float)Ipxlbuf[i*npxlx+j];
	  cnt++;
	}
      }
    }else if(cstr->arr->pxlbuftype=='f'){
      fpxlbuf=&(((float*)cstr->arr->pxlbufs)[cstr->npxlCum[cam]]);
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	for(j=loc[3]; j<loc[4]; j+=loc[5]){
	  tstr->subap[cnt]=(float)fpxlbuf[i*npxlx+j];
	  cnt++;
	}
      }
    }else{
      printf("Error in rtccalibrate - raw pixel data type %c not understood\n",cstr->arr->pxlbuftype);
    }
  }
  if(cstr->subapImgGainArr!=NULL)
    subapImgGain=cstr->subapImgGainArr[tstr->cursubindx];
  else
    subapImgGain=cstr->subapImgGain;
  if(subapImgGain!=1. && cstr->integratedImg!=NULL){
    int indx;
    //apply the gain.
    for(i=0;i<tstr->curnpxl;i++)
      tstr->subap[i]*=subapImgGain;
    subapImgGain=1.-subapImgGain;
    cnt=0;
    //now add in the previous...
    for(i=loc[0]; i<loc[1]; i+=loc[2]){
      indx=cstr->npxlCum[cam]+i*cstr->npxlx[cam];
      for(j=loc[3]; j<loc[4]; j+=loc[5]){
	tstr->subap[cnt]+=subapImgGain*cstr->integratedImg[indx+j];
	cstr->integratedImg[indx+j]=tstr->subap[cnt];
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
  CalThreadStruct *tstr=cstr->tstr[threadno];
  int i;
  //int j;
  //float min=threadInfo->subap[0];
  //int n=info->useBrightest;
  //float v;
  float *sort=tstr->sort;
  float *subap=tstr->subap;
  float thr;
  int useBrightest;
  int subtract=0;
  int useBrightAv=0;//number of next brightest pixels to average for a background subtraction
  float sub;
  float ssum;
#ifdef WITHSIM
  if(tstr->simulating){
    if(cstr->simuseBrightestArr!=NULL)
      useBrightest=cstr->simuseBrightestArr[tstr->cursubindx];
    else
      useBrightest=cstr->simuseBrightest;
    if(useBrightest<0){
      useBrightest=-useBrightest;
      subtract=1;
      if(cstr->simuseBrightAvArr!=NULL)
	useBrightAv=cstr->simuseBrightAvArr[tstr->cursubindx];
      else
	useBrightAv=cstr->useBrightAv;
    }
  }else{
#endif
  if(cstr->useBrightestArr!=NULL){
    useBrightest=cstr->useBrightestArr[tstr->cursubindx];
  }else{
    useBrightest=cstr->useBrightest;
  }
  if(useBrightest<0){
    useBrightest=-useBrightest;
    subtract=1;
    if(cstr->useBrightAvArr!=NULL)
      useBrightAv=cstr->useBrightAvArr[tstr->cursubindx];
    else
      useBrightAv=cstr->useBrightAv;
  }
#ifdef WITHSIM
  }
#endif
  if(useBrightest>=tstr->curnpxl || useBrightest==0)
    return 0;//want to allow more pixels than there are...
  //copy the first n pixels, and then sort this...
  //Actually - may as well copy the whole lot, and sort the whole lot... not a lot of difference in speed depending on how many pixels you want to keep, and this is more understandable and robust...
  memcpy(sort,subap,sizeof(float)*tstr->curnpxl);
#define cmp(a,b) ((*a)<(*b))
  QSORT(float,sort,tstr->curnpxl,cmp);
#undef cmp

  //The threshold to use is:
  thr=sort[tstr->curnpxl-useBrightest];
  if(subtract){//want to subtract the next brightest pixel
    subtract=tstr->curnpxl-useBrightest-1;
    while(subtract>=0 && sort[subtract]==thr)
      subtract--;
    if(subtract>=0){
      if(useBrightAv>1){
	//now average this many pixels, to find the subtraction threshold.
	ssum=0.;
	subtract++;//we want upto, but not including this.
	if(subtract<useBrightAv)
	  useBrightAv=subtract;
	for(i=subtract-useBrightAv;i<subtract;i++)
	  ssum+=sort[i];
	sub=ssum/useBrightAv;
      }else{
	sub=sort[subtract];
      }
    }else
      sub=0;
    for(i=0; i<tstr->curnpxl; i++){
      if(subap[i]<thr)
	subap[i]=0;
      else
	subap[i]-=sub;
    }
  }else{
    for(i=0; i<tstr->curnpxl; i++){
      if(subap[i]<thr)
	subap[i]=0;
    }
  }
  return 0;
}
/*
Apply a total variadic minimisation to the image
*/
void totVarMin(CalStruct *cstr,int threadno){
  CalThreadStruct *tstr=cstr->tstr[threadno];
  float *subap=tstr->subap;
  float tau=0.25;
  float diff=1.;
  int ny=tstr->curnpxly;
  int nx=tstr->curnpxlx;
  //float N=sqrt(nx*ny);//(cstr->curnpxlx+cstr->curnpxly)/2.;//prob should do sqrt(nx*ny) - but mean probably good enough.
  int nxy=tstr->curnpxl;
  float *div=tstr->sort;
  float *parr=&tstr->sort[nxy];
  float *tmpstore=&tstr->sort[3*nxy];
  float sigma,lamb=1.,precision=0.01,norm,tmpval,g0,g1,oldparr2,oldparr1=0,oldparr0=0,ig,a0,a1;
  int pos0,pos1;
  int y,x,niter=0,tvmMaxIter;
  int printIter=0;
  if(cstr->totVarMinArr!=NULL){
    sigma=cstr->totVarMinArr[tstr->cursubindx];
  }else{
    sigma=cstr->totVarMin;
  }
  if(cstr->tvmPrecisionArr!=NULL)
    precision=cstr->tvmPrecisionArr[tstr->cursubindx];
  else
    precision=cstr->tvmPrecision;
  if(cstr->tvmMaxIterArr!=NULL)
    tvmMaxIter=cstr->tvmMaxIterArr[tstr->cursubindx];
  else
    tvmMaxIter=cstr->tvmMaxIter;
  if(tvmMaxIter<0){
    tvmMaxIter=-tvmMaxIter;
    printIter=1;
  }
  if(sigma==0)
    return;
  memset(tstr->sort,0,sizeof(float)*(tstr->curnpxl*3+tstr->curnpxlx));
  while(diff>precision && niter<tvmMaxIter){
    //do div-=subap/lamb;
    /*saxpy(-1/lamb,subap,div);
    iterarr=tau*grad(div);
    oldparr[:]=parr;
    parr+=iterarr;
    parr*=1/(1+abs_varr(iterarr));
    diff=max(abs(parr-oldparr));
    div=divArr(parr);
    lamb=N*sigma/norm(div);*/
    diff=0.;
    norm=0.;
    pos0=0;
    pos1=nxy;
    for(y=0;y<ny;y++){
      for(x=0;x<nx;x++){
	tmpval=parr[pos0];
	if(y<ny-1)
	  g0=tau*(div[pos0+nx]-subap[pos0+nx]/lamb-(div[pos0]-subap[pos0]/lamb));
	else
	  g0=0;
	if(x<nx-1)
	  g1=tau*(div[pos0+1]-subap[pos0+1]/lamb-(div[pos0]-subap[pos0]/lamb));
	else
	  g1=0;
	oldparr2=oldparr1;
	oldparr0=parr[pos0];
	oldparr1=parr[pos1];
	parr[pos0]+=g0;
	parr[pos1]+=g1;
	ig=1/(1+sqrtf(g0*g0+g1*g1));
	parr[pos0]*=ig;
	parr[pos1]*=ig;
	a0=fabsf(parr[pos0]-oldparr0);
	a1=fabsf(parr[pos1]-oldparr1);
	if(a0>diff)
	  diff=a0;
	if(a1>diff)
	  a1=diff;
	if(y<ny-1)
	  div[pos0]=parr[pos0];
	else
	  div[pos0]=0;
	if(x<nx-1)
	  div[pos0]+=parr[pos1];
	div[pos0]-=tmpstore[x];
	div[pos0]-=oldparr2;
	norm+=div[pos0]*div[pos0];
	tmpstore[x]=tmpval;
	pos0++;
	pos1++;
      }
    }
    lamb=sqrtf(nxy/norm)*sigma;
    niter++;
  }
  for(y=0;y<nxy;y++)
    subap[y]-=div[y]*lamb;
  if(printIter)
    printf("totVarMin iters %d, diff %g, lamb %g\n",niter,diff,lamb);
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
  CalThreadStruct *tstr=cstr->tstr[threadno];
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
  CalThreadStruct *tstr=cstr->tstr[threadno];
  int *loc;
  int i,j;
  int cnt=0;
  int pos,pos2;
  float *subap=tstr->subap;
  float *calmult;//=cstr->calmult;
  float *calthr;//=cstr->calthr;
  float *calsub;//=cstr->calsub;
  int thresholdAlgo;
  int npxlx=cstr->npxlx[cam];
  int npxlCum=cstr->npxlCum[cam];
#ifdef WITHSIM
  if(tstr->simulating){
    calmult=cstr->simcalmult;
    calthr=cstr->simcalthr;
    calsub=cstr->simcalsub;
    thresholdAlgo=cstr->simthresholdAlgo;
  }else{
#endif
    calmult=cstr->calmult;
    calthr=cstr->calthr;
    calsub=cstr->calsub;
    thresholdAlgo=cstr->thresholdAlgo;
#ifdef WITHSIM
  }
#endif
  //STARTTIMING;
  loc=&(cstr->arr->subapLocation[tstr->cursubindx*6]);
  if(calmult!=NULL && calsub!=NULL){
    if((thresholdAlgo==1 || thresholdAlgo==2) && calthr!=NULL){
#ifdef DOPREFETCH //didn't seem to make a difference (or was slightly worse)
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	__builtin_prefetch(&calmult[npxlCum+i*npxlx],0,0);
	__builtin_prefetch(&calsub[npxlCum+i*npxlx],0,0);
	__builtin_prefetch(&calthr[npxlCum+i*npxlx],0,0);
      }
#endif
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
    if((thresholdAlgo==1 || thresholdAlgo==2) && calthr!=NULL){
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
    if((thresholdAlgo==1 || thresholdAlgo==2 ) && calthr!=NULL){
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
    if((thresholdAlgo==1 || thresholdAlgo==2) && calthr!=NULL){
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
  if(cstr->pxlMap!=NULL && cstr->pxlMap[tstr->cursubindx]>=cstr->nsubaps){
    //Apply a per-subap pixel map.  Allows for masks of arbitrary shapes within the sub-aperture.  Applied to the adaptively windowed spot.
    int offset,mapny,mapnx,ny,nx;
    float *map;
    int ssy,ssx;//subap-start-x/y.
    int msy,msx;//map-start-x/y.
    int nny,nnx;
    offset=cstr->pxlMap[tstr->cursubindx];
    mapny=cstr->pxlMap[offset];
    mapnx=cstr->pxlMap[offset+1];
    map=(float*)&cstr->pxlMap[offset+2];
    nx=(loc[4]-loc[3])/loc[5];
    ny=(loc[1]-loc[0])/loc[2];
    //If mapnx or mapny less than nx,ny, then zero out the edge pixels.
    if(mapny<ny){
      ssy=(ny-mapny+1)/2;
      msy=0;
      nny=mapny;
      memset(subap,0,sizeof(float)*nx*ssy);
      memset(&subap[nx*ssy+mapny],0,sizeof(float)*nx*(ny-mapny)/2);
    }else{//mapny>ny or mapny==ny
      ssy=0;
      msy=(mapny-ny+1)/2;
      nny=ny;
    }
    if(mapnx<nx){
      ssx=(nx-mapnx+1)/2;
      msx=0;
      nnx=mapnx;
      for(i=0;i<ny;i++){
	memset(&subap[i*nx],0,sizeof(float)*ssx);
	memset(&subap[i*nx+ssx+mapnx],0,sizeof(float)*(nx-mapnx)/2);
      }
    }else{
      ssx=0;
      msx=(mapnx-nx+1)/2;
      nnx=nx;
    }
    //Now, apply the map.  Clip the map if its larger than the subap.
    for(i=0;i<nny;i++){
      for(j=0;j<nnx;j++){
	subap[(i+ssy)*nx+j+ssx]*=map[(i+msy)*mapnx+j+msx];
      }
    }
  }

#ifdef WITHSIM
  if(tstr->simulating==0){
#endif
    if(cstr->totVarMin!=0 || cstr->totVarMinArr!=NULL)
      totVarMin(cstr,threadno);
#ifdef WITHSIM
  }
#endif
#ifdef WITHSIM
  if(tstr->simulating){
    if(cstr->simuseBrightest!=0 || cstr->simuseBrightestArr!=NULL)
      applyBrightest(cstr,threadno);
  }else{
#endif
  if(cstr->useBrightest!=0 || cstr->useBrightestArr!=NULL)//we only want to use brightest useBrightest pixels
    applyBrightest(cstr,threadno);
#ifdef WITHSIM
  }
#endif
#ifdef WITHSIM
  if(tstr->simulating==0){
#endif
  //Now do the powerfactor.
  applyPowerFactor(cstr,threadno);
#ifdef WITHSIM
  }
#endif
  return 0;
}


int storeCalibratedSubap(CalStruct *cstr,int cam,int threadno){
  CalThreadStruct *tstr=cstr->tstr[threadno];
  int cnt=0;
  int i,j;
  int *loc;
  float *calpxlbuf;//=cstr->arr->calpxlbuf;
  float *subap=tstr->subap;
  int indx;
  //NOTE: If subaps overlap, this is NOT thread-safe.  However, doesn't affect RTC results, only the rtcCalPxlBuf, so probably not a problem.
#ifdef WITHSIM
  if(tstr->simulating)
    calpxlbuf=cstr->simPxlBuf;
  else
#endif
    calpxlbuf=cstr->arr->calpxlbuf;

  loc=&(cstr->arr->subapLocation[tstr->cursubindx*6]);
  //printf("store %d %d\n",loc[0],loc[3]);
  if(cstr->addSubapToCalBuf){
    for(i=loc[0]; i<loc[1]; i+=loc[2]){
      indx=cstr->npxlCum[cam]+i*cstr->npxlx[cam];
      for(j=loc[3]; j<loc[4]; j+=loc[5]){
	calpxlbuf[indx+j]+=subap[cnt];
	cnt++;
      }
    }
  }else{
    for(i=loc[0]; i<loc[1]; i+=loc[2]){
      indx=cstr->npxlCum[cam]+i*cstr->npxlx[cam];
      for(j=loc[3]; j<loc[4]; j+=loc[5]){
	calpxlbuf[indx+j]=subap[cnt];
	cnt++;
      }
    }
  }
  return 0;
}

#ifdef WITHSIM


//Define a function to allow easy indexing into the simFFTPattern array...
#define B(y,x) cstr->simFFTPattern[cstr->simnpxlcum[cam]+(loc[0]+(y)*loc[2])*cstr->simnpxlx[cam]+loc[3]+(x)*loc[5]]
/**
   Calculates the correlation of the spot with the reference.
   simFFTPattern is distributed in memory as per subapLocation, and is
   equal to numpy.conjugate(numpy.fft.fft2(numpy.fft.fftshift(corr)))
   where corr is the reference spot pattern (ie what you are correlating to).
   Should be stored in half complex form (reals then imags)
*/
int simcalcCorrelation(CalStruct *cstr,int cam,int threadno){
  CalThreadStruct *tstr=cstr->tstr[threadno];
  int *loc;
  int i,j,n,m,neven,meven;
  float *a;
  float r1,r2,r3,r4,r5,r6,r7,r8;
  int *tmp;
  fftwf_plan fPlan=NULL,ifPlan=NULL;
  int curnpxlx=tstr->curnpxlx;
  int curnpxly=tstr->curnpxly;
  //int cursubindx=tstr->subindx;
  float *subap=tstr->subap;
  int dx=0,dy=0,simnpxlx,simnpxly;
  //This is how the plans should be created (elsewhere).  Will need a different plan for each different sized subap (see subapLocation).
  //fftwPlan=fftwf_plan_r2r_2d(curnpxly,curnpxlx, double *in, double *out,FFTW_R2HC, FFTW_R2HC, FFTW_ESTIMATE);
  //ifftwPlan=fftwf_plan_r2r_2d(curnpxly,curnpxlx, double *in, double *out,FFTW_HC2R, FFTW_HC2R, FFTW_ESTIMATE);

  loc=&(cstr->simSubapLoc[tstr->cursubindx*6]);
  simnpxly=(loc[1]-loc[0])/loc[2];
  simnpxlx=(loc[4]-loc[3])/loc[5];

  if(simnpxlx>curnpxlx || simnpxly>curnpxly){
    //have to move spot into the middle (ie zero pad it).
    //if(curnpxlx==8)
    //printf("Moving spot %d, %d %d %d %d %d %d\n",tstr->cursubindx,loc[0],loc[1],loc[2],loc[3],loc[4],loc[5]);
    if(tstr->simSubapSize<simnpxlx*simnpxly){
      if(tstr->simSubap!=NULL){
	printf("Freeing existing simSubap\n");
	free(tstr->simSubap);
      }
      tstr->simSubapSize=simnpxlx*simnpxly;
      printf("memaligning simSubap to %dx%d\n",simnpxly,simnpxlx);
      if((i=posix_memalign((void**)(&(tstr->simSubap)),SUBAPALIGN,sizeof(float)*tstr->simSubapSize))!=0){//equivalent to fftwf_malloc... (kernel/kalloc.h in fftw source).
	tstr->simSubapSize=0;
	tstr->simSubap=NULL;
	printf("simSubap re-malloc failed thread %d, size %d\nExiting...\n",threadno,tstr->simSubapSize);
	exit(0);//Nasty!!!
      }
      printf("Aligned, address %p thread %d\n",tstr->simSubap,threadno);
    }
    dx=(simnpxlx-curnpxlx)/2;
    dy=(simnpxly-curnpxly)/2;
    memset(tstr->simSubap,0,sizeof(float)*simnpxlx*simnpxly);
    //copy from subap to simSubap, allowing for the padding.
    //for(i=((loc[1]-loc[0]+loc[2]-1)/loc[2])*loc[2]+loc[0]-loc[2];i>=loc[0];i-=loc[2]){
    for(i=0;i<curnpxly;i++){
      memcpy(&tstr->simSubap[(dy+i)*simnpxlx+dx],&tstr->subap[i*curnpxlx],sizeof(float)*curnpxlx);
    }
    subap=tstr->simSubap;
  }else if(simnpxlx<curnpxlx || simnpxly<curnpxly){
    printf("Error - correlation pattern subaperture locations have subapertures smaller than the real ones.  Please correct this.\n");
  }


  for(i=0; i<cstr->fftIndexSize; i++){
    if(cstr->fftIndex[i*2]==0 || cstr->fftIndex[i*2+1]==0){
      break;
    }
    if(cstr->fftIndex[i*2]==simnpxlx && cstr->fftIndex[i*2+1]==simnpxly){
      fPlan=cstr->fftPlanArray[i*2];
      ifPlan=cstr->fftPlanArray[i*2+1];
      break;
    }
  }
  if(fPlan==NULL){//need to create the plan...
    //only allow 1 thread to create a plan at a time.
    pthread_mutex_lock(&cstr->fftcreateMutex);//reuse camMutex...
    //now just check again... (another thread may have created it while we were waiting for the mutex)...
    for(i=0; i<cstr->fftIndexSize; i++){
      if(cstr->fftIndex[i*2]==0 || cstr->fftIndex[i*2+1]==0){
	break;
      }
      if(cstr->fftIndex[i*2]==simnpxlx && cstr->fftIndex[i*2+1]==simnpxly){
	fPlan=cstr->fftPlanArray[i*2];
	ifPlan=cstr->fftPlanArray[i*2+1];
	break;
      }
    }
    if(fPlan==NULL){
      if(i==cstr->fftIndexSize){//need to make the index larger...
	if((tmp=realloc(cstr->fftIndex,sizeof(int)*2*(cstr->fftIndexSize+16)))==NULL){
	  printf("realloc of fftIndex failed - exiting\n");
	  exit(1);
	}
	//fill the new stuff with zeros...
	cstr->fftIndex=(int*)tmp;
	memset(&cstr->fftIndex[i*2],0,sizeof(int)*2*16);
	if((tmp=realloc(cstr->fftPlanArray,sizeof(fftwf_plan)*2*(cstr->fftIndexSize+16)))==NULL){
	  printf("realloc of fftPlanArray failed - exiting\n");
	  exit(1);
	}
	cstr->fftPlanArray=(fftwf_plan*)tmp;
	memset(&cstr->fftPlanArray[i*2],0,sizeof(fftwf_plan)*2*16);
	cstr->fftIndexSize+=16;
      }
      //now do the planning
      printf("Planning FFTs size %d x %d\n",simnpxly,simnpxlx);
      cstr->fftPlanArray[i*2]=fftwf_plan_r2r_2d(simnpxly,simnpxlx,subap,subap,FFTW_R2HC, FFTW_R2HC, FFTW_ESTIMATE);
      cstr->fftPlanArray[i*2+1]=fftwf_plan_r2r_2d(simnpxly,simnpxlx,subap,subap,FFTW_HC2R, FFTW_HC2R, FFTW_ESTIMATE);
      cstr->fftIndex[i*2]=simnpxlx;
      cstr->fftIndex[i*2+1]=simnpxly;
      fPlan=cstr->fftPlanArray[i*2];
      ifPlan=cstr->fftPlanArray[i*2+1];

    }
    pthread_mutex_unlock(&cstr->fftcreateMutex);//reuse camMutex...

  }
  //FFT the SH image.
  fftwf_execute_r2r(fPlan,subap,subap);

  //Now multiply by the reference...
  //This is fairly complicated due to the half complex format.  If you need to edit this, make sure you know what you're doing.
  //Here, we want to use the real subap location rather than the moving one, because this image map in question (the fft'd psf) doesn't move around, and we're just using subap location for convenience rather than having to identify another way to specify it.
  a=subap;
  n=simnpxlx;
  m=simnpxly;


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
      a[(m-i)*n+n/2]=r8;//changed from r7 on 120830 by agb
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
  fftwf_execute_r2r(ifPlan,subap,subap);
  //and now, if we were zero padding, copy the right parts back.
  //Here, if we have oversampled for the FFT, we may need to clip
  if((simnpxlx>curnpxlx || simnpxly>curnpxly)){
    for(i=0;i<curnpxly;i++){
      memcpy(&tstr->subap[i*curnpxlx],&tstr->simSubap[(dy+i)*simnpxlx+dx],sizeof(float)*curnpxlx);
    }
  }
  return 0;
}

#undef B

/**
   Applies the chosen threshold algorithm to the correlation
   There are 4 possible thresholding algorithms.  The threshold is either a fixed value, or a fraction of the maximum value found in subap.  This threshold is then either subtracted with everything negative being zero'd, or anything below this threshold is zero'd.
*/
enum CorrelationThresholdType{CORR_ABS_SUB,CORR_ABS_ZERO,CORR_FRAC_SUB,CORR_FRAC_ZERO};

int simthresholdCorrelation(CalStruct *cstr,int cam,int threadno){
  CalThreadStruct *tstr=cstr->tstr[threadno];
  int i;
  float thresh=0.;
  float *subap=tstr->subap;
  int curnpxl=tstr->curnpxl;
  int simCorrThreshType;
  float simCorrThresh;
  if(cstr->simCorrThreshTypeArr!=NULL)
    simCorrThreshType=cstr->simCorrThreshTypeArr[tstr->cursubindx];
  else
    simCorrThreshType=cstr->simCorrThreshType;
  if(cstr->simCorrThreshArr!=NULL)
    simCorrThresh=cstr->simCorrThreshArr[tstr->cursubindx];
  else
    simCorrThresh=cstr->simCorrThresh;
  if(simCorrThreshType==CORR_ABS_SUB || simCorrThreshType==CORR_ABS_ZERO){
    thresh=simCorrThresh;
  }else if(simCorrThreshType==CORR_FRAC_SUB || simCorrThreshType==CORR_FRAC_ZERO){
    //first find the max.
    for(i=0; i<curnpxl; i++){
      if(thresh<subap[i])
	thresh=subap[i];
    }
    thresh*=simCorrThresh;
  }
  if(simCorrThreshType==CORR_ABS_SUB || simCorrThreshType==CORR_FRAC_SUB){
    for(i=0; i<curnpxl; i++){
      if(subap[i]<thresh){
	subap[i]=0;
      }else{
	subap[i]-=thresh;
      }
    }
  }else if(simCorrThreshType==CORR_ABS_ZERO || simCorrThreshType==CORR_FRAC_ZERO){
    for(i=0; i<curnpxl; i++){
      if(subap[i]<thresh)
	subap[i]=0;
    }
  }
  return 0;
}


void simAddNoise(CalStruct *cstr,int cam,int threadno){
  CalThreadStruct *tstr=cstr->tstr[threadno];
  int i;
  float *subap=tstr->subap;
  int curnpxl=tstr->curnpxl;
  //Operations:  Normalise, scale, add offset, poissonise, readout.
  float scale;
  float sum=0;
  float offset;
  float sig;
  float skybg;
  int poisson;
  double readnoise;
  if(cstr->simScaleArr!=NULL)
    scale=cstr->simScaleArr[tstr->cursubindx];
  else
    scale=cstr->simScale;
  if(cstr->simOffsetArr!=NULL)
    offset=cstr->simOffsetArr[tstr->cursubindx];
  else
    offset=cstr->simOffset;
  if(cstr->simSigArr!=NULL)
    sig=cstr->simSigArr[tstr->cursubindx];
  else
    sig=cstr->simSig;
  if(cstr->simSkybgArr!=NULL)
    skybg=cstr->simSkybgArr[tstr->cursubindx];
  else
    skybg=cstr->simSkybg;
  if(cstr->simPoissonArr!=NULL)
    poisson=cstr->simPoissonArr[tstr->cursubindx];
  else
    poisson=cstr->simPoisson;
  if(cstr->simReadnoiseArr!=NULL)
    readnoise=(double)cstr->simReadnoiseArr[tstr->cursubindx];
  else
    readnoise=(double)cstr->simReadnoise;

  if(sig>0){//normalise subap intensity to 1, then scale.
    for(i=0;i<curnpxl;i++)
      sum+=subap[i];
    if(sum!=0)
      sig=sig/sum;
    else
      sig=0;
  }else//if <0, then multiply by scale...
    sig=-sig;
  if(sig>0 && sig!=1){//just multiply by scale.
    for(i=0;i<curnpxl;i++)
      subap[i]*=sig;
  }
  if(skybg!=0){
    for(i=0;i<curnpxl;i++)
      subap[i]+=skybg;
  }
  //now Poissonise
  if(poisson){
    for(i=0;i<curnpxl;i++)
      subap[i]=(float)gsl_ran_poisson(tstr->gslRand,(double)subap[i]);
  }
  //now readnoise
  if(readnoise){
    for(i=0;i<curnpxl;i++)
      subap[i]+=(float)gsl_ran_gaussian(tstr->gslRand,readnoise);
  }
  //now offset and scale
  if(offset){
    for(i=0;i<curnpxl;i++)
      subap[i]+=offset;
  }
  if(scale!=1){
    for(i=0;i<curnpxl;i++)
      subap[i]*=scale;
  }


}

int simConvolveSubap(CalStruct *cstr,int cam,int threadno){
  simcalcCorrelation(cstr,cam,threadno);
  simthresholdCorrelation(cstr,cam,threadno);
  return 0;
}


int simulateSubap(CalStruct *cstr,int cam,int threadno){
  CalThreadStruct *tstr=cstr->tstr[threadno];
  int simMode;
  if(cstr->simModeArr!=NULL)
    simMode=cstr->simModeArr[tstr->cursubindx];
  else
    simMode=cstr->simMode;
  if(simMode){
    tstr->simulating=1;
    subapPxlCalibration(cstr,cam,threadno);
    //we now have a calibrated image - so convolve it...
    if(cstr->simFFTPattern!=NULL)
      simConvolveSubap(cstr,cam,threadno);
    simAddNoise(cstr,cam,threadno);
    //Now store, in rtcSimPxlBuf.
    storeCalibratedSubap(cstr,cam,threadno);
    //need to copy the simulated subap back into arr->pxlbufs - so that it appears in rtcPxlBuf.  This may include clipping/type conversion.
    tstr->simulating=0;
  }
  return 0;
}
#endif




int calibrateClose(void **calibrateHandle){
  int i;
  CalStruct *cstr=(CalStruct*)*calibrateHandle;
  printf("Closing rtccalibrate library\n");
  if(cstr!=NULL){
    //pthread_mutex_destroy(&cstr->calmutex);
    //pthread_cond_destroy(&cstr->calcond);
    if(cstr->paramNames!=NULL)
      free(cstr->paramNames);
    if(cstr->npxlCum!=NULL)
      free(cstr->npxlCum);
    if(cstr->integratedImg!=NULL)
      free(cstr->integratedImg);
    if(cstr->tstr!=NULL){
      for(i=0; i<cstr->nthreads; i++){
	if(cstr->tstr[i]!=NULL){
	  //if(cstr->tstr[i]->subap!=NULL)
	  // free(cstr->tstr[i]->subap);
	  if(cstr->tstr[i]->subapSizeHandle!=NULL)//this is always the case, unless the subap has done no work at all...
	    *(cstr->tstr[i]->subapSizeHandle)=0;
	  if(cstr->tstr[i]->subapHandle!=NULL)
	    *(cstr->tstr[i]->subapHandle)=NULL;
	  if(cstr->tstr[i]->sort!=NULL)
	    free(cstr->tstr[i]->sort);
#ifdef WITHSIM
	  if(cstr->tstr[i]->simSubap!=NULL)
	    free(cstr->tstr[i]->simSubap);
	  if(cstr->tstr[i]->gslRand!=NULL)
	    gsl_rng_free(cstr->tstr[i]->gslRand);
#endif
	  free(cstr->tstr[i]);
	}
      }
      free(cstr->tstr);
    }
#ifdef WITHSIM
    if(cstr->rtcSimPxlBuf!=NULL)
      circClose(cstr->rtcSimPxlBuf);
    if(cstr->rtcSimRawBuf!=NULL)
      circClose(cstr->rtcSimRawBuf);
    pthread_mutex_destroy(&cstr->fftcreateMutex);
    for(i=0;i<cstr->fftIndexSize;i++){
      if(cstr->fftIndex[i*2]==0 || cstr->fftIndex[i*2+1]==0){
	break;
      }
      fftwf_destroy_plan(cstr->fftPlanArray[i*2]);
      fftwf_destroy_plan(cstr->fftPlanArray[i*2+1]);
    }
    if(cstr->fftIndex!=NULL)
      free(cstr->fftIndex);
    if(cstr->fftPlanArray!=NULL)
      free(cstr->fftPlanArray);

#endif
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
  //if(cstr->finalise){
  //  cstr->finalise=0;
    //do anything to finalise previous frame.

  //}
  cstr->arr=arr;
  nfound=bufferGetIndex(pbuf,NBUFFERVARIABLES,cstr->paramNames,index,values,dtype,nbytes);
  if(nfound!=NBUFFERVARIABLES){// && (nfound!=NBUFFERVARIABLES-1 && (index[USEBRIGHTTHRAV]>=0 || index[IMGGAIN]>=0)) && (nfound!=NBUFFERVARIABLES-2){
    err=0;
    for(i=0;i<NBUFFERVARIABLES;i++){
      if(index[i]<0 && (i!=USEBRIGHTTHRAV && i!=IMGGAIN && i!=TOTVARMIN && i!=TVMPRECISION && i!=TVMMAXITER && i!=ADDSUBAPTOCAL && i!=PXLMAP)){
#ifdef WITHSIM
	if(i<SIMALLIMG || i>SIMUSEBRIGHT){
#endif
	  printf("Error: missing %16s\n",&cstr->paramNames[i*BUFNAMESIZE]);
	  err=1;
#ifdef WITHSIM
	}
#endif
      }
    }
    if(err)
      printf("Didn't get all buffer entries for calibrate module:\n");
  }
  if(err==0){
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
    if(cstr->npxlCum==NULL){
      if((cstr->npxlCum=malloc(sizeof(int)*(cstr->ncam+1)))==NULL){
	printf("Error mallocing npxlCum\n");
	err=1;
      }
    }
    if(cstr->npxlCum!=NULL){
      cstr->npxlCum[0]=0;
      if(err==0){
	for(i=0; i<cstr->ncam; i++){
	  cstr->nsubaps+=cstr->nsub[i];
	  cstr->totPxls+=cstr->npxlx[i]*cstr->npxly[i];
	  cstr->npxlCum[i+1]=cstr->npxlCum[i]+cstr->npxlx[i]*cstr->npxly[i];
	}
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
    if(dtype[SUBAPFLAG]=='i' && nbytes[SUBAPFLAG]==sizeof(int)*cstr->nsubaps){
      cstr->subapFlagArr=(int*)values[SUBAPFLAG];
    }else{
      printf("subapFlag error\n");
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
    if(index[USEBRIGHTTHRAV]>=0){
      if(dtype[USEBRIGHTTHRAV]=='i'){
	if(nbytes[USEBRIGHTTHRAV]==sizeof(int)){
	  cstr->useBrightAv=*((int*)values[USEBRIGHTTHRAV]);
	  cstr->useBrightAvArr=NULL;
	}else if(nbytes[USEBRIGHTTHRAV]==sizeof(int)*cstr->nsubaps){
	  cstr->useBrightAv=0;
	  cstr->useBrightAvArr=(int*)values[USEBRIGHTTHRAV];
	}else{
	  cstr->useBrightAv=0;
	  cstr->useBrightAvArr=NULL;
	  printf("useBrighThrAv error\n");
	  err=1;
	}
      }else{
	printf("useBrighThrAv error\n");
	err=1;
      }
    }else{
      printf("useBrightThrAv not found - continuing\n");
      cstr->useBrightAv=0;
      cstr->useBrightAvArr=NULL;
    }
    if(index[TOTVARMIN]>=0){
      if(dtype[TOTVARMIN]=='f'){
	if(nbytes[TOTVARMIN]==sizeof(float)){
	  cstr->totVarMin=*((float*)values[TOTVARMIN]);
	  cstr->totVarMinArr=NULL;
	}else if(nbytes[TOTVARMIN]==sizeof(float)*cstr->nsubaps){
	  cstr->totVarMin=0;
	  cstr->totVarMinArr=(float*)values[TOTVARMIN];
	}else{
	  cstr->totVarMin=0;
	  cstr->totVarMinArr=NULL;
	  printf("totVarMin error\n");
	  err=1;
	}
      }else{
	cstr->totVarMin=0;
	cstr->totVarMinArr=NULL;
	printf("totVarMin error\n");
	err=1;
      }
    }else{
      cstr->totVarMin=0;
      cstr->totVarMinArr=NULL;
    }

    if(index[TVMMAXITER]>=0){
      if(dtype[TVMMAXITER]=='i'){
	if(nbytes[TVMMAXITER]==sizeof(int)){
	  cstr->tvmMaxIter=*((int*)values[TVMMAXITER]);
	  cstr->tvmMaxIterArr=NULL;
	}else if(nbytes[TVMMAXITER]==sizeof(int)*cstr->nsubaps){
	  cstr->tvmMaxIter=100;
	  cstr->tvmMaxIterArr=(int*)values[TVMMAXITER];
	}else{
	  cstr->tvmMaxIter=100;
	  cstr->tvmMaxIterArr=NULL;
	  printf("tvmMaxIter error\n");
	  err=1;
	}
      }else{
	printf("tvmMaxIter error\n");
	cstr->tvmMaxIter=100;
	cstr->tvmMaxIterArr=NULL;
	err=1;
      }
    }else{
      cstr->tvmMaxIter=100;
      cstr->tvmMaxIterArr=NULL;
    }

    if(index[TVMPRECISION]>=0){
      if(dtype[TVMPRECISION]=='f'){
	if(nbytes[TVMPRECISION]==sizeof(float)){
	  cstr->tvmPrecision=*((float*)values[TVMPRECISION]);
	  cstr->tvmPrecisionArr=NULL;
	}else if(nbytes[TVMPRECISION]==sizeof(float)*cstr->nsubaps){
	  cstr->tvmPrecision=0.1;
	  cstr->tvmPrecisionArr=(float*)values[TVMPRECISION];
	}else{
	  cstr->tvmPrecision=0.1;
	  cstr->tvmPrecisionArr=NULL;
	  printf("tvmPrecision error\n");
	  err=1;
	}
      }else{
	printf("tvmPrecision error\n");
	  cstr->tvmPrecision=0.1;
	err=1;
      }
    }else{
      cstr->tvmPrecision=0.1;
      cstr->tvmPrecisionArr=NULL;
    }

    if(index[IMGGAIN]>=0){
      if(dtype[IMGGAIN]=='f'){
	int resetCalImg=0;
	if(nbytes[IMGGAIN]==sizeof(float)){
	  float imgGain;
	  imgGain=*((float*)values[IMGGAIN]);
	  if(imgGain!=1. && cstr->subapImgGain==1.)
	    resetCalImg=1;
	  cstr->subapImgGain=imgGain;
	  cstr->subapImgGainArr=NULL;
	}else if(nbytes[IMGGAIN]==sizeof(float)*cstr->nsubaps){
	  if(cstr->subapImgGainArr==NULL)
	    resetCalImg=1;
	  cstr->subapImgGainArr=(float*)values[IMGGAIN];
	  cstr->subapImgGain=1.;
	}else{
	  printf("imgGain error\n");
	  err=1;
	}
	if(err==0 &&  resetCalImg==1){//reset the integrator.
	  if(cstr->integratedImgSize<cstr->totPxls){
	    if(cstr->integratedImg!=NULL)
	      free(cstr->integratedImg);
	    if((cstr->integratedImg=malloc(sizeof(float)*cstr->totPxls))==NULL){
	      printf("Failed to malloc memory for integrating pixels\n");
	      cstr->subapImgGain=1.;
	      cstr->subapImgGainArr=NULL;
	      cstr->integratedImgSize=0;
	    }else
	      cstr->integratedImgSize=cstr->totPxls;
	  }
	  if(cstr->integratedImg!=NULL)
	    memset(cstr->integratedImg,0,sizeof(float)*cstr->totPxls);
	}
      }else{
	printf("imgGain error\n");
	err=1;
      }
    }else{
      printf("imgGain not found - continuing\n");
      cstr->subapImgGain=1.;
      cstr->subapImgGainArr=NULL;
    }
    cstr->pxlMap=NULL;
    if(index[PXLMAP]>=0){
      if(nbytes[PXLMAP]>=sizeof(int)*cstr->nsubaps){
	cstr->pxlMap=(int*)values[PXLMAP];
      }
    }
    cstr->addSubapToCalBuf=0;
    if(index[ADDSUBAPTOCAL]>=0){
      if(nbytes[ADDSUBAPTOCAL]==sizeof(int) && dtype[ADDSUBAPTOCAL]=='i'){
	cstr->addSubapToCalBuf=*(int*)values[ADDSUBAPTOCAL];
      }else{
	printf("addSubapToCal ignored\n");
      }
    }
#ifdef WITHSIM
    cstr->simAllImg=0;
    if(index[SIMALLIMG]>=0){
      if(dtype[SIMALLIMG]=='i' && nbytes[SIMALLIMG]==sizeof(int)){
	cstr->simAllImg=*(int*)values[SIMALLIMG];
      }else{
	printf("simAllImg error\n");
	err=1;
      }
    }
    cstr->simnpxlx=NULL;
    if(index[SIMNPXLX]>=0){
      if(dtype[SIMNPXLX]=='i' && nbytes[SIMNPXLX]==sizeof(int)*cstr->ncam){
	cstr->simnpxlx=(int*)values[SIMNPXLX];
      }else{
	printf("simNpxlx error\n");
	err=1;
      }
    }else{
      printf("Using npxlx for simnpxlx\n");
      cstr->simnpxlx=cstr->npxlx;
    }
    cstr->simnpxlcum=NULL;
    if(index[SIMNPXLCUM]>=0){
      if(dtype[SIMNPXLCUM]=='i' && nbytes[SIMNPXLCUM]==sizeof(int)*(cstr->ncam+1)){
	cstr->simnpxlcum=(int*)values[SIMNPXLCUM];
      }else{
	printf("simNpxlcum error\n");
	err=1;
      }
    }else{
      printf("Using npxlCum for simnpxlcum\n");
      cstr->simnpxlcum=cstr->npxlCum;
    }
    cstr->simnstore=2;
    if(index[SIMNSTORE]>=0){
      if(dtype[SIMNSTORE]=='i' && nbytes[SIMNSTORE]==sizeof(int)){
	cstr->simnstore=*(int*)values[SIMNSTORE];
      }else{
	printf("simNstore error\n");
	err=1;
      }
    }
    cstr->simFFTPattern=NULL;
    if(index[SIMFFTPATTERN]>=0 && cstr->simnpxlcum!=NULL){
      if(dtype[SIMFFTPATTERN]=='f' && nbytes[SIMFFTPATTERN]==sizeof(float)*cstr->simnpxlcum[cstr->ncam]){
	cstr->simFFTPattern=(float*)values[SIMFFTPATTERN];
      }else{
	printf("simFFTPattern error\n");
	err=1;
      }
    }
    cstr->simCorrThreshType=0;
    cstr->simCorrThreshTypeArr=NULL;
    if(index[SIMCORRTHRTYPE]>=0){
      if(dtype[SIMCORRTHRTYPE]=='i'){
	if(nbytes[SIMCORRTHRTYPE]==sizeof(int))
	  cstr->simCorrThreshType=*(int*)values[SIMCORRTHRTYPE];
	else if(nbytes[SIMCORRTHRTYPE]==sizeof(int)*cstr->nsubaps)
	  cstr->simCorrThreshTypeArr=(int*)values[SIMCORRTHRTYPE];
	else{
	  printf("simCorrThrType error\n");
	  err=1;
	}
      }else{
	  printf("simCorrThrType error\n");
	  err=1;
      }
    }
    cstr->simCorrThresh=0;
    cstr->simCorrThreshArr=NULL;
    if(index[SIMCORRTHRESH]>=0){
      if(dtype[SIMCORRTHRESH]=='f'){
	if(nbytes[SIMCORRTHRESH]==sizeof(float))
	  cstr->simCorrThresh=*(float*)values[SIMCORRTHRESH];
	else if(nbytes[SIMCORRTHRESH]==sizeof(float)*cstr->nsubaps)
	  cstr->simCorrThreshArr=(float*)values[SIMCORRTHRESH];
	else{
	  printf("simCorrThresh error\n");
	  err=1;
	}
      }else{
	  printf("simCorrThresh error\n");
	  err=1;
      }
    }
    cstr->simMode=0;
    cstr->simModeArr=NULL;
    if(index[SIMMODE]>=0){
      if(dtype[SIMMODE]=='i'){
	if(nbytes[SIMMODE]==sizeof(int))
	  cstr->simMode=*(int*)values[SIMMODE];
	else if(nbytes[SIMMODE]==sizeof(int)*cstr->nsubaps)
	  cstr->simModeArr=(int*)values[SIMMODE];
	else{
	  printf("simMode error\n");
	  err=1;
	}
      }else{
	printf("simMode error\n");
	err=1;
      }
    }
    cstr->simOffset=0;
    cstr->simOffsetArr=NULL;
    if(index[SIMOFFSET]>=0){
      if(dtype[SIMOFFSET]=='f'){
	if(nbytes[SIMOFFSET]==sizeof(float))
	  cstr->simOffset=*(float*)values[SIMOFFSET];
	else if(nbytes[SIMOFFSET]==sizeof(float)*cstr->nsubaps)
	  cstr->simOffsetArr=(float*)values[SIMOFFSET];
	else{
	  printf("simOffset error\n");
	  err=1;
	}
      }else{
	  printf("simOffset error\n");
	  err=1;
      }
    }
    cstr->simPoisson=0;
    cstr->simPoissonArr=NULL;
    if(index[SIMPOISSON]>=0){
      if(dtype[SIMPOISSON]=='i'){
	if(nbytes[SIMPOISSON]==sizeof(int))
	  cstr->simPoisson=*(int*)values[SIMPOISSON];
	else if(nbytes[SIMPOISSON]==sizeof(int)*cstr->nsubaps)
	  cstr->simPoissonArr=(int*)values[SIMPOISSON];
	else{
	  printf("simPoisson error\n");
	  err=1;
	}
      }else{
	  printf("simPoisson error\n");
	  err=1;
      }
    }
    cstr->simScale=1;
    cstr->simScaleArr=NULL;
    if(index[SIMSCALE]>=0){
      if(dtype[SIMSCALE]=='f'){
	if(nbytes[SIMSCALE]==sizeof(float))
	  cstr->simScale=*(float*)values[SIMSCALE];
	else if(nbytes[SIMSCALE]==sizeof(float)*cstr->nsubaps)
	  cstr->simScaleArr=(float*)values[SIMSCALE];
	else{
	  printf("simScale error\n");
	  err=1;
	}
      }else{
	  printf("simScale error\n");
	  err=1;
      }
    }
    cstr->simSig=1000;
    cstr->simSigArr=NULL;
    if(index[SIMSIG]>=0){
      if(dtype[SIMSIG]=='f'){
	if(nbytes[SIMSIG]==sizeof(float))
	  cstr->simSig=*(float*)values[SIMSIG];
	else if(nbytes[SIMSIG]==sizeof(float)*cstr->nsubaps)
	  cstr->simSigArr=(float*)values[SIMSIG];
	else{
	  printf("simSig error\n");
	  err=1;
	}
      }else{
	  printf("simSig error\n");
	  err=1;
      }
    }
    cstr->simReadnoise=0;
    cstr->simReadnoiseArr=NULL;
    if(index[SIMREADNOISE]>=0){
      if(dtype[SIMREADNOISE]=='f'){
	if(nbytes[SIMREADNOISE]==sizeof(float))
	  cstr->simReadnoise=*(float*)values[SIMREADNOISE];
	else if(nbytes[SIMREADNOISE]==sizeof(float)*cstr->nsubaps)
	  cstr->simReadnoiseArr=(float*)values[SIMREADNOISE];
	else{
	  printf("simReadnoise error\n");
	  err=1;
	}
      }else{
	  printf("simReadnoise error\n");
	  err=1;
      }
    }

    cstr->simSkybg=0;
    cstr->simSkybgArr=NULL;
    if(index[SIMSKYBG]>=0){
      if(dtype[SIMSKYBG]=='f'){
	if(nbytes[SIMSKYBG]==sizeof(float))
	  cstr->simSkybg=*(float*)values[SIMSKYBG];
	else if(nbytes[SIMSKYBG]==sizeof(float)*cstr->nsubaps)
	  cstr->simSkybgArr=(float*)values[SIMSKYBG];
	else{
	  printf("simSkybg error\n");
	  err=1;
	}
      }else{
	  printf("simSkybg error\n");
	  err=1;
      }
    }


    cstr->simSubapLoc=NULL;
    if(index[SIMSUBAPLOC]>=0){
      if(dtype[SIMSUBAPLOC]=='i' && nbytes[SIMSUBAPLOC]==sizeof(int)*cstr->nsubaps*6){
	cstr->simSubapLoc=(int*)values[SIMSUBAPLOC];
      }else{
	printf("simSubapLoc error\n");
	err=1;
      }
    }else{
      printf("Assuming default subapLocation for simulation correlation images\n");
      if(dtype[SUBAPLOCATION]=='i' && nbytes[SUBAPLOCATION]==sizeof(int)*6*cstr->nsubaps)
	cstr->simSubapLoc=(int*)values[SUBAPLOCATION];
      else{
	cstr->simSubapLoc=NULL;
	printf("subapLocation error\n");
	err=1;
      }
    }

    if(index[SIMCALMULT]<0 || nbytes[SIMCALMULT]==0)
      cstr->simcalmult=NULL;
    else if(dtype[SIMCALMULT]=='f' && nbytes[SIMCALMULT]==sizeof(float)*cstr->totPxls)
      cstr->simcalmult=(float*)values[SIMCALMULT];
    else{
      cstr->simcalmult=NULL;
      printf("simcalmult error\n");
      err=1;
    }
    if(index[SIMCALSUB]<0 || nbytes[SIMCALSUB]==0)
      cstr->simcalsub=NULL;
    else if(dtype[SIMCALSUB]=='f' && nbytes[SIMCALSUB]==sizeof(float)*cstr->totPxls)
      cstr->simcalsub=(float*)values[SIMCALSUB];
    else{
      cstr->simcalsub=NULL;
      printf("simcalsub error\n");
      err=1;
    }
    if(index[SIMCALTHR]<0 || nbytes[SIMCALTHR]==0)
      cstr->simcalthr=NULL;
    else if(dtype[SIMCALTHR]=='f' && nbytes[SIMCALTHR]==sizeof(float)*cstr->totPxls)
      cstr->simcalthr=(float*)values[SIMCALTHR];
    else{
      cstr->simcalthr=NULL;
      printf("simcalthr error\n");
      err=1;
    }
    cstr->simthresholdAlgo=0;
    if(index[SIMTHRESHALGO]>=0){
      if(dtype[SIMTHRESHALGO]=='i' && nbytes[THRESHOLDALGO]==sizeof(int)){
	cstr->simthresholdAlgo=*((int*)values[THRESHOLDALGO]);
      }else{
	printf("simthresholdAlgo error\n");
	err=1;
      }
    }
    cstr->simuseBrightest=0;
    cstr->simuseBrightestArr=NULL;
    if(index[SIMUSEBRIGHT]>=0){
      if(dtype[SIMUSEBRIGHT]=='i'){
	if(nbytes[SIMUSEBRIGHT]==sizeof(int)){
	  cstr->simuseBrightest=*((int*)values[SIMUSEBRIGHT]);
	  cstr->simuseBrightestArr=NULL;
	}else if(nbytes[SIMUSEBRIGHT]==sizeof(int)*cstr->nsubaps){
	  cstr->simuseBrightest=0;
	  cstr->simuseBrightestArr=(int*)values[SIMUSEBRIGHT];
	}else{
	  cstr->simuseBrightest=0;
	  cstr->simuseBrightestArr=NULL;
	  printf("simuseBrightest error\n");
	  err=1;
	}
      }else{
	printf("simuseBrightest error\n");
	err=1;
      }
    }
    if(index[SIMUBTHRAV]>=0){
      if(dtype[SIMUBTHRAV]=='i'){
	if(nbytes[SIMUBTHRAV]==sizeof(int)){
	  cstr->simuseBrightAv=*((int*)values[SIMUBTHRAV]);
	  cstr->simuseBrightAvArr=NULL;
	}else if(nbytes[SIMUBTHRAV]==sizeof(int)*cstr->nsubaps){
	  cstr->simuseBrightAv=0;
	  cstr->simuseBrightAvArr=(int*)values[SIMUBTHRAV];
	}else{
	  cstr->simuseBrightAv=0;
	  cstr->simuseBrightAvArr=NULL;
	  printf("simUBThrAv error\n");
	  err=1;
	}
      }else{
	printf("simUBThrAv error\n");
	err=1;
      }
    }else{
      cstr->useBrightAv=0;
      cstr->useBrightAvArr=NULL;
    }
    if(cstr->simPxlBuf==NULL){
      if((cstr->simPxlBuf=malloc(sizeof(float)*cstr->totPxls))==NULL){
	printf("Error - failed to malloc simPxlBuf\n");
	err=1;
      }else
	memset(cstr->simPxlBuf,0,sizeof(float)*cstr->totPxls);
    }
#endif




  }
  return err;
}
int calibrateOpen(char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **calibrateHandle,int nthreads,unsigned int frameno,unsigned int **calibrateframeno,int *calibrateframenosize){
  CalStruct *cstr;
  int err;
  char *pn;
  int i;
#ifdef WITHSIM
  char *tmp;
#endif
  printf("Opening rtccalibrate\n");
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
  cstr->arr=arr;
  cstr->prefix=prefix;
  cstr->subapImgGain=1.;
  //cstr->calpxlbufReady=1;
  //pthread_mutex_init(&cstr->calmutex,NULL);
  //pthread_cond_init(&cstr->calcond,NULL);
  //threadInfo->globals->reconStruct=(void*)reconStruct;
  *calibrateHandle=(void*)cstr;
  //cstr->buf=1;
  cstr->nthreads=nthreads;//this doesn't change.
  if((cstr->tstr=calloc(sizeof(CalThreadStruct*),nthreads))==NULL){
    printf("Error allocating CalThreadStruct\n");
    free(cstr);
    *calibrateHandle=NULL;
    return 1;
  }
  for(i=0; i<nthreads;i++){
    if((cstr->tstr[i]=malloc(sizeof(CalThreadStruct)))==NULL){
      printf("Error allocating CalThreadStruct %d\n",i);
      i--;
      while(i>=0){//free ones that have been allocated
	free(cstr->tstr[i]);
	i--;
      }
      free(cstr->tstr);
      free(cstr);
      *calibrateHandle=NULL;
      return 1;
    }else{
      memset(cstr->tstr[i],0,sizeof(CalThreadStruct));
    }
  }
  cstr->rtcErrorBuf=rtcErrorBuf;
#ifdef WITHSIM
  for(i=0;i<nthreads;i++){
    cstr->tstr[i]->gslRand=gsl_rng_alloc(gsl_rng_mt19937);
    gsl_rng_set(cstr->tstr[i]->gslRand,(unsigned long)time(0)+i);
  }
  pthread_mutex_init(&cstr->fftcreateMutex,NULL);

#endif




  err=calibrateNewParam(*calibrateHandle,pbuf,frameno,arr);//this will change ->buf to 0.
  if(err!=0){
    printf("Error in calibrateOpen...\n");
    calibrateClose(calibrateHandle);
    *calibrateHandle=NULL;
    return 1;
  }

#ifdef WITHSIM
  if(asprintf(&tmp,"/%srtcSimRawBuf",cstr->prefix)==-1){
    printf("Error asprintf in rtccalibrate - exiting\n");
    exit(1);
  }
  cstr->rtcSimRawBuf=openCircBuf(tmp,1,&cstr->totPxls,cstr->arr->pxlbuftype,cstr->simnstore);
  free(tmp);
  if(asprintf(&tmp,"/%srtcSimPxlBuf",cstr->prefix)==-1){
    printf("Error asprintf in rtccalibrate: exiting\n");
    exit(1);
  }
  cstr->rtcSimPxlBuf=openCircBuf(tmp,1,&cstr->totPxls,'f',cstr->simnstore);
  free(tmp);


#endif

  return 0;
}
/*
int calibrateNewFrame(void *calibrateHandle,unsigned int frameno){//#non-subap thread (once)
  //Should do any finalisation, if it is needed, and has not been done by calibrateNewParam (if a buffer switch has been done).
  //At this point, we know it is safe to use the calpxlbuf again (but we mustn't before this point).
  CalStruct *cstr=(CalStruct*)calibrateHandle;
  if(cstr->finalise){
    cstr->finalise=0;
    //do anything to finalise previous frame.
  }
  return 0;
  }*/
//Uncomment if needed.
//int calibrateStartFrame(void *calibrateHandle,int cam,int threadno){//subap thread (once per thread)
//}
#ifndef OLDMULTINEWFN //this is now the usual case.
int calibrateNewSubap(void *calibrateHandle,int cam,int threadno,int cursubindx,float **subap,int *subapSize,int *NProcessing,int *rubbish){//subap thread
  CalStruct *cstr=(CalStruct*)calibrateHandle;
  CalThreadStruct *tstr=cstr->tstr[threadno];
  int nprocessing=*NProcessing;
  int i;
  int *loc;
  float *tmp;
  int curnpxly,curnpxlx,curnpxl,size,max,pos;
  //calibrating all subaps at once - first work out how much space we need.
  tstr->subapHandle=subap;
  tstr->subapSizeHandle=subapSize;
  size=0;
  max=0;
  if(tstr->nprocSize<nprocessing){
    if((loc=malloc(sizeof(int)*nprocessing*3))==NULL){
      printf("Unable to malloc nproc array\n");
      return 1;
    }
    if(tstr->nproc!=NULL)free(tstr->nproc);
    tstr->nproc=loc;
    tstr->nprocSize=nprocessing;
  }

  for(i=0;i<nprocessing;i++){
    if(cstr->subapFlagArr[cursubindx+i]==1){
      loc=&(cstr->arr->subapLocation[(cursubindx+i)*6]);
      curnpxly=(loc[1]-loc[0])/loc[2];
      curnpxlx=(loc[4]-loc[3])/loc[5];
      curnpxl=curnpxly*curnpxlx;
      tstr->nproc[i*3]=curnpxly;
      tstr->nproc[i*3+1]=curnpxlx;
      tstr->nproc[i*3+2]=curnpxl;
      if(curnpxl*3+curnpxlx>max)//the *3+curnpxlx is required for the tvm algorithm, but not for the sorting.  Added with the tvm implementation.
	max=curnpxl*3+curnpxlx;//this is needed for the sort array (useBrightest).
      //Also, want the subap array to be 16 byte aligned.  Note, should make this 64 byte for future processors/xeon Phi.  Ok - now 64 bit aligned.
      size+=(((curnpxl+(SUBAPALIGN/sizeof(float))-1)/(SUBAPALIGN/sizeof(float))))*(SUBAPALIGN/sizeof(float));
    }
  }
  //Now allocate memory if needed.
  if(*subapSize<size){
    if((i=posix_memalign((void**)(&tmp),SUBAPALIGN,sizeof(float)*size))!=0){
      tmp=NULL;
      printf("subap re-malloc failed for thread %d, size %d\n",threadno,size);
      return 1;
    }
    if(*subap!=NULL)
      free(*subap);
    *subap=tmp;
    *subapSize=size;
    tstr->subapSize=size;
  }
  //and allocate the sort array if needed.
  if(tstr->sortSize<max){
    if((tmp=malloc(sizeof(float)*max))==NULL){
      printf("sort remalloc failed for thread %d size %d\n",threadno,max);
      return 1;
    }
    if(tstr->sort!=NULL)free(tstr->sort);
    tstr->sort=tmp;
    tstr->sortSize=max;
  }
  //Now we have the temporary array, calibrate into it.
  pos=0;
  for(i=0;i<nprocessing;i++){
    if(cstr->subapFlagArr[cursubindx]==1){
      tstr->subap=&((*subap)[pos]);
      tstr->cursubindx=cursubindx;
      tstr->curnpxly=tstr->nproc[i*3];
      tstr->curnpxlx=tstr->nproc[i*3+1];
      tstr->curnpxl=tstr->nproc[i*3+2];
      pos+=((tstr->curnpxl+(SUBAPALIGN/sizeof(float))-1)/(SUBAPALIGN/sizeof(float)))*(SUBAPALIGN/sizeof(float));
      copySubap(cstr,cam,threadno);
#ifdef WITHSIM
      simulateSubap(cstr,cam,threadno);
#endif
      subapPxlCalibration(cstr,cam,threadno);
      if(cstr->arr->rtcCalPxlBuf->addRequired || cstr->subapImgGain!=1. || cstr->subapImgGainArr!=NULL)
	storeCalibratedSubap(cstr,cam,threadno);
    }
    cursubindx++;
  }
  return 0;
}


#else
int calibrateNewSubap(void *calibrateHandle,int cam,int threadno,int cursubindx,float **subap,int *subapSize,int *curnpxlx,int *curnpxly){//subap thread
  CalStruct *cstr=(CalStruct*)calibrateHandle;
  CalThreadStruct *tstr=cstr->tstr[threadno];
  tstr->cursubindx=cursubindx;
  copySubap(cstr,cam,threadno);
  subapPxlCalibration(cstr,cam,threadno);
  //Note - this function isn't called until the circular buffer calpxlbuf has been written and cleared - so we can start writing to it again.
  //But - what is the point of doing this if the buffer will not be read?  And if we do, it will slow us down because of cache coherency... so, we should only write if actually needed.
  if(cstr->arr->rtcCalPxlBuf->addRequired || cstr->subapImgGain!=1. || cstr->subapImgGainArr!=NULL)
    storeCalibratedSubap(cstr,cam,threadno);
  *subap=tstr->subap;
  *subapSize=tstr->subapSize;
  *curnpxlx=tstr->curnpxlx;
  *curnpxly=tstr->curnpxly;
  //cstr->finalise=1;
  return 0;
}
#endif


//uncomment if needed
/*
int calibrateEndFrame(void *calibrateHandle,int cam,int threadno,int err){//subap thread (once per thread)
}
*/

#ifdef WITHSIM


int calibrateNewFrameSync(void *calibrateHandle,unsigned int frameno,double timestamp){
  CalStruct *cstr=(CalStruct*)calibrateHandle;
  CalThreadStruct *tstr=cstr->tstr[0];

  cstr->frameno=frameno;
  cstr->timestamp=timestamp;
  if(cstr->simAllImg){//want all image to be simulated not just used subaps
    //For each camera, use the readnoise, etc specified for the first subap of that camera.
    int cursub=0,npxl,poisson;
    float offset,scale;
    int i,cam;
    double skybg,readnoise;
    float *simbuf;
    for(cam=0;cam<cstr->ncam;cam++){
      simbuf=&(cstr->simPxlBuf[cstr->npxlCum[cam]]);
      npxl=cstr->npxlx[cam]*cstr->npxly[cam];
      if(cstr->simPoissonArr!=NULL)
	poisson=cstr->simPoissonArr[cursub];
      else
	poisson=cstr->simPoisson;
      if(cstr->simSkybgArr!=NULL)
	skybg=(double)cstr->simSkybgArr[cursub];
      else
	skybg=(double)cstr->simSkybg;

      readnoise=(double)(cstr->simReadnoiseArr==NULL?cstr->simReadnoise:cstr->simReadnoiseArr[cursub]);
      offset=cstr->simOffsetArr==NULL?cstr->simOffset:cstr->simOffsetArr[cursub];
      scale=(cstr->simScaleArr==NULL?cstr->simScale:cstr->simScaleArr[cursub]);
      if(poisson && skybg!=0){
	for(i=0;i<npxl;i++)
	  simbuf[i]=(float)gsl_ran_poisson(tstr->gslRand,skybg);
      }else{
	for(i=0;i<npxl;i++)
	  simbuf[i]=skybg;
      }
      if(readnoise!=0){
	for(i=0;i<npxl;i++)
	  simbuf[i]+=gsl_ran_gaussian(tstr->gslRand,readnoise);
      }
      if(offset){
	for(i=0;i<npxl;i++)
	  simbuf[i]+=offset;
      }
      if(scale!=1){
	for(i=0;i<npxl;i++)
	  simbuf[i]*=scale;
      }
      cursub+=cstr->nsub[cam];
    }
  }else
    memset(cstr->simPxlBuf,0,sizeof(float)*cstr->totPxls);
  return 0;
}
int calibrateFrameFinishedSync(void *calibrateHandle,int err,int forcewrite){//subap thread (once)
  //At this stage, the raw pixel buf should have all pixels in it.
  //i.e arr->pxlbufs.
  //So, we copy this into rtcSimRawBuf (so as to have a record) with circAdd.
  //Then we copy simPxlBuf into arr->pxlbufs, but only the subaps we've simulated.
  //Then we also circAdd simPxlBuf to rtcSimPxlBuf so we can see what it looks like before converting to the type of the camera.
  CalStruct *cstr=(CalStruct*)calibrateHandle;
  int subCum=0;
  unsigned short *Hpxlbuf;//=&cstr->arr->pxlbufs[cstr->npxlCum[cam]];
  short *hpxlbuf;
  float *fpxlbuf;
  int *ipxlbuf;
  char *cpxlbuf;
  unsigned char *Cpxlbuf;
  unsigned int *Ipxlbuf;
  int cam,i,j;
  float *simbuf;
  int sub;
  int *loc;
  int npxlx;
  circAdd(cstr->rtcSimRawBuf,cstr->arr->pxlbufs,cstr->timestamp,cstr->frameno);
  circAdd(cstr->rtcSimPxlBuf,cstr->simPxlBuf,cstr->timestamp,cstr->frameno);
  //Now, copy simPxlBuf into arr->pxlbufs.
  if(cstr->simAllImg){
    //copy all image
    if(cstr->arr->pxlbuftype=='b'){
      for(i=0;i<cstr->totPxls;i++)
	((char*)cstr->arr->pxlbufs)[i]=(char)cstr->simPxlBuf[i];
    }else if(cstr->arr->pxlbuftype=='B'){
      for(i=0;i<cstr->totPxls;i++)
	((unsigned char*)cstr->arr->pxlbufs)[i]=(unsigned char)cstr->simPxlBuf[i];
    }else if(cstr->arr->pxlbuftype=='h'){
      for(i=0;i<cstr->totPxls;i++)
	((short*)cstr->arr->pxlbufs)[i]=(short)cstr->simPxlBuf[i];
    }else if(cstr->arr->pxlbuftype=='H'){
      for(i=0;i<cstr->totPxls;i++)
	((unsigned short*)cstr->arr->pxlbufs)[i]=(unsigned short)cstr->simPxlBuf[i];
    }else if(cstr->arr->pxlbuftype=='i'){
      for(i=0;i<cstr->totPxls;i++)
	((int*)cstr->arr->pxlbufs)[i]=(int)cstr->simPxlBuf[i];
    }else if(cstr->arr->pxlbuftype=='I'){
      for(i=0;i<cstr->totPxls;i++)
	((unsigned int*)cstr->arr->pxlbufs)[i]=(unsigned int)cstr->simPxlBuf[i];
    }else if(cstr->arr->pxlbuftype=='f'){
      memcpy(cstr->arr->pxlbufs,cstr->simPxlBuf,sizeof(float)*cstr->totPxls);
    }
  }else{//copy only used pixels (so real camera data will remain in others)...

    for(cam=0;cam<cstr->ncam;cam++){
      npxlx=cstr->npxlx[cam];
      simbuf=&(cstr->simPxlBuf[cstr->npxlCum[cam]]);
      if(cstr->arr->pxlbuftype=='b'){
	cpxlbuf=&(((char*)cstr->arr->pxlbufs)[cstr->npxlCum[cam]]);
	for(sub=0;sub<cstr->nsub[cam];sub++){
	  if((cstr->simModeArr!=NULL && cstr->simModeArr[sub+subCum]==1) || cstr->simMode==1){
	    loc=&(cstr->arr->subapLocation[(sub+subCum)*6]);
	    for(i=loc[0]; i<loc[1]; i+=loc[2]){
	      for(j=loc[3]; j<loc[4]; j+=loc[5]){
		cpxlbuf[i*npxlx+j]=(char)simbuf[i*npxlx+j];
	      }
	    }
	  }
	}
      }else if(cstr->arr->pxlbuftype=='B'){
	Cpxlbuf=&(((unsigned char*)cstr->arr->pxlbufs)[cstr->npxlCum[cam]]);
	for(sub=0;sub<cstr->nsub[cam];sub++){
	  if((cstr->simModeArr!=NULL && cstr->simModeArr[sub+subCum]==1) || cstr->simMode==1){
	    loc=&(cstr->arr->subapLocation[(sub+subCum)*6]);
	    for(i=loc[0]; i<loc[1]; i+=loc[2]){
	      for(j=loc[3]; j<loc[4]; j+=loc[5]){
		Cpxlbuf[i*npxlx+j]=(unsigned char)simbuf[i*npxlx+j];
	      }
	    }
	  }
	}
      }else if(cstr->arr->pxlbuftype=='h'){
	hpxlbuf=&(((short*)cstr->arr->pxlbufs)[cstr->npxlCum[cam]]);
	for(sub=0;sub<cstr->nsub[cam];sub++){
	  if((cstr->simModeArr!=NULL && cstr->simModeArr[sub+subCum]==1) || cstr->simMode==1){
	    loc=&(cstr->arr->subapLocation[(sub+subCum)*6]);
	    for(i=loc[0]; i<loc[1]; i+=loc[2]){
	      for(j=loc[3]; j<loc[4]; j+=loc[5]){
		hpxlbuf[i*npxlx+j]=(short)simbuf[i*npxlx+j];
	      }
	    }
	  }
	}
      }else if(cstr->arr->pxlbuftype=='H'){
	Hpxlbuf=&(((unsigned short*)cstr->arr->pxlbufs)[cstr->npxlCum[cam]]);
	for(sub=0;sub<cstr->nsub[cam];sub++){
	  if((cstr->simModeArr!=NULL && cstr->simModeArr[sub+subCum]==1) || cstr->simMode==1){
	    loc=&(cstr->arr->subapLocation[(sub+subCum)*6]);
	    for(i=loc[0]; i<loc[1]; i+=loc[2]){
	      for(j=loc[3]; j<loc[4]; j+=loc[5]){
		Hpxlbuf[i*npxlx+j]=(unsigned short)simbuf[i*npxlx+j];
	      }
	    }
	  }
	}
      }else if(cstr->arr->pxlbuftype=='i'){
	ipxlbuf=&(((int*)cstr->arr->pxlbufs)[cstr->npxlCum[cam]]);
	for(sub=0;sub<cstr->nsub[cam];sub++){
	  if((cstr->simModeArr!=NULL && cstr->simModeArr[sub+subCum]==1) || cstr->simMode==1){
	    loc=&(cstr->arr->subapLocation[(sub+subCum)*6]);
	    for(i=loc[0]; i<loc[1]; i+=loc[2]){
	      for(j=loc[3]; j<loc[4]; j+=loc[5]){
		ipxlbuf[i*npxlx+j]=(int)simbuf[i*npxlx+j];
	      }
	    }
	  }
	}
      }else if(cstr->arr->pxlbuftype=='I'){
	Ipxlbuf=&(((unsigned int*)cstr->arr->pxlbufs)[cstr->npxlCum[cam]]);
	for(sub=0;sub<cstr->nsub[cam];sub++){
	  if((cstr->simModeArr!=NULL && cstr->simModeArr[sub+subCum]==1) || cstr->simMode==1){
	    loc=&(cstr->arr->subapLocation[(sub+subCum)*6]);
	    for(i=loc[0]; i<loc[1]; i+=loc[2]){
	      for(j=loc[3]; j<loc[4]; j+=loc[5]){
		Ipxlbuf[i*npxlx+j]=(unsigned int)simbuf[i*npxlx+j];
	      }
	    }
	  }
	}
      }else if(cstr->arr->pxlbuftype=='f'){
	fpxlbuf=&(((float*)cstr->arr->pxlbufs)[cstr->npxlCum[cam]]);
	for(sub=0;sub<cstr->nsub[cam];sub++){
	  if((cstr->simModeArr!=NULL && cstr->simModeArr[sub+subCum]==1) || cstr->simMode==1){
	    loc=&(cstr->arr->subapLocation[(sub+subCum)*6]);
	    for(i=loc[0]; i<loc[1]; i+=loc[2]){
	      for(j=loc[3]; j<loc[4]; j+=loc[5]){
		fpxlbuf[i*npxlx+j]=(float)simbuf[i*npxlx+j];
	      }
	    }
	  }
	}
      }
      subCum+=cstr->nsub[cam];
    }
  }
  return 0;


}

#endif

/*
int calibrateFrameFinished(void *calibrateHandle,int err){//non-subap thread (once)
}
int calibrateOpenLoop(void *calibrateHandle){

}
*/
