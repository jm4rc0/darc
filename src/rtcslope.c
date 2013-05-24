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
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fftw3.h>
#include "darc.h"
#include "rtcslope.h"
enum CentroidModes{CENTROIDMODE_COG,CENTROIDMODE_CORRELATIONCOG,CENTROIDMODE_ERROR};
enum CorrelationThresholdType{CORR_ABS_SUB,CORR_ABS_ZERO,CORR_FRAC_SUB,CORR_FRAC_ZERO};


typedef enum{
  ADAPRESETCOUNT,
  ADAPTIVEGROUP,
  ADAPTIVEWINGAIN,
  CENTCALBOUNDS,
  CENTCALDATA,
  CENTCALSTEPS,
  CENTINDEXARRAY,
  CENTROIDMODE,
  CENTROIDWEIGHT,
  CORRCLIP,
  CORRFFTPATTERN,
  CORRNSTORE,
  CORRNPXLCUM,
  CORRNPXLX,
  CORRSUBAPLOCATION,
  CORRTHRESH,
  CORRTHRESHTYPE,
  FLUXTHRESHOLD,
  MAXADAPOFFSET,
  NCAM,
  NPXLX,
  NPXLY,
  NSUB,
  REFCENTROIDS,
  SUBAPFLAG,
  SUBAPLOCATION,
  WINDOWMODE,
  NBUFFERVARIABLES
}calibrateNames;

//char calibrateParamList[NBUFFERVARIABLES][16]={
#define makeParamNames() bufferMakeNames(NBUFFERVARIABLES,	\
					 "adapResetCount",	\
					 "adaptiveGroup",	\
					 "adaptiveWinGain",	\
					 "centCalBounds",	\
					 "centCalData",		\
					 "centCalSteps",	\
					 "centIndexArray",	\
					 "centroidMode",	\
					 "centroidWeight",	\
					 "corrClip",		\
					 "corrFFTPattern",	\
					 "corrNStore",		\
					 "corrNpxlCum",		\
					 "corrNpxlx",		\
					 "corrSubapLoc",	\
					 "corrThresh",		\
					 "corrThreshType",	\
					 "fluxThreshold",	\
					 "maxAdapOffset",	\
					 "ncam",		\
					 "npxlx",		\
					 "npxly",		\
					 "nsub",		\
					 "refCentroids",	\
					 "subapFlag",		\
					 "subapLocation",	\
					 "windowMode"		\
					 )



typedef struct{
  float *subap;
  //int subapSize;
  int curnpxlx;
  int curnpxly;
  int curnpxl;
  int subindx;
  int centindx;
  int cam;
  float *corrSubap;
  int corrSubapSize;
  int corrnpxlx;
  int corrnpxly;
  int curnpxlSubap;
}CentThreadStruct;
typedef struct{
  enum CentroidModes centroidMode;
  int *centroidModeArr;
  double timestamp;
  unsigned int frameno;
  int totPxls;
  circBuf *rtcCorrBuf;
  float *corrbuf;
  circBuf *rtcCalCorrBuf;
  float *calcorrbuf;
}CentPostStruct;

typedef struct{
  CentThreadStruct **tstr;
  float *adaptiveWinPos;
  int adaptiveWinPosSize;
  int *adaptiveCentPos;
  int adaptiveCentPosSize;
  float adaptiveWinGain;
  int maxAdapOffset;
  int *maxAdapOffsetArr;
  int adapResetCount;
  int *adapResetCountArr;
  int *adaptiveMaxCount;
  int adaptiveMaxCountSize;
    
  arrayStruct *arr;
  int ncam;
  int *nsub;
  int totCents;
  int nsubaps;
  int totPxls;
  int *nsubapCum;
  int *subapFlag;
  int *adaptiveGroup;
  float *groupSumX;
  float *groupSumY;
  int *groupSum;
  int nAdaptiveGroups;
  int adaptiveGroupSize;
  float *fftCorrelationPattern;//the spot PSF array, FFT'd in HC format, and placed as per subapLocation...
  int fftCorrPatternSize;
  int fftIndexSize;
  int *fftIndex;
  fftwf_plan *fftPlanArray;//array holding all the fftw plans
  pthread_mutex_t fftcreateMutex;
  int correlationThresholdType;
  float correlationThreshold;
  int corrClip;
  int *corrClipArr;
  float *centCalData;
  float *centCalSteps;
  int *centCalBounds;
  int centCalnsteps;
  float *refCents;
  int *realSubapLocation;
  int *corrSubapLocation;
  int *npxlCum;
  int *npxlx;
  int *npxly;
  int *corrnpxlCum;
  int *corrnpxlx;
  char *paramNames;
  int nthreads;
  unsigned int frameno;//rtc value
  double timestamp;
  //unsigned int *centframeno;//can be updated if want to inform the rtc of our frameno (e.g. frame numbers from WPU).   Of size ncam.
  circBuf *rtcErrorBuf;
  float *centWeighting;
  float fluxThreshold;
  float *fluxThresholdArr;
  enum WindowModes windowMode;
  enum CentroidModes centroidMode;
  int *centroidModeArr;
  char *prefix;
  int corrNStore;
  circBuf *rtcCorrBuf;
  float *corrbuf;
  circBuf *rtcCalCorrBuf;
  float *calcorrbuf;
  int corrbufSize;
  float *centIndexArr;
  int centIndexSize;//1-4 if centroidIndexArr!=NULL.
  CentPostStruct post;
  int index[NBUFFERVARIABLES];
  void *values[NBUFFERVARIABLES];
  char dtype[NBUFFERVARIABLES];
  int nbytes[NBUFFERVARIABLES];
}CentStruct;

/**
   Does nothing
*/
int applyCentWeighting(CentStruct *cstr,int threadno){
  printf("centWeighting does nothing\n");
  return 0;
}

/**
   Calculates the adaptive windows for next time, and updates the current centroids to take into account the position of the current window.
*/
int calcAdaptiveWindow(CentStruct *cstr,int threadno,float cx,float cy){
  CentThreadStruct *tstr=cstr->tstr[threadno];
  float *adaptiveWinPos=cstr->adaptiveWinPos;
  int *adaptiveCentPos=cstr->adaptiveCentPos;
  int centindx=tstr->centindx;
  float adaptiveWinGain=cstr->adaptiveWinGain;
  int* adaptiveMaxCount=cstr->adaptiveMaxCount;
  int maxAdapOffset;
  int adapResetCount;
  //Now use these values to calculate the window position for next time.
  adaptiveWinPos[centindx]*=1-adaptiveWinGain;
  adaptiveWinPos[centindx+1]*=1-adaptiveWinGain;
  adaptiveWinPos[centindx]+=cx*adaptiveWinGain;
  adaptiveWinPos[centindx+1]+=cy*adaptiveWinGain;
  adaptiveCentPos[centindx]=(int)roundf(adaptiveWinPos[centindx]);
  adaptiveCentPos[centindx+1]=(int)roundf(adaptiveWinPos[centindx+1]);
  if(cstr->maxAdapOffsetArr!=NULL)
    maxAdapOffset=cstr->maxAdapOffsetArr[centindx/2];
  else
    maxAdapOffset=cstr->maxAdapOffset;
  if(cstr->adapResetCountArr!=NULL)
    adapResetCount=cstr->adapResetCountArr[centindx/2];
  else
    adapResetCount=cstr->adapResetCount;
  if(maxAdapOffset>0 || cstr->maxAdapOffsetArr!=NULL){
    if(adaptiveCentPos[centindx]>maxAdapOffset){
      adaptiveCentPos[centindx]=maxAdapOffset;
      adaptiveMaxCount[centindx*2]++;
      adaptiveMaxCount[centindx*2+1]=0;
    }else if(adaptiveCentPos[centindx]<-maxAdapOffset){
      adaptiveCentPos[centindx]=-maxAdapOffset;
      adaptiveMaxCount[centindx*2]=0;
      adaptiveMaxCount[centindx*2+1]++;
    }else{
      adaptiveMaxCount[centindx*2]=0;
      adaptiveMaxCount[centindx*2+1]=0;
    }
    if(adaptiveCentPos[centindx+1]>maxAdapOffset){
      adaptiveCentPos[centindx+1]=maxAdapOffset;
      adaptiveMaxCount[centindx*2+2]++;
      adaptiveMaxCount[centindx*2+3]=0;
    }else if(adaptiveCentPos[centindx+1]<-maxAdapOffset){
      adaptiveCentPos[centindx+1]=-maxAdapOffset;
      adaptiveMaxCount[centindx*2+2]=0;
      adaptiveMaxCount[centindx*2+3]++;
    }else{
      adaptiveMaxCount[centindx*2+2]=0;
      adaptiveMaxCount[centindx*2+3]=0;
    }
    if(adapResetCount>0 && maxAdapOffset>0){
      if(adaptiveMaxCount[centindx*2]>adapResetCount || adaptiveMaxCount[centindx*2+1]>adapResetCount || adaptiveMaxCount[centindx*2+2]>adapResetCount || adaptiveMaxCount[centindx*2+3]>adapResetCount){
	//reset the adaptive windows
	//printf("Resetting adaptive window %d\n",centindx/2);
	adaptiveWinPos[centindx]=0.;
	adaptiveWinPos[centindx+1]=0.;
	adaptiveCentPos[centindx]=0;
	adaptiveCentPos[centindx+1]=0;
	adaptiveMaxCount[centindx*2]=0;
	adaptiveMaxCount[centindx*2+1]=0;
	adaptiveMaxCount[centindx*2+2]=0;
	adaptiveMaxCount[centindx*2+3]=0;
      }
    }
  }

 return 0;
}

/**
   Calculates the adaptive windows for next time, and updates the current centroids to take into account the position of the current window.  Here, the adaptive window moves with a global tip-tilt.
   This should be called by a single thread when all centroids have been computed.
*/
int calcGlobalAdaptiveWindow(CentStruct *cstr){
  float sumx=0.,sumy=0.;
  int i,group;
  float *centroids=cstr->arr->centroids;
  int totCents=cstr->totCents;
  float *adaptiveWinPos=cstr->adaptiveWinPos;
  float adaptiveWinGain=cstr->adaptiveWinGain;
  int *adaptiveCentPos=cstr->adaptiveCentPos;
  int maxAdapOffset=cstr->maxAdapOffset;
  int *adaptiveGroup=cstr->adaptiveGroup;
  float *groupSumX=cstr->groupSumX;
  float *groupSumY=cstr->groupSumY;
  int *groupSum=cstr->groupSum;
  int nAdaptiveGroups=cstr->nAdaptiveGroups;
  //compute the sum of x and y centroids.
  int *maxAdapOffsetArr=cstr->maxAdapOffsetArr;
  int* adaptiveMaxCount=cstr->adaptiveMaxCount;
  int adapResetCount=cstr->adapResetCount;
  if(cstr->adaptiveGroup==NULL || cstr->nAdaptiveGroups==1){
    for(i=0; i<totCents; i+=2){
      sumx+=centroids[i];
      sumy+=centroids[i+1];
    }
    sumx/=totCents/2.;
    sumy/=totCents/2.;
    //Now use these values to calculate the window position for next time.
    for(i=0; i<totCents; i+=2){
      adaptiveWinPos[i]*=1-adaptiveWinGain;
      adaptiveWinPos[i+1]*=1-adaptiveWinGain;
      adaptiveWinPos[i]+=sumx*adaptiveWinGain;
      adaptiveWinPos[i+1]+=sumy*adaptiveWinGain;
      adaptiveCentPos[i]=(int)roundf(adaptiveWinPos[i]);
      adaptiveCentPos[i+1]=(int)roundf(adaptiveWinPos[i+1]);
      if(maxAdapOffsetArr!=NULL)
	maxAdapOffset=maxAdapOffsetArr[i/2];
      if(maxAdapOffset>0 || maxAdapOffsetArr!=NULL){
	if(adaptiveCentPos[i]>maxAdapOffset){
	  adaptiveCentPos[i]=maxAdapOffset;
	  adaptiveMaxCount[i*2]++;
	  adaptiveMaxCount[i*2+1]=0;
	}else if(adaptiveCentPos[i]<-maxAdapOffset){
	  adaptiveCentPos[i]=-maxAdapOffset;
	  adaptiveMaxCount[i*2]=0;
	  adaptiveMaxCount[i*2+1]++;
	}else{
	  adaptiveMaxCount[i*2]=0;
	  adaptiveMaxCount[i*2+1]=0;
	}
	if(adaptiveCentPos[i+1]>maxAdapOffset){
	  adaptiveCentPos[i+1]=maxAdapOffset;
	  adaptiveMaxCount[i*2+2]++;
	  adaptiveMaxCount[i*2+3]=0;
	}else if(adaptiveCentPos[i+1]<-maxAdapOffset){
	  adaptiveCentPos[i+1]=-maxAdapOffset;
	  adaptiveMaxCount[i*2+2]=0;
	  adaptiveMaxCount[i*2+3]++;
	}else{
	  adaptiveMaxCount[i*2+2]=0;
	  adaptiveMaxCount[i*2+3]=0;
	}
	if(cstr->adapResetCountArr!=NULL)
	  adapResetCount=cstr->adapResetCountArr[i/2];
	if(adapResetCount>0){
	  if(adaptiveMaxCount[i*2]>adapResetCount || adaptiveMaxCount[i*2+1]>adapResetCount || adaptiveMaxCount[i*2+2]>adapResetCount || adaptiveMaxCount[i*2+3]>adapResetCount){
	    //reset the adaptive windows
	    //printf("Resetting adaptive window %d\n",i/2);
	    adaptiveWinPos[i]=0.;
	    adaptiveWinPos[i+1]=0.;
	    adaptiveCentPos[i]=0;
	    adaptiveCentPos[i+1]=0;
	    adaptiveMaxCount[i*2]=0;
	    adaptiveMaxCount[i*2+1]=0;
	    adaptiveMaxCount[i*2+2]=0;
	    adaptiveMaxCount[i*2+3]=0;
	  }
	}
      }
    }
  }else{
    if(groupSumX!=NULL && groupSumY!=NULL && groupSum!=NULL){
      memset(groupSumX,0,sizeof(float)*nAdaptiveGroups);
      memset(groupSumY,0,sizeof(float)*nAdaptiveGroups);
      memset(groupSum,0,sizeof(int)*nAdaptiveGroups);
      for(i=0; i<totCents; i+=2){
	group=adaptiveGroup[i/2];
	groupSumX[group]+=centroids[i];
	groupSumY[group]+=centroids[i+1];
	groupSum[group]++;
      }
      //now get the mean of each group.
      for(i=0; i<nAdaptiveGroups; i++){
	groupSumX[i]/=groupSum[i];
	groupSumY[i]/=groupSum[i];
      }
      //Now use these values to calculate the window position for next time.
      for(i=0; i<totCents; i+=2){
	group=adaptiveGroup[i/2];
	adaptiveWinPos[i]*=1-adaptiveWinGain;
	adaptiveWinPos[i+1]*=1-adaptiveWinGain;
	adaptiveWinPos[i]+=groupSumX[group]*adaptiveWinGain;
	adaptiveWinPos[i+1]+=groupSumY[group]*adaptiveWinGain;
	adaptiveCentPos[i]=(int)roundf(adaptiveWinPos[i]);
	adaptiveCentPos[i+1]=(int)roundf(adaptiveWinPos[i+1]);
	if(maxAdapOffsetArr!=NULL)
	  maxAdapOffset=maxAdapOffsetArr[i/2];
	if(maxAdapOffset>0 || maxAdapOffsetArr!=NULL){
	  if(adaptiveCentPos[i]>maxAdapOffset){
	    adaptiveCentPos[i]=maxAdapOffset;
	    adaptiveMaxCount[i*2]++;
	    adaptiveMaxCount[i*2+1]=0;
	  }else if(adaptiveCentPos[i]<-maxAdapOffset){
	    adaptiveCentPos[i]=-maxAdapOffset;
	    adaptiveMaxCount[i*2]=0;
	    adaptiveMaxCount[i*2+1]++;
	  }else{
	    adaptiveMaxCount[i*2]=0;
	    adaptiveMaxCount[i*2+1]=0;
	  }
	  if(adaptiveCentPos[i+1]>maxAdapOffset){
	    adaptiveCentPos[i+1]=maxAdapOffset;
	    adaptiveMaxCount[i*2+2]++;
	    adaptiveMaxCount[i*2+3]=0;
	  }else if(adaptiveCentPos[i+1]<-maxAdapOffset){
	    adaptiveCentPos[i+1]=-maxAdapOffset;
	    adaptiveMaxCount[i*2+2]=0;
	    adaptiveMaxCount[i*2+3]++;
	  }else{
	    adaptiveMaxCount[i*2+2]=0;
	    adaptiveMaxCount[i*2+3]=0;
	  }
	  if(cstr->adapResetCountArr!=NULL)
	    adapResetCount=cstr->adapResetCountArr[i/2];
	  if(adapResetCount>0){
	    if(adaptiveMaxCount[i*2]>adapResetCount || adaptiveMaxCount[i*2+1]>adapResetCount || adaptiveMaxCount[i*2+2]>adapResetCount || adaptiveMaxCount[i*2+3]>adapResetCount){
	      //reset the adaptive windows
	      //printf("Resetting adaptive window %d\n",i/2);
	      adaptiveWinPos[i]=0.;
	      adaptiveWinPos[i+1]=0.;
	      adaptiveCentPos[i]=0;
	      adaptiveCentPos[i+1]=0;
	      adaptiveMaxCount[i*2]=0;
	      adaptiveMaxCount[i*2+1]=0;
	      adaptiveMaxCount[i*2+2]=0;
	      adaptiveMaxCount[i*2+3]=0;
	    }
	  }
	}
      }
    }
  }
  return 0;
}


//Define a function to allow easy indexing into the fftCorrelationPattern array...
#define B(y,x) cstr->fftCorrelationPattern[cstr->corrnpxlCum[tstr->cam]+(loc[0]+(y)*loc[2])*cstr->corrnpxlx[tstr->cam]+loc[3]+(x)*loc[5]]
/**
   Calculates the correlation of the spot with the reference.
   fftCorrelationPattern is distributed in memory as per subapLocation, and is
   equal to numpy.conjugate(numpy.fft.fft2(numpy.fft.fftshift(corr)))
   where corr is the reference spot pattern (ie what you are correlating to).
   Should be stored in half complex form (reals then imags)
*/
int calcCorrelation(CentStruct *cstr,int threadno){
  CentThreadStruct *tstr=cstr->tstr[threadno];
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
  int dx,dy,corrnpxlx,corrnpxly,corrClip;
  //This is how the plans should be created (elsewhere).  Will need a different plan for each different sized subap (see subapLocation).  
  //fftwPlan=fftwf_plan_r2r_2d(curnpxly,curnpxlx, double *in, double *out,FFTW_R2HC, FFTW_R2HC, FFTW_ESTIMATE);
  //ifftwPlan=fftwf_plan_r2r_2d(curnpxly,curnpxlx, double *in, double *out,FFTW_HC2R, FFTW_HC2R, FFTW_ESTIMATE);
  if(cstr->corrClipArr!=NULL){
    corrClip=cstr->corrClipArr[tstr->subindx];
  }else
    corrClip=cstr->corrClip;


  if(cstr->corrSubapLocation!=NULL)
    loc=&(cstr->corrSubapLocation[tstr->subindx*6]);
  else
    loc=&(cstr->realSubapLocation[tstr->subindx*6]);
  tstr->corrnpxly=corrnpxly=(loc[1]-loc[0])/loc[2];
  tstr->corrnpxlx=corrnpxlx=(loc[4]-loc[3])/loc[5];

  if(corrnpxlx>curnpxlx || corrnpxly>curnpxly){
    //have to move spot into the middle (ie zero pad it).
    if(tstr->corrSubapSize<corrnpxlx*corrnpxly){
      if(tstr->corrSubap!=NULL){
	printf("Freeing existing corrSubap\n");
	free(tstr->corrSubap);
      }
      tstr->corrSubapSize=corrnpxlx*corrnpxly;
      printf("memaligning corrSubap to %dx%d\n",corrnpxly,corrnpxlx);
      if((i=posix_memalign((void**)(&(tstr->corrSubap)),16,sizeof(float)*tstr->corrSubapSize))!=0){//equivalent to fftwf_malloc... (kernel/kalloc.h in fftw source).
	tstr->corrSubapSize=0;
	tstr->corrSubap=NULL;
	printf("corrSubap re-malloc failed thread %d, size %d\nExiting...\n",threadno,tstr->corrSubapSize);
	exit(0);
      }
      printf("Aligned, address %p thread %d\n",tstr->corrSubap,threadno);
    }
    dx=(corrnpxlx-curnpxlx)/2;
    dy=(corrnpxly-curnpxly)/2;
    memset(tstr->corrSubap,0,sizeof(float)*corrnpxlx*corrnpxly);
    //copy from subap to corrSubap, allowing for the padding.
    //for(i=((loc[1]-loc[0]+loc[2]-1)/loc[2])*loc[2]+loc[0]-loc[2];i>=loc[0];i-=loc[2]){
    for(i=0;i<curnpxly;i++){
      memcpy(&tstr->corrSubap[(dy+i)*corrnpxlx+dx],&tstr->subap[i*curnpxlx],sizeof(float)*curnpxlx);
    }
    subap=tstr->corrSubap;
  }else if(corrnpxlx<curnpxlx || corrnpxly<curnpxly){
    printf("Error - correlation pattern subaperture locations have subapertures smaller than the real ones.  Please correct this.\n");
  }
  

  for(i=0; i<cstr->fftIndexSize; i++){
    if(cstr->fftIndex[i*2]==0 || cstr->fftIndex[i*2+1]==0){
      break;
    }
    if(cstr->fftIndex[i*2]==corrnpxlx && cstr->fftIndex[i*2+1]==corrnpxly){
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
      if(cstr->fftIndex[i*2]==corrnpxlx && cstr->fftIndex[i*2+1]==corrnpxly){
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
      printf("Planning FFTs size %d x %d\n",corrnpxly,corrnpxlx);
      cstr->fftPlanArray[i*2]=fftwf_plan_r2r_2d(corrnpxly,corrnpxlx,subap,subap,FFTW_R2HC, FFTW_R2HC, FFTW_ESTIMATE);
      cstr->fftPlanArray[i*2+1]=fftwf_plan_r2r_2d(corrnpxly,corrnpxlx,subap,subap,FFTW_HC2R, FFTW_HC2R, FFTW_ESTIMATE);
      cstr->fftIndex[i*2]=corrnpxlx;
      cstr->fftIndex[i*2+1]=corrnpxly;
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
  n=corrnpxlx;
  m=corrnpxly;


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
  //Here, if we have oversampled for the FFT, we may need to clip.
  tstr->curnpxlSubap=tstr->curnpxl;
  if((corrnpxlx>curnpxlx || corrnpxly>curnpxly)){
    if(corrClip>0){
      int nx,ny;
      nx=corrnpxlx-2*corrClip;
      ny=corrnpxly-2*corrClip;
      for(i=0;i<ny;i++){//loc[0]+corrClip*loc[2]; i<loc[1]-corrClip*loc[2]; i+=loc[2]){
	memcpy(&subap[i*nx],&subap[(i+corrClip)*corrnpxlx+corrClip],sizeof(float)*nx);
      }
      tstr->curnpxlSubap=(corrnpxlx-2*corrClip)*(corrnpxly-2*corrClip);
    }else
      tstr->curnpxlSubap=corrnpxlx*corrnpxly;
  }
  return 0;
}

#undef B
/**
   Applies the chosen threshold algorithm to the correlation
   There are 4 possible thresholding algorithms.  The threshold is either a fixed value, or a fraction of the maximum value found in subap.  This threshold is then either subtracted with everything negative being zero'd, or anything below this threshold is zero'd.
*/
int thresholdCorrelation(CentStruct *cstr,int threadno){
  CentThreadStruct *tstr=cstr->tstr[threadno];
  int i;
  float thresh=0.;
  float *subap=tstr->subap;
  int curnpxl=tstr->curnpxlSubap;
  if(tstr->corrnpxlx>tstr->curnpxlx || tstr->corrnpxly>tstr->curnpxly){
    subap=tstr->corrSubap;
  }
  if(cstr->correlationThresholdType==CORR_ABS_SUB || cstr->correlationThresholdType==CORR_ABS_ZERO){
    thresh=cstr->correlationThreshold;
  }else if(cstr->correlationThresholdType==CORR_FRAC_SUB || cstr->correlationThresholdType==CORR_FRAC_ZERO){
    //first find the max.
    for(i=0; i<curnpxl; i++){
      if(thresh<subap[i])
	thresh=subap[i];
    }
    thresh*=cstr->correlationThreshold;
  }
  if(cstr->correlationThresholdType==CORR_ABS_SUB || cstr->correlationThresholdType==CORR_FRAC_SUB){
    for(i=0; i<curnpxl; i++){
      if(subap[i]<thresh){
	subap[i]=0;
      }else{
	subap[i]-=thresh;
      }
    }
  }else if(cstr->correlationThresholdType==CORR_ABS_ZERO || cstr->correlationThresholdType==CORR_FRAC_ZERO){
    for(i=0; i<curnpxl; i++){
      if(subap[i]<thresh)
	subap[i]=0;
    }
  }
  return 0;
}
int storeCorrelationSubap(CentStruct *cstr,int threadno,float* corrbuf){
  CentThreadStruct *tstr=cstr->tstr[threadno];
  int cnt=0;
  int i,j;
  int *loc;
  float *subap;
  int *loc2;
  //int *rloc;
  int corrClip,npx;
  if(cstr->corrClipArr!=NULL){
    corrClip=cstr->corrClipArr[tstr->subindx];
  }else
    corrClip=cstr->corrClip;
  if(cstr->corrSubapLocation!=NULL)
    loc2=&(cstr->corrSubapLocation[tstr->subindx*6]);
  else
    loc2=&(cstr->realSubapLocation[tstr->subindx*6]);
  //rloc=&(cstr->realSubapLocation[tstr->subindx*6]);
  loc=&(cstr->arr->subapLocation[tstr->subindx*6]);
  if(tstr->corrnpxlx>tstr->curnpxlx || tstr->corrnpxly>tstr->curnpxly){
    //sy=loc[0]-rloc[0]+corrClip;//shift due to adaptive windowing.
    //sx=loc[3]-rloc[3]+corrClip;
    subap=tstr->corrSubap;
    npx=cstr->corrnpxlx[tstr->cam];
    corrbuf=&corrbuf[cstr->corrnpxlCum[tstr->cam]];//+corrClip*cstr->corrnpxlx[tstr->cam]+corrClip];
    for(i=loc2[0]+corrClip*loc2[2]; i<loc2[1]-corrClip*loc2[2];i+=loc2[2]){
      for(j=loc2[3]+corrClip*loc2[5]; j<loc2[4]-corrClip*loc2[5];j+=loc2[5]){
	corrbuf[i*npx+j]=subap[cnt];
	cnt++;
      }
    }
  }else{//corr image is same as subap size.
    subap=tstr->subap;
    corrbuf=&corrbuf[cstr->npxlCum[tstr->cam]];
    npx=cstr->npxlx[tstr->cam];
    for(i=loc[0]; i<loc[1]; i+=loc[2]){
      for(j=loc[3]; j<loc[4]; j+=loc[5]){
	corrbuf[i*npx+j]=subap[cnt];
	cnt++;
      }
    }
  }
  return 0;
}

/**
   Apply a slope linearisation to the measured slope values.
   This has been taken from DASP, slightly modified for out of range values.
*/
int applySlopeLinearisation(CentStruct *cstr,int threadno,float *cx, float *cy){
  CentThreadStruct *tstr=cstr->tstr[threadno];
  int lb,ub;
  float *cd,*cs;
  int centIndx=tstr->centindx;
  int nsteps;
  int i;
  //threadInfo->centIndx and +1 is the current centroid.  Note, centIndx is only ever even.
  nsteps=cstr->centCalnsteps;
  //first do the X centroid.
  lb=cstr->centCalBounds[centIndx*2];
  ub=cstr->centCalBounds[centIndx*2+1];
  cd=&cstr->centCalData[centIndx*nsteps];
  cs=&cstr->centCalSteps[centIndx*nsteps];
  if(*cx>cd[lb] && *cx<cd[ub]){
    //lies within the boundries specified, so can be linearised.
    i=1+lb;
    while(*cx>cd[i])
      i++;
    *cx=(*cx-cd[i-1])*(cs[i]-cs[i-1])/(cd[i]-cd[i-1])+cs[i-1];
  }else{//otherwise, out of bounds, so just keep as is.
    writeErrorVA(cstr->rtcErrorBuf,SLOPELINERROR,cstr->frameno,"Slope outside calibration range");
  }


  //then do the Y centroid.
  lb=cstr->centCalBounds[centIndx*2+2];
  ub=cstr->centCalBounds[centIndx*2+3];
  cd=&cstr->centCalData[(centIndx+1)*nsteps];
  cs=&cstr->centCalSteps[(centIndx+1)*nsteps];
  if(*cy>cd[lb] && *cy<cd[ub]){
    //lies within the boundries specified, so can be linearised.
    i=1+lb;
    while(*cy>cd[i])
      i++;
    *cy=(*cy-cd[i-1])*(cs[i]-cs[i-1])/(cd[i]-cd[i-1])+cs[i-1];
  }else{//otherwise, out of bounds, so just keep as is.
    writeErrorVA(cstr->rtcErrorBuf,SLOPELINERROR,cstr->frameno,"Slope outside calibration range");
  }
  return 0;
}


/**
   Updates subap locations, when in adaptive windowing mode
   At the moment, this is performed by 1 thread.  Need to think if there is a way that it can be spread between the threads, as in darc1.  The trouble is that it needs to be done before any pixels are read out, since this is used for computing how many pixels are required (i.e. how many the camera should wait for), and also shouldn't be done before dm demands are sent because that would increase latency... the best solution would be to split between threads while waiting for the first pixels to arrive... But at the moment, we don't assign threads to subaps, so can't do this. However, once that is changed, then we can... i.e. once we have a mapping assigning subapertures to threads in advance.
*/
int updateSubapLocation(CentStruct *cstr){
  int *loc=NULL,*rloc;
  int i,cnt=0;//,npxl=0,maxpxl=0,imax;
  int cam=0;
  //now update subapLocation for next time...
  i=0;
  //for(i=0; i<threadInfo->nsubapsProcessing; i++){
  for(i=0; i<cstr->nsubaps; i++){
    while(cstr->nsubapCum[cam+1]==i)//have got to end of subaps for this camera.
      cam++;
    if(cstr->subapFlag[i]==1){
      //subap is valid.
      loc=&(cstr->arr->subapLocation[i*6]);//adaptive.
      rloc=&(cstr->realSubapLocation[i*6]);//user defined
      loc[0]=rloc[0]+cstr->adaptiveCentPos[2*cnt+1]*rloc[2];
      loc[1]=rloc[1]+cstr->adaptiveCentPos[2*cnt+1]*rloc[2];
      loc[3]=rloc[3]+cstr->adaptiveCentPos[2*cnt]*rloc[5];
      loc[4]=rloc[4]+cstr->adaptiveCentPos[2*cnt]*rloc[5];
      loc[2]=rloc[2];
      loc[5]=rloc[5];
      //now move the centers if the window moves outside the CCD...
      while(loc[0]<0){
	loc[0]+=rloc[2];
	loc[1]+=rloc[2];
	cstr->adaptiveCentPos[2*cnt+1]++;
      }
      while(loc[1]>cstr->npxly[cam]){
	loc[1]-=rloc[2];
	loc[0]-=rloc[2];
	cstr->adaptiveCentPos[2*cnt+1]--;
      }
      while(loc[3]<0){
	loc[3]+=rloc[5];
	loc[4]+=rloc[5];
	cstr->adaptiveCentPos[2*cnt]++;
      }
      while(loc[4]>cstr->npxlx[cam]){
	loc[4]-=rloc[5];
	loc[3]-=rloc[5];
	cstr->adaptiveCentPos[2*cnt]--;
      }
      //calculate the new number of pixels required for this shifted subap.
      //npxl=info->pxlCnt[threadInfo->cursubindx+i]+((loc[0]-rloc[0])/rloc[2])*info->npxlx+(loc[3]-rloc[3])/rloc[5];
      //if(npxl>maxpxl)//and keep a note of the maximum number required.
      //maxpxl=npxl;
      cnt++;//increment the number of subaps valid.
    }
  }

  //here, we calculate the number of extra pixels required for the last subap of this block.  
  //if(maxpxl>info->npxlx*info->npxly)
  //maxpxl=info->npxlx*info->npxly;
  return 0;//maxpxl;// ((loc[0]-rloc[0])/rloc[2])*info->npxlx+(loc[3]-rloc[3])/rloc[5];
}


/**
   Calculates the slope - currently, basic and adaptive windowing only, centre of gravity estimation (or weighted if weighting supplied, though the method for this hasn't been written yet).
*/
int calcCentroid(CentStruct *cstr,int threadno){
  CentThreadStruct *tstr=cstr->tstr[threadno];
  float sum=0.;
  int i,j;
  float cy,cx;
  float minflux;
  int centindx=tstr->centindx;
  float *subap=tstr->subap;
  int curnpxlx=tstr->curnpxlx;
  int curnpxly=tstr->curnpxly;
  int centroidMode;
  if(cstr->centroidModeArr==NULL)
    centroidMode=cstr->centroidMode;
  else
    centroidMode=cstr->centroidModeArr[tstr->subindx];
  //If doing correlation centroiding, the idea would be to perform the correlation first here, including any flooring etc of the corelation image.  Then, can apply the chosen centroid algorithm to this here (ie CoG, WCoG etc).

  if(cstr->centWeighting!=NULL){
    applyCentWeighting(cstr,threadno);
  }
  cx=0.;
  cy=0.;
  if(cstr->fluxThresholdArr!=NULL){
    minflux=cstr->fluxThresholdArr[centindx/2];
  }else{
    minflux=cstr->fluxThreshold;
  }
  //if(info->windowMode==WINDOWMODE_BASIC || info->windowMode==WINDOWMODE_ADAPTIVE){
  if(centroidMode==CENTROIDMODE_CORRELATIONCOG){// || cstr->centroidMode==CENTROIDMODE_CORRELATIONGAUSSIAN){
    //do the correlation...
    calcCorrelation(cstr,threadno);
    //here, before thresholding, should probably store this in a circular buffer that can be sent to user.  Or maybe, this is the calibrated image buffer.
    if(cstr->rtcCorrBuf!=NULL){// && cstr->rtcCorrBuf->addRequired){
      printf("addRequired\n");
      storeCorrelationSubap(cstr,threadno,cstr->corrbuf);
    }
    thresholdCorrelation(cstr,threadno);
    if(cstr->rtcCalCorrBuf!=NULL && cstr->rtcCalCorrBuf->addRequired)
      storeCorrelationSubap(cstr,threadno,cstr->calcorrbuf);
    if(tstr->corrnpxlx>tstr->curnpxlx || tstr->corrnpxly>tstr->curnpxly){
      int corrClip;
      if(cstr->corrClipArr!=NULL){
	corrClip=cstr->corrClipArr[tstr->subindx];
      }else
	corrClip=cstr->corrClip;

      subap=tstr->corrSubap;
      curnpxlx=tstr->corrnpxlx-2*corrClip;
      curnpxly=tstr->corrnpxly-2*corrClip;
    }
  }

  if(centroidMode==CENTROIDMODE_COG || centroidMode==CENTROIDMODE_CORRELATIONCOG){
    if(cstr->centIndexArr==NULL){
      int cnt=0;
      for(i=0; i<curnpxly; i++){
	for(j=0; j<curnpxlx; j++){
	  sum+=subap[cnt];//i*curnpxlx+j];
	  cx+=j*subap[cnt];//i*curnpxlx+j];
	  cy+=i*subap[cnt];//i*curnpxlx+j];
	  cnt++;
	}
      }
      if(sum>=minflux && sum!=0){
	cy/=sum;
	cx/=sum;
	cy-=curnpxly/2.-0.5;
	cx-=curnpxlx/2.-0.5;
      }else{
	cy=0;
	cx=0;
      }
    }else{
      int *loc;
      int cnt=0;
      int pos;
      int loc54;
      float cres[4];//[0] is cy, [1] is cx, [2] is sumy, [3] is sumx
      int k;
      if(cstr->corrSubapLocation==NULL)
	loc=&cstr->realSubapLocation[tstr->subindx*6];
      else
	loc=&cstr->corrSubapLocation[tstr->subindx*6];
      loc54=loc[5]*cstr->centIndexSize;
      for(i=0; i<4; i++)
	cres[i]=0;
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	pos=(cstr->corrnpxlCum[tstr->cam]+i*cstr->corrnpxlx[tstr->cam])*cstr->centIndexSize;
	for(j=loc[3]; j<loc[4]; j+=loc[5]){
	  for(k=0;k<cstr->centIndexSize;k++)//NOTE: 1<=centIndexSize<=4
	    cres[k]+=subap[cnt]*cstr->centIndexArr[pos+k];
	  for(k=cstr->centIndexSize;k<4;k++)//note, k>=1
	    cres[k]+=subap[cnt]*((k==1)*j+(k>1));//no index arrays provided for these ones.
	  //cy+=subap[cnt]*cstr->centIndexArr[pos+0];
	  //cx+=subap[cnt]*cstr->centIndexArr[pos+1];
	  //ysum+=subap[cnt]*cstr->centIndexArr[pos+2];
	  //xsum+=subap[cnt]*cstr->centIndexArr[pos+3];
	  cnt++;
	  pos+=loc54;
	}
      }
      //Looks slightly strange way of doing it, but this way, matched filter can also be used - when centIndexArr[2 and 3] are all zeros, so cres[2,3]==0, if set minflux to less than zero.
      if(cres[2]>=minflux){
	if(cres[2]!=0)
	  cy=cres[0]/cres[2];
	else
	  cy=cres[0];
	sum=cres[2];
      }else
	cy=0;
      if(cres[3]>=minflux){
	if(cres[3]!=0)
	  cx=cres[1]/cres[3];
	else
	  cx=cres[1];
	sum=cres[3];
      }else
	cx=0;
      //don't subtract an offset here, this can be done by the refCentroids.
    }
      
    /*}else if(cstr->centroidMode==CENTROIDMODE_GAUSSIAN || cstr->centroidMode==CENTROIDMODE_CORRELATIONGAUSSIAN){
    //do some sort of gaussian fit to the data...
    printf("TODOxxx - gaussian fit to the data to get centroid (not yet implemented)\n");*/
  }else{
    printf("centroid mode not yet implemented\n");
  }
  if(sum>=minflux){
    if(cstr->windowMode==WINDOWMODE_ADAPTIVE){
      //add centroid offsets to get the overall location correct
      //(i.e. the distance from it's nominal centre).
      cx+=cstr->adaptiveCentPos[centindx];
      cy+=cstr->adaptiveCentPos[centindx+1];
      //and calculate adaptive window for next time.
      calcAdaptiveWindow(cstr,threadno,cx,cy);
    }else if(cstr->windowMode==WINDOWMODE_GLOBAL){//add the current subap offset here
      cx+=cstr->adaptiveCentPos[centindx];
      cy+=cstr->adaptiveCentPos[centindx+1];
    }
    if(cstr->centCalData!=NULL){//appy centroid linearisation...
      //Here, we apply the centroid linearisation.
      applySlopeLinearisation(cstr,threadno,&cx,&cy);
    }
    if(cstr->refCents!=NULL){//subtract reference centroids.
      cx-=cstr->refCents[centindx];
      cy-=cstr->refCents[centindx+1];
    }
  }
  cstr->arr->centroids[centindx]=cx;
  cstr->arr->centroids[centindx+1]=cy;
  cstr->arr->flux[centindx/2]=sum;
  return 0;
}




int slopeOpen(char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **centHandle,int ncam,int nthreads,unsigned int frameno,unsigned int **centframeno,int *centframenoSize,int totCents){
  CentStruct *cstr;
  int err;
  char *pn;
  int i;
  printf("Opening rtcslope\n");
  if((pn=makeParamNames())==NULL){
    printf("Error making paramList - please recode rtcslope.c\n");
    *centHandle=NULL;
    return 1;
  }
  if((cstr=calloc(sizeof(CentStruct),1))==NULL){
    printf("Error allocating slope memory\n");
    *centHandle=NULL;
    return 1;
  }
  *centHandle=(void*)cstr;
  cstr->paramNames=pn;
  cstr->arr=arr;
  cstr->totCents=totCents;
  //cstr->centframeno=centframeno;
  //cstr->centframenoSize=centframenoSize;
  cstr->prefix=prefix;
  pthread_mutex_init(&cstr->fftcreateMutex,NULL);
  cstr->nthreads=nthreads;
  if((cstr->tstr=calloc(sizeof(CentThreadStruct*),nthreads))==NULL){
    printf("Error allocating CentThreadStruct\n");
    slopeClose(centHandle);
    *centHandle=NULL;
    return 1;
  }
  for(i=0; i<nthreads;i++){
    if((cstr->tstr[i]=malloc(sizeof(CentThreadStruct)))==NULL){
      printf("Error allocating CentThreadStruct %d\n",i);
      slopeClose(centHandle);
      *centHandle=NULL;
      return 1;
    }else{
      memset(cstr->tstr[i],0,sizeof(CentThreadStruct));
    }
  }


  cstr->rtcErrorBuf=rtcErrorBuf;
  err=slopeNewParam(*centHandle,pbuf,frameno,arr,totCents);
  if(err!=0){
    printf("Error in slopeOpen...\n");
    slopeClose(centHandle);
    *centHandle=NULL;
    return 1;
  }
  return 0;
}

/**
   Called when parameters have changed
*/
int slopeNewParam(void *centHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr,int totCents){
  CentStruct *cstr=(CentStruct*)centHandle;
  int nfound;
  int err=0;
  int i,nb,m;
  int *index=cstr->index;
  void **values=cstr->values;
  char *dtype=cstr->dtype;
  int *nbytes=cstr->nbytes;
  int resetAdaptiveWindows=0;
  cstr->arr=arr;
  cstr->totCents=totCents;
  nfound=bufferGetIndex(pbuf,NBUFFERVARIABLES,cstr->paramNames,index,values,dtype,nbytes);
  if(nfound!=NBUFFERVARIABLES){
    for(i=0; i<NBUFFERVARIABLES; i++){
      if(index[i]<0){
	if(i==CORRFFTPATTERN || i==CORRTHRESHTYPE || i==CORRTHRESH || i==CORRSUBAPLOCATION || i==CORRNPXLX || i==CORRNPXLCUM || i==CORRCLIP || i==CENTCALDATA || i==CENTCALSTEPS || i==CENTCALBOUNDS || i==CORRNSTORE){
	  printf("%16s not found - continuing\n",&cstr->paramNames[i*BUFNAMESIZE]);
	}else{
	  printf("Missing %16s\n",&cstr->paramNames[i*BUFNAMESIZE]);
	  err=1;
	  writeErrorVA(cstr->rtcErrorBuf,-1,cstr->frameno,"ncam error\n");
	}
      }
    }
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
    if(cstr->npxlCum==NULL && ((cstr->npxlCum=malloc(sizeof(int)*(cstr->ncam+1)))==NULL)){
      err=1;
      printf("npxlCum malloc failed in rtcslope\n");
    }
    if(cstr->nsubapCum==NULL && ((cstr->nsubapCum=malloc(sizeof(int)*(cstr->ncam+1)))==NULL)){
      err=1;
      printf("nsubapCum malloc failed in rtcslope\n");
    }
    if(err==0){
      cstr->nsubapCum[0]=0;
      cstr->npxlCum[0]=0;
      for(i=0; i<cstr->ncam; i++){
	cstr->nsubaps+=cstr->nsub[i];
	cstr->nsubapCum[i+1]=cstr->nsubaps;
	cstr->totPxls+=cstr->npxlx[i]*cstr->npxly[i];
	cstr->npxlCum[i+1]=cstr->totPxls;//cstr->npxlCum[i]+cstr->npxlx[i]*cstr->npxly[i];
      }
    }
    if(dtype[SUBAPLOCATION]=='i' && nbytes[SUBAPLOCATION]==sizeof(int)*6*cstr->nsubaps)
      cstr->realSubapLocation=(int*)values[SUBAPLOCATION];
    else{
      cstr->realSubapLocation=NULL;
      printf("subapLocation error\n");
      err=1;
    }
    if(dtype[SUBAPFLAG]=='i' && nbytes[SUBAPFLAG]==sizeof(int)*cstr->nsubaps){
      cstr->subapFlag=(int*)values[SUBAPFLAG];
    }else{
      printf("subapFlag error\n");
      err=1;
    }
    
    if(dtype[MAXADAPOFFSET]=='i'){
      if(nbytes[MAXADAPOFFSET]==sizeof(int)){
	cstr->maxAdapOffset=*((int*)values[MAXADAPOFFSET]);
	cstr->maxAdapOffsetArr=NULL;
      }else if(nbytes[MAXADAPOFFSET]==sizeof(int)*cstr->totCents/2){
	cstr->maxAdapOffsetArr=((int*)values[MAXADAPOFFSET]);
      }else{
	cstr->maxAdapOffsetArr=NULL;
	err=1;
	printf("maxAdapOffset error\n");
      }
    }else{
      cstr->maxAdapOffsetArr=NULL;
      err=1;
      printf("maxAdapOffset error\n");
    }
    if(dtype[ADAPRESETCOUNT]=='i'){
      if(nbytes[ADAPRESETCOUNT]==sizeof(int)){
	cstr->adapResetCount=*((int*)values[ADAPRESETCOUNT]);
	cstr->adapResetCountArr=NULL;
      }else if(nbytes[ADAPRESETCOUNT]==sizeof(int)*cstr->totCents/2){
	cstr->adapResetCountArr=((int*)values[ADAPRESETCOUNT]);
      }else{
	cstr->adapResetCountArr=NULL;
	err=1;
	printf("adapResetCount error\n");
      }
    }else{
      cstr->adapResetCountArr=NULL;
      err=1;
      printf("adapResetCount error\n");
    }
    nb=nbytes[ADAPTIVEGROUP];
    if(nb==0){
      cstr->adaptiveGroup=NULL;
      cstr->nAdaptiveGroups=1;
    }else if(nb==sizeof(int)*cstr->totCents/2 && dtype[ADAPTIVEGROUP]=='i'){
      cstr->adaptiveGroup=((int*)values[ADAPTIVEGROUP]);
      m=0;
      for(i=0; i<cstr->totCents/2; i++){
	if(cstr->adaptiveGroup[i]>m)
	  m=cstr->adaptiveGroup[i];
      }
      cstr->nAdaptiveGroups=m+1;
      if(cstr->adaptiveGroupSize<cstr->nAdaptiveGroups){//need to re malloc mem.
	if(cstr->groupSumX!=NULL)
	  free(cstr->groupSumX);
	cstr->groupSumX=malloc(sizeof(float)*cstr->nAdaptiveGroups);
	if(cstr->groupSumY!=NULL)
	  free(cstr->groupSumY);
	cstr->groupSumY=malloc(sizeof(float)*cstr->nAdaptiveGroups);
	if(cstr->groupSum!=NULL)
	  free(cstr->groupSum);
	cstr->groupSum=malloc(sizeof(int)*cstr->nAdaptiveGroups);
	if(cstr->groupSumX==NULL || cstr->groupSumY==NULL || cstr->groupSum==NULL){
	  printf("unable to malloc groupSum for adaptiveWinGroup\n");
	  err=1;
	  cstr->adaptiveGroupSize=0;
	  cstr->adaptiveGroup=NULL;
	  cstr->nAdaptiveGroups=1;
	}else{
	  cstr->adaptiveGroupSize=cstr->nAdaptiveGroups;
	}
      }
    }else{
      printf("adaptiveWinGroup error: nbytes=%d should be %d type %c should be i\n",nb,(int)(sizeof(int)*cstr->totCents/2),dtype[ADAPTIVEGROUP]);
      err=1;
      cstr->adaptiveGroup=NULL;
      cstr->nAdaptiveGroups=1;
    }
    nb=nbytes[REFCENTROIDS];
    if(nb==0){
      cstr->refCents=NULL;
    }else if(dtype[REFCENTROIDS]=='f' && nb==sizeof(float)*cstr->totCents){
      cstr->refCents=(float*)values[REFCENTROIDS];
    }else{
      printf("refCentroids error\n");
      err=1;
      cstr->refCents=NULL;
    }
    if(dtype[ADAPTIVEWINGAIN]=='f' && nbytes[ADAPTIVEWINGAIN]==sizeof(float)){
      cstr->adaptiveWinGain=*((float*)values[ADAPTIVEWINGAIN]);
    }else{
      err=1;
      printf("adaptiveWinGain error\n");
    }
    if(index[CORRTHRESHTYPE]<0){
      cstr->correlationThresholdType=0;
    }else{
      if(dtype[CORRTHRESHTYPE]=='i' && nbytes[CORRTHRESHTYPE]==sizeof(int)){
	cstr->correlationThresholdType=*((int*)values[CORRTHRESHTYPE]);
      }else{
	err=1;
	printf("correlationThresholdType error\n");
      }
    }
    if(index[CORRTHRESH]<0){
      cstr->correlationThreshold=0.;
    }else{
      if(dtype[CORRTHRESH]=='f' && nbytes[CORRTHRESH]==sizeof(float)){
	cstr->correlationThreshold=*((float*)values[CORRTHRESH]);
      }else{
	err=1;
	printf("correlationThreshold error\n");
      }
    }
    if(index[CORRSUBAPLOCATION]<0){
      cstr->corrSubapLocation=NULL;
    }else{
      if(dtype[CORRSUBAPLOCATION]=='i' && nbytes[CORRSUBAPLOCATION]==sizeof(int)*6*cstr->nsubaps)
	cstr->corrSubapLocation=(int*)values[CORRSUBAPLOCATION];
      else if(nbytes[CORRSUBAPLOCATION]==0){
	cstr->corrSubapLocation=NULL;
      }else{
	cstr->corrSubapLocation=NULL;
	printf("corrSubapLoc error\n");
	err=1;
      }
    }
    if(cstr->corrSubapLocation!=NULL){
      if(index[CORRNPXLX]<0){
	err=1;
	printf("error: corrnpxlx not found\n");
	writeErrorVA(cstr->rtcErrorBuf,-1,cstr->frameno,"corrnpxlx error");
      }else{
	if(nbytes[CORRNPXLX]==sizeof(int)*cstr->ncam && dtype[CORRNPXLX]=='i')
	  cstr->corrnpxlx=(int*)values[CORRNPXLX];
	else{
	  cstr->corrnpxlx=NULL;
	  printf("corrnpxlx error\n");
	  err=1;
	}
      }
      if(index[CORRNPXLCUM]<0){
	err=1;
	printf("error: corrnpxlxCum not found\n");
	writeErrorVA(cstr->rtcErrorBuf,-1,cstr->frameno,"corrnpxlxCum error");
      }else{
	if(nbytes[CORRNPXLCUM]==sizeof(int)*(cstr->ncam+1) && dtype[CORRNPXLCUM]=='i')
	  cstr->corrnpxlCum=(int*)values[CORRNPXLCUM];
	else{
	  cstr->corrnpxlCum=NULL;
	  printf("corrnpxlCum error\n");
	  err=1;
	}
      }
    }else{
      cstr->corrnpxlCum=cstr->npxlCum;
      cstr->corrnpxlx=cstr->npxlx;
    }
    cstr->fftCorrPatternSize=cstr->totPxls;//but it may get bigger.
    if(index[CORRFFTPATTERN]<0){
      cstr->fftCorrelationPattern=NULL;
    }else{
      nb=nbytes[CORRFFTPATTERN];
      if(nb==0)
	cstr->fftCorrelationPattern=NULL;
      else if(dtype[CORRFFTPATTERN]=='f'){// && nb/sizeof(float)>=cstr->totPxls){
	cstr->fftCorrelationPattern=(float*)values[CORRFFTPATTERN];
	cstr->fftCorrPatternSize=nb/sizeof(float);
      }else{
	printf("fftCorrelationPattern error\n");
	err=1;
	cstr->fftCorrelationPattern=NULL;
      }
    }
    if(index[CORRCLIP]<0){
      cstr->corrClipArr=NULL;
      cstr->corrClip=0;
    }else{
      nb=nbytes[CORRCLIP];
      cstr->corrClipArr=NULL;
      if(nb==0)
	cstr->corrClip=0;
      else if(dtype[CORRCLIP]=='i'){
	if(nb==sizeof(int))
	  cstr->corrClip=*((int*)values[CORRCLIP]);
	else if(nb==sizeof(int)*cstr->nsubaps)
	  cstr->corrClipArr=(int*)values[CORRCLIP];
	else{
	  printf("corrClip error\n");
	  err=1;
	}
      }else{
	printf("corrClip error\n");
	err=1;
      }
    }
    cstr->centCalnsteps=0;
    if(index[CENTCALDATA]<0)
      cstr->centCalData=NULL;
    else{
      nb=nbytes[CENTCALDATA];
      if(nb==0)
	cstr->centCalData=NULL;
      else if(dtype[CENTCALDATA]=='f'){
	cstr->centCalnsteps=nb/sizeof(float)/cstr->totCents;
	if(nb==sizeof(float)*cstr->totCents*cstr->centCalnsteps)
	  cstr->centCalData=(float*)values[CENTCALDATA];
	else{
	  printf("centCalData error\n");
	  err=1;
	  cstr->centCalnsteps=0;
	}
      }else{
	printf("centCalData error\n");
	err=1;
      }
    }
    if(index[CENTCALSTEPS]<0 && (nb==0 || cstr->centCalnsteps==0)){
      cstr->centCalSteps=NULL;
    }else{
      nb=nbytes[CENTCALSTEPS];
      if(nb==0 || cstr->centCalnsteps==0)
	cstr->centCalSteps=NULL;
      else if(dtype[CENTCALSTEPS]=='f' && nb/sizeof(float)==cstr->totCents*cstr->centCalnsteps){
	cstr->centCalSteps=(float*)values[CENTCALSTEPS];
      }else{
	printf("centCalSteps error\n");
	err=1;
      }
    }
    if(index[CENTCALBOUNDS]<0 && (nb==0 || cstr->centCalnsteps==0)){
      cstr->centCalBounds=NULL;
    }else{
      nb=nbytes[CENTCALBOUNDS];
      if(nb==0 || cstr->centCalnsteps==0)
	cstr->centCalBounds=NULL;
      else if(dtype[CENTCALBOUNDS]=='i' && nb/sizeof(int)==2*cstr->totCents){
	cstr->centCalBounds=(int*)values[CENTCALBOUNDS];
      }else{
	printf("centCalBounds error\n");
	err=1;
      }
    }
    if(index[CORRNSTORE]<0)
      cstr->corrNStore=100;
    else if(dtype[CORRNSTORE]=='i' && nbytes[CORRNSTORE]==sizeof(int)){
      cstr->corrNStore=*((int*)values[CORRNSTORE]);
    }else{
      printf("corrNStore error - using 100\n");
      cstr->corrNStore=100;
    }
    if(cstr->centCalData==NULL || cstr->centCalSteps==NULL || cstr->centCalBounds==NULL){
      cstr->centCalData=NULL;
      cstr->centCalSteps=NULL;
      cstr->centCalBounds=NULL;
    }
    nb=nbytes[CENTROIDMODE];
    if(nb!=0 && dtype[CENTROIDMODE]=='s'){
      if(strncmp(values[CENTROIDMODE],"CoG",nb)==0){
	cstr->centroidMode=CENTROIDMODE_COG;
      }else if(strncmp(values[CENTROIDMODE],"CorrelationCoG",nb)==0){
	cstr->centroidMode=CENTROIDMODE_CORRELATIONCOG;
      }else{
	cstr->centroidMode=CENTROIDMODE_ERROR;
	printf("Unrecognised centroidMode\n");
      }
      cstr->centroidModeArr=NULL;
    }else if(nb==sizeof(int) && dtype[CENTROIDMODE]=='i'){
      cstr->centroidMode=*((int*)values[CENTROIDMODE]);
      cstr->centroidModeArr=NULL;
    }else if(nb==sizeof(int)*cstr->nsubaps && dtype[CENTROIDMODE]=='i'){
      cstr->centroidModeArr=(int*)values[CENTROIDMODE];
      cstr->centroidMode=0;
    }else{
      err=1;
      printf("centroidMode error\n");
      cstr->centroidModeArr=NULL;
      cstr->centroidMode=0;
    }

    if(/*cstr->centroidMode==CENTROIDMODE_CORRELATIONGAUSSIAN ||*/ cstr->centroidMode==CENTROIDMODE_CORRELATIONCOG || cstr->centroidModeArr!=NULL){
      if(cstr->fftCorrelationPattern==NULL){
	printf("Error - corrFFTPattern not specified correctly\n");
	err=1;
      }

      if(cstr->rtcCorrBuf==NULL && cstr->corrNStore>0){
	//open the circular buffer.
	char *tmp;
	if(asprintf(&tmp,"/%srtcCorrBuf",cstr->prefix)==-1){
	  printf("Error asprintf in rtcslope - exiting\n");
	  exit(1);
	}
	cstr->rtcCorrBuf=openCircBuf(tmp,1,&cstr->fftCorrPatternSize,'f',cstr->corrNStore);
	free(tmp);
      }else if(cstr->rtcCorrBuf!=NULL){
	if(cstr->rtcCorrBuf->datasize!=cstr->fftCorrPatternSize*sizeof(float)){
	  if(circReshape(cstr->rtcCorrBuf,1,&cstr->fftCorrPatternSize,'f')!=0){
	    printf("Error reshaping rtcCorrBuf\n");
	    err=1;
	  }
	}
      }
      if(cstr->rtcCalCorrBuf==NULL && cstr->corrNStore>0){
	//open the circular buffer.
	char *tmp;
	if(asprintf(&tmp,"/%srtcCalCorrBuf",cstr->prefix)==-1)
	  exit(1);
	cstr->rtcCalCorrBuf=openCircBuf(tmp,1,&cstr->fftCorrPatternSize,'f',cstr->corrNStore);
	free(tmp);
      }else if(cstr->rtcCalCorrBuf!=NULL){
	if(cstr->rtcCalCorrBuf->datasize!=cstr->fftCorrPatternSize*sizeof(float)){
	  if(circReshape(cstr->rtcCalCorrBuf,1,&cstr->fftCorrPatternSize,'f')!=0){
	    printf("Error reshaping rtcCalCorrBuf\n");
	    err=1;
	  }
	}
      }
      if(cstr->corrbufSize<cstr->fftCorrPatternSize){
	if(cstr->corrbuf!=NULL)
	  free(cstr->corrbuf);
	if(cstr->calcorrbuf!=NULL)
	  free(cstr->calcorrbuf);
	cstr->corrbufSize=cstr->fftCorrPatternSize;
	if((cstr->corrbuf=malloc(sizeof(float)*cstr->fftCorrPatternSize))==NULL){
	  printf("malloc of corrbuf failed\n");
	  err=1;
	  cstr->corrbufSize=0;
	}
	if((cstr->calcorrbuf=malloc(sizeof(float)*cstr->fftCorrPatternSize))==NULL){
	  printf("malloc of calcorrbuf failed\n");
	  err=1;
	  cstr->corrbufSize=0;
	}
      }
      memset(cstr->calcorrbuf,0,sizeof(float)*cstr->fftCorrPatternSize);
      memset(cstr->corrbuf,0,sizeof(float)*cstr->fftCorrPatternSize);
    }

    nb=nbytes[CENTROIDWEIGHT];
    if(nb==0)
      cstr->centWeighting=NULL;
    else if(dtype[CENTROIDWEIGHT]=='f' && nb==4){
      cstr->centWeighting=((float*)values[CENTROIDWEIGHT]);
    }else{
      err=1;
      printf("centWeight error\n");
    }
    nb=nbytes[FLUXTHRESHOLD];
    if(dtype[FLUXTHRESHOLD]=='f'){
      if(nb==sizeof(float)){
	cstr->fluxThresholdArr=NULL;
	cstr->fluxThreshold=*(float*)values[FLUXTHRESHOLD];
      }else if(nb==sizeof(float)*cstr->totCents/2){
	cstr->fluxThresholdArr=(float*)values[FLUXTHRESHOLD];;
	cstr->fluxThreshold=0.;
      }else{
	cstr->fluxThresholdArr=NULL;
	printf("fluxThreshold error\n");
	err=1;
      }
    }else{
      cstr->fluxThresholdArr=NULL;
      printf("fluxThreshold error\n");
      err=1;
    }
    nb=nbytes[WINDOWMODE];
    resetAdaptiveWindows=0;
    if(nb!=0 && dtype[WINDOWMODE]=='s'){
      if(strncmp(values[WINDOWMODE],"basic",nb)==0){
	cstr->windowMode=WINDOWMODE_BASIC;
      }else if(strncmp(values[WINDOWMODE],"adaptive",nb)==0){
	if(cstr->windowMode!=WINDOWMODE_ADAPTIVE){
	  cstr->windowMode=WINDOWMODE_ADAPTIVE;
	  //if(updateIndex)
	  resetAdaptiveWindows=1;
	}
      }else if(strncmp(values[WINDOWMODE],"global",nb)==0){
	if(cstr->windowMode!=WINDOWMODE_GLOBAL){
	  cstr->windowMode=WINDOWMODE_GLOBAL;
	  //if(updateIndex)
	  resetAdaptiveWindows=1;
	}
      }else{
	cstr->windowMode=WINDOWMODE_ERROR;
	printf("windowMode string error (unrecognised)\n");
      }
    }else{
      err=1;
      printf("windowMode error\n");
    }
    nb=nbytes[CENTINDEXARRAY];
    if(nb==0){
      cstr->centIndexArr=NULL;
      cstr->centIndexSize=0;
    }else{
      if(dtype[CENTINDEXARRAY]=='f'){
	cstr->centIndexSize=nb/(sizeof(float)*cstr->fftCorrPatternSize);
	if(nb==cstr->centIndexSize*sizeof(float)*cstr->fftCorrPatternSize){
	  cstr->centIndexArr=values[CENTINDEXARRAY];
	}else{
	  cstr->centIndexArr=NULL;
	  cstr->centIndexSize=0;
	  printf("centIndexArray error - wrong size\n");
	  err=1;
	}
	if(cstr->centIndexSize>4){
	  cstr->centIndexArr=NULL;
	  cstr->centIndexSize=0;
	  printf("centIndexArray error - wrong size (too large)\n");
	  err=1;
	}	  
      }else{
	err=1;
	printf("centIndexArray error - wrong type\n");
	cstr->centIndexArr=NULL;
	cstr->centIndexSize=0;
      }
    }
  }
  if(err==0){
    if(cstr->adaptiveCentPosSize<cstr->totCents){
      if(cstr->adaptiveCentPos!=NULL)
	free(cstr->adaptiveCentPos);
      cstr->adaptiveCentPosSize=cstr->totCents;
      if((cstr->adaptiveCentPos=malloc(sizeof(int)*cstr->totCents))==NULL){
	printf("malloc of adaptiveCentPos failed\n");
	err=1;
	cstr->adaptiveCentPosSize=0;
      }
    }
    if(cstr->adaptiveWinPosSize<cstr->totCents){
      if(cstr->adaptiveWinPos!=NULL)
	free(cstr->adaptiveWinPos);
      cstr->adaptiveWinPosSize=cstr->totCents;
      if((cstr->adaptiveWinPos=malloc(sizeof(float)*cstr->totCents))==NULL){
	printf("malloc of adaptiveWinPos failed\n");
	err=1;
	cstr->adaptiveWinPosSize=0;
      }
    }
    if(cstr->adaptiveMaxCountSize<cstr->totCents*2){
      if(cstr->adaptiveMaxCount!=NULL)
	free(cstr->adaptiveMaxCount);
      cstr->adaptiveMaxCountSize=cstr->totCents*2;
      if((cstr->adaptiveMaxCount=calloc(sizeof(int),cstr->totCents*2))==NULL){
	printf("alloc of adaptiveMaxCount failed\n");
	err=1;
	cstr->adaptiveMaxCountSize=0;
      }
    }
    if(err==0 && resetAdaptiveWindows==1){
      memset(cstr->adaptiveCentPos,0,sizeof(int)*cstr->totCents);
      memset(cstr->adaptiveWinPos,0,sizeof(float)*cstr->totCents);
      memset(cstr->adaptiveMaxCount,0,sizeof(int)*cstr->totCents*2);
    }
  }

  return err;
}
/**
   Close a centroid camera of type name.  Args are passed in the float array of size n, and state data is in centHandle, which should be freed and set to NULL before returning.
*/

int slopeClose(void **centHandle){
  CentStruct *cstr=(CentStruct*)*centHandle;
  int i;
  printf("closing rtcslope library %p %p\n",centHandle,cstr);
  if(cstr!=NULL){
    if(cstr->rtcCorrBuf!=NULL){
      circClose(cstr->rtcCorrBuf);
    }
    if(cstr->rtcCalCorrBuf!=NULL){
      circClose(cstr->rtcCalCorrBuf);
    }
    pthread_mutex_destroy(&cstr->fftcreateMutex);
    if(cstr->corrbuf!=NULL)
      free(cstr->corrbuf);
    if(cstr->calcorrbuf!=NULL)
      free(cstr->calcorrbuf);
    if(cstr->paramNames!=NULL)
      free(cstr->paramNames);
    if(cstr->npxlCum!=NULL)
      free(cstr->npxlCum);
    if(cstr->tstr!=NULL){
      for(i=0; i<cstr->nthreads; i++){
	if(cstr->tstr[i]!=NULL){
	  if(cstr->tstr[i]->corrSubap!=NULL)
	    free(cstr->tstr[i]->corrSubap);
	  free(cstr->tstr[i]);
	}
      }
      free(cstr->tstr);
    }
    if(cstr->groupSum!=NULL)
      free(cstr->groupSum);
    if(cstr->groupSumY!=NULL)
      free(cstr->groupSumY);
    if(cstr->groupSumX!=NULL)
      free(cstr->groupSumX);
    if(cstr->fftIndex!=NULL)
      free(cstr->fftIndex);
    if(cstr->fftPlanArray!=NULL)
      free(cstr->fftPlanArray);
    if(cstr->adaptiveMaxCount!=NULL)
      free(cstr->adaptiveMaxCount);
    free(cstr);
  }
  *centHandle=NULL;
  return 0;
}
int slopeNewFrameSync(void *centHandle,unsigned int frameno,double timestamp){
  CentStruct *cstr=(CentStruct*)centHandle;
  cstr->frameno=frameno;
  cstr->timestamp=timestamp;
  if(cstr->windowMode==WINDOWMODE_ADAPTIVE || cstr->windowMode==WINDOWMODE_GLOBAL){
    updateSubapLocation(cstr);
  }
  
  if(cstr->rtcCorrBuf)
    circSetAddIfRequired(cstr->rtcCorrBuf,frameno);
  printf("SetAddIfRequired %d %d\n",(int)frameno,cstr->rtcCorrBuf->addRequired);
  if(cstr->rtcCalCorrBuf)
    circSetAddIfRequired(cstr->rtcCalCorrBuf,frameno);
  return 0;
}


/**
   If WPU - Wait for the next n subapertures of the current frame to arrive.
   Or, process the data (probably in subap, if you are using a standard calibration)
   Return 1 on error, 0 on okay or -1 if no slopes arrived, but this is not an error.
   The frameno can also be updated.
*/
#ifndef OLDMULTINEWFN
int slopeCalcSlope(void *centHandle,int cam,int threadno,int nsubs,float *subap, int subapSize,int subindx,int centindx,int nprocessing,int rubbish){//subap thread.
  CentStruct *cstr=(CentStruct*)centHandle;
  CentThreadStruct *tstr=cstr->tstr[threadno];
  int i,pos=0;
  int *loc;
  if(subapSize==0 || subap==NULL)
    return 0;
  tstr->cam=cam;
  for(i=0;i<nprocessing;i++){
    if(cstr->subapFlag[subindx]==1){
      tstr->subap=&subap[pos];
      tstr->subindx=subindx;
      tstr->centindx=centindx;
      loc=&(cstr->arr->subapLocation[subindx*6]);
      tstr->curnpxly=(loc[1]-loc[0])/loc[2];
      tstr->curnpxlx=(loc[4]-loc[3])/loc[5];
      tstr->curnpxl=tstr->curnpxly*tstr->curnpxlx;
      if(pos+tstr->curnpxl>subapSize){
	printf("Error - subapSize smaller than expected in rtcslope\n");
      }else{
	calcCentroid(cstr,threadno);
      }
      pos+=((tstr->curnpxl+15)/16)*16;
      centindx+=2;
    }
    subindx++;
  }
  return 0;
}
#else
int slopeCalcSlope(void *centHandle,int cam,int threadno,int nsubs,float *subap, int subapSize,int subindx,int centindx,int curnpxlx,int curnpxly){//subap thread.
  CentStruct *cstr=(CentStruct*)centHandle;
  CentThreadStruct *tstr=cstr->tstr[threadno];
  tstr->subap=subap;
  //tstr->subapSize=subapSize;
  tstr->subindx=subindx;
  tstr->centindx=centindx;
  tstr->curnpxlx=curnpxlx;
  tstr->curnpxly=curnpxly;
  tstr->curnpxl=curnpxlx*curnpxly;
  tstr->cam=cam;
  calcCentroid(cstr,threadno);
  return 0;
}
#endif

int slopeFrameFinishedSync(void *centHandle,int err,int forcewrite){//subap thread (once)
  CentStruct *cstr=(CentStruct*)centHandle;
  if(cstr->rtcCorrBuf!=NULL && forcewrite!=0)//Hmm - actually I'm not sure this does anything - since previously we may have decided not to write into corrBuf.
    FORCEWRITE(cstr->rtcCorrBuf)=forcewrite;
  if(cstr->rtcCalCorrBuf!=NULL && forcewrite!=0)
    FORCEWRITE(cstr->rtcCalCorrBuf)=forcewrite; 
  if(err==0 && cstr->windowMode==WINDOWMODE_GLOBAL)
    calcGlobalAdaptiveWindow(cstr);
  cstr->post.centroidMode=cstr->centroidMode;
  cstr->post.centroidModeArr=cstr->centroidModeArr;
  cstr->post.timestamp=cstr->timestamp;
  cstr->post.frameno=cstr->frameno;
  cstr->post.totPxls=cstr->totPxls;
  cstr->post.rtcCorrBuf=cstr->rtcCorrBuf;
  cstr->post.rtcCalCorrBuf=cstr->rtcCalCorrBuf;
  cstr->post.corrbuf=cstr->corrbuf;
  cstr->post.calcorrbuf=cstr->calcorrbuf;
  return 0;
}

/**
   Called when we've finished everything for this frame.
*/
int slopeComplete(void *centHandle){
  //Note, centNewParam could be called at the same time as this, by a different thread...
  CentStruct *cstr=(CentStruct*)centHandle;
  CentPostStruct *p=&cstr->post;
  if(p->centroidMode==CENTROIDMODE_CORRELATIONCOG || p->centroidModeArr!=NULL){//p->centroidMode==CENTROIDMODE_CORRELATIONGAUSSIAN){
    printf("complete %d %d\n",p->rtcCorrBuf->addRequired,(int)p->frameno);
    if(p->rtcCorrBuf!=NULL && p->rtcCorrBuf->addRequired)
      circAdd(p->rtcCorrBuf,p->corrbuf,p->timestamp,p->frameno);
    memset(p->corrbuf,0,sizeof(float)*p->totPxls);
    if(p->rtcCalCorrBuf!=NULL && p->rtcCalCorrBuf->addRequired)
      circAdd(p->rtcCalCorrBuf,p->calcorrbuf,p->timestamp,p->frameno);
    memset(p->calcorrbuf,0,sizeof(float)*p->totPxls);
  }
  return 0;
}
/*Uncomment if needed
int slopeNewFrame(void *centHandle,unsigned int frameno,double timestamp){
}
int slopeStartFrame(void *centHandle,int cam,int threadno){
}
int slopeEndFrame(void *centHandle,int cam,int threadno,int err){//subap thread (once per thread)
}
int slopeFrameFinished(void *centHandle,int err){//non-subap thread (once)
}
int slopeOpenLoop(void *centHandle){
}

*/
