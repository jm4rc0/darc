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
enum CentroidModes{CENTROIDMODE_WPU,CENTROIDMODE_COG,CENTROIDMODE_GAUSSIAN,CENTROIDMODE_CORRELATIONCOG,CENTROIDMODE_CORRELATIONGAUSSIAN,CENTROIDMODE_ERROR};
enum CorrelationThresholdType{CORR_ABS_SUB,CORR_ABS_ZERO,CORR_FRAC_SUB,CORR_FRAC_ZERO};


typedef enum{
  ADAPTIVEGROUP,
  ADAPTIVEWINGAIN,
  CENTCALBOUNDS,
  CENTCALDATA,
  CENTCALSTEPS,
  CENTROIDMODE,
  CENTROIDWEIGHT,
  CORRFFTPATTERN,
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
					 "adaptiveGroup",	\
					 "adaptiveWinGain",	\
					 "centCalBounds",	\
					 "centCalData",		\
					 "centCalSteps",	\
					 "centroidMode",	\
					 "centroidWeight",	\
					 "corrFFTPattern",	\
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
  int subapSize;
  int curnpxlx;
  int curnpxly;
  int curnpxl;
  int subindx;
  int centindx;
}CentThreadStruct;
typedef struct{
  enum CentroidModes centroidMode;
  double timestamp;
  unsigned int frameno;
  int totPxls;
  circBuf *rtcCorrBuf;
  float *corrbuf;
}CentPostStruct;

typedef struct{
  CentThreadStruct *tstr;
  float *adaptiveWinPos;
  int adaptiveWinPosSize;
  int *adaptiveCentPos;
  int adaptiveCentPosSize;
  float adaptiveWinGain;
  int maxAdapOffset;
  int *maxAdapOffsetArr;
  arrayStruct *arr;
  int ncam;
  int *nsub;
  int totCents;
  int nsubaps;
  int totPxls;
  int *nsubapCum;
  int *subapFlag;
  int*adaptiveGroup;
  float *groupSumX;
  float *groupSumY;
  int *groupSum;
  int nAdaptiveGroups;
  int adaptiveGroupSize;
  float *fftCorrelationPattern;//the spot PSF array, FFT'd in HC format, and placed as per subapLocation...
  int fftIndexSize;
  int *fftIndex;
  fftwf_plan *fftPlanArray;//array holding all the fftw plans
  pthread_mutex_t fftcreateMutex;
  int correlationThresholdType;
  float correlationThreshold;
  float *centCalData;
  float *centCalSteps;
  int *centCalBounds;
  int centCalnsteps;
  float *refCents;
  int *realSubapLocation;
  int *npxlCum;
  int *npxlx;
  int *npxly;
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
  char *prefix;
  circBuf *rtcCorrBuf;
  float *corrbuf;
  int corrbufSize;
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
  CentThreadStruct *tstr=&cstr->tstr[threadno];
  float *adaptiveWinPos=cstr->adaptiveWinPos;
  int *adaptiveCentPos=cstr->adaptiveCentPos;
  int centindx=tstr->centindx;
  float adaptiveWinGain=cstr->adaptiveWinGain;
  int maxAdapOffset;

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
  if(maxAdapOffset>0 || cstr->maxAdapOffsetArr!=NULL){
    if(adaptiveCentPos[centindx]>maxAdapOffset)
      adaptiveCentPos[centindx]=maxAdapOffset;
    else if(adaptiveCentPos[centindx]<-maxAdapOffset)
      adaptiveCentPos[centindx]=-maxAdapOffset;
    if(adaptiveCentPos[centindx+1]>maxAdapOffset)
      adaptiveCentPos[centindx+1]=maxAdapOffset;
    else if(adaptiveCentPos[centindx+1]<-maxAdapOffset)
      adaptiveCentPos[centindx+1]=-maxAdapOffset;
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
	if(adaptiveCentPos[i]>maxAdapOffset)
	  adaptiveCentPos[i]=maxAdapOffset;
	else if(adaptiveCentPos[i]<-maxAdapOffset)
	  adaptiveCentPos[i]=-maxAdapOffset;
	if(adaptiveCentPos[i+1]>maxAdapOffset)
	  adaptiveCentPos[i+1]=maxAdapOffset;
	else if(adaptiveCentPos[i+1]<-maxAdapOffset)
	  adaptiveCentPos[i+1]=-maxAdapOffset;
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
	  if(adaptiveCentPos[i]>maxAdapOffset)
	    adaptiveCentPos[i]=maxAdapOffset;
	  else if(adaptiveCentPos[i]<-maxAdapOffset)
	    adaptiveCentPos[i]=-maxAdapOffset;
	  if(adaptiveCentPos[i+1]>maxAdapOffset)
	    adaptiveCentPos[i+1]=maxAdapOffset;
	  else if(adaptiveCentPos[i+1]<-maxAdapOffset)
	    adaptiveCentPos[i+1]=-maxAdapOffset;
	}
      }
    }
  }
  return 0;
}


//Define a function to allow easy indexing into the fftCorrelationPattern array...
#define B(y,x) cstr->fftCorrelationPattern[cstr->npxlCum[threadno]+(loc[0]+y*loc[2])*cstr->npxlx[threadno]+loc[3]+x*loc[5]]
/**
   Calculates the correlation of the spot with the reference.
   fftCorrelationPattern is distributed in memory as per subapLocation, and is
   equal to numpy.conjugate(numpy.fft.fft2(numpy.fft.fftshift(corr)))
   where corr is the reference spot pattern (ie what you are correlating to).
   Should be stored in half complex form (reals then imags)
*/
int calcCorrelation(CentStruct *cstr,int threadno){
  CentThreadStruct *tstr=&cstr->tstr[threadno];
  int *loc;
  int i,j,n,m,neven,meven;
  float *a;
  float r1,r2,r3,r4,r5,r6,r7,r8;
  int *tmp;
  fftwf_plan fPlan=NULL,ifPlan=NULL;
  int curnpxlx=tstr->curnpxlx;
  int curnpxly=tstr->curnpxly;
  int cursubindx=tstr->subindx;
  float *subap=tstr->subap;
  //This is how the plans should be created (elsewhere).  Will need a different plan for each different sized subap (see subapLocation).  
  //fftwPlan=fftwf_plan_r2r_2d(curnpxly,curnpxlx, double *in, double *out,FFTW_R2HC, FFTW_R2HC, FFTW_ESTIMATE);
  //ifftwPlan=fftwf_plan_r2r_2d(curnpxly,curnpxlx, double *in, double *out,FFTW_HC2R, FFTW_HC2R, FFTW_ESTIMATE);
  for(i=0; i<cstr->fftIndexSize; i++){
    if(cstr->fftIndex[i*2]==0 || cstr->fftIndex[i*2+1]==0){
      break;
    }
    if(cstr->fftIndex[i*2]==curnpxlx && cstr->fftIndex[i*2+1]==curnpxly){
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
      if(cstr->fftIndex[i*2]==curnpxlx && cstr->fftIndex[i*2+1]==curnpxly){
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
      printf("Planning FFTs size %d x %d\n",curnpxly,curnpxlx);
      cstr->fftPlanArray[i*2]=fftwf_plan_r2r_2d(curnpxly,curnpxlx,subap,subap,FFTW_R2HC, FFTW_R2HC, FFTW_ESTIMATE);
      cstr->fftPlanArray[i*2+1]=fftwf_plan_r2r_2d(curnpxly,curnpxlx,subap,subap,FFTW_HC2R, FFTW_HC2R, FFTW_ESTIMATE);
      cstr->fftIndex[i*2]=curnpxlx;
      cstr->fftIndex[i*2+1]=curnpxly;
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
  loc=&(cstr->realSubapLocation[cursubindx*6]);
  a=subap;
  n=curnpxlx;
  m=curnpxly;


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
      a[(m-i)*n+n/2]=r7;
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
  return 0;
}

#undef B
/**
   Applies the chosen threshold algorithm to the correlation
   There are 4 possible thresholding algorithms.  The threshold is either a fixed value, or a fraction of the maximum value found in subap.  This threshold is then either subtracted with everything negative being zero'd, or anything below this threshold is zero'd.
*/
int thresholdCorrelation(CentStruct *cstr,int threadno){
  CentThreadStruct *tstr=&cstr->tstr[threadno];
  int i;
  float thresh=0.;
  float *subap=tstr->subap;
  int curnpxl=tstr->curnpxl;
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
      if(subap[i]<thresh)
	subap[i]=0;
      else
	subap[i]-=thresh;
    }
  }else if(cstr->correlationThresholdType==CORR_ABS_ZERO || cstr->correlationThresholdType==CORR_FRAC_ZERO){
    for(i=0; i<curnpxl; i++){
      if(subap[i]<thresh)
	subap[i]=0;
    }
  }
  return 0;
}
int storeCorrelationSubap(CentStruct *cstr,int threadno){
  CentThreadStruct *tstr=&cstr->tstr[threadno];
  int cnt=0;
  int i,j;
  int *loc;
  float *corrbuf=cstr->corrbuf;
  float *subap=tstr->subap;
  loc=&(cstr->arr->subapLocation[tstr->subindx*6]);

  for(i=loc[0]; i<loc[1]; i+=loc[2]){
    for(j=loc[3]; j<loc[4]; j+=loc[5]){
      corrbuf[cstr->npxlCum[threadno]+i*cstr->npxlx[threadno]+j]=subap[cnt];
      cnt++;
    }
  }
  return 0;
}

/**
   Apply a slope linearisation to the measured slope values.
   This has been taken from DASP, slightly modified for out of range values.
*/
int applySlopeLinearisation(CentStruct *cstr,int threadno,float *cx, float *cy){
  CentThreadStruct *tstr=&cstr->tstr[threadno];
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
  CentThreadStruct *tstr=&cstr->tstr[threadno];
  float sum=0.;
  int i,j;
  float cy,cx;
  float minflux;
  int centindx=tstr->centindx;
  float *subap=tstr->subap;
  int curnpxlx=tstr->curnpxlx;
  int curnpxly=tstr->curnpxly;
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
  if(cstr->centroidMode==CENTROIDMODE_CORRELATIONCOG || cstr->centroidMode==CENTROIDMODE_CORRELATIONGAUSSIAN){
    //do the correlation...
    calcCorrelation(cstr,threadno);
    //here, before thresholding, should probably store this in a circular buffer that can be sent to user.  Or maybe, this is the calibrated image buffer.
    storeCorrelationSubap(cstr,threadno);
    thresholdCorrelation(cstr,threadno);
  }

  if(cstr->centroidMode==CENTROIDMODE_COG || cstr->centroidMode==CENTROIDMODE_CORRELATIONCOG){
    for(i=0; i<curnpxly; i++){
      for(j=0; j<curnpxlx; j++){
	sum+=subap[i*curnpxlx+j];
	cx+=j*subap[i*curnpxlx+j];
	cy+=i*subap[i*curnpxlx+j];
      }
    }
    if(sum>minflux){
      cy/=sum;
      cx/=sum;
      cy-=curnpxly/2.-0.5;
      cx-=curnpxlx/2.-0.5;
    }else{
      cy=0;
      cx=0;
    }

  }else if(cstr->centroidMode==CENTROIDMODE_GAUSSIAN || cstr->centroidMode==CENTROIDMODE_CORRELATIONGAUSSIAN){
    //do some sort of gaussian fit to the data...
    printf("TODOxxx - gaussian fit to the data to get centroid (not yet implemented)\n");
  }else{
    printf("centroid mode not yet implemented\n");
  }
  if(sum>minflux){
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
  if((cstr->tstr=calloc(sizeof(CentThreadStruct),nthreads))==NULL){
    printf("Error allocating CentThreadStruct\n");
    slopeClose(centHandle);
    *centHandle=NULL;
    return 1;
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
    err=1;
    printf("Didn't get all buffer entries for rtcslope module:\n");
    for(i=0; i<NBUFFERVARIABLES; i++){
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
      printf("adaptiveWinGroup error: nbytes=%d should be %d type %c should be i\n",nb,sizeof(int)*cstr->totCents/2,dtype[ADAPTIVEGROUP]);
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
    }
    if(dtype[ADAPTIVEWINGAIN]=='f' && nbytes[ADAPTIVEWINGAIN]==sizeof(float)){
      cstr->adaptiveWinGain=*((float*)values[ADAPTIVEWINGAIN]);
    }else{
      err=1;
      printf("adaptiveWinGain error\n");
    }
    if(dtype[CORRTHRESHTYPE]=='i' && nbytes[CORRTHRESHTYPE]==sizeof(int)){
      cstr->correlationThresholdType=*((int*)values[CORRTHRESHTYPE]);
    }else{
      err=1;
      printf("correlationThresholdType error\n");
    }
    if(dtype[CORRTHRESH]=='f' && nbytes[CORRTHRESH]==sizeof(float)){
      cstr->correlationThreshold=*((float*)values[CORRTHRESH]);
    }else{
      err=1;
      printf("correlationThreshold error\n");
    }
    nb=nbytes[CORRFFTPATTERN];
    if(nb==0)
      cstr->fftCorrelationPattern=NULL;
    else if(dtype[CORRFFTPATTERN]=='f' && nb/sizeof(float)==cstr->totPxls){
      cstr->fftCorrelationPattern=(float*)values[CORRFFTPATTERN];
    }else{
      printf("fftCorrelationPattern error\n");
      err=1;
    }
    cstr->centCalnsteps=0;
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
    i=CENTCALSTEPS;
    nb=nbytes[CENTCALSTEPS];
    if(nb==0 || cstr->centCalnsteps==0)
      cstr->centCalSteps=NULL;
    else if(dtype[CENTCALSTEPS]=='f' && nb/sizeof(float)==cstr->totCents*cstr->centCalnsteps){
      cstr->centCalSteps=(float*)values[CENTCALSTEPS];
    }else{
      printf("centCalSteps error\n");
      err=1;
    }
    i=CENTCALBOUNDS;
    nb=nbytes[CENTCALBOUNDS];
    if(nb==0 || cstr->centCalnsteps==0)
      cstr->centCalBounds=NULL;
    else if(dtype[CENTCALBOUNDS]=='i' && nb/sizeof(int)==2*cstr->totCents){
      cstr->centCalBounds=(int*)values[CENTCALBOUNDS];
    }else{
      printf("centCalBounds error\n");
      err=1;
    }
    if(cstr->centCalData==NULL || cstr->centCalSteps==NULL || cstr->centCalBounds==NULL){
      cstr->centCalData=NULL;
      cstr->centCalSteps=NULL;
      cstr->centCalBounds=NULL;
    }
    nb=nbytes[CENTROIDMODE];
    if(nb!=0 && dtype[CENTROIDMODE]=='s'){
      //cstr->useWPU=0;
      if(strncmp(values[CENTROIDMODE],"WPU",nb)==0){
	//cstr->useWPU=1;
	cstr->centroidMode=CENTROIDMODE_WPU;
      }else if(strncmp(values[CENTROIDMODE],"CoG",nb)==0){
	cstr->centroidMode=CENTROIDMODE_COG;
      }else if(strncmp(values[CENTROIDMODE],"Gaussian",nb)==0){
	cstr->centroidMode=CENTROIDMODE_GAUSSIAN;
      }else if(strncmp(values[CENTROIDMODE],"CorrelationCoG",nb)==0){
	cstr->centroidMode=CENTROIDMODE_CORRELATIONCOG;
      }else if(strncmp(values[CENTROIDMODE],"CorrelationGaussian",nb)==0){
	cstr->centroidMode=CENTROIDMODE_CORRELATIONGAUSSIAN;
      }else{
	cstr->centroidMode=CENTROIDMODE_ERROR;
	printf("Unrecognised centroidMode\n");
      }
    }else{
      err=1;
      printf("centroidMode error\n");
    }
    if(cstr->centroidMode==CENTROIDMODE_CORRELATIONGAUSSIAN || cstr->centroidMode==CENTROIDMODE_CORRELATIONCOG){
      if(cstr->rtcCorrBuf==NULL){
	//open the circular buffer.
	char *tmp;
	if(asprintf(&tmp,"/%srtcCorrBuf",cstr->prefix)==-1)
	  exit(1);
	cstr->rtcCorrBuf=openCircBuf(tmp,1,&cstr->totPxls,'f',100);
	free(tmp);
      }else{
	if(cstr->rtcCorrBuf->datasize!=cstr->totPxls*sizeof(float)){
	  if(circReshape(cstr->rtcCorrBuf,1,&cstr->totPxls,'f')!=0){
	    printf("Error reshaping rtcCorrBuf\n");
	    err=1;
	  }
	}
      }
      if(cstr->corrbufSize<cstr->totPxls){
	if(cstr->corrbuf!=NULL)
	  free(cstr->corrbuf);
	cstr->corrbufSize=cstr->totPxls;
	if((cstr->corrbuf=malloc(sizeof(float)*cstr->totPxls))==NULL){
	  printf("malloc of corrbuf failed\n");
	  err=1;
	  cstr->corrbufSize=0;
	}
      }
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
    if(err==0 && resetAdaptiveWindows==1){
      memset(cstr->adaptiveCentPos,0,sizeof(int)*cstr->totCents);
      memset(cstr->adaptiveWinPos,0,sizeof(float)*cstr->totCents);
    }
  }

  return err;
}
/**
   Close a centroid camera of type name.  Args are passed in the float array of size n, and state data is in centHandle, which should be freed and set to NULL before returning.
*/

int slopeClose(void **centHandle){
  CentStruct *cstr=(CentStruct*)*centHandle;
  printf("closing rtcslope library %p %p\n",centHandle,cstr);
  if(cstr!=NULL){
    if(cstr->rtcCorrBuf!=NULL){
      circClose(cstr->rtcCorrBuf);
    }
    pthread_mutex_destroy(&cstr->fftcreateMutex);
    if(cstr->corrbuf!=NULL)
      free(cstr->corrbuf);
    if(cstr->paramNames!=NULL)
      free(cstr->paramNames);
    if(cstr->npxlCum!=NULL)
      free(cstr->npxlCum);
    if(cstr->tstr!=NULL)
      free(cstr->tstr);
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
  return 0;
}


/**
   If WPU - Wait for the next n subapertures of the current frame to arrive.
   Or, process the data (probably in subap, if you are using a standard calibration)
   Return 1 on error, 0 on okay or -1 if no slopes arrived, but this is not an error.
   The frameno can also be updated.
*/
int slopeCalcSlope(void *centHandle,int cam,int threadno,int nsubs,float *subap, int subapSize,int subindx,int centindx,int curnpxlx,int curnpxly){//subap thread.
  CentStruct *cstr=(CentStruct*)centHandle;
  CentThreadStruct *tstr=&cstr->tstr[threadno];
  tstr->subap=subap;
  tstr->subapSize=subapSize;
  tstr->subindx=subindx;
  tstr->centindx=centindx;
  tstr->curnpxlx=curnpxlx;
  tstr->curnpxly=curnpxly;
  calcCentroid(cstr,threadno);
  return 0;
}

int slopeFrameFinishedSync(void *centHandle,int err,int forcewrite){//subap thread (once)
  CentStruct *cstr=(CentStruct*)centHandle;
  if(cstr->rtcCorrBuf!=NULL && forcewrite!=0)
    FORCEWRITE(cstr->rtcCorrBuf)=forcewrite;
  if(err==0 && cstr->windowMode==WINDOWMODE_GLOBAL)
    calcGlobalAdaptiveWindow(cstr);
  cstr->post.centroidMode=cstr->centroidMode;
  cstr->post.timestamp=cstr->timestamp;
  cstr->post.frameno=cstr->frameno;
  cstr->post.totPxls=cstr->totPxls;
  cstr->post.rtcCorrBuf=cstr->rtcCorrBuf;
  cstr->post.corrbuf=cstr->corrbuf;
  return 0;
}

/**
   Called when we've finished everything for this frame.
*/
int slopeComplete(void *centHandle){
  //Note, centNewParam could be called at the same time as this, by a different thread...
  CentStruct *cstr=(CentStruct*)centHandle;
  CentPostStruct *p=&cstr->post;
  if(p->centroidMode==CENTROIDMODE_CORRELATIONCOG || p->centroidMode==CENTROIDMODE_CORRELATIONGAUSSIAN){
    circAdd(p->rtcCorrBuf,p->corrbuf,p->timestamp,p->frameno);
    memset(p->corrbuf,0,sizeof(float)*p->totPxls);
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
