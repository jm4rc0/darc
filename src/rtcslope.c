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
#include "agbcblas.h"
enum CentroidModes{CENTROIDMODE_COG,CENTROIDMODE_CORRELATIONCOG,CENTROIDMODE_GAUSSIAN,CENTROIDMODE_CORRELATIONGAUSSIAN,CENTROIDMODE_QUADRATIC,CENTROIDMODE_CORRELATIONQUADRATIC,CENTROIDMODE_DIFFSQUCOG,CENTROIDMODE_DIFFSQUGAUSSIAN,CENTROIDMODE_DIFFSQUQUADRATIC,CENTROIDMODE_CORRELATIONQUADRATICINTERP,CENTROIDMODE_DIFFSQUQUADRATICINTERP,CENTROIDMODE_QUADRATICINTERP,CENTROIDMODE_BRUTECOG,CENTROIDMODE_BRUTEGAUSSIAN,CENTROIDMODE_BRUTEQUADRATIC,CENTROIDMODE_BRUTEQUADRATICINTERP,CENTROIDMODE_ERROR};
enum CorrelationThresholdType{CORR_ABS_SUB,CORR_ABS_ZERO,CORR_FRAC_SUB,CORR_FRAC_ZERO};

typedef enum{
  ADAPBOUNDARY,
  ADAPRESETCOUNT,
  ADAPWINOFFSET,
  ADAPTIVEGROUP,
  ADAPTIVEWINGAIN,
  CENTCALBOUNDS,
  CENTCALDATA,
  CENTCALSTEPS,
  CENTINDEXARRAY,
  CENTROIDMODE,
  CENTROIDWEIGHT,
  CORRCLIP,
  CORRCLIPINTEGIMG,
  CORRFFTPATTERN,
  CORRIMGOFFSET,//an offset when creating integrated images, for non-symmetric spots.
  CORRNSTORE,
  CORRNPXLCUM,
  CORRNPXLX,
  CORRSUBAPLOCATION,
  CORRTHRESH,
  CORRTHRESHTYPE,
  CORRUPDATEGAIN,//gain when doing on-the-fly correlation ref updates.
  CORRUPDATENFR,
  CORRUPDATETOCOG,//shift to CoG centre or correlation centre.  Default=1 (CoG), 0 is slightly less computation, but possibly not so robust.
  FITMATRICES, //used for Gaussian/Quadratic fitting.
  FITSIZE,//for gaussian/quadratic fitting - the number of pixels around which to fit.
  FLUXTHRESHOLD,
  GAUSSMINVAL,
  GAUSSREPLACEVAL,
  MAXADAPOFFSET,
  NCAM,
  NCAMTHREADS,
  NPXLX,
  NPXLY,
  NSUB,
  PYRAMIDMODE,
  REFCENTROIDS,
  SUBAPALLOCATION,
  SUBAPFLAG,
  SUBAPLOCATION,
  WINDOWMODE,
  NBUFFERVARIABLES
}calibrateNames;

//char calibrateParamList[NBUFFERVARIABLES][16]={
#define makeParamNames() bufferMakeNames(NBUFFERVARIABLES,	\
					 "adapBoundary",	\
					 "adapResetCount",	\
					 "adapWinOffset",	\
					 "adaptiveGroup",	\
					 "adaptiveWinGain",	\
					 "centCalBounds",	\
					 "centCalData",		\
					 "centCalSteps",	\
					 "centIndexArray",	\
					 "centroidMode",	\
					 "centroidWeight",	\
					 "corrClip",		\
					 "corrClipIntegImg",	\
					 "corrFFTPattern",	\
					 "corrImgOffset",	\
					 "corrNStore",		\
					 "corrNpxlCum",		\
					 "corrNpxlx",		\
					 "corrSubapLoc",	\
					 "corrThresh",		\
					 "corrThreshType",	\
					 "corrUpdateGain",	\
					 "corrUpdateNfr",	\
					 "corrUpdateToCoG",	\
					 "fitMatrices",		\
					 "fitSize",		\
					 "fluxThreshold",	\
					 "gaussMinVal",		\
					 "gaussReplaceVal",	\
					 "maxAdapOffset",	\
					 "ncam",		\
					 "ncamThreads",		\
					 "npxlx",		\
					 "npxly",		\
					 "nsub",		\
					 "pyramidMode",		\
					 "refCentroids",	\
					 "subapAllocation",	\
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
  float *tmpSubap;
  int tmpSubapSize;
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
  int addReqCorr;
  int addReqCalCorr;
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
  int *adapBoundary;
  int adapResetCount;
  int *adapResetCountArr;
  int *adaptiveMaxCount;
  int adaptiveMaxCountSize;
  float *adapWinOffset;//an offset to specify the centre of strange shaped spots - where cog is far from centre.

  arrayStruct *arr;
  int ncam;
  int *nsub;
  int totCents;
  int nsubaps;
  int totPxls;
  int *nsubapCum;
  int *centIndxCum;
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
  circBuf *rtcCorrRefBuf;
  circBuf *rtcCentRefBuf;
  circBuf *rtcIntegratedImgBuf;
  float *calcorrbuf;
  int corrbufSize;
  float *centIndexArr;
  int centIndexSize;//1-4 if centroidIndexArr!=NULL.
  int addReqCorr;
  int addReqCalCorr;
  int addReqCorrRef;
  int addReqCentRef;
  int addReqIntegratedImg;
  float *fitMatrices;
  int fitsize;
  int nFitMatrices;
  float gaussReplaceVal;
  float gaussMinVal;
  float *rawSlopes;
  int rawSlopesSize;
  int *ncamThread;
  int *ncamThreadCum;
  int *subapAllocation;
  float corrUpdateGain;//When in correlation mode, set to >0 to automatically update the correlation reference every iteration.
  float lastCorrUpdateGain;
  float *integratedImg;//Stores the leaky box integrated image for correlation update.
  int integratedImgSize;//Stores the leaky box integrated image for correlation update.
  float *updatedRefCents;//updated reference slopes (updated with auto-correlation update).
  int updatedRefCentsSize;//updated reference slopes (updated with auto-correlation update).
  float *updatedCorrFFTPattern;//updated correlation pattern (fft-d in HC format)
  int shiftToCoG;//if 1 will shift and add based on a CoG, if 0 will use the correlation offset value.
  int clipIntegImg;//If 1, will clip the shift and add image.
  float *corrImgOffset;
  int updateOverNFrames;
  float *totalCamFlux;//for pyramid.  Mean flux of each quadrant summed, from previous frame, for each cam.
  pthread_barrier_t *barrier;
  //int updatedFFTCorrPatternSize;Not required - same as integratedImgSize.
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
  float *centroids=cstr->rawSlopes;//arr->centroids;
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
#define B(y,x) corrRef[cstr->corrnpxlCum[tstr->cam]+(loc[0]+(y)*loc[2])*cstr->corrnpxlx[tstr->cam]+loc[3]+(x)*loc[5]]
void calcDiffSquared(CentStruct *cstr,int threadno){
  CentThreadStruct *tstr=cstr->tstr[threadno];
  int *loc;
  int curnpxlx=tstr->curnpxlx;
  int curnpxly=tstr->curnpxly;
  //int cursubindx=tstr->subindx;
  float *subap=tstr->subap;
  int corrnpxlx,corrnpxly,corrClip;
  float *corrRef;
  int i,j,x,y;
  float s,d,tot,tot2,b,c;
  int mx,my;
  int nceny,ncenx;
  int mode=2;
  int minx,miny,srefy,srefx,offx,offy,sinx,siny;
  tstr->curnpxlSubap=curnpxlx*curnpxly;
  //The corrRef can be larger than the subap.  The output is then of size of the original subap, clipped if specified (to reduce computational load)
  if(cstr->corrClipArr!=NULL){corrClip=cstr->corrClipArr[tstr->subindx];
  }else
    corrClip=cstr->corrClip;
  corrRef=cstr->fftCorrelationPattern;
  if(cstr->corrSubapLocation!=NULL)
    loc=&(cstr->corrSubapLocation[tstr->subindx*6]);
  else
    loc=&(cstr->realSubapLocation[tstr->subindx*6]);
  corrnpxly=(loc[1]-loc[0])/loc[2];
  corrnpxlx=(loc[4]-loc[3])/loc[5];
  nceny=curnpxly-2*corrClip;
  ncenx=curnpxlx-2*corrClip;
  tstr->corrnpxlx=ncenx;//curnpxlx;//the correlation image won't be any bigger
  tstr->corrnpxly=nceny;//curnpxly;//than the shs image.
  //need some memory for the intermediate result...
  if(tstr->corrSubapSize<ncenx*nceny){//curnpxlx*curnpxly){
    if(tstr->corrSubap!=NULL){
      printf("Freeing existing corrSubap\n");
      free(tstr->corrSubap);
    }
    tstr->corrSubapSize=ncenx*nceny;//curnpxlx*curnpxly;
    printf("memaligning corrSubap to %dx%d\n",nceny,ncenx);
    if((i=posix_memalign((void**)(&(tstr->corrSubap)),SUBAPALIGN,sizeof(float)*tstr->corrSubapSize))!=0){//equivalent to fftwf_malloc... (kernel/kalloc.h in fftw source).
      tstr->corrSubapSize=0;
      tstr->corrSubap=NULL;
      printf("corrSubap re-malloc failed thread %d, size %d\nExiting...\n",threadno,tstr->corrSubapSize);
      exit(0);
    }
    printf("Aligned, address %p thread %d\n",tstr->corrSubap,threadno);
  }
  minx=corrnpxlx<curnpxlx?corrnpxlx:curnpxlx;
  miny=corrnpxly<curnpxly?corrnpxly:curnpxly;

  //memset(tstr->corrSubap,0,sizeof(float)*curnpxly*curnpxlx);
  if(mode==1){//best for solar
    for(i=0;i<nceny;i++){
      my=corrnpxly-abs(nceny/2-i);
      for(j=0;j<ncenx;j++){
	mx=corrnpxlx-abs(ncenx/2-j);
	s=0;
	for(y=0;y<my;y++){
	  for(x=0;x<mx;x++){
	    if(i<nceny/2){
	      if(j<ncenx/2){
		d=B(y,x)-subap[(nceny/2-i+y)*curnpxlx+ncenx/2-j+x];
	      }else{
		d=B(y,j-ncenx/2+x)-subap[(nceny/2-i+y)*curnpxlx+x];
	      }
	    }else{
	      if(j<ncenx/2){
		d=B(i-nceny/2+y,x)-subap[(y)*curnpxlx+ncenx/2-j+x];
	      }else{
		d=B(i-nceny/2+y,j-ncenx/2+x)-subap[(y)*curnpxlx+x];
	      }
	    }
	    s+=d*d;
	  }
	}
	//tstr->corrSubap[(i+(curnpxly-nceny)/2)*curnpxlx+j+(curnpxlx-ncenx)/2]=s/(my*mx);
	tstr->corrSubap[i*ncenx+j]=s/(my*mx);
      }
    }
  }else{//best for lgs (different scaling).
    for(i=0;i<nceny;i++){
      offy=i-nceny/2;//this offset.  0 for the central point.
      my=miny;
      srefy=-offy-(curnpxly-corrnpxly)/2;//starting point in the reference.
      if(srefy<0) srefy=0;
      siny=(curnpxly-corrnpxly)/2+offy;//starting point in the input image
      if(siny<0) siny=0;
      if(siny+my>curnpxly) my=curnpxly-siny;
      if(srefy+my>corrnpxly) my=corrnpxly-srefy;
      for(j=0;j<ncenx;j++){
	offx=j-ncenx/2;
	mx=minx;
	srefx=-offx-(curnpxlx-corrnpxlx)/2;//starting point in the refernce
	if(srefx<0) srefx=0;
	sinx=(curnpxlx-corrnpxlx)/2+offx;//starting point in the input image
	if(sinx<0) sinx=0;
	if(sinx+mx>curnpxlx) mx=curnpxlx-sinx;
	if(srefx+mx>corrnpxlx) mx=corrnpxlx-srefx;
	s=0;
	tot=0;
	tot2=0;
	for(y=0;y<my;y++){
	  for(x=0;x<mx;x++){
	    b=subap[(siny+y)*curnpxlx+sinx+x];
	    tot+=b;
	    c=B(srefy+y,srefx+x);//corrPattern[(srefy+y)*corrnpxl+srefx+x];
	    tot2+=c;
	    d=c-b;
	    s+=d*d;
	  }
	}
	if(tot!=0 && tot2!=0)
	  tstr->corrSubap[i*ncenx+j]=s/(tot*tot2);//[(i+(corrnpxly-nceny)/2)*corrnpxlx+j+(corrnpxlx-ncenx)/2]=s/(tot*tot2);
      }
    }
  }
  memcpy(subap,tstr->corrSubap,tstr->corrSubapSize*sizeof(float));
}
#undef B


#define B(y,x) corrRef[cstr->corrnpxlCum[tstr->cam]+(loc[0]+(y)*loc[2])*cstr->corrnpxlx[tstr->cam]+loc[3]+(x)*loc[5]]
void calcBruteCorr(CentStruct *cstr,int threadno){//brute force correlation - i.e. not using FFTs.
  CentThreadStruct *tstr=cstr->tstr[threadno];
  int *loc;
  int curnpxlx=tstr->curnpxlx;
  int curnpxly=tstr->curnpxly;
  //int cursubindx=tstr->subindx;
  float *subap=tstr->subap;
  int corrnpxlx,corrnpxly,corrClip;
  float *corrRef;
  int i,j,x,y;
  float s,tot,tot2,b,c;
  int mx,my;
  int nceny,ncenx;
  int minx,miny,srefy,srefx,offx,offy,sinx,siny;
  tstr->curnpxlSubap=curnpxlx*curnpxly;
  //The corrRef can be larger than the subap.  The output is then of size of the original subap, clipped if specified (to reduce computational load)
  if(cstr->corrClipArr!=NULL){corrClip=cstr->corrClipArr[tstr->subindx];
  }else
    corrClip=cstr->corrClip;
  corrRef=cstr->fftCorrelationPattern;
  if(cstr->corrSubapLocation!=NULL)
    loc=&(cstr->corrSubapLocation[tstr->subindx*6]);
  else
    loc=&(cstr->realSubapLocation[tstr->subindx*6]);
  corrnpxly=(loc[1]-loc[0])/loc[2];
  corrnpxlx=(loc[4]-loc[3])/loc[5];
  nceny=curnpxly-2*corrClip;
  ncenx=curnpxlx-2*corrClip;
  tstr->corrnpxlx=ncenx;//curnpxlx;//the correlation image won't be any bigger
  tstr->corrnpxly=nceny;//curnpxly;//than the shs image.
  //need some memory for the intermediate result...
  if(tstr->corrSubapSize<ncenx*nceny){//curnpxlx*curnpxly){
    if(tstr->corrSubap!=NULL){
      printf("Freeing existing corrSubap\n");
      free(tstr->corrSubap);
    }
    tstr->corrSubapSize=ncenx*nceny;//curnpxlx*curnpxly;
    printf("memaligning corrSubap to %dx%d\n",nceny,ncenx);
    if((i=posix_memalign((void**)(&(tstr->corrSubap)),SUBAPALIGN,sizeof(float)*tstr->corrSubapSize))!=0){//equivalent to fftwf_malloc... (kernel/kalloc.h in fftw source).
      tstr->corrSubapSize=0;
      tstr->corrSubap=NULL;
      printf("corrSubap re-malloc failed thread %d, size %d\nExiting...\n",threadno,tstr->corrSubapSize);
      exit(0);
    }
    printf("Aligned, address %p thread %d\n",tstr->corrSubap,threadno);
  }
  minx=corrnpxlx<curnpxlx?corrnpxlx:curnpxlx;
  miny=corrnpxly<curnpxly?corrnpxly:curnpxly;

  for(i=0;i<nceny;i++){
    offy=i-nceny/2;//this offset.  0 for the central point.
    my=miny;
    srefy=-offy-(curnpxly-corrnpxly)/2;//starting point in the reference.
    if(srefy<0) srefy=0;
    siny=(curnpxly-corrnpxly)/2+offy;//starting point in the input image
    if(siny<0) siny=0;
    if(siny+my>curnpxly) my=curnpxly-siny;
    if(srefy+my>corrnpxly) my=corrnpxly-srefy;
    for(j=0;j<ncenx;j++){
      offx=j-ncenx/2;
      mx=minx;
      srefx=-offx-(curnpxlx-corrnpxlx)/2;//starting point in the refernce
      if(srefx<0) srefx=0;
      sinx=(curnpxlx-corrnpxlx)/2+offx;//starting point in the input image
      if(sinx<0) sinx=0;
      if(sinx+mx>curnpxlx) mx=curnpxlx-sinx;
      if(srefx+mx>corrnpxlx) mx=corrnpxlx-srefx;
      s=0;
      tot=0;
      tot2=0;
      for(y=0;y<my;y++){
	for(x=0;x<mx;x++){
	  b=subap[(siny+y)*curnpxlx+sinx+x];
	  tot+=b;
	  c=B(srefy+y,srefx+x);//corrPattern[(srefy+y)*corrnpxl+srefx+x];
	  tot2+=c;
	  s+=c*b;
	}
      }
      //if(tot!=0 && tot2!=0)
      tstr->corrSubap[i*ncenx+j]=s;///(tot*tot2);
    }
  }
  memcpy(subap,tstr->corrSubap,tstr->corrSubapSize*sizeof(float));
}
#undef B


//Define a function to allow easy indexing into the fftCorrelationPattern array...
#define B(y,x) fftCorrelationPattern[cstr->corrnpxlCum[tstr->cam]+(loc[0]+(y)*loc[2])*cstr->corrnpxlx[tstr->cam]+loc[3]+(x)*loc[5]]
/**
   Calculates the correlation of the spot with the reference.
   fftCorrelationPattern is distributed in memory as per subapLocation, and is
   equal to numpy.conjugate(numpy.fft.fft2(numpy.fft.fftshift(corr)))
   where corr is the reference spot pattern (ie what you are correlating to).
   Should be stored in half complex form (reals then imags)
*/
int calcCorrelation(CentStruct *cstr,int threadno,float *fftOut){
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
  float *fftCorrelationPattern;
  //This is how the plans should be created (elsewhere).  Will need a different plan for each different sized subap (see subapLocation).  
  //fftwPlan=fftwf_plan_r2r_2d(curnpxly,curnpxlx, double *in, double *out,FFTW_R2HC, FFTW_R2HC, FFTW_ESTIMATE);
  //ifftwPlan=fftwf_plan_r2r_2d(curnpxly,curnpxlx, double *in, double *out,FFTW_HC2R, FFTW_HC2R, FFTW_ESTIMATE);
  if(cstr->corrClipArr!=NULL){
    corrClip=cstr->corrClipArr[tstr->subindx];
  }else
    corrClip=cstr->corrClip;
  if(cstr->corrUpdateGain==0 || fftOut!=NULL)//use the user defined pattern
    fftCorrelationPattern=cstr->fftCorrelationPattern;
  else//use the continuously updating pattern.
    fftCorrelationPattern=cstr->updatedCorrFFTPattern;


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
      if((i=posix_memalign((void**)(&(tstr->corrSubap)),SUBAPALIGN,sizeof(float)*tstr->corrSubapSize))!=0){//equivalent to fftwf_malloc... (kernel/kalloc.h in fftw source).
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
  if(fftOut!=NULL)
    memcpy(fftOut,subap,sizeof(float)*corrnpxlx*corrnpxly);
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
    if(corrClip>0 && fftOut==NULL){//only clip when fftOut is NULL (which is the usual case).  If its not NULL, it means we're doing a real-time reference update, which means that we don't want to clip.
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
  int fftBasedCorr=0;
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
  if(cstr->centroidMode==CENTROIDMODE_CORRELATIONCOG || cstr->centroidMode==CENTROIDMODE_CORRELATIONGAUSSIAN || cstr->centroidMode==CENTROIDMODE_CORRELATIONQUADRATIC || cstr->centroidMode==CENTROIDMODE_CORRELATIONQUADRATICINTERP)
    fftBasedCorr=1;
  if(fftBasedCorr==1 && (tstr->corrnpxlx>tstr->curnpxlx || tstr->corrnpxly>tstr->curnpxly)){
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
  }else if(fftBasedCorr==1){//corr image is same as subap size.
    subap=tstr->subap;
    corrbuf=&corrbuf[cstr->npxlCum[tstr->cam]];
    npx=cstr->npxlx[tstr->cam];
    for(i=loc[0]; i<loc[1]; i+=loc[2]){
      for(j=loc[3]; j<loc[4]; j+=loc[5]){
	corrbuf[i*npx+j]=subap[cnt];
	cnt++;
      }
    }
  }else{//corr image probably smaller than subap size.
    subap=tstr->subap;
    corrbuf=&corrbuf[cstr->npxlCum[tstr->cam]];
    npx=cstr->npxlx[tstr->cam];
    for(i=loc[0]+corrClip*loc[2]; i<loc[1]-corrClip*loc[2]; i+=loc[2]){
      for(j=loc[3]+corrClip*loc[5]; j<loc[4]-corrClip*loc[5]; j+=loc[5]){
	corrbuf[i*npx+j]=subap[cnt];
	cnt++;
      }
    }
    for(i=loc[0];i<loc[0]+corrClip*loc[2];i+=loc[2]){
      for(j=loc[3];j<loc[3]+corrClip*loc[5];j+=loc[5])
	corrbuf[i*npx+j]=0;
      for(j=loc[4]-corrClip*loc[5];j<loc[4];j+=loc[5])
	corrbuf[i*npx+j]=0;
    }
    for(i=loc[1]-corrClip*loc[2];i<loc[1];i+=loc[2]){
      for(j=loc[3];j<loc[3]+corrClip*loc[5];j+=loc[5])
	corrbuf[i*npx+j]=0;
      for(j=loc[4]-corrClip*loc[5];j<loc[4];j+=loc[5])
	corrbuf[i*npx+j]=0;
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
  int xmin,ymin,xmax,ymax;
  if(cstr->adapBoundary!=NULL){
    ymin=cstr->adapBoundary[0];
    ymax=cstr->adapBoundary[1];
    xmin=cstr->adapBoundary[2];
    xmax=cstr->adapBoundary[3];
  }else{
    xmin=0;
    xmax=cstr->npxlx[0];
    ymin=0;
    ymax=cstr->npxly[0];
  }
  //now update subapLocation for next time...
  i=0;
  //for(i=0; i<threadInfo->nsubapsProcessing; i++){
  for(i=0; i<cstr->nsubaps; i++){
    while(cstr->nsubapCum[cam+1]==i){//have got to end of subaps for this camera.
      cam++;
      if(cstr->adapBoundary!=NULL){
	ymin=cstr->adapBoundary[cam*4+0];
	ymax=cstr->adapBoundary[cam*4+1];
	xmin=cstr->adapBoundary[cam*4+2];
	xmax=cstr->adapBoundary[cam*4+3];;
      }else{
	xmin=0;
	xmax=cstr->npxlx[cam];
	ymin=0;
	ymax=cstr->npxly[cam];
      }
    }
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
      while(loc[0]<ymin){
	loc[0]+=rloc[2];
	loc[1]+=rloc[2];
	cstr->adaptiveCentPos[2*cnt+1]++;
      }
      while(loc[1]>ymax){
	loc[1]-=rloc[2];
	loc[0]-=rloc[2];
	cstr->adaptiveCentPos[2*cnt+1]--;
      }
      while(loc[3]<xmin){
	loc[3]+=rloc[5];
	loc[4]+=rloc[5];
	cstr->adaptiveCentPos[2*cnt]++;
      }
      while(loc[4]>xmax){
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


inline void makeFitVector(float *vec,float *subap,int nx,int ny,int fitsize,int startx,int starty){
  //vec should have size [6].
  int x,y;
  float val;
  for(y=0;y<6;y++)//probably faster than memset.
    vec[y]=0;
  for(y=0;y<fitsize;y++){
    for(x=0;x<fitsize;x++){
      val=subap[(y+starty)*nx+x+startx];
      vec[0]+=val*x*x;
      vec[1]+=val*x*y;
      vec[2]+=val*y*y;
      vec[3]+=val*x;
      vec[4]+=val*y;
      vec[5]+=val;
    }
  }
}

/*Used during auto-correlation update*/
void shiftAndIntegrateImage(CentStruct *cstr,int threadno,float *subap,int imgnx,int imgny,int cx,int cy){
  //Shift subap by cx, cy and put into the output.
  //Note, subap is likely to be smaller than the output.  We assume that it is not larger than the output.
  CentThreadStruct *tstr=cstr->tstr[threadno];
  int nx,ny,cnx,cny,i,j,ssx,ssy,sx,sy,indx;
  int *loc;
  float gain=cstr->corrUpdateGain;
  int imageWidth=cstr->corrnpxlx[tstr->cam];
  float *out=&cstr->integratedImg[cstr->corrnpxlCum[tstr->cam]];
  if(cstr->corrSubapLocation!=NULL)
    loc=&(cstr->corrSubapLocation[tstr->subindx*6]);
  else
    loc=&(cstr->realSubapLocation[tstr->subindx*6]);
  //if cx>0, shift image to left.  If cx<0, shift to right.
  ny=(loc[1]-loc[0])/loc[2];
  nx=(loc[4]-loc[3])/loc[5];
  if(cstr->lastCorrUpdateGain==0){
    //Reset the integrated image to 0 (and set gain to 1 so that all of current image gets put in there).
    gain=1;
  }
  for(i=0;i<ny;i++){
    for(j=0;j<nx;j++){
      out[(loc[0]+loc[2]*i)*imageWidth + loc[3]+loc[5]*j]*=1-gain;
    }
  }
  //integrate the image.
  cnx=imgnx;//number of pixels in x to integrate over
  cny=imgny;//May be reduced by clipping.
  if(cx>=0){//shift image left
    ssx=(nx-imgnx)/2-cx;//start in the output array.
    if(ssx<0){
      sx=-ssx;//start point in tmpsubap
      cnx=imgnx+ssx;//note: ssx is negative.
      ssx=0;
    }else{
      sx=0;//start point in tmpsubap
    }
  }else{//shift image right
    sx=0;
    ssx=(nx-imgnx)/2-cx;//note: cx is -ve.
    if(ssx+imgnx>nx){
      cnx=imgnx-(ssx+imgnx-nx);
    }
  }
  
  if(cy>=0){//shift image left
    ssy=(ny-imgny)/2-cy;//start in the output array.
    if(ssy<0){
      sy=-ssy;//start point in tmpsubap
      cny=imgny+ssy;//note: ssy is negative.
      ssy=0;
    }else{
      sy=0;//start point in tmpsubap
    }
  }else{//shift image right
    sy=0;
    ssy=(ny-imgny)/2-cy;//note: cy is -ve.
    if(ssy+imgny>ny){
      cny=imgny-(ssy+imgny-ny);
    }
  }
  if(cstr->clipIntegImg){
    int *loc2=&cstr->realSubapLocation[tstr->subindx*6];
    int inx=(loc2[4]-loc2[3])/loc2[5];
    int iny=(loc2[1]-loc2[0])/loc2[2];
    //Get the padding - how much larger are the correlation images than the real image.
    int padx=(nx-inx)/2;
    int pady=(ny-iny)/2;
    if(ssx<padx){
      cnx-=(padx-ssx);
      sx+=padx-ssx;
      ssx=padx;
    }
    if(ssx+cnx>nx-padx){
      cnx=nx-padx-ssx;
    }
    if(ssy<pady){
      cny-=(pady-ssy);
      sy+=pady-ssy;
      ssy=pady;
    }
    if(ssy+cny>ny-pady){
      cny=ny-pady-ssy;
    }
  }
  for(i=0;i<cny;i++){
    for(j=0;j<cnx;j++){
      indx=(loc[0]+loc[2]*(i+ssy))*imageWidth + loc[3]+loc[5]*(j+ssx);
      //out[indx]*=1-gain;//Moved earlier, so that whole image gets it, not just the part being updated.  Not entirely sure which is correct!
      out[indx]+=gain*subap[(i+sy)*imgnx+j+sx];
    }
  }
}

/*Used during auto-correlation update*/
void setSubapDeltaFn(CentStruct *cstr,int cam,int threadno, int subapNo,int centindx){
  int *aloc,*loc;
  int i,j,nx,ny;
  float *integratedImg=&cstr->integratedImg[cstr->corrnpxlCum[cam]];
  int imageWidth=cstr->corrnpxlx[cam];
  //first copy reference centroids.
  if(cstr->refCents==NULL){
    cstr->updatedRefCents[centindx]=0;
    cstr->updatedRefCents[centindx+1]=0;
  }else{
    cstr->updatedRefCents[centindx]=cstr->refCents[centindx];
    cstr->updatedRefCents[centindx+1]=cstr->refCents[centindx+1];
  }
  if(cstr->corrSubapLocation!=NULL)
    aloc=cstr->corrSubapLocation;
  else
    aloc=cstr->realSubapLocation;
  loc=&aloc[subapNo*6];
  ny=(loc[1]-loc[0])/loc[2];
  nx=(loc[4]-loc[3])/loc[5];
  //clear info
  for(i=0;i<ny;i++){
    for(j=0;j<nx;j++){
      integratedImg[(loc[0]+loc[2]*i)*imageWidth + loc[3]+loc[5]*j]=0;
    }
  }
  //And set subaps to a delta function.
  integratedImg[(loc[0]+loc[2]*(ny/2))*imageWidth + loc[3]+loc[5]*(nx/2)]=1;

}  

/*Used during auto-correlation update*/
int updateCorrReference(CentStruct *cstr,int cam,int threadno,int subapNo,int centNo){
  //Here, we have a new correlation reference.
  //So, for each sub-aperture:
  //1. Do the fft of integratedImg.
  //2. Correlate integratedImg with existing (user supplied) reference.
  //3. Compute refSlope update
  //4. Update reference slope
  //5. UPdate correlation reference image.
  //6. Publish to telemetry streams.
  int i,j,ox,oy,ii,oi,mm,nn;
  int *aloc,*loc;
  int ny,nx;
  float sum=0,cx=0,cy=0;
  CentThreadStruct *tstr=cstr->tstr[threadno];
  float *corrbuf=&cstr->updatedCorrFFTPattern[cstr->corrnpxlCum[cam]];
  float *integratedImg=&cstr->integratedImg[cstr->corrnpxlCum[cam]];
  int imageWidth=cstr->corrnpxlx[cam];
  int indx,indx2,cnt;
  float *subap;
  if(cstr->corrSubapLocation!=NULL)
    aloc=cstr->corrSubapLocation;
  else
    aloc=cstr->realSubapLocation;
  loc=&aloc[subapNo*6];
  ny=(loc[1]-loc[0])/loc[2];
  nx=(loc[4]-loc[3])/loc[5];
  
  tstr->curnpxlx=nx;
  tstr->curnpxly=ny;
  tstr->curnpxl=nx*ny;
  tstr->subindx=subapNo;
  tstr->centindx=centNo;
  tstr->cam=cam;
  if(tstr->corrSubapSize<nx*ny){
    if(tstr->corrSubap!=NULL){
      printf("Freeing existing corrSubap\n");
      free(tstr->corrSubap);
    }
    tstr->corrSubapSize=nx*ny;
    printf("memaligning corrSubap to %dx%d\n",ny,nx);
    if((i=posix_memalign((void**)(&(tstr->corrSubap)),SUBAPALIGN,sizeof(float)*tstr->corrSubapSize))!=0){//equivalent to fftwf_malloc... (kernel/kalloc.h in fftw source).
      tstr->corrSubapSize=0;
      tstr->corrSubap=NULL;
      printf("corrSubap re-malloc failed in updateCorrReference thread %d, size %d\nExiting...\n",threadno,tstr->corrSubapSize);
      exit(0);
    }
    printf("updateCorrRef Aligned, address %p thread %d\n",tstr->corrSubap,threadno);
  }
  tstr->corrnpxlx=nx;
  tstr->corrnpxly=ny;
  tstr->curnpxlSubap=nx*ny;
  if(tstr->tmpSubapSize<nx*ny){
    if(tstr->tmpSubap!=NULL){
      printf("Freeing existing tmpSubap\n");
      free(tstr->tmpSubap);
    }
    tstr->tmpSubapSize=nx*ny;
    printf("memaligning tmpSubap to %dx%d\n",ny,nx);
    if((i=posix_memalign((void**)(&(tstr->tmpSubap)),SUBAPALIGN,sizeof(float)*tstr->tmpSubapSize))!=0){//equivalent to fftwf_malloc... (kernel/kalloc.h in fftw source).
      tstr->tmpSubapSize=0;
      tstr->tmpSubap=NULL;
      printf("tmpSubap re-malloc failed in updateCorrReference thread %d, size %d\nExiting...\n",threadno,tstr->tmpSubapSize);
      exit(0);
    }
  }

  tstr->subap=tstr->tmpSubap;
  //Copy data into subap, fftshifted.
  for(i=0;i<ny;i++){
    for(j=0;j<nx;j++){
      indx=(loc[0]+loc[2]*i)*imageWidth + loc[3]+loc[5]*j;
      indx2=((i+ny/2)%ny)*nx+(j+nx/2)%nx;
      tstr->tmpSubap[indx2]=integratedImg[indx];
    }
  }

  calcCorrelation(cstr,threadno,tstr->corrSubap);
  //compute centroid of the correlation.  Note, have to fftshift back.  Do this just by changing indexes.
  subap=tstr->subap;
  ox=nx/2;
  oy=ny/2;
  cnt=0;
  for(i=0; i<ny; i++){
    for(j=0; j<nx; j++){
      sum+=subap[cnt];//i*curnpxlx+j];
      cx+=((j+ox)%nx)*subap[cnt];//i*curnpxlx+j];
      cy+=((i+oy)%ny)*subap[cnt];//i*curnpxlx+j];
      cnt++;
    }
  }
  if(sum!=0){
    cy/=sum;
    cx/=sum;
    cy-=ny/2.;//-0.5;//correlation is pixel centred, not 2x2 centred.
    cx-=nx/2.;//-0.5;
  }else{
    cy=0;
    cx=0;
  }
  //Copy from tmpfft to the correlation ref, after conjugate.
  //The bottom left and top right quadrants remain unchanged, but the other 2 should be multiplied by -1.
  mm=ny/2+1;
  nn=nx/2+1;
  subap=tstr->corrSubap;//now use the FFT'd mean image.
  for(i=0;i<mm;i++){
    ii=i*nx;
    oi=(loc[0]+i*loc[2])*cstr->corrnpxlx[cam];
    for(j=0;j<nn;j++)//bottom left - a real quadrant, copy unchanged
      corrbuf[oi+loc[3]+j*loc[5]]=subap[ii+j];
    for(j=nn;j<nx;j++)//bottom right - an imag quadrant, copy *-1
      corrbuf[oi+loc[3]+j*loc[5]]=-subap[ii+j];
  }
  for(i=mm;i<ny;i++){
    ii=i*nx;
    oi=(loc[0]+i*loc[2])*cstr->corrnpxlx[cam];
    for(j=0;j<nn;j++)//top left - an imag quadrant, copy *-1
      corrbuf[oi+loc[3]+j*loc[5]]=-subap[ii+j];
    for(j=nn;j<nx;j++)//top right - a real quadrant, copy unchanged
      corrbuf[oi+loc[3]+j*loc[5]]=subap[ii+j];
  }

  //Update ref slopes.
  //Subtract cy,cx from refCentroids.
  //BUT: There is a problem here.  If I use the refCents array, then whenever a buffer swap is done, the original will be copied back, and problems arise.  So, need to have a copy of refCents here, and a flag to specify when to actually copy from the user supplied refCents.
  //cstr->updatedRefCents[centNo]-=cx;
  //cstr->updatedRefCents[centNo+1]-=cy;
  //The correlation of new ref images with the user supplied image has been done, so now compute new slopes relative to that.
  //if(cstr->lastCorrUpdateGain!=0){
  if(cstr->refCents!=NULL){
    cstr->updatedRefCents[centNo]=cstr->refCents[centNo]-cx;
    cstr->updatedRefCents[centNo+1]=cstr->refCents[centNo+1]-cy;
  }else{
    cstr->updatedRefCents[centNo]=-cx;
    cstr->updatedRefCents[centNo+1]=-cy;
  }
    //}
  //publish telemetry - done in frameFinishedSync.
  return 0;
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
  int origSubapX,origSubapY;
  int corrUpdateRequired=0;
  float *adapWinOffset=cstr->adapWinOffset;//this gets set to NULL if in correlation mode.  
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
  if(centroidMode==CENTROIDMODE_CORRELATIONCOG || centroidMode==CENTROIDMODE_CORRELATIONGAUSSIAN || centroidMode==CENTROIDMODE_CORRELATIONQUADRATIC || centroidMode==CENTROIDMODE_CORRELATIONQUADRATICINTERP){
    adapWinOffset=NULL;//correlation should return a position defined by the correlation image... so an offset shouldn't be used.  And this only affects adaptive window position, not anything else.
    if(cstr->corrUpdateGain!=0){
      corrUpdateRequired=1;
      origSubapX=tstr->curnpxlx;//store these so we can shift-add the image later.  Since curnpxlx will change if correlation padding.
      origSubapY=tstr->curnpxly;
      if(tstr->tmpSubapSize<tstr->curnpxlx*tstr->curnpxly){
	tstr->tmpSubapSize=tstr->curnpxlx*tstr->curnpxly;
	free(tstr->tmpSubap);
	if(posix_memalign((void**)(&(tstr->tmpSubap)),SUBAPALIGN,sizeof(float)*tstr->tmpSubapSize)!=0){
	  printf("Error allocing tmpsubap\n");
	  tstr->tmpSubapSize=0;
	}
      }
      //save it - because subap can (and probably will) be replaced by the correlated function.
      memcpy(tstr->tmpSubap,subap,sizeof(float)*tstr->tmpSubapSize);
    }
    //do the correlation...
    calcCorrelation(cstr,threadno,NULL);
    //here, before thresholding, should probably store this in a circular buffer that can be sent to user.  Or maybe, this is the calibrated image buffer.
    if(cstr->rtcCorrBuf!=NULL && cstr->addReqCorr){// && cstr->rtcCorrBuf->addRequired){
      storeCorrelationSubap(cstr,threadno,cstr->corrbuf);
    }
    thresholdCorrelation(cstr,threadno);
    if(cstr->rtcCalCorrBuf!=NULL && cstr->addReqCalCorr)
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
  }else if(centroidMode==CENTROIDMODE_DIFFSQUCOG || centroidMode==CENTROIDMODE_DIFFSQUGAUSSIAN || centroidMode==CENTROIDMODE_DIFFSQUQUADRATIC || centroidMode==CENTROIDMODE_DIFFSQUQUADRATICINTERP){
    //int corrClip;
    calcDiffSquared(cstr,threadno);
    if(cstr->rtcCorrBuf!=NULL && cstr->addReqCorr)
      storeCorrelationSubap(cstr,threadno,cstr->corrbuf);
    thresholdCorrelation(cstr,threadno);
    if(cstr->rtcCalCorrBuf!=NULL && cstr->addReqCalCorr)
      storeCorrelationSubap(cstr,threadno,cstr->calcorrbuf);
    //if(cstr->corrClipArr!=NULL){
    //  corrClip=cstr->corrClipArr[tstr->subindx];
    //}else
    //  corrClip=cstr->corrClip;
    
    subap=tstr->subap;
    curnpxlx=tstr->corrnpxlx;//-2*corrClip;
    curnpxly=tstr->corrnpxly;//-2*corrClip;
    
  }else if (centroidMode==CENTROIDMODE_BRUTECOG || centroidMode==CENTROIDMODE_BRUTEGAUSSIAN || centroidMode==CENTROIDMODE_BRUTEQUADRATIC || centroidMode==CENTROIDMODE_BRUTEQUADRATICINTERP){//compute brute force correlation (not fft)
    calcBruteCorr(cstr,threadno);
    if(cstr->rtcCorrBuf!=NULL && cstr->addReqCorr)
      storeCorrelationSubap(cstr,threadno,cstr->corrbuf);
    thresholdCorrelation(cstr,threadno);
    if(cstr->rtcCalCorrBuf!=NULL && cstr->addReqCalCorr)
      storeCorrelationSubap(cstr,threadno,cstr->calcorrbuf);
    subap=tstr->subap;
    curnpxlx=tstr->corrnpxlx;//-2*corrClip;
    curnpxly=tstr->corrnpxly;//-2*corrClip;
    
  }
  if(centroidMode==CENTROIDMODE_COG || centroidMode==CENTROIDMODE_CORRELATIONCOG || centroidMode==CENTROIDMODE_DIFFSQUCOG || centroidMode==CENTROIDMODE_BRUTECOG){
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
      int pos,ypos;
      //int loc54;
      float cres[4];//[0] is cy, [1] is cx, [2] is sumy, [3] is sumx
      int k;
      if(centroidMode==CENTROIDMODE_DIFFSQUCOG || centroidMode==CENTROIDMODE_BRUTECOG)
	printf("NOT YET IMPLEMENTED: centIndexArr with DIFF-SQUARED or BRUTE correlation - segemtation fault is likely!\n");//to fix at a later date!
      if(cstr->corrSubapLocation==NULL)
	loc=&cstr->realSubapLocation[tstr->subindx*6];
      else
	loc=&cstr->corrSubapLocation[tstr->subindx*6];
      //loc54=loc[5]*cstr->centIndexSize;
      for(i=0; i<4; i++)
	cres[i]=0;
      for(i=loc[0]; i<loc[1]; i+=loc[2]){
	ypos=(cstr->corrnpxlCum[tstr->cam]+i*cstr->corrnpxlx[tstr->cam])*cstr->centIndexSize;
	for(j=loc[3]; j<loc[4]; j+=loc[5]){
	  pos=ypos+j*cstr->centIndexSize;
	  for(k=0;k<cstr->centIndexSize;k++)//NOTE: 1<=centIndexSize<=4
	    cres[k]+=subap[cnt]*cstr->centIndexArr[pos+k];
	  for(k=cstr->centIndexSize;k<4;k++)//note, k>=1
	    cres[k]+=subap[cnt]*((k==1)*j+(k>1));//no index arrays provided for these ones.
	  cnt++;
	}
      }
      //Looks slightly strange way of doing it, but this way, matched filter can also be used - when centIndexArr[2 and 3] are all zeros, so cres[2,3]==0, if set minflux to less than zero.  (or if want the flux scaling have centIndexArr having only [0 and 1].
      if(cstr->totalCamFlux==NULL){
	sum=cres[2];
	if(sum>=minflux){
	  if(sum!=0)
	    cy=cres[0]/sum;
	  else
	    cy=cres[0];
	}else{
	  cy=0;
	}
	sum=cres[3];
	if(sum>=minflux){
	  if(sum!=0)
	    cx=cres[1]/sum;
	  else
	    cx=cres[1];
	}else{
	  cx=0;
	}
      }else{//used for pyramid - needs to calibrate using total flux of the image from prev frame. 
      //for pyr, division by total flux from previous frame here... (todo).  ie cy=cres[0]/totflux, cx=cres[1]/totflux.  But for now, not needed - see the part above without centIndexArr.
	sum=cstr->totalCamFlux[tstr->cam];
	if(sum>=minflux && sum!=0){
	  cy=cres[0]/sum;
	  cx=cres[1]/sum;
	  //cy+=curnpxly/2.-0.5;
	  //cx+=curnpxlx/2.-0.5;
	}else{
	  cy=0;
	  cx=0;
	}
	sum=cres[2];
      }
      
      //don't subtract an offset here, this can be done by the refCentroids.
    }
  }else if(centroidMode==CENTROIDMODE_GAUSSIAN || centroidMode==CENTROIDMODE_CORRELATIONGAUSSIAN || centroidMode==CENTROIDMODE_DIFFSQUGAUSSIAN || centroidMode==CENTROIDMODE_BRUTEGAUSSIAN){
    //do some sort of gaussian fit to the data...
    float vec[6];
    float res[5];
    int mxpos=0;
    //find the minimum or maximum of the parabola (depending on mode).
    float mx=subap[0];
    if(centroidMode==CENTROIDMODE_DIFFSQUGAUSSIAN){//find min.
      for(i=0;i<curnpxlx*curnpxly;i++){
	sum+=subap[i];
	if(subap[i]<mx){
	  mxpos=i;
	  mx=subap[i];
	}
	if(subap[i]<=cstr->gaussMinVal)
	  subap[i]=cstr->gaussReplaceVal;
	else
	  subap[i]=logf(subap[i]);
      }
    }else{//find max
      for(i=0;i<curnpxlx*curnpxly;i++){
	sum+=subap[i];
	if(subap[i]>mx){
	  mxpos=i;
	  mx=subap[i];
	}
	if(subap[i]<=cstr->gaussMinVal)
	  subap[i]=cstr->gaussReplaceVal;
	else
	  subap[i]=logf(subap[i]);
      }
    }
    if(sum>=minflux){
      int fitsize=cstr->fitsize;//number of pixels to fit to.  3 is good!
      int startx=(int)(mxpos%curnpxlx-fitsize/2);
      int starty=(int)(mxpos/curnpxlx-fitsize/2);
      if(startx<0)
	startx=0;
      if(starty<0)
	starty=0;
      if(startx>curnpxlx-fitsize)
	startx=curnpxlx-fitsize;
      if(starty>curnpxly-fitsize)
	starty=curnpxly-fitsize;
      makeFitVector(vec,subap,curnpxlx,curnpxly,fitsize,startx,starty);//this won't work yet...
      //dot vector with the matrix.
      for(i=0;i<cstr->nFitMatrices;i++){
	if(cstr->fitMatrices[i*32+30]==fitsize && cstr->fitMatrices[i*32+31]==fitsize){
	  //found a suitable one.
	  break;
	}
      }
      if(i<cstr->nFitMatrices){
	agb_cblas_sgemvRowMN1N101(5,6,&cstr->fitMatrices[i*32],vec,res);
	cx=-res[3]/(2*res[0]);
	cy=-res[4]/(2*res[2]);
	//cy-=curnpxly/2.-0.5;
	//cx-=curnpxlx/2.-0.5;
	cx+=startx+curnpxlx/2.-tstr->curnpxlx/2.;
	cy+=starty+curnpxly/2.-tstr->curnpxly/2.;
      
      }else{
	printf("Error - suitable fitting matrix not found - please supply...\n");
	cy=0;
	cx=0;
	sum=0;
      }
    }
  }else if(centroidMode==CENTROIDMODE_QUADRATIC || centroidMode==CENTROIDMODE_CORRELATIONQUADRATIC || centroidMode==CENTROIDMODE_DIFFSQUQUADRATIC || centroidMode==CENTROIDMODE_QUADRATICINTERP || centroidMode==CENTROIDMODE_CORRELATIONQUADRATICINTERP || centroidMode==CENTROIDMODE_DIFFSQUQUADRATICINTERP || centroidMode==CENTROIDMODE_BRUTEQUADRATIC  || centroidMode==CENTROIDMODE_BRUTEQUADRATICINTERP ){
    float vec[6];
    float res[5];
    int mxpos=0;
    //find the minimum or maximum of the parabola (depending on mode).
    float mx=subap[0];
    if(centroidMode==CENTROIDMODE_DIFFSQUQUADRATIC || centroidMode==CENTROIDMODE_DIFFSQUQUADRATICINTERP){//find min.
      for(i=0;i<curnpxlx*curnpxly;i++){
	sum+=subap[i];
	if(subap[i]<mx){
	  mxpos=i;
	  mx=subap[i];
	}
      }
    }else{//find max
      for(i=0;i<curnpxlx*curnpxly;i++){
	sum+=subap[i];
	if(subap[i]>mx){
	  mxpos=i;
	  mx=subap[i];
	}
      }
    }
    
    if(sum>=minflux){
      int fitsize=cstr->fitsize;//number of pixels to fit to.  3 is good!
      int startx=(int)(mxpos%curnpxlx-fitsize/2);
      int starty=(int)(mxpos/curnpxlx-fitsize/2);
      if(startx<0)
	startx=0;
      if(starty<0)
	starty=0;
      if(startx>curnpxlx-fitsize)
	startx=curnpxlx-fitsize;
      if(starty>curnpxly-fitsize)
	starty=curnpxly-fitsize;
      if(centroidMode==CENTROIDMODE_QUADRATIC || centroidMode==CENTROIDMODE_CORRELATIONQUADRATIC || centroidMode==CENTROIDMODE_DIFFSQUQUADRATIC || centroidMode==CENTROIDMODE_BRUTEQUADRATIC){
	makeFitVector(vec,subap,curnpxlx,curnpxly,fitsize,startx,starty);
	//dot vector with the matrix.
	for(i=0;i<cstr->nFitMatrices;i++){
	  if(cstr->fitMatrices[i*32+30]==fitsize && cstr->fitMatrices[i*32+31]==fitsize){
	    //found a suitable one.
	    break;
	  }
	}
	if(i<cstr->nFitMatrices){
	  agb_cblas_sgemvRowMN1N101(5,6,&cstr->fitMatrices[i*32],vec,res);
	  cx=(res[1]*res[4]/(2*res[2])-res[3])/(2.*res[0]-res[1]*res[1]/(2.*res[2]));
	  cy=-(res[4]+res[1]*cx)/(2.*res[2]);//changed - to + on 150821
	  cx+=startx+curnpxlx/2.-tstr->curnpxlx/2.;
	  cy+=starty+curnpxlx/2.-tstr->curnpxlx/2.;
	}else{
	  printf("Error - suitable fitting matrix not found (%d) - please supply...\n",fitsize);
	  cy=0;
	  cx=0;
	  sum=0;
	}
      }else{//do quadratic interpolation (cf Lofsdahl paper) must be 3x3
	float a2,a3,a4,a5,a6;
	float *img=&subap[starty*curnpxlx+startx];
	a2=(img[2*curnpxlx+1]-img[1])/2.;
	a3=(img[2*curnpxlx+1]-2*img[curnpxlx+1]+img[1])/2.;
	a4=(img[curnpxlx+2]-img[curnpxlx])/2.;
	a5=(img[curnpxlx+2]-2*img[curnpxlx+1]+img[curnpxlx])/2.;
	a6=(img[2*curnpxlx+2]-img[2]-img[2*curnpxlx]+img[0])/4.;
	cy=(2*a2*a5-a4*a6)/(a6*a6-4*a3*a5)+1;
	cx=(2*a3*a4-a2*a6)/(a6*a6-4*a3*a5)+1;
	if(cx<0 || cy<0 || cx>=3 || cy>=3){
	  printf("Warning: x,y outside parabolic fit region: %g %g - adjusting\n",cx,cy);
	  cx=mxpos%curnpxlx-startx;
	  cy=mxpos/curnpxlx-starty;
	}
	cx+=startx+curnpxlx/2.-tstr->curnpxlx/2.;
	cy+=starty+curnpxlx/2.-tstr->curnpxlx/2.;

      }
    }
  }else{
    printf("centroid mode not yet implemented\n");
  }
  if(sum>=minflux){
    if(corrUpdateRequired){//i.e. correlation mode, and corrUpdateGain!=0.
      //Difficult: How do we store the images?  Subaps could overlap, and so 
      //we can't just store using subap location.  On the otherhand, the correlation references shouldn't overlap, so we could use the corrSubapLoc.
      //Question:  Should the shift be based on centroid returned, or from a CoG estimate?  CoG might be more robust, though more computationally demanding..
      //Compute a cog estimate for the shift and add.:

      //Not yet implemented for CENTROIDMODE_DIFFSQU* or CENTROIDMODE_BRUTE*
      float ccx=0,ccy=0,csum=0;
      if(cstr->shiftToCoG){
	int cnt=0;
	float *tsubap=tstr->tmpSubap;
	for(i=0; i<origSubapY; i++){
	  for(j=0; j<origSubapX; j++){
	    csum+=tsubap[cnt];//i*curnpxlx+j];
	    ccx+=j*tsubap[cnt];//i*curnpxlx+j];
	    ccy+=i*tsubap[cnt];//i*curnpxlx+j];
	    cnt++;
	  }
	}
	if(csum!=0){
	  ccy/=csum;
	  ccx/=csum;
	  ccy-=origSubapY/2.-0.5;
	  ccx-=origSubapX/2.-0.5;
	  if(cstr->corrImgOffset!=NULL){
	    ccx+=cstr->corrImgOffset[centindx];//for spots that have a strange shape - where teh CoG cannot be used to define the centre of teh spot, because the spot is offset - eg a sodium profile that is heavier at one end.
	    ccy+=cstr->corrImgOffset[centindx+1];
	  }
	}else{
	  ccy=0;
	  ccx=0;
	}
      }else{
	ccx=cx-0.5;
	ccy=cy-0.5;
      }
      //printf("%d %d %g %g\n",(int)roundf(ccx),(int)roundf(ccy),ccx,ccy);
      shiftAndIntegrateImage(cstr,threadno,tstr->tmpSubap,origSubapX,origSubapY,(int)roundf(ccx),(int)roundf(ccy));
    }
    if(cstr->windowMode==WINDOWMODE_ADAPTIVE){
      //add centroid offsets to get the overall location correct
      //(i.e. the distance from it's nominal centre).
      cx+=cstr->adaptiveCentPos[centindx];
      cy+=cstr->adaptiveCentPos[centindx+1];
      //and calculate adaptive window for next time.
      if(adapWinOffset==NULL || sum<minflux)
	calcAdaptiveWindow(cstr,threadno,cx,cy);
      else//for spots that have a strange shape - where the CoG cannot be used to define the centre of the spot.  Note, in correlation, this probably shouldn't be used - i.e. the position of the correlation image can define this instead.
	calcAdaptiveWindow(cstr,threadno,cx+adapWinOffset[centindx],cy+adapWinOffset[centindx+1]);
	
    }else if(cstr->windowMode==WINDOWMODE_GLOBAL){//add the current subap offset here
      cx+=cstr->adaptiveCentPos[centindx];
      cy+=cstr->adaptiveCentPos[centindx+1];
      if(adapWinOffset==NULL || sum<minflux){
	cstr->rawSlopes[centindx]=cx;
	cstr->rawSlopes[centindx+1]=cy;
      }else{
	cstr->rawSlopes[centindx]=cx+adapWinOffset[centindx];
	cstr->rawSlopes[centindx+1]=cy+adapWinOffset[centindx+1];
      }
    }
    if(cstr->centCalData!=NULL){//appy centroid linearisation...
      //Here, we apply the centroid linearisation.
      applySlopeLinearisation(cstr,threadno,&cx,&cy);
    }
    if(corrUpdateRequired){//used of corrUpdateGain!=0.
      cx-=cstr->updatedRefCents[centindx];
      cy-=cstr->updatedRefCents[centindx+1];
    }else if(cstr->refCents!=NULL){//subtract reference centroids.
      cx-=cstr->refCents[centindx];
      cy-=cstr->refCents[centindx+1];
    }
  }else{
    if(cstr->rawSlopes!=NULL){
      cstr->rawSlopes[centindx]=0;
      cstr->rawSlopes[centindx+1]=0;
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
  if((cstr->barrier=malloc(sizeof(pthread_barrier_t)*ncam))==NULL){
    printf("Error allocing barrier in slopeOpen\n");
    slopeClose(centHandle);
    *centHandle=NULL;
    return 1;
  }
  for(i=0;i<ncam;i++)
    pthread_barrier_init(&cstr->barrier[i],NULL,cstr->ncamThread[i]);

  return 0;
}

/**
   Called when parameters have changed
*/
int slopeNewParam(void *centHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr,int totCents){
  CentStruct *cstr=(CentStruct*)centHandle;
  int nfound;
  int err=0;
  int i,j,nb,m;
  int *index=cstr->index;
  void **values=cstr->values;
  char *dtype=cstr->dtype;
  int *nbytes=cstr->nbytes;
  int resetAdaptiveWindows=0;
  cstr->updateOverNFrames=1;
  cstr->arr=arr;
  cstr->totCents=totCents;
  nfound=bufferGetIndex(pbuf,NBUFFERVARIABLES,cstr->paramNames,index,values,dtype,nbytes);
  if(nfound!=NBUFFERVARIABLES){
    for(i=0; i<NBUFFERVARIABLES; i++){
      if(index[i]<0){
	if(i==CORRFFTPATTERN || i==CORRTHRESHTYPE || i==CORRTHRESH || i==CORRSUBAPLOCATION || i==CORRNPXLX || i==CORRNPXLCUM || i==CORRCLIP || i==CENTCALDATA || i==CENTCALSTEPS || i==CENTCALBOUNDS || i==CORRNSTORE || i==GAUSSMINVAL || i==GAUSSREPLACEVAL || i==FITMATRICES || i==FITSIZE || i==ADAPBOUNDARY || i==CORRUPDATEGAIN || i==SUBAPALLOCATION || i==CORRUPDATETOCOG || i==CORRCLIPINTEGIMG || i==ADAPWINOFFSET || i==CORRIMGOFFSET || i==CORRUPDATENFR || i==PYRAMIDMODE){
	  printf("%.16s not found - continuing\n",&cstr->paramNames[i*BUFNAMESIZE]);
	}else{
	  printf("Missing %.16s\n",&cstr->paramNames[i*BUFNAMESIZE]);
	  err=1;
	  writeErrorVA(cstr->rtcErrorBuf,-1,cstr->frameno,"rtcSlope module error");
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
    if(nbytes[NCAMTHREADS]==sizeof(int)*cstr->ncam && dtype[NCAMTHREADS]=='i'){
      cstr->ncamThread=(int*)values[NCAMTHREADS];
      if(cstr->ncamThreadCum==NULL && ((cstr->ncamThreadCum=malloc(sizeof(int*)*(cstr->ncam+1)))==NULL)){
	printf("Alloc ncamThreadCum error\n");
	err=1;
      }else{
	cstr->ncamThreadCum[0]=0;
	for(i=0;i<cstr->ncam;i++){
	  cstr->ncamThreadCum[i+1]=cstr->ncamThreadCum[i]+cstr->ncamThread[i];
	}
      }
    }else{
      printf("ncamThreads error\n");
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
    if(cstr->centIndxCum==NULL){
      if((cstr->centIndxCum=malloc(sizeof(int)*(cstr->ncam+1)))==NULL){
	err=1;
	printf("centIndxCum malloc failed in rtcslope\n");
      }else{
	cstr->centIndxCum[0]=0;
	for(i=0;i<cstr->ncam;i++){
	  cstr->centIndxCum[i+1]=cstr->centIndxCum[i];
	  for(j=cstr->nsubapCum[i];j<cstr->nsubapCum[i+1];j++){
	    cstr->centIndxCum[i+1]+=cstr->subapFlag[j]*2;
	  }
	}
      }
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
    if(index[ADAPWINOFFSET]<0 || nbytes[ADAPWINOFFSET]==0){
      cstr->adapWinOffset=NULL;
    }else{
      if(dtype[ADAPWINOFFSET]=='f' && nbytes[ADAPWINOFFSET]==sizeof(float)*cstr->totCents){
	cstr->adapWinOffset=(float*)values[ADAPWINOFFSET];
      }else{
	printf("adapWinOffset error\n");
	err=1;
	cstr->adapWinOffset=NULL;
      }
    }
    if(index[CORRIMGOFFSET]<0){
      cstr->corrImgOffset=NULL;
    }else{
      if(dtype[CORRIMGOFFSET]=='f' && nbytes[CORRIMGOFFSET]==sizeof(float)*cstr->totCents){
	cstr->corrImgOffset=(float*)values[CORRIMGOFFSET];
      }else{
	printf("corrImgOffset error\n");
	err=1;
	cstr->corrImgOffset=NULL;
      }
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
    cstr->fftCorrPatternSize=cstr->totPxls;//but it may get bigger.  Note, this should be specified even if corrFFTPattern==NULL.. because its used by centIndexArray (see below).
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
    if(index[CORRUPDATENFR]>=0){
      if(dtype[CORRUPDATENFR]=='i' && nbytes[CORRUPDATENFR]==sizeof(int)){
	cstr->updateOverNFrames=*((int*)values[CORRUPDATENFR]);
      }else{
	printf("Error- corrUpdateNfr\n");
	err=1;
      }
    }
    if(cstr->updateOverNFrames<1)
      cstr->updateOverNFrames=1;
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
    
    cstr->gaussMinVal=0;
    if(index[GAUSSMINVAL]>=0){
      if(nbytes[GAUSSMINVAL]==sizeof(float) && dtype[GAUSSMINVAL]=='f')
	cstr->gaussMinVal=*(float*)values[GAUSSMINVAL];
      else{
	printf("gaussMinVal error - ignoring\n");
      }
    }
    cstr->gaussReplaceVal=0;
    if(index[GAUSSREPLACEVAL]>=0){
      if(nbytes[GAUSSREPLACEVAL]==sizeof(float) && dtype[GAUSSREPLACEVAL]=='f')
	cstr->gaussReplaceVal=*(float*)values[GAUSSREPLACEVAL];
      else{
	printf("gaussReplaceVal error - ignoring\n");
      }
    }
    cstr->nFitMatrices=0;
    cstr->fitMatrices=NULL;
    if(index[FITMATRICES]>=0){
      if(dtype[FITMATRICES]=='f' && (nbytes[FITMATRICES]%(32*sizeof(float)))==0){
	cstr->fitMatrices=(float*)values[FITMATRICES];
	cstr->nFitMatrices=nbytes[FITMATRICES]/sizeof(float)/32;
	
      }else{
	printf("fitMatrices error - continuing\n");
      }
    }
    cstr->fitsize=3;
    if(index[FITSIZE]>=0){
      if(dtype[FITSIZE]=='i' && nbytes[FITSIZE]==sizeof(int)){
	cstr->fitsize=*((int*)values[FITSIZE]);
      }else{
	printf("fitsize error - ignoring\n");
      }
    }
    cstr->adapBoundary=NULL;
    if(index[ADAPBOUNDARY]>=0){
      if(dtype[ADAPBOUNDARY]=='i' && nbytes[ADAPBOUNDARY]==cstr->ncam*sizeof(int)*4){
	cstr->adapBoundary=(int*)values[ADAPBOUNDARY];
      }else{
	printf("adapBoundary error - ignoring\n");
      }
    }
    if(/*cstr->centroidMode==CENTROIDMODE_CORRELATIONGAUSSIAN ||*/ cstr->centroidMode==CENTROIDMODE_CORRELATIONCOG || cstr->centroidMode==CENTROIDMODE_CORRELATIONGAUSSIAN || cstr->centroidMode==CENTROIDMODE_CORRELATIONQUADRATIC || cstr->centroidModeArr!=NULL || cstr->centroidMode==CENTROIDMODE_DIFFSQUCOG || cstr->centroidMode==CENTROIDMODE_DIFFSQUGAUSSIAN || cstr->centroidMode==CENTROIDMODE_DIFFSQUQUADRATIC || cstr->centroidMode==CENTROIDMODE_CORRELATIONQUADRATICINTERP || cstr->centroidMode==CENTROIDMODE_DIFFSQUQUADRATICINTERP || cstr->centroidMode==CENTROIDMODE_BRUTECOG || cstr->centroidMode==CENTROIDMODE_BRUTEGAUSSIAN || cstr->centroidMode==CENTROIDMODE_BRUTEQUADRATIC || cstr->centroidMode==CENTROIDMODE_BRUTEQUADRATICINTERP){
      if(cstr->fftCorrelationPattern==NULL){//actually - should check, if entroidModeArr!=NULL that at least some of the entries use correlation, before raising an error...
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
      //Check for auto correlation update:
      cstr->corrUpdateGain=0;
      if(index[CORRUPDATEGAIN]>=0){
	if(dtype[CORRUPDATEGAIN]=='f' && nbytes[CORRUPDATEGAIN]==sizeof(float)){
	  cstr->corrUpdateGain=*(float*)values[CORRUPDATEGAIN];
	  if(cstr->corrUpdateGain>0){
	    if(cstr->rtcCorrRefBuf==NULL && cstr->corrNStore>0){
	      //open the circular buffer.
	      char *tmp;
	      if(asprintf(&tmp,"/%srtcCorrRefBuf",cstr->prefix)==-1){
		printf("Error asprintf in rtcslope - exiting\n");
		exit(1);
	      }
	      cstr->rtcCorrRefBuf=openCircBuf(tmp,1,&cstr->fftCorrPatternSize,'f',cstr->corrNStore);
	      free(tmp);
	    }else if(cstr->rtcCorrRefBuf!=NULL){
	      if(cstr->rtcCorrRefBuf->datasize!=cstr->fftCorrPatternSize*sizeof(float)){
		if(circReshape(cstr->rtcCorrRefBuf,1,&cstr->fftCorrPatternSize,'f')!=0){
		  printf("Error reshaping rtcCorrRefBuf\n");
		  err=1;
		}
	      }
	    }
	    if(cstr->rtcCentRefBuf==NULL && cstr->corrNStore>0){
	      //open the circular buffer.
	      char *tmp;
	      if(asprintf(&tmp,"/%srtcCentRefBuf",cstr->prefix)==-1){
		printf("Error asprintf in rtcslope - exiting\n");
		exit(1);
	      }
	      cstr->rtcCentRefBuf=openCircBuf(tmp,1,&cstr->totCents,'f',cstr->corrNStore);
	      free(tmp);
	    }else if(cstr->rtcCentRefBuf!=NULL){
	      if(cstr->rtcCentRefBuf->datasize!=cstr->totCents*sizeof(float)){
		if(circReshape(cstr->rtcCentRefBuf,1,&cstr->totCents,'f')!=0){
		  printf("Error reshaping rtcCentRefBuf\n");
		  err=1;
		}
	      }
	    }
	    if(cstr->rtcIntegratedImgBuf==NULL && cstr->corrNStore>0){
	      //open the circular buffer.
	      char *tmp;
	      if(asprintf(&tmp,"/%srtcIntegratedImgBuf",cstr->prefix)==-1){
		printf("Error asprintf in rtcslope - exiting\n");
		exit(1);
	      }
	      cstr->rtcIntegratedImgBuf=openCircBuf(tmp,1,&cstr->fftCorrPatternSize,'f',cstr->corrNStore);
	      free(tmp);
	    }else if(cstr->rtcIntegratedImgBuf!=NULL){
	      if(cstr->rtcIntegratedImgBuf->datasize!=cstr->fftCorrPatternSize*sizeof(float)){
		if(circReshape(cstr->rtcIntegratedImgBuf,1,&cstr->fftCorrPatternSize,'f')!=0){
		  printf("Error reshaping rtcIntegratedImgBuf\n");
		  err=1;
		}
	      }
	    }
	    if(cstr->updatedRefCentsSize<cstr->totCents){
	      if(cstr->updatedRefCents!=NULL)
		free(cstr->updatedRefCents);
	      if(posix_memalign((void**)(&(cstr->updatedRefCents)),SUBAPALIGN,sizeof(float)*cstr->totCents)!=0){
		printf("Error memaligning updatedRefCents\n");
		cstr->updatedRefCentsSize=0;
		cstr->updatedRefCents=NULL;
		err=1;
	      }else{
		cstr->updatedRefCentsSize=cstr->totCents;
	      }
	      memset(cstr->updatedRefCents,0,sizeof(float)*cstr->totCents);
	    }
	    if(cstr->integratedImgSize<cstr->fftCorrPatternSize){
	      if(cstr->integratedImg!=NULL)
		free(cstr->integratedImg);
	      if(posix_memalign((void**)(&(cstr->integratedImg)),SUBAPALIGN,sizeof(float)*cstr->fftCorrPatternSize)!=0){
		printf("Error memaligning integratedImg\n");
		cstr->integratedImgSize=0;
		cstr->integratedImg=NULL;
		err=1;
	      }else{
		cstr->integratedImgSize=cstr->fftCorrPatternSize;
		memset(cstr->integratedImg,0,sizeof(float)*cstr->fftCorrPatternSize);
		if(cstr->updatedCorrFFTPattern!=NULL)
		  free(cstr->updatedCorrFFTPattern);
		if(posix_memalign((void**)(&(cstr->updatedCorrFFTPattern)),SUBAPALIGN,sizeof(float)*cstr->fftCorrPatternSize)!=0){
		  printf("Error memaligning updatedCorrFFTPattern\n");
		  cstr->integratedImgSize=0;
		  free(cstr->integratedImg);
		  cstr->integratedImg=NULL;
		  cstr->updatedCorrFFTPattern=NULL;
		  err=1;
		}else{
		  memset(cstr->updatedCorrFFTPattern,0,sizeof(float)*cstr->fftCorrPatternSize);
		}
	      }
	    }
	    cstr->shiftToCoG=1;
	    if(index[CORRUPDATETOCOG]>=0){
	      if(dtype[CORRUPDATETOCOG]=='i' && nbytes[CORRUPDATETOCOG]==sizeof(int))
		cstr->shiftToCoG=*(int*)values[CORRUPDATETOCOG];
	      else
		printf("Unrecognised datatype/size for corrUpdateToCoG\n");
	    }
	    cstr->clipIntegImg=1;
	    if(index[CORRCLIPINTEGIMG]>=0){
	      if(dtype[CORRCLIPINTEGIMG]=='i' && nbytes[CORRCLIPINTEGIMG]==sizeof(int))
		cstr->clipIntegImg=*(int*)values[CORRCLIPINTEGIMG];
	      else
		printf("Unrecognised datatype/size for corrClipIntegImg\n");
	    }
	  }else if(cstr->lastCorrUpdateGain!=0){
	    //User is switching out of automatic gain update.  So here, should we copy the updatedRefCents and updatedCorrFFTPattern back across?
	    //If so, then the latest pattern will continue to be used, without update.
	    //But note if the user has updated a new pattern during the same buffer swap, this will be overwritten.  But that is probably a restriction worth having...
	    /*Actually, a bit of a pain during testing, so removed this.
	    if(cstr->refCents!=NULL && cstr->updatedRefCents!=NULL)
	      memcpy(cstr->refCents,cstr->updatedRefCents,sizeof(float)*cstr->totCents);
	    if(cstr->fftCorrelationPattern!=NULL && cstr->updatedCorrFFTPattern!=NULL)
	      memcpy(cstr->fftCorrelationPattern,cstr->updatedCorrFFTPattern,sizeof(float)*cstr->fftCorrPatternSize);
	    */
	  }
	}else{
	  printf("Error - corrUpdateGain not specified correctly\n");
	  writeErrorVA(cstr->rtcErrorBuf,-1,cstr->frameno,"corrUpdateGain error");
	  err=1;
	}
      }
    }

    if(index[SUBAPALLOCATION]==0){
      cstr->subapAllocation=NULL;
    }else if(dtype[SUBAPALLOCATION]=='i' && nbytes[SUBAPALLOCATION]==cstr->nsubaps*sizeof(int)){
      cstr->subapAllocation=(int*)values[SUBAPALLOCATION];
    }else{
      if(nbytes[SUBAPALLOCATION]!=0){
	printf("Error - subapAllocation\n");
	err=1;
      }
      cstr->subapAllocation=NULL;
    }
    if(index[PYRAMIDMODE]>=0 && nbytes[PYRAMIDMODE]==sizeof(int) && dtype[PYRAMIDMODE]=='i' && *((int*)values[PYRAMIDMODE])==1){
      if(cstr->totalCamFlux==NULL){
        if((cstr->totalCamFlux=calloc(sizeof(float),cstr->ncam))==NULL)
	  printf("Unable to alloc totalCamFlux for pyramidMode\n");
      }
    }else{
      if(cstr->totalCamFlux!=NULL){
        free(cstr->totalCamFlux);
        cstr->totalCamFlux=NULL;
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
	if(cstr->rawSlopesSize<cstr->totCents){
	  free(cstr->rawSlopes);
	  if((cstr->rawSlopes=malloc(sizeof(float)*cstr->totCents))==NULL){
	    printf("Error allocing rawslopes\n");
	    err=1;
	    cstr->rawSlopesSize=0;
	  }else{
	    memset(cstr->rawSlopes,0,sizeof(float)*cstr->totCents);
	    cstr->rawSlopesSize=cstr->totCents;
	  }
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
	cstr->centIndexSize=nb/(sizeof(float)*cstr->fftCorrPatternSize);//note - if no correlation pattern is specified, fftCorrPatternSize is equal to totPxls.
	if(nb==cstr->centIndexSize*sizeof(float)*cstr->fftCorrPatternSize){
	  cstr->centIndexArr=(float*)values[CENTINDEXARRAY];
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
    circClose(cstr->rtcCorrRefBuf);
    circClose(cstr->rtcCentRefBuf);
    circClose(cstr->rtcIntegratedImgBuf);
    if(cstr->barrier!=NULL){
      for(i=0;i<cstr->ncam;i++)
	pthread_barrier_destroy(&cstr->barrier[i]);
      free(cstr->barrier);
    }
    if(cstr->nsubapCum!=NULL)
      free(cstr->nsubapCum);
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
	  if(cstr->tstr[i]->tmpSubap!=NULL)
	    free(cstr->tstr[i]->tmpSubap);
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
    if(cstr->rawSlopes!=NULL)
      free(cstr->rawSlopes);
    if(cstr->adaptiveCentPos!=NULL)
      free(cstr->adaptiveCentPos);
    if(cstr->adaptiveWinPos!=NULL)
      free(cstr->adaptiveWinPos);
    if(cstr->ncamThreadCum!=NULL)
      free(cstr->ncamThreadCum);
    if(cstr->centIndxCum!=NULL)
      free(cstr->centIndxCum);
    if(cstr->integratedImg!=NULL)
      free(cstr->integratedImg);
    if(cstr->updatedRefCents!=NULL)
      free(cstr->updatedRefCents);
    if(cstr->updatedCorrFFTPattern!=NULL)
      free(cstr->updatedCorrFFTPattern);
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
  
  if(cstr->rtcCorrBuf){//we unset it here, because we set it later in completeFn - there is something not quite thead safe here - will have to think.
    cstr->addReqCorr=circSetAddIfRequired(cstr->rtcCorrBuf,frameno);
    cstr->rtcCorrBuf->addRequired=0;
  }
  if(cstr->rtcCalCorrBuf){
    cstr->addReqCalCorr=circSetAddIfRequired(cstr->rtcCalCorrBuf,frameno);
    cstr->rtcCalCorrBuf->addRequired=0;
  }
  if(cstr->rtcCentRefBuf){
    cstr->addReqCentRef=circSetAddIfRequired(cstr->rtcCentRefBuf,frameno);
    cstr->rtcCentRefBuf->addRequired=0;
  }
  if(cstr->rtcIntegratedImgBuf){
    cstr->addReqIntegratedImg=circSetAddIfRequired(cstr->rtcIntegratedImgBuf,frameno);
    cstr->rtcIntegratedImgBuf->addRequired=0;
  }
  if(cstr->rtcCorrRefBuf){
    cstr->addReqCorrRef=circSetAddIfRequired(cstr->rtcCorrRefBuf,frameno);
    cstr->rtcCorrRefBuf->addRequired=0;
  }


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
	printf("Error - subapSize smaller than expected in rtcslope: %d %d %d %d\n",subindx,pos,tstr->curnpxl,subapSize);
      }else{
	calcCentroid(cstr,threadno);
      }
      pos+=((tstr->curnpxl+(SUBAPALIGN/sizeof(float))-1)/(SUBAPALIGN/sizeof(float)))*(SUBAPALIGN/sizeof(float));
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
  int centroidMode;
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
  cstr->post.addReqCorr=cstr->addReqCorr;
  cstr->post.addReqCalCorr=cstr->addReqCalCorr;
  centroidMode=cstr->centroidMode;
  if(centroidMode==CENTROIDMODE_CORRELATIONCOG || centroidMode==CENTROIDMODE_CORRELATIONGAUSSIAN || centroidMode==CENTROIDMODE_CORRELATIONQUADRATIC || cstr->centroidModeArr!=NULL || centroidMode==CENTROIDMODE_DIFFSQUCOG || centroidMode==CENTROIDMODE_DIFFSQUGAUSSIAN || centroidMode==CENTROIDMODE_DIFFSQUQUADRATIC || centroidMode==CENTROIDMODE_CORRELATIONQUADRATICINTERP || centroidMode==CENTROIDMODE_DIFFSQUQUADRATICINTERP || centroidMode==CENTROIDMODE_BRUTECOG || centroidMode==CENTROIDMODE_BRUTEGAUSSIAN || centroidMode==CENTROIDMODE_BRUTEQUADRATIC || centroidMode==CENTROIDMODE_BRUTEQUADRATICINTERP){
    if(cstr->corrUpdateGain!=0 || cstr->lastCorrUpdateGain!=0){
      if(forcewrite!=0){
	FORCEWRITE(cstr->rtcCorrRefBuf)=forcewrite;
	FORCEWRITE(cstr->rtcCentRefBuf)=forcewrite;
	FORCEWRITE(cstr->rtcIntegratedImgBuf)=forcewrite;
      }
      cstr->rtcCorrRefBuf->addRequired=cstr->addReqCorrRef;
      circAdd(cstr->rtcCorrRefBuf,cstr->updatedCorrFFTPattern,cstr->timestamp,cstr->frameno);
      cstr->rtcCentRefBuf->addRequired=cstr->addReqCentRef;
      circAdd(cstr->rtcCentRefBuf,cstr->updatedRefCents,cstr->timestamp,cstr->frameno);
      cstr->rtcIntegratedImgBuf->addRequired=cstr->addReqIntegratedImg;
      circAdd(cstr->rtcIntegratedImgBuf,cstr->integratedImg,cstr->timestamp,cstr->frameno);
    }
  }
  cstr->lastCorrUpdateGain=cstr->corrUpdateGain;
  if(cstr->totalCamFlux!=NULL){
    //sum the flux for this camera.
    float tot;
    int i,cam,start,end;
    for(cam=0;cam<cstr->ncam;cam++){
      start=cstr->centIndxCum[cam]/2;
      end=cstr->centIndxCum[cam+1]/2;
      tot=0;
      
      for(i=start;i<end;i++)
	tot+=cstr->arr->flux[i];
      if(end>start)
	cstr->totalCamFlux[cam]=tot/(end-start);
    }
  }
  
  return 0;
}

/**
   Called when we've finished everything for this frame.
*/
int slopeComplete(void *centHandle){
  //Note, centNewParam could be called at the same time as this, by a different thread...
  CentStruct *cstr=(CentStruct*)centHandle;
  CentPostStruct *p=&cstr->post;
  if(p->centroidMode==CENTROIDMODE_CORRELATIONCOG || p->centroidMode==CENTROIDMODE_CORRELATIONGAUSSIAN || p->centroidMode==CENTROIDMODE_CORRELATIONQUADRATIC || p->centroidModeArr!=NULL || p->centroidMode==CENTROIDMODE_DIFFSQUCOG || p->centroidMode==CENTROIDMODE_DIFFSQUGAUSSIAN || p->centroidMode==CENTROIDMODE_DIFFSQUQUADRATIC || p->centroidMode==CENTROIDMODE_CORRELATIONQUADRATICINTERP || p->centroidMode==CENTROIDMODE_DIFFSQUQUADRATICINTERP || p->centroidMode==CENTROIDMODE_BRUTECOG || p->centroidMode==CENTROIDMODE_BRUTEGAUSSIAN || p->centroidMode==CENTROIDMODE_BRUTEQUADRATIC || p->centroidMode==CENTROIDMODE_BRUTEQUADRATICINTERP){//p->centroidMode==CENTROIDMODE_CORRELATIONGAUSSIAN){
    //Note - should check that correlation is actually used!
    if(p->corrbuf!=NULL){
      if(p->rtcCorrBuf!=NULL){
	p->rtcCorrBuf->addRequired=p->addReqCorr;
	circAdd(p->rtcCorrBuf,p->corrbuf,p->timestamp,p->frameno);
      }
      memset(p->corrbuf,0,sizeof(float)*p->totPxls);
    }
    if(p->calcorrbuf!=NULL){
      if(p->rtcCalCorrBuf!=NULL){// && p->rtcCalCorrBuf->addRequired)
	p->rtcCalCorrBuf->addRequired=p->addReqCalCorr;
	circAdd(p->rtcCalCorrBuf,p->calcorrbuf,p->timestamp,p->frameno);
      }
      memset(p->calcorrbuf,0,sizeof(float)*p->totPxls);
    }
  }
  return 0;
}

/*Used during auto-correlation update*/
int slopeStartFrame(void *centHandle,int cam,int threadno){
  //Update the correlation reference, and the reference centroids.
  //Basically, prepare the correlation reference ready for this frame,
  //using the integrated images from previous frames.
  //Note, this is done here because it allows it to be done during spare time while waiting for the cameras to deliver first pixels.  If done in slopeCalcSlope, latency will be increased.  However, doing it here may result in thread safety issues, since if subapAllocation isn't being used, it is possible for slopeCalcSlope to be called for a subap that is having its correlation ref updated by this thread here, before or after.  So, to prevent this, a barrier has been inserted.
  int nsub;
  int centOffset;
  int *subapFlag;
  int t,subStart,subEnd,i;
  int centroidMode;
  CentStruct *cstr=(CentStruct*)centHandle;
  unsigned int corrUpdateFrameno=cstr->frameno%cstr->updateOverNFrames;
  if(cstr->corrUpdateGain!=0){
    nsub=cstr->nsub[cam];
    centOffset=cstr->centIndxCum[cam];
    subapFlag=cstr->subapFlag;
    if(cstr->subapAllocation==NULL){
      //Process (nsub+ncamThread[cam]-1)/ncamThread[cam] subaps
      t=threadno-cstr->ncamThreadCum[cam];//offset for this camera
      subStart=t*((nsub+cstr->ncamThread[cam]-1)/cstr->ncamThread[cam])+ cstr->nsubapCum[cam];
      subEnd=subStart+(nsub+cstr->ncamThread[cam]-1)/cstr->ncamThread[cam];
      if(subEnd>cstr->nsubapCum[cam+1])
	subEnd=cstr->nsubapCum[cam+1];
      for(i=cstr->nsubapCum[cam];i<subStart;i++){
	if(subapFlag[i])
	  centOffset+=2;
      }
      for(i=subStart;i<subEnd;i++){
	if(subapFlag[i]){
	  if(cstr->centroidModeArr==NULL)
	    centroidMode=cstr->centroidMode;
	  else
	    centroidMode=cstr->centroidModeArr[i];
	  if(centroidMode==CENTROIDMODE_CORRELATIONCOG || centroidMode==CENTROIDMODE_CORRELATIONGAUSSIAN || centroidMode==CENTROIDMODE_CORRELATIONQUADRATIC || centroidMode==CENTROIDMODE_CORRELATIONQUADRATICINTERP){
	    if(cstr->lastCorrUpdateGain==0){//if its the first time, update all references... (this could take a while - infact, there is a quicker way to do it, but not yet implemented.)
	      setSubapDeltaFn(cstr,cam,threadno,i,centOffset);
	      updateCorrReference(cstr,cam,threadno,i,centOffset);
	    }else if(i%cstr->updateOverNFrames==corrUpdateFrameno){//otherwise, only update a subset each iteration - to reduce computational load.
	      updateCorrReference(cstr,cam,threadno,i,centOffset);
	    }
	  }
	  centOffset+=2;
	}
      }
      //Now block until all threads for this camera have updated their references.  Otherwise, thread safety issues can result.
      pthread_barrier_wait(&cstr->barrier[cam]);
    }else{
      //Process subaps that are allocated to this thread.
      for(i=cstr->nsubapCum[cam];i<cstr->nsubapCum[cam+1];i++){
	if(subapFlag[i]){
	  if(cstr->centroidModeArr==NULL)
	    centroidMode=cstr->centroidMode;
	  else
	    centroidMode=cstr->centroidModeArr[i];
	  if(centroidMode==CENTROIDMODE_CORRELATIONCOG || centroidMode==CENTROIDMODE_CORRELATIONGAUSSIAN || centroidMode==CENTROIDMODE_CORRELATIONQUADRATIC || centroidMode==CENTROIDMODE_CORRELATIONQUADRATICINTERP){
	    
	    if(cstr->subapAllocation[i]==threadno){
	      if(cstr->lastCorrUpdateGain==0){
		setSubapDeltaFn(cstr,cam,threadno,i,centOffset);
		updateCorrReference(cstr,cam,threadno,i,centOffset);
	      }else if(i%cstr->updateOverNFrames==corrUpdateFrameno){
		updateCorrReference(cstr,cam,threadno,i,centOffset);
	      }
	    }
	  }
	  centOffset+=2;
	}
      }
    }
  }else if(cstr->lastCorrUpdateGain!=0){
    //User has just turned off auto correlation update.
    //copy updatedRefCents to refCents.
    //Why copy?  Because the correlation reference will still be set to the automatically produced one, so slopes need to be consistent.  The user should then set both at the same time.
    //Actually - don't to this!  
    //if(cstr->updatedRefCents!=NULL && cstr->refCents!=NULL){
    // memcpy(cstr->refCents,cstr->updatedRefCents,sizeof(float)*cstr->totCents);
    //}
  }
  return 0;
}
/*Uncomment if needed
int slopeNewFrame(void *centHandle,unsigned int frameno,double timestamp){
}
int slopeEndFrame(void *centHandle,int cam,int threadno,int err){//subap thread (once per thread)
}
int slopeFrameFinished(void *centHandle,int err){//non-subap thread (once)
}
int slopeOpenLoop(void *centHandle){
}

*/
