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
#define USECOND
//#include <fftw3.h>
#include "circ.h"
#include "arrayStruct.h"
#include "buffer.h"
#include "rtccamera.h"
#include "rtccalibrate.h"
#include "rtcslope.h"
#include "rtcrecon.h"
#include "rtcmirror.h"
#include "rtcfigure.h"
#include "rtcbuffer.h"
#ifndef DARC_H
#define DARC_H
#define STATUSBUFSIZE 320
#define ERRORBUFSIZE 128
//#define PROJID 98
//#define NCAMERAPARAMETERS 10//the max number of parameters for the camera library.
//#define MAXSUBAPSIZE 64//the maximum linear size of a subap. Used when allocating threadInfo->subap, and the fftPlanArray.


enum WindowModes{WINDOWMODE_BASIC,WINDOWMODE_ADAPTIVE,WINDOWMODE_GLOBAL,WINDOWMODE_ERROR};
/*
enum CentroidModes{CENTROIDMODE_WPU,CENTROIDMODE_COG,CENTROIDMODE_GAUSSIAN,CENTROIDMODE_CORRELATIONCOG,CENTROIDMODE_CORRELATIONGAUSSIAN,CENTROIDMODE_ERROR};
enum CorrelationThresholdType{CORR_ABS_SUB,CORR_ABS_ZERO,CORR_FRAC_SUB,CORR_FRAC_ZERO};
*/
enum Errors{CLIPERROR,PARAMERROR,CAMSYNCERROR,CAMGETERROR,SLOPELINERROR,SLOPEERROR,FRAMENOERROR,MIRRORSENDERROR};//,CAMOPENERROR,CAMFRAMEERROR,MIRROROPENERROR};
//various different reconstruction modes when not using kalman.
//typedef enum{RECONMODE_SIMPLE,RECONMODE_TRUTH,RECONMODE_OPEN,RECONMODE_OFFSET}ReconModeType;


/**
   data for the pre/post computation thread.
*/
typedef struct{
  float *dmCommand;
  float *dmCommandSave;
  float *dmCommandFigure;
  int nclipped;
  int maxClipped;
  //float *Xdmc;
  float *calpxlbuf;//for storing the calibrated pixels for circular buffer
  //float *corrbuf;//for storing the correlated images for circular buffer
  float *centroids;
  int nacts;
  int totCents;
  int totPxls;
  float *userActs;
  float *userActsMask;//a mask to apply to the computed actuator vals
  int addUserActs;
  int userActSeqLen;//no of entries in userActs, each of size nacts.
  int *userActSeq;//NULL or size userActSeqLen.
  int seqCnt;
  int actCnt;
  float *v0;
  //int usingDMC;
  void *mirrorHandle;
  int (*mirrorSendFn)(MIRRORSENDARGS);
  int (*reconFrameFinishedFn)(RECONFRAMEFINISHEDARGS);
  int (*reconOpenLoopFn)(RECONOPENLOOPARGS);
  int (*camFrameFinishedFn)(CAMFRAMEFINISHEDARGS);//void *calibrateHandle,int err);
  int (*calibrateFrameFinishedFn)(CALIBRATEFRAMEFINISHEDARGS);//void *calibrateHandle,int err);
  int (*camOpenLoopFn)(CAMOPENLOOPARGS);//void *calibrateHandle);
  int (*calibrateOpenLoopFn)(CALIBRATEOPENLOOPARGS);//void *calibrateHandle);
  int (*centFrameFinishedFn)(SLOPEFRAMEFINISHEDARGS);
  int (*centOpenLoopFn)(SLOPEOPENLOOPARGS);

  int (*camCompleteFn)(CAMCOMPLETEARGS);
  int (*calibrateCompleteFn)(CALIBRATECOMPLETEARGS);
  int (*centCompleteFn)(SLOPECOMPLETEARGS);
  int (*reconCompleteFn)(RECONCOMPLETEARGS);

  int thisiter;
  enum WindowModes windowMode;
  int *subapLocation;
  int *closeLoop;
  int openLoopIfClipped;
  int pxlCentInputError;
  double timestamp;
  int *recordCents;
  float *pmx;
  //float *v0;
  int pmxSize;
  float *flux;
  //int *averageImg;
  //int nAvImg;
  //float *avCalPxlBuf;//for optionally storing an averaged cal pxls over several frames.
  //int avCalPxlBufSize;
  //int *averageCent;
  //int nAvCent;
  //float *avCentBuf;
  //int avCentBufSize;
  float *actsRequired;//used when running as a figure sensor, should be set to the latest DM demands from the RTC.  This should be allocated by the .so library responsible for reading in the DM demands, and free'd when the library is closed.
  pthread_mutex_t actsRequiredMutex;//used in figure sensor mode
  pthread_mutex_t *libraryMutex;
  pthread_cond_t actsRequiredCond;//used in figure sensor mode
  void *mirrorLib;
  int go;
  //float adaptiveWinGain;
  float figureGain;
  float *figureGainArr;
  int noPrePostThread;//if set, then pre and post processing is done by a subap thread.
  int circAddFlags;
  int clipOccurred;
}PostComputeData;

typedef struct{
  pthread_mutex_t prepMutex;
  pthread_mutex_t postMutex;
  pthread_cond_t prepCond;
  pthread_cond_t postCond;

  //pthread_cond_t pxlcentCond;
  //pthread_mutex_t pxlcentMutex;
  //int pxlcentReady;
  float *dmCommand;
  int nacts;
  pthread_t threadid;
  int totCents;
  int readyToStart;
  int doswitch;
  PostComputeData post;
}PreComputeData;


/**
   holds info relevent for one camera (ie one frame grabber), common to all the threads processing for this camera
*/
typedef struct{//one array for each camera, double buffered...
  //int nsuby;
  int pause;
  int nsub;
  int npxlx;
  int npxly;
  int npxlCum;
  //int nusedSubaps;//number of subaps valid for this camera
  int centCumIndx;//cumulative centroid number for the first centroid of this camera.
  int subCumIndx;//cumulative subap number for the first subap of this camera.
  int cam;//this current number
  int *subflag;//flat as to whether subap is valid or not.
  int nthreads;//number of threads serving this camera.
  int subindx;//index of the next subap to be evaluated...
  int centindx;//index of next centroid to be evaluated.
  //float *centroids;//cents for each subap, x,y,x,y...
  //float *flux;//flux in each subap
  //float *refCents;//reference centroids
  //int thresholdAlgo;
  //float threshold;
  //float *thresholdArr;
  //int thresholdArrType;
  //float powerFactor;
  //float *darkNoise;
  //float *darkNoiseArr;
  //float *background;
  //float *backgroundArr;
  //float *flatField;
  //float *flatFieldArr;
  //float *pxlweight;
  //float *pxlweightArr;
  pthread_mutex_t subapMutex;
  pthread_mutex_t startInfoMutex;//mutex obtained during buffer swap
  int threadInfoCount;//used during buffer swap to determine if this is the first thread for this camera.
  int frameFinished;//[2];
  int *subapLocation;//either points to realSubapLocation or is own array, when using adaptive windowing.
  //float *centWeighting;
  //float *dmCommand;//this is float here, but converted to int for actsSent.
  //float *calpxlbuf;
  //float *corrbuf;
  //enum CentroidModes centroidMode;
  //short *pxlbuf;//not sure how to handle this: this is the pxlbuf for this cam
  //int go;//whether to run or not.
  int id;//for debugging
  int threadCountFinished;//count of number of threads finished for this camera
  //float adaptiveWinGain;
  //enum CorrelationThresholdType correlationThresholdType;//type of threshold applied when doing correlation centroiding
  //float correlationThreshold;//value of threshold when doing correlation centroiding.
  //int resetAdaptiveWindows;
  //float *adaptiveWinPos;
  //int *adaptiveCentPos;
  //float *fftCorrelationPattern;//the spot PSF array, FFT'd in HC format, and placed as per subapLocation...
  //float *fftCorrelationPatternArr;
  //int nsubapsTogether;//number of subaps evaluated by a thread each time.
  int pxlCentInputError;//set if error raised.
  //float *calmult;
  //float *calsub;
  //float *calthr;
  //float *centCalData;
  //float *centCalSteps;
  //int *centCalBounds;
  //int centCalnsteps;
  //int useBrightest;
  //int *useBrightestArr;
  //int maxAdapOffset;
  //int nAdaptiveGroups;
  //float *groupSumX;
  //float *groupSumY;
  //int *groupSum;
  //int *adaptiveGroup;
  //int adaptiveGroupSize;
  int nsteps;
}infoStruct;

/**
   holds global information.
*/
typedef struct{//info shared between all threads.
  int ncam;//set at start and never changed...
  int *nsubList;
  //int *nsubxList;
  int *realSubapLocation;//user supplied subap locations
  int maxPxlPerSubap;
  int subapLocationType;
  int *npxlxList;
  int *npxlyList;
  int nsubaps;
  enum WindowModes windowMode;
  int *subapFlagArr;//subflag indexes into this array.
  int nacts;
  int *pxlCnt;
  int totPxls;
  int totCents;
  int doswitch;//[2];//whether buffer needs to bufswap...
  int *switchRequested;//set from python... via shared memory for the c version
  float *userActs;
  float *userActsMask;
  int addUserActs;
  int userActSeqLen;
  int *userActSeq;
  float *v0;
  int *recordCents;
  int *closeLoop;
  int openLoopIfClipped;
  int printTime;
  //int *fakeCCDImage;//can be specified and this is used instead of camera data
  int signalled;//set if a fatal error occurs.
  unsigned int *threadAffinityList;
  int *threadPriorityList;
  unsigned int *threadAffinityListPrev;
  unsigned int *threadAffinityListPrevMem;
  int *threadPriorityListPrev;
  int threadAffinityElSize;
  int threadAffinityElSizePrev;
  int noPrePostThread;//if set, then pre and post processing is done by a subap thread.
  int delay;//artificial delay to slow rtc down.
  int nsteps;//number of iters to do before pausing (continuous if <=0)
  int maxClipped;
  //int *averageImg;//no of frames of calpxl to average and send to generic stream
  //int nAvImg;//store above.
  //int *averageCent;
  //int nAvCent;
  float figureGain;
  float *figureGainArr;
  pthread_mutex_t startMutex;//[2];
  pthread_mutex_t startFirstMutex;
  pthread_mutex_t endMutex;//[2];
  pthread_mutex_t frameRunningMutex;//[2];
  pthread_cond_t frameRunningCond;//[2];
  int threadCount;//[2];
  int threadCountFinished;//[2];
  arrayStruct *arrays;//[2];
  int niters;
  paramBuf *buffer[2];//the SHM buffer containing all the config data.
  int curBuf;
  int nthreads;
  int *bufferHeaderIndex;
  int *bufferNbytes;
  char *bufferDtype;
  void **bufferValues;
  char *paramNames;
  pthread_mutex_t camMutex;
  
  void **threadInfoHandle;
  PreComputeData *precomp;
  pthread_t *threadList;
  //for timings (mean, stdev, maximum - jitter)
  double sumtime;
  double sum2time;
  double maxtime;
  double frameTime;
  int maxtimeiter;
  int thisiter;//copied from myiter, or from one of the .so library frame numbers
  int itersource;//where to copy thisiter from
  unsigned int iterindex;//index into the frameno array from source itersource
  int myiter;//incremented by rtc every iteration.
  //int camiter;//the last frame number received and accepted from camera
  int ntime;
  int *ppause;//pointer to the pause entry in the shm buffer, or NULL.
  int buferr;//set when an error occurs during buffer swap.
  //struct timeval starttime;
  double starttime;
  int go;//whether to run or not.
  int nclipped;
  char statusBuf[STATUSBUFSIZE];
  int statusBufPos;
  circBuf *rtcPxlBuf;
  circBuf *rtcCalPxlBuf;
  //circBuf *rtcCorrBuf;
  circBuf *rtcCentBuf;
  circBuf *rtcMirrorBuf;
  circBuf *rtcActuatorBuf;
  circBuf *rtcStatusBuf;
  circBuf *rtcTimeBuf;
  circBuf *rtcErrorBuf;
  circBuf *rtcSubLocBuf;
  circBuf *rtcGenericBuf;
  circBuf *rtcFluxBuf;
  int prepThreadAffinity;//thread affinity of the prepareActuators thread.
  int prepThreadPriority;//thread affinity of the prepareActuators thread.
  void *camHandle;
  void *mirrorHandle;
  void *centHandle;
  void *figureHandle;
  //int camFraming;
  //int centFraming;
  //float *centbufs;
  //int centbufsSize;
  //pthread_t fftPlanThreadid;
  //int fftPlanningDone;
  //now the camera dynamic library functions...
  int *camerasOpen;
  //int *camerasFraming;
  int *cameraParams;
  int cameraParamsCnt;
  int *centOpen;
  int *mirrorOpen;
  int *mirrorParams;
  int mirrorParamsCnt;
  int *reconlibOpen;
  int *figureOpen;
  int *figureParams;
  int *calibrateOpen;
  int *bufferlibOpen;
  int figureParamsCnt;

  void *cameraLib;
  char *cameraName;
  char *cameraNameOpen;
  //These should agree with rtccamera.h...
  int (*camOpenFn)(CAMOPENARGS);
  int (*camCloseFn)(CAMCLOSEARGS);
  //int (*camStartFramingFn)(int n,int *args,void *camHandle);
  //int (*camStopFramingFn)(void *camHandle);
  int (*camNewFrameFn)(CAMNEWFRAMEARGS);
  int (*camNewFrameSyncFn)(CAMNEWFRAMESYNCARGS);
  int (*camWaitPixelsFn)(CAMWAITPIXELSARGS);
  int (*camComputePixelsFn)(CAMCOMPUTEPIXELSARGS);
  int (*camNewParamFn)(CAMNEWPARAMARGS);
  int (*camStartFrameFn)(CAMSTARTFRAMEARGS);
  int (*camEndFrameFn)(CAMENDFRAMEARGS);
  int (*camFrameFinishedSyncFn)(CAMFRAMEFINISHEDSYNCARGS);
  int (*camFrameFinishedFn)(CAMFRAMEFINISHEDARGS);
  int (*camOpenLoopFn)(CAMOPENLOOPARGS);
  int (*camCompleteFn)(CAMCOMPLETEARGS);
  //and the mirror dynamic library...
  char *mirrorName;
  char *mirrorNameOpen;
  void *mirrorLib;
  int (*mirrorOpenFn)(MIRROROPENARGS);
  int (*mirrorCloseFn)(MIRRORCLOSEARGS);
  int (*mirrorNewParamFn)(MIRRORNEWPARAMARGS);
  int (*mirrorSendFn)(MIRRORSENDARGS);
  //And the centroider dynamic library functions...
  char *centName;
  char *centNameOpen;
  void *centLib;
  int *centParams;
  int centParamsCnt;
  int (*centOpenFn)(SLOPEOPENARGS);
  int (*centNewParamFn)(SLOPENEWPARAMARGS);
  int (*centCloseFn)(SLOPECLOSEARGS);
  int (*centNewFrameFn)(SLOPENEWFRAMEARGS);
  int (*centNewFrameSyncFn)(SLOPENEWFRAMESYNCARGS);
  int (*centStartFrameFn)(SLOPESTARTFRAMEARGS);
  int (*centCalcSlopeFn)(SLOPECALCSLOPEARGS);
  int (*centEndFrameFn)(SLOPEENDFRAMEARGS);
  int (*centFrameFinishedSyncFn)(SLOPEFRAMEFINISHEDSYNCARGS);
  int (*centFrameFinishedFn)(SLOPEFRAMEFINISHEDARGS);
  int (*centOpenLoopFn)(SLOPEOPENLOOPARGS);
  int (*centCompleteFn)(SLOPECOMPLETEARGS);
  void *calibrateLib;
  char *calibrateName;
  char *calibrateNameOpen;
  int calibrateParamsCnt;
  int *calibrateParams;
  int (*calibrateOpenFn)(CALIBRATEOPENARGS);//char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **handle,int nthreads,unsigned int frameno);
  int (*calibrateNewParamFn)(CALIBRATENEWPARAMARGS);//void *calibrateHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr);
  int (*calibrateCloseFn)(CALIBRATECLOSEARGS);//(void **calibrateHandle);
  int (*calibrateNewFrameFn)(CALIBRATENEWFRAMEARGS);//void *calibrateHandle,unsigned int frameno);
  int (*calibrateNewFrameSyncFn)(CALIBRATENEWFRAMESYNCARGS);//void *calibrateHandle,unsigned int frameno);//subap thread (once/iter).
  int (*calibrateStartFrameFn)(CALIBRATESTARTFRAMEARGS);//void *calibrateHandle,int cam,int threadno);
  int (*calibrateNewSubapFn)(CALIBRATENEWSUBAPARGS);//void *calibrateHandle,int cam,int threadno,int cursubindx,float **subap, int *subapSize,int *curnpxlx,int *curnpxly);
  int (*calibrateEndFrameFn)(CALIBRATEENDFRAMEARGS);//void *calibrateHandle,int cam,int threadno,int err);
  int (*calibrateFrameFinishedSyncFn)(CALIBRATEFRAMEFINISHEDSYNCARGS);//void *calibrateHandle,int err);
  int (*calibrateFrameFinishedFn)(CALIBRATEFRAMEFINISHEDARGS);//void *calibrateHandle,int err);
  int (*calibrateOpenLoopFn)(CALIBRATEOPENLOOPARGS);//void *calibrateHandle);
  int (*calibrateCompleteFn)(CALIBRATECOMPLETEARGS);

  void *calibrateHandle;

  //and the figure sensing dynamic library...
  char *figureName;
  char *figureNameOpen;
  void *figureLib;
  int (*figureOpenFn)(FIGUREOPENARGS);
  int (*figureCloseFn)(FIGURECLOSEARGS);
  int (*figureNewParamFn)(FIGURENEWPARAMARGS);
  void *reconLib;
  char *reconName;
  char *reconNameOpen;
  int reconParamsCnt;
  int *reconParams;
  int (*reconOpenFn)(RECONOPENARGS);
  int (*reconNewParamFn)(RECONNEWPARAMARGS);
  int (*reconCloseFn)(RECONCLOSEARGS);
  int (*reconNewFrameFn)(RECONNEWFRAMEARGS);
  int (*reconNewFrameSyncFn)(RECONNEWFRAMESYNCARGS);
  int (*reconStartFrameFn)(RECONSTARTFRAMEARGS);
  int (*reconNewSlopesFn)(RECONNEWSLOPESARGS);
  int (*reconEndFrameFn)(RECONENDFRAMEARGS);
  int (*reconFrameFinishedSyncFn)(RECONFRAMEFINISHEDSYNCARGS);
  int (*reconFrameFinishedFn)(RECONFRAMEFINISHEDARGS);
  int (*reconOpenLoopFn)(RECONOPENLOOPARGS);
  int (*reconCompleteFn)(RECONCOMPLETEARGS);
  void *reconHandle;


  
  void *bufferLib;
  char *bufferName;
  char *bufferNameOpen;
  int bufferParamsCnt;
  int *bufferParams;
  int (*bufferOpenFn)(BUFFEROPENARGS);//char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **handle,int nthreads,unsigned int frameno);
  int (*bufferNewParamFn)(BUFFERNEWPARAMARGS);//void *bufferHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr);
  int (*bufferCloseFn)(BUFFERCLOSEARGS);//(void **bufferHandle);
  int (*bufferUpdateFn)(BUFFERUPDATEARGS);//(void **bufferHandle);
  void *bufferHandle;
  int bufferUseSeq;
  

  pthread_mutex_t libraryMutex;
  pthread_mutex_t calibrateMutex;
  //int fftIndexSize;
  //int *fftIndex;
  //fftwf_plan *fftPlanArray;//array holding all the fftw plans
  //fftwf_plan **ifftPlanArray;//array holding all the inverse fftw plans
  unsigned int *camframeno;//this frameno is generated by the camera, 1 entry per cam
  unsigned int *centframeno;//this frameno is generated by the wpu, 1 entry per wpucam
  unsigned int *calframeno;
  unsigned int *reconframeno;
  unsigned int *mirrorframeno;
  unsigned int *figureframeno;
  unsigned int *bufferframeno;
  int camframenoSize;
  int calframenoSize;
  int centframenoSize;
  int reconframenoSize;
  int mirrorframenoSize;
  int figureframenoSize;
  int bufferframenoSize;
  int camReadCnt;//no of threads that have read camera this frame.
  int pxlCentInputError;//set if error raised.
  char *shmPrefix;
  int ncpu;
  //int *ncentsList;
  int figureFrame;
  //float fluxThreshold;
  //float *fluxThresholdArr;
  int printUnused;//whether to print a message about unused parameters - set if debugging a config file...
  int *switchRequestedPtr;
  int setFrameno;
  int *subapLocationMem;
  int calCentReady;
  pthread_mutex_t calCentMutex;
  pthread_cond_t calCentCond;
  char *mainGITID;
  int *subapAllocationArr;
  int *adapWinShiftCnt;
  int resetAdaptiveWin;
  int circAddFlags;
  int forceWriteAll;
  int rtcErrorBufNStore;
  int rtcPxlBufNStore;
  int rtcCalPxlBufNStore;
  int rtcCentBufNStore;
  int rtcMirrorBufNStore;
  int rtcActuatorBufNStore;
  int rtcSubLocBufNStore;
  int rtcTimeBufNStore;
  int rtcStatusBufNStore;
  int rtcGenericBufNStore;
  int rtcFluxBufNStore;
#ifdef DOTIMING
  double endFrameTimeSum;
  int endFrameTimeCnt;
  double partialReconstructTimeSum;
  int partialReconstructTimeCnt;
  double subtractBgTimeSum;
  int subtractBgTimeCnt;
  double calcCentroidTimeSum;
  int calcCentroidTimeCnt;
  double applyFlatFieldTimeSum;
  int applyFlatFieldTimeCnt;
  double applyThresholdTimeSum;
  int applyThresholdTimeCnt;
  double subapPxlCalibrationTimeSum;
  int subapPxlCalibrationTimeCnt;
  double applyPhaseTimeSum;
  int applyPhaseTimeCnt;
  double sendActuatorsTimeSum;
  int sendActuatorsTimeCnt;
  double TimeSum;
  int TimeCnt;
#endif//DOTIMING
}globalStruct;

/**
   holds thread specific information.
*/
typedef struct{
  float *subap;//array holding the subap.  Should be malloced to something large initially... (ie large enough to hold all the pixels a subap could envisage requiring).
  int subapSize;//number of elements subap can hold.
  float *sort;//array to hold sorted subap if using only brightest N pixels.
  int threadno;//the number of this thread.
  int cursubindx;//current subap index (cursubx + nsubx*cursuby)
  int centindx;//current centroid indx for this thread.
  int curnpxlx;//x size of current subap
  int curnpxly;//y size of current subap
  int curnpxl;//number of pxls in current subap
  infoStruct *info;//most things readonly by the threads (except eg centroids, etc where we know conflict won't occur.)
  //infoStruct *infobuf;//the double buffered - these are swapped around.
  globalStruct *globals;
  //int mybuf;//0 or 1, which buffer is currently in use.
  //int frameno;//number of current frame this thread is processing...
  int threadAffinity;
  int threadPriority;
  int nsubapsDoing;//the number of active subaps the thread is doing this time
  int nsubapsProcessing;//the number of active and inactive subaps...
  int nsubs;//number of subapertures that must arrived before the centroiding livbrary can continue.
  int frameFinished;
  int err;//an error has occurred during processing.
}threadStruct;


#ifdef DEBUG
#define dprintf(...) printf(__VA_ARGS__);fflush(NULL)
#else
#define dprintf(...)
#endif
void writeErrorVA(circBuf *rtcErrorBuf,int errnum,int frameno,char *txt,...);
int freezeParamBuf(paramBuf *b1,paramBuf *b2);
int getSwitchRequested(globalStruct *globals);
int prepareActuators(globalStruct *glob);
int processFrame(threadStruct *threadInfo);
int figureThread(PostComputeData *p);
void setGITID(globalStruct *glob);
int openLibraries(globalStruct *glob,int getlock);
#endif
