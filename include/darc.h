#define USECOND
#include <fftw3.h>
#include "circ.h"
#ifndef DARC_H
#define DARC_H
#define STATUSBUFSIZE 256
#define ERRORBUFSIZE 128
//#define PROJID 98
//#define NCAMERAPARAMETERS 10//the max number of parameters for the camera library.
//#define MAXSUBAPSIZE 64//the maximum linear size of a subap. Used when allocating threadInfo->subap, and the fftPlanArray.
enum WindowModes{WINDOWMODE_BASIC,WINDOWMODE_ADAPTIVE,WINDOWMODE_GLOBAL,WINDOWMODE_ERROR};
enum CentroidModes{CENTROIDMODE_WPU,CENTROIDMODE_COG,CENTROIDMODE_GAUSSIAN,CENTROIDMODE_CORRELATIONCOG,CENTROIDMODE_CORRELATIONGAUSSIAN,CENTROIDMODE_ERROR};
enum CorrelationThresholdType{CORR_ABS_SUB,CORR_ABS_ZERO,CORR_FRAC_SUB,CORR_FRAC_ZERO};

enum Errors{CLIPERROR,PARAMERROR,CAMSYNCERROR,CAMGETERROR,SLOPELINERROR};//,CAMOPENERROR,CAMFRAMEERROR,MIRROROPENERROR};
//various different reconstruction modes when not using kalman.
//typedef enum{RECONMODE_SIMPLE,RECONMODE_TRUTH,RECONMODE_OPEN,RECONMODE_OFFSET}ReconModeType;

/**
   Hold the parameter buffer shared memory pointer and semaphore.
*/
typedef struct{
  char *buf;
#ifdef USECOND
  char *arr;//everything (header+buf)
  int *hdr;
  pthread_mutex_t *condmutex;
  pthread_cond_t *cond;
#else
  int semid;
#endif
}paramBuf;

/**
   data for the pre/post computation thread.
*/
typedef struct{
  //float *Xpred;
  //int kalmanPhaseSize;
  //float *kalmanAtur;
  //int kalmanUsed;
  //int kalmanReset;
  float *dmCommand;
  float *dmCommandSave;
  float *dmCommandFigure;
  int nclipped;
  int maxClipped;
  //float *Xdmc;
  float *calpxlbuf;//for storing the calibrated pixels for circular buffer
  float *corrbuf;//for storing the correlated images for circular buffer
  float *centroids;
  int nacts;
  int totCents;
  int totPxls;
  //float *kalmanInvN;
  //unsigned short *actsSent;
  //float *latestDmCommand;
  //int actMax;
  //int actMin;
  //float bleedGainOverNact;
  //float midRangeTimesBleed;
  float *userActs;
  float *userActsMask;//a mask to apply to the computed actuator vals
  int addUserActs;
  int userActSeqLen;//no of entries in userActs, each of size nacts.
  int *userActSeq;//NULL or size userActSeqLen.
  int seqCnt;
  int actCnt;
  //int usingDMC;
  void *mirrorHandle;
  int (*mirrorSendFn)(void *mirrorHandle,int n,float *data,unsigned int frameno,double timestamp,int err);
  int (*reconFrameFinishedFn)(void *reconHandle,float *dmCommand,int err);
  int (*reconOpenLoopFn)(void *reconHandle);


  int thisiter;
  enum WindowModes windowMode;
  int *subapLocation;
  int closeLoop;
  int pxlCentInputError;
  double timestamp;
  int *recordCents;
  float *pmx;
  //float *v0;
  int pmxSize;
  float *flux;
  int *averageImg;
  int nAvImg;
  float *avCalPxlBuf;//for optionally storing an averaged cal pxls over several frames.
  int avCalPxlBufSize;
  int *averageCent;
  int nAvCent;
  float *avCentBuf;
  int avCentBufSize;
  float *actsRequired;//used when running as a figure sensor, should be set to the latest DM demands from the RTC.  This should be allocated by the .so library responsible for reading in the DM demands, and free'd when the library is closed.
  pthread_mutex_t actsRequiredMutex;//used in figure sensor mode
  pthread_mutex_t *mirrorMutex;
  pthread_cond_t actsRequiredCond;//used in figure sensor mode
  void *mirrorLib;
  int go;
  //float adaptiveWinGain;
  float figureGain;
  float *figureGainArr;
}PostComputeData;

typedef struct{
  pthread_mutex_t prepMutex;
  pthread_mutex_t postMutex;
  pthread_cond_t prepCond;
  pthread_cond_t postCond;
  //pthread_cond_t dmCond;
  //pthread_mutex_t dmMutex;
  pthread_cond_t pxlcentCond;
  pthread_mutex_t pxlcentMutex;
  //int dmReady;
  int pxlcentReady;
  //ReconModeType reconMode;
  float *dmCommand;
  int nacts;
  //float *latestDmCommand;
  //float *v0;
  //float *gainE;
  pthread_t threadid;
  //int mybuf;
  //int kalmanUsed;
  //int kalmanReset;
  //float *Xdmc;
  //float *Xpred;
  //float *precompDMXpred;
  //int kalmanPhaseSize;
  int totCents;
  //float *kalmanHinfDM;
  //float *calpxlbuf;
  //float *centroids;
  //int *actsSent;
  //float *kalmanAtur;
  //int actMax;
  //float bleedGainOverNact;
  //float midRangeTimesBleed;
  //int *userActs;
  //int nclipped;
  //int usingDMC;
  //float *kalmanInvN;
  int readyToStart;

  PostComputeData post;
}PreComputeData;

/**
   holds internal memory allocations
*/
typedef struct{
  float *flux;
  float *centroids;
  //float *precompDMXpred;
  float *dmCommand;
  float *dmCommandSave;
  float *dmCommandFigure;
  //float *latestDmCommand;
  //unsigned short *actsSent;
  //float *Xpred;
  //float *Xdmc;
  float *calpxlbuf;
  float *corrbuf;
  int fluxSize;
  int centroidsSize;
  //int precompDMXpredSize;
  int dmCommandSize;
  //int latestDmCommandSize;
  //int actsSentSize;
  //int XpredSize;
  //int XdmcSize;
  int calpxlbufSize;
  int corrbufSize;
  int *subapLocation;
  int subapLocationSize;
  int *adaptiveCentPos;
  int adaptiveCentPosSize;
  float *adaptiveWinPos;
  int adaptiveWinPosSize;
}arrayStruct;

/**
   holds info relevent for one camera (ie one frame grabber), common to all the threads processing for this camera
*/
typedef struct{//one array for each camera, double buffered...
  int nsuby;
  int nsubx;
  int npxlx;
  int npxly;
  int *nsubyList;
  int *nsubxList;
  int *npxlxList;
  int *npxlyList;
  int npxlCum;
  int nsubaps;
  int totPxls;
  
  int totCents;
  //int nusedSubaps;//number of subaps valid for this camera
  int centCumIndx;//cumulative centroid number for the first centroid of this camera.
  int subCumIndx;//cumulative subap number for the first subap of this camera.
  int cam;//this current number
  int ncam;//number of cameras...
  int *subflag;//flat as to whether subap is valid or not.
  int *subapFlagArr;//subflag indexes into this array.
  int *pxlCnt;
  int nthreads;//number of threads serving this camera.
  int subindx;//index of the next subap to be evaluated...
  int centindx;//index of next centroid to be evaluated.
  float *centroids;//cents for each subap, x,y,x,y...
  float *flux;//flux in each subap
  float *refCents;//reference centroids
  int thresholdAlgo;
  float threshold;
  float *thresholdArr;
  int thresholdArrType;
  float powerFactor;
  float *darkNoise;
  float *darkNoiseArr;
  float *background;
  float *backgroundArr;
  float *flatField;
  float *flatFieldArr;
  float *pxlweight;
  float *pxlweightArr;
  pthread_mutex_t subapMutex;
  pthread_mutex_t startInfoMutex;//mutex obtained during buffer swap
  int threadInfoCount;//used during buffer swap to determine if this is the first thread for this camera.
  int frameFinished;//[2];
  int *subapLocation;//either points to realSubapLocation or is own array, when using adaptive windowing.
  int *realSubapLocation;//user supplied subap locations
  float *centWeighting;
  //int kalmanReset;
  //int kalmanUsed;
  //float *Xdmc;
  //int kalmanPhaseSize;
  //float *Xpred;
  //float *precompDMXpred;
  //float *kalmanHinfDM;
  //float *kalmanInvN;
  //float *kalmanAtur;
  //float *kalmanHinfT;
  //float *rmxT;//reconstructor, modified by gain.
  float *dmCommand;//this is float here, but converted to int for actsSent.
  //float *latestDmCommand;
  //unsigned short *actsSent;
  float *calpxlbuf;
  float *corrbuf;
  //int usingDMC;
  int nacts;
  enum WindowModes windowMode;
  enum CentroidModes centroidMode;
  short *pxlbuf;//not sure how to handle this: this is the pxlbuf for this cam
  //float *centbuf;//this is the centbuf for this cam - when using WPU centroiding
  //int useWPU;
  int go;//whether to run or not.
  int id;//for debugging
  int threadCountFinished;//count of number of threads finished for this camera
  //ReconModeType reconMode;
  //float *gainE;//used in reconstruction
  //float *v0;//used in reconstruction
  //float bleedGainOverNact;
  //float midRangeTimesBleed;
  float *userActs;
  float *userActsMask;
  int addUserActs;
  //int actMax;
  //int actMin;
  int userActSeqLen;
  int *userActSeq;
  int *recordCents;
  int pause;
  int closeLoop;
  int printTime;
  int *fakeCCDImage;//can be specified and this is used instead of camera data
  int *threadAffinityList;
  int *threadPriorityList;
  int delay;//artificial delay to slow rtc down.
  int maxClipped;
  //int reconMVSize;
  //int *reconMVSizeList;
  //pthread_mutex_t reconMVMutex;
  //int *reconMVCnt;
  //int reconMVCntSize;
  //int stopCamerasFraming;
  //int startCamerasFraming;
  //int openCameras;
  //int closeCameras;
  int *camerasOpen;
  int *camerasFraming;
  int *cameraParams;
  int cameraParamsCnt;
  int *centroidersOpen;
  int *centroidersFraming;
  int *centroidersParams;
  int centroidersParamsCnt;
  int *mirrorOpen;
  int *mirrorParams;
  int mirrorParamsCnt;
  int *reconlibOpen;
  int *figureOpen;
  int *figureParams;
  int figureParamsCnt;
  float adaptiveWinGain;
  enum CorrelationThresholdType correlationThresholdType;//type of threshold applied when doing correlation centroiding
  float correlationThreshold;//value of threshold when doing correlation centroiding.
  int resetAdaptiveWindows;
  float *adaptiveWinPos;
  int *adaptiveCentPos;
  float *fftCorrelationPattern;//the spot PSF array, FFT'd in HC format, and placed as per subapLocation...
  float *fftCorrelationPatternArr;
  int nsubapsTogether;//number of subaps evaluated by a thread each time.
  int nsteps;//number of iters to do before pausing (continuous if <=0)
  int pxlCentInputError;//set if error raised.
  int *averageImg;//no of frames of calpxl to average and send to generic stream
  int nAvImg;//store above.
  int *averageCent;
  int nAvCent;
  float *calmult;
  float *calsub;
  float *calthr;
  float *centCalData;
  float *centCalSteps;
  int *centCalBounds;
  int centCalnsteps;
  int useBrightest;
  int *useBrightestArr;
  float figureGain;
  float *figureGainArr;
  int maxAdapOffset;
  int nAdaptiveGroups;
  float *groupSumX;
  float *groupSumY;
  int *groupSum;
  int *adaptiveGroup;
  int adaptiveGroupSize;

}infoStruct;

/**
   holds global information.
*/
typedef struct{//info shared between all threads.
  int doswitch;//[2];//whether buffer needs to bufswap...
  int *switchRequested;//set from python... via shared memory for the c version
  pthread_mutex_t startMutex;//[2];
  pthread_mutex_t startFirstMutex;
  pthread_mutex_t endMutex;//[2];
  pthread_mutex_t frameRunningMutex;//[2];
  pthread_cond_t frameRunningCond;//[2];
  //int *updateBuf;//size ncam, whether the buffer for this camera needsupdating.
  int threadCount;//[2];
  int threadCountFinished;//[2];
  arrayStruct *arrays;//[2];
  int niters;
  paramBuf *buffer[2];//the SHM buffer containing all the config data.
  int ncam;//set at start and never changed...
  int curBuf;
  //int frameRunning[2];
  //int frameBlocking[2];
  int nthreads;
  int *bufferHeaderIndex;
  //pthread_mutex_t bufMutex;
  pthread_mutex_t camMutex;
  
  //int bufBlocking;
  //pthread_cond_t bufCond;
  void **threadInfoHandle;
  //int bufInUse;//threads are currently reading from the buffer
  PreComputeData *precomp;
  pthread_t *threadList;
  //for timings (mean, stdev, maximum - jitter)
  double sumtime;
  double sum2time;
  double maxtime;
  double frameTime;
  int maxtimeiter;
  int thisiter;
  int camiter;//the last frame number received and accepted from camera
  int ntime;
  int *ppause;//pointer to the pause entry in the shm buffer, or NULL.
  int buferr;//set when an error occurs during buffer swap.
  //struct timeval starttime;
  double starttime;
  int nclipped;
  char statusBuf[STATUSBUFSIZE];
  circBuf *rtcPxlBuf;
  circBuf *rtcCalPxlBuf;
  circBuf *rtcCorrBuf;
  circBuf *rtcCentBuf;
  circBuf *rtcMirrorBuf;
  circBuf *rtcActuatorBuf;
  circBuf *rtcStatusBuf;
  circBuf *rtcTimeBuf;
  circBuf *rtcErrorBuf;
  circBuf *rtcSubLocBuf;
  circBuf *rtcGenericBuf;
  circBuf *rtcFluxBuf;
  int *threadAffinityList;
  int *threadPriorityList;
  int prepThreadAffinity;//thread affinity of the prepareActuators thread.
  int prepThreadPriority;//thread affinity of the prepareActuators thread.
  void *camHandle;
  void *mirrorHandle;
  void *centHandle;
  void *figureHandle;
  int camFraming;
  int centFraming;
  short *pxlbufs;
  int pxlbufsSize;
  //float *centbufs;
  //int centbufsSize;
  pthread_t fftPlanThreadid;
  int fftPlanningDone;
  //now the camera dynamic library functions...
  void *cameraLib;
  char *cameraName;
  char *cameraNameOpen;
  //These should agree with rtccamera.h...
  int (*camOpenFn)(char *name,int n,int *args,char *buf,circBuf *rtcErrorBuf,char *prefix,void **camHandle,int npxls,short *pxlbuf,int ncam,int *pxlx,int* pxly,int* frameno);
  int (*camCloseFn)(void **camHandle);
  int (*camStartFramingFn)(int n,int *args,void *camHandle);
  int (*camStopFramingFn)(void *camHandle);
  int (*camNewFrameFn)(void *camHandle);
  int (*camWaitPixelsFn)(int n,int cam,void *camHandle);
  int (*camNewParamFn)(void *camHandle,char *buf,unsigned int frameno);
  //and the mirror dynamic library...
  char *mirrorName;
  char *mirrorNameOpen;
  void *mirrorLib;
  int (*mirrorOpenFn)(char *name,int n,int *args,char *buf,circBuf *rtcErrorBuf,char *prefix,void **camHandle,int nacts,circBuf *rtcActuatorBuf,unsigned int frameno);
  int (*mirrorCloseFn)(void **mirrorHandle);
  int (*mirrorNewParamFn)(void *mirrorHandle,char *buf,unsigned int frameno);
  int (*mirrorSendFn)(void *mirrorHandle,int n,float *data,unsigned int frameno,double timestamp,int err);
  //And the centroider dynamic library functions...
  char *centName;
  char *centNameOpen;
  void *centLib;
  int (*centOpenFn)(char *name,int n,int *args,char *buf,circBuf *rtcErrorBuf,char *prefix,void **handle,float *centbufs,int ncam,int *ncents,int* frameno);
  int (*centNewParamFn)(void *centHandle,char *buf,unsigned int frameno);
  int (*centCloseFn)(void **centHandle);
  int (*centStartFramingFn)(int n,int *args,void *centHandle);
  int (*centStopFramingFn)(void *centHandle);
  int (*centNewFrameFn)(void *centHandle);
  int (*centWaitPixelsFn)(int n,int cam,void *centHandle);
  //and the figure sensing dynamic library...
  char *figureName;
  char *figureNameOpen;
  void *figureLib;
  int (*figureOpenFn)(char *name,int n,int *args,char *buf,circBuf *rtcErrorBuf,char *prefix,void **handle,int nacts,pthread_mutex_t m,pthread_cond_t cond,float **actsRequired,unsigned int *frameno);
  int (*figureCloseFn)(void **figureHandle);
  int (*figureNewParamFn)(void *figureHandle,char *buf,unsigned int frameno);
  void *reconLib;
  char *reconName;
  char *reconNameOpen;
  int reconParamsCnt;
  int *reconParams;
  int (*reconOpenFn)(char *name,int n,int *args,char *buf,circBuf *rtcErrorBuf,char *prefix,void **handle,int nthreads,int frameno,int totCents);
  int (*reconNewParamFn)(char *buf,void *reconHandle,unsigned int frameno,int totCents);
  int (*reconFreeFn)(void **reconHandle);
  int (*reconNewFrameFn)(void *reconHandle,float *dmCommand);
  int (*reconStartFrameFn)(void *reconHandle,int threadno);
  int (*reconNewSlopesFn)(void *reconHandle,int centindx,int threadno,int nsubapsDoing,float *centroids,float *dmCommand);
  int (*reconEndFrameFn)(void *reconHandle,int threadno,float *centroids,float *dmCommand,int err);
  int (*reconFrameFinishedSyncFn)(void *reconHandle,float *centroids,float *dmCommand,int err);
  int (*reconFrameFinishedFn)(void *reconHandle,float *dmCommand,int err);
  int (*reconOpenLoopFn)(void *reconHandle);

  //int (*reconInitFn)(char *buf,void *threadInfo);
  //int (*reconNewParamFn)(char *buf,void *threadInfo);
  //int (*reconFreeFn)(void *threadInfo);
  //int (*reconNewFrameFn)(void *glob);
  //int (*reconStartFrameFn)(void *threadInfo);
  //int (*reconNewSlopesFn)(void *threadInfo);
  //int (*reconEndFrameFn)(void *threadInfo);
  //int (*reconFrameFinishedSyncFn)(void *threadInfo);
  //int (*reconFrameFinishedFn)(void *glob);
  //int (*reconOpenLoopFn)(void *glob);
  void *reconStruct;
  pthread_mutex_t mirrorMutex;
  pthread_mutex_t reconMutex;
  int fftIndexSize;
  int *fftIndex;
  fftwf_plan *fftPlanArray;//array holding all the fftw plans
  //fftwf_plan **ifftPlanArray;//array holding all the inverse fftw plans
  int *camframeno;//this frameno is generated by the camera/wpu, 1 entry per cam
  int camReadCnt;//no of threads that have read camera this frame.
  int pxlCentInputError;//set if error raised.
  char *shmPrefix;
  int ncpu;
  int *ncentsList;
  unsigned int figureFrame[2];
  float fluxThreshold;
  float *fluxThresholdArr;
  int printUnused;//whether to print a message about unused parameters - set if debugging a config file...
  int *switchRequestedPtr;
  int setFrameno;
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
  //int cursuby;//current subap coord being processed (y)
  //int cursubx;//current subap coord being processed (x)
  int cursubindx;//current subap index (cursubx + nsubx*cursuby)
  int centindx;//current centroid indx for this thread.
  int curnpxlx;//x size of current subap
  int curnpxly;//y size of current subap
  int curnpxl;//number of pxls in current subap
  infoStruct *info;//most things readonly by the threads (except eg centroids, etc where we know conflict won't occur.)
  //infoStruct *infobuf;//the double buffered - these are swapped around.
  globalStruct *globals;
  //int mybuf;//0 or 1, which buffer is currently in use.
  int frameno;//number of current frame this thread is processing...
  //int dmCommandSize;
  //float *dmCommand;
  //float *Xpred;
  //int XpredSize;
  int threadAffinity;
  int threadPriority;
  int nsubapsDoing;//the number of active subaps the thread is doing this time
  int err;//an error has occurred during processing.
}threadStruct;

#define NHDR 128
#define HDRROWSIZE 72
#define NBYTES ((int*)(&buf[NHDR*36]))
#define START ((int*)(&buf[NHDR*32]))
//these should be in agreement with buffer.py.

#ifdef DEBUG
#define dprintf(...) printf(__VA_ARGS__);fflush(NULL)
#else
#define dprintf(...)
#endif
void writeErrorVA(circBuf *rtcErrorBuf,int errnum,int frameno,char *txt,...);
#endif
