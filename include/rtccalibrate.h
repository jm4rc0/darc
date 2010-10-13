#include "circ.h"
#include "arrayStruct.h"
#include "buffer.h"
int calibrateClose(void **calibrateHandle);
int calibrateNewParam(void *calibrateHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr);//Can do finalisation of previous frame if required.
int calibrateOpen(char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **handle,int nthreads,int frameno);
int calibrateNewFrame(void *calibrateHandle);//non-subap thread (once)  Called after calibrateNewParam in the case that there is a param buffer swap.  Can do finalisation of previous frame if required, and if not already done by calibrateNewParam.
int calibrateStartFrame(void *calibrateHandle,int cam,int threadno);//subap thread (once per thread)
int calibrateNewSubap(void *calibrateHandle,int cam,int threadno,int cursubindx,float **subap,int *subapSize);//subap thread
int calibrateEndFrame(void *calibrateHandle,int cam,int threadno,int err);//subap thread (once per thread)
int calibrateFrameFinishedSync(void *calibrateHandle,int err);//subap thread (once)
int calibrateFrameFinished(void *calibrateHandle,int err);//non-subap thread (once)
int calibrateOpenLoop(void *calibrateHandle);
