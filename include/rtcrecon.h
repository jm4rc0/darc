#include "circ.h"
#include "arrayStruct.h"
#include "buffer.h"
int reconClose(void **reconHandle);
int reconNewParam(void *reconHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr,int totCents);
int reconOpen(char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **handle,int nthreads,int frameno,int totCents);
int reconNewFrame(void *reconHandle,float *dmCommand);//non-subap thread (once)
int reconStartFrame(void *reconHandle,int threadno);//subap thread (once per thread)
int reconNewSlopes(void *reconHandle,int centindx,int threadno,int nsubapsDoing,float *centroids,float *dmCommand);//subap-thread
int reconEndFrame(void *reconHandle,int threadno,float *centroids,float *dmCommand,int err);//subap thread (once per thread)
int reconFrameFinishedSync(void *reconHandle,float *centroids,float *dmCommand,int err);//subap thread (once)
int reconFrameFinished(void *reconHandle,float *dmCommand,int err);//non-subap thread (once)
int reconOpenLoop(void *reconHandle);
