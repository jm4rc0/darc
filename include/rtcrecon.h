//#ifdef NEWRECON
#include "circ.h"
#include "arrayStruct.h"
int reconClose(void **reconHandle);
int reconNewParam(char *buf,void *reconHandle,unsigned int frameno,arrayStruct *arr,int totCents);
int reconOpen(char *name,int n,int *args,char *buf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **handle,int nthreads,int frameno,int totCents);
int reconNewFrame(void *reconHandle,float *dmCommand);
int reconStartFrame(void *reconHandle,int threadno);
int reconNewSlopes(void *reconHandle,int centindx,int threadno,int nsubapsDoing,float *centroids,float *dmCommand);
int reconEndFrame(void *reconHandle,int threadno,float *centroids,float *dmCommand,int err);
int reconFrameFinishedSync(void *reconHandle,float *centroids,float *dmCommand,int err);
int reconFrameFinished(void *reconHandle,float *dmCommand,int err);
int reconOpenLoop(void *reconHandle);
/*#else
#include "darc.h"
int reconFree(threadStruct *threadInfo);
int reconNewParam(char *buf,threadStruct *threadInfo);
int reconInit(char *buf, threadStruct *threadInfo);
int reconNewFrame(globalStruct *glob);
int reconStartFrame(threadStruct *threadInfo);
int reconNewSlopes(threadStruct *threadInfo);
int reconEndFrame(threadStruct *threadInfo);
int reconFrameFinishedSync(threadStruct *threadInfo);
int reconFrameFinished(globalStruct *glob);
int reconOpenLoop(globalStruct *glob);
#endif
*/
