#include "circ.h"
#include "arrayStruct.h"
#include "buffer.h"
#define RECONCLOSEARGS void **reconHandle
int reconClose(RECONCLOSEARGS);
#define RECONNEWPARAMARGS void *reconHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr,int totCents
int reconNewParam(RECONNEWPARAMARGS);
#define RECONOPENARGS char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **handle,int nthreads,unsigned int frameno,unsigned int **reconframeno,int *reconframenoSize,int totCents
int reconOpen(RECONOPENARGS);
#define RECONNEWFRAMEARGS void *reconHandle,unsigned int frameno,double timestamp
int reconNewFrame(RECONNEWFRAMEARGS);//non-subap thread (once)
#define RECONNEWFRAMESYNCARGS void *reconHandle,unsigned int frameno,double timestamp
int reconNewFrameSync(RECONNEWFRAMESYNCARGS);//subap thread (once)
#define RECONSTARTFRAMEARGS void *reconHandle,int cam,int threadno
int reconStartFrame(RECONSTARTFRAMEARGS);//subap thread (once per thread)
#define RECONNEWSLOPESARGS void *reconHandle,int cam,int centindx,int threadno,int nsubapsDoing
int reconNewSlopes(RECONNEWSLOPESARGS);//subap-thread
#define RECONENDFRAMEARGS void *reconHandle,int cam,int threadno,int err
int reconEndFrame(RECONENDFRAMEARGS);//subap thread (once per thread)
#define RECONFRAMEFINISHEDSYNCARGS void *reconHandle,int err,int forcewrite
int reconFrameFinishedSync(RECONFRAMEFINISHEDSYNCARGS);//subap thread (once)
#define RECONFRAMEFINISHEDARGS void *reconHandle,int err
int reconFrameFinished(RECONFRAMEFINISHEDARGS);//non-subap thread (once)
#define RECONOPENLOOPARGS void *reconHandle
int reconOpenLoop(RECONOPENLOOPARGS);
#define RECONCOMPLETEARGS void *reconHandle
int reconComplete(RECONCOMPLETEARGS);
