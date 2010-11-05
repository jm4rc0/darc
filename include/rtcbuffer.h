#include "circ.h"
#include "arrayStruct.h"
#include "buffer.h"
#define BUFFERCLOSEARGS void **bufferHandle
int bufferClose(BUFFERCLOSEARGS);
#define BUFFERNEWPARAMARGS void *bufferHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr,paramBuf *inactive
int bufferNewParam(BUFFERNEWPARAMARGS);//Can do finalisation of previous frame if required.
#define BUFFEROPENARGS char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **handle,int nthreads,unsigned int frameno,unsigned int** bufferframeno,int *bufferframenosize,paramBuf *inactive
int bufferOpen(BUFFEROPENARGS);
#define BUFFERUPDATEARGS void *bufferHandle
int bufferUpdate(BUFFERUPDATEARGS);

