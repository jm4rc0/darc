#include "circ.h"
#include "arrayStruct.h"
#include "buffer.h"
#define FIGUREOPENARGS char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **handle,int nthreads,unsigned int frameno,unsigned int **figureframeno,int *figureframenoSize,int totCents,int nacts,pthread_mutex_t m,pthread_cond_t cond,float **actsRequired
int figureOpen(FIGUREOPENARGS);
#define FIGURECLOSEARGS void **figureHandle
int figureClose(FIGURECLOSEARGS);
#define FIGURENEWPARAMARGS void *figureHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr
int figureNewParam(FIGURENEWPARAMARGS);
