#include "circ.h"
#include "arrayStruct.h"
#include "buffer.h"
int figureOpen(char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **handle,int nthreads,int frameno,int totCents);
int figureClose(void **figureHandle);
int figureNewParam(void *figureHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr);
