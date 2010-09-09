#include "circ.h"
int figureOpen(char *name,int n,int *args,char *buf,circBuf *rtcErrorBuf,char *prefix,void **handle,int nthreads,int frameno,int totCents);
int figureClose(void **figureHandle);
int figureNewParam(void *figureHandle,char *buf,unsigned int frameno);
