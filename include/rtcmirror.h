#include "circ.h"
#include "arrayStruct.h"
#include "buffer.h"
#define MIRROROPENARGS char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **handle,int nacts,circBuf *rtcActuatorBuf,unsigned int frameno,unsigned int **mirrorframeno,int *mirrorframenoSize
int mirrorOpen(MIRROROPENARGS);
#define MIRRORCLOSEARGS void **mirrorHandle
int mirrorClose(MIRRORCLOSEARGS);
#define MIRRORSENDARGS void *mirrorHandle,int n,float *data,unsigned int frameno,double timestamp,int err
int mirrorSend(MIRRORSENDARGS);
#define MIRRORNEWPARAMARGS void *mirrorHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr
int mirrorNewParam(MIRRORNEWPARAMARGS);
