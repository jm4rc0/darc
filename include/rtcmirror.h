#include "circ.h"
#include "arrayStruct.h"
#include "buffer.h"
int mirrorQuery(char *name);
int mirrorOpen(char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **handle,int nacts,circBuf *rtcActuatorBuf,unsigned int frameno);
int mirrorClose(void **mirrorHandle);
int mirrorSend(void *mirrorHandle,int n,float *data,unsigned int frameno,double timestamp,int err);
int mirrorNewParam(void *mirrorHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr);
