#include "circ.h"
int mirrorQuery(char *name);
int mirrorOpen(char *name,int n,int *args,char *buf,circBuf *rtcErrorBuf,char *prefix,void **handle,int nacts,circBuf *rtcActuatorBuf,unsigned int frameno);
int mirrorClose(void **mirrorHandle);
int mirrorSend(void *mirrorHandle,int n,float *data,unsigned int frameno,double timestamp,int err);
int mirrorNewParam(void *mirrorHandle,char *buf,unsigned int frameno);
