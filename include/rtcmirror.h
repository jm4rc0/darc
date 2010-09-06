#include "circ.h"
int mirrorQuery(char *name);
int mirrorOpen(char *name,int narg,int *args, int nacts,void **mirrorHandle,circBuf *rtcErrorBuf,circBuf *rtcActuatorBuf,unsigned int frameno,char *buf);
int mirrorClose(void **mirrorHandle);
int mirrorSend(void *mirrorHandle,int n,float *data,unsigned int frameno,double timestamp,int err);
int mirrorNewParam(void *mirrorHandle,char *buf,unsigned int frameno);
