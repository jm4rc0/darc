#include "circ.h"
int figureOpen(char *name,int narg,int *args, int nacts,pthread_mutex_t m,pthread_cond_t cond,float **actsRequired,unsigned int *frameno,void **figureHandle,char *buf,circBuf *rtcErrorBuf);
int figureClose(void **figureHandle);
int figureNewParam(void *figureHandle,char *buf,unsigned int frameno);
