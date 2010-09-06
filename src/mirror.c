/**
   The code here is used to create a shared object library, which can then be swapped around depending on which mirrors/interfaces you have in use, ie you simple rename the mirror file you want to mirror.so (or better, change the soft link), and restart the coremain.

The library is written for a specific mirror configuration - ie in multiple mirror situations, the library is written to handle multiple mirrors, not a single mirror many times.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rtcmirror.h"
/**
   Find out if this SO library supports your camera.

*/

typedef struct{
  int tmp;
}mirrorStruct;

int mirrorQuery(char *name){
  int rtval=0;
#ifdef OLD
  rtval=(strcmp(name,"librtcmirror.so")!=0);
#endif
  return rtval;
}
/**
   Open a camera of type name.  Args are passed in a float array of size n, which can be cast if necessary.  Any state data is returned in camHandle, which should be NULL if an error arises.
   pxlbuf is the array that should hold the data. The library is free to use the user provided version, or use its own version as necessary (ie a pointer to physical memory or whatever).  It is of size npxls*sizeof(short).
   ncam is number of cameras, which is the length of arrays pxlx and pxly, which contain the dimensions for each camera.
   Name is used if a library can support more than one camera.

*/

int mirrorOpen(char *name,int narg,int *args, int nacts,void **mirrorHandle,circBuf *rtcErrorBuf,circBuf *rtcActuatorBuf,unsigned int frameno,char *buf){
  int err;
  //mirrorStruct *m;
  printf("Initialising mirror %s\n",name);
  if((err=mirrorQuery(name)))
    printf("Error - wrong mirror name\n");
  else
    *mirrorHandle=malloc(sizeof(mirrorStruct));
  return err;
}

/**
   Close a camera of type name.  Args are passed in the float array of size n, and state data is in camHandle, which should be freed and set to NULL before returning.
*/
int mirrorClose(void **mirrorHandle){
  printf("Closing mirror\n");
  if(*mirrorHandle!=NULL)
    free(*mirrorHandle);
  *mirrorHandle=NULL;
  return 0;
}
/**
   Return <0 on error, or otherwise, the number of clipped actuators (or zero).
*/
int mirrorSend(void *mirrorHandle,int n,float *data,unsigned int frameno,double timestamp){
  if(mirrorHandle!=NULL)
    printf("Sending %d values to mirror\n",n);
  return 0;
}
int mirrorNewParam(void *mirrorHandle,char *buf,unsigned int frameno){
  if(mirrorHandle!=NULL){
    printf("Changing mirror params\n");
  }
  return 0;
}
