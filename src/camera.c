/**
   The code here is used to create a shared object library, which can then be swapped around depending on which cameras you have in use, ie you simple rename the camera file you want to camera.so (or better, change the soft link), and restart the coremain.

The library is written for a specific camera configuration - ie in multiple camera situations, the library is written to handle multiple cameras, not a single camera many times.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rtccamera.h"
/**
   Find out if this SO library supports your camera.

*/

int camQuery(char *name){
#ifdef OLD
  if(strcmp(name,"dummy")==0)
    return 0;
  return 1;
#else
  return 0;
#endif
}
/**
   Open a camera of type name.  Args are passed in a int32 array of size n, which can be cast if necessary.  Any state data is returned in camHandle, which should be NULL if an error arises.
   pxlbuf is the array that should hold the data. The library is free to use the user provided version, or use its own version as necessary (ie a pointer to physical memory or whatever).  It is of size npxls*sizeof(short).
   ncam is number of cameras, which is the length of arrays pxlx and pxly, which contain the dimensions for each camera.
   Name is used if a library can support more than one camera.
   frameno is a pointer which should be set with the current frame number when written to.
*/

int camOpen(char *name,int n,int *args,char *buf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **camHandle,int npxls,short *pxlbuf,int ncam,int *pxlx,int* pxly,int* frameno){
  printf("Initialising camera %s\n",name);
  if(camQuery(name)){
    printf("Error: Camera %s not found in this camera library\n",name);
    return 1;
  }
  return 0;
}

/**
   Close a camera of type name.  Args are passed in the int32 array of size n, and state data is in camHandle, which should be freed and set to NULL before returning.
*/
int camClose(void **camHandle){
  printf("Closing camera\n");
  return 0;
}
/**
   New parameters in the buffer (optional)...
*/
int camNewParam(void *camHandle,char *buf,unsigned int frameno,arrayStruct *arr){
  return 0;
}
/**
   Start the camera framing, using the args and camera handle data.
*/
int camStartFraming(int n,int *args,void *camHandle){
  printf("Framing camera\n");
  return 0;
}
/**
   Stop the camera framing
*/
int camStopFraming(void *camHandle){
  printf("Stopping framing\n");
  return 0;
}

/**
   Can be called to get the latest iamge taken by the camera
*/
int camGetLatest(void *camHandle){
  printf("Getting latest frame\n");
  return 0;
}

/**
   Called when we're starting processing the next frame.  This doesn't actually wait for any pixels.
*/
int camNewFrame(void *camHandle){
  //printf("camNewFrame\n");
  return 0;
}

/**
   Wait until n pixels of the current frame to arrive.
   Note, it is just possible I think that n could be <0 or > than total number of pixels on CCD.  If this is a problem you should perform error checking.
*/
int camWaitPixels(int n,int cam,void *camHandle){
  //printf("camWaitPixels %d, camera %d\n",n,cam);
  return 0;
}
