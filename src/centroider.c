/**
   The code here is used to create a shared object library, which can then be swapped around depending on which cameras you have in use, ie you simple rename the camera file you want to camera.so (or better, change the soft link), and restart the coremain.

The library is written for a specific camera configuration - ie in multiple camera situations, the library is written to handle multiple cameras, not a single camera many times.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rtccentroider.h"
/**
   Find out if this SO library supports your camera.

*/

int centQuery(char *name){
#ifdef OLD
  if(strcmp(name,"dummy")==0)
    return 0;
  return 1;
#else
  return 0;
#endif
}
/**
   Open a centroid camera of type name.  Args are passed in a float array of size n, which can be cast if necessary.  Any state data is returned in centHandle, which should be NULL if an error arises.
   pxlbuf is the array that should hold the data. The library is free to use the user provided version, or use its own version as necessary (ie a pointer to physical memory or whatever).  It is of size npxls*sizeof(short).
   ncam is number of cameras, which is the length of arrays pxlx and pxly, which contain the dimensions for each camera.
   Name is used if a library can support more than one camera.
   frameno is a pointer which should be set with the current frame number when written to.
*/

int centOpen(char *name,int n,int *args,void **centHandle,float *pxlbuf,int ncam,int *nsubs,int *frameno,char *buf,circBuf *rtcErrorBuf){
  printf("Initialising centroid camera %s\n",name);
  if(centQuery(name)){
    printf("Error: Centroid Camera %s not found in this centroid camera library\n",name);
    return 1;
  }
  return 0;
}

/**
   Close a centroid camera of type name.  Args are passed in the float array of size n, and state data is in centHandle, which should be freed and set to NULL before returning.
*/
int centClose(void **centHandle){
  printf("Closing centroid camera\n");
  return 0;
}
/**
   Start the centroid camera framing, using the args and centroid camera handle data.
*/
int centStartFraming(int n,int *args,void *centHandle){
  printf("Framing centroid camera\n");
  return 0;
}
/**
   Stop the centroid camera framing
*/
int centStopFraming(void *centHandle){
  printf("Stopping framing\n");
  return 0;
}

/**
   Can be called to get the latest iamge taken by the centroid camera
*/
int centGetLatest(void *centHandle){
  printf("Getting latest frame\n");
  return 0;
}

/**
   Called when we're starting processing the next frame.  This doesn't actually wait for any pixels.
*/
int centNewFrame(void *centHandle){
  //printf("centNewFrame\n");
  return 0;
}
/**
   Called when new parameters are available.
*/
int centNewParam(void *centHandle,char *buf,unsigned int frameno){
  return 0;
}
/**
   Wait until n pixels of the current frame to arrive.
   Note, it is just possible I think that n could be <0 or > than total number of pixels on CCD.  If this is a problem you should perform error checking.
*/
int centWaitPixels(int n,int cam,void *centHandle){
  //printf("centWaitPixels %d, centroid camera %d\n",n,cent);
  return 0;
}
