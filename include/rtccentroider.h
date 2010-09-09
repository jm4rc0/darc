#include "circ.h"
#include "arrayStruct.h"
/**
   The code here is used to create a shared object library, which can then be swapped around depending on which centroid cameras you have in use, ie you simple rename the centroid camera file you want to centroid camera.so (or better, change the soft link), and restart the coremain.

The library is written for a specific centroid camera configuration - ie in multiple centroid camera situations, the library is written to handle multiple centroid cameras, not a single centroid camera many times.
*/

/**
   Find out if this SO library supports your centroid camera.
*/

int centQuery(char *name);
/**
   Open a centroid camera of type name.  Args are passed in a float array of size n, which can be cast if necessary.  Any state data is returned in centHandle, which should be NULL if an error arises.
   pxlbuf is the array that should hold the data. The library is free to use the user provided version, or use its own version as necessary (ie a pointer to physical memory or whatever).  It is of size npxls*sizeof(short).
   ncam is number of cameras, which is the length of arrays pxlx and pxly, which contain the dimensions for each camera.
   Name is used if a library can support more than one centroid camera.

*/

int centOpen(char *name,int n,int *args,char *buf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **centHandle,float *centbufs,int ncam,int *ncents,int* frameno);

/**
   Called when parameters have changed
*/
int centNewParam(void *centHandle,char *buf,unsigned int frameno,arrayStruct *arr);
/**
   Close a centroid camera of type name.  Args are passed in the float array of size n, and state data is in centHandle, which should be freed and set to NULL before returning.
*/

int centClose(void **centHandle);
/**
   Start the centroid camera framing, using the args and centroid camera handle data.
*/
int centStartFraming(int n,int *args,void *centHandle);
/**
   Stop the centroid camera framing
*/
int centStopFraming(void *centHandle);

/**
   Can be called to get the latest iamge taken by the centroid camera
   Actuatlly, not used...
*/
int centGetLatest(void *centHandle);

/**
   Called when we're starting processing the next frame.  This doesn't actually wait for any pixels.
*/
int centNewFrame(void *centHandle);

/**
   Wait for the next n pixels of the current frame to arrive.
*/
int centWaitPixels(int n,int cam,void *centHandle);
