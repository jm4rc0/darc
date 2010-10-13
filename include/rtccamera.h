#include "circ.h"
#include "arrayStruct.h"
#include "buffer.h"
/**
   The code here is used to create a shared object library, which can then be swapped around depending on which cameras you have in use, ie you simple rename the camera file you want to camera.so (or better, change the soft link), and restart the coremain.

The library is written for a specific camera configuration - ie in multiple camera situations, the library is written to handle multiple cameras, not a single camera many times.
*/

/**
   Find out if this SO library supports your camera.
*/

int camQuery(char *name);
/**
   Open a camera of type name.  Args are passed in a int32 array of size n, which can be cast if necessary.  Any state data is returned in camHandle, which should be NULL if an error arises.
   pxlbuf is the array that should hold the data. The library is free to use the user provided version, or use its own version as necessary (ie a pointer to physical memory or whatever).  It is of size npxls*sizeof(short).
   ncam is number of cameras, which is the length of arrays pxlx and pxly, which contain the dimensions for each camera.
   Name is used if a library can support more than one camera.

*/
#ifdef __cplusplus
extern "C" 
#endif
int camOpen(char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **handle,int npxls,short *pxlbuf,int ncam,int *pxlx,int* pxly,int* frameno);

/**
   Close a camera of type name.  Args are passed in the int32 array of size n, and state data is in camHandle, which should be freed and set to NULL before returning.
*/
#ifdef __cplusplus
extern "C" 
#endif
int camClose(void **camHandle);
/**
   Start the camera framing, using the args and camera handle data.
*/
#ifdef __cplusplus
extern "C" 
#endif
int camStartFraming(int n,int *args,void *camHandle);
/**
   Stop the camera framing
*/
#ifdef __cplusplus
extern "C" 
#endif
int camStopFraming(void *camHandle);

/**
   Can be called to get the latest iamge taken by the camera
   Actuatlly, not used...
*/
int camGetLatest(void *camHandle);

/**
   Called when we're starting processing the next frame.  This doesn't actually wait for any pixels.
*/
#ifdef __cplusplus
extern "C" 
#endif
int camNewFrame(void *camHandle);

/**
   Wait for the next n pixels of the current frame to arrive.
*/
#ifdef __cplusplus
extern "C" 
#endif
int camWaitPixels(int n,int cam,void *camHandle);
/**
   New parameters...
*/
#ifdef __cplusplus
extern "C" 
#endif
int camNewParam(void *camHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr);
