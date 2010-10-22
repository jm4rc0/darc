#include "circ.h"
#include "arrayStruct.h"
#include "buffer.h"
/**
   The code here is used to create a shared object library, which can then be swapped around depending on which centroid cameras you have in use, ie you simple rename the centroid camera file you want to centroid camera.so (or better, change the soft link), and restart the coremain.

The library is written for a specific centroid camera configuration - ie in multiple centroid camera situations, the library is written to handle multiple centroid cameras, not a single centroid camera many times.
*/

/**
   Open a centroid camera of type name.  Args are passed in a float array of size n, which can be cast if necessary.  Any state data is returned in centHandle, which should be NULL if an error arises.
   pxlbuf is the array that should hold the data. The library is free to use the user provided version, or use its own version as necessary (ie a pointer to physical memory or whatever).  It is of size npxls*sizeof(short).
   ncam is number of cameras, which is the length of arrays pxlx and pxly, which contain the dimensions for each camera.
   Name is used if a library can support more than one centroid camera.
   New slopes should be written to arr->wpucentroids.

*/
#define SLOPEOPENARGS char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **slopeHandle,int ncam,int nthreads,unsigned int frameno, unsigned int** slopeframeno,int *slopeframenosize,int totCents
int slopeOpen(SLOPEOPENARGS);

/**
   Called when parameters have changed
*/
#define SLOPENEWPARAMARGS void *slopeHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr,int totCents
int slopeNewParam(SLOPENEWPARAMARGS);
/**
   Close a centroid camera of type name.  Args are passed in the float array of size n, and state data is in centHandle, which should be freed and set to NULL before returning.
*/
#define SLOPECLOSEARGS void **slopeHandle
int slopeClose(SLOPECLOSEARGS);
/**
   Called when we're starting processing the next frame.  This doesn't actually wait for any pixels.
*/
#define SLOPENEWFRAMEARGS void *slopeHandle,unsigned int frameno,double timestamp
int slopeNewFrame(SLOPENEWFRAMEARGS);
#define SLOPENEWFRAMESYNCARGS void *slopeHandle,unsigned int frameno,double timestamp
int slopeNewFrameSync(SLOPENEWFRAMESYNCARGS);
#define SLOPESTARTFRAMEARGS void *slopeHandle,int cam,int threadno
int slopeStartFrame(SLOPESTARTFRAMEARGS);

/**
   If WPU - Wait for the next n subapertures of the current frame to arrive.
   Or, process the data (probably in subap, if you are using a standard calibration)
   Return 1 on error, 0 on okay or -1 if no slopes arrived, but this is not an error.
   The frameno can also be updated.
*/
#define SLOPECALCSLOPEARGS void *slopeHandle,int cam,int threadno,int nsubs,float *subap, int subapSize,int subindx,int slopeindx,int curnpxlx,int curnpxly
int slopeCalcSlope(SLOPECALCSLOPEARGS);//subap thread
#define SLOPEENDFRAMEARGS void *slopeHandle,int cam,int threadno,int err
int slopeEndFrame(SLOPEENDFRAMEARGS);//subap thread (once per thread)
#define SLOPEFRAMEFINISHEDSYNCARGS void *slopeHandle,int err,int forcewrite
int slopeFrameFinishedSync(SLOPEFRAMEFINISHEDSYNCARGS);//subap thread (once)
#define SLOPEFRAMEFINISHEDARGS void *slopeHandle,int err
int slopeFrameFinished(SLOPEFRAMEFINISHEDARGS);//non-subap thread (once)
#define SLOPEOPENLOOPARGS void *slopeHandle
int slopeOpenLoop(SLOPEOPENLOOPARGS);
#define SLOPECOMPLETEARGS void *slopeHandle
int slopeComplete(SLOPECOMPLETEARGS);
