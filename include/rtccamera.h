/*
darc, the Durham Adaptive optics Real-time Controller.
Copyright (C) 2010 Alastair Basden.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "circ.h"
#include "arrayStruct.h"
#include "buffer.h"
/**
   The code here is used to create a shared object library, which can then be swapped around depending on which cameras you have in use, ie you simple rename the camera file you want to camera.so (or better, change the soft link), and restart the coremain.

The library is written for a specific camera configuration - ie in multiple camera situations, the library is written to handle multiple cameras, not a single camera many times.
*/

/**
   Open a camera of type name.  Args are passed in a int32 array of size n, which can be cast if necessary.  Any state data is returned in camHandle, which should be NULL if an error arises.
   pxlbuf is the array that should hold the data. The library is free to use the user provided version, or use its own version as necessary (ie a pointer to physical memory or whatever).  It is of size npxls*sizeof(short).
   ncam is number of cameras, which is the length of arrays pxlx and pxly, which contain the dimensions for each camera.
   Name is used if a library can support more than one camera.

*/
#define CAMOPENARGS char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **handle,int nthreads,unsigned int frameno,unsigned int **camframeno,int *camframenoSize,int npxls,int ncam,int *pxlx,int* pxly
#ifdef __cplusplus
extern "C" 
#endif
int camOpen(CAMOPENARGS);

/**
   Close a camera of type name.  Args are passed in the int32 array of size n, and state data is in camHandle, which should be freed and set to NULL before returning.
*/
#define CAMCLOSEARGS void **camHandle
#ifdef __cplusplus
extern "C" 
#endif
int camClose(CAMCLOSEARGS);
/**
   Start the camera framing, using the args and camera handle data.
*/
/*
#ifdef __cplusplus
extern "C" 
#endif
int camStartFraming(int n,int *args,void *camHandle);
#ifdef __cplusplus
extern "C" 
#endif
int camStopFraming(void *camHandle);
*/

/**
   Called when we're starting processing the next frame.  
*/
#define CAMNEWFRAMESYNCARGS void *camHandle,unsigned int thisiter,double timestamp
#ifdef __cplusplus
extern "C" 
#endif
int camNewFrameSync(CAMNEWFRAMESYNCARGS);

#define CAMNEWFRAMEARGS void *camHandle,unsigned int thisiter,double timestamp
#ifdef __cplusplus
extern "C" 
#endif
int camNewFrame(CAMNEWFRAMEARGS);

#define CAMSTARTFRAMEARGS void *camHandle,int cam,int threadno
#ifdef __cplusplus
extern "C" 
#endif
int camStartFrame(CAMSTARTFRAMEARGS);

/**
   Wait for the next n pixels of the current frame to arrive.
*/
#define CAMWAITPIXELSARGS int n,int cam,void *camHandle
#ifdef __cplusplus
extern "C" 
#endif
int camWaitPixels(CAMWAITPIXELSARGS);
/**
   New parameters...
*/
#define CAMNEWPARAMARGS void *camHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr
#ifdef __cplusplus
extern "C" 
#endif
int camNewParam(CAMNEWPARAMARGS);


#define CAMENDFRAMEARGS void *camHandle,int cam,int threadno,int err
#ifdef __cplusplus
extern "C" 
#endif
int camEndFrame(CAMENDFRAMEARGS);//subap thread (once per thread)

#define CAMFRAMEFINISHEDSYNCARGS void *camHandle,int err,int forcewrite
#ifdef __cplusplus
extern "C" 
#endif
int camFrameFinishedSync(CAMFRAMEFINISHEDSYNCARGS);//subap thread (once)

#define CAMFRAMEFINISHEDARGS void *camHandle,int err
#ifdef __cplusplus
extern "C" 
#endif
int camFrameFinished(CAMFRAMEFINISHEDARGS);//non-subap thread (once)

#define CAMOPENLOOPARGS void *camHandle
#ifdef __cplusplus
extern "C" 
#endif
int camOpenLoop(CAMOPENLOOPARGS);

#define CAMCOMPLETEARGS void *camHandle
#ifdef __cplusplus
extern "C" 
#endif
int camComplete(CAMCOMPLETEARGS);

#define CAMCOMPUTEPIXELSARGS int *subapLocation,int nsubapsProcessing,int cursubindx,int cam,void *camHandle
#ifdef __cplusplus
extern "C"
#endif
int camComputePixels(CAMCOMPUTEPIXELSARGS);//subap thread (lots of times) - can compute number of pixels required.
