/**
   The code here is used to create a shared object library, which can
   then be swapped around depending on which cameras you have in use,
   ie you simple rename the camera file you want to camera.so (or
   better, change the soft link), and restart the coremain.

   The library is written for a specific camera configuration - ie in
   multiple camera situations, the library is written to handle
   multiple cameras, not a single camera many times.

   This interface is for the JAI pulnix camera.

   It is possible that this won't work (if the JAI library isn't
   relocatable).  In which case, should write some stand alone code to
   read the camera and put into a shm buffer, and then a .so which
   coremain can use that gets data from the .so.  This would also get
   around the unique buffer problem.

   Need
export LD_PRELOAD=/usr/lib/libgomp.so.1 (EDITOR) LD_LIBRARY_PATH=./
   for this to work.

*/
#include <Jai_Factory.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include "rtccamera.h"
#include <time.h>
//#include <unistd.h>
#include <pthread.h>

#define HDRSIZE 0		//8 //the size of a WPU header - 4 bytes for frame no, 4 bytes for something else.

// A ring-buffer of size 8 to camera frames:
#define NBUF 8
#define BUFMASK 0x7

/**
   The struct to hold info.  If using multi cameras would need to
   recode, so have multiple instances of this struct.
*/
typedef struct {
  int err;
  int ncam;			// no of cameras (must be 1 for Pulnix);
  int npxls;			// number of pixels in the frame
  int transferRequired;
  volatile int dataReady;       // ring buffer contains unread camera data;
  volatile int first;           // true if this is first frame received
  volatile int tail;		// the frame being, or to be, xferred to the RTC
  volatile int head;		// the latest whole frame arrived
  pthread_mutex_t m;            // serialises access to this struct.
  pthread_cond_t cond;	        // sync between main RTC
  unsigned char *ringBuf[NBUF];	// buffers for receiving the frames:
                                // each bytesPerPxl * npxls.  
  int open;			// set by RTC if the camera is open
  int framing;			// set by RTC if the camera is framing.
  volatile int newframeAll;	// set by RTC when a new frame is starting to be 
                                // requested.
  volatile int *newframe;
  short *imgdata;
  int *pxlsTransferred;	        //number of pixels copied into the RTC memory.
  pthread_t threadid;
  unsigned int *userFrameNo;	// pointer to the RTC frame number... 
                                // to be updated for new frame.
  int *setFrameNo;		// tells thread to set the userFrameNo.
  int threadPriority;
  int threadAffinity;
  int bytesPerPxl;
  THRD_HANDLE m_hThread;
  FACTORY_HANDLE m_hFactory;	// Factory Handle
  int8_t m_sCameraId[J_CAMERA_ID_SIZE];	// Camera ID
  CAM_HANDLE m_hCam;		// Camera Handle
  NODE_HANDLE hNode;
  struct timespec timeout;	//sec and ns


  
} CamStruct;

typedef struct {
   CamStruct *camstr;
   int camNo;
} ThreadStruct;

/**
   Find out if this SO library supports your camera.

*/
//extern "C" camOpenFn(char *name,int n,int *args,void **camHandle,int npxls,short *pxlbuf,int ncam,int *pxlx,int* pxly,int* frameno);

int
camQuery(char *name)
{
  int rtval=0;
#ifdef OLD
   //Note, the strings aren't necessarily null terminated...
  rtval=(strcmp(name, "pulnix") != 0);
#endif
   return rtval;
}

unsigned char *buffer;    // Buffer to receive frame from camera
pthread_mutex_t jaimutex; // Protect access to camera ring buffer
pthread_cond_t jaicond;   // Signal new camera frame arrived
int jaiDataReady; //flag - set if a new frame has been copied to buffer.
//----------------------------------------------
// StreamCBFunc
//----------------------------------------------
// Pulnix API callback function
// This gets called asynchronously by the JAI API 
// when a frame arrived interrupt occurs.
// Note - since buffer is a global, we can't tell 
// for which camera the interrupt occured, so currently, 
// is only safe with 1 camera.

static void 
StreamCBFunc(J_tIMAGE_INFO * pAqImageInfo)
{

   pthread_mutex_lock(&jaimutex);
   memcpy(buffer, pAqImageInfo->pImageBuffer, pAqImageInfo->iImageSize);
   jaiDataReady=1;
   pthread_cond_signal(&jaicond);
   pthread_mutex_unlock(&jaimutex);

}

#define NODE_NAME_WIDTH     "Width"
#define NODE_NAME_HEIGHT    "Height"
#define NODE_NAME_GAIN      "GainRaw"
#define NODE_NAME_ACQSTART  "AcquisitionStart"
#define NODE_NAME_ACQSTOP   "AcquisitionStop"
#define NODE_NAME_PIXFORMAT "PixelFormat"
#define REG_ADDR_SCDA0      (0x0d18)
#define MULTICASTADDR 0


//----------------------------------------------
// Camara setting
//----------------------------------------------
int
Camera_JAI(CamStruct *camstr, unsigned int cam, int imgSizeX, int imgSizeY)
{
   J_STATUS_TYPE retval;
   bool8_t bHasChange;
   uint32_t iNumDev;
   uint32_t iSize;
   int iPos;
   int64_t int64Val;
   SIZE ViewSize;
   void *vfptr = reinterpret_cast < void *>(StreamCBFunc);
   J_IMG_CALLBACK_FUNCTION *cbfptr =
       reinterpret_cast < J_IMG_CALLBACK_FUNCTION * >(&vfptr);

   retval = J_Factory_Open((int8_t *)"", &camstr->m_hFactory);
   if (retval != J_ST_SUCCESS) {
     printf("J_Factory_Open failed: %d\n", retval);
     return 1;
   }
   retval = J_Factory_UpdateCameraList(camstr->m_hFactory, &bHasChange);
   if (retval != J_ST_SUCCESS) {
     printf("J_Factory_UpdateCameraList failed: %d\n", retval);
     return 1;
   }
   retval = J_Factory_GetNumOfCameras(camstr->m_hFactory, &iNumDev);
   if (retval != J_ST_SUCCESS) {
     printf("J_Factory_getNumOfCameras failed: %d\n", retval);
     return 1;
   }
   // Get camera ID (of cam^th camera...)
   if (cam > iNumDev) {
     printf("Error - trying to set camera %d out of %d\n", 
	     cam, iNumDev);
      return 1;
   }
   iSize = (uint32_t) sizeof(camstr->m_sCameraId);
   retval = J_Factory_GetCameraIDByIndex(camstr->m_hFactory,
					 cam, camstr->m_sCameraId, &iSize);
   if (retval != J_ST_SUCCESS) {
     printf("J_Factory_GetCameraIDByIndex failed: %d\n", retval);
     return 1;
   }
   // Open camera
   retval = J_Camera_Open(camstr->m_hFactory,
			  camstr->m_sCameraId, &camstr->m_hCam);
   if (retval != J_ST_SUCCESS) {
     printf("J_Camera_Open failed: %d\n", retval);
     return 1;
   }

   // Set pixel format to 10-bit mono
   retval = J_Camera_GetNodeByName(camstr->m_hCam, 
 				   (int8_t *)"EnumEntry_PixelFormat_Mono10",  
 				   &camstr->hNode);
   if (retval != J_ST_SUCCESS) {
     printf("Failed to get EnumEntry_PixelFormat_Mono10 node: %d\n",
	    retval);
     return 1;
   }

   retval = J_Node_GetEnumEntryValue(camstr->hNode, &int64Val);
   if (retval != J_ST_SUCCESS) {
     printf("Failed to get EnumEntry_PixelFormat_Mono10 node value: %d\n", 
	     retval);
     return 1;
   }
   retval = J_Camera_GetNodeByName(camstr->m_hCam, 
				   (int8_t *)NODE_NAME_PIXFORMAT,
				   &camstr->hNode);
   if (retval != J_ST_SUCCESS) {
     printf("Failed to get ImageFormat node: %d\n", retval);
     return 1;
   }
   retval = J_Node_SetValueInt64(camstr->hNode, 0, int64Val);
   if (retval != J_ST_SUCCESS) {
     printf("Failed to set Image Format: %i\n", retval);
   }


   // Get & Set Width from the camera
   retval = J_Camera_GetNodeByName(camstr->m_hCam,
				   (int8_t *)NODE_NAME_WIDTH, &camstr->hNode);
   if (retval != J_ST_SUCCESS) {
     printf("J_Camera_GetNodeByName failed: %d\n", retval);
     return 1;
   }
   iPos = imgSizeX;
   int64Val = (int64_t) iPos;
   retval = J_Node_SetValueInt64(camstr->hNode, 0, int64Val);	// Set Value
   if (retval != J_ST_SUCCESS) {
     printf("J_Node_SetValueInt64 failed: %d\n", retval);
      return 1;
   }
   retval = J_Node_GetValueInt64(camstr->hNode, 0, &int64Val);
   if (retval != J_ST_SUCCESS) {
     printf("J_Node_GetValueInt64 failed: %d\n", retval);
      return 1;
   }
   ViewSize.cx = (LONG) int64Val;	// Set window size cx

   // Get & Set Height from the camera
   retval = J_Camera_GetNodeByName(camstr->m_hCam,
				   (int8_t *)NODE_NAME_HEIGHT, &camstr->hNode);
   if (retval != J_ST_SUCCESS) {
     printf("J_Camera_GetNodeByName failed for height: %d\n", retval);
      return 1;
   }
   iPos = imgSizeY;
   int64Val = (int64_t) iPos;
   retval = J_Node_SetValueInt64(camstr->hNode, 0, int64Val);	// Set Value
   if (retval != J_ST_SUCCESS) {
     printf("J_Node_SetValueInt64 failed: %d\n", retval);
      return 1;
   }
   retval = J_Node_GetValueInt64(camstr->hNode, 0, &int64Val);
   if (retval != J_ST_SUCCESS) {
     printf("J_Node_GetValueInt64 failed: %d\n", retval);
      return 1;
   }
   ViewSize.cy = (LONG) int64Val;	// Set window size cy
   printf("Camera is %d x %d\n",(int)ViewSize.cy,(int)ViewSize.cx);
   //-Get & Set  Gain 
   retval = J_Camera_GetNodeByName(camstr->m_hCam,
				   (int8_t *)NODE_NAME_GAIN, &camstr->hNode);
   if (retval != J_ST_SUCCESS) {
     printf("J_Camera_GetNodeByName for gain failed: %d\n", retval);
      return 1;
   }
   iPos = 350;
   int64Val = (int64_t) iPos;
   // Set Value
   retval = J_Node_SetValueInt64(camstr->hNode, 0, int64Val);
   if (retval != J_ST_SUCCESS) {
     printf("J_Node_SetValueInt64 failed: %d\n", retval);
      return 1;
   }
   // Get Gain
   retval = J_Node_GetValueInt64(camstr->hNode, 0, &int64Val);
   if (retval != J_ST_SUCCESS) {
     printf("J_Node_GetValueInt64 failed: %d\n", retval);
      return 1;
   }
   // Open stream
   retval = J_Image_OpenStream(camstr->m_hCam, 0, NULL, *cbfptr, &camstr->m_hThread, ViewSize.cx * ViewSize.cy * camstr->bytesPerPxl, MULTICASTADDR);	//todo - should 0 be cam?
   if (retval != J_ST_SUCCESS) {
     printf("J_Image_OpenStream failed: %d\n", retval);
      return 1;
   }
   //    SetThreadPriority(hThread,THREAD_PRIORITY_HIGHEST);    //top priority: THREAD_PRIORITY_TIME_CRITICAL

   // Start Acquision
   retval = J_Camera_GetNodeByName(camstr->m_hCam,
				   (int8_t *)NODE_NAME_ACQSTART, 
				   &camstr->hNode);
   if (retval != J_ST_SUCCESS) {
     printf("J_Camera_GetNodeByName failed: %d\n", retval);
      return 1;
   }
   retval = J_Node_ExecuteCommand(camstr->hNode);
   if (retval != J_ST_SUCCESS) {
     printf("J_Node_ExecuteCommand failed: %d\n", retval);
      return 1;
   }
   return 0;
}

//------------------------------------------------
// Close camera
//------------------------------------------------
int
Close_JAI(CamStruct *camstr)
{
   J_STATUS_TYPE retval;

   if (camstr == (CamStruct *)NULL)
     return 0;

   if (camstr->m_hCam) {
      retval = J_Camera_GetNodeByName(camstr->m_hCam,
				      (int8_t *)NODE_NAME_ACQSTOP, 
				      &camstr->hNode);
      if (retval != J_ST_SUCCESS)
	 printf("J_Camera_GetNodeByName failed\n");
      retval = J_Node_ExecuteCommand(camstr->hNode);
      if (retval != J_ST_SUCCESS)
	 printf("J_Node_ExecuteCommand failed\n");
   }

   if (camstr->m_hCam) {
      // Close camera
      retval = J_Camera_Close(camstr->m_hCam);
      if (retval != J_ST_SUCCESS)
	 printf("J_Camera_Close failed\n");
      camstr->m_hCam = NULL;
   }
   // Close stream
   retval = J_Image_CloseStream(camstr->m_hThread);
   if (retval != J_ST_SUCCESS)
      printf("J_Image_CloseStream failed\n");
   camstr->m_hThread = NULL;

   if (camstr->m_hFactory) {
      // Close factory
      retval = J_Factory_Close(camstr->m_hFactory);
      if (retval != J_ST_SUCCESS)
	 printf("J_Factory_Close failed\n");
      camstr->m_hFactory = NULL;
   }
   return 0;
}


#define safefree(ptr) if(ptr!=NULL)free(ptr);
/*void
safefree(void *ptr)
{
   if (ptr != NULL)
      free(ptr);
      }*/

void
dofree(CamStruct * camstr)
{
  int i;

   printf("dofree called\n");
   if (camstr != NULL) {

      pthread_cond_destroy(&camstr->cond);
      pthread_mutex_destroy(&camstr->m);
      pthread_cond_destroy(&jaicond);
      pthread_mutex_destroy(&jaimutex);
      safefree((void *)camstr->newframe);
      safefree(camstr->pxlsTransferred);
      safefree(camstr->setFrameNo);
      for(i=0; i<NBUF; i++){
	safefree(camstr->ringBuf[i]);
      }
      free(camstr);
   }
}


int
setThreadAffinityAndPriority(int threadAffinity, int threadPriority)
{
   int i;
   cpu_set_t mask;
   int ncpu;
   struct sched_param param;

   printf("Getting CPUs\n");
   ncpu = sysconf(_SC_NPROCESSORS_ONLN);
   printf("Got %d CPUs\n", ncpu);
   CPU_ZERO(&mask);
   printf("Setting %d CPUs\n", ncpu);
   for (i = 0; i < ncpu; i++) {
      if (((threadAffinity) >> i) & 1) {
	 CPU_SET(i, &mask);
      }
   }
   printf("Thread affinity %d\n", threadAffinity & 0xffff);
   if (sched_setaffinity(0, sizeof(cpu_set_t), &mask))
      perror("Error in sched_setaffinity");
   printf("Setting setparam\n");
   param.sched_priority = threadPriority;
   if (sched_setparam(0, &param)) {
      printf
	  ("Error in sched_setparam: %s - probably need to run as root if this is important\n",
	   strerror(errno));
   }
   return 0;
}


/**
   This thread is responsible to update the camera struct and 
   set the receive buffer address after a new frame is received. 
   It is started when the camera is opened.
*/
void *
worker(void *thrstrv)
{
   CamStruct *camstr = (CamStruct *)thrstrv;
   //int req, off, err, i;
   int err;
   struct timespec timeout;	//sec and ns
   printf("Calling setThreadAffinityAndPriority\n");
   setThreadAffinityAndPriority(camstr->threadAffinity,
				camstr->threadPriority);

   camstr->first = 1;
   camstr->tail = 0;
   camstr->head = 0;
   buffer = camstr->ringBuf[camstr->head];

   pthread_mutex_lock(&camstr->m);
   while (camstr->open) {
     while (camstr->framing) {	//camera is framing...
       pthread_mutex_unlock(&camstr->m);
       //wait for buffer to be filled.
       if((err=pthread_mutex_lock(&jaimutex))!=0){
	 printf("Error locking jaimutex: %s\n",strerror(err));
       }
       if(!jaiDataReady) {
	 //printf("worker waiting timeout: %d %d\n",(int)camstr->timeout.tv_sec,(int)camstr->timeout.tv_nsec);
	 clock_gettime(CLOCK_REALTIME, &timeout);
	 timeout.tv_sec+=camstr->timeout.tv_sec;
	 timeout.tv_nsec+=camstr->timeout.tv_nsec;
	 if((err = pthread_cond_timedwait(&jaicond, &jaimutex, &timeout))!=0){
	   if(err == ETIMEDOUT){
	     printf("Timeout waiting for pixels\n");
	   }else{
	     printf("Error waiting for pixels: %d %s\n",err,strerror(err));
	     //perror("Error waiting for pixels");
	   }
	 }
	 //printf("Worker waited\n");
       } //else {
       pthread_mutex_unlock(&jaimutex);
       if(err)
	 sleep(1);
	 //}
       jaiDataReady = 0;
       //printf("worker getting lock\n");
       pthread_mutex_lock(&camstr->m);
       //printf("worker got lock\n");
       camstr->err = err;
       if(err == 0){
	 if ((!camstr->first) && (camstr->head == camstr->tail)) {	
	   // not the first frame...
	   // ring overflow; drop this frame frame
	   printf("Ring buffer overflow\n");
	 } else {
	   
	   // advance the ring head pointer
	   camstr->head++;
	   camstr->head &= BUFMASK;
	   //printf("worker getting jaimutex\n");
	   pthread_mutex_lock(&jaimutex);           // Yes!
	   buffer = camstr->ringBuf[camstr->head];
	   pthread_mutex_unlock(&jaimutex);
	   //printf("worker released jaimutex\n");
	   camstr->dataReady = 1;
	   
	 }
       }
   
       // Unblock the worker thread, if it's waiting
       pthread_cond_broadcast(&camstr->cond);

     }
     //no longer framing...
     if (camstr->open) {
       pthread_cond_wait(&camstr->cond, &camstr->m);
       //pthread_mutex_lock(&camstr->m);   // pthread_cond_wait releases mutex BUT IT REACQUIRES IT BEFORE RETURNING
     }
   }
   pthread_mutex_unlock(&camstr->m);
   return 0;
}

/**
   Open a camera of type name.  Args are passed in a int32 array of
   size n, which can be cast if necessary.  Any state data is returned
   in camHandle, which should be NULL if an error arises.  pxlbuf is
   the array that should hold the data. The library is free to use the
   user provided version, or use its own version as necessary (ie a
   pointer to physical memory or whatever).  It is of size
   npxls*sizeof(short).  ncam is number of cameras, which is the
   length of arrays pxlx and pxly, which contain the dimensions for
   each camera.  Currently, ncam must equal 1.  Name is used if a
   library can support more than one camera.  frameno is a pointer to
   an array with a value for each camera in which the frame number
   should be placed.

   This is for getting data from the Pulnix camera
   args here currently is unused.
*/


#define TEST(a) if((a)==NULL){perror("calloc error");dofree(camstr);*camHandle=NULL;return 1;}

#ifdef __cplusplus
extern "C" 
#endif
int camOpen(char *name, int n, int *args, void **camHandle,
	int npxls, short *pxlbuf, int ncam, int *pxlx, int *pxly, int *frameno)
{
   CamStruct *camstr;
   unsigned int i;
   int err;
   printf("Initialising camera %s\n", name);
   if (camQuery(name)) {
      printf("Wrong camera type %s\n", name);
      return 1;
   }

   if (ncam != 1) {
      printf
	("ERROR: the JAI PULNIX interface doesn't support >1 camera\n");
      return 1;
   }

   if ((*camHandle = malloc(sizeof(CamStruct))) == NULL) {
      perror("Couldn't malloc camera handle");
      return 1;
   }

   printf("Malloced camstr\n");
   memset(*camHandle, 0, sizeof(CamStruct));
   camstr = (CamStruct *) *camHandle;
   camstr->imgdata = pxlbuf;
   camstr->userFrameNo = (unsigned int *)frameno;
   memset(frameno, 0, sizeof(int) * ncam);
   camstr->ncam = ncam;
   camstr->npxls = npxls;	//*pxlx * *pxly;

   TEST(camstr->setFrameNo = (int *)calloc(ncam, sizeof(int)));
   printf("malloced things\n");

   if (n == 4 * ncam) {

     camstr->bytesPerPxl = args[0];
     camstr->timeout.tv_sec = args[1] / 1000;	//timeout in ms.
     camstr->timeout.tv_nsec = (args[1] % 1000) * 1000000;

     camstr->threadAffinity = args[2];	//thread affinity
     camstr->threadPriority = args[3];	//thread priority


   } else {
      printf ("wrong number of cmd args, should be 4: bytesperpxl,"
	      " timeout, thread affinity, thread priority\n"); 
      dofree(camstr);
      *camHandle = NULL;
      return 1;
   }

/*    printf("got args\n"); */
/*    for (i = 0; i < ncam; i++) { */
/*       printf("%d %d %d\n", */
/* 	     camstr->blocksize[i], */
/* 	     (int)camstr->timeout[i].tv_sec, (int)camstr->timeout[i].tv_nsec); */
/*    } */


   if (pthread_cond_init(&camstr->cond, NULL) != 0) {
     printf("Error initialising condition variable %d\n", i);
     dofree(camstr);
     *camHandle = NULL;
     return 1;
   }


/*    if (pthread_cond_init(&camstr->thrcond, NULL) != 0) { */
/*       perror("Error initialising thread condition variable"); */
/*       dofree(camstr); */
/*       *camHandle = NULL; */
/*       return 1; */
/*    } */

   if (pthread_cond_init(&jaicond, NULL) != 0) {
      perror("Error initialising thread condition variable");
      dofree(camstr);
      *camHandle = NULL;
      return 1;
   }
   //maybe think about having one per camera???
   if (pthread_mutex_init(&camstr->m, NULL) != 0) {
      perror("Error initialising mutex");
      dofree(camstr);
      *camHandle = NULL;
      return 1;
   }

   if (pthread_mutex_init(&jaimutex, NULL) != 0) {
      perror("Error initialising mutex variable");
      dofree(camstr);
      *camHandle = NULL;
      return 1;
   }
   printf("done mutex\n");


   printf("doingf ringBuf\n");
   for (i = 0; i < NBUF; i++) {
     camstr->ringBuf[i] = (unsigned char *)malloc(camstr->bytesPerPxl 
						             * camstr->npxls);
     if (camstr->ringBuf[i] == NULL) {
	 perror("Couldn't allocate ring buffer");
	 dofree(camstr);
	 *camHandle = NULL;
	 return 1;
      }

     printf("Clearing ring buffer[ %d ]...\n",i);
      memset(camstr->ringBuf[i], 0,
	     (camstr->bytesPerPxl * camstr->npxls));
   }

   printf("done dmabuf\n");
   buffer = camstr->ringBuf[0];
   jaiDataReady=0;
   //Now do the JAI stuff...
   err = 0;
   for (i = 0; err == 0 && i < (unsigned int)ncam; i++) {
     err = Camera_JAI(camstr, i, pxlx[i], pxly[i]);
     if (err) {
       printf("Failed to open pulnix camera %d\n", i);
     }else
       printf("Opened pulnix camera %d\n",i);
   }
   if (err) {
     dofree(camstr);
     *camHandle = NULL;
   } else {
     camstr->open = 1;
     err = pthread_create(&camstr->threadid, NULL, worker,
			   (void *)camstr);
     if (err) {
       perror("pthread_create() failed");
     } else {
       printf("created worker thread\n");
     }
   } 

   return err;
}


/**
   Close a camera of type name.  State data is in camHandle, which
   should be freed and set to NULL before returning.
*/
#ifdef __cplusplus
extern "C" 
#endif
int
camClose(void **camHandle)
{
   CamStruct *camstr;
   //int i;

   printf("Closing camera\n");
   if (*camHandle == NULL)
      return 1;
   camstr = (CamStruct *) *camHandle;
   pthread_mutex_lock(&camstr->m);
   camstr->open = 0;
   camstr->framing = 0;
   printf("signalling\n");
   pthread_cond_signal(&camstr->cond);

   pthread_mutex_unlock(&camstr->m);
   printf("joining\n");
   pthread_join(camstr->threadid, NULL);  //wait for worker thread to complete
   printf("joined - locking\n");
   pthread_mutex_lock(&camstr->m);
   printf("close_JAI\n");
   Close_JAI(camstr);
   pthread_mutex_unlock(&camstr->m);

   dofree(camstr);
   *camHandle = NULL;
   printf("Camera closed\n");
   return 0;
}

/**
   Start the camera framing, using the args and camera handle data.
   TODO: turn on th ePulnix here rather than in camOpen.
*/
#ifdef __cplusplus
extern "C" 
#endif
int
camStartFraming(int n, int *args, void *camHandle)
{
   CamStruct *camstr;
   //int i;

   if (camHandle == NULL) {
      printf("called camStartFraming with camHandle==NULL\n");
      return 1;
   }
   camstr = (CamStruct *) camHandle;
   pthread_mutex_lock(&camstr->m);
   camstr->framing = 1;

   pthread_cond_broadcast(&camstr->cond);
   pthread_mutex_unlock(&camstr->m);
   printf("Framing camera\n");
   return 0;
}

/**
   Stop the camera framing
   TODO: actually stop the Pulnix from delivering frames
*/
#ifdef __cplusplus
extern "C" 
#endif
int
camStopFraming(void *camHandle)
{
   CamStruct *camstr;

   if (camHandle == NULL) {
      printf("called camStopFraming with camHandle==NULL\n");
      return 1;
   }
   camstr = (CamStruct *) camHandle;
   pthread_mutex_lock(&camstr->m);
   camstr->framing = 0;
   pthread_mutex_unlock(&camstr->m);
   printf("Stopping framing\n");
   return 0;
}


/**
   Called when we're starting processing the next frame.  This doesn't
   actually wait for any pixels. It simply advances the read pointer
   to the next ring buffer element, unless this is the first frame
   to be read, when we're already at the correct place in the ring.
*/
#ifdef __cplusplus
extern "C" 
#endif
int
camNewFrame(void *camHandle)
{
   //printf("camNewFrame\n");
   CamStruct *camstr;
   //int i;

   camstr = (CamStruct *) camHandle;
   if (camHandle == NULL || camstr->framing == 0) {
      //printf("called camNewFrame with camHandle==NULL\n");
      return 1;
   }
   pthread_mutex_lock(&camstr->m);
   //printf("New frame\n");
   camstr->transferRequired=1;
   if (!camstr->first) {
     camstr->tail++;
     camstr->tail &= BUFMASK;
     if (camstr->tail == camstr->head) {
       // ring underfow - set dataReady to 0.
       printf("Ring buffer underflow\n");
       camstr->dataReady = 0;
     } 
   }
   *(camstr->userFrameNo)++;
   pthread_mutex_unlock(&camstr->m);
   return 0;
}

/**
   Wait for the first n pixels of the current frame to arrive.
   WARNING - probably not thread safe.  But, in this case, since this
   library is for single camera mode only, and this is only ever
   called by 1 thread per camera at once, then we're okay...
*/
#ifdef __cplusplus
extern "C" 
#endif
int camWaitPixels(int n,int cam,void *camHandle){
  //printf("camWaitPixels %d, camera %d\n",n,cam);
  //For andor, we actually have to wait for the whole frame...
  CamStruct *camstr=(CamStruct*)camHandle;
  //char *cd;
  int i,err=0;
  //char tmp;
  //static struct timeval t1;
  //struct timeval t2;
  //struct timeval t3;
   if (cam != 0) {
     printf("Error: camWaitPixels called for camera %d:"
	    "only one camera (0) is supported\n", cam);
     return 1;
   }
   if (camHandle == NULL) {
      printf("called camWaitPixels with camHandle==NULL\n");
      return 1;
   }
   if (camstr->framing == 0) {
     printf("camera is not framing\n");
     return 1;
   }
   //printf("camWaitPixels %d %d %d\n",n,cam,camstr->transferRequired);
  if(camstr->transferRequired){
    pthread_mutex_lock(&camstr->m);
    err=camstr->err;
    if(err==0){
      while(!camstr->dataReady) {
	pthread_cond_wait(&camstr->cond, &camstr->m); // ...releases mutex AND REACQUIRES IT BEFORE RETURNING.
	//pthread_mutex_lock(&camstr->m);               // ...so re-acquire it
      }

      /* Camera returns pixels as unsigned char */
      for (i = 0; i < camstr->npxls; i++)
	camstr->imgdata[i] = (short)camstr->ringBuf[camstr->tail][i];
      printf("Pixels: %d %d %d %d\n",camstr->imgdata[10000],camstr->imgdata[10001],camstr->imgdata[10002],camstr->imgdata[10003]);
    }
    pthread_mutex_unlock(&camstr->m);
    camstr->transferRequired=0;
  }
  return err;
}

