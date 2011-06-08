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
#include <sys/time.h>
#include <pthread.h>

#define NUM_OF_BUFFER 64 //was 16 // Buffers used internally by libgenicam



#define HDRSIZE 0		//8 //the size of a WPU header - 4 bytes for frame no, 4 bytes for something else.

// A ring-buffer of size 8 to camera frames:
#define NBUF 8
#define BUFMASK 0x7


//typedef struct {
//  CamStruct *camstr;
//  int camNo;
//} ThreadStruct;

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
  volatile int tail;		// the frame being, or to be, xferred to the RTC
  volatile int head;		// the latest whole frame arrived
  pthread_mutex_t m;            // serialises access to this struct.
  pthread_cond_t cond;	        // sync between main RTC
  pthread_cond_t cond2;	        // sync between threads?

  unsigned char *ringBuf[NBUF];	// buffers for receiving the frames:
                                // each bytesPerPxl * npxls.  
  int bufframeno[NBUF];//stores a frame count for each buffer.
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
  unsigned int *threadAffinity;
  int threadAffinElSize;
  int bytesPerPxl;
  FACTORY_HANDLE m_hFactory;	// Factory Handle
  int8_t m_sCameraId[J_CAMERA_ID_SIZE];	// Camera ID
  CAM_HANDLE m_hCam;		// Camera Handle
  NODE_HANDLE hNode;
  //CStreamThread *pStreamObj;
  struct timespec timeout;	//sec and ns
  int offsetX;//used for region of interest
  int offsetY;
  int exptime;
  int scanmode;//desired scan mode - or 0 for use smallest.
  int timerDelayRaw;
  int timerDurationRaw;
  int timerGranularityFactor;
  int testmode;
  int reterr;//set if error when transferring pixels... (ie if transferRequired set).
  unsigned int maxWaitingFrames;//number of frames that camera an queue up before we start skipping them...
  int internalTrigger;
} CamStruct;


//=====================================================================
// CStreamThread
//=====================================================================
class CStreamThread {
 public:
   CStreamThread(void);
   ~CStreamThread(void);
   bool CreateStreamThread(CAM_HANDLE hDev, uint32_t iChannel,
			   uint32_t iBufferSize, DWORD iMcIP);//,int priority);
   bool TerminateStreamThread(void);
   uint32_t PrepareBuffer(void);
   bool UnPrepareBuffer(void);
   void CloseThreadHandle(void);
   void StreamProcess(void *context);
  void RegisterCallback(void callback(J_tIMAGE_INFO * pAqImageInfo,CamStruct *camstr));

   uint32_t m_iStreamChannel;
   uint32_t m_iValidBuffers;
   bool m_bEnableThread;
   uint32_t m_iBufferSize;
   bool m_bAwb;
   pthread_t m_hStreamThread;
   CAM_HANDLE m_hDev;
   STREAM_HANDLE m_hDS;
   HANDLE m_hEventNewImage;
   BUF_HANDLE m_pAquBufferID[NUM_OF_BUFFER];
   uint8_t *m_pAquBuffer[NUM_OF_BUFFER];
   bool m_bStreamStarted;	// Flag indicating that the J_Stream_StartAcquisition() has been called
  void (*callbackFn) (J_tIMAGE_INFO * pAqImageInfo,CamStruct *camstr);
  CamStruct *camstr;
};

typedef struct{
  CamStruct *camstr;
  CStreamThread *pStreamObj;
}CamStreamStruct;

//=====================================================================
// StreamProcessDispatch
//=====================================================================
void *
StreamProcessDispatch(void *context)
{
   CStreamThread::CStreamThread * pC =
       dynamic_cast <CStreamThread::CStreamThread *> ((CStreamThread::CStreamThread *) context);

   pC->StreamProcess(context);
   return NULL;
}

//=====================================================================
// CStreamThread constructor
//=====================================================================
CStreamThread::CStreamThread(void)
  :  m_iStreamChannel(0)
  , m_iValidBuffers(0)
  , m_bEnableThread(false)
  , m_iBufferSize(J_XGA_WIDTH * J_XGA_HEIGHT * J_MAX_BPP)
  , m_bAwb(false)
  , m_hStreamThread(0)
  , m_hDev(NULL)
  , m_hDS(NULL)
  , m_hEventNewImage(NULL)
  , m_bStreamStarted(false)
  , callbackFn(NULL)
{

}

//=====================================================================
// CStreamThread destructor
//=====================================================================
CStreamThread::~CStreamThread(void)
{
}


//=====================================================================
// Create Stream Thread
//=====================================================================
bool CStreamThread::CreateStreamThread(CAM_HANDLE hDev,
				       uint32_t iChannel,
				       uint32_t iBufferSize, DWORD iMcIP)//,int priority)
{
   if (m_hStreamThread)
      return false;

   m_iBufferSize = iBufferSize;
   m_hDev = hDev;

   // Open the stream channel(GVSP)
   if (m_hDS == NULL) {
      if ((J_ST_SUCCESS != J_Camera_CreateDataStreamMc(m_hDev, iChannel,
						       &m_hDS, iMcIP))
	  || (m_hDS == NULL)) {
	 fprintf(stderr, "*** Error : J_Camera_CreateDataStreamMc\n");
	 return false;
      }
   }
   // Prepare the frame buffer.
   if (0 == PrepareBuffer()) {
      fprintf(stderr, "*** Error : PrepareBuffer\n");
      J_DataStream_Close(m_hDS);
      return false;
   }

   m_bEnableThread = true;

   //pthread_attr_t ThreadAttr;
   //pthread_attr_init(&ThreadAttr);
   //pthread_attr_setschedpolicy(&ThreadAttr, SCHED_RR);
   //struct sched_param schedParam;
   //schedParam.sched_priority=priority;
   //pthread_attr_setschedparam(&ThreadAttr,&schedParam);

   pthread_create(&m_hStreamThread, NULL,//&ThreadAttr,
		  StreamProcessDispatch, (void *)this);
   //pthread_attr_destroy(&ThreadAttr);
   return true;
}

//=====================================================================
// Terminate Stream Thread
//=====================================================================
bool 
CStreamThread::TerminateStreamThread(void)
{
   // Stop the Acquisiiton engine.
   if (m_hDS == NULL)
      return false;
   printf("J_DataStream_StopAcquisition()\n");
   J_DataStream_StopAcquisition(m_hDS, ACQ_STOP_FLAG_KILL);

   // Reset the thread execution flag.
   m_bEnableThread = false;

   // Mark stream acquisition as stopped
   m_bStreamStarted = false;
   printf("J_Event_ExitCondition()\n");
   // Sets the event to terminate thread.
   J_Event_ExitCondition(m_hEventNewImage);
   printf("CloseThreadHandle\n");
   // Close thread handle.
   CloseThreadHandle();
   printf("UnprepareBuffer()\n");
   // UnPrepare Buffers
   UnPrepareBuffer();

   // Close Stream
   if (m_hDS) {
     printf("J_DataStream_Close()\n");
     J_DataStream_Close(m_hDS);
     m_hDS = NULL;
   }

   return true;
}

//=====================================================================
// Prepare frame buffers
//=====================================================================
uint32_t 
CStreamThread::PrepareBuffer(void)
{
   int i;

   m_iValidBuffers = 0;

   for (i = 0; i < NUM_OF_BUFFER; i++) {
      // Makes the buffer for one frame.
      m_pAquBuffer[i] = new uint8_t[m_iBufferSize];

      // Announces the buffer pointer to the Acquisition engine.
      if (J_ST_SUCCESS != J_DataStream_AnnounceBuffer(m_hDS, m_pAquBuffer[i],
						      m_iBufferSize,
						      NULL,
						      &(m_pAquBufferID[i]))) {
	 delete m_pAquBuffer[i];
	 break;
      }
      // Queueing it.
      if (J_ST_SUCCESS != J_DataStream_QueueBuffer(m_hDS, m_pAquBufferID[i])) {
	 delete m_pAquBuffer[i];
	 break;
      }

      m_iValidBuffers++;
   }

   return m_iValidBuffers;
}

//=====================================================================
// Unprepare buffers
//=====================================================================
bool
CStreamThread::UnPrepareBuffer(void)
{
   void *pPrivate;
   void *pBuffer;
   uint32_t i;

   // Flush Queues
   J_DataStream_FlushQueue(m_hDS, ACQ_QUEUE_INPUT_TO_OUTPUT);
   J_DataStream_FlushQueue(m_hDS, ACQ_QUEUE_OUTPUT_DISCARD);

   for (i = 0; i < m_iValidBuffers; i++) {
      // Removes the frame buffer from the Acquisition engine.
      J_DataStream_RevokeBuffer(m_hDS, m_pAquBufferID[i], &pBuffer, &pPrivate);

      delete m_pAquBuffer[i];

      m_pAquBuffer[i] = NULL;
      m_pAquBufferID[i] = 0;
   }

   m_iValidBuffers = 0;

   return true;
}

//=====================================================================
// Close handles and stream
//=====================================================================
void
CStreamThread::CloseThreadHandle(void)
{
  if (m_hStreamThread) {
    printf("pthread_join...\n");
    pthread_join(m_hStreamThread, NULL);
    m_hStreamThread = 0;
  }
}

//=====================================================================
// Register a callback function, executed when new data arrives

//=====================================================================
void
CStreamThread::RegisterCallback(void (*callback) (J_tIMAGE_INFO * pAqImageInfo,CamStruct *camstr))
{
  callbackFn = callback;
}

int jaiSetThreadAffinityAndPriority(unsigned int *threadAffinity, int threadPriority,int affinElSize){
   int i;
   cpu_set_t mask;
   int ncpu;
   struct sched_param param;

   printf("Getting CPUs\n");
   ncpu = sysconf(_SC_NPROCESSORS_ONLN);
   printf("Got %d CPUs\n", ncpu);
   CPU_ZERO(&mask);
   printf("Setting %d CPUs\n", ncpu);
   for (i = 0; (i < ncpu) && (i<affinElSize*32); i++) {
     if (((threadAffinity[i/32]) >> (i%32)) & 1) {
	 CPU_SET(i, &mask);
      }
   }
   if (sched_setaffinity(0, sizeof(cpu_set_t), &mask))
      perror("Error in sched_setaffinity");

   printf("Setting setparam\n");
   param.sched_priority = threadPriority;
   if (sched_setparam(0, &param)) {
      printf
	  ("Error in sched_setparam: %s - probably need to run as root if this is important\n",
	   strerror(errno));
   }
   if(sched_setscheduler(0,SCHED_RR,&param))
     printf("sched_setscheduler: %s - probably need to run as root if this is important\n",strerror(errno));
  if(pthread_setschedparam(pthread_self(),SCHED_RR,&param))
    printf("error in pthread_setschedparam - maybe run as root?\n");
   return 0;
}


//=====================================================================
// Stream Processing Function
//=====================================================================
void
CStreamThread::StreamProcess(void *context)
{
   J_STATUS_TYPE iResult;
   uint32_t iSize;
   BUF_HANDLE iBufferID;
   J_tIMAGE_INFO tAqImageInfo;
   uint64_t prevTimestamp=0;
   J_COND_WAIT_RESULT iWaitResult;
   uint64_t iQueued = 0;
   uint64_t iQueuedTimeout = 0;
   uint64_t iAwait = 0;
   uint64_t iAwaitTimeout = 0;
   int firstTime=1;
   int nskipped=0;
   int ngot=0,ndup=0,nduplast=0;
   int err;
   struct tm timeouttime;
   float freq=0,freq2=0;
   int ignoreFrame;
   struct timeval t1,t2;
   char timebuf[32];
   time_t ttime;
   iResult = J_Event_CreateCondition(&m_hEventNewImage);
   if (iResult || (m_hEventNewImage == NULL))
      fprintf(stderr, " CreateEvent : NG\n");

   EVT_HANDLE hEvent;

   jaiSetThreadAffinityAndPriority(camstr->threadAffinity,camstr->threadPriority,camstr->threadAffinElSize);

   J_DataStream_RegisterEvent(m_hDS, EVENT_NEW_BUFFER, m_hEventNewImage,
			      (void **)&hEvent);

   iResult = J_DataStream_StartAcquisition(m_hDS, ACQ_START_NEXT_IMAGE, 0);
   timeouttime.tm_year=0;
   timebuf[0]='\0';
   // Mark stream acquisition as started
   m_bStreamStarted = true;
   gettimeofday(&t1,NULL);
   // Loop of Stream Processing
   while (m_bEnableThread) {
     err=1;//will be set to 0 if everything okay
     //try a sched_yield or a nanosleep?
      iResult = J_Event_WaitForCondition(m_hEventNewImage, 1000, &iWaitResult);

      if (iResult == J_ST_SUCCESS) {
	 if (iWaitResult == J_COND_WAIT_SIGNAL) {
	   timeouttime.tm_year=0;
	   err=0;
	   if (m_bEnableThread == false)
	     break;
	   iSize = (uint32_t) sizeof(void *);
	   iResult=J_Event_GetData(hEvent, &iBufferID, &iSize);
	   if(iResult!=J_ST_SUCCESS)
	     printf("J_Event_GetData failed: ignoring\n");
	   
	   // Gets the pointer to the frame buffer.
	   iSize = (uint32_t) sizeof(void *);
	   iResult =
	     J_DataStream_GetBufferInfo(m_hDS, iBufferID, BUFFER_INFO_BASE,
					&(tAqImageInfo.pImageBuffer),
					&iSize);
	   if(firstTime){//these things (size, type, etc) shouldn't change during operation
	     //firstTime=0;
	     //printf("before %d %d %d %d %d %d %d %d\n",(int)tAqImageInfo.iImageSize,(int)tAqImageInfo.iPixelType,(int)tAqImageInfo.iSizeX,(int)tAqImageInfo.iSizeY,(int)tAqImageInfo.iTimeStamp,(int)tAqImageInfo.iMissingPackets,(int)tAqImageInfo.iOffsetX,(int)tAqImageInfo.iOffsetY);
	     // Gets the effective data size.
	     iSize = (uint32_t) sizeof(uint32_t);
	     iResult =
	       J_DataStream_GetBufferInfo(m_hDS, iBufferID, BUFFER_INFO_SIZE,
					  &(tAqImageInfo.iImageSize), &iSize);
	     // Gets Pixel Format Type.
	     iSize = (uint32_t) sizeof(uint32_t);
	     iResult =
	       J_DataStream_GetBufferInfo(m_hDS, iBufferID,
					  BUFFER_INFO_PIXELTYPE,
					  &(tAqImageInfo.iPixelType), &iSize);
	     // Gets Frame Width.
	     iSize = (uint32_t) sizeof(uint32_t);
	     iResult =
	       J_DataStream_GetBufferInfo(m_hDS, iBufferID, BUFFER_INFO_WIDTH,
					  &(tAqImageInfo.iSizeX), &iSize);
	     // Gets Frame Height.
	     iSize = (uint32_t) sizeof(uint32_t);
	     iResult =
	       J_DataStream_GetBufferInfo(m_hDS, iBufferID, 
					  BUFFER_INFO_HEIGHT,
					  &(tAqImageInfo.iSizeY), &iSize);
	     // Gets X Offset.
	     iSize = (uint32_t) sizeof(uint32_t);
	     iResult =
	       J_DataStream_GetBufferInfo(m_hDS, iBufferID,
					  BUFFER_INFO_XOFFSET,
					  &(tAqImageInfo.iOffsetX), &iSize);
	     // Gets Y Offset.
	     iSize = (uint32_t) sizeof(uint32_t);
	     iResult =
	       J_DataStream_GetBufferInfo(m_hDS, iBufferID,
					  BUFFER_INFO_YOFFSET,
					  &(tAqImageInfo.iOffsetY), &iSize);
	   }
	   // Gets Timestamp.
	   iSize = (uint32_t) sizeof(uint64_t);
	   iResult =
	     J_DataStream_GetBufferInfo(m_hDS, iBufferID,
					BUFFER_INFO_TIMESTAMP,
					&(tAqImageInfo.iTimeStamp), &iSize);
	   // Gets Number of missing packets.
	   iSize = (uint32_t) sizeof(uint32_t);
	   iResult =
	     J_DataStream_GetBufferInfo(m_hDS, iBufferID,
					BUFFER_INFO_NUM_PACKETS_MISSING,
					&(tAqImageInfo.iMissingPackets),
					&iSize);
	   tAqImageInfo.iAnnouncedBuffers = m_iValidBuffers;
	   iSize = (uint32_t) sizeof(uint64_t);
	   iResult =
	     J_DataStream_GetStreamInfo(m_hDS,
					STREAM_INFO_CMD_NUMBER_OF_FRAMES_QUEUED,
					&iQueued, &iSize);
	   tAqImageInfo.iQueuedBuffers =
	     static_cast < uint32_t > (iQueued & 0x0ffffffffL);
	   
	   //printf("%d %d %d %d %d %d %d %d\n",(int)tAqImageInfo.iImageSize,(int)tAqImageInfo.iPixelType,(int)tAqImageInfo.iSizeX,(int)tAqImageInfo.iSizeY,(int)tAqImageInfo.iTimeStamp,(int)tAqImageInfo.iMissingPackets,(int)tAqImageInfo.iOffsetX,(int)tAqImageInfo.iOffsetY);
	   ignoreFrame=0;
	   if(tAqImageInfo.iTimeStamp==prevTimestamp){
	     ndup++;
	     ignoreFrame=1;
	   }else{
	     ngot++;
	   }
	   if((ngot%1000)==0){
	     gettimeofday(&t2,NULL);
	     freq=1./((t2.tv_sec-t1.tv_sec+1e-6*(t2.tv_usec-t1.tv_usec))/1000.);
	     freq2=freq/1000.*(1000+ndup-nduplast);
	     nduplast=ndup;
	     t1=t2;
	     printf("jaicam freq %g (%g), ngot=%d, ndup=%d, nskipped=%d size %d iAwait %d iQueued %d timestamp %qu %qu\n",freq,freq2,ngot,ndup,nskipped,tAqImageInfo.iImageSize,(int)iAwait,(int)iQueued,prevTimestamp,tAqImageInfo.iTimeStamp);
	   }
	   prevTimestamp=tAqImageInfo.iTimeStamp;
	   if (m_bEnableThread && ignoreFrame==0) {
	     if (iAwait < camstr->maxWaitingFrames) {//agb this was 2...
	       if (callbackFn == NULL) {
		 camstr->err=0;//don't wake p the RTC here - we're just testing to see what rate the camera frames arrive at...
		 //for (int ii = 0; ii < 32; ii++) {
		 //  printf("0x%x ", tAqImageInfo.pImageBuffer[ii]);
		 //}
	       } else {
		 callbackFn(&tAqImageInfo,camstr);
	       }
	     }else{
	       nskipped++;
	       //printf("iAwait=%d, not calling callback (nskipped = %d, ngot=%d) freq %g\n",(int)iAwait,nskipped,ngot,freq);
	     }
	     // Queue This Buffer Again
	     iResult = J_DataStream_QueueBuffer(m_hDS, iBufferID);
	   }
	   
	   iSize = (uint32_t) sizeof(uint64_t);
	   iResult = J_DataStream_GetStreamInfo(m_hDS,
						STREAM_INFO_CMD_NUMBER_OF_FRAMES_AWAIT_DELIVERY,
						&iAwait, &iSize);
	   if ((iResult == J_ST_SUCCESS) && (iAwait > 0)){
	     //printf("J_Event_SignalCondition()\n");
	     J_Event_SignalCondition(m_hEventNewImage);
	   }
	 } else if (iWaitResult == J_COND_WAIT_EXIT) {
	   // Exits from streaming loop.
	   break;
	 } else if (iWaitResult == J_COND_WAIT_TIMEOUT) {
	   if(timeouttime.tm_year==0){
	     ttime=time(NULL);
	     localtime_r(&ttime,&timeouttime);
	     asctime_r(&timeouttime,timebuf);
	   }
	   iSize = (uint32_t) sizeof(uint64_t);
	   iResult=J_DataStream_GetStreamInfo(m_hDS,STREAM_INFO_CMD_NUMBER_OF_FRAMES_QUEUED,&iQueuedTimeout, &iSize);
	   iSize = (uint32_t) sizeof(uint64_t);
	   iResult=J_DataStream_GetStreamInfo(m_hDS,STREAM_INFO_CMD_NUMBER_OF_FRAMES_AWAIT_DELIVERY,&iAwaitTimeout, &iSize);

	   printf("J_COND_WAIT_TIMEOUT first occurred at %s\n(probably need to reboot computer and powercycle camera several times), iAwait=%d,%d iQueued=%d,%d\n",timebuf,(int)iAwait,(int)iAwaitTimeout,(int)iQueued,(int)iQueuedTimeout);
	   if (m_bEnableThread == true){
	     iSize = (uint32_t) sizeof(void *);
	     iResult=J_Event_GetData(hEvent, &iBufferID, &iSize);
	     if(iResult!=J_ST_SUCCESS)
	       printf("J_Event_GetData failed: ignoring\n");
	     else{
	       // Gets the pointer to the frame buffer.
	       iSize = (uint32_t) sizeof(void *);
	       iResult =J_DataStream_GetBufferInfo(m_hDS, iBufferID, BUFFER_INFO_BASE,&(tAqImageInfo.pImageBuffer),&iSize);
	       printf("J_Event_GetData succeeded!!! First pixel 0x%x pxl[30] 0x%x\n",tAqImageInfo.pImageBuffer[0],tAqImageInfo.pImageBuffer[30]);
	     }


	   }


	 } else if (iWaitResult == J_COND_WAIT_ERROR) {
	   printf("J_Event_WaitForCondition returned error Wait result!\n");
	 } else {
	   printf("J_Event_WaitForCondition returned unknown Wait result (%d)!\n",iWaitResult);
	 }
      }else{
	printf("J_Event_WaitForCondition failed\n");
      }
      //do something with err now...
      if(err){
	pthread_mutex_lock(&camstr->m);
	camstr->err=err;
	camstr->dataReady=0;
	camstr->tail=camstr->head;
	pthread_cond_signal(&camstr->cond);
	pthread_mutex_unlock(&camstr->m);
      }  
   }
   
   if (m_hEventNewImage != NULL)
     J_Event_CloseCondition(m_hEventNewImage);
   
   m_hEventNewImage = NULL;
}

//===========================================================================




/**
   Find out if this SO library supports your camera.

*/
//extern "C" camOpenFn(char *name,int n,int *args,void **camHandle,int npxls,short *pxlbuf,int ncam,int *pxlx,int* pxly,int* frameno);


//unsigned char *buffer;    // Buffer to receive frame from camera
//int *bufferframeno;//stores framecount corresponding to buffer
//pthread_mutex_t jaimutex; // Protect access to camera ring buffer
//pthread_cond_t jaicond;   // Signal new camera frame arrived
//int jaiDataReady;         // flag - set if new frame has been copied to buffer.
//----------------------------------------------
// StreamCBFunc
//----------------------------------------------
// Pulnix API callback function
// This gets called asynchronously by the JAI API 
// when a frame arrived interrupt occurs.
// Note - since buffer is a global, we can't tell 
// for which camera the interrupt occured, so currently, 
// is only safe with 1 camera.

//This gets called whenever there is image data available.
static void 
StreamCBFunc(J_tIMAGE_INFO * pAqImageInfo,CamStruct *camstr)
{
  // printf("callback: size = %d\n", pAqImageInfo->iImageSize);
  static int framecnt=0;
  framecnt++;
  if ((pAqImageInfo->iAnnouncedBuffers - pAqImageInfo->iQueuedBuffers) > 10) {
    printf("Dropped a frame! (%d %d %d)\n",framecnt,pAqImageInfo->iAnnouncedBuffers,pAqImageInfo->iQueuedBuffers);
    return;
  }
  pthread_mutex_lock(&camstr->m);
  //printf("worker got lock\n");
  camstr->err=0;
  //copy the data
  memcpy(camstr->ringBuf[camstr->head], pAqImageInfo->pImageBuffer, pAqImageInfo->iImageSize);
  camstr->bufframeno[camstr->head]=framecnt;
  //*bufferframeno=framecnt;

  // advance the ring head pointer
  camstr->head++;
  camstr->head &= BUFMASK;
  //buffer = camstr->ringBuf[camstr->head];
  //bufferframeno=&camstr->bufframeno[camstr->head];
  if (camstr->head == camstr->tail) {	
    // ring overflow; overwrite tail
    camstr->tail++;
    camstr->tail &= BUFMASK;
    printf("Ring buffer overflow %d\n",framecnt);
  }
  camstr->dataReady = 1;
  //if(err){
  // camstr->dataReady=0;
  // camstr->tail=camstr->head;
  //   }
  // Unblock the buffer read thread, if it's waiting
  pthread_cond_signal(&camstr->cond);
  pthread_mutex_unlock(&camstr->m);
  return;
}




int setInt64Val(const char *name, int *val,CamStruct *camstr){
  J_STATUS_TYPE retval;
  int64_t int64Val;

  if((retval=J_Camera_GetNodeByName(camstr->m_hCam,(int8_t *)name,&camstr->hNode))!=J_ST_SUCCESS){
    printf("J_Camera_GetNodeByName failed for %s: %d\n",name,retval);
    return 1;
  }
  //Set the value
  int64Val=(int64_t)*val;
  if((retval=J_Node_SetValueInt64(camstr->hNode,0,int64Val))!=J_ST_SUCCESS){
    printf("J_Node_SetValueInt64 failed for %s: %d\n",name,retval);
    return 1;
  }
  //Now retrieve the value.
  if((retval=J_Node_GetValueInt64(camstr->hNode,0,&int64Val))!=J_ST_SUCCESS){
    printf("J_Node_GetValueInt64 failed for %s: %d\n",name, retval);
    return 1;
  }
  *val=(int)int64Val;
  return 0;
}
int setEnumVal(const char *name,const char *val,CamStruct *camstr){//val will be enumentry something or other
  int8_t pBuffer[80]; 
  J_STATUS_TYPE retval;
  int64_t int64Val;
  uint32_t pSize;
  if((retval=J_Camera_GetNodeByName(camstr->m_hCam,(int8_t*)val,&camstr->hNode))!=J_ST_SUCCESS){
    printf("J_Camera_GetNodeByName failed for %s: %d\n",name,retval);
    return 1;
  }
  if((retval=J_Node_GetEnumEntryValue(camstr->hNode, &int64Val))!=J_ST_SUCCESS){
    printf("Failed to get %s node value: %d\n",val,retval);
    return 1;
  }
  if((retval = J_Camera_GetNodeByName(camstr->m_hCam,(int8_t *)name,&camstr->hNode))!=J_ST_SUCCESS){
    printf("Failed to get %s node: %d\n",name,retval);
    return 1;
  }

  pSize=80;
  if(J_Node_GetValueString(camstr->hNode,0,pBuffer,&pSize)!=J_ST_SUCCESS){
    printf("failed J_Node_GetValueString - continuing anyway\n");
  }else{
    pBuffer[79]='\0';
    if(pSize>=0 && pSize<79)
      pBuffer[pSize]='\0';
    printf("J_Node_GetValueString gave value %s, size %d\n",(char*)pBuffer,(int)pSize);
  } 
  if((retval=J_Node_SetValueInt64(camstr->hNode, 0, int64Val))!=J_ST_SUCCESS){
    printf("Failed to set %s: %i\n",name,retval);
  }
  pSize=80;
  if(J_Node_GetValueString(camstr->hNode,0,pBuffer,&pSize)!=J_ST_SUCCESS){
    printf("failed J_Node_GetValueString - continuing anyway\n");
  }else{
    pBuffer[79]='\0';
    if(pSize>=0 && pSize<79)
      pBuffer[pSize]='\0';
    printf("J_Node_GetValueString gave value %s, size %d\n",(char*)pBuffer,(int)pSize);
  } 
  return 0;
}

#define NODE_NAME_WIDTH     "Width"
#define NODE_NAME_HEIGHT    "Height"
#define NODE_NAME_GAIN      "GainRaw"
#define NODE_NAME_ACQSTART  "AcquisitionStart"
#define NODE_NAME_ACQSTOP   "AcquisitionStop"
#define NODE_NAME_PIXFORMAT "PixelFormat"
#define REG_ADDR_SCDA0      (0x0d18)
#define MULTICASTADDR 0
#define NODE_NAME_OFFSETX "OffsetX"
#define NODE_NAME_OFFSETY "OffsetY"
#define NODE_NAME_EXPOSUREMODE "ExposureMode"
#define NODE_NAME_EXPOSURETIME "ContinuousProgrammable"//"AsyncExposureTimeRaw"//this is a guess?

//----------------------------------------------
// Camara setting
//----------------------------------------------
/*
Messages from JAI...
To change the frame rate, you can use your own pulse generator, or
internal Pulse Generator.
Following is an example to setup Pulse Generator from JAI Control Tool.

AcquisitionAndTriggerControls category > ExposureMode: PulseWidthControl
CounterAndTimerControls > 
TimerSelector: Timer1
TimerDelayRaw(ns): 1024
TimerDurationRaw(ns): 4096 
TimerFrequency(fps): 1.00141
TimerGranularityFactor: 6499
TimerTriggerSource: Continuous

IPEngine > ProgrammableLogicController > LookupTable > Q4 >
PLC_Q4_Variable0: PLC_I7_Not

Pulse will be created as below.
High duration = TimerDurationRaw x (TimerGranularityFactor + 1) x 30 
Low duration = (TimerDelayRaw + 1) x (TimerGranularityFactor + 1) x 30

Changing TimerDurationRaw will change the exposure time as well as the
frame rate.

ExposureMode = AsyncShutter_Preset9


I have a camera running here with scan mode D controlled by the internal
pulse generator.

I have set scan mode D, set width and height to 224/160. Then I use the
GPIO lookup table Q4= PLC_I7_Not.

That makes the pulse width control the exposure. It seems like the clock
frequency is slightly off, so timerdelayraw = 30966 and
timerdurationraw= 3350 gives me 1000 fps. I haven't measured that with
an oscilloscope, so it might be a PC problem. According to the tool,
these settings should give me around 971Hz, but according to the frame
counter, it gives 1000Hz.

I did have problems, if I didn's set width and height after changing to
scan mode D. Could be a minor detail in our GenICam xml file.

*/

int
Camera_JAI(CamStreamStruct *camstrstr, unsigned int cam, int imgSizeX, int imgSizeY,int imgOffsetX,int imgOffsetY,int exptime,int scanmode)
{
  CamStruct *camstr=camstrstr->camstr;
  J_STATUS_TYPE retval;
   bool8_t bHasChange;
   uint32_t iNumDev;
   uint32_t iSize;
   int iPos;
   int64_t int64Val;
   SIZE ViewSize;
   //int8_t *cbuf;
   //uint32_t pNum;
   int8_t pBuffer[80];
   uint32_t pSize;
   //uint32_t i;
   double doubleVal;
   //NODE_HANDLE tmpNode;
   //int8_t          sNodeName[256];
   //uint32_t        size;
   //J_NODE_TYPE pNodeType;
   int minscan=1;
   int scan;
   printf("Initialisint JAI (Camera_JAI)\n");
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
   if (cam >= iNumDev) {
     printf("Error - trying to set camera %d out of %d\n", 
	     cam, iNumDev);
      return 1;
   }
   printf("Setting camera %d out of %d\n",cam,iNumDev);
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

   pSize=80;
   if(J_Node_GetValueString(camstr->hNode,0,pBuffer,&pSize)!=J_ST_SUCCESS){
     printf("failed J_Node_GetValueString - continuing anyway\n");
   }else{
     pBuffer[79]='\0';
     if(pSize>=0 && pSize<79)
       pBuffer[pSize]='\0';
     printf("J_Node_GetValueString gave value %s, size %d\n",(char*)pBuffer,(int)pSize);
   } 

   retval = J_Node_SetValueInt64(camstr->hNode, 0, int64Val);
   if (retval != J_ST_SUCCESS) {
     printf("Failed to set Image Format: %i\n", retval);
   }
   printf("Pixel Format set to %li\n", (long int)int64Val);

   pSize=80;
   if(J_Node_GetValueString(camstr->hNode,0,pBuffer,&pSize)!=J_ST_SUCCESS){
     printf("failed J_Node_GetValueString - continuing anyway\n");
   }else{
     pBuffer[79]='\0';
     if(pSize>=0 && pSize<79)
       pBuffer[pSize]='\0';
     printf("J_Node_GetValueString gave value %s, size %d\n",(char*)pBuffer,(int)pSize);
   } 




   /*scanmode 
     1= 640x480
     2= 640x160
     3= 224x480
     4= 224x160
   */

   //Now try to set the scan mode - to use middle 224x160 pixels if possible.
   if(imgSizeX+imgOffsetX<=224){
     if(imgSizeY+imgOffsetY<=160){
       minscan=4;
       //if((retval=J_Camera_GetNodeByName(camstr->m_hCam,(int8_t*)"EnumEntry_ScanMode_D_224X160",&camstr->hNode))!=J_ST_SUCCESS)
       // printf("Couldn't get EnumEntry_ScanMode_D_224X160\n");
       //else
       //	 printf("Using mode D_224x160 - max frame rate 1250Hz\n");
     }else if(imgSizeY+imgOffsetY<=480){
       minscan=3;
       //if((retval=J_Camera_GetNodeByName(camstr->m_hCam,(int8_t*)"EnumEntry_ScanMode_C_224X480",&camstr->hNode))!=J_ST_SUCCESS)
       //	 printf("Couldn't get EnumEntry_ScanMode_C_224X480\n");
       //else
       //	 printf("Using mode C_224x480 - max frame rate 500Hz\n");
     }else{
       printf("Image doesn't fit on camera - error\n");
       return -1;
     }
   }else if(imgSizeX+imgOffsetX<=640){
     if(imgSizeY+imgOffsetY<=160){
       minscan=2;
       //if((retval=J_Camera_GetNodeByName(camstr->m_hCam,(int8_t*)"EnumEntry_ScanMode_B_640X160",&camstr->hNode))!=J_ST_SUCCESS)
       //	 printf("Couldn't get EnumEntry_ScanMode_B_640X160\n");
       //else
       //	 printf("Using mode B_640x160 - max frame rate 540Hz\n");
     }else if(imgSizeY+imgOffsetY<=480){
       minscan=1;
       //if((retval=J_Camera_GetNodeByName(camstr->m_hCam,(int8_t*)"EnumEntry_ScanMode_A_640X480",&camstr->hNode))!=J_ST_SUCCESS)
       //printf("Couldn't get EnumEntry_ScanMode_A_640X480\n");
       //else
       //printf("Using mode A_640x480 - max frame rate 200Hz\n");
     }else{
       printf("Image doesn't fit on camera - error\n");
       return -1;
     }
   }else{
     printf("Image doesn't fit on camera - error\n");
     return -1;
   }
   scan=minscan;
   if(scanmode>0){
     if(scanmode<minscan)
       scan=scanmode;
   }
   if(scan<1 || scan>4){
      printf("Image doesn't fit on camera - error\n");
      return -1;
   }else if(scan==1){
     retval=J_Camera_GetNodeByName(camstr->m_hCam,(int8_t*)"EnumEntry_ScanMode_A_640x480",&camstr->hNode);
   }else if(scan==2){
     retval=J_Camera_GetNodeByName(camstr->m_hCam,(int8_t*)"EnumEntry_ScanMode_B_640X160",&camstr->hNode);
   }else if(scan==3){
     retval=J_Camera_GetNodeByName(camstr->m_hCam,(int8_t*)"EnumEntry_ScanMode_C_224X480",&camstr->hNode);
   }else if(scan==4){
     retval=J_Camera_GetNodeByName(camstr->m_hCam,(int8_t*)"EnumEntry_ScanMode_D_224X160",&camstr->hNode);
   }
   if(retval!=J_ST_SUCCESS){
     printf("Couldn't get EnumEntry_ScanMode_%s\n",scan==4?"D_224X160":(scan==3?"C_224X480":(scan==2?"B_640X160":"A_640x480")));
   }else{
     printf("Using mode %s - max frame rate %dHz\n",scan==4?"D_224X160":(scan==3?"C_224X480":(scan==2?"B_640X160":"A_640x480")),scan==4?1250:(scan==1?200:500));
     if((retval=J_Node_GetEnumEntryValue(camstr->hNode,&int64Val))!=J_ST_SUCCESS){
       printf("Failed to GetEnumEntryValue\n");
     }else if((retval=J_Camera_GetNodeByName(camstr->m_hCam,(int8_t*)"ScanMode",&camstr->hNode))!=J_ST_SUCCESS){
       printf("GetNodeByName(scanmode) failed\n");
     }else{//now set the scan mode.
       pSize=80;
       if(J_Node_GetValueString(camstr->hNode,0,pBuffer,&pSize)!=J_ST_SUCCESS){
	 printf("failed J_Node_GetValueString - continuing anyway\n");
       }else{
	 pBuffer[79]='\0';
	 if(pSize>=0 && pSize<79)
	   pBuffer[pSize]='\0';
	 printf("J_Node_GetValueString gave value %s, size %d\n",(char*)pBuffer,(int)pSize);
       } 
       if(J_Node_SetValueInt64(camstr->hNode,0,int64Val)!=J_ST_SUCCESS){
	 printf("Failed to set mode\n");
       }
       //if(J_Node_SetValueString(camstr->hNode,0,(int8_t*)"D")!=J_ST_SUCCESS){
       //  printf("Failed to set mode to D_224x160\n");
       //}
       pSize=80;
       if(J_Node_GetValueString(camstr->hNode,0,pBuffer,&pSize)!=J_ST_SUCCESS){
	 printf("failed J_Node_GetValueString - continuing anyway\n");
       }else{
	 pBuffer[79]='\0';
	 if(pSize>=0 && pSize<79)
	   pBuffer[pSize]='\0';
	 printf("J_Node_GetValueString gave value %s, size %d\n",(char*)pBuffer,(int)pSize);
       }     
     }
   }
   //Now try to set the acquisition mode - to continuous
   if((retval=J_Camera_GetNodeByName(camstr->m_hCam,(int8_t*)"EnumEntry_AcquisitionMode_Continuous",&camstr->hNode))!=J_ST_SUCCESS){
     printf("Couldn't get EnumEntry_AcquisitionMode_Continuous\n");
   }else if((retval=J_Node_GetEnumEntryValue(camstr->hNode,&int64Val))!=J_ST_SUCCESS){
     printf("Failed to GetEnumEntryValue\n");
   }else if((retval=J_Camera_GetNodeByName(camstr->m_hCam,(int8_t*)"AcquisitionMode",&camstr->hNode))!=J_ST_SUCCESS){
     printf("GetNodeByName(acquisitionMode) failed\n");
   }else{//now set the acq mode.
     pSize=80;
     if(J_Node_GetValueString(camstr->hNode,0,pBuffer,&pSize)!=J_ST_SUCCESS){
       printf("failed J_Node_GetValueString - continuing anyway\n");
     }else{
       pBuffer[79]='\0';
       if(pSize>=0 && pSize<79)
	 pBuffer[pSize]='\0';
       printf("J_Node_GetValueString gave value %s, size %d\n",(char*)pBuffer,(int)pSize);
     } 
     if(J_Node_SetValueInt64(camstr->hNode,0,int64Val)!=J_ST_SUCCESS){
       printf("Failed to set mode to Continuous\n");
     }
     //if(J_Node_SetValueString(camstr->hNode,0,(int8_t*)"D")!=J_ST_SUCCESS){
     //  printf("Failed to set mode to D_224x160\n");
     //}
     pSize=80;
     if(J_Node_GetValueString(camstr->hNode,0,pBuffer,&pSize)!=J_ST_SUCCESS){
       printf("failed J_Node_GetValueString - continuing anyway\n");
     }else{
       pBuffer[79]='\0';
       if(pSize>=0 && pSize<79)
	 pBuffer[pSize]='\0';
       printf("J_Node_GetValueString gave value %s, size %d\n",(char*)pBuffer,(int)pSize);
     }     
   }

   //Now set up the internal/external triggering.
   if(setInt64Val("TimerDelayRaw",&camstr->timerDelayRaw,camstr)!=0){
     printf("setInt64Val failed for TimerDelayRaw\n");
     return 1;
   }
   if(setInt64Val("TimerDurationRaw",&camstr->timerDurationRaw,camstr)!=0){
     printf("setInt64Val failed for TimerDurationRaw\n");
     return 1;
   }
   if(setInt64Val("TimerGranularityFactor",&camstr->timerGranularityFactor,camstr)!=0){
     printf("setInt64Val failed for TimerGranularityFactor\n");
     return 1;
   }
   //now read back the info to get computed framerate...
   if((retval=J_Camera_GetNodeByName(camstr->m_hCam,(int8_t *)"TimerFrequency", &camstr->hNode))!=J_ST_SUCCESS){
     printf("J_Camera_GetNodeByName failed for TimerFrequency: %d\n",retval);
     return 1;
   }
   if((retval=J_Node_GetValueDouble(camstr->hNode,0,&doubleVal))!=J_ST_SUCCESS){
     printf("J_Node_GetValueDouble failed for TimerFrequency: %d\n",retval);
     return 1;
   }
   printf("Frequency %gHz, delayRaw %d, durationRaw %d, granularity %d\n",doubleVal,camstr->timerDelayRaw,camstr->timerDurationRaw,camstr->timerGranularityFactor);
   //Now set up the trigger sources...
   if(setEnumVal("TimerTriggerSource","EnumEntry_TimerTriggerSource_Continuous",camstr)!=0){
     printf("setEnumVal failed TriggerSource\n");
     return 1;
   }

   //set signalRoutingBlock PCL_I0 to External_Trigger_In_Pin6.  Note, can do this regardless of whether internal or external triggering.
   printf("todo... check signalRoutingBlock JAI pulnix camera\n");
   if(setEnumVal("PLC_I0","EnumEntry_PLC_I0_External_Trigger_In_Pin6",camstr)!=0){
     printf("setEnumVal failed for PLC_I0 to ExternalTrigger\n");
     return 1;
   }

   
   if(camstr->internalTrigger){//For internal triggering need...
     if(setEnumVal("PLC_Q4_Variable0","EnumEntry_PLC_Q4_Variable0_PLC_I7_Not",camstr)!=0){
       printf("setEnumVal failed PLC_Q4_Variable0\n");
       return 1;
     }
   }else{
     //and for external triggering need
     //PLC_Q4_Variable0 set to PLC_I0
     if(setEnumVal("PLC_Q4_Variable0","EnumEntry_PLC_Q4_Variable0_PLC_I0",camstr)!=0){
       printf("setEnumVal failed PLC_Q4_Variable0\n");
       return 1;
     }
   }


   //This is the same as pulseWidthControl on newer firmware cameras...
   if(setEnumVal("ExposureMode","EnumEntry_ExposureMode_AsyncShutter_Preset9",camstr)!=0){
     printf("setEnumVal failed ExposureMode\n");
     return 1;
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
   printf("Camera size is %d x %d\n",(int)ViewSize.cy,(int)ViewSize.cx);


   // Get & Set OffsetX from the camera
   retval = J_Camera_GetNodeByName(camstr->m_hCam,
				   (int8_t *)NODE_NAME_OFFSETX, &camstr->hNode);
   if (retval != J_ST_SUCCESS) {
     printf("J_Camera_GetNodeByName failed: %d\n", retval);
     return 1;
   }
   iPos = imgOffsetX;
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
   // Get & Set OffsetY from the camera
   retval = J_Camera_GetNodeByName(camstr->m_hCam,
				   (int8_t *)NODE_NAME_OFFSETY, &camstr->hNode);
   if (retval != J_ST_SUCCESS) {
     printf("J_Camera_GetNodeByName failed: %d\n", retval);
     return 1;
   }
   iPos = imgOffsetY;
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

   //get and set the exposure time?




   // Starts streaming thread
   if ((camstrstr->pStreamObj = new CStreamThread()) == NULL) {
     printf("Failed to create StreamThread object!\n");
     return (1);
   }
   camstrstr->pStreamObj->camstr=camstr;
   if (camstrstr->pStreamObj->CreateStreamThread(camstr->m_hCam, 0,
				      ViewSize.cx * ViewSize.cy * sizeof(short), 
						 0)){//,camstr->threadPriority+1)) {

     camstr->threadid=camstrstr->pStreamObj->m_hStreamThread;
     if(camstr->testmode==0)
       camstrstr->pStreamObj->RegisterCallback(&StreamCBFunc);
     else{
       printf("In test mode (libjaicam.so) - not registering jaicam callback\n");
       camstrstr->pStreamObj->RegisterCallback(NULL);
     }
     // Wait for the stream thread to start the acquisition
     while (!camstrstr->pStreamObj->m_bStreamStarted) { ;
     }


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
   } else { 
     return 1;
   }
}

//------------------------------------------------
// Close camera
//------------------------------------------------
int
Close_JAI(CamStruct *camstr,CStreamThread *thr)
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

   // Close stream
   //camstr->pStreamObj->TerminateStreamThread();
   
   if(thr!=NULL){
     printf("Calling TerminateStreamThread()\n");
     thr->TerminateStreamThread();
   }
   if (camstr->m_hCam) {
      // Close camera
     printf("J_Camera_Close\n");
     retval = J_Camera_Close(camstr->m_hCam);
     if (retval != J_ST_SUCCESS)
       printf("J_Camera_Close failed\n");
     camstr->m_hCam = NULL;
   }

   if (camstr->m_hFactory) {
     printf("J_Factory_Close\n");
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
      pthread_cond_destroy(&camstr->cond2);
      pthread_mutex_destroy(&camstr->m);
      //pthread_cond_destroy(&jaicond);
      //pthread_mutex_destroy(&jaimutex);
      safefree((void *)camstr->newframe);
      safefree(camstr->pxlsTransferred);
      safefree(camstr->setFrameNo);
      for(i=0; i<NBUF; i++){
	safefree(camstr->ringBuf[i]);
      }
      free(camstr);
   }
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
int camOpen(char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **camHandle,int nthreads,unsigned int frameno,unsigned int **camframeno,int *camframenoSize,int npxls,int ncam,int *pxlx,int* pxly){
   CamStruct *camstr;
   CamStreamStruct *camstrstr;
   unsigned int i;
   int err;
   unsigned short *tmps;
   printf("Initialising camera %s\n", name);
   if (ncam != 1) {
      printf
	("ERROR: the JAI PULNIX interface doesn't support >1 camera\n");
      return 1;
   }

   if ((*camHandle = malloc(sizeof(CamStreamStruct))) == NULL) {
      perror("Couldn't malloc camera handle");
      return 1;
   }

   printf("Malloced camstrstr\n");
   memset(*camHandle, 0, sizeof(CamStreamStruct));

   if((arr->pxlbuftype!='H') || ((int)arr->pxlbufsSize!=(int)sizeof(unsigned short)*npxls)){
     //need to resize the pxlbufs...
     arr->pxlbufsSize=sizeof(unsigned short)*npxls;
     arr->pxlbuftype='H';
     arr->pxlbufelsize=sizeof(unsigned short);
     tmps=(unsigned short*)realloc(arr->pxlbufs,arr->pxlbufsSize);
     if(tmps==NULL){
       if(arr->pxlbufs!=NULL)
	 free(arr->pxlbufs);
       printf("pxlbuf malloc error in camfile.\n");
       arr->pxlbufsSize=0;
       free(*camHandle);
       *camHandle=NULL;
       return 1;
     }
     arr->pxlbufs=(void*)tmps;
     memset(tmps,0,arr->pxlbufsSize);
   }

   camstrstr=static_cast <CamStreamStruct*>(*camHandle);
   if((camstrstr->camstr=(CamStruct*)malloc(sizeof(CamStruct)))==NULL){
     printf("Couldn't malloc camstr\n");
     free(*camHandle);
     *camHandle=NULL;
     return 1;
   }
   camstr = ((CamStreamStruct *) *camHandle)->camstr;
   camstr->imgdata = (short*)arr->pxlbufs;
   if(*camframenoSize<1){
     if((*camframeno=malloc(sizeof(unsigned int)))==NULL){
       printf("Couldn't allocate frameno\n");
       free(*camHandle);
       *camHandle=NULL;
       return 1;
     }
     *camframenoSize=1;
   }
   camstr->userFrameNo = (unsigned int *)*camframeno;
   camstr->framing=1;
   memset(camstr->userFrameNo, 0, sizeof(int) * ncam);
   camstr->ncam = ncam;
   camstr->npxls = npxls;	//*pxlx * *pxly;
   camstr->dataReady = 0;
   camstr->head = 0;
   camstr->tail = 0;

   TEST(camstr->setFrameNo = (int *)calloc(ncam, sizeof(int)));
   printf("malloced things\n");

   if (n>0 && n == 14+args[2]) {
     if(args[0]!=2){
       printf("Wrong number of bytes per pixel specified - forcing to 2\n");
     }
     camstr->bytesPerPxl = 2;//args[0];
     camstr->timeout.tv_sec = args[1] / 1000;	//timeout in ms.
     camstr->timeout.tv_nsec = (args[1] % 1000) * 1000000;

     camstr->threadAffinElSize = args[2];	//thread affinity element size
     camstr->threadPriority = args[3];	//thread priority
     camstr->offsetX=args[4];
     camstr->offsetY=args[5];
     camstr->exptime=args[6];//not used.
     camstr->scanmode=args[7];
     camstr->timerDelayRaw=args[8];
     camstr->timerDurationRaw=args[9];
     camstr->timerGranularityFactor=args[10];
     camstr->testmode=args[11];
     camstr->maxWaitingFrames=(unsigned int)args[12];
     camstr->internalTrigger=args[13];
     camstr->threadAffinity=(unsigned int*)&args[14];
     //Pulse will be created as below.
     //High duration = TimerDurationRaw x (TimerGranularityFactor + 1) x 30 
     //Low duration = (TimerDelayRaw + 1) x (TimerGranularityFactor + 1) x 30
     //Changing TimerDurationRaw will change the exposure time as well as the
     //frame rate.
     //Framerate is therefore 1e9/((DurationRaw + DelayRaw + 1) * (Gran + 1) * 30)
     //Some combinations do not work, others are noisy so test before using it.



   } else {
      printf ("wrong number of cmd args, should be 12: bytesperpxl,"
	      " timeout, nAffin, thread priority, offsetX, offsetY, exptime, scanmode timerDelayRaw timerDurationRaw timerGranularityFactor testmode maxWaitingFrames internalTrigger flag (1==internal, 0==external), threadAffinity[Naffin]\n"); 
      dofree(camstr);
      free(*camHandle);
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
     printf("Error initialising condition variable %d\n", 0);
     dofree(camstr);
     free(*camHandle);
     *camHandle = NULL;
     return 1;
   }
   if (pthread_cond_init(&camstr->cond2, NULL) != 0) {
     printf("Error initialising condition variable 2 \n");
     dofree(camstr);
     free(*camHandle);
     *camHandle = NULL;
     return 1;
   }

   /*if (pthread_cond_init(&jaicond, NULL) != 0) {
      perror("Error initialising thread condition variable");
      dofree(camstr);
      *camHandle = NULL;
      return 1;
      }*/
   //maybe think about having one per camera???
   if (pthread_mutex_init(&camstr->m, NULL) != 0) {
      perror("Error initialising camstr mutex");
      dofree(camstr);
      free(*camHandle);
      *camHandle = NULL;
      return 1;
   }


   /*if (pthread_mutex_init(&jaimutex, NULL) != 0) {
      perror("Error initialising jai mutex variable");
      dofree(camstr);
      *camHandle = NULL;
      return 1;
      }*/
   printf("done mutex\n");


   printf("doingf ringBuf\n");
   for (i = 0; i < NBUF; i++) {
     camstr->ringBuf[i] = (unsigned char *)malloc(camstr->bytesPerPxl 
						  * camstr->npxls);
     if (camstr->ringBuf[i] == NULL) {
	 perror("Couldn't allocate ring buffer");
	 dofree(camstr);
	 free(*camHandle);
	 *camHandle = NULL;
	 return 1;
      }

     printf("Clearing ring buffer[ %d ]...\n",i);
      memset(camstr->ringBuf[i], 0,
	     (camstr->bytesPerPxl * camstr->npxls));
   }

   printf("done dmabuf\n");
   //buffer = camstr->ringBuf[0];
   //bufferframeno=&camstr->bufframeno[0];

   //Now do the JAI stuff...
   err = 0;
   //jaiDataReady = 0;
   
   camstr->open = 1;
   for (i = 0; err == 0 && i < (unsigned int)ncam; i++) {
     err = Camera_JAI(camstrstr, i, pxlx[i], pxly[i],camstr->offsetX,camstr->offsetY,camstr->exptime,camstr->scanmode);
     if (err) {
       printf("Failed to open pulnix camera %d\n", i);
     }else
       printf("Opened pulnix camera %d\n",i);
   }

   /*
   if(err==0){
     err = pthread_create(&camstr->threadid, NULL, worker,
			  (void *)camstr);
     if (err) {
       perror("pthread_create() failed");
     } else {
       printf("created worker thread\n");
     }   

     }*/
   if (err) {
     dofree(camstr);
     free(*camHandle);
     *camHandle = NULL;
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
   CStreamThread *cst;
   //int i;

   printf("Closing camera\n");
   if (*camHandle == NULL)
      return 1;
   camstr = ((CamStreamStruct *) *camHandle)->camstr;
   cst=((CamStreamStruct *)*camHandle)->pStreamObj;
   pthread_mutex_lock(&camstr->m);
   camstr->open = 0;
   camstr->framing = 0;
   printf("signalling\n");
   pthread_cond_signal(&camstr->cond);

   //pthread_mutex_unlock(&camstr->m);
   //pthread_mutex_lock(&camstr->m);
   pthread_mutex_unlock(&camstr->m);
   printf("close_JAI\n");
   Close_JAI(camstr,cst);
   printf("joining\n");
   pthread_join(camstr->threadid, NULL);  //wait for worker thread to complete
   printf("joined - locking\n");

   dofree(camstr);
   free(*camHandle);
   *camHandle = NULL;
   printf("Camera closed\n");
   return 0;
}


/**
   Called when we're starting processing the next frame.  This doesn't
   actually wait for any pixels. 
*/
#ifdef __cplusplus
extern "C" 
#endif
int
camNewFrameSync(void *camHandle,unsigned int thisiter,double starttime)
{
   //printf("camNewFrame\n");
   CamStruct *camstr;
   //int i;

   if (camHandle == NULL){// || camstr->framing == 0) {
      //printf("called camNewFrame with camHandle==NULL\n");
      return 1;
   }
   camstr = ((CamStreamStruct *) camHandle)->camstr;
   if(camstr->framing==0){
     return 1;
   }
   pthread_mutex_lock(&camstr->m);
   //printf("New frame\n");
   camstr->transferRequired=1;
   //*(camstr->userFrameNo)++;
   //camstr->userFrameNo[0]++;
   //printf("Frameno %d\n",*(camstr->userFrameNo));
   pthread_mutex_unlock(&camstr->m);
   return 0;
}

/**
   Wait for the first n pixels of the current frame to arrive.
   Can be called by many threads simultaneously.
*/
#ifdef __cplusplus
extern "C" 
#endif
int 
camWaitPixels(int n,int cam,void *camHandle)
{
  // printf("camWaitPixels %d, camera %d\n",n,cam);
  //For andor, we actually have to wait for the whole frame...
  CamStruct *camstr;// = static_cast <CamStruct*>(camHandle);
  //char *cd;
  //int i;
  int err=0;
  struct timespec timeout;	//sec and ns
  if (camHandle == NULL) {
    printf("called camWaitPixels with camHandle==NULL\n");
    return 1;
  }
  camstr=((CamStreamStruct*)camHandle)->camstr;
  //char tmp;
  //static struct timeval t1;
  //struct timeval t2;
  //struct timeval t3;
   if (cam != 0) {
     printf("Error: camWaitPixels called for camera %d:"
	    "only one camera (0) is supported\n", cam);
     return 1;
   }
   if (camstr->framing == 0) {
     printf("camera is not framing\n");
     return 1;
   }
   //printf("camWaitPixels %d %d %d\n",n,cam,camstr->transferRequired);
  if (camstr->transferRequired!=0) {
    pthread_mutex_lock(&camstr->m);
    if(camstr->transferRequired==1){//now we've got the lock, just check it is still required - if so, do the transfer
      camstr->transferRequired=2;
      //err = camstr->err;
      camstr->err=0;
      while(camstr->dataReady==0 && camstr->err==0) {
	// printf("Waiting for pixels...\n");
	//pthread_cond_wait(&camstr->cond, &camstr->m); 
	//Do a timed wait instead...
	clock_gettime(CLOCK_REALTIME, &timeout);
	timeout.tv_sec+=camstr->timeout.tv_sec;
	timeout.tv_nsec+=camstr->timeout.tv_nsec;
	if (timeout.tv_nsec > 1.0e9) {
	  timeout.tv_nsec -= 1.0e9;
	  timeout.tv_sec += 1;
	}
	if ((err = pthread_cond_timedwait(&camstr->cond, &camstr->m, &timeout))!=0){
	  camstr->err=1;
	  if (err == ETIMEDOUT) {
	    printf("camWaitPxls jaicam: Timeout waiting for pixels frame %d\n",camstr->bufframeno[camstr->tail]);
	  } else {
	    printf("Error waiting for pixels: %d %s\n",err,strerror(err));
	    printf("timeout = %ld %ld\n", timeout.tv_sec, timeout.tv_nsec);
	    return 1;
	    //perror("Error waiting for pixels");
	  }
	}
	
      }    
      if (camstr->err == 0) {
	memcpy(camstr->imgdata,camstr->ringBuf[camstr->tail],camstr->npxls*sizeof(short));
	camstr->userFrameNo[0]=camstr->bufframeno[camstr->tail];
	camstr->tail++;  
	camstr->tail &= BUFMASK;  
	if (camstr->tail == camstr->head) { 
	  // ring underflow
	  /*  printf("Ring buffer underflow!\n"); */
	  camstr->dataReady = 0;
	}else{
	  printf("Lagging %d %d %d\n",camstr->tail,camstr->head,camstr->userFrameNo[0]);
	}
      }
      
      camstr->reterr=camstr->err;
      camstr->transferRequired=0;
      pthread_cond_broadcast(&camstr->cond2);//wake up anyone waiting for the transfer
      pthread_mutex_unlock(&camstr->m);
    }else if(camstr->transferRequired==2){//someone is already doing the transfer, so we must wait for them...
      pthread_cond_wait(&camstr->cond2,&camstr->m);
      pthread_mutex_unlock(&camstr->m);
    }else{//transfer no longer required...
      pthread_mutex_unlock(&camstr->m);
    }
  }
  return camstr->reterr;
}

