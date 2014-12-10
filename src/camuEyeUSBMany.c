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
   The code here is used to create a shared object library, which can then be
   swapped around depending on which cameras you have in use, ie you simple
   rename the camera file you want to camera.so (or better, change the soft
   link), and restart the coremain.

   The library is written for a specific camera configuration - ie in multiple
   camera situations, the library is written to handle multiple cameras, not a
   single camera many times.
*/

/* IDS uEye USB camera library
 * Based on camuEyeUSB.c
 * Multiple camera capability (upto 1000)
 * Written for & tested with SDK version 4.30
 * Not compatible with SDK version 4.01 or below
 * Only tested with EO-0312M
 *       http://www.edmundoptics.com/imaging/cameras/usb-cameras/eo-usb-2-0-ccd-machine-vision-cameras/62837
 *    which is the same as
 *       http://en.ids-imaging.com/store/produkte/kameras/ui-2210se.html
 * with two cameras.
 *
 *
 * NAB
 * 2014/11/11
 */



/* the following ought to be in a header file */
#define     UEYE_WAITFORNEMTRANSFERS    10
#define     UEYE_NBUFFERS                8
#define     UEYE_ARGUMENTS               8
#define     SAFE_CALLOC(a,b) {if((a)==NULL){printf("camuEye::calloc error\n");b;}}
#define     SAFE_FREE(a) if((a)!=NULL){free(a);}

#ifndef __LINUX__
   #define __LINUX__ 1
   #warning Defining the __LINUX__ pre-processor macro
#endif



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include "rtccamera.h"
#include "uEye.h"
#ifdef UEYE_VERSION_CODE
   #if UEYE_VERSION_CODE<UEYE_VERSION(4,0,1)
      #error "camuEyeManyUSB.c:: not compatible with this version of the SDK"
   #endif
#endif
#include "darc.h"
#include "buffer.h"
#include "qsort.h"




typedef enum{
  UEYE_ARG_HID,
  UEYE_ARG_XPOS,
  UEYE_ARG_YPOS,
  UEYE_ARG_WIDTH,
  UEYE_ARG_HEIGHT,
  UEYE_ARG_FRAMERATE,
  UEYE_ARG_BINNING,
  /* Add more before this line. */
  CAMNARGUMENTS /* equal to number of entries in the enum */
}CAMARGUMENTINDX; /* should be in a header file */



typedef enum{
  UEYEACTUALEXP,
  UEYEBLACKLEVEL,
  UEYEBOOSTGAIN,
  UEYEEXPTIME,
  UEYEFRAMERATE,
  UEYEGAIN,
  UEYEGRABMODE,
  UEYENFRAMES,
  UEYEPIXELCLOCK,
  /* Add more before this line. */
  CAMNBUFFERVARIABLES /* equal to number of entries in the enum */
}CAMBUFFERVARIABLEINDX; /* should be in a header file */

char *camParamNameStems[] = {
/*  bufferMakeNames(CAMNBUFFERVARIABLES*camstr->ncam, "uEyeActualExp",
 *  "uEyeBlackLevel", "uEyeBoostGain", "uEyeExpTime", "uEyeFrameRate",
 *  "uEyeGain", "uEyeGrabMode", "uEyeNFrames", "uEyePixelClock")
 * Ensure the following are in alphabetical order ('<'>!sort in vi)
 * keep names short enough to append a 3 digit code plus (??) zero termination
 * 1234567890123456
 */
  "uEyeActExp", /* not used */
  "uEyeBlackLvl",
  "uEyeBoostG",
  "uEyeExpTime",
  "uEyeFrameRte",
  "uEyeGain",
  "uEyeGrabMode",
  "uEyeNFrames",
  "uEyePxlClock",
};


typedef struct{
  float *imgdata;            /* pointer to where data is stored */
  int npxls;
  int *pxlx;                 /* per camera number of pixels [ncam] */
  int *pxly;                 /* per camera number of pixels [ncam] */
  int *npxlsArr;             /* per camera number of pixels [ncam] */
  unsigned int *userFrameNo;
  int ncam;
  /* parameter section */
  char *paramNames;
  int *index;                /* indices of parameters [CAMNBUFFERVARIABLES] */
  void **values;             /* values [CAMNBUFFERVARIABLES] */
  char *dtype;               /* indices of parameters [CAMNBUFFERVARIABLES] */
  int *nbytes;               /* indices of parameters [CAMNBUFFERVARIABLES] */
  int *captureStarted;
  int *camOpen;              /* camera open via camOpen? [ncam] */
  /* per camera section */
  HIDS *hCam;
  char **imageMem;           /* image memory ptrs [ncam][CAMNBUFFERVARIABLES] */
  INT *imageMemPID;          /* image memory PIDs [ncam][CAMNBUFFERVARIABLES] */
  circBuf *rtcErrorBuf;
  float *frameRate;          /* [ncam] */
  double *expTime;           /* [ncam] */
  unsigned char **lastImgMem;/* (internal) checking transfer completed [ncam] */
  int *nFrames;              /* how many to bin [ncam] */
  int *grabMode;             /* continuous or one-by-one [ncam] */
  int *gain;                 /* 0--100 [ncam] */
  int *pxlClock;             /* [ncam] */
  int *black;                /* black-level adjusment [ncam] */
  int *binning;              /* integer, see uEye.h [ncam] */
}CamStruct;


/* Safely free all allocated memory and release camera
 */
void camdoFree(CamStruct *camstr){
  int i,j;
  if(camstr!=NULL){
      for(i=0;i<camstr->ncam;camstr++) {
          if(camstr->captureStarted[i]){
              is_StopLiveVideo(camstr->hCam[i],IS_WAIT);
              is_ClearSequence(camstr->hCam[i]);
          }
          for(j=0;j<UEYE_NBUFFERS;j++){
              if((char *)( camstr->imageMem[i*UEYE_NBUFFERS+j] )!=NULL)
                  is_FreeImageMem(camstr->hCam[i],
                          camstr->imageMem[i*UEYE_NBUFFERS+j],
                          camstr->imageMemPID[i*UEYE_NBUFFERS+j]);
          }
          if(camstr->camOpen[i]){
              is_DisableEvent(camstr->hCam[i],IS_SET_EVENT_FRAME);
              is_ExitEvent(camstr->hCam[i],IS_SET_EVENT_FRAME);
              is_ExitCamera(camstr->hCam[i]);
          }
      }
      SAFE_FREE( camstr->paramNames )
      SAFE_FREE( camstr->imageMem )
      SAFE_FREE( camstr->hCam )
      SAFE_FREE( camstr->frameRate )
      SAFE_FREE( camstr->expTime )
      SAFE_FREE( camstr->captureStarted )
      SAFE_FREE( camstr->imageMemPID )
      SAFE_FREE( camstr->camOpen )
      SAFE_FREE( camstr->lastImgMem )
      SAFE_FREE( camstr->pxlx )
      SAFE_FREE( camstr->pxly )
      SAFE_FREE( camstr->npxlsArr )
      SAFE_FREE( camstr->index )
      SAFE_FREE( camstr->values )
      SAFE_FREE( camstr->dtype )
      SAFE_FREE( camstr->nbytes )
      SAFE_FREE( camstr->nFrames)
      SAFE_FREE( camstr->grabMode )
      SAFE_FREE( camstr->gain )
      SAFE_FREE( camstr->pxlClock )
      SAFE_FREE( camstr->black )
      SAFE_FREE( camstr->binning )
      free(camstr);
  }
}



/** Set the camera parameters
 *  Note, not the arguments (to some extent, these represent duplicate
 *  functionality.
 *  The logic is a bit crude but based on the old version.
 *  Note that this section is mainly where changes to the SDK were
 *  implemented to make it *incompatible* with version prior to 4.01.
 */
int camNewParam(void *camHandle, paramBuf *pbuf, unsigned int frameno,
      arrayStruct *arr) {
  int i,j,ncam;
  int err=0;
  int prevGrabMode;
  INT nRet;
  INT nMode; /* for blacklevel */
  CamStruct *camstr=(CamStruct*)camHandle;
  double actualFrameRate;
  double actualExpTime;
  /*float *actualExp=NULL; REDUNDANT? */
  
  nRet=bufferGetIndex(pbuf, camstr->ncam*CAMNBUFFERVARIABLES,
        camstr->paramNames, camstr->index, camstr->values, camstr->dtype,
        camstr->nbytes);
  /* order of parameters is:: for each CAMNBUFFERVARIABLES parameter, there are
   * camstr->ncam values, one per camera so CAMNBUFFERVARIABLES*camstr->ncam in
   * total, ordered by the parameter
   */
  ncam=camstr->ncam;
  for(i=0; i<camstr->ncam; i++){
      /* first, loop through every camera, within which go through all
       * parameters, which steps in ncam points through allocated memory a ncam
       * times, each interleaved wrt the others
       */
      j=UEYEPIXELCLOCK;
      if(camstr->index[i+j*ncam]>=0){
        if(camstr->dtype[i+j*ncam]=='i' &&
              camstr->nbytes[i+j*ncam]==sizeof(int)){
            /* in MHz: recommends that frame rate and exposure time are also
             * set
             * after this.
             */
            camstr->pxlClock[i]=*(int*)camstr->values[i+j*ncam];
          /* NB The old (?) SDK may have specified SetPixelClock
           */
          if((nRet=is_PixelClock(camstr->hCam[i],IS_PIXELCLOCK_CMD_SET,
                        (void *)&camstr->pxlClock[i],
                        sizeof(UINT)))!=IS_SUCCESS)
            printf("\ncamuEye:(%d):[camNewParam]=%d:is_PixelClock failed for "
                   "%d\n", i,nRet, camstr->pxlClock[i] );
        }else{
          printf("\ncamuEye:(%d):[camNewParam]:uEyePixelClock error(%d,%d)\n",
                  i, camstr->dtype[i+j*ncam], camstr->nbytes[i+j*ncam]);
          writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"uEyePixelClock error");
          err=1;
        }
      }else{
        printf("\ncamuEye:(%d):[camNewParam]:uEyePixelClock not found - "
              "ignoring\n",i);
      }
/* REDUNDANT?
      j=UEYEACTUALEXP;
      if(camstr->index[i+j*ncam]>=0){
        if(camstr->nbytes[i+j*ncam]==sizeof(float) &&
            camstr->dtype[i+j*ncam]=='f'){
          actualExp=(float*)camstr->values[i+j*ncam];
        }else{
          printf("camuEye:(%d):Not returning actual exposure time - wrong
                datatype\n",i);
          actualExp=NULL;
        }
      }else{
        actualExp=NULL;
        printf("camuEye:(%d):uEyeActualExp not found - ignoring\n",i);
      }
 */

      j=UEYEFRAMERATE;
      if(camstr->index[i+j*ncam]>=0){
        if(camstr->dtype[i+j*ncam]=='i' &&
              camstr->nbytes[i+j*ncam]==sizeof(int)){
           printf("camuEye:WARNING:(%d):[camNewParam]:uEyeFrameRte%03d was an "
                  "'i', bodging to a 'f'\n",i,i);
           camstr->dtype[i+j*ncam]='f';
           camstr->nbytes[i+j*ncam]=sizeof(float);
        }
        if(camstr->dtype[i+j*ncam]=='f' &&
              camstr->nbytes[i+j*ncam]==sizeof(float)){
          camstr->frameRate[i]=*((float*)camstr->values[i+j*ncam]);
          if( (nRet=is_SetFrameRate(camstr->hCam[i],
                    (double)camstr->frameRate[i],&actualFrameRate))
                != IS_SUCCESS )
             if( camstr->frameRate[i]==0 && actualFrameRate>0 ) {
                 /* it probably did work, just that frameRate==0 will not
                  * return IS_SUCCESS, sadly
                  */
                 printf("camuEye:(%d):[camNewParam]:Framerate was forced to "
                     "%g\n", i, actualFrameRate);
             } else 
               printf("camuEye:(%d):[camNewParam]=%x:is_SetFrameRate failed; "
                    "asked for %f and got %f.\n",
                    i, nRet, camstr->frameRate[i], actualFrameRate);
          else
            printf("camuEye:(%d):[camNewParam]:Framerate asked/set to "
                  "%f/%f\n", i, camstr->frameRate[i],actualFrameRate);
            /* QUERY:: should the camstr->frameRate[i] variable be assigned the
             * value actualFrameRate i.e. does it reflect the specfied
             * parameter or the actual value of the parameter post application
             * of the function? Probably the latter.
             */
        }else{
          printf("camuEye:(%d):[camNewParam]:uEyeFrameRate error\n",i);
          writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"uEyeFrameRate error");
          err=1;
        }
      }else{
        printf("camuEye:(%d):[camNewParam]:uEyeFrameRate not found - "
              "ignoring\n",i);
      }
      j=UEYEEXPTIME; /* ==0, -> exposure time is 1/frame rate. */
      if(camstr->index[i+j*ncam]>=0){
        if(camstr->dtype[i+j*ncam]=='i' &&
              camstr->nbytes[i+j*ncam]==sizeof(int)){
           printf("camuEye:WARNING:(%d):[camNewParam]:uEyeExpTime%03d was an "
                  "'i', bodging to a 'f'\n",i,i);
           camstr->dtype[i+j*ncam]='f';
           camstr->nbytes[i+j*ncam]=sizeof(float);
        }
        if(camstr->dtype[i+j*ncam]=='f' &&
              camstr->nbytes[i+j*ncam]==sizeof(float)){
          actualExpTime=*((float *)camstr->values[i+j*ncam]); /* overwritten */
          camstr->expTime[i]=actualExpTime;
          /* WAS is_SetExposureTime */
          /* file://./usr/share/doc/ids/ueye_manual/is_exposure.html */
          if( (nRet=is_Exposure(camstr->hCam[i], IS_EXPOSURE_CMD_SET_EXPOSURE,
                          &actualExpTime, 8)) != IS_SUCCESS )
              printf("camuEye:(%d):[camNewParam]=%x:is_SetExposureTime failed "
                    "asked for %f and got %f.\n",
                    i, nRet, camstr->expTime[i], actualExpTime );
          else{
            /* QUERY:: should the camstr->expTime[i] variable be assigned the
             * value actualExpTime i.e. does it reflect the specfied parameter
             * or the actual value of the parameter post application of the
             * function? Probably the latter.
             */
               printf("camuEye:(%d):[camNewParam]:Exposure asked/set to "
                     "%f/%f\n", i, camstr->expTime[i], actualExpTime);
          }
        }else{
          printf("camuEye:(%d):[camNewParam]:uEyeExpTime error\n",i);
          writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"uEyeExpTime error");
          err=1;
        }
      }else{
        printf("camuEye:(%d):[camNewParam]:uEyeExpTime not found - "
              "ignoring\n",i);
      }
      j=UEYENFRAMES;
      if(camstr->index[i+j*ncam]>=0){
        if(camstr->dtype[i+j*ncam]=='i' &&
              camstr->nbytes[i+j*ncam]==sizeof(int)){
          camstr->nFrames[i]=*(int*)camstr->values[i+j*ncam];
        }else{
          printf("camuEye:(%d):[camNewParam]:uEyeNFrames error\n",i);
          writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"uEyeNFrames error");
          err=1;
          camstr->nFrames[i]=1;
        }
      }else{
        printf("camuEye:(%d):[camNewParam]:uEyeNFrames not found - "
              "ignoring\n",i);
        camstr->nFrames[i]=1;
      }
      j=UEYEBOOSTGAIN;
      if(camstr->index[i+j*ncam]>=0){
        if(camstr->dtype[i+j*ncam]=='i' &&
              camstr->nbytes[i+j*ncam]==sizeof(int)) {
          if( *( (int*) camstr->values[i+j*ncam] ) ){
            if( is_SetGainBoost(camstr->hCam[i], IS_SET_GAINBOOST_ON) !=
                  IS_SUCCESS)
               printf("camuEye:(%d):[camNewParam]:SetGainBoost (ON) failed - "
                    "maybe not available\n",i);
          }else{
            if( is_SetGainBoost(camstr->hCam[i],IS_SET_GAINBOOST_OFF) !=
                  IS_SUCCESS)
               printf("camuEye:(%d):[camNewParam]:SetGainBoost (OFF) failed - "
                    "maybe not available\n",i);
          }
        }else{
          printf("camuEye:(%d):[camNewParam]:uEyeBoostGain error\n",i);
          writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"uEyeBoostGain error");
          err=1;
        }
      }else{
        printf("camuEye:(%d):[camNewParam]:uEyeBoostGain not found - "
              "ignoring\n",i); 
      }
      j=UEYEGAIN;
      if(camstr->index[i+j*ncam]>=0){
        if(camstr->dtype[i+j*ncam]=='i' &&
              camstr->nbytes[i+j*ncam]==sizeof(int)){
          camstr->gain[i]=*(int*)camstr->values[i+j*ncam];
          if(camstr->gain[i]>100){
            camstr->gain[i]=100;
            printf("camuEye:(%d):[camNewParam]:Clipping gain to <=100\n",i);
          }else if(camstr->gain<0){
            camstr->gain[i]=0;
            printf("camuEye:(%d):[camNewParam]:Clipping gain to >=0\n",i);
          }
          is_SetHardwareGain(camstr->hCam[i],
                  camstr->gain[i],
                  IS_IGNORE_PARAMETER,IS_IGNORE_PARAMETER,IS_IGNORE_PARAMETER);
          printf("camuEye:(%d):[camNewParam]:Setting gain to "
                "%d\n",i,camstr->gain[i]);
        }else{
          printf("camuEye:(%d):[camNewParam]:uEyeGain error\n",i);
          writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"uEyeGain error");
          err=1;
        }
      }else{
        printf("camuEye:(%d):[camNewParam]:uEyeGain not found - ignoring\n",i);
      }
      j=UEYEBLACKLEVEL;
      if(camstr->index[i+j*ncam]>=0){
        if(camstr->dtype[i+j*ncam]=='i' &&
              camstr->nbytes[i+j*ncam]==sizeof(int)) {
          camstr->black[i]=*(int*)camstr->values[i+j*ncam];
/*
 * OLD API: is_SetBlCompensation(camstr->hCam[i],IS_BL_COMPENSATION_DISABLE,abs(camstr->black[i]),0)!=IS_SUCCESS
 */
          if( (nRet=is_Blacklevel( camstr->hCam[i],
              camstr->black[i]<0?IS_AUTO_BLACKLEVEL_OFF:IS_AUTO_BLACKLEVEL_ON,
              (void *)&nMode, sizeof(nMode) )) != IS_SUCCESS )
                printf("camuEye:(%d):[camNewParam]:SetBlCompensation (OFF) "
                      "failedi=%d\n",i,nRet);
        } else {
          printf("camuEye:(%d):[camNewParam]:is_Blacklevel error\n",i);
          writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"uEyeBlackLevel error");
          err=1;
        }
      }else{
        printf("camuEye:(%d):[camNewParam]:uEyeBlackLevel not found - "
              "ignoring\n",i);
      }

      prevGrabMode=camstr->grabMode[i];
      j=UEYEGRABMODE;
      if(camstr->index[i+j*ncam]>=0){
        if(camstr->dtype[i+j*ncam]=='i' &&
              camstr->nbytes[i+j*ncam]==sizeof(int)){
          camstr->grabMode[i]=*(int*)camstr->values[i+j*ncam];
        }else{
          printf("camuEye:(%d):[camNewParam]:uEyeGrabMode error\n",i);
          writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"uEyeGrabMode error");
          err=1;
          camstr->grabMode[i]=0;
        }
      }else{
        printf("camuEye:(%d):[camNewParam]:uEyeGrabMode not found - "
              "ignoring\n",i);
        camstr->grabMode[i]=0;
      }
      if(camstr->grabMode[i]!=prevGrabMode){
        /* change the operation mode */
        if(camstr->grabMode[i]){
          if(camstr->captureStarted[i]){
            camstr->captureStarted[i]=0;
            is_StopLiveVideo(camstr->hCam[i],IS_WAIT);
          }
        }else{
          /* turn on framing. */
          is_CaptureVideo(camstr->hCam[i],IS_WAIT);/* set to live mode */
          camstr->captureStarted[i]=1;

        }
      }
  }
  return err;
}


/* camMakeNames:: allocate memory and create parameter names
 * Given a CamStruct point which has only the ncam value instantiated, and the
 * relevant enum and static char ** array defined, produce the camera parameter
 * names. Possibly a bit overcooked but this was based on camAravis.c
 *
 * Returns NULL on failure, otherwise the address of the calloc'd parameter
 * name array.
 */
char* camMakeNames(CamStruct *camHandle) {
  int i,j,ncam;/*,nparams;*/
  CamStruct *camstr=(CamStruct*)camHandle;
  ncam=camstr->ncam;
/*  nparams=ncam*CAMNBUFFERVARIABLES;*/
  char *camParamName;
   
  SAFE_CALLOC(
        camParamName=calloc(ncam*CAMNBUFFERVARIABLES*BUFNAMESIZE, sizeof(char)),
          {return(NULL);} )
  for(j=0;j<CAMNBUFFERVARIABLES;j++)
    for(i=0;i<ncam;i++) {
        snprintf( camParamName+(i+j*ncam)*BUFNAMESIZE,
                BUFNAMESIZE,"%s%03d",camParamNameStems[j],i );
    }
/*#define islt(a,b) (strcmp((*a),(*b))<0)
  QSORT(char*,camParamName,nparams,islt);
      *sort, but unecessary?
       thus commented out *
#undef islt*/
  return( camParamName );
}


/**
   Open a camera of type name.  Args are passed in a int32 array of size n,
   which can be cast if necessary.  Any state data is returned in camHandle,
   which should be NULL if an error arises.  pxlbuf is the array that should
   hold the data. The library is free to use the user provided version, or use
   its own version as necessary (ie a pointer to physical memory or whatever).
   It is of size npxls*sizeof(short).
   ncam is number of cameras, which is the length of arrays pxlx and pxly,
   which contain the dimensions for each camera.
   Name is used if a library can support more than one camera.
*/
int camOpen(char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char
        *prefix,arrayStruct *arr,void **camHandle,int nthreads,unsigned int
        thisiter,unsigned int **frameno,int *camframenoSize,int npxls,int
        ncam,int *pxlx,int* pxly){ CamStruct *camstr;
  int i,j;
  int binning;
  unsigned short *tmps;
  double actualFrameRate, expTime;
  char *errtxt;
  INT xpos,ypos,width,height,bitsPerPxl=8,nRet,nDetectedCams,nAvailableCams;
  INT cerr;
  UEYE_CAMERA_LIST *pucl; /* file://./ueye_manual/is_getcameralist.html */
  IS_RECT aoiRect;
  SENSORINFO camInfo;
  
  /* n = The number of ARGUMENTS (not parameters)(?) */
  if((n/ncam)!=CAMNARGUMENTS || n%CAMNARGUMENTS ) {
    printf("camuEye:ERROR: require %d arguments per "
           "camera\n",CAMNARGUMENTS); *camHandle=NULL;
    return 1;
  }
  
  is_GetNumberOfCameras( &nDetectedCams );
  if( (int) nDetectedCams!=ncam )
      /* file://./ueye_manual/is_getnumberofcameras.html */
      printf("camuEye:INFO:mismatch in number of detected:ncam "
             "(%d:%d)\n",(int)nDetectedCams,ncam);
  
  /* file://./ueye_manual/is_getcameralist.html */
  pucl = (UEYE_CAMERA_LIST*) malloc(
          sizeof(DWORD)+nDetectedCams*sizeof(UEYE_CAMERA_INFO) );
  pucl->dwCount=(ULONG) nDetectedCams;
  if( (nRet=is_GetCameraList(pucl)) != IS_SUCCESS) {
        printf("camuEye:ERROR=%d:failed to query cameras\n",nRet);
        return(1);
  }
  nAvailableCams=0;
  for(i=0; i<(int)pucl->dwCount; i++) {
        nAvailableCams+=pucl->uci[i].dwInUse==0?1:0;
        printf("camuEye:INFO:[%d|%d)CameraID=%d\n"
               "                  DeviceID=%d\n"
               "                     SerNo=%s\n"
               "                     Model=%s\n",i,nAvailableCams,
                (int)pucl->uci[i].dwCameraID,
                (int)pucl->uci[i].dwDeviceID,
                (char *)pucl->uci[i].SerNo,
                (char *)pucl->uci[i].Model);
  }
  if( nAvailableCams<1 ) {
        printf("camuEye:ERROR:could not find (any available) cameras\n");
        return(1);
  } else if( nAvailableCams!=ncam ) {
        printf("camuEye:ERROR:nAvailableCams!=ncam\n");
        return(1);
  } 
  free(pucl);

  if(arr->pxlbuftype!='f' || arr->pxlbufsSize!=sizeof(float)*npxls){
    //need to resize the pxlbufs...
    arr->pxlbufsSize=sizeof(float)*npxls;
    arr->pxlbuftype='f';
    arr->pxlbufelsize=sizeof(float);
    tmps=realloc(arr->pxlbufs,arr->pxlbufsSize);
    if(tmps==NULL){
      if(arr->pxlbufs!=NULL)
        free(arr->pxlbufs);
      printf("camuEye:ERROR:pxlbuf malloc error\n");
      arr->pxlbufsSize=0;
      free(*camHandle);
      *camHandle=NULL;
      return 1;
    }
    arr->pxlbufs=tmps;
    memset(arr->pxlbufs,0,arr->pxlbufsSize);
  }

  printf("camuEye:INFO:Initialising camera %s\n",name);
  if((*camHandle=malloc(sizeof(CamStruct)))==NULL){
    printf("camuEye:ERROR:(1)Couldn't malloc camera handle\n");
    return 1;
  }
  memset(*camHandle,0,sizeof(CamStruct));
  camstr=(CamStruct*)*camHandle;
  camstr->ncam=ncam;
  camstr->rtcErrorBuf=rtcErrorBuf;
  camstr->npxls=npxls;//*pxlx * *pxly;
  camstr->imgdata=arr->pxlbufs;
  SAFE_CALLOC( camstr->imageMem=
                (char **)calloc( ncam*UEYE_NBUFFERS, sizeof(char *) ),
          {camdoFree(camstr);*camHandle=NULL;return 1;} )
  SAFE_CALLOC( camstr->imageMemPID=
                (INT *)calloc( ncam*UEYE_NBUFFERS, sizeof(INT) ),
          {camdoFree(camstr);*camHandle=NULL;return 1;} )
  SAFE_CALLOC( camstr->hCam=(HIDS *)calloc( ncam, sizeof(HIDS) ),
          {camdoFree(camstr);*camHandle=NULL;return 1;} )
  SAFE_CALLOC( camstr->frameRate=(float *)calloc( ncam, sizeof(float) ),
          {camdoFree(camstr);*camHandle=NULL;return 1;} )
  SAFE_CALLOC( camstr->expTime=(double *)calloc( ncam, sizeof(double) ),
          {camdoFree(camstr);*camHandle=NULL;return 1;} )
  SAFE_CALLOC( camstr->captureStarted=(int *)calloc( ncam, sizeof(float) ),
          {camdoFree(camstr);*camHandle=NULL;return 1;} )
  SAFE_CALLOC( camstr->camOpen=(int *)calloc( ncam, sizeof(int) ),
          {camdoFree(camstr);*camHandle=NULL;return 1;} )
  SAFE_CALLOC( camstr->lastImgMem=
                (unsigned char **)calloc( ncam, sizeof(char *)),
          {camdoFree(camstr);*camHandle=NULL;return 1;} )
  SAFE_CALLOC( camstr->pxlx=(int *)calloc( ncam, sizeof(int)),
          {camdoFree(camstr);*camHandle=NULL;return 1;} )
  SAFE_CALLOC( camstr->pxly=(int *)calloc( ncam, sizeof(int)),
          {camdoFree(camstr);*camHandle=NULL;return 1;} )
  SAFE_CALLOC( camstr->npxlsArr=(int *)calloc( ncam, sizeof(int)),
          {camdoFree(camstr);*camHandle=NULL;return 1;} )
  SAFE_CALLOC( camstr->index=
                (int *)calloc( CAMNBUFFERVARIABLES*ncam, sizeof(int)),
          {camdoFree(camstr);*camHandle=NULL;return 1;} )
/* the following is an educated guess!
 */
  SAFE_CALLOC( camstr->values=
                (void **)calloc( CAMNBUFFERVARIABLES*ncam, sizeof(float*)),
          {camdoFree(camstr);*camHandle=NULL;return 1;} )
  SAFE_CALLOC( camstr->dtype=(char *)calloc( CAMNBUFFERVARIABLES*ncam, sizeof(char)),
          {camdoFree(camstr);*camHandle=NULL;return 1;} )
  SAFE_CALLOC( camstr->nbytes=(int *)calloc( CAMNBUFFERVARIABLES*ncam, sizeof(int)),
          {camdoFree(camstr);*camHandle=NULL;return 1;} )
  SAFE_CALLOC( camstr->nFrames=(int *)calloc( ncam, sizeof(int)),
          {camdoFree(camstr);*camHandle=NULL;return 1;} )
  SAFE_CALLOC( camstr->grabMode=(int *)calloc( ncam, sizeof(int)),
          {camdoFree(camstr);*camHandle=NULL;return 1;} )
  SAFE_CALLOC( camstr->gain=(int *)calloc( ncam, sizeof(int)),
          {camdoFree(camstr);*camHandle=NULL;return 1;} )
  SAFE_CALLOC( camstr->pxlClock=(int *)calloc( ncam, sizeof(int)),
          {camdoFree(camstr);*camHandle=NULL;return 1;} )
  SAFE_CALLOC( camstr->black=(int *)calloc( ncam, sizeof(int)),
          {camdoFree(camstr);*camHandle=NULL;return 1;} )
  SAFE_CALLOC( camstr->binning=(int *)calloc( ncam, sizeof(int)),
          {camdoFree(camstr);*camHandle=NULL;return 1;} )
  SAFE_CALLOC( camstr->paramNames=(char *)camMakeNames( camstr ),
          {camdoFree(camstr);*camHandle=NULL;return 1;} )

  if(*camframenoSize<ncam){
    if(*frameno!=NULL)free(*frameno);
        *frameno=malloc(sizeof(int)*ncam);
    if(*frameno==NULL){
        *camframenoSize=0;
        printf("camuEye:WARNING:Unable to malloc camframeno\n");
    }else{
        *camframenoSize=ncam;
    }
  }
  camstr->userFrameNo=*frameno;

  for(i=0; i<ncam; i++) {
        camstr->userFrameNo[i]=-1;
        camstr->hCam[i]=(HIDS) args[UEYE_ARG_HID*ncam+i];
        xpos=args[UEYE_ARG_XPOS*ncam+i];
        ypos=args[UEYE_ARG_YPOS*ncam+i];
        width=args[UEYE_ARG_WIDTH*ncam+i];
        height=args[UEYE_ARG_HEIGHT*ncam+i];
        camstr->frameRate[i]=(double)(args[UEYE_ARG_FRAMERATE*ncam+i]);
        binning=args[UEYE_ARG_BINNING*ncam+i];

        camstr->pxlx[i]=pxlx[i];
        camstr->pxly[i]=pxly[i];
        camstr->npxlsArr[i]=pxly[i]*pxlx[i];


        if((nRet=is_InitCamera(&camstr->hCam[i],NULL))!=IS_SUCCESS){
            printf("camuEye:ERROR=%d: Failed to open camera(%d)\n",nRet,i);
            is_GetError(camstr->hCam[i],&cerr,&errtxt);
            printf("   Error %d was %s\n",cerr,errtxt);
            if(nRet==IS_STARTER_FW_UPLOAD_NEEDED){
                INT nTime;
                is_GetDuration(camstr->hCam[i],IS_SE_STARTER_FW_UPLOAD,&nTime);
                /* printf("Uploading firmware - will take %ds\n",nTime); */
                printf("   NOT uploading firmware due to: it doesn't compile\n");
                /* nRet=is_InitCamera((&camstr->hCam[i])|IS_ALLOW_STARTER_FW_UPLOAD,NULL); */
            }/*else{*/
                camdoFree(camstr);
                *camHandle=NULL;
                return 1;
            /*}*/
        } else 
            camstr->camOpen[i]=1;
        if((nRet=is_GetSensorInfo(camstr->hCam[i],&camInfo))!=IS_SUCCESS)
            printf("camuEye:WARNING:failed to get camera(%d) info\n",i);
        printf("camuEye:INFO:Opened camera(%d) '%32s', size %dx%d\n",i,
                camInfo.strSensorName,camInfo.nMaxWidth,camInfo.nMaxHeight);
        if((nRet=is_SetColorMode(camstr->hCam[i],IS_CM_MONO8))!=IS_SUCCESS)
            printf("camuEye:WARNING:(%d)setColorMode failed\n",i);
        if((nRet=is_HotPixel(camstr->hCam[i],
                        IS_HOTPIXEL_DISABLE_CORRECTION,NULL,0))!=IS_SUCCESS)
            printf("camuEye:WARNING:(%d)is_HotPixel(disable) failed\n",i);

        if(binning!=1){
            int mode;
            switch(binning){
            case 2:
              mode=IS_BINNING_2X_VERTICAL;
              break;
            case 3:
              mode=IS_BINNING_3X_VERTICAL;
              break;
            case 4:
              mode=IS_BINNING_4X_VERTICAL;
              break;
            default:
              mode=IS_BINNING_DISABLE;
              break;
            }
            if(is_SetBinning(camstr->hCam[i],mode)!=IS_SUCCESS)
              printf("camuEye:WARNING:(%d)is_SetBinning failed\n",i);
        }

        aoiRect.s32X=xpos;
        aoiRect.s32Y=ypos;
        aoiRect.s32Width=width;
        aoiRect.s32Height=height;
        if((nRet=
/*
 * OLD API: is_SetAOI(camstr->hCam[i],IS_SET_IMAGE_AOI,&xpos,&ypos,&width,&height))!=IS_SUCCESS)
 */
               is_AOI(camstr->hCam[i], IS_AOI_IMAGE_SET_AOI, &aoiRect,
                     sizeof(aoiRect)))!=IS_SUCCESS)
                        printf("camuEye:WARNING:(%d)is_SetAOI failed\n",i);
        /* NAB: it isn't clear why this next function call is commented out */
/*  double expMax, REDUNDANT? 
        if((nRet=is_SetExposureTime(camstr->hCam[i],IS_SET_ENABLE_AUTO_SHUTTER,&expMax))!=IS_SUCCESS)
          printf("is_SetExposureTime failed\n");
*/
        if( (nRet=is_SetFrameRate(camstr->hCam[i],
                        (double)camstr->frameRate[i],
                        &actualFrameRate))!=IS_SUCCESS)
            printf("camuEye:WARNING:(%d)is_SetFrameRate failed=%d\n",i, nRet);
        expTime=(double) 0; 
/*
 * OLD API: (nRet=is_SetExposureTime(camstr->hCam[i],0.,&expMax))!=IS_SUCCESS)
 */
          /* file://./usr/share/doc/ids/ueye_manual/is_exposure.html */
        if( (nRet=is_Exposure( camstr->hCam[i], IS_EXPOSURE_CMD_SET_EXPOSURE,
                      &expTime, 8 )) != IS_SUCCESS )
            printf("camuEye:WARNING:(%d):is_SetExposureTime (0) failed=%d\n",
                  i,nRet);
        printf("camuEye:INFO:(%d) Exposure time set to %gms with frame rate %g"
               "Hz\n",i,expTime,actualFrameRate);
        if (camInfo.bGlobShutter == TRUE)
            is_SetGlobalShutter(camstr->hCam[i], IS_SET_GLOBAL_SHUTTER_ON);
        // Alloc some memory for image buffer
        for(j=0;j<UEYE_NBUFFERS;j++){
            if((nRet=is_AllocImageMem(camstr->hCam[i], (INT)width, (INT)height,
                            bitsPerPxl,
                            &(camstr->imageMem[i*UEYE_NBUFFERS+j]),
                            &(camstr->imageMemPID[i*UEYE_NBUFFERS+j])
                                     ))!=IS_SUCCESS) {
              printf("camuEye:ERROR=%d:Failed to allocate camera(%d/%d) memory "
                    "(%dx%d)@%d closing camera\n",
                     nRet,i,
                     camstr->hCam[i],width,height,bitsPerPxl); camdoFree(camstr);
              *camHandle=NULL;
              return 1;
            }
            if((nRet=is_AddToSequence(camstr->hCam[i],
                camstr->imageMem[i*UEYE_NBUFFERS+j],
                camstr->imageMemPID[i*UEYE_NBUFFERS+j]))!=IS_SUCCESS) {
              printf("camuEye:ERROR=%d:Failed to assign memory to camera(%d) "
                     "library memory sequence\n",nRet,i); camdoFree(camstr);
              *camHandle=NULL;
              return 1;
            }
        }
        /* IS_SET_TRIGGER_SOFTWARE as previously used - but is slower */
        if((nRet=is_SetExternalTrigger(camstr->hCam[i],
                        IS_SET_TRIGGER_OFF))!=IS_SUCCESS)
            printf("camuEye:WARNING:(%d)is_SetExternalTrigger failed\n",i);
        nRet=is_SetDisplayMode(camstr->hCam[i], IS_GET_DISPLAY_MODE);
        if(!(nRet & IS_SET_DM_DIB)){
            printf("camuEye:ERROR=%d:(%d)is_SetDisplayMode failed\n",nRet,i);
            camdoFree(camstr);
            *camHandle=NULL;
            return 1;
        }
        /* Now install handling... do we want an event or a message handler?
         * An event, because messages are windoze only.
         */
        /* Windows API only, so not sure why this was here originally.
         * if( (nRet=is_InitEvent(camstr->hCam[i],NULL,IS_SET_EVENT_FRAME)) !=
              IS_SUCCESS)
            printf("camuEye:WARNING:(%d)is_InitEvent failed\n",i);
         */
        if((nRet=is_EnableEvent(camstr->hCam[i],IS_SET_EVENT_FRAME)) != 
              IS_SUCCESS)
            printf("camuEye:WARNING:(%d)is_EnableEvent failed\n",i);
        is_CaptureVideo(camstr->hCam[i],IS_WAIT);//set to live mode...
        camstr->captureStarted[i]=1;

        if(camNewParam(*camHandle,pbuf,thisiter,arr)!=0){
            printf("camuEye:ERROR:Error in setting parameters for "
                   "camera(%d)\n",i);
            camdoFree(camstr);
            *camHandle=NULL;
            return 1;
        }
    }

/*
    if((nRet=is_WaitEvent(camstr->hCam[i],IS_SET_EVENT_FRAME,1000))!=IS_SUCCESS)
      printf("is_WaitEvent failed\n");
    is_WaitForNextImage(camstr->hCam[i],1000,&imgptr,&imgid);

    is_FreezeVideo(camstr->hCam[i],IS_DONT_WAIT);
*/
    return 0;
}


/**
   Close a camera of type name. 
   Args are passed in the int32 array of size n, and state data is in
   camHandle, which should be freed and set to NULL before returning.
*/
int camClose(void **camHandle){
  CamStruct *camstr;
  printf("camuEye::Closing camera\n");
  if(*camHandle==NULL)
    return 1;
  camstr=(CamStruct*)*camHandle;
  camdoFree(camstr);
  *camHandle=NULL;
  printf("camuEye::Camera closed\n");
  return 0;
}


/**
   Called when we're starting to process the next frame.
   Does all the image capture, averaging(on a *frame* basis, not pixel basis),
    for all cameras.
*/
int camNewFrameSync(void *camHandle,unsigned int thisiter,double starttime){
  //printf("camNewFrame\n");
   CamStruct *camstr;
   int i,j,k,n;
   unsigned char *imgMem=NULL;
   int imgdataPointerOffset=0;
  //INT pitch;
   camstr=(CamStruct*)camHandle;
   if(camHandle==NULL){// || camstr->streaming==0){
    //printf("called camNewFrame with camHandle==NULL\n");
      return 1;
   }
   memset(camstr->imgdata,0,sizeof(float)*camstr->npxls);
   imgdataPointerOffset=0; /* set to start of imgdata */
   for(i=0;i<camstr->ncam;i++) {
      if(camstr->nFrames[i]<1) camstr->nFrames[i]=1;
      if(camstr->grabMode[i]) is_FreezeVideo(camstr->hCam[i],IS_WAIT);
      for(n=0;n<camstr->nFrames[i];n++){ /* sum this many frames */
         is_GetImageMem(camstr->hCam[i],(void**)&imgMem);
         k=UEYE_WAITFORNEMTRANSFERS;
         while(k-->0 && imgMem==camstr->lastImgMem[i]){ /* wait for new data */
            if(camstr->grabMode[i]) is_FreezeVideo(camstr->hCam[i],IS_WAIT);
                 else is_WaitEvent(camstr->hCam[i],IS_SET_EVENT_FRAME,1000);
            is_GetImageMem(camstr->hCam[i],(void**)&imgMem);
         }
         if(imgMem==camstr->lastImgMem[i])
            printf("camuEye:WARNING:(%d)Duplicate image retrieved at "
                  "%p\n",i,imgMem); camstr->lastImgMem[i]=imgMem;
      }
      for(j=0;j<camstr->npxlsArr[i];j++){//byte to float conversion.
         *(camstr->imgdata+imgdataPointerOffset+j) += 
            ((float)*(imgMem+j))/(float)camstr->nFrames[i];
      }
      imgdataPointerOffset+=camstr->npxlsArr[i];
  }
  /* camstr->frameno++; REDUNDANT? */
  for(i=0; i<camstr->ncam; i++){
    camstr->userFrameNo[i]++;
  }
  return 0;
}
