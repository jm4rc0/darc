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
   The code here is used to create a shared object library, which can then be swapped around depending on which cameras you have in use, ie you simple rename the camera file you want to camera.so (or better, change the soft link), and restart the coremain.

The library is written for a specific camera configuration - ie in multiple camera situations, the library is written to handle multiple cameras, not a single camera many times.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rtccamera.h"
#include <time.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#define __LINUX__ 1
#include "uEye.h"


#define nBuffers 8
typedef struct{
  int frameno;
  unsigned short *imgdata;
  int npxls;
  unsigned int *userFrameNo;
  int ncam;
  int captureStarted;
  int camOpen;
  HIDS hCam;
  char *imageMem[nBuffers];
  INT pid[nBuffers];
  char *paramNames;
  float frameRate;
  circBuf *rtcErrorBuf;
}CamStruct;


typedef enum{
  UEYEFRAMERATE,
  //Add more before this line.
  CAMNBUFFERVARIABLES//equal to number of entries in the enum
}CAMBUFFERVARIABLEINDX;

#define camMakeNames() bufferMakeNames(CAMNBUFFERVARIABLES,"uEyeFrameRate")


void camdoFree(CamStruct *camstr){
  int hCam=camstr->hCam;
  int i;
  if(camstr!=NULL){
    if(camstr->paramNames!=NULL)
      free(camstr->paramNames);
    if(camstr->captureStarted){
      is_StopLiveVideo(hCam,IS_WAIT);
      is_ClearSequence(hCam);
    }
    for(i=0;i<nBuffers;i++){
      if(camstr->imageMem[i]!=NULL)
	is_FreeImageMem(hCam,camstr->imageMem[i],camstr->pid[i]);
    }
    if(camstr->camOpen){
      is_DisableEvent(hCam,IS_SET_EVENT_FRAME);
      is_ExitEvent(hCam,IS_SET_EVENT_FRAME);
      is_ExitCamera(hCam);
    }
    free(camstr);
  }
}


int camNewParam(void *camHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr){
  //the only param needed is camReorder if reorder!=0.
  int i,j;
  CamStruct *camstr=(CamStruct*)camHandle;
  int nfound,err=0;
  nfound=bufferGetIndex(pbuf,CAMNBUFFERVARIABLES,camstr->paramNames,camstr->index,camstr->values,camstr->dtype,camstr->nbytes);
  i=UEYEFRAMERATE;
  if(camstr->index[i]>=0){//has been found...
    if(dtype[i]=='f' && nbytes[i]==4){
      camstr->frameRate=*((float*)values[i]);
    }else{
      printf("uEyeFrameRate error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"uEyeFrameRate error");
      err=1;
    }
  }else{
    printf("uEyeFrameRate not found - ignoring\n");
  }
  return err;
}


/**
   Open a camera of type name.  Args are passed in a int32 array of size n, which can be cast if necessary.  Any state data is returned in camHandle, which should be NULL if an error arises.
   pxlbuf is the array that should hold the data. The library is free to use the user provided version, or use its own version as necessary (ie a pointer to physical memory or whatever).  It is of size npxls*sizeof(short).
   ncam is number of cameras, which is the length of arrays pxlx and pxly, which contain the dimensions for each camera.
   Name is used if a library can support more than one camera.

   This library opens a FITS file (assumes 2880 header size), and uses this for the data.
   args here contains filename
*/

int camOpen(char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **camHandle,int nthreads,unsigned int thisiter,unsigned int **frameno,int *framenoSize,int npxls,int ncam,int *pxlx,int* pxly){
  CamStruct *camstr;
  int naxis=0;
  char buf[80];
  int end;
  int axis;
  int i,framePixels;
  unsigned short *tmps;
  HIDS hCam=0;
  SENSORINFO camInfo;
  INT xpos,ypos,width,height,bitsPerPxl=8,nRet;
  double expMax,actualFrameRate;
  //unsigned short *pxlbuf=arr->pxlbufs;
  printf("Initialising camera %s\n",name);
  if((*camHandle=malloc(sizeof(CamStruct)))==NULL){
    printf("Couldn't malloc camera handle\n");
    return 1;
  }
  memset(*camHandle,0,sizeof(CamStruct));
  camstr=(CamStruct*)*camHandle;
  camstr->paramNames=reconMakeNames();
  camstr->rtcErrorBuf=rtcErrorBuf
  if(n==5){
    xpos=args[0];
    ypos=args[1];
    width=args[2];
    height=args[3];
    camstr->frameRate=(double)(args[4]);
  }else{
    printf("Error - args should be 5 in camuEyeUSB: xpos,ypos,width,height,frameRate\n");
    camdoFree(camstr);
    *camHandle=NULL;
    return 1;
  }


  if(arr->pxlbuftype!='c' || arr->pxlbufsSize!=sizeof(char)*npxls){
    //need to resize the pxlbufs...
    arr->pxlbufsSize=sizeof(char)*npxls;
    arr->pxlbuftype='c';
    arr->pxlbufelsize=sizeof(char);
    tmps=realloc(arr->pxlbufs,arr->pxlbufsSize);
    if(tmps==NULL){
      if(arr->pxlbufs!=NULL)
	free(arr->pxlbufs);
      printf("pxlbuf malloc error in camfile.\n");
      arr->pxlbufsSize=0;
      free(*camHandle);
      *camHandle=NULL;
      return 1;
    }
    arr->pxlbufs=tmps;
    memset(arr->pxlbufs,0,arr->pxlbufsSize);
  }
  if(ncam!=1){
    printf("Sorry - only 1 camera currently allowed\n");
    free(*camHandle);
    *camHandle=NULL;
    return 1;
  }
  camstr->imgdata=arr->pxlbufs;
  if(*framenoSize<ncam){
    if(*frameno!=NULL)free(*frameno);
    *frameno=malloc(sizeof(int)*ncam);
    if(*frameno==NULL){
      *framenoSize=0;
      printf("Unable to malloc camframeno\n");
    }else{
      *framenoSize=ncam;
    }
  }
  camstr->userFrameNo=*frameno;
  camstr->npxls=npxls;//*pxlx * *pxly;
  camstr->ncam=ncam;
  camstr->frameno=-1;
  for(i=0; i<ncam; i++){
    camstr->userFrameNo[i]=-1;
  }

  if((nRet=is_InitCamera(&hCam,NULL))!=IS_SUCCESS){
    printf("Failed to open camera: %d\n",nRet);
    if(nRet==IS_STARTER_FW_UPLOAD_NEEDED){
      INT nTime;
      is_GetDuration(hCam,IS_SE_STARTER_FW_UPLOAD,&nTime);
      printf("Uploading firmware - will take %ds\n",nTime);
      printf("Actually - not doing this because it doesn't compile\n");
      //nRet=is_InitCamera((&hCam)|IS_ALLOW_STARTER_FW_UPLOAD,NULL);
    }else{
      camdoFree(camstr);
      *camHandle=NULL;
      return 1;
    }
  }
  camstr->hCam=hCam;
  camstr->camOpen=1;
  if((nRet=is_GetSensorInfo(hCam,&camInfo))!=IS_SUCCESS){
    printf("failed to get camera info\n");
  }
  printf("Opened camera %32s, size %dx%d\n",camInfo.strSensorName,camInfo.nMaxWidth,camInfo.nMaxHeight);
  if((nRet=is_SetColorMode(hCam,IS_CM_MONO8))!=IS_SUCCESS){
    printf("setColorMode failed\n");
  }
  

  if((nRet=is_SetAOI(hCam,IS_SET_IMAGE_AOI,&xpos,&ypos,&width,&height))!=IS_SUCCESS)
    printf("is_SetAOI failed\n");
  //if((nRet=is_SetExposureTime(hCam,IS_SET_ENABLE_AUTO_SHUTTER,&expMax))!=IS_SUCCESS)
  //  printf("is_SetExposureTime failed\n");
  if((nRet=is_SetFrameRate(hCam,(double)camstr->frameRate,&actualFrameRate))!=IS_SUCCESS)
    printf("is_SetFrameRate failed\n");
  if((nRet=is_SetExposureTime(hCam,0.,&expMax))!=IS_SUCCESS)
    printf("is_SetExposureTime failed\n");
  printf("Exposure time set to %gms with frame rate %g Hz\n",expMax,actualFrameRate);
  if (camInfo.bGlobShutter == TRUE)
    is_SetGlobalShutter(hCam, IS_SET_GLOBAL_SHUTTER_ON);
  // Alloc some memory for image buffer
  for(i=0;i<nBuffers;i++){
    if((nRet=is_AllocImageMem(hCam, (INT)width, (INT)height, bitsPerPxl,&(camstr->imageMem[i]), &(camstr->pid[i])))!=IS_SUCCESS){
      printf("Failed to allocate camera memory - closing camera\n");
      camdoFree(camstr);
      *camHandle=NULL;
      return 1;
    }
    if((nRet=is_AddToSequence(hCam,camstr->imageMem[i],camstr->pid[i]))!=IS_SUCCESS){
      printf("Failed is_AddToSequence\n");
      camdoFree(camstr);
      *camHandle=NULL;
      return 1;
    }
  }
  if((nRet=is_SetExternalTrigger(hCam, IS_SET_TRIGGER_SOFTWARE))!=IS_SUCCESS)
    printf("is_SetExternalTrigger failed\n");
  nRet=is_SetDisplayMode(hCam, IS_GET_DISPLAY_MODE);
  if(!(nRet & IS_SET_DM_DIB)){
    printf("is_SetDisplayMode failed\n");
    camdoFree(camstr);
    *camHandle=NULL;
    return 1;
  }
  //Now install handling... do we want an event or a message handler?  An event, because messages are windoze only.
  
  if((nRet=is_InitEvent(hCam,NULL,IS_SET_EVENT_FRAME))!=IS_SUCCESS)
    printf("is_InitEvent failed\n");
  if((nRet=is_EnableEvent(hCam,IS_SET_EVENT_FRAME))!=IS_SUCCESS)
    print("is_EnableEvent failed\n");
  is_CaptureVideo(camstr->hCam,IS_WAIT);//set to live mode...
  camstr->captureStarted=1;

  if(camNewParam(*camHandle,pbuf,thisiter,arr)!=0){
    printf("Error in camOpen->newParam...\n");
    dofree(camstr);
    *camHandle=NULL;
    return 1;
  }



  //if((nRet=is_WaitEvent(hCam,IS_SET_EVENT_FRAME,1000))!=IS_SUCCESS)
  //  printf("is_WaitEvent failed\n");
  //is_WaitForNextImage(hCam,1000,&imgptr,&imgid);

  //is_FreezeVideo(hCam,IS_DONT_WAIT);
  return 0;
}


/**
   Close a camera of type name.  Args are passed in the int32 array of size n, and state data is in camHandle, which should be freed and set to NULL before returning.
*/
int camClose(void **camHandle){
  CamStruct *camstr;
  printf("Closing camera\n");
  if(*camHandle==NULL)
    return 1;
  camstr=(CamStruct*)*camHandle;
  camdoFree(camstr);
  *camHandle=NULL;
  printf("Camera closed\n");
  return 0;
}


/**
   Called when we're starting processing the next frame.  This doesn't actually wait for any pixels.
*/
int camNewFrameSync(void *camHandle,unsigned int thisiter,double starttime){
  //printf("camNewFrame\n");
  CamStruct *camstr;
  int i;
  char *cd;
  char tmp;
  char *imgMem=NULL;
  INT pitch;
  camstr=(CamStruct*)camHandle;
  if(camHandle==NULL){// || camstr->streaming==0){
    //printf("called camNewFrame with camHandle==NULL\n");
    return 1;
  }

  is_GetImageMem(camstr->hCam,(void**)&imgMem);
  is_GetImageMemPitch(camstr->hCam,&pitch);
  printf("Image retrieved at %p, pitch %d\n",imgMem,pitch);
  memcpy(camstr->imgdata,imgMem,sizeof(char)*camstr->npxls);
  camstr->frameno++;
  for(i=0; i<camstr->ncam; i++){
    camstr->userFrameNo[i]++;//=camstr->frameno;
  }
  return 0;
}
