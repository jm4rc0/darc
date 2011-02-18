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
#include "/root/andor/examples/common/atmcdLXd.h"

typedef struct{
  int transferRequired;
  int frameno;
  short *imgdata;
  int npxls;
  int streaming;
  int userFrameNo;
}Andor;

/**
   Find out if this SO library supports your camera.

*/
int camQuery(char *name){
  //Note, the strings aren't necessarily null terminated...
  if(strcmp(name,"andorpci")==0)
    return 0;
  return 1;
}
/**
   Open a camera of type name.  Args are passed in a int32 array of size n, which can be cast if necessary.  Any state data is returned in camHandle, which should be NULL if an error arises.
   pxlbuf is the array that should hold the data. The library is free to use the user provided version, or use its own version as necessary (ie a pointer to physical memory or whatever).  It is of size npxls*sizeof(short).
   ncam is number of cameras, which is the length of arrays pxlx and pxly, which contain the dimensions for each camera.
   Name is used if a library can support more than one camera.

*/

int camOpen(char *name,int n,int *args,char *buf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **camHandle,int npxls,unsigned short *pxlbuf,int ncam,int *pxlx,int* pxly,int* frameno){
  float exp,acc,kinetic;
  Andor *andor;
  printf("Initialising camera %s\n",name);
  if(camQuery(name)){
    printf("Wrong camera type\n");
    return 1;
  }
  if(ncam!=1){
    printf("error: andorpci 1 camera only in rtccamera.so\n");
    return 1;
  }

  if(Initialize("/root/andor/examples/common")!=DRV_SUCCESS){
    printf("Error Initialize\n");
    return 1;
  }
  if(SetShutter(1,1,50,50)!=DRV_SUCCESS){
    printf("SetShutter error\n");
    return 1;
  }
  if(SetTemperature(10)!=DRV_SUCCESS){
    printf("SetTemperature error\n");
    return 1;
  }
  if(CoolerON()!=DRV_SUCCESS){
    printf("CoolerON error\n");
    return 1;
  }
  if(SetReadMode(4)!=DRV_SUCCESS){
    printf("SetReadMode error\n");
    return 1;
  }
  if(SetImage(1,1,1,*pxlx,1,*pxly)!=DRV_SUCCESS){
    printf("SetImage error\n");
    return 1;
  }
  if(SetAcquisitionMode(5)!=DRV_SUCCESS){
    printf("SetAcquisitionMode error\n");
    return 1;
  }
  if(SetExposureTime(0.0)!=DRV_SUCCESS){
    printf("SetExposureTime error\n");
    return 1;
  }
  if(SetEMCCDGain(4000)!=DRV_SUCCESS){
    printf("SetEMCCDGain error\n");
    return 1;
  }
  if(SetVSSpeed(1)!=DRV_SUCCESS){
    printf("SetVSSpeed error\n");
    return 1;
  }
  if(SetHSSpeed(0,0)!=DRV_SUCCESS){
    printf("SetHSSpeed error\n");
    return 1;
  }
  if(SetDMAParameters(1,0.003)!=DRV_SUCCESS){
    printf("SetDMAParameters error\n");
    return 1;
  }
  if(GetAcquisitionTimings(&exp,&acc,&kinetic)!=DRV_SUCCESS){
    printf("GetAcquisitionTimings error\n");
    return 1;
  }
  if((*camHandle=malloc(sizeof(Andor)))==NULL){
    printf("Couldn't malloc camera handle\n");
    return 1;
  }
  memset(*camHandle,0,sizeof(Andor));
  andor=(Andor*)*camHandle;
  andor->imgdata=pxlbuf;
  andor->userFrameNo=frameno;
  andor->npxls=*pxlx * *pxly;
  printf("Camera opened\n");

  return 0;
}

/**
   Close a camera of type name.  Args are passed in the int32 array of size n, and state data is in camHandle, which should be freed and set to NULL before returning.
*/
int camClose(void **camHandle){
  int t;
  printf("Closing camera\n");
  if(*camHandle==NULL)
    return 1;
  t=-100;
  if(CoolerOFF()!=DRV_SUCCESS){
    printf("CoolerOFF error\n");
    printf("Skipping GetTemperature...\n");
    t=-20;
  }
  
  while(t<-20){
    GetTemperature(&t);
    printf("GetTemperature temp %d\n",t);
    sleep(1);
  }
  if(ShutDown()!=DRV_SUCCESS){
    printf("Shutdown error\n");
  }
  free(*camHandle);
  *camHandle=NULL;
  printf("Camera closed\n");
  return 0;
}
/**
   Start the camera framing, using the args and camera handle data.
*/
int camStartFraming(int n,int *args,void *camHandle){
  Andor *andor;
  if(camHandle==NULL){
    printf("called camStartFraming with camHandle==NULL\n");
    return 1;
  }
  if(StartAcquisition()!=DRV_SUCCESS){
    printf("StartAcquisition error\n");
    return 1;
  }
  andor=(Andor*)camHandle;
  andor->streaming=1;
  printf("Framing camera\n");
  return 0;
}
/**
   Stop the camera framing
*/
int camStopFraming(void *camHandle){
  Andor *andor;
  if(camHandle==NULL){
    printf("called camStopFraming with camHandle==NULL\n");
    return 1;
  }
  if(AbortAcquisition()!=DRV_SUCCESS){
    printf("AbortAcquisition error\n");
  }
  andor=(Andor*)camHandle;
  andor->streaming=0;
  printf("Stopping framing\n");
  return 0;
}

/**
   Can be called to get the latest iamge taken by the camera
*/
int camGetLatest(void *camHandle){
  if(camHandle==NULL){
    printf("called camGetLatest with camHandle==NULL\n");
    return 1;
  }
  printf("Getting latest frame\n");
  return 0;
}

/**
   Called when we're starting processing the next frame.  This doesn't actually wait for any pixels.
*/
int camNewFrame(void *camHandle){
  //printf("camNewFrame\n");
  Andor *andor;
  andor=(Andor*)camHandle;
  if(camHandle==NULL || andor->streaming==0){
    //printf("called camNewFrame with camHandle==NULL\n");
    return 1;
  }
  //printf("camNewFrame\n");
  andor->frameno++;
  andor->transferRequired=1;
  return 0;
}

/**
   Wait for the next n pixels of the current frame to arrive.
*/
int camWaitPixels(int n,int cam,void *camHandle){
  //printf("camWaitPixels %d, camera %d\n",n,cam);
  //For andor, we actually have to wait for the whole frame...
  Andor *andor;
  andor=(Andor*)camHandle;
  long indx;
  int err;
  //static struct timeval t1;
  //struct timeval t2;
  //struct timeval t3;
  if(camHandle==NULL || andor->streaming==0){
    //printf("called camWaitPixels with camHandle==NULL\n");
    return 1;
  }
  if(n<0)
    n=0;
  if(n>andor->npxls)
    n=andor->npxls;
  //printf("camWaitPixels\n");
  if(andor->transferRequired){
    andor->transferRequired=0;
    //gettimeofday(&t3,NULL);
    /*
    if(GetTotalNumberImagesAcquired(&indx)!=DRV_SUCCESS)
      printf("GetTotalNumberImagesAcquired error\n");
    while(indx<andor->frameno){//wait for the frame
      usleep(50);
      if(GetTotalNumberImagesAcquired(&indx)!=DRV_SUCCESS)
	printf("GetTotalNumberImagesAcquired error\n");
    }
    andor->frameno=indx;
    //if a new frame has arrived... transfer it...
    if((err=GetMostRecentImage16(andor->imgdata,andor->npxls))!=DRV_SUCCESS){
      printf("GetMostRecentImage16 error %d\n",err);
      }*/
    if(GetTotalNumberImagesAcquired(&indx)!=DRV_SUCCESS)
      printf("GetTotalNumberImagesAcquired error\n");
    while(andor->frameno>indx){//wait for next frame...
      if((err=WaitForAcquisition())!=DRV_SUCCESS)
	printf("WaitForAcquisition error %d\n",err);
      if(GetTotalNumberImagesAcquired(&indx)!=DRV_SUCCESS)
	printf("GetTotalNumberImagesAcquired error\n");
    }
    if((err=GetMostRecentImage16(andor->imgdata,andor->npxls))!=DRV_SUCCESS)
      printf("GetMostRecentImage16 error %d\n",err);
    //if(GetTotalNumberImagesAcquired(&indx)!=DRV_SUCCESS)
    //  printf("GetTotalNumberImagesAcquired error\n");
    if(andor->frameno==indx){
      //printf("Did advance a frame %ld\n",indx);
    }else if(andor->frameno>indx){
      printf("Didn't advance a frame %ld\n",indx);
    }else{
      printf("Advanced more than one frame %ld\n",indx);
    }
    andor->frameno=indx;
    *andor->userFrameNo=indx;
    //gettimeofday(&t2,NULL);
    //printf("Frame %ld transferred in %g (waiting %g)\n",indx,t2.tv_sec-t1.tv_sec+(t2.tv_usec-t1.tv_usec)*1e-6,t2.tv_sec-t3.tv_sec+(t2.tv_usec-t3.tv_usec)*1e-6);
    //t1=t2;
  }
  return 0;
}
