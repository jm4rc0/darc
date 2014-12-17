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

Requires libusb1-devel for fedora

And 
wget ftp://ftp.sbig.com/pub/devsw/LinuxDevKit.tar.gz
cp x86/c/testapp/sbigudrv.h /usr/local/include/
cp x86/c/testapp/lpardrv.h /usr/local/include/
cp x86/c/lib64/libsbigyudrv.so /usr/local/lib
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
#include <libusb.h>
#include "lpardrv.h"
#include "darc.h"
typedef enum{
  SBIGACTUALEXP,
  SBIGBINNING,
  SBIGBOOSTGAIN,
  SBIGEXPTIME,  
  SBIGEXTTRIGGER,
  SBIGGRABMODE,
  SBIGNFRAMES,
  SBIGSHUTTER,
  SBIGTIMEOUT,
  SBIGTURBOMODE,
  //Add more before this line.
  CAMNBUFFERVARIABLES//equal to number of entries in the enum
}CAMBUFFERVARIABLEINDX;

#define camMakeNames() bufferMakeNames(CAMNBUFFERVARIABLES,"sbigActualExp","sbigBinning","sbigBoostGain","sbigExpTime","sbigExtTrigger","sbigGrabMode","sbigNFrames","sbigShutter","sbigTimeout","sbigTurboMode")

#define MAX_XPXLS 3326
#define MAX_YPXLS 2504
//#define nBuffers 8
typedef struct{
  int frameno;
  unsigned short *imgdata;
  int npxls;
  int xpxls;
  int ypxls;
  int xpos;
  int ypos;
  int width;
  int height;
  unsigned int *userFrameNo;
  int ncam;
  int captureStarted;
  int camOpen;
  int nCounts;
  FILE *fp;
  //HIDS hCam;
  //char *imageMem[nBuffers];
  //INT pid[nBuffers];
  char *paramNames;
  int turbomode;
  int timeout;
  ulong expTime;
  circBuf *rtcErrorBuf;
  int index[CAMNBUFFERVARIABLES];
  void *values[CAMNBUFFERVARIABLES];
  char dtype[CAMNBUFFERVARIABLES];
  int nbytes[CAMNBUFFERVARIABLES];
  unsigned char *lastImgMem;
  int nFrames;
  int grabMode;
  int trigger;
  int shutter;
  int black;
  int binning;
  int maxcnt;
}CamStruct;



void camdoFree(CamStruct *camstr){
  int err=0;
  time_t rawtime; 
  struct tm *ti;   
  printf("Freeing camera (camdoFree)\n");
  if(camstr!=NULL){
    if(camstr->paramNames!=NULL)
      free(camstr->paramNames);

    if(camstr->camOpen){
      //Temporary method to make sure the shutter closed before close the driver
      StartExposureParams2 	sep2;
      sep2.ccd 			= CCD_IMAGING;
      sep2.exposureTime = 0.01;
      sep2.abgState 	= ABG_CLK_MED7;
      sep2.openShutter 	= SC_CLOSE_SHUTTER;
      sep2.top 			= 0;
      sep2.left 		= 0;
      sep2.height 		= 10;
      sep2.width 		= 10;
      sep2.readoutMode 	= RM_1X1; 
      EndExposureParams eep;
      eep.ccd = CCD_IMAGING;             
      SBIGUnivDrvCommand(CC_START_EXPOSURE2, &sep2, NULL);
      SBIGUnivDrvCommand(CC_END_EXPOSURE, &eep, NULL);      
      
      //Close device for science camera
      err=(PAR_ERROR)SBIGUnivDrvCommand(CC_CLOSE_DEVICE, NULL, NULL);
      time(&rawtime);
      ti = localtime(&rawtime);   
      if(camstr->fp){	
		fprintf(camstr->fp, "\n%d:%d:%d  Count of images taken: %d", ti->tm_hour, ti->tm_min, ti->tm_sec, camstr->nCounts);
		fprintf(camstr->fp, "\n%d:%d:%d  CC_CLOSE_DEVICE  err: %d", ti->tm_hour, ti->tm_min, ti->tm_sec, err);
	  }      
      if(err!=CE_NO_ERROR)
        printf ("CC_CLOSE_DEVICE    err: %d\n", err);
    
      //Close driver for science camera
      err=(PAR_ERROR)SBIGUnivDrvCommand(CC_CLOSE_DRIVER, NULL, NULL);
      time(&rawtime);
      ti = localtime(&rawtime);       
      if(camstr->fp)	
		fprintf(camstr->fp, "\n%d:%d:%d  CC_CLOSE_DRIVER  err: %d", ti->tm_hour, ti->tm_min, ti->tm_sec, err);      
      if(err!=CE_NO_ERROR)
        printf ("CC_CLOSE_DRIVER    err: %d\n", err);        
    }
    if(camstr->fp && camstr->fp!=stdout) fclose(camstr->fp);
    free(camstr);
    camstr=NULL;
  }
}


int camNewParam(void *camHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr){
  //the only param needed is camReorder if reorder!=0.
  int i;
  CamStruct *camstr=(CamStruct*)camHandle;
  int nfound,err=0;
  //int nRet;
  //double actualFrameRate;
  //double actualExpTime;
  //float *actualExp=NULL;
  //int prevGrabMode;
  nfound=bufferGetIndex(pbuf,CAMNBUFFERVARIABLES,camstr->paramNames,camstr->index,camstr->values,camstr->dtype,camstr->nbytes);
  printf("Number of found parameters: %d\n", nfound);
  
  time_t rawtime; 
  struct tm *ti; 
  time(&rawtime);
  ti = localtime(&rawtime);
  
  i=SBIGSHUTTER;
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      int shutter;
      shutter=*((int*)camstr->values[i]);
      if(shutter!=camstr->shutter){
	MiscellaneousControlParams cp;
	camstr->shutter=shutter;
	if(shutter<0)
	  shutter=0;
	printf("Shutter state set to %d\n",shutter);
	cp.fanEnable=0;
	cp.shutterCommand=shutter;
	cp.ledState=2;//0==off, 1=on, 2=blink slowly.
	err=(PAR_ERROR)SBIGUnivDrvCommand(CC_MISCELLANEOUS_CONTROL, &cp, NULL);
	if(err!=CE_NO_ERROR){
	  printf("Error setting shutter\n");
	}

      }
      if(camstr->fp)	
        fprintf(camstr->fp, "\n%d:%d:%d  Shutter state set to %d", 
			  ti->tm_hour, ti->tm_min, ti->tm_sec, camstr->shutter);
    }else{
      printf("sbigShutter error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"sbigShutter error");
      err=1;
    }
  }else{
    printf("sbigShutter not found - ignoring\n");
  }
  
  i=SBIGACTUALEXP;
/*
  if(camstr->index[i]>=0){
    if(camstr->nbytes[i]==4 && camstr->dtype[i]=='f'){
      actualExp=(float*)camstr->values[i];
    }else{
      printf("Not returning actual exposure time - wrong datatype\n");
      actualExp=NULL;
    }
  }else{
    actualExp=NULL;
    printf("sbigActualExp not found - ignoring\n");
  }
*/

  i=SBIGTURBOMODE;
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==4){
      camstr->turbomode=*((int*)camstr->values[i]);
      printf("Turbo Mode set to %d\n",camstr->turbomode);
      if(camstr->fp)	
        fprintf(camstr->fp, "\n%d:%d:%d  Turbo Mode set to %d", 
			  ti->tm_hour, ti->tm_min, ti->tm_sec, camstr->turbomode);
    }else{
      printf("sbigTurboMode error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"sbigTurboMode error");
      err=1;
    }
  }else{
    printf("sbigTurboMode not found - ignoring\n");
  }
  
  i=SBIGTIMEOUT;
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==4){
      camstr->timeout=*((int*)camstr->values[i]);
      printf("Waiting timeout set to %d s\n",camstr->timeout);
      if(camstr->fp)	
        fprintf(camstr->fp, "\n%d:%d:%d  Waiting timeout set to %d", 
			  ti->tm_hour, ti->tm_min, ti->tm_sec, camstr->timeout);
    }else{
      printf("sbigTimeout error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"sbigTimeout error");
      err=1;
    }
  }else{
    printf("sbigTimeout not found - ignoring\n");
  }
  
  i=SBIGEXPTIME;//If ==0, then exp time is 1/frame rate.
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==4){
      camstr->expTime=*((ulong*)camstr->values[i]);
	  printf("Exposure time set to %u ms\n", (uint)camstr->expTime*10);
      if(camstr->fp)	
        fprintf(camstr->fp, "\n%d:%d:%d  Exposure time set to %u ms", 
			  ti->tm_hour, ti->tm_min, ti->tm_sec, (uint)camstr->expTime*10);
    }else{
      printf("sbigExpTime error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"sbigExpTime error");
      err=1;
    }
  }else{
    printf("sbigExpTime not found - ignoring\n");
  }
  
  i=SBIGNFRAMES;
/*
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      camstr->nFrames=*((int*)camstr->values[i]);
    }else{
      printf("sbigNFrames error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"sbigNFrames error");
      err=1;
      camstr->nFrames=1;
    }
  }else{
    printf("sbigNFrames not found - ignoring\n");
    camstr->nFrames=1;
  }
*/
  
  i=SBIGBOOSTGAIN;
/*
  if(camstr->index[i]>=0){//has been found
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      if(*((int*)camstr->values[i])){
	if(is_SetGainBoost(camstr->hCam,IS_SET_GAINBOOST_ON)!=IS_SUCCESS)
	  printf("SetGainBoost(on) failed - maybe not available\n");
      }else{
	if(is_SetGainBoost(camstr->hCam,IS_SET_GAINBOOST_OFF)!=IS_SUCCESS)
	  printf("SetGainBoost(off) failed - maybe not available\n");
      }
    }else{
      printf("sbigBoostGain error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"sbigBoostGain error");
      err=1;
    }
  }else{
    printf("sbigBoostGain not found - ignoring\n");
  }
*/  
  
  i=SBIGEXTTRIGGER;
  if(camstr->index[i]>=0){//has been found
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      camstr->trigger=*((int*)camstr->values[i]);
      printf("External Trigger set to %d\n",camstr->trigger);
      if(camstr->fp)	
        fprintf(camstr->fp, "\n%d:%d:%d  External Trigger set to %d", 
			  ti->tm_hour, ti->tm_min, ti->tm_sec, camstr->trigger);
    }else{
      printf("sbigExtTrigger error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"sbigExtTrigger error");
      err=1;
    }
  }else{
    printf("sbigExtTrigger not found - ignoring\n");
  }
  
  
  i=SBIGBINNING;
/*
  if(camstr->index[i]>=0){//has been found
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      binning=(*((int*)camstr->values[i]));
      
      //set the value only when it is valid
      if(((binning&0x00FF)==0x000A&&(binning&0xFF00)>0x0100) || binning==0){	
	    camstr->binning = binning;	
        binning = (camstr->binning&0xFF00)/0x0100;
        binning = binning==0?1:binning;    
        
        //Make sure that the frame size is in the view field of CCD
        camstr->width  = binning*(camstr->xpos+camstr->xpxls)<MAX_XPXLS?camstr->xpxls:(MAX_XPXLS-binning*camstr->xpos)/binning;
        camstr->height = binning*(camstr->ypos+camstr->ypxls)<MAX_YPXLS?camstr->ypxls:(MAX_YPXLS-binning*camstr->ypos)/binning;
        printf("camstr->width=%d, camstr->height=%d, camstr->binning=%x, binning=%d\n", camstr->width, camstr->height, camstr->binning, binning);
      }else{		
		printf("input parameter for binning error\n");
	  }		       
    }else{
      printf("sbigbinning error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"sbigBinning error");
      err=1;
    }
  }else{
    printf("sbigbinning not found - ignoring\n");
  }
*/

  //prevGrabMode=camstr->grabMode;
  i=SBIGGRABMODE;
/*
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      camstr->grabMode=*((int*)camstr->values[i]);
    }else{
      printf("sbigGrabMode error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"sbigGrabMode error");
      err=1;
      camstr->grabMode=0;
    }
  }else{
    printf("sbigGrabMode not found - ignoring\n");
    camstr->grabMode=0;
  }
  
  if(camstr->grabMode!=prevGrabMode){
    //change the operation mode
    if(camstr->grabMode){
      if(camstr->captureStarted){
	camstr->captureStarted=0;
	is_StopLiveVideo(camstr->hCam,IS_WAIT);
      }
    }else{
      //turn on framing.
      is_CaptureVideo(camstr->hCam,IS_WAIT);//set to live mode...
      camstr->captureStarted=1;

    }
  }
*/
  
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
  int i;
  unsigned short *tmps;
  int binning=0;
  PAR_ERROR err;
  printf("Initialising camera %s\n",name);
  if((*camHandle=malloc(sizeof(CamStruct)))==NULL){
    printf("Couldn't malloc camera handle\n");
    return 1;
  }
  memset(*camHandle,0,sizeof(CamStruct));
  camstr=(CamStruct*)*camHandle;
  camstr->paramNames=camMakeNames();
  camstr->rtcErrorBuf=rtcErrorBuf;
  if(n>=5){
    camstr->xpos=args[0];
    camstr->ypos=args[1];
    camstr->width=args[2];
    camstr->height=args[3];
    camstr->turbomode=(args[4]);
    if(n>5)
      binning=args[5];
  }else{
    printf("Error - args should be >=5 in camsbigUSB: xpos,ypos,width,height,turbomode,(optional)binning\n");
    camdoFree(camstr);
    *camHandle=NULL;
    return 1;
  }

  if(arr->pxlbuftype!='H' || arr->pxlbufsSize!=sizeof(unsigned short)*npxls){
    //need to resize the pxlbufs...
    arr->pxlbufsSize=sizeof(unsigned short)*npxls;
    arr->pxlbuftype='H';
    arr->pxlbufelsize=sizeof(unsigned short);
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
  camstr->imgdata=arr->pxlbufs;  

  if(ncam!=1){
    printf("Sorry - only 1 camera currently allowed\n");
    free(*camHandle);
    *camHandle=NULL;
    return 1;
  }

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
 
  camstr->expTime=100;  //default exposuretime
  camstr->shutter=0;
  camstr->turbomode=0;
  camstr->trigger=0;
  camstr->timeout=50;
  camstr->npxls=npxls;//*pxlx * *pxly;
  camstr->xpxls=*pxlx;
  camstr->ypxls=*pxly;
  camstr->binning=binning;
  camstr->ncam=ncam;
  camstr->frameno=-1;
  camstr->userFrameNo=*frameno;
  for(i=0; i<ncam; i++){
    camstr->userFrameNo[i]=-1;
  }
  
  //Make sure that the frame size is in the view field of CCD 
  if((binning&0x00FF)==0x000A&&(binning&0xFF00)>0x0100)
    binning = (camstr->binning&0xFF00)/0x0100;   
  else{
	camstr->binning = 0;
    binning = 1;
  }
   
  camstr->width  = binning*(camstr->xpos+camstr->xpxls)<MAX_XPXLS?camstr->xpxls:(MAX_XPXLS-binning*camstr->xpos)/binning;
  camstr->height = binning*(camstr->ypos+camstr->ypxls)<MAX_YPXLS?camstr->ypxls:(MAX_YPXLS-binning*camstr->ypos)/binning;
  printf("camstr->width=%d, camstr->height=%d, camstr->binning=%x, binning=%d\n", camstr->width, camstr->height, camstr->binning, binning);
  
  //Open a log file to record science camera operation
  time_t rawtime; 
  struct tm *ti;   
  camstr->fp =stdin;// fopen("sbigcamera.log", "a");  
  if(camstr->fp){
    //record date and time
    time(&rawtime);
    ti = localtime(&rawtime); 
    fprintf(camstr->fp, "\n\nCurrent date/time: %s", asctime(ti));
    fprintf(camstr->fp, "Preparing to open the science camera");
    printf("Suceeded in opening log file in home directory\n");
  }else{
    printf("failed to open log file in home directory\n");
  }
  
  //Open driver for science camera
  err = (PAR_ERROR)SBIGUnivDrvCommand(CC_OPEN_DRIVER, NULL, NULL);
  time(&rawtime);
  ti = localtime(&rawtime);   
  if(camstr->fp)
    fprintf(camstr->fp, "\n%d:%d:%d  CC_OPEN_DRIVER  err: %d", ti->tm_hour, ti->tm_min, ti->tm_sec, err);  
  
  if(err!=CE_NO_ERROR){
    printf ("CC_OPEN_DRIVER    err: %d\n", err);
    camdoFree(camstr);
    *camHandle=NULL;
    return (int)err;
  }
					   
  //Open device for science camera through USB connection
  OpenDeviceParams odp;
  odp.deviceType = DEV_USB1;  
  err=(PAR_ERROR)SBIGUnivDrvCommand(CC_OPEN_DEVICE, &odp, NULL);
  time(&rawtime);
  ti = localtime(&rawtime); 
  if(camstr->fp)	
    fprintf(camstr->fp, "\n%d:%d:%d  CC_OPEN_DEVICE  err: %d", ti->tm_hour, ti->tm_min, ti->tm_sec, err);
  
  if(err!=CE_NO_ERROR){
    printf ("CC_OPEN_DEVICE    err: %d\n", err);
    SBIGUnivDrvCommand(CC_CLOSE_DRIVER, NULL, NULL);
    camdoFree(camstr);
    *camHandle=NULL;
    return (int)err;
  }
  camstr->camOpen=1;

  //Establish link for science camera
  EstablishLinkParams  elp;
  EstablishLinkResults elr;
  elp.sbigUseOnly = 0;
  err=(PAR_ERROR)SBIGUnivDrvCommand(CC_ESTABLISH_LINK, &elp, &elr);
  time(&rawtime);
  ti = localtime(&rawtime);   
  
  if(err!=CE_NO_ERROR){
    printf ("CC_ESTABLISH_LINK    err: %d\n", err);
    if(camstr->fp)	
      fprintf(camstr->fp, "\n%d:%d:%d  CC_ESTABLISH_LINK  err: %d", ti->tm_hour, ti->tm_min, ti->tm_sec, err);
    camdoFree(camstr);
    *camHandle=NULL;
    return (int)err;
  }else{
	 printf("Succeeded in establishing link with a camera of cameraType: %d\n", elr.cameraType);
	 if(camstr->fp) 
	   fprintf(camstr->fp, "\n%d:%d:%d  Succeeded in establishing link with a camera of cameraType: %d", 
			   ti->tm_hour, ti->tm_min, ti->tm_sec, elr.cameraType);
  }
  
  if(camNewParam(*camHandle,pbuf,thisiter,arr)!=0){
    printf("Error in camOpen->newParam...\n");
    camdoFree(camstr);
    *camHandle=NULL;
    return 1;
  }
  printf("sbig camera opened\n");
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
   Called when we're starting processing the next frame.
*/
int camNewFrameSync(void *camHandle,unsigned int thisiter,double starttime){
  int i, nHret, nLret;
  time_t rawtime; 
  struct tm *ti;   
  PAR_ERROR err;
  CamStruct *camstr;
  camstr=(CamStruct*)camHandle;
  if(camHandle==NULL)
    return 1;
  
  time(&rawtime);
  ti = localtime(&rawtime); 
  memset(camstr->imgdata,0,sizeof(unsigned short)*camstr->npxls);
  if(camstr->nCounts==0 && camstr->fp)	
    fprintf(camstr->fp, "\n%d:%d:%d  Begin to take images", ti->tm_hour, ti->tm_min, ti->tm_sec);  
  
  //To grab an image and store it in memory
  //Starting exposure  
  StartExposureParams2 	sep2;
  sep2.ccd          = CCD_IMAGING;
  sep2.abgState 	= ABG_CLK_MED7;
  sep2.openShutter 	= 0;//0=leave the shutter where it is.camstr->shutter;
  sep2.exposureTime = camstr->expTime;
  sep2.top 			= camstr->ypos;
  sep2.left 		= camstr->xpos;
  sep2.height 		= camstr->height;
  sep2.width 		= camstr->width;
  sep2.readoutMode 	= camstr->binning;
    
  if(camstr->turbomode){
    sep2.ccd          = sep2.ccd|START_SKIP_VDD;
    sep2.exposureTime = sep2.exposureTime|EXP_FAST_READOUT;
  }
  
  if(camstr->trigger)
    sep2.exposureTime = sep2.exposureTime|EXP_WAIT_FOR_TRIGGER_IN;
  printf("Start exposure\n");
  if((err=(PAR_ERROR)SBIGUnivDrvCommand(CC_START_EXPOSURE2, &sep2, NULL))!=CE_NO_ERROR){
	time(&rawtime);
	ti = localtime(&rawtime);     
	printf ("CC_START_EXPOSURE2    err: %d\n", err);
    if(camstr->fp)	
      fprintf(camstr->fp, "\n%d:%d:%d  CC_START_EXPOSURE2  err: %d  counts: %d", 
			  ti->tm_hour, ti->tm_min, ti->tm_sec, err, camstr->nCounts);
    return (int)err;
  }
  
  time_t ltimestart,ldelta;
  time(&ltimestart);
  
  //Query if exposure completed
  QueryCommandStatusParams qcsp;
  QueryCommandStatusResults qcsr;
  qcsp.command = CC_START_EXPOSURE2;  
  int printmsg=0;
  do{
	  qcsr.status = 0;
	  if((err=(PAR_ERROR)SBIGUnivDrvCommand(CC_QUERY_COMMAND_STATUS, &qcsp, &qcsr))!=CE_NO_ERROR){
		  time(&rawtime);
		  ti = localtime(&rawtime); 
		  printf ("CC_QUERY_COMMAND_STATUS    err: %d\n", err);
		  if(camstr->fp)
		    fprintf(camstr->fp, "\n%d:%d:%d  CC_QUERY_COMMAND_STATUS  err: %d  counts: %d", 
					ti->tm_hour, ti->tm_min, ti->tm_sec, err, camstr->nCounts);
		  return (int)err;
	  }
	  nHret = qcsr.status & 0x8000; //Get the highest bit
	  nLret = qcsr.status & 3;	//Get the last two bits
	  
	  time(&rawtime);
	  ldelta = rawtime - ltimestart;
	  if(ldelta>camstr->timeout){
	    if(printmsg==1){
	      printmsg=0;
	      printf("Timeout...\n");
	    }
	    if(nHret==0x8000){
	      printf("External trigger waiting timeout\n");
	      return 1;
	    }
	  }
  }while(nLret==2 || nHret==0x8000);	//Exposure in progress or waiting for external trigger 
 
  //End exposure
  EndExposureParams eep;
  eep.ccd = CCD_IMAGING;
  if(camstr->turbomode)
    eep.ccd = CCD_IMAGING|END_SKIP_DELAY;
  printf("End exposure\n");
  if((err=(PAR_ERROR)SBIGUnivDrvCommand(CC_END_EXPOSURE, &eep, NULL))!=CE_NO_ERROR){
	time(&rawtime);
	ti = localtime(&rawtime); 
    printf ("CC_END_EXPOSURE  err: %d\n", err);
    if(camstr->fp)
      fprintf(camstr->fp, "\n%d:%d:%d  CC_END_EXPOSURE  err: %d  counts: %d", 
			  ti->tm_hour, ti->tm_min, ti->tm_sec, err, camstr->nCounts);
    return (int)err;
  }
  
  //Starting to read the data
  StartReadoutParams 	srp;
  ReadoutLineParams rlp;
  srp.ccd    = CCD_IMAGING;
  srp.top    = camstr->ypos;  
  srp.left   = camstr->xpos;
  srp.height = camstr->height;
  srp.width  = camstr->width;
  srp.readoutMode = camstr->binning;
  printf("Start readout\n");
  if((err=(PAR_ERROR)SBIGUnivDrvCommand(CC_START_READOUT, &srp, NULL))!=CE_NO_ERROR){
	time(&rawtime);
	ti = localtime(&rawtime); 
    printf ("CC_START_READOUT    err: %d\n", err);
    if(camstr->fp)
      fprintf(camstr->fp, "\n%d:%d:%d  CC_START_READOUT  err: %d  counts: %d", 
			  ti->tm_hour, ti->tm_min, ti->tm_sec, err, camstr->nCounts);
    return (int)err;
  }
  
  rlp.ccd = CCD_IMAGING;
  rlp.pixelStart  = srp.left;
  rlp.pixelLength = srp.width;
  rlp.readoutMode = camstr->binning;		
  printf("Readout lines...\n");
  for (i=0; i<srp.height&&err==CE_NO_ERROR; i++){
    if((err=(PAR_ERROR)SBIGUnivDrvCommand(CC_READOUT_LINE, &rlp, camstr->imgdata+(long)i*camstr->xpxls))!=CE_NO_ERROR){
	  time(&rawtime);
	  ti = localtime(&rawtime); 		
      printf("CC_READOUT_LINE    err: %d, i: %d\n", err,i);
      if(camstr->fp)
        fprintf(camstr->fp, "\n%d:%d:%d  CC_READOUT_LINE  err: %d  i: %d  counts: %d",
				ti->tm_hour, ti->tm_min, ti->tm_sec, err, i, camstr->nCounts);
      return (int)err;
    }	  
  }
  printf("End readout\n");
  //End readout
  EndReadoutParams erp;	
  erp.ccd = CCD_IMAGING;
  if((err=(PAR_ERROR)SBIGUnivDrvCommand(CC_END_READOUT, &erp, NULL))!=CE_NO_ERROR){
    time(&rawtime);
	ti = localtime(&rawtime); 	
	printf ("CC_END_READOUT    err: %d\n", err);
    if(camstr->fp)	
      fprintf(camstr->fp, "\n%d:%d:%d  CC_END_READOUT  err: %d  counts: %d", 
			  ti->tm_hour, ti->tm_min, ti->tm_sec, err, camstr->nCounts);
    return (int)err;
  }  
  printf("Done %d\n",camstr->nCounts);
  camstr->nCounts++;
  
  camstr->frameno++;  
  for(i=0; i<camstr->ncam; i++){
    camstr->userFrameNo[i]++;//=camstr->frameno;
  }
  return 0;
}
