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
#include <dc1394/dc1394.h>
#define __LINUX__ 1

#include "darc.h"
typedef enum{
  FWBRIGHTNESS,
  FWEXPOSURE,
  FWFRAMERATE,
  FWGAIN,
  FWPACKETSIZE,
  FWPRINT,
  FWSHUTTER,
  //Add more before this line.
  CAMNBUFFERVARIABLES//equal to number of entries in the enum
}CAMBUFFERVARIABLEINDX;

#define camMakeNames() bufferMakeNames(CAMNBUFFERVARIABLES,"fwBrightness","fwExposure","fwFrameRate","fwGain","fwPacketSize","fwPrint","fwShutter")


typedef struct{
  int go;
  int frameno;
  unsigned short *imgdata;
  int datasize;
  int npxls;
  unsigned int *userFrameNo;
  int ncam;
  char *paramNames;
  circBuf *rtcErrorBuf;
  int index[CAMNBUFFERVARIABLES];
  void *values[CAMNBUFFERVARIABLES];
  char dtype[CAMNBUFFERVARIABLES];
  int nbytes[CAMNBUFFERVARIABLES];
  pthread_mutex_t mutex;
  pthread_cond_t cond;
  dc1394video_frame_t *frame;
  dc1394_t *dc1394;
  dc1394camera_t *cam;
  pthread_t threadid;
  int minit;
  int fno;
  int lastfno;
  int brightness;
  int exposure;
  int frameRate;
  int gain;
  int packetSize;
  int shutter;
}CamStruct;



void camdofree(CamStruct *camstr){
  //int i;
  struct timespec timeout;
  if(camstr!=NULL){
    if(camstr->paramNames!=NULL)
      free(camstr->paramNames);
    camstr->go=0;
    //now wait for the thread.
    if(camstr->threadid!=0){
      printf("Joining thread\n");
      clock_gettime(CLOCK_REALTIME, &timeout);
      timeout.tv_sec++;//wait 1 second.
      if(pthread_timedjoin_np(camstr->threadid,NULL,&timeout)!=0){
	printf("Thread failed to join after timeout - cancelling\n");
	pthread_cancel(camstr->threadid);
	printf("Joining again...\n");
	pthread_join(camstr->threadid,NULL);
      }
    }
    if(camstr->minit>0)
      pthread_mutex_destroy(&camstr->mutex);
    if(camstr->minit>1)
      pthread_cond_destroy(&camstr->cond);
    if(camstr->cam!=NULL){
      dc1394_capture_stop(camstr->cam);
      dc1394_video_set_transmission(camstr->cam, DC1394_OFF);
      dc1394_camera_free(camstr->cam);
    }
    if(camstr->dc1394!=NULL)
      dc1394_free (camstr->dc1394);

    
    free(camstr);
  }
}

void *grabloop(void *cHandle){
  CamStruct *camstr=(CamStruct*)cHandle;
  dc1394video_frame_t *frame2=NULL;
  int err;
  pthread_mutex_lock(&camstr->mutex);
  while(camstr->go && camstr->frame==NULL){
    if((err=dc1394_capture_dequeue(camstr->cam, DC1394_CAPTURE_POLICY_WAIT, &camstr->frame))!=DC1394_SUCCESS){
      printf("Error grabbing from cam\n");
    }
  }
  pthread_mutex_unlock(&camstr->mutex);
  while(camstr->go){
    err=dc1394_capture_dequeue(camstr->cam, DC1394_CAPTURE_POLICY_WAIT, &frame2);
    if (err!=DC1394_SUCCESS) {
      printf("Error grabbing from camera\n");
      sleep(1);
    }else{
      pthread_mutex_lock(&camstr->mutex);
      dc1394_capture_enqueue(camstr->cam,camstr->frame);//frame has been finished with
      camstr->frame=frame2;//so now point to the next one.
      camstr->fno++;
      pthread_cond_signal(&camstr->cond);
      pthread_mutex_unlock(&camstr->mutex);
    }
  }
  return NULL;
}
int printCurrentInfo(CamStruct *camstr){
  dc1394camera_t *camera=camstr->cam;
  dc1394trigger_mode_t trigmode;
  unsigned int ui,mi,ma;
  dc1394bool_t tf;
  dc1394video_mode_t vmode;
  dc1394featureset_t features;
  dc1394error_t err;
  unsigned int w,h,colorCoding,min_bytes,max_bytes,actual_bytes,recommendedPacketSize;
  float f;
  uint64_t total_bytes = 0;
  dc1394color_codings_t colorCodings;
  int j;
  dc1394framerate_t framerate;
  
  dc1394_external_trigger_get_mode(camera,&trigmode);
  printf("Trigger mode %d (384?=internal)\n",trigmode);
  dc1394_feature_get_boundaries(camera,DC1394_FEATURE_SHUTTER,&mi,&ma);
  dc1394_feature_get_value(camera,DC1394_FEATURE_SHUTTER,&ui);
  printf("Shutter: %d (%d to %d valid)\n",ui,mi,ma);

  dc1394_feature_get_boundaries(camera,DC1394_FEATURE_BRIGHTNESS,&mi,&ma);
  dc1394_feature_get_value(camera,DC1394_FEATURE_BRIGHTNESS,&ui);
  printf("Brightness: %d (%d to %d valid)\n",ui,mi,ma);

  dc1394_feature_get_boundaries(camera,DC1394_FEATURE_EXPOSURE,&mi,&ma);
  dc1394_feature_get_value(camera,DC1394_FEATURE_EXPOSURE,&ui);
  printf("Exposure: %d (%d to %d valid)\n",ui,mi,ma);

  dc1394_feature_get_boundaries(camera,DC1394_FEATURE_GAIN,&mi,&ma);
  dc1394_feature_get_value(camera,DC1394_FEATURE_GAIN,&ui);
  printf("Gain: %d (%d to %d valid)\n",ui,mi,ma);

  dc1394_feature_is_present(camera,DC1394_FEATURE_TEMPERATURE,&tf);
  if(tf){
    dc1394_feature_get_boundaries(camera,DC1394_FEATURE_TEMPERATURE,&mi,&ma);
    dc1394_feature_get_value(camera,DC1394_FEATURE_TEMPERATURE,&ui);
    printf("Temperature: %d (%d to %d valid)\n",ui,mi,ma);
  }
  ma=0;
  mi=0;
  dc1394_feature_get_boundaries(camera,DC1394_FEATURE_TRIGGER,&mi,&ma);
  dc1394_feature_get_value(camera,DC1394_FEATURE_TRIGGER,&ui);
  printf("Trigger: %d (%d to %d valid)\n",ui,mi,ma);

  dc1394_feature_get_boundaries(camera,DC1394_FEATURE_TRIGGER_DELAY,&mi,&ma);
  dc1394_feature_get_value(camera,DC1394_FEATURE_TRIGGER_DELAY,&ui);
  printf("Trigger_Delay: %d (%d to %d valid)\n",ui,mi,ma);

  if(dc1394_feature_get_all(camera,&features)!=DC1394_SUCCESS)
    printf("Couldn't get feature set\n");
  else
    dc1394_feature_print_all(&features, stdout);
  dc1394_video_get_mode(camera,&vmode);
  if(vmode>=DC1394_VIDEO_MODE_FORMAT7_0 && vmode<=DC1394_VIDEO_MODE_FORMAT7_7){
    if((err=dc1394_format7_get_max_image_size(camera,vmode,&w,&h))==DC1394_SUCCESS)
      printf("Max size: %dx%d\n",w,h);
    if((err=dc1394_format7_get_color_codings(camera,vmode,&colorCodings))==DC1394_SUCCESS){
      for(j=0;j<colorCodings.num;j++){
	printf("Color coding %d\n",colorCodings.codings[j]);
	//probably want     DC1394_COLOR_CODING_MONO16,
      }
    }
    if((err=dc1394_format7_get_color_coding(camera,vmode,&colorCoding))==DC1394_SUCCESS){
      printf("Current color coding %d (352==8bit, 357==16bit)\n",colorCoding);
    }
    if((err=dc1394_format7_get_packet_parameters(camera, vmode, &min_bytes, &max_bytes))==DC1394_SUCCESS)
      printf( "camera reports allowed packet size from %d - %d bytes\n", min_bytes, max_bytes);
    if((err=dc1394_format7_get_packet_size(camera, vmode, &actual_bytes))==DC1394_SUCCESS)
      printf( "camera reports actual packet size = %d bytes\n", actual_bytes);
    if((err=dc1394_format7_get_recommended_packet_size(camera,vmode,&recommendedPacketSize))==DC1394_SUCCESS)
      printf("Recommended packet size %d\n",recommendedPacketSize);
    if((err=dc1394_format7_get_data_depth(camera,vmode,&ui))==DC1394_SUCCESS)
      printf("Bits per pixel: %d\n",ui);
    if((err=dc1394_format7_get_frame_interval(camera,vmode,&f))==DC1394_SUCCESS)
      printf("Frame interval: %g\n",f);
    if((err=dc1394_format7_get_pixel_number(camera,vmode,&ui))==DC1394_SUCCESS)
      printf("Pixels per frame: %d\n",ui);
    if((err=dc1394_format7_get_total_bytes(camera, vmode, &total_bytes))==DC1394_SUCCESS)
      printf( "camera reports total bytes per frame = %lu bytes\n",total_bytes);
  }else{
    if((err=dc1394_video_get_framerate(camera,&framerate))==DC1394_SUCCESS)
      printf("Framerate %d (enum not fps)\n",framerate);
    else
      printf("Error getting framerate\n");
	
  }
  return 0;
}

int camNewParam(void *camHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr){
  int err=0;
  int i;
  CamStruct *camstr=(CamStruct*)camHandle;
  //int nfound;
  dc1394camera_t *camera=camstr->cam;
  int *index=camstr->index;
  void **values=camstr->values;
  dc1394video_mode_t vmode;
  int brightness=camstr->brightness;
  int gain=camstr->gain;
  int exposure=camstr->exposure;
  int frameRate=camstr->frameRate;
  int shutter=camstr->shutter;
  int packetSize=camstr->packetSize;
  int print=0;
  bufferGetIndex(pbuf,CAMNBUFFERVARIABLES,camstr->paramNames,camstr->index,camstr->values,camstr->dtype,camstr->nbytes);
  i=FWBRIGHTNESS;
  if(index[i]>=0){
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      brightness=*((int*)values[i]);
    }else{
      printf("fwBrightness error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"fwBrightness error");
      err=1;
    }
  }else{
    printf("fwBrightness not found - ignoring\n");
  }
  i=FWEXPOSURE;
  if(index[i]>=0){
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      exposure=*((int*)values[i]);
    }else{
      printf("fwExposure error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"fwExposure error");
      err=1;
    }
  }else{
    printf("fwExposure not found - ignoring\n");
  }
  i=FWFRAMERATE;
  if(index[i]>=0){
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      frameRate=*((int*)values[i]);
    }else{
      printf("fwFrameRate error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"fwFrameRate error");
      err=1;
    }
  }else{
    printf("fwFrameRate not found - ignoring\n");
  }
  i=FWGAIN;
  if(index[i]>=0){
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      gain=*((int*)values[i]);
    }else{
      printf("fwGain error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"fwGain error");
      err=1;
    }
  }else{
    printf("fwGain not found - ignoring\n");
  }
  i=FWPACKETSIZE;
  if(index[i]>=0){
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      packetSize=*((int*)values[i]);
    }else{
      printf("fwPacketSize error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"fwPacketSize error");
      err=1;
    }
  }else{
    printf("fwPacketSize not found - ignoring\n");
  }
  i=FWSHUTTER;
  if(index[i]>=0){
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      shutter=*((int*)values[i]);
    }else{
      printf("fwShutter error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"fwShutter error");
      err=1;
    }
  }else{
    printf("fwShutter not found - ignoring\n");
  }
  i=FWPRINT;
  if(index[i]>=0){
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      print=*((int*)values[i]);
      *((int*)values[i])=0;
    }else{
      printf("fwPrint error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"fwPrint error");
      err=1;
    }
  }else{
    printf("fwPrint not found - ignoring\n");
  }
  if(brightness!=camstr->brightness || exposure!=camstr->exposure || gain!=camstr->gain || packetSize!=camstr->packetSize || shutter!=camstr->shutter || frameRate!=camstr->frameRate){
    //change some parameters.
    printf("Pausing transmission\n");
    if((err=dc1394_video_set_transmission(camera, DC1394_OFF))!=DC1394_SUCCESS){
      printf("Failed in pausing transmission - continuing anyway\n");
    }
    //if(dc1394_capture_stop(camera)!=DC1394_SUCCESS)
    //printf("Failed in capture_stop - continuing\n");
    if(brightness!=camstr->brightness){
      if(dc1394_feature_set_value(camera,DC1394_FEATURE_BRIGHTNESS,brightness)!=DC1394_SUCCESS)
	printf("Error setting brightness\n");
      else
	camstr->brightness=brightness;
    }
    if(exposure!=camstr->exposure){
      if(dc1394_feature_set_value(camera,DC1394_FEATURE_EXPOSURE,exposure)!=DC1394_SUCCESS)
	printf("Error setting exposure\n");
      else
	camstr->exposure=exposure;
    }
    if(frameRate!=camstr->frameRate){
      if((err=dc1394_video_set_framerate(camera,frameRate))!=DC1394_SUCCESS)
	printf("Error setting framerate\n");
      else
	camstr->frameRate=frameRate;
    }
    if(gain!=camstr->gain){
      if(dc1394_feature_set_value(camera,DC1394_FEATURE_GAIN,gain)!=DC1394_SUCCESS)
	printf("Error setting gain\n");
      else
	camstr->gain=gain;
    }
    if(packetSize!=camstr->packetSize){
      dc1394_video_get_mode(camera,&vmode);
      if(dc1394_format7_set_packet_size(camera, vmode, packetSize)!=DC1394_SUCCESS)
	printf("Error setting packet size\n");
      else
	camstr->packetSize=packetSize;
    }
    if(shutter!=camstr->shutter){
      if(dc1394_feature_set_value(camera,DC1394_FEATURE_SHUTTER,shutter)!=DC1394_SUCCESS)
	printf("Error setting shutter\n");
      else
	camstr->shutter=shutter;
    }
    printf("Resuming transmission\n");
    //if((err=dc1394_capture_setup(camera, 4, DC1394_CAPTURE_FLAGS_DEFAULT))!=DC1394_SUCCESS){
    //  printf("Error starting capture - continuing... and hoping...\n");
    //}
    if((err=dc1394_video_set_transmission(camera, DC1394_ON))!=DC1394_SUCCESS){
      printf("Failed in resuming transmission - continuing anyway and hoping for the best\n");
    }


  }
  if(print){
    printCurrentInfo(camstr);
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

int camOpen(char *name,int nargs,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **camHandle,int nthreads,unsigned int thisiter,unsigned int **frameno,int *framenoSize,int npxls,int ncam,int *pxlx,int* pxly){
  CamStruct *camstr;
  int i,j;
  unsigned short *tmps;
  dc1394camera_t *camera;
  unsigned int min_bytes, max_bytes;
  unsigned int actual_bytes;
  uint64_t total_bytes = 0,guid;
  unsigned int w, h;
  dc1394_t * d;
  dc1394camera_list_t * list;
  dc1394error_t err;
  dc1394video_modes_t video_modes;
  unsigned int colorCoding,recommendedPacketSize,ui;
  float f;
  //unsigned long ul;
  dc1394color_codings_t colorCodings;
  //dc1394feature_info_t feature;
  dc1394video_mode_t vmode;
  dc1394color_coding_t pmode;
  int offx,offy;
  dc1394framerates_t framerates;
  dc1394framerate_t framerate;
  unsigned int bits;
  printf("Initialising camera %s\n",name);
  if(ncam>1){
    printf("Oops - at the moment this library only supports one camera.  It won't be difficult to recode it to add extra, but at the moment I don't have a requirement for this, so it will have to wait (or you could try it).\n");
    *camHandle=NULL;
    return 1;
  }
  if((*camHandle=malloc(sizeof(CamStruct)))==NULL){
    printf("Couldn't malloc camera handle\n");
    return 1;
  }
  memset(*camHandle,0,sizeof(CamStruct));
  camstr=(CamStruct*)*camHandle;
  camstr->paramNames=camMakeNames();
  camstr->rtcErrorBuf=rtcErrorBuf;
  camstr->brightness=-1;
  camstr->exposure=-1;
  camstr->frameRate=-1;
  camstr->gain=-1;
  camstr->packetSize=-1;
  camstr->shutter=-1;
  if(pthread_mutex_init(&camstr->mutex,NULL)!=0){
    printf("Error initialising mutex\n");
    camdofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  camstr->minit++;
  if(pthread_cond_init(&camstr->cond,NULL)!=0){
    printf("Error initialising cond\n");
    camdofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  camstr->minit++;
  

  if(arr->pxlbuftype!='H' || arr->pxlbufsSize!=sizeof(unsigned short)*npxls){
    //need to resize the pxlbufs...
    arr->pxlbufsSize=sizeof(unsigned short)*npxls;
    arr->pxlbuftype='H';
    arr->pxlbufelsize=sizeof(unsigned short);
    tmps=realloc(arr->pxlbufs,arr->pxlbufsSize);
    if(tmps==NULL){
      if(arr->pxlbufs!=NULL)
	free(arr->pxlbufs);
      printf("pxlbuf malloc error in camfirewire.\n");
      arr->pxlbufsSize=0;
      free(*camHandle);
      *camHandle=NULL;
      return 1;
    }
    arr->pxlbufs=tmps;
    memset(arr->pxlbufs,0,arr->pxlbufsSize);
  }
  camstr->imgdata=arr->pxlbufs;
  camstr->datasize=arr->pxlbufsSize;
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


  d = dc1394_new ();
  camstr->dc1394=d;
  if (!d){
    printf("Failed to create firewire interface\n");
    camdofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  if((err=dc1394_camera_enumerate (d, &list))!=DC1394_SUCCESS){
    printf("Failed to enumerate cameras\n");
    camdofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  if (list->num == 0) {
    printf("No firewire cameras found\n");
    dc1394_camera_free_list(list);
    camdofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  guid=list->ids[0].guid;
  for(i=0;i<list->num;i++){
    printf("Camera with guid %lu found\n",list->ids[i].guid);
    if(nargs>1 && (*(unsigned long*)args)==list->ids[i].guid)
      guid=list->ids[i].guid;
  }
  printf("Opening camera with guid %lu\n",guid);
  camera=dc1394_camera_new (d, guid);
  if (!camera) {
    printf("Failed to initialize camera with guid %lu",guid);
    dc1394_camera_free_list(list);
    camdofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  camstr->cam=camera;
  dc1394_camera_free_list(list);
  printf("Camera opened\n");
  if(nargs>2 && args[2]){//print supported modes
    if((err=dc1394_video_get_supported_modes(camera,&video_modes))!=DC1394_SUCCESS){
      printf("Failed to get supported modes\n");
      video_modes.num=0;
    }
    for (i=video_modes.num-1;i>=0;i--) {
      printf("Mode %d\n",video_modes.modes[i]);
      if(video_modes.modes[i]>=DC1394_VIDEO_MODE_FORMAT7_0 && video_modes.modes[i]<=DC1394_VIDEO_MODE_FORMAT7_7){
	//print max allowed size
	if((err=dc1394_format7_get_max_image_size(camera,video_modes.modes[i],&w,&h))==DC1394_SUCCESS)
	  printf("Max %dx%d\n",w,h);
	if((err=dc1394_format7_get_color_codings(camera,video_modes.modes[i],&colorCodings))==DC1394_SUCCESS){
	  for(j=0;j<colorCodings.num;j++){
	    printf("Color coding %d\n",colorCodings.codings[j]);
	    //probably want     DC1394_COLOR_CODING_MONO16,
	  }
	}
	if((err=dc1394_format7_get_color_coding(camera,video_modes.modes[i],&colorCoding))==DC1394_SUCCESS){
	  printf("Current color coding %d (352==8bit, 357==16bit)\n",colorCoding);
	}
	if((err=dc1394_format7_get_packet_parameters(camera, video_modes.modes[i], &min_bytes, &max_bytes))==DC1394_SUCCESS)
	  printf( "camera reports allowed packet size from %d - %d bytes\n", min_bytes, max_bytes);
	if((err=dc1394_format7_get_packet_size(camera, video_modes.modes[i], &actual_bytes))==DC1394_SUCCESS)
	  printf( "camera reports actual packet size = %d bytes\n", actual_bytes);
	if((err=dc1394_format7_get_recommended_packet_size(camera,video_modes.modes[i],&recommendedPacketSize))==DC1394_SUCCESS)
	  printf("Recommended packet size %d\n",recommendedPacketSize);
	if((err=dc1394_format7_get_data_depth(camera,video_modes.modes[i],&ui))==DC1394_SUCCESS)
	  printf("Bits per pixel: %d\n",ui);
	if((err=dc1394_format7_get_frame_interval(camera,video_modes.modes[i],&f))==DC1394_SUCCESS)
	  printf("Frame interval: %g\n",f);
	if((err=dc1394_format7_get_pixel_number(camera,video_modes.modes[i],&ui))==DC1394_SUCCESS)
	  printf("Pixels per frame: %d\n",ui);
	if((err=dc1394_format7_get_total_bytes(camera, video_modes.modes[i], &total_bytes))==DC1394_SUCCESS)
	  printf( "camera reports total bytes per frame = %lu bytes\n",total_bytes);
      }else{
	if(dc1394_video_get_supported_framerates(camera,video_modes.modes[i],&framerates)!=DC1394_SUCCESS)
	  printf("Error getting framerates\n");
	else{
	  for(j=0;j<framerates.num;j++)
	    printf("Allowed framerate: %d\n",framerates.framerates[j]);
	}
      }
    }

  }
  if(nargs>3 && args[3]!=-1)
    vmode=args[3];//88 for FORMAT7_0.  70 for 640x480, 16bpp.
  else
    vmode=DC1394_VIDEO_MODE_FORMAT7_0;
  if(nargs>4 && args[4]!=-1)
    pmode=args[4];//352 for 8 bit, 357 for 16 bit.
  else
    pmode=DC1394_COLOR_CODING_MONO16;
  if(nargs>5 && args[5]!=0)
    w=args[5];
  else
    w=640;
  if(nargs>6 && args[6]!=0)
    h=args[6];
  else
    h=480;
  if(nargs>7)
    offx=args[7];
  else
    offx=0;
  if(nargs>8)
    offy=args[8];
  else
    offy=0;
  if(nargs>9)
    framerate=args[9];
  else
    framerate=DC1394_FRAMERATE_15;

     
  printf("Setting video mode to %d, color mode to %d, offset %d,%d w,h %d %d\n",vmode,pmode,offx,offy,w,h);
  dc1394_video_set_mode(camera, vmode);
  if(vmode>=DC1394_VIDEO_MODE_FORMAT7_0 && vmode<=DC1394_VIDEO_MODE_FORMAT7_7){
    err = dc1394_format7_set_roi(camera, vmode,
				 pmode,
				 DC1394_USE_MAX_AVAIL, // use max packet size
				 offx, offy, // left, top
				 w,h);  // width, height
    if(err!=DC1394_SUCCESS){
      printf("Error setting ROI/color mode or format\n");
      camdofree(camstr);
      *camHandle=NULL;
      return 1;
    }
  }else{
    if((err=dc1394_video_set_framerate(camera,framerate))!=DC1394_SUCCESS){
      printf("Failed to set framerate\n");
      camdofree(camstr);
      *camHandle=NULL;
      return 1;
    }else
      printf("framerate %d (enum not fps)\n",framerate);
  }
  if(nargs>2 && args[2]){
    printCurrentInfo(camstr);
  }

  if((err=dc1394_capture_setup(camera, 4, DC1394_CAPTURE_FLAGS_DEFAULT))!=DC1394_SUCCESS){
    printf("Error starting capture\n");
    camdofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  if(vmode>=DC1394_VIDEO_MODE_FORMAT7_0 && vmode<=DC1394_VIDEO_MODE_FORMAT7_7){
    if((err=dc1394_format7_get_total_bytes(camera, vmode, &total_bytes))!=DC1394_SUCCESS){
      printf("ERror getting total bytes\n");
      camdofree(camstr);
      *camHandle=NULL;
      return 1;
    }
  }else{
    if(dc1394_get_image_size_from_video_mode(camera,vmode,&w,&h)!=DC1394_SUCCESS){
      printf("Error getting size\n");
      camdofree(camstr);
      *camHandle=NULL;
      return 1;
    }
    if(dc1394_video_get_data_depth(camera,&bits)!=DC1394_SUCCESS){
      printf("Error getting bpp\n");
      camdofree(camstr);
      *camHandle=NULL;
      return 1;
    }
    printf("%dx%d with %dbpp\n",w,h,bits);
    total_bytes=w*h*((bits+7)/8);
  }
  if(total_bytes!=camstr->datasize){
    printf("Total bytes not what is expected\n");
    camdofree(camstr);
    *camHandle=NULL;
    return 1;
  }

  if((err=dc1394_video_set_transmission(camera,DC1394_ON))!=DC1394_SUCCESS){
    printf("unable to start camera iso transmission");
    dc1394_capture_stop(camera);
    dc1394_camera_free(camera);
    camdofree(camstr);
    *camHandle=NULL;
    return 1;
  }

  if(camNewParam(*camHandle,pbuf,thisiter,arr)!=0){
    printf("Error in camOpen->newParam...\n");
    camdofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  camstr->go=1;


  //ok now time so start the thread that will grab the images.
  pthread_create(&camstr->threadid,NULL,grabloop,camstr);


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
  camdofree(camstr);
  *camHandle=NULL;
  printf("Camera closed\n");
  return 0;
}


/**
   Called when we're starting processing the next frame.
   Called by a single subap processing thread.
*/
int camNewFrameSync(void *camHandle,unsigned int thisiter,double starttime){
  //printf("camNewFrame\n");
  CamStruct *camstr;
  int i;
  struct timespec timeout;
  char *im;
  //unsigned char *imgMem=NULL;
  //INT pitch;
  camstr=(CamStruct*)camHandle;
  if(camHandle==NULL){// || camstr->streaming==0){
    //printf("called camNewFrame with camHandle==NULL\n");
    return 1;
  }
  pthread_mutex_lock(&camstr->mutex);
  if(camstr->lastfno==camstr->fno){//fno hasn't updated
    clock_gettime(CLOCK_REALTIME, &timeout);
    timeout.tv_sec++;//wait 1 second.
    if(pthread_cond_timedwait(&camstr->cond,&camstr->mutex,&timeout)!=0){
      printf("Error doing cond wait which waiting for new camera frame\n");
      return 1;
    }
  }
  if(camstr->frame!=NULL){
    im=(char*)camstr->frame->image;
    if(camstr->frame->little_endian==DC1394_TRUE){
      memcpy(camstr->imgdata,camstr->frame->image,camstr->datasize);
    }else{
      for(i=0;i<camstr->npxls;i++){
	camstr->imgdata[i]=im[i*2]|(im[i*2+1]<<16);
      }
      //printf("%d %lu %d %d %d\n",camstr->frame->image_bytes,camstr->frame->total_bytes,camstr->frame->stride,camstr->frame->padding_bytes,camstr->frame->packet_size);
    }
  }else
    memset(camstr->imgdata,0,camstr->datasize);
  camstr->lastfno=camstr->fno;
  pthread_mutex_unlock(&camstr->mutex);
  camstr->frameno++;
  for(i=0; i<camstr->ncam; i++){
    camstr->userFrameNo[i]++;//=camstr->frameno;
  }
  return 0;
}
