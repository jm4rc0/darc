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
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "atmcdLXd.h"

#include "darc.h"
typedef enum{
  ANDORCHANGECLAMP,
  ANDORCLAMP,
  ANDORCOOLERON,
  ANDOREMADVANCED,
  ANDOREMGAIN,
  ANDOREMMODE,
  ANDOREXPTIME,
  ANDORFANMODE,
  ANDORFASTEXTTRIG,
  ANDORHSSPEED,
  ANDOROUTPUTAMP,
  ANDOROUTPUTTYPE,
  ANDORPREAMP,
  ANDORTEMPERATURE,
  ANDORTRIGMODE,
  ANDORVSSPEED,
  ANDORVSAMP,
  //Add more before this line.
  CAMNBUFFERVARIABLES//equal to number of entries in the enum
}CAMBUFFERVARIABLEINDX;

#define camMakeNames() bufferMakeNames(CAMNBUFFERVARIABLES,\
    "andorChangeClamp",\
    "andorClamp",\
    "andorCoolerOn",\
    "andorEmAdvanced",\
    "andorEmGain",\
    "andorEmMode",\
    "andorExpTime",\
    "andorFanMode",\
    "andorFastExtTrig",\
    "andorHSSpeed",\
    "andorOutputAmp",\
    "andorOutputType",\
    "andorPreAmp",\
    "andorTemperature",\
    "andorTrigMode",\
    "andorVSSpeed",\
    "andorVSamp"\
)


typedef struct{
  unsigned short *imgdata;
  int npxls;
  int *npxlx;
  int *npxly;
  unsigned int *userFrameNo;
  int ncam;
  int camOpen;
  char *paramNames;
  circBuf *rtcErrorBuf;
  int index[CAMNBUFFERVARIABLES];
  void *values[CAMNBUFFERVARIABLES];
  char dtype[CAMNBUFFERVARIABLES];
  int nbytes[CAMNBUFFERVARIABLES];
  int err;
  int temp;
  int triggerMode;
  int fastExtTrig;
  int coolerOn;
  int outputType;
  int hsspeed;
  int vsspeed;
  int vsamp;
  int fanMode;
  int emAdvanced;
  int emmode;
  int emgain;
  int outputAmp;
  int preamp;
  int tempCurrent;
  int changeClamp;
  int clamp;
  float expTime;
  int triggerModeCurrent;
  int fastExtTrigCurrent;
  int coolerOnCurrent;
  int outputTypeCurrent;
  int hsspeedCurrent;
  int vsspeedCurrent;
  int vsampCurrent;
  int fanModeCurrent;
  int emAdvancedCurrent;
  int emmodeCurrent;
  int emgainCurrent;
  int outputAmpCurrent;
  int preampCurrent;
  int changeClampCurrent;
  int clampCurrent;
  float expTimeCurrent;
  int setAll;
  int started;
}CamStruct;





void camdoFree(CamStruct *camstr){
  if(camstr!=NULL){
    if(camstr->paramNames!=NULL)
      free(camstr->paramNames);
    free(camstr);
  }
}
/*
void *camWorker(void *CStr){
  CamStruct *camstr=(CamStruct*)CStr;
  while(camstr->go){
    

  }

}
*/

int camSetup(CamStruct *camstr){
  int i,low,high;
  int stopped=0;
  int err=0;
  for(i=0;i<camstr->ncam;i++){
    at_32 handle;
    if(GetCameraHandle(i,&handle)!=DRV_SUCCESS){
      printf("GetCameraHandle %d failed\n",i);
      return 1;
    }
    if(SetCurrentCamera(handle)!=DRV_SUCCESS){
      printf("SetCurrentCamera(%d) failed\n",i);
      return 1;
    }
    if(camstr->setAll || camstr->temp!=camstr->tempCurrent){
      //AbortAcquisition();
      if(SetTemperature(camstr->temp)!=DRV_SUCCESS){
	printf("SetTemperature error\n");
	err=1;
      }else{}
	//camstr->tempCurrent=camstr->temp;}
    }
    if(camstr->setAll || camstr->coolerOn!=camstr->coolerOnCurrent){
      if(camstr->coolerOn){
	if(CoolerON()!=DRV_SUCCESS){
	  printf("CoolerON error\n");
	  err=1;
	}else{}
	  //camstr->coolerOnCurrent=camstr->coolerOn;}
      }else{
	if(CoolerOFF()!=DRV_SUCCESS){
	  printf("CoolerOFF error\n");
	  err=1;
	}else{}
	  //camstr->coolerOnCurrent=camstr->coolerOn;{
      }
    }
    if(camstr->setAll || camstr->triggerMode!=camstr->triggerModeCurrent){
      if(SetTriggerMode(camstr->triggerMode)!=DRV_SUCCESS){
	printf("SetTriggerMode error\n");
	err=1;
      }else{}
	//camstr->triggerModeCurrent=camstr->triggerMode;}
    }
    if(camstr->setAll || camstr->expTime!=camstr->expTimeCurrent){
      int status=0;
      if(camstr->started){
	AbortAcquisition();
	camstr->started=0;
	stopped=1;
      }
      while(status!=DRV_IDLE){
	printf("Getting status - waiting for abort to complete\n");
	GetStatus(&status);
      }
      if(SetExposureTime(camstr->expTime)!=DRV_SUCCESS){
	printf("SetExposureTime error\n");
	err=1;
      }else{
	printf("Exposure time set to %g\n",camstr->expTime);
	//camstr->expTimeCurrent=camstr->expTime;
      }
    }
    if(camstr->setAll || camstr->fastExtTrig!=camstr->fastExtTrigCurrent){
      if(SetFastExtTrigger(camstr->fastExtTrig)!=DRV_SUCCESS){
	printf("SetFastExtTrig error\n");
	err=1;
      }else{}
	//camstr->fastExtTrigCurrent=camstr->fastExtTrig;}
    }
    if(camstr->setAll || camstr->vsspeed!=camstr->vsspeedCurrent){
      if(SetVSSpeed(camstr->vsspeed)!=DRV_SUCCESS){
	printf("SetVSSpeed error\n");
	err=1;
      }else{}
	//camstr->vsspeedCurrent=camstr->vsspeed;}
    }
    if(camstr->setAll || camstr->vsamp!=camstr->vsampCurrent){
      if(SetVSAmplitude(camstr->vsamp)!=DRV_SUCCESS){
	printf("SetVSAmplitude error\n");
	err=1;
      }else{}
	//camstr->vsampCurrent=camstr->vsamp;
    }
    if(camstr->setAll || camstr->hsspeed!=camstr->hsspeedCurrent){
      if(SetHSSpeed(camstr->outputType,camstr->hsspeed)!=DRV_SUCCESS){//fastest speed is 0,0
	printf("SetHSSpeed error\n");
	err=1;
      }else{}
      //camstr->hsspeedCurrent=camstr->hsspeed;
    }
    if(camstr->setAll || camstr->fanMode!=camstr->fanModeCurrent){
      if(SetFanMode(camstr->fanMode)!=DRV_SUCCESS){
	printf("SetFanMode error\n");
	err=1;
      }else{}
      //camstr->fanModeCurrent=camstr->fanMode;
    }
    if(camstr->setAll || camstr->clamp!=camstr->clampCurrent){
      if(SetBaselineClamp(camstr->clamp)!=DRV_SUCCESS){
	printf("SetBaselineClamp error\n");
	err=1;
      }else{}
	//camstr->clampCurrent=camstr->clamp;
    }
    if(camstr->setAll || camstr->changeClamp!=camstr->changeClampCurrent){
      if(camstr->changeClamp>0){
	if(SetBaselineOffset(100)!=DRV_SUCCESS){
	  printf("SetBaselineOffset+100 error\n");
	  err=1;
	}else{
	  //camstr->changeClamp=0;
	  //camstr->changeClampCurrent=camstr->changeClamp;
	}
      }else if(camstr->changeClamp<0){
	if(SetBaselineOffset(-100)!=DRV_SUCCESS){
	  printf("SetBaselineOffset-100 error\n");
	  err=1;
	}else{
	  //camstr->changeClamp=0;
	  //camstr->changeClampCurrent=camstr->changeClamp;
	}
      }
    }
    if(camstr->setAll || camstr->outputAmp!=camstr->outputAmpCurrent){
      if(SetOutputAmplifier(camstr->outputAmp)!=DRV_SUCCESS){
	printf("SetOutputAmplifier error\n");
	err=1;
      }else{}
	//camstr->outputAmpCurrent=camstr->outputAmp;
    }
    if(camstr->setAll || camstr->preamp!=camstr->preampCurrent){
      if(SetPreAmpGain(camstr->preamp)!=DRV_SUCCESS){
	printf("SetPreAmpGain error\n");
	err=1;
      }else{
	float preampgain;
	if(GetPreAmpGain(camstr->preamp,&preampgain)==DRV_SUCCESS){
	  printf("Preamp gain set to %g (index %d)\n",preampgain,camstr->preamp);
	  //camstr->preampCurrent=camstr->preamp;
	}else{
	  printf("GetPreAmpGain error\n");
	  err=1;
	}
      }
    }
    if(camstr->setAll || camstr->emAdvanced!=camstr->emAdvancedCurrent){
      if(SetEMAdvanced(camstr->emAdvanced)!=DRV_SUCCESS){//set to allow em gains >300.
	printf("SetEMAdvanced error\n");
	err=1;
      }else{}
	//camstr->emAdvancedCurrent=camstr->emAdvanced;
    }
    if(camstr->setAll || camstr->emmode!=camstr->emmodeCurrent){
      if(SetEMGainMode(camstr->emmode)!=DRV_SUCCESS){//0=>DAC from 0-255, 1=>DAC from 0-4095, 2=>Linear mode, 3=>Real EM gain.
	printf("SetEMGainMode error\n");
	err=1;
      }else{}
        //camstr->emmodeCurrent=camstr->emmode;
    }
    if(camstr->setAll || camstr->emgain!=camstr->emgainCurrent){
      if(GetEMGainRange(&low,&high)!=DRV_SUCCESS){
	printf("GetEMGainRange failed\n");
	err=1;
      }else{
	if(camstr->emgain<low || camstr->emgain>high){
	  printf("Requested gain %d out of range (%d - %d)\n",camstr->emgain,low,high);
	  err=1;
	}else{
	  if(SetEMCCDGain(camstr->emgain)!=DRV_SUCCESS){//the em gain
	    printf("SetEMCCDGain error\n");
	    err=1;
	  }else{}
	    //camstr->emgainCurrent=camstr->emgain;
	}
      }
    }
  }
  if(stopped){
    if(StartAcquisition()!=DRV_SUCCESS){
      printf("Error in StartAcquirisition\n");
      err=1;
    }else{
      printf("StartAcquisition successful\n");
      camstr->started=1;
    }
  }
  if(err==0){
    camstr->tempCurrent=camstr->temp;
    camstr->coolerOnCurrent=camstr->coolerOn;
    camstr->triggerModeCurrent=camstr->triggerMode;
    camstr->expTimeCurrent=camstr->expTime;
    camstr->fastExtTrigCurrent=camstr->fastExtTrig;
    camstr->vsspeedCurrent=camstr->vsspeed;
    camstr->hsspeedCurrent=camstr->hsspeed;
    camstr->vsampCurrent=camstr->vsamp;
    camstr->fanModeCurrent=camstr->fanMode;
    camstr->clampCurrent=camstr->clamp;
    camstr->changeClamp=0;
    camstr->changeClampCurrent=camstr->changeClamp;
    camstr->outputAmpCurrent=camstr->outputAmp;
    camstr->preampCurrent=camstr->preamp;
    camstr->emAdvancedCurrent=camstr->emAdvanced;
    camstr->emmodeCurrent=camstr->emmode;
    camstr->emgainCurrent=camstr->emgain;
  }
  camstr->setAll=0;
  return err;
}

int camNewParam(void *camHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr){
  //the only param needed is camReorder if reorder!=0.
  int i;
  CamStruct *camstr=(CamStruct*)camHandle;
  int nfound,err=0;
  nfound=bufferGetIndex(pbuf,CAMNBUFFERVARIABLES,camstr->paramNames,camstr->index,camstr->values,camstr->dtype,camstr->nbytes);
  i=ANDORCHANGECLAMP;
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      camstr->changeClamp=*((int*)camstr->values[i]);
      *((int*)camstr->values[i])=0;
      if(camstr->changeClamp!=0)
	printf("Changing clamp by %d\n",camstr->changeClamp>0?100:-100);
    }else{
      printf("andorChangeClamp error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"andorChangeClamp error");
      err=1;
    }
  }else{
    printf("andorChangeClamp not found - ignoring\n");
  }
  i=ANDORCLAMP;
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      camstr->clamp=*((int*)camstr->values[i]);
    }else{
      printf("andorClamp error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"andorClamp error");
      err=1;
    }
  }else{
    printf("andorClamp not found - ignoring\n");
  }
  i=ANDORCOOLERON;
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      camstr->coolerOn=*((int*)camstr->values[i]);
    }else{
      printf("andorCoolerOn error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"andorCoolerOn error");
      err=1;
    }
  }else{
    printf("andorCoolerOn not found - ignoring\n");
  }
  i=ANDOREMADVANCED;
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      camstr->emAdvanced=*((int*)camstr->values[i]);
    }else{
      printf("andorEmAdvanced error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"andorEmAdvanced error");
      err=1;
    }
  }else{
    printf("andorEmAdvanced not found - ignoring\n");
  }
  i=ANDOREMGAIN;
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      camstr->emgain=*((int*)camstr->values[i]);
    }else{
      printf("andorEmAdvanced error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"andorEmGain error");
      err=1;
    }
  }else{
    printf("andorEmGain not found - ignoring\n");
  }
  i=ANDOREMMODE;
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      camstr->emmode=*((int*)camstr->values[i]);
    }else{
      printf("andorEmMode error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"andorEmMode error");
      err=1;
    }
  }else{
    printf("andorEmMode not found - ignoring\n");
  }
  i=ANDOREXPTIME;
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='f' && camstr->nbytes[i]==sizeof(float)){
      camstr->expTime=*((float*)camstr->values[i]);
    }else{
      printf("andorExpTime error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"andorExpTime error");
      err=1;
    }
  }else{
    printf("andorExpTime not found - ignoring\n");
  }
  i=ANDORFANMODE;
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      camstr->fanMode=*((int*)camstr->values[i]);
    }else{
      printf("andorFanMode error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"andorFanMode error");
      err=1;
    }
  }else{
    printf("andorFanMode not found - ignoring\n");
  }
  i=ANDORFASTEXTTRIG;
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      camstr->fastExtTrig=*((int*)camstr->values[i]);
    }else{
      printf("andorFastExtTrig error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"andorFastExtTrig error");
      err=1;
    }
  }else{
    printf("andorFastExtTrig not found - ignoring\n");
  }
  i=ANDORHSSPEED;
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      camstr->hsspeed=*((int*)camstr->values[i]);
    }else{
      printf("andorHSSpeed error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"andorHSSpeed error");
      err=1;
    }
  }else{
    printf("andorHSSpeed not found - ignoring\n");
  }
  i=ANDOROUTPUTAMP;
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      camstr->outputAmp=*((int*)camstr->values[i]);
    }else{
      printf("andorOutputAmp error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"andorOutputAmp error");
      err=1;
    }
  }else{
    printf("andorOutputAmp not found - ignoring\n");
  }
  i=ANDOROUTPUTTYPE;
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      camstr->outputType=*((int*)camstr->values[i]);
    }else{
      printf("andorOutputType error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"andorOutputType error");
      err=1;
    }
  }else{
    printf("andorOutputType not found - ignoring\n");
  }
  i=ANDORPREAMP;
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      camstr->preamp=*((int*)camstr->values[i]);
    }else{
      printf("andorPreAmp error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"andorPreAmp error");
      err=1;
    }
  }else{
    printf("andorPreAmp not found - ignoring\n");
  }
  i=ANDORTEMPERATURE;
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      camstr->temp=*((int*)camstr->values[i]);
    }else{
      printf("andorTemperature error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"andorTemperature error");
      err=1;
    }
  }else{
    printf("andorTemperature not found - ignoring\n");
  }
  i=ANDORTRIGMODE;
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      camstr->triggerMode=*((int*)camstr->values[i]);
    }else{
      printf("andorTrigMode error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"andorTrigMode error");
      err=1;
    }
  }else{
    printf("andorTrigMode not found - ignoring\n");
  }
  i=ANDORVSSPEED;
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      camstr->vsspeed=*((int*)camstr->values[i]);
    }else{
      printf("andorVSSpeed error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"andorVSSpeed error");
      err=1;
    }
  }else{
    printf("andorVSSpeed not found - ignoring\n");
  }
  i=ANDORVSAMP;
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      camstr->vsamp=*((int*)camstr->values[i]);
    }else{
      printf("andorVSamp error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"andorVSamp error");
      err=1;
    }
  }else{
    printf("andorVSamp not found - ignoring\n");
  }
  camSetup(camstr);
  camstr->setAll=0;
  return err;
}


/**
   Open a andor PCI camera.  Args are passed in a int32 array of size n, which can be cast if necessary.  Any state data is returned in camHandle, which should be NULL if an error arises.
   ncam is number of cameras, which is the length of arrays pxlx and pxly, which contain the dimensions for each camera.
*/

int camOpen(char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **camHandle,int nthreads,unsigned int thisiter,unsigned int **frameno,int *framenoSize,int npxls,int ncam,int *pxlx,int* pxly){
  float exp,acc,kinetic;
  CamStruct *camstr;
  int xpos,ypos;
  unsigned short *tmps;
  int i;
  at_32 totCams;
  printf("Initialising camera %s\n",name);
  GetAvailableCameras(&totCams);
  if(ncam>totCams){
    printf("error: %d cameras requested, only %d found\n",ncam,(int)totCams);
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
  if(n==2){
    xpos=args[0];//1
    ypos=args[1];//1
  }else{
    printf("Error - args should be 2 in andor: xpos(1),ypos(1)\n");
    camdoFree(camstr);
    *camHandle=NULL;
    return 1;
  }
  if(xpos<1)xpos=1;
  if(ypos<1)ypos=1;
  if(arr->pxlbuftype!='H' || arr->pxlbufsSize!=sizeof(unsigned short)*npxls){
    //need to resize the pxlbufs...
    arr->pxlbufsSize=sizeof(unsigned short)*npxls;
    arr->pxlbuftype='H';
    arr->pxlbufelsize=sizeof(unsigned short);
    tmps=realloc(arr->pxlbufs,arr->pxlbufsSize);
    if(tmps==NULL){
      if(arr->pxlbufs!=NULL)
	free(arr->pxlbufs);
      printf("pxlbuf malloc error in andor.c.\n");
      arr->pxlbufsSize=0;
      free(*camHandle);
      *camHandle=NULL;
      return 1;
    }
    arr->pxlbufs=tmps;
    memset(arr->pxlbufs,0,arr->pxlbufsSize);
  }
  camstr->imgdata=arr->pxlbufs;
  if(*framenoSize<ncam){
    if(*frameno!=NULL)free(*frameno);
    *frameno=malloc(sizeof(unsigned int)*ncam);
    if(*frameno==NULL){
      *framenoSize=0;
      printf("Unable to malloc camframeno\n");
    }else{
      *framenoSize=ncam;
    }
  }
  camstr->userFrameNo=*frameno;
  camstr->setAll=1;
  camstr->npxls=npxls;
  camstr->npxlx=pxlx;
  camstr->npxly=pxly;
  camstr->ncam=ncam;
  printf("TODO: what are sensible defaults, and what should we not set by default?  These ones, we should set to zero and set the Current to zero too\n");
  camstr->temp=-70;
  camstr->triggerMode=1;//7=external exposure mode (bulb), 0=internal,1=external
  camstr->fastExtTrig=1;//1==disable keep clean cycles in triggerMode==1.
  camstr->coolerOn=1;
  camstr->outputType=0;///0==em, 1=conventional
  camstr->hsspeed=0;//0 for fastest.
  camstr->vsspeed=1;//0 for fastest.
  camstr->vsamp=1;//0 for normal, 1-4 for increasing amplitude
  camstr->fanMode=0;//0=full, 1=half, 2=off
  camstr->emAdvanced=1;//1==to allow gains >300
  camstr->emmode=1;//0==0-255, 1==0-4095, 2==linear mode, 3==real em mode
  camstr->emgain=1;
  camstr->outputAmp=0;//0==emccd, 1==ccd
  camstr->preamp=0;//0 to 2 I think.
  camstr->clamp=0;//0 or 1
  camstr->expTime=0.;
  //current settings - unset.
  camstr->tempCurrent=-1;
  camstr->triggerModeCurrent=-2147483647;
  camstr->fastExtTrigCurrent=-2147483647;
  camstr->coolerOnCurrent=-2147483647;
  camstr->outputTypeCurrent=-2147483647;
  camstr->hsspeedCurrent=-2147483647;
  camstr->vsspeedCurrent=-2147483647;
  camstr->vsampCurrent=-2147483647;
  camstr->fanModeCurrent=-2147483647;
  camstr->emAdvancedCurrent=-2147483647;
  camstr->emmodeCurrent=-2147483647;
  camstr->emgainCurrent=-2147483647;
  camstr->outputAmpCurrent=-2147483647;
  camstr->preampCurrent=-2147483647;
  camstr->expTimeCurrent=-1;
  camstr->clampCurrent=0;

  for(i=0; i<ncam; i++){
    camstr->userFrameNo[i]=-1;
  }
  for(i=0;i<ncam;i++){
    at_32 handle;
    struct stat st;
    if(GetCameraHandle(i,&handle)!=DRV_SUCCESS){
      printf("GetCameraHandle %d failed\n",i);
    }
    if(SetCurrentCamera(handle)!=DRV_SUCCESS){
      printf("SetCurrentCamera(%d) failed\n",i);
    }

    if(stat("/root/andor/examples/common",&st)==0){
      printf("Using /root/andor/examples/common/ for initialisation\n");
      if(Initialize("/root/andor/examples/common")!=DRV_SUCCESS){
	printf("Error Initialize cam %d\n",i);
	camdoFree(camstr);
	*camHandle=NULL;
	return 1;
      }
    }else if(stat("/Canary/etc/andor",&st)==0){
      printf("Using /Canary/etc/andor/ for initialisation\n");
      if(Initialize("/Canary/etc/andor")!=DRV_SUCCESS){
	printf("Error Initialize cam %d\n",i);
	camdoFree(camstr);
	*camHandle=NULL;
	return 1;
      }
    }else if(stat("/usr/local/etc/andor",&st)==0){
      printf("Using /usr/local/etc/andor/ for initialisation\n");
      if(Initialize("/usr/local/etc/andor")!=DRV_SUCCESS){
	printf("Error Initialize cam %d\n",i);
	camdoFree(camstr);
	*camHandle=NULL;
	return 1;
      }
    }
    if(SetFPDP(1)!=DRV_SUCCESS){
      printf("SetFPDP error\n");
      return 1;
    }
    if(SetShutter(0,1,0,0)!=DRV_SUCCESS){
      printf("SetShutter error\n");
      return 1;
    }
    if(SetPhotonCounting(0)!=DRV_SUCCESS){
      printf("SetPhotonCounting error\n");
      return 1;
    }
    if(SetFrameTransferMode(1)!=DRV_SUCCESS){
      printf("SetFrameTransferMode error\n");
      return 1;
    }
    if(SetReadMode(4)!=DRV_SUCCESS){
      printf("SetReadMode error\n");
      return 1;
    }
    if(SetImage(1,1,xpos,pxlx[i],ypos,pxly[i])!=DRV_SUCCESS){
      printf("SetImage error\n");
      return 1;
    }
    if(SetAcquisitionMode(5)!=DRV_SUCCESS){//run til abort
      printf("SetAcquisitionMode error\n");
      return 1;
    }
    /*if(SetEMCCDGain(4000)!=DRV_SUCCESS){
      printf("SetEMCCDGain error\n");
      return 1;
      }*/
    if(SetDMAParameters(1,0.0015)!=DRV_SUCCESS){
      printf("SetDMAParameters error\n");
      return 1;
    }
    if(GetAcquisitionTimings(&exp,&acc,&kinetic)!=DRV_SUCCESS){
      printf("GetAcquisitionTimings error\n");
      return 1;
    }
    int noVSSpeeds;
    GetNumberVSSpeeds(&noVSSpeeds);
    int index;
    float speed;
    int noADChan;
    int chan,typ;
    int number;
    for (index = 0; index < noVSSpeeds; index++) {
      GetVSSpeed(index, &speed);
      printf("VSSpeed %d: %g\n",index,speed);
    }
    GetNumberADChannels(&noADChan);
    for (chan = 0; chan < noADChan; chan++) {
      for (typ = 0; typ < 2; typ++) {
	GetNumberHSSpeeds(chan, typ, &number);
	for (index = 0; index < number; index++) {
	  GetHSSpeed(chan, typ, index, &speed);
          printf("HSSpeed for AD%d %s %d: %g\n",chan,typ==0?"EM":"Non-EM",index,speed);
	}
      }
    }

  }
  if(camNewParam(*camHandle,pbuf,thisiter,arr)!=0){
    printf("Error in camOpen->newParam...\n");
    camdoFree(camstr);
    *camHandle=NULL;
    return 1;
  }
  if(StartAcquisition()!=DRV_SUCCESS){
    printf("Error in StartAcquirisition\n");
    camdoFree(camstr);
    *camHandle=NULL;
    return 1;
  }
  camstr->started=1;
  camstr->camOpen=1;
  printf("Camera opened\n");
  return 0;
}


/**
   Close a camera of type name.  Args are passed in the int32 array of size n, and state data is in camHandle, which should be freed and set to NULL before returning.
*/
int camClose(void **camHandle){
  int t;
  CamStruct *camstr=(CamStruct*)(*camHandle);
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
  camdoFree(camstr);
  //free(*camHandle);
  *camHandle=NULL;
  printf("Camera closed\n");
  return 0;
}


/**
   Called when we're starting processing the next frame.
   Just transfer all the frame here.  With the andor, we can't get access to the pixel stream, so we may as well do it now...  
   Called by 1 processing thread each iteration.
*/

int camNewFrameSync(void *camHandle,unsigned int thisiter,double starttime){
  //printf("camNewFrame\n");
  CamStruct *camstr;
  int i;
  unsigned int status=0;
  int offset=0;
  //INT pitch;
  camstr=(CamStruct*)camHandle;
  if(camHandle==NULL){// || camstr->streaming==0){
    //printf("called camNewFrame with camHandle==NULL\n");
    return 1;
  }
  
  for(i=0;i<camstr->ncam;i++){
    at_32 handle;
    unsigned int err=0;
    if ((err|=GetCameraHandle(i, &handle))!=DRV_SUCCESS)
      printf("Failed to get camera handle %d\n",i);
    if((err|=SetCurrentCamera(handle))!=DRV_SUCCESS)
      printf("Failed to set current camera\n");
    if ((err|=WaitForAcquisitionByHandle(handle))!=DRV_SUCCESS)
      printf("Failed to wait for acquisition");
    if((err|=GetMostRecentImage16(&camstr->imgdata[offset],camstr->npxlx[i]*camstr->npxly[i]))!=DRV_SUCCESS)
      printf("GetMostRecentImage16 failed for cam %d\n",i);
    if(err==DRV_SUCCESS)
      camstr->userFrameNo[i]++;
    offset+=camstr->npxlx[i]*camstr->npxly[i];
    status|=err;
  }
  camstr->err=1-(status==DRV_SUCCESS);
  return camstr->err;
}
