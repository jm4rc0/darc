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
   The code here is used to create a shared object library, which can then be swapped around depending on which mirrors/interfaces you have in use, ie you simple rename the mirror file you want to mirror.so (or better, change the soft link), and restart the coremain.

The library is written for a specific mirror configuration - ie in multiple mirror situations, the library is written to handle multiple mirrors, not a single mirror many times.

This is written for the boston micromachines multi DM.

*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include "rtcmirror.h"
#include <time.h>
#include <pthread.h>
#include <usb.h>
#include "darc.h"

//This is from CIUsbLib.h
#define USB_NUM_ACTUATORS_MULTI		160	// Number of words sent in either 128 or 140 actuator modes (MULTI)
#define USB_BYTES_PER_FRAME_MULTI	(USB_NUM_ACTUATORS_MULTI*2)
#define USB_NUM_ACTUATORS_MINI		32	// Number of words sent in either 32 actuator mode (MINI)
#define USB_BYTES_PER_FRAME_MINI	(USB_NUM_ACTUATORS_MINI*2)

//This is from CIUsbShared.h
// Vendor Requests to USB device.
enum
{
	eCIUsbCmndGetFirmwareVer	= 0xF0,
	eCIUsbCmndSetGetRam		= 0xF1,
	eCIUsbCmndSetGetCodeEeprom	= 0xF2,
	eCIUsbCmndResetAll		= 0xF3,
	eCIUsbCmndGetStatusBits		= 0xF4, // bit[0]=FrameErr,  bit[1]=RdHvaE, bit[2]=CableOk, bit[9]=ExEepromPresent
	eCIUsbCmndSetControlBits	= 0xF5, // bit[7]=Set/Reset, bit[1]=Freset, bit[2]=FrameSync, bit[3]=HvEnab
	eCIUsbCmndSetGetCodeExEeprom	= 0xF6,
	eCIUsbCmndSetDac		= 0xF7
};
//And this is from BMC_Mappings.h
#define BMC_USB_VENDOR ((u_int16_t) 0x1781)
#define BMC_USB_MULTIDRIVER ((u_int16_t) 0x0ED8)
#define BMC_USB_MINIDRIVER ((u_int16_t) 0x0ED9)
#define NUM_ACTUATORS	USB_NUM_ACTUATORS_MINI
#define USB_BYTES_PER_FRAME USB_BYTES_PER_FRAME_MINI

#define TIMEOUT 1000
#define BYTESPERLINE 16



typedef enum{
  MIRRORACTINIT,
  MIRRORACTMAPPING,
  MIRRORACTMAX,
  MIRRORACTMIN,
  MIRRORACTOFFSET,
  MIRRORACTOSCARR,
  MIRRORACTOSCPERACT,
  MIRRORACTOSCTIME,
  MIRRORACTPOWER,
  MIRRORACTSCALE,
  MIRRORACTSOURCE,
  MIRRORNACTS,

  //Add more before this line.
  MIRRORNBUFFERVARIABLES//equal to number of entries in the enum
}MIRRORBUFFERVARIABLEINDX;

#define makeParamNames() bufferMakeNames(MIRRORNBUFFERVARIABLES,\
					 "actInit","actMapping","actMax","actMin","actOffset","actOscArr","actOscPerAct","actOscTime","actPower","actScale","actSource", "nacts")



typedef struct{
  int nacts;
  unsigned short *arr;
  unsigned int arrsize;
  int open;
  int err;
  pthread_t threadid;
  pthread_cond_t cond;
  pthread_mutex_t m;
  unsigned int *threadAffinity;
  int threadAffinElSize;
  int threadPriority;
  circBuf *rtcActuatorBuf;
  circBuf *rtcErrorBuf;
  unsigned short *actInit;
  int initLen;
  unsigned short *actMin;
  unsigned short *actMax;
  usb_dev_handle *udev;
  unsigned short dmarr[NUM_ACTUATORS];//the array that gets sent to the DM.
  int *actMapping;
  int actMappingSize;
  int actMappingLen;
  int *actSource;
  float *actScale;
  float *actPower;
  float *actOffset;
  float *oscillateArr;
  int oscillateIters;
  int oscillateArrSize;
  int oscillateSleepTime;
  unsigned short *arrPrev;
  char *paramNames;
  int index[MIRRORNBUFFERVARIABLES];
  void *values[MIRRORNBUFFERVARIABLES];
  char dtype[MIRRORNBUFFERVARIABLES];
  int nbytes[MIRRORNBUFFERVARIABLES];

}MirrorStruct;

void printbytes(char *buffer, int count)
{
  int i;
  
  // printf("Hex Dump follows... 0x%x %d ", buffer, count);
  
  if(count==0) return;
  
  for(i=0;i<count;i++) {
    if(i != 0 && i%BYTESPERLINE == 0)
      printf("\n%08X: ", i);
    
    printf("%02X ", (int)(0x00FF & buffer[i]));
    
  }
  
  printf("\n");
}


int vcmd(usb_dev_handle *udev, int request, int value, int index, int size, char *bytes)
{
  int err;
  
  printf("Vendor command: %02X %04X %04X %5d: \n", request,value,index,size);
  printbytes(bytes,size);	
  
  err = usb_control_msg(udev,USB_TYPE_VENDOR,request,value,index,bytes,size,TIMEOUT)<0;
  if(err) {
    printf("  (err=%d) >>> FAILED TO COMPLETE VENDOR COMMAND!\n",err);
    return(err);
  } 
  
  return(0);
}


void mirrordofree(MirrorStruct *mirstr){
  //int i;
  char string[256];
  int err;
  if(mirstr!=NULL){
    if(mirstr->arr!=NULL)
      free(mirstr->arr);
    if(mirstr->arrPrev!=NULL)
      free(mirstr->arrPrev);
    if(mirstr->udev!=NULL){
      printf("Shutting down dm\n");
      if((err=vcmd(mirstr->udev, eCIUsbCmndSetControlBits, 0, 0x0008, 0, string)))
	printf("Mysterious undoc command 0x0008 failed.  err=%d\n",err);
      printf("Releasing the DM USB interface...\n");
      if (usb_release_interface(mirstr->udev,0)<0)
	printf("Couldn't release interface.\n");
      usb_close (mirstr->udev);
      mirstr->udev=NULL;
    }
    /*for(i=0; i<2; i++){
      if(mirstr->msb[i].actMin!=NULL)free(mirstr->msb[i].actMin);
      if(mirstr->msb[i].actMax!=NULL)free(mirstr->msb[i].actMax);
      if(mirstr->msb[i].actMapping!=NULL)free(mirstr->msb[i].actMapping);
      }*/
    pthread_cond_destroy(&mirstr->cond);
    pthread_mutex_destroy(&mirstr->m);
    free(mirstr);
  }
}

int initDM(MirrorStruct *mirstr){
  int err=0;
  int ret;
  usb_dev_handle *udev;
  char string[256];
  struct usb_bus *bus;
  struct usb_device *dev;
  usb_init();
  usb_find_busses();
  usb_find_devices();
  for (bus = usb_busses; bus && mirstr->udev==NULL; bus = bus->next) {
    for (dev = bus->devices; dev && mirstr->udev==NULL; dev = dev->next) {
      if (dev->descriptor.idVendor == BMC_USB_VENDOR && dev->descriptor.idProduct==BMC_USB_MINIDRIVER) {
	printf("Located a Boston MicroMachines device on USB /dev/bus/usb/%s/%s vendor 0x%04X product 0x%04X.  This is a BMC mini-driver\n",bus->dirname, dev->filename,dev->descriptor.idVendor, dev->descriptor.idProduct);
	if((udev=usb_open(dev))){
	  if(dev->descriptor.iManufacturer){
	    if((ret=usb_get_string_simple(udev, dev->descriptor.iManufacturer, string, sizeof(string)))>0)
	      printf("Manufacturer: %s\n", string);
	    else
	      printf("Unable to fetch manufacturer string - continuing\n");
	  }
	  if (dev->descriptor.iProduct) {
	    if((ret=usb_get_string_simple(udev, dev->descriptor.iProduct, string, sizeof(string)))>0)
	      printf("Product: %s\n", string);
	    else
	      printf("Unable to fetch product string - continuing\n");
	  }
	  usb_set_debug(5);//why 5?
	  if(usb_claim_interface(udev,0)<0){
	    printf("Couldn't claim interface - do you need to run as root or add yourself to the usb group (though I'm not sure that can be done?)?\n");
	    usb_close(udev);
	  }else{
	    string[0] = 0;
	    err = usb_control_msg(udev,0xC0,eCIUsbCmndGetFirmwareVer,0,0,string,0x16,TIMEOUT);
	    printf("Firmware version: <%s>\n",string);
	    if((err=vcmd(udev, eCIUsbCmndSetControlBits, 0, 0x0002, 0, string))!=0){ // de-assert reset
	      printf("Mysterious undoc command 0x0002 failed.  err=%d\n",err);
	    }else if((err = vcmd(udev, eCIUsbCmndSetControlBits, 0, 0x0082, 0, string))!=0){ // assert Reset
	      printf("Mysterious undoc command 0x0082 failed.  err=%d\n",err);
	    }else if((err=vcmd(udev, eCIUsbCmndSetControlBits, 0, 0x0088, 0, string))!=0){ // turn ON HV.
	      printf("Mysterious undoc command 0x0088 failed.  err=%d\n",err);
	    }else{
	      //and if we get here, we've succeeded in opening the dm.
	      mirstr->udev=udev;
	    }
	    if(mirstr->udev==NULL){
	      //haven't succeeded in opening - so turn off HV and close.
	      vcmd(udev,eCIUsbCmndSetControlBits,0,0x0008,0,string);
	      if(usb_release_interface(udev,0)<0)
		printf("Couldn't release interface\n");
	      usb_close(udev);
	    }
	  }
	}
      }else if (dev->descriptor.idVendor == BMC_USB_VENDOR){
	printf("Located BMM with product ID: %d\nBut this is not equal to that requested: %d",dev->descriptor.idProduct,BMC_USB_MINIDRIVER);
      }
    }
  }
  return mirstr->udev==NULL;
}

int mirrorsetThreadAffinity(unsigned int *threadAffinity,int threadPriority,int threadAffinElSize){
  int i;
  cpu_set_t mask;
  int ncpu;
  struct sched_param param;
  ncpu= sysconf(_SC_NPROCESSORS_ONLN);
  CPU_ZERO(&mask);
  for(i=0; i<ncpu && i<threadAffinElSize*32; i++){
    if(((threadAffinity[i/32])>>(i%32))&1){
      CPU_SET(i,&mask);
    }
  }
  //printf("Thread affinity %d\n",threadAffinity&0xff);
  if(sched_setaffinity(0,sizeof(cpu_set_t),&mask))
    printf("Error in sched_setaffinity: %s\n",strerror(errno));
  param.sched_priority=threadPriority;
  if(sched_setparam(0,&param)){
    printf("Error in sched_setparam: %s - probably need to run as root if this is important\n",strerror(errno));
  }
  if(sched_setscheduler(0,SCHED_RR,&param))
    printf("sched_setscheduler: %s - probably need to run as root if this is important\n",strerror(errno));
  if(pthread_setschedparam(pthread_self(),SCHED_RR,&param))
    printf("error in pthread_setschedparam - maybe run as root?\n");
  return 0;
}
inline void dmWrite(MirrorStruct *mirstr,int indx,unsigned short val){
  if(indx>=NUM_ACTUATORS)
    printf("Error setting actuator %d - this doesn't exist\n",indx);
  else//here, we could convert to quadratic for the boston...
    mirstr->dmarr[indx]=val;
}

int dmSend(MirrorStruct *mirstr){
  int err=0,rtval;
  if((rtval=usb_bulk_write(mirstr->udev,2,(char*)mirstr->dmarr,NUM_ACTUATORS*sizeof(unsigned short),TIMEOUT))!=USB_BYTES_PER_FRAME){
    printf("Sending actuators failed - written %d bytes <%s>\n",rtval,usb_strerror());
    err=1;
  }
  return err;

}

/**
   The thread that does the work - copies actuators, and sends to the DAC
*/
void* worker(void *mirstrv){
  MirrorStruct *mirstr=(MirrorStruct*)mirstrv;
  int i,j;
  int val;
  struct timespec tme;
  int nel,skip,step;
  mirrorsetThreadAffinity(mirstr->threadAffinity,mirstr->threadPriority,mirstr->threadAffinElSize);
  pthread_mutex_lock(&mirstr->m);
  if(mirstr->open && mirstr->actInit!=NULL){
    for(i=0; i<mirstr->initLen; i++){
      dmWrite(mirstr,i,mirstr->actInit[i]);
    }
    dmSend(mirstr);
  }
  
  while(mirstr->open){
    pthread_cond_wait(&mirstr->cond,&mirstr->m);//wait for actuators.
    if(mirstr->open){
      //Here, think about adding the option to oscillate the DM to the desired solution.  This would be using a pre-determined array.  The values to put on the DM would be something like:  exp(-t/5)*cos(t)*D/2+acts where t is 0,pi,2pi,3pi,..., and D is acts-acts_-1, i.e. the difference between last and current requested demands.  t could be of dimensions n, or n,nacts where n is the number of steps that you wish to take.  Would also need a sleep time, which would be something like frametime/2/n, allowing the update to be finished within half a frame time.  

      mirstr->err=0;
      if(mirstr->oscillateArr==NULL){
	if(mirstr->actMapping==NULL){
	  for(i=0; i<mirstr->nacts; i++){
	    dmWrite(mirstr,i,mirstr->arr[i]);
	  }
	}else{
	  for(i=0; i<mirstr->actMappingLen; i++){
	    dmWrite(mirstr,mirstr->actMapping[i],mirstr->arr[i]);
	  }
	}
	mirstr->err=dmSend(mirstr);
      }else{//need to oscillate to the solution.
	clock_gettime(CLOCK_REALTIME,&tme);
	nel=mirstr->actMapping==NULL?mirstr->nacts:mirstr->actMappingLen;
	for(j=0;j<mirstr->oscillateIters;j++){
	  if(mirstr->oscillateArrSize==mirstr->oscillateIters){//one only per timestep
	    skip=1;
	    step=0;
	  }else{//a value per actuator per timestep.
	    skip=nel;
	    step=1;
	  }
	  if(mirstr->actMapping==NULL){
	    for(i=0;i<mirstr->nacts;i++){
	      val=(int)(0.5+mirstr->arr[i]+mirstr->oscillateArr[j*skip+step*i]*((int)mirstr->arr[i]-(int)mirstr->arrPrev[i]));
	      if(val>mirstr->actMax[i])
		val=mirstr->actMax[i];
	      if(val<mirstr->actMin[i])
		val=mirstr->actMin[i];
	      dmWrite(mirstr,i,(unsigned short)val);
	    }
	  }else{
	    for(i=0; i<mirstr->actMappingLen; i++){
	      val=(int)(0.5+mirstr->arr[i]+mirstr->oscillateArr[j*skip+step*i]*((int)mirstr->arr[i]-(int)mirstr->arrPrev[i]));
	      if(val>mirstr->actMax[i])
		val=mirstr->actMax[i];
	      if(val<mirstr->actMin[i])
		val=mirstr->actMin[i];
	      dmWrite(mirstr,mirstr->actMapping[i],(unsigned short)val);
	    }
	  }
	  mirstr->err=dmSend(mirstr);
	  //wait before adjusting the mirror slightly.
	  tme.tv_nsec+=mirstr->oscillateSleepTime;
	  if(tme.tv_nsec>999999999){
	    tme.tv_sec++;
	    tme.tv_nsec-=1000000000;
	  }
	  clock_nanosleep(CLOCK_REALTIME,TIMER_ABSTIME,&tme,NULL);
	}
	//Store actuators for next iteration.
	memcpy(mirstr->arrPrev,mirstr->arr,sizeof(unsigned short)*nel);
      }
    }
  }
  pthread_mutex_unlock(&mirstr->m);
  return NULL;
}

/**
   Open a camera of type name.  Args are passed in a float array of size n, which can be cast if necessary.  Any state data is returned in camHandle, which should be NULL if an error arises.
   pxlbuf is the array that should hold the data. The library is free to use the user provided version, or use its own version as necessary (ie a pointer to physical memory or whatever).  It is of size npxls*sizeof(short).
   ncam is number of cameras, which is the length of arrays pxlx and pxly, which contain the dimensions for each camera.
   Name is used if a library can support more than one camera.

*/

int mirrorOpen(char *name,int narg,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **mirrorHandle,int nacts,circBuf *rtcActuatorBuf,unsigned int frameno,unsigned int **mirrorframeno,int *mirrorframenoSize){
  int err;
  MirrorStruct *mirstr;
  char *pn;
  printf("Initialising mirror %s\n",name);
  if((pn=makeParamNames())==NULL){
    printf("Error making paramList - please recode mirrorBMMMulti.c\n");
    *mirrorHandle=NULL;
    return 1;
  }


  if((*mirrorHandle=malloc(sizeof(MirrorStruct)))==NULL){
    printf("couldn't malloc mirrorHandle\n");
    return 1;
  }
  mirstr=(MirrorStruct*)*mirrorHandle;
  memset(mirstr,0,sizeof(MirrorStruct));
  mirstr->paramNames=pn;
  mirstr->nacts=nacts;
  mirstr->rtcErrorBuf=rtcErrorBuf;
  mirstr->rtcActuatorBuf=rtcActuatorBuf;
  mirstr->arrsize=nacts*sizeof(unsigned short);
  if((mirstr->arr=malloc(mirstr->arrsize))==NULL){
    printf("couldn't malloc arr\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  if((mirstr->arrPrev=malloc(mirstr->arrsize))==NULL){
    printf("couldn't malloc arrPrev\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  memset(mirstr->arr,0,mirstr->arrsize);
  memset(mirstr->arrPrev,0,mirstr->arrsize);
  if(narg>2 && narg==2+args[0]){
    mirstr->threadAffinElSize=args[0];
    mirstr->threadPriority=args[1];
    mirstr->threadAffinity=(unsigned int*)&args[2];
  }else{
    printf("wrong number of args - should be Naffin, thread priority, thread affinity[Naffin]\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  if(mirstr->rtcActuatorBuf!=NULL && mirstr->rtcActuatorBuf->datasize!=mirstr->nacts*sizeof(unsigned short)){
    if(circReshape(mirstr->rtcActuatorBuf,1,&mirstr->nacts,'H')!=0){
      printf("Error reshaping rtcActuatorBuf\n");
    }
  }
  if(pthread_cond_init(&mirstr->cond,NULL)!=0){
    printf("Error initialising thread condition variable\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  //maybe think about having one per camera???
  if(pthread_mutex_init(&mirstr->m,NULL)!=0){
    printf("Error initialising mutex variable\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  //initialise acquisition session
  if(initDM(mirstr)){//failed...
    printf("Failed to initBMM\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  mirstr->open=1;
  if((err=mirrorNewParam(*mirrorHandle,pbuf,frameno,arr))){
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  pthread_create(&mirstr->threadid,NULL,worker,mirstr);
  return 0;
}

/**
   Close a camera of type name.  Args are passed in the float array of size n, and state data is in camHandle, which should be freed and set to NULL before returning.
*/
int mirrorClose(void **mirrorHandle){
  MirrorStruct *mirstr=(MirrorStruct*)*mirrorHandle;
  printf("Closing mirror\n");
  if(mirstr!=NULL){
    pthread_mutex_lock(&mirstr->m);
    if(mirstr->paramNames!=NULL)
      free(mirstr->paramNames);
    mirstr->open=0;
    pthread_cond_signal(&mirstr->cond);//wake the thread.
    pthread_mutex_unlock(&mirstr->m);
    printf("Joining mirror worker thread\n");
    pthread_join(mirstr->threadid,NULL);//wait for worker thread to complete
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
  }
  printf("Mirror closed\n");
  return 0;
}
int mirrorSend(void *mirrorHandle,int n,float *data,unsigned int frameno,double timestamp,int err,int writeCirc){
  MirrorStruct *mirstr=(MirrorStruct*)mirrorHandle;
  int nclipped=0;
  int intDMCommand;
  int i;
  //MirrorStructBuffered *msb;
  if(err==0 && mirstr!=NULL && mirstr->open==1){
    //printf("Sending %d values to mirror\n",n);
    pthread_mutex_lock(&mirstr->m);
    err=mirstr->err;//get the error from the last time.  Even if there was an error, need to send new actuators, to wake up the thread... incase the error has gone away.
    //First, copy actuators.  Note, should n==mirstr->nacts.
    //Note, need to convert to uint16...
    if(mirstr->actMapping==NULL){
      for(i=0; i<mirstr->nacts; i++){
	if(mirstr->actScale!=NULL)
	  data[i]*=mirstr->actScale[i];
	if(mirstr->actOffset!=NULL)
	  data[i]*=mirstr->actOffset[i];
	if(mirstr->actPower!=NULL)
	  data[i]=powf(data[i],mirstr->actPower[i]);
	intDMCommand=(int)(data[i]+0.5);
	mirstr->arr[i]=(unsigned short)intDMCommand;
	if(intDMCommand<mirstr->actMin[i]){
	  nclipped++;
	  mirstr->arr[i]=mirstr->actMin[i];
	}
	if(intDMCommand>mirstr->actMax[i]){
	  nclipped++;
	  mirstr->arr[i]=mirstr->actMax[i];
	}
      }
    }else{//actMapping is specified...
      if(mirstr->actSource==NULL){
	if(mirstr->actScale==NULL){
	  if(mirstr->actOffset==NULL){
	    for(i=0; i<mirstr->actMappingLen; i++){
	      intDMCommand=(int)(data[i]+0.5);
	      mirstr->arr[i]=(unsigned short)intDMCommand;
	      if(intDMCommand<mirstr->actMin[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMin[i];
	      }
	      if(intDMCommand>mirstr->actMax[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMax[i];
	      }
	    }
	  }else{//actoffset defined.
	    for(i=0; i<mirstr->actMappingLen; i++){
	      intDMCommand=(int)(data[i]+mirstr->actOffset[i]+0.5);
	      mirstr->arr[i]=(unsigned short)intDMCommand;
	      if(intDMCommand<mirstr->actMin[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMin[i];
	      }
	      if(intDMCommand>mirstr->actMax[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMax[i];
	      }
	    }
	  }
	}else{//actscale defined
	  if(mirstr->actOffset==NULL){
	    for(i=0; i<mirstr->actMappingLen; i++){
	      intDMCommand=(int)(data[i]*mirstr->actScale[i]+0.5);
	      mirstr->arr[i]=(unsigned short)intDMCommand;
	      if(intDMCommand<mirstr->actMin[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMin[i];
	      }
	      if(intDMCommand>mirstr->actMax[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMax[i];
	      }
	    }
	  }else{//actScale and actoffset defined
	    for(i=0; i<mirstr->actMappingLen; i++){
	      intDMCommand=(int)(data[i]*mirstr->actScale[i]+mirstr->actOffset[i]+0.5);
	      mirstr->arr[i]=(unsigned short)intDMCommand;
	      if(intDMCommand<mirstr->actMin[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMin[i];
	      }
	      if(intDMCommand>mirstr->actMax[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMax[i];
	      }
	    }
	  }
	}
      }else{//actSource defined
	if(mirstr->actScale==NULL){
	  if(mirstr->actOffset==NULL){
	    for(i=0; i<mirstr->actMappingLen; i++){
	      intDMCommand=(int)(data[mirstr->actSource[i]]+0.5);
	      mirstr->arr[i]=(unsigned short)intDMCommand;
	      if(intDMCommand<mirstr->actMin[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMin[i];
	      }
	      if(intDMCommand>mirstr->actMax[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMax[i];
	      }
	    }
	  }else{//actSource and actoffset defined.
	    for(i=0; i<mirstr->actMappingLen; i++){
	      intDMCommand=(int)(data[mirstr->actSource[i]]+mirstr->actOffset[i]+0.5);
	      mirstr->arr[i]=(unsigned short)intDMCommand;
	      if(intDMCommand<mirstr->actMin[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMin[i];
	      }
	      if(intDMCommand>mirstr->actMax[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMax[i];
	      }
	    }
	  }
	}else{//actSource and actscale defined
	  if(mirstr->actOffset==NULL){
	    for(i=0; i<mirstr->actMappingLen; i++){
	      intDMCommand=(int)(data[mirstr->actSource[i]]*mirstr->actScale[i]+0.5);
	      mirstr->arr[i]=(unsigned short)intDMCommand;
	      if(intDMCommand<mirstr->actMin[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMin[i];
	      }
	      if(intDMCommand>mirstr->actMax[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMax[i];
	      }
	    }
	  }else{//actSource and actScale and actoffset defined
	    for(i=0; i<mirstr->actMappingLen; i++){
	      intDMCommand=(int)(data[mirstr->actSource[i]]*mirstr->actScale[i]+mirstr->actOffset[i]+0.5);
	      mirstr->arr[i]=(unsigned short)intDMCommand;
	      if(intDMCommand<mirstr->actMin[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMin[i];
	      }
	      if(intDMCommand>mirstr->actMax[i]){
		nclipped++;
		mirstr->arr[i]=mirstr->actMax[i];
	      }
	    }
	  }
	}
      }
    }
    //memcpy(mirstr->arr,data,sizeof(unsigned short)*mirstr->nacts);
    //Wake up the thread.
    pthread_cond_signal(&mirstr->cond);
    pthread_mutex_unlock(&mirstr->m);
    if(writeCirc)
      circAddForce(mirstr->rtcActuatorBuf,mirstr->arr,timestamp,frameno);
  }else{
    err=1;
  }
  if(err)
    return -1;
  return nclipped;
}
/**
   This is called by a main processing thread - asynchronously with mirrorSend.
*/

int mirrorNewParam(void *mirrorHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr){
  MirrorStruct *mirstr=(MirrorStruct*)mirrorHandle;
  int err=0;
  //int got=0;
  int dim;
  //int bufno;
  //MirrorStructBuffered *msb;
  //int *indx=mirstr->bufindx;
  //MIRRORBUFFERVARIABLEINDX i;
  int j=0;
  int nfound;
  int *indx=mirstr->index;
  void **values=mirstr->values;
  char *dtype=mirstr->dtype;
  int *nbytes=mirstr->nbytes;
  if(mirstr==NULL || mirstr->open==0){
    printf("Mirror not open\n");
    return 1;
  }
  //bufno=1-mirstr->buf;
  //msb=&mirstr->msb[bufno];

  nfound=bufferGetIndex(pbuf,MIRRORNBUFFERVARIABLES,mirstr->paramNames,indx,values,dtype,nbytes);
  if(nfound!=MIRRORNBUFFERVARIABLES){
    for(j=0; j<MIRRORNBUFFERVARIABLES; j++){
      if(indx[j]<0){
	printf("Missing %16s\n",&mirstr->paramNames[j*16]);
	if(j!=MIRRORACTOSCARR && j!=MIRRORACTOSCPERACT && j!=MIRRORACTOSCTIME){
	  writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"Error in mirror parameter buffer: %16s",&mirstr->paramNames[j*16]);
	  err=-1;
	}
      }
    }
  }
  if(err==0){
    pthread_mutex_lock(&mirstr->m);
    if(dtype[MIRRORNACTS]=='i' && nbytes[MIRRORNACTS]==sizeof(int)){
      if(mirstr->nacts!=*((int*)values[MIRRORNACTS])){
	printf("Error - nacts changed - please close and reopen mirror library\n");
	err=1;
      }
    }else{
      printf("mirrornacts error\n");
      writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"mirrornacts error");
      err=1;
    }
    if(nbytes[MIRRORACTMAPPING]==0){
      mirstr->actMapping=NULL;
    }else if(dtype[MIRRORACTMAPPING]=='i' && nbytes[MIRRORACTMAPPING]%sizeof(int)==0){
      mirstr->actMappingLen=nbytes[MIRRORACTMAPPING]/sizeof(int);
      mirstr->actMapping=(int*)values[MIRRORACTMAPPING];
      if(mirstr->arrsize<nbytes[MIRRORACTMAPPING]){
	if(mirstr->arr!=NULL) 
	  free(mirstr->arr);
	if((mirstr->arr=malloc(nbytes[MIRRORACTMAPPING]))==NULL){
	  printf("Error allocating mirstr->arr\n");
	  err=1;
	  mirstr->arrsize=0;
	}else{
	  mirstr->arrsize=nbytes[MIRRORACTMAPPING];
	  memset(mirstr->arr,0,nbytes[MIRRORACTMAPPING]);
	}
	if(mirstr->arrPrev!=NULL) 
	  free(mirstr->arrPrev);
	if((mirstr->arrPrev=malloc(nbytes[MIRRORACTMAPPING]))==NULL){
	  printf("Error allocating mirstr->arrPrev\n");
	  err=1;
	  mirstr->arrsize=0;
	}else{
	  mirstr->arrsize=nbytes[MIRRORACTMAPPING];
	  memset(mirstr->arrPrev,0,nbytes[MIRRORACTMAPPING]);
	}

      }
    }else{
      printf("Warning - bad actuator mapping\n");
      mirstr->actMapping=NULL;
    }

    dim=mirstr->actMapping==NULL?mirstr->nacts:mirstr->actMappingLen;
    if(mirstr->rtcActuatorBuf!=NULL && mirstr->rtcActuatorBuf->datasize!=dim*sizeof(unsigned short)){
      if(circReshape(mirstr->rtcActuatorBuf,1,&dim,'H')!=0){
	printf("Error reshaping rtcActuatorBuf\n");
      }
    }
    if(nbytes[MIRRORACTSOURCE]==0){
      mirstr->actSource=NULL;
    }else if(nbytes[MIRRORACTSOURCE]==dim*sizeof(int) && dtype[MIRRORACTSOURCE]=='i'){
      mirstr->actSource=(int*)values[MIRRORACTSOURCE];
    }else{
      printf("actSource wrong\n");
      mirstr->actSource=NULL;
    }
    if(nbytes[MIRRORACTSCALE]==0){
      mirstr->actScale=NULL;
    }else if(nbytes[MIRRORACTSCALE]==dim*sizeof(float) && dtype[MIRRORACTSCALE]=='f'){
      mirstr->actScale=(float*)values[MIRRORACTSCALE];
    }else{
      printf("actScale wrong\n");
      mirstr->actScale=NULL;
    }
    if(nbytes[MIRRORACTPOWER]==0){
      mirstr->actPower=NULL;
    }else if(nbytes[MIRRORACTPOWER]==dim*sizeof(float) && dtype[MIRRORACTPOWER]=='f'){
      mirstr->actPower=(float*)values[MIRRORACTPOWER];
    }else{
      printf("actPower wrong\n");
      mirstr->actPower=NULL;
    }

    if(nbytes[MIRRORACTOFFSET]==0){
      mirstr->actOffset=NULL;
    }else if(nbytes[MIRRORACTOFFSET]==dim*sizeof(float) && dtype[MIRRORACTOFFSET]=='f'){
      mirstr->actOffset=(float*)values[MIRRORACTOFFSET];
    }else{
      printf("actOffset wrong\n");
      mirstr->actOffset=NULL;
    }
    if(dtype[MIRRORACTMIN]=='H' && nbytes[MIRRORACTMIN]==sizeof(unsigned short)*dim){
      mirstr->actMin=(unsigned short*)values[MIRRORACTMIN];
    }else{
      printf("mirrorActMin error\n");
      writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"mirrorActMin error");
      err=1;
    }
    if(dtype[MIRRORACTMAX]=='H' && nbytes[MIRRORACTMAX]==sizeof(unsigned short)*dim){
      mirstr->actMax=(unsigned short*)values[MIRRORACTMAX];
    }else{
      printf("mirrorActMax error\n");
      writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"mirrorActMax error");
      err=1;
    }
    if(dtype[MIRRORACTINIT]=='H' && nbytes[MIRRORACTINIT]%sizeof(unsigned short)==0){
      mirstr->actInit=(unsigned short*)values[MIRRORACTINIT];
      mirstr->initLen=nbytes[MIRRORACTINIT]/sizeof(unsigned short);
    }else if(nbytes[MIRRORACTINIT]!=0){
      printf("actInit error\n");
      writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"actInit error");
      err=1;
      mirstr->actInit=NULL;
      mirstr->initLen=0;
    }else{
      mirstr->actInit=NULL;
      mirstr->initLen=0;
    }
    if(indx[MIRRORACTOSCARR]>=0){
      if(dtype[MIRRORACTOSCARR]=='f'){//is it 1D or 2D?
	if(nbytes[MIRRORACTOSCARR]%(dim*sizeof(float))==0){//multiple of nacts, so probably 2D
	  if(indx[MIRRORACTOSCPERACT]>=0 && dtype[MIRRORACTOSCPERACT]=='i' && nbytes[MIRRORACTOSCPERACT]==sizeof(int) && *((int*)values[MIRRORACTOSCPERACT])==1){//2D
	    mirstr->oscillateArr=(float*)values[MIRRORACTOSCARR];
	    mirstr->oscillateIters=nbytes[MIRRORACTOSCARR]/sizeof(float)/dim;
	    mirstr->oscillateArrSize=nbytes[MIRRORACTOSCARR]/sizeof(float);
	  }else{//1D
	    mirstr->oscillateArr=(float*)values[MIRRORACTOSCARR];
	    mirstr->oscillateIters=nbytes[MIRRORACTOSCARR]/sizeof(float);
	    mirstr->oscillateArrSize=nbytes[MIRRORACTOSCARR]/sizeof(float);
	  }
	}else if(nbytes[MIRRORACTOSCARR]%sizeof(float)==0){//1D
	  mirstr->oscillateArr=(float*)values[MIRRORACTOSCARR];
	  mirstr->oscillateIters=nbytes[MIRRORACTOSCARR]/sizeof(float);
	  mirstr->oscillateArrSize=nbytes[MIRRORACTOSCARR]/sizeof(float);
	}else{//wrong shape
	  printf("actOscArr error\n");
	  writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"actOscArr error");
	  mirstr->oscillateArr=NULL;
	  err=1;
	}
      }else{
	printf("actOscArr error\n");
	writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"actOscArr error");
	mirstr->oscillateArr=NULL;
	err=1;
      }
    }else{
      mirstr->oscillateArr=NULL;
    }
    if(indx[MIRRORACTOSCTIME]>=0){
      if(dtype[MIRRORACTOSCTIME]=='i' && nbytes[MIRRORACTOSCTIME]==sizeof(int)){
	mirstr->oscillateSleepTime=*((int*)values[MIRRORACTOSCTIME]);
      }else{
	printf("actOscTime error\n");
	writeErrorVA(mirstr->rtcErrorBuf,-1,frameno,"actOscTime error");
	mirstr->oscillateSleepTime=0;
	err=1;
      }	
    }else{
      mirstr->oscillateSleepTime=0;
    }
    pthread_mutex_unlock(&mirstr->m);
  }
  
  //mirstr->swap=1;
  return err;
}
