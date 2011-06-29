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
DARC mirror module for driving laser launch system mirrors on WHT.

Here, the worker thread is asynchronous.  i.e. it may take a long time for these  "mirrors" to be updated, but in which time, the image processing etc can continue for alignment process - but then additional slope measurements are ignored.

*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <pthread.h>
#include <termios.h>
#include <math.h>
#include "rtcmirror.h"
#include "darc.h"


#define MAXMOVE 200
#define MOVESTEPTIME 0.005

typedef enum{
  MIRRORGETPOS,
  MIRRORMIDRANGE,
  MIRRORNACTS,
  MIRRORRESET,
  MIRRORUPDATE,
  //Add more before this line.
  MIRRORNBUFFERVARIABLES//equal to number of entries in the enum
}MIRRORBUFFERVARIABLEINDX;

#define makeParamNames() bufferMakeNames(MIRRORNBUFFERVARIABLES,\
					 "mirrorGetPos","mirrorMidRange","nacts","mirrorReset","mirrorUpdate")



typedef struct{
  int nacts;
  int open;
  int err;
  pthread_t threadid;
  pthread_cond_t cond;
  pthread_mutex_t m;
  circBuf *rtcActuatorBuf;
  circBuf *rtcErrorBuf;
  char *paramNames;
  int index[MIRRORNBUFFERVARIABLES];
  void *values[MIRRORNBUFFERVARIABLES];
  char dtype[MIRRORNBUFFERVARIABLES];
  int nbytes[MIRRORNBUFFERVARIABLES];
  int fd;
  int doMidrange;
  int getMirrorPosition;
  int updateMirror;
  int resetMirror;
  int demandsUpdated;
  char retbuf[128];
  char *devname;
  int *acts;
  int *demands;
  int minit;
  int cinit;
}MirrorStruct;



/**
   Free mirror/memory/sl240
*/
void mirrordofree(MirrorStruct *mirstr){
  //int i;
  if(mirstr!=NULL){
    if(mirstr->paramNames!=NULL)
      free(mirstr->paramNames);
    if(mirstr->cinit)
      pthread_cond_destroy(&mirstr->cond);
    if(mirstr->minit)
      pthread_mutex_destroy(&mirstr->m);
    if(mirstr->devname!=NULL)
      free(mirstr->devname);
    if(mirstr->demands!=NULL)
      free(mirstr->demands);
    if(mirstr->acts!=NULL)
      free(mirstr->acts);
    free(mirstr);
  }
}



char *sendCommand(MirrorStruct *mirstr,char *command,float wait,...){
  //wait in seconds.
  va_list ap;
  char *tmp,*tmp2;
  char *retbuf=mirstr->retbuf;
  int l,n;
  fd_set input;
  struct timeval timeout;
  timeout.tv_sec=(int)wait;
  timeout.tv_usec=(int)((wait-timeout.tv_sec)*1000000);
  va_start(ap,wait);
  if((l=vasprintf(&tmp,command,ap))>0){
    if((l=asprintf(&tmp2,"%s\r\n",tmp))>0){
      printf("Writing: %s\\r\\n\n",tmp);
      if((n=write(mirstr->fd,tmp2,l))!=l)
	printf("Failed to write (n=%d)\n",n);
      free(tmp2);
    }else{
      printf("Error in asprintf for %s\n",tmp);
    }
    free(tmp);
  }else{//error doing formatting... just print the raw string.
    printf("Error formatting command %s\n",command);
  }
  va_end(ap);
  FD_ZERO(&input);
  FD_SET(mirstr->fd,&input);
  retbuf[0]='\0';
  n=select(mirstr->fd+1,&input,NULL,NULL,&timeout);
  if(n<0)
    printf("error in select\n");
  else if(n>0){
    if(FD_ISSET(mirstr->fd,&input)){
      n=read(mirstr->fd,retbuf,128);
      printf("Read %d chars: %s\n",n,retbuf);
    }
  }else{//timeout
  }
  return retbuf;
}

int waitTimeout(MirrorStruct *mirstr,int channel,float timeout){
  //sendCommand(mirStr,command,0.,...) should be called first.
  //Channel should be 1 or 2.
  float totalTime=0;
  float pollTime=timeout/100.;
  char *status;
  usleep(500000);
  while(totalTime<timeout){
    status=sendCommand(mirstr,"%dTS",pollTime,channel);
    printf("Status return from %dTS gives: %s\n",channel,status);
    if(strcmp("TS0\r\n",&status[1])==0){
      printf("Channel has stopped moving\n");
      //because we sometimes have 1s pauses in middle of an operation, need to check a couple of times.
      sleep(1);
      status=sendCommand(mirstr,"%dTS",0.1,channel);
      totalTime+=1.1;
      if(strcmp("TS0\r\n",&status[1])==0){
	printf("Channel still not moving - finished\n");
	break;
      }
    }
    totalTime+=pollTime;
  }
  return 0;
}




int openLLSMirror(MirrorStruct *mirstr){
  char buf[80];
  fd_set         input;
  struct timeval timeout;
  int rt=0;
  int n;
  int gotdata=1;
 struct termios options;
  //Open modem device for reading and writing and not as controlling tty
  //because we don't want to get killed if linenoise sends CTRL-C.
  if((mirstr->fd=open(mirstr->devname, O_RDWR | O_NOCTTY /*| O_NDELAY*/ ))<0){
    printf("Failed to open USB driver for mirror: %s\n",strerror(errno));
    return 1;
  }
  fcntl(mirstr->fd, F_SETFL, 0);
  // Get the current options for the port...
  tcgetattr(mirstr->fd, &options);
  cfsetispeed(&options, B921600);
  cfsetospeed(&options, B921600);
  // Enable the receiver and set local mode...
  options.c_cflag|=B921600;//set baud rate
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~CSIZE; // Mask the character size bits 
  options.c_cflag |= CS8;    // Select 8 data bits 
 
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag |= CSTOPB;//1 stop bit
  //options.c_cflag |= CRTSCTS;  
  // Set the new options for the port...
  //options.c_lflag |= (ICANON | ECHO | ECHOE);
  tcsetattr(mirstr->fd, TCSANOW, &options);
  //do a few resets
  if((n=write(mirstr->fd,"RS\r\n",4))!=4)
    printf("RS\\r\\n failed\n");
  usleep(100000);
  if((n=write(mirstr->fd,"RS\r\n",4))!=4)
    printf("RS\\r\\n failed\n");
  usleep(100000);
  if((n=write(mirstr->fd,"RS\r\n",4))!=4)
    printf("RS\\r\\n failed\n");
  sleep(1);//wait for it to come out of reset.
  //and put into remote mode
  if((n=write(mirstr->fd,"MR\r\n",4))!=4)
    printf("MR\\r\\n failed\n");
  usleep(10000);
  //Now see if we can get a response...
  if((n=write(mirstr->fd,"CC1\r\n",5))!=5)
    printf("CC1\\r\\n failed\n");
  usleep(10000);
  if((n=write(mirstr->fd,"1TS\r\n",5))!=5)
    printf("1TS\\r\\n failed: %d\n",n);
  usleep(10000);
  timeout.tv_sec  = 2;
  timeout.tv_usec = 0;
  while(gotdata){
    gotdata=0;
    FD_ZERO(&input);
    FD_SET(mirstr->fd, &input);
    n=select(mirstr->fd+1, &input, NULL, NULL, &timeout);
    timeout.tv_sec=0;//no timeout for next reads - flush buffer only.
    timeout.tv_usec=0;
    if(n<0){
      printf("select failed: %s",strerror(errno));
      rt=1;
    }else if(n==0){
      printf("TIMEOUT waiting for motors to return status\n");
      rt=1;
    }else{
      if(FD_ISSET(mirstr->fd,&input)){
	gotdata=1;
	n=read(mirstr->fd,buf,80);
	buf[n]='\0';
	printf("Read %d chars: %s\n",n,buf);
      }
    }
  }
  return rt;
}


int mirrorAbsMove(MirrorStruct *mirstr,int pos1,int pos2,int pos3,int pos4){
  sendCommand(mirstr,"CC1",0.2);
  sendCommand(mirstr,"1PA%d",0,pos1);
  waitTimeout(mirstr,1,60);
  sendCommand(mirstr,"2PA%d",0,pos2);
  waitTimeout(mirstr,2,60);
  sendCommand(mirstr,"CC2",0.2);
  sendCommand(mirstr,"1PA%d",0,pos3);
  waitTimeout(mirstr,1,60);
  sendCommand(mirstr,"2PA%d",0,pos4);
  waitTimeout(mirstr,2,60);
  return 0;
}

/**
   The thread that does the work - copies actuators, and sends to the DAC
*/
void* worker(void *mirstrv){
  MirrorStruct *mirstr=(MirrorStruct*)mirstrv;
  int i,max,j,val;
  float tmove;
  int nchunks;
  
  while(mirstr->open){
    //make a copy of current desired actuators
    pthread_mutex_lock(&mirstr->m);
    if(mirstr->doMidrange){
      if(mirstr->doMidrange<0)
	mirstr->doMidrange++;
      pthread_mutex_unlock(&mirstr->m);
      //send to middle
      mirrorAbsMove(mirstr,500,500,500,500);
      //and zero the counters.
      sendCommand(mirstr,"CC1",0.2);
      sendCommand(mirstr,"1ZP",0.2);
      sendCommand(mirstr,"2ZP",0.2);
      sendCommand(mirstr,"CC2",0.2);
      sendCommand(mirstr,"1ZP",0.2);
      sendCommand(mirstr,"2ZP",0.2);
    }else if(mirstr->getMirrorPosition){
      if(mirstr->getMirrorPosition<0)
	mirstr->getMirrorPosition++;
      pthread_mutex_unlock(&mirstr->m);
      sendCommand(mirstr,"CC1",0.2);
      printf("Mirror positions:\n");
      printf("%s\n",sendCommand(mirstr,"1TP",0.2));
      printf("%s\n",sendCommand(mirstr,"2TP",0.2));
      sendCommand(mirstr,"CC2",0.2);
      printf("%s\n",sendCommand(mirstr,"1TP",0.2));
      printf("%s\n",sendCommand(mirstr,"2TP",0.2));
    }else if(mirstr->resetMirror){//sometimes gets in a state, needs resetting a few times.
      if(mirstr->resetMirror<0)
	mirstr->resetMirror++;
      pthread_mutex_unlock(&mirstr->m);
      sendCommand(mirstr,"RS",1);
      sendCommand(mirstr,"RS",1);
      sendCommand(mirstr,"RS",1);
      sendCommand(mirstr,"MR",1);
    }else if(mirstr->updateMirror && mirstr->demandsUpdated){
      if(mirstr->updateMirror<0)
	mirstr->updateMirror++;
      mirstr->demandsUpdated=0;
      memcpy(mirstr->acts,mirstr->demands,sizeof(int)*mirstr->nacts);
      pthread_mutex_unlock(&mirstr->m);
      //now operate on the actuators...
      //First, find the max requested move.
      max=0;
      for(i=0;i<mirstr->nacts;i++){
	if(abs(mirstr->acts[i])>max)
	  max=abs(mirstr->acts[i]);
      }
      //Now do it in chunks
      nchunks=(max+MAXMOVE-1)/MAXMOVE;
      for(i=0;i<nchunks;i++){
	for(j=0;j<mirstr->nacts;j++){
	  val=mirstr->acts[j]/(nchunks-i);
	  mirstr->acts[i]-=val;
	  if(j%2==0)//change to correct channel
	    sendCommand(mirstr,"CC%d",0.2,j/2+1);
	  tmove=MOVESTEPTIME*abs(val);
	  sendCommand(mirstr,"%dPR%d",(tmove<0.01?0.01:tmove),j%2+1,val);
	}
      }
    }else{
      if(mirstr->open){
	printf("mirrorLLS - nothing to do - waiting for signal\n");
	pthread_cond_wait(&mirstr->cond,&mirstr->m);
      }
      pthread_mutex_unlock(&mirstr->m);
    }
  }
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
    printf("Error making paramList - please recode mirrorPdAO32.c\n");
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
  mirstr->demands=malloc(sizeof(int)*mirstr->nacts);
  mirstr->acts=malloc(sizeof(int)*mirstr->nacts);
  if(narg>2)
    mirstr->devname=strdup((char*)args);
  else
    mirstr->devname=strdup("/dev/ttyUSB4");
  printf("Using device %s\n",mirstr->devname);
  if(mirstr->rtcActuatorBuf!=NULL && mirstr->rtcActuatorBuf->datasize!=mirstr->nacts*sizeof(int)){
    if(circReshape(mirstr->rtcActuatorBuf,1,&mirstr->nacts,'i')!=0){
      printf("Error reshaping rtcActuatorBuf\n");
    }
  }
  if(pthread_cond_init(&mirstr->cond,NULL)!=0){
    printf("Error initialising thread condition variable\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  mirstr->cinit=1;
  if(pthread_mutex_init(&mirstr->m,NULL)!=0){
    printf("Error initialising mutex variable\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }
  mirstr->minit=1;
  if((err=mirrorNewParam(*mirrorHandle,pbuf,frameno,arr))){
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }

  if(openLLSMirror(mirstr)){
    printf("Error opening mirror\n");
    mirrordofree(mirstr);
    mirrorHandle=NULL;
    return 1;
  }else{
    mirstr->open=1;
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
    if(mirstr->open)
      close(mirstr->fd);
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
  int i;
  //MirrorStructBuffered *msb;
  if(err==0 && mirstr!=NULL && mirstr->open==1){
    //printf("Sending %d values to mirror\n",n);
    pthread_mutex_lock(&mirstr->m);
    mirstr->demandsUpdated=1;
    for(i=0;i<mirstr->nacts;i++){
      mirstr->demands[i]=(int)roundf(data[i]);
    }
    //wake up the thread...
    pthread_cond_signal(&mirstr->cond);
    pthread_mutex_unlock(&mirstr->m);
    if(writeCirc)
      circAddForce(mirstr->rtcActuatorBuf,mirstr->demands,timestamp,frameno);
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
  if(indx[MIRRORNACTS]<0){
    printf("Error - nacts not found\n");
    err=-1;
  }else{
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
  }    
  pthread_mutex_lock(&mirstr->m);
  if(indx[MIRRORGETPOS]>=0 && dtype[MIRRORGETPOS]=='i' && nbytes[MIRRORGETPOS]==sizeof(int)){
    mirstr->getMirrorPosition=*((int*)values[MIRRORGETPOS]);
  }else{
    printf("no mirrorGetPos - continuing\n");
  }
  if(indx[MIRRORMIDRANGE]>=0 && dtype[MIRRORMIDRANGE]=='i' && nbytes[MIRRORMIDRANGE]==sizeof(int)){
    mirstr->doMidrange=*((int*)values[MIRRORMIDRANGE]);
  }else{
    printf("no mirrorMidRange - continuing\n");
  }
  if(indx[MIRRORUPDATE]>=0 && dtype[MIRRORUPDATE]=='i' && nbytes[MIRRORUPDATE]==sizeof(int)){
    mirstr->updateMirror=*((int*)values[MIRRORUPDATE]);
  }else{
    printf("no mirrorUpdate - continuing\n");
  }
  if(indx[MIRRORRESET]>=0 && dtype[MIRRORRESET]=='i' && nbytes[MIRRORRESET]==sizeof(int)){
    mirstr->resetMirror=*((int*)values[MIRRORRESET]);
  }else{
    printf("no mirrorReset - continuing\n");
  }
  pthread_cond_signal(&mirstr->cond);
  pthread_mutex_unlock(&mirstr->m);

  
  //mirstr->swap=1;
  return err;
}
