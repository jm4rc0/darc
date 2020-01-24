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

This interface is for the ESO RTDNP camera protocol (e.g. for the LVSM), which is in turn build upon the ESO mudpi protocol.

Problems with RTDNP: No way of getting number of bytes per pixel from
the data, except by inspecting the packet size (which would be wrong
if a MTU is mis-configured).


*/
//typedef unsigned int uint32;
//#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>//setsockopt
#include <sys/socket.h>//setsockopt
#include <netinet/in.h>//struct ip_mreq
//#include <unistd.h>
#include <pthread.h>
#include "darc.h"
#include "rtccamera.h"
#include "qsort.h"
#include "buffer.h"
#include <endian.h>
#include <sched.h>

#include <arpa/inet.h>
#include <netinet/if_ether.h>
#include <netinet/ip.h>
#include <netinet/udp.h>


#include <linux/if_packet.h>
#include <net/ethernet.h>

#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)

#define NBUF 4


/**
   The struct to hold info.
*/


typedef struct{
  int ncam;//no of cameras
  int npxls;//number of pixels in the frame (total for all cameras)
  //int pxlsRequested;//number of pixels requested by DMA for this frame
  darc_mutex_t *camMutex;
  darc_mutex_t *camMutexWorker;
  darc_cond_t *camCond;//sync between main RTC (used with caMutexWorker)
  darc_cond_t *camCond2;//sync between main RTC (used with camMutex)
  int *blocksize;//number of bytes to transfer per DMA;
  int open;//set by RTC if the camera is open
  volatile int *newframe;
  unsigned int thisiter;
  char *imgdata;//interpret as uint8 or uint16 depending on mode...
  volatile int *pxlsTransferred;//number of pixels copied into the RTC memory.
  unsigned int *userFrameNo;//pointer to the RTC frame number... to be updated for new frame.
  void *thrStruct;//pointer to an array of threadStructs.
  int *npxlsArr;
  volatile int *npxlsArrCum;
  int *threadPriority;
  unsigned int *threadAffinity;
  int threadAffinElSize;
  char *paramNames;
  int *index;
  int *nbytes;
  void **values;
  char *dtype;
  int *pxlx;
  int *pxly;
  circBuf *rtcErrorBuf;
  volatile int *frameReady;
  struct timespec *timestamp;
  int recordTimestamp;//option to use timestamp of last pixel arriving rather than frame number.
  int *reorder;//is pixel reordering required, and if so, which pattern?
  int **reorderBuf;//pixels for reordering.
  int *reorderIndx;
  int *reorderno;
  int nReorders;
  volatile int *camErr;//NBUF*ncam
  char **DMAbuf;
  volatile int *mostRecentFilled;
  volatile int *currentFilling;
  volatile int *rtcReading;
  int bytespp;//bytes per pixel of the image passed to darc.
  int *bytesPerPixel;//array[ncam], bytes per pixel of the input images.
  int *senderAddress;
  int *sock;
  int *topicId;
  unsigned short *componentId;
  short *applicationTag;
  volatile int *pxlcnt;//NBUF*ncam
  volatile unsigned int *curSampleId;//NBUF*ncam;
  int *port;
  int *host;
  char *multicast;
  pthread_t *threadid;
  char *sourceMacAddr;
  char *myMacAddr;
}CamStruct;

typedef struct{
  CamStruct *camstr;
  int camNo;
}ThreadStruct;




void safefree(void *ptr){
  if(ptr!=NULL)
    free(ptr);
}

void dofree(CamStruct *camstr){
  int i;
  printf("dofree called\n");
  if(camstr!=NULL){
    for(i=0; i<camstr->ncam; i++){
      darc_cond_destroy(&camstr->camCond[i]);
      darc_cond_destroy(&camstr->camCond2[i]);
      darc_mutex_destroy(&camstr->camMutex[i]);
      darc_mutex_destroy(&camstr->camMutexWorker[i]);
      if(camstr->sock[i]>0){
	close(camstr->sock[i]);
      }
    }
    safefree(camstr->npxlsArr);
    safefree((void*)camstr->npxlsArrCum);
    safefree(camstr->blocksize);
    safefree((void*)camstr->newframe);
    safefree((void*)camstr->pxlsTransferred);
    safefree(camstr->thrStruct);
    safefree(camstr->threadPriority);
    safefree(camstr->threadAffinity);
    safefree(camstr->index);
    safefree(camstr->paramNames);
    safefree(camstr->nbytes);
    safefree(camstr->dtype);
    safefree(camstr->values);
    safefree(camstr->pxlx);
    safefree(camstr->pxly);
    safefree(camstr->timestamp);
    safefree((void*)camstr->mostRecentFilled);
    safefree((void*)camstr->currentFilling);
    safefree((void*)camstr->rtcReading);
    safefree((void*)camstr->camErr);
    safefree(camstr->bytesPerPixel);
    safefree(camstr->reorder);
    safefree(camstr->reorderBuf);
    safefree(camstr->reorderno);
    safefree(camstr->reorderIndx);
    safefree((void*)camstr->frameReady);
    safefree(camstr->senderAddress);
    safefree(camstr->sock);
    safefree(camstr->topicId);
    safefree(camstr->componentId);
    safefree(camstr->applicationTag);
    safefree((void*)camstr->pxlcnt);
    safefree((void*)camstr->curSampleId);
    safefree(camstr->port);
    safefree(camstr->host);
    safefree(camstr->multicast);
    safefree(camstr->threadid);
    free(camstr);
  }
}


int camSetThreadAffinityAndPriority(unsigned int *threadAffinity,int threadPriority,int threadAffinElSize){
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
  if(pthread_setaffinity_np(pthread_self(),sizeof(cpu_set_t),&mask))
    printf("Error in sched_setaffinity: %s\n",strerror(errno));
  param.sched_priority=threadPriority;
  if(pthread_setschedparam(pthread_self(),SCHED_RR,&param))
    printf("error in pthread_setschedparam - maybe run as root?\n");
  return 0;
}

#define checkRtdnpHeader(payload,payloadSize) (1-((payload)[0]&0x7==0x4))

int checkRtdnp(char *buf, int bufSize,int expectedItemSize,int *type,int *endian,int *framesize,int *payloadTag, char **data){
  //If a data packet, framesize contains the number of elements (not bytes) of the data in this packet.
  //If a header packet, framesize contains the frame size (elements) of the whole frame.
  //expectedItemSize is the number of bytes per pixel expected.  Strangely, from rtdnp, there is no way of obtaining what the item size is, except by inspecting the packet size (and this could be wrong if truncated by the MTU).
  int err=0;
  if(unlikely(buf[2]!=0 || buf[3]!=1)){//version 0.1 only.
    printf("Wrong protocol version: %d.%d.  Expecting 0.1\n",buf[2],buf[3]);
    return -1;
  }
  *type=buf[0]&0x7;//4 for leader, 0 for payload, 1 for trailer.
  *endian=(buf[0]>>4)&0x1;//1 for little endian (data only)
  if(unlikely(*type==4)){//start of frame
    *framesize=ntohl(*((unsigned int*)&buf[8]));
    *payloadTag=0;
    if(buf[1]){//exteded info size
      *data=&buf[12];
    }else{
      *data=NULL;
    }
  }else if(likely(*type==0)){//data
    *framesize=(unsigned int)ntohs((*((unsigned short*)&buf[8])));
    *payloadTag=ntohl(*((int*)&buf[12]));
    *data=&buf[16+buf[10]];//buf[10] contains lead padding size.
    if((*framesize)*expectedItemSize+16+buf[10]>bufSize){
      printf("RTDNP packet not large enough - suggest bytes per pixel is wrong?\n");
      err=-1;
    }

  }else{//end of frame
    if(*type!=1){
      printf("Unknown packet type %d\n",*type);
    }
    *framesize=0;
    *payloadTag=0;
    *data=NULL;
  }
  return err;
}

//a faster version.
int checkMudpi2(char *buf,int bufSize,char *check,unsigned short *numFrames, unsigned int *sampleId,unsigned short *msgId,char **payload,unsigned short *payloadSize){
  int err=0;
  //char tmp[8];
  //int i;
  if(likely(bufSize>=32 && memcmp(buf,check,12)==0)){
    //if((*(int*)buf)==topicId && (*(unsigned short*)&buf[4])==componentId && (*(short*)&buf[6])==applicationTag && (*(unsigned short*)&buf[10])==htons(1)){//all correct so far (note, version==1)
    *msgId=ntohs(*((unsigned short*)&buf[24]));
    *numFrames=ntohs(*((unsigned short*)&buf[26]));
    *sampleId=ntohl(*((unsigned int*)&buf[12]));//this is the frame number really...
    *payload=&buf[32];
    *payloadSize=ntohs(*((unsigned short*)&buf[28]));
    if(unlikely(bufSize<*payloadSize+32)){
      err=-1;
      printf("Packet size too small (%d < %d)\n",bufSize,*payloadSize+32);
    }
  }else{
    err=-1;
    printf("Disagreement in packet header:\nWhat\tExpected\tGot\ntopicId:\t%d\t%d\ncomponentId:\t%d\t%d\napplicationTag:\t%d\t%d\nVersion:\t1\t%d\n",ntohl(*(int*)check),ntohl((*(int*)buf)),ntohs(*(unsigned short*)&check[4]),ntohs((*(unsigned short*)&buf[4])),ntohs(*(short*)&check[6]),ntohs((*(short*)&buf[6])),ntohs((*(unsigned short*)&buf[10])));
  }
  return err;
}


int checkMudpi(char *buf,int bufSize,int topicId,unsigned short componentId, short applicationTag, int requiredMsgId, unsigned short *numFrames, unsigned int *sampleId, double *timestamp, unsigned short *msgId,char **payload,unsigned short *payloadSize){
  int err=0;
  char tmp[8];
  int i;
  if(likely((*(int*)buf)==topicId && (*(unsigned short*)&buf[4])==componentId && (*(short*)&buf[6])==applicationTag && (*(unsigned short*)&buf[10])==htons(1))){//all correct so far (note, version==1)
    *msgId=ntohs(*((unsigned short*)&buf[24]));
    if(likely(requiredMsgId<0 || (*msgId)==requiredMsgId)){
      *numFrames=ntohs(*((unsigned short*)&buf[26]));
      *sampleId=ntohl(*((unsigned int*)&buf[12]));//this is the frame number really...
      if(BYTE_ORDER==LITTLE_ENDIAN){
	for(i=0;i<8;i++)
	  tmp[7-i]=buf[16+i];
	memcpy(&buf[16],tmp,sizeof(double));
      }
      *timestamp=*((double*)&buf[16]);
      *payload=&buf[32];
      *payloadSize=ntohs(*((unsigned short*)&buf[28]));
      if(unlikely(bufSize<*payloadSize+32)){
	err=-1;
	printf("Packet size too small (%d < %d)\n",bufSize,*payloadSize+32);
      }
    }else{
      printf("msgId (%d) != requiredMsgId (%d)\n",*msgId,requiredMsgId);
      err=-1;
    }
  }else{
    err=-1;
    printf("Disagreement in packet header:\nWhat\tExpected\tGot\ntopicId:\t%d\t%d\ncomponentId:\t%d\t%d\napplicationTag:\t%d\t%d\nVersion:\t1\t%d\n",ntohl(topicId),ntohl((*(int*)buf)),ntohs(componentId),ntohs((*(unsigned short*)&buf[4])),ntohs(applicationTag),ntohs((*(short*)&buf[6])),ntohs((*(unsigned short*)&buf[10])));
  }
  return err;
}
/*
struct timeval watchDogTime;

void *watchdog(void *pcamstr){
  CamStruct *camstr=(CamStruct*)pcamstr;
  struct timeval t1;
  int dt;
  while(1){
    gettimeofday(&t1,NULL);
    dt=t1.tv_sec-watchDogTime.tv_sec;
    if(dt>10)
      printf("watchdog timer not updated for %d seconds\n",dt);
    sleep(1);
  }
  return NULL;
}
*/
void* camWorker(void *thrstrv){
  ThreadStruct *thrstr=(ThreadStruct*)thrstrv;
  CamStruct *camstr=thrstr->camstr;
  int cam=thrstr->camNo;
  int err;
  volatile int pxlcnt=0;
  volatile int bufindx=0;
  //struct sockaddr_in sendAddr;
  //socklen_t otherAddrLen;
  //unsigned int htonlSenderAddress=htonl(camstr->senderAddress[cam]);
  int newFrame=1;
  ssize_t recvLen=0;
  char *udpBuf;
  unsigned short nframes,msgId;
  volatile unsigned short curMsgId=1;//number of the packet within the frame.
  unsigned int sample,curSampleId=0;//essentially the frame number.
  //double timestamp;
  char *payload;
  unsigned short payloadSize;
  int type,endian,numPixelsInPacket,payloadTag;
  char *data;
  char *dest;
  int i;
  char *check;
  char *buffer;
  //struct sockaddr_ll saddr;
  char destsrccheck[14];
  int islocalhost=1;
  //socklen_t saddr_len=sizeof(saddr);
  if((buffer=malloc(65536))==NULL){
    printf("Unable to alloc buffer in camrtdnpPacketSocket\n");
  }
  struct ethhdr *eth=(struct ethhdr*)buffer;
  struct iphdr *ip=(struct iphdr*)(buffer+sizeof(struct ethhdr));

  memset(buffer,0,65536);
  camSetThreadAffinityAndPriority(&camstr->threadAffinity[cam*camstr->threadAffinElSize],camstr->threadPriority[cam],camstr->threadAffinElSize);
  if((check=calloc(3,4))==NULL){
    printf("failed to alloc 12 bytes in camrtdnp.c\n");
  }
  *((int*)check)=camstr->topicId[cam];
  *((unsigned short*)&check[4])=camstr->componentId[cam];
  *((short*)&check[6])=camstr->applicationTag[cam];
  *((unsigned short*)&check[10])=htons(1);

  memcpy(&destsrccheck[6],&camstr->sourceMacAddr[cam*6],6);
  if(camstr->multicast[cam*8+6]!=0){//multicasting
    memcpy(destsrccheck,&camstr->multicast[cam*8],6);
  }else{
    memcpy(destsrccheck,&camstr->myMacAddr[cam*6],6);
  }
  destsrccheck[12]=8;//ip
  destsrccheck[13]=0;//ip
  //check to see whether the source is coming from localhost.
  for(i=0;i<6;i++){
    if(destsrccheck[i]!=0){
      islocalhost=0;
    }
  }
  darc_mutex_lock(&camstr->camMutexWorker[cam]);//new
  //camstr->rtcReading[cam]=0;//rtcReading was transferFrame
  camstr->mostRecentFilled[cam]=-1;//mostRecentFilled was latestframe
  camstr->currentFilling[cam]=-1;//currentFilling was curframe
  darc_mutex_unlock(&camstr->camMutexWorker[cam]);//new
  while(camstr->open){
    err=0;
    //Now read packets until we've got them all, or a packet from the next frame arrives...
    //otherAddrLen=sizeof(sendAddr);
    //sendAddr.sin_addr.s_addr=0;
    //while((sendAddr.sin_addr.s_addr!=htonlSenderAddress) && (err==0)){
    memset(eth->h_source,islocalhost,6);//clear the mac address (or set to 1 if localhost)
    while((memcmp(eth->h_source,&destsrccheck[6],6)!=0) && (err==0)){
      //gettimeofday(&watchDogTime,NULL);
      //recvLen=recvfrom(camstr->sock[cam],buffer,65536,0,(struct sockaddr*)&saddr,&saddr_len);
      recvLen=recvfrom(camstr->sock[cam],buffer,65536,0,NULL,NULL);
      if(recvLen<=0){
	printf("UDP receiving error for cam %d\n",cam);
	printf("%s\n",strerror(errno));
	err=1;
      }
    }
    if(likely(memcmp(eth,destsrccheck,14)==0)){
      if(likely(ip->protocol==17 && ip->version==4)){//udp packet
	int iphdrlen = ip->ihl*4;
	struct udphdr *udp=(struct udphdr*)(buffer + iphdrlen + sizeof(struct ethhdr));
	if(likely(ntohs(udp->dest)==camstr->port[cam])){
	  recvLen=ntohs(udp->len);
	  udpBuf=(buffer + iphdrlen + sizeof(struct ethhdr) + sizeof(struct udphdr));

	  //if(err==0 && checkMudpi(udpBuf,recvLen,camstr->topicId[cam],camstr->componentId[cam],camstr->applicationTag[cam],-1,&nframes,&sample,&timestamp,&msgId,&payload,&payloadSize)==0){//okay, its a mudpi packet!
	  if(likely(err==0 && checkMudpi2(udpBuf,recvLen,check,&nframes,&sample,&msgId,&payload,&payloadSize)==0)){//okay, its a mudpi packet!
	    //now get the data from it.
	    if(likely(checkRtdnp(payload,payloadSize,camstr->bytesPerPixel[cam],&type,&endian,&numPixelsInPacket,&payloadTag,&data)==0)){//numPixelsInPacket will be number of data units total if a header, or number of data units in this packet if a payload.
	      if(likely(type==0 && newFrame==0)){//a payload packet, and header already received.
		//copy data to the correct point and update the map of received packets, and inform waiting threads if necessary.
		//For now, since we have no way of knowing the offset of this data within the frame (since its not embedded in mudpi or rtdnp), I will enforce in-rder packet arrival...
		if(likely(msgId==curMsgId+1) && curSampleId==sample){//in order packet
		  curMsgId++;
		  dest=&camstr->DMAbuf[cam][bufindx*(camstr->npxlsArr[cam]*camstr->bytesPerPixel[cam]) + pxlcnt*camstr->bytesPerPixel[cam]];
		  memcpy(dest,data,camstr->bytesPerPixel[cam]*numPixelsInPacket);
		  if(camstr->bytesPerPixel[cam]==2 && ((BYTE_ORDER==LITTLE_ENDIAN && endian==0) || (BYTE_ORDER==BIG_ENDIAN && endian==1))){//do a byteswap for time.
		    for(i=0;i<numPixelsInPacket;i++){
		      ((unsigned short*)dest)[i]=ntohs(((unsigned short*)dest)[i]);
		    }
		  }else if(camstr->bytesPerPixel[cam]>2){
		    printf("TODO: Implement byteswap for >2 bytes per pixel\n");
		  }
		  pxlcnt+=numPixelsInPacket;
		  //todo: Can we remove this mutex somehow?
		  camstr->pxlcnt[NBUF*cam+bufindx]=pxlcnt;
		  if(pxlcnt>=camstr->npxlsArr[cam]){//all pixels have arrived...
		    darc_mutex_lock(&camstr->camMutexWorker[cam]);
		    if(camstr->mostRecentFilled[cam]!=-1)
		      printf("Cam %d skipping: darc not keeping up/diff Hz/lost data\n",cam);
		    if(camstr->currentFilling[cam]!=-1){//not currently being read
		      printf("Processing of cam %d not yet started (the other cameras probably aren't producing pixels)\n",cam);
		      camstr->mostRecentFilled[cam]=camstr->currentFilling[cam];
		    }else{
		      camstr->mostRecentFilled[cam]=-1;//should be anyway.
		    }
		    camstr->currentFilling[cam]=-1;
		    darc_cond_broadcast(&camstr->camCond[cam]);//signal should do.
		    darc_mutex_unlock(&camstr->camMutexWorker[cam]);
		    if(camstr->recordTimestamp)//an option to put camera frame number as time in us of last pixel arriving... useful for synchronising different cameras so that last pixel arrives at same time.
		      clock_gettime(CLOCK_MONOTONIC,&camstr->timestamp[NBUF*cam+bufindx]);
		  }
		}else{
		  printf("Packet missing (or out of order). %d->%d  Skipping frame.\n",curMsgId,msgId);
		  newFrame=1;
		  camstr->currentFilling[cam]=-1;
		  camstr->newframe[cam]=1;
		  camstr->camErr[NBUF*cam+bufindx]=1;
		}
	      }else if(type==0x4){//header packet
		curMsgId=1;
		//If not all messages received from previous frame raise error.
		if(newFrame!=1){//cancel previous frame
		  printf("Start of frame received before frame finished\n");
		  camstr->camErr[NBUF*cam+bufindx]=1;
		}
		newFrame=0;//we're now expecting some data, not a frame header.
		bufindx++;//new frame
		if(bufindx>=NBUF){
		  bufindx=0;
		}
		camstr->camErr[NBUF*cam+bufindx]=0;
		pxlcnt=0;
		curSampleId=sample;
		camstr->curSampleId[NBUF*cam+bufindx]=sample;
		camstr->pxlcnt[NBUF*cam+bufindx]=0;
		//This next bit is actually done when last pixel arrives.
		//camstr->mostRecentFilled[cam]=camstr->currentFilling[cam];
		darc_mutex_lock(&camstr->camMutexWorker[cam]);
		camstr->currentFilling[cam]=bufindx;
		darc_cond_broadcast(&camstr->camCond[cam]);
		darc_mutex_unlock(&camstr->camMutexWorker[cam]);
	      }else if(type==0x1){//trailer packet
		newFrame=1;
		if(unlikely(msgId!=curMsgId+1 || msgId!=nframes)){//arrived out of order, or badly formatted (i.e. error in sender)
		  printf("Missing packets somewhere\n");
		  camstr->camErr[NBUF*cam+bufindx]=1;
		  camstr->currentFilling[cam]=-1;
		  camstr->newframe[cam]=1;
		}
	      }else if(type!=0){
		printf("Unrecognised rtdnp packet type %d\n",type);
	      }
	    }else{
	      printf("Not rtdnp packet\n");
	    }
	  }else{
	    printf("Not mudpi packet\n");
	  }
	}else{//wrong port
	}
      }else{//not udp, or not ipv4
      }
    }else{//wrong src/dest or not ip
    }
  }
  return NULL;
}





/**
   Open a camera of type name.  Args are passed in a int32 array of size n, which can be cast if necessary.  Any state data is returned in camHandle, which should be NULL if an error arises.
   pxlbuf is the array that should hold the data. The library is free to use the user provided version, or use its own version as necessary (ie a pointer to physical memory or whatever).  It is of size npxls*sizeof(short).
   ncam is number of cameras, which is the length of arrays pxlx and pxly, which contain the dimensions for each camera.
   Name contains the library name.
   frameno is a pointer to an array with a value for each camera in which the frame number should be placed.

*/


#define TEST(a) if((a)==NULL){printf("calloc error\n");dofree(camstr);*camHandle=NULL;return 1;}

int camOpen(char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **camHandle,int nthreads,unsigned int frameno,unsigned int **camframeno,int *camframenoSize,int npxls,int ncam,int *pxlx,int* pxly){
  CamStruct *camstr;
  int i,j,k;
  unsigned short *tmps;
  char pxlbuftype;
  int maxbpp;
  int ngot;
  int bytespp;
  char **camParamName;
  printf("Initialising camera %s\n",name);
  if((*camHandle=malloc(sizeof(CamStruct)))==NULL){
    printf("Couldn't malloc camera handle\n");
    return 1;
  }
  memset(*camHandle,0,sizeof(CamStruct));
  camstr=(CamStruct*)*camHandle;
  TEST(camstr->bytesPerPixel=calloc(sizeof(int),ncam));
  maxbpp=0;
  if(n>=ncam){
    for(i=0;i<ncam;i++){
      camstr->bytesPerPixel[i]=args[i];//bytes per pixel.  1 or 2.
      if(args[i]>maxbpp)
	maxbpp=args[i];
    }
  }
    bytespp=maxbpp;//save the largest bytes per pixel - this is what the array will be, then any cameras that use fewer bytes will have to be converted.
  camstr->bytespp=bytespp;
  if(bytespp==1){
    pxlbuftype='B';
  }else if(bytespp==2){
    pxlbuftype='H';
  }else if(bytespp==4){
    pxlbuftype='f';
    printf("using float array for pixels...\n");
  }else{
    printf("%d bytes per pixel not yet coded - please recode camrtdnp.c\n",bytespp);
    free(*camHandle);
    *camHandle=NULL;
    return 1;
  }
  if(arr->pxlbuftype!=pxlbuftype || arr->pxlbufsSize!=bytespp*npxls){
      //need to resize the pxlbufs...
    arr->pxlbufsSize=bytespp*npxls;
    arr->pxlbuftype=pxlbuftype;
    arr->pxlbufelsize=bytespp;
    tmps=realloc(arr->pxlbufs,arr->pxlbufsSize);
    if(tmps==NULL){
      if(arr->pxlbufs!=NULL)
	free(arr->pxlbufs);
      printf("pxlbuf malloc error in camrtdnp.c\n");
      arr->pxlbufsSize=0;
      free(*camHandle);
      *camHandle=NULL;
      return 1;
    }
    arr->pxlbufs=tmps;
    memset(arr->pxlbufs,0,arr->pxlbufsSize);
  }
  camstr->imgdata=arr->pxlbufs;

  if(*camframenoSize<ncam*3){
    if(*camframeno!=NULL)
      free(*camframeno);
    if((*camframeno=calloc(3*ncam,sizeof(unsigned int)))==NULL){
      printf("Couldn't malloc camframeno\n");
      *camframenoSize=0;
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }else{
      *camframenoSize=3*ncam;
    }
  }

  camstr->userFrameNo=*camframeno;
  camstr->ncam=ncam;
  camstr->rtcErrorBuf=rtcErrorBuf;
  camstr->npxls=npxls;
  TEST(camstr->npxlsArr=calloc(ncam,sizeof(int)));
  TEST(camstr->npxlsArrCum=calloc((ncam+1),sizeof(int)));
  TEST(camstr->blocksize=calloc(ncam,sizeof(int)));
  TEST(camstr->newframe=calloc(ncam,sizeof(int)));
  TEST(camstr->pxlsTransferred=calloc(ncam,sizeof(int)));
  TEST(camstr->thrStruct=calloc(ncam,sizeof(ThreadStruct)));
  TEST(camstr->camMutex=calloc(ncam,sizeof(darc_mutex_t)));
  TEST(camstr->camMutexWorker=calloc(ncam,sizeof(darc_mutex_t)));
  TEST(camstr->camCond=calloc(ncam,sizeof(darc_cond_t)));
  TEST(camstr->camCond2=calloc(ncam,sizeof(darc_cond_t)));
  TEST(camstr->threadPriority=calloc(ncam,sizeof(int)));
  TEST(camstr->pxlx=calloc(ncam,sizeof(int)));
  TEST(camstr->pxly=calloc(ncam,sizeof(int)));
  TEST(camstr->frameReady=calloc(ncam,sizeof(int)));
  TEST(camstr->reorder=calloc(ncam,sizeof(int)));
  TEST(camstr->reorderno=calloc(ncam,sizeof(int)));
  TEST(camstr->reorderIndx=calloc(ncam,sizeof(int)));
  TEST(camstr->reorderBuf=calloc(ncam,sizeof(int*)));

  TEST(camstr->timestamp=calloc(ncam*NBUF,sizeof(struct timespec)));
  TEST(camstr->camErr=calloc(ncam*NBUF,sizeof(int*)));

  TEST(camstr->mostRecentFilled=calloc(ncam,sizeof(int)));
  TEST(camstr->currentFilling=calloc(ncam,sizeof(int)));
  TEST(camstr->rtcReading=calloc(ncam,sizeof(int)));
  TEST(camstr->senderAddress=calloc(ncam,sizeof(unsigned int)));
  TEST(camstr->sock=calloc(ncam,sizeof(unsigned int)));
  TEST(camstr->topicId=calloc(ncam,sizeof(unsigned int)));
  TEST(camstr->componentId=calloc(ncam,sizeof(unsigned short)));
  TEST(camstr->applicationTag=calloc(ncam,sizeof(short)));
  TEST(camstr->pxlcnt=calloc(ncam*NBUF,sizeof(int)));
  TEST(camstr->curSampleId=calloc(ncam*NBUF,sizeof(unsigned int)));
  TEST(camstr->port=calloc(ncam,sizeof(unsigned int)));
  TEST(camstr->host=calloc(ncam,sizeof(int)));
  TEST(camstr->multicast=calloc(ncam,8));
  TEST(camstr->threadid=calloc(ncam,sizeof(pthread_t)));
  TEST(camstr->sourceMacAddr=calloc(ncam,6));
  TEST(camstr->myMacAddr=calloc(ncam,6));

  camstr->npxlsArrCum[0]=0;
  printf("malloced things\n");
  for(i=0; i<ncam; i++){
    camstr->pxlx[i]=pxlx[i];
    camstr->pxly[i]=pxly[i];
    camstr->npxlsArr[i]=pxlx[i]*pxly[i];
    camstr->npxlsArrCum[i+1]=camstr->npxlsArrCum[i]+camstr->npxlsArr[i];
  }

  //Parameters are:
  //bytes per pixel[ncam]
  //port to bind[ncam]
  //-1 or host interface number to bind to[ncam] (socket.if_nametoindex("eth0"))
  //multicast[2*ncam] (therefore subscription required, or 0 if not required)
  //blocksize[ncam]
//reorder[ncam]
  //topicId[ncam]
  //componentId[ncam]
  //applicationTag[ncam]
  //sourceMacAddr[2*ncam]
  //myMacAddr[2*ncam]
  //prio[ncam]
  //affinElSize
  //affin[ncam*elsize]
  //recordTimestamp


  //For multicast, if 0, not a multicast camera.  Else, the mac address - see below.

  //host: the index number of the ethernet interface - this can be found using python3: socket.if_nametoindex("eth0") etc.

  //for mac addresses, use something like (for a mac address of aa:bb:cc:dd:ee:ff:   params[i]=0xddccbbaa; params[i+1]=0x01ffee (note, the 01 is necessary as a flag for multicast to be used.).  Alternatively, socket.htonl(0xaabbccdd),socket.htonl(0xeeff1)

  if(n>15*ncam)
    camstr->threadAffinElSize=args[15*ncam];
  else
    camstr->threadAffinElSize=1;
  TEST(camstr->threadAffinity=calloc(ncam*camstr->threadAffinElSize,sizeof(int)));
  if(n!=15*ncam + 1 + ncam*camstr->threadAffinElSize +1){
    printf("Wrong number of args for cameraParams\n");
    dofree(camstr);
    *camHandle=NULL;
    return 1;
  }


  for(i=0;i<ncam;i++){
    camstr->port[i]=args[ncam+i];
    camstr->host[i]=args[2*ncam+i];
    //camstr->senderAddress[i]=args[3*ncam+i];//not used
    //camstr->multicast[i]=args[3*ncam+i];
    memcpy(&camstr->multicast[i*8],&args[3*ncam+i],8);//copy 6 bytes + 2 bytes for a flag - which must ==1 for multicast to be used.
    camstr->blocksize[i]=args[5*ncam+i];
    camstr->reorder[i]=args[6*ncam+i];
    camstr->topicId[i]=htonl(args[7*ncam+i]);
    camstr->componentId[i]=htons((unsigned short)args[8*ncam+i]);
    camstr->applicationTag[i]=htons((short)args[9*ncam+i]);
    memcpy(&camstr->sourceMacAddr[i*6],&args[10*ncam+i],6);//copy 6 bytes
    memcpy(&camstr->myMacAddr[i*6],&args[12*ncam+i],6);//copy 6 bytes
    camstr->threadPriority[i]=args[14*ncam+i];
    for(j=0;j<camstr->threadAffinElSize;j++)
      camstr->threadAffinity[i*camstr->threadAffinElSize+j]=args[15*ncam+1+i*camstr->threadAffinElSize+j];
  }
  camstr->recordTimestamp=args[15*ncam+1+camstr->threadAffinElSize*ncam];
  if(n<11*ncam+1+camstr->threadAffinElSize*ncam)
    printf("ERROR - wrong number of arguments specified\n");

  printf("got args (recordTimestamp=%d)\n",camstr->recordTimestamp);


  //now need to prepare the camera parameter buffer names: aravisCmdN, camReorderN
  ngot=0;
  for(i=0; i<ncam; i++){
    if(camstr->reorder[i]!=0){
      for(j=0;j<ngot;j++){//have we already got this reorder?
	if(camstr->reorderno[j]==camstr->reorder[i]){
	  break;
	}
      }
      if(j==ngot){//a new entry
	camstr->reorderno[j]=camstr->reorder[i];
	ngot++;
      }
    }
  }
  memset(camstr->reorderIndx,-1,sizeof(int)*ncam);


  TEST(camParamName=calloc(ngot,sizeof(char*)));
  for(i=0;i<ngot;i++){
    if((camParamName[i]=calloc(BUFNAMESIZE,1))==NULL){
      printf("Failed to calloc reorders in camAravis\n");
      dofree(camstr);
      *camHandle=NULL;
      for(i=0;i<ngot;i++)
	free(camParamName[i]);
      free(camParamName);
      return 1;
    }
    snprintf(camParamName[i],16,"camReorder%d",camstr->reorderno[i]);
  }



  //Now sort them... (actually, not necessary if ncam<10 - already sorted).
#define islt(a,b) (strcmp((*a),(*b))<0)
  QSORT(char*,camParamName,ngot,islt);
#undef islt
  //now capture the order
  for(i=0;i<ngot;i++){
    j=atoi(&camParamName[i][10]);
    for(k=0;k<ncam;k++){
      if(camstr->reorder[k]==j){
	camstr->reorderIndx[k]=i;
      }
    }
  }

  //now make the parameter buffer
  if((camstr->paramNames=calloc(ngot,BUFNAMESIZE))==NULL){
    printf("Failed to mallocparamNames in camrtdnp.c\n");
    dofree(camstr);
    *camHandle=NULL;
    for(i=0; i<ngot; i++)
      free(camParamName[i]);
    free(camParamName);
    return 1;
  }
  for(i=0; i<ngot; i++){
    memcpy(&camstr->paramNames[i*BUFNAMESIZE],camParamName[i],BUFNAMESIZE);
    printf("%16s\n",&camstr->paramNames[i*BUFNAMESIZE]);
    free(camParamName[i]);
  }
  free(camParamName);
  TEST(camstr->index=calloc(sizeof(int),ngot));
  TEST(camstr->values=calloc(sizeof(void*),ngot));
  TEST(camstr->dtype=calloc(sizeof(char),ngot));
  TEST(camstr->nbytes=calloc(sizeof(int),ngot));
  camstr->nReorders=ngot;

  for(i=0; i<ncam; i++){
    if(darc_cond_init(&camstr->camCond[i],NULL)!=0){
      printf("Error initialising condition variable %d\n",i);
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }
    if(darc_cond_init(&camstr->camCond2[i],NULL)!=0){
      printf("Error initialising condition variable2 %d\n",i);
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }
    if(darc_mutex_init(&camstr->camMutex[i],darc_mutex_init_NULL)!=0){
      printf("Error initialising mutex variable\n");
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }
    if(darc_mutex_init(&camstr->camMutexWorker[i],darc_mutex_init_NULL)!=0){
      printf("Error initialising mutex worker variable\n");
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }
  }

  if((camstr->DMAbuf=(char**)malloc(ncam*sizeof(char*)))==NULL){
    printf("Couldn't allocate DMA buffer\n");
    dofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  for(i=0; i<ncam; i++){
    if((camstr->DMAbuf[i]=(char*)calloc(camstr->npxlsArr[i]*NBUF,camstr->bytesPerPixel[i]))==NULL){
      printf("Couldn't allocate DMA buffer %d\n",i);
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }
  }
  if(camNewParam(*camHandle,pbuf,frameno,arr)!=0){
    printf("Error in camOpen->newParam...\n");
    dofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  printf("Reorders:\n");
  for(i=0;i<ncam;i++)
    printf("%d %p\n",camstr->reorder[i],camstr->reorderBuf[i]);

  camstr->open=1;
  struct sockaddr_ll sockIn;
  int optval;
  memset(&sockIn,0,sizeof(sockIn));
  sockIn.sll_family = PF_PACKET;
  for(i=0;i<ncam;i++){//start the camera threads.

    if((camstr->sock[i]=socket(AF_PACKET,SOCK_RAW,htons(ETH_P_IP)))==-1){
      printf("Cannot create socket for cam %d\n",i);
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }
    optval=1;
    if(setsockopt(camstr->sock[i],SOL_SOCKET,SO_REUSEADDR,&optval, sizeof(int))!=0){
      printf("setsockopt failed for SO_REUSEADDR - ignoring\n");
    }
    if(camstr->host[i]!=-1){
      sockIn.sll_family=PF_PACKET;
      sockIn.sll_protocol=htons(ETH_P_ALL);
      sockIn.sll_ifindex=camstr->host[i];//This can be got using (python3): socket.if_nametoindex("eth0") etc.  Or, in c, given a string, if_nametoindex(camstr->host[i]); Does this need a htonl???
      if(bind(camstr->sock[i], (struct sockaddr*) &sockIn, sizeof(sockIn))<0){
	printf("bind failed\n");
	dofree(camstr);
	*camHandle=NULL;
	return 1;
      }
    }//otherwise, don't bind, just listen on all interfaces.
    if(camstr->multicast[i*8+6]!=0){//multicast is used... (the 7th byte is a flag)
      struct packet_mreq mreq;
      if(camstr->host[i]!=-1)
	mreq.mr_ifindex=camstr->host[i];
      else
	mreq.mr_ifindex=0;
      mreq.mr_type=PACKET_MR_MULTICAST;
      mreq.mr_alen=ETH_ALEN;//6.
      memcpy(mreq.mr_address,&camstr->multicast[i*8],ETH_ALEN);//physical-layer address
      setsockopt(camstr->sock[i], SOL_PACKET,PACKET_ADD_MEMBERSHIP, &mreq, sizeof(mreq));
    }

    ((ThreadStruct*)camstr->thrStruct)[i].camNo=i;
    ((ThreadStruct*)camstr->thrStruct)[i].camstr=camstr;
    pthread_create(&camstr->threadid[i],NULL,camWorker,(void*)(&((ThreadStruct*)camstr->thrStruct)[i]));
  }
  //pthread_t watchdogtid;
  //pthread_create(&watchdogtid,NULL,watchdog,(void*)camstr);
  printf("created threads (%d)\n",ncam);
  return 0;
}





/**
   Close a camera of type name.  Args are passed in the int32 array of size n, and state data is in camHandle, which should be freed and set to NULL before returning.
*/
int camClose(void **camHandle){
  CamStruct *camstr;
  int i;
  printf("Closing camera\n");
  if(*camHandle==NULL)
    return 1;
  camstr=(CamStruct*)*camHandle;
  camstr->open=0;
  for(i=0; i<camstr->ncam; i++){
    memset((char*)camstr->camErr,1,sizeof(int)*NBUF*camstr->ncam);
    //camstr->camErr[i]=1;
    pthread_cancel(camstr->threadid[i]);//,SIGTERM);
    darc_mutex_lock(&camstr->camMutexWorker[i]);
    darc_cond_broadcast(&camstr->camCond[i]);
    darc_mutex_unlock(&camstr->camMutexWorker[i]);
  }
  for(i=0; i<camstr->ncam; i++){
    pthread_join(camstr->threadid[i],NULL);//wait for worker thread to complete
  }
  dofree(camstr);
  *camHandle=NULL;
  printf("Camera closed\n");
  return 0;
}


/**
   New parameters in the buffer (optional)...
*/
int camNewParam(void *camHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr){
  //the only params needed is camReorder if reorder!=0/
  int i,j;
  CamStruct *camstr=(CamStruct*)camHandle;
  int err=0;

  bufferGetIndex(pbuf,camstr->nReorders,camstr->paramNames,camstr->index,camstr->values,camstr->dtype,camstr->nbytes);
  memset(camstr->reorderBuf,0,camstr->ncam*sizeof(int*));
  for(i=0;i<camstr->nReorders;i++){
    printf("%16s: Index %d\n",&camstr->paramNames[i*BUFNAMESIZE],camstr->index[i]);
    if(camstr->index[i]>=0){
      if(i<camstr->nReorders){//camReorder...
	if(camstr->nbytes[i]>0){
	  if(camstr->dtype[i]=='i'){
	    //for which camera(s) is this?
	    for(j=0; j<camstr->ncam; j++){
	      if(camstr->reorderIndx[j]==i){//a reorder for this camera
		if(camstr->nbytes[i]==sizeof(int)*camstr->npxlsArr[j]){
		  camstr->reorderBuf[j]=(int*)camstr->values[i];
		}else{
		  printf("Wrong size for camReorder\n");
		  err=1;
		}
	      }
	    }
	  }else{
	    printf("Wrong dtype for camReorder\n");
	    err=1;
	  }
	}
      }
    }
  }
  return err;
}


/**
   Called when we're starting processing the next frame.  This doesn't actually wait for any pixels.
Single thread
*/
inline int camNewFrameSync(void *camHandle,unsigned int thisiter,double starttime){
  //printf("camNewFrame\n");
  CamStruct *camstr;
  int i;
  camstr=(CamStruct*)camHandle;
  if(camHandle==NULL){
    return 1;
  }
  camstr->thisiter=thisiter;
  for(i=0;i<camstr->ncam; i++)
    camstr->newframe[i]=1;
  return 0;
}

/**
   Wait for the next n pixels of the current frame to arrive from camera cam.
   Note - this can be called by multiple threads at same time.  Need to make sure its thread safe.

*/
inline int camWaitPixels(int n,int cam,void *camHandle){
  CamStruct *camstr=(CamStruct*)camHandle;
  int rt=0;
  int i;
  volatile int gotNewFrame;
  int j;
  int bufindx;
  volatile int cnt;
  struct timespec t1;
  if(camHandle==NULL){
    return 1;
  }
  if(n<0)
    n=0;
  if(n>camstr->npxlsArr[cam])
    n=camstr->npxlsArr[cam];
  //try to hold mutex for minimum amount of time, since the UDP reading thread also blocks on this.
  darc_mutex_lock(&camstr->camMutex[cam]);
  if(camstr->newframe[cam]){//first thread for this camera after new frame...
    camstr->newframe[cam]=0;
    camstr->frameReady[cam]=0;
    camstr->pxlsTransferred[cam]=0;
    gotNewFrame=0;
    darc_mutex_lock(&camstr->camMutexWorker[cam]);
    while(gotNewFrame==0 && rt==0){
      if(camstr->currentFilling[cam]!=-1 && camstr->currentFilling[cam]!=camstr->rtcReading[cam]){
	//jump to the newest (currently arriving) pixel image.
	camstr->rtcReading[cam]=camstr->currentFilling[cam];
	camstr->currentFilling[cam]=-1;
	if(camstr->mostRecentFilled[cam]!=-1)
	  printf("Skipping frame for cam %d\n",cam);
	camstr->mostRecentFilled[cam]=-1;
	gotNewFrame=1;
      }else if(camstr->currentFilling[cam]==-1 && camstr->mostRecentFilled[cam]!=-1 && camstr->mostRecentFilled[cam]!=camstr->rtcReading[cam]){
	//have a full buffer waiting for processing
	camstr->rtcReading[cam]=camstr->mostRecentFilled[cam];
	camstr->mostRecentFilled[cam]=-1;
	gotNewFrame=1;
      }else{
	//wait for the next frame to start.
	//What should we do about errors here?
	//We don't care about errors here.  Since we're waiting for a new frame, it means that we've completed previous frames.  So, we can simple reset camErr to zero.
	camstr->rtcReading[cam]=-1;
	//printf("waiting for sof\n");
	clock_gettime(CLOCK_MONOTONIC,&t1);
	t1.tv_sec++;//1 second wait.
	if(darc_cond_timedwait(&camstr->camCond[cam],&camstr->camMutexWorker[cam],&t1)!=0){
	  printf("timeout while waiting for start of frame\n");
	  rt=1;
	  camstr->rtcReading[cam]=-1;
	}
      }
      //if(camstr->rtcReading>=0){
      //camstr->camErr[NBUF*cam+camstr->rtcReading[cam]]=0;//we're waiting for new frame - so don't care about errors.  And if a frame has an error, it won't appear on currentFilling, or mostRecentFilled, so we're ok to ignore.
      //}
    }
    //wake the other threads for this camera that are waiting.
    if(rt==0){
      camstr->userFrameNo[cam*3]=camstr->curSampleId[NBUF*cam+camstr->rtcReading[cam]];
    }
    camstr->frameReady[cam]=1;
    darc_mutex_unlock(&camstr->camMutexWorker[cam]);
  }
  cnt=0;
  bufindx=camstr->rtcReading[cam];
  if(unlikely(bufindx<0)){
    for(i=0;i<NBUF;i++)
      camstr->camErr[NBUF*cam+i]=1;
    darc_mutex_unlock(&camstr->camMutex[cam]);
    return 1;
  }
  darc_mutex_unlock(&camstr->camMutex[cam]);

  while(rt==0 && camstr->pxlcnt[NBUF*cam+bufindx]<n && (rt=camstr->camErr[NBUF*cam+bufindx])==0){
    //busy wait...
    cnt++;
    if(cnt>1000000000){
      printf("Timeout while waiting for pixels\n");
      rt=1;
    }
  }
  darc_mutex_lock(&camstr->camMutex[cam]);

  //Now copy the data.
  if(rt==0 && n>camstr->pxlsTransferred[cam]){
      if(camstr->bytespp==1){//all cameras are <=8 bits per pixel.
        if(camstr->reorder[cam]==0){//no pixel reordering
	  memcpy(&camstr->imgdata[(camstr->npxlsArrCum[cam]+camstr->pxlsTransferred[cam])],&(camstr->DMAbuf[cam][bufindx*(camstr->npxlsArr[cam]*camstr->bytesPerPixel[cam])+camstr->pxlsTransferred[cam]]),(n-camstr->pxlsTransferred[cam]));
        }else{
	  if(camstr->reorderBuf[cam]!=NULL){//reordering based on parameter buffer.
	    for(i=camstr->pxlsTransferred[cam];i<n;i++){
	      camstr->imgdata[camstr->npxlsArrCum[cam]+camstr->reorderBuf[cam][i]]=camstr->DMAbuf[cam][bufindx*(camstr->npxlsArr[cam]*camstr->bytesPerPixel[cam])+i];
	    }
	  }else if(camstr->reorder[cam]==1){//specific reorder for LVSM
	    int top,block;
	    for(i=camstr->pxlsTransferred[cam];i<n;i++){
	      top=(i/4000)%2;
	      block=i/8000;
	      if(top){
	        j=block*4000+i%4000;
	      }else{
	        j=camstr->npxlsArr[cam]-1-(block*4000+i%4000);
	      }
	      camstr->imgdata[camstr->npxlsArrCum[cam]+j]=camstr->DMAbuf[cam][bufindx*(camstr->npxlsArr[cam]*camstr->bytesPerPixel[cam])+i];
	    }
	  }
        }
      }else if(camstr->bytespp==2){
        if(camstr->bytesPerPixel[cam]==1){//cast from uint8 to uint16
	  for(i=camstr->pxlsTransferred[cam];i<n;i++){
	    ((unsigned short*)camstr->imgdata)[camstr->npxlsArrCum[cam]+i]=(unsigned short)(camstr->DMAbuf[cam][bufindx*(camstr->npxlsArr[cam])+i]);
	  }
        }else if(camstr->bytesPerPixel[cam]==2){//copy
	  if(camstr->reorder[cam]==0){//no reordering of pixels
	    memcpy(&camstr->imgdata[sizeof(unsigned short)*(camstr->npxlsArrCum[cam]+camstr->pxlsTransferred[cam])],&(((unsigned short*)(camstr->DMAbuf[cam]))[bufindx*(camstr->npxlsArr[cam])+camstr->pxlsTransferred[cam]]),sizeof(unsigned short)*(n-camstr->pxlsTransferred[cam]));
	  }else{
	    if(camstr->reorderBuf[cam]!=NULL){
	      for(i=camstr->pxlsTransferred[cam];i<n;i++){
	        ((unsigned short*)camstr->imgdata)[camstr->npxlsArrCum[cam]+camstr->reorderBuf[cam][i]]=(((unsigned short*)(camstr->DMAbuf[cam]))[bufindx*(camstr->npxlsArr[cam])+i]);
	      }
	    }else if(camstr->reorder[cam]==1){//specific reorder for LVSM
	      //transfers 5 rows (4k pixels) at a time, 5xtop, 5xbottom, 5xtop etc
	      int top,block;
	      for(i=camstr->pxlsTransferred[cam];i<n;i++){
	        top=(i/4000)%2;
	        block=i/8000;
	        if(top){
	  	j=block*4000+i%4000;
	        }else{
	  	j=camstr->npxlsArr[cam]-1-(block*4000+i%4000);
	        }
	        ((unsigned short*)camstr->imgdata)[camstr->npxlsArrCum[cam]+j]=(((unsigned short*)(camstr->DMAbuf[cam]))[bufindx*(camstr->npxlsArr[cam])+i]);
	      }
	    }

	  }
	}
      }else if(camstr->bytespp==4){
        if(camstr->bytesPerPixel[cam]==1){//cast from uint8 to float32
	  for(i=camstr->pxlsTransferred[cam];i<n;i++){
	    ((float*)camstr->imgdata)[camstr->npxlsArrCum[cam]+i]=(float)(camstr->DMAbuf[cam][bufindx*(camstr->npxlsArr[cam])+i]);
	  }
        }else if(camstr->bytesPerPixel[cam]==2){//cast from uint16 to float32
          // const int npxlsArrCum = camstr->npxlsArrCum[cam];
	  // const int npxlsArr = camstr->npxlsArr[cam];
	  float *pxlData = &(((float*)camstr->imgdata)[camstr->npxlsArrCum[cam]]);
	  unsigned short *DMAbuf = &(((unsigned short*)(camstr->DMAbuf[cam]))[bufindx*(camstr->npxlsArr[cam])]);
	  if(camstr->reorder[cam]==0){//no reordering of pixels
	    for(i=camstr->pxlsTransferred[cam];i<n;i++){
	      pxlData[i]=(float)(DMAbuf[i]);
	    }
	  }else{
	    if(camstr->reorderBuf[cam]!=NULL){
	      for(i=camstr->pxlsTransferred[cam];i<n;i++){
	        ((float*)camstr->imgdata)[camstr->npxlsArrCum[cam]+camstr->reorderBuf[cam][i]]=(float)(((unsigned short*)(camstr->DMAbuf[cam]))[bufindx*(camstr->npxlsArr[cam])+i]);
	      }
	    }else if(camstr->reorder[cam]==1){//specific reorder for LVSM
	      //transfers 5 rows (4k pixels) at a time, 5xtop, 5xbottom, 5xtop etc
	      int top,block;
	      for(i=camstr->pxlsTransferred[cam];i<n;i++){
	        top=(i/4000)%2;
	        block=i/8000;
	        if(top){
	  	j=block*4000+i%4000;
	        }else{
	  	j=camstr->npxlsArr[cam]-1-(block*4000+i%4000);
	        }
	        ((float*)camstr->imgdata)[camstr->npxlsArrCum[cam]+j]=(float)(((unsigned short*)(camstr->DMAbuf[cam]))[bufindx*(camstr->npxlsArr[cam])+i]);
	      }
	    }

	  }
	}else{
	  printf("Can't yet handle >4 bytes per pixel in camrtdnp - please recode\n");
        }
      }
      camstr->pxlsTransferred[cam]=n;
    }
    if(rt!=0){//An error has arisen - probably dropped packet.  So how do we handle this?
    camstr->newframe[cam]=1;
    printf("camWaitPixels got err %d (cam %d) frame[0]=%d frame[%d]=%d\n",rt,cam,camstr->userFrameNo[0],cam,camstr->userFrameNo[cam*3]);

  }
  darc_mutex_unlock(&camstr->camMutex[cam]);
  return rt;
}

inline int camFrameFinishedSync(void *camHandle,int err,int forcewrite){//subap thread (once)
  int i;
  int bufindx;
  CamStruct *camstr=(CamStruct*)camHandle;
  if(camstr->recordTimestamp){//an option to put camera frame number as us time
    for(i=0; i<camstr->ncam; i++){
      bufindx=camstr->rtcReading[i];
      camstr->userFrameNo[i*3+1]=(unsigned int)(camstr->timestamp[NBUF*i+bufindx].tv_sec);//-TIMESECOFFSET);
      camstr->userFrameNo[i*3+2]=(unsigned int)(camstr->timestamp[NBUF*i+bufindx].tv_nsec);
    }
  }
  return 0;
}
