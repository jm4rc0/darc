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
   This is a library that can be used for combining async mirror demands.
*/
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
//#include <sys/select.h>
#include <poll.h>
#include <arpa/inet.h>
#include <mqueue.h>
#include <netinet/in.h>//struct ip_mreq // drj 140422: added for ip_mreq definition

#include <sys/mman.h>
// #ifdef USEAGBBLAS
#ifdef USEMKL
#include <mkl.h>
#include <mkl_types.h>
#include <mkl_blas.h>
#include <mkl_cblas.h>
#include <mkl_vml.h>
#endif
// #else
// #include <gsl/gsl_cblas.h>
// typedef enum CBLAS_ORDER CBLAS_ORDER;
// typedef enum CBLAS_TRANSPOSE CBLAS_TRANSPOSE;
// #endif
#include "darc.h"
#include "rtcrecon.h"
#include "buffer.h"
#include "agbcblas.h"

typedef enum{
  ASYNCCOMBINES,//which inputs are synchronous with others.
  ASYNCINITSTATE,//initial state of the actuators.
  ASYNCMULTIADDR,//list of IP addresses, separated by ;
  ASYNCNAMES,//list of the prefixes.
  ASYNCOFFSETS,//list of offsets
  ASYNCRESET,//reset it.
  ASYNCSCALES,//list of scales - multiply the inputs by scale then add offset
  ASYNCSTARTS,//flag - wait until first entry received before sending to mirror?
  ASYNCTYPES,//flag - 0==socket, 1==shm/mqueue
  ASYNCUDPIF,//list of IP addresses, separated by ;
  ASYNCUDPPORT,//list of UDP port numbers to use.
  ASYNCUPDATES,//flag - can this client cause an update? (probably 1).
  NACTS,
  POLCMX,//the polc matrix, shape nacts,nacts.
  POLCSOURCE,//where to get the polc acts from: 0=no polc, 1=as computed here, 2=returned from DM (e.g. M4), etc.
  //Add more before this line.
  RECONNBUFFERVARIABLES//equal to number of entries in the enum
}RECONBUFFERVARIABLEINDX;

#define reconMakeNames() bufferMakeNames(RECONNBUFFERVARIABLES,"asyncCombines","asyncInitState","asyncMultiAddr","asyncNames","asyncOffsets","asyncReset","asyncScales","asyncStarts","asyncTypes","asyncUdpIF","asyncUdpPort","asyncUpdates","nacts","polcMx","polcSource")

#define HDRSIZE 8 //matches that from mirrorSocket, which is where this probably gets data from... if alternatives are needed, then recode...  HDRSIZE MUST not be larger than BUFALIGN.
#define BUFALIGN 64 //Used to ensure alignment of the DM vectors is well suited to the arcitecture - in this case, use 64 for avx512 registers.  Note, this must be at least as big as HDRSIZE.  Used in posix_memalign, so must also be a power of 2.
#define UDPHDRSIZE 24 //size of UDP header.  6 ints: (0x5555<<16|nacts),fno, packetNumber(fno),npackets(fno),msgsize,offset(packet)
//#define FHDRSIZE (8/sizeof(float))
#define RECON_SHM 1
#define RECON_SOCKET 0
#define RECON_UDP 2

typedef struct{
  int threadno;
  void *rstr;
}ThreadStruct;

typedef struct{
  int *ready;
  int *combineReady;
  darc_mutex_t mutex;
  darc_cond_t cond;
  circBuf *rtcErrorBuf;
  char *paramNames;
  int index[RECONNBUFFERVARIABLES];
  void *values[RECONNBUFFERVARIABLES];
  char dtype[RECONNBUFFERVARIABLES];
  int nbytes[RECONNBUFFERVARIABLES];
  arrayStruct *arrStr;
  int nclients;
  float *outarr;
  int *bytesReceived;
  int *startFlags;
  int *started;
  int dataReady;
  int reset;
  int *discard;
  int nacts;
  int *sock;
  int lsock;
  char *inbuf;
  char **asyncNames;
  float *prev;
  float *initState;
  pthread_t *threadid;
  int *causeUpdate;
  int *combines;
  float *scales;
  float *scalesArr;
  float *offsets;
  float *offsetsArr;
  int *types;
  char **shmbuf;
  mqd_t *mq;
  int go;
  int port;
  unsigned int *threadAffinity;
  int threadAffinElSize;
  int threadPriority;
  int ftimeout;
  int nconnected;
  unsigned int *reconframeno;
  int ovrwrt;
  struct sockaddr_in *sockAddr;
  unsigned int *curframeno;
  char *udpRecvBuf;
  int udpRecvLen;
  char **multicastAddr;//strings such as "224.1.1.1" (the multicast port)
  char **udpInterface;//strings such as "192.168.1.3" (the local IP).
  int *uport;//the port number for this UDP connection
  struct pollfd *poll;
  int npoll;
  int polcSource;
  float *polcMx;
  float *polcActs;
  float *polcActsPrev;
  float *polcActsLocal;
  ThreadStruct *tstr;
  darc_barrier_t barrier;
  darc_cond_t polcCond;
  darc_mutex_t outarrMutex;
  int *polcThreadPriority;
  unsigned int *polcThreadAffinity;
  int nPolcThreads;
}ReconStruct;

void reconCloseShmQueue(ReconStruct *rstr){
  int i;
  char name[80];
  int size=rstr->nacts*sizeof(float)+HDRSIZE+sizeof(darc_mutex_t);
  strcpy(name,"/reconAsync");
  name[79]='\0';
  if(rstr->types==NULL || rstr->asyncNames==NULL )
    return;
  //remove everything...
  for(i=0; i<rstr->nclients; i++){
    if(rstr->types[i]==RECON_SHM){//this one is shm
      if(rstr->shmbuf[i]!=NULL){
	darc_mutex_destroy((darc_mutex_t*)(rstr->shmbuf[i]));
	munmap(rstr->shmbuf[i],size);
      }
      rstr->shmbuf[i]=NULL;
      if(rstr->mq[i]>0)
	mq_close(rstr->mq[i]);
      rstr->mq[i]=0;
      rstr->poll[i].fd=-1;
      if(rstr->asyncNames[i]!=NULL){
	strncpy(&name[11],rstr->asyncNames[i],68);
	shm_unlink(name);
	mq_unlink(name);
      }
    }
  }
}

int reconOpenShmQueue(ReconStruct *rstr){
  //Opens shared memory which should be written into by the childs.
  int i,err=0;
  char name[80];
  int fd;
  int size=rstr->nacts*sizeof(float)+HDRSIZE+sizeof(darc_mutex_t);
  darc_mutexattr_t mutexattr;
  struct mq_attr mqattr;
  strcpy(name,"/reconAsync");
  name[79]='\0';
  darc_mutexattr_init(&mutexattr);
  darc_mutexattr_setpshared(&mutexattr,PTHREAD_PROCESS_SHARED);//can this work with a darc_mutex type?
  mqattr.mq_flags=0;
  mqattr.mq_curmsgs=0;
  mqattr.mq_maxmsg=1;//only 1 message at a time
  mqattr.mq_msgsize=1;//one byte per message
  for(i=0; i<rstr->nclients && err==0; i++){
    if(rstr->types[i]==RECON_SHM){//this one is shm
      strncpy(&name[11],rstr->asyncNames[i],68);
      if((fd=shm_open(name,O_RDWR|O_CREAT|(O_EXCL*(1-rstr->ovrwrt)),0777))<0){
	printf("shm_open failed in reconAsync for %s\n",name);
	err=1;
      }else{
	if(ftruncate(fd,size)!=0){
	  printf("ftruncate failed in reconAsync for %s\n",name);
	  err=1;
	}else{
	  if((rstr->shmbuf[i]=mmap(0,size,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0))==MAP_FAILED){
	    printf("mmap failed in reconAsync for %s\n",name);
	    err=1;
	    rstr->shmbuf[i]=NULL;
	  }else{
	    memset(rstr->shmbuf[i],0,size);
	    //initialise the mutex (for serialising data access).
	    darc_mutex_init((darc_mutex_t*)(rstr->shmbuf[i]),darc_mutex_init_attr);
	  }
	}
      }
      close(fd);
      if(err==0){//now open the mqueue.
	if((rstr->mq[i]=mq_open(name,O_RDONLY|O_CREAT|(O_EXCL*(1-rstr->ovrwrt)),0777,&mqattr))==-1){
	  printf("mq_open failed in reconAsync for %s: %s\n",name,strerror(errno));
	  // to view existing queues:
	  // mkdir /dev/mqueue
          // mount -t mqueue none /dev/mqueue

	  err=1;
	}else{
	  rstr->poll[i].fd=rstr->mq[i];
	  rstr->poll[i].events=POLLIN;
	}
      }
    }
  }
  if(err){
    reconCloseShmQueue(rstr);
  }
  darc_mutexattr_destroy(&mutexattr);
  return err;
}

int reconOpenListeningSocket(ReconStruct *rstr){
  //opens a listening socket
  //Create the socket and set up to accept connections.
  struct sockaddr_in name;
  int err=0;
  int optval;
  int i;
  int needed=0;
  //first see if any clients will be connecting via socket.
  for(i=0; i<rstr->nclients; i++){
    if(rstr->types[i]==RECON_SOCKET){
      needed=1;
      break;
    }
  }
  
  if(needed){//open the listening socket
    rstr->npoll=rstr->nclients+1;
    
    if((rstr->lsock=socket(PF_INET,SOCK_STREAM,0))<0){
      printf("Error opening listening socket in reconAsync\n");
      rstr->lsock=0;
      err=1;
    }else{
      optval=1;
      if(setsockopt(rstr->lsock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(int))!=0){
	printf("setsockopt failed - ignoring\n");
      }
      name.sin_family=AF_INET;
      name.sin_port=htons(rstr->port);
      name.sin_addr.s_addr=htonl(INADDR_ANY);
      if(bind(rstr->lsock,(struct sockaddr*)&name,sizeof(name))<0){
	printf("Unable to bind in reconAsync\n");
	err=1;
	close(rstr->lsock);
	rstr->lsock=0;
      }
    }
    if(err==0){
      if(listen(rstr->lsock,rstr->nclients)<0){
	printf("Failed to listen on port\n");
	close(rstr->lsock);
	rstr->lsock=0;
	err=1;
      }
      rstr->poll[rstr->nclients].fd=rstr->lsock;
      rstr->poll[rstr->nclients].events=POLLIN;
    }
  }else{
    rstr->lsock=0;
    rstr->npoll=rstr->nclients;
    
  }
  return err;
}

int reconOpenUDP(ReconStruct *rstr){
  //Opens a UDP (uni or multi-cast) socket
  int err=0;
  int i;
  for(i=0; i<rstr->nclients && err==0; i++){
    if(rstr->types[i]==RECON_UDP){//this one is shm
      if((rstr->sock[i]=socket(PF_INET,SOCK_DGRAM,0))<0){
	printf("Error opening UDP socket in reconAsync\n");
	rstr->sock[i]=0;
	err=1;
      }else{
	int optval=1;
	
	if(setsockopt(rstr->sock[i], SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(int))!=0){
	  printf("setsockopt failed - ignoring\n");
	}
	rstr->sockAddr[i].sin_family=AF_INET;
	rstr->sockAddr[i].sin_port=htons(rstr->uport[i]);
	if(rstr->udpInterface[i]!=NULL)
	  inet_aton(rstr->udpInterface[i],&rstr->sockAddr[i].sin_addr);
	else
	  rstr->sockAddr[i].sin_addr.s_addr=htonl(INADDR_ANY);
	//bind to the port that we know will receive uni/multicast data.
	if(bind(rstr->sock[i],(struct sockaddr*)&rstr->sockAddr[i],sizeof(struct sockaddr_in))<0){
	  printf("Unable to bind to UDP port in reconAsync\n");
	  err=1;
	  close(rstr->sock[i]);
	  rstr->sock[i]=0;
	}
      }
      if(err==0){
	if(rstr->multicastAddr[i]!=NULL){//e.g. "224.1.1.1"
	  struct ip_mreq mreq;
	  //tell the kernel we're a multicast socket
	  //char ttl=1;
	  //setsockopt(mirstr->sock[i],IPPROTO_IP,IP_MULTICAST_TTL,&ttl,sizeof(ttl));//default is 1 anyway.
	  //subscribe to the multicast group.
	  inet_aton(rstr->multicastAddr[i],&mreq.imr_multiaddr);//e.g. "224.1.1.1"
	  if(rstr->udpInterface[i]!=NULL)
	    inet_aton(rstr->udpInterface[i],&mreq.imr_interface);//e.g. "192.168.1.2" or INADDR_ANY;
	  else
	    mreq.imr_interface.s_addr=htonl(INADDR_ANY);
	  setsockopt(rstr->sock[i],IPPROTO_IP,IP_ADD_MEMBERSHIP,&mreq,sizeof(mreq));
	}
	rstr->poll[i].fd=rstr->sock[i];
	rstr->poll[i].events=POLLIN;
	  
	rstr->bytesReceived[i]=0;
	rstr->ready[i]=0;
	rstr->combineReady[i]=0;
	rstr->started[i]=0;
	rstr->discard[i]=0;
	//rstr->nconnected++;

      }
    }
  }
  return err;
}

int reconAcceptConnection(ReconStruct *rstr){
  struct sockaddr_in clientname;
  socklen_t size;
  int err=0;
  int sock;
  int n;
  char *buf;
  int i;
  size=(socklen_t)sizeof(struct sockaddr_in);
  if((sock=accept(rstr->lsock,(struct sockaddr*)&clientname,&size))<0){
    printf("Failed to accept on socket: %s\n",strerror(errno));
    err=1;
  }else{
    printf("Connected from %s port %hd\n",inet_ntoa(clientname.sin_addr),ntohs(clientname.sin_port));
    //Now, read from the sock to get the prefix, and try to match it with those in the list.
    if(recv(sock,&n,sizeof(int),0)!=sizeof(int)){
      printf("Error reading prefix size - closing socket\n");
      err=1;
      close(sock);
    }else{
      if((buf=malloc(n+1))==NULL){
	printf("Unable to malloc temporary buffer size %d - closing socket\n",n+1);
	err=1;
	close(sock);
      }else{
	if(recv(sock,buf,n,0)!=n){
	  printf("Error reading prefix - closing socket\n");
	  err=1;
	  close(sock);
	}else{//got the prefix.
	  buf[n]='\0';
	  for(i=0; i<rstr->nclients; i++){
	    if(strncmp(buf,rstr->asyncNames[i],n)==0){//matching prefix
	      rstr->sock[i]=sock;
	      rstr->poll[i].fd=sock;
	      rstr->poll[i].events=POLLIN;
	      rstr->bytesReceived[i]=0;
	      rstr->ready[i]=0;
	      rstr->combineReady[i]=0;
	      rstr->started[i]=0;
	      rstr->discard[i]=0;
	      rstr->nconnected++;
	      printf("Prefix %s connected\n",buf);
	      break;
	    }
	  }
	  if(i==rstr->nclients){
	    printf("Didn't match prefix %s to any requested - closing socket\n",buf);
	    err=1;
	    close(sock);
	  }
	}
	free(buf);
      }
    }
  }
  return err;
}

int reconSetThreadPrioAffin(pthread_t this_thread,ReconStruct *rstr,int prio,unsigned int *affin){
  cpu_set_t mask;
  int n,i;
  struct sched_param param;
  n= sysconf(_SC_NPROCESSORS_ONLN);
  CPU_ZERO(&mask);
  for(i=0; i<n && i<rstr->threadAffinElSize*32; i++){
    if(((affin[i/32])>>(i%32))&1){
      CPU_SET(i,&mask);
    }
  }
  if(pthread_setaffinity_np(this_thread,sizeof(cpu_set_t),&mask))
    printf("Error in pthread_setaffinity_np: %s\n",strerror(errno));
  param.sched_priority=prio;
  if(pthread_setschedparam(this_thread,SCHED_RR,&param))
    printf("error in pthread_setschedparam - maybe run as root?\n");
  return 0;
}

//polc worker threads.  Each does a part of the polc matrix reconstruction.
void *polcWorker(void *threadHandle){
  ThreadStruct *tstr=(ThreadStruct*)threadHandle;
  int threadno=tstr->threadno;
  ReconStruct *rstr=tstr->rstr;
  //compute number of actuators per thread.
  int nActCompute=(rstr->nacts+rstr->nPolcThreads-1)/rstr->nPolcThreads;
  int start=nActCompute*threadno;
  int end=nActCompute*(threadno+1);
  int nacts=rstr->nacts;
  int fno;
  float *tmp;
  if(end>nacts){
    end=nacts;
  }
  if(start>nacts){
    start=nacts;
  }
  nActCompute=end-start;

  while(rstr->go){
    //wait for some data.
    //This lock allows us to read dmCommand without it being altered.
    //the writer on this lock will depend on the value of polcSource.  If polcSource==1, then the thread that writes to dmCommand shoudl get it.  If polcSource==2, it should be the thread that is receiving actuators from the DM (e.g. M4).
    //After getting the readlock, we then cond wait until polc data is ready.
    if(threadno==0){
      darc_mutex_lock(&rstr->outarrMutex);
      darc_cond_wait(&rstr->polcCond,&rstr->outarrMutex);
      //double buffer the pointers...
      tmp=rstr->polcActsPrev;
      rstr->polcActsPrev=rstr->polcActs;
      rstr->polcActs=tmp;//no need to clear it because the sgemv does that.
      //copy the array to be used for polc...
      fno=rstr->reconframeno[1];
      if(rstr->polcSource==1){
	memcpy(rstr->polcActsLocal,rstr->arrStr->dmCommand,nacts*sizeof(float));
      }else{//todo - copy from source into polcActsLocal.
	printf("todo - compute actuators based on polc from DM\n");
      }
      darc_mutex_unlock(&rstr->outarrMutex);
    }
    if(rstr->go){
      darc_barrier_wait(&rstr->barrier);
      if(nActCompute!=0){
	agb_cblas_sgemvRowMN1N101(nActCompute,nacts,&rstr->polcMx[start*nacts],rstr->polcActsLocal,&rstr->polcActs[start]);
      }
      darc_barrier_wait(&rstr->barrier);
      if(threadno==0){//first thread puts result in correct place.
	darc_mutex_lock(&rstr->outarrMutex);
	//first subtract the previous polc value
	agb_cblas_saxpym111(nacts,rstr->polcActsPrev,rstr->outarr);
	//and now add the new values.
	agb_cblas_saxpy111(nacts,rstr->polcActs,rstr->outarr);
	rstr->reconframeno[rstr->nclients]=rstr->reconframeno[1]-fno;
	darc_mutex_unlock(&rstr->outarrMutex);
      }
    }
  }
  return NULL;
}

  


void *reconWorker(void *reconHandle){
  ReconStruct *rstr=(ReconStruct*)reconHandle;
  float offset,scale;
  float *offsets,*scales,*inbuf;
  char *cinbuf;
  //fd_set fdset;
  //int max;
  int j,i;
  int n;
  //struct timeval timeout;
  int doupdate;
  unsigned int *hdr;
  int ready,update;
  char msg[2]="\0\0";
  //timeout.tv_sec=1;
  //timeout.tv_usec=0;
  darc_mutex_lock(&rstr->mutex);
  printf("reconWorker starting loop\n");
  while(rstr->go){
    darc_mutex_unlock(&rstr->mutex);
    /*FD_ZERO(&fdset);
    max=rstr->lsock;
    if(rstr->lsock>0)//there is a listening socket
      FD_SET(rstr->lsock,&fdset);
    for(i=0; i<rstr->nclients; i++){
      if(rstr->types[i]==RECON_SHM){
	if(rstr->mq[i]>0){
	  FD_SET(rstr->mq[i],&fdset);
	  if(rstr->mq[i]>max)
	    max=rstr->mq[i];
	}
      }else{//socket or UDP type.
	if(rstr->sock[i]!=0){
	  printf("Setting %d %d\n",i,rstr->sock[i]);
	  FD_SET(rstr->sock[i],&fdset);
	  if(rstr->sock[i]>max)
	    max=rstr->sock[i];
	}
      }
      }*/
    //rstr->poll[i].events=POLLIN
    //timeout.tv_sec=1;
    //timeout.tv_usec=0;
    //n=select(max+1,&fdset,NULL,NULL,&timeout);
    n=poll(rstr->poll,rstr->npoll,1000);
    //printf("select done %d %d\n",n,max);
    darc_mutex_lock(&rstr->mutex);
    if(rstr->reset){
      if(rstr->initState!=NULL)
	memcpy(rstr->outarr,rstr->initState,sizeof(float)*rstr->nacts);
      else
	memset(rstr->outarr,0,sizeof(float)*rstr->nacts);
      memset(rstr->prev,0,(((sizeof(float)*rstr->nacts+BUFALIGN-1)/BUFALIGN)*BUFALIGN)*rstr->nclients);
      memset(rstr->started,0,sizeof(int)*rstr->nclients);
      rstr->dataReady=1;
      doupdate=1;
      //rstr->reset=0;
    }
    if(n==0){
      //timeout - wake the processing thread so it can continue, so the rtc doesn't become frozen.   It will just send the existing actuators to the mirror.
      printf("Timeout in reconAsync (no data arrived)\n");
      darc_cond_broadcast(&rstr->cond);//wake the processing thread.
    }else if(n<0){
      //select error
      printf("Error in select in reconAsync\n");
      darc_cond_broadcast(&rstr->cond);//wake the processing thread
      darc_mutex_unlock(&rstr->mutex);
      sleep(1);
      darc_mutex_lock(&rstr->mutex);
    }else{//data ready to read.
      for(i=0; i<rstr->nclients; i++){
	if(rstr->reset && rstr->bytesReceived[i]>0){
	  rstr->discard[i]=1;//in reset, so need to discard this frame.
	}
	//printf("here %d %d=%d, sock %d, set %d, received %d\n",i,rstr->types[i],RECON_UDP,rstr->sock[i],rstr->poll[i].revents,rstr->bytesReceived[i]);
	if(rstr->types[i]==RECON_SHM){//shm type
	  if(rstr->poll[i].revents){//FD_ISSET(rstr->mq[i],&fdset)){//all bytes have been copied into shm
	    if(mq_receive(rstr->mq[i],msg,2,NULL)<0){
	      printf("Error in reconAsync mq_receive\n");
	    }else
	      rstr->bytesReceived[i]=rstr->nacts*sizeof(float);
	  }
	}else if(rstr->types[i]==RECON_SOCKET){//socket type - read the socket
	  if(rstr->sock[i]>0 && rstr->poll[i].revents && rstr->bytesReceived[i]<rstr->nacts*sizeof(float)+HDRSIZE){
	    //data is ready, and can be read.
	    cinbuf=&rstr->inbuf[i*(BUFALIGN+((rstr->nacts*sizeof(float)+BUFALIGN-1)/BUFALIGN)*BUFALIGN)+BUFALIGN-HDRSIZE];
	    if((n=recv(rstr->sock[i],&cinbuf[rstr->bytesReceived[i]],rstr->nacts*sizeof(float)+HDRSIZE-rstr->bytesReceived[i],0))<=0){
	      //no data received - error - so close socket.
	      printf("Closing socket %d (ret=%d, sock %d)\n",i,n,rstr->sock[i]);
	      if(n<0)
		printf("%s\n",strerror(errno));
	      close(rstr->sock[i]);
	      rstr->poll[i].fd=-1;
	      rstr->sock[i]=0;
	      rstr->nconnected--;
	    }else{
	      rstr->bytesReceived[i]+=n;//Note - total bytesReceived here will be nacts*sizeof(float)+HDRSIZE once a full frame has been received.
	    }
	  }
	}else if(rstr->types[i]==RECON_UDP){
	  if(rstr->sock[i]>0 && rstr->poll[i].revents){// && rstr->bytesReceived[i]<rstr->nacts*sizeof(float)){
	    //data is ready and can be read.
	    size_t addrlen=sizeof(struct sockaddr_in);
	    //printf("recv %d\n",i);
	    if((n=recvfrom(rstr->sock[i],&rstr->udpRecvBuf[i*rstr->udpRecvLen],rstr->udpRecvLen,0,&rstr->sockAddr[i],(socklen_t*)&addrlen))<=0){
	      //no data received - error.
	      printf("Error on UDP socket - not sure what to do!\n");
	      if(n<0)
		printf("%s\n",strerror(errno));
	      close(rstr->sock[i]);
	      rstr->poll[i].fd=-1;
	      rstr->sock[i]=0;
	      rstr->nconnected--;
	    }else{
	      //int jj;
	      //The received data (from mirrorUDP.c) should have a UDPHDRSIZE byte header (i.e. 6 ints):
	      //(0x5555<<17 | nacts), fno, packetNumber(fno),npackets(fno),msgsize,offset(packet)
	      unsigned int *hdr=(unsigned int*)(&rstr->udpRecvBuf[i*rstr->udpRecvLen]);
	      unsigned int tag=hdr[0];//0x5555<<17 | nacts
	      unsigned int fno=hdr[1];
	      int offset=hdr[5];
	      int bytes=hdr[4];
	      //for(jj=0;jj<6;jj++)
	      //printf("%d ",hdr[jj]);
	      if(rstr->bytesReceived[i]==0)
		rstr->curframeno[i]=fno;
	      if(tag!=(0x5555<<17|rstr->nacts)){
		printf("Error - wrong UDP header received, got %#x, should be %#x\n",tag,(0x5555<<17|rstr->nacts));
	      }else if(bytes+offset>sizeof(float)*rstr->nacts){
		printf("Data size too large:  bytes=%d, offset=%d, but tot size = %ld\n",bytes,offset,sizeof(float)*rstr->nacts);
	      }else if(n!=UDPHDRSIZE+bytes){
		printf("reconAsync: Mismatch between bytes received and that reported in header\n");
	      }else{//data received okay.  So copy into buffer.
		if(fno!=rstr->curframeno[i]){// && rstr->packetCnt[i]!=0){
		  printf("client %d UDP frame number %d != current frame %d - skipping\n",i,fno,rstr->curframeno[i]);
		  //rstr->packetCnt[i]=0;
		  rstr->bytesReceived[i]=0;
		  rstr->curframeno[i]=fno;
		}
		//rstr->packetCnt[i]++;
		cinbuf=&rstr->inbuf[i*(BUFALIGN+((rstr->nacts*sizeof(float)+BUFALIGN-1)/BUFALIGN)*BUFALIGN)+BUFALIGN];//skip the header.
		//printf("%ld ",i*(BUFALIGN+((rstr->nacts*sizeof(float)+BUFALIGN-1)/BUFALIGN)*BUFALIGN)+BUFALIGN);
		//for(jj=0;jj<4;jj++)
		//printf("%g ",((float*)cinbuf)[jj]);
		  
		memcpy(&cinbuf[offset],&hdr[6],bytes);
		rstr->bytesReceived[i]+=n-UDPHDRSIZE;//don't include the header here.
		hdr=(unsigned int*)&rstr->inbuf[i*(BUFALIGN+((rstr->nacts*sizeof(float)+BUFALIGN-1)/BUFALIGN)*BUFALIGN)+BUFALIGN-HDRSIZE];//&rstr->inbuf[i*(rstr->nacts+FHDRSIZE)];
		hdr[0]=tag;//0x5555<<17|(rstr->bytesReceived[i]/sizeof(float));
		hdr[1]=fno;
	      }
	    }
	  }
	}
      }
      doupdate=0;
      for(i=0; i<rstr->nclients; i++){
	if(rstr->bytesReceived[i]>=rstr->nacts*sizeof(float)){
	  if(rstr->discard[i]){
	    rstr->discard[i]=0;
	    rstr->bytesReceived[i]=0;
	  }else{
	    rstr->ready[i]=1;
	    rstr->combineReady[i]=1;
	    //a full set of actuator demands has been received.
	    if(rstr->combines[i]>=0){
	      //this one is synchronous with others... only if they are all ready can we continue.
	      ready=1;
	      update=0;
	      for(j=0; j<rstr->nclients; j++){
		//printf("%d %d %d\n",i,j,rstr->bytesReceived[j]);
		if((j==rstr->combines[i] || rstr->combines[j]==rstr->combines[i])){
		  if(rstr->combineReady[j]){
		    //if at least one of them can cause an update, set update.
		    update+=rstr->causeUpdate[i];
		    update+=rstr->causeUpdate[j];
		  }else{
		    ready=0;
		  }
		}
	      }
	      if(ready){
		if(update>0)//at least one has causeUpdate set - so can update.
		  doupdate=1;
		
	      }
	    }else{//not synchronous with any - so ready...
	      if(rstr->causeUpdate[i])//and it can cause an update.
		doupdate=1;
	    }
	  }
	}
      }
      ready=0;
      for(i=0; i<rstr->nclients; i++){
	if(rstr->ready[i]){
	  ready++;
	  //now point inbuf to the start of the data.
	  if(rstr->types[i]==RECON_SHM){
	    darc_mutex_lock((darc_mutex_t*)(rstr->shmbuf[i]));
	    inbuf=(float*)(rstr->shmbuf[i]+sizeof(darc_mutex_t)+HDRSIZE);
	    hdr=(unsigned int*)(rstr->shmbuf[i]+sizeof(darc_mutex_t));
	  }else if(rstr->types[i]==RECON_SOCKET){
	    inbuf=(float*)&rstr->inbuf[i*(BUFALIGN+((rstr->nacts*sizeof(float)+BUFALIGN-1)/BUFALIGN)*BUFALIGN)+BUFALIGN];//skip the header.
	    //&rstr->inbuf[i*(rstr->nacts+FHDRSIZE)+FHDRSIZE];
	    hdr=(unsigned int*)&rstr->inbuf[i*(BUFALIGN+((rstr->nacts*sizeof(float)+BUFALIGN-1)/BUFALIGN)*BUFALIGN)+BUFALIGN-HDRSIZE];//(&rstr->inbuf[i*(rstr->nacts+FHDRSIZE)]);
	  }else if(rstr->types[i]==RECON_UDP){
	    inbuf=(float*)&rstr->inbuf[i*(BUFALIGN+((rstr->nacts*sizeof(float)+BUFALIGN-1)/BUFALIGN)*BUFALIGN)+BUFALIGN];//skip the header.
	    //&rstr->inbuf[i*(rstr->nacts+FHDRSIZE)+FHDRSIZE];
	    hdr=(unsigned int*)&rstr->inbuf[i*(BUFALIGN+((rstr->nacts*sizeof(float)+BUFALIGN-1)/BUFALIGN)*BUFALIGN)+BUFALIGN-HDRSIZE];//(&rstr->inbuf[i*(rstr->nacts+FHDRSIZE)]);
	    //printf("%d %ld\n",i,i*(BUFALIGN+((rstr->nacts*sizeof(float)+BUFALIGN-1)/BUFALIGN)*BUFALIGN)+BUFALIGN);
	  }else{
	    printf("Unknown type\n");
	    hdr=NULL;
	    inbuf=NULL;
	  }
	  //first, subtract the previous value for this client, then add the current value, and store the current value.
	  if(hdr[0]!=(0x5555<<17|rstr->nacts)){
	    printf("ERROR - data header not what expected: %u %u\n",hdr[0],(0x5555<<17|rstr->nacts));
	  }else if(rstr->reset==0){
	    float *prev=&rstr->prev[i*(((sizeof(float)*rstr->nacts+BUFALIGN-1)/BUFALIGN)*BUFALIGN)/sizeof(float)];
	    darc_mutex_lock(&rstr->outarrMutex);
	    #ifdef USEMKL
	    cblas_saxpy(rstr->nacts,-1.0,prev,1,rstr->outarr,1);
	    #elif defined(USEAGBBLAS)
	    agb_cblas_saxpym111(rstr->nacts,prev,rstr->outarr);//outarr-=prev
	    #endif 
	    darc_mutex_unlock(&rstr->outarrMutex);
	    if(rstr->offsets==NULL){
	      if(rstr->offsetsArr==NULL){
		if(rstr->scales==NULL){
		  if(rstr->scalesArr==NULL){
		    memcpy(prev,inbuf,sizeof(float)*rstr->nacts);
		  }else{//scale for each actuator
		    //inbuf=&rstr->inbuf[i*(rstr->nacts+FHDRSIZE)+FHDRSIZE];
		    scales=&rstr->scalesArr[i*rstr->nacts];
		    for(j=0; j<rstr->nacts; j++)
		      prev[j]=inbuf[j]*scales[j];
		  }
		}else{//single scale
		  //inbuf=&rstr->inbuf[i*(rstr->nacts+FHDRSIZE)+FHDRSIZE];
		  scale=rstr->scales[i];
		  for(j=0; j<rstr->nacts; j++)
		    prev[j]=inbuf[j]*scale;
		}
	      }else{//offset for each actuator
		//inbuf=&rstr->inbuf[i*(rstr->nacts+FHDRSIZE)+FHDRSIZE];
		offsets=&rstr->offsetsArr[i*rstr->nacts];
		if(rstr->scales==NULL){
		  if(rstr->scalesArr==NULL){
		    for(j=0; j<rstr->nacts; j++)
		      prev[j]=inbuf[j]+offsets[j];
		  }else{//scale for each actuator
		    scales=&rstr->scalesArr[i*rstr->nacts];
		    for(j=0; j<rstr->nacts; j++)
		      prev[j]=inbuf[j]*scales[j]+offsets[j];
		  }
		}else{//single scale
		  scale=rstr->scales[i];
		  for(j=0; j<rstr->nacts; j++)
		    prev[j]=inbuf[j]*scale+offsets[j];
		}
	      }
	    }else{//single offset.
	      //inbuf=&rstr->inbuf[i*(rstr->nacts+FHDRSIZE)+FHDRSIZE];
	      offset=rstr->offsets[i];
	      if(rstr->scales==NULL){
		if(rstr->scalesArr==NULL){
		  for(j=0; j<rstr->nacts; j++)
		    prev[j]=inbuf[j]+offset;
		}else{//scale for each actuator
		  scales=&rstr->scalesArr[i*rstr->nacts];
		  for(j=0; j<rstr->nacts; j++)
		    prev[j]=inbuf[j]*scales[j]+offset;
		}
	      }else{//single scale
		scale=rstr->scales[i];
		for(j=0; j<rstr->nacts; j++)
		  prev[j]=inbuf[j]*scale+offset;
	      }
	    }
	    //now add current. (which has been placed into prev): y+=x
	    darc_mutex_lock(&rstr->outarrMutex);
	    #ifdef USEMKL
	    cblas_saxpy(rstr->nacts,1.0,prev,1,rstr->outarr,1);
	    #elif defined(USEAGBBLAS)
	    agb_cblas_saxpy111(rstr->nacts,prev,rstr->outarr);
	    #endif
	    darc_mutex_unlock(&rstr->outarrMutex);
	    rstr->started[i]=1;
	  }
	  rstr->ready[i]=0;
	  rstr->bytesReceived[i]=0;
	  rstr->reconframeno[i]=hdr[1];//((unsigned int*)rstr->inbuf)[i*(rstr->nacts+FHDRSIZE)+1];
	  if(rstr->types[i]==RECON_SHM){
	    darc_mutex_unlock((darc_mutex_t*)(rstr->shmbuf[i]));
	  }
	}
	if(rstr->startFlags[i]==1 && rstr->started[i]==0){
	  //Data is needed for this one, but it hasn't yet produced any - so the outarr is currently invalid - so we shouldn't send the data.
	  ready=-rstr->nclients;//set it low enough to be negative.
	}
      }
      if((ready>0 && doupdate==1) || rstr->dataReady==1){//something is ready... so wake the processing thread.  This happens if we're in reset (in which case rstr->dataReady will be 1), or if we're actually ready.  But we also need to consider the case that one child darc isn't sending data (maybe has never sent data) and e.g. startFlags is one for this child or it is the only child with causeUpdate set.  In this case, if we took no action, this darc would appear to freeze.  So, we need to set a maximum timeout after which it will wake... actually wil will handle this in the pthread_cond_wait by using a timedwait instead... probably better.
	rstr->dataReady=1;
	darc_cond_broadcast(&rstr->cond);
  // reset combineReady for all
	for(i=0; i<rstr->nclients; i++){
	  rstr->combineReady[i] = 0;
	}
      }
      darc_mutex_unlock(&rstr->mutex);
      if(rstr->lsock>0 && rstr->poll[rstr->nclients].revents/*FD_ISSET(rstr->lsock,&fdset)*/ && rstr->nconnected<rstr->nclients){
	reconAcceptConnection(rstr);
      }
      darc_mutex_lock(&rstr->mutex);

    }
  }
  darc_cond_broadcast(&rstr->cond);
  darc_mutex_unlock(&rstr->mutex);
  printf("ReconWorker finished\n");
  return NULL;
}


/**
   Called to free the reconstructor module when it is being closed.
*/
#define SAFEFREE(a) if(a!=NULL){free(a);}
int reconClose(void **reconHandle){//reconHandle is &globals->rstr.
  ReconStruct *rstr=(ReconStruct*)*reconHandle;
  int i;
  printf("Closing reconlibrary\n");
  if(rstr!=NULL){
    rstr->go=0;
    if(rstr->threadid!=NULL){
      if(rstr->threadid[0]!=0){
	pthread_join(rstr->threadid[0],NULL);
      }
      for(i=0;i<rstr->nPolcThreads;i++){
	pthread_join(rstr->threadid[i+1],NULL);
      }
      free(rstr->threadid);
      rstr->threadid=NULL;
    }
    if(rstr->tstr!=NULL){
      free(rstr->tstr);
    }
    if(rstr->paramNames!=NULL)
      free(rstr->paramNames);

    darc_cond_broadcast(&rstr->cond);
    darc_mutex_destroy(&rstr->mutex);
    darc_cond_destroy(&rstr->cond);
    darc_cond_broadcast(&rstr->polcCond);
    darc_mutex_destroy(&rstr->outarrMutex);
    darc_cond_destroy(&rstr->polcCond);
    darc_barrier_destroy(&rstr->barrier);
    for(i=0; i<rstr->nclients; i++){
      if(rstr->sock[i]!=0)
	close(rstr->sock[i]);
    }
    close(rstr->lsock);
    reconCloseShmQueue(rstr);
    SAFEFREE(rstr->shmbuf);
    SAFEFREE(rstr->mq);
    SAFEFREE(rstr->outarr);
    SAFEFREE(rstr->inbuf);
    SAFEFREE(rstr->prev);
    SAFEFREE(rstr->sock);
    SAFEFREE(rstr->started);
    SAFEFREE(rstr->bytesReceived);
    SAFEFREE(rstr->ready);
    SAFEFREE(rstr->combineReady);
    SAFEFREE(rstr->discard);
    SAFEFREE(rstr->sockAddr);
    SAFEFREE(rstr->polcActs);
    SAFEFREE(rstr->polcActsPrev);
    SAFEFREE(rstr->polcActsLocal);
    free(rstr);
  }
  *reconHandle=NULL;
  printf("Finished reconClose\n");
  return 0;
}

/**
   Called asynchronously, whenever new parameters are ready.
   Once this returns, a call to swap buffers will be issued.
   (actually, at the moment, this is called synchronously by first thread when a buffer swap is done).
*/
int reconNewParam(void *reconHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr,int totCents){
  int j=0,err=0;
  int nb;
  //globalStruct *globals=threadInfo->globals;
  //infoStruct *info=threadInfo->info;//totcents and nacts
  ReconStruct *rstr=(ReconStruct*)reconHandle;//threadInfo->globals->rstr;
  //int *indx=rstr->bufindx;
  //Use the buffer not currently in use.
  int i;
  int nfound;
  int *nbytes=rstr->nbytes;
  void **values=rstr->values;
  char *dtype=rstr->dtype;
  int *index=rstr->index;
  int pos;
  char *ptr,*semi;
  rstr->arrStr=arr;
  //swap the buffers...
  nfound=bufferGetIndex(pbuf,RECONNBUFFERVARIABLES,rstr->paramNames,index,values,dtype,nbytes);
  if(nfound!=RECONNBUFFERVARIABLES){
    err=-1;
    printf("Didn't get all buffer entries for reconAsync module:\n");
    for(j=0; j<RECONNBUFFERVARIABLES; j++){
      if(rstr->index[j]<0)
	printf("Missing %16s\n",&rstr->paramNames[j*BUFNAMESIZE]);
    }
  }
  darc_mutex_lock(&rstr->mutex);
  if(err==0){
    if(index[NACTS]>=0 && dtype[NACTS]=='i' && nbytes[NACTS]==4){
      if(rstr->nacts==0)
	rstr->nacts=*((int*)values[NACTS]);
      else if(rstr->nacts!=*((int*)values[NACTS])){
	printf("nacts changed error - please reopen reconAsync library\n");
	writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"nacts error");
	err=1;
      }
    }else{
      printf("nacts error\n");
      writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"nacts error");
      err=1;
    }
    if(index[ASYNCINITSTATE]>=0 && nbytes[ASYNCINITSTATE]==sizeof(float)*rstr->nacts && dtype[ASYNCINITSTATE]=='f'){
      rstr->initState=(float*)values[ASYNCINITSTATE];
    }else if(index[ASYNCINITSTATE]<0 || nbytes[ASYNCINITSTATE]==0){
      rstr->initState=NULL;
    }else{
      rstr->initState=NULL;
      printf("asyncInitState error\n");
      writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"asyncInitState error\n");
      err=1;
    }
    if(index[ASYNCRESET]>=0 && nbytes[ASYNCRESET]==sizeof(int) && dtype[ASYNCRESET]=='i'){
      rstr->reset=*((int*)values[ASYNCRESET]);
    }else{
      printf("asyncReset error\n");
      writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"asyncReset error\n");
      err=1;
    }
    if(index[ASYNCNAMES]>=0){
      nb=nbytes[ASYNCNAMES];
      pos=0;
      for(i=0; i<rstr->nclients && pos<nb; i++){
	rstr->asyncNames[i]=&(((char*)values[ASYNCNAMES])[pos]);
	pos+=strnlen(rstr->asyncNames[i],nb-pos)+1;
      }
      if(pos>nb){
	printf("asyncNames error - format not understood\n");
	err=1;
	writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"asyncNames error");
      }
    }else{
      writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"asyncNames error\n");
      err=1;
    }
    if(index[ASYNCOFFSETS]>=0){
      nb=nbytes[ASYNCOFFSETS];
      if(nb==0){
	rstr->offsets=NULL;
	rstr->offsetsArr=NULL;
      }else if(dtype[ASYNCOFFSETS]=='f'){
	if(nb==rstr->nclients*sizeof(float)){
	  rstr->offsets=(float*)values[ASYNCOFFSETS];
	  rstr->offsetsArr=NULL;
	}else if(nb==rstr->nclients*rstr->nacts*sizeof(float)){
	  rstr->offsetsArr=(float*)values[ASYNCOFFSETS];
	  rstr->offsets=NULL;
	}else{
	  printf("asyncOffsets error\n");
	  err=1;
	  writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"asyncOffsets error\n");
	}
      }else{
	printf("asyncOffsets error\n");
	err=1;
	writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"asyncOffsets error\n");
      }
    }else{
      rstr->offsets=NULL;
      rstr->offsetsArr=NULL;
      printf("No asyncOffsets used\n");
    }
    if(index[ASYNCSCALES]>=0){
      nb=nbytes[ASYNCSCALES];
      if(nb==0){
	rstr->scales=NULL;
	rstr->scalesArr=NULL;
      }else if(dtype[ASYNCSCALES]=='f'){
	if(nb==rstr->nclients*sizeof(float)){
	  rstr->scales=(float*)values[ASYNCSCALES];
	  rstr->scalesArr=NULL;
	}else if(nb==rstr->nclients*rstr->nacts*sizeof(float)){
	  rstr->scalesArr=(float*)values[ASYNCSCALES];
	  rstr->scales=NULL;
	}else{
	  printf("asyncScales error\n");
	  err=1;
	  writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"asyncScales error\n");
	}
      }else{
	printf("asyncScales error\n");
	err=1;
	writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"asyncScales error\n");
      }
    }else{
      rstr->scales=NULL;
      rstr->scalesArr=NULL;
      printf("no asyncScales used\n");
    }
    if(index[ASYNCSTARTS]>=0 && dtype[ASYNCSTARTS]=='i' && nbytes[ASYNCSTARTS]==sizeof(int)*rstr->nclients){
      rstr->startFlags=(int*)values[ASYNCSTARTS];
    }else{
      rstr->startFlags=NULL;
      printf("asyncStarts error\n");
      err=1;
      writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"asyncStarts error\n");
    }
    if(index[ASYNCCOMBINES]>=0 && dtype[ASYNCCOMBINES]=='i' && nbytes[ASYNCCOMBINES]==sizeof(int)*rstr->nclients){
      rstr->combines=(int*)values[ASYNCCOMBINES];
    }else{
      printf("asyncCombines error\n");
      rstr->combines=NULL;
      err=1;
      writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"asyncCombines error\n");
    }
    if(index[ASYNCTYPES]>=0 && dtype[ASYNCTYPES]=='i' && nbytes[ASYNCTYPES]==sizeof(int)*rstr->nclients){
      rstr->types=(int*)values[ASYNCTYPES];
    }else{
      printf("asyncTypes error\n");
      rstr->types=NULL;
      err=1;
      writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"asyncTypes error\n");
    }
    if(index[ASYNCUPDATES]>=0 && dtype[ASYNCUPDATES]=='i' && nbytes[ASYNCUPDATES]==sizeof(int)*rstr->nclients){
      rstr->causeUpdate=(int*)values[ASYNCUPDATES];
    }else{
      printf("asyncUpdates error\n");
      rstr->causeUpdate=NULL;
      err=1;
      writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"asyncUpdates error\n");
    }
    if(index[ASYNCUDPPORT]>=0 && dtype[ASYNCUDPPORT]=='i' && nbytes[ASYNCUDPPORT]==sizeof(int)*rstr->nclients){
      rstr->uport=(int*)values[ASYNCUDPPORT];
    }else{
      printf("asyncUdpPort error\n");
      rstr->uport=NULL;
      err=1;
      writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"asyncUdpPort error\n");
    }
    if(index[ASYNCUDPIF]>=0){
      if(dtype[ASYNCUDPIF]=='s'){
	i=0;
	if(rstr->udpInterface[0]!=NULL)
	  free(rstr->udpInterface[0]);
	ptr=strndup((char*)values[ASYNCUDPIF],nbytes[ASYNCUDPIF]);
	while(i<rstr->nclients){
	  semi=strchr(ptr,';');
	  rstr->udpInterface[i]=ptr;
	  if(semi!=NULL){
	    semi[0]='\0';
	    ptr=semi+1;//move to start of next command.
	  }else if(i<rstr->nclients-1){
	    printf("Not found enough UDP interface addresses in asyncUdpIf\n");
	    break;
	  }
	  printf("UDP interface %d: %s\n",i,rstr->udpInterface[i]);
	  i++;
	}
      }else if(dtype[ASYNCUDPIF]=='n'){
	if(rstr->udpInterface[0]!=NULL)
	  free(rstr->udpInterface[0]);
	for(i=0;i<rstr->nclients;i++)
	  rstr->udpInterface[i]=NULL;
      }else{
	printf("asyncUdpIF error\n");
	err=1;
	writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"asyncUdpIF error\n");
      }
    }else{
      printf("no asyncUdpIF defined\n");
      if(rstr->udpInterface[0]!=NULL)
	free(rstr->udpInterface[0]);
      for(i=0;i<rstr->nclients;i++)
	rstr->udpInterface[i]=NULL;
    }     
    if(index[ASYNCMULTIADDR]>=0){
      if(dtype[ASYNCMULTIADDR]=='s'){
	i=0;
	if(rstr->multicastAddr[0]!=NULL)
	  free(rstr->multicastAddr[0]);
	ptr=strndup((char*)values[ASYNCMULTIADDR],nbytes[ASYNCMULTIADDR]);
	while(i<rstr->nclients){
	  semi=strchr(ptr,';');
	  rstr->multicastAddr[i]=ptr;
	  if(semi!=NULL){
	    semi[0]='\0';
	    ptr=semi+1;//move to start of next command.
	  }else if(i<rstr->nclients-1){
	    printf("Not found enough UDP interface addresses in asyncmultiaddr\n");
	    err=1;
	    break;
	  }
	  printf("Multicast address %d: %s\n",i,rstr->multicastAddr[i]);
	  i++;
	}
      }else if(dtype[ASYNCMULTIADDR]=='n'){
	if(rstr->multicastAddr[0]!=NULL)
	  free(rstr->multicastAddr[0]);
	for(i=0;i<rstr->nclients;i++)
	  rstr->multicastAddr[i]=NULL;
      }else{
	printf("asyncmultiaddr error\n");
	err=1;
	writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"asyncmultiaddr error\n");
      }
    }else{
      printf("No asyncmultiaddr defined\n");
      if(rstr->multicastAddr[0]!=NULL)
	free(rstr->multicastAddr[0]);
      for(i=0;i<rstr->nclients;i++)
	rstr->multicastAddr[i]=NULL;
    }
    if(index[POLCMX]>=0){
      if(dtype[POLCMX]=='f' && nbytes[POLCMX]==sizeof(float)*rstr->nacts*rstr->nacts){
	rstr->polcMx=(float*)values[POLCMX];
      }else{
	rstr->polcMx=NULL;
	err=0;
	writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"polcMx error\n");
      }
    }else{
      rstr->polcMx=NULL;
      printf("No polcMx - not doing POLC\n");
    }
    if(rstr->polcMx!=NULL && index[POLCSOURCE]>=0){
      if(dtype[POLCSOURCE]=='i' && nbytes[POLCSOURCE]==sizeof(int)){
	rstr->polcSource=*((int*)values[POLCSOURCE]);
      }else{
	rstr->polcSource=0;
	writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"polcSource error\n");
	err=1;
      }
    }else{
      printf("No polcSource - not doing POLC\n");
      rstr->polcSource=0;
    }
  }
  darc_mutex_unlock(&rstr->mutex);
  //rstr->swap=1;
  return err;
}

/**
   Initialise the reconstructor module
 */
int reconOpen(char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **reconHandle,int nthreads,unsigned int frameno,unsigned int **reconframeno,int *reconframenoSize,int totCents){
  //Sort through the parameter buffer, and get the things we need, and do 
  //the allocations we need.
  ReconStruct *rstr;
  //ReconStructEntry *rs;
  int err=0,i;
  if(n<=7 || n!=8+args[2]+args[7+args[2]]*(1+args[2])){
    printf("Error - args for reconAsync should be: number of async clients, tcp listening port, Naffin, thread priority, timeout (in ms), overwrite flag (use 0 unless you know what it does),thread affinity[Naffin],udpRecvLen,npolcThreads,polcPriority[n],polcAffinity[n*elsize]\n");
    *reconHandle=NULL;
    return 1;
  }
  if(sizeof(darc_mutex_t)%4!=0){
    printf("Error - bad assumptions made in reconAsync - recode needed\n");
    *reconHandle=NULL;
    return 1;
  }
  if((rstr=calloc(sizeof(ReconStruct),1))==NULL){
    printf("Error allocating recon memory\n");
    *reconHandle=NULL;
    //threadInfo->globals->rstr=NULL;
    return 1;
  }
  //threadInfo->globals->rstr=(void*)rstr;
  *reconHandle=(void*)rstr;
  rstr->go=1;
  rstr->nclients=args[0];
  rstr->port=args[1];
  rstr->threadAffinElSize=args[2];
  rstr->threadPriority=args[3];
  rstr->ftimeout=args[4]/1000.;
  rstr->ovrwrt=args[5];
  rstr->threadAffinity=(unsigned int*)&args[6];
  rstr->arrStr=arr;
  rstr->udpRecvLen=args[6+rstr->threadAffinElSize];
  rstr->nPolcThreads=args[7+rstr->threadAffinElSize];
  rstr->polcThreadPriority=(int*)&args[8+rstr->threadAffinElSize];
  rstr->polcThreadAffinity=(unsigned int*)&args[8+rstr->threadAffinElSize+rstr->nPolcThreads];

  if(*reconframenoSize<(rstr->nclients+1)){
    if(*reconframeno!=NULL)
      free(*reconframeno);
    if((*reconframeno=malloc(sizeof(unsigned int)*(rstr->nclients+1)))==NULL){
      printf("Error allocating frame numbers for reconAsync\n");
      reconClose(reconHandle);
      *reconHandle=NULL;
      return 1;
    }else{
      *reconframenoSize=rstr->nclients+1;
    }
  }
  rstr->reconframeno=*reconframeno;
  //rstr->nthreads=nthreads;//this doesn't change.
  rstr->rtcErrorBuf=rtcErrorBuf;
  rstr->paramNames=reconMakeNames();
  if((rstr->asyncNames=malloc(rstr->nclients*sizeof(char*)))==NULL){
    printf("Error allocating asyncNames pointers\n");
    err=-1;
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  //the condition variable and mutex don't need to be buffer swaped...
  if(darc_mutex_init(&rstr->mutex,darc_mutex_init_NULL)){
    printf("Error init recon mutex\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if(darc_mutex_init(&rstr->outarrMutex,darc_mutex_init_NULL)){
    printf("Error init recon outarrMutex\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if(darc_cond_init(&rstr->cond,NULL)){
    printf("Error init recon cond\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if(darc_cond_init(&rstr->polcCond,NULL)){
    printf("Error init recon polcCond\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if(darc_barrier_init(&rstr->barrier,NULL,rstr->nPolcThreads)){
    printf("Error init barrier in reconAsync\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if((rstr->poll=calloc(sizeof(struct pollfd),rstr->nclients+1))==NULL){
    printf("Unable to alloc reconAsync poll\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if((rstr->multicastAddr=calloc(sizeof(char*),rstr->nclients))==NULL){
    printf("Unable to alloc reconAsync multicastAddr\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if((rstr->udpInterface=calloc(sizeof(char*),rstr->nclients))==NULL){
    printf("Unable to alloc reconAsync udpInterface\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  err=reconNewParam(*reconHandle,pbuf,frameno,arr,totCents);//this will change ->buf to 0.
  //rs->swap=0;//no - we don't need to swap.
  //rs=&rstr->rs[rstr->buf];
  if(err!=0){
    printf("Error in reconAsync...\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if((rstr->outarr=malloc(sizeof(float)*rstr->nacts))==NULL){
    printf("unable to malloc reconAsync outarr\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if((rstr->polcActs=calloc(sizeof(float),rstr->nacts))==NULL){
    printf("Unable to alloc polcActs arr\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  } 
  if((rstr->polcActsPrev=calloc(sizeof(float),rstr->nacts))==NULL){
    printf("Unable to alloc polcActsPrev arr\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if((rstr->polcActsLocal=calloc(sizeof(float),rstr->nacts))==NULL){
    printf("Unable to alloc polcActsLocal arr\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
 
  //Want to make sure that inbuf[HDRSIZE], inbuf[HDRSIZE+step] etc is well aligned so that the dm vector additions can be done efficiently.
  if(posix_memalign((void**)&rstr->inbuf,BUFALIGN,(BUFALIGN+((sizeof(float)*rstr->nacts+BUFALIGN-1)/BUFALIGN)*BUFALIGN)*rstr->nclients)!=0){
    //if((rstr->inbuf=malloc((BUFALIGN+((sizeof(float)*rstr->nacts+BUFALIGN-1)/BUFALIGN)*BUFALIGN)*rstr->nclients))==NULL){
    printf("unable to alloc reconAsync inbuf\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if(posix_memalign((void**)&rstr->prev,BUFALIGN,(((sizeof(float)*rstr->nacts+BUFALIGN-1)/BUFALIGN)*BUFALIGN)*rstr->nclients)!=0){
  //if((rstr->prev=calloc(sizeof(float),rstr->nacts*rstr->nclients))==NULL){
    printf("unable to malloc reconAsync prev\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  memset(rstr->prev,0,(((sizeof(float)*rstr->nacts+BUFALIGN-1)/BUFALIGN)*BUFALIGN)*rstr->nclients);
  if((rstr->sock=calloc(sizeof(int),rstr->nclients))==NULL){
    printf("unable to malloc reconAsync sock\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if((rstr->started=calloc(sizeof(int),rstr->nclients))==NULL){
    printf("unable to malloc reconAsync started\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if((rstr->bytesReceived=calloc(sizeof(int),rstr->nclients))==NULL){
    printf("unable to malloc reconAsync bytesReceived\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if((rstr->ready=calloc(sizeof(int),rstr->nclients))==NULL){
    printf("unable to malloc reconAsync ready\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if((rstr->combineReady=calloc(sizeof(int),rstr->nclients))==NULL){
    printf("unable to malloc reconAsync ready\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if((rstr->discard=calloc(sizeof(int),rstr->nclients))==NULL){
    printf("unable to malloc reconAsync discard\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if((rstr->shmbuf=calloc(sizeof(char*),rstr->nclients))==NULL){
    printf("unable to malloc reconAsync shmbuf\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if((rstr->mq=calloc(sizeof(mqd_t),rstr->nclients))==NULL){
    printf("unable to malloc reconAsync mq\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if((rstr->sockAddr=calloc(sizeof(struct sockaddr_in),rstr->nclients))==NULL){
    printf("unable to malloc reconAsync sockAddr\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if((rstr->udpRecvBuf=calloc(rstr->udpRecvLen,rstr->nclients))==NULL){
    printf("unable to alloc reconAsync udpRecvBuf\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if((rstr->curframeno=calloc(sizeof(unsigned int),rstr->nclients))==NULL){
    printf("unable to alloc reconAsync curframeno\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }

  /*if((rstr->uport=calloc(sizeof(int),rstr->nclients))==NULL){
    printf("Unable to alloc reconAsync uport\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
    }*/
    
  
  if(rstr->initState!=NULL)
    memcpy(rstr->outarr,rstr->initState,sizeof(float)*rstr->nacts);
  else
    memset(rstr->outarr,0,sizeof(float)*rstr->nacts);

  //Open the shared memory and mqueues...
  if(reconOpenShmQueue(rstr)!=0){
    printf("Error opening reconAsync shm\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }

  //Now open the connections...
  if(reconOpenListeningSocket(rstr)!=0){
    printf("Error opening reconAsync listening sockets\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if(reconOpenUDP(rstr)!=0){
    printf("Error opening reconAsync UDP sockets\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  //allocate thread structures.
  if((rstr->threadid=calloc(1+rstr->nPolcThreads,sizeof(pthread_t)))==NULL){
    printf("Error allocing threadid in reconAsync\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if(rstr->nPolcThreads>0){
    if((rstr->tstr=calloc(rstr->nPolcThreads,sizeof(ThreadStruct)))==NULL){
      printf("Unable to alloc threadStruct in reconAsync\n");
      reconClose(reconHandle);
      *reconHandle=NULL;
      return 1;
    }
  }
  //and start the worker thread.
  if(pthread_create(&rstr->threadid[0],NULL,reconWorker,rstr)!=0){
    rstr->threadid[0]=0;
    printf("Unable to create thread for reconAsync\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }

  // set up worker thread affinity and priority
  reconSetThreadPrioAffin(rstr->threadid[0],rstr,rstr->threadPriority,rstr->threadAffinity);

  //and the polc threads.
  for(i=0;i<rstr->nPolcThreads;i++){
    rstr->tstr[i].threadno=i;
    rstr->tstr[i].rstr=(void*)rstr;
    if(pthread_create(&rstr->threadid[i+1],NULL,polcWorker,&rstr->tstr[i])!=0){
      rstr->threadid[i+1]=0;
      printf("Unable to create polc thread %d\n",i);
      reconClose(reconHandle);
      *reconHandle=NULL;
      return 1;
    }
    reconSetThreadPrioAffin(rstr->threadid[i+1],rstr,rstr->polcThreadPriority[i],&rstr->polcThreadAffinity[i*rstr->threadAffinElSize]);
  }
  
  printf("reconOpen done\n");
  return 0;
}


/**
   Called by single thread per frame at the end of frame.
   This is called by a subaperture processing thread.
   The bare minimum should be placed here, as most processing should be done in the reconFrameFinished function instead, which doesn't hold up processing.
*/
int reconFrameFinishedSync(void *reconHandle,int err,int forcewrite){
  ReconStruct *rstr=(ReconStruct*)reconHandle;
  float *dmCommand=rstr->arrStr->dmCommand;
  struct timespec timeout;
  timeout.tv_sec = (long)rstr->ftimeout; // drj 140422: darc_cond_timedwait now uses a relative time
  timeout.tv_nsec = (rstr->ftimeout-timeout.tv_sec)*1000000000L;
  if(darc_mutex_lock(&rstr->mutex))
    printf("darc_mutex_lock error in copyThreadPhase: %s\n",strerror(errno));
  if(rstr->dataReady==0){//wait for a signal
    //we do a timed wait here with typically a large (1-10s) timeout so that if one of the children darcs is having problems, it doesn't lock this one.
    if(darc_cond_timedwait(&rstr->cond,&rstr->mutex,&timeout)!=0){
//     if(darc_cond_wait(&rstr->cond,&rstr->mutex)!=0){
      printf("Error in reconFrameFinishedSync - probably timeout while waiting for child data\n");
    }
  }
  if(rstr->dataReady){
    rstr->dataReady=0;
    //Now copy the data.
    //printf("reconFrameFinishedSync copying data\n");
    darc_mutex_lock(&rstr->outarrMutex);//this protects both dmCommand and outarr.
    memcpy(dmCommand,rstr->outarr,sizeof(float)*rstr->nacts);
    if(rstr->polcSource==1){
      //wake up the polc threads.
      darc_cond_broadcast(&rstr->polcCond);
    }
    darc_mutex_unlock(&rstr->outarrMutex);
    //could do some bleeding here?  Add if needed.
  }
  darc_mutex_unlock(&rstr->mutex);
  return 0;
}
/**
   Called by the single thread per frame, when the actuator values aren't being sent to the dm - so we need to reset ourselves.
   May not be called at all, if the loop is closed.
   Not called by a subap processing thread - subap processing for next frame have have started before this is called...
*/

int reconOpenLoop(void *reconHandle){//globalStruct *glob){
  ReconStruct *rstr=(ReconStruct*)reconHandle;//glob->rstr;
  darc_mutex_lock(&rstr->mutex);
  rstr->reset=1;
  darc_mutex_unlock(&rstr->mutex);
  return 0;

}
