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
#include <sys/select.h>
#include <arpa/inet.h>
#include <mqueue.h>

#include <sys/mman.h>
#ifdef USEAGBBLAS
#include "agbcblas.h"
#else
#include <gsl/gsl_cblas.h>
typedef enum CBLAS_ORDER CBLAS_ORDER;
typedef enum CBLAS_TRANSPOSE CBLAS_TRANSPOSE;
#endif
#include "darc.h"
#include "rtcrecon.h"
#include "buffer.h"

typedef enum{
  ASYNCCOMBINES,//which inputs are synchronous with others.
  ASYNCINITSTATE,//initial state of the actuators.
  ASYNCNAMES,//list of the prefixes.
  ASYNCOFFSETS,//list of offsets
  ASYNCRESET,//reset it.
  ASYNCSCALES,//list of scales - multiply the inputs by scale then add offset
  ASYNCSTARTS,//flag - wait until first entry received before sending to mirror?
  ASYNCTYPES,//flag - 0==socket, 1==shm/mqueue
  ASYNCUPDATES,//flag - can this client cause an update? (probably 1).
  NACTS,
  //Add more before this line.
  RECONNBUFFERVARIABLES//equal to number of entries in the enum
}RECONBUFFERVARIABLEINDX;

#define reconMakeNames() bufferMakeNames(RECONNBUFFERVARIABLES,"asyncCombines","asyncInitState","asyncNames","asyncOffsets","asyncReset","asyncScales","asyncStarts","asyncTypes","asyncUpdates","nacts")

#define HDRSIZE 8 //matches that from mirrorSocket, which is where this probably gets data from... if alternatives are needed, then recode...
#define FHDRSIZE (8/sizeof(float))
#define RECON_SHM 1


typedef struct{
  int *ready;
  pthread_mutex_t mutex;
  pthread_cond_t cond;
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
  float *inbuf;
  char **asyncNames;
  float *prev;
  float *initState;
  pthread_t threadid;
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
}ReconStruct;

void reconCloseShmQueue(ReconStruct *rstr){
  int i;
  char name[80];
  int size=rstr->nacts*sizeof(float)+HDRSIZE+sizeof(pthread_mutex_t);
  strcpy(name,"/reconAsync");
  name[79]='\0';
  if(rstr->types==NULL || rstr->asyncNames==NULL )
    return;
  //remove everything...
  for(i=0; i<rstr->nclients; i++){
    if(rstr->types[i]==RECON_SHM){//this one is shm
      if(rstr->shmbuf[i]!=NULL){
	pthread_mutex_destroy((pthread_mutex_t*)(rstr->shmbuf[i]));
	munmap(rstr->shmbuf[i],size);
      }
      rstr->shmbuf[i]=NULL;
      if(rstr->mq[i]>0)
	mq_close(rstr->mq[i]);
      rstr->mq[i]=0;
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
  int size=rstr->nacts*sizeof(float)+HDRSIZE+sizeof(pthread_mutex_t);
  pthread_mutexattr_t mutexattr;
  struct mq_attr mqattr;
  strcpy(name,"/reconAsync");
  name[79]='\0';
  pthread_mutexattr_init(&mutexattr);
  pthread_mutexattr_setpshared(&mutexattr,PTHREAD_PROCESS_SHARED);
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
	    pthread_mutex_init((pthread_mutex_t*)(rstr->shmbuf[i]),&mutexattr);
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
	}
      }
    }
  }
  if(err){
    reconCloseShmQueue(rstr);
  }
  pthread_mutexattr_destroy(&mutexattr);
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
    if(rstr->types[i]!=RECON_SHM){
      needed=1;
      break;
    }
  }
  if(needed){//open the listening socket
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
    }
  }else{
    rstr->lsock=0;
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
	      rstr->bytesReceived[i]=0;
	      rstr->ready[i]=0;
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

void *reconWorker(void *reconHandle){
  ReconStruct *rstr=(ReconStruct*)reconHandle;
  float offset,scale;
  float *offsets,*scales,*prev,*inbuf;
  fd_set fdset;
  int max;
  int j,i;
  cpu_set_t mask;
  int n;
  struct sched_param param;
  struct timeval timeout;
  int doupdate;
  unsigned int *hdr;
  int ready,update;
  char msg[2]="\0\0";
  timeout.tv_sec=1;
  timeout.tv_usec=0;
  n= sysconf(_SC_NPROCESSORS_ONLN);
  CPU_ZERO(&mask);
  for(i=0; i<n && i<rstr->threadAffinElSize*32; i++){
    if(((rstr->threadAffinity[i/32])>>(i%32))&1){
      CPU_SET(i,&mask);
    }
  }
  if(sched_setaffinity(0,sizeof(cpu_set_t),&mask))
    printf("Error in sched_setaffinity: %s\n",strerror(errno));
  param.sched_priority=rstr->threadPriority;
  if(sched_setparam(0,&param)){
    printf("Error in sched_setparam: %s - probably need to run as root if this is important\n",strerror(errno));
  }
  if(sched_setscheduler(0,SCHED_RR,&param))
    printf("sched_setscheduler: %s - probably need to run as root if this is important\n",strerror(errno));
  if(pthread_setschedparam(pthread_self(),SCHED_RR,&param))
    printf("error in pthread_setschedparam - maybe run as root?\n");
  pthread_mutex_lock(&rstr->mutex);
  printf("reconWorker starting loop\n");
  while(rstr->go){
    pthread_mutex_unlock(&rstr->mutex);
    FD_ZERO(&fdset);
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
      }else{//socket type.
	if(rstr->sock[i]!=0){
	  FD_SET(rstr->sock[i],&fdset);
	  if(rstr->sock[i]>max)
	    max=rstr->sock[i];
	}
      }
    }
    
    timeout.tv_sec=1;
    timeout.tv_usec=0;
    n=select(max+1,&fdset,NULL,NULL,&timeout);
    //printf("select done %d %d\n",n,max);
    pthread_mutex_lock(&rstr->mutex);
    if(rstr->reset){
      if(rstr->initState!=NULL)
	memcpy(rstr->outarr,rstr->initState,sizeof(float)*rstr->nacts);
      else
	memset(rstr->outarr,0,sizeof(float)*rstr->nacts);
      memset(rstr->prev,0,sizeof(float)*rstr->nacts);
      memset(rstr->started,0,sizeof(int)*rstr->nclients);
      rstr->dataReady=1;
      doupdate=1;
      //rstr->reset=0;
    }
    if(n==0){
      //timeout - wake the processing thread so it can continue, so the rtc doesn't become frozen.   It will just send the existing actuators to the mirror.
      printf("Timeout in reconAsync (no data arrived)\n");
      pthread_cond_signal(&rstr->cond);//wake the processing thread.
    }else if(n<0){
      //select error
      printf("Error in select in reconAsync\n");
      pthread_cond_signal(&rstr->cond);//wake the processing thread
      pthread_mutex_unlock(&rstr->mutex);
      sleep(1);
      pthread_mutex_lock(&rstr->mutex);
    }else{//data ready to read.
      for(i=0; i<rstr->nclients; i++){
	if(rstr->reset && rstr->bytesReceived[i]>0){
	  rstr->discard[i]=1;//in reset, so need to discard this frame.
	}
	if(rstr->types[i]==RECON_SHM){//shm type
	  if(FD_ISSET(rstr->mq[i],&fdset)){//all bytes have been copied into shm
	    if(mq_receive(rstr->mq[i],msg,2,NULL)<0){
	      printf("Error in reconAsync mq_receive\n");
	    }else
	      rstr->bytesReceived[i]=rstr->nacts*sizeof(float)+HDRSIZE;
	  }
	}else{//socket type - read the socket
	  if(FD_ISSET(rstr->sock[i],&fdset) && rstr->bytesReceived[i]<rstr->nacts*sizeof(float)+HDRSIZE){
	    //data is ready, and can be read.
	    if((n=recv(rstr->sock[i],&rstr->inbuf[i*(rstr->nacts+FHDRSIZE)+rstr->bytesReceived[i]],rstr->nacts*sizeof(float)+HDRSIZE-rstr->bytesReceived[i],0))<=0){
	      //no data received - error - so close socket.
	      printf("Closing socket %d (ret=%d)\n",i,n);
	      if(n<0)
		printf("%s\n",strerror(errno));
	      close(rstr->sock[i]);
	      rstr->sock[i]=0;
	      rstr->nconnected--;
	    }else{
	      rstr->bytesReceived[i]+=n;
	    }
	  }
	}
      }
      doupdate=0;
      for(i=0; i<rstr->nclients; i++){
	if(rstr->bytesReceived[i]==rstr->nacts*sizeof(float)+HDRSIZE){
	  if(rstr->discard[i]){
	    rstr->discard[i]=0;
	    rstr->bytesReceived[i]=0;
	  }else{
	    //a full set of actuator demands has been received.
	    if(rstr->combines[i]>=0){
	      //this one is synchronous with others... only if they are all ready can we continue.
	      ready=1;
	      update=0;
	      for(j=0; j<rstr->nclients; j++){
		if((j==rstr->combines[i] || rstr->combines[j]==rstr->combines[i]) && rstr->bytesReceived[j]==rstr->nacts*sizeof(float)+HDRSIZE){
		  //if at least one of them can cause an update, set update.
		  update+=rstr->causeUpdate[i];
		  
		}else{
		  ready=0;
		}
	      }
	      if(ready){
		if(update>0)//at least one has causeUpdate set - so can update.
		  doupdate=1;
		for(j=0; j<rstr->nclients; j++){
		  if((j==rstr->combines[i] || rstr->combines[j]==rstr->combines[i]) && rstr->bytesReceived[j]==rstr->nacts*sizeof(float)+HDRSIZE){
		    rstr->ready[j]=1;
		  }
		}
	      }
	    }else{//not synchronous with any - so ready...
	      rstr->ready[i]=1;
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
	  if(rstr->types[i]==RECON_SHM){
	    pthread_mutex_lock((pthread_mutex_t*)(rstr->shmbuf[i]));
	    inbuf=(float*)(rstr->shmbuf[i]+sizeof(pthread_mutex_t)+HDRSIZE);
	    hdr=(unsigned int*)(rstr->shmbuf[i]+sizeof(pthread_mutex_t));
	  }else{
	    inbuf=&rstr->inbuf[i*(rstr->nacts+FHDRSIZE)+FHDRSIZE];
	    hdr=(unsigned int*)(&rstr->inbuf[i*(rstr->nacts+FHDRSIZE)]);
	  }
	  //first, subtract the previous value for this client, then add the current value, and store the current value.
	  //y-=x
	  //if(((unsigned int*)rstr->inbuf)[i*(rstr->nacts+FHDRSIZE)]!=(0x5555<<17|rstr->nacts)){
	  if(hdr[0]!=(0x5555<<17|rstr->nacts)){
	    printf("ERROR - data header not what expected: %u %u\n",hdr[0],(0x5555<<17|rstr->nacts));
	  }else if(rstr->reset==0){
	    agb_cblas_saxpym111(rstr->nacts,&rstr->prev[i*rstr->nacts],rstr->outarr);
	    if(rstr->offsets==NULL){
	      if(rstr->offsetsArr==NULL){
		if(rstr->scales==NULL){
		  if(rstr->scalesArr==NULL){
		    memcpy(&rstr->prev[i*rstr->nacts],inbuf,sizeof(float)*rstr->nacts);
		  }else{//scale for each actuator
		    prev=&rstr->prev[i*rstr->nacts];
		    //inbuf=&rstr->inbuf[i*(rstr->nacts+FHDRSIZE)+FHDRSIZE];
		    scales=&rstr->scalesArr[i*rstr->nacts];
		    for(j=0; j<rstr->nacts; j++)
		      prev[j]=inbuf[j]*scales[j];
		  }
		}else{//single scale
		  prev=&rstr->prev[i*rstr->nacts];
		  //inbuf=&rstr->inbuf[i*(rstr->nacts+FHDRSIZE)+FHDRSIZE];
		  scale=rstr->scales[i];
		  for(j=0; j<rstr->nacts; j++)
		    prev[j]=inbuf[j]*scale;
		}
	      }else{//offset for each actuator
		prev=&rstr->prev[i*rstr->nacts];
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
	      prev=&rstr->prev[i*rstr->nacts];
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
	    //now add current. y+=x
	    agb_cblas_saxpy111(rstr->nacts,&rstr->prev[i*rstr->nacts],rstr->outarr);
	    rstr->started[i]=1;
	  }
	  rstr->ready[i]=0;
	  rstr->bytesReceived[i]=0;
	  rstr->reconframeno[i]=hdr[1];//((unsigned int*)rstr->inbuf)[i*(rstr->nacts+FHDRSIZE)+1];
	  if(rstr->types[i]==RECON_SHM){
	    pthread_mutex_unlock((pthread_mutex_t*)(rstr->shmbuf[i]));
	  }
	}
	if(rstr->startFlags[i]==1 && rstr->started[i]==0){
	  //Data is needed for this one, but it hasn't yet produced any - so the outarr is currently invalid - so we shouldn't send the data.
	  ready=-rstr->nclients;//set it low enough to be negative.
	}
      }
      if((ready>0 && doupdate==1) || rstr->dataReady==1){//something is ready... so wake the processing thread.  This happens if we're in reset (in which case rstr->dataReady will be 1), or if we're actually ready.  But we also need to consider the case that one child darc isn't sending data (maybe has never sent data) and e.g. startFlags is one for this child or it is the only child with causeUpdate set.  In this case, if we took no action, this darc would appear to freeze.  So, we need to set a maximum timeout after which it will wake... actually wil will handle this in the pthread_cond_wait by using a timedwait instead... probably better.
	rstr->dataReady=1;
	pthread_cond_signal(&rstr->cond);
      }
      pthread_mutex_unlock(&rstr->mutex);
      if(rstr->lsock>0 && FD_ISSET(rstr->lsock,&fdset) && rstr->nconnected<rstr->nclients){
	reconAcceptConnection(rstr);
      }
      pthread_mutex_lock(&rstr->mutex);

    }
  }
  pthread_cond_signal(&rstr->cond);
  pthread_mutex_unlock(&rstr->mutex);
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
    if(rstr->threadid!=0){
      pthread_join(rstr->threadid,NULL);
    }
    if(rstr->paramNames!=NULL)
      free(rstr->paramNames);
    pthread_mutex_destroy(&rstr->mutex);
    pthread_cond_destroy(&rstr->cond);
    for(i=0; i<rstr->nclients; i++){
      close(rstr->sock[i]);
      close(rstr->lsock);
    }
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
    SAFEFREE(rstr->discard);
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
  pthread_mutex_lock(&rstr->mutex);
  if(err==0){
    if(dtype[NACTS]=='i' && nbytes[NACTS]==4){
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
    if(nbytes[ASYNCINITSTATE]==sizeof(float)*rstr->nacts && dtype[ASYNCINITSTATE]=='f'){
      rstr->initState=(float*)values[ASYNCINITSTATE];
    }else if(nbytes[ASYNCINITSTATE]==0){
      rstr->initState=NULL;
    }else{
      rstr->initState=NULL;
      printf("asyncInitState error\n");
      writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"asyncInitState error\n");
      err=1;
    }
    if(nbytes[ASYNCRESET]==sizeof(int) && dtype[ASYNCRESET]=='i'){
      rstr->reset=*((int*)values[ASYNCRESET]);
      //if(*((int*)values[ASYNCRESET])==1){
      //	rstr->reset=1;
      //*((int*)values[ASYNCRESET])=0;
      //}
    }else{
      printf("asyncReset error\n");
      writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"asyncReset error\n");
      err=1;
    }
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
    if(dtype[ASYNCSTARTS]=='i' && nbytes[ASYNCSTARTS]==sizeof(int)*rstr->nclients){
      rstr->startFlags=(int*)values[ASYNCSTARTS];
    }else{
      printf("asyncStarts error\n");
      err=1;
      writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"asyncStarts error\n");
    }
    if(dtype[ASYNCCOMBINES]=='i' && nbytes[ASYNCCOMBINES]==sizeof(int)*rstr->nclients){
      rstr->combines=(int*)values[ASYNCCOMBINES];
    }else{
      printf("asyncCombines error\n");
      err=1;
      writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"asyncCombines error\n");
    }
    if(dtype[ASYNCTYPES]=='i' && nbytes[ASYNCTYPES]==sizeof(int)*rstr->nclients){
      rstr->types=(int*)values[ASYNCTYPES];
    }else{
      printf("asyncTypes error\n");
      err=1;
      writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"asyncTypes error\n");
    }
    if(dtype[ASYNCUPDATES]=='i' && nbytes[ASYNCUPDATES]==sizeof(int)*rstr->nclients){
      rstr->causeUpdate=(int*)values[ASYNCUPDATES];
    }else{
      printf("asyncUpdates error\n");
      err=1;
      writeErrorVA(rstr->rtcErrorBuf,-1,frameno,"asyncUpdates error\n");
    }

  }
  pthread_mutex_unlock(&rstr->mutex);
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
  int err=0;
  if(n<=6 || n!=6+args[2]){
    printf("Error - args for reconAsync should be: number of async clients, port, Naffin, thread priority, timeout (in ms), overwrite flag (use 0 unless you know what it does),thread affinity[Naffin].\n");
    *reconHandle=NULL;
    return 1;
  }
  if(sizeof(pthread_mutex_t)%4!=0){
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
  if(*reconframenoSize<rstr->nclients){
    if(*reconframeno!=NULL)
      free(*reconframeno);
    if((*reconframeno=malloc(sizeof(unsigned int)*rstr->nclients))==NULL){
      printf("Error allocating frame numbers for reconAsync\n");
      reconClose(reconHandle);
      *reconHandle=NULL;
      return 1;
    }else{
      *reconframenoSize=rstr->nclients;
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
  if(pthread_mutex_init(&rstr->mutex,NULL)){
    printf("Error init recon mutex\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if(pthread_cond_init(&rstr->cond,NULL)){
    printf("Error init recon cond\n");
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
  if((rstr->inbuf=malloc((HDRSIZE+sizeof(float)*rstr->nacts)*rstr->nclients))==NULL){
    printf("unable to malloc reconAsync inbuf\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
  if((rstr->prev=calloc(sizeof(float),rstr->nacts*rstr->nclients))==NULL){
    printf("unable to malloc reconAsync prev\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
  }
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
  //and start the worker thread.
  if(pthread_create(&rstr->threadid,NULL,reconWorker,rstr)!=0){
    rstr->threadid=0;
    printf("Unable to create thread for reconAsync\n");
    reconClose(reconHandle);
    *reconHandle=NULL;
    return 1;
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
  clock_gettime(CLOCK_REALTIME, &timeout);
  timeout.tv_sec+=(int)rstr->ftimeout;
  timeout.tv_nsec+=(int)((rstr->ftimeout-(int)rstr->ftimeout)*1e9);
  if(timeout.tv_nsec>1000000000){
    timeout.tv_sec++;
    timeout.tv_nsec-=1000000000;
  }
  if(pthread_mutex_lock(&rstr->mutex))
    printf("pthread_mutex_lock error in copyThreadPhase: %s\n",strerror(errno));
  if(rstr->dataReady==0){//wait for a signal
    //we do a timed wait here with typically a large (1-10s) timeout so that if one of the children darcs is having problems, it doesn't lock this one.
    if(pthread_cond_timedwait(&rstr->cond,&rstr->mutex,&timeout)!=0){
      printf("Error in reconFrameFinishedSync - probably timeout while waiting for child data\n");
    }
  }
  if(rstr->dataReady){
    rstr->dataReady=0;
    //Now copy the data.
    //printf("reconFrameFinishedSync copying data\n");
    memcpy(dmCommand,rstr->outarr,sizeof(float)*rstr->nacts);
    //could do some bleeding here?  Add if needed.
  }
  pthread_mutex_unlock(&rstr->mutex);
  return 0;
}
/**
   Called by the single thread per frame, when the actuator values aren't being sent to the dm - so we need to reset ourselves.
   May not be called at all, if the loop is closed.
   Not called by a subap processing thread - subap processing for next frame have have started before this is called...
*/

int reconOpenLoop(void *reconHandle){//globalStruct *glob){
  ReconStruct *rstr=(ReconStruct*)reconHandle;//glob->rstr;
  pthread_mutex_lock(&rstr->mutex);
  rstr->reset=1;
  pthread_mutex_unlock(&rstr->mutex);
  return 0;

}
 
/**
   Called by single thread at the start of each frame.
   This thread is not a subap processing thread.
   If doing a param buffer swap, this is called after the swap has completed.
*/
/*
int reconNewFrame(void *reconHandle,unsigned int frameno,double timestamp){
  ReconStruct *rstr=(ReconStruct*)reconHandle;
  ReconStructEntry *rs;
  int i;
  //if(rstr->swap){
  // rstr->swap=0;
  // rstr->buf=1-rstr->buf;
  //}
  rs=&rstr->rs[rstr->buf];
  //Now wake up the thread that does the initial processing...
  //No need to get a mutex here, because the preprocessing thread must be sleeping.
  //We do this processing in a separate thread because it has a while to complete, so we may as well let subap processing threads get on with their tasks...
  //pthread_cond_signal(&rstr->dmCond);
  if(rs->reconMode==RECONMODE_SIMPLE){//simple open loop
    //memset(p->dmCommand,0,sizeof(float)*p->nacts);
    memcpy(dmCommand,rs->v0,sizeof(float)*rs->nacts);
  }else if(rs->reconMode==RECONMODE_TRUTH){//closed loop
    if(rs->decayFactor==NULL){
      memcpy(dmCommand,rstr->latestDmCommand,sizeof(float)*rs->nacts);
    }else{
      for(i=0; i<rs->nacts; i++){
	dmCommand[i]=rs->decayFactor[i]*rstr->latestDmCommand[i];
      }
    }
  }else if(rs->reconMode==RECONMODE_OPEN){//reconmode_open
    //initialise by copying v0 into the result.
    //This line removed 100528 after discussion with Eric Gendron.
    //memcpy(glob->arrays->dmCommand,rs->v0,sizeof(float)*rs->nacts);
    //beta=1.;
    //memset(glob->arrays->dmCommand,0,sizeof(float)*rs->nacts);
    //Now: dmcommand=v0+dot(gainE,latestDmCommand)
#ifdef USEAGBBLAS
    agb_cblas_sgemvRowNN1N101(rs->nacts,rs->gainE,rstr->latestDmCommand,dmCommand);
#else
    //beta=0.;
    cblas_sgemv(order,trans,rs->nacts,rs->nacts,alpha,rs->gainE,rs->nacts,rstr->latestDmCommand,inc,beta,dmCommand,inc);
#endif
  }else{//reconmode_offset
    memcpy(dmCommand,rs->v0,sizeof(float)*rs->nacts);
  }	


  //set the DM arrays ready.
  if(pthread_mutex_lock(&rstr->dmMutex))
    printf("pthread_mutex_lock error in setDMArraysReady: %s\n",strerror(errno));
  rstr->dmReady=1;
  //wake up any of the subap processing threads that are waiting.
  pthread_cond_broadcast(&rstr->dmCond);
  pthread_mutex_unlock(&rstr->dmMutex);
  return 0;
}
*/
/**
   Called once per thread at the start of each frame, possibly simultaneously.
*/
/*
int reconStartFrame(void *reconHandle,int cam,int threadno){
  ReconStruct *rstr=(ReconStruct*)reconHandle;//threadInfo->globals->rstr;
  ReconStructEntry *rs=&rstr->rs[rstr->buf];
  memset((void*)(&rs->dmCommandArr[rs->nacts*threadno]),0,rs->nacts*sizeof(float));
  return 0;
}
*/
/**
   Called multiple times by multiple threads, whenever new slope data is ready
   centroids may not be complete, and writing to dmCommand is not thread-safe without locking.
*/
/*
int reconNewSlopes(void *reconHandle,int cam,int centindx,int threadno,int nsubapsDoing){
#if !defined(USEAGBBLAS)
  CBLAS_ORDER order=CblasColMajor;
  CBLAS_TRANSPOSE trans=CblasNoTrans;
  float alpha=1.,beta=1.;
  int inc=1;
#endif
  int step;//number of rows to do in mmx...
  ReconStruct *rstr=(ReconStruct*)reconHandle;
  ReconStructEntry *rs=&rstr->rs[rstr->buf];
  float *centroids=rstr->arrStr->centroids;
  //float *dmCommand=rstr->arr->dmCommand;
  //infoStruct *info=threadInfo->info;
  //globalStruct *glob=threadInfo->globals;
  //We assume that each row i of the reconstructor has already been multiplied by gain[i].  
  //So, here we just do dmCommand+=rmx[:,n]*centx+rmx[:,n+1]*centy.
  dprintf("in partialReconstruct %d %d %d %p %p %p\n",rs->nacts,centindx,rs->totCents,centroids,rs->rmxT,&rs->dmCommandArr[rs->nacts*threadno]);
  step=2*nsubapsDoing;
#ifdef USEAGBBLAS
  agb_cblas_sgemvColMN1M111(rs->nacts,step,&(rs->rmxT[centindx*rs->nacts]),&(centroids[centindx]),&rs->dmCommandArr[rs->nacts*threadno]);
#else
  cblas_sgemv(order,trans,rs->nacts,step,alpha,&(rs->rmxT[centindx*rs->nacts]),rs->nacts,&(centroids[centindx]),inc,beta,&rs->dmCommandArr[rs->nacts*threadno],inc);
#endif
  return 0;
}
*/
/**
   Called once for each thread at the end of a frame
   Here we sum the individual dmCommands together to get the final one.
   centroids may not be complete, and writing to dmCommand is not thread-safe without locking.
*/
/*
int reconEndFrame(void *reconHandle,int cam,int threadno,int err){
  //dmCommand=glob->arrays->dmCommand;
  //globalStruct *glob=threadInfo->globals;
  ReconStruct *rstr=(ReconStruct*)reconHandle;
  ReconStructEntry *rs=&rstr->rs[rstr->buf];
  //float *centroids=rstr->arr->centroids;
  float *dmCommand=rstr->arrStr->dmCommand;
  if(pthread_mutex_lock(&rstr->dmMutex))
    printf("pthread_mutex_lock error in copyThreadPhase: %s\n",strerror(errno));
  if(rstr->dmReady==0)//wait for the precompute thread to finish (it will call setDMArraysReady when done)...
    if(pthread_cond_wait(&rstr->dmCond,&rstr->dmMutex))
      printf("pthread_cond_wait error in copyThreadPhase: %s\n",strerror(errno));
  //now add threadInfo->dmCommand to threadInfo->info->dmCommand.
#ifdef USEAGBBLAS
  agb_cblas_saxpy111(rs->nacts,&rs->dmCommandArr[rs->nacts*threadno],dmCommand);
#else
  cblas_saxpy(rs->nacts,1.,&rs->dmCommandArr[rs->nacts*threadno],1,dmCommand,1);
#endif
  pthread_mutex_unlock(&rstr->dmMutex);
  return 0;
}
*/
/**
   Called by single thread per frame - end of frame
   Do any post processing here.
   This it typically called by a non-subaperture processing thread.
   At the end of this method, dmCommand must be ready...
   Note, while this is running, subaperture processing of the next frame may start.
*/
/*
int reconFrameFinished(void *reconHandle,int err){//globalStruct *glob){
  //Note: dmCommand=glob->arrays->dmCommand.
  ReconStruct *rstr=(ReconStruct*)reconHandle;//glob->rstr;
  ReconStructEntry *rs=&rstr->rs[rstr->postbuf];
  float bleedVal=0.;
  int i;
  float *dmCommand=rstr->arrStr->dmCommand;
  if(rs->bleedGainOverNact!=0.){//compute the bleed value
    for(i=0; i<rs->nacts; i++){
      //bleedVal+=glob->arrays->dmCommand[i];
      bleedVal+=dmCommand[i]-rs->v0[i];
    }
    bleedVal*=rs->bleedGainOverNact;
    //bleedVal-=rs->midRangeTimesBleed;//Note - really midrange times bleed over nact... maybe this should be replaced by v0 - to allow a midrange value per actuator?
    for(i=0; i<rs->nacts; i++)
      dmCommand[i]-=bleedVal;
  }
  //bleedVal-=0.5;//do proper rounding...
  if(err==0)
    memcpy(rstr->latestDmCommand,dmCommand,sizeof(float)*rs->nacts);
  return 0;
  }*/

