#define _GNU_SOURCE
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <stdarg.h>
#include <sched.h>
#include <arpa/inet.h>
#include <time.h>
#include <pthread.h>
#include "circ.h"

typedef struct{
  char *shmprefix;
  char *host;
  int port;
  int debug;
  int log;
  int raw;
  int connect;
  //int tries;
  int startWithLatest;
  int decimate;
  char *streamname; 
  char *fullname;// "/" + shmprefix + streamname.
  int shmOpen;
  circBuf *cb;
  int cumfreq;
  int go;
  char *saver;
  char hdr[HSIZE];
  char prevhdr[HSIZE];
  int hasclient;
  int lsocket;
  int client;
  char nd;
  int dims[6];
  char dtype;
  int nstore;
  pthread_t thread;
  time_t timeDataLastRequested;
}RecvStruct;

void *poller(void *rrstr){
  RecvStruct *rstr=(RecvStruct*)rrstr;
  int lastDec=0;
  while(rstr->go){
    if(FREQ(rstr->cb)!=lastDec){
      lastDec=FREQ(rstr->cb);
      if(send(rstr->client,&lastDec,sizeof(int),0)!=sizeof(int)){
	printf("Error sending decimate value %d\n",lastDec);
	close(rstr->client);
	rstr->hasclient=0;
      }
    }
    usleep(100000);

  }
  return NULL;
}

int openSHM(RecvStruct *rstr){
  rstr->shmOpen=0;
  while(rstr->shmOpen==0){
    if((rstr->cb=openCircBuf(rstr->fullname,rstr->nd,rstr->dims,rstr->dtype,rstr->nstore))!=NULL){
      rstr->shmOpen=1;
    }else{
      printf("Failed to open /dev/shm%s\n",rstr->fullname);
      rstr->cb=NULL;
      sleep(1);
    }
  }
  printf("/dev/shm%s opened\n",rstr->fullname);
  //pthread_create(&rstr->thread,NULL,poller,(void*)rstr);
  return 0;
}


int setThreadAffinity(int affinity,int priority){
  int i;
  cpu_set_t mask;
  struct sched_param param;
  int ncpu=sysconf(_SC_NPROCESSORS_ONLN);
  if(ncpu>32){
    printf("sender: Unable to set affinity to >32 CPUs at present\n");
    ncpu=32;
  }
  CPU_ZERO(&mask);
  for(i=0; i<ncpu; i++){
    if(((affinity)>>i)&1){
      CPU_SET(i,&mask);
    }
  }
  //printf("Thread affinity %d\n",threadAffinity&0xff);
  if(sched_setaffinity(0,sizeof(cpu_set_t),&mask))
    printf("sender: Error in sched_setaffinity: %s\n",strerror(errno));
  param.sched_priority=priority;
  //if(sched_setparam(0,&param)){
  if(sched_setscheduler(0,SCHED_RR,&param)){
    printf("sender: Error in sched_setparam: %s\n",strerror(errno));
  }
  return 0;
}


int openSocket(RecvStruct *rstr){
  //opens a listening socket
  //Create the socket and set up to accept connections.
  struct sockaddr_in name;
  int err=0;
  if((rstr->lsocket=socket(PF_INET,SOCK_STREAM,0))<0){
    printf("Error opening listening socket\n");
    err=1;
  }else{
    name.sin_family=AF_INET;
    name.sin_port=htons(rstr->port);
    name.sin_addr.s_addr=htonl(INADDR_ANY);
    if(bind(rstr->lsocket,(struct sockaddr*)&name,sizeof(name))<0){
      printf("Unable to bind\n");
      err=1;
      close(rstr->lsocket);
      rstr->lsocket=0;
    }
  }
  if(err==0){
    if(listen(rstr->lsocket,1)<0){
      printf("Failed to listen on port\n");
      err=1;
    }
  }
  //if(err==0)
  //rstr->socketOpen=1;
  return err;
}

int acceptSocket(RecvStruct *rstr){
  struct sockaddr_in clientname;
  size_t size;
  int err=0;
  rstr->hasclient=0;
  if((rstr->client=accept(rstr->lsocket,(struct sockaddr*)&clientname,&size))<0){
    printf("Failed to accept on socket\n");
    err=1;
  }else{
    printf("Connected from %s port %hd\n",inet_ntoa(clientname.sin_addr),ntohs(clientname.sin_port));
    rstr->hasclient=1;
  }
  return err;
}
int readData(RecvStruct *rstr){
  //First read the data size (4 bytes), then read the data.
  int err=0;
  int n=0;
  int dsize;
  int indx=-1;
  char *data;
  int rec;
  printf("readData\n");
  while(err==0 && n<HSIZE){
    rec=recv(rstr->client,&rstr->hdr[n],HSIZE-n,0);
    if(rec>0)
      n+=rec;
    else{
      printf("Error reading data size\n");
      err=-1;
      close(rstr->client);
      rstr->hasclient=0;
    }
  }
  printf("Got hdr, size %d\n",((int*)rstr->hdr)[0]);
  if(err!=0)
    return err;
  //Now compare new header with previous header... to reshape if necessary
  if(((int*)rstr->hdr)[0]==28 && rstr->hdr[4]==0x55 && rstr->hdr[5]==0x55){
    //This means we're being sent size/shape information.
    rstr->nd=rstr->hdr[6];
    rstr->dtype=rstr->hdr[7];
    memcpy((char*)rstr->dims,&(rstr->hdr[8]),sizeof(int)*6);

    //Now update the shm
    if(rstr->cb==NULL){
      printf("Received hdr packet - opening shm\n");
      openSHM(rstr);
    }else{
      circReshape(rstr->cb,rstr->nd,rstr->dims,rstr->dtype);
    }
  }else{//receiving a data packet.
    if(((int*)rstr->hdr)[0]==((int*)rstr->prevhdr)[0]){//data size same, so no need to reset counter...
      if(rstr->cb==NULL){
	printf("Recieved first data packet - opening shm\n");
	openSHM(rstr);
      }
      indx=(LASTWRITTEN(rstr->cb)+1)%NSTORE(rstr->cb);
    }else{
      //need to update the shm.
      if(rstr->cb==NULL){
	printf("Received data pack - opening shm\n");
	openSHM(rstr);
      }else{
	printf("CircReshape in receiver %d %c (%d %d)\n",rstr->nd,rstr->dtype,rstr->dims[0],rstr->dims[1]);
	circReshape(rstr->cb,rstr->nd,rstr->dims,rstr->dtype);
      }
      LASTWRITTEN(rstr->cb)=-1;
      indx=0;
    }
    //and now store this header.
    memcpy(rstr->prevhdr,rstr->hdr,HSIZE);
    //and copy it into the shm
    memcpy(&(((char*)rstr->cb->data)[indx*rstr->cb->frameSize]),rstr->hdr,HSIZE);
    //now get a pointer to the data...
    data=&(((char*)rstr->cb->data)[indx*rstr->cb->frameSize+HSIZE]);
    //Now get the data size...
    dsize=((int*)rstr->hdr)[0]-HSIZE+4;
    //and write the data into the shm array
    printf("Reading data of size %d\n",dsize);
    while(err==0 && n<dsize){
      rec=recv(rstr->client,&data[n],dsize-n,0);
      if(rec>0)
	n+=rec;
      else if(n<=0){
	printf("Error in readSocket in receiver\n");
	err=-1;
	close(rstr->client);
	rstr->hasclient=0;
      }
    }
    printf("Setting lastwritten to %d\n",indx);
    LASTWRITTEN(rstr->cb)=indx;



    if(err==0){
      //Now wake any clients...
      if(CIRCSIGNAL(rstr->cb)!=0){//someone has set this - so they are interested in the data.
	rstr->timeDataLastRequested=time(NULL);
	CIRCSIGNAL(rstr->cb)=0;
      }else{
	if(time(NULL)-rstr->timeDataLastRequested>60){//nothing requested data for 60 seconds - so data no longer needed - so turn off.
	  FREQ(rstr->cb)=0;
	  //and send a zero decimate to the sender.
	  //Actually, this should be done in another thread that is polling the decimate rate... (because it can also be set by clients).
	  /*if(send(rstr->client,&zero,sizeof(int),0)!=sizeof(int)){
	    printf("Error sending zero decimate to sender\n");
	    close(rstr->client);
	    rstr->hasclient=0;
	    }*/
	}
      }
      pthread_cond_broadcast(rstr->cb->cond);
    }
  }
  return err;

}

int loop(RecvStruct *rstr){
  int err=0;
  while(rstr->go){
    acceptSocket(rstr);
    while(rstr->hasclient){
      rstr->timeDataLastRequested=time(NULL);
      while(err==0 && rstr->hasclient){
	err=readData(rstr);
	printf("readData finished, err=%d\n",err);
	//if(err==0 && rstr->shmOpen==0){
	//  err=openSHM(rstr)
	//}
	//write the data to shm.
	//if(err==0)
	//  err=updateShm(rstr);//check that is correct size etc...
	//if(err==0)
	//  err=writeShm(rstr);//write the data and inform clients.

      }
      

    }


  }
  return 0;
}



int main(int argc, char **argv){
  int setprio=0;
  int affin=0x7fffffff;
  int prio=0;
  RecvStruct *rstr;
  int i;
  int port;
  int err=0;
  if((rstr=malloc(sizeof(RecvStruct)))==NULL){
    printf("Unable to malloc SendStruct\n");
    return 1;
  }
  memset(rstr,0,sizeof(RecvStruct));
  rstr->port=4243;
  //rstr->host=NULL;
  rstr->connect=1;
  //  rstr->decimate=1;
  rstr->go=1;
  rstr->nstore=100;
  for(i=1; i<argc; i++){
    if(argv[i][0]=='-'){
      switch(argv[i][1]){
      case 'p':
	rstr->port=atoi(&argv[i][2]);
	break;
	//      case 'h':
	//rstr->host=&argv[i][2];
	//break;
      case 'a':
	affin=atoi(&argv[i][2]);
	setprio=1;
	break;
      case 'i':
	prio=atoi(&argv[i][2]);
	setprio=1;
	break;
      case 's':
	rstr->shmprefix=&argv[i][2];
	break;
      case 'v':
	rstr->debug=1;
	break;
	//case 'd':
	//rstr->decimate=atoi(&argv[i][2]);
	//break;
      case 'l':
	rstr->log=1;
	break;
      case 'r':
	rstr->raw=1;
	break;
      case 'c':
	rstr->connect=0;
	break;
      case 'n':
	rstr->nstore=atoi(&argv[i][2]);//size of the circular buffer to set up.
	break;
      default:
	break;
      }
    }else{//the stream name
      rstr->streamname=argv[i];
    }
  }
  if(rstr->streamname==NULL){
    printf("Must specify a stream\n");
    return 1;
  }
  if(rstr->shmprefix==NULL){
    if(asprintf(&rstr->fullname,"/%s",rstr->streamname)==-1){
      printf("Error asprintf\n");
      return 1;
    }
  }else{
    if(asprintf(&rstr->fullname,"/%s%s",rstr->shmprefix,rstr->streamname)==-1){
      printf("Error asprintf\n");
      return 1;
    }
  }
  if(setprio){
    setThreadAffinity(affin,prio);
  }
  //Open the listening socket and wait for a connection.
  //Upon first connection, determine the data size, and then open the circular buffer.
  port=rstr->port;
  err=1;
  while(err!=0 && rstr->port<port+100){
    if((err=openSocket(rstr))!=0){
      printf("Couldn't open listening socket port %d\n",(int)rstr->port);
      rstr->port++;
    }
  }
  if(err==0){//now listening okay...
    rstr->go=1;
    loop(rstr);
  }
  printf("sender %s exiting\n",rstr->fullname);
  return 0;


}
