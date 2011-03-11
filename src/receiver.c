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
#define _GNU_SOURCE
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
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
#include <signal.h>
#include "circ.h"

typedef struct{
  char *shmprefix;
  int port;
  int debug;
  //int tries;
  int startWithLatest;
  int decimate;
  char *streamname; 
  char *fullname;// "/" + shmprefix + streamname.
  char *outputname;//same as fullname, or optionally /rtcWhateverBuf
  int shmOpen;
  circBuf *cb;
  int cumfreq;
  int go;
  char *saver;
  int ihdr[(HSIZE+sizeof(int)-1)/sizeof(int)];
  char *hdr;
  int iprevhdr[(HSIZE+sizeof(int)-1)/sizeof(int)];
  char *prevhdr;
  int hasclient;
  int lsocket;
  int client;
  char nd;
  int dims[6];
  char dtype;
  int nstore;
  int raw;
  pthread_t thread;
  time_t timeDataLastRequested;
  pthread_mutex_t m;
}RecvStruct;

//this is a thread that watches the decimate value, and if it changes, sends the new value to the sender...
void *poller(void *rrstr){
  RecvStruct *rstr=(RecvStruct*)rrstr;
  int lastDec=0;
  char *buf;
  struct stat st;
  if(asprintf(&buf,"/dev/shm%s",rstr->outputname)==-1){
    printf("Error - couldn't malloc buf in receiver.c\n");
  }
  while(rstr!=NULL && rstr->go){
    //pthread_mutex_lock(&rstr->m);
    if(rstr->hasclient==0){
      lastDec=0;
    }else{
      if(rstr->cb!=NULL && FREQ(rstr->cb)!=lastDec){
	lastDec=FREQ(rstr->cb);
	printf("receiver sending new decimate val of %d\n",lastDec);
	if(send(rstr->client,&lastDec,sizeof(int),0)!=sizeof(int)){
	  printf("Error sending decimate value %d\n",lastDec);
	  close(rstr->client);
	  rstr->hasclient=0;
	}
      }
    }
    if(stat(buf,&st)!=0){//shm has gone?
      printf("Local SHM %s removed - receiver exiting...\n",buf);
      exit(0);
    }
    //pthread_mutex_unlock(&rstr->m);
    usleep(100000);

  }
  return NULL;
}

char *shmname=NULL;//global for the signal handler...

int openSHM(RecvStruct *rstr){
  rstr->shmOpen=0;
  while(rstr->shmOpen==0){
    if((rstr->cb=openCircBuf(rstr->outputname,rstr->nd,rstr->dims,rstr->dtype,rstr->nstore))!=NULL){
      rstr->shmOpen=1;
      shmname=rstr->outputname;
    }else{
      printf("Failed to open /dev/shm%s\n",rstr->outputname);
      rstr->cb=NULL;
      sleep(1);
    }
  }
  printf("/dev/shm%s opened\n",rstr->outputname);
  pthread_create(&rstr->thread,NULL,poller,(void*)rstr);
  return 0;
}

void handleInterrupt(int sig){
  if(shmname!=NULL){
    printf("Receiver signal %d received - removing shm %s\n",sig,shmname);
    shm_unlink(shmname);
  }else
    printf("Receiver interrupt received (%d) - exiting\n",sig);
  exit(1);
}

int setThreadAffinity(int affinity,int priority){
  int i;
  cpu_set_t mask;
  struct sched_param param;
  int ncpu=sysconf(_SC_NPROCESSORS_ONLN);
  if(ncpu>32){
    printf("receiver: Unable to set affinity to >32 CPUs at present\n");
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
    printf("receiver: Error in sched_setaffinity: %s\n",strerror(errno));
  param.sched_priority=priority;
  //if(sched_setparam(0,&param)){
  if(sched_setscheduler(0,SCHED_RR,&param)){
    printf("receiver: Error in sched_setparam: %s\n",strerror(errno));
  }
  return 0;
}

int recvSize(int sock,int size,char *buf){
  int err=0,n=0;
  int rec;
  while(err==0 && n<size){
    rec=recv(sock,&buf[n],size-n,0);
    if(rec>0)
      n+=rec;
    else{
      printf("Error reading data in recvSize\n");
      err=-1;
    }
  }
  return err;
}
/*
int deserialise(int sock){
  //first read the header (5 bytes).
  //First byte is the type<<1 | littleEndian.
  //Next 4 bytes are the data size (big endian).
  char hdr[5];
  int err=0;
  int bytes;
  if((err=recvSize(sock,5,hdr))!=0){
    printf("deserialise failed to read header\n");
  }else{
    if(hdr[0]&1==0){
      printf("Not little endian... error\n");
      err=1;
    }else{
      typ=hdr[0]>>1;
      bytes=hdr[1]<<24 | data[2]<<16 | data[3]<<8 | data[4];
      if((err=recvSize(sock,bytes,buf))!=0){
	printf("deserialise failed to read data\n");
      }else{//now deserialise the data
	

      }
    }

  }
  return err;
}
*/

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
  socklen_t size;
  int err=0;
  char buf[80];
  char namesize;
  memset(buf,0,80);
  rstr->hasclient=0;
  size=(socklen_t)sizeof(struct sockaddr_in);
  if((rstr->client=accept(rstr->lsocket,(struct sockaddr*)&clientname,&size))<0){
    printf("Failed to accept on socket: %s\n",strerror(errno));
    err=1;
  }else{
    printf("Connected from %s port %d\n",inet_ntoa(clientname.sin_addr),(int)ntohs(clientname.sin_port));
    rstr->hasclient=1;
    //When clients connect, they send in serialised form, until they specify they are converting to raw.  But, we can only handle raw, so we wait here until raw is received...
    //Actually, no, we'll maje them not send the header - they can send the stream name only...
    //while(rstr->raw==0){
    //  deserialise(rstr);
    //}
    if((err=recvSize(rstr->client,sizeof(char),&namesize))!=0){
      printf("Failed to get length of name - closing\n");
    }else{
      if(namesize>79){
	printf("name too long - closing\n");
	err=1;
      }else{
	if((err=recvSize(rstr->client,namesize,buf))!=0){
	  printf("didn't receive name - closing\n");
	}else{
	  if(strncmp(&rstr->fullname[1],buf,80)!=0){
	    printf("stream %s expected, got %s - closing\n",&rstr->fullname[1],buf);
	    err=1;
	  }
	}
      }
    }
    if(err){
      close(rstr->client);
      rstr->hasclient=0;
    }
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
  pthread_mutex_lock(&rstr->m);
  err=1-rstr->hasclient;
  //printf("readData\n");
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
  pthread_mutex_unlock(&rstr->m);
  //printf("Got hdr, size %d, %d bytes (err=%d)\n",((int*)rstr->hdr)[0],n,err);
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
    pthread_mutex_lock(&rstr->m);
    //printf("Reading data of size %d\n",dsize);
    n=0;
    err=1-rstr->hasclient;
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
	if(time(NULL)-rstr->timeDataLastRequested>10){//nothing requested data for 10 seconds - so data no longer needed - so turn off.
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
      pthread_cond_broadcast(rstr->cb->cond);//wake up anything waiting for new data.
    }
    pthread_mutex_unlock(&rstr->m);

  }
  return err;

}

int loop(RecvStruct *rstr){
  int err=0;
  while(rstr->go){
    pthread_mutex_lock(&rstr->m);

    acceptSocket(rstr);
    while(rstr->hasclient){
      rstr->timeDataLastRequested=time(NULL);
      while(err==0 && rstr->hasclient){
	pthread_mutex_unlock(&rstr->m);
	err=readData(rstr);
	//printf("readData finished, err=%d\n",err);
	pthread_mutex_lock(&rstr->m);

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
    pthread_mutex_unlock(&rstr->m);


  }
  printf("Receiver loop ending\n");
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
  int datasize=128*128*4*16;//default memory allocation for the shm buffer
  struct sigaction sigact;
  if((rstr=malloc(sizeof(RecvStruct)))==NULL){
    printf("Unable to malloc SendStruct\n");
    return 1;
  }
  memset(rstr,0,sizeof(RecvStruct));
  rstr->hdr=(char*)rstr->ihdr;
  rstr->prevhdr=(char*)rstr->iprevhdr;
  rstr->port=4243;
  //rstr->host=NULL;
  rstr->decimate=1;
  rstr->go=1;
  rstr->nstore=100;
  for(i=1; i<argc; i++){
    if(argv[i][0]=='-'){
      switch(argv[i][1]){
      case 'p':
	rstr->port=atoi(&argv[i][2]);
	break;
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
      case 'd':
	rstr->decimate=atoi(&argv[i][2]);
	break;
      case 'n':
	datasize=atoi(&argv[i][2]);
	break;
      case 'o':
	rstr->outputname=&argv[i][2];
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
  if(rstr->outputname==NULL)
    rstr->outputname=rstr->fullname;
  if(setprio){
    setThreadAffinity(affin,prio);
  }
  //Open the listening socket and wait for a connection.
  //Upon first connection, determine the data size, and then open the circular buffer.
  port=rstr->port;
  err=1;
  pthread_mutex_init(&rstr->m,NULL);
  while(err!=0 && rstr->port<port+100){
    if((err=openSocket(rstr))!=0){
      printf("Couldn't open listening socket port %d\n",(int)rstr->port);
      rstr->port++;
    }
  }
  if(err==0){    //now listening okay
    sigact.sa_flags=0;
    sigemptyset(&sigact.sa_mask);
    sigact.sa_handler=handleInterrupt;
    if(sigaction(SIGSEGV,&sigact,NULL)!=0)
      printf("Error calling sigaction SIGSEGV\n");
    if(sigaction(SIGBUS,&sigact,NULL)!=0)
      printf("Error calling sigaction SIGBUS\n");
    if(sigaction(SIGTERM,&sigact,NULL)!=0)
      printf("Error calling sigaction SIGTERM\n");
    if(sigaction(SIGINT,&sigact,NULL)!=0)
      printf("Error calling sigaction SIGINT\n");
    //Now open the shm, and write the port number into it.
    //This serves 2 purposes - it reserves the shm for us and also lets the process that started us know which port we are listening on.  (grabbing stdout doesn't work becasue we're supposed to run as a daemon).
    rstr->nd=1;
    rstr->dims[0]=(datasize-circCalcHdrSize()-HSIZE)/sizeof(int);
    rstr->nstore=1;
    rstr->dtype='i';
    openSHM(rstr);
    //now write port number
    FORCEWRITE(rstr->cb)=1;
    circAddSize(rstr->cb,&rstr->port,sizeof(int),0,0.,0);
    rstr->go=1;
    loop(rstr);
  }
  printf("receiver %s exiting\n",rstr->outputname);
  shm_unlink(rstr->outputname);
  pthread_mutex_destroy(&rstr->m);
  return 0;


}
