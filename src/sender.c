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
#include <sys/select.h>

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
  int sock;
  int attempts;
  int cumfreq;
  int go;
  int sendSerialisedHdr;
  int readFromHead;
  char *saver;
}SendStruct;


int initSockaddr (struct sockaddr_in *name,const char *hostname,uint16_t port){
  struct hostent *hostinfo;
  name->sin_family = AF_INET;
  name->sin_port = htons (port);
  hostinfo = gethostbyname (hostname);
  if (hostinfo == NULL){
    printf("Unknown host %s.\n", hostname);
    return 1;
  }
  name->sin_addr = *(struct in_addr *) hostinfo->h_addr;
  return 0;
}

int connectsock(const char *host,int port){
  int sock;
  struct sockaddr_in servername;
  // Create the socket.
  sock = socket (PF_INET, SOCK_STREAM, 0);
  if (sock < 0){
    printf("socket error %s\n",strerror(errno));
    return -1;
  }
  // Connect to the server.
  if(initSockaddr (&servername, host,(uint16_t)port)!=0){
    close(sock);
    return -1;
  }
  if(connect(sock,(struct sockaddr *) &servername,sizeof (servername))!=0){
    printf("connect error %s\n",strerror(errno));
    close(sock);
    return -1;
  }
  return sock;
}


int serialiseSend(int sock,char *fmt,...){
  //incomplete function - currently only sends strings... and assumes that these are wrapped in a list.  But for now, it should do...
  va_list ap;
  char data[80];
  char *s;
  int l;
  int datapos=5;
  int err=0;
  int n;
  int nsent=0;
  //int i;
  va_start(ap,fmt);
  data[0]=23;//list, little endian (lsb)
  while(*fmt && err==0){
    switch(*fmt++){
    case 's'://string
      s=va_arg(ap,char*);
      l=strlen(s)+1;//include the null.
      if(datapos+5+l>=80){
	printf("Error - serialiseSend buffer not big enough\n");
	exit(EXIT_FAILURE);
      }
      data[datapos++]=1;//string, little endian
      data[datapos++]=(char)((l>>24)&0xff);
      data[datapos++]=(char)((l>>16)&0xff);
      data[datapos++]=(char)((l>>8)&0xff);
      data[datapos++]=(char)(l&0xff);
      memcpy(&data[datapos],s,l);
      datapos+=l;
      break;
      //case 'd'://int
      //d=va_arg(ap,int);
      //break;
      //case 'c'://char
      //c=(char)va_arg(ap,int);//care with promoted types...
      //break;
    default:
      printf("Type not recognised in serialiseSend: exiting\n");
      exit(EXIT_FAILURE);
      break;
    }
  }
  va_end(ap);
  datapos-=5;
  data[1]=(char)((datapos>>24)&0xff);
  data[2]=(char)((datapos>>16)&0xff);
  data[3]=(char)((datapos>>8)&0xff);
  data[4]=(char)(datapos&0xff);
  datapos+=5;
  /*printf("Sending to %d\n",sock);
  for(i=0; i<datapos; i++){
    printf("%d ",data[i]);
  }
  printf("\n");*/
  while(nsent<datapos && err==0){
    if((n=send(sock,&data[nsent],datapos-nsent,0))==-1){
      printf("Failed to send\n");
      err=1;
    }else{
      nsent+=n;
    }
  }
  //printf("send %d bytes/%d\n",nsent,datapos);
  return err;
} 


int openSHM(SendStruct *sstr){
  int cnt=1,n=0;
  sstr->shmOpen=0;
  while(sstr->shmOpen==0){
    if((sstr->cb=circOpenBufReader(sstr->fullname))!=NULL){
      sstr->shmOpen=1;
    }else{
      printf("Failed to open /dev/shm%s\n",sstr->fullname);
      sstr->cb=NULL;
      sleep(1);
      n++;
      if(n==cnt)
	cnt*=2;
    }
  }
  printf("/dev/shm%s opened\n",sstr->fullname);
  return 0;
}

int connectReceiver(SendStruct *sstr){
  sstr->sock=0;
  while(sstr->sock==0 && sstr->attempts!=0){
    if(sstr->attempts>0)
      sstr->attempts-=1;
    if((sstr->sock=connectsock(sstr->host,sstr->port))<0){
      printf("failed to connect to %s port %d: %s\n",sstr->host,sstr->port,strerror(errno));
      sstr->sock=0;
      sleep(1);
    }else{
      printf("Connected to Receiver %s port %d\n",sstr->host,sstr->port);
    }
  }
  if(sstr->sock==0){
    //exit(EXIT_FAILURE);
    return 1;
  }
  if(sstr->sendSerialisedHdr){
    if(serialiseSend(sstr->sock,"ss","name",&sstr->fullname[1])!=0){
      printf("serialiseSend failed\n");
      close(sstr->sock);
      sstr->sock=0;
      return 1;
      //exit(EXIT_FAILURE);
    }
    printf("name sent %s\n",&sstr->fullname[1]);
    if(sstr->raw){//inform that we'll be sending raw data...
      printf("Sending raw flag\n");
      if(serialiseSend(sstr->sock,"sss","raw","raw",&sstr->fullname[1])!=0){
	printf("serialiseSend failed...\n");
	close(sstr->sock);
	sstr->sock=0;
	return 1;
	//exit(EXIT_FAILURE);
      }
    }
  }
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

int checkSHM(SendStruct *sstr){
  //Check to see whether open shm is same as newly opened shm.
  //Return 1 on failure - ie if not the same, or if not openable.
  int fd;
  int *buf,*cbuf;
  int hdrsize;
  int i,rt;
  if(sstr->cb==NULL){
    return 1;
  }
  //now try mapping the shm, and read the header.  Compare this header with the current circbuf.
  if((fd=shm_open(sstr->fullname,O_RDONLY,0))==-1){
    //printf("shm_open failed for %s:%s\n",name,strerror(errno));
    return 1;
  }
  hdrsize=(int)((long)sstr->cb->data-(long)sstr->cb->mem);
  //printf("checkSHM got hdrsize %d\n",hdrsize);
  buf=(int*)mmap(0,hdrsize,PROT_READ,MAP_SHARED,fd,0);
  close(fd);
  if(buf==MAP_FAILED){
    //printf("mmap failed for\n");
    //printf("%s with error number %d, error\n",name,errno);
    //printf("%s\n",strerror(errno));
    return 1;
  }
  //now compare the two buffers...
  rt=1;
  cbuf=(int*)sstr->cb->mem;
  if(cbuf[0]==buf[0] && cbuf[1]==buf[1]){
    rt=0;
    for(i=3; i<hdrsize/sizeof(int); i++){
      if(cbuf[i]!=buf[i]){
	rt=1;
	break;
      }
    }
  }
  munmap(buf,hdrsize);
  return rt;
}


int loop(SendStruct *sstr){
  //Waits for data, and sends it over socket.
  //first, open the buffer...
  void *ret;
  int lw;
  int diff;
  int cbfreq;
  int err=0;
  struct timeval selectTimeout;
  char hdrmsg[32];
  fd_set readfd;
  selectTimeout.tv_sec=0;
  selectTimeout.tv_usec=0;
  memset(hdrmsg,0,sizeof(hdrmsg));
  sstr->cumfreq=sstr->decimate;
  circHeaderUpdated(sstr->cb);
  if(sstr->startWithLatest){
    ret=circGetLatestFrame(sstr->cb);
    //printf("Last received %d last written %d\n",sstr->cb->lastReceived,LASTWRITTEN(sstr->cb));
  }
  while(sstr->go && err==0){
    //wait for data to be ready
    ret=circGetNextFrame(sstr->cb,10,1);
    //printf("Last received %d last written %d\n",sstr->cb->lastReceived,LASTWRITTEN(sstr->cb));
    
    if(sstr->debug){
      if(ret==NULL){
	printf("No data yet %s\n",sstr->fullname);
      }else{
	printf("Got data %s\n",sstr->fullname);
      }
    }
    //How can I tell if a timeout occurred because rtc is dead/restarted or because its paused/not sending?
    //First, open the shm in a new array, and look at header.  If header is same as existing, don't do anything else.  Otherwise, reopen and reinitialise the circular buffer.
    if(ret==NULL){
      if(checkSHM(sstr)){//returns 1 on failure...
	printf("Reopening SHM\n");
	openSHM(sstr);
	ret=circGetNextFrame(sstr->cb,10,1);
      }else{
	//shm still valid - probably timeout occurred, meaning RTC still dead, or just not producing this stream.
      }
      //Check to see if we're lagging behind the RTC - if so, send the latest frame...
    }
    lw=LASTWRITTEN(sstr->cb);//circbuf.lastWritten[0];
    if(lw>=0){
      diff=lw-sstr->cb->lastReceived;
      if(diff<0){
	diff+=NSTORE(sstr->cb);//circbuf.nstore[0];
      }
      //printf("diff %d %d %d\n",diff,lw,sstr->cb->lastReceived);
      if(diff>NSTORE(sstr->cb)*0.75){//ircbuf.nstore[0]*.75){
	printf("Sending of %s lagging - skipping %d frames\n",sstr->fullname,diff-1);
	ret=circGetFrame(sstr->cb,lw);//sstr->circbuf.get(lw,copy=1);
      }
    }
    if(ret!=NULL){
      cbfreq=FREQ(sstr->cb);//int(sstr->circbuf.freq[0]);
      if(cbfreq<1){
	cbfreq=1;
      }
      sstr->cumfreq+=cbfreq;
      sstr->cumfreq-=sstr->cumfreq%cbfreq;
      if(sstr->cumfreq>=sstr->decimate){//#so now send the data
	sstr->cumfreq=0;
	//data,timestamp,frameno=ret
	//print "got data at %s %s %s"%(str(timestamp),str(data.shape),str(data.dtype.char))
	if(sstr->readFromHead && lw>=0){
	  ret=circGetFrame(sstr->cb,lw);//get the latest frame.
	}
	if(sstr->saver!=NULL && ret!=NULL){
	  if(sstr->raw){
	    printf("todo - saver.writeRaw\n");
	    //sstr->saver.writeRaw(ret);
	  }else{
	    printf("non-raw not yet implemented\n");
	    //data,ftime,fno=ret;
	    //sstr->saver.write(data,ftime,fno);
	  }
	}
	//send the data
	//print "sending",sstr->sock
	if(sstr->sock!=0){
	  //Check here - has the data type or shape changed?  If so, send this info first.
	  if((NDIM(sstr->cb)!=hdrmsg[6]) || (DTYPE(sstr->cb)!=hdrmsg[7]) || strncmp(&hdrmsg[8],(char*)SHAPEARR(sstr->cb),24)!=0){
	    int nsent,n;
	    ((int*)hdrmsg)[0]=28;
	    hdrmsg[4]=0x55;
	    hdrmsg[5]=0x55;
	    hdrmsg[6]=NDIM(sstr->cb);
	    hdrmsg[7]=DTYPE(sstr->cb);
	    memcpy(&hdrmsg[8],SHAPEARR(sstr->cb),24);
	    if(sstr->debug)
	      printf("Sending shape info for %s\n",sstr->fullname);
	    nsent=0;
	    err=0;
	    //change in shape etc.
	    while(nsent<32 && err==0){
	      if((n=send(sstr->sock,&hdrmsg[nsent],32-nsent,0))<0){
		printf("error writing new shape info to socket - closing\n");
		err=1;
		close(sstr->sock);
		sstr->sock=0;
		sstr->go=0;
	      }else{
		nsent+=n;
	      }
	    }
	  }
	  //Now send the data/
	  if(sstr->debug)
	    printf("Sending %s\n",sstr->fullname);
	  if(sstr->raw && ret!=NULL){
	    if(sstr->sock!=0){
	      int size,nsent,n;
	      size=((int*)ret)[0]+4;
	      nsent=0;
	      err=0;
	      //printf("Sending size %d\n",size);
	      while(nsent<size && err==0){
		if((n=send(sstr->sock,&(((char*)ret)[nsent]),size-nsent,0))<0){
		  printf("Error writing raw data to socket - closing socket\n");
		  err=1;
		  close(sstr->sock);
		  sstr->sock=0;
		  //exit(EXIT_FAILURE);
		  sstr->go=0;
		}else{
		  nsent+=n;
		}
	      }
	    }
	  }else{
	    printf("non-raw not yet implemented - not sending\n");
	    //if(serialise.Send(["data",&sstr->fullname[1],ret],sstr->sock)){
	    //printf("error in serialise.Send - exiting - finishing sending of %s\n",sstr->fullname);
	    //sstr->go=0;
	    //}
	  }
	  //Finally, see if the socket has anything to read - if so , this will be a int32, a new decimate rate.
	  //We have to do this non-blocking, so use a select call.
	  if(err==0){
	    FD_ZERO(&readfd);
	    FD_SET(sstr->sock, &readfd);
	    if((err=select(sstr->sock+1,&readfd,NULL,NULL,&selectTimeout))>0){
	      int dec;
	      //something to read...
	      if(recv(sstr->sock,&dec,sizeof(int),0)!=sizeof(int)){
		printf("Error reading decimation in sender\n");
	      }else{
		sstr->decimate=dec;
		printf("Setting sender decimate to %d\n",dec);
	      }
	    }else if(err<0){
	      //error during select.
	      printf("Error during select call while waiting for decimation value\n");
	      err=1;
	      close(sstr->sock);
	      sstr->sock=0;
	      sstr->go=0;
	    }
	  }

	}
      }
    }
  }
  if(sstr->saver!=NULL){
    printf("todo - saver.close\n");
    //sstr->saver.close();
  }
  return 0;
}


int main(int argc, char **argv){
  int setprio=0;
  int affin=0x7fffffff;
  int prio=0;
  SendStruct *sstr;
  int i;
  int err=0;
  if((sstr=malloc(sizeof(SendStruct)))==NULL){
    printf("Unable to malloc SendStruct\n");
    return 1;
  }
  memset(sstr,0,sizeof(SendStruct));
  sstr->port=4243;
  sstr->connect=1;
  sstr->decimate=1;
  sstr->go=1;
  sstr->attempts=-1;
  sstr->sendSerialisedHdr=1;
  for(i=1; i<argc; i++){
    if(argv[i][0]=='-'){
      switch(argv[i][1]){
      case 'p':
	sstr->port=atoi(&argv[i][2]);
	break;
      case 'h':
	sstr->host=&argv[i][2];
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
	sstr->shmprefix=&argv[i][2];
	break;
      case 'v':
	sstr->debug=1;
	break;
      case 'd':
	sstr->decimate=atoi(&argv[i][2]);
	break;
      case 'l':
	sstr->log=1;
	break;
      case 'r':
	sstr->raw=1;
	break;
      case 'c':
	sstr->connect=0;
	break;
      case 't':
	sstr->attempts=atoi(&argv[i][2]);
	break;
      case 'n':
	sstr->startWithLatest=1;
	break;
      case 'R':
	sstr->sendSerialisedHdr=0;
	break;
      case 'f'://probably a data display - want quick update, not every frame.
	sstr->readFromHead=1;
	break;
      default:
	break;
      }
    }else{//the stream name
      sstr->streamname=argv[i];
    }
  }
  if(sstr->streamname==NULL){
    printf("Must specify a stream\n");
    return 1;
  }
  if(sstr->shmprefix==NULL){
    if(asprintf(&sstr->fullname,"/%s",sstr->streamname)==-1){
      printf("Error asprintf\n");
      return 1;
    }
  }else{
    if(asprintf(&sstr->fullname,"/%s%s",sstr->shmprefix,sstr->streamname)==-1){
      printf("Error asprintf\n");
      return 1;
    }
  }
  if(setprio){
    setThreadAffinity(affin,prio);
  }

  openSHM(sstr);
  if(sstr->connect && sstr->host!=NULL){
    if((err=connectReceiver(sstr))!=0){
      printf("Couldn't connect\n");
    }
  }
  if(err==0){
    sstr->go=1;
    loop(sstr);
  }
  printf("sender %s exiting\n",sstr->fullname);
  return 0;
}
