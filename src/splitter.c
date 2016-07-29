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
/*
$Id$
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
#include <sys/select.h>
#include <signal.h>
#include "circ.h"
typedef struct{
  char *shmprefix;
  int debug;
  //int decimate;
  char *streamname; 
  char *fullname;// "/" + shmprefix + streamname.
  char *shmname;// /dev/shm/shmprefix+streamname
  int shmOpen;
  circBuf *cb;
  circBuf *outbuf;
  int sock;
  int cumfreq;
  int go;
  int readFromHead;
  char *outputname;
  int nstore;//number of entries in output.
  void *data;
  int readfrom;
  int readto;
  int origreadto;
  int readstep;
  int readblock;
  int readpartial;

}SendStruct;

int openSHMReader(SendStruct *sstr){
  int cnt=1,n=0;
  struct stat st;
  sstr->shmOpen=0;
  while(sstr->shmOpen==0){
    if((sstr->cb=circOpenBufReader(sstr->fullname))!=NULL){
      sstr->shmOpen=1;
    }else{
      sstr->cb=NULL;
      sleep(1);
      n++;
      if(n==cnt){
	cnt*=2;
	printf("Failed to open /dev/shm%s\n",sstr->fullname);
      }
    }
    if(sstr->shmname!=NULL && stat(sstr->shmname,&st)!=0){//shm has gone?
      printf("Local SHM %s removed - splitter exiting...\n",sstr->shmname);
      exit(0);
    }
  }
  return 0;
}
char *shmname=NULL;//global required for signal handler.

int openSHMWriter(SendStruct *sstr){
  struct stat st;
  char *tmp;
  int i,dim;
  int size=1;
  if(asprintf(&tmp,"/dev/shm%s",sstr->outputname)==-1){
    printf("asprintf failed - unable to check for existance of %s so eting\n",sstr->outputname);
    return 1;
  }
  if(stat(tmp,&st)==0){//file exists - don't overwritem, but exit in error...
    printf("File in shm %s already exists - splitter exiting\n",tmp);
    free(tmp);
    tmp=NULL;
    return 1;
  }
  sstr->shmname=tmp;
  size=1;//get the total number of elements...
  for(i=0;i<(int)(NDIM(sstr->cb));i++)
    size*=SHAPEARR(sstr->cb)[i];
  
  if(sstr->nstore<0)
    sstr->nstore=NSTORE(sstr->cb);
  //Now compute size...
  if(sstr->origreadto<0){
    sstr->readto=size;
  }else if(sstr->origreadto>size){
    printf("Error - splitter trying to read beyond end of data... exiting\n");
    return 1;
  }else
    sstr->readto=sstr->origreadto;
  dim=((sstr->readto-sstr->readfrom+sstr->readstep-1)/sstr->readstep)*sstr->readblock;
  if((sstr->outbuf=openCircBuf(sstr->outputname,1,&dim,DTYPE(sstr->cb),sstr->nstore))==NULL){
    printf("Failed to open circular buffer %s\n",sstr->outputname);
    return 1;
  }
  shmname=sstr->outputname;
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
void handleInterrupt(int sig){
  if(shmname!=NULL){
    printf("Signal %d received - removing shm %s\n",sig,shmname);
    shm_unlink(shmname);
  }else
    printf("Splitter interrupt received (%d) - exiting\n",sig);
  exit(1);
}
int splitData(SendStruct *sstr,char *ret){
  int i,j,pos;
  char dtype;
  dtype=ret[16];
  ret=&ret[32];//skip the 32 byte header...
  switch(dtype){
  case 'f':
    pos=0;
    for(i=sstr->readfrom;i<sstr->readto;i+=sstr->readstep){
      for(j=i;j<i+sstr->readblock && j<sstr->readto;j++){
	((float*)sstr->data)[pos]=((float*)ret)[j];
	pos++;
      }
    }
    break;
  case 'd':
    pos=0;
    for(i=sstr->readfrom;i<sstr->readto;i+=sstr->readstep){
      for(j=i;j<i+sstr->readblock && j<sstr->readto;j++){
	((double*)sstr->data)[pos]=((double*)ret)[j];
	pos++;
      }
    }
    break;
  case 'i':
    pos=0;
    for(i=sstr->readfrom;i<sstr->readto;i+=sstr->readstep){
      for(j=i;j<i+sstr->readblock && j<sstr->readto;j++){
	((int*)sstr->data)[pos]=((int*)ret)[j];
	pos++;
      }
    }
    break;
  case 'H':
    pos=0;
    for(i=sstr->readfrom;i<sstr->readto;i+=sstr->readstep){
      for(j=i;j<i+sstr->readblock && j<sstr->readto;j++){
	((unsigned short*)sstr->data)[pos]=((unsigned short*)ret)[j];
	pos++;
      }
    }
    break;
  case 'h':
    pos=0;
    for(i=sstr->readfrom;i<sstr->readto;i+=sstr->readstep){
      for(j=i;j<i+sstr->readblock && j<sstr->readto;j++){
	((short*)sstr->data)[pos]=((short*)ret)[j];
	pos++;
      }
    }
    break;
  default:
    printf("Splitter not coded for type %c\n",dtype);
    break;
  }
  return 0;
}

int loop(SendStruct *sstr){
  //Waits for data, and sends it over socket.
  //first, open the buffer...
  void *ret;
  int lw;
  int diff;
  int cbfreq;
  int err=0;
  //struct timeval selectTimeout;
  struct stat st;
  int ihdrmsg[8];
  char *hdrmsg=(char*)ihdrmsg;
  int sleeping=1;
  //int checkDecimation;
  //selectTimeout.tv_sec=0;
  //selectTimeout.tv_usec=0;
  memset(hdrmsg,0,sizeof(ihdrmsg));
  circHeaderUpdated(sstr->cb);
    //printf("Last received %d last written %d\n",sstr->cb->lastReceived,LASTWRITTEN(sstr->cb));
  while(sstr->go && err==0){
    if(FREQ(sstr->outbuf)>0 || FORCEWRITE(sstr->outbuf)!=0){
      if(sleeping){//skip to head.
	sleeping=0;
	if(FREQ(sstr->cb)==0){//turn on the producer if necessary
	  FREQ(sstr->cb)=FREQ(sstr->outbuf);
	  FORCEWRITE(sstr->cb)+=(FREQ(sstr->outbuf)==0);
	}
	sstr->cumfreq=FREQ(sstr->outbuf);
	ret=circGetLatestFrame(sstr->cb);
      }
      //wait for data to be ready
      ret=circGetNextFrame(sstr->cb,1,1);
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
	  openSHMReader(sstr);
	  if(FREQ(sstr->cb)==0)
	    FREQ(sstr->cb)=FREQ(sstr->outbuf);
	  ret=circGetNextFrame(sstr->cb,1,1);
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
	  printf("Averaging of %s lagging - skipping %d frames\n",sstr->fullname,diff-1);
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
	//checkDecimation=0;
	if(sstr->cumfreq>=FREQ(sstr->outbuf)){//#so now send the data
	  sstr->cumfreq=0;
	  //data,timestamp,frameno=ret
	  //print "got data at %s %s %s"%(str(timestamp),str(data.shape),str(data.dtype.char))
	  if((FREQ(sstr->outbuf)%cbfreq)==0 && FREQ(sstr->outbuf)!=0){//attempt to synchronise frame numbers
	    //so that frame number is a multiple of decimate.
	    sstr->cumfreq=((int*)ret)[1]%FREQ(sstr->outbuf);//(sstr->decimate-((int*)ret)[1]%sstr->decimate)%sstr->decimate;
	  }
	  if(sstr->readFromHead && lw>=0){
	    ret=circGetFrame(sstr->cb,lw);//get the latest frame.
	  }
	  //check the shm to write too still exists..
	  if(stat(sstr->shmname,&st)!=0){//shm has gone?
	    printf("Local SHM %s removed - splitter exiting...\n",sstr->shmname);
	    exit(0);
	  }

	  //Check here - has the data type or shape changed?  If so, reset the average counters...
	  if((NDIM(sstr->cb)!=hdrmsg[6]) || (DTYPE(sstr->cb)!=hdrmsg[7]) || ihdrmsg[2]!=SHAPEARR(sstr->cb)[0]){//strncmp(&hdrmsg[8],(char*)SHAPEARR(sstr->cb),24)!=0){//shapearr is now 1d
	    
	    ihdrmsg[0]=28;
	    hdrmsg[4]=0x55;
	    hdrmsg[5]=0x55;
	    hdrmsg[6]=NDIM(sstr->cb);
	    hdrmsg[7]=DTYPE(sstr->cb);
	    //memcpy(&hdrmsg[8],SHAPEARR(sstr->cb),24);
	    ihdrmsg[2]=SHAPEARR(sstr->cb)[0];
	    if(sstr->debug)
	      printf("Changing datasize for %s\n",sstr->fullname);
	    if(sstr->origreadto<0){
	      int n,i,dim;
	      //datasize may have changed - in which case we'll need to realloc.
	      n=1;
	      for(i=0;i<NDIM(sstr->cb);i++)
		n*=SHAPEARR(sstr->cb)[i];
	      if(n!=sstr->readto){
		sstr->readto=n;
		dim=((sstr->readto-sstr->readfrom+sstr->readstep-1)/sstr->readstep)*sstr->readblock;
		circReshape(sstr->outbuf,1,&dim,DTYPE(sstr->cb));
		if(sstr->data!=NULL)
		  free(sstr->data);
		if((sstr->data=malloc(sstr->outbuf->datasize))==NULL){
		  printf("Unable to malloc data in splitter.c\n");
		  shm_unlink(sstr->outputname);
		  return 1;
		}
	      }
	    }		  
	  }
	  //Now sum the data
	  if(sstr->debug)
	    printf("Summing %s\n",sstr->fullname);
	  //printf("Summing frame %d, pointer %p %c %c\n",sstr->cb->lastReceived,ret,((char*)ret)[16],sstr->dtype);
	  splitData(sstr,ret);
	  if(sstr->debug)
	    printf("Writing split data to shm\n");
	  if(FORCEWRITE(sstr->outbuf)==0)
	    FORCEWRITE(sstr->outbuf)++;//make sure it adds it...
	  circAdd(sstr->outbuf,sstr->data,((double*)ret)[1],((int*)ret)[1]);
	}
      }
    }else{//turned off decimation... so just wait... can then either be turned back on, or can be killed...
      sleeping=1;
      if(stat(sstr->shmname,&st)!=0){//shm has gone?
	printf("Local SHM %s removed - splitter exiting...\n",sstr->shmname);
	exit(0);
      }
      sleep(1);
    }
  }
  return 0;
}

void *rotateLog(void *n){
  char **stdoutnames=NULL;
  int nlog=4;
  int logsize=80000;
  FILE *fd;
  char *fullname=(char*)n;
  struct stat st; 
  int i;
  umask(0);
  stdoutnames=calloc(nlog,sizeof(char*));
  for(i=0; i<nlog; i++){
    if(asprintf(&stdoutnames[i],"/dev/shm/%sSplitterStdout%d",fullname,i)<0){
      printf("rotateLog filename creation failed\n");
      return NULL;
    }
  }
  printf("redirecting stdout to %s...\n",stdoutnames[0]);
  fd=freopen(stdoutnames[0],"a+",stdout);
  setvbuf(fd,NULL,_IOLBF,0);
  printf("rotateLog started\n");
  printf("New log cycle\n");
  while(1){
    sleep(60);
    fstat(fileno(fd),&st);
    if(st.st_size>logsize){
      printf("LOGROTATE\n");
      for(i=nlog-1; i>0; i--){
	rename(stdoutnames[i-1],stdoutnames[i]);
      }
      fd=freopen(stdoutnames[0],"w",stdout);
      setvbuf(fd,NULL,_IOLBF,0);
      printf("New log cycle\n");
    }
  }
}

int main(int argc, char **argv){
  int setprio=0;
  int affin=0x7fffffff;
  int prio=0;
  SendStruct *sstr;
  int i,redirect=0;
  struct sigaction sigact;
  if((sstr=malloc(sizeof(SendStruct)))==NULL){
    printf("Unable to malloc SendStruct\n");
    return 1;
  }
  memset(sstr,0,sizeof(SendStruct));
  sstr->go=1;
  sstr->nstore=-1;
  sstr->readfrom=0;
  sstr->readto=-1;
  sstr->readstep=1;
  sstr->readblock=1;
  for(i=1; i<argc; i++){
    if(argv[i][0]=='-'){
      switch(argv[i][1]){
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
      case 'h'://probably a data display - want quick update, not every frame.
	sstr->readFromHead=1;
	break;
      case 'o':
	sstr->outputname=&argv[i][2];
	break;
      case 'S':
	sstr->nstore=atoi(&argv[i][2]);
	break;
      case 'F'://not reading all the data - start From this value
	sstr->readfrom=atoi(&argv[i][2]);
	if(sstr->readfrom>0)
	  sstr->readpartial=1;
	break;
      case 'T'://not reading all the data - read To this value
	sstr->origreadto=atoi(&argv[i][2]);
	if(sstr->origreadto>0)
	  sstr->readpartial=1;
	break;
      case 'J'://not reading all the data - Jump (skip) by this amount
	sstr->readstep=atoi(&argv[i][2]);
	if(sstr->readstep>1)
	  sstr->readpartial=1;
	break;
      case 'b':
	sstr->readblock=atoi(&argv[i][2]);
	if(sstr->readblock>0)
	  sstr->readpartial=1;
	break;
      case 'q':
	redirect=1;
	break;
      default:
	break;
      }
    }else{//the stream name
      sstr->streamname=argv[i];
    }
  }
  if(sstr->readstep<1){
    printf("Illegal value for readstep in splitter - exiting\n");
    return 1;
  }
  if(sstr->readblock<1){
    printf("Illegal value for readblock in splitter - exiting\n");
    return 1;
  }
  if(sstr->readpartial==0){
    printf("Splitter not required - exiting\n");
    return 0;
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
  if(sstr->outputname==NULL){
    if(asprintf(&sstr->outputname,"/%sf%dt%dj%db%d%sBuf",&sstr->fullname[1],sstr->readfrom,sstr->origreadto,sstr->readstep,sstr->readblock,sstr->readFromHead?"Head":"Tail")==-1){
      printf("Error asprintf2\n");
      return 1;
    }
  }
  printf("Splitter starting for %s\n",sstr->outputname);
  if(setprio){
    setThreadAffinity(affin,prio);
  }

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


  if(redirect){//redirect stdout to a file...
    pthread_t logid;
    if(pthread_create(&logid,NULL,rotateLog,&sstr->outputname[1])){
      printf("pthread_create rotateLog failed\n");
    }
  }
  openSHMReader(sstr);
  if(openSHMWriter(sstr)){
    printf("Failed to open SHM to write to %s",sstr->outputname);
    return 1;
  }
  //FREQ(sstr->outbuf)=decimate;

  if((sstr->data=malloc(sstr->outbuf->datasize))==NULL){
    printf("Unable to malloc data in splitter.c\n");
    shm_unlink(sstr->outputname);
    return 1;
  }
  sstr->go=1;
  //sstr->cumfreq=decimate;
  loop(sstr);
  shm_unlink(sstr->outputname);
  return 0;
}
