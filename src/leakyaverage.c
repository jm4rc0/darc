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

/*
Integrates a signal using:
a_n = (1-g)a_{n-1} + gv_n

Has the option that if the stream stops, or disappears, continues to integrate with the last known version, with a specified time-period, or the mean time period of previous measurements.

Used for computing average DM shape for the creep-prone ALPAO DM.

*/

#define _GNU_SOURCE
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <fcntl.h>
//#include <sys/socket.h>
//#include <netinet/in.h>
//#include <netdb.h>
#include <string.h>
#include <stdarg.h>
#include <sched.h>
//#include <sys/select.h>
#include <signal.h>
#include <math.h>
#include "circ.h"
typedef struct{
  char *shmprefix;
  int debug;
  int startWithLatest;
  //int decimate;
  char *streamname; 
  char *fullname;// "/" + shmprefix + streamname.
  int shmOpen;
  circBuf *cb;
  circBuf *outbuf;
  circBuf *out2buf;
  int go;
  int readFromHead;
  float gain;
  float timeout;
  char *outputname;
  char *squareoutputname;
  char *shmname;// /dev/shm+outputname
  char *shm2name;// /dev/shm+outputname
  int nstore;//number of entries in output.
  float *data;
  float *data2;
  int sumsquare;
  double latestIntegTime;
  char *store;
  int cnt;
  int latestcbsize;
}SendStruct;



int openSHMReader(SendStruct *sstr,int block){
  int cnt=1,n=0;
  struct stat st;
  sstr->shmOpen=0;
  while(sstr->shmOpen==0){
    if((sstr->cb=circOpenBufReader(sstr->fullname))!=NULL){
      sstr->shmOpen=1;
    }else{
      sstr->cb=NULL;
      if(block){
	sleep(1);
	n++;
	if(n==cnt){
	  cnt*=2;
	  printf("Failed to open /dev/shm%s\n",sstr->fullname);
	}
      }else{
	printf("Failed to open /dev/shm%s\n",sstr->fullname);
	return 1;
      }
    }
    if(sstr->shmname!=NULL && stat(sstr->shmname,&st)!=0){//shm has gone? (ie output no longer required)
      printf("Local SHM %s removed - leakyaverage exiting...\n",sstr->shmname);
      if(sstr->sumsquare)
	shm_unlink(sstr->squareoutputname);
      exit(0);
    }
    if(sstr->sumsquare){
      if(sstr->shm2name!=NULL && stat(sstr->shm2name,&st)!=0){//shm has gone?
	printf("Local SHM %s removed - leakyaverage exiting...\n",sstr->shm2name);
	shm_unlink(sstr->outputname);
	exit(0);
      }
    }
  }
  return 0;
}
char *shmname=NULL;//global required for signal handler.
char *shm2name=NULL;//global required for signal handler.

int openSHMWriter(SendStruct *sstr,int squared){
  struct stat st;
  char *tmp;
  circBuf *ctmp;
  if(asprintf(&tmp,"/dev/shm%s",squared?sstr->squareoutputname:sstr->outputname)==-1){
    printf("asprintf failed - unable to check for existance of %s so eting\n",squared?sstr->squareoutputname:sstr->outputname);
    return 1;
  }
  if(stat(tmp,&st)==0){//file exists - don't overwritem, but exit in error...
    printf("File in shm %s already exists - leakyaverage exiting\n",tmp);
    free(tmp);
    tmp=NULL;
    if(squared)//need to unlink the standard one too
      shm_unlink(sstr->outputname);

    return 1;
  }
  if(squared)
    sstr->shm2name=tmp;
  else
    sstr->shmname=tmp;
  if((ctmp=openCircBuf(squared?sstr->squareoutputname:sstr->outputname,(int)(NDIM(sstr->cb)),SHAPEARR(sstr->cb),'f',sstr->nstore))==NULL){
    printf("Failed to open circular buffer %s\n",squared?sstr->squareoutputname:sstr->outputname);
    return 1;
  }
  if(squared){
    sstr->out2buf=ctmp;
    shm2name=sstr->squareoutputname;
  }else{
    sstr->outbuf=ctmp;
    shmname=sstr->outputname;
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
void handleInterrupt(int sig){
  if(shmname!=NULL){
    printf("Signal %d received - removing shm %s\n",sig,shmname);
    shm_unlink(shmname);
  }else
    printf("Leakyaverage interrupt received (%d) - exiting\n",sig);
  if(shm2name!=NULL){
    printf("Signal %d received - removing shm %s\n",sig,shm2name);
    shm_unlink(shm2name);
  }
  exit(1);
}
int leakyIntegrateData(SendStruct *sstr,char *ret,int ntimes){
  int n=1,i;
  char dtype;
  float gain=sstr->gain,decay=(1-sstr->gain);
  if(sstr->cb!=NULL){
    for(i=0; i<NDIM(sstr->cb); i++){
      n*=SHAPEARR(sstr->cb)[i];
    }
    sstr->latestcbsize=n;
  }else{
    n=sstr->latestcbsize;
  }
  dtype=ret[16];
  ret=&ret[32];//skip the 32 byte header...
  if(sstr->cnt==0){//first time...
    gain=1.;
    decay=0.;
    memset(sstr->data,0,sizeof(float)*n);
    if(sstr->data2!=NULL)
      memset(sstr->data2,0,sizeof(float)*n);
  }else{
    //need to adjust decay and gain if integrating more than 1 time - i.e. rather than actually looping through the integral n times, can use maths (geometric progression).
    gain*=(1-powf(decay,(float)ntimes))/(1-decay);
    decay=powf(decay,(float)ntimes);
  }
  sstr->cnt+=ntimes;
  for(i=0;i<n;i++)//first decay existing data
    ((float*)sstr->data)[i]*=decay;
  if(dtype=='f'){//and now add the rest.
    for(i=0;i<n;i++)
      ((float*)sstr->data)[i]+=gain*(float)((float*)ret)[i];
  }else if(dtype=='d'){
    for(i=0;i<n;i++)
      ((float*)sstr->data)[i]+=gain*(float)((double*)ret)[i];
  }else if(dtype=='i'){
    for(i=0;i<n;i++)
      ((float*)sstr->data)[i]+=gain*(float)((int*)ret)[i];
  }else if(dtype=='h'){
    for(i=0;i<n;i++)
      ((float*)sstr->data)[i]+=gain*(float)((short*)ret)[i];
  }else if(dtype=='H'){
    for(i=0;i<n;i++)
      (((float*)sstr->data)[i])+=gain*(float)(((unsigned short*)ret)[i]);
  }else{
    printf("Not coded for type %c in leakyaverage.c\n",dtype);
  }
  if(sstr->sumsquare){
    for(i=0;i<n;i++)
      ((float*)sstr->data2)[i]*=decay;
    if(dtype=='f'){
      for(i=0;i<n;i++)
	((float*)sstr->data2)[i]+=gain*(float)((float*)ret)[i]*(float)((float*)ret)[i];
    }else if(dtype=='d'){
      for(i=0;i<n;i++)
	((float*)sstr->data2)[i]+=gain*(float)((double*)ret)[i]*(float)((double*)ret)[i];
    }else if(dtype=='i'){
      for(i=0;i<n;i++)
	((float*)sstr->data2)[i]+=gain*(float)((int*)ret)[i]*(float)((int*)ret)[i];
    }else if(dtype=='h'){
      for(i=0;i<n;i++)
	((float*)sstr->data2)[i]+=gain*(float)((short*)ret)[i]*(float)((short*)ret)[i];
    }else if(dtype=='H'){
      for(i=0;i<n;i++)
	(((float*)sstr->data2)[i])+=gain*(float)(((unsigned short*)ret)[i])*(float)(((unsigned short*)ret)[i]);
    }
  }
  return 0;
}


int loop(SendStruct *sstr){
  //Waits for data, and sends it over socket.
  //first, open the buffer...
  void *ret;
  int lw;
  int diff;
  int err=0;
  struct timeval now;
  //struct timeval selectTimeout;
  int ihdrmsg[8];
  char *hdrmsg=(char*)ihdrmsg;
  struct stat st;
  double tdiff;
  double tstamp;
  int fno;
  int writeCirc=0;
  //int checkDecimation;
  //selectTimeout.tv_sec=0;
  //selectTimeout.tv_usec=0;
  memset(hdrmsg,0,sizeof(ihdrmsg));
  circHeaderUpdated(sstr->cb);
  if(sstr->startWithLatest){
    ret=circGetLatestFrame(sstr->cb);
    //printf("Last received %d last written %d\n",sstr->cb->lastReceived,LASTWRITTEN(sstr->cb));
  }
  while(sstr->go && err==0){
    writeCirc=0;
    //wait for data to be ready
    if(sstr->cb!=NULL)
      ret=circGetNextFrame(sstr->cb,sstr->timeout,0);
    else
      ret=NULL;
    if(sstr->debug){
      if(ret==NULL) printf("Leaky averager: No data yet %s\n",sstr->fullname);
      else printf("Leaky averager: Got data %s\n",sstr->fullname);
    }
    if(ret==NULL){
      //How can I tell if a timeout occurred because rtc is dead/restarted or because its paused/not sending?
      //First, open the shm in a new array, and look at header.  If header is same as existing, don't do anything else.  Otherwise, reopen and reinitialise the circular buffer.
      //If enough time has passed since we last integrated, we should integrated the more recent data.
      gettimeofday(&now,NULL);
      tdiff=now.tv_sec+1e-6*now.tv_usec-sstr->latestIntegTime;
      if(tdiff > sstr->timeout*1.2){
	if(sstr->debug)
	  printf("Averaging Store %s\n",sstr->fullname);
	leakyIntegrateData(sstr,sstr->store,(int)(tdiff/sstr->timeout));
	sstr->latestIntegTime+=sstr->timeout*(int)(tdiff/sstr->timeout);
	writeCirc=1;
      }
      if(checkSHM(sstr)){//returns 1 on failure...  not present, or has changed.
	printf("Leaky averager: Reopening SHM\n");
	if(openSHMReader(sstr,0)!=0){//error opening...
	  usleep(sstr->timeout*1e6);
	}else{
	  if(FREQ(sstr->cb)==0)
	    FREQ(sstr->cb)=1;
	  ret=circGetNextFrame(sstr->cb,sstr->timeout,0);
	}
      }else{
	//shm still valid - probably timeout occurred, meaning RTC still dead, or just not producing this stream.
      }
    }
    //Check to see if we're lagging behind the RTC - if so, send the latest frame...
    if(sstr->cb!=NULL){
      lw=LASTWRITTEN(sstr->cb);//circbuf.lastWritten[0];
      if(lw>=0){
	diff=lw-sstr->cb->lastReceived;
	if(diff<0){
	  diff+=NSTORE(sstr->cb);//circbuf.nstore[0];
	}
	//printf("diff %d %d %d\n",diff,lw,sstr->cb->lastReceived);
	if(diff>NSTORE(sstr->cb)*0.75){//ircbuf.nstore[0]*.75){
	  printf("Leaky averaging of %s lagging - skipping %d frames\n",sstr->fullname,diff-1);
	  ret=circGetFrame(sstr->cb,lw);//sstr->circbuf.get(lw,copy=1);
	}
      }
    }
    if(ret!=NULL){
      gettimeofday(&now,NULL);
      tdiff=now.tv_sec+1e-6*now.tv_usec-sstr->latestIntegTime;
      if(tdiff > sstr->timeout*1.2 && sstr->latestIntegTime!=0){
	//a delay has occurred since last data received... 
	//So integrate with the last data received for enough time...
	if(sstr->debug)
	  printf("Averaging store %s\n",sstr->fullname);
	leakyIntegrateData(sstr,sstr->store,(int)(tdiff/sstr->timeout));
	sstr->latestIntegTime+=sstr->timeout*(int)(tdiff/sstr->timeout);
	writeCirc=1;
      }
      //check the shm to write too still exists..
      if(stat(sstr->shmname,&st)!=0){//shm has gone?
	printf("Local SHM %s removed - leakyaverage exiting...\n",sstr->outputname);
	if(sstr->sumsquare)
	  shm_unlink(sstr->squareoutputname);
	
	exit(0);
      }
      if(sstr->sumsquare){
	if(stat(sstr->shm2name,&st)!=0){//shm has gone?
	  printf("Local shm %s removed - leakyaverage exiting\n",sstr->squareoutputname);
	  shm_unlink(sstr->outputname);
	  
	  exit(0);
	}
      }
      //integrate the data
      //Check here - has the data type or shape changed?  If so, reset the average counters...
      if(sstr->cb!=NULL && ((NDIM(sstr->cb)!=hdrmsg[6]) || (DTYPE(sstr->cb)!=hdrmsg[7]) || strncmp(&hdrmsg[8],(char*)SHAPEARR(sstr->cb),24)!=0)){
	sstr->cnt=0;
	ihdrmsg[0]=28;
	hdrmsg[4]=0x55;
	hdrmsg[5]=0x55;
	hdrmsg[6]=NDIM(sstr->cb);
	hdrmsg[7]=DTYPE(sstr->cb);
	memcpy(&hdrmsg[8],SHAPEARR(sstr->cb),24);
	if(sstr->debug)
	  printf("Resetting counter for %s\n",sstr->fullname);
	circReshape(sstr->outbuf,(int)NDIM(sstr->cb),SHAPEARR(sstr->cb),'f');
	if(sstr->sumsquare)
	  circReshape(sstr->out2buf,(int)NDIM(sstr->cb),SHAPEARR(sstr->cb),'f');
      }
      //Now sum the data
      if(sstr->debug)
	printf("Averaging %s\n",sstr->fullname);
      leakyIntegrateData(sstr,ret,1);
      sstr->latestIntegTime=now.tv_sec+1e-6*now.tv_usec;
      memcpy(sstr->store,ret,((sstr->outbuf->datasize+HSIZE+ALIGN-1)/ALIGN)*ALIGN);//for future integration if buf freezes
      writeCirc=1;
    }
    if(writeCirc){
      writeCirc=0;
      //FORCEWRITE(sstr->outbuf)++;
      if(ret!=NULL){
	tstamp=((double*)ret)[1];
	fno=((int*)ret)[1];
      }else{
	tstamp=sstr->latestIntegTime;//((double*)sstr->store)[1];
	fno=((int*)sstr->store)[1];
      }
	
      circAdd(sstr->outbuf,sstr->data,tstamp,fno);
      if(sstr->sumsquare){
	//FORCEWRITE(sstr->out2buf)++;
	circAdd(sstr->out2buf,sstr->data2,tstamp,fno);
      }
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
    if(asprintf(&stdoutnames[i],"/dev/shm/%sLeakyaverageStdout%d",fullname,i)<0){
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
  int i,decimate=1,redirect=0;
  struct sigaction sigact;
  if((sstr=malloc(sizeof(SendStruct)))==NULL){
    printf("Unable to malloc SendStruct\n");
    return 1;
  }
  memset(sstr,0,sizeof(SendStruct));
  sstr->go=1;
  sstr->nstore=10;
  sstr->gain=0.00001;
  sstr->timeout=0.01;//100Hz
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
      case 'd':
	decimate=atoi(&argv[i][2]);
	break;
      case 'l':
	sstr->startWithLatest=1;
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
      case 'q':
	redirect=1;
	break;
      case '2':
	sstr->sumsquare=1;
	break;
      case 'g':
	sstr->gain=atof(&argv[i][2]);
	break;
      case 't':
	sstr->timeout=atof(&argv[i][2]);
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
  if(sstr->outputname==NULL){
    if(asprintf(&sstr->outputname,"/%sLeakyAv%gf%sBuf",&sstr->fullname[1],sstr->gain,sstr->readFromHead?"Head":"Tail")==-1){
      printf("Error asprintf2\n");
      return 1;
    }
    if(sstr->sumsquare){
      if(asprintf(&sstr->squareoutputname,"/%s2LeakyAv%gf%sBuf",&sstr->fullname[1],sstr->gain,sstr->readFromHead?"Head":"Tail")==-1){
	printf("Error asprintf3\n");
	return 1;
      }
    }
  }else if(sstr->sumsquare){//outputname defined - need to create a similar one for squares...
    if(asprintf(&sstr->squareoutputname,"%s2Buf",sstr->outputname)==-1){
      printf("Error asprintf4\n");
      return 1;
    }
  }
  printf("Leakyaverage starting for %s\n",sstr->outputname);
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
  openSHMReader(sstr,1);
  if(FREQ(sstr->cb)==0)
    FREQ(sstr->cb)=1;
  if(openSHMWriter(sstr,0)){
    printf("Failed to open SHM to write to %s",sstr->outputname);
    return 1;
  }
  FREQ(sstr->outbuf)=decimate;
  if(sstr->sumsquare){
    if(openSHMWriter(sstr,1)){
    printf("Failed to open SHM to write to %s",sstr->squareoutputname);
    shm_unlink(sstr->outputname);
    return 1;
    }
    FREQ(sstr->out2buf)=decimate;
  }

  if((sstr->data=malloc(sstr->outbuf->datasize))==NULL){
    printf("Unable to malloc data in leakyaverage.c\n");
    shm_unlink(sstr->outputname);
    if(sstr->sumsquare)
      shm_unlink(sstr->squareoutputname);
    return 1;
  }
  if((sstr->store=malloc(((sstr->outbuf->datasize+HSIZE+ALIGN-1)/ALIGN)*ALIGN))==NULL){
    printf("Unable to malloc store in leakyaverage.c\n");
    shm_unlink(sstr->outputname);
    if(sstr->sumsquare)
      shm_unlink(sstr->squareoutputname);
    return 1;
  }
  memset(sstr->store,0,((sstr->outbuf->datasize+HSIZE+ALIGN-1)/ALIGN)*ALIGN);
  if(sstr->sumsquare){
    if((sstr->data2=malloc(sstr->outbuf->datasize))==NULL){
      printf("Unable to malloc data2 in leakyaverage.c\n");
      shm_unlink(sstr->outputname);
      shm_unlink(sstr->squareoutputname);
      return 1;
    }
  }
  sstr->go=1;
  loop(sstr);
  shm_unlink(sstr->outputname);
  if(sstr->sumsquare)
    shm_unlink(sstr->squareoutputname);
  return 0;
}
