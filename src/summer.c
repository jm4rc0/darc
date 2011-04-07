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
  int startWithLatest;
  //int decimate;
  char *streamname; 
  char *fullname;// "/" + shmprefix + streamname.
  int shmOpen;
  circBuf *cb;
  circBuf *outbuf;
  int sock;
  int cumfreq;
  int go;
  int readFromHead;
  int single;
  int rolling;//rolling average, or a collation
  int nsum;
  char dtype;
  char *outputname;
  int nstore;//number of entries in output.
  int sumcnt;
  void *dataHist;
  int dataHistTail;
  int dataHistHead;
  void *data;
  int dtypeAsData;
}SendStruct;

int openSHMReader(SendStruct *sstr){
  int cnt=1,n=0;
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
  }
  return 0;
}
char *shmname=NULL;//global required for signal handler.

int openSHMWriter(SendStruct *sstr){
  struct stat st;
  char *tmp;
  if(asprintf(&tmp,"/dev/shm%s",sstr->outputname)==-1){
    printf("asprintf failed - unable to check for existance of %s so eting\n",sstr->outputname);
    return 1;
  }
  if(stat(tmp,&st)==0){//file exists - don't overwritem, but exit in error...
    printf("File in shm %s already exists - summer exiting\n",tmp);
    free(tmp);
    tmp=NULL;
    return 1;
  }
  if((sstr->outbuf=openCircBuf(sstr->outputname,(int)(NDIM(sstr->cb)),SHAPEARR(sstr->cb),sstr->dtype,sstr->nstore))==NULL){
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
    printf("Summer interrupt received (%d) - exiting\n",sig);
  exit(1);
}
int sumData(SendStruct *sstr,char *ret){
  int n=1,i;
  char dtype;
  void *arr;
  for(i=0; i<NDIM(sstr->cb); i++){
    n*=SHAPEARR(sstr->cb)[i];
  }
  dtype=ret[16];
  ret=&ret[32];//skip the 32 byte header...
  if(sstr->sumcnt==sstr->nsum){
    if(sstr->rolling==1){
      //subtract dataHist[dataHistTail]
      if(sstr->dtype=='f'){
	arr=(void*)&(((float*)sstr->dataHist)[sstr->dataHistTail*n]);
	for(i=0; i<n; i++){
	  ((float*)sstr->data)[i]-=((float*)arr)[i];
	}
      }else if(sstr->dtype=='d'){
	arr=(void*)&(((double*)sstr->dataHist)[sstr->dataHistTail*n]);
	for(i=0; i<n; i++){
	  ((double*)sstr->data)[i]-=((double*)arr)[i];
	}
      }else if(sstr->dtype=='i'){
	arr=(void*)&(((int*)sstr->dataHist)[sstr->dataHistTail*n]);
	for(i=0; i<n; i++){
	  ((int*)sstr->data)[i]-=((int*)arr)[i];
	}
      }else if(sstr->dtype=='h'){
	arr=(void*)&(((short*)sstr->dataHist)[sstr->dataHistTail*n]);
	for(i=0; i<n; i++){
	  ((short*)sstr->data)[i]-=((short*)arr)[i];
	}
      }else if(sstr->dtype=='H'){
	arr=(void*)&(((unsigned short*)sstr->dataHist)[sstr->dataHistTail*n]);
	for(i=0; i<n; i++){
	  ((unsigned short*)sstr->data)[i]-=((unsigned short*)arr)[i];
	}
      }else{
	printf("Unknown datatype %c in summer.c\n",sstr->dtype);
      }
      sstr->dataHistTail++;
      if(sstr->dataHistTail==sstr->nsum)
	sstr->dataHistTail=0;


      sstr->sumcnt--;
    }else{
      //reset
      sstr->sumcnt=0;
      memset(sstr->data,0,sstr->outbuf->datasize);
    }

  }
  if(sstr->rolling==1){
    //now write the new data to head.
    if(sstr->dtype=='f'){
      arr=(void*)&(((float*)sstr->dataHist)[sstr->dataHistHead*n]);
      if(dtype=='f'){
	for(i=0; i<n; i++){
	  ((float*)arr)[i]=((float*)ret)[i];
	  ((float*)sstr->data)[i]+=((float*)arr)[i];
	}
      }else if(dtype=='d'){
	for(i=0; i<n; i++){
	  ((float*)arr)[i]=((double*)ret)[i];
	  ((float*)sstr->data)[i]+=((float*)arr)[i];
	}
      }else if(dtype=='i'){
	for(i=0; i<n; i++){
	  ((float*)arr)[i]=((int*)ret)[i];
	  ((float*)sstr->data)[i]+=((float*)arr)[i];
	}
      }else if(dtype=='h'){
	for(i=0; i<n; i++){
	  ((float*)arr)[i]=((short*)ret)[i];
	  ((float*)sstr->data)[i]+=((float*)arr)[i];
	}
      }else if(dtype=='H'){
	for(i=0; i<n; i++){
	  ((float*)arr)[i]=((unsigned short*)ret)[i];
	  ((float*)sstr->data)[i]+=((float*)arr)[i];
	}
      }else{
	printf("notcoded for type %c in summer.c\n",dtype);
      }
    }else if(sstr->dtype=='i'){
      arr=(void*)&(((int*)sstr->dataHist)[sstr->dataHistHead*n]);
      if(dtype=='f'){
	for(i=0; i<n; i++){
	  ((int*)arr)[i]=((float*)ret)[i];
	  ((int*)sstr->data)[i]+=((int*)arr)[i];
	}
      }else if(dtype=='d'){
	for(i=0; i<n; i++){
	  ((int*)arr)[i]=((double*)ret)[i];
	  ((int*)sstr->data)[i]+=((int*)arr)[i];
	}
      }else if(dtype=='i'){
	for(i=0; i<n; i++){
	  ((int*)arr)[i]=((int*)ret)[i];
	  ((int*)sstr->data)[i]+=((int*)arr)[i];
	}
      }else if(dtype=='h'){
	for(i=0; i<n; i++){
	  ((int*)arr)[i]=((short*)ret)[i];
	  ((int*)sstr->data)[i]+=((int*)arr)[i];
	}
      }else if(dtype=='H'){
	for(i=0; i<n; i++){
	  ((int*)arr)[i]=((unsigned short*)ret)[i];
	  ((int*)sstr->data)[i]+=((int*)arr)[i];
	}
      }else{
	printf("notcoded for type %c in summer.c\n",dtype);
      }
    }else if(sstr->dtype=='h'){
      arr=(void*)&(((short*)sstr->dataHist)[sstr->dataHistHead*n]);
      if(dtype=='f'){
	for(i=0; i<n; i++){
	  ((short*)arr)[i]=((float*)ret)[i];
	  ((short*)sstr->data)[i]+=((short*)arr)[i];
	}
      }else if(dtype=='d'){
	for(i=0; i<n; i++){
	  ((short*)arr)[i]=((double*)ret)[i];
	  ((short*)sstr->data)[i]+=((short*)arr)[i];
	}
      }else if(dtype=='i'){
	for(i=0; i<n; i++){
	  ((short*)arr)[i]=((int*)ret)[i];
	  ((short*)sstr->data)[i]+=((short*)arr)[i];
	}
      }else if(dtype=='h'){
	for(i=0; i<n; i++){
	  ((short*)arr)[i]=((short*)ret)[i];
	  ((short*)sstr->data)[i]+=((short*)arr)[i];
	}
      }else if(dtype=='H'){
	for(i=0; i<n; i++){
	  ((short*)arr)[i]=((unsigned short*)ret)[i];
	  ((short*)sstr->data)[i]+=((short*)arr)[i];
	}
      }else{
	printf("notcoded for type %c in summer.c\n",dtype);
      }
    }else if(sstr->dtype=='H'){
      arr=(void*)&(((unsigned short*)sstr->dataHist)[sstr->dataHistHead*n]);
      if(dtype=='f'){
	for(i=0; i<n; i++){
	  ((unsigned short*)arr)[i]=((float*)ret)[i];
	  ((unsigned short*)sstr->data)[i]+=((unsigned short*)arr)[i];
	}
      }else if(dtype=='d'){
	for(i=0; i<n; i++){
	  ((unsigned short*)arr)[i]=((double*)ret)[i];
	  ((unsigned short*)sstr->data)[i]+=((unsigned short*)arr)[i];
	}
      }else if(dtype=='i'){
	for(i=0; i<n; i++){
	  ((unsigned short*)arr)[i]=((int*)ret)[i];
	  ((unsigned short*)sstr->data)[i]+=((unsigned short*)arr)[i];
	}
      }else if(dtype=='h'){
	for(i=0; i<n; i++){
	  ((unsigned short*)arr)[i]=((short*)ret)[i];
	  ((unsigned short*)sstr->data)[i]+=((unsigned short*)arr)[i];
	}
      }else if(dtype=='H'){
	for(i=0; i<n; i++){
	  ((unsigned short*)arr)[i]=((unsigned short*)ret)[i];
	  ((unsigned short*)sstr->data)[i]+=((unsigned short*)arr)[i];
	}
      }else{
	printf("notcoded for type %c in summer.c\n",dtype);
      }
    }else if(sstr->dtype=='d'){
      arr=(void*)&(((double*)sstr->dataHist)[sstr->dataHistHead*n]);
      if(dtype=='f'){
	for(i=0; i<n; i++){
	  ((double*)arr)[i]=((float*)ret)[i];
	  ((double*)sstr->data)[i]+=((double*)arr)[i];
	}
      }else if(dtype=='d'){
	for(i=0; i<n; i++){
	  ((double*)arr)[i]=((double*)ret)[i];
	  ((double*)sstr->data)[i]+=((double*)arr)[i];
	}
      }else if(dtype=='i'){
	for(i=0; i<n; i++){
	  ((double*)arr)[i]=((int*)ret)[i];
	  ((double*)sstr->data)[i]+=((double*)arr)[i];
	}
      }else if(dtype=='h'){
	for(i=0; i<n; i++){
	  ((double*)arr)[i]=((short*)ret)[i];
	  ((double*)sstr->data)[i]+=((double*)arr)[i];
	}
      }else if(dtype=='H'){
	for(i=0; i<n; i++){
	  ((double*)arr)[i]=((unsigned short*)ret)[i];
	  ((double*)sstr->data)[i]+=((double*)arr)[i];
	}
      }else{
	printf("notcoded for type %c in summer.c\n",dtype);
      }
    }else{
      printf("summer dtype %c not yet coded\n",sstr->dtype);
    }
    sstr->dataHistHead++;
    if(sstr->dataHistHead==sstr->nsum)
      sstr->dataHistHead=0;
  }else{
    if(sstr->dtype=='f'){
      if(dtype=='f'){
	for(i=0;i<n;i++)
	  ((float*)sstr->data)[i]+=(float)((float*)ret)[i];
      }else if(dtype=='d'){
	for(i=0;i<n;i++)
	  ((float*)sstr->data)[i]+=(float)((double*)ret)[i];
      }else if(dtype=='i'){
	for(i=0;i<n;i++)
	  ((float*)sstr->data)[i]+=(float)((int*)ret)[i];
      }else if(dtype=='h'){
	for(i=0;i<n;i++)
	  ((float*)sstr->data)[i]+=(float)((short*)ret)[i];
      }else if(dtype=='H'){
	for(i=0;i<n;i++)
	  (((float*)sstr->data)[i])+=(float)(((unsigned short*)ret)[i]);
      }else{
	printf("Not coded for type %c in summer.c\n",dtype);
      }
    }else if(sstr->dtype=='i'){
      if(dtype=='f'){
	for(i=0;i<n;i++)
	  ((int*)sstr->data)[i]+=((float*)ret)[i];
      }else if(dtype=='d'){
	for(i=0;i<n;i++)
	  ((int*)sstr->data)[i]+=((double*)ret)[i];
      }else if(dtype=='i'){
	for(i=0;i<n;i++)
	  ((int*)sstr->data)[i]+=((int*)ret)[i];
      }else if(dtype=='h'){
	for(i=0;i<n;i++)
	  ((int*)sstr->data)[i]+=((short*)ret)[i];
      }else if(dtype=='H'){
	for(i=0;i<n;i++)
	  ((int*)sstr->data)[i]+=((unsigned short*)ret)[i];
      }else{
	printf("Not coded for type %c in summer.c\n",dtype);
      }
    }else if(sstr->dtype=='h'){
      if(dtype=='f'){
	for(i=0;i<n;i++)
	  ((short*)sstr->data)[i]+=((float*)ret)[i];
      }else if(dtype=='d'){
	for(i=0;i<n;i++)
	  ((short*)sstr->data)[i]+=((double*)ret)[i];
      }else if(dtype=='i'){
	for(i=0;i<n;i++)
	  ((short*)sstr->data)[i]+=((int*)ret)[i];
      }else if(dtype=='h'){
	for(i=0;i<n;i++)
	  ((short*)sstr->data)[i]+=((short*)ret)[i];
      }else if(dtype=='H'){
	for(i=0;i<n;i++)
	  ((short*)sstr->data)[i]+=((unsigned short*)ret)[i];
      }else{
	printf("Not coded for type %c in summer.c\n",dtype);
      }
    }else if(sstr->dtype=='H'){
      if(dtype=='f'){
	for(i=0;i<n;i++)
	  ((unsigned short*)sstr->data)[i]+=((float*)ret)[i];
      }else if(dtype=='d'){
	for(i=0;i<n;i++)
	  ((unsigned short*)sstr->data)[i]+=((double*)ret)[i];
      }else if(dtype=='i'){
	for(i=0;i<n;i++)
	  ((unsigned short*)sstr->data)[i]+=((int*)ret)[i];
      }else if(dtype=='h'){
	for(i=0;i<n;i++)
	  ((unsigned short*)sstr->data)[i]+=((short*)ret)[i];
      }else if(dtype=='H'){
	for(i=0;i<n;i++)
	  ((unsigned short*)sstr->data)[i]+=((unsigned short*)ret)[i];
      }else{
	printf("Not coded for type %c in summer.c\n",dtype);
      }
    }else if(sstr->dtype=='d'){
      if(dtype=='f'){
	for(i=0;i<n;i++)
	  ((double*)sstr->data)[i]+=(double)((float*)ret)[i];
      }else if(dtype=='d'){
	for(i=0;i<n;i++)
	  ((double*)sstr->data)[i]+=(double)((double*)ret)[i];
      }else if(dtype=='i'){
	for(i=0;i<n;i++)
	  ((double*)sstr->data)[i]+=(double)((int*)ret)[i];
      }else if(dtype=='h'){
	for(i=0;i<n;i++)
	  ((double*)sstr->data)[i]+=(double)((short*)ret)[i];
      }else if(dtype=='H'){
	for(i=0;i<n;i++)
	  (((double*)sstr->data)[i])+=(double)(((unsigned short*)ret)[i]);
      }else{
	printf("Not coded for type %c in summer.c\n",dtype);
      }
    }else{
      printf("Not coded in summer.c for type %c\n",sstr->dtype);
    }
  }
  sstr->sumcnt++;
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
  struct timeval selectTimeout;
  int ihdrmsg[8];
  char *hdrmsg=(char*)ihdrmsg;
  int dsize;
  int sleeping=0;
  //int checkDecimation;
  selectTimeout.tv_sec=0;
  selectTimeout.tv_usec=0;
  memset(hdrmsg,0,sizeof(hdrmsg));
  circHeaderUpdated(sstr->cb);
  if(sstr->startWithLatest){
    ret=circGetLatestFrame(sstr->cb);
    //printf("Last received %d last written %d\n",sstr->cb->lastReceived,LASTWRITTEN(sstr->cb));
  }
  while(sstr->go && err==0){
    if(FREQ(sstr->outbuf)>0){
      if(sleeping){//skip to head.
	sleeping=0;
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
	  if((FREQ(sstr->outbuf)%cbfreq)==0){//attempt to synchronise frame numbers
	    //so that frame number is a multiple of decimate.
	    sstr->cumfreq=((int*)ret)[1]%FREQ(sstr->outbuf);//(sstr->decimate-((int*)ret)[1]%sstr->decimate)%sstr->decimate;
	  }
	  if(sstr->readFromHead && lw>=0){
	    ret=circGetFrame(sstr->cb,lw);//get the latest frame.
	  }
	  //sum the data
	  //Check here - has the data type or shape changed?  If so, reset the average counters...
	  if((NDIM(sstr->cb)!=hdrmsg[6]) || (DTYPE(sstr->cb)!=hdrmsg[7]) || strncmp(&hdrmsg[8],(char*)SHAPEARR(sstr->cb),24)!=0){
	    sstr->sumcnt=sstr->nsum;
	    ihdrmsg[0]=28;
	    hdrmsg[4]=0x55;
	    hdrmsg[5]=0x55;
	    hdrmsg[6]=NDIM(sstr->cb);
	    hdrmsg[7]=DTYPE(sstr->cb);
	    memcpy(&hdrmsg[8],SHAPEARR(sstr->cb),24);
	    if(sstr->debug)
	      printf("Rsetting countr for %s\n",sstr->fullname);
	    dsize=sstr->outbuf->datasize;
	    if(sstr->dtypeAsData){//change the dtype.
	      sstr->dtype=DTYPE(sstr->cb);
	      circReshape(sstr->outbuf,(int)NDIM(sstr->cb),SHAPEARR(sstr->cb),sstr->dtype);
	      if(dsize!=sstr->outbuf->datasize){
		if(sstr->data!=NULL)free(sstr->data);
		if((sstr->data=malloc(sstr->outbuf->datasize))==NULL){
		  printf("Unable to re-malloc data in summer.c\n");
		  return 1;
		}
	      }
	      if(sstr->rolling==1){//doing a rolling average, so malloc the space
		if(dsize!=sstr->outbuf->datasize){
		  if(sstr->dataHist!=NULL)
		    free(sstr->dataHist);
		  if((sstr->dataHist=calloc(sstr->nsum,sstr->outbuf->datasize))==NULL){
		    printf("Error mallocing dataHist in summer.c\n");
		    return 1;
		  }
		}
	      }
	    }else{
	      circReshape(sstr->outbuf,(int)NDIM(sstr->cb),SHAPEARR(sstr->cb),sstr->dtype);
	    }
	  }
	  //Now sum the data
	  if(sstr->debug)
	    printf("Summing %s\n",sstr->fullname);
	  //printf("Summing frame %d, pointer %p %c %c\n",sstr->cb->lastReceived,ret,((char*)ret)[16],sstr->dtype);
	  sumData(sstr,ret);
	  if(sstr->sumcnt==sstr->nsum){
	    if(sstr->debug)
	      printf("Writing summed data to shm\n");
	    FORCEWRITE(sstr->outbuf)++;
	    circAdd(sstr->outbuf,sstr->data,((double*)ret)[1],((int*)ret)[1]);
	    if(sstr->single)
	      FREQ(sstr->outbuf)=0;
	      
	  }
	}
      }
    }else{//turned off decimation... so just wait... can then either be turned back on, or can be killed...
      sstr->sumcnt=sstr->nsum;//so that if we restart we start summing fresh data.
      sleeping=1;
      sleep(1);
    }
  }
  return 0;
}


int main(int argc, char **argv){
  int setprio=0;
  int affin=0x7fffffff;
  int prio=0;
  SendStruct *sstr;
  int i,decimate=1;
  struct sigaction sigact;
  if((sstr=malloc(sizeof(SendStruct)))==NULL){
    printf("Unable to malloc SendStruct\n");
    return 1;
  }
  memset(sstr,0,sizeof(SendStruct));
  sstr->go=1;
  sstr->nsum=10;
  sstr->dtype='n';
  sstr->nstore=10;
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
      case '1':
	sstr->single=1;
	break;
      case 'r':
	sstr->rolling=1;
	break;
      case 'n':
	sstr->nsum=atoi(&argv[i][2]);
	break;
      case 't'://if not specified, will be same as the data (which may lead to overflow and wrapping for int, especially short, types)
	sstr->dtype=argv[i][2];
	break;
      case 'o':
	sstr->outputname=&argv[i][2];
	break;
      case 'S':
	sstr->nstore=atoi(&argv[i][2]);
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
  if(sstr->outputname==NULL){
    if(asprintf(&sstr->outputname,"/%s%s%d%c%sBuf",sstr->fullname,sstr->rolling?"Rolling":"Summed",sstr->nsum,sstr->dtype,sstr->readFromHead?"Head":"Tail")==-1){
      printf("Error asprintf2\n");
      return 1;
    }
  }
  printf("Summer starting for %s\n",sstr->outputname);
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

  openSHMReader(sstr);
  if(sstr->dtype=='n'){//copy type from input.
    sstr->dtype=DTYPE(sstr->cb);
    sstr->dtypeAsData=1;
  }
  if(openSHMWriter(sstr)){
    printf("Failed to open SHM to write to %s",sstr->outputname);
    return 1;
  }
  FREQ(sstr->outbuf)=decimate;

  if(sstr->rolling==1){//doing a rolling average, so malloc the space
    if((sstr->dataHist=calloc(sstr->nsum,sstr->outbuf->datasize))==NULL){
      printf("Error mallocing dataHist in summer.c\n");
      shm_unlink(sstr->outputname);
      return 1;
    }
  }
  if((sstr->data=malloc(sstr->outbuf->datasize))==NULL){
    printf("Unable to malloc data in summer.c\n");
    shm_unlink(sstr->outputname);
    return 1;
  }
  sstr->go=1;
  sstr->cumfreq=decimate;
  loop(sstr);
  shm_unlink(sstr->outputname);
  return 0;
}
