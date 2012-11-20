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

#include "circ.h"
typedef struct{
  char *shmprefix;
  char *host;
  int port;
  int debug;
  int log;
  int raw;
  int connect;
  int haveHadReceiver;
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
  int sendNameHdr;
  int readFromHead;
  char *saver;
  int ihdrmsg[8];
  int readpartial;
  int readfrom;
  int readto;
  int readstep;
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
  sock=socket(PF_INET,SOCK_STREAM,0);
  if(sock<0){
    printf("socket error %s\n",strerror(errno));
    return -1;
  }
  // Connect to the server.
  if(initSockaddr(&servername,host,(uint16_t)port)!=0){
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
      sstr->cb=NULL;
      //Check that are still connected to the receiver - if not, exit...
      if(sstr->haveHadReceiver){
	if(sstr->sock!=0){//test the connection
	  //Do this by sending the header only.

	  int nsent,n,err;
	  int *ihdrmsg=sstr->ihdrmsg;
	  char *hdrmsg=(char*)ihdrmsg;
	  ihdrmsg[0]=28;
	  hdrmsg[4]=0x0;
	  hdrmsg[5]=0x0;
	  //hdrmsg[6]=1;//NDIM(sstr->cb);
	  //hdrmsg[7]='n';//DTYPE(sstr->cb);
	  //memcpy(&hdrmsg[8],SHAPEARR(sstr->cb),24);
	  if(sstr->debug)
	    printf("Sending commcheck info for %s\n",sstr->fullname);
	  nsent=0;
	  err=0;
	  //change in shape etc.
	  while(nsent<32 && err==0){
	    if((n=send(sstr->sock,&hdrmsg[nsent],32-nsent,0))<0){
	      printf("sender.c: error writing commcheck info to socket - closing and exiting\n");
	      err=1;
	      close(sstr->sock);
	      sstr->sock=0;
	      sstr->go=0;
	      if(sstr->saver!=NULL){
		printf("saver.close not yet implemented\n");
	      }
	      exit(0);
	    }else{
	      nsent+=n;
	    }
	  }
	}else{//already closed, so exit.
	  printf("Sender exiting - no client, no shm\n");
	  if(sstr->saver!=NULL){
	    printf("saver.close not yet implemented\n");
	  }
	  exit(0);
	}
      }
      sleep(1);
      n++;
      if(n==cnt){
	cnt*=2;
	printf("sender failed to open /dev/shm%s\n",sstr->fullname);
      }
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
  }else if(sstr->sendNameHdr){
    int nsent=0;
    int n;
    int err=0;
    int nb=(int)strlen(&sstr->fullname[1])+1;//include the null.
    printf("Sender sending name\n");
    sstr->fullname[0]=(char)nb;//temporary
    nb++;
    while(nsent<nb && err==0){
      if((n=send(sstr->sock,&sstr->fullname[nsent],nb-nsent,0))==-1){
	printf("Failed to send header in sender\n");
	err=1;
      }else{
	nsent+=n;
      }
    }
    sstr->fullname[0]='/';//replace it.
    if(err){
      close(sstr->sock);
      sstr->sock=0;
      return 1;
    }
  }
  sstr->haveHadReceiver=1;
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
  int *ihdrmsg=sstr->ihdrmsg;
  char *hdrmsg=(char*)ihdrmsg;
  fd_set readfd;
  int copydatasize=0;
  void *copydata=NULL;
  int nel,elsize;
  int readto=0;
  //int checkDecimation;
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
    if(sstr->decimate>0){
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
	  openSHM(sstr);
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
	//checkDecimation=0;
	if(sstr->cumfreq>=sstr->decimate){//#so now send the data
	  sstr->cumfreq=0;
	  //data,timestamp,frameno=ret
	  //print "got data at %s %s %s"%(str(timestamp),str(data.shape),str(data.dtype.char))
	  if((sstr->decimate%cbfreq)==0){//attempt to synchronise frame numbers
	    //so that frame number is a multiple of decimate.
	    sstr->cumfreq=((int*)ret)[1]%sstr->decimate;//(sstr->decimate-((int*)ret)[1]%sstr->decimate)%sstr->decimate;
	  }
	  if(sstr->readFromHead && lw>=0 && cbfreq!=1){
	    ret=circGetFrame(sstr->cb,lw);//get the latest frame.
	  }
	  if(sstr->saver!=NULL && ret!=NULL){
	    if(sstr->raw){
	      printf("saver.writeRaw not yet implemented\n");
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
	      ihdrmsg[0]=28;
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
	      if(sstr->readpartial!=0){
		//check the limits are still valid
		int totsize=1,i;
		for(i=0;i<NDIM(sstr->cb);i++)
		  totsize*=SHAPEARR(sstr->cb)[i];
		if(sstr->readfrom>totsize || sstr->readto>totsize){
		  printf("ERROR - sender read from/to (%d->%d) out of range (data elements %d)\n",sstr->readfrom,sstr->readto,totsize);
		  err=1;
		  sstr->go=0;
		}
		if(sstr->readto==-1)
		  readto=totsize;
		else
		  readto=sstr->readto;

		
	      }
	    }
	    //Now send the data/
	    if(sstr->debug)
	      printf("Sending %s\n",sstr->fullname);
	    if(sstr->raw && ret!=NULL){
	      if(sstr->sock!=0){
		int size,nsent;
		void *dataToSend;
		err=0;
		if(sstr->readpartial==0){//send everything.
		  size=((int*)ret)[0]+4;
		  dataToSend=ret;
		  /*nsent=0;
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
		    }*/
		}else{//only sending a subset of the data.
		  nel=(readto-sstr->readfrom)/sstr->readstep;
		  switch(((char*)ret)[16]){
		  case 'f':
		    elsize=4;
		    break;
		  case 'i':
		    elsize=4;
		    break;
		  case 'h':
		    elsize=2;
		    break;
		  case 'H':
		    elsize=2;
		    break;
		  case 'c':
		    elsize=1;
		    break;
		  case 'd':
		    elsize=8;
		    break;
		  default:
		    printf("Unknown datatype %c in sender.c - recode...\n",((char*)ret)[16]);
		    elsize=4;
		    err=1;
		    sstr->go=0;
		    break;
		  }
		  if(copydatasize<nel*elsize+32){
		    if(copydata!=NULL)
		      free(copydata);
		    copydatasize=nel*elsize+32;
		    if((copydata=malloc(copydatasize))==NULL){
		      printf("Error mallocing copydata in sender - closing socket\n");
		      err=1;
		      close(sstr->sock);
		      sstr->sock=0;
		      sstr->go=0;
		      copydatasize=0;
		    }
		  }
		  //now copy the data (and header)
		  if(copydata!=NULL && err==0){
		    if(sstr->readstep==1){//bulk copy
		      memcpy(&(((char*)copydata)[32]),&(((char*)ret)[32+sstr->readfrom*elsize]),(readto-sstr->readfrom)*elsize);
		      
		    }else{//piecewise copy
		      int i;
		      int pos=32/elsize;
		      switch(((char*)ret)[16]){
		      case 'f':
		      case 'i':
			for(i=sstr->readfrom; i<readto; i+=sstr->readstep)
			  ((int*)copydata)[pos++]=((int*)ret)[32/elsize+i];
			break;
		      case 'h':
		      case 'H':
			for(i=sstr->readfrom; i<readto; i+=sstr->readstep)
			  ((short*)copydata)[pos++]=((short*)ret)[32/elsize+i];
			break;
			for(i=sstr->readfrom; i<readto; i+=sstr->readstep)
			  ((int*)copydata)[pos++]=((int*)ret)[32/elsize+i];
			break;
		      case 'c':
			for(i=sstr->readfrom; i<readto; i+=sstr->readstep)
			  ((char*)copydata)[pos++]=((char*)ret)[32/elsize+i];
			break;
		      case 'd':
			for(i=sstr->readfrom; i<readto; i+=sstr->readstep)
			 ((double*)copydata)[pos++]=((double*)ret)[32/elsize+i];
			break;
		      default:
			printf("Unknown datatype %c in sender.c - recode...\n",((char*)ret)[16]);
			err=1;
			sstr->go=0;
			break;
		      }
		    }
		    memcpy(copydata,ret,32);//copy the header
		    ((int*)copydata)[0]=28+nel*elsize;//update the size
		  }

		  //and send...
		  size=nel*elsize+32;
		  dataToSend=copydata;
		}
		nsent=0;
		while(nsent<size && err==0){
		  int n;
		  if((n=send(sstr->sock,&(((char*)dataToSend)[nsent]),size-nsent,0))<0){
		    printf("Error writing raw data to socket - closing socket: %s\n",strerror(errno));
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
	    //if(err==0){
	    //  checkDecimation=1;
	    // }
	  }
	}
      }
    }
    if(err==0  && sstr->sock!=0 && sstr->go!=0){
      FD_ZERO(&readfd);
      FD_SET(sstr->sock, &readfd);
      selectTimeout.tv_sec=(sstr->decimate==0);
      if((err=select(sstr->sock+1,&readfd,NULL,NULL,&selectTimeout))>0){
	int dec;
	//something to read...
	if(recv(sstr->sock,&dec,sizeof(int),0)!=sizeof(int)){
	  printf("Error reading decimation in sender\n");
	}else{
	  if(sstr->decimate==0 && dec!=0){//we have been woken up...
	    //jump to the head of the circular buffer.
	    //lw=LASTWRITTEN(sstr->cb);//circbuf.lastWritten[0];
	    //if(lw>=0)
	    ret=circGetLatestFrame(sstr->cb);//get the latest frame.
	    sstr->cumfreq=dec;
	    cbfreq=FREQ(sstr->cb);//int(sstr->circbuf.freq[0]);
	    if(cbfreq<1)
	      cbfreq=1;
	    if((dec%cbfreq)==0 && ret!=NULL && dec!=cbfreq){//attempt to synchronise frame numbers so that frame number received is a multiple of decimate.
	      //ie reduce cumfreq by the current frame number mod something
	      sstr->cumfreq-=dec-((int*)ret)[1]%dec;
	    }
	  }
	  sstr->decimate=dec;


	  printf("sender setting decimate to %d\n",dec);
	  err=0;
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
  if(sstr->saver!=NULL){
    printf("saver.close not yet implemented\n");
    //sstr->saver.close();
  }
  if(copydata!=NULL)
    free(copydata);
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
    if(asprintf(&stdoutnames[i],"/dev/shm/%sSenderStdout%d",fullname,i)<0){
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
  sstr->readto=-1;
  sstr->readstep=1;
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
	if(argv[i][2]!='\0'){
	  sstr->sendNameHdr=1;
	}
	break;
      case 'f'://probably a data display - want quick update, not every frame.
	sstr->readFromHead=1;
	break;
      case 'F'://not reading all the data - start From this value
	sstr->readfrom=atoi(&argv[i][2]);
	if(sstr->readfrom>0)
	  sstr->readpartial=1;
	break;
      case 'T'://not reading all the data - read To this value
	sstr->readto=atoi(&argv[i][2]);
	if(sstr->readto>0)
	  sstr->readpartial=1;
	break;
      case 'S'://not reading all the data - Skip by this amount
	sstr->readstep=atoi(&argv[i][2]);
	if(sstr->readstep>1)
	  sstr->readpartial=1;
	break;
      case 'q'://quiet - redirect output...
	redirect=1;
	break;
      default:
	break;
      }
    }else{//the stream name
      sstr->streamname=argv[i];
    }
  }
  if(sstr->readstep==0)
    sstr->readstep=1;
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

  if(redirect){//redirect stdout to a file...
    pthread_t logid;
    char *tmp;
    if(asprintf(&tmp,"%s%s",&sstr->fullname[1],sstr->host==NULL?"NULL":sstr->host)==-1){
      printf("Error asprintfin redirect\n");
    }else if(pthread_create(&logid,NULL,rotateLog,tmp)){
      printf("pthread_create rotateLog failed\n");
    }
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
