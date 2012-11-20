/*TODO
sendAsHead should send frames according to required decimation (currently, it will just send the head frame - which may not be required).


 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <arpa/inet.h>//for sockaddr_in
#include <netdb.h>//gethostbyname
#include <errno.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <dirent.h>
#include <sys/inotify.h>
#include "circ.h"

//#define NSTREAMS 65536
//#define NCLIENTS 256 //max of 256 since class C network assumed, and only 1 client per host computer.
//#define MAXSTREAMNAMELEN 80
/*  //To get MTU use:
  struct	ifreq		ifr;
  if (ioctl(s, SIOCGIFMTU, (caddr_t)&ifr) < 0)
    warn("ioctl (SIOCGIFMTU)");
  else
    mtu = ifr.ifr_mtu;
*/
/***********
What needs to be in shared memory (so that state is preserved on restart)?
The table of stream name and index.
Decimation requests for each client and each stream.
Also, the fraction of a buffer which they can afford to fall behind.  This is a value from 0 to 0.75 (greater values truncated at 0.75).  Then, the largest of these values is used...
This allows most of the time, displays etc with a value of 0 to get the most recent data, but possibly at the expense of not getting them as often as they wish.  But then clients that require large blocks of data can get these by specifying a larger value... the tail may fall behind the head in this case.
Why do this?  I think it simplifies things a lot... there is then no need to send packets in advance of other frames.  It does have the side effect that displays may lag at times.  

Another option might be to have duplicate streams, one for low latency, one for contiguous streams.  But that would mean sending some frames twice, and is more complicated to ensure you subscribe to the correct one.  Actually, this might be a better solution...  And you could still avoid sending twice, if the receivers knew to copy data from the low latency stream to the contiguous stream.  

And finally - a better solution is to have in each shm file, a circular buffer for contiguous streams (say 100 long), and a circular buffer for low latency (default 10% of the other, min 3).

The dds receiver can then put inorder frames into the contiguous buffer, and header streams into the low latency buffer.

Then, circGetNext() does the obvious, while circGetLatest returns the latest.
Yes, I think that is the way to do it.


Threads:
The main DDSing threads which send out the data.
Another thread checking that the clients are still alive.
Another thread checking decimation rates.
Another thread that responds to requests for resends (and re-broadcasts the data).  Could this be the one checking if clients are still alive?  Could 1 thread do all this? i.e. have a connection to the clients, listen for resend or decimation requests, and send a keep-alive occassionally.  Yes, I think that is best.  
But - how to store state in shm?  Using the IP address as the key, and then decimate requests as the data.
If the server is restarted, clients attempt to connect, and after a certain gracetime, if they haven't they are removed.    

***********/

/*
The layout of the shared memory is like this:
typedef struct{//This is placed in SHM - contains all the state required to start a new dds.
  int bufsize;//constant
  int go;
  int nstreams;//constant
  int nclients;//constant
  int maxstreamnamelen;//constant
  int mutexInit;//0 or 1.
  int mutexSize;//constant
  //mutex memory may need to be 64 byte aligned?
  pthread_mutex_t sendMutex;//currently - this could be not in shm - but I suppose there may always be the circumstance where something else will want to broadcast or use the dds network...
  char streamIndex[NSTREAMS][MAXSTREAMNAMELEN];
  int decHead[NSTREAMS][NCLIENTS];//The first of these is used to force the decimation. (i.e. decHead[:][0])
  int decTail[NSTREAMS][NCLIENTS];//The first of these is used to force the decimation.
  float decFreqTime[NSTREAMS][NCLIENTS];//The first of these is used to force the decimation.
}ddsStruct;
*/
typedef struct{
  int nclients;
  int nstreams;
  int packetsize;
  int resendThroughSocket;
  int maxStreamNameLen;
  int bcastsock;
  struct sockaddr_in bcastaddr;
  int lsock;
  int *clientsock;//the socket fd for clients, one per client
  int lport;
  char *lhost;
  int *grace;//grace time before removing decimation for this client.
  char **dataToSend;//one per client
  int *dataToSendSize;//one per client
  int *dataToSendMemSize;//one per client
  void **streamStructs;//one per client
  pthread_t *streamThreadId;//one per client.
  char *streamIndex;//these are pointers to the shm data.
  int *decHead;//shm ptr
  int *decTail;//shm ptr
  float *decFreqTime;//shm ptr
  char *dds;//shm ptr
  int ddsSize;
  char *bcastHost;
  int bcastPort;
  float ftimeout;
  float gracetimeout;
  int inotify;
  int inotifywd;
  pthread_mutex_t sendMutex;
  pthread_mutex_t streamNameMutex;
  pthread_mutex_t timeoutmutex;
  pthread_cond_t timeoutcond;

}globalStruct;

typedef struct{
  globalStruct *glob;
  int index;
  circBuf *cb;
  int decHead;
  int decTail;
  float decFreqTime;
  unsigned int lastReceivedFrameTail;
  unsigned int lastReceivedFrameHead;
  double lastHeadFrametimeSent;
  int lastReceivedTail;//index into circ buf that was last used by us.
  int *sendAsHead;
  int sendAsHeadSize;
  int lastSendHead;
  char *fullname;// /somethingrtcSomthingBuf
  char *devshmfullname;// /dev/shm/somethingrtcSomthingBuf
}streamStruct;

#define DDSSIZE(buf) (*((int*)buf))
#define DDSGO(buf) (((int*)buf)[1])
#define DDSNSTREAMS(buf) (((int*)buf)[2])
#define DDSNCLIENTS(buf) (((int*)buf)[3])
#define DDSMAXNAMELEN(buf) (((int*)buf)[4])
#define DDSMUTEXINIT(buf) (((int*)buf)[5])
#define DDSMUTEXSIZE(buf) (((int*)buf)[6])
#define DDSMUTEX(buf) ((pthread_mutex_t*)&((int*)buf)[7])
#define DDSSTREAMINDEX(buf) (((char*)(&((int*)buf)[7]))+DDSMUTEXSIZE(buf))
#define DDSDECHEAD(buf) ((int*)(DDSSTREAMINDEX(buf)+DDSNSTREAMS(buf)*DDSMAXNAMELEN(buf)))
#define DDSDECTAIL(buf) (DDSDECHEAD(buf)+DDSNSTREAMS(buf)*DDSNCLIENTS(buf))
#define DDSDECFREQTIME(buf) ((float*)(DDSDECTAIL(buf)+DDSNSTREAMS(buf)*DDSNCLIENTS(buf)))


int calcDDSSize(nstreams,maxStreamNameLen,nclients){
  int size;
  size=(1+1+1+1+1+nstreams*nclients+nstreams*nclients)*sizeof(int);
  size+=(nstreams*nclients)*sizeof(float);
  size+=nstreams*maxStreamNameLen;
  size+=sizeof(pthread_mutex_t);
  return size;
}

int getNewStreamIndex(globalStruct *glob,char *name){
  int i=0;
  int len=strlen(name);
  //check that the stream is a darc stream...
  //Should end with buf, and has rtc in it.
  if(len>79)
    return -1;
  if(len<7)
    return -1;
  if(strcmp("Buf",&name[len-3])!=0)//ends with "Buf"?
    return -1;
  if(strstr(name,"rtc")==NULL)//contains "rtc"?
     return -1;
  for(i=0;i<glob->nstreams;i++){
    if(glob->streamIndex[i*glob->maxStreamNameLen]!='\0' && strcmp(&glob->streamIndex[i*glob->maxStreamNameLen],name)==0){
      printf("Stream %s already exists in dds index - not starting new thread\n",name);
      return -1;
    }
  }
  pthread_mutex_lock(&glob->streamNameMutex);
  i=0;
  while(i<glob->nstreams){
    if(glob->streamIndex[i*glob->maxStreamNameLen]==0){
      memcpy(&glob->streamIndex[i*glob->maxStreamNameLen],name,len);
      glob->streamIndex[i*glob->maxStreamNameLen+len]='\0';
      break;
    }
    i++;
  }
  pthread_mutex_unlock(&glob->streamNameMutex);
  if(i==glob->nstreams){
    printf("No space left for new streams (%d found) - serious recompiling required!\n",glob->nstreams);
    i=-1;
  }
  return i;
}
int getIndexOfStream(globalStruct *glob,char *stream){
  int i;
  for(i=0;i<glob->nstreams;i++){
    if(strncmp(stream,&glob->streamIndex[i*glob->maxStreamNameLen],glob->maxStreamNameLen)==0)
      return i;
  }
  return -1;
}

int checkSendTail(streamStruct *s,int lw){
  //returns 1 if we should send, 0 if not yet.
  //need a check that this is always legal - ie if the buffer has changed shape, and now lw>NSTORE or something.
  //Note - we won't necessarily send lw - this is the newest frame, and it is possible we want to send an older one.
  unsigned int f;
  f=CIRCFRAMENO(s->cb,lw);
  if(s->decTail>0 && (f-s->lastReceivedFrameTail>=s->decTail))
    return 1;
  return 0;
}
int checkSendHead(streamStruct *s,int lw){
  //returns 1 if we should send, 0 if not yet.
  double curtime;
  struct timeval t;
  unsigned int f;
  int rt=0;
  gettimeofday(&t,NULL);
  curtime=t.tv_sec+1e-6*t.tv_usec;
  f=CIRCFRAMENO(s->cb,lw);
  if(s->decHead>0 && f-s->lastReceivedFrameHead>=s->decHead)
    rt=1;
  else if(s->decFreqTime>0. && curtime-s->lastHeadFrametimeSent>=s->decFreqTime)
    rt=2;;
  return rt;
}

int broadcast(streamStruct *s,int indx){
  //broadcasts indx in the circular buffer.  If indx==-1, retrieves the latest entry (after having obtained the lock).
  globalStruct *glob=s->glob;
  unsigned int hdr[4],fno,sent;
  int hdrsize=sizeof(unsigned int)*4;
  int packetsPerFrame;
  int packetno;
  int dsize,nsent,ashead=0,size,i,lw,cnt,ns,thisframe,latest;
  char *data;
  pthread_mutex_lock(&glob->sendMutex);
  if(indx==-1){
    ashead=1;
    indx=LASTWRITTEN(s->cb);
    if(indx==-1){
      pthread_mutex_unlock(&glob->sendMutex);
      return -1;
    }
    printf("broadcasting %s from head, frame %d (%d)\n",&glob->streamIndex[s->index*glob->maxStreamNameLen],indx,CIRCFRAMENO(s->cb,indx));
  }else if(indx<0){//chose a most recent frame that is multiple of -indx.
    ashead=1;
    i=lw=LASTWRITTEN(s->cb);
    if(i==-1){
      pthread_mutex_unlock(&glob->sendMutex);
      return -1;
    }
    latest=CIRCFRAMENO(s->cb,i);
    thisframe=latest;
    ns=NSTORE(s->cb);
    cnt=0;
    while(thisframe%(-indx)!=0 && latest-thisframe<(-indx) && cnt<ns && thisframe>s->lastReceivedFrameHead && thisframe>s->lastReceivedFrameTail){
      i--;
      cnt++;
      if(i<0)//wrap back round the buffer
	i=ns-1;
      thisframe=CIRCFRAMENO(s->cb,i);
    }
    if(thisframe%(-indx)==0 && latest-thisframe<(-indx) && thisframe>s->lastReceivedFrameHead && thisframe>s->lastReceivedFrameTail){
      indx=i;
    }else{//might get here eg if the decimations aren't compatible.
      indx=lw;
    }
    printf("broadcasting %s from head, frame %d (%d, head at %d)\n",&glob->streamIndex[s->index*glob->maxStreamNameLen],indx,CIRCFRAMENO(s->cb,indx),lw);
  }
  //can we be sure that this hasn't already been sent?


  //check whether data has been resized - if so, broadcast this info first?
  //actually - we'll send dtype and size with every frame.  (I'm going to ignore shape now).
  //What do we do if the circular buffer gets reshaped in the middle of this function?  Could it cause a crash?
  data=circGetFrame(s->cb,indx);//get the frame.

  //Packets are now sent.
  //packet contains:  1 bit as head, 1 bit resend, 14 bits frame number (lowest 14), 16 bits index, 32 bits packet offset in circular buffer, counter(32), packet size(32), then the data.
  fno=CIRCFRAMENO(s->cb,indx);
  //totdatabytes=s->cb->frameSize*NSTORE(s->cb);//=BUFSIZE(s->cb)-circCalcHdrSize();
  dsize=CIRCDATASIZE(s->cb,indx);
  packetsPerFrame=((s->cb->frameSize+glob->packetsize-hdrsize-1)/(glob->packetsize-hdrsize));
  packetno=packetsPerFrame*indx;
  hdr[0]=(ashead<<31) | ((fno&0x3fff)<<16) | (s->index&0xffff);
  hdr[1]=packetno;
  hdr[2]=0;//start of data - incremented for each packet in this set of data.
  hdr[3]=glob->packetsize;
  sent=0;
  while(sent<dsize){
    if((nsent=sendto(glob->bcastsock,hdr,hdrsize,MSG_MORE,(struct sockaddr*)&glob->bcastaddr,sizeof(glob->bcastaddr)))!=hdrsize){
      printf("Error sending header (%d/%d sent): %s\n",nsent,hdrsize,strerror(errno));
    }
    size=glob->packetsize-hdrsize;
    if(size>dsize-sent){
      size=dsize-sent;
      hdr[3]=size+hdrsize;//new packet size for this one.
    }
    if((nsent=sendto(glob->bcastsock,&data[sent],size,0,(struct sockaddr*)&glob->bcastaddr,sizeof(glob->bcastaddr)))!=size){
      printf("Error sending data (%d/%d sent): %s\n",nsent,size,strerror(errno));
    }
    sent+=size;
    hdr[1]++;
    hdr[2]++;
  }
  
  pthread_mutex_unlock(&glob->sendMutex);
  return indx;
}



int broadcastDataFromTail(streamStruct *s,int lw){
  struct timeval t;
  int lr=s->lastReceivedTail;//this is the last circbuf entry that we have used (but may not be at the head of the circular buffer).
  int lf=s->lastReceivedFrameTail;//this is last frameno used by us.
  int ns=NSTORE(s->cb);
  int desiredFrame=lf+s->decTail;
  int lww;
  int *tmp;
  int diff;
  globalStruct *glob=s->glob;
  if(lw>=0){
    diff=lw-lr;
    if(diff<0){
      diff+=ns;
    }
    //printf("diff %d %d %d\n",diff,lw,sstr->cb->lastReceived);
    if(diff>ns*0.75){//ircbuf.nstore[0]*.75){
      printf("Sending of %s lagging - skipping %d frames\n",&glob->streamIndex[s->index*glob->maxStreamNameLen],diff-1);
      //ret=circGetFrame(s->cb,lw);//sstr->circbuf.get(lw,copy=1);
      //skip to most recent frame.
      desiredFrame=CIRCFRAMENO(s->cb,lw);
      memset(s->sendAsHead,0,sizeof(int)*s->sendAsHeadSize);
    }
  }
  
  desiredFrame-=desiredFrame%s->decTail;//so it becomes a multiple of decimation
  //So, need to find the first buffer entry for which the frame number is >= desiredFrame.
  if(lw<lr)
    lww=lw+ns;
  else
    lww=lw;
  lr++;
  if(s->sendAsHeadSize<ns){
    if((tmp=realloc(s->sendAsHead,ns*sizeof(int)))==NULL){
      printf("Unable to realloc memory for sendAsHead - probably time to crash!\n");
      return 1;
    }else{
      s->sendAsHead=tmp;
      //set the rest to zero.
      memset(&tmp[s->sendAsHeadSize],0,sizeof(int)*(ns-s->sendAsHeadSize));
      s->sendAsHeadSize=ns;
    }
  }
  while(lr<=lww){
    if(CIRCFRAMENO(s->cb,lr%ns)>=desiredFrame){
      //this is the entry we want.
      //lr=lr%ns;
      if(s->sendAsHead[lr%ns]){//already sent - try the next one...
	desiredFrame+=s->decTail;
	s->lastReceivedTail=lr%ns;
	s->lastReceivedFrameTail=CIRCFRAMENO(s->cb,lr%ns);
	s->sendAsHead[lr%ns]=0;
      }else
	break;
    }
    s->sendAsHead[lr%ns]=0;//clear the head flag.
    lr++;
  }
  if(lr<=lww){//we've found a frame to send.
    lr=lr%ns;
    s->lastReceivedTail=lr;
    s->lastReceivedFrameTail=CIRCFRAMENO(s->cb,lr);
    if(s->sendAsHead[lr]){//actually, shouldn't get here...
      printf("Frame %u already sent as head (not resending)\n",s->lastReceivedFrameTail);
      s->sendAsHead[lr]=0;
    }else{
      printf("broadcasting %s from tail, frame %d (%d, desired %d)\n",&glob->streamIndex[s->index*glob->maxStreamNameLen],lr,s->lastReceivedFrameTail,desiredFrame);
      broadcast(s,lr);
    }
  }else{
    //Hopefully, the only reason we'd get here is if the frame number has wrapped round (32 bit int).  So, in this case, we ought to reset some counters.  Or, we could get here if we're already up to date, and not needing to send...
    if(s->lastReceivedFrameTail>CIRCFRAMENO(s->cb,lw)){//we've already sent a frame with a higher frame number... so frame numbers must have wrapped round
      s->lastReceivedTail=lw;
      s->lastReceivedFrameTail=CIRCFRAMENO(s->cb,lw);
      printf("Broadcast problem - can't find frame - has counter wrapped around?  Broadcasting %s from tail, frame %d (%d)\n",&glob->streamIndex[s->index*glob->maxStreamNameLen],lw,s->lastReceivedFrameTail);
      broadcast(s,lw);
      lr=lw;
    }else{//probably already sent... so nothing to do.
      
    }
  }
  if(lr==lw || s->lastReceivedFrameHead<s->lastReceivedFrameTail){//we've just sent the head anyway, since the tail was at the head.
    s->lastReceivedFrameHead=s->lastReceivedFrameTail;
    gettimeofday(&t,NULL);
    s->lastHeadFrametimeSent=t.tv_sec+1e-6*t.tv_usec;
  }
  return 0;
}

int broadcastDataFromHead(streamStruct *s,int lw,int headtype){
  struct timeval t;
  int ns=NSTORE(s->cb);
  int *tmp;
  //If s->decHead!=0 then the frame sent should be a multiple of this value.  Otherwise, it should just be the latest.
  //Note - we don't select the frame number here, rather in the broadcast function, because by then the mutex will have been acquired...
  if(headtype==1)//decHead!=0, and time to send...
    lw=broadcast(s,-s->decHead);
  else//most recent (time based request).
    lw=broadcast(s,-1);
  if(lw>=0){
    s->lastReceivedFrameHead=CIRCFRAMENO(s->cb,lw);
    gettimeofday(&t,NULL);
    s->lastHeadFrametimeSent=t.tv_sec+1e-6*t.tv_usec;
    ns=NSTORE(s->cb);
    if(s->sendAsHeadSize<ns){
      if((tmp=realloc(s->sendAsHead,ns*sizeof(int)))==NULL){
	printf("Unable to realloc memory for sendAsHead - probably time to crash!\n");
	return 1;
      }else{
	s->sendAsHead=tmp;
	memset(&tmp[s->sendAsHeadSize],0,sizeof(int)*(ns-s->sendAsHeadSize));
	s->sendAsHeadSize=ns;
      }
    }
    s->sendAsHead[lw]=1;
  }
  return 0;
}
int hcf(int a,int b){
  //for a>b.y
  int c;
  if(b>a){
    c=a;
    a=b;
    b=c;
  }
  while(1){
    c = a%b;
    if(c==0)
      return b;
    a = b;
    b = c;
  }
}
int hcfArr(int n,int *arr){
  int i=0,c=0;
  while(c==0 && i<n){//get first nonzero value
    c=arr[i];
    i++;
  }
  while(i<n){
    if(arr[i]!=0)
      c=hcf(c,arr[i]);
    i++;
  }
  return c;
}

void updateDecs(streamStruct *s){
  int i;
  globalStruct *glob=s->glob;
  int offset=s->index*glob->nclients;
  if(glob->decHead[offset]!=0)
    s->decHead=glob->decHead[offset];
  else
    s->decHead=hcfArr(glob->nclients,&glob->decHead[offset]);
  if(glob->decTail[offset]!=0)
    s->decTail=glob->decTail[offset];
  else
    s->decTail=hcfArr(glob->nclients,&glob->decTail[offset]);
  if(glob->decFreqTime[offset]!=0)
    s->decFreqTime=glob->decFreqTime[offset];
  else{//find minimum...
    s->decFreqTime=0;
    for(i=1;i<glob->nclients;i++){
      if(glob->decFreqTime[offset+i]>0 && (glob->decFreqTime[offset+i]<s->decFreqTime || s->decFreqTime==0))
	s->decFreqTime=glob->decFreqTime[offset+i];
    }
  }
}

int openCircSHM(streamStruct *s){
  //int cnt=1,n=0;
  //int shmOpen=0;
  int rt=0;
  char *fullname=&s->glob->streamIndex[s->index*s->glob->maxStreamNameLen];
  struct stat statbuf;
  if(fullname[0]==0){
    printf("Error - cannot (re)open unnamed circular buffer\n");
    return -1;
  }
  if(s->fullname!=NULL)
    free(s->fullname);
  s->fullname=NULL;
  if(asprintf(&s->fullname,"/%s",fullname)==-1){
    printf("Error (re)opening circular buffer: %s\n",strerror(errno));
    return -1;
  }
  if(s->devshmfullname!=NULL)
    free(s->devshmfullname);
  s->devshmfullname=NULL;
  if(asprintf(&s->devshmfullname,"/dev/shm/%s",fullname)==-1){
    printf("Error (re)-opening circular buffer: %s\n",strerror(errno));
    return -1;
  }


  //while(shmOpen==0){
  if(stat(s->devshmfullname,&statbuf)!=-1 && (s->cb=circOpenBufReader(s->fullname))!=NULL){
    //shmOpen=1;
    printf("/dev/shm%s opened\n",s->fullname);
  }else{
    rt=-1;
    printf("Failed to open %s\n",s->devshmfullname);
    //s->cb=NULL;
    // sleep(1);
    //n++;
    //if(n==cnt){
    //	cnt*=2;
    //	printf("ddsServer failed to (re)open /dev/shm%s\ntodo: remove thread if a stream doesn't exist.\n",s->fullname);
    //}
    //}
  }
  return rt;
}
int checkSHM(streamStruct *s){
  //Check to see whether open shm is same as newly opened shm.
  //Return 1 on failure - ie if not the same, or if not openable.
  int fd;
  int *buf,*cbuf;
  int hdrsize;
  int i,rt;
  if(s->cb==NULL){
    return 1;
  }
  //now try mapping the shm, and read the header.  Compare this header with the current circbuf.
  if((fd=shm_open(s->fullname,O_RDONLY,0))==-1){
    //printf("shm_open failed for %s:%s\n",name,strerror(errno));
    return 1;
  }
  hdrsize=(int)((long)s->cb->data-(long)s->cb->mem);
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
  cbuf=(int*)s->cb->mem;
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


void *watchStream(void *sstr){
  //This is run in a thread.
  //Should this exit, it should remove the entry in the streamIndex, so that it can be restarted.
  //This function does:
  //opens the shm
  //broadcasts it when new stuff arrives
  //reopens shm if necessary.
  streamStruct *s=(streamStruct*)sstr;
  globalStruct *glob=s->glob;
  char *dds=glob->dds;
  char *ret;
  //int index=s->index;
  int err=0;
  int lw,headtype;
  if(openCircSHM(s)==-1){
    printf("error opening stream %s - thread exiting\n",s->fullname);
    pthread_mutex_lock(&glob->streamNameMutex);
    glob->streamIndex[s->index*glob->maxStreamNameLen]=0;
    pthread_mutex_unlock(&glob->streamNameMutex);
    return NULL;
  }
  
  circHeaderUpdated(s->cb);
  ret=circGetLatestFrame(s->cb);
  while(DDSGO(dds) && err==0){
    updateDecs(s);
    if(s->decHead!=0 || s->decTail!=0 || s->decFreqTime!=0){//some data is wanted
      ret=circGetNextFrame(s->cb,1,0);//1 second timeout
      if(ret==NULL){
	if(checkSHM(s)){//returns 1 on failure... or if the shm has changed.
	  printf("Reopening SHM\n");
	  if(openCircSHM(s)==-1){
	    printf("error reopening stream %s - thread exiting\n",s->fullname);
	    pthread_mutex_lock(&glob->streamNameMutex);
	    glob->streamIndex[s->index*glob->maxStreamNameLen]=0;
	    pthread_mutex_unlock(&glob->streamNameMutex);
	    return NULL;
	  }

	  ret=circGetNextFrame(s->cb,1,0);
	}
      }
      lw=LASTWRITTEN(s->cb);//circbuf.lastWritten[0];
      if(ret!=NULL && lw>=0){//There is new data in the buffer - but do we need to send it?
	//We look at the frame number of this frame (not the circular buffer entry number).
	if(s->lastSendHead){//we last sent from the head - so now try from tail
	  if(checkSendTail(s,lw)){//ready to send from the tail
	    broadcastDataFromTail(s,lw);
	    s->lastSendHead=0;
	  }else if((headtype=checkSendHead(s,lw))==1){//ready to send from head
	    broadcastDataFromHead(s,lw,1);
	  }else if(headtype==2){//time based head send...
	    broadcastDataFromHead(s,lw,2);
	  }
	}else{//might be time to send from head
	  if((headtype=checkSendHead(s,lw))==1){
	    broadcastDataFromHead(s,lw,1);
	    s->lastSendHead=1;
	  }else if(headtype==2){
	    broadcastDataFromHead(s,lw,2);
	    s->lastSendHead=1;
	  }else if(checkSendTail(s,lw)){
	    broadcastDataFromTail(s,lw);
	  }
	}
      }
    }else{//nothing is being requested.
      //so what do we wait for?  There will be another thread, or threads listening to requests from clients - decimates etc.  So, wait for that - with a timeout, in case the shm is altered directly, or in case the condition variable is signalled between us checking the decimations and getting here...
      struct timespec timeout;
      clock_gettime(CLOCK_REALTIME, &timeout);
      timeout.tv_sec+=(int)glob->ftimeout;
      timeout.tv_nsec+=(int)((glob->ftimeout-(int)glob->ftimeout)*1e9);
      if(timeout.tv_nsec>1000000000){
	timeout.tv_sec++;
	timeout.tv_nsec-=1000000000;
      }
      pthread_mutex_lock(&glob->timeoutmutex);
      pthread_cond_timedwait(&glob->timeoutcond,&glob->timeoutmutex,&timeout);
      pthread_mutex_unlock(&glob->timeoutmutex);
    }
  }
  return NULL;
}

int addToSendBuf(globalStruct *glob,int client,int size,char *data){
  int err=0;
  char *tmp;
  if(glob->dataToSendMemSize[client]-glob->dataToSendSize[client]<size){//realloc memory
    if((tmp=realloc(glob->dataToSend[client],glob->dataToSendSize[client]+size))==NULL){
      printf("Unable to realloc memory for sending data - data will not be sent\n");
      return 1;
    }else{
      glob->dataToSend[client]=tmp;
      glob->dataToSendMemSize[client]=glob->dataToSendSize[client]+size;
    }
  }
  memcpy(&(glob->dataToSend[client][glob->dataToSendSize[client]]),data,size);
  glob->dataToSendSize[client]+=size;
  return err;
}


int addStreamListToSendBuf(globalStruct *glob,int client){
  unsigned int cmd;
  int i,cnt,l;
  pthread_mutex_lock(&glob->streamNameMutex);
  cmd=0x55555506;
  addToSendBuf(glob,client,sizeof(unsigned int),(char*)&cmd);
  cnt=0;
  for(i=0;i<glob->nstreams;i++){
    if(glob->streamIndex[i*glob->maxStreamNameLen]!=0){
      cnt++;
    }
  }
  addToSendBuf(glob,client,sizeof(int),(char*)&cnt);//number of streams
  for(i=0;i<glob->nstreams;i++){
    if(glob->streamIndex[i*glob->maxStreamNameLen]!=0){
      addToSendBuf(glob,client,sizeof(int),(char*)&i);//the index
      l=strlen(&glob->streamIndex[i*glob->maxStreamNameLen]);
      addToSendBuf(glob,client,sizeof(int),(char*)&l);//length of name
      addToSendBuf(glob,client,l,&glob->streamIndex[i*glob->maxStreamNameLen]);//the name.
    }
  }
  pthread_mutex_unlock(&glob->streamNameMutex);
  return 0;
}

int resend(globalStruct *glob,int client,unsigned int indx0,unsigned int packetno){
  //packet contains:  1 bit as head, 1 bit resend, 14 bits frame number (lowest 14), 16 bits index, 32 bits packet offset in circular buffer, counter(32), packet size(32), then the data.
  //first of all, work out whether the packet is still available, or whether it has been overwritten.
  int hdrsize=sizeof(unsigned int)*4;
  unsigned int hdr[4];
  unsigned int frameno;
  int streamindx=indx0&0xffff;
  streamStruct *s=(streamStruct*)glob->streamStructs[streamindx];
  int partfno=(indx0>>16)&0x3fff;
  int packetsPerFrame=((s->cb->frameSize+glob->packetsize-hdrsize-1)/(glob->packetsize-hdrsize));
  int index=packetno/packetsPerFrame;//the index of the circular buffer.
  char *data;
  int pos;
  int size;
  int dsize;
  int offset;
  int nsent;
  if(!glob->resendThroughSocket)
    pthread_mutex_lock(&glob->sendMutex);
  if(index<NSTORE(s->cb)){
    frameno=CIRCFRAMENO(s->cb,index);
    if((frameno&0x3fff)==partfno){//data is still valid
      data=circGetFrame(s->cb,index);
      pos=packetno%packetsPerFrame;
      size=glob->packetsize-hdrsize;
      dsize=CIRCDATASIZE(s->cb,index);
      offset=size*pos;
      hdr[0]=(1<<30) | ((frameno&0x3fff)<<16) | (streamindx&0xffff);
      hdr[1]=packetno;
      hdr[2]=pos;
      hdr[3]=glob->packetsize;

      if(size>dsize-offset){
	size=dsize-offset;
	hdr[3]=size+hdrsize;//new packet size for this one.
      }
      if(glob->resendThroughSocket){
	addToSendBuf(glob,client,hdrsize,(char*)hdr);
	addToSendBuf(glob,client,size,&data[offset]);

      }else{//resend by broadcast...
	if((nsent=sendto(glob->bcastsock,hdr,hdrsize,MSG_MORE,(struct sockaddr*)&glob->bcastaddr,sizeof(glob->bcastaddr)))!=hdrsize){
	  printf("Error sending header (%d/%d sent): %s\n",nsent,hdrsize,strerror(errno));
	}
	if((nsent=sendto(glob->bcastsock,&data[offset],size,0,(struct sockaddr*)&glob->bcastaddr,sizeof(glob->bcastaddr)))!=size){
	  printf("Error sending data (%d/%d sent): %s\n",nsent,size,strerror(errno));
	}
      }
      
    }else{
      printf("Not resending packet %u for stream %d - data has been overwritten (frame now %d, requested %d)\n",packetno,streamindx,frameno,partfno);
      
    }
  }else{
    printf("Illegal index for resending - has data shape changed?  (stream %d, packetno %u)\n",streamindx,packetno);
  }
  if(!glob->resendThroughSocket)
    pthread_mutex_unlock(&glob->sendMutex);
  return 0;

}


int readPacket(globalStruct *glob,int client,unsigned int *packet){
  //a packet from client consists of:
  //cmd (1,2), (4 bytes, of which top 3 might be a header, eg 0x55555500)
  //if 1, stream id, decimations *3, 16 bytes.
  //if 2, packet id to resend (8 bytes), 8 bytes padding.
  //if 3, request a list of streams and current decimations.
  //if 4, set the forced (overriding) decimations.
  //if 5, this is sent from here to client as a keep alive.
  //if 6, this is sent, and is followed by the stream names.
  //if 7, prints out a list of active threads.
  //So, packet sizes arriving are all 20 bytes, but departing can be larger.
  int err=0;
  int id,i;
  if(packet[0]==0x55555501){//decimations sent.
    id=packet[1];
    if(id<glob->nstreams){
      glob->decHead[id*glob->nclients+client]=packet[2];
      glob->decTail[id*glob->nclients+client]=packet[3];
      glob->decFreqTime[id*glob->nclients+client]=((float*)packet)[4];
      //now wake up the threads.  This will actually wake them all, but thats probably ok...
      pthread_cond_broadcast(&glob->timeoutcond);
    }else{
      printf("Misunderstood packet - stream ID too large (%d > %d)\n",id,glob->nstreams);
    }
  }else if(packet[0]==0x55555502){//packet ID to resend.
    resend(glob,client,packet[1],packet[2]);
    
  }else if(packet[0]==0x55555503){
    //send a list of streams and current decimations.
    //Need a list of data to be sent out... (so that I don't do a blocking send).
    addStreamListToSendBuf(glob,client);
  }else if(packet[0]==0x55555504){
    //set the forced decimations
    id=packet[1];
    if(id<glob->nstreams){
      glob->decHead[id*glob->nclients]=packet[2];
      glob->decTail[id*glob->nclients]=packet[3];
      glob->decFreqTime[id*glob->nclients]=((float*)packet)[4];
      //now wake up the threads.  This will actually wake them all, but thats probably ok...
      pthread_cond_broadcast(&glob->timeoutcond);
    }else{
      printf("Misunderstood packet - stream ID too large (%d > %d)\n",id,glob->nstreams);
    }
  }else if(packet[0]==0x55555507){
    //print out a list of active threads...
    streamStruct *s;
    int cnt=0;
    pthread_mutex_lock(&glob->streamNameMutex);
    printf("Thread list:\n");
    for(i=0;i<glob->nstreams;i++){
      if(glob->streamIndex[i*glob->maxStreamNameLen]!=0){
	s=glob->streamStructs[i];
	printf("Thread with index %d on stream %s\n",(int)s->index,&glob->streamIndex[s->index*glob->maxStreamNameLen]);
	cnt++;
      }
    }
    printf("Total of %d threads\n",cnt);
    pthread_mutex_unlock(&glob->streamNameMutex);
    

  }else{
    printf("Packet header %#x not understood\n",packet[0]);
    err=1;//how do we re-sync the header in this case?
  }
  return err;
}


int acceptSocket(globalStruct *glob){
  struct sockaddr_in clientname;
  socklen_t size;
  int err=0;
  int clientsock,clientindx,i;
  size=(socklen_t)sizeof(struct sockaddr_in);
  if((clientsock=accept(glob->lsock,(struct sockaddr*)&clientname,&size))<0){
    printf("Failed to accept on socket: %s\n",strerror(errno));
    err=1;
  }else{
    clientindx=(clientname.sin_addr.s_addr>>24)&0xff;
    printf("Connected from %s port %d, this becomes client %d\n",inet_ntoa(clientname.sin_addr),(int)ntohs(clientname.sin_port),clientindx);
    if(glob->clientsock[clientindx]!=0){
      printf("Error - we already have a client from address %d - closing new socket\n",clientindx);
      close(clientsock);
      err=1;
    }else{
      glob->clientsock[clientindx]=clientsock;
      //turn off broadcasting to this client (until it requests something).
      for(i=0;i<glob->nstreams;i++){
	glob->decHead[i*glob->nclients+clientindx]=0;
	glob->decTail[i*glob->nclients+clientindx]=0;
	glob->decFreqTime[i*glob->nclients+clientindx]=0;
      }
      glob->dataToSendSize[clientindx]=0;//flush the buffer (ie it contains nowt).

    }
  }
  return err;
}

int openSocket(globalStruct *glob){
  //opens a listening socket
  //Create the socket and set up to accept connections.
  struct sockaddr_in name;
  int err=0;
  if((glob->lsock=socket(PF_INET,SOCK_STREAM,0))<0){
    printf("Error opening listening socket\n");
    err=1;
  }else{
    name.sin_family=AF_INET;
    name.sin_port=htons(glob->lport);
    name.sin_addr.s_addr=htonl(INADDR_ANY);
    if(bind(glob->lsock,(struct sockaddr*)&name,sizeof(name))<0){
      printf("Unable to bind\n");
      err=1;
      close(glob->lsock);
      glob->lsock=0;
    }
  }
  if(err==0){
    if(listen(glob->lsock,1)<0){
      printf("Failed to listen on port\n");
      err=1;
    }
  }
  return err;
}
int launchStreamThread(globalStruct *glob,int index){
  streamStruct *ss;
  if((ss=malloc(sizeof(streamStruct)))==NULL){
    printf("Failed to alloc streamStruct for %s\n",&glob->streamIndex[index*glob->maxStreamNameLen]);
    glob->streamIndex[index*glob->maxStreamNameLen]='\0';
    return -1;
  }
  memset(ss,0,sizeof(streamStruct));
  ss->index=index;
  ss->glob=glob;
  glob->streamStructs[index]=(void*)ss;
  pthread_create(&glob->streamThreadId[index],NULL,watchStream,(void*)ss);
  return 0;
}
int startExistingStreams(globalStruct *glob){
  //now, go through the stream index and create threads for each entry for which the circular buffer still exists.
  char *cbuf;
  struct stat st;
  int i;
  if((cbuf=malloc(glob->maxStreamNameLen+9))==NULL){
    printf("Unable to alloc temporary memory:Changing stream indexes\n");
    memset(glob->streamIndex,0,glob->maxStreamNameLen*glob->nstreams);
  }else{
    strcpy(cbuf,"/dev/shm/");
    for(i=0;i<glob->nstreams;i++){
      if(glob->streamIndex[i*glob->maxStreamNameLen]!='\0'){
	strncpy(&cbuf[9],&glob->streamIndex[i*glob->maxStreamNameLen],glob->maxStreamNameLen);
	cbuf[glob->maxStreamNameLen+8]='\0';
	if(stat(cbuf,&st)==-1){//no longer exists
	  printf("Removing index for non-existant stream %s\n",&glob->streamIndex[i*glob->maxStreamNameLen]);
	  glob->streamIndex[i*glob->maxStreamNameLen]='\0';
	}else{//still exists...
	  printf("Starting thread for %s\n",&glob->streamIndex[i*glob->maxStreamNameLen]);
	  launchStreamThread(glob,i);
	}
      }
    }
    free(cbuf);
  }
  return 0;
}

int checkStreamsClients(globalStruct *glob){
  //watch /dev/shm for streams, spawning a new thread for each.
  //First set a watch on the directory.
  char *dds=glob->dds;
  char buf[1024];
  struct inotify_event *event;
  int index;
  DIR *dirfd;
  fd_set readfd;
  fd_set writefd;
  char inbuf[20];
  unsigned int msg;
  struct dirent *dent;
  struct timeval t1;
  int mx,i;
  double ctime;
  int wmx;
  int n,pos;
  struct timeval selectTimeout;
  if((glob->inotify=inotify_init())==-1){
    printf("Error opening inotify: %s\n",strerror(errno));
    return -1;
  }
  if((glob->inotifywd=inotify_add_watch(glob->inotify,"/dev/shm",IN_CREATE))==-1){
    printf("Error getting watch descriptor: %s\n",strerror(errno));
    return -1;
  }
  //now, for any existing streams...
  if((dirfd=opendir("/dev/shm"))==NULL){
    printf("Error getting listing of files in /dev/shm\n");
    return 1;
  }
  startExistingStreams(glob);
  while((dent=readdir(dirfd))!=NULL){
    if((index=getNewStreamIndex(glob,dent->d_name))>=0){
      launchStreamThread(glob,index);
    }
  }
  closedir(dirfd);
  while(DDSGO(dds)){
    gettimeofday(&t1,NULL);
    ctime=t1.tv_sec+1e-6*t1.tv_usec;
    FD_ZERO(&readfd);
    mx=glob->lsock;
    wmx=0;
    FD_SET(glob->lsock,&readfd);
    FD_SET(glob->inotify,&readfd);
    if(glob->inotify>mx)
      mx=glob->inotify;
    for(i=0;i<glob->nclients;i++){
      if(glob->clientsock[i]!=0){
	FD_SET(glob->clientsock[i], &readfd);
	if(glob->clientsock[i]>mx)
	  mx=glob->clientsock[i];
	if(glob->grace[i]<ctime){//time to see if the socket is alive.
	  msg=0x55555505;//keepalive flag.
	  addToSendBuf(glob,i,sizeof(unsigned int),(char*)&msg);
	  glob->grace[i]=ctime+glob->gracetimeout;
	}
	if(glob->dataToSendSize[i]>0){//need to send something.
	  FD_SET(glob->clientsock[i],&writefd);
	  if(glob->clientsock[i]>wmx)
	    wmx=glob->clientsock[i];
	}
      }
    }
    selectTimeout.tv_sec=10;//(sstr->decimate==0);
    selectTimeout.tv_usec=0;
    if(select((mx>wmx?mx:wmx)+1,&readfd,&writefd,NULL,&selectTimeout)<0){
      printf("Error in select: %s\n",strerror(errno));
    }else{
      //check for new files...
      if(FD_ISSET(glob->inotify,&readfd)){
	n=read(glob->inotify,buf,sizeof(buf));
	if(n<0){
	  printf("Error in inotify read: %s\n",strerror(errno));
	}else{
	  i=0;
	  while(i<n){
	    event=(struct inotify_event*)&buf[i];
	    if(event->wd==glob->inotifywd){
	      if(event->mask&IN_CREATE){
		if(event->len>0 && (index=getNewStreamIndex(glob,event->name))>=0){
		  launchStreamThread(glob,index);
		  //should we send message to clients?
		}
	      }
	    }else{
	      printf("Wrong inotify event descriptor returned by read\n");
	    }
	    i+=sizeof(struct inotify_event)+event->len;
	  }
	}
      }
      //check for new clients.
      if(FD_ISSET(glob->lsock,&readfd)){//new client...
	acceptSocket(glob);
      }
      //now write/read any sockets... we write first, because reading can add stuff to the send buffer.
      for(i=0;i<glob->nclients;i++){
	if(glob->dataToSendSize[i]>0 && glob->clientsock[i]!=0){//time to send a keep alive.  We just write to the socket.  Then if the socket is no longer functioning, we'll get an error.
	  if(FD_ISSET(glob->clientsock[i],&writefd)){
	    if((n=write(glob->clientsock[i],glob->dataToSend[i],glob->dataToSendSize[i]))<0){
	      printf("Error writing to client %d: %s - closing connection\n",i,strerror(errno));
	      close(glob->clientsock[i]);
	      glob->clientsock[i]=0;
	    }else if(n==0){
	      printf("Warning nothing written to client %d\n",i);
	    }else{
	      glob->dataToSendSize[i]-=n;
	      if(glob->dataToSendSize[i]>0)
		memmove(glob->dataToSend[i],&glob->dataToSend[i][n],glob->dataToSendSize[i]);
	    }
	  }else{//what can we do if the socket isn't ready for writing?
	    printf("Client %d not ready for writing keepalive - what should we do?\n",i);
	  }
	}

	//a packet from client consists of:
	//cmd (1,2), (4 bytes, of which top 3 might be a header, eg 0x55555500)
	//if 1, stream id, decimations *3, 16 bytes.
	//if 2, packet id to resend (8 bytes), 8 bytes padding.
	//if 3, request a list of streams and current decimations.
	//if 4, set the forced (overriding) decimations.
	//So, packet sizes are all 20 bytes.
	if(glob->clientsock[i]!=0){
	  if(FD_ISSET(glob->clientsock[i],&readfd)){
	    //we'll read the whole 20 bytes at once - they should appear simultaneously since fit within one packet.
	    pos=0;
	    while(pos<20){
	      if((n=read(glob->clientsock[i],&inbuf[pos],20-pos))<=0){
		if(n==0){
		  printf("Socket to ddsServer closed\n");
		}else{
		  printf("Error reading socket in ddsServer: %s\n",strerror(errno));
		}
		//now close the socket.
		close(glob->clientsock[i]);
		glob->clientsock[i]=0;
		break;
	      }else{
		pos+=n;
	      }
	    }
	    if(pos==20){//read full packet
	      readPacket(glob,i,(unsigned int*)inbuf);
	    }
	  }
	}

      }
    }
  }
  inotify_rm_watch(glob->inotify,glob->inotifywd);
  close(glob->inotify);
  return 0;
}

int openNewSHM(globalStruct *glob){
  int size=calcDDSSize(glob->nstreams,glob->maxStreamNameLen,glob->nclients);
  int fd;
  char *buf;
  pthread_mutexattr_t mutexattr;
  if((fd=shm_open("/dds",O_RDWR|O_CREAT,0777))==-1){
    printf("shm_open failed for /dds:%s\n",strerror(errno));
    return 1;
  }
  if(ftruncate(fd,size)==-1){
    printf("ftruncate failed /dds:%s\n",strerror(errno));
    close(fd);
    return 1;
  }
  //printf("Doing mmap\n");
  buf=mmap(0,size,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
  close(fd);
  if(buf==MAP_FAILED){
    printf("mmap failed for dds: %s\n",strerror(errno));
    return 1;
  }
  //printf("mmap done buf=%p now calling memset(buf,0,%d)\n",buf,size);
  memset(buf,0,size);
  //printf("done memset of mmap buffer\n");
  glob->dds=buf;
  glob->ddsSize=size;
  DDSGO(buf)=1;
  DDSNSTREAMS(buf)=glob->nstreams;
  DDSNCLIENTS(buf)=glob->nclients;
  DDSMAXNAMELEN(buf)=glob->maxStreamNameLen;
  DDSMUTEXSIZE(buf)=sizeof(pthread_mutex_t);
  pthread_mutexattr_init(&mutexattr);
  pthread_mutexattr_setpshared(&mutexattr,PTHREAD_PROCESS_SHARED);
  pthread_mutexattr_setrobust_np(&mutexattr,PTHREAD_MUTEX_ROBUST);
  pthread_mutex_init(DDSMUTEX(buf),&mutexattr);//darc should never try to lock this mutex.  All it should do is broadcast on the cond (without locking the mutex).
  DDSMUTEXINIT(buf)=1;
  glob->streamIndex=DDSSTREAMINDEX(buf);
  glob->decHead=DDSDECHEAD(buf);
  glob->decTail=DDSDECTAIL(buf);
  glob->decFreqTime=DDSDECFREQTIME(buf);
  return 0;
}

int setDec(globalStruct *glob,char *stream,int client,int decHead,int decTail,float decFreqTime){
  //sets decimation for client 0.
  int indx;
  indx=getIndexOfStream(glob,stream);
  if(indx>=0 && client<glob->nclients){
    if(decHead>=0)
      glob->decHead[indx*glob->nclients+client]=decHead;
    if(decTail>=0)
      glob->decTail[indx*glob->nclients+client]=decTail;
    if(decFreqTime>=0)
      glob->decFreqTime[indx*glob->nclients+client]=decFreqTime;
  }
  return indx<0;
}


int openExistingSHM(globalStruct *glob){
  int size,fd;
  void *buf;
  struct stat st;
  pthread_mutexattr_t mutexattr;
  if((fd=shm_open("/dds",O_RDWR,0))==-1){
    printf("shm_open failed for dds:%s\n",strerror(errno));
    return 1;
  }
  if(fstat(fd,&st)!=0){
    printf("Failed to stat dds\n");
    close(fd);
    return 1;
  }
  size=(int)st.st_size;
  //printf("Doing mmap (size %d)\n",size);
  buf=mmap(0,size,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
  close(fd);
  if(buf==MAP_FAILED){
    printf("mmap failed for dds: %s\n",strerror(errno));
    return 1;
  }
  //printf("mmap done buf=%p\n",buf);
  glob->dds=buf;
  glob->ddsSize=size;
  if(glob->nstreams!=0 && glob->nstreams!=DDSNSTREAMS(buf)){
    printf("Error - number of streams in shared memory (%d) not equal to that requested (%d).  Please either request the correct amount or remove /dev/shm/dds to start again\n",DDSNSTREAMS(buf),glob->nstreams);
    return 1;
  }else
    glob->nstreams=DDSNSTREAMS(buf);
  if(glob->nclients!=0 && glob->nclients!=DDSNCLIENTS(buf)){
    printf("Error - number of clients in shared memory (%d) not equal to that requested (%d).  Please either request the correct amount or remove /dev/shm/dds to start again\n",DDSNCLIENTS(buf),glob->nclients);
    return 1;
  }else
    glob->nclients=DDSNCLIENTS(buf);
  if(glob->maxStreamNameLen!=0 && glob->maxStreamNameLen!=DDSMAXNAMELEN(buf)){
    printf("Error - max stream name length in shared memory (%d) not equal to that requested (%d).  Please either request the correct amount or remove /dev/shm/dds to start again\n",DDSMAXNAMELEN(buf),glob->maxStreamNameLen);
    return 1;
  }else
    glob->maxStreamNameLen=DDSMAXNAMELEN(buf);
  if(DDSMUTEXSIZE(buf)!=sizeof(pthread_mutex_t)){
    printf("Error - dds mutex size wrong (%d instead of %d)\n",(int)DDSMUTEXSIZE(buf),(int)sizeof(pthread_mutex_t));
    return 1;
  }
  //do we need to initialise the mutex?  Probably not.
  if(DDSMUTEXINIT(buf)==0){
    pthread_mutexattr_init(&mutexattr);
    pthread_mutexattr_setpshared(&mutexattr,PTHREAD_PROCESS_SHARED);
    pthread_mutexattr_setrobust_np(&mutexattr,PTHREAD_MUTEX_ROBUST);
    pthread_mutex_init(DDSMUTEX(buf),&mutexattr);//darc should never try to lock this mutex.  All it should do is broadcast on the cond (without locking the mutex).
    DDSMUTEXINIT(buf)=1;
  }
  pthread_mutexattr_destroy(&mutexattr);
  
  glob->streamIndex=DDSSTREAMINDEX(buf);
  glob->decHead=DDSDECHEAD(buf);
  glob->decTail=DDSDECTAIL(buf);
  glob->decFreqTime=DDSDECFREQTIME(buf);
  return 0;
}

int initSockaddr (struct sockaddr_in *name,const char *hostname,uint16_t port){
  struct hostent *hostinfo;
  name->sin_family = AF_INET;
  name->sin_port = htons(port);
  hostinfo=gethostbyname(hostname);
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


int sendPacket(globalStruct *glob,int *packet){
  int sock;
  //connects to existing, and sends packet (which should be 20 bytes).
  if((sock=connectsock(glob->lhost,glob->lport))<0){
    printf("Unable to connect to dds\n");
    return -1;
  }else{
    send(sock,packet,sizeof(int)*5,0);
  }
  return sock;
}


void printUsage(char *n){
  printf("Usage: %s [--packetsize=1492] [--nstreams=4096] [--resendThroughSocket=0] [--nclients=256] [--maxNameLen=80] [--bcasthost=10.0.1.0] [--bcastport=4225] [--printdds] [--clear] [--resetTail] [--resetHead] [--resetTime] [--setHead=DEC --stream=STREAM] [--setTail=DEC --stream=STREAM] [--setTime=DEC --stream=STREAM] [--printthread] [configfile]\n",n);

}



int main(int argc, char **argv){
  int i,j,k;
  globalStruct *glob;
  struct stat statbuf;
  int dec;
  float fdec;
  char *stream;
  if((glob=calloc(sizeof(globalStruct),1))==NULL){
    printf("Error mallocing glob\n");
    return 1;
  }
  //set defaults
  glob->packetsize=1492;
  glob->resendThroughSocket=0;
  glob->bcastPort=4225;
  glob->ftimeout=1.;
  glob->gracetimeout=1.;
  glob->lport=4226;
  glob->lhost=strdup("localhost");

  //sort out configuration.
  for(i=1;i<argc;i++){
    if(argv[i][0]=='-'){
      if(strncmp(argv[i],"--packetsize=",13)==0){
	glob->packetsize=atoi(&argv[i][13]);
      }else if(strncmp(argv[i],"--nclients=",11)==0){
	glob->nclients=atoi(&argv[i][11]);
      }else if(strncmp(argv[i],"--nstreams=",11)==0){
	glob->nstreams=atoi(&argv[i][11]);
      }else if(strncmp(argv[i],"--maxNameLen=",13)==0){
	glob->maxStreamNameLen=atoi(&argv[i][13]);
      }else if(strncmp(argv[i],"--resendThroughSocket=",22)==0){
	glob->resendThroughSocket=atoi(&argv[i][22]);
      }else if(strncmp(argv[i],"--bcasthost=",12)==0){
	if(glob->bcastHost!=NULL)
	  free(glob->bcastHost);
	glob->bcastHost=strdup(&argv[i][12]);
      }else if(strncmp(argv[i],"--bcastport=",12)==0){
	glob->bcastPort=atoi(&argv[i][12]);
      }else if(strcmp(argv[i],"--printdds")==0){
	//prints contents of the dds shm.
	if(stat("/dev/shm/dds",&statbuf)!=-1){//dds exists.
	  if(openExistingSHM(glob)!=0){
	    printf("Error reopening /dev/shm/dds\n");
	    return 1;
	  }
	  printf("dds:\nsize %d\ngo %d\nnstreams %d\nnclients %d\nmaxnamelen %d\nmutex size %d\n",DDSSIZE(glob->dds),DDSGO(glob->dds),DDSNSTREAMS(glob->dds),DDSNCLIENTS(glob->dds),DDSMAXNAMELEN(glob->dds),DDSMUTEXSIZE(glob->dds));
	  for(j=0;j<DDSNSTREAMS(glob->dds);j++){
	    if(DDSSTREAMINDEX(glob->dds)[j*DDSMAXNAMELEN(glob->dds)]!='\0'){
	      printf("Stream %d %s\n",j,&DDSSTREAMINDEX(glob->dds)[j*DDSMAXNAMELEN(glob->dds)]);
	    }
	  }
	  for(k=0;k<DDSNSTREAMS(glob->dds);k++){
	    for(j=0;j<DDSNCLIENTS(glob->dds);j++){
	      if(DDSDECHEAD(glob->dds)[k*glob->nclients+j]!=0 || DDSDECTAIL(glob->dds)[k*glob->nclients+j]!=0 || DDSDECFREQTIME(glob->dds)[k*glob->nclients+j]!=0){
		printf("Client %d stream %s decimations %d %d %g\n",j,&glob->streamIndex[k*glob->maxStreamNameLen],DDSDECHEAD(glob->dds)[k*glob->nclients+j],DDSDECTAIL(glob->dds)[k*glob->nclients+j],DDSDECFREQTIME(glob->dds)[k*glob->nclients+j]);
	      }
	    }
	  }
	}else{
	  printf("/dev/shm/dds does not exist\n");
	}
	return 0;
      }else if(strcmp(argv[i],"--clear")==0){//removes existing shm
	unlink("/dev/shm/dds");
      }else if(strcmp(argv[i],"--resetHead")==0){
	if(stat("/dev/shm/dds",&statbuf)!=-1){//dds exists.
	  if(openExistingSHM(glob)!=0){
	    printf("Error reopening /dev/shm/dds\n");
	    return 1;
	  }
	  memset(DDSDECHEAD(glob->dds),0,sizeof(int)*glob->nclients*glob->nstreams);
	}
	return 0;
      }else if(strcmp(argv[i],"--resetTail")==0){
	if(stat("/dev/shm/dds",&statbuf)!=-1){//dds exists.
	  if(openExistingSHM(glob)!=0){
	    printf("Error reopening /dev/shm/dds\n");
	    return 1;
	  }
	  memset(DDSDECTAIL(glob->dds),0,sizeof(int)*glob->nclients*glob->nstreams);
	}
	return 0;
      }else if(strcmp(argv[i],"--resetTime")==0){
	if(stat("/dev/shm/dds",&statbuf)!=-1){//dds exists.
	  if(openExistingSHM(glob)!=0){
	    printf("Error reopening /dev/shm/dds\n");
	    return 1;
	  }
	  memset(DDSDECFREQTIME(glob->dds),0,sizeof(int)*glob->nclients*glob->nstreams);
	}
	return 0;
      }else if(strncmp(argv[i],"--setHead=",10)==0){
	dec=atoi(&argv[i][10]);
	//find out which stream...
	if(argc>i+1 && strncmp(argv[i+1],"--stream=",9)==0){
	  stream=&argv[i+1][9];
	  if(stat("/dev/shm/dds",&statbuf)!=-1){//dds exists.
	    if(openExistingSHM(glob)!=0){
	      printf("Error reopening /dev/shm/dds\n");
	      return 1;
	    }
	    setDec(glob,stream,0,dec,-1,-1.);
	    printf("Setting %4s decimation of %s to %d\n",&argv[i][5],stream,dec);
	  }
	}else{
	  printf("%s expects a following --stream=STREAM, but not found\n",argv[i]);
	}
	return 0;
      }else if(strncmp(argv[i],"--setTail=",10)==0){
	dec=atoi(&argv[i][10]);
	//find out which stream...
	if(argc>i+1 && strncmp(argv[i+1],"--stream=",9)==0){
	  stream=&argv[i+1][9];
	  if(stat("/dev/shm/dds",&statbuf)!=-1){//dds exists.
	    if(openExistingSHM(glob)!=0){
	      printf("Error reopening /dev/shm/dds\n");
	      return 1;
	    }
	    setDec(glob,stream,0,-1,dec,-1.);
	    printf("Setting %4s decimation of %s to %d\n",&argv[i][5],stream,dec);
	  }
	}else{
	  printf("%s expects a following --stream=STREAM, but not found\n",argv[i]);
	}
	return 0;
      }else if(strncmp(argv[i],"--setTime=",10)==0){
	fdec=(float)atof(&argv[i][10]);
	//find out which stream...
	if(argc>i+1 && strncmp(argv[i+1],"--stream=",9)==0){
	  stream=&argv[i+1][9];
	  if(stat("/dev/shm/dds",&statbuf)!=-1){//dds exists.
	    if(openExistingSHM(glob)!=0){
	      printf("Error reopening /dev/shm/dds\n");
	      return 1;
	    }
	    setDec(glob,stream,0,-1,-1,fdec);
	    printf("Setting %4s decimation of %s to %g\n",&argv[i][5],stream,fdec);
	  }
	}else{
	  printf("%s expects a following --stream=STREAM, but not found\n",argv[i]);
	}
	return 0;
      }else if(strcmp(argv[i],"--stop")==0){
	if(stat("/dev/shm/dds",&statbuf)!=-1){//dds exists.
	  if(openExistingSHM(glob)!=0){
	    printf("Error reopening /dev/shm/dds\n");
	    return 1;
	  }
	  DDSGO(glob->dds)=0;
	}
      }else if(strcmp(argv[i],"--printthread")==0){
	//connects to existing, and sends 0x55555507.
	int packet[5];
	packet[0]=0x55555507;
	sendPacket(glob,packet);
	return 0;
      }else if(strcmp(argv[i],"--help")==0 || strcmp(argv[i],"-h")==0){
	printUsage(argv[0]);
	exit(0);
      }else if(strncmp(argv[i],"--stream=",9)==0){
	//do nowt - this option has been used when setting time etc.
      }else{
	printf("Unknown option: %s\n",argv[i]);
	exit(0);
      }
    }else{//must be a config file.
      struct stat statbuf;
      if(stat(argv[i],&statbuf)==-1){
	printf("Error finding configfile %s: %s\n",argv[i],strerror(errno));
	printUsage(argv[0]);
	exit(0);
      }else{
	//read the config file.
	printf("todo - read config file\n");
      }
    }
  }

  //first see if we're starting with existing shm or from scratch
  if(stat("/dev/shm/dds",&statbuf)!=-1){//dds exists.
    if(openExistingSHM(glob)!=0){
      printf("Error reopening /dev/shm/dds\n");
      return 1;
    }
  }else{
    if(glob->nclients>256){
      printf("ERROR - nclients>256 specified.  Resetting to 256\n");
      glob->nclients=256;
    }else if(glob->nclients==0)
      glob->nclients=256;
    if(glob->nstreams==0)
      glob->nstreams=128;
    if(glob->maxStreamNameLen==0)
      glob->maxStreamNameLen=80;
    if(openNewSHM(glob)!=0){
      printf("Error opening /dev/shm/dds\n");
      return 1;
    }
  }
  pthread_mutex_init(&glob->sendMutex,NULL);
  pthread_mutex_init(&glob->streamNameMutex,NULL);
  pthread_mutex_init(&glob->timeoutmutex,NULL);
  pthread_cond_init(&glob->timeoutcond,NULL);

  //now do a load of mallocing...
  if((glob->clientsock=calloc(glob->nclients,sizeof(int)))==NULL){
    printf("Error allocating clientsock array\n");
    return 1;
  }
  if((glob->grace=calloc(glob->nclients,sizeof(int)))==NULL){
    printf("Error allocating grace array\n");
    return 1;
  }
  if((glob->dataToSend=calloc(glob->nclients,sizeof(char*)))==NULL){
    printf("Error allocating dataToSend array\n");
    return 1;
  }
  if((glob->dataToSendSize=calloc(glob->nclients,sizeof(int)))==NULL){
    printf("Error allocating dataToSendSize array\n");
    return 1;
  }
  if((glob->dataToSendMemSize=calloc(glob->nclients,sizeof(int)))==NULL){
    printf("Error allocating dataToSendMemSize array\n");
    return 1;
  }
  if((glob->streamStructs=calloc(glob->nstreams,sizeof(streamStruct*)))==NULL){
    printf("Error allocating streamStructs array\n");
    return 1;
  }
  if((glob->streamThreadId=calloc(glob->nstreams,sizeof(pthread_t)))==NULL){
    printf("Error allocating streamThreadId array\n");
    return 1;
  }
  //open the broadcast socket.
  if((glob->bcastsock=socket(PF_INET,SOCK_DGRAM,IPPROTO_UDP))<0){
    printf("Error opening broadcast socket: %s\n",strerror(errno));
    return 1;
  }
  
  i=1;
  if(setsockopt(glob->bcastsock,SOL_SOCKET,SO_BROADCAST,(void*)&i,sizeof(int))<0){
    printf("Error enabling broadcast: %s\n",strerror(errno));
    return 1;
  }
  if(glob->bcastHost==NULL)
    glob->bcastHost=strdup("10.0.1.0");
  memset(&glob->bcastaddr,0,sizeof(glob->bcastaddr));
  glob->bcastaddr.sin_family=AF_INET;
  glob->bcastaddr.sin_addr.s_addr=inet_addr(glob->bcastHost);
  glob->bcastaddr.sin_port = htons(glob->bcastPort);
  //open the listening socket
  if(openSocket(glob)!=0){
    printf("Error opening listening socket\n");
    return 1;
  }

  DDSGO(glob->dds)=1;
  //Get a list of current streams.  This will also spawn a thread for each stream, which will then start watching these streams, sending out when necessary.
  checkStreamsClients(glob);

  printf("todo: clean up exiting (if needed)\n");
  return 0;
}

  
  
