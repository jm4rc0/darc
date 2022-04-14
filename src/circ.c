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
Structs and functions for circular buffers...
Please note, these functions are not thread safe, and so must be called from a thread safe environment.
*/
#define _GNU_SOURCE
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ipc.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
//#include <semaphore.h>


#include "circ.h"
#include "darcMutex.h"



int calcDatasize(int nd,int *dims,char dtype){
  //return -1 on error.
  int i;
  int datasize=(nd>0);
  for(i=0; i<nd; i++){
    datasize*=dims[i];
  }
  switch(dtype){
  case 'i':
  case 'I':
  case 'f':
    datasize*=4;
    break;
  case 'l':
  case 'L':
  case 'd':
  case 'q':
    datasize*=8;
    break;
  case 'u':
  case 'c':
  case 'b':
  case 'B':
    datasize*=1;
    break;
  case 'h':
  case 'H':
    datasize*=2;
    break;
  default:
    printf("Datatype %c not known\n",dtype);
    datasize=-1;
    break;
  }
  return datasize;
}


int calcHdrsize(){
  int hdrsize=8+4+4+4+2+1+1+4+3*4+8;
  hdrsize+=4+4+4+4+4+sizeof(darc_condwait_t); // drj 140422: replaced darc_futex* calls with darc_condwait*
  hdrsize=((hdrsize+ALIGN-1)/ALIGN)*ALIGN;
  return hdrsize;
}

int circCalcHdrSize(){
  return calcHdrsize();
}

int makeArrays(circBuf *cb){
  int hdrsize=calcHdrsize();//((8+4+4+4+2+1+1+6*4+ALIGN-1)/ALIGN)*ALIGN;
  //int timesize;
  //int frameNoSize;
  //int nstore=NSTORE(cb);
  //frameNoSize=((4*nstore+ALIGN-1)/ALIGN)*ALIGN;
  //timesize=((8*nstore+ALIGN-1)/ALIGN)*ALIGN;
  //and now update the pointers...
  cb->data=(void*)(&(cb->mem[hdrsize]));//+frameNoSize+timesize]));
  //cb->timestamp=(double*)(&(cb->mem[hdrsize+frameNoSize]));
  //cb->frameNo=(int*)(&(cb->mem[hdrsize]));
  cb->datasize=calcDatasize((int)NDIM(cb),SHAPEARR(cb),DTYPE(cb));
  if(cb->datasize>=0)
    cb->frameSize=((cb->datasize+CIRCHSIZE+ALIGN-1)/ALIGN)*ALIGN;//the size of the data and the header...
  else
    cb->frameSize=0;//((16+ALIGN-1)/ALIGN)*ALIGN;
  return cb->datasize<0;//return 1 on error...
}

int circReshape(circBuf *cb,int nd, int *dims,char dtype){
  //Reshape the circular buffer.
  //return nonzero on error.
  int hdrsize=calcHdrsize();//((8+4+4+4+2+1+1+6*4+ALIGN-1)/ALIGN)*ALIGN;
  //int timesize;
  //int frameNoSize;
  //int datasize;
  int nstore;
  int err=0;
  int memleft;
  //int actualsize;
  int samesize;
  //int frameSize;
  int oldsize;
  int dim[CIRCDIMSIZE],i;
  oldsize=cb->datasize;
  DTYPE(cb)=dtype;

  if(nd>CIRCDIMSIZE){
    printf("DEPRECIATION WARNING - dimensions>%d for circular buffer no longer supported (requested %d)\n",CIRCDIMSIZE,nd);
    dim[0]=1;
    for(i=0;i<nd;i++)
      dim[0]*=dims[i];
    nd=1;
  }else
    memcpy(dim,dims,sizeof(int)*nd);
  memcpy(SHAPEARR(cb),dim,sizeof(int)*nd);
  if(nd<CIRCDIMSIZE)
    memset(&((SHAPEARR(cb))[nd]),0,sizeof(int)*(CIRCDIMSIZE-nd));
  NDIM(cb)=(char)nd;

  err=makeArrays(cb);
  //datasize=calcDatasize(nd,dims,dtype);
  //if(datasize<0)
  //  err=1;
  samesize=(oldsize==cb->datasize);
  if(err==0){
    if(!samesize){
      LASTWRITTEN(cb)=-1;
      memleft=BUFSIZE(cb)-hdrsize;
      nstore=memleft/cb->frameSize;//(4+8+datasize);
      NSTORE(cb)=nstore;
      if(nstore<=0){
	printf("Not enough memory to store anything in the buffer\n");
	err=1;
	NSTORE(cb)=0;
      }
    /*if(err==0){
      nstore++;
      do{
	nstore--;
	frameNoSize=((4*nstore+ALIGN-1)/ALIGN)*ALIGN;//#size of the frameno (int32 for each entry)
	timesize=((8*nstore+ALIGN-1)/ALIGN)*ALIGN;//#size of the timestamp (float 64 for each entry)
	actualsize=hdrsize+frameNoSize+timesize+nstore*datasize;
      }while(actualsize>BUFSIZE(cb));
      NSTORE(cb)=nstore;
      }*/
      printf("circReshape: Got nstore %d (datasize %d, framesize %d)\n",nstore,cb->datasize,cb->frameSize);
      //if(err==0)
      //err=makeArrays(cb);
    }else{//same size as previously...
      if((nstore=NSTORE(cb))==0){
	err=1;
	printf("circReshape: Got nstore %d (datasize %d)\n",nstore,cb->datasize);
      }
    }
  }else{
    NSTORE(cb)=0;
    printf("circReshape: Got nstore 0 (datasize %d)\n",cb->datasize);
  }
  return err;
}

#define TIMESTAMP(cb,indx) *((double*)(&(((char*)cb->data)[indx*cb->frameSize+8])))
#define FRAMENO(cb,indx) *((int*)(&(((char*)cb->data)[indx*cb->frameSize+4])))
#define DATASIZE(cb,indx) *((int*)(&(((char*)cb->data)[indx*cb->frameSize])))
#define DATATYPE(cb,indx) *((char*)(&(((char*)cb->data)[indx*cb->frameSize+16])))
#define THEDATA(cb,indx) &(((char*)cb->data)[indx*cb->frameSize+CIRCHSIZE])
#define THEFRAME(cb,indx) &(((char*)cb->data)[indx*cb->frameSize])
/**
   Add data which is of size, to the circular buffer.  This may not be a complete entry, but we add it anyway - e.g. status or error messages, which may not be of fixed length.
*/
int circSetAddIfRequired(circBuf *cb,int frameno){
  if((FREQ(cb)>0 && frameno%FREQ(cb)==0) || FORCEWRITE(cb)!=0){
    //add to the buffer
    cb->addRequired=1;
    if(FORCEWRITE(cb)>0)
      FORCEWRITE(cb)--;
  }else
    cb->addRequired=0;
  return cb->addRequired;
}
/*Now defined in header
inline int circCheckAddRequired(circBuf *cb){
  return cb->addRequired;
}
*/
int circAddSize(circBuf *cb,void *data,int size,int setzero,double timestamp,int frameno){
  //struct timeval t1;
  int err=0,indx;
  if(cb==NULL)
    return 1;
  //if((FREQ(cb)>0 && frameno%FREQ(cb)==0) || FORCEWRITE(cb)!=0){//add to the buffer...
    //if(FORCEWRITE(cb)>0)
    //FORCEWRITE(cb)--;
  if(cb->addRequired!=0 || ((FREQ(cb)>0 && frameno%FREQ(cb)==0) || FORCEWRITE(cb)!=0)){
    if(cb->addRequired!=0){
      cb->addRequired=0;
      if(FORCEWRITE(cb)>0)
	FORCEWRITE(cb)--;
    }
    indx=LASTWRITTEN(cb)+1;
    if(indx>=NSTORE(cb))
      indx=0;
    memcpy(&(((char*)cb->data)[indx*cb->frameSize+CIRCHSIZE]),data,size<cb->datasize?size:cb->datasize);
    if(setzero==1 && size<cb->datasize){//set the rest to zero...
      memset(&(((char*)cb->data)[indx*cb->frameSize+CIRCHSIZE+size]),0,cb->datasize-size);
    }
    //gettimeofday(&t1,NULL);
    DATASIZE(cb,indx)=cb->datasize+CIRCHSIZE-4;
    FRAMENO(cb,indx)=frameno;
    TIMESTAMP(cb,indx)=timestamp;
    DATATYPE(cb,indx)=DTYPE(cb);
    //cb->timestamp[indx]=timestamp;//t1.tv_sec+t1.tv_usec*1e-6;
    //cb->frameNo[indx]=frameno;//cb->framecnt;
    LASTWRITTEN(cb)=indx;
    //unblock condwait
    darc_condwait_broadcast(cb->condwait);
  }
  return err;
}

int circAdd(circBuf *cb,void *data,double timestamp,int frameno){
  //struct timeval t1;
  int err=0,indx;
  if(cb==NULL)
    return 1;
  //if((FREQ(cb)>0 && frameno%FREQ(cb)==0) || FORCEWRITE(cb)!=0){//add to the buffer...
  //if(FORCEWRITE(cb)>0)
  //  FORCEWRITE(cb)--;
  if(cb->addRequired!=0 || ((FREQ(cb)>0 && frameno%FREQ(cb)==0) || FORCEWRITE(cb)!=0)){
    if(cb->addRequired!=0){
      cb->addRequired=0;
      if(FORCEWRITE(cb)>0)
	FORCEWRITE(cb)--;
    }
    indx=LASTWRITTEN(cb)+1;
    if(indx>=NSTORE(cb))
      indx=0;
    memcpy(&(((char*)cb->data)[indx*cb->frameSize+CIRCHSIZE]),data,cb->datasize);
    //gettimeofday(&t1,NULL);
    DATASIZE(cb,indx)=cb->datasize+CIRCHSIZE-4;
    FRAMENO(cb,indx)=frameno;
    TIMESTAMP(cb,indx)=timestamp;
    DATATYPE(cb,indx)=DTYPE(cb);
    //cb->timestamp[indx]=t1->tv_sec+t1->tv_usec*1e-6;
    //cb->timestamp[indx]=timestamp;
    //cb->frameNo[indx]=frameno;//cb->framecnt;
    LASTWRITTEN(cb)=indx;
    darc_condwait_broadcast(cb->condwait);
  }
  return err;
}

int circAddForce(circBuf *cb,void *data,double timestamp,int frameno){
  //Always add to the circular buffer.
  int err=0,indx;
  if(cb==NULL)
    return 1;
  indx=LASTWRITTEN(cb)+1;
  if(indx>=NSTORE(cb))
    indx=0;
  memcpy(&(((char*)cb->data)[indx*cb->frameSize+CIRCHSIZE]),data,cb->datasize);
  //gettimeofday(&t1,NULL);
  DATASIZE(cb,indx)=cb->datasize+CIRCHSIZE-4;
  FRAMENO(cb,indx)=frameno;
  TIMESTAMP(cb,indx)=timestamp;
  DATATYPE(cb,indx)=DTYPE(cb);
  LASTWRITTEN(cb)=indx;
  darc_condwait_broadcast(cb->condwait);
  return err;
}

int circAddSizeForce(circBuf *cb,void *data,int size,int setzero,double timestamp,int frameno){
  //Always add to the circular buffer.
  int err=0,indx;
  if(cb==NULL)
    return 1;
  indx=LASTWRITTEN(cb)+1;
  if(indx>=NSTORE(cb))
    indx=0;
  memcpy(&(((char*)cb->data)[indx*cb->frameSize+CIRCHSIZE]),data,size<cb->datasize?size:cb->datasize);
  if(setzero==1 && size<cb->datasize){//set the rest to zero
    memset(&(((char*)cb->data)[indx*cb->frameSize+CIRCHSIZE+size]),0,cb->datasize-size);
  }
  //gettimeofday(&t1,NULL);
  DATASIZE(cb,indx)=cb->datasize+CIRCHSIZE-4;
  FRAMENO(cb,indx)=frameno;
  TIMESTAMP(cb,indx)=timestamp;
  DATATYPE(cb,indx)=DTYPE(cb);
  LASTWRITTEN(cb)=indx;
  darc_condwait_broadcast(cb->condwait);
  return err;
}

int circInsert(circBuf *cb,void* data,int size, int offset){
  //Insert data of size bytes into the buffer, at offset.
  //After this has been called, a circAddSize(Force) call should also be done, to publish the data.
  int indx;
  if(cb==NULL)
    return 1;
  indx=LASTWRITTEN(cb)+1;
  if(indx>=NSTORE(cb))
    indx=0;
  if(size+offset<=cb->datasize)
    memcpy(&(((char*)cb->data)[indx*cb->frameSize+CIRCHSIZE+offset]),data,size);
  else
    return 1;
  return 0;
  
}

/*
int circAddPartial(circBuf *cb,void *data,int offset,int size,double timestamp,int frameno){
  //struct timeval t1;
  int err=0,indx,lastEntry=0,forcewrite;
  //increment if 1 is the first entry, if -1 is the last entry.
  //offset and size are in bytes.
  if(cb==NULL)
    return 1;
  if(cb->sizecnt==0){
    //cb->freqcnt++;
    //cb->framecnt++;
  }
  forcewrite=FORCEWRITE(cb);//needed for thread safety
  cb->sizecnt+=size;
  if(cb->sizecnt==cb->datasize){//last entry...
    cb->sizecnt=0;
    lastEntry=1;
  }

  if(offset+size>cb->datasize)
    printf("Error - circAddPartial %d %d %d\n",offset,size,cb->datasize);
  if((FREQ(cb)>0 && frameno%FREQ(cb)==0) || forcewrite!=0){//add to the buffer...
    indx=LASTWRITTEN(cb)+1;
    if(indx>=NSTORE(cb))
      indx=0;
    memcpy(&(((char*)cb->data)[indx*cb->frameSize+CIRCHSIZE+offset]),data,size);
    if(lastEntry==1){//last entry...
      cb->sizecnt=0;
      //cb->freqcnt=0;
      if(FORCEWRITE(cb)>0)
	FORCEWRITE(cb)--;
      //gettimeofday(&t1,NULL);
      //cb->timestamp[indx]=t1.tv_sec+t1.tv_usec*1e-6;
      DATASIZE(cb,indx)=cb->datasize+CIRCHSIZE-4;
      FRAMENO(cb,indx)=frameno;
      TIMESTAMP(cb,indx)=timestamp;
      DATATYPE(cb,indx)=DTYPE(cb);
      //cb->timestamp[indx]=timestamp;
      //cb->frameNo[indx]=frameno;//cb->framecnt;
      LASTWRITTEN(cb)=indx;
      pthread_cond_broadcast(cb->cond);
    }
  }
  return err;
}
*/
void *circGetLatest(circBuf *cb){
  int indx=LASTWRITTEN(cb);
  CIRCSIGNAL(cb)=1;
  if(indx<0 || makeArrays(cb)!=0)
    return NULL;
  return (void*)(&(((char*)cb->data)[indx*cb->frameSize+CIRCHSIZE]));
}

void *circGetNext(circBuf *cb){
  //called by a client... (typically this won't be used - the python version will be used instead).
  //block on the semaphore...
  int err=0;
  CIRCSIGNAL(cb)=1;
  darc_condwait_wait(cb->condwait);
  if(err)
    return NULL;
  else
    return circGetLatest(cb);
}


/*
pthread_cond_t *circCreateCond(char *name,int create){
  //name starts with a /
  //create condition variable that can be shared between processes.
  pthread_cond_t *cond;
  pthread_condattr_t condattr;
  int fd;
  if((fd=shm_open(name,O_RDWR|(O_CREAT*create),0777))==-1){
    printf("shm_open failed for %s:%s\n",name,strerror(errno));
    return NULL;
  }
  if(create){
    if(ftruncate(fd,sizeof(pthread_cond_t))==-1){
      printf("ftruncate failed:%s\n",strerror(errno));
      close(fd);
      return NULL;
    }
  }
  cond=mmap(0,sizeof(pthread_cond_t),PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
  close(fd);
  if(cond==MAP_FAILED){
    printf("mmap failed for\n");
    printf("%s with error number %d, error\n",name,errno);
    printf("%s\n",strerror(errno));
    return NULL;
  }
  if(create){
    pthread_condattr_init(&condattr);
    pthread_condattr_setpshared(&condattr,PTHREAD_PROCESS_SHARED);
    pthread_cond_init(cond,&condattr);
    pthread_condattr_destroy(&condattr);
  }
  return cond;
}
pthread_mutex_t *circCreateMutex(char *name,int create){
  //name starts with a /
  //create mutex that can be shared between processes.
  pthread_mutex_t *mutex;
  pthread_mutexattr_t mutexattr;
  int fd;
  if((fd=shm_open(name,O_RDWR|(O_CREAT*create),0777))==-1){
    printf("shm_open failed for %s:%s\n",name,strerror(errno));
    return NULL;
  }
  if(create){
    if(ftruncate(fd,sizeof(pthread_mutex_t))==-1){
      printf("ftruncate failed:%s\n",strerror(errno));
      close(fd);
      return NULL;
    }
  }
  mutex=mmap(0,sizeof(pthread_mutex_t),PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
  close(fd);
  if(mutex==MAP_FAILED){
    printf("mmap failed for\n");
    printf("%s with error number %d, error\n",name,errno);
    printf("%s\n",strerror(errno));
    return NULL;
  }
  if(create){
    pthread_mutexattr_init(&mutexattr);
    pthread_mutexattr_setpshared(&mutexattr,PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(mutex,&mutexattr);
    pthread_mutexattr_destroy(&mutexattr);
  }
  return mutex;
}
*/

circBuf* circAssign(char *name,void *mem,int memsize,int semid,int nd, int *dims,char dtype, circBuf *cb){
  //Use memory mem to create the circular buffer.
  //return NULL on error...
  //name starts with a /
  //if nd==0, assumed to be a reader.
  int err,malloced;
//   pthread_mutexattr_t mutexattr;
//   pthread_condattr_t condattr;

  if(cb==NULL){
    if((cb=malloc(sizeof(circBuf)))==NULL)
      printf("Error -circAssign malloc=NULL\n");
    malloced=1;
    memset(cb,0,sizeof(circBuf));
  }else{
    malloced=0;
  }
  cb->mem=mem;
  cb->memsize=memsize;
  cb->name=strdup(name);
  //snprintf(tmp,80,"%scond",name);
  //cb->cond=circCreateCond(tmp,nd>0);
  //snprintf(tmp,80,"%smutex",name);
  //cb->condmutex=circCreateMutex(tmp,nd>0);
  if(nd!=0){//opening as a writer - so need to initialise the mutex/cond
    CIRCPID(cb)=(int)getpid();
    CONDWAITSIZE(cb)=sizeof(darc_condwait_t);
    CIRCHDRSIZE(cb)=calcHdrsize();
    cb->condwait=CONDWAIT(cb);
    darc_condwait_init_shared(cb->condwait);
  }else{
    cb->condwait=CONDWAIT(cb);
  }

  //memset(mem,0,sizeof(memsize));
  BUFSIZE(cb)=memsize;
  if(nd>0){//use these values to set the buffer
    err=circReshape(cb,nd,dims,dtype);
  }else//buffer should already be set.
    err=makeArrays(cb);

  if(err){
    if(malloced)
      free(cb);
    cb=NULL;
  }

  return cb;
}


circBuf* circOpenBufReader(char *name){
  circBuf *cb=NULL;
  int size,fd;
  int semid=0;
  void *buf;
  struct stat st;
  //printf("circOpenBufReader %s\n",name);
  if((fd=shm_open(name,O_RDWR,0))==-1){
    //printf("shm_open failed for %s:%s\n",name,strerror(errno));
    return NULL;
  }
  if(fstat(fd,&st)!=0){
    printf("Failed to stat %s\n",name);
    close(fd);
    return NULL;
  }
  size=(int)st.st_size;
  //printf("Doing mmap (size for %s: %d)\n",name,size);
  buf=mmap(0,size,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
  close(fd);
  if(buf==MAP_FAILED){
    printf("mmap failed for\n");
    printf("%s with error number %d, error\n",name,errno);
    printf("%s\n",strerror(errno));
    return NULL;
  }
  //printf("mmap done buf=%p\n",buf);
  if((cb=circAssign(name,buf,size,semid,0,NULL,'\0',NULL))==NULL){
    printf("Could not create %s circular buffer object\n",name);
  }
  return cb;

}
int circCloseBufReader(circBuf *cb){
  if(cb!=NULL){
    if(cb->name!=NULL){
      free(cb->name);
      cb->name=NULL;
    }
    if(cb->mem!=NULL){
      munmap(cb->mem,cb->memsize);
      cb->mem=NULL;
    }
    free(cb);
  }
  return 0;
}
void *circGetFrame(circBuf *cb,int indx){
  CIRCSIGNAL(cb)=1;
  cb->lastReceived=indx;
  if(indx<0){
    cb->lastReceivedFrame=-1;
    return NULL;
  }
  //check arrays are right shape...
  if(makeArrays(cb)!=0){
    printf("Error making circ buf makeArrays()\n");
    return NULL;
  }
  cb->lastReceivedFrame=FRAMENO(cb,indx);
  return THEFRAME(cb,indx);
}

void *circGetLatestFrame(circBuf *cb){
  //Get the latest one...
  CIRCSIGNAL(cb)=1;
  int indx=LASTWRITTEN(cb);
  return circGetFrame(cb,indx);
}
int circHeaderUpdated(circBuf *cb){
  int update=0,i;
  if(NSTORE(cb)!=cb->nstoreSave || NDIM(cb)!=cb->ndimSave || DTYPE(cb)!=cb->dtypeSave){
    update=1;
  }else{
    for(i=0;i<NDIM(cb); i++){
      if(SHAPEARR(cb)[i]!=cb->dimsSave[i]){
	update=1;
	break;
      }
    }
  }
  if(update){
    cb->nstoreSave=NSTORE(cb);
    cb->ndimSave=NDIM(cb);
    cb->dtypeSave=DTYPE(cb);
    memcpy(cb->dimsSave,SHAPEARR(cb),sizeof(int)*CIRCDIMSIZE);
    circReshape(cb,NDIM(cb),SHAPEARR(cb),DTYPE(cb));
    //printf("circHeaderUpdated, dtype %c, last written %d\n",cb->dtypeSave,LASTWRITTEN(cb));
  }
  return update;
}


void *circGetNextFrame(circBuf *cb,float ftimeout,int retry){
  //Look at lastReceived and lastWritten, and then send if not equal, otherwise wait...
  //But - what to do if the buffer has just been reshaped and written to, so that lastWritten==0?  This typically might happen in the case of rtcGenericBuf.
  void *data=NULL;
  struct timespec timeout;
  int lw,lwf,timeup;
  CIRCSIGNAL(cb)=1;
  while(data==NULL){
    if(circHeaderUpdated(cb)){//self.nstoreSave!=self.nstore[0] or self.ndimSave!=self.ndim[0] or (not numpy.alltrue(self.shapeArrSave==self.shapeArr[:self.ndim[0]])) or self.dtypeSave!=self.dtype[0]:
      cb->lastReceived=-1;
      cb->lastReceivedFrame=-1;
    }
    lw=LASTWRITTEN(cb);//int(self.lastWritten[0])
    if(lw>=0){
      lwf=FRAMENO(cb,lw);//int(self.frameNo[lw])
    }else{
      lwf=-1;
    }
    if(lw<0){//no frame written yet - so block, waiting.
    }else if(lw==cb->lastReceived){
      //If the buffer has been reshaped, this might be the case even if new data has arrived.  So, we should also check the frame numbers here.  Also if the buffer has wrapped round...
      if(cb->lastReceivedFrame!=lwf){//probably new data
	cb->lastReceivedFrame=lwf;
	data=THEFRAME(cb,cb->lastReceived);
      }else{
	//printf("circGetNextFrame no new data\n");
      }
    }else{
      cb->lastReceived=(cb->lastReceived+1)%NSTORE(cb);
      cb->lastReceivedFrame=FRAMENO(cb,cb->lastReceived);
      data=THEFRAME(cb,cb->lastReceived);
    }
    if(data==NULL){
      if(ftimeout==0)
	break;
      // clock_gettime(CLOCK_REALTIME, &timeout);
      timeout.tv_sec = (long)ftimeout;
      timeout.tv_nsec = (ftimeout-timeout.tv_sec)*1000000000L; // drj 140422: changed conversion from float time to timespec
      errno=0;
      if(*((unsigned long*)cb->mem)==0){
	printf("Circular buffer size zero - probably buffer no longer in existance\n");
	break;
      }else{
	timeup=darc_condwait_timedwait(cb->condwait,&timeout);
      }
      if(timeup==0){
	if(circHeaderUpdated(cb)){//has been remade.
	  cb->lastReceived=-1;
	  cb->lastReceivedFrame=-1;
	}
	lw=LASTWRITTEN(cb);
	if(lw>=0)
	  lwf=FRAMENO(cb,lw);
	else
	  lwf=-1;
	//printf("lw now %d lr %d lwf %d\n",lw,cb->lastReceived,lwf);
	if(lw<0){//no frame yet written - do nothing.
	}else if(lw==cb->lastReceived){
	  if(cb->lastReceivedFrame!=lwf){//probably new data
	    cb->lastReceivedFrame=lwf;
	    data=THEFRAME(cb,cb->lastReceived);
	  }
	}else{
	  cb->lastReceived=(cb->lastReceived+1)%NSTORE(cb);
	  cb->lastReceivedFrame=FRAMENO(cb,cb->lastReceived);
	  data=THEFRAME(cb,cb->lastReceived);
	  //printf("Data is for frame %d (frameno %d, framesize %d datsize %d)\n",cb->lastReceived,cb->lastReceivedFrame,cb->frameSize,cb->datasize);
	}
      }else if(errno==EAGAIN || timeup==ETIMEDOUT){//timeout
	//printf("timeup %d in circGetNextFrame, retry=%d\n",timeup,retry);
	if(retry==0){
	  break;
	}else if(retry>0){
	  retry--;
	}
      }else{
	printf("semtimedop/ptread_cond_timedwait failed: %s\n",strerror(errno));
	break;
      }
    }
  }
  return data;
}

//circOpen
circBuf* openCircBuf(char *name,int nd,int *dims,char dtype,int nstore){
  //opens a circbuf for writing.
  //When implementing the additional buffer for head data, need to include the space for this in size, but not pass it to circAssign.  circAssign needs additional args for this extra buffer.
  circBuf *cb=NULL;
  int size,fd,semid=0;
  int hdrsize=calcHdrsize();//((8+4+4+4+2+1+1+6*4+ALIGN-1)/ALIGN)*ALIGN;
  void *buf;
  int dim[CIRCDIMSIZE],i;
  printf("openCircBuf %s %d %d %c %d\n",name,nd,dims[0],dtype,nstore);
  if(nd>CIRCDIMSIZE){
    printf("DEPRECIATION WARNING - dimensions>%d for circular buffer no longer supported (requested %d)\n",CIRCDIMSIZE,nd);
    dim[0]=1;
    for(i=0;i<nd;i++)
      dim[0]*=dims[i];
    nd=1;
  }else
    memcpy(dim,dims,sizeof(int)*nd);
  //size=calcDatasize(nd,dims,dtype)*nstore;
  //size+=hdrsize+((nstore*8+ALIGN-1)/ALIGN)*ALIGN+((nstore*4+ALIGN-1)/ALIGN)*ALIGN;//8 is for float64 timestamp, 4 is for indx int32.
  size=calcDatasize(nd,dim,dtype);
  if(size>=0){
    size=((size+CIRCHSIZE+ALIGN-1)/ALIGN)*ALIGN;//size of the data and associated header.
    size*=nstore;
    size+=hdrsize;
  }else{
    return NULL;
  }

  umask(0);
  if(shm_unlink(name)){
    //printf("unlink failed: %s\n",strerror(errno));
  }
  if((fd=shm_open(name,O_RDWR|O_CREAT,0777))==-1){
    //printf("shm_open failed for %s:%s\n",name,strerror(errno));
    return NULL;
  }
  if(ftruncate(fd,size)==-1){
    printf("ftruncate failed %s:%s\n",name,strerror(errno));
    close(fd);
    return NULL;
  }
  //printf("Doing mmap\n");
  buf=mmap(0,size,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
  close(fd);
  if(buf==MAP_FAILED){
    printf("mmap failed for\n");
    printf("%s with error number %d, error\n",name,errno);
    printf("%s\n",strerror(errno));
    return NULL;
  }
  //printf("mmap done buf=%p now calling memset(buf,0,%d)\n",buf,size);
  memset(buf,0,size);
  //printf("done memset of mmap buffer\n");
  if((cb=circAssign(name,buf,size,semid,nd,dim,dtype,NULL))==NULL){
    printf("Could not create %s circular buffer object\n",name);
  }
  if(cb!=NULL){
//     pthread_mutex_init(&cb->mutex,NULL);
    darc_condwait_init_shared(cb->condwait);
  }
  return cb;

}

void circClose(circBuf *cb){//should be called by the owner (writer) of the buffer.
  if(cb!=NULL){
    if(cb->name!=NULL){
      if(shm_unlink(cb->name))
	printf("Unable to unlink /dev/shm%s\n",cb->name);
      free(cb->name);
    }
    BUFSIZE(cb)=0;
    darc_condwait_destroy(cb->condwait);
//     pthread_mutex_destroy(cb->condmutex);
//     pthread_cond_broadcast(cb->cond);
//     pthread_cond_destroy(cb->cond);

    if(cb->mem!=NULL)
      munmap(cb->mem,cb->memsize);
  }
}
