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
#include <stdlib.h>
#include <wchar.h>
#include <string.h>
#include <stdarg.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <errno.h>
#include "buffer.h"
/**
   Checks that paramList is valid
   (alphabetical order, and of correct lenght (<16 chars), and correct size (16 bytes)
   paramList should have size of at least n*BUFNAMESIZE bytes.
*/
int bufferCheckNames(int n,char *paramList){
  int i;
  int err=0;
  if(sizeof(long long)!=8 || sizeof(wchar_t)!=4){
    printf("ERROR - sizeof(long long)!=8 or sizeof(wchar_t)!=4\n");
    err=1;
  }
  /*No longer needed since checking by strncmp
  if((((unsigned long)paramList)&0xf)!=0){
    printf("Error in bufferCheckNames - must be aligned to 16 byte boundary\n");
    err=1;
    }*/
  for(i=0; i<n; i++){
    if(i>0 && strncmp(&paramList[(i-1)*BUFNAMESIZE],&paramList[i*BUFNAMESIZE],BUFNAMESIZE)>=0){//alphabetical order?
      printf("ERROR - params not in alphabetical order: %16s %16s\n",&paramList[(i-1)*BUFNAMESIZE],&paramList[i*BUFNAMESIZE]);
      err=1;
    }
  }
  return err;
}

/**
   Given a list of n names, puts these into a newly created buffer, which can then be passed to bufferCheckNames and bufferGetIndex.
   e.g. bufferMakeNames(3,"bgImage","flatField","refCentroids");
*/
char *bufferMakeNames(int n,...){
  va_list l;
  char *b;
  int i;
  char *name;
  //if((i=posix_memalign((void**)&b,16,n*BUFNAMESIZE))!=0){
  if((b=calloc(n,BUFNAMESIZE))==NULL){
    printf("Unable to allocate in bufferMakeNames\n");
    return NULL;
  }
  memset(b,0,n*BUFNAMESIZE);
  va_start(l,n);
  for(i=0; i<n; i++){
    name=va_arg(l,char*);
    if(strlen(name)>BUFNAMESIZE){
      printf("Parameter %s is too long - should be max %d characters\n",name,BUFNAMESIZE);
      free(b);
      b=NULL;
      break;
    }else{
      strncpy(&b[i*BUFNAMESIZE],name,BUFNAMESIZE);
    }
  }
  va_end(l);
  if(b!=NULL && bufferCheckNames(n,b)!=0){
    free(b);
    b=NULL;
  }
  return b;
}


/**
   Returns the index of each param named in paramList, or -1 if not found, placed into array index, which should be of size n.  paramList must have n entries, each a null terminated string.
   pbuf is the parameter buffer.
   values is an array of void* with size n.  Each entry is then a pointer to the data, that can be cast as required according to pbuf->nbytes[index] and pbuf->dtype[index], for example, if these are 4 and i, you would do *(int*)(values[index])
   If these are 16 and f, you would do (float*)(values[index])
   paramList is if size n * BUFNAMESIZE (n*16 currently), with each set of BUFNAMESIZE bytes containing the name of the variable.
 */

int bufferGetIndex(paramBuf *pbuf,int n,char *paramList,int *index,void **values,char *dtype,int *nbytes){
  int i=0,j;
  char *buf=pbuf->buf;
  int nhdr=BUFNHDR(pbuf);
  int nfound=0;
  int s;
  //printf("BufferGetIndex %d %p %p\n",n,buf,paramList);
  wmemset(index,-1,n);//initialise to -1.
  while(i<nhdr && buf[16*i]!='\0'){//go through everything in the buffer
    //quick find version...
    /*
    s=0;
    e=n;
    p=n/2;
    found=0;
    while(found==0 && e-s>0){
      if((m=strncmp(&buf[i*16],paramList[p],16))==0){
	//match found
	index[j]=i;
	value[j]=BUFGETVALUE(pbuf,j);
	found=1;
      }else if(m<0){
	//search the first half
	e=p;
	p=(e+s)/2;
      }else{
	//search the second half
	s=p;
	p=(e+s)/2;
      }
      }*/
    for(j=0; j<n; j++){//compare it with our paramList.
      //if(strncmp(&buf[i*16],paramList[j],16)==0){
      s=strncmp(&buf[i*BUFNAMESIZE],&paramList[j*BUFNAMESIZE],16);
      if(s==0){//match found
	index[j]=i;
	values[j]=BUFGETVALUE(pbuf,i);
	dtype[j]=pbuf->dtype[i];
	nbytes[j]=pbuf->nbytes[i];
	nfound++;
	//printf("Found %16s\n",&paramList[j*BUFNAMESIZE]);
	break;
      }else if(s<0){
	break;
      }

      /*
      if(strncmp(&buf[i*BUFNAMESIZE],&paramList[j*BUFNAMESIZE],16)==0){
	printf("strncmp match %16s %16s\n",&buf[i*BUFNAMESIZE],&paramList[j*BUFNAMESIZE]);
      }else
	printf("N: %16s %16s\n",&buf[i*BUFNAMESIZE],&paramList[j*BUFNAMESIZE]);
      if((*((long long*)(&buf[i*BUFNAMESIZE])))==(*((long long*)(&paramList[j*BUFNAMESIZE])))){
	if((*((long long*)(&buf[i*BUFNAMESIZE+8])))==(*((long long*)(&paramList[j*BUFNAMESIZE+8])))){
	  //match found.
	  index[j]=i;
	  values[j]=BUFGETVALUE(pbuf,i);
	  dtype[j]=pbuf->dtype[i];
	  nbytes[j]=pbuf->nbytes[i];
	  nfound++;
	  printf("Found %16s\n",&paramList[j*BUFNAMESIZE]);
	  break;
	}else if((*((long long*)(&buf[i*BUFNAMESIZE+8])))<(*((long long*)(&paramList[j*BUFNAMESIZE+8])))){
	  //match not found, but we're past it in the alphabet now.  So this entry in the buffer isn't needed at the moment.
	  printf("Partial match %16s %16s\n",&buf[i*BUFNAMESIZE],&paramList[j*BUFNAMESIZE]);
	  break;
	  

	}
      }else if((*((long long*)(&buf[i*BUFNAMESIZE])))<(*((long long*)(&paramList[j*BUFNAMESIZE])))){
	//match not found, but we're past it in the alphabet now.  So this entry in the buffer isn't needed at the moment.
	printf("Break... %16s %16s %lld %lld %p %p\n",&buf[i*BUFNAMESIZE],&paramList[j*BUFNAMESIZE],(*((long long*)(&buf[i*BUFNAMESIZE]))),(*((long long*)(&paramList[j*BUFNAMESIZE]))),&buf[i*BUFNAMESIZE],&paramList[j*BUFNAMESIZE]);
	break;
      }
      */
    }
    i++;
  }
  //printf("bufferGetIndex done, nfound=%d/%d\n",nfound,n);
  return nfound;
}

paramBuf *bufferOpenFromData(char *arr,size_t arrsize){
  //arr contains a buffer.
  paramBuf *pbuf;
  int hdrsize;
  int nhdr;
  
  if((pbuf=calloc(sizeof(paramBuf),1))==NULL){
    printf("Failed to alloc paramBuf\n");
    return NULL;
  }
  pbuf->arr=arr;
  pbuf->hdr=(int*)arr;
  hdrsize=((int*)arr)[0];
  printf("Buffer headersize of %d bytes\n",hdrsize);
  pbuf->buf=&arr[hdrsize];
  pbuf->arrsize=arrsize;
  nhdr=BUFNHDR(pbuf);
  if(nhdr%sizeof(int)!=0)
    printf("Whoops - please use nhdr as a multiple of %d - bus error likely!\n",(int)sizeof(int));
  pbuf->condmutex=(pthread_mutex_t*)&arr[20];
  pbuf->cond=(pthread_cond_t*)&arr[20+sizeof(pthread_mutex_t)];
  pbuf->nbytes=(int*)(&(pbuf->buf[(BUFNAMESIZE+1+sizeof(int))*nhdr]));
  pbuf->dtype=(char*)(&(pbuf->buf[BUFNAMESIZE*nhdr]));
  pbuf->start=(int*)(&(pbuf->buf[(BUFNAMESIZE+1)*nhdr]));
  return pbuf;
}

paramBuf *bufferOpen(char *name){
  //name should be like "/rtcParam1" etc.
  //memory pointed to by buf should be valid.
  //struct stat st;
  char *tmp;
  char *arr;
  int fd;
  int hdrsize;
  int nhdr;
  struct stat st;
  paramBuf *pbuf;
  if((pbuf=calloc(sizeof(paramBuf),1))==NULL){
    printf("Failed to alloc paramBuf\n");
    return NULL;
  }

  if(asprintf(&tmp,"/dev/shm%s",name)==-1){
    printf("Failed to asprintf /dev/shm /name\n");
    free(pbuf);
    return NULL;
  }
  if(stat(tmp,&st)==-1){
    printf("Error statting %s: %s\n",name,strerror(errno));
    free(tmp);
    free(pbuf);
    return NULL;
  }
  free(tmp);
  printf("Opening buffer of size %lu bytes\n",st.st_size);
  if((fd=shm_open(name,O_RDWR,0))==-1){
    printf("shm_open failed for %s: %s\n",name,strerror(errno));
    free(pbuf);
    return NULL;
  }
  arr=(char*)mmap(0,st.st_size,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
  close(fd);
  if(arr==MAP_FAILED){
    printf("mmap failed %s:%s\n",name,strerror(errno));
    free(pbuf);
    return NULL;
  }
  pbuf->arr=arr;
  pbuf->hdr=(int*)arr;
  pbuf->arrsize=st.st_size;
  do{
    hdrsize=((int*)arr)[0];
    if(hdrsize==0){
      printf("Waiting to get hdrsize of parambuf\n");
      sleep(1);
    }
  }while(hdrsize==0);
  printf("Got buffer header size of %d bytes\n",hdrsize);
  pbuf->buf=&arr[hdrsize];
  //get the number of entries.
  do{
    nhdr=BUFNHDR(pbuf);
    if(nhdr==0){
      printf("Waiting to get nhdr\n");
      sleep(1);
    }
  }while(nhdr==0);
  printf("Got nhdr %d\n",nhdr);
  if(nhdr%sizeof(int)!=0){
    printf("Whoops - please use nhdr as a multiple of %d - bus error likely!\n",(int)sizeof(int));
  }
  pbuf->condmutex=(pthread_mutex_t*)&arr[20];
  pbuf->cond=(pthread_cond_t*)&arr[20+sizeof(pthread_mutex_t)];
  pbuf->nbytes=(int*)(&(pbuf->buf[(BUFNAMESIZE+1+sizeof(int))*nhdr]));
  pbuf->dtype=(char*)(&(pbuf->buf[BUFNAMESIZE*nhdr]));
  pbuf->start=(int*)(&(pbuf->buf[(BUFNAMESIZE+1)*nhdr]));
  //pbuf->lcomment=(int*)(&(pbuf->buf[(BUFNAMESIZE+1+4+4+4+24)*nhdr]));
  //pbuf->labels=(char*)(header);//16 bytes per label, nhdr labels.
  return pbuf;
}

void bufferClose(paramBuf *pbuf){
  if(pbuf!=NULL){
    if(pbuf->arr!=NULL)
      munmap(pbuf->arr,pbuf->arrsize);
    free(pbuf);
  }
}

int bufferGetNEntries(paramBuf *pbuf){
  //Get the number of entries...
  int i=0;
  char *labels=pbuf->buf;
  while(i<BUFNHDR(pbuf) && labels[i*BUFNAMESIZE]!=0)
    i+=1;
  return i;
}

unsigned long bufferGetMem(paramBuf *pbuf,int includeArrHdrSize){
  //Finds how much space the buffer is using from start to finish (i.e. including any free space inbetween)
  //usually, includearrHdrSize will be 0 (i.e. that is the python default).
  int n,i;
  unsigned long mem;
  unsigned long offset;
  n=bufferGetNEntries(pbuf);
  mem=((int)(BUFNHDR(pbuf)*BUFHDRSIZE+BUFALIGN-1)/BUFALIGN)*BUFALIGN;
  for(i=0;i<n;i++){
    offset=pbuf->start[i]+pbuf->nbytes[i]+BUFLCOMMENT(pbuf,i);
    mem=mem>offset?mem:offset;
  }
  if(includeArrHdrSize)
    mem+=BUFARRHDRSIZE(pbuf);
  return mem;
}

int bufferGetInactive(paramBuf *pbuf[2]){
  //Return the buffer index that can be safely written to
  int rt=-1;
  if(pbuf[0]==NULL || pbuf[1]==NULL){
    printf("No buffers in bufferGetInactive\n");
    rt=-1;
  }else{
    if((BUFFLAG(pbuf[0])&0x1)==0){
      rt=0;
    }else if((BUFFLAG(pbuf[1])&0x1)==0){
      rt=1;
    }else{
      printf("No inactive buffer found\n");
      rt=-1;
    }
  }
  printf("inactive buffer %d\n",rt);
  return rt;
}
int bufferGetActive(paramBuf *pbuf[2]){
  //Return the buffer index that can be safely written to
  int rt=-1;
  if(pbuf[0]==NULL || pbuf[1]==NULL){
    printf("No buffers in bufferGetInactive\n");
    rt=-1;
  }else{
    if((BUFFLAG(pbuf[0])&0x1)==1){
      rt=0;
    }else if((BUFFLAG(pbuf[1])&0x1)==1){
      rt=1;
    }else{
      printf("No active buffer found\n");
      rt=-1;
    }
  }
  return rt;
}
int bufferNewEntry(paramBuf *pbuf,char *name){
  //Get the index for a new entry
  int indx,l;
  indx=bufferGetNEntries(pbuf);
  if(indx>=BUFNHDR(pbuf)){
    return -1;
  }
  BUFNBYTES(pbuf,indx)=0;
  l=strnlen(name,16);
  memcpy(&pbuf->buf[indx*BUFNAMESIZE],name,l);
  if(l<16)
    memset(&pbuf->buf[indx*BUFNAMESIZE+l],0,16-l);
  return indx;
}

size_t bufferGetSpace(paramBuf *pbuf,size_t bytes){
  //find space in the buffer for this many bytes...
  size_t s,e;
  int found,nEntries,i;
  if(bytes==0)
    return -1;
  s=((BUFNHDR(pbuf)*BUFHDRSIZE+BUFALIGN-1)/BUFALIGN)*BUFALIGN;
  e=s+bytes;
  found=0;
  nEntries=bufferGetNEntries(pbuf);
  while(found==0 && e<=(pbuf->arrsize-BUFARRHDRSIZE(pbuf))){
    found=1;
    for(i=0;i<nEntries;i++){
      if(BUFNBYTES(pbuf,i)+BUFLCOMMENT(pbuf,i)>0){
	if((BUFSTART(pbuf,i)<=s && BUFSTART(pbuf,i)+BUFNBYTES(pbuf,i)+BUFLCOMMENT(pbuf,i)>s) || (BUFSTART(pbuf,i)<e && BUFSTART(pbuf,i)+BUFNBYTES(pbuf,i)+BUFLCOMMENT(pbuf,i)>=e) || (s<BUFSTART(pbuf,i) && e>BUFSTART(pbuf,i))){
	  found=0;
	  s=((BUFSTART(pbuf,i)+BUFNBYTES(pbuf,i)+BUFLCOMMENT(pbuf,i)+BUFALIGN-1)/BUFALIGN)*BUFALIGN;
	  e=s+bytes;
	}
      }
    }
  }
  if(found==0)
    return -1;
  return s;
}

int bufferSetIgnoringLock(paramBuf *pbuf,char *name, bufferVal *value){
  int rt=0,indx=-1;
  char *bufferNames;
  void *values;
  char dtype;
  int nbytes;
  size_t start;
  int itemsize;
  if((bufferNames=bufferMakeNames(1,name))==NULL){
    printf("Error bufferMakeNames\n");
    rt=1;
  }else{
    bufferGetIndex(pbuf,1,bufferNames,&indx,&values,&dtype,&nbytes);
  }
  if(indx<0){//insert a new value
    indx=bufferNewEntry(pbuf,name);
    if(indx==-1){
      printf("Error creating new parameter %s - no space remaining\n",name);
      rt=2;
    }
  }
  if(indx>=0){
    //We now have a buffer entry, so check there is enough space, and then insert it.
    if(BUFNBYTES(pbuf,indx)+BUFLCOMMENT(pbuf,indx)<value->size){//no space at current location.
      start=bufferGetSpace(pbuf,value->size);
      if(start==-1){
	printf("Error - no space left in buffer\n");
	rt=3;
      }else{
	//copy the data to location at start.
	memcpy(&pbuf->buf[start],value->data,value->size);
	BUFSTART(pbuf,indx)=start;
      }
    }else{//copy into current lcoation
      memcpy(&pbuf->buf[BUFSTART(pbuf,indx)],value->data,value->size);
    }
    if(rt==0){
      switch(value->dtype){
      case 'd':
	itemsize=8;
	break;
      case 'i':
      case 'I':
      case 'f':
	itemsize=4;
	break;
      case 'h':
      case 'H':
	itemsize=2;
	break;
      case 'b':
      case 'n':
      case 's':
	itemsize=1;
	break;
      default:
	printf("Error - undefined itemsize for type %c in buffer.c - please define\n",value->dtype);
	itemsize=1;
      }
      BUFLCOMMENT(pbuf,indx)=0;
      BUFNBYTES(pbuf,indx)=value->size;
      BUFNDIM(pbuf,indx)=1;
      BUFDIM(pbuf,indx)[0]=value->size/itemsize;
      BUFDTYPE(pbuf,indx)=value->dtype;
    }
  }
  return rt;
}
int bufferSet(paramBuf *pbuf,char *name, bufferVal *value){
  int i=0,err,rt=0;
  struct timespec abstime;
  struct timeval t1;
  pthread_mutex_lock(pbuf->condmutex);
  while((BUFFLAG(pbuf)&1)==1 && i<50){//currently frozen - wait for unblock.
    gettimeofday(&t1,NULL);
    abstime.tv_sec=t1.tv_sec+1;//add 1 second.
    abstime.tv_nsec=t1.tv_usec*1000;
    err=pthread_cond_timedwait(pbuf->cond,pbuf->condmutex,&abstime);
    if(err==ETIMEDOUT){
      printf("Timeout waiting for buffer switch to complete (%d/50) in bufferSet(%s)\n",i,name);
    }else if(err!=0){
      printf("pthread_cond_timedwait failed in darccontrolc\n");
    }
    i++;
    if(i==50){
      printf("Error waiting for buffer to be unlocked\n");
      rt=1;
    }
  }
  pthread_mutex_unlock(pbuf->condmutex);
  if(rt==0)
    rt=bufferSetIgnoringLock(pbuf,name,value);
  return rt;
}



bufferVal *bufferGet(paramBuf *pbuf,char *name){
  //Note, returned memory should be free'd when finished with.
  int err=0;
  //int n=0;
  //int namelen;
  //char param[BUFNAMESIZE+1];
  //int bufno;
  //paramBuf *b;
  char *bufferNames;
  int indx=-1;
  void *values;
  char dtype;
  int nbytes;
  //char data[BUFNAMESIZE+sizeof(int)*9];
  bufferVal *val=NULL;
  if((bufferNames=bufferMakeNames(1,name))==NULL){
    printf("Error bufferMakeNames\n");
    err=1;
  }else{
    bufferGetIndex(pbuf,1,bufferNames,&indx,&values,&dtype,&nbytes);
  }
  if(indx==-1){//not found
    if(err==0)
      err=1;
  }else if(err==0){
    if((val=calloc(sizeof(bufferVal),1))==NULL){
      printf("Error allocing bufferVal in bufferGet\n");
      err=2;
    }else{
      val->size=nbytes;
      val->dtype=dtype;
      val->data=values;
    }
  }
  return val;
}
int bufferInit(paramBuf *pbuf,char *fitsfilename){
  //Initialise the buffer from a FITS file.
  printf("todo - bufferInit (implemented in darccontrolc)\n");
  return 0;
}

