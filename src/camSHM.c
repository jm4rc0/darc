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
   The code here is used to create a shared object library, which can then be swapped around depending on which cameras you have in use, ie you simple rename the camera file you want to camera.so (or better, change the soft link), and restart the coremain.

The library is written for a specific camera configuration - ie in multiple camera situations, the library is written to handle multiple cameras, not a single camera many times.

This library is for a SHM interface.  Pixels are copied to SHM, and then a condition variable is signalled, at which point, this library wakes up and processes the data.

The shared memory is double buffered.  

Condition variable for each buffer signalled when new pixels are ready in this buffer - which may or may not be the full frame.  A buffer counter stores the number of pixels received for each buffer.  

When this library has finished reading a buffer, it sets the counter to zero.

Locking: A lock on the counter.  The associated condition variable is signalled whenever the lock number changes (both when set to zero by darc, and when incremented by the shm writer.

There is also the option to have a single buffer operation, for simpler systems.



SHM buffer makeup:
4 bytes total buffer size (including these four bytes)
ncam (1 byte)
dtype (1 byte) (0==uint16, 1==float32)
alignment (1 byte) - number of bytes that counter and data are aligned to.
inter-camera alignment (1 byte)
npxlx (4*ncam bytes)
npxly (4*ncam bytes)
Currently Active (writing) buffer number (0 or 1) (4 bytes)
mutex (sizeof(mutex)*NBUF)
cond (sizeof(cond)*NBUF)
Alignment to N byte boundary
counter (4*ncam*NBUF bytes)
data (with each camera inter-N byte aligned, all cameras for given buffer together, each buffer N byte aligned)

The header info must not change while the darc camera object is open.

*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rtccamera.h"
#include <time.h>
#include <unistd.h>
#include <sys/types.h>

typedef struct{
  //char *ffname;
  char *host;
  int port;
  int transferRequired;
  int frameno;
  unsigned short *imgdata;
  float *fimgdata;
  int npxls;
  //int streaming;
  //FILE *fd;
  int sd;//the socket descriptor.
  int sockOpen;
  int nframes;
  int hdrsize;
  unsigned int header[2];
  int *axisarr;
  unsigned int *userFrameNo;
  int asfloat;
}CamStruct;

/**
   Find out if this SO library supports your camera.

*/

void dofree(CamStruct *camstr){
  if(camstr!=NULL){
    if(camstr->sockOpen){
      close(camstr->sd);
    }
    if(camstr->host)
      free(camstr->host);
    free(camstr);
  }
}
/**
   Open a camera of type name.  Args are passed in a int32 array of size n, which can be cast if necessary.  Any state data is returned in camHandle, which should be NULL if an error arises.
   pxlbuf is the array that should hold the data. The library is free to use the user provided version, or use its own version as necessary (ie a pointer to physical memory or whatever).  It is of size npxls*sizeof(short).
   ncam is number of cameras, which is the length of arrays pxlx and pxly, which contain the dimensions for each camera.
   Name is used if a library can support more than one camera.

*/

int camOpen(char *name,int narg,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **camHandle,int nthreads,unsigned int thisiter,unsigned int **frameno,int *framenoSize,int npxls,int ncam,int *pxlx,int* pxly){ 
  CamStruct *camstr;
  struct sockaddr_in sin;
  struct hostent *host;
  unsigned short *tmps;
  printf("Initialising camera %s\n",name);
  if(narg<3){
    printf("Error - need arguments asfloat,create,shmname(null terminated string)\n");
    return 1;
  }
  if(ncam<1){
    printf("Error - expecting at least 1 camera\n");
    return 1;
  }
  if((*camHandle=malloc(sizeof(CamStruct)))==NULL){
    printf("Couldn't malloc camera handle\n");
    return 1;
  }
  memset(*camHandle,0,sizeof(CamStruct));
  camstr=(CamStruct*)*camHandle;
  camstr->asfloat=args[0];
  if(camstr->asfloat){
    if(arr->pxlbuftype!='f' || arr->pxlbufsSize!=sizeof(float)*npxls){
      //need to resize the pxlbufs...
      arr->pxlbufsSize=sizeof(float)*npxls;
      arr->pxlbuftype='f';
      arr->pxlbufelsize=sizeof(float);
      tmps=realloc(arr->pxlbufs,arr->pxlbufsSize);
      if(tmps==NULL){
	if(arr->pxlbufs!=NULL)
	  free(arr->pxlbufs);
	printf("pxlbuf malloc error in camSHM.\n");
	arr->pxlbufsSize=0;
	free(*camHandle);
	*camHandle=NULL;
	return 1;
      }
      arr->pxlbufs=tmps;
      memset(arr->pxlbufs,0,arr->pxlbufsSize);
    }
    camstr->fimgdata=arr->pxlbufs;
    camstr->imgdata=arr->pxlbufs;
  }else{//uint16
    if(arr->pxlbuftype!='H' || arr->pxlbufsSize!=sizeof(unsigned short)*npxls){
      //need to resize the pxlbufs...
      arr->pxlbufsSize=sizeof(unsigned short)*npxls;
      arr->pxlbuftype='H';
      arr->pxlbufelsize=sizeof(unsigned short);
      tmps=realloc(arr->pxlbufs,arr->pxlbufsSize);
      if(tmps==NULL){
	if(arr->pxlbufs!=NULL)
	  free(arr->pxlbufs);
	printf("pxlbuf malloc error in camSHM.\n");
	arr->pxlbufsSize=0;
	dofree(*camHandle);
	*camHandle=NULL;
	return 1;
      }
      arr->pxlbufs=tmps;
      memset(arr->pxlbufs,0,arr->pxlbufsSize);
    }
    camstr->imgdata=arr->pxlbufs;
    camstr->fimgdata=arr->pxlbufs;
  }
  if(*framenoSize<ncam){
    if(*frameno!=NULL)free(*frameno);
    if((*frameno=malloc(sizeof(unsigned int)*ncam))==NULL){
      printf("Unable to malloc camframeno\n");
      *framenoSize=0;
    }else{
      *framenoSize=ncam;
    }
  }
  camstr->userFrameNo=*frameno;
  camstr->npxls=npxls;//*pxlx * *pxly;
  camstr->create=args[1];
  camstr->shmname=strndup((char*)&args[2],(narg-2)*sizeof(int));
  printf("Got shmname %s\n",camstr->shmname);
  //now open the shm, creating if necessary.
  //Then check shm is of correct size.
  if((camstr->fd=shm_open(camstr->shmname,O_RDWR|(camstr->create*(O_CREAT|O_EXCL)),0777))<0){
    printf("Failed to open shm %s: %s\n",camstr->shmname,strerror(errno));
    dofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  if(camstr->create){//make it correct size and fill in header details.


  }else{//check that it is correct size.
    struct stat st;
    st.st_size=0;
    i=0;
    while(st.st_size==0 && i<20){
      if(fstat(fd,&st)!=0){
	printf("Error statting %s: %s\n",camstr->shmname,strerror(errno));
	close(fd);
	dofree(camstr);
	*camHandle=NULL;
	return 1;
      }
      if(st.st_size==0)
	usleep(100000);
      i++;
    }
    buf=NULL;
    i=0;
    buf=(int*)mmap(0,st.st_size,PROT_READ,MAP_SHARED,fd,0);
    if(buf==MAP_FAILED){
      printf("Failed to map shm %s...\n",camstr->shmname);
      close(fd);
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }
    close(fd);
    i=0;
    //wait for buffer to finish initialising
    while(*((int*)buf)!=st.st_size && i<2){
      printf("Waiting for %s buffer contents\n",camstr->shmname);
      if(*((int*)buf)!=st.st_size)
	usleep(100000);
      i++;
    }
    if(st.st_size!=*((int*)buf)){
      printf("Buffer %s failed to initialise\n",camstr->shmname);
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }
    //now check that we understand the size.
    size=4+1+1+1+1+4;//must be at least this big for initial checking.
    if(size.st_size<size){
      printf("Size of shm %s is wrong\n",camstr->shmname);
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }
    nc=(int)((char*)buf)[4];
    if(nc!=ncam){
      printf("Wrong number of cameras in shared memory %s\n",camstr->shmname);
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }
    asfloat=(int)((char*)buf)[5];
    align=(int)((char*)buf)[6];
    interalign=(int)((char*)buf)[7];
    //now compute the more full size... have to be careful with the alignment.
    size+=4*ncam+4*ncam+sizeof(pthread_mutex_t)*NBUF+sizeof(pthread_cond_t)*NBUF;
    npx=(int*)(&buf[8]);
    npy=(int*)(&buf[12]);
    for(i=0;i<ncam;i++){
      if(npx[i]!=npxlx[i] || npy[i]!=npxly[i]){
	printf("npxlx or npxly wrong in shared memory.  For cam %d is %dx%d, expecting %dx%d\n",i,npx[i],npy[i],npxlx[i],npxly[i]);
	dofree(camstr);
	*camHandle=NULL;
	return 1;
      }
    }
    if((size%align)!=0)
      size+=align-size%align;
    size+=4*NBUF*ncam;
    for(i=0;i<NBUF;i++){
      if((size%align)!=0)
        size+=align-size%align;
      for(j=0;j<ncam;j++){
        if((size%interalign)!=0)
	  size+=interalign-size%interalign;
        size+=npxlx[j]*npxly[j]*(2+asfloat*2);
      }
    }
    if(size!=st.st_size){
      printf("Wrong size of buffer - expecting it to be %d, actually %d\n",(int)size,(int)st.st_size);
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }
    camstr->marr=(pthread_mutex_t*)(&buf[12+8*ncam]);
    camstr->condarr=(pthread_cond_t*)(&buf[12+8*ncam+sizeof(pthread_mutex_t)*NBUF]);
    if((camstr->dataarr=calloc(sizeof(char*)*NBUF,ncam))==NULL){
      printf("Unable to allocate data pointer arrays\n");
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }

    offset=12+8*ncam+NBUF*sizeof(mutex_t)+NBUF*sizeof(pthread_cond_t)
      if((offset%align)!=0)
	offset+=align-offset%align;
    camstr->counter=(int*)(&buf[offset]);
    offset+=4*ncam*NBUF;
    for(i=0;i<NBUF;i++){
      if((offset%align)!=0)
	offset+=align-offset%align;
      for(j=0;j<ncam;j++){
	if((offset%interalign)!=0)
	  offset+=interalign-offset%interalign;
	camstr->dataarr[i*ncam+j]=(char*)(&buf[offset]);
	offset+=npxlx[j]*npxly[j]*(2+camstr->asfloat*2);
      }
    }

  }
  return 0;
}


  /**
   Close a camera of type name.  Args are passed in the int32 array of size n, and state data is in camHandle, which should be freed and set to NULL before returning.
*/
int camClose(void **camHandle){
  CamStruct *camstr;
  printf("Closing camera\n");
  if(*camHandle==NULL)
    return 1;
  camstr=(CamStruct*)*camHandle;
  dofree(camstr);
  *camHandle=NULL;
  printf("Camera closed\n");
  return 0;
}
/**
   New parameters ready
*/
//int camNewParam(void *camHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr){
//  return 0;
//}
/**
   Start the camera framing, using the args and camera handle data.
*/
/*int camStartFraming(int n,int *args,void *camHandle){
  CamStruct *camstr;
  if(camHandle==NULL){
    printf("called camStartFraming with camHandle==NULL\n");
    return 1;
  }
  camstr=(CamStruct*)camHandle;
  camstr->streaming=1;
  printf("Framing camera\n");
  return 0;
}
int camStopFraming(void *camHandle){
  CamStruct *camstr;
  if(camHandle==NULL){
    printf("called camStopFraming with camHandle==NULL\n");
    return 1;
  }
  camstr=(CamStruct*)camHandle;
  camstr->streaming=0;
  printf("Stopping framing\n");
  return 0;
}*/

/**
   Can be called to get the latest iamge taken by the camera
*/
/*int camGetLatest(void *camHandle){
  if(camHandle==NULL){
    printf("called camGetLatest with camHandle==NULL\n");
    return 1;
  }
  printf("Getting latest frame\n");
  return 0;
  }*/

/**
   Called when we're starting processing the next frame.  This doesn't actually wait for any pixels.
*/
int camNewFrameSync(void *camHandle,unsigned int thisiter,double starttime){
  //printf("camNewFrame\n");
  CamStruct *camstr;
  int tmp;
  int err=0;
  int retval;
  fd_set rfds;
  struct timeval tv;
  int ns,nr;
  camstr=(CamStruct*)camHandle;
  if(camHandle==NULL){// || camstr->streaming==0){
    //printf("called camNewFrame with camHandle==NULL\n");
    return 1;
  }
  //printf("camNewFrame\n");
  //camstr->frameno++;
  camstr->transferRequired=1;
  //printf("New frame %d\n",(int)ftell(camstr->fd));
  if(camstr->transferRequired){
    camstr->transferRequired=0;

    FD_ZERO(&rfds);
    FD_SET(camstr->sd, &rfds);
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    retval=select(camstr->sd+1,&rfds,NULL,NULL,&tv);
    if(retval==-1){
      printf("Error in select\n");
      err=1;
    }else if(retval==0){
      printf("Timeout while waiting for socket camera\n");
      err=1;
    }else{
      //first read the 8 byte header...
      ns=sizeof(unsigned int)*2;
      nr=0;
      while(nr<ns && err==0){
	tmp=read(camstr->sd,&camstr->header[nr],ns-nr);
	if(tmp==-1){
	  printf("Error reading cam header\n");
	  err=1;
	  nr=ns;
	}else if(tmp==0){
	  printf("Cam socket closed?\n");
	  err=1;
	  nr=ns;
	}else{
	  nr+=tmp;
	}
      }
      if((camstr->header[0]>>28)!=(0x5<<camstr->asfloat) || (camstr->header[0]&0xfffffff)!=camstr->npxls){
	printf("Unexpected header or wrong number of pixels, %#x %d     \r",camstr->header[0],camstr->header[1]);
	err=1;
      }
      if(err==0){
	camstr->frameno=camstr->header[1];
	//now read the data
	ns=(camstr->asfloat?sizeof(float):sizeof(unsigned short))*camstr->npxls;
	nr=0;
	while(nr<ns && err==0){
	  tmp=read(camstr->sd,&camstr->imgdata[nr],ns-nr);
	  if(tmp==-1){
	    printf("Error reading cam socket\n");
	    err=1;
	    nr=ns;
	  }else if(tmp==0){
	    printf("cam socket closed?\n");
	    err=1;
	    nr=ns;
	  }else{
	    nr+=tmp;
	  }
	}
      }
    }

    *camstr->userFrameNo=camstr->frameno;

  }


  return 0;
}

/**
   Wait for the next n pixels of the current frame to arrive.
   Note - this is a lazy implementation - the camNewFrameSync waits for all the data to arrive... so increases the latency.  If you intend to use this in a production environment, should rewrite this so that the socket reading is done in camWaitPixels, waiting only for the required pixels to arrive, before continuing.
*/
/*
int camWaitPixels(int n,int cam,void *camHandle){
  //printf("camWaitPixels %d, camera %d\n",n,cam);
  //For andor, we actually have to wait for the whole frame...
  CamStruct *camstr=(CamStruct*)camHandle;
  int err=0;
  //static struct timeval t1;
  //struct timeval t2;
  //struct timeval t3;
  if(camHandle==NULL){// || camstr->streaming==0){
    //printf("called camWaitPixels with camHandle==NULL\n");
    return 1;
  }
  if(n<0)
    n=0;
  if(n>camstr->npxls)
    n=camstr->npxls;
  //printf("camWaitPixels\n");
  return err;
}
*/
