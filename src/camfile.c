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
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rtccamera.h"
#include <time.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>

typedef struct{
  char *ffname;
  //int transferRequired;
  int frameno;
  unsigned short *imgdata;
  unsigned short *membuf;//used if copying file to RAM.
  int npxls;
  //int streaming;
  FILE *fd;
  int nframes;
  int hdrsize;
  int *axisarr;
  unsigned int *userFrameNo;
  int ncam;
  int loadIntoMem;
  //pthread_mutex_t m;
}CamStruct;


void dofree(CamStruct *camstr){
  if(camstr!=NULL){
    if(camstr->axisarr!=NULL)
      free(camstr->axisarr);
    if(camstr->ffname!=NULL)
      free(camstr->ffname);
    if(camstr->membuf!=NULL)
      free(camstr->membuf);
    //pthread_mutex_destroy(&camstr->m);
    free(camstr);
  }
}
/**
   Open a camera of type name.  Args are passed in a int32 array of size n, which can be cast if necessary.  Any state data is returned in camHandle, which should be NULL if an error arises.
   pxlbuf is the array that should hold the data. The library is free to use the user provided version, or use its own version as necessary (ie a pointer to physical memory or whatever).  It is of size npxls*sizeof(short).
   ncam is number of cameras, which is the length of arrays pxlx and pxly, which contain the dimensions for each camera.
   Name is used if a library can support more than one camera.

   This library opens a FITS file (assumes 2880 header size), and uses this for the data.
   args here contains filename
*/

int camOpen(char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **camHandle,int nthreads,unsigned int thisiter,unsigned int **frameno,int *framenoSize,int npxls,int ncam,int *pxlx,int* pxly){
  CamStruct *camstr;
  int naxis=0;
  char buf[80];
  int end;
  int axis;
  int i,framePixels;
  unsigned short *tmps;
  //unsigned short *pxlbuf=arr->pxlbufs;
  printf("Initialising camera %s\n",name);
  if((*camHandle=malloc(sizeof(CamStruct)))==NULL){
    printf("Couldn't malloc camera handle\n");
    return 1;
  }
  memset(*camHandle,0,sizeof(CamStruct));
  camstr=(CamStruct*)*camHandle;
  if(arr->pxlbuftype!='H' || arr->pxlbufsSize!=sizeof(unsigned short)*npxls){
    //need to resize the pxlbufs...
    arr->pxlbufsSize=sizeof(unsigned short)*npxls;
    arr->pxlbuftype='H';
    arr->pxlbufelsize=sizeof(unsigned short);
    tmps=realloc(arr->pxlbufs,arr->pxlbufsSize);
    if(tmps==NULL){
      if(arr->pxlbufs!=NULL)
	free(arr->pxlbufs);
      printf("pxlbuf malloc error in camfile.\n");
      arr->pxlbufsSize=0;
      free(*camHandle);
      *camHandle=NULL;
      return 1;
    }
    arr->pxlbufs=tmps;
    memset(arr->pxlbufs,0,arr->pxlbufsSize);
  }
  camstr->imgdata=arr->pxlbufs;
  if(*framenoSize<ncam){
    if(*frameno!=NULL)free(*frameno);
    *frameno=malloc(sizeof(int)*ncam);
    if(*frameno==NULL){
      *framenoSize=0;
      printf("Unable to malloc camframeno\n");
    }else{
      *framenoSize=ncam;
    }
  }
  camstr->userFrameNo=*frameno;
  camstr->npxls=npxls;//*pxlx * *pxly;
  camstr->ncam=ncam;
  camstr->frameno=-1;
  for(i=0; i<ncam; i++){
    camstr->userFrameNo[i]=-1;
  }
  /*if(pthread_mutex_init(&camstr->m,NULL)!=0){
    printf("Error initialising mutex variable\n");
    dofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  printf("done mutex\n");
  */
  camstr->ffname=strndup((char*)args,sizeof(int)*n);
  if(strlen(camstr->ffname)<sizeof(int)*(n-1)){
    //there may be an extra flag to tell us to load file to memory, and read from there rather from disk.
    if(args[n-1]){
      camstr->loadIntoMem=1;
    }
  }
  printf("Opening file '%s'\n",camstr->ffname);
  if((camstr->fd=fopen(camstr->ffname,"r"))==NULL){
    printf("Failed to open file\n");
    dofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  printf("File opened\n");
  end=0;
  while(end==0){
    if(fread(buf,1,80,camstr->fd)!=80){
      printf("Failed to read file\n");
      end=-1;
    }
    if(strncmp(buf,"NAXIS   ",8)==0){
      naxis=atoi(&buf[10]);
      camstr->axisarr=malloc(sizeof(int)*naxis);
      memset(camstr->axisarr,0,sizeof(int)*naxis);
    }else if(strncmp(buf,"BITPIX  ",8)==0){
      if(atoi(&buf[10])!=16){
	printf("FITS file should be 16 bit integer\n");
	end=-1;
      }
    }else if(strncmp(buf,"NAXIS",5)==0){
      if(buf[5]!=' '){
	axis=atoi(&buf[5])-1;
	camstr->axisarr[axis]=atoi(&buf[10]);
      }
    }else if(strncmp(buf,"END",3)==0){
      end=1;
    }
  }
  if(end==1){
    printf("Got %dD FITS array with dimensions",naxis);
    for(i=0; i<naxis; i++)
      printf(" %d",camstr->axisarr[i]);
    printf(".\n");
    framePixels=1;
    for(i=0; i<naxis-1; i++)
      framePixels*=camstr->axisarr[i];
    if(framePixels!=npxls){
      printf("Image size doesn't agree with the expected value\n");
      end=-1;
    }
    camstr->nframes=camstr->axisarr[naxis-1];
    camstr->hdrsize=(int)((ftell(camstr->fd)+2880-1)/2880)*2880;
    fseek(camstr->fd,camstr->hdrsize,SEEK_SET);
  }
  if(end==-1){//failed...
    dofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  if(camstr->loadIntoMem){
    printf("Loading file into memory\n");
    if((camstr->membuf=malloc(sizeof(unsigned short)*npxls*camstr->nframes))==NULL){
      printf("Unable to load camera image file %s into memory\n",camstr->ffname);
      dofree(camstr);
      *camHandle=NULL;
      return 1;
    }else{
      //now load the data
      if(fread(camstr->membuf,1,camstr->nframes*sizeof(unsigned short)*(camstr->npxls),camstr->fd)!=camstr->nframes*sizeof(unsigned short)*camstr->npxls){
	printf("Error reading FITS file data\n");
	dofree(camstr);
	*camHandle=NULL;
	return 1;
      }else{
	char *cd;
	char tmp;
	//Now byteswap the data... (fits format is big endian)
	cd=(char*)camstr->membuf;
	for(i=0; i<camstr->npxls*2*camstr->nframes; i+=2){
	  tmp=cd[i];
	  cd[i]=cd[i+1];
	  cd[i+1]=tmp;
	}
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
   Called when we're starting processing the next frame.  This doesn't actually wait for any pixels.
*/
int camNewFrameSync(void *camHandle,unsigned int thisiter,double starttime){
  //printf("camNewFrame\n");
  CamStruct *camstr;
  int i;
  char *cd;
  char tmp;

  camstr=(CamStruct*)camHandle;
  if(camHandle==NULL){// || camstr->streaming==0){
    //printf("called camNewFrame with camHandle==NULL\n");
    return 1;
  }
  //printf("camNewFrame\n");
  camstr->frameno++;
  if(camstr->frameno>=camstr->nframes){
    camstr->frameno=0;
    if(camstr->loadIntoMem==0)
      fseek(camstr->fd,camstr->hdrsize,SEEK_SET);
  }
  //printf("New frame %d\n",(int)ftell(camstr->fd));
  if(camstr->loadIntoMem){
    memcpy(camstr->imgdata,&camstr->membuf[camstr->frameno*camstr->npxls],sizeof(unsigned short)*camstr->npxls);
  }else{//load from disk
    if(fread(camstr->imgdata,1,sizeof(unsigned short)*(camstr->npxls),camstr->fd)!=sizeof(unsigned short)*camstr->npxls){
      printf("Error reading FITS file data\n");
      return 1;
    }
    cd=(char*)camstr->imgdata;
    //do the byteswap (fits format is big endian).
    for(i=0; i<camstr->npxls*2; i+=2){
      tmp=cd[i];
      cd[i]=cd[i+1];
      cd[i+1]=tmp;
    }
  }
  for(i=0; i<camstr->ncam; i++){
    camstr->userFrameNo[i]++;//=camstr->frameno;
    if((camstr->userFrameNo[i]%camstr->nframes)!=camstr->frameno){
      printf("camfile frameno error...[%d] %d %d %d\n",i,camstr->frameno,camstr->userFrameNo[i],camstr->userFrameNo[i]%camstr->nframes);
    }
  }
  return 0;
}

/**
   Wait for the next n pixels of the current frame to arrive.
*/
/*
int camWaitPixels(int n,int cam,void *camHandle){
  //printf("camWaitPixels %d, camera %d\n",n,cam);
  //For andor, we actually have to wait for the whole frame...
  CamStruct *camstr=(CamStruct*)camHandle;
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
  //pthread_mutex_lock(&camstr->m);
  //pthread_mutex_unlock(&camstr->m);

  return 0;
  }*/
