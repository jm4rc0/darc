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
  int transferRequired;
  int frameno;
  short *imgdata;
  int npxls;
  int streaming;
  FILE *fd;
  int nframes;
  int hdrsize;
  int *axisarr;
  int *userFrameNo;
  int ncam;
  pthread_mutex_t m;
}CamStruct;

/**
   Find out if this SO library supports your camera.

*/
int camQuery(char *name){
  //Note, the strings aren't necessarily null terminated...
  int rtval=0;
#ifdef OLD
  rtval=(strcmp(name,"camfile")!=0);
#endif
  return rtval;
}

void dofree(CamStruct *camstr){
  if(camstr!=NULL){
    if(camstr->axisarr!=NULL)
      free(camstr->axisarr);
    if(camstr->ffname!=NULL)
      free(camstr->ffname);
    pthread_mutex_destroy(&camstr->m);
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

int camOpen(char *name,int n,int *args,char *parambuf,circBuf *rtcErrorBuf,char *prefix,void **camHandle,int npxls,short *pxlbuf,int ncam,int *pxlx,int* pxly,int* frameno){
  CamStruct *camstr;
  int naxis=0;
  char buf[80];
  int end;
  int axis;
  int i,framePixels;
  printf("Initialising camera %s\n",name);
  if(camQuery(name)){
    printf("Wrong camera type\n");
    return 1;
  }
  if((*camHandle=malloc(sizeof(CamStruct)))==NULL){
    printf("Couldn't malloc camera handle\n");
    return 1;
  }
  memset(*camHandle,0,sizeof(CamStruct));
  camstr=(CamStruct*)*camHandle;
  camstr->imgdata=pxlbuf;
  camstr->userFrameNo=frameno;
  camstr->npxls=npxls;//*pxlx * *pxly;
  camstr->ncam=ncam;
  for(i=0; i<ncam; i++){
    camstr->userFrameNo[i]=0;
  }
  if(pthread_mutex_init(&camstr->m,NULL)!=0){
    printf("Error initialising mutex variable\n");
    dofree(camstr);
    *camHandle=NULL;
    return 1;
  }
  printf("done mutex\n");

  camstr->ffname=strndup((char*)args,sizeof(int)*n);
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
int camNewParam(void *camHandle,char *buf,unsigned int frameno){
  return 0;
}
/**
   Start the camera framing, using the args and camera handle data.
*/
int camStartFraming(int n,int *args,void *camHandle){
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
/**
   Stop the camera framing
*/
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
}

/**
   Can be called to get the latest iamge taken by the camera
*/
int camGetLatest(void *camHandle){
  if(camHandle==NULL){
    printf("called camGetLatest with camHandle==NULL\n");
    return 1;
  }
  printf("Getting latest frame\n");
  return 0;
}

/**
   Called when we're starting processing the next frame.  This doesn't actually wait for any pixels.
*/
int camNewFrame(void *camHandle){
  //printf("camNewFrame\n");
  CamStruct *camstr;
  camstr=(CamStruct*)camHandle;
  if(camHandle==NULL || camstr->streaming==0){
    //printf("called camNewFrame with camHandle==NULL\n");
    return 1;
  }
  //printf("camNewFrame\n");
  camstr->frameno++;
  camstr->transferRequired=1;
  if(camstr->frameno>=camstr->nframes){
    camstr->frameno=0;
    fseek(camstr->fd,camstr->hdrsize,SEEK_SET);
  }
  //printf("New frame %d\n",(int)ftell(camstr->fd));
  return 0;
}

/**
   Wait for the next n pixels of the current frame to arrive.
   WARNING - probably not thread safe.  But, in this case, since this library is for single camera mode only, and this is only ever called by 1 thread per camera at once, then we're okay...
*/
int camWaitPixels(int n,int cam,void *camHandle){
  //printf("camWaitPixels %d, camera %d\n",n,cam);
  //For andor, we actually have to wait for the whole frame...
  CamStruct *camstr=(CamStruct*)camHandle;
  char *cd;
  int i;
  char tmp;
  //static struct timeval t1;
  //struct timeval t2;
  //struct timeval t3;
  if(camHandle==NULL || camstr->streaming==0){
    //printf("called camWaitPixels with camHandle==NULL\n");
    return 1;
  }
  if(n<0)
    n=0;
  if(n>camstr->npxls)
    n=camstr->npxls;
  //printf("camWaitPixels\n");
  pthread_mutex_lock(&camstr->m);
  if(camstr->transferRequired){
    camstr->transferRequired=0;
    if(fread(camstr->imgdata,1,sizeof(short)*(camstr->npxls),camstr->fd)!=sizeof(short)*camstr->npxls){
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
    for(i=0; i<camstr->ncam; i++){
      camstr->userFrameNo[i]++;//=camstr->frameno;
      if((camstr->userFrameNo[i]%camstr->nframes)!=camstr->frameno){
	printf("camfile frameno error...\n");
      }
    }

  }
  pthread_mutex_unlock(&camstr->m);

  return 0;
}
