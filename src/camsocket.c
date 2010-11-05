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
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/select.h>

typedef struct{
  //char *ffname;
  char *host;
  int port;
  int transferRequired;
  int frameno;
  short *imgdata;
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
}CamStruct;

/**
   Find out if this SO library supports your camera.

*/
int camQuery(char *name){
  //Note, the strings aren't necessarily null terminated...
  if(strcmp(name,"camsocket")==0)
    return 0;
  return 1;
}

void dofree(CamStruct *camstr){
  if(camstr!=NULL){
    if(camstr->sockOpen){
      close(camstr->sd);
    }
    free(camstr->host);
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

int camOpen(char *name,int narg,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **camHandle,int nthreads,unsigned int thisiter,unsigned int **frameno,int *framenoSize,int npxls,short *pxlbuf,int ncam,int *pxlx,int* pxly){ 
  CamStruct *camstr;
  struct sockaddr_in sin;
  struct hostent *host;
  
  printf("Initialising camera %s\n",name);
  if(narg<2){
    printf("Error - need arguments port(int32),hostname(null terminated string)\n");
    return 1;
  }
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
  if(*framenoSize==0){
    if((*frameno=malloc(sizeof(int)))==NULL){
      printf("Unable to malloc camframeno\n");
    }else{
      *framenoSize=1;
    }
  }
  camstr->userFrameNo=*frameno;
  camstr->npxls=npxls;//*pxlx * *pxly;
  camstr->port=args[0];
  camstr->host=strndup((char*)&args[1],(narg-1)*sizeof(int));
  printf("Got host %s, port %d\n",camstr->host,camstr->port);
  if((host=gethostbyname(camstr->host))==NULL){
    printf("gethostbyname error\n");
    free(camstr);
    *camHandle=NULL;
    return 1;
  }
  camstr->sd = socket(AF_INET, SOCK_STREAM, 0);
  memcpy(&sin.sin_addr.s_addr, host->h_addr, host->h_length);
  sin.sin_family = AF_INET;
  sin.sin_port = htons(camstr->port);
  if(connect(camstr->sd,(struct sockaddr *)&sin, sizeof(sin))<0){
    printf("Error connecting\n");
    close(camstr->sd);
    free(*camHandle);
    *camHandle=NULL;
    return 1;
  }else{
    printf("Connected\n");
    camstr->sockOpen=1;
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
int camNewParam(void *camHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr){
  return 0;
}
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
      while(nr<ns){
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
      if((camstr->header[0]>>24)!=0x55 || (camstr->header[0]&0xffffff)!=camstr->npxls){
	printf("Unexpected header or wrong number of pixels, %#x %d     \r",camstr->header[0],camstr->header[1]);
	err=1;
      }
      if(err==0){
	camstr->frameno=camstr->header[1];
	//now read the data
	ns=sizeof(short)*camstr->npxls;
	nr=0;
	while(nr<ns){
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
