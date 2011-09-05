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

This is for a v4l device, e.g. a webcam.  Note - not coded in the most efficient way - was in a hurry - so if you need this for a production system, probably best to recode - e.g. put the main grabbing loop into a thread or something.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rtccamera.h"
#include <time.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <fcntl.h>              /* low-level i/o */
#include <asm/types.h>          /* for videodev2.h */
#include <linux/videodev2.h>
#include "darc.h"

typedef enum {
	IO_METHOD_READ,
	IO_METHOD_MMAP,
	IO_METHOD_USERPTR,
} io_method;

struct buffer {
        void *                  start;
        size_t                  length;
};

typedef struct{
  char *dev_name;
  io_method io;
  int fd;
  struct buffer *buffers;
  unsigned int n_buffers;
  unsigned short *imgdata;
  int npxls;
  int npxlx;
  int npxly;
  unsigned int *userFrameNo;
  int ncam;
  int captureStarted;
  int camOpen;
  circBuf *rtcErrorBuf;

}CamStruct;


int stop_capturing(CamStruct *camstr);
int uninit_device(CamStruct *camstr);
int close_device(CamStruct *camstr);

void camdoFree(CamStruct *camstr){
  if(camstr!=NULL){
    stop_capturing (camstr);
    uninit_device (camstr);
    close_device (camstr);
    if(camstr->dev_name!=NULL)
      free(camstr->dev_name);
    free(camstr);
  }
}



static int xioctl(int fd,int request,void *arg){
  int r;
  do r = ioctl (fd, request, arg);
  while (r==-1 && errno==EINTR);
  return r;
}

void init_read(CamStruct *camstr,unsigned int		buffer_size){
  printf("Not implemented yet\n");
  /*  buffers = calloc (1, sizeof (*buffers));
  
        if (!buffers) {
                fprintf (stderr, "Out of memory\n");
                exit (EXIT_FAILURE);
        }

	buffers[0].length = buffer_size;
	buffers[0].start = malloc (buffer_size);

	if (!buffers[0].start) {
    		fprintf (stderr, "Out of memory\n");
            	exit (EXIT_FAILURE);
		}*/
}

int init_mmap(CamStruct *camstr){
  struct v4l2_requestbuffers req;
  memset(&req,0,sizeof(req));
  req.count               = 4;
  req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory              = V4L2_MEMORY_MMAP;
  if (xioctl (camstr->fd, VIDIOC_REQBUFS, &req)==-1){
    if (errno==EINVAL) 
      printf ("%s does not support memory mapping\n", camstr->dev_name);
    return 1;
  }
  if (req.count < 2) {
    printf ("Insufficient buffer memory on %s\n",camstr->dev_name);
    return 1;
  }
  camstr->buffers = calloc (req.count, sizeof (*camstr->buffers));
  if (!camstr->buffers) {
    printf ("Out of memory memmapping in camv4l.c\n");
    return 1;
  }
  for (camstr->n_buffers = 0; camstr->n_buffers<req.count;++camstr->n_buffers){
    struct v4l2_buffer buf;
    memset(&buf,0,sizeof(buf));
    buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory      = V4L2_MEMORY_MMAP;
    buf.index       = camstr->n_buffers;
    if (xioctl (camstr->fd, VIDIOC_QUERYBUF, &buf)==-1)
      return 1;
    camstr->buffers[camstr->n_buffers].length = buf.length;
    camstr->buffers[camstr->n_buffers].start = mmap (NULL,buf.length,PROT_READ | PROT_WRITE,MAP_SHARED,camstr->fd, buf.m.offset);
    if (camstr->buffers[camstr->n_buffers].start==MAP_FAILED){
      printf("memmap failed\n");
      return 1;
    }
  }
  return 0;
}

int init_userp(CamStruct *camstr,unsigned int		buffer_size){
  printf("Not yet implemented\n");
  /*struct v4l2_requestbuffers req;
        unsigned int page_size;

        page_size = getpagesize ();
        buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);

	memset(&req,0,sizeof(req));

        req.count               = 4;
        req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory              = V4L2_MEMORY_USERPTR;

        if (-1 == xioctl (fd, VIDIOC_REQBUFS, &req)) {
                if (EINVAL == errno) {
                        fprintf (stderr, "%s does not support "
                                 "user pointer i/o\n", dev_name);
                        exit (EXIT_FAILURE);
                } else {
                        errno_exit ("VIDIOC_REQBUFS");
                }
        }

        buffers = calloc (4, sizeof (*buffers));

        if (!buffers) {
                fprintf (stderr, "Out of memory\n");
                exit (EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
                buffers[n_buffers].length = buffer_size;
                buffers[n_buffers].start = memalign (page_size,
                                                     buffer_size);

                if (!buffers[n_buffers].start) {
    			fprintf (stderr, "Out of memory\n");
            		exit (EXIT_FAILURE);
		}
        }*/
  return 0;
}


int init_device(CamStruct *camstr){
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  unsigned int min;
  int err=0;
  if (xioctl (camstr->fd, VIDIOC_QUERYCAP, &cap)==-1) {
    if (errno==EINVAL)
      printf ( "%s is no V4L2 device\n",camstr->dev_name);
    else
      printf("VIDIOC_QUERYCAP error\n");
    return 1;
  }
  if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    printf( "%s is no video capture device\n",camstr->dev_name);
    return 1;
  }
  switch (camstr->io) {
  case IO_METHOD_READ:
    if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
      printf ("%s does not support read i/o\n", camstr->dev_name);
      return 1;
    }
    break;
  case IO_METHOD_MMAP:
  case IO_METHOD_USERPTR:
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
      printf ("%s does not support streaming i/o\n",camstr->dev_name);
      //return 1;
    }
    break;
  }
  // Select video input, video standard and tune here. 
  memset(&cropcap,0,sizeof(cropcap));
  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(camstr->fd,VIDIOC_CROPCAP,&cropcap)==0){
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; // reset to default 
    if (xioctl (camstr->fd, VIDIOC_S_CROP, &crop)==-1) {
      printf("Cropping not supported\n");
    }
  }else{
    printf("VIDIOC_CROPCAP error ignored - continuing\n");
  }
  memset(&fmt,0,sizeof(fmt));
  fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width       = camstr->npxlx;
  fmt.fmt.pix.height      = camstr->npxly;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;//Y16 is 16 bit gray
  fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
  if (xioctl (camstr->fd, VIDIOC_S_FMT, &fmt)==-1){
    printf("VIDIOC_S_FMT failed\n");
    return 1;
  }
  // Note VIDIOC_S_FMT may change width and height. 
  // Buggy driver paranoia. 
  min = fmt.fmt.pix.width * 2;
  if (fmt.fmt.pix.bytesperline < min)
    fmt.fmt.pix.bytesperline = min;
  min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
  if (fmt.fmt.pix.sizeimage < min)
    fmt.fmt.pix.sizeimage = min;
  printf("Sizes: %d %d\n",fmt.fmt.pix.width,fmt.fmt.pix.height);
  switch (camstr->io) {
  case IO_METHOD_READ:
    init_read (camstr,fmt.fmt.pix.sizeimage);
    break;
  case IO_METHOD_MMAP:
    err=init_mmap (camstr);
    break;
  case IO_METHOD_USERPTR:
    init_userp (camstr,fmt.fmt.pix.sizeimage);
    break;
  }
  return err;
}
int open_device(CamStruct *camstr){
  struct stat st; 
  if (stat (camstr->dev_name, &st)==-1) {
    printf ( "Cannot identify '%s': %d, %s\n",camstr->dev_name, errno, strerror (errno));
    return 1;
  }
  if (!S_ISCHR (st.st_mode)){
    printf( "%s is no device\n", camstr->dev_name);
    return 1;
  }
  camstr->fd = open (camstr->dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);
  if (camstr->fd==-1) {
    printf("Cannot open '%s': %d, %s\n",camstr->dev_name, errno, strerror (errno));
    return 1;
  }
  return 0;
}

int start_capturing(CamStruct *camstr){
  unsigned int i;
  enum v4l2_buf_type type;
  switch (camstr->io) {
  case IO_METHOD_MMAP:
    for (i = 0; i < camstr->n_buffers; ++i) {
      struct v4l2_buffer buf;
      memset(&buf,0,sizeof(buf));
      buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory      = V4L2_MEMORY_MMAP;
      buf.index       = i;
      if (xioctl (camstr->fd, VIDIOC_QBUF, &buf)==-1){
	printf("Error qbuf\n");
	return 1;
      }
    }
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl (camstr->fd, VIDIOC_STREAMON, &type)==-1){
      printf("Error stream on\n");
      return 1;
    }
    break;
      /*
	case IO_METHOD_USERPTR:
	for (i = 0; i < n_buffers; ++i) {
	struct v4l2_buffer buf;
	memset(&buf,0,sizeof(buf));
	buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory      = V4L2_MEMORY_USERPTR;
	buf.index       = i;
	buf.m.userptr	= (unsigned long) buffers[i].start;
	buf.length      = buffers[i].length;
	if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
	errno_exit ("VIDIOC_QBUF");
	}
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl (fd, VIDIOC_STREAMON, &type))
	errno_exit ("VIDIOC_STREAMON");
	break;*/
  default:
    break;
  }
  return 0;
}

int process_image(CamStruct *camstr,unsigned short *addr){
  int i;
  for(i=0;i<camstr->npxls;i++){//copy with byteswap
    camstr->imgdata[i]=((addr[i]>>8)&0xff) | ((addr[i]&0xff)<<8) ;
  }
  return 0;
}
int read_frame(CamStruct *camstr){
  struct v4l2_buffer buf;
  switch (camstr->io) {
  case IO_METHOD_READ:
    printf("Not implemented in camv4l\n");
    /*if (-1 == read (fd, buffers[0].start, buffers[0].length)) {
            		switch (errno) {
            		case EAGAIN:
                    		return 0;

			case EIO:
			// Could ignore EIO, see spec. 

			// fall through 
			default:
				errno_exit ("read");
			}
		}

    		process_image (buffers[0].start);
    */
    break;
  case IO_METHOD_MMAP:
    memset(&buf,0,sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (xioctl (camstr->fd, VIDIOC_DQBUF, &buf)==-1) {
      switch (errno) {
      case EAGAIN:
	printf("VIDIOC_DQBUF EAGAIN error - continuing\n");
	return 0;
      case EIO://couuld ignore EIO, see spec.
      default:
	printf("VIDIOC_DQBUF\n");
	return 1;
      }
    }
    if(buf.index>= camstr->n_buffers){
      printf("error: buf.index>n_buffers in camv4l\n");
      return 1;
    }
    //printf("Processing %d start %p len %d val %d\n",buf.index,camstr->buffers[buf.index].start,camstr->buffers[buf.index].length,(int)(*(unsigned short*)camstr->buffers[buf.index].start));
    process_image (camstr,(unsigned short*)camstr->buffers[buf.index].start);
    if (xioctl (camstr->fd, VIDIOC_QBUF, &buf)==-1){
      printf("VIDIOC_QBUF\n");
      return 1;
    }
    break;
  case IO_METHOD_USERPTR:
    printf("Not implemented\n");
    /*
	  memset(&buf,0,sizeof(buf));

    		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    		buf.memory = V4L2_MEMORY_USERPTR;

		if (-1 == xioctl (fd, VIDIOC_DQBUF, &buf)) {
			switch (errno) {
			case EAGAIN:
				return 0;

			case EIO:
				// Could ignore EIO, see spec. 

				// fall through 

			default:
				errno_exit ("VIDIOC_DQBUF");
			}
		}

		for (i = 0; i < n_buffers; ++i)
			if (buf.m.userptr == (unsigned long) buffers[i].start
			    && buf.length == buffers[i].length)
				break;

		assert (i < n_buffers);

    		process_image ((void *) buf.m.userptr);

		if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
			errno_exit ("VIDIOC_QBUF");

		break;
	}


	return 1;
    */
    break;
  }
  return 0;
}

int stop_capturing(CamStruct *camstr){
  enum v4l2_buf_type type;
  switch (camstr->io) {
  case IO_METHOD_READ:
    /* Nothing to do. */
    break;
  case IO_METHOD_MMAP:
  case IO_METHOD_USERPTR:
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
    if (-1 == xioctl (camstr->fd, VIDIOC_STREAMOFF, &type))
      printf("VIDIOC_STREAMOFF\n");
    break;
  }
  return 0;
}

int uninit_device(CamStruct *camstr){
  unsigned int i;
  switch (camstr->io) {
  case IO_METHOD_READ:
    free (camstr->buffers[0].start);
    break;
  case IO_METHOD_MMAP:
    for (i = 0; i < camstr->n_buffers; ++i)
      if (-1 == munmap (camstr->buffers[i].start, camstr->buffers[i].length))
	printf("munmap error camv4l\n");
    break;
  case IO_METHOD_USERPTR:
    for (i = 0; i < camstr->n_buffers; ++i)
      free (camstr->buffers[i].start);
    break;
  }
  free (camstr->buffers);
  return 0;
}
int close_device(CamStruct *camstr){
  if(camstr->fd>0)
    if (-1 == close (camstr->fd))
      printf("close error camv4l\n");
  camstr->fd = -1;
  return 0;
}


/*int camNewParam(void *camHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr){
  //the only param needed is camReorder if reorder!=0.
  int i;
  CamStruct *camstr=(CamStruct*)camHandle;
  int nfound,err=0;
  INT nRet;
  double actualFrameRate;
  double actualExpTime;
  int prevGrabMode;
  nfound=bufferGetIndex(pbuf,CAMNBUFFERVARIABLES,camstr->paramNames,camstr->index,camstr->values,camstr->dtype,camstr->nbytes);
  i=UEYEFRAMERATE;
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='f' && camstr->nbytes[i]==4){
      camstr->frameRate=*((float*)camstr->values[i]);
      if((nRet=is_SetFrameRate(camstr->hCam,(double)camstr->frameRate,&actualFrameRate))!=IS_SUCCESS)
	printf("is_SetFrameRate failed\n");
      else
	printf("Framerate set to %g\n",actualFrameRate);
    }else{
      printf("uEyeFrameRate error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"uEyeFrameRate error");
      err=1;
    }
  }else{
    printf("uEyeFrameRate not found - ignoring\n");
  }
  i=UEYEEXPTIME;
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='f' && camstr->nbytes[i]==4){
      camstr->expTime=*((float*)camstr->values[i]);
      if((nRet=is_SetExposureTime(camstr->hCam,(double)camstr->expTime,&actualExpTime))!=IS_SUCCESS)
	printf("is_SetExposureTime failed\n");
      else
	printf("Exposure time set to %gms\n",actualExpTime);
    }else{
      printf("uEyeExpTime error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"uEyeExpTime error");
      err=1;
    }
  }else{
    printf("uEyeExpTime not found - ignoring\n");
  }
  i=UEYENFRAMES;
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      camstr->nFrames=*((int*)camstr->values[i]);
    }else{
      printf("uEyeNFrames error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"uEyeNFrames error");
      err=1;
      camstr->nFrames=1;
    }
  }else{
    printf("uEyeNFrames not found - ignoring\n");
    camstr->nFrames=1;
  }
  prevGrabMode=camstr->grabMode;
  i=UEYEGRABMODE;
  if(camstr->index[i]>=0){//has been found...
    if(camstr->dtype[i]=='i' && camstr->nbytes[i]==sizeof(int)){
      camstr->grabMode=*((int*)camstr->values[i]);
    }else{
      printf("uEyeGrabMode error\n");
      writeErrorVA(camstr->rtcErrorBuf,-1,frameno,"uEyeGrabMode error");
      err=1;
      camstr->grabMode=0;
    }
  }else{
    printf("uEyeGrabMode not found - ignoring\n");
    camstr->grabMode=0;
  }
  if(camstr->grabMode!=prevGrabMode){
    //change the operation mode
    if(camstr->grabMode){
      if(camstr->captureStarted){
	camstr->captureStarted=0;
	is_StopLiveVideo(camstr->hCam,IS_WAIT);
      }
    }else{
      //turn on framing.
      is_CaptureVideo(camstr->hCam,IS_WAIT);//set to live mode...
      camstr->captureStarted=1;

    }
  }
  return err;
  }*/


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
  int i;
  unsigned short *tmps;
  //INT cerr;
  //char *errtxt;
  //unsigned short *pxlbuf=arr->pxlbufs;
  printf("Initialising camera %s\n",name);
  if((*camHandle=malloc(sizeof(CamStruct)))==NULL){
    printf("Couldn't malloc camera handle\n");
    return 1;
  }
  memset(*camHandle,0,sizeof(CamStruct));
  camstr=(CamStruct*)*camHandle;
  camstr->rtcErrorBuf=rtcErrorBuf;
  camstr->npxlx=*pxlx;
  camstr->npxly=*pxly;
  if(n>0)
    camstr->dev_name=strndup((char*)args,n*sizeof(int));
  else
    camstr->dev_name=strdup("/dev/video0");
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
  arr->pxlbufsSize=sizeof(unsigned short)*npxls;
  arr->pxlbuftype='H';
  arr->pxlbufelsize=sizeof(unsigned short);
  if(ncam!=1){
    printf("Sorry - only 1 camera currently allowed\n");
    free(*camHandle);
    *camHandle=NULL;
    return 1;
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
  for(i=0; i<ncam; i++){
    camstr->userFrameNo[i]=-1;
  }
  camstr->io=IO_METHOD_MMAP;
  if(open_device(camstr) ||   init_device(camstr) || start_capturing(camstr)){
    printf("Failed to open/init camera\n");
    camdoFree(camstr);
    *camHandle=NULL;
    return 1;
  }
  camstr->camOpen=1;
  camstr->captureStarted=1;

  /*if(camNewParam(*camHandle,pbuf,thisiter,arr)!=0){
    printf("Error in camOpen->newParam...\n");
    camdoFree(camstr);
    *camHandle=NULL;
    return 1;
    }*/
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
  camdoFree(camstr);
  *camHandle=NULL;
  printf("Camera closed\n");
  return 0;
}


/**
   Called when we're starting processing the next frame.
*/
int camNewFrameSync(void *camHandle,unsigned int thisiter,double starttime){
  //printf("camNewFrame\n");
  CamStruct *camstr;
  int i;
  fd_set fds;
  struct timeval tv;
  int r;
  int err=0;
  camstr=(CamStruct*)camHandle;
  if(camHandle==NULL){// || camstr->streaming==0){
    //printf("called camNewFrame with camHandle==NULL\n");
    return 1;
  }
  FD_ZERO (&fds);
  FD_SET (camstr->fd, &fds);
  tv.tv_sec = 2;
  tv.tv_usec = 0;
  r = select (camstr->fd + 1, &fds, NULL, NULL, &tv);
  if (r<=0) {
    printf("Error in select in camv4l\n");
    err=1;
  }else{
    if (read_frame (camstr)){
      printf("Error reading frame\n");
      err=1;
    }
  }


  //memset(camstr->imgdata,0,sizeof(float)*camstr->npxls);
  for(i=0; i<camstr->ncam; i++){
    camstr->userFrameNo[i]++;//=camstr->frameno;
  }
  return err;
}
