#include <nslapi.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#define IMGSIZE 128
#define FIBRE_PORT 0
int main(int argc, char *argv[]){
  nslDeviceInfo info;
  nslHandle handle;
  uint32 *buffer;           /* tx data */
  uint16 *hbuf;
  uint32 bufLenBytes;
  uint32 flagsIn, flagsOut;
  uint32 timeout;
  uint32 sofWord;
  uint32 bytesXfered;
  nslSeq seq;
  uint32 status;
  int i, j, k;
  int fno;
  int naxis=0;
  char buf[80];
  int end;
  int axis;
  int framePixels;
  int *axisarr;
  int bitpix,bytepix;
  FILE *fd;
  int nframes,hdrsize,niter,nread,stime,npxls;
  char *cd,tmp;
  double timeTaken;
  struct timeval t1,t2;
  
/*  Create and fill buffer */
  /*bufLenBytes = 8 * sizeof(uint32);
  buffer = (uint32 *)calloc(8, sizeof(uint32));
  if (buffer == (uint32 *)NULL) {
    fprintf(stderr, "Failed to allocate buffer\n");
    exit(1);
    }*/
  sofWord=0xa5a5a5a5;

  if(argc!=5){
    printf("Usage: %s file.fits npxls niter sleeptime\n",argv[0]);
    return 1;
  }
  npxls=atoi(argv[2]);
  fd=fopen(argv[1],"r");
  niter=atoi(argv[3]);
  stime=atoi(argv[4]);
  printf("File opened\n");
  end=0;
  while(end==0){
    if(fread(buf,1,80,fd)!=80){
      printf("Failed to read file\n");
      end=-1;
    }
    if(strncmp(buf,"NAXIS   ",8)==0){
      naxis=atoi(&buf[10]);
      axisarr=malloc(sizeof(int)*naxis);
      memset(axisarr,0,sizeof(int)*naxis);
    }else if(strncmp(buf,"BITPIX  ",8)==0){
      bitpix=atoi(&buf[10]);
      bytepix=bitpix/8;
      if(bytepix<0)
	bytepix=-bytepix;
      printf("Using %d bit/pxl (%d byte/pxl)\n",bitpix,bytepix);
    }else if(strncmp(buf,"NAXIS",5)==0){
      if(buf[5]!=' '){
	axis=atoi(&buf[5])-1;
	axisarr[axis]=atoi(&buf[10]);
      }
    }else if(strncmp(buf,"END",3)==0){
      end=1;
    }
  }
  if(end==1){
    printf("Got %dD FITS array with dimensions",naxis);
    for(i=0; i<naxis; i++)
      printf(" %d",axisarr[i]);
    printf(".\n");
    framePixels=1;
    for(i=0; i<naxis-1; i++)
      framePixels*=axisarr[i];
    if(framePixels!=npxls){
      printf("Image size doesn't agree with the expected value\n");
      end=-1;
      return 1;
    }
    nframes=axisarr[naxis-1];
    hdrsize=(int)((ftell(fd)+2880-1)/2880)*2880;
    fseek(fd,hdrsize,SEEK_SET);
  }


  bufLenBytes = framePixels*bytepix + 2*sizeof(uint32);
  buffer = (uint32 *)malloc(bufLenBytes);
  if (buffer == NULL) {
    fprintf(stderr, "Failed to alloc buffer\n");
    return (-1);
  }
  /*hbuf=(uint16*)&buffer[2];
  for(i=0; i<IMGSIZE*IMGSIZE; i++){
    hbuf[i]=i;
    }*/
  buffer[0] = 0xbabebabe;
  buffer[1] = 0x5a5a5a5a;

  status = nslOpen(FIBRE_PORT, &handle);
  if (status != NSL_SUCCESS) {
    fprintf(stderr, "Failed to open SL240: %s\n\n", nslGetErrStr(status));
    return (status);
  }

  status = nslGetDeviceInfo(&handle, &info);
  if (status != NSL_SUCCESS) {
    fprintf(stderr, "Failed to get SL240 device info: ");
    goto error;
  }

  fprintf(stderr, "\n\nSL240 Device info:\n"
	  "==================\n");
  fprintf(stderr, "Unit no.\t %d\n", info.unitNum);
  fprintf(stderr, "Board loc.\t %s\n", info.boardLocationStr);
  fprintf(stderr, "Serial no.\t 0x%x.%x\n", 
	  info.serialNumH, info.serialNumL);
  fprintf(stderr, "Firmware rev.\t 0x%x\n", info.revisionID);
  fprintf(stderr, "Driver rev.\t %s\n", info.driverRevisionStr);
  fprintf(stderr, "Fifo size\t %dM\n", info.popMemSize/0x100000);
  fprintf(stderr, "Link speed\t %d MHz\n", info.linkSpeed);
  fprintf(stderr, "No. links\t %d\n\n\n", info.numLinks);


  /*
   * Setup card state
   */
  status = nslSetState(&handle, NSL_EN_EWRAP, 0);
  if (status != NSL_SUCCESS)
    goto error;
  status = nslSetState(&handle,NSL_EN_RECEIVE, 1);
  if (status != NSL_SUCCESS)
    goto error;
  status = nslSetState(&handle,NSL_EN_RETRANSMIT, 0);
  if (status != NSL_SUCCESS)
    goto error;
  status = nslSetState(&handle,NSL_EN_CRC, 1);
  if (status != NSL_SUCCESS)
    goto error;
  status = nslSetState(&handle,NSL_EN_FLOW_CTRL, 0);
  if (status != NSL_SUCCESS)
    goto error;
  status = nslSetState(&handle,NSL_EN_LASER, 1);
  if (status != NSL_SUCCESS)
    goto error;
  status = nslSetState(&handle,NSL_EN_BYTE_SWAP, 0);
  if (status != NSL_SUCCESS)
    goto error;
  status = nslSetState(&handle,NSL_EN_WORD_SWAP, 0);
  if (status != NSL_SUCCESS)
    goto error;
  status = nslSetState(&handle, NSL_STOP_ON_LNK_ERR, 1);
  if (status != NSL_SUCCESS)
    goto error;
  status = nslSetState(&handle,NSL_EN_TRANSMIT, 1);
  if (status != NSL_SUCCESS)
    goto error;

  fno=0;
  nread=0;
  gettimeofday(&t1,NULL);
  for (i=0;(niter<0 || i<niter);i++) {
    
    buffer[0]=fno;
    flagsIn = NSL_DMA_USE_SYNCDV;
    status = nslSend(&handle, (void *)&sofWord, sizeof(uint32), flagsIn, 100,
		     &bytesXfered, &flagsOut);
    if (status == NSL_TIMEOUT) {
      fprintf(stderr, "Tx timeout frame %d\n",fno);
    } else if (status == NSL_LINK_ERROR) {
      fprintf(stderr, "Link error detected frame %d\n",fno);
    }else if(status==NSL_SUCCESS){
      if(sizeof(uint32)!=bytesXfered){
	fprintf(stderr, "%ld bytes requested, "
		"%d bytes sent frame %d\n", 
		sizeof(uint32), bytesXfered,fno);
      }
    }else{
      fprintf(stderr, "error: %s frame %d\n", nslGetErrStr(status),fno);
    }
    //now read and send the data
    if(fread(&buffer[2],1,bytepix*framePixels,fd)!=bytepix*framePixels){
      printf("Error reading FITS file data\n");
      return 1;
    }
    nread++;
    if(nread==nframes){//go back to beginning of file
      nread=0;
      fseek(fd,hdrsize,SEEK_SET);
    }
    cd=(char*)&buffer[2];
    //do the byteswap (fits format is big endian).
    for(j=0; j<framePixels*bytepix; j+=bytepix){
      for(k=0; k<bytepix/2; k++){
	tmp=cd[j+k];
	cd[j+k]=cd[j+bytepix-1-k];
	cd[j+bytepix-1-k]=tmp;
      }
    }

    flagsIn = 0;
    status = nslSend(&handle, (void *)buffer, bufLenBytes, flagsIn, 100,
		     &bytesXfered, &flagsOut);
    if (status == NSL_TIMEOUT) {
      fprintf(stderr, "Tx timeout frame %d\n",fno);
    } else if (status == NSL_LINK_ERROR) {
      fprintf(stderr, "Link error detected frame %d\n",fno);
    } else if (status == NSL_SUCCESS){
      if(bufLenBytes!=bytesXfered){
	fprintf(stderr, "%d bytes requested, "
		"%d bytes sent frame %d\n", 
		bufLenBytes, bytesXfered,fno);
      }
    } else {
      fprintf(stderr, "%s\n", nslGetErrStr(status));
    }
    usleep(stime);
    fno++;
    gettimeofday(&t2,NULL);
    timeTaken=t2.tv_sec-t1.tv_sec+1e-6*(t2.tv_usec-t1.tv_usec);
    printf("%d %gs %gHz       \r",fno,timeTaken,1/timeTaken);
    fflush(NULL);
    t1=t2;
  }

  status = nslClose(&handle);
  if (status != NSL_SUCCESS) {
    fprintf(stderr, "Failed to close SL240: %s\n\n", nslGetErrStr(status));
  }
  printf("\n");
  return (status);

 error:
  fprintf(stderr, "%s\n\n", nslGetErrStr(status));
  nslClose(&handle);
  return (status);
}

  
  
  
