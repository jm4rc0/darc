#include <nslapi.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>

#define IMGSIZE 128
#define FIBRE_PORT 1
#define NBLOCKS 1
/*
nblocks, time:
1 0.005
2 0.0059
4 0.0066
8 0.007
16 0.009
64 0.02
*/
int
main(int argc, char *argv[])
{
  nslDeviceInfo info;
  nslHandle handle;
  uint32 *buffer;           /* receive data */
  uint16 *hbuf;
  uint8 *cbuf;
  uint32 *ibuf;
  uint32 bufLenBytes;
  uint32 flagsIn, flagsOut;
  uint32 timeout;
  uint32 bytesXfered;
  uint32 *sofWord;
  nslSeq seq;
  uint32 status;
  int fibreport;
  int i, j, k,off,nb,err,nblocks;
  int bytepix;
  int niter;
  short htmp;
  int printn=0;
  struct timeval t1,t2;
  if(argc!=5){
    printf("Usage: %s fibreport(1) bytes(including 8 byte header) byte/pxl niter\n",argv[0]);
    return 1;
  }
  fibreport=atoi(argv[1]);
  bufLenBytes=atoi(argv[2]);
  bytepix=atoi(argv[3]);
  niter=atoi(argv[4]);
  /* Create a buffer */
  nblocks=NBLOCKS;
  sofWord=malloc(sizeof(uint32)*3);
  //bufLenBytes = (IMGSIZE * IMGSIZE) * sizeof(uint16)+2*sizeof(uint32);
  buffer = (uint32 *)malloc(bufLenBytes);
  if (buffer == (uint32 *)0) {
    fprintf(stderr, "Failed to alloc buffer\n");
    return (-1);
  }
  hbuf=(short*)&buffer[2];
  ibuf=(int*)&buffer[2];
  cbuf=(char*)&buffer[2];
  status = nslOpen(fibreport, &handle);
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
  status = nslSetState(&handle,NSL_EN_RECEIVE, 1);
  if (status != NSL_SUCCESS)
    goto error;

  gettimeofday(&t1,NULL);
  for (k = 0; niter<0 || k < niter; k++) {

    /* Wait for Start of Frame: Sync with Data Valid */
    
    /* fprintf(stderr, "Waiting for SYNC frame...\n"); */

    flagsIn = NSL_DMA_USE_SYNCDV;
    //for (i = 0; i < 100; i++) {
    while(1){
      status = nslRecv(&handle, (void *)sofWord, 4, flagsIn, 100, 
		       &bytesXfered, &flagsOut, &seq);
 
      if (status == NSL_SUCCESS) {
	if (flagsOut & NSL_DMA_USE_SYNCDV) {
	  fprintf(stderr, "SYNC frame data = 0x%x\n", sofWord[0]);
	  break;
	}else{
	  fprintf(stderr,"WARNING: SYNCDV not set\n");
	}
      } else if (status != NSL_TIMEOUT) {
	goto error;
      }else{
	printf("timeout\r");
	printn=1;
	fflush(NULL);
      }
    }
    if(printn){
      printn=0;
      printf("\n");
    }
    /*
    if (i == 100) {
      fprintf(stderr, "No SYNC after 100 reads\n\n");
      return (-1);
      // continue;
      }*/
    fprintf(stderr, "SYNC after %d reads\n\n", i); 

    flagsIn = 0;
    off=0;
    nb=(bufLenBytes/nblocks)&~0x3;
    err=0;
    for(i=0; (i<nblocks && err==0); i++){
      if(i==nblocks-1)
	nb=bufLenBytes-off;
      status = nslRecv(&handle, (void *)&buffer[off/4], nb, flagsIn, 1000,
		     &bytesXfered, &flagsOut, &seq);
      if (status == NSL_TIMEOUT) {
	fprintf(stderr, "Receive timeout\n");
	err=1;
      } else if (status == NSL_LINK_ERROR) {
	fprintf(stderr, "Link error detected\n");
	err=1;
      } else if (status == NSL_SUCCESS){
	if(nb!=bytesXfered){
	  fprintf(stderr, "%d bytes requested, "
		  "%d bytes received, "
		  "seq. no. is %d\n", 
		  nb, bytesXfered, (int)seq.seqNum);
	}
      }else{
	fprintf(stderr, "%s %d\n", nslGetErrStr(status),status);
	err=1;
      }
      off+=nb;
    }
    if(err==0){
      /* Print SPARTA format header */
      printf("SOF: 0x%x  ", sofWord[0]);
      for (i = 0; i < 2; i++) { 
	printf(" 0x%x  ", buffer[i]);
      }
      printf("\n");
      
      /* Print first 4 dwords of data */
      for (i = 0; i < 16 && i<(bufLenBytes-8)/bytepix; i++) {
	if ((i & 0x3) == 0)
	  printf("\n");
	htmp=(short)ibuf[i];
	if(bytepix==1)
	  printf("0x%x ", (int)cbuf[i]);//+2]);
	else if(bytepix==2)
	  printf("0x%x ", (int)hbuf[i]);//+2]);
	else if(bytepix==4)
	  printf("0x%x 0x%hx  ", (int)ibuf[i],htmp);//+2]);
      }
    } else {
      fprintf(stderr, "%s\n", nslGetErrStr(status));
    }
    printf("\n\n");
  }
  gettimeofday(&t2,NULL);
  printf("Time taken %gs\n",t2.tv_sec-t1.tv_sec+1e-6*(t2.tv_usec-t1.tv_usec));

  status = nslClose(&handle);
  if (status != NSL_SUCCESS) {
    fprintf(stderr, "Failed to close SL240: %s\n\n", nslGetErrStr(status));
  }

  return (status);

 error:
  fprintf(stderr,"Error\n");
  fprintf(stderr, "%s\n\n", nslGetErrStr(status));
  nslClose(&handle);
  return (status);
}

  
  
  
