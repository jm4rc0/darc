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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "rtcbuffer.h"

typedef enum{
  BUFFERSEQ,
  BUFFERUSESEQ,
  NBUFFERVARIABLES
}bufferNames;

//char bufferParamList[NBUFFERVARIABLES][16]={
#define makeParamNames() bufferMakeNames(NBUFFERVARIABLES,\
					 "bufferSeq",	\
					 "bufferUseSeq"\
					 )

typedef struct{
  char *paramNames;
  circBuf *rtcErrorBuf;
  paramBuf *pbuf;//the rtc active buffer
  paramBuf *inactive;//the rtc inactive buffer
  paramBuf *curbuf;//pointer to internal buffer inside bufSeq.
  int *which;
  int restore;
  char *bufSeq;
  char *endArrayPtr;
  int bufSeqSize;
  int use;
  int prevUse;
  int assignNeeded;
  int pos;//position in the buffer
  int cnt;//counter for this position in the buffer.
  char *bufptr;//current position in the buffer.
  int *activeIndex;
  void **activeValues;
  char *activeDtype;
  int *activenbytes;
  int *inactiveIndex;
  void **inactiveValues;
  char *inactiveDtype;
  int *inactivenbytes;
  int maxIndexSize;
  unsigned int *bufferframeno;
}BufferSeqStruct;

int bufferNewParam(void *bufferHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr,paramBuf *inactive){
  //Here,if we have any finalisation to do, should do it.
  BufferSeqStruct *bstr=(BufferSeqStruct*)bufferHandle;
  int index[NBUFFERVARIABLES];
  void *values[NBUFFERVARIABLES];
  char dtype[NBUFFERVARIABLES];
  int nbytes[NBUFFERVARIABLES];
  int nfound;
  int err=0;
  bstr->inactive=inactive;//copy previously active buffer to inactive.
  bstr->pbuf=pbuf;
  nfound=bufferGetIndex(pbuf,NBUFFERVARIABLES,bstr->paramNames,index,values,dtype,nbytes);
  /*if(nfound!=NBUFFERVARIABLES){
    printf("Didn't get all buffer entries for buffer module:\n");
    for(i=0;i<NBUFFERVARIABLES;i++){
      if(index[i]<0)
	printf("Missing %16s (non-fatal)\n",&bstr->paramNames[i*BUFNAMESIZE]);
    }
    }*/
  if(index[BUFFERUSESEQ]>=0 && nbytes[BUFFERUSESEQ]==sizeof(int) && dtype[BUFFERUSESEQ]=='i'){
    bstr->use=*((int*)values[BUFFERUSESEQ]);
    //printf("got bstr->use %d\n",bstr->use);
  }else{
    printf("no bufferUseSeq found\n");
    bstr->use=0;
    bstr->prevUse=0;
  }
  //Now, we should only update the pointer to bufSeq if prevUse==0.
  //This way, we keep the same buffer all the way through.
  if(bstr->use){
    if(bstr->prevUse==0){
      if(index[BUFFERSEQ]>0 && dtype[BUFFERSEQ]=='b'){
	bstr->bufSeq=(char*)values[BUFFERSEQ];
	bstr->bufSeqSize=nbytes[BUFFERSEQ];
	bstr->endArrayPtr=bstr->bufSeq+bstr->bufSeqSize;
    //if(bstr->prevUse==0){
      //using buffer for first time, so do setup.
	bstr->prevUse=1;
	bstr->assignNeeded=1;
      //}
      }else{
	printf("Wrong type for bufferSeq, or not found\n");
	bstr->use=0;
	bstr->assignNeeded=0;
      }
    }else{
      //dont do anything - we've already got the bufferSeq and are using it.
      bstr->assignNeeded=0;
    }
    //Have the contents of bufSeq changed?  If so, we need to reset... and set assignNeeded.
    //Actually, I should make it safe to assume that bufSeq can't be changed if useBufSeq is set (put into control.py).


  }else{
    bstr->prevUse=0;
    bstr->assignNeeded=0;
  }
  //Now - does anything need restoring to the original state...
  if(bstr->restore && bstr->use==0){
    bstr->restore=0;
    //copy the original buffer contents back.
  }
  return err;
}

int bufferOpen(char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **bufferHandle,int nthreads,unsigned int frameno,unsigned int **bufferframeno,int *bufferframenosize,paramBuf *inactive){
  BufferSeqStruct *bstr;
  int err;
  char *pn;
  printf("Opening rtcbuffer\n");
  if((pn=makeParamNames())==NULL){
    printf("Error making paramList - please recode rtcbuffer.c\n");
    *bufferHandle=NULL;
    return 1;
  }
  if((bstr=calloc(sizeof(BufferSeqStruct),1))==NULL){
    printf("Error allocating calibration memory\n");
    *bufferHandle=NULL;
    //threadInfo->globals->reconStruct=NULL;
    return 1;
  }
  if(*bufferframenosize==0){
    if(*bufferframeno!=NULL){
      free(*bufferframeno);
      *bufferframenosize=0;
    }
    if((*bufferframeno=malloc(sizeof(unsigned int)))==NULL){
      printf("Failed to malloc bufferframeno\n");
    }else{
      *bufferframenosize=1;
    }
  }
  bstr->bufferframeno=*bufferframeno;
  bstr->bufferframeno[0]=0;
  bstr->paramNames=pn;
  bstr->pbuf=inactive;//this will get copied in the call to bufferNewParam.
  bstr->assignNeeded=1;
  //bstr->inactive=inactive;
  *bufferHandle=(void*)bstr;
  if((bstr->curbuf=malloc(sizeof(paramBuf)))==NULL){
    printf("Failed to malloc curbuf\n");
    free(*bufferHandle);
    *bufferHandle=NULL;
    return 1;
  }
  //bstr->buf=1;
  bstr->rtcErrorBuf=rtcErrorBuf;
  err=bufferNewParam(*bufferHandle,pbuf,frameno,arr,inactive);//this will change ->buf to 0.
  if(err!=0){
    printf("Error in bufferOpen...\n");
    bufferClose(bufferHandle);
    *bufferHandle=NULL;
    return 1;
  }
  return 0;
}
int bufferClose(void **bufferHandle){
  BufferSeqStruct *bstr=(BufferSeqStruct*)*bufferHandle;
  printf("Closing rtcbuffer library\n");
  if(bstr!=NULL){
    //pthread_mutex_destroy(&bstr->calmutex);
    //pthread_cond_destroy(&bstr->calcond);
    if(bstr->paramNames!=NULL)
      free(bstr->paramNames);
    if(bstr->curbuf!=NULL)
      free(bstr->curbuf);
    if(bstr->activeIndex!=NULL)
      free(bstr->activeIndex);
    free(bstr);
  }
  *bufferHandle=NULL;
  return 0;
}
int bufferUpdate(void *bufferHandle){
  BufferSeqStruct *bstr=(BufferSeqStruct*)bufferHandle;
  paramBuf *buf;
  int err=0;
  int i;
  printf("bufferUpdate %d\n",bstr->use);
  if(bstr->use){
    bstr->restore=1;
    //look at the information in bufSeq, and set everything in parambuf to what it needs to be for this iteration...
    
    //What is the format of bufSeq?
    //It should contain a set of HDUs:
    //Each HDU is like a mini paramBuf.
    //hdr[0]==header size
    //hdr[1]==number of entries
    //hdr[2]==number of times to repeat (i.e. do nothing before moving on to next one)
    //hdr[3]==buffer size
    //Then comes the buffer
    //The buffer has array of names(16), array of dtypes(1), array of start (4), array of nbytes(4), array of dims (unused,4), dims (24), array of which buffer to write to (active==0 or inactive) (4) , and then the data.
    //Note - this approach only works with arrays, unless you also set switchRequested to 1 each time... in which case the buffer is switched... in which case, bufferUseSeq should be set in the inactive buffer too, to make sure that when switched, it is still used.
    //The buffer array contains NHDR*16 bytes for names, NHDR bytes for dtype, NHDR bytes for which buffer to write to (1==inactive), NHDR*4 bytes for start and NDHR*4 bytes for nbytes.  We don't care about dims or comment, so these aren't included.  
    //Note that, the dtype and nbytes must match that already existing, so that start doesn't have to change in the real parameter buf.
    if(bstr->assignNeeded){
      //first time round for this bufferSeq, so initialise stuff.
      bstr->cnt=0;
      bstr->pos=0;
      bstr->bufptr=bstr->bufSeq;
      bstr->bufferframeno[0]=0;
      //printf("assigned\n");
    }else if(bstr->cnt>=bstr->curbuf->hdr[2]){//time to move to the next one.
      bstr->bufptr+=bstr->curbuf->hdr[3];
      //printf("Moving to next entry %p %d %p\n",bstr->bufptr,bstr->curbuf->hdr[3],bstr->endArrayPtr);
      bstr->cnt=0;
      bstr->pos++;
      if(bstr->bufptr>=bstr->endArrayPtr || bstr->bufptr+bstr->curbuf->hdr[3]>bstr->endArrayPtr){//wrap around.
	bstr->pos=0;
	bstr->bufptr=bstr->bufSeq;
      }
      bstr->assignNeeded=1;
    }else{
      //printf("Staying with this entry: %d %d\n",bstr->cnt,bstr->curbuf->hdr[2]);
    }
    if(bstr->assignNeeded){
      bstr->assignNeeded=0;
      //rearrange a few pointers...
      buf=bstr->curbuf;
      buf->arr=bstr->bufptr;
      buf->hdr=(int*)bstr->bufptr;
      buf->buf=bstr->bufptr+buf->hdr[0];
      buf->dtype=&(buf->buf[BUFNHDR(buf)*BUFNAMESIZE]);
      buf->start=(int*)(buf->dtype+BUFNHDR(buf));
      buf->nbytes=buf->start+BUFNHDR(buf);
      bstr->which=(int*)(buf->nbytes+BUFNHDR(buf)*8);
      if(bufferCheckNames(buf->hdr[1],buf->buf)!=0){
	printf("Error with entry in bufferSeq, position %d",bstr->pos);
	err=1;
	bstr->use=0;
	bstr->prevUse=0;
      }else{
	//First make the index...
	if(buf->hdr[1]>bstr->maxIndexSize){
	  //need to reallocate stuff.
	  if(bstr->activeIndex!=NULL) free(bstr->activeIndex);
	  if((bstr->activeIndex=malloc((sizeof(int)+sizeof(void*)+sizeof(char)+sizeof(int))*2*buf->hdr[1]))==NULL){
	    printf("Failed to malloc activeIndex in rtcbuffer\n");
	    err=1;
	    bstr->use=0;
	    bstr->prevUse=0;
	    bstr->activeValues=NULL;
	    bstr->activeDtype=NULL;
	    bstr->activenbytes=NULL;
	    bstr->inactiveIndex=NULL;
	    bstr->inactiveValues=NULL;
	    bstr->inactiveDtype=NULL;
	    bstr->inactivenbytes=NULL;
	    bstr->maxIndexSize=0;
	  }else{//nice - pointer arithmetic!
	    bstr->activeValues=(void**)(bstr->activeIndex+buf->hdr[1]);
	    bstr->activeDtype=(char*)(bstr->activeValues+buf->hdr[1]);
	    bstr->activenbytes=(int*)(bstr->activeDtype+buf->hdr[1]);
	    bstr->inactiveIndex=(int*)(bstr->activenbytes+buf->hdr[1]);
	    bstr->inactiveValues=(void**)(bstr->inactiveIndex+buf->hdr[1]);
	    bstr->inactiveDtype=(char*)(bstr->inactiveValues+buf->hdr[1]);
	    bstr->inactivenbytes=(int*)(bstr->inactiveDtype+buf->hdr[1]);
	    bstr->maxIndexSize=buf->hdr[1];
	  }
	}
	if(err==0){
	  bufferGetIndex(bstr->pbuf,buf->hdr[1],buf->buf,bstr->activeIndex,bstr->activeValues,bstr->activeDtype,bstr->activenbytes);
	  bufferGetIndex(bstr->inactive,buf->hdr[1],buf->buf,bstr->inactiveIndex,bstr->inactiveValues,bstr->inactiveDtype,bstr->inactivenbytes);
	}
      }
    }
    //printf("pos %d cnt %d err %d\n",bstr->pos,bstr->cnt,err);

    if(err==0){
      buf=bstr->curbuf;
      //now do what we need to do...
      for(i=0; i<buf->hdr[1]; i++){//for each entry... put the data into the appropriate buffer
	//printf("Updating %16s\n",&buf->buf[i*BUFNAMESIZE]);
	if(bstr->which[i]==0){//active
	  if(bstr->activeIndex[i]>=0 && BUFNBYTES(buf,i)==bstr->activenbytes[i] && BUFDTYPE(buf,i)==bstr->activeDtype[i]){
	    if(BUFNBYTES(buf,i)>0){//othrewise, it can only be a None/NULL value.
	      //copy the data
	      memcpy(bstr->activeValues[i],BUFGETVALUE(buf,i),BUFNBYTES(buf,i));
	    }
	  }else{
	    printf("Wrong data type/size for rtcbuffer[%d]: %16s (index=%d)\n",bstr->pos,&buf->buf[i*BUFNAMESIZE],bstr->activeIndex[i]);
	    if(bstr->activeIndex[i]>=0){
	      printf("Should be %d %c, is %d %c\n",bstr->activenbytes[i],bstr->activeDtype[i],BUFNBYTES(buf,i),BUFDTYPE(buf,i));
	    }
	  }
	}else{//inactive
	  if(bstr->inactiveIndex[i]>=0 && BUFNBYTES(buf,i)==bstr->inactivenbytes[i] && BUFDTYPE(buf,i)==bstr->inactiveDtype[i]){
	    if(BUFNBYTES(buf,i)>0){//othrewise, it can only be a None/NULL value.
	      //copy the data
	      memcpy(bstr->inactiveValues[i],BUFGETVALUE(buf,i),BUFNBYTES(buf,i));
	    }
	  }else{
	    printf("Wrong data size/type for rtcbuffer[%d]: %16s\n",bstr->pos,&buf->buf[i*BUFNAMESIZE]);
	  }
	}
      }
    }
    bstr->bufferframeno[0]++;
    bstr->cnt++;
  }
  return 0;
}

