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
Functions to read the parameter buffer
*/
#ifndef BUFFER_H //header guard
#define BUFFER_H

#include <pthread.h>
/**
   Hold the parameter buffer shared memory pointer and semaphore.
   arr is the shared memory array.
   buf holds the buffer, and is an index into arr.
   hdr holds buffer header information, and is the first part of arr.
   hdr[0]==header size.
   hdr[1]==max number of buffer entries (NHDR)
   hdr[2]==block (flags)
   hdr[3]==sizeof(pthread_mutex_t)
   hdr[4]==sizeof(pthread_cond_t)

   nbytes, start and dtype are indexs into buf.

   buf contains NHDR*BUFNAMESIZE bytes for name entries
   NHDR bytes dtype entries
   HNDR*4 bytes start entries
   NHDR*4 bytes nbytes
   NHDR*4 bytes ndim
   NHDR*4*6 bytes dims
   NHDR*4 bytes comment length - total of 57*NHDR bytes.
*/
typedef struct{
  char *buf;
  char *arr;//everything (header+buf)
  int *hdr;
  size_t arrsize;//size of arr in bytes.
  pthread_mutex_t *condmutex;
  pthread_cond_t *cond;
  int *nbytes;
  int *start;
  char *dtype;
  //int *lcomment;
  int nNumaNodes;
  void **numaBufs;//a pointer to an array of paramBufs, one per numa node, if requested by the user...
}paramBuf;

typedef struct{
  void *data;
  char dtype;
  unsigned int size;
}bufferVal;

/**
   Checks that paramList is valid
   (alphabetical order, and of correct lenght (<16 chars)
*/
int bufferCheckNames(int n,char *paramList);
/**
   Returns the index of each param named in paramList, or -1 if not found, placed into array index, which should be of size n.  paramList must have n entries, each a null terminated string.
   pbuf is the parameter buffer.
   values is an array of void* with size n.  Each entry is then a pointer to the data, that can be cast as required according to pbuf->nbytes[index] and pbuf->dtype[index], for example, if these are 4 and i, you would do *(int*)(values[index])
   If these are 16 and f, you would do (float*)(values[index])
 */
#ifdef __cplusplus
extern "C" 
#endif
int bufferGetIndex(paramBuf *pbuf,int n,char *paramList,int *index,void **values,char *dtype, int *nbytes);

#ifdef __cplusplus
extern "C" 
#endif
char *bufferMakeNames(int n,...);

paramBuf *bufferOpen(char *name);
paramBuf *bufferOpenFromData(char *arr,size_t size);
void bufferClose(paramBuf *pb);
unsigned long bufferGetMem(paramBuf *pbuf,int includeArrHdrSize);
int bufferGetNEntries(paramBuf *pbuf);
int bufferGetInactive(paramBuf *pbuf[2]);
int bufferGetActive(paramBuf *pbuf[2]);
int bufferSet(paramBuf *pbuf,char *name,bufferVal *val);
int bufferSetIgnoringLock(paramBuf *pbuf,char *name,bufferVal *val);
bufferVal *bufferGet(paramBuf *pbuf,char *name);
int bufferInit(paramBuf *pbuf,char *fitsfilename);//initialise a buffer with parameters from teh fits file.

#define BUFNHDR(pbuf) (pbuf->hdr[1])// probably 128 by default
#define BUFFLAG(pbuf) (pbuf->hdr[2])
#define BUFSTART(pbuf,index) pbuf->start[index]
#define BUFNBYTES(pbuf,index) pbuf->nbytes[index]
#define BUFDTYPE(pbuf,index) pbuf->dtype[index]
//#define BUFLCOMMENT(pbuf,index) pbuf->lcomment[index]
#define BUFARRHDRSIZE(pbuf) (pbuf->hdr[0])

#define BUFNAMESIZE 16

#define BUFNDIM(pbuf,index) (((int*)&pbuf->buf[pbuf->hdr[1]*(BUFNAMESIZE+1+4+4)])[index])
#define BUFDIM(pbuf,index) (&((int*)&pbuf->buf[pbuf->hdr[1]*(BUFNAMESIZE+1+4+4+4)])[index*6])
#define BUFLCOMMENT(pbuf,index) (((int*)&pbuf->buf[pbuf->hdr[1]*(BUFNAMESIZE+1+4+4+4+6*4)])[index])

#define START ((int*)(&buf[NHDR*20]))//depreciated
#define NBYTES ((int*)(&buf[NHDR*24]))//depreciated
//these should be in agreement with buffer.py.

#define BUFGETVALUE(pbuf,index) ((void*)(pbuf->buf+pbuf->start[index]))

#define BUFALIGN 64
//BUFALIGN should agree with buffer.py.
//#define BUFCIRCHDRSIZE (((int)((8+4+4+4+2+1+1+6*4+4+4+4+4+4+sizeof(pthread_mutex_t)+sizeof(pthread_cond_t)+BUFALIGN-1)/BUFALIGN))*BUFALIGN)

#define BUFHDRSIZE 57 //should be in agreement with buffer.py.

#endif //header guard
