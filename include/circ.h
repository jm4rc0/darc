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
The header of the circular buffer is:
8 bytes BUFSIZE(cb) (*((long*)cb->mem))
4 bytes LASTWRITTEN(cb) (*((int*)&(cb->mem[8])))
4 bytes FREQ(cb) (*((int*)&(cb->mem[12])))
4 bytes NSTORE(cb) (*((int*)&(cb->mem[16])))
1 byte FORCEWRITEALL(cb) cb->mem[20] This is also shared with receiver objects when not running on the RTCS, and is used as the number of processes waiting for the next frame.
1 byte NDIM(cb) cb->mem[21]
1 byte DTYPE(cb) cb->mem[22]
1 byte FORCEWRITE(cb) cb->mem[23]
4 bytes SHAPEARR(cb) ((int*)&(cb->mem[24]))
4 bytes CONTIGUOUS(cb) for contiguous (whether data should be sent contiguously) (*((int*)&cb->mem[28]))
4*2 bytes spare.  (previously was used for dimensions)
8 bytes LATESTBUFOFFSET(cb) (*((unsigned long*)&(cb->mem[40])))
Then, as we're using darc_condwaits:
4 bytes CIRCPID(cb) (*((int*)(&cb->mem[48])))
4 bytes CIRCHDRSIZE(cb) (*((int*)(&cb->mem[52])))
1 byte for signalling (used remotely) CIRCSIGNAL(cb) cb->mem[56];
3 bytes spare
4 bytes (sizeof(darc_condwait_t)) CONDWAITSIZE(cb) (*((int*)(&cb->mem[60])))
sizeof(darc_condwait_t) bytes CONDWAIT(cb) (((darc_condwait_t*)(&cb->mem[64])))


The data then uysed to be frame number array, time array, data array.
This has changed to: 4 bytes of size, 4 bytes of frameno, 8 bytes of time, 1 bytes dtype, 15 bytes spare then the data, this is repeated for each circular buffer entry - ie they all have a mini header... makes it easier for moving a raw frame about... 

Then, if LATESTBUFOFFSET(cb)!=0, at this many bytes into the circular buffer, we start another circular buffer for the header data.  This commences with a mini header: the size, lastwritten, nstore, 
*/

#ifndef _CIRC_H
#define _CIRC_H
#include <pthread.h>
#include "darcMutex.h"
#define CIRCDIMSIZE 1//was 6, but nothing used it, so now 1.
//At some point, dimensions should be removed entirely, and just a size remaining.  

typedef struct {
  char *mem;
  int memsize;
  void *data;
  char *name;
  //double *timestamp;
  //int *frameNo;
  //int freqcnt;//only used by the owner
  //int framecnt;//only used by the owner
  int sizecnt;//only used by owner, internally when circAddPartial is called.
  int errFlag;//used by rtcErrorBuf.  
  int datasize;
  int frameSize;//datasize+16, aligned. (4 bytes for size, 8 for time, 4 for frameno).
  pthread_mutex_t mutex;//not currently used in circ.c, but can be used by calling functions to ensure thread safety.
  int lastReceived;//used when a reader - the circular buffer index.
  int lastReceivedFrame;//used when a reader - the RTC framenumber
  int nstoreSave;//only used when a reader
  char ndimSave;//only used when a reader
  int dimsSave[CIRCDIMSIZE];//only used when a reader
  char dtypeSave;//only used when a reader
  int addRequired;
  darc_condwait_t *condwait; // drj 140422: changed darc_futex* to darc_condwait*
}circBuf;
#define BUFSIZE(cb) (*((long*)cb->mem))
#define LASTWRITTEN(cb) (*((int*)&(cb->mem[8])))
#define FREQ(cb) (*((int*)&(cb->mem[12])))
#define NSTORE(cb) (*((int*)&(cb->mem[16])))
#define FORCEWRITEALL(cb) cb->mem[20]
//#define NDIM(cb) (*((short*)&(cb->mem[20])))
#define NDIM(cb) cb->mem[21]
#define DTYPE(cb) cb->mem[22]
#define FORCEWRITE(cb) cb->mem[23]
#define SHAPEARR(cb) ((int*)&(cb->mem[24]))
#define CONTIGUOUS(cb) (*((int*)&cb->mem[28]))
#define LATESTBUFOFFSET(cb) (*((unsigned long*)&(cb->mem[40])))
#define CIRCPID(cb) (*((int*)(&cb->mem[48])))
#define CIRCHDRSIZE(cb) (*((int*)(&cb->mem[52])))
#define CIRCSIGNAL(cb) cb->mem[56]
#define CONDWAITSIZE(cb) (*((int*)(&cb->mem[60])))
#define CONDWAIT(cb) (((darc_condwait_t*)(&cb->mem[64])))

#define CIRCFRAMENO(cb,indx) *((int*)(&(((char*)cb->data)[indx*cb->frameSize+4])))
#define CIRCDATASIZE(cb,indx) *((int*)(&(((char*)cb->data)[indx*cb->frameSize])))

#define MSGDEC 1//used for receiver sending new decimation
#define MSGCONTIG 2//or contiguous value to sender.

#define ALIGN 8
#define HSIZE 32 //NOW DEPRECIATED - USE CIRCHSIZE INSTEAD.
#define CIRCHSIZE 32 //the mini header size - recorded for each entry, preceeding the data - size, frameno, time, dtype etc.
//circBuf* circAssign(void *mem,int memsize,int semid,int nd, int *dims,char dtype, circBuf *cb);
int circSetAddIfRequired(circBuf *cb,int frameno);
#define circCheckAddRequired(cb) ((cb)->addRequired)
//inline int circCheckAddRequired(circBuf *cb);
int circAdd(circBuf *cb,void *data,double timestamp,int frameno);
int circAddForce(circBuf *cb,void *data,double timestamp,int frameno);
//int circAddPartial(circBuf *cb,void *data,int offset,int size,double timestamp,int frameno);
int circReshape(circBuf *cb,int nd, int *dims,char dtype);
int calcDatasize(int nd,int *dims,char dtype);
int circAddSize(circBuf *cb,void *data,int size,int setzero,double timestamp,int frameno);//adds size-bytes to a buffer and publishes,
int circAddSizeForce(circBuf *cb,void *data,int size,int setzero,double timestamp,int frameno);//adds data (size-bytes) to a buffer, whether requested or not, and publishes.
int circInsert(circBuf *cb,void* data,int size, int offset);//inserts data of size-bytes into a buffer, at offset, but doesn't publish (a subsequent call to circAddSize should then be made to publish - potentially with a size of 0).
void *circGetNextFrame(circBuf *cb,float ftimeout,int retry);
int circHeaderUpdated(circBuf *cb);
void *circGetLatestFrame(circBuf *cb);
void *circGetFrame(circBuf *cb,int indx);
circBuf* circOpenBufReader(char *name);
circBuf* openCircBuf(char *name,int nd,int *dims,char dtype,int nstore);
void circClose(circBuf *cb);//should be called by the owner (writer) of the buf
int circCloseBufReader(circBuf *cb);//called on value returned from circOpenBufReader();
int circCalcHdrSize();



#endif
