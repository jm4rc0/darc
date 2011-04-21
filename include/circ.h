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
4*6 bytes SHAPEARR(cb) ((int*)&(cb->mem[24]))
Then, if USECOND is defined (i.e. we're using pthread_conds) then:
4 bytes CIRCHDRSIZE(cb) (*((int*)(&cb->mem[48])))
1 byte for signalling (used remotely) CIRCSIGNAL(cb) cb->mem[52];
3 bytes spare
4 bytes (sizeof(pthread_mutex_t)) MUTEXSIZE(cb) (*((int*)(&cb->mem[56])))
4 bytes (sizeof(pthread_cond_t)) CONDSIZE(cb) (*((int*)(&cb->mem[60])))
sizeof(pthread_mutex_t) bytes MUTEX(cb) (((pthread_mutex_t*)(&cb->mem[64])))
sizeof(pthread_cond_t) bytes COND(cb) (((pthread_cond_t*)(&cb->mem[64+MUTEXSIZE(cb)])))


The data then uysed to be frame number array, time array, data array.
This has changed to: 4 bytes of size, 4 bytes of frameno, 8 bytes of time, 1 bytes dtype, 15 bytes spare then the data, this is repeated for each circular buffer entry - ie they all have a mini header... makes it easier for moving a raw frame about... 
*/

#ifndef _CIRC_H
#define _CIRC_H
#include <pthread.h>
#define USECOND
#if defined(__GNU_LIBRARY__) && !defined(_SEM_SEMUN_UNDEFINED) || defined(__APPLE__) 
     /* union semun is defined by including <sys/sem.h> */
#elif !defined(USECOND)
     /* according to X/OPEN we have to define it ourselves */
   union semun {
       int val;                    /* value for SETVAL */
       struct semid_ds *buf;       /* buffer for IPC_STAT, IPC_SET */
       unsigned short int *array;  /* array for GETALL, SETALL */
       struct seminfo *__buf;      /* buffer for IPC_INFO */
       };
#endif

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
  int dimsSave[6];//only used when a reader
  char dtypeSave;//only used when a reader
  int addRequired;
#ifdef USECOND
  pthread_cond_t *cond;
  pthread_mutex_t *condmutex;
#else
  int semid;
#endif
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
#ifdef USECOND
#define CIRCHDRSIZE(cb) (*((int*)(&cb->mem[48])))
#define CIRCSIGNAL(cb) cb->mem[52]
#define MUTEXSIZE(cb) (*((int*)(&cb->mem[56])))
#define CONDSIZE(cb) (*((int*)(&cb->mem[60])))
#define MUTEX(cb) (((pthread_mutex_t*)(&cb->mem[64])))
#define COND(cb) (((pthread_cond_t*)(&cb->mem[64+MUTEXSIZE(cb)])))
#endif
#define ALIGN 8
#define HSIZE 32 //the mini header size - recorded for each entry, preceeding the data - size, frameno, time, dtype etc.

//circBuf* circAssign(void *mem,int memsize,int semid,int nd, int *dims,char dtype, circBuf *cb);
int circSetAddIfRequired(circBuf *cb,int frameno);
inline int circCheckAddRequired(circBuf *cb);
int circAdd(circBuf *cb,void *data,double timestamp,int frameno);
int circAddForce(circBuf *cb,void *data,double timestamp,int frameno);
//int circAddPartial(circBuf *cb,void *data,int offset,int size,double timestamp,int frameno);
int circReshape(circBuf *cb,int nd, int *dims,char dtype);
int calcDatasize(int nd,int *dims,char dtype);
int circAddSize(circBuf *cb,void *data,int size,int setzero,double timestamp,int frameno);
void *circGetNextFrame(circBuf *cb,float ftimeout,int retry);
int circHeaderUpdated(circBuf *cb);
void *circGetLatestFrame(circBuf *cb);
void *circGetFrame(circBuf *cb,int indx);
circBuf* circOpenBufReader(char *name);
#ifdef USECOND
#else
int circNewSemId(char *name,int create);
#endif
circBuf* openCircBuf(char *name,int nd,int *dims,char dtype,int nstore);
void circClose(circBuf *cb);//should be called by the owner (writer) of the buf
int circCalcHdrSize();



#endif
