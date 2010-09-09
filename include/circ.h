#ifndef _CIRC_H
#define _CIRC_H
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
#define MUTEXSIZE(cb) (*((int*)(&cb->mem[52])))
#define CONDSIZE(cb) (*((int*)(&cb->mem[56])))
#define MUTEX(cb) (((pthread_mutex_t*)(&cb->mem[60])))
#define COND(cb) (((pthread_cond_t*)(&cb->mem[60+MUTEXSIZE(cb)])))
#endif
#define ALIGN 8
#define HSIZE 32 //the mini header size - recorded for each entry, preceeding the data - size, frameno, time, dtype etc.

//circBuf* circAssign(void *mem,int memsize,int semid,int nd, int *dims,char dtype, circBuf *cb);
int circAdd(circBuf *cb,void *data,double timestamp,int frameno);
int circAddPartial(circBuf *cb,void *data,int offset,int size,double timestamp,int frameno);
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





#endif
