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
#define USECOND
#define MAINGITID "$Id$"
//#include "darccore.c"

#define _GNU_SOURCE
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <stdlib.h>
#include <dlfcn.h>
#include <unistd.h>
#include <limits.h>
#include <sys/types.h>
#include <sys/signal.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <assert.h>
#include <math.h>
#include <sys/mman.h>
#include <pthread.h>
//#include <fftw3.h>
#include <signal.h>
#include "darc.h"
#include "darcNames.h"
//Code which uses core.c for a stand alone RTC...
//ie opens the shared memory etc itself.

paramBuf *openParamBuf(char *name,int size,int block,int nhdr){
  //open shared memory file of name name, size size, and set a semaphore to value of block.
  int fd;
  paramBuf *pb;
  char *buf;
#ifdef USECOND
  pthread_mutexattr_t mutexattr;
  pthread_condattr_t condattr;
#else
  union semun argument;
#endif
  if(shm_unlink(name)){
    printf("unlink failed: %s\n",strerror(errno));
  }
  umask(0);
  if((fd=shm_open(name,O_RDWR|O_CREAT,0777))==-1){
    printf("shm_open failed for %s:%s\n",name,strerror(errno));
    return NULL;
  }
  if(ftruncate(fd,size)==-1){
    printf("ftruncate failed: %s %s\n",name,strerror(errno));
    close(fd);
    return NULL;
  }
  buf=(char*)mmap(0,size,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
  close(fd);
  if(buf==MAP_FAILED){
    printf("mmap failed %s:%s\n",name,strerror(errno));
    return NULL;
  }
  printf("Setting buf to zero %p size %d\n",buf,size);
  memset(buf,0,size); 
  printf("Set to zero\n");
  if((pb=malloc(sizeof(paramBuf)))==NULL){
    printf("Malloc of paramBuf failed %s: %s\n",name,strerror(errno));
    return NULL;
  }
#ifdef USECOND
  pb->arr=buf;
  //buffer has a header with hdrsize(4),nhdr(4),flags(4),mutexsize(4),condsize(4),mutex(N),cond(N),spare bytes for alignment purposes.
  pb->hdr=(int*)pb->arr;
  pb->hdr[0]=4+4+4+4+4+sizeof(pthread_cond_t)+sizeof(pthread_mutex_t);
  //just make sure that buf (&pb->arr[pb->hdr[0]]) is 16 byte aligned
  pb->hdr[0]+=(16-((((unsigned long)pb->arr)+pb->hdr[0])&0xf))%16;
  pb->hdr[1]=nhdr;
  pb->hdr[2]=block;
  pb->hdr[3]=sizeof(pthread_mutex_t);
  pb->hdr[4]=sizeof(pthread_cond_t);
  pb->condmutex=(pthread_mutex_t*)&(pb->arr[20]);
  pb->cond=(pthread_cond_t*)&(pb->arr[20+pb->hdr[3]]);
  pb->buf=&pb->arr[pb->hdr[0]];
  pb->dtype=&pb->buf[pb->hdr[1]*16];
  pb->start=(int*)(&pb->buf[pb->hdr[1]*17]);
  pb->nbytes=(int*)(&pb->buf[pb->hdr[1]*21]);

  pthread_mutexattr_init(&mutexattr);
  pthread_mutexattr_setpshared(&mutexattr,PTHREAD_PROCESS_SHARED);
  pthread_mutexattr_setrobust_np(&mutexattr,PTHREAD_MUTEX_ROBUST);

  pthread_mutex_init(pb->condmutex,&mutexattr);//darc should never try to lock this mutex.  All it should do is broadcast on the cond (without locking the mutex).
  pthread_mutexattr_destroy(&mutexattr);
  pthread_condattr_init(&condattr);
  pthread_condattr_setpshared(&condattr,PTHREAD_PROCESS_SHARED);
  pthread_cond_init(pb->cond,&condattr);
  pthread_condattr_destroy(&condattr);
#else
  pb->buf=buf;
  //now do the semaphore...
  pb->semid=circNewSemId(name,1);
  if(pb->semid<0)
    return NULL;
  argument.val=block;//initialise it so that things can write to the buffer.
  if(semctl(pb->semid,0,SETVAL,argument)==-1){
    printf("semctl failed %s: %s\n",name,strerror(errno));
    return NULL;
  }
#endif
  printf("Opened %s\n",name);
  return pb;
}


int waitBufferValid(paramBuf *pb,int *ncam,int **ncamThreads){
  //wait until the buffer has valid contents, and contains at least info to initialise the rtc.  This is ncam and ncamThreads.  Returns 1 if a buffer has initialised but doesn't contains there, or 0 if initialised and does contain these.
  int ready=0;
  int ncamthrindx=-1;
  int gotncam=0,gotncamthreads=0;
  int j;
  char *buf=pb->buf;
  while(ready==0){
    //wait for buffer to be readable.
    if(buf[0]!='\0'){
      ready=1;
    }
    usleep(10000);
  }
  printf("Reading buffer...\n");
  //now see if the buffer contains what we need to set up the stuff...
  //This is ncam, and ncamThreads.
  j=0;
  while(j<BUFNHDR(pb) && buf[j*16]!='\0'){
    if(strncmp(&buf[j*16],"ncam",16)==0){
      if(pb->dtype[j]=='i' && pb->nbytes[j]==sizeof(int)){
	gotncam=1;
	*ncam=*((int*)BUFGETVALUE(pb,j));//(buf+START[j]));
      }else{
	printf("ncam error\n");
      }
    }else if(strncmp(&buf[j*16],"ncamThreads",16)==0){
      ncamthrindx=j;
    }
    j++;
  }
  if(ncamthrindx>=0 && gotncam==1){
    j=ncamthrindx;
    if(pb->dtype[j]=='i' && pb->nbytes[j]==sizeof(int)*(*ncam)){
      gotncamthreads=1;
      *ncamThreads=((int*)BUFGETVALUE(pb,j));//(buf+START[j]));
    }
  }
  printf("gotncam: %d, gotncamthreads %d\n",gotncam,gotncamthreads);
  return ((gotncam&gotncamthreads)==0);
}

int isSwitchRequested(paramBuf *pb){
  //return 1 if switch requested.
  char *buf=pb->buf;
  int j,sr=0;
  j=0;
  while(j<BUFNHDR(pb) && buf[j*16]!='\0'){
    if(strncmp(&buf[j*16],"switchRequested",16)==0){
      if(pb->dtype[j]=='i' && pb->nbytes[j]==sizeof(int)){
	sr=*((int*)(BUFGETVALUE(pb,j)));//buf+START[j]));
      }else{
	printf("switchRequested error\n");
      }
    }
    j++;
  }
  return sr;
}

int shmUnlink(char *prefix,char *name){
  char *buf;
  if(asprintf(&buf,"/%s%s",prefix,name)==-1){
    printf("unable to get name %s%s\n",prefix,name);
  }else{
    if(shm_unlink(buf))
      printf("Unable to unlink /dev/shm%s\n",buf);
    free(buf);
  }
  return 0;
}

int removeSharedMem(char *prefix){
  shmUnlink(prefix,"rtcParam1");
  shmUnlink(prefix,"rtcParam2");
  shmUnlink(prefix,"rtcPxlBuf");
  shmUnlink(prefix,"rtcCalPxlBuf");
  //shmUnlink(prefix,"rtcCorrBuf");
  shmUnlink(prefix,"rtcCentBuf");
  shmUnlink(prefix,"rtcMirrorBuf");
  shmUnlink(prefix,"rtcActuatorBuf");
  shmUnlink(prefix,"rtcStatusBuf");
  shmUnlink(prefix,"rtcTimeBuf");
  shmUnlink(prefix,"rtcErrorBuf");
  shmUnlink(prefix,"rtcSubLocBuf");
  shmUnlink(prefix,"rtcGenericBuf");
  shmUnlink(prefix,"rtcFluxBuf");
  return 0;
}
#ifdef USECOND
//#define REMSEM(a) if(a!=NULL){pthread_cond_destroy(a->cond);pthread_mutex_destroy(a->condmutex);}
#define REMSEM(a)
#else
#define REMSEM(a) if(a!=NULL){semctl(a->semid,0,IPC_RMID);snprintf(tmp,80,"remove %d\n",a->semid);if(write(fd,tmp,strlen(tmp))==-1)printf("Error writing semid\n");}
#endif
int removeSemaphores(globalStruct *glob){
#ifdef USECOND
#else
  int fd;
  char tmp[80];
  fd=open("/tmp/semid.txt",O_RDWR|O_CREAT|O_APPEND,0777);
  if(fd<=0)
    printf("Error opening semid.txt\n");
  //snprintf(tmp,80,"%d %s\n",semid,sname);
  //write(fd,tmp,strlen(tmp));
#endif
  
  if(glob!=NULL){
    printf("removing mutexes\n");
    if(glob->buffer!=NULL){
      REMSEM(glob->buffer[0]);//->semid,0,IPC_RMID);
      REMSEM(glob->buffer[1]);//->semid,0,IPC_RMID);
    }
    REMSEM(glob->rtcPxlBuf);//->semid,0,IPC_RMID);
    REMSEM(glob->rtcCalPxlBuf);//->semid,0,IPC_RMID);
    //REMSEM(glob->rtcCorrBuf);//->semid,0,IPC_RMID);
    REMSEM(glob->rtcCentBuf);//->semid,0,IPC_RMID);
    REMSEM(glob->rtcMirrorBuf);//->semid,0,IPC_RMID);
    REMSEM(glob->rtcActuatorBuf);//->semid,0,IPC_RMID);
    REMSEM(glob->rtcStatusBuf);//->semid,0,IPC_RMID);
    REMSEM(glob->rtcTimeBuf);//->semid,0,IPC_RMID);
    REMSEM(glob->rtcErrorBuf);//->semid,0,IPC_RMID);
    REMSEM(glob->rtcSubLocBuf);//->semid,0,IPC_RMID);
    REMSEM(glob->rtcGenericBuf);//->semid,0,IPC_RMID);
    REMSEM(glob->rtcFluxBuf);//->semid,0,IPC_RMID);
  }
#ifdef USECOND
#else
  close(fd);
#endif
  printf("Removed mutexes\n");
  return 0;
}

void *delayedExit(void *n){
  sleep(60);
  printf("Delayed exit calling exit(1) for darcmain\n");
  exit(1);
  return NULL;
}

void closeLibraries(globalStruct *glob){
  pthread_t t;
  glob->go=0;
  if(glob->ncpu!=0){
    printf("Starting exit thread\n");
    pthread_create(&t,NULL,delayedExit,NULL);
    printf("Closing libraries...\n");
    //what to do about resource contention?  i.e. if closing the libraries causes a mutex_lock on an already locked mutex?
    //Simple - start a thread that will call exit after a delay.
    openLibraries(glob,0);
    printf("Libraries closed...\n");
  }
}

char globalSHMPrefix[80];
globalStruct *globalGlobStruct;
void handleInterrupt(int sig){
  printf("Signal %d received, prefix %s\n",sig,globalSHMPrefix);
  if(globalGlobStruct->signalled==0){
    globalGlobStruct->signalled=1;
    removeSharedMem(globalSHMPrefix);
    removeSemaphores(globalGlobStruct);
    closeLibraries(globalGlobStruct);
  }
  printf("Exiting...\n");
  exit(1);
}
void handleIgnoreInterrupt(int sig){
  printf("Signal %d received and ignored, prefix %s\n",sig,globalSHMPrefix);
}

void *rotateLog(void *n){
  char **stdoutnames=NULL;
  int nlog=4;
  int logsize=1024*1024;
  FILE *fd;
  char *shmPrefix=(char*)n;
  struct stat st; 
  int i;
  stdoutnames=calloc(nlog,sizeof(char*));
  for(i=0; i<nlog; i++){
    if(asprintf(&stdoutnames[i],"/dev/shm/%sstdout%d",shmPrefix,i)<0){
      printf("rotateLog filename creation failed\n");
      return NULL;
    }
  }
  printf("redirecting stdout to /dev/shm/%sstdout...\n",shmPrefix);
  fd=freopen(stdoutnames[0],"a+",stdout);
  setvbuf(fd,NULL,_IOLBF,0);
  printf("rotateLog started\n");
  printf("New log cycle\n");
  while(1){
    sleep(60);
    fstat(fileno(fd),&st);
    if(st.st_size>logsize){
      printf("LOGROTATE\n");
      for(i=nlog-1; i>0; i--){
	rename(stdoutnames[i-1],stdoutnames[i]);
      }
      fd=freopen(stdoutnames[0],"w",stdout);
      setvbuf(fd,NULL,_IOLBF,0);
      printf("New log cycle\n");
    }
  }
}
void *runPrepareActuators(void *glob){
  prepareActuators((globalStruct*)glob);
  return NULL;
}
void *startThreadFunc(void *t){
  processFrame((threadStruct*)t);
  return NULL;
}

char* initParamNames(){//char paramNames[NBUFFERVARIABLES][16]){
    char *paramNames;
    int i;
    if((i=posix_memalign((void**)&paramNames,16,16*NBUFFERVARIABLES))!=0){
      printf("Error in initParamNames\n");
    }
    memset(paramNames,0,16*NBUFFERVARIABLES);
    strncpy(&paramNames[NCAM*16],"ncam",16);
    strncpy(&paramNames[NACTS*16],"nacts",16);
    strncpy(&paramNames[NSUB*16],"nsub",16);
    strncpy(&paramNames[NPXLX*16],"npxlx",16);
    strncpy(&paramNames[NPXLY*16],"npxly",16);
    //strncpy(&paramNames[REFCENTROIDS*16],"refCentroids",16);
    strncpy(&paramNames[SUBAPLOCATION*16],"subapLocation",16);
    strncpy(&paramNames[SUBAPLOCTYPE*16],"subapLocType",16);
    //strncpy(&paramNames[BGIMAGE*16],"bgImage",16);
    //strncpy(&paramNames[DARKNOISE*16],"darkNoise",16);
    //strncpy(&paramNames[FLATFIELD*16],"flatField",16);
    //strncpy(&paramNames[THRESHOLDALGO*16],"thresholdAlgo",16);
    //strncpy(&paramNames[THRESHOLDVALUE*16],"thresholdValue",16);
    //strncpy(&paramNames[POWERFACTOR*16],"powerFactor",16);
    //strncpy(&paramNames[CENTROIDWEIGHT*16],"centroidWeight",16);
    strncpy(&paramNames[WINDOWMODE*16],"windowMode",16);
    strncpy(&paramNames[SUBAPFLAG*16],"subapFlag",16);
    strncpy(&paramNames[GO*16],"go",16);
    strncpy(&paramNames[PXLCNT*16],"pxlCnt",16);
    //strncpy(&paramNames[CENTROIDMODE*16],"centroidMode",16);
    strncpy(&paramNames[PAUSE*16],"pause",16);
    strncpy(&paramNames[PRINTTIME*16],"printTime",16);
    strncpy(&paramNames[NCAMTHREADS*16],"ncamThreads",16);
    strncpy(&paramNames[SWITCHREQUESTED*16],"switchRequested",16);
    strncpy(&paramNames[ACTUATORS*16],"actuators",16);
    //strncpy(&paramNames[FAKECCDIMAGE*16],"fakeCCDImage",16);
    strncpy(&paramNames[THREADAFFINITY*16],"threadAffinity",16);
    strncpy(&paramNames[THREADPRIORITY*16],"threadPriority",16);
    strncpy(&paramNames[DELAY*16],"delay",16);
    strncpy(&paramNames[MAXCLIPPED*16],"maxClipped",16);
    strncpy(&paramNames[CLEARERRORS*16],"clearErrors",16);
    strncpy(&paramNames[CAMERASOPEN*16],"camerasOpen",16);
    //strncpy(&paramNames[CAMERASFRAMING*16],"camerasFraming",16);
    strncpy(&paramNames[CAMERAPARAMS*16],"cameraParams",16);
    strncpy(&paramNames[CAMERANAME*16],"cameraName",16);
    strncpy(&paramNames[MIRROROPEN*16],"mirrorOpen",16);
    strncpy(&paramNames[MIRRORNAME*16],"mirrorName",16);
    strncpy(&paramNames[FRAMENO*16],"frameno",16);
    strncpy(&paramNames[SWITCHTIME*16],"switchTime",16);
    //strncpy(&paramNames[ADAPTIVEWINGAIN*16],"adaptiveWinGain",16);
    //strncpy(&paramNames[CORRTHRESHTYPE*16],"corrThreshType",16);
    //strncpy(&paramNames[CORRTHRESH*16],"corrThresh",16);
    //strncpy(&paramNames[CORRFFTPATTERN*16],"corrFFTPattern",16);
    strncpy(&paramNames[NSTEPS*16],"nsteps",16);
    strncpy(&paramNames[CLOSELOOP*16],"closeLoop",16);
    strncpy(&paramNames[MIRRORPARAMS*16],"mirrorParams",16);
    strncpy(&paramNames[ADDACTUATORS*16],"addActuators",16);
    strncpy(&paramNames[ACTSEQUENCE*16],"actSequence",16);
    strncpy(&paramNames[RECORDCENTS*16],"recordCents",16);
    //strncpy(&paramNames[PXLWEIGHT*16],"pxlWeight",16);
    strncpy(&paramNames[AVERAGEIMG*16],"averageImg",16);
    strncpy(&paramNames[SLOPEOPEN*16],"slopeOpen",16);
    //strncpy(&paramNames[CENTFRAMING*16],"centFraming",16);
    strncpy(&paramNames[SLOPEPARAMS*16],"slopeParams",16);
    strncpy(&paramNames[SLOPENAME*16],"slopeName",16);
    strncpy(&paramNames[ACTUATORMASK*16],"actuatorMask",16);
    strncpy(&paramNames[AVERAGECENT*16],"averageCent",16);
    //strncpy(&paramNames[CALMULT*16],"calmult",16);
    //strncpy(&paramNames[CALSUB*16],"calsub",16);
    //strncpy(&paramNames[CALTHR*16],"calthr",16);
    //strncpy(&paramNames[CENTCALDATA*16],"centCalData",16);
    //strncpy(&paramNames[CENTCALBOUNDS*16],"centCalBounds",16);
    //strncpy(&paramNames[CENTCALSTEPS*16],"centCalSteps",16);
    strncpy(&paramNames[FIGUREOPEN*16],"figureOpen",16);
    strncpy(&paramNames[FIGURENAME*16],"figureName",16);
    strncpy(&paramNames[FIGUREPARAMS*16],"figureParams",16);
    strncpy(&paramNames[RECONNAME*16],"reconName",16);
    //strncpy(&paramNames[FLUXTHRESHOLD*16],"fluxThreshold",16);
    strncpy(&paramNames[PRINTUNUSED*16],"printUnused",16);
    //strncpy(&paramNames[USEBRIGHTEST*16],"useBrightest",16);
    strncpy(&paramNames[FIGUREGAIN*16],"figureGain",16);
    strncpy(&paramNames[RECONLIBOPEN*16],"reconlibOpen",16);
    //strncpy(&paramNames[MAXADAPOFFSET*16],"maxAdapOffset",16);
    strncpy(&paramNames[VERSION*16],"version",16);
    strncpy(&paramNames[CURRENTERRORS*16],"currentErrors",16);
    strncpy(&paramNames[RECONPARAMS*16],"reconParams",16);
    //strncpy(&paramNames[ADAPTIVEGROUP*16],"adaptiveGroup",16);
    strncpy(&paramNames[CALIBRATENAME*16],"calibrateName",16);
    strncpy(&paramNames[CALIBRATEPARAMS*16],"calibrateParams",16);
    strncpy(&paramNames[CALIBRATEOPEN*16],"calibrateOpen",16);
    strncpy(&paramNames[ITERSOURCE*16],"iterSource",16);
    strncpy(&paramNames[BUFFERNAME*16],"bufferName",16);
    strncpy(&paramNames[BUFFERPARAMS*16],"bufferParams",16);
    strncpy(&paramNames[BUFFEROPEN*16],"bufferOpen",16);
    strncpy(&paramNames[BUFFERUSESEQ*16],"bufferUseSeq",16);
    strncpy(&paramNames[NOPREPOSTTHREAD*16],"noPrePostThread",16);
    strncpy(&paramNames[SUBAPALLOCATION*16],"subapAllocation",16);
    strncpy(&paramNames[OPENLOOPIFCLIP*16],"openLoopIfClip",16);
    strncpy(&paramNames[ADAPWINSHIFTCNT*16],"adapWinShiftCnt",16);
    return paramNames;
}
int main(int argc, char **argv){
  //first open the shared memory buffers, rtcParam1 and rtcParam2.
  //then open the circular buffers
  //determine the number of threads for each camera and number of cameras 
  //from the buffer (once initialised).
  //Determine the number of iterations from argc.
  int niters=-1;
  paramBuf *rtcbuf[2];
  globalStruct *glob;
  int curbuf=0;
  int ncam,*ncamThreads;
  int err=0;
  int nthreads,i,j,threadno,nthread;
  infoStruct *info;//,*info2;
  threadStruct *tinfo;
  struct timeval t1,t2;
  double tottime;
  int bufsize=-1;
  char *shmPrefix=NULL;
  char *bufname;
  struct sigaction sigact;
  int ignoreKeyboardInterrupt=0;
  char *buffile=NULL;
  int redirect=0;
  pthread_t logid;
  pthread_t figureThreadID;
  int dim;
  char *tmp;
  struct sched_param schedParam;
  int nhdr=128;
  globalGlobStruct=NULL;

  //args are -nNITERS -bBUFSIZE -sSHMPREFIX -i (to ignore keyboard interrupt) -fFILENAME.FITS to initialise with a fits file buffer (not yet implemented).
  for(i=1; i<argc; i++){
    if(argv[i][0]=='-'){
      switch(argv[i][1]){
      case 'n':
	niters=atoi(&argv[i][2]);
	break;
      case 'b':
	bufsize=atoi(&argv[i][2]);
	break;
      case 's':
	shmPrefix=&argv[i][2];
	break;
      case 'i':
	ignoreKeyboardInterrupt=1;
	break;
      case 'f':
	buffile=&argv[i][2];
	break;
      case 'h':
	printf("Usage: %s -nNITERS -bBUFSIZE -sSHMPREFIX -fFILENAME.FITS (not yet implemented) -i (to ignore keyboard interrupt) -r (to redirect stdout) -eNHDR\n",argv[0]);
	exit(0);
	break;
      case 'r':
	redirect=1;
	break;
      case 'e':
	nhdr=atoi(&argv[i][2]);
	break;
      default:
	printf("Unrecognised argument %s\n",argv[i]);
	break;
      }
    }else{
      printf("Unrecognised argument %s\n",argv[i]);
    }
  }
  if(bufsize<0){
    bufsize=64*1024*1024;
  }
  if((glob=malloc(sizeof(globalStruct)))==NULL){
    printf("glob malloc\n");
    return -1;
  }
  memset(glob,0,sizeof(globalStruct));
  glob->paramNames=initParamNames();//defined in darcNames.h... fill in paramNames array.
  if(bufferCheckNames(NBUFFERVARIABLES,glob->paramNames)){
    printf("Exiting\n");
    exit(0);
  }
  if((err=pthread_mutex_init(&glob->libraryMutex,NULL))!=0){
    printf("Failed libraryMutex\n");
    exit(0);
  }

  if(mlockall(MCL_CURRENT|MCL_FUTURE)==-1){
    printf("mlockall failed (you need to be running as root): %s (note this probably makes no performance difference if you aren't swapping)\n",strerror(errno));
  }
  if(shmPrefix==NULL){
    shmPrefix=strdup("\0");
    rtcbuf[0]=openParamBuf("/rtcParam1",bufsize,1,nhdr);
    rtcbuf[1]=openParamBuf("/rtcParam2",bufsize,0,nhdr);
  }else{
    if(asprintf(&bufname,"/%srtcParam1",shmPrefix)==-1){
      printf("Couldn't allocate name\n");
      exit(1);
    }
    rtcbuf[0]=openParamBuf(bufname,bufsize,1,nhdr);
    free(bufname);
    if(asprintf(&bufname,"/%srtcParam2",shmPrefix)==-1){
      printf("couldn't allocate name\n");
      exit(1);
    }
    rtcbuf[1]=openParamBuf(bufname,bufsize,0,nhdr);
    free(bufname);
  }
  strncpy(globalSHMPrefix,shmPrefix,79);
  globalSHMPrefix[79]='\0';

  if(redirect){//redirect stdout to a file...
    if(pthread_create(&logid,NULL,rotateLog,shmPrefix)){
      printf("pthread_create rotateLog failed\n");
      return -1;
    }
  }

  //probably put all of this in a loop, including the pthreads_join.  That way, if a thread detects that ncam has changed, the threads can exit, the system can reinitialise, and start again..  In this case, have to think about which buffer to wait upon.
  if(rtcbuf[0]==NULL || rtcbuf[1]==NULL){
    printf("rtc shared memory buffer error: exiting\n");
    return -1;
  }
  glob->mainGITID=MAINGITID;
  glob->buffer[0]=rtcbuf[0];
  glob->buffer[1]=rtcbuf[1];
  glob->bufferHeaderIndex=calloc(sizeof(int),NBUFFERVARIABLES);
  glob->bufferNbytes=calloc(sizeof(int),NBUFFERVARIABLES);
  glob->bufferDtype=calloc(sizeof(char),NBUFFERVARIABLES);
  glob->bufferValues=calloc(sizeof(void*),NBUFFERVARIABLES);
  //glob->paramNames=paramNames;//global defined in darcNames.h
  globalGlobStruct=glob;
  if((glob->arrays=malloc(sizeof(arrayStruct)))==NULL){
    printf("arrays malloc\n");
    return -1;
  }
  memset(glob->arrays,0,sizeof(arrayStruct));
  /*if((glob->arrays[1]=malloc(sizeof(arrayStruct)))==NULL){
    printf("arrays[1] malloc\n");
    return -1;
  }
  memset(glob->arrays[1],0,sizeof(arrayStruct));*/
  glob->go=1;
  //glob->fftIndexSize=16;
  //if((glob->fftIndex=malloc(sizeof(int)*2*glob->fftIndexSize))==NULL){
  //  printf("fftIndex malloc\n");
  //  return -1;
  //}
  //memset(glob->fftIndex,0,sizeof(int)*2*glob->fftIndexSize);
  //if((glob->fftPlanArray=malloc(sizeof(fftwf_plan)*2*glob->fftIndexSize))==NULL){
  //  printf("fftPlanArray malloc\n");
  //  return -1;
  // }
  //memset(glob->fftPlanArray,0,sizeof(fftwf_plan)*2*glob->fftIndexSize);


  //glob->fftPlanArrayXsize=MAXSUBAPSIZE;
  /*if((glob->fftPlanArray=malloc(sizeof(fftwf_plan)*glob->fftPlanArrayXsize*glob->fftPlanArrayXsize))==NULL){
    printf("fftPlanArray malloc\n");
    return -1;
  }
  memset(glob->fftPlanArray,0,sizeof(fftwf_plan)*glob->fftPlanArrayXsize*glob->fftPlanArrayXsize);
  if((glob->ifftPlanArray=malloc(sizeof(fftwf_plan)*glob->fftPlanArrayXsize*glob->fftPlanArrayXsize))==NULL){
    printf("ifftPlanArray malloc\n");
    return -1;
  }
  memset(glob->ifftPlanArray,0,sizeof(fftwf_plan)*glob->fftPlanArrayXsize*glob->fftPlanArrayXsize);*/
  if((glob->precomp=malloc(sizeof(PreComputeData)))==NULL){
    printf("precomp malloc\n");
    return -1;
  }
  memset(glob->precomp,0,sizeof(PreComputeData));
  glob->precomp->post.go=1;
  //Now create FFTW plans...
  //pthread_create(&glob->fftPlanThreadid,NULL,doFFTPlanning,glob);

  sigact.sa_flags=0;
  sigemptyset(&sigact.sa_mask);
  sigact.sa_handler=handleInterrupt;
  if(sigaction(SIGSEGV,&sigact,NULL)!=0)
    printf("Error calling sigaction SIGSEGV\n");
  if(sigaction(SIGBUS,&sigact,NULL)!=0)
    printf("Error calling sigaction SIGBUS\n");
  if(sigaction(SIGTERM,&sigact,NULL)!=0)
    printf("Error calling sigaction SIGTERM\n");
  if(ignoreKeyboardInterrupt==0){
    if(sigaction(SIGINT,&sigact,NULL)!=0)
      printf("Error calling sigaction SIGINT\n");
  }else{
    sigact.sa_handler=handleIgnoreInterrupt;
    if(sigaction(SIGINT,&sigact,NULL)!=0)
      printf("Error calling sigaction SIGINT\n");
  }
  err=1;
  printf("Waiting for valid buffer contents in /%srtcParam%d shared memory (globalStruct %d)\n",shmPrefix,curbuf+1,(int)sizeof(globalStruct));
  while(err==1){
    printf("WaitBufferValid curbuf=%d\n",curbuf);
    err=waitBufferValid(rtcbuf[curbuf],&ncam,&ncamThreads);//wait for another process to write to the SHM
    printf("err=%d\n",err);
    if(isSwitchRequested(rtcbuf[curbuf])){
      printf("Switching buffers\n");
      curbuf=1-curbuf;
#ifdef USECOND
      freezeParamBuf(rtcbuf[curbuf],rtcbuf[1-curbuf]);
#else
      freezeParamBuf(rtcbuf[curbuf]->semid,rtcbuf[1-curbuf]->semid);
#endif
      err=1;
    }else if(err){//if buffer isn't ready, try the other buffer, as the buffer writing process may be writing to the other buffer.
      printf("Waiting for shared parameter buffer /rtcParam%d to become valid\n",curbuf+1);
      usleep(10000);
    }
  }
  printf("Got valid buffer contents, ncam=%d curBuf=%d\n",ncam,curbuf);
  gettimeofday(&t1,NULL);
  dim=ERRORBUFSIZE;
  if(glob->rtcErrorBuf==NULL){
    printf("Creating rtcErrorBuf\n");
    if(asprintf(&tmp,"/%srtcErrorBuf",shmPrefix)==-1)
      exit(1);
    glob->rtcErrorBuf=openCircBuf(tmp,1,&dim,'b',10);
    FREQ(glob->rtcErrorBuf)=1;
    free(tmp);
  }

  /*if((glob->updateBuf=malloc(sizeof(int)*ncam))==NULL){
    printf("updateBuf malloc\n");
    return -1;
  }
  memset(glob->updateBuf,0,sizeof(int)*ncam);
  */
  glob->niters=niters;
  glob->ncpu=sysconf(_SC_NPROCESSORS_ONLN);
  if(glob->ncpu>32){
    printf("Warning - thread affinity currently only used for up to 32 CPUs - you have %d - suggest change code...\n",glob->ncpu);
  }
  //glob->switchRequested=1;
  glob->curBuf=1-curbuf;//this one will then tell the threads to doa buffer swap.
  getSwitchRequested(glob);//a new parameter buffer is ready
  glob->doswitch=1;
  
  schedParam.sched_priority=10;//set a default priority - incase any other threads forget too.
  if(sched_setscheduler(0,SCHED_RR,&schedParam)!=0){
    printf("Error in sched_setscheduler %s - maybe you should run as root?\n",strerror(errno));
  }


  err=0;
  //err|=pthread_cond_init(&glob->bufCond,NULL);
  err|=pthread_cond_init(&glob->frameRunningCond,NULL);
  //err|=pthread_cond_init(&glob->frameRunningCond[1],NULL);
  err|=pthread_cond_init(&glob->precomp->prepCond,NULL);
  err|=pthread_cond_init(&glob->precomp->postCond,NULL);
  //err|=pthread_cond_init(&glob->precomp->dmCond,NULL);
  err|=pthread_cond_init(&glob->calCentCond,NULL);
  err|=pthread_mutex_init(&glob->startMutex,NULL);
  err|=pthread_mutex_init(&glob->startFirstMutex,NULL);
  //err|=pthread_mutex_init(&glob->startMutex[1],NULL);
  err|=pthread_mutex_init(&glob->endMutex,NULL);
  //err|=pthread_mutex_init(&glob->endMutex[1],NULL);
  err|=pthread_mutex_init(&glob->frameRunningMutex,NULL);
  //err|=pthread_mutex_init(&glob->frameRunningMutex[1],NULL);
  //err|=pthread_mutex_init(&glob->bufMutex,NULL);
  err|=pthread_mutex_init(&glob->camMutex,NULL);
  err|=pthread_mutex_init(&glob->precomp->prepMutex,NULL);
  err|=pthread_mutex_init(&glob->precomp->postMutex,NULL);
  //err|=pthread_mutex_init(&glob->precomp->dmMutex,NULL);
  err|=pthread_mutex_init(&glob->calCentMutex,NULL);
  err|=pthread_mutex_init(&glob->precomp->post.actsRequiredMutex,NULL);
  err|=pthread_cond_init(&glob->precomp->post.actsRequiredCond,NULL);
  if(err){
    printf("Error in pthread_cond/mutex_init functions: %s\n",strerror(errno));
    return -1;
  }
  glob->precomp->post.libraryMutex=&glob->libraryMutex;
  glob->calCentReady=1;
  glob->shmPrefix=shmPrefix;
  nthreads=0;
  for(i=0; i<ncam; i++){//get total number of threads
    nthreads+=ncamThreads[i];//((int*)nthreadsArr->data)[i];
    //glob->updateBuf[i]=1;
  }
  glob->nthreads=nthreads;
  //dims=nthreads;
  if((glob->threadInfoHandle=malloc(nthreads*sizeof(void*)))==NULL){
    printf("threadInfoHandle malloc\n");
    return -1;
  }
  if((glob->threadList=malloc(sizeof(pthread_t)*nthreads))==NULL){
    printf("threadList malloc\n");
    return -1;
  }
  if((glob->threadAffinityListPrev=calloc(sizeof(int),nthreads+1))==NULL){
    printf("threadAffinityListPrev malloc\n");
    return -1;
  }
  memset(glob->threadAffinityListPrev,-1,sizeof(int)*(nthreads+1));
  if((glob->threadPriorityListPrev=calloc(sizeof(int),nthreads+1))==NULL){
    printf("threadPriorityListPrev malloc\n");
    return -1;
  }

  if(pthread_create(&glob->precomp->threadid,NULL,runPrepareActuators,glob)){
    printf("pthread_create runPrepareActuators failed\n");
    return -1;
  }
  /*if((glob->camframeno=calloc(ncam,sizeof(int)))==NULL){//malloc
    printf("camframeno malloc failed\n");
    return -1;
  }
  if((glob->centframeno=calloc(ncam,sizeof(int)))==NULL){//malloc
    printf("centframeno malloc failed\n");
    return -1;
    }*/
  glob->ncam=ncam;
  /*if((glob->ncentsList=calloc(ncam,sizeof(int)))==NULL){
    printf("ncentsList malloc failed\n");
    return -1;
    }*/
  threadno=0;
  for(i=0; i<ncam; i++){
    //now start the threads...
    //For each camera, need an info struct...
    if((info=malloc(sizeof(infoStruct)))==NULL){
      printf("info %d malloc\n",i);
      return -1;
    }
    //if((info2=malloc(sizeof(infoStruct)))==NULL){
    //  printf("info2 %d malloc\n",i);
    //  return -1;
    //}
    memset(info,0,sizeof(infoStruct));
    //memset(info2,0,sizeof(infoStruct));
    nthread=ncamThreads[i];
    info->cam=i;
    info->nthreads=nthread;
    info->id=1;
    //info2->cam=i;
    //info2->nthreads=nthread;
    //info2->id=2;
    //info->go=1;
    //info2->go=1;
    err=0;
    err|=pthread_mutex_init(&info->subapMutex,NULL);
    err|=pthread_mutex_init(&info->startInfoMutex,NULL);
    //err|=pthread_mutex_init(&info2->subapMutex,NULL);
    //err|=pthread_mutex_init(&info->reconMVMutex,NULL);
    //err|=pthread_mutex_init(&info2->reconMVMutex,NULL);
    if(err){
      printf("pthread_mutex_init failed (subapMutex or startInfoMutex): %s\n",strerror(errno));
      return -1;
    }
    for(j=0; j<nthread; j++){
      if((tinfo=malloc(sizeof(threadStruct)))==NULL){
	printf("tinfo %d %d malloc\n",j,i);
	return -1;
      }
      glob->threadInfoHandle[threadno]=(void*)tinfo;
      memset(tinfo,0,sizeof(threadStruct));
      //tinfo->subapSize=64*64;//MAXSUBAPSIZE*MAXSUBAPSIZE;
      //if((tinfo->subap=fftwf_malloc(sizeof(float)*tinfo->subapSize))==NULL){//must be freed using fftwf_free.
      //	printf("subap %d %d malloc\n",j,i);
      //return -1;
      //}
      //if((tinfo->sort=malloc(sizeof(float)*tinfo->subapSize))==NULL){
      //printf("sort %d %d malloc\n",j,i);
      //return -1;
      //}

      //091109tinfo->mybuf=1;
      tinfo->info=info;
      //tinfo->infobuf=info2;
      tinfo->globals=glob;
      tinfo->threadno=threadno;
      if(threadno==0){//only do this for the first thread...
	setGITID(glob);
      }
      printf("Starting thread %d %d\n",j,i);
      if(pthread_create(&glob->threadList[threadno],NULL,startThreadFunc,tinfo)){
	printf("pthread_create startThreadFunc failed for threadno %d: %s\n",threadno,strerror(errno));
	return -1;
      }


      threadno++;
    }
  }
  pthread_create(&figureThreadID,NULL,(void *(*)(void*))figureThread,&glob->precomp->post);
  printf("Main thread waiting for RTC to finish\n");
  //and now wait for the threads to finish - ie rtc has finished...
  for(i=0; i<glob->nthreads; i++){
    pthread_join(glob->threadList[i],NULL);
  }
  //now wake up the figure sensor thread, so that it stops...
  glob->precomp->post.go=0;
  pthread_cond_signal(&glob->precomp->post.actsRequiredCond);
  printf("Waiting for figureThread\n");
  pthread_join(figureThreadID,NULL);
  gettimeofday(&t2,NULL);
  tottime=t2.tv_sec-t1.tv_sec+(t2.tv_usec-t1.tv_usec)*1e-6;
  //printf("Done core for %d iters, time %gs, %gs per iter, %gHz\n",niters,tottime,tottime/niter,niter/tottime);
  printf("Done core for %d iters, time %gs, %gs per iter, %gHz\n",niters,tottime,tottime/niters,niters/tottime);
  removeSemaphores(glob);
  removeSharedMem(glob->shmPrefix);

  //pthread_kill(glob->precomp->threadid,SIGKILL);
  //pthread_join(glob->precomp->threadid,NULL);
  return 0;
}
