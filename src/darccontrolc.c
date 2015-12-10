
/*Offers a cut down version of darccontrol, so that darc can be operated without python - i.e. entirely in c.  
For operation on embedded systems, accelerators, etc

Offers a subset of the functionality of darccontrol, but enough for an operational system.

Once running, listens to incoming connections on a network port.

Commandline args include:
prefix
FITS filename for parameter buffer
Port number
Buffer size
Param size
nhdr.

Operational commands, sent over a socket, include:
set
get
decimate
startSender
init
initFile
stop
stopServer

*/
#define _GNU_SOURCE
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdarg.h>
#include <string.h>
#include <arpa/inet.h>
#include <argp.h>
#include <time.h>
#include <pthread.h>
#include <poll.h>
#include <signal.h>
#include "buffer.h"
#include "circ.h"

const char *argp_program_version =
  "darccontrolc v0.1";
const char *argp_program_bug_address =
  "<a.g.basden@durham.ac.uk>";

/* Program documentation. */
static char doc[] =
  "darccontrolc - a simplified control daemon for darc";

/* A description of the arguments we accept. */
static char args_doc[] = "ARG1 ARG2";

/* The options we understand. */
static struct argp_option options[] = {
  {"prefix",   's', "string",      0,  "Prefix for darc"},
  {"port",     'p', "int",      0,  "Port number to listen on"},
  {"affin",    'a', "int",      0,  "Thread affinity"},
  {"prio",     'i', "int",      0,  "Thread priority"},
  {"output",   'o', 0,      0,  "Don't redirect the output of darcmain"},
  {"buffersize",'b',"int",      0,  "Set the buffer size"},
  {"nhdr",     'e', "int",      0,  "NHDR size"},
  {"configfile",'f',"FILE",      0,  "Config file name"},
  {"nstore",   'c', "string",      0,  "Stream name and number of circular buffer entries, e.g. -c rtcPxlBuf=5 -c rtcCalPxlBuf=10 etc"},
  {"circBufMemSize",'m',"int",  0,  "Memory for circular buffers"},
  { 0 }
};

typedef enum {DARCSET=1,DARCGET,DARCDECIMATE,DARCSENDER,DARCSTREAM,DARCSTOP,DARCCONTROLSTOP,DARCINIT,DARCINITFILE} darcCommands;

/* Used by main to communicate with parse_opt. */
struct arguments
{
  char *prefix;
  int port;
  unsigned long affin;
  int prio;
  int output;
  long bufsize;
  int nhdr;
  char *configfile;
  char *nstore;
  long circBufMemSize;
  char **darcargs;//store the arguments.
  int curdarcarg;//pointer to where we're at.
  int darcargsize;//keep size of the array.
};

typedef struct{
  struct arguments options;
  paramBuf *bufList[2];
  int corestarted;
  int lsocket;
  //uint16_t port;
  int go;
  pid_t coremain;
}ControlStruct;

typedef struct{
  ControlStruct *c;
  pthread_t threadid;
  int sock;
}ThreadStruct;


int setdarcarg(struct arguments *args,char *fmt,...){
  va_list ap;
  int rt=0;
  int l;
  char **tmp;
  if(args->curdarcarg==args->darcargsize){//need to increase size.
    if((tmp=calloc(sizeof(char*),args->darcargsize+8+1))==NULL){
      //+1 is for terminating NULL
      printf("Error callocing in setdarcarg\n");
      rt=1;
    }else{
      memcpy(tmp,args->darcargs,sizeof(char*)*args->darcargsize);
      args->darcargsize+=8;
      free(args->darcargs);
      args->darcargs=tmp;
    }
  }
  va_start(ap,fmt);
  if(rt==0 && (l=vasprintf(&args->darcargs[args->curdarcarg],fmt,ap))<=0){
    printf("vasprintf error...\n");//should probably do something about this!
    rt=1;
  }else
    args->curdarcarg++;
  va_end(ap);
  return rt;
}


/* Parse a single option. */
static error_t
parse_opt (int key, char *arg, struct argp_state *state)
{
  /* Get the input argument from argp_parse, which we
     know is a pointer to our arguments structure. */
  struct arguments *args = state->input;
  int val;
  char *tmp;
  switch (key)
    {
    case 's':
      args->prefix=arg;
      setdarcarg(args,"-s%s",arg);
      break;
    case 'p':
      args->port=atoi(arg);
      break;
    case 'a':
      args->affin=strtoul(arg,NULL,10);
      break;
    case 'i':
      args->prio=atoi(arg);
      break;
    case 'o':
      args->output=1;
      //setdarcarg(args,"-r");
      break;
    case 'b':
      args->bufsize=atol(arg);
      setdarcarg(args,"-b%d",args->bufsize);
      break;
    case 'e':
      args->nhdr=atoi(arg);
      setdarcarg(args,"-e%d",args->nhdr);
      break;
    case'f':
      args->configfile=arg;
      break;
    case 'c'://arg is e.g. rtcPxlBuf=10
      tmp=strchr(arg,'=');
      if(tmp==NULL){
	printf("Error: nstore for %s not found\nPlease use format -c %s=10\n",arg,arg);
	exit(1);
      }
      tmp[0]='\0';
      val=atoi(&tmp[1]);
      if(args->nstore==NULL){
	if(asprintf(&tmp,"-c %s %d",arg,val)==-1)
	  printf("asprintf error\n");
      }else{
	if(asprintf(&tmp,"%s -c %s %d",args->nstore,arg,val)==-1)
	  printf("asprintf error\n");
      }
      free(args->nstore);
      args->nstore=tmp;
      setdarcarg(args,"-c");
      setdarcarg(args,arg);
      setdarcarg(args,"%d",val);
      break;
    case 'm':
      args->circBufMemSize=atol(arg);
      setdarcarg(args,"-m%ld",args->circBufMemSize);
      break;
    case ARGP_KEY_ARG:
      //if (state->arg_num >= 2)   //Too many arguments.
      //  argp_usage (state);
      //args->args[state->arg_num] = arg;
      if(args->configfile==NULL)
	args->configfile=arg;
      else
	printf("Unrecognised argument: %s\n",arg);
      break;

    case ARGP_KEY_END:
      //if (state->arg_num < 2)// Not enough arguments.
      //  argp_usage (state);
      break;

    default:
      return ARGP_ERR_UNKNOWN;
    }
  return 0;
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
    if(asprintf(&stdoutnames[i],"/dev/shm/%srtcCCtrlStdout%d",shmPrefix,i)<0){
      printf("rotateLog filename creation failed\n");
      return NULL;
    }
  }
  printf("redirecting stdout to %s...\n",stdoutnames[0]);
  fd=freopen(stdoutnames[0],"a+",stdout);
  setvbuf(fd,NULL,_IOLBF,0);
  printf("rotateLog started\n");
  printf("New log cycle\n");
  while(1){
    sleep(10);
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

int setSwitchRequested(ControlStruct *c,int preservePause,int wait){
  //By default, you probably want preservePause==1, and wait==0.
  int inactive,active;
  bufferVal *val;
  int i,err,rtval=0;
  struct timespec abstime;
  struct timeval t1;
  inactive=bufferGetInactive(c->bufList);
  if(inactive==-1){
    printf("Oops - no inactive buffer found\n");
    return -1;
  }
  active=1-inactive;
  if(preservePause){
    val=bufferGet(c->bufList[active],"pause");
    if(val==NULL){
      if((val=calloc(sizeof(bufferVal),1))==NULL){
	printf("Error allocing bufferVal\n");//should probably do something!
	return -1;
      }
      i=0;
      val->data=&i;
      val->dtype='i';
      val->size=sizeof(int);
    }
    printf("Setting pause\n");
    bufferSetIgnoringLock(c->bufList[inactive],"pause",val);
  }else{
    if((val=calloc(sizeof(bufferVal),1))==NULL){
      printf("Error allocing bufferVal\n");//should probably do something!
      return -1;
    }
  }
  i=1;
  val->data=&i;
  val->dtype='i';
  val->size=sizeof(int);
  printf("Setting switchRequested\n");
  bufferSetIgnoringLock(c->bufList[active],"switchRequested",val);
  free(val);
  if(wait){
    pthread_mutex_lock(c->bufList[active]->condmutex);
    while((BUFFLAG(c->bufList[active])&1)==1 && rtval==0){
      printf("Waiting for inactive buffer\n");
      gettimeofday(&t1,NULL);
      abstime.tv_sec=t1.tv_sec+1;//add 1 second.
      abstime.tv_nsec=t1.tv_usec*1000;
      err=pthread_cond_timedwait(c->bufList[active]->cond,c->bufList[active]->condmutex,&abstime);
      if(err==ETIMEDOUT){
	printf("Timeout waiting for buffer switch to complete\n");
	rtval=1;
      }else if(err!=0){
	printf("pthread_cond_timedwait failed in darccontrolc\n");
	rtval=1;
      }
    }
    pthread_mutex_unlock(c->bufList[active]->condmutex);
  }
  return 0;
}


int darcstop(ControlStruct *c){
  int bufno;
  paramBuf *b;
  char chr[80];
  int state=0;
  bufferVal val;
  int i;
  bufno=bufferGetInactive(c->bufList);
  b=c->bufList[bufno];
  i=0;
  val.data=&i;
  val.dtype='i';
  val.size=sizeof(int);
  bufferSet(b,"go",&val);
  setSwitchRequested(c,0,0);
  b=*&c->bufList[1-bufno];
  bufferSetIgnoringLock(b,"go",&val);
  //Now remove /dev/shm/*rtcParam1/2.
  snprintf(chr,80,"/dev/shm/%srtcParam1",c->options.prefix);
  unlink(chr);
  snprintf(chr,80,"/dev/shm/%srtcParam2",c->options.prefix);
  unlink(chr);
  //And now wait for darcmain to finish.  Terminate if >10s.
  if(c->coremain!=0){
    for(i=0;i<10;i++){
      printf("Waiting for existing darc to finish cleanly\n");
      if(waitpid(c->coremain,&state,WNOHANG)!=0 && (WIFEXITED(state) || WIFSIGNALED(state)))
	break;
      sleep(1);
    }
    if(i==10){
      printf("Terminating darcmain\n");
      kill(c->coremain,SIGTERM);
      sleep(1);
      waitpid(c->coremain,&state,WNOHANG);
      if(!WIFEXITED(state) && !WIFSIGNALED(state)){
	printf("Not responding - now killing darcmain\n");
	kill(c->coremain,SIGKILL);
      }
    }
  }
  c->coremain=0;
  return 0;
}

int startDarc(ControlStruct *c){
  //if its running, kill it.
  pid_t pid;
  char chr[80];
  int i=0;
  //int startCirc=0;
  //If there is an existing darcmain instance, kill it.
  if(c->bufList[0]==NULL){
    snprintf(chr,80,"/%srtcParam1",c->options.prefix);
    if((c->bufList[0]=bufferOpen(chr))==NULL){
      printf("Failed to open /dev/shm/%srtcParam1\n",c->options.prefix);
    }
  }
  if(c->bufList[1]==NULL){
    snprintf(chr,80,"/%srtcParam2",c->options.prefix);
    if((c->bufList[1]=bufferOpen(chr))==NULL){
      printf("Failed to open /dev/shm/%srtcParam2\n",c->options.prefix);
    }
  }
  if(c->bufList[0]==NULL || c->bufList[1]==NULL || (bufferGetNEntries(c->bufList[0])==0 && bufferGetNEntries(c->bufList[1])==0)){
    printf("Not stopping RTC\n");
    //startCirc=0;
  }else{
    //startCirc=1;
    unsigned long n1=0,n2=0;
    if(c->bufList[0]!=0)
      n1=bufferGetNEntries(c->bufList[0]);
    if(c->bufList[1]!=0)
      n2=bufferGetNEntries(c->bufList[1]);
    
    printf("Stopping existing RTC (with %ld and %ld entries in param buffers at %p, %p)\n",n1,n2,c->bufList[0],c->bufList[1]);
    darcstop(c);
    sleep(1);
  }

  //Taken from subprocess.py in the standard python library.
  //pipe2(pipefd,O_CLOEXEC);
  c->coremain=0;
  pid=fork();
  if(pid==0){//child
    //close parent pipe ends.
    //close(errpipe_read);
    //close();
    //duplicate some fds.
    //dup();
    //close pipe fds.
    //close();
    printf("darcmain");
    i=1;
    while(c->options.darcargs[i]!=NULL){
      printf(" %s",c->options.darcargs[i]);
      i++;
    }
    printf("\n");
    execvp("darcmain",&c->options.darcargs[1]);
    exit(255);//not reported, so doesn't matter what it is.
  }else if(pid==-1){//error
    printf("Error forking\n");
  }else{
    c->coremain=pid;
    c->bufList[0]=NULL;
    c->bufList[1]=NULL;
    //now wait for the buffers...
    snprintf(chr,80,"/%srtcParam1",c->options.prefix);
    while(c->bufList[0]==NULL){
      printf("Opening %s\n",chr);
      c->bufList[0]=bufferOpen(chr);
      if(c->bufList[0]==NULL)
	sleep(1);
    }
    snprintf(chr,80,"/%srtcParam2",c->options.prefix);
    while(c->bufList[1]==NULL){
      printf("Opening %s\n",chr);
      c->bufList[1]=bufferOpen(chr);
      if(c->bufList[1]==NULL)
	sleep(1);
    }

  }
  return 0;
}
int recvSize(int sock,size_t size,char *buf){
  int err=0;
  size_t n=0;
  int rec;
  while(err==0 && n<size){
    rec=recv(sock,&buf[n],size-n,0);
    if(rec>0)
      n+=rec;
    else{
      printf("Error reading data in darccontrolc recvSize\n");
      err=-1;
    }
  }
  return err;
}

int darcinit(int sock,ControlStruct *c,char *data,size_t datasize){
  //read the data from sock, then init.
  //Or, if sock==0, then use the data already in data.
  int n,i,bufno;
  paramBuf *pbuf;
  bufferVal *ncamval=NULL,*val;
  bufferVal pval;
  char name[BUFNAMESIZE+1];
  struct timespec abstime;
  struct timeval t1;
  int rt=0;
  int err;
  int tmp;
  name[BUFNAMESIZE]='\0';
  if(sock>0){
    if(data!=NULL){
      printf("Warning in darcinit - data!=NULL - will be ignorred\n");
    }
    //receive the buffer from the socket.
    if((n=recv(sock,&datasize,sizeof(size_t),0))!=sizeof(size_t)){
      printf("Error receiving data length in darcinit\n");
      rt=1;
    }else if((data=calloc(datasize,1))==NULL){
      printf("Error allocing data in darcinit\n");
      rt=2;
    }else if(recvSize(sock,datasize,data)!=0){
      printf("Error receiving data in darcinit\n");
      free(data);
      rt=3;
    }
  }
  if(data!=NULL){
    bufno=bufferGetInactive(c->bufList);
    if(bufno==-1)
      bufno=0;
    //data now contains a mock darc parameter buffer.  So, use this, and copy all values into the real darc buffer.
    pbuf=bufferOpenFromData(data,datasize);
    //Note - put ncam in to the buffer last.
    n=bufferGetNEntries(pbuf);
    printf("Got %d entries\n",n);
    for(i=0;i<n;i++){
      //copy the parameters into the darc buffer.
      memcpy(name,&pbuf->buf[i*BUFNAMESIZE],BUFNAMESIZE);
      val=bufferGet(pbuf,name);
      if(strcmp(name,"ncam")==0){
	ncamval=val;//set this at the end.
      }else if(val!=NULL){
	if(strcmp(name,"switchRequested")==0){
	  tmp=0;
	  val->data=&tmp;
	}
	bufferSetIgnoringLock(c->bufList[bufno],name,val);
	free(val);
      }
    }
    if(ncamval!=NULL){
      bufferSetIgnoringLock(c->bufList[bufno],"ncam",ncamval);
      free(ncamval);
    }
    
    free(pbuf);
    pval.dtype='i';
    pval.size=sizeof(int);
    i=1;
    pval.data=&i;
    printf("darccontrolc set entries, now requesting switch\n");
    bufferSetIgnoringLock(c->bufList[1-bufno],"switchRequested",&pval);
    pbuf=c->bufList[1-bufno];
    i=0;
    //now wait for the switch to complete.
    pthread_mutex_lock(pbuf->condmutex);
    while((BUFFLAG(pbuf)&0x1)==1 && i<50){//slight race condition here - may result in delaying darccontrolc by up to a second (but no other effects).
      gettimeofday(&t1,NULL);
      abstime.tv_sec=t1.tv_sec+1;//add 1 second.
      abstime.tv_nsec=t1.tv_usec*1000;
      err=pthread_cond_timedwait(pbuf->cond,pbuf->condmutex,&abstime);
      if(err==ETIMEDOUT){
	printf("Timeout waiting for buffer switch to complete\n");
      }else if(err!=0){
	printf("pthread_cond_timedwait failed in darccontrolc\n");
      }
      i++;
      if(i==50){
	printf("Error waiting for RTC to complete buffer switch - possibly failed to start\n");
	rt=4;
      }
    }
    pthread_mutex_unlock(pbuf->condmutex);
    //now copy the buffer.
    if((BUFFLAG(pbuf)&0x1)==0 && i<50){
      memcpy(pbuf->buf,c->bufList[bufno]->buf,pbuf->arrsize-BUFARRHDRSIZE(pbuf));
    }
  }else{//no data received.
    rt=5;
  }
  return rt;
}
int darcinitfile(int sock,ControlStruct *c,char *name){
  //read the filename from sock, then read the file, then init.
  //If sock==0, then name is taken from name.  
  int namelen;
  int fd;
  char line[80];
  int i,n;
  int rt=0;
  off_t start,end;
  char *data;
  if(sock!=0){
    if((n=recv(sock,&namelen,sizeof(int),0))!=sizeof(int)){
      printf("Error receiving filename length\n");
      rt=1;
    }else if((name=calloc(namelen+1,1))==NULL){
      printf("Error allocing name in darcinitfile\n");
      rt=2;
    }else if(recvSize(sock,namelen,name)!=0){
      printf("Error receiving name in darcinitfile\n");
      free(name);
      name=NULL;
      rt=3;
    }else{
      name[namelen]='\0';
    }
  }else if(name!=NULL){
    namelen=strlen(name);
  }
  if(name!=NULL){
    if((fd=open(name,O_RDONLY))==-1){//open the fits file
      printf("Error opening file %s: %s\n",name,strerror(errno));
      if(sock!=0)
	free(name);
      rt=4;
    }else{//read the file.
      end=0;
      i=0;
      while(end==0){
	if((n=read(fd,line,80))!=80){
	  printf("Error reading to END in file %s\n",name);
	  end=1;
	  i=-2;
	}else{
	  if(strncmp(line,"END",3)==0 && (line[3]=='\0' || line[3]==' ')){
	    //end of header found.
	    end=1;
	  }
	}
	i++;
      }
      if(end==1 && i>0){
	if(i%36!=0)
	  i+=36-i%36;
	end=lseek(fd,0,SEEK_END);
	//skip to end of FITS header.
	start=lseek(fd,i*80,SEEK_SET);
	printf("File required %ld bytes\n",end-start);
	if((data=calloc(end-start,1))==NULL){
	  printf("Error allocing data of %ld bytes to read fits file\n",(long)(end-start));
	  rt=6;
	}else if((n=read(fd,data,end-start))!=end-start){
	  printf("Error reading file data - only %d bytes read (expecting %ld)\n",n,(long)(end-start));
	  free(data);
	  rt=7;
	}else{
	  //then darcinit with the data.
	  rt=darcinit(0,c,data,end-start);
	  free(data);
	}
      }else{
	rt=5;
      }
      if(sock!=0)
	free(name);
    }
  }
  return rt;
}

int initialiseRTC(ControlStruct *c){
  char path1[80],path2[80];
  int rtcStarted=0;
  int rt=0;
  int startCirc=0;
  int bufno;
  int frozen;
  paramBuf *b;
  int nlabels;
  snprintf(path1,80,"/%srtcParam1",c->options.prefix);
  snprintf(path2,80,"/%srtcParam2",c->options.prefix);
  
  //if(c->options.configfile!=NULL){
    //attempt to open existing parameter buffer.
  if((c->bufList[0]=bufferOpen(path1))==NULL || (c->bufList[1]=bufferOpen(path2))==NULL){
    rtcStarted=0;
  }else{
    rtcStarted=1;
  }
  if(rtcStarted==0){//need to start darc...
    printf("Starting darcmain\n");
    startDarc(c);
    rtcStarted=1;
  }
  //}
  if(rtcStarted){
    if(bufferGetMem(c->bufList[0],0)==0 && bufferGetMem(c->bufList[1],0)==0){
      startCirc=1;//buffer currently empty so start circular buffers.
    }
    bufno=bufferGetInactive(c->bufList);
    b=c->bufList[bufno];
    frozen=BUFFLAG(b)&0x1;
    nlabels=bufferGetNEntries(b);
    if(nlabels==0 && frozen==0){//time to initialise the buffer - rtc probably waiting.
      if(c->options.configfile==NULL){//wait for a config to come from client

      }else{
	printf("Loading rtc config %s\n",c->options.configfile);
	if((rt=darcinitfile(0,c,c->options.configfile))!=0){
	  printf("Failed to initialise buffer (err %d) - stopping darc\n",rt);
	  rt=1;
	  darcstop(c);//stop darc, but not control.
	}
      }
    }
    if(startCirc || c->corestarted){
      //open circular buffers?
    }
  }
  return rt;
}


int openSocket(ControlStruct *c){
  //opens a listening socket
  //Create the socket and set up to accept connections.
  struct sockaddr_in name;
  int err=0,optval;
  if((c->lsocket=socket(AF_INET,SOCK_STREAM,0))<0){
    printf("Error opening listening socket\n");
    err=1;
  }else{
    optval=1;
    if(setsockopt(c->lsocket, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(int))!=0){
      printf("setsockopt failed - ignoring\n");
    }
    name.sin_family=AF_INET;
    name.sin_port=htons(c->options.port);
    name.sin_addr.s_addr=htonl(INADDR_ANY);
    if(bind(c->lsocket,(struct sockaddr*)&name,sizeof(name))<0){
      printf("Unable to bind\n");
      err=1;
      close(c->lsocket);
      c->lsocket=0;
    }
  }
  if(err==0){
    if(listen(c->lsocket,4)<0){
      printf("Failed to listen on port\n");
      err=1;
    }
  }
  //if(err==0)
  //c->socketOpen=1;
  return err;
}
int sendall(int sock,char *values,int nbytes,int flags){
  int err=0;
  int n=0;
  int nsent;
  while(err==0 && n<nbytes){
    nsent=send(sock,&values[n],nbytes-n,flags);
    if(nsent<=0)
      err=1;
    else
      n+=nsent;
  }
  return err;
}
int darcset(int sock,ControlStruct *c){
  //get the name, dtype and size from client, followed by the data.  Then set in darc.
  int err=0;
  int n=0;
  int namelen;
  char name[BUFNAMESIZE+1];
  int bufno;
  bufferVal value;
  int doswitch=0;
  int dtype=0;
  size_t nbytes=0;
  char *data=NULL;
  if((n=recv(sock,&namelen,sizeof(int),0))!=sizeof(int)){
    err=1;
    printf("Length of name not received (got %d bytes)\n",n);
  }else if(namelen>BUFNAMESIZE){
    printf("Error: name is too long (>%d)\n",BUFNAMESIZE);
    err=2;
  }else if(recvSize(sock,(size_t)namelen,name)!=0){
    err=3;
    printf("Error receiving name\n");
  }else if((recv(sock,&nbytes,sizeof(size_t),0))!=sizeof(size_t)){
    printf("Error getting nbytes\n");
    err=4;
  }else if(recv(sock,&dtype,sizeof(int),0)!=sizeof(int)){
    printf("Error getting dtype\n");
    err=5;
  }else if(recv(sock,&doswitch,sizeof(int),0)!=sizeof(int)){
    printf("Error getting doswitch\n");
    err=6;
  }else if(nbytes!=0 && (data=calloc(nbytes,1))==NULL){
    printf("Error allocing data (nbytes=%ld)\n",nbytes);
    err=7;
  }else if(nbytes!=0 && recvSize(sock,nbytes,data)!=0){
    printf("Error getting data\n");
    err=8;
  }else{
    name[namelen]='\0';
    //now get the buffer.
    value.data=data;
    value.dtype=(char)dtype;
    value.size=nbytes;
    if((bufno=bufferGetInactive(c->bufList))==-1){
      printf("Unable to get inactive buffer\n");
      err=9;
    }else if(bufferSet(c->bufList[bufno],name,&value)!=0){
      printf("Error setting value %s\n",name);
      err=10;
    }else if(doswitch){
      printf("setSwitchRequested\n");
      setSwitchRequested(c,1,1);
      printf("Done\n");
    }
  }
  if(data!=NULL)
    free(data);
  printf("darcset returning with error %d\n",err);
  return err;
}

int darcget(int sock,ControlStruct *c){
  //Get the parameter name from socket, find out its value in darc, then send this to client.
  int err=0;
  int n=0;
  int namelen;
  char param[BUFNAMESIZE+1];
  int bufno;
  paramBuf *b;
  char *bufferNames;
  int indx=-1;
  void *values;
  char dtype;
  int nbytes;
  char data[BUFNAMESIZE+sizeof(int)*9];
  memset(param,0,BUFNAMESIZE+1);
  if((n=recv(sock,&namelen,sizeof(int),0))!=sizeof(int)){
    err=1;
    printf("Length of name not received (got %d bytes)\n",n);
  }else{
    if(namelen>BUFNAMESIZE){
      printf("Error: name is too long (>%d)\n",BUFNAMESIZE);
      err=2;
    }else if(recvSize(sock,(size_t)namelen,param)!=0){
      err=3;
      printf("Error receiving name\n");
    }else{
      param[namelen]='\0';
      printf("get: %s\n",param);
      //now read parameter "param" from the darc buffer.
      if((bufno=bufferGetActive(c->bufList))==-1){
	printf("Unable to get active buffer\n");
	err=4;
      }else{
	b=c->bufList[bufno];
	if((bufferNames=bufferMakeNames(1,param))==NULL){
	  printf("Error bufferMakeNames\n");
	  err=5;
	}else{
	  bufferGetIndex(b,1,bufferNames,&indx,&values,&dtype,&nbytes);
	}
      }
      if(indx==-1){//not found
	printf("Parameter %s not found\n",param);
	if(err==0)
	  err=6;
      }else if(err==0){
	//now send back to the client.
	//We send:
	//name (16 bytes)
	//nbytes (4 bytes)
	//dtype (4 bytes)
	//ndim (4 bytes)
	//dims (24 bytes)
	//Then the data (nbytes bytes).
	memcpy(data,param,BUFNAMESIZE);
	((int*)&data[BUFNAMESIZE])[0]=nbytes;
	data[BUFNAMESIZE+sizeof(int)]=(int)dtype;
	((int*)&data[BUFNAMESIZE])[2]=BUFNDIM(b,indx);
	memcpy(&data[BUFNAMESIZE+sizeof(int)*3],BUFDIM(b,indx),sizeof(int)*6);
	printf("Sending Get reply of %ld bytes\n",BUFNAMESIZE+sizeof(int)*9);
	if(sendall(sock,data,BUFNAMESIZE+sizeof(int)*9,0)!=0){
	  err=7;
	  printf("Error sendall header\n");
	}else if(sendall(sock,values,nbytes,0)!=0){
	  err=8;
	  printf("Error sendall data\n");
	}

      }
    }
  }
  return err;
}
int darcdecimate(int sock,ControlStruct *c){
  //Read from the socket: 
  //streamnamelen
  //streamname
  //decimate value
  //Then: set the decimation.
  int err=0;
  int namelen;
  char name[256];
  int dec,n;
  circBuf *cb=NULL;
  name[0]='/';
  if((n=recv(sock,&namelen,sizeof(int),0))!=sizeof(int)){
    err=1;
    printf("Length of name not received (got %d bytes)\n",n);
  }else if(namelen>254){
    printf("Error - stream name length larger than expected (%d bytes)\n",namelen);
    err=1;
  }else if((n=recv(sock,&name[1],namelen,0))!=namelen){
    printf("Stream name of length %d bytes not received (got %d bytes)\n",namelen,n);
    err=1;
  }else if((n=recv(sock,&dec,sizeof(int),0))!=sizeof(int)){
    printf("Error - decimation value not received (got %d bytes)\n",n);
    err=1;
  }else{
    //set the decimation for this stream.
    name[namelen+1]='\0';
    if((cb=circOpenBufReader(name))!=NULL){
      FREQ(cb)=dec;
    }else{
      printf("Error opening %s\n",name);
      err=2;
    }
  }
  if(cb!=NULL){
    circCloseBufReader(cb);
  }
  return err;
}
int darcsender(int sock,ControlStruct *c){
  //Read from the socket:
  //streamnamelen
  //streamname
  //IP addr (4 bytes)
  //port
  int err=0;
  int namelen;
  char name[256];
  //int data[2];
  int *data;
  char *args[9];
  unsigned char *cdata;//=(char*)data;
  int n,i;
  pid_t pid;
  args[0]="sender";
  if(posix_memalign((void**)&data,8,8)!=0){
    printf("Error allocing data in darcsender()\n");
    return 1;
  }
  cdata=(unsigned char*)data;
  if((n=recv(sock,&namelen,sizeof(int),0))!=sizeof(int)){
    err=1;
    printf("Length of name not received (got %d bytes)\n",n);
  }else if(namelen>255){
    printf("Error - stream name length larger than expected (%d bytes)\n",namelen);
    err=1;
  }else if((n=recv(sock,name,namelen,0))!=namelen){
    printf("Stream name of length %d bytes not received (got %d bytes)\n",namelen,n);
    err=1;
  }else if((n=recv(sock,data,sizeof(int)*2,0))!=sizeof(int)*2){
    printf("Error - host/port not received (got %d bytes)\n",n);
    err=1;
  }else{
    name[namelen]='\0';
    if(asprintf(&args[1],"-p%d",data[1])==-1){
      printf("Error in asprintf in starting sender\n");
      args[1]=NULL;
      err=2;
    }else if(asprintf(&args[2],"-h%d.%d.%d.%d",cdata[0],cdata[1],cdata[2],cdata[3])==-1){
      printf("Error in asprintf in starting sender\n");
      args[2]=NULL;
      err=2;
    }else if(asprintf(&args[3],"-t1")==-1){
      printf("Error in asprintf in starting sender\n");
      args[3]=NULL;
      err=2;
    }else if(asprintf(&args[4],"-i1")==-1){
      printf("Error in asprintf in starting sender\n");
      args[4]=NULL;
      err=2;
    }else if(asprintf(&args[5],"-r")==-1){
      printf("Error in asprintf in starting sender\n");
      args[5]=NULL;
      err=2;
    }else if(asprintf(&args[6],"-n")==-1){
      printf("Error in asprintf in starting sender\n");
      args[6]=NULL;
      err=2;
    }else{
      args[7]=name;
      args[8]=NULL;//end
    }
    if(err==0){
      //printf("sender");
      for(i=0;i<8;i++)
	printf(" %s",args[i]);
      printf("\n");
      pid=fork();
      if(pid==0){//child
	//close parent pipe ends.
	//close(errpipe_read);
	//close();
	//duplicate some fds.
	//dup();
	//close pipe fds.
	//close();
	printf("sender\n");
	execvp("sender",args);
	exit(255);//not reported, so doesn't matter what it is.
      }
    }else{
      for(i=1;i<7;i++)
	free(args[i]);
    }
  }
  free(data);
  return err;
}
int darccontrolstop(ControlStruct *c){
  printf("Exiting...\n");
  c->go=0;
  exit(0);
  return 0;
}
int darcreadcircbuf(int sock,ControlStruct *c){
  //returns the latest contents of circular buffer to the socket.
  //Read from the socket: 
  //streamnamelen
  //streamname
  //Flags: 1 for block.
  //       2 for forcewrite.
  //Then: read the frame.

  int err=0;
  int namelen;
  char name[256];
  int flags;
  circBuf *cb=NULL;
  int n,size;
  char *data;
  name[0]='/';
  if((n=recv(sock,&namelen,sizeof(int),0))!=sizeof(int)){
    err=1;
    printf("Length of name not received (got %d bytes)\n",n);
  }else if(namelen>254){
    printf("Error - stream name length larger than expected (%d bytes)\n",namelen);
    err=1;
  }else if((n=recv(sock,&name[1],namelen,0))!=namelen){
    printf("Stream name of length %d bytes not received (got %d bytes)\n",namelen,n);
    err=1;
  }else if((n=recv(sock,&flags,sizeof(int),0))!=sizeof(int)){
    printf("Error - flags not received (got %d bytes)\n",n);
    err=1;
  }else{
    //set the decimation for this stream.
    name[namelen+1]='\0';
    if((cb=circOpenBufReader(name))!=NULL){
      circHeaderUpdated(cb);
      cb->lastReceived=LASTWRITTEN(cb);//so that we're not always getting entry 0 in the circ buffer.
      if(cb->lastReceived>=0)
	cb->lastReceivedFrame=CIRCFRAMENO(cb,cb->lastReceived);
      printf("Set lastReceived to %d\n",cb->lastReceived);
      if(flags&0x2)//set forcewrite
	FORCEWRITE(cb)=1;
      if(flags&0x1){//block until frame...
	printf("calling circGetNExtFrame for %s\n",name);
	data=circGetNextFrame(cb,1.,10);
      }else{//get the newest frame that has already been written.
	printf("Calling circGetLatestFrame for %s\n",name);
	data=circGetLatestFrame(cb);
      }
      if(data==NULL){
	size=0;
	err=sendall(sock,(char*)&size,sizeof(int),0);
	if(err!=0){
	  printf("Error sending frame\n");
	  err=3;
	}else{
	  printf("No stream data available\n");
	}
      }else{
	size=((int*)data)[0]+4;
	err=sendall(sock,data,size,0);
	if(err!=0){
	  printf("Error sending frame\n");
	  err=3;
	}
      }
    }else{
      printf("Error opening %s\n",name);
      err=2;
    }
  }
  if(cb!=NULL){
    circCloseBufReader(cb);
  }
  return err;
}

void *clientThread(void *T){
  //Has a client attached, sits in a loop waiting for data and acting on it.
  ThreadStruct *t=(ThreadStruct*)T;
  ControlStruct *c=(ControlStruct*)t->c;
  int n=1;
  int hdr;
  int ndiscard=0;
  int err=0;
  int ret[2];
  int nsent;
  int cmd;
  ret[0]=0x55555555;
  while(c->go && n>0){
    n=recv(t->sock,&hdr,sizeof(int),0);
    if(n==sizeof(int) && hdr==0x55555555){//got the header okay
      if(ndiscard!=0){
	printf("Discarding %d bytes\n",ndiscard);
	ndiscard=0;
      }
      //now get the command.
      n=recv(t->sock,&cmd,sizeof(int),0);
      if(n==sizeof(int)){
	printf("Got command %d\n",cmd);
	switch(cmd){
	case DARCSET:
	  err=darcset(t->sock,c);
	  break;
	case DARCGET:
	  err=darcget(t->sock,c);
	  break;
	case DARCDECIMATE:
	  err=darcdecimate(t->sock,c);
	  break;
	case DARCSENDER:
	  err=darcsender(t->sock,c);
	  break;
	case DARCSTREAM:
	  err=darcreadcircbuf(t->sock,c);
	  break;
	case DARCSTOP:
	  err=darcstop(c);
	  break;
	case DARCCONTROLSTOP:
	  err=darccontrolstop(c);
	  break;
	case DARCINIT:
	  startDarc(c);
	  err=darcinit(t->sock,c,NULL,0);
	  break;
	case DARCINITFILE:
	  startDarc(c);
	  err=darcinitfile(t->sock,c,NULL);
	  break;
	default:
	  printf("Unrecognised command %d\n",cmd);
	  break;
	}
	ret[1]=err|(cmd<<16);
	if((nsent=send(t->sock,ret,sizeof(int)*2,0))!=sizeof(int)*2){
	  printf("Error sending response to client: %d bytes send\n",nsent);
	}else{
	  printf("Response sent\n");
	}
      }else{
	printf("Truncated command (%d bytes)\n",n);
      }
    }else{
      ndiscard+=n;
    }
  }
  printf("Closing socket\n");
  close(t->sock);
  free(T);
  return 0;
}

int acceptSocket(ControlStruct *c){
  struct sockaddr_in clientname;
  socklen_t size;
  int err=0;
  char buf[80];
  //char namesize;
  int sock;
  ThreadStruct *thrstr;
  pthread_t threadid;
  memset(buf,0,80);
  //c->hasclient=0;
  size=(socklen_t)sizeof(struct sockaddr_in);
  printf("Waiting for client to connect on port %d...\n",c->options.port);
  if((sock=accept(c->lsocket,(struct sockaddr*)&clientname,&size))<0){
    printf("Failed to accept on socket: %s\n",strerror(errno));
    err=1;
  }else{
    printf("Connected from %s port %d\n",inet_ntoa(clientname.sin_addr),(int)ntohs(clientname.sin_port));
    //c->hasclient=1;
    thrstr=calloc(sizeof(ThreadStruct),1);
    thrstr->sock=sock;
    thrstr->c=c;
    pthread_create(&threadid,NULL,clientThread,thrstr);
    thrstr->threadid=threadid;
  }
  return err;
}


int serverLoop(ControlStruct *c){
  //Waits for clients to connect, and when they do, starts a thread to serve them.
  int err;
  if((err=openSocket(c))!=0){
    printf("Couldn't open listening socket port %d\n",c->options.port);
  }
  while(err==0 && c->go){
    err=acceptSocket(c);
  }
  printf("darccontrolc exiting\n");
  return err;
}
int main(int argc,char **argv){
  struct argp argp = { options, parse_opt, args_doc, doc };
  struct arguments arguments;
  ControlStruct c;
  pthread_t logid;
  memset(&arguments,0,sizeof(struct arguments));
  setdarcarg(&arguments,"darcmain");
  setdarcarg(&arguments,"-i");
  argp_parse (&argp, argc, argv, 0, 0, &arguments);
  if(arguments.prefix==NULL)
    arguments.prefix=strdup("\0");
  if(arguments.output==0){//redirect stdout to a file...
    setdarcarg(&arguments,"-r");
    if(pthread_create(&logid,NULL,rotateLog,arguments.prefix)){
      printf("pthread_create rotateLog failed\n");
      return -1;
    }
    sleep(1);
  }
  if(arguments.port==0)
    arguments.port=8900;
  printf ("prefix = %s\nport = %d\naffin = %lu\n"\
          "prio = %d\noutput = %d\n"\
	  "bufsize = %ld\nnhdr = %d\nconfigfile = %s\n"\
	  "nstore = %s\ncircBufMemSize = %ld\n",
	  arguments.prefix,
	  arguments.port,
	  arguments.affin,
	  arguments.prio,
	  arguments.output,
	  arguments.bufsize,
	  arguments.nhdr,
	  arguments.configfile,
	  arguments.nstore,
	  arguments.circBufMemSize);
  c.options=arguments;
  c.go=1;
  initialiseRTC(&c);//start darcmain, or connect to existing instance.
  //watchStreams();//start watching /dev/shm
  //startBroadcastThread(&c);
  serverLoop(&c);//wait for clients to connect and send commands.
  

  return 0;
}


