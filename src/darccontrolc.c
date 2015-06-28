
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
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <argp.h>
#include <time.h>
#include <pthread.h>

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
  int darcargssize;//keep size of the array.
};

typedef struct{
  struct arguments options;
  paramBuf bufList[2];
  int corestarted;
  int lsocket;
  uint16_t port;
  int go;
}ControlStruct;

typedef struct{
  ControlStruct *c;
  pthread_t threadid;
  int sock;
}ThreadStruct;


int setdarcarg(struct arguments *args,char *fmt,...){
  va_list ap;
  int rt=0;
  char **tmp;
  if(args->curdarcarg==args->darcargsize){//need to increase size.
    if((tmp=calloc(sizeof(char*),args->darcargsize+8+1))==NULL){
      //+1 is for terminating NULL
      printf("Error callocing in setdarcarg\n");
      rt=1;
    }else{
      memcpy(tmp,args->darcargs,sizeof(char*)*args->darcargsize);
      args->darcargsize+=8;
    }
  }
  va_start(ap,fmt);
  if((rt==0 && l=vasprintf(&args->darcargs[args->curdarcarg],fmt,ap))<=0){
    printf("vasprintf error...\n");//should probably do something about this!
    rt=1;
  }
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
      setdarcarg(args,"-r");
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

int startDarc(controlStruct *c){
  //if its running, kill it.
  char *cmd;
  if(c->coremainid!=NULL){
    pthread_terminate(c->coremainid);
    sleep(1);
    if(poll(c->coremainid)!=NULL)
      pthread_kill(c->coremainid);
  }
  //Taken from subprocess.py in the standard python library.
  //pipe2(pipefd,O_CLOEXEC);
  pid=fork();
  if(pid==0){//child
    //close parent pipe ends.
    //close(errpipe_read);
    //close();
    //duplicate some fds.
    //dup();
    //close pipe fds.
    //close();
    execvp("darcmain",c->options.darcargs);
    exit(255);//not reported, so doesn't matter what it is.
  }else if(pid==-1){//error
    printf("Error forking\n");
  }
  
}

int initialiseRTC(ControlStruct *c){
  char *path1,*path2;
  int rtcStarted=0;
  int rt=0;
  paramBuf *pbuf[2];
  asprintf(&path1,"/%srtcParam1",c->options.prefix);
  asprintf(&path2,"/%srtcParam2",c->options.prefix);
  
  if(c->options.configfile!=NULL){
    //attempt to open existing parameter buffer.
    if((pbuf[0]=bufferOpen(path1,&c->bufList[0]))==NULL || (pbuf[1]=bufferOpen(path2,&c->bufList[1]))==-1){
      rtcStarted=0;
    }else{
      rtcStarted=1;
    }
    if(rtcStarted==0){//need to start darc...
      startDarc(c);
      rtcStarted=1;
    }
  }
  free(path1);
  free(path2);
  if(rtcStarted){
    if(bufferGetMem(&c->bufList[0])==0 && bufferGetMem(&c->bufList[1])==0){
      startCirc=1;//buffer currently empty so start circular buffers.
    }
    bufno=bufferGetInactive(c->bufList);
    b=c->bufList[bufno];
    frozen=b->flags[0]&0x1;
    nlabels=bufferGetNLabels(b);
    if(nlabels==0 && frozen==0){//time to initialise the buffer - rtc probably waiting.
      if(c->options.configfile==NULL){//wait for a config to come from client

      }else{
	printf("Loading rtc config %s\n",c->options.configfile);
	if(bufferInit(c,bufno,c->options.configfile)!=0){
	  rt=1;
	  printf("Failed to initialise buffer - stopping darc\n");
	  darcstop(1,0);//stop darc, but not control.
	}
      }
    }
    if(startCirc || c->corestarted){
      //open circular buffers?
    }
  }
  return rt;
}

int recvSize(int sock,int size,char *buf){
  int err=0,n=0;
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

int openSocket(ControlStruct *c){
  //opens a listening socket
  //Create the socket and set up to accept connections.
  struct sockaddr_in name;
  int err=0;
  if((c->lsocket=socket(PF_INET,SOCK_STREAM,0))<0){
    printf("Error opening listening socket\n");
    err=1;
  }else{
    name.sin_family=AF_INET;
    name.sin_port=htons(c->port);
    name.sin_addr.s_addr=htonl(INADDR_ANY);
    if(bind(c->lsocket,(struct sockaddr*)&name,sizeof(name))<0){
      printf("Unable to bind\n");
      err=1;
      close(c->lsocket);
      c->lsocket=0;
    }
  }
  if(err==0){
    if(listen(c->lsocket,1)<0){
      printf("Failed to listen on port\n");
      err=1;
    }
  }
  //if(err==0)
  //c->socketOpen=1;
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

  if((n=recv(sock,&namelen,sizeof(int),0))!=sizeof(int)){
    err=1;
    printf("Length of name not received (got %d bytes)\n",n);
  }else{
    if(namelen>BUFNAMESIZE){
      printf("Error: name is too long (>%d)\n",BUFNAMESIZE);
      err=2;
    }else if(recvSize(sock,namelen,param)!=0){
      err=3;
      printf("Error receiving name\n");
    }else{
      param[namelen]='\0';
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
	if(err==0)
	  err=6;
      }else{
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
	data[BUFNAMESIZE+sizeof(int)]=dtype;
	((int*)&data[BUFNAMESIZE])[2]=BUFNDIM(b,indx);
	memcpy(&data[BUFNAMESIZE+sizeof(int)*3],BUFDIM(b,indx),sizeof(int)*6);
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
	case DARCSTOP:
	  err=darcstop(t->sock,c,1,0);
	  break;
	case DARCCONTROLSTOP:
	  err=darcstop(t->sock,c,0,1);
	  break;
	case DARCINIT:
	  err=darcinit(t->sock,c);
	  break;
	case DARCINITFILE:
	  err=darcinitfile(t->sock,c);
	  break;
	default:
	  printf("Unrecognised command %d\n",cmd);
	  break;
	}
	ret[1]=err|(cmd<<16);
	if((nsent=send(sock,ret,sizeof(int)*2,0))!=sizeof(int)*2){
	  printf("Error sending response to client: %d bytes send\n",nsent);
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

}

int acceptSocket(ControlStruct *c){
  struct sockaddr_in clientname;
  socklen_t size;
  int err=0;
  char buf[80];
  char namesize;
  int sock;
  ThreadStruct *thrstr;
  pthread_t threadid;
  memset(buf,0,80);
  //c->hasclient=0;
  size=(socklen_t)sizeof(struct sockaddr_in);
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
  if((err=openSocket(c))!=0){
    printf("Couldn't open listening socket port %d\n",c->options.port);
  }
  while(err==0 && c->go){
    err=acceptSocket(c);
  }
  print "darccontrolc exiting\n");
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
    if(pthread_create(&logid,NULL,rotateLog,arguments.prefix)){
      printf("pthread_create rotateLog failed\n");
      return -1;
    }
    sleep(1);
  }
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


