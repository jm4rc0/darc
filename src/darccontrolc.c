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
};

/* Parse a single option. */
static error_t
parse_opt (int key, char *arg, struct argp_state *state)
{
  /* Get the input argument from argp_parse, which we
     know is a pointer to our arguments structure. */
  struct arguments *arguments = state->input;
  int val;
  char *tmp;
  switch (key)
    {
    case 's':
      arguments->prefix=arg;
      break;
    case 'p':
      arguments->port=atoi(arg);
      break;
    case 'a':
      arguments->affin=strtoul(arg,NULL,10);
      break;
    case 'i':
      arguments->prio=atoi(arg);
      break;
    case 'o':
      arguments->output=1;
      break;
    case 'b':
      arguments->bufsize=atol(arg);
      break;
    case 'e':
      arguments->nhdr=atoi(arg);
      break;
    case'f':
      arguments->configfile=arg;
      break;
    case 'c'://arg is e.g. rtcPxlBuf=10
      tmp=strchr(arg,'=');
      if(tmp==NULL){
	printf("Error: nstore for %s not found\nPlease use format -c %s=10\n",arg,arg);
	exit(1);
      }
      tmp[0]='\0';
      val=atoi(&tmp[1]);
      if(arguments->nstore==NULL){
	if(asprintf(&tmp,"-c %s %d",arg,val)==-1)
	  printf("asprintf error\n");
      }else{
	if(asprintf(&tmp,"%s -c %s %d",arguments->nstore,arg,val)==-1)
	  printf("asprintf error\n");
      }
      free(arguments->nstore);
      arguments->nstore=tmp;
      break;
    case 'm':
      arguments->circBufMemSize=atol(arg);
      break;
    case ARGP_KEY_ARG:
      //if (state->arg_num >= 2)   //Too many arguments.
      //  argp_usage (state);
      //arguments->args[state->arg_num] = arg;
      if(arguments->configfile==NULL)
	arguments->configfile=arg;
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
int initialiseRTC(ControlStruct *c){
  char *path1,*path2;
  int rtcStarted=0;
  int rt=0;
  asprintf(&path1,"/%srtcParam1",c->options.prefix);
  asprintf(&path2,"/%srtcParam2",c->options.prefix);
  
  if(c->options.configfile!=NULL){
    //attempt to open existing parameter buffer.
    if((c->bufList[0]=bufferOpen(path1))==-1 || (c->bufList[1]=bufferOpen(path2))==-1){
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
    if(bufferGetMem(c->bufList[0])==0 && bufferGetMem(c->bufList[1])==0){
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

int acceptSocket(ControlStruct *c){
  struct sockaddr_in clientname;
  socklen_t size;
  int err=0;
  char buf[80];
  char namesize;
  memset(buf,0,80);
  c->hasclient=0;
  size=(socklen_t)sizeof(struct sockaddr_in);
  if((c->client=accept(c->lsocket,(struct sockaddr*)&clientname,&size))<0){
    printf("Failed to accept on socket: %s\n",strerror(errno));
    err=1;
  }else{
    printf("Connected from %s port %d\n",inet_ntoa(clientname.sin_addr),(int)ntohs(clientname.sin_port));
    c->hasclient=1;
    pthread_create(&threadid,NULL,clientThread,threadStr);

    if((err=recvSize(c->client,sizeof(char),&namesize))!=0){
      printf("Failed to get length of name - closing\n");
    }else{
      if(namesize>79){
	printf("name too long - closing\n");
	err=1;
      }else{
	if((err=recvSize(c->client,namesize,buf))!=0){
	  printf("didn't receive name - closing\n");
	}else{
	  if(strncmp(&c->fullname[1],buf,80)!=0){
	    printf("stream %s expected, got %s - closing\n",&c->fullname[1],buf);
	    err=1;
	  }
	}
      }
    }
    if(err){
      close(c->client);
      c->hasclient=0;
    }
  }
  return err;
}


int serverLoop(ControlStruct *c){
  //Waits for clients to connect, and when they do, starts a thread to serve them.
  if((err=openSocket(c))!=0){
    printf("Couldn't open listening socket port %d\n",c->options.port);
  }
  while(err==0 && c->go){
    acceptSocket(c);

}
int main(int argc,char **argv){
  struct argp argp = { options, parse_opt, args_doc, doc };
  struct arguments arguments;
  pthread_t logid;
  memset(&arguments,0,sizeof(struct arguments));
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
  initialiseRTC(&c);//start darcmain, or connect to existing instance.
  //watchStreams();//start watching /dev/shm
  //startBroadcastThread(&c);
  serverLoop(&c);//wait for clients to connect and send commands.
  

  return 0;
}


