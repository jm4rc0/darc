/**
   The code here is used to create a shared object library, which can then be swapped around depending on which mirrors/interfaces you have in use, ie you simple rename the mirror file you want to mirror.so (or better, change the soft link), and restart the coremain.

The library is written for a specific mirror configuration - ie in multiple mirror situations, the library is written to handle multiple mirrors, not a single mirror many times.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/select.h>
#include "rtcmirror.h"
/**
   Find out if this SO library supports your camera.

*/

typedef struct{
  int sd;
  int port;
  char *host;
}mirrorStruct;

int mirrorQuery(char *name){
  if(strcmp(name,"dmcSocket")==0)
    return 0;
  return 1;
}
/**
   Open a camera of type name.  Args are passed in a float array of size n, which can be cast if necessary.  Any state data is returned in camHandle, which should be NULL if an error arises.
   pxlbuf is the array that should hold the data. The library is free to use the user provided version, or use its own version as necessary (ie a pointer to physical memory or whatever).  It is of size npxls*sizeof(short).
   ncam is number of cameras, which is the length of arrays pxlx and pxly, which contain the dimensions for each camera.
   Name is used if a library can support more than one camera.

*/

int mirrorOpen(char *name,int narg,int *args, int nacts,void **mirrorHandle){
  int err=0;
  int sd;
  struct sockaddr_in sin;
  struct hostent *host;
  int portno;
  mirrorStruct *mirror;
  printf("Initialising mirror %s\n",name);
  if(narg<2){
    printf("Error - need arguments port(int32),hostname(null terminated string)\n");
    err=1;
    return err;
  }
  if((err=mirrorQuery(name))){
    printf("Error - wrong mirror name\n");
    *mirrorHandle=NULL;
  }else{
    if((*mirrorHandle=malloc(sizeof(mirrorStruct)))==NULL){
      printf("mirrorHandle malloc error\n");
      err=1;
    }
    mirror=(mirrorStruct*)*mirrorHandle;
    if(mirror!=NULL){
      memset(mirror,0,sizeof(mirrorStruct));
      //host=gethostbyname("cfai26");
      portno=args[0];
      mirror->port=portno;
      mirror->host=strndup((char*)&args[1],(narg-1)*sizeof(int));
      printf("Got host %s, port %d\n",mirror->host,portno);
      if((host=gethostbyname(mirror->host))==NULL){
      //if((host=gethostbyname("129.234.187.102"))==NULL){
	printf("gethostbyname error\n");
	free(mirror);
	*mirrorHandle=NULL;
	err=1;
      }else{
	sd = socket(AF_INET, SOCK_STREAM, 0);
	mirror->sd=sd;
	memcpy(&sin.sin_addr.s_addr, host->h_addr, host->h_length);
	sin.sin_family = AF_INET;
	sin.sin_port = htons(portno);
	if(connect(sd,(struct sockaddr *)&sin, sizeof(sin))<0){
	  printf("Error connecting\n");
	  close(sd);
	  free(*mirrorHandle);
	  *mirrorHandle=NULL;
	  err=1;
	}else{
	  printf("Connected\n");
	}
      }
    }
  }
  return err;
}

/**
   Close a camera of type name.  Args are passed in the float array of size n, and state data is in camHandle, which should be freed and set to NULL before returning.
*/
int mirrorClose(void **mirrorHandle){
  mirrorStruct *mirror=(mirrorStruct*)*mirrorHandle;
  printf("Closing mirror\n");
  if(*mirrorHandle!=NULL){
    close(mirror->sd);
    free(*mirrorHandle);
  }
  *mirrorHandle=NULL;
  return 0;
}
int mirrorSend(void *mirrorHandle,int ndata,unsigned short *data,unsigned int frameno){
  mirrorStruct *mirror=(mirrorStruct*)mirrorHandle;
  //unsigned short sync;
  int ns;
  //ssize_t nr;
  ssize_t tmp;
  int err=0;
  int n;
  unsigned int header[2];
  //int retval;
  //fd_set rfds;
  //struct timeval tv;
  if(mirrorHandle!=NULL){
    //printf("Waiting for mirror sync\n");
    /*//This send from the server seems to take a LONG time on windows...
    //So we decided that the server doesn't send anything.
    FD_ZERO(&rfds);
    FD_SET(mirror->sd, &rfds);
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    retval=select(mirror->sd+1,&rfds,NULL,NULL,&tv);
    if(retval==-1){
      printf("Error in select\n");
      err=1;
    }else if(retval==0){
      printf("Timeout while waiting for mirror\n");
      err=1;
    }else{
      ns=sizeof(short);
      nr=0;
      while(nr<ns){
	tmp=read(mirror->sd,&sync,ns-nr);
	if(tmp==-1){
	  printf("Error reading mirror socket\n");
	  err=1;
	  nr=ns;
	}else if(tmp==0){
	  printf("DMC socket closed?\n");
	  err=1;
	  nr=ns;
	}else{
	  nr+=tmp;
	}
      }
    }
    */
    if(err==0){
      //printf("Got sync %#x, sending %d values to mirror\n",sync,n);
      //if(sync!=0x5555)
      //printf("ERROR: Sync not equal to 0x5555\n");
      header[0]=(0x5555<<16)|ndata;
      header[1]=frameno;
      ns=0;
      n=2*sizeof(unsigned int);
      while(ns<n){
	tmp=write(mirror->sd,&(((char*)header)[ns]),n-ns);
	if(tmp==-1){
	  printf("Error writing mirror header, error: %s\n",strerror(errno));
	  err=1;
	  ns=n;
	}else{
	  ns+=tmp;
	}
      }
      ns=0;
      n=ndata*sizeof(unsigned short);
      while(ns<n){
	tmp=write(mirror->sd,&(((char*)data)[ns]),n-ns);
	if(tmp==-1){
	  printf("Error writing mirror socket\n");
	  err=1;
	  ns=n;
	}else{
	  ns+=tmp;
	}
      }
    }
    //if(err==0)
    //  printf("Values sent to mirror (err %d)\n",err);
  }
  return err;
}
