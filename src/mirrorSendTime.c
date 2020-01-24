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
   The code here is used to create a shared object library, which can then be swapped around depending on which mirrors/interfaces you have in use, ie you simple rename the mirror file you want to mirror.so (or better, change the soft link), and restart the coremain.

The library is written for a specific mirror configuration - ie in multiple mirror situations, the library is written to handle multiple mirrors, not a single mirror many times.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include "darc.h"
/**
   Find out if this SO library supports your camera.

*/

typedef struct{
  int tmp;
  unsigned int frameno;
  unsigned int *mirrorframeno;
  struct timespec *timestamp;
  int socket;
  char *host;
  unsigned short port;
  char *multicastAdapterIP;
  struct sockaddr_in sin;
  circBuf *rtcActuatorBuf;

}mirrorStruct;

/**
   Open a camera of type name.  Args are passed in a float array of size n, which can be cast if necessary.  Any state data is returned in camHandle, which should be NULL if an error arises.
   pxlbuf is the array that should hold the data. The library is free to use the user provided version, or use its own version as necessary (ie a pointer to physical memory or whatever).  It is of size npxls*sizeof(short).
   ncam is number of cameras, which is the length of arrays pxlx and pxly, which contain the dimensions for each camera.
   Name is used if a library can support more than one camera.

*/

void mirrordofree(mirrorStruct *mirstr){
  if(mirstr!=NULL){
    if(mirstr->timestamp!=NULL)
      free(mirstr->timestamp);
    free(mirstr);
  }
}

int openMirrorSocket(mirrorStruct *mirstr){
  int err=0;
  struct hostent *host;
  if((host=gethostbyname(mirstr->host))==NULL){//not thread safe.
    printf("gethostbyname error %s\n",mirstr->host);
    err=1;
  }else{
    if((mirstr->socket=socket(PF_INET,SOCK_DGRAM,0))<0)
      printf("Warning - mirrorUDP cannot open socket\n");

    memset(&mirstr->sin,0,sizeof(mirstr->sin));

    memcpy(&mirstr->sin.sin_addr.s_addr,host->h_addr,host->h_length);
    mirstr->sin.sin_family=AF_INET;
    mirstr->sin.sin_port=htons(mirstr->port);
    if(mirstr->multicastAdapterIP!=NULL){
      //tell the kernel we want to multicast, and that data is sent to everyone (1-255 is level of multicasting, 1==local network only).
      //setsockopt(mirstr->socket,IPPROTO_IP,IP_MULTICAST_TTL,1);//default is 1 anyway.
      //set the interface from which datagrams are sent...
      in_addr_t localadapter = inet_addr(mirstr->multicastAdapterIP);//e.g. "192.168.1.1");
      setsockopt(mirstr->socket,IPPROTO_IP,IP_MULTICAST_IF,(char*)&localadapter,sizeof(localadapter));
    }else{
      printf("No multicast adapter specified - using default NIC\n");
    }
    //to use:
    //sendto(mirstr->socket,buf,len,0,&mirstr->sin,sizeof(mirstr->sin);

  }
  return err;
}

int mirrorOpen(char *name,int narg,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **mirrorHandle,int nacts,circBuf *rtcActuatorBuf,unsigned int frameno,unsigned int **mirrorframeno,int *mirrorframenoSize){
  int err=0;
  char *ptr;
  mirrorStruct *mirstr;
  printf("Initialising mirror %s\n",name);
  if((*mirrorHandle=malloc(sizeof(mirrorStruct)))==NULL){
    printf("Couldn't malloc mirror handle\n");
    return 1;
  }
  memset(*mirrorHandle,0,sizeof(mirrorStruct));
  mirstr=(mirrorStruct*)*mirrorHandle;

  if(narg>0){
    mirstr->port=args[0];
    if(mirstr->port>0){
      mirstr->host=strndup((char*)&(args[1]),(narg-1)*sizeof(int));
      ptr=strchr(mirstr->host,';');
      if(ptr!=NULL){//if the string has a ; in it, then it should be host;multicast interface address, which is then used to define the interface overwhich the packets are to be sent.  e.g. 224.1.1.1;192.168.3.1
        ptr[0]='\0';
        mirstr->multicastAdapterIP=&ptr[1];
      }
      printf("Got host %s\n",mirstr->host);
      if(mirstr->multicastAdapterIP!=NULL)
        printf("Got multicast adapter IP %s\n",mirstr->multicastAdapterIP);
      if((err=openMirrorSocket(mirstr))!=0){
        printf("error opening socket\n");
        mirrordofree(mirstr);
        *mirrorHandle=NULL;
        return 1;
      }
    }else{
      printf("port == 0, therefore mirror will not send\n");
    }
  }else{
    printf("wrong number of args - should be port, hostname[;multicast interface address] (strings)\n");
    mirrordofree(mirstr);
    *mirrorHandle=NULL;
    return 1;
  }

  printf("mirrorFrameNoSize = %d\n",*mirrorframenoSize);
  if(*mirrorframenoSize<2){
    if(*mirrorframeno!=NULL)
      free(*mirrorframeno);
    if((*mirrorframeno=calloc(2,sizeof(unsigned int)))==NULL){
      printf("Unable to alloc mirrorframeno\n");
      mirrordofree(mirstr);
      *mirrorHandle=NULL;
    }
    *mirrorframenoSize=2;
  }

  mirstr->mirrorframeno=*mirrorframeno;
  mirstr->frameno = 0;
  mirstr->timestamp=malloc(sizeof(struct timespec)*1);
  printf("mirror initialised\n");

  mirstr->rtcActuatorBuf=rtcActuatorBuf;

  return err;
}

/**
   Close a camera of type name.  Args are passed in the float array of size n, and state data is in camHandle, which should be freed and set to NULL before returning.
*/
int mirrorClose(void **mirrorHandle){
  printf("Closing mirror\n");
  mirrordofree(*mirrorHandle);
  *mirrorHandle=NULL;
  return 0;
}

/**
   Return <0 on error, or otherwise, the number of clipped actuators (or zero).
*/
int mirrorSend(void *mirrorHandle,int n,float *data,unsigned int frameno,double timestamp,int err,int writeCirc){
  mirrorStruct *mirstr=(mirrorStruct*)mirrorHandle;
  struct timespec timestamp2;
  double thistime;
  unsigned int header[3];
  if(mirrorHandle!=NULL){
    // printf("Sending %d values to non-existant mirror\n",n);

    if(writeCirc){
      if(mirstr->rtcActuatorBuf!=NULL && mirstr->rtcActuatorBuf->datasize!=n*sizeof(float)){
        if(circReshape(mirstr->rtcActuatorBuf,1,&n,'f')!=0){
          printf("Error reshaping rtcActuatorBuf\n");
        }
      }
      circAddForce(mirstr->rtcActuatorBuf,data,timestamp,frameno);
    }
    clock_gettime(CLOCK_MONOTONIC,&timestamp2);
    // mirstr->mirrorframeno[0]=mirstr->frameno++;
    mirstr->mirrorframeno[0]=(unsigned int)timestamp2.tv_sec;//-TIMESECOFFSET;
    mirstr->mirrorframeno[1]=(unsigned int)timestamp2.tv_nsec;
    if(mirstr->port>0){
      header[0]=(unsigned int)frameno;
      header[1]=(unsigned int)timestamp2.tv_sec;//-TIMESECOFFSET;
      header[2]=(unsigned int)timestamp2.tv_nsec;
      sendto(mirstr->socket,header,3*sizeof(unsigned int),0,&mirstr->sin,sizeof(mirstr->sin));
    }
  }
  return 0;
}

int mirrorNewParam(void *mirrorHandle,paramBuf *buf,unsigned int frameno,arrayStruct *arr){
  if(mirrorHandle!=NULL){
    printf("Changing mirror params\n");
  }
  return 0;
}
