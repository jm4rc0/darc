//The adaptica drivers are only provided 32 bit, while most AO systems
//will be running on 64 bit machines.  Therefore, to drive this
//mirror, we take the approach that a 32 bit process runs, opening
//some shared memory, and a mutex/condition variable, into which the
//darc process writes the data and signals.

//To be able to compile on ubuntu, you need g++-multilib and gcc-multilib  packages

//This file is the 32 bit process.

//OK - so pthread_mutexes are different sizes for 32 and 64 bit processes.  So, that won't work.  Use a pipe instead



#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <sys/mman.h>
#include <sys/stat.h>        /* For mode constants */
#include <fcntl.h>           /* For O_* constants */
#include <errno.h>
#include "mirrorDriver.h"//adaptica driver

using namespace std;

int main(int argc,char **argv){
  int nact;
  mirrorDriver *ac=new mirrorDriver();
  char *buf;
  float *data;
  int err;
  int fd;
  struct stat st;
  int n;
  if(argc>1){
    if(!ac->configIPAddress(argv[1])){
      printf("Error - cannot set IP\n");
      return 1;
    }
  }
  if(!ac->connectToMirror()){
    printf("Couldn't connect to DM.  Please specify IP address as an argument\n");
    return 1;
  }
  nact=ac->getNumMirrorChannels();
  /*
  //Now open some shared memory.
  //This has the form:
  //nact
  //sizeof mutex
  //sizeof cond
  //mutex
  //cond
  //data (float, size nact).
  int size=sizeof(int)*3+sizeof(pthread_mutex_t)+sizeof(pthread_cond_t)+sizeof(float)*nact;
  char name[]="/adaptica";
  char fullname[]="/dev/shm/adaptica";
  int fd;
  shm_unlink(name);//remove if it exists.
  if((fd=shm_open(name,O_RDWR|O_CREAT,0777))==-1){
    printf("shm_open failed for %s:%s\n",name,strerror(errno));
    return 1;
  }
  if(ftruncate(fd,size)==-1){
    printf("ftruncate failed %s:%s\n",name,strerror(errno));
    close(fd);
    return 1;
  }
  buf=(char*)mmap(0,size,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
  close(fd);
  if(buf==MAP_FAILED){
    printf("mmap failed for\n");
    printf("%s with error number %d, error\n",name,errno);
    printf("%s\n",strerror(errno));
    return 1;
  }
  memset(buf,0,size);
  int *ibuf=(int*)buf;
  ibuf[0]=nact;
  ibuf[1]=sizeof(pthread_mutex_t);
  ibuf[2]=sizeof(pthread_cond_t);
  pthread_mutex_t *mutex=(pthread_mutex_t*)&ibuf[3];
  pthread_cond_t *cond=(pthread_cond_t*)&buf[3*sizeof(int)+ibuf[1]];
  float *data=(float*)&buf[3*sizeof(int)+ibuf[1]+ibuf[2]];
  pthread_mutexattr_t mutexattr;
  pthread_condattr_t condattr;
  pthread_mutexattr_init(&mutexattr);
  pthread_mutexattr_setpshared(&mutexattr,PTHREAD_PROCESS_SHARED);
  pthread_mutexattr_setrobust_np(&mutexattr,PTHREAD_MUTEX_ROBUST);
  pthread_mutex_init(mutex,&mutexattr);
  pthread_mutexattr_destroy(&mutexattr);
  pthread_condattr_init(&condattr);
  pthread_condattr_setpshared(&condattr,PTHREAD_PROCESS_SHARED);
  pthread_cond_init(cond,&condattr);
  pthread_condattr_destroy(&condattr);
  
  //now, we simply wait on the cond, and whenever set, send the actuators.
  //The program will exit when number of actuators changes or the shm is removed.
  struct timespec timeout;
  int err=0;
  struct stat st;
  pthread_mutex_lock(mutex);
  while(ibuf[0]==nact){
    clock_gettime(CLOCK_REALTIME, &timeout);
    timeout.tv_sec+=10;
    err=pthread_cond_timedwait(cond,mutex,&timeout);
    if(err==EOWNERDEAD){
      printf("Mutex lock owner has died - making consistent\n");
      pthread_mutex_consistent_np(mutex);
      err=0;
    }
    if(err==0){
      //send the data.
      if(!ac->setChannels(data))
	printf("Failed to set channel\n");
    }else{
      //check the shm still exists.
      if(stat(fullname,&st)!=0){//removed...
	printf("%s removed - %s exiting\n",fullname,argv[0]);
	break;
      }
    }
  }
  pthread_mutex_unlock(mutex);
  shm_unlink(name);
  */

  //malloc
  printf("Got %d actuators\n",nact);
  if((data=(float*)calloc(nact+2,sizeof(float)))==NULL){
    printf("Failed to allocate memory\n");
    if(!ac->disconnect())
      printf("Error disconnecting\n");
    return 1;
  }
    
  //Create a fifo.
  if((err=mkfifo("/tmp/adaptica",0666))!=0){
    printf("Error creating /tmp/adaptica fifo - exiting: %s\n",strerror(errno));
    if(!ac->disconnect())
      printf("Error disconnecting\n");
    return 1;
  }
  if((fd=open("/tmp/adaptica",O_RDONLY))==-1){
    printf("Failed to open /tmp/adaptica: %s\n",strerror(errno));
    unlink("/tmp/adaptica");
    if(!ac->disconnect())
      printf("Error disconnecting\n");
    return 1;
  }
  //write number of actuators
  /*if(write(fd,&nact,sizeof(int))!=sizeof(int)){
    printf("Failed to write nact: %s\n",strerror(errno));
    unlink("/tmp/adaptica");
    if(!ac->disconnect())
      printf("Error disconnecting\n");
    return 1;
    } */   
  while(stat("/tmp/adaptica",&st)==0){//wait until pipe removed
    if((n=read(fd,data,sizeof(float)*(nact+2)))!=sizeof(float)*(nact+2)){
      printf("Failed to read actuators (read %d/%d bytes)\n",n,sizeof(float)*(nact+2));
      break;
    }
    if(data[0]!=0 && data[nact+1]!=0){
      printf("Data corrupt - should start and end with zero\n");
    }else if(!ac->setChannels(data))
      printf("Failed to set channels\n");
  }
  close(fd);

  unlink("/tmp/adaptica");
  if(!ac->disconnect())
    printf("Error disconnecting\n");
  return 0;

}
