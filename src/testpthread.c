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
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <fcntl.h>
#include <pthread.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
//description, deliverables (dates), fte/cost
int main(int argc,char **argv){
  pthread_cond_t *cond;
  pthread_condattr_t condattr;
  pthread_mutex_t m;
  int fd;
  if(argc==1){
    //create condition variable.
    if((fd=shm_open("/cond",O_RDWR|O_CREAT,0777))==-1){
      printf("shm_open failed for /cond:%s\n",strerror(errno));
      return 0;
    }
    if(ftruncate(fd,sizeof(pthread_cond_t))==-1){
      printf("ftruncate failed:%s\n",strerror(errno));
      close(fd);
      return 0;
    }
    printf("Doing mmap\n");
    cond=mmap(0,sizeof(pthread_cond_t),PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
    close(fd);
    if(cond==MAP_FAILED){
      printf("mmap failed for\n");
      printf("cond with error number %d, error\n",errno);
      printf("%s\n",strerror(errno));
      return 0;
    }
    memset(cond,0,sizeof(pthread_cond_t));
    pthread_condattr_init(&condattr);
    pthread_condattr_setpshared(&condattr,PTHREAD_PROCESS_SHARED);
    pthread_cond_init(cond,&condattr);
    pthread_condattr_destroy(&condattr);
    pthread_mutex_init(&m,NULL);
    pthread_mutex_lock(&m);
    printf("Waiting\n");
    pthread_cond_wait(cond,&m);
    pthread_mutex_unlock(&m);
    printf("waited\n");
  }else{
    if((fd=shm_open("/cond",O_RDWR,0777))==-1){
      printf("shm_open failed for /cond:%s\n",strerror(errno));
      return 0;
    }
    printf("Doing mmap\n");
    cond=mmap(0,sizeof(pthread_cond_t),PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
    close(fd);
    if(cond==MAP_FAILED){
      printf("mmap failed for\n");
      printf("cond with error number %d, error\n",errno);
      printf("%s\n",strerror(errno));
      return 0;
    }
    printf("signalling\n");
    pthread_cond_signal(cond);
    printf("done\n");
  }
  return 0;
}
