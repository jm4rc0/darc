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
#include <pthread.h>
#include "rtcfigure.h"
/**
   Find out if this SO library supports your camera.

*/

typedef struct{
  int go;
  pthread_mutex_t m;
  pthread_cond_t cond;
  float **actsRequired;//shared with the figure sensor RTC core.
  pthread_t threadid;
  float *acts;//temporary space for reading actuators into
  int nacts;
  useconds_t sleep;
  unsigned int *frameno;
}figureStruct;
int figureDofree(void **figureHandle){
  printf("TODO - figureDofree\n");
  return 0;
}

int figureGetActuators(figureStruct *f){
  //actuators should arrive into f->acts.
  usleep(f->sleep);
  return 0;
}
int figureSubtractPiston(figureStruct *f){
  int i;
  float s=0.;
  for(i=0; i<f->nacts; i++)
    s+=f->acts[i];
  s/=f->nacts;
  for(i=0; i<f->nacts; i++)
    f->acts[i]-=s;
  return 0;
}
/**
   A thread started by figureOpen and stopped by figureClose, which get new actuator setpoints when they are ready, and copies them into actsRequired.
*/
void *figureWorker(void *ff){
  figureStruct *f=ff;
  while(f->go){
    //get the actuators from the actuator interface
    figureGetActuators(f);
    figureSubtractPiston(f);
    //Now write the actuators into the RTC array - actsRequired.
    pthread_mutex_lock(&f->m);
    if(*(f->actsRequired)==NULL){
      if((*(f->actsRequired)=malloc(f->nacts*sizeof(float)))==NULL){
	printf("Error actsRequired malloc\n");
      }
    }
    if(*(f->actsRequired)!=NULL){
      memcpy(*(f->actsRequired),f->acts,sizeof(float)*f->nacts);
      pthread_cond_signal(&f->cond);
    }
    pthread_mutex_unlock(&f->m);
  }
  //do some clearing up.
  return NULL;
}

/**
   Open a channel for reading actuator setpoints into this figure sensor.  Must be of type name.  Args are passed in an int array of size n, which can be cast if necessary.  Any state data is returned in figureHandle, which should be NULL if an error arises.
   Name is used if a library can support more than one camera, and to check that the currently compiled library is what you want it to be.
   The mutex should be obtained whenever new actuator setpoints arrive and are placed into actsRequired.  actsRequired should be allocated.
*/

int figureOpen(char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **figureHandle,int nthreads,unsigned int frameno,unsigned int **figureframeno,int *figureframenoSize,int totCents,int nacts,pthread_mutex_t m,pthread_cond_t cond,float **actsRequired){
  int err;
  figureStruct *f;
  printf("Initialising figure %s\n",name);
    if((*figureHandle=malloc(sizeof(figureStruct)))==NULL){
      printf("Error malloc figureHandle\n");
      err=1;
    }else{
      f=(figureStruct*)*figureHandle;
      memset(f,0,sizeof(figureStruct));
      f->go=1;
      f->m=m;
      f->cond=cond;
      f->actsRequired=actsRequired;
      f->nacts=nacts;
      f->sleep=500;
      if(*figureframenoSize<1){
	if((*figureframeno=malloc(sizeof(int)))==NULL){
	  printf("figure failed malloc\n");
	  err=1;
	}else
	  *figureframenoSize=1;
      }
      f->frameno=*figureframeno;
      if((f->acts=malloc(sizeof(float)*nacts))==NULL){
	printf("Unable to malloc temprary acts array\n");
	err=1;
      }
      if(err==0 && pthread_create(&f->threadid,NULL,figureWorker,f)){
	printf("pthread_create figureWorker failed\n");
	err=1;
      }
    }
  if(err)
    figureDofree(figureHandle);
  return err;
}

/**
   Close a camera of type name.  Args are passed in the float array of size n, and state data is in camHandle, which should be freed and set to NULL before returning.
*/
int figureClose(void **figureHandle){
  printf("Closing figure\n");
  if(*figureHandle!=NULL){
    //pthread_cancel?
    free(*figureHandle);
  }
  *figureHandle=NULL;
  return 0;
}
/**
New parameters ready - use if you need to...
*/
int figureNewParam(void *figureHandle,paramBuf *buf,unsigned int frameno,arrayStruct *arr){
  return 0;
}
