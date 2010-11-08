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
#include <wchar.h>
#include <string.h>
#include <stdarg.h>
#include "buffer.h"

/**
   Checks that paramList is valid
   (alphabetical order, and of correct lenght (<16 chars), and correct size (16 bytes)
   paramList should have size of at least n*BUFNAMESIZE bytes.
*/
int bufferCheckNames(int n,char *paramList){
  int i;
  int err=0;
  if(sizeof(long long)!=8 || sizeof(wchar_t)!=4){
    printf("ERROR - sizeof(long long)!=8 or sizeof(wchar_t)!=4\n");
    err=1;
  }
  /*No longer needed since checking by strncmp
  if((((unsigned long)paramList)&0xf)!=0){
    printf("Error in bufferCheckNames - must be aligned to 16 byte boundary\n");
    err=1;
    }*/
  for(i=0; i<n; i++){
    if(i>0 && strncmp(&paramList[(i-1)*BUFNAMESIZE],&paramList[i*BUFNAMESIZE],BUFNAMESIZE)>=0){//alphabetical order?
      printf("ERROR - params not in alphabetical order: %16s %16s\n",&paramList[(i-1)*BUFNAMESIZE],&paramList[i*BUFNAMESIZE]);
      err=1;
    }
  }
  return err;
}

/**
   Given a list of n names, puts these into a newly created buffer, which can then be passed to bufferCheckNames and bufferGetIndex.
   e.g. bufferMakeNames(3,"bgImage","flatField","refCentroids");
*/
char *bufferMakeNames(int n,...){
  va_list l;
  char *b;
  int i;
  char *name;
  //if((i=posix_memalign((void**)&b,16,n*BUFNAMESIZE))!=0){
  if((b=calloc(n,BUFNAMESIZE))==NULL){
    printf("Unable to allocate in bufferMakeNames\n");
    return NULL;
  }
  memset(b,0,n*BUFNAMESIZE);
  va_start(l,n);
  for(i=0; i<n; i++){
    name=va_arg(l,char*);
    if(strlen(name)>BUFNAMESIZE){
      printf("Parameter %s is too long - should be max %d characters\n",name,BUFNAMESIZE);
      free(b);
      b=NULL;
      break;
    }else{
      strncpy(&b[i*BUFNAMESIZE],name,BUFNAMESIZE);
    }
  }
  va_end(l);
  if(b!=NULL && bufferCheckNames(n,b)!=0){
    free(b);
    b=NULL;
  }
  return b;
}


/**
   Returns the index of each param named in paramList, or -1 if not found, placed into array index, which should be of size n.  paramList must have n entries, each a null terminated string.
   pbuf is the parameter buffer.
   values is an array of void* with size n.  Each entry is then a pointer to the data, that can be cast as required according to pbuf->nbytes[index] and pbuf->dtype[index], for example, if these are 4 and i, you would do *(int*)(values[index])
   If these are 16 and f, you would do (float*)(values[index])
   paramList is if size n * BUFNAMESIZE (n*16 currently), with each set of BUFNAMESIZE bytes containing the name of the variable.
 */

int bufferGetIndex(paramBuf *pbuf,int n,char *paramList,int *index,void **values,char *dtype,int *nbytes){
  int i=0,j;
  char *buf=pbuf->buf;
  int nhdr=BUFNHDR(pbuf);
  int nfound=0;
  int s;
  //printf("BufferGetIndex %d %p %p\n",n,buf,paramList);
  wmemset(index,-1,n);//initialise to -1.
  while(i<nhdr && buf[16*i]!='\0'){//go through everything in the buffer
    //quick find version...
    /*
    s=0;
    e=n;
    p=n/2;
    found=0;
    while(found==0 && e-s>0){
      if((m=strncmp(&buf[i*16],paramList[p],16))==0){
	//match found
	index[j]=i;
	value[j]=BUFGETVALUE(pbuf,j);
	found=1;
      }else if(m<0){
	//search the first half
	e=p;
	p=(e+s)/2;
      }else{
	//search the second half
	s=p;
	p=(e+s)/2;
      }
      }*/
    for(j=0; j<n; j++){//compare it with our paramList.
      //if(strncmp(&buf[i*16],paramList[j],16)==0){
      s=strncmp(&buf[i*BUFNAMESIZE],&paramList[j*BUFNAMESIZE],16);
      if(s==0){//match found
	index[j]=i;
	values[j]=BUFGETVALUE(pbuf,i);
	dtype[j]=pbuf->dtype[i];
	nbytes[j]=pbuf->nbytes[i];
	nfound++;
	//printf("Found %16s\n",&paramList[j*BUFNAMESIZE]);
	break;
      }else if(s<0){
	break;
      }

      /*
      if(strncmp(&buf[i*BUFNAMESIZE],&paramList[j*BUFNAMESIZE],16)==0){
	printf("strncmp match %16s %16s\n",&buf[i*BUFNAMESIZE],&paramList[j*BUFNAMESIZE]);
      }else
	printf("N: %16s %16s\n",&buf[i*BUFNAMESIZE],&paramList[j*BUFNAMESIZE]);
      if((*((long long*)(&buf[i*BUFNAMESIZE])))==(*((long long*)(&paramList[j*BUFNAMESIZE])))){
	if((*((long long*)(&buf[i*BUFNAMESIZE+8])))==(*((long long*)(&paramList[j*BUFNAMESIZE+8])))){
	  //match found.
	  index[j]=i;
	  values[j]=BUFGETVALUE(pbuf,i);
	  dtype[j]=pbuf->dtype[i];
	  nbytes[j]=pbuf->nbytes[i];
	  nfound++;
	  printf("Found %16s\n",&paramList[j*BUFNAMESIZE]);
	  break;
	}else if((*((long long*)(&buf[i*BUFNAMESIZE+8])))<(*((long long*)(&paramList[j*BUFNAMESIZE+8])))){
	  //match not found, but we're past it in the alphabet now.  So this entry in the buffer isn't needed at the moment.
	  printf("Partial match %16s %16s\n",&buf[i*BUFNAMESIZE],&paramList[j*BUFNAMESIZE]);
	  break;
	  

	}
      }else if((*((long long*)(&buf[i*BUFNAMESIZE])))<(*((long long*)(&paramList[j*BUFNAMESIZE])))){
	//match not found, but we're past it in the alphabet now.  So this entry in the buffer isn't needed at the moment.
	printf("Break... %16s %16s %lld %lld %p %p\n",&buf[i*BUFNAMESIZE],&paramList[j*BUFNAMESIZE],(*((long long*)(&buf[i*BUFNAMESIZE]))),(*((long long*)(&paramList[j*BUFNAMESIZE]))),&buf[i*BUFNAMESIZE],&paramList[j*BUFNAMESIZE]);
	break;
      }
      */
    }
    i++;
  }
  //printf("bufferGetIndex done, nfound=%d/%d\n",nfound,n);
  return nfound;
}
