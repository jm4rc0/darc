#include <stdio.h>
#include <stdlib.h>
#include <wchar.h>
#include <string.h>
#include <stdarg.h>
#include "buffer.h"

/**
   Checks that paramList is valid
   (alphabetical order, and of correct lenght (<16 chars), and correct size (16 bytes)
*/
int bufferCheckNames(int n,char *paramList){
  int i;
  int err=0;
  if(sizeof(long long)!=8 || sizeof(wchar_t)!=4){
    printf("ERROR - sizeof(long long)!=8 or sizeof(wchar_t)!=4\n");
    err=1;
  }
  if(sizeof(paramList)!=n*16){
    printf("Error - paramList not equal to n*16 bytes\n");
    err=1;
  }
  for(i=0; i<n; i++){
    if(i>0 && strncmp(&paramList[(i-1)*BUFNAMESIZE],&paramList[i*BUFNAMESIZE],BUFNAMESIZE)>=0){//alphabetical order?
      printf("ERROR - params not in alphabetical order\n");
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
  if((b=calloc(n,BUFNAMESIZE))==NULL){
    printf("Unable to allocate in bufferMakeNames\n");
    return NULL;
  }
  va_start(l,n);
  for(i=0; i<n; i++){
    name=va_arg(l,char*);
    if(strlen(name)>BUFNAMESIZE){
      printf("Parameter %s is too long - should be max %d characters\n",name,BUFNAMESIZE);
      free(b);
      b=NULL;
      break;
    }else{
      strncpy(&b[i*BUFNAMESIZE],va_arg(l,char*),BUFNAMESIZE);
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
      //do the comparison as long long (8 bytes) - quicker than strncmp()
      if((*((long long*)(&buf[i*BUFNAMESIZE]))==*((long long*)(&paramList[j*BUFNAMESIZE])))){
	if((*((long long*)(&buf[i*BUFNAMESIZE+8]))==*((long long*)(&paramList[j*BUFNAMESIZE+8])))){
	  //match found.
	  index[j]=i;
	  values[j]=BUFGETVALUE(pbuf,i);
	  dtype[j]=pbuf->dtype[i];
	  nbytes[j]=pbuf->nbytes[i];
	  nfound++;
	  break;
	}else if((*((long long*)(&buf[i*BUFNAMESIZE+8]))<*((long long*)(&paramList[j*BUFNAMESIZE+8])))){
	  //match not found, but we're past it in the alphabet now.  So this entry in the buffer isn't needed at the moment.
	  break;
	  

	}
      }else if((*((long long*)(&buf[i*BUFNAMESIZE]))<*((long long*)(&paramList[j*BUFNAMESIZE])))){
	//match not found, but we're past it in the alphabet now.  So this entry in the buffer isn't needed at the moment.
	break;
      }
    }
    i++;
  }
  return nfound;
}
