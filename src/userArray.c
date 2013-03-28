#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "arrayStruct.h"
//Note these functions are not thread safe.  The add and remove functions can only be called during a rtc*Open and rtc*Close functions.  The getUserArray can be called anywhere, and is thread safe (but not thread locked).



int addUserArray(arrayStruct *arr,char *name, void *data, char typecode, int size){//not thread safe - call in rtc*Open and rtc*Close functions only (or other single threaded ones, eg newParam). Returns 0 on success, 1 on failure (if already exists).
  UserArrayStruct *ptr=arr->userArrayList;
  UserArrayStruct *new;
  UserArrayStruct *prev=NULL;

  if((new=malloc(sizeof(UserArrayStruct)))==NULL){
    printf("Error mallocing new userArrayStruct\n");
    return 1;
  }
  memset(new,0,sizeof(UserArrayStruct));
  
  while(ptr!=NULL){
    if(strcmp(ptr->name,name)==0){
      printf("Error - userArrayStruct %s already exists\n",name);
      free(new);
      return 1;
    }
    prev=ptr;
    ptr=(UserArrayStruct*)(ptr->next);
  }
  if(prev==NULL)
    arr->userArrayList=new;
  else
    prev->next=new;
  new->name=strdup(name);
  new->ptr=data;
  new->typecode=typecode;
  new->size=size;
  return 0;
}



UserArrayStruct *getUserArray(arrayStruct *arr,char *name){//Get data from the userArrayList... this can be called by multiple threads, so longs as nothing is calling removeUserArray or addUserARray at the same time.
  UserArrayStruct *ptr=arr->userArrayList;
  UserArrayStruct *rt=NULL;
  while(ptr!=NULL){
    if(strcmp(ptr->name,name)==0){
      //this is the one.
      rt=ptr;
      break;
    }
    ptr=(UserArrayStruct*)(ptr->next);
  }
  if(rt==NULL){
    printf("UserArray %s not found\n",name);
  }
  return rt;
}
    
void *removeUserArray(arrayStruct *arr,char *name){//remove user data.  Returns pointer to the data, which can then be freed.  Not thread safe.  Call in rtc*Open or rtc*Close only (or other single threaded ones, eg newParam).
  UserArrayStruct *ptr=arr->userArrayList;
  UserArrayStruct *old;
  UserArrayStruct *prev=NULL;
  void *rt=NULL;
  if(ptr==NULL){
    printf("userArray %s not found\n",name);
    return NULL;
  }
  while(ptr!=NULL){
    if(strcmp(ptr->name,name)==0){
      //this is the one to remove.
      old=ptr;
      if(prev==NULL){//first one...
	arr->userArrayList=(UserArrayStruct*)ptr->next;
      }else{
	prev->next=ptr->next;
      }
      free(old->name);
      rt=old->ptr;
      free(old);
      break;
    }
    prev=ptr;
    ptr=(UserArrayStruct*)(ptr->next);

  }
  if(rt==NULL){//could be the current one (the last one)...
    printf("UserArray %s not found\n",name);
  }
  return rt;
}
