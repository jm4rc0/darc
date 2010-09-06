#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "qsort.h"

int cmp(const float *a,const float *b){
  if(*a==*b)
    return 0;
  else if(*a<*b)
    return -1;
  return 1;
}
#define cmp2(a,b) ((*a)<(*b))

inline int qfind(float *a,int n,float x){
  int i=0;
  int mid;
  while(i<n){
    mid=(n+i)/2;
    if(x<a[mid])
      n=mid;
    else if(x>a[mid])
      i=mid+1;
    else
      return mid-1;
  }
  return (n+i)/2-1;
}

int main(int argc,char **argv){
  float *a;
  float *b;
  float *c;
  int size=64;
  int size2;
  int i,j;
  int cnt,cnt2;
  int pos;
  float v;
  struct timeval t1,t2,t3,t4;
  if(argc>=2)
    size=atoi(argv[1]);
  size2=size/2;
  if(argc>=3)
    size2=atoi(argv[2]);
  srand(time(0));
  a=malloc(sizeof(float)*size);
  b=malloc(sizeof(float)*size);
  c=malloc(sizeof(float)*size);
  for(i=0; i<size; i++)
    a[i]=((float)random())/RAND_MAX;
  memcpy(b,a,sizeof(float)*size);
  memcpy(c,a,sizeof(float)*size);
  gettimeofday(&t1,NULL);
  qsort(a,size,sizeof(float),cmp);
  gettimeofday(&t2,NULL);
  //sort all
  QSORT(float,b,size,cmp2);
  gettimeofday(&t3,NULL);
  //now sort half, and put the rest in if greater than those already sorted
  QSORT(float,c,size2,cmp2);
  for(i=size2; i<size; i++){
    if(c[i]>c[0]){
      pos=qfind(c,size2,c[i]);
      if(pos>0)//shift to make room
	memmove(c,&c[1],sizeof(float)*pos);
      //for(j=0; j<pos; j++)
      //	  c[j]=c[j+1];
      c[pos]=c[i];//insert
    }
  }
  gettimeofday(&t4,NULL);


  cnt=0;
  cnt2=0;
  for(i=0; i<size; i++){
    if(a[i]!=b[i])
      cnt++;
    //printf("%3d: %g %g\n",i,a[i],c[i]);
  }
  for(i=0; i<size2; i++){
    if(c[i]!=a[i+size-size2])
      cnt2++;
  }
  printf("%g %g %g diff: %d %d\n",t2.tv_sec-t1.tv_sec+1e-06*(t2.tv_usec-t1.tv_usec),t3.tv_sec-t2.tv_sec+1e-06*(t3.tv_usec-t2.tv_usec),t4.tv_sec-t3.tv_sec+1e-06*(t4.tv_usec-t3.tv_usec),cnt,cnt2);


  pos=qfind(a,size,0.5);
  printf("%d %g %g %g\n",pos,a[pos-1],a[pos],a[pos+1]);
  pos=qfind(a,size,0.0005);
  printf("%d %g %g %g\n",pos,a[pos-1],a[pos],a[pos+1]);
  pos=qfind(a,size,1.5);
  printf("%d %g %g %g\n",pos,a[pos-1],a[pos],a[pos+1]);
  pos=qfind(a,size,(a[0]+a[1])/2.);
  printf("%d %g %g %g\n",pos,a[pos-1],a[pos],a[pos+1]);
  printf("\n");
  for(i=0; i<10; i++){
    v=((float)random())/RAND_MAX;
    pos=qfind(a,size,v);
    printf("%d %g %g %g\n",pos,a[pos],v,a[pos+1]);
  }
  v=a[size-size2];
  cnt=0;
  for(i=0; i<size; i++)
    if(a[i]>=v)
      cnt++;
  printf("Using %d\n",cnt);
  return 0;
}
