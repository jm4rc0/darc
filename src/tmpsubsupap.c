/*
agb_cblas_saxpy111
	  cblas_saxpy(p->totCents,1.,p->centroids,1,&p->pmx[p->actCnt*p->totCents],1);
agb_cblas_sscal1
	  cblas_sscal(p->totCents,1./userActSeq[p->actCnt],&p->pmx[p->actCnt*p->totCents],1);//average the pmx.
agb_cblas_saxpy11
	  cblas_saxpy(nacts,p->figureGain,p->actsRequired,1,dmCommand,1);//dmCommand=dmCommand + actsRequired*figureGain.
agb_cblas_saxpy111
	  cblas_saxpy(pp->totPxls,1.,pp->calpxlbuf,1,pp->avCalPxlBuf,1);
agb_cblas_saxpy111
	  cblas_saxpy(pp->totCents,1.,pp->centroids,1,pp->avCentBuf,1);
agb_cblas_sscal1
      cblas_sscal(pp->totPxls,1./pp->nAvImg,pp->avCalPxlBuf,1);
agb_cblas_sscal1
      cblas_sscal(pp->totCents,(float)(1./pp->nAvCent),pp->avCentBuf,1);

agb_Cblas_sgemvRowMN1N101
  CBLAS_ORDER order=CblasRowMajor;
  CBLAS_TRANSPOSE trans=CblasNoTrans;

    cblas_sgemv(order,trans,rs->nacts,rs->nacts,alpha,rs->gainE,rs->nacts,reconStruct->latestDmCommand,inc,beta,glob->arrays->dmCommand,inc);

agb_cblas_sgemvCol1M111
  CBLAS_ORDER order=CblasColMajor;
  CBLAS_TRANSPOSE trans=CblasNoTrans;
step=2*nsubapsDoing
 alpha=1
inc=1
beta=1
  cblas_sgemv(order,trans,rs->nacts,step,alpha,&(rs->rmxT[threadInfo->centindx*rs->nacts]),rs->nacts,&(info->centroids[threadInfo->centindx]),inc,beta,&rs->dmCommandArr[rs->nacts*threadInfo->threadno],inc);

agb_cblas_saxpy111
  cblas_saxpy(rs->nacts,1.,&rs->dmCommandArr[rs->nacts*threadInfo->threadno],1,glob->arrays->dmCommand,1);

*/
#include <stdio.h>
#include <stdlib.h>
#include <gsl/gsl_cblas.h>
#include <time.h>
#include <sys/time.h>
#include "agbcblas.h"//gcc -Wall -O3 -o tmpsubsupapusingagbcblas tmpsubsupap.c -lgslcblas agbcblas.o
//#include "tmpcblas.h"//gcc -Wall -O3 -o tmpsubsupap tmpsubsupap.c -lgslcblas -funroll-loops -msse2 -mfpmath=sse -march=native
typedef enum CBLAS_ORDER CBLAS_ORDER;
typedef enum CBLAS_TRANSPOSE CBLAS_TRANSPOSE;


			       


//gcc -Wall -O3 -o tmpsubsupap tmpsubsupap.c -lgslcblas -funroll-loops
//or follow instructions in agbcblas.h
//or change agb_cblas to agb_cblas and use tmpcblas.o
int main(int argc, char ** argv){
  CBLAS_ORDER order=CblasRowMajor;
  CBLAS_TRANSPOSE trans=CblasNoTrans;

  FILE *fd;
  char hdr[2880];
  float subap[256];
  float sum=0;
  int i,j;
  int n=256;
  int inc=1;
  float sum2=0;
  struct timeval t1,t2,t3;
  double d1=0.,d2=0.;
  int ntests=10000;
  int totCents=288;
  float *cents;
  float *y1,*y2;
  int nacts=54;
  float *acts;
  int totPxls=128*128*4;
  float *pxls;
  float *gainE;
  float beta;
  float alpha;
  int step;
  int m;
  float *arr;
  if((fd=fopen("subap.fits","r"))==NULL){
    printf("error opening file subap.fits\n");
    return 0;
  }
  if(argc==2){
    ntests=atoi(argv[1]);
  }
  printf("testing sasum1\n");
  j=fread(hdr,2880,1,fd);
  j=fread(subap,256,4,fd);
  for(j=0; j<ntests; j++){
    //sum=0;
    gettimeofday(&t1,NULL);
    sum=agb_cblas_sasum1(n,subap);
    //for(i=0; i<n; i++){
    //sum+=subap[i];
    //}
    gettimeofday(&t2,NULL);
    sum2=cblas_sasum(n,subap,inc);
    gettimeofday(&t3,NULL);
    d1+=t2.tv_sec-t1.tv_sec+(t2.tv_usec-t1.tv_usec)*1e-6;
    d2+=t3.tv_sec-t2.tv_sec+(t3.tv_usec-t2.tv_usec)*1e-6;
  }
  printf("%g %g times %g %g\n",sum,sum2,d1,d2);
  printf("testing saxpy111\n");
  y1=(float*)malloc(sizeof(float)*totPxls);
  y2=(float*)malloc(sizeof(float)*totPxls);
  cents=(float*)malloc(sizeof(float)*totCents);
  acts=(float*)malloc(sizeof(float)*nacts*3);
  pxls=(float*)malloc(sizeof(float)*totPxls);
  gainE=(float*)malloc(sizeof(float)*nacts*nacts);
  for(i=0; i<totCents; i++){
    cents[i]=random()/(RAND_MAX-1.)*10;
    y1[i]=y2[i]=random()/(RAND_MAX-1.);
  }
  d1=d2=0.;
  for(j=0; j<ntests; j++){
    gettimeofday(&t1,NULL);
    agb_cblas_saxpy111(totCents,cents,y1);
    gettimeofday(&t2,NULL);
    cblas_saxpy(totCents,1.,cents,1,y2,1);
    gettimeofday(&t3,NULL);
    d1+=t2.tv_sec-t1.tv_sec+(t2.tv_usec-t1.tv_usec)*1e-6;
    d2+=t3.tv_sec-t2.tv_sec+(t3.tv_usec-t2.tv_usec)*1e-6;
  }
  printf("%g %g times %g %g\n",agb_cblas_sasum1(totCents,y1),agb_cblas_sasum1(totCents,y2),d1,d2);
  for(i=0; i<totCents; i++){
    if(y2[i]!=y1[i])
      printf("%d: %g %g\n",i,y1[i],y2[i]);
  }
  printf("testing sscal1\n");
  d1=d2=0.;
  for(j=0; j<ntests; j++){
    gettimeofday(&t1,NULL);
    agb_cblas_sscal1(totCents,1/1.1,y1);
    gettimeofday(&t2,NULL);
    cblas_sscal(totCents,1/1.1,y2,1);
    gettimeofday(&t3,NULL);
    d1+=t2.tv_sec-t1.tv_sec+(t2.tv_usec-t1.tv_usec)*1e-6;
    d2+=t3.tv_sec-t2.tv_sec+(t3.tv_usec-t2.tv_usec)*1e-6;
  }
  printf("%g %g times %g %g\n",agb_cblas_sasum1(totCents,y1),agb_cblas_sasum1(totCents,y2),d1,d2);
  for(i=0; i<totCents; i++){
    if(y2[i]!=y1[i])
      printf("%d: %g %g\n",i,y1[i],y2[i]);
  }
  printf("testing saxpy11\n");
  for(i=0; i<nacts; i++){
    acts[i]=random()/(RAND_MAX-1.)*5;
    y1[i]=y2[i]=random()/(RAND_MAX-1.)*2;
  }
  d1=d2=0.;
  for(j=0; j<ntests; j++){
    gettimeofday(&t1,NULL);
    agb_cblas_saxpy11(nacts,0.5,acts,y1);
    gettimeofday(&t2,NULL);
    cblas_saxpy(nacts,0.5,acts,1,y2,1);
    gettimeofday(&t3,NULL);
    d1+=t2.tv_sec-t1.tv_sec+(t2.tv_usec-t1.tv_usec)*1e-6;
    d2+=t3.tv_sec-t2.tv_sec+(t3.tv_usec-t2.tv_usec)*1e-6;
  }
  printf("%g %g times %g %g\n",agb_cblas_sasum1(nacts,y1),agb_cblas_sasum1(nacts,y2),d1,d2);
  for(i=0; i<nacts; i++){
    if(y2[i]!=y1[i])
      printf("%d: %g %g\n",i,y1[i],y2[i]);
  }
  printf("testing saxpy111\n");
  for(i=0; i<totPxls; i++){
    pxls[i]=random()/(RAND_MAX-1.)*5;
    y1[i]=y2[i]=random()/(RAND_MAX-1.)*2;
  }
  d1=d2=0.;
  for(j=0; j<ntests; j++){
    gettimeofday(&t1,NULL);
    agb_cblas_saxpy111(totPxls,pxls,y1);
    gettimeofday(&t2,NULL);
    cblas_saxpy(totPxls,1.,pxls,1,y2,1);
    gettimeofday(&t3,NULL);
    d1+=t2.tv_sec-t1.tv_sec+(t2.tv_usec-t1.tv_usec)*1e-6;
    d2+=t3.tv_sec-t2.tv_sec+(t3.tv_usec-t2.tv_usec)*1e-6;
  }
  printf("%g %g times %g %g\n",agb_cblas_sasum1(totPxls,y1),agb_cblas_sasum1(totPxls,y2),d1,d2);
  for(i=0; i<totPxls; i++){
    if(y2[i]!=y1[i])
      printf("%d: %g %g\n",i,y1[i],y2[i]);
  }
  printf("testing sscal1\n");
  d1=d2=0.;
  for(j=0; j<ntests; j++){
    gettimeofday(&t1,NULL);
    agb_cblas_sscal1(totPxls,0.99,y1);
    gettimeofday(&t2,NULL);
    cblas_sscal(totPxls,0.99,y2,1);
    gettimeofday(&t3,NULL);
    d1+=t2.tv_sec-t1.tv_sec+(t2.tv_usec-t1.tv_usec)*1e-6;
    d2+=t3.tv_sec-t2.tv_sec+(t3.tv_usec-t2.tv_usec)*1e-6;
  }
  printf("%g %g times %g %g\n",agb_cblas_sasum1(totPxls,y1),agb_cblas_sasum1(totPxls,y2),d1,d2);
  for(i=0; i<totPxls; i++){
    if(y2[i]!=y1[i])
      printf("%d: %g %g\n",i,y1[i],y2[i]);
  }
  printf("testing saxpy111\n");
  for(i=0; i<nacts; i++){
    acts[i]=random()/(RAND_MAX-1.)*5;
    y1[i]=y2[i]=random()/(RAND_MAX-1.)*2;
  }
  d1=d2=0.;
  for(j=0; j<ntests; j++){
    gettimeofday(&t1,NULL);
    agb_cblas_saxpy111(nacts,acts,y1);
    gettimeofday(&t2,NULL);
    cblas_saxpy(nacts,1.,acts,1,y2,1);
    gettimeofday(&t3,NULL);
    d1+=t2.tv_sec-t1.tv_sec+(t2.tv_usec-t1.tv_usec)*1e-6;
    d2+=t3.tv_sec-t2.tv_sec+(t3.tv_usec-t2.tv_usec)*1e-6;
  }
  printf("%g %g times %g %g\n",agb_cblas_sasum1(nacts,y1),agb_cblas_sasum1(nacts,y2),d1,d2);

  for(i=0; i<nacts; i++){
    if(y2[i]!=y1[i])
      printf("%d: %g %g\n",i,y1[i],y2[i]);
  }


  printf("testig n     agb_cblas_sgemvRowNN1N101\n");
  for(i=0; i<nacts; i++){
    acts[i]=random()/(RAND_MAX-1.)*5;
    y1[i]=y2[i]=random()/(RAND_MAX-1.)*2;
    for(j=0; j<nacts; j++){
      gainE[i*nacts+j]=random()/(RAND_MAX-1.)*3;
    }
  }
  d1=d2=0.;
  alpha=1.;
  beta=0.;
  inc=1;
  order=CblasRowMajor;
  trans=CblasNoTrans;
  for(j=0; j<ntests; j++){
    gettimeofday(&t1,NULL);
    agb_cblas_sgemvRowNN1N101(nacts,gainE,acts,y1);
    gettimeofday(&t2,NULL);
    cblas_sgemv(order,trans,nacts,nacts,alpha,gainE,nacts,acts,inc,beta,y2,inc);
    gettimeofday(&t3,NULL);
    d1+=t2.tv_sec-t1.tv_sec+(t2.tv_usec-t1.tv_usec)*1e-6;
    d2+=t3.tv_sec-t2.tv_sec+(t3.tv_usec-t2.tv_usec)*1e-6;
  }
  printf("%g %g times %g %g\n",agb_cblas_sasum1(nacts,y1),agb_cblas_sasum1(nacts,y2),d1,d2);
  for(i=0; i<nacts; i++){
    if(y2[i]!=y1[i])
      printf("%d: %g %g\n",i,y1[i],y2[i]);
  }

  printf("testing     agb_cblas_sgemvColMN1M111\n");
  for(i=0; i<nacts; i++){
    acts[i]=random()/(RAND_MAX-1.)*5;
    y1[i]=y2[i]=random()/(RAND_MAX-1.)*2;
    for(j=0; j<nacts; j++){
      gainE[i*nacts+j]=random()/(RAND_MAX-1.)*3;
    }
  }
  d1=d2=0.;
  alpha=1.;
  beta=1.;
  inc=1;
  step=2;
  order=CblasColMajor;
  trans=CblasNoTrans;
  for(j=0; j<ntests; j++){
    gettimeofday(&t1,NULL);
    agb_cblas_sgemvColMN1M111(nacts,step,gainE,cents,y1);
    gettimeofday(&t2,NULL);
    cblas_sgemv(order,trans,nacts,step,alpha,gainE,nacts,cents,inc,beta,y2,inc);
    gettimeofday(&t3,NULL);
    d1+=t2.tv_sec-t1.tv_sec+(t2.tv_usec-t1.tv_usec)*1e-6;
    d2+=t3.tv_sec-t2.tv_sec+(t3.tv_usec-t2.tv_usec)*1e-6;
  }
  printf("%g %g times %g %g\n",agb_cblas_sasum1(nacts,y1),agb_cblas_sasum1(nacts,y2),d1,d2);
  for(i=0; i<nacts; i++){
    if(y2[i]!=y1[i])
      printf("%d: %g %g %g %g\n",i,y1[i],y2[i],y1[i]-y2[i],(y1[i]-y2[i])/y1[i]);
  }

  printf("Testing agb_cblas_sgemvRowMN1N1m11\n");
  n=nacts;
  m=n*3;
  arr=malloc(sizeof(float)*m*n);
  for(i=0; i<n*m; i++){
    arr[i]=random()/(RAND_MAX-1.)*3;
  }
  for(i=0; i<m; i++){
    y1[i]=y2[i]=random()/(RAND_MAX-1.)*2;
    acts[i]=random()/(RAND_MAX-1.)*5;
  }
  d1=d2=0.;
  alpha=1.;
  beta=-1.;
  inc=1;
  order=CblasRowMajor;
  trans=CblasNoTrans;
  for(j=0; j<ntests; j++){
    gettimeofday(&t1,NULL);
    agb_cblas_sgemvRowMN1N1m11(m,n,arr,acts,y1);
    gettimeofday(&t2,NULL);
    cblas_sgemv(order,trans,m,n,alpha,arr,n,acts,inc,beta,y2,inc);
    gettimeofday(&t3,NULL);
    d1+=t2.tv_sec-t1.tv_sec+(t2.tv_usec-t1.tv_usec)*1e-6;
    d2+=t3.tv_sec-t2.tv_sec+(t3.tv_usec-t2.tv_usec)*1e-6;
  }
  printf("%g %g times %g %g\n",agb_cblas_sasum1(nacts,y1),agb_cblas_sasum1(nacts,y2),d1,d2);
  for(i=0; i<m; i++){
    if(y2[i]!=y1[i])
      printf("%d: %g %g %g %g\n",i,y1[i],y2[i],y1[i]-y2[i],(y1[i]-y2[i])/y1[i]);
  }


  printf("Testing agb_cblas_sgemvRowMN1N101\n");
  n=nacts;
  m=n*3;
  arr=malloc(sizeof(float)*m*n);
  for(i=0; i<n*m; i++){
    arr[i]=random()/(RAND_MAX-1.)*3;
  }
  for(i=0; i<m; i++){
    y1[i]=y2[i]=random()/(RAND_MAX-1.)*2;
    acts[i]=random()/(RAND_MAX-1.)*5;
  }
  d1=d2=0.;
  alpha=1.;
  beta=0;
  inc=1;
  order=CblasRowMajor;
  trans=CblasNoTrans;
  for(j=0; j<ntests; j++){
    gettimeofday(&t1,NULL);
    agb_cblas_sgemvRowMN1N101(m,n,arr,acts,y1);
    gettimeofday(&t2,NULL);
    cblas_sgemv(order,trans,m,n,alpha,arr,n,acts,inc,beta,y2,inc);
    gettimeofday(&t3,NULL);
    d1+=t2.tv_sec-t1.tv_sec+(t2.tv_usec-t1.tv_usec)*1e-6;
    d2+=t3.tv_sec-t2.tv_sec+(t3.tv_usec-t2.tv_usec)*1e-6;
  }
  printf("%g %g times %g %g\n",agb_cblas_sasum1(nacts,y1),agb_cblas_sasum1(nacts,y2),d1,d2);
  for(i=0; i<m; i++){
    if(y2[i]!=y1[i])
      printf("%d: %g %g %g %g\n",i,y1[i],y2[i],y1[i]-y2[i],(y1[i]-y2[i])/y1[i]);
  }


  return 0;

}
