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
//Important for performance - compile with -O3 and -funroll-loops andmaybe -msse2 and -mfpmath=sse or -mfpmath=both (experimental gcc option - seems to give slightly different results - different rounding or something) -march=native
//gcc -Wall -O3 -c -o agbcblas.o agbcblas.c -lgslcblas -funroll-loops -msse2 -mfpmath=sse -march=native
//#include <string.h>
#include "agbcblas.h"
#ifdef USEICC
#include <immintrin.h>
#endif

inline float agb_cblas_sdot11(int n,float *x,float*y){
  /*vector dot product
   cblas_sdot(n,x,1,y,1)*/
  int i;
  float tot=0;
  for(i=0;i<n;i++){
    tot+=x[i]*y[i];
  }
  return tot;
}

inline float agb_cblas_sasum1(int n,float *x){
  /*sasum with inc=1.  Sums vector x.
    cblas_sasum(n,x,1);
   */
  int i;
  float sum=0;
  for(i=0; i<n; i++){
    //sum+=x[i];
    sum+=*x++;
  }
  return sum;
}
inline void agb_cblas_saxpym111(int n, float *x, float *y){
  /*does saxpy with inc=1, alpha=-1
    y-=x
    cblas_saxpy(n,-1.0,x,1,y,1);
  */
  int i;
  for(i=0; i<n; i++){
    //y[i]-=x[i];
    (*y++)-=*x++;
  }
}
inline void agb_cblas_saxpy111(int n, float *x, float *y){
  /*does saxpy with inc=1, alpha=-1
    y+=x
    cblas_saxpy(n,1.0,x,1,y,1);
  */
  int i;
  for(i=0; i<n; i++){
    //y[i]+=x[i];
    (*y++)+=*x++;
  }
}
inline void agb_cblas_sscal1(int n,float s,float *x){
  /*Does sscal with inc=1.
    x*=s
    cblas_sscal(n,s,x,1);
  */
  int i;
  for(i=0; i<n; i++)
    x[i]*=s;
  //(*x++)*=s;
}
inline void agb_cblas_saxpy11(int n,float a,float *x,float *y){
  /*does saxpy with inc=1.
    y+=a*x
    cblas_saxpy(n,a,x,1,y,1);
  */
  int i;
  for(i=0; i<n; i++)
    //y[i]+=a*x[i];
    (*y++)+=a*(*x++);
}
inline void agb_cblas_saxpym11(int n,float a,float *x,float *y){
  /*does saxpy with inc=1.
    y-=a*x
    cblas_saxpy(n,a,x,1,y,1);
  */
  int i;
  for(i=0; i<n; i++)
    //y[i]+=a*x[i];
    (*y++)-=a*(*x++);
}
inline void agb_cblas_saxpy1(int n,float a,float *x,int incx,float *y){
  /*does saxpy with incy=1.
    y+=a*x
    cblas_saxpy(n,a,x,incx,y,1);
  */
  int i;
  int id=0;
  for(i=0; i<n; i++){
    y[i]+=a*x[id];
    id+=incx;
    //(*y++)+=a*(*x);
    //x+=incx;
  }
}
inline void agb_cblas_sgemvRowNN1N101(int n, float *a, float *x,float *y){
  /*perform sgemv with m==n and lda==m, alpha=1, beta=0 and inc=1.
    Row major format, no transpose.
    Does y=a.x
    cblas_sgemv(CblasRowMajor,CblasNoTrans,n,n,1.,a,n,x,1,0.,y,1);
   */
  int i,j;
  int pos=0;
  float tmp;
  for(i=0; i<n; i++){
    //y[i]=0;
    tmp=0.;
    for(j=0; j<n; j++){
      tmp+=a[pos]*x[j];
      pos++;
    }
    y[i]=tmp;
  }
}
inline void agb_cblas_sgemvRowNN1L101(int n,float *a,int lda,float *x,float *y){
  /*perform sgemv with m==n and lda==L(>m), alpha=1, beta=0 and inc=1.
    Row major format, no transpose.
    Does y=a.x
    cblas_sgemv(CblasRowMajor,CblasNoTrans,n,n,1.,a,n,x,1,0.,y,1);
   */
  int i,j;
  int pos=0;
  float tmp;
  int ldainc=lda-n;
  for(i=0; i<n; i++){
    //y[i]=0;
    tmp=0.;
    for(j=0; j<n; j++){
      tmp+=a[pos]*x[j];
      pos++;
    }
    pos+=ldainc;
    y[i]=tmp;
  }
}

inline void agb_cblas_sgemvColMN1M111(int m, int n, float *a,float *x,float *y){
  /*perform sgemv with lda==m, alpha=1, beta=1 and inc=1.
    Col major format, no transpose.
    Does y+=a.x
    cblas_sgemv(CblasColMajor,CblasNoTrans,m,n,1.,a,m,x,1,1.,y,1);
  */
  int i,j;
  int pos=0;
  float tmp;
  for(i=0; i<n; i++){
    tmp=x[i];
    for(j=0; j<m; j++){
      y[j]+=a[pos]*tmp;
      pos++;
    }
  }
}
inline void agb_cblas_sgemvColMN1M101(int m, int n, float *a,float *x,float *y){
  /*perform sgemv with lda==m, alpha=1, beta=0 and inc=1.
    Col major format, no transpose.
    Does y+=a.x
    cblas_sgemv(CblasColMajor,CblasNoTrans,m,n,1.,a,m,x,1,0.,y,1);
  */
  int i,j;
  int pos=0;
  float tmp;
  //memset(y,0,sizeof(float)*m);
  for(i=0;i<m;i++)
    y[i]=0;
  for(i=0; i<n; i++){
    tmp=x[i];
    for(j=0; j<m; j++){
      y[j]+=a[pos]*tmp;
      pos++;
    }
  }
}
inline void agb_cblas_sgemvRowMN1N1m11(int m,int n, float *a, float *x,float *y){
  /*perform sgemv with lda==n, alpha=1, beta=-1 and inc=1.
    Row major format, no transpose.
    Does y=a.x-y
    cblas_sgemv(CblasRowMajor,CblasNoTrans,m,n,1.,a,n,x,1,-1.,y,1);
    y=a.x-y
   */
  int i,j;
  int pos=0;
  float tmp;
  for(i=0; i<m; i++){
    //y[i]=-y[i];
    tmp=0;
    for(j=0; j<n; j++){
      tmp+=a[pos]*x[j];
      pos++;
    }
    y[i]=tmp-y[i];
  }
}
inline void agb_cblas_sgemvRowMN1N101(int m,int n, float *a, float *x,float *y){
  /*perform sgemv with lda==n, alpha=1, beta=0 and inc=1.
    Row major format, no transpose.
    Does y=a.x
    cblas_sgemv(CblasRowMajor,CblasNoTrans,m,n,1.,a,n,x,1,0.,y,1);
    y=a.x
   */
  int i,j;
  int pos=0;
  float tmp;
  //for(i=0; i<m; i++){
  //  y[i]=0;
  //}
  for(i=0; i<m; i++){
    tmp=0.;
    for(j=0; j<n; j++){
      tmp+=a[pos]*x[j];
      pos++;
    }
    y[i]=tmp;
  }
}
inline void agb_cblas_sgemvRowMN1N111(int m, int n, float *a,float *x,float *y){
  /*perform sgemv with lda==m, alpha=1, beta=1 and inc=1.
    Row major format, no transpose.
    Does y+=a.x
    cblas_sgemv(CblasRowMajor,CblasNoTrans,m,n,1.,a,n,x,1,1.,y,1);
  */
  int i,j;
  int pos=0;
  float tmp;
  for(i=0; i<m; i++){
    tmp=y[i];
    for(j=0; j<n; j++){
      tmp+=a[pos]*x[j];
      pos++;
    }
    y[i]=tmp;
  }
}

inline void agb_cblas_sgemvRowMNm1N111(int m,int n, float *a,float *x,float *y){
  /*perform sgemv with lda==m, alpha=-1, beta=1 and inc=1.
    Row major format, no transpose.
    Does y-=a.x
    cblas_sgemv(CblasRowMajor,CblasNoTrans,m,n,-1.,a,n,x,1,1.,y,1);
  */
  int i,j;
  int pos=0;
  float tmp;
  for(i=0; i<m; i++){
    tmp=y[i];
    for(j=0; j<n; j++){
      tmp-=a[pos]*x[j];
      pos++;
    }
    y[i]=tmp;
  }
}

inline void agb_cblas_sgemvRowMN1L101(int m, int n, float *a,int l,float*x,float*y){
  /*perform sgemv with lda==l, alpha=1, beta=0, inc=1.
    Row major format, no transpose.
    Does y=a.x
    cblas_sgemv(CblasRowMajor,CblasNoTrans,m,n,1.,a,l,x,1,0.,y,1);
  */
  int i,j;
  int pos=0;
  float tmp;
  int lmn=l-n;
  for(i=0;i<m;i++){
    tmp=0.;
    for(j=0;j<n;j++){
      tmp+=a[pos]*x[j];
      pos++;
    }
    pos+=lmn;
    y[i]=tmp;
  }
}
inline void agb_cblas_sgemvRowMN1L111(int m, int n, float *a,int l,float*x,float*y){
  /*perform sgemv with lda==l, alpha=1, beta=1, inc=1.
    Row major format, no transpose.
    Does y=a.x
    cblas_sgemv(CblasRowMajor,CblasNoTrans,m,n,1.,a,l,x,1,1.,y,1);
  */
  int i,j;
  int pos=0;
  float tmp;
  int lmn=l-n;
  for(i=0;i<m;i++){
    tmp=y[i];
    for(j=0;j<n;j++){
      tmp+=a[pos]*x[j];
      pos++;
    }
    pos+=lmn;
    y[i]=tmp;
  }
}


//sparse stuff
inline void agb_cblas_sparse_csr_sgemvRowMN1N101(int m,int n, int *a, float *x,float *y){
  /*perform sgemv with lda==n, alpha=1, beta=0 and inc=1.
    Row major format, no transpose.
    Does y=a.x
    cblas_sgemv(CblasRowMajor,CblasNoTrans,m,n,1.,a,n,x,1,0.,y,1);
    y=a.x
    x is of size n, y is of size m.
    csr format.
    A suitable array can be created using:
    csr=scipy.sparse.csr(denseMatrix)
    a=numpy.concatenate([csr.indptr.astype(numpy.int32),csr.indices.astype(numpy.int32),csr.data.astype(numpy.float32).view(numpy.int32)])
   */
  int i,j,k;
  
  float tmp;
  int *rowindptr=(int*)a;
  int *colindices=(int*)&a[m+1];
  float *data=(float*)&a[m+1+a[m]];
  for(i=0; i<m; i++){
    tmp=0.;
    for(j=rowindptr[i];j<rowindptr[i+1];j++){
      k=colindices[j];
      tmp+=data[j]*x[k];
    }
    y[i]=tmp;
  }
}

#ifdef USEICC
inline int ipow(int base, int exp){
    int result = 1;
    while (exp){
        if (exp & 1)
            result *= base;
        exp >>= 1;
        base *= base;
    }
    return result;
}

inline void agb_cblas_32sgemvColMN1M111(int m, int n, void *a, float *x, float *y){
    /* manual cblas_sgemv for implementing 16 bit stored rmx */
    int i,j,k;
    float * const vy = y;
    float * const vx = x;
    const int vects = m/16;
    const int step  = n/8;
    register __m512 floats0,floats1,floats2,floats3,floats4,floats5,floats6,floats7;
    register __m512 tmpr0,  tmpr1,  tmpr2,  tmpr3,  tmpr4,  tmpr5,  tmpr6,  tmpr7;
    register __m512 dmCom;
    __mmask16 mask;

    for(i=0; i<step; i++){
        tmpr0 = _mm512_set1_ps(vx[8*i+0]);
        tmpr1 = _mm512_set1_ps(vx[8*i+1]);
        tmpr2 = _mm512_set1_ps(vx[8*i+2]);
        tmpr3 = _mm512_set1_ps(vx[8*i+3]);
        tmpr4 = _mm512_set1_ps(vx[8*i+4]);
        tmpr5 = _mm512_set1_ps(vx[8*i+5]);
        tmpr6 = _mm512_set1_ps(vx[8*i+6]);
        tmpr7 = _mm512_set1_ps(vx[8*i+7]);
        for(j=0; j<vects; j++){
            k = j*16;
            dmCom = _mm512_loadu_ps(y+j*16);
            floats0 = _mm512_loadu_ps(a+0*m);
            floats1 = _mm512_loadu_ps(a+4*m);
            floats2 = _mm512_loadu_ps(a+8*m);
            floats3 = _mm512_loadu_ps(a+12*m);
            floats4 = _mm512_loadu_ps(a+16*m);
            floats5 = _mm512_loadu_ps(a+20*m);
            floats6 = _mm512_loadu_ps(a+24*m);
            floats7 = _mm512_loadu_ps(a+28*m);
            dmCom = _mm512_fmadd_ps(floats0, tmpr0, dmCom);
            dmCom = _mm512_fmadd_ps(floats1, tmpr1, dmCom);
            dmCom = _mm512_fmadd_ps(floats2, tmpr2, dmCom);
            dmCom = _mm512_fmadd_ps(floats3, tmpr3, dmCom);
            dmCom = _mm512_fmadd_ps(floats4, tmpr4, dmCom);
            dmCom = _mm512_fmadd_ps(floats5, tmpr5, dmCom);
            dmCom = _mm512_fmadd_ps(floats6, tmpr6, dmCom);
            dmCom = _mm512_fmadd_ps(floats7, tmpr7, dmCom);
            _mm512_store_ps(y+j*16, dmCom);
            a+=64;
        }
        if(m%16){
            // check....
            mask = ipow(2,m%16) - 1;
            dmCom = _mm512_loadu_ps(y+j*16);
            floats0 = _mm512_loadu_ps(a);
            floats1 = _mm512_loadu_ps(a+4*m);
            floats2 = _mm512_loadu_ps(a+8*m);
            floats3 = _mm512_loadu_ps(a+12*m);
            floats4 = _mm512_loadu_ps(a+16*m);
            floats5 = _mm512_loadu_ps(a+20*m);
            floats6 = _mm512_loadu_ps(a+24*m);
            floats7 = _mm512_loadu_ps(a+28*m);
            dmCom = _mm512_fmadd_ps(floats0, tmpr0, dmCom);
            dmCom = _mm512_fmadd_ps(floats1, tmpr1, dmCom);
            dmCom = _mm512_fmadd_ps(floats2, tmpr2, dmCom);
            dmCom = _mm512_fmadd_ps(floats3, tmpr3, dmCom);
            dmCom = _mm512_fmadd_ps(floats4, tmpr4, dmCom);
            dmCom = _mm512_fmadd_ps(floats5, tmpr5, dmCom);
            dmCom = _mm512_fmadd_ps(floats6, tmpr6, dmCom);
            dmCom = _mm512_fmadd_ps(floats7, tmpr7, dmCom);
            _mm512_mask_storeu_ps(y+j*16,mask,dmCom);
            a+=(m%16)*4;
        }
        a+=28*m;
    }
}

inline void agb_cblas_16sgemvColMN1M111(int m, int n, void *a, float *x, float *y){
    /* manual cblas_sgemv for implementing 16 bit stored rmx */
    int i,j,k;
    float * const vy = y;
    float * const vx = x;
    const int vects = m/16;
    const int steps = n/8;
    register __m512  floats0,floats1,floats2,floats3,floats4,floats5,floats6,floats7;
    register __m512  tmpr0,  tmpr1,  tmpr2,  tmpr3,  tmpr4,  tmpr5,  tmpr6,  tmpr7;
    register __m512  dmCom;
    __mmask16 mask;

    for(i=0; i<steps; i++){
        tmpr0 = _mm512_set1_ps(vx[8*i+0]);
        tmpr1 = _mm512_set1_ps(vx[8*i+1]);
        tmpr2 = _mm512_set1_ps(vx[8*i+2]);
        tmpr3 = _mm512_set1_ps(vx[8*i+3]);
        tmpr4 = _mm512_set1_ps(vx[8*i+4]);
        tmpr5 = _mm512_set1_ps(vx[8*i+5]);
        tmpr6 = _mm512_set1_ps(vx[8*i+6]);
        tmpr7 = _mm512_set1_ps(vx[8*i+7]);
        for(j=0; j<vects; j++){
            k=j*16;
            dmCom = _mm512_load_ps(vy+k);
            floats0 = _mm512_cvtph_ps(_mm256_load_si256((a+0*m)));
            floats1 = _mm512_cvtph_ps(_mm256_load_si256((a+2*m)));
            floats2 = _mm512_cvtph_ps(_mm256_load_si256((a+4*m)));
            floats3 = _mm512_cvtph_ps(_mm256_load_si256((a+6*m)));
            floats4 = _mm512_cvtph_ps(_mm256_load_si256((a+8*m)));
            floats5 = _mm512_cvtph_ps(_mm256_load_si256((a+10*m)));
            floats6 = _mm512_cvtph_ps(_mm256_load_si256((a+12*m)));
            floats7 = _mm512_cvtph_ps(_mm256_load_si256((a+14*m)));
            dmCom = _mm512_fmadd_ps(floats0, tmpr0, dmCom);
            dmCom = _mm512_fmadd_ps(floats1, tmpr1, dmCom);
            dmCom = _mm512_fmadd_ps(floats2, tmpr2, dmCom);
            dmCom = _mm512_fmadd_ps(floats3, tmpr3, dmCom);
            dmCom = _mm512_fmadd_ps(floats4, tmpr4, dmCom);
            dmCom = _mm512_fmadd_ps(floats5, tmpr5, dmCom);
            dmCom = _mm512_fmadd_ps(floats6, tmpr6, dmCom);
            dmCom = _mm512_fmadd_ps(floats7, tmpr7, dmCom);
            _mm512_store_ps(vy+k, dmCom);
            a+=32;
        }
        if(m%16){
            // check....
            k = j*16;
            mask = ipow(2,m%16) - 1;
            dmCom = _mm512_load_ps(vy+k);
            floats0 = _mm512_cvtph_ps(_mm256_load_si256((a+0*m)));
            floats1 = _mm512_cvtph_ps(_mm256_load_si256((a+2*m)));
            floats2 = _mm512_cvtph_ps(_mm256_load_si256((a+4*m)));
            floats3 = _mm512_cvtph_ps(_mm256_load_si256((a+6*m)));
            floats4 = _mm512_cvtph_ps(_mm256_load_si256((a+8*m)));
            floats5 = _mm512_cvtph_ps(_mm256_load_si256((a+10*m)));
            floats6 = _mm512_cvtph_ps(_mm256_load_si256((a+12*m)));
            floats7 = _mm512_cvtph_ps(_mm256_load_si256((a+14*m)));
            dmCom = _mm512_fmadd_ps(floats0, tmpr0, dmCom);
            dmCom = _mm512_fmadd_ps(floats1, tmpr1, dmCom);
            dmCom = _mm512_fmadd_ps(floats2, tmpr2, dmCom);
            dmCom = _mm512_fmadd_ps(floats3, tmpr3, dmCom);
            dmCom = _mm512_fmadd_ps(floats4, tmpr4, dmCom);
            dmCom = _mm512_fmadd_ps(floats5, tmpr5, dmCom);
            dmCom = _mm512_fmadd_ps(floats6, tmpr6, dmCom);
            dmCom = _mm512_fmadd_ps(floats7, tmpr7, dmCom);
            _mm512_mask_store_ps(vy+k,mask,dmCom);
            a+=(m%16)*2;
        }
        a+=14*m;
    }
}
#endif
