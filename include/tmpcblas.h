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
//gcc -Wall -O3 -o tmpsubsupap tmpsubsupap.c -lgslcblas -funroll-loops -msse2 -mfpmath=sse -march=native

#ifndef TMPCBLAS_H //header guard
#define TMPCBLAS_H

inline float agb_cblas_sasum1(int n,float *x){
  /*sasum with inc=1.  Sums vector x.
    cblas_sasum(n,x,1);
   */
  int i;
  float sum=0;
  for(i=0; i<n; i++){
    sum+=x[i];
  }
  return sum;
}
inline void agb_cblas_saxpy111(int n, float *x, float *y){
  /*does saxpy with inc=1, alpha=1
    y+=x
    cblas_saxpy(n,1.0,x,1,y,1);
  */
  int i;
  for(i=0; i<n; i++){
    y[i]+=x[i];
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
}
inline void agb_cblas_saxpy11(int n,float a,float *x,float *y){
  /*does saxpy with inc=1.
    y+=a*x
    cblas_saxpy(n,a,x,1,y,1);
  */
  int i;
  for(i=0; i<n; i++)
    y[i]+=a*x[i];
}
inline void agb_cblas_sgemvRowNN1N101(int n, float *a, float *x,float *y){
  /*perform sgemv with m==n and lda==m, alpha=1, beta=0 and inc=1.
    Row major format, no transpose.
    Does y=a.x
    cblas_sgemv(CblasRowMajor,CblasNoTrans,n,n,1.,a,n,x,1,0.,y,1);
   */
  int i,j;
  for(i=0; i<n; i++){
    y[i]=0;
    for(j=0; j<n; j++){
      y[i]+=a[i*n+j]*x[j];
    }
  }
}
inline void agb_cblas_sgemvColMN1M111(int m, int n, float *a,float *x,float *y){
  /*perform sgemv with lda==m, alpha=1, beta=1 and inc=1.
    Col major format, no transpose.
    Does y+=a.x
    cblas_sgemv(CblasColMajor,CblasNoTrans,m,n,1.,a,m,x,1,1.,y,1);
  */
  int i,j;
  for(i=0; i<n; i++){
    for(j=0; j<m; j++){
      y[j]+=a[j+i*m]*x[i];
    }
  }
}

#endif //header guard