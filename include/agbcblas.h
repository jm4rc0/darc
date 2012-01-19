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
inline float agb_cblas_sdot11(int n,float *x,float*y);
inline float agb_cblas_sasum1(int n,float *x);
inline void agb_cblas_saxpy111(int n, float *x, float *y);
inline void agb_cblas_saxpym111(int n, float *x, float *y);
inline void agb_cblas_sscal1(int n,float s,float *x);
inline void agb_cblas_saxpy11(int n,float a,float *x,float *y);
inline void agb_cblas_saxpym11(int n,float a,float *x,float *y);
inline void agb_cblas_saxpy1(int n,float a,float *x,int incx,float *y);
inline void agb_cblas_sgemvRowNN1N101(int n, float *a, float *x,float *y);
inline void agb_cblas_sgemvColMN1M111(int m, int n, float *a,float *x,float *y);
inline void agb_cblas_sgemvRowMN1N1m11(int m,int n, float *a, float *x,float *y);
inline void agb_cblas_sgemvRowMN1N101(int m,int n, float *a, float *x,float *y);
inline void agb_cblas_sgemvRowMN1N111(int m, int n, float *a,float *x,float *y);
inline void agb_cblas_sparse_csr_sgemvRowMN1N101(int m,int n, int *a, float *x,float *y);
