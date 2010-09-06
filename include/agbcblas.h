inline float agb_cblas_sasum1(int n,float *x);
inline void agb_cblas_saxpy111(int n, float *x, float *y);
inline void agb_cblas_sscal1(int n,float s,float *x);
inline void agb_cblas_saxpy11(int n,float a,float *x,float *y);
inline void agb_cblas_saxpy1(int n,float a,float *x,int incx,float *y);
inline void agb_cblas_sgemvRowNN1N101(int n, float *a, float *x,float *y);
inline void agb_cblas_sgemvColMN1M111(int m, int n, float *a,float *x,float *y);
inline void agb_cblas_sgemvRowMN1N1m11(int m,int n, float *a, float *x,float *y);
inline void agb_cblas_sgemvRowMN1N101(int m,int n, float *a, float *x,float *y);
