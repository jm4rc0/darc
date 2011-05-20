#ifdef DARCCPP
extern "C" 
#endif
int gpusgemvMN1M101(int M,int N,float *Da,float *Dx,float *Dy,int numThreads);
#ifdef DARCCPP
extern "C" 
#endif
int gpusgemvMN1M111(int M,int N,float *Da,float *Dx,float *Dy,int numThreads);
