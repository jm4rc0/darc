#ifndef ARRAYSTRUCT_H
#define ARRAYSTRUCT_H
/**
   holds internal memory allocations
*/
typedef struct{
  short *pxlbufs;
  int pxlbufsSize;
  float *flux;
  float *centroids;
  float *wpucentroids;
  float *dmCommand;
  float *dmCommandSave;
  float *dmCommandFigure;
  float *calpxlbuf;
  float *corrbuf;
  int fluxSize;
  int centroidsSize;
  int dmCommandSize;
  int calpxlbufSize;
  int corrbufSize;
  int *subapLocation;
  int subapLocationSize;
  int *adaptiveCentPos;
  int adaptiveCentPosSize;
  float *adaptiveWinPos;
  int adaptiveWinPosSize;
}arrayStruct;
#endif
