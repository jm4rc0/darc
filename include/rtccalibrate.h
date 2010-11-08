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
#include "circ.h"
#include "arrayStruct.h"
#include "buffer.h"
#define CALIBRATECLOSEARGS void **calibrateHandle
int calibrateClose(CALIBRATECLOSEARGS);
#define CALIBRATENEWPARAMARGS void *calibrateHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr
int calibrateNewParam(CALIBRATENEWPARAMARGS);//Can do finalisation of previous frame if required.
#define CALIBRATEOPENARGS char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **handle,int nthreads,unsigned int frameno,unsigned int **calframeno,int *calframenoSize;
int calibrateOpen(CALIBRATEOPENARGS);
#define CALIBRATENEWFRAMEARGS void *calibrateHandle,unsigned int frameno,double timestamp
int calibrateNewFrame(CALIBRATENEWFRAMEARGS);//non-subap thread (once)  Called after calibrateNewParam in the case that there is a param buffer swap.  Can do finalisation of previous frame if required, and if not already done by calibrateNewParam.  At this point, it is safe to use the calpxlbuf again.  This is called after calibrateNewParam in cases where a buffer swap is requested.
#define CALIBRATENEWFRAMESYNCARGS void *calibrateHandle,unsigned int frameno,double timestamp
int calibrateNewFrameSync(CALIBRATENEWFRAMESYNCARGS);//subap thread (once).  Called after calibrateNewParam and before calibrateNewSubap.
#define CALIBRATESTARTFRAMEARGS void *calibrateHandle,int cam,int threadno
int calibrateStartFrame(CALIBRATESTARTFRAMEARGS);//subap thread (once per thread).  May be called before or after calibrateNewFrame, depending on how threads get scheduled.  Always called after calibrateNewFrameSync.
#define CALIBRATENEWSUBAPARGS void *calibrateHandle,int cam,int threadno,int cursubindx,float **subap,int *subapSize,int *curnpxlx,int *curnpxly
int calibrateNewSubap(CALIBRATENEWSUBAPARGS);//subap thread
#define CALIBRATEENDFRAMEARGS void *calibrateHandle,int cam,int threadno,int err
int calibrateEndFrame(CALIBRATEENDFRAMEARGS);//subap thread (once per thread)
#define CALIBRATEFRAMEFINISHEDSYNCARGS void *calibrateHandle,int err,int forcewrite
int calibrateFrameFinishedSync(CALIBRATEFRAMEFINISHEDSYNCARGS);//subap thread (once)
#define CALIBRATEFRAMEFINISHEDARGS void *calibrateHandle,int err
int calibrateFrameFinished(CALIBRATEFRAMEFINISHEDARGS);//non-subap thread (once)
#define CALIBRATEOPENLOOPARGS void *calibrateHandle
int calibrateOpenLoop(CALIBRATEOPENLOOPARGS);
#define CALIBRATECOMPLETEARGS void *calibrateHandle
int calibrateComplete(CALIBRATECOMPLETEARGS);
