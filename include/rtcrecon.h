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
#define RECONCLOSEARGS void **reconHandle
int reconClose(RECONCLOSEARGS);
#define RECONNEWPARAMARGS void *reconHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr,int totCents
int reconNewParam(RECONNEWPARAMARGS);
#define RECONOPENARGS char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **handle,int nthreads,unsigned int frameno,unsigned int **reconframeno,int *reconframenoSize,int totCents
int reconOpen(RECONOPENARGS);
#define RECONNEWFRAMEARGS void *reconHandle,unsigned int frameno,double timestamp
int reconNewFrame(RECONNEWFRAMEARGS);//non-subap thread (once)
#define RECONNEWFRAMESYNCARGS void *reconHandle,unsigned int frameno,double timestamp
int reconNewFrameSync(RECONNEWFRAMESYNCARGS);//subap thread (once)
#define RECONSTARTFRAMEARGS void *reconHandle,int cam,int threadno
int reconStartFrame(RECONSTARTFRAMEARGS);//subap thread (once per thread)
#define RECONNEWSLOPESARGS void *reconHandle,int cam,int centindx,int threadno,int nsubapsDoing
int reconNewSlopes(RECONNEWSLOPESARGS);//subap-thread
#define RECONENDFRAMEARGS void *reconHandle,int cam,int threadno,int err
int reconEndFrame(RECONENDFRAMEARGS);//subap thread (once per thread)
#define RECONFRAMEFINISHEDSYNCARGS void *reconHandle,int err,int forcewrite
int reconFrameFinishedSync(RECONFRAMEFINISHEDSYNCARGS);//subap thread (once)
#define RECONFRAMEFINISHEDARGS void *reconHandle,int *err
int reconFrameFinished(RECONFRAMEFINISHEDARGS);//non-subap thread (once)
#define RECONOPENLOOPARGS void *reconHandle
int reconOpenLoop(RECONOPENLOOPARGS);
#define RECONCOMPLETEARGS void *reconHandle
int reconComplete(RECONCOMPLETEARGS);
