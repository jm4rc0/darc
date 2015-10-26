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
#define BUFFERCLOSEARGS void **bufferHandle
int bufferlibClose(BUFFERCLOSEARGS);
#define BUFFERNEWPARAMARGS void *bufferHandle,paramBuf *pbuf,unsigned int frameno,arrayStruct *arr,paramBuf *inactive
int bufferlibNewParam(BUFFERNEWPARAMARGS);//Can do finalisation of previous frame if required.
#define BUFFEROPENARGS char *name,int n,int *args,paramBuf *pbuf,circBuf *rtcErrorBuf,char *prefix,arrayStruct *arr,void **handle,int nthreads,unsigned int frameno,unsigned int** bufferframeno,int *bufferframenosize,paramBuf *inactive
int bufferlibOpen(BUFFEROPENARGS);
#define BUFFERUPDATEARGS void *bufferHandle
int bufferlibUpdate(BUFFERUPDATEARGS);

