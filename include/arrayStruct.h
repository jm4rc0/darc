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
#ifndef ARRAYSTRUCT_H
#define ARRAYSTRUCT_H
/**
   holds internal memory allocations
*/

typedef struct{
  char *name;//the name of this data
  void *ptr;//pointer to the data
  char typecode;//type of the data
  int size;//size of the data (bytes)
  void *next;//NULL or a pointer to the next userArrayStruct.
}UserArrayStruct;


typedef struct{
  short *pxlbufs;
  int pxlbufsSize;
  float *flux;
  float *centroids;
  //float *wpucentroids;
  float *dmCommand;
  float *dmCommandSave;
  float *dmCommandFigure;
  float *calpxlbuf;
  //float *corrbuf;
  int fluxSize;
  int centroidsSize;
  int dmCommandSize;
  int calpxlbufSize;
  //int corrbufSize;
  int *subapLocation;
  int subapLocationSize;
  //int *adaptiveCentPos;
  //int adaptiveCentPosSize;
  //float *adaptiveWinPos;
  //int adaptiveWinPosSize;
  UserArrayStruct *userArrayList;//NULL, unless you know what you are doing, and need to share data between modules.
}arrayStruct;
#endif

//Note these functions are not thread safe.  The add and remove functions can only be called during a rtc*Open and rtc*Close functions.  The getUserArray can be called anywhere, and is thread safe (but not thread locked).
int addUserArray(char *name, void *data, char typecode, int size);//add data to the userArrayList
UserArrayStruct *getUserArray(char *name);//Get data from the userArrayList
void *removeUserArray(char *name);//remove user data.  Returns pointer to the data, which can then be freed.
