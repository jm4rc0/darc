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
//gcc -rdynamic -o test test.c -ldl
#include <stdlib.h>
#include <stdio.h>
#include <dlfcn.h>
#include <unistd.h>

int main(void){
  void *dl;
  int (*fn)(void*);
  int (*fn2)(void*);
  //if((dl=dlopen("/home/ali/cvsstuff/rtc/librtccamera.so",RTLD_LAZY))==NULL){
  if((dl=dlopen("./librtccamera.so",RTLD_LAZY))==NULL){
    printf("dlopen failed\n");
    return 1;
  }
  if((fn2=(int*)dlsym(dl,"camStopFraming"))==NULL){
    printf("dlsym failed\n");
    return 1;
  }
  (*fn2)(NULL);

  if(dlclose(dl)==0){
    printf("dl closed okay\n");
  }else{
    printf("Failed to close dl\n");
    return 1;
  }
  if((dl=dlopen("/home/ali/cvsstuff/rtc/librtccamera.so",RTLD_LAZY))==NULL){
    printf("dlopen failed\n");
    return 1;
  }
  if((*(void**)(&fn)=dlsym(dl,"camOpen"))==NULL){
    printf("dlsym failed\n");
    return 1;
  }
  (*fn)("ali");//note - this doesn't pass the rest of the required args, but as they aren't used, it doesn't fail...

  if(dlclose(dl)==0){
    printf("dl closed okay\n");
  }else{
    printf("Failed to close dl\n");
    return 1;
  }


  return 0;
}
