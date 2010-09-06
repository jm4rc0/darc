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
