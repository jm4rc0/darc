#include <stdio.h>
#include <stdlib.h>
int main(int argc, char** argv){
  float fval;
  int sval=0;
  unsigned short val;
  if(argc==1){
    printf("Usage: %s floatval shortval(optional)\n",argv[0]);
    exit(0);
  }
  if(argc>1){
    fval=atof(argv[1]);
  }
  if(argc>2){
    sval=atoi(argv[2]);
  }
  val=(unsigned short)fval;
  printf("Convert to short: %u\n",val);
  if(sval!=0)
    val+=sval;
  printf("Adding: %u\n",val);
  return 0;
}
