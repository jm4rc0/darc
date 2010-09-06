#include <stdio.h>
#include <string.h>
#include <unistd.h>
typedef struct{
  int sock;
}SendStruct;

/*
translate = [#currently no more that 128 (was 16) different types allowed (was 8).
    types.StringType, 
    "b",#numpy.int8,
    "h",#numpy.int16,
    "i",#numpy.int32,
    None,
    "f",#numpy.float32,
    "d",#numpy.float64,
    types.TupleType,
    types.DictType,#added by agb
    types.IntType,#added by agb
    types.FloatType,#added by agb
    types.ListType,#added by agb
    "Pickled",#added by agb
    'l',#added by agb
    'D',#added by agb
    'L',#added by agb
    'F',#added by agb
    'I',#added by agb
    'H',#added by agb
    numpy.float32#added by agb - for single float32 values... (ie not arrays)
]
*/

int main(int argc,char **argv){
  SendStruct *senstr;
  char *host, *shmname;
  int port;
  if(argc!=4){
    printf("Usage: %s shmname host port\n",argv[0]);
    exit(0);
  }
  senstr=malloc(sizeof(SendStruct));
  memset(senstr,0,sizeof(SendStruct));
  senstr->shmname=argv[1];
  senstr->host=argv[2];
  senstr->port=atoi(argv[3]);
  while(1){
    connect(senstr);
    openshm(senstr);
    while(senstr->connected && senstr->shmopen){
      if(LASTWRITTEN(senstr->cb)>=0){//contains valid data.
	while(LASTWRITTEN(senstr->cb)==senstr->lastsend){
	  if(semtimedop()==-1){//wait for a buffer update.
	    //timeout or semid failed - try reopening buffer...
	    closeshm(semstr);
	  }
	}
	if(semstr->shmopen){
	  //serialise the data and send it over the socket...
	  hdr[0]=TRANSLATE(DTYPE(semstr->cb))<<1;
	  hdr[1:5]=BYTESWAP(calcDatasize(ND(semstr->cb),DIMS(semstr->cb),DTYPE(semstr->cb)));
	  hdr[5:]=thedata;
	  semddata(hdr);
	}
	
      }


    }

  }

  return 0;
}
