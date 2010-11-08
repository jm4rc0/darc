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
