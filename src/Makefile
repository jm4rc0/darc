##darc, the Durham Adaptive optics Real-time Controller.
#Copyright (C) 2010 Alastair Basden.

#This program is free software: you can redistribute it and/or modify
#it under the terms of the GNU Affero General Public License as
#published by the Free Software Foundation, either version 3 of the
#License, or (at your option) any later version.

#This program is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU Affero General Public License for more details.

#You should have received a copy of the GNU Affero General Public License
#along with this program.  If not, see <http://www.gnu.org/licenses/>.

all: darcmain utilsmodule.so libreconmvm.so libcamfile.so libreconKalman.so sender libcamsocket.so librtccalibrate.so librtcslope.so librtcbuffer.so libmirrorSocket.so libreconAsync.so libcamera.so libcentroider.so libsl240Int32camNoCam.so libmirror.so libmirrorNoSL240.so libfigure.so libnosl240centroider.so libmirrorSHM.so libfigureSL240NONSL.so libfigureSL240NONSLNODMPassThrough.so libfigureSL240SOCKET.so summer
include ../Makefile.config

#OPTS=-g
OPTS=

SINC=../include
install:
	cp darcmain $(BIN)
	cp utilsmodule.so $(PY)
	cp libreconmvm.so $(LIB)
	cp libreconAsync.so $(LIB)
	cp libmirrorSocket.so $(LIB)
	cp libmirrorSHM.so $(LIB)
	cp libcamfile.so $(LIB)
	cp libreconKalman.so $(LIB)
	cp librtccalibrate.so $(LIB)
	cp librtcbuffer.so $(LIB)
	cp librtcslope.so $(LIB)
	cp libfigure.so $(LIB)
	cp libmirrorNoSL240.so $(LIB)
	cp libmirror.so $(LIB)
	cp libsl240Int32camNoCam.so $(LIB)
	cp libcentroider.so $(LIB)
	cp libcamera.so $(LIB)
	cp libfigureSL240NONSL.so $(LIB)
	cp libfigureSL240SOCKET.so $(LIB)
	cp sender $(BIN)
	cp summer $(BIN)
	cp libcamsocket.so $(LIB)
	cp agbcblas.c $(SRC)
	cp andor.c $(SRC)
	cp camera.c $(SRC)
	cp camfile.c $(SRC)
	cp camsocket.c $(SRC)
	cp centroider.c $(SRC)
	cp circ.c $(SRC)
	cp darccore.c $(SRC)
	cp darcmain.c $(SRC)
	cp dmcPdAO32mirror.c $(SRC)
	cp dmcSL240mirror.c $(SRC)
	cp dmcSocketMirror.c $(SRC)
	cp figure.c $(SRC)
	cp figureSL240.c $(SRC)
	cp figureSL240PassThrough.c $(SRC)
	cp figureSL240SC.c $(SRC)
	cp figureSL240SCPassThrough.c $(SRC)
	cp jaicam.cpp $(SRC)
	cp mirror.c $(SRC)
	cp mirrorPdAO32.c $(SRC)
	cp mirrorSL240.c $(SRC)
	cp mirrorTest.c $(SRC)
	cp nslRecv.c $(SRC)
	cp nslSendData.c $(SRC)
	cp reconKalman.c $(SRC)
	cp reconmvm.c $(SRC)
	cp reconmvmcuda.c $(SRC)
	cp senddata.c $(SRC)
	cp sender.c $(SRC)
	cp summer.c $(SRC)
	cp utils.c $(SRC)
	cp sl240cam.c $(SRC)
	cp sl240centroider.c $(SRC)
	cp sl240Int32cam.c $(SRC)
	cp test.c $(SRC)
	cp testclip.c $(SRC)
	cp testpthread.c $(SRC)
	cp tmpsubsupap.c $(SRC)
	cp Makefile $(SRC)
	cp reconAsync.c $(SRC)

installdev:
	ln -sf $(PWD)/darcmain $(PWD)/../bin
	ln -sf $(PWD)/utilsmodule.so $(PWD)/../lib/python
	ln -sf $(PWD)/libreconmvm.so $(PWD)/../lib
	ln -sf $(PWD)/libreconAsync.so $(PWD)/../lib
	ln -sf $(PWD)/libmirrorSocket.so $(PWD)/../lib
	ln -sf $(PWD)/libmirrorSHM.so $(PWD)/../lib
	ln -sf $(PWD)/libcamfile.so $(PWD)/../lib
	ln -sf $(PWD)/libreconKalman.so $(PWD)/../lib
	ln -sf $(PWD)/sender $(PWD)/../bin
	ln -sf $(PWD)/summer $(PWD)/../bin
	ln -sf $(PWD)/libcamsocket.so $(PWD)/../lib
	ln -sf $(PWD)/librtccalibrate.so $(PWD)/../lib
	ln -sf $(PWD)/librtcbuffer.so $(PWD)/../lib
	ln -sf $(PWD)/librtcslope.so $(PWD)/../lib
	ln -sf $(PWD)/libfigure.so $(PWD)/../lib
	ln -sf $(PWD)/libmirrorNoSL240.so $(PWD)/../lib
	ln -sf $(PWD)/libmirror.so $(PWD)/../lib
	ln -sf $(PWD)/libcentroider.so $(PWD)/../lib
	ln -sf $(PWD)/libcamera.so $(PWD)/../lib
	ln -sf $(PWD)/libsl240Int32camNoCam.so $(PWD)/../lib
	ln -sf $(PWD)/libfigureSL240NONSL.so $(PWD)/../lib
	ln -sf $(PWD)/libfigureSL240SOCKET.so $(PWD)/../lib
clean:
	rm *.o
	rm *.so
	rm darcmain

agbcblas.o: $(SINC)/agbcblas.h agbcblas.c
	gcc -Wall -O3 -c -I../include -o agbcblas.o agbcblas.c  -funroll-loops -msse2 -mfpmath=sse -march=native

darcmaingsl: darccore.c darcmain.c circ.o $(SINC)/darcNames.h $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -pthread -rdynamic -O3 -DUSEGSL -Wall -I../include -I/usr/local/include circ.o -L/usr/local/lib -L/usr/lib64 -lgslcblas -lpthread -lfftw3f -lm -lrt -ldl darcmain.c -o darcmain
	echo USING GSL
darcmain: darcmain.c darccore.o circ.o buffer.o $(SINC)/darcNames.h $(SINC)/darc.h agbcblas.o $(SINC)/arrayStruct.h $(SINC)/circ.h $(SINC)/buffer.h
	gcc -pthread -rdynamic -O3 -DUSEAGBBLAS -Wall -I../include -I/usr/local/include darccore.o circ.o agbcblas.o buffer.o -L/usr/local/lib -L/usr/lib64 -lpthread -lfftw3f -lm -lrt -ldl darcmain.c -o darcmain
	echo USING AGB BLAS

darccore.o: darccore.c $(SINC)/darcNames.h $(SINC)/darc.h $(SINC)/arrayStruct.h $(SINC)/circ.h $(SINC)/buffer.h $(SINC)/agbcblas.h $(SINC)/rtccamera.h $(SINC)/rtcmirror.h $(SINC)/rtcrecon.h $(SINC)/qsort.h $(SINC)/rtccalibrate.h $(SINC)/rtcslope.h $(SINC)/rtcfigure.h $(SINC)/rtcbuffer.h
	gcc -Wall -DUSEAGBBLAS -O3 -c -I../include -o darccore.o darccore.c
	echo "USING AGB BLAS"
	echo "Note - the dereferencing type-punned pointer warnings are due to an incompatibility between posix and unix standards, and can safely be ignored.  The could be switched off with -fno-strict-aliasing, but this may also disable other warnings that you may wish to catch."

darcdebug: darccore.c darcmain.c circ.o $(SINC)/darcNames.h $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc $(OPTS) -pthread -rdynamic -O3 -DUSEGSL -Wall -I../include -I/usr/local/include circ.o -L/usr/local/lib -L/usr/lib64 -lgslcblas -lpthread -lfftw3f -lm -lrt -ldl darcmain.c -o darcmain
	echo USING GSL

libreconmvm.so: reconmvm.c $(SINC)/darc.h $(SINC)/darcNames.h agbcblas.o $(SINC)/arrayStruct.h $(SINC)/buffer.h buffer.o 
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DUSEAGBBLAS -fPIC -O3 -I../include $(OPTS) -c -Wall -o reconmvm.o reconmvm.c
	gcc -O3 -shared -Wl,-soname,libreconmvm.so.1 -o libreconmvm.so.1.0.1 reconmvm.o agbcblas.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libreconmvm.so
	ln -s  libreconmvm.so.1 libreconmvm.so
libreconAsync.so: reconAsync.c $(SINC)/darc.h $(SINC)/darcNames.h agbcblas.o $(SINC)/arrayStruct.h $(SINC)/buffer.h buffer.o 
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DUSEAGBBLAS -fPIC -O3 -I../include $(OPTS) -c -Wall -o reconAsync.o reconAsync.c
	gcc -O3 -shared -Wl,-soname,libreconAsync.so.1 -o libreconAsync.so.1.0.1 reconAsync.o agbcblas.o -lpthread -lrt -lc 
	/sbin/ldconfig -n ./
	rm -f libreconAsync.so
	ln -s  libreconAsync.so.1 libreconAsync.so
libreconmvmgsl.so: reconmvm.c $(SINC)/darc.h $(SINC)/darcNames.h $(SINC)/arrayStruct.h  $(SINC)/buffer.h buffer.o 
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -I../include $(OPTS) -c -Wall -o reconmvmgsl.o reconmvm.c
	gcc -O3 -shared -Wl,-soname,libreconmvmgsl.so.1 -o libreconmvmgsl.so.1.0.1 reconmvmgsl.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libreconmvmgsl.so
	ln -s  libreconmvmgsl.so.1 libreconmvmgsl.so

libreconmvmcuda.so: reconmvm.c $(SINC)/darc.h $(SINC)/darcNames.h $(SINC)/arrayStruct.h  $(SINC)/buffer.h buffer.o 
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DUSECUBLAS -I../include -fPIC -O3 $(OPTS) -c -Wall -o reconmvmcuda.o reconmvm.c -I/usr/local/cuda/include
	gcc -O3 -shared -Wl,-soname,libreconmvmcuda.so.1 -o libreconmvmcuda.so.1.0.1 reconmvmcuda.o -lpthread -lc -L/usr/local/cuda/lib -L/usr/local/cuda/lib64 -lcublas
	/sbin/ldconfig -n ./
	rm -f libreconmvmcuda.so
	ln -s  libreconmvmcuda.so.1 libreconmvmcuda.so

libreconKalman.so: reconKalman.c $(SINC)/darc.h $(SINC)/darcNames.h agbcblas.o $(SINC)/arrayStruct.h  $(SINC)/buffer.h buffer.o 
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DUSEAGBBLAS -I../include -fPIC -O3 $(OPTS) -c -Wall -o reconKalman.o reconKalman.c
	gcc -O3 -shared -Wl,-soname,libreconKalman.so.1 -o libreconKalman.so.1.0.1 reconKalman.o agbcblas.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libreconKalman.so
	ln -s  libreconKalman.so.1 libreconKalman.so
libreconKalman.so_gsl: reconKalman.c $(SINC)/darc.h $(SINC)/darcNames.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 $(OPTS) -c -I../include -Wall -o reconKalman.o reconKalman.c
	gcc -O3 -shared -Wl,-soname,libreconKalman.so.1 -o libreconKalman.so.1.0.1 reconKalman.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libreconKalman.so
	ln -s  libreconKalman.so.1 libreconKalman.so


buffer.o: buffer.c $(SINC)/buffer.h
	gcc -Wall $(OPTS) -O3 -I../include -c buffer.c -o buffer.o

circ.o: circ.c $(SINC)/circ.h
	gcc $(OPTS) -Wall -O3 -I../include -c circ.c -o circ.o -DUSEGSL

libcamera.so: camera.c $(SINC)/rtccamera.h
	gcc -fPIC $(OPTS) -c -Wall -I../include -o camera.o camera.c
	gcc -shared -Wl,-soname,libcamera.so.1 -o libcamera.so.1.0.1 camera.o -lc
	/sbin/ldconfig -n ./
	rm -f libcamera.so
	ln -s  libcamera.so.1 libcamera.so
libcentroider.so: centroider.c $(SINC)/rtcslope.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -fPIC $(OPTS) -c -Wall -I../include -o centroider.o centroider.c
	gcc -shared -Wl,-soname,libcentroider.so.1 -o libcentroider.so.1.0.1 centroider.o -lc
	/sbin/ldconfig -n ./
	rm -f libcentroider.so
	ln -s  libcentroider.so.1 libcentroider.so

librtccalibrate.so: rtccalibrate.c $(SINC)/rtccalibrate.h circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -fPIC $(OPTS) -c -Wall -O3 -I../include -o rtccalibrate.o rtccalibrate.c
	gcc -shared -Wl,-soname,librtccalibrate.so.1 -o librtccalibrate.so.1.0.1 rtccalibrate.o -lc
	/sbin/ldconfig -n ./
	rm -f librtccalibrate.so
	ln -s librtccalibrate.so.1 librtccalibrate.so
librtcbuffer.so: rtcbuffer.c $(SINC)/rtcbuffer.h circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -fPIC $(OPTS) -c -Wall -O3 -I../include -o rtcbuffer.o rtcbuffer.c
	gcc -shared -Wl,-soname,librtcbuffer.so.1 -o librtcbuffer.so.1.0.1 rtcbuffer.o -lc
	/sbin/ldconfig -n ./
	rm -f librtcbuffer.so
	ln -s librtcbuffer.so.1 librtcbuffer.so

librtcslope.so: rtcslope.c $(SINC)/rtcslope.h circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -fPIC $(OPTS) -c -Wall -O3 -I../include -o rtcslope.o rtcslope.c
	gcc -shared -Wl,-soname,librtcslope.so.1 -o librtcslope.so.1.0.1 rtcslope.o -lc -lm
	/sbin/ldconfig -n ./
	rm -f librtcslope.so
	ln -s librtcslope.so.1 librtcslope.so
libcamfile.so: camfile.c $(SINC)/rtccamera.h circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -fPIC -I../include -O3 $(OPTS) -c -Wall -o camfile.o camfile.c
	gcc -O3 -shared -Wl,-soname,libcamfile.so.1 -o libcamfile.so.1.0.1 camfile.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libcamfile.so
	ln -s  libcamfile.so.1 libcamfile.so

libcamsocket.so: camsocket.c $(SINC)/rtccamera.h circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -fPIC -O3 -I../include $(OPTS) -c -Wall -o camsocket.o camsocket.c
	gcc -O3 -shared -Wl,-soname,libcamsocket.so.1 -o libcamsocket.so.1.0.1 camsocket.o -lc -lm
	/sbin/ldconfig -n ./
	rm -f libcamsocket.so
	ln -s  libcamsocket.so.1 libcamsocket.so

libsl240Int32cam.so: sl240Int32cam.c $(SINC)/rtccamera.h circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -I../include -O3 $(OPTS) -c -Wall -o sl240Int32cam.o sl240Int32cam.c -I/opt/sl240/nsl/inc -I/opt/nsl/inc
	gcc -O3 -shared -Wl,-soname,libsl240Int32cam.so.1 -o libsl240Int32cam.so.1.0.1 sl240Int32cam.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi
	/sbin/ldconfig -n ./
	rm -f libsl240Int32cam.so
	ln -s  libsl240Int32cam.so.1 libsl240Int32cam.so
	echo "Remember to install (copy to /rtc/lib/)"
#this one is for testing compilation only...
libsl240Int32camNoCam.so: sl240Int32cam.c $(SINC)/rtccamera.h circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DNOSL240 -DPLATFORM_UNIX -fPIC -I../include -O3 $(OPTS) -c -Wall -o sl240Int32camNoCam.o sl240Int32cam.c -I/opt/sl240/nsl/inc -I/opt/nsl/inc
	gcc -O3 -shared -Wl,-soname,libsl240Int32camNoCam.so.1 -o libsl240Int32camNoCam.so.1.0.1 sl240Int32camNoCam.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib
	/sbin/ldconfig -n ./
	rm -f libsl240Int32camNoCam.so
	ln -s  libsl240Int32camNoCam.so.1 libsl240Int32camNoCam.so

libsl240centroider.so: sl240centroider.c circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 $(OPTS) -c -Wall -o sl240centroider.o sl240centroider.c -I/opt/sl240/nsl/inc -I/opt/nsl/inc 
	gcc -O3 -shared -Wl,-soname,libsl240centroider.so.1 -o libsl240centroider.so.1.0.1 sl240centroider.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi
	/sbin/ldconfig -n ./
	rm -f libsl240centroider.so
	ln -s  libsl240centroider.so.1 libsl240centroider.so
libnosl240centroider.so: sl240centroider.c circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DNOSL240 -fPIC -I../include -O3 $(OPTS) -c -Wall -o nosl240centroider.o sl240centroider.c
	gcc -O3 -shared -Wl,-soname,libnosl240centroider.so.1 -o libnosl240centroider.so.1.0.1 nosl240centroider.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libnosl240centroider.so
	ln -s  libnosl240centroider.so.1 libnosl240centroider.so

libmirror.so: mirror.c $(SINC)/rtcmirror.h $(SINC)/circ.h circ.o buffer.o $(SINC)/buffer.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -fPIC -O3 $(OPTS) -c -Wall -I../include -o mirror.o mirror.c
	gcc -O3 -shared -Wl,-soname,libmirror.so.1 -o libmirror.so.1.0.1 mirror.o -lc
	/sbin/ldconfig -n ./
	rm -f libmirror.so
	ln -s  libmirror.so.1 libmirror.so
libmirrorSL240.so: mirrorSL240.c circ.o $(SINC)/rtcmirror.h $(SINC)/darc.h $(SINC)/circ.h $(SINC)/arrayStruct.h buffer.o $(SINC)/buffer.h   $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -I../include -fPIC -O3 $(OPTS) -c -Wall -o mirrorSL240.o mirrorSL240.c -I/opt/sl240/nsl/inc -I/opt/nsl/inc
	gcc -O3 -shared -Wl,-soname,libmirrorSL240.so.1 -o libmirrorSL240.so.1.0.1 mirrorSL240.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi
	/sbin/ldconfig -n ./
	rm -f libmirrorSL240.so
	ln -s  libmirrorSL240.so.1 libmirrorSL240.so
libmirrorNoSL240.so: mirrorSL240.c circ.o $(SINC)/rtcmirror.h $(SINC)/darc.h $(SINC)/circ.h $(SINC)/arrayStruct.h  buffer.o $(SINC)/buffer.h 
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DNOSL240 -fPIC -O3 $(OPTS) -c -I../include -Wall -o mirrorNoSL240.o mirrorSL240.c 
	gcc -O3 -shared -Wl,-soname,libmirrorNoSL240.so.1 -o libmirrorNoSL240.so.1.0.1 mirrorNoSL240.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libmirrorNoSL240.so
	ln -s  libmirrorNoSL240.so.1 libmirrorNoSL240.so
libmirrorSocket.so: mirrorSocket.c circ.o $(SINC)/rtcmirror.h $(SINC)/darc.h $(SINC)/circ.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 $(OPTS) -c -Wall -I../include -o mirrorSocket.o mirrorSocket.c
	gcc -O3 -shared -Wl,-soname,libmirrorSocket.so.1 -o libmirrorSocket.so.1.0.1 mirrorSocket.o -lpthread -lc
	/sbin/ldconfig -n ./
	rm -f libmirrorSocket.so
	ln -s  libmirrorSocket.so.1 libmirrorSocket.so

libmirrorSHM.so: mirrorSHM.c circ.o $(SINC)/rtcmirror.h $(SINC)/darc.h $(SINC)/circ.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 $(OPTS) -c -Wall -I../include -o mirrorSHM.o mirrorSHM.c
	gcc -O3 -shared -Wl,-soname,libmirrorSHM.so.1 -o libmirrorSHM.so.1.0.1 mirrorSHM.o -lpthread -lrt -lc
	/sbin/ldconfig -n ./
	rm -f libmirrorSHM.so
	ln -s  libmirrorSHM.so.1 libmirrorSHM.so


libfigure.so: figure.c $(SINC)/rtcfigure.h $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -fPIC -O3 $(OPTS) -c -Wall -I../include -o figure.o figure.c
	gcc -O3 -shared -Wl,-soname,libfigure.so.1 -o libfigure.so.1.0.1 figure.o -lc
	/sbin/ldconfig -n ./
	rm -f libfigure.so
	ln -s  libfigure.so.1 libfigure.so

libfigureSL240.so: figureSL240.c $(SINC)/rtcfigure.h $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -I../include -fPIC -O3 $(OPTS) -c -Wall -o figureSL240.o figureSL240.c -I/opt/nsl/inc
	gcc -O3 -shared -Wl,-soname,libfigureSL240.so.1 -o libfigureSL240.so.1.0.1 figureSL240.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi
	/sbin/ldconfig -n ./
	rm -f libfigureSL240.so
	ln -s  libfigureSL240.so.1 libfigureSL240.so

libfigureSL240PassThrough.so: figureSL240PassThrough.c $(SINC)/rtcfigure.h $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 $(OPTS) -c -Wall -I../include -o figureSL240PassThrough.o figureSL240PassThrough.c -I/opt/nsl/inc -I/Canary/src/dmc/powerdaq-3.6.20/include
	gcc -O3 -shared -Wl,-soname,libfigureSL240PassThrough.so.1 -o libfigureSL240PassThrough.so.1.0.1 figureSL240PassThrough.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi -lc -lpowerdaq32
	/sbin/ldconfig -n ./
	rm -f libfigureSL240PassThrough.so
	ln -s  libfigureSL240PassThrough.so.1 libfigureSL240PassThrough.so
libfigureSL240SCPassThrough.so: figureSL240SCPassThrough.c $(SINC)/rtcfigure.h circ.o $(SINC)/darc.h $(SINC)/circ.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 $(OPTS) -c -Wall -I../include -o figureSL240SCPassThrough.o figureSL240SCPassThrough.c -I/Canary/src/SL240/sl240/inc -I/Canary/src/dmc/powerdaq-3.6.20/include
	gcc -O3 -shared -Wl,-soname,libfigureSL240SCPassThrough.so.1 -o libfigureSL240SCPassThrough.so.1.0.1 figureSL240SCPassThrough.o -lpthread -lc -L/Canary/src/SL240/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -L/Canary/src/SL240/sl240/bin -lfxsl -lc -lpowerdaq32
	/sbin/ldconfig -n ./
	rm -f libfigureSL240SCPassThrough.so
	ln -s  libfigureSL240SCPassThrough.so.1 libfigureSL240SCPassThrough.so


libfigureSL240NSLPassThrough.so: figureSL240SCPassThrough.c $(SINC)/rtcfigure.h circ.o $(SINC)/darc.h $(SINC)/circ.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DNSL -fPIC -O3 $(OPTS) -c -Wall -I../include -o figureSL240NSLPassThrough.o figureSL240SCPassThrough.c -I/Canary/src/SL240/sl240/inc -I/Canary/src/dmc/powerdaq-3.6.20/include
	gcc -O3 -shared -Wl,-soname,libfigureSL240NSLPassThrough.so.1 -o libfigureSL240NSLPassThrough.so.1.0.1 figureSL240NSLPassThrough.o -lpthread -lc -L/Canary/src/SL240/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -L/Canary/src/SL240/sl240/bin -lfxsl -lc -lpowerdaq32
	/sbin/ldconfig -n ./
	rm -f libfigureSL240NSLPassThrough.so
	ln -s  libfigureSL240NSLPassThrough.so.1 libfigureSL240NSLPassThrough.so

libfigureSL240NONSLPassThrough.so: figureSL240SCPassThrough.c $(SINC)/rtcfigure.h circ.o $(SINC)/darc.h $(SINC)/circ.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -DNONSL $(OPTS) -c -Wall -I../include -o figureSL240NONSLPassThrough.o figureSL240SCPassThrough.c -I/Canary/src/SL240/sl240/inc -I/Canary/src/dmc/powerdaq-3.6.20/include
	gcc -O3 -shared -Wl,-soname,libfigureSL240NONSLPassThrough.so.1 -o libfigureSL240NONSLPassThrough.so.1.0.1 figureSL240NONSLPassThrough.o -lpthread -lc -L/Canary/src/SL240/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -L/Canary/src/SL240/sl240/bin -lfxsl -lc -lpowerdaq32
	/sbin/ldconfig -n ./
	rm -f libfigureSL240NONSLPassThrough.so
	ln -s  libfigureSL240NONSLPassThrough.so.1 libfigureSL240NONSLPassThrough.so

libfigureSL240NONSLNODMPassThrough.so: figureSL240SCPassThrough.c $(SINC)/rtcfigure.h circ.o $(SINC)/darc.h $(SINC)/circ.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -DNONSL -DNODM -O3 $(OPTS) -c -Wall -I../include -o figureSL240NONSLNODMPassThrough.o figureSL240SCPassThrough.c
	gcc -O3 -shared -Wl,-soname,libfigureSL240NONSLNODMPassThrough.so.1 -o libfigureSL240NONSLNODMPassThrough.so.1.0.1 figureSL240NONSLNODMPassThrough.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libfigureSL240NONSLNODMPassThrough.so
	ln -s  libfigureSL240NONSLNODMPassThrough.so.1 libfigureSL240NONSLNODMPassThrough.so


libfigureSocketPassThru.so: figureSocketPassThru.c $(SINC)/rtcfigure.h circ.o $(SINC)/darc.h $(SINC)/circ.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 $(OPTS) -c -Wall -o figureSocketPassThru.o figureSocketPassThru.c -I../include -I/Canary/src/dmc/powerdaq-3.6.20/include
	gcc -O3 -shared -Wl,-soname,libfigureSocketPassThru.so.1 -o libfigureSocketPassThru.so.1.0.1 figureSocketPassThru.o -lpthread -lc -lpowerdaq32
	/sbin/ldconfig -n ./
	rm -f libfigureSocketPassThru.so
	ln -s  libfigureSocketPassThru.so.1 libfigureSocketPassThru.so

libfigureSL240SC.so: figureSL240SC.c $(SINC)/rtcfigure.h $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -I../include -O3 $(OPTS) -c -Wall -o figureSL240SC.o figureSL240SC.c -I/Canary/src/SL240/sl240/inc 
	gcc -O3 -shared -Wl,-soname,libfigureSL240SC.so.1 -o libfigureSL240SC.so.1.0.1 figureSL240SC.o -lpthread -lc -L/Canary/src/SL240/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -L/Canary/src/SL240/sl240/bin -lfxsl -lc
	/sbin/ldconfig -n ./
	rm -f libfigureSL240SC.so
	ln -s  libfigureSL240SC.so.1 libfigureSL240SC.so

libfigureSL240NSL.so: figureSL240SC.c $(SINC)/rtcfigure.h $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DNSL -fPIC -I../include -O3 $(OPTS) -c -Wall -o figureSL240NSL.o figureSL240SC.c -I/Canary/src/SL240/sl240/inc 
	gcc -O3 -shared -Wl,-soname,libfigureSL240NSL.so.1 -o libfigureSL240NSL.so.1.0.1 figureSL240NSL.o -lpthread -lc -L/Canary/src/SL240/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -L/Canary/src/SL240/sl240/bin -lfxsl -lc
	/sbin/ldconfig -n ./
	rm -f libfigureSL240NSL.so
	ln -s  libfigureSL240NSL.so.1 libfigureSL240NSL.so
libfigureSL240NONSL.so: figureSL240SC.c $(SINC)/rtcfigure.h $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DNONSL -fPIC -I../include -O3 $(OPTS) -c -Wall -o figureSL240NONSL.o figureSL240SC.c
	gcc -O3 -shared -Wl,-soname,libfigureSL240NONSL.so.1 -o libfigureSL240NONSL.so.1.0.1 figureSL240NONSL.o -lpthread -lc -lc
	/sbin/ldconfig -n ./
	rm -f libfigureSL240NONSL.so
	ln -s  libfigureSL240NONSL.so.1 libfigureSL240NONSL.so

libfigureSL240SOCKET.so: figureSL240SC.c $(SINC)/rtcfigure.h $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DSOCKET -fPIC -I../include -O3 $(OPTS) -c -Wall -o figureSL240SOCKET.o figureSL240SC.c
	gcc -O3 -shared -Wl,-soname,libfigureSL240SOCKET.so.1 -o libfigureSL240SOCKET.so.1.0.1 figureSL240SOCKET.o -lpthread -lc -lc
	/sbin/ldconfig -n ./
	rm -f libfigureSL240SOCKET.so
	ln -s  libfigureSL240SOCKET.so.1 libfigureSL240SOCKET.so

nslSendData: nslSendData.c
	gcc -o nslSendData -DPLATFORM_UNIX  -I/opt/nsl/inc -I/opt/sl240/nsl/inc nslSendData.c -L/opt/nsl/linux-2.6/lib -L/opt/sl240/nsl/linux-2.6/lib -lnslapi
nslRecv: nslRecv.c
	gcc -o nslRecv -DPLATFORM_UNIX -I/opt/sl240/nsl/inc -I/opt/nsl/inc nslRecv.c -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi
utilsmodule.so: utils.c
	python setup.py build
	python setup.py install --install-lib=.
libmirrorPdAO32.so: mirrorPdAO32.c $(SINC)/rtcmirror.h $(SINC)/circ.h circ.o $(SINC)/darc.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -fPIC -O3 $(OPTS) -c -Wall -I../include -I/Canary/src/dmc/powerdaq-3.6.20/include -o mirrorPdAO32.o mirrorPdAO32.c
	gcc -O3 -shared -Wl,-soname,libmirrorPdAO32.so.1 -o libmirrorPdAO32.so.1.0.1 mirrorPdAO32.o -lc -lpowerdaq32
	/sbin/ldconfig -n ./
	rm -f libmirrorPdAO32.so
	ln -s  libmirrorPdAO32.so.1 libmirrorPdAO32.so
libjaicam.so: jaicam.cpp $(SINC)/rtccamera.h circ.o $(SINC)/darc.h $(SINC)/circ.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	g++ -Wall $(OPTS) -I../include   -c -o jaicam.o jaicam.cpp
	g++ -Wall $(OPTS)  -fPIC -shared -Wl,-soname,libjaicam.so.1 -o libjaicam.so.1.0.1 jaicam.o -lJAIFactory
	#gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 $(OPTS) -c -Wall -I/usr/include/JAI -o jaicam.o jaicam.c 
	#gcc -O3 -shared -Wl,-soname,librtccamera.so.1 -o librtccamera.so.1.0.1 jaicam.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libjaicam.so
	ln -s  libjaicam.so.1 libjaicam.so
sender: sender.c circ.o $(SINC)/circ.h
	gcc -O3 -o sender -I../include sender.c circ.o -lrt -Wall
summer: summer.c circ.o $(SINC)/circ.h
	gcc -O3 -o summer -I../include summer.c circ.o -lrt -Wall
receiver: receiver.c circ.o $(SINC)/circ.h
	gcc -O3- o receiver -I../include receiver.c circ.o -lpthread -lrt -Wall

gpucalibrate: gpucalibrate.cu
	/usr/local/cuda/bin/nvcc -gencode=arch=compute_10,code=\"sm_10,compute_10\"  -gencode=arch=compute_20,code=\"sm_20,compute_20\"  -m64 --compiler-options -fno-strict-aliasing  -I. -I/usr/local/cuda/include -DUNIX -O2   -o gpucalibrate.o -c gpucalibrate.cu
	gcc -fPIC -m64 -o gpucalibrate gpucalibrate.o -L/usr/local/cuda/lib64 -lcudart -L/usr/local/cuda/lib64 -L./ -lcuda 

#-fPIC -W -Wall -Wimplicit -Wswitch -Wformat -Wchar-subscripts -Wparentheses -Wmultichar -Wtrigraphs -Wpointer-arith -Wcast-align -Wreturn-type -Wno-unused-function   -m64 -fno-strict-aliasing -I. -I/usr/local/cuda/include -DUNIX -O2  -o gpucalibrate.o -c gpucalibrate.c
