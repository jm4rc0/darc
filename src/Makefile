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

all: utilsmodule.so libreconmvm.so libcamfile.so libreconKalman.so sender libcamsocket.so librtccalibrate.so librtccalibrateSim.so librtcslope.so librtcbuffer.so libmirrorSocket.so libmirrorSoundcard.so libreconAsync.so libmirrorLLS.so libcamera.so libcentroider.so libsl240Int32camNoCam.so libmirror.so libmirrorNoSL240.so libfigure.so libnosl240centroider.so libmirrorSHM.so libfigureSL240NONSL.so libfigureSL240NONSLNODMPassThrough.so libfigureSL240SOCKET.so libfigureSocketPassThruNODM.so libreconpcg.so libcamudp.so libreconLQG.so libreconneural.so summer splitter receiver leakyaverage darcmain Makefilelibs libraries userArray.o libmirrorPdAO32NODM.so libmirrorPdAO32ManyNODM.so libmirrorPdAO32SocketNODM.so libmirrorAlpaoSdkNODM.so libmirrorPdAO32AlpaoNODM.so darccontrolc

#Makefilelibs libraries

#libraries:
#	echo "Makefile.libs not yet made - rerun make"
#librariesInstall:
#	echo "Makefile.libs not yet made - rerun make"
#librariesInstalldev:
#	echo "Makefile.libs not yet made - rerun make"
include ../Makefile.config
-include ../Makefile.config.local
include Makefile.libs

#OPTS=-g
#OPTS=-DOLDMULTINEWFN
#OPTS=-pg  # for profiling with gprof
#OPTS=-march=native # better performance... - currently experimental.
OPTS=

SINC=../include
install: librariesInstall
	cp -f darcmain $(BIN)
	cp utilsmodule.so $(PY)
	cp libreconmvm.so $(LIB)
	cp libreconneural.so $(LIB)
	cp libreconpcg.so $(LIB)
	cp libreconAsync.so $(LIB)
	cp libmirrorLLS.so $(LIB)
	cp libmirrorSocket.so $(LIB)
	cp libmirrorSoundcard.so $(LIB)
	cp libmirrorSHM.so $(LIB)
	cp libcamfile.so $(LIB)
	cp libreconKalman.so $(LIB)
	cp libreconLQG.so $(LIB)
	cp librtccalibrate.so $(LIB)
	cp librtccalibrateSim.so $(LIB)
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
	cp libfigureSocketPassThruNODM.so $(LIB)
	cp libmirrorPdAO32NODM.so $(LIB)
	cp libmirrorPdAO32ManyNODM.so $(LIB)
	cp libmirrorPdAO32SocketNODM.so $(LIB)
	cp libmirrorAlpaoSdkNODM.so $(LIB)
	cp libmirrorPdAO32AlpaoNODM.so $(LIB)
	cp libcamudp.so $(LIB)
	cp -f sender $(BIN)
	cp -f summer $(BIN)
	cp -f splitter $(BIN)
	cp -f receiver $(BIN)
	cp -f leakyaverage $(BIN)
	cp libcamsocket.so $(LIB)
	cp agbcblas.c $(SRC)
	cp agbcblas.o $(LIB)
	cp andor.c $(SRC)
	cp buffer.c $(SRC)
	cp buffer.o $(LIB)
	cp camera.c $(SRC)
	cp camfile.c $(SRC)
	cp camv4l.c $(SRC)
	cp camsocket.c $(SRC)
	cp centroider.c $(SRC)
	cp circ.c $(SRC)
	cp circ.o $(LIB)
	cp darccore.c $(SRC)
	cp darcmain.c $(SRC)
	cp dmcPdAO32mirror.c $(SRC)
	cp dmcSL240mirror.c $(SRC)
	cp dmcSocketMirror.c $(SRC)
	cp figure.c $(SRC)
	cp figureSL240SC.c $(SRC)
	cp figureSL240SCPassThrough.c $(SRC)
	cp jaicam.cpp $(SRC)
	cp mirror.c $(SRC)
	cp mirrorPdAO32.c $(SRC)
	cp mirrorPdAO32Many.c $(SRC)
	cp mirrorSL240.c $(SRC)
	cp mirrorTest.c $(SRC)
	cp nslRecv.c $(SRC)
	cp nslSendData.c $(SRC)
	cp reconKalman.c $(SRC)
	cp reconLQG.c $(SRC)
	cp reconmvm.c $(SRC)
	cp reconpcg.c $(SRC)
	cp reconmvmcuda.c $(SRC)
	cp senddata.c $(SRC)
	cp sender.c $(SRC)
	cp summer.c $(SRC)
	cp splitter.c $(SRC)
	cp receiver.c $(SRC)
	cp leakyaverage.c $(SRC)
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
	cp mirrorLLS.c $(SRC)
	cp camuEyeUSB.c $(SRC)
	cp camuEyeUSBMany.c $(SRC)
	cp userArray.c $(SRC)
	cp darccontrolc $(BIN)
	cp darccontrolc.c $(SRC)
	echo "Remember to make and install manually any custom shared libraries"

#cp figureSL240.c $(SRC)
#cp figureSL240PassThrough.c $(SRC)

installdev: librariesInstalldev
	ln -sf $(PWD)/darcmain $(PWD)/../bin
	ln -sf $(PWD)/utilsmodule.so $(PWD)/../lib/python
	ln -sf $(PWD)/libreconmvm.so $(PWD)/../lib
	ln -sf $(PWD)/libreconneural.so $(PWD)/../lib
	ln -sf $(PWD)/libreconpcg.so $(PWD)/../lib
	ln -sf $(PWD)/libreconAsync.so $(PWD)/../lib
	ln -sf $(PWD)/libmirrorLLS.so $(PWD)/../lib
	ln -sf $(PWD)/libmirrorSocket.so $(PWD)/../lib
	ln -sf $(PWD)/libmirrorSoundcard.so $(PWD)/../lib
	ln -sf $(PWD)/libmirrorSHM.so $(PWD)/../lib
	ln -sf $(PWD)/libcamfile.so $(PWD)/../lib
	ln -sf $(PWD)/libreconKalman.so $(PWD)/../lib
	ln -sf $(PWD)/libreconLQG.so $(PWD)/../lib
	ln -sf $(PWD)/sender $(PWD)/../bin
	ln -sf $(PWD)/summer $(PWD)/../bin
	ln -sf $(PWD)/splitter $(PWD)/../bin
	ln -sf $(PWD)/receiver $(PWD)/../bin
	ln -sf $(PWD)/leakyaverage $(PWD)/../bin
	ln -sf $(PWD)/libcamsocket.so $(PWD)/../lib
	ln -sf $(PWD)/libcamudp.so $(PWD)/../lib
	ln -sf $(PWD)/librtccalibrate.so $(PWD)/../lib
	ln -sf $(PWD)/librtccalibrateSim.so $(PWD)/../lib
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
	ln -sf $(PWD)/libfigureSocketPassThruNODM.so $(PWD)/../lib
	ln -sf $(PWD)/libmirrorPdAO32NODM.so $(PWD)/../lib
	ln -sf $(PWD)/libmirrorPdAO32ManyNODM.so $(PWD)/../lib
	ln -sf $(PWD)/libmirrorPdAO32SocketNODM.so $(PWD)/../lib
	ln -sf $(PWD)/libmirrorAlpaoSdkNODM.so $(PWD)/../lib
	ln -sf $(PWD)/libmirrorPdAO32AlpaoNODM.so $(PWD)/../lib
	ln -sf $(PWD)/darccontrolc $(PWD)../bin
clean:
	rm *.o
	rm *.so
	rm darcmain
	rm darccontrolc

Makefilelibs:
	echo "Removing Makefile.libs"
	rm -f Makefile.libs.save
	mv Makefile.libs Makefile.libs.save

#Makes a makefile that can be used for library installation.
Makefile.libs:
	echo "Making Makefile.libs"
	echo libraries: `ls lib*.so` > Makefile.libs
	echo "	echo Made libraries" >> Makefile.libs
	echo librariesInstall: `ls lib*.so` >> Makefile.libs
	echo "	cp " `ls lib*.so` $(LIB) >> Makefile.libs
	echo librariesInstalldev: `ls lib*.so` >> Makefile.libs
	echo "	echo \"Please make softlinks manually of lib*.so into ../lib\"" >> Makefile.libs

userArray.o: $(SINC)/arrayStruct.h userArray.c
	gcc $(OPTS) -Wall -c -I$(SINC) -o userArray.o userArray.c -fPIC

agbcblas.o: $(SINC)/agbcblas.h agbcblas.c
	gcc $(OPTS) -Wall -O3 -c -I$(SINC) -o agbcblas.o agbcblas.c  -funroll-loops -msse2 -mfpmath=sse -march=native -fPIC

darcmaingsl: darccore.c darcmain.c circ.o $(SINC)/darcNames.h $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc $(OPTS) -pthread -rdynamic -O3 -DUSEGSL -Wall -I../include -I/usr/local/include circ.o -L/usr/local/lib -L/usr/lib64 -lgslcblas -lpthread -lfftw3f -lm -lrt -ldl darcmain.c -o darcmain
	echo USING GSL
darcmain: darcmain.c darccore.o circ.o buffer.o $(SINC)/darcNames.h $(SINC)/darc.h agbcblas.o $(SINC)/arrayStruct.h $(SINC)/circ.h $(SINC)/buffer.h
	gcc $(OPTS) -pthread -rdynamic -O3 -DUSEAGBBLAS -Wall -I../include -I/usr/local/include darccore.o circ.o agbcblas.o buffer.o -L/usr/local/lib -L/usr/lib64 -lpthread -lfftw3f -lm -lrt -ldl darcmain.c -o darcmain
	echo USING AGB BLAS

darccore.o: darccore.c $(SINC)/darcNames.h $(SINC)/darc.h $(SINC)/arrayStruct.h $(SINC)/circ.h $(SINC)/buffer.h $(SINC)/agbcblas.h $(SINC)/rtccamera.h $(SINC)/rtcmirror.h $(SINC)/rtcrecon.h $(SINC)/qsort.h $(SINC)/rtccalibrate.h $(SINC)/rtcslope.h $(SINC)/rtcfigure.h $(SINC)/rtcbuffer.h
	gcc $(OPTS) -Wall -DUSEAGBBLAS -O3 -c -I../include -o darccore.o darccore.c
	echo "USING AGB BLAS"
	echo "Note - the dereferencing type-punned pointer warnings are due to an incompatibility between posix and unix standards, and can safely be ignored.  The could be switched off with -fno-strict-aliasing, but this may also disable other warnings that you may wish to catch."

darcdebug: darccore.c darcmain.c circ.o $(SINC)/darcNames.h $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc $(OPTS) -pthread -rdynamic -O3 -DUSEGSL -Wall -I../include -I/usr/local/include circ.o -L/usr/local/lib -L/usr/lib64 -lgslcblas -lpthread -lfftw3f -lm -lrt -ldl darcmain.c -o darcmain
	echo USING GSL

darccontrolc: darccontrolc.c
	gcc $(OPTS) -pthread -O3 -Wall -I../include darccontrolc.c -o darccontrolc

libreconmvm.so: reconmvm.c $(SINC)/darc.h $(SINC)/darcNames.h agbcblas.o $(SINC)/arrayStruct.h $(SINC)/buffer.h buffer.o 
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DUSEAGBBLAS -fPIC -O3 -I../include $(OPTS) -c -Wall -o reconmvm.o reconmvm.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libreconmvm.so.1 -o libreconmvm.so.1.0.1 reconmvm.o agbcblas.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libreconmvm.so
	ln -s  libreconmvm.so.1 libreconmvm.so
libreconmvmDUMMY.so: reconmvm.c $(SINC)/darc.h $(SINC)/darcNames.h agbcblas.o $(SINC)/arrayStruct.h $(SINC)/buffer.h buffer.o 
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DUSEAGBBLAS -DDUMMY -fPIC -O3 -I../include $(OPTS) -c -Wall -o reconmvmDUMMY.o reconmvm.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libreconmvmDUMMY.so.1 -o libreconmvmDUMMY.so.1.0.1 reconmvmDUMMY.o agbcblas.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libreconmvmDUMMY.so
	ln -s  libreconmvmDUMMY.so.1 libreconmvmDUMMY.so
libreconAsync.so: reconAsync.c $(SINC)/darc.h $(SINC)/darcNames.h agbcblas.o $(SINC)/arrayStruct.h $(SINC)/buffer.h buffer.o 
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DUSEAGBBLAS -fPIC -O3 -I../include $(OPTS) -c -Wall -o reconAsync.o reconAsync.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libreconAsync.so.1 -o libreconAsync.so.1.0.1 reconAsync.o agbcblas.o -lpthread -lrt -lc 
	/sbin/ldconfig -n ./
	rm -f libreconAsync.so
	ln -s  libreconAsync.so.1 libreconAsync.so
libreconmvmgsl.so: reconmvm.c $(SINC)/darc.h $(SINC)/darcNames.h $(SINC)/arrayStruct.h  $(SINC)/buffer.h buffer.o 
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -I../include $(OPTS) -c -Wall -o reconmvmgsl.o reconmvm.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libreconmvmgsl.so.1 -o libreconmvmgsl.so.1.0.1 reconmvmgsl.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libreconmvmgsl.so
	ln -s  libreconmvmgsl.so.1 libreconmvmgsl.so

libreconmvmcuda.so: reconmvm.c $(SINC)/darc.h $(SINC)/darcNames.h agbcblas.o $(SINC)/arrayStruct.h  $(SINC)/buffer.h buffer.o 
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DUSECUDA -I../include -fPIC -O3 $(OPTS) -c -Wall -o reconmvmcuda.o reconmvm.c -I/usr/local/cuda/include
	gcc $(OPTS) -O3 -shared -Wl,-soname,libreconmvmcuda.so.1 -o libreconmvmcuda.so.1.0.1 reconmvmcuda.o -lpthread -lc -L/usr/local/cuda/lib -L/usr/local/cuda/lib64 -lcublas
	/sbin/ldconfig -n ./
	rm -f libreconmvmcuda.so
	ln -s  libreconmvmcuda.so.1 libreconmvmcuda.so

libreconKalman.so: reconKalman.c $(SINC)/darc.h $(SINC)/darcNames.h agbcblas.o $(SINC)/arrayStruct.h  $(SINC)/buffer.h buffer.o 
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DUSEAGBBLAS -I../include -fPIC -O3 $(OPTS) -c -Wall -o reconKalman.o reconKalman.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libreconKalman.so.1 -o libreconKalman.so.1.0.1 reconKalman.o agbcblas.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libreconKalman.so
	ln -s  libreconKalman.so.1 libreconKalman.so
libreconKalman.so_gsl: reconKalman.c $(SINC)/darc.h $(SINC)/darcNames.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 $(OPTS) -c -I../include -Wall -o reconKalman.o reconKalman.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libreconKalman.so.1 -o libreconKalman.so.1.0.1 reconKalman.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libreconKalman.so
	ln -s  libreconKalman.so.1 libreconKalman.so

libreconpcg.so: reconpcg.c $(SINC)/darc.h $(SINC)/darcNames.h agbcblas.o $(SINC)/arrayStruct.h $(SINC)/buffer.h buffer.o 
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DUSEAGBBLAS -fPIC -O3 -I../include $(OPTS) -c -Wall -o reconpcg.o reconpcg.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libreconpcg.so.1 -o libreconpcg.so.1.0.1 reconpcg.o agbcblas.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libreconpcg.so
	ln -s  libreconpcg.so.1 libreconpcg.so

buffer.o: buffer.c $(SINC)/buffer.h
	gcc -Wall $(OPTS) -O3 -I$(SINC) -c buffer.c -o buffer.o

circ.o: circ.c $(SINC)/circ.h
	gcc $(OPTS) -Wall -O3 -I$(SINC) -c circ.c -o circ.o -DUSEGSL

libcamera.so: camera.c $(SINC)/rtccamera.h
	gcc -fPIC $(OPTS) -c -Wall -I../include -o camera.o camera.c
	gcc $(OPTS) -shared -Wl,-soname,libcamera.so.1 -o libcamera.so.1.0.1 camera.o -lc
	/sbin/ldconfig -n ./
	rm -f libcamera.so
	ln -s  libcamera.so.1 libcamera.so
libcentroider.so: centroider.c $(SINC)/rtcslope.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -fPIC $(OPTS) -c -Wall -I../include -o centroider.o centroider.c
	gcc $(OPTS) -shared -Wl,-soname,libcentroider.so.1 -o libcentroider.so.1.0.1 centroider.o -lc
	/sbin/ldconfig -n ./
	rm -f libcentroider.so
	ln -s  libcentroider.so.1 libcentroider.so

librtccalibrate.so: rtccalibrate.c $(SINC)/rtccalibrate.h circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -fPIC $(OPTS) -c -Wall -O3 -I../include -o rtccalibrate.o rtccalibrate.c
	gcc $(OPTS) -shared -Wl,-soname,librtccalibrate.so.1 -o librtccalibrate.so.1.0.1 rtccalibrate.o -lc
	/sbin/ldconfig -n ./
	rm -f librtccalibrate.so
	ln -s librtccalibrate.so.1 librtccalibrate.so
librtccalibrateSim.so: rtccalibrate.c $(SINC)/rtccalibrate.h circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -fPIC $(OPTS) -D_GNU_SOURCE -DWITHSIM -c -Wall -O3 -I../include -o rtccalibrateSim.o rtccalibrate.c
	gcc $(OPTS) -shared -Wl,-soname,librtccalibrateSim.so.1 -o librtccalibrateSim.so.1.0.1 rtccalibrateSim.o -lc -lfftw3f -lgsl
	/sbin/ldconfig -n ./
	rm -f librtccalibrateSim.so
	ln -s librtccalibrateSim.so.1 librtccalibrateSim.so
librtccalibrateCombineTT.so: rtccalibrateCombineTT.c $(SINC)/rtccalibrate.h circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -fPIC $(OPTS) -c -Wall -O3 -I../include -o rtccalibrateCombineTT.o rtccalibrateCombineTT.c
	gcc $(OPTS) -shared -Wl,-soname,librtccalibrateCombineTT.so.1 -o librtccalibrateCombineTT.so.1.0.1 rtccalibrateCombineTT.o -lc
	/sbin/ldconfig -n ./
	rm -f librtccalibrateCombineTT.so
	ln -s librtccalibrateCombineTT.so.1 librtccalibrateCombineTT.so

librtcbuffer.so: rtcbuffer.c $(SINC)/rtcbuffer.h circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -fPIC $(OPTS) -c -Wall -O3 -I../include -o rtcbuffer.o rtcbuffer.c
	gcc $(OPTS) -shared -Wl,-soname,librtcbuffer.so.1 -o librtcbuffer.so.1.0.1 rtcbuffer.o -lc
	/sbin/ldconfig -n ./
	rm -f librtcbuffer.so
	ln -s librtcbuffer.so.1 librtcbuffer.so

librtcslope.so: rtcslope.c $(SINC)/rtcslope.h circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -fPIC $(OPTS) -c -Wall -O3 -I../include -o rtcslope.o rtcslope.c
	gcc $(OPTS) -shared -Wl,-soname,librtcslope.so.1 -o librtcslope.so.1.0.1 rtcslope.o -lc -lfftw3f -lm 
	/sbin/ldconfig -n ./
	rm -f librtcslope.so
	ln -s librtcslope.so.1 librtcslope.so
libcamfile.so: camfile.c $(SINC)/rtccamera.h circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -fPIC -I../include -O3 $(OPTS) -c -Wall -o camfile.o camfile.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libcamfile.so.1 -o libcamfile.so.1.0.1 camfile.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libcamfile.so
	ln -s  libcamfile.so.1 libcamfile.so

libcamsocket.so: camsocket.c $(SINC)/rtccamera.h circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -fPIC -O3 -I../include $(OPTS) -c -Wall -o camsocket.o camsocket.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libcamsocket.so.1 -o libcamsocket.so.1.0.1 camsocket.o -lc -lm
	/sbin/ldconfig -n ./
	rm -f libcamsocket.so
	ln -s  libcamsocket.so.1 libcamsocket.so

libsl240Int32cam.so: sl240Int32cam.c $(SINC)/rtccamera.h circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -I../include -O3 $(OPTS) -c -Wall -o sl240Int32cam.o sl240Int32cam.c -I/opt/nsl/inc
	gcc $(OPTS) -O3 -shared -Wl,-soname,libsl240Int32cam.so.1 -o libsl240Int32cam.so.1.0.1 sl240Int32cam.o -lpthread -lc -L/opt/nsl/linux-current/lib -lnslapi
	/sbin/ldconfig -n ./
	rm -f libsl240Int32cam.so
	ln -s  libsl240Int32cam.so.1 libsl240Int32cam.so
	echo "Remember to install (copy to /rtc/lib/ or /opt/darc/lib/)"

libsl240Int32camNOCRC.so: sl240Int32cam.c $(SINC)/rtccamera.h circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DNOCRCCHECK -DPLATFORM_UNIX -fPIC -I../include -O3 $(OPTS) -c -Wall -o sl240Int32camNOCRC.o sl240Int32cam.c -I/opt/nsl/inc
	gcc $(OPTS) -O3 -shared -Wl,-soname,libsl240Int32camNOCRC.so.1 -o libsl240Int32camNOCRC.so.1.0.1 sl240Int32camNOCRC.o -lpthread -lc -L/opt/nsl/linux-current/lib -lnslapi
	/sbin/ldconfig -n ./
	rm -f libsl240Int32camNOCRC.so
	ln -s  libsl240Int32camNOCRC.so.1 libsl240Int32camNOCRC.so
	echo "Remember to install (copy to /rtc/lib/ or /opt/darc/lib/)"

#this one is for testing compilation only...
libsl240Int32camNoCam.so: sl240Int32cam.c $(SINC)/rtccamera.h circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DNOSL240 -DPLATFORM_UNIX -fPIC -I../include -O3 $(OPTS) -c -Wall -o sl240Int32camNoCam.o sl240Int32cam.c -I/opt/nsl/inc
	gcc $(OPTS) -O3 -shared -Wl,-soname,libsl240Int32camNoCam.so.1 -o libsl240Int32camNoCam.so.1.0.1 sl240Int32camNoCam.o -lpthread -lc -L/opt/nsl/linux-current/lib
	/sbin/ldconfig -n ./
	rm -f libsl240Int32camNoCam.so
	ln -s  libsl240Int32camNoCam.so.1 libsl240Int32camNoCam.so

libsl240centroider.so: sl240centroider.c circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 $(OPTS) -c -Wall -o sl240centroider.o sl240centroider.c -I/opt/sl240/nsl/inc -I/opt/nsl/inc 
	gcc $(OPTS) -O3 -shared -Wl,-soname,libsl240centroider.so.1 -o libsl240centroider.so.1.0.1 sl240centroider.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi
	/sbin/ldconfig -n ./
	rm -f libsl240centroider.so
	ln -s  libsl240centroider.so.1 libsl240centroider.so
libnosl240centroider.so: sl240centroider.c circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DNOSL240 -fPIC -I../include -O3 $(OPTS) -c -Wall -o nosl240centroider.o sl240centroider.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libnosl240centroider.so.1 -o libnosl240centroider.so.1.0.1 nosl240centroider.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libnosl240centroider.so
	ln -s  libnosl240centroider.so.1 libnosl240centroider.so

libmirror.so: mirror.c $(SINC)/rtcmirror.h $(SINC)/circ.h circ.o buffer.o $(SINC)/buffer.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -fPIC -O3 $(OPTS) -c -Wall -I../include -o mirror.o mirror.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libmirror.so.1 -o libmirror.so.1.0.1 mirror.o -lc
	/sbin/ldconfig -n ./
	rm -f libmirror.so
	ln -s  libmirror.so.1 libmirror.so
libmirrorSL240.so: mirrorSL240.c circ.o $(SINC)/rtcmirror.h $(SINC)/darc.h $(SINC)/circ.h $(SINC)/arrayStruct.h buffer.o $(SINC)/buffer.h   $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -I../include -fPIC -O3 $(OPTS) -c -Wall -o mirrorSL240.o mirrorSL240.c -I/opt/sl240/nsl/inc -I/opt/nsl/inc
	gcc $(OPTS) -O3 -shared -Wl,-soname,libmirrorSL240.so.1 -o libmirrorSL240.so.1.0.1 mirrorSL240.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi
	/sbin/ldconfig -n ./
	rm -f libmirrorSL240.so
	ln -s  libmirrorSL240.so.1 libmirrorSL240.so
libmirrorNoSL240.so: mirrorSL240.c circ.o $(SINC)/rtcmirror.h $(SINC)/darc.h $(SINC)/circ.h $(SINC)/arrayStruct.h  buffer.o $(SINC)/buffer.h 
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DNOSL240 -fPIC -O3 $(OPTS) -c -I../include -Wall -o mirrorNoSL240.o mirrorSL240.c 
	gcc $(OPTS) -O3 -shared -Wl,-soname,libmirrorNoSL240.so.1 -o libmirrorNoSL240.so.1.0.1 mirrorNoSL240.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libmirrorNoSL240.so
	ln -s  libmirrorNoSL240.so.1 libmirrorNoSL240.so
libmirrorSocket.so: mirrorSocket.c circ.o $(SINC)/rtcmirror.h $(SINC)/darc.h $(SINC)/circ.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 $(OPTS) -c -Wall -I../include -o mirrorSocket.o mirrorSocket.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libmirrorSocket.so.1 -o libmirrorSocket.so.1.0.1 mirrorSocket.o -lpthread -lc
	/sbin/ldconfig -n ./
	rm -f libmirrorSocket.so
	ln -s  libmirrorSocket.so.1 libmirrorSocket.so

libmirrorSoundcard.so: mirrorSoundcard.c circ.o $(SINC)/rtcmirror.h $(SINC)/darc.h $(SINC)/circ.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 $(OPTS) -c -Wall -I../include -o mirrorSoundcard.o mirrorSoundcard.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libmirrorSoundcard.so.1 -o libmirrorSoundcard.so.1.0.1 mirrorSoundcard.o  -lc
	/sbin/ldconfig -n ./
	rm -f libmirrorSoundcard.so
	ln -s  libmirrorSoundcard.so.1 libmirrorSoundcard.so

libmirrorSHM.so: mirrorSHM.c circ.o $(SINC)/rtcmirror.h $(SINC)/darc.h $(SINC)/circ.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 $(OPTS) -c -Wall -I../include -o mirrorSHM.o mirrorSHM.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libmirrorSHM.so.1 -o libmirrorSHM.so.1.0.1 mirrorSHM.o -lpthread -lrt -lc
	/sbin/ldconfig -n ./
	rm -f libmirrorSHM.so
	ln -s  libmirrorSHM.so.1 libmirrorSHM.so


libfigure.so: figure.c $(SINC)/rtcfigure.h $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -fPIC -O3 $(OPTS) -c -Wall -I../include -o figure.o figure.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libfigure.so.1 -o libfigure.so.1.0.1 figure.o -lc
	/sbin/ldconfig -n ./
	rm -f libfigure.so
	ln -s  libfigure.so.1 libfigure.so

#libfigureSL240.so: figureSL240.c $(SINC)/rtcfigure.h $(SINC)/darc.h $(SINC)/arrayStruct.h
#	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -I../include -fPIC -O3 $(OPTS) -c -Wall -o figureSL240.o figureSL240.c -I/opt/nsl/inc
#	gcc -O3 -shared -Wl,-soname,libfigureSL240.so.1 -o libfigureSL240.so.1.0.1 figureSL240.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi
#	/sbin/ldconfig -n ./
#	rm -f libfigureSL240.so
#	ln -s  libfigureSL240.so.1 libfigureSL240.so

#libfigureSL240PassThrough.so: figureSL240PassThrough.c $(SINC)/rtcfigure.h $(SINC)/darc.h $(SINC)/arrayStruct.h
#	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 $(OPTS) -c -Wall -I../include -o figureSL240PassThrough.o figureSL240PassThrough.c -I/opt/nsl/inc -I/Canary/src/dmc/powerdaq-3.6.21/include
#	gcc -O3 -shared -Wl,-soname,libfigureSL240PassThrough.so.1 -o libfigureSL240PassThrough.so.1.0.1 figureSL240PassThrough.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi -lc -lpowerdaq32
#	/sbin/ldconfig -n ./
#	rm -f libfigureSL240PassThrough.so
#	ln -s  libfigureSL240PassThrough.so.1 libfigureSL240PassThrough.so
libfigureSL240SCPassThrough.so: figureSL240SCPassThrough.c $(SINC)/rtcfigure.h circ.o $(SINC)/darc.h $(SINC)/circ.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 $(OPTS) -c -Wall -I../include -o figureSL240SCPassThrough.o figureSL240SCPassThrough.c -I/Canary/src/SL240/sl240/inc -I/Canary/src/dmc/powerdaq-3.6.21/include
	gcc $(OPTS) -O3 -shared -Wl,-soname,libfigureSL240SCPassThrough.so.1 -o libfigureSL240SCPassThrough.so.1.0.1 figureSL240SCPassThrough.o -lpthread -lc -L/Canary/src/SL240/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -L/Canary/src/SL240/sl240/bin -lfxsl -lc -lpowerdaq32
	/sbin/ldconfig -n ./
	rm -f libfigureSL240SCPassThrough.so
	ln -s  libfigureSL240SCPassThrough.so.1 libfigureSL240SCPassThrough.so


libfigureSL240NSLPassThrough.so: figureSL240SCPassThrough.c $(SINC)/rtcfigure.h circ.o $(SINC)/darc.h $(SINC)/circ.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DNSL -fPIC -O3 $(OPTS) -c -Wall -I../include -o figureSL240NSLPassThrough.o figureSL240SCPassThrough.c -I/Canary/src/SL240/sl240/inc -I/Canary/src/dmc/powerdaq-3.6.21/include -I/opt/nsl/inc
	gcc $(OPTS) -O3 -shared -Wl,-soname,libfigureSL240NSLPassThrough.so.1 -o libfigureSL240NSLPassThrough.so.1.0.1 figureSL240NSLPassThrough.o -lpthread -lc -L/Canary/src/SL240/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -L/Canary/src/SL240/sl240/bin -lnslapi -lc -lpowerdaq32
	/sbin/ldconfig -n ./
	rm -f libfigureSL240NSLPassThrough.so
	ln -s  libfigureSL240NSLPassThrough.so.1 libfigureSL240NSLPassThrough.so

libfigureSL240NONSLPassThrough.so: figureSL240SCPassThrough.c $(SINC)/rtcfigure.h circ.o $(SINC)/darc.h $(SINC)/circ.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -DNONSL $(OPTS) -c -Wall -I../include -o figureSL240NONSLPassThrough.o figureSL240SCPassThrough.c -I/Canary/src/SL240/sl240/inc -I/Canary/src/dmc/powerdaq-3.6.21/include
	gcc $(OPTS) -O3 -shared -Wl,-soname,libfigureSL240NONSLPassThrough.so.1 -o libfigureSL240NONSLPassThrough.so.1.0.1 figureSL240NONSLPassThrough.o -lpthread -lc -L/Canary/src/SL240/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -L/Canary/src/SL240/sl240/bin  -lc -lpowerdaq32
	/sbin/ldconfig -n ./
	rm -f libfigureSL240NONSLPassThrough.so
	ln -s  libfigureSL240NONSLPassThrough.so.1 libfigureSL240NONSLPassThrough.so

libfigureSL240NONSLNODMPassThrough.so: figureSL240SCPassThrough.c $(SINC)/rtcfigure.h circ.o $(SINC)/darc.h $(SINC)/circ.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -DNONSL -DNODM -O3 $(OPTS) -c -Wall -I../include -o figureSL240NONSLNODMPassThrough.o figureSL240SCPassThrough.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libfigureSL240NONSLNODMPassThrough.so.1 -o libfigureSL240NONSLNODMPassThrough.so.1.0.1 figureSL240NONSLNODMPassThrough.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libfigureSL240NONSLNODMPassThrough.so
	ln -s  libfigureSL240NONSLNODMPassThrough.so.1 libfigureSL240NONSLNODMPassThrough.so


libfigureSocketPassThru.so: figureSocketPassThru.c $(SINC)/rtcfigure.h circ.o $(SINC)/darc.h $(SINC)/circ.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h $(SINC)/agbcblas.h agbcblas.o
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 $(OPTS) -c -Wall -o figureSocketPassThru.o figureSocketPassThru.c -I../include -I/Canary/src/dmc/powerdaq-3.6.21/include
	gcc $(OPTS) -O3 -shared -Wl,-soname,libfigureSocketPassThru.so.1 -o libfigureSocketPassThru.so.1.0.1 figureSocketPassThru.o -lpthread -lc -lpowerdaq32
	/sbin/ldconfig -n ./
	rm -f libfigureSocketPassThru.so
	ln -s  libfigureSocketPassThru.so.1 libfigureSocketPassThru.so

libfigureSocketPassThruNODM.so: figureSocketPassThru.c $(SINC)/rtcfigure.h circ.o $(SINC)/darc.h $(SINC)/circ.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h $(SINC)/agbcblas.h agbcblas.o
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DNODM -fPIC -O3 $(OPTS) -c -Wall -o figureSocketPassThruNODM.o figureSocketPassThru.c -I../include
	gcc $(OPTS) -O3 -shared -Wl,-soname,libfigureSocketPassThruNODM.so.1 -o libfigureSocketPassThruNODM.so.1.0.1 figureSocketPassThruNODM.o -lpthread -lc
	/sbin/ldconfig -n ./
	rm -f libfigureSocketPassThruNODM.so
	ln -s  libfigureSocketPassThruNODM.so.1 libfigureSocketPassThruNODM.so


libfigureSL240SC.so: figureSL240SC.c $(SINC)/rtcfigure.h $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -I../include -O3 $(OPTS) -c -Wall -o figureSL240SC.o figureSL240SC.c -I/Canary/src/SL240/sl240/inc 
	gcc $(OPTS) -O3 -shared -Wl,-soname,libfigureSL240SC.so.1 -o libfigureSL240SC.so.1.0.1 figureSL240SC.o -lpthread -lc -L/Canary/src/SL240/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -L/Canary/src/SL240/sl240/bin -lfxsl -lc
	/sbin/ldconfig -n ./
	rm -f libfigureSL240SC.so
	ln -s  libfigureSL240SC.so.1 libfigureSL240SC.so

libfigureSL240NSL.so: figureSL240SC.c $(SINC)/rtcfigure.h $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DNSL -fPIC -I../include -O3 $(OPTS) -c -Wall -o figureSL240NSL.o figureSL240SC.c -I/Canary/src/SL240/sl240/inc  -I/opt/nsl/inc  
	gcc $(OPTS) -O3 -shared -Wl,-soname,libfigureSL240NSL.so.1 -o libfigureSL240NSL.so.1.0.1 figureSL240NSL.o -lpthread -lc -L/Canary/src/SL240/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -L/Canary/src/SL240/sl240/bin -lnslapi -lc
	/sbin/ldconfig -n ./
	rm -f libfigureSL240NSL.so
	ln -s  libfigureSL240NSL.so.1 libfigureSL240NSL.so
libfigureSL240NONSL.so: figureSL240SC.c $(SINC)/rtcfigure.h $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DNONSL -fPIC -I../include -O3 $(OPTS) -c -Wall -o figureSL240NONSL.o figureSL240SC.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libfigureSL240NONSL.so.1 -o libfigureSL240NONSL.so.1.0.1 figureSL240NONSL.o -lpthread -lc -lc
	/sbin/ldconfig -n ./
	rm -f libfigureSL240NONSL.so
	ln -s  libfigureSL240NONSL.so.1 libfigureSL240NONSL.so

libfigureSL240SOCKET.so: figureSL240SC.c $(SINC)/rtcfigure.h $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DSOCKET -fPIC -I../include -O3 $(OPTS) -c -Wall -o figureSL240SOCKET.o figureSL240SC.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libfigureSL240SOCKET.so.1 -o libfigureSL240SOCKET.so.1.0.1 figureSL240SOCKET.o -lpthread -lc -lc
	/sbin/ldconfig -n ./
	rm -f libfigureSL240SOCKET.so
	ln -s  libfigureSL240SOCKET.so.1 libfigureSL240SOCKET.so

nslSendData: nslSendData.c
	gcc $(OPTS) -o nslSendData -DPLATFORM_UNIX  -I/opt/nsl/inc -I/opt/sl240/nsl/inc nslSendData.c -L/opt/nsl/linux-2.6/lib -L/opt/sl240/nsl/linux-2.6/lib -lnslapi
nslRecv: nslRecv.c
	gcc $(OPTS) -o nslRecv -DPLATFORM_UNIX -I/opt/nsl/inc -I/opt/nsl/inc nslRecv.c -L/opt/nsl/linux-current/lib -L/opt/sl240/nsl/linux-current/lib -lnslapi
utilsmodule.so: utils.c
	python setup.py build
	python setup.py install --install-lib=.
libmirrorPdAO32.so: mirrorPdAO32.c $(SINC)/rtcmirror.h $(SINC)/circ.h circ.o $(SINC)/darc.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h $(SINC)/agbcblas.h agbcblas.o
	gcc -D_GNU_SOURCE -fPIC -O3 $(OPTS) -c -Wall -I../include -I/Canary/src/dmc/powerdaq-3.6.24/include -I/opt/cfai/include/powerdaq -o mirrorPdAO32.o mirrorPdAO32.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libmirrorPdAO32.so.1 -o libmirrorPdAO32.so.1.0.1 mirrorPdAO32.o -lc -lpowerdaq32
	/sbin/ldconfig -n ./
	rm -f libmirrorPdAO32.so
	ln -s  libmirrorPdAO32.so.1 libmirrorPdAO32.so
libmirrorPdAO32Many.so: mirrorPdAO32Many.c $(SINC)/rtcmirror.h $(SINC)/circ.h circ.o $(SINC)/darc.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h $(SINC)/agbcblas.h agbcblas.o
	gcc -D_GNU_SOURCE -fPIC -O3 $(OPTS) -c -Wall -I../include -I/Canary/src/dmc/powerdaq-3.6.24/include -I/opt/cfai/include/powerdaq -o mirrorPdAO32Many.o mirrorPdAO32Many.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libmirrorPdAO32Many.so.1 -o libmirrorPdAO32Many.so.1.0.1 mirrorPdAO32Many.o -lc -lpowerdaq32
	/sbin/ldconfig -n ./
	rm -f libmirrorPdAO32Many.so
	ln -s  libmirrorPdAO32Many.so.1 libmirrorPdAO32Many.so
libmirrorPdAO32NODM.so: mirrorPdAO32.c $(SINC)/rtcmirror.h $(SINC)/circ.h circ.o $(SINC)/darc.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h $(SINC)/agbcblas.h agbcblas.o
	gcc -D_GNU_SOURCE -DNODM -fPIC -O3 $(OPTS) -c -Wall -I../include -I/Canary/src/dmc/powerdaq-3.6.21/include -o mirrorPdAO32NODM.o mirrorPdAO32.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libmirrorPdAO32NODM.so.1 -o libmirrorPdAO32NODM.so.1.0.1 mirrorPdAO32NODM.o -lc 
	/sbin/ldconfig -n ./
	rm -f libmirrorPdAO32NODM.so
	ln -s  libmirrorPdAO32NODM.so.1 libmirrorPdAO32NODM.so
libmirrorPdAO32ManyNODM.so: mirrorPdAO32Many.c $(SINC)/rtcmirror.h $(SINC)/circ.h circ.o $(SINC)/darc.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h $(SINC)/agbcblas.h agbcblas.o
	gcc -D_GNU_SOURCE -DNODM -fPIC -O3 $(OPTS) -c -Wall -I../include -I/Canary/src/dmc/powerdaq-3.6.21/include -o mirrorPdAO32ManyNODM.o mirrorPdAO32Many.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libmirrorPdAO32ManyNODM.so.1 -o libmirrorPdAO32ManyNODM.so.1.0.1 mirrorPdAO32ManyNODM.o -lc 
	/sbin/ldconfig -n ./
	rm -f libmirrorPdAO32ManyNODM.so
	ln -s  libmirrorPdAO32ManyNODM.so.1 libmirrorPdAO32ManyNODM.so

libmirrorAlpaoSdk.so: mirrorAlpaoSdk.c circ.o $(SINC)/rtcmirror.h $(SINC)/darc.h $(SINC)/circ.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 $(OPTS) -c -Wall -I../include -I/opt/alpao/Include -o mirrorAlpaoSdk.o mirrorAlpaoSdk.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libmirrorAlpaoSdk.so.1 -o libmirrorAlpaoSdk.so.1.0.1 mirrorAlpaoSdk.o -L/opt/alpao/Lib/x64 -lasdk -lpthread -lc
	/sbin/ldconfig -n ./
	rm -f libmirrorAlpaoSdk.so
	ln -s  libmirrorAlpaoSdk.so.1 libmirrorAlpaoSdk.so

libmirrorAlpaoSdkNODM.so: mirrorAlpaoSdk.c circ.o $(SINC)/rtcmirror.h $(SINC)/darc.h $(SINC)/circ.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DNODM -fPIC -O3 $(OPTS) -c -Wall -I../include -o mirrorAlpaoSdkNODM.o mirrorAlpaoSdk.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libmirrorAlpaoSdkNODM.so.1 -o libmirrorAlpaoSdkNODM.so.1.0.1 mirrorAlpaoSdkNODM.o -lpthread -lc
	/sbin/ldconfig -n ./
	rm -f libmirrorAlpaoSdkNODM.so
	ln -s  libmirrorAlpaoSdkNODM.so.1 libmirrorAlpaoSdkNODM.so

libmirrorPdAO32Alpao.so: mirrorPdAO32Alpao.c circ.o $(SINC)/rtcmirror.h $(SINC)/darc.h $(SINC)/circ.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h $(SINC)/agbcblas.h agbcblas.o
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 $(OPTS) -c -Wall -I../include  -I/Canary/src/dmc/powerdaq-3.6.21/include -I/opt/cfai/include/powerdaq -I/opt/alpao/Include -o mirrorPdAO32Alpao.o mirrorPdAO32Alpao.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libmirrorPdAO32Alpao.so.1 -o libmirrorPdAO32Alpao.so.1.0.1 mirrorPdAO32Alpao.o -L/opt/alpao/Lib/x64 -lasdk -lpthread -lc
	/sbin/ldconfig -n ./
	rm -f libmirrorPdAO32Alpao.so
	ln -s  libmirrorPdAO32Alpao.so.1 libmirrorPdAO32Alpao.so

libmirrorPdAO32AlpaoNODM.so: mirrorPdAO32Alpao.c circ.o $(SINC)/rtcmirror.h $(SINC)/darc.h $(SINC)/circ.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h $(SINC)/agbcblas.h agbcblas.o
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DNODM -fPIC -O3 $(OPTS) -c -Wall -I../include  -I/Canary/src/dmc/powerdaq-3.6.21/include -I/opt/alpao/Include -o mirrorPdAO32AlpaoNODM.o mirrorPdAO32Alpao.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libmirrorPdAO32AlpaoNODM.so.1 -o libmirrorPdAO32AlpaoNODM.so.1.0.1 mirrorPdAO32AlpaoNODM.o -L/opt/alpao/Lib/x64  -lpthread -lc
	/sbin/ldconfig -n ./
	rm -f libmirrorPdAO32AlpaoNODM.so
	ln -s  libmirrorPdAO32AlpaoNODM.so.1 libmirrorPdAO32AlpaoNODM.so


libjaicam.so: jaicam.cpp $(SINC)/rtccamera.h circ.o $(SINC)/darc.h $(SINC)/circ.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	g++ -Wall $(OPTS) -fPIC -I../include  -I/usr/include/JAI -c -o jaicam.o jaicam.cpp
	g++ -Wall $(OPTS)  -fPIC -shared -Wl,-soname,libjaicam.so.1 -o libjaicam.so.1.0.1 jaicam.o -lJAIFactory
#gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 $(OPTS) -c -Wall -I/usr/include/JAI -o jaicam.o jaicam.c 
#gcc -O3 -shared -Wl,-soname,librtccamera.so.1 -o librtccamera.so.1.0.1 jaicam.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libjaicam.so
	ln -s  libjaicam.so.1 libjaicam.so
sender: sender.c circ.o $(SINC)/circ.h
	gcc $(OPTS) -O3 -o sender -I../include sender.c circ.o -lrt -Wall -lpthread
summer: summer.c circ.o $(SINC)/circ.h
	gcc $(OPTS) -O3 -o summer -I../include summer.c circ.o -lrt -Wall -lpthread
splitter: splitter.c circ.o $(SINC)/circ.h
	gcc $(OPTS) -O3 -o splitter -I../include splitter.c circ.o -lrt -Wall -lpthread
receiver: receiver.c circ.o $(SINC)/circ.h
	gcc $(OPTS) -O3 -o receiver -I../include receiver.c circ.o -lpthread -lrt -Wall
leakyaverage: leakyaverage.c circ.o $(SINC)/circ.h
	gcc $(OPTS) -O3 -o leakyaverage -I../include leakyaverage.c circ.o -lrt -Wall -lpthread -lm

gpucalibrate: gpucalibrate.cu
	/usr/local/cuda/bin/nvcc -gencode=arch=compute_10,code=\"sm_10,compute_10\"  -gencode=arch=compute_20,code=\"sm_20,compute_20\"  -m64 --compiler-options -fno-strict-aliasing  -I. -I/usr/local/cuda/include -DUNIX -O2   -o gpucalibrate.o -c gpucalibrate.cu
	gcc $(OPTS) -fPIC -m64 -o gpucalibrate gpucalibrate.o -L/usr/local/cuda/lib64 -lcudart -L/usr/local/cuda/lib64 -L./ -lcuda 

gpumvm: gpumvm.cu
	/usr/local/cuda/bin/nvcc -gencode=arch=compute_20,code=\"sm_20,compute_20\"  -m64 --compiler-options -fno-strict-aliasing  -I. -I/usr/local/cuda/include -DUNIX -O2   -o gpumvm.o -c gpumvm.cu --ptxas-options=-v
	gcc $(OPTS) -fPIC -m64 -o gpumvm gpumvm.o -L/usr/local/cuda/lib64 -lcudart -L/usr/local/cuda/lib64 -L./ -lcuda 
gpumvmgen.o: gpumvmgen.cu $(SINC)/gpumvm.h
	/usr/local/cuda/bin/nvcc -gencode=arch=compute_20,code=\"sm_20,compute_20\"  -m64 --compiler-options -fno-strict-aliasing  -I. -I../include -I/usr/local/cuda/include -DUNIX -O2  -DDARCCPP -o gpumvmgen.o -c gpumvmgen.cu --ptxas-options=-v
libgpumvmgenlib.so: gpumvmgen.cu $(SINC)/gpumvm.h
	/usr/local/cuda/bin/nvcc -gencode=arch=compute_20,code=\"sm_20,compute_20\"  -m64 --compiler-options -fno-strict-aliasing  -I. -I../include -I/usr/local/cuda/include -DUNIX -O2 -shared --compiler-options '-fPIC' -o libgpumvmgenlib.so  gpumvmgen.cu -DGPUNOMAIN
gpumvmgenlib.o: gpumvmgen.cu $(SINC)/gpumvm.h
	/usr/local/cuda/bin/nvcc -gencode=arch=compute_20,code=\"sm_20,compute_20\"  -m64 --compiler-options -fno-strict-aliasing  -I. -I../include -I/usr/local/cuda/include -DUNIX -DDARCCPP -O2 -c --compiler-options '-fPIC' -o gpumvmgenlib.o  gpumvmgen.cu -DGPUNOMAIN
gpumvmgen: gpumvmgen.o
	gcc $(OPTS) -fPIC -m64 -o gpumvmgen gpumvmgen.o -L/usr/local/cuda/lib64 -lcudart -L/usr/local/cuda/lib64 -L./ -lcuda

mygpumvm.ptx: mygpumvm.cu
	/usr/local/cuda/bin/nvcc mygpumvm.cu -ptx

libreconmvmmycuda.so: gpumvmgenlib.o reconmvm.c $(SINC)/darc.h $(SINC)/darcNames.h agbcblas.o $(SINC)/arrayStruct.h  $(SINC)/buffer.h buffer.o 
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DUSECUDA -DMYCUBLAS -DUNIX -I../include -fPIC -O3 $(OPTS) -c -Wall -o reconmvmmycuda.o reconmvm.c -I/usr/local/cuda/include
	gcc $(OPTS) -O3 -shared -fPIC -Wl,-soname,libreconmvmmycuda.so.1 -o libreconmvmmycuda.so.1.0.1 gpumvmgenlib.o reconmvmmycuda.o -lpthread -lc -L/usr/local/cuda/lib64 -L/usr/local/cuda/lib64 -L/usr/lib/nvidia-current  -lcudart -lcuda -lstdc++ 
	/sbin/ldconfig -n ./
	rm -f libreconmvmmycuda.so
	ln -s  libreconmvmmycuda.so.1 libreconmvmmycuda.so

mygpumvm: mygpumvm.c mygpumvm.ptx
	gcc $(OPTS) -o mygpumvm mygpumvm.c -I/usr/local/cuda/include -L/usr/local/cuda/lib64 -m64 -lcuda -lm

cublasSgemvtest: cublasSgemvtest.c
	gcc $(OPTS) -o cublasSgemvtest cublasSgemvtest.c -O3 -Wall -I/usr/local/cuda/include -L/usr/local/cuda/lib64 -lcublas

#-fPIC -W -Wall -Wimplicit -Wswitch -Wformat -Wchar-subscripts -Wparentheses -Wmultichar -Wtrigraphs -Wpointer-arith -Wcast-align -Wreturn-type -Wno-unused-function   -m64 -fno-strict-aliasing -I. -I/usr/local/cuda/include -DUNIX -O2  -o gpucalibrate.o -c gpucalibrate.c

libcamuEyeUSB.so: camuEyeUSB.c $(SINC)/darc.h $(SINC)/darcNames.h agbcblas.o $(SINC)/arrayStruct.h  $(SINC)/buffer.h buffer.o 
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -I../include $(OPTS) -c -Wall -o camuEyeUSB.o camuEyeUSB.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libcamuEyeUSB.so.1 -o libcamuEyeUSB.so.1.0.1 camuEyeUSB.o -lc -lueye_api
	/sbin/ldconfig -n ./
	rm -f libcamuEyeUSB.so
	ln -s  libcamuEyeUSB.so.1 libcamuEyeUSB.so
libcamuEyeUSBMany.so: camuEyeUSBMany.c $(SINC)/darc.h $(SINC)/darcNames.h agbcblas.o $(SINC)/arrayStruct.h  $(SINC)/buffer.h buffer.o 
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -I../include $(OPTS) -c -Wall -o camuEyeUSBMany.o camuEyeUSBMany.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libcamuEyeUSBMany.so.1 -o libcamuEyeUSBMany.so.1.0.1 camuEyeUSBMany.o -lc -lueye_api
	/sbin/ldconfig -n ./
	rm -f libcamuEyeUSBMany.so
	ln -s  libcamuEyeUSBMany.so.1 libcamuEyeUSBMany.so

libmirrorLLS.so: mirrorLLS.c $(SINC)/rtcmirror.h $(SINC)/circ.h circ.o $(SINC)/darc.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -fPIC -O3 $(OPTS) -c -Wall -I../include  -o mirrorLLS.o mirrorLLS.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libmirrorLLS.so.1 -o libmirrorLLS.so.1.0.1 mirrorLLS.o -lc -lm
	/sbin/ldconfig -n ./
	rm -f libmirrorLLS.so
	ln -s  libmirrorLLS.so.1 libmirrorLLS.so
libandorcam.so: andor.c $(SINC)/rtccamera.h circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -I../include -O3 $(OPTS) -c -Wall -o andor.o andor.c -I/root/andor/examples/common -I/root/andor/include -I/home/ali
	gcc $(OPTS) -O3 -shared -Wl,-soname,libandorcam.so.1 -o libandorcam.so.1.0.1 andor.o -lc -L/root/andor/examples/common -L/root/andor/lib -landor
	/sbin/ldconfig -n ./
	rm -f libandorcam.so
	ln -s  libandorcam.so.1 libandorcam.so
libcamv4l.so: camv4l.c $(SINC)/rtccamera.h circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -fPIC -I../include -O3 $(OPTS) -c -Wall -o camv4l.o camv4l.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libcamv4l.so.1 -o libcamv4l.so.1.0.1 camv4l.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libcamv4l.so
	ln -s  libcamv4l.so.1 libcamv4l.so
libcamudp.so: camudp.c $(SINC)/rtccamera.h circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -fPIC -I../include -O3 $(OPTS) -c -Wall -o camudp.o camudp.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libcamudp.so.1 -o libcamudp.so.1.0.1 camudp.o -lpthread -lc
	/sbin/ldconfig -n ./
	rm -f libcamudp.so
	ln -s libcamudp.so.1 libcamudp.so
libreconLQG.so: reconLQG.c $(SINC)/darc.h $(SINC)/darcNames.h agbcblas.o $(SINC)/arrayStruct.h  $(SINC)/buffer.h buffer.o 
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DUSEAGBBLAS -I../include -fPIC -O3 $(OPTS) -c -Wall -o reconLQG.o reconLQG.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libreconLQG.so.1 -o libreconLQG.so.1.0.1 reconLQG.o agbcblas.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libreconLQG.so
	ln -s  libreconLQG.so.1 libreconLQG.so
ddsServer: ddsServer.c circ.o
	gcc $(OPTS) -g -O3 -o ddsServer -I../include ddsServer.c circ.o -lpthread -lrt -Wall
libcamfirewire.so: camfirewire.c $(SINC)/rtccamera.h circ.o buffer.o $(SINC)/buffer.h $(SINC)/circ.h  $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -fPIC -I../include -O3 $(OPTS) -c -Wall -o camfirewire.o camfirewire.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libcamfirewire.so.1 -o libcamfirewire.so.1.0.1 camfirewire.o -ldc1394 -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libcamfirewire.so
	ln -s  libcamfirewire.so.1 libcamfirewire.so
libmirrorBMMMulti.so: mirrorBMMMulti.c $(SINC)/rtcmirror.h $(SINC)/circ.h circ.o $(SINC)/darc.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -fPIC -O3 $(OPTS) -c -Wall -I../include -o mirrorBMMMulti.o mirrorBMMMulti.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libmirrorBMMMulti.so.1 -o libmirrorBMMMulti.so.1.0.1 mirrorBMMMulti.o -lc -lm -lusb
	/sbin/ldconfig -n ./
	rm -f libmirrorBMMMulti.so
	ln -s  libmirrorBMMMulti.so.1 libmirrorBMMMulti.so
libmirrorBMMMini.so: mirrorBMMMini.c $(SINC)/rtcmirror.h $(SINC)/circ.h circ.o $(SINC)/darc.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -fPIC -O3 $(OPTS) -c -Wall -I../include -o mirrorBMMMini.o mirrorBMMMini.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libmirrorBMMMini.so.1 -o libmirrorBMMMini.so.1.0.1 mirrorBMMMini.o -lc -lm -lusb
	/sbin/ldconfig -n ./
	rm -f libmirrorBMMMini.so
	ln -s  libmirrorBMMMini.so.1 libmirrorBMMMini.so

mirrorAdapticaProcess: mirrorAdapticaProcess.cpp
	g++ -g -L/opt/Adaptica_SW_PKG_1.2/LINUX/C/mirrorTestC++/Lib -I/opt/Adaptica_SW_PKG_1.2/LINUX/C/mirrorTestC++/Header -o mirrorAdapticaProcess -m32 mirrorAdapticaProcess.cpp -lpthread -lrt -lmirrorDriver /opt/Adaptica_SW_PKG_1.2/LINUX/C/mirrorTestC++/Src/ws-util.cpp /opt/Adaptica_SW_PKG_1.2/LINUX/C/mirrorTestC++/Src/serverSocketLnx.cpp /opt/Adaptica_SW_PKG_1.2/LINUX/C/mirrorTestC++/Src/Timer.cpp
	echo Put /opt/Adaptica_SW_PKG_1.2/LINUX/C/mirrorTestC++/Lib on your LD_LIBARRY_PATH
	sudo cp mirrorAdapticaProcess $(BIN)/
libmirrorAdaptica64.so: mirrorAdaptica64.c $(SINC)/rtcmirror.h $(SINC)/circ.h circ.o $(SINC)/darc.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -fPIC -O3 -g $(OPTS) -c -Wall -I../include -o mirrorAdaptica64.o mirrorAdaptica64.c
	gcc $(OPTS) -g -O3 -shared -Wl,-soname,libmirrorAdaptica64.so.1 -o libmirrorAdaptica64.so.1.0.1 mirrorAdaptica64.o -lc
	/sbin/ldconfig -n ./
	rm -f libmirrorAdaptica64.so
	ln -s  libmirrorAdaptica64.so.1 libmirrorAdaptica64.so
libmirrorBMMMultiAdaptica64.so: mirrorBMMMultiAdaptica64.c $(SINC)/rtcmirror.h $(SINC)/circ.h circ.o $(SINC)/darc.h $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -fPIC -O3 $(OPTS) -c -Wall -I../include -o mirrorBMMMultiAdaptica64.o mirrorBMMMultiAdaptica64.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libmirrorBMMMultiAdaptica64.so.1 -o libmirrorBMMMultiAdaptica64.so.1.0.1 mirrorBMMMultiAdaptica64.o -lc -lusb
	/sbin/ldconfig -n ./
	rm -f libmirrorBMMMultiAdaptica64.so
	ln -s  libmirrorBMMMultiAdaptica64.so.1 libmirrorBMMMultiAdaptica64.so
libreconneural.so: reconneural.c $(SINC)/darc.h $(SINC)/darcNames.h agbcblas.o $(SINC)/arrayStruct.h $(SINC)/buffer.h buffer.o 
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DUSEAGBBLAS -fPIC -O3 -I../include $(OPTS) -c -Wall -o reconneural.o reconneural.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libreconneural.so.1 -o libreconneural.so.1.0.1 reconneural.o agbcblas.o -lpthread -lm -lc 
	/sbin/ldconfig -n ./
	rm -f libreconneural.so
	ln -s  libreconneural.so.1 libreconneural.so
libreconscidar.so: reconscidar.c $(SINC)/darc.h $(SINC)/darcNames.h agbcblas.o $(SINC)/arrayStruct.h $(SINC)/buffer.h buffer.o 
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DUSEAGBBLAS -fPIC -O3 -I../include $(OPTS) -c -Wall -o reconscidar.o reconscidar.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libreconscidar.so.1 -o libreconscidar.so.1.0.1 reconscidar.o agbcblas.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libreconscidar.so
	ln -s  libreconscidar.so.1 libreconscidar.so
#probably needs libusb1-devel or libusb-1.0-0-dev
libcamsbig.so: camsbig.c $(SINC)/darc.h $(SINC)/darcNames.h agbcblas.o $(SINC)/arrayStruct.h  $(SINC)/buffer.h buffer.o 
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -I/usr/include/libusb-1.0 -I../include -I/opt/sbig $(OPTS) -c -Wall -o camsbig.o camsbig.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libcamsbig.so.1 -o libcamsbig.so.1.0.1 camsbig.o -L/opt/sbig -lc -lsbigudrv
	/sbin/ldconfig -n ./
	rm -f libcamsbig.so
	ln -s  libcamsbig.so.1 libcamsbig.so
libmirrorPdAO32Socket.so: mirrorPdAO32Socket.c $(SINC)/darc.h $(SINC)/rtcmirror.h $(SINC)/circ.h circ.o $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 $(OPTS) -c -Wall -I../include -I/Canary/src/dmc/powerdaq-3.6.21/include -o mirrorPdAO32Socket.o mirrorPdAO32Socket.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libmirrorPdAO32Socket.so.1 -o libmirrorPdAO32Socket.so.1.0.1 mirrorPdAO32Socket.o -lpthread -lc -lpowerdaq32
	/sbin/ldconfig -n ./
	rm -f libmirrorPdAO32Socket.so
	ln -s  libmirrorPdAO32Socket.so.1 libmirrorPdAO32Socket.so
libmirrorPdAO32SocketNODM.so: mirrorPdAO32Socket.c $(SINC)/darc.h $(SINC)/rtcmirror.h $(SINC)/circ.h circ.o $(SINC)/buffer.h buffer.o $(SINC)/arrayStruct.h $(SINC)/agbcblas.h agbcblas.o
	gcc -D_GNU_SOURCE -DNODM -DPLATFORM_UNIX -fPIC -O3 $(OPTS) -c -Wall -I../include -I/Canary/src/dmc/powerdaq-3.6.21/include -o mirrorPdAO32SocketNODM.o mirrorPdAO32Socket.c
	gcc $(OPTS) -O3 -shared -Wl,-soname,libmirrorPdAO32SocketNODM.so.1 -o libmirrorPdAO32SocketNODM.so.1.0.1 mirrorPdAO32SocketNODM.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libmirrorPdAO32SocketNODM.so
	ln -s  libmirrorPdAO32SocketNODM.so.1 libmirrorPdAO32SocketNODM.so
