all: darcmain utilsmodule.so libreconmvm.so libcamfile.so libreconKalman.so sender libcamsocket.so
include ../Makefile.config

SINC=../include
install:
	cp darcmain $(BIN)
	cp utilsmodule.so $(PY)
	cp libreconmvm.so $(LIB)
	cp libcamfile.so $(LIB)
	cp libreconKalman.so $(LIB)
	cp sender $(BIN)
	cp libcamsocket.so $(LIB)
	cp agbcblas.c $(SRC)
	cp andor.c $(SRC)
	cp camera.c $(SRC)
	cp camfile.c $(SRC)
	cp camsocket.c $(SRC)
	cp centroider.c $(SRC)
	cp circ.c $(SRC)
	cp core.c $(SRC)
	cp coremain.c $(SRC)
	cp coremodule.c $(SRC)
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
	cp utils.c $(SRC)
	cp sl240cam.c $(SRC)
	cp sl240centroider.c $(SRC)
	cp sl240Int32cam.c $(SRC)
	cp test.c $(SRC)
	cp testclip.c $(SRC)
	cp testpthread.c $(SRC)
	cp tmp.c $(SRC)
	cp tmpsubsupap.c $(SRC)
	cp Makefile $(SRC)

installdev:
	ln -sf $(PWD)/darcmain $(PWD)/../bin
	ln -sf $(PWD)/utilsmodule.so $(PWD)/../lib/python
	ln -sf $(PWD)/libreconmvm.so $(PWD)/../lib
	ln -sf $(PWD)/libcamfile.so $(PWD)/../lib
	ln -sf $(PWD)/libreconKalman.so $(PWD)/../lib
	ln -sf $(PWD)/sender $(PWD)/../bin
	ln -sf $(PWD)/libcamsocket.so $(PWD)/../lib

agbcblas.o: $(SINC)/agbcblas.h agbcblas.c
	gcc -Wall -O3 -c -I../include -o agbcblas.o agbcblas.c  -funroll-loops -msse2 -mfpmath=sse -march=native

darcmaingsl: darccore.c darcmain.c circ.o $(SINC)/darcNames.h $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -pthread -rdynamic -O3 -DUSEGSL -Wall -I../include -I/usr/local/include circ.o -L/usr/local/lib -L/usr/lib64 -lgslcblas -lpthread -lfftw3f -lm -lrt -ldl darcmain.c -o darcmain
	echo USING GSL
darcmain: darccore.c darcmain.c circ.o $(SINC)/darcNames.h $(SINC)/darc.h agbcblas.o $(SINC)/arrayStruct.h
	gcc -pthread -rdynamic -O3 -DUSEAGBBLAS -Wall -I../include -I/usr/local/include circ.o agbcblas.o -L/usr/local/lib -L/usr/lib64 -lpthread -lfftw3f -lm -lrt -ldl darcmain.c -o darcmain
	echo USING AGB BLAS

darcdebug: darccore.c darcmain.c circ.o $(SINC)/darcNames.h $(SINC)/darc.h $(SINC)/arrayStruct.h
	gcc -g -pthread -rdynamic -O3 -DUSEGSL -Wall -I../include -I/usr/local/include circ.o -L/usr/local/lib -L/usr/lib64 -lgslcblas -lpthread -lfftw3f -lm -lrt -ldl darcmain.c -o darcmain
	echo USING GSL

libreconmvm.so: reconmvm.c $(SINC)/darc.h $(SINC)/darcNames.h agbcblas.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DUSEAGBBLAS -fPIC -O3 -I../include -g -c -Wall -o reconmvm.o reconmvm.c
	gcc -O3 -shared -Wl,-soname,libreconmvm.so.1 -o libreconmvm.so.1.0.1 reconmvm.o agbcblas.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libreconmvm.so
	ln -s  libreconmvm.so.1 libreconmvm.so
libreconmvmgsl.so: reconmvm.c $(SINC)/darc.h $(SINC)/darcNames.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -I../include -g -c -Wall -o reconmvmgsl.o reconmvm.c
	gcc -O3 -shared -Wl,-soname,libreconmvmgsl.so.1 -o libreconmvmgsl.so.1.0.1 reconmvmgsl.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libreconmvmgsl.so
	ln -s  libreconmvmgsl.so.1 libreconmvmgsl.so

libreconmvmcuda.so: reconmvm.c $(SINC)/darc.h $(SINC)/darcNames.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DUSECUBLAS -I../include -fPIC -O3 -g -c -Wall -o reconmvmcuda.o reconmvm.c -I/usr/local/cuda/include
	gcc -O3 -shared -Wl,-soname,libreconmvmcuda.so.1 -o libreconmvmcuda.so.1.0.1 reconmvmcuda.o -lpthread -lc -L/usr/local/cuda/lib -L/usr/local/cuda/lib64 -lcublas
	/sbin/ldconfig -n ./
	rm -f libreconmvmcuda.so
	ln -s  libreconmvmcuda.so.1 libreconmvmcuda.so

libreconKalman.so: reconKalman.c $(SINC)/darc.h $(SINC)/darcNames.h agbcblas.o $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DUSEAGBBLAS -I../include -fPIC -O3 -g -c -Wall -o reconKalman.o reconKalman.c
	gcc -O3 -shared -Wl,-soname,libreconKalman.so.1 -o libreconKalman.so.1.0.1 reconKalman.o agbcblas.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libreconKalman.so
	ln -s  libreconKalman.so.1 libreconKalman.so
libreconKalman.so_gsl: reconKalman.c $(SINC)/darc.h $(SINC)/darcNames.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -g -c -I../include -Wall -o reconKalman.o reconKalman.c
	gcc -O3 -shared -Wl,-soname,libreconKalman.so.1 -o libreconKalman.so.1.0.1 reconKalman.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libreconKalman.so
	ln -s  libreconKalman.so.1 libreconKalman.so


circ.o: circ.c $(SINC)/circ.h
	gcc -Wall -O3 -I../include -c circ.c -o circ.o -DUSEGSL

libcamera.so: camera.c $(SINC)/rtccamera.h
	gcc -fPIC -g -c -Wall -I../include -o camera.o camera.c
	gcc -shared -Wl,-soname,libcamera.so.1 -o libcamera.so.1.0.1 camera.o -lc
	/sbin/ldconfig -n ./
	rm -f libcamera.so
	ln -s  libcamera.so.1 libcamera.so
libcentroider.so: centroider.c $(SINC)/rtccentroider.h
	gcc -fPIC -g -c -Wall -I../include -o centroider.o centroider.c
	gcc -shared -Wl,-soname,libcentroider.so.1 -o libcentroider.so.1.0.1 centroider.o -lc
	/sbin/ldconfig -n ./
	rm -f libcentroider.so
	ln -s  libcentroider.so.1 libcentroider.so

andor: andor.c
	gcc -fPIC -O3 -g -c -Wall -o andor.o andor.c
	gcc -O3 -shared -Wl,-soname,librtccamera.so.1 -o librtccamera.so.1.0.1 andor.o -lc -L/root/andor/examples/common -landor-shared-gcc-3.4.4
	/sbin/ldconfig -n ./
	rm -f librtccamera.so
	ln -s  librtccamera.so.1 librtccamera.so
libcamfile.so: camfile.c $(SINC)/rtccamera.h
	gcc -D_GNU_SOURCE -fPIC -I../include -O3 -g -c -Wall -o camfile.o camfile.c
	gcc -O3 -shared -Wl,-soname,libcamfile.so.1 -o libcamfile.so.1.0.1 camfile.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libcamfile.so
	ln -s  libcamfile.so.1 libcamfile.so

libcamsocket.so: camsocket.c $(SINC)/rtccamera.h
	gcc -D_GNU_SOURCE -fPIC -O3 -I../include -g -c -Wall -o camsocket.o camsocket.c
	gcc -O3 -shared -Wl,-soname,libcamsocket.so.1 -o libcamsocket.so.1.0.1 camsocket.o -lc 
	/sbin/ldconfig -n ./
	rm -f libcamsocket.so
	ln -s  libcamsocket.so.1 libcamsocket.so

libsl240Int32cam.so: sl240Int32cam.c $(SINC)/rtccamera.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -I../include -O3 -g -c -Wall -o sl240Int32cam.o sl240Int32cam.c -I/opt/sl240/nsl/inc -I/opt/nsl/inc
	gcc -O3 -shared -Wl,-soname,libsl240Int32cam.so.1 -o libsl240Int32cam.so.1.0.1 sl240Int32cam.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi
	/sbin/ldconfig -n ./
	rm -f libsl240Int32cam.so
	ln -s  libsl240Int32cam.so.1 libsl240Int32cam.so
libsl240centroider.so: sl240centroider.c
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -g -c -Wall -o sl240centroider.o sl240centroider.c -I/opt/sl240/nsl/inc -I/opt/nsl/inc 
	gcc -O3 -shared -Wl,-soname,libsl240centroider.so.1 -o libsl240centroider.so.1.0.1 sl240centroider.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi
	/sbin/ldconfig -n ./
	rm -f libsl240centroider.so
	ln -s  libsl240centroider.so.1 libsl240centroider.so
libmirror.so: mirror.c $(SINC)/rtcmirror.h $(SINC)/circ.h
	gcc -fPIC -O3 -g -c -Wall -I../include -o mirror.o mirror.c
	gcc -O3 -shared -Wl,-soname,libmirror.so.1 -o libmirror.so.1.0.1 mirror.o -lc
	/sbin/ldconfig -n ./
	rm -f libmirror.so
	ln -s  libmirror.so.1 libmirror.so
libmirrorSL240.so: mirrorSL240.c circ.o $(SINC)/rtcmirror.h $(SINC)/darc.h $(SINC)/circ.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -I../include -fPIC -O3 -g -c -Wall -o mirrorSL240.o mirrorSL240.c -I/opt/sl240/nsl/inc -I/opt/nsl/inc
	gcc -O3 -shared -Wl,-soname,libmirrorSL240.so.1 -o libmirrorSL240.so.1.0.1 mirrorSL240.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi
	/sbin/ldconfig -n ./
	rm -f libmirrorSL240.so
	ln -s  libmirrorSL240.so.1 libmirrorSL240.so
libmirrorNoSL240.so: mirrorSL240.c circ.o $(SINC)/rtcmirror.h $(SINC)/darc.h $(SINC)/circ.h $(SINC)/arrayStruct.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DNOSL240 -fPIC -O3 -g -c -I../include -Wall -o mirrorNoSL240.o mirrorSL240.c 
	gcc -O3 -shared -Wl,-soname,libmirrorNoSL240.so.1 -o libmirrorNoSL240.so.1.0.1 mirrorNoSL240.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libmirrorNoSL240.so
	ln -s  libmirrorNoSL240.so.1 libmirrorNoSL240.so

libfigure.so: figure.c $(SINC)/rtcfigure.h
	gcc -fPIC -O3 -g -c -Wall -I../include -o figure.o figure.c
	gcc -O3 -shared -Wl,-soname,libfigure.so.1 -o libfigure.so.1.0.1 figure.o -lc
	/sbin/ldconfig -n ./
	rm -f libfigure.so
	ln -s  libfigure.so.1 libfigure.so

libfigureSL240.so: figureSL240.c $(SINC)/rtcfigure.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -I../include -fPIC -O3 -g -c -Wall -o figureSL240.o figureSL240.c -I/opt/nsl/inc
	gcc -O3 -shared -Wl,-soname,libfigureSL240.so.1 -o libfigureSL240.so.1.0.1 figureSL240.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi
	/sbin/ldconfig -n ./
	rm -f libfigureSL240.so
	ln -s  libfigureSL240.so.1 libfigureSL240.so

libfigureSL240PassThrough.so: figureSL240PassThrough.c $(SINC)/rtcfigure.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -g -c -Wall -I../include -o figureSL240PassThrough.o figureSL240PassThrough.c -I/opt/nsl/inc -I/Canary/src/dmc/powerdaq-3.6.20/include
	gcc -O3 -shared -Wl,-soname,libfigureSL240PassThrough.so.1 -o libfigureSL240PassThrough.so.1.0.1 figureSL240PassThrough.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi -lc -lpowerdaq32
	/sbin/ldconfig -n ./
	rm -f libfigureSL240PassThrough.so
	ln -s  libfigureSL240PassThrough.so.1 libfigureSL240PassThrough.so
libfigureSL240SCPassThrough.so: figureSL240SCPassThrough.c $(SINC)/rtcfigure.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -g -c -Wall -I../include -o figureSL240SCPassThrough.o figureSL240SCPassThrough.c -I/Canary/src/SL240/sl240/inc -I/Canary/src/dmc/powerdaq-3.6.20/include
	gcc -O3 -shared -Wl,-soname,libfigureSL240SCPassThrough.so.1 -o libfigureSL240SCPassThrough.so.1.0.1 figureSL240SCPassThrough.o -lpthread -lc -L/Canary/src/SL240/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -L/Canary/src/SL240/sl240/bin -lfxsl -lc -lpowerdaq32
	/sbin/ldconfig -n ./
	rm -f libfigureSL240SCPassThrough.so
	ln -s  libfigureSL240SCPassThrough.so.1 libfigureSL240SCPassThrough.so

libfigureSL240SC.so: figureSL240SC.c $(SINC)/rtcfigure.h
	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -I../include -O3 -g -c -Wall -o figureSL240SC.o figureSL240SC.c -I/Canary/src/SL240/sl240/inc 
	gcc -O3 -shared -Wl,-soname,libfigureSL240SC.so.1 -o libfigureSL240SC.so.1.0.1 figureSL240SC.o -lpthread -lc -L/Canary/src/SL240/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -L/Canary/src/SL240/sl240/bin -lfxsl -lc
	/sbin/ldconfig -n ./
	rm -f libfigureSL240SC.so
	ln -s  libfigureSL240SC.so.1 libfigureSL240SC.so
nslSendData: nslSendData.c
	gcc -o nslSendData -DPLATFORM_UNIX  -I/opt/nsl/inc -I/opt/sl240/nsl/inc nslSendData.c -L/opt/nsl/linux-2.6/lib -L/opt/sl240/nsl/linux-2.6/lib -lnslapi
nslRecv: nslRecv.c
	gcc -o nslRecv -DPLATFORM_UNIX -I/opt/sl240/nsl/inc -I/opt/nsl/inc nslRecv.c -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi
utilsmodule.so: utils.c
	python setup.py build
	python setup.py install --install-lib=.
libmirrorPdAO32.so: mirrorPdAO32.c $(SINC)/rtcmirror.h $(SINC)/circ.h
	gcc -D_GNU_SOURCE -fPIC -O3 -g -c -Wall -I../include -I/Canary/src/dmc/powerdaq-3.6.20/include -o mirrorPdAO32.o mirrorPdAO32.c
	gcc -O3 -shared -Wl,-soname,libmirrorPdAO32.so.1 -o libmirrorPdAO32.so.1.0.1 mirrorPdAO32.o -lc -lpowerdaq32
	/sbin/ldconfig -n ./
	rm -f libmirrorPdAO32.so
	ln -s  libmirrorPdAO32.so.1 libmirrorPdAO32.so
libjaicam.so: jaicam.cpp $(SINC)/rtccamera.h
	g++ -Wall -g -I../include   -c -o jaicam.o jaicam.cpp
	g++ -Wall -g  -fPIC -shared -Wl,-soname,libjaicam.so.1 -o libjaicam.so.1.0.1 jaicam.o -lJAIFactory
	#gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -g -c -Wall -I/usr/include/JAI -o jaicam.o jaicam.c 
	#gcc -O3 -shared -Wl,-soname,librtccamera.so.1 -o librtccamera.so.1.0.1 jaicam.o -lpthread -lc 
	/sbin/ldconfig -n ./
	rm -f libjaicam.so
	ln -s  libjaicam.so.1 libjaicam.so
sender: sender.c circ.o $(SINC)/circ.h
	gcc -o sender -I../include sender.c circ.o -lrt -Wall
