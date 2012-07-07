#darc, the Durham Adaptive optics Real-time Controller.
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


include Makefile.config
-include Makefile.config.local

all:
	(cd idl && make)
	(cd src && make)
	(cd lib && make)
	(cd conf && make)
	(cd test && make)
	(cd etc && make)
	(cd include && make)
	(cd bin && make)
	export PYTHONPATH=$$PYTHONPATH:$(PWD)/lib/python
	export PATH=$$PATH:$(PWD)/bin
	export LD_LIBRARY_PATH=$$LD_LIBRARY_PATH:$(PWD)/lib


path:
	export PYTHONPATH=$$PYTHONPATH:$(PWD)/lib/python
	export PATH=$$PATH:$(PWD)/bin
	export LD_LIBRARY_PATH=$$LD_LIBRARY_PATH:$(PWD)/lib


clean:
	(cd idl && make clean)
	(cd src && make clean)
	(cd lib && make clean)
	(cd conf && make clean)
	(cd test && make clean)
	(cd etc && make clean)
	(cd include && make clean)
	(cd bin && make clean)


##all: control_idl.py darcmain utilsmodule.so libreconmvm.so libcamfile.so libreconKalman.so sender

docs: 
	(cd doc && make)
installdocs:
	(cd doc && make install)

install: all darcclient.tgz
	mkdir -p $(BASE)
	mkdir -p $(BIN)
	mkdir -p $(LIB)
	mkdir -p $(PY)
	mkdir -p $(IDL)
	mkdir -p $(ETC)
	mkdir -p $(SRC)
	mkdir -p $(CONF)
	mkdir -p $(TEST)
	mkdir -p $(INC)
	mkdir -p $(DOC)
	(cd idl && make install)
	(cd src && make install)
	(cd lib && make install)
	(cd conf && make install)
	(cd test && make install)
	(cd etc && make install)
	(cd include && make install)
	(cd bin && make install)
	cp README $(BASE)
	cp Makefile $(BASE)
	cp Makefile.config $(BASE)
	cp Makefile.client $(BASE)
	cp README.client $(BASE)
#	cp README.rtcgui $(BASE)
	cp INSTALL $(BASE)
	cp darcclient.tgz $(BASE)
	date > $(BASE)/date.txt
	export PYTHONPATH=$$PYTHONPATH:$(PY)
	export PATH=$$PATH:$(BIN)
	export LD_LIBRARY_PATH=$$LD_LIBRARY_PATH:$(LIB)
	echo "Remember to make and install manually any custom shared libraries"

#Copy links so that your development area works like an install
#This should only ever create soft links.
installdev: all
	(cd idl && make installdev)
	(cd src && make installdev)
	(cd lib && make installdev)
	(cd conf && make installdev)
	(cd test && make installdev)
	(cd etc && make installdev)
	(cd include && make installdev)
	(cd bin && make installdev)
	export PYTHONPATH=$$PYTHONPATH:$(PWD)/lib/python
	export PATH=$$PATH:$(PWD)/bin
	export LD_LIBRARY_PATH=$$LD_LIBRARY_PATH:$(PWD)/lib

ubuntu1004:
	sudo apt-get install git-core emacs openssh-server python-omniorb fftw3 fftw3-dev omniidl4 omniidl4-python omniorb4-nameserver python-dev python-numpy glade python-matplotlib
ubuntu1004docs:
	sudo apt-get install texlive-latex-base texlive-fonts-recommended doxygen texlive
ubuntu1010:
	sudo apt-get install git-core emacs openssh-server python-omniorb fftw3 fftw3-dev omniidl omniidl-python omniorb-nameserver python-dev python-numpy glade python-matplotlib
ubuntu1204:
	sudo apt-get install git-core emacs openssh-server python-omniorb libfftw3-3 libfftw3-dev omniidl omniidl-python omniorb-nameserver python-dev python-numpy glade python-matplotlib

fedora12: omniORB
	yum install emacs git numpy python-devel glade3 python-matplotlib gcc fftw3-devel gcc-c++ python-inotify
fedora14: omniORB
	yum install emacs git numpy python-devel glade3 python-matplotlib gcc fftw3-devel gcc-c++ python-inotify
fedora12docs:
	yum install texlive-latex doxygen

installold: all darcclient.tgz
	mkdir -p $(BASE)
	mkdir -p $(BIN)
	mkdir -p $(LIB)
	mkdir -p $(PY)
	mkdir -p $(IDL)
	mkdir -p $(ETC)
	mkdir -p $(SRC)
	mkdir -p $(CONF)
	mkdir -p $(TEST)
	mkdir -p $(INC)
	cp control.py $(PY)
##	rm -f $(BIN)/control.py
##	ln -s $(PY)/control.py $(BIN)/control.py
	cp startStreams.py $(PY)
##	rm -f $(BIN)/startStreams.py
##	ln -s $(PY)/startStreams.py $(BIN)/startStreams.py
	cp sendStream.py $(PY)
##	rm -f $(BIN)/sendStream.py
##	ln -s $(PY)/sendStream.py $(BIN)/sendStream.py
##	cp sender $(BIN)
	cp recvStream.py $(PY)
##	rm -f $(BIN)/recvStream.py
##	ln -s $(PY)/recvStream.py $(BIN)/recvStream.py
	cp rtcgui.py $(PY)
##	rm -f $(BIN)/rtcgui.py
##	ln -s $(PY)/rtcgui.py $(BIN)/rtcgui.py
	cp rtcgui.glade $(BIN)
	cp deletesem $(BIN)
##	cp utilsmodule.so $(PY)
	cp buffer.py $(PY)
	cp Check.py  $(PY)
	cp stdoutlog.py $(PY)
	cp ConnObj.py  $(PY)
	cp controlCorba.py  $(PY)
##	cp control_idl.py  $(PY)
	cp correlation.py  $(PY)
	cp dataSwitchClient.py  $(PY)
	cp logread.py $(PY)
	cp spot.py $(PY)
	cp PSuser.py $(PY)
	cp FITS.py  $(PY)
	cp overlayMaker.py  $(PY)
	cp plot.py  $(PY)
##	rm -f $(BIN)/plot.py
##	ln -s $(PY)/plot.py $(BIN)/plot.py
	cp tel.py  $(PY)
	cp serialise.py  $(PY)
	cp SockConn.py  $(PY)
	cp zernike.py  $(PY)
	cp control.idl $(IDL)
	cp rtc.bashrc $(ETC)
	cp runsend.sh $(BIN)
	cp runcontrol.sh $(BIN)
	cp runrtc.sh $(BIN)
	cp runrecv.sh $(BIN)
	cp Saver.py $(PY)
	cp config*.py $(CONF)
	cp plot*.xml $(CONF)
	cp init*.py $(CONF)
	cp rtcguirc.py $(CONF)
	cp socketCam.py $(PY)
	##	rm -f $(BIN)/socketCam.py
	##	ln -s $(PY)/socketCam.py $(BIN)/socketCam.py
	cp socketDM.py $(PY)
	##	rm -f $(BIN)/socketDM.py
	##	ln -s $(PY)/socketDM.py $(BIN)/socketDM.py
	cp testScript.py $(TEST)
	cp testcal.py $(TEST)
	cp *.h $(INC)
	cp *.c $(SRC)
	cp *.cpp $(SRC)
	cp Makefile $(BASE)
	cp Makefile.config $(BASE)
	cp Makefile.client $(BASE)
	cp README $(BASE)
	cp README.client $(BASE)
#	cp README.rtcgui $(BASE)
	cp INSTALL $(BASE)
	##	cp darctalk.tgz $(BASE)
	cp darctalk $(BIN)
	cp darcmagic $(BIN)
	##	chmod a+r $(BASE)/darctalk.tgz
	##	cp darcmain $(BIN)
	##	chmod a+rx $(BIN)/darcmain
	##	chmod a+rx $(BIN)/control.py
	##	chmod a+rx $(BIN)/startStreams.py
	##	chmod a+rx $(BIN)/sendStream.py
	##	chmod a+rx $(BIN)/recvStream.py
	##	chmod a+rx $(BIN)/rtcgui.py
	chmod a+rx $(BIN)/deletesem
	chmod a+rx $(BIN)/runsend.sh
	chmod a+rx $(BIN)/runcontrol.sh
	chmod a+rx $(BIN)/runrtc.sh
	chmod a+rx $(BIN)/runrecv.sh
	chmod a+rx $(BIN)/darctalk
	chmod a+rx $(BIN)/darcmagic
	##	chmod a+rx $(BIN)/sender
	chmod a+r $(PY)/*
	chmod a+r $(BASE)/*
#	rm -f $(PY)/startStreams.py
#	ln -s $(BIN)/startStreams.py $(PY)/startStreams.py
#	rm -f $(PY)/recvStream.py
#	ln -s $(BIN)/recvStream.py $(PY)/recvStream.py
#	rm -f $(PY)/sendStream.py
#	ln -s $(BIN)/sendStream.py $(PY)/sendStream.py
##	chmod a+x $(BIN)/plot.py
##	date > $(BASE)/date.txt
##	cp libreconmvm.so $(LIB)
##	cp libcamfile.so $(LIB)
##	echo "Copying existing .so library modules to lib"
##	ls lib*.so
##	cp lib*.so $(LIB)
##	echo "Do you need any library modules:"
##	grep lib Makefile | grep .so: | sed -e 's/:.*//'
##	echo "If so, copy them to lib"

installRecv: src/utilsmodule.so
	cp lib/python/recvStream.py $(PY)
	cp lib/python/ConnObj.py  $(PY)
	cp lib/python/SockConn.py  $(PY)
	cp src/utilsmodule.so $(PY)
	cp lib/python/serialise.py  $(PY)
	cp etc/rtc.bashrc $(ETC)
	cp bin/runrecv.sh $(BIN)
	chmod a+x $(PY)/recvStream.py
	chmod a+x $(BIN)/runrecv.sh
	ln -fs $(PY)/recvStream.py $(BIN)/recvStream.py 
#	ln -fs $(PY)/sendStream.py $(BIN)/sendStream.py

src/utilsmodule.so: src/utils.c
	(cd src && make utilsmodule.so)

version:
	git commit -m version bin/darctalk bin/darcmagic src/darcmain.c src/darccore.c lib/python/control.py lib/python/controlCorba.py
	rm bin/darctalk bin/darcmagic src/darcmain.c src/darccore.c lib/python/control.py lib/python/controlCorba.py
	git checkout -- bin/darctalk bin/darcmagic src/darcmain.c src/darccore.c lib/python/control.py lib/python/controlCorba.py

# darctalk.tgz:
# 	mkdir -p DARC
# 	mkdir -p DARC/lib
# 	cp bin/darctalk DARC
# 	cp bin/darcmagic DARC/lib/darcmagic.py
# 	cp README.darctalk DARC
# 	cp lib/python/controlCorba.py idl/control.idl lib/python/FITS.py lib/python/recvStream.py lib/python/SockConn.py lib/python/serialise.py lib/python/Saver.py lib/python/ConnObj.py DARC/lib/
# 	tar -zcvf darctalk.tgz DARC
# 	rm -rf DARC
# 	cp darctalk.tgz darctalk-`grep "Id:" bin/darctalk | sed 's/CVSID="\$$Id: //;s/ \$$"//'`.tgz
# 	echo darctalk-`grep "Id:" bin/darctalk | sed 's/CVSID="\$$Id: //;s/ \$$"//'`.tgz
# #	cp darctalk.tgz darctalk-`grep "Id:" bin/darctalk | sed 's/CVSID="//;s/Id: darctalk,v //;s/\([ ]*\) .*/\1/;s/.\(.*\)/\1/'`.tgz
# #	echo darctalk-`grep "Id:" bin/darctalk | sed 's/CVSID="//;s/Id: darctalk,v //;s/\([ ]*\) .*/\1/;s/.\(.*\)/\1/'`.tgz


# rtcgui.tgz:
# 	mkdir -p RTCGUI
# 	mkdir -p RTCGUI/lib
# 	cp lib/python/rtcgui.py RTCGUI
# 	cp bin/rtcgui.glade RTCGUI
# 	cp README.rtcgui RTCGUI
# 	cp lib/python/controlCorba.py idl/control.idl lib/python/FITS.py lib/python/buffer.py lib/python/serialise.py lib/python/plot.py lib/python/correlation.py lib/python/Check.py lib/python/recvStream.py lib/python/Saver.py lib/python/SockConn.py lib/python/ConnObj.py lib/python/plotxml.py RTCGUI/lib
# 	(cd RTCGUI && ln -fs lib/plot.py plot.py  && chmod a+x lib/plot.py)
# 	tar -zcvf rtcgui.tgz RTCGUI
# 	rm -rf RTCGUI
# 	cp rtcgui.tgz rtcgui-`grep "Id:" bin/darctalk | sed 's/CVSID="\$$Id: //;s/ \$$"//'`.tgz
# 	echo rtcgui-`grep "Id:" bin/darctalk | sed 's/CVSID="\$$Id: //;s/ \$$"//'`.tgz
# #	cp rtcgui.tgz rtcgui-`grep "Id:" lib/python/rtcgui.py | sed 's/CVSID="//;s/Id: rtcgui\.py,v //;s/\([ ]*\) .*/\1/;s/.\(.*\)/\1/'`.tgz
# #	echo rtcgui-`grep "Id:" lib/python/rtcgui.py | sed 's/CVSID="//;s/Id: rtcgui\.py,v //;s/\([ ]*\) .*/\1/;s/.\(.*\)/\1/'`.tgz


darcclient.tgz:
	mkdir -p DARC
	mkdir -p DARC/lib
	mkdir -p DARC/src
	mkdir -p DARC/include
	cp include/circ.h DARC/include/
	cp src/utils.c DARC/src/
	cp src/setup.py DARC/src/
	cp src/receiver.c DARC/src/
	cp src/circ.c DARC/src/
	cp Makefile.client DARC/Makefile
	grep SINC= src/Makefile > DARC/src/Makefile
	grep OPTS= src/Makefile >> DARC/src/Makefile
	grep -A 1 "receiver:" src/Makefile >> DARC/src/Makefile
	grep -A 1 "circ.o:" src/Makefile >> DARC/src/Makefile
	cp bin/darctalk DARC
	cp bin/darcmagic DARC
	cp lib/python/rtcgui.py DARC/lib/
	cp bin/rtcgui.glade DARC
	cp README.client DARC
	cp lib/python/controlCorba.py idl/control.idl lib/python/FITS.py lib/python/recvStream.py lib/python/SockConn.py lib/python/serialise.py lib/python/Saver.py lib/python/ConnObj.py lib/python/buffer.py lib/python/plot.py lib/python/correlation.py lib/python/Check.py lib/python/plotxml.py lib/python/startStreams.py DARC/lib/
	(cd DARC && ln -fs lib/plot.py darcplot && chmod a+x lib/plot.py) 
	(cd DARC && ln -fs lib/rtcgui.py darcgui && chmod a+x lib/rtcgui.py) 
	tar -zcvf darcclient.tgz DARC
	rm -rf DARC
	cp darcclient.tgz darcclient-`grep "Id:" bin/darctalk | sed 's/CVSID="\$$Id: //;s/ \$$"//'`-`grep "Id:" bin/darctalk | sed 's/CVSID="\$$Id: //;s/ \$$"//'`.tgz
	echo darcclient-`grep "Id:" bin/darctalk | sed 's/CVSID="\$$Id: //;s/ \$$"//'`-`grep "Id:" bin/darctalk | sed 's/CVSID="\$$Id: //;s/ \$$"//'`.tgz

fftw:
	wget http://www.fftw.org/fftw-3.2.2.tar.gz
	tar -zxvf fftw-3.2.2.tar.gz
	(cd fftw-3.2.2 && ./configure --enable-float --enable-shared && make && make install)

gsl:
	wget ftp://www.mirrorservice.org/sites/ftp.gnu.org/gnu/gsl/gsl-1.14.tar.gz
	tar -zxvf gsl-1.14.tar.gz
	(cd gsl-1.14 && ./configure && make && make install)

numpy:
	wget http://sourceforge.net/projects/numpy/files/NumPy/1.4.1/numpy-1.4.1.tar.gz/download
	tar -zxvf numpy-1.4.1.tar.gz
	(cd numpy-1.4.1 && python setup.py build && python setup.py install)

omniORB:
	wget http://omniorb.sourceforge.net/releases/omniORB-4.1.4.tar.gz
	tar -zxvf omniORB-4.1.4.tar.gz
	mkdir -p omniORB-4.1.4/build
	(cd omniORB-4.1.4/build && ../configure && make && make install)
	wget http://omniorb.sourceforge.net/releases/omniORBpy-3.4.tar.gz
	tar -zxvf omniORBpy-3.4.tar.gz
	mkdir -p omniORBpy-3.4/build
	(cd omniORBpy-3.4/build && ../configure && make && make install)
matplotlib:
	wget http://sourceforge.net/projects/matplotlib/files/matplotlib/matplotlib-0.99.1/matplotlib-0.99.1.2.tar.gz/download
	tar -zxvf matplotlib-0.99.1.2.tar.gz
	(cd matplotlib-0.99.1* && python setup.py build && python setup.py install)

#rtc: core.c coremain.c
#	gcc -Wall -O3 -c circ.c -o circ.o -DUSEMKL
#	gcc -pthread -O3 -DUSEMKL -Wall -I/opt/intel/mkl/10.0.1.014/include circ.o -L/opt/intel/mkl/10.0.1.014/lib/em64t -lmkl_intel_lp64 -lmkl_intel_thread -lmkl_core -lguide -lpthread -lm -lrt -L. -lrtccamera coremain.c -o coremain
#	sudo chown root:root coremain
#	echo USING MKL

#agbcblas.o: agbcblas.h agbcblas.c
#	gcc -Wall -O3 -c -o agbcblas.o agbcblas.c  -funroll-loops -msse2 -mfpmath=sse -march=native

#coremain: core.c coremain.c circ.o paramNames.h
#	gcc -pthread -rdynamic -O3 -DUSEGSL -Wall -I/usr/local/include circ.o -L/usr/local/lib -L/usr/lib64 -lgslcblas -lpthread -lfftw3f -lm -lrt -ldl coremain.c -o coremain
#	echo USING GSL
#sudo chown root:root coremain

# darcmaingsl: darccore.c darcmain.c circ.o darcNames.h darc.h
# 	gcc -pthread -rdynamic -O3 -DUSEGSL -Wall -I/usr/local/include circ.o -L/usr/local/lib -L/usr/lib64 -lgslcblas -lpthread -lfftw3f -lm -lrt -ldl darcmain.c -o darcmain
# 	echo USING GSL
# darcmain: darccore.c darcmain.c circ.o darcNames.h darc.h agbcblas.o
# 	gcc -pthread -rdynamic -O3 -DUSEAGBBLAS -Wall -I/usr/local/include circ.o agbcblas.o -L/usr/local/lib -L/usr/lib64 -lpthread -lfftw3f -lm -lrt -ldl darcmain.c -o darcmain
# 	echo USING AGB BLAS

# darcdebug: darccore.c darcmain.c circ.o darcNames.h darc.h
# 	gcc -g -pthread -rdynamic -O3 -DUSEGSL -Wall -I/usr/local/include circ.o -L/usr/local/lib -L/usr/lib64 -lgslcblas -lpthread -lfftw3f -lm -lrt -ldl darcmain.c -o darcmain
# 	echo USING GSL

# libreconmvm.so: reconmvm.c darc.h darcNames.h agbcblas.o
# 	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DUSEAGBBLAS -fPIC -O3 -g -c -Wall -o reconmvm.o reconmvm.c
# 	gcc -O3 -shared -Wl,-soname,libreconmvm.so.1 -o libreconmvm.so.1.0.1 reconmvm.o agbcblas.o -lpthread -lc 
# 	/sbin/ldconfig -n ./
# 	rm -f libreconmvm.so
# 	ln -s  libreconmvm.so.1 libreconmvm.so
# libreconmvmgsl.so: reconmvm.c darc.h darcNames.h
# 	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -g -c -Wall -o reconmvmgsl.o reconmvm.c
# 	gcc -O3 -shared -Wl,-soname,libreconmvmgsl.so.1 -o libreconmvmgsl.so.1.0.1 reconmvmgsl.o -lpthread -lc 
# 	/sbin/ldconfig -n ./
# 	rm -f libreconmvmgsl.so
# 	ln -s  libreconmvmgsl.so.1 libreconmvmgsl.so

# libreconmvmcuda.so: reconmvm.c darc.h darcNames.h
# 	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DUSECUBLAS -fPIC -O3 -g -c -Wall -o reconmvmcuda.o reconmvm.c -I/usr/local/cuda/include
# 	gcc -O3 -shared -Wl,-soname,libreconmvmcuda.so.1 -o libreconmvmcuda.so.1.0.1 reconmvmcuda.o -lpthread -lc -L/usr/local/cuda/lib -L/usr/local/cuda/lib64 -lcublas
# 	/sbin/ldconfig -n ./
# 	rm -f libreconmvmcuda.so
# 	ln -s  libreconmvmcuda.so.1 libreconmvmcuda.so

# libreconKalman.so: reconKalman.c darc.h darcNames.h agbcblas.o
# 	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DUSEAGBBLAS -fPIC -O3 -g -c -Wall -o reconKalman.o reconKalman.c
# 	gcc -O3 -shared -Wl,-soname,libreconKalman.so.1 -o libreconKalman.so.1.0.1 reconKalman.o agbcblas.o -lpthread -lc 
# 	/sbin/ldconfig -n ./
# 	rm -f libreconKalman.so
# 	ln -s  libreconKalman.so.1 libreconKalman.so
# libreconKalman.so_gsl: reconKalman.c darc.h darcNames.h
# 	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -g -c -Wall -o reconKalman.o reconKalman.c
# 	gcc -O3 -shared -Wl,-soname,libreconKalman.so.1 -o libreconKalman.so.1.0.1 reconKalman.o -lpthread -lc 
# 	/sbin/ldconfig -n ./
# 	rm -f libreconKalman.so
# 	ln -s  libreconKalman.so.1 libreconKalman.so


# gdb:	circ.o paramNames.h core.c coremain.c
# 	gcc -ggdb -pthread -rdynamic -O3 -DUSEGSL -DNOFFT -Wall -I/usr/local/include circ.o -L/usr/local/lib -lefence -lgslcblas -lpthread -lfftw3f -lm -lrt -ldl coremain.c -o coremain


# circ.o: circ.c circ.h
# 	gcc -Wall -O3 -c circ.c -o circ.o -DUSEGSL


# coremodule.so: core.c coremodule.c circ.c circ.h
# 	rm -rf build
# 	python setup.py install --install-lib=./


# debug: core.c coremodule.c
# 	rm -r build
# 	python setup.py install --install-lib=./ --debug

# timing: core.c coremodule.c
# 	rm -r build
# 	python setup.py install --install-lib=./ --timing


# rtcDebug: core.c coremain.c
# 	gcc -Wall -DDEBUG -c circ.c -o circ.o
# 	gcc -DDEBUG -pthread -Wall -I/opt/intel/mkl/10.0.1.014/include circ.o -L/opt/intel/mkl/10.0.1.014/lib/em64t -lmkl_intel_lp64 -lmkl_intel_thread -lmkl_core -lguide -lpthread -lm -lrt  coremain.c -o coremain

# doc: rtc.dox man.tex manintro.tex
# 	latex man.tex
# 	dvipdf man
# 	doxygen rtc.dox
# 	(cd latex && make)
# 	cp latex/refman.pdf .

# cameraOld: camera.c
# 	gcc -fPIC -g -c -Wall -DOLD -o camera.o camera.c
# 	gcc -shared -Wl,-soname,librtccamera.so.1 -o librtccamera.so.1.0.1 camera.o -lc
# 	/sbin/ldconfig -n ./
# 	rm -f librtccamera.so
# 	ln -s  librtccamera.so.1 librtccamera.so
# libcamera.so: camera.c rtccamera.h
# 	gcc -fPIC -g -c -Wall -o camera.o camera.c
# 	gcc -shared -Wl,-soname,libcamera.so.1 -o libcamera.so.1.0.1 camera.o -lc
# 	/sbin/ldconfig -n ./
# 	rm -f libcamera.so
# 	ln -s  libcamera.so.1 libcamera.so
# centroiderOld: centroider.c rtccentroider.h
# 	gcc -fPIC -g -c -Wall -DOLD -o centroider.o centroider.c
# 	gcc -shared -Wl,-soname,librtccentroider.so.1 -o librtccentroider.so.1.0.1 centroider.o -lc
# 	/sbin/ldconfig -n ./
# 	rm -f librtccentroider.so
# 	ln -s  librtccentroider.so.1 librtccentroider.so
# libcentroider.so: centroider.c rtccentroider.h
# 	gcc -fPIC -g -c -Wall -o centroider.o centroider.c
# 	gcc -shared -Wl,-soname,libcentroider.so.1 -o libcentroider.so.1.0.1 centroider.o -lc
# 	/sbin/ldconfig -n ./
# 	rm -f libcentroider.so
# 	ln -s  libcentroider.so.1 libcentroider.so

# andor: andor.c
# 	gcc -fPIC -O3 -g -c -Wall -o andor.o andor.c
# 	gcc -O3 -shared -Wl,-soname,librtccamera.so.1 -o librtccamera.so.1.0.1 andor.o -lc -L/root/andor/examples/common -landor-shared-gcc-3.4.4
# 	/sbin/ldconfig -n ./
# 	rm -f librtccamera.so
# 	ln -s  librtccamera.so.1 librtccamera.so

# camfileOld: camfile.c
# 	gcc -D_GNU_SOURCE -DOLD -fPIC -O3 -g -c -Wall -o camfile.o camfile.c
# 	gcc -O3 -shared -Wl,-soname,librtccamera.so.1 -o librtccamera.so.1.0.1 camfile.o -lc 
# 	/sbin/ldconfig -n ./
# 	rm -f librtccamera.so
# 	ln -s  librtccamera.so.1 librtccamera.so
# libcamfile.so: camfile.c rtccamera.h
# 	gcc -D_GNU_SOURCE -fPIC -O3 -g -c -Wall -o camfile.o camfile.c
# 	gcc -O3 -shared -Wl,-soname,libcamfile.so.1 -o libcamfile.so.1.0.1 camfile.o -lpthread -lc 
# 	/sbin/ldconfig -n ./
# 	rm -f libcamfile.so
# 	ln -s  libcamfile.so.1 libcamfile.so

# camsocketOld: camsocket.c
# 	gcc -D_GNU_SOURCE -fPIC -O3 -g -c -Wall -o camsocket.o camsocket.c
# 	gcc -O3 -shared -Wl,-soname,librtccamera.so.1 -o librtccamera.so.1.0.1 camsocket.o -lc 
# 	/sbin/ldconfig -n ./
# 	rm -f librtccamera.so
# 	ln -s  librtccamera.so.1 librtccamera.so
# libcamsocket.so: camsocket.c rtccamera.h
# 	gcc -D_GNU_SOURCE -fPIC -O3 -g -c -Wall -o camsocket.o camsocket.c
# 	gcc -O3 -shared -Wl,-soname,libcamsocket.so.1 -o libcamsocket.so.1.0.1 camsocket.o -lc 
# 	/sbin/ldconfig -n ./
# 	rm -f libcamsocket.so
# 	ln -s  libcamsocket.so.1 libcamsocket.so

# sl240cam: sl240cam.c
# 	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -g -c -Wall -o sl240cam.o sl240cam.c -I/opt/sl240/nsl/inc -I/opt/nsl/inc 
# 	gcc -O3 -shared -Wl,-soname,librtccamera.so.1 -o librtccamera.so.1.0.1 sl240cam.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi
# 	/sbin/ldconfig -n ./
# 	rm -f librtccamera.so
# 	ln -s  librtccamera.so.1 librtccamera.so
# sl240Int32camOld: sl240Int32cam.c
# 	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DOLD -fPIC -O3 -g -c -Wall -o sl240Int32cam.o sl240Int32cam.c -I/opt/sl240/nsl/inc -I/opt/nsl/inc
# 	gcc -O3 -shared -Wl,-soname,librtccamera.so.1 -o librtccamera.so.1.0.1 sl240Int32cam.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi
# 	/sbin/ldconfig -n ./
# 	rm -f librtccamera.so
# 	ln -s  librtccamera.so.1 librtccamera.so
# libsl240Int32cam.so: sl240Int32cam.c rtccamera.h
# 	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -g -c -Wall -o sl240Int32cam.o sl240Int32cam.c -I/opt/sl240/nsl/inc -I/opt/nsl/inc
# 	gcc -O3 -shared -Wl,-soname,libsl240Int32cam.so.1 -o libsl240Int32cam.so.1.0.1 sl240Int32cam.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi
# 	/sbin/ldconfig -n ./
# 	rm -f libsl240Int32cam.so
# 	ln -s  libsl240Int32cam.so.1 libsl240Int32cam.so
# sl240centroiderOld: sl240centroider.c rtccentroider.h
# 	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DOLD -fPIC -O3 -g -c -Wall -o sl240centroider.o sl240centroider.c -I/opt/sl240/nsl/inc -I/opt/nsl/inc 
# 	gcc -O3 -shared -Wl,-soname,librtccentroider.so.1 -o librtccentroider.so.1.0.1 sl240centroider.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi
# 	/sbin/ldconfig -n ./
# 	rm -f librtccentroider.so
# 	ln -s  librtccentroider.so.1 librtccentroider.so
# libsl240centroider.so: sl240centroider.c
# 	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -g -c -Wall -o sl240centroider.o sl240centroider.c -I/opt/sl240/nsl/inc -I/opt/nsl/inc 
# 	gcc -O3 -shared -Wl,-soname,libsl240centroider.so.1 -o libsl240centroider.so.1.0.1 sl240centroider.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi
# 	/sbin/ldconfig -n ./
# 	rm -f libsl240centroider.so
# 	ln -s  libsl240centroider.so.1 libsl240centroider.so

# mirrorOld: mirror.c
# 	gcc -fPIC -O3 -g -c -Wall -DOLD -o mirror.o mirror.c
# 	gcc -O3 -shared -Wl,-soname,librtcmirror.so.1 -o librtcmirror.so.1.0.1 mirror.o -lc
# 	/sbin/ldconfig -n ./
# 	rm -f librtcmirror.so
# 	ln -s  librtcmirror.so.1 librtcmirror.so
# libmirror.so: mirror.c rtcmirror.h circ.h
# 	gcc -fPIC -O3 -g -c -Wall -o mirror.o mirror.c
# 	gcc -O3 -shared -Wl,-soname,libmirror.so.1 -o libmirror.so.1.0.1 mirror.o -lc
# 	/sbin/ldconfig -n ./
# 	rm -f libmirror.so
# 	ln -s  libmirror.so.1 libmirror.so
# libmirrorSL240.so: mirrorSL240.c circ.o rtcmirror.h darc.h circ.h
# 	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -g -c -Wall -o mirrorSL240.o mirrorSL240.c -I/opt/sl240/nsl/inc -I/opt/nsl/inc
# 	gcc -O3 -shared -Wl,-soname,libmirrorSL240.so.1 -o libmirrorSL240.so.1.0.1 mirrorSL240.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi
# 	/sbin/ldconfig -n ./
# 	rm -f libmirrorSL240.so
# 	ln -s  libmirrorSL240.so.1 libmirrorSL240.so
# libmirrorNoSL240.so: mirrorSL240.c circ.o rtcmirror.h darc.h circ.h
# 	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -DNOSL240 -fPIC -O3 -g -c -Wall -o mirrorNoSL240.o mirrorSL240.c 
# 	gcc -O3 -shared -Wl,-soname,libmirrorNoSL240.so.1 -o libmirrorNoSL240.so.1.0.1 mirrorNoSL240.o -lpthread -lc 
# 	/sbin/ldconfig -n ./
# 	rm -f libmirrorNoSL240.so
# 	ln -s  libmirrorNoSL240.so.1 libmirrorNoSL240.so

# figureOld: figure.c
# 	gcc -fPIC -O3 -g -c -Wall -DOLD -o figure.o figure.c
# 	gcc -O3 -shared -Wl,-soname,librtcfigure.so.1 -o librtcfigure.so.1.0.1 figure.o -lc
# 	/sbin/ldconfig -n ./
# 	rm -f librtcfigure.so
# 	ln -s  librtcfigure.so.1 librtcfigure.so

# libfigure.so: figure.c rtcfigure.h
# 	gcc -fPIC -O3 -g -c -Wall -o figure.o figure.c
# 	gcc -O3 -shared -Wl,-soname,libfigure.so.1 -o libfigure.so.1.0.1 figure.o -lc
# 	/sbin/ldconfig -n ./
# 	rm -f libfigure.so
# 	ln -s  libfigure.so.1 libfigure.so

# figureSL240Old: figureSL240.c
# 	gcc -DOLD -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -g -c -Wall -o figureSL240.o figureSL240.c -I/opt/nsl/inc
# 	gcc -O3 -shared -Wl,-soname,librtcfigure.so.1 -o librtcfigure.so.1.0.1 figureSL240.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi
# 	/sbin/ldconfig -n ./
# 	rm -f librtcfigure.so
# 	ln -s  librtcfigure.so.1 librtcfigure.so
# libfigureSL240.so: figureSL240.c rtcfigure.h
# 	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -g -c -Wall -o figureSL240.o figureSL240.c -I/opt/nsl/inc
# 	gcc -O3 -shared -Wl,-soname,libfigureSL240.so.1 -o libfigureSL240.so.1.0.1 figureSL240.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi
# 	/sbin/ldconfig -n ./
# 	rm -f libfigureSL240.so
# 	ln -s  libfigureSL240.so.1 libfigureSL240.so

# libfigureSL240PassThrough.so: figureSL240PassThrough.c rtcfigure.h
# 	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -g -c -Wall -o figureSL240PassThrough.o figureSL240PassThrough.c -I/opt/nsl/inc -I/Canary/src/dmc/powerdaq-3.6.20/include
# 	gcc -O3 -shared -Wl,-soname,libfigureSL240PassThrough.so.1 -o libfigureSL240PassThrough.so.1.0.1 figureSL240PassThrough.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi -lc -lpowerdaq32
# 	/sbin/ldconfig -n ./
# 	rm -f libfigureSL240PassThrough.so
# 	ln -s  libfigureSL240PassThrough.so.1 libfigureSL240PassThrough.so
# libfigureSL240SCPassThrough.so: figureSL240SCPassThrough.c rtcfigure.h
# 	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -g -c -Wall -o figureSL240SCPassThrough.o figureSL240SCPassThrough.c -I/Canary/src/SL240/sl240/inc -I/Canary/src/dmc/powerdaq-3.6.20/include
# 	gcc -O3 -shared -Wl,-soname,libfigureSL240SCPassThrough.so.1 -o libfigureSL240SCPassThrough.so.1.0.1 figureSL240SCPassThrough.o -lpthread -lc -L/Canary/src/SL240/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -L/Canary/src/SL240/sl240/bin -lfxsl -lc -lpowerdaq32
# 	/sbin/ldconfig -n ./
# 	rm -f libfigureSL240SCPassThrough.so
# 	ln -s  libfigureSL240SCPassThrough.so.1 libfigureSL240SCPassThrough.so

# libfigureSL240SC.so: figureSL240SC.c rtcfigure.h
# 	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -g -c -Wall -o figureSL240SC.o figureSL240SC.c -I/Canary/src/SL240/sl240/inc 
# 	gcc -O3 -shared -Wl,-soname,libfigureSL240SC.so.1 -o libfigureSL240SC.so.1.0.1 figureSL240SC.o -lpthread -lc -L/Canary/src/SL240/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -L/Canary/src/SL240/sl240/bin -lfxsl -lc
# 	/sbin/ldconfig -n ./
# 	rm -f libfigureSL240SC.so
# 	ln -s  libfigureSL240SC.so.1 libfigureSL240SC.so

# dmc: dmcSocketMirror.c
# 	gcc -D_GNU_SOURCE -fPIC -O3 -g -c -Wall -o dmcSocketMirror.o dmcSocketMirror.c
# 	gcc -O3 -shared -Wl,-soname,librtcmirror.so.1 -o librtcmirror.so.1.0.1 dmcSocketMirror.o -lc
# 	/sbin/ldconfig -n ./
# 	rm -f librtcmirror.so
# 	ln -s  librtcmirror.so.1 librtcmirror.so
# dmcSL240mirrorOld: dmcSL240mirror.c
# 	gcc -DOLD -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -g -c -Wall -o dmcSL240mirror.o dmcSL240mirror.c -I/opt/sl240/nsl/inc -I/opt/nsl/inc
# 	gcc -O3 -shared -Wl,-soname,librtcmirror.so.1 -o librtcmirror.so.1.0.1 dmcSL240mirror.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi
# 	/sbin/ldconfig -n ./
# 	rm -f librtcmirror.so
# 	ln -s  librtcmirror.so.1 librtcmirror.so
# libdmcSL240mirror.so: dmcSL240mirror.c rtcmirror.h circ.h
# 	echo
# 	echo "****************** Do you mean libmirrorSL240.so instead? ******************"
# 	echo
# 	gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -g -c -Wall -o dmcSL240mirror.o dmcSL240mirror.c -I/opt/sl240/nsl/inc -I/opt/nsl/inc
# 	gcc -O3 -shared -Wl,-soname,libdmcSL240mirror.so.1 -o libdmcSL240mirror.so.1.0.1 dmcSL240mirror.o -lpthread -lc -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi
# 	/sbin/ldconfig -n ./
# 	rm -f libdmcSL240mirror.so
# 	ln -s  libdmcSL240mirror.so.1 libdmcSL240mirror.so
# 	echo "This is old - doesn't work with darc"
# control_idl.py: control.idl
# 	omniidl -bpython control.idl
# nslSendData: nslSendData.c
# 	gcc -o nslSendData -DPLATFORM_UNIX  -I/opt/nsl/inc -I/opt/sl240/nsl/inc nslSendData.c -L/opt/nsl/linux-2.6/lib -L/opt/sl240/nsl/linux-2.6/lib -lnslapi
# nslRecv: nslRecv.c
# 	gcc -o nslRecv -DPLATFORM_UNIX -I/opt/sl240/nsl/inc -I/opt/nsl/inc nslRecv.c -L/opt/sl240/nsl/linux-2.6/lib -L/opt/nsl/linux-2.6/lib -lnslapi
# utilsmodule.so: utils.c
# 	python setup.py build
# 	python setup.py install --install-lib=.
# dmcPdAO32mirrorOld: dmcPdAO32mirror.c rtcmirror.h circ.h
# 	gcc -DOLD -D_GNU_SOURCE -fPIC -O3 -g -c -Wall -I/Canary/src/dmc/powerdaq-3.6.20/include -o dmcPdAO32mirror.o dmcPdAO32mirror.c
# 	gcc -O3 -shared -Wl,-soname,librtcmirror.so.1 -o librtcmirror.so.1.0.1 dmcPdAO32mirror.o -lc -lJAIFactory
# 	/sbin/ldconfig -n ./
# 	rm -f librtcmirror.so
# 	ln -s  librtcmirror.so.1 librtcmirror.so

# libdmcPdAO32mirror.so: dmcPdAO32mirror.c rtcmirror.h circ.h
# 	gcc -D_GNU_SOURCE -fPIC -O3 -g -c -Wall -I/Canary/src/dmc/powerdaq-3.6.20/include -o dmcPdAO32mirror.o dmcPdAO32mirror.c
# 	gcc -O3 -shared -Wl,-soname,libdmcPdAO32mirror.so.1 -o libdmcPdAO32mirror.so.1.0.1 dmcPdAO32mirror.o -lc -lpowerdaq32
# 	/sbin/ldconfig -n ./
# 	rm -f libdmcPdAO32mirror.so
# 	ln -s  libdmcPdAO32mirror.so.1 libdmcPdAO32mirror.so
# 	echo "This is old - doesn't work with darc"
# libmirrorPdAO32.so: mirrorPdAO32.c rtcmirror.h circ.h
# 	gcc -D_GNU_SOURCE -fPIC -O3 -g -c -Wall -I/Canary/src/dmc/powerdaq-3.6.20/include -o mirrorPdAO32.o mirrorPdAO32.c
# 	gcc -O3 -shared -Wl,-soname,libmirrorPdAO32.so.1 -o libmirrorPdAO32.so.1.0.1 mirrorPdAO32.o -lc -lpowerdaq32
# 	/sbin/ldconfig -n ./
# 	rm -f libmirrorPdAO32.so
# 	ln -s  libmirrorPdAO32.so.1 libmirrorPdAO32.so
# jaicam: jaicam.cpp rtccamera.h
# 	g++ -DOLD -Wall -g -I../include   -c -o jaicam.o jaicam.cpp
# 	g++ -Wall -g  -fPIC -shared -Wl,-soname,librtccamera.so.1 -o librtccamera.so.1.0.1 jaicam.o -lJAIFactory
# 	/sbin/ldconfig -n ./
# 	rm -f librtccamera.so
# 	ln -s  librtccamera.so.1 librtccamera.so
# libjaicam.so: jaicam.cpp rtccamera.h
# 	g++ -Wall -g -I../include   -c -o jaicam.o jaicam.cpp
# 	g++ -Wall -g  -fPIC -shared -Wl,-soname,libjaicam.so.1 -o libjaicam.so.1.0.1 jaicam.o -lJAIFactory
# 	#gcc -D_GNU_SOURCE -DPLATFORM_UNIX -fPIC -O3 -g -c -Wall -I/usr/include/JAI -o jaicam.o jaicam.c 
# 	#gcc -O3 -shared -Wl,-soname,librtccamera.so.1 -o librtccamera.so.1.0.1 jaicam.o -lpthread -lc 
# 	/sbin/ldconfig -n ./
# 	rm -f libjaicam.so
# 	ln -s  libjaicam.so.1 libjaicam.so
# libjaicam2.so: jaicam2.cpp rtccamera.h
# 	g++ -Wall -g -I../include   -c -o jaicam2.o jaicam2.cpp
# 	g++ -Wall -g  -fPIC -shared -Wl,-soname,libjaicam2.so.1 -o libjaicam2.so.1.0.1 jaicam2.o -lJAIFactory
# 	/sbin/ldconfig -n ./
# 	rm -f libjaicam2.so
# 	ln -s  libjaicam2.so.1 libjaicam2.so

# cleanc:
# 	rm -f [a-r]*.c
# 	rm -f [t-z]*.c

# sender: sender.c circ.o circ.h
# 	gcc -o sender sender.c circ.o -lrt -Wall
