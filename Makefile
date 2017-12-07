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

# Choose which src/Makefile to use
SRCMAKEFILE=Makefile
#SRCMAKEFILE=Makefile.knl

all:
	(cd idl && make)
	(cd src && make -f $(SRCMAKEFILE))
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
	(cd src && make clean -f $(SRCMAKEFILE))
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
	(cd src && make install -f $(SRCMAKEFILE))
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
	export LD_LIBRARY_PATH=$$LD_LIBRARY_PATH:$(LIB):/usr/local/lib:/usr/local/lib64
	echo "Edit /etc/omniORB.cfg and add (or adjust) giopMaxMsgSize = 209715200 #200 MBytes"

	echo "Remember to make and install manually any custom shared libraries"

#Copy links so that your development area works like an install
#This should only ever create soft links.
installdev: all
	(cd idl && make installdev)
	(cd src && make installdev -f $(SRCMAKEFILE))
	(cd lib && make installdev)
	(cd conf && make installdev)
	(cd test && make installdev)
	(cd etc && make installdev)
	(cd include && make installdev)
	(cd bin && make installdev)
	echo export PYTHONPATH='$$PYTHONPATH':$(PWD)/lib/python > etc/local.bashrc
	echo export PATH='$$PATH':$(PWD)/bin >> etc/local.bashrc
	echo export LD_LIBRARY_PATH='$$LD_LIBRARY_PATH':$(PWD)/lib >> etc/local.bashrc

	export PYTHONPATH=$$PYTHONPATH:$(PWD)/lib/python
	export PATH=$$PATH:$(PWD)/bin
	export LD_LIBRARY_PATH=$$LD_LIBRARY_PATH:$(PWD)/lib

ubuntu1004:
	sudo apt-get install git-core emacs openssh-server python-omniorb fftw3 fftw3-dev omniidl4 omniidl4-python omniorb4-nameserver python-dev python-numpy glade python-matplotlib gsl-bin libgsl0-dev
ubuntu1004docs:
	sudo apt-get install texlive-latex-base texlive-fonts-recommended doxygen texlive
ubuntu1010:
	sudo apt-get install git-core emacs openssh-server python-omniorb fftw3 fftw3-dev omniidl omniidl-python omniorb-nameserver python-dev python-numpy glade python-matplotlib gsl-bin libgsl0-dev
ubuntu1204:
	sudo apt-get install git-core emacs openssh-server python-omniorb libfftw3-3 libfftw3-dev omniidl omniidl-python omniorb-nameserver python-dev python-numpy glade python-matplotlib python-pyinotify gsl-bin libgsl0-dev
ubuntu1404:
	sudo apt-get install git-core emacs openssh-server python-omniorb libfftw3-3 libfftw3-dev omniidl omniidl-python omniorb-nameserver python-dev python-numpy glade python-matplotlib python-pyinotify gsl-bin libgsl0-dev

ubuntu1604:
	sudo apt-get install git-core emacs openssh-server python-omniorb libfftw3-3 libfftw3-dev omniidl omniidl-python omniorb-nameserver python-dev python-numpy glade python-matplotlib python-pyinotify gsl-bin libgsl-dev

cygwin:
	apt-cyg install emacs libfftw3_3 libfftw3-devel python2-devel python2-numpy gsl libgsl-devel make
fedora12: omniORB
	yum install emacs git numpy python-devel glade3 python-matplotlib gcc fftw3-devel gcc-c++ python-inotify gsl-devel python-argparse
fedora14: omniORB
	yum install emacs git numpy python-devel glade3 python-matplotlib gcc fftw3-devel gcc-c++ python-inotify gsl-devel python-argparse

redhat5: omniORB 
	yum install emacs git numpy python-devel glade3 python-matplotlib gcc fftw3-devel gcc-c++ python-inotify gsl-devel #omniORB omniORB-devel 

scientific43: omniORB  #needs more work...
	yum install gsl-devel python-devel emacs

buildtools20:
	yum groupinstall 'Development Tools'
	yum install gcc-c++
	yum install python-devel

fedora20: buildtools20 omniORB
	yum install emacs git numpy python-devel glade3 python-matplotlib gcc fftw3-devel gcc-c++ python-inotify gsl-devel

rhel6: buildtools20  #This one is to test whether omniORB can be got from RHEL repositories (haven't actually run this yet).
	yum install omniORB omniORB-utils omniORB-servers omniORB-devel emacs git numpy python-devel glade3 python-matplotlib gcc fftw3-devel gcc-c++ python-inotify gsl-devel python-argparse

centos6: buildtools20 omniORB
	yum install emacs git numpy python-devel glade3 python-matplotlib gcc fftw3-devel gcc-c++ python-inotify gsl-devel python-argparse
	rpm -Uvh http://dl.fedoraproject.org/pub/epel/6/i386/epel-release-6-8.noarch.rpm
	yum install python-inotify

centos7: buildtools20 omniORB
	yum install emacs git numpy python-devel glade3 python-matplotlib gcc fftw3-devel gcc-c++ python-inotify gsl-devel python-argparse
	yum install python-inotify

opensuseTumbleweed: omniORB  #note, you will need to follow the instructions about updating config.sub and config.guess when you install omniORB.
	zypper install emacs git numpy python-devel glade3 python-matplotlib gcc fftw3-devel gcc-c++ python-inotify gsl-devel python-argparse make python-numpy-devel python-pyinotify


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
	(cd src && make utilsmodule.so -f $(SRCMAKEFILE))

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
	grep SINC= src/$(SRCMAKEFILE) > DARC/src/$(SRCMAKEFILE)
	grep OPTS= src/$(SRCMAKEFILE) >> DARC/src/$(SRCMAKEFILE)
	grep -A 1 "receiver:" src/$(SRCMAKEFILE) >> DARC/src/$(SRCMAKEFILE)
	grep -A 1 "circ.o:" src/$(SRCMAKEFILE) >> DARC/src/$(SRCMAKEFILE)
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

#wget http://omniorb.sourceforge.net/releases/omniORB-4.1.4.tar.gz
#wget http://omniorb.sourceforge.net/releases/omniORBpy-3.4.tar.gz
omniORB:
	wget http://sourceforge.net/projects/omniorb/files/omniORB/omniORB-4.1.4/omniORB-4.1.4.tar.gz
	tar -zxvf omniORB-4.1.4.tar.gz
	mkdir -p omniORB-4.1.4/build
	(cd omniORB-4.1.4/build && ../configure && make && make install)
	wget http://sourceforge.net/projects/omniorb/files/omniORBpy/omniORBpy-3.4/omniORBpy-3.4.tar.gz
	tar -zxvf omniORBpy-3.4.tar.gz
	mkdir -p omniORBpy-3.4/build
	(cd omniORBpy-3.4/build && ../configure && make && make install)
matplotlib:
	wget http://sourceforge.net/projects/matplotlib/files/matplotlib/matplotlib-0.99.1/matplotlib-0.99.1.2.tar.gz/download
	tar -zxvf matplotlib-0.99.1.2.tar.gz
	(cd matplotlib-0.99.1* && python setup.py build && python setup.py install)
