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
all:

include ../Makefile.config

install:
	cp python/buffer.py $(PY)
	cp python/Check.py $(PY)
	cp python/ConnObj.py $(PY)
	cp python/controlCorba.py $(PY)
	cp python/control.py $(PY)
	cp python/correlation.py $(PY)
	cp python/dataSwitchClient.py $(PY)
	cp python/FITS.py $(PY)
	cp python/logread.py $(PY)
	cp python/overlayMaker.py $(PY)
	cp python/plot.py $(PY)
	cp python/PSuser.py $(PY)
	cp python/recvStream.py $(PY)
	cp python/rtcgui.py $(PY)
	cp python/Saver.py $(PY)
	cp python/sendStream.py $(PY)
	cp python/serialise.py $(PY)
	cp python/SockConn.py $(PY)
	cp python/socketCam.py $(PY)
	cp python/socketDM.py $(PY)
	cp python/spot.py $(PY)
	cp python/startStreams.py $(PY)
	cp python/stdoutlog.py $(PY)
	cp python/tel.py $(PY)
	cp python/zernike.py $(PY)
	cp python/plotxml.py $(PY)
	cp python/nodestream.py $(PY)
	python -c "import compileall; compileall.main()" $(PY)
	chmod a+x $(PY)/control.py
	chmod a+x $(PY)/control.pyc
	chmod a+x $(PY)/startStreams.py
	chmod a+x $(PY)/startStreams.pyc
	chmod a+x $(PY)/sendStream.py
	chmod a+x $(PY)/sendStream.pyc
	chmod a+x $(PY)/recvStream.py
	chmod a+x $(PY)/recvStream.pyc
	chmod a+x $(PY)/rtcgui.py
	chmod a+x $(PY)/rtcgui.pyc
	chmod a+x $(PY)/plot.py
	chmod a+x $(PY)/plot.pyc
	chmod a+x $(PY)/socketCam.py
	chmod a+x $(PY)/socketCam.pyc
	chmod a+x $(PY)/socketDM.py
	chmod a+x $(PY)/socketDM.pyc
	chmod a+x $(PY)/nodestream.py
	chmod a+x $(PY)/nodestream.pyc
	ln -fs $(PY)/control.py $(BIN)/
	ln -fs $(PY)/control.pyc $(BIN)/
	ln -fs $(PY)/startStreams.py $(BIN)/
	ln -fs $(PY)/startStreams.pyc $(BIN)/
	ln -fs $(PY)/sendStream.py $(BIN)/
	ln -fs $(PY)/sendStream.pyc $(BIN)/
	ln -fs $(PY)/recvStream.py $(BIN)/
	ln -fs $(PY)/recvStream.pyc $(BIN)/
	ln -fs $(PY)/rtcgui.py $(BIN)/
	ln -fs $(PY)/rtcgui.pyc $(BIN)/
	ln -fs $(PY)/plot.py $(BIN)/
	ln -fs $(PY)/plot.pyc $(BIN)/
	ln -fs $(PY)/socketCam.py $(BIN)/
	ln -fs $(PY)/socketCam.pyc $(BIN)/
	ln -fs $(PY)/socketDM.py $(BIN)/
	ln -fs $(PY)/socketDM.pyc $(BIN)/
	ln -fs $(PY)/nodestream.py $(BIN)/
	ln -fs $(PY)/nodestream.pyc $(BIN)/
	cp Makefile $(LIB)

installdev:
	chmod a+x $(PWD)/python/control.py
	chmod a+x $(PWD)/python/startStreams.py
	chmod a+x $(PWD)/python/sendStream.py
	chmod a+x $(PWD)/python/recvStream.py
	chmod a+x $(PWD)/python/rtcgui.py
	chmod a+x $(PWD)/python/plot.py
	chmod a+x $(PWD)/python/socketCam.py
	chmod a+x $(PWD)/python/socketDM.py
	chmod a+x $(PWD)/python/nodestream.py
	ln -fs $(PWD)/python/control.py $(PWD)/../bin/
	ln -fs $(PWD)/python/control.pyc $(PWD)/../bin/
	ln -fs $(PWD)/python/startStreams.py $(PWD)/../bin/
	ln -fs $(PWD)/python/startStreams.pyc $(PWD)/../bin/
	ln -fs $(PWD)/python/sendStream.py $(PWD)/../bin/
	ln -fs $(PWD)/python/sendStream.pyc $(PWD)/../bin/
	ln -fs $(PWD)/python/recvStream.py $(PWD)/../bin/
	ln -fs $(PWD)/python/recvStream.pyc $(PWD)/../bin/
	ln -fs $(PWD)/python/rtcgui.py $(PWD)/../bin/
	ln -fs $(PWD)/python/rtcgui.pyc $(PWD)/../bin/
	ln -fs $(PWD)/python/plot.py $(PWD)/../bin/
	ln -fs $(PWD)/python/plot.pyc $(PWD)/../bin/
	ln -fs $(PWD)/python/socketCam.py $(PWD)/../bin/
	ln -fs $(PWD)/python/socketCam.pyc $(PWD)/../bin/
	ln -fs $(PWD)/python/socketDM.py $(PWD)/../bin/
	ln -fs $(PWD)/python/socketDM.pyc $(PWD)/../bin/
	ln -fs $(PWD)/python/nodestream.py $(PWD)/../bin/
	ln -fs $(PWD)/python/nodestream.pyc $(PWD)/../bin/
