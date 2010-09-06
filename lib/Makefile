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
	cp Makefile $(LIB)
