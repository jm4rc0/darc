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
-include ../Makefile.config.local

install:
	cp darcmagic $(BIN)
	cp darctalk $(BIN)
	cp deletesem $(BIN)
	cp rtcgui.glade $(BIN)
	cp runcontrol.sh $(BIN)
	cp runrecv.sh $(BIN)
	cp runrtc.sh $(BIN)
	cp runsend.sh $(BIN)
	cp Makefile $(BIN)
	chmod a+rx $(BIN)/darcmagic
	chmod a+rx $(BIN)/darctalk
	chmod a+rx $(BIN)/deletesem
	chmod a+rx $(BIN)/runcontrol.sh
	chmod a+rx $(BIN)/runrecv.sh
	chmod a+rx $(BIN)/runrtc.sh
	chmod a+rx $(BIN)/runsend.sh
installdev:
	echo Done
clean:
	echo nowt to do here.
