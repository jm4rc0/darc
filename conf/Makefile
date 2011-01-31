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
	cp config*.py $(CONF)
	cp plot*.xml $(CONF)
	cp init*.py $(CONF)
	cp rtcguirc.py $(CONF)
	cp Makefile $(CONF)
	cp shimgb1stripped_noisy.fits $(CONF)
	chmod a+r $(CONF)/config*.py
	chmod a+r $(CONF)/plot*.xml
	chmod a+r $(CONF)/init*.py
	chmod a+r $(CONF)/rtcguirc.py
	chmod a+r $(CONF)/Makefile
	chmod a+r $(CONF)/shimgb1stripped_noisy.fits

installdev:
	echo Done
clean:
	echo "Nothing to do"
