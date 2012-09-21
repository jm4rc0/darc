#!/bin/sh
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
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
source /Canary/etc/bashrc
source /rtc/etc/rtc.bashrc
echo $PATH
echo $LD_LIBRARY_PATH
#wait for network to initialise...
sleep 10
while [ 1 ];do control.py ; sleep 1 ; done ;
