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
export ORBInitRef="NameService=corbaname::localhost"
export PYTHONPATH=$PYTHONPATH:/opt/darc/lib/python
export PATH=$PATH:/opt/darc/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/darc/lib
alias darcpy="python -i -c \"import sys,controlCorba;darc=controlCorba.controlClient(sys.argv[1] if len(sys.argv)>1 else '')\""
