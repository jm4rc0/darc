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

all: man.pdf refman.pd

man.pdf: man.tex manintro.tex
	pdflatex man.tex
	pdflatex man.tex
	pdflatex man.tex

#dvipdf man

refman.pd: rtc.dox
	echo "You may need something like sudo apt-get install texlive-fonts-recommended"
	doxygen rtc.dox
	(cd latex && make)
	cp latex/refman.pdf .


include ../Makefile.config
-include ../Makefile.config.local

install: refman.pdf man.pdf
	cp -f man.pdf $(DOC)
	cp -f refman.pdf $(DOC)

ubuntu1204:
	sudo apt-get install doxygen texlive-fonts-recommended texlive-latex-base