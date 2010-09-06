all: man.pdf refman.pd

man.pdf: man.tex manintro.tex
	latex man.tex
	dvipdf man

refman.pd: rtc.dox
	doxygen rtc.dox
	(cd latex && make)
	cp latex/refman.pdf .


include ../Makefile.config

install: refman.pdf man.pdf
	cp -f man.pdf $(DOC)
	cp -f refman.pdf $(DOC)
