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
