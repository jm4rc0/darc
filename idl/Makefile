include ../Makefile.config
-include ../Makefile.config.local

control_idl.py: control.idl
	omniidl -bpython control.idl
install: control_idl.py
	cp control_idl.py $(PY)
	chmod a+r $(PY)/control_idl.py
	cp Makefile $(IDL)
	cp control.idl $(IDL)
installdev: control_idl.py
	ln -sf $(PWD)/control_idl.py $(PWD)/../lib/python
clean: 
	rm -f control_idl.py
	rm -rf RTC/
	rm -rf RTC__POA/
