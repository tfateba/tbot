
openDebug:
	@echo Trying to open the Debug console with picocom:
	picocom -b 115200 /dev/ttyACM0
	@echo
	@echo Done

fullclean:
	@echo Delate all vim backup files.
	rm *~
	@echo
	@echo Done.
