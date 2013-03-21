#======================================================
# Makefile for O2Micro OZSCR Smartcardbus Reader Driver
# Author: O2Micro Inc.,
# License: See file COPYING
#======================================================

obj-m := ozscrlx.o

EXTRA_CFLAGS=

ifeq "$(debug)" "1"
	EXTRA_CFLAGS+=-DPCMCIA_DEBUG
endif

default: $(objs-m)
	$(MAKE) -C /lib/modules/`uname -r`/build SUBDIRS=`pwd` modules

install:
	cp -f ozscrlx.ko /lib/modules/`uname -r`/pcmcia

clean:
	rm -f *.o
	rm -f .*.o.d
	rm -f *.ko
	rm -f *.mod.*
	rm -f .*.cmd
	rm -rf .tmp_*
	rm -f *~	
 
