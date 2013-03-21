#======================================================
# Makefile for O2Micro OZSCR Smartcardbus Reader Driver
# Author: O2Micro Inc.,
# License: See file COPYING
#======================================================


CC       = gcc -DCPU_ICAP_PC -I/usr/local/include/PCSC
LD       = ld
IXOZOBJ  = ctapi.o ifdhandler.o
IXOZNAME = lib_OZSCR.so

ifeq "$(debug)" "1"
        CC+=-DPCSC_DEBUG
endif

all: ixOZ

clean:
	rm -f cttest *.o *.so *~

ixOZ: $(IXOZOBJ)
	$(LD) -shared $(IXOZOBJ) -o $(IXOZNAME)

ifdhandler.o: ifdhandler.c ctapi.h ctbcs.h
	$(CC) -c ifdhandler.c

ctapi.o: ctapi.c o2rgmap.h ozscrlx.h ctapi.h 
	$(CC) -c ctapi.c

