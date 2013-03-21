/*
 *   OZSCR PCMCIA SmartCardBus Reader Driver Kernel module for 2.6 kernel
 *   Copyright (C) 2005-2006 O2Micro Inc.

 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.

 *   This library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   Lesser General Public License for more details.

 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 *  O2Micro Inc., hereby disclaims all copyright interest in the
 *  library ozscrlx.ko written by Jeffrey Dai

 *    Module:       ozscrlx.h
 *    Author:       O2Micro Inc.,
 */

#ifndef OzSCRLx_H
#define OzSCRLx_H

#include <linux/ioctl.h>

/* Structure used to fetch reader status information */
struct ozscr_status {
	unsigned char status;		/* reader status		*/
	unsigned char os_version;	/* reader os version		*/
	unsigned char flash_mem;	/* flash memory present		*/
	unsigned char manufacturer;	/* Card manufacturer byte	*/
	unsigned char rom_sum;		/* ROM checksum			*/
	unsigned char ram_sum;		/* RAM checksum			*/
	unsigned char flash_sum;	/* Flash checksum		*/
	unsigned char reg1;		/* Smartcard register 1		*/
	unsigned char reg2;		/* Smartcard register 2		*/
	unsigned char info;		/* Clock & Control register	*/
	unsigned char card_inserted;	/* boolean for card inserted	*/
};

struct ozscr_cmd {
	unsigned char dir;		/* 00 to card, 01 from card	*/
	unsigned char cla;		/* CLA byte			*/
	unsigned char ins;		/* INS byte			*/
	unsigned char p1;		/* P1 byte			*/
	unsigned char p2;		/* P2 byte			*/
	unsigned char len;		/* LEN byte			*/
	unsigned char data[256];	/* data buffer			*/
	unsigned char status;		/* reader status		*/
	unsigned char sw1;		/* SW1 status byte		*/
	unsigned char sw2;		/* SW2 status byte		*/

};

struct ozscr_atr {
	unsigned char status;		/* status of card reader	*/
	unsigned char len;		/* length of ATR		*/
	unsigned char data[62];		/* buffer for ATR		*/
};

struct ozscr_ram {
	unsigned char ram[2016];	/* RAM area			*/
};

struct ozscr_apdu
{
   unsigned char    Command[4];
   unsigned short   LengthExpected;
   unsigned long    LengthIn;
   unsigned char    DataIn[300];
   unsigned long    LengthOut;
   unsigned char    DataOut[300];
   unsigned short   Status;
};

/* ioctl's for the OZSCR smartcardbus reader */
#define OZSCR_RESET     _IO('g', 0x01)  /* Reset CT */
#define OZSCR_PWROFF    _IO('g', 0x02)
#define OZSCR_STNDBY    _IO('g', 0x03)
#define OZSCR_OPEN      _IOR('g', 0x04, struct ozscr_apdu) /* Request ICC */
#define OZSCR_CLOSE     _IO('g', 0x05)  /* Eject ICC */
#define OZSCR_SELECT    _IO('g', 0x06)
#define OZSCR_STATUS    _IOR('g', 0x07, READER_EXTENSION) /* Get Status */
#define OZSCR_RAM       _IOR('g', 0x08, READER_EXTENSION)
#define OZSCR_CMD       _IOWR('g', 0x09, struct ozscr_apdu)

#endif /*OzSCRLx_H*/
