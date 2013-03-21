/*
 *   OZSCR PCMCIA SmartCardBus Reader Driver library for pcsc-lite
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
 *  library lib_OZSCR.so written by Jeffrey Dai

 *    Module:       ctapi.h
 *    Author:       O2Micro Inc.,
 */

#ifndef _ctapi_h_
#define _ctapi_h_

#ifdef __cplusplus
extern "C" {
#endif

int CT_init (
      unsigned short	Ctn,                  /* Terminal Number */
      unsigned short	pn                    /* Port Number */
      );

int CT_close(
      unsigned short	Ctn                  /* Terminal Number */
      );

int CT_data( 
       unsigned short	ctn,                 /* Terminal Number */
       unsigned char 	*dad,               /* Destination */
       unsigned char 	*sad,               /* Source */
       unsigned short	lc,                 /* Length of command */
       unsigned char 	*cmd,               /* Command/Data Buffer */
       unsigned short	*lr,                /* Length of Response */
       unsigned char 	*rsp                /* Response */
       );


/*
 *  API error codes
 */
#define ICC_ERR_OPENFAIL       1000
#define ICC_ERR_DCBFAIL        1001
#define ICC_ERR_TIMOSETUPFAIL  1002
#define ICC_ERR_ATSCFAIL       1003
#define ICC_ERR_MSGFAIL        1004
#define ICC_ERR_MSGCORRUPT     1005
#define ICC_ERR_BUFTOOSMALL    1006
#define ICC_ERR_NOTOPEN        1007
#define ICC_ERR_NOCMDMODE      1009

#define IX_ICC_OK                 0
#define IX_ICC_DONE_OK          126
#define IX_ICC_CRD_REMOVED      128
#define IX_ICC_CRD_NO_RESPONSE  129
#define IX_ICC_CRD_PAR_ERRORS   130
#define IX_ICC_WRONG_CRD_TYPE   131
#define IX_ICC_ILLEGAL_CMD      133
#define IX_ICC_UNNORMAL_SW1_2   135
#define IX_ICC_ILLEGAL_PAR      136
#define IX_ICC_SW1_2_TOO_EARLY  141
#define IX_ICC_CRD_IS_T0        143
#define IX_ICC_CRD_IS_T1        144
#define IX_ICC_BAD_LENGTH       146


#define OK               0               /* Success */
#define ERR_INVALID     -1               /* Invalid Data */
#define ERR_CT          -8               /* CT Error */
#define ERR_TRANS       -10              /* Transmission Error */
#define ERR_MEMORY      -11              /* Memory Allocate Error */
#define ERR_HTSI        -128             /* HTSI Error */

#define PORT_COM1	   0             /* COM 1 */
#define PORT_COM2	   1             /* COM 2 */
#define PORT_COM3	   2             /* COM 3 */
#define PORT_COM4	   3             /* COM 4 */
#define PORT_Printer   4             /* Printer Port (MAC) */
#define PORT_Modem     5             /* Modem Port (MAC)   */
#define PORT_LPT1	   6             /* LPT 1 */
#define PORT_LPT2	   7             /* LPT 2 */
#define PORT_Pcmcia    8             /* Pcmcia Port */

#ifdef __cplusplus
}
#endif

#endif
