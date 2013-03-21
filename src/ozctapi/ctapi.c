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

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <fcntl.h>
#include <wintypes.h>

#include "debug.h"
#include "ctapi.h"
#include "o2rgmap.h"
#include "ozscrlx.h"


#define TRASHBUFLEN 270  /* Length of a buffer big enough for any return */
static unsigned char protocol;  /* Protocol of active card */
struct IO_Specs {
    int handle;
    BYTE   baud;
    BYTE   bits;
    char   parity;
    long   blocktime;
} ioport;

#define DEBUG_BUF_SIZE ((256 + 20) *3 + 10)

static char DebugBuffer[DEBUG_BUF_SIZE];

void debug_msg(char *fmt, ...)
{
    va_list argptr;

    va_start(argptr, fmt);
    vsnprintf(DebugBuffer, DEBUG_BUF_SIZE, fmt, argptr);
    va_end(argptr);

    fprintf(stderr, "%s\n", DebugBuffer);
}

/*
 * Translates return value from the Intertex reader into ctapi
 * values.
 */
int ixret2ctret(int ixret)
{
    switch (ixret)
    {
        case IX_ICC_OK:
        case IX_ICC_UNNORMAL_SW1_2: /* Let application detect SW1+SW2 itself */
        case IX_ICC_SW1_2_TOO_EARLY:
            return OK;
        case IX_ICC_ILLEGAL_CMD:
        case IX_ICC_ILLEGAL_PAR:
        case IX_ICC_BAD_LENGTH:
            return ERR_INVALID;
        case IX_ICC_CRD_REMOVED:
            return ERR_CT;
        case IX_ICC_CRD_PAR_ERRORS:
        case ICC_ERR_OPENFAIL:
        case ICC_ERR_DCBFAIL:
        case ICC_ERR_TIMOSETUPFAIL:
        case ICC_ERR_ATSCFAIL:
        case ICC_ERR_MSGFAIL:
        case ICC_ERR_MSGCORRUPT:
        case ICC_ERR_NOTOPEN:
            return ERR_TRANS;
        case ICC_ERR_BUFTOOSMALL:
            return ERR_MEMORY;
        default: /* If nothing else fits */
            return ERR_CT;
    }
}

/* Initializes the port on which the reader resides */
int CT_init ( unsigned short Ctn,  unsigned short pn )
{
    BOOL BretVal;        /* Return value from IO_InitializePort() */
    int  IretVal;        /* Return for this function */  

    READER_EXTENSION RdrExt;
    int handle = ioport.handle;

    DebugLogA("CT_init enter");

    switch( pn )
    {
        case PORT_COM1:
        case PORT_COM2:
        case PORT_COM3:
        case PORT_COM4:
        case PORT_Printer:
        case PORT_Modem:
            BretVal = ICC_ERR_OPENFAIL;
            break;
        case PORT_Pcmcia:
            DebugLogA("Try to open channel dev/ozscrlx");
            BretVal = ICC_Open_Channel("/dev/ozscrlx");
            break;
        default:
            BretVal = ICC_ERR_OPENFAIL;
            break;
    }

    if (BretVal)
    {
        IretVal = ERR_MEMORY;        /* Could not allocate port */
    }
    else
    {
        IretVal = OK;
    }

    DebugLogB("CT_init exit (%d)", IretVal);
    return IretVal;  
}

int CT_close( unsigned short Ctn )
{

    if (ICC_Close_Channel() == TRUE)
    {
        return OK;   
    }
    else
    {
        return ERR_CT;
    }
}

/* 
 * Sends/Receives Data to/from the Reader
 * Parameters:
 *  ctn: logical cardterminal number.
 *  dad: destination address.
 *  sad: source address.
 *   lc: cmd length in bytes.
 *  cmd: ICC cmd or CT cmd.
 *   lr: passing of the max. buffer size of the rsp and
 *       return of the actual length of the rsp.
 *  rsp: rsp to the cmd.
 * Returns:
 *      OK or CT-API error code.
 */ 
int CT_data( unsigned short ctn, unsigned char *dad, unsigned char *sad,
    unsigned short lc, unsigned char *cmd, unsigned short *lr,
    unsigned char *rsp )
{
    int IretVal = ERR_INVALID;     /* Return Value. Assume error.    */
    int IXret;                     /* Return from IX API call */
    int lrtmp;                     /* Saves return buffer length temporary */
    unsigned char buf[TRASHBUFLEN]; /* Trash return buffer */
    unsigned char ixcmd;           /* Command echo return */
    unsigned char ixpar;           /* Parameter return */
    int handle = ioport.handle;
    struct ozscr_atr anatr;
    struct ozscr_status anstatus;
    READER_EXTENSION RdrExt;
    struct ozscr_apdu apdu;

    int i;

    if ( *dad == 1 ) 
    {
        /* This command goes to the reader  */
        /* Request ICC  - Turns on the reader/card */
        if ( (cmd[0] == 0x20) && (cmd[1] == 0x11) )
        {
#ifdef PCSC_DEBUG
            DebugLogA("CT-Api: CT_data: pcPowerUp");

            DebugLogB("CT-Api: lc = %d", lc);
            for(i = 0; i < lc; i ++)
            {
                DebugLogC("CT-Api: cmd[%d] = %x ", i, cmd[i]);
            }
#endif

            if (*lr < 2)
                return ERR_MEMORY; /* Return buffer too small */
            *lr -= 2;
            IretVal = ioctl(handle, OZSCR_OPEN, &apdu);
            *lr = (ULONG)apdu.LengthOut;
            memcpy( rsp, apdu.DataOut, apdu.LengthOut );
            if ( IretVal == STATUS_SUCCESS )
            {
                rsp[(*lr)++] = 0x90;
                rsp[(*lr)++] = 0x01;
            }
            else
            {
                rsp[0] = 0x64;
                rsp[1] = 0x00;
            }
        /* Resets the Card/Terminal and returns Atr */
        }
        else if ( (cmd[0] == 0x20) && (cmd[1] == 0x12) )
        {
#ifdef PCSC_DEBUG
            DebugLogA("CT-Api: CT_data: pcReset");

            DebugLogB("CT-Api: lc = %d", lc);
            for(i = 0; i < lc; i ++)
            {
                DebugLogC("CT-Api: cmd[%d] = %x ", i, cmd[i]);
            }
#endif
            if (*lr < 2) return ERR_MEMORY; /* Return buffer too small */
            *lr -= 2; /* Reserve 2 bytes for SW1+SW2 */
            IretVal = ioctl(handle, OZSCR_OPEN, &apdu);
            *lr = (ULONG)apdu.LengthOut;
            memcpy( rsp, apdu.DataOut, apdu.LengthOut );
            rsp[(*lr)++] = 0x90;
            rsp[(*lr)++] = 0x00;
           /* Get Status - Gets reader status */
        }
        else if ( (cmd[0] == 0x20) && (cmd[1] == 0x13) )
        {
            if (*lr < 3) return ERR_MEMORY; /* Return buffer too small */
            /* Ask for status */
            IXret = ioctl(handle, OZSCR_STATUS, &RdrExt);
            ixpar = (unsigned char)(IXret) & 0xC0;
            ixpar = ixpar >> 6;
            *lr = 3;
            IretVal = STATUS_SUCCESS;
            switch (ixpar)
            {
                case 2: /* Card inserted, but inactive */
                case 3: /* Card there, and active */
                    rsp[0] = 4;
                    break;
                case 0: /* Card not inserted */
                    rsp[0] = 0;
                    break;
                case 1: /* Card not inserted, and inactive */
                    rsp[0] = 1;
                    break;
                default: /* Should not happen. Report as error. */
                    IretVal = IX_ICC_ILLEGAL_PAR;
            }
            rsp[1] = 0x90;
            rsp[2] = 0x00;
            /* Eject ICC - Deactivates Reader  */
        }
        else if ( (cmd[0] == 0x20) && (cmd[1] == 0x15) )
        {
#ifdef PCSC_DEBUG
            DebugLogA("CT-Api: CT_data: Eject ICC");
            DebugLogB("CT-Api: lc = %d", lc);
            for(i = 0; i < lc; i ++)
            {
            DebugLogC("CT-Api: cmd[%d] = %x ", i, cmd[i]);
            }
#endif
            if (*lr < 2)
                return ERR_MEMORY; /* Return buffer too small */
            IretVal = ioctl(handle, OZSCR_CLOSE);
            *lr = 2;
            rsp[1] = 0x90; /* successful */
            rsp[2] = 0x00;
        }
        else if ((cmd[0] == 0x20) && (cmd[1] == 0x16))
        {
#ifdef PCSC_DEBUG
            DebugLogA("CT-Api: CT_data: Set Protocol");
#endif
            if (*lr < 2)
            {
                return ERR_MEMORY; /* Return buffer too small */
            }

            apdu.LengthIn = (ULONG)lc;
            memcpy(&apdu.DataIn, cmd, apdu.LengthIn);
            IretVal = ioctl(handle, OZSCR_SELECT, &apdu);
            *lr = 2;
            rsp[1] = 0x90; /* successful */
            rsp[2] = 0x00;
        }
        else
        {
#ifdef PCSC_DEBUG
            DebugLogA("CT-Api: CT_data: dad=1 Unsupport Command");
            DebugLogD("cmd = [%X], [%X], [%X]\n", cmd[0], cmd[1], cmd[2]);
#endif
            apdu.LengthIn = (ULONG)lc;
            memcpy( apdu.DataIn, cmd, apdu.LengthIn );
            IretVal = ioctl(handle, OZSCR_CMD, &apdu);
            *lr = (ULONG)apdu.LengthOut;
            memcpy( rsp, apdu.DataOut, apdu.LengthOut );
            /* Use ICC_Do_Card_Command() with parameters here, if needed. */
            /* CHANGE:: Write directly to the reader. */
        }
    }
    else if ( *dad == 0 )
    { /* This command goes to the card */
#ifdef PCSC_DEBUG
        DebugLogB("CT-Api: CT_data: dad=0, lc=%d", lc);
        DebugLogA("CT-Api: data == 0");
        DebugLogB("CT-Api: lc = %d", lc);
        for(i = 0; i < lc; i ++)
        {
            DebugLogC("CT-Api: cmd[%d] = %x ", i, cmd[i]);
        }
#endif
        apdu.LengthIn = (ULONG)lc;
        memcpy( apdu.DataIn, cmd, apdu.LengthIn );
        IretVal = ioctl(handle, OZSCR_CMD, &apdu);
        *lr = (ULONG)apdu.LengthOut;
        memcpy( rsp, apdu.DataOut, apdu.LengthOut );
    }
    else
    {

#ifdef PCSC_DEBUG
        DebugLogA("CT-Api: CT_data: dad!=0 and dad!=1");
#endif
        IretVal = ERR_INVALID;              /* Invalid SAD/DAD Address */

    }
#ifdef PCSC_DEBUG
    if(cmd[1]!=0x13)
        DebugLogC("CT-Api: rv=%d, lr=%d cmd[1]!=0x13", IretVal, *lr);
#endif

    if (IretVal != OK)
        *lr = 0;

    return IretVal;
}

/*
  Connects to modem/reader
  In:               port    String with filename of port
                            (example: "/dev/ttyS1")
  Returns:          0 if OK
                    Otherwise error code.
*/
int ICC_Open_Channel(char *port)
{
    int handle;
    handle = open(port, O_RDWR | O_NOCTTY);  
    if (handle < 0)    
        return ICC_ERR_OPENFAIL;
    ioport.handle = handle;
    return 0;
}

/*
  Closes connection to modem/reader
  Returns:          0
*/
int ICC_Close_Channel()
{
    int handle = ioport.handle;
    if ( close(handle) == 0 ) 
        return TRUE;
    else
        return FALSE;
}
