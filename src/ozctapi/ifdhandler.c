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

 *    Module:       ifdhandler.c
 *    Author:       O2Micro Inc.,
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define IFDHANDLERv2
#include <ifdhandler.h>

#include "debug.h"
#include "ctapi.h"
#include "ctbcs.h"


/*
 * Not exported constants definition
 */

/* Maximum number of readers handled */
#define IFDH_MAX_READERS	4

/* Maximum number of slots per reader handled */
#define IFDH_MAX_SLOTS		2

/*
 * Not exported data types definition
 */
typedef struct _SLOT_DESC
{
    UCHAR ATR[MAX_ATR_SIZE];
    DWORD ATR_Length;
} SLOT_DESC;

typedef struct
{
    DEVICE_CAPABILITIES device_capabilities;
    SLOT_DESC icc_state;
    PROTOCOL_OPTIONS protocol_options;
} IFDH_Status;

/*
 * Not exported variables definition
 */

/* Matrix that stores status information of all slots and readers */
IFDH_Status *ifdh_status[IFDH_MAX_READERS][IFDH_MAX_SLOTS] =
{
    { NULL, NULL},
    { NULL, NULL},
    { NULL, NULL},
    { NULL, NULL},
};

/* 
 * Exported functions definition
 */


RESPONSECODE
IFDHCreateChannel (DWORD Lun, DWORD Channel)
{
  /* Lun - Logical Unit Number, use this for multiple card slots
     or multiple readers. 0xXXXXYYYY -  XXXX multiple readers,
     YYYY multiple slots. The resource manager will set these 
     automatically.  By default the resource manager loads a new
     instance of the driver so if your reader does not have more than
     one smartcard slot then ignore the Lun in all the functions.
     Future versions of PC/SC might support loading multiple readers
     through one instance of the driver in which XXXX would be important
     to implement if you want this.
  */

  /* Channel - Channel ID.  This is denoted by the following:
     0x000001 - /dev/pcsc/1
     0x000002 - /dev/pcsc/2
     0x000003 - /dev/pcsc/3

     USB readers may choose to ignore this parameter and query 
     the bus for the particular reader.
  */

  /* This function is required to open a communications channel to the 
     port listed by Channel.  For example, the first serial reader on COM1 would
     link to /dev/pcsc/1 which would be a sym link to /dev/ttyS0 on some machines
     This is used to help with intermachine independance.
     Once the channel is opened the reader must be in a state in which it is possible
     to query IFDHICCPresence() for card status.

     returns:
     IFD_SUCCESS
     IFD_COMMUNICATION_ERROR
  */

    char ret;
    unsigned short ctn, pn, slot;

#ifdef PCSC_DEBUG
    DebugLogC("Lun %X, Channel %X\n", Lun, Channel);
#endif
    ctn = ((unsigned short) (Lun & 0x0000FFFF)) % IFDH_MAX_READERS;
    slot = ((unsigned short) (Lun >> 8)) % IFDH_MAX_SLOTS;

    if (ifdh_status[ctn][slot] != NULL)
        return IFD_SUCCESS;

    /* Conversion of old-style ifd-hanler 1.0 CHANNELID new style */
    if (Channel == 0x0103F8)
        Channel = 0x000001;
    else if (Channel == 0x0102F8)
        Channel = 0x000002;
    else if (Channel == 0x0103E8)
        Channel = 0x000003;
    else if (Channel == 0x0102E8)
        Channel = 0x000004;
    else if (Channel == 0xF10000)
        Channel = 0x000009;

    pn = (unsigned short)Channel - 1;
    ret = CT_init(ctn, pn);

    if (ret != OK)
        return IFD_COMMUNICATION_ERROR;

    for (slot = 0; slot < IFDH_MAX_SLOTS; slot++)
    {
        ifdh_status[ctn][slot] = (IFDH_Status *) malloc (sizeof (IFDH_Status));

        if (ifdh_status[ctn][slot] != NULL)
            memset (ifdh_status[ctn][slot], 0, sizeof(IFDH_Status));
    }

    return IFD_SUCCESS;
}

RESPONSECODE
IFDHCloseChannel (DWORD Lun)
{
  /* This function should close the reader communication channel
     for the particular reader.  Prior to closing the communication channel
     the reader should make sure the card is powered down and the terminal
     is also powered down.

     returns:

     IFD_SUCCESS
     IFD_COMMUNICATION_ERROR
  */

    char ret;
    unsigned short ctn, slot;

    ctn = ((unsigned short) (Lun & 0x0000FFFF)) % IFDH_MAX_READERS;
    ret = CT_close (ctn);

    if (ret != OK)
        return IFD_COMMUNICATION_ERROR;

    for (slot = 0; slot < IFDH_MAX_SLOTS; slot++)
    {
        if (ifdh_status[ctn][slot] != NULL)
        {
            free (ifdh_status[ctn][slot]);
            ifdh_status[ctn][slot] = NULL;
        }
    }

    return IFD_SUCCESS;
}

RESPONSECODE
IFDHGetCapabilities (DWORD Lun, DWORD Tag, PDWORD Length, PUCHAR Value)
{
  /* This function should get the slot/card capabilities for a particular
     slot/card specified by Lun.  Again, if you have only 1 card slot and don't mind
     loading a new driver for each reader then ignore Lun.

     Tag - the tag for the information requested
         example: TAG_IFD_ATR - return the Atr and it's size (required).
         these tags are defined in ifdhandler.h

     Length - the length of the returned data
     Value  - the value of the data

     returns:

     IFD_SUCCESS
     IFD_ERROR_TAG
  */

    unsigned short ctn, slot;

    ctn = ((unsigned short) (Lun & 0x0000FFFF)) % IFDH_MAX_READERS;
    slot = ((unsigned short) (Lun >> 8)) % IFDH_MAX_SLOTS;

    if (ifdh_status[ctn][slot] == NULL)
        return IFD_ICC_NOT_PRESENT;

    if (Tag == TAG_IFD_ATR)
    {
        (*Length) = ifdh_status[ctn][slot]->icc_state.ATR_Length;
        memcpy (Value, ifdh_status[ctn][slot]->icc_state.ATR, (*Length));
    }
    else
        return IFD_ERROR_TAG;

    return IFD_SUCCESS;
}

RESPONSECODE
IFDHSetCapabilities (DWORD Lun, DWORD Tag, DWORD Length, PUCHAR Value)
{
  /* This function should set the slot/card capabilities for a particular
     slot/card specified by Lun.  Again, if you have only 1 card slot and don't mind
     loading a new driver for each reader then ignore Lun.

     Tag - the tag for the information needing set

     Length - the length of the returned data
     Value  - the value of the data

     returns:

     IFD_SUCCESS
     IFD_ERROR_TAG
     IFD_ERROR_SET_FAILURE
     IFD_ERROR_VALUE_READ_ONLY
  */

    return IFD_NOT_SUPPORTED;
}


RESPONSECODE
IFDHSetProtocolParameters (DWORD Lun, DWORD Protocol,
                           UCHAR Flags, UCHAR PTS1, UCHAR PTS2, UCHAR PTS3)
{
  /* This function should set the PTS of a particular card/slot using
     the three PTS parameters sent

     Protocol  - 0 .... 14  T=0 .... T=14
     Flags     - Logical OR of possible values:
     IFD_NEGOTIATE_PTS1 IFD_NEGOTIATE_PTS2 IFD_NEGOTIATE_PTS3
     to determine which PTS values to negotiate.
     PTS1,PTS2,PTS3 - PTS Values.

     returns:

     IFD_SUCCESS
     IFD_ERROR_PTS_FAILURE
     IFD_COMMUNICATION_ERROR
     IFD_PROTOCOL_NOT_SUPPORTED
  */
    /* define variables*/
    char           ret;
    unsigned short ctn;
    unsigned short slot;
    unsigned short lc;
    unsigned short lr;
    UCHAR          cmd[10];
    UCHAR          rsp[256];
    UCHAR          sad;
    UCHAR          dad;

    RESPONSECODE   response = IFD_SUCCESS;

    DebugLogA("IFDHSetProtocolParameters() Start! \n");
    DebugLogB("Set protocol type to %x \n", Protocol);


    /* check HW resource. */
    ctn = ((unsigned short) (Lun & 0x0000FFFF)) % IFDH_MAX_READERS;
    slot = ((unsigned short) (Lun >> 8)) % IFDH_MAX_SLOTS;

    if (ifdh_status[ctn][slot] == NULL)
    {
        response = IFD_ICC_NOT_PRESENT;
        goto IFDHSetProtocolParameters_exit;
    }

    /* check inputting variables */
    if ((Protocol  != 0x01) && (Protocol != 0x02))
    {
        DebugLogA("Input wrong protocol type! \n");
        response = IFD_PROTOCOL_NOT_SUPPORTED;
        goto IFDHSetProtocolParameters_exit;      
    }

    /* set protocol */
    cmd[0] = CTBCS_CLA;
    cmd[1] = CTBCS_INS_PROTOCOL;
    cmd[2] = 0x00;
    cmd[3] = 0x00;
    cmd[4] = 0x05;
    cmd[5] = (UCHAR)Protocol;
    cmd[6] = Flags;
    cmd[7] = PTS1;
    cmd[8] = PTS2;
    cmd[9] = PTS3;

    dad = 0x01;
    sad = 0x02;
    lr = 256;
    lc = 10;

    ret = CT_data(ctn, &dad, &sad, lc, cmd, &lr, rsp);

    /* check result. */
    if (ret != OK)
    {
        lr = 0;
        response = IFD_ERROR_PTS_FAILURE;
        goto IFDHSetProtocolParameters_exit;
    }

IFDHSetProtocolParameters_exit:
#ifdef PCSC_DEBUG
    DebugLogA("Exit IFDHSetProtocolParameters! \n");
#endif
    return response;
}


RESPONSECODE
IFDHPowerICC (DWORD Lun, DWORD Action, PUCHAR Atr, PDWORD AtrLength)
{
  /* This function controls the power and reset signals of the smartcard reader
     at the particular reader/slot specified by Lun.


     Action - Action to be taken on the card.

     IFD_POWER_UP - Power and reset the card if not done so 
     (store the ATR and return it and it's length).

     IFD_POWER_DOWN - Power down the card if not done already 
     (Atr/AtrLength should
     be zero'd)

    IFD_RESET - Perform a quick reset on the card.  If the card is not powered
     power up the card.  (Store and return the Atr/Length)


     Atr - Answer to Reset of the card.  The driver is responsible for caching
     this value in case IFDHGetCapabilities is called requesting the ATR and it's
     length.  This should not exceed MAX_ATR_SIZE.

     AtrLength - Length of the Atr.  This should not exceed MAX_ATR_SIZE.

     Notes:

     Memory cards without an ATR should return IFD_SUCCESS on reset
     but the Atr should be zero'd and the length should be zero

     Reset errors should return zero for the AtrLength and return 
     IFD_ERROR_POWER_ACTION.

     returns:

     IFD_SUCCESS
     IFD_ERROR_POWER_ACTION
     IFD_COMMUNICATION_ERROR
     IFD_NOT_SUPPORTED
  */

    char ret; 
    unsigned short ctn, slot, lc, lr;
    UCHAR cmd[5], rsp[256], sad, dad;

    ctn = ((unsigned short) (Lun & 0x0000FFFF)) % IFDH_MAX_READERS;
    slot = ((unsigned short) (Lun >> 8)) % IFDH_MAX_SLOTS;

    if (ifdh_status[ctn][slot] == NULL)
        return IFD_ICC_NOT_PRESENT;

    if (Action == IFD_POWER_UP)
    {
#ifdef PCSC_DEBUG
        DebugLogA("In Function IFDHPowerICC Action:IFD_POWER_UP\n");
#endif
        cmd[0] = CTBCS_CLA;
        cmd[1] = CTBCS_INS_REQUEST;
        cmd[2] = (UCHAR) (slot + 1);
        cmd[3] = CTBCS_P2_REQUEST_GET_ATR;
        cmd[4] = 0x00;

        dad = 0x01;
        sad = 0x02;
        lr = 256;
        lc = 5;

        ret = CT_data (ctn, &dad, &sad, 5, cmd, &lr, rsp);
#ifdef PCSC_DEBUG
        DebugLogB("In Function IFDHPowerICC, CT_data=%d\n", ret);
#endif

        if ((ret != OK) || (lr < 2))
            return IFD_COMMUNICATION_ERROR;

        ifdh_status[ctn][slot]->icc_state.ATR_Length = (DWORD) lr - 2;
        memcpy (ifdh_status[ctn][slot]->icc_state.ATR, rsp, lr-2);

        (*AtrLength) = (DWORD) lr - 2;
        memcpy (Atr, rsp, lr - 2);
    }
    else if (Action == IFD_POWER_DOWN)
    {
#ifdef PCSC_DEBUG
        DebugLogA("In Function IFDHPowerICC Action:IFD_POWER_DOWN\n");
#endif
        cmd[0] = CTBCS_CLA;
        cmd[1] = CTBCS_INS_EJECT;
        cmd[2] = (UCHAR) (slot + 1);
        cmd[3] = 0x00;
        cmd[4] = 0x00;

        dad = 0x01;
        sad = 0x02;
        lr = 256;
        lc = 5;

        ret = CT_data (ctn, &dad, &sad, 5, cmd, &lr, rsp);
#ifdef PCSC_DEBUG
	DebugLogB("In Function IFDHPowerICC, CT_data=%d\n", ret);
#endif
        if (ret != OK)
            return IFD_COMMUNICATION_ERROR;

        ifdh_status[ctn][slot]->icc_state.ATR_Length = 0;
        memset (ifdh_status[ctn][slot]->icc_state.ATR, 0, MAX_ATR_SIZE);

        (*AtrLength) = 0;
    }
    else if (Action == IFD_RESET)
    {
#ifdef PCSC_DEBUG
        DebugLogA("In Function IFDHPowerICC Action:IFD_RESET\n");
#endif
        cmd[0] = CTBCS_CLA;
        cmd[1] = CTBCS_INS_RESET;
        cmd[2] = (UCHAR) (slot + 1);
        cmd[3] = CTBCS_P2_RESET_GET_ATR;
        cmd[4] = 0x00;

        dad = 0x01;
        sad = 0x02;
        lr = 256;
        lc = 5;

        ret = CT_data (ctn, &dad, &sad, 5, cmd, &lr, rsp);
#ifdef PCSC_DEBUG
        DebugLogB("In Function IFDHPowerICC, CT_data=%d\n", ret);
#endif

        if ((ret != OK) || (lr < 2))
            return IFD_ERROR_POWER_ACTION;

        ifdh_status[ctn][slot]->icc_state.ATR_Length = (DWORD) lr - 2;
        memcpy (ifdh_status[ctn][slot]->icc_state.ATR,rsp, lr - 2);

        (*AtrLength) = (DWORD) lr - 2;
        memcpy (Atr, rsp, lr - 2);
    }
    else
        return IFD_NOT_SUPPORTED;

    return IFD_SUCCESS;
}

RESPONSECODE
IFDHTransmitToICC (DWORD Lun, SCARD_IO_HEADER SendPci,
                   PUCHAR TxBuffer, DWORD TxLength,
                   PUCHAR RxBuffer, PDWORD RxLength, PSCARD_IO_HEADER RecvPci)
{
  /* This function performs an APDU exchange with the card/slot specified by
     Lun.  The driver is responsible for performing any protocol specific exchanges
     such as T=0/1 ... differences.  Calling this function will abstract all protocol
     differences.

     SendPci
     Protocol - 0, 1, .... 14
     Length   - Not used.

     TxBuffer - Transmit APDU example (0x00 0xA4 0x00 0x00 0x02 0x3F 0x00)
     TxLength - Length of this buffer.
     RxBuffer - Receive APDU example (0x61 0x14)
     RxLength - Length of the received APDU.  This function will be passed
     the size of the buffer of RxBuffer and this function is responsible for
     setting this to the length of the received APDU.  This should be ZERO
     on all errors.  The resource manager will take responsibility of zeroing
     out any temporary APDU buffers for security reasons.

     RecvPci
     Protocol - 0, 1, .... 14
     Length   - Not used.

     Notes:
     The driver is responsible for knowing what type of card it has.  If the current
     slot/card contains a memory card then this command should ignore the Protocol
     and use the MCT style commands for support for these style cards and transmit 
     them appropriately.  If your reader does not support memory cards or you don't
     want to then ignore this.

     RxLength should be set to zero on error.

     returns:

     IFD_SUCCESS
     IFD_COMMUNICATION_ERROR
     IFD_RESPONSE_TIMEOUT
     IFD_ICC_NOT_PRESENT
     IFD_PROTOCOL_NOT_SUPPORTED
  */

    char ret; 
    unsigned short ctn, slot, lc, lr;
    UCHAR sad, dad;

    ctn = ((unsigned short) (Lun & 0x0000FFFF)) % IFDH_MAX_READERS;
    slot = ((unsigned short) (Lun >> 8)) % IFDH_MAX_SLOTS;

    if (ifdh_status[ctn][slot] == NULL)
        return IFD_ICC_NOT_PRESENT;

    dad = (UCHAR) ((slot == 0)? 0x00 : slot+1);
    sad = 0x02;
    lr = (unsigned short) (*RxLength);
    lc = (unsigned short) TxLength;

    ret = CT_data (ctn, &dad, &sad, lc, TxBuffer, &lr, RxBuffer);

    if (ret != OK)
    {
        (*RxLength) = 0;
            return IFD_COMMUNICATION_ERROR;
    }

    (*RxLength) = lr;

    return IFD_SUCCESS;
}

RESPONSECODE
IFDHControl (DWORD Lun, PUCHAR TxBuffer,
             DWORD TxLength, PUCHAR RxBuffer, PDWORD RxLength)
{
  /* This function performs a data exchange with the reader (not the card)
     specified by Lun.  Here XXXX will only be used.
     It is responsible for abstracting functionality such as PIN pads,
     biometrics, LCD panels, etc.  You should follow the MCT, CTBCS 
     specifications for a list of accepted commands to implement.

     TxBuffer - Transmit data
     TxLength - Length of this buffer.
     RxBuffer - Receive data
     RxLength - Length of the received data.  This function will be passed
     the length of the buffer RxBuffer and it must set this to the length
     of the received data.

     Notes:
     RxLength should be zero on error.
  */

    char ret; 
    unsigned short ctn, slot,  lc, lr;
    UCHAR sad, dad;

    DebugLogD("IFDHControl : [%X], [%X], [%X]\n", TxBuffer[0], TxBuffer[1], TxBuffer[2]);

    ctn = ((unsigned short) (Lun & 0x0000FFFF)) % IFDH_MAX_READERS;
    slot = ((unsigned short) (Lun >> 8)) % IFDH_MAX_SLOTS;

    if (ifdh_status[ctn][slot] == NULL)
        return IFD_ICC_NOT_PRESENT;

    dad = 0x01;
    sad = 0x02;
    lr = (unsigned short) (*RxLength);
    lc = (unsigned short) TxLength;

    ret = CT_data (ctn, &dad, &sad, lc, TxBuffer, &lr, RxBuffer);

    if (ret != OK)
    {
        (*RxLength) = 0;
        return IFD_COMMUNICATION_ERROR;
    }

    (*RxLength) = lr;

    return IFD_SUCCESS;
}

RESPONSECODE
IFDHICCPresence (DWORD Lun)
{
  /* This function returns the status of the card inserted in the
     reader/slot specified by Lun.  It will return either:

     returns:
     IFD_ICC_PRESENT
     IFD_ICC_NOT_PRESENT
     IFD_COMMUNICATION_ERROR
  */

    char ret; 
    unsigned short ctn, slot,  lc, lr;
    UCHAR cmd[5], rsp[256], sad, dad;

    ctn = ((unsigned short) (Lun & 0x0000FFFF)) % IFDH_MAX_READERS;
    slot = ((unsigned short) (Lun >> 8)) % IFDH_MAX_SLOTS;

    cmd[0] = CTBCS_CLA;
    cmd[1] = CTBCS_INS_STATUS;
    cmd[2] = CTBCS_P1_CT_KERNEL;
    cmd[3] = CTBCS_P2_STATUS_ICC;
    cmd[4] = 0x00;

    dad = 0x01;
    sad = 0x02;
    lc = 5;
    lr = 256;

    ret = CT_data (ctn, &dad, &sad, lc, cmd, &lr, rsp);

    if (ret != OK)
        return IFD_COMMUNICATION_ERROR;

    if (slot >= lr - 2)
        return IFD_ICC_NOT_PRESENT;

    if (rsp[slot] == CTBCS_DATA_STATUS_NOCARD)
        return IFD_ICC_NOT_PRESENT;

    return IFD_ICC_PRESENT;
}
