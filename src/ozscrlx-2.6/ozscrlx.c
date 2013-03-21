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

 *    Module:       ozscrlx.c
 *    Author:       O2Micro Inc.,
 */
 

/*
 *
 * IRQ : One non-sharable IRQ
 * I/O : Length 0x20, 16bits. => instead of 8 bits
 * Mem : Length 0x20, 16bits, 200ns => instead of 8 bits
 * 
 */
#define OZSCR_IRQACK	0x04

/*
 * Commands to the OZSCR are TLV (Tag, Length, Value) coded and placed into
 * the I/O buffer starting at byte 2. If the length of a command exceeds 28
 * bytes (32 - 2 - 2) the command is sent in pieces of maximum 28 bytes with
 * a handshake performed after each chunk. The tag of such a command is
 * OR'ed with 0x04 (except for the last chunk). The maximum length of data
 * that can be transmitted to or read from the OZSCR is 256 bytes. When
 * data is received from the OZSCR the data is tagged with the same tag
 * as the command sent to get the data with bit 2 set.
 */
#define OZSCR_CLSE	0x10	/* Close session tag */
#define OZSCR_SLCT	0x50	/* Select card tag */

#define OZSCR_MAJOR	123	/* from sample=> experimental!*/
#define OZSCR_BUFSZ	256
#define ATR_SIZE        0x40    /* TS + 32 + SW + PROLOGUE + EPILOGUE...*/

/*MANUAL PORT Register Map*/
#define VCC 0x80
#define VPP 0x40
#define RST 0x20
#define CLK 0x10
#define IO  0x08

#include <linux/config.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/system.h>

#include <pcmcia/version.h>
#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
#include <pcmcia/cistpl.h>
#include <pcmcia/ds.h>

#include "ozscrlx.h"
#include "o2rgmap.h"
#include "o2error.h"

#undef MODULE_NAME
#define MODULE_NAME	"OZSCRLX "

BYTE bit_table[] = { 1, 2, 4, 8, 0x10, 0x20, 0x40, 0x80 };
USHORT Fi[] = { 372, 372, 558, 744, 1116, 1488, 1860, 0xFFFF, 0xFFFF,
                512, 768, 1024, 1536, 2048, 0xFFFF, 0xFFFF };
USHORT Di[] = { 0xFFFF, 1, 2, 4, 8, 16, 32, 0xFFFF, 12, 20, 0xFFFF,
                0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF };

/* Correct Table */
static WORD crctab[256] = {
0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

static NTSTATUS CmdResetInterface( PREADER_EXTENSION pRdrExt );
static NTSTATUS CmdResetReader( PREADER_EXTENSION pRdrExt, BOOLEAN WarmReset,
                                PUCHAR pATR, PULONG pATRLength);

static NTSTATUS CmdDeactivate( PREADER_EXTENSION pRdrExt );
static NTSTATUS CBTransmit( PREADER_EXTENSION pRdrExt );

/* Parameters that can be set with 'insmod' */
static char rcs_id[] = "O2Micro SmartCardBus Reader for kernel 2.6 (upto 2.6.12)";
static char *version = rcs_id;
static unsigned int	ozscr_major = OZSCR_MAJOR;

/* Bit map of interrupts to choose from */
static u_long irq_mask = 0xffff;

/*
 * The event() function is this driver's Card Services event handler.
 * It will be called by Card Services when an appropriate card status
 * event is received.  The config() and release() entry points are
 * used to configure or release a socket, in response to card insertion
 * and ejection events.  They are invoked from the OZSCR event
 * handler.
 */
static int ozscr_event(event_t event,int priority, event_callback_args_t *args);
static void ozscr_config(dev_link_t *link);
static void ozscr_release(u_long arg);

/*
 * The attach() and detach() entry points are used to create and destroy
 * "instances" of the driver, where each instance represents everything
 * needed to manage one actual PCMCIA card.
 */
static dev_link_t*  ozscr_attach(void);
static void         ozscr_detach(dev_link_t *);

/* 
 * Prototypes of card access functions(File Operations) 
 */
static irqreturn_t ozscr_interrupt(int irq, void *dev_id, struct pt_regs *regs);


/*
 * The dev_info variable is the "key" that is used to match up this
 * device driver with appropriate cards, through the card configuration
 * database.
 */
static dev_info_t dev_info = "ozscrlx_cs";

/*
 * A linked list of "instances" of the OZSCR device.  Each actual
 * PCMCIA card corresponds to one device instance, and is described
 * by one dev_link_t structure (defined in ds.h).
 */
static dev_link_t *dev_list = NULL;

/*
 * Private data for OZSCR reader. Need to provide a dev_node_t
 * structure for the device.
 * FIX: Possibly needs to be extended to support PRG encryption and/or
 *	downloading of additional drivers to the OZSCR.
 */
typedef struct ozscr_dev_t {
    dev_node_t  node;        /* associated device node           */
    caddr_t     am_base;     /* Base of mapped attribute memory  */
    caddr_t     cm_base;     /* We need to handle common memory*/
    u_long      io_base;     /* Base of I/O port range           */
    int         irq;         /* irq assigned to this card reader */
} ozscr_drv_t;


static struct pcmcia_driver ozscrlx_driver = {
	.owner		=THIS_MODULE,
	.drv		={
	.name	="ozscrlx_cs",
	},
	.attach		=ozscr_attach,
	.detach		=ozscr_detach,
};

/* Parameters that can be set with "insmod" */
static int mem_speed = 0;           /* in ns */
MODULE_PARM(mem_speed, "i");
static UCHAR ct;                    /*Contact current value*/
static PPSCR_REGISTERS sync_IOBase;
static PUCHAR sync_membase;

static READER_EXTENSION ozscr_reader;

/*
 * interrupt handler
 */
static irqreturn_t ozscr_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
    dev_link_t         *link;
    struct ozscr_dev_t *dev;
    u_char             ack;

#ifdef PCMCIA_DEBUG
    printk(KERN_INFO "OZSCRLX: interrupt\n");
#endif

    /* Find device that caused the interrupt.*/
    for (link = dev_list; link; link = link->next)
    {
        dev = (struct ozscr_dev_t *)link->priv;
        if (dev && dev->irq == irq)
            break;
    }

    if (!DEV_OK(link))
        return IRQ_NONE;

    dev = (struct ozscr_dev_t *)link->priv;

    /* Acknowledge interrupt to reader.*/
    ack = inb(dev->io_base);
    ack &= ~OZSCR_IRQACK;
    outb(ack, dev->io_base);

    return IRQ_HANDLED;
}

/*
 * open channel on OZSCR
 */
static int ozscr_open(struct inode *inode, struct file *file)
{
    #undef FUNC_NAME
    #define FUNC_NAME   "ozscr_open: "

    int                minor = MINOR(inode->i_rdev);
    dev_link_t         *link;
    struct ozscr_dev_t *dev;

#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "minor(%d)\n", minor);
#endif

    for (link = dev_list; link; link = link->next) 
        if (link->dev && link->dev->minor == minor)
            break;

    if (!DEV_OK(link))
        return (-ENODEV);

    dev = (struct ozscr_dev_t *)link->priv;

    /* Only one process may use the reader*/
    if (link->open > 0)	   
        return (-EBUSY);
    ++link->open;
    return 0;
}

/*
 * close channel on OZSCR
 */
static int ozscr_close(struct inode *inode, struct file *file)
{

#undef FUNC_NAME

#define FUNC_NAME   "ozscr_close: "

    int                minor = MINOR(inode->i_rdev);
    dev_link_t         *link;
    struct ozscr_dev_t *dev;

#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "minor(%d)\n", minor);
#endif

    for (link = dev_list; link; link = link->next)
        if (link->dev && link->dev->minor == minor)
            break;

    if (!DEV_OK(link))
        return 0;

    dev = (struct ozscr_dev_t *)link->priv;

    /* Make device available again, regardless of errors during close.*/
    --(link->open);

    /* If card was removed clean up*/
    if (link->state & DEV_STALE_CONFIG)
        ozscr_release((u_long)link);

    return 0;
}

/*
 *	All of the card reader operations are currently performed through
 *	ioctl's. This is because the semantics of read and write are not
 *	easily mapped onto reading/writing a card through the ISO interface.

 *

 * 	The ioctl's that do not have the possibility to return values other

 *	than a status return EIO for all possible problems. The ioctl's that
 *	copy information back to the user fill the reader status bit and
 *	succeed, even if the reader had an error.
 */
static int
ozscr_ioctl(struct inode *inode, struct file *file, u_int cmd, u_long arg)
{
#undef FUNC_NAME
#define FUNC_NAME	"ozscr_ioctl: "

    int                minor = MINOR(inode->i_rdev);
    dev_link_t         *link;
    struct ozscr_dev_t *dev;
    int                ret = 0;                /* return value */
    u_int              size;                   /* size for data transfers  */
    ULONG              ATRLength;
    UCHAR              ATRBuffer[ ATR_SIZE ];  /* TLVList[16]; */
    PREADER_EXTENSION  pRdrExt = &ozscr_reader;

    struct ozscr_apdu apdu;

    for (link = dev_list; link!=NULL; link = link->next)
        if ((link->dev != NULL) && (link->dev->minor == minor))
            break;
    if (!DEV_OK(link))
    {
        return -ENODEV;
    }

    dev = (struct ozscr_dev_t *)link->priv;
    size = (cmd & IOCSIZE_MASK) >> IOCSIZE_SHIFT;
    if (cmd & IOC_IN)
        if (!access_ok(VERIFY_READ, (char *)arg, size))
        {
            return -EFAULT;
        }
    if (cmd & IOC_OUT)
        if (!access_ok(VERIFY_WRITE, (char *)arg, size))
        {
            return -EFAULT;
        }

    switch (cmd)
    {
        case OZSCR_RESET: /* Reset CT */
#undef CMD_NAME
#define CMD_NAME   "  OZSCR_RESET  "
#ifdef PCMCIA_DEBUG
            printk(KERN_INFO MODULE_NAME FUNC_NAME CMD_NAME "\n");
#endif
            pRdrExt->membase = dev->am_base;
            ret = CmdResetInterface( pRdrExt );
            break;
        case OZSCR_PWROFF:
            ret = -EINVAL;
            break;
        case OZSCR_STNDBY:
            ret = -EINVAL;
            break;
        case OZSCR_OPEN: /* Request ICC */
#undef CMD_NAME
#define CMD_NAME   "  OZSCR_OPEN  "

#ifdef PCMCIA_DEBUG
            printk(KERN_INFO MODULE_NAME FUNC_NAME CMD_NAME "\n");
#endif
            ATRLength = ATR_SIZE;
            pRdrExt->IOBase = (PPSCR_REGISTERS) dev->io_base;
            pRdrExt->membase = dev->am_base;

            pRdrExt->m_SCard.AvailableProtocol = 0;
            pRdrExt->m_SCard.RqstProtocol = 0;  
#ifdef PCMCIA_DEBUG
            printk(KERN_INFO MODULE_NAME FUNC_NAME "membase:%p \n", pRdrExt->membase);
            printk(KERN_INFO MODULE_NAME FUNC_NAME "ioport:0x%03x\n", (unsigned)pRdrExt->IOBase);
#endif

            ret = CmdResetReader( pRdrExt, FALSE, ATRBuffer, &ATRLength );
            apdu.LengthOut = ATRLength;

#ifdef PCMCIA_DEBUG
            for( ATRLength = 0; ATRLength < apdu.LengthOut; ATRLength++ )
                printk(KERN_INFO " [%02X] ", ATRBuffer[ATRLength] );
            printk(KERN_INFO "\n");
#endif

            memcpy( apdu.DataOut, ATRBuffer, ATRLength );
            ret = copy_to_user((struct ozscr_apdu *)arg, &apdu, sizeof(struct ozscr_apdu));
            break;
        case OZSCR_CLOSE: /* Eject ICC */
#undef CMD_NAME
#define CMD_NAME   "  OZSCR_CLOSE  "
#ifdef PCMCIA_DEBUG
            printk(KERN_INFO MODULE_NAME FUNC_NAME CMD_NAME "\n");
#endif
            pRdrExt->membase = dev->am_base;

            ret = CmdDeactivate( pRdrExt );
            break;
        case OZSCR_SELECT:
#undef CMD_NAME

#define CMD_NAME   "  OZSCR_SELECT  "
#ifdef PCMCIA_DEBUG
            printk(KERN_INFO MODULE_NAME FUNC_NAME CMD_NAME " test clk\n");
#endif

            /* set protocol */
            ret = copy_from_user(&apdu, (struct ozscr_apdu *)arg, sizeof(struct ozscr_apdu));
            pRdrExt->IOBase = (PPSCR_REGISTERS) dev->io_base;
            pRdrExt->membase = dev->am_base;
            pRdrExt->m_SCard.RqstProtocol = apdu.DataIn[6];
#ifdef PCMCIA_DEBUG
            printk(KERN_INFO MODULE_NAME FUNC_NAME "membase:%p \n", pRdrExt->membase);
            printk(KERN_INFO MODULE_NAME FUNC_NAME "ioport:0x%03x\n", (unsigned)pRdrExt->IOBase);
#endif
            ret = CmdResetReader( pRdrExt, FALSE, ATRBuffer, &ATRLength );
            apdu.LengthOut = ATRLength;
            memcpy( apdu.DataOut, ATRBuffer, ATRLength );
            ret = copy_to_user((struct ozscr_apdu *)arg, &apdu, sizeof(struct ozscr_apdu));
            break;

        case OZSCR_STATUS: /* Get Status */
#undef CMD_NAME
#define CMD_NAME   "  OZSCR_STATUS  "
            pRdrExt->membase = dev->am_base;
            ret = readw(pRdrExt->membase + STATUS_EXCH);
            break;
    case OZSCR_RAM:
            ret = -EINVAL;
            break;
    case OZSCR_CMD:
#undef CMD_NAME
#define CMD_NAME "  OZSCR_CMD  "
#ifdef PCMCIA_DEBUG
            printk(KERN_INFO MODULE_NAME FUNC_NAME CMD_NAME "protocol=%d\n", pRdrExt->m_SCard.Protocol );
#endif

            ret = copy_from_user(&apdu, (struct ozscr_apdu *)arg, sizeof(struct ozscr_apdu));
            pRdrExt->IOBase = (PPSCR_REGISTERS) dev->io_base;
            pRdrExt->membase = dev->am_base;
#ifdef PCMCIA_DEBUG
            printk("In data transfer: apdu.length = %d \n", (USHORT)apdu.LengthIn);
            for(ATRLength=0;ATRLength<apdu.LengthIn;ATRLength++)
                printk("[%02X] ", apdu.DataIn[ATRLength] );
            printk("\n");
            printk("%s\n", apdu.DataIn+5);
#endif
            pRdrExt->SmartcardRequest.Buffer = apdu.DataIn; 
            pRdrExt->SmartcardRequest.BufferLength = apdu.LengthIn;
            pRdrExt->SmartcardReply.Buffer = apdu.DataOut;

            pRdrExt->SmartcardReply.BufferLength = apdu.LengthOut;

            ret = CBTransmit( pRdrExt );
            apdu.LengthOut = pRdrExt->SmartcardReply.BufferLength;
#ifdef PCMCIA_DEBUG
            printk( "Dump FIFO (a.L=%2X) ", (USHORT)apdu.LengthOut );
            printk( " (r.L=%2X) ", (USHORT)pRdrExt->SmartcardReply.BufferLength );
#if 0
            for( ATRLength = 0; ATRLength < apdu.LengthOut; ATRLength++ )
                printk( " [%02X]", apdu.DataOut[ATRLength] );
#endif
            printk("\n");
#endif

            ret = copy_to_user((struct ozscr_apdu *)arg, &apdu, sizeof(struct ozscr_apdu));

            break;
        default:
            ret = -EINVAL;
            break;
    }

    return ret;

} /* end of ozscr_ioctl */


/*
 * cs_error -- decode card service error message
 */

void cs_error(client_handle_t handle, int func, int ret)
{
	error_info_t err = { func, ret };
	pcmcia_report_error(handle, &err);
}

/*
 * Card service configruation
 * Helper function to get a tuple, get it's data and parse it. 
 */
static int
get_tuple(client_handle_t handle, tuple_t *tuple, cisparse_t *parse)
{
    int i;

#undef FUNC_NAME
#define FUNC_NAME	"get_tuple: "

#ifdef PCMCIA_DEBUG
	printk(KERN_INFO MODULE_NAME FUNC_NAME "func begin\n");
#endif

    if((i = pcmcia_get_tuple_data(handle, tuple)) != CS_SUCCESS) 
	return i;

    i = pcmcia_parse_tuple(handle, tuple, parse);
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "func end\n");
#endif

    return i;
}


/*
 *  ozscrconfig() is scheduled to run after a CARD_INSERTION event
 *  is received, to configure the PCMCIA socket, and to make the
 *  device available to the system.
 */
static void ozscr_config(dev_link_t *link)
{
#undef FUNC_NAME
#define FUNC_NAME	"ozscr_config: "

    client_handle_t         handle = link->handle;
    struct ozscr_dev_t      *dev = link->priv;
    tuple_t                 tuple;
    cisparse_t              parse;
    cistpl_cftable_entry_t  *cf = &parse.cftable_entry;
    int                     i;
    u_char                  buf[OZSCR_BUFSZ];
    win_req_t               req;
    memreq_t                mem;

#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "Device Link(0x%p)\n", link);
#endif

    /* This reads the card's CONFIG tuple to find its configuration registers. */
    tuple.Attributes = 0;
    tuple.TupleData = buf;
    tuple.TupleOffset = 0;
    tuple.TupleDataMax = OZSCR_BUFSZ - 1;
    tuple.DesiredTuple = CISTPL_CONFIG;

#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "get tuple\n");
#endif

    if((i = pcmcia_get_first_tuple(handle, &tuple)) != CS_SUCCESS) 
    {
        cs_error(link->handle, ParseTuple, i);
        link->state &= ~DEV_CONFIG_PENDING;
        goto error_return;
    }
    if((i = get_tuple(handle, &tuple, &parse)) != CS_SUCCESS) 
    {
        cs_error(link->handle, ParseTuple, i);
        link->state &= ~DEV_CONFIG_PENDING;
        goto error_return;
    }

#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "GetFirstTuple Complete\n");
#endif

    link->conf.ConfigBase = parse.config.base;
    link->conf.Present = parse.config.rmask[0];

    /* Configure card Find I/O port for the card.*/
    link->state |= DEV_CONFIG;
    tuple.TupleOffset = 0;
    tuple.Attributes = 0;

    tuple.TupleData = buf; 
    tuple.TupleDataMax = OZSCR_BUFSZ - 1;	

    tuple.DesiredTuple = CISTPL_CFTABLE_ENTRY;

#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "config card (Find IO port)\n");
#endif

    if((i = pcmcia_get_first_tuple(handle, &tuple)) != CS_SUCCESS) 
    {
        cs_error(link->handle, ParseTuple, i);
        link->state &= ~DEV_CONFIG_PENDING;
        goto error_return;
    }
    i = get_tuple(handle, &tuple, &parse);
    while (i == CS_SUCCESS) 
    {
        if (cf->io.nwin > 0) 
        {
            link->conf.ConfigIndex = cf->index;
            link->io.BasePort1 = cf->io.win[0].base;
            i = pcmcia_request_io(link->handle, &link->io);

            if (i == CS_SUCCESS)
                break;
        }
	if((i = pcmcia_get_next_tuple(handle, &tuple)) != CS_SUCCESS) 
	{
		cs_error(link->handle, ParseTuple, i);
		link->state &= ~DEV_CONFIG_PENDING;
		goto error_return;
	}
	i = get_tuple(handle, &tuple, &parse);
    }

    if (i != CS_SUCCESS)
    {
        cs_error(link->handle, RequestIO, i);
        goto error_return;
    }
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "IO Port 0x%03x\n", link->io.BasePort1);
    printk(KERN_INFO MODULE_NAME FUNC_NAME "Begin to request IRQ\n");
#endif
    /* Configure card Now allocate an interrupt line. (IRQ)*/
    if ((i = pcmcia_request_irq(handle, &link->irq)) != CS_SUCCESS) 
    {
#ifdef PCMCIA_DEBUG
        printk(KERN_INFO MODULE_NAME FUNC_NAME "Request IRQ fail!\n");
#endif
        cs_error(link->handle, RequestIRQ, i);
        goto error_return;
    }
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "Request IRQ successful");
    printk("(IRQ: %d)\n", link->irq.AssignedIRQ );
#endif

    /* Set up the I/O window and the interrupt mapping.*/
    i = pcmcia_request_configuration(link->handle, &link->conf);
    if ( i != CS_SUCCESS)
    {
        cs_error(link->handle, RequestConfiguration, i);
        goto error_return;
    }
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "setup I/O Win & INT map ok\n");
#endif

    /* Allocate a 2K memory window for the attribute space. This
       contains four registers of interest.*/
    req.Attributes = WIN_DATA_WIDTH_16|WIN_MEMORY_TYPE_CM|WIN_ENABLE;
    req.Base = 0;
    req.Size = 0x1000; /* Request 2K memory */
    req.AccessSpeed = mem_speed;
    link->win = (window_handle_t)link->handle;
    if ((i = pcmcia_request_window(&link->handle, &req, &link->win)) != CS_SUCCESS)
    {
#ifdef PCMCIA_DEBUG
        printk(KERN_INFO MODULE_NAME FUNC_NAME "Request memory fail!\n");
#endif
        cs_error(link->handle, RequestWindow, i);
        goto error_return;
    }
    dev->cm_base = (caddr_t)req.Base;
    dev->am_base = ioremap(req.Base,0x1000);
    mem.CardOffset = 0x0;
    mem.Page = 0;
    if ((i = pcmcia_map_mem_page(link->win, &mem))) 
    {
#ifdef PCMCIA_DEBUG
        printk(KERN_INFO MODULE_NAME FUNC_NAME "Map Memory Page fail!\n");
#endif
        cs_error(link->handle, MapMemPage, i);
        goto error_return;
    }

#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "am_base:%p \n", dev->am_base);
#endif

    /* Initialize the dev_node_t structure */
    sprintf(dev->node.dev_name, "ozscrlx");
    dev->node.major = ozscr_major;
    dev->node.minor = 0;
    dev->cm_base = (caddr_t)req.Base;
    dev->io_base = link->io.BasePort1;
    dev->irq = link->irq.AssignedIRQ;

    link->dev = &dev->node;

    link->state &= ~DEV_CONFIG_PENDING;

#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "OZSCR device loaded\n");
#endif
    return;

error_return:

    /* If any step failed, release any partially configured state */
    ozscr_release((u_long)link);
    return;
}

/*
 *  The card status event handler.  Mostly, this schedules other
 *  stuff to run after an event is received.  A CARD_REMOVAL event
 *  also sets some flags to discourage the net drivers from trying
 *  to talk to the card any more.
 *
 *  When a CARD_REMOVAL event is received, we immediately set a flag
 *  to block future accesses to this device.  All the functions that

 *  actually access the device should check this flag to make sure
 *  the card is still present.
 */
static int ozscr_event(event_t event, int priority, event_callback_args_t *args)
{
    dev_link_t *link = args->client_data;

#undef FUNC_NAME
#define FUNC_NAME	"ozscr_event: "
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "function begins\n");
#endif

    switch (event)
    {
        case CS_EVENT_CARD_INSERTION:

#ifdef PCMCIA_DEBUG
            printk(KERN_INFO MODULE_NAME FUNC_NAME "Event: Card_Inserted\n");
#endif

            link->state |= DEV_PRESENT | DEV_CONFIG_PENDING;
            ozscr_config(link);
            break;
        case CS_EVENT_REGISTRATION_COMPLETE:
#ifdef PCMCIA_DEBUG
            printk(KERN_INFO MODULE_NAME FUNC_NAME	"registration complete\n");
#endif
            break;
        case CS_EVENT_CARD_REMOVAL:
#ifdef PCMCIA_DEBUG
            printk(KERN_INFO MODULE_NAME FUNC_NAME "Event: Card_Removed\n");
#endif
            link->state &= ~DEV_PRESENT;
            link->state &= DEV_STALE_CONFIG;
            if (link->state & DEV_CONFIG) 
            {
		flush_scheduled_work();
            }
            ozscr_release((u_long)link);
            break;
        case CS_EVENT_PM_SUSPEND:
#ifdef PCMCIA_DEBUG
            printk(KERN_INFO MODULE_NAME FUNC_NAME "Event: Card_Suspend\n");
#endif
            link->state |= DEV_SUSPEND;
            /* Fall through... */
        case CS_EVENT_RESET_PHYSICAL:
#ifdef PCMCIA_DEBUG
            printk(KERN_INFO MODULE_NAME FUNC_NAME "Event: Card_Physical_Reset\n");
#endif
            if (link->state & DEV_CONFIG)
                pcmcia_release_configuration(link->handle);
            break;
        case CS_EVENT_PM_RESUME:

#ifdef PCMCIA_DEBUG
            printk(KERN_INFO MODULE_NAME FUNC_NAME "Event: Card_Resume\n");
#endif

            link->state &= ~DEV_SUSPEND;
            /* Fall through... */
        case CS_EVENT_CARD_RESET:
#ifdef PCMCIA_DEBUG
            printk(KERN_INFO MODULE_NAME FUNC_NAME "Event: Card_Reset\n");
#endif
            if (link->state & DEV_CONFIG)
                pcmcia_request_configuration(link->handle, &link->conf);
            break;
        default:
#ifdef PCMCIA_DEBUG
            printk(KERN_INFO MODULE_NAME FUNC_NAME "Event: Unknown\n");
#endif
            break;
    }
    return 0;
}



/*
 *  ozscr_attach() creates an "instance" of the driver, allocating
 *  local data structures for one device.  The device is registered
 *  with Card Services.
 *
 *  The dev_link structure is initialized, but we don't actually
 *  configure the card at this point -- we wait until we receive a
 *  card insertion event.
 */
static dev_link_t *ozscr_attach(void)
{
#undef FUNC_NAME

#define FUNC_NAME	"ozscr_attach: "

    client_reg_t       client_reg;
    dev_link_t         *link=NULL;
    struct ozscr_dev_t *local=NULL;
    int                ret=0;
    PREADER_EXTENSION  pRdrExt=NULL;

#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "function begins\n"); 
#endif 
    /* Initialize the dev_link_t structure */
    link = kmalloc(sizeof(struct dev_link_t), GFP_KERNEL);
    if (link == NULL)
    {
#ifdef PCMCIA_DEBUG
        printk(KERN_INFO MODULE_NAME FUNC_NAME "allocate dev_link_t fail\n"); 
#endif

        goto ErrHandle;
    }
    memset(link, 0, sizeof(struct dev_link_t));

    /* Allocate space for private device-specific data */
    local = kmalloc(sizeof(struct ozscr_dev_t), GFP_KERNEL);
    if (local == NULL)
    {
#ifdef PCMCIA_DEBUG
        printk(KERN_INFO MODULE_NAME FUNC_NAME "allocate dev_link_t fail\n");
#endif
        goto ErrHandle;
    }
    memset(local, 0, sizeof(struct ozscr_dev_t));

    /* Allocate space for private device-specific data */
    pRdrExt = kmalloc(sizeof(PREADER_EXTENSION), GFP_KERNEL);

    if (pRdrExt == NULL)
    {
#ifdef PCMCIA_DEBUG
        printk(KERN_INFO MODULE_NAME FUNC_NAME "allocate dev_link_t fail\n");
#endif
        goto ErrHandle;
    }
    memset(local, 0, sizeof(PREADER_EXTENSION));

    link->priv = local;

    /* The io structure describes IO port mapping */
    link->io.NumPorts1 = 32;
    link->io.Attributes1 = IO_DATA_PATH_WIDTH_16;
    link->io.NumPorts2 = 0;
    link->io.IOAddrLines = 5;

    /* Interrupt setup */
    link->irq.Attributes = IRQ_TYPE_EXCLUSIVE|IRQ_HANDLE_PRESENT;
    link->irq.IRQInfo1 = IRQ_INFO2_VALID|IRQ_LEVEL_ID;
    link->irq.IRQInfo2 = irq_mask;
    link->irq.Handler = &ozscr_interrupt;
    link->irq.Instance = &ozscr_reader;

    /* General socket nfiguration */
    link->conf.Attributes = CONF_ENABLE_IRQ;
    link->conf.Vcc = 50;
    link->conf.Vpp1 = link->conf.Vpp2 = 50;	
    link->conf.IntType = INT_MEMORY_AND_IO;
    link->conf.Present = PRESENT_OPTION | PRESENT_STATUS;

    link->next = dev_list; /* build linked to handle multiple instances */
    dev_list = link;

    client_reg.dev_info = &dev_info;
    client_reg.Attributes = INFO_IO_CLIENT | INFO_CARD_SHARE;
    client_reg.EventMask = CS_EVENT_CARD_INSERTION | CS_EVENT_CARD_REMOVAL |
                           CS_EVENT_RESET_PHYSICAL | CS_EVENT_CARD_RESET |
                           CS_EVENT_PM_SUSPEND | CS_EVENT_PM_RESUME;
    client_reg.event_handler = &ozscr_event;
    client_reg.Version = 0x0210;
    client_reg.event_callback_args.client_data = link;

    ret = pcmcia_register_client(&link->handle, &client_reg);
    if (ret != CS_SUCCESS)
    {
#ifdef PCMCIA_DEBUG
        printk(KERN_INFO MODULE_NAME FUNC_NAME "register client fail!\n");
#endif
        goto ErrHandle;
    }
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "function complete\n"); 
#endif 
    pRdrExt->IOBase = (PPSCR_REGISTERS)local->io_base;
    pRdrExt->membase = local->am_base;

    CmdResetInterface(pRdrExt);
    return link;

ErrHandle:

    if (ret != 0)
    {
        cs_error(link->handle, RegisterClient, ret);
        ozscr_detach(link);	link = NULL;
    }
    /* Free the allocated memory space */
    if (link != NULL)
    {
        kfree(link);	
        link = NULL;
    }
    if (local != NULL)
    {
        kfree(local);	
        local = NULL;
    }
    if (pRdrExt != NULL)
    {
        kfree(pRdrExt);	
        pRdrExt = NULL;
    }

    return NULL;
}



/*
 * After a card is removed, ozscr_release() will unregister the net
 * device, and release the PCMCIA configuration. This is also called
 * through a timer while the device is still open when the card has
 * been removed.
 */
static void
ozscr_release(u_long arg)
{

    dev_link_t *link = (dev_link_t *)arg;

#undef FUNC_NAME
#define FUNC_NAME	"ozscr_release: "
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "Device Link(0x%p)\n", link);
#endif

/*
 * If the device is still in use we may not release resources
 * right now but we must ensure that no further actions are
 * done on the device. The resources we still hold will be
 * eventually freed through ozscr_detach.
 */

    if (link->open)
    {
#ifdef PCMCIA_DEBUG
        printk(KERN_INFO MODULE_NAME FUNC_NAME "release postponed,%s still open\n", link->dev->dev_name);
#endif
        link->state |= DEV_STALE_CONFIG;
        return;
    }

    /* Unlink the device chain */
    link->dev = NULL;

    /* Don't bother checking to see if these succeed or not */
    pcmcia_release_window(link->win);
    pcmcia_release_configuration(link->handle);

    pcmcia_release_io(link->handle, &link->io);
    pcmcia_release_irq(link->handle, &link->irq);
    link->state &= ~DEV_CONFIG;

    if (link->state & DEV_STALE_LINK) 
        ozscr_detach(link);

    return;
}

/*
 *  This deletes a driver "instance".  The device is de-registered
 *  with Card Services.  If it has been released, all local data
 *  structures are freed.
 */
static void
ozscr_detach(dev_link_t *link)
{

#undef FUNC_NAME
#define FUNC_NAME	"ozscr_detach: "

    dev_link_t **linkp;

#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "Device Link (0x%p)\n", link);
#endif

    /*
     * Locate device structure
     */
    for (linkp = &dev_list; *linkp; linkp = &(*linkp)->next) 
    {
        if (*linkp == link)
            break;
    }
    if (*linkp == NULL)
        return;

    if (link->state & DEV_CONFIG)
    {
#ifdef PCMCIA_DEBUG
        printk(KERN_INFO MODULE_NAME FUNC_NAME ": detach of %s postponed"
               " - device still configured", link->dev->dev_name);
#endif

        ozscr_release((u_long)*linkp);
        if (link->state & DEV_STALE_CONFIG)
        {
            link->state |= DEV_STALE_LINK;
            return;
        }
    }


    /* Break the link with Card Services */
    if (link->handle)
        pcmcia_deregister_client(link->handle);

    /* Unlink device structure, free pieces */
    *linkp = link->next;

    if (link->priv) 
    {
        kfree(link->priv);
    }
    kfree(link);

    return;

}

ssize_t ozscr_read(
	struct file *file,
	char	*pucRxBuffer,
	size_t	count,
	loff_t	*loc
)
{
#ifdef PCMCIA_DEBUG
	printk("ozscr_read called\n");
#endif
	return 1;
}


ssize_t ozscr_write(
	struct file *file,
	const char *pucTxBuffer,
	size_t	count,
	loff_t	*loc
)
{
#ifdef PCMCIA_DEBUG
	printk("ozscr_write called\n");
#endif
	return 1;
}

static struct file_operations ozscr_chr_fops = {
    read:     ozscr_read,   /* read */
    ioctl:    ozscr_ioctl,  /* ioctl */
    write:    ozscr_write,  /* write */
    open:     ozscr_open,   /* open */
    release:  ozscr_close,  /* release */
};

/* ========================================================================= 
   Entry/Ending point of the driver
*/
static int
init_ozscrlx(void)
{

#undef FUNC_NAME
#define FUNC_NAME   "init_ozscrlx: "

    servinfo_t    serv;

    printk(KERN_INFO MODULE_NAME "version: %s\n", version);

    pcmcia_get_card_services_info(&serv);
#if 1
    if (serv.Revision != CS_RELEASE_CODE) 
    {
#ifdef PCMCIA_DEBUG
        printk(KERN_NOTICE MODULE_NAME FUNC_NAME "CardServices does not match!\n");
#endif
        return -1;
    }
#endif

    pcmcia_register_driver(&ozscrlx_driver);

    /* Register new character device with kernel */
    if (register_chrdev(OZSCR_MAJOR, "ozscrlx", &ozscr_chr_fops) != 0) 
    {
#ifdef PCMCIA_DEBUG
        printk(KERN_WARNING MODULE_NAME FUNC_NAME "Grab device fail#%d\n", OZSCR_MAJOR);
#endif
        pcmcia_unregister_driver(&ozscrlx_driver);
        return -1;
    }

#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "major num: %d\n", OZSCR_MAJOR);
    printk(KERN_INFO MODULE_NAME FUNC_NAME "function complete!\n");
#endif

    return 0;

}

static void
exit_ozscrlx(void)
{
#undef FUNC_NAME

#define FUNC_NAME   "exit_ozscrlx: "
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "unloading module\n");
#endif
    unregister_chrdev(ozscr_major, "ozscrlx");
    pcmcia_unregister_driver(&ozscrlx_driver);
    while (dev_list != NULL)
    {
        if (dev_list->state & DEV_CONFIG)
            ozscr_release((u_long)dev_list);
        ozscr_detach(dev_list);
    }

    return;
}


/*
Description: Since all memory or I/O R/W is 16 bits, even we need
             read a byte.
Procedure: Read Word
           return the low byte
*/

UCHAR wREAD_REGISTER_UCHAR( PUCHAR address  )
{
    USHORT us = readw( address );
    return (UCHAR)us;
}

void wWRITE_REGISTER_UCHAR( PUCHAR address, USHORT byte )
{
    writew( byte, address );
}


UCHAR wREAD_PORT_UCHAR( PUCHAR address )
{
    unsigned short us;
    us = inw( (unsigned)address );
    return (UCHAR) (us >> 8);
}

void wWRITE_PORT_UCHAR( PUCHAR address, USHORT byte )
{

    byte = byte << 8;
    outw( byte, (unsigned)address );
}


void wWRITE_REGISTER_ULONG( PULONG address, ULONG ul )
{
    USHORT us;
    us = (USHORT) (ul >> 16);
    writew( us, address );
    us = (USHORT) ul;
    address = (PULONG) (((PUSHORT)address) + 1);

    writew( us, address );
}

/*
    Description: Excute command correspond to exeCmd
    Procedure:
    EXE.exeCmd = 1 

    Wait for STATUS_IT.End_EXE =1 to see if command complete
    Check first byte in STATUS_EXCH for any error
    END OF PROCEDURE
*/
static NTSTATUS
CmdExecuteWithTimeOutAndTOC( PREADER_EXTENSION pRdrExt, USHORT exeCmd, USHORT *status_exch, ULONG timeout, BOOLEAN SetTOC )
{
    NTSTATUS NTStatus = STATUS_SUCCESS;
    UCHAR uc;
    PUCHAR membase;
    ULONG timeoutcount;


    membase = pRdrExt->membase;
    writew( exeCmd, membase+EXE );

    for( timeoutcount = timeout; timeoutcount > 0; timeoutcount -- )
    {

        /*need check STATUS_IT.End_EXE to see if command complete*/
        uc = wREAD_REGISTER_UCHAR( membase+STATUS_IT );
        if( (uc & END_EXE ) == END_EXE )
        {
            NTStatus = STATUS_SUCCESS;
            /* Clear correspond bit in DEVAL_IT*/
            wWRITE_REGISTER_UCHAR( membase+DEVAL_IT, ~END_EXE_CLR_B );
            break;
        }

        mdelay(1);
    } 

    *status_exch = readw( membase+STATUS_EXCH );
    if( SetTOC == TRUE )
    {
        writew( S_TOC_EXE, membase+EXE );
    }
    return NTStatus;
}

static NTSTATUS
CmdExecuteWithTimeOut( PREADER_EXTENSION pRdrExt, USHORT exeCmd, USHORT *status_exch, int timeout )
{
    return CmdExecuteWithTimeOutAndTOC( pRdrExt, exeCmd, status_exch, timeout, FALSE );
}


static NTSTATUS
CmdExecute( PREADER_EXTENSION pRdrExt, USHORT exeCmd, USHORT *status_exch )
{
    return CmdExecuteWithTimeOut(pRdrExt,exeCmd,status_exch, CMD_MAX_DELAY );
}

/* Launch the command and don't wait for it finished or not */
void CmdExecuteWithNoWait( PREADER_EXTENSION ReaderExtension, USHORT exeCmd )
{

    PUCHAR membase;
    membase = ReaderExtension->membase;

    writew( exeCmd, membase+EXE );

}


NTSTATUS CmdWaitForEndOfCmd( PREADER_EXTENSION pRdrExt, USHORT *status_exch, int timeout  )
{
    NTSTATUS NTStatus = STATUS_SUCCESS;
    UCHAR uc;
    PUCHAR membase;
    int timeoutcount;

    membase = pRdrExt->membase;
    NTStatus = STATUS_IO_DEVICE_ERROR; /* Assume it will failed first */

    for( timeoutcount = timeout; timeoutcount > 0; timeoutcount -- )
    {

        /* need check STATUS_IT.End_EXE to see if command complete */
        uc = wREAD_REGISTER_UCHAR( membase+STATUS_IT );
        if( (uc & END_EXE ) == END_EXE ) 
        {
            NTStatus = STATUS_SUCCESS;
            /* Clear correspond bit in DEVAL_IT */
            wWRITE_REGISTER_UCHAR( membase+DEVAL_IT, ~END_EXE_CLR_B );
            break;
        }
        mdelay(1);
    } 

#undef FUNC_NAME
#define FUNC_NAME   "CmdWaitForEndOfCmd: "
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "timeout = %d\n", timeoutcount);
#endif
    /* Check if STATUS_EXCH */
    *status_exch = readw( (PUSHORT)(membase+STATUS_EXCH) );

    return NTStatus;
}

/*
	Description: Clear FIFO
	Procedure:
	EXE.RST_FIFO_EXE = 1 
	'need check STATUS_IT.End_EXE to see if command complete
	'The STATUS_EXCH.FIFO_EMPTY = 1
	END OF PROCEDURE
*/
static NTSTATUS 
CmdClearFIFO( PREADER_EXTENSION pRdrExt )
{
    NTSTATUS NTStatus = STATUS_SUCCESS;
    USHORT status_exch;
    PUCHAR membase;

#undef FUNC_NAME

#define FUNC_NAME   "CmdClearFIFO: "

#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "function start\n");
#endif

    membase = pRdrExt->membase;
    /*Check if FIFO already empty or not*/
    if( readw( membase+FIFO_NB ) == 0 )
        return NTStatus;

    /*EXE.RST_FIFO_EXE = 1*/
    NTStatus = CmdExecute( pRdrExt, RST_FIFO_EXE, &status_exch );
    if( NTStatus )
    {
        /*Check if STATUS_EXCH.FIFO_EMPTY = 1*/
        if( (status_exch & FIFO_EMPTY) != FIFO_EMPTY )
        { 
#ifdef PCMCIA_DEBUG
            printk("ERROR #8 :: IO DEVICE ERROR in CmdClearFIFO() \n");
#endif
            NTStatus = STATUS_IO_DEVICE_ERROR;
        }
    }


#undef FUNC_NAME
#define FUNC_NAME   "CmdClearFIFO: "
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "function complete\n");
#endif

    return NTStatus;
}

static void
PscrFlushInterface( PREADER_EXTENSION pRdrExt )
{
    ULONG Status;

    Status = CmdClearFIFO( pRdrExt );
}


/*
 * Read All DATA in FIFO_OUT
 */
NTSTATUS ReadATR( PREADER_EXTENSION pRdrExt, PUCHAR pATR, PULONG pATRLength)
{
    NTSTATUS NTStatus = STATUS_SUCCESS;
    ULONG len;
    PPSCR_REGISTERS IOBase;

    PUCHAR membase;
    ULONG fifoLength;

    IOBase = pRdrExt->IOBase;
    membase = pRdrExt->membase;

    /*Read number of bytes in FIFO_NB*/
    fifoLength = readw( membase+FIFO_NB );
#ifdef PCMCIA_DEBUG
    printk("fifoLength = %ld in ReadATR() \n", fifoLength);
#endif
    fifoLength &= 0X1FF;

#ifdef PCMCIA_DEBUG
    printk("fifoLength = %ld in ReadATR() after & \n", fifoLength);
#endif

    if( fifoLength > *pATRLength )
    {
#ifdef PCMCIA_DEBUG
        printk(" STATUS_BUFFER_TOO_SMALL: code 047 \n");
#endif
        return STATUS_BUFFER_TOO_SMALL;
    }
    *pATRLength = fifoLength;
    pRdrExt->m_SCard.Atr_len = fifoLength;

    /*Read all bytes via FIFO_OUT*/
    for( len = 0; len < *pATRLength; len++ )
    {
        *(pATR+len) = wREAD_PORT_UCHAR( &IOBase->FIFO_OUT );
        pRdrExt->m_SCard.ATR[len] = *(pATR+len);
    }

#undef FUNC_NAME
#define FUNC_NAME   "ReadATR: "
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME );
    for( len = 0; len < *pATRLength; len++ )
        printk(KERN_INFO " [%02X]", pRdrExt->m_SCard.ATR[len]);
    printk(KERN_INFO "\n");
#endif

    return NTStatus;
}



/*
 * Read All DATA in FIFO_OUT
 */
NTSTATUS ReadFIFO( PREADER_EXTENSION pRdrExt, PUCHAR pATR, PULONG pATRLength)
{
    NTSTATUS NTStatus = STATUS_SUCCESS;
    ULONG len;
    PPSCR_REGISTERS IOBase;
    PUCHAR membase;
    ULONG fifoLength;

    IOBase = pRdrExt->IOBase;
    membase = pRdrExt->membase;

    /* Read number of bytes in FIFO_NB */
    fifoLength = readw( membase+FIFO_NB );

#ifdef PCMCIA_DEBUG
    printk("fifoLength = %ld in ReadFIFO() \n", fifoLength);
#endif

    fifoLength &= 0X1FF;
#ifdef PCMCIA_DEBUG
    printk("fifoLength = %ld in ReadFIFO() after & \n", fifoLength);
#endif
    /*Check if buffer big enough to hold the data*/
    if( fifoLength > *pATRLength )
    {
#ifdef PCMCIA_DEBUG
        printk(" STATUS_BUFFER_TOO_SMALL: code 047 \n");
#endif
        return STATUS_BUFFER_TOO_SMALL;
    }
    *pATRLength = fifoLength; 

    for( len = 0; len < *pATRLength; len++ )
    {
        *(pATR+len) = wREAD_PORT_UCHAR( &IOBase->FIFO_OUT );
    }

#undef FUNC_NAME
#define FUNC_NAME   "ReadFIFO: "
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME );
    for( len = 0; len < *pATRLength; len++ )
        printk(KERN_INFO " [%02X]", pATR[len]);
    printk(KERN_INFO "\n");
    printk("ReadFIFO() exti: data1 = %02x, data2 = %02x \n", pATR[0], pATR[1]);
    for(len = 0; len < *pATRLength; len ++)
    {
        printk("[%02x] ", pATR[len]);
    }
    printk("\n");
#endif
    return NTStatus;
}

/* CmdResetInterface:

   Description: Reset the Reader. (CmdResetInterface)
   Procedure:
   'Before reset
   MASK_IT = 0 'Unmask all interrupt
   MODE.CRD_DET = 1 'Ref: T P.11 and G P.16. Card Insertion will make SC_DET = 0

   'Reset now
   EXE.RESET_EXE = 1 '??? Ref: G P.25.

   Wait for at least 1 microseconds
   EXE.RESET_EXE = 0

   ClearFIFO


   'FREQUENCE ??? 33Mhz or external ???
   FRQ_MODE = 4 'Our CLK f = 33Mhz => f/(4*2) = 4.125Mhz


   'Make interrupt available for Card Insertion/Extraction
   MASK_IT.SCP_MSK = 1
   MASK_IT.SCI_MSK = 1

   END OF PROCEDURE


Arguments:
   ReaderExtension context of call

Return Value:
   STATUS_SUCCESS
   STATUS_IO_DEVICE_ERROR
 */
NTSTATUS
CmdResetInterface( PREADER_EXTENSION pRdrExt )
{
    NTSTATUS NTStatus = STATUS_SUCCESS;
    PUCHAR membase;
    UCHAR uc;

#undef FUNC_NAME
#define FUNC_NAME   "CmdResetInterface: "
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "function start\n");
#endif

    PscrFlushInterface( pRdrExt );
    membase = pRdrExt->membase;

#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "MEM Base:%p\n", membase);
#endif
    wWRITE_REGISTER_UCHAR( membase+MASK_IT, 0);

    uc = wREAD_REGISTER_UCHAR( membase+MODE);
    uc &= ~CRD_DET;
    wWRITE_REGISTER_UCHAR( membase+MODE, uc);


    /*Reset now*/
    writew(RESET_EXE, membase+EXE);

    /*Wait for at least 1 microseconds*/
    mdelay(1);

    /*EXE.RESET_EXE = 0*/
    writew(0x0000, membase+EXE);

    /*Wait for at least 1 microseconds*/
    mdelay(1);

    /*MODE.EDC = 1; Since the SMCLIB will calculate the EDC for us*/
    uc = wREAD_REGISTER_UCHAR( membase+MODE);
    uc |= EDC | ATR_TO;
    wWRITE_REGISTER_UCHAR( membase+MODE, uc);
    wWRITE_REGISTER_ULONG( (PULONG)(membase+BWT_MSB), 64000 );

    /*Clear FIFO*/
    NTStatus = CmdClearFIFO(pRdrExt);
    if( NTStatus != STATUS_SUCCESS ) return NTStatus;

    wWRITE_REGISTER_UCHAR(membase+FRQ_MODE, FRQ_DIV << 4 );

    /*Ref. definition of O2_POWER_DELAY_REG for more information*/
    writew( 0XB00B, membase+O2_POWER_DELAY_REG);

#undef FUNC_NAME
#define FUNC_NAME   "CmdResetInterface: "
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "function complete\n");
#endif

    return ( NTStatus );
}

BYTE ATRFindProtocol(BYTE *Buf)
{

    BYTE    i=0, offset, protocol = 0, cnt=0, mask;

#ifdef PCMCIA_DEBUG
    int j;
    printk("ATRFindProtocol : start \n");
    printk("First data in buffer Buf[%d] = %02x \n", i, Buf[i]);
    for(j = 0; j < 0x20; j ++)
    {
        printk("[%02x] ", Buf[j]);
    }
    printk("\n");
#endif
    while ( (Buf[i] & bit_table[7]) )
    {
        offset = 1;
        mask = bit_table[6];
        for ( cnt = 0; cnt < 3; ++cnt )
        {
            if ( Buf[i] & mask )
                ++offset;
            mask >>= 1;
        }
        i += offset;
#ifdef PCMCIA_DEBUG
        printk("Buf[%d] = %02x \n", i, Buf[i]);
#endif
        protocol = (Buf[i] & bit_table[0]);
        break;
    }

#ifdef PCMCIA_DEBUG
    printk("Result protocol = %02x \n", protocol);
#endif
    return protocol;
}

short ATRFindTA1(BYTE *Buf, BYTE *Ta1)
{
    BYTE offset = 1;

    if ( !(Buf[1] & 0x10) )
        return -1;
    *Ta1 = Buf[offset+1];

    return G_OK;
}

short ATRFindTB1(BYTE *Buf, BYTE *Tb1)
{
    BYTE offset = 1;

    if ( !(Buf[1] & 0x20) )
        return -1;

    if ( Buf[1] & 0x10 )
        ++offset;

    *Tb1 = Buf[offset+1];

    return G_OK;
}

short ATRFindTC1(BYTE *Buf, BYTE *tc1)
{
    BYTE offset = 1;

    if ( !(Buf[1] & 0x40) )
        return -1;

    if ( Buf[1] & 0x10 )
        ++offset;

    if ( Buf[1] & 0x20 )
        ++offset;

    *tc1 = Buf[offset+1];

    return G_OK;
}

USHORT ATRFindTA2(BYTE *Buf, BYTE *ta2)
{
    short n=1, offset = 0, TAi = 1;
    BYTE k;

    if ( !(Buf[n] & 0x80) )
        return -1;

    offset = 0;
    TAi = 1;

    while (Buf[n] & 0x80)
    {
        offset = 0;
        for(k = 0x10; k > 0;  k <<=1)
        if (Buf[n] & k)
            offset++;
        n += offset;

        TAi++;
        if (TAi == 2)
            break;
    };


    if ( TAi != 2 )
        return -1;

    if (Buf[n] & 0x10)	
        *ta2 = Buf[n+1];
    else
        return -1;

    return G_OK;
}

USHORT ATRFindTB2(BYTE *Buf, BYTE *tb2)
{
    short n=1, offset = 0, TBi = 1;
    BYTE k;

    if ( !(Buf[n] & 0x80) )	return -1;
    offset = 0;
    TBi = 1;


    while (Buf[n] & 0x80)
    {
        offset = 0;
        for(k = 0x10; k > 0;  k <<=1)
        if (Buf[n] & k)
           offset++;

        n += offset;
        TBi++;
        if (TBi == 2)
            break;
    };

    if ( TBi != 2 )
        return -1;

    if ( !(Buf[n] & 0x20) )
        return -1;

    offset = 1;

    if ( Buf[n] & 0x10 )
        offset ++;

    *tb2 = Buf[n+offset];

    return G_OK;
}

USHORT ATRFindTC2(BYTE *Buf, BYTE *tc2)
{
    short n=1, offset = 0, TCi = 1;
    BYTE k;

    if ( !(Buf[n] & 0x80) )
        return -1;

    offset = 0;
    TCi = 1;

    while (Buf[n] & 0x80)
    {
        offset = 0;
        for(k = 0x10; k > 0;  k <<=1)
            if (Buf[n] & k)
               offset++;

        n += offset;
        TCi++;
        if (TCi == 2)
            break;

    };


    if ( TCi != 2 )
        return -1;

    if ( !(Buf[n] & 0x40) )
        return -1;

    offset = 1;

    if ( Buf[n] & 0x10 )
        offset ++;

    if ( Buf[n] & 0x20 )
        offset ++;

    *tc2 = Buf[n+offset];

    return G_OK;
}

USHORT ATRFindIFSC(BYTE *Buf)

{
    USHORT ifsc = 0;
    short n=1, offset = 0, TAi = 1;
    BYTE k;

    if ( !(Buf[n] & 0x80) )
        return 32;

    offset = 0;
    TAi = 1;

    while (Buf[n] & 0x80)
    {
        offset = 0;
        for(k = 0x10; k > 0;  k <<=1)
            if (Buf[n] & k)
                offset++;

        n += offset;
        TAi++;

        if (TAi == 3)
            break;

    };

    if ( TAi < 3 )
        return 32; /* default IFSC */

    if (Buf[n] & 0x10)
        ifsc = (USHORT)(Buf[n+1] );

    if ( ifsc < 5 )
        if ( ifsc < 32 )
            ifsc = 32;

    return ifsc;
}


USHORT ATRFindTB3(BYTE *Buf, BYTE *tb3)
{

    short n=1, offset = 0, TBi = 1;
    BYTE k;

    if ( !(Buf[n] & 0x80) )	return -1;

    offset = 0;
    TBi = 1;

    while (Buf[n] & 0x80)
    {
        offset = 0;
        for(k = 0x10; k > 0;  k <<=1)
            if (Buf[n] & k)
               offset++;

        n += offset;
        TBi++;

        if (TBi == 3)
            break;
    };

    if ( TBi != 3 )
        return -1;

    if ( !(Buf[n] & 0x20) )
        return -1;

    offset = 1;

    if ( Buf[n] & 0x10 )
        offset ++;

    *tb3 = Buf[n+offset];

    return G_OK;
}


USHORT ATRFindTC3(BYTE *Buf, BYTE *tc3)
{
    short n=1, offset = 0, TCi = 1;

    BYTE k;

    if ( !(Buf[n] & 0x80) )
        return -1;

    offset = 0;
    TCi = 1;

    while (Buf[n] & 0x80)
    {
        offset = 0;
        for(k = 0x10; k > 0;  k <<=1)
            if (Buf[n] & k)
                offset++;

        n += offset;
        TCi++;

        if (TCi == 3)
            break;
    };

    if ( TCi != 3 )
        return -1;

    if ( !(Buf[n] & 0x40) )
        return -1;

    offset = 1;

    if ( Buf[n] & 0x10 )
        offset ++;

    if ( Buf[n] & 0x20 )
        offset ++;

    *tc3 = Buf[n+offset];

    return G_OK;
}

short InitATRParam(PREADER_EXTENSION pRdrExt)
{
    USHORT ifsc;
    BYTE ta1,tb1,tc1,ta2,tb2,tc2,tb3,tc3;


    pRdrExt->m_SCard.Protocol = ATRFindProtocol(pRdrExt->m_SCard.ATR+1);


    if(pRdrExt->m_SCard.RqstProtocol == 0x01)
    {
        pRdrExt->m_SCard.Protocol = 0x00;  

    }
    else
    {
        if(pRdrExt->m_SCard.RqstProtocol == 0x02)
        {
            pRdrExt->m_SCard.Protocol = 0x01;  
        }
    }

    pRdrExt->m_SCard.EdcType = 0;


    pRdrExt->m_SCard.AtrTA1 = 0xFFFF;
    pRdrExt->m_SCard.TA1 = 0x11;

    if ( ATRFindTA1( pRdrExt->m_SCard.ATR, &ta1 ) == G_OK )
        pRdrExt->m_SCard.AtrTA1 = ta1;


    pRdrExt->m_SCard.TB1 = 0xFFFF;

    if ( ATRFindTB1(pRdrExt->m_SCard.ATR, &tb1) == G_OK )
        pRdrExt->m_SCard.TB1 = tb1;


    pRdrExt->m_SCard.TC1 = 0xFFFF;

    if ( ATRFindTC1(pRdrExt->m_SCard.ATR, &tc1) == G_OK )
        pRdrExt->m_SCard.TC1 = tc1;


    if ( ATRFindTA2(pRdrExt->m_SCard.ATR, &ta2) == G_OK )
    {

        pRdrExt->m_SCard.TA2 = ta2;
        pRdrExt->m_SCard.Protocol = ta2 & 0x01; /*Only supp T=0/1 protocol*/
    }

    if ( ATRFindTB2(pRdrExt->m_SCard.ATR, &tb2) == G_OK )

        pRdrExt->m_SCard.TB2 = tb2;


    pRdrExt->m_SCard.TC2 = 0xFFFF;

    if ( ATRFindTC2(pRdrExt->m_SCard.ATR, &tc2) == G_OK )
        pRdrExt->m_SCard.TC2 = tc2;

    ifsc = ATRFindIFSC(pRdrExt->m_SCard.ATR);
    pRdrExt->T1.ifsc = ifsc;

    pRdrExt->T1.ns = 0;
    pRdrExt->T1.nr = 0;
    pRdrExt->T1.nad = 0;

    if ( ATRFindTB3(pRdrExt->m_SCard.ATR, &tb3) == G_OK )
        pRdrExt->m_SCard.TB3 = tb3;


    pRdrExt->m_SCard.TC3 = 0xFFFF;

    if ( ATRFindTC3(pRdrExt->m_SCard.ATR, &tc3) == G_OK )
        pRdrExt->m_SCard.TC3 = tc3;

    if( tc3 & 0x40 ) 
    {

        if( tc3 & 0x01 ) 
            pRdrExt->T1.rc = SC_T1_CHECKSUM_CRC;
        else 
            pRdrExt->T1.rc = SC_T1_CHECKSUM_LRC;
    }

    pRdrExt->m_SCard.PowerOn = TRUE;
    pRdrExt->m_SCard.WakeUp = TRUE;

    return G_OK;
}


NTSTATUS AsyT1Ini(PREADER_EXTENSION pRdrExt)
{
    ULONG   cwt, BwtT1;
    USHORT  Cgt;

    USHORT  etu_wrk;
    USHORT  F, D;
    BYTE    tc3;
    BYTE    ta1 = (BYTE)(pRdrExt->m_SCard.TA1 & 0xff);
    PUCHAR  membase = pRdrExt->membase;

#undef FUNC_NAME 
#define FUNC_NAME   "AsyT1Ini: "
#ifdef PCMCIA_DEBUG
printk(KERN_INFO MODULE_NAME FUNC_NAME "Init Start\n");
#endif

    if ( pRdrExt->m_SCard.TA1 != 0xffff )
    {
        F = (ta1 & 0xf0) >> 4;	
        F &= 0x000f;
        D = ta1 & 0x000f;
        F = Fi[F];
        D = Di[D];

        if ( F == 0xffff || D == 0xffff )
        {
            F = 372;
            D = 1;
        }

    }
    else
    {
         F = 372;
         D = 1;
    }

    Cgt= 0;
    if( pRdrExt->m_SCard.TC1 !=  0xffff )
        Cgt= pRdrExt->m_SCard.TC1 & 0x00ff;
    writew( (Cgt + 1) , membase+CGT );

    etu_wrk = (((F*10)/D)+5)/10;
    writew( etu_wrk, membase+ETU_WRK );

    cwt= 13;

    BwtT1= 4;
    if ( pRdrExt->m_SCard.TB3 != 0xffff )
    {
        BYTE tb3 = (BYTE)(pRdrExt->m_SCard.TB3 & 0xff);
        cwt= tb3 & 0x0f;

        BwtT1 = ( (pRdrExt->m_SCard.TB3 & 0xf0) >> 4 );

        if( BwtT1 > 9)
            return (GE_II_ATR_PARM);
    }
    pRdrExt->m_SCard.Bwi = (USHORT) BwtT1;
    cwt= 11+(1<<cwt);
    BwtT1= ((1<<BwtT1)*960*372)/F;
    BwtT1= BwtT1*D+11;

    tc3 = 0;
    if ( pRdrExt->m_SCard.TC3 == 0xffff )
    {
        pRdrExt->m_SCard.TC3 = 0;
    }
    else
    {
        tc3 = (BYTE) (pRdrExt->m_SCard.TC3 & 0x00ff );
        if ( pRdrExt->m_SCard.TC3 & 0x00fe )
            return (GE_II_ATR_PARM);
    }

    if ( tc3 )
    {
        writew( 0x81 | PROTO_EDC_TYP, membase+PROTO );
        pRdrExt->m_SCard.EdcType = 1;
    }
    else
    {
        writew( 0x81, membase+PROTO );
        pRdrExt->m_SCard.EdcType = 0;
    }

    writew( (USHORT) (cwt / 0x10000), membase+CWT_MSB );
    writew( (USHORT) (cwt % 0x10000), membase+CWT_LSB );
    writew( (USHORT) (BwtT1/0x10000), membase+BWT_MSB );
    writew( (USHORT) (BwtT1%0x10000), membase+BWT_LSB );
    pRdrExt->T1.bwt = BwtT1;

#undef FUNC_NAME
#define FUNC_NAME   "AsyT1Ini: "
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "Init Complete\n");
#endif

    return G_OK;
}




NTSTATUS AsyT0Ini(PREADER_EXTENSION pRdrExt)

{
    USHORT F, D;
    BYTE ta1;
    USHORT etu_wrk;
    ULONG cwt;
    USHORT Cgt= 0;
    PUCHAR membase = pRdrExt->membase;

#undef FUNC_NAME
#define FUNC_NAME   "AsyT0Ini: "

#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "Init Start\n");
#endif


    writew( PROTO_CP | PROTO_RC, membase+PROTO );

    ta1 = (BYTE)(pRdrExt->m_SCard.TA1 & 0xff);
    if ( pRdrExt->m_SCard.TA1 != 0xffff )
    {
        F = (ta1 & 0xf0) >> 4;
        F &= 0x000f;
        D = ta1 & 0x000f;
        F = Fi[F];

        D = Di[D];

        if ( F == 0 )
        {
            F = 372;
            D = 1;
        }
    }
    else
    {
        F = 372;
        D = 1;
    }

    if( pRdrExt->m_SCard.TC1 !=  0xffff )
        Cgt= pRdrExt->m_SCard.TC1 & 0x00ff;
    writew( (Cgt + 1), membase+CGT );

    etu_wrk = (((F*10)/D)+5)/10;
    writew( etu_wrk, membase+ETU_WRK );

    if ( pRdrExt->m_SCard.TC2 == 0xffff )
        cwt = 10;
    else
        cwt = pRdrExt->m_SCard.TC2;

    if ( pRdrExt->m_SCard.TA1 == 0xffff )
        cwt = cwt * 1;
    else
        cwt = (((cwt*D*F*10)/F)+5)/10;

    cwt *= 960;
    writew( (USHORT) (cwt / 0x10000), membase+CWT_MSB );
    writew( (USHORT) (cwt % 0x10000), membase+CWT_LSB );

#undef FUNC_NAME
#define FUNC_NAME   "AsyT0Ini: "
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "Init Complete\n");
#endif

    return G_OK;
}


static void RecievMode(void)

{
    /*Receiving Mode*/
    wWRITE_REGISTER_UCHAR( sync_membase+MANUAL_E_R, 0xE0); 

    mdelay(1);
}


static void EmmitionMode(void)
{
    wWRITE_REGISTER_UCHAR( sync_membase+MANUAL_E_R, 0x60);
}

static void ManulMode(void) 
{
    UCHAR uc;
    uc = wREAD_REGISTER_UCHAR( sync_membase+MODE);
    uc |= MANUAL;

    wWRITE_REGISTER_UCHAR( sync_membase+MODE, uc);

    RecievMode();
}

static void VCCOn(void)
{
    /*VCC - Power On*/
    wWRITE_PORT_UCHAR( &sync_IOBase->MANUAL_IN, 0x80);
    mdelay(1);
    ct = wREAD_PORT_UCHAR( &sync_IOBase->MANUAL_OUT );
}

void VCCOff(void)
{
    wWRITE_PORT_UCHAR( &sync_IOBase->MANUAL_IN, 0x00);

    mdelay(1);
    ct = wREAD_PORT_UCHAR( &sync_IOBase->MANUAL_OUT );
}

static void delay(void)

{
}

static void low( int v )
{
    ct &=  ~v;
    wWRITE_PORT_UCHAR( &sync_IOBase->MANUAL_IN, ct );
    delay();
}


static void high( int v )
{

    ct |= v;
    wWRITE_PORT_UCHAR( &sync_IOBase->MANUAL_IN, ct );
    delay();
}


static BOOLEAN readIO(void)

{
    return (wREAD_PORT_UCHAR( &sync_IOBase->MANUAL_OUT ) & IO ) != 0;
}

static void CommandEntry(BYTE control, BYTE address, BYTE data)
{
    DWORD controlDWord, dword, temp=0x01;
    UCHAR i;

    dword = 0 | data;

    controlDWord = 0 | ( dword << 16 );
    dword = 0 | address;
    controlDWord |= ( dword << 8 );

    dword = 0 | control;
    controlDWord |= dword;

    low(RST);
    mdelay(1);
    EmmitionMode();

    mdelay(1);
    high(RST);
    for( i = 0; i < 24; i++ )
    {
        if( controlDWord & temp )
            high(IO);
        else
            low(IO);

        mdelay(1);
        high(CLK);

        mdelay(1);
        low(CLK);
        temp <<= 1;

    }
    RecievMode();
    mdelay(1);
    low(RST);
}

static BYTE readByte(WORD addressWord)
{  
    BYTE control = 0x0E;
    BYTE address, data;
    UCHAR i, temp=0x01;
    data = 0;
    address = (BYTE) ( (addressWord >> 8) << 6 );
    control = control | address;
    address = (BYTE) (0xFF & addressWord);
    CommandEntry( control, address, 0x00 );

    mdelay(1);
    ManulMode();
    mdelay(1);
    high(CLK);
    mdelay(1);
    low(CLK);
    for( i = 0; i < 8; i++)
    {
        mdelay(1);
        high(CLK);

        if( readIO() )
            data |= temp;
        mdelay(1);
        low(30);
        temp <<= 1;
    }
    mdelay(1);
    high(RST);

    return data;
}

static NTSTATUS writeByte(WORD addressWord, BYTE data)
{
    BYTE control = 0x33;
    BYTE address;
    UCHAR i, duraCycle=203, chkCycle=200;
    BYTE ackData = 1;
    BYTE oldData;
    BOOLEAN writeOnly = TRUE;
    NTSTATUS NTStatus = STATUS_UNSUCCESSFUL;
    BYTE bittable[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};

    address = (BYTE) ( (addressWord >> 8) << 6 );
    control = control | address;
    address = (BYTE) (0xFF & addressWord);
    oldData = readByte( addressWord );

    if( data == oldData )
    {
        NTStatus = STATUS_SUCCESS;
    }
    else
    {
        if( (data==0) || (data==0xFF) || (oldData==0xFF) )
        {
            duraCycle = 103;
        }
        else
        {
            for(i = 0; i < 8; i++)
            {
                if(!(oldData & bittable[i]))
                {
                    if(data & bittable[i])
                    {
                        writeOnly = FALSE;
                        break;
                    }
                }
            }

        }

        if( writeOnly )
            duraCycle = 103;
        chkCycle = duraCycle-3;

        CommandEntry( control, address, data );
        mdelay(1);
        for(i = 0; i < duraCycle; i++)
        {
            mdelay(1);
            high(CLK);
            if( i==chkCycle )
            {
                ackData = readIO();
                if( ackData )
                    NTStatus = STATUS_SUCCESS;
            }
            mdelay(1);
            low(CLK);
        }
    }
 

    return NTStatus;
}


/*
 *  Sub Function: Sync. Card Reset Power On
 */
static void SyncReset( UCHAR astr[4] )
{
    int i, j;
    UCHAR c;
    UCHAR temp;

#undef FUNC_NAME	
#define FUNC_NAME   "Sync PowerReset: "
#ifdef PCMCIA_DEBUG

printk(KERN_INFO MODULE_NAME FUNC_NAME "Function Start.\n");
#endif

    ManulMode();
    VCCOn();
    mdelay(1);
    high( RST );

    mdelay(1);
    high( CLK );
    mdelay(1);
    low( CLK );

    mdelay(1);

    low( RST );

    mdelay(1);

    for(i=0; i < 4; i++ )
    {
        c = 0;
        temp = 0x01;
        astr[i] = 0;
        for( j = 0; j < 8; j++ )
        {
            high(CLK);

            if( readIO() )
                c |= temp;

            mdelay(1);
            low(CLK);
            mdelay(1);
            temp <<= 1;
        }

        astr[i] = c;

#ifdef PCMCIA_DEBUG
        printk(KERN_INFO " %02X ", c);
#endif
    }
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO "\n");
#endif
}


/*
	Sub Function: COLD/HOT/GETATRBACK
	Description: Single subroutine for Cold/Hot Reset and Get ATR back
	Procedure:
        End Of Subfunction
*/
NTSTATUS
CmdResetReader(
   PREADER_EXTENSION	pRdrExt,
   BOOLEAN		WarmReset,	/*TRUE: Warm, FALSE: cold Reset */
   PUCHAR		pATR,
   PULONG		pATRLength)
{
    NTSTATUS        NTStatus = STATUS_IO_DEVICE_ERROR;
    PPSCR_REGISTERS IOBase;
    PUCHAR          membase;

    USHORT          status_exch;
    BYTE	    protocol;



    IOBase = pRdrExt->IOBase;

    membase = pRdrExt->membase;

#ifdef PCMCIA_DEBUG
    printk("original protocol before reset: pRdrExt->m_SCard.Protocol = %d \n", pRdrExt->m_SCard.Protocol);
#endif    
    /*Clear FIFO before reset the card*/
    NTStatus  = CmdClearFIFO(pRdrExt);

    if( NTStatus != STATUS_SUCCESS )
        return (NTStatus);

    switch( WarmReset )
    {
        case TRUE:/*Hot Reset*/
#undef FUNC_NAME

#define FUNC_NAME   "CmdResetReader: "
#ifdef PCMCIA_DEBUG
            printk(KERN_INFO MODULE_NAME FUNC_NAME "WarmReset\n");
#endif

            /*Restore some resetting default value before Warm Reset*/
            wWRITE_REGISTER_ULONG( (PULONG)(membase+CWT_MSB), 0x2580 );
            wWRITE_REGISTER_ULONG( (PULONG)(membase+BWT_MSB), 0xfa00 );
            NTStatus = CmdExecute( pRdrExt, RST_EXE, &status_exch );
            break;
        case FALSE: /*Cold Reset*/
#undef FUNC_NAME
#define FUNC_NAME   "CmdResetReader: "

#ifdef PCMCIA_DEBUG
            printk(KERN_INFO MODULE_NAME FUNC_NAME "ColdReset\n");
#endif
            /*Turn off the power if it is on*/
            NTStatus = CmdDeactivate( pRdrExt );
            CmdResetInterface( pRdrExt );

            /*EXE.PON_EXE = 1 'Power on and reset sequence*/
            CmdExecuteWithNoWait( pRdrExt, PON_EXE );

            /*Since the command is not excute until power ready
             we need wait the command finished*/
            NTStatus = CmdWaitForEndOfCmd(pRdrExt,&status_exch,CMD_MAX_DELAY);

            /* Check if STATUS_EXCH*/
            if( status_exch & 0xF700 )
                NTStatus = STATUS_IO_DEVICE_ERROR;

#undef FUNC_NAME
#define FUNC_NAME   "CmdResetReader: "
#ifdef PCMCIA_DEBUG
            printk(KERN_INFO MODULE_NAME FUNC_NAME " status_exch=%04X \n", status_exch );
#endif
            break;
        default:
            break;
    }

    if( NTStatus != STATUS_SUCCESS )  
    {
#undef FUNC_NAME

#define FUNC_NAME   "CmdResetReader: "
#ifdef PCMCIA_DEBUG
        printk(KERN_INFO MODULE_NAME FUNC_NAME "Sync. Card Reset!!! \n");
#endif
        sync_IOBase = IOBase;
        sync_membase = membase;
        pRdrExt->m_SCard.Protocol = 0x03;

        SyncReset(pATR);
        *pATRLength = 4;

        NTStatus = STATUS_SUCCESS;

        return (NTStatus);
    }

    ReadATR( pRdrExt, pATR, pATRLength);

    InitATRParam( pRdrExt );

    protocol = pRdrExt->m_SCard.Protocol & bit_table[0];
    if( protocol )
    {
#ifdef PCMCIA_DEBUG
        printk("AsyT1Ini() is called \n");
#endif        
        AsyT1Ini( pRdrExt );
    }
    else
    {
#ifdef PCMCIA_DEBUG
        printk("AsyT0Ini() is called \n");
#endif        
        AsyT0Ini( pRdrExt );
    }

    if( NTStatus != STATUS_SUCCESS )
        *pATRLength = 0;

    return NTStatus;
}


/*++
CmdDeactivate:

	Description: Power off the card
	Procedure:
	'Following condition must meet before a Power off is possible
	'Otherwise it do nothing
	STATUS_EXCH.CRD_INS == 1 'Card Inserted
	STATUS_EXCH.CRD_ON == 1 'Card is powered


	EXE.POF_EXE = 1 'Power off

	END OF PROCEDURE


Arguments:
	ReaderExtension		context of call
	Device				requested device


Return Value:
	STATUS_SUCCESS
	error values from PscrRead / PscrWrite
--*/
NTSTATUS
CmdDeactivate( PREADER_EXTENSION pRdrExt )

{
    NTSTATUS NTStatus = STATUS_SUCCESS;

    PUCHAR membase;
    USHORT us;
    USHORT status_exch;
    membase = pRdrExt->membase;

    us = readw( (PUSHORT)(membase+STATUS_EXCH));
    if( (us & (CRD_ON | CRD_INS ) ) == ( CRD_ON | CRD_INS ) ) 
    {
        /*Power off*/
        NTStatus = CmdExecute( pRdrExt, POF_EXE, &status_exch );

        mdelay(1);
        us = readw( membase+STATUS_EXCH );
        if( us & CRD_ON ) 
            NTStatus = STATUS_UNSUCCESSFUL;
        else
            NTStatus = STATUS_SUCCESS;
    }

    return( NTStatus );
}

NTSTATUS PscrWriteDirect( PREADER_EXTENSION pRdrExt, PUCHAR pData, ULONG DataLen, PULONG pNBytes )
{
    NTSTATUS NTStatus = STATUS_IO_DEVICE_ERROR;
    PPSCR_REGISTERS IOBase;
    ULONG Idx;
    USHORT status_exch, us;
    PUCHAR membase;
    
#undef FUNC_NAME

#define FUNC_NAME " PscrWriteDirect : "
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "Function Start \n");
#endif

    membase = pRdrExt->membase;
    IOBase = pRdrExt->IOBase;
    if( pNBytes )
        *pNBytes = 0;

    /* S_TOC_EXE just in case some command is still running */
    CmdExecuteWithTimeOut( pRdrExt, S_TOC_EXE , &status_exch, CMD_MAX_DELAY );

    NTStatus = CmdClearFIFO( pRdrExt );
    if( NTStatus != STATUS_SUCCESS )
    { 
#ifdef PCMCIA_DEBUG
        printk("ERROR #7 :: CmdClearFIFO() faid in PscrWriteDirect() \n");
#endif
        return( NTStatus );
    }

#ifdef PCMCIA_DEBUG
    printk("WriteFIFO :");
    for (Idx = 0; Idx < DataLen; Idx++)
        printk(" [%02X]",pData[ Idx ]);
    printk("\n");
#endif

    /*Write Data into FIFO*/
    for (Idx = 0; Idx < DataLen; Idx++)
    {
        wWRITE_PORT_UCHAR( &IOBase->FIFO_IN, pData[ Idx ] );
    }

    us = readw( membase+STATUS_EXCH );
    if( us & FIFO_FULL)
    {
#ifdef PCMCIA_DEBUG
        printk(" ERROR # FIFO FULL :: readw() in PscrWriteDirect() \n");
#endif
        return( STATUS_UNSUCCESSFUL );

    }

    /* Check if Number of Bytes is correct */
    us = readw( membase+FIFO_NB );
    if( us != DataLen )
    {
#ifdef PCMCIA_DEBUG
        printk(" ERROR # FIFO NB :: readw() in PscrWriteDirect() \n");
#endif
        return( STATUS_UNSUCCESSFUL );
    }

    /*Tell user the byte number of Data been write to FIFO Succesfuly*/
    *pNBytes = DataLen; 

    CmdExecuteWithTimeOut( pRdrExt, S_TOC_EXE , &status_exch, CMD_MAX_DELAY );

    NTStatus = CmdExecuteWithTimeOutAndTOC( pRdrExt, EXCH_EXE, &status_exch, CMD_MAX_DELAY, TRUE );

    if( status_exch & ( BAD_TS | BAD_PB | ERR_PAR | ERR_EXE ) )
    {
#ifdef PCMCIA_DEBUG
        printk(" ERROR #BAD_TS|BAD_PB|ERR_PAR|ERR_EXE :: CmdExecuteWithTimeOutAndTOC() in PscrWriteDirect() \n");
#endif
        NTStatus = STATUS_IO_DEVICE_ERROR;
    }
    else
    {
        if( status_exch & ( TOB | TOR ) ) /* Randy temp. don't care TOC flag */
        {
#ifdef PCMCIA_DEBUG
            printk(" ERROR # TOB | TOR :: CmdExecuteWithTimeOutAndTOC() in PscrWriteDirect() \n");
#endif
            NTStatus = STATUS_IO_TIMEOUT;
        }
    }

    CmdExecuteWithTimeOut( pRdrExt, S_TOC_EXE , &status_exch, CMD_MAX_DELAY );

#undef FUNC_NAME
#define FUNC_NAME " PscrWriteDirect : "

#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "Function Complete \n");
#endif

    return( NTStatus ); 

}

NTSTATUS PscrWriteDirectWithFIFOLevel( PREADER_EXTENSION pRdrExt, PUCHAR pData, ULONG DataLen, PULONG pNBytes, USHORT fifoLev )
{
    NTSTATUS NTStatus = STATUS_IO_DEVICE_ERROR;
    PPSCR_REGISTERS IOBase;
    ULONG Idx;
    USHORT status_exch;
    USHORT us;
    PUCHAR membase; 

    UCHAR uc;
    int timeoutcount = 0;  

#undef FUNC_NAME

#define FUNC_NAME " PscrWriteDirectWithFIFOLevel : "
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "Function Start \n");
#endif

    membase = pRdrExt->membase;
    IOBase = pRdrExt->IOBase;

    if( pNBytes )
        *pNBytes = 0;

    CmdExecuteWithTimeOut( pRdrExt, S_TOC_EXE , &status_exch, CMD_MAX_DELAY );

    NTStatus = CmdClearFIFO(pRdrExt);

    if( NTStatus != STATUS_SUCCESS )
        return( NTStatus );

    if( DataLen < 5 )
    {
        DataLen = 5;
        pData[4] = 0;
    }

    /* Write Data into FIFO */
    for (Idx = 0; Idx < DataLen; Idx++)
        wWRITE_PORT_UCHAR( &IOBase->FIFO_IN, pData[ Idx ] );

    us = readw( membase+STATUS_EXCH );
    if( us & FIFO_FULL)
        return( STATUS_UNSUCCESSFUL );

    /* Check if Number of Bytes is correct */
    us = readw( membase+FIFO_NB );

    if( us != DataLen )
        return( STATUS_UNSUCCESSFUL );

    /* Tell user the byte number of Data been write to FIFO Succesfuly */
    *pNBytes = DataLen; 

    /* EXE.EXCH_EXE = 1 	'Launch the Exchange command */
    CmdExecuteWithNoWait( pRdrExt, EXCH_EXE );

    /* Wait for the command finish, FIFO_NB==0 */
    if( fifoLev <= DataLen )
    {
        for( timeoutcount = CMD_MAX_DELAY; timeoutcount > 0; timeoutcount -- )
        {
            /* Check STATUS_IT.End_EXE to see if command complete */
            uc = wREAD_REGISTER_UCHAR( membase+STATUS_IT );
            if( (uc & END_EXE ) == END_EXE ) 
            {
                NTStatus = STATUS_SUCCESS;
                /* Clear correspond bit in DEVAL_IT */
                wWRITE_REGISTER_UCHAR( membase+DEVAL_IT, ~END_EXE_CLR_B );
                break;
            }

            us = readw( membase+FIFO_NB );
            if( us == 0 )
                 break;

            mdelay(1);
        } 
    }

    /* Wait command END_EXE or FIFO_NB==fifoLev */
    for( timeoutcount = CMD_MAX_DELAY; timeoutcount > 0; timeoutcount -- )
    {
        /* Check STATUS_IT.End_EXE to see if command complete */
        uc = wREAD_REGISTER_UCHAR( membase+STATUS_IT );
        if( (uc & END_EXE ) == END_EXE ) 
        {
            NTStatus = STATUS_SUCCESS;
            /* Clear correspond bit in DEVAL_IT */
            wWRITE_REGISTER_UCHAR( membase+DEVAL_IT, ~END_EXE_CLR_B );
            break;
        }

        us = readw( membase+FIFO_NB );
        if( us == fifoLev )
            break;

        mdelay(1);
    } 

    /* Check what kind of error we got */
    status_exch = readw( membase+STATUS_EXCH );
    if( status_exch & ( BAD_TS | BAD_PB | ERR_PAR | ERR_EXE ) )
        NTStatus = STATUS_IO_DEVICE_ERROR;
    else if( status_exch & ( TOC | TOB | TOR ) )
        NTStatus = STATUS_IO_TIMEOUT;

    /* S_TOC_EXE just in case command is termiate by FIFO_LEVEL_TRIGERED */
    CmdExecuteWithTimeOut( pRdrExt, S_TOC_EXE , &status_exch, CMD_MAX_DELAY );

#undef FUNC_NAME
#define FUNC_NAME " PscrWriteDirectWithFIFOLevel : "
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "Function Complete \n");
#endif

    return( NTStatus );
}

/*++
CBT0Transmit:
    finishes the callback RDF_TRANSMIT for the T0 protocol
Arguments:
    pRdrExt  context of call
Return Value:
    STATUS_SUCCESS
    STATUS_NO_MEDIA
    STATUS_TIMEOUT
    STATUS_INVALID_DEVICE_REQUEST
--*/
NTSTATUS CBT0Transmit( PREADER_EXTENSION pRdrExt )
{
    NTSTATUS NTStatus = STATUS_SUCCESS;
    PUCHAR pRequest, pReply;
    ULONG IOBytes = 0;
    ULONG RequestLength;
    USHORT T0ExpctedReturnLen;


#undef FUNC_NAME
#define FUNC_NAME " CBT0Transmit : "
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "Function Start \n");
#endif

    pRequest = pRdrExt->SmartcardRequest.Buffer;
    pReply = pRdrExt->SmartcardReply.Buffer;

    /* Transmit pRdrExt->SmartcardRequest.Buffer to smart card */
    RequestLength = pRdrExt->SmartcardRequest.BufferLength;

    pRdrExt->SmartcardReply.BufferLength = 0;
#ifdef PCMCIA_DEBUG
    printk("CBT0 :: RequestLength = %ld \n", RequestLength);
#endif
    /* Since H/W may not STOP the command automaticaly. Check Return Byte */
    if( RequestLength >= 5 )
        T0ExpctedReturnLen = pRequest[4];
    else
        T0ExpctedReturnLen = 0;

    if( T0ExpctedReturnLen == 0 )
        T0ExpctedReturnLen = 256;

    T0ExpctedReturnLen += 2; /*SW1 and SW2*/

    if( RequestLength > 5 )
    {
        NTStatus = PscrWriteDirect(pRdrExt, pRequest, RequestLength, &IOBytes);
#ifdef PCMCIA_DEBUG
        printk("RequestLength > 5 : code 048 \n");
#endif
    }
    else
    {
        NTStatus = PscrWriteDirectWithFIFOLevel(pRdrExt, pRequest, RequestLength, &IOBytes, T0ExpctedReturnLen );
#ifdef PCMCIA_DEBUG
        printk("RequestLength <= 5 : code 049 \n");
#endif
    }

    if( NTStatus == STATUS_SUCCESS )
    {
        IOBytes = MAX_T1_BLOCK_SIZE;

        NTStatus = ReadFIFO( pRdrExt, pReply, &IOBytes );
        if( NTStatus == STATUS_SUCCESS )
            pRdrExt->SmartcardReply.BufferLength = IOBytes;
#ifdef PCMCIA_DEBUG
        else
            printk("ReadFIFO() fail CBT0 : code 051 \n");
#endif
    }
#ifdef PCMCIA_DEBUG
    else
    {
        printk("Fail CBT0 : code 050 \n");
    }
#endif

#undef FUNC_NAME
#define FUNC_NAME " CBT0Transmit : "
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "Function Complete \n");
#endif

    return( NTStatus );
}

/* Returns LRC of data */
BYTE scT1Lrc( BYTE *data, int datalen)
{
    BYTE lrc=0x00;
    int i;

    for( i=0; i<datalen; i++ ) lrc^=data[i];
        return lrc;
}

/* Calculates CRC of data */
void scT1Crc( BYTE *data, int datalen, BYTE *crc )
{
    int i;
    WORD tmpcrc=0xFFFF;

    for (i=0; i<datalen; i++)
    tmpcrc = ((tmpcrc >> 8) & 0xFF) ^ crctab[(tmpcrc ^ *data++) & 0xFF];
    crc[0]=(tmpcrc >> 8) & 0xFF;
    crc[1]=tmpcrc & 0xFF;
}

/* Checks RC. */
BOOLEAN scT1CheckRc( PREADER_EXTENSION pRdrExt, PUCHAR data, PULONG datalen )
{
    BYTE rc[2];
    BYTE cmp[2];

    if( pRdrExt->T1.rc == SC_T1_CHECKSUM_LRC ) 
    {
        /* Check LEN. */
        if( (data[2]+3+1) != *datalen )
        {
#ifdef PCMCIA_DEBUG
            printk("(data[2]+3+1) != *datalen: code 040 \n");
#endif
            return( FALSE );
        }
        rc[1] = data[*datalen-1];
        cmp[1] = scT1Lrc( data, *datalen-1 );

        if( rc[1]==cmp[1] )
        { 
#ifdef PCMCIA_DEBUG
            printk("rc[1]==cmp[1] return TRUE: code 041 \n");
#endif
            return( TRUE );
        }
#ifdef PCMCIA_DEBUG
        printk("if SC_T1_CHECKSUM_LRC failed: code 042 \n");
#endif
        return( FALSE );
    }
    else if( pRdrExt->T1.rc == SC_T1_CHECKSUM_CRC)
    {
        /* Check LEN. */
        if( (data[2]+3+2) != *datalen )
        { 
#ifdef PCMCIA_DEBUG
            printk("data[2]+3+2) != *datalen: code 043 \n");
#endif
            return( FALSE );
        }
        scT1Crc( data, *datalen-2, cmp );
        if( memcmp( data+(*datalen)-2, cmp, 2 )==0 )
        {
#ifdef PCMCIA_DEBUG
            printk("memcmp( data+(*datalen)-2, cmp, 2 )==0 return TRUE: code 044 \n");
#endif
            return( TRUE );
        }
#ifdef PCMCIA_DEBUG
        printk("if SC_T1_CHECKSUM_CRC failed: code 045 \n");
#endif
        return( FALSE );
    }
#ifdef PCMCIA_DEBUG
    printk("scT1CheckRc failed: 046 \n");
#endif
    return( FALSE );
}




/* Appends RC */
NTSTATUS scT1AppendRc( PREADER_EXTENSION pRdrExt, PUCHAR data, PULONG datalen )
{
    if( pRdrExt->T1.rc == SC_T1_CHECKSUM_LRC )
    {
        data[*datalen]=scT1Lrc( data, *datalen );
        *datalen+=1;
#ifdef PCMCIA_DEBUG
        printk("SC_T1_CHECKSUM_LRC: code 038 \n");
#endif
        return( SC_EXIT_OK );
    }
    else if( pRdrExt->T1.rc == SC_T1_CHECKSUM_CRC )
    {
        scT1Crc( data, *datalen, data+*datalen );
        *datalen+=2;
#ifdef PCMCIA_DEBUG
        printk("SC_T1_CHECKSUM_CRC: code 039 \n");
#endif
        return( SC_EXIT_OK );
    }
#ifdef PCMCIA_DEBUG
    printk("scT1AppendRc() failed: code 037 \n");
#endif
    return( SC_EXIT_BAD_PARAM );

}


/* Builds S-Block */
NTSTATUS scT1SBlock( PREADER_EXTENSION pRdrExt, int type, int dir, int param,
                     PUCHAR block, PULONG pLen )
{
    NTSTATUS    NTStatus = STATUS_SUCCESS;

    block[0] = pRdrExt->T1.nad;
    switch( type )
    {
        case SC_T1_S_RESYNCH:
            if( dir==SC_T1_S_REQUEST )
                block[1]=0xC0;
            else
                block[1]=0xE0;
            block[2]=0x00;
            *pLen=3;
            if( (NTStatus = scT1AppendRc(pRdrExt, block, pLen)) )
            {
#ifdef PCMCIA_DEBUG
                printk("ERROR #5 :: Call scT1AppendRc() failed in scT1SBlock() \n");
#endif
                return( NTStatus );
            }
            break;
        case SC_T1_S_IFS:
            if( dir==SC_T1_S_REQUEST )
                block[1]=0xC1;
            else
                block[1]=0xE1;
            block[2]=0x01;
            block[3]=(BYTE)param;

            *pLen=4;
            if( (NTStatus = scT1AppendRc(pRdrExt, block, pLen)) )
            {
#ifdef PCMCIA_DEBUG
                printk("ERROR #5 :: Call scT1AppendRc() failed in scT1SBlock() \n");
#endif
                return( NTStatus );
            }
            break;
        case SC_T1_S_ABORT:
            if( dir==SC_T1_S_REQUEST )
                block[1]=0xC2;
            else
                block[1]=0xE2;

            block[2]=0x00;
            *pLen=3;

            if( (NTStatus = scT1AppendRc(pRdrExt, block, pLen)) )
                return( NTStatus );
            break;
        case SC_T1_S_WTX:
            if( dir==SC_T1_S_REQUEST )
                block[1]=0xC3;
            else
                block[1]=0xE3;

            block[2]=0x00;
            block[3]=(BYTE)param;
            *pLen=4;

            if( (NTStatus = scT1AppendRc(pRdrExt, block, pLen)) )
                return( NTStatus );
            break;
        default:
#ifdef PCMCIA_DEBUG
            printk("ERROR #6:: default in scT1SBlock() \n");
#endif
            return( SC_EXIT_BAD_PARAM );

    }

    return( SC_EXIT_OK );

}

/* Builds I-Block */
NTSTATUS scT1IBlock( PREADER_EXTENSION pRdrExt, BOOLEAN more, PUCHAR data,
                     PULONG datalen, PUCHAR block, PULONG blocklen )
{

    NTSTATUS    ret = STATUS_SUCCESS;

    block[0]=pRdrExt->T1.nad;
    block[1]=0x00;

    if( pRdrExt->T1.ns )
        block[1]|=0x40;

    if( more )
        block[1]|=0x20;

    if( *datalen > pRdrExt->T1.ifsc )
        return( SC_EXIT_BAD_PARAM );

    block[2]=(BYTE) *datalen;

    memcpy( block+3, data, *datalen );

    *blocklen = (*datalen)+3;

    if( (ret=scT1AppendRc(pRdrExt, block, blocklen)) )
        return( ret );

    return( SC_EXIT_OK );
}


/* Builds R-Block */
NTSTATUS scT1RBlock( PREADER_EXTENSION pRdrExt, int type, PUCHAR block, PULONG len )
{
    NTSTATUS NTStatus;

    block[0]=pRdrExt->T1.nad;
    block[2]=0x00;

    switch( type )
    {
        case SC_T1_R_OK:
#ifdef PCMCIA_DEBUG
            printk(" case SC_T1_R_OK: code 032 \n");
#endif
            if( pRdrExt->T1.nr )
                block[1]=0x90;
            else
                block[1]=0x80;
            break;
        case SC_T1_R_EDC_ERROR:
#ifdef PCMCIA_DEBUG
            printk("case  SC_T1_R_EDC_ERROR: code 033 \n");
#endif
            if( pRdrExt->T1.nr )
                block[1]=0x91;
            else
                block[1]=0x81;

            break;
        case SC_T1_R_OTHER_ERROR:
#ifdef PCMCIA_DEBUG
            printk("case SC_T1_R_OTHER_ERROR: code 034 \n");
#endif
            if( pRdrExt->T1.nr )
                block[1]=0x92;
            else
                block[1]=0x82;
            break;
        default:
#ifdef PCMCIA_DEBUG
            printk("case default, SC_EXIT_BAD_PARAM: code 035 \n");
#endif
            return( SC_EXIT_BAD_PARAM );
    }

    *len=3;

    if( (NTStatus = scT1AppendRc( pRdrExt, block, len )) )
    {
#ifdef PCMCIA_DEBUG
        printk("scT1AppendRc() failed: code 036 \n");
#endif
        return( NTStatus );
    }

    return( SC_EXIT_OK );
}

/* Returns N(R) or N(S) from R/I-Block. */
int scT1GetN( BYTE *block )
{
    /* R-Block */
    if( (block[1]&0xC0)==0x80 )
        return( (block[1]>>4)&0x01 );

    /* I-Block */
    if( (block[1]&0x80)==0x00 )
        return( (block[1]>>6)&0x01 );

    return( 0 );
}

/* Change IFSD. */
NTSTATUS scT1ChangeIFSD( PREADER_EXTENSION pRdrExt, BYTE ifsd )

{
    UCHAR	block[ SC_T1_MAX_SBLKLEN ];
    PUCHAR      rblock;

    ULONG	blocklen,rblocklen;

    BOOLEAN success=FALSE;
    int errors=0;
    NTSTATUS    ret = STATUS_SUCCESS;

    rblock = pRdrExt->SmartcardReply.Buffer;
    rblocklen = pRdrExt->SmartcardReply.BufferLength;

    ret=scT1SBlock(pRdrExt,SC_T1_S_IFS,SC_T1_S_REQUEST,ifsd,block,&blocklen);
    if(ret != SC_EXIT_OK)
    {
#ifdef PCMCIA_DEBUG
        printk("ERROR #1 :: Call scTiSBlock() failed in scT1ChangeIFSD() \n");
#endif
        return( ret );
    }

    while( !success )
    {
        ret = PscrWriteDirect(pRdrExt, block, blocklen, &rblocklen);

        if( ret != SC_EXIT_OK )
        {
#ifdef PCMCIA_DEBUG
            printk("ERROR #2 :: Call PscrWriteDirect() failed in scT1ChangeIFSD() \n");
#endif
            return( SC_EXIT_IO_ERROR );
        }

        mdelay(3000);

        rblocklen = MAX_T1_BLOCK_SIZE;
        ret = ReadFIFO( pRdrExt, rblock, &rblocklen );
        if( ret == STATUS_SUCCESS )
        {
            if( (rblocklen==blocklen) && (rblock[1]==0xE1) && 
                 scT1CheckRc( pRdrExt, rblock, &rblocklen ) )
            {
                pRdrExt->T1.ifsreq = TRUE;
                pRdrExt->T1.ifsd = rblock[3];
                success=TRUE;
            }
            else
            {
#ifdef PCMCIA_DEBUG
                printk("ERROR #3 :: rblocklen==blocklen) && (rblock[1]==0xE1) && \
                        scT1CheckRc( pRdrExt, rblock, &rblocklen ) failed in scT1ChangeIFSD() \n");
                printk(" values: \n rblocklen = %ld : blocklen = %ld \n rblock[1] = %x \n",
			rblocklen, blocklen, rblock[1]);
#endif
                errors++;

           }


        }
#ifdef PCMCIA_DEBUG
        else
        {
            printk("ERROR #4 :: Call ReadFIFO() failed in scT1ChangeIFSD() \n");  
        }
#endif
        if( errors>2 ) 
        {
            pRdrExt->T1.ifsreq = TRUE;
            success=TRUE;
        }
    }

    return( SC_EXIT_OK );
}

NTSTATUS CBT1Transmit( PREADER_EXTENSION pRdrExt )
{

    NTSTATUS ret = SC_EXIT_OK;
    PUCHAR pRequest, pReply;
    ULONG RequestLength, ReplyLength;

#ifdef PCMCIA_DEBUG
    int i;
#endif
    unsigned char szRev[150];


    ULONG sendptr=0;          /* Points to begining of unsent data. */

    ULONG sendlen, cpylen;

    UCHAR block[ SC_T1_MAX_BLKLEN ];
    ULONG blocklen;


    UCHAR block2[ SC_T1_MAX_BLKLEN ];

    ULONG block2len;



    PUCHAR rblock;
    ULONG rblocklen;

    UCHAR rsp[ SC_GENERAL_SHORT_DATA_SIZE+3 ];
    UCHAR rsplen=0;



    BOOLEAN more=TRUE;      /* More data to send. */
    BOOLEAN lastiicc=FALSE; /* It's ICCs turn to send I-Blocks. */

    int timeouts=0;

    int errcntr=0;
    int rerrcntr=0;
    int other_err_cntr = 0;
    int parity_err_cntr = 0;
    int ib_other_err_cntr = 0;
    int ierrcntr = 0;
    int time_out_cntr = 0;

    /* define a new variable for WTX */
    unsigned long ulWTX = 0;
    UCHAR         ucRqstWTX = 0;

    int quit_cntr = 0;

#undef FUNC_NAME
#define FUNC_NAME " CBT1Transmit : "
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "Function Start \n");
    printk(" ########## PREADER_EXTENSION pRdrExt = %x \n", sizeof(READER_EXTENSION));
#endif

    pRequest = pRdrExt->SmartcardRequest.Buffer;
    RequestLength = pRdrExt->SmartcardRequest.BufferLength;
    rblock = pReply = pRdrExt->SmartcardReply.Buffer;
    ReplyLength = pRdrExt->SmartcardReply.BufferLength = 0;

#ifdef PCMCIA_DEBUG
    printk(" @@@ Request Data from ifdtest: Length = %ld\n", RequestLength);
    for(i = 0; i < RequestLength; i ++)

    {
        printk("[%02x]", pRequest[i]);
    }
    printk("\n");
#endif
    blocklen = 0;
    pRdrExt->T1.ifsreq = FALSE;

    writew( (USHORT) (pRdrExt->T1.bwt / 0x10000), (pRdrExt->membase) + BWT_MSB );
    writew( (USHORT) (pRdrExt->T1.bwt % 0x10000), (pRdrExt->membase) + BWT_LSB );

    /* Change IFSD if not allready changed. */
    if( !pRdrExt->T1.ifsreq )
        if( (ret = scT1ChangeIFSD( pRdrExt, 0xFE )) != SC_EXIT_OK )
        {
#ifdef PCMCIA_DEBUG
            printk("Change IFSD if not already changed :: FAILED!\n");
#endif
            return( ret );
        }
#ifdef PCMCIA_DEBUG
    printk("Finish ChangeIFSD routine.\n");
#endif

    sendlen = min(RequestLength - sendptr, (ULONG)pRdrExt->T1.ifsc );
    if( sendlen == (RequestLength-sendptr) )
    {
        more=FALSE;
     }

    ret = scT1IBlock(pRdrExt, more, pRequest, &sendlen, block, &blocklen );
#ifdef PCMCIA_DEBUG
    printk("IBlock :");
    for(cpylen=0; cpylen<blocklen; cpylen++)
        printk(" [%02X]", block[cpylen]);

    printk("\n");
#endif

    if (ret != SC_EXIT_OK )
    {
#ifdef PCMCIA_DEBUG
        printk(" scT1IBlock() failed: code 001\n");
#endif
        return( ret );
    }
    sendptr+=sendlen;
#ifdef PCMCIA_DEBUG
    printk("PscrWriteDirect: 002 \n");
#endif
    ret = PscrWriteDirect(pRdrExt, block, blocklen, &rblocklen);

    if( ret != SC_EXIT_OK )
    {
#ifdef PCMCIA_DEBUG
      printk("PscrWriteDirect() failed: code 002\n");
#endif
      return( SC_EXIT_IO_ERROR );
    }
#ifdef PCMCIA_DEBUG
printk("Finish IFSD Write IBlock.\n");
#endif

    while( TRUE )
    {
        rblocklen = 270;
        ret = ReadFIFO( pRdrExt, rblock, &rblocklen );

        if( (ret != SC_EXIT_OK) || (rblocklen == 0) )
        {
            /* Timeout handling. */
            timeouts++;
            if( timeouts >= 2 )
            {
                if(time_out_cntr >= 2)
                {
#ifdef PCMCIA_DEBUG
                    printk("time_out_cntr >= 2: quit!");
#endif
                    return SC_EXIT_IO_ERROR;
                }

                /* send S-Block: resync request. */
#ifdef PCMCIA_DEBUG
                printk("/* S-Block: send resync Request */ \n");
#endif
                if ((ret=scT1SBlock( pRdrExt, SC_T1_S_RESYNCH, SC_T1_S_REQUEST,
                   0, block2, &block2len ) ) !=SC_EXIT_OK )
                {
#ifdef PCMCIA_DEBUG
                    printk("S-Block: scT1SBlock() failed(resync request) \n");
#endif
                    return ret;
                }
                ret = PscrWriteDirect(pRdrExt, block2, block2len, &rblocklen);
                if( ret != SC_EXIT_OK )
                {
#ifdef PCMCIA_DEBUG
                    printk("S-Block: PscrWriteDirect() failed(resync request) \n");
#endif
                    return SC_EXIT_IO_ERROR;
                }

                timeouts = 0;
                time_out_cntr ++;
            }
            else
            {

                /* indicate SC_T1_R_OTHER_ERROR */
#ifdef PCMCIA_DEBUG
                printk(" --------------> scT1RBlock::SC_T1_R_OTHER_ERROR in ReadFIFO error\n");
#endif
                ret = scT1RBlock(pRdrExt, SC_T1_R_OTHER_ERROR, block2, &block2len);
                if( ret != SC_EXIT_OK )
                {
#ifdef PCMCIA_DEBUG
                    printk("scT1RBlock() failed: code 004\n");
#endif
                    return( ret );
                }
#ifdef PCMCIA_DEBUG
                printk("PscrWriteDirect: 003 \n");
#endif
                ret = PscrWriteDirect(pRdrExt, block2, block2len, &rblocklen);
            }
            continue;
        }

        memset(szRev, 0, 150);
        memcpy(szRev, pRdrExt, sizeof(READER_EXTENSION));
#ifdef PCMCIA_DEBUG
        printk("READER_EXTENSION data = \n");
#endif

         /* Wrong length or RC error. */
        if( !scT1CheckRc( pRdrExt, rblock, &rblocklen ) )
        {
            if( pRdrExt->T1.rc == SC_T1_CHECKSUM_LRC )
            {
                if( (rblock[2]+3+1) != rblocklen )
                {
                    ret = scT1RBlock(pRdrExt, SC_T1_R_OTHER_ERROR, block2, &block2len);
                    if( ret != SC_EXIT_OK )
                    {
#ifdef PCMCIA_DEBUG
                        printk("scT1RBlock() failed: code 004\n");
#endif
                        return( ret );
                    }
#ifdef PCMCIA_DEBUG
                    printk("PscrWriteDirect: 003 \n");
#endif
                    ret = PscrWriteDirect(pRdrExt, block2, block2len, &rblocklen);

                    continue;
                }
            }
            else if( pRdrExt->T1.rc == SC_T1_CHECKSUM_CRC)
            {
                if( (rblock[2]+3+2) != rblocklen )
                {
                    ret = scT1RBlock(pRdrExt, SC_T1_R_OTHER_ERROR, block2, &block2len);
                    if( ret != SC_EXIT_OK )
                    {
#ifdef PCMCIA_DEBUG
                        printk("scT1RBlock() failed: code 004\n");
#endif
                        return( ret );
                    }
#ifdef PCMCIA_DEBUG
                    printk("PscrWriteDirect: 003 \n");
#endif
                    ret = PscrWriteDirect(pRdrExt, block2, block2len, &rblocklen);

                    continue;
                }
            }

            parity_err_cntr ++;
            if (parity_err_cntr >= 3)
            {
                if (errcntr >= 1)
                {
#ifdef PCMCIA_DEBUG
                    printk("CBT1Transmit() :: errcntr >= 1. Quit ! \n");
#endif
                    return SC_EXIT_IO_ERROR;
                }

                /* send S-Block: resync request. */
#ifdef PCMCIA_DEBUG
                printk("/* S-Block: send resync Request: parity error */ \n");
#endif
                if ((ret=scT1SBlock( pRdrExt, SC_T1_S_RESYNCH, SC_T1_S_REQUEST,
                     0, block2, &block2len ) ) !=SC_EXIT_OK )
                {
#ifdef PCMCIA_DEBUG
                    printk("S-Block: scT1SBlock() failed(resync request): parity error \n");
#endif
                    return ret;

                }
                ret = PscrWriteDirect(pRdrExt, block2, block2len, &rblocklen);
                if( ret != SC_EXIT_OK )
                {
#ifdef PCMCIA_DEBUG
                    printk("S-Block: PscrWriteDirect() failed(resync request): parity error \n");
#endif
                    return SC_EXIT_IO_ERROR;
                }

                /* reset parity_err_cntr, increase rerrcntr. */
                parity_err_cntr = 0;
                errcntr ++;
            }
            else
            {
#ifdef PCMCIA_DEBUG
                printk("pRdrExt->T1.rc = 0x%x \n", pRdrExt->T1.rc);
                printk("-----------> scT1RBlock::SC_T1_R_EDC_ERROR in Wrong length or RC error\n ");
#endif
                ret=scT1RBlock(pRdrExt, SC_T1_R_EDC_ERROR, block2, &block2len);
                if( ret != SC_EXIT_OK )
                {
#ifdef PCMCIA_DEBUG
                    printk("scT1RBlock() failed: code 007 \n");
#endif
                    return( ret );
                }
#ifdef PCMCIA_DEBUG
                printk("PscrWriteDirect: 004 \n");
#endif
                ret = PscrWriteDirect(pRdrExt, block2, block2len, &rblocklen);
                if( ret != SC_EXIT_OK )
                {
#ifdef PCMCIA_DEBUG
                    printk("PscrWriteDirect() failed: code 008 \n");
#endif
                    return( SC_EXIT_IO_ERROR );
                }
            }

            continue;
        }

        /* R-Block */
        if( (rblock[1]&0xC0)==0x80 )
        {
#ifdef PCMCIA_DEBUG
            printk("/* R-Block */ \n");
#endif
            if( lastiicc )
            {
                quit_cntr ++;
                if (quit_cntr >= 3)
                    return SC_EXIT_IO_ERROR;
                /* Card is sending I-Blocks, so send R-Block. */
#ifdef PCMCIA_DEBUG
                printk("----------> scT1RBlock::SC_T1_R_OK in R-Block\n");
#endif

                ret = scT1RBlock(pRdrExt, SC_T1_R_OK, block2, &block2len);

                if( ret != SC_EXIT_OK )
                {
#ifdef PCMCIA_DEBUG
                    printk("scT1RBlock() failed: code 010 \n");
#endif
                    return( ret );
                }

#ifdef PCMCIA_DEBUG
                printk("PscrWriteDirect: 005 \n");
#endif
                ret = PscrWriteDirect(pRdrExt, block2, block2len, &rblocklen);
                if( ret != SC_EXIT_OK )
                {
#ifdef PCMCIA_DEBUG
                    printk("PscrWriteDirect() failed: code 011 \n");
#endif
                    return( SC_EXIT_IO_ERROR );
                }
            } else {
                if( scT1GetN(rblock) == pRdrExt->T1.ns )
                {
resend_RBlock:
                    other_err_cntr ++;
                    if (other_err_cntr >= 3)
                    {
                        /* other_err_cntr >= 2, the third time. quit and return SC_EXIT_IO_ERROR */
                        if (rerrcntr >= 2)
                        {
#ifdef PCMCIA_DEBUG
                            printk("CBT1Transmit() :: rerrcntr >= 2. Quit ! \n");
#endif
                            return SC_EXIT_IO_ERROR;
                        }

                        /* send S-Block: resync request. */
#ifdef PCMCIA_DEBUG
                        printk("/* S-Block: send resync Request */ \n");
#endif
                        if ((ret=scT1SBlock( pRdrExt, SC_T1_S_RESYNCH, SC_T1_S_REQUEST,
                             0, block2, &block2len ) ) !=SC_EXIT_OK )
                        {
#ifdef PCMCIA_DEBUG
                            printk("S-Block: scT1SBlock() failed(resync request) \n");
#endif
                            return ret;
                        }
                        ret = PscrWriteDirect(pRdrExt, block2, block2len, &rblocklen);
                        if( ret != SC_EXIT_OK )
                        {
#ifdef PCMCIA_DEBUG
                            printk("S-Block: PscrWriteDirect() failed(resync request) \n");
#endif
                            return SC_EXIT_IO_ERROR;
                        }

                        /* reset other_err_cntr, increase rerrcntr. */
                        other_err_cntr = 0;
                        rerrcntr ++;
                    }
                    else
                    {
                        /* N(R) is old N(S), so resend I-Block. */
#ifdef PCMCIA_DEBUG
                        printk("PscrWriteDirect: 006 \n");
#endif
                        ret = PscrWriteDirect(pRdrExt, block, blocklen, &rblocklen);
                        if( ret != SC_EXIT_OK )
                        {
#ifdef PCMCIA_DEBUG
                            printk("PscrWriteDirect() failed: code 012 \n");
#endif
                            return( SC_EXIT_IO_ERROR );
                        }
                    }
                } else {
                    /* N(R) is next N(S), so make next I-Block and send it. */
                    /* Check if data available. */
                    if( more==FALSE )
                    {
#ifdef PCMCIA_DEBUG
                        printk("SC_EXIT_PROTOCOL_ERROR: code 013 \n");
#endif
                        quit_cntr ++;
                        if (quit_cntr > 3)
                        {
                            return( SC_EXIT_PROTOCOL_ERROR );
                        }
                        else
                        {
                            goto resend_RBlock;
                        }
                    }
                    /* Change N(S) to new value. */
                    pRdrExt->T1.ns^=1;

                    /* Make next I-Block. */
                    sendlen = min(RequestLength - sendptr, (ULONG)pRdrExt->T1.ifsc );

                    if( sendlen==(RequestLength-sendptr) )
                    {
#ifdef PCMCIA_DEBUG
                        printk(" sendlen==(RequestLength-sendptr): code 014 \n");
#endif
                        more=FALSE;

                    }
                    if( (ret = scT1IBlock( pRdrExt, more, pRequest+sendptr,
                        &sendlen, block, &blocklen )) != SC_EXIT_OK )
                    {
#ifdef PCMCIA_DEBUG
                        printk("scT1IBlock() failed: code 015 \n");
#endif
                        return( ret );
                    }
                    sendptr+=sendlen;

                    /* Send I-Block. */
#ifdef PCMCIA_DEBUG
                    printk("PscrWriteDirect: 007 \n");
#endif
                    ret = PscrWriteDirect(pRdrExt, block, blocklen, &rblocklen);
                    if( ret != SC_EXIT_OK )

                    {
#ifdef PCMCIA_DEBUG
                        printk("PscrWriteDirect() failed: code 016 \n");
#endif
                        return( SC_EXIT_IO_ERROR );
                    }
                }
            }
            continue;
        }
        /* Reset rerrcntr, because when it is here it had not received an
         * R-Block.
         */

        /* I-Block */
        if( (rblock[1]&0x80)==0x00 )
        {
#ifdef PCMCIA_DEBUG
            printk("/* I-Block */ \n");
#endif
            if( !lastiicc )
                pRdrExt->T1.ns^=1; /* Change N(S) to new value. */
            lastiicc=TRUE;
            if( scT1GetN(rblock) != pRdrExt->T1.nr )
            {
                ib_other_err_cntr ++;
                if (ib_other_err_cntr >= 3)
                {
                    /* other_err_cntr >= 2, the third time. quit and return SC_EXIT_IO_ERROR */
                    if (ierrcntr >= 2)
                    {
#ifdef PCMCIA_DEBUG
                        printk("CBT1Transmit() :: ierrcntr >= 2. Quit ! \n");
#endif
                        return SC_EXIT_IO_ERROR;
                    }

                    /* send S-Block: resync request. */
#ifdef PCMCIA_DEBUG
                    printk("/* S-Block: send resync Request */: i-block \n");
#endif
                    if ((ret=scT1SBlock( pRdrExt, SC_T1_S_RESYNCH, SC_T1_S_REQUEST,
                         0, block2, &block2len ) ) !=SC_EXIT_OK )
                    {
#ifdef PCMCIA_DEBUG
                        printk("S-Block: scT1SBlock() failed(resync request): i-block \n");
#endif
                        return ret;

                    }
                    ret = PscrWriteDirect(pRdrExt, block2, block2len, &rblocklen);
                    if( ret != SC_EXIT_OK )
                    {
#ifdef PCMCIA_DEBUG
                        printk("S-Block: PscrWriteDirect() failed(resync request): i-block \n");
#endif
                        return SC_EXIT_IO_ERROR;
                    }

                    /* reset other_err_cntr, increase rerrcntr. */
                    ib_other_err_cntr = 0;
                    ierrcntr ++;
                }

                else
                {
                    /* Card is sending wrong I-Block, so send R-Block. */
#ifdef PCMCIA_DEBUG
                    printk("-----------> scT1RBlock::SC_T1_R_OTHER_ERROR in I-Block \n");
#endif
                    ret=scT1RBlock(pRdrExt,SC_T1_R_OTHER_ERROR,block2,&block2len);
                    if( ret != SC_EXIT_OK )
                    {
                        printk("scT1RBlock() failed: code 017 \n");
                        return( ret );
                    }
#ifdef PCMCIA_DEBUG
                    printk("PscrWriteDirect: 008 \n");
#endif
                    ret = PscrWriteDirect(pRdrExt, block2, block2len, &rblocklen);
                    if( ret	!= SC_EXIT_OK )
                    {
#ifdef PCMCIA_DEBUG
                        printk("PscrWriteDirect() failed: code 018 \n");
#endif
                        return( SC_EXIT_IO_ERROR );
                    }
                }

                continue;
            }


            /* Copy data. */
            if( rblock[2]>(SC_GENERAL_SHORT_DATA_SIZE+2-rsplen) )
            {
#ifdef PCMCIA_DEBUG
                printk("rblock[2]>(SC_GENERAL_SHORT_DATA_SIZE+2-rsplen): code 019 \n");
#endif
                return( SC_EXIT_PROTOCOL_ERROR );
            }
            memcpy( rsp+rsplen, rblock+3, rblock[2] );
            rsplen+=rblock[2];
            if( (rblock[1]>>5) & 1 )
            {
                /* More data available. */
                /* Change N(R) to new value. */
                pRdrExt->T1.nr^=1;
                /* Send R-Block. */
#ifdef PCMCIA_DEBUG
                printk("------------> scT1RBlock::SC_T1_R_OK in copy data\n");
#endif
                ret=scT1RBlock( pRdrExt, SC_T1_R_OK, block2, &block2len );
                if( ret != SC_EXIT_OK )
                {
#ifdef PCMCIA_DEBUG
                    printk("scT1RBlock() failed: code 020 \n");
#endif
                    return( ret );
                }
#ifdef PCMCIA_DEBUG
                printk("PscrWriteDirect: 009 \n");
#endif
                ret = PscrWriteDirect(pRdrExt, block2, block2len, &rblocklen);
                if( ret != SC_EXIT_OK )
                {
#ifdef PCMCIA_DEBUG
                    printk("PscrWriteDirect() failed: code 021 \n");
#endif
                    return( SC_EXIT_IO_ERROR );
                }
            } else {
                /* Last block. */
                /* Change N(R) to new value. */
                pRdrExt->T1.nr^=1;

                if( rsplen<2 )
                {
#ifdef PCMCIA_DEBUG
                    printk("rsplen<2: code 022 \n");
#endif
                    return( SC_EXIT_BAD_SW );
                }

                if( (pRdrExt->T1.cse==SC_APDU_CASE_2_SHORT) ||
                    (pRdrExt->T1.cse==SC_APDU_CASE_4_SHORT) )


                {
                    /* Copy response and SW. */
                    cpylen = min( rsplen-2, SC_GENERAL_SHORT_DATA_SIZE );
                    memcpy( pReply, rsp, cpylen );

                    memcpy( pReply+cpylen, rsp+rsplen-2, 2 );
                    pRdrExt->SmartcardReply.BufferLength=cpylen+2;
                } else {
                    /* Copy only SW. */
                    memcpy( pReply, rsp+rsplen-2, 2 );
                    pRdrExt->SmartcardReply.BufferLength=2;
                }
#ifdef PCMCIA_DEBUG
                printk("SC_EXIT_OK: code 023(Jordan) return correctly. ");
#endif
                return( SC_EXIT_OK );
            }
            continue;
        }

        /* S-Block IFS Request */
        if( rblock[1]==0xC1 )
        {
#ifdef PCMCIA_DEBUG
            printk("/* S-Block IFS Request */ \n");
#endif
            if( (ret=scT1SBlock( pRdrExt, SC_T1_S_IFS, SC_T1_S_RESPONSE,
                rblock[3], block2, &block2len ) ) !=SC_EXIT_OK )
            {
#ifdef PCMCIA_DEBUG
                printk("scT1SBlock() failed: code 024 \n");
#endif
                return( ret );
            }

#ifdef PCMCIA_DEBUG
            printk("PscrWriteDirect: 012 \n");
#endif
            memset(block2, 0, 5);
            block2len = 4;
            block2[0] = rblock[0];
            block2[1] = 0xE1;
            block2[2] = rblock[2];
            block2[3] = rblock[3];
            scT1AppendRc(pRdrExt, block2, &block2len);

            ret = PscrWriteDirect(pRdrExt, block2, block2len, &rblocklen);

            if( ret != SC_EXIT_OK )
            {
#ifdef PCMCIA_DEBUG
                printk("PscrWriteDirect() failed: code 025 \n");
#endif
                return( SC_EXIT_IO_ERROR );
            }
            pRdrExt->T1.ifsc=rblock[3];
            mdelay(3000);
            continue;
        }

        /* S-Block ABORT Request */
        if( rblock[1]==0xC2 )
        {
#ifdef PCMCIA_DEBUG
            printk("/* S-Block ABORT Request */ \n");
#endif
            if( (ret=scT1SBlock(pRdrExt, SC_T1_S_ABORT, SC_T1_S_RESPONSE,
                0x00, block2, &block2len ) ) !=SC_EXIT_OK )

            {
#ifdef PCMCIA_DEBUG
                printk("scT1SBlock() failed: code 025 \n");
#endif
                return( ret );
            }

#ifdef PCMCIA_DEBUG
            printk("PscrWriteDirect: 011 \n");
#endif
            ret = PscrWriteDirect(pRdrExt, block2, block2len, &rblocklen);
            if( ret !=block2len )
            {
#ifdef PCMCIA_DEBUG
                printk("PscrWriteDirect() failed: code 026 \n");
#endif
                return( SC_EXIT_IO_ERROR );
            }
            /* Wait BWT. */
            mdelay(3000);
            ret = ReadFIFO( pRdrExt, rblock, &rblocklen );
            if( ret != SC_EXIT_OK )
            {
#ifdef PCMCIA_DEBUG
                printk("ReadFIFO() failed: code 027 \n");
#endif
                return( SC_EXIT_IO_ERROR );
            }
#ifdef PCMCIA_DEBUG
            printk("SC_EXIT_UNKNOWN_ERROR: code 028 \n");
#endif
            return( SC_EXIT_UNKNOWN_ERROR );
        }

        /* S-Block WTX Request */
        if( rblock[1]==0xC3 )
        {
#ifdef PCMCIA_DEBUG
            printk("/* S-Block WTX Request */ \n");
#endif
            ucRqstWTX = rblock[3];
            ulWTX = (pRdrExt->T1.bwt) * ucRqstWTX;
#ifdef PCMCIA_DEBUG
            printk("rblock[3] = %x\n", rblock[3]);
            printk("pRdrExt->T1.bwt = 0x%lx \n", pRdrExt->T1.bwt);
            printk("ulWTX = 0x%lx :: ucRqstWTX = %x \n", ulWTX, ucRqstWTX);
#endif
            writew( (USHORT) (ulWTX/0x10000), (pRdrExt->membase) + BWT_MSB );
            writew( (USHORT) (ulWTX%0x10000), (pRdrExt->membase) + BWT_LSB );
#ifdef PCMCIA_DEBUG
            printk("PscrWriteDirect: 012 \n");
#endif
            memset(block2, 0, 5);

            block2len = 5;
            block2[0] = rblock[0];
            block2[1] = rblock[1] + 0x20;
            block2[2] = rblock[2];
            block2[3] = rblock[3];
            block2[4] = rblock[4] + 0x20;
            ret = PscrWriteDirect(pRdrExt, block2, block2len, &rblocklen);
            if( ret != SC_EXIT_OK )
            {
#ifdef PCMCIA_DEBUG
                printk("PscrWriteDirect() failed: code 030 \n");
#endif
                return( SC_EXIT_IO_ERROR );
            }

            continue;
        }

        /* S-Block resync response */
        if (rblock[1] == 0xE0)
        {
            /* resend I-Block */
            pRdrExt->T1.ns = 0;
            pRdrExt->T1.nr = 0;
            sendptr = 0;
            sendlen = min(RequestLength - sendptr, (ULONG)pRdrExt->T1.ifsc);
            if (sendlen == (RequestLength - sendptr))
            {
                more =  FALSE;
            }
            ret = scT1IBlock(pRdrExt, more, pRequest, &sendlen, block, &blocklen);
            if (ret != SC_EXIT_OK)
            {
                return ret;
            }
            sendptr += sendlen;
            ret = PscrWriteDirect(pRdrExt, block, blocklen, &rblocklen);
            if (ret != SC_EXIT_OK)
            {
                return SC_EXIT_IO_ERROR;
            }

            continue;
        }

    }


#undef FUNC_NAME
#define FUNC_NAME " CBT1Transmit : "
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "Function Complete \n");
    printk("Function Complete \n");
#endif

    /* Ooops! Should never be here. */
#ifdef PCMCIA_DEBUG
    printk("SC_EXIT_OK: code 031 \n");
#endif    
    return( SC_EXIT_OK );
}

NTSTATUS VerifyByte( BYTE control, BYTE address, BYTE data )
{
    BYTE ackData = 1;
    NTSTATUS NTStatus = STATUS_UNSUCCESSFUL;
    UCHAR i;

    CommandEntry( control, address, data );
    mdelay(1);

    for( i = 0; i < 2; i++)
    {
        mdelay(1);
        high(CLK);
        mdelay(1);
        ackData = readIO();
        low(CLK);
        if( !ackData && (i==1) )
            NTStatus = STATUS_SUCCESS;
    }
 
    return NTStatus;
}

NTSTATUS VerifyCounter( BYTE control, BYTE address, BYTE data )
{
    BYTE ackData = 1;
    NTSTATUS NTStatus = STATUS_UNSUCCESSFUL;
    UCHAR i;

    CommandEntry( control, address, data );
    mdelay(1);
    for( i = 0; i < 103; i++)
    {
        mdelay(1);
        high(CLK);
        ackData = readIO();

        mdelay(1);
        low(CLK);
        if( ackData && (i==100) )
            NTStatus = STATUS_SUCCESS;
    }
 
    return NTStatus;
}

static BYTE
ErrorCounter_NumTrials (BYTE b)
{
  int count = 0;
  int i;

  for (i = 0; i < 8; i++)
  {
      count += ((b & 0x01) == 0x01) ? 1 : 0;
      b >>= 1;
  }

  return count;
}



NTSTATUS VerifyData( PREADER_EXTENSION pRdrExt )
{
    BYTE control;
    BYTE address;
    BYTE data;
    BYTE trials;
    PUCHAR pRequest = pRdrExt->SmartcardRequest.Buffer;

    PUCHAR pReply = pRdrExt->SmartcardReply.Buffer;

    NTSTATUS NTStatus = STATUS_UNSUCCESSFUL;


    data = readByte( 1021 );
    trials = ErrorCounter_NumTrials( data );
    if( trials )
    {
        data = (trials == 8) ? 0xFE :
        (trials == 7) ? 0xFC :
        (trials == 6) ? 0xF8 :
        (trials == 5) ? 0xF0 :
        (trials == 4) ? 0xE0 :
        (trials == 3) ? 0xC0 :
        (trials == 2) ? 0x80 : 0x00;
        control = 0xF2;
        address = 0xFD;

        NTStatus = VerifyCounter( control, address, data );
        mdelay(1);

        if( NTStatus == STATUS_SUCCESS )
        {
            /* enter first PSC-code byte */
            control = 0xCD;
            address = 0xFE;
            data = pRequest[5];

            NTStatus = VerifyByte( control, address, data );

            mdelay(1);

           if( NTStatus == STATUS_SUCCESS )
           {
               /* enter second PSC-code byte */
               control = 0xCD;
               address = 0xFF;
               data = pRequest[6];

               NTStatus = VerifyByte( control, address, data );

               mdelay(1);
           }

        }

        control = 0xF3;
        address = 0xFD;
        data = 0xFF;

        NTStatus = VerifyCounter( control, address, data );
    }


    pRdrExt->SmartcardReply.BufferLength = 2;
    if( NTStatus == STATUS_SUCCESS )
    {
        pReply[0] = 0x90;
        pReply[1] = 0x00;
    }
    else
    {
        pReply[0] = 0x63;
        trials = (trials == 0) ? 0 : trials-1;
        pReply[1] = 0xC0 | trials;
    }

    return( SC_EXIT_OK );
}

NTSTATUS ChangeVerifyData( PREADER_EXTENSION pRdrExt )
{
    BYTE control;
    BYTE address;
    BYTE data;
    PUCHAR pRequest = pRdrExt->SmartcardRequest.Buffer;
    PUCHAR pReply = pRdrExt->SmartcardReply.Buffer;
    NTSTATUS NTStatus = STATUS_UNSUCCESSFUL;

    /* update first PSC-code byte */
    control = 0xF3;
    address = 0xFE;


    data = pRequest[8];

    NTStatus = VerifyCounter( control, address, data );

    mdelay(1);

    if( NTStatus == STATUS_SUCCESS )
    {
        /* update second PSC-code byte */
        control = 0xF3;
        address = 0xFF;

        data = pRequest[9];
        NTStatus = VerifyCounter( control, address, data );

        mdelay(1);
    }

    pRdrExt->SmartcardReply.BufferLength = 2;
    if( NTStatus == STATUS_SUCCESS )
    {
        pReply[0] = 0x90;
        pReply[1] = 0x00;
    }
    else
    {
        pReply[0] = 0x63;
        pReply[1] = 0xC0;
    }

    return( SC_EXIT_OK );

}

NTSTATUS UpdateBinary( PREADER_EXTENSION pRdrExt )
{
    PUCHAR pRequest = pRdrExt->SmartcardRequest.Buffer;
    PUCHAR pReply = pRdrExt->SmartcardReply.Buffer;
    NTSTATUS NTStatus = STATUS_UNSUCCESSFUL;
    UCHAR i;

    WORD address;

    pRdrExt->SmartcardReply.BufferLength = 2;
    address = pRequest[2] << 8;

    address |= pRequest[3];
    for( i = 0; i < pRequest[4] ; i++ )
    {
        NTStatus = writeByte( address + i, pRequest[5+i] );
        if( NTStatus != STATUS_SUCCESS )
            break;
    }

    if( NTStatus == STATUS_SUCCESS )
    {
        pReply[0] = 0x90;
        pReply[1] = 0x00;
    }
    else
    {
        pReply[0] = 0x62;
        pReply[1] = 0x00;
    }

    return( SC_EXIT_OK );
}


NTSTATUS ReadBinary( PREADER_EXTENSION pRdrExt )
{
    PUCHAR pRequest = pRdrExt->SmartcardRequest.Buffer;
    PUCHAR pReply = pRdrExt->SmartcardReply.Buffer;
    ULONG ReplyLength;
    NTSTATUS NTStatus = STATUS_UNSUCCESSFUL;
    UCHAR i;
    WORD address;

    ReplyLength = pRequest[4] + 2;
    address = pRequest[2] << 8;
    address |= pRequest[3];

    for( i = 0; i < pRequest[4] ; i++ )
        pReply[i] = readByte( address + i );

    pRdrExt->SmartcardReply.BufferLength = ReplyLength;

    NTStatus = STATUS_SUCCESS;
    if( NTStatus == STATUS_SUCCESS )
    {
        pReply[ReplyLength-2] = 0x90;
        pReply[ReplyLength-1] = 0x00;
    }
    else
    {
        pReply[ReplyLength-2] = 0x62;
        pReply[ReplyLength-1] = 0x81;
    }

    return( SC_EXIT_OK );
}


NTSTATUS SelectFile( PREADER_EXTENSION pRdrExt )
{
    PUCHAR pRequest = pRdrExt->SmartcardRequest.Buffer;
    PUCHAR pReply = pRdrExt->SmartcardReply.Buffer;
    pRdrExt->SmartcardReply.BufferLength = 2;

    if( (pRequest[5] == 0x3F) && (pRequest[6] == 0x00) )
    {
        pReply[0] = 0x90;
        pReply[1] = 0x00;
    }
    else
    {
        pReply[0] = 0x6A;
        pReply[1] = 0x82;
    }

    return( SC_EXIT_OK );
}



NTSTATUS BadCommand( PREADER_EXTENSION pRdrExt )
{
    PUCHAR  pReply = pRdrExt->SmartcardReply.Buffer;
    pRdrExt->SmartcardReply.BufferLength = 2;
    pReply[0] = 0x6E;
    pReply[1] = 0x00;

    return( SC_EXIT_OK );
}

/*
CBRawTransmit:
    finishes the callback RDF_TRANSMIT for the RAW protocol

Arguments:
    pRdrExt  context of call
Return Value:

    STATUS_SUCCESS
    STATUS_NO_MEDIA
    STATUS_TIMEOUT
    STATUS_INVALID_DEVICE_REQUEST
*/
NTSTATUS CBRawTransmit( PREADER_EXTENSION pRdrExt )
{
    NTSTATUS NTStatus = STATUS_SUCCESS;
    PUCHAR pRequest, pReply;

    ULONG RequestLength;

#undef FUNC_NAME

#define FUNC_NAME " CBRawTransmit : "
#ifdef PCMCIA_DEBUG
printk(KERN_INFO MODULE_NAME FUNC_NAME "Function Start \n");
#endif

    pRequest = pRdrExt->SmartcardRequest.Buffer;
    pReply = pRdrExt->SmartcardReply.Buffer;

    /* Transmit pRdrExt->SmartcardRequest.Buffer to smart card */
    RequestLength = pRdrExt->SmartcardRequest.BufferLength;
    pRdrExt->SmartcardReply.BufferLength = 0;

    /* Interindustry Commands */
    switch ( pRequest[1] )
    {
        case 0xA4:
            NTStatus = SelectFile( pRdrExt );
            printk(KERN_INFO MODULE_NAME FUNC_NAME "SelectFile \n");
            break;
        case 0xB0:
            NTStatus = ReadBinary( pRdrExt );
            printk(KERN_INFO MODULE_NAME FUNC_NAME "ReadBinary \n");
            break;
        case 0xD6:
            NTStatus = UpdateBinary( pRdrExt );
            printk(KERN_INFO MODULE_NAME FUNC_NAME "UpdateBinary \n");
            break;
        case 0x20:
            NTStatus = VerifyData( pRdrExt );
            printk(KERN_INFO MODULE_NAME FUNC_NAME "VerifyData \n");
            break;
        case 0x24:
            NTStatus = ChangeVerifyData( pRdrExt );
            printk(KERN_INFO MODULE_NAME FUNC_NAME "ChangeVerifyData \n");
            break;
        default:
            NTStatus = BadCommand( pRdrExt );
    }


#undef FUNC_NAME
#define FUNC_NAME " CBRawTransmit : "
#ifdef PCMCIA_DEBUG
    printk(KERN_INFO MODULE_NAME FUNC_NAME "Function Complete \n");
#endif

  return NTStatus;
}

/*++
CBTransmit:
    callback handler for SMCLIB RDF_TRANSMIT
Arguments:
    pRdrExt  context of call
Return Value:
    STATUS_SUCCESS

    STATUS_NO_MEDIA
    STATUS_TIMEOUT

    STATUS_INVALID_DEVICE_REQUEST
--*/
NTSTATUS CBTransmit( PREADER_EXTENSION pRdrExt )
{
    NTSTATUS  NTStatus = STATUS_SUCCESS;
    BYTE protocol = pRdrExt->m_SCard.Protocol & 0x03;
    USHORT apdulen;


    if((protocol==SCARD_PROTOCOL_T0) || (protocol==SCARD_PROTOCOL_T1))
    {
        apdulen = pRdrExt->SmartcardRequest.BufferLength;
        if(apdulen < 5)
        {
            pRdrExt->T1.cse = SC_APDU_CASE_1;
        }
        else if(apdulen == 5)
        {
            pRdrExt->T1.cse = SC_APDU_CASE_2_SHORT;
        }
        else
        {
            if((apdulen-5) == pRdrExt->SmartcardRequest.Buffer[4])
                pRdrExt->T1.cse = SC_APDU_CASE_3_SHORT;
            else
                pRdrExt->T1.cse = SC_APDU_CASE_4_SHORT;
        }
    }

    switch( protocol )
    {
        case SCARD_PROTOCOL_T0:
            NTStatus = CBT0Transmit( pRdrExt );
            break;
        case SCARD_PROTOCOL_T1:
            NTStatus = CBT1Transmit( pRdrExt );
            break;
        case SCARD_PROTOCOL_RAW:
            NTStatus = CBRawTransmit( pRdrExt );
            break;
        default:
            NTStatus = STATUS_INVALID_DEVICE_REQUEST;
            break;
    }

    return( NTStatus );
}


module_init(init_ozscrlx);
module_exit(exit_ozscrlx);
MODULE_LICENSE("GPL");



