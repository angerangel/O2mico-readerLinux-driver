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

 *    Module:       debug.h
 *    Author:       O2Micro Inc.,
 */

#ifndef _O2MICRO_DEBUG_H_
#define _O2MICRO_DEBUG_H_

#ifdef PCSC_DEBUG
#define DebugLogA(fmt) debug_msg("%s:%d:%s " fmt, __FILE__, __LINE__, __FUNCTION__)
#define DebugLogB(fmt, data) debug_msg("%s:%d:%s " fmt, __FILE__, __LINE__, __FUNCTION__, data)
#define DebugLogC(fmt, data1, data2) debug_msg("%s:%d:%s " fmt, __FILE__, __LINE__, __FUNCTION__, data1, data2)
#define DebugLogD(fmt, data1, data2, data3) debug_msg("%s:%d:%s " fmt, __FILE__, __LINE__, __FUNCTION__, data1, data2, data3)
#else
#define DebugLogA(fmt) 
#define DebugLogB(fmt, data) 
#define DebugLogC(fmt, data1, data2) 
#define DebugLogD(fmt, data1, data2, data3) 
#endif /*PCSC_DEBUG*/

#endif /* _O2MICRO_DEBUG_H_ */
