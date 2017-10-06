/****************************************************************************
 * arch/mips/include/loongson1x/chip.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2017 unixjet. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           unixjet  <unixjet@hotmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_MIPS_INCLUDE_LOONGSON_LOONGSON1X_CHIP_H
#define __ARCH_MIPS_INCLUDE_LOONGSON_LOONGSON1X_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


//#if defined(CONFIG_ARCH_CHIP_PIC32MZ2048ECH)
#if defined(CONFIG_ARCH_CHIP_LOONGSON1C)
#  define CHIP_LOONGSON1C    1
//#  define CHIP_BOOTFLASH_KB 160  /* 160Kb boot FLASH */
//#  define CHIP_PROGFLASH_KB 2048 /* 2048Kb program FLASH */
#  define CHIP_DATAMEM_KB   32*1024  /* 32MB data memory */
#  define CHIP_NTIMERS      2    /* 2 timers */
#  define CHIP_NIC          0    /* 0 input capture */
#  define CHIP_NOC          0    /* 5 output compare */
#  define CHIP_NUARTS       4    /* 4 UARTS */
//#  define CHIP_UARTFIFOD    8    /* 8 level deep UART FIFOs */
#  define CHIP_NSPI         2    /* 6 SPI/I2S interfaces */
#  define CHIP_NCAN         2    /* 2 CAN interfaces */
#  define CHIP_NCRTYPO      0    /* No crtypo support */
#  define CHIP_RNG          0    /* 1 Random number generator */
#  define CHIP_NDMACH       8    /* 8 programmable DMA channels */
#  define CHIP_NUSBDMACHAN  16   /* 16 dedicated DMA channels */
#  define CHIP_NADC10       48   /* 48 ADC channels */
#  define CHIP_NCM          2    /* 2 Analog comparators */
#  define CHIP_USBHSOTG     1    /* 1 USB 2.0 HSOTG */
#  define CHIP_NI2C         2    /* 5 I2C interfaces */
#  define CHIP_NPMP         0    /* Have parallel master port */
#  define CHIP_NEBI         1    /* Have eternal bus interface */
#  define CHIP_NSQI         0    /* 1 Serial quad interface */
#  define CHIP_NRTCC        1    /* Has RTCC */
#  define CHIP_NETHERNET    1    /* 1 Ethernet MAC */
#  define CHIP_NPORTS       5    /* 10 ports (A-E) */
#  define CHIP_NJTAG        1    /* Has JTAG */
#  define CHIP_NTRACE       1    /* Has trace capability */


#else
#  error "Unrecognized Loongson device
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_INCLUDE_LOONGSON1X_CHIP_H */
