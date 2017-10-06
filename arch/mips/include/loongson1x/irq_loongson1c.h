/****************************************************************************
 * arch/mips/include/loongson1x/irq_loongson1c.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/* This file should never be included directed but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_MIPS_INCLUDE_LOONGSON_IRQ_LS1C_H
#define __ARCH_MIPS_INCLUDE_LOONGSON_IRQ_LS1C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Interrupt vector numbers.  These should be used to attach to interrupts
 * and to change interrupt priorities.
 */
#define LS1X_IRQ_ACPT			  0
#define LS1X_IRQ_HPET			  1

#define LS1X_IRQ_UART0			  3
#define LS1X_IRQ_UART1			  4
#define LS1X_IRQ_UART2			  5
#define LS1X_IRQ_CAN0			  6
#define LS1X_IRQ_CAN1			  7
#define LS1X_IRQ_SPI0			  8
#define LS1X_IRQ_SPI1			  9
#define LS1X_IRQ_AC97			  10
#define LS1X_IRQ_MS				  11
#define LS1X_IRQ_KB				  12
#define LS1X_IRQ_DMA0			  13
#define LS1X_IRQ_DMA1			  14
#define LS1X_IRQ_DMA2			  15
#define LS1X_IRQ_NAND			  16
#define LS1X_IRQ_PWM0			  17
#define LS1X_IRQ_PWM1			  18
#define LS1X_IRQ_PWM2			  19
#define LS1X_IRQ_PWM3			  20
#define LS1X_IRQ_RTC_INT0         21
#define LS1X_IRQ_RTC_INT1         22
#define LS1X_IRQ_RTC_INT2         23
#define LS1X_IRQ_UART3            29
#define LS1X_IRQ_ADC              30
#define LS1X_IRQ_SDIO             31

#define LS1X_IRQ_EHCI             32
#define LS1X_IRQ_OHCI             33
#define LS1X_IRQ_OTG              34
#define LS1X_IRQ_MAC              35
#define LS1X_IRQ_CAM              36
#define LS1X_IRQ_UART4            37
#define LS1X_IRQ_UART5            38
#define LS1X_IRQ_UART6            39
#define LS1X_IRQ_UART7            40
#define LS1X_IRQ_UART8            41

#define LS1X_IRQ_UART9            45
#define LS1X_IRQ_UART10           46
#define LS1X_IRQ_UART11           47

#define LS1X_IRQ_I2C2             49 
#define LS1X_IRQ_I2C1             50
#define LS1X_IRQ_I2C0             51


#define LS1X_GPIO_IRQ             64
#define LS1X_IRQ_FIRST_GPIO       64
#define LS1X_IRQ_GPIO_COUNT       96
#define LS1X_IRQ_LAST_GPIO        (LS1X_IRQ_FIRST_GPIO + LS1X_IRQ_GPIO_COUNT)

#define NR_IRQS                   (32 * 5)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
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
#endif /* __ARCH_MIPS_INCLUDE_LOONGSON_IRQ_LS1C_H */

