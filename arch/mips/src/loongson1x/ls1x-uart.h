/************************************************************************************
 * arch/mips/src/loongsonlx/ls1x-uart.h
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
 ************************************************************************************/

#ifndef __ARCH_MIPS_SRC_LOONGSON1X_UART_H
#define __ARCH_MIPS_SRC_LOONGSON1X_UART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "ls1x-memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Register Offsets *****************************************************************/

#define LS1X_UART_TXFIFO_OFFSET     0x0000
#define LS1X_UART_IER_OFFSET        0x0001
#define LS1X_UART_IIR_OFFSET        0x0002
#define LS1X_UART_FCR_OFFSET        0x0002
#define LS1X_UART_LCR_OFFSET        0x0003
#define LS1X_UART_MCR_OFFSET        0x0004
#define LS1X_UART_LSR_OFFSET        0x0005
#define LS1X_UART_MSR_OFFSET        0x0006

#define LS1X_UART_FFL_LSB_OFFSET    0x0000 /* UARTx Fractional Frequency Lock Latch Low */
#define LS1X_UART_FFL_MSB_OFFSET    0x0001 /* UARTx Fractional Frequency Lock Latch High */

#define LS1X_UART_CR_OFFSET         0x0008
#define LS1X_UART_MR_OFFSET         0x0009
#define LS1X_UART_FIDI_OFFSET       0x000A

/* Register Addresses ****************************************************************/

#if CHIP_NUARTS > 0
#  define LS1X_UART0_TXFIFO         (LS1X_UART0_K1BASE+LS1X_UART_TXFIFO_OFFSET)
#  define LS1X_UART0_IER            (LS1X_UART0_K1BASE+LS1X_UART_IER_OFFSET)
#  define LS1X_UART0_IIR            (LS1X_UART0_K1BASE+LS1X_UART_IIR_OFFSET)
#  define LS1X_UART0_FCR            (LS1X_UART0_K1BASE+LS1X_UART_FCR_OFFSET)
#  define LS1X_UART0_LCR            (LS1X_UART0_K1BASE+LS1X_UART_LCR_OFFSET)
#  define LS1X_UART0_MCR            (LS1X_UART0_K1BASE+LS1X_UART_MCR_OFFSET)
#  define LS1X_UART0_LSR            (LS1X_UART0_K1BASE+LS1X_UART_LSR_OFFSET)
#  define LS1X_UART0_MSR            (LS1X_UART0_K1BASE+LS1X_UART_MSR_OFFSET)
#  define LS1X_UART0_FFL_LSB        (LS1X_UART0_K1BASE+LS1X_UART_FFL_LSB_OFFSET)
#  define LS1X_UART0_FFL_MSB        (LS1X_UART0_K1BASE+LS1X_UART_FFL_MSB_OFFSET)
#  define LS1X_UART0_CR             (LS1X_UART0_K1BASE+LS1X_UART_CR_OFFSET)
#  define LS1X_UART0_MR             (LS1X_UART0_K1BASE+LS1X_UART_MR_OFFSET)
#  define LS1X_UART0_FIDI           (LS1X_UART0_K1BASE+LS1X_UART_FIDI_OFFSET)
#endif

#if CHIP_NUARTS > 1
#  define LS1X_UART1_TXFIFO         (LS1X_UART1_K1BASE+LS1X_UART_TXFIFO_OFFSET)
#  define LS1X_UART1_IER            (LS1X_UART1_K1BASE+LS1X_UART_IER_OFFSET)
#  define LS1X_UART1_IIR            (LS1X_UART1_K1BASE+LS1X_UART_IIR_OFFSET)
#  define LS1X_UART1_FCR            (LS1X_UART1_K1BASE+LS1X_UART_FCR_OFFSET)
#  define LS1X_UART1_LCR            (LS1X_UART1_K1BASE+LS1X_UART_LCR_OFFSET)
#  define LS1X_UART1_MCR            (LS1X_UART1_K1BASE+LS1X_UART_MCR_OFFSET)
#  define LS1X_UART1_LSR            (LS1X_UART1_K1BASE+LS1X_UART_LSR_OFFSET)
#  define LS1X_UART1_MSR            (LS1X_UART1_K1BASE+LS1X_UART_MSR_OFFSET)
#  define LS1X_UART1_FFL_LSB        (LS1X_UART1_K1BASE+LS1X_UART_FFL_LSB_OFFSET)
#  define LS1X_UART1_FFL_MSB        (LS1X_UART1_K1BASE+LS1X_UART_FFL_MSB_OFFSET)
#  define LS1X_UART1_CR             (LS1X_UART1_K1BASE+LS1X_UART_CR_OFFSET)
#  define LS1X_UART1_MR             (LS1X_UART1_K1BASE+LS1X_UART_MR_OFFSET)
#  define LS1X_UART1_FIDI           (LS1X_UART1_K1BASE+LS1X_UART_FIDI_OFFSET)
#endif

#if CHIP_NUARTS > 2
#  define LS1X_UART2_TXFIFO         (LS1X_UART2_K1BASE+LS1X_UART_TXFIFO_OFFSET)
#  define LS1X_UART2_IER            (LS1X_UART2_K1BASE+LS1X_UART_IER_OFFSET)
#  define LS1X_UART2_IIR            (LS1X_UART2_K1BASE+LS1X_UART_IIR_OFFSET)
#  define LS1X_UART2_FCR            (LS1X_UART2_K1BASE+LS1X_UART_FCR_OFFSET)
#  define LS1X_UART2_LCR            (LS1X_UART2_K1BASE+LS1X_UART_LCR_OFFSET)
#  define LS1X_UART2_MCR            (LS1X_UART2_K1BASE+LS1X_UART_MCR_OFFSET)
#  define LS1X_UART2_LSR            (LS1X_UART2_K1BASE+LS1X_UART_LSR_OFFSET)
#  define LS1X_UART2_MSR            (LS1X_UART2_K1BASE+LS1X_UART_MSR_OFFSET)
#  define LS1X_UART2_FFL_LSB        (LS1X_UART2_K1BASE+LS1X_UART_FFL_LSB_OFFSET)
#  define LS1X_UART2_FFL_MSB        (LS1X_UART2_K1BASE+LS1X_UART_FFL_MSB_OFFSET)
#  define LS1X_UART2_CR             (LS1X_UART2_K1BASE+LS1X_UART_CR_OFFSET)
#  define LS1X_UART2_MR             (LS1X_UART2_K1BASE+LS1X_UART_MR_OFFSET)
#  define LS1X_UART2_FIDI           (LS1X_UART2_K1BASE+LS1X_UART_FIDI_OFFSET)
#endif

#if CHIP_NUARTS > 3
#endif

#if CHIP_NUARTS > 4
#endif

#if CHIP_NUARTS > 5
#endif

/* Register Bit-Field Definitions ****************************************************/

/* UARTx  IER register */

#define UART_IER_IME_SHIFT         3 /* Modem status interrupt bit field */
#define UART_IER_IME_MASK          (1 << UART_IER_IME_SHIFT)
#define UART_IER_IME_ENABLE        (1 << UART_IER_IME_SHIFT)

#define UART_IER_ILE_SHIFT         2 /* Receive line status interrupt bit field */
#define UART_IER_ILE_MASK          (1 << UART_IER_ILE_SHIFT)
#define UART_IER_ILE_ENABLE        (1 << UART_IER_ILE_SHIFT)

#define UART_IER_ITXE_SHIFT        1 /* TX empty interrupt bit filed */
#define UART_IER_ITXE_MASK         (1 << UART_IER_ITXE_SHIFT)
#define UART_IER_ITXE_ENABLE       (1 << UART_IER_ITXE_SHIFT)

#define UART_IER_IRXE_SHIFT        0 /* Rx valid interrupt bit field */
#define UART_IER_IRXE_MASK         (1 << UART_IER_IRXE_SHIFT)
#define UART_IER_IRXE_ENABLE       (1 << UART_IER_IRXE_SHIFT)


/* UARTx IIR register */

#define UART_IIR_II_SHIFT         1 /*Interrupt source indicating bit field */
#define UART_IIR_II_MASK          (7 << UART_IIR_II_SHIFT)
#define UART_IIR_II_RLS           (3 << UART_IIR_II_SHIFT) /* Receive line status, Interrupt source:Odd-even, overflow or framing error, or break interrupt */
#define UART_IIR_II_RVN           (2 << UART_IIR_II_SHIFT) /* Receive valid number, Interrupt source: the number of elements in FIFO reaches the trigger level */
#define UART_IIR_II_RTO           (6 << UART_IIR_II_SHIFT) /* Receive Time out */
#define UART_IIR_II_ETS           (1 << UART_IIR_II_SHIFT) /* Empty transmission save register */
#define UART_IIR_II_MS            (0 << UART_IIR_II_SHIFT) /* Modem status */

#define UART_IIR_INTP_SHIFT       0
#define UART_IIR_INTP_MASK        (1 << UART_IIR_INTP_SHIFT)
#define UART_IIR_INTP             (1 << UART_IIR_INTP_SHIFT)


/* UARTx FIFO control register */

#define UART_FCR_TL_SHIFT         6 /* Trigger Level */
#define UART_FCR_TL_MASK          (3 << UART_FCR_TL_SHIFT)
#  define UART_FCR_TL_1B          (0 << UART_FCR_TL_SHIFT)
#  define UART_FCR_TL_4B          (1 << UART_FCR_TL_SHIFT)
#  define UART_FCR_TL_8B          (2 << UART_FCR_TL_SHIFT)
#  define UART_FCR_TL_14B         (3 << UART_FCR_TL_SHIFT)

/* Txset, set '1' to reset TX FIFO and clear its content */

#define UART_FCR_TXSET_SHIFT      2
#define UART_FCR_TXSET_MASK       (1 << UART_FCR_TXSET_SHIFT)
#define UART_FCR_TXSET            (1 << UART_FCR_TXSET_SHIFT)

/* Rxset, set '1' to reset RX FIFO and clear its content
#define UART_FCR_RXSET_SHIFT      1
#define UART_FCR_RXSET_MASK       (1 << UART_FCR_RXSET_SHIFT)
#define UART_FCR_RXSET            (1 << UART_FCR_RXSET_SHIFT)

/*  UARTx LCR regisger */

#define UART_LCR_DLAB_SHIFT       7 /* Fractional frequency latch access bit field */
#define UART_LCR_DLAB_MASK        (1 << UART_LCR_DLAB_SHIFT)
#  define UART_LCR_DLAB_FFL       (1 << UART_LCR_DLAB_SHIFT)
#  define UART_LCR_DLAB_NORMAL    (0 << UART_LCR_DLAB_SHIFT)

#define UART_LCR_BCB_SHIFT        6 /* Interrupt control bit field */
#define UART_LCR_BCB_MASK         (1 << UART_LCR_BCB_SHIFT)
#  define UART_LCR_BCB_INTERRUPT  (1 << UART_LCR_BCB_SHIFT)
#  define UART_LCR_BCB_NORMAL     (0 << UART_LCR_BCB_SHIFT)

#define UART_LCR_SPB_SHIFT        5 /* Specified parity check bit field */
#define UART_LCR_SPB_MASK         (1 << UART_LCR_SPB_SHIFT)
#  define UART_LCR_SPB_NONE       (0 << UART_LCR_SPB_SHIFT)
#  define UART_LCR_SPB_ENABLE     (1 << UART_LCR_SPB_SHIFT)

#define UART_LCR_EPS_SHIFT        4 /* Parity check bit selections */
#define UART_LCR_EPS_MASK         (1 << UART_LCR_EPS_SHIFT)
#  define UART_LCR_EPS_ODD        (0 << UART_LCR_EPS_SHIFT) /* odd number of 1 in each char */
#  define UART_LCR_EPS_EVEN       (1 << UART_LCR_EPS_SHIFT) /* even number of 1 in each char */

#define UART_LCR_PE_SHIFT         3
#define UART_LCR_PE_MASK          (1 << UART_LCR_PE_SHIFT)
#  define UART_LCR_PE_NONE        (0 << UART_LCR_PE_SHIFT)
#  define UART_LCR_PE_ENABLE      (1 << UART_LCR_PE_SHIFT)

#define UART_LCR_SB_SHIFT         2 /* Stop bit field */
#define UART_LCR_SB_MASK          (1 << UART_LCR_SB_SHIFT)
#  define UART_LCR_SB_ONE         (0 << UART_LCR_SB_SHIFT)
#  define UART_LCR_SB_ONE_HALF    (1 << UART_LCR_SB_SHIFT)
#  define UART_LCR_SB_TWO         (1 << UART_LCR_SB_SHIFT)

#define UART_LCR_BEC_SHIFT        1 /* number of digits in each char */
#define UART_LCR_BEC_MASK         (3 << UART_LCR_BEC_SHIFT)
#  define UART_LCR_BEC_5          (0 << UART_LCR_BEC_SHIFT)
#  define UART_LCR_BEC_6          (1 << UART_LCR_BEC_SHIFT)
#  define UART_LCR_BEC_7          (2 << UART_LCR_BEC_SHIFT)
#  define UART_LCR_BEC_8          (3 << UART_LCR_BEC_SHIFT)


/* UARTx Modem status register */
/* ... */

#define UART_LSR_ERROR_SHIFT     7 /* Error indicating bit field */
#define UART_LSR_ERROR_MASK      (1 << UART_LSR_ERROR_SHIFT)
#define UART_LSR_ERROR           (1 << UART_LSR_ERROR_SHIFT)

#define UART_LSR_TE_SHIFT        6 /* Tx Empty bit field */
#define UART_LSR_TE_MASK         (1 << UART_LSR_TE_SHIFT)
#define UART_LSR_TE              (1 << UART_LSR_TE_SHIFT)

#define UART_LSR_TFE_SHIFT       5 /* Tx FIFO Empty bit field */
#define UART_LSR_TFE_MASK        (1 << UART_LSR_TFE_SHIFT)
#define UART_LSR_TFE             (1 << UART_LSR_TFE_SHIFT)

#define UART_LSR_BI_SHIFT        4 /* Break interrupt indicating bit filed */
#define UART_LSR_BI_MASK         (1 << UART_LSR_BI_SHIFT)
#define UART_LSR_BI              (1 << UART_LSR_BI_SHIFT)

#define UART_LSR_FE_SHIFT        3 /* Frame error indicating bit field */
#define UART_LSR_FE_MASK         (1 << UART_LSR_BI_SHIFT)
#define UART_LSR_FE              (1 << UART_LSR_BI_SHIFT)

#define UART_LSR_PE_SHIFT        2 /* Error bit of parity check bit */
#define UART_LSR_PE_MASK         (1 << UART_LSR_PE_SHIFT)
#define UART_LSR_PE              (1 << UART_LSR_PE_SHIFT)

#define UART_LSR_OE_SHIFT        1 /* Overflow indicating bit field */
#define UART_LSR_OE_MASK         (1 << UART_LSR_OE_SHIFT)
#define UART_LSR_OE              (1 << UART_LSR_OE_SHIFT)

#define UART_LSR_DR_SHIFT        0 /* Rx data valid bit field */
#define UART_LSR_DR_MASK         (1 << UART_LSR_DR_SHIFT)
#define UART_LSR_DR              (1 << UART_LSR_DR_SHIFT)

/* UARTx Modem status register */
/* ... */

/* UARTx Control Register */

#define UART_CR_EN_SHIFT        0 /* Serial port enable bit field */
#define UART_CR_EN_MASK         (1 << UART_CR_EN_SHIFT)
#define UART_CR_EN              (1 << UART_CR_EN_SHIFT)

#define UART_CR_IRDA_EN_SHIFT   1 /* Infrared interface enable bit field */
#define UART_CR_IRDA_EN_MASK    (1 << UART_CR_IRDA_EN_SHIFT)
#define UART_CR_IRDA_EN         (1 << UART_CR_IRDA_EN_SHIFT)

#define UART_CR_SC_EN_SHIFT     0 /* Smart card interface enable bit field */
#define UART_CR_SC_EN_MASK      (1 << UART_CR_SC_EN_SHIFT)
#define UART_CR_SC_EN           (1 << UART_CR_SC_EN_SHIFT)

/* UARTx Mode register */

#define UART_MR_MODE_SHIFT      3 /* UARTx mode bit field */
#define UART_MR_MODE_MASK       (0xf << UART_MR_MODE_SHIFT)
#  define UART_MR_MODE_NORMAL   (0 << UART_MR_MODE_SHIFT)
#  define UART_MR_MODE_SERIAL   (1 << UART_MR_MODE_SHIFT)
#  define UART_MR_MODE_HANDSHAKE (2 << UART_MR_MODE_SHIFT)
#  define UART_MR_MODE_MODEM    (3 << UART_MR_MODE_SHIFT)
#  define UART_MR_MODE_ISO7816T0 (4 << UART_MR_MODE_SHIFT)
#  define UART_MR_MODE_ISO7816T1 (6 << UART_MR_MODE_SHIFT)
#  define UART_MR_MODE_IRDA     (8 << UART_MR_MODE_SHIFT)


/* UARTx FIDIR regiser */

#define UART_FIDIR_FI_DI_RATIO_SHIFT  11
#define UART_FIDIR_FI_DI_RATIO_MASK   (0x7ff << UART_FIDIR_FI_DI_RATIO_SHIFT)
#  define UART_FIDIR_FI_DI_RATIO_NONE (0 << UART_FIDIR_FI_DI_RATIO_SHIFT)



/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

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
#endif /* __ARCH_MIPS_SRC_LOONGSON1X_UART_H */
