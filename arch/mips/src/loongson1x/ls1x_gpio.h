/************************************************************************************
 * arch/mips/src/loongson1x/ls1x_gpio.h
 *
 *   Copyright (C) 2009, 2011-2012, 2015 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_LOONGSON_LOONGSON1X_GPIO_H
#define __ARCH_ARM_SRC_LOONGSON_LOONGSON1X_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

#include <nuttx/irq.h>

#include "chip.h"


/************************************************************************************
 * Pre-Processor Declarations
 ************************************************************************************/


/* Register Offsets *****************************************************************/

#define LS1X_GPIO_CFG_OFFSET              0x0000
#  define LS1X_GPIO_CFG(N)               (1 << N)

#define LS1X_GPIO_EN_OFFSET               0x0010
#  define  LS1X_GPIO_EN_INPUT(N)         (1 << N)

#define LS1X_GPIO_IN_OFFSET               0x0020
#define LS1X_GPIO_OUT_OFFSET              0x0030
#define LS1X_GPIO_CBUS_FIRST_OFFSET       0x0100
#define LS1X_GPIO_CBUS_SECOND_OFFSET      0x0110
#define LS1X_GPIO_CBUS_THIRD_OFFSET       0x0120
#define LS1X_GPIO_CBUS_FOURTH_OFFSET      0x0130
#define LS1X_GPIO_CBUS_FIFTH_OFFSET       0x0140


/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Base addresses for each GPIO block */

EXTERN const uint32_t g_gpiobase[CHIP_NPORTS];

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/* Each port bit of the general-purpose I/O (GPIO) ports can be individually configured
 * by software in several modes:
 *
 *  - Input floating
 *  - Input pull-up
 *  - Input-pull-down
 *  - Output open-drain with pull-up or pull-down capability
 *  - Output push-pull with pull-up or pull-down capability
 *  - Alternate function push-pull with pull-up or pull-down capability
 *  - Alternate function open-drain with pull-up or pull-down capability
 *  - Analog
 *
 * 20-bit Encoding:       1111 1111 1100 0000 0000
 *                        9876 5432 1098 7654 3210
 *                        ---- ---- ---- ---- ----
 * Inputs:                MMUU .... ...X PPPP BBBB
 * Outputs:               MMUU .... FFOV PPPP BBBB
 * Alternate Functions:   MMUU AAAA FFO. PPPP BBBB
 * Analog:                MM.. .... .... PPPP BBBB
 */

/* Mode:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * MM.. .... .... .... ....
 */

#define GPIO_MODE_SHIFT               (18)                       /* Bits 18-19: GPIO port mode */
#define GPIO_MODE_MASK                (3 << GPIO_MODE_SHIFT)
#  define GPIO_INPUT                  (0 << GPIO_MODE_SHIFT)     /* Input mode */
#  define GPIO_OUTPUT                 (1 << GPIO_MODE_SHIFT)     /* General purpose output mode */
#  define GPIO_ALT                    (2 << GPIO_MODE_SHIFT)     /* Alternate function mode */
//#  define GPIO_ANALOG                 (3 << GPIO_MODE_SHIFT)     /* Analog mode */

/* Input/output pull-ups/downs (not used with analog):
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * ..UU .... .... .... ....
 */

#define GPIO_PUPD_SHIFT               (16)                       /* Bits 16-17: Pull-up/pull down */
#define GPIO_PUPD_MASK                (3 << GPIO_PUPD_SHIFT)
#  define GPIO_FLOAT                  (0 << GPIO_PUPD_SHIFT)     /* No pull-up, pull-down */
#  define GPIO_PULLUP                 (1 << GPIO_PUPD_SHIFT)     /* Pull-up */
#  define GPIO_PULLDOWN               (2 << GPIO_PUPD_SHIFT)     /* Pull-down */

/* Alternate Functions:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... AAAA .... .... ....
 */

#define GPIO_AF_SHIFT                 (12)                       /* Bits 12-15: Alternate function */
#define GPIO_AF_MASK                  (15 << GPIO_AF_SHIFT)
#  define GPIO_AF(n)                  ((n) << GPIO_AF_SHIFT)
#  define GPIO_AF0                    (0 << GPIO_AF_SHIFT)
#  define GPIO_AF1                    (1 << GPIO_AF_SHIFT)
#  define GPIO_AF2                    (2 << GPIO_AF_SHIFT)
#  define GPIO_AF3                    (3 << GPIO_AF_SHIFT)
#  define GPIO_AF4                    (4 << GPIO_AF_SHIFT)

/****  reserved ***
 *
#  define GPIO_AF5                    (5 << GPIO_AF_SHIFT)
#  define GPIO_AF6                    (6 << GPIO_AF_SHIFT)
#  define GPIO_AF7                    (7 << GPIO_AF_SHIFT)
#  define GPIO_AF8                    (8 << GPIO_AF_SHIFT)
#  define GPIO_AF9                    (9 << GPIO_AF_SHIFT)
#  define GPIO_AF10                   (10 << GPIO_AF_SHIFT)
#  define GPIO_AF11                   (11 << GPIO_AF_SHIFT)
#  define GPIO_AF12                   (12 << GPIO_AF_SHIFT)
#  define GPIO_AF13                   (13 << GPIO_AF_SHIFT)
#  define GPIO_AF14                   (14 << GPIO_AF_SHIFT)
#  define GPIO_AF15                   (15 << GPIO_AF_SHIFT)
  *************************/

/* Output/Alt function frequency selection:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... FF.. .... ....
 */

#define GPIO_SPEED_SHIFT              (10)                       /* Bits 10-11: GPIO frequency selection */
#define GPIO_SPEED_MASK               (3 << GPIO_SPEED_SHIFT)
#if defined(CONFIG_STM32_STM32L15XX)
#  define GPIO_SPEED_400KHz           (0 << GPIO_SPEED_SHIFT)     /* 400 kHz Very low speed output */
#  define GPIO_SPEED_2MHz             (1 << GPIO_SPEED_SHIFT)     /* 2 MHz Low speed output */
#  define GPIO_SPEED_10MHz            (2 << GPIO_SPEED_SHIFT)     /* 10 MHz Medium speed output */
#  define GPIO_SPEED_40MHz            (3 << GPIO_SPEED_SHIFT)     /* 40 MHz High speed output */
#else
#  define GPIO_SPEED_2MHz             (0 << GPIO_SPEED_SHIFT)     /* 2 MHz Low speed output */
#  define GPIO_SPEED_25MHz            (1 << GPIO_SPEED_SHIFT)     /* 25 MHz Medium speed output */
#  define GPIO_SPEED_50MHz            (2 << GPIO_SPEED_SHIFT)     /* 50 MHz Fast speed output  */
#endif

/* Output/Alt function type selection:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... ..O. .... ....
 */

#define GPIO_OPENDRAIN                (1 << 9)                   /* Bit9: 1=Open-drain output */
#define GPIO_PUSHPULL                 (0)                        /* Bit9: 0=Push-pull output */

/* If the pin is a GPIO digital output, then this identifies the initial output value.
 * If the pin is an input, this bit is overloaded to provide the qualifier to
 * distinquish input pull-up and -down:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... ...V .... ....
 */

#define GPIO_OUTPUT_SET               (1 << 8)                   /* Bit 8: If output, inital value of output */
#define GPIO_OUTPUT_CLEAR             (0)

/* External interrupt selection (GPIO inputs only):
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... ...X .... ....
 */

#define GPIO_EXTI                     (1 << 8)                    /* Bit 8: Configure as EXTI interrupt */

/* This identifies the GPIO port:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... .... PPPP ....
 */

#define GPIO_PORT_SHIFT               (5)                        /* Bit 5-7:  Port number */
#define GPIO_PORT_MASK                (15 << GPIO_PORT_SHIFT)
#  define GPIO_PORT0                  (0 << GPIO_PORT_SHIFT)     /*   GPIO0 */
#  define GPIO_PORT1                  (1 << GPIO_PORT_SHIFT)     /*   GPIO1 */
#  define GPIO_PORT2                  (2 << GPIO_PORT_SHIFT)     /*   GPIO2 */
#  define GPIO_PORT3                  (3 << GPIO_PORT_SHIFT)     /*   GPIO3 */
#  define GPIO_PORT4                  (4 << GPIO_PORT_SHIFT)     /*   GPIO4 */

/* This identifies the bit in the port:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... .... .... BBBB
 */

#define GPIO_PIN_SHIFT                (0)                        /* Bits 0-4: GPIO number: 0-31 */
#define GPIO_PIN_MASK                 (31 << GPIO_PIN_SHIFT)

#  define GPIO_PIN0                   (0 << GPIO_PIN_SHIFT)
#  define GPIO_PIN1                   (1 << GPIO_PIN_SHIFT)
#  define GPIO_PIN2                   (2 << GPIO_PIN_SHIFT)
#  define GPIO_PIN3                   (3 << GPIO_PIN_SHIFT)
#  define GPIO_PIN4                   (4 << GPIO_PIN_SHIFT)
#  define GPIO_PIN5                   (5 << GPIO_PIN_SHIFT)
#  define GPIO_PIN6                   (6 << GPIO_PIN_SHIFT)
#  define GPIO_PIN7                   (7 << GPIO_PIN_SHIFT)

#  define GPIO_PIN8                   (8 << GPIO_PIN_SHIFT)
#  define GPIO_PIN9                   (9 << GPIO_PIN_SHIFT)
#  define GPIO_PIN10                  (10 << GPIO_PIN_SHIFT)
#  define GPIO_PIN11                  (11 << GPIO_PIN_SHIFT)
#  define GPIO_PIN12                  (12 << GPIO_PIN_SHIFT)
#  define GPIO_PIN13                  (13 << GPIO_PIN_SHIFT)
#  define GPIO_PIN14                  (14 << GPIO_PIN_SHIFT)
#  define GPIO_PIN15                  (15 << GPIO_PIN_SHIFT)

#  define GPIO_PIN16                  (16 << GPIO_PIN_SHIFT)
#  define GPIO_PIN17                  (17 << GPIO_PIN_SHIFT)
#  define GPIO_PIN18                  (18 << GPIO_PIN_SHIFT)
#  define GPIO_PIN19                  (19 << GPIO_PIN_SHIFT)
#  define GPIO_PIN20                  (20 << GPIO_PIN_SHIFT)
#  define GPIO_PIN21                  (21 << GPIO_PIN_SHIFT)
#  define GPIO_PIN22                  (22 << GPIO_PIN_SHIFT)
#  define GPIO_PIN23                  (23 << GPIO_PIN_SHIFT)

#  define GPIO_PIN24                  (24 << GPIO_PIN_SHIFT)
#  define GPIO_PIN25                  (25 << GPIO_PIN_SHIFT)
#  define GPIO_PIN26                  (26 << GPIO_PIN_SHIFT)
#  define GPIO_PIN27                  (27 << GPIO_PIN_SHIFT)
#  define GPIO_PIN28                  (28 << GPIO_PIN_SHIFT)
#  define GPIO_PIN29                  (29 << GPIO_PIN_SHIFT)
#  define GPIO_PIN30                  (30 << GPIO_PIN_SHIFT)
#  define GPIO_PIN31                  (31 << GPIO_PIN_SHIFT)

#  define GPIO_PIN32                  (GPIO_PIN0 | GPIO_PORT1)
#  define GPIO_PIN33                  (33 << GPIO_PIN_SHIFT)
#  define GPIO_PIN34                  (34 << GPIO_PIN_SHIFT)
#  define GPIO_PIN35                  (35 << GPIO_PIN_SHIFT)
#  define GPIO_PIN36                  (36 << GPIO_PIN_SHIFT)
#  define GPIO_PIN37                  (37 << GPIO_PIN_SHIFT)
#  define GPIO_PIN38                  (38 << GPIO_PIN_SHIFT)
#  define GPIO_PIN39                  (39 << GPIO_PIN_SHIFT)

#  define GPIO_PIN40                  (40 << GPIO_PIN_SHIFT)
#  define GPIO_PIN41                  (41 << GPIO_PIN_SHIFT)
#  define GPIO_PIN42                  (42 << GPIO_PIN_SHIFT)
#  define GPIO_PIN43                  (43 << GPIO_PIN_SHIFT)
#  define GPIO_PIN44                  (44 << GPIO_PIN_SHIFT)
#  define GPIO_PIN45                  (45 << GPIO_PIN_SHIFT)
#  define GPIO_PIN46                  (46 << GPIO_PIN_SHIFT)
#  define GPIO_PIN47                  (47 << GPIO_PIN_SHIFT)

#  define GPIO_PIN48                  (48 << GPIO_PIN_SHIFT)
#  define GPIO_PIN49                  (49 << GPIO_PIN_SHIFT)
#  define GPIO_PIN50                  (50 << GPIO_PIN_SHIFT)
#  define GPIO_PIN51                  (51 << GPIO_PIN_SHIFT)
#  define GPIO_PIN52                  (52 << GPIO_PIN_SHIFT)
#  define GPIO_PIN53                  (53 << GPIO_PIN_SHIFT)
#  define GPIO_PIN54                  (54 << GPIO_PIN_SHIFT)
#  define GPIO_PIN55                  (55 << GPIO_PIN_SHIFT)

#  define GPIO_PIN56                  (56 << GPIO_PIN_SHIFT)
#  define GPIO_PIN57                  (57 << GPIO_PIN_SHIFT)
#  define GPIO_PIN58                  (58 << GPIO_PIN_SHIFT)
#  define GPIO_PIN59                  (59 << GPIO_PIN_SHIFT)
#  define GPIO_PIN60                  (60 << GPIO_PIN_SHIFT)
#  define GPIO_PIN61                  (61 << GPIO_PIN_SHIFT)
#  define GPIO_PIN62                  (62 << GPIO_PIN_SHIFT)
#  define GPIO_PIN63                  (63 << GPIO_PIN_SHIFT)

#  define GPIO_PIN64                  (64 << GPIO_PIN_SHIFT)
#  define GPIO_PIN65                  (65 << GPIO_PIN_SHIFT)
#  define GPIO_PIN66                  (66 << GPIO_PIN_SHIFT)
#  define GPIO_PIN67                  (67 << GPIO_PIN_SHIFT)
#  define GPIO_PIN68                  (68 << GPIO_PIN_SHIFT)
#  define GPIO_PIN69                  (69 << GPIO_PIN_SHIFT)
#  define GPIO_PIN70                  (70 << GPIO_PIN_SHIFT)
#  define GPIO_PIN71                  (71 << GPIO_PIN_SHIFT)

#  define GPIO_PIN72                  (72 << GPIO_PIN_SHIFT)
#  define GPIO_PIN73                  (73 << GPIO_PIN_SHIFT)
#  define GPIO_PIN74                  (74 << GPIO_PIN_SHIFT)
#  define GPIO_PIN75                  (75 << GPIO_PIN_SHIFT)
#  define GPIO_PIN76                  (76 << GPIO_PIN_SHIFT)
#  define GPIO_PIN77                  (77 << GPIO_PIN_SHIFT)
#  define GPIO_PIN78                  (78 << GPIO_PIN_SHIFT)
#  define GPIO_PIN79                  (79 << GPIO_PIN_SHIFT)

#  define GPIO_PIN80                  (80 << GPIO_PIN_SHIFT)
#  define GPIO_PIN81                  (81 << GPIO_PIN_SHIFT)
#  define GPIO_PIN82                  (82 << GPIO_PIN_SHIFT)
#  define GPIO_PIN83                  (83 << GPIO_PIN_SHIFT)
#  define GPIO_PIN84                  (84 << GPIO_PIN_SHIFT)
#  define GPIO_PIN85                  (85 << GPIO_PIN_SHIFT)
#  define GPIO_PIN86                  (86 << GPIO_PIN_SHIFT)
#  define GPIO_PIN87                  (87 << GPIO_PIN_SHIFT)

#  define GPIO_PIN88                  (88 << GPIO_PIN_SHIFT)
#  define GPIO_PIN89                  (89 << GPIO_PIN_SHIFT)
#  define GPIO_PIN90                  (90 << GPIO_PIN_SHIFT)
#  define GPIO_PIN91                  (91 << GPIO_PIN_SHIFT)
#  define GPIO_PIN92                  (92 << GPIO_PIN_SHIFT)
#  define GPIO_PIN93                  (93 << GPIO_PIN_SHIFT)
#  define GPIO_PIN94                  (94 << GPIO_PIN_SHIFT)
#  define GPIO_PIN95                  (95 << GPIO_PIN_SHIFT)

#  define GPIO_PIN96                  (96 << GPIO_PIN_SHIFT)
#  define GPIO_PIN97                  (97 << GPIO_PIN_SHIFT)
#  define GPIO_PIN98                  (98 << GPIO_PIN_SHIFT)
#  define GPIO_PIN99                  (99 << GPIO_PIN_SHIFT)
#  define GPIO_PIN100                 (100 << GPIO_PIN_SHIFT)
#  define GPIO_PIN101                 (101 << GPIO_PIN_SHIFT)
#  define GPIO_PIN102                 (102 << GPIO_PIN_SHIFT)
#  define GPIO_PIN103                 (103 << GPIO_PIN_SHIFT)

#  define GPIO_PIN104                 (104 << GPIO_PIN_SHIFT)
#  define GPIO_PIN105                 (105 << GPIO_PIN_SHIFT)
#  define GPIO_PIN106                 (106 << GPIO_PIN_SHIFT)
#  define GPIO_PIN107                 (107 << GPIO_PIN_SHIFT)
#  define GPIO_PIN108                 (108 << GPIO_PIN_SHIFT)
#  define GPIO_PIN109                 (109 << GPIO_PIN_SHIFT)
#  define GPIO_PIN110                 (110 << GPIO_PIN_SHIFT)
#  define GPIO_PIN111                 (111 << GPIO_PIN_SHIFT)

#  define GPIO_PIN112                 (112 << GPIO_PIN_SHIFT)
#  define GPIO_PIN113                 (113 << GPIO_PIN_SHIFT)
#  define GPIO_PIN114                 (114 << GPIO_PIN_SHIFT)
#  define GPIO_PIN115                 (115 << GPIO_PIN_SHIFT)
#  define GPIO_PIN116                 (116 << GPIO_PIN_SHIFT)
#  define GPIO_PIN117                 (117 << GPIO_PIN_SHIFT)
#  define GPIO_PIN118                 (118 << GPIO_PIN_SHIFT)
#  define GPIO_PIN119                 (119 << GPIO_PIN_SHIFT)

#  define GPIO_PIN120                 (120 << GPIO_PIN_SHIFT)
#  define GPIO_PIN121                 (121 << GPIO_PIN_SHIFT)
#  define GPIO_PIN122                 (122 << GPIO_PIN_SHIFT)
#  define GPIO_PIN123                 (123 << GPIO_PIN_SHIFT)
#  define GPIO_PIN124                 (124 << GPIO_PIN_SHIFT)
#  define GPIO_PIN125                 (125 << GPIO_PIN_SHIFT)
#  define GPIO_PIN126                 (126 << GPIO_PIN_SHIFT)
#  define GPIO_PIN127                 (127 << GPIO_PIN_SHIFT)

#  define GPIO_PIN128                 (128 << GPIO_PIN_SHIFT)
#  define GPIO_PIN129                 (129 << GPIO_PIN_SHIFT)
#  define GPIO_PIN130                 (130 << GPIO_PIN_SHIFT)
#  define GPIO_PIN131                 (131 << GPIO_PIN_SHIFT)
#  define GPIO_PIN132                 (132 << GPIO_PIN_SHIFT)
#  define GPIO_PIN133                 (133 << GPIO_PIN_SHIFT)
#  define GPIO_PIN134                 (134 << GPIO_PIN_SHIFT)
#  define GPIO_PIN135                 (135 << GPIO_PIN_SHIFT)

#  define GPIO_PIN136                 (136 << GPIO_PIN_SHIFT)
#  define GPIO_PIN137                 (137 << GPIO_PIN_SHIFT)
#  define GPIO_PIN138                 (138 << GPIO_PIN_SHIFT)
#  define GPIO_PIN139                 (139 << GPIO_PIN_SHIFT)
#  define GPIO_PIN140                 (140 << GPIO_PIN_SHIFT)
#  define GPIO_PIN141                 (141 << GPIO_PIN_SHIFT)
#  define GPIO_PIN142                 (142 << GPIO_PIN_SHIFT)
#  define GPIO_PIN143                 (143 << GPIO_PIN_SHIFT)

#  define GPIO_PIN144                 (144 << GPIO_PIN_SHIFT)
#  define GPIO_PIN145                 (145 << GPIO_PIN_SHIFT)
#  define GPIO_PIN146                 (146 << GPIO_PIN_SHIFT)
#  define GPIO_PIN147                 (147 << GPIO_PIN_SHIFT)
#  define GPIO_PIN148                 (148 << GPIO_PIN_SHIFT)
#  define GPIO_PIN149                 (149 << GPIO_PIN_SHIFT)
#  define GPIO_PIN150                 (150 << GPIO_PIN_SHIFT)
#  define GPIO_PIN151                 (151 << GPIO_PIN_SHIFT)

#  define GPIO_PIN152                 (152 << GPIO_PIN_SHIFT)
#  define GPIO_PIN153                 (153 << GPIO_PIN_SHIFT)
#  define GPIO_PIN154                 (154 << GPIO_PIN_SHIFT)
#  define GPIO_PIN155                 (155 << GPIO_PIN_SHIFT)
#  define GPIO_PIN156                 (156 << GPIO_PIN_SHIFT)
#  define GPIO_PIN157                 (157 << GPIO_PIN_SHIFT)
#  define GPIO_PIN158                 (158 << GPIO_PIN_SHIFT)
#  define GPIO_PIN159                 (159 << GPIO_PIN_SHIFT)

#  define GPIO_PIN160                 (160 << GPIO_PIN_SHIFT)
#  define GPIO_PIN161                 (161 << GPIO_PIN_SHIFT)
#  define GPIO_PIN162                 (162 << GPIO_PIN_SHIFT)
#  define GPIO_PIN163                 (163 << GPIO_PIN_SHIFT)
#  define GPIO_PIN164                 (164 << GPIO_PIN_SHIFT)
#  define GPIO_PIN165                 (165 << GPIO_PIN_SHIFT)
#  define GPIO_PIN166                 (166 << GPIO_PIN_SHIFT)
#  define GPIO_PIN167                 (167 << GPIO_PIN_SHIFT)

/************************************************************************************
 * Name: ls1x_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *   Once it is configured as Alternative (GPIO_ALT|...)
 *   function, it must be unconfigured with ls1x_unconfiggpio() with
 *   the same cfgset first before it can be set to non-alternative function.
 *
 * Returned Value:
 *   OK on success
 *   ERROR on invalid port, or when pin is locked as ALT function.
 *
 ************************************************************************************/

int ls1x_configgpio(uint32_t cfgset);

/************************************************************************************
 * Name: ls1x_unconfiggpio
 *
 * Description:
 *   Unconfigure a GPIO pin based on bit-encoded description of the pin, set it
 *   into default HiZ state (and possibly mark it's unused) and unlock it whether
 *   it was previsouly selected as alternative function (GPIO_ALT|GPIO_CNF_AFPP|...).
 *
 *   This is a safety function and prevents hardware from schocks, as unexpected
 *   write to the Timer Channel Output GPIO to fixed '1' or '0' while it should
 *   operate in PWM mode could produce excessive on-board currents and trigger
 *   over-current/alarm function.
 *
 * Returned Value:
 *  OK on success
 *  ERROR on invalid port
 *
 ************************************************************************************/

int ls1x_unconfiggpio(uint32_t cfgset);

/************************************************************************************
 * Name: ls1x_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

void ls1x_gpiowrite(uint32_t pinset, bool value);

/************************************************************************************
 * Name: ls1x_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

bool ls1x_gpioread(uint32_t pinset);

/************************************************************************************
 * Name: ls1x_gpiosetevent
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 *
 * Input Parameters:
 *  - pinset: gpio pin configuration
 *  - rising/falling edge: enables
 *  - event:  generate event when set
 *  - func:   when non-NULL, generate interrupt
 *  - arg:    Argument passed to the interrupt callback
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure indicating the
 *   nature of the failure.
 *
 ************************************************************************************/

int ls1x_gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge,
                       bool event, xcpt_t func, void *arg);

/************************************************************************************
 * Function:  ls1x_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
int ls1x_dumpgpio(uint32_t pinset, const char *msg);
#else
#  define ls1x_dumpgpio(p,m)
#endif

/************************************************************************************
 * Function:  ls1x_gpioinit
 *
 * Description:
 *   Based on configuration within the .config file, it does:
 *    - Remaps positions of alternative functions.
 *
 *   Typically called from ls1x_start().
 *
 ************************************************************************************/

void ls1x_gpioinit(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LOONGSON_LOONGSON1X_GPIO_H */
