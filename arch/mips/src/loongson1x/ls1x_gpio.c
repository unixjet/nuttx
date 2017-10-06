/****************************************************************************
 * arch/mips/src/loongson1x/ls1x_gpio.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>

#include "up_arch.h"

#include "chip.h"
#include "ls1x_gpio.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/
/* Base addresses for each GPIO block */

const uint32_t g_gpiobase[CHIP_NPORTS] =
{
#if CHIP_NPORTS > 0
  LS1X_GPIO0_K1BASE,
#endif
#if CHIP_NPORTS > 1
  LS1X_GPIO1_K1BASE,
#endif
#if CHIP_NPORTS > 2
  LS1X_GPIO2_K1BASE,
#endif
#if CHIP_NPORTS > 3
  LS1X_GPIO3_K1BASE,
#endif

};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  ls1x_gpioinit
 *
 * Description:
 *   Based on configuration within the .config file, it does:
 *    - Remaps positions of alternative functions.
 *
 *   Typically called from ls1x_start().
 *
 * Assumptions:
 *   This function is called early in the initialization sequence so that
 *   no mutual exlusion is necessary.
 *
 ****************************************************************************/

void ls1x_gpioinit(void)
{

}

/****************************************************************************
 * Name: ls1x_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *   Once it is configured as Alternative (GPIO_ALT|GPIO_CNF_AFPP|...)
 *   function, it must be unconfigured with stm32_unconfiggpio() with
 *   the same cfgset first before it can be set to non-alternative function.
 *
 * Returns:
 *   OK on success
 *   A negated errono valu on invalid port, or when pin is locked as ALT
 *   function.
 *
 ****************************************************************************/
int stm32_configgpio(uint32_t cfgset)
{
  uintptr_t base;
  uint32_t regval;
  uint32_t setting;
  unsigned int regoffset;
  unsigned int port;
  unsigned int pin;
  unsigned int pos;
  unsigned int pinmode;
  irqstate_t flags;

  /* Verify that this hardware supports the select GPIO port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port >= CHIP_NPORTS)
    {
      return -EINVAL;
    }

  /* Get the port base address */

  base = g_gpiobase[port];

  /* Get the pin number and select the port configuration register for that
   * pin
   */

  pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  /* Set up the mode register (and remember whether the pin mode) */

  switch (cfgset & GPIO_MODE_MASK)
    {
      default:
      case GPIO_INPUT:      /* Input mode */
        pinmode = GPIO_MODER_INPUT;
        break;

      case GPIO_OUTPUT:     /* General purpose output mode */
        stm32_gpiowrite(cfgset, (cfgset & GPIO_OUTPUT_SET) != 0); /* Set the initial output value */
        pinmode = GPIO_MODER_OUTPUT;
        break;

      case GPIO_ALT:        /* Alternate function mode */
        pinmode = GPIO_MODER_ALT;
        break;
#if 0
      case GPIO_ANALOG:     /* Analog mode */
        pinmode = GPIO_MODER_ANALOG;
        break;
#endif 
    }

  /* Interrupts must be disabled from here on out so that we have mutually
   * exclusive access to all of the GPIO configuration registers.
   */

  flags = enter_critical_section();

  /* Now apply the configuration to the mode register */

  regval = getreg32(base + LS1X_GPIO_CBUS_FIRST_OFFSET);
  regval &= ~(1 << pin);
  putreg32(regval, base + LS1X_GPIO_CBUS_FIRST_OFFSET);

  regval = getreg32(base + LS1X_GPIO_CBUS_SECOND_OFFSET);
  regval &= ~(1 << pin);
  putreg32(regval, base + LS1X_GPIO_CBUS_SECOND_OFFSET);

  regval = getreg32(base + LS1X_GPIO_CBUS_THIRD_OFFSET);
  regval &= ~(1 << pin);
  putreg32(regval, base + LS1X_GPIO_CBUS_THIRD_OFFSET);

  regval = getreg32(base + LS1X_GPIO_CBUS_FOURTH_OFFSET);
  regval &= ~(1 << pin);
  putreg32(regval, base + LS1X_GPIO_CBUS_FOURTH_OFFSET);

  regval = getreg32(base + LS1X_GPIO_CBUS_FIFTH_OFFSET);
  regval &= ~(1 << pin);
  putreg32(regval, base + LS1X_GPIO_CBUS_FIFTH_OFFSET);

  if (pinmode == GPIO_MODE_INPUT || pinmode == GPIO_MODE_OUTPUT)
	{
	  regval = getreg32(base + LS1X_GPIO_CFG_OFFSET);
	  regval |= LS1X_GPIO_CFG(pin);
	  putreg32(reg, base + LS1X_GPIO_CFG_OFFSET);
	}
  else if (pinmode == GPIO_MODE_ALT)
	{
	  setting = (cfgset & GPIO_AF_MASK) >> GPIO_AF_SHIFT;
	  regoffset = LS1X_GPIO_CBUS_FIRST_OFFSET + (setting << 4);

	  if (regoffset > LS1X_GPIO_CBUS_FIFTH_OFFSET) 
		{
		  retunrn -EINVAL;
		}

	  regval = getreg32(base + regoffset);
	  regval |= 1 << pin;
	  putreg32(regval, base + regoffset);

	}
  else 
	{
	  /* why we get here */
	}


#if 0
  /* Set up the pull-up/pull-down configuration (all but analog pins) */

  setting = GPIO_PUPDR_NONE;
  if (pinmode != GPIO_MODER_ANALOG)
    {
      switch (cfgset & GPIO_PUPD_MASK)
        {
          default:
          case GPIO_FLOAT:      /* No pull-up, pull-down */
            break;

          case GPIO_PULLUP:     /* Pull-up */
            setting = GPIO_PUPDR_PULLUP;
            break;

          case GPIO_PULLDOWN:   /* Pull-down */
            setting = GPIO_PUPDR_PULLDOWN;
            break;
        }
    }

  regval  = getreg32(base + STM32_GPIO_PUPDR_OFFSET);
  regval &= ~GPIO_PUPDR_MASK(pin);
  regval |= (setting << GPIO_PUPDR_SHIFT(pin));
  putreg32(regval, base + STM32_GPIO_PUPDR_OFFSET);


  /* Set the alternate function (Only alternate function pins) */

  if (pinmode == GPIO_MODER_ALT)
    {
      setting = (cfgset & GPIO_AF_MASK) >> GPIO_AF_SHIFT;
    }
  else
    {
      setting = 0;
    }

  if (pin < 8)
    {
      regoffset = STM32_GPIO_AFRL_OFFSET;
      pos       = pin;
    }
  else
    {
      regoffset = STM32_GPIO_AFRH_OFFSET;
      pos       = pin - 8;
    }

  regval  = getreg32(base + regoffset);
  regval &= ~GPIO_AFR_MASK(pos);
  regval |= (setting << GPIO_AFR_SHIFT(pos));
  putreg32(regval, base + regoffset);

  /* Set speed (Only outputs and alternate function pins) */

  if (pinmode == GPIO_MODER_OUTPUT || pinmode == GPIO_MODER_ALT)
    {
      switch (cfgset & GPIO_SPEED_MASK)
        {
          case GPIO_SPEED_2MHz:    /* 2 MHz Low speed output */
            setting = GPIO_OSPEED_2MHz;
            break;

          case GPIO_SPEED_25MHz:   /* 25 MHz Medium speed output */
            setting = GPIO_OSPEED_25MHz;
            break;

          case GPIO_SPEED_50MHz:   /* 50 MHz Fast speed output  */
            setting = GPIO_OSPEED_50MHz;
            break;
        }
    }
  else
    {
      setting = 0;
    }

  regval  = getreg32(base + STM32_GPIO_OSPEED_OFFSET);
  regval &= ~GPIO_OSPEED_MASK(pin);
  regval |= (setting << GPIO_OSPEED_SHIFT(pin));
  putreg32(regval, base + STM32_GPIO_OSPEED_OFFSET);

  /* Set push-pull/open-drain (Only outputs and alternate function pins) */

  regval  = getreg32(base + STM32_GPIO_OTYPER_OFFSET);
  setting = GPIO_OTYPER_OD(pin);

  if ((pinmode == GPIO_MODER_OUTPUT || pinmode == GPIO_MODER_ALT) &&
      (cfgset & GPIO_OPENDRAIN) != 0)
    {
      regval |= setting;
    }
  else
    {
      regval &= ~setting;
    }

  putreg32(regval, base + STM32_GPIO_OTYPER_OFFSET);

  /* Otherwise, it is an input pin.  Should it configured as an EXTI interrupt? */

  if (pinmode != GPIO_MODER_OUTPUT && (cfgset & GPIO_EXTI) != 0)
    {
      /* "In STM32 F1 the selection of the EXTI line source is performed through
       *  the EXTIx bits in the AFIO_EXTICRx registers, while in F2 series this
       *  selection is done through the EXTIx bits in the SYSCFG_EXTICRx registers.
       *
       * "Only the mapping of the EXTICRx registers has been changed, without any
       *  changes to the meaning of the EXTIx bits. However, the range of EXTI
       *  bits values has been extended to 0b1000 to support the two ports added
       *  in F2, port H and I (in F1 series the maximum value is 0b0110)."
       */

      uint32_t regaddr;
      int shift;

      /* Set the bits in the SYSCFG EXTICR register */

      regaddr = STM32_SYSCFG_EXTICR(pin);
      regval  = getreg32(regaddr);
      shift   = SYSCFG_EXTICR_EXTI_SHIFT(pin);
      regval &= ~(SYSCFG_EXTICR_PORT_MASK << shift);
      regval |= (((uint32_t)port) << shift);

      putreg32(regval, regaddr);
    }

#endif  /* #if 0 */

  leave_critical_section(flags);
  return OK;
}

