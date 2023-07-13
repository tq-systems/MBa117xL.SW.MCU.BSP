//******************************************************************************
/*!
 * \copyright
 * SPDX-License-Identifier: BSD-3-Clause
 * \copyright
 * Copyright (c) 2021 - 2023 TQ-Systems GmbH <license@tq-group.com>,
 * D-82229 Seefeld, Germany.
 * Author: Isaac L. L. Yuki
 */
//******************************************************************************

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "board.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * \brief  This is the main application entry point where system is initialized
 *         and all system components are started.
 */
int main(void)
{
  BOARD_Initialize();

  /* Print a note to terminal. */
  PRINTF("GPIO Driver example.\r\n");
  PRINTF("The LED is blinking.\r\n");

  /* Init output LED GPIO. */
  USER_LED_INIT(0);

  while (1)
  {
    SDK_DelayAtLeastUs(500000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
#if (defined(FSL_FEATURE_IGPIO_HAS_DR_TOGGLE)                                  \
     && (FSL_FEATURE_IGPIO_HAS_DR_TOGGLE == 1))
    GPIO_PortToggle(BOARD_USER_LED_GPIO, 1u << BOARD_USER_LED_GPIO_PIN);
#else
    USER_LED_TOGGLE();
#endif /* FSL_FEATURE_IGPIO_HAS_DR_TOGGLE */
  }
}
