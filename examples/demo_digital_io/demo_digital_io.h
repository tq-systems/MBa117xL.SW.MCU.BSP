//*****************************************************************************
/*!
 * \copyright
 * SPDX-License-Identifier: BSD-3-Clause
 * \copyright
 * Copyright (c) 2021 - 2023 TQ-Systems GmbH <license@tq-group.com>,
 * D-82229 Seefeld, Germany.
 * Author: Isaac L. L. Yuki
 */
//******************************************************************************

#ifndef _DEMO_DIGITAL_IO_H_
#define _DEMO_DIGITAL_IO_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/

/*******************************************************************************
 * includes
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define LPI2C_BASE_ADDR LPI2C3
#define LPI2C_CLOCK_FREQUENCY BOARD_BOOTCLOCKRUN_LPI2C3_CLK_ROOT
#define LPI2C_BITRATE 100000U

#define PORTEXP_I2C_DEVICE_ADR 0x20

#define DIG_IN_1_GPIO GPIO10
#define DIG_IN_1_GPIO_PIN 16U

#define DIG_IN_2_GPIO GPIO10
#define DIG_IN_2_GPIO_PIN 17U

#define DIG_IN_3_GPIO GPIO10
#define DIG_IN_3_GPIO_PIN 18U

#define DIG_IN_4_GPIO GPIO10
#define DIG_IN_4_GPIO_PIN 19U

#endif /* _DEMO_DIGITAL_IO_H_ */
