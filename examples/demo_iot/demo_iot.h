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

#ifndef IOT_IOT_H_
#define IOT_IOT_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "fsl_lpuart.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "clock_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define GNSS_IOT_UART LPUART11
#define GNSS_IOT_UART_CLOCK_ROOT BOARD_BOOTCLOCKRUN_LPUART11_CLK_ROOT
#define MAIN_IOT_UART LPUART6
#define MAIN_IOT_UART_CLOCK_ROOT BOARD_BOOTCLOCKRUN_LPUART6_CLK_ROOT
#define PORTEXP_I2C_DEVICE_ADR 0x20
#define LPI2C_BASE_ADDR LPI2C3
#define LPI2C_BITRATE 100000
#define LPI2C_CLOCK_FREQUENCY BOARD_BOOTCLOCKRUN_LPI2C3_CLK_ROOT

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

#endif /* IOT_IOT_H_ */
