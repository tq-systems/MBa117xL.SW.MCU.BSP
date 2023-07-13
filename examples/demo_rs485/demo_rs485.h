//******************************************************************************
/*!
 * \copyright
 * SPDX-License-Identifier: BSD-3-Clause
 * \copyright
 * Copyright (c) 2021 - 2023 TQ-Systems GmbH <license@tq-group.com>,
 * D-82229 Seefeld, Germany.
 * Author: Isaac L. L. Yuki, Bernhardt Herz
 */
//******************************************************************************

#ifndef _DEMO_RS485_H_
#define _DEMO_RS485_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "fsl_lpuart.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"
#include "clock_config.h"
#include "fsl_gpio.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

#define LPUART_RS485_PERIPHERAL LPUART8
#define LPUART_RS485_PERIPHERAL_CLOCK_ROOT BOARD_BOOTCLOCKRUN_LPUART8_CLK_ROOT

#define RS485_TRAN_EN_PORT BOARD_INITPINS_LPUART8_CTS_PERIPHERAL
#define RS485_TRAN_EN_PIN BOARD_INITPINS_LPUART8_CTS_CHANNEL
#define RS485_UART_BAUDRATE 115200

/******************************************************************************/

#endif /* _DEMO_RS485_H_ */
