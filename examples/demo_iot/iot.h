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

#ifndef IOT_H
#define IOT_H

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "lpi2c_api.h"
#include "PCA9555BS.h"
#include "lpuart_api.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

extern status_t IOT_start(LPUART_Peripheral_t *peripheral);

extern status_t IOT_executeCommand(unsigned char       *commandString,
                                   size_t               stringSize,
                                   LPUART_Peripheral_t *peripheral);

extern void IOT_initMainUart(LPUART_Peripheral_t *main_uart_peripheral);

extern void IOT_initGNSSPeripheral(LPUART_Peripheral_t *gnss_uart_peripheral);

extern status_t IOT_read(LPUART_Peripheral_t *peripheral);

/*******************************************************************************
 * Variables
 ******************************************************************************/

#endif /* IOT_H_ */
