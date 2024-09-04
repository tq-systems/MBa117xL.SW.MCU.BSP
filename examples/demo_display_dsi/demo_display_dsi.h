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

#ifndef _DEMO_DISPLAY_DSI_H_
#define _DEMO_DISPLAY_DSI_H_

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
#define PORTEXP_IO_MASK_0 0xF0
#define PORTEXP_IO_MASK_1 0x00

#endif /* _DEMO_DISPLAY_DSI_H_ */
