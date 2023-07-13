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

#ifndef _DEMO_SE97BTP_H_
#define _DEMO_SE97BTP_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "clock_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* peripheral */
#define LPI2C_BASE_ADDR LPI2C6
#define LPI2C_MASTER_CLK_FREQ BOARD_BOOTCLOCKRUN_LPI2C6_CLK_ROOT
#define I2C_BITRATE 100000U

/* definition of sensor device parameters */
#define S97BTP_TEMP_MAINBOARD 0x1B
#define S97BTP_EEPROM_MAINBOARD 0x53

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

#endif /* _DEMO_SE97BTP_H_ */
