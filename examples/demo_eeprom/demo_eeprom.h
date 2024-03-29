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

#ifndef _DEMO_EEPROM_H_
#define _DEMO_EEPROM_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Definition of master device parameters */
#define LPI2C_BASE_ADDR LPI2C6
#define LPI2C_MASTER_CLK_FREQ BOARD_BOOTCLOCKRUN_LPSPI6_CLK_ROOT
#define I2C_BITRATE 100000U

/* Definition of device parameters */
#define EEPROM_MAIN_ADDRESS 0x54

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

#endif /* _DEMO_EEPROM_H_ */
