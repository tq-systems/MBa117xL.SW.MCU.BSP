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

#ifndef _DEMO_AFE_H_
#define _DEMO_AFE_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fsl_lpspi.h"
#include "lpspi_api.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*definition of master device parameters*/
#define SPI_BASE_ADDR LPSPI1
#define SPI_TRANSFER_BAUDRATE 400000
#define SPI_CLOCK_FREQUENCY BOARD_BOOTCLOCKRUN_LPSPI1_CLK_ROOT

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

#endif /* _DEMO_AFE_H_ */
