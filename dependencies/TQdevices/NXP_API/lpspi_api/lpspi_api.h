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

#ifndef _LPSPI_API_H_
#define _LPSPI_API_H_

/*!
 * \defgroup LPSPI_API LPSPI_API
 * \brief Contains structure that summarize all peripheral relevant types from
 * NXP.
 * @{
 * \file
 * \details This file defines structure(s) that encapsulate relevant
 * peripheral types, offering a streamlined and consistent interface
 * for interacting with the specified peripheral(s).
 * The structure(s) are designed to simplify peripheral management by
 * consolidating necessary configurations and handles into cohesive entities.
 * This approach aims to facilitate
 * peripheral initialization, configuration, and operation, making it more
 * intuitive and efficient for developers to work with peripherals in
 * their applications, regardless of the number of structures defined.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fsl_lpspi.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef enum LPSPI_TransferMode
{
  LPSPI_NonBlockingMode, /**< Non Blocking Mode.*/
  LPSPI_BlockingMode     /**< Blocking Mode.*/
} LPSPI_TransferMode_t;

/*!
 * \brief Representation of LPSPI master instance
 * \note Refer to NXP's documentation of each element of this struct for
 * further information.
 */
typedef struct LPSPI_device
{
  LPSPI_Type            *baseAddr;     /**< Pointer to LPSPI instance.*/
  lpspi_master_config_t  masterConfig; /**< Configuration of LPSPI instance.*/
  lpspi_transfer_t       masterXfer;   /**< Transfer configuration.*/
  lpspi_master_handle_t *masterHandle; /**< LPSPI_Handle.*/
  LPSPI_TransferMode_t   transferMode; /**< Blocking or NonBlocking mode*/
} LPSPI_device_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!@}*/

#endif /* _LPSPI_API_H_ */
