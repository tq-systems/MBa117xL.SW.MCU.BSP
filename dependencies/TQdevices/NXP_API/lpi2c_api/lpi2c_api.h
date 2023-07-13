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

#ifndef _LPI2C_API_H_
#define _LPI2C_API_H_

/*!
 * \defgroup LPI2C_API LPI2C_API
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
#include "fsl_lpi2c.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 * \brief Representation of LPI2C transfer mode
 */
typedef enum LPI2C_TransferMode
{
  LPI2C_NonBlockingMode, /**< Non Blocking Mode*/
  LPI2C_BlockingMode     /**< Blocking Mode*/
} LPI2C_TransferMode_t;

/*!
 * \brief Representation of LPI2C master device
 * \note Refer to NXP's documentation of each element of this struct for
 * further information.
 */
typedef struct LPI2C_masterDevice
{
  LPI2C_Type             *base_addr;    /**< Pointer to LPI2C instance.*/
  lpi2c_master_config_t   masterConfig; /**< Configuration of LPI2C instance.*/
  lpi2c_master_transfer_t masterXfer;   /**< Transfer configuration.*/
  lpi2c_master_handle_t  *masterHandle; /**< LPI2C_Handle. */
  LPI2C_TransferMode_t    mode;         /**< Blocking or NonBlocking mode*/
} LPI2C_masterDevice_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

extern void LPI2C_scanBus(LPI2C_masterDevice_t *master);

/*!@}*/

#endif /* _LPI2C_API_H_ */
