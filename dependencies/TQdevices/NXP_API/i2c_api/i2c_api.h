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

#ifndef _I2C_API_H_
#define _I2C_API_H_

/*!
 * \defgroup I2C_API I2C_API
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

#include "fsl_i2c.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 * \brief Representation of I2C transfer mode
 */
typedef enum I2C_TransferMode
{
  I2C_NonBlockingMode, /**< Non Blocking Mode.*/
  I2C_BlockingMode     /**< Blocking Mode.*/
} I2C_TransferMode_t;

/*!
 * \brief Representation of I2C master device
 * \note Refer to NXP's documentation of each element of this struct for
 * further information.
 */
typedef struct I2C_masterDevice
{
  I2C_Type             *base_addr;    /**< Pointer to I2C instance.*/
  i2c_master_config_t   masterConfig; /**< Configuration of I2C instance.*/
  i2c_master_transfer_t masterXfer;   /**< Transfer configuration.*/
  i2c_master_handle_t  *masterHandle; /**< I2C_Handle. */
  I2C_TransferMode_t    mode;         /**< Blocking or NonBlocking mode*/
} I2C_masterDevice_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

extern void I2C_scanBus(I2C_masterDevice_t *master);

/*!@}*/

#endif /* _I2C_API_H_ */
