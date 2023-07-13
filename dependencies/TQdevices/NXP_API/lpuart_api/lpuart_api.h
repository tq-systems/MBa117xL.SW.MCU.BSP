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

#ifndef _LPUART_API_H_
#define _LPUART_API_H_

/*!
 * \defgroup LPUART_API LPUART_API
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

#ifdef USE_FREE_RTOS
#include "fsl_lpuart_freertos.h"
#endif
#include "fsl_lpuart.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#ifdef USE_FREE_RTOS
/**
 * \brief LPUART peripheral structure for RTOS.
 * \note Refer to NXP's documentation of each element of this struct for
 * further information.
 */
typedef struct LPUART_Rtos_Peripheral
{
  size_t bufferCount;          /**< Number of bytes in buffer for the LPUART. */
  lpuart_rtos_config_t config; /**< RTOS configuration for the LPUART. */
  lpuart_rtos_handle_t handle; /**< RTOS handle for the LPUART. */
  lpuart_handle_t      t_handle; /**< Transfer handle for the LPUART. */
} LPUART_Rtos_Peripheral_t;
#endif

/*!
 * \brief LPUART peripheral structure.
 * \note Refer to NXP's documentation of each element of this struct for
 * further information.
 */
typedef struct LPUART_Peripheral
{
  LPUART_Type      *base;     /**< Base address of the LPUART instance. */
  lpuart_config_t   config;   /**< Configuration structure for LPUART. */
  lpuart_handle_t   handle;   /**< Handle for the LPUART. */
  lpuart_transfer_t transfer; /**< Transfer structure for LPUART. */
} LPUART_Peripheral_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!@}*/

#endif /* _LPUART_API_H_ */
