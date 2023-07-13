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

#ifndef _FLEXCAN_API_H_
#define _FLEXCAN_API_H_

/*!
 * \defgroup FLEXCAN_API FLEXCAN_API
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
#include "fsl_flexcan.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/**
 * \brief FlexCAN device configuration structure.
 * \note Refer to NXP's documentation of each element of this struct for
 * further information.
 */
typedef struct FlexCAN_device
{
  CAN_Type *base_addr;    /**< Base address of the FlexCAN instance. */
  uint32_t  srcClockRoot; /**< Source clock root. */

  /**
   * \brief Identifiers for FlexCAN device.
   */
  struct Identifier
  {
    uint32_t rx; /**< Receive identifier. */
    uint32_t tx; /**< Transmit identifier. */
  } identifier;  /**< Structure for FlexCAN Identifiers. */

  flexcan_frame_t       frame;  /**< CAN frame structure for message buffers. */
  flexcan_mb_transfer_t txXfer; /**< Transfer structure for transmit. */
  flexcan_mb_transfer_t rxXfer; /**< Transfer structure for receive. */
  flexcan_handle_t      handle; /**< FlexCAN handle. */
  flexcan_config_t      config; /**< Configuration structure for FlexCAN. */
  flexcan_rx_mb_config_t
    mbConfig; /**< Message buffer configuration structure. */
} FlexCAN_device_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!@}*/

#endif /* _FLEXCAN_API_H_ */
