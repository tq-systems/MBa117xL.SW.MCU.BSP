//*****************************************************************************
/*!
 * \copyright
 * SPDX-License-Identifier: BSD-3-Clause
 * \copyright
 * Copyright (c) 2021 - 2023 TQ-Systems GmbH <license@tq-group.com>,
 * D-82229 Seefeld, Germany.
 * Author: Isaac L. L. Yuki, Forstner
 */
//******************************************************************************

#ifndef PMIC_H_
#define PMIC_H_

/*!
 * \defgroup PMIC PMIC
 * \brief PMIC Driver.
 * @{
 * \file
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>
#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * \brief Enumeration for selecting voltage levels for VDD_SOC.
 * This enumeration is used to set different voltage levels for the System on
 * Chip (SoC) power supply (VDD_SOC) through the PMIC.
 */
typedef enum PMIC_VCC_SEL
{
  PMIC_VDD_SOC_1V000 = 96,  /**< Selects a core voltage of 1.000V for the SoC.*/
  PMIC_VDD_SOC_1V100 = 112, /**< Selects a core voltage of 1.100V for the SoC.*/
  PMIC_VDD_SOC_0V900 = 80,  /**< Selects a core voltage of 0.900V for the SoC.*/
} PMIC_VCC_SEL_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

extern status_t PMIC_setCoreVoltage(PMIC_VCC_SEL_t newVoltage);
extern status_t PMIC_readCoreVoltage(float *voltage);
extern status_t PMIC_readDeviceID(uint8_t *deviceId);

/*!@}*/

#endif /* PMIC_H_ */
