//*****************************************************************************
/*!
 * \copyright
 * SPDX-License-Identifier: BSD-3-Clause
 * \copyright
 * Copyright (c) 2021 - 2023 TQ-Systems GmbH <license@tq-group.com>,
 * D-82229 Seefeld, Germany.
 * Author: Isaac L. L. Yuki, Bernhard Herz
 */
//******************************************************************************

/*!
 * \addtogroup PMIC
 * \brief PMIC Driver.
 * @{
 * \file
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "PMIC.h"
#include "fsl_lpi2c.h"
#include "fsl_debug_console.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define PMIC_REG_DATA_SIZE (1U)
#define PMIC_REG_ADDRESS_SIZE (1U)
#define PMIC_DEVICE_ADDRESS 0x08

#define PMIC_ERROR_NO_MATCH_WITH_FACTORY_DATA 200

typedef const enum PMIC_Banks {
  FREQ_CTRL,
  SW1_RUN_VOLT,
  SW1_PWRUP,
  SW1_CONFIG1,
  SW1_CONFIG2,
  SW2_RUN_VOLT,
  SW2_PWRUP,
  SW2_CONFIG1,
  SW2_CONFIG2,
  SW_RAMP,
  PMIC_BANK_ARRAY_SIZE
} PMIC_Banks_t;

typedef struct Register_tag
{
  uint8_t address;
  uint8_t value;
  uint8_t mask;
} Register_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

static Register_t Banks[PMIC_BANK_ARRAY_SIZE] = {
  [FREQ_CTRL] = {.address = 0x3A},   [SW1_RUN_VOLT] = {.address = 0x4B},
  [SW1_PWRUP] = {.address = 0x49},   [SW1_CONFIG1] = {.address = 0x47},
  [SW1_CONFIG2] = {.address = 0x48}, [SW2_RUN_VOLT] = {.address = 0x53},
  [SW2_PWRUP] = {.address = 0x51},   [SW2_CONFIG1] = {.address = 0x4F},
  [SW2_CONFIG2] = {.address = 0x50}, [SW_RAMP] = {.address = 0x46},
};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 *  Code
 ******************************************************************************/

/*!
 * \brief PMIC_SetCoreVoltage
 * This function configures a new VCC_SOC voltage. Range 0.9V...1.1V.
 * Voltages fixed by PMIC_VCC_SEL_t.
 * \param[in] newVoltage	Voltage by defined by PMIC_VCC_SEL_t.
 * \return Status of transfer. See API documentation for error codes.
 * \note Assumes that the I2C master is properly configured before this
 * function is called.
 */
status_t PMIC_setCoreVoltage(PMIC_VCC_SEL_t newVoltage)
{
  status_t                status;
  lpi2c_master_transfer_t i2c_config;

  /* Set transfer parameters*/
  i2c_config.flags          = kLPI2C_TransferDefaultFlag;
  i2c_config.slaveAddress   = PMIC_DEVICE_ADDRESS;
  i2c_config.dataSize       = PMIC_REG_DATA_SIZE;
  i2c_config.subaddressSize = PMIC_REG_ADDRESS_SIZE;
  i2c_config.subaddress     = Banks[SW1_RUN_VOLT].address;

  if ((newVoltage >= PMIC_VDD_SOC_0V900) && (newVoltage <= PMIC_VDD_SOC_1V100))
  {
    Banks[SW1_RUN_VOLT].value = newVoltage;

    /* send */
    i2c_config.data      = &Banks[SW1_RUN_VOLT].value;
    i2c_config.direction = kLPI2C_Write;
    status               = LPI2C_MasterTransferBlocking(LPI2C6, &i2c_config);
  }
  else
  {
    status = kStatus_Fail;
  }
  return status;
}

/*!
 * \brief Reads the core voltage from the PMIC.
 * This function reads the core voltage level from the PMIC by accessing a
 * specific register over I2C. The voltage is calculated based on the register
 * value.
 * \param[out] voltage Pointer to a float where the core voltage will be stored.
 * \return Status of the I2C transfer. Refer to the LPI2C_MasterTransferBlocking
 * documentation for potential error codes.
 * \note Assumes that the I2C master is properly configured before this function
 * is called.
 */
status_t PMIC_readCoreVoltage(float *voltage)
{
  status_t                status;
  uint8_t                 registerValue;
  lpi2c_master_transfer_t i2c_config;
  /* Set transfer parameters*/
  i2c_config.flags          = kLPI2C_TransferDefaultFlag;
  i2c_config.slaveAddress   = PMIC_DEVICE_ADDRESS;
  i2c_config.dataSize       = PMIC_REG_DATA_SIZE;
  i2c_config.subaddressSize = PMIC_REG_ADDRESS_SIZE;
  i2c_config.subaddress     = Banks[SW1_RUN_VOLT].address;
  i2c_config.data           = &registerValue;
  i2c_config.direction      = kLPI2C_Read;
  /* read */
  status   = LPI2C_MasterTransferBlocking(LPI2C6, &i2c_config);
  *voltage = (float) (0.4 + ((double) registerValue * 0.00625));
  return status;
}

/*!
 * \brief Reads the device ID from the PMIC.
 * This function reads the device ID of the PMIC by sending a read command to
 * the I2C address reserved for the device ID register.
 * \param[out] deviceId Pointer to a uint8_t where the device ID will be stored.
 * \return Status of the I2C transfer. Refer to the LPI2C_MasterTransferBlocking
 * documentation for potential error codes.
 * \note Assumes that the I2C master is properly configured before this function
 * is called.
 */
status_t PMIC_readDeviceID(uint8_t *deviceId)
{
  status_t                status;
  lpi2c_master_transfer_t i2c_config;
  /* Set transfer parameters*/
  i2c_config.flags          = kLPI2C_TransferDefaultFlag;
  i2c_config.slaveAddress   = PMIC_DEVICE_ADDRESS;
  i2c_config.data           = deviceId;
  i2c_config.dataSize       = PMIC_REG_DATA_SIZE;
  i2c_config.subaddressSize = PMIC_REG_ADDRESS_SIZE;
  i2c_config.subaddress     = 0x0;
  i2c_config.direction      = kLPI2C_Read;
  /* read */
  status = LPI2C_MasterTransferBlocking(LPI2C6, &i2c_config);

  return status;
}

/*!@}*/
