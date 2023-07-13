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

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "demo_SE97BTP.h"
#include "SE97BTP.h"
#include "board.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#ifdef USE_LPI2C
#include "lpi2c_api.h"
#else
#error "Only LPI2C supported."
#endif
#include "pin_mux.h"
#include "TQ_utils.h"
#include <string.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define DATA_BUFFER_SIZE 255U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static uint32_t SE97BTP_transfer_TempSensor(
  void *peripheral, uint32_t regAddress, size_t regAddressSize, uint8_t *buffer,
  uint8_t dataSize, SE97BTP_TransferDirection_t transferDirection);
static uint32_t SE97BTP_transferFunction_EEPROM(
  void *peripheral, uint8_t slaveAddress, uint8_t regAddress,
  size_t regAddressSize, uint8_t dataSize, uint8_t *buffer,
  SE97BTP_TransferDirection_t transferDirection);
static void SE97BTP_delayFunction(uint32_t us);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static char dataBuffer[DATA_BUFFER_SIZE];

#ifdef USE_LPI2C
static LPI2C_masterDevice_t i2c_master_temperature = {
  .base_addr                 = LPI2C_BASE_ADDR,
  .masterXfer.slaveAddress   = S97BTP_TEMP_MAINBOARD,
  .masterXfer.flags          = kLPI2C_TransferDefaultFlag,
  .masterConfig.enableMaster = true,
};

static LPI2C_masterDevice_t i2c_master_eeprom = {
  .base_addr                 = LPI2C_BASE_ADDR,
  .masterConfig.enableMaster = true,
};

static SE97BTP_EEPROM_Handle_t transferEEPROM = {
  .transferFunction = SE97BTP_transferFunction_EEPROM,
  .delayFunction    = SE97BTP_delayFunction,
  .peripheral       = &i2c_master_eeprom,
  .slaveAddress     = S97BTP_EEPROM_MAINBOARD};

static SE97BTP_Sensor_Handle_t TempSensorMainboard = {
  .transfer   = SE97BTP_transfer_TempSensor,
  .peripheral = &i2c_master_temperature};

#else
#error "Only LPI2C supported."
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * \brief  This is the main application entry point where system is initialized
 *         and all system components are started.
 */
int main(void)
{
  float    temp;
  status_t status;
  uint8_t  address_EEPROM = 0x0;
  size_t   dataBufferSize = 0x0;
  BOARD_Initialize();

  PRINTF("Demonstration of the SE97BTP device. Press any button to "
         "initialize. \r\n");
  GETCHAR();
  /* Initialize the I2C master */
#ifdef USE_LPI2C
  LPI2C_MasterGetDefaultConfig(&i2c_master_temperature.masterConfig);
  i2c_master_temperature.masterConfig.baudRate_Hz = I2C_BITRATE;
  i2c_master_eeprom.masterConfig.baudRate_Hz      = I2C_BITRATE;
  LPI2C_MasterInit(LPI2C_BASE_ADDR, &i2c_master_temperature.masterConfig,
                   LPI2C_MASTER_CLK_FREQ);
#else
#error "Only LPI2C supported."
#endif

  while (1)
  {
    status = (status_t) SE97BTP_readTemp(&TempSensorMainboard, &temp);
    if (status != kStatus_Success)
    {
      PRINTF("ERROR STATUS: %u \r\n", status);
    }
    PRINTF("Temperature of Mainboard in degree Celsius: %f \r\n", temp);
    status =
      (status_t) SE97BTP_readEEPROM(&transferEEPROM, address_EEPROM,
                                    (uint8_t *) dataBuffer, DATA_BUFFER_SIZE);
    if (status != kStatus_Success)
    {
      PRINTF("ERROR STATUS: %u \r\n", status);
    }
    PRINTF("Reading starting from address: %u \r\n", address_EEPROM);
    PRINTF("Content: %s \r\n", dataBuffer);
    PRINTF("Select an address for r/w. \r\n");
    while (!getline(dataBuffer, 3, &dataBufferSize))
    {
      PRINTF("Input Error. Please type in again.\r\n");
    };
    if (dataBufferSize != 0)
    {
      {
        address_EEPROM = (uint8_t) atoi(dataBuffer);
      }
    }
    PRINTF("Write something new into the EEPROM! Press enter if you want to "
           "skip. \r\n");
    while (!getline(dataBuffer, DATA_BUFFER_SIZE, &dataBufferSize))
    {
      PRINTF("Input Error. Please type in again.\r\n");
    };
    if (dataBufferSize != 0)
    {
      status = (status_t) SE97BTP_writeEEPROM(&transferEEPROM, address_EEPROM,
                                              (uint8_t *) dataBuffer,
                                              (uint8_t) dataBufferSize);
      if (status != kStatus_Success)
      {
        PRINTF("ERROR STATUS: %u \r\n", status);
      }
    }
  }

  return 0;
}

/*!
 * \brief This is the transfer function that is used by the driver.
 * \note Parameters are described in the drivers function.
 */
static uint32_t SE97BTP_transfer_TempSensor(
  void *peripheral, uint32_t regAddress, size_t regAddressSize, uint8_t *buffer,
  uint8_t dataSize, SE97BTP_TransferDirection_t transferDirection)
{
  status_t status = kStatus_NoTransferInProgress;

#ifdef USE_LPI2C
  LPI2C_masterDevice_t *master      = (LPI2C_masterDevice_t *) peripheral;
  master->masterXfer.data           = buffer;
  master->masterXfer.dataSize       = dataSize;
  master->masterXfer.subaddress     = regAddress;
  master->masterXfer.subaddressSize = regAddressSize;
  if (SE97BTP_READ == transferDirection)
  {
    master->masterXfer.direction = kLPI2C_Read;
    status =
      LPI2C_MasterTransferBlocking(master->base_addr, &master->masterXfer);
  }
  else if (SE97BTP_WRITE == transferDirection)
  {
    master->masterXfer.direction = kLPI2C_Write;
    status =
      LPI2C_MasterTransferBlocking(master->base_addr, &master->masterXfer);
  }
#else

#error "Only LPI2C supported."

#endif

  return (uint32_t) status;
}

/*!
 * \brief This is the transfer function that is used by the driver.
 * \note Parameters are described in the drivers function.
 */
static uint32_t SE97BTP_transferFunction_EEPROM(
  void *peripheral, uint8_t slaveAddress, uint8_t regAddress,
  size_t regAddressSize, uint8_t dataSize, uint8_t *buffer,
  SE97BTP_TransferDirection_t transferDirection)
{
  status_t status = kStatus_NoTransferInProgress;
#ifdef USE_LPI2C
  LPI2C_masterDevice_t *master      = (LPI2C_masterDevice_t *) peripheral;
  master->masterXfer.slaveAddress   = slaveAddress;
  master->masterXfer.data           = buffer;
  master->masterXfer.subaddress     = regAddress;
  master->masterXfer.dataSize       = dataSize;
  master->masterXfer.subaddressSize = regAddressSize;
  switch (transferDirection)
  {
  case SE97BTP_READ:
    master->masterXfer.flags     = kLPI2C_TransferRepeatedStartFlag;
    master->masterXfer.direction = kLPI2C_Read;
    status =
      LPI2C_MasterTransferBlocking(master->base_addr, &master->masterXfer);
    break;
  case SE97BTP_WRITE:
    master->masterXfer.direction = kLPI2C_Write;
    master->masterXfer.flags     = kLPI2C_TransferDefaultFlag;
    status =
      LPI2C_MasterTransferBlocking(master->base_addr, &master->masterXfer);
    break;
  default:
    status = kStatus_Fail;
  }

#else

#error "Only LPI2C supported."

#endif

  return (uint32_t) status;
}

/*!
 * \brief This is the delay function that is used by the driver.
 * \note Parameters are described in the drivers function.
 */
static void SE97BTP_delayFunction(uint32_t us)
{
  SDK_DelayAtLeastUs(us, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
}
