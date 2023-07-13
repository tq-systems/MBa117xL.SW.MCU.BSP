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

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "demo_eeprom.h"
#include "M24C64.h"
#include "board.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#ifdef USE_LPI2C
#include "lpi2c_api.h"
#else
#include "i2c_api.h"
#endif
#include "pin_mux.h"
#include "TQ_utils.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define DATA_BUFFER_SIZE 256

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static uint32_t M24C64_transferFunction(
  void *peripheral, uint8_t slaveAddr, uint16_t regAddress,
  size_t regAddressSize, uint8_t *buffer, uint16_t dataSize,
  M24C64_TransferDirection_t transferDirection);
static void M24C64_delayFunction(uint32_t us);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*configure peripheral */
static LPI2C_masterDevice_t eeprom_peripheral = {
  .base_addr = LPI2C_BASE_ADDR,
};

/* configure Transfer */
static M24C64_Handle_t handle = {.transferFunction = M24C64_transferFunction,
                                 .delayFunction    = M24C64_delayFunction,
                                 .slaveAddress     = EEPROM_MAIN_ADDRESS,
                                 .peripheral       = &eeprom_peripheral};

static char dataBuffer[DATA_BUFFER_SIZE];

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * \brief  This is the main application entry point where system is
 * initialized and all system components are started.
 */
int main(void)
{
  char    *ident            = BOARD_NAME;
  size_t   dataBufferSize   = 0;
  uint16_t operationAddress = 0x0;
  uint32_t status;
  BOARD_Initialize();
  /* Initialize the I2C master */
#ifdef USE_LPI2C
  LPI2C_MasterGetDefaultConfig(&eeprom_peripheral.masterConfig);
  eeprom_peripheral.masterConfig.baudRate_Hz = I2C_BITRATE;
  LPI2C_MasterInit(LPI2C_BASE_ADDR, &eeprom_peripheral.masterConfig,
                   LPI2C_MASTER_CLK_FREQ);
#else
  I2C_MasterGetDefaultConfig(&eeprom_peripheral.masterConfig);
  eeprom_peripheral.masterConfig.baudRate_Bps = I2C_BITRATE;
  I2C_MasterInit(eeprom_peripherall.base_addr, &eeprom_peripheral.masterConfig,
                 I2C_MASTER_CLK_FREQ);
#endif

  PRINTF("This demo shows the operation of the eeprom M24C64[DF]. Press "
         "anything to start. \r\n");
  status = M24C64_writeIdPage((uint8_t *) ident, 32, 0, &handle);
  if (status != kStatus_Success)
  {
    PRINTF("ERROR STATUS: %u \r\n", status);
  }
  status = M24C64_readIdPage((uint8_t *) dataBuffer, 32, &handle);
  if (status != kStatus_Success)
  {
    PRINTF("ERROR STATUS: %u \r\n", status);
  }
  else
  {
    PRINTF("Content ID: %s \r\n", dataBuffer);
  }

  while (1)
  {
    status         = M24C64_read(operationAddress, (uint8_t *) dataBuffer,
                                 DATA_BUFFER_SIZE, &handle);
    dataBufferSize = 0;
    if (status != kStatus_Success)
    {
      PRINTF("ERROR STATUS: %u \r\n", status);
    }
    PRINTF("Reading starting from address: %u \r\n", operationAddress);
    PRINTF("Content: %s \r\n", dataBuffer);
    PRINTF("Select an address for r/w. Press enter if you want to "
           "skip. \r\n");
    while (!getline(dataBuffer, 3, &dataBufferSize))
    {
      PRINTF("Input Error. Please type in again.\r\n");
    };
    if (dataBufferSize != 0)
    {
      operationAddress = (uint16_t) atoi(dataBuffer);
    }
    PRINTF("Write something new into the EEPROM! Press enter if you want to "
           "skip. \r\n");
    while (!getline(dataBuffer, 256, &dataBufferSize))
    {
      PRINTF("Input Error. Please type in again.\r\n");
    };
    if (dataBufferSize != 0)
    {
      status = M24C64_write(operationAddress, (uint8_t *) dataBuffer,
                            (uint16_t) dataBufferSize, &handle);
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
static uint32_t M24C64_transferFunction(
  void *peripheral, uint8_t slaveAddr, uint16_t regAddress,
  size_t regAddressSize, uint8_t *buffer, uint16_t dataSize,
  M24C64_TransferDirection_t transferDirection)
{
  status_t status;
#ifdef USE_LPI2C
  LPI2C_masterDevice_t *master      = (LPI2C_masterDevice_t *) peripheral;
  master->masterXfer.slaveAddress   = slaveAddr;
  master->masterXfer.data           = buffer;
  master->masterXfer.dataSize       = dataSize;
  master->masterXfer.subaddressSize = regAddressSize;
  master->masterXfer.subaddress     = regAddress;
  switch (transferDirection)
  {
  case M24C64_READ:
    master->masterXfer.flags     = kLPI2C_TransferRepeatedStartFlag;
    master->masterXfer.direction = kLPI2C_Read;
    status =
      LPI2C_MasterTransferBlocking(master->base_addr, &master->masterXfer);
    break;
  case M24C64_WRITE:
    master->masterXfer.direction = kLPI2C_Write;
    master->masterXfer.flags     = kLPI2C_TransferDefaultFlag;
    status =
      LPI2C_MasterTransferBlocking(master->base_addr, &master->masterXfer);
    break;
  default:
    status = kStatus_Fail;
  }
#else
  I2C_masterDevice_t *master        = (I2C_masterDevice_t *) peripheral;
  master->masterXfer.slaveAddress   = slaveAddr;
  master->masterXfer.data           = buffer;
  master->masterXfer.dataSize       = dataSize;
  master->masterXfer.subaddressSize = regAddressSize;
  master->masterXfer.subaddress     = regAddress;
  switch (transferDirection)
  {
  case M24C64_READ:
    master->masterXfer.flags     = kI2C_TransferRepeatedStartFlag;
    master->masterXfer.direction = kI2C_Read;
    status = I2C_MasterTransferBlocking(master->base_addr, &master->masterXfer);
    break;
  case M24C64_WRITE:
    master->masterXfer.direction = kI2C_Write;
    master->masterXfer.flags     = kI2C_TransferDefaultFlag;
    status = I2C_MasterTransferBlocking(master->base_addr, &master->masterXfer);
    break;
  default:
    status = kStatus_Fail;
  }
#endif

  return (uint32_t) status;
}

/*!
 * \brief This is the delay function that is used by the driver.
 * \note Parameters are described in the drivers function.
 */
static void M24C64_delayFunction(uint32_t us)
{
  SDK_DelayAtLeastUs(us, SystemCoreClock);
}
