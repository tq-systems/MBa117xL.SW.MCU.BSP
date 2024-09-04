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

#include "board.h"
#include "PCA9555BS.h"
#ifdef USE_LPI2C
#include "lpi2c_api.h"
#else
#include "i2c_api.h"
#endif
#include "demo_digital_io.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"
#include "pin_mux.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void     execute_digital_loopback(void);
static uint32_t PCA9555BS_TransferFunction(
  void *master, uint32_t regAddress, size_t regAddressSize,
  PCA9555BS_TransferDirection_t transferDirection, uint8_t *buffer,
  size_t bufferSize);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Initialize peripheral */
#ifdef USE_LPI2C
static LPI2C_masterDevice_t i2c_master = {
  .base_addr                 = LPI2C_BASE_ADDR,
  .masterConfig.baudRate_Hz  = LPI2C_BITRATE,
  .mode                      = LPI2C_BlockingMode,
  .masterXfer.flags          = kLPI2C_TransferDefaultFlag,
  .masterXfer.slaveAddress   = PORTEXP_I2C_DEVICE_ADR,
  .masterXfer.subaddressSize = 1,
  .masterXfer.dataSize       = 2};
#else
static I2C_masterDevice_t i2c_master = {
  .base_addr                 = I2C_BASE_ADDR,
  .masterConfig.baudRate_Hz  = I2C_BITRATE,
  .mode                      = I2C_BlockingMode,
  .masterXfer.flags          = kI2C_TransferDefaultFlag,
  .masterXfer.slaveAddress   = PORTEXP_I2C_DEVICE_ADR,
  .masterXfer.subaddressSize = 1,
  .masterXfer.dataSize       = 2};
#endif

/* Device configuration */
static PCA9555BS_Handle_t portExpander = {.IOMask.PIN00_PIN07_Mask       = 0xF0,
                                          .IOMask.PIN10_PIN17_Mask       = 0x00,
                                          .PolarityMask.PIN00_PIN07_Mask = 0x00,
                                          .PolarityMask.PIN10_PIN17_Mask = 0x00,
                                          .transferFunction =
                                            PCA9555BS_TransferFunction,
                                          .peripheral = &i2c_master};

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * \brief  This is the main application entry point where system is initialized
 *         and all system components are started.
 */
int main(void)
{
  BOARD_Initialize();
  PRINTF("This demo is a demonstration of the portexpander working with "
         "digital IO Pins. Please Press a button to continue \r\n");
  GETCHAR();
/*Init peripheral*/
#ifdef USE_LPI2C
  LPI2C_MasterGetDefaultConfig(&i2c_master.masterConfig);
  i2c_master.masterConfig.baudRate_Hz = LPI2C_BITRATE;
  LPI2C_MasterInit(i2c_master.base_addr, &i2c_master.masterConfig,
                   LPI2C_CLOCK_FREQUENCY);
#else
  I2C_MasterGetDefaultConfig(&i2c_master.masterConfig);
  i2c_master.masterConfig.baudRate_Hz = I2C_BITRATE;
  I2C_MasterInit(i2c_master.base_addr, &i2c_master.masterConfig,
                 I2C_CLOCK_FREQUENCY);
#endif
  /*Configure PortExpander with the values with which the portExpander struct
   * was initialized*/
  PCA9555BS_configure(&portExpander);
  while (1)
  {
    execute_digital_loopback();
    PRINTF("Please press any button to execute the demonstration again. \r\n");
    GETCHAR();
  }
}

/*!
 * \brief This is the transfer function that is used by the driver.
 * \note Parameters are described in the drivers function.
 */
static uint32_t PCA9555BS_TransferFunction(
  void *peripheral, uint32_t regAddress, size_t regAddressSize,
  PCA9555BS_TransferDirection_t transferDirection, uint8_t *buffer,
  size_t bufferSize)
{
  uint32_t status;
#ifdef USE_LPI2C
  LPI2C_masterDevice_t *master      = (LPI2C_masterDevice_t *) peripheral;
  master->masterXfer.data           = buffer;
  master->masterXfer.dataSize       = bufferSize;
  master->masterXfer.subaddress     = regAddress;
  master->masterXfer.subaddressSize = regAddressSize;
  switch (transferDirection)
  {
  case PCA9555BS_WRITE:
    master->masterXfer.direction = kLPI2C_Write;
    status = (uint32_t) LPI2C_MasterTransferBlocking(master->base_addr,
                                                     &master->masterXfer);
    break;
  case PCA9555BS_READ:
    master->masterXfer.direction = kLPI2C_Read;
    status = (uint32_t) LPI2C_MasterTransferBlocking(master->base_addr,
                                                     &master->masterXfer);
    break;
  default:
    status = kStatus_Fail;
  }
#else
  I2C_masterDevice_t *master        = (I2C_masterDevice_t *) peripheral;
  master->masterXfer.data           = buffer;
  master->masterXfer.dataSize       = bufferSize;
  master->masterXfer.subaddress     = regAddress;
  master->masterXfer.subaddressSize = regAddressSize;
  switch (transferDirection)
  {
  case PCA9555BS_WRITE:
    master->masterXfer.direction = kLPI2C_Write;
    status = (uint32_t) LPI2C_MasterTransferBlocking(master->base_addr,
                                                     &master->masterXfer);
    break;
  case PCA9555BS_READ:
    master->masterXfer.direction = kLPI2C_Read;
    status = (uint32_t) LPI2C_MasterTransferBlocking(master->base_addr,
                                                     &master->masterXfer);
    break;
  default:
    status = kStatus_Fail;
  }
#endif
  return status;
}

/*!
 * \brief  Loopback example function for demonstration.
 */
static void execute_digital_loopback(void)
{
  uint8_t     expectedInputs[] = {0x01, 0x02, 0x04, 0x08};
  const char *ioNames[] = {"Digital IO 1", "Digital IO 2", "Digital IO 3",
                           "Digital IO 4"};

  for (uint8_t i = 0; i < 4; i++)
  {
    portExpander.OutputMask.PIN00_PIN07_Mask = expectedInputs[i];

    // Write the output setting to the port expander.
    PCA9555BS_setOutput(&portExpander);
    // Allow IO settling time after changing the output setting.
    SDK_DelayAtLeastUs(20000, SystemCoreClock);

    // Check the state of each digital input pin.
    if (GPIO_ReadPinInput(DIG_IN_1_GPIO, DIG_IN_1_GPIO_PIN)
          == ((portExpander.OutputMask.PIN00_PIN07_Mask & 0x01) >> 0)
        && GPIO_ReadPinInput(DIG_IN_1_GPIO, DIG_IN_2_GPIO_PIN)
             == ((portExpander.OutputMask.PIN00_PIN07_Mask & 0x02) >> 1)
        && GPIO_ReadPinInput(DIG_IN_1_GPIO, DIG_IN_3_GPIO_PIN)
             == ((portExpander.OutputMask.PIN00_PIN07_Mask & 0x04) >> 2)
        && GPIO_ReadPinInput(DIG_IN_1_GPIO, DIG_IN_4_GPIO_PIN)
             == ((portExpander.OutputMask.PIN00_PIN07_Mask & 0x08) >> 3))
    {
      PRINTF("[ OK ]\t%s Loopback\r\n", ioNames[i]);
    }
    else
    {
      PRINTF("[\033[1;31mERROR\033[0m] %s Loopback \r\n", ioNames[i]);
    }
  }
  PRINTF("Loopback ended. \r\n");
}
