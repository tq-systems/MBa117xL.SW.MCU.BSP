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
#include "demo_afe.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"
#include "NAFE1388B40BS.h"
#include "NAFE1388B40BS_REGISTERS.h"
#include "pin_mux.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void     AFE_InitChannels(void);
static uint32_t AFE_TransferFunction(void *peripheral, uint8_t *txBuffer,
                                     uint8_t *rxBuffer, uint8_t dataSize);
static void     AFE_resetFunction(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static LPSPI_device_t spi_peripheral = {
  .baseAddr                                   = SPI_BASE_ADDR,
  .transferMode                               = LPSPI_BlockingMode,
  .masterConfig.baudRate                      = SPI_TRANSFER_BAUDRATE,
  .masterConfig.bitsPerFrame                  = 8,
  .masterConfig.cpol                          = kLPSPI_ClockPolarityActiveHigh,
  .masterConfig.cpha                          = kLPSPI_ClockPhaseSecondEdge,
  .masterConfig.direction                     = kLPSPI_MsbFirst,
  .masterConfig.whichPcs                      = kLPSPI_Pcs0,
  .masterConfig.lastSckToPcsDelayInNanoSec    = 10UL,
  .masterConfig.pcsToSckDelayInNanoSec        = 10UL,
  .masterConfig.betweenTransferDelayInNanoSec = 100UL,
  .masterConfig.pinCfg                        = kLPSPI_SdiInSdoOut,
  .masterConfig.dataOutConfig                 = kLpspiDataOutRetained,

};

static NAFE1388B40BS_Handle_t device = {.peripheral = &spi_peripheral,
                                        .transferFunction =
                                          AFE_TransferFunction,
                                        .resetFunction = AFE_resetFunction};

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * \brief  This is the main application entry point where system is
 * initialized and all system components are started.
 */
int main(void)
{
  float    temperature;
  float    voltage;
  uint32_t AFE_data;

  BOARD_Initialize();

  PRINTF("AFE demonstration example. Press any button.\r\n\n");
  GETCHAR();
  /*Setting SPI*/
  LPSPI_MasterInit(spi_peripheral.baseAddr, &spi_peripheral.masterConfig,
                   SPI_CLOCK_FREQUENCY);

  NAFE1388B40BS_getDefaultConfiguration(&device);
  NAFE1388B40BS_enable(&device);

  PRINTF("[ OK ]\tAFE13388 Chip Ready\r\n");

  /*CHECK 16-Bit REGISTERS operation*/
  NAFE1388B40BS_readRegister(&device, NAFE1388B40BS_PARTNAME_MSB, &AFE_data,
                             NAFE1388B40BS_CHANNEL_BASE_16_BIT);
  PRINTF(" Partname: %0x", AFE_data);
  NAFE1388B40BS_readRegister(&device, NAFE1388B40BS_PARTNAME_LSB, &AFE_data,
                             NAFE1388B40BS_CHANNEL_BASE_16_BIT);
  PRINTF(" %0x \r\n", AFE_data);

  /*CHECK 24-Bit REGISTERS operation*/
  NAFE1388B40BS_readRegister(&device, NAFE1388B40BS_SERIAL_MSB, &AFE_data,
                             NAFE1388B40BS_CHANNEL_BASE_24_BIT);
  PRINTF(" SE_NR: %0x", AFE_data);
  NAFE1388B40BS_readRegister(&device, NAFE1388B40BS_SERIAL_LSB, &AFE_data,
                             NAFE1388B40BS_CHANNEL_BASE_24_BIT);
  PRINTF(" %0x \r\n", AFE_data);

  /*configure Channels (not part of driver)*/
  AFE_InitChannels();

  NAFE1388B40BS_configure(&device);

  PRINTF("Press any Button to initialize measurements.\r\n");
  GETCHAR();
  while (1)
  {
    for (uint8_t i = 0; i < 4; i++)
    {
      /*select Channel*/
      NAFE1388B40BS_command(&device, NAFE1388B40BS_CMD_CH0 + i);
      /*set operation type of channels via commands*/
      NAFE1388B40BS_command(&device, NAFE1388B40BS_CMD_SS);
      uint16_t status0_regValue = 0xFFFF;
      while (
        (status0_regValue & NAFE1388B40BS_SYS_STATUS0_SINGLE_CH_ACTIVE_MASK)
        != 0x0)
      {
        NAFE1388B40BS_readRegister(&device, NAFE1388B40BS_SYS_STATUS0,
                                   &AFE_data,
                                   NAFE1388B40BS_CHANNEL_BASE_16_BIT);
        status0_regValue = (uint16_t) AFE_data;
      }
      NAFE1388B40BS_readVoltage(&device, NAFE1388B40BS_CH_DATA0 + i, &voltage);
      PRINTF("\r\nCHANNEL%u %.2fV \r\n", i, voltage);
    }
    NAFE1388B40BS_readTemperature(&device, &temperature);
    PRINTF("\r\nTemperature: %.1f degree Celsius\r\n", temperature);
    /* slow down reading*/
    SDK_DelayAtLeastUs(1000000, SystemCoreClock);
  }
}

/*!
 * \brief Function to initialize NAFE1388B40BS with board specific settings.
 * \note This function is not part of the API and only for demonstrational
 * purpose.
 */
static void AFE_InitChannels(void)
{
  //------------------------------------------------------------------------------------------------------
  // Channel 0
  //------------------------------------------------------------------------------------------------------
  /*
   * Channel Configuration
   *    			Type				 Gain			Nominal Input Range
   *    AI1P:    Unipolar SE			0.2V/V				+10V
   */
  device.channelConfig[0].channelConfigReg0 =
    NAFE1388B40BS_CH_CONFIG0_HV_AIP(1) | NAFE1388B40BS_CH_CONFIG0_HV_AIN(0)
    | NAFE1388B40BS_CH_CONFIG0_CH_GAIN(0) | NAFE1388B40BS_CH_CONFIG0_LVSIG_IN(0)
    | NAFE1388B40BS_CH_CONFIG0_HV_SEL(1);

  device.channelConfig[0].channelConfigReg1 = 0x38; // DATA_RATE 7
  device.channelConfig[0].channelConfigReg2 = 0x00; // Default
  device.channelConfig[0].channelConfigReg3 = 0x0;  // Default

  //------------------------------------------------------------------------------------------------------
  // Channel 1
  //------------------------------------------------------------------------------------------------------
  /*
   * Channel Configuration
   *    Type				 Gain			Nominal Input     Range
   *    AI1N:    Unipolar SE			0.2V/V				+10V
   */
  device.channelConfig[1].channelConfigReg0 =
    NAFE1388B40BS_CH_CONFIG0_HV_AIP(0) | NAFE1388B40BS_CH_CONFIG0_HV_AIN(1)
    | NAFE1388B40BS_CH_CONFIG0_CH_GAIN(0) | NAFE1388B40BS_CH_CONFIG0_LVSIG_IN(0)
    | NAFE1388B40BS_CH_CONFIG0_HV_SEL(1);

  device.channelConfig[1].channelConfigReg1 = 0x38; // DATA_RATE 7
  device.channelConfig[1].channelConfigReg2 = 0x0;  // Default
  device.channelConfig[1].channelConfigReg3 = 0x0;  // Default
  //------------------------------------------------------------------------------------------------------
  // Channel 2
  //------------------------------------------------------------------------------------------------------
  /*
   * Channel Configuration
   *    Type				 Gain			Nominal Input     Range
   *    AI2P:    Unipolar SE			0.2V/V				+10V
   */
  device.channelConfig[2].channelConfigReg0 =
    NAFE1388B40BS_CH_CONFIG0_HV_AIP(2) | NAFE1388B40BS_CH_CONFIG0_HV_AIN(0)
    | NAFE1388B40BS_CH_CONFIG0_CH_GAIN(0) | NAFE1388B40BS_CH_CONFIG0_LVSIG_IN(0)
    | NAFE1388B40BS_CH_CONFIG0_HV_SEL(1);

  device.channelConfig[2].channelConfigReg1 = 0x38; // DATA_RATE 7
  device.channelConfig[2].channelConfigReg2 = 0x0;  // Default
  device.channelConfig[2].channelConfigReg3 = 0x0;  // Default
  //------------------------------------------------------------------------------------------------------
  // Channel 3
  //------------------------------------------------------------------------------------------------------
  /*
   * Channel Configuration
   *    Type				 Gain			Nominal Input     Range
   *    AI2N:    Unipolar SE			0.2V/V				+10V
   */
  device.channelConfig[3].channelConfigReg0 =
    NAFE1388B40BS_CH_CONFIG0_HV_AIP(0) | NAFE1388B40BS_CH_CONFIG0_HV_AIN(2)
    | NAFE1388B40BS_CH_CONFIG0_CH_GAIN(0) | NAFE1388B40BS_CH_CONFIG0_LVSIG_IN(0)
    | NAFE1388B40BS_CH_CONFIG0_HV_SEL(1);

  device.channelConfig[3].channelConfigReg1 = 0x38; // DATA_RATE 7
  device.channelConfig[3].channelConfigReg2 = 0x0;  // Default
  device.channelConfig[3].channelConfigReg3 = 0x0;  // Default
  //------------------------------------------------------------------------------------------------------
  // Channel 4
  //------------------------------------------------------------------------------------------------------
  /*
   * Channel Configuration
   *    Type				 Gain			Nominal Input   Range
   *    AI3P:    Unipolar SE			1V/V				+2.0V
   */
  device.channelConfig[4].channelConfigReg0 =
    NAFE1388B40BS_CH_CONFIG0_HV_AIP(3) | NAFE1388B40BS_CH_CONFIG0_HV_AIN(0)
    | NAFE1388B40BS_CH_CONFIG0_CH_GAIN(3) | NAFE1388B40BS_CH_CONFIG0_LVSIG_IN(0)
    | NAFE1388B40BS_CH_CONFIG0_HV_SEL(1);

  device.channelConfig[4].channelConfigReg1 = 0x38; // DATA_RATE 7
  device.channelConfig[4].channelConfigReg2 = 0x0;  // Default
  device.channelConfig[4].channelConfigReg3 = 0x0;  // Default
}

/*!
 * \brief This is the transfer function that is used by the driver.
 * \note Parameters are described in the drivers function.
 */
static uint32_t AFE_TransferFunction(void *peripheral, uint8_t *txBuffer,
                                     uint8_t *rxBuffer, uint8_t dataSize)
{
  uint32_t        status;
  LPSPI_device_t *master = (LPSPI_device_t *) peripheral;
  /*spi_peripheral Transfer config*/
  master->masterXfer.txData      = txBuffer;
  master->masterXfer.rxData      = rxBuffer;
  master->masterXfer.configFlags = kLPSPI_MasterPcsContinuous;
  master->masterXfer.dataSize    = dataSize;
  status = (uint32_t) LPSPI_MasterTransferBlocking(master->baseAddr,
                                                   &master->masterXfer);
  return status;
}

/*!
 * \brief This is the board specific reset function for operating the device.
 * \note Parameters are described in the drivers function.
 */
static void AFE_resetFunction(void)
{
  /* Enable GPIO reset pin of AFE*/
  gpio_pin_config_t config = {
    .direction = kGPIO_DigitalOutput, .outputLogic = 1, .interruptMode = 0};
  GPIO_PinInit(NAFE1388B40BS_RST_GPIO, NAFE1388B40BS_RST_PIN, &config);
  GPIO_WritePinOutput(NAFE1388B40BS_RST_GPIO, NAFE1388B40BS_RST_PIN, 0);
  GPIO_WritePinOutput(NAFE1388B40BS_RST_GPIO, NAFE1388B40BS_RST_PIN, 1);
}

void AFE_delay(uint32_t us)
{
  SDK_DelayAtLeastUs(us, SystemCoreClock);
}
