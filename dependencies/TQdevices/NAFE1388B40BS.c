//*****************************************************************************
/*!
 * \copyright
 * SPDX-License-Identifier: BSD-3-Clause
 * \copyright
 * Copyright (c) 2021 - 2023 TQ-Systems GmbH <license@tq-group.com>,
 * D-82229 Seefeld, Germany.
 * Author: Isaac L. L. Yuki
 */
//*****************************************************************************

/*!
 * \addtogroup NAFE1388B40BS
 * @{
 * \file
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "NAFE1388B40BS.h"
#include "NAFE1388B40BS_REGISTERS.h"
#include <math.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define NAFE1388B40BS_INTERNAL_BUFFER_SIZE (5U)
#define NAFE1388B40BS_MAX_CHANNEL_COUNT (8U)
#define NAFE1388B40BS_GPIO_HIGH (1U)
#define NAFE1388B40BS_ERROR_IMPROPER_CMD_CH (200U)
#define NAFE1388B40BS_ERROR_IMPROPER_REG_SIZE (201U)

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*!
 \brief Gain Values for channel:
  0\h = 0.2x, 1\h = 0.4x, 2\h = 0.8x, 3\h =
  1x, 4\h = 2x, 5\h = 4x, 6\h = 8x, 7\h = 16x.
*/
static const double gain[8] = {0.2, 0.4, 0.8, 1, 2, 4, 8, 16};

static float NAFE1388B40BS_convertRaw2Voltage(uint32_t data, double gain);

static float NAFE1388B40BS_DieTemp(uint32_t data);

static uint8_t NAFE1388B40BS_getGainfromConfig(
  NAFE1388B40BS_ChannelConfig_t *channelConfig);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * \brief               Reads a register of NAFE1388B40BS.
 * \param handle        Is the NAFE1388B40BS handle that is being referred to.
 * \param regAddr       Address of register which shall be read from.
 * \param data          Pointer to a variable where the received value shall be
 * stored.
 * \param channelBase   Channelbase 16- or 24-bit.
 * \return Status of
 * transfer.
 * - 0: Successful
 * - 201: Improper channel size.
 * - Other error codes depend on the implementation of the transfer
 * function.
 */
uint32_t NAFE1388B40BS_readRegister(NAFE1388B40BS_Handle_t *handle,
                                    uint16_t regAddr, uint32_t *data,
                                    NAFE1388B40BS_ChannelBase_t channelBase)
{
  uint8_t
    txBuffer[NAFE1388B40BS_INTERNAL_BUFFER_SIZE]; /**< Transmit buffer used for
                                                     SPI communication.*/
  uint8_t
    rxBuffer[NAFE1388B40BS_INTERNAL_BUFFER_SIZE]; /**< Receive buffer used for
                                                     SPI communication.*/
  uint8_t  dataSize;
  uint32_t status   = 0x01;
  uint16_t protocol = 0x04000;
  protocol |= (regAddr << 1);
  txBuffer[0] = (uint8_t) (protocol >> 8);
  txBuffer[1] = (uint8_t) protocol;
  switch (channelBase)
  {
  case NAFE1388B40BS_CHANNEL_BASE_16_BIT:
    dataSize = 4;
    break;
  case NAFE1388B40BS_CHANNEL_BASE_24_BIT:
    dataSize = 5;
    break;
  default:
    return NAFE1388B40BS_ERROR_IMPROPER_REG_SIZE;
  }
  status =
    handle->transferFunction(handle->peripheral, txBuffer, rxBuffer, dataSize);
  switch (channelBase)
  {
  case NAFE1388B40BS_CHANNEL_BASE_24_BIT:
    *data = (uint32_t) (rxBuffer[2] << 16);
    *data |= (uint32_t) (rxBuffer[3] << 8);
    *data |= (uint32_t) (rxBuffer[4]);
    break;
  case NAFE1388B40BS_CHANNEL_BASE_16_BIT:
    *data = (uint32_t) (rxBuffer[2] << 8);
    *data |= (uint32_t) (rxBuffer[3]);
    break;
  default:
    return NAFE1388B40BS_ERROR_IMPROPER_REG_SIZE;
  }
  return status;
}

/*!
 * \brief Reads the value from selected data register and converts it into a
 * voltage value.
 * \param[in] handle        Is the NAFE1388B40BS handle that is being
 * referred to.
 * \param[in] regAddr       Address of register which shall be read
 * from.
 * \param[out] result       Pointer to the variable where the voltage value
 * shall be stored.
 * \return Status of transfer.
 * - 0: Successful
 * - 200: Improper channelCMDAddress
 * - Other error codes depend on the implementation of the transfer
 * function.
 * \note After configuration you can operate the analog front end in single
 * shot mode by setting the channel via "NAFE1388B40BS_command( &device, channel
 * )" and then calling "NAFE1388B40BS_command( &device, NAFE1388B40BS_CMD_SS )"
 * or you can activate continuous reading in multichannel mode so you don't have
 * always to call this operations. For this you must enable the channels you
 * want to read using "NAFE1388B40BS_configureChannelEnable".
 */
uint32_t NAFE1388B40BS_readVoltage(NAFE1388B40BS_Handle_t *handle,
                                   uint16_t regAddr, float *result)
{
  uint32_t status;
  uint32_t data;
  if (regAddr < NAFE1388B40BS_CH_DATA0 || regAddr > NAFE1388B40BS_CH_DATAF)
  {
    return NAFE1388B40BS_ERROR_IMPROPER_CMD_CH;
  }
  status = NAFE1388B40BS_readRegister(handle, regAddr, &data,
                                      NAFE1388B40BS_CHANNEL_BASE_24_BIT);
  if (status != 0)
  {
    return status;
  }
  *result = NAFE1388B40BS_convertRaw2Voltage(
    data,
    gain[handle->channelConfig[regAddr - NAFE1388B40BS_CH_DATA0].gainCoeff]);
  return status;
}

/*!
 * \brief               Writes to a register of NAFE1388B40BS.
 * \param handle        Is the NAFE1388B40BS handle that is being referred to.
 * \param regAddr       Address of register which shall be written to.
 * \param data          Data which shall be transferred.
 * \param channelBase   Channelbase 16- or 24-bit.
 * \return Status of transfer.
 * - 0: Successful
 * - 201: Improper channel size.
 * - Other error codes depend on the implementation of the transfer
 * function.
 */
uint32_t NAFE1388B40BS_writeRegister(NAFE1388B40BS_Handle_t *handle,
                                     uint16_t regAddr, uint32_t data,
                                     NAFE1388B40BS_ChannelBase_t channelBase)
{
  uint32_t status;
  uint8_t  dataSize;
  uint8_t
    txBuffer[NAFE1388B40BS_INTERNAL_BUFFER_SIZE]; /**< Transmit buffer used for
                                                     SPI communication.*/
  uint8_t
    rxBuffer[NAFE1388B40BS_INTERNAL_BUFFER_SIZE]; /**< Receive buffer used for
                                                     SPI communication.*/
  uint16_t protocol = 0x00000;
  protocol |= (regAddr << 1);
  txBuffer[0] = (uint8_t) (protocol >> 8);
  txBuffer[1] = (uint8_t) protocol;
  switch (channelBase)
  {
  case NAFE1388B40BS_CHANNEL_BASE_16_BIT:
    txBuffer[2] = (uint8_t) (data >> 8);
    txBuffer[3] = (uint8_t) data;
    dataSize    = 4;
    break;
  case NAFE1388B40BS_CHANNEL_BASE_24_BIT:
    txBuffer[2] = (uint8_t) (data >> 16);
    txBuffer[3] = (uint8_t) (data >> 8);
    txBuffer[4] = (uint8_t) data;
    dataSize    = 5;
    break;
  default:
    return NAFE1388B40BS_ERROR_IMPROPER_REG_SIZE;
  }
  status =
    handle->transferFunction(handle->peripheral, txBuffer, rxBuffer, dataSize);
  return status;
}

/*!
 * \brief Enables the NAFE1388B40BS device and waits until it's ready.
 * This function performs the necessary initialization steps to enable the
 * NAFE1388B40BS device. It first invokes the reset function (if provided)
 * to reset the device. After a delay for the crystal startup time, it
 * continuously reads the STATUS0 register until the chip indicates it's ready.
 * \param handle Pointer to the NAFE1388B40BS device handle.
 */
void NAFE1388B40BS_enable(NAFE1388B40BS_Handle_t *handle)
{
  uint16_t reg_read_value = 0;
  uint32_t data;
  if (handle->resetFunction != NULL)
  {
    handle->resetFunction();
    // Crystal Start Up Time 15ms
    if (handle->delayFunction != NULL)
    {
      handle->delayFunction(15000);
    }
  }
  /*Read of STATUS0 until Chip is ready*/
  while (reg_read_value != NAFE1388B40BS_SYS_STATUS0_CHIP_READY_MASK)
  {
    NAFE1388B40BS_readRegister(handle, NAFE1388B40BS_SYS_STATUS0, &data,
                               NAFE1388B40BS_CHANNEL_BASE_16_BIT);
    reg_read_value = (uint16_t) data;
    /*Masking Chip ready bit*/
    reg_read_value &= NAFE1388B40BS_SYS_STATUS0_CHIP_READY_MASK;
  }
}

/*!
 * \brief Configures CH_CONFIG0, CH_CONFIG1, CH_CONFIG2, CH_CONFIG3.
 * \param handle        Is the NAFE1388B40BS handle that is being referred to.
 * \param channelCMDAddress Addressed Channel to be configured.
 * \return Status of transfer.
 * - 0: Successful
 * - 200: Improper channelCMDAddress
 * - Other error codes depend on the implementation of the transfer
 * function.
 */
uint32_t NAFE1388B40BS_configureChannel(NAFE1388B40BS_Handle_t *handle,
                                        uint16_t channelCMDAddress)
{
  uint32_t status;
  if (channelCMDAddress > NAFE1388B40BS_CMD_CH15)
  {
    return NAFE1388B40BS_ERROR_IMPROPER_CMD_CH;
  }
  status = NAFE1388B40BS_command(handle, channelCMDAddress);
  if (status != 0)
  {
    return status;
  }
  handle->channelConfig->gainCoeff =
    NAFE1388B40BS_getGainfromConfig(handle->channelConfig);

  status = NAFE1388B40BS_writeRegister(
    handle, NAFE1388B40BS_CH_CONFIG0,
    handle->channelConfig[channelCMDAddress - NAFE1388B40BS_CMD_CH0]
      .channelConfigReg0,
    NAFE1388B40BS_CHANNEL_BASE_16_BIT);
  if (status != 0)
  {
    return status;
  }
  status = NAFE1388B40BS_writeRegister(
    handle, NAFE1388B40BS_CH_CONFIG1,
    handle->channelConfig[channelCMDAddress - NAFE1388B40BS_CMD_CH0]
      .channelConfigReg1,
    NAFE1388B40BS_CHANNEL_BASE_16_BIT);
  if (status != 0)
  {
    return status;
  }
  status = NAFE1388B40BS_writeRegister(
    handle, NAFE1388B40BS_CH_CONFIG2,
    handle->channelConfig[channelCMDAddress - NAFE1388B40BS_CMD_CH0]
      .channelConfigReg2,
    NAFE1388B40BS_CHANNEL_BASE_16_BIT);
  if (status != 0)
  {
    return status;
  }
  status = NAFE1388B40BS_writeRegister(
    handle, NAFE1388B40BS_CH_CONFIG3,
    handle->channelConfig[channelCMDAddress - NAFE1388B40BS_CMD_CH0]
      .channelConfigReg3,
    NAFE1388B40BS_CHANNEL_BASE_16_BIT);
  return status;
}

/*!
 * \brief Configures multichannel enable out of the set configuration in the
 *  NAFE1388B40BS_Handle_t structure.
 * \param handle NAFE1388B40BS_Handle_t handle structure
 * \note This function is only needed if you want to activate one or more
 * channels in multichannel reading mode. Make sure to set "channelConfigReg4"
 * properly in your handle structure.
 * \return Status of transfer.
 * - 0: Successful
 * - Other error codes depend on the implementation of the transfer
 * function.
 */
uint32_t NAFE1388B40BS_configureChannelEnable(NAFE1388B40BS_Handle_t *handle)
{
  return NAFE1388B40BS_writeRegister(handle, NAFE1388B40BS_CH_CONFIG4,
                                     handle->channelConfigReg4 & 0xFF,
                                     NAFE1388B40BS_CHANNEL_BASE_16_BIT);
}

/*!
 * \brief Configures all CH_CONFIGS for all channels following the datasheet.
 * \param handle Is the NAFE1388B40BS handle that is being referred to.
 * \return Status of transfer.
 * - 0: Successful
 * - Other error codes depend on the implementation of the transfer
 * function.
 */
uint32_t NAFE1388B40BS_configure(NAFE1388B40BS_Handle_t *handle)
{
  uint32_t status;
  status = NAFE1388B40BS_configureChannelEnable(handle);
  if (status != 0)
  {
    return status;
  }
  for (uint8_t i = 0; i < 16; i++)
  {
    status = NAFE1388B40BS_configureChannel(handle, NAFE1388B40BS_CMD_CH0 + i);
    if (status != 0)
    {
      return status;
    }
  }
  return status;
}

/*!
 * \brief Sets all possible configuration to default values described in the
 * datasheet.
 * \param handle Is the NAFE1388B40BS handle that is being referred to.
 */
void NAFE1388B40BS_getDefaultConfiguration(NAFE1388B40BS_Handle_t *handle)
{
  handle->channelConfigReg4 = 0x0;
  for (uint8_t i = 0; i < NAFE1388B40BS_MAX_CHANNEL_COUNT; i++)
  {
    handle->channelConfig[i].channelConfigReg0 = 0x0;
    handle->channelConfig[i].channelConfigReg1 = 0x0;
    handle->channelConfig[i].channelConfigReg2 = 0x0;
    handle->channelConfig[i].channelConfigReg3 = 0x0;
    handle->channelConfig[i].channelConfigReg5 = 0x0;
    handle->channelConfig[i].channelConfigReg6 = 0x0;
    handle->channelConfig[i].gainCoeff         = NAFE1388B40BS_0_2_GAIN;
  }
  handle->globalConfig.GLOBAL_ALARM_ENABLE_Reg = 0x0;
  handle->globalConfig.SYS_CONFIG_Reg          = 0x0;
}

/*!
 * \brief Sends a command to the NAFE1388B40BS device.
 * \param handle Is the NAFE1388B40BS handle that is being referred to.
 * \param cmd_value The command value to be sent.
 * \return Status of transfer.
 * - 0: Successful
 * - Other error codes depend on the implementation of the transfer
 * function.
 */
uint32_t NAFE1388B40BS_command(NAFE1388B40BS_Handle_t  *handle,
                               NAFE1388B40BS_Commands_t cmd_value)
{
  uint32_t status;
  uint8_t
    txBuffer[NAFE1388B40BS_INTERNAL_BUFFER_SIZE]; /**< Transmit buffer used for
                                                     SPI communication.*/
  uint8_t
    rxBuffer[NAFE1388B40BS_INTERNAL_BUFFER_SIZE]; /**< Receive buffer used for
                                                     SPI communication.*/
  uint16_t command = (uint16_t) cmd_value;
  txBuffer[0]      = (uint8_t) ((command >> 8) << 1);
  txBuffer[1]      = (uint8_t) (command << 1);
  status = handle->transferFunction(handle->peripheral, txBuffer, rxBuffer, 2);
  return status;
}

/*!
 * \brief Reads the temperature from the NAFE1388B40BS device.
 * \param handle Is the NAFE1388B40BS handle that is being referred to.
 * \param[out] result Pointer to store the read temperature value.
 * \return Status of transfer.
 * - 0: Successful
 * - Other error codes depend on the implementation of the transfer
 * function.
 */
uint32_t NAFE1388B40BS_readTemperature(NAFE1388B40BS_Handle_t *handle,
                                       float                  *result)
{
  uint32_t status;
  uint32_t data;
  status = NAFE1388B40BS_readRegister(handle, NAFE1388B40BS_DIE_TEMP, &data,
                                      NAFE1388B40BS_CHANNEL_BASE_16_BIT);
  if (status != 0)
  {
    return status;
  }
  *result = NAFE1388B40BS_DieTemp(data);
  return status;
}

/*!
 * \brief Converts raw data to voltage.
 * \param data The raw data to be converted.
 * \param setGain The gain setting for the conversion.
 * \return The converted voltage value.
 */
static float NAFE1388B40BS_convertRaw2Voltage(uint32_t data, double setGain)
{
  double voltage;
  // Check Sign
  if ((data & 0x800000) == 0)
  {
    voltage = ((double) (data) * 10) / (pow(2, 24) * setGain);
  }
  else
  {
    voltage = 0;
  }
  return (float) voltage;
}

/*!
 * \brief Converts raw data to die temperature.
 * \param[in] data The raw data to be converted.
 * \return The converted temperature value.
 */
static float NAFE1388B40BS_DieTemp(uint32_t data)
{
  float temperature;
  // Check Sign
  if ((data & 0x8000) == 0)
  {
    temperature = (float) data / 64;
  }
  else
  {
    data = (~data);
    data += 1;
    temperature = (float) data / 64;
  }
  return temperature;
}

/*!
 * \brief Selection of channel gain coefficient out from channelConfig0.
 * \param[in] channelConfig Pointer to the channel being refered to.
 * \return gainSelection 0-7
 */
static uint8_t NAFE1388B40BS_getGainfromConfig(
  NAFE1388B40BS_ChannelConfig_t *channelConfig)
{
  uint8_t gainSelection = 0;
  gainSelection         = (uint8_t) channelConfig->channelConfigReg0 >> 5;
  gainSelection &= 0xF;
  return gainSelection;
}

/*!@}*/
