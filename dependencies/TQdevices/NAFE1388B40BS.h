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

#ifndef _NAFE1388B40BS_H_
#define _NAFE1388B40BS_H_

/*!
 * \defgroup NAFE1388B40BS NAFE1388B40BS
 * \brief Driver for the NAFE1388B40BS analog front end.
 * @{
 * \file
 * \details To utilize this driver, start by initializing the peripheral
 * configurations specific to your MCU.
 * Then, declare the device you wish to use by employing the handle declared in
 * this file. Initialize all parameters within the handle, and ensure to define
 * and implement the required functions as specified in the handle. Detailed
 * instructions for each parameter are provided in the documentation. Prior to
 * using the driver, it's essential to initialize all peripherals that interface
 * with the target device.
 * Once initialization is complete, call the NAFE1388B40BS_Enable function.
 * Then, use NAFE1388B40BS_configure to configure all channels at once.
 * You can also configure only one specific channel by calling
 * "NAFE1388B40BS_configureChannelCoeff", "NAFE1388B40BS_configureChannel" and
 * "NAFE1388B40BS_configureChannelEnable".
 * After configuring you will be able to read voltage and temperature by calling
 * the proper functions.
 * \todo Add CRC operations.
 * \note CRC can not be used yet.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>
#include <stddef.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define NAFE1388B40BS_MAX_CHANNEL_COUNT (8U)

/*!
 * \brief Enumeration for channel base options.
 */
typedef enum NAFE1388B40BS_ChannelBase
{
  NAFE1388B40BS_CHANNEL_BASE_16_BIT,
  NAFE1388B40BS_CHANNEL_BASE_24_BIT
} NAFE1388B40BS_ChannelBase_t;

/*!
 * brief Enumeration for gain coefficient selection.
 */
typedef enum NAFE1388B40BS_GainCoeff
{
  NAFE1388B40BS_0_2_GAIN,
  NAFE1388B40BS_0_4_GAIN,
  NAFE1388B40BS_0_8_GAIN,
  NAFE1388B40BS_1_0_GAIN,
  NAFE1388B40BS_2_0_GAIN,
  NAFE1388B40BS_4_0_GAIN,
  NAFE1388B40BS_8_0_GAIN,
  NAFE1388B40BS_16_0_GAIN
} NAFE1388B40BS_GainCoeff_t;

/*!
 * \brief Structure for channel configuration settings.
 */
typedef struct NAFE1388B40BS_ChannelConfig
{
  uint16_t channelConfigReg0;
  uint16_t channelConfigReg1;
  uint16_t channelConfigReg2;
  uint16_t channelConfigReg3; /**< Under-range threshold setting for each
 logical data channel.*/
  uint32_t channelConfigReg5; /**< Over-range threshold setting for each logical
 data channel.*/
  uint32_t channelConfigReg6; /**< Under-range threshold setting for each
  logical data channel.*/
  NAFE1388B40BS_GainCoeff_t gainCoeff; /**< gain coefficient selection */
} NAFE1388B40BS_ChannelConfig_t;

/*!
 * \brief Enumeration for device commands.
 */
typedef enum NAFE1388B40BS_Commands
{
  NAFE1388B40BS_CMD_ABORT = 0x0010,
  NAFE1388B40BS_CMD_END,
  NAFE1388B40BS_CMD_CLEAR_ALARM,
  NAFE1388B40BS_CMD_CLEAR_DATA,
  NAFE1388B40BS_CMD_RESET,
  NAFE1388B40BS_CMD_CLEAR_REG,
  NAFE1388B40BS_CMD_RELOAD,
  NAFE1388B40BS_NON_RETURN_CMD_BORDER,
  NAFE1388B40BS_CMD_SS = 0x2000,
  NAFE1388B40BS_CMD_SC,
  NAFE1388B40BS_CMD_MS,
  NAFE1388B40BS_CMD_MM,
  NAFE1388B40BS_CMD_MC,
  NAFE1388B40BS_CMD_BURST_DATA = 0x2005,
  NAFE1388B40BS_CMD_CALC_CRC_CONFG,
  NAFE1388B40BS_CMD_CALC_CRC_COEF,
  NAFE1388B40BS_CMD_CALC_CRC_FAC,
} NAFE1388B40BS_Commands_t;

/*!
 * \brief Structure for configuration registers.
 */
typedef struct NAFE1388B40BS_ConfigRegisters
{
  uint32_t SYS_CONFIG_Reg;
  uint32_t GLOBAL_ALARM_ENABLE_Reg;
} NAFE1388B40BS_ConfigRegisters_t;

/*!
 * \brief Enumeration for CRC options.
 */
typedef enum NAFE1388B40BS_CrcOption
{
  NAFE1388B40BS_CRC_Off = 0x0,
  NAFE1388B40BS_CRC_On  = 0x1
} NAFE1388B40BS_CrcOption_t;

/*!
 * \brief Function pointer for the transfer function.
 * \param peripheral Pointer to the peripheral (e.g., SPI) associated with the
 * device.
 * \param txBuffer Pointer to the transmit buffer used for
 * communication.
 * \param rxBuffer Pointer to the receive buffer used for
 * communication.
 * \param dataSize Number of bytes to be transferred.
 * \return Status of transfer.
 * - 0: Successful
 * - Other error codes depend on the implementation of the transfer
 * function.
 */
typedef uint32_t (*NAFE1388B40BS_TransferFunction_t)(void    *peripheral,
                                                     uint8_t *txBuffer,
                                                     uint8_t *rxBuffer,
                                                     uint8_t  dataSize);

/*!
 * \brief Function pointer for the reset function.
 */
typedef void (*NAFE1388B40BS_ResetFunction_t)(void);

/*!
 * \brief Function pointer for the delay function.
 * \param us Delay duration in microseconds.
 */
typedef void (*NAFE1388B40BS_DelayFunction_t)(uint32_t us);

/*!
 * \brief Structure for the device handle.
 * \note All configuration related parameters can be set by calling the
 * "NAFE1388B40BS_getDefaultConfiguration" function.
 */
typedef struct NAFE1388B40BS_Handle
{
  void *
    peripheral; /**< Pointer to the SPI peripheral associated with the device.*/
  uint16_t
    channelConfigReg4; /**< Configuration register 4 for channel settings.*/
  NAFE1388B40BS_ChannelConfig_t
    channelConfig[NAFE1388B40BS_MAX_CHANNEL_COUNT]; /**< Array of channel
                                                       configurations.*/
  NAFE1388B40BS_ConfigRegisters_t
    globalConfig; /**< Configuration registers for the device.*/
  NAFE1388B40BS_TransferFunction_t
    transferFunction; /**< Pointer to the SPI transfer function.*/
  NAFE1388B40BS_DelayFunction_t
    delayFunction; /**< Pointer to the delay function.*/
  NAFE1388B40BS_ResetFunction_t
    resetFunction; /**< Pointer to the device reset function.*/
} NAFE1388B40BS_Handle_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* Basic transfer functions*/

extern uint32_t NAFE1388B40BS_writeRegister(
  NAFE1388B40BS_Handle_t *handle, uint16_t regAddr, uint32_t data,
  NAFE1388B40BS_ChannelBase_t channelBase);

extern uint32_t NAFE1388B40BS_readRegister(
  NAFE1388B40BS_Handle_t *handle, uint16_t regAddr, uint32_t *data,
  NAFE1388B40BS_ChannelBase_t channelBase);

extern uint32_t NAFE1388B40BS_command(NAFE1388B40BS_Handle_t  *handle,
                                      NAFE1388B40BS_Commands_t command);

/* Abstracted functions */

extern uint32_t NAFE1388B40BS_readVoltage(NAFE1388B40BS_Handle_t *handle,
                                          uint16_t regAddr, float *result);

extern uint32_t NAFE1388B40BS_readTemperature(NAFE1388B40BS_Handle_t *handle,
                                              float                  *result);

extern uint32_t NAFE1388B40BS_configure(NAFE1388B40BS_Handle_t *handle);

extern void NAFE1388B40BS_enable(NAFE1388B40BS_Handle_t *handle);

/* Configuration */

extern void NAFE1388B40BS_getDefaultConfiguration(
  NAFE1388B40BS_Handle_t *handle);

extern uint32_t NAFE1388B40BS_configureChannel(NAFE1388B40BS_Handle_t *handle,
                                               uint16_t channelCMDAddress);

extern uint32_t NAFE1388B40BS_configureChannelEnable(
  NAFE1388B40BS_Handle_t *handle);

/*!@}*/

#endif /* _NAFE1388B40BS_H_ */
