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

#ifndef _SE97BTP_H_
#define _SE97BTP_H_

/*!
 * \defgroup SE97BTP SE97BTP
 * \brief Driver for the SE97BTP temperature sensor and EEPROM.
 * @{
 * \file
 * \details This driver abstracts the SE97BTP as a temperature sensor and an
 * EEPROM.
 * To utilize this driver, start by initializing the peripheral
 * configurations specific to your MCU.
 * Then, declare the device you wish to use by employing the appropriate handle.
 * Initialize all parameters within the handle, and ensure to define and
 * implement the required functions as specified in the handle.
 * Detailed instructions for each parameter are provided in the documentation.
 * Prior to using the driver, it's essential to initialize all peripherals that
 * interface with the target device.
 * Once initialization is complete, you are free to use any function provided in
 * this file to interact with the device.
 * \todo Add control register operation functions.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * \brief Enumeration of possible data transfer directions.
 */
typedef enum SE97BTP_TransferDirection
{
  SE97BTP_READ, /**< Indicates a read operation.*/
  SE97BTP_WRITE /**< Indicates a write operation.*/
} SE97BTP_TransferDirection_t;

/*!
 * \brief Enumeration of register addresses for the SE97BTP sensor.
 */
typedef enum SE97BTP_SensorRegisters
{
  SE97BTP_CONFIGURATIONREGISTER =
    0x01, /**< Address of the configuration register.*/
  SE97BTP_UPPERALARMREGISTER = 0x02, /**< Address of the upper alarm register.*/
  SE97BTP_LOWERALARMREGISTER = 0x03 /**< Address of the lower alarm register.*/,
  SE97BTP_CRITICALALARMREGISTER =
    0x04, /**< Address of the critical alarm register.*/
  SE97BTP_TEMPERATUREREGISTER = 0x05,
  /**< Address of the temperature register.*/
  SE97BTP_MANUFACTURERIDREGISTER, /**< Address of the manufacturer ID
                                   * register.*/
  SE97BTPDEVICEIDREGISTER,        /**< Address of the device ID register.*/
  SE97BTP_IDREGISTER,             /**< Address of the ID register.*/
} SE97BTP_SensorRegisters_t;

/*!
 * \brief A user-defined transfer operation for the temperature device.
 * \param[in] peripheral Void pointer to the peripheral instance used.
 * \param[in] regAddress Address of the register.
 * \param[in] regAddressSize The size of the registerAddress in bytes.
 * \param[in,out] buffer Pointer to the data buffer.
 * \param[in] dataSize The size of the output data in bytes. This value is set
 * to two for all temperature sensor operations.
 * \param[in] transferDirection Indicates I2C read or write direction.
 * \return Returns a status code indicating the result of the operation.
 * - 0: Successful
 * - Other error codes depend on the implementation of the transfer
 * function.
 * \note The function pointed to by this function pointer type is expected to
 * handle data transfer (both read and write operations) with the
 * SE97BTP's Temperature Sensor via I2C communication. The return value should
 * be adapted to the driver used for the peripheral, where 0 represents a
 * successful operation. It is crucial to ensure that the logic within the
 * transfer function accurately evaluates and executes read and write
 * operations, and that it is compatible with the error values used by this
 * driver. Detailed documentation on the error values and their meanings can be
 * found in the documentation for the specific driver's functions in use.
 * The parameter peripheral must not be used if you know, that you only have
 * one peripheral used for communication.
 */
typedef uint32_t (*SE97BTP_TransferFunctionTemperature_t)(
  void *peripheral, uint32_t regAddress, size_t regAddressSize, uint8_t *buffer,
  uint8_t dataSize, SE97BTP_TransferDirection_t transferDirection);

/*!
 * \brief A user-defined transfer operation for the EEPROM device.
 * \param[in] peripheral Void pointer to the peripheral instance used.
 * \param[in] slaveAddress The slave address of the device used for normal
 * operation.
 * \param[in] regAddress Address of the EEPROM.
 * \param[in] regAddressSize The size of the registerAddress in bytes.
 * \param[in] dataSize Size of the buffer that shall be transferred.
 * \param[in,out] buffer Pointer to the data buffer.
 * \param[in] transferDirection Indicates I2C read or write direction.
 * \return Returns a status code indicating the result of the operation.
 * - 0: Successful
 * - Other error codes depend on the implementation of the transfer
 * function.
 * \note The function pointed to by this function pointer type is expected to
 * handle data transfer (both read and write operations) with the
 * SE97BTP's EEPROM via I2C communication. The return value should be
 * adapted to the driver used for the peripheral, where 0 represents a
 * successful operation. It is crucial to ensure that the logic within the
 * transfer function accurately evaluates and executes read and write
 * operations, and that it is compatible with the error values used by this
 * driver. Detailed documentation on the error values and their meanings can be
 * found in the documentation for the specific driver's functions in use.
 */
typedef uint32_t (*SE97BTP_TransferFunctionEEPROM_t)(
  void *peripheral, uint8_t slaveAddress, uint8_t regAddress,
  size_t regAddressSize, uint8_t dataSize, uint8_t *buffer,
  SE97BTP_TransferDirection_t transferDirection);

/*!
 * \brief A user-defined delay function for EEPROM operations.
 * \param[in] us The delay time in microseconds.
 * This function type is designed to introduce a delay, specified in
 * microseconds, during EEPROM operations.
 * \note Example usage with NXP's SDK:
 * \code{.c}
 * void myDelayFunction(uint32_t us)
 * {
 *     SDK_DelayAtLeastUs( us, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY )
 * }
 * \endcode
 * */
typedef void (*SE97BTP_DelayFunctionEEPROM_t)(uint32_t us);

/*!
 * \brief handle structure for the SE97BTP EEPROM driver.
 * \note This struct should always be declared for the
 * operation of the driver.
 * \note Example usage:
 * \code{.c}
 * SE97BTP_transferEEPROM_t eeprom =
 * {
 * .transferFunction = myTransferFunction,
 * .delayFunction = myDelayFunction
 * };
 * \endcode
 */
typedef struct SE97BTP_EEPROM_Handle
{
  void   *peripheral;   /**< Pointer to the I2C peripheral object. */
  uint8_t slaveAddress; /**< The slaveAddress of the device used for
                         * accessing the memory array. */
  SE97BTP_TransferFunctionEEPROM_t
    transferFunction; /**< Pointer to user-defined transfer operation for
                       * protecting the EEPROM. */
  SE97BTP_DelayFunctionEEPROM_t delayFunction; /**< A function pointer to a
                                                * user-defined delay function.*/
} SE97BTP_EEPROM_Handle_t;

/*!
 * \brief handle structure for the SE97BTP temperature sensor driver.
 * \details The data buffer is sized with 2 bytes due to the sensor output.
 */
typedef struct SE97BTP_Sensor_Handle
{
  void   *peripheral; /**< Data buffer (2 bytes) where raw data is stored.*/
  uint8_t data[2];    /**< Function pointer to the transfer function.*/
  SE97BTP_TransferFunctionTemperature_t transfer;
} SE97BTP_Sensor_Handle_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

extern uint32_t SE97BTP_readTemp(SE97BTP_Sensor_Handle_t *sensor,
                                 float                   *temperatureValue);

extern uint32_t SE97BTP_shutDownMode(SE97BTP_Sensor_Handle_t *sensor);

extern uint32_t SE97BTP_writeEEPROM(SE97BTP_EEPROM_Handle_t *handle,
                                    uint8_t address, uint8_t *data,
                                    uint8_t dataSize);

extern uint32_t SE97BTP_readEEPROM(SE97BTP_EEPROM_Handle_t *handle,
                                   uint8_t address, uint8_t *data,
                                   uint8_t dataSize);

extern uint32_t SE97BTP_allowReadDuringProtection(
  SE97BTP_EEPROM_Handle_t *handle);

extern uint32_t SE97BTP_permanentWriteProtectEEPROM(
  SE97BTP_EEPROM_Handle_t *handle);

/*!@}*/

#endif /* _SE97BTP_H_ */
