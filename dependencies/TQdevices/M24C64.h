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

#ifndef _M24C64_H_
#define _M24C64_H_

/*!
 * \defgroup M24C64 M24C64
 * \brief Driver for the M24C64 EEPROM.
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
 * Once initialization is complete, you are free to use any function provided
 * in this file to interact with the device.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>
#include <stddef.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * \brief selection of transfer direction.
 */
typedef enum M24C64_TransferDirection
{
  M24C64_READ, /**< select read mode.*/
  M24C64_WRITE /**< select write mode.*/
} M24C64_TransferDirection_t;

/*!
 * \param[in] peripheral Representation of used peripheral.
 * \param slaveAddr The slaveAddress of the device used for accessing the
 * memory array.
 * \param[in] regAddress An array of size 2 representing the address that is
 * wished to be read.
 * \param[in] regAddressSize The size of the registerAddress
 * in bytes.
 * \param[in,out] buffer  An array of the dataSize that is being used as a data
 * buffer.
 * \param[in] dataSize The size of the buffer.
 * \param transferDirection The transfer direction.
 * \return Returns a status code indicating the result of the operation.
 * - 0: Successful
 * - Other error codes depend on the implementation of the transfer
 * function.
 * \note The function pointed to by this function pointer type is expected to
 * handle data transfer (both read and write operations) with the
 * MC24C64 via I2C communication. The return value should be
 * adapted to the driver used for the peripheral, where 0 represents a
 * successful operation. It is crucial to ensure that the logic within the
 * transfer function accurately evaluates and executes read and write
 * operations, and that it is compatible with the error values used by this
 * driver. Detailed documentation on the error values and their meanings can be
 * found in the documentation for the specific driver's functions in use.
 */
typedef uint32_t (*M24C64_TransferFunction_t)(
  void *peripheral, uint8_t slaveAddr, uint16_t regAddress,
  size_t regAddressSize, uint8_t *buffer, uint16_t dataSize,
  M24C64_TransferDirection_t transferDirection);

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
 */
typedef void (*M24C64_DelayFunction_t)(uint32_t us);

/*!
 * \brief Struct encapsulating needed variables, transfer and delay functions
 * for the EEPROM operations.
 * \note This struct should always be declared for the operation of the driver.
 * \note Example usage:
 * \code{.c}
 * M24C64_Handle_t eeprom =
 * {
 * .peripheral = i2c_peripheral_configs
 * .slaveAddress = M24C64_ADRESS
 * .transferFunction = myTransferFunction,
 * .delayFunction = myDelayFunction
 * };
 * \endcode
 */
typedef struct M24C64_Handle
{
  void *peripheral; /**< Representation of used peripheral. This parameter can
                     * be also set to NULL.*/
  uint8_t slaveAddress; /**< The slaveAddress of the device used for
                         * accessing the memory array.*/
  M24C64_TransferFunction_t
    transferFunction; /**< A function
                       * pointer to a user-defined data transfer function for
                       * EEPROM operations.*/
  M24C64_DelayFunction_t
    delayFunction; /**< A function pointer to a user-defined delay function.
                    * This parameter can be set to NULL if a delay is not
                    * desired.*/
} M24C64_Handle_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

extern uint32_t M24C64_read(uint16_t address, uint8_t *data, uint16_t dataSize,
                            M24C64_Handle_t *handle);
extern uint32_t M24C64_write(uint16_t address, uint8_t *data, uint16_t dataSize,
                             M24C64_Handle_t *handle);
extern uint32_t M24C64_readIdPage(uint8_t *data, uint8_t dataSize,
                                  M24C64_Handle_t *handle);
extern uint32_t M24C64_writeIdPage(uint8_t *value, uint8_t dataSize,
                                   uint8_t byteOffset, M24C64_Handle_t *handle);
extern uint32_t M24C64_lockIdPage(M24C64_Handle_t *handle);

/*!@}*/

#endif /* _M24C64_H_ */
