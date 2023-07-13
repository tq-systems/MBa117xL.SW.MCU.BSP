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

#ifndef _PCF85063ATL_H_
#define _PCF85063ATL_H_

/*!
 * \defgroup PCF85063ATL PCF85063ATL
 * \brief Driver for the PCF85063ATL RTC.
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
 * Once initialization is complete, you are free to use
 * any function provided in this file to interact with the device.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * \brief Enumerates the possible directions for data transfer with the
 * PCF85063ATL RTC.
 */
typedef enum PCF85063ATL_TransferDirection
{
  PCF85063ATL_READ, /**< Indicates a read operation from the RTC.*/
  PCF85063ATL_WRITE /**< Indicates a write operation from the RTC.*/
} PCF85063ATL_TransferDirection_t;

/*!
 * \brief Represents a date and time in the PCF85063ATL RTC format.
 */
typedef struct PCF85063ATL_DateTime
{
  uint16_t year;  /**< The year value, ranging from 0 to 99.*/
  uint8_t  month; /**< The month value, ranging from 1 to 12.*/
  uint8_t
    day; /**< The day value, ranging from 1 to 31, depending on the month.*/
  uint8_t hour;    /**< The hour value ranging from 0 to 24.*/
  uint8_t minute;  /**< The minute value ranging from 0 to 60.*/
  uint8_t second;  /**< The second value ranging from 0 to 60.*/
  uint8_t weekday; /**< The weekday value, ranging from 0 to 6.*/

  bool AmPm; /**< Represents AM if false or PM if true (is not evaluated if
              * timemode is set to 24 hours).*/
} PCF85063ATL_DateTime_t;

/*!
 * \brief Configuration structure for the PCF85063ATL RTC.
 * \note This structure is used to configure various settings of the
 * PCF85063ATL RTC. The settings include software reset, external clock mode,
 * RTC stop, time mode, correction interrupt enable, and capacitance select
 * for the crystal oscillator. For setting use the PCF85063ATL_configure
 * function.
 */
typedef struct PCF85063ATL_Configuration
{
  bool softwareReset; /**<
                       * - true: Perform a software reset and then set
                       * the configuration. After reset register values are set
                       * to default. However the hardware initializes with
                       * hardware default values.
                       * - false: Do not perform a software reset.*/
  bool ext_clock;     /**<
                       * - true: External clock mode.
                       * - false: Normal mode.*/
  bool stop;          /**<
                       * - true: RTC stopped.
                       * - false: Clock runs.*/
  bool timemode;      /**<
                       * - true: 12-hour mode.
                       * - false: 24-hour mode.*/
  bool CIE;           /**<  *
                       * - true: Correction interrupt enabled.
                       * - false: No correction interrupt.*/
  bool cap_sel;       /**<
                       * - true: 12 pF capacitance select for the crystal
                       * oscillator.
                       * - false: 7 pF capacitance select for the crystal
                       * oscillator.*/
} PCF85063ATL_Configuration_t;

/*!
 * \brief Type definition for a function pointer, facilitating data transfer
 * operations with the PCF85063ATL RTC.
 * \param peripheral A pointer to the I2C peripheral object.
 * \param regAddress The address of the RTC register targeted for read or
 * write operations.
 * \param regAddressSize The address of the RTC register
 * targeted for read or write
 * \param buffer A pointer to the data buffer
 * utilized for read or write operations.
 * \param dataSize The size, in bytes,
 * of the data to be transferred. \
 * param transferDirection Specifies the I2C
 * transfer direction (read/write).
 * \return Returns a status code indicating
 * the outcome of the transfer operation.
 * \note The function pointed to by
 * this function pointer type is expected to handle data transfer (both read
 * and write operations) with the PCF85063ATL RTC via I2C communication. The
 * return value should be adapted to the driver used for the peripheral, where
 * 0 represents a successful operation. It is crucial to ensure that the logic
 * within the transfer function accurately evaluates and executes read and
 * write operations, and that it is compatible with the error values used by
 * this driver. Detailed documentation on the error values and their meanings
 * can be found in the documentation for the driver's functions.
 */
typedef uint32_t (*PCF85063ATL_TransferFunction_t)(
  void *peripheral, uint8_t regAddress, const size_t regAddressSize,
  uint8_t *buffer, const size_t dataSize,
  PCF85063ATL_TransferDirection_t transferDirection);

/*!
 * \brief handle structure for the PCF85063ATL RTC driver.
 */
typedef struct PCF85063ATL_Handle
{
  void *peripheral; /**< Pointer to the I2C peripheral object.*/
  PCF85063ATL_Configuration_t config; /**< Configuration settings for the
                                       * PCF85063ATL RTC. */
  PCF85063ATL_TransferFunction_t
    transferFunction; /**< Callback function for handling data transfer
                       * operations with the RTC.*/
} PCF85063ATL_Handle_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

extern uint32_t PCF85063ATL_writeDateTime(PCF85063ATL_Handle_t   *handle,
                                          PCF85063ATL_DateTime_t *dateTime);
extern uint32_t PCF85063ATL_readDateTime(PCF85063ATL_Handle_t   *handle,
                                         PCF85063ATL_DateTime_t *dateTime);
extern uint32_t PCF85063ATL_write(PCF85063ATL_Handle_t *handle,
                                  uint8_t regAddress, uint8_t *buffer);
extern uint32_t PCF85063ATL_read(PCF85063ATL_Handle_t *handle,
                                 uint8_t regAddress, uint8_t *buffer);
extern uint32_t PCF85063ATL_configure(PCF85063ATL_Handle_t *handle);

extern void PCF85063ATL_getDefaultConfiguration(PCF85063ATL_Handle_t *handle);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!@}*/

#endif /* _PCF85063ATL_H_ */
