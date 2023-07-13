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

/*!
 * \addtogroup PCF85063ATL
 * @{
 * \file
 */

/*******************************************************************************
 * Includes
 *******************************************************************************/

#include "PCF85063ATL.h"

/*******************************************************************************
 * Definitions
 *******************************************************************************/

/*!
 *\name Register and bitmasks
 * @{
 */
#define PCF85063ATL_CR1 0x00 /*Control Register 1 Address*/
#define PCF85063ATL_CR2 0x01 /*Control Register 2 Address*/
#define PCF85063ATL_OFF 0x02 /*Offset Register*/
#define PCF85063ATL_RAM 0x03 /*RAM_BYTE Register*/
#define PCF85063ATL_SECONDS 0x04
#define PCF85063ATL_MINUTES 0x05
#define PCF85063ATL_HOURS 0x06
#define PCF85063ATL_DAYS 0x07
#define PCF85063ATL_WEEKDAY 0x08
#define PCF85063ATL_MONTHS 0x09
#define PCF85063ATL_YEARS 0x0A

/*Configuration register bitmasks*/
#define PCF85063ATL_SOFTWARE_RESET_MASK 0x58U
#define PCF85063ATL_EXTERNAL_CLOCK_MASK 0x80U
#define PCF85063ATL_RTC_STOP_MASK 0x2U
#define PCF85063ATL_CIE_MASK 0x04U
#define PCF85063ATL_TIMEMODE_MASK 0x02U
#define PCF85063ATL_CAP_SEL_MASK 0x01

/*! @} */

#define PCF85063ATL_REGISTER_ADDRESS_SIZE (1U)
#define PCF85063ATL_REGISTER_SIZE (1U)
#define PCF85063ATL_DATETIME_REGISTERS_COUNT (7U)

#define PCF85063ATL_YEAROFFSET (2000U)

#define PCF85063ATL_ERROR (200U)

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static bool    bin_to_bcd(uint8_t *value);
static bool    bcd_to_bin(uint8_t *value);
static bool    checkdateTimeFormat(PCF85063ATL_Handle_t   *handle,
                                   PCF85063ATL_DateTime_t *dateTime);
static uint8_t parseYear(PCF85063ATL_DateTime_t *dateTime);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * \brief Writes date and time data to the PCF85063ATL RTC.
 *
 * \param[in] handle handle structure for the PCF85063ATL RTC driver.
 * \param[in] dateTime Struct containing date and time.
 * \return Returns a status code indicating the result of the write operation.
 * - 0: Successful
 * - PCF85063ATL_ERROR : If bin_to_bcd conversion did not succeed or date/time
 * format is invalid.
 * - Other error codes depend on the implementation of the transfer
 * function.
 * \note This function converts the binary date and time data (seconds,
 * minutes, hours, day, month, and year) in the provided
 * PCF85063ATL_DateTime_t structure to BCD (Binary-Coded Decimal) format and
 * writes it to the corresponding registers of the PCF85063ATL RTC
 * handle->peripheral. The function communicates with the RTC
 * handle->peripheral using the provided transfer function. This function
 * takes the timemode in account, meaning that in AM/PM mode, the 5th bit for
 * the hour will be written accordingly.
 */
uint32_t PCF85063ATL_writeDateTime(PCF85063ATL_Handle_t   *handle,
                                   PCF85063ATL_DateTime_t *dateTime)
{
  uint32_t status            = 256;
  uint8_t  transferBuffer[7] = {dateTime->second,
                                dateTime->minute,
                                dateTime->hour,
                                dateTime->day,
                                dateTime->weekday,
                                dateTime->month,
                                0x0U};

  if (!checkdateTimeFormat(handle, dateTime))
  {
    return PCF85063ATL_ERROR;
  }

  for (uint8_t i = 0; i < PCF85063ATL_DATETIME_REGISTERS_COUNT; i++)
  {
    if (i == 2)
    {
      if (handle->config.timemode)
      {
        if (dateTime->AmPm)
        {
          transferBuffer[i] |= 0x10;
        }
      }
    }
    else if (i == 6)
    {
      transferBuffer[i] = parseYear(dateTime);
    }

    if (!bin_to_bcd(&transferBuffer[i]))
    {
      return PCF85063ATL_ERROR;
    };
  }

  status = handle->transferFunction(
    handle->peripheral, PCF85063ATL_SECONDS, PCF85063ATL_REGISTER_SIZE,
    transferBuffer, PCF85063ATL_DATETIME_REGISTERS_COUNT, PCF85063ATL_WRITE);

  return status;
}

/*!
 * \brief Reads the date and time from the PCF85063ATL RTC.
 *
 * \param[in] handle Handle structure for the PCF85063ATL RTC driver.
 * \param[out] dateTime Struct containing date and time.
 * \return Returns a status code indicating the result of the read operation.
 * - 0: Successful
 * - PCF85063ATL_ERROR : If bcd_to_bin conversion did not succeed.
 * Other error codes depend on the implementation of the transfer
 * function.
 * \note This function reads the date and time (seconds, minutes, hours, day,
 * month, and year) from the PCF85063ATL RTC and converts the BCD (Binary-Coded
 * Decimal) values to binary format. The read values are stored in the provided
 * PCF85063ATL_DateTime_t structure.
 * This function takes the timemode in account, meaning that in AM/PM mode, the
 * 5th bit for the hour will be evaluated accordingly.
 */
uint32_t PCF85063ATL_readDateTime(PCF85063ATL_Handle_t   *handle,
                                  PCF85063ATL_DateTime_t *dateTime)
{
  uint32_t status = 256;
  uint8_t  buffer[7];
  uint8_t  registerValuesBitmask[7] = {0x7FU, 0x7FU, 0x3FU, 0x3FU,
                                       0x07U, 0x1FU, 0xFF};
  dateTime->AmPm                    = false;
  status                            = handle->transferFunction(
    handle->peripheral, PCF85063ATL_SECONDS, PCF85063ATL_REGISTER_ADDRESS_SIZE,
    buffer, PCF85063ATL_DATETIME_REGISTERS_COUNT, PCF85063ATL_READ);
  if (status != 0)
  {
    return status;
  }

  for (uint8_t i = 0; i < PCF85063ATL_DATETIME_REGISTERS_COUNT; i++)
  {
    if (i == 2)
    {
      if (!handle->config.timemode)
      {
        buffer[i] &= 0x3FU;
      }
      else
      {
        if (buffer[i] & 0x10)
        {
          dateTime->AmPm = true;
        }
        buffer[i] &= 0xFU;
      }
    }
    else
    {
      buffer[i] &= registerValuesBitmask[i];
    }
    if (!bcd_to_bin(&buffer[i]))
    {
      return PCF85063ATL_ERROR;
    };
  }

  dateTime->second  = buffer[0];
  dateTime->minute  = buffer[1];
  dateTime->hour    = buffer[2];
  dateTime->day     = buffer[3];
  dateTime->weekday = buffer[4];
  dateTime->month   = buffer[5];
  dateTime->year    = (uint16_t) buffer[6] + PCF85063ATL_YEAROFFSET;

  return status;
}

/*!
 * \brief Convert an 8-bit BCD (Binary-Coded Decimal) value to its binary
 * equivalent.
 *
 * \param value Pointer to an 8-bit BCD value to convert.
 * \return Returns true if the conversion was successful. Otherwise, returns
 * false.
 * \note This function converts an 8-bit BCD value to its binary equivalent.
 * The function iteratively extracts each digit from the BCD value, converts it
 * to its binary equivalent, and accumulates the result. The original BCD value
 * is replaced with the binary equivalent upon successful conversion.
 */
static bool bcd_to_bin(uint8_t *value)
{
  uint8_t multiplier = 1U;
  uint8_t result     = 0;
  while (*value > 0U)
  {
    uint8_t const digit = *value & 0xFU;

    // Error check: BCD digit should be 0-9
    if (digit > 9U)
    {
      return false;
    }
    *value >>= 4U;
    result += multiplier * digit;
    multiplier *= 10U;
  }
  *value = result;
  return true;
}

/*!
 * \brief Convert an 8-bit binary value to its BCD (Binary-Coded Decimal)
 * equivalent.
 * \param value Pointer to an 8-bit binary value to convert. Must be less
 * than 100.
 * \return Returns true if the conversion was successful. Otherwise,
 * returns false.
 * \note This function converts an 8-bit binary value to its BCD
 * equivalent. The function checks if the binary value is less than 100,
 * extracts the units and tens digits, and constructs the BCD equivalent. The
 * original binary value is replaced with the BCD equivalent upon successful
 * conversion.
 */
static bool bin_to_bcd(uint8_t *value)
{
  // Error check: binary value should be < 100
  if (*value >= 100U)
  {
    return false;
  }
  uint8_t unit = *value % 10;
  *value       = *value - unit;
  *value       = (((*value / 10) << 4) | unit);
  return true;
}

/*!
 * \brief Validate the format of a date and time structure.
 *
 * \param handle A pointer to a PCF85063ATL_Handle_t structure containing
 * the date and time data to validate.
 * \return Returns true if the date and time are in valid
 * formats, otherwise returns false.
 * \note This function checks whether the day, month, year, and weekday fields
 * of the provided PCF85063ATL_DateTime_t structure within the handle are
 * within the valid ranges for a date. The valid ranges are:
 * - Day: 1-31 (adjusted for months and leap years)
 * - Month: 1-12
 * - Weekday: 0-6 or 1-7 (depending on the system, where 0 or 1 can represent
 * Sunday)
 * \note This function also checks whether the hour, minute, and second
 * fields are within valid ranges, which vary depending on the time mode:
 * - For 24-hour mode: Hour: 0-23, Minute: 0-59, Second: 0-59
 * - For 12-hour mode: Hour: 1-12, Minute: 0-59, Second: 0-59 (AM/PM indicator
 * not checked)
 */
static bool checkdateTimeFormat(PCF85063ATL_Handle_t   *handle,
                                PCF85063ATL_DateTime_t *dateTime)
{
  uint8_t daysPerMonth[] = {0U,  31U, 28U, 31U, 30U, 31U, 30U,
                            31U, 31U, 30U, 31U, 30U, 31U};

  /* Check Date */
  if ((dateTime->month > 12U) || (dateTime->month < 1U))
  {
    return false;
  }

  /* Adjust the days in February for a leap year */
  if ((((dateTime->year & 3U) == 0U) && (dateTime->year % 100U != 0U))
      || (dateTime->year % 400U == 0U))
  {
    daysPerMonth[2] = 29U;
  }

  /* Check the validity of the day or weekday */
  if ((dateTime->day > daysPerMonth[dateTime->month]) || (dateTime->day < 1U)
      || dateTime->weekday < 1 || dateTime->weekday > 7)
  {
    return false;
  }

  /* Check time */
  if (handle->config.timemode)
  {
    if (dateTime->hour < 1 || dateTime->hour > 12 || dateTime->minute >= 60
        || dateTime->second >= 60)
    {
      return false;
    }
  }
  else
  {
    if (dateTime->hour >= 24 || dateTime->minute >= 60
        || dateTime->second >= 60)
    {
      return false;
    }
  }
  return true;
}

/*!
 * \brief Parses the year format to the acceptable format of the PCF85063ATL
 * ranging from 0 to 99.
 * \param dateTime A pointer to a PCF85063ATL_DateTime_t structure containing
 * the time data to validate.
 * \return Parsed year as uint8_t entity.
 */
static uint8_t parseYear(PCF85063ATL_DateTime_t *dateTime)
{
  if (dateTime->year > 99)
  {
    return (uint8_t) (dateTime->year % 100);
  }
  return (uint8_t) dateTime->year;
}

/*!
 * \brief Configure the PCF85063ATL Real-Time Clock (RTC).
 *
 * \param handle handle structure for the PCF85063ATL RTC driver.
 * \return Returns a status code indicating the result of the configuration
 * operation.
 * \note This function configures the PCF85063ATL RTC with the provided
 * settings. It sets various control registers based on the configuration
 * settings and performs a software reset if specified in the configuration.
 */
uint32_t PCF85063ATL_configure(PCF85063ATL_Handle_t *handle)
{
  uint8_t  value = 0;
  uint32_t status;
  if (handle->config.softwareReset)
  {
    value  = PCF85063ATL_SOFTWARE_RESET_MASK;
    status = handle->transferFunction(handle->peripheral, PCF85063ATL_CR1,
                                      PCF85063ATL_REGISTER_ADDRESS_SIZE, &value,
                                      1, PCF85063ATL_WRITE);
    if (status != 0)
    {
      return status;
    }
  }
  value = 0;
  if (handle->config.ext_clock)
  {
    value |= PCF85063ATL_EXTERNAL_CLOCK_MASK;
  }
  if (handle->config.stop)
  {
    value |= PCF85063ATL_RTC_STOP_MASK;
  }
  if (handle->config.CIE)
  {
    value |= PCF85063ATL_CIE_MASK;
  }
  if (handle->config.timemode)
  {
    value |= PCF85063ATL_TIMEMODE_MASK;
  }
  if (handle->config.cap_sel)
  {
    value |= PCF85063ATL_CAP_SEL_MASK;
  }
  status = handle->transferFunction(
    handle->peripheral, PCF85063ATL_CR1, PCF85063ATL_REGISTER_ADDRESS_SIZE,
    &value, PCF85063ATL_REGISTER_SIZE, PCF85063ATL_WRITE);
  return status;
}

/*!
 * \brief Configure the PCF85063ATL Real-Time Clock (RTC).
 *
 * \param handle handle structure for the PCF85063ATL RTC driver.
 *
 */
void PCF85063ATL_getDefaultConfiguration(PCF85063ATL_Handle_t *handle)
{
  handle->config.cap_sel       = false;
  handle->config.CIE           = false;
  handle->config.ext_clock     = false;
  handle->config.stop          = false;
  handle->config.timemode      = false;
  handle->config.softwareReset = false;
}

/*!
 * \brief Write a byte of data to a register of the PCF85063ATL RTC.
 *
 * \param handle handle structure for the PCF85063ATL RTC driver.
 * \param regAddress The address of the RTC register to write to.
 * \param buffer Pointer to the data buffer for write operations.
 * \return Returns a status code indicating the result of the write operation.
 * - 0: Successful
 * - PCF85063ATL_ERROR : If bin_to_bcd conversion did not succeed or date/time
 * format is invalid.
 * - Other error codes depend on the implementation of the transfer
 * \note This function writes a byte of data to a specified register of the
 * PCF85063ATL RTC. If the register address is greater than
 * PCF85063ATL_SECONDS, the function converts the data from binary to BCD
 * format before writing.
 */
uint32_t PCF85063ATL_write(PCF85063ATL_Handle_t *handle, uint8_t regAddress,
                           uint8_t *buffer)
{
  if (regAddress > PCF85063ATL_SECONDS)
  {
    if (bin_to_bcd(buffer))
    {
      return PCF85063ATL_ERROR;
    }
  }
  return handle->transferFunction(handle->peripheral, regAddress,
                                  PCF85063ATL_REGISTER_ADDRESS_SIZE, buffer,
                                  PCF85063ATL_REGISTER_SIZE, PCF85063ATL_WRITE);
}

/*!
 * \brief Read a byte of data from a register of the PCF85063ATL RTC.
 *
 * \param handle handle structure for the PCF85063ATL RTC driver.
 * \param regAddress The address of the RTC register to read from.
 * \param buffer Pointer to the data buffer for read operations.
 * \return Returns a status code indicating the result of the write operation.
 * - 0: Successful
 * - PCF85063ATL_ERROR : If bin_to_bcd conversion did not succeed or date/time
 * format is invalid.
 * - Other error codes depend on the implementation of the transfer
 * \note This function reads a byte of data from a specified register of the
 * PCF85063ATL RTC. If the register address is greater than PCF85063ATL_SECONDS,
 * the function converts the data from BCD to binary format after reading.
 */
uint32_t PCF85063ATL_read(PCF85063ATL_Handle_t *handle, uint8_t regAddress,
                          uint8_t *buffer)
{
  if (regAddress > PCF85063ATL_SECONDS)
  {
    if (bcd_to_bin(buffer))
    {
      return PCF85063ATL_ERROR;
    }
  }
  return handle->transferFunction(handle->peripheral, regAddress,
                                  PCF85063ATL_REGISTER_ADDRESS_SIZE, buffer,
                                  PCF85063ATL_REGISTER_SIZE, PCF85063ATL_READ);
}

/*!@}*/
