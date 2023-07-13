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
 * \addtogroup SE97BTP
 * @{
 * \file
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "SE97BTP.h"
#include <string.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * \name General definitions
 * @{
 */
#define SE97BTP_PINSSEL_BITMASK (0x7U)
/*!
 * @}
 * \name Temperature sensor related definitions
 * @{
 */
#define SE97BTP_TEMPERATUREREGISTER_SIZE (2U)
#define SE97BTP_TEMPERATUREREGISTERADDRESS_SIZE (1U)
#define SE97BTP_SHUTDOWN_TEMP_MEASUREMENT_MASK (0x80U)
/*!
 * @}
 * \name EEPROM related definitions
 * @{
 */
#define SE97BTP_PAGE_SIZE ((uint8_t) 16U)
#define SE97BTP_PROTECT_COMMAND (0x6U)
#define SE97BTP_EEPROM_REGISTERADDRESS_SIZE (1U)
#define SE97BTP_EEPROM_PROTECT_DONT_CARE_SIZE (2U)
#define SE97BTP_EEPROM_PROTECT_REGISTERADDRESS_SIZE (0U)
#define SE97BTP_EEPROM_DONT_CARE (0U)
/*!@}*/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void SE97BTP_convertRaw2Temp(float                   *temperatureValue,
                                    SE97BTP_Sensor_Handle_t *sensor);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * \brief Reads the temperature from the SE97BTP sensor.
 * \param[in] sensor Pointer to configured handle in use.
 * transfer function being used.
 * \param[out] temperatureValue A pointer to where the value of the temperature
 * shall be stored.
 * \return Status of transfer.
 * - 0: Successful
 * - Other error codes depend on the implementation of the transfer
 * function.
 */
uint32_t SE97BTP_readTemp(SE97BTP_Sensor_Handle_t *sensor,
                          float                   *temperatureValue)
{
  uint32_t status =
    sensor->transfer(sensor->peripheral, SE97BTP_TEMPERATUREREGISTER,
                     SE97BTP_TEMPERATUREREGISTERADDRESS_SIZE, sensor->data,
                     SE97BTP_TEMPERATUREREGISTER_SIZE, SE97BTP_READ);
  SE97BTP_convertRaw2Temp(temperatureValue, sensor);
  return status;
}

/*!
 * \brief Converts raw temperature data to a float value.
 * \param[out] temperatureValue A pointer to where the converted temperature
 * value shall be stored.
 * \param[out] sensor Pointer to configured handle in use.
 * transfer function being used.
 */
static void SE97BTP_convertRaw2Temp(float                   *temperatureValue,
                                    SE97BTP_Sensor_Handle_t *sensor)
{
  const float conversion = 0.125;
  uint8_t    *data       = (uint8_t *) sensor->data;
  uint16_t    raw;
  raw = (((data[0] << 8) + data[1]) & 0x0FFF) >> 1;
  if ((raw & 0x0800)) // checks sign of value
  {
    raw               = (~raw + 1) & 0x7FF;
    *temperatureValue = -(float) raw * conversion;
  }
  else
  {
    *temperatureValue = (float) raw * conversion;
  }
}

/*!
 * \brief Toggles the "shutdown mode"-bit of the configuration register.
 * \param[in] sensor Pointer to configured handle in use.
 * transfer function being used.
 * \return Status of transfer.
 * - 0: Successful
 * - Other error codes depend on the implementation of the transfer
 * function.
 */
uint32_t SE97BTP_shutDownMode(SE97BTP_Sensor_Handle_t *sensor)
{
  uint32_t status =
    sensor->transfer(sensor->peripheral, SE97BTP_CONFIGURATIONREGISTER,
                     SE97BTP_TEMPERATUREREGISTERADDRESS_SIZE, sensor->data,
                     SE97BTP_TEMPERATUREREGISTER_SIZE, SE97BTP_READ);
  sensor->data[1] ^= SE97BTP_SHUTDOWN_TEMP_MEASUREMENT_MASK;
  status =
    sensor->transfer(sensor->peripheral, SE97BTP_CONFIGURATIONREGISTER,
                     SE97BTP_TEMPERATUREREGISTERADDRESS_SIZE, sensor->data,
                     SE97BTP_TEMPERATUREREGISTER_SIZE, SE97BTP_WRITE);
  return status;
}

/*!
 * \brief Reads a certain area of the EEPROM.
 * \param[in] handle Pointer to configured handle in use.
 * transfer function being used.
 * \param[in] address The struct with page and byte that shall be read.
 * \param[out] data Pointer to the data buffer.
 * \param[in] dataSize Size of the data to be read.
 * \return Status of transfer.
 * - 0: Successful
 * - Other error codes depend on the implementation of the transfer
 * function.
 */
uint32_t SE97BTP_readEEPROM(SE97BTP_EEPROM_Handle_t *handle, uint8_t address,
                            uint8_t *data, uint8_t dataSize)
{
  uint32_t status = handle->transferFunction(
    handle->peripheral, handle->slaveAddress, address,
    SE97BTP_EEPROM_REGISTERADDRESS_SIZE, dataSize, data, SE97BTP_READ);
  return status;
}

/*!
 * \brief Reads a certain area of the EEPROM.
 * \param[in] handle Pointer to configured handle in use.
 * transfer function being used.
 * \param[in] address The struct with page and byte that shall be read.
 * \param[in] data Pointer to the data buffer.
 * \param[in] dataSize Size of the data to be read.
 * \return Status of transfer.
 * - 0: Successful
 * - Other error codes depend on the implementation of the transfer
 * function.
 */
uint32_t SE97BTP_writeEEPROM(SE97BTP_EEPROM_Handle_t *handle, uint8_t address,
                             uint8_t *data, uint8_t dataSize)
{
  uint32_t status     = 255;
  uint8_t  byteOffset = 0U, length, remainder;

  do
  {
    remainder = (address + byteOffset) % SE97BTP_PAGE_SIZE;
    length    = (remainder + dataSize - byteOffset) > SE97BTP_PAGE_SIZE
                  ? (SE97BTP_PAGE_SIZE - remainder)
                  : (dataSize - byteOffset);
    status = handle->transferFunction(handle->peripheral, handle->slaveAddress,
                                      address + byteOffset,
                                      SE97BTP_EEPROM_REGISTERADDRESS_SIZE,
                                      length, &data[byteOffset], SE97BTP_WRITE);
    byteOffset += length;
    if (handle->delayFunction != NULL)
    {
      handle->delayFunction(10000);
    }
  } while (byteOffset < dataSize && status == 0);
  return status;
}

/*!
 * \brief Sends command to permanently write protect the first 128 bytes of the
 * EEPROM memory.
 * \param[in] handle Pointer to configured handle in use.
 * transfer function being used.
 * \return Status of transfer.
 * - 0: Successful
 * - Other error codes depend on the implementation of the transfer
 * function.
 * \details The sent slaveAddress is the slave Address of the device composed of
 * it's 3 last bits(derived from the pin selection) and the protect command.
 * \note Function has to be tested yet.
 */
uint32_t SE97BTP_permanentWriteProtectEEPROM(SE97BTP_EEPROM_Handle_t *handle)
{
  handle->slaveAddress = (SE97BTP_PROTECT_COMMAND << 3)
                         | (handle->slaveAddress & SE97BTP_PINSSEL_BITMASK);
  return handle->transferFunction(
    handle->peripheral, handle->slaveAddress, SE97BTP_EEPROM_DONT_CARE,
    SE97BTP_EEPROM_PROTECT_REGISTERADDRESS_SIZE,
    SE97BTP_EEPROM_PROTECT_DONT_CARE_SIZE, SE97BTP_EEPROM_DONT_CARE,
    SE97BTP_WRITE); // has to send 2 don't care bytes of data
}

/*!
 * \brief Sends command to allow read in write permanent protection mode.
 * \param[in] handle Pointer to configured handle in use.
 * transfer function being used.
 * \return Status of transfer.
 * - 0: Successful
 * - Other error codes depend on the implementation of the transfer
 * function.
 * \details The sent slaveAddress is the slave Address of the device composed of
 * it's 3 last bits(
 * derived from the pin selection) and the protect command.
 * \note Function has to be tested yet.
 */
uint32_t SE97BTP_allowReadDuringProtection(SE97BTP_EEPROM_Handle_t *handle)
{
  handle->slaveAddress = (SE97BTP_PROTECT_COMMAND << 3)
                         | (handle->slaveAddress & SE97BTP_PINSSEL_BITMASK);
  return handle->transferFunction(
    handle->peripheral, handle->slaveAddress, SE97BTP_EEPROM_DONT_CARE,
    SE97BTP_EEPROM_PROTECT_REGISTERADDRESS_SIZE,
    SE97BTP_EEPROM_PROTECT_DONT_CARE_SIZE, SE97BTP_EEPROM_DONT_CARE,
    SE97BTP_READ); // has to send 2 don't care bytes of data
}

/*!@}*/
