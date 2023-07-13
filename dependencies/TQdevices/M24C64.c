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
 * \addtogroup M24C64
 * @{
 * \file
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "M24C64.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define M24C64_PAGE_SIZE (32)
#define M24C64_REG_ADDRESS_SIZE (2)
#define M24C64_REG_LOCK_COMMAND 0x400
#define M24C64_MEMORY_ACCESS_ADDRESS 0x50
#define M24C64_ID_PAGE_ACCESS_ADDRESS 0x58

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * \brief Read Data from EEPROM M24C64-DFMC6TG.
 * \param[in] address The starting address to read from.
 * \param[out] data Pointer to the buffer where the read data will be stored.
 * \param[in] dataSize The size of the data to be read.
 * \param[in] handle Pointer to configured handle in use.
 * \return Status of the transfer.
 * - 0: Succeeded
 *  - Other values: Error, according to the implementation of the
 * transfer function.
 */
uint32_t M24C64_read(uint16_t address, uint8_t *data, uint16_t dataSize,
                     M24C64_Handle_t *handle)
{
  handle->slaveAddress =
    M24C64_MEMORY_ACCESS_ADDRESS | (handle->slaveAddress & 0x7);
  return handle->transferFunction(handle->peripheral, handle->slaveAddress,
                                  address, M24C64_REG_ADDRESS_SIZE, data,
                                  dataSize, M24C64_READ);
}

/*!
 * \brief Write Data to EEPROM M24C64-DFMC6TG.
 * \param[in] address The starting address to write to.
 * \param[in] data Pointer to the data to be written.
 * \param[in] dataSize The size of the data to be written.
 * \param[in] handle Pointer to configured handle in use.
 * \return Status of the transfer.
 * - 0: Succeeded
 * - Other values: Error, according to the implementation of the
 * transfer function.
 */
uint32_t M24C64_write(uint16_t address, uint8_t *data, uint16_t dataSize,
                      M24C64_Handle_t *handle)
{
  uint32_t status     = 255;
  uint16_t byteOffset = 0U, length, remainder;
  handle->slaveAddress =
    M24C64_MEMORY_ACCESS_ADDRESS | (handle->slaveAddress & 0x7);
  do
  {
    remainder = (address + byteOffset) % M24C64_PAGE_SIZE;
    length    = (remainder + dataSize - byteOffset) > M24C64_PAGE_SIZE
                  ? (M24C64_PAGE_SIZE - remainder)
                  : (dataSize - byteOffset);
    status    = handle->transferFunction(
      handle->peripheral, handle->slaveAddress, address + byteOffset,
      M24C64_REG_ADDRESS_SIZE, &data[byteOffset], length, M24C64_WRITE);
    if (status != 0)
    {
      break;
    }
    if (handle->delayFunction != NULL)
    {
      handle->delayFunction(5000);
    }
    byteOffset += length;
  } while (byteOffset < dataSize);
  return status;
}

/*!
 * \brief Read the entire ID-Page of EEPROM M24C64-DFMC6TG.
 * \param[out] data Pointer to the buffer where the ID-Page data
 * will be stored.
 * \param[in] dataSize The size of the data to be read.
 * \param[in] handle Pointer to configured handle in use.
 * \return Status of the transfer.
 * - 0: Succeeded
 * - 200: Buffer Size exceeds limit of the ID Page.
 * - Other values: Error, according to the implementation of the of the
 * transfer function.
 */
uint32_t M24C64_readIdPage(uint8_t *data, uint8_t dataSize,
                           M24C64_Handle_t *handle)
{
  handle->slaveAddress =
    M24C64_ID_PAGE_ACCESS_ADDRESS | (handle->slaveAddress & 0x7);
  if (dataSize < M24C64_PAGE_SIZE)
  {
    return 200;
  }
  return handle->transferFunction(handle->peripheral, handle->slaveAddress, 0x0,
                                  M24C64_REG_ADDRESS_SIZE, data,
                                  M24C64_PAGE_SIZE, M24C64_READ);
}

/*!
 * \brief Write to the Identification Page of EEPROM M24C64-DFMC6TG.
 * \param[in] value Pointer to the data to be written.
 * \param[in] dataSize Size of the data to be written.
 * \param[in] byteOffset The offset of
 * the data that will be written.
 * \param[in] handle Pointer to configured handle in use.
 * \return Status of the transfer.
 * - 0: Succeeded
 * - 200: Buffer Size exceeds limit of the ID Page.
 * - Other values: Error, according to the implementation of the
 * transfer function.
 */
uint32_t M24C64_writeIdPage(uint8_t *value, uint8_t dataSize,
                            uint8_t byteOffset, M24C64_Handle_t *handle)
{
  uint32_t status;
  handle->slaveAddress =
    M24C64_ID_PAGE_ACCESS_ADDRESS | (handle->slaveAddress & 0x7);
  byteOffset &= 0xF;
  if ((dataSize + byteOffset) > M24C64_PAGE_SIZE)
  {
    return 200;
  }
  status = handle->transferFunction(handle->peripheral, handle->slaveAddress,
                                    byteOffset, M24C64_REG_ADDRESS_SIZE, value,
                                    (uint16_t) dataSize, M24C64_WRITE);
  if (status != 0)
  {
    return status;
  }
  if (handle->delayFunction != NULL)
  {
    handle->delayFunction(5000);
  }

  return status;
}

/*!
 * \brief Lock the ID Page of EEPROM M24C64-DFMC6TG.
 * \param[in] handle Pointer to configured handle in use.
 * \return Status of the transfer.
 *         - 0: Succeeded
 *         - Other values: Error, according to the implementation of the
 * transfer function.
 * \details The address of ID-Page is 0x0. Slave Address must be changed before
 * transfer!
 */
uint32_t M24C64_lockIdPage(M24C64_Handle_t *handle)
{
  handle->slaveAddress =
    M24C64_ID_PAGE_ACCESS_ADDRESS | (handle->slaveAddress & 0x7);
  uint8_t lockData = 0x02;
  return handle->transferFunction(
    handle->peripheral, handle->slaveAddress, M24C64_REG_LOCK_COMMAND,
    M24C64_REG_ADDRESS_SIZE, &lockData, 1, M24C64_WRITE);
}

/*!@}*/
