//*****************************************************************************
/*!
 * \copyright
 * SPDX-License-Identifier: BSD-3-Clause
 * \copyright
 * Copyright (c) 2021 - 2023 TQ-Systems GmbH <license@tq-group.com>,
 * D-82229 Seefeld, Germany.
 * Author: Isaac L. L. Yuki, Maximilian Kürth
 */
//******************************************************************************

/*!
 * \addtogroup MX25R1635F
 * @{
 * \file
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "MX25R1635F.h"
#include <string.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define WRITE_STATUS_REGISTER_CYCLE_TIME 30
#define SUSPEND_LATENCY 60

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

typedef enum
{
  MX25R1635F_COM_WRITE                     = 0x0,
  MX25R1635F_COM_WRITESTATUSREGISTER       = 0x01U,
  MX25R1635F_COM_PAGEPROGRAM               = 0x02U,
  MX25R1635F_COM_READ                      = 0x03U,
  MX25R1635F_COM_WRITEDISABLE              = 0x04U,
  MX25R1635F_COM_READSTATUSREGISTER        = 0x05U,
  MX25R1635F_COM_WRITEENABLE               = 0x06U,
  MX25R1635F_COM_READFAST                  = 0x0BU,
  MX25R1635F_COM_READCONFIGURATIONREGISTER = 0x15U,
  MX25R1635F_COM_SECTORERASE               = 0x20U,
  MX25R1635F_COM_WRITESECURITYREGISTER     = 0x2FU,
  MX25R1635F_COM_READSECURITYREGISTER      = 0x2BU,
  MX25R1635F_COM_BLOCKERASE32KB            = 0x52U,
  MX25R1635F_COM_CHIPERASE                 = 0x60U,
  MX25R1635F_COM_READIDENTIFICATION        = 0x9FU,
  MX25R1635F_COM_BLOCKERASE64KB            = 0xD8U
} MX25R1635F_command;

typedef enum
{
  MX25R1635F_SR_WIP  = 0x01U,
  MX25R1635F_SR_WEL  = 0x02U,
  MX25R1635F_SR_BP0  = 0x04U,
  MX25R1635F_SR_BP1  = 0x08U,
  MX25R1635F_SR_BP2  = 0x10U,
  MX25R1635F_SR_BP3  = 0x20U,
  MX25R1635F_SR_QE   = 0x40U,
  MX25R1635F_SR_SRWD = 0x80U
} MX25R1635F_status_register_flags;

static uint32_t MX25R1635F_readStatusRegister(
  MX25R1635F_Handle_t *const handle, unsigned char *const value);

static uint32_t MX25R1635F_enableWrite(MX25R1635F_Handle_t *const handle);

static uint32_t MX25R1635F_disableWrite(MX25R1635F_Handle_t *const handle);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * \brief Detects the MX25R1635F device.
 * This function sends a read identification command to the device and checks
 * the received data to verify the presence of the device.
 * \param[in,out] handle A pointer to the MX25R1635F_Handle_t structure.
 * \return true if the device is detected, false otherwise.
 */
bool MX25R1635F_detect(MX25R1635F_Handle_t *const handle)
{
  handle->tx_buffer[0] = MX25R1635F_COM_READIDENTIFICATION;
  uint32_t status      = handle->transferFunction(
    handle->peripheral, handle->tx_buffer, handle->rx_buffer, 4U);
  if ((status == 0) && handle->rx_buffer[1] == 0xC2U)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/*!
 * \brief Reads a page from the MX25R1635F device.
 * This function sends a read command to the device and retrieves the data
 * from the specified address.
 * \param[in,out] handle A pointer to the MX25R1635F_Handle_t structure.
 * \param[in] address The address from where to read the data.
 * \param[in] dataSize The length of the data to be read.
 * \return A status code indicating the result of the operation.
 * - 0: Succeeded
 * - Other error codes depend on the implementation of the transfer function.
 * \note Only one page will be read if data length exceeds MX25R1635F_PAGE_SIZE.
 */
uint32_t MX25R1635F_readPage(MX25R1635F_Handle_t *const handle,
                              size_t const address, size_t const dataSize)
{
  uint32_t status;

  handle->tx_buffer[0] = MX25R1635F_COM_READ;
  handle->tx_buffer[1] = (address >> 16U) & 0xFFU;
  handle->tx_buffer[2] = (address >> 8U) & 0xFFU;
  handle->tx_buffer[3] = (address >> 0U) & 0xFFU;
  status = handle->transferFunction(handle->peripheral, handle->tx_buffer,
                                    handle->rx_buffer, dataSize);
  if (handle->delayFunction != NULL)
  {
    handle->delayFunction(SUSPEND_LATENCY);
  }
  return status;
}

/*!
 * \brief Programs a page in the MX25R1635F device.
 * This function writes data to a specified page in the device after enabling
 * write operations.
 * \param[in,out] handle A pointer to the MX25R1635F_Handle_t structure.
 * \param[in] address The address where to write the data.
 * \param[in] buffer A pointer to the buffer containing the data to be written.
 * \param[in] dataSize The length of the data to be written.
 * \return A status code indicating the result of the operation.
 * - 0: Succeeded
 * - Other error codes depend on the implementation of the transfer function.
 * \note Ensure that the data to be written does not exceed the page size.
 */
uint32_t MX25R1635F_programPage(MX25R1635F_Handle_t *const handle,
                                 size_t const address, char *buffer,
                                 size_t *const dataSize)
{
  uint32_t status = 0;
  size_t   len;

  status = MX25R1635F_enableWrite(handle);
  if (status != 0)
  {
    return status;
  }

  /*programm page*/
  len =
    (MX25R1635F_PAGE_SIZE > *dataSize ? *dataSize : MX25R1635F_PAGE_SIZE) + 4U;
  handle->tx_buffer[0] = MX25R1635F_COM_PAGEPROGRAM;
  handle->tx_buffer[1] = (address >> 16U) & 0xFFU;
  handle->tx_buffer[2] = (address >> 8U) & 0xFFU;
  handle->tx_buffer[3] = (address >> 0U) & 0xFFU;
  memcpy(&handle->tx_buffer[4], buffer, *dataSize);
  status = handle->transferFunction(handle->peripheral, handle->tx_buffer,
                                    handle->rx_buffer, len);
  if (handle->delayFunction != NULL)
  {
    handle->delayFunction(SUSPEND_LATENCY);
  }

  while (!MX25R1635F_WipReady(handle))
  {
  }

  return status;
}

/*!
 * \brief Erases a sector in the MX25R1635F device.
 * This function sends a sector erase command to the specified address in the
 * device.
 * \param[in,out] handle A pointer to the MX25R1635F_Handle_t structure.
 * \param[in] address The address of the sector to be erased.
 * \return A status code indicating the result of the operation.
 * - 0: Succeeded
 * - Other error codes depend on the implementation of the transfer function.
 */
uint32_t MX25R1635F_sectorErase(MX25R1635F_Handle_t *const handle,
                                 size_t const               address)
{
  uint32_t status;
  /*write enable*/
  status = MX25R1635F_enableWrite(handle);
  if (status != 0)
  {
    return status;
  }
  handle->tx_buffer[0] = MX25R1635F_COM_SECTORERASE;
  handle->tx_buffer[1] = (address >> 16U) & 0xFFU;
  handle->tx_buffer[2] = (address >> 8U) & 0xFFU;
  handle->tx_buffer[3] = (address >> 0U) & 0xFFU;
  status = handle->transferFunction(handle->peripheral, handle->tx_buffer,
                                    handle->rx_buffer, 4);
  /*Wait for erase*/
  while (!MX25R1635F_WipReady(handle))
  {
  }
  return status;
}

/*!
 * \brief Checks if the Write In Progress (WIP) bit is ready.
 * This function reads the status register and checks the WIP bit to determine
 * if the device is ready for the next write operation.
 * \param[in,out] handle A pointer to the MX25R1635F_Handle_t structure.
 * \return true if the WIP bit is ready (not busy), false otherwise or if
 * transfer failed.
 */
bool MX25R1635F_WipReady(MX25R1635F_Handle_t *const handle)
{
  unsigned char value;
  uint32_t      status = MX25R1635F_readStatusRegister(handle, &value);
  if ((status == 0)
      && ((value & (MX25R1635F_SR_WIP | MX25R1635F_SR_WEL)) == 0U))
  {
    return true;
  }
  return false;
}

/*!
 * \brief Reads the status register of the MX25R1635F device.
 * This function sends a read status register command to the device and
 * retrieves the status register value.
 * \param[in,out] handle A pointer to the MX25R1635F_Handle_t structure.
 * \param[out] value A pointer to a variable where the status register value
 * will be stored.
 * \return A status code indicating the result of the operation.
 * - 0: Succeeded
 * - Other error codes depend on the implementation of the transfer function.
 */
uint32_t MX25R1635F_readStatusRegister(MX25R1635F_Handle_t *const handle,
                                         unsigned char *const       value)
{
  uint32_t status      = 0;
  handle->tx_buffer[0] = MX25R1635F_COM_READSTATUSREGISTER;
  status = handle->transferFunction(handle->peripheral, handle->tx_buffer,
                                    handle->rx_buffer, 2U);
  if (handle->delayFunction != NULL)
  {
    handle->delayFunction(WRITE_STATUS_REGISTER_CYCLE_TIME);
  }
  *value = handle->rx_buffer[1];
  return status;
}

/*!
 * \brief Reads the security register of the MX25R1635F device.
 * This function sends a read security register command to the device and
 * retrieves the security register value.
 * \param[in,out] handle A pointer to the MX25R1635F_Handle_t structure.
 * \param[out] value A pointer to a variable where the security register value
 * will be stored.
 * \return A status code indicating the result of the operation.
 * - 0: Succeeded
 * - Other error codes depend on the implementation of the transfer function.
 */
uint32_t MX25R1635F_readSecurityRegisterRead(MX25R1635F_Handle_t *const handle,
                                             unsigned char *const       value)
{
  uint32_t status      = 0;
  handle->tx_buffer[0] = MX25R1635F_COM_READSECURITYREGISTER;
  status = handle->transferFunction(handle->peripheral, handle->tx_buffer,
                                    handle->rx_buffer, 2U);
  if (handle->delayFunction != NULL)
  {
    handle->delayFunction(WRITE_STATUS_REGISTER_CYCLE_TIME);
  }
  *value = handle->rx_buffer[1];
  return status;
}

/*!
 * \brief Writes to the security register of the MX25R1635F device.
 * This function sends a write security register command to the device.
 * \param[in,out] handle A pointer to the MX25R1635F_Handle_t structure.
 * \param[in] value A pointer to a variable containing the security register
 * value to be written. \return A status code indicating the result of the
 * operation.
 * - 0: Succeeded
 * - Other error codes depend on the implementation of the transfer function.
 */
uint32_t MX25R1635F_writeSecurityRegisterRead(MX25R1635F_Handle_t *const handle,
                                             unsigned char *const       value)
{
  uint32_t status      = 0;
  handle->tx_buffer[0] = MX25R1635F_COM_WRITESECURITYREGISTER;
  status = handle->transferFunction(handle->peripheral, handle->tx_buffer,
                                    handle->rx_buffer, 2U);
  if (handle->delayFunction != NULL)
  {
    handle->delayFunction(WRITE_STATUS_REGISTER_CYCLE_TIME);
  }
  *value = handle->rx_buffer[1];
  return status;
}

/*!
 * \brief Writes to the status register of the MX25R1635F device.
 * This function sends a write status register command to the device.
 * \param[in,out] handle A pointer to the MX25R1635F_Handle_t structure.
 * \param[in] value A pointer to the data to be written to the status register.
 * \param[in] dataSize The length of the data to be written.
 * \return A status code indicating the result of the operation.
 * - 0: Succeeded
 * - Other error codes depend on the implementation of the transfer function.
 */
uint32_t MX25R1635F_writeStatusRegister(MX25R1635F_Handle_t *const handle,
                                          unsigned char const *const value,
                                          size_t const               dataSize)
{
  uint32_t status = 0;
  status          = MX25R1635F_enableWrite(handle);
  if (status != 0)
  {
    return status;
  }
  handle->tx_buffer[0] = MX25R1635F_COM_WRITESTATUSREGISTER;
  memcpy(&handle->tx_buffer[0], value, (dataSize > 3U ? 3U : dataSize));
  status = handle->transferFunction(handle->peripheral, handle->tx_buffer,
                                    handle->rx_buffer,
                                    (dataSize > 3U ? 3U : dataSize));
  if (status == 0)
  {
    status = MX25R1635F_disableWrite(handle);
  }
  return status;
}

/*!
 * \brief Enables write operations for the MX25R1635F status register.
 * This function sends a write enable command to the device and checks the
 * status register to ensure that the Write Enable Latch (WEL) bit is set.
 * \param[in,out] handle A pointer to the MX25R1635F_Handle_t structure.
 * \return A status code indicating the result of the operation.
 * - 0: Succeeded
 * - Other error codes depend on the implementation of the transfer function.
 */
static uint32_t MX25R1635F_enableWrite(MX25R1635F_Handle_t *const handle)
{
  uint32_t status = 0;
  uint8_t  status_register;
  do
  {
    handle->tx_buffer[0] = MX25R1635F_COM_WRITEENABLE;
    status = handle->transferFunction(handle->peripheral, handle->tx_buffer,
                                      handle->rx_buffer, 1U);
    if (handle->delayFunction != NULL)
    {
      handle->delayFunction(WRITE_STATUS_REGISTER_CYCLE_TIME);
    }
    if (status != 0)
    {
      return status;
    }
    status = MX25R1635F_readStatusRegister(handle, &status_register);
    if (status != 0)
    {
      return status;
    }
    status_register &= MX25R1635F_SR_WIP | MX25R1635F_SR_WEL;
  } while (status_register != 0x02);
  return status;
}

/*!
 * \brief Disables write operations for the MX25R1635F status register.
 * This function sends a write disable command to the device.
 * \param[in,out] handle A pointer to the MX25R1635F_Handle_t structure.
 * \return A status code indicating the result of the operation.
 * - 0: Succeeded
 * - Other error codes depend on the implementation of the transfer function.
 */
static uint32_t MX25R1635F_disableWrite(MX25R1635F_Handle_t *const handle)
{
  uint32_t status      = 0;
  handle->tx_buffer[0] = MX25R1635F_COM_WRITEDISABLE;
  status = handle->transferFunction(handle->peripheral, handle->tx_buffer,
                                    handle->rx_buffer, 1);
  if (handle->delayFunction != NULL)
  {
    handle->delayFunction(WRITE_STATUS_REGISTER_CYCLE_TIME);
  }
  return status;
}

/*!@}*/
