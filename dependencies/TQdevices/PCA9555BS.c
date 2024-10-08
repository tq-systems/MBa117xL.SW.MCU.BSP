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
 * \addtogroup PCA9555BS
 * @{
 * \file
 */

/*******************************************************************************
 * Includes
 *******************************************************************************/

#include "PCA9555BS.h"

/*******************************************************************************
 * Definitions
 *******************************************************************************/

#define PCA9555BS_REG_ADDRESS_SIZE 1
#define PCA9555BS_REG_SIZE 2

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * \brief Configures Ports and polarity of one PCA9555BS handle.
 * \param[in,out]  handle handle structure for the PCA9555BS.
 * \return Status of transfer.
 * - 0: Successful
 * - Other error codes depend on the implementation of the transfer
 * function.
 * \note The IO pins are set by changing the "IOMask"
 * attribute of the struct "PCA9555BS_portExpander_t".
 * \note The polarity of the
 * pins are set by changing the "PolarityMask" attribute of the struct
 * "PCA9555BS_portExpander_t".
 * \note The output of the
 * pins are set by changing the "OutputMask" attribute of the struct
 * "PCA9555BS_portExpander_t".
 */
uint32_t PCA9555BS_configure(PCA9555BS_Handle_t *handle)
{
  uint32_t status;
  status = PCA9555BS_configPortsIO(handle);
  if (status != 0)
  {
    return status;
  }
  status = PCA9555BS_setOutput(handle);
  if (status != 0)
  {
    return status;
  }

  return PCA9555BS_configPolarity(handle);
}

/*!
 * \brief  Configures the function of the IO pins (if it is an input/output).
 * \param[in,out]  handle handle structure for the PCA9555BS.
 * \return Status of
 * transfer.
 * - 0: Successful
 * - Other error codes depend on the implementation of the transfer
 * function.
 * \note The IO pins are set by changing the "IOMask" attribute
 * of the struct "PCA9555BS_portExpander_t" and afterwards calling this
 * function. Registers of this handle are written continuously. Make sure you
 * set the relevant parameter you want to change in the handle structure before
 * calling this function!
 * \note By configuring the IO-Ports all data out
 * registers are set to low.
 */
uint32_t PCA9555BS_configPortsIO(PCA9555BS_Handle_t *handle)
{
  uint32_t status;
  uint8_t  transferBuffer[2] = {0, 0};
  /*configure all data out registers to LOW*/
  transferBuffer[0] = 0x00;
  transferBuffer[1] = 0x00;
  status = handle->transferFunction(handle->peripheral, PORTEXP_OUTPUTPORT0_REG,
                                    PCA9555BS_REG_ADDRESS_SIZE, PCA9555BS_WRITE,
                                    transferBuffer, PCA9555BS_REG_SIZE);
  if (status != 0)
  {
    return status;
  }
  /*Configure Direction Register*/
  transferBuffer[0] = handle->IOMask.PIN00_PIN07_Mask;
  transferBuffer[1] = handle->IOMask.PIN10_PIN17_Mask;
  return handle->transferFunction(handle->peripheral, PORTEXP_CONFIGPORT0_REG,
                                  PCA9555BS_REG_ADDRESS_SIZE, PCA9555BS_WRITE,
                                  transferBuffer, PCA9555BS_REG_SIZE);
}

/*!
 * \brief Configures the polarity of the IO pins.
 * \param[in,out]  handle handle structure for the PCA9555BS.
 * \return Status of transfer.
 * - 0: Successful
 * - Other error codes depend on the implementation of the transfer
 * function.
 * \note The polarity of the pins are set by changing the
 * "PolarityMask" attribute of the struct "PCA9555BS_portExpander_t" and
 * afterwards calling this function. Make sure you set the
 * relevant parameter you want to change in the handle structure before calling
 * this function!
 */
uint32_t PCA9555BS_configPolarity(PCA9555BS_Handle_t *handle)
{
  uint8_t transferBuffer[2] = {0, 0};
  /*Configure Direction Register*/
  transferBuffer[0] = handle->PolarityMask.PIN00_PIN07_Mask;
  transferBuffer[1] = handle->PolarityMask.PIN10_PIN17_Mask;
  /*Configure Direction Register*/
  return handle->transferFunction(handle->peripheral, PORTEXP_POLINVPORT0_REG,
                                  PCA9555BS_REG_ADDRESS_SIZE, PCA9555BS_WRITE,
                                  transferBuffer, PCA9555BS_REG_SIZE);
}

/*!
 * \brief Writes data to register chosen by user.
 * \param[in,out] handle handle structure for the PCA9555BS analog frontend
 * driver.
 * \param[in] value Pointer to value that shall be written.
 * \param[in] regAddress The register address to read from.
 * \return Status of transfer.
 * - 0: Successful
 * - Other error codes depend on the implementation of the transfer
 * function.
 */
uint32_t PCA9555BS_writeRegister(uint8_t regAddress, uint8_t *value,
                                 PCA9555BS_Handle_t *handle)
{
  return handle->transferFunction(handle->peripheral, regAddress,
                                  PCA9555BS_REG_ADDRESS_SIZE, PCA9555BS_WRITE,
                                  value, PCA9555BS_REG_ADDRESS_SIZE);
}

/*!
 * \brief Reads register chosen by user.
 * \param[in,out] handle handle structure for the PCA9555BS.
 * \param[in] buffer Pointer to buffer where data shall be stored.
 * \param[in] regAddress The register address to read from.
 * \return Status of transfer.
 * - 0: Successful
 * - Other error codes depend on the implementation of the transfer
 * function.
 */
uint32_t PCA9555BS_readRegister(uint8_t regAddress, uint8_t *buffer,
                                PCA9555BS_Handle_t *handle)
{
  return handle->transferFunction(handle->peripheral, regAddress,
                                  PCA9555BS_REG_ADDRESS_SIZE, PCA9555BS_READ,
                                  buffer, PCA9555BS_REG_ADDRESS_SIZE);
}

/*!
 * \brief Writes data to the output port for the respective output pins set with
 * "PCA9555BS_configPortsIO".
 * \param[in,out] handle handle structure for the PCA9555BS.
 * \return Status of transfer.
 * - 0: Successful
 * - Other error codes depend on the implementation of the transfer
 * function.
 * \note The output of the pins are set by changing the
 * "OutputMask" attribute of the struct "PCA9555BS_portExpander_t" and
 * afterwards calling this function.
 */
uint32_t PCA9555BS_setOutput(PCA9555BS_Handle_t *handle)
{
  uint32_t status;
  status = PCA9555BS_writeRegister(
    PORTEXP_OUTPUTPORT0_REG, (uint8_t *) &handle->OutputMask.PIN00_PIN07_Mask,
    handle);
  if (status != 0)
  {
    return status;
  }
  return PCA9555BS_writeRegister(
    PORTEXP_OUTPUTPORT1_REG, (uint8_t *) &handle->OutputMask.PIN10_PIN17_Mask,
    handle);
}

/*!@}*/
