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

#ifndef _MX25R1635F_H_
#define _MX25R1635F_H_

/*!
 * \defgroup MX25R1635F MX25R1635F
 * \brief Driver for the MX25R1635F flash.
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
 * \todo Rework "MX25R1635F_program_page" function in a write function capable
 * of writing to the entire chip. Add missing functionalities of chip that
 * aren't described in this driver.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define MX25R1635F_PAGE_SIZE 0x100U
#define MX25R1635F_SECTOR_SIZE 0x1000U

/*Internal buffer has size of page_size + command + address */
#define MX25R1635F_INTERNAL_BUFFER_SIZE (MX25R1635F_PAGE_SIZE + 1U + 3U)

/*!
 * \brief User-defined transfer operation for the MX25R1635F device.
 * \param[in,out] peripheral Pointer to the peripheral instance.
 * \param[in] tx_buffer Pointer to the transmit buffer.
 * \param[in] rx_buffer Pointer to the receive buffer.
 * \param[in] bufferSize Length of the transmit and receive buffers.
 * \return Status of the transfer operation.
 * - 0: Successful
 * - Other: Error codes based on the transfer function implementation.
 * \note
 * The TX and RX buffers of the handle must be linked to its peripheral.
 * This function is expected to handle data transfer (both read and write
 * operations) via SPI. The return value should be adapted to the driver used
 * for the peripheral, where 0 represents a successful operation.
 */
typedef uint32_t (*MX25R1635F_TransferFunction_t)(void          *peripheral,
                                                  unsigned char *tx_buffer,
                                                  unsigned char *rx_buffer,
                                                  size_t const   bufferSize);

/*!
 * \brief A user-defined delay function.
 * \param[in] us The delay time in microseconds.
 * This function type is designed to introduce a delay, specified in
 * microseconds, during EEPROM operations.
 * \note Example usage with NXP's SDK:
 * \code{.c}
 * void myDelayFunction(uint32_t us)
 * {
 *     SDK_DelayAtLeastUs( us, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY );
 * }
 * \endcode
 */
typedef void (*MX25R1635F_DelayFunction_t)(uint32_t us);

/*!
 * \brief Structure representing an MX25R1635F handle instance.
 * This structure encapsulates the peripheral and transfer function, along with
 * transmit and receive buffers, for managing interactions with an MX25R1635F
 * device.
 * \param[in] peripheral Pointer to the peripheral instance used.
 * \param[in] tx_buffer Transmit buffer for SPI communication.
 * \param[in] rx_buffer Receive buffer for SPI communication.
 * \param[in] transferFunction User-defined function for performing SPI
 * transfers.
 * \param[in] delayFunction User-defined delay function.
 * \note Initialize the struct properly before using the driver's
 * functions.
 * \details The structure is designed to manage SPI communication
 * with an MX25R1635F device, providing buffers for transmit and receive
 * operations, and a user-defined function for performing SPI transfers. Example
 * usage:
 * \code{.c} MX25R1635F_Handle_t myHandle = { .peripheral =
 * &myPeripheralInstance, .tx_buffer = {0}, .rx_buffer = {0}, .transferFunction
 * = myTransferFunction, .delayFunction = myDelayFunction
 * };
 * \endcode
 */
typedef struct MX25R1635F_handle
{
  void                         *peripheral;
  unsigned char                 tx_buffer[MX25R1635F_INTERNAL_BUFFER_SIZE];
  unsigned char                 rx_buffer[MX25R1635F_INTERNAL_BUFFER_SIZE];
  MX25R1635F_TransferFunction_t transferFunction;
  MX25R1635F_DelayFunction_t    delayFunction;
} MX25R1635F_Handle_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

extern bool MX25R1635F_detect(MX25R1635F_Handle_t *const handle);

extern uint32_t MX25R1635F_readPage(MX25R1635F_Handle_t *const handle,
                                     size_t const               address,
                                     size_t const               dataSize);

extern uint32_t MX25R1635F_programPage(MX25R1635F_Handle_t *const handle,
                                        size_t const address, char *buffer,
                                        size_t *const dataSize);

extern uint32_t MX25R1635F_readSecurityRegisterRead(
  MX25R1635F_Handle_t *const handle, unsigned char *const value);

extern uint32_t MX25R1635F_writeSecurityRegisterRead(
  MX25R1635F_Handle_t *const handle, unsigned char *const value);

extern uint32_t MX25R1635F_sectorErase(MX25R1635F_Handle_t *const handle,
                                        size_t const               address);

extern bool MX25R1635F_WipReady(MX25R1635F_Handle_t *const handle);

extern uint32_t MX25R1635F_writeStatusRegister(
  MX25R1635F_Handle_t *const handle, unsigned char const *const value,
  size_t const dataSize);

/*!@}*/

#endif /* _MX25R1635F_H_ */
