//******************************************************************************
/*!
 * \copyright
 * SPDX-License-Identifier: BSD-3-Clause
 * \copyright
 * Copyright (c) 2021 - 2023 TQ-Systems GmbH <license@tq-group.com>,
 * D-82229 Seefeld, Germany.
 * Author: Isaac L. L. Yuki
 *
 * \brief Driver for the PCA9555BS port expander.
 */
//******************************************************************************

/*!
 * \defgroup PCA9555BS PCA9555BS
 * \brief Driver for the PCA9555BS port expander.
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
 * Once initialization is complete, call the "PCA9555BS_configure" function to
 * set your desired configuration set in the handle structure. For further
 * operation consult the documentation in the functions.
 */

#ifndef PCA9555BS_H_
#define PCA9555BS_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>
#include <stdlib.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*
 * PortExpander Registers 8-bit
 * Register Order:
 * 0: Input Port0
 * 1: Input Port1
 * 2: Output Port0
 * 3: Output Port1
 * 4: Polarity Inversion Port0
 * 5: Polarity Inversion Port1
 * 6: Configuration Port0
 * 7: Configuration Port1
 */
/*Group1*/
/*Reflects the incoming logic levels of the pins, regardless of pin in/out*/
/*Input Port 0 Register*/
#define PORTEXP_INPUTPORT0_REG 0x00
/*Input Port 1 Register*/
#define PORTEXP_INPUTPORT1_REG 0x01

/*Group2*/
/*Output Only Port, Reflects outgoing logic levels, No Effect on Input Pins
 * Read of the Register reprensent FlipFlop controlling value, not actual pin
 * value
 */
/*Output Port 0 Register*/
#define PORTEXP_OUTPUTPORT0_REG 0x02
/*Output Port 1 Register*/
#define PORTEXP_OUTPUTPORT1_REG 0x03

/*Group3*/
/*Allows to invert polartiy of input port register data
 * Bit = 1 --> Invertion of input port data polarity
 * Bit = 0 --> Input port data polarity is retained
 */
/*Polarity Inversion Port 0 Register*/
#define PORTEXP_POLINVPORT0_REG 0x04
/*Polarity Inversion Port 1 Register*/
#define PORTEXP_POLINVPORT1_REG 0x05

/*Group4*/
/*Directions of I/O Pins
 * Bit = 1 --> Port Pin is Input with High-Impedance output driver
 * Bit = 0 --> Port Pin is Output
 *
 * Reset --> All Port Pins are inputs with pull-ups to VDD
 */
/*Configuration Port 0 Register*/
#define PORTEXP_CONFIGPORT0_REG 0x06
/*Configuration Port 1 Register*/
#define PORTEXP_CONFIGPORT1_REG 0x07

#define PORTEXP_DATA0_REG 0x02
#define PORTEXP_DATA1_REG 0x03

/*I2C Transfer Buffer Size*/
#define PORTEXP_BUFFER_SIZE 3

/*!
 * \brief Configuration for I/O pins of PCA9555BS.
 * This structure holds the masks used to configure the I/O settings
 * for each 8-bit port of the PCA9555BS.
 */
typedef struct PCA9555BS_PIN_IOConfig
{
  uint8_t PIN00_PIN07_Mask; /**< Mask for configuring pins P0_0 to P0_7.*/
  uint8_t PIN10_PIN17_Mask; /**< Mask for configuring pins P1_0 to P1_7.*/
} PCA9555BS_PIN_IOConfig_t;

/*!
 * \brief Configuration for polarity of PCA9555BS pins.
 * This structure holds the masks used to configure the polarity settings
 * for each 8-bit port of the PCA9555BS.
 */
typedef struct PCA9555BS_PIN_Polarity
{
  uint8_t
    PIN00_PIN07_Mask; /**< Mask for configuring polarity of pins P0_0 to P0_7.*/
  uint8_t
    PIN10_PIN17_Mask; /**< Mask for configuring polarity of pins P1_0 to P1_7.*/
} PCA9555BS_PIN_Polarity_t;

/*!
 * \brief Enumerates the possible directions for data transfer with the
 * PCF85063ATL RTC.
 */
typedef enum PCA9555BS_TransferDirection
{
  PCA9555BS_READ, /**< Indicates a read operation from the RTC.*/
  PCA9555BS_WRITE /**< Indicates a write operation to the RTC.*/
} PCA9555BS_TransferDirection_t;

/*!
 * \brief Type definition for I2C transfer function.
 * This type defines a function pointer for I2C transfers with the PCA9555BS.
 * The function should perform an I2C transfer based on the provided parameters.
 * \param peripheral Pointer to the I2C peripheral device structure.
 * \param regAddress The register address to read from or write to.
 * \param regAddressSize The size of the register address in bytes.
 * \param transferDirection Specifies whether the operation is a read or write.
 * \param buffer Pointer to the data buffer for read or write.
 * \param bufferSize The size of the data buffer in bytes.
 * \return Returns a status code indicating the outcome of the transfer
 * operation.
 * \note The function pointed to by this function pointer type is expected to
 * handle data transfer (both read and write operations) with the PCA9555BS
 * RTC via I2C communication. The return value should be adapted to the driver
 * used for the peripheral, where 0 represents a successful operation. It is
 * crucial to ensure that the logic within the transfer function accurately
 * evaluates and executes read and write operations, and that it is compatible
 * with the error values used by this driver. Detailed documentation on the
 * error values and their meanings can be found in the documentation for the
 * driver's functions.
 */
typedef uint32_t (*PCA9555BS_TransferFunction_t)(
  void *peripheral, uint32_t regAddress, size_t regAddressSize,
  PCA9555BS_TransferDirection_t transferDirection, uint8_t *buffer,
  size_t bufferSize);

/*!
 * \brief handle structure for the PCA9555BS port expander driver.
 * \note This structure is used to configure the PCA9555BS port expander's
 * pin masks for I/O and polarity configurations.
 * \note  Configuring all parameters here is indispensable for using this
 * driver!
 */
typedef struct PCA9555BS_Handle
{
  void *peripheral; /**< Pointer to the I2C peripheral object.*/
  PCA9555BS_PIN_IOConfig_t
    IO_Mask; /**< Mask used for configuring PINs as input or output.
              * - Bit = 1: Corresponding pin is configured as input.
              * - Bit = 0: Corresponding pin is configured as output.
              * Example: 0b00001111 will configure P0_0 to P0_3 as outputs
              * and P0_4 to P0_7 as inputs.*/
  PCA9555BS_PIN_Polarity_t
    Polarity_Mask; /**< Mask used for configuring PINs polarity.
                    * - Bit = 1: Inverts the polarity of the corresponding
                    * input pin.
                    * - Bit = 0: Retains the original polarity of the
                    * corresponding input pin. Example: 0b00000001 will invert
                    * the polarity of P0_0 and retain the polarity for other
                    * pins.*/
  PCA9555BS_TransferFunction_t
    transferFunction; /**< Callback function for handling data transfer
                       * operations with the port expander.*/
} PCA9555BS_Handle_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

extern uint32_t PCA9555BS_configure(PCA9555BS_Handle_t *handle);

extern uint32_t PCA9555BS_configPolarity(PCA9555BS_Handle_t *handle);

extern uint32_t PCA9555BS_configPortsIO(PCA9555BS_Handle_t *handle);

extern uint32_t PCA9555BS_writeRegister(uint8_t registerAddress, uint8_t *value,
                                        PCA9555BS_Handle_t *handle);

extern uint32_t PCA9555BS_readRegister(uint8_t registerAddress, uint8_t *buffer,
                                       PCA9555BS_Handle_t *handle);

/*!@}*/

#endif /* _PCA9555BS_H_ */
