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

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdbool.h>
#include "demo_spi_flash.h"
#include "fsl_clock.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "lpspi_api.h"
#include "MX25R1635F.h"
#include "TQ_utils.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define TRANSFER_BUFFER_SIZE (32)
#define INPUT_BUFFER_SIZE TRANSFER_BUFFER_SIZE

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*random address*/
static const char address = 0b111000;

/*CLI-Buffer*/
static char input[INPUT_BUFFER_SIZE];

static LPSPI_device_t master = {
  .baseAddr = SPI_ADDR,
};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static uint32_t MX25R1635F_transfer(void          *peripheral,
                                          unsigned char *tx_buffer,
                                          unsigned char *rx_buffer,
                                          size_t const   bufferSize);

static void                MX25R1635F_delayFunction(uint32_t us);
static MX25R1635F_Handle_t mx25 = {.peripheral = &master,
                                   .transferFunction =
                                     MX25R1635F_transfer,
                                   .delayFunction = MX25R1635F_delayFunction};

static void SpiTest(void);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * \brief  This is the main application entry point where system is initialized
 *         and all system components are started.
 */
void main(void)
{
  size_t   dataBufferSize = 0;
  status_t status;
  BOARD_Initialize();
  /*default config is right for the device*/
  LPSPI_MasterGetDefaultConfig(&master.masterConfig);
  master.masterConfig.baudRate = SPI_TRANSFER_BAUDRATE;
  LPSPI_MasterInit(master.baseAddr, &master.masterConfig, SPI_CLOCK_FREQUENCY);
  PRINTF("Please press any button.\r\n");
  GETCHAR();
  SpiTest();
  while (1)
  {
    PRINTF("Reading given address: \r\n");
    status =
      (status_t) MX25R1635F_readPage(&mx25, address, TRANSFER_BUFFER_SIZE);
    if (kStatus_Success != status)
    {
      PRINTF("An error ocurred while reading given address.\r\n");
    }
    PRINTF("%s \r\n", &mx25.rx_buffer[4]);
    PRINTF("Write something new into the MX25! Press enter if you want to "
           "skip. \r\n");
    while (!getline(input, TRANSFER_BUFFER_SIZE, &dataBufferSize))
    {
      PRINTF("Input Error. Please type in again.\r\n");
    }
    if (dataBufferSize != 0)
    {
      status = (status_t) MX25R1635F_sectorErase(&mx25, address);
      if (kStatus_Success != status)
      {
        PRINTF("An error ocurred while erasing given page.\r\n");
      }
      status = (status_t) MX25R1635F_programPage(&mx25, address, input,
                                                  &dataBufferSize);
      if (kStatus_Success != status)
      {
        PRINTF("An error ocurred while writing given page.\r\n");
      }
    }
  }
}

static void SpiTest(void)
{
  bool detected = MX25R1635F_detect(&mx25);
  if (detected)
  {
    PRINTF("MX25 detected.\r\n");
    PRINTF("%02x \r\n", mx25.rx_buffer[1]);
    PRINTF("%02x \r\n", mx25.rx_buffer[2]);
  }
  else
  {
    PRINTF("MX25 not detected or failed.\r\n");
  }
}

/*!
 * \brief Implementation of the MX25R1635F_lpspi_transfer.
 */
static uint32_t MX25R1635F_transfer(void          *peripheral,
                                          unsigned char *tx_buffer,
                                          unsigned char *rx_buffer,
                                          size_t const   bufferSize)
{
  status_t        status;
  LPSPI_device_t *spi_master      = (LPSPI_device_t *) peripheral;
  spi_master->masterXfer.txData   = tx_buffer;
  spi_master->masterXfer.rxData   = rx_buffer;
  spi_master->masterXfer.dataSize = bufferSize;
  spi_master->masterXfer.configFlags =
    kLPSPI_MasterPcsContinuous | kLPSPI_MasterPcs0 | kLPSPI_MasterByteSwap;
  status =
    LPSPI_MasterTransferBlocking(spi_master->baseAddr, &spi_master->masterXfer);
  return (uint32_t) status;
}

void MX25R1635F_delayFunction(uint32_t us)
{
  SDK_DelayAtLeastUs(us, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
}
