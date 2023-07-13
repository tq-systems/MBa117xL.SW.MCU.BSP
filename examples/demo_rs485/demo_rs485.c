//*****************************************************************************
/*!
 * \copyright
 * SPDX-License-Identifier: BSD-3-Clause
 * \copyright
 * Copyright (c) 2021 - 2023 TQ-Systems GmbH <license@tq-group.com>,
 * D-82229 Seefeld, Germany.
 * Author: Isaac L. L. Yuki
 */
//******************************************************************************

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <string.h>

#include "board.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_lpuart.h"
#include "pin_mux.h"
#include "demo_rs485.h"
#include "lpuart_api.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define RS485_RX_BUFFER_SIZE 50
#define RS485_TX_BUFFER_SIZE 50

/*******************************************************************************
 * Variables
 ******************************************************************************/

static LPUART_Peripheral_t uart = {.base = LPUART_RS485_PERIPHERAL};

/*Global flags for NonBlocking transfer operation*/
static volatile bool              rxOnGoing     = false;
static volatile long unsigned int receivedBytes = 0;

static uint8_t txBuffer[RS485_TX_BUFFER_SIZE];
static uint8_t rxBuffer[RS485_RX_BUFFER_SIZE];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void LPUART_callback(LPUART_Type *base, lpuart_handle_t *handle,
                            status_t status, void *userData);
static void RS485_write(LPUART_Peripheral_t *peripheral);
static void RS485_read(LPUART_Peripheral_t *peripheral);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * \brief  This is the main application entry point where system is
 * initialized and all system components are started.
 */
int main(void)
{
  gpio_pin_config_t gpio_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

  BOARD_Initialize();

  LPUART_GetDefaultConfig(&uart.config);
  uart.config.baudRate_Bps = RS485_UART_BAUDRATE;
  uart.config.enableTx     = true;
  uart.config.enableRx     = true;
  LPUART_Init(uart.base, &uart.config, LPUART_RS485_PERIPHERAL_CLOCK_ROOT);
  LPUART_TransferCreateHandle(uart.base, &uart.handle, LPUART_callback, NULL);

  /* Pin init */
  GPIO_PinInit(BOARD_INITPINS_BT_PD_GPIO, BOARD_INITPINS_BT_PD_GPIO_PIN,
               &gpio_config);
  GPIO_PinInit(RS485_TRAN_EN_PORT, RS485_TRAN_EN_PIN, &gpio_config);

  /*Enable UART8 for RS485 Transfer at MUX*/
  GPIO_PinWrite(BOARD_INITPINS_BT_PD_GPIO, BOARD_INITPINS_BT_PD_GPIO_PIN, 0);

  PRINTF("This is a demonstration of the RS485-Bus\r\n");
  /*Enable Transmission Write of RS485 (Flow Control)*/
  GPIO_PinWrite(RS485_TRAN_EN_PORT, RS485_TRAN_EN_PIN, 1);
  strcpy((char *) txBuffer, "Hello! Type something! Press enter to send! \r\n");
  RS485_write(&uart);
  /*Enable Read of RS485 (Flow Control)*/
  GPIO_PinWrite(RS485_TRAN_EN_PORT, RS485_TRAN_EN_PIN, 0);
  RS485_read(&uart);

  PRINTF("Demonstration end. \r\n");
  while (1)
    ;
}

/*!
 * \brief RS485_Write
 * \param peripheral LPUART peripheral representation
 */
static void RS485_write(LPUART_Peripheral_t *peripheral)
{
  status_t status = kStatus_NoTransferInProgress;

  status =
    LPUART_WriteBlocking(peripheral->base, txBuffer, RS485_TX_BUFFER_SIZE);

  /*Check Flag tx_rs485_onGoing set from Callback function*/
  if (status == kStatus_Success)
  {
    PRINTF("Successful Write of RS485: %s\r\n", txBuffer);
  }
  else
  {
    PRINTF("Error Write of RS485\r\n");
  }
}

/*!
 * \brief RS485_Read
 * \param peripheral LPUART peripheral representation
 */
static void RS485_read(LPUART_Peripheral_t *peripheral)
{
  status_t status               = kStatus_NoTransferInProgress;
  peripheral->transfer.rxData   = rxBuffer;
  peripheral->transfer.dataSize = RS485_RX_BUFFER_SIZE - 1;
  rxOnGoing                     = true;
  status                        = LPUART_TransferReceiveNonBlocking(
    peripheral->base, &peripheral->handle, &peripheral->transfer, NULL);
  if (kStatus_Success != status)
  {
    PRINTF("Transfer failed");
    return;
  }
  PRINTF("Wating for input... \r\n \r\n");
  while (rxOnGoing)
  {
  }

  /* Print read data on console */
  if (receivedBytes > 0)
  {
    PRINTF("Successful Read of RS485: %s\r\n", rxBuffer);
  }
  else
  {
    PRINTF("No data occurred of RS485 Read\r\n");
  }
  LPUART_TransferAbortReceive(peripheral->base, &peripheral->handle);
}

/* LPUART user callback */
static void LPUART_callback(LPUART_Type *base, lpuart_handle_t *handle,
                            status_t status, void *userData)
{
  (void) userData;
  status_t          error;
  long unsigned int n = receivedBytes;
  error               = LPUART_TransferGetReceiveCount(base, handle, &n);
  receivedBytes       = n;
  if (((kStatus_LPUART_RxIdle == status)
       || ((rxBuffer[receivedBytes - 1] == '\n')
           || (rxBuffer[receivedBytes - 1] == '\r')))
      && kStatus_Success == error)
  {
    rxBuffer[receivedBytes - 1] = '\0';
    rxOnGoing                   = false;
  }
  else if (kStatus_LPUART_Error == status || kStatus_Success != error)
  {
    rxOnGoing     = false;
    receivedBytes = 0;
    PRINTF("An error ocurred during transfer.\r\n");
  }
}
