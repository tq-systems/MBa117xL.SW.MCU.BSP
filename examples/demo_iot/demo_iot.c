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

#include "board.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"
#include "pin_mux.h"
#include "demo_iot.h"
#include "iot.h"
#include "lpi2c_api.h"
#include "PCA9555BS.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "timers.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"
#include "lpuart_api.h"
#include "TQ_utils.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define uart_task_PRIORITY (configMAX_PRIORITIES - 1)
#define CLI_UART_BUFFER_SIZE 64

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void uart_task(void *);
static void iot_task(void *);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static char *to_send =
  "\r\nIOT-Device demonstation. Type help to get a list of commands.\r\n";
static char   *send_ring_overrun     = "\r\nRing buffer overrun!\r\n";
static char   *send_hardware_overrun = "\r\nHardware buffer overrun!\r\n";
static char   *send_timeout          = "\r\nTimeout expired!\r\n";
static uint8_t background_buffer[32];

static unsigned char rxBuffer[CLI_UART_BUFFER_SIZE];
static unsigned char txBuffer[CLI_UART_BUFFER_SIZE];

/* Task priorities. */
SemaphoreHandle_t   iot_sem;
static TaskHandle_t uart_task_handle;
static TaskHandle_t iot_task_handle;

static LPUART_Rtos_Peripheral_t rtos_uart = {
  .config.base                     = BOARD_DEBUG_UART_BASEADDR,
  .config.srcclk                   = BOARD_DEBUG_UART_CLK_FREQ,
  .config.baudrate                 = 115200,
  .config.parity                   = kLPUART_ParityDisabled,
  .config.stopbits                 = kLPUART_OneStopBit,
  .config.buffer                   = background_buffer,
  .config.buffer_size              = sizeof(background_buffer),
  .config.rx_timeout_constant_ms   = 0,
  .config.rx_timeout_multiplier_ms = 0,
  .config.tx_timeout_constant_ms   = 0,
  .config.tx_timeout_multiplier_ms = 0,
};

/* IOT_UART */
static LPUART_Peripheral_t main_uart_peripheral = {.base = MAIN_IOT_UART};
static LPUART_Peripheral_t gnss_uart_peripheral = {
  .base = GNSS_IOT_UART, .config.baudRate_Bps = 115200};

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * \brief  This is the main application entry point where system is initialized
 *         and all system components are started.
 */
int main(void)
{

  BOARD_Initialize();

  /* Initialize IOT peripherals */
  IOT_initMainUart(&main_uart_peripheral);
  IOT_initGNSSPeripheral(&gnss_uart_peripheral);

  PRINTF("This is a demonstration of the IOT-Device-operation.\r\n");
  NVIC_SetPriority(LPUART1_IRQn, 5);
  iot_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(iot_sem);
  if (xTaskCreate(uart_task, "Uart_task", configMINIMAL_STACK_SIZE + 100, NULL,
                  uart_task_PRIORITY, &uart_task_handle)
      != pdPASS)
  {
    PRINTF("Task creation failed!.\r\n");
    while (1)
      ;
  }
  if (xTaskCreate(iot_task, "IOT_task", configMINIMAL_STACK_SIZE + 100, NULL,
                  uart_task_PRIORITY, &iot_task_handle)
      != pdPASS)
  {
    PRINTF("Task creation failed!.\r\n");
    while (1)
      ;
  }

  vTaskStartScheduler();
  for (;;)
    ;
}

/*!
 * \brief Task responsible for the interaction between terminal and controller
 * through the uart.
 */
static void uart_task(void *)
{
  int    error;
  size_t n = 0;
  if (kStatus_Success
      != LPUART_RTOS_Init(&rtos_uart.handle, &rtos_uart.t_handle,
                          &rtos_uart.config))
  {
    vTaskSuspend(NULL);
  }

  /* Send introduction message. */
  if (kStatus_Success
      != LPUART_RTOS_Send(&rtos_uart.handle, (uint8_t *) to_send,
                          strlen(to_send)))
  {
    vTaskSuspend(NULL);
  }

  /* Receive user input and send it back to terminal. */
  do
  {
    if (iot_sem != NULL)
    {
      if (xSemaphoreTake(iot_sem, 10) == pdTRUE)
      {
        error = LPUART_RTOS_Receive(&rtos_uart.handle, rxBuffer, 1, &n);
        if (error == kStatus_LPUART_RxHardwareOverrun)
        {
          /* Notify about hardware buffer overrun */
          if (kStatus_Success
              != LPUART_RTOS_Send(&rtos_uart.handle,
                                  (uint8_t *) send_hardware_overrun,
                                  strlen(send_hardware_overrun)))
          {
            break;
          }
        }
        if (error == kStatus_LPUART_RxRingBufferOverrun)
        {
          /* Notify about ring buffer overrun */
          if (kStatus_Success
              != LPUART_RTOS_Send(&rtos_uart.handle,
                                  (uint8_t *) send_ring_overrun,
                                  strlen(send_ring_overrun)))
          {
            break;
          }
        }
        if (error == kStatus_Timeout)
        {
          /* Notify about Timeout */
          if (kStatus_Success
              != LPUART_RTOS_Send(&rtos_uart.handle, (uint8_t *) send_timeout,
                                  strlen(send_timeout)))
          {
            break;
          }
        }
        if (n > 0)
        {
          if (rtos_uart.bufferCount < CLI_UART_BUFFER_SIZE
              && rxBuffer[0] != '\r' && rxBuffer[0] != '\n')
          {
            txBuffer[rtos_uart.bufferCount] = rxBuffer[0];
            rtos_uart.bufferCount++;
          }
          /* send back the received data */
          if (kStatus_Success
              != LPUART_RTOS_Send(&rtos_uart.handle, rxBuffer, 1))
          {
            break;
          }
        }
        if (xSemaphoreGive(iot_sem) != pdTRUE)
        {
          break;
        }
        else
        {
          vTaskDelay(5);
        }
      }
    }
  } while ((kStatus_Success == error) || (kStatus_Busy == error));

  PRINTF("An error ocurred and all tasks will be terminated.");
  LPUART_RTOS_Deinit(&rtos_uart.handle);
  vTaskSuspend(iot_task_handle);
  vTaskSuspend(NULL);
}

/*!
 * \brief Task responsible for recognizing commands.
 */
static void iot_task(void *)
{
  status_t status = kStatus_Success;
  do
  {
    {
      if (rxBuffer[0] == '\r' || rxBuffer[0] == '\n')
      {
        if (iot_sem != NULL)
        {
          if (xSemaphoreTake(iot_sem, 10) == pdTRUE)
          {
            PRINTF("\r\n");
            if (strcmp((char *) txBuffer, "END") == 0)
            {
              unsigned char off[] = "AT+QPOWD";
              status =
                IOT_executeCommand(off, sizeof(off), &main_uart_peripheral);
            }
            else if (strcmp((char *) txBuffer, "START") == 0)
            {
              status = IOT_start(&main_uart_peripheral);
            }
            else if ((strcmp((char *) txBuffer, "GNSS INIT") == 0))
            {
              unsigned char cmd[] = "AT+QGPSCFG= "
                                    " outport "
                                    ","
                                    " uartnmea "
                                    "";
              status =
                IOT_executeCommand(cmd, sizeof(cmd), &main_uart_peripheral);
              unsigned char gnss[] = "AT+QGPS=1,1";
              status =
                IOT_executeCommand(gnss, sizeof(gnss), &main_uart_peripheral);
              unsigned char test[] = "AT+QGPSLOC=?";
              status =
                IOT_executeCommand(test, sizeof(test), &main_uart_peripheral);
            }
            else if ((strcmp((char *) txBuffer, "GNSS READ") == 0))
            {
              status = IOT_read(&gnss_uart_peripheral);
            }

            else if (strcmp((char *) txBuffer, "help") == 0)
            {
              PRINTF("Command: 'START' - starts IOT device\r\n");
              PRINTF("Command: 'END' - shut down IOT device\r\n");
              PRINTF("Command: 'GNSS INIT' - Initializes GPS\r\n");
              PRINTF("Command: 'GNSS READ' - Reads GNSS output\r\n");
              PRINTF(
                "For any other specific commands refer to datasheet of IOT "
                "device from "
                "Quectel.\r\n");
              PRINTF("Mind that in order to read data from GNSS you need to "
                     "initialize it via command.\r\n ");
            }
            else
            {
              status = IOT_executeCommand(txBuffer, rtos_uart.bufferCount,
                                          &main_uart_peripheral);
            }
            rtos_uart.bufferCount = 0;
            memset(txBuffer, 0, sizeof(txBuffer));
            memset(rxBuffer, 0, sizeof(rxBuffer));
            PRINTF("Command was sent. Verify execution. Type in any command "
                   "or 'help'.\r\n");
            if (xSemaphoreGive(iot_sem) != pdTRUE)
            {
              vSemaphoreDelete(iot_sem);
              break;
            }
          }
        }
      }
    }
  } while (status == kStatus_Success);
  vSemaphoreDelete(iot_sem);
}
