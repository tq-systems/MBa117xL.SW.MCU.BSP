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

#include "demo_iot.h"
#include "iot.h"
#include "FreeRTOS.h"
#include "timers.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define IOT_UART_BUFFER_SIZE 1024
#define IOT_COMMAND_BUFFER_SIZE 256
#define GPIO_BG95_PWR_ON GPIO8

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static uint32_t PortExpandertransferFunction(
  void *peripheral, uint32_t regAddress, size_t regAddressSize,
  PCA9555BS_TransferDirection_t transferDirection, uint8_t *buffer,
  size_t bufferSize);
static void IOT_powerOn(void);

static void LPUART_UserCallback(LPUART_Type *base, lpuart_handle_t *handle,
                                status_t status, void *userData);

static void timerCallback(TimerHandle_t xTimerID);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static volatile bool     rxOnGoing = false;
static SemaphoreHandle_t transfer_sem;

static unsigned char rxBuffer[IOT_UART_BUFFER_SIZE];
static uint8_t       commandBuffer[IOT_COMMAND_BUFFER_SIZE];

/*Initialize peripheral*/
static LPI2C_masterDevice_t i2c_master = {
  .base_addr                 = LPI2C_BASE_ADDR,
  .masterConfig.baudRate_Hz  = LPI2C_BITRATE,
  .mode                      = LPI2C_BlockingMode,
  .masterXfer.flags          = kLPI2C_TransferDefaultFlag,
  .masterXfer.slaveAddress   = PORTEXP_I2C_DEVICE_ADR,
  .masterXfer.subaddressSize = 1,
  .masterXfer.dataSize       = 2};

/*Portexpander for enabling IOT_MAIN_UART*/
/*device configuration*/
static PCA9555BS_Handle_t portExpander = {.peripheral = &i2c_master,
                                          .IOMask.PIN00_PIN07_Mask       = 0xF0,
                                          .IOMask.PIN10_PIN17_Mask       = 0x00,
                                          .PolarityMask.PIN00_PIN07_Mask = 0x00,
                                          .PolarityMask.PIN10_PIN17_Mask = 0x00,
                                          .OutputMask.PIN10_PIN17_Mask   = 0x00,
                                          .transferFunction =
                                            PortExpandertransferFunction};

/*******************************************************************************
 * Callback functions
 ******************************************************************************/

/*!
 * \brief generic callback function for uart
 */
static void LPUART_UserCallback(LPUART_Type *base, lpuart_handle_t *handle,
                                status_t status, void *userData)
{
  (void) base;
  (void) handle;
  (void) userData;
  if (kStatus_LPUART_RxIdle == status)
  {
    rxOnGoing = false;
    xSemaphoreGiveFromISR(transfer_sem, NULL);
  }
}

/*!
 * \brief timer callback function
 */
static void timerCallback(TimerHandle_t xTimerID)
{
  (void) xTimerID;
  xSemaphoreGiveFromISR(transfer_sem, NULL);
}

/*******************************************************************************
 *  Code
 ******************************************************************************/

/*!
 * \brief Sends command to IOT device and awaits it's response.
 * \param commandString Array containing the command string.
 * \param stringSize Size of the array containing the command string.
 * \param peripheral Peripheral responsible for the communication.
 */
status_t IOT_executeCommand(unsigned char *commandString, size_t stringSize,
                            LPUART_Peripheral_t *peripheral)
{
  status_t status = kStatus_Fail;
  if ((!rxOnGoing))
  {
    if (stringSize > IOT_COMMAND_BUFFER_SIZE)
    {
      return kStatus_Fail;
    }
    memcpy(commandBuffer, commandString, stringSize);
    strcat((char *) commandBuffer, "\r");
    stringSize++;
    status = LPUART_WriteBlocking(peripheral->base, commandBuffer, stringSize);
    if (status != kStatus_Success)
    {
      return status;
    }
    return IOT_read(peripheral);
  }
  return status;
}

/*!
 * \brief Reads from any Uart peripheral.
 * \param peripheral Peripheral responsible for the communication.
 */
status_t IOT_read(LPUART_Peripheral_t *peripheral)
{
  status_t      status = kStatus_Fail;
  TimerHandle_t timer  = NULL;
  if ((!rxOnGoing))
  {
    if (transfer_sem == NULL)
    {
      transfer_sem = xSemaphoreCreateBinary();
      if (xSemaphoreGive(transfer_sem) != pdTRUE)
      {
        return kStatus_Fail;
      }
    }
    timer =
      xTimerCreate("UART_Timer", pdMS_TO_TICKS(100), pdFALSE, 0, timerCallback);
    if (timer == NULL)
    {
      // Failed to create the timer
      vSemaphoreDelete(transfer_sem);
      return kStatus_Fail;
    }
    rxOnGoing                     = true;
    peripheral->transfer.data     = rxBuffer;
    peripheral->transfer.dataSize = IOT_UART_BUFFER_SIZE;
    status                        = LPUART_TransferReceiveNonBlocking(
      peripheral->base, &peripheral->handle, &peripheral->transfer, NULL);
    if (xSemaphoreTake(transfer_sem, 10) == pdFAIL)
    {
      vSemaphoreDelete(transfer_sem);
      return kStatus_Fail;
    }
    if (xTimerStart(timer, 0) != pdPASS)
    {
      // Failed to start the timer
      vSemaphoreDelete(transfer_sem);
      return kStatus_Fail;
    }
    xSemaphoreTake(transfer_sem, portMAX_DELAY);
    LPUART_TransferAbortReceive(peripheral->base, &peripheral->handle);
    rxOnGoing = false;
    if (timer != NULL)
    {
      if (xTimerDelete(timer, 0) == pdFAIL)
      {
        vSemaphoreDelete(transfer_sem);
        return kStatus_Fail;
      }
      timer = NULL;
    }
    PRINTF("%s", rxBuffer);
    PRINTF("\r\n");
    memset(rxBuffer, '\0', IOT_UART_BUFFER_SIZE);
    if (xSemaphoreGive(transfer_sem) != pdTRUE)
    {
      status = kStatus_Fail;
    }
  }
  return status;
}

/*!
 * \brief Initializes peripheral for IOT communication on the Board MBa117xL.
 * \param peripheral Peripheral responsible for the communication.
 * \note For the BG95-M4 this is the main communication peripheral.
 */
void IOT_initMainUart(LPUART_Peripheral_t *peripheral)
{
  /*INIT UART PERIPHERAL*/
  LPUART_GetDefaultConfig(&peripheral->config);
  peripheral->config.enableTx = true;
  peripheral->config.enableRx = true;
  LPUART_Init(peripheral->base, &peripheral->config,
              CLOCK_GetRootClockFreq(kCLOCK_Root_Lpuart6));
  LPUART_TransferCreateHandle(peripheral->base, &peripheral->handle,
                              LPUART_UserCallback, NULL);

  /*Init main_uart_peripheral*/
  LPI2C_MasterGetDefaultConfig(&i2c_master.masterConfig);
  i2c_master.masterConfig.baudRate_Hz = LPI2C_BITRATE;
  LPI2C_MasterInit(i2c_master.base_addr, &i2c_master.masterConfig,
                   LPI2C_CLOCK_FREQUENCY);
  /*CONFIGURE PORTEXPANDER*/
  PCA9555BS_configure(&portExpander);

  SDK_DelayAtLeastUs(20000, SystemCoreClock);
}

/*!
 * \brief Send command or initialize GNSS.
 * \param peripheral Peripheral responsible for the communication.
 * \note For the BG95-M4 this is the GNSS communication peripheral.
 */
void IOT_initGNSSPeripheral(LPUART_Peripheral_t *peripheral)
{
  LPUART_GetDefaultConfig(&peripheral->config);
  peripheral->config.enableTx = true;
  peripheral->config.enableRx = true;
  status_t status = LPUART_Init(peripheral->base, &peripheral->config,
                                CLOCK_GetRootClockFreq(kCLOCK_Root_Lpuart11));
  LPUART_EnableInterrupts(peripheral->base,
                          kLPUART_RxDataRegFullInterruptEnable);
  EnableIRQ(LPUART11_IRQn);
  LPUART_TransferCreateHandle(peripheral->base, &peripheral->handle,
                              LPUART_UserCallback, NULL);
  if (kStatus_Success == status)
  {
    PRINTF("[\033[1;33mINFO\033[0m] GNSS peripheral initialized.\r\n");
  }
  else
  {
    PRINTF(
      "[\033[1;33mINFO\033[0m] GNSS peripheral initialization failed.\r\n");
  }
}

/*!
 * \brief Powers on IOT device.
 */
static void IOT_powerOn(void)
{
  /*GPIO BG95_PWRKEY*/
  gpio_pin_config_t BG95_PWRKEY_config = {.direction     = kGPIO_DigitalOutput,
                                          .outputLogic   = 0U,
                                          .interruptMode = kGPIO_NoIntmode};
  GPIO_PinInit(GPIO_BG95_PWR_ON, 12U, &BG95_PWRKEY_config);

  PRINTF("[ OK ]\tToggle BG95_PWR\r\n");

  // PWRKEY Toggle
  GPIO_PinWrite(GPIO_BG95_PWR_ON, 12U, 1U);

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  GPIO_PinWrite(GPIO_BG95_PWR_ON, 12U, 0U);

  PRINTF("\r\n[\033[1;33mINFO\033[0m] Device is on.\r\n");
}

/*!
 * \brief Turns on the IOT device.
 * \param peripheral Peripheral responsible for the communication.
 * \note For the BG95-M4 this is the main communication peripheral.
 */
status_t IOT_start(LPUART_Peripheral_t *peripheral)
{
  uint8_t  rxCnt            = 0;
  uint8_t  ATCommand_ATI[]  = "ATI\r";
  uint8_t  ATCommand_IMEI[] = "AT+GSN\r";
  status_t status;

  IOT_powerOn();

  /*Wait for Module Ready Message*/
  while (rxCnt < 18)
  {
    LPUART_ClearStatusFlags(peripheral->base, kLPUART_AllClearFlags);
    status = LPUART_ReadBlocking(peripheral->base, rxBuffer, 1);
    if (status == kStatus_Success)
    {
      PRINTF("%s", rxBuffer);
      rxCnt += 1;
    }
  }

  /*ATI Command lists further module information*/
  status = IOT_executeCommand(ATCommand_ATI, sizeof(ATCommand_ATI), peripheral);
  if (status != kStatus_Success)
  {
    return status;
  }
  vTaskDelay(100);
  status =
    IOT_executeCommand(ATCommand_IMEI, sizeof(ATCommand_IMEI), peripheral);
  PRINTF(
    "\r\n[\033[1;33mINFO\033[0m]  BG95-M4 is now ready for operation.\r\n");
  return status;
}

/*!
 * \brief This is the transfer function that is used by the driver.
 * \note Parameters are described in the drivers transfer function typedef.
 */
static uint32_t PortExpandertransferFunction(
  void *peripheral, uint32_t regAddress, size_t regAddressSize,
  PCA9555BS_TransferDirection_t transferDirection, uint8_t *buffer,
  size_t bufferSize)
{
  status_t status = kStatus_LPUART_Error;
#ifdef USE_LPI2C
  LPI2C_masterDevice_t *master      = (LPI2C_masterDevice_t *) peripheral;
  master->masterXfer.data           = buffer;
  master->masterXfer.dataSize       = bufferSize;
  master->masterXfer.subaddress     = regAddress;
  master->masterXfer.subaddressSize = regAddressSize;
  switch (transferDirection)
  {
  case PCA9555BS_WRITE:
    master->masterXfer.direction = kLPI2C_Write;
    status =
      LPI2C_MasterTransferBlocking(master->base_addr, &master->masterXfer);
    break;
  case PCA9555BS_READ:
    master->masterXfer.direction = kLPI2C_Read;
    status =
      LPI2C_MasterTransferBlocking(master->base_addr, &master->masterXfer);
    break;
  default:
    return kStatus_LPUART_Error;
  }
#else
#error "only LPI2C supported"
#endif

  return (uint32_t) status;
}
