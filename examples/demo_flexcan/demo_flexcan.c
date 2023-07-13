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
#include "pin_mux.h"
#include "demo_flexcan.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "flexcan_api.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define RX_MESSAGE_BUFFER_NUM (9)
#define TX_MESSAGE_BUFFER_NUM (8)
#define DLC (8)

typedef void     (*flexcan_callback_t)(CAN_Type *base, flexcan_handle_t *handle,
                                   status_t status, uint64_t result,
                                   void *userData);
FlexCAN_device_t flexcan1 = {.base_addr     = FLEXCAN1_BASE_ADDR,
                             .identifier.tx = 0x321u,
                             .identifier.rx = 0x123u,
                             .srcClockRoot  = FLEXCAN1_CLOCK_ROOT};

FlexCAN_device_t flexcan2 = {.base_addr     = FLEXCAN2_BASE_ADDR,
                             .identifier.tx = 0x123u,
                             .identifier.rx = 0x321u,
                             .srcClockRoot  = FLEXCAN2_CLOCK_ROOT};

static volatile bool txComplete;
static volatile bool rxComplete;
static volatile bool wakenUp;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void FLEXCAN_send(uint8_t msg[8], FlexCAN_device_t *device);
static void FLEXCAN_listen(FlexCAN_device_t *device);
static void FLEXCAN_awaitReceive(FlexCAN_device_t *device);
static void flexcan_callback(CAN_Type *base, flexcan_handle_t *handle,
                             status_t status, uint64_t result, void *userData);
static void FLEXCAN_configure(FlexCAN_device_t           *device,
                              flexcan_transfer_callback_t flexcan_callback);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static uint8_t buffer[8] = {1};

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
  FLEXCAN_configure(&flexcan1, flexcan_callback);
  FLEXCAN_configure(&flexcan2, flexcan_callback);
  char input;

  PRINTF("This demo shows the flexcan devices communication.\r\n");
  PRINTF("Please press anything to initialize:\r\n");
  GETCHAR();
  PRINTF("Started.. \r\n");
  for (uint8_t cnt = 0; cnt < 50; cnt++)
  {
    FLEXCAN_listen(&flexcan1);
    FLEXCAN_send(buffer, &flexcan2);
    FLEXCAN_awaitReceive(&flexcan1);
    buffer[0] = 1 + flexcan1.rxXfer.frame->dataByte0;
    FLEXCAN_listen(&flexcan2);
    FLEXCAN_send(buffer, &flexcan1);
    FLEXCAN_awaitReceive(&flexcan2);
    buffer[0] = 1 + flexcan2.rxXfer.frame->dataByte0;
  }
  PRINTF("Main demonstration done! \r\n");
  PRINTF("Type in anything you want! \r\n");
  while (1)
  {
    for (uint8_t i = 0; i < 8; i++)
    {
      input = GETCHAR();
      if ((input == '\n') || (input == '\r'))
      {
        break;
      }
      buffer[i] = input;
    }
    FLEXCAN_listen(&flexcan1);
    FLEXCAN_send(buffer, &flexcan2);
    FLEXCAN_awaitReceive(&flexcan1);
    for (uint8_t i = 0; i < 8; i++)
    {
      PRINTF("%c", buffer[i]);
    }
    PRINTF("\r\n");
    for (uint8_t i = 0; i < 8; i++)
    {
      input = GETCHAR();
      if ((input == '\n') || (input == '\r'))
      {
        break;
      }
      buffer[i] = input;
    }
    FLEXCAN_listen(&flexcan2);
    FLEXCAN_send(buffer, &flexcan1);
    FLEXCAN_awaitReceive(&flexcan2);
    for (uint8_t i = 0; i < 8; i++)
    {
      PRINTF("%c", buffer[i]);
    }
    PRINTF("\r\n");
  }
  return 0;
}

static void flexcan_callback(CAN_Type *base, flexcan_handle_t *handle,
                             status_t status, uint64_t result, void *userData)
{
  (void) base;
  (void) handle;
  (void) userData;
  (void) result;
  switch (status)
  {
  case kStatus_FLEXCAN_RxIdle:
    rxComplete = true;
    break;

  case kStatus_FLEXCAN_TxIdle:
    txComplete = true;
    break;

  case kStatus_FLEXCAN_WakeUp:
    wakenUp = true;
    break;
  default:
    break;
  }
}

/*!
 * \brief  Sends data over FlexCAN device.
 * \param msg The message being sent.
 * \param device The FlexCAN device being used.
 */
static void FLEXCAN_send(uint8_t msg[8], FlexCAN_device_t *device)
{
  memset(device->txXfer.frame, 0, sizeof(device->txXfer.frame));
  const uint32_t ident = device->identifier.tx;

  device->frame.id     = FLEXCAN_ID_STD(ident);
  device->frame.format = (uint8_t) kFLEXCAN_FrameFormatStandard;
  device->frame.type   = (uint8_t) kFLEXCAN_FrameTypeData;
  device->frame.length = (uint8_t) DLC;

  device->frame.dataByte0 = msg[0];
  device->frame.dataByte1 = msg[1];
  device->frame.dataByte2 = msg[2];
  device->frame.dataByte3 = msg[3];
  device->frame.dataByte4 = msg[4];
  device->frame.dataByte5 = msg[5];
  device->frame.dataByte6 = msg[6];
  device->frame.dataByte7 = msg[7];
  device->txXfer.mbIdx    = TX_MESSAGE_BUFFER_NUM;
  device->txXfer.frame    = &device->frame;
  FLEXCAN_TransferSendNonBlocking(device->base_addr, &device->handle,
                                  &device->txXfer);
  while (!(txComplete))
  {
  };
  txComplete = false;
}

/*!
 * \brief Listens for incoming messages.
 * \param device The FlexCAN device being used.
 */
static void FLEXCAN_listen(FlexCAN_device_t *device)
{
  memset(device->rxXfer.frame, 0, sizeof(device->rxXfer.frame));

  const uint32_t ident = device->identifier.rx;
  device->frame.id     = FLEXCAN_ID_STD(ident);
  device->frame.format = (uint8_t) kFLEXCAN_FrameFormatStandard;
  device->frame.type   = (uint8_t) kFLEXCAN_FrameTypeData;
  device->frame.length = (uint8_t) DLC;

  device->rxXfer.mbIdx = RX_MESSAGE_BUFFER_NUM;
  device->rxXfer.frame = &device->frame;
  FLEXCAN_TransferReceiveNonBlocking(device->base_addr, &device->handle,
                                     &device->rxXfer);
}

/*!
 * \brief  This function waits for confirmation of the recipient
 * for receiving the sent bytes.
 * \param device The FlexCAN device being used.
 */
static void FLEXCAN_awaitReceive(FlexCAN_device_t *device)
{
  while (!rxComplete)
  {
  };
  rxComplete = false;
  PRINTF("Rx MB ID: 0x%3x, Rx MB data: %i, Time stamp: %d\r\n",
         device->frame.id >> CAN_ID_STD_SHIFT, device->frame.dataByte0,
         device->frame.timestamp);
  buffer[0] = device->rxXfer.frame->dataByte0;
  buffer[1] = device->rxXfer.frame->dataByte1;
  buffer[2] = device->rxXfer.frame->dataByte2;
  buffer[3] = device->rxXfer.frame->dataByte3;
  buffer[4] = device->rxXfer.frame->dataByte4;
  buffer[5] = device->rxXfer.frame->dataByte5;
  buffer[6] = device->rxXfer.frame->dataByte6;
  buffer[7] = device->rxXfer.frame->dataByte7;
}

static void FLEXCAN_configure(FlexCAN_device_t           *device,
                              flexcan_transfer_callback_t callback)
{
  const uint32_t ident = device->identifier.rx;
  FLEXCAN_GetDefaultConfig(&device->config);
  device->config.enableLoopBack = false;
  FLEXCAN_Init(device->base_addr, &device->config,
               CLOCK_GetRootClockFreq(device->srcClockRoot));
  FLEXCAN_TransferCreateHandle(device->base_addr, &device->handle, callback,
                               (void *) device);
  FLEXCAN_SetRxMbGlobalMask(
    device->base_addr, FLEXCAN_RX_MB_STD_MASK(device->identifier.rx, 0, 0));
  device->mbConfig.id     = FLEXCAN_ID_STD(ident),
  device->mbConfig.type   = kFLEXCAN_FrameTypeData,
  device->mbConfig.format = kFLEXCAN_FrameFormatStandard;
  FLEXCAN_SetTxMbConfig(device->base_addr, TX_MESSAGE_BUFFER_NUM, true);
  FLEXCAN_SetRxMbConfig(device->base_addr, RX_MESSAGE_BUFFER_NUM,
                        &device->mbConfig, true);
}
