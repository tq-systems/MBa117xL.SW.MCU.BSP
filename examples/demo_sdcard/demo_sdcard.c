//*****************************************************************************
/*!
 * \copyright
 * SPDX-License-Identifier: BSD-3-Clause
 * \copyright
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 * \copyright
 * Copyright (c) 2021 - 2023 TQ-Systems GmbH <license@tq-group.com>,
 * D-82229 Seefeld, Germany.
 * Author: Isaac L. L. Yuki, Bernhard Herz
 */
//******************************************************************************

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdbool.h>
#include "board.h"
#include "clock_config.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"
#include "sdmmc_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! \brief Data block count accessed in card */
#define DATA_BLOCK_COUNT (5U)
/*! \brief Start data block number accessed in card */
#define DATA_BLOCK_START (2U)
/*! \brief Data buffer size. */
#define DATA_BUFFER_SIZE (FSL_SDMMC_DEFAULT_BLOCK_SIZE * DATA_BLOCK_COUNT)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void     CardInformationLog(sd_card_t *card);
static void     SDCARD_DetectCallBack(bool isInserted, void *userData);
static status_t AccessCard(sd_card_t *card, bool isReadOnly);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static volatile bool s_cardInserted = false;

/*!
 * \brief Card descriptor.
 */
static sd_card_t g_sd;

/*!
 * \brief Data written to the card
 * \note decription about the read/write buffer
 * The size of the read/write buffer should be a multiple of 512, since
 * SDHC/SDXC card uses 512-byte fixed block length and this driver example is
 * enabled with a SDHC/SDXC card.If you are using a SDSC card, you can define
 * the block length by yourself if the card supports partial access. The address
 * of the read/write buffer should align to the specific DMA data buffer address
 * align value if DMA transfer is used, otherwise the buffer address is not
 * important. At the same time buffer address/size should be aligned to the
 * cache line size if cache is supported.
 */
SDK_ALIGN(uint8_t g_dataWrite[DATA_BUFFER_SIZE],
          BOARD_SDMMC_DATA_BUFFER_ALIGN_SIZE);
/*!
 * \brief Data read from the card
 */
SDK_ALIGN(uint8_t g_dataRead[DATA_BUFFER_SIZE],
          BOARD_SDMMC_DATA_BUFFER_ALIGN_SIZE);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 *\brief Callback for changing state of s_cardInterted
 */
static void SDCARD_DetectCallBack(bool isInserted, void *userData)
{
  (void) userData;
  s_cardInserted = isInserted;
}

/*!
 * \brief  This is the main application entry point where system is
 * initialized and all system components are started.
 */
void main(void)
{
  BOARD_Initialize();

  sd_card_t *card = &g_sd;
  bool       isReadOnly;

  BOARD_SD_Config(card, SDCARD_DetectCallBack, BOARD_SDMMC_SD_HOST_IRQ_PRIORITY,
                  NULL);

  /* SD host init function */
  if (SD_HostInit(card) != kStatus_Success)
  {
    PRINTF("SD host init fail\r\n");
  }

  PRINTF("Please press any Button to execute the SD Card test. Make sure "
         "you've inserted a SD Card! \r\n");
  GETCHAR();
  /* wait card insert */
  SD_PollingCardInsert(card, kSD_Inserted);

  /* power off card */
  SD_SetCardPower(card, false);
  /* power on the card */
  SD_SetCardPower(card, true);

  PRINTF("--> Card detected\r\n");

  /* Init card. */
  if (SD_CardInit(card))
  {
    PRINTF("SD card init failed\r\n");
  }
  /* card information log */
  CardInformationLog(card);

  /* Check if card is readonly. */
  isReadOnly = SD_CheckReadOnly(card);

  PRINTF("\r\nRead/Write/Erase the card continuously until encounter "
         "error......\r\n");

  if (kStatus_Success != AccessCard(card, isReadOnly))
  {
    /* access card fail, due to card remove. */
    if (SD_IsCardPresent(card) == false)
    {
      SD_HostDoReset(card);
      PRINTF("Card removed\r\n");
    }
    else
    {
      PRINTF("Error \r\n");
    }
  }
  PRINTF("\r\nEnd of demonstration.\r\n");
}

/*!
 * \brief  Accesses the SD-Card by writing, reading and erasing data blocks.
 * \param card The SD-Card being used.
 * \param isReadOnly Determines writing permissions on SD-Card.
 */
static status_t AccessCard(sd_card_t *card, bool isReadOnly)
{
  if (isReadOnly)
  {
    PRINTF("\r\nRead one data block......\r\n");
    if (kStatus_Success
        != SD_ReadBlocks(card, g_dataRead, DATA_BLOCK_START, 1U))
    {
      PRINTF("Read one data block failed.\r\n");
      return kStatus_Fail;
    }

    PRINTF("Read multiple data blocks......\r\n");
    if (kStatus_Success
        != SD_ReadBlocks(card, g_dataRead, DATA_BLOCK_START, DATA_BLOCK_COUNT))
    {
      PRINTF("Read multiple data blocks failed.\r\n");
      return kStatus_Fail;
    }
  }
  else
  {
    memset(g_dataWrite, 0x67U, sizeof(g_dataWrite));

    PRINTF("\r\nWrite/read one data block......\r\n");
    if (kStatus_Success
        != SD_WriteBlocks(card, g_dataWrite, DATA_BLOCK_START, 1U))
    {
      PRINTF("Write one data block failed.\r\n");
      return kStatus_Fail;
    }

    memset(g_dataRead, 0U, sizeof(g_dataRead));
    if (kStatus_Success
        != SD_ReadBlocks(card, g_dataRead, DATA_BLOCK_START, 1U))
    {
      PRINTF("Read one data block failed.\r\n");
      return kStatus_Fail;
    }

    PRINTF("Compare the read/write content......\r\n");
    if (memcmp(g_dataRead, g_dataWrite, FSL_SDMMC_DEFAULT_BLOCK_SIZE))
    {
      PRINTF("The read/write content isn't consistent.\r\n");
      return kStatus_Fail;
    }
    PRINTF("The read/write content is consistent.\r\n");

    PRINTF("Write/read multiple data blocks......\r\n");
    if (kStatus_Success
        != SD_WriteBlocks(card, g_dataWrite, DATA_BLOCK_START,
                          DATA_BLOCK_COUNT))
    {
      PRINTF("Write multiple data blocks failed.\r\n");
      return kStatus_Fail;
    }

    memset(g_dataRead, 0U, sizeof(g_dataRead));

    if (kStatus_Success
        != SD_ReadBlocks(card, g_dataRead, DATA_BLOCK_START, DATA_BLOCK_COUNT))
    {
      PRINTF("Read multiple data blocks failed.\r\n");
      return kStatus_Fail;
    }

    PRINTF("Compare the read/write content......\r\n");
    if (memcmp(g_dataRead, g_dataWrite, FSL_SDMMC_DEFAULT_BLOCK_SIZE))
    {
      PRINTF("The read/write content isn't consistent.\r\n");
      return kStatus_Fail;
    }
    PRINTF("The read/write content is consistent.\r\n");

    PRINTF("Erase multiple data blocks......\r\n");
    if (kStatus_Success
        != SD_EraseBlocks(card, DATA_BLOCK_START, DATA_BLOCK_COUNT))
    {
      PRINTF("Erase multiple data blocks failed.\r\n");
      return kStatus_Fail;
    }
  }

  return kStatus_Success;
}

/*!
 * \brief  Accesses various information from the log.
 * \param card The SD-Card being used.
 */
static void CardInformationLog(sd_card_t *card)
{
  assert(card);

  PRINTF("\r\nCard size %d * %d bytes\r\n", card->blockCount, card->blockSize);
  PRINTF("\r\nWorking condition:\r\n");
  if (card->operationVoltage == kSDMMC_OperationVoltage330V)
  {
    PRINTF("\r\n  Voltage : 3.3V\r\n");
  }
  else if (card->operationVoltage == kSDMMC_OperationVoltage180V)
  {
    PRINTF("\r\n  Voltage : 1.8V\r\n");
  }

  if (card->currentTiming == kSD_TimingSDR12DefaultMode)
  {
    if (card->operationVoltage == kSDMMC_OperationVoltage330V)
    {
      PRINTF("\r\n  Timing mode: Default mode\r\n");
    }
    else if (card->operationVoltage == kSDMMC_OperationVoltage180V)
    {
      PRINTF("\r\n  Timing mode: SDR12 mode\r\n");
    }
  }
  else if (card->currentTiming == kSD_TimingSDR25HighSpeedMode)
  {
    if (card->operationVoltage == kSDMMC_OperationVoltage180V)
    {
      PRINTF("\r\n  Timing mode: SDR25\r\n");
    }
    else
    {
      PRINTF("\r\n  Timing mode: High Speed\r\n");
    }
  }
  else if (card->currentTiming == kSD_TimingSDR50Mode)
  {
    PRINTF("\r\n  Timing mode: SDR50\r\n");
  }
  else if (card->currentTiming == kSD_TimingSDR104Mode)
  {
    PRINTF("\r\n  Timing mode: SDR104\r\n");
  }
  else if (card->currentTiming == kSD_TimingDDR50Mode)
  {
    PRINTF("\r\n  Timing mode: DDR50\r\n");
  }

  PRINTF("\r\n  Freq : %d HZ\r\n", card->busClock_Hz);
}
