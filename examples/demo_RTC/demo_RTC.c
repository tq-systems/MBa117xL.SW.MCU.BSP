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
#include "string.h"
#include "fsl_snvs_hp.h"
#include "fsl_snvs_lp.h"
#include "TQ_utils.h"
#include "PCF85063ATL.h"
#include "lpi2c_api.h"
#include "demo_RTC.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define INPUT_BUFFER_SIZE 32

#define SECOND 20
#define MINUTE 30
#define HOUR 13
#define DAY 12
#define WEEKDAY 3
#define MONTH 11
#define YEAR 2023

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void     internalRTC_init(void);
static uint32_t PCF85063ATL_TransferFunction(
  void *peripheral, uint8_t regAddress, const size_t regAddressSize,
  uint8_t *buffer, const size_t dataSize,
  PCF85063ATL_TransferDirection_t transferDirection);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static LPI2C_masterDevice_t rtc_peripheral = {
  .base_addr                 = LPI2C_BASE_ADDR,
  .masterXfer.slaveAddress   = RTC_I2C_SLAVE_ADDR,
  .masterXfer.subaddressSize = 0U,
  .masterXfer.dataSize       = 0U,
};
static PCF85063ATL_Handle_t moduleRTC_handle = {.peripheral = &rtc_peripheral,
                                                .transferFunction =
                                                  PCF85063ATL_TransferFunction};

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * \brief  This is the main application entry point where system is
 * initialized and all system components are started.
 */
void main(void)
{
  status_t               status          = kStatus_Fail;
  PCF85063ATL_DateTime_t datetime        = {.day     = DAY,
                                            .month   = MONTH,
                                            .year    = YEAR,
                                            .hour    = HOUR,
                                            .minute  = MINUTE,
                                            .second  = SECOND,
                                            .weekday = WEEKDAY};
  bool                   moduleRTCexists = false;

  BOARD_Initialize();

  /* initialize I2C peripheral for external RTC*/
  LPI2C_MasterGetDefaultConfig(&rtc_peripheral.masterConfig);
  rtc_peripheral.masterConfig.baudRate_Hz = I2C_BITRATE;
  LPI2C_MasterInit(LPI2C_BASE_ADDR, &rtc_peripheral.masterConfig,
                   LPI2C_MASTER_CLK_FREQ);

  /* initialize internal RTC*/
  internalRTC_init();

  /* Set time on internal RTC*/
  SNVS_LP_SRTC_StopTimer(SNVS);
  SNVS_LP_SRTC_SetDatetime(SNVS, (snvs_lp_srtc_datetime_t *) &datetime);
  SNVS_LP_SRTC_StartTimer(SNVS);
  SNVS_HP_RTC_StopTimer(SNVS);
  SNVS_HP_RTC_TimeSynchronize(SNVS);
  SNVS_HP_RTC_StartTimer(SNVS);

  /* Check if module RTC is available */
  moduleRTCexists = (LPI2C_MasterTransferBlocking(rtc_peripheral.base_addr,
                                                  &rtc_peripheral.masterXfer)
                     == kStatus_Success)
                      ? true
                      : false;
  if (moduleRTCexists)
  {
    /* configure module RTC */
    PCF85063ATL_getDefaultConfiguration(&moduleRTC_handle);
    moduleRTC_handle.config.cap_sel = true; // used capacitance is 12pF

    status = (status_t) PCF85063ATL_configure(&moduleRTC_handle);
    if (status != kStatus_Success)
    {
      PRINTF("Error configuring module RTC: %u \r\n", status);
    }
    /* Set time on module RTC */
    status = (status_t) PCF85063ATL_writeDateTime(&moduleRTC_handle, &datetime);
    if (status != kStatus_Success)
    {
      PRINTF("Error writing to module RTC: %u \r\n", status);
    }
  }
  PRINTF("Reading...\r\n\n");
  while (1)
  {
    if (moduleRTCexists)
    {
      status =
        (status_t) PCF85063ATL_readDateTime(&moduleRTC_handle, &datetime);
      if (status != kStatus_Success)
      {
        PRINTF("Error reading to module RTC: %u \r\n", status);
      }
      else
      {
        PRINTF("Current datetime, module: %04d-%02d-%02d %02d:%02d:%02d\r\n",
               datetime.year, datetime.month, datetime.day, datetime.hour,
               datetime.minute, datetime.second);
      }
    }
    SNVS_HP_RTC_GetDatetime(SNVS, (snvs_hp_rtc_datetime_t *) &datetime);
    PRINTF("Current datetime, internal: %04d-%02d-%02d %02d:%02d:%02d\r\n",
           datetime.year, datetime.month, datetime.day, datetime.hour,
           datetime.minute, datetime.second);
    SDK_DelayAtLeastUs(1000000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
  }
}

/*!
 * \brief Initialize the internal Real-Time Clock (RTC).
 *
 * This function initializes the internal RTC of an NXP processor using the
 * fsl_snvs library. It configures both the High-Power (HP) and Low-Power (LP)
 * sections of the SNVS (Secure Non-Volatile Storage) block.
 *
 * The initialization process involves the following steps:
 * 1. Get default configurations for the SNVS High-Power and Low-Power RTC.
 * 2. Initialize the SNVS HP RTC and LP SRTC (Secure Real Time Clock).
 * 3. Start the SNVS LP SRTC timer.
 * 4. Synchronize the SNVS HP RTC time.
 * 5. Start the SNVS HP RTC timer.
 * 6. Manually set the calibration value for both LP and HP RTC.
 *
 * \note The calibration step involves disabling timer calibration and setting
 * the calibration value. The chosen value adds 2 counts per each 32768 ticks
 * of the counter clock. This value was choosen based on empiric research.
 * After the setting is complete, timer calibration is enabled again.
 */
static void internalRTC_init(void)
{
  snvs_hp_rtc_config_t  snvsRtcConfig;
  snvs_lp_srtc_config_t snvsSrtcConfig;
  SNVS_HP_RTC_GetDefaultConfig(&snvsRtcConfig);
  SNVS_HP_RTC_Init(SNVS, &snvsRtcConfig);

  SNVS_LP_SRTC_GetDefaultConfig(&snvsSrtcConfig);
  SNVS_LP_SRTC_Init(SNVS, &snvsSrtcConfig);

  SNVS_LP_SRTC_StartTimer(SNVS);
  SNVS_HP_RTC_TimeSynchronize(SNVS);
  SNVS_HP_RTC_StartTimer(SNVS);

  // setting calibration manually
  uint32_t const cal = 2U;
  SNVS->LPCR &= ~SNVS_LPCR_LPCALB_EN_MASK;
  SNVS->LPCR =
    (SNVS->LPCR & ~SNVS_LPCR_LPCALB_VAL_MASK) | SNVS_LPCR_LPCALB_VAL(cal);
  SNVS->LPCR |= SNVS_LPCR_LPCALB_EN_MASK;
  SNVS->HPCR &= ~SNVS_HPCR_HPCALB_EN_MASK;
  SNVS->HPCR =
    (SNVS->HPCR & ~SNVS_HPCR_HPCALB_VAL_MASK) | SNVS_HPCR_HPCALB_VAL(cal);
  SNVS->HPCR |= SNVS_HPCR_HPCALB_EN_MASK;
}

/*!
 * \brief This is the transfer function that is used by the driver.
 * \note Parameters are described in the drivers function.
 */
static uint32_t PCF85063ATL_TransferFunction(
  void *peripheral, uint8_t regAddress, const size_t regAddressSize,
  uint8_t *buffer, const size_t dataSize,
  PCF85063ATL_TransferDirection_t transferDirection)
{
  uint32_t status;
#ifdef USE_LPI2C
  LPI2C_masterDevice_t *master      = (LPI2C_masterDevice_t *) peripheral;
  master->masterXfer.subaddress     = regAddress;
  master->masterXfer.data           = buffer;
  master->masterXfer.dataSize       = dataSize;
  master->masterXfer.subaddressSize = regAddressSize;
  switch (transferDirection)
  {
  case PCF85063ATL_WRITE:
    master->masterXfer.flags     = kLPI2C_TransferDefaultFlag;
    master->masterXfer.direction = kLPI2C_Write;
    status = (uint32_t) LPI2C_MasterTransferBlocking(master->base_addr,
                                                     &master->masterXfer);
    break;
  case PCF85063ATL_READ:
    master->masterXfer.flags     = kLPI2C_TransferRepeatedStartFlag;
    master->masterXfer.direction = kLPI2C_Read;
    status = (uint32_t) LPI2C_MasterTransferBlocking(master->base_addr,
                                                     &master->masterXfer);
    break;
  default:
    status = kStatus_Fail;
  }
#else
  I2C_masterDevice_t *master    = (I2C_masterDevice_t *) peripheral;
  master->masterXfer.subaddress = regAddress;
  master->masterXfer.data       = buffer;
  master->masterXfer.dataSize   = dataSize;
  switch (transferDirection)
  {
  case PCF85063ATL_WRITE:
    master->masterXfer.flags     = I2C_TransferDefaultFlag;
    master->masterXfer.direction = I2C_Write;
    status = (uint32_t) I2C_MasterTransferBlocking(master->base_addr,
                                                   &master->masterXfer);
    break;
  case PCF85063ATL_READ:
    master->masterXfer.flags     = I2C_TransferRepeatedStartFlag;
    master->masterXfer.direction = I2C_Read;
    status = (uint32_t) I2C_MasterTransferBlocking(master->base_addr,
                                                   &master->masterXfer);
    break;
  default:
    status = kStatus_Fail;
  }
#endif
  return status;
}
