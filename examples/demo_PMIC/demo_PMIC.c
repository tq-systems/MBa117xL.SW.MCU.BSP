//*****************************************************************************
/*!
 * \copyright
 * SPDX-License-Identifier: BSD-3-Clause
 * \copyright
 * Copyright (c) 2021 - 2023 TQ-Systems GmbH <license@tq-group.com>,
 * D-82229 Seefeld, Germany.
 * Author: Isaac L. L. Yuki, Bernhardt Herz
 */
//******************************************************************************

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "PMIC.h"
#include "demo_PMIC.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "lpi2c_api.h"
#include "TQ_utils.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define PMIC_REG_VSW1_RUN 0x4B
#define PMIC_REG_DATA_SIZE (1U)
#define PMIC_REG_ADDRESS_SIZE (1U)

/*******************************************************************************
 * Variables
 ******************************************************************************/

lpi2c_master_config_t masterConfig;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 *  Code
 ******************************************************************************/

/*!
 * \brief  This is the main application entry point where system is initialized
 *         and all system components are started.
 */
void main()
{
  char     string[10];
  float    voltageValue;
  uint8_t  deviceId;
  status_t status = kStatus_NoTransferInProgress;
  BOARD_Initialize();

  LPI2C_MasterGetDefaultConfig(&masterConfig);
  masterConfig.baudRate_Hz = I2C_BITRATE;
  LPI2C_MasterInit(LPI2C_BASE_ADDR, &masterConfig, LPI2C_MASTER_CLK_FREQ);

  PRINTF("Searching for device... \r\n");
  if (kStatus_Success == PMIC_readDeviceID(&deviceId))
  {
    if ((deviceId & 0xF0) == 0x50)
    {
      if ((deviceId & 0x0F) == 0x0)
      {
        PRINTF("[ OK ]\tPMIC PF5020 QM\r\n");
        status = kStatus_Success;
      }
      else
      {
        if ((deviceId & 0x0F) == 0x8)
        {
          PRINTF("[ OK ]\tPMIC PF5020 ASIL B\r\n");
          status = kStatus_Success;
        }
        else
        {
          PRINTF("[\033[1;31mERROR \033[0m] PMIC Access ID = 0x%x \r\n",
                 deviceId);
          status = kStatus_Fail;
        }
      }
    }
  }
  else
  {
    PRINTF("[\033[1;31mERROR \033[0m] PMIC Access ID = 0x%x \r\n", deviceId);
    status = kStatus_Fail;
  }

  if (status == kStatus_Success)
  {
    while (status == kStatus_Success)
    {
      PRINTF("Select one of three power states. 1/2/3 \r\n");
      getline(string, 2, NULL);
      uint8_t setting = (uint8_t) strtoul(string, NULL, 0);
      switch (setting)
      {
      case 1:
        status = PMIC_setCoreVoltage(PMIC_VDD_SOC_1V000);
        break;
      case 2:
        status = PMIC_setCoreVoltage(PMIC_VDD_SOC_1V100);
        break;
      case 3:
        status = PMIC_setCoreVoltage(PMIC_VDD_SOC_0V900);
        break;
      default:
        PRINTF("Error\r\n");
      }
      PMIC_readCoreVoltage(&voltageValue);
      PRINTF(" \t - New VDD_SOC:    %.3f V\r\n", voltageValue);
    }
    PRINTF("There was an Error with the I2C-Transfer. Errorcode: %zu \r\n",
           status);
  }
}
