//*****************************************************************************
/*!
 * \copyright
 * SPDX-License-Identifier: BSD-3-Clause
 * \copyright
 * Copyright 2019 - 2021, 2023 NXP
 * All rights reserved.
 * \copyright
 * Copyright (c) 2021 - 2024 TQ-Systems GmbH <license@tq-group.com>,
 * D-82229 Seefeld, Germany.
 * Author: Isaac L. L. Yuki, Maximilian Kürth
 */
//******************************************************************************

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "display_support.h"
#include "fsl_gpio.h"
#include "fsl_mipi_dsi.h"
#include "pin_mux.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "PCA9555BS.h"
#include "fsl_lpi2c.h"
#include "lpi2c_api.h"
#include "demo_display_lvds.h"
#include "fsl_pwm.h"
#include "fsl_xbara.h"
#include "fsl_common.h"

#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)
#include "fsl_dc_fb_lcdifv2.h"
#else
#include "fsl_dc_fb_elcdif.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define TEST_MODE 0

#define DEMO_HSW 1  // 6
#define DEMO_HFP 64 // 12
#define DEMO_HBP 5  // 24
#define DEMO_VSW 1  // 2
#define DEMO_VFP 40 // 16
#define DEMO_VBP 2  // 14

#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)

#define DEMO_LCDIF_POL_FLAGS                                                   \
  (kLCDIFV2_DataEnableActiveHigh | kLCDIFV2_VsyncActiveLow                     \
   | kLCDIFV2_HsyncActiveLow | kLCDIFV2_DriveDataOnFallingClkEdge)

#define DEMO_LCDIF LCDIFV2

#endif

/* Definitions for MIPI. */
#define DEMO_MIPI_DSI (&g_mipiDsi)
#define DEMO_MIPI_DSI_LANE_NUM 2

#define LVDS_PW_EN_PIN 0x08U
#define LVDS_RST_PIN 0x04U
#define LVDS_BL_PIN 0x02U
#define LVDS_DSI_SEL_PIN 0x20U

#define BOARD_PWM_BASEADDR PWM2
#define PWM_SRC_CLK_FREQ CLOCK_GetRootClockFreq(kCLOCK_Root_Bus)
#define DEMO_PWM_CLOCK_DEVIDER kPWM_Prescale_Divide_1

/*
 * The DPHY bit clock must be fast enough to send out the pixels, it should be
 * larger than:
 *
 *         (Pixel clock * bit per output pixel) / number of MIPI data lane
 *
 * Here the desired DPHY bit clock multiplied by ( 9 / 8 = 1.125) to ensure
 * it is fast enough.
 */
#define DEMO_MIPI_DPHY_BIT_CLK_ENLARGE(origin) (((origin) / 8) * 9)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void     BOARD_SetupBacklightPWM(void);
static void     BOARD_InitLcdifClock(void);
static void     BOARD_InitMipiDsiClock(void);
static uint32_t PCA9555BS_TransferFunction(
  void *peripheral, uint32_t regAddress, size_t regAddressSize,
  PCA9555BS_TransferDirection_t transferDirection, uint8_t *buffer,
  size_t bufferSize);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Initialize peripheral */
#ifdef USE_LPI2C
static LPI2C_masterDevice_t i2c_master = {
  .base_addr                 = LPI2C_BASE_ADDR,
  .masterConfig.baudRate_Hz  = LPI2C_BITRATE,
  .mode                      = LPI2C_BlockingMode,
  .masterXfer.flags          = kLPI2C_TransferDefaultFlag,
  .masterXfer.slaveAddress   = PORTEXP_I2C_DEVICE_ADR,
  .masterXfer.subaddressSize = 1,
  .masterXfer.dataSize       = 2};
#else
#error "Only LPI2C supported."
#endif

/* Device configuration */
static PCA9555BS_Handle_t portExpander = {
  .IOMask.PIN00_PIN07_Mask = PORTEXP_IO_MASK_0,
  .IOMask.PIN10_PIN17_Mask = PORTEXP_IO_MASK_1,

  /* Pin descriptions:
    All: Don't care.

    The Pin descriptions are related to this application.
  */
  .OutputMask.PIN00_PIN07_Mask = 0x0,
  /* Pin descriptions:
     7: Don't care, 6: Don't care, 5: MSPI_DSI_SEL, 4: #DSI_CTP_RST,
     3: DSI_LCD_PWR_EN, 2: #DSI_LCD_RST, 1: Don't care, 0: Don't care

    The Pin descriptions are related to this application.
  */
  .OutputMask.PIN10_PIN17_Mask = 0b00101010,
  .transferFunction            = PCA9555BS_TransferFunction,
  .peripheral                  = &i2c_master};

static uint32_t mipiDsiTxEscClkFreq_Hz;
static uint32_t mipiDsiDphyBitClkFreq_Hz;
static uint32_t mipiDsiDphyRefClkFreq_Hz;
static uint32_t mipiDsiDpiClkFreq_Hz;

const MIPI_DSI_Type g_mipiDsi = {
  .host = DSI_HOST,
  .apb  = DSI_HOST_APB_PKT_IF,
  .dpi  = DSI_HOST_DPI_INTFC,
  .dphy = DSI_HOST_DPHY_INTFC,
};

static uint32_t SN65DSI83_Initialize(void)
{
  lpi2c_master_config_t   config;
  lpi2c_master_transfer_t transfer;
  uint8_t                 data;

  LPI2C_MasterGetDefaultConfig(&config);
  config.baudRate_Hz = 100000u;
  LPI2C_MasterInit(LPI2C5_BASE, &config,
                   CLOCK_GetRootClockFreq(kCLOCK_Root_Lpi2c5));

  transfer.data           = &data;
  transfer.dataSize       = 1u;
  transfer.direction      = kLPI2C_Read;
  transfer.flags          = kLPI2C_TransferDefaultFlag;
  transfer.slaveAddress   = 0x2Cu;
  transfer.subaddress     = 0x5Eu;
  transfer.subaddressSize = 1u;
  uint32_t status =
    (uint32_t) LPI2C_MasterTransferBlocking(LPI2C5_BASE, &transfer);
  if (status == kStatus_Success)
  {
    PRINTF("0xE5 content at boot-time: 0x%02X\r\n", data);
  }

  return status;
}

static uint32_t SN65DSI83_ReadCSR(uint8_t const addr, uint8_t *const data)
{
  lpi2c_master_transfer_t transfer;

  transfer.data           = data;
  transfer.dataSize       = 1u;
  transfer.direction      = kLPI2C_Read;
  transfer.flags          = kLPI2C_TransferDefaultFlag;
  transfer.slaveAddress   = 0x2Cu;
  transfer.subaddress     = addr;
  transfer.subaddressSize = 1u;
  return (uint32_t) LPI2C_MasterTransferBlocking(LPI2C5_BASE, &transfer);
}

static uint32_t SN65DSI83_DumpCRSs(void)
{
  uint8_t data;
  uint8_t addr[] = {0x09u, 0x0Au, 0x0Bu, 0x0Du, 0x10u, 0x11u, 0x12u, 0x18u,
                    0x19u, 0x1Au, 0x1Bu, 0x20u, 0x21u, 0x24u, 0x25u, 0x28u,
                    0x29u, 0x2Cu, 0x2Du, 0x30u, 0x31u, 0x34u, 0x36u, 0x38u,
                    0x3Au, 0x3Cu, 0xE0u, 0xE1u, 0xE5u};

  for (uint8_t i = 0u; i < sizeof(addr); ++i)
  {
    data            = 0x00u;
    uint32_t status = SN65DSI83_ReadCSR(addr[i], &data);
    if (status == 0u)
    {
      PRINTF("SN65DSI83 CSR 0x%02X: 0x%02X\r\n", addr[i], data);
    }
    else
    {
      PRINTF("SN65DSI83 failed to read CSR 0x%02X\r\n", addr[i]);
    }
  }

  return kStatus_Success;
}

static uint32_t SN65DSI83_WriteCSR(uint8_t const addr, uint8_t const data)
{
  lpi2c_master_transfer_t transfer;

  transfer.data           = &data;
  transfer.dataSize       = 1u;
  transfer.direction      = kLPI2C_Write;
  transfer.flags          = kLPI2C_TransferDefaultFlag;
  transfer.slaveAddress   = 0x2Cu;
  transfer.subaddress     = addr;
  transfer.subaddressSize = 1u;
  return (uint32_t) LPI2C_MasterTransferBlocking(LPI2C5_BASE, &transfer);
}

#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)

static dc_fb_lcdifv2_handle_t s_dcFbLcdifv2Handle = {0};

static const dc_fb_lcdifv2_config_t s_dcFbLcdifv2Config = {
  .lcdifv2       = DEMO_LCDIF,
  .width         = DEMO_PANEL_WIDTH,
  .height        = DEMO_PANEL_HEIGHT,
  .hsw           = DEMO_HSW,
  .hfp           = DEMO_HFP,
  .hbp           = DEMO_HBP,
  .vsw           = DEMO_VSW,
  .vfp           = DEMO_VFP,
  .vbp           = DEMO_VBP,
  .polarityFlags = DEMO_LCDIF_POL_FLAGS,
  .lineOrder     = kLCDIFV2_LineOrderRGB,
/* CM4 is domain 1, CM7 is domain 0. */
#if (__CORTEX_M <= 4)
  .domain = 1,
#else
  .domain = 0,
#endif
};

const dc_fb_t g_dc = {
  .ops     = &g_dcFbOpsLcdifv2,
  .prvData = &s_dcFbLcdifv2Handle,
  .config  = &s_dcFbLcdifv2Config,
};

#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

static void BOARD_SetupBacklightPWM(void)
{
  pwm_config_t PWM2_SM0_config = {
    .clockSource           = kPWM_BusClock,
    .prescale              = kPWM_Prescale_Divide_1,
    .pairOperation         = kPWM_Independent,
    .initializationControl = kPWM_Initialize_LocalSync,
    .reloadLogic           = kPWM_ReloadImmediate,
    .reloadSelect          = kPWM_LocalReload,
    .reloadFrequency       = kPWM_LoadEveryOportunity,
    .forceTrigger          = kPWM_Force_Local,
    .enableDebugMode       = true,
  };

  pwm_signal_param_t PWM2_SM0_pwm_function_config[1] = {
    {.pwmChannel       = kPWM_PwmA,
     .dutyCyclePercent = 100U,
     .level            = kPWM_HighTrue,
     .faultState       = kPWM_PwmFaultState0,
     .pwmchannelenable = true,
     .deadtimeValue    = 0U},
  };

  const pwm_fault_input_filter_param_t PWM2_faultInputFilter_config = {
    .faultFilterPeriod  = 1U,
    .faultFilterCount   = 3U,
    .faultGlitchStretch = false};
  const pwm_fault_param_t PWM2_Fault0_fault_config = {
    .faultClearingMode       = kPWM_Automatic,
    .faultLevel              = false,
    .enableCombinationalPath = true,
    .recoverMode             = kPWM_RecoverHalfCycle};
  const pwm_fault_param_t PWM2_Fault1_fault_config = {
    .faultClearingMode       = kPWM_Automatic,
    .faultLevel              = false,
    .enableCombinationalPath = true,
    .recoverMode             = kPWM_RecoverHalfCycle};
  const pwm_fault_param_t PWM2_Fault2_fault_config = {
    .faultClearingMode       = kPWM_Automatic,
    .faultLevel              = false,
    .enableCombinationalPath = true,
    .recoverMode             = kPWM_RecoverHalfCycle};
  const pwm_fault_param_t PWM2_Fault3_fault_config = {
    .faultClearingMode       = kPWM_Automatic,
    .faultLevel              = false,
    .enableCombinationalPath = true,
    .recoverMode             = kPWM_RecoverHalfCycle};

  /* Set the PWM Fault inputs to a low value */
  XBARA_Init(XBARA1);
  XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputLogicHigh,
                             kXBARA1_OutputFlexpwm2Fault0);
  XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputLogicHigh,
                             kXBARA1_OutputFlexpwm2Fault1);
  XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputLogicHigh,
                             kXBARA1_OutputFlexpwm1234Fault2);
  XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputLogicHigh,
                             kXBARA1_OutputFlexpwm1234Fault3);

  /* Initialize PWM submodule SM0 main configuration */
  PWM_Init(PWM2, kPWM_Module_2, &PWM2_SM0_config);
  /* Initialize fault input filter configuration */
  PWM_SetupFaultInputFilter(PWM2, &PWM2_faultInputFilter_config);
  /* Initialize fault channel 0 fault Fault0 configuration */
  PWM_SetupFaults(PWM2, kPWM_Fault_0, &PWM2_Fault0_fault_config);
  /* Initialize fault channel 0 fault Fault1 configuration */
  PWM_SetupFaults(PWM2, kPWM_Fault_1, &PWM2_Fault1_fault_config);
  /* Initialize fault channel 0 fault Fault2 configuration */
  PWM_SetupFaults(PWM2, kPWM_Fault_2, &PWM2_Fault2_fault_config);
  /* Initialize fault channel 0 fault Fault3 configuration */
  PWM_SetupFaults(PWM2, kPWM_Fault_3, &PWM2_Fault3_fault_config);
  /* Initialize deadtime logic input for the channel A */
  PWM_SetupForceSignal(PWM2, kPWM_Module_2, kPWM_PwmA, kPWM_UsePwm);
  /* Setup PWM output setting for submodule SM0 */
  PWM_SetupPwm(PWM2, kPWM_Module_2, PWM2_SM0_pwm_function_config, 1U,
               kPWM_EdgeAligned, 100000U, 160000000U);
  /* Initialize LDOK for update of the working registers */
  PWM_SetPwmLdok(PWM2, (kPWM_Control_Module_2), true);
  /* Start selected counters */
  PWM_StartTimer(PWM2, (kPWM_Control_Module_2));
}

static void BOARD_InitLcdifClock(void)
{
  /*
   * The pixel clock is (height + VSW + VFP + VBP) * (width + HSW + HFP + HBP) *
   * frame rate.
   *
   * For the TIANMA TM070JHG33 the configured frame rate is 30 FPS, so the
   * pixel clock is 33MHz.
   */
  const clock_root_config_t lcdifClockConfig = {
    .clockOff = false,
    .mux      = 4, /*!< PLL_528. */
    .div      = 16,
  };

#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)
  CLOCK_SetRootClock(kCLOCK_Root_Lcdifv2, &lcdifClockConfig);

  mipiDsiDpiClkFreq_Hz = CLOCK_GetRootClockFreq(kCLOCK_Root_Lcdifv2);

#else

  CLOCK_SetRootClock(kCLOCK_Root_Lcdif, &lcdifClockConfig);

  mipiDsiDpiClkFreq_Hz = CLOCK_GetRootClockFreq(kCLOCK_Root_Lcdif);
#endif
}

static void BOARD_InitMipiDsiClock(void)
{
  uint32_t mipiDsiEscClkFreq_Hz;

  /* RxClkEsc max 60MHz, TxClkEsc 12 to 20MHz. */
  /* RxClkEsc = 528MHz / 11 = 48MHz. */
  /* TxClkEsc = 528MHz / 11 / 4 = 16MHz. */
  const clock_root_config_t mipiEscClockConfig = {
    .clockOff = false,
    .mux      = 4, /*!< PLL_528. */
    .div      = 11,
  };

  CLOCK_SetRootClock(kCLOCK_Root_Mipi_Esc, &mipiEscClockConfig);

  mipiDsiEscClkFreq_Hz = CLOCK_GetRootClockFreq(kCLOCK_Root_Mipi_Esc);

  const clock_group_config_t mipiEscClockGroupConfig = {
    .clockOff = false, .resetDiv = 2, .div0 = 2, /* TX esc clock. */
  };

  CLOCK_SetGroupConfig(kCLOCK_Group_MipiDsi, &mipiEscClockGroupConfig);

  mipiDsiTxEscClkFreq_Hz = mipiDsiEscClkFreq_Hz / 3;

  /* DPHY reference clock, use OSC 24MHz clock. */
  const clock_root_config_t mipiDphyRefClockConfig = {
    .clockOff = false,
    .mux      = 1, /*!< OSC_24M. */
    .div      = 1,
  };

  CLOCK_SetRootClock(kCLOCK_Root_Mipi_Ref, &mipiDphyRefClockConfig);

  mipiDsiDphyRefClkFreq_Hz = BOARD_XTAL0_CLK_HZ;
}

static void BOARD_SetMipiDsiConfig(void)
{
  dsi_config_t      dsiConfig;
  dsi_dphy_config_t dphyConfig;

  const dsi_dpi_config_t dpiConfig = {.pixelPayloadSize = DEMO_PANEL_WIDTH,
                                      .dpiColorCoding   = kDSI_Dpi24Bit,
                                      .pixelPacket      = kDSI_PixelPacket24Bit,
                                      .videoMode        = kDSI_DpiBurst,
                                      .bllpMode         = kDSI_DpiBllpLowPower,
                                      .polarityFlags    = kDSI_DpiVsyncActiveLow
                                                       | kDSI_DpiHsyncActiveLow,
                                      .hfp            = DEMO_HFP,
                                      .hbp            = DEMO_HBP,
                                      .hsw            = DEMO_HSW,
                                      .vfp            = DEMO_VFP,
                                      .vbp            = DEMO_VBP,
                                      .panelHeight    = DEMO_PANEL_HEIGHT,
                                      .virtualChannel = 0};

  /*
   * dsiConfig.numLanes = 4;
   * dsiConfig.enableNonContinuousHsClk = false;
   * dsiConfig.autoInsertEoTp = true;
   * dsiConfig.numExtraEoTp = 0;
   * dsiConfig.htxTo_ByteClk = 0;
   * dsiConfig.lrxHostTo_ByteClk = 0;
   * dsiConfig.btaTo_ByteClk = 0;
   */
  DSI_GetDefaultConfig(&dsiConfig);
  dsiConfig.numLanes                 = DEMO_MIPI_DSI_LANE_NUM;
  dsiConfig.autoInsertEoTp           = true;
  dsiConfig.enableNonContinuousHsClk = false;

  /* Init the DSI module. */
  DSI_Init(DEMO_MIPI_DSI, &dsiConfig);

  /* Init DPHY.
   *
   * The DPHY bit clock must be fast enough to send out the pixels, it should be
   * larger than:
   *
   *         (Pixel clock * bit per output pixel) / number of MIPI data lane
   *
   * Here the desired DPHY bit clock multiplied by ( 9 / 8 = 1.125) to ensure
   * it is fast enough.
   *
   * Note that the DSI output pixel is 24bit per pixel.
   */
  mipiDsiDphyBitClkFreq_Hz = // mipiDsiDphyBitClkFreq_Hz = mipiDsiDpiClkFreq_Hz
                             // 2* bpp / lanes;
    mipiDsiDpiClkFreq_Hz * (24 * 2 / DEMO_MIPI_DSI_LANE_NUM);

  mipiDsiDphyBitClkFreq_Hz =
    DEMO_MIPI_DPHY_BIT_CLK_ENLARGE(mipiDsiDphyBitClkFreq_Hz);

  DSI_GetDphyDefaultConfig(&dphyConfig, mipiDsiDphyBitClkFreq_Hz,
                           mipiDsiTxEscClkFreq_Hz);

  mipiDsiDphyBitClkFreq_Hz =
    DSI_InitDphy(DEMO_MIPI_DSI, &dphyConfig, mipiDsiDphyRefClkFreq_Hz);

  /* Init DPI interface. */
  DSI_SetDpiConfig(DEMO_MIPI_DSI, &dpiConfig, DEMO_MIPI_DSI_LANE_NUM,
                   mipiDsiDpiClkFreq_Hz, mipiDsiDphyBitClkFreq_Hz);
}

status_t BOARD_InitDisplayInterface(void)
{
  CLOCK_EnableClock(kCLOCK_Video_Mux);

#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)
  /* LCDIF v2 output to MIPI DSI. */
  VIDEO_MUX->VID_MUX_CTRL.SET = VIDEO_MUX_VID_MUX_CTRL_MIPI_DSI_SEL_MASK;
#else
  /* ELCDIF output to MIPI DSI. */
  VIDEO_MUX->VID_MUX_CTRL.CLR = VIDEO_MUX_VID_MUX_CTRL_MIPI_DSI_SEL_MASK;
#endif

  /* 1. Power on and isolation off. */
  PGMC_BPC4->BPC_POWER_CTRL |= (PGMC_BPC_BPC_POWER_CTRL_PSW_ON_SOFT_MASK
                                | PGMC_BPC_BPC_POWER_CTRL_ISO_OFF_SOFT_MASK);

  /* 2. Assert MIPI reset. */
  IOMUXC_GPR->GPR62 &= ~(IOMUXC_GPR_GPR62_MIPI_DSI_PCLK_SOFT_RESET_N_MASK
                         | IOMUXC_GPR_GPR62_MIPI_DSI_ESC_SOFT_RESET_N_MASK
                         | IOMUXC_GPR_GPR62_MIPI_DSI_BYTE_SOFT_RESET_N_MASK
                         | IOMUXC_GPR_GPR62_MIPI_DSI_DPI_SOFT_RESET_N_MASK);

  /* 3. Setup clock. */
  BOARD_InitMipiDsiClock();

  /* 4. Deassert PCLK and ESC reset. */
  IOMUXC_GPR->GPR62 |= (IOMUXC_GPR_GPR62_MIPI_DSI_PCLK_SOFT_RESET_N_MASK
                        | IOMUXC_GPR_GPR62_MIPI_DSI_ESC_SOFT_RESET_N_MASK);

  /* 5. Configures peripheral. */
  BOARD_SetMipiDsiConfig();

  /* 6. Deassert BYTE and DBI reset. */
  IOMUXC_GPR->GPR62 |= (IOMUXC_GPR_GPR62_MIPI_DSI_BYTE_SOFT_RESET_N_MASK
                        | IOMUXC_GPR_GPR62_MIPI_DSI_DPI_SOFT_RESET_N_MASK);

  /* 7. Configure the panel. */
  return kStatus_Success;
}

#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)
void LCDIFv2_IRQHandler(void)
{
  DC_FB_LCDIFV2_IRQHandler(&g_dc);
}
#else
void eLCDIF_IRQHandler(void)
{
  DC_FB_ELCDIF_IRQHandler(&g_dc);
}
#endif

status_t BOARD_VerifyDisplayClockSource(void)
{
  status_t status;
  uint32_t srcClkFreq;

  /*
   * In this implementation, the SYSPLL2 (528M) clock is used as the source
   * of LCDIFV2 pixel clock and MIPI DSI ESC clock. The OSC24M clock is used
   * as the MIPI DSI DPHY PLL reference clock. This function checks the clock
   * source are valid. OSC24M is always valid, so only verify the SYSPLL2.
   */
  srcClkFreq = CLOCK_GetPllFreq(kCLOCK_PllSys2);
  if (528 != (srcClkFreq / 1000000))
  {
    status = kStatus_Fail;
  }
  else
  {
    status = kStatus_Success;
  }

  return status;
}

status_t BOARD_PrepareDisplayController(void)
{
  status_t status;
  uint32_t result;

  status = BOARD_VerifyDisplayClockSource();

  if (status != kStatus_Success)
  {
    PRINTF("Error: Invalid display clock source.\r\n");
    return status;
  }

  BOARD_InitLcdifClock();

#ifdef USE_LPI2C
  LPI2C_MasterGetDefaultConfig(&i2c_master.masterConfig);
  i2c_master.masterConfig.baudRate_Hz = LPI2C_BITRATE;
  LPI2C_MasterInit(i2c_master.base_addr, &i2c_master.masterConfig,
                   LPI2C_CLOCK_FREQUENCY);
#else
#error "Only LPI2C supported."
#endif

  PCA9555BS_configure(&portExpander);

  // enable backlight
  if (status == kStatus_Success)
  {
    portExpander.OutputMask.PIN10_PIN17_Mask |= (LVDS_BL_PIN);
    result = PCA9555BS_setOutput(&portExpander);
    if (result != 0u)
    {
      PRINTF("failed to enable backlight\r\n");
      return kStatus_Fail;
    }
  }

  // setup backlight
  BOARD_SetupBacklightPWM();

  // init seq 1: power on
  // init seq 2: enable DSI CLK
  status = BOARD_InitDisplayInterface();
  portExpander.OutputMask.PIN10_PIN17_Mask |= (LVDS_DSI_SEL_PIN);
  result = PCA9555BS_setOutput(&portExpander);
  if (result != 0u)
  {
    PRINTF("failed to unselect LVDS bridge\r\n");
    return kStatus_Fail;
  }

  // init seq 2: enable LP11 on inactive DSI lanes
  DSI_HOST_NXP_FDSOI28_DPHY_INTFC_AUTO_PD_EN_AUTO_PD_EN(0);

  // wait for it
  SDK_DelayAtLeastUs(10000U, CLOCK_GetCpuClkFreq());

  // init seq 5: initialize CSRs
  if (SN65DSI83_Initialize() != 0u)
  {
    PRINTF("failed to initialize SN65DSI83 I2C interface\r\n");
    return kStatus_Fail;
  }

  SN65DSI83_WriteCSR(0x0Au, 0x05u);
  SN65DSI83_WriteCSR(0x0Bu, 0x28u); // div= 6
  SN65DSI83_WriteCSR(0x10u, 0x30u); // 2 lanes
  SN65DSI83_WriteCSR(0x12u, 0x58u); // 88u == 440 - 445 MHz
  SN65DSI83_WriteCSR(0x18u, 0x7Au);

  SN65DSI83_WriteCSR(0x20u, 0x00u); // CHA_ACTIVE_LINE_LENGTH_LOW
  SN65DSI83_WriteCSR(0x21u, 0x05u); // CHA_ACTIVE_LINE_LENGTH_HIGH
  SN65DSI83_WriteCSR(0x28u, 0x21u); // CHA_SYNC_DELAY_LOW
  SN65DSI83_WriteCSR(0x29u, 0x00u); // CHA_SYNC_DELAY_HIGH

#if (defined(TEST_MODE) && (TEST_MODE == 1))
  SN65DSI83_WriteCSR(0x24u, 0x20u); // CHA_VERTICAL_DISPLAY_SIZE_LOW
  SN65DSI83_WriteCSR(0x25u, 0x03u); // CHA_VERTICAL_DISPLAY_SIZE_HIGH
  SN65DSI83_WriteCSR(0x2Cu, 0x01u); // CHA_HSYNC_PULSE_WIDTH_LOW
  SN65DSI83_WriteCSR(0x2Du, 0x00u); // CHA_HSYNC_PULSE_WIDTH_HIGH
  SN65DSI83_WriteCSR(0x30u, 0x01u); // CHA_VSYNC_PULSE_WIDTH_LOW
  SN65DSI83_WriteCSR(0x31u, 0x00u); // CHA_VSYNC_PULSE_WIDTH_HIGH
  SN65DSI83_WriteCSR(0x34u, 0x05u); // CHA_HORIZONTAL_BACK_PORCH
  SN65DSI83_WriteCSR(0x36u, 0x02u); // CHA_VERTICAL_BACK_PORCH
  SN65DSI83_WriteCSR(0x38u, 0x40u); // CHA_HORIZONTAL_FRONT_PORCH
  SN65DSI83_WriteCSR(0x3Au, 0x02u); // CHA_VERTICAL_FRONT_PORCH
  SN65DSI83_WriteCSR(0x3Cu, 0x10u); // test mode
#endif

  SDK_DelayAtLeastUs(3000U, CLOCK_GetCpuClkFreq());
  // init seq 6: set PLL_EN bit in CSR
  SN65DSI83_WriteCSR(0x0Du, 0x01u);
  // wait for it
  SDK_DelayAtLeastUs(10000U, CLOCK_GetCpuClkFreq());
  // init seq 7: set SOFT_RESET bit in CSR
  SN65DSI83_WriteCSR(0x09u, 0x01u);
  // wait for it
  SDK_DelayAtLeastUs(10000U, CLOCK_GetCpuClkFreq());
  // init seq 8: change DSI data lanes to HS and start video stream
  // init seq 9: overstept
  // init seq 10: clear all errors in CSR
  status = (status_t) SN65DSI83_WriteCSR(0xE5u, 0xFFu);
  // wait for it
  SDK_DelayAtLeastUs(1000U, CLOCK_GetCpuClkFreq());
  // init seq 11: verify no error in CSR
  BOARD_ReadBridgeStatus();

  if (kStatus_Success == status)
  {
#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)
    NVIC_ClearPendingIRQ(LCDIFv2_IRQn);
    NVIC_SetPriority(LCDIFv2_IRQn, 3);
    EnableIRQ(LCDIFv2_IRQn);
#endif
  }

  return status;
}

/*!
 * \brief This is the transfer function that is used by the driver.
 * \note Parameters are described in the drivers function.
 */
static uint32_t PCA9555BS_TransferFunction(
  void *peripheral, uint32_t regAddress, size_t regAddressSize,
  PCA9555BS_TransferDirection_t transferDirection, uint8_t *buffer,
  size_t bufferSize)
{
  uint32_t status;
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
    status = (uint32_t) LPI2C_MasterTransferBlocking(master->base_addr,
                                                     &master->masterXfer);
    break;
  case PCA9555BS_READ:
    master->masterXfer.direction = kLPI2C_Read;
    status = (uint32_t) LPI2C_MasterTransferBlocking(master->base_addr,
                                                     &master->masterXfer);
    break;
  default:
    status = kStatus_Fail;
  }
#else
#error "Only LPI2C supported."
#endif
  return status;
}

status_t BOARD_ReadBridgeStatus(void)
{
  status_t status;
  uint8_t  data = 0x00u;

  status = (status_t) SN65DSI83_ReadCSR(0xE5u, &data);
  if (status == kStatus_Success)
  {
    if (data != 0x00u)
    {
      PRINTF("Error in SN65DSI83 status 0xE5: 0x%02X\r\n", data);
      status = (status_t) SN65DSI83_WriteCSR(0xE5u, 0xFFu);
      if (status != kStatus_Success)
      {
        PRINTF("failed to clear SN65DSI83 status\r\n");
      }
    }
  }
  else
  {
    PRINTF("failed to read SN65DSI83 status\r\n");
  }

  return status;
}
