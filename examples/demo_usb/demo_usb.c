//******************************************************************************
/*!
 * \copyright
 * SPDX-License-Identifier: BSD-3-Clause
 * \copyright
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2017 NXP
 * All rights reserved.
 * \copyright
 * Copyright (c) 2021 - 2023 TQ-Systems GmbH <license@tq-group.com>,
 * D-82229 Seefeld, Germany.
 * Author: Isaac L. L. Yuki
 */
//******************************************************************************

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "usb_host_config.h"
#include "usb_host.h"
#include "fsl_device_registers.h"
#include "usb_host_msd.h"
#include "host_msd_command.h"
#include "fsl_common.h"
#include "clock_config.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT)                                     \
     && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
#include "fsl_sysmpu.h"
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */
#ifdef USE_LPI2C
#include "lpi2c_api.h"
#else
#error only LPI2C supported
#endif
#include "demo_usb.h"
#include "PCA9555BS.h"
#include "fsl_debug_console.h"

#if ((!USB_HOST_CONFIG_KHCI) && (!USB_HOST_CONFIG_EHCI)                        \
     && (!USB_HOST_CONFIG_OHCI) && (!USB_HOST_CONFIG_IP3516HS))
#error Please enable USB_HOST_CONFIG_KHCI, USB_HOST_CONFIG_EHCI, USB_HOST_CONFIG_OHCI, or USB_HOST_CONFIG_IP3516HS in file usb_host_config.
#endif

#include "usb_phy.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

// USE this define to change between usb otg over micro usb - USE_USB_OTG (1)
// and usb host over usb hub - USE_USB_OTG (0)
#define USE_USB_OTG (0)

#ifdef CONTROLLER_ID
#undef CONTROLLER_ID
#endif

/* For communication with port expander */
#define LPI2C_BASE_ADDR LPI2C3
#define LPI2C_BITRATE 100000
#define LPI2C_CLOCK_FREQUENCY BOARD_BOOTCLOCKRUN_LPI2C3_CLK_ROOT

#if (USE_USB_OTG)
#define CONTROLLER_ID kUSB_ControllerEhci0
#else
#define CONTROLLER_ID kUSB_ControllerEhci1
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static uint32_t PortExpander_transferFunction(
  void *peripheral, uint32_t regAddress, size_t regAddressSize,
  PCA9555BS_TransferDirection_t transferDirection, uint8_t *buffer,
  size_t bufferSize);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Device configuration */

/*Initialize peripheral*/
static LPI2C_masterDevice_t i2c_portexpader = {
  .base_addr                = LPI2C_BASE_ADDR,
  .masterConfig.baudRate_Hz = LPI2C_BITRATE,
  .mode                     = LPI2C_BlockingMode,
  .masterXfer.flags         = kLPI2C_TransferDefaultFlag,
  .masterXfer.slaveAddress  = PORTEXP_I2C_DEVICE_ADR,
};

/* Portexpander for enabling USB_MAIN_UART */
static PCA9555BS_Handle_t portExpander = {.peripheral = &i2c_portexpader,
                                          .IOMask.PIN00_PIN07_Mask       = 0xF0,
                                          .IOMask.PIN10_PIN17_Mask       = 0x00,
                                          .PolarityMask.PIN00_PIN07_Mask = 0x00,
                                          .PolarityMask.PIN10_PIN17_Mask = 0x00,
                                          .transferFunction =
                                            PortExpander_transferFunction};

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * \brief host callback function.
 * device attach/detach callback function.
 * \param deviceHandle        device handle.
 * \param configurationHandle attached device's configuration descriptor
 * information.
 * \param eventCode           callback event code, please reference to
 * enumeration host_event_t.
 * \retval kStatus_USB_Success              The host is initialized
 * successfully.
 * \retval kStatus_USB_NotSupported         The application doesn't support the
 * configuration.
 */
static usb_status_t USB_HostEvent(
  usb_device_handle             deviceHandle,
  usb_host_configuration_handle configurationHandle, uint32_t eventCode);

/*!
 * @brief application initialization.
 */
static void USB_HostApplicationInit(void);

static void USB_HostTask(void *param);

static void USB_HostApplicationTask(void *param);

extern void USB_HostClockInit(void);

extern void USB_HostIsrEnable(void);

extern void USB_HostTaskFn(void *param);

void BOARD_InitHardware(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Allocate the memory for the heap. */
#if defined(configAPPLICATION_ALLOCATED_HEAP)                                  \
  && (configAPPLICATION_ALLOCATED_HEAP)
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t ucHeap[configTOTAL_HEAP_SIZE];
#endif
/*! @brief USB host msd command instance global variable */
extern usb_host_msd_command_instance_t g_MsdCommandInstance;
usb_host_handle                        g_HostHandle;

/*******************************************************************************
 * Code
 ******************************************************************************/

void USB_OTG1_IRQHandler(void)
{
  USB_HostEhciIsrFunction(g_HostHandle);
}

void USB_OTG2_IRQHandler(void)
{
  USB_HostEhciIsrFunction(g_HostHandle);
}

/*!
 * \brief Initialize the USB host clock.
 * This function initializes the clock for the USB host controller.
 * It sets the USB clock frequency and enables the necessary clocks
 * for the USB host controller specified by CONTROLLER_ID.
 */
void USB_HostClockInit(void)
{
  uint32_t                usbClockFreq;
  usb_phy_config_struct_t phyConfig = {
    BOARD_USB_PHY_D_CAL,
    BOARD_USB_PHY_TXCAL45DP,
    BOARD_USB_PHY_TXCAL45DM,
  };
  usbClockFreq = 24000000;
  if (CONTROLLER_ID == kUSB_ControllerEhci0)
  {
    CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, usbClockFreq);
    CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, usbClockFreq);
  }
  else
  {
    CLOCK_EnableUsbhs1PhyPllClock(kCLOCK_Usbphy480M, usbClockFreq);
    CLOCK_EnableUsbhs1Clock(kCLOCK_Usb480M, usbClockFreq);
  }
  USB_EhciPhyInit(CONTROLLER_ID, BOARD_XTAL0_CLK_HZ, &phyConfig);
}

/*!
 * \brief Enable the USB host interrupt service routine.
 * This function enables the interrupt service routine for the USB host
 * controller specified by CONTROLLER_ID. It sets the priority for the
 * interrupt and enables it.
 */
void USB_HostIsrEnable(void)
{
  uint8_t irqNumber;

  uint8_t usbHOSTEhciIrq[] = USBHS_IRQS;
  /* USB_HOST_CONFIG_EHCI */
  irqNumber = usbHOSTEhciIrq[CONTROLLER_ID - kUSB_ControllerEhci0];

/* Install isr, set priority, and enable IRQ. */
#if defined(__GIC_PRIO_BITS)
  GIC_SetPriority((IRQn_Type) irqNumber, USB_HOST_INTERRUPT_PRIORITY);
#else
  NVIC_SetPriority((IRQn_Type) irqNumber, USB_HOST_INTERRUPT_PRIORITY);
#endif
  EnableIRQ((IRQn_Type) irqNumber);
}

/*!
 * \brief USB Host task function.
 * This function calls the USB_HostEhciTaskFunction which handles the
 * EHCI task for the USB host. It is intended to be run in a continuous
 * loop by a dedicated task in the system.
 * \param param The host handle.
 */
void USB_HostTaskFn(void *param)
{
  USB_HostEhciTaskFunction(param);
}

/*!
 * \brief Handle USB host events.
 * This function handles events triggered by the USB host controller
 * and performs actions like attaching/detaching devices, enumeration, etc.
 * \param deviceHandle Handle to the USB device.
 * \param configurationHandle Handle to the USB host configuration.
 * \param eventCode Event code that specifies the triggered event.
 * \return usb_status_t Status of the event handling.
 * Returns kStatus_USB_Success if successful, error code otherwise.
 */
static usb_status_t USB_HostEvent(
  usb_device_handle             deviceHandle,
  usb_host_configuration_handle configurationHandle, uint32_t eventCode)
{
  usb_status_t status = kStatus_USB_Success;
  switch (eventCode & 0x0000FFFFU)
  {
  case kUSB_HostEventAttach:
    status = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
    break;

  case kUSB_HostEventNotSupported:
    usb_echo("device not supported.\r\n");
    break;

  case kUSB_HostEventEnumerationDone:
    status = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
    break;

  case kUSB_HostEventDetach:
    status = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
    break;

  case kUSB_HostEventEnumerationFail:
    usb_echo("enumeration failed\r\n");
    break;

  default:
    break;
  }
  return status;
}

/*!
 * \brief Initialize the USB host application.
 * This function initializes the USB host application. It initializes
 * the USB host clock, enables the SYSMPU (if applicable), initializes
 * the USB host, enables the USB ISR, and prints a message to indicate
 * successful initialization.
 */
static void USB_HostApplicationInit(void)
{
  usb_status_t status = kStatus_USB_Success;

  USB_HostClockInit();

#if ((defined FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT))
  SYSMPU_Enable(SYSMPU, 0);
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

  status = USB_HostInit(CONTROLLER_ID, &g_HostHandle, USB_HostEvent);
  if (status != kStatus_USB_Success)
  {
    usb_echo("host init error\r\n");
    return;
  }
  USB_HostIsrEnable();

  usb_echo("host init done\r\n");
}

static void USB_HostTask(void *param)
{
  while (1)
  {
    USB_HostTaskFn(param);
  }
}

static void USB_HostApplicationTask(void *param)
{
  while (1)
  {
    USB_HostMsdTask(param);
  }
}

/*!
 * \brief  This is the main application entry point where system is initialized
 *         and all system components are started.
 */
int main(void)
{
  BOARD_Initialize();
  PRINTF("This demonstration shows how to operate USB on the %s", BOARD_NAME);
  GETCHAR();
  /*Init port expander*/
  LPI2C_MasterGetDefaultConfig(&i2c_portexpader.masterConfig);
  i2c_portexpader.masterConfig.baudRate_Hz = LPI2C_BITRATE;
  LPI2C_MasterInit(i2c_portexpader.base_addr, &i2c_portexpader.masterConfig,
                   LPI2C_CLOCK_FREQUENCY);
  PCA9555BS_configure(&portExpander);

  /*Toggle USB*/
  portExpander.OutputMask.PIN10_PIN17_Mask = 0x1;
  PCA9555BS_setOutput(&portExpander);
  SDK_DelayAtLeastUs(50000, SystemCoreClock);
  portExpander.OutputMask.PIN10_PIN17_Mask = 0x0;
  PCA9555BS_setOutput(&portExpander);
  SDK_DelayAtLeastUs(50000, SystemCoreClock);
  PRINTF("\r\n[ OK ]\t Toggle done. \r\n");
  /* Enable UART bridge */
  portExpander.OutputMask.PIN10_PIN17_Mask = 0x01;
  PCA9555BS_setOutput(&portExpander);

  USB_HostApplicationInit();

  if (xTaskCreate(USB_HostTask, "usb host task", 2000L / sizeof(portSTACK_TYPE),
                  g_HostHandle, 4, NULL)
      != pdPASS)
  {
    usb_echo("create host task error\r\n");
  }

  if (xTaskCreate(USB_HostApplicationTask, "app task",
                  2000L / sizeof(portSTACK_TYPE), &g_MsdCommandInstance, 3,
                  NULL)
      != pdPASS)
  {
    usb_echo("create msd task error\r\n");
  }

  vTaskStartScheduler();

  while (1)
  {
    ;
  }
}

/*!
 * \brief Implementation of the PCA9555BS transfer function.
 */
static uint32_t PortExpander_transferFunction(
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
#error only LPI2C supported
#endif

  return (uint32_t) status;
}
