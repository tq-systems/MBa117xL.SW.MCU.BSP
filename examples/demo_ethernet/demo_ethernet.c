//*****************************************************************************
/*!
 * \copyright
 * SPDX-License-Identifier: BSD-3-Clause
 * \copyright
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2020,2022 NXP
 * All rights reserved.
 * \copyright
 * Copyright (c) 2021 - 2023 TQ-Systems GmbH <license@tq-group.com>,
 * D-82229 Seefeld, Germany.
 * Author: Bernhard Herz, Isaac L. L. Yuki
 */
//******************************************************************************

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "lwip/opt.h"
#if LWIP_IPV4 && LWIP_RAW

#include "ping.h"
#include "lwip/timeouts.h"
#include "lwip/init.h"
#include "netif/ethernet.h"
#include "ethernetif.h"

#include "pin_mux.h"
#include "board.h"
#ifndef configMAC_ADDR
// #include "fsl_silicon_id.h"
/* MAC address configuration. */
#define configMAC_ADDR                                                         \
  {                                                                            \
    0x02, 0x12, 0x13, 0x10, 0x15, 0x11                                         \
  }

#endif
#include "fsl_phy.h"
#include "DP83867.h"
#include "fsl_enet_qos.h"
#include "demo_ethernet.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

static phy_dp83867_resource_t g_phy_resource;
static phy_handle_t           phyHandle = {.ops      = EXAMPLE_PHY_OPS,
                                           .phyAddr  = EXAMPLE_PHY_ADDRESS,
                                           .resource = EXAMPLE_PHY_RESOURCE};

/*******************************************************************************
 * Code
 ******************************************************************************/

void BOARD_InitModuleClock(void)
{
  /* Select syspll2pfd3, 528*18/24 = 396M */
  CLOCK_InitPfd(kCLOCK_PllSys2, kCLOCK_Pfd3, 24);
  const clock_sys_pll1_config_t sysPll1Config = {
    .pllDiv2En = true,
  };
  CLOCK_InitSysPll1(&sysPll1Config);
  clock_root_config_t rootCfg = {.mux = 4,
                                 .div = 4}; /* Generate 125M root clock. */
  CLOCK_SetRootClock(kCLOCK_Root_Enet_Qos, &rootCfg);
  rootCfg.div = 10;
  CLOCK_SetRootClock(kCLOCK_Root_Enet_Timer3,
                     &rootCfg); /* Generate 50M PTP REF clock. */

  rootCfg.mux = 7;
  rootCfg.div = 2;
  CLOCK_SetRootClock(kCLOCK_Root_Bus, &rootCfg); /* Generate 198M bus clock. */
}

void BOARD_UpdateENETModuleClock(enet_qos_mii_speed_t miiSpeed)
{
  /* ENET_QOS clock source: Select SysPll1Div2, 1G/2 = 500M */
  clock_root_config_t rootCfg = {.mux = 4};

  switch (miiSpeed)
  {
  case kENET_QOS_MiiSpeed1000M:
    /* Generate 125M root clock for 1000Mbps. */
    rootCfg.div = 4U;
    break;
  case kENET_QOS_MiiSpeed100M:
    /* Generate 25M root clock for 100Mbps. */
    rootCfg.div = 20U;
    break;
  case kENET_QOS_MiiSpeed10M:
    /* Generate 2.5M root clock for 10Mbps. */
    rootCfg.div = 200U;
    break;
  default:
    /* Generate 125M root clock. */
    rootCfg.div = 4U;
    break;
  }
  CLOCK_SetRootClock(kCLOCK_Root_Enet_Qos, &rootCfg);
}

void ENET_QOS_EnableClock(bool enable)
{
  IOMUXC_GPR->GPR6 =
    (IOMUXC_GPR->GPR6 & (~IOMUXC_GPR_GPR6_ENET_QOS_CLKGEN_EN_MASK))
    | IOMUXC_GPR_GPR6_ENET_QOS_CLKGEN_EN(enable);
}
void ENET_QOS_SetSYSControl(enet_qos_mii_mode_t miiMode)
{
  IOMUXC_GPR->GPR6 =
    (IOMUXC_GPR->GPR6 & (~IOMUXC_GPR_GPR6_ENET_QOS_INTF_SEL_MASK))
    | IOMUXC_GPR_GPR6_ENET_QOS_INTF_SEL(miiMode);
}

static void MDIO_Init(void)
{
  CLOCK_EnableClock(s_enetqosClock[ENET_QOS_GetInstance(ENET_QOS)]);
  ENET_QOS_SetSMI(ENET_QOS, EXAMPLE_CLOCK_FREQ);
}

static status_t MDIO_Write(uint8_t phyAddr, uint8_t regAddr, uint16_t data)
{
  return ENET_QOS_MDIOWrite(ENET_QOS, phyAddr, regAddr, data);
}

static status_t MDIO_Read(uint8_t phyAddr, uint8_t regAddr, uint16_t *pData)
{
  return ENET_QOS_MDIORead(ENET_QOS, phyAddr, regAddr, pData);
}

/*!
 * @brief Interrupt service for SysTick timer.
 */
void SysTick_Handler(void)
{
  time_isr();
}

/*!
 * @brief Main function.
 */
int main(void)
{
  struct netif        netif;
  ip4_addr_t          netif_ipaddr, netif_netmask, netif_gw;
  ethernetif_config_t enet_config = {.phyHandle   = &phyHandle,
                                     .phyAddr     = EXAMPLE_PHY_ADDRESS,
                                     .phyOps      = EXAMPLE_PHY_OPS,
                                     .phyResource = &g_phy_resource,
#ifdef configMAC_ADDR
                                     .macAddress = configMAC_ADDR
#endif
  };

  // gpio_pin_config_t gpio_config = { kGPIO_DigitalOutput, 0, kGPIO_NoIntmode
  // };

  /* Hardware Initialization. */
  BOARD_Initialize();
  BOARD_InitModuleClock();

  IOMUXC_GPR->GPR6 |= IOMUXC_GPR_GPR6_ENET_QOS_RGMII_EN_MASK;

  IOMUXC_GPR->GPR6 |= IOMUXC_GPR_GPR6_ENET_QOS_RGMII_EN_MASK;
  /* Set this bit to enable ENET_QOS RGMII TX clock output on TX_CLK pad. */

  // GPIO_PinInit(GPIO11, 14, &gpio_config);
  /* For a complete PHY reset of RTL8211FDI-CG, this pin must be asserted low
   * for at least 10ms. And wait for a further 30ms(for internal circuits
   * settling time) before accessing the PHY register */

  /*Reset ETPPHY*/
  GPIO_WritePinOutput(RESET_ETH, RESET_ETH_PIN, 0); // Set of RST_ETH Pin
  SDK_DelayAtLeastUs(10000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
  GPIO_WritePinOutput(RESET_ETH, RESET_ETH_PIN, 1); // Reset of RST_ETH Pin
  SDK_DelayAtLeastUs(30000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);

  EnableIRQ(ENET_1G_MAC0_Tx_Rx_1_IRQn);
  EnableIRQ(ENET_1G_MAC0_Tx_Rx_2_IRQn);

  MDIO_Init();
  g_phy_resource.read  = MDIO_Read;
  g_phy_resource.write = MDIO_Write;

  time_init();

  /* Set MAC address. */
#ifndef configMAC_ADDR
  (void) SILICONID_ConvertToMacAddr(&enet_config.macAddress);
#endif

  /* Get clock after hardware init. */
  enet_config.srcClockHz = EXAMPLE_CLOCK_FREQ;

  IP4_ADDR(&netif_ipaddr, configIP_ADDR0, configIP_ADDR1, configIP_ADDR2,
           configIP_ADDR3);
  IP4_ADDR(&netif_netmask, configNET_MASK0, configNET_MASK1, configNET_MASK2,
           configNET_MASK3);
  IP4_ADDR(&netif_gw, configGW_ADDR0, configGW_ADDR1, configGW_ADDR2,
           configGW_ADDR3);

  lwip_init();

  netif_add(&netif, &netif_ipaddr, &netif_netmask, &netif_gw, &enet_config,
            EXAMPLE_NETIF_INIT_FN, ethernet_input);
  netif_set_default(&netif);
  netif_set_up(&netif);

  while (ethernetif_wait_linkup(&netif, 5000) != ERR_OK)
  {
    PRINTF("PHY Auto-negotiation failed. Please check the cable connection "
           "and link partner setting.\r\n");
  }

  ping_init(&netif_gw);

  PRINTF("\r\n************************************************\r\n");
  PRINTF(" PING example\r\n");
  PRINTF("************************************************\r\n");
  PRINTF(" IPv4 Address     : %u.%u.%u.%u\r\n", ((u8_t *) &netif_ipaddr)[0],
         ((u8_t *) &netif_ipaddr)[1], ((u8_t *) &netif_ipaddr)[2],
         ((u8_t *) &netif_ipaddr)[3]);
  PRINTF(" IPv4 Subnet mask : %u.%u.%u.%u\r\n", ((u8_t *) &netif_netmask)[0],
         ((u8_t *) &netif_netmask)[1], ((u8_t *) &netif_netmask)[2],
         ((u8_t *) &netif_netmask)[3]);
  PRINTF(" IPv4 Gateway     : %u.%u.%u.%u\r\n", ((u8_t *) &netif_gw)[0],
         ((u8_t *) &netif_gw)[1], ((u8_t *) &netif_gw)[2],
         ((u8_t *) &netif_gw)[3]);
  PRINTF("************************************************\r\n");

  while (1)
  {

    /* Poll the driver, get any outstanding frames */
    ethernetif_input(&netif);

    sys_check_timeouts(); /* Handle all system timeouts for all core protocols
                           */
  }
}
#endif
