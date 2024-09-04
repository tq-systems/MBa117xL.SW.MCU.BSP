/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2020,2022 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

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
#include "fsl_phy.h"
#include "DP83867.h"
#include "fsl_enet.h"
#include "demo_ethernet.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Address of PHY interface. */
#define EXAMPLE_PHY_ADDRESS 0x03U

/* MDIO operations. */
#define EXAMPLE_ENET ENET_1G

#ifndef EXAMPLE_NETIF_INIT_FN
/*! @brief Network interface initialization function. */
#define EXAMPLE_NETIF_INIT_FN ethernetif1_init
#endif /* EXAMPLE_NETIF_INIT_FN */

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
  clock_sys_pll1_config_t const sysPllConfig = {
    .pllDiv2En = true,
  };
  clock_root_config_t rootCfg = {
    .mux = 4U,
    .div = 4U,
  };
  CLOCK_InitSysPll1(&sysPllConfig);
  CLOCK_SetRootClock(kCLOCK_Root_Enet2, &rootCfg);
}

void BOARD_ENETFlexibleConfigure(enet_config_t *config)
{
  config->miiMode = kENET_RgmiiMode;
}

static void MDIO_Init(void)
{
  (void) CLOCK_EnableClock(s_enetClock[ENET_GetInstance(EXAMPLE_ENET)]);
  ENET_SetSMI(EXAMPLE_ENET, EXAMPLE_CLOCK_FREQ, false);
}

static status_t MDIO_Write(uint8_t phyAddr, uint8_t regAddr, uint16_t data)
{
  return ENET_MDIOWrite(EXAMPLE_ENET, phyAddr, regAddr, data);
}

static status_t MDIO_Read(uint8_t phyAddr, uint8_t regAddr, uint16_t *pData)
{
  return ENET_MDIORead(EXAMPLE_ENET, phyAddr, regAddr, pData);
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
                                     .phyResource = EXAMPLE_PHY_RESOURCE,
                                     .srcClockHz  = EXAMPLE_CLOCK_FREQ,
#ifdef configMAC_ADDR
                                     .macAddress = configMAC_ADDR
#endif
  };

  gpio_pin_config_t gpio_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

  BOARD_Initialize();
  BOARD_InitModuleClock();

  IOMUXC_GPR->GPR5 |= IOMUXC_GPR_GPR5_ENET1G_RGMII_EN_MASK;

  GPIO_PinInit(RESET_ETH, RESET_ETH_PIN, &gpio_config);
  GPIO_WritePinOutput(RESET_ETH, RESET_ETH_PIN, 0); // Set of RST_ETH Pin
  SDK_DelayAtLeastUs(50000, CLOCK_GetFreq(kCLOCK_CpuClk));
  GPIO_WritePinOutput(RESET_ETH, RESET_ETH_PIN, 1); // Set of RST_ETH Pin
  SDK_DelayAtLeastUs(50000, CLOCK_GetFreq(kCLOCK_CpuClk));

  EnableIRQ(ENET_1G_MAC0_Tx_Rx_1_IRQn);
  EnableIRQ(ENET_1G_MAC0_Tx_Rx_2_IRQn);

  MDIO_Init();
  g_phy_resource.read  = MDIO_Read;
  g_phy_resource.write = MDIO_Write;

  time_init();

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
    PRINTF("PHY Auto-negotiation failed. Please check the cable connection and "
           "link partner setting.\r\n");
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

    sys_check_timeouts(); /* Handle all system timeouts for all core protocols */
  }
}
#endif
