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

#ifndef ENET_LWIP_ENET_QOS_SRC_ENET_QOS_FUNC_H_
#define ENET_LWIP_ENET_QOS_SRC_ENET_QOS_FUNC_H_

/*******************************************************************************
 * Defines
 ******************************************************************************/
/* IP address configuration. */
#define configIP_ADDR0 10
#define configIP_ADDR1 255
#define configIP_ADDR2 255
#define configIP_ADDR3 102

/* Netmask configuration. */
#define configNET_MASK0 255
#define configNET_MASK1 255
#define configNET_MASK2 255
#define configNET_MASK3 0

/* Gateway address configuration. */
#define configGW_ADDR0 10
#define configGW_ADDR1 255
#define configGW_ADDR2 255
#define configGW_ADDR3 1

/* MAC address configuration. */
#define configMAC_ADDR                                                         \
  {                                                                            \
    0x02, 0x12, 0x13, 0x10, 0x15, 0x11                                         \
  }

/* Address of PHY interface. */
#define EXAMPLE_PHY_ADDRESS 0x04U // 0x01U

/* MDIO operations. */
#define EXAMPLE_MDIO_OPS enet_qos_ops

/* PHY operations. */
#define EXAMPLE_PHY_OPS &phydp83867_ops
#define EXAMPLE_PHY_RESOURCE &g_phy_resource

/* ENET clock frequency. */
#define EXAMPLE_CLOCK_FREQ CLOCK_GetRootClockFreq(kCLOCK_Root_Bus)

/* ENET IRQ priority. Used in FreeRTOS. */
#ifndef ENET_PRIORITY
#define ENET_PRIORITY (6U)
#endif

#ifndef EXAMPLE_NETIF_INIT_FN
/*! @brief Network interface initialization function. */
#define EXAMPLE_NETIF_INIT_FN ethernetif0_init
#endif /* EXAMPLE_NETIF_INIT_FN */

/*! @brief Stack size of the temporary lwIP initialization thread. */
#define INIT_THREAD_STACKSIZE 1024

/*! @brief Priority of the temporary lwIP initialization thread. */
#define INIT_THREAD_PRIO DEFAULT_THREAD_PRIO

/*Reset Pin for Ethernet Moduls*/
#define RESET_ETH BOARD_INITPINS_RESET_ETH_GPIO
#define RESET_ETH_PIN BOARD_INITPINS_RESET_ETH_GPIO_PIN

/******************************************************************************/

#endif /* ENET_LWIP_ENET_QOS_SRC_ENET_QOS_FUNC_H_ */

/*[EOF]************************************************************************/
