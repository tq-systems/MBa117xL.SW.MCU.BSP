//******************************************************************************
/*!
 * \copyright
 * SPDX-License-Identifier: BSD-3-Clause
 * \copyright
 * Copyright (c) 2021 - 2023 TQ-Systems GmbH <license@tq-group.com>,
 * D-82229 Seefeld, Germany.
 * Author: Maximilian Kürth
 */
//******************************************************************************

/*!
 * \addtogroup DP83867
 * @{
 * \file
 */

#include "DP83867.h"

/*! @brief Defines the PHY DP83867 ID number. */
#define PHY_CONTROL_ID1 0x2000 /*!< The PHY ID1 . */

#define PHY_RGMII_CTR_REG 0x0032U /*RGMII Control Register*/
#define PHY_RGMII_EN_MASK 0x0040U /*RGMII Enable Mask*/

#define PHY_RGMII_TXRX_DELAY_CTR_REG 0x0086 /*RGMII Delay Control Register*/
#define PHY_RGMII_TX_DELAY_MASK 0x00F0U     /*!< The RGMII TXC delay mask. */
#define PHY_RGMII_RX_DELAY_MASK 0x000FU     /*!< The RGMII RXC delay mask. */

#define PHY_REG_CTR_REG 0x000D /*Register Control Register*/
#define PHY_DEVAD_MASK 0x000F  /*Device Address Mask*/

/*! @brief ADDAR used with REGCR to access by indirect R/W mechanism */
#define PHY_ADD_OR_DATA_REG 0x000E /*Address or Data Register*/

#define PHY_MMD_DEVICE_ADDR 0x1F /*General MMD Device Address*/

#define PHY_STATUS_REG 0x11U        /*PHY Status Register*/
#define PHY_SPEED_MASK 0xC000       /*Speed Select Status Mask*/
#define PHY_LINK_STATUS_MASK 0x0400 /*Link Status Mask*/
#define PHY_DUPLEX_MODE_MASK 0x2000 /*Duplex Mode Mask*/
#define PHY_SPEED_SHIFT 14U         /*Speed Select Status Shift*/

/*! @brief Defines the PHY RTL8211F vendor defined registers. */
#define PHY_SPECIFIC_STATUS_REG                                                \
  PHY_STATUS_REG // 0x1AU /*!< The PHY specific status register. */
#define PHY_PAGE_SELECT_REG 0x1FU /*!< The PHY page select register. */

/*! @brief Defines the mask flag in specific status register. */
#define PHY_SSTATUS_LINKSTATUS_MASK                                            \
  PHY_LINK_STATUS_MASK // 0x04U /*!< The PHY link status mask. */
#define PHY_SSTATUS_LINKSPEED_MASK                                             \
  PHY_SPEED_MASK // 0x30U /*!< The PHY link speed mask. */
#define PHY_SSTATUS_LINKDUPLEX_MASK                                            \
  PHY_DUPLEX_MODE_MASK // 0x08U /*!< The PHY link duplex mask. */
#define PHY_SSTATUS_LINKSPEED_SHIFT                                            \
  PHY_SPEED_SHIFT // 4U    /*!< The link speed shift */

/*! @brief Defines the PHY RTL8211F extra page and the registers in specified
 * page. */
#define PHY_PAGE_RGMII_TXRX_DELAY_ADDR                                         \
  0xD08U /*!< The register page including RGMII TX/RX delay setting. */

/*! @brief Defines the timeout macro. */
#define PHY_READID_TIMEOUT_COUNT 1000U

#define PHY_DP83867_WRITE(handle, regAddr, data)                               \
  ((phy_dp83867_resource_t *) (handle)->resource)                              \
    ->write((handle)->phyAddr, regAddr, data)
#define PHY_DP83867_READ(handle, regAddr, pData)                               \
  ((phy_dp83867_resource_t *) (handle)->resource)                              \
    ->read((handle)->phyAddr, regAddr, pData)

const phy_operations_t phydp83867_ops = {
  .phyInit            = PHY_DP83867_Init,
  .phyWrite           = PHY_DP83867_Write,
  .phyRead            = PHY_DP83867_Read,
  .getAutoNegoStatus  = PHY_DP83867_GetAutoNegotiationStatus,
  .getLinkStatus      = PHY_DP83867_GetLinkStatus,
  .getLinkSpeedDuplex = PHY_DP83867_GetLinkSpeedDuplex,
  .setLinkSpeedDuplex = PHY_DP83867_SetLinkSpeedDuplex,
  .enableLoopback     = PHY_DP83867_EnableLoopback};

/*!
 *
 */
status_t PHY_DP83867_Init(phy_handle_t *handle, const phy_config_t *config)
{
  uint32_t counter = PHY_READID_TIMEOUT_COUNT;
  status_t result;
  uint16_t regValue = 0U;

  /* Check PHY ID. */
  do
  {
    result = PHY_DP83867_READ(handle, PHY_ID1_REG, &regValue);
    if (result != kStatus_Success)
    {
      return result;
    }
    counter--;
  } while ((regValue != PHY_CONTROL_ID1) && (counter != 0U));

  if (counter == 0U)
  {
    return kStatus_Fail;
  }

  /* Reset PHY. */
  result = PHY_DP83867_WRITE(handle, PHY_BASICCONTROL_REG, PHY_BCTL_RESET_MASK);
  if (result != kStatus_Success)
  {
    return result;
  }

  //    /* The RGMII specifies output TXC/RXC and TXD/RXD without any clock
  //    skew. Need to add skew on clock line
  //       to make sure the other side sample right data. This can also be done
  //       in PCB traces. */
  //    result = MDIO_Write(handle->mdioHandle, handle->phyAddr,
  //    PHY_PAGE_SELECT_REG, PHY_PAGE_RGMII_TXRX_DELAY_ADDR); if (result !=
  //    kStatus_Success)
  //    {
  //        return result;
  //    }
  //

  /* Set Tx Delay. */
  result = PHY_DP83867_READ(handle, PHY_RGMII_TXRX_DELAY_CTR_REG, &regValue);
  if (result == kStatus_Success)
  {
    regValue |= PHY_RGMII_TX_DELAY_MASK;
    result = PHY_DP83867_WRITE(handle, PHY_RGMII_TXRX_DELAY_CTR_REG, regValue);
    if (result != kStatus_Success)
    {
      return result;
    }
  }
  else
  {
    return result;
  }
  /* Set Rx Delay. */
  result = PHY_DP83867_READ(handle, PHY_RGMII_TXRX_DELAY_CTR_REG, &regValue);
  if (result == kStatus_Success)
  {
    regValue |= PHY_RGMII_RX_DELAY_MASK;
    result = PHY_DP83867_WRITE(handle, PHY_RGMII_TXRX_DELAY_CTR_REG, regValue);
    if (result != kStatus_Success)
    {
      return result;
    }
  }
  else
  {
    return result;
  }

  /* set LEDs active low */
  result = PHY_DP83867_Read(handle, 0x19, &regValue);
  result = PHY_DP83867_Write(handle, 0x19, (regValue & ~0x0444));

  if (config->autoNeg)
  {
    /* Set the auto-negotiation. */
    result = PHY_DP83867_WRITE(
      handle, PHY_AUTONEG_ADVERTISE_REG,
      PHY_100BASETX_FULLDUPLEX_MASK | PHY_100BASETX_HALFDUPLEX_MASK
        | PHY_10BASETX_FULLDUPLEX_MASK | PHY_10BASETX_HALFDUPLEX_MASK
        | PHY_IEEE802_3_SELECTOR_MASK);
    if (result == kStatus_Success)
    {
      result = PHY_DP83867_WRITE(handle, PHY_1000BASET_CONTROL_REG,
                                 PHY_1000BASET_FULLDUPLEX_MASK);
      if (result == kStatus_Success)
      {
        result = PHY_DP83867_READ(handle, PHY_BASICCONTROL_REG, &regValue);
        if (result == kStatus_Success)
        {
          result = PHY_DP83867_WRITE(
            handle, PHY_BASICCONTROL_REG,
            (regValue | PHY_BCTL_AUTONEG_MASK | PHY_BCTL_RESTART_AUTONEG_MASK));
        }
      }
    }
  }
  else
  {
    /* Disable isolate mode */
    result = PHY_DP83867_READ(handle, PHY_BASICCONTROL_REG, &regValue);
    if (result != kStatus_Success)
    {
      return result;
    }
    regValue &= PHY_BCTL_ISOLATE_MASK;
    result = PHY_DP83867_WRITE(handle, PHY_BASICCONTROL_REG, regValue);
    if (result != kStatus_Success)
    {
      return result;
    }

    /* Disable the auto-negotiation and set user-defined speed/duplex
     * configuration. */
    result =
      PHY_DP83867_SetLinkSpeedDuplex(handle, config->speed, config->duplex);
  }
  return result;
}
/*!
 *
 */
status_t PHY_DP83867_Write(phy_handle_t *handle, uint8_t phyReg, uint16_t data)
{
  return PHY_DP83867_WRITE(handle, phyReg, data);
}
/*!
 *
 */
status_t PHY_DP83867_Read(phy_handle_t *handle, uint8_t phyReg,
                          uint16_t *dataPtr)
{
  return PHY_DP83867_READ(handle, phyReg, dataPtr);
}
/*!
 *
 */
status_t PHY_DP83867_GetAutoNegotiationStatus(phy_handle_t *handle,
                                              bool         *status)
{
  assert(status);

  status_t result;
  uint16_t regValue;

  *status = false;

  /* Check auto negotiation complete. */
  result = PHY_DP83867_READ(handle, PHY_BASICSTATUS_REG, &regValue);
  if (result == kStatus_Success)
  {
    if ((regValue & PHY_BSTATUS_AUTONEGCOMP_MASK) != 0U)
    {
      *status = true;
    }
  }
  return result;
}
/*!
 *
 */
status_t PHY_DP83867_GetLinkStatus(phy_handle_t *handle, bool *status)
{
  assert(status);

  status_t result;
  uint16_t regValue;

  /* Read the basic status register. */
  result = PHY_DP83867_READ(handle, PHY_SPECIFIC_STATUS_REG, &regValue);
  if (result == kStatus_Success)
  {
    if ((PHY_SSTATUS_LINKSTATUS_MASK & regValue) != 0U)
    {
      /* Link up. */
      *status = true;
    }
    else
    {
      /* Link down. */
      *status = false;
    }
  }
  return result;
}
/*!
 *
 */
status_t PHY_DP83867_GetLinkSpeedDuplex(phy_handle_t *handle,
                                        phy_speed_t  *speed,
                                        phy_duplex_t *duplex)
{
  assert(!((speed == NULL) && (duplex == NULL)));

  status_t result;
  uint16_t regValue;

  /* Read the status register. */
  result = PHY_DP83867_READ(handle, PHY_SPECIFIC_STATUS_REG, &regValue);
  if (result == kStatus_Success)
  {
    if (speed != NULL)
    {
      switch ((regValue & PHY_SSTATUS_LINKSPEED_MASK)
              >> PHY_SSTATUS_LINKSPEED_SHIFT)
      {
      case (uint32_t) kPHY_Speed10M:
        *speed = kPHY_Speed10M;
        break;
      case (uint32_t) kPHY_Speed100M:
        *speed = kPHY_Speed100M;
        break;
      case (uint32_t) kPHY_Speed1000M:
        *speed = kPHY_Speed1000M;
        break;
      default:
        *speed = kPHY_Speed10M;
        break;
      }
    }

    if (duplex != NULL)
    {
      if ((regValue & PHY_SSTATUS_LINKDUPLEX_MASK) != 0U)
      {
        *duplex = kPHY_FullDuplex;
      }
      else
      {
        *duplex = kPHY_HalfDuplex;
      }
    }
  }
  return result;
}
/*!
 *
 */
status_t PHY_DP83867_SetLinkSpeedDuplex(phy_handle_t *handle, phy_speed_t speed,
                                        phy_duplex_t duplex)
{
  status_t result;
  uint16_t regValue;

  result = PHY_DP83867_READ(handle, PHY_BASICCONTROL_REG, &regValue);
  if (result == kStatus_Success)
  {
    /* Disable the auto-negotiation and set according to user-defined
     * configuration. */
    regValue &= ~PHY_BCTL_AUTONEG_MASK;
    if (speed == kPHY_Speed1000M)
    {
      regValue &= PHY_BCTL_SPEED0_MASK;
      regValue |= PHY_BCTL_SPEED1_MASK;
    }
    else if (speed == kPHY_Speed100M)
    {
      regValue |= PHY_BCTL_SPEED0_MASK;
      regValue &= ~PHY_BCTL_SPEED1_MASK;
    }
    else
    {
      regValue &= ~PHY_BCTL_SPEED0_MASK;
      regValue &= ~PHY_BCTL_SPEED1_MASK;
    }
    if (duplex == kPHY_FullDuplex)
    {
      regValue |= PHY_BCTL_DUPLEX_MASK;
    }
    else
    {
      regValue &= ~PHY_BCTL_DUPLEX_MASK;
    }
    result = PHY_DP83867_WRITE(handle, PHY_BASICCONTROL_REG, regValue);
  }
  return result;
}
/*!
 *
 */
status_t PHY_DP83867_EnableLoopback(phy_handle_t *handle, phy_loop_t mode,
                                    phy_speed_t speed, bool enable)
{
  /* This PHY only supports local loopback. */
  assert(mode == kPHY_LocalLoop);

  status_t result;
  uint16_t regValue;

  /* Set the loop mode. */
  if (enable)
  {
    if (speed == kPHY_Speed1000M)
    {
      regValue =
        PHY_BCTL_SPEED1_MASK | PHY_BCTL_DUPLEX_MASK | PHY_BCTL_LOOP_MASK;
    }
    else if (speed == kPHY_Speed100M)
    {
      regValue =
        PHY_BCTL_SPEED0_MASK | PHY_BCTL_DUPLEX_MASK | PHY_BCTL_LOOP_MASK;
    }
    else
    {
      regValue = PHY_BCTL_DUPLEX_MASK | PHY_BCTL_LOOP_MASK;
    }
    result = PHY_DP83867_WRITE(handle, PHY_BASICCONTROL_REG, regValue);
  }
  else
  {
    /* First read the current status in control register. */
    result = PHY_DP83867_READ(handle, PHY_BASICCONTROL_REG, &regValue);
    if (result == kStatus_Success)
    {
      regValue &= ~PHY_BCTL_LOOP_MASK;
      result = PHY_DP83867_WRITE(handle, PHY_BASICCONTROL_REG,
                                 (regValue | PHY_BCTL_RESTART_AUTONEG_MASK));
    }
  }
  return result;
}

/*!@}*/
