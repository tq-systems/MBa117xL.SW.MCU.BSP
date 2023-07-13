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

#ifndef _PHY_DP83867_H_
#define _PHY_DP83867_H_

/*!
 * \defgroup DP83867 DP83867
 * \brief Driver for the DP83867 PHY.
 * @{
 * \file
 */

#include "fsl_phy.h"

/*!
 * \brief Structure containing all LAN8720Ai operations to match FSL API.
 */
extern const phy_operations_t phydp83867_ops;

/*!
 * \brief Contains all MDIO access functions used to configure the PHY.
 */
typedef struct _phy_dp83867_resource
{
  mdioWrite    write; //! Provides the MDIO write funtion.
  mdioRead     read;  //! Provides the MDIO read function.
  mdioWriteExt writeExt;
  mdioReadExt  readExt;
} phy_dp83867_resource_t;

/*!
 * \brief Initializes the PHY in regard of the provided configuration.
 *
 * \param[in] handle is the PHY handle for e. g. MDIO access.
 * \param[in] config is the configuration of the PHY.
 * \return <code>kStatus_Success</code> if the initialization was successful.
 */
extern status_t PHY_DP83867_Init(phy_handle_t       *handle,
                                 const phy_config_t *config);

/*!
 * \brief Writes the provided register address of the PHY.
 *
 * \param[in] handle is the PHY handle for e. g. MDIO access.
 * \param[in] phyReg is the PHY register address to write.
 * \param[in] data   contains the new value for the register.
 * \return <code>kStatus_Success</code> if writing the PHY register was
 * successful.
 */
extern status_t PHY_DP83867_Write(phy_handle_t *handle, uint8_t phyReg,
                                  uint16_t data);

/*!
 * \brief Reads the provided register address of the PHY.
 *
 * \param[in]  handle  is the PHY handle for e. g. MDIO access.
 * \param[in]  phyReg  is the PHY register address to read.
 * \param[out] dataPtr will contain the current value of the register.
 * \return <code>kStatus_Success</code> if reading the PHY register was
 * successful.
 */
extern status_t PHY_DP83867_Read(phy_handle_t *handle, uint8_t phyReg,
                                 uint16_t *dataPtr);

/*!
 * \brief Gets the auto negotiation status of the PHY.
 *
 * \param[in] handle is the PHY handle for e. g. MDIO access.
 * \param[in] status will contain the auto negotiation status of the PHY.
 * \return <code>kStatus_Success</code> if accessing the auto negotiation status
 * was successful.
 */
extern status_t PHY_DP83867_GetAutoNegotiationStatus(phy_handle_t *handle,
                                                     bool         *status);

/*!
 * \brief Gets the link status of the PHY.
 *
 * \param[in]  handle is the PHY handle for e. g. MDIO access.
 * \param[out] status will contain the current link status.
 * \return <code>kStatus_Success</code> is accessing the link status was
 * successful.
 */
extern status_t PHY_DP83867_GetLinkStatus(phy_handle_t *handle, bool *status);

/*!
 * \brief Gets link speed and duplex mode of the PHY.
 *
 * \param[in]  handle is the PHY handle for e. g. MDIO access.
 * \param[out] speed  will contain the current speed if not NULL.
 * \param[out] duplex will contain the current duplex mode if not NULL.
 * \return <code>kStatus_Success</code> if link speed and duplex mode were read
 * successfully.
 */
extern status_t PHY_DP83867_GetLinkSpeedDuplex(phy_handle_t *handle,
                                               phy_speed_t  *speed,
                                               phy_duplex_t *duplex);

/*!
 * \brief Sets link speed and duplex mode of the PHY.
 *
 * \param[in] handle is the PHY handle for e. g. MDIO access.
 * \param[in] speed  is the link speed to set.
 * \param[in] duplex is the duplex mode to set.
 * \return <code>kStatus_Success</code> if link speed and duplex mode was
 * adjusted accordingly.
 */
extern status_t PHY_DP83867_SetLinkSpeedDuplex(phy_handle_t *handle,
                                               phy_speed_t   speed,
                                               phy_duplex_t  duplex);

/*!
 * \brief Enables or disables the loopback mode of the PHY.
 *
 * \param[in] handle is the PHY handle for e. g. MDIO access.
 * \param[in] mode   is the loop mode to set.
 * \param[in] speed  is the link speed to set.
 * \param[in] enable determines whether loopback should be enabled or disabled.
 * \return <code>kStatus_Success</code> is the loop mode was adjusted
 * accordingly.
 */
extern status_t PHY_DP83867_EnableLoopback(phy_handle_t *handle,
                                           phy_loop_t mode, phy_speed_t speed,
                                           bool enable);

/*!@}*/

#endif /* _PHY_DP83867_H_ */
