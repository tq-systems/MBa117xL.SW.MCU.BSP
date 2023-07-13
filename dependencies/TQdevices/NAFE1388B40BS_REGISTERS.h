//******************************************************************************
/*!
 * \copyright
 * SPDX-License-Identifier: BSD-3-Clause
 * \copyright
 * Copyright (c) 2021 - 2023 TQ-Systems GmbH <license@tq-group.com>,
 * D-82229 Seefeld, Germany.
 * Author: Isaac L. L. Yuki, Bernhardt Herz
 */
//******************************************************************************

#ifndef _NAFE1388_REGISTERS_H_
#define _NAFE1388_REGISTERS_H_

/*!
 * \addtogroup NAFE1388B40BS
 * @{
 * \file
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*Revision of NAFE13388*/
#ifndef NAFE13388_REVA
#define NAFE13388_REVB /*rev.B*/
#endif
// NAFE13388 Evalkit is rev.B LP
/*----------------------------------------------------------------------------*/

/*******************************************************************************
 * CH_CONFIG0 Register Masks
 ******************************************************************************/

/*Bit Masking HV_AIP_MUX*/
#define NAFE1388B40BS_CH_CONFIG0_HV_AIP_MASK (0xF000U)
#define NAFE1388B40BS_CH_CONFIG0_HV_AIP_SHIFT (12)
/*! HV_AIP
 *  0b0000..Writing 0 to this field results in an HV_AIP connect to GND
 *  0b0001..Write 1 --> HV_AIP connect to AI1P
 *  0b0010..Write 2 --> HV_AIP connect to AI2P
 *  0b0011..Write 3 --> HV_AIP connect to AI3P
 *  0b0100..Write 4 --> HV_AIP connect to AI4P
 *  0b0101..Write 5 --> HV_AIP connect to REFCAL_H
 *  0b0110..Write 6 --> HV_AIP connect to REFCAL_L
 *  0b0111..Write 7 --> HV_AIP connect to AICOM
 *  0b1000..Write 8 --> HV_AIP connect to VEXC
 */
#define NAFE1388B40BS_CH_CONFIG0_HV_AIP(x)                                     \
  (((uint16_t) (((uint16_t) (x)) << NAFE1388B40BS_CH_CONFIG0_HV_AIP_SHIFT))    \
   & NAFE1388B40BS_CH_CONFIG0_HV_AIP_MASK)

/*Bit Masking HV_AIN_MUX*/
#define NAFE1388B40BS_CH_CONFIG0_HV_AIN_MASK (0x0F00U)
#define NAFE1388B40BS_CH_CONFIG0_HV_AIN_SHIFT (8)
/*! HV_AIN
 *  0b0000..Writing 0 to this field results in an HV_AIN connect to GND
 *  0b0001..Write 1 --> HV_AIN connect to AI1N
 *  0b0010..Write 2 --> HV_AIN connect to AI2N
 *  0b0011..Write 3 --> HV_AIN connect to AI3N
 *  0b0100..Write 4 --> HV_AIN connect to AI4N
 *  0b0101..Write 5 --> HV_AIN connect to REFCAL_H
 *  0b0110..Write 6 --> HV_AIN connect to REFCAL_L
 *  0b0111..Write 7 --> HV_AIN connect to AICOM
 *  0b1000..Write 8 --> HV_AIN connect to VEXC
 */
#define NAFE1388B40BS_CH_CONFIG0_HV_AIN(x)                                     \
  (((uint16_t) (((uint16_t) (x)) << NAFE1388B40BS_CH_CONFIG0_HV_AIN_SHIFT))    \
   & NAFE1388B40BS_CH_CONFIG0_HV_AIN_MASK)

/*Bit Masking CH_GAIN*/
#define NAFE1388B40BS_CH_CONFIG0_CH_GAIN_MASK (0x00E0U)
#define NAFE1388B40BS_CH_CONFIG0_CH_GAIN_SHIFT (5)
/*! Channel Gain
 *  0b000..Writing 0 to this field results in an CH_GAIN of 0.2 V/V
 *  0b001..Write 1 --> Channel Gain of 0.4 V/V
 *  0b010..Write 2 --> Channel Gain of 0.8 V/V
 *  0b011..Write 3 --> Channel Gain of 1 V/V
 *  0b100..Write 4 --> Channel Gain of 2 V/V
 *  0b101..Write 5 --> Channel Gain of 4 V/V
 *  0b110..Write 6 --> Channel Gain of 8 V/V
 *  0b111..Write 7 --> Channel Gain of 16 V/V
 */
#define NAFE1388B40BS_CH_CONFIG0_CH_GAIN(x)                                    \
  (((uint16_t) (((uint16_t) (x)) << NAFE1388B40BS_CH_CONFIG0_CH_GAIN_SHIFT))   \
   & NAFE1388B40BS_CH_CONFIG0_CH_GAIN_MASK)

/*Bit Masking HV_Selection_MUX*/
#define NAFE1388B40BS_CH_CONFIG0_HV_SEL_MASK (0x0010U)
#define NAFE1388B40BS_CH_CONFIG0_HV_SEL_SHIFT (4)
/*! HV_Selection_MUX
 *  0b0..Writing 0 to this field results in an HV_SEL connect to LVSIG_IN
 *  0b1..Write 1 --> HV_SEL connect to HV_AIP-HV_AIN
 */
#define NAFE1388B40BS_CH_CONFIG0_HV_SEL(x)                                     \
  (((uint16_t) (((uint16_t) (x)) << NAFE1388B40BS_CH_CONFIG0_HV_SEL_SHIFT))    \
   & NAFE1388B40BS_CH_CONFIG0_HV_SEL_MASK)

/*Bit Masking LVSIG_IN_MUX*/
#define NAFE1388B40BS_CH_CONFIG0_LVSIG_IN_MASK (0x000EU)
#define NAFE1388B40BS_CH_CONFIG0_LVSIG_IN_SHIFT (1)
/*! Channel Gain
 *  0b0000..Writing 0 to this field results in an LVSIG_IN(pos,neg) connect to
 * (REF/2,REF/2) 0b0010..Write 1 --> LVSIG_IN(pos,neg) connect to (GPIO0,GPIO1)
 *  0b0100..Write 2 --> LVSIG_IN(pos,neg) connect to (REF_Coarse,REF/2)
 *  0b0110..Write 3 --> LVSIG_IN(pos,neg) connect to (VADD,REF/2)
 *  0b1000..Write 4 --> LVSIG_IN(pos,neg) connect to (GPIO0,GPIO1)
 *  0b1010..Write 5 --> LVSIG_IN(pos,neg) connect to (REF/2,VHSS)
 */
#define NAFE1388B40BS_CH_CONFIG0_LVSIG_IN(x)                                   \
  (((uint16_t) (((uint16_t) (x)) << NAFE1388B40BS_CH_CONFIG0_LVSIG_IN_SHIFT))  \
   & NAFE1388B40BS_CH_CONFIG0_LVSIG_IN_MASK)

#ifdef NAFE13388_REVB
/*Bit Masking Proprietary Channel Temperature Coefficient Compensation Bypass*/
#define NAFE1388B40BS_CH_CONFIG0_TCC_BYP_MASK (0x0001U)
#define NAFE1388B40BS_CH_CONFIG0_TCC_BYP_SHIFT (0)
/*! TCC_BYP
 *  0b0..Writing 0 to this field results in an TCC_BYP = enable,ON
 *  0b1..Write 1 --> TCC_BYP = disable,OFF
 */
#define NAFE1388B40BS_CH_CONFIG0_TCC_BYP(x)                                    \
  (((uint16_t) (((uint16_t) (x)) << NAFE1388B40BS_CH_CONFIG0_TCC_BYP_SHIFT))   \
   & NAFE1388B40BS_CH_CONFIG0_TCC_BYP_MASK)

#endif

/*******************************************************************************
 * CH_CONFIG1 Register Masks
 ******************************************************************************/
/*Bit Masking CH_CAL_GAIN_OFFSET*/
#define NAFE1388B40BS_CH_CONFIG1_CH_CAL_GAIN_OFFSET_MASK (0xF000U)
#define NAFE1388B40BS_CH_CONFIG1_CH_CAL_GAIN_OFFSET_SHIFT (12)
/*! CH_CAL_GAIN_OFFSET
 *  Selection 1 of 16 calibrated gain and offset coefficient pairs in the
 * Calibrated Channel
 */ //ToDO
#define NAFE1388B40BS_CH_CONFIG1_CH_CAL_GAIN_OFFSET(x)                         \
  (((uint16_t) (((uint16_t) (x))                                               \
                << NAFE1388B40BS_CH_CONFIG1_CH_CAL_GAIN_OFFSET_SHIFT))         \
   & NAFE1388B40BS_CH_CONFIG1_CH_CAL_GAIN_OFFSET_MASK)

/*Bit Masking Channel Over-/Under-range Threshold*/
#define NAFE1388B40BS_CH_CONFIG1_CH_THRS_MASK (0x0F00U)
#define NAFE1388B40BS_CH_CONFIG1_CH_THRS_SHIFT (8)
// ToDO Comments with TABLE_ADC_RANGE_THRS
#define NAFE1388B40BS_CH_CONFIG1_CH_THRS(x)                                    \
  (((uint16_t) (((uint16_t) (x)) << NAFE1388B40BS_CH_CONFIG1_CH_THRS_SHIFT))   \
   & NAFE1388B40BS_CH_CONFIG1_CH_THRS_MASK)

/*Bit Masking ADC_DATA_RATE*/
#define NAFE1388B40BS_CH_CONFIG1_ADC_DATA_RAT_MASK (0x00F8U)
#define NAFE1388B40BS_CH_CONFIG1_ADC_DATA_RAT_SHIFT (2)
// ToDO Comments with TABLE_ADC_RATE
#define NAFE1388B40BS_CH_CONFIG1_CH_THRS_OFFSET(x)                             \
  (((uint16_t) (((uint16_t) (x))                                               \
                << NAFE1388B40BS_CH_CONFIG1_CH_CAL_GAIN_OFFSET_SHIFT))         \
   & NAFE1388B40BS_CH_CONFIG1_CH_CAL_GAIN_OFFSET_MASK)

/*Bit Masking ADC_SINC*/
#define NAFE1388B40BS_CH_CONFIG1_ADC_SINC_MASK (0x0007U)
#define NAFE1388B40BS_CH_CONFIG1_ADC_SINC_STIFT (0)
/*! ADC_SINC
 *  0b000..Writing 0 to this field results in an ADC_SINC = SINC4
 *  0b001..Write 1 --> ADC_SINC = SINC4+1
 *  0b010..Write 2 --> ADC_SINC = SINC4+2
 *  0b011..Write 3 --> ADC_SINC = SINC4+3
 *  0b100..Write 4 --> ADC_SINC = SINC4+4
 *  0b101..Write 5 --> ADC_SINC = SINC4
 *  0b110..Write 6 --> ADC_SINC = SINC4
 *  0b111..Write 5 --> ADC_SINC = SINC4
 */
#define NAFE1388B40BS_CH_CONFIG1_ADC_SINC(x)                                   \
  (((uint16_t) (((uint16_t) (x)) << NAFE1388B40BS_CH_CONFIG1_ADC_SINC_STIFT))  \
   & NAFE1388B40BS_CH_CONFIG1_ADC_SINC_MASK)

/*******************************************************************************
 * CH_CONFIG2 Register Masks
 ******************************************************************************/
#ifdef NAFE13388_REVA
/*Bit Masking Channel Delay*/
#define CH_CONFIG2_CH_DELAY_MASK (0xF000U)
#define CH_CONFIG2_CH_DELAY_SHIFT (12)
/*! ToDO Programmable Delay Table
 *  0b0000..Writing 0 to this field results in an
 *  0b0001..
 */
#define CH_CONFIG2_CH_DELAY(x)                                                 \
  (((uint16_t) (((uint16_t) (x)) << CH_CONFIG2_CH_DELAY_SHIFT))                \
   & CH_CONFIG2_CH_DELAY_MASK)

/*Bit Masking ADC Settling Mode*/
#define CH_CONFIG2_ADC_NORMAL_SETTLING_MASK (0x0800U)
#define CH_CONFIG2_ADC_NORMAL_SETTLING_SHIFT (11)
/*! ADC Settling Mode
 *  0b0..Writing 0 to this field results in an Single-Sycle Settling (SCS)
 *  0b1..Write 1 --> Normal Settling (NS)
 */
#define CH_CONFIG2_ADC_NORMAL_SETTLING(x)                                      \
  (((uint16_t) (((uint16_t) (x)) << CH_CONFIG2_ADC_NORMAL_SETTLING_SHIFT))     \
   & CH_CONFIG2_ADC_NORMAL_SETTLING_MASK)

/*Bit Masking ADC Filter Reset at Start of Conversion*/
#define CH_CONFIG2_ADC_FILTER_RESET_MASK (0x0400U)
#define CH_CONFIG2_ADC_FILTER_RESET_SHIFT (10)
/*! ADC Filter Reset
 *  0b0..Writing 0 to this field results in an "hold of digital filters data
 * from previous conversion" 0b1..Write 1 --> Reset digital filter
 */
#define CH_CONFIG2_ADC_FILTER_RESET(x)                                         \
  (((uint16_t) (((uint16_t) (x)) << CH_CONFIG2_ADC_FILTER_RESET_SHIFT))        \
   & CH_CONFIG2_ADC_FILTER_RESET_MASK)

/*Bit Masking Enable Input Channel Level chopping with 2 ADC Conversions*/
#define CH_CONFIG2_CH_CHOP_MASK (0x0040U)
#define CH_CONFIG2_CH_CHOP_SHIFT (7)
/*! CH_CHOP
 *  0b0..Writing 0 to this field results in Normal mode
 *  0b1..Write 1 --> Precision mode with 2 conversions chopping
 */
#define CH_CONFIG2_CH_CHOP(x)                                                  \
  (((uint16_t) (((uint16_t) (x)) << CH_CONFIG2_CH_CHOP_SHIFT))                 \
   & CH_CONFIG2_CH_CHOP_MASK)

/*Bit Masking Excitation V/I Source Polarity chopping with 2 ADC Conversions*/
#define CH_CONFIG2_VIEX_CHOP_MASK (0x0020U)
#define CH_CONFIG2_VIEX_CHOP_SHIFT (6)
/*! VIEX_CHOP
 *  0b0..Writing 0 to this field results in Normal mode
 *  0b1..Write 1 --> Conversions with polarity chopping
 */
#define CH_CONFIG2_VIEX_CHOP(x)                                                \
  (((uint16_t) (((uint16_t) (x)) << CH_CONFIG2_VIEX_CHOP_SHIFT))               \
   & CH_CONFIG2_VIEX_CHOP_MASK)
#endif

#ifdef NAFE13388_REVB
/*Bit Masking Channel Delay*/
#define NAFE1388B40BS_CH_CONFIG2_CH_DELAY_MASK (0xFC00U)
#define NAFE1388B40BS_CH_CONFIG2_CH_DELAY_SHIFT (10)
/*! ToDO Programmable Delay Table
 *  0b000000..Writing 0 to this field results in an
 *  0b000001..
 */
#define NAFE1388B40BS_CH_CONFIG2_CH_DELAY(x)                                   \
  (((uint16_t) (((uint16_t) (x)) << NAFE1388B40BS_CH_CONFIG2_CH_DELAY_SHIFT))  \
   & NAFE1388B40BS_CH_CONFIG2_CH_DELAY_MASK)

/*Bit Masking ADC Settling Mode*/
#define NAFE1388B40BS_CH_CONFIG2_ADC_NORMAL_SETTLING_MASK (0x0100U)
#define NAFE1388B40BS_CH_CONFIG2_ADC_NORMAL_SETTLING_SHIFT (8)
/*! ADC Settling Mode
 *  0b0..Writing 0 to this field results in an Single-Sycle Settling (SCS)
 *  0b1..Write 1 --> Normal Settling (NS)
 */
#define NAFE1388B40BS_CH_CONFIG2_ADC_NORMAL_SETTLING(x)                        \
  (((uint16_t) (((uint16_t) (x))                                               \
                << NAFE1388B40BS_CH_CONFIG2_ADC_NORMAL_SETTLING_SHIFT))        \
   & NAFE1388B40BS_CH_CONFIG2_ADC_NORMAL_SETTLING_MASK)

/*Bit Masking ADC Filter Reset at Start of Conversion*/
#define NAFE1388B40BS_CH_CONFIG2_ADC_FILTER_RESET_MASK (0x0080U)
#define NAFE1388B40BS_CH_CONFIG2_ADC_FILTER_RESET_SHIFT (7)
/*! ADC Filter Reset
 *  0b0..Writing 0 to this field results in an "hold of digital filters data
 * from previous conversion" 0b1..Write 1 --> Reset digital filter
 */
#define NAFE1388B40BS_CH_CONFIG2_ADC_FILTER_RESET(x)                           \
  (((uint16_t) (((uint16_t) (x))                                               \
                << NAFE1388B40BS_CH_CONFIG2_ADC_FILTER_RESET_SHIFT))           \
   & NAFE1388B40BS_CH_CONFIG2_ADC_FILTER_RESET_MASK)

#endif

/*******************************************************************************
 *CH_CONFIG3 Register Masks
 ******************************************************************************/
/*Bit Masking Excitation V/I Source Selection Voltage/Current*/
#define NAFE1388B40BS_CH_CONFIG3_VIEX_VI_MASK (0x8000U)
#define NAFE1388B40BS_CH_CONFIG3_VIEX_VI_SHIFT (14)
/*! VIEX_VI
 *  0b0..Writing 0 to this field selects Voltage Source
 *  0b1..Write 1 --> Current Source
 */
#define NAFE1388B40BS_CH_CONFIG3_VIEX_VI(x)                                    \
  (((uint16_t) (((uint16_t) (x)) << NAFE1388B40BS_CH_CONFIG3_VIEX_VI_SHIFT))   \
   & NAFE1388B40BS_CH_CONFIG3_VIEX_VI_MASK)

/*Bit Masking Excitation V/I Source Polarity*/
#define NAFE1388B40BS_CH_CONFIG3_VIEX_POL_MASK (0x4000U)
#define NAFE1388B40BS_CH_CONFIG3_VIEX_POL_SHIFT (13)
/*! VIEX_VI
 *  0b0..Writing 0 to this field selects Positive Polarity
 *  0b1..Write 1 --> Negative Polarity
 */
#define NAFE1388B40BS_CH_CONFIG3_VIEX_POL(x)                                   \
  (((uint16_t) (((uint16_t) (x)) << NAFE1388B40BS_CH_CONFIG3_VIEX_POL_SHIFT))  \
   & NAFE1388B40BS_CH_CONFIG3_VIEX_POL_MASK)

/*Bit Masking Excitation V/I Source Magnitude*/
#define NAFE1388B40BS_CH_CONFIG3_VIEX_MAG_MASK (0x3C00U)
#define NAFE1388B40BS_CH_CONFIG3_VIEX_MAG_SHIFT (9)
/*! VIEX_VI ToDO
 *  0b0..Writing 0 to this field
 *  0b1..Write 1 -->
 */
#define NAFE1388B40BS_CH_CONFIG3_VIEX_MAG(x)                                   \
  (((uint16_t) (((uint16_t) (x)) << NAFE1388B40BS_CH_CONFIG3_VIEX_MAG_SHIFT))  \
   & NAFE1388B40BS_CH_CONFIG3_VIEX_MAG_MASK)

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*ToDO NAFE13388 rev.A Mistake at Datasheet Register Map p.119????*/
/*
 * If VIEX_CHOP and CH_CHOP are both set,this particular bit is ignored
 * Refer to CH_CONFIG2 Register!
 */
/*Bit Masking Enable Excitation V/I Source Polarity chopping*/
#define NAFE1388B40BS_CH_CONFIG3_VIEX_CHOP_MASK (0x0040U)
#define NAFE1388B40BS_CH_CONFIG3_VIEX_CHOP_SHIFT (5)
/*! VIEX_VI
 *  0b0..Writing 0 to this field select Normal
 *  0b1..Write 1 --> Select Conversion with polarity chopping
 */
#define NAFE1388B40BS_CH_CONFIG3_VIEX_CHOP(x)                                  \
  (((uint16_t) (((uint16_t) (x)) << NAFE1388B40BS_CH_CONFIG3_VIEX_CHOP_SHIFT)) \
   & NAFE1388B40BS_CH_CONFIG3_VIEX_CHOP_MASK)

/*Bit Masking Excitation V/I Source applied at AIxP Pin*/
#define NAFE1388B40BS_CH_CONFIG3_VIEX_AIP_EN_MASK (0x0038U)
#define NAFE1388B40BS_CH_CONFIG3_VIEX_AIP_EN_SHIFT (3)
/*! VIEX_AIP_EN
 *  0b000..Writing 0 to this field selects input pin none
 *  0b001..Write 1 --> Input Pin AI1P
 *  0b010..Write 2 --> Input Pin AI2P
 *  0b011..Write 3 --> Input Pin AI3P
 *  0b100..Write 4 --> Input Pin AI4P
 */
#define NAFE1388B40BS_CH_CONFIG3_VIEX_AIP_EN(x)                                \
  (((uint16_t) (((uint16_t) (x))                                               \
                << NAFE1388B40BS_CH_CONFIG3_VIEX_AIP_EN_SHIFT))                \
   & NAFE1388B40BS_CH_CONFIG3_VIEX_AIP_EN_MASK)

/*Bit Masking Excitation V/I Source applied at AIxN Pin*/
#define NAFE1388B40BS_CH_CONFIG3_VIEX_AIN_EN_MASK (0x0007U)
#define NAFE1388B40BS_CH_CONFIG3_VIEX_AIN_EN_SHIFT (0)
/*! VIEX_AIN_EN
 *  0b000..Writing 0 to this field selects input pin none
 *  0b001..Write 1 --> Input Pin AI1N
 *  0b010..Write 2 --> Input Pin AI2N
 *  0b011..Write 3 --> Input Pin AI3N
 *  0b100..Write 4 --> Input Pin AI4N
 */
#define NAFE1388B40BS_CH_CONFIG3_VIEX_AIN_EN(x)                                \
  (((uint16_t) (((uint16_t) (x))                                               \
                << NAFE1388B40BS_CH_CONFIG3_VIEX_AIN_EN_SHIFT))                \
   & NAFE1388B40BS_CH_CONFIG3_VIEX_AIN_EN_MASK)

#ifdef NAFE13388_REVB
/*Bit Masking Excitation V/I Source Enable VEXC Voltage*/
#define NAFE1388B40BS_CH_CONFIG3_VIEX_EN_MASK (0x0200U)
#define NAFE1388B40BS_CH_CONFIG3_VIEX_EN_SHIFT (9)
/*! VIEX_EN
 *  0b0..Writing 0 to this field disables VEXC voltage output
 *  0b1..Write 1 --> Enable VEXC voltage output
 */
#define NAFE1388B40BS_CH_CONFIG3_VIEX_AIN_EN(x)                                \
  (((uint16_t) (((uint16_t) (x))                                               \
                << NAFE1388B40BS_CH_CONFIG3_VIEX_AIN_EN_SHIFT))                \
   & NAFE1388B40BS_CH_CONFIG3_VIEX_AIN_EN_MASK)
#endif

/*******************************************************************************
 *CH_CONFIG4 Register Masks
 ******************************************************************************/
/*Bit Masking for Logic Channel x for ADC conversion in MC Mode*/
#define NAFE1388B40BS_CH_CONFIG4_MCH_EN_MASK (0xFFFFU)
#define NAFE1388B40BS_CH_CONFIG4_MCH_EN_SHIFT (0)
/*! VIEX_AIN_EN
 *  Bit settings 0=disable, 1=enable --> CH15 is bit15, CH0 is bit0
 *  Example:
 *  0b1000000000000000 -> CH15 is enable for MC Mode
 *  0b0000000000000010 -> CH1 is enabled for MC Mode
 */
#define NAFE1388B40BS_CH_CONFIG4_MCH_EN(x)                                     \
  (((uint16_t) (((uint16_t) (x)) << NAFE1388B40BS_CH_CONFIG4_MCH_EN_SHIFT))    \
   & NAFE1388B40BS_CH_CONFIG4_MCH_EN_MASK)

/*******************************************************************************
 *SYS_CONFIG0 Register Masks
 ******************************************************************************/
#ifdef NAFE13388_REVA
/*Bit Masking for Current Pointer of 16 Data Channels*/
#define NAFE1388B40BS_SYS_CONFIG0_CURR_POINTER_MASK (0xF000U)
#define NAFE1388B40BS_SYS_CONFIG0_CURR_POINTER_SHIFT (12)
/*! CURRENT_POINTER
 *  After reset the Pointer is at Channel0
 *  0b0000..Writing 0 to this field selects Channel0
 *  0b0001..Write 1 --> Channel1
 *  0b0010..Write 2 --> Channel2
 *  0b0011..Write 3 --> Channel3
 *  0b0100..Write 4 --> Channel4
 *  ...........
 */
#define NAFE1388B40BS_SYS_CONFIG0_CURR_POINTER(x)                              \
  (((uint16_t) (((uint16_t) (x))                                               \
                << NAFE1388B40BS_SYS_CONFIG0_CURR_POINTER_SHIFT))              \
   & NAFE1388B40BS_SYS_CONFIG0_CURR_POINTER_MASK)

/*Bit Masking DRDYB pin behavior*/
#define SYS_CONFIG0_DRDYB_PIN_SEQ_MASK (0x0010U)
#define SYS_CONFIG0_DRDYB_PIN_SEQ_SHIFT (4)
/*! DRDYB_PIN_SEQ (Especially for sequencer Mode)
 *  0b0..Write 0 --> Produce falling on every channel conversion done
 *  0b1..Write 1 --> Produce falling edge only when the sequ. done with last
 * enabled channel conversion
 */
#define NAFE1388B40BS_SYS_CONFIG0_DRDYB_PIN_SEQ(x)                             \
  (((uint16_t) (((uint16_t) (x))                                               \
                << NAFE1388B40BS_SYS_CONFIG0_DRDYB_PIN_SEQ_SHIFT))             \
   & NAFE1388B40BS_SYS_CONFIG0_DRDYB_PIN_SEQ_MASK)
#endif

/*Bit Masking Int./External Voltage Reference*/
#define NAFE1388B40BS_SYS_CONFIG0_REF_SEL_MASK (0x0C00U)
#define NAFE1388B40BS_SYS_CONFIG0_REF_SEL_SHIFT (10)
/*! REF_SEL
 *  After reset the Pointer is at Channel0
 *  0b00..Writing 0 to this field select both REF_BYP and REF_ADC (Internal 2.5V
 * REF_INT) 0b01..Write 1 --> REF_BYP = 2.5V(REF_INT) / REF_ADC = 2.5V(REF_EXT)
 *  0b10..Write 2 --> REF_BYP = 2.5V(REF_EXT) / REF_ADC = 2.5V(REF_INT)
 *  0b11..Write 3 --> REF_BYP = 2.5V(REF_EXT) / REF_ADC = 2.5V(REF_EXT)
 */
#define NAFE1388B40BS_SYS_CONFIG0_REF_SEL(x)                                   \
  (((uint16_t) (((uint16_t) (x)) << NAFE1388B40BS_SYS_CONFIG0_REF_SEL_SHIFT))  \
   & NAFE1388B40BS_SYS_CONFIG0_REF_SEL_MASK)

/*Bit Masking Clock source Selection*/
#define NAFE1388B40BS_SYS_CONFIG0_CK_SRC_SEL_MASK (0x0300U)
#define NAFE1388B40BS_SYS_CONFIG0_CK_SRC_SEL_SHIFT (8)
/*! CK_SRC_SEL
 *  0b00..Write 0 --> Internal Clock
 *  0b01..Write 1 --> Internal Clock, disable crystal oscillator circuit
 *  0b10..Write 2 --> Ext. 18.432MHz clock at XI pin, disable crystal oscillator
 * circuit 0b11..Write 3 --> 18.432MHz crystal is installed at XI/XO pins
 */
#define NAFE1388B40BS_SYS_CONFIG0_CK_SRC_SEL(x)                                \
  (((uint16_t) (((uint16_t) (x))                                               \
                << NAFE1388B40BS_SYS_CONFIG0_CK_SRC_SEL_SHIFT))                \
   & NAFE1388B40BS_SYS_CONFIG0_CK_SRC_SEL_MASK)

/*Bit Masking CRC Activation*/
#define NAFE1388B40BS_SYS_CONFIG0_CRC_EN_MASK (0x0080U)
#define NAFE1388B40BS_SYS_CONFIG0_CRC_EN_SHIFT (7)
/*! CRC_EN
 *  0b0..Write 0 --> CRC disabled
 *  0b1..Write 1 --> CRC enabled
 */
#define NAFE1388B40BS_SYS_CONFIG0_CRC_EN(x)                                    \
  (((uint16_t) (((uint16_t) (x)) << NAFE1388B40BS_SYS_CONFIG0_CRC_EN_SHIFT))   \
   & NAFE1388B40BS_SYS_CONFIG0_CRC_EN_MASK)

/*Bit Masking Status to ADC data*/
#define NAFE1388B40BS_SYS_CONFIG0_STATUS_EN_MASK (0x0040U)
#define NAFE1388B40BS_SYS_CONFIG0_STATUS_EN_SHIFT (6)
/*! STATUS_EN
 *  Prepend 8-bit status bits to ADC data of enabled channels, MCH_EN[i] = 1
 *  In MC-Mode-Reading 8 bits status is OR'd of enabled channels when in
 *  data ouput burst
 *  status_bits(MSB to LSB):
 *  over-load, under-load, over-range, under-range, over-temp, global_alarm,
 *  over-voltage, CRC error
 */
#define NAFE1388B40BS_SYS_CONFIG0_STATUS_EN(x)                                 \
  (((uint16_t) (((uint16_t) (x))                                               \
                << NAFE1388B40BS_SYS_CONFIG0_STATUS_EN_SHIFT))                 \
   & NAFE1388B40BS_SYS_CONFIG0_STATUS_EN_MASK)

/*Bit Masking ADC synchronization mode with SYNC pulse*/
#define NAFE1388B40BS_SYS_CONFIG0_ADC_SYNC_MASK (0x0020U)
#define NAFE1388B40BS_SYS_CONFIG0_ADC_SYNC_SHIFT (5)
/*! ADC_SYNC
 *
 * NAFE REV.A:
 *  !!Only with Single conversion mode!!
 *  --> in MC-Mode NAFE1388B40BS_SYNC pulse is ignored
 *  0b0..Write 0 --> Internal Timing
 *  0b1..Write 1 --> ADC synchronizated via SYNC pulse at rising edge in SC-Mode
 *
 *  NAFE REV.B:
 *  0b0..Write 0 --> ADC Conversation triggerd by SPI CMD
 *  0b1..Write 1 --> ADC Conversation triggerd by SYNC impulse and SPI_CMD only
 * decides which Conversion Mode is executed
 */
#define NAFE1388B40BS_SYS_CONFIG0_ADC_SYNC(x)                                  \
  (((uint16_t) (((uint16_t) (x)) << NAFE1388B40BS_SYS_CONFIG0_ADC_SYNC_SHIFT)) \
   & NAFE1388B40BS_SYS_CONFIG0_ADC_SYNC_MASK)

/*Bit Masking Global alarm Interrupt*/
#define NAFE1388B40BS_SYS_CONFIG0_GLOBAL_ALRM_STICKY_MASK (0x0008U)
#define NAFE1388B40BS_SYS_CONFIG0_GLOBAL_ALRM_STICKY_SHIFT (3)
/*! GLOBAL_ALRM_STICKY
 *  0b0..Write 0 --> Non-sticky, read live status
 *  0b1..Write 1 --> Sticky, write 1 to clear a secific bit
 */
#define NAFE1388B40BS_SYS_CONFIG0_GLOBAL_ALRM_STICKY(x)                        \
  (((uint16_t) (((uint16_t) (x))                                               \
                << NAFE1388B40BS_SYS_CONFIG0_GLOBAL_ALRM_STICKY_SHIFT))        \
   & NAFE1388B40BS_SYS_CONFIG0_GLOBAL_ALRM_STICKY_MASK)

/*Bit Masking SPI_DOUT_DRIVE*/
#define NAFE1388B40BS_SYS_CONFIG0_SPI_DOUT_DRIVE_MASK (0x0004U)
#define NAFE1388B40BS_SYS_CONFIG0_SPI_DOUT_DRIVE_SHIFT (2)
/*! SPI_DOUT_DRIVE
 *  Increase DOUT output drive if high capacitance loading
 */
#define NAFE1388B40BS_SYS_CONFIG0_SPI_DOUT_DRIVE(x)                            \
  (((uint16_t) (((uint16_t) (x))                                               \
                << NAFE1388B40BS_SYS_CONFIG0_SPI_DOUT_DRIVE_SHIFT))            \
   & NAFE1388B40BS_SYS_CONFIG0_SPI_DOUT_DRIVE_MASK)

/*Bit Masking INTB_DRIVER_TYPE*/
#define NAFE1388B40BS_SYS_CONFIG0_INTB_DRIVER_TYPE_MASK (0x0002U)
#define NAFE1388B40BS_SYS_CONFIG0_INTB_DRIVER_TYPE_SHIFT (1)
/*! INTB_DRIVER_TYPE
 *  INTB pin driver type:
 *  0b0..Write 0 --> 100kOhm pull up with open drain
 *  0b1..Write 1 --> CMOS push pull
 */
#define NAFE1388B40BS_SYS_CONFIG0_INTB_DRIVER_TYPE(x)                          \
  (((uint16_t) (((uint16_t) (x))                                               \
                << NAFE1388B40BS_SYS_CONFIG0_INTB_DRIVER_TYPE_SHIFT))          \
   & NAFE1388B40BS_SYS_CONFIG0_INTB_DRIVER_TYPE_MASK)

/*Bit Masking Rout of CRC_ERROR*/
#define NAFE1388B40BS_SYS_CONFIG0_CRC_ERROR_ON_GPIO2_MASK (0x0001U)
#define NAFE1388B40BS_SYS_CONFIG0_CRC_ERROR_ON_GPIO2_SHIFT (0)
/*! CRC_ERROR_ON_GPIO2
 *  Routing of CRC_ERROR interrupt to GPIO2
 *  0b0..Write 0 --> Normal GPIO function
 *  0b1..Write 1 --> Output CRC_ERROR to GPIO2 pin, active high
 */
#define NAFE1388B40BS_SYS_CONFIG0_CRC_ERROR_ON_GPIO2(x)                        \
  (((uint16_t) (((uint16_t) (x))                                               \
                << NAFE1388B40BS_SYS_CONFIG0_CRC_ERROR_ON_GPIO2_SHIFT))        \
   & NAFE1388B40BS_SYS_CONFIG0_CRC_ERROR_ON_GPIO2_MASK)

#ifdef NAFE13388_REVB
/*Bit Masking DRDY pulse with duration (# of SYSCLCK cycle)*/
#define NAFE1388B40BS_SYS_CONFIG0_DRDY_PWDT_MASK (0x8000U)
#define NAFE1388B40BS_SYS_CONFIG0_DRDY_PWDT_SHIFT (15)
/*! DRDY_PWDT
 *  0b0..Write 0 --> Value 2
 *  0b1..Write 1 --> Value 8
 */
#define NAFE1388B40BS_SYS_CONFIG0_DRDY_PWDT(x)                                 \
  (((uint16_t) (((uint16_t) (x))                                               \
                << NAFE1388B40BS_SYS_CONFIG0_DRDY_PWDT_SHIFT))                 \
   & NAFE1388B40BS_SYS_CONFIG0_DRDY_PWDT_MASK)

/*Bit Masking ADC Data register wide*/
#define NAFE1388B40BS_SYS_CONFIG0_ADC_DATA_OUT_16BIT_MASK (0x4000U)
#define NAFE1388B40BS_SYS_CONFIG0_ADC_DATA_OUT_16BIT_SHIFT (14)
/*! ADC_DATA_OUT_16BIT
 *  0b0..Write 0 --> ADC Data register 24bit
 *  0b1..Write 1 --> ADC Data register 16bit
 */
#define NAFE1388B40BS_SYS_CONFIG0_ADC_DATA_OUT_16BIT(x)                        \
  (((uint16_t) (((uint16_t) (x))                                               \
                << NAFE1388B40BS_SYS_CONFIG0_ADC_DATA_OUT_16BIT_SHIFT))        \
   & NAFE1388B40BS_SYS_CONFIG0_ADC_DATA_OUT_16BIT_MASK)

/*Bit Masking DRDYB pin edge behavior*/
#define NAFE1388B40BS_SYS_CONFIG0_DRDYB_PIN_EDGE_MASK (0x0010U)
#define NAFE1388B40BS_SYS_CONFIG0_DRDYB_PIN_EDGE_SHIFT (4)
/*! DRDYB_PIN_EDGE
 *  DRDY pin set for CMD_MS, CMD_MM, CMD_MC reading modes:
 *  0b0..Write 0 --> Produce rising edge on every channel conversion done
 *  0b1..Write 1 --> Produce rising edge only when the last enabled channel
 * conversion is done
 */
#define NAFE1388B40BS_SYS_CONFIG0_DRDYB_PIN_EDGE(x)                            \
  (((uint16_t) (((uint16_t) (x))                                               \
                << NAFE1388B40BS_SYS_CONFIG0_DRDYB_PIN_EDGE_SHIFT))            \
   & NAFE1388B40BS_SYS_CONFIG0_DRDYB_PIN_EDGE_MASK)
#endif

/*******************************************************************************
 *SYS_STATUS0 Register Masks
 ******************************************************************************/
#ifdef NAFE13388_REVA
#endif

/*******************************************************************************
 *GLOBAL_ALRM_ENABLE Register Masks
 ******************************************************************************/
/*
 * 0b0..Write 0 --> ALRM is disabled
 * 0b1..Write 1 --> ALRm is enabled
 */
#define NAFE1388B40BS_OVER_TEMP_ALRM(x)                                        \
  (((uint16_t) (((uint16_t) (x)) << 15)) & 0x8000) // Warning >145°C
#define NAFE1388B40BS_HVDD_ALRM(x)                                             \
  (((uint16_t) (((uint16_t) (x)) << 14))                                       \
   & 0x4000) // HVDD supply detect below preset threshold
#define NAFE1388B40BS_HVSS_ALRM(x)                                             \
  (((uint16_t) (((uint16_t) (x)) << 13))                                       \
   & 0x2000) // HVSS supply detect below preset threshold
#define NAFE1388B40BS_DVDD_ALRM(x)                                             \
  (((uint16_t) (((uint16_t) (x)) << 12))                                       \
   & 0x1000) // DVDD supply detect below preset threshold
/*bit 11 is reserved*/
#define NAFE1388B40BS_GPI_POS_ALRM(x)                                          \
  (((uint16_t) (((uint16_t) (x)) << 10))                                       \
   & 0x0400) // Rising edge detected at an of GPI pins
#define NAFE1388B40BS_GPI_NEG_ALRM(x)                                          \
  (((uint16_t) (((uint16_t) (x)) << 9))                                        \
   & 0x0200) // Falling edge detected at an of GPI pins
#define NAFE1388B40BS_CONFIG_ERROR_ALRM(x)                                     \
  (((uint16_t) (((uint16_t) (x)) << 8))                                        \
   & 0x0100) // Register configuration error
#define NAFE1388B40BS_OVRRING_ALRM(x)                                          \
  (((uint16_t) (((uint16_t) (x)) << 7))                                        \
   & 0x0080) // On or more data channel in over-range
#define NAFE1388B40BS_UNDRING_ALRM(x)                                          \
  (((uint16_t) (((uint16_t) (x)) << 6))                                        \
   & 0x0040) // On or more data channel in under-range
#define NAFE1388B40BS_OVRLOAD_ALRM(x)                                          \
  (((uint16_t) (((uint16_t) (x)) << 5))                                        \
   & 0x0020) // On or more data channel is over-loaded or under-loaded
#define NAFE1388B40BS_EXTCLK_FRQ_ALRM(x)                                       \
  (((uint16_t) (((uint16_t) (x)) << 4))                                        \
   & 0x0010) // XTAL or EXTCLK freq. varies with internal CLK
#define NAFE1388B40BS_PGA_OV_ALRM(x)                                           \
  (((uint16_t) (((uint16_t) (x)) << 3))                                        \
   & 0x0008) // On or more data channel is over-voltage stressing PGA

#define NAFE1388B40BS_TEMP_ALRM(x)                                             \
  (((uint16_t) (((uint16_t) (x)) << 0))                                        \
   & 0x0001) // Programmable Temp. alarm (Threshold set at THRES_TEMP register
             // bits)

#ifdef NAFE13388_REVA
#define VIEX_OV_ALRM(x)                                                        \
  (((uint16_t) (((uint16_t) (x)) << 2))                                        \
   & 0x0004) // Excitation voltage source is over loaded
#define VIEX_OI_ALRM(x)                                                        \
  (((uint16_t) (((uint16_t) (x)) << 1))                                        \
   & 0x0002) // Excitation current source is over loaded
#endif

#ifdef NAFE13388_REVB
#define VIEX_OVLD_ALRM(x)                                                      \
  (((uint16_t) (((uint16_t) (x)) << 2))                                        \
   & 0x0004) // Excitation source is over loaded
/*bit 1 is reserved*/
#endif

/*******************************************************************************
 *GLOBAL_ALRM_INT Register Masks
 ******************************************************************************/
/*
 * Bit clear behavior controlled by GLOBAL_ALRM_STICKY bit at SYS_CONFIG0 Reg.
 * Default values are: 0x0000
 */
#define NAFE1388B40BS_OVER_TEMP_INT(x)                                         \
  (((uint16_t) (((uint16_t) (x)) << 15)) & 0x8000)
#define NAFE1388B40BS_HVDD_INT(x)                                              \
  (((uint16_t) (((uint16_t) (x)) << 14)) & 0x4000)
#define NAFE1388B40BS_HVSS_INT(x)                                              \
  (((uint16_t) (((uint16_t) (x)) << 13)) & 0x2000)
#define NAFE1388B40BS_DVDD_INT(x)                                              \
  (((uint16_t) (((uint16_t) (x)) << 12)) & 0x1000)
/*bit 11 is reserved*/
#define NAFE1388B40BS_GPI_POS_INT(x)                                           \
  (((uint16_t) (((uint16_t) (x)) << 10)) & 0x0400)
#define NAFE1388B40BS_GPI_NEG_INT(x)                                           \
  (((uint16_t) (((uint16_t) (x)) << 9)) & 0x0200)
#define NAFE1388B40BS_CONFIG_ERROR_INT(x)                                      \
  (((uint16_t) (((uint16_t) (x)) << 8)) & 0x0100)
#define NAFE1388B40BS_OVRRING_INT(x)                                           \
  (((uint16_t) (((uint16_t) (x)) << 7)) & 0x0080)
#define NAFE1388B40BS_UNDRING_INT(x)                                           \
  (((uint16_t) (((uint16_t) (x)) << 6)) & 0x0040)
#define NAFE1388B40BS_OVRLOAD_INT(x)                                           \
  (((uint16_t) (((uint16_t) (x)) << 5)) & 0x0020)
#define NAFE1388B40BS_EXTCLK_FRQ_INT(x)                                        \
  (((uint16_t) (((uint16_t) (x)) << 4)) & 0x0010)
#define NAFE1388B40BS_PGA_OV_INT(x)                                            \
  (((uint16_t) (((uint16_t) (x)) << 3)) & 0x0008)

#define NAFE1388B40BS_TEMP_INT(x)                                              \
  (((uint16_t) (((uint16_t) (x)) << 0)) & 0x0001)

#ifdef NAFE13388_REVA
#define NAFE1388B40BS_VIEX_OV_INT(x)                                           \
  (((uint16_t) (((uint16_t) (x)) << 2)) & 0x0004)
#define NAFE1388B40BS_VIEX_OI_INT(x)                                           \
  (((uint16_t) (((uint16_t) (x)) << 1)) & 0x0002)
#endif

#ifdef NAFE13388_REVB
#define NAFE1388B40BS_VIEX_OVLD_INT(x)                                         \
  (((uint16_t) (((uint16_t) (x)) << 2)) & 0x0004)
/*bit 1 is reserved*/
#endif

/*******************************************************************************
 *Register Mapping
 ******************************************************************************/

/*Instruction commands for NAFE13388*/

/*Commands with non-return ADC Data at SPI Transfer*/

/*Selection of logical channel*/
#define NAFE1388B40BS_CMD_CH0 0x0000  /*CMD_Pointer set to CH0*/
#define NAFE1388B40BS_CMD_CH1 0x0001  /*CMD_Pointer set to CH1*/
#define NAFE1388B40BS_CMD_CH2 0x0002  /*CMD_Pointer set to CH2*/
#define NAFE1388B40BS_CMD_CH3 0x0003  /*CMD_Pointer set to CH3*/
#define NAFE1388B40BS_CMD_CH4 0x0004  /*CMD_Pointer set to CH4*/
#define NAFE1388B40BS_CMD_CH5 0x0005  /*CMD_Pointer set to CH5*/
#define NAFE1388B40BS_CMD_CH6 0x0006  /*CMD_Pointer set to CH6*/
#define NAFE1388B40BS_CMD_CH7 0x0007  /*CMD_Pointer set to CH7*/
#define NAFE1388B40BS_CMD_CH8 0x0008  /*CMD_Pointer set to CH8*/
#define NAFE1388B40BS_CMD_CH9 0x0009  /*CMD_Pointer set to CH9*/
#define NAFE1388B40BS_CMD_CH10 0x000A /*CMD_Pointer set to CH10*/
#define NAFE1388B40BS_CMD_CH11 0x000B /*CMD_Pointer set to CH11*/
#define NAFE1388B40BS_CMD_CH12 0x000C /*CMD_Pointer set to CH12*/
#define NAFE1388B40BS_CMD_CH13 0x000D /*CMD_Pointer set to CH13*/
#define NAFE1388B40BS_CMD_CH14 0x000E /*CMD_Pointer set to CH14*/
#define NAFE1388B40BS_CMD_CH15 0x000F /*CMD_Pointer set to CH15*/

/*Offset for Register RW Operations*/
#define NAFE1388B40BS_WRITE_REG_OFFSET 0x0000
#define NAFE1388B40BS_READ_REG_OFFSET 0x4000 // 0x2000

/**
 * @def PARTNAME_MSB_REGISTER
 */
#define NAFE1388B40BS_PARTNAME_MSB 0x007C /*Read only Register*/

/**
 * @def PARTNAME_LSB_REGISTER
 */
#define NAFE1388B40BS_PARTNAME_LSB 0x007D /*Read only Register*/

/**
 * @def PARTNAME_MSB_REGISTER
 */
#define NAFE1388B40BS_SERIAL_MSB 0x00AE /*Read only Register*/

/**
 * @def PARTNAME_LSB_REGISTER
 */
#define NAFE1388B40BS_SERIAL_LSB 0x00AF /*Read only Register*/

/*
 * CH_CONFIG0,1,2,3 are logical channel configuration channel registers. Send
 * SPI CMD_CHi (i=0..15)! CONFIG_CH_PTR[3:0] shows selected channel.
 */
#define NAFE1388B40BS_CH_CONFIG0 0x0020 /*Config HV/LV-MUX + PGA*/
#define NAFE1388B40BS_CH_CONFIG1                                               \
  0x0021                                /*Config ADC + SINC +                  \
                                           CAL_GAIN_OFFSET*/
#define NAFE1388B40BS_CH_CONFIG2 0x0022 /*Config ADC + Delay*/
#define NAFE1388B40BS_CH_CONFIG3 0x0023 /*Config Excitation V/I Source*/
#define NAFE1388B40BS_CH_CONFIG4 0x0024 /*Enable CH Config ADC Multi Channel*/
#define NAFE1388B40BS_CH_CONFIG5 0x005
#define NAFE1388B40BS_CH_CONFIG6 0x006

/*
 * <SYS Control/Status Registers>
 */
#define NAFE1388B40BS_SYS_CONFIG0 0x0030 /*System configuration Register*/
#define NAFE1388B40BS_SYS_STATUS0 0x0031 /*System status Register*/

/*
__MONITORING_REG
 */
#define NAFE1388B40BS_GLOBAL_ALARM_ENABLE 0x0032    /*ALARM Enable Register*/
#define NAFE1388B40BS_GLOBAL_ALARM_INTERRUPT 0x0033 /*ALARM INT Register*/
#define NAFE1388B40BS_DIE_TEMP 0x34  /*16bit Temp read only 2's complement*/
#define NAFE1388B40BS_THRS_TEMP 0x37 /*Threshold Temperature Register*/

/*
 * CH_DATA_REG
 */
#define NAFE1388B40BS_CH_DATA0 0x40
#define NAFE1388B40BS_CH_DATA1 0x41
#define NAFE1388B40BS_CH_DATA2 0x42
#define NAFE1388B40BS_CH_DATA3 0x43
#define NAFE1388B40BS_CH_DATA4 0x44
#define NAFE1388B40BS_CH_DATA5 0x45
#define NAFE1388B40BS_CH_DATA6 0x46
#define NAFE1388B40BS_CH_DATA7 0x47
#define NAFE1388B40BS_CH_DATA8 0x48
#define NAFE1388B40BS_CH_DATA9 0x49
#define NAFE1388B40BS_CH_DATAA 0x4A
#define NAFE1388B40BS_CH_DATAB 0x4B
#define NAFE1388B40BS_CH_DATAC 0x4C
#define NAFE1388B40BS_CH_DATAD 0x4D
#define NAFE1388B40BS_CH_DATAE 0x4E
#define NAFE1388B40BS_CH_DATAF 0x4F

/*
 * CH_STATUS_REG
 */
#define NAFE1388B40BS_CH_STATUS0 0x35
#define NAFE1388B40BS_CH_STATUS1 0x36

/*
 * CH_COEF_REG
 */
#define NAFE1388B40BS_CH_GAIN_COEF_0 0x8

#define NAFE1388B40BS_CH_OFFSET_COEF_0 0x9

/*******************************************************************************
 *Masks
 ******************************************************************************/

#define NAFE1388B40BS_SYS_STATUS0_SINGLE_CH_ACTIVE_MASK 0x8000
#define NAFE1388B40BS_SYS_STATUS0_CHIP_READY_MASK 0x2000
#define NAFE1388B40BS_RST_GPIO GPIO9
#define NAFE1388B40BS_RST_PIN 11U

#define NAFE1388B40BS_ADC_DATA_RDY_GPIO GPIO9
#define NAFE1388B40BS_ADC_DATA_RDY_PIN 13U

/*!@}*/

#endif /* _NAFE1388_REGISTERS_H_ */
