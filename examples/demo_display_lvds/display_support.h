/*
 * Copyright 2019-2021, 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _DISPLAY_SUPPORT_H_
#define _DISPLAY_SUPPORT_H_

#include "fsl_dc_fb.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* @TEST_ANCHOR */

#define DEMO_DISPLAY_CONTROLLER_ELCDIF 0
#define DEMO_DISPLAY_CONTROLLER_LCDIFV2 1

#ifndef DEMO_DISPLAY_CONTROLLER
/* Use LCDIFV2 by default, could use ELCDIF by changing this macro. */
#define DEMO_DISPLAY_CONTROLLER DEMO_DISPLAY_CONTROLLER_LCDIFV2
#endif

#define DEMO_BUFFER_FIXED_ADDRESS 0

#if DEMO_BUFFER_FIXED_ADDRESS
#define DEMO_BUFFER0_ADDR 0x80000000
#define DEMO_BUFFER1_ADDR 0x80200000
#endif

/*
 * Use the MIPI dumb panel
 */

/* Definitions for the frame buffer. */
#define DEMO_BUFFER_COUNT 3 /* 2 is enough for DPI interface display. */

#ifndef DEMO_USE_XRGB8888
#define DEMO_USE_XRGB8888 0
#endif

/* Use LCDIF LUT (or named color palette) which is 8-bit per-pixel */
#ifndef DEMO_USE_LUT8
#define DEMO_USE_LUT8 0
#endif

#if DEMO_USE_XRGB8888
#define DEMO_BUFFER_PIXEL_FORMAT kVIDEO_PixelFormatXRGB8888
#define DEMO_BUFFER_BYTE_PER_PIXEL 4
#elif DEMO_USE_LUT8
#define DEMO_BUFFER_PIXEL_FORMAT kVIDEO_PixelFormatLUT8
#define DEMO_BUFFER_BYTE_PER_PIXEL 1
#else
#define DEMO_BUFFER_PIXEL_FORMAT kVIDEO_PixelFormatRGB888
#define DEMO_BUFFER_BYTE_PER_PIXEL 3
#endif

#define DEMO_PANEL_WIDTH (1280)
#define DEMO_PANEL_HEIGHT (800)

#define DEMO_BUFFER_WIDTH DEMO_PANEL_WIDTH
#define DEMO_BUFFER_HEIGHT DEMO_PANEL_HEIGHT

/* Where the frame buffer is shown in the screen. */
#define DEMO_BUFFER_START_X 0U
#define DEMO_BUFFER_START_Y 0U

#define DEMO_BUFFER_STRIDE_BYTE (DEMO_BUFFER_WIDTH * DEMO_BUFFER_BYTE_PER_PIXEL)
/* There is not frame buffer aligned requirement, consider the 64-bit AXI data
 * bus width and 32-byte cache line size, the frame buffer alignment is set to
 * 32 byte.
 */
#define FRAME_BUFFER_ALIGN 32

extern const dc_fb_t g_dc;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C"
{
#endif /* __cplusplus */

  status_t BOARD_ReadBridgeStatus(void);

  status_t BOARD_PrepareDisplayController(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _DISPLAY_SUPPORT_H_ */
