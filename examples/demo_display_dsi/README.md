# demo_display_dsi {#demo_display_dsi}

## Overview

This example demonstrates the use of the fbdev component. Fbdev abstracts the differences between LCD controllers and
provides unified APIs for upper layers. It also manages frame buffers. When running, this example displays a rectangle
moving across the screen, changing color upon reaching the screen border.

## Supported Toolchain

See [Requirements](../../README.md#requirements).

## Hardware Requirements

- Mini/micro USB cable
- Compatible board (as per the targeted board in your project)
- Personal Computer
- Compatible panel (RK055MHD091, RK055AHD091, or RK055IQH091)

Please refer to the READMEs of the specific board you are using for any additional instructions or requirements.

## Board Settings

- Connect the panel to X40.

## Software Settings

Please mind to use the configuration for SD-Ram-Flash for this application, otherwise it won't compile.

## Demo Preparation

1. Connect a USB cable between the PC host and the target board.
2. Open a serial terminal on the PC with the following settings:
   - 115200 baud rate
   - 8 data bits
   - No parity
   - One stop bit
   - No flow control
3. Build the project, which defaults to RK055MHD091. To use other panels, modify `#define DEMO_PANEL` in `display_support.h`.
4. Download the program to the target board.
5. To start the demo, press the reset button on your board or launch the debugger in your IDE.

## Running the Demo

Upon successful execution, the screen will display a moving rectangle as described in the overview.
