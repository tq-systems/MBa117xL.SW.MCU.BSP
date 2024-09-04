demo_PMIC {#demo_PMIC}
=========

## Overview

The `demo_PMIC` code provides a demonstration of interfacing and manipulating the Power Management Integrated Circuit (PMIC)
on TQMa117xL using I2C or LPI2C based on the configuration. The code helps in reading the Device ID of the PMIC and
allows the user to set different core voltages.

## Supported Toolchain

See [Requirements](../../README.md#requirements).

## Hardware Requirements

- Mini/micro USB cable
- SD Card
- Personal Computer
- Compatible board (as per the targeted board in your project)

## Board Settings

No special settings are required for this demo.

## Demo Preparation

1. Connect a USB cable between the PC host and the target board.
2. Insert an SD Card into the card slot of the target board.
3. Open a serial terminal on PC with the following settings:
   - 115200 baud rate
   - 8 data bits
   - No parity
   - One stop bit
   - No flow control
4. Download the program to the target board.
5. To begin running the demo, either press the reset button on your board or launch the debugger in your IDE.

## Running the Demo

Upon successful execution, the following message will be displayed in the terminal:

``` txt

  TQ-Systems GmbH
  This is a demo application for the device <module-board>.
  Welcome to the demonstration.

  Searching for device...
  [ OK ]	PMIC PF5020 QM
  Select one of three power states. 1/2/3
  <input> 1
   	 - New VDD_SOC:    1.000 V
  Select one of three power states. 1/2/3
  <input> 2
   	 - New VDD_SOC:    1.100 V
  Select one of three power states. 1/2/3
  <input> 3
   	 - New VDD_SOC:    0.900 V
  Select one of three power states. 1/2/3
 ...

```
