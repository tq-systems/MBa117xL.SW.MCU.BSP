demo_eeprom {#demo_eeprom}
===========

## Overview

This example, `eeprom`, demonstrates the usage of the eeprom M23C64(DF). 

## Supported Toolchain

See [Requirements](../../README.md#requirements).

## Demo Preparation

1. Connect the appropriate power supply and J-Link Debug Probe to the board.
2. Connect a USB cable between the host PC and the J-Link-debugger.
3. Connect a USB-Cable between the host PC and the Mainboard.
4. Open one serial terminal with the settings specified for the board.
5. Download the program to the target board.
6. Launch the debugger in your IDE to begin running the demo.
7. Put the terminal in "CR+LF" mode.

## Running the Demo

Upon successful execution, the following message will be displayed in the terminal:

``` txt
TQ-Systems GmbH
This is a demo application for the device <module-board>.
Welcome to the demonstration.

This demo shows the operation of the eeprom M24C64[DF]. Press anything to start.

Reading starting from page: 0
Content: <last written content>
Select a page to read and/or write. Press enter if you want to skip.
<user input>
Write something new into the EEPROM! Press enter if you want to skip.
...

```
