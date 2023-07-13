demo_afe {#demo_afe}
========

## Overview
The `afe` example demonstrates the spi-driven analog front end NAFE1388B40. The demonstration makes a single shot record of the voltages connected between AIP-pins and ground.


## Supported Toolchain
See [Requirements](../../README.md#requirements).

## Front End Interface Setting
The following table provides a generic connection guide:


| PIN  | Connects To | PIN/SOURCE              |
| ---- | ----------- | ----------------------- |
| AIxP | -           | Analog voltage max. 25V |
| GND  | -           | GND                     |

X: 1 <= x <= 4

## Demo Preparation
1. Connect the J-Link Debug Probe to the board.
2. Connect a USB cable between the host PC and the J-Link-debugger.
3. Connect a USB-Cable between the host PC and the board.
4. Open two serial terminal with the settings specified for the board.
5. Download the program to the target board.
6. Launch the debugger in your IDE to begin running the demo.

Please refer to the READMEs of the specific board you are using for any additional instructions or requirements.

## Running the Demo
Upon successful execution, the following message will be displayed in the terminal:

```
TQ-Systems GmbH
This is a demo application for the device <module-board>.
Welcome to the demonstration.

AFE demonstration example. Press any button.

[ OK ]  AFE13388 Chip Ready
 Partname: 1338 8b40
 SE_NR: 2b6e e066c

  Press any Button to initialize measurements.

  CHANNEL0 <value> V

  CHANNEL1 <value> V

  CHANNEL2 <value> V

  CHANNEL3 <value> V

  CHANNEL4 <value> V
         
  Temperature: <value> degree Celsius

...
