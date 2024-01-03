demo_digital_io {#demo_digital_io}
===============

## Overview
This example, ``demo digital IO``, demonstrates the usage of the port expander PCA9555 from with a GPIO-interface.

## Supported Toolchain
See [Requirements](../../README.md#requirements).

## Board Settings
The following table provides a generic connection guide for the port expander:

| PIN  | Connects to PIN/SOURCE |
| ---- | ---------------------- |
| 1A   | SOURCE (~5V)           |
| 3A   | 7A                     |
| 4A   | 8A                     |
| 5A   | 9A                     |
| 6A   | 10A                    |
| DGND | GND                    |

__NOTE__: GND pins are on the upper side of connector X37. These pins are interconnected.

__NOTE__: If the demonstration doesn't work properly, try increasing the source voltage on PIN 1A. 

## Demo Preparation
1. Connect the appropriate power supply and J-Link Debug Probe to the board.
2. Connect a USB cable between the host PC and the J-Link-debugger.
3. Connect a USB-Cable between the host PC and the Mainboard.
4. Open one serial terminal with the settings specified for the board.
5. Connect the pins according to the table above.
6. Download the program to the target board.
7. Launch the debugger in your IDE to begin running the demo.

## Running the Demo
Upon successful execution, the following message will be displayed in the terminal:

```
TQ-Systems GmbH
This is a demo application for the device <module>.
Welcome to the demonstration.

This demo is a demonstration of the portexpander working with digital IO Pins. Please Press a button to continue
[OK] Digital IO 1 Loopback
[OK] Digital IO 2 Loopback
[OK] Digital IO 3 Loopback
[OK] Digital IO 4 Loopback
Loopback ended ended.
Please press any button to execute the demonstration again.

...

```
