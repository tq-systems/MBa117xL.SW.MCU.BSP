demo_usb {#demo_usb}
========
## Overview
The `demo_usb_host_msd` example illustrates the implementation of a USB Host Mass Storage Device (MSD) using NXP's USB stack. It demonstrates initialization, enumeration, and communication with attached USB MSD devices, such as USB Flash drives, through the USB Host Controller Driver.

This example also demonstrates the utilization of Port Expander and I2C Master, which enables the IOT_MAIN_UART, and the LPI2C API to configure and interact with peripheral devices connected via I2C.

## Supported Toolchain
See [Requirements](../../../README.md#requirements).

## Demo Preparation
1. Connect the J-Link Debug Probe to the board.
2. Connect a USB cable between the host PC and the J-Link-debugger.
3. Connect a USB-Cable between the host PC and the board.
4. Open two serial terminal with the settings specified for the board.
5. Download the program to the target board.
6. Launch the debugger in your IDE to begin running the demo.


Please refer to the READMEs of the specific board you are using for any additional instructions or requirements.

## Running the Demo
Upon successful execution and connection of a USB device, the application should enumerate the attached device. In case the attached device is not supported, a message will be displayed in the terminal indicating "device not supported."

You should be able to interact with the connected USB devices and observe the communications and any corresponding outputs on the terminal. If there are any errors during enumeration or communication, appropriate error messages will be displayed in the terminal.
