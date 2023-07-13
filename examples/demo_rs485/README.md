demo_rs485 {#demo_rs485}
==========

## Overview
The `rs485` example showcases the utilization of the UART-driven RS485 lane.

## Supported Toolchain
Refer to the [Requirements](../../README.md#requirements) section.

## Hardware Requirements
- RS485 USB adapter

## Demo Preparation
1. Connect the J-Link Debug Probe to the board.
2. Connect a USB cable between the host PC and the J-Link debugger.
3. Connect another USB cable between the host PC and the board.
4. Attach the RS485 USB adapter to the RS485 port. Ensure the Tx and Rx lines are correctly connected.
5. Launch two serial terminals, such as "Tera Term".
6. In one terminal, select your board as the serial input (115200 BPS).
7. In the other terminal, choose the RS485 for connection (115200 BPS).
8. Download the program to the target board.
9. Start the debugger in your IDE to run the demo.

For additional instructions or requirements specific to your board, please refer to its README.

## Running the Demo
Upon successful execution, the terminal connected to the board will display:

```
TQ-Systems GmbH
This is a demo application for the device <module-board>.
Welcome to the demo version.

This is a demonstration of the RS485-Bus.
Successful Write of RS485: Hello! Type something! Press enter to send!

Waiting for input...

```

The terminal connected to the RS485 will show:

```
Hello! Type something! Press enter to send!

```

After providing input, the board terminal will display:

```
Successful Read of RS485: < user input >
Demonstration end.

```
