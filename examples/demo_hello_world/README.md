demo_hello_world {#demo_hello_world}
================

## Overview
The 'hello_world' demo application serves as a sanity check for new SDK build environments and board bring-up. This demo prints the "Hello World" string to the terminal using the SDK UART drivers. The purpose of this demo is to demonstrate the use of the UART and provide a simple project for debugging and further development. Note: Please input one character at a time. If you input too many characters at once, the receiver may overflow because the low-level UART uses a simple polling method for receiving. If you want to try inputting many characters at once, define DEBUG_CONSOLE_TRANSFER_NON_BLOCKING in your project to use the advanced debug console utility.

## Supported Toolchain
See [Requirements](../../README.md#requirements).

## Demo Preparation
1. Connect the J-Link Debug Probe to your board.
2. Connect a USB cable between the host PC and the J-Link-debugger.
3. Connect a USB-Cable between the host PC and the board.
4. Open two serial terminal with the settings specified for the board.
5. Download the program to the target board.
6. Launch the debugger in your IDE to begin running the demo.

## Running the Demo
The log below shows the output of the Hello World demo in the terminal window:

```
TQ-Systems GmbH
This is a demo application for the device <module-board>.
Welcome to the demonstration.

hello world.
Type in anything:

```
