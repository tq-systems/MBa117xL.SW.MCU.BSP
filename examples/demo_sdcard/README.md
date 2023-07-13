demo_sdcard {#demo_sdcard}
===========

## Overview
This example, `demo_sdcard`, demonstrates the usage of the SD Card. It initializes the card, logs the card information, performs read/write operations, and compares the read/write content for consistency. It also handles SD card detection and read-only mode efficiently.

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

```

TQ-Systems GmbH
This is a demo application for the device <module-board>.
Welcome to the demonstration.

Please press any Button to execute the SD Card test. Make sure you've inserted a SD Card!

<user input>

--> Card detected

Card size 61069312 * 512 bytes

Working condition:

  Voltage : <value> V

  Timing mode: SDR104

  Freq : <value> HZ

Read/Write/Erase the card continuously until encounter error......

Write/read one data block......
Compare the read/write content......
The read/write content is consistent.
Write/read multiple data blocks......
Compare the read/write content......
The read/write content is consistent.
Erase multiple data blocks......
Please press any Button to execute the SD Card test

End of demonstration.

```
