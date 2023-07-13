demo_iot {#demo_iot}
========

## Overview
The ``demo_iot`` is designed to demonstrate interactions with an IoT device through UART, running on a FreeRTOS operating system. 

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
Upon successful execution, the following message will be displayed in the terminal:

```

TQ-Systems GmbH
This is a demo application for the device <module-board>.
Welcome to the demonstration.

[INFO] GNSS peripheral initialized.
This is a demonstration of the IOT-Device-operation.

IOT-Device demonstation. Type help to get a list of commands.

<user input: help>

Command: 'START' - starts IOT device
Command: 'END' - shut down IOT device
Command: 'GNSS INIT' - Initializes GPS
Command: 'GNSS READ' - Reads GNSS output
For any other specific commands refer to datasheet of IOT device from Quectel.
Mind that in order to read data from GNSS you need to initialize it via command.

<user input: START>

[ OK ]  LPUART6 routed to IOT
[ OK ]  Toggle BG95_PWR
[INFO]  Start of UART6 Output

RDY

APP RDY

 ATI
Quectel
BG95-M4
Revision: BG95M4LAR02A02

OK


 AT+GSN
865341040062814

OK


[INFO]  BG95-M4 is now ready for operation.
Command was sent. Verify execution. Type in any command or  'help'.

...

```

If the IOT_Device was successfully started the IOT NET-STATUS LED will blink on the board.
