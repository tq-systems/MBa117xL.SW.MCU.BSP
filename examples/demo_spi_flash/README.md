demo_spi_flash {#demo_spi_flash}
==============

## Overview
This example, `demo_spi_flash`, demonstrates the operation of reading and writing to a SPI flash, specifically to an MX25R1635F device, using the LPSPI interface. The user can read the given address, and has the option to write something new into the MX25 flash or skip to continue.

## Supported Toolchain
See [Requirements](../../README.md#requirements).

## Demo Preparation
1. Connect the J-Link Debug Probe to the board.
2. Connect a USB cable between the host PC and the J-Link-debugger.
3. Connect a USB-Cable between the host PC and the board.
4. Open two serial terminal with the settings specified for the board.
5. Download the program to the target board.
6. Launch the debugger in your IDE to begin running the demo.

Please refer to the READMEs of the specific board you are using for any additional instructions or requirements.

## Demo Preparation
1. Connect the appropriate power supply and J-Link Debug Probe to the board.
2. Connect a USB cable between the host PC and the J-Link-debugger.
3. Connect a USB-Cable between the host PC and the Mainboard.
4. Open one serial terminal with the settings specified for the board.
5. Download the program to the target board.
6. Launch the debugger in your IDE to begin running the demo.

## Running the Demo
Upon successful execution, the user will be prompted to press any button to begin. The SPI Test will be executed, detecting the MX25 and reading its security register. The user can then read the given address and choose to write something new into the MX25 or skip.

```

TQ-Systems GmbH
This is a demo application for the device <module-board>.
Welcome to the demonstration.

[Display typical output logs here, example:]
MX25 detected
[Values read from security register, e.g., “02x 03x”]
Reading given address:
[Displayed read value]
Write something new into the MX25! Press enter if you want to skip.
[User Input or Skip]

...

```
