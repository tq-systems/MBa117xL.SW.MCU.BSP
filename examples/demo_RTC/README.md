demo_RTC {#demo_RTC}
========

## Overview
The `demo_RTC` example demonstrates the use of the internal RTC in M-core MCUs. Also it demonstrates how to communicate to a exemplary external i2c-driven RTC.

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

## Running the Demo
Upon successful execution, the following message will be displayed in the terminal:

```

TQ-Systems GmbH
This is a demo application for the device <module-board>.
Welcome to the demonstration.

Reading...

Current datetime, module: 2023-11-12 13:30:20
Current datetime, internal: 2023-11-12 13:30:20

...

```
