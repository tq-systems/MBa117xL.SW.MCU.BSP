demo_gpio_led {#demo_gpio_led}
=============

## Overview

The `gpio_led` project is a demonstration program that utilizes the KSDK software to manipulate the general-purpose
outputs. This example is supported by the set, clear, and toggle write-only registers for each port output data register.
The example alternates to illuminate the LED.

## Supported Toolchain

See [Requirements](../../README.md#requirements).

## Demo Preparation

1. Connect the J-Link Debug Probe to the board.
2. Connect a USB cable between the host PC and the USB port on the board.
3. Open two serial terminal with the settings specified for the board.
4. Download the program to the target board.
5. Launch the debugger in your IDE to begin running the demo.

Please refer to the READMEs of the specific board you are using for any additional instructions or requirements.

## Running the Demo

Upon successful execution, the following message will be displayed in the terminal:

``` txt

TQ-Systems GmbH
This is a demo application for the device <module-board>.
Welcome to the demonstration.

GPIO Driver example
The LED is blinking.

```

The USER_LED_1 should be blinking.
