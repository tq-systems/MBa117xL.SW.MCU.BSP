demo_flexcan {#demo_flexcan}
============

## Overview
This example, `flexcan_ping_pong`, demonstrates the usage of two FlexCAN devices with an interrupt. In this scenario, FlexCAN Device 2 initiates communication by sending a message to FlexCAN Device 1. Subsequently, FlexCAN Device 1 responds by sending a message back to FlexCAN Device 2. This exchange is repeated 50 times, after which the user can input an 8-byte string.

## Supported Toolchain
See [Requirements](../../README.md#requirements).

## Board Settings
Ensure the J-Link debugger is correctly connected to the board. The following table provides a generic connection guide:

| Device 1 | Connects To | Device 2 |
| -------- | ----------- | -------- |
| CAN1_TX  | -           | CAN2_TX  |
| CAN1_RX  | -           | CAN2_RX  |
| GND      | -           | GND      |

## Demo Preparation
1. Connect the appropriate power supply and J-Link Debug Probe to the board.
2. Connect a USB cable between the host PC and the J-Link-debugger.
3. Connect a USB-Cable between the host PC and the Mainboard.
4. Open two serial terminal with the settings specified for the board.
5. Set the board with the terminal to the u-boot mode.
6. Download the program to the target board.
7. Launch the debugger in your IDE to begin running the demo.

## Running the Demo
Upon successful execution, the following message will be displayed in the terminal:

```
TQ-Systems GmbH
This is a demo application for the device <module-board>.
Welcome to the demonstration.

This demo shows the flexcan devices communication.
Please press anything to initialize.
Rx MB ID: 0x123, Rx MB data: 1, Time stamp: <value>
Rx MB ID: 0x321, Rx MB data: 2, Time stamp: <value>
Rx MB ID: 0x123, Rx MB data: 3, Time stamp: <value>
Rx MB ID: 0x321, Rx MB data: 4, Time stamp: <value>
Rx MB ID: 0x123, Rx MB data: 5, Time stamp: <value>
Rx MB ID: 0x321, Rx MB data: 6, Time stamp: <value>
...
Main demonstration done! 
Type in anything you want:

Rx MB ID: 0x123, Rx MB data: 1, Time stamp: <some time stamp>
<userInput>
```

Please replace `<userInput>` with your input.
