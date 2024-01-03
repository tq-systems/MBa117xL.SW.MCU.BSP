demo_SE97BTP {#demo_SE97BTP}
============

## Overview
The `i2c_SE97BTP` example demonstrates the use of the i2c driver as a master to read the built-in temperature sensor on the supported board. In this example, the i2c instance of the board is used to poll one built-in temperature sensor. Further this examples shows how to access the eeprom of the SE97B sensor via I2C. 

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

Demonstration of the SE97BTP device. Press any button to initialize.
Temperature of Mainboard in °C: <value>
Reading starting from address: <value>
Content: <string>
Select an address for r/w.
<user input>
Write something new into the EEPROM! Press enter if you want to skip.
<user input>
Temperature of Mainboard in degree Celsius: <new value>
Reading starting from address: <new value>
Content: <new content>
Select an address for r/w.
...

```
