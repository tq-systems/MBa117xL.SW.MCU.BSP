# MBa117xL Board {#MBa117xL_Board}

## Table of Contents

[[_TOC_]]

## Overview
This README provides a comprehensive guide for using the MBa117xL board with the TQMBa1176L module using the M4-Cortex and the M7-Cortex. It includes several demo applications that showcase the capabilities of the board and the MCUXpresso SDK software. The demos range from simple sanity checks like the "Hello World" demo to more complex demonstrations.

## Software requirements

* General [requirements](../README.md#requirements)
* MCUXpresso [Secure Provisioning Tool](https://www.nxp.com/design/design-center/software/development-software/mcuxpresso-software-and-tools-/mcuxpresso-secure-provisioning-tool:MCUXPRESSO-SECURE-PROVISIONING#downloads): V7

## Hardware Requirements

| Hardware           | Description                                                                           | Pin Reference |
| ------------------ | ------------------------------------------------------------------------------------- | ------------- |
| Micro USB cable    | Required for connecting the board to your computer for programming and communication. | X1            |
| J-Link Debug Probe | Used for debugging and programming the board.                                         | X39           |
| Personal Computer  | Required for programming and interacting with the board.                              | -             |
| Power Supply       | A power supply with a voltage of 24V is required to power the board.                  | X13           |

Please note that these requirements are specific to the MBa117xL board using the TQMBa1176L module. Always refer to the specific READMEs of the demo you are running for any additional hardware requirements. Further informations can be found in the documentation directory.

## Serial Port Configuration

Configure the serial terminal with the following settings:
- 115200 baud rate
- 8 data bits
- No parity
- One stop bit
- No flow control
- Select the appropriate COM interface for the two cores.

## Applications

The MBa117xL supports the following applications:

| Application                                      | Purpose                                                                      |
| ------------------------------------------------ | ---------------------------------------------------------------------------- |
| [demo_afe](./demo_afe/README.md)                 | Demonstrates the Analog Front End (AFE) capabilities                         |
| [demo_digital_io](./demo_digital_io/README.md)   | Illustrates basic functions of the port expander                             |
| [demo_eeprom](./demo_eeprom/README.md)           | Illustrates the usage of the EEPROM M24C64                                   |
| [demo_ethernet](./demo_ethernet/README.md)       | Demonstrates Ethernet communication features                                 |
| [demo_flexcan](./demo_flexcan/README.md)         | Shows the functionalities of the Flexible Controller Area Network (FlexCAN)  |
| [demo_gpio_led](./demo_gpio_led/README.md)       | Demonstrates General-Purpose Input/Output (GPIO) with LED control            |
| [demo_hello_world](./demo_hello_world/README.md) | Basic "hello_world" demonstration                                            |
| [demo_iot](./demo_iot/README.md)                 | Showcases IoT module BG95-M4                                                 |
| [demo_PMIC](./demo_PMIC/README.md)               | Demonstrates PMIC functionalities                                            |
| [demo_rs485](./demo_rs485/README.md)             | Demonstrates operations over RS-485 communication standard                   |
| [demo_RTC](./demo_RTC/README.md)                 | Shows built-in RTC's functionalities                                         |
| [demo_sdcard](./demo_sdcard/README.md)           | Demonstrates reading from and writing to a Secure Digital (SD) card          |
| [demo_SE97BTP](./demo_SE97BTP/README.md)         | Showcases possible operations with the temperature-sensor and EEPROM SE97BTP |
| [demo_spi_flash](./demo_spi_flash/README.md)     | Demonstrates SPI flash memory (MX25R1635F) operations                        |
| [demo_usb](./demo_usb/README.md)                 | Demonstrates USB communication features                                      |

Each demo includes a README with more detailed information, including its purpose, preparation steps, and expectations when running the demo.

## Booting

Refer to [GDB-Server](#gdb-server) or follow instructions for [VS-Code debugging](#debugging) in order to load the targets after setting up the proper booting configuration. 

### Booting from Internal RAM

Load the desired target `.bin` file into address 0x00 or use a `.elf` file linked directly to the internal RAM. 

### Booting from Flash

When using J-Link for debugging or firmware loading, ensure the FLEXSPI1 pin multiplexing is correctly configured for the MBa117xL.

This are the settings for JLink-GDB related programs:

```
 "-device",
 "MIMXRT1176xxxA_M7?BankAddr=0x30000000&Loader=nCS@AD18_CLK@AD19_D0@AD20_D1@AD21_D2@AD22_D3@AD23&BankAddr=0x60000000&Loader=nCS@SDB100_CLK@SDB101_D0@SDB102_D1@SDB103_D2@SDB104_D3@SDB105"
```

__Attention__: Before booting, it is **mandatory** to set the right [fuses](#setting-fuses).

After setting fuses, set the dip switch as follow:

| S3-1 | S3-2 | S3-3 | S3-4 |
| ---- | ---- | ---- | ---- |
| OFF  | OFF  | OFF  | OFF  |

| S4-1 | S4-2 | S4-3 | S4-4 |
| ---- | ---- | ---- | ---- |
| OFF  | OFF  | OFF  | OFF  |

| S5-1 | S5-2 | S5-3 | S5-4 |
| ---- | ---- | ---- | ---- |
| OFF  | OFF  | OFF  | OFF  |

| S6-1 | S6-2 | S6-3 | S6-4 |
| ---- | ---- | ---- | ---- |
| OFF  | ON   | ON   | ON   |

## Debugging

Before debugging, specify the appropriate device configuration in the [launch.json](../templates/launch.json) file. Refer to the [launch.json](../templates/launch.json) example provided. Follow the instructions on [debugging with VS-Code](../README.md/#debugging-with-vs-code).

## GDB-Server

Use GDB for independent loading and debugging targets from your IDE:

  - Start GDB server.
  - Select the connection via USB.
  - Select device `MIMXRT1176xxxA_M7` and set the endian `Little Endian`.
  - Select `JTAG` as interface with the fixed speed of 4000 kHz.
    - __NOTE__: If debugging shouldn't work as expected, you can try changing the interface to SWD and/or varying the interface speed.
  - Select the right flash bank for the FlexSPI 1: 
    - `nCS@AD18_CLK@AD19_D0@AD20_D1@AD21_D2@AD22_D3@AD23&BankAddr=0x60000000&Loader=nCS@SDB100_CLK@SDB101_D0@SDB102_D1@SDB103_D2@SDB104_D3@SDB105`
  - Further options are optional.
  - Start then gdb via CLI.
  - Follow the command for booting:

   ```bat
   file <PATH.elf>
   target remote localhost:<port>
   monitor reset
   monitor halt
   load
   monitor go
   ```

## Setting fuses

To boot from the flash device, follow these steps:

- Power off the board.
- Set the dipswitch as follow:

| S3-1 | S3-2 | S3-3 | S3-4 |
| ---- | ---- | ---- | ---- |
| OFF  | OFF  | OFF  | OFF  |

| S4-1 | S4-2 | S4-3 | S4-4 |
| ---- | ---- | ---- | ---- |
| OFF  | OFF  | OFF  | OFF  |

| S5-1 | S5-2 | S5-3 | S5-4 |
| ---- | ---- | ---- | ---- |
| OFF  | OFF  | OFF  | OFF  |

| S6-1   | S6-2    | S6-3 | S6-4 |
| ------ | ------- | ---- | ---- |
| **ON** | **OFF** | ON   | ON   |

- After setting up the boot mode, establish a connection between the host computer and the target hardware via UART1.
- Open the [Secure Provisioning Tool](#software-requirements).
- Open the processor menu in the upper left conner and select the target processor `MIMXRT1176`.
- Select UART as connection type and set the COM port to the one that is used for the target hardware connection. The default baud rate setting of 115200 can be kept. Check the connection with the “Test Connection” button afterwards.
- Now open the `Build image` tab in right below the target processor in the upper left corner.
- Select `OTP Configuration`.
- The OTP Configuration menu opens and asks to read the current values from the target processor,
select “Yes” here. 
- If the fuses can be read correctly the status message “Successfully updated fuses values” is shown in the message area at the bottom of the “OTP Configuration” window. 
- If something went wrong please check the connection to the target hardware and try to read the fuses again by clicking the “Read” button at the window bottom.
- Now select the `fuse 0x9A0` from the Boot param section and select the wildcard on bit 10.
- When clicking on the wildcard it changes from * to 1. The “Required value” should now be `0x400`. **Double check these values.**
 - __ATTENTION__: A **wrong setting** on `fuse 0x9A0` **leads to a bricked target hardware** that no longer can be booted from the QSPI NOR Flash.
- Select `Advanced Mode` in the bottom left corner.
- Now click on `Burn`. Confirm.
- You should have got a success message.
