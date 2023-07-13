MBa117xL Board {#MBa117xL_Board}
==============

## Overview
This README provides a comprehensive guide for using the MBa117xL board with the TQMBa1176L module using the M4-Cortex and the M7-Cortex. It includes several demo applications that showcase the capabilities of the board and the MCUXpresso SDK software. The demos range from simple sanity checks like the "Hello World" demo to more complex demonstrations of inter-core communication in a multicore system.

## Trivial Hardware Requirements

| Hardware           | Description                                                                           | Pin Reference |
| ------------------ | ------------------------------------------------------------------------------------- | ------------- |
| Micro USB cable    | Required for connecting the board to your computer for programming and communication. | X1            |
| J-Link Debug Probe | Used for debugging and programming the board.                                         | X39           |
| Personal Computer  | Required for programming and interacting with the board.                              | -             |
| Power Supply       | A power supply with a voltage of 24V is required to power the board.                  | X13           |

Please note that these requirements are specific to the MBa117xL board using the TQMBa1176L module. Always refer to the specific READMEs of the demo you are running for any additional hardware requirements. Further informations can be found in the documentation directory.

## Serial port configuration
- Serial terminal has following settings:
   - 115200 baud rate
   - 8 data bits
   - No parity
   - One stop bit
   - No flow control
   - Select the COM-interface for the two cores.


## Demos
The demos that are included are listed as directories.

Each demo has its own README that provides more detailed information about the demo, including its purpose, how to prepare for the demo, and what to expect when running the demo.

## Debugging and Flash-Loading Instructions

When utilizing J-Link programs for debugging or loading firmware onto the flash device, it's crucial to configure the correct FLEXSPI1 pin multiplexing for the MBa117xL. This involves specifying the appropriate device configuration in the [launch.json](../../templates/launch.json) file:

```
 "-device",
 "MIMXRT1176xxxA_M7?BankAddr=0x30000000&Loader=nCS@AD18_CLK@AD19_D0@AD20_D1@AD21_D2@AD22_D3@AD23&BankAddr=0x60000000&Loader=nCS@SDB100_CLK@SDB101_D0@SDB102_D1@SDB103_D2@SDB104_D3@SDB105"
```

Additionally, with the version 7.92^ onwards in J-Link, you can now easily choose the required settings through the dropdown menu in the flash-bank setting, available across various J-Link programs including J-Link Flash, J-Link Flash Lite, and the J-Link GDB Server. 
