demo_ethernet {#demo_ethernet}
=============

## Overview
This example, `demo_ethernet`, demonstrates the Ping functionality on the lwIP TCP/IP stack utilizing the ICMP protocol. The application sends ICMP echo requests periodically to a PC and processes the replies. To send an ICMP echo request to the board, type "ping $board_address" in the PC command window. The lwIP stack then sends the ICMP echo reply back to the PC.

## Supported Toolchain
See [Requirements](../../../README.md#requirements).

## Hardware Requirements
- Mini/micro USB cable
- Network cable (RJ45 standard)
- MIMXRT1170-EVK board
- Personal Computer

## Board Settings

No special settings are required for this demo.

## Demo Preparation

1. Connect a USB cable between the PC host and the OpenSDA (or USB to Serial) USB port on the target board.
2. Open a serial terminal with the settings specified for the board.
3. Connect an Ethernet Cable to the target board's RJ45 port and to your PC network adapter.
4. Adapt the IP4_ADDR in the demo_ethernet.h file.
5. Download the program to the target board.

## Running the Demo

__NOTE__: Port X36 isn't supported yet.

Upon successful execution, the following logs will be displayed in the terminal (values are exemplary):

``` txt

Initializing PHY...

************************************************
 PING example
************************************************
 IPv4 Address     : 192.168.0.102
 IPv4 Subnet mask : 255.255.255.0
 IPv4 Gateway     : 192.168.0.100
************************************************
ping: send
192.168.0.100

ping: recv
192.168.0.100
 4 ms

...

```
