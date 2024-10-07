Bootloader for STM32 Nucleo-F446RE
This repository contains a custom UART-based bootloader for the STM32 Nucleo-F446RE microcontroller. The bootloader allows for secure and flexible firmware updates and provides a set of commands to interact with the microcontroller’s memory and control various protection mechanisms.

Project Overview:
The custom bootloader is designed to enable secure and flexible firmware updates for the STM32 Nucleo-F446RE development board. The bootloader can communicate over UART, receive command packets from a host PC, and perform various memory and system operations. It can handle:
- Jumping to the main user application stored in Flash memory.
- Reading and writing to specific memory locations.
- Erasing Flash sectors.
- Enabling or disabling read/write protection for specific memory sectors.

Features: 
- UART Communication: Receives commands from the host and sends responses.
- Firmware Update: Supports in-field firmware updates.
- Memory Operations: Supports reading, writing, and erasing memory.
- Read/Write Protection: Configures read and write protection for specific sectors.
- Debug Mode: Includes debug messages for detailed bootloader operation status.

Prerequisites
- STM32 Nucleo-F446RE Development Board
- Keil-MDK-5 IDE for ARM Cortex-M based microcontrollers
- STM32CubeMX for hardware configuration and code generation
- USB-to-Serial Converter for UART communication with the host
