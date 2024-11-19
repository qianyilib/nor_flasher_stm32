# NOR Flash Firmware Burner （Test on S29GL064N90TFI04）

This project provides a firmware burning solution for the S29GL064N90TFI04 NOR Flash memory. It leverages UART communication to write, read, and erase firmware data to and from the flash memory.

Ensure that the [NOR Flash PC](https://github.com/qianyilib/nor_flasher_pc.git) is properly installed and running on your computer.

## Features

- **Write Firmware**: Writes firmware data to NOR flash in blocks.
- **Read Firmware**: Reads firmware data from NOR flash.
- **Erase Blocks**: Erases individual blocks of NOR flash.
- **CRC Verification**: Ensures data integrity using CRC16 checksum.
- **UART Communication**: Uses UART for communication between the host and the microcontroller.
- **Reset Support**: Allows for a software-triggered system reset after operations.

## Hardware Requirements

- **Microcontroller**: STM32 or any other compatible MCU with UART support.
- **Flash Memory**: S29GL064N90TFI04 NOR Flash.
- **UART Interface**: To communicate with the host PC or other devices for firmware operations.
- **External Memory**: The firmware data will be transferred via UART and stored in NOR flash.

## Software Requirements

- **STM32 HAL Library**: The code is built using STM32 HAL drivers for hardware abstraction.
- **Keil/STM32CubeIDE**: You can use Keil or STM32CubeIDE for building and flashing the firmware to the microcontroller.

## Build the Project
Open the generated project in STM32CubeIDE or Keil.
Build the project to compile the firmware.

## Flash the Firmware
Flash the compiled firmware to your STM32 microcontroller using a compatible programmer (e.g., ST-Link).

## Test the Firmware
Once the firmware is flashed, you can test the functionality by connecting the microcontroller's UART interface to a terminal program or any UART-to-USB adapter.

Use the following commands to interact with the firmware:

- **Write Firmware**: Send a packet to write the firmware to NOR flash.
- **Read Firmware**: Send a packet to read the firmware from NOR flash.
- **Erase Block**: Send a packet to erase specific blocks in the flash memory.
- **Reset**: Trigger a software reset after the operations.

Communication Protocol
The communication between the microcontroller and the host is based on UART packets with the following structure:

- **Packet Format**

```C
typedef struct {
    uint8_t  packetType;    // Packet type identifier
    uint8_t  command;       // Command (e.g., WRITE, READ, ERASE)
    uint32_t length;        // Length of data to be transferred
    uint32_t addr;          // Start address in NOR flash
    uint16_t crc;           // CRC16 checksum for integrity verification
} UartPacket;

```

- **Packet Types**

```
0x01: Write firmware
0x02: Read firmware
0x03: Write data
0x04: Read data
0x06: Erase block
0x07: Erase all blocks
0x09: Reset
```

- **Response Packet**

The microcontroller will respond with the following structure:

```C
typedef struct {
    uint8_t  packetType;    // Same packet type as request
    uint8_t  status;        // Status of the operation (0x00 = OK, 0x01 = Error)
    uint16_t crc;           // CRC16 checksum
} UartResponse;

```

- **CRC16 Calculation**

All packets are verified using a CRC16 checksum for data integrity. The CalculateCRC16 function computes the checksum for a given packet.


- **Example Command Flow**

Host sends a WRITE packet with the firmware data.
Microcontroller receives the data, writes it to NOR flash, and sends an OK response.
If there is any error (e.g., write failure), the microcontroller will send an ERROR response.
Host can request a READ operation to verify the data.

- **License**

This project is open-source and released under the MIT License. Feel free to fork, modify, and contribute.

- **Acknowledgements**

STM32 HAL library
S29GL064N90TFI04 NOR Flash datasheet

Yong for the initial development

- **Contact**
If you have any questions, please open an issue on GitHub or contact [mutouman@gmail.com].

