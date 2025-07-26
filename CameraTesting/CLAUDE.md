# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an STM32H7-based embedded camera system project designed for CubeSat applications. The project interfaces an OV5640 camera module with SD card storage using the STM32H753ZI microcontroller on a NUCLEO-H753ZI development board.

## Hardware Configuration

- **Microcontroller**: STM32H753ZIT6 (ARM Cortex-M7 @ 480MHz)
- **Development Board**: NUCLEO-H753ZI
- **Camera Module**: OV5640 (640x480 resolution configured)
- **Storage**: SD card via SDMMC1 interface
- **Communication**: I2C1, USART2/3
- **File System**: FatFS

## Build System

This project uses the STM32CubeIDE toolchain with GNU ARM GCC compiler:

### Build Commands
```bash
# Build the project (from STM32CubeIDE or command line)
make -C Debug

# Clean build artifacts
make -C Debug clean

# Flash and debug (requires ST-Link)
# Use STM32CubeIDE's integrated debugging or:
# st-flash write Debug/CameraTesting.elf 0x8000000
```

### Key Build Artifacts
- `Debug/CameraTesting.elf` - Main executable
- `Debug/CameraTesting.map` - Memory map file
- `Debug/CameraTesting.list` - Assembly listing

## Project Architecture

### Core Structure
- **Core/Src/main.c**: Main application logic, camera initialization, and SD card operations
- **Core/Inc/main.h**: Main header with system includes and defines
- **Core/Src/stm32h7xx_hal_msp.c**: Hardware abstraction layer configuration
- **Core/Src/stm32h7xx_it.c**: Interrupt handlers

### Camera System
- **Core/Inc/ov5640.h & Core/Src/ov5640.c**: OV5640 camera driver
- **Core/Inc/ov5640_reg.h & Core/Src/ov5640_reg.c**: Camera register definitions
- Camera I2C address: 0x3C
- Frame buffer size: 640x480 pixels
- Control pins: Reset (PB0), Power-down (PB14)

### Storage System
- **FATFS/**: FatFS middleware integration
- **FATFS/Target/bsp_driver_sd.c**: SD card BSP driver
- **FATFS/Target/sd_diskio.c**: SD card disk I/O interface
- **FATFS/App/fatfs.c**: FatFS application layer
- SD card interface via SDMMC1 (4-bit wide bus mode)

### Key Peripherals Configuration
- **SDMMC1**: SD card interface (PC8-D0, PC9-D1, PC10-D2, PC11-D3, PC12-CLK, PD2-CMD)
- **I2C1**: Camera communication (PB8-SCL)
- **USART2**: Debug/communication (PA2-TX, PA3-RX)
- **USART3**: Additional communication (PD8-TX, PD9-RX)

## Development Notes

### Clock Configuration
- System clock: 480MHz
- AHB clock: 240MHz
- APB1 clock: 120MHz
- APB2 clock: 30MHz
- SDMMC clock: 240MHz (divided by 8 = 30MHz)

### Memory Configuration
- Flash: STM32H753ZITX_FLASH.ld
- RAM: STM32H753ZITX_RAM.ld
- Cache disabled (CPU_ICache=Disabled, CPU_DCache=Disabled)

### Current Development Status
Based on recent commits, the project is currently debugging SD card and camera integration:
- SD card drivers are implemented but experiencing write failures
- Camera initialization has been removed for debugging
- FatFS integration is partially working
- Debug functions are in place for troubleshooting

### Common Development Tasks

#### Debugging SD Card Issues
The project currently has SD card write failures. Key areas to investigate:
- Check SDMMC clock configuration and timing
- Verify FatFS configuration in ffconf.h
- Review sd_diskio.c implementation
- Test with different SD card sizes/types

#### Camera Development
- OV5640 configuration via I2C
- Frame capture and processing
- Integration with storage system
- Power management for camera module

#### Testing
- Use STM32CubeIDE's integrated debugger
- Monitor via USART2 for debug output
- Test SD card operations with various file sizes
- Validate camera image capture quality