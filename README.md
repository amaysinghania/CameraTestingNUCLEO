# STM32 CubeSAT Prototype Code

## Camera Testing
The camera testing folder contains the main files for the intial testing of the camera board with the NUCLEO board. Under Core/SRC, the main.c file contains the start to the code implementation of the OV5640 drivers. The drivers, under ov6540.c and ov5640_reg.c, implement the ov5640 camera module for the STM32 architecture. This should allow communication between the board and the camera. However, I am not able to understand how to set pins for I2C, DCMI, and the clock pin to drive the camera.

## SD Card Debugging

### Overview
The project includes a comprehensive SD card debugging function `debug_sd_card()` in `main.c` to troubleshoot SD card communication issues. This function tests SD card functionality at three different levels and provides detailed error reporting via UART.

### Common SD Card Errors
Based on FATFS documentation, common errors include:
- **FR_OK = 0**: Succeeded
- **FR_DISK_ERR = 1**: A hard error occurred in the low level disk I/O layer
- **FR_INT_ERR = 2**: Assertion failed
- **FR_NOT_READY = 3**: The physical drive cannot work
- **FR_NO_FILESYSTEM = 13**: There is no valid FAT volume

### Debug Function Features

#### 1. Low-Level Disk Operations Testing
- Tests `disk_initialize()` and reports status flags (STA_NOINIT, STA_NODISK, STA_PROTECT)
- Tests `disk_status()` to check drive readiness
- Reads sector 0 (boot sector) and validates the 0x55AA signature
- Displays the first 16 bytes of the boot sector for inspection

#### 2. FATFS Mount Operations Testing
- Tests `f_mount()` with both immediate (opt=1) and delayed (opt=0) mounting options
- Provides detailed error code explanations for common mounting failures
- Tests `f_getfree()` to retrieve filesystem information (total/free sectors, cluster size)

#### 3. File Operations Testing
- Tests file creation with `f_open()` using FA_CREATE_ALWAYS | FA_WRITE flags
- Tests writing data with `f_write()` and reports bytes written
- Tests file reading with `f_read()` to verify data integrity
- Tests proper file closing and filesystem unmounting

### Hardware Configuration
- **SD Card Interface**: SPI1 (defined as SD_SPI_HANDLE in main.h)
- **Chip Select Pin**: PB3 (SD_CS_Pin)
- **SPI Pins**: 
  - SCK: PA5
  - MISO: PB4  
  - MOSI: PB5
  - CS: PB3

### Troubleshooting Guide

#### For Cycling Mount Errors (1, 3, 13):
1. **Check Physical Connections**: Verify all SPI wires (MISO, MOSI, SCK, CS) are properly connected
2. **SD Card Format**: Ensure SD card is formatted as FAT16 or FAT32
3. **SPI Clock Speed**: Verify SPI starts with slow clock (280 KHz) for initialization
4. **Power Supply**: Check SD card has stable 3.3V power supply
5. **Card Compatibility**: Test with different SD card brands/sizes

#### For f_write() Error 1 (FR_DISK_ERR):
1. **Low-Level Communication**: Check if disk_initialize() and disk_read() work properly
2. **Write Protection**: Verify SD card write-protect switch is not enabled
3. **SPI Timing**: Ensure adequate delays between SPI operations
4. **Buffer Alignment**: Check if write buffers are properly aligned
5. **Card Capacity**: Verify card has sufficient free space

### Using the Debug Function
The debug function runs automatically before the main SD card operations. Monitor UART output (115200 baud) to see detailed test results and identify the specific failure point.

### SD Card Driver Files
- `FATFS/Target/user_diskio_spi.c`: Custom SPI-based SD card driver implementation
- `FATFS/Target/user_diskio_spi.h`: Driver header file
- `FATFS/Target/user_diskio.c`: FATFS interface wrapper (heavily modified)
