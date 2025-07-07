# OV5640 Camera Testing with STM32H753ZI NUCLEO

This project implements OV5640 camera module testing with the STM32H753ZI NUCLEO board, including proper pin configuration for I2C, DCMI, and clock interfaces.

## 📋 Pin Configuration

### I2C Communication (Camera Control)
| STM32 Pin | Function | Camera Pin | Purpose                       |
| --------- | -------- | ---------- | ----------------------------- |
| PB6       | I2C1_SCL | SCL        | I2C Clock for register access |
| PB9       | I2C1_SDA | SDA        | I2C Data for register access  |

### DCMI Data Interface (Image Data)
| STM32 Pin | Function    | Camera Pin | Purpose                    |
| --------- | ----------- | ---------- | -------------------------- |
| PA4       | DCMI_HSYNC  | HREF       | Horizontal synchronization |
| PA6       | DCMI_PIXCLK | PCLK       | Pixel clock from camera    |
| PC6       | DCMI_D0     | D0         | Data bit 0                 |
| PC7       | DCMI_D1     | D1         | Data bit 1                 |
| PC8       | DCMI_D2     | D2         | Data bit 2                 |
| PC9       | DCMI_D3     | D3         | Data bit 3                 |
| PE4       | DCMI_D4     | D4         | Data bit 4                 |
| PD3       | DCMI_D5     | D5         | Data bit 5                 |
| PE5       | DCMI_D6     | D6         | Data bit 6                 |
| PE6       | DCMI_D7     | D7         | Data bit 7                 |
| PG9       | DCMI_VSYNC  | VSYNC      | Vertical synchronization   |

### Clock & Control
| STM32 Pin | Function    | Camera Pin | Purpose                              |
| --------- | ----------- | ---------- | ------------------------------------ |
| PA8       | MCO1        | XCLK       | 24MHz master clock to camera         |
| PB0       | GPIO_Output | RESET      | Camera reset control (optional)      |
| PB14      | GPIO_Output | PWDN       | Camera power down control (optional) |

### Debug Interface
| STM32 Pin | Function  | Purpose      |
| --------- | --------- | ------------ |
| PA9       | USART1_TX | Debug output |
| PB15      | USART1_RX | Debug input  |

## 🔧 Hardware Setup

### Required Connections
1. **Power**: Connect camera VCC to 3.3V, GND to GND
2. **I2C**: Connect SCL and SDA with appropriate pull-up resistors (typically 4.7kΩ)
3. **Clock**: Connect PA8 to camera XCLK pin
4. **Data**: Connect all DCMI data pins (D0-D7) and sync pins (HSYNC, VSYNC, PIXCLK)
5. **Optional**: Connect reset and power-down control pins

### Camera Module Configuration
- **I2C Address**: 0x78 (0x3C << 1)
- **Input Clock**: 24MHz from MCO1
- **Data Format**: 8-bit parallel (DCMI)
- **Initial Resolution**: VGA (640x480)
- **Pixel Format**: RGB565

## 🚀 Testing Procedure

### 1. Build and Flash
```bash
# Build the project in STM32CubeIDE or using command line
# Flash to NUCLEO-H753ZI board
```

### 2. Serial Monitor Setup
- **Baud Rate**: 115200
- **Data Bits**: 8
- **Stop Bits**: 1
- **Parity**: None
- **Flow Control**: None

### 3. Expected Debug Output
```
OV5640 Camera Test Starting...
Powering up camera...
Resetting camera...
Registering camera I/O...
Camera I/O registration: SUCCESS
Reading camera ID...
Camera ID read: SUCCESS (ID=0x5640)
OV5640 camera detected!
Initializing camera with VGA resolution...
Camera initialization: SUCCESS
Camera is ready for image capture!
```

### 4. LED Status Indicators
- 🟢 **Green LED (LD1)**: I2C registration successful
- 🔵 **Blue LED (LD2)**: Camera ID read successfully
- 🔴 **Red LED (LD3)**: Error occurred during setup

## 🔍 Troubleshooting Guide

### Camera ID Read Fails
**Symptoms**:
- Red LED turns on
- Debug output shows "Camera ID read: FAILED"

**Solutions**:
1. ✅ **Check I2C Connections**:
   - Verify PB6 → SCL connection
   - Verify PB9 → SDA connection
   - Ensure 4.7kΩ pull-up resistors on both lines

2. ✅ **Verify Power Supply**:
   - Check 3.3V supply to camera module
   - Ensure stable power (measure with multimeter)
   - Check all ground connections

3. ✅ **Clock Signal Verification**:
   - Use oscilloscope to verify 24MHz signal on PA8
   - Camera won't respond to I2C without proper clock

4. ✅ **I2C Address Check**:
   - Some camera modules use 0x60 instead of 0x78
   - Try modifying `OV5640_I2C_ADDRESS` in main.c

### Clock Generation Issues
**Symptoms**:
- No camera response
- I2C communication fails

**Solutions**:
1. **Verify MCO1 Configuration**:
   - Check PA8 pin configuration in CubeMX
   - Ensure MCO1 source is HSI48 with /2 divider
   - Measure clock frequency with oscilloscope

2. **Check System Clock**:
   - Verify HSI48 is enabled and stable
   - Check PLL configuration

### Communication Timeouts
**Symptoms**:
- Intermittent I2C failures
- Long delays in responses

**Solutions**:
1. **I2C Bus Issues**:
   - Check for proper pull-up resistors (4.7kΩ recommended)
   - Reduce I2C speed if needed
   - Check for electrical noise on I2C lines

2. **Timing Issues**:
   - Increase delays in initialization sequence
   - Check camera datasheet for minimum timing requirements

### Camera Initialization Fails
**Symptoms**:
- Camera ID reads correctly but initialization fails
- Debug shows "Camera initialization: FAILED"

**Solutions**:
1. **Configuration Issues**:
   - Verify camera module supports selected resolution/format
   - Check register configuration sequences
   - Try different pixel formats or resolutions

2. **Hardware Issues**:
   - Verify all DCMI data pins are connected
   - Check sync signal connections (HSYNC, VSYNC)
   - Ensure pixel clock (PCLK) connection

## 🔧 Advanced Debugging

### Using Logic Analyzer
1. **I2C Communication**:
   - Monitor SCL/SDA lines during camera ID read
   - Verify proper start/stop conditions and ACK/NACK responses

2. **Clock Signals**:
   - Verify XCLK (24MHz input to camera)
   - Monitor PCLK (pixel clock output from camera)

### Oscilloscope Measurements
1. **Power Supply**:
   - Check for stable 3.3V with minimal ripple
   - Verify power-up sequence timing

2. **Signal Integrity**:
   - Check rise/fall times on I2C lines
   - Verify clock signal quality and frequency

## 📷 Next Steps

Once camera initialization succeeds:

### 1. Image Capture Setup
- Configure DCMI DMA for continuous capture
- Set up frame buffers for image storage
- Implement interrupt handlers for frame complete

### 2. Image Processing
- Add format conversion (RGB565 to other formats)
- Implement basic image processing algorithms
- Add compression/streaming capabilities

### 3. Additional Features
- Support multiple resolutions
- Add camera parameter controls (brightness, contrast, etc.)
- Implement different pixel formats (YUV, JPEG, etc.)

## 📚 Useful Resources

- [OV5640 Datasheet](https://cdn.sparkfun.com/datasheets/Sensors/LightImaging/OV5640_datasheet.pdf)
- [STM32H7 DCMI Reference Manual](https://www.st.com/resource/en/reference_manual/rm0433-stm32h742-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [STM32CubeMX Configuration Guide](https://www.st.com/resource/en/user_manual/um1718-stm32cubemx-for-stm32-configuration-and-initialization-c-code-generation-stmicroelectronics.pdf)

## 🐛 Common Issues & Solutions

| Issue                 | Cause                | Solution                       |
| --------------------- | -------------------- | ------------------------------ |
| No I2C response       | Missing pull-ups     | Add 4.7kΩ resistors to SCL/SDA |
| Wrong camera ID       | Incorrect address    | Try 0x60 instead of 0x78       |
| Clock not detected    | MCO1 not configured  | Check PA8 alternate function   |
| Power issues          | Insufficient current | Use dedicated 3.3V regulator   |
| Intermittent failures | Loose connections    | Check all solder joints        |

## 📝 Configuration Notes

- **I2C Speed**: Standard mode (100kHz) recommended for initial testing
- **DCMI Mode**: 8-bit external sync mode
- **Clock Source**: HSI48 divided by 2 for 24MHz output
- **Resolution**: VGA (640x480) for initial testing
- **Format**: RGB565 for compatibility
