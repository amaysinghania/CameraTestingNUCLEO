# STM32 CubeSAT Prototype Code

## Camera Testing
The camera testing folder contains the main files for the intial testing of the camera board with the NUCLEO board. Under Core/SRC, the main.c file contains the start to the code implementation of the OV5640 drivers. The drivers, under ov6540.c and ov5640_reg.c, implement the ov5640 camera module for the STM32 architecture. This should allow communication between the board and the camera. However, I am not able to understand how to set pins for I2C, DCMI, and the clock pin to drive the camera.
