/*V
 *	OV5640 Adafruit Camera Breakout Board I2C Driver
 *
 *  Author: Amay Singhania
 *  Created on: Jul 9, 2025
 *
 */

#include "adafruit_OV5640.h"

/*
 * INITIALIZATION
 */

uint8_t OV5640_Init(OV5640 *dev, I2C_HandleTypeDef *i2cHandle){

	// Set struct Parameters
	dev -> i2cHandle = i2cHandle;
	dev -> isInitialized = 0;

	// Store the number of transaction errors (to be returned)
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	uint8_t regData;

	status = OV5640_ReadRegister(dev, OV5640_CHIP_ID_HIGH_BYTE, &regData);
	errNum += (status != HAL_OK);

	if (regData != OV5640_ID_DEFAULT) {

		printf("ID is incorrect\n");
//		return 255;
	} else {
		printf("ID is correct\n");
	}

	status = HAL_OK;

	// Set default registers to default values
//	status = OV5640_WriteRegisters_Dict(dev, &DataOnInit, sizeof(&DataOnInit));
	for (uint32_t i = 0; i < sizeof(&DataOnInit)/4; i++) {
			uint8_t data = (uint8_t) DataOnInit[i][1];

			if (DataOnInit[i][0] == OV5640_REG_DLY){
				HAL_Delay(data);
				continue;
			}

			status = OV5640_WriteRegister(
				dev,         			// I2C Handle
				DataOnInit[i][0],      	// Register address
				&data                 	// Pointer to data
			);

			if (status != HAL_OK) {
				// Handle error (e.g. retry, log, break, etc.)
				return status;
			}

			HAL_Delay(1); // Small delay may help with some sensors
		}
	errNum += (status != HAL_OK);
	if (status == HAL_OK){
		printf("Successfully written to all registers");
	}

	return errNum;
}


/*
 * Low Level Functions
 */

HAL_StatusTypeDef OV5640_ReadRegister(OV5640 *dev, uint16_t reg, uint8_t *data){
	return HAL_I2C_Mem_Read(dev -> i2cHandle, OV5640_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef OV5640_ReadRegisters(OV5640 *dev, uint16_t reg, uint8_t *data, uint16_t length){
	return HAL_I2C_Mem_Read(dev -> i2cHandle, OV5640_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef OV5640_WriteRegister(OV5640 *dev, uint16_t reg, uint8_t *data){
	return HAL_I2C_Mem_Write(dev -> i2cHandle, OV5640_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef OV5640_WriteRegisters_Dict(OV5640 *dev, uint16_t *reg_data[][2], int *size_data){
	HAL_StatusTypeDef status;

	for (uint32_t i = 0; i < *size_data/4; i++) {
		uint8_t data = (uint8_t) *reg_data[i][1];

		if (*reg_data[i][0] == OV5640_REG_DLY){
			HAL_Delay(data);
			continue;
		}

		status = OV5640_WriteRegister(
			dev,         			// I2C address (7-bit << 1)
			*reg_data[i][0],      	// Register address
			&data                 	// Pointer to data
		);

		if (status != HAL_OK) {
			// Handle error (e.g. retry, log, break, etc.)
			return status;
		}

		HAL_Delay(1); // Small delay may help with some sensors
	}
	return status;
}


