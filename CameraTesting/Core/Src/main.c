/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <ov5640.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OV5640_I2C_ADDRESS    (0x3C)  // OV5640 I2C address (0x3C << 1)

// Optional camera control pins (you can configure these in CubeMX if needed)
#define CAM_RESET_PIN         GPIO_PIN_0
#define CAM_RESET_PORT        GPIOB
#define CAM_PWDN_PIN          GPIO_PIN_14
#define CAM_PWDN_PORT         GPIOB

#define FRAME_WIDTH  640
#define FRAME_HEIGHT 480
#define FRAME_BUFFER_SIZE (FRAME_WIDTH * FRAME_HEIGHT)

#define True 							1
#define False 						0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
OV5640_Object_t cam;
OV5640_IO_t io;
int32_t ret;
uint32_t camera_id;

uint8_t frame_buffer[FRAME_BUFFER_SIZE] __attribute__((section(".sram1")));
volatile uint8_t camImgReady = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_DCMI_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
// Camera I2C communication functions
static int32_t OV5640_IO_Init(void);
static int32_t OV5640_IO_DeInit(void);
static int32_t OV5640_IO_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
static int32_t OV5640_IO_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
static int32_t OV5640_IO_GetTick(void);
static void Camera_IO_Init(void);
static void Camera_Reset(void);
static void Camera_PowerUp(void);
static void capture_image(void);
static void output_to_SD(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Test OV5640 using the BSP driver functions
  * @param  None
  * @retval None
  */
void InitOV5640(void)
{
  Camera_PowerUp();
  Camera_Reset();
	Camera_PowerUp();

  printf("=== OV5640 BSP Driver Test ===\r\n");

  // Initialize I/O structure
  Camera_IO_Init();

  // Register bus I/O
  printf("Registering BSP I/O...\r\n");
  ret = OV5640_RegisterBusIO(&cam, &io);
  if (ret == OV5640_OK) {
    printf("✓ BSP I/O registration successful\r\n");
  } else {
    printf("✗ BSP I/O registration failed (ret=%ld)\r\n", ret);
    return;
  }

  // Test ReadID function
  printf("Testing OV5640_ReadID()...\r\n");
  ret = OV5640_ReadID(&cam, &camera_id);
  if (ret == OV5640_OK) {
    printf("✓ OV5640_ReadID successful: ID=0x%04lX\r\n", camera_id);
    if (camera_id == 0x5640) {
      printf("✓ Correct OV5640 chip detected\r\n");
      BSP_LED_On(LED_BLUE);
    } else {
      printf("⚠ Unexpected chip ID (expected 0x5640)\r\n");
    }
  } else {
    printf("✗ OV5640_ReadID failed (ret=%ld)\r\n", ret);
  }

  // Test camera capabilities
  OV5640_Capabilities_t caps;
  printf("Testing OV5640_GetCapabilities()...\r\n");
  ret = OV5640_GetCapabilities(&cam, &caps);
  if (ret == OV5640_OK) {
    printf("✓ Camera capabilities read successfully\r\n");
    printf("  Resolution config: %ld\r\n", caps.Config_Resolution);
    printf("  Brightness config: %ld\r\n", caps.Config_Brightness);
    printf("  Contrast config: %ld\r\n", caps.Config_Contrast);
  } else {
    printf("✗ Failed to read camera capabilities (ret=%ld)\r\n", ret);
  }

  // Test camera initialization
  printf("Testing OV5640_Init()...\r\n");
  ret = OV5640_Init(&cam, OV5640_R640x480, OV5640_RGB565);
  if (ret == OV5640_OK) {
  	ret = OV5640_SetPCLK(&cam, OV5640_PCLK_24M);
  	if (ret == OV5640_OK) {
			printf("✓ OV5640_Init successful (VGA, RGB565)\r\n");
			printf("✓ Camera is ready for image capture!\r\n");
			BSP_LED_On(LED_GREEN);
  	} else {
      printf("✗ OV5640_SetPCLK failed (ret=%ld)\r\n", ret);
    }
  }
  else {
    printf("✗ OV5640_Init failed (ret=%ld)\r\n", ret);
  }

  printf("=============================\r\n\r\n");
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_DCMI_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  InitOV5640();
  OV5640_Start(&cam);
  printf("Taking Photo\r\n");
  capture_image();

  output_to_SD();

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  	HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV8;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI48, RCC_MCODIV_2);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_HIGH;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_ENABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10707DBC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, Shutdown_Pin|Reset_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Shutdown_Pin */
  GPIO_InitStruct.Pin = Shutdown_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Shutdown_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Reset_Pin */
  GPIO_InitStruct.Pin = Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Reset_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Captures a frame from the camera using DCMI and DMA.
  * @retval None
  */
static void capture_image(void)
{
  camImgReady = False;

  printf("Starting image capture...\r\n");

  if (HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)frame_buffer, FRAME_BUFFER_SIZE / 4) != HAL_OK)
  {
    printf("✗ Failed to start DCMI DMA\r\n");
    Error_Handler();
  }

  // Wait for frame ready
  while (camImgReady == False)
  {
    HAL_Delay(1);
  }

  printf("✓ Frame captured into RAM\r\n");

  // Optional: Print a small portion of the data


//  for (int i = 0; i < 32; i++) {
//    printf("%02X ", frame_buffer[i]);
//  }

}

static void output_to_SD(void){
	printf("\r\n=== Interfacing with SD Card ===\r\n\r\n");

	HAL_Delay(1000); //a short delay is important to let the SD card settle

	//some variables for FatFs
	FATFS FatFs; 	//Fatfs handle
	FIL fil; 		//File handle
	FRESULT fres; //Result after operations

	//Open the file system
	fres = f_mount(&FatFs, "", 1); //1=mount now
	if (fres != FR_OK) {
	printf("f_mount error (%i)\r\n", fres);
	while(1);
	}

	//Let's get some statistics from the SD card
	DWORD free_clusters, free_sectors, total_sectors;

	FATFS* getFreeFs;

	fres = f_getfree("", &free_clusters, &getFreeFs);
	if (fres != FR_OK) {
		printf("f_getfree error (%i)\r\n", fres);
		while(1);
	}

	//Formula comes from ChaN's documentation
	total_sectors = ((getFreeFs->n_fatent - 2) * getFreeFs->csize) / 2 / 1000000;
	free_sectors = (free_clusters * getFreeFs->csize) / 2 / 1000000;

	printf("SD card stats:\r\n%10lu GB total drive space.\r\n%10lu GB available.\r\n", total_sectors, free_sectors);

	//Now let's try and write a file "write.txt"
	fres = f_open(&fil, "frame.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
	if(fres == FR_OK) {
		printf("I was able to open 'frame.txt' for writing\r\n");
	} else {
		printf("f_open error (%i)\r\n", fres);
	}

	UINT bytesWrote;
	fres = f_write(&fil, frame_buffer, FRAME_BUFFER_SIZE, &bytesWrote);
	if(fres == FR_OK) {
		printf("Wrote %i bytes to 'frame.txt'!\r\n", bytesWrote);
	} else {
		printf("f_write error (%i)\r\n", fres);
	}

	//Be a tidy kiwi - don't forget to close your file!
	fres = f_close(&fil);
	if(fres == FR_OK) {
		printf("I was able to close 'frame.txt'\r\n");
	} else {
		printf("f_close error (%i)\r\n", fres);
	}

	//We're done, so de-mount the drive
	f_mount(NULL, "", 0);
	if (fres != FR_OK) {
		printf("f_mount error in un-mounting (%i)\r\n", fres);
		while(1);
	}
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	printf("HAL_DCMI_FrameEventCallback fired!\r\n");
	camImgReady = True;
	HAL_DCMI_Stop(hdcmi);
}

/**
  * @brief  Initialize Camera I/O structure
  * @param  None
  * @retval None
  */
static void Camera_IO_Init(void)
{
  // Configure the camera I/O structure with function pointers
  io.Init       = OV5640_IO_Init;
  io.DeInit     = OV5640_IO_DeInit;
  io.Address    = OV5640_I2C_ADDRESS;
  io.WriteReg   = OV5640_IO_WriteReg;
  io.ReadReg    = OV5640_IO_ReadReg;
  io.GetTick    = OV5640_IO_GetTick;
  camImgReady = 0;
}

/**
  * @brief  Initialize Camera I2C communication
  * @param  None
  * @retval 0 if OK, -1 if error
  */
static int32_t OV5640_IO_Init(void)
{
  // I2C is already initialized in MX_I2C1_Init()
  // Could add additional initialization here if needed
  return 0;
}

/**
  * @brief  Deinitialize Camera I2C communication
  * @param  None
  * @retval 0 if OK, -1 if error
  */
static int32_t OV5640_IO_DeInit(void)
{
  // Could add deinitialization code here if needed
  return 0;
}

/**
  * @brief  Write camera register via I2C
  * @param  DevAddr: Device I2C address
  * @param  Reg: Register address
  * @param  pData: Pointer to data buffer
  * @param  Length: Data length
  * @retval 0 if OK, -1 if error
  */
static int32_t OV5640_IO_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  HAL_StatusTypeDef status;

  // Use the DevAddr parameter, shifted left by 1 for HAL I2C functions
  status = HAL_I2C_Mem_Write(&hi2c1, (DevAddr << 1), Reg, I2C_MEMADD_SIZE_16BIT, pData, Length, HAL_MAX_DELAY);
  if (status != HAL_OK) {
    return OV5640_ERROR;
  }

  return OV5640_OK;
}

/**
  * @brief  Read camera register via I2C
  * @param  DevAddr: Device I2C address
  * @param  Reg: Register address
  * @param  pData: Pointer to data buffer
  * @param  Length: Data length
  * @retval 0 if OK, -1 if error
  */
static int32_t OV5640_IO_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  HAL_StatusTypeDef status;

  // Use the DevAddr parameter, shifted left by 1 for HAL I2C functions
  status = HAL_I2C_Mem_Read(&hi2c1, (DevAddr << 1), Reg, I2C_MEMADD_SIZE_16BIT, pData, Length, HAL_MAX_DELAY);
  if (status != HAL_OK) {
    return OV5640_ERROR;
  }

  return OV5640_OK;
}

/**
  * @brief  Get system tick for timing operations
  * @param  None
  * @retval Current tick value
  */
static int32_t OV5640_IO_GetTick(void)
{
  return HAL_GetTick();
}

/**
  * @brief  Reset the camera module
  * @param  None
  * @retval None
  */
static void Camera_Reset(void)
{
  // Pull reset pin low for at least 1ms, then high
	printf("Resetting up camera...\r\n");
  HAL_GPIO_WritePin(Reset_GPIO_Port, Reset_Pin, GPIO_PIN_RESET);
  HAL_Delay(2);  // Hold reset for 2ms
  HAL_GPIO_WritePin(Reset_GPIO_Port, Reset_Pin, GPIO_PIN_SET);
  HAL_Delay(60); // Allow camera to boot up
  printf("=============================\r\n\r\n");
}

/**
  * @brief  Power up the camera module
  * @param  None
  * @retval None
  */
static void Camera_PowerUp(void)
{
  // Set power down pin low to power up the camera
	printf("Powering up camera...\r\n");
  HAL_GPIO_WritePin(Shutdown_GPIO_Port, Shutdown_Pin, GPIO_PIN_RESET);
  HAL_Delay(60); // Allow camera to boot up
  printf("=============================\r\n\r\n");

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
