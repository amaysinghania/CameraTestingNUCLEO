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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <ov5640.h>
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

DCMI_HandleTypeDef hdcmi;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
OV5640_Object_t cam;
OV5640_IO_t io;
int32_t ret;
uint32_t camera_id;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DCMI_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
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
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Comprehensive OV5640 Debug Test Functions

/**
  * @brief  Scan I2C bus for all connected devices
  * @param  None
  * @retval None
  */
void I2C_Scanner(void)
{
  printf("\r\n=== I2C Bus Scanner ===\r\n");
  printf("Scanning I2C1 bus for devices...\r\n");

  uint8_t devices_found = 0;
  for(uint8_t addr = 1; addr < 128; addr++) {
    if(HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 3, 100) == HAL_OK) {
      printf("✓ Device found at address 0x%02X (7-bit) / 0x%02X (8-bit)\r\n", addr, addr << 1);
      devices_found++;
    }
    HAL_Delay(1); // Small delay between checks
  }

  if(devices_found == 0) {
    printf("✗ No I2C devices found!\r\n");
    printf("  Check connections:\r\n");
    printf("  - PB6 → SCL with 4.7kΩ pull-up\r\n");
    printf("  - PB9 → SDA with 4.7kΩ pull-up\r\n");
    printf("  - Camera power (3.3V)\r\n");
  } else {
    printf("Total devices found: %d\r\n", devices_found);
  }
  printf("========================\r\n\r\n");
}

/**
  * @brief  Test basic I2C communication with OV5640
  * @param  None
  * @retval None
  */
void Test_OV5640_I2C_Communication(void)
{
  printf("=== OV5640 I2C Communication Test ===\r\n");

  // Test if OV5640 is ready
  printf("Testing device ready at 0x3C...\r\n");
  if(HAL_I2C_IsDeviceReady(&hi2c1, 0x3C << 1, 3, 1000) == HAL_OK) {
    printf("✓ OV5640 responds to I2C address 0x3C\r\n");
  } else {
    printf("✗ OV5640 does not respond to I2C address 0x3C\r\n");
    return;
  }

  // Test reading chip ID registers
  uint8_t chip_id_high = 0;
  uint8_t chip_id_low = 0;

  printf("Reading chip ID registers...\r\n");

  // Read high byte (should be 0x56)
  if(HAL_I2C_Mem_Read(&hi2c1, 0x3C << 1, 0x300A, I2C_MEMADD_SIZE_16BIT, &chip_id_high, 1, 1000) == HAL_OK) {
    printf("✓ Chip ID High: 0x%02X\r\n", chip_id_high);
  } else {
    printf("✗ Failed to read Chip ID High register (0x300A)\r\n");
  }

  // Read low byte (should be 0x40)
  if(HAL_I2C_Mem_Read(&hi2c1, 0x3C << 1, 0x300B, I2C_MEMADD_SIZE_16BIT, &chip_id_low, 1, 1000) == HAL_OK) {
    printf("✓ Chip ID Low: 0x%02X\r\n", chip_id_low);
  } else {
    printf("✗ Failed to read Chip ID Low register (0x300B)\r\n");
  }

  // Combine and verify chip ID
  uint16_t chip_id = (chip_id_high << 8) | chip_id_low;
  printf("Combined Chip ID: 0x%04X\r\n", chip_id);

  if(chip_id == 0x5640) {
    printf("✓ OV5640 camera detected successfully!\r\n");
    BSP_LED_On(LED_BLUE);
  } else if(chip_id_high == 0x56) {
    printf("⚠ Partial detection: High byte correct, low byte: 0x%02X (expected 0x40)\r\n", chip_id_low);
  } else {
    printf("✗ Unknown device ID: 0x%04X (expected 0x5640)\r\n", chip_id);
  }

  printf("====================================\r\n\r\n");
}

/**
  * @brief  Test OV5640 register read/write operations
  * @param  None
  * @retval None
  */
void Test_OV5640_Register_Access(void)
{
  printf("=== OV5640 Register Access Test ===\r\n");

  // Test reading various registers
  struct {
    uint16_t reg;
    const char* name;
    uint8_t expected_mask;
  } test_registers[] = {
    {0x300A, "Chip ID High", 0xFF},
    {0x300B, "Chip ID Low", 0xFF},
    {0x302A, "Chip Revision", 0xFF},
    {0x3008, "System Control", 0xFF},
    {0x3034, "PLL Control 0", 0xFF}
  };

  for(int i = 0; i < sizeof(test_registers)/sizeof(test_registers[0]); i++) {
    uint8_t value = 0;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, 0x3C << 1, test_registers[i].reg,
                                                I2C_MEMADD_SIZE_16BIT, &value, 1, 1000);

    if(status == HAL_OK) {
      printf("✓ %s (0x%04X): 0x%02X\r\n", test_registers[i].name, test_registers[i].reg, value);
    } else {
      printf("✗ Failed to read %s (0x%04X)\r\n", test_registers[i].name, test_registers[i].reg);
    }
    HAL_Delay(10);
  }

  // Test write operation (safe register)
  printf("\nTesting register write...\r\n");
  uint8_t original_value = 0;
  uint8_t test_value = 0;

  // Read original value from a safe test register (0x3212 - Group access register)
  if(HAL_I2C_Mem_Read(&hi2c1, 0x3C << 1, 0x3212, I2C_MEMADD_SIZE_16BIT, &original_value, 1, 1000) == HAL_OK) {
    printf("Original value of test register (0x3212): 0x%02X\r\n", original_value);

    // Write a test value
    uint8_t write_test = 0x55;
    if(HAL_I2C_Mem_Write(&hi2c1, 0x3C << 1, 0x3212, I2C_MEMADD_SIZE_16BIT, &write_test, 1, 1000) == HAL_OK) {
      HAL_Delay(1);

      // Read back to verify
      if(HAL_I2C_Mem_Read(&hi2c1, 0x3C << 1, 0x3212, I2C_MEMADD_SIZE_16BIT, &test_value, 1, 1000) == HAL_OK) {
        if(test_value == write_test) {
          printf("✓ Register write/read test successful: wrote 0x%02X, read 0x%02X\r\n", write_test, test_value);
        } else {
          printf("⚠ Register write/read mismatch: wrote 0x%02X, read 0x%02X\r\n", write_test, test_value);
        }
      } else {
        printf("✗ Failed to read back test value\r\n");
      }

      // Restore original value
      HAL_I2C_Mem_Write(&hi2c1, 0x3C << 1, 0x3212, I2C_MEMADD_SIZE_16BIT, &original_value, 1, 1000);
    } else {
      printf("✗ Failed to write test value\r\n");
    }
  } else {
    printf("✗ Failed to read original test register value\r\n");
  }

  printf("==================================\r\n\r\n");
}

/**
  * @brief  Test OV5640 camera initialization sequence
  * @param  None
  * @retval None
  */
void Test_OV5640_Initialization(void)
{
  printf("=== OV5640 Initialization Test ===\r\n");

  // Step 1: Check if camera is in normal power mode
  uint8_t sys_ctrl = 0;
  if(HAL_I2C_Mem_Read(&hi2c1, 0x3C << 1, 0x3008, I2C_MEMADD_SIZE_16BIT, &sys_ctrl, 1, 1000) == HAL_OK) {
    printf("System Control Register (0x3008): 0x%02X\r\n", sys_ctrl);
    if(sys_ctrl & 0x80) {
      printf("⚠ Camera is in software standby mode\r\n");
    } else {
      printf("✓ Camera is in normal operation mode\r\n");
    }
  }

  // Step 2: Test soft reset
  printf("Testing soft reset...\r\n");
  uint8_t reset_cmd = 0x82;
  if(HAL_I2C_Mem_Write(&hi2c1, 0x3C << 1, 0x3008, I2C_MEMADD_SIZE_16BIT, &reset_cmd, 1, 1000) == HAL_OK) {
    printf("✓ Soft reset command sent\r\n");
    HAL_Delay(50); // Wait for reset to complete

    // Check if reset completed
    if(HAL_I2C_Mem_Read(&hi2c1, 0x3C << 1, 0x3008, I2C_MEMADD_SIZE_16BIT, &sys_ctrl, 1, 1000) == HAL_OK) {
      printf("System Control after reset: 0x%02X\r\n", sys_ctrl);
    }
  } else {
    printf("✗ Failed to send soft reset command\r\n");
  }

  // Step 3: Wake up from standby
  printf("Waking up camera from standby...\r\n");
  uint8_t wakeup_cmd = 0x02;
  if(HAL_I2C_Mem_Write(&hi2c1, 0x3C << 1, 0x3008, I2C_MEMADD_SIZE_16BIT, &wakeup_cmd, 1, 1000) == HAL_OK) {
    printf("✓ Wake up command sent\r\n");
    HAL_Delay(50);

    // Verify camera is awake
    if(HAL_I2C_Mem_Read(&hi2c1, 0x3C << 1, 0x3008, I2C_MEMADD_SIZE_16BIT, &sys_ctrl, 1, 1000) == HAL_OK) {
      printf("System Control after wake up: 0x%02X\r\n", sys_ctrl);
      if((sys_ctrl & 0x80) == 0) {
        printf("✓ Camera successfully woken up\r\n");
      } else {
        printf("⚠ Camera may still be in standby\r\n");
      }
    }
  } else {
    printf("✗ Failed to send wake up command\r\n");
  }

  printf("================================\r\n\r\n");
}

/**
  * @brief  Test OV5640 using the BSP driver functions
  * @param  None
  * @retval None
  */
void Test_OV5640_BSP_Driver(void)
{
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
    printf("✓ OV5640_Init successful (VGA, RGB565)\r\n");
    printf("✓ Camera is ready for image capture!\r\n");
    BSP_LED_On(LED_GREEN);
  } else {
    printf("✗ OV5640_Init failed (ret=%ld)\r\n", ret);
  }

  printf("=============================\r\n\r\n");
}

/**
  * @brief  Run complete OV5640 diagnostic test suite
  * @param  None
  * @retval None
  */
void Run_OV5640_Diagnostic_Suite(void)
{
  printf("\r\n");
  printf("╔════════════════════════════════════════╗\r\n");
  printf("║        OV5640 DIAGNOSTIC SUITE         ║\r\n");
  printf("║    Based on STM32 Official Driver      ║\r\n");
  printf("╚════════════════════════════════════════╝\r\n");
  printf("\r\n");

  // Turn off all LEDs initially
  BSP_LED_Off(LED_GREEN);
  BSP_LED_Off(LED_BLUE);
  BSP_LED_Off(LED_RED);

  // Step 1: Hardware setup
  printf("Step 1: Hardware Setup\r\n");
  printf("Power up camera...\r\n");
  Camera_PowerUp();
  HAL_Delay(10);

  printf("Reset camera...\r\n");
  Camera_Reset();
  HAL_Delay(50);

  printf("Hardware setup complete.\r\n\r\n");

  // Step 2: I2C bus scan
  I2C_Scanner();
  HAL_Delay(100);

  // Step 3: Basic I2C communication
  Test_OV5640_I2C_Communication();
  HAL_Delay(100);

  // Step 4: Register access tests
  Test_OV5640_Register_Access();
  HAL_Delay(100);

  // Step 5: Initialization sequence
  Test_OV5640_Initialization();
  HAL_Delay(100);

  // Step 6: BSP driver tests
  Test_OV5640_BSP_Driver();

  printf("╔════════════════════════════════════════╗\r\n");
  printf("║          DIAGNOSTIC COMPLETE           ║\r\n");
  printf("╚════════════════════════════════════════╝\r\n");
  printf("\r\n");

  // Final status indication
  if(BSP_LED_GetState(LED_GREEN) && BSP_LED_GetState(LED_BLUE)) {
    printf("🎉 SUCCESS: Camera fully operational!\r\n");
  } else if(BSP_LED_GetState(LED_BLUE)) {
    printf("⚠  PARTIAL: Camera detected but initialization issues\r\n");
  } else {
    printf("❌ FAILED: Camera communication issues\r\n");
    BSP_LED_On(LED_RED);
  }
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DCMI_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
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

  // Run comprehensive OV5640 diagnostic suite
  Run_OV5640_Diagnostic_Suite();

  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// Main loop - camera is now initialized and ready for capture
	HAL_Delay(1000);
//	HAL_UART_Transmit(&huart2, (uint8_t) "OV5640 Camera Test Starting...\r\n", sizeof((uint8_t)"OV5640 Camera Test Starting...\r\n"), HAL_MAX_DELAY);
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
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_LOW;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Shutdown_Pin Reset_Pin */
  GPIO_InitStruct.Pin = Shutdown_Pin|Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /*Configure Camera Reset Pin */
  GPIO_InitStruct.Pin = CAM_RESET_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAM_RESET_PORT, &GPIO_InitStruct);

  /*Configure Camera Power Down Pin */
  GPIO_InitStruct.Pin = CAM_PWDN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAM_PWDN_PORT, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
  HAL_GPIO_WritePin(Reset_GPIO_Port, Reset_Pin, GPIO_PIN_RESET);
  HAL_Delay(2);  // Hold reset for 2ms
  HAL_GPIO_WritePin(Reset_GPIO_Port, Reset_Pin, GPIO_PIN_SET);
  HAL_Delay(50); // Allow camera to boot up
}

/**
  * @brief  Power up the camera module
  * @param  None
  * @retval None
  */
static void Camera_PowerUp(void)
{
  // Set power down pin low to power up the camera
  HAL_GPIO_WritePin(Shutdown_GPIO_Port, Shutdown_Pin, GPIO_PIN_RESET);
  HAL_Delay(50); // Allow camera to boot up
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
//int _write(int file, char *ptr, int len)
//{
//  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
//  return len;
//}

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
