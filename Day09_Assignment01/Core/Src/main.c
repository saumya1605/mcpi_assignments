/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SCL_PIN		6
#define SDA_PIN		7

void I2C_Init(void);
void I2C_Start(void);
void I2C_RepeatStart(void);
void I2C_Stop(void);
void I2C_SendSlaveAddr(uint8_t slaveAddr);
void I2C_SendData(uint8_t data);
uint16_t I2C_RecvData(void);
uint16_t I2C_RecvDataNAck(void);
int I2C_IsDeviceReady(uint8_t slaveAddr);

#define LCD_SLAVE_ADDR_W	0x4E
#define LCD_SLAVE_ADDR_R	0x4F

#define LCD_CLEAR		0x01
#define LCD_FN_SET_8BIT	0x30
#define LCD_FN_SET_4BIT	0x20
#define LCD_FN_SET_4BIT_1LINES	0x20
#define LCD_DISP_CTRL	0x08
#define LCD_DISP_ON		0x0C
#define LCD_ENTRY_MODE	0x06
#define LCD_LINE1		0x80
#define LCD_LINE2		0xC0
#define LCD_SHIFT		0x18

#define LCD_RS	0
#define LCD_RW	1
#define LCD_EN	2
#define LCD_BL	3

#define LCD_CMD		0x80
#define LCD_DATA	1


int Lcd_Init(void);
void Lcd_Write4BitAndCtrl(uint8_t val);
void Lcd_WriteByte(uint8_t rs, uint8_t val);
void Lcd_Puts(uint8_t line, char str[]);
void Lcd_Shift(void);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void I2C_Init(void) {
	// PB6, PB7 -- GPIO Init -- MODER, AFRL, PUPDR, CLKEN
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	GPIOB->MODER |= (BV(SCL_PIN*2+1) | BV(SDA_PIN*2+1));
	GPIOB->MODER &= ~(BV(SCL_PIN*2) | BV(SDA_PIN*2));
    GPIOB->PUPDR &= ~(BV(SCL_PIN*2+1) | BV(SDA_PIN*2+1) | BV(SCL_PIN*2) | BV(SDA_PIN*2)); // no pull-up/down
    // choose AF4 for I2C1 in Alternate Function registers
    GPIOB->AFR[0] |= BV(30) | BV(26);
    GPIOB->AFR[0] &= ~(BV(31) | BV(29) | BV(28) | BV(27) | BV(25) | BV(24));

	// I2C1 Init -- CR1, CR2, CCR, TRISE, CLKEN
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    // reset and clear reg
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;
    // set pclk in CR2 (16 MHz)
    I2C1->CR2 |= 16;
    // set i2c bitrate to 100KHz (std mode)
    I2C1->CCR &= ~I2C_CCR_FS; // change mode to standard mode
    I2C1->CCR |= 80;			// CCR = Ton / Tpclk = 5 / 0.0625 = 80
    // standard mode Max Trise = 1000 ns
    // set Trise = Max Trise / Tpclk = 1000 / 62.5 = 17
    I2C1->TRISE |= 17;
    // Enable Ack
    I2C1->CR1 |= I2C_CR1_ACK;
    // Enable I2C
    I2C1->CR1 |= I2C_CR1_PE;
}

void I2C_Start(void) {
	// CR1 -- send start bit
    I2C1->CR1 |= I2C_CR1_START;
	// SR1 -- poll for SB (start bit sent)
    while(!(I2C1->SR1 & I2C_SR1_SB));
}
void I2C_RepeatStart(void) {
	I2C_Start();
}
void I2C_Stop(void) {
	// CR1 -- send stop bit
    I2C1->CR1 |= I2C_CR1_STOP;
    // SR2 -- poll for bus to be release
    while(!(I2C1->SR2 & I2C_SR2_BUSY));
}
void I2C_SendSlaveAddr(uint8_t slaveAddr) {
	// DR -- write slave addr in data regr
    I2C1->DR = slaveAddr;
	// SR1 -- poll for addr is transferred
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    // dummy read to clear flags
    (void)I2C1->SR1;
    (void)I2C1->SR2;
}
void I2C_SendData(uint8_t data) {
	// SR1 -- wait while previous data is not transmitted
	while (!(I2C1->SR1 & I2C_SR1_TXE));
	// DR -- write data in data regr
    I2C1->DR = data;
	// SR1 -- poll for BTF is transferred
    while (!(I2C1->SR1 & I2C_SR1_BTF));
}
uint16_t I2C_RecvData(void) {
	I2C1->CR1 |= I2C_CR1_ACK | I2C_CR1_POS;
	// wait until receive buffer is not empty
	while (!(I2C1->SR1 & I2C_SR1_RXNE));
	// read content and clear flags
	uint16_t val = I2C1->DR;
	return val;
}

uint16_t I2C_RecvDataNAck(void) {
	I2C1->CR1 &= ~(I2C_CR1_ACK | I2C_CR1_POS);
	// wait until receive buffer is not empty
	while (!(I2C1->SR1 & I2C_SR1_RXNE));
	// read content and clear flags
	uint16_t val = I2C1->DR;
	return val;
}

int I2C_IsDeviceReady(uint8_t slaveAddr) {
	// DR -- write slave addr in data regr
    I2C1->DR = slaveAddr;
	// SR1 -- poll for addr is transferred
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    // dummy read to clear flags
    (void)I2C1->SR1;
    (void)I2C1->SR2;
    return 1;
}

void Lcd_Write4BitAndCtrl(uint8_t val) {
	I2C_Start();
	I2C_SendSlaveAddr(LCD_SLAVE_ADDR_W);
	I2C_SendData(val);
	I2C_Stop();
}

void Lcd_WriteByte(uint8_t rs, uint8_t val) {
	uint8_t high = val & 0xF0, low = (val << 4) & 0xF0;
	uint8_t bvrs = (rs == LCD_CMD) ? 0 : BV(LCD_RS);
	Lcd_Write4BitAndCtrl(high | bvrs | BV(LCD_EN) | BV(LCD_BL));
	DelayMs(1);
	Lcd_Write4BitAndCtrl(high | bvrs | BV(LCD_BL));

	Lcd_Write4BitAndCtrl(low | bvrs | BV(LCD_EN) | BV(LCD_BL));
	DelayMs(1);
	Lcd_Write4BitAndCtrl(low | bvrs | BV(LCD_BL));
}

// As per 4-bit initialization sequence mentioned HD44780 datasheet fig 24 (page 46)
int Lcd_Init() {
	// wait for min 15 ms (for 5V)
	DelayMs(20);
	I2C_Init();
	// check if lcd is ready
	I2C_Start();
	int ret = I2C_IsDeviceReady(LCD_SLAVE_ADDR_W);
	I2C_Stop();
	if(!ret)
		return 0;

	// attention sequence
	Lcd_Write4BitAndCtrl(LCD_FN_SET_8BIT | BV(LCD_EN));
	__NOP();
	Lcd_Write4BitAndCtrl(LCD_FN_SET_8BIT);
	DelayMs(5);

	Lcd_Write4BitAndCtrl(LCD_FN_SET_8BIT | BV(LCD_EN));
	__NOP();
	Lcd_Write4BitAndCtrl(LCD_FN_SET_8BIT);
	DelayMs(1);

	Lcd_Write4BitAndCtrl(LCD_FN_SET_8BIT | BV(LCD_EN));
	__NOP();
	Lcd_Write4BitAndCtrl(LCD_FN_SET_8BIT);
	DelayMs(3);

	Lcd_Write4BitAndCtrl(LCD_FN_SET_4BIT | BV(LCD_EN));
	__NOP();
	Lcd_Write4BitAndCtrl(LCD_FN_SET_4BIT);
	DelayMs(3);

	// lcd initialization
	Lcd_Write4BitAndCtrl(LCD_FN_SET_4BIT_1LINES);
	DelayMs(1);
	Lcd_WriteByte(LCD_CMD, LCD_DISP_CTRL);
	DelayMs(1);
	Lcd_WriteByte(LCD_CMD, LCD_CLEAR);
	DelayMs(1);
	Lcd_WriteByte(LCD_CMD, LCD_ENTRY_MODE);
	DelayMs(1);
	Lcd_WriteByte(LCD_CMD, LCD_DISP_ON);
	DelayMs(1);

	return ret;
}

void Lcd_Puts(uint8_t line, char str[]) {
	int i;
	Lcd_WriteByte(LCD_CMD, line); // line address
	DelayMs(1);
	for(i=0; str[i]!='\0'; i++)
		Lcd_WriteByte(LCD_DATA, str[i]);
}

void Lcd_Shift(void)
{
	Lcd_WriteByte(LCD_CMD, LCD_SHIFT);
	DelayMs(1);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	int ret;
		char str[32];
		SystemInit();
		ret = Lcd_Init();
		if(ret) {
			Lcd_Puts(LCD_LINE1, "Manish Chandrakant Zine");
			for(int i=0; i<=22; i++){
				Lcd_Shift();
				DelayMs(500);
			}

		}
		return 0;
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
