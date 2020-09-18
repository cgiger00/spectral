/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "smbus.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CHANNELS 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

const int DEV_SEL = 0x4F;

const int I2C_AS72XX_SLAVE_STATUS_REG = 0x00;
const int I2C_AS72XX_SLAVE_WRITE_REG = 0x01;
const int I2C_AS72XX_SLAVE_READ_REG = 0x02;
const int I2C_AS72XX_SLAVE_TX_VALID = 0x02;
const int I2C_AS72XX_SLAVE_RX_VALID = 0x01;

const int DEVICE_SLAVE_ADDRESS_READ = 0x93;
const int DEVICE_SLAVE_ADDRESS_WRITE = 0x92;
const int DEVICE_SLAVE_ADDRESS = 0x49;

// registers for triad
const int RAW_VALUE_RGA_HIGH = 0x08;
const int RAW_VALUE_RGA_LOW = 0x09;

const int RAW_VALUE_SHB_HIGH = 0x0A;
const int RAW_VALUE_SHB_LOW = 0x0B;

const int RAW_VALUE_TIC_HIGH = 0x0C;
const int RAW_VALUE_TIC_LOW = 0x0D;

const int RAW_VALUE_UJD_HIGH = 0x0E;
const int RAW_VALUE_UJD_LOW = 0x0F;

const int RAW_VALUE_VKE_HIGH = 0x10;
const int RAW_VALUE_VKE_LOW = 0x11;

const int RAW_VALUE_WLF_HIGH = 0x12;
const int RAW_VALUE_WLF_LOW = 0x13;

static HAL_StatusTypeDef ret;
uint8_t buf[30];

Bus *i2cBus;


typedef struct {
	uint8_t lsb_register;
	uint8_t msb_register;
	uint16_t color_data;
} Channel;

typedef struct {
	uint8_t dev_register;
	Channel *channels[CHANNELS];
} Device;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

int check_error();

void nucleo_byte_write(uint8_t addr, uint8_t data);
void virtual_write(uint8_t v_reg, uint8_t data);

uint8_t nucleo_byte_read(uint8_t device_reg);
uint8_t virtual_read(uint8_t v_reg);





/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


int check_error() {
	if (ret != HAL_OK) {
		strcpy((char*)buf, "Err \r\n");

    HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
    HAL_Delay(10);
		return 0;
	}
	return 1;
}

uint8_t nucleo_byte_read(uint8_t device_reg) {
	//transmits the address to read from
	buf[0] = device_reg;
	ret = HAL_I2C_Master_Transmit(&hi2c1, DEVICE_SLAVE_ADDRESS << 1, buf, 1, HAL_MAX_DELAY);
	check_error();

	//reads from address sent above
	ret = HAL_I2C_Master_Receive(&hi2c1, (DEVICE_SLAVE_ADDRESS << 1) | 1, buf, 1, HAL_MAX_DELAY);
	check_error();
	return buf[0];
}

void nucleo_byte_write(uint8_t addr, uint8_t data) {
	buf[0] = addr;
	buf[1] = data;

	//SMBUS docs first byte is addr to write to, second is data
	ret = HAL_I2C_Master_Transmit(&hi2c1, DEVICE_SLAVE_ADDRESS << 1, buf, 2, HAL_MAX_DELAY);
	check_error();
}

uint8_t virtual_read(uint8_t v_reg) {
	uint8_t status;
	uint8_t d;

	// status = nucleo_byte_read(I2C_AS72XX_SLAVE_STATUS_REG);
	status = read_byte_data(i2cBus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);

	if ((status & I2C_AS72XX_SLAVE_RX_VALID) != 0) {
		// d = nucleo_byte_read(I2C_AS72XX_SLAVE_READ_REG);
		d = read_byte_data(i2cBus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_READ_REG);
	}

	while(1) {
		// status = nucleo_byte_read(I2C_AS72XX_SLAVE_STATUS_REG);
		status = read_byte_data(i2cBus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);

		if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0) {
			break;
		}
		HAL_Delay(5); //delay for 5 ms
	}

	// nucleo_byte_write(I2C_AS72XX_SLAVE_WRITE_REG, v_reg);
	write_byte_data(i2cBus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_WRITE_REG, v_reg);

	while(1) {
		// status = nucleo_byte_read(I2C_AS72XX_SLAVE_STATUS_REG);
		status = read_byte_data(i2cBus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);

		if ((status & I2C_AS72XX_SLAVE_RX_VALID) != 0) {
			break;
		}
		HAL_Delay(5); //delay for 5 ms
	}

	// d = nucleo_byte_read( I2C_AS72XX_SLAVE_READ_REG);

	d = read_byte_data(i2cBus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_READ_REG);

	return d;
}


void virtual_write(uint8_t v_reg, uint8_t data) {
	uint8_t status;

	while(1) {
		// status = nucleo_byte_read(I2C_AS72XX_SLAVE_STATUS_REG);
		status = read_byte_data(i2cBus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);
		if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0) {
			break;
		}
		HAL_Delay(5);
	}

	// nucleo_byte_write(I2C_AS72XX_SLAVE_WRITE_REG, (v_reg | 1 << 7));
	write_byte_data(i2cBus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_WRITE_REG, (v_reg | 1 << 7));

	while(1) {
		status = nucleo_byte_read(I2C_AS72XX_SLAVE_STATUS_REG);
		if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0) {
			break;
		}
		HAL_Delay(5);
	}

	// nucleo_byte_write(I2C_AS72XX_SLAVE_WRITE_REG, data);
	write_byte_data(i2cBus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_WRITE_REG, data);
}

uint16_t get_decimal(uint8_t virtual_reg_l, uint8_t virtual_reg_h) {
	uint16_t high = virtual_read(virtual_reg_h) << 8;
	return high | (virtual_read(virtual_reg_l) & 0xFF);
}

Channel* new_channel(uint8_t lsb_r, uint8_t msb_r) {
	Channel* ch = malloc(sizeof(Channel));
	ch->color_data = 0;
	ch->lsb_register = lsb_r;
	ch->msb_register = msb_r;
	return ch;
}

//not the most readable code..
//dev channels start at 0x08 and increase by 8 up until 0x13

Device* new_device(uint8_t dev_register) {
	Device* dev = malloc(sizeof(Device));
	dev->dev_register = dev_register;

	uint8_t START_REG = 0x08; //RAW_VALUE_RGA_LOW;

	for (uint8_t i = 0; i < CHANNELS; ++i) {
		dev->channels[i] = new_channel(START_REG + (2 * i), START_REG + (2 * i) + 1);
	}
	return dev;
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

  	i2cBus = new_bus(&hi2c1, &huart2);

	uint8_t buf[30];

	Device *triad_dev_1 = new_device(0x00);

	Device *triad_dev_2 = new_device(0x01);

	Device *triad_dev_3 = new_device(0x02);

	Device *triad[3] = { triad_dev_1, triad_dev_2, triad_dev_3 };
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //enable the spectral sensor

  virtual_write(0x04, 0x28);
  virtual_write(0x04, 0x28);
  virtual_write(0x05, 0xFF);

  while (1)
  {
	  // strcpy((char*)buf, "Hello!\r\n");
	  // HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
	  // HAL_Delay(500);

	  Channel *channel = new_channel(0, 0);

	  for (uint8_t i = 0; i < 3; ++i) {
		  virtual_write(DEV_SEL, triad[i]->dev_register);
		  for (uint8_t j = 0; j < CHANNELS; ++j) {
			  channel = triad[i]->channels[j];
			  channel->color_data = get_decimal(channel->lsb_register, channel->msb_register);

			  //complicated way to print "channel {x} : {data}"
			  sprintf((char*)buf , "channel %u : %f \r\n", (unsigned int)((i*CHANNELS) + j), (float)channel->color_data);

			  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
			  HAL_Delay(10);
		  }
	  }
	  HAL_Delay(1000);


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x2000090E;
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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
