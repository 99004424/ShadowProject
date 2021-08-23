/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "crc.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#define   MAX_CONFIG_PARAM    5
#define   MAX_CONFIG_BYTES	  (MAX_CONFIG_PARAM*4)
#define   EEPROM_ADDR         0xA0

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t I2C_Transmit_DefaultParameters(void);
uint8_t I2C_Transmit_NewParameters(uint8_t bytearray[],uint8_t arraysize,uint8_t start_addr);
void readByte(uint8_t byteNumber,uint8_t *byteValue);
void EEPROM_ErasePage1();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
///////DEFAULT PARAMETER VALUES//////
uint32_t default_config_parameters[MAX_CONFIG_PARAM]={40,200,150,40,150};
uint8_t default_cdefault_config_parameters_bytearray[MAX_CONFIG_BYTES];
/////DATA BUFFERS///////
uint8_t uart_config_param_data[4];
uint8_t uart_config_param_buf[100]={'*'};


///////FLAGS//////
uint8_t uart_param_flag=0;
uint8_t UART_TX_FLAG=0;
uint8_t READY_TO_WRITE=0;

//Rx Transfer completed callback.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    //check and be ready to Rx next frame or not
    //already rx one full frame
    if(uart_param_flag==1){
    	//check for frame integrity
        if(uart_config_param_data[0]=='(' && uart_config_param_buf[uart_config_param_data[1]*4]==')'){
        	UART_TX_FLAG=1;
        	//call UART Transmit
        	HAL_UART_TxCpltCallback(&huart2);
        	UART_TX_FLAG=0;
        }
        //frame not of correct format
        else{
        	UART_TX_FLAG=2;
        	HAL_UART_TxCpltCallback(&huart2);
        	UART_TX_FLAG=0;
        }
        HAL_UART_Receive_IT(&huart2, uart_config_param_data, 4);
        uart_param_flag=0;
    }
    //Rx Remaining frame
    else{
      int no_of_param=uart_config_param_data[1];
      HAL_UART_Receive_IT(&huart2,uart_config_param_buf,(no_of_param*4)+1);
      uart_param_flag=1;
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	//Check for flag if frame is valid
	if(UART_TX_FLAG==1){
			 HAL_UART_Transmit_IT(&huart2,(uint8_t* )"Data Received",sizeof("Data Received"));
			 READY_TO_WRITE=1;
	}
	else if(UART_TX_FLAG==2){
	HAL_UART_Transmit_IT(&huart2,(uint8_t* )"Invalid Frame",sizeof("Invalid Frame"));
	}
	//HAL_Delay(5);
}
uint8_t I2C_Transmit_DefaultParameters(void){
  //calculate CRC for float array
  //CRC functions defined in crc.c and declared in crc.h
  crcInit();
  crc config_crc=crcFast((uint8_t*)&default_config_parameters,MAX_CONFIG_BYTES);
  HAL_StatusTypeDef i2cRetVal;
  uint8_t senddatabuf[32]={0};
  uint8_t crcbuf[2]={0};
  memcpy(crcbuf, (uint8_t*)&config_crc, MAX_CONFIG_BYTES);
  memcpy(default_cdefault_config_parameters_bytearray, (uint8_t*)&default_config_parameters, MAX_CONFIG_BYTES);
  senddatabuf[0]=0xAA;
  for(int i=1;i<MAX_CONFIG_BYTES;i++){
	  senddatabuf[i]=default_cdefault_config_parameters_bytearray[i-1];
  }
  for(int i=0;i<2;i++){
 	  senddatabuf[MAX_CONFIG_BYTES+1+i]=crcbuf[i];
   }
  i2cRetVal=HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDR, (1<<5), 2, senddatabuf,32, 1000);
  HAL_Delay(5);
  if(i2cRetVal==HAL_OK){
	  return 1;
  }
  else
	  return 0;
}
uint8_t I2C_Transmit_NewParameters(uint8_t bytearray[],uint8_t arraysize,uint8_t start_address){
	HAL_StatusTypeDef i2cRetVal;
	uint8_t i2C_new_write=0;
	if(start_address>1){
		start_address=(start_address*4)-1;
	}
	  i2cRetVal=HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDR, (1<<5)|start_address|1, 2, bytearray, arraysize, 1000);
	  HAL_Delay(5);
	  if(i2cRetVal==HAL_OK){
		  i2C_new_write|=1<<0;
	  }
	  uint8_t tempbuf[MAX_CONFIG_BYTES];
	  HAL_I2C_Mem_Read(&hi2c1, 0xA0, (1<<5)|1, 2, tempbuf,MAX_CONFIG_BYTES, 1000);
	  //recalculate CRC based on new parameters written
	  crcInit();
	  crc new_config_crc=crcFast((uint8_t*)&tempbuf,MAX_CONFIG_BYTES);
	  //Write updated CRC of data to EEPROM
	  i2cRetVal=HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDR, (1<<5)|1|MAX_CONFIG_BYTES, 2, (uint8_t*)&new_config_crc,2, 1000);
	  HAL_Delay(5);
	  if(i2cRetVal==HAL_OK){
		  i2C_new_write|=1<<1;
	  }
	  //check if all the writes to EEPROM were successful
	  if(i2C_new_write==3){
	  	  return 1;
	  }
	  else
	  	  return 0;
}
void EEPROM_ErasePage1(){
	uint8_t data[32];
	memset(data,0xFF,32);
	HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDR, (1<<5),2,data,32, 1000);
	HAL_Delay(5);
}
void readByte(uint8_t byteNumber,uint8_t *byteValue){
	uint8_t pagenum=(byteNumber/32 +1);
	uint8_t bytenum=byteNumber%32;
	HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDR, (pagenum<<5)|bytenum, 2, byteValue, 1, 1000);
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
  HAL_Delay(5);

  /* USER CODE BEGIN Init */

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

  //EEPROM_ErasePage1();
  //Read first memory location to check if default parameters already written
  uint8_t writeflag=0;
  readByte(0,&writeflag);
  if(writeflag!=0xAA){
	  //Write Default parameters
	  uint8_t I2C_Returnflag=I2C_Transmit_DefaultParameters();
	  if(I2C_Returnflag==1){
		  HAL_UART_Transmit(&huart2,(uint8_t* )"Default Write Successful",sizeof("Default Write Successful"),1000);
	  }
	  else if(I2C_Returnflag==0){
		  HAL_UART_Transmit(&huart2,(uint8_t* )"Default Write Failed",sizeof("Default Write Failed"),1000);
	  }
  }

  //Receive config param data when UART interrupt gets triggered
  HAL_UART_Receive_IT(&huart2, uart_config_param_data, 4);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
	  if(READY_TO_WRITE==1){
		  uint8_t I2C_Returnflag=I2C_Transmit_NewParameters(uart_config_param_buf,uart_config_param_data[1]*4,uart_config_param_data[3]);
		  uint8_t buffert[32];
		    HAL_I2C_Mem_Read(&hi2c1, 0xA0, (1<<5), 2, buffert, 32, 1000);
		  	  if(I2C_Returnflag==1){
		  		  HAL_UART_Transmit(&huart2,(uint8_t* )"Change Successful",sizeof("Change Successful"),1000);
		  	  }
		  	  else if(I2C_Returnflag==0){
		  		  HAL_UART_Transmit(&huart2,(uint8_t* )"Change Failed",sizeof("Change Failed"),1000);
		  	  }
		  READY_TO_WRITE=0;
	  }
  }
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  /** Initializes the CPU, AHB and APB buses clocks
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
