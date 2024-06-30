/**
  ******************************************************************************
  * @file    BSP/Src/main.c
  * @author  MCD Application Team
  * @brief   This example code shows how to use the STM32H750B_DISCOVERY BSP Drivers
  *          This is the main program.   
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "VEML6070.h"


static void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);
static void MX_GPIO_Init(void);
static void MX_I2C4_Init(void);
void getTempHumi(void);
void getAirQ(void);
void getUV(void);


#define SHT40_ADDRESS (0x44 << 1)


I2C_HandleTypeDef hi2c4;
uint32_t Trials = 3;
uint32_t Timeout = 100;
char SendBuffer1[50];
char SendBuffer2[50];
char SendBuffer3[50];
char SendBuffer4[50];
char SendBuffer5[50];
uint8_t data_tx[1] = {0xFD};
uint8_t data_rx[6];



int main(void)
{
	CPU_CACHE_Enable();
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_I2C4_Init();

	BSP_LCD_Init(0, LCD_ORIENTATION_LANDSCAPE);
	UTIL_LCD_SetFuncDriver(&LCD_Driver);
	UTIL_LCD_Clear(UTIL_LCD_COLOR_WHITE);

	while (1)
	{
		getAirQ();
		getUV();
		getTempHumi();
		Touchscreen_demo(SendBuffer1,SendBuffer2,SendBuffer3,SendBuffer4,SendBuffer5);
	}
}



void getUV(void) {

	#define VEML6070_ADDR_CMD 0x38 <<1
	#define VEML6070_ADDR_DATA_LSB 0x38 << 1
	#define VEML6070_ADDR_DATA_MSB 0x39 << 1
	#define VEML6070_CMD 0x0C

	uint8_t cmd = VEML6070_CMD;
	uint8_t msb;
	uint8_t lsb;
	uint8_t data;

	HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c4, VEML6070_ADDR_DATA_MSB, Trials, Timeout);
	if (ret != HAL_OK){
		snprintf(SendBuffer4, sizeof(SendBuffer4), "Device not ready");
	}else{
		ret = HAL_I2C_Master_Transmit(&hi2c4, VEML6070_ADDR_CMD, (uint8_t*)&cmd, 1, 1000);
		HAL_Delay(300);
	 	if ( ret != HAL_OK ) {
	 		snprintf(SendBuffer4, sizeof(SendBuffer4), "Test Not Sent");
	 	}else{
	 		ret = HAL_I2C_Master_Receive(&hi2c4, VEML6070_ADDR_DATA_MSB,(uint8_t*)&msb, 1, 1000);
	 		if (ret != HAL_OK){
	 			snprintf(SendBuffer4, sizeof(SendBuffer4), "MSB missing");
	 		}else{
	 			ret = HAL_I2C_Master_Receive(&hi2c4, VEML6070_ADDR_DATA_LSB,(uint8_t*)&lsb, 1, 1000);
	 			if (ret != HAL_OK){
	 				snprintf(SendBuffer4, sizeof(SendBuffer4), "LSB missing");
	 			}else{
	 				data = (msb << 8) | lsb;
	 				snprintf(SendBuffer4, sizeof(SendBuffer4), "Amount of Light: %u", data);
	 				if(data < 1120){
	 					snprintf(SendBuffer5, sizeof(SendBuffer5), "UV Index : LOW (0-2)");
	 				}else if(1120 < data < 2241){
	 					snprintf(SendBuffer5, sizeof(SendBuffer5), "UV Index : MODERATE (3-5)");
	 				}else if(2241 < data < 2988){
	 					snprintf(SendBuffer5, sizeof(SendBuffer5), "UV Index : HIGH (6-7)");
	 				}else if(2988 < data < 4108){
	 					snprintf(SendBuffer5, sizeof(SendBuffer5), "UV Index : VERY HIGH (8-10)");
	 				}else if(data > 4108){
	 					snprintf(SendBuffer5, sizeof(SendBuffer5), "UV Index : EXTREME (10+)");
	 				}
	 			}
	 		}
	 	}
	}
}


void getAirQ(void){

	#define SGP40_ADDRESS 0x59 << 1
	 uint8_t data_rx[3];
	 uint8_t test[2] = {0x28,0x0E};
	 uint8_t data_tx[8] = {0x26,0x0F,0x80,0x00,0xA2,0x66,0x66,0x93};

	 HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c4, SGP40_ADDRESS, Trials, Timeout);
	 if (ret != HAL_OK){
		 snprintf(SendBuffer3, sizeof(SendBuffer3), "Device not ready");
	 }else{
		 ret = HAL_I2C_Master_Transmit(&hi2c4, SGP40_ADDRESS, test, 2, 1000);
		 if ( ret != HAL_OK ) {
			 snprintf(SendBuffer3, sizeof(SendBuffer3), "Test Not Sent");
		 }else{
			 HAL_Delay(300);
			 ret = HAL_I2C_Master_Receive(&hi2c4, SGP40_ADDRESS,(uint8_t*)&data_rx, 3, 1000);
			 if (ret != HAL_OK){
				 snprintf(SendBuffer3, sizeof(SendBuffer3), "Test fail");
			 }else{
				 ret = HAL_I2C_Master_Transmit(&hi2c4, SGP40_ADDRESS, data_tx, 8, 1000);
				 if ( ret != HAL_OK ) {
					 snprintf(SendBuffer3, sizeof(SendBuffer3), "Data Not Sent");
				 }else{
					 HAL_Delay(300);
					 ret = HAL_I2C_Master_Receive(&hi2c4, SGP40_ADDRESS,(uint8_t*)&data_rx, 3, 1000);
					 if (ret != HAL_OK){
						 snprintf(SendBuffer3, sizeof(SendBuffer3), "Data not received");
					 }else{
						 snprintf(SendBuffer3, sizeof(SendBuffer3),"Air Quality(from 0 to 500): %u", data_rx[2]);
					 }
				 }
			 }
		 }
	 }
 }

 void getTempHumi(void){

	  HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c4, SHT40_ADDRESS, Trials, Timeout);
	  if (ret != HAL_OK)
	  {
	      snprintf(SendBuffer1, sizeof(SendBuffer1), "Device not ready");

	  }else{
		  	ret = HAL_I2C_Master_Transmit(&hi2c4, SHT40_ADDRESS, data_tx, 1, 1000);
			if ( ret != HAL_OK ) {
				snprintf(SendBuffer1, sizeof(SendBuffer1), "Data not transmitted");
			}else{

				 HAL_Delay(10);
				 ret =  HAL_I2C_Master_Receive(&hi2c4, SHT40_ADDRESS, (uint8_t*)&data_rx, 6,1000);
				 if ( ret != HAL_OK ) {
					 snprintf(SendBuffer1, sizeof(SendBuffer1), "Data not received");
				 }
				 else{

					 float t_ticks = data_rx[0] * 256 + data_rx[1];
					 float rh_ticks = data_rx[3] * 256 + data_rx[4];

					 float t_degC = -45 + 175 * t_ticks/65535;
					 float rh_pRH = -6 + 125 * rh_ticks/65535;

					 snprintf(SendBuffer1, sizeof(SendBuffer1), "Temperature: %f",t_degC);
					 snprintf(SendBuffer2, sizeof(SendBuffer2), "Relative Humidity: %f",rh_pRH);
				 }
			}
	  }
}

static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 | \
                                 RCC_CLOCKTYPE_PCLK2  | RCC_CLOCKTYPE_D3PCLK1);

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
  if(ret != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_RCC_CSI_ENABLE() ;

  HAL_EnableCompensationCell();
}


void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void MX_I2C4_Init(void) {
	hi2c4.Instance = I2C4;
	    hi2c4.Init.Timing = 0x10909CEC;
	    hi2c4.Init.OwnAddress1 = 0;
	    hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	    hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	    hi2c4.Init.OwnAddress2 = 0;
	    hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	    hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	    hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c4) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void)
{
  BSP_LED_On(LED_RED);
  while(1)
  {
  }
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

/**
  * @}
  */

/**
  * @}
  */

