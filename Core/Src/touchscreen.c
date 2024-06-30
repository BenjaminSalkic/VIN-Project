/**
  ******************************************************************************
  * @file    BSP/Src/touchscreen.c
  * @author  MCD Application Team
  * @brief   This example code shows how to use the touchscreen driver.
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

/** @addtogroup STM32H7xx_HAL_Examples
  * @{
  */

/** @addtogroup BSP
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define  CIRCLE_RADIUS        25
#define  LINE_LENGHT          30
/* Private macro -------------------------------------------------------------*/
#define  CIRCLE_XPOS(i)       ((i * 480) / 5)
#define  CIRCLE_YPOS(i)       (302 - CIRCLE_RADIUS - 60)
#define  CIRCLE_YPOS1(i)       (235 - CIRCLE_RADIUS - 60)

/* Private Structures and Enumerations ------------------------------------------------------------*/
/* Global variables ---------------------------------------------------------*/
TS_State_t  TS_State;

/* Private variables ---------------------------------------------------------*/
/* Static variable holding the current touch color index : used to change color at each touch */
TS_Init_t hTS;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Touchscreen Demo1 : test touchscreen calibration and single touch in polling mode
  * @param  None
  * @retval None
  */
void Touchscreen_demo(char SendBuffer1[50],char SendBuffer2[50],char SendBuffer3[50],char SendBuffer4[50],char SendBuffer5[50])
{
  uint16_t x1, y1;
  uint8_t state = 0;
  uint32_t ts_status = BSP_ERROR_NONE;
  uint32_t x_size, y_size;

  BSP_LCD_GetXSize(0, &x_size);
  BSP_LCD_GetYSize(0, &y_size);

  hTS.Width = x_size;
  hTS.Height = y_size;
  hTS.Orientation =TS_SWAP_XY ;
  hTS.Accuracy = 5;

  ts_status = BSP_TS_Init(0, &hTS);

  if(ts_status == BSP_ERROR_NONE)
  {
	uint32_t x_size, y_size;

	BSP_LCD_GetXSize(0, &x_size);
	BSP_LCD_GetYSize(0, &y_size);

	UTIL_LCD_FillRect(0, 0, x_size, 80, UTIL_LCD_COLOR_BLUE);
	UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_WHITE);
	UTIL_LCD_SetBackColor(UTIL_LCD_COLOR_BLUE);
	UTIL_LCD_SetFont(&Font24);

	UTIL_LCD_DisplayStringAt(0, 15, (uint8_t *)"Healthy Home", CENTER_MODE);
	UTIL_LCD_SetFont(&Font12);
	UTIL_LCD_DisplayStringAt(0, 45, (uint8_t *)"Please use the screen to monitor your home.", CENTER_MODE);

	UTIL_LCD_DrawRect(10, 90, x_size - 20, y_size - 100, UTIL_LCD_COLOR_BLUE);
	UTIL_LCD_DrawRect(11, 91, x_size - 22, y_size - 102, UTIL_LCD_COLOR_BLUE);

    UTIL_LCD_DisplayStringAt(30, 105, (uint8_t *)SendBuffer1, LEFT_MODE);
    UTIL_LCD_DisplayStringAt(30, 120, (uint8_t *)SendBuffer2, LEFT_MODE);
    UTIL_LCD_DisplayStringAt(30, 135, (uint8_t *)SendBuffer3, LEFT_MODE);
    UTIL_LCD_DisplayStringAt(30, 150, (uint8_t *)SendBuffer4, LEFT_MODE);
    UTIL_LCD_DisplayStringAt(30, 165, (uint8_t *)SendBuffer5, LEFT_MODE);

    UTIL_LCD_DisplayStringAt(315, 105, "Change temperature:", LEFT_MODE);
    UTIL_LCD_FillCircle(CIRCLE_XPOS(4), CIRCLE_YPOS(1), CIRCLE_RADIUS, UTIL_LCD_COLOR_BLUE);
    UTIL_LCD_FillCircle(CIRCLE_XPOS(4), CIRCLE_YPOS1(4), CIRCLE_RADIUS, UTIL_LCD_COLOR_BLUE);
    UTIL_LCD_DisplayStringAt(382, 144, "+", LEFT_MODE);
    UTIL_LCD_DisplayStringAt(382, 211, "-", LEFT_MODE);
  }
}

