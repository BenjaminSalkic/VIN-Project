#ifndef __VEML6070_H__
#define __VEML6070_H__

#include "main.h"

typedef enum veml6070_refreshtime {
  VEML6070_05_T,
  VEML6070_1_T,
  VEML6070_2_T,
  VEML6070_4_T,
} veml6070_refreshtime_t;

    void VEML6070_setRefreshTime(veml6070_refreshtime_t Time);
    uint8_t VEML6070_ClearACK(void);
    void VEML6070_Init();
    void VEML6070_Enable(void);
    void VEML6070_Disable(void);
    void VEML6070_setINT_STP(uint8_t intVal, uint8_t stepVal);
    uint16_t VEML6070_getUV(void);
#endif  // __VEML6070_H__
