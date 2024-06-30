#include "VEML6070.h"

#define VEML6070_ADDR_H     (0x39 << 1)
#define VEML6070_ADDR_L     (0x38 << 1)
#define VEML6070_ADDR_ARA   0x18 // Alert Resp Address (read to clear condition)
#define VEML6070_CMD_REG    0x02 // ACK = 0; THD = 0; IT1 = 0; IT0 = 0; BIT2 = 1

#define ENABLE_BIT          0X01
#define INT_ENABLE_BIT      0X20
#define THRES_BIT           0X10

#define VEML6070_RESPONCE_TIME   500

uint8_t cmd_reg = VEML6070_CMD_REG;
I2C_HandleTypeDef hi2c2;

static void VEML6070_setCMD(void) {
    HAL_I2C_Master_Transmit(&hi2c2, VEML6070_ADDR_L, (uint8_t*)(&cmd_reg), 1, VEML6070_RESPONCE_TIME);
}

static uint8_t VEML6070_getVal(uint8_t regAddr) {
    uint8_t data;
    HAL_I2C_Master_Receive(&hi2c2, regAddr, &data, 1, VEML6070_RESPONCE_TIME);
    return data;
}

void VEML6070_setRefreshTime(veml6070_refreshtime_t Time) {
    cmd_reg |= (Time << 2);
    VEML6070_setCMD();
}

uint8_t VEML6070_ClearACK(void) {
    return VEML6070_getVal(VEML6070_ADDR_ARA);
}

void VEML6070_Init(void) {
    //Default start
    //BIT 0, 1 are reserved
    //ACK = 0 = no interrupt
    //THD = 1 = 145 steps
    //IT1 = 1; IT0 = 1 = Refresh time 4T
    //BIT2 = 1 = reserved
    // SD = 0 = shutdown disabled
    //printf("Init->cmd_reg %#02x\r\n", cmd_reg);
    VEML6070_ClearACK();
    //printf("Init->cmd_reg %#02x\r\n", cmd_reg);
  VEML6070_setCMD();
}

void VEML6070_Enable(void) {
    //printf("Enable->cmd_reg %#02x\r\n", cmd_reg);
    cmd_reg &= ~ENABLE_BIT;
    //printf("Enable->cmd_reg %#02x\r\n", cmd_reg);
    VEML6070_setCMD();
}

void VEML6070_Disable(void) {
    cmd_reg |= ENABLE_BIT;
    VEML6070_setCMD();
}

void VEML6070_setINT_STP(uint8_t intVal, uint8_t stepVal) {
    VEML6070_ClearACK();
    //printf("setINT->cmd_reg %#02x\r\n", cmd_reg);
    if(intVal)
        cmd_reg |= INT_ENABLE_BIT;
    else
        cmd_reg &= ~INT_ENABLE_BIT;
    //printf("setINT<-cmd_reg %#02x\r\n", cmd_reg);
    if(stepVal)
        cmd_reg |= THRES_BIT;
    else
        cmd_reg &= ~THRES_BIT;
    //printf("setSteps<-cmd_reg %#02x\r\n", cmd_reg);
    VEML6070_setCMD();
}

uint16_t VEML6070_getUV(void) {
    uint16_t result = 0;
    result = (VEML6070_getVal(VEML6070_ADDR_H) << 8);
    result |= VEML6070_getVal(VEML6070_ADDR_L);
    return result;
}
