/*
 * bh.h
 *
 *  Created on: 2022年7月19日
 *      Author: David7_Yuan
 */
#include "stm32h7xx_hal.h"
#include <stdlib.h>
#include <string.h>
#include "cmsis_os.h"

#ifndef INC_BH_H_
#define INC_BH_H_

//i2c address
#define ADDR_H 0x5c
#define ADDR_L 0x23
#define BH_SEND 0x00
#define BH_READ  0x01

//command code
typedef enum{
POWER_DOWN=0x00U,
POWER_ON=0x01U,
BH_RESET=0x07U,
CONTINUOUS_HRES=0x10U,
CONTINUOUS_HRES2=0x11U,
CONTINUOUS_LRES=0x13U,
ONE_TIME_HRES=0x20U,
ONE_TIME_HRES2=0x21U,
ONE_TIME_LRES=0x23U
}cmd_t;

typedef struct bh1750{
  uint8_t sensor_number;
  I2C_HandleTypeDef* i2c_handle;
  uint8_t send_address;
  uint8_t read_address;
  uint8_t buffer[2];
  uint16_t illuminance;
  HAL_StatusTypeDef i2c_state;
}bh1750_t;

bh1750_t* bh1750_init(uint8_t ADDR_PIN,I2C_HandleTypeDef* i2c_handle);
HAL_StatusTypeDef bh1750_startup(bh1750_t* sensor);
HAL_StatusTypeDef  bh1750_Send(bh1750_t* sensor,uint8_t cmd);
HAL_StatusTypeDef  bh1750_Read(bh1750_t* sensor);
HAL_StatusTypeDef b1750_ReadFromBuffer(bh1750_t* sensor);
HAL_StatusTypeDef send_illuminance(bh1750_t* sensor,UART_HandleTypeDef* huart);

#endif /* INC_BH_H_ */
