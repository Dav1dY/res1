/*
 * mpu6050.h
 *
 *  Created on: Oct 14, 2022
 *      Author: David7_Yuan
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_


#include "stm32h7xx_hal.h"
#include <stdlib.h>
#include <string.h>

#define ADDR_LOW 0x68
#define ADDR_HIGH 0x69 //最后加一位

#define MPU6050_SMPLRT_DIV     0x19
#define MPU6050_CONFIG         0x1A
#define MPU6050_GYRO_CONFIG    0x1B
#define MPU6050_ACCEL_CONFIG   0x1C

#define MPU6050_INT_PIN_CFG	   0x37
#define MPU6050_INT_ENABLE	   0x38
#define MPU6050_INT_STATUS	   0x3A
#define MPU6050_ACC_XOUT_HIGH  0x3B
#define MPU6050_ACC_XOUT_LOW   0x3C
#define MPU6050_ACC_YOUT_HIGH  0x3D
#define MPU6050_ACC_YOUT_LOW   0x3E
#define MPU6050_ACC_ZOUT_HIGH  0x3F
#define MPU6050_ACC_ZOUT_LOW   0x40
#define MPU6050_TEMP_OUT_HIGH  0x41
#define MPU6050_TEMP_OUT_LOW   0x42
#define MPU6050_GYRO_XOUT_HIGH 0x43
#define MPU6050_GYRO_XOUT_LOW  0x44
#define MPU6050_GYRO_YOUT_HIGH 0x45
#define MPU6050_GYRO_YOUT_LOW  0x46
#define MPU6050_GYRO_ZOUT_HIGH 0x47
#define MPU6050_GYRO_ZOUT_LOW  0x48
#define MPU6050_PWR_MGMT_1     0x6B
#define MPU6050_WHO_AM_I       0x75

typedef struct mpu6050{
	I2C_HandleTypeDef* i2c_handle;
	uint8_t address;
	uint8_t acc_data[6];
	uint8_t gyro_data[6];
	uint8_t temp_data[2];
	uint8_t reg_data;
	uint32_t i2c_timeout;
}mpu6050_t;

mpu6050_t* MPU6050_t_INIT(I2C_HandleTypeDef* i2c_handle);
HAL_StatusTypeDef MPU6050_INIT(mpu6050_t* mpu6050);
HAL_StatusTypeDef MPU6050_Read_ACC(mpu6050_t* mpu6050);
HAL_StatusTypeDef MPU6050_Read_GYRO(mpu6050_t* mpu6050);
HAL_StatusTypeDef MPU6050_Read_TEMP(mpu6050_t* mpu6050);

#endif /* INC_MPU6050_H_ */
