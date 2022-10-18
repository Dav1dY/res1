/*
 * mpu6050.c
 *
 *  Created on: Oct 14, 2022
 *      Author: David7_Yuan
 */

#include "mpu6050.h"

mpu6050_t* MPU6050_t_INIT(I2C_HandleTypeDef* i2c_handle)
{
	mpu6050_t* mpu6050 = (mpu6050_t*)malloc(sizeof(mpu6050_t));
	mpu6050->i2c_handle = i2c_handle;
	mpu6050->address = ADDR_LOW<<1;;
	mpu6050->reg_data = 0x00;
	mpu6050->i2c_timeout = 1000;
	return mpu6050;
}

HAL_StatusTypeDef MPU6050_INIT(mpu6050_t* mpu6050)
{
	//check who_am_i
	if(HAL_I2C_Mem_Read(mpu6050->i2c_handle, mpu6050->address, MPU6050_WHO_AM_I, 1, &mpu6050->reg_data, 1, mpu6050->i2c_timeout)!=HAL_OK)
	{
		return HAL_ERROR;
	}
	if(mpu6050->reg_data!=ADDR_LOW)
	{
		return HAL_ERROR;
	}

	//wake up
	mpu6050->reg_data = 0x00;
	if(HAL_I2C_Mem_Write(mpu6050->i2c_handle, mpu6050->address, MPU6050_PWR_MGMT_1, 1, &mpu6050->reg_data, 1, mpu6050->i2c_timeout)!=HAL_OK)
	{
		return HAL_ERROR;
	}

	//set data rate
	mpu6050->reg_data = 0x07;
	if(HAL_I2C_Mem_Write(mpu6050->i2c_handle, mpu6050->address, MPU6050_SMPLRT_DIV, 1, &mpu6050->reg_data, 1, mpu6050->i2c_timeout)!=HAL_OK)  //原始8k 目前8/（1+7）=1khz
	{
		return HAL_ERROR;
	}

	//set gyro config
	mpu6050->reg_data = 0x00;
	if(HAL_I2C_Mem_Write(mpu6050->i2c_handle, mpu6050->address, MPU6050_GYRO_CONFIG, 1, &mpu6050->reg_data, 1, mpu6050->i2c_timeout)!=HAL_OK)
	{
		return HAL_ERROR;
	}

	//set accelerometer config
	mpu6050->reg_data = 0x00;
	if(HAL_I2C_Mem_Write(mpu6050->i2c_handle, mpu6050->address, MPU6050_ACCEL_CONFIG, 1, &mpu6050->reg_data, 1, mpu6050->i2c_timeout)!=HAL_OK)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Read_ACC(mpu6050_t* mpu6050)
{
	if(HAL_I2C_Mem_Read(mpu6050->i2c_handle,mpu6050->address,MPU6050_ACC_XOUT_HIGH,sizeof(MPU6050_ACC_XOUT_HIGH),&mpu6050->acc_data[0],sizeof(mpu6050->acc_data),mpu6050->i2c_timeout)!=HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Read_GYRO(mpu6050_t* mpu6050)
{
	if(HAL_I2C_Mem_Read(mpu6050->i2c_handle,mpu6050->address,MPU6050_GYRO_XOUT_HIGH,sizeof(MPU6050_GYRO_XOUT_HIGH),&mpu6050->gyro_data[0],sizeof(mpu6050->gyro_data),mpu6050->i2c_timeout)!=HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}
HAL_StatusTypeDef MPU6050_Read_TEMP(mpu6050_t* mpu6050)
{
	if(HAL_I2C_Mem_Read(mpu6050->i2c_handle,mpu6050->address,MPU6050_TEMP_OUT_HIGH,sizeof(MPU6050_TEMP_OUT_HIGH),&mpu6050->temp_data[0],sizeof(mpu6050->temp_data),mpu6050->i2c_timeout)!=HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}
