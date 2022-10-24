/*
 * mpu6050.c
 *
 *  Created on: Oct 14, 2022
 *      Author: David7_Yuan
 */

#include "mpu6050.h"

#define RAD_TO_DEG 57.295779513082320876798154814105 //弧度转角度

uint32_t timer;
Kalman_t KalmanX = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f
};

Kalman_t KalmanY = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f,
};

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
	if(HAL_I2C_Mem_Read(mpu6050->i2c_handle,mpu6050->address,MPU6050_ACC_XOUT_HIGH,1,&mpu6050->acc_data[0],6,mpu6050->i2c_timeout)!=HAL_OK)
	{
		return HAL_ERROR;
	}
	mpu6050->acc_x = ((int16_t)(mpu6050->acc_data[0]<<8|mpu6050->acc_data[1]))/16384.0; //2g
	mpu6050->acc_y = ((int16_t)(mpu6050->acc_data[2]<<8|mpu6050->acc_data[3]))/16384.0;
	mpu6050->acc_z = ((int16_t)(mpu6050->acc_data[4]<<8|mpu6050->acc_data[5]))/16384.0;
	//printf("acc_x:%f\r\n",mpu6050->acc_x);
	//printf("acc_y:%f\r\n",mpu6050->acc_y);
	//printf("acc_z:%f\r\n",mpu6050->acc_z);
	return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Read_GYRO(mpu6050_t* mpu6050)
{
	if(HAL_I2C_Mem_Read(mpu6050->i2c_handle,mpu6050->address,MPU6050_GYRO_XOUT_HIGH,1,&mpu6050->gyro_data[0],6,mpu6050->i2c_timeout)!=HAL_OK)
	{
		return HAL_ERROR;
	}
	mpu6050->gyro_x = ((int16_t)(mpu6050->gyro_data[0]<<8|mpu6050->gyro_data[1]))/131.0;
	mpu6050->gyro_y = ((int16_t)(mpu6050->gyro_data[2]<<8|mpu6050->gyro_data[3]))/131.0;
	mpu6050->gyro_z = ((int16_t)(mpu6050->gyro_data[4]<<8|mpu6050->gyro_data[5]))/131.0;
	//printf("gyro_x:%f\r\n",mpu6050->gyro_x);
	//printf("gyro_y:%f\r\n",mpu6050->gyro_y);
	//printf("gyro_z:%f\r\n",mpu6050->gyro_z);
	return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Read_TEMP(mpu6050_t* mpu6050)
{
	int16_t temp;
	float temp_float;
	if(HAL_I2C_Mem_Read(mpu6050->i2c_handle,mpu6050->address,MPU6050_TEMP_OUT_HIGH,1,&mpu6050->temp_data[0],2,mpu6050->i2c_timeout)!=HAL_OK)
	{
		return HAL_ERROR;
	}
	temp = (int16_t)(mpu6050->temp_data[0]<<8|mpu6050->temp_data[1]);
	temp_float = (float)temp/340+36.53;
	printf("temp:%f\r\n",temp_float);
	return HAL_OK;
}

HAL_StatusTypeDef CAL(mpu6050_t* mpu6050)
{
	double roll;
	double pitch;
	double dt = (double) (HAL_GetTick() - timer) / 1000;
	timer = HAL_GetTick();
	MPU6050_Read_ACC(mpu6050);
	MPU6050_Read_GYRO(mpu6050);

	//Roll
	double roll_sqrt = sqrt(mpu6050->acc_x*mpu6050->acc_x+mpu6050->acc_z*mpu6050->acc_z);
	if(roll_sqrt!=0)
	{
		roll = atan(mpu6050->acc_y / roll_sqrt) * RAD_TO_DEG;
	}
	else
	{
		roll = 0.0;
	}

	//Pitch
	pitch = atan2(-mpu6050->acc_x, mpu6050->acc_z) * RAD_TO_DEG;
	if ((pitch < -90 && mpu6050->kal_y > 90) || (pitch > 90 && mpu6050->kal_y < -90))
	{
		KalmanY.angle = pitch;
		mpu6050->kal_y = pitch;
	}
	else
	{
		mpu6050->kal_y = Kalman_Get_Angle(&KalmanY, pitch, mpu6050->gyro_y, dt);
	}
	if (fabs(mpu6050->kal_y) > 90)
		mpu6050->gyro_x = -mpu6050->gyro_x;
	mpu6050->kal_x = Kalman_Get_Angle(&KalmanX,roll,mpu6050->gyro_y,dt);

	//Yaw
	mpu6050->integralZ += mpu6050->gyro_z*dt;
	if (mpu6050->integralZ > 360)
		mpu6050->integralZ -= 360;
	if (mpu6050->integralZ < -360)
		mpu6050->integralZ += 360;
	DMP_Get_Data();
	//printf("integralZ=%f\r\n",mpu6050->integralZ);
	//printf("pitch:%f\r\n",mpu6050->kal_y);
	//printf("roll:%f\r\n",mpu6050->kal_x);
	return HAL_OK;
}

HAL_StatusTypeDef Zero_Offset_Calculates(mpu6050_t* mpu6050)
{
	float gyroZ_offset = 0.0f;
	float dt = 1/1000;
	for(int i = 0 ; i<1000 ; i++)
	{
		MPU6050_Read_GYRO(mpu6050);
		gyroZ_offset += mpu6050->gyro_z * dt;  // 每次1%积分，累计加权1000次
	}

	mpu6050->gyro_z -= gyroZ_offset;//矫正
	return HAL_OK;
}

HAL_StatusTypeDef YAW_Calculates(mpu6050_t* mpu6050, double dt)
{
	//integralX += gyroX * dt;  // 每次8%权重累计偏转角度
	//integralY += gyroY * dt;
	//MPU6050_Read_GYRO(mpu6050);
	mpu6050->integralZ += mpu6050->gyro_z*dt;
	// 360°一个循环
	/*
	if (integralX > 360)
		integralX -= 360;
	if (integralX < -360)
		integralX += 360;
	if (integralY > 360)
		integralY -= 360;
	if (integralY < -360)
		integralY += 360;
	*/

	if (mpu6050->integralZ > 360)
		mpu6050->integralZ -= 360;
	if (mpu6050->integralZ < -360)
		mpu6050->integralZ += 360;
	printf("integralZ=%f\r\n",mpu6050->integralZ);
	return HAL_OK;
}

double Kalman_Get_Angle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
}

HAL_StatusTypeDef DMP_Get_Data(void)
{
	float pitch,roll,yaw;
	if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
	{
		printf("pitch:%f\r\n",pitch);
		printf("roll:%f\r\n",roll);
		printf("yaw:%f\r\n",yaw);
	}
	return HAL_OK;
}
