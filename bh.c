/*
 * bh.c
 *
 *  Created on: 2022年7月19日
 *      Author: David7_Yuan
 */

#include "bh.h"

bh1750_t* bh1750_init(uint8_t ADDR_PIN,I2C_HandleTypeDef* i2c_handle)
{
/*
  uint8_t send_add;
  uint8_t read_add;
  if(ADDR_PIN){
    send_add = ADDR_H<<1;
    read_add = ADDR_H<<1;
  }
  else{
	send_add = ADDR_L<<1;
	read_add = ADDR_L<<1;
  }
  */
  bh1750_t* sensor = (bh1750_t*)malloc(sizeof(bh1750_t));
  sensor->sensor_number = 0;
  sensor->i2c_handle = i2c_handle;
  sensor->send_address = 0x46;
  sensor->read_address = 0x47;
  sensor->buffer[0]=0x00;
  sensor->buffer[1]=0x00;
  sensor->illuminance=0x00;
  return sensor;
}

HAL_StatusTypeDef bh1750_startup(bh1750_t* sensor){

  if(bh1750_Send(sensor,POWER_ON)!=HAL_OK){
    return HAL_ERROR;
  };
  //if(bh1750_Send(sensor,BH_RESET)!=HAL_OK){
    //return HAL_ERROR;
  //};
  if(bh1750_Send(sensor,CONTINUOUS_HRES)!=HAL_OK){
    return HAL_ERROR;
  };

	//bh1750_Send(sensor,POWER_ON);
	//bh1750_Send(sensor,BH_RESET);
	//bh1750_Send(sensor,CONTINUOUS_HRES);
    return HAL_OK;
}

HAL_StatusTypeDef bh1750_Send(bh1750_t* sensor,uint8_t cmd){
  if(HAL_I2C_Master_Transmit(sensor->i2c_handle, sensor->send_address, &cmd, 1, 10)!=HAL_OK){
	return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef bh1750_Read(bh1750_t* sensor)
{
  sensor->buffer[0]=0;
  sensor->buffer[1]=0;
  //uint8_t tmp[2];
  if(HAL_I2C_Master_Receive(sensor->i2c_handle, sensor->read_address, sensor->buffer, 2, 10)!=HAL_OK){
    return HAL_ERROR;
  }
  //sensor->buffer[0]=tmp[0];
  //sensor->buffer[1]=tmp[1];
  return HAL_OK;
}

HAL_StatusTypeDef b1750_ReadFromBuffer(bh1750_t* sensor){
  sensor->illuminance=0;
  sensor->illuminance=sensor->buffer[0];
  sensor->illuminance=((sensor->illuminance<<8)|sensor->buffer[1]);
  //sensor->illuminance=sensor->illuminance/1.2;
  return HAL_OK;
}

HAL_StatusTypeDef send_illuminance(bh1750_t* sensor,UART_HandleTypeDef* huart){
  uint8_t data[6]={0};
  float tmp = sensor->illuminance/1.2;
  uint16_t lum = (uint16_t)tmp;
  int lenth = 1;
  for(int i=0;i<5;i++){
    data[i]=lum/(pow(10,(4-i)));
    lum=lum-data[i]*(pow(10,(4-i)));
    data[i]=data[i]+48;
  }
  for(int i=0;i<5;i++){
    if(data[i]!=48){
      lenth = 5-i;
      break;
    }
  }
  data[5]=0x0A;
  if((HAL_UART_Transmit(huart,&data[5-lenth],lenth+1,10))!=HAL_OK){
    return HAL_ERROR;
  }
  //HAL_Delay(1000);
/*
  if((HAL_UART_Transmit(huart,&sensor->buffer[1],1,10))!=HAL_OK){
    return HAL_ERROR;
  }
  if((HAL_UART_Transmit(huart,&sensor->buffer[0],1,10))!=HAL_OK){
    return HAL_ERROR;
  }
  uint8_t enter = 0x0A;
  if((HAL_UART_Transmit(huart,&enter,1,10))!=HAL_OK){
    return HAL_ERROR;
  }
*/
/*
  if((HAL_UART_Transmit(huart,&sensor->buffer[1],1,10))!=HAL_OK){
    return HAL_ERROR;
  }
  HAL_Delay(500);
  if((HAL_UART_Transmit(huart,&sensor->buffer[0],1,10))!=HAL_OK){
    return HAL_ERROR;
  }
*/
  return HAL_OK;
}
