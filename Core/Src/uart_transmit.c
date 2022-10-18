/*
 * uart_transmit.c
 *
 *  Created on: Jul 20, 2022
 *      Author: David7_Yuan
 */

#include "uart_transmit.h"

HAL_StatusTypeDef uart_monitor(UART_HandleTypeDef* uart_handle, bh1750_t* sensor1)
{
	HAL_GPIO_TogglePin(GPIOI,GPIO_PIN_12);
	sensor1->i2c_state=bh1750_Read(sensor1);
    osDelay(10);
    sensor1->i2c_state=b1750_ReadFromBuffer(sensor1);
    sensor1->i2c_state=send_illuminance(sensor1,uart_handle);
    HAL_Delay(1000);
    return HAL_OK;
}

uint8_t command_monitor(UART_HandleTypeDef* uart_handle, ring_buffer_t ring_buffer, uint8_t start_flag)
{
	for(int i=0;i<10;i++)
	{
		buffer[i]=0;
	}
	uint8_t size = ring_buffer.head_index-ring_buffer.tail_index;
	ring_buffer_dequeue_arr(&ring_buffer,(char *)&buffer[0],size);
	if((buffer[0]==0x73)&&(buffer[1]==0x74)&&(buffer[2]==0x61)&&(buffer[3]==0x72)&&(buffer[4]==0x74)&&buffer[5]==0x0A)
	{
		HAL_UART_Transmit_IT(uart_handle,(uint8_t*)&buffer[0],size);
		return 1;
	}
	else if((buffer[0]==0x73)&&(buffer[1]==0x74)&&(buffer[2]==0x6F)&&(buffer[3]==0x70)&&(buffer[4]==0x0A))
	{
		HAL_UART_Transmit_IT(uart_handle,(uint8_t*)&buffer[0],size);
		return 0;
	}
	return start_flag;
}
