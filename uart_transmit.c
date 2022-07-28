/*
 * uart_transmit.c
 *
 *  Created on: Jul 20, 2022
 *      Author: David7_Yuan
 */

#include "uart_transmit.h"

uint8_t uart_monitor(UART_HandleTypeDef* uart_handle,uint8_t flag, bh1750_t* sensor1){
    if(flag==0){
    	uint8_t buffer[5]={0};
    	HAL_UART_Receive(uart_handle,(uint8_t*)&buffer,5,0xFFFFFFFF);
    	if((buffer[0]==0x73)&&(buffer[1]==0x74)&&(buffer[2]==0x61)&&(buffer[3]==0x72)&&(buffer[4]==0x74)){
    		HAL_UART_Transmit(uart_handle,(uint8_t*)&buffer,5,100);
            return 1;
    	}
    }
    else{
    	HAL_GPIO_TogglePin(GPIOI,GPIO_PIN_12);
    	sensor1->i2c_state=bh1750_Read(sensor1);
        osDelay(10);
        sensor1->i2c_state=b1750_ReadFromBuffer(sensor1);
        sensor1->i2c_state=send_illuminance(sensor1,uart_handle);
        return 1;
    }
    return 0;
}

//HAL_StatusTypeDef HAL_UART_Receive (UART_HandleTypeDef * huart, uint8_t * pData, uint16_t Size,uint32_t Timeout)
