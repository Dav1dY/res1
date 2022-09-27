/*
 * key.h
 *
 *  Created on: Jul 15, 2022
 *      Author: David7_Yuan
 */

#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include "cmsis_os.h"

//down=pk3 left=pk4 right=pk5 up=pk6

#ifndef INC_KEY_H_
#define INC_KEY_H_

typedef enum{
Key_Release=0x00001U,
Key_Pressing=0x00010U,
Key_Pressed_Once=0x00100U,
Key_Pressed_Twice=0x01000U,
Key_Pressed_Long=0x10000U,
Key_Released=0x100000U
}key_state_t;


typedef struct key{
  uint8_t number;
  GPIO_TypeDef* GPIOx;
  uint16_t GPIO_Pin;
  osSemaphoreId_t Semaphore_fall; //falling edge isr 按下
  osSemaphoreId_t Semaphore_rise; //rising edge isr 松开
  osSemaphoreId_t Semaphore_pressed; //press confirmed
  osEventFlagsId_t key_event; //0:pressed 1:once 2:twice 3:long
  IRQn_Type IRQn_fall;
  key_state_t state;
  key_state_t last_state;
  //放弃使用信号量
  //osSemaphoreId_t Semaphore_1; //pressed once
  //osSemaphoreId_t Semaphore_2; //pressed twice
  //osSemaphoreId_t Semaphore_longpress; //pressed long
}key_t;

#define shake_time 30  //消抖时间
#define wait_time 970  //等待第二次电机
#define disable_time 300  //disable key
#define long_time 4970 //长按

key_t* key_init(uint8_t number,GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin, osSemaphoreId_t Semaphore_fall, osSemaphoreId_t Semaphore_rise, osSemaphoreId_t Semaphore_pressed, osEventFlagsId_t event_group, IRQn_Type IRQn_fall);
//key_t key_init(uint8_t number,GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin, osSemaphoreId_t Semaphore_fall, osSemaphoreId_t Semaphore_pressed, osEventFlagsId_t event_group, IRQn_Type IRQn_fall);
void key_monitor(key_t* key_id);
//void key_monitor(key_t key_id);
void key_state_machine(key_t* key_id);

#endif /* INC_KEY_H_ */


