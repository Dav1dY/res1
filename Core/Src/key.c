/*
 * key->c
 *
 *  Created on: Jul 15, 2022
 *      Author: David7_Yuan
 */
#include "key.h"

key_t* key_init(uint8_t number,GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin, osSemaphoreId_t Semaphore_fall, osSemaphoreId_t Semaphore_rise, osSemaphoreId_t Semaphore_pressed, osEventFlagsId_t event_group, IRQn_Type IRQn_fall)
{
  key_t* button = (key_t*)malloc(sizeof(key_t));
  button->number=number;
  button->GPIOx=GPIOx;
  button->GPIO_Pin=GPIO_Pin;
  button->Semaphore_fall=Semaphore_fall;
  button->Semaphore_rise=Semaphore_rise;
  button->Semaphore_pressed=Semaphore_pressed;
  button->key_event=event_group;
  button->IRQn_fall=IRQn_fall; //EXTI3_IRQn
  button->state=Key_Release;
  button->last_state=Key_Release;
  return button;
}

void key_monitor(key_t* key_id)
{
  if(osSemaphoreAcquire(key_id->Semaphore_fall,osWaitForever)==osOK)
  {
    //HAL_NVIC_DisableIRQ(key_id->IRQn_fall);
    osDelay(shake_time);
    if(HAL_GPIO_ReadPin(key_id->GPIOx,key_id->GPIO_Pin)==0){
      //osEventFlagsSet(key_id->key_event,Key_Pressing);
      //HAL_GPIO_TogglePin(GPIOI,GPIO_PIN_12); //for testing
      osSemaphoreRelease(key_id->Semaphore_pressed);  //original design
    }
    //HAL_NVIC_EnableIRQ(key_id->IRQn_fall);
  }
}

void key_state_machine(key_t* key_id)
{
/*
  if(osSemaphoreAcquire(key_id->Semaphore_pressed, osWaitForever)==osOK){
	  if(osSemaphoreAcquire(key_id->Semaphore_rise, long_time)==osOK){
		  if(osSemaphoreAcquire(key_id->Semaphore_pressed,wait_time)==osOK){
			  HAL_GPIO_TogglePin(GPIOI,GPIO_PIN_13); //for testing
		  }
	  }
  }
*/

  switch(key_id->state){
    case Key_Release:
      //osEventFlagsWait(key_id->key_event,Key_Pressing,osFlagsWaitAny,osWaitForever);
      if(osSemaphoreAcquire(key_id->Semaphore_pressed, osWaitForever)==osOK){
        key_id->last_state = key_id->state;
        key_id->state = Key_Pressing;
        break;
      }
    case Key_Pressing:
      switch(key_id->last_state){
        //空闲转到单击
        case Key_Release:
          key_id->last_state = key_id->state;
          key_id->state = Key_Pressed_Once;
          break;
        //空闲转到双击
        case Key_Pressed_Once:
          key_id->last_state = key_id->state;
          key_id->state = Key_Pressed_Twice;
          break;
        default:
          break;
      break;
      }
    case Key_Pressed_Once:
      if(osSemaphoreAcquire(key_id->Semaphore_rise, long_time)==osOK){//直到1000ms等待按键上升
        //1000ms内按键上升时
        if(osSemaphoreAcquire(key_id->Semaphore_pressed,wait_time)==osOK){//370ms内等待第二次按下
        //370ms第二次按下 判断为双击
          key_id->last_state = key_id->state;
          key_id->state = Key_Pressing;
          break;
        }
        else{//370ms内没有第二次按下 判断为单击
      	  HAL_GPIO_TogglePin(GPIOI,GPIO_PIN_12); //for testing
          osEventFlagsSet(key_id->key_event,Key_Pressed_Once);
          //osSemaphoreRelease(key_id->Semaphore_1);
          //if(osSemaphoreAcquire(key_id->Semaphore_rise,osWaitForever)==osOK){
          HAL_NVIC_DisableIRQ(key_id->IRQn_fall);
          key_id->last_state = key_id->state;
          key_id->state = Key_Release;
          osDelay(disable_time);
    	  HAL_NVIC_EnableIRQ(key_id->IRQn_fall);
    	  break;
        }
      }
      else{
      //1000ms内没有上升 判定为长按
        key_id->last_state = key_id->state;
        key_id->state = Key_Pressed_Long;
        break;
      }
    case Key_Pressed_Twice:
	  HAL_GPIO_TogglePin(GPIOI,GPIO_PIN_13); //for testing
      osEventFlagsSet(key_id->key_event,Key_Pressed_Twice);
      //osSemaphoreRelease(key_id->Semaphore_2);
      if(osSemaphoreAcquire(key_id->Semaphore_rise,osWaitForever)==osOK){
      //if(HAL_GPIO_ReadPin(key_id->GPIOx,key_id->GPIO_Pin)){
        HAL_NVIC_DisableIRQ(key_id->IRQn_fall);
  	  	key_id->last_state = key_id->state;
  	  	key_id->state = Key_Release;
  	    osDelay(disable_time);
  	  	HAL_NVIC_EnableIRQ(key_id->IRQn_fall);
  	  }
  	  break;
  	//确认长按
    case Key_Pressed_Long:
	  HAL_GPIO_TogglePin(GPIOI,GPIO_PIN_14); //for testing
      osEventFlagsSet(key_id->key_event,Key_Pressed_Long);
      //osSemaphoreRelease(key_id->Semaphore_longpress);
      if(osSemaphoreAcquire(key_id->Semaphore_rise,osWaitForever)==osOK){
      //if(HAL_GPIO_ReadPin(key_id->GPIOx,key_id->GPIO_Pin)){
        HAL_NVIC_DisableIRQ(key_id->IRQn_fall);
  	  	osDelay(disable_time);
        key_id->last_state = key_id->state;
        key_id->state = Key_Release;
  	  	HAL_NVIC_EnableIRQ(key_id->IRQn_fall);
  	  }
  	  break;
    default:
      break;
  }

}
