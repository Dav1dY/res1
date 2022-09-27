/*
 * pca.h
 *
 *  Created on: Jul 5, 2022
 *      Author: David7_Yuan
 */
#include "stm32h7xx.h"
#include <string.h>
#include <stdio.h>

#ifndef INC_PCA_H_
#define INC_PCA_H_

#endif /* INC_PCA_H_ */

extern I2C_HandleTypeDef hi2c1;

//address
#define PCA_ADDR 0x40
#define PCA_WRITE 0x00
#define PCA_READ 0x01

//command
#define PCA_INPUT0 0x00
#define PCA_INPUT1 0x01
#define PCA_OUTPUT0 0x02
#define PCA_OUTPUT1 0x03
#define PCA_POLAR0 0x04
#define PCA_POLAR1 0x05
#define PCA_CONFIG0 0x06
#define PCA_CONFIG1 0x07

//GPIO
#define PCA_PORT  GPIOB
#define PCA_SDA GPIO_PIN_7
#define PCA_SCL GPIO_PIN_8

#define PCA_ERROR 0x00
#define PCA_OK 0x01
