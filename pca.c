/*
 * pca.c
 *
 *  Created on: Jul 5, 2022
 *      Author: David7_Yuan
 */
#include "pca.h"

void pca6416_WritePin(uint8_t io,uint8_t buf)
{
  uint8_t DevAddr = PCA_ADDR|PCA_WRITE;
  uint8_t cmd[2];
  if(io==0){
    //cmd = {PCA_OUTPUT0,buf};
    cmd[0] = PCA_OUTPUT0;
    cmd[1] = buf;
  }
  else{
	//cmd = {PCA_OUTPUT1,buf};
    cmd[0] = PCA_OUTPUT1;
    cmd[1] = buf;
  }
  HAL_I2C_Master_Transmit(&hi2c1, DevAddr, cmd, 2, HAL_MAX_DELAY);
}

void pca6416_ReadPin(uint8_t io,uint8_t buf)
{
  uint8_t DevAddr1 = PCA_ADDR|PCA_WRITE;
  uint8_t DevAddr2 = PCA_ADDR|PCA_READ;
  //uint8_t dataBuffer[2];
  uint8_t cmd;
  if(io==0)
  {
    cmd = PCA_INPUT0;
  }
  else if(io==1)
  {
	cmd = PCA_INPUT1;
  }
  else
  {
	return PCA_ERROR;
  }
  HAL_I2C_Master_Transmit(&hi2c1, DevAddr1, &cmd, 1, HAL_MAX_DELAY);

  HAL_I2C_Master_Receive(&hi2c1,DevAddr2, &buf, 1, HAL_MAX_DELAY);
}

