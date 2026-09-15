#ifndef AS5600_H
#define AS5600_H

#include "stm32f4xx_hal.h"




typedef struct {
  I2C_HandleTypeDef *hi2c;
  uint16_t address;
}as5600_t;

void as5600_init(as5600_t * dev,I2C_HandleTypeDef *hi2c );
HAL_StatusTypeDef  as5600_read_raw(as5600_t * dev,uint16_t *raw_angle);
HAL_StatusTypeDef  as5600_read_mechanical_angle_rad( as5600_t *dev,float *mechanical_angle_rad);

#endif // AS5600_H
