#include "as5600.h"

#define AS5600_I2C_ADDRESS    (0x36 << 1)
#define AS5600_ANGLE_REGISTER 0x0E



void as5600_init(as5600_t * dev,I2C_HandleTypeDef *hi2c )
{
    dev->address=AS5600_I2C_ADDRESS;
    dev->hi2c=hi2c;
}

HAL_StatusTypeDef  as5600_read_raw(as5600_t * dev,uint16_t *raw_angle)
{

    HAL_StatusTypeDef status;
	uint8_t angle_data[2] = {0};
	status = HAL_I2C_Mem_Read(dev->hi2c, dev->address, AS5600_ANGLE_REGISTER, I2C_MEMADD_SIZE_8BIT, angle_data, 2,10);
    if (status != HAL_OK)
    {
        return status;
    }
	*raw_angle = ((uint16_t)angle_data[0] << 8 | angle_data[1]) & 0x0fff;
	return HAL_OK;
}
HAL_StatusTypeDef as5600_read_mechanical_angle_rad(as5600_t *dev,float *mechanical_angle_rad)
{
    uint16_t raw_angle;
    HAL_StatusTypeDef status;
    status = as5600_read_raw(dev,&raw_angle);
    if (status != HAL_OK)
    {
        return status;
    }
    *mechanical_angle_rad  = (float)raw_angle * 2.0f * 3.14159265358979323846f / 4096.0f;
    
    return HAL_OK;
}

