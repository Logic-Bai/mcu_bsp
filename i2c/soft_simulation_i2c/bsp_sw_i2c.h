#ifndef __BSP_SW_I2C_H
#define __BSP_SW_I2C_H

#include <stdint.h>

typedef enum i2c_dev {
    RGB_LED = 0,
    RGB_LED_EXT,
    I2C_DEV_NUM_MAX
} I2C_DEV_NAME;

int8_t i2c_write_reg(uint8_t reg_addr, uint8_t *data_buff, uint32_t data_len);
void i2c_dev_init(I2C_DEV_NAME i2c_dev_name);
void i2c_sel_dev(I2C_DEV_NAME i2c_addr);

#endif

