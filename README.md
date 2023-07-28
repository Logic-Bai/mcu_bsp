MCU BSP
=====

This repository is used to maintain the generic mcu_bsp I created.

## I2C

### soft_simulation_i2c

It can flexibly add i2c devices without occupying addtional RAM, and it overhead for switching i2c device is O(1).

After calling and swithing i2c devices, there will be no additional overhead when using i2c operation function.

it's a very good library to manage multi-i2c device.

* It easy to add and remove a i2c device, `good for programmer coding`.
* The cost of RAM is the same as just have a I2C device, `good for save RAM`.
* The cost of time to operate i2c bus is the same as just have a I2C device, `good for saving time`.