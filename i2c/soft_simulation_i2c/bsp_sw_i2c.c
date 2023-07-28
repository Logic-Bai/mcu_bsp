#include "bsp_sw_i2c.h"
#include "drv_usb_hw.h"
#include "hal_system_gpio.h"

#define IIC_DBG                 printf

struct i2c_pin_config {
    rcu_periph_enum rcu_periph;
    uint32_t        gpio_periph;
    uint32_t        pin;
};

struct i2c_bus_io_config {
    struct i2c_pin_config   i2c_data_cfg;
    struct i2c_pin_config   i2c_sclk_cfg;
};

struct i2c_config {
    const struct i2c_bus_io_config*		i2c_bus_io_config;
    uint8_t								current_i2c_addr;
};

const static struct i2c_bus_io_config i2c_bus_cfg[] = {
    // LED_RGB & LED_RGB_EXT
    { {RCU_GPIOE, GPIOE, SDA_LED_Pin}, {RCU_GPIOE, GPIOE, SCL_LED_Pin} }
};

const static uint8_t i2c_bus_io_cfg_prt_table[I2C_DEV_NUM_MAX] = {
    0,
    0
};

const static uint8_t i2c_addr_table[I2C_DEV_NUM_MAX] = {
    0x68,
    0x6a
};

static struct i2c_config global_i2c_config;

void i2c_sel_dev(I2C_DEV_NAME i2c_dev_name) {
    global_i2c_config.current_i2c_addr  = i2c_addr_table[i2c_dev_name];
    global_i2c_config.i2c_bus_io_config = i2c_bus_cfg + i2c_bus_io_cfg_prt_table[i2c_dev_name];
}

void i2c_modify_addr(uint8_t i2c_addr) {
    global_i2c_config.current_i2c_addr = i2c_addr;
}

void i2c_dev_init(I2C_DEV_NAME i2c_dev_name) {
    i2c_sel_dev(i2c_dev_name);

    rcu_periph_clock_enable(global_i2c_config.i2c_bus_io_config->i2c_data_cfg.rcu_periph);
    gpio_mode_set(global_i2c_config.i2c_bus_io_config->i2c_data_cfg.gpio_periph,
                  GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  global_i2c_config.i2c_bus_io_config->i2c_data_cfg.pin);
    gpio_output_options_set(global_i2c_config.i2c_bus_io_config->i2c_data_cfg.gpio_periph,
                            GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,
                            global_i2c_config.i2c_bus_io_config->i2c_data_cfg.pin);
    gpio_bit_write(global_i2c_config.i2c_bus_io_config->i2c_data_cfg.gpio_periph,
                        global_i2c_config.i2c_bus_io_config->i2c_data_cfg.pin, SET);

    rcu_periph_clock_enable(global_i2c_config.i2c_bus_io_config->i2c_sclk_cfg.rcu_periph);
    gpio_mode_set(global_i2c_config.i2c_bus_io_config->i2c_sclk_cfg.gpio_periph,
                  GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  global_i2c_config.i2c_bus_io_config->i2c_sclk_cfg.pin);
    gpio_output_options_set(global_i2c_config.i2c_bus_io_config->i2c_sclk_cfg.gpio_periph,
                            GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,
                            global_i2c_config.i2c_bus_io_config->i2c_sclk_cfg.pin);
    gpio_bit_write(global_i2c_config.i2c_bus_io_config->i2c_sclk_cfg.gpio_periph,
                        global_i2c_config.i2c_bus_io_config->i2c_sclk_cfg.pin, SET);
}

static void i2c_data_set(bit_status bit_value) {
    gpio_bit_write(global_i2c_config.i2c_bus_io_config->i2c_data_cfg.gpio_periph, global_i2c_config.i2c_bus_io_config->i2c_data_cfg.pin, bit_value);
}

static FlagStatus i2c_data_get(void) {
    return gpio_input_bit_get(global_i2c_config.i2c_bus_io_config->i2c_data_cfg.gpio_periph, global_i2c_config.i2c_bus_io_config->i2c_data_cfg.pin);
}

static void i2c_sclk_set(bit_status bit_value) {
    gpio_bit_write(global_i2c_config.i2c_bus_io_config->i2c_sclk_cfg.gpio_periph, global_i2c_config.i2c_bus_io_config->i2c_sclk_cfg.pin, bit_value);
}

static void i2c_data_dir_set(uint32_t mode) {
	if (mode == GPIO_MODE_OUTPUT)
	{
		gpio_mode_set(global_i2c_config.i2c_bus_io_config->i2c_data_cfg.gpio_periph, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, global_i2c_config.i2c_bus_io_config->i2c_data_cfg.pin);
		gpio_output_options_set(global_i2c_config.i2c_bus_io_config->i2c_data_cfg.gpio_periph , GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, global_i2c_config.i2c_bus_io_config->i2c_data_cfg.pin);
	}
	else if(mode == GPIO_MODE_INPUT)
	{
		gpio_mode_set(global_i2c_config.i2c_bus_io_config->i2c_data_cfg.gpio_periph , GPIO_MODE_INPUT, GPIO_PUPD_NONE, global_i2c_config.i2c_bus_io_config->i2c_data_cfg.pin);
	}
}

static void i2c_start(void) {
    i2c_sclk_set(SET);
    i2c_data_set(SET);
    usb_udelay(5);
    i2c_data_set(RESET);
    usb_udelay(5);
    i2c_sclk_set(RESET);
}

static void i2c_send_byte(uint8_t byteout) {
    for (int i = 0; i < 8; i++) {
        // MSB
        if (byteout & 0x80) {
            i2c_data_set(SET);
        } else {
            i2c_data_set(RESET);
        }
        usb_udelay(5);

        i2c_sclk_set(SET);
        byteout = byteout << 1;
        usb_udelay(5);
        i2c_sclk_set(RESET);
    }
    i2c_data_set(SET);
}

static uint8_t i2c_read_ack(void) {
    uint8_t result;

    i2c_data_dir_set(GPIO_MODE_INPUT);
    usb_udelay(5);
    i2c_sclk_set(SET);
    usb_udelay(5);
    result = (uint8_t)i2c_data_get();
    i2c_sclk_set(RESET);
    // i2c data pin must be set as low, becasue "end signal" may follow it.
    i2c_data_dir_set(GPIO_MODE_OUTPUT);

    return result;
}

static void i2c_stop(void) {
    i2c_data_set(RESET);
	usb_udelay(5);
    i2c_sclk_set(SET);
    usb_udelay(5);
    i2c_data_set(SET);
}

int8_t i2c_write_reg(uint8_t reg_addr, uint8_t *data_buff, uint32_t data_len) {
    i2c_start();

    i2c_send_byte(global_i2c_config.current_i2c_addr);
    if (i2c_read_ack()) {
        i2c_stop();
        IIC_DBG("i2c no device ack\r\n");
        return -1;
    }

    i2c_send_byte(reg_addr);
    if (i2c_read_ack()) {
        i2c_stop();
        IIC_DBG("i2c no device ack\r\n");
        return -1;
    }

    while(data_len)
    {
        i2c_send_byte(*data_buff);
        if (i2c_read_ack()) {
            i2c_stop();
            IIC_DBG("i2c no device ack\r\n");
            return -1;
        }
        data_len--;
        data_buff++;
    }

    i2c_stop();

    return 0;
}
