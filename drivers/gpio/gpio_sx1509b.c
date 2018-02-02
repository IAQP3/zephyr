/*
 * Copyright (c) 2018 Aapo Vienamo
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <gpio.h>
#include <i2c.h>
#include <misc/byteorder.h>

/** Cache of the output configuration and data of the pins */
struct gpio_sx1509b_pin_state {
	u16_t input_disable;
	u16_t pull_up;
	u16_t pull_down;
	u16_t open_drain;
	u16_t polarity;
	u16_t dir;
	u16_t data;
};

/** Runtime driver data */
struct gpio_sx1509b_drv_data {
	struct device *i2c_master;
	struct gpio_sx1509b_pin_state pin_state;
};

/** Configuration data */
struct gpio_sx1509b_config {
	const char *i2c_master_dev_name;
	u16_t i2c_slave_addr;
};

/* General configuratino register addresses */
enum {
	/* TODO: Add rest of the regs */
	SX1509B_REG_CLOCK               = 0x1e,
	SX1509B_REG_RESET               = 0x7d,
};

/* Register bits for SX1509B_REG_CLOCK */
enum {
	SX1509B_REG_CLOCK_FOSC_OFF	= 0<<5,
	SX1509B_REG_CLOCK_FOSC_EXT	= 1<<5,
	SX1509B_REG_CLOCK_FOSC_INT_2MHZ	= 2<<5,
};

/* Pin configuration register addresses */
enum {
	SX1509B_REG_INPUT_DISABLE	= 0x00,
	SX1509B_REG_PULL_UP		= 0x06,
	SX1509B_REG_PULL_DOWN		= 0x08,
	SX1509B_REG_OPEN_DRAIN		= 0x0a,
	SX1509B_REG_DIR			= 0x0e,
	SX1509B_REG_DATA		= 0x10,
	SX1509B_REG_LED_DRIVER_ENABLE	= 0x20,
};

/**
 * @brief Write a 16-bit wide word to an internal address of an I2C slave.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param dev_addr Address of the I2C device for writing.
 * @param reg_addr Address of the internal register being written.
 * @param value Value to be written to internal register.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static inline int i2c_reg_write_word_be(struct device *dev, u16_t dev_addr,
					u8_t reg_addr, u16_t value)
{
	u8_t tx_buf[3] = {reg_addr, value>>8, value & 0xff};

	return i2c_write(dev, tx_buf, 3, dev_addr);
}

/**
 * @brief Configure pin or port
 *
 * @param dev Device struct of the SX1509B
 * @param access_op Access operation (pin or port)
 * @param pin The pin number
 * @param flags Flags of pin or port
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_sx1509b_config(struct device *dev, int access_op, u32_t pin,
			       int flags)
{
	const struct gpio_sx1509b_config *cfg = dev->config->config_info;
	struct gpio_sx1509b_drv_data *drv_data = dev->driver_data;
	struct gpio_sx1509b_pin_state *pins = &drv_data->pin_state;
	int ret;

	switch (access_op) {
	case GPIO_ACCESS_BY_PIN:
		if ((flags & GPIO_DIR_MASK) == GPIO_DIR_IN) {
			pins->dir |= 1<<pin;
			pins->input_disable &= ~(1<<pin);
		} else {
			pins->dir &= ~(1<<pin);
			pins->input_disable |= 1<<pin;
		}
		if ((flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_UP) {
			pins->pull_up |= 1<<pin;
		} else {
			pins->pull_up &= ~(1<<pin);
		}
		if ((flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_DOWN) {
			pins->pull_down |= 1<<pin;
		} else {
			pins->pull_down &= ~(1<<pin);
		}
		if ((flags & GPIO_PUD_MASK) == GPIO_PUD_NORMAL) {
			pins->pull_up &= ~(1<<pin);
			pins->pull_down &= ~(1<<pin);
		}
		if (flags & GPIO_DS_DISCONNECT_HIGH) {
			pins->open_drain |= 1<<pin;
		} else {
			pins->open_drain &= ~(1<<pin);
		}
		if (flags & GPIO_POL_INV) {
			pins->polarity |= 1<<pin;
		} else {
			pins->polarity &= ~(1<<pin);
		}
		break;
	case GPIO_ACCESS_BY_PORT:
		if ((flags & GPIO_DIR_MASK) == GPIO_DIR_IN) {
			pins->dir = 0xffff;
			pins->input_disable = 0x0000;
		} else {
			pins->dir = 0x0000;
			pins->input_disable = 0xffff;
		}
		if ((flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_UP) {
			pins->pull_up = 0xffff;
		} else {
			pins->pull_up = 0x0000;
		}
		if ((flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_DOWN) {
			pins->pull_down = 0xffff;
		} else {
			pins->pull_down = 0x0000;
		}
		if ((flags & GPIO_PUD_MASK) == GPIO_PUD_NORMAL) {
			pins->pull_up = 0x0000;
			pins->pull_down = 0x0000;
		}
		if (flags & GPIO_DS_DISCONNECT_HIGH) {
			pins->open_drain = 0xffff;
		} else {
			pins->open_drain = 0x0000;
		}
		if (flags & GPIO_POL_INV) {
			pins->polarity = 0xffff;
		} else {
			pins->polarity = 0x0000;
		}
		break;
	default:
		return -ENOTSUP;
	}

	ret = i2c_reg_write_word_be(drv_data->i2c_master, cfg->i2c_slave_addr,
				    SX1509B_REG_DIR, pins->dir);
	if (ret)
		return ret;

	ret = i2c_reg_write_word_be(drv_data->i2c_master, cfg->i2c_slave_addr,
				    SX1509B_REG_INPUT_DISABLE,
				    pins->input_disable);
	if (ret)
		return ret;

	ret = i2c_reg_write_word_be(drv_data->i2c_master, cfg->i2c_slave_addr,
				    SX1509B_REG_PULL_UP,
				    pins->pull_up);
	if (ret)
		return ret;

	ret = i2c_reg_write_word_be(drv_data->i2c_master, cfg->i2c_slave_addr,
				    SX1509B_REG_PULL_DOWN,
				    pins->pull_up);
	if (ret)
		return ret;

	ret = i2c_reg_write_word_be(drv_data->i2c_master, cfg->i2c_slave_addr,
				    SX1509B_REG_OPEN_DRAIN,
				    pins->pull_up);

	return ret;
}

/**
 * @brief Set the pin or port output
 *
 * @param dev Device struct of the SX1509B
 * @param access_op Access operation (pin or port)
 * @param pin The pin number
 * @param value Value to set (0 or 1)
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_sx1509b_write(struct device *dev, int access_op, u32_t pin,
			      u32_t value)
{
	const struct gpio_sx1509b_config *cfg = dev->config->config_info;
	struct gpio_sx1509b_drv_data *drv_data = dev->driver_data;
	u16_t *pin_data = &drv_data->pin_state.data;

	switch (access_op) {
	case GPIO_ACCESS_BY_PIN:
		if (value) {
			*pin_data |= 1<<pin;
		} else {
			*pin_data &= ~(1<<pin);
		}
		break;
	case GPIO_ACCESS_BY_PORT:
		*pin_data = value;
		break;
	default:
		return -ENOTSUP;
	}

	return i2c_reg_write_word_be(drv_data->i2c_master, cfg->i2c_slave_addr,
				     SX1509B_REG_DATA, *pin_data);
}

/**
 * @brief Read the pin or port data
 *
 * @param dev Device struct of the SX1509B
 * @param access_op Access operation (pin or port)
 * @param pin The pin number
 * @param value Value of input pin(s)
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_sx1509b_read(struct device *dev, int access_op, u32_t pin,
			      u32_t *value)
{
	const struct gpio_sx1509b_config *cfg = dev->config->config_info;
	struct gpio_sx1509b_drv_data *drv_data = dev->driver_data;
	u16_t pin_data;
	int ret;

	ret = i2c_burst_read(drv_data->i2c_master, cfg->i2c_slave_addr,
			     SX1509B_REG_DATA, (u8_t *)&pin_data,
			     sizeof(pin_data));
	if (ret)
		return ret;

	pin_data = sys_be16_to_cpu(pin_data);

	switch (access_op) {
	case GPIO_ACCESS_BY_PIN:
		*value = (pin_data & (1<<pin)) && 1;
		break;
	case GPIO_ACCESS_BY_PORT:
		*value = pin_data;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static const struct gpio_driver_api gpio_sx1509b_drv_api_funcs = {
	.config	= gpio_sx1509b_config,
	.write	= gpio_sx1509b_write,
	.read	= gpio_sx1509b_read,
};

/**
 * @brief Initialization function of SX1509B
 *
 * @param dev Device struct
 * @return 0 if successful, failed otherwise.
 */
static int gpio_sx1509b_init(struct device *dev)
{
	const struct gpio_sx1509b_config *cfg = dev->config->config_info;
	struct gpio_sx1509b_drv_data *drv_data = dev->driver_data;
	int ret;

	drv_data->i2c_master = device_get_binding(cfg->i2c_master_dev_name);
	if (!drv_data->i2c_master)
		return -EINVAL;

	/* Reset state */
	drv_data->pin_state = (struct gpio_sx1509b_pin_state) {
		.input_disable	= 0x0000,
		.pull_up	= 0x0000,
		.pull_down	= 0x0000,
		.open_drain	= 0x0000,
		.dir		= 0xffff,
		.data		= 0xffff,
	};

	ret = i2c_reg_write_byte(drv_data->i2c_master, cfg->i2c_slave_addr,
				 SX1509B_REG_RESET, 0x12);
	if (ret)
		return ret;
	ret = i2c_reg_write_byte(drv_data->i2c_master, cfg->i2c_slave_addr,
				 SX1509B_REG_RESET, 0x34);
	if (ret)
		return ret;

	ret = i2c_reg_write_byte(drv_data->i2c_master, cfg->i2c_slave_addr,
				 SX1509B_REG_CLOCK,
				 SX1509B_REG_CLOCK_FOSC_INT_2MHZ);
	if (ret)
		return ret;

	dev->driver_api = &gpio_sx1509b_drv_api_funcs;

	return 0;
}

static const struct gpio_sx1509b_config gpio_sx1509b_cfg = {
	.i2c_master_dev_name = CONFIG_GPIO_SX1509B_I2C_MASTER_DEV_NAME,
	.i2c_slave_addr	= CONFIG_GPIO_SX1509B_I2C_ADDR,
};

static struct gpio_sx1509b_drv_data gpio_sx1509b_drvdata;

DEVICE_INIT(gpio_sx1509b, CONFIG_GPIO_SX1509B_DEV_NAME, gpio_sx1509b_init,
	    &gpio_sx1509b_drvdata, &gpio_sx1509b_cfg,
	    POST_KERNEL, CONFIG_GPIO_SX1509B_INIT_PRIORITY);
