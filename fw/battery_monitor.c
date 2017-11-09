/*
 * STC3100 battery monitor related functions
 *
 * Copyright (C) 2017, Marek Koza, qyx@krtko.org
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>

#include "uxb_slave.h"
#include "battery_monitor.h"


/* The STC3100 battery monitor is connected to the I2C1 peripheral on pins
 * PB9 and PB10 (AF4). */

/**
 * @brief I2C1 interrupt service routine
 *
 * This handler is called if a I2C1 error occurs.
 */
void i2c1_isr(void) {
	if (I2C_ISR(I2C1) & I2C_ISR_TIMEOUT) {
		I2C_ICR(I2C1) |= I2C_ICR_TIMOUTCF;
		/** @todo Reset the I2C peripheral here. */
	}
}

void stc3100_init(void) {
	/* Initialize GPIO pins needed for the I2C bus. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO9 | GPIO10);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO9 | GPIO10);
	gpio_set_af(GPIOA, GPIO_AF4, GPIO9 | GPIO10);

	/* Enable clock and initialize the I2C1 bus. */
	rcc_periph_clock_enable(RCC_I2C1);
	i2c_reset(I2C1);
	i2c_peripheral_disable(I2C1);
	i2c_enable_analog_filter(I2C1);
	i2c_set_digital_filter(I2C1, I2C_CR1_DNF_DISABLED);
	i2c_set_speed(I2C1, i2c_speed_sm_100k, 8);
	i2c_enable_stretching(I2C1);
	i2c_set_7bit_addr_mode(I2C1);

	/* Detect SCL low. 25ms timeout with 8MHz i2cclk. */
	I2C_TIMEOUTR(I2C1) |= 0x61;
	I2C_TIMEOUTR(I2C1) |= I2C_TIMEOUTR_TIMOUTEN;

	/* Enable error interrupts. */
	I2C_CR1(I2C1) |= I2C_CR1_ERRIE;
	/** @todo NVIC interrupt enable */

	i2c_peripheral_enable(I2C1);

	/* Initialize the STC3100 IC. */
	i2c_transfer7(I2C1, STC3100_ADDRESS, (uint8_t[]){0x00, 0x10}, 2, NULL, 0, I2C_TIMEOUT);
}


static uint8_t stc3100_status(void) {
	uint8_t status;
	i2c_transfer7(I2C1, STC3100_ADDRESS, (uint8_t[]){0x01}, 1, &status, 1, I2C_TIMEOUT);
	return status;
}


/**
 * @brief Return battery voltage in millivolts
 */
static int32_t stc3100_voltage(void) {
	uint8_t voltage[2];
	if (i2c_transfer7(I2C1, STC3100_ADDRESS, (uint8_t[]){0x08}, 1, voltage, 2, I2C_TIMEOUT) == false) {
		return 0;
	}
	return ((voltage[1] << 8 | voltage[0]) * 2440) / 1000;
}


/**
 * @brief Return battery current in milliamps.
 *
 * Positive value means battery charging, negative means discharging.
 */
static int32_t stc3100_current(void) {
	uint8_t current_code[2];
	if (i2c_transfer7(I2C1, STC3100_ADDRESS, (uint8_t[]){0x06}, 1, current_code, 2, I2C_TIMEOUT) == false) {
		return 0;
	}
	int16_t current = (current_code[1] << 10) | (current_code[0] << 2);
	return current / 4 * 1177 / 5;
}


/**
 * @brief Return temperature of the STC3100 die
 */
static int32_t stc3100_temp(void) {
	uint8_t temp_code[2];
	if (i2c_transfer7(I2C1, STC3100_ADDRESS, (uint8_t[]){0x0a}, 1, temp_code, 2, I2C_TIMEOUT) == false) {
		return 0;
	}
	int16_t temperature = (temp_code[1] << 12) | (temp_code[0] << 4);
	return temperature * 125 / 16;
}


/**
 * @brief Return accumulated battery charge in mAh
 */
static int32_t stc3100_charge(void) {
	uint8_t charge_code[2];
	if (i2c_transfer7(I2C1, STC3100_ADDRESS, (uint8_t[]){0x02}, 1, charge_code, 2, I2C_TIMEOUT) == false) {
		return 0;
	}
	int16_t charge = (charge_code[1] << 8) | charge_code[0];
	return charge * 67 / 50 / 10;
}


void stc3100_read(void) {
	battery_voltage_mv = stc3100_voltage();
	battery_current_ma = stc3100_current();
	board_temperature_mc = stc3100_temp();
	battery_charge_mah = stc3100_charge();
}

