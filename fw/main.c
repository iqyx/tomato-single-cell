/*
 * tomato-series single cell LiFePo4/Li-Ion charger firmware
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

#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "uxb_slave.h"
#include "battery_monitor.h"
#include "voltage_control.h"


#define LED_PORT GPIOB
#define LED_PIN GPIO3


/* Semihosting. */
extern void initialise_monitor_handles(void);


static void clock_setup(void) {
	/* No prediv, running at the full 8MHz HSI speed. */
	rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
}


/* GPIO initialization which does not fit elsewere. */
static void gpio_setup(void) {
	/* Initialize the STAT LED. */
	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
}


int main(void) {

	clock_setup();
	initialise_monitor_handles();
	gpio_setup();
	uxb_slave_init();
	stc3100_init();
	voltage_control_init();
	voltage_control_set_vin_reg(10000);

	while (1) {
		/* Blink the LED just for the lulz. */
		gpio_toggle(LED_PORT, LED_PIN);
		for (uint32_t i = 0; i < 100000; i++) {
			__asm__("nop");
		}

		/** @todo read by a periodic timer interrupt */
		stc3100_read();
	}

	return 0;
}


