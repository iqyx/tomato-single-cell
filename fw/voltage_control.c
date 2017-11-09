/*
 * PWM control of the Vin_reg and Vfb resistor dividers
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
#include <libopencm3/stm32/timer.h>

#include "uxb_slave.h"
#include "voltage_control.h"


#define PWM_MAX 255

void voltage_control_init(void) {

	/* PWM outputs. */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO0 | GPIO1);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO0 | GPIO1);
	gpio_set_af(GPIOB, GPIO_AF1, GPIO0 | GPIO1);

	/* Setup the timer used to control the PWM outputs. */
	rcc_periph_clock_enable(RCC_TIM3);
	timer_reset(TIM3);
	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_continuous_mode(TIM3);
	timer_direction_up(TIM3);
	timer_disable_preload(TIM3);
	timer_enable_update_event(TIM3);
	timer_set_prescaler(TIM3, 0);
	timer_set_period(TIM3, PWM_MAX);

	timer_disable_oc_preload(TIM3, TIM_OC3);
	timer_set_oc_mode(TIM3, TIM_OC3, TIM_OCM_PWM2);
	timer_set_oc_value(TIM3, TIM_OC3, 0);
	timer_enable_oc_output(TIM3, TIM_OC3);

	timer_disable_oc_preload(TIM3, TIM_OC4);
	timer_set_oc_mode(TIM3, TIM_OC4, TIM_OCM_PWM2);
	timer_set_oc_value(TIM3, TIM_OC4, 0);
	timer_enable_oc_output(TIM3, TIM_OC4);

	timer_enable_counter(TIM3);
}


void voltage_control_set_vin_reg(uint32_t vin_reg_mv) {

	/* Check the voltage range and calculate the pwm duty cycle value. */
	if (vin_reg_mv < 5400) {
		vin_reg_mv = 5400;
	}
	uint32_t pwm = (vin_reg_mv - 5400) * PWM_MAX / (29945 - 5400);
	if (pwm > PWM_MAX) {
		pwm = PWM_MAX;
	}

	timer_set_oc_value(TIM3, TIM_OC3, pwm);
}


void voltage_control_set_vbat(uint32_t vbat_mv) {

	if (vbat_mv < 3630) {
		vbat_mv = 3630;
	}
	uint32_t pwm = (vbat_mv - 3630) * PWM_MAX / (4320 - 3630);
	if (pwm > PWM_MAX) {
		pwm = PWM_MAX;
	}

	timer_set_oc_value(TIM3, TIM_OC4, pwm);
}


uint32_t voltage_control_get_vin_reg_mv(void) {
	return 0;
}


uint32_t voltage_control_get_vbat_mv(void) {
	return 0;
}
