/*
 * NTC temperature battery monitor
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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>

#include "uxb_slave.h"
#include "ntc.h"


void ntc_init(void) {
	rcc_periph_clock_enable(RCC_ADC1);
	// adc_disable_scan_mode(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_239DOT5);
	adc_power_on(ADC1);
	adc_enable_vrefint();

	gpio_mode_setup(NTC_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, NTC_PIN);
	gpio_mode_setup(NTC_ENABLE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, NTC_ENABLE_PIN);
}


void ntc_read(void) {
	uint8_t channels[16];

	/* Enable the NTC resistor divider. */
	gpio_set(NTC_ENABLE_PORT, NTC_ENABLE_PIN);

	/* Capture voltage of the internal voltage reference. */
	channels[0] = 17;
	adc_set_regular_sequence(ADC1, 1, channels);
	adc_start_conversion_regular(ADC1);
	while (!adc_eoc(ADC1)) {
		;
	}

	/* Read Vref reading at 3.3V Vdda (calibration values, see the datasheet) and compute its voltage. */
	uint16_t vref_cal = *(uint16_t *)0x1ffff7ba;
	uint32_t vref_uv = vref_cal * 3300000ull / 4096ull;

	/* Compute Vdda using computed value of the internal Vref */
	uint32_t vdda_uv = vref_uv * 4096ull / adc_read_regular(ADC1);

	/* Convert the NTC input voltage now. */
	channels[0] = NTC_ADC_CHANNEL;
	adc_set_regular_sequence(ADC1, 1, channels);
	adc_start_conversion_regular(ADC1);
	while (!adc_eoc(ADC1)) {
		;
	}

	/* Compute the resulting value in millivolts. */
	int32_t reading = adc_read_regular(ADC1);
	int32_t vntc_uv = (int32_t)((uint64_t)reading * (uint64_t)vdda_uv / 4096ull);
	int32_t rntc = 10000.0 / (((float)vdda_uv / (float)vntc_uv) - 1.0);
	battery_temperature_mc = (((NTC_BETA * 298.15) / (NTC_BETA + (298.15 * log(rntc / 10000.0)))) - 273.15) * 1000.0;

	/* Disable the NTC resistor divider. */
	// gpio_clear(NTC_ENABLE_PORT, NTC_ENABLE_PIN);


}
