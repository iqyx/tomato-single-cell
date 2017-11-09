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

#pragma once

#include <stdint.h>


void voltage_control_init(void);

/**
 * @brief Adjust the Vin_reg divider
 *
 * The input resistor divider is 1M/1M. The LT battery charger is regulating the Vin_reg
 * voltage to 2.7V. With no pull down, the MPP voltage is set to 2.7*2 = 5.4V (PWM duty
 * 0). When the PWM duty cycle is set to 255, another pull down resistor of 110K is added
 * to the resistor divider forming a divider with 1M/1M|110K = ~1M/99.1K. The Vin_reg
 * voltage in this case is 2.7/99.1*1099.1 = 29.945V.
 *
 * @return Nothing. The function is designed not to fail.
 */
void voltage_control_set_vin_reg(uint32_t vin_reg_mv);

/**
 * @brief Adjust the Vfb divider
 *
 * Vfb is regulating to 3.3V. Vbat divider is 100K/1M. Minimum Vbat is 3.3/1000*1100
 * = 3.63V (LiFePo4). Maximum PWM duty adds another 480K. The bottom resistor will be
 * 1000K|480K = ~324K. Maximum battery voltage is 3.3/324*424 = 4.32V (Li-Ion).
 *
 * @return Nothing. The function is designed not to fail.
 */
void voltage_control_set_vbat(uint32_t vbat_mv);

/**
 * @brief Calculate the Vin_reg voltage back from the currently set PWM duty cycle
 *
 * @return Voltage in millivolts.
 */
uint32_t voltage_control_get_vin_reg_mv(void);

/**
 * @brief Calculate the Vfb voltage back from the currently set PWM duty cycle
 *
 * @return Voltage in millivolts.
 */
uint32_t voltage_control_get_vbat_mv(void);
