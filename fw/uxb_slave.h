/*
 * UXB Bus slave communication handling
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
#include <stdbool.h>

#include "libuxb.h"


typedef enum {
	UXB_SLAVE_OK = 0,
	UXB_SLAVE_FAILED,
} uxb_slave_ret_t;

struct uxb_slave_variables {
};

typedef struct {
	struct uxb_slave_variables vars;

} UxbSlave;


extern int32_t battery_voltage_mv;
extern int32_t battery_current_ma;
extern int32_t board_temperature_mc;
extern int32_t battery_temperature_mc;
extern int32_t battery_charge_mah;
extern int32_t battery_float_voltage_mv;
extern int32_t input_min_voltage_mv;

uxb_slave_ret_t uxb_slave_init(void);

