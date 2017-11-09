/*
 * Driver for the UXB bus using the libopencm3 library
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


/* I2C address of the STC3100 battery monitor. See the datasheet. */
#define STC3100_ADDRESS 0x70

/* Timeout for I2C wait cycles. */
#define I2C_TIMEOUT 500


void stc3100_init(void);
void stc3100_read(void);
