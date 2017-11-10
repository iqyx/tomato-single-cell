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

#pragma once

#include <stdint.h>

#define NTC_ADC_CHANNEL 3
#define NTC_PORT GPIOA
#define NTC_PIN GPIO3
#define NTC_ENABLE_PORT GPIOA
#define NTC_ENABLE_PIN GPIO15
#define NTC_BETA 3977.0



void ntc_init(void);
void ntc_read(void);

