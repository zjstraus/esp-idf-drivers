/* Event loop wrapper around SGP40 for ESP-IDF
 * Copyright (C) 2023 Zach Strauss
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "driver/i2c.h"
#include "sensirion_gas_index_algorithm.h"

typedef void *sgp40_manager_handle_t;

ESP_EVENT_DECLARE_BASE(SGP40_MANAGER_EVENT);
typedef enum {
    SGP40_MANAGER_READING_INDEX
} sgp40_manager_event_id_t;

sgp40_manager_handle_t sgp40_manager_init(i2c_port_t port, uint8_t address, esp_event_loop_handle_t event_target);

void sgp40_manager_set_compensation(sgp40_manager_handle_t handle, float temperature, float humidity);