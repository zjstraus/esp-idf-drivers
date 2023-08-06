/* SGP40 driver for ESP-IDF
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

typedef void *sgp40_sensor_handle_t;

/**
 * @brief Alloc and init a new sensor instance
 * @param [in] port I2C port to use
 * @param [in] address I2C address to target
 * @return Pointer to sensor instance
 */
sgp40_sensor_handle_t sgp40_sensor_init(i2c_port_t port, uint8_t address);

/**
 * @brief Query a sensor for its serial number
 * @param [in] handle Pointer to sensor instance
 * @param [out] data Target for serial number
 */
esp_err_t sgp40_get_serial_number(sgp40_sensor_handle_t handle, uint8_t data[6]);

/**
 * @brief Put a sensor into idle mode
 * @param [in] handle Pointer to sensor instance
 */
esp_err_t sgp40_turn_heater_off(sgp40_sensor_handle_t handle);

/**
 * @brief Request a sensor perform a self-test and retrieve the result
 * @param [in] handle Pointer to sensor instance
 * @param [out] data Target for test results
 */
esp_err_t sgp40_execute_self_test(sgp40_sensor_handle_t handle, uint8_t data[2]);

/**
 * @brief Retrieve the current raw value from a sensor
 * @param [in] handle Pointer to sensor instance
 * @param [out] data Target for results
 */
esp_err_t sgp40_measure_raw_signal(sgp40_sensor_handle_t handle, uint16_t *data);

/**
 * @brief Retrieve the current raw value from a sensor with temperature and humidity compensation
 * @param [in] handle Pointer to sensor instance
 * @param [in] humidity Relative humidity (0-100%)
 * @param [in] temperature Ambient temperature (-45-130 deg C)
 * @param [out] data Target for results
 */
esp_err_t sgp40_measure_raw_signal_compensated(sgp40_sensor_handle_t handle, float humidity, float temperature, uint16_t *data);