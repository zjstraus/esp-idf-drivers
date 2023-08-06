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

#include <stdlib.h>
#include <stdio.h>
#include "sgp40.h"

static const uint8_t SGP40_GET_SERIAL_CMD[] = {0x36, 0x82};
static const uint8_t SGP40_TURN_HEATER_OFF_CMD[] = {0x36, 0x15};
static const uint8_t SGP40_SELF_TEST_CMD[] = {0x28, 0x0E};
static const uint8_t SGP40_RAW_READ_CMD[] = {0x26, 0x0F, 0x80, 0x00, 0xA2, 0x66, 0x66, 0x93};

typedef struct {
    i2c_port_t i2c_port;
    uint8_t i2c_address;

    uint8_t *read_buffer;
} sgp40_sensor_runtime_t;

sgp40_sensor_handle_t sgp40_sensor_init(i2c_port_t port, uint8_t address)
{
    sgp40_sensor_runtime_t *runtime = calloc(1, sizeof(sgp40_sensor_runtime_t));
    if (!runtime) {
        free(runtime);
        return NULL;
    }

    runtime->i2c_port=port;
    runtime->i2c_address=address;

    runtime->read_buffer = calloc(1, 9);
    if (!runtime->read_buffer) {
        free(runtime->read_buffer);
        free(runtime);
        return NULL;
    }

    return runtime;
}

static bool sgp40_crc(sgp40_sensor_handle_t handle, uint8_t index)
{
    sgp40_sensor_runtime_t *runtime = (sgp40_sensor_runtime_t *)handle;
    uint8_t crc = 0xFF;

    for (int i = index; i < index+2; i++) {
        crc ^= runtime->read_buffer[i];

        for (uint8_t bit = 8; bit > 0; --bit) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31u;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc == runtime->read_buffer[index + 2];
}

esp_err_t sgp40_get_serial_number(sgp40_sensor_handle_t handle, uint8_t data[6])
{
    sgp40_sensor_runtime_t *runtime = (sgp40_sensor_runtime_t *)handle;
    int i2c_err = ESP_OK;

    i2c_err = i2c_master_write_to_device(runtime->i2c_port, runtime->i2c_address, &SGP40_GET_SERIAL_CMD, sizeof(SGP40_GET_SERIAL_CMD), 1000 / portTICK_PERIOD_MS);
    if (i2c_err != ESP_OK) {
        return i2c_err;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);


    i2c_err = i2c_master_read_from_device(runtime->i2c_port, runtime->i2c_address, runtime->read_buffer, 9, 100 / portTICK_PERIOD_MS);
    if (i2c_err != ESP_OK) {
        return i2c_err;
    }

    data[0] = runtime->read_buffer[0];
    data[1] = runtime->read_buffer[1];
    data[2] = runtime->read_buffer[3];
    data[3] = runtime->read_buffer[4];
    data[4] = runtime->read_buffer[6];
    data[5] = runtime->read_buffer[7];

    if (!sgp40_crc(runtime, 0)) {
        return 2;
    }

    if (!sgp40_crc(runtime, 3)) {
        return 3;
    }

    if (!sgp40_crc(runtime, 6)) {
        return 4;
    }

    return ESP_OK;
}

esp_err_t sgp40_turn_heater_off(sgp40_sensor_handle_t handle)
{
    sgp40_sensor_runtime_t *runtime = (sgp40_sensor_runtime_t *)handle;
    return i2c_master_write_to_device(runtime->i2c_port, runtime->i2c_address, &SGP40_TURN_HEATER_OFF_CMD, sizeof(SGP40_TURN_HEATER_OFF_CMD),  1000 / portTICK_PERIOD_MS);
}

esp_err_t sgp40_execute_self_test(sgp40_sensor_handle_t handle, uint8_t data[2])
{
    sgp40_sensor_runtime_t *runtime = (sgp40_sensor_runtime_t *)handle;
    int i2c_err = ESP_OK;

    i2c_err = i2c_master_write_to_device(runtime->i2c_port, runtime->i2c_address, &SGP40_SELF_TEST_CMD, sizeof(SGP40_SELF_TEST_CMD), 1000 / portTICK_PERIOD_MS);
    if (i2c_err != ESP_OK) {
        return i2c_err;
    }

    vTaskDelay(320 / portTICK_PERIOD_MS);

    i2c_err = i2c_master_read_from_device(runtime->i2c_port, runtime->i2c_address, runtime->read_buffer, 3, 100 / portTICK_PERIOD_MS);
    if (i2c_err != ESP_OK) {
        return i2c_err;
    }

    data[0] = runtime->read_buffer[0];
    data[1] = runtime->read_buffer[1];

    if (!sgp40_crc(runtime, 0)) {
        return 2;
    }

    return ESP_OK;
}

esp_err_t sgp40_measure_raw_signal(sgp40_sensor_handle_t handle, uint8_t data[2])
{
    sgp40_sensor_runtime_t *runtime = (sgp40_sensor_runtime_t *)handle;
    int i2c_err = ESP_OK;

    i2c_err = i2c_master_write_to_device(runtime->i2c_port, runtime->i2c_address, &SGP40_RAW_READ_CMD, sizeof(SGP40_RAW_READ_CMD), 1000 / portTICK_PERIOD_MS);
    if (i2c_err != ESP_OK) {
        return i2c_err;
    }

    vTaskDelay(30 / portTICK_PERIOD_MS);

    i2c_err = i2c_master_read_from_device(runtime->i2c_port, runtime->i2c_address, runtime->read_buffer, 3, 100 / portTICK_PERIOD_MS);
    if (i2c_err != ESP_OK) {
        return i2c_err;
    }

    data[0] = runtime->read_buffer[0];
    data[1] = runtime->read_buffer[1];

    if (!sgp40_crc(runtime, 0))  {
        return 2;
    }

    return ESP_OK;
}