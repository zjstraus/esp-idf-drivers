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

#include <stdio.h>
#include <stdlib.h>

#include "esp_event.h"
#include "esp_log.h"

#include "sgp40.h"
#include "sgp40_manager.h"

static const char *SGP40_MANAGER_TAG = "SGP40 manager";

static const TickType_t DELAY_POLL_SLEEP =
        ((CONFIG_SGP40_MANAGER_POLL_TIME * 1000) -
         (CONFIG_SGP40_COMMAND_DELAY_GETSERIAL + CONFIG_SGP40_MANAGER_SENSOR_WARMUP_TIME)) / portTICK_PERIOD_MS;

static const TickType_t DELAY_SENSOR_WARMUP = CONFIG_SGP40_MANAGER_SENSOR_WARMUP_TIME / portTICK_PERIOD_MS;

ESP_EVENT_DEFINE_BASE(SGP40_MANAGER_EVENT);

typedef struct {
    sgp40_sensor_handle_t sensor_handle;
    GasIndexAlgorithmParams sensiron_algo_params;
    float humidity;
    float temperature;

    uint16_t raw_reading;
    int32_t processed_index;

    TaskHandle_t task_handle;
    esp_event_loop_handle_t event_target;
} sgp40_manager_runtime_t;

static void sgp40_manager_task_entry(void *arg)
{
    sgp40_manager_runtime_t *runtime = (sgp40_manager_runtime_t *) arg;
    esp_err_t op_err;
    while (1) {
        // Take and discard a reading to wake the sensor
        op_err = sgp40_measure_raw_signal_compensated(runtime->sensor_handle, runtime->humidity, runtime->temperature,
                                                      &runtime->raw_reading);
        if (op_err != ESP_OK) {
            ESP_LOGE(SGP40_MANAGER_TAG, "Error triggering sensor warmup %x", op_err);
            goto skip_reading;
        }
        vTaskDelay(DELAY_SENSOR_WARMUP);

        // Take real reading
        op_err = sgp40_measure_raw_signal_compensated(runtime->sensor_handle, runtime->humidity, runtime->temperature,
                                                      &runtime->raw_reading);
        if (op_err != ESP_OK) {
            ESP_LOGE(SGP40_MANAGER_TAG, "Error reading sensor %x", op_err);
            goto skip_reading;
        }

        // Sleep sensor
        sgp40_turn_heater_off(runtime->sensor_handle);

        GasIndexAlgorithm_process(&runtime->sensiron_algo_params, runtime->raw_reading, &(runtime->processed_index));

        esp_event_post_to(runtime->event_target, SGP40_MANAGER_EVENT, SGP40_MANAGER_READING_INDEX,
                          &(runtime->processed_index), sizeof(runtime->processed_index), 1000 / portTICK_PERIOD_MS);

        skip_reading:
        vTaskDelay(DELAY_POLL_SLEEP);
    }
}


sgp40_manager_handle_t sgp40_manager_init(i2c_port_t port, uint8_t address, esp_event_loop_handle_t event_target)
{
    sgp40_manager_runtime_t *runtime = calloc(1, sizeof(sgp40_manager_runtime_t));
    if (!runtime) {
        ESP_LOGE(SGP40_MANAGER_TAG, "calloc for runtime failed");
        goto error_struct;
    }
    runtime->temperature = 20;
    runtime->humidity = 50;

    GasIndexAlgorithm_init_with_sampling_interval(&runtime->sensiron_algo_params, GasIndexAlgorithm_ALGORITHM_TYPE_VOC,
                                                  1.0 * CONFIG_SGP40_MANAGER_POLL_TIME);

    runtime->event_target = event_target;
    runtime->sensor_handle = sgp40_sensor_init(port, address);
    if (!runtime->sensor_handle) {
        ESP_LOGE(SGP40_MANAGER_TAG, "init of sensor runtime failed");
        goto error_struct;
    }

    BaseType_t taskErr = xTaskCreate(sgp40_manager_task_entry, "SGP40 Sensor Manager", 2000,
                                     runtime, 2, &runtime->task_handle);

    if (taskErr != pdTRUE) {
        ESP_LOGE(SGP40_MANAGER_TAG, "task creation failed");
        goto error_task_create;
    }

    ESP_LOGI(SGP40_MANAGER_TAG, "manager started");

    return runtime;

    error_task_create:
    error_struct:
    free(runtime);
    return NULL;
}

void sgp40_manager_set_compensation(sgp40_manager_handle_t handle, float temperature, float humidity)
{
    sgp40_manager_runtime_t *runtime = (sgp40_manager_runtime_t *) handle;
    runtime->humidity = humidity;
    runtime->temperature = temperature;
}