#include "include/dsp_controller.hpp"

#include <atomic>
#include <cmath>
#include <memory>

#include "adau1452.hpp"
#include "device_state.hpp"
#include "dsp_mapping.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

namespace dsp_controller {
    namespace {
        constexpr const char *TAG = "dsp_controller";

        constexpr uint32_t TaskStackSize = 4096;
        constexpr UBaseType_t TaskPriority = 5;

        TaskHandle_t s_task_handle = nullptr;
        SemaphoreHandle_t s_boot_semaphore = nullptr;

        std::atomic<bool> s_ready{};

        esp_err_t s_boot_result = ESP_FAIL;

        uint32_t encode_volume(const float gain_db) {
            const double linear_gain = std::pow(10.0, static_cast<double>(gain_db) / 20.0);

            return static_cast<uint32_t>(std::llround(linear_gain * 16777216.0));
        }

        uint32_t encode_mute(const bool muted) {
            return muted ? 0x00000000 : 0x01000000;
        }

        esp_err_t set_init_mute(const bool muted) {
            const uint32_t value = muted ? 0x00000000 : 0x01000000;
            esp_err_t err = adau1452::write_parameter(adau1452::init_mute.address, value);

            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set init mute.");
                return err;
            }

            return ESP_OK;
        }

        esp_err_t apply_initial_state(const device::State &state) {
            for (size_t i = 0; i < adau1452::output_processing_count; ++i) {
                const auto &mapping = adau1452::output_processing[i];
                const auto &output = state.dsp.outputs[i];

                const uint32_t volume = encode_volume(output.gain_db);
                const uint32_t mute = encode_mute(output.muted);

                esp_err_t err = adau1452::write_parameter(mapping.volume.address, volume);
                if (err != ESP_OK) return err;

                err = adau1452::write_parameter(mapping.mute.address, mute);
                if (err != ESP_OK) return err;
            }

            return ESP_OK;
        }

        esp_err_t boot_dsp() {
            ESP_LOGI(TAG, "Starting DSP boot");

            // 1. mute all outputs
            esp_err_t err = set_init_mute(true);
            if (err != ESP_OK) return err;

            // 2. get initial device state
            auto snapshot = std::make_unique<device::State>();
            uint32_t revision{};

            err = device::get_snapshot(*snapshot, revision);
            if (err != ESP_OK) return err;

            // 3. apply initial parameters
            err = apply_initial_state(*snapshot);
            if (err != ESP_OK) return err;

            // 4. release init mute
            err = set_init_mute(false);
            if (err != ESP_OK) return err;

            ESP_LOGI(TAG, "DSP boot completed");

            return ESP_OK;
        }

        void controller_task(void *arg) {
            ESP_LOGI(TAG, "Controller task started");

            s_boot_result = boot_dsp();

            if (s_boot_result != ESP_OK) {
                ESP_LOGE(TAG, "DSP boot failed: %s", esp_err_to_name(s_boot_result));

                xSemaphoreGive(s_boot_semaphore);

                s_task_handle = nullptr;
                vTaskDelete(nullptr);
                return;
            }

            s_ready.store(true);

            ESP_LOGI(TAG, "Dsp Controller ready");

            xSemaphoreGive(s_boot_semaphore);

            while (true) {
                // TODO: synchronize the DSP
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
    }

    esp_err_t start() {
        if (s_task_handle != nullptr) {
            return ESP_ERR_INVALID_STATE;
        }

        s_ready.store(false);

        s_boot_semaphore = xSemaphoreCreateBinary();
        if (s_boot_semaphore == nullptr) {
            return ESP_ERR_NO_MEM;
        }

        BaseType_t result = xTaskCreate(
            controller_task,
            "dsp_controller",
            TaskStackSize,
            nullptr,
            TaskPriority,
            &s_task_handle
        );

        if (result != pdPASS) {
            vSemaphoreDelete(s_boot_semaphore);
            s_boot_semaphore = nullptr;
            s_task_handle = nullptr;

            return ESP_ERR_NO_MEM;
        }

        if (xSemaphoreTake(s_boot_semaphore, portMAX_DELAY) != pdTRUE) {
            return ESP_FAIL;
        }

        const esp_err_t err = s_boot_result;

        vSemaphoreDelete(s_boot_semaphore);
        s_boot_semaphore = nullptr;

        return err;
    }

    bool is_ready() {
        return s_ready.load();
    }
}