#include "include/dsp_controller.hpp"

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <memory>

#include "adau1452.hpp"
#include "device_state.hpp"
#include "dsp_mapping.hpp"
#include "esp_log.h"
#include "peq_coefficients.hpp"
#include "protocol.hpp"
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

        constexpr uint32_t NotifyStateChanged = (1U << 0);
        constexpr uint32_t NotifyFullSync = (1U << 1);

        constexpr TickType_t LevelPollPeriod = pdMS_TO_TICKS(50);

        struct AppliedState {
            bool valid = false;

            std::array<uint32_t, std::size(adau1452::output_processing)> output_volume{};
            std::array<uint32_t, std::size(adau1452::output_processing)> output_mute{};
            std::array<
                std::array<dsp::SigmaStudioBiquadWords, device::OutputPeqBands>,
                device::OutputCount
            > output_peq{};

            std::array<uint32_t, std::size(adau1452::input_processing)> input_volume{};
            std::array<uint32_t, std::size(adau1452::input_processing)> input_mute{};
            std::array<
                std::array<dsp::SigmaStudioBiquadWords, device::InputPeqBands>,
                device::InputCount
            > input_peq{};
        };

        uint32_t encode_volume(const float gain_db) {
            const double linear_gain = std::pow(10.0, static_cast<double>(gain_db) / 20.0);

            return static_cast<uint32_t>(std::llround(linear_gain * 16777216.0));
        }

        uint32_t encode_mute(const bool muted) {
            return muted ? 0x00000000 : 0x01000000;
        }

        float peak_to_dbfs(uint32_t raw) {
            constexpr float Scale = 16777216.0f;
            constexpr float FloorDbfs = -120.0f;

            const float amplitude = static_cast<float>(raw) / Scale;

            if (amplitude <= 0.0f) {
                return FloorDbfs;
            }

            const float dbfs = 20.0f * std::log10(amplitude);
            return dbfs < FloorDbfs ? FloorDbfs : dbfs;
        }

        esp_err_t read_level_snapshot(protocol::LevelSnapshot &levels) {
            for (size_t i = 0; i < std::size(adau1452::input_processing); i++) {
                uint32_t raw = 0;

                const auto &mapping = adau1452::input_processing[i];

                const esp_err_t err = adau1452::read_parameter(mapping.peak_readback.address, raw);

                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to read input %u peak", static_cast<unsigned>(i));
                    return err;
                }

                levels.inputs[i] = peak_to_dbfs(raw);
            }

            for (size_t i = 0; i < std::size(adau1452::output_processing); i++) {
                uint32_t raw = 0;

                const auto &mapping = adau1452::output_processing[i];

                const esp_err_t err = adau1452::read_parameter(mapping.peak_readback.address, raw);

                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to read input %u peak", static_cast<unsigned>(i));
                    return err;
                }

                levels.outputs[i] = peak_to_dbfs(raw);
            }

            return ESP_OK;
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

        esp_err_t apply_state(const device::State &state, bool force_all, AppliedState &applied) {
            const bool write_all = force_all || !applied.valid;

            if (write_all) {
                applied.valid = false;
            }

            for (size_t i = 0; i < std::size(adau1452::output_processing); ++i) {
                const auto &mapping = adau1452::output_processing[i];
                const auto &output = state.dsp.outputs[i];

                const uint32_t volume = encode_volume(output.gain_db);
                const uint32_t mute = encode_mute(output.muted);

                if (write_all || volume != applied.output_volume[i]) {
                    esp_err_t err = adau1452::write_parameter(mapping.volume.address, volume);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to write output %u volume", static_cast<unsigned>(i));
                        return err;
                    }
                    applied.output_volume[i] = volume;
                }

                if (write_all || mute != applied.output_mute[i]) {
                    esp_err_t err = adau1452::write_parameter(mapping.mute.address, mute);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to write output %u mute", static_cast<unsigned>(i));
                        return err;
                    }
                    applied.output_mute[i] = mute;
                }

                for (size_t band = 0; band < output.peq.bands.size(); ++band) {
                    const device::PeqBand &peq_band = output.peq.bands[band];

                    const bool active = output.peq.enabled && band < output.peq.band_count && peq_band.enabled;

                    const double gain_db = active ? static_cast<double>(peq_band.gain_db) : 0.0;

                    dsp::SigmaStudioBiquadWords words{};
                    if (!dsp::make_peaking_eq_words(
                        device::SampleRate, peq_band.frequency_hz, gain_db, peq_band.q, words)) {
                        ESP_LOGE(TAG, "Failed to calculate PEQ coefficients for output %u, band %u",
                                 static_cast<unsigned>(i), static_cast<unsigned>(band));
                        return ESP_ERR_INVALID_ARG;
                    }

                    if (write_all || words != applied.output_peq[i][band]) {
                        esp_err_t err = adau1452::write_safeload(mapping.peq[band].address, words.data(),
                                                                 words.size(), device::SampleRate);

                        if (err != ESP_OK) {
                            ESP_LOGE(TAG, "Failed to write output %u PEQ band %u",
                                     static_cast<unsigned>(i), static_cast<unsigned>(band));
                            return err;
                        }
                        applied.output_peq[i][band] = words;
                    }
                }
            }

            for (size_t i = 0; i < std::size(adau1452::input_processing); ++i) {
                const auto &mapping = adau1452::input_processing[i];
                const auto &input = state.dsp.inputs[i];

                const uint32_t volume = encode_volume(input.gain_db);
                const uint32_t mute = encode_mute(input.muted);

                if (write_all || volume != applied.input_volume[i]) {
                    esp_err_t err = adau1452::write_parameter(mapping.volume.address, volume);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to write input %u volume", static_cast<unsigned>(i));
                        return err;
                    }
                    applied.input_volume[i] = volume;
                }

                if (write_all || mute != applied.input_mute[i]) {
                    esp_err_t err = adau1452::write_parameter(mapping.mute.address, mute);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to write input %u mute", static_cast<unsigned>(i));
                        return err;
                    }
                    applied.input_mute[i] = mute;
                }

                for (size_t band = 0; band < input.peq.bands.size(); ++band) {
                    const device::PeqBand &peq_band = input.peq.bands[band];

                    const bool active = input.peq.enabled && band < input.peq.band_count && peq_band.enabled;

                    const double gain_db = active ? static_cast<double>(peq_band.gain_db) : 0.0;

                    dsp::SigmaStudioBiquadWords words{};
                    if (!dsp::make_peaking_eq_words(
                        device::SampleRate, peq_band.frequency_hz, gain_db, peq_band.q, words)) {
                        ESP_LOGE(TAG, "Failed to calculate PEQ coefficients for input %u, band %u",
                                 static_cast<unsigned>(i), static_cast<unsigned>(band));
                        return ESP_ERR_INVALID_ARG;
                    }

                    if (write_all || words != applied.input_peq[i][band]) {
                        esp_err_t err = adau1452::write_safeload(mapping.peq[band].address, words.data(),
                                                                 words.size(), device::SampleRate);

                        if (err != ESP_OK) {
                            ESP_LOGE(TAG, "Failed to write input %u PEQ band %u",
                                     static_cast<unsigned>(i), static_cast<unsigned>(band));
                            return err;
                        }
                        applied.input_peq[i][band] = words;
                    }
                }
            }

            applied.valid = true;

            return ESP_OK;
        }

        esp_err_t boot_dsp(device::State &snapshot, AppliedState &applied) {
            ESP_LOGI(TAG, "Starting DSP boot");

            // 1. mute all outputs
            esp_err_t err = set_init_mute(true);
            if (err != ESP_OK) return err;

            // 2. get initial device state
            uint32_t revision{};

            err = device::get_snapshot(snapshot, revision);
            if (err != ESP_OK) return err;

            // 3. apply all initial parameters (force_all)
            err = apply_state(snapshot, true, applied);
            if (err != ESP_OK) return err;

            // 4. release init mute
            err = set_init_mute(false);
            if (err != ESP_OK) return err;

            ESP_LOGI(TAG, "DSP boot completed");

            return ESP_OK;
        }

        void controller_task(void *arg) {
            ESP_LOGI(TAG, "Controller task started");

            auto applied = std::make_unique<AppliedState>();
            auto snapshot = std::make_unique<device::State>();
            if (!snapshot) {
                s_boot_result = ESP_ERR_NO_MEM;
                xSemaphoreGive(s_boot_semaphore);
                s_task_handle = nullptr;
                vTaskDelete(nullptr);
                return;
            }

            s_boot_result = boot_dsp(*snapshot, *applied);

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


            TickType_t next_level_poll = xTaskGetTickCount() + LevelPollPeriod;

            while (true) {
                TickType_t wait_ticks = next_level_poll - xTaskGetTickCount();

                if (wait_ticks > LevelPollPeriod) {
                    wait_ticks = 0;
                }

                uint32_t notifications = 0;

                // wakes: on notification or on meter polling deadline
                xTaskNotifyWait(0, UINT32_MAX, &notifications, wait_ticks);


                // PROCESS CHANGES

                if ((notifications & (NotifyStateChanged | NotifyFullSync)) != 0) {
                    const bool full_sync = (notifications & NotifyFullSync) != 0;

                    uint32_t revision{};

                    esp_err_t err = device::get_snapshot(*snapshot, revision);

                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to obtain device snapshot");

                        // retry after 100ms
                        vTaskDelay(pdMS_TO_TICKS(100));
                        xTaskNotify(xTaskGetCurrentTaskHandle(), full_sync ? NotifyFullSync : NotifyStateChanged,
                                    eSetBits);
                        continue;
                    }

                    if (full_sync) {
                        err = set_init_mute(true);
                        if (err != ESP_OK) {
                            ESP_LOGE(TAG, "Failed to mute DSP for full sync");
                        } else {
                            err = apply_state(*snapshot, true, *applied);

                            if (err == ESP_OK) {
                                err = set_init_mute(false);
                            }
                        }
                    } else {
                        err = apply_state(*snapshot, false, *applied);
                    }

                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "DSP synchronization failed: %s", esp_err_to_name(err));

                        vTaskDelay(pdMS_TO_TICKS(100));

                        xTaskNotify(xTaskGetCurrentTaskHandle(), full_sync ? NotifyFullSync : NotifyStateChanged,
                                    eSetBits);
                    }
                }


                // Check if meter polling is due
                const TickType_t remaining = next_level_poll - xTaskGetTickCount();

                const bool level_poll_due = remaining == 0 || remaining > LevelPollPeriod;
                if (!level_poll_due) {
                    continue;
                }

                uint32_t pending_notifications = 0;

                if (xTaskNotifyWait(0, UINT32_MAX, &pending_notifications, 0) == pdPASS) {
                    xTaskNotify(xTaskGetCurrentTaskHandle(), pending_notifications, eSetBits);

                    continue;
                }

                // publish meter levels
                protocol::LevelSnapshot levels{};

                const esp_err_t err = read_level_snapshot(levels);

                if (err == ESP_OK) {
                    protocol::publish_levels(levels);
                } else {
                    ESP_LOGW(TAG, "Level readback failed: %s", esp_err_to_name(err));
                }

                next_level_poll = xTaskGetTickCount() + LevelPollPeriod;
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

    void notify_state_changed() {
        if (s_task_handle != nullptr) {
            xTaskNotify(s_task_handle, NotifyStateChanged, eSetBits);
        }
    }

    void request_full_sync() {
        if (s_task_handle != nullptr) {
            xTaskNotify(s_task_handle, NotifyFullSync, eSetBits);
        }
    }
}
