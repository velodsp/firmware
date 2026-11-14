#include "include/adau1452_test.hpp"

#include "adau1452.hpp"
#include "dsp_mapping.hpp"

#include "esp_log.h"

#include <cmath>

#include "esp_check.h"

static constexpr char TAG[] = "adau1452_test";

namespace adau1452_test {
    namespace {
        uint32_t gain_db_to_8_24(const float gain_db) {
            const double linear_gain = std::pow(10.0, static_cast<double>(gain_db) / 20.0);

            return static_cast<uint32_t>(std::llround(linear_gain * 16777216.0));
        }
    }

    esp_err_t test_set_outputs_muted(bool muted) {
        ESP_LOGI(TAG, "Unmuting outputs...");

        ESP_RETURN_ON_ERROR(
            adau1452::write_parameter(adau1452::init_mute.address, muted ? 0x00000000 : 0x01000000),
            TAG,
            "Failed to unmute output %s", adau1452::init_mute.address
        );

        return ESP_OK;
    }

    esp_err_t test_output_volume(size_t index, float volume) {
        if (index >= adau1452::output_processing_count) {
            return ESP_ERR_INVALID_ARG;
        }

        const auto &output = adau1452::output_processing[index];

        uint32_t gain = gain_db_to_8_24(volume);

        ESP_RETURN_ON_ERROR(
            adau1452::write_safeload(output.volume.address, &gain, 1, 48000),
            TAG,
            "Failed to set output gain"
        );

        ESP_LOGI(TAG, "Testing output: %s", output.name);

        return ESP_OK;
    }
}
