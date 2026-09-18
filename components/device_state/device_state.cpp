#include "device_state.hpp"
#include <cstring>
#include <cmath>

#include "esp_err.h"

namespace device {
    namespace {
        constexpr float_t OutputGainMinDB = -80.0f;
        constexpr float_t OutputGainMaxDb = 12.0f;

        State s_state = {};
        uint32_t s_revision = 0;

        void init_default_peq_band(PeqBand *band) {
            *band = (PeqBand){
                .enabled = false,
                .frequency_hz = 1000.0f,
                .gain_db = 0.0f,
                .q = 1.0f,
                .type = FilterType::PEAK,
            };
        }

        void init_default_input_state(InputState *state) {
            state->gain_db = 0.0f;
            state->muted = false;

            for (auto &i: state->peq) {
                init_default_peq_band(&i);
            }
        }

        void init_default_output_state(OutputState *state) {
            state->gain_db = 0.0f;
            state->muted = false;

            for (auto &i: state->peq) {
                init_default_peq_band(&i);
            }
        }
    }

    esp_err_t init() {
        memset(&s_state, 0, sizeof(s_state));

        for (auto &input: s_state.dsp.inputs) {
            init_default_input_state(&input);
        }

        for (auto &output: s_state.dsp.outputs) {
            init_default_output_state(&output);
        }

        s_state.active_preset = InvalidPreset;
        s_state.preset_modified = false;

        s_revision = 0;
        return ESP_OK;
    }

    const State &get_state() {
        return s_state;
    }

    uint32_t get_revision() {
        return s_revision;
    }

    MutationResult set_output_gain(const size_t output, const float gain_db) {
        if (output >= OutputCount) {
            return {
                .error = DeviceError::InvalidOutput,
                .changed = false,
                .revision = s_revision
            };
        }

        if (!std::isfinite(gain_db)) {
            return {
                .error = DeviceError::InvalidGain,
                .changed = false,
                .revision = s_revision
            };
        }

        if (gain_db < OutputGainMinDB || gain_db > OutputGainMaxDb) {
            return {
                .error = DeviceError::GainOutOfRange,
                .changed = false,
                .revision = s_revision
            };
        }

        if (s_state.dsp.outputs[output].gain_db == gain_db) {
            return {
                .error = DeviceError::Ok,
                .changed = false,
                .revision = s_revision
            };
        }

        s_state.dsp.outputs[output].gain_db = gain_db;
        s_state.preset_modified = true;

        ++s_revision;

        return {
            .error = DeviceError::Ok,
            .changed = true,
            .revision = s_revision
        };
    }
}
