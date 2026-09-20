#include "device_state.hpp"
#include <cmath>

#include "esp_err.h"

namespace device {
    namespace {
        constexpr float ChannelGainMinDB = -80.0f;
        constexpr float ChannelGainMaxDb = 12.0f;

        State s_state{};
        uint32_t s_revision = 0;

        template<typename ChannelArray>
        MutationResult set_channel_gain(ChannelArray &channels, const size_t index, const float gain_db) {
            if (index >= channels.size()) {
                return {
                    .error = DeviceError::InvalidChannel,
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

            if (gain_db < ChannelGainMinDB || gain_db > ChannelGainMaxDb) {
                return {
                    .error = DeviceError::GainOutOfRange,
                    .changed = false,
                    .revision = s_revision
                };
            }

            auto &channel = channels[index];

            if (channel.gain_db == gain_db) {
                return {
                    .error = DeviceError::Ok,
                    .changed = false,
                    .revision = s_revision
                };
            }

            channel.gain_db = gain_db;
            s_state.preset_modified = true;

            ++s_revision;

            return {
                .error = DeviceError::Ok,
                .changed = true,
                .revision = s_revision
            };
        }
    }

    const State &get_state() {
        return s_state;
    }

    uint32_t get_revision() {
        return s_revision;
    }

    MutationResult set_input_gain(const size_t input, const float gain_db) {
        return set_channel_gain(s_state.dsp.inputs, input, gain_db);
    }

    MutationResult set_output_gain(const size_t output, const float gain_db) {
        return set_channel_gain(s_state.dsp.outputs, output, gain_db);
    }
}
