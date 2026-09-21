#include "device_state.hpp"
#include <cmath>

#include "esp_err.h"

namespace device {
    namespace {
        constexpr float ChannelGainMinDb = -80.0f;
        constexpr float ChannelGainMaxDb = 12.0f;

        State s_state{};
        uint32_t s_revision = 0;

        MutationResult error_result(const DeviceError error) {
            return {
                .error = error,
                .changed = false,
                .revision = s_revision,
            };
        }

        MutationResult unchanged_result() {
            return {
                .error = DeviceError::Ok,
                .changed = false,
                .revision = s_revision,
            };
        }

        MutationResult changed_result() {
            s_state.preset_modified = true;
            ++s_revision;

            return {
                .error = DeviceError::Ok,
                .changed = true,
                .revision = s_revision,
            };
        }

        template<typename ChannelArray, typename Fn>
        MutationResult with_channel(ChannelArray &channels, size_t index, Fn &&fn) {
            if (index >= channels.size()) {
                return error_result(DeviceError::InvalidChannel);
            }

            return std::forward<Fn>(fn)(channels[index]);
        }

        template<typename Fn>
        MutationResult with_channel(ChannelTarget target, Fn &&fn) {
            switch (target.kind) {
                case ChannelKind::Input:
                    return with_channel(s_state.dsp.inputs,target.index,std::forward<Fn>(fn));

                case ChannelKind::Output:
                    return with_channel(s_state.dsp.outputs,target.index,std::forward<Fn>(fn));
            }

            return error_result(DeviceError::InvalidChannel);
        }
    }

    const State &get_state() {
        return s_state;
    }

    uint32_t get_revision() {
        return s_revision;
    }

    MutationResult set_channel_gain(const ChannelTarget target, const float gain_db) {
        if (!std::isfinite(gain_db)) {
            return error_result(DeviceError::InvalidGain);
        }

        if (gain_db < ChannelGainMinDb || gain_db > ChannelGainMaxDb) {
            return error_result(DeviceError::GainOutOfRange);
        }

        return with_channel(target, [gain_db](auto &channel) {
            if (channel.gain_db == gain_db) {
                return unchanged_result();
            }

            channel.gain_db = gain_db;

            return changed_result();
        });
    }
}
