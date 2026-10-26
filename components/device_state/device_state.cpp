#include "device_state.hpp"
#include <cmath>
#include <type_traits>
#include <vector>

#include "esp_err.h"

namespace device {
    namespace {
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

    MutationResult set_channel_muted(const ChannelTarget target, const bool muted) {
        return with_channel(target, [muted](auto &channel) {
            if (channel.muted == muted) {
                return unchanged_result();
            }

            channel.muted = muted;

            return changed_result();
        });
    }

    MutationResult set_channel_peq_enabled(const ChannelTarget target, const bool enabled) {
        return with_channel(target, [enabled](auto &channel) {
            if (channel.peq.enabled == enabled) {
                return unchanged_result();
            }

            channel.peq.enabled = enabled;

            return changed_result();
        });
    }

    MutationResult set_channel_peq_band(const ChannelTarget target, size_t band_index, const PeqBandPatch &patch, PeqBand &band_result) {
        return with_channel(target, [&](auto &channel) {
            if (band_index >= channel.peq.bands.size()) {
                return error_result(DeviceError::InvalidPeqBand);
            }

            auto &band = channel.peq.bands[band_index];

            if (patch.frequency_hz) {
                if (!std::isfinite(*patch.frequency_hz)) {
                    return error_result(DeviceError::InvalidPeqFrequency);
                }

                if (*patch.frequency_hz < PeqFrequencyMinHz || *patch.frequency_hz > PeqFrequencyMaxHz) {
                    return error_result(DeviceError::PeqFrequencyOutOfRange);
                }
            }

            if (patch.q) {
                if (!std::isfinite(*patch.q)) {
                    return error_result(DeviceError::InvalidPeqQ);
                }

                if (*patch.q < PeqQMin || *patch.q > PeqQMax) {
                    return error_result(DeviceError::PeqQOutOfRange);
                }
            }

            if (patch.gain_db) {
                if (!std::isfinite(*patch.gain_db)) {
                    return error_result(DeviceError::InvalidPeqGain);
                }

                if (*patch.gain_db < PeqGainDbMin || *patch.gain_db > PeqGainDbMax) {
                    return error_result(DeviceError::PeqGainOutOfRange);
                }
            }

            bool changed = false;

            auto set_if_changed = [&](auto &dst, const auto &value) {
                if (value && dst != *value) {
                    dst = *value;
                    changed = true;
                }
            };

            set_if_changed(band.enabled, patch.enabled);
            set_if_changed(band.frequency_hz, patch.frequency_hz);
            set_if_changed(band.q, patch.q);
            set_if_changed(band.gain_db, patch.gain_db);
            set_if_changed(band.type, patch.type);

            band_result = band;

            return changed ? changed_result() : unchanged_result();
        });
    }

    MutationResult add_channel_peq_band(ChannelTarget target, PeqBand &band_result, size_t &band_index) {
        return with_channel(target, [&](auto &channel) {
            auto &peq = channel.peq;

            if (peq.bands.size() >= std::remove_reference_t<decltype(peq)>::MaxBands) {
                return error_result(DeviceError::MaxPeqBandsReached);
            }

            peq.bands.emplace_back();
            band_result = peq.bands.back();
            band_index = peq.bands.size() - 1;

            return changed_result();
        });
    }

    MutationResult delete_channel_peq_band(ChannelTarget target, size_t band_index) {
        return with_channel(target, [&](auto &channel) {
            auto& bands = channel.peq.bands;

            if (band_index >= bands.size()) {
                return error_result(DeviceError::InvalidPeqBand);
            }

            bands.erase(bands.begin() + band_index);

            return changed_result();
        });
    }
}
