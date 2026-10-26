#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "esp_err.h"

namespace device {
    constexpr size_t InputCount = 4;
    constexpr size_t OutputCount = 8;
    constexpr size_t InputPeqBands = 16;
    constexpr size_t OutputPeqBands = 24;
    constexpr uint8_t InvalidPreset = std::numeric_limits<uint8_t>::max();
    constexpr float ChannelGainMinDb = -80.0f;
    constexpr float ChannelGainMaxDb = 12.0f;
    constexpr float PeqFrequencyMinHz = 10.0f;
    constexpr float PeqFrequencyMaxHz = 20000.0f;
    constexpr float PeqQMin = 0.1f;
    constexpr float PeqQMax = 51.0f;
    constexpr float PeqGainDbMin = -24.0f;
    constexpr float PeqGainDbMax = 24.0f;

    enum class DeviceError {
        Ok,
        InvalidChannel,
        GainOutOfRange,
        InvalidGain,
        InvalidPeqBand,
        InvalidPeqFrequency,
        PeqFrequencyOutOfRange,
        InvalidPeqQ,
        PeqQOutOfRange,
        InvalidPeqGain,
        PeqGainOutOfRange,
        InvalidPeqFilter,
        MaxPeqBandsReached
    };

    enum class FilterType {
        PEAK,
    };

    struct PeqBand {
        bool enabled = true;
        float frequency_hz = 1000.0f;
        float gain_db = 0.0f;
        float q = 1.0f;
        FilterType type = FilterType::PEAK;
    };

    enum class ChannelKind {
        Input,
        Output
    };

    struct ChannelTarget {
        ChannelKind kind;
        size_t index;
    };

    template<size_t MaxPeqBandCount>
    struct Peq {
        static constexpr size_t MaxBands = MaxPeqBandCount;

        bool enabled = false;
        std::vector<PeqBand> bands{};
    };

    template<size_t PeqBandCount>
    struct ChannelState {
        float gain_db = 0.0f;
        bool muted = false;

        Peq<PeqBandCount> peq{};
    };

    struct InputState : ChannelState<InputPeqBands> {
    };

    struct OutputState : ChannelState<OutputPeqBands> {
    };

    struct DspConfig {
        std::array<InputState, InputCount> inputs{};
        std::array<OutputState, OutputCount> outputs{};
    };

    struct Preset {
        char name[32]{};
        DspConfig config{};
    };

    struct State {
        DspConfig dsp{};

        uint8_t active_preset = InvalidPreset;
        bool preset_modified = false;
    };

    struct MutationResult {
        DeviceError error;
        bool changed;
        uint32_t revision;
    };

    struct PeqBandPatch {
        std::optional<bool> enabled;
        std::optional<float> frequency_hz;
        std::optional<float> q;
        std::optional<float> gain_db;
        std::optional<FilterType> type;
    };

    const State &get_state();

    uint32_t get_revision();

    MutationResult set_channel_gain(ChannelTarget target, float gain_db);
    MutationResult set_channel_muted(ChannelTarget target, bool muted);
    MutationResult set_channel_peq_enabled(ChannelTarget target, bool enabled);
    MutationResult set_channel_peq_band(ChannelTarget target, size_t band_index, const PeqBandPatch &patch, PeqBand &band_result);
    MutationResult add_channel_peq_band(ChannelTarget target, PeqBand &band_result, size_t &band_index);
    MutationResult delete_channel_peq_band(ChannelTarget target, size_t band_index);
}
