#pragma once

#include <array>
#include <cstdint>
#include <limits>

#include "esp_err.h"

namespace device {
    constexpr size_t InputCount = 4;
    constexpr size_t OutputCount = 8;
    constexpr size_t InputPeqBands = 16;
    constexpr size_t OutputPeqBands = 24;
    constexpr uint8_t InvalidPreset = std::numeric_limits<uint8_t>::max();

    enum class DeviceError {
        Ok,
        InvalidChannel,
        GainOutOfRange,
        InvalidGain,
    };

    enum class FilterType {
        PEAK,
    };

    struct PeqBand {
        bool enabled = false;
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

    template<size_t PeqBandCount>
    struct ChannelState {
        float gain_db = 0.0f;
        bool muted = false;

        std::array<PeqBand, PeqBandCount> peq{};
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

    const State &get_state();

    uint32_t get_revision();

    MutationResult set_channel_gain(ChannelTarget target, float gain_db);
}
