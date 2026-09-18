#pragma once

#include <array>
#include <cstdint>
#include <limits>

#include "esp_err.h"

namespace device {
    constexpr size_t InputCount = 4;
    constexpr size_t OutputCount = 8;
    constexpr size_t PeqBands = 10;
    constexpr uint8_t InvalidPreset = std::numeric_limits<uint8_t>::max();

    enum class DeviceError {
        Ok,
        InvalidOutput,
        GainOutOfRange,
        InvalidGain,
    };

    enum class FilterType {
        PEAK,
    };

    struct PeqBand {
        bool enabled;
        float frequency_hz;
        float gain_db;
        float q;
        FilterType type;
    };

    struct InputState {
        float gain_db;
        bool muted;

        std::array<PeqBand, PeqBands> peq;
    };

    struct OutputState {
        float gain_db;
        bool muted;

        std::array<PeqBand, PeqBands> peq;
    };

    struct DspConfig {
        std::array<InputState, InputCount> inputs;
        std::array<OutputState, OutputCount> outputs;
    };

    struct Preset {
        char name[32];
        DspConfig config;
    };

    struct State {
        DspConfig dsp;

        uint8_t active_preset;
        bool preset_modified;
    };

    struct MutationResult {
        DeviceError error;
        bool changed;
        uint32_t revision;
    };

    const State& get_state();
    uint32_t get_revision();

    esp_err_t init();

    MutationResult set_output_gain(size_t output, float gain_db);
}
