#pragma once

#include <array>
#include <cstdint>

namespace dsp {
    struct BiquadCoefficients {
        double b0 = 1.0;
        double b1 = 0.0;
        double b2 = 0.0;
        double a1 = 0.0;
        double a2 = 0.0;
    };

    // B2, B1, B0, A2, A1; denominator coefficients are negated on encode
    using SigmaStudioBiquadWords = std::array<uint32_t, 5>;

    constexpr uint32_t BiquadBypassWords[5] = {
        0x00000000,
        0x00000000,
        0x01000000,
        0x00000000,
        0x00000000
    };

    // RBJ audio EQ Cookbook peaking-EQ biquad.
    // q uses the RBJ peaking-EQ convention
    bool calculate_peaking_eq(
        double sample_rate_hz,
        double frequency_hz,
        double gain_db,
        double q,
        BiquadCoefficients &out
    );

    bool encode_signed_8_24(double value, uint32_t &out) noexcept;

    // encodes RBJ coefficients into SigmaStudio General EQ order: B2, B1, B0, -A2, -A1
    bool encode_sigmastudio_general_eq(const BiquadCoefficients &coefficients, SigmaStudioBiquadWords &out) noexcept;

    bool make_peaking_eq_words(
        double sample_rate_hz,
        double frequency_hz,
        double gain_db,
        double q,
        SigmaStudioBiquadWords &out
    ) noexcept;
}
