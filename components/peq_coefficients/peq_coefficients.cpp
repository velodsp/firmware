#include "peq_coefficients.hpp"

#include <cmath>

namespace dsp {
    bool calculate_peaking_eq(
        const double sample_rate_hz,
        const double frequency_hz,
        const double gain_db,
        const double q,
        BiquadCoefficients &out
    ) {
        if (!std::isfinite(sample_rate_hz) ||
            !std::isfinite(frequency_hz) ||
            !std::isfinite(gain_db) ||
            !std::isfinite(q) ||
            sample_rate_hz <= 0.0 ||
            frequency_hz <= 0.0 ||
            frequency_hz >= sample_rate_hz * 0.5 ||
            q <= 0.0) {
            return false;
        }

        constexpr double Pi = 3.141592653589793238462643383279502884;

        const double A = std::pow(10.0, gain_db / 40.0);
        if (!std::isfinite(A) || A <= 0.0) {
            return false;
        }

        const double omega = 2.0 * Pi * frequency_hz / sample_rate_hz;
        const double sine = std::sin(omega);
        const double cosine = std::cos(omega);
        const double alpha = sine / (2.0 * q);

        const double a0 = 1.0 + alpha / A;
        if (!std::isfinite(a0) || a0 == 0.0) {
            return false;
        }

        BiquadCoefficients result{};
        result.b0 = (1.0 + alpha * A) / a0;
        result.b1 = (-2.0 * cosine) / a0;
        result.b2 = (1.0 - alpha * A) / a0;
        result.a1 = (-2.0 * cosine) / a0;
        result.a2 = (1.0 - alpha / A) / a0;

        out = result;
        return true;
    }

    bool encode_signed_8_24(const double value, uint32_t &out) noexcept {
        if (!std::isfinite(value)) {
            return false;
        }

        constexpr double Scale = 16777216.0; // 2^24
        const double scaled = std::round(value * Scale);

        constexpr double min_raw = std::numeric_limits<std::int32_t>::min();
        constexpr double max_raw = std::numeric_limits<std::int32_t>::max();

        if (!std::isfinite(scaled) || scaled < min_raw || scaled > max_raw) {
            return false;
        }

        const auto signed_raw = static_cast<std::int32_t>(scaled);
        out = static_cast<std::uint32_t>(signed_raw);
        return true;
    }

    bool encode_sigmastudio_general_eq(
        const BiquadCoefficients &coefficients,
        SigmaStudioBiquadWords &out
    ) noexcept {
        const std::array ordered = {
            coefficients.b2,
            coefficients.b1,
            coefficients.b0,
            -coefficients.a2,
            -coefficients.a1
        };

        SigmaStudioBiquadWords result{};
        for (std::size_t i = 0; i < ordered.size(); ++i) {
            if (!encode_signed_8_24(ordered[i], result[i])) {
                return false;
            }
        }

        out = result;
        return true;
    }

    bool make_peaking_eq_words(
        const double sample_rate_hz,
        const double frequency_hz,
        const double gain_db,
        const double q,
        SigmaStudioBiquadWords &out
    ) noexcept {
        BiquadCoefficients coefficients{};
        if (!calculate_peaking_eq(
            sample_rate_hz, frequency_hz, gain_db, q, coefficients)) {
            return false;
        }

        return encode_sigmastudio_general_eq(coefficients, out);
    }
}
