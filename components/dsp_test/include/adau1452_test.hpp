#pragma once

#include "esp_err.h"
#include <cstddef>

namespace adau1452_test {
    esp_err_t test_unmute_outputs();
    esp_err_t test_set_outputs_muted(bool muted);
}
