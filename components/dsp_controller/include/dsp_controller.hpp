#pragma once

#include "esp_err.h"

namespace dsp_controller {
    esp_err_t start();

    bool is_ready();
}
