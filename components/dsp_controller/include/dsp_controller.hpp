#pragma once

#include "esp_err.h"

namespace dsp_controller {
    esp_err_t start();

    bool is_ready();

    void notify_state_changed();

    void request_full_sync();
}
