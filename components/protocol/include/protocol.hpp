#pragma once

#include "device_state.hpp"
#include "esp_err.h"
#include "esp_http_server.h"

namespace protocol {

    struct LevelSnapshot {
        std::array<float, device::InputCount> inputs{};
        std::array<float, device::OutputCount> outputs{};
    };

    esp_err_t handle_message(httpd_req_t *req, const char *data, size_t len);
    esp_err_t send_hello(httpd_req_t *req);
    esp_err_t broadcast_heartbeat(httpd_handle_t server);

    void set_http_server(httpd_handle_t server);
    void publish_levels(const LevelSnapshot &levels);

}
