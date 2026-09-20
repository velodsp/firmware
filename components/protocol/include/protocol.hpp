#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

namespace protocol {

    esp_err_t handle_message(httpd_req_t *req, const char *data, size_t len);
    esp_err_t send_hello(httpd_req_t *req);

}
