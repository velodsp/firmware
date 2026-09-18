#include "web_server.hpp"

#include <vector>

#include "esp_check.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "protocol.hpp"

static auto TAG = "web_server";

static httpd_handle_t s_server = nullptr;

static esp_err_t websocket_connected(httpd_req_t *req) {
    ESP_LOGI(TAG, "WebSocket connected");
    return protocol::send_hello(req);
}

static esp_err_t websocket_handler(httpd_req_t *req) {
    httpd_ws_frame_t frame = {};

    ESP_RETURN_ON_ERROR(
        httpd_ws_recv_frame(req, &frame, 0),
        TAG,
        "Failed to get frame length"
    );

    if (frame.type != HTTPD_WS_TYPE_TEXT) {
        ESP_LOGW(TAG, "Unsupported WebSocket frame type: %d", frame.type);
        return ESP_OK;
    }

    auto *buffer = static_cast<char *>(malloc(frame.len + 1));
    if (buffer == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    frame.payload = reinterpret_cast<uint8_t *>(buffer);

    esp_err_t err = httpd_ws_recv_frame(req, &frame, frame.len);

    if (err == ESP_OK) {
        buffer[frame.len] = '\0';

        ESP_LOGD(TAG, "Received WebSocket message: %.*s", static_cast<int>(frame.len), buffer);

        err = protocol::handle_message(req, buffer, frame.len);
    }

    free(buffer);

    return err;
}

esp_err_t web_server_start() {
    if (s_server != nullptr) {
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.lru_purge_enable = true;

    esp_err_t err = httpd_start(&s_server, &config);
    if (err != ESP_OK) {
        return err;
    }

    static constexpr httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = websocket_handler,
        .user_ctx = nullptr,
        .is_websocket = true,
        .handle_ws_control_frames = false,
        .supported_subprotocol = nullptr,
        .ws_post_handshake_cb = websocket_connected
    };

    err = httpd_register_uri_handler(s_server, &ws_uri);
    if (err != ESP_OK) {
        httpd_stop(s_server);
        s_server = nullptr;
        return err;
    }

    ESP_LOGI(TAG, "WebSocket server started");

    return ESP_OK;
}

void web_server_stop() {
    if (s_server != nullptr) {
        httpd_stop(s_server);
        s_server = nullptr;
    }
}
