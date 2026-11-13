#include <cstdint>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "status_led.h"
#include "web_server.hpp"
#include "wifi.h"
#include "discovery.h"
#include "device_state.hpp"
#include "adau1452.hpp"

#include "freertos/FreeRTOS.h"

#define RGB_LED_GPIO 38

static const char *TAG = "main";

static void wifi_status_changed(wifi_status_t status) {
    switch (status) {
        case WIFI_STATUS_CONNECTED:
            status_led_set_hsv(120, 255, 10);

            // components are idempotent
            ESP_ERROR_CHECK(discovery_start());
            ESP_ERROR_CHECK(web_server_start());

            break;

        case WIFI_STATUS_DISCONNECTED:
            status_led_set_hsv(0, 255, 20);
            break;
    }
}

extern "C" void app_main(void) {
    esp_err_t err;

    adau1452::Config dsp_config{};

    dsp_config.host = SPI2_HOST;

    dsp_config.mosi_gpio = GPIO_NUM_11;
    dsp_config.miso_gpio = GPIO_NUM_13;
    dsp_config.sclk_gpio = GPIO_NUM_12;
    dsp_config.cs_gpio = GPIO_NUM_10;

    dsp_config.selfboot_wait_ms = 100;

    ESP_ERROR_CHECK(adau1452::init(dsp_config));
    ESP_ERROR_CHECK(device::init());

    err = status_led_init(RGB_LED_GPIO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize status LED: %s", esp_err_to_name(err));
        return;
    }

    err = wifi_init_sta();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Wi-Fi: %s", esp_err_to_name(err));

        status_led_set_hsv(0, 255, 20);
        return;
    }

    wifi_set_status_callback(wifi_status_changed);

    // ESP_ERROR_CHECK(wifi_set_credentials("ABC", "123"));

    err = wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Wi-Fi connection: %s", esp_err_to_name(err));

        status_led_set_hsv(0, 255, 20);
        return;
    }
}
