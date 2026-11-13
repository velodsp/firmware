#pragma once

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

namespace adau1452 {
    struct Config {
        spi_host_device_t host;

        gpio_num_t sclk_gpio;
        gpio_num_t mosi_gpio;
        gpio_num_t miso_gpio;
        gpio_num_t cs_gpio;

        uint32_t selfboot_wait_ms;

        // after entering SPI mode, poll CORE_STATUS for this long
        uint32_t core_ready_timeout_ms = 1000;
    };

    esp_err_t init(const Config &config);

    esp_err_t read_control_register(uint16_t address, uint16_t &value);

    esp_err_t write_control_register(uint16_t address, uint16_t value);

    esp_err_t read_parameter(uint16_t address, uint32_t &value);

    esp_err_t write_parameter(uint16_t address, uint32_t value);

    constexpr size_t MaxWordsPerWrite = 5;

    esp_err_t write_parameters(uint16_t first_address, const uint32_t *values, size_t word_count);

    // these can differ depending on compiler version
    constexpr uint16_t DataSafeload0Address = 0x0014;
    constexpr uint16_t AddressSafeloadAddress = 0x0019;
    constexpr uint16_t NumSafeloadAddress = 0x001A;

    constexpr size_t MaxSafeloadWords = 5;

    esp_err_t write_safeload(uint16_t target_address, const uint32_t *words, size_t word_count,
                             uint32_t sample_rate_hz);
}
