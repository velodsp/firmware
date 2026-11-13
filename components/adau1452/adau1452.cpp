#include "include/adau1452.hpp"

#include <array>
#include <cstddef>
#include <cstring>
#include <limits>

#include "freertos/FreeRTOS.h"

namespace adau1452 {
    namespace {
        constexpr const char *TAG = "adau1452";

        // Datasheet page 61 table: CORE_STATUS
        constexpr uint16_t CoreStatusAddress = 0xF405;

        constexpr int InitialSpiClockHz = 300000;

        // SPI frame:
        //   1 byte command
        //   2 bytes subaddress
        //   up to 5 * 4 bytes of parameter data
        constexpr size_t MaxPayloadBytes = MaxWordsPerWrite * 4;
        constexpr size_t MaxFrameBytes = 3 + MaxPayloadBytes;

        spi_device_handle_t s_device = nullptr;
        bool s_bus_initialized = false;
        spi_host_device_t s_host{};

        void delay_ms(const uint32_t ms) {
            if (ms == 0) return;

            auto ticks = pdMS_TO_TICKS(ms);

            if (ticks == 0) ticks = 1;

            vTaskDelay(ticks);
        }

        void cleanup() {
            if (s_device != nullptr) {
                spi_bus_remove_device(s_device);
                s_device = nullptr;
            }

            if (s_bus_initialized) {
                spi_bus_free(s_host);
                s_bus_initialized = false;
            }
        }

        esp_err_t transfer(const uint8_t *tx, const size_t length, uint8_t *rx = nullptr) {
            if (s_device == nullptr || tx == nullptr || length == 0) {
                return ESP_ERR_INVALID_STATE;
            }

            spi_transaction_t transaction{};

            transaction.length = length * 8;
            transaction.tx_buffer = tx;

            if (rx != nullptr) {
                transaction.rxlength = length * 8;
                transaction.rx_buffer = rx;
            }

            return spi_device_transmit(s_device, &transaction);
        }

        // reference equivalent: adau1452_write_block()
        // reference frame: 0x00, address MSB, address LSB, payload...
        esp_err_t write_bytes(uint16_t address, const uint8_t *payload, size_t count) {
            if (payload == nullptr || count == 0 || count > MaxPayloadBytes) return ESP_ERR_INVALID_ARG;

            alignas(4) std::array<uint8_t, MaxFrameBytes> tx{};

            tx[0] = 0x00; // SPI write command
            tx[1] = static_cast<uint8_t>(address >> 8);
            tx[2] = static_cast<uint8_t>(address & 0xFF);

            std::memcpy(tx.data() + 3, payload, count);

            return transfer(tx.data(), 3 + count);
        }

        // reference equivalent: adau1452_read_word()
        // the first three received bytes are not returned to the caller
        // the requested data follows the command and 16-bit subaddress
        esp_err_t read_bytes(uint16_t address, uint8_t *out, size_t count) {
            if (out == nullptr || count == 0 || count > 4) {
                return ESP_ERR_INVALID_ARG;
            }

            alignas(4) std::array<uint8_t, 7> tx{};
            alignas(4) std::array<uint8_t, 7> rx{};

            tx[0] = 0x01; // SPI read command
            tx[1] = static_cast<uint8_t>(address >> 8);
            tx[2] = static_cast<uint8_t>(address & 0xFF);

            // the remaining zero bytes generate the clocks needed to receive the requested data bytes
            const size_t frame_length = 3 + count;

            esp_err_t err = transfer(tx.data(), frame_length, rx.data());

            if (err != ESP_OK) return err;

            std::memcpy(out, rx.data() + 3, count);

            return ESP_OK;
        }

        esp_err_t wait_for_core() {
            // 001 = running normally; 011 = sleep mode.
            constexpr uint32_t timeout_ms = 1000;
            constexpr uint32_t PollIntervalMs = 10;

            uint32_t elapsed_ms = 0;

            while (elapsed_ms <= timeout_ms) {
                uint16_t status = 0;

                esp_err_t err = read_control_register(CoreStatusAddress, status);

                if (err != ESP_OK) return err;

                if ((status & 0x0001U) != 0) return ESP_OK;

                if (elapsed_ms == timeout_ms) break;

                const uint32_t remaining = timeout_ms - elapsed_ms;
                const uint32_t delay = std::min(PollIntervalMs, remaining);

                delay_ms(delay);
                elapsed_ms += delay;
            }

            return ESP_ERR_TIMEOUT;
        }
    }

    esp_err_t init(const Config &config) {
        if (s_device != nullptr) {
            return ESP_OK;
        }

        if (config.sclk_gpio < 0
            || config.mosi_gpio < 0
            || config.miso_gpio < 0
            || config.cs_gpio < 0
            || config.selfboot_wait_ms == 0
            || config.core_ready_timeout_ms == 0) {
            return ESP_ERR_INVALID_ARG;
        }

        delay_ms(config.selfboot_wait_ms);

        spi_bus_config_t bus_config{};

        bus_config.sclk_io_num = config.sclk_gpio;
        bus_config.mosi_io_num = config.mosi_gpio;
        bus_config.miso_io_num = config.miso_gpio;
        bus_config.quadwp_io_num = -1;
        bus_config.quadhd_io_num = -1;
        bus_config.max_transfer_sz = MaxFrameBytes;

        s_host = config.host;

        esp_err_t err = spi_bus_initialize(config.host, &bus_config, SPI_DMA_DISABLED);

        if (err != ESP_OK) {
            return err;
        }

        s_bus_initialized = true;

        spi_device_interface_config_t device_config{};

        device_config.clock_speed_hz = InitialSpiClockHz;
        device_config.mode = 3;
        device_config.spics_io_num = config.cs_gpio;
        device_config.queue_size = 1;

        err = spi_bus_add_device(config.host, &device_config, &s_device);

        if (err != ESP_OK) {
            cleanup();
            return err;
        }

        // reference equivalent: adau1452_initialize(), the SPI init loop
        for (int i = 0; i < 3; ++i) {
            constexpr uint8_t dummy_byte = 0x00;

            err = transfer(&dummy_byte, 1);

            if (err != ESP_OK) {
                cleanup();
                return err;
            }

            esp_rom_delay_us(2);
        }

        // reference: adau1452_initialize(), CORE_STATUS check
        err = wait_for_core();

        if (err != ESP_OK) {
            cleanup();
            return err;
        }

        return ESP_OK;
    }

    esp_err_t write_control_register(uint16_t address, uint16_t value) {
        const uint8_t bytes[2] = {
            static_cast<uint8_t>(value >> 8),
            static_cast<uint8_t>(value & 0xFF)
        };

        return write_bytes(address, bytes, sizeof(bytes));
    }


    esp_err_t read_control_register(uint16_t address, uint16_t &value) {
        uint8_t bytes[2]{};

        esp_err_t err = read_bytes(address, bytes, sizeof(bytes));

        if (err != ESP_OK) {
            return err;
        }

        value = (static_cast<uint16_t>(bytes[0]) << 8) |
                static_cast<uint16_t>(bytes[1]);

        return ESP_OK;
    }

    esp_err_t write_parameter(uint16_t address, uint32_t value) {
        const uint32_t words[] = {value};

        return write_parameters(address, words, 1);
    }

    esp_err_t write_parameters(uint16_t first_address, const uint32_t *values, size_t word_count) {
        if (values == nullptr ||
            word_count == 0 ||
            word_count > MaxWordsPerWrite) {
            return ESP_ERR_INVALID_ARG;
        }

        if (static_cast<uint32_t>(first_address) + word_count - 1 >
            std::numeric_limits<uint16_t>::max()) {
            return ESP_ERR_INVALID_ARG;
        }

        // each ADAU1452 parameter RAM word is 32 bits
        // SPI data is sent most-significant byte first
        uint8_t payload[MaxPayloadBytes]{};

        for (size_t i = 0; i < word_count; ++i) {
            const uint32_t value = values[i];
            const size_t offset = i * 4;

            payload[offset + 0] = static_cast<uint8_t>(value >> 24);
            payload[offset + 1] = static_cast<uint8_t>(value >> 16);
            payload[offset + 2] = static_cast<uint8_t>(value >> 8);
            payload[offset + 3] = static_cast<uint8_t>(value);
        }

        return write_bytes(first_address, payload, word_count * 4);
    }

    esp_err_t read_parameter(uint16_t address, uint32_t &value) {
        uint8_t bytes[4]{};

        esp_err_t err = read_bytes(address, bytes, sizeof(bytes));

        if (err != ESP_OK) {
            return err;
        }

        value = (static_cast<uint32_t>(bytes[0]) << 24) |
                (static_cast<uint32_t>(bytes[1]) << 16) |
                (static_cast<uint32_t>(bytes[2]) << 8) |
                static_cast<uint32_t>(bytes[3]);

        return ESP_OK;
    }

    esp_err_t write_safeload(uint16_t target_address, const uint32_t *words, size_t word_count,
                             uint32_t sample_rate_hz) {
        if (words == nullptr || word_count == 0 || word_count > MaxSafeloadWords || sample_rate_hz == 0) {
            return ESP_ERR_INVALID_ARG;
        }

        // write data words into safeload slots
        esp_err_t err = write_parameters(DataSafeload0Address, words, word_count);
        if (err != ESP_OK) return err;

        // write the destination parameter address
        err = write_parameter(AddressSafeloadAddress, target_address);
        if (err != ESP_OK) return err;

        // write NUM_SAFELOAD last (triggers the safeload operation)
        err = write_parameter(NumSafeloadAddress, word_count);
        if (err != ESP_OK) return err;

        // allow at least one audio frame for the safeload to execute
        const auto frame_time_us = static_cast<uint32_t>((1000000ULL + sample_rate_hz - 1) / sample_rate_hz);
        esp_rom_delay_us(frame_time_us);

        return ESP_OK;
    }
}
