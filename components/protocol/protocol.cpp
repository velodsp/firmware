#include "protocol.hpp"

#include <array>
#include <cmath>
#include <optional>
#include <limits>
#include <cstring>
#include <string_view>
#include <cinttypes>

#include "cJSON.h"
#include "esp_log.h"

#include "device_state.hpp"

namespace protocol {
    namespace {
        constexpr char TAG[] = "protocol";

        enum class JsonFieldError {
            Ok,
            Missing,
            WrongType,
            NotFinite,
            NotInteger,
            OutOfRange,
            InvalidChannel
        };

        struct JsonFieldResult {
            JsonFieldError error;
            const char *field;
        };

        JsonFieldError json_get_int(const cJSON *root, const char *name, int *out) {
            const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, name);

            if (item == nullptr) return JsonFieldError::Missing;

            if (!cJSON_IsNumber(item)) return JsonFieldError::WrongType;

            const double value = item->valuedouble;

            if (!std::isfinite(value)) return JsonFieldError::NotFinite;

            if (std::floor(value) != value) return JsonFieldError::NotInteger;

            if (value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max()) {
                return JsonFieldError::OutOfRange;
            }

            *out = static_cast<int>(value);
            return JsonFieldError::Ok;
        }

        JsonFieldError json_get_float(const cJSON *root, const char *name, float *out) {
            const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, name);

            if (item == nullptr) return JsonFieldError::Missing;

            if (!cJSON_IsNumber(item)) return JsonFieldError::WrongType;

            const double value = item->valuedouble;

            if (!std::isfinite(value)) return JsonFieldError::NotFinite;

            if (value < std::numeric_limits<float>::lowest() || value > std::numeric_limits<float>::max()) {
                return JsonFieldError::OutOfRange;
            }

            *out = static_cast<float>(value);
            return JsonFieldError::Ok;
        }

        JsonFieldError json_get_uint32(const cJSON *root, const char *name, uint32_t *out) {
            const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, name);

            if (item == nullptr) return JsonFieldError::Missing;

            if (!cJSON_IsNumber(item)) return JsonFieldError::WrongType;

            const double value = item->valuedouble;

            if (!std::isfinite(value)) return JsonFieldError::NotFinite;

            if (std::floor(value) != value) return JsonFieldError::NotInteger;

            if (value < 0 || value > std::numeric_limits<uint32_t>::max()) {
                return JsonFieldError::OutOfRange;
            }

            *out = static_cast<uint32_t>(value);
            return JsonFieldError::Ok;
        }

        JsonFieldError json_get_bool(const cJSON *root, const char *name, bool *out) {
            const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, name);

            if (item == nullptr) return JsonFieldError::Missing;

            if (!cJSON_IsBool(item)) return JsonFieldError::WrongType;

            *out = cJSON_IsTrue(item);
            return JsonFieldError::Ok;
        }

        const char *filter_type_to_string(device::FilterType type) {
            switch (type) {
                case device::FilterType::PEAK:
                    return "peak";

                default:
                    return "unknown";
            }
        }

        cJSON *serialize_peq_band(const device::PeqBand *band) {
            cJSON *obj = cJSON_CreateObject();
            if (obj == nullptr) return nullptr;

            cJSON_AddBoolToObject(obj, "enabled", band->enabled);
            cJSON_AddNumberToObject(obj, "frequency_hz", band->frequency_hz);
            cJSON_AddNumberToObject(obj, "gain_db", band->gain_db);
            cJSON_AddNumberToObject(obj, "q", band->q);
            cJSON_AddStringToObject(obj, "type", filter_type_to_string(band->type));

            return obj;
        }

        cJSON *serialize_output(const device::OutputState *output) {
            cJSON *obj = cJSON_CreateObject();
            if (obj == nullptr) return nullptr;

            cJSON_AddNumberToObject(obj, "gain_db", output->gain_db);
            cJSON_AddBoolToObject(obj, "muted", output->muted);

            cJSON *peq = cJSON_AddArrayToObject(obj, "peq");
            if (peq == nullptr) {
                cJSON_Delete(obj);
                return nullptr;
            }
            for (const auto &i: output->peq) {
                cJSON *band = serialize_peq_band(&i);

                if (band == nullptr) {
                    cJSON_Delete(obj);
                    return nullptr;
                }

                cJSON_AddItemToArray(peq, band);
            }

            return obj;
        }

        cJSON *serialize_input(const device::InputState *input) {
            cJSON *obj = cJSON_CreateObject();
            if (obj == nullptr) return nullptr;

            cJSON_AddNumberToObject(obj, "gain_db", input->gain_db);
            cJSON_AddBoolToObject(obj, "muted", input->muted);

            cJSON *peq = cJSON_AddArrayToObject(obj, "peq");
            if (peq == nullptr) {
                cJSON_Delete(obj);
                return nullptr;
            }
            for (const auto &i: input->peq) {
                cJSON *band = serialize_peq_band(&i);

                if (band == nullptr) {
                    cJSON_Delete(obj);
                    return nullptr;
                }

                cJSON_AddItemToArray(peq, band);
            }

            return obj;
        }

        cJSON *serialize_dsp_config(const device::DspConfig *config) {
            cJSON *root = cJSON_CreateObject();
            if (root == nullptr) return nullptr;

            cJSON *inputs = cJSON_AddArrayToObject(root, "inputs");
            if (inputs == nullptr) {
                cJSON_Delete(root);
                return nullptr;
            }
            for (const auto &i: config->inputs) {
                cJSON *input = serialize_input(&i);
                if (input == nullptr) {
                    cJSON_Delete(root);
                    return nullptr;
                }

                cJSON_AddItemToArray(inputs, input);
            }

            cJSON *outputs = cJSON_AddArrayToObject(root, "outputs");
            if (outputs == nullptr) {
                cJSON_Delete(root);
                return nullptr;
            }
            for (const auto &i: config->outputs) {
                cJSON *output = serialize_output(&i);
                if (output == nullptr) {
                    cJSON_Delete(root);
                    return nullptr;
                }

                cJSON_AddItemToArray(outputs, output);
            }

            return root;
        }

        cJSON *serialize_device_state(const device::State &state) {
            cJSON *root = cJSON_CreateObject();
            if (root == nullptr) return nullptr;

            cJSON_AddNumberToObject(root, "active_preset", state.active_preset);
            cJSON_AddBoolToObject(root, "preset_modified", state.preset_modified);

            cJSON *config = serialize_dsp_config(&state.dsp);
            if (config == nullptr) {
                cJSON_Delete(root);
                return nullptr;
            }

            cJSON_AddItemToObject(root, "dsp", config);

            return root;
        }

        esp_err_t send_json(httpd_req_t *req, const cJSON *json) {
            char *payload = cJSON_PrintUnformatted(json);
            if (payload == nullptr) {
                return ESP_ERR_NO_MEM;
            }

            httpd_ws_frame_t frame = {
                .final = true,
                .fragmented = false,
                .type = HTTPD_WS_TYPE_TEXT,
                .payload = reinterpret_cast<uint8_t *>(payload),
                .len = strlen(payload)
            };

            const esp_err_t err = httpd_ws_send_frame(req, &frame);

            cJSON_free(payload);

            return err;
        }

        esp_err_t broadcast_json(httpd_handle_t server, const cJSON *json) {
            char *payload = cJSON_PrintUnformatted(json);

            if (payload == nullptr) {
                return ESP_ERR_NO_MEM;
            }

            httpd_ws_frame_t frame = {
                .final = true,
                .fragmented = false,
                .type = HTTPD_WS_TYPE_TEXT,
                .payload = reinterpret_cast<uint8_t *>(payload),
                .len = strlen(payload)
            };

            std::array<int, CONFIG_LWIP_MAX_SOCKETS> client_fds{};

            size_t client_count = client_fds.size();

            const esp_err_t list_err = httpd_get_client_list(server, &client_count, client_fds.data());

            if (list_err != ESP_OK) {
                cJSON_free(payload);
                return list_err;
            }

            esp_err_t result = ESP_OK;

            for (size_t i = 0; i < client_count; ++i) {
                const int fd = client_fds[i];

                if (httpd_ws_get_fd_info(server, fd) != HTTPD_WS_CLIENT_WEBSOCKET) {
                    continue;
                }

                const esp_err_t err = httpd_ws_send_frame_async(server, fd, &frame);

                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to broadcast to fd=%d: %s", fd, esp_err_to_name(err));
                    result = err;
                }
            }

            cJSON_free(payload);
            return result;
        }

        esp_err_t send_error(httpd_req_t *req, std::optional<uint32_t> request_id, const char *code,
                             const char *field = nullptr,
                             const char *message = nullptr) {
            cJSON *root = cJSON_CreateObject();
            if (root == nullptr) {
                return ESP_ERR_NO_MEM;
            }

            if (request_id.has_value()) {
                cJSON_AddNumberToObject(root, "id", request_id.value());
            }

            cJSON_AddStringToObject(root, "type", "error");

            cJSON *error = cJSON_AddObjectToObject(root, "error");
            if (error == nullptr) {
                cJSON_Delete(root);
                return ESP_ERR_NO_MEM;
            }

            cJSON_AddStringToObject(error, "code", code);

            if (field != nullptr) cJSON_AddStringToObject(error, "field", field);

            if (message != nullptr) cJSON_AddStringToObject(error, "message", message);

            const esp_err_t err = send_json(req, root);
            cJSON_Delete(root);

            return err;
        }

        esp_err_t send_field_error(httpd_req_t *req, const std::optional<uint32_t> request_id, const char *field,
                                   const JsonFieldError error) {
            switch (error) {
                case JsonFieldError::Missing:
                    return send_error(req, request_id, "missing_argument", field);

                case JsonFieldError::WrongType:
                    return send_error(req, request_id, "wrong_type", field);

                case JsonFieldError::NotFinite:
                case JsonFieldError::OutOfRange:
                    return send_error(req, request_id, "invalid_value", field);

                case JsonFieldError::NotInteger:
                    return send_error(req, request_id, "not_integer", field);

                case JsonFieldError::InvalidChannel:
                    return send_error(req, request_id, "invalid_channel", field);

                case JsonFieldError::Ok:
                    break;
            }

            return ESP_ERR_INVALID_STATE;
        }

        esp_err_t send_ok(httpd_req_t *req, uint32_t request_id, uint32_t revision) {
            cJSON *root = cJSON_CreateObject();
            if (root == nullptr) {
                return ESP_ERR_NO_MEM;
            }

            cJSON_AddNumberToObject(root, "id", request_id);
            cJSON_AddStringToObject(root, "type", "ok");
            cJSON_AddNumberToObject(root, "revision", revision);

            const esp_err_t err = send_json(req, root);
            cJSON_Delete(root);

            return err;
        }

        JsonFieldResult json_get_channel_target(const cJSON *root, device::ChannelTarget *out) {
            const cJSON *type = cJSON_GetObjectItemCaseSensitive(root, "channel_type");

            if (type == nullptr) {
                return {.error = JsonFieldError::Missing, .field = "channel_type"};
            }

            if (!cJSON_IsString(type)) {
                return {.error = JsonFieldError::WrongType, .field = "channel_type"};
            }

            device::ChannelKind kind;

            if (strcmp(type->valuestring, "input") == 0) {
                kind = device::ChannelKind::Input;
            } else if (strcmp(type->valuestring, "output") == 0) {
                kind = device::ChannelKind::Output;
            } else {
                return {JsonFieldError::InvalidChannel, "channel_type"};
            }

            uint32_t index;

            const auto index_error = json_get_uint32(root, "channel", &index);

            if (index_error != JsonFieldError::Ok) {
                return {index_error, "channel"};
            }

            *out = {
                .kind = kind,
                .index = index
            };

            return {JsonFieldError::Ok, nullptr};
        }

        const char *channel_kind_to_string(device::ChannelKind kind) {
            switch (kind) {
                case device::ChannelKind::Input:
                    return "input";

                case device::ChannelKind::Output:
                    return "output";
            }

            return "unknown";
        }

        cJSON *create_channel_change(const char *type, device::ChannelTarget target) {
            cJSON *change = cJSON_CreateObject();
            if (change == nullptr) return nullptr;

            cJSON_AddStringToObject(change, "type", type);
            cJSON_AddStringToObject(change, "channel_type", channel_kind_to_string(target.kind));
            cJSON_AddNumberToObject(change, "channel", target.index);

            return change;
        }

        cJSON *create_preset_modified_change() {
            cJSON *change = cJSON_CreateObject();
            if (change == nullptr) return nullptr;

            cJSON_AddStringToObject(change, "type", "preset_modified");
            cJSON_AddBoolToObject(change, "value", true);

            return change;
        }

        esp_err_t broadcast_state_update(httpd_handle_t server, uint32_t revision,
                                         std::initializer_list<cJSON *> changes) {
            cJSON *root = cJSON_CreateObject();
            if (root == nullptr) {
                for (cJSON *change: changes) {
                    cJSON_Delete(change);
                }
                return ESP_ERR_NO_MEM;
            }

            cJSON_AddStringToObject(root, "type", "state_update");
            cJSON_AddNumberToObject(root, "revision", revision);

            cJSON *changes_json = cJSON_AddArrayToObject(root, "changes");

            if (changes_json == nullptr) {
                cJSON_Delete(root);

                for (cJSON *change: changes) {
                    cJSON_Delete(change);
                }

                return ESP_ERR_NO_MEM;
            }

            for (cJSON *change: changes) {
                if (change == nullptr) {
                    cJSON_Delete(root);
                    return ESP_ERR_NO_MEM;
                }

                cJSON_AddItemToArray(changes_json, change);
            }

            const esp_err_t err = broadcast_json(server, root);
            cJSON_Delete(root);

            return err;
        }

        cJSON *create_channel_gain_change(device::ChannelTarget target, float gain_db) {
            cJSON *change = create_channel_change("channel_gain", target);

            if (change == nullptr) {
                return nullptr;
            }

            cJSON_AddNumberToObject(change, "gain_db", gain_db);
            return change;
        }

        cJSON *create_channel_mute_change(device::ChannelTarget target, bool muted) {
            cJSON *change = create_channel_change("channel_muted", target);

            if (change == nullptr) {
                return nullptr;
            }

            cJSON_AddBoolToObject(change, "muted", muted);
            return change;
        }

        esp_err_t broadcast_channel_gain_update(httpd_handle_t server, const uint32_t revision,
                                                device::ChannelTarget target, const float gain_db) {
            return broadcast_state_update(server, revision, {
                                              create_channel_gain_change(target, gain_db),
                                              create_preset_modified_change()
                                          });
        }

        esp_err_t broadcast_channel_mute_update(httpd_handle_t server, const uint32_t revision,
                                                device::ChannelTarget target, const bool muted) {
            return broadcast_state_update(server, revision, {
                                              create_channel_mute_change(target, muted),
                                              create_preset_modified_change()
                                          });
        }

        void log_broadcast_error(esp_err_t err) {
            if (err != ESP_OK) {
                ESP_LOGW(
                    TAG,
                    "State update broadcast failed: %s",
                    esp_err_to_name(err)
                );
            }
        }

        esp_err_t handle_set_channel_gain(httpd_req_t *req, const uint32_t request_id, const cJSON *root) {
            device::ChannelTarget target{};

            const auto target_result = json_get_channel_target(root, &target);

            if (target_result.error != JsonFieldError::Ok) {
                return send_field_error(req, request_id, target_result.field, target_result.error);
            }

            float gain_db;
            const auto gain_err = json_get_float(root, "gain_db", &gain_db);

            if (gain_err != JsonFieldError::Ok) {
                return send_field_error(req, request_id, "gain_db", gain_err);
            }

            const auto result = device::set_channel_gain(target, gain_db);

            switch (result.error) {
                case device::DeviceError::Ok:
                    break;

                case device::DeviceError::InvalidChannel:
                    return send_error(req, request_id, "out_of_range", "channel", "channel does not exist");

                case device::DeviceError::GainOutOfRange:
                    return send_error(req, request_id, "out_of_range", "gain_db");

                case device::DeviceError::InvalidGain:
                    return send_error(req, request_id, "invalid_gain", "gain_db");

                default:
                    return send_error(req, request_id, "internal_error");
            }

            const esp_err_t response_err = send_ok(req, request_id, result.revision);

            if (result.changed) {
                log_broadcast_error(
                    broadcast_channel_gain_update(
                        req->handle, result.revision, target, gain_db
                    )
                );
            }

            return response_err;
        }

        esp_err_t handle_set_channel_muted(httpd_req_t *req, const uint32_t request_id, const cJSON *root) {
            device::ChannelTarget target{};

            const auto target_result = json_get_channel_target(root, &target);

            if (target_result.error != JsonFieldError::Ok) {
                return send_field_error(req, request_id, target_result.field, target_result.error);
            }

            bool muted;
            const auto muted_err = json_get_bool(root, "muted", &muted);

            if (muted_err != JsonFieldError::Ok) {
                return send_field_error(req, request_id, "muted", muted_err);
            }

            const auto result = device::set_channel_muted(target, muted);

            const esp_err_t response_err = send_ok(req, request_id, result.revision);

            if (result.changed) {
                log_broadcast_error(
                    broadcast_channel_mute_update(
                        req->handle, result.revision, target, muted
                    )
                );
            }

            return response_err;
        }

        using ProtocolHandler = esp_err_t (*)(
            httpd_req_t *req,
            uint32_t request_id,
            const cJSON *root
        );

        struct ProtocolCommand {
            std::string_view type;
            ProtocolHandler handler;
        };

        constexpr std::array COMMANDS{
            ProtocolCommand{"set_channel_gain", handle_set_channel_gain},
            ProtocolCommand{"set_channel_muted", handle_set_channel_muted}
        };

        esp_err_t dispatch_command(httpd_req_t *req, uint32_t request_id, const cJSON *root, std::string_view type) {
            for (const auto &command: COMMANDS) {
                if (command.type == type) {
                    ESP_LOGD(TAG, "Request id=%" PRIu32 " type=%.*s", request_id, static_cast<int>(type.size()),
                             type.data());
                    return command.handler(req, request_id, root);
                }
            }

            ESP_LOGW(TAG, "Unknown command: %.*s", static_cast<int>(type.size()), type.data());
            return send_error(req, request_id, "unknown_command");
        }
    }

    esp_err_t send_hello(httpd_req_t *req) {
        cJSON *root = cJSON_CreateObject();
        if (root == nullptr) {
            return ESP_ERR_NO_MEM;
        }

        cJSON_AddStringToObject(root, "type", "hello");
        cJSON_AddNumberToObject(root, "protocol_version", 1);
        cJSON_AddStringToObject(root, "firmware_version", "0.1.0");

        cJSON_AddNumberToObject(root, "revision", device::get_revision());
        cJSON *state_json = serialize_device_state(device::get_state());
        if (state_json == nullptr) {
            cJSON_Delete(root);
            return ESP_ERR_NO_MEM;
        }
        cJSON_AddItemToObject(root, "state", state_json);

        const esp_err_t err = send_json(req, root);

        cJSON_Delete(root);

        return err;
    }

    esp_err_t handle_message(httpd_req_t *req, const char *data, size_t len) {
        cJSON *root = cJSON_ParseWithLength(data, len);

        if (root == nullptr) {
            ESP_LOGW(TAG, "Received invalid JSON");
            return send_error(req, std::nullopt, "invalid_json");
        }

        const cJSON *type = cJSON_GetObjectItemCaseSensitive(root, "type");

        if (type == nullptr) {
            cJSON_Delete(root);
            return send_error(req, std::nullopt, "missing_argument", "type");
        }

        if (!cJSON_IsString(type)) {
            cJSON_Delete(root);
            return send_error(req, std::nullopt, "wrong_type", "type");
        }

        uint32_t request_id = 0;

        const auto id_err =
                json_get_uint32(root, "id", &request_id);

        if (id_err != JsonFieldError::Ok) {
            cJSON_Delete(root);
            return send_field_error(req, std::nullopt, "id", id_err);
        }

        const esp_err_t err = dispatch_command(req, request_id, root, type->valuestring);

        cJSON_Delete(root);

        return err;
    }
}
