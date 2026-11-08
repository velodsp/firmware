#include "protocol.hpp"

#include <array>
#include <cmath>
#include <optional>
#include <limits>
#include <cstring>
#include <string_view>
#include <cinttypes>
#include <memory>

#include "cJSON.h"
#include "esp_log.h"

#include "device_state.hpp"

namespace protocol {
    namespace {
        constexpr char TAG[] = "protocol";

        using JsonPtr = std::unique_ptr<cJSON, decltype(&cJSON_Delete)>;

        JsonPtr make_json(cJSON *json) {
            return JsonPtr(json, &cJSON_Delete);
        }

#define JSON_CHECK_NULL(expr) \
    do { \
        if (!(expr)) \
            return nullptr; \
    } while (0)

#define JSON_CHECK_ERR(expr) \
    do { \
        if (!(expr)) \
            return ESP_ERR_NO_MEM; \
    } while (0)

        enum class JsonFieldError {
            Ok,
            Missing,
            WrongType,
            NotFinite,
            NotInteger,
            OutOfRange,
            InvalidValue,
            InvalidChannel
        };

        struct JsonFieldResult {
            JsonFieldError error;
            const char *field;
        };

        const char *filter_type_to_string(device::FilterType type) {
            switch (type) {
                case device::FilterType::PEAK:
                    return "peak";

                default:
                    return "unknown";
            }
        }

        std::optional<device::FilterType> filter_type_from_string(const std::string_view value) {
            if (value == "peak") {
                return device::FilterType::PEAK;
            }

            return std::nullopt;
        }

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

        JsonFieldError json_get_optional_float(const cJSON *root, const char *name, std::optional<float> *out) {
            const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, name);

            if (item == nullptr) {
                out->reset();
                return JsonFieldError::Ok;
            }

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

        JsonFieldError json_get_optional_bool(const cJSON *root, const char *name, std::optional<bool> *out) {
            const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, name);

            if (item == nullptr) {
                out->reset();
                return JsonFieldError::Ok;
            }

            if (!cJSON_IsBool(item)) return JsonFieldError::WrongType;

            *out = cJSON_IsTrue(item);
            return JsonFieldError::Ok;
        }

        JsonFieldError json_get_optional_filter_type(const cJSON *root, const char *name,
                                                     std::optional<device::FilterType> *out) {
            const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, name);

            if (item == nullptr) {
                out->reset();
                return JsonFieldError::Ok;
            }

            if (!cJSON_IsString(item)) {
                return JsonFieldError::WrongType;
            }

            const auto type = filter_type_from_string(item->valuestring);

            if (!type.has_value()) {
                return JsonFieldError::InvalidValue;
            }

            *out = *type;
            return JsonFieldError::Ok;
        }

        cJSON *serialize_peq_band(const device::PeqBand *band) {
            auto obj = make_json(cJSON_CreateObject());
            JSON_CHECK_NULL(obj);

            JSON_CHECK_NULL(cJSON_AddBoolToObject(obj.get(), "enabled", band->enabled));
            JSON_CHECK_NULL(cJSON_AddNumberToObject(obj.get(), "frequency_hz", band->frequency_hz));
            JSON_CHECK_NULL(cJSON_AddNumberToObject(obj.get(), "gain_db", band->gain_db));
            JSON_CHECK_NULL(cJSON_AddNumberToObject(obj.get(), "q", band->q));
            JSON_CHECK_NULL(cJSON_AddStringToObject(obj.get(), "type", filter_type_to_string(band->type)));

            return obj.release();
        }

        cJSON *serialize_output(const device::OutputState *output) {
            auto obj = make_json(cJSON_CreateObject());
            JSON_CHECK_NULL(obj);

            JSON_CHECK_NULL(cJSON_AddNumberToObject(obj.get(), "gain_db", output->gain_db));
            JSON_CHECK_NULL(cJSON_AddBoolToObject(obj.get(), "muted", output->muted));

            cJSON *peq = cJSON_AddObjectToObject(obj.get(), "peq");
            JSON_CHECK_NULL(peq);

            JSON_CHECK_NULL(cJSON_AddBoolToObject(peq, "enabled", output->peq.enabled));

            cJSON *bands = cJSON_AddArrayToObject(peq, "bands");
            JSON_CHECK_NULL(bands);

            for (size_t i = 0; i < output->peq.band_count; ++i) {
                auto band = make_json(serialize_peq_band(&output->peq.bands[i]));
                JSON_CHECK_NULL(band);

                JSON_CHECK_NULL(cJSON_AddItemToArray(bands, band.get()));
                band.release();
            }

            return obj.release();
        }

        cJSON *serialize_input(const device::InputState *input) {
            auto obj = make_json(cJSON_CreateObject());
            JSON_CHECK_NULL(obj);

            JSON_CHECK_NULL(cJSON_AddNumberToObject(obj.get(), "gain_db", input->gain_db));
            JSON_CHECK_NULL(cJSON_AddBoolToObject(obj.get(), "muted", input->muted));

            cJSON *peq = cJSON_AddObjectToObject(obj.get(), "peq");
            JSON_CHECK_NULL(peq);

            JSON_CHECK_NULL(cJSON_AddBoolToObject(peq, "enabled", input->peq.enabled));

            cJSON *bands = cJSON_AddArrayToObject(peq, "bands");
            JSON_CHECK_NULL(bands);
            for (size_t i = 0; i < input->peq.band_count; ++i) {
                auto band = make_json(serialize_peq_band(&input->peq.bands[i]));
                JSON_CHECK_NULL(band);

                JSON_CHECK_NULL(cJSON_AddItemToArray(bands, band.get()));
                band.release();
            }

            return obj.release();
        }

        cJSON *serialize_dsp_config(const device::DspConfig *config) {
            auto root = make_json(cJSON_CreateObject());
            JSON_CHECK_NULL(root);

            cJSON *inputs = cJSON_AddArrayToObject(root.get(), "inputs");
            JSON_CHECK_NULL(inputs);

            for (const auto &i: config->inputs) {
                auto input = make_json(serialize_input(&i));
                JSON_CHECK_NULL(input);

                JSON_CHECK_NULL(cJSON_AddItemToArray(inputs, input.get()));
                input.release();
            }

            cJSON *outputs = cJSON_AddArrayToObject(root.get(), "outputs");
            JSON_CHECK_NULL(outputs);

            for (const auto &i: config->outputs) {
                auto output = make_json(serialize_output(&i));
                JSON_CHECK_NULL(output);

                JSON_CHECK_NULL(cJSON_AddItemToArray(outputs, output.get()));
                output.release();
            }

            return root.release();
        }

        cJSON *serialize_device_state(const device::State &state) {
            auto root = make_json(cJSON_CreateObject());
            JSON_CHECK_NULL(root);

            JSON_CHECK_NULL(cJSON_AddNumberToObject(root.get(), "active_preset", state.active_preset));
            JSON_CHECK_NULL(cJSON_AddBoolToObject(root.get(), "preset_modified", state.preset_modified));

            auto config = make_json(serialize_dsp_config(&state.dsp));
            JSON_CHECK_NULL(config);

            JSON_CHECK_NULL(cJSON_AddItemToObject(root.get(), "dsp", config.get()));
            config.release();

            return root.release();
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
            auto root = make_json(cJSON_CreateObject());
            JSON_CHECK_ERR(root);

            if (request_id.has_value()) {
                JSON_CHECK_ERR(cJSON_AddNumberToObject(root.get(), "id", request_id.value()));
            }

            JSON_CHECK_ERR(cJSON_AddStringToObject(root.get(), "type", "error"));

            cJSON *error = cJSON_AddObjectToObject(root.get(), "error");
            JSON_CHECK_ERR(error);

            JSON_CHECK_ERR(cJSON_AddStringToObject(error, "code", code));

            if (field != nullptr)
                JSON_CHECK_ERR(cJSON_AddStringToObject(error, "field", field));

            if (message != nullptr)
                JSON_CHECK_ERR(cJSON_AddStringToObject(error, "message", message));

            return send_json(req, root.get());
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
                case JsonFieldError::InvalidValue:
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
            auto root = make_json(cJSON_CreateObject());
            JSON_CHECK_ERR(root);

            JSON_CHECK_ERR(cJSON_AddNumberToObject(root.get(), "id", request_id));
            JSON_CHECK_ERR(cJSON_AddStringToObject(root.get(), "type", "ok"));
            JSON_CHECK_ERR(cJSON_AddNumberToObject(root.get(), "revision", revision));

            return send_json(req, root.get());
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
            auto change = make_json(cJSON_CreateObject());
            JSON_CHECK_NULL(change);

            JSON_CHECK_NULL(cJSON_AddStringToObject(change.get(), "type", type));
            JSON_CHECK_NULL(cJSON_AddStringToObject(change.get(), "channel_type", channel_kind_to_string(target.kind)));
            JSON_CHECK_NULL(cJSON_AddNumberToObject(change.get(), "channel", target.index));

            return change.release();
        }

        cJSON *create_preset_modified_change() {
            auto change = make_json(cJSON_CreateObject());
            JSON_CHECK_NULL(change);

            JSON_CHECK_NULL(cJSON_AddStringToObject(change.get(), "type", "preset_modified"));
            JSON_CHECK_NULL(cJSON_AddBoolToObject(change.get(), "value", true));

            return change.release();
        }

        template<typename... Changes>
        esp_err_t broadcast_state_update(httpd_handle_t server, uint32_t revision, Changes... changes) {
            auto root = make_json(cJSON_CreateObject());
            JSON_CHECK_ERR(root);

            JSON_CHECK_ERR(cJSON_AddStringToObject(root.get(), "type", "state_update"));
            JSON_CHECK_ERR(cJSON_AddNumberToObject(root.get(), "revision", revision));

            cJSON *changes_json = cJSON_AddArrayToObject(root.get(), "changes");
            JSON_CHECK_ERR(changes_json);

            auto add_change = [changes_json](JsonPtr &change) {
                if (!change || !cJSON_AddItemToArray(changes_json, change.get())) {
                    return false;
                }

                change.release();
                return true;
            };

            JSON_CHECK_ERR((add_change(changes) && ...));

            return broadcast_json(server, root.get());
        }

        cJSON *create_channel_gain_change(device::ChannelTarget target, float gain_db) {
            auto change = make_json(create_channel_change("channel_gain", target));
            JSON_CHECK_NULL(change);

            JSON_CHECK_NULL(cJSON_AddNumberToObject(change.get(), "gain_db", gain_db));
            return change.release();
        }

        esp_err_t broadcast_channel_gain_update(httpd_handle_t server, const uint32_t revision,
                                                device::ChannelTarget target, const float gain_db) {
            return broadcast_state_update(server, revision,
                                          make_json(create_channel_gain_change(target, gain_db)),
                                          make_json(create_preset_modified_change())
            );
        }

        cJSON *create_channel_mute_change(device::ChannelTarget target, bool muted) {
            auto change = make_json(create_channel_change("channel_muted", target));
            JSON_CHECK_NULL(change);

            JSON_CHECK_NULL(cJSON_AddBoolToObject(change.get(), "muted", muted));
            return change.release();
        }

        esp_err_t broadcast_channel_mute_update(httpd_handle_t server, const uint32_t revision,
                                                device::ChannelTarget target, const bool muted) {
            return broadcast_state_update(server, revision,
                                          make_json(create_channel_mute_change(target, muted)),
                                          make_json(create_preset_modified_change())
            );
        }

        cJSON *create_channel_peq_enabled_change(device::ChannelTarget target, bool enabled) {
            auto change = make_json(create_channel_change("channel_peq_enabled", target));
            JSON_CHECK_NULL(change);

            JSON_CHECK_NULL(cJSON_AddBoolToObject(change.get(), "enabled", enabled));
            return change.release();
        }

        esp_err_t broadcast_channel_peq_enabled_update(httpd_handle_t server, const uint32_t revision,
                                                       device::ChannelTarget target, const bool enabled) {
            return broadcast_state_update(server, revision,
                                          make_json(create_channel_peq_enabled_change(target, enabled)),
                                          make_json(create_preset_modified_change())
            );
        }

        cJSON *create_channel_peq_band_event(const char *event_type, device::ChannelTarget target, uint32_t band_index,
                                             device::PeqBand band) {
            auto change = make_json(create_channel_change(event_type, target));
            JSON_CHECK_NULL(change);

            JSON_CHECK_NULL(cJSON_AddNumberToObject(change.get(), "band", band_index));

            cJSON *value = cJSON_AddObjectToObject(change.get(), "value");
            JSON_CHECK_NULL(value);

            JSON_CHECK_NULL(cJSON_AddBoolToObject(value, "enabled", band.enabled));
            JSON_CHECK_NULL(cJSON_AddNumberToObject(value, "frequency_hz", band.frequency_hz));
            JSON_CHECK_NULL(cJSON_AddNumberToObject(value, "q", band.q));
            JSON_CHECK_NULL(cJSON_AddNumberToObject(value, "gain_db", band.gain_db));
            JSON_CHECK_NULL(cJSON_AddStringToObject(value, "type", filter_type_to_string(band.type)));

            return change.release();
        }

        esp_err_t broadcast_channel_peq_band_change_update(httpd_handle_t server, const uint32_t revision,
                                                           device::ChannelTarget target, uint32_t band_index,
                                                           const device::PeqBand &band) {
            return broadcast_state_update(server, revision,
                                          make_json(create_channel_peq_band_event(
                                              "channel_peq_band_changed", target, band_index, band)),
                                          make_json(create_preset_modified_change())
            );
        }

        esp_err_t broadcast_channel_peq_band_add_update(httpd_handle_t server, const uint32_t revision,
                                                        device::ChannelTarget target, uint32_t band_index,
                                                        const device::PeqBand &band) {
            return broadcast_state_update(server, revision,
                                          make_json(create_channel_peq_band_event(
                                              "channel_peq_band_added", target, band_index, band)),
                                          make_json(create_preset_modified_change())
            );
        }

        cJSON *create_channel_peq_band_deletion(device::ChannelTarget target, uint32_t band_index) {
            auto change = make_json(create_channel_change("channel_peq_band_deleted", target));
            JSON_CHECK_NULL(change);

            JSON_CHECK_NULL(cJSON_AddNumberToObject(change.get(), "band", band_index));

            return change.release();
        }

        esp_err_t broadcast_channel_peq_band_delete_update(httpd_handle_t server, const uint32_t revision,
                                                           device::ChannelTarget target, uint32_t band_index) {
            return broadcast_state_update(server, revision,
                                          make_json(create_channel_peq_band_deletion(target, band_index)),
                                          make_json(create_preset_modified_change())
            );
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

        esp_err_t handle_set_channel_peq_enabled(httpd_req_t *req, const uint32_t request_id, const cJSON *root) {
            device::ChannelTarget target{};

            const auto target_result = json_get_channel_target(root, &target);

            if (target_result.error != JsonFieldError::Ok) {
                return send_field_error(req, request_id, target_result.field, target_result.error);
            }

            bool enabled;

            if (const auto field_err = json_get_bool(root, "enabled", &enabled); field_err != JsonFieldError::Ok) {
                return send_field_error(req, request_id, "enabled", field_err);
            }

            const auto result = device::set_channel_peq_enabled(target, enabled);

            const esp_err_t response_err = send_ok(req, request_id, result.revision);

            if (result.changed) {
                log_broadcast_error(
                    broadcast_channel_peq_enabled_update(
                        req->handle, result.revision, target, enabled
                    )
                );
            }

            return response_err;
        }

        esp_err_t handle_set_channel_peq_band(httpd_req_t *req, const uint32_t request_id, const cJSON *root) {
            device::ChannelTarget target{};

            const auto target_result = json_get_channel_target(root, &target);

            if (target_result.error != JsonFieldError::Ok) {
                return send_field_error(req, request_id, target_result.field, target_result.error);
            }

            uint32_t band_index;

            if (const auto field_err = json_get_uint32(root, "band", &band_index); field_err != JsonFieldError::Ok) {
                return send_field_error(req, request_id, "band", field_err);
            }

            device::PeqBandPatch patch{};

            if (const auto field_err = json_get_optional_bool(root, "enabled", &patch.enabled);
                field_err != JsonFieldError::Ok) {
                return send_field_error(req, request_id, "enabled", field_err);
            }
            if (const auto field_err = json_get_optional_float(root, "frequency_hz", &patch.frequency_hz);
                field_err != JsonFieldError::Ok) {
                return send_field_error(req, request_id, "frequency_hz", field_err);
            }
            if (const auto field_err = json_get_optional_float(root, "q", &patch.q); field_err != JsonFieldError::Ok) {
                return send_field_error(req, request_id, "q", field_err);
            }
            if (const auto field_err = json_get_optional_float(root, "gain_db", &patch.gain_db);
                field_err != JsonFieldError::Ok) {
                return send_field_error(req, request_id, "gain_db", field_err);
            }
            if (const auto field_err = json_get_optional_filter_type(root, "filter_type", &patch.type);
                field_err != JsonFieldError::Ok) {
                return send_field_error(req, request_id, "filter_type", field_err);
            }

            device::PeqBand band_result{};

            const auto result = device::set_channel_peq_band(target, band_index, patch, band_result);

            switch (result.error) {
                case device::DeviceError::Ok:
                    break;

                case device::DeviceError::InvalidPeqBand:
                    return send_error(req, request_id, "invalid_peq_band", "band");

                case device::DeviceError::InvalidPeqFrequency:
                    return send_error(req, request_id, "invalid_frequency", "frequency_hz");

                case device::DeviceError::PeqFrequencyOutOfRange:
                    return send_error(req, request_id, "out_of_range", "frequency_hz");

                case device::DeviceError::InvalidPeqQ:
                    return send_error(req, request_id, "invalid_q", "q");

                case device::DeviceError::PeqQOutOfRange:
                    return send_error(req, request_id, "out_of_range", "q");

                case device::DeviceError::InvalidPeqGain:
                    return send_error(req, request_id, "invalid_gain", "gain_db");

                case device::DeviceError::PeqGainOutOfRange:
                    return send_error(req, request_id, "out_of_range", "gain_db");

                default:
                    return send_error(req, request_id, "internal_error");
            }

            const esp_err_t response_err = send_ok(req, request_id, result.revision);

            if (result.changed) {
                log_broadcast_error(
                    broadcast_channel_peq_band_change_update(
                        req->handle, result.revision, target, band_index, band_result
                    )
                );
            }

            return response_err;
        }

        esp_err_t handle_add_channel_peq_band(httpd_req_t *req, const uint32_t request_id, const cJSON *root) {
            device::ChannelTarget target{};

            const auto target_result = json_get_channel_target(root, &target);

            if (target_result.error != JsonFieldError::Ok) {
                return send_field_error(req, request_id, target_result.field, target_result.error);
            }

            device::PeqBand band_result{};
            size_t band_index;

            const auto result = device::add_channel_peq_band(target, band_result, band_index);

            switch (result.error) {
                case device::DeviceError::Ok:
                    break;
                case device::DeviceError::MaxPeqBandsReached:
                    return send_error(req, request_id, "max_peq_bands");
                default:
                    return send_error(req, request_id, "internal_error");
            }

            const esp_err_t response_err = send_ok(req, request_id, result.revision);

            if (result.changed) {
                log_broadcast_error(
                    broadcast_channel_peq_band_add_update(
                        req->handle, result.revision, target, band_index, band_result
                    )
                );
            }

            return response_err;
        }

        esp_err_t handle_delete_channel_peq_band(httpd_req_t *req, const uint32_t request_id, const cJSON *root) {
            device::ChannelTarget target{};

            const auto target_result = json_get_channel_target(root, &target);

            if (target_result.error != JsonFieldError::Ok) {
                return send_field_error(req, request_id, target_result.field, target_result.error);
            }

            uint32_t band_index;

            if (const auto field_err = json_get_uint32(root, "band", &band_index); field_err != JsonFieldError::Ok) {
                return send_field_error(req, request_id, "band", field_err);
            }

            const auto result = device::delete_channel_peq_band(target, band_index);

            switch (result.error) {
                case device::DeviceError::Ok:
                    break;

                case device::DeviceError::InvalidPeqBand:
                    return send_error(req, request_id, "invalid_peq_band", "band");

                default:
                    return send_error(req, request_id, "internal_error");
            }

            const esp_err_t response_err = send_ok(req, request_id, result.revision);

            if (result.changed) {
                log_broadcast_error(
                    broadcast_channel_peq_band_delete_update(
                        req->handle, result.revision, target, band_index
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
            ProtocolCommand{"set_channel_muted", handle_set_channel_muted},
            ProtocolCommand{"set_channel_peq_enabled", handle_set_channel_peq_enabled},
            ProtocolCommand{"set_channel_peq_band", handle_set_channel_peq_band},
            ProtocolCommand{"add_channel_peq_band", handle_add_channel_peq_band},
            ProtocolCommand{"delete_channel_peq_band", handle_delete_channel_peq_band}
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
        auto root = make_json(cJSON_CreateObject());
        JSON_CHECK_ERR(root);

        JSON_CHECK_ERR(cJSON_AddStringToObject(root.get(), "type", "hello"));
        JSON_CHECK_ERR(cJSON_AddNumberToObject(root.get(), "protocol_version", 1));
        JSON_CHECK_ERR(cJSON_AddStringToObject(root.get(), "firmware_version", "0.1.0"));
        JSON_CHECK_ERR(cJSON_AddNumberToObject(root.get(), "revision", device::get_revision()));

        auto state_json = make_json(serialize_device_state(device::get_state()));
        JSON_CHECK_ERR(state_json);

        JSON_CHECK_ERR(cJSON_AddItemToObject(root.get(), "state", state_json.get()));
        state_json.release();

        return send_json(req, root.get());
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
