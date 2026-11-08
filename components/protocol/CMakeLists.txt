idf_component_register(
        SRCS "protocol.cpp"
        INCLUDE_DIRS "include"
        REQUIRES
            cjson
            esp_http_server
            device_state
            esp_timer
)
