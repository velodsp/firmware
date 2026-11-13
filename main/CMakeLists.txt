idf_component_register(
        SRCS "main.cpp"
        INCLUDE_DIRS "."
        REQUIRES
            status_led
            wifi
            web_server
            discovery
            device_state
            adau1452
)
