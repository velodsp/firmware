idf_component_register(
        SRCS "adau1452.cpp"
        INCLUDE_DIRS "include"
        REQUIRES
            esp_driver_gpio
            esp_driver_spi
            freertos
)
