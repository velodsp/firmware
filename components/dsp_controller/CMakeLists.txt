idf_component_register(
        SRCS "dsp_controller.cpp"
        INCLUDE_DIRS "include"
        REQUIRES
            adau1452
            freertos
            device_state
)
