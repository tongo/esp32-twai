idf_component_register(
    SRCS "main.cpp" "can_manager.cpp"
    PRIV_REQUIRES esp_driver_twai esp_timer
    INCLUDE_DIRS "."
)