idf_component_register(
    INCLUDE_DIRS "include"
    SRC_DIRS "src/aes" "src/hal" "src/lmic"
    REQUIRES driver esp_timer
)