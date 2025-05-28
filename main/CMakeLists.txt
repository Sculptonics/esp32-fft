idf_component_register(SRCS "u8g2_esp32_hal.c" "test_fft.c"
                    ##PRIV_REQUIRES esp_driver_i2s fatfs driver
                    INCLUDE_DIRS "$ENV{IDF_PATH}/examples/peripherals/i2s/common" ".")