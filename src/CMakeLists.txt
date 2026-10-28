idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        esp_driver_gpio
        esp_driver_uart
        esp_driver_ledc
        esp_driver_spi
        esp_driver_sdspi
        esp_driver_sdmmc
        esp_timer
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        fatfs
        sdmmc
        mbedtls
)
