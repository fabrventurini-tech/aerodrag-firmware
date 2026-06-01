idf_component_register(
    INCLUDE_DIRS "."
    REQUIRES
        driver
        bt
        freertos
        esp_common
        nvs_flash
        esp_timer
        esp_wifi
        esp_netif
        espressif__qrcode
)
