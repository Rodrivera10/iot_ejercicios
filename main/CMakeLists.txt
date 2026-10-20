idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES freertos esp_timer esp_driver_spi esp_driver_gpio esp_driver_ledc esp_adc esp_wifi esp_event esp_netif nvs_flash mqtt rc522 cjson)