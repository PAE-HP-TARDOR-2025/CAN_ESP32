idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES canopennode efuse esp_timer driver nvs_flash log freertos spiffs
                    )