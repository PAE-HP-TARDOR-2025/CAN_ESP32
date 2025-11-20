idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES canopennode esp_timer driver log freertos
                    )