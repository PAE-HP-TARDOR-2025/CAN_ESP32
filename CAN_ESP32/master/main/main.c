#include <stdio.h>
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_spiffs.h"

// Incluimos la cabecera de TU lógica de Master
#include "CANopen_LSS_Master.h"

// --- CONFIGURACIÓN DE HARDWARE MASTER ---
// Ajusta este pin según tu placa. En muchas placas CAN es el GPIO 16.
// Si tu placa no tiene pin de "Enable" o "Standby", puedes ignorar esto.
#define GPIO_CAN_ENABLE  GPIO_NUM_16 

static const char *TAG = "MAIN_MASTER";

// Configuración inicial de pines físicos
void setup_hardware_master()
{
    ESP_LOGI(TAG, "Configurando hardware del Master...");
    
    // Encender el Transceptor CAN (Chip SN65HVD230 o similar)
    // Sin esto, el bus da error "TX FAILED" o "Passive".
    gpio_reset_pin(GPIO_CAN_ENABLE);
    gpio_set_direction(GPIO_CAN_ENABLE, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_CAN_ENABLE, 1); // 1 = Enable (Generalmente)
}

void app_main(void)
{
    // 1. Inicializar NVS (Memoria No Volátil)
    // Obligatorio para CANopenNode (guarda parámetros y configuración LSS)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS corrupta o versión nueva. Borrando y reiniciando...");
        nvs_flash_erase();
        nvs_flash_init();
    }

    // 2. Configurar pines
    setup_hardware_master();

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "   ARRANCANDO ESP32 CANOPEN MASTER      ");
    ESP_LOGI(TAG, "========================================");

    // 3. Montar SPIFFS para acceder a la imagen de firmware (ej. /spiffs/slave.bin)
    const esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "storage",  // Especificar la partición SPIFFS
        .max_files = 4,
        .format_if_mount_failed = false,
    };
    esp_err_t spiffs_err = esp_vfs_spiffs_register(&conf);
    if (spiffs_err != ESP_OK) {
        ESP_LOGE(TAG, "ERROR: SPIFFS no montado (0x%x). OTA no funcionará.", (unsigned)spiffs_err);
    } else {
        size_t total = 0, used = 0;
        if (esp_spiffs_info(conf.partition_label, &total, &used) == ESP_OK) {
            ESP_LOGI(TAG, "SPIFFS montado OK: total=%u used=%u", (unsigned)total, (unsigned)used);
            
            // Verificar que slave.bin existe
            FILE *f = fopen("/spiffs/slave.bin", "rb");
            if (f) {
                fseek(f, 0, SEEK_END);
                long size = ftell(f);
                fclose(f);
                ESP_LOGI(TAG, "Firmware slave.bin encontrado: %ld bytes", size);
            } else {
                ESP_LOGE(TAG, "ERROR: /spiffs/slave.bin NO encontrado!");
            }
        }
    }

    // 4. Ejecutar la lógica del Master (crea tareas y retorna)
    CO_ESP32_Master_Run();

    // 5. Bucle infinito del Main
    // El trabajo real lo hacen las tareas creadas arriba.
    // Aquí solo dejamos un log para saber que el chip no se ha colgado.
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(5000)); // Cada 5 segundos
        //ESP_LOGI(TAG, "Master (CPU 0) sigue vivo...");
    }
}

