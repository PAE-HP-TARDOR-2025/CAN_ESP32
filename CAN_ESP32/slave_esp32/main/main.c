#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "CO_driver.h"
#include "CO_NMT_Heartbeat.h"
#include "CANopen.h"
#include "CO_LSSslave.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_efuse.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "OD.h"
#include "CO_storage.h"
#include "CANopenNode_ESP32.h"


// --- LA CORRECCIÓN DEL ERROR ---
// Aunque hayamos puesto los includes arriba, si el archivo .h de la librería
// no tiene escrita explícitamente esta función, fallará. 
// Dejamos esta línea aquí para asegurar al 100% que compila.
bool CO_ESP32_init(void); 

static const char *TAG = "APP_MAIN";

// ---------------------------------------------------------
// DEFINICIÓN DE PINES
// ---------------------------------------------------------
// Pin ENABLE del Transceptor (Hardware necesario)
#define GPIO_CAN_ENABLE     GPIO_NUM_16 

// Botón BOOT (GPIO 0) para simular Emergencia
#define GPIO_BOTON_BOOT     GPIO_NUM_0  

// ---------------------------------------------------------
// SETUP
// ---------------------------------------------------------
void setup_hardware_externo()
{
    ESP_LOGI(TAG, "Configurando hardware externo...");

    // 1. Activar Transceptor CAN
    gpio_reset_pin(GPIO_CAN_ENABLE);
    gpio_set_direction(GPIO_CAN_ENABLE, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_CAN_ENABLE, 1); // 1 = Chip Activo
    ESP_LOGI(TAG, "Transceptor CAN ON (GPIO %d)", GPIO_CAN_ENABLE);

    // 2. Configurar Botón BOOT
    gpio_reset_pin(GPIO_BOTON_BOOT);
    gpio_set_direction(GPIO_BOTON_BOOT, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_BOTON_BOOT, GPIO_PULLUP_ONLY);
    ESP_LOGI(TAG, "Botón BOOT listo (GPIO %d)", GPIO_BOTON_BOOT);
}

// ---------------------------------------------------------
// MAIN
// ---------------------------------------------------------
void app_main(void)
{
    // Inicializar Flash (NVS)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    setup_hardware_externo();

    ESP_LOGI(TAG, "--- INICIANDO CANOPEN ---");

    // Iniciar la librería (Llama a la función declarada manualmente arriba)
    if (CO_ESP32_init()) 
    {
        ESP_LOGI(TAG, "CANopen iniciado. Pulsa BOOT (GPIO 0) para Emergencia.");
    }
    else 
    {
        ESP_LOGE(TAG, "Fallo al iniciar CANopen.");
    }

    // Bucle para mantener vivo el main
    while(1) 
    {
        vTaskDelay(pdMS_TO_TICKS(10000)); 
    }
}