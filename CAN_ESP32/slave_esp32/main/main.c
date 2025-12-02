#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/gpio.h" 

// --- AÑADIDO: NECESARIO PARA vTaskDelay y pdMS_TO_TICKS ---
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Incluimos tu cabecera
#include "CANopenNode_ESP32.h"

static const char *TAG = "APP_MAIN";

// ---------------------------------------------------------
// DEFINICIÓN DE PINES
// ---------------------------------------------------------
#define GPIO_CAN_ENABLE     GPIO_NUM_16 

// ---------------------------------------------------------
// SETUP
// ---------------------------------------------------------
void setup_hardware_externo()
{
    ESP_LOGI(TAG, "Configurando hardware externo...");

    // Activar Transceptor CAN (Solo si tu placa lo necesita)
    gpio_reset_pin(GPIO_CAN_ENABLE);
    gpio_set_direction(GPIO_CAN_ENABLE, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_CAN_ENABLE, 1); // 1 = Chip Activo
    ESP_LOGI(TAG, "Transceptor CAN ON (GPIO %d)", GPIO_CAN_ENABLE);
}

// ---------------------------------------------------------
// MAIN
// ---------------------------------------------------------
void app_main(void)
{
    // 1. Inicializar Flash
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Configurar hardware auxiliar
    setup_hardware_externo();

    ESP_LOGI(TAG, "--- INICIANDO CANOPEN ---");

    // 3. Arrancar la librería CANopen
    if (CO_ESP32_init()) 
    {
        ESP_LOGI(TAG, "Sistema Arrancado. Tarea CANopen corriendo en Core 1.");
    }
    else 
    {
        ESP_LOGE(TAG, "Fallo crítico al iniciar CANopen.");
    }

    // 4. El main se duerme (ahora sí funcionará el vTaskDelay)
    while(1) 
    {
        vTaskDelay(pdMS_TO_TICKS(10000)); 
    }
}

/*
#include <stdio.h>
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Tu librería encapsulada
#include "CANopen_LSS.h" 

#define GPIO_BOTON_EMERGENCIA   GPIO_NUM_0 
#define GPIO_CAN_ENABLE         GPIO_NUM_16 
static const char *TAG = "APP_MAIN";

// SETUP
void setup_hardware_externo()
{
    ESP_LOGI(TAG, "Configurando hardware...");
    
    // Enable Transceptor
    gpio_reset_pin(GPIO_CAN_ENABLE);
    gpio_set_direction(GPIO_CAN_ENABLE, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_CAN_ENABLE, 1); 

    // Botón
    gpio_reset_pin(GPIO_BOTON_EMERGENCIA);
    gpio_set_direction(GPIO_BOTON_EMERGENCIA, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_BOTON_EMERGENCIA, GPIO_PULLUP_ONLY);
}

// MAIN
void app_main(void)
{
    // Flash
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); nvs_flash_init();
    }

    setup_hardware_externo();

    ESP_LOGI(TAG, "--- ARRANCANDO ---");

    // Arrancar la librería (Velocidad 500, ID 0x20 para pruebas manuales)
    CO_ESP32_LSS_Run(500, 0x20);

    // Dormir
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); 
    }
}
*/