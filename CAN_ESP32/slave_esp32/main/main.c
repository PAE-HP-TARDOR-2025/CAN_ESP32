#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_gpio.h"

// Includes
#include "CANopen.h"
#include "OD.h"
#include "CO_driver.h"
// #include "CANopenNode_ESP32.h" // A veces no es necesario si CO_driver.h ya hace el trabajo, pero si te da error descoméntalo.

// --- CONFIGURACIÓN DE HARDWARE ---
#define CAN_TX_GPIO         GPIO_NUM_5
#define CAN_RX_GPIO         GPIO_NUM_4
#define CAN_EN_GPIO         GPIO_NUM_16
#define CAN_EN_ACTIVE_LEVEL 1
#define SENSOR_INT_GPIO     GPIO_NUM_18

// --- CONFIGURACIÓN CANOPEN ---
#define NODE_ID             0x02
#define CAN_BAUDRATE        500 

// Variables Globales
CO_t *CO = NULL;
volatile bool activar_emergencia = false;

// --- 1. HARDWARE ENABLE ---
static void enable_transceiver(void) {
    esp_rom_gpio_pad_select_gpio(CAN_EN_GPIO);
    gpio_set_direction(CAN_EN_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CAN_EN_GPIO, CAN_EN_ACTIVE_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(10));
    printf("Transceiver OK (GPIO %d)\n", CAN_EN_GPIO);
}

// --- 2. ISR (Interrupción) ---
static void IRAM_ATTR sensor_isr_handler(void* arg) {
    activar_emergencia = true;
}

// --- MAIN ---
void app_main(void)
{
    printf("--- INICIANDO CANOPEN (Driver con Tareas Automáticas) ---\n");
    
    enable_transceiver();

    // Configurar Botón
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << SENSOR_INT_GPIO),
        .pull_up_en = 1
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(SENSOR_INT_GPIO, sensor_isr_handler, NULL);

    // Inicializar Memoria (v4)
#if CO_VERSION_MAJOR >= 4
    CO = CO_new(NULL, (uint32_t*)&OD_RAM);
#else
    CO = CO_new(NULL, NULL);
#endif
    if (!CO) { printf("Error Memoria\n"); return; }

    // Inicializar Stack
    // El driver interno inicializará las tareas TX/RX aquí dentro
    CO_ReturnError_t err;
    err = CO_CANopenInit(CO, NULL, NULL, OD, NULL, 0, 
                         1000, 1000, false, 
                         NODE_ID, 10, NULL); 

    if(err != CO_ERROR_NO) {
        printf("Error Init: %d\n", err);
        CO_delete(CO);
        return;
    }

    // Arrancar Driver
    CO_CANsetNormalMode(CO->CANmodule);
    
    // Pasar a Operacional (Esto activa el Heartbeat si está configurado en el OD)
    CO_NMT_sendInternalCommand(CO->NMT, CO_NMT_ENTER_OPERATIONAL);

    printf("Sistema Corriendo. ID: %d\n", NODE_ID);

    uint32_t time_prev = (uint32_t)(esp_timer_get_time());
    uint32_t time_now;
    uint32_t timerNext = 0;

    while (1) {
        time_now = (uint32_t)(esp_timer_get_time());
        uint32_t time_diff = time_now - time_prev;
        time_prev = time_now;

        // 1. Lógica del Stack (Timers, Heartbeats, Máquina de estados)
        CO_process(CO, false, time_diff, &timerNext);
        
        // NOTA CRÍTICA: 
        // YA NO LLAMAMOS A CO_CANinterrupt(). 
        // El driver 'sicrisembay' usa tareas FreeRTOS en background para esto.
        
        // 2. Gestión de Emergencia
        if (activar_emergencia) {
            printf(">> [ISR] EMERGENCIA DETECTADA\n");
            
            // Enviar Error (ID 0x80 + NodeID)
            CO_errorReport(CO->em, 0x01, 0x5000, 0x00);
            
            activar_emergencia = false;
            vTaskDelay(pdMS_TO_TICKS(200)); // Debounce
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}