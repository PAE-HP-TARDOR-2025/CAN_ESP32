#include "sdkconfig.h"

// --- PARCHE DE CONFIGURACIÓN (Valores por defecto si no están en menuconfig) ---
#ifndef CONFIG_CO_MAIN_TASK_STACK_SIZE
#define CONFIG_CO_MAIN_TASK_STACK_SIZE 4096
#endif
#ifndef CONFIG_CO_PERIODIC_TASK_STACK_SIZE
#define CONFIG_CO_PERIODIC_TASK_STACK_SIZE 4096
#endif
#ifndef CONFIG_CO_MAIN_TASK_PRIORITY
#define CONFIG_CO_MAIN_TASK_PRIORITY 5
#endif
#ifndef CONFIG_CO_PERIODIC_TASK_PRIORITY
#define CONFIG_CO_PERIODIC_TASK_PRIORITY 10
#endif
#ifndef CONFIG_CO_TASK_CORE
#define CONFIG_CO_TASK_CORE 1
#endif
#ifndef CONFIG_CO_MAIN_TASK_INTERVAL_MS
#define CONFIG_CO_MAIN_TASK_INTERVAL_MS 10
#endif
#ifndef CONFIG_CO_PERIODIC_TASK_INTERVAL_MS
#define CONFIG_CO_PERIODIC_TASK_INTERVAL_MS 1
#endif
#ifndef CONFIG_CO_DEFAULT_NODE_ID
#define CONFIG_CO_DEFAULT_NODE_ID 10
#endif
#ifndef CONFIG_CO_DEFAULT_BPS
#define CONFIG_CO_DEFAULT_BPS 500
#endif
#ifndef CONFIG_CO_FIRST_HB_TIME
#define CONFIG_CO_FIRST_HB_TIME 500
#endif
#ifndef CONFIG_CO_SDO_SERVER_TIMEOUT
#define CONFIG_CO_SDO_SERVER_TIMEOUT 1000
#endif
#ifndef CONFIG_CO_SDO_CLIENT_TIMEOUT
#define CONFIG_CO_SDO_CLIENT_TIMEOUT 1000
#endif
#ifndef CONFIG_CO_SDO_CLIENT_BLOCK_TRANSFER
#define CONFIG_CO_SDO_CLIENT_BLOCK_TRANSFER 0
#endif
// --------------------------------------------------------------

#include "esp_log.h"
#include "CANopen.h"
#include "OD.h"
#include "driver/gpio.h" 
#include "freertos/FreeRTOS.h" 
#include "freertos/semphr.h" 
#include "driver/twai.h" // NECESARIO para el monitor de tráfico

#if (CONFIG_FREERTOS_HZ != 1000)
// #error "FreeRTOS tick interrupt frequency must be 1000Hz" 
#endif

#define CO_PERIODIC_TASK_INTERVAL_US (CONFIG_CO_PERIODIC_TASK_INTERVAL_MS * 1000)
#define CO_MAIN_TASK_INTERVAL_US (CONFIG_CO_MAIN_TASK_INTERVAL_MS * 1000)

// Configuración Hardware
#define PIN_EMERGENCIA GPIO_NUM_0
static const char *TAG = "CO_ESP32";

// Configuración NMT básica
#define NMT_CONTROL (CO_NMT_STARTUP_TO_OPERATIONAL | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION)

// Objetos Globales
static CO_t *CO = NULL;
static void *CANptr = NULL;

static StaticTask_t xCoMainTaskBuffer;
static StackType_t xCoMainStack[CONFIG_CO_MAIN_TASK_STACK_SIZE];
static TaskHandle_t xCoMainTaskHandle = NULL;
static void CO_mainTask(void *pxParam);

static StaticTask_t xCoPeriodicTaskBuffer;
static StackType_t xCoPeriodicStack[CONFIG_CO_PERIODIC_TASK_STACK_SIZE];
static TaskHandle_t xCoPeriodicTaskHandle = NULL;
static void CO_periodicTask(void *pxParam);

// Variables lógicas de aplicación
bool b_emergencia_activa = false;
uint8_t u8_dato_dummy = 0;
SemaphoreHandle_t xSemaforoEmergencia = NULL;

// --------------------------------------------------------------------------
// RUTINA DE INTERRUPCIÓN (ISR) - Botón
// --------------------------------------------------------------------------
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xSemaforoEmergencia, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// --------------------------------------------------------------------------
// INICIALIZACIÓN
// --------------------------------------------------------------------------
bool CO_ESP32_init()
{
    ESP_LOGI(TAG, "Initializing");
    xCoMainTaskHandle = xTaskCreateStaticPinnedToCore(
        CO_mainTask,
        "CO_main",
        CONFIG_CO_MAIN_TASK_STACK_SIZE,
        (void *)0,
        CONFIG_CO_MAIN_TASK_PRIORITY,
        &xCoMainStack[0],
        &xCoMainTaskBuffer,
        CONFIG_CO_TASK_CORE);
    
    return (xCoMainTaskHandle != NULL);
}

// --------------------------------------------------------------------------
// TAREA PRINCIPAL (MAIN TASK)
// --------------------------------------------------------------------------
static void CO_mainTask(void *pxParam)
{
    CO_ReturnError_t err;
    uint32_t errInfo = 0;
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
    uint32_t heapMemoryUsed;
    uint8_t activeNodeId = CONFIG_CO_DEFAULT_NODE_ID;
    TickType_t xLastWakeTime;
    TickType_t xTimerUltimoEnvio = 0;

    ESP_LOGI(TAG, "main task running.");

    // 1. Configuración de Interrupción (Botón)
    xSemaforoEmergencia = xSemaphoreCreateBinary();
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE; 
    io_conf.pin_bit_mask = (1ULL << PIN_EMERGENCIA);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1; 
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_EMERGENCIA, gpio_isr_handler, (void*) PIN_EMERGENCIA);
    
    // 2. Inicialización de Memoria CANopen
    CO = CO_new(NULL, &heapMemoryUsed);
    if (CO == NULL) ESP_LOGW(TAG, "Can't allocate memory");

    while (reset != CO_RESET_APP)
    {
        ESP_LOGI(TAG, "CANopenNode - Reset communication");
        CO->CANmodule->CANnormal = false;
        CO_CANsetConfigurationMode(CANptr);

        // Inicializar Driver CAN
        err = CO_CANinit(CO, CANptr, CONFIG_CO_DEFAULT_BPS);
        
        // --- CORRECCIÓN: NO LLAMAR A CO_delete SI FALLA EL INIT ---
        if (err != CO_ERROR_NO) 
        {
            ESP_LOGE(TAG, "CAN initialization failed: %d", err);
            
            vTaskDelete(NULL); // Simplemente matamos la tarea y nos quedamos quietos.
            return;            
        }
        // ------------------------------------------------------------------

        // Inicializar Protocolo
        err = CO_CANopenInit(CO, NULL, NULL, OD, NULL, 
                             NMT_CONTROL, 
                             CONFIG_CO_FIRST_HB_TIME,
                             CONFIG_CO_SDO_SERVER_TIMEOUT,
                             CONFIG_CO_SDO_CLIENT_TIMEOUT,
#if CONFIG_CO_SDO_CLIENT_BLOCK_TRANSFER
                             true,
#else
                             false,
#endif
                             activeNodeId, &errInfo);
                             
        CO_CANopenInitPDO(CO, CO->em, OD, activeNodeId, &errInfo);

        // Crear tarea periódica si no existe
        if (xCoPeriodicTaskHandle == NULL)
        {
            xCoPeriodicTaskHandle = xTaskCreateStaticPinnedToCore(
                CO_periodicTask, "CO_timer", CONFIG_CO_PERIODIC_TASK_STACK_SIZE,
                (void *)0, CONFIG_CO_PERIODIC_TASK_PRIORITY,
                &xCoPeriodicStack[0], &xCoPeriodicTaskBuffer, CONFIG_CO_TASK_CORE);
        }

#if CO_CONFIG_LEDS
        CO_LEDs_init(CO->LEDs);
#endif

        CO_CANsetNormalMode(CO->CANmodule);
        
        // ---------------------------------------------------------------------
        // IMPORTANTE: ACTIVAR MONITOR DE TRÁFICO (ESPÍA)
        // Esto permite ver en el log si las tramas entran (RX) y si salen (TX)
        // ---------------------------------------------------------------------
        twai_reconfigure_alerts(TWAI_ALERT_RX_DATA | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED, NULL);

        reset = CO_RESET_NOT;
        ESP_LOGI(TAG, "CANopenNode is running");
        
        xLastWakeTime = xTaskGetTickCount();
        xTimerUltimoEnvio = xTaskGetTickCount();
        b_emergencia_activa = false; 
        
        // Limpiar semáforo inicial
        xSemaphoreTake(xSemaforoEmergencia, 0);

        // BUCLE OPERATIVO
        while (reset == CO_RESET_NOT)
        {
            vTaskDelayUntil(&xLastWakeTime, CONFIG_CO_MAIN_TASK_INTERVAL_MS);
            
            // --- A. Proceso CANopen ---
            reset = CO_process(CO, false, CO_MAIN_TASK_INTERVAL_US, NULL);

            // --- B. Monitor de Tráfico (LOGS) ---
            uint32_t alerts = 0;
            twai_read_alerts(&alerts, 0); // Leer alertas sin bloquear
            
            if (alerts & TWAI_ALERT_RX_DATA) {
                ESP_LOGI(TAG, ">>> [BUS] Trama Recibida (RX)");
            }
            if (alerts & TWAI_ALERT_TX_SUCCESS) {
                ESP_LOGI(TAG, "<<< [BUS] Trama Enviada OK (TX ACK Recibido)");
            }
            if (alerts & TWAI_ALERT_TX_FAILED) {
                ESP_LOGE(TAG, "xxx [BUS] Fallo de Envío (Nadie escucha o Error Bus)");
            }

            // --- C. Botón de Emergencia (TX por Evento) ---
            if (xSemaphoreTake(xSemaforoEmergencia, 0) == pdTRUE)
            {
                if (!b_emergencia_activa) 
                {
                    ESP_LOGE(TAG, "!!! BOTÓN: Enviando Emergencia CRÍTICA !!!");
                    b_emergencia_activa = true;
                    // Envia error 0x5000
                    CO_errorReport(CO->em, 1, CO_EMC_GENERIC, 0x5000);
                }
            }

            // Recuperación: Si estamos en emergencia y el botón YA NO está pulsado
            if (b_emergencia_activa && gpio_get_level(PIN_EMERGENCIA) == 1)
            {
                b_emergencia_activa = false;
                
                CO_errorReset(CO->em, 1, 0); 
                
                ESP_LOGI(TAG, "Botón soltado. Error limpiado. Listo para la próxima.");
            }

            
            // --- D. Envío Cíclico "Dummy" (Hack TX sin OD) ---
            TickType_t now = xTaskGetTickCount();
            if (!b_emergencia_activa && (now - xTimerUltimoEnvio > pdMS_TO_TICKS(1000)))
            {
                xTimerUltimoEnvio = now;
                u8_dato_dummy++;
                
                // Usamos el canal de emergencia para enviar el dato crudo
                uint32_t data_para_enviar = (uint32_t)u8_dato_dummy;
                
                ESP_LOGI(TAG, "Intentando enviar dato: %d ...", u8_dato_dummy);
                
                CO_errorReport(CO->em, 2, CO_EMC_GENERIC, data_para_enviar);
                CO->em->errorStatusBits[2] = 0; // Limpiar bit inmediatamente para permitir el siguiente
            }
                
        }
    }

    CO_delete(CO);
    ESP_LOGI(TAG, "resetting");
    vTaskDelay(100);
    esp_restart();
    vTaskDelete(NULL);
}

// --------------------------------------------------------------------------
// TAREA PERIÓDICA (TIMER 1ms)
// --------------------------------------------------------------------------
static void CO_periodicTask(void *pxParam)
{
    while (1)
    {
        vTaskDelay(1); 
        if ((!CO->nodeIdUnconfigured) && (CO->CANmodule->CANnormal))
        {
            bool syncWas = false;
#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
            syncWas = CO_process_SYNC(CO, CO_PERIODIC_TASK_INTERVAL_US, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
            CO_process_RPDO(CO, syncWas, CO_PERIODIC_TASK_INTERVAL_US, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
            CO_process_TPDO(CO, syncWas, CO_PERIODIC_TASK_INTERVAL_US, NULL);
#endif
        }
    }
}