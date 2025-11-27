#include "sdkconfig.h"

// --- PARCHE DE CONFIGURACIÓN (OBLIGATORIO) ---
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
#include "freertos/FreeRTOS.h" // Necesario para Semáforos
#include "freertos/semphr.h"   // Necesario para Semáforos

#if (CONFIG_FREERTOS_HZ != 1000)
// #error "FreeRTOS tick interrupt frequency must be 1000Hz" 
#endif

#define CO_PERIODIC_TASK_INTERVAL_US (CONFIG_CO_PERIODIC_TASK_INTERVAL_MS * 1000)
#define CO_MAIN_TASK_INTERVAL_US (CONFIG_CO_MAIN_TASK_INTERVAL_MS * 1000)

// PIN DE EMERGENCIA (BOOT BUTTON)
#define PIN_EMERGENCIA GPIO_NUM_0

static const char *TAG = "CO_ESP32";

#define NMT_CONTROL (CO_NMT_STARTUP_TO_OPERATIONAL | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION)

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

// Variables lógicas
bool b_emergencia_activa = false;
uint8_t u8_dato_dummy = 0;

// --- NUEVO: Semáforo para comunicar la interrupción con el bucle ---
SemaphoreHandle_t xSemaforoEmergencia = NULL;

// --------------------------------------------------------------------------
// RUTINA DE INTERRUPCIÓN (ISR) - SE EJECUTA AL PULSAR EL BOTÓN
// Debe ser IRAM_ATTR para estar en memoria RAM rápida
// --------------------------------------------------------------------------
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    // Solo "Damos" el semáforo. Es una operación segura y rapidísima.
    // No enviamos el CAN aquí para no bloquear el sistema.
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xSemaforoEmergencia, &xHigherPriorityTaskWoken);
    
    // Si la tarea principal estaba esperando, forzamos que se ejecute ya
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

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

    // --- CONFIGURACIÓN DE INTERRUPCIÓN HARDWARE ---
    // 1. Crear Semáforo Binario
    xSemaforoEmergencia = xSemaphoreCreateBinary();

    // 2. Configurar GPIO
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE; // Interrupción por FLANCO DE BAJADA (al pulsar)
    io_conf.pin_bit_mask = (1ULL << PIN_EMERGENCIA);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1; // Pull-up interno
    gpio_config(&io_conf);

    // 3. Instalar el servicio de interrupciones y añadir el manejador
    // El '0' significa flags por defecto
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_EMERGENCIA, gpio_isr_handler, (void*) PIN_EMERGENCIA);
    
    ESP_LOGI(TAG, "Interrupción configurada en GPIO %d (Flanco de bajada)", PIN_EMERGENCIA);
    // ------------------------------------------------

    CO = CO_new(NULL, &heapMemoryUsed);
    if (CO == NULL) ESP_LOGW(TAG, "Can't allocate memory");

    while (reset != CO_RESET_APP)
    {
        ESP_LOGI(TAG, "CANopenNode - Reset communication");
        CO->CANmodule->CANnormal = false;
        CO_CANsetConfigurationMode(CANptr);

        err = CO_CANinit(CO, CANptr, CONFIG_CO_DEFAULT_BPS);
        if (err != CO_ERROR_NO) ESP_LOGE(TAG, "CAN initialization failed: %d", err);

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

#if (CONFIG_CO_PERIODIC_TASK_PRIORITY <= CONFIG_CO_MAIN_TASK_PRIORITY)
#error "Invalid CANopenNode task priority"
#endif
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
        reset = CO_RESET_NOT;
        ESP_LOGI(TAG, "CANopenNode is running");
        
        xLastWakeTime = xTaskGetTickCount();
        xTimerUltimoEnvio = xTaskGetTickCount();
        b_emergencia_activa = false; 

        // Limpiamos el semáforo por si se pulsó durante el reinicio
        xSemaphoreTake(xSemaforoEmergencia, 0);

        // ============================================================
        // BUCLE PRINCIPAL
        // ============================================================
        while (reset == CO_RESET_NOT)
        {
            vTaskDelayUntil(&xLastWakeTime, CONFIG_CO_MAIN_TASK_INTERVAL_MS);
            
            // Proceso estándar de CANopen
            reset = CO_process(CO, false, CO_MAIN_TASK_INTERVAL_US, NULL);

            // ------------------------------------------------------------
            // 1. CHEQUEO DE INTERRUPCIÓN (SEMÁFORO)
            // ------------------------------------------------------------
            // xSemaphoreTake con tiempo 0 no bloquea. Solo mira si la ISR lo activó.
            if (xSemaphoreTake(xSemaforoEmergencia, 0) == pdTRUE)
            {
                // ¡Si entramos aquí es que se pulsó el botón y saltó la ISR!
                if (!b_emergencia_activa) 
                {
                    ESP_LOGE(TAG, "!!! INTERRUPCIÓN HARDWARE RECIBIDA !!!");
                    b_emergencia_activa = true;
                    
                    // Enviamos el mensaje de emergencia de forma segura
                    CO_errorReport(CO->em, 1, CO_EMC_GENERIC, 0x5000);
                }
            }

            // Recuperación: Si estamos en emergencia y el botón YA NO está pulsado (1)
            // (La recuperación no necesita ser por interrupción, puede ser por polling)
            if (b_emergencia_activa && gpio_get_level(PIN_EMERGENCIA) == 1)
            {
                b_emergencia_activa = false;
                ESP_LOGI(TAG, "Emergencia finalizada (Botón soltado).");
            }

            // ------------------------------------------------------------
            // 2. LÓGICA DE ENVÍO CADA 1 SEGUNDO
            // ------------------------------------------------------------
            TickType_t now = xTaskGetTickCount();
            if (!b_emergencia_activa && (now - xTimerUltimoEnvio > pdMS_TO_TICKS(1000)))
            {
                xTimerUltimoEnvio = now;
                u8_dato_dummy++;
                
                // OD_RAM.x6000_... = u8_dato_dummy; 
                ESP_LOGI(TAG, "TX Enviado: %d", u8_dato_dummy);
            }
        }
    }

    CO_delete(CO);
    ESP_LOGI(TAG, "resetting");
    vTaskDelay(100);
    esp_restart();
    vTaskDelete(NULL);
}

static void CO_periodicTask(void *pxParam)
{
    ESP_LOGI(TAG, "Periodic task running"); // Esto SÍ puede estar aquí (se ejecuta 1 vez)

    while (1)
    {
        vTaskDelay(1); // Espera 1ms
        
        // --- AQUÍ DENTRO NO PONGAS NINGÚN ESP_LOG O PRINTF ---
        // Imprimir cada 1ms colapsa el chip.
        
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