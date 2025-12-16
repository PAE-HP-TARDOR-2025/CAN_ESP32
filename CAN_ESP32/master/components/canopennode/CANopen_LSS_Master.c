#include "CANopen.h"
#include "CO_LSSmaster.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "OD.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

// --- CONFIGURACIÓN ---
#define MASTER_NODE_ID       0x01
#define MASTER_BITRATE       500   // IMPORTANTE: Debe coincidir con el Esclavo
#define TAG "MASTER_LSS"

// Empezaremos asignando la ID 16. Si hay otro, le dará la 17.
#define ID_INICIO_ASIGNACION 0x10 

// Tiempos RTOS
#define MAIN_TASK_PRIO       4
#define PERIODIC_TASK_PRIO   5
#define MAIN_INTERVAL_MS     100
#define PERIODIC_INTERVAL_MS 10 

// Macro NMT
#define NMT_CONTROL (CO_NMT_STARTUP_TO_OPERATIONAL | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION)

// Estados de la máquina LSS
typedef enum {
    LSS_INIT,
    LSS_SCANNING,
    LSS_CONFIG_ID,
    LSS_CONFIG_STORE,
    LSS_ACTIVATE,
    LSS_DONE
} LssState_t;

static CO_t *CO = NULL;
static uint32_t heapMemoryUsed = 0;
static LssState_t lssState = LSS_INIT;
static bool log_config_id = false;
static bool log_config_store = false;

// Variables para el escaneo LSS
static CO_LSSmaster_fastscan_t fastScan;
static uint8_t next_id_to_assign = ID_INICIO_ASIGNACION;

TaskHandle_t mainTaskHandle = NULL;
TaskHandle_t periodicTaskHandle = NULL;

static void CO_mainTask(void *pxParam);
static void CO_periodicTask(void *pxParam);

// Callback de Emergencia
void emergencyCallback(const uint16_t ident, const uint16_t errorCode, const uint8_t errorRegister, const uint8_t errorBit, const uint32_t infoCode) {
    ESP_LOGE(TAG, "EMCY Recibida -> NodeID: 0x%02X | Code: 0x%04X", ident & 0x7F, errorCode);
}

// Función de arranque pública
void CO_ESP32_Master_Run(void) {
    xTaskCreatePinnedToCore(CO_mainTask, "CO_Master", 4096, NULL, MAIN_TASK_PRIO, &mainTaskHandle, 1);
}

// -------------------------------------------------------------------------
// TAREA PRINCIPAL (LÓGICA LSS)
// -------------------------------------------------------------------------
static void CO_mainTask(void *pxParam) {
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
    void* CANptr = NULL;
    CO = CO_new(NULL, &heapMemoryUsed);

    while (reset != CO_RESET_APP) {
        ESP_LOGI(TAG, "Iniciando MASTER...");
        CO->CANmodule->CANnormal = false;
        CO_CANsetConfigurationMode(CANptr);

        if(CO_CANinit(CO, CANptr, MASTER_BITRATE) != CO_ERROR_NO) {
            ESP_LOGE(TAG, "Error CAN Init"); vTaskDelay(pdMS_TO_TICKS(1000)); continue;
        }

        // Init LSS con los parámetros estándar (0x7E5/0x7E4)
        CO_LSSmaster_init(CO->LSSmaster, CO_LSSmaster_DEFAULT_TIMEOUT, CO->CANmodule, 0, 0x7E5, CO->CANmodule, 0, 0x7E4);
        // Reducir timeout LSS para acelerar fast-scan y confirmaciones
        CO_LSSmaster_changeTimeout(CO->LSSmaster, 200);

        uint32_t errInfo = 0;
        CO_CANopenInit(CO, NULL, NULL, OD, NULL, NMT_CONTROL, 1000, 1000, 1000, false, MASTER_NODE_ID, &errInfo);
        CO_CANopenInitPDO(CO, CO->em, OD, MASTER_NODE_ID, &errInfo);

        // Configurar SYNC (1 segundo)
        OD_PERSIST_COMM.x1005_COB_ID_SYNCMessage = 0x40000080; 
        OD_PERSIST_COMM.x1006_communicationCyclePeriod = 1000000; 

        #if (CO_CONFIG_EM) & CO_CONFIG_EM_CONSUMER
        CO_EM_initCallbackRx(CO->em, emergencyCallback);
        #endif

        if (periodicTaskHandle == NULL) {
            xTaskCreatePinnedToCore(CO_periodicTask, "CO_Periodic", 4096, NULL, PERIODIC_TASK_PRIO, &periodicTaskHandle, 1);
        }

        twai_reconfigure_alerts(TWAI_ALERT_RX_DATA | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED, NULL);
        CO_CANsetNormalMode(CO->CANmodule);
        reset = CO_RESET_NOT;
        
        ESP_LOGI(TAG, "MASTER LISTO. Escaneando red...");
        
        // Reiniciamos la máquina de estados
        lssState = LSS_INIT;
        log_config_id = false;
        log_config_store = false;
        next_id_to_assign = ID_INICIO_ASIGNACION;
        uint32_t co_timer_us = MAIN_INTERVAL_MS * 1000;

        while (reset == CO_RESET_NOT) {
            vTaskDelay(pdMS_TO_TICKS(MAIN_INTERVAL_MS)); 
            reset = CO_process(CO, false, co_timer_us, NULL);
            
            // --- MÁQUINA DE ESTADOS LSS FASTSCAN ---
            switch (lssState) {
                
                case LSS_INIT:
                    // Limpiamos la estructura de escaneo
                    log_config_id = false;
                    log_config_store = false;
                    memset(&fastScan, 0, sizeof(fastScan));
                    
                    // Indicamos que queremos escanear TODOS los campos (Vendor, Product, Rev, Serial)
                    // En tu librería, poner un valor != 0 significa "Escanear este campo"
                    fastScan.scan[CO_LSS_FASTSCAN_VENDOR_ID] = CO_LSSmaster_FS_SCAN;
                    fastScan.scan[CO_LSS_FASTSCAN_PRODUCT]   = CO_LSSmaster_FS_SCAN;
                    fastScan.scan[CO_LSS_FASTSCAN_REV]       = CO_LSSmaster_FS_SCAN;
                    fastScan.scan[CO_LSS_FASTSCAN_SERIAL]    = CO_LSSmaster_FS_SCAN;
                    
                    lssState = LSS_SCANNING;
                    break;

                case LSS_SCANNING:
                    {
                        // Ejecutamos un paso del escaneo.
                        // Esta función devuelve el ESTADO, no necesitamos mirar variables internas.
                        CO_LSSmaster_return_t ret = CO_LSSmaster_IdentifyFastscan(CO->LSSmaster, co_timer_us, &fastScan);
                        
                        if (ret == CO_LSSmaster_SCAN_FINISHED) {
                            lssState = LSS_CONFIG_ID;
                        }
                        else if (ret == CO_LSSmaster_SCAN_NOACK || ret == CO_LSSmaster_TIMEOUT) {
                            // Nadie responde.
                            ESP_LOGI(TAG, "Escaneo finalizado. No hay mas nodos nuevos.");
                            lssState = LSS_DONE;
                        }
                        // Si devuelve CO_LSSmaster_WAIT_SLAVE, seguimos aquí en la próxima vuelta.
                    }
                    break;

                case LSS_CONFIG_ID:
                    if (!log_config_id) {
                        ESP_LOGI(TAG, "Asignando ID %d...", next_id_to_assign);
                        log_config_id = true;
                    }
                    if (CO_LSSmaster_configureNodeId(CO->LSSmaster, co_timer_us, next_id_to_assign) == CO_LSSmaster_OK) {
                        lssState = LSS_CONFIG_STORE;
                        log_config_store = false;
                    }
                    break;

                case LSS_CONFIG_STORE:
                    if (!log_config_store) {
                        ESP_LOGI(TAG, "Guardando configuracion...");
                        log_config_store = true;
                    }
                    {
                        CO_LSSmaster_return_t sret = CO_LSSmaster_configureStore(CO->LSSmaster, co_timer_us);
                        if (sret == CO_LSSmaster_OK) {
                            ESP_LOGI(TAG, "ID %d asignada y almacenada en el nodo.", next_id_to_assign);
                            lssState = LSS_ACTIVATE;
                        } else if (sret != CO_LSSmaster_WAIT_SLAVE) {
                            ESP_LOGW(TAG, "Store LSS sin ACK (%d). Continuo sin persistir.", sret);
                            lssState = LSS_ACTIVATE;
                        }
                    }
                    break;

                case LSS_ACTIVATE:
                    ESP_LOGI(TAG, "Nodo %d configurado (sin cambio de bitrate).", next_id_to_assign);
                    next_id_to_assign++; // Preparamos ID para el siguiente
                    // Volvemos a buscar más nodos sin ID
                    lssState = LSS_INIT;
                    break;

                case LSS_DONE:
                    // Idle operativo, pero reintenta fast-scan periódicamente.
                    static int timer_start = 0;
                    static int rescan_ticks = 0;
                    if (timer_start++ > 2) { // ~5 s
                        ESP_LOGI(TAG, "Red Operativa. Enviando NMT Start All.");
                        CO_NMT_sendCommand(CO->NMT, CO_NMT_ENTER_OPERATIONAL, 0);
                        timer_start = 0;
                    }
                    if (rescan_ticks++ > 2) { // ~3 s
                        rescan_ticks = 0;
                        lssState = LSS_INIT; // relanzar fast-scan por si aparece un nodo nuevo
                    }
                    break;
            }

            // MONITOR DE TRÁFICO
            uint32_t alerts = 0;
            if (twai_read_alerts(&alerts, 0) == ESP_OK) {
                if (alerts & TWAI_ALERT_RX_DATA) {
                    // ESP_LOGI(TAG, "RX Data en Master");
                }
            }
        }
        CO_CANsetConfigurationMode(CANptr);
        CO_CANmodule_disable(CO->CANmodule);
    }
    if(periodicTaskHandle != NULL) { vTaskDelete(periodicTaskHandle); periodicTaskHandle = NULL; }
    CO_delete(CO);
    vTaskDelete(NULL);
}

// -------------------------------------------------------------------------
// TAREA PERIÓDICA
// -------------------------------------------------------------------------
static void CO_periodicTask(void *pxParam) {
    uint32_t co_timer_us = PERIODIC_INTERVAL_MS * 1000;
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(PERIODIC_INTERVAL_MS)); 
        if (!CO->CANmodule->CANnormal) continue; 

        bool syncWas = false;
        #if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
        syncWas = CO_process_SYNC(CO, co_timer_us, NULL);
        #endif
        CO_process_RPDO(CO, syncWas, co_timer_us, NULL); 
        CO_process_TPDO(CO, syncWas, co_timer_us, NULL);
        #if (CO_CONFIG_HB_CONS) & CO_CONFIG_HB_CONS_ENABLE
        CO_HBconsumer_process(CO->HBcons, true, co_timer_us, NULL);
        #endif
    }
}