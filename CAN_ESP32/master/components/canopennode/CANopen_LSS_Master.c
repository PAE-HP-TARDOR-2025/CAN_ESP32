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
#include <inttypes.h>

// --- CONFIGURACIÓN ---
#define MASTER_NODE_ID       0x01
#define MASTER_BITRATE       500   // IMPORTANTE: Debe coincidir con el Esclavo
#define TAG "MASTER_LSS"

// Empezaremos asignando la ID 16. Si hay otro, le dará la 17.
#define ID_INICIO_ASIGNACION 0x10 

// Tiempos RTOS
#define MAIN_TASK_PRIO       4
#define PERIODIC_TASK_PRIO   5
#define MAIN_INTERVAL_MS     10
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
static uint64_t scan_start_us = 0;
static uint32_t cached_vendor = 0;
static uint32_t cached_product = 0;
static bool cached_vendor_known = false;

TaskHandle_t mainTaskHandle = NULL;
TaskHandle_t periodicTaskHandle = NULL;

static void CO_mainTask(void *pxParam);
#if (((CO_CONFIG_LSS)&CO_CONFIG_FLAG_CALLBACK_PRE) != 0)
static void lss_master_signal(void* object) {
    (void)object;
    if (mainTaskHandle) xTaskNotifyGive(mainTaskHandle);
}
#endif
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
#if (((CO_CONFIG_LSS)&CO_CONFIG_FLAG_CALLBACK_PRE) != 0)
        /* Registrar callback pre para despertar la tarea cuando llegue trama LSS */
        CO_LSSmaster_initCallbackPre(CO->LSSmaster, NULL, lss_master_signal);
#endif
        // Reducir timeout LSS para acelerar fast-scan y confirmaciones
        CO_LSSmaster_changeTimeout(CO->LSSmaster, 50);

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
            /* Espera por notificación (LSS pre-callback) o timeout de ciclo */
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MAIN_INTERVAL_MS));
            reset = CO_process(CO, false, co_timer_us, NULL);
            
            // --- MÁQUINA DE ESTADOS LSS FASTSCAN ---
            switch (lssState) {
                
                case LSS_INIT:
                    // Limpiamos la estructura de escaneo
                    log_config_id = false;
                    log_config_store = false;
                    memset(&fastScan, 0, sizeof(fastScan));
                    
                    // Intentamos MATCH con Vendor/Product/Revision específicos para acelerar
                    fastScan.scan[CO_LSS_FASTSCAN_VENDOR_ID] = CO_LSSmaster_FS_MATCH;
                    fastScan.match.identity.vendorID = 0xFFFF0001;
                    fastScan.scan[CO_LSS_FASTSCAN_PRODUCT] = CO_LSSmaster_FS_MATCH;
                    fastScan.match.identity.productCode = 0x00000001;
                    fastScan.scan[CO_LSS_FASTSCAN_REV] = CO_LSSmaster_FS_MATCH;
                    fastScan.match.identity.revisionNumber = 0x00000000;
                    fastScan.scan[CO_LSS_FASTSCAN_SERIAL]    = CO_LSSmaster_FS_SCAN;
                    
                    scan_start_us = esp_timer_get_time();
                    lssState = LSS_SCANNING;
                    break;

                case LSS_SCANNING:
                    {
                        // Ejecutamos pasos del escaneo más rápidamente.
                        CO_LSSmaster_return_t ret = CO_LSSmaster_WAIT_SLAVE;
                        /* Use a small constant step (1 ms) for faster scans, independent of LSS timeout */
                        uint32_t fast_step_us = 1000; /* 1 ms */
                        const int max_steps = 200; /* up to ~200 ms total advance */
                        int steps = 0;
                        while ((ret == CO_LSSmaster_WAIT_SLAVE) && (steps++ < max_steps)) {
                            ret = CO_LSSmaster_IdentifyFastscan(CO->LSSmaster, fast_step_us, &fastScan);
                        }
                        /* Fastscan step log suppressed to reduce output */

                        if (ret == CO_LSSmaster_SCAN_FINISHED) {
                            uint32_t serial = fastScan.found.addr[3];
                            uint64_t now = esp_timer_get_time();
                            uint32_t elapsed_ms = (uint32_t)((now - scan_start_us) / 1000ULL);
                            ESP_LOGI(TAG, "Nodo DETECTADO: serial ...%08" PRIX32 " (took %" PRIu32 " ms, steps=%d)", (uint32_t)serial, (uint32_t)elapsed_ms, steps);
                            /* Guardamos vendor/product del nodo para acelerar próximos scans */
                            cached_vendor = fastScan.found.addr[0];
                            cached_product = fastScan.found.addr[1];
                            lssState = LSS_CONFIG_ID;
                        }
                        else if (ret == CO_LSSmaster_SCAN_NOACK || ret == CO_LSSmaster_TIMEOUT) {
                            uint64_t now_no = esp_timer_get_time();
                            uint32_t elapsed_ms_no = (uint32_t)((now_no - scan_start_us) / 1000ULL);
                            ESP_LOGI(TAG, "Escaneo finalizado (sin respuesta) después de %" PRIu32 " ms, pasos=%d.", elapsed_ms_no, steps);
                            lssState = LSS_DONE;
                        }
                        // Si devuelve CO_LSSmaster_WAIT_SLAVE, seguiremos avanzando en la siguiente iteración.
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
                    // Idle operativo, reintenta fast-scan periódicamente (cada 1s) usando tiempo real
                    static int timer_start = 0;
                    static uint64_t last_rescan_us = 0;
                    uint64_t now_us = esp_timer_get_time();
                    if (timer_start++ > 50) { // ~5 s
                        ESP_LOGI(TAG, "Red Operativa. Enviando NMT Start All.");
                        CO_NMT_sendCommand(CO->NMT, CO_NMT_ENTER_OPERATIONAL, 0);
                        timer_start = 0;
                    }
                    if ((now_us - last_rescan_us) > 1000000ULL) { // 1 segundo
                        last_rescan_us = now_us;
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