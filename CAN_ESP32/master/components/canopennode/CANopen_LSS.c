#include "CANopen_LSS.h"
#include "CANopen.h" // Necesario
#include "CO_driver_target.h"
#include "CO_NMT_Heartbeat.h"
#include "CO_LSSslave.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "OD.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// --- CONFIGURACIÓN ---
#define PIN_EMERGENCIA       GPIO_NUM_0 
#define NMT_CONTROL (CO_NMT_STARTUP_TO_OPERATIONAL | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION)
#define TAG "CO_WRAPPER"

// Configuración de la tarea (Igual que Código A)
#define TASK_PRIORITY        5
#define TASK_STACK_SIZE      4096
#define TASK_CORE            1

static CO_t *CO = NULL;
static uint32_t heapMemoryUsed = 0;

// Variables internas de la lógica
static bool b_emergencia_activa = false;
static uint8_t u8_dato_dummy = 0;
static uint64_t lastTime_dummy_us = 0;

// Parámetros pasados desde el main
static uint16_t g_bitRate;
static uint8_t  g_nodeId;

// --- STORAGE (Igual que antes) ---
#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
static CO_storage_t storage; 
static CO_storage_entry_t storageEntry1;
static CO_storage_entry_t *storageEntries[] = {&storageEntry1};
static uint8_t storageEntriesCount = 0;
static uint32_t storageInitError = 0;

static int config_storage(void){
    storageEntry1 = (CO_storage_entry_t){
        .addr = &OD_PERSIST_COMM,
        .len = sizeof(OD_PERSIST_COMM),
        .subIndexOD = 2,
        .attr = CO_storage_cmd | CO_storage_restore,
        .addrNV = NULL 
    };
    storageEntriesCount = 1;
    return CO_storageESP32_init(&storage, CO->CANmodule, OD_ENTRY_H1010_storeParameters, OD_ENTRY_H1011_restoreDefaultParameters, storageEntries, storageEntriesCount, &storageInitError);
}
#endif

// static uint32_t getSerialNumberFromMAC() {
//     uint8_t mac[6];
//     esp_read_mac(mac, ESP_MAC_BASE);
//     return ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | ((uint32_t)mac[5]);
// }

// -------------------------------------------------------------------------
// TAREA PRINCIPAL (Con el Monitor "Espía" integrado dentro)
// -------------------------------------------------------------------------
// static void co_main_task_thread(void *arg) {
//     CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
//     void* CANptr = NULL;
    
//     uint16_t bitRate = g_bitRate;
//     uint8_t nodeId = g_nodeId;
//     uint8_t active_node_id = nodeId;
//     uint32_t serial_number = 0;
//     uint64_t lastTime_us = esp_timer_get_time();

//     // 1. Memoria
//     CO = CO_new(NULL, &heapMemoryUsed);
//     if (!CO) { ESP_LOGE(TAG, "Fallo Memoria"); vTaskDelete(NULL); }

//     // 2. Storage
//     #if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
//     if (config_storage() != 0) { 
//         ESP_LOGE(TAG, "Fallo Storage"); 
//         CO_delete(CO); vTaskDelete(NULL);
//     }
//     #endif

//     while(reset != CO_RESET_APP){
//         ESP_LOGI(TAG, "Iniciando Stack...");
        
//         CO->CANmodule->CANnormal = false; 
//         CO_CANsetConfigurationMode(CANptr);

//         // 3. Init Driver
//         if(CO_CANinit(CO, CANptr, bitRate) != CO_ERROR_NO) {
//             ESP_LOGE(TAG, "Error Driver CAN");
//             vTaskDelay(pdMS_TO_TICKS(2000));
//             continue; 
//         }

//         // 4. Init LSS
//         serial_number = getSerialNumberFromMAC();
//         CO_LSS_address_t lss_address = {
//             .identity = { .vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
//                           .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
//                           .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
//                           .serialNumber = serial_number }
//         };
//         CO_LSSinit(CO, &lss_address, &nodeId, &bitRate);
//         active_node_id = nodeId; 

//         // 5. Init Protocolo
//         uint32_t errInfo = 0;
//         CO_CANopenInit(CO, NULL, NULL, OD, NULL, NMT_CONTROL, 500, 1000, 500, false, active_node_id, &errInfo);
//         CO_CANopenInitPDO(CO, CO->em, OD, active_node_id, &errInfo);

//         // ============================================================
//         // ACTIVAR MONITOR DE TRÁFICO (ESPÍA)
//         // Esto le dice al driver: "Avísame cuando llegue o salga algo"
//         // ============================================================
//         twai_reconfigure_alerts(TWAI_ALERT_RX_DATA | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED, NULL);
        
//         // 6. Arrancar
//         CO_CANsetNormalMode(CO->CANmodule); 
//         reset = CO_RESET_NOT;
//         lastTime_us = esp_timer_get_time();
//         lastTime_dummy_us = lastTime_us; 

//         ESP_LOGI(TAG, "--- NODO OPERATIVO (ID: %d) ---", active_node_id);

//         while (reset == CO_RESET_NOT) {
//             vTaskDelay(pdMS_TO_TICKS(10)); // Watchdog fix
            
//             uint64_t nowTime_us = esp_timer_get_time();
//             uint32_t timeDifference_us = (uint32_t)(nowTime_us - lastTime_us);
//             lastTime_us = nowTime_us;

//             // Procesar protocolo
//             reset = CO_process(CO, false, timeDifference_us, NULL);
//             if (CO->LSSslave) CO_LSSslave_process(CO->LSSslave);

//             // ============================================================
//             // 1. MONITOR DE TRÁFICO (AQUÍ ESTÁ LA MAGIA)
//             // Igual que en el Código A, leemos las alertas dentro del bucle
//             // ============================================================
//             uint32_t alerts = 0;
//             // Leemos alertas sin esperar (0ms)
//             if (twai_read_alerts(&alerts, 0) == ESP_OK) 
//             {
//                 if (alerts & TWAI_ALERT_RX_DATA) {
//                     ESP_LOGI(TAG, ">>> [BUS] Trama Recibida (RX)");
//                 }
//                 if (alerts & TWAI_ALERT_TX_SUCCESS) {
//                     ESP_LOGI(TAG, "<<< [BUS] Trama Enviada OK (ACK)");
//                 }
//                 if (alerts & TWAI_ALERT_TX_FAILED) {
//                     ESP_LOGE(TAG, "xxx [BUS] Fallo Envio (Error Bus)");
//                 }
//             }

//             // ============================================================
//             // 2. LÓGICA BOTÓN
//             // ============================================================
//             if (gpio_get_level(PIN_EMERGENCIA) == 0) { 
//                 if (!b_emergencia_activa) {
//                     b_emergencia_activa = true;
//                     ESP_LOGE(TAG, "!!! BOTÓN: EMERGENCIA !!!");
//                     CO_errorReport(CO->em, 1, CO_EMC_GENERIC, 0x5000);
//                 }
//             } else { 
//                 if (b_emergencia_activa) {
//                     b_emergencia_activa = false;
//                     CO_errorReset(CO->em, 1, 0); 
//                     ESP_LOGI(TAG, "Botón soltado. Reset.");
//                 }
//             }

//             // ============================================================
//             // 3. ENVÍO DUMMY
//             // ============================================================
//             if (!b_emergencia_activa && (nowTime_us - lastTime_dummy_us > 1000000)) { 
//                 lastTime_dummy_us = nowTime_us;
//                 u8_dato_dummy++;
//                 ESP_LOGI(TAG, "TX Dummy: %d", u8_dato_dummy);
//                 CO_errorReport(CO->em, 2, CO_EMC_GENERIC, (uint32_t)u8_dato_dummy);
//                 CO->em->errorStatusBits[2] = 0; 
//             }

//             if(reset == CO_RESET_COMM) break; 
//         }
        
//         CO_CANsetConfigurationMode(CANptr);
//         CO_CANmodule_disable(CO->CANmodule); 
//     }
//     CO_delete(CO);
//     vTaskDelete(NULL);
// }

// // FUNCIÓN DE ARRANQUE PÚBLICA
// void CO_ESP32_LSS_Run(uint16_t pendingBitRate, uint8_t pendingNodeId) {
//     g_bitRate = pendingBitRate;
//     g_nodeId = pendingNodeId;
//     xTaskCreatePinnedToCore(co_main_task_thread, "CO_Task", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL, TASK_CORE);
// }