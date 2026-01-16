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
#include "freertos/semphr.h"
#include "fw_master_update.h"
#include <string.h>
#include <inttypes.h>


// --- CONFIGURACIÓN ---
#define MASTER_NODE_ID       0x01
#define MASTER_BITRATE       500   // IMPORTANTE: Debe coincidir con el Esclavo
#define TAG "MASTER_LSS"
// Modo compatibilidad v1: desactivar PDO para evitar RX Data del esclavo v2
#ifndef MASTER_ENABLE_PDO
#define MASTER_ENABLE_PDO 0
#endif


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
    LSS_VERIFY_ID, /* verify node ID after store */
    LSS_DESELECT, /* deselect node after store */
    LSS_ACTIVATE,
    LSS_FORCE_SELECT,
    LSS_DONE
} LssState_t;


static CO_t *CO = NULL;
static uint32_t heapMemoryUsed = 0;
static LssState_t lssState = LSS_INIT;
static bool log_config_id = false;
static bool log_config_store = false;


// Variables para el escaneo LSS
static CO_LSSmaster_fastscan_t fastScan;
static CO_LSS_address_t last_found_address; /* dirección LSS del último nodo detectado */
static uint8_t next_id_to_assign = ID_INICIO_ASIGNACION;
static uint64_t scan_start_us = 0;
static uint32_t cached_vendor = 0;
static uint32_t cached_product = 0;

/* SDO client reference. */
static CO_SDOclient_t *g_sdoClient = NULL;


// Lista simple de nodos ya configurados en runtime para evitar reconfigurar el mismo serial
#define MAX_CONFIGURED_NODES 32
// Tiempo que evitaremos reintentar reconfigurar la misma dirección (ms)
#define CONFIGURED_NODE_SKIP_MS 30000


typedef struct {
    CO_LSS_address_t addr;
    uint64_t skip_until_us; /* until what time to skip (esp_timer_get_time() units) */
    uint8_t assigned_node_id; /* ID previously assigned to this node */
    bool upload_launched;     /* true if firmware upload task already started for this node */
} configured_node_t;


static configured_node_t configured_nodes[MAX_CONFIGURED_NODES];
static int configured_nodes_count = 0;
/* Global rescan timestamp used by LSS_DONE state to schedule next rescan */
static uint64_t last_rescan_us_global = 0;

/* Find the next free node-id not used in configured_nodes. Wraps around on 127. */
static uint8_t find_next_free_node_id(void) {
    uint8_t candidate = ID_INICIO_ASIGNACION;
    while (true) {
        bool used = false;
        for (int i = 0; i < configured_nodes_count; i++) {
            if (configured_nodes[i].assigned_node_id == candidate) {
                used = true;
                break;
            }
        }
        if (!used && candidate != MASTER_NODE_ID) {
            return candidate;
        }
        candidate = (candidate < 127) ? (uint8_t)(candidate + 1) : 2;
    }
}

/* Probe mínimo para saber si un nodo con ID responde por SDO sin generar logs de error. */
static bool sdo_probe_node(uint8_t nodeId);

static bool is_node_id_used(uint8_t candidate) {
    if (candidate == MASTER_NODE_ID) {
        return true;
    }
    for (int i = 0; i < configured_nodes_count; i++) {
        if (configured_nodes[i].assigned_node_id == candidate) {
            return true;
        }
    }
    return false;
}

static bool is_node_id_active_on_bus(uint8_t candidate) {
    if (candidate == MASTER_NODE_ID) {
        return true;
    }
    if (g_sdoClient == NULL) {
        return false;
    }
    return sdo_probe_node(candidate);
}

static uint8_t find_next_free_node_id_active_aware(void) {
    uint8_t candidate = ID_INICIO_ASIGNACION;
    while (true) {
        if (!is_node_id_used(candidate) && !is_node_id_active_on_bus(candidate)) {
            return candidate;
        }
        candidate = (candidate < 127) ? (uint8_t)(candidate + 1) : 2;
    }
}

/* Firmware update plan */
#define FW_IMAGE_PATH   "/spiffs/slave.bin"
#define FW_TARGET_BANK  1
#define FW_MAX_CHUNK    256U
#define FW_VERSION      2

typedef struct {
    uint8_t nodeId;
} fw_upload_args_t;

static SemaphoreHandle_t sdoMutex = NULL;

static TaskHandle_t s_fwUploadTasks[128] = {0};
static volatile int s_activeUploads = 0;
static uint64_t s_lastDuplicateScan_us = 0;
static volatile bool s_force_lss_reassign = false;


// Re-scan interval (ms) para preguntar por nodos nuevos conectados posteriormente
#define RESCAN_INTERVAL_MS 10000
// Tiempo que esperamos tras deseleccionar (ms) para que el esclavo haga reset/apply NID
// Incrementado para dar más tiempo al esclavo a aplicar la configuración
#define DESELECT_DELAY_MS 1000


// Fastscan adaptive parameters
// NOTA: Ya no usamos deadline fijo - permitimos que el escaneo complete el rango LSS completo
// Solo aplicamos timeout si no hay progreso (escaneo atasca)
static int fastscan_no_response_count = 0;            /* adaptive backoff counter */


// Estado de asignación: candidato actual y contador de intentos (para evitar bucles infinitos)
static uint8_t current_candidate_id = 0;
static int id_attempt_rounds = 0;
static uint64_t last_deselected_us = 0; // 0 = no pendiente
static int selection_verify_attempts = 0; /* intentos de selección para verificar si el nodo sigue en modo LSS */


TaskHandle_t mainTaskHandle = NULL;
TaskHandle_t periodicTaskHandle = NULL;

/* SDO client reference (como en el demo de firmware_updater) */

static void CO_mainTask(void *pxParam);
#if (((CO_CONFIG_LSS)&CO_CONFIG_FLAG_CALLBACK_PRE) != 0)
static void lss_master_signal(void* object) {
    (void)object;
    if (mainTaskHandle) xTaskNotifyGive(mainTaskHandle);
}
#endif
static void CO_periodicTask(void *pxParam);

/* -------------------------------------------------------------------------- */
/* Helpers SDO para el uploader                                               */
/* -------------------------------------------------------------------------- */

static TickType_t wait_ticks(uint32_t ms) {
    TickType_t ticks = pdMS_TO_TICKS(ms);
    return (ticks > 0) ? ticks : 1;
}

static bool sdo_download(uint8_t nodeId, uint16_t index, uint8_t subIndex, const uint8_t *data, size_t len) {
    if (g_sdoClient == NULL) {
        ESP_LOGE(TAG, "SDO client not available");
        return false;
    }
    
    CO_SDOclient_t *client = g_sdoClient;
    CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;
    size_t sizeTransferred = 0;

    CO_SDOclient_setup(client, 0x600 + nodeId, 0x580 + nodeId, nodeId);
    CO_SDO_return_t ret = CO_SDOclientDownloadInitiate(client, index, subIndex, len, 3000, true);
    if (ret < CO_SDO_RT_ok_communicationEnd) {
        ESP_LOGE(TAG, "SDO download init failed: ret=%d idx=0x%04X:%u", ret, index, subIndex);
        return false;
    }

    size_t offset = 0;
    bool bufferPartial = true;
    uint64_t start_us = esp_timer_get_time();
    const uint64_t timeout_us = 2000000ULL; /* 2s guard */
    bool timed_out = false;
    do {
        if (offset < len) {
            size_t written = CO_SDOclientDownloadBufWrite(client, data + offset, len - offset);
            offset += written;
            bufferPartial = (offset < len);
        } else {
            bufferPartial = false;
        }
        ret = CO_SDOclientDownload(client, 1000, false, bufferPartial, &abortCode, &sizeTransferred, NULL);
        if (ret == CO_SDO_RT_waitingResponse) {
            if ((esp_timer_get_time() - start_us) > timeout_us) {
                timed_out = true;
                break;
            }
            vTaskDelay(wait_ticks(1));
        }
    } while (ret > CO_SDO_RT_ok_communicationEnd);

    if (timed_out || ret != CO_SDO_RT_ok_communicationEnd) {
        ESP_LOGE(TAG, "SDO download failed: ret=%d abort=0x%08lX idx=0x%04X:%u%s", ret, (unsigned long)abortCode, index, subIndex, timed_out ? " (timeout)" : "");
        return false;
    }
    return true;
}

static bool sdo_upload(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint8_t *buf, size_t maxLen, size_t *actualLen) {
    if (g_sdoClient == NULL) {
        ESP_LOGE(TAG, "SDO client not available");
        return false;
    }
    
    CO_SDOclient_t *client = g_sdoClient;
    CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;
    size_t sizeTransferred = 0;
    size_t sizeIndicated = 0;

    CO_SDOclient_setup(client, 0x600 + nodeId, 0x580 + nodeId, nodeId);
    CO_SDO_return_t ret = CO_SDOclientUploadInitiate(client, index, subIndex, 3000, true);
    if (ret < CO_SDO_RT_ok_communicationEnd) {
        ESP_LOGE(TAG, "SDO upload init failed: ret=%d idx=0x%04X:%u", ret, index, subIndex);
        return false;
    }

    uint64_t start_us = esp_timer_get_time();
    const uint64_t timeout_us = 2000000ULL; /* 2s guard */
    bool timed_out = false;
    do {
        ret = CO_SDOclientUpload(client, 1000, false, &abortCode, &sizeIndicated, &sizeTransferred, NULL);
        if (ret == CO_SDO_RT_waitingResponse) {
            if ((esp_timer_get_time() - start_us) > timeout_us) {
                timed_out = true;
                break;
            }
            vTaskDelay(wait_ticks(1));
        }
    } while (ret > CO_SDO_RT_ok_communicationEnd);

    if (timed_out || ret != CO_SDO_RT_ok_communicationEnd) {
        ESP_LOGE(TAG, "SDO upload failed: ret=%d abort=0x%08lX idx=0x%04X:%u%s", ret, (unsigned long)abortCode, index, subIndex, timed_out ? " (timeout)" : "");
        return false;
    }
    size_t read = CO_SDOclientUploadBufRead(client, buf, maxLen);
    if (actualLen) *actualLen = read;
    return true;
}

/* Probe mínimo para saber si un nodo con ID responde por SDO sin generar logs de error. */
static bool sdo_probe_node(uint8_t nodeId) {
    if (g_sdoClient == NULL) {
        return false;
    }

    CO_SDOclient_t *client = g_sdoClient;
    CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;
    size_t sizeTransferred = 0;
    size_t sizeIndicated = 0;

    /* Configurar SDO a la ID indicada */
    CO_SDOclient_setup(client, 0x600 + nodeId, 0x580 + nodeId, nodeId);
    CO_SDO_return_t ret = CO_SDOclientUploadInitiate(client, 0x1000, 0, 500, true); /* 0x1000 = Device Type */
    if (ret < CO_SDO_RT_ok_communicationEnd) {
        return false; /* no responde */
    }

    uint64_t start_us = esp_timer_get_time();
    const uint64_t timeout_us = 500000ULL; /* 500 ms total guard for probe */
    do {
        ret = CO_SDOclientUpload(client, 300, false, &abortCode, &sizeIndicated, &sizeTransferred, NULL);
        if (ret == CO_SDO_RT_waitingResponse) {
            if ((esp_timer_get_time() - start_us) > timeout_us) {
                return false;
            }
            vTaskDelay(wait_ticks(1));
        }
    } while (ret > CO_SDO_RT_ok_communicationEnd);

    if (ret != CO_SDO_RT_ok_communicationEnd) {
        return false;
    }

    /* Leemos pero no usamos el valor; solo importa que respondió. */
    uint8_t tmpBuf[4];
    CO_SDOclientUploadBufRead(client, tmpBuf, sizeof(tmpBuf));
    return true;
}

bool fw_master_send_metadata(const fw_upload_plan_t *plan, const fw_payload_t *payload, uint16_t crc) {
    uint8_t meta[10];
    uint32_t size = (uint32_t)payload->size;
    meta[0] = (uint8_t)(size & 0xFF);
    meta[1] = (uint8_t)((size >> 8) & 0xFF);
    meta[2] = (uint8_t)((size >> 16) & 0xFF);
    meta[3] = (uint8_t)((size >> 24) & 0xFF);
    meta[4] = (uint8_t)(crc & 0xFF);
    meta[5] = (uint8_t)((crc >> 8) & 0xFF);
    meta[6] = (uint8_t)plan->type;
    meta[7] = plan->targetBank;
    meta[8] = (uint8_t)(plan->firmwareVersion & 0xFF);
    meta[9] = (uint8_t)((plan->firmwareVersion >> 8) & 0xFF);

    if (sdoMutex == NULL) sdoMutex = xSemaphoreCreateMutex();
    if (sdoMutex == NULL) return false;
    xSemaphoreTake(sdoMutex, portMAX_DELAY);
    bool ok = sdo_download(plan->targetNodeId, 0x1F57, 1, meta, sizeof(meta));
    xSemaphoreGive(sdoMutex);
    return ok;
}

bool fw_master_send_start_command(const fw_upload_plan_t *plan) {
    uint8_t cmd[3] = {0x01, 0x00, 0x00};
    if (sdoMutex == NULL) sdoMutex = xSemaphoreCreateMutex();
    if (sdoMutex == NULL) return false;
    xSemaphoreTake(sdoMutex, portMAX_DELAY);
    bool ok = sdo_download(plan->targetNodeId, 0x1F51, 1, cmd, sizeof(cmd));
    xSemaphoreGive(sdoMutex);
    return ok;
}

bool fw_master_send_chunk(const fw_upload_plan_t *plan, const uint8_t *chunk, size_t len, size_t offset) {
    (void)offset;
    if (sdoMutex == NULL) sdoMutex = xSemaphoreCreateMutex();
    if (sdoMutex == NULL) return false;
    xSemaphoreTake(sdoMutex, portMAX_DELAY);
    bool ok = sdo_download(plan->targetNodeId, 0x1F50, 1, chunk, len);
    xSemaphoreGive(sdoMutex);
    return ok;
}

bool fw_master_send_finalize_request(const fw_upload_plan_t *plan, uint16_t crc) {
    uint8_t status[2] = {(uint8_t)(crc & 0xFF), (uint8_t)((crc >> 8) & 0xFF)};
    if (sdoMutex == NULL) sdoMutex = xSemaphoreCreateMutex();
    if (sdoMutex == NULL) return false;
    xSemaphoreTake(sdoMutex, portMAX_DELAY);
    bool ok = sdo_download(plan->targetNodeId, 0x1F5A, 1, status, sizeof(status));
    xSemaphoreGive(sdoMutex);
    return ok;
}

bool fw_master_query_slave_crc(const fw_upload_plan_t *plan, uint16_t *slaveCrc) {
    uint8_t buf[2] = {0};
    size_t got = 0;
    if (sdoMutex == NULL) sdoMutex = xSemaphoreCreateMutex();
    if (sdoMutex == NULL) return false;
    xSemaphoreTake(sdoMutex, portMAX_DELAY);
    bool ok = sdo_upload(plan->targetNodeId, 0x1F5B, 1, buf, sizeof(buf), &got);
    xSemaphoreGive(sdoMutex);
    if (!ok || got < 2) return false;
    *slaveCrc = (uint16_t)(buf[0] | (buf[1] << 8));
    return true;
}

bool fw_master_query_slave_version(const fw_upload_plan_t *plan, uint16_t *slaveVersion) {
    uint8_t buf[2] = {0};
    size_t got = 0;
    if (sdoMutex == NULL) sdoMutex = xSemaphoreCreateMutex();
    if (sdoMutex == NULL) return false;
    xSemaphoreTake(sdoMutex, portMAX_DELAY);
    bool ok = sdo_upload(plan->targetNodeId, 0x1F5C, 1, buf, sizeof(buf), &got);
    xSemaphoreGive(sdoMutex);
    if (!ok || got < 2) return false;
    *slaveVersion = (uint16_t)(buf[0] | (buf[1] << 8));
    return true;
}

static void fw_upload_task(void *arg) {
    fw_upload_args_t *args = (fw_upload_args_t *)arg;
    uint8_t nodeId = args->nodeId;
    vPortFree(arg);  // Liberar args inmediatamente, ya tenemos nodeId
    
    // Esperar un momento para asegurar que CANopen esté totalmente operativo
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Verificar que SDO client esté disponible
    if (g_sdoClient == NULL) {
        ESP_LOGE(TAG, "Upload abortado: SDO client no disponible para nodo %u", nodeId);
        vTaskDelete(NULL);
        return;
    }
    
    fw_upload_plan_t plan = {
        .firmwarePath = FW_IMAGE_PATH,
        .type = FW_IMAGE_MAIN,
        .targetBank = FW_TARGET_BANK,
        .targetNodeId = nodeId,
        .maxChunkBytes = FW_MAX_CHUNK,
        .expectedCrc = 0,
        .firmwareVersion = FW_VERSION,
    };

    ESP_LOGI(TAG, "Iniciando uploader para nodo %u", nodeId);
    
    // Ejecutar una sola vez: subir si hace falta y salir para no seguir chequeando.
    if (!fw_master_run_upload_if_needed(&plan)) {
        ESP_LOGW(TAG, "Upload falló o no se completó para nodo %u", nodeId);
        s_force_lss_reassign = true;
    } else {
        ESP_LOGI(TAG, "Nodo %u tiene firmware actualizado (CRC match)", nodeId);
    }

    // No reintentar continuamente: finalizar la tarea.
    if (nodeId < 128) {
        s_fwUploadTasks[nodeId] = NULL;
    }
    if (s_activeUploads > 0) {
        s_activeUploads--;
    }
    vTaskDelete(NULL);
}

static bool start_firmware_upload(uint8_t nodeId) {
    if (sdoMutex == NULL) {
        sdoMutex = xSemaphoreCreateMutex();
    }

    if (nodeId < 128 && s_fwUploadTasks[nodeId] != NULL) {
        ESP_LOGI(TAG, "Uploader ya en ejecución para nodo %u; saltando", nodeId);
        return false;
    }

    if (s_activeUploads > 0) {
        ESP_LOGI(TAG, "Uploader activo (%d). Esperando para nodo %u", s_activeUploads, nodeId);
        return false;
    }

    fw_upload_args_t *args = pvPortMalloc(sizeof(fw_upload_args_t));
    if (args == NULL) {
        ESP_LOGW(TAG, "Sin memoria para tarea uploader (node %u)", nodeId);
        return false;
    }
    args->nodeId = nodeId;

    TaskHandle_t handle = NULL;
    BaseType_t created = xTaskCreatePinnedToCore(fw_upload_task, "fw_upload", 8192, args, 4, &handle, 1);
    if (created != pdPASS) {
        ESP_LOGW(TAG, "No se pudo crear tarea uploader (node %u)", nodeId);
        vPortFree(args);
        return false;
    }

    if (nodeId < 128) {
        s_fwUploadTasks[nodeId] = handle;
    }
    s_activeUploads++;
    return true;
}


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
        g_sdoClient = NULL;  // Resetear al reiniciar
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
        CO_CANopenInit(CO, NULL, NULL, OD, NULL, NMT_CONTROL, 1000, 1000, 3000, true, MASTER_NODE_ID, &errInfo);
        CO_CANopenInitPDO(CO, CO->em, OD, MASTER_NODE_ID, &errInfo);
        
        // Obtener y verificar SDO client reference (como en demo firmware_updater)
        g_sdoClient = CO->SDOclient;
        if (g_sdoClient == NULL) {
            ESP_LOGE(TAG, "SDO client no disponible tras CO_CANopenInit");
        } else {
            ESP_LOGI(TAG, "SDO client inicializado correctamente");
        }


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
                   
                    // Usar scanning amplio en vendor/product/rev para detectar cualquier nodo sin configurar
                    fastScan.scan[CO_LSS_FASTSCAN_VENDOR_ID] = CO_LSSmaster_FS_SCAN;
                    fastScan.scan[CO_LSS_FASTSCAN_PRODUCT] = CO_LSSmaster_FS_SCAN;
                    fastScan.scan[CO_LSS_FASTSCAN_REV] = CO_LSSmaster_FS_SCAN;
                    fastScan.scan[CO_LSS_FASTSCAN_SERIAL]    = CO_LSSmaster_FS_SCAN;
                   
                    // Reiniciamos candidatos de ID
                    current_candidate_id = next_id_to_assign;
                    id_attempt_rounds = 0;


                    scan_start_us = esp_timer_get_time();
                    lssState = LSS_SCANNING;
                    break;


                case LSS_SCANNING:
                    {
                        // Ejecutamos pasos del escaneo hasta que complete el rango LSS completo
                        // SIN abortar por deadline: permitimos que busque en TODO el rango de parámetros
                        CO_LSSmaster_return_t ret = CO_LSSmaster_WAIT_SLAVE;
                        /* Increment bigger per step to reduce number of loop iterations on slow buses */
                        const uint32_t fast_step_us = 2000; /* 2 ms por paso - reduce iteraciones sin ser demasiado pequeño */
                        /* Límite de seguridad: máximo tiempo permitido sin progreso (no progreso = escaneo atasca) */
                        uint64_t progress_deadline_us = esp_timer_get_time() + 10000000ULL; /* 10 segundos timeout si no hay progreso */
                        int steps = 0;
                        int last_step_at_check = 0;
                        uint64_t last_progress_check_us = esp_timer_get_time();


                        while (ret == CO_LSSmaster_WAIT_SLAVE) {
                            ret = CO_LSSmaster_IdentifyFastscan(CO->LSSmaster, fast_step_us, &fastScan);
                            steps++;
                            /* Ceder menos frecuentemente pero con el mínimo de 10 ms para no ser tratado como 0 */
                            if ((steps & 0xFF) == 0) {
                                uint64_t now = esp_timer_get_time();
                                vTaskDelay(pdMS_TO_TICKS(10));
                                
                                /* Verificar progreso cada 500ms: si no avanza en pasos, puede ser un error real */
                                if ((now - last_progress_check_us) > 500000ULL) {
                                    if (steps == last_step_at_check && ret == CO_LSSmaster_WAIT_SLAVE) {
                                        /* No progresa en 500ms, posible timeout real */
                                        ESP_LOGW(TAG, "Fastscan no progress detected (steps stuck at %d), timeout", steps);
                                        ret = CO_LSSmaster_TIMEOUT;
                                        break;
                                    }
                                    last_step_at_check = steps;
                                    last_progress_check_us = now;
                                }
                                
                                if (now > progress_deadline_us) {
                                    ESP_LOGW(TAG, "Fastscan safety timeout (10s), aborting, steps=%d", steps);
                                    break;
                                }
                            }
                        }
                        /* Update adaptive counters: if no response, increment, otherwise reset */
                        if (ret == CO_LSSmaster_SCAN_FINISHED) {
                            /* success -> reset no-response counter */
                            fastscan_no_response_count = 0;
                        } else if (ret == CO_LSSmaster_SCAN_NOACK || ret == CO_LSSmaster_TIMEOUT) {
                            /* no response -> increase backoff for next scan */
                            if (fastscan_no_response_count < 10) fastscan_no_response_count++;
                        }
                        /* Si hemos salido por still waiting, considerarlo como timeout para seguir la lógica siguiente */
                        if (ret == CO_LSSmaster_WAIT_SLAVE) {
                            ret = CO_LSSmaster_TIMEOUT;
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
                            /* Guardamos la dirección LSS */
                            last_found_address = fastScan.found;


                            /* Si ya configuramos este nodo anteriormente, lo saltamos con expiración */
                            bool already = false;
                            uint64_t now_check = esp_timer_get_time();
                            int matched_index = -1;
                            for (int ci = 0; ci < configured_nodes_count; ci++) {
                                if (CO_LSS_ADDRESS_EQUAL(configured_nodes[ci].addr, last_found_address)) {
                                    already = true;
                                    matched_index = ci;
                                    break;
                                }
                            }
                            if (already) {
                                /* Ya fue configurado: intentar reasignarle su ID anterior para detectar reset
                                 * Si responde a la reasignación, significa que fue reseteado y está listo
                                 * Si no responde, entonces mantener el skip existente */
                                current_candidate_id = configured_nodes[matched_index].assigned_node_id;
                                configured_nodes[matched_index].skip_until_us = now_check + (uint64_t)CONFIGURED_NODE_SKIP_MS * 1000ULL;
                                ESP_LOGI(TAG, "Nodo detectado (serial ...%08" PRIX32 ") - intentando reasignar ID original %d.", (uint32_t)serial, current_candidate_id);
                                selection_verify_attempts = 0;
                                lssState = LSS_CONFIG_ID;
                            } else {
                                selection_verify_attempts = 0;
                                lssState = LSS_CONFIG_ID;
                            }
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
                    {
                        if (!log_config_id) {
                            ESP_LOGI(TAG, "Asignando ID empezando en %d...", current_candidate_id);
                            log_config_id = true;
                        }

                        if (is_node_id_used(current_candidate_id) || is_node_id_active_on_bus(current_candidate_id)) {
                            uint8_t next_free = find_next_free_node_id_active_aware();
                            ESP_LOGW(TAG, "ID %d ocupada en bus. Probando siguiente libre=%u", current_candidate_id, next_free);
                            current_candidate_id = next_free;
                            log_config_id = false;
                        }


                        CO_LSSmaster_return_t idret = CO_LSSmaster_configureNodeId(CO->LSSmaster, co_timer_us, current_candidate_id);


                        if (idret == CO_LSSmaster_OK) {
                            ESP_LOGI(TAG, "LSS: ID %d configurada correctamente.", current_candidate_id);
                            lssState = LSS_CONFIG_STORE;
                            log_config_store = false;
                        } else if (idret == CO_LSSmaster_OK_ILLEGAL_ARGUMENT) {
                            /* ID no válida o ocupada: intentar siguiente ID (no bloqueamos aquí) */
                            ESP_LOGW(TAG, "LSS: ID %d no válida/ocupada. Probando siguiente...", current_candidate_id);
                            /* avanzar candidato y evitar usar la ID del master */
                            current_candidate_id = find_next_free_node_id_active_aware();
                            if (++id_attempt_rounds > 126) {
                                ESP_LOGW(TAG, "LSS: No se encontró ID libre tras muchos intentos. Abortando asignación.");
                                lssState = LSS_DONE;
                            }
                        } else if (idret == CO_LSSmaster_WAIT_SLAVE) {
                            /* En progreso: ceder CPU un poco para que la tarea de transmisión procese envíos pendientes */
                            vTaskDelay(pdMS_TO_TICKS(10));
                        } else {
                            /* timeout u otro error -> reiniciar escaneo */
                            ESP_LOGW(TAG, "LSS: Error configureNodeId (%d). Volviendo a escanear.", idret);
                            lssState = LSS_INIT;
                        }
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
                            ESP_LOGI(TAG, "ID %d asignada y almacenada en el nodo. Enviando deselect y esperando breve tiempo antes de relanzar escaneo...", current_candidate_id);
                            /* Enviar deselect para que el esclavo aplique la ID */
                            CO_LSSmaster_swStateDeselect(CO->LSSmaster);


                            /* Añadimos la dirección a la lista de configurados SOLO si no existe ya */
                            bool already_in_list = false;
                            for (int ci = 0; ci < configured_nodes_count; ci++) {
                                if (CO_LSS_ADDRESS_EQUAL(configured_nodes[ci].addr, last_found_address)) {
                                    already_in_list = true;
                                    ESP_LOGD(TAG, "Nodo ya en lista, no se agrega duplicado");
                                    break;
                                }
                            }
                            
                            if (!already_in_list && configured_nodes_count < MAX_CONFIGURED_NODES) {
                                configured_nodes[configured_nodes_count].addr = last_found_address;
                                configured_nodes[configured_nodes_count].assigned_node_id = current_candidate_id;
                                configured_nodes[configured_nodes_count].skip_until_us = esp_timer_get_time() + (uint64_t)CONFIGURED_NODE_SKIP_MS * 1000ULL;
                                configured_nodes[configured_nodes_count].upload_launched = false;  // Inicializar flag de upload
                                configured_nodes_count++;
                                ESP_LOGI(TAG, "Guardado nodo configurado (serial ...%08" PRIX32 ") en lista (count=%d)", (uint32_t)last_found_address.addr[3], configured_nodes_count);
                            } else if (already_in_list) {
                                ESP_LOGD(TAG, "Nodo ya configurado previamente, skip");
                            } else {
                                ESP_LOGW(TAG, "Lista de nodos configurados llena, no se almacenó la dirección");
                            }

                            /* El uploader se lanzará después, cuando la red esté operativa en LSS_DONE */

                            /* Preparar siguiente candidato y esperar un breve tiempo no bloqueante */
                            next_id_to_assign = find_next_free_node_id_active_aware();
                            current_candidate_id = next_id_to_assign;
                            id_attempt_rounds = 0;
                            selection_verify_attempts = 0;


                            /* Esperar DESELECT_DELAY_MS antes de reiniciar el fast-scan para dar tiempo al esclavo */
                            last_deselected_us = esp_timer_get_time();
                            lssState = LSS_ACTIVATE; /* pasamos a activar espera corta */
                        } else if (sret == CO_LSSmaster_WAIT_SLAVE) {
                            /* En progreso: ceder CPU brevemente para procesar TX */
                            vTaskDelay(pdMS_TO_TICKS(10));
                        } else {
                            ESP_LOGW(TAG, "Store LSS sin ACK (%d). Volviendo a iniciar escaneo.", sret);
                            lssState = LSS_INIT;
                        }
                    }
                    break;


                case LSS_VERIFY_ID:
                    {
                        static int verify_attempts = 0;
                        uint32_t nodeid_val = 0;
                        CO_LSSmaster_return_t iret = CO_LSSmaster_Inquire(CO->LSSmaster, co_timer_us, CO_LSS_INQUIRE_NODE_ID, &nodeid_val);
                        if (iret == CO_LSSmaster_OK) {
                            uint8_t reported_id = (uint8_t)(nodeid_val & 0xFFU);
                            if (reported_id == current_candidate_id) {
                                ESP_LOGI(TAG, "Verificacion OK: nodo responde con ID %d.", reported_id);
                                verify_attempts = 0;
                                /* Ir a deseleccionar y luego reiniciar para siguientes nodos */
                                lssState = LSS_DESELECT;
                            } else {
                                ESP_LOGW(TAG, "Verificacion NOK: nodo responde con ID %d (esperaba %d). Reintentando...", reported_id, current_candidate_id);
                                /* Si el nodo no aplicó la ID, intentamos un par de veces antes de reiniciar escaneo */
                                if (++verify_attempts > 5) {
                                    ESP_LOGW(TAG, "Verificacion fallida repetida, relanzando escaneo.");
                                    verify_attempts = 0;
                                    lssState = LSS_INIT;
                                }
                            }
                        } else if (iret == CO_LSSmaster_WAIT_SLAVE) {
                            /* En progreso: ceder CPU un momento para que la transmisión avance */
                            vTaskDelay(pdMS_TO_TICKS(10));
                        } else {
                            ESP_LOGW(TAG, "Inquire node-id fallo (%d). Intentando deselección para proceder.", iret);
                            lssState = LSS_DESELECT;
                        }
                    }
                    break;


                case LSS_DESELECT:
                    {
                        /* Deselect node (reset selection) para que pueda detectarse un nuevo nodo */
                        CO_LSSmaster_return_t dret = CO_LSSmaster_swStateDeselect(CO->LSSmaster);
                        if (dret == CO_LSSmaster_OK || dret == CO_LSSmaster_INVALID_STATE) {
                            ESP_LOGI(TAG, "Nodo deseleccionado correctamente. Esperando %.0f ms para que aplique ID...", (double)DESELECT_DELAY_MS);
                            /* avanzar next_id_to_assign y preparar siguiente candidato */
                            next_id_to_assign = (current_candidate_id < 127) ? (uint8_t)(current_candidate_id + 1) : 2;
                            if (next_id_to_assign == MASTER_NODE_ID) next_id_to_assign++;
                            /* Reiniciar candidato para el siguiente ciclo */
                            current_candidate_id = next_id_to_assign;
                            id_attempt_rounds = 0;
                            /* guardamos tiempo para esperar antes de re-lanzar el fastscan */
                            last_deselected_us = esp_timer_get_time();
                            lssState = LSS_ACTIVATE;
                        } else {
                            ESP_LOGW(TAG, "LSS: fallo al deseleccionar nodo (%d). Volviendo a escanear.", dret);
                            /* Volver a iniciar escaneo para no quedar bloqueados */
                            lssState = LSS_INIT;
                        }
                    }
                    break;


                case LSS_ACTIVATE:
                    {
                        /* Esperamos un tiempo tras la deselección para que el esclavo aplique la ID.
                         * No hacemos selects adicionales para evitar saturación; tras el retraso
                         * relanzamos el fast-scan. */
                        if (last_deselected_us == 0) {
                            lssState = LSS_INIT;
                        } else {
                            uint64_t now = esp_timer_get_time();
                            if ((now - last_deselected_us) > (uint64_t)DESELECT_DELAY_MS * 1000ULL) {
                                ESP_LOGI(TAG, "Continuando: relanzando fast-scan tras deselect wait (%d ms).", DESELECT_DELAY_MS);
                                last_deselected_us = 0;
                                selection_verify_attempts = 0;
                                lssState = LSS_INIT;
                            } else {
                                /* todavía esperando */
                            }
                        }
                    }
                    break;

                case LSS_FORCE_SELECT:
                    {
                        CO_LSSmaster_return_t sret = CO_LSSmaster_swStateSelect(CO->LSSmaster, co_timer_us, NULL);
                        if (sret == CO_LSSmaster_OK) {
                            ESP_LOGW(TAG, "Forzando LSS global: re-asignando IDs por duplicados");
                            s_force_lss_reassign = false;
                            lssState = LSS_INIT;
                        } else if (sret == CO_LSSmaster_WAIT_SLAVE) {
                            vTaskDelay(pdMS_TO_TICKS(10));
                        } else {
                            ESP_LOGW(TAG, "LSS global select falló (%d). Reintentando...", sret);
                        }
                    }
                    break;


                case LSS_DONE:
                    // Operativo: envía NMT Start All periódicamente y vuelve a escanear cada RESCAN_INTERVAL_MS
                    static int nmt_timer = 0;
                    static uint64_t last_active_scan_us = 0;
                    uint64_t now_us = esp_timer_get_time();
                    const uint64_t ACTIVE_SCAN_INTERVAL_US = 30000000ULL; // 30 s (menos frecuente para no bloquear)
                    const uint8_t ACTIVE_SCAN_START_ID = ID_INICIO_ASIGNACION;
                    const uint8_t ACTIVE_SCAN_END_ID = 0x20;

                    if (s_force_lss_reassign && s_activeUploads == 0) {
                        ESP_LOGW(TAG, "Reasignación LSS solicitada. Forzando modo configuración.");
                        lssState = LSS_FORCE_SELECT;
                        break;
                    }

                    // Escanear nodos activos en el bus cada intervalo, pero CON TIMEOUT CORTO para no bloquear
                    if (now_us - last_active_scan_us > ACTIVE_SCAN_INTERVAL_US) {
                        if (s_activeUploads > 0) {
                            ESP_LOGD(TAG, "Uploads activos (%d); posponiendo escaneo de nodos activos", s_activeUploads);
                            last_active_scan_us = now_us;
                            break;
                        }
                        ESP_LOGI(TAG, "Escaneando nodos activos (IDs %u-%u) con timeout=100ms...", ACTIVE_SCAN_START_ID, ACTIVE_SCAN_END_ID);
                        bool duplicate_found = false;

                        for (uint8_t test_id = ACTIVE_SCAN_START_ID; test_id <= ACTIVE_SCAN_END_ID; test_id++) {
                            if (test_id == MASTER_NODE_ID) continue;

                            // Intenta tomar mutex con timeout corto (100ms) para no bloquear
                            if (sdoMutex == NULL) sdoMutex = xSemaphoreCreateMutex();
                            if (sdoMutex != NULL) {
                                // Timeout de 100ms (no portMAX_DELAY)
                                if (xSemaphoreTake(sdoMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                                    bool responds = sdo_probe_node(test_id);
                                    xSemaphoreGive(sdoMutex);

                                    if (responds) {
                                        ESP_LOGI(TAG, "Nodo activo detectado: ID=%d", test_id);

                                        // Buscar en lista de configurados
                                        bool found = false;
                                        for (int i = 0; i < configured_nodes_count; i++) {
                                            if (configured_nodes[i].assigned_node_id == test_id) {
                                                if (!configured_nodes[i].upload_launched) {
                                                    ESP_LOGI(TAG, "Lanzando uploader para nodo ID=%d", test_id);
                                                    if (start_firmware_upload(test_id)) {
                                                        configured_nodes[i].upload_launched = true;
                                                    }
                                                } else {
                                                    ESP_LOGD(TAG, "Uploader ya lanzado para ID=%d; verificando duplicados", test_id);
                                                    /* Sospecha de ID duplicada: relanzar fast-scan para reasignar un ID libre */
                                                    uint64_t now_dup = esp_timer_get_time();
                                                    if ((now_dup - s_lastDuplicateScan_us) > 5000000ULL) { /* 5s cooldown */
                                                        uint8_t next_free = find_next_free_node_id();
                                                        ESP_LOGW(TAG, "Posible ID duplicado en el bus (ID=%d). Relanzando fast-scan para reasignar al siguiente libre=%u", test_id, next_free);
                                                        next_id_to_assign = next_free;
                                                        lssState = LSS_INIT;
                                                        duplicate_found = true;
                                                        s_lastDuplicateScan_us = now_dup;
                                                    }
                                                }
                                                found = true;
                                                break;
                                            }
                                        }

                                        // Si no está en la lista, agregarlo y lanzar uploader
                                        if (!found && configured_nodes_count < MAX_CONFIGURED_NODES) {
                                            memset(&configured_nodes[configured_nodes_count].addr, 0, sizeof(CO_LSS_address_t));
                                            configured_nodes[configured_nodes_count].assigned_node_id = test_id;
                                            configured_nodes[configured_nodes_count].skip_until_us = 0;
                                            configured_nodes[configured_nodes_count].upload_launched = false;
                                            configured_nodes_count++;
                                            ESP_LOGI(TAG, "Lanzando uploader para nodo activo ID=%d", test_id);
                                            if (start_firmware_upload(test_id)) {
                                                configured_nodes[configured_nodes_count - 1].upload_launched = true;
                                            }
                                        }

                                        vTaskDelay(pdMS_TO_TICKS(20));
                                    }
                                } else {
                                    // Timeout tomando mutex, saltar este nodo
                                    ESP_LOGD(TAG, "ID=%d: mutex timeout, saltando.", test_id);
                                }
                            }
                            vTaskDelay(pdMS_TO_TICKS(1));
                            if (duplicate_found) break;
                        }

                        last_active_scan_us = now_us;
                        ESP_LOGI(TAG, "Escaneo activo completado. Nodos configurados: %d", configured_nodes_count);
                    }

                    if (nmt_timer++ > (1000 / MAIN_INTERVAL_MS)) { // approx every 1s
                        ESP_LOGI(TAG, "Red Operativa. Enviando NMT Start All.");
                        CO_NMT_sendCommand(CO->NMT, CO_NMT_ENTER_OPERATIONAL, 0);
                        nmt_timer = 0;
                    }
                    if ((now_us - last_rescan_us_global) > (uint64_t)RESCAN_INTERVAL_MS * 1000ULL) {
                        last_rescan_us_global = now_us;
                        ESP_LOGD(TAG, "Re-lanzando fast-scan para detectar nuevos nodos.");
                        lssState = LSS_INIT;
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
        #if MASTER_ENABLE_PDO
        CO_process_RPDO(CO, syncWas, co_timer_us, NULL);
        CO_process_TPDO(CO, syncWas, co_timer_us, NULL);
        #endif
        #if (CO_CONFIG_HB_CONS) & CO_CONFIG_HB_CONS_ENABLE
        CO_HBconsumer_process(CO->HBcons, true, co_timer_us, NULL);
        #endif
    }
}
