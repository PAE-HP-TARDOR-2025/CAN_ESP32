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
    LSS_VERIFY_ID, /* verify node ID after store */
    LSS_DESELECT, /* deselect node after store */
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
static CO_LSS_address_t last_found_address; /* dirección LSS del último nodo detectado */
static uint8_t next_id_to_assign = ID_INICIO_ASIGNACION;
static uint64_t scan_start_us = 0;
static uint32_t cached_vendor = 0;
static uint32_t cached_product = 0;


// Lista simple de nodos ya configurados en runtime para evitar reconfigurar el mismo serial
#define MAX_CONFIGURED_NODES 32
// Tiempo que evitaremos reintentar reconfigurar la misma dirección (ms)
#define CONFIGURED_NODE_SKIP_MS 30000


typedef struct {
    CO_LSS_address_t addr;
    uint64_t skip_until_us; /* until what time to skip (esp_timer_get_time() units) */
    uint8_t assigned_node_id; /* ID previously assigned to this node */
} configured_node_t;


static configured_node_t configured_nodes[MAX_CONFIGURED_NODES];
static int configured_nodes_count = 0;
/* Global rescan timestamp used by LSS_DONE state to schedule next rescan */
static uint64_t last_rescan_us_global = 0;


// Re-scan interval (ms) para preguntar por nodos nuevos conectados posteriormente
#define RESCAN_INTERVAL_MS 5000
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
                            bool skip_active = false;
                            uint64_t now_check = esp_timer_get_time();
                            int matched_index = -1;
                            for (int ci = 0; ci < configured_nodes_count; ci++) {
                                if (CO_LSS_ADDRESS_EQUAL(configured_nodes[ci].addr, last_found_address)) {
                                    already = true;
                                    matched_index = ci;
                                    if (configured_nodes[ci].skip_until_us > now_check) skip_active = true;
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


                        CO_LSSmaster_return_t idret = CO_LSSmaster_configureNodeId(CO->LSSmaster, co_timer_us, current_candidate_id);


                        if (idret == CO_LSSmaster_OK) {
                            ESP_LOGI(TAG, "LSS: ID %d configurada correctamente.", current_candidate_id);
                            lssState = LSS_CONFIG_STORE;
                            log_config_store = false;
                        } else if (idret == CO_LSSmaster_OK_ILLEGAL_ARGUMENT) {
                            /* ID no válida o ocupada: intentar siguiente ID (no bloqueamos aquí) */
                            ESP_LOGW(TAG, "LSS: ID %d no válida/ocupada. Probando siguiente...", current_candidate_id);
                            /* avanzar candidato y evitar usar la ID del master */
                            current_candidate_id = (current_candidate_id < 127) ? (uint8_t)(current_candidate_id + 1) : 2;
                            if (current_candidate_id == MASTER_NODE_ID) current_candidate_id++;
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


                            /* Añadimos la dirección a la lista de configurados para evitar volver a tocarla */
                            if (configured_nodes_count < MAX_CONFIGURED_NODES) {
                                configured_nodes[configured_nodes_count].addr = last_found_address;
                                configured_nodes[configured_nodes_count].assigned_node_id = current_candidate_id;
                                configured_nodes[configured_nodes_count].skip_until_us = esp_timer_get_time() + (uint64_t)CONFIGURED_NODE_SKIP_MS * 1000ULL;
                                configured_nodes_count++;
                                ESP_LOGI(TAG, "Guardado nodo configurado (serial ...%08" PRIX32 ") en lista (count=%d)", (uint32_t)last_found_address.addr[3], configured_nodes_count);
                            } else {
                                ESP_LOGW(TAG, "Lista de nodos configurados llena, no se almacenó la dirección");
                            }


                            /* Preparar siguiente candidato y esperar un breve tiempo no bloqueante */
                            next_id_to_assign = (current_candidate_id < 127) ? (uint8_t)(current_candidate_id + 1) : 2;
                            if (next_id_to_assign == MASTER_NODE_ID) next_id_to_assign++;
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


                case LSS_DONE:
                    // Operativo: envía NMT Start All periódicamente y vuelve a escanear cada RESCAN_INTERVAL_MS
                    static int nmt_timer = 0;
                    uint64_t now_us = esp_timer_get_time();
                    if (nmt_timer++ > (1000 / MAIN_INTERVAL_MS)) { // approx every 1s
                        ESP_LOGI(TAG, "Red Operativa. Enviando NMT Start All.");
                        CO_NMT_sendCommand(CO->NMT, CO_NMT_ENTER_OPERATIONAL, 0);
                        nmt_timer = 0;
                    }
                    if ((now_us - last_rescan_us_global) > (uint64_t)RESCAN_INTERVAL_MS * 1000ULL) {
                        last_rescan_us_global = now_us;
                        ESP_LOGI(TAG, "Re-lanzando fast-scan para detectar nuevos nodos.");
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
        CO_process_RPDO(CO, syncWas, co_timer_us, NULL);
        CO_process_TPDO(CO, syncWas, co_timer_us, NULL);
        #if (CO_CONFIG_HB_CONS) & CO_CONFIG_HB_CONS_ENABLE
        CO_HBconsumer_process(CO->HBcons, true, co_timer_us, NULL);
        #endif
    }
}
