#include "CANopenNode_ESP32.h"
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern CO_t *CO;

void app_main(void) {
    CO_ESP32_init();

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    while (1) {
        // --------- TRANSMITIR PDO (TPDO1) -----------
        // Escribe el dato en el buffer del TPDO mapeado
        uint8_t myValue = 42;
        CO->TPDO[0]->mapPointer[0] = myValue; // primer byte del mapping
        CO_TPDOsend(CO->TPDO[0]); // fuerza el envío inmediato del TPDO1

        // ---------- RECIBIR PDO (RPDO1) --------------
        // Lee el dato recibido del buffer del RPDO mapeado
        uint8_t valueIn = CO->RPDO[0]->mapPointer[0];
        printf("Valor recibido en RPDO1: %d\n", valueIn);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
