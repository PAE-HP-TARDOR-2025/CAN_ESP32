/* app_main.c */
#include "CANopenNode_ESP32.h"
#include "OD.h"                /* donde has declarado extern uint8_t OD_0x2000_0; */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Variables expuestas desde OD.c (acordar con lluc)*/
extern uint8_t OD_0x2000_0;
extern uint8_t OD_0x2001_0; /* si quieres leer otro valor de OD */

void app_main(void)
{
    /* Inicializa las tareas/stack CANopen */
    /*Esta función crea y lanza una tarea FreeRTOS en segundo plano, llamada CO_mainTask.*/
    CO_ESP32_init();

    /* Esperar hasta que el CO esté inicializado y CAN en modo normal.
       Mejor comprobar periódicamente en lugar de vTaskDelay fijo. */
    CO_t *CO = NULL;
    const TickType_t timeout = pdMS_TO_TICKS(5000);
    TickType_t start = xTaskGetTickCount();
    while (1) {
        CO = CO_ESP32_getCO();
        if (CO != NULL && CO->CANmodule != NULL && CO->CANmodule->CANnormal) {
            break; /* stack listo */
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        if ((xTaskGetTickCount() - start) > timeout) {
            printf("Warning: CANopen stack not ready after 5s, continuing anyway\n");
            break;
        }
    }

    printf("CANopen stack inicializado (puntero CO=%p)\n", (void*)CO);

    while (1) {
        /* Simulamos lectura del sensor (valor pasado como constante) */
        uint8_t valor_sensor = 123; 

        /* Escribir en la variable mapeada en el OD.
           Protegemos con una zona crítica simple para evitar races. */
        taskENTER_CRITICAL();
        OD_0x2000_0 = valor_sensor;
        taskEXIT_CRITICAL();

        /* Si tienes un TPDO mapeado a 0x2000 sub 0, el stack lo enviará
           en el siguiente proceso TPDO (dependiendo de configuración). */

        /* Leer otro valor del OD (ejemplo) */
        uint8_t recibido;
        taskENTER_CRITICAL();
        recibido = OD_0x2001_0;
        taskEXIT_CRITICAL();

        printf("Escrito a OD[0x2000]=%u  - Leído OD[0x2001]=%u\n", valor_sensor, recibido);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
