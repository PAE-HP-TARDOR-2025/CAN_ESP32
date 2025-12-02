#ifndef CANOPEN_LSS_H
#define CANOPEN_LSS_H

#include <stdint.h>

/**
 * @brief Arranca el nodo CANopen con toda la lógica integrada.
 * Incluye: LSS, Storage, Monitor de Tráfico, Botón y Envío Dummy.
 * @param pendingBitRate Velocidad inicial (ej. 500)
 * @param pendingNodeId  ID inicial (0xFF para LSS, 0x20 para manual)
 */
void CO_ESP32_LSS_Run(uint16_t pendingBitRate, uint8_t pendingNodeId);

#endif