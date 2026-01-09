#ifndef FW_MASTER_UPDATE_H
#define FW_MASTER_UPDATE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Image families supported by the master uploader. */
typedef enum {
    FW_IMAGE_MAIN = 0,
    FW_IMAGE_BOOTLOADER = 1,
    FW_IMAGE_CONFIG = 2
} fw_image_type_t;

/* High-level upload description shared across helper functions. */
typedef struct {
    const char* firmwarePath;
    fw_image_type_t type;
    uint8_t targetBank;
    uint8_t targetNodeId;
    uint32_t maxChunkBytes;
    uint16_t expectedCrc;
    uint16_t firmwareVersion;  /* Firmware version to send and check */
} fw_upload_plan_t;

/* Payload buffer returned by the file loader. */
typedef struct {
    uint8_t* buffer;
    size_t size;
} fw_payload_t;

bool fw_master_load_payload(const fw_upload_plan_t* plan, fw_payload_t* payload);
uint16_t fw_master_crc16(const uint8_t* data, size_t len);
bool fw_master_send_metadata(const fw_upload_plan_t* plan, const fw_payload_t* payload, uint16_t crc);
bool fw_master_send_start_command(const fw_upload_plan_t* plan);
bool fw_master_send_chunk(const fw_upload_plan_t* plan, const uint8_t* chunk, size_t len, size_t offset);
bool fw_master_send_finalize_request(const fw_upload_plan_t* plan, uint16_t crc);
bool fw_master_stream_payload(const fw_upload_plan_t* plan, const fw_payload_t* payload);
bool fw_master_run_upload_session(const fw_upload_plan_t* plan);

/**
 * Query the slave's running firmware CRC via SDO upload from 0x1F5B:01.
 * Returns the CRC in *slaveCrc, or false if the query failed.
 * Implement the actual SDO upload in place of the stub.
 */
bool fw_master_query_slave_crc(const fw_upload_plan_t* plan, uint16_t* slaveCrc);

/**
 * Query the slave's running firmware version via SDO upload from 0x1F5C:01.
 * Returns the version in *slaveVersion, or false if the query failed.
 * Implement the actual SDO upload in place of the stub.
 */
bool fw_master_query_slave_version(const fw_upload_plan_t* plan, uint16_t* slaveVersion);

/**
 * Smart wrapper: queries slave CRC and version first; skips upload if both match.
 * Returns true if firmware is up-to-date or upload succeeded.
 */
bool fw_master_run_upload_if_needed(const fw_upload_plan_t* plan);

#ifdef __cplusplus
}
#endif

#endif /* FW_MASTER_UPDATE_H */
