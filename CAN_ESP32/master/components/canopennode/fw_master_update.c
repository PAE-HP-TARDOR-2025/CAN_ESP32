#include "fw_master_update.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef FW_MASTER_LOG
#define FW_MASTER_LOG(fmt, ...) printf("[FW-MASTER] " fmt, ##__VA_ARGS__)
#endif

#ifndef FW_MASTER_ERR
#define FW_MASTER_ERR(fmt, ...) printf("[FW-ERROR ] " fmt, ##__VA_ARGS__)
#endif

#define RETURN_IF_FALSE(cond, msg, ...)                                                              \
    do {                                                                                             \
        if (!(cond)) {                                                                               \
            FW_MASTER_ERR(msg "\n", ##__VA_ARGS__);                                                 \
            return false;                                                                            \
        }                                                                                            \
    } while (0)

static bool fw_master_file_info(const fw_upload_plan_t* plan, size_t* outSize) {
    FILE* f = fopen(plan->firmwarePath, "rb");
    RETURN_IF_FALSE(f != NULL, "Cannot open firmware file %s", plan->firmwarePath);
    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        FW_MASTER_ERR("Failed to seek to end of %s\n", plan->firmwarePath);
        return false;
    }
    long fileSize = ftell(f);
    if (fileSize <= 0) {
        fclose(f);
        FW_MASTER_ERR("Firmware file %s is empty\n", plan->firmwarePath);
        return false;
    }
    *outSize = (size_t)fileSize;
    fclose(f);
    return true;
}

static bool fw_master_crc_from_file(const fw_upload_plan_t* plan, uint16_t* outCrc, size_t* outSize) {
    FILE* f = fopen(plan->firmwarePath, "rb");
    RETURN_IF_FALSE(f != NULL, "Cannot open firmware file %s", plan->firmwarePath);
    uint16_t crc = 0xFFFFU;
    size_t total = 0;
    uint8_t buf[1024];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
        total += n;
        for (size_t i = 0; i < n; i++) {
            crc ^= (uint16_t)buf[i] << 8;
            for (int bit = 0; bit < 8; bit++) {
                if (crc & 0x8000U) {
                    crc = (uint16_t)((crc << 1) ^ 0x1021U);
                } else {
                    crc <<= 1;
                }
            }
        }
    }
    fclose(f);
    RETURN_IF_FALSE(total > 0, "Firmware file %s is empty", plan->firmwarePath);
    *outCrc = crc;
    if (outSize) *outSize = total;
    return true;
}

uint16_t
fw_master_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

__attribute__((weak)) bool
fw_master_send_metadata(const fw_upload_plan_t* plan, const fw_payload_t* payload, uint16_t crc) {
    FW_MASTER_LOG("Sending metadata to slave node %u\n", plan->targetNodeId);
    FW_MASTER_LOG(" - image bytes : %zu\n", payload->size);
    FW_MASTER_LOG(" - crc         : 0x%04X\n", crc);
    FW_MASTER_LOG(" - image type  : %u\n", plan->type);
    FW_MASTER_LOG(" - bank        : %u\n", plan->targetBank);
    FW_MASTER_LOG(" - version     : %u\n", plan->firmwareVersion);

    /* Replace this stub with real CO_SDOclientDownload* calls to index 0x1F57.
     * Metadata format: [size(4) | crc(2) | type(1) | bank(1) | version(2)] = 10 bytes */
    bool linkOk = true;
    RETURN_IF_FALSE(linkOk, "Metadata write failed (stub)");
    return true;
}

__attribute__((weak)) bool
fw_master_send_start_command(const fw_upload_plan_t* plan) {
    FW_MASTER_LOG("Issuing start command through object 0x1F51\n");
    /* Replace with real SDO write of the start token. */
    bool linkOk = true;
    RETURN_IF_FALSE(linkOk, "Control write failed (stub)");
    return true;
}

__attribute__((weak)) bool
fw_master_send_chunk(const fw_upload_plan_t* plan, const uint8_t* chunk, size_t len, size_t offset) {
    (void)plan;
    FW_MASTER_LOG("Sending chunk offset %zu size %zu\n", offset, len);
    /* Replace with block download segments (CO_SDOclientDownload). */
    bool linkOk = true;
    RETURN_IF_FALSE(linkOk, "Chunk transfer failed (stub)");
    return true;
}

__attribute__((weak)) bool
fw_master_send_finalize_request(const fw_upload_plan_t* plan, uint16_t crc) {
    (void)plan;
    FW_MASTER_LOG("Sending finalize request with crc 0x%04X\n", crc);
    bool linkOk = true;
    RETURN_IF_FALSE(linkOk, "Finalize write failed (stub)");
    return true;
}

static bool fw_master_stream_file(const fw_upload_plan_t* plan, size_t fileSize) {
    FILE* f = fopen(plan->firmwarePath, "rb");
    RETURN_IF_FALSE(f != NULL, "Cannot open firmware file %s", plan->firmwarePath);

    size_t chunkSize = plan->maxChunkBytes;
    if (chunkSize == 0 || chunkSize > 1024) chunkSize = 1024; /* sanity */

    uint8_t* buf = (uint8_t*)malloc(chunkSize);
    RETURN_IF_FALSE(buf != NULL, "Out of memory allocating chunk buffer (%zu)", chunkSize);

    size_t offset = 0;
    size_t n;
    bool ok = true;
    size_t next_log = 0;
    while ((n = fread(buf, 1, chunkSize, f)) > 0) {
        if (!fw_master_send_chunk(plan, buf, n, offset)) {
            ok = false;
            break;
        }
        offset += n;
        if (offset >= next_log || offset == fileSize) {
            FW_MASTER_LOG("Progress: %zu/%zu bytes (%.1f%%)\n", offset, fileSize,
                          (fileSize > 0) ? (100.0 * (double)offset / (double)fileSize) : 0.0);
            next_log = offset + 32768; /* log roughly cada 32 KB */
        }
    }
    free(buf);
    fclose(f);

    if (ok && offset != fileSize) {
        FW_MASTER_ERR("Short stream: expected %zu bytes, sent %zu\n", fileSize, offset);
        ok = false;
    }
    return ok;
}

bool
fw_master_run_upload_session(const fw_upload_plan_t* plan) {
    size_t fileSize = 0;
    RETURN_IF_FALSE(fw_master_file_info(plan, &fileSize), "Cannot stat firmware file");

    uint16_t crc = plan->expectedCrc;
    if (crc == 0U) {
        RETURN_IF_FALSE(fw_master_crc_from_file(plan, &crc, NULL), "CRC calc failed");
        FW_MASTER_LOG("Auto-computed crc: 0x%04X\n", crc);
    }

    fw_payload_t payload = {.buffer = NULL, .size = fileSize};
    bool ok = fw_master_send_metadata(plan, &payload, crc) && fw_master_send_start_command(plan) &&
              fw_master_stream_file(plan, fileSize) && fw_master_send_finalize_request(plan, crc);
    return ok;
}

__attribute__((weak)) bool
fw_master_query_slave_crc(const fw_upload_plan_t* plan, uint16_t* slaveCrc) {
    FW_MASTER_LOG("Querying running firmware CRC from slave node %u (0x1F5B:01)\n", plan->targetNodeId);
    /* TODO: Replace this stub with a real SDO upload from index 0x1F5B subindex 1.
     * On success, parse the 2-byte response into *slaveCrc and return true.
     * On failure (e.g., object does not exist or timeout), return false.
     */
    (void)slaveCrc;
    return false; /* stub always fails so upload proceeds */
}

__attribute__((weak)) bool
fw_master_query_slave_version(const fw_upload_plan_t* plan, uint16_t* slaveVersion) {
    FW_MASTER_LOG("Querying running firmware version from slave node %u (0x1F5C:01)\n", plan->targetNodeId);
    /* TODO: Replace this stub with a real SDO upload from index 0x1F5C subindex 1.
     * On success, parse the 2-byte response into *slaveVersion and return true.
     * On failure (e.g., object does not exist or timeout), return false.
     */
    (void)slaveVersion;
    return false; /* stub always fails so upload proceeds */
}

bool
fw_master_run_upload_if_needed(const fw_upload_plan_t* plan) {
    size_t fileSize = 0;
    RETURN_IF_FALSE(fw_master_file_info(plan, &fileSize), "Cannot stat firmware file");

    uint16_t localCrc = plan->expectedCrc;
    if (localCrc == 0U) {
        RETURN_IF_FALSE(fw_master_crc_from_file(plan, &localCrc, NULL), "CRC calc failed");
        FW_MASTER_LOG("Local firmware CRC: 0x%04X\n", localCrc);
    }

    uint16_t localVersion = plan->firmwareVersion;
    FW_MASTER_LOG("Local firmware version: %u\n", localVersion);

    /* Query slave CRC and version - upload only if BOTH match */
    uint16_t slaveCrc = 0U;
    uint16_t slaveVersion = 0U;
    FW_MASTER_LOG("Querying slave CRC/version...\n");
    bool crcQueried = fw_master_query_slave_crc(plan, &slaveCrc);
    bool verQueried = fw_master_query_slave_version(plan, &slaveVersion);

    if (crcQueried && verQueried) {
        FW_MASTER_LOG("Slave running: CRC=0x%04X, version=%u\n", slaveCrc, slaveVersion);
        if (slaveCrc == localCrc && slaveVersion == localVersion) {
            FW_MASTER_LOG("Slave firmware matches (CRC=0x%04X, ver=%u); skipping upload.\n", slaveCrc, slaveVersion);
            return true;
        }
        if (slaveCrc == localCrc) {
            FW_MASTER_LOG("CRC matches but version differs (%u vs %u); uploading.\n", slaveVersion, localVersion);
        } else if (slaveVersion == localVersion) {
            FW_MASTER_LOG("Version matches but CRC differs (0x%04X vs 0x%04X); uploading.\n", slaveCrc, localCrc);
        } else {
            FW_MASTER_LOG("Both CRC and version differ; uploading.\n");
        }
    } else if (crcQueried) {
        FW_MASTER_LOG("Slave CRC=0x%04X (version query failed); proceeding with upload.\n", slaveCrc);
    } else if (verQueried) {
        FW_MASTER_LOG("Slave version=%u (CRC query failed); proceeding with upload.\n", slaveVersion);
    } else {
        FW_MASTER_LOG("Could not query slave CRC or version; proceeding with upload.\n");
    }

    fw_payload_t payload = {.buffer = NULL, .size = fileSize};
    FW_MASTER_LOG("Sending metadata...\n");
    bool ok = fw_master_send_metadata(plan, &payload, localCrc);
    if (!ok) return false;
    FW_MASTER_LOG("Sending start command...\n");
    ok = fw_master_send_start_command(plan);
    if (!ok) return false;
    FW_MASTER_LOG("Streaming firmware (%zu bytes)...\n", fileSize);
    ok = fw_master_stream_file(plan, fileSize);
    if (!ok) return false;
    FW_MASTER_LOG("Sending finalize request...\n");
    ok = fw_master_send_finalize_request(plan, localCrc);
    return ok;
}
