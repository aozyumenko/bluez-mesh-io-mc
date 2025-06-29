/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2025  Alexander Ozumenko <scg@stdio.ru>.
 *
 *
 */

#include <stdint.h>



#ifndef __packed
#define __packed __attribute__((packed))
#endif


#define MC_SERIAL_BAUD_RATE             (1000000)

#define MC_API_VERSION                  (0x0001)

#define MC_PACKET_LENGTH_OVERHEAD       (1UL)
#define MC_PACKET_OVERHEAD              (2UL - MC_PACKET_LENGTH_OVERHEAD)
#define MC_PACKET_PAYLOAD_MAXLEN        (UINT8_MAX - MC_PACKET_OVERHEAD)

#define MC_CMD_OVERHEAD                 (4UL)
#define MC_CMD_PAYLOAD_MAXLEN           (MC_PACKET_PAYLOAD_MAXLEN - MC_CMD_OVERHEAD)

#define MC_MAX_PACKET_SIZE              (UINT8_MAX + MC_PACKET_LENGTH_OVERHEAD)
#define MC_MAX_ENCODED_PACKET_SIZE      (1UL + (MC_MAX_PACKET_SIZE * 2) + 1UL)

#define MC_MESH_UUID_SIZE               (16)

#define DEVICEADDR_LEN                  (6)



/* Mesh Controller interface BLE Advertising Send command */

typedef struct __attribute((packed))
{
    uint8_t data[MC_CMD_PAYLOAD_MAXLEN];
} mc_cmd_ble_ad_data_t;


/** Mesh Controller interface commands */
typedef struct __attribute((packed))
{
    uint32_t token;
    union __attribute((packed)) {
        mc_cmd_ble_ad_data_t ble_ad_data;
    } payload;
} mc_cmd_t;


/* Mesh Contoller commands response */

/* command response data with version information */
typedef struct __attribute((packed))
{
    uint16_t ver;
} mc_evt_cmd_rsp_data_version_t;

/* command response data with device UUID */
typedef struct __attribute((packed))
{
    uint8_t addr[DEVICEADDR_LEN];
} mc_evt_cmd_rsp_data_deviceaddr_t;

/* Mesh Controller housekeeping data */
typedef struct __attribute((packed))
{
    uint32_t alloc_fail_count;
    uint32_t rx_fail_count;
} mc_evt_cmd_rsp_data_housekeeping_t;


/* Mesh Controller interface events */

typedef struct __attribute((packed))
{
    uint8_t opcode;
    uint32_t token;
    uint8_t status;
#define MC_STATUS_SUCCESS                       (0x00)
#define MC_STATUS_ERROR_UNKNOWN                 (0x80)
#define MC_STATUS_ERROR_INTERNAL                (0x81)
#define MC_STATUS_ERROR_CMD_UNKNOWN             (0x82)
#define MC_STATUS_ERROR_INVALID_STATE           (0x83)
#define MC_STATUS_ERROR_INVALID_LENGTH          (0x84)
#define MC_STATUS_ERROR_INVALID_PARAMETER       (0x85)
#define MC_STATUS_ERROR_BUSY                    (0x86)
#define MC_STATUS_ERROR_INVALID_DATA            (0x87)
#define MC_STATUS_ERROR_REJECTED                (0x8e)
#define MC_STATUS_ERROR_TIMEOUT                 (0x93)
#define MC_STATUS_ERROR_INVALID_KEY_DATA        (0x98)
    union __packed {
        mc_evt_cmd_rsp_data_version_t version;
        mc_evt_cmd_rsp_data_deviceaddr_t deviceaddr;
        mc_evt_cmd_rsp_data_housekeeping_t hk_data;
    } data;
} mc_evt_cmd_rsp_t;

typedef struct __packed {
    uint8_t operating_mode;
#define MC_DEVICE_OPERATING_MODE_TEST           (0)
#define MC_DEVICE_OPERATING_MODE_BOOTLOADER     (1)
#define MC_DEVICE_OPERATING_MODE_APPLICATION    (2)
    uint8_t hw_error;
    uint8_t data_credit_available;
} mc_evt_device_started_t;

typedef struct __attribute((packed))
{
    uint8_t data[MC_PACKET_PAYLOAD_MAXLEN];
} mc_evt_ble_ad_data_t;


typedef union __attribute((packed))
{
    mc_evt_cmd_rsp_t cmd_rsp;
    mc_evt_device_started_t started;
    mc_evt_ble_ad_data_t ble_ad_data;
} mc_evt_t;


/* Mesh Controller interface packet */
typedef struct __packed {
    uint8_t length;
    uint8_t opcode;
#define MC_OPCODE_CMD_RESET                     (0x01)
#define MC_OPCODE_CMD_VERSION_GET               (0x02)
#define MC_OPCODE_CMD_START                     (0x03)
#define MC_OPCODE_CMD_STOP                      (0x04)
#define MC_OPCODE_CMD_BLE_AD_DATA_SEND          (0x05)
#define MC_OPCODE_CMD_DEVICEADDR_GET            (0x06)
#define MC_OPCODE_CMD_HOUSEKEEPING_DATA_GET     (0x7e)
#define MC_OPCODE_CMD_HOUSEKEEPING_DATA_CLEAR   (0x7f)

#define MC_OPCODE_EVT_DEVICE_STARTED            (0x81)
#define MC_OPCODE_EVT_CMD_RSP                   (0x82)
#define MC_OPCODE_EVT_BLE_AD_DATA_RECEIVED      (0x83)

    union __attribute((packed))
    {
        mc_cmd_t cmd;
        mc_evt_t evt;
    } payload;
} mc_packet_t;
