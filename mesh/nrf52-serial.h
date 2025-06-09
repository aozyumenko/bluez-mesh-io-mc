#include <stdint.h>



#ifndef __packed
#define __packed __attribute__((packed))
#endif

#define FLOW_CTL        0x0001
//#define BAUD_RATE       1000000
#define BAUD_RATE       460800


/* serial protocol version */
#define SERIAL_API_VERSION              (0x0001)


/* serial interface defines */
#define NRF_MESH_SERIAL_PACKET_LENGTH_OVERHEAD  (1UL)
#define NRF_MESH_SERIAL_PACKET_OVERHEAD         (2UL - NRF_MESH_SERIAL_PACKET_LENGTH_OVERHEAD)
#define NRF_MESH_SERIAL_PACKET_PAYLOAD_MAXLEN   (UINT8_MAX - NRF_MESH_SERIAL_PACKET_OVERHEAD)

#define NRF_MESH_SERIAL_CMD_OVERHEAD            (4UL)
#define NRF_MESH_SERIAL_CMD_PAYLOAD_MAXLEN      (NRF_MESH_SERIAL_PACKET_PAYLOAD_MAXLEN - NRF_MESH_SERIAL_CMD_OVERHEAD)

#define NRF_SERIAL_MAX_PACKET_SIZE              (UINT8_MAX + NRF_MESH_SERIAL_PACKET_LENGTH_OVERHEAD)
#define NRF_SERIAL_MAX_ENCODED_PACKET_SIZE      (1UL + (NRF_SERIAL_MAX_PACKET_SIZE * 2) + 1UL)

#define NRF_MESH_UUID_SIZE                      (16)


/** BLE Advertising command packet. */
typedef struct __attribute((packed))
{
    uint8_t data[NRF_MESH_SERIAL_CMD_PAYLOAD_MAXLEN];
} serial_cmd_ble_ad_data_t;



typedef struct __attribute((packed))
{
    uint32_t token;     /**< Unique ID of the command. */
    union __attribute((packed)) {
        serial_cmd_ble_ad_data_t ble_ad_data;
    } payload;
} serial_cmd_t;




/* Serial interface common types */

/* Serial interface BLE Advertising Send command */

typedef struct __attribute((packed))
{
    uint8_t data[NRF_MESH_SERIAL_CMD_PAYLOAD_MAXLEN];
} nrf_serial_cmd_ble_ad_data_t;


/** Serial interface commands */
typedef struct __attribute((packed))
{
    uint32_t token;     /**< Unique ID of the command. */
    union __attribute((packed)) {
        nrf_serial_cmd_ble_ad_data_t ble_ad_data;
    } payload;
} nrf_serial_cmd_t;


/* nRF52 serial proxy commands response */

/* command response data with version information */
typedef struct __attribute((packed))
{
    uint16_t serial_ver;
} nrf_serial_evt_cmd_rsp_data_serial_version_t;

/* command response data with device UUID */
typedef struct __attribute((packed))
{
    uint8_t device_uuid[NRF_MESH_UUID_SIZE];
} serial_evt_cmd_rsp_data_device_uuid_t;

/* serial interface housekeeping data. */
typedef struct __attribute((packed))
{
    uint32_t alloc_fail_count;  /**< Number of failed serial packet allocations. */
    uint32_t rx_fail_count;
} nrf_serial_evt_cmd_rsp_data_housekeeping_t;


/* Serial interface events */

typedef struct __attribute((packed))
{
    uint8_t opcode;
    uint32_t token;
    uint8_t status;
#define SERIAL_STATUS_SUCCESS                   0x00
#define SERIAL_STATUS_ERROR_UNKNOWN             0x80
#define SERIAL_STATUS_ERROR_INTERNAL            0x81
#define SERIAL_STATUS_ERROR_CMD_UNKNOWN         0x82
#define SERIAL_STATUS_ERROR_INVALID_STATE       0x83
#define SERIAL_STATUS_ERROR_INVALID_LENGTH      0x84
#define SERIAL_STATUS_ERROR_INVALID_PARAMETER   0x85
#define SERIAL_STATUS_ERROR_BUSY                0x86
#define SERIAL_STATUS_ERROR_INVALID_DATA        0x87
#define SERIAL_STATUS_ERROR_REJECTED            0x8e
#define SERIAL_STATUS_ERROR_TIMEOUT             0x93
#define SERIAL_STATUS_ERROR_INVALID_KEY_DATA    0x98
    union __packed {
        nrf_serial_evt_cmd_rsp_data_serial_version_t    serial_version;
        serial_evt_cmd_rsp_data_device_uuid_t           device_uuid;
        nrf_serial_evt_cmd_rsp_data_housekeeping_t      hk_data;
    } data;
} nrf_serial_evt_cmd_rsp_t;

typedef struct __packed {
    uint8_t operating_mode;
#define SERIAL_DEVICE_OPERATING_MODE_TEST               0
#define SERIAL_DEVICE_OPERATING_MODE_BOOTLOADER         1
#define SERIAL_DEVICE_OPERATING_MODE_APPLICATION        2
    uint8_t hw_error;
    uint8_t data_credit_available;
} nrf_serial_evt_device_started_t;

typedef struct __attribute((packed))
{
    uint8_t data[NRF_MESH_SERIAL_PACKET_PAYLOAD_MAXLEN];
} nrf_serial_evt_ble_ad_data_t;


typedef union __attribute((packed))
{
    nrf_serial_evt_cmd_rsp_t            cmd_rsp;
    nrf_serial_evt_device_started_t     started;
    nrf_serial_evt_ble_ad_data_t        ble_ad_data;
} nrf_serial_evt_t;


/* Serial interface packet */
typedef struct __packed {
    uint8_t length;
    uint8_t opcode;
#define SERIAL_OPCODE_CMD_RESET                         (0x01)
#define SERIAL_OPCODE_CMD_SERIAL_VERSION_GET            (0x02)
#define SERIAL_OPCODE_CMD_START                         (0x03)
#define SERIAL_OPCODE_CMD_STOP                          (0x04)
#define SERIAL_OPCODE_CMD_BLE_AD_DATA_SEND              (0x05)
#define SERIAL_OPCODE_CMD_UUID_GET                      (0x06)
#define SERIAL_OPCODE_CMD_HOUSEKEEPING_DATA_GET         (0x7e)
#define SERIAL_OPCODE_CMD_HOUSEKEEPING_DATA_CLEAR       (0x7f)

#define SERIAL_OPCODE_EVT_DEVICE_STARTED                (0x81)
#define SERIAL_OPCODE_EVT_CMD_RSP                       (0x82)
#define SERIAL_OPCODE_EVT_BLE_AD_DATA_RECEIVED          (0x83)

    union __attribute((packed))
    {
        nrf_serial_cmd_t cmd;
        nrf_serial_evt_t evt;
    } payload;
} nrf_serial_packet_t;


typedef void (*nrf_packet_rx_cb_t)(const nrf_serial_packet_t *packet, void *user_data);



int nrf_uart_init(const char *filename, int speed, int flags);

bool nrf_packet_send(int fd, const nrf_serial_packet_t *packet);
bool nrf_packet_receive(int fd, uint8_t *rx_buffer, size_t rx_buffer_size, int *rx_idx,
                        nrf_packet_rx_cb_t rx_cb, void *user_data);
