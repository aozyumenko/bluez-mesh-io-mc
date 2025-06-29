// SPDX-License-Identifier: LGPL-2.1-or-later
/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2025  Alexander Ozumenko <scg@stdio.ru>.
 *
 *
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>
#include <sys/time.h>
#include <time.h>
#include <fcntl.h>
#include <termios.h>
#include <ell/ell.h>

#include "src/shared/mgmt.h"
#include "src/shared/tty.h"
#include "util.h"

#include "mesh/list.h"
#include "mesh/mesh-defs.h"
#include "mesh/mesh-mgmt.h"
#include "mesh/mesh-io.h"
#include "mesh/mesh-io-api.h"
#include "mesh/mesh-io-mc.h"
#include "mesh/mesh-io-mc-api.h"



#define SLIP_END                0xc0
#define SLIP_ESC                0xdb
#define SLIP_ESC_END            0xdc
#define SLIP_ESC_ESC            0xdd

#define CMD_RSP_TIMEOUT         50
#define ERROR_CHECK_PERIOD      10000

#define SERIAL_FLOW_CTL         0x0001



enum stat_e {
    STAT_TX = 0,
    STAT_TX_RSP,
    STAT_TX_RSP_ERR,
    STAT_TX_NO_ACK,
    STAT_RX,
    STAT_RX_ERR,
    STAT_RX_DROP,
    STAT_HW_ALLOC_ERR,
    STAT_HW_RX_ERR,
    STAT_MAX
};


struct mesh_io_private {
    struct mesh_io *io;

    char *serial_path;
    int serial_flags;
    int serial_speed;
    int serial_fd;
    struct l_io  *serial_io;
    ssize_t data_credit_available;

    bool reset;
    bool started;

    uint8_t deviceaddr[DEVICEADDR_LEN];

    /* Rx packet */
    uint8_t rx_buffer[MC_MAX_ENCODED_PACKET_SIZE];
    int rx_idx;

    /* Tx packet */
    struct list_head tx_pkts;
    struct l_timeout *tx_timeout;
    struct list_head tx_cmd_rsps;
    struct l_timeout *tx_cmd_rsp_timeout;
    mc_packet_t tx_cur_packet;

    /* error checking */
    struct l_timeout *error_check_timeout;
    time_t stat_reset_time;
    unsigned long long int stat[STAT_MAX];
};


struct process_data {
    struct mesh_io_private *pvt;
    const uint8_t *data;
    uint8_t len;
    struct mesh_io_recv_info info;
};

struct tx_pkt {
    struct mesh_io *io;
    struct mesh_io_send_info info;
    bool delete;
    uint32_t token;
    uint8_t len;
    uint8_t pkt[MC_CMD_PAYLOAD_MAXLEN];
    struct list_head list;
};


typedef void (*cmd_rsp_cb_t)(uint32_t token, const mc_packet_t *packet, void *user_data);

struct cmd_rsp {
    bool used;
    uint8_t opcode;
    uint32_t token;
    unsigned long long instant;
    cmd_rsp_cb_t cb;
    void *user_data;
    struct list_head list;
};


typedef void (*mc_serial_packet_handler_t)(const mc_packet_t *packet, void *user_data);

typedef struct {
    uint8_t opcode;
    mc_serial_packet_handler_t handler;
} mc_packet_handler_entry_t;


typedef void (*serial_packet_rx_cb_t)(const mc_packet_t *packet, void *user_data);



/* forward declaritions */

static void cmd_send(struct mesh_io *io, uint8_t opcode, uint32_t token, size_t length,
                        uint8_t *payload, cmd_rsp_cb_t cb, void *user_data);

static void mc_reset(struct mesh_io *io);
static void mc_start(struct mesh_io *io);
static void mc_stop(struct mesh_io *io);

static void evt_device_started_cb(const mc_packet_t *packet, void *user_data);
static void cmd_resp_reset_cb(uint32_t token, const mc_packet_t *packet, void *user_data);
static void cmd_resp_version_get_cb(uint32_t token, const mc_packet_t *packet, void *user_data);
static void cmd_resp_deviceaddr_get_cb(uint32_t token, const mc_packet_t *packet, void *user_data);

static void ad_data_send(struct mesh_io *io, struct tx_pkt *tx);
static void evt_ble_ad_data_received(const mc_packet_t *packet, void *user_data);



/* global varoables */

static const char *stat_str[STAT_MAX] = {
    [STAT_TX] = "tx",
    [STAT_TX_RSP] = "tx_rsp",
    [STAT_TX_RSP_ERR] = "tx_rsp_err",
    [STAT_TX_NO_ACK] = "tx_no_ack",
    [STAT_RX] = "rx",
    [STAT_RX_ERR] = "rx_err",
    [STAT_RX_DROP] = "rx_drop",
    [STAT_HW_ALLOC_ERR] = "hw_alloc_err",
    [STAT_HW_RX_ERR] = "hw_rx_err"
};

static mc_packet_handler_entry_t packet_handler_list[] = {
    { MC_OPCODE_EVT_DEVICE_STARTED, evt_device_started_cb },
    { MC_OPCODE_EVT_BLE_AD_DATA_RECEIVED, evt_ble_ad_data_received },
};

// FIXME: for debug only - drop in future
static int tx_packets_alloc = 0;



/* statictics */

static void stat_reset(struct mesh_io *io)
{
    struct mesh_io_private *pvt = io->pvt;
    pvt->stat_reset_time = time(NULL);
    memset(&pvt->stat, 0, sizeof(pvt->stat));
}


static void stat_print(struct mesh_io *io)
{
    struct mesh_io_private *pvt = io->pvt;
    struct tm *tm;
    char time_str[32];

    tm = localtime(&pvt->stat_reset_time);
    strftime(time_str, sizeof(time_str), "%F %T", tm);

    l_debug("Mesh Controller statistics (%s)", time_str);
    for (int i = 0; i < STAT_MAX; i++) {
        l_debug("    %s: %llu", stat_str[i], pvt->stat[i]);
    }
    l_debug("    TX alloc: %d", tx_packets_alloc);
}


static inline void stat_report(struct mesh_io *io, enum stat_e id)
{
    struct mesh_io_private *pvt = io->pvt;

    if (id >= STAT_MAX)
        return;

    if (pvt->stat[id] < ULLONG_MAX) {
        pvt->stat[id]++;
    } else {
        stat_reset(io);
    }
}


static inline void stat_report_val(struct mesh_io *io, enum stat_e id,
                                   unsigned long long val)
{
    struct mesh_io_private *pvt = io->pvt;

    if (id >= STAT_MAX)
        return;

    if ((ULLONG_MAX - pvt->stat[id]) > val) {
        pvt->stat[id] += val;
    } else {
        stat_reset(io);
    }
}



/* common tools */

static uint32_t get_instant(void)
{
    struct timeval tm;
    uint32_t instant;

    gettimeofday(&tm, NULL);
    instant = tm.tv_sec * 1000;
    instant += tm.tv_usec / 1000;

    return instant;
}


static uint32_t instant_remaining_ms(uint32_t instant)
{
    instant -= get_instant();
    return instant;
}


static uint32_t get_next_token()
{
    static uint32_t m_token = 0;
    return m_token++;
}



/* Mesh Controller transport level */

static size_t slip_encode(const uint8_t *src, ssize_t src_len,
                          uint8_t *dest, size_t dest_len)
{
    int src_i, dest_i = 0;

    for (src_i = 0; src_i < src_len && dest_i < dest_len; src_i++, dest_i++) {
        if (SLIP_END == src[src_i]) {
            dest[dest_i++] = SLIP_ESC;
            if (dest_i >= dest_len) break;
            dest[dest_i] = SLIP_ESC_END;
        } else if (SLIP_ESC == src[src_i]) {
            dest[dest_i++] = SLIP_ESC;
            if (dest_i >= dest_len) break;
            dest[dest_i] = SLIP_ESC_ESC;
        } else {
            dest[dest_i] = src[src_i];
        }
    }

    return dest_i;
}


static size_t slip_decode(const uint8_t *src, ssize_t src_len,
                          uint8_t *dest, size_t dest_len)
{
    int src_i, dest_i = 0;
    uint8_t prev_char = 0;

    for (src_i = 0; src_i < src_len && dest_i < dest_len; src_i++) {
        if (SLIP_END == src[src_i]) {
            break;
        } else if (SLIP_ESC != prev_char) {
            if (SLIP_ESC != src[src_i]) {
                dest[dest_i++] = src[src_i];
            }
        } else {
            switch (src[src_i]) {
            case SLIP_ESC_END:
                dest[dest_i++] = SLIP_END;
                break;
            case SLIP_ESC_ESC:
                dest[dest_i++] = SLIP_ESC;
                break;
            default:
                return dest_i;
            }
        }
        prev_char = src[src_i];
    }

    return dest_i;
}


static int serial_set_speed(int fd, struct termios *ti, int speed)
{
    if (cfsetospeed(ti, tty_get_speed(speed)) < 0)
        return -errno;

    if (cfsetispeed(ti, tty_get_speed(speed)) < 0)
        return -errno;

    if (tcsetattr(fd, TCSANOW, ti) < 0)
        return -errno;

    return 0;
}


static int serial_init(const char *filename, int speed, int flags)
{
    struct termios ti;
    int fd;

    fd = open(filename, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        l_error("Failed to open serial port %s: %s",
                filename, strerror(errno));
        return -1;
    }

    tcflush(fd, TCIOFLUSH);

    if (tcgetattr(fd, &ti) < 0) {
        l_error("Can't get port settings: %s", strerror(errno));
        goto fail;
    }

    cfmakeraw(&ti);

    ti.c_cflag |= CLOCAL;
    if (flags & SERIAL_FLOW_CTL)
        ti.c_cflag |= CRTSCTS;
    else
        ti.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &ti) < 0) {
        l_error("Can't set port settings: %s", strerror(errno));
        goto fail;
    }

    if (serial_set_speed(fd, &ti, speed) < 0) {
        l_error("Can't set baud rate: %s", strerror(errno));
        goto fail;
    }

    return fd;

fail:
    close(fd);
    return -1;
}


static bool serial_packet_send(int fd, const mc_packet_t *packet)
{
    uint8_t encoded_packet[sizeof(*packet) * 2 + 2];    /* MAX = END + <every byte is ESC or END> + END */
    size_t encoded_packet_length = 0;
    ssize_t bytes;

    encoded_packet[0] = SLIP_END;
    encoded_packet_length++;

    encoded_packet_length += slip_encode((uint8_t *)packet, packet->length + 1,
                                         encoded_packet + encoded_packet_length,
                                         sizeof(encoded_packet) - 2);

    encoded_packet[encoded_packet_length] = SLIP_END;
    encoded_packet_length++;

    bytes = write(fd, encoded_packet, encoded_packet_length);
    if (bytes != encoded_packet_length) {
        l_error("Can't send packet to serial port: %s", strerror(errno));
        return false;
    }

    return true;
}


bool serial_packet_receive(int fd, uint8_t *rx_buffer, size_t rx_buffer_size, int *rx_idx,
                           serial_packet_rx_cb_t rx_cb, void *user_data)
{
    uint8_t buf[MC_MAX_ENCODED_PACKET_SIZE];
    size_t bytes;
    const mc_packet_t packet;
    size_t packet_length;
    int src_idx, dst_idx;

    bytes = read(fd, buf, sizeof(buf));
    if (bytes <= 0)
        return false;

    dst_idx = *rx_idx;
    for (src_idx = 0; src_idx < bytes; src_idx++) {
        if (buf[src_idx] == SLIP_END) {
            if (dst_idx > 0) {
                packet_length = slip_decode(rx_buffer, dst_idx,
                                            (uint8_t *)&packet, sizeof(packet));

                *rx_idx = dst_idx = 0;

                if (packet_length == packet.length + 1) {
                    rx_cb(&packet, user_data);
                } else {
                    l_debug("Invalid packet length %u, received %lu",
                             packet.length, packet_length);
                    return false;
                }
            }
        } else {
            if (dst_idx < rx_buffer_size) {
                rx_buffer[dst_idx++] = buf[src_idx];
            }
        }
    }

    *rx_idx = dst_idx;
    return true;
}



/* send command */

static inline struct cmd_rsp *cmd_rsp_find_by_instant(struct mesh_io *io, unsigned long long instant)
{
    struct mesh_io_private *pvt = io->pvt;
    struct cmd_rsp *rsp;

    list_for_each_entry(rsp, &pvt->tx_cmd_rsps, list) {
        if(rsp->instant <= instant)
            return rsp;
    }
    return NULL;
}


static void cmd_rsp_worker(struct l_timeout *timeout, void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;
    struct cmd_rsp *rsp;
    struct timeval tm;
    unsigned long long instant;

    if (pvt == NULL)
        return;

    gettimeofday(&tm, NULL);
    instant = tm.tv_sec * 1000;
    instant += tm.tv_usec / 1000;

    while ((rsp = cmd_rsp_find_by_instant(io, instant)) != NULL) {
        rsp->cb(rsp->token, NULL, rsp->user_data);
        stat_report(io, STAT_TX_NO_ACK);
        list_del(&rsp->list);
        l_free(rsp);
    }

    if (l_queue_length(io->rx_regs) == 0) {
        l_timeout_remove(timeout);
        pvt->tx_cmd_rsp_timeout = NULL;
    } else {
        l_timeout_modify_ms(timeout, CMD_RSP_TIMEOUT);
    }
}


static void cmd_rsp_add(struct mesh_io *io, uint8_t opcode, uint32_t token, cmd_rsp_cb_t cb, void *user_data)
{
    struct mesh_io_private *pvt = io->pvt;
    struct cmd_rsp *rsp;
    struct timeval tm;
    unsigned long long instant;

    if (pvt == NULL || cb == NULL)
        return;

    gettimeofday(&tm, NULL);
    instant = tm.tv_sec * 1000;
    instant += tm.tv_usec / 1000;

    rsp = l_new(struct cmd_rsp, 1);

    rsp->opcode = opcode;
    rsp->token = token;
    rsp->instant = instant + CMD_RSP_TIMEOUT;
    rsp->user_data = user_data;
    rsp->cb = cb;

    list_add_tail(&rsp->list, &pvt->tx_cmd_rsps);

    if (pvt->tx_cmd_rsp_timeout == NULL) {
        pvt->tx_cmd_rsp_timeout = l_timeout_create_ms(CMD_RSP_TIMEOUT,
                                                      cmd_rsp_worker, io, NULL);
    }
}


static bool cmd_rsp_handle(struct mesh_io *io, const mc_packet_t *packet)
{
    struct mesh_io_private *pvt = io->pvt;
    struct cmd_rsp *rsp;

    if (pvt == NULL)
        return false;

    list_for_each_entry(rsp, &pvt->tx_cmd_rsps, list) {
        if (rsp->opcode == packet->payload.evt.cmd_rsp.opcode && rsp->token == packet->payload.evt.cmd_rsp.token) {
            rsp->cb(rsp->token, packet, rsp->user_data);
            stat_report(io, STAT_TX_RSP);
            list_del(&rsp->list);
            l_free(rsp);
            return true;
        }
    }

    stat_report(io, STAT_RX_DROP);
    return false;
}


static void cmd_send(struct mesh_io *io, uint8_t opcode, uint32_t token, size_t length,
                     uint8_t *payload, cmd_rsp_cb_t cb, void *user_data)
{
    struct mesh_io_private *pvt = io->pvt;
    mc_packet_t packet;

    if (pvt == NULL)
        return;

    packet.length = MC_PACKET_OVERHEAD + MC_CMD_OVERHEAD + length;
    packet.opcode = opcode;
    packet.payload.cmd.token = token;
    if (length > 0 && payload != NULL) {
        memcpy(&packet.payload.cmd.payload, payload, length);
    }

    (void)serial_packet_send(pvt->serial_fd, &packet);
    cmd_rsp_add(io, opcode, token, cb, user_data);

    stat_report(io, STAT_TX);
}



/* mesh controller initialization sequence */

static void cmd_resp_reset_cb(uint32_t token, const mc_packet_t *packet, void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;

    if (pvt == NULL)
        return;

    if (packet != NULL || (packet == NULL && pvt->reset)) {
        l_error("Can't reset Mesh Controller");
        mc_reset(io);
    }
}


static void evt_device_started_cb(const mc_packet_t *packet, void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;

    if (pvt == NULL)
        return;

    l_debug("Mesh Controller started, operating_mode: 0x%x, hw_error: 0x%x, data_credit_available: %d",
        packet->payload.evt.started.operating_mode,
        packet->payload.evt.started.hw_error,
        packet->payload.evt.started.data_credit_available);

    if (packet->payload.evt.started.operating_mode == MC_DEVICE_OPERATING_MODE_APPLICATION &&
            packet->payload.evt.started.hw_error == 0) {
        pvt->data_credit_available = packet->payload.evt.started.data_credit_available;
        stat_reset(io);
        pvt->reset = false;

        /* get protocol version */
        cmd_send(io, MC_OPCODE_CMD_VERSION_GET, get_next_token(),
                 0, NULL, cmd_resp_version_get_cb, io);
    }
}


static void cmd_resp_version_get_cb(uint32_t token, const mc_packet_t *packet, void *user_data)
{
    struct mesh_io *io = user_data;

    if (packet == NULL) {
        mc_reset(io);
        return;
    }

    if (packet->payload.evt.cmd_rsp.status != MC_STATUS_SUCCESS) {
        l_error("Can't get protocol version, status: 0x%x",
                packet->payload.evt.cmd_rsp.status);
        if (io->ready)
            io->ready(io->user_data, false);
        return;
    }

    uint16_t api_ver = packet->payload.evt.cmd_rsp.data.version.ver;
    l_debug("Protocol version: %u.%u", (api_ver >> 8), (api_ver & 0xff));

    if (packet->payload.evt.cmd_rsp.data.version.ver == MC_API_VERSION) {
        cmd_send(io, MC_OPCODE_CMD_DEVICEADDR_GET, get_next_token(),
                 0, NULL, cmd_resp_deviceaddr_get_cb, io);
    } else {
        l_error("Invalid protocol version: %u.%u, expected: %u.%u",
                (api_ver >> 8), (api_ver & 0xff),
                (MC_API_VERSION >> 8), (MC_API_VERSION & 0xff));
        if (io->ready)
            io->ready(io->user_data, false);
    }
}


static void cmd_resp_deviceaddr_get_cb(uint32_t token, const mc_packet_t *packet, void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;

    if (pvt == NULL)
        return;

    if (packet == NULL) {
        mc_reset(io);
        return;
    }

    if (packet->payload.evt.cmd_rsp.status != MC_STATUS_SUCCESS) {
        l_error("Can't get device address, status: 0x%x",
                packet->payload.evt.cmd_rsp.status);
        if (io->ready)
            io->ready(io->user_data, false);
        return;
    }

    memcpy(pvt->deviceaddr, packet->payload.evt.cmd_rsp.data.deviceaddr.addr, sizeof(pvt->deviceaddr));
    l_debug("Device address: %02x:%02x:%02x:%02x:%02x:%02x",
            pvt->deviceaddr[0], pvt->deviceaddr[1], pvt->deviceaddr[2],
            pvt->deviceaddr[3], pvt->deviceaddr[4], pvt->deviceaddr[5]);

    l_info("Started Mesh Controller");

    if (io->ready)
        io->ready(io->user_data, true);
}


static void cmd_resp_start_cb(uint32_t token, const mc_packet_t *packet, void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;

    if(pvt == NULL)
        return;

    if (packet == NULL) {
        cmd_send(io, MC_OPCODE_CMD_START, get_next_token(),
                     0, NULL, cmd_resp_start_cb, io);
        return;
    }

    if (packet->payload.evt.cmd_rsp.status != MC_STATUS_SUCCESS) {
        l_error("Can't start Mesh Controller, status: 0x%x",
                packet->payload.evt.cmd_rsp.status);
        pvt->started = false;
    }

    pvt->started = true;
}



/* mesh controller control */

static void mc_reset(struct mesh_io *io)
{
    struct mesh_io_private *pvt = io->pvt;

    if (pvt == NULL)
        return;

    l_debug("send Reset Mesh Controller command");
    pvt->reset = true;
    cmd_send(io, MC_OPCODE_CMD_RESET, get_next_token(),
             0, NULL, cmd_resp_reset_cb, io);
}


static void mc_start(struct mesh_io *io)
{
    l_debug("send Start Mesh Controller command");
    cmd_send(io, MC_OPCODE_CMD_START, get_next_token(),
             0, NULL, cmd_resp_start_cb, io);
}


static void mc_stop(struct mesh_io *io)
{
    struct mesh_io_private *pvt = io->pvt;

    if (pvt == NULL)
        return;

    l_debug("send Stop Mesh Controller command");
    cmd_send(io, MC_OPCODE_CMD_STOP, get_next_token(),
             0, NULL, NULL, io);
    pvt->started = false;
}



/* receive packet from mesh controller */

static void mc_packet_rx(const mc_packet_t *packet, void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;
    cmd_rsp_cb_t cmd_rsp_cb;
    int i;

    if (pvt == NULL)
        return;

    if (packet->opcode == MC_OPCODE_EVT_CMD_RSP) {
        (void)cmd_rsp_handle(io, packet);
        stat_report(io, STAT_RX);
    } else {
        for (i = 0; i < L_ARRAY_SIZE(packet_handler_list); i++) {
            if (packet_handler_list[i].opcode == packet->opcode) {
                packet_handler_list[i].handler(packet, io);
                stat_report(io, STAT_RX);
                return;
            }
        }

        stat_report(io, STAT_RX_ERR);
        l_debug("Invalid packet opcode: 0x%02x", packet->opcode);
    }
}


static bool rx_worker(struct l_io *l_io, void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;
    bool result;

    if (pvt == NULL)
        return false;

    result = serial_packet_receive(pvt->serial_fd, pvt->rx_buffer,
                                   sizeof(pvt->rx_buffer), &pvt->rx_idx,
                                   mc_packet_rx, io);
    if (!result)
        stat_report(io, STAT_RX_ERR);

    return true;
}



/* processing received data packets */

static void process_rx_callbacks(void *v_reg, void *v_rx)
{
    struct mesh_io_reg *rx_reg = v_reg;
    struct process_data *rx = v_rx;

    if (!memcmp(rx->data, rx_reg->filter, rx_reg->len))
        rx_reg->cb(rx_reg->user_data, &rx->info, rx->data, rx->len);
}


static void process_rx(struct mesh_io_private *pvt, int8_t rssi,
                       uint32_t instant, const uint8_t *addr,
                       const uint8_t *data, uint8_t len)
{
    struct process_data rx = {
        .pvt = pvt,
        .data = data,
        .len = len,
        .info.instant = instant,
        .info.addr = addr,
        .info.chan = 7,
        .info.rssi = rssi,
    };

    l_queue_foreach(pvt->io->rx_regs, process_rx_callbacks, &rx);
}


static void evt_ble_ad_data_received(const mc_packet_t *packet, void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;
    uint32_t instant;
    const uint8_t *adv;
    uint8_t adv_len;
    const uint8_t *addr;
    int8_t rssi;

    if (pvt == NULL)
        return;

    instant = get_instant();
    adv = packet->payload.evt.ble_ad_data.data + 1;
    adv_len = packet->payload.evt.ble_ad_data.data[0];
    addr = pvt->deviceaddr;
    rssi = 0;

    process_rx(pvt, rssi, instant, addr, adv, adv_len);
}



/* send data packets */

static void ad_data_send_cb(uint32_t token, const mc_packet_t *packet, void *user_data)
{
    struct tx_pkt *tx = user_data;
    struct mesh_io *io = tx->io;
    struct mesh_io_private *pvt = io->pvt;

    if (pvt == NULL)
        return;

    if (packet == NULL) {
        l_debug("Sent packet not ACKed, token 0x%x", token);
        stat_report(io, STAT_TX_NO_ACK);
        ad_data_send(io, tx);
        return;
    }

    if (packet->payload.evt.cmd_rsp.status != MC_STATUS_SUCCESS) {
        stat_report(io, STAT_TX_RSP_ERR);

        l_debug("Sent packet error: status 0x%x, token 0x%x",
            packet->payload.evt.cmd_rsp.status, token);
        mc_reset(io);
    }

    if (tx->delete) {
        tx_packets_alloc--;
        l_free(tx);
    }
}


static void ad_data_send(struct mesh_io *io, struct tx_pkt *tx)
{
    struct mesh_io_private *pvt = io->pvt;
    mc_packet_t packet;
    bool ret;

    if (pvt == NULL)
        return;

    pvt->tx_cur_packet.length = MC_PACKET_OVERHEAD + MC_CMD_OVERHEAD + tx->len + 1;
    pvt->tx_cur_packet.opcode = MC_OPCODE_CMD_BLE_AD_DATA_SEND;
    pvt->tx_cur_packet.payload.cmd.token = tx->token;
    pvt->tx_cur_packet.payload.cmd.payload.ble_ad_data.data[0] = tx->len;
    memcpy(&pvt->tx_cur_packet.payload.cmd.payload.ble_ad_data.data[1], tx->pkt, tx->len);

    (void)serial_packet_send(pvt->serial_fd, &pvt->tx_cur_packet);
    cmd_rsp_add(io, MC_OPCODE_CMD_BLE_AD_DATA_SEND, tx->token,
                ad_data_send_cb, tx);

    stat_report(io, STAT_TX);
}


static void tx_to(struct l_timeout *timeout, void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;
    struct tx_pkt *tx;
    uint16_t ms;
    uint8_t count;

    if (pvt == NULL)
        return;

    if (list_empty(&pvt->tx_pkts)) {
        l_timeout_remove(timeout);
        pvt->tx_timeout = NULL;
        return;
    }

    tx = list_first_entry(&pvt->tx_pkts, struct tx_pkt, list);
    list_del(&tx->list);

    if (tx->info.type == MESH_IO_TIMING_TYPE_GENERAL) {
        ms = tx->info.u.gen.interval;
        count = tx->info.u.gen.cnt;
        if (count != MESH_IO_TX_COUNT_UNLIMITED)
            tx->info.u.gen.cnt--;
    } else {
        ms = 25;
        count = 1;
    }

    tx->delete = !!(count == 1);

    ad_data_send(io, tx);

    if (count == 1) {
        if (!list_empty(&pvt->tx_pkts)) {
            /* Recalculate wakeup if we are responding to POLL */
            tx = list_first_entry(&pvt->tx_pkts, struct tx_pkt, list);
            if (tx->info.type == MESH_IO_TIMING_TYPE_POLL_RSP) {
                ms = instant_remaining_ms(tx->info.u.poll_rsp.instant +
                    tx->info.u.poll_rsp.delay);
            }
        }
    }
    else
        list_add_tail(&tx->list, &pvt->tx_pkts);

    if (timeout != NULL) {
        pvt->tx_timeout = timeout;
        l_timeout_modify_ms(timeout, ms);
    } else
        pvt->tx_timeout = l_timeout_create_ms(ms, tx_to, io, NULL);
}


static void tx_worker(void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;
    struct tx_pkt *tx;
    uint32_t delay;

    if (pvt == NULL)
        return;

    if (list_empty(&pvt->tx_pkts))
        return;

    tx = list_first_entry(&pvt->tx_pkts, struct tx_pkt, list);

    switch (tx->info.type) {
    case MESH_IO_TIMING_TYPE_GENERAL:
        if (tx->info.u.gen.min_delay == tx->info.u.gen.max_delay)
            delay = tx->info.u.gen.min_delay;
        else {
            l_getrandom(&delay, sizeof(delay));
            delay %= tx->info.u.gen.max_delay -
                        tx->info.u.gen.min_delay;
            delay += tx->info.u.gen.min_delay;
        }
        break;

    case MESH_IO_TIMING_TYPE_POLL:
        if (tx->info.u.poll.min_delay == tx->info.u.poll.max_delay)
            delay = tx->info.u.poll.min_delay;
        else {
            l_getrandom(&delay, sizeof(delay));
            delay %= tx->info.u.poll.max_delay -
                        tx->info.u.poll.min_delay;
            delay += tx->info.u.poll.min_delay;
        }
        break;

    case MESH_IO_TIMING_TYPE_POLL_RSP:
        /* Delay until Instant + Delay */
        delay = instant_remaining_ms(tx->info.u.poll_rsp.instant +
                        tx->info.u.poll_rsp.delay);
        if (delay > 255)
            delay = 0;
        break;

    default:
        return;
    }

    if (delay == 0)
        tx_to(pvt->tx_timeout, io);
    else if (pvt->tx_timeout != NULL)
        l_timeout_modify_ms(pvt->tx_timeout, delay);
    else
        pvt->tx_timeout = l_timeout_create_ms(delay, tx_to, io, NULL);
}



/* statistics and error checking  */

static void cmd_resp_housekeeping_data_get_cb(uint32_t token, const mc_packet_t *packet, void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;

    if (pvt == NULL)
        return;

    if (packet != NULL && packet->payload.evt.cmd_rsp.status == MC_STATUS_SUCCESS) {
        stat_report_val(io, STAT_HW_ALLOC_ERR,
                        packet->payload.evt.cmd_rsp.data.hk_data.alloc_fail_count);
        stat_report_val(io, STAT_HW_RX_ERR,
                        packet->payload.evt.cmd_rsp.data.hk_data.rx_fail_count);

        cmd_send(io, MC_OPCODE_CMD_HOUSEKEEPING_DATA_CLEAR, get_next_token(),
                 0, NULL, NULL, io);
    }

    stat_print(io);
}


static void error_check_worker(struct l_timeout *timeout, void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;

    if (pvt == NULL)
        return;

    cmd_send(io, MC_OPCODE_CMD_HOUSEKEEPING_DATA_GET, get_next_token(),
             0, NULL, cmd_resp_housekeeping_data_get_cb, io);

    l_timeout_modify_ms(timeout, ERROR_CHECK_PERIOD);
}


static void serial_disconnect_cb(struct l_io *serial_io, void *user_data)
{
    l_main_quit();
}



/* initialize mesh controller */
static void mc_initalize(void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;
    bool result = true;
    int fd;

    if (pvt == NULL)
        return;

    fd = serial_init(pvt->serial_path, pvt->serial_speed, pvt->serial_flags);
    if (fd < 0)
        result = false;

    if (result) {
        pvt->serial_fd = fd;

        pvt->serial_io = l_io_new(pvt->serial_fd);
        (void)l_io_set_read_handler(pvt->serial_io, rx_worker, io, NULL);
        (void)l_io_set_disconnect_handler(pvt->serial_io, serial_disconnect_cb, io, NULL);

        pvt->error_check_timeout = l_timeout_create_ms(ERROR_CHECK_PERIOD,
                                   error_check_worker, io, NULL);

        /* reset device */
        mc_reset(io);
    }

    if (io->ready)
        io->ready(io->user_data, result);
}



/* interface functions */

static bool dev_init(struct mesh_io *io, void *opts, void *user_data)
{
    struct mesh_io_private *pvt;
    char *serial_path;

    if (io == NULL || io->pvt != NULL)
        return false;

    serial_path = (char *)opts;

    pvt = l_new(struct mesh_io_private, 1);

    pvt->io = io;

    pvt->serial_path = l_strdup(serial_path);
    pvt->serial_flags = SERIAL_FLOW_CTL;
    pvt->serial_speed = MC_SERIAL_BAUD_RATE;
    pvt->serial_fd = -1;

    pvt->reset = false;
    pvt->started = false;

    pvt->serial_io = NULL;
    pvt->rx_idx = 0;

    INIT_LIST_HEAD(&pvt->tx_pkts);
    pvt->tx_timeout = NULL;
    INIT_LIST_HEAD(&pvt->tx_cmd_rsps);
    pvt->tx_cmd_rsp_timeout = NULL;

    io->pvt = pvt;

    stat_reset(io);

    l_idle_oneshot(mc_initalize, io, NULL);

    return true;
}


static bool dev_destroy(struct mesh_io *io)
{
    struct mesh_io_private *pvt = io->pvt;
    struct cmd_rsp *rsp, *rsp_tmp;
    struct tx_pkt *tx, *tx_tmp;

    if (pvt == NULL)
        return true;

    pvt->started = false;
    pvt->reset = false;

    l_io_destroy(pvt->serial_io);
    pvt->serial_io = NULL;

    close(pvt->serial_fd);
    pvt->serial_fd = -1;

    l_timeout_remove(pvt->tx_timeout);
    pvt->tx_timeout = NULL;
    list_for_each_entry_safe(tx, tx_tmp, &pvt->tx_pkts, list) {
        list_del(&tx->list);
        l_free(tx);
    }
    l_timeout_remove(pvt->tx_cmd_rsp_timeout);
    list_for_each_entry_safe(rsp, rsp_tmp, &pvt->tx_cmd_rsps, list) {
        list_del(&rsp->list);
        l_free(rsp);
    }

    l_timeout_remove(pvt->error_check_timeout);
    pvt->error_check_timeout = NULL;

    l_free(pvt->serial_path);
    io->pvt = NULL;
    l_free(pvt);

    return true;
}


static bool dev_caps(struct mesh_io *io, struct mesh_io_caps *caps)
{
    struct mesh_io_private *pvt = io->pvt;

    if (pvt == NULL || caps == NULL)
        return false;

    caps->max_num_filters = 255;
    caps->window_accuracy = 50;

    return true;
}


static bool send_tx(struct mesh_io *io, struct mesh_io_send_info *info,
                    const uint8_t *data, uint16_t len)
{
    struct mesh_io_private *pvt = io->pvt;
    struct tx_pkt *tx;
    bool sending = false;


    if (pvt == NULL || info == NULL || data == NULL || len == 0 || len > sizeof(tx->pkt))
        return false;

    if (!pvt->started)
        return false;

    tx = l_new(struct tx_pkt, 1);
    tx->io = io;
    memcpy(&tx->info, info, sizeof(tx->info));
    memcpy(tx->pkt, data, len);
    tx->len = len;
    tx->token = get_next_token();

    tx_packets_alloc++;

    if (info->type == MESH_IO_TIMING_TYPE_POLL_RSP)
        list_add(&tx->list, &pvt->tx_pkts);
    else
        list_add_tail(&tx->list, &pvt->tx_pkts);

    l_timeout_remove(pvt->tx_timeout);
    pvt->tx_timeout = NULL;
    l_idle_oneshot(tx_worker, io, NULL);

    return true;
}


static bool tx_cancel(struct mesh_io *io, const uint8_t *data, uint8_t len)
{
    struct mesh_io_private *pvt = io->pvt;
    struct tx_pkt *tx, *tx_tmp;

    if (pvt == NULL || data == NULL)
        return false;

    if (!pvt->started)
        return false;

    if (len == 1) {
        list_for_each_entry_safe(tx, tx_tmp, &pvt->tx_pkts, list) {
            if (!data[0] || data[0] == tx->pkt[0]) {
                list_del(&tx->list);
                l_free(tx);
                tx_packets_alloc--;
            }
        }
    } else {
        list_for_each_entry_safe(tx, tx_tmp, &pvt->tx_pkts, list) {
            if (tx->len >= len && !memcmp(tx->pkt, data, len)) {
                list_del(&tx->list);
                l_free(tx);
                tx_packets_alloc--;
            }
        }
    }

    if (list_empty(&pvt->tx_pkts)) {
        l_timeout_remove(pvt->tx_timeout);
        pvt->tx_timeout = NULL;
    }

    return true;
}


static bool recv_register(struct mesh_io *io, const uint8_t *filter,
                          uint8_t len, mesh_io_recv_func_t cb, void *user_data)
{
    struct mesh_io_private *pvt = io->pvt;
    bool already_started;

    if (pvt == NULL)
        return false;

    already_started = l_queue_length(io->rx_regs) > 1;

    if (!pvt->started && !already_started)
        mc_start(io);

    return true;
}


static bool recv_deregister(struct mesh_io *io, const uint8_t *filter,
                            uint8_t len)
{
    struct mesh_io_private *pvt = io->pvt;
    bool already_started;

    if (pvt == NULL)
        return false;

    already_started = l_queue_length(io->rx_regs) > 1;

    if (!already_started)
        mc_stop(io);

    return true;
}



const struct mesh_io_api mesh_io_mc = {
    .init = dev_init,
    .destroy = dev_destroy,
    .caps = dev_caps,
    .send = send_tx,
    .reg = recv_register,
    .dereg = recv_deregister,
    .cancel = tx_cancel,
};
