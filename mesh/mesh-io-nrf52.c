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
#include <ell/ell.h>

#include "src/shared/mgmt.h"
#include "util.h"

#include "mesh/mesh-defs.h"
#include "mesh/mesh-mgmt.h"
#include "mesh/mesh-io.h"
#include "mesh/mesh-io-api.h"
#include "mesh/mesh-io-nrf52.h"
#include "mesh/nrf52-serial.h"



#define CMD_RSP_TIMEOUT         50
#define ERROR_CHECK_PERIOD      10000



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
    uint8_t rx_buffer[NRF_SERIAL_MAX_ENCODED_PACKET_SIZE];
    int rx_idx;

    /* Tx packet */
    struct l_queue *tx_pkts;
    struct l_timeout *tx_timeout;
    struct l_queue *tx_cmd_rsps;
    struct l_timeout *tx_cmd_rsp_timeout;

    /* error checking */
    struct l_timeout *error_check_timeout;
    time_t stat_reset_time;
    unsigned long long int stat[STAT_MAX];
};


typedef void (*cmd_rsp_cb_t)(uint32_t token, const nrf_serial_packet_t *packet, void *user_data);

struct cmd_rsp {
    bool used;
    uint8_t opcode;
    uint32_t token;
    unsigned long long instant;
    cmd_rsp_cb_t cb;
    void *user_data;
};


struct process_data {
    struct mesh_io_private *pvt;
    const uint8_t *data;
    uint8_t len;
    struct mesh_io_recv_info info;
};


struct tx_pkt {
    struct mesh_io_send_info info;
    bool delete;
    uint32_t token;
    uint8_t len;
    uint8_t pkt[NRF_MESH_SERIAL_CMD_PAYLOAD_MAXLEN];
};


struct tx_pattern {
    const uint8_t *data;
    uint8_t len;
};




static void nrf_start(void *user_data);
static void nrf_stop(struct mesh_io *io);
static void nrf_cmd_send(struct mesh_io *io, uint8_t opcode, uint32_t token, size_t length,
                         uint8_t *payload, cmd_rsp_cb_t cb, void *user_data);

static bool nrf_cmd_send_reset(struct mesh_io *io);
static bool nrf_cmd_send_serial_version_get(struct mesh_io *io);
static bool nrf_cmd_send_start(struct mesh_io *io);
static void nrf_cmd_resp_deviceaddr_get_cb(uint32_t token, const nrf_serial_packet_t *packet, void *user_data);
static void ad_data_send_cb(uint32_t token, const nrf_serial_packet_t *packet, void *user_data);



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


static bool simple_match(const void *a, const void *b)
{
    return a == b;
}

static bool find_by_ad_type(const void *a, const void *b)
{
    const struct tx_pkt *tx = a;
    uint8_t ad_type = L_PTR_TO_UINT(b);

    return !ad_type || ad_type == tx->pkt[0];
}

static bool find_by_pattern(const void *a, const void *b)
{
    const struct tx_pkt *tx = a;
    const struct tx_pattern *pattern = b;

    if (tx->len < pattern->len)
        return false;

    return (!memcmp(tx->pkt, pattern->data, pattern->len));
}

static bool find_by_token(const void *a, const void *b)
{
    const struct tx_pkt *tx = a;
    uint32_t token = L_PTR_TO_UINT(b);

    return token == tx->token;
}


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

static inline void stat_report_val(struct mesh_io *io, enum stat_e id, unsigned long long val)
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



/* send command */

static bool cmd_rsp_find_by_instant(const void *a, const void *b)
{
    const struct cmd_rsp *rsp = a;
    const unsigned long long *instant = b;

    return rsp->instant <= *instant;
}


static bool cmd_rsp_find_by_packet(const void *a, const void *b)
{
    const struct cmd_rsp *rsp = a;
    const nrf_serial_packet_t *packet = b;

    return packet->opcode == SERIAL_OPCODE_EVT_CMD_RSP &&
        rsp->opcode == packet->payload.evt.cmd_rsp.opcode &&
        rsp->token == packet->payload.evt.cmd_rsp.token;
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

    while ((rsp = l_queue_find(pvt->tx_cmd_rsps, cmd_rsp_find_by_instant,
                               &instant)) != NULL) {
        rsp->cb(rsp->token, NULL, rsp->user_data);
        stat_report(io, STAT_TX_NO_ACK);

        l_queue_remove_if(pvt->tx_cmd_rsps, simple_match, rsp);
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

    l_queue_push_tail(pvt->tx_cmd_rsps, rsp);

    if (pvt->tx_cmd_rsp_timeout == NULL) {
        pvt->tx_cmd_rsp_timeout = l_timeout_create_ms(CMD_RSP_TIMEOUT,
                                                      cmd_rsp_worker, io, NULL);
    }
}


static bool cmd_rsp_handle(struct mesh_io *io, const nrf_serial_packet_t *packet)
{
    struct mesh_io_private *pvt = io->pvt;
    struct cmd_rsp *rsp;

    if (pvt == NULL)
        return false;

    rsp = l_queue_find(pvt->tx_cmd_rsps, cmd_rsp_find_by_packet, packet);
    if (rsp != NULL) {
        rsp->cb(rsp->token, packet, rsp->user_data);
        stat_report(io, STAT_TX_RSP);

        l_queue_remove_if(pvt->tx_cmd_rsps, simple_match, rsp);
        l_free(rsp);
        return true;
    }

    stat_report(io, STAT_RX_DROP);
    return false;
}


static void nrf_cmd_send(struct mesh_io *io, uint8_t opcode, uint32_t token, size_t length,
                         uint8_t *payload, cmd_rsp_cb_t cb, void *user_data)
{
    struct mesh_io_private *pvt = io->pvt;
    nrf_serial_packet_t packet;
    bool ret;

    if (pvt == NULL)
        return;

    packet.length = NRF_MESH_SERIAL_PACKET_OVERHEAD + NRF_MESH_SERIAL_CMD_OVERHEAD + length;
    packet.opcode = opcode;
    packet.payload.cmd.token = token;
    if (length > 0 && payload != NULL) {
        memcpy(&packet.payload.cmd.payload, payload, length);
    }

    ret = nrf_packet_send(pvt->serial_fd, &packet);
    if (ret) {
        cmd_rsp_add(io, opcode, token, cb, user_data);
    }

    stat_report(io, STAT_TX);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void nrf_cmd_resp_reset_cb(uint32_t token, const nrf_serial_packet_t *packet, void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;

    if (pvt == NULL)
        return;

    if (packet != NULL || (packet == NULL && pvt->reset)) {
        l_error("Can't reset radio");
        (void)nrf_cmd_send_reset(io);
    }
}

static bool nrf_cmd_send_reset(struct mesh_io *io)
{
    struct mesh_io_private *pvt = io->pvt;

    if (pvt == NULL)
        return false;

    l_debug("Send CMD_RESET");
    pvt->reset = true;
    nrf_cmd_send(io, SERIAL_OPCODE_CMD_RESET, get_next_token(),
                 0, NULL, nrf_cmd_resp_reset_cb, io);
    return true;
}



static void nrf_cmd_resp_serial_version_get_cb(uint32_t token, const nrf_serial_packet_t *packet, void *user_data)
{
    struct mesh_io *io = user_data;


    if (packet != NULL && packet->payload.evt.cmd_rsp.status == SERIAL_STATUS_SUCCESS) {
        uint16_t api_ver = packet->payload.evt.cmd_rsp.data.serial_version.serial_ver;
        l_debug("Serial protocol version: %u.%u", (api_ver >> 8), (api_ver & 0xff));

        if (packet->payload.evt.cmd_rsp.data.serial_version.serial_ver == SERIAL_API_VERSION) {
            nrf_cmd_send(io, SERIAL_OPCODE_CMD_DEVICEADDR_GET, get_next_token(),
                         0, NULL, nrf_cmd_resp_deviceaddr_get_cb, io);
        } else {
            l_error("Invalid serial protocol version: %u.%u, expected: %u.%u",
                (api_ver >> 8), (api_ver & 0xff),
                (SERIAL_API_VERSION >> 8), (SERIAL_API_VERSION & 0xff));

            if (io->ready)
                io->ready(io->user_data, false);
        }
    }
    else {
        (void) nrf_cmd_send_reset(io);
    }

}

static bool nrf_cmd_send_serial_version_get(struct mesh_io *io)
{
    l_debug("Send CMD DEVICE_SERIAL_VERSION");
    nrf_cmd_send(io, SERIAL_OPCODE_CMD_SERIAL_VERSION_GET, get_next_token(),
                 0, NULL, nrf_cmd_resp_serial_version_get_cb, io);
    return true;
}


static void nrf_cmd_resp_deviceaddr_get_cb(uint32_t token, const nrf_serial_packet_t *packet, void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;

    if (pvt == NULL)
        return;

    if (packet != NULL && packet->payload.evt.cmd_rsp.status == SERIAL_STATUS_SUCCESS) {
        memcpy(pvt->deviceaddr, packet->payload.evt.cmd_rsp.data.deviceaddr.addr, sizeof(pvt->deviceaddr));
        l_debug("Device address: %02x:%02x:%02x:%02x:%02x:%02x",
            pvt->deviceaddr[0], pvt->deviceaddr[1], pvt->deviceaddr[2],
            pvt->deviceaddr[3], pvt->deviceaddr[4], pvt->deviceaddr[5]);

        if (io->ready)
            io->ready(io->user_data, true);
    }
    else {
        (void) nrf_cmd_send_reset(io);
    }
}


static void nrf_cmd_resp_start_cb(uint32_t token, const nrf_serial_packet_t *packet, void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;

    if(pvt == NULL)
        return;

    if (packet != NULL && packet->payload.evt.cmd_rsp.status == SERIAL_STATUS_SUCCESS) {
        l_debug("Started radio");
        pvt->started = true;
    }
    else {
        l_error("Can't start radio");
        (void) nrf_cmd_send_reset(io);
//        pvt->started = false;
    }
}

static bool nrf_cmd_send_start(struct mesh_io *io)
{
    l_debug("Send CMD_START");
    nrf_cmd_send(io, SERIAL_OPCODE_CMD_START, get_next_token(),
                 0, NULL, nrf_cmd_resp_start_cb, io);
    return true;
}


static void nrf_cmd_resp_stop_cb(uint32_t token, const nrf_serial_packet_t *packet, void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;

    if (pvt == NULL)
        return;

    if (packet != NULL && packet->payload.evt.cmd_rsp.status == SERIAL_STATUS_SUCCESS) {
        l_debug("Stopped radio");
        pvt->started = false;
    }
    else {
        l_error("Can't stop radio");
        (void) nrf_cmd_send_reset(io);
    }
}

static bool nrf_cmd_send_stop(struct mesh_io *io)
{
    l_debug("Send CMD_STOP");
    nrf_cmd_send(io, SERIAL_OPCODE_CMD_STOP, get_next_token(),
                 0, NULL, nrf_cmd_resp_stop_cb, io);
    return true;
}


/////////////////////////////////////////////////


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



/////////////////////////////////////////////////


/* receive packet */

typedef void (*nrf_serial_packet_handler_t)(const nrf_serial_packet_t *packet, void *user_data);

typedef struct {
    uint8_t opcode;
    nrf_serial_packet_handler_t handler;
} nrf_serial_packet_handler_entry_t;

static void nrf_serial_handler_evt_device_started(const nrf_serial_packet_t *packet, void *user_data);
static void nrf_serial_handler_evt_ble_ad_data_received(const nrf_serial_packet_t *packet, void *user_data);


static nrf_serial_packet_handler_entry_t packet_handler_list[] = {
    { SERIAL_OPCODE_EVT_DEVICE_STARTED, nrf_serial_handler_evt_device_started },
    { SERIAL_OPCODE_EVT_BLE_AD_DATA_RECEIVED, nrf_serial_handler_evt_ble_ad_data_received },
};

static void nrf_serial_handler_evt_device_started(const nrf_serial_packet_t *packet, void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;

    if (pvt == NULL)
        return;

    l_debug("EVT: DEVICE_STARTED, operating_mode=0x%x, hw_error=0x%x, data_credit_available=%d",
        packet->payload.evt.started.operating_mode,
        packet->payload.evt.started.hw_error,
        packet->payload.evt.started.data_credit_available);

    if (packet->payload.evt.started.operating_mode == SERIAL_DEVICE_OPERATING_MODE_APPLICATION && packet->payload.evt.started.hw_error == 0) {
        pvt->data_credit_available = packet->payload.evt.started.data_credit_available;
        stat_reset(io);
        pvt->reset = false;

        /* get protocol version */
        (void)nrf_cmd_send_serial_version_get(io);
        // TODO check result and call io->ready()
    }
}


static void nrf_serial_handler_evt_ble_ad_data_received(const nrf_serial_packet_t *packet, void *user_data)
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

static void nrf_serial_packet_rx(const nrf_serial_packet_t *packet, void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;
    cmd_rsp_cb_t cmd_rsp_cb;
    int i;

    if (pvt == NULL)
        return;

    if (packet->opcode == SERIAL_OPCODE_EVT_CMD_RSP) {
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
        print_packet("", (void *)packet, (packet->length + 1));
    }
}


/* nRF52 Serial Proxy section */

static bool rx_worker(struct l_io *l_io, void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;
    bool result;

    if (pvt == NULL)
        return false;

    result = nrf_packet_receive(pvt->serial_fd, pvt->rx_buffer,
                                sizeof(pvt->rx_buffer), &pvt->rx_idx,
                                nrf_serial_packet_rx, io);
    if (!result)
        stat_report(io, STAT_RX_ERR);

    return true;
}



///////////////////////////////////////////////////////


static void ad_data_send_cb(uint32_t token, const nrf_serial_packet_t *packet, void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;

    if (pvt == NULL)
        return;

    if (packet == NULL) {
        l_debug("Sent packet not ACKed, token 0x%x", token);
        stat_report(io, STAT_TX_NO_ACK);
        return;
    }

    stat_report(io, STAT_TX_RSP);

    if (packet->payload.evt.cmd_rsp.status != SERIAL_STATUS_SUCCESS) {
        stat_report(io, STAT_TX_RSP_ERR);

        l_debug("Sent packet error: status 0x%x, token 0x%x",
            packet->payload.evt.cmd_rsp.status, token);
    }
}


static void ad_data_send(struct mesh_io *io, struct tx_pkt *tx)
{
    struct mesh_io_private *pvt = io->pvt;
    nrf_serial_packet_t packet;
    bool ret;

    if (pvt == NULL)
        return;

    l_debug("Send CMD_BLE_AD_DATA_SEND: token=0x%x", tx->token);

    packet.length = NRF_MESH_SERIAL_PACKET_OVERHEAD + NRF_MESH_SERIAL_CMD_OVERHEAD + tx->len + 1;
    packet.opcode = SERIAL_OPCODE_CMD_BLE_AD_DATA_SEND;
    packet.payload.cmd.token = tx->token;
    packet.payload.cmd.payload.ble_ad_data.data[0] = tx->len;
    memcpy(&packet.payload.cmd.payload.ble_ad_data.data[1], tx->pkt, tx->len);

    if (tx->delete) {
        l_queue_remove_if(pvt->tx_pkts, simple_match, tx);
        l_free(tx);
    }

    (void) nrf_packet_send(pvt->serial_fd, &packet);
    cmd_rsp_add(io, SERIAL_OPCODE_CMD_BLE_AD_DATA_SEND, tx->token,
                ad_data_send_cb, io);

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

    tx = l_queue_pop_head(pvt->tx_pkts);
    if (tx == NULL) {
        l_timeout_remove(timeout);
        pvt->tx_timeout = NULL;
        return;
    }

    if (tx->info.type == MESH_IO_TIMING_TYPE_GENERAL) {
        ms = tx->info.u.gen.interval;
        count = tx->info.u.gen.cnt;
        if (count != MESH_IO_TX_COUNT_UNLIMITED)
            tx->info.u.gen.cnt--;
    } else {
        ms = 25;
        count = 1;
    }

//l_debug("tx=%p, count=%u, interval=%u", tx, count, ms);

    tx->delete = !!(count == 1);
//l_debug("########## token=0x%x, count=%d, tx->delete=%d", tx->token, count, tx->delete);

    ad_data_send(io, tx);

    if (count == 1) {
        /* Recalculate wakeup if we are responding to POLL */
        tx = l_queue_peek_head(pvt->tx_pkts);

        if (tx != NULL && tx->info.type == MESH_IO_TIMING_TYPE_POLL_RSP) {
            ms = instant_remaining_ms(tx->info.u.poll_rsp.instant +
                        tx->info.u.poll_rsp.delay);
        }
    }
    else
        l_queue_push_tail(pvt->tx_pkts, tx);

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

    tx = l_queue_peek_head(pvt->tx_pkts);
    if (tx == NULL)
        return;

    switch (tx->info.type) {
    case MESH_IO_TIMING_TYPE_GENERAL:
l_debug("##### MESH_IO_TIMING_TYPE_GENERAL: token=0x%x, count=%u, interval=%u", tx->token, tx->info.u.gen.cnt, tx->info.u.gen.interval);
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


///////////////////////////////////////////////////////////////////


static void nrf_cmd_resp_housekeeping_data_get_cb(uint32_t token, const nrf_serial_packet_t *packet, void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;

    if (pvt == NULL)
        return;

    if (packet != NULL && packet->payload.evt.cmd_rsp.status == SERIAL_STATUS_SUCCESS) {
        stat_report_val(io, STAT_HW_ALLOC_ERR,
                        packet->payload.evt.cmd_rsp.data.hk_data.alloc_fail_count);
        stat_report_val(io, STAT_HW_RX_ERR,
                        packet->payload.evt.cmd_rsp.data.hk_data.rx_fail_count);

        nrf_cmd_send(io, SERIAL_OPCODE_CMD_HOUSEKEEPING_DATA_CLEAR, get_next_token(),
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

    nrf_cmd_send(io, SERIAL_OPCODE_CMD_HOUSEKEEPING_DATA_GET, get_next_token(),
                 0, NULL, nrf_cmd_resp_housekeeping_data_get_cb, io);

    l_timeout_modify_ms(timeout, ERROR_CHECK_PERIOD);
}


static void serial_disconnect_cb(struct l_io *serial_io, void *user_data)
{
//    struct mesh_io *io = user_data;
//    struct mesh_io_private *pvt = io->pvt;

    l_main_quit();
}



///////////////////////////////////////////////////////////////////

static void nrf_start(void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;
    bool result = true;
    int fd;

    if (pvt == NULL)
        return;

    l_debug("start");

    fd = nrf_uart_init(pvt->serial_path, pvt->serial_speed, pvt->serial_flags);
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
        result = nrf_cmd_send_reset(io);
    }

    if (!result && io->ready)
        io->ready(io->user_data, false);
}

static void nrf_stop(struct mesh_io *io)
{
    struct mesh_io_private *pvt = io->pvt;
    void *ptr;

    if (pvt == NULL)
        return;

    pvt->started = false;
    pvt->reset = false;

    l_timeout_remove(pvt->tx_timeout);
    pvt->tx_timeout = NULL;

    while ((ptr = l_queue_pop_head(pvt->tx_pkts)) != NULL) {
        l_free(ptr);
    }
    while ((ptr = l_queue_pop_head(pvt->tx_cmd_rsps)) != NULL) {
        l_free(ptr);
    }

    l_io_destroy(pvt->serial_io);
    pvt->serial_io = NULL;
    l_timeout_remove(pvt->error_check_timeout);
    pvt->error_check_timeout = NULL;

    close(pvt->serial_fd);
    pvt->serial_fd = -1;
}



/* interface functions */

static bool dev_init(struct mesh_io *io, void *opts, void *user_data)
{
    struct mesh_io_private *pvt;
    char *serial_path;

    if (io == NULL || io->pvt != NULL)
        return false;

    l_debug("Starting nRF52 IO");

    serial_path = (char *)opts;

    pvt = l_new(struct mesh_io_private, 1);

    pvt->io = io;

    pvt->serial_path = l_strdup(serial_path);
    pvt->serial_flags = FLOW_CTL;
    pvt->serial_speed = BAUD_RATE;
    pvt->serial_fd = -1;

    pvt->reset = false;
    pvt->started = false;

    pvt->serial_io = NULL;
    pvt->rx_idx = 0;

    pvt->tx_pkts = l_queue_new();
    pvt->tx_timeout = NULL;
    pvt->tx_cmd_rsps = l_queue_new();
    pvt->tx_cmd_rsp_timeout = NULL;

    io->pvt = pvt;

    stat_reset(io);

    l_idle_oneshot(nrf_start, io, NULL);

    return true;
}


static bool dev_destroy(struct mesh_io *io)
{
    struct mesh_io_private *pvt = io->pvt;

    if (pvt == NULL)
        return true;

    nrf_stop(io);

    l_timeout_remove(pvt->tx_timeout);
    l_queue_destroy(pvt->tx_pkts, l_free);
    l_timeout_remove(pvt->tx_cmd_rsp_timeout);
    l_queue_destroy(pvt->tx_cmd_rsps, l_free);


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


    tx = l_new(struct tx_pkt, 1);
    memcpy(&tx->info, info, sizeof(tx->info));
    memcpy(tx->pkt, data, len);
    tx->len = len;
    tx->token = get_next_token();

l_debug("***** tx=%p, token=0x%x", tx, tx->token);

    if (info->type == MESH_IO_TIMING_TYPE_POLL_RSP)
        l_queue_push_head(pvt->tx_pkts, tx);
    else {
        l_queue_push_tail(pvt->tx_pkts, tx);
    }

    l_timeout_remove(pvt->tx_timeout);
    pvt->tx_timeout = NULL;
    l_idle_oneshot(tx_worker, io, NULL);

    return true;
}


static bool tx_cancel(struct mesh_io *io, const uint8_t *data, uint8_t len)
{
    struct mesh_io_private *pvt = io->pvt;
    struct tx_pkt *tx;

    if (pvt == NULL || data == NULL)
        return false;

    if (len == 1) {
        do {
            tx = l_queue_remove_if(pvt->tx_pkts, find_by_ad_type,
                                   L_UINT_TO_PTR(data[0]));
            l_free(tx);
        } while (tx);
    } else {
        struct tx_pattern pattern = {
            .data = data,
            .len = len
        };

        do {
            tx = l_queue_remove_if(pvt->tx_pkts, find_by_pattern,
                                   &pattern);
            l_free(tx);
        } while (tx);
    }

    if (l_queue_isempty(pvt->tx_pkts)) {
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

    if (!pvt->started && !already_started) {
        (void)nrf_cmd_send_start(io);
    }

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

    if (!already_started) {
        (void)nrf_cmd_send_stop(io);
    }

    return true;
}



const struct mesh_io_api mesh_io_nrf52 = {
    .init = dev_init,
    .destroy = dev_destroy,
    .caps = dev_caps,
    .send = send_tx,
    .reg = recv_register,
    .dereg = recv_deregister,
    .cancel = tx_cancel,
};
