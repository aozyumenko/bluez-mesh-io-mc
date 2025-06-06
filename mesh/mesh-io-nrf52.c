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
#include <sys/time.h>
#include <time.h>
#include <ell/ell.h>

#include "src/shared/mgmt.h"
//#include "lib/bluetooth.h"
//#include "lib/mgmt.h"

#include "mesh/mesh-defs.h"
#include "mesh/mesh-mgmt.h"
#include "mesh/mesh-io.h"
#include "mesh/mesh-io-api.h"
#include "mesh/mesh-io-nrf52.h"
#include "mesh/nrf52-serial.h"



#define DEFAULT_ADDR            "\x55\x55\x55\x55\x55\x55"
#define CMD_RSP_TIMEOUT         50



struct mesh_io_private {
    struct mesh_io *io;

    char *serial_path;
    int serial_flags;
    int serial_speed;
    int serial_fd;
    ssize_t data_credit_available;

    bool reset;
    bool started;

    /* Rx packet */
    struct l_idle *rx_worker_task;
    uint8_t rx_buffer[NRF_SERIAL_MAX_ENCODED_PACKET_SIZE];
    int rx_idx;
    int rx_error_cnt;

    /* Tx packet */
    struct l_queue *tx_pkts;
    struct l_timeout *tx_timeout;
    struct l_queue *tx_cmd_rsps;
    struct l_timeout *tx_cmd_rsp_timeout;
    int tx_error_cnt;
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
static bool nrf_cmd_send(struct mesh_io *io, uint8_t opcode, uint32_t token, size_t length,
                         uint8_t *payload, cmd_rsp_cb_t cb, void *user_data);

static bool nrf_cmd_send_reset(struct mesh_io *io);
static bool nrf_cmd_send_serial_version_get(struct mesh_io *io);
static bool nrf_cmd_send_start(struct mesh_io *io);



// FIXME: for debuging only
#include <stdio.h>
void _dump(const char *data, size_t len)
{
    char str[80];
    int i;
    off_t offset = 0;

    for (i = 0; i < len; i++) {
        if (i > 0 && (i % 16) == 0) {
            str[offset] = '\0';
            l_debug("%s", str);
            offset = 0;
        }
        offset += sprintf(str + offset, "%02x ", (unsigned char)data[i]);
    }
    if (offset > 0) {
        str[offset] = '\0';
        l_debug("%s", str);
    }
}



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
        l_queue_remove_if(pvt->tx_cmd_rsps, simple_match, rsp);
        l_free(rsp);
        return true;
    }

    return false;
}


static bool nrf_cmd_send(struct mesh_io *io, uint8_t opcode, uint32_t token, size_t length,
                         uint8_t *payload, cmd_rsp_cb_t cb, void *user_data)
{
    struct mesh_io_private *pvt = io->pvt;
    nrf_serial_packet_t packet;
    bool ret;

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

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void nrf_cmd_resp_reset_cb(uint32_t token, const nrf_serial_packet_t *packet, void *user_data)
{
    struct mesh_io *io = user_data;

    if (packet != NULL || (packet == NULL && io->pvt->reset)) {
        l_error("Can't reset radio");
        (void) nrf_cmd_send_reset(io);
    }
}

static bool nrf_cmd_send_reset(struct mesh_io *io)
{
    l_debug("Send CMD_RESET");
    io->pvt->reset = true;
io->pvt->rx_idx = 0;     // FixMe: drop
    return nrf_cmd_send(io, SERIAL_OPCODE_CMD_RESET, get_next_token(),
                        0, NULL, nrf_cmd_resp_reset_cb, io);
}



static void nrf_cmd_resp_serial_version_get_cb(uint32_t token, const nrf_serial_packet_t *packet, void *user_data)
{
    struct mesh_io *io = user_data;

    if (packet != NULL && packet->payload.evt.cmd_rsp.status == SERIAL_STATUS_SUCCESS) {
        uint16_t api_ver = packet->payload.evt.cmd_rsp.data.serial_version.serial_ver;
        l_debug("Serial protocol version: %u.%u", (api_ver >> 8), (api_ver & 0xff));

        // TODO: real protocol version
        if (packet->payload.evt.cmd_rsp.data.serial_version.serial_ver == SERIAL_API_VERSION) {
            if (io->ready)
                io->ready(io->user_data, true);
        } else {
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
    return nrf_cmd_send(io, SERIAL_OPCODE_CMD_SERIAL_VERSION_GET, get_next_token(),
                        0, NULL, nrf_cmd_resp_serial_version_get_cb, io);
}



static void nrf_cmd_resp_start_cb(uint32_t token, const nrf_serial_packet_t *packet, void *user_data)
{
    struct mesh_io *io = user_data;

    if (packet != NULL && packet->payload.evt.cmd_rsp.status == SERIAL_STATUS_SUCCESS) {
        l_debug("Started radio");
        io->pvt->started = true;
    }
    else {
        l_error("Can't start radio");
        io->pvt->started = false;
    }
}

static bool nrf_cmd_send_start(struct mesh_io *io)
{
    l_debug("Send CMD_START");
    return nrf_cmd_send(io, SERIAL_OPCODE_CMD_START, get_next_token(),
                        0, NULL, nrf_cmd_resp_start_cb, io);
}


static void nrf_cmd_resp_stop_cb(uint32_t token, const nrf_serial_packet_t *packet, void *user_data)
{
    struct mesh_io *io = user_data;

    if (packet != NULL && packet->payload.evt.cmd_rsp.status == SERIAL_STATUS_SUCCESS) {
        l_debug("Stopped radio");
        io->pvt->started = false;
    }
    else {
        l_error("Can't stop radio");
    }
}

static bool nrf_cmd_send_stop(struct mesh_io *io)
{
    l_debug("Send CMD_STOP");
    return nrf_cmd_send(io, SERIAL_OPCODE_CMD_STOP, get_next_token(),
                        0, NULL, nrf_cmd_resp_stop_cb, io);
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

    l_debug("EVT: DEVICE_STARTED, operating_mode=0x%x, hw_error=0x%x, data_credit_available=%d",
        packet->payload.evt.started.operating_mode,
        packet->payload.evt.started.hw_error,
        packet->payload.evt.started.data_credit_available);

    if (packet->payload.evt.started.operating_mode == SERIAL_DEVICE_OPERATING_MODE_APPLICATION && packet->payload.evt.started.hw_error == 0) {
        io->pvt->data_credit_available = packet->payload.evt.started.data_credit_available;
        io->pvt->rx_error_cnt = 0;
        io->pvt->reset = false;

        /* get protocol version */
        (void)nrf_cmd_send_serial_version_get(io);
        // TODO check result and call io->ready()
    }
}


static void nrf_serial_handler_evt_ble_ad_data_received(const nrf_serial_packet_t *packet, void *user_data)
{
    struct mesh_io *io = user_data;
    uint32_t instant;
    const uint8_t *adv;
    uint8_t adv_len;
    const uint8_t *addr;
    int8_t rssi;

    if (packet->payload.evt.ble_ad_data.data[1] == 0x29) {
        l_debug("PB_ADV");
        _dump(packet->payload.evt.ble_ad_data.data, packet->payload.evt.ble_ad_data.data[0] + 1);
    }

    instant = get_instant();
    adv = packet->payload.evt.ble_ad_data.data + 1;
    adv_len = packet->payload.evt.ble_ad_data.data[0];
    addr = DEFAULT_ADDR;
    rssi = 0;

    process_rx(io->pvt, rssi, instant, addr, adv, adv_len);
}

static void nrf_serial_packet_rx(const nrf_serial_packet_t *packet, void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;
    cmd_rsp_cb_t cmd_rsp_cb;
    int i;

    if (packet->opcode == SERIAL_OPCODE_EVT_CMD_RSP) {
        (void)cmd_rsp_handle(io, packet);
//        nrf_cmd_rsp_wait_task_handle(packet);
    } else {
        for (i = 0; i < L_ARRAY_SIZE(packet_handler_list); i++) {
            if (packet_handler_list[i].opcode == packet->opcode) {
                packet_handler_list[i].handler(packet, io);
                return;
            }
        }

        io->pvt->rx_error_cnt++;
        l_error("Invalid packet opcode: 0x%02x, error count: %d",
            packet->opcode, io->pvt->rx_error_cnt);
        _dump((uint8_t *)packet, (packet->length + 1));
    }
}


/* nRF52 Serial Proxy section */

static void rx_worker(struct l_idle *idle, void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;
    bool result;

    if (io == NULL || io->pvt == NULL || io->pvt->serial_fd < 0)
        return;

    result = nrf_packet_receive(pvt->serial_fd, pvt->rx_buffer,
                                sizeof(pvt->rx_buffer), &pvt->rx_idx,
                                nrf_serial_packet_rx, io);

    if (!result) {
        pvt->rx_error_cnt++;
        l_debug("rx_error_cnt=%d", pvt->rx_error_cnt);
    }

    // TODO: check rx_error_cnt and tx_error_cnt
}



///////////////////////////////////////////////////////


static void nrf_cmd_resp_ble_ad_data_send_cb(uint32_t token, const nrf_serial_packet_t *packet, void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;

    if (pvt == NULL)
        return;

    if (packet == NULL || packet->payload.evt.cmd_rsp.status != SERIAL_STATUS_SUCCESS) {
        pvt->tx_error_cnt++;

        l_debug("Can't send BLE Advertising: %d, token=0x%x, tx_error_cnt=%u",
            packet != NULL ? packet->payload.evt.cmd_rsp.status : -1, token, pvt->tx_error_cnt);
    }
//    else {
//        l_debug("ACK packet: token=0x%x", token);
//    }
}


static void ad_data_send(struct mesh_io *io, struct tx_pkt *tx)
{
    struct mesh_io_private *pvt = io->pvt;
    nrf_serial_packet_t packet;
    bool ret;

    l_debug("Send CMD_BLE_AD_DATA_SEND: token=0x%x", tx->token);

    packet.length = NRF_MESH_SERIAL_PACKET_OVERHEAD + NRF_MESH_SERIAL_CMD_OVERHEAD + tx->len + 1;
    packet.opcode = SERIAL_OPCODE_CMD_BLE_AD_DATA_SEND;
    packet.payload.cmd.token = tx->token;
    packet.payload.cmd.payload.ble_ad_data.data[0] = tx->len;
    memcpy(&packet.payload.cmd.payload.ble_ad_data.data[1], tx->pkt, tx->len);

    if (tx->delete) {
l_debug("delete packet tx=%p", tx);
        l_queue_remove_if(pvt->tx_pkts, simple_match, tx);
        l_free(tx);
    }

    (void) nrf_packet_send(pvt->serial_fd, &packet);
    cmd_rsp_add(io, SERIAL_OPCODE_CMD_BLE_AD_DATA_SEND, tx->token,
                nrf_cmd_resp_ble_ad_data_send_cb, io);
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

static void nrf_start(void *user_data)
{
    struct mesh_io *io = user_data;
    struct mesh_io_private *pvt = io->pvt;
    bool result = true;
    int fd;

    l_debug("start");

    fd = nrf_uart_init(pvt->serial_path, pvt->serial_speed, pvt->serial_flags);
    if (fd < 0)
        result = false;

    if (result) {
        io->pvt->serial_fd = fd;
        io->pvt->rx_worker_task = l_idle_create(rx_worker, io, NULL);

        /* reset device */
        result = nrf_cmd_send_reset(io);
    }

    if (!result && io->ready)
        io->ready(io->user_data, false);
}

static void nrf_stop(struct mesh_io *io)
{
    struct mesh_io_private *pvt = io->pvt;

    pvt->started = false;
    pvt->reset = false;

    l_idle_remove(pvt->rx_worker_task);
    pvt->rx_worker_task = NULL;

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
    pvt->serial_speed = 1000000;
    pvt->serial_fd = -1;

    pvt->reset = false;
    pvt->started = false;

    pvt->rx_worker_task = NULL;
    pvt->rx_idx = 0;
    pvt->rx_error_cnt = 0;

    pvt->tx_pkts = l_queue_new();
    pvt->tx_timeout = NULL;
    pvt->tx_cmd_rsps = l_queue_new();
    pvt->tx_cmd_rsp_timeout = NULL;

    io->pvt = pvt;

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

    if (!pvt || !caps)
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

    if (!info || !data || !len || len > sizeof(tx->pkt))
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

    if (data == NULL)
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
