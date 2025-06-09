#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <ell/ell.h>
#include "util.h"
#include "src/shared/tty.h"
#include "nrf52-serial.h"



#define SLIP_END                        0xc0
#define SLIP_ESC                        0xdb
#define SLIP_ESC_END                    0xdc
#define SLIP_ESC_ESC                    0xdd



struct l_queue *cmd_rsp_wait_list;

///////////////////////////////////////////////////////


/* Set UART speed */
static int uart_set_speed(int fd, struct termios *ti, int speed)
{
    if (cfsetospeed(ti, tty_get_speed(speed)) < 0)
        return -errno;

    if (cfsetispeed(ti, tty_get_speed(speed)) < 0)
        return -errno;

    if (tcsetattr(fd, TCSANOW, ti) < 0)
        return -errno;

    return 0;
}


static size_t nrf_slip_encode(const uint8_t *src, ssize_t src_len,
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


static size_t nrf_slip_decode(const uint8_t *src, ssize_t src_len,
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



/* interface functions */

int nrf_uart_init(const char *filename, int speed, int flags)
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
    if (flags & FLOW_CTL)
        ti.c_cflag |= CRTSCTS;
    else
        ti.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &ti) < 0) {
        l_error("Can't set port settings: %s", strerror(errno));
        goto fail;
    }

    if (uart_set_speed(fd, &ti, speed) < 0) {
        l_error("Can't set baud rate: %s", strerror(errno));
        goto fail;
    }

    return fd;

fail:
    close(fd);
    return -1;
}


bool nrf_packet_send(int fd, const nrf_serial_packet_t *packet)
{
    uint8_t encoded_packet[sizeof(*packet) * 2 + 2];    /* MAX = END + <every byte is ESC or END> + END */
    size_t encoded_packet_length = 0;
    ssize_t bytes;

    encoded_packet[0] = SLIP_END;
    encoded_packet_length++;

    encoded_packet_length += nrf_slip_encode((uint8_t *)packet, packet->length + 1,
                                            encoded_packet + encoded_packet_length,
                                            sizeof(encoded_packet) - 2);

    encoded_packet[encoded_packet_length] = SLIP_END;
    encoded_packet_length++;

//    print_packet("send packet", (void *)packet, packet->length + 1);

    bytes = write(fd, encoded_packet, encoded_packet_length);
    if (bytes != encoded_packet_length) {
        l_error("Can't send packet to serial port: %s", strerror(errno));
        return false;
    }

    return true;
}



bool nrf_packet_receive(int fd, uint8_t *rx_buffer, size_t rx_buffer_size, int *rx_idx,
                        nrf_packet_rx_cb_t rx_cb, void *user_data)
{
    uint8_t buf[NRF_SERIAL_MAX_ENCODED_PACKET_SIZE];
    size_t bytes;
    const nrf_serial_packet_t packet;
    size_t packet_length;
    int src_idx, dst_idx;

    bytes = read(fd, buf, sizeof(buf));
    if (bytes <= 0)
        return false;

    dst_idx = *rx_idx;
    for (src_idx = 0; src_idx < bytes; src_idx++) {
        if (buf[src_idx] == SLIP_END) {
            if (dst_idx > 0) {
                packet_length = nrf_slip_decode(rx_buffer, dst_idx,
                                                (uint8_t *)&packet, sizeof(packet));

//                print_packet("receive packet", (void *)&packet, packet_length);

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
