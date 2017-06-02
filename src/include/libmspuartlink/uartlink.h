#ifndef LIBMSPUARTLINK_UARTLINK_H
#define LIBMSPUARTLINK_UARTLINK_H

#include <stdint.h>

// We don't need any type ID, all packets are the same, because all that the
// radio server supports is pkts to be transmitted (no config cmds, etc.)
typedef struct __attribute__((packed)) {
    unsigned size:4;
    unsigned pay_chksum:2;
    unsigned hdr_chksum:2;
} ul_header_t;

#define UARTLINK_HDR_CHKSUM_MASK  0x0003 /* must match the bitfield len */
#define UARTLINK_PAYLOAD_CHKSUM_MASK 0x0003 /* must match the bitfield len */

#define UARTLINK_MAX_PAYLOAD_SIZE 15 /* size is 4-bit */

typedef union __attribute__((packed)) {
    ul_header_t typed;
    uint8_t raw;
} ul_header_ut;

typedef struct {
    ul_header_t header;
    uint8_t payload[UARTLINK_MAX_PAYLOAD_SIZE];
} ul_pkt_t;

#ifdef LIBMSPUARTLINK_PIN_RX_PORT
void uartlink_open_rx();
#endif // LIBMSPUARTLINK_PIN_RX_PORT

#ifdef LIBMSPUARTLINK_PIN_TX_PORT
void uartlink_open_tx();
#endif // LIBMSPUARTLINK_PIN_TX_PORT

#if defined(LIBMSPUARTLINK_PIN_TX_PORT) && defined(LIBMSPUARTLINK_PIN_RX_PORT)
void uartlink_open();
#endif // RX && TX

void uartlink_close();

// Returns length of received data, or zero if no pkt decoded
unsigned uartlink_receive(uint8_t *payload);

// Pushes bytes over UART synchronously (sleeps as much as possible)
void uartlink_send(uint8_t *payload, unsigned len);

#endif // LIBMSPUARTLINK_UARTLINK_H
