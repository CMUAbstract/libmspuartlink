#ifndef RADIO_COMM_H
#define RADIO_COMM_H

#include <stdint.h>

// We don't need any type ID, all packets are the same, because all that the
// radio server supports is pkts to be transmitted (no config cmds, etc.)
typedef struct __attribute__((packed)) {
    unsigned size:4;
    unsigned hdr_chksum:2;
    unsigned pay_chksum:2;
} pkt_header_t;

#define UARTLINK_HDR_CHKSUM_MASK  0x0003 /* must match the bitfield len */
#define UARTLINK_PAYLOAD_CHKSUM_MASK 0x0003 /* must match the bitfield len */

#define UARTLINK_MAX_PAYLOAD_SIZE 15 /* size is 4-bit */

typedef union __attribute__((packed)) {
    pkt_header_t typed;
    uint8_t raw;
} pkt_header_ut;

typedef struct {
    pkt_header_t header;
    uint8_t payload[UARTLINK_MAX_PAYLOAD_SIZE];
} pkt_t;

void uartlink_open();
void uartlink_close();

// Returns length of received data, or zero if no pkt decoded
unsigned uartlink_receive(uint8_t *payload);

#endif // RADIO_COMM_H
