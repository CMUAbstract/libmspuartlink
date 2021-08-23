#ifndef LIBMSPUARTLINK_UARTLINK_H
#define LIBMSPUARTLINK_UARTLINK_H

#include <stdint.h>
#include <libartibeus/comm.h>


// We don't need any type ID, all packets are the same, because all that the
// radio server supports is pkts to be transmitted (no config cmds, etc.)
typedef struct __attribute__((packed)) {
    // Note: little-endian means that field order in memory is opposite of order here
    unsigned hdr_chksum:2;
    unsigned pay_chksum:1;
    unsigned size:5;
} ul_header_t;

#define UARTLINK_HDR_CHKSUM_MASK  0x1FF /* must match the bitfield len */
#define UARTLINK_PAYLOAD_CHKSUM_MASK 0x01 /* must match the bitfield len */

#define UARTLINK_MAX_PAYLOAD_SIZE 511 /* size is 5-bit */

typedef union __attribute__((packed)) {
    ul_header_t typed;
    uint8_t raw;
} ul_header_ut;

typedef struct {
    ul_header_t header;
    uint8_t payload[UARTLINK_MAX_PAYLOAD_SIZE];
} ul_pkt_t;

typedef enum {
  wait_esp0,
  wait_esp1,
  wait_len,
  wait_cmd,
  wait_sub_cmd,
  wait_keys,
  wait_score,
  wait_msg,
  wait_page,
  wait_extra,
  receive_data,
} incoming_status_t;


#if defined(LIBMSPUARTLINK0_PIN_RX_PORT) || \
defined(LIBMSPUARTLINK1_PIN_RX_PORT) || \
defined(LIBMSPUARTLINK2_PIN_RX_PORT)
void uartlink_open_rx(size_t port);
#endif // LIBMSPUARTLINK_PIN_RX_PORT

#if defined(LIBMSPUARTLINK0_PIN_TX_PORT) || \
defined(LIBMSPUARTLINK1_PIN_TX_PORT) || \
defined(LIBMSPUARTLINK2_PIN_TX_PORT)
void uartlink_open_tx(size_t port);
#endif // LIBMSPUARTLINK_PIN_TX_PORT

#if (defined(LIBMSPUARTLINK0_PIN_TX_PORT) && \
defined(LIBMSPUARTLINK0_PIN_RX_PORT)) ||  \
(defined(LIBMSPUARTLINK1_PIN_TX_PORT) && \
defined(LIBMSPUARTLINK1_PIN_RX_PORT)) || \
(defined(LIBMSPUARTLINK2_PIN_TX_PORT) && \
defined(LIBMSPUARTLINK2_PIN_RX_PORT))
void uartlink_open(size_t port);
#endif // RX && TX

void uartlink_close(size_t port);

// Returns length of received data, or zero if no pkt decoded
unsigned uartlink_receive(size_t port, uint8_t *payload);

// Same as uartlink_receive, except we don't encode a checksum or use nice
// header format, so we need to read in a certain number of bytes
unsigned uartlink_receive_basic(size_t port, uint8_t *payload, unsigned size);

// Pushes bytes over UART synchronously (sleeps as much as possible)
void uartlink_send(size_t port, uint8_t *payload, unsigned len);

// Same as uartlink_send, except we don't encode a checksum
void uartlink_send_basic(size_t port, uint8_t *payload, unsigned len);

#define XFER_BUFFER_SIZE 256 //TODO confirm this

extern uint8_t xfer_buffer[XFER_BUFFER_SIZE];

// These are really just for internal use, but we split up packet processing
typedef enum {
    DECODER_STATE_HEADER  = 0x0,
    DECODER_STATE_PAYLOAD,
} decoder_state_t;

#define RX_FIFO_SIZE 511
#define RX_FIFO_SIZE_MASK 0x1ff

extern __nv uint8_t rx_fifo[3][RX_FIFO_SIZE];
extern unsigned rx_fifo_head[3] ;
extern unsigned rx_fifo_tail[3] ;
extern uint8_t *tx_data[3];
extern unsigned tx_len[3];
extern decoder_state_t decoder_state[3] ;
extern unsigned rx_payload_len[3] ;
extern ul_header_t rx_header[3];

#define UART0_BUFFER_CNT 2
#define UART1_BUFFER_CNT 2

#define COMPLETE 0xa5

#endif // LIBMSPUARTLINK_UARTLINK_H
