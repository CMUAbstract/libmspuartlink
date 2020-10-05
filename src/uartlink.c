#include <msp430.h>

#include <libio/console.h>
#include <libmsp/periph.h>
#include <libmsp/mem.h>
#include <libmspware/driverlib.h>
#include <libartibeus/comm.h>

#include "uartlink.h"

typedef enum {
    DECODER_STATE_HEADER  = 0x0,
    DECODER_STATE_PAYLOAD,
} decoder_state_t;


__nv uint8_t xfer_buffer[XFER_BUFFER_SIZE];
uint8_t xfer_count = 0;

#define RX_FIFO_SIZE 511
#define RX_FIFO_SIZE_MASK 0x1ff

// we want this volatile!
transfer comm_expt_link = {TRANSFER_DONE, 4, 0};
__nv uint8_t link_page[0x87];

// We want this volatile!!
uint8_t rf_kill_count = 0;
__nv uint8_t rf_dead = 0;

// We force this out into NVM for linking, but we'll let the head/tail be zero'd
// on reboots
static __nv uint8_t rx_fifo[3][RX_FIFO_SIZE];
static unsigned rx_fifo_head[3] = {0};
static unsigned rx_fifo_tail[3] = {0};

static uint8_t *tx_data[3];
static unsigned tx_len[3];

static decoder_state_t decoder_state[3] = {DECODER_STATE_HEADER};
static unsigned rx_payload_len[3] = {0};
static ul_header_t rx_header[3];

static void uartlink_configure(size_t port)
{
    switch(port) {
      case LIBMSPUARTLINK0_UART_IDX:
        UART(LIBMSPUARTLINK0_UART_IDX, CTL1) |= UCSWRST; // put state machine in reset
        UART(LIBMSPUARTLINK0_UART_IDX, CTL1) |= CONCAT(UCSSEL__, LIBMSPUARTLINK0_CLOCK);

        UART_SET_BR(LIBMSPUARTLINK0_UART_IDX, LIBMSPUARTLINK0_BAUDRATE_BR);
        UART_MCTL(LIBMSPUARTLINK0_UART_IDX) = UCBRF_0 | UART_BRS(LIBMSPUARTLINK0_BAUDRATE_BRS);

        UART(LIBMSPUARTLINK0_UART_IDX, CTL1) &= ~UCSWRST; // turn on
        break;
      case LIBMSPUARTLINK1_UART_IDX:
        UART(LIBMSPUARTLINK1_UART_IDX, CTL1) |= UCSWRST; // put state machine in reset
        UART(LIBMSPUARTLINK1_UART_IDX, CTL1) |= CONCAT(UCSSEL__, LIBMSPUARTLINK1_CLOCK);

        UART_SET_BR(LIBMSPUARTLINK1_UART_IDX, LIBMSPUARTLINK1_BAUDRATE_BR);
        UART_MCTL(LIBMSPUARTLINK1_UART_IDX) = UCBRF_0 | UART_BRS(LIBMSPUARTLINK1_BAUDRATE_BRS);

        UART(LIBMSPUARTLINK1_UART_IDX, CTL1) &= ~UCSWRST; // turn on
        break;
      case LIBMSPUARTLINK2_UART_IDX:
        UART(LIBMSPUARTLINK2_UART_IDX, CTL1) |= UCSWRST; // put state machine in reset
        UART(LIBMSPUARTLINK2_UART_IDX, CTL1) |= CONCAT(UCSSEL__, LIBMSPUARTLINK2_CLOCK);

        UART_SET_BR(LIBMSPUARTLINK2_UART_IDX, LIBMSPUARTLINK2_BAUDRATE_BR);
        UART_MCTL(LIBMSPUARTLINK2_UART_IDX) = UCBRF_0 | UART_BRS(LIBMSPUARTLINK2_BAUDRATE_BRS);

        UART(LIBMSPUARTLINK2_UART_IDX, CTL1) &= ~UCSWRST; // turn on
        break;
      default:
        UART(LIBMSPUARTLINK0_UART_IDX, CTL1) |= UCSWRST; // put state machine in reset
        UART(LIBMSPUARTLINK0_UART_IDX, CTL1) |= CONCAT(UCSSEL__, LIBMSPUARTLINK0_CLOCK);

        UART_SET_BR(LIBMSPUARTLINK0_UART_IDX, LIBMSPUARTLINK0_BAUDRATE_BR);
        UART_MCTL(LIBMSPUARTLINK0_UART_IDX) = UCBRF_0 | UART_BRS(LIBMSPUARTLINK0_BAUDRATE_BRS);

        UART(LIBMSPUARTLINK0_UART_IDX, CTL1) &= ~UCSWRST; // turn on
        break;
    }
}

void uartlink_open_rx(size_t port)
{
    switch(port) {
      case LIBMSPUARTLINK0_UART_IDX:
        UART_SET_SEL(LIBMSPUARTLINK0_PIN_RX_PORT, BIT(LIBMSPUARTLINK0_PIN_RX_PIN));
        uartlink_configure(port);
        UART(LIBMSPUARTLINK0_UART_IDX, IE) |= UCRXIE;
        break;
      case LIBMSPUARTLINK1_UART_IDX:
        UART_SET_SEL(LIBMSPUARTLINK1_PIN_RX_PORT, BIT(LIBMSPUARTLINK1_PIN_RX_PIN));
        uartlink_configure(port);
        UART(LIBMSPUARTLINK1_UART_IDX, IE) |= UCRXIE;
        break;
      case LIBMSPUARTLINK2_UART_IDX:
        UART_SET_SEL(LIBMSPUARTLINK2_PIN_RX_PORT, BIT(LIBMSPUARTLINK2_PIN_RX_PIN));
        uartlink_configure(port);
        UART(LIBMSPUARTLINK2_UART_IDX, IE) |= UCRXIE;
        break;
      default:
        UART_SET_SEL(LIBMSPUARTLINK0_PIN_RX_PORT, BIT(LIBMSPUARTLINK0_PIN_RX_PIN));
        uartlink_configure(port);
        UART(LIBMSPUARTLINK0_UART_IDX, IE) |= UCRXIE;
    }
}
void uartlink_open_tx(size_t port)
{
    switch(port) {
      case LIBMSPUARTLINK0_UART_IDX:
        UART_SET_SEL(LIBMSPUARTLINK0_PIN_TX_PORT, BIT(LIBMSPUARTLINK0_PIN_TX_PIN));
        uartlink_configure(LIBMSPUARTLINK0_UART_IDX);
        break;
      case LIBMSPUARTLINK1_UART_IDX:
        UART_SET_SEL(LIBMSPUARTLINK1_PIN_TX_PORT, BIT(LIBMSPUARTLINK1_PIN_TX_PIN));
        uartlink_configure(LIBMSPUARTLINK1_UART_IDX);
        break;
      case LIBMSPUARTLINK2_UART_IDX:
        UART_SET_SEL(LIBMSPUARTLINK2_PIN_TX_PORT, BIT(LIBMSPUARTLINK2_PIN_TX_PIN));
        uartlink_configure(LIBMSPUARTLINK2_UART_IDX);
        break;
      default:
        UART_SET_SEL(LIBMSPUARTLINK0_PIN_TX_PORT, BIT(LIBMSPUARTLINK0_PIN_TX_PIN));
        uartlink_configure(LIBMSPUARTLINK0_UART_IDX);
        break;
    }
}

#if (defined(LIBMSPUARTLINK0_PIN_TX_PORT) && \
defined(LIBMSPUARTLINK0_PIN_RX_PORT)) ||  \
(defined(LIBMSPUARTLINK1_PIN_TX_PORT) && \
defined(LIBMSPUARTLINK1_PIN_RX_PORT)) || \
(defined(LIBMSPUARTLINK2_PIN_TX_PORT) && \
defined(LIBMSPUARTLINK2_PIN_RX_PORT))
//TODO fix the EUSCI_A<x>_BASE so it's not hard coded
void uartlink_open(size_t port)
{
  switch(port) {
    case LIBMSPUARTLINK0_UART_IDX:

      UART_SET_SEL(LIBMSPUARTLINK0_PIN_RX_PORT, BIT(LIBMSPUARTLINK0_PIN_RX_PIN) |
        BIT(LIBMSPUARTLINK0_PIN_TX_PIN));
      uartlink_configure(port);

      EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

      // Enable USCI_A0 RX interrupt
      EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
      break;
    case LIBMSPUARTLINK1_UART_IDX:
      UART_SET_SEL(LIBMSPUARTLINK1_PIN_RX_PORT, BIT(LIBMSPUARTLINK1_PIN_RX_PIN) |
        BIT(LIBMSPUARTLINK1_PIN_TX_PIN));
      /*UART_SET_SEL(LIBMSPUARTLINK1_PIN_RX_PORT,
        BIT(LIBMSPUARTLINK1_PIN_TX_PIN));*/
      uartlink_configure(port);

      EUSCI_A_UART_clearInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

      // Enable USCI_A0 RX interrupt
      EUSCI_A_UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
      break;
    case LIBMSPUARTLINK2_UART_IDX:
      UART_SET_SEL(LIBMSPUARTLINK2_PIN_RX_PORT, BIT(LIBMSPUARTLINK2_PIN_RX_PIN) |
        BIT(LIBMSPUARTLINK2_PIN_TX_PIN));
      uartlink_configure(port);

      EUSCI_A_UART_clearInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

      // Enable USCI_A0 RX interrupt
      EUSCI_A_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
      break;
    default:
      break;
  }
}
#endif // Both tx and rx ports are defined

void uartlink_close(size_t port)
{
    switch (port) {
      case LIBMSPUARTLINK0_UART_IDX:
        UART(LIBMSPUARTLINK0_UART_IDX, CTL1) |= UCSWRST;
        break;
      case LIBMSPUARTLINK1_UART_IDX:
        UART(LIBMSPUARTLINK1_UART_IDX, CTL1) |= UCSWRST;
        break;
      case LIBMSPUARTLINK2_UART_IDX:
        UART(LIBMSPUARTLINK2_UART_IDX, CTL1) |= UCSWRST;
        break;
      default:
        UART(LIBMSPUARTLINK0_UART_IDX, CTL1) |= UCSWRST;
        break;
    }
}

static inline void uartlink_disable_interrupt(size_t port)
{
    switch (port) {
      case LIBMSPUARTLINK0_UART_IDX:
        UART(LIBMSPUARTLINK0_UART_IDX, IE) &= ~UCRXIE;
        break;
      case LIBMSPUARTLINK1_UART_IDX:
        UART(LIBMSPUARTLINK1_UART_IDX, IE) &= ~UCRXIE;
        break;
      case LIBMSPUARTLINK2_UART_IDX:
        UART(LIBMSPUARTLINK2_UART_IDX, IE) &= ~UCRXIE;
        break;
      default:
        UART(LIBMSPUARTLINK0_UART_IDX, IE) &= ~UCRXIE;
        break;
    }
}

static inline void uartlink_enable_interrupt(size_t port)
{
    switch (port) {
      case LIBMSPUARTLINK0_UART_IDX:
        UART(LIBMSPUARTLINK0_UART_IDX, IE) |= UCRXIE;
        break;
      case LIBMSPUARTLINK1_UART_IDX:
        UART(LIBMSPUARTLINK1_UART_IDX, IE) |= UCRXIE;
        break;
      case LIBMSPUARTLINK2_UART_IDX:
        UART(LIBMSPUARTLINK2_UART_IDX, IE) |= UCRXIE;
        break;
      default:
        UART(LIBMSPUARTLINK0_UART_IDX, IE) |= UCRXIE;
        break;
    }
}

static inline void send_byte(uint8_t *buf, unsigned len)
{
}

// Just like uartlink_send but without the checksum and packetizing
void uartlink_send_basic(size_t port, uint8_t *payload, unsigned len)
{

    // Setup pointers for the ISR
    tx_data[port] = payload;
    tx_len[port] = len;
    switch (port) {
      case LIBMSPUARTLINK0_UART_IDX:
        UART(LIBMSPUARTLINK0_UART_IDX, IE) |= UCTXIE;
        UART(LIBMSPUARTLINK0_UART_IDX, TXBUF) = *tx_data[port]++; // first byte, clears IFG
        tx_len[port]--;

        // Sleep, while ISR TXes the remaining bytes
        //
        // We have to disable TX int from ISR, otherwise, will enter infinite ISR loop.
        // So, we might as well use it as the sleep flag.
        __disable_interrupt(); // classic lock-check-(sleep+unlock)-lock pattern
        while (UART(LIBMSPUARTLINK0_UART_IDX, IE) & UCTXIE) {
            __bis_SR_register(LPM0_bits + GIE); // will wakeup after ISR TXes last byte
            __disable_interrupt();
        }
        __enable_interrupt();

        // TXIE is set before the last byte has finished transmitting
        while (UART(LIBMSPUARTLINK0_UART_IDX, STATW) & UCBUSY);
      break;
      case LIBMSPUARTLINK1_UART_IDX:
        UART(LIBMSPUARTLINK1_UART_IDX, IE) |= UCTXIE;
        UART(LIBMSPUARTLINK1_UART_IDX, TXBUF) = *tx_data[port]; // first byte, clears IFG

        // Sleep, while ISR TXes the remaining bytes
        //
        // We have to disable TX int from ISR, otherwise, will enter infinite ISR loop.
        // So, we might as well use it as the sleep flag.
        __disable_interrupt(); // classic lock-check-(sleep+unlock)-lock pattern
        while (UART(LIBMSPUARTLINK1_UART_IDX, IE) & UCTXIE) {
            __bis_SR_register(LPM0_bits + GIE); // will wakeup after ISR TXes last byte
            __disable_interrupt();
        }
        __enable_interrupt();

        // TXIE is set before the last byte has finished transmitting
        while (UART(LIBMSPUARTLINK1_UART_IDX, STATW) & UCBUSY);
      break;
      case LIBMSPUARTLINK2_UART_IDX:
        UART(LIBMSPUARTLINK2_UART_IDX, IE) |= UCTXIE;
        UART(LIBMSPUARTLINK2_UART_IDX, TXBUF) = *tx_data[port];// first byte, clears IFG

        // Sleep, while ISR TXes the remaining bytes
        //
        // We have to disable TX int from ISR, otherwise, will enter infinite ISR loop.
        // So, we might as well use it as the sleep flag.
        __disable_interrupt(); // classic lock-check-(sleep+unlock)-lock pattern
        while (UART(LIBMSPUARTLINK2_UART_IDX, IE) & UCTXIE) {
            __bis_SR_register(LPM0_bits + GIE); // will wakeup after ISR TXes last byte
            __disable_interrupt();
        }
        __enable_interrupt();

        // TXIE is set before the last byte has finished transmitting
        while (UART(LIBMSPUARTLINK2_UART_IDX, STATW) & UCBUSY);
      break;
      default:
        UART(LIBMSPUARTLINK2_UART_IDX, IE) |= UCTXIE;
        UART(LIBMSPUARTLINK2_UART_IDX, TXBUF) = *tx_data[port]; // first byte, clears IFG

        // Sleep, while ISR TXes the remaining bytes
        //
        // We have to disable TX int from ISR, otherwise, will enter infinite ISR loop.
        // So, we might as well use it as the sleep flag.
        __disable_interrupt(); // classic lock-check-(sleep+unlock)-lock pattern
        while (UART(LIBMSPUARTLINK2_UART_IDX, IE) & UCTXIE) {
            __bis_SR_register(LPM0_bits + GIE); // will wakeup after ISR TXes last byte
            __disable_interrupt();
        }
        __enable_interrupt();

        // TXIE is set before the last byte has finished transmitting
        while (UART(LIBMSPUARTLINK1_UART_IDX, STATW) & UCBUSY);
    }
}


void uartlink_send(size_t port, uint8_t *payload, unsigned len)
{
    // Payload checksum
    CRCINIRES = 0xFFFF; // initialize checksumer
    for (int i = 0; i < len; ++i) {
        CRCDI_L = *(payload + i);
    }
    uint8_t pay_chksum = CRCINIRES & UARTLINK_PAYLOAD_CHKSUM_MASK;

    ul_header_ut header = { .typed = { .size = len,
                                       .pay_chksum = pay_chksum,
                                       .hdr_chksum = 0 /* to be filled in shortly */ } };

    CRCINIRES = 0xFFFF; // initialize checksumer
    CRCDI_L = header.raw;
    header.typed.hdr_chksum = CRCINIRES & UARTLINK_HDR_CHKSUM_MASK;

    // Setup pointers for the ISR
    tx_data[port] = payload;
    tx_len[port] = len;
    switch (port) {
      case LIBMSPUARTLINK0_UART_IDX:
        UART(LIBMSPUARTLINK0_UART_IDX, IE) |= UCTXIE;
        UART(LIBMSPUARTLINK0_UART_IDX, TXBUF) = header.raw; // first byte, clears IFG

        // Sleep, while ISR TXes the remaining bytes
        //
        // We have to disable TX int from ISR, otherwise, will enter infinite ISR loop.
        // So, we might as well use it as the sleep flag.
        __disable_interrupt(); // classic lock-check-(sleep+unlock)-lock pattern
        while (UART(LIBMSPUARTLINK0_UART_IDX, IE) & UCTXIE) {
            __bis_SR_register(LPM0_bits + GIE); // will wakeup after ISR TXes last byte
            __disable_interrupt();
        }
        __enable_interrupt();

        // TXIE is set before the last byte has finished transmitting
        while (UART(LIBMSPUARTLINK0_UART_IDX, STATW) & UCBUSY);
      break;
      case LIBMSPUARTLINK1_UART_IDX:
        UART(LIBMSPUARTLINK1_UART_IDX, IE) |= UCTXIE;
        UART(LIBMSPUARTLINK1_UART_IDX, TXBUF) = header.raw; // first byte, clears IFG

        // Sleep, while ISR TXes the remaining bytes
        //
        // We have to disable TX int from ISR, otherwise, will enter infinite ISR loop.
        // So, we might as well use it as the sleep flag.
        __disable_interrupt(); // classic lock-check-(sleep+unlock)-lock pattern
        while (UART(LIBMSPUARTLINK1_UART_IDX, IE) & UCTXIE) {
            __bis_SR_register(LPM0_bits + GIE); // will wakeup after ISR TXes last byte
            __disable_interrupt();
        }
        __enable_interrupt();

        // TXIE is set before the last byte has finished transmitting
        while (UART(LIBMSPUARTLINK1_UART_IDX, STATW) & UCBUSY);
      break;
      case LIBMSPUARTLINK2_UART_IDX:
        UART(LIBMSPUARTLINK2_UART_IDX, IE) |= UCTXIE;
        UART(LIBMSPUARTLINK2_UART_IDX, TXBUF) = header.raw; // first byte, clears IFG

        // Sleep, while ISR TXes the remaining bytes
        //
        // We have to disable TX int from ISR, otherwise, will enter infinite ISR loop.
        // So, we might as well use it as the sleep flag.
        __disable_interrupt(); // classic lock-check-(sleep+unlock)-lock pattern
        while (UART(LIBMSPUARTLINK2_UART_IDX, IE) & UCTXIE) {
            __bis_SR_register(LPM0_bits + GIE); // will wakeup after ISR TXes last byte
            __disable_interrupt();
        }
        __enable_interrupt();

        // TXIE is set before the last byte has finished transmitting
        while (UART(LIBMSPUARTLINK2_UART_IDX, STATW) & UCBUSY);
      break;
      default:
        UART(LIBMSPUARTLINK2_UART_IDX, IE) |= UCTXIE;
        UART(LIBMSPUARTLINK2_UART_IDX, TXBUF) = header.raw; // first byte, clears IFG

        // Sleep, while ISR TXes the remaining bytes
        //
        // We have to disable TX int from ISR, otherwise, will enter infinite ISR loop.
        // So, we might as well use it as the sleep flag.
        __disable_interrupt(); // classic lock-check-(sleep+unlock)-lock pattern
        while (UART(LIBMSPUARTLINK2_UART_IDX, IE) & UCTXIE) {
            __bis_SR_register(LPM0_bits + GIE); // will wakeup after ISR TXes last byte
            __disable_interrupt();
        }
        __enable_interrupt();

        // TXIE is set before the last byte has finished transmitting
        while (UART(LIBMSPUARTLINK1_UART_IDX, STATW) & UCBUSY);
    }
}




// Should be called whenever MCU wakes up, from the context of a main loop
// precondition: payload points to a buffer of at least UARTLINK_MAX_PAYLOAD_SIZE
unsigned uartlink_receive(size_t port, uint8_t *payload)
{
    unsigned rx_pkt_len = 0;

    uartlink_disable_interrupt(port);
    // keep processing fifo content until empty, or until got pkt
    while (rx_fifo_head[port] != rx_fifo_tail[port] && !rx_pkt_len) {
        uint8_t rx_byte = rx_fifo[port][rx_fifo_head[port]++];
        rx_fifo_head[port] &= RX_FIFO_SIZE_MASK; // wrap around
        uartlink_enable_interrupt(port);

        //LOG2("uartlink: rcved: %02x\r\n", rx_byte);

        switch (decoder_state[port]) {
            case DECODER_STATE_HEADER: {

                ul_header_ut header = { .raw = rx_byte };

                ul_header_ut header_unchksumed = header;
                header_unchksumed.typed.hdr_chksum = 0;
                CRCINIRES = 0xFFFF; // initialize checksum'er for header
                CRCDI_L = header_unchksumed.raw;

                unsigned hdr_chksum_local = CRCINIRES & UARTLINK_HDR_CHKSUM_MASK;
                if (hdr_chksum_local == header.typed.hdr_chksum) {
                    if (header.typed.size > 0) {
                        LOG("hdr 0x%02x: size %u | chksum: hdr 0x%02x pay 0x%02x\r\n",
                            header.raw, header.typed.size,
                            header.typed.hdr_chksum, header.typed.pay_chksum);

                        rx_header[port] = header.typed;
                        rx_payload_len[port] = 0; // init packet
                        CRCINIRES = 0xFFFF; // initialize checksum'er for payload
                        decoder_state[port] = DECODER_STATE_PAYLOAD;
                    } else {
                        LOG("uartlink: finished receiving pkt: len %u\r\n",
                        rx_payload_len[port]);
                        rx_pkt_len = 0;
                    }
                } else {
                    LOG("uartlink: hdr chksum mismatch (0x%02x != 0x%02x)\r\n",
                        hdr_chksum_local, header.typed.hdr_chksum);
                }
                break;
            }
            case DECODER_STATE_PAYLOAD:
                // assert: pkt.header.size < UARTLINK_MAX_PAYLOAD_SIZE
                payload[rx_payload_len[port]++] = rx_byte;
                CRCDI_L = rx_byte;
                if (rx_payload_len[port] == rx_header[port].size) {
                    // check payload checksum
                    uint8_t rx_payload_chksum = CRCINIRES & UARTLINK_PAYLOAD_CHKSUM_MASK;
                    if (rx_payload_chksum != rx_header[port].pay_chksum) {
                        LOG("uartlink: payload chksum mismatch (0x%02x != 0x%02x)\r\n",
                            rx_payload_chksum, rx_header[port].pay_chksum);
                        // reset decoder
                        rx_payload_len[port] = 0;
                        decoder_state[port] = DECODER_STATE_HEADER;
                    } else {
                        LOG("uartlink: finished receiving pkt: len %u\r\n",
                        rx_payload_len[port]);
                        rx_pkt_len = rx_payload_len[port];

                        // reset decoder
                        rx_payload_len[port] = 0;
                        decoder_state[port] = DECODER_STATE_HEADER;
                    }
                } else if (rx_payload_len[port] == UARTLINK_MAX_PAYLOAD_SIZE) {
                    LOG("uartlink: payload too long\r\n");
                    // reset decoder
                    rx_payload_len[port] = 0;
                    decoder_state[port] = DECODER_STATE_HEADER;
                }
                break;
        }

        uartlink_disable_interrupt(port); // classic: check-and-...-sleep pattern
    }
    uartlink_enable_interrupt(port);
    return rx_pkt_len;
}

// Similar operation to uartlink_receive but without the checksum and we use the
// last argument to indicate how many bytes to read
unsigned uartlink_receive_basic(size_t port, uint8_t *payload, unsigned size)
{
    unsigned rx_pkt_len = 0;

    uartlink_disable_interrupt(port);
    // keep processing fifo content until empty, or until got pkt
    while (rx_fifo_head[port] != rx_fifo_tail[port] && !rx_pkt_len) {
        uint8_t rx_byte = rx_fifo[port][rx_fifo_head[port]++];
        rx_fifo_head[port] &= RX_FIFO_SIZE_MASK; // wrap around
        uartlink_enable_interrupt(port);

        //LOG("uartlink: rcved: %02x %c\r\n", rx_byte,rx_byte);

        // assert: pkt.header.size < UARTLINK_MAX_PAYLOAD_SIZE
        payload[rx_payload_len[port]++] = rx_byte;
        if (rx_payload_len[port] == size) {
                LOG("uartlink: finished receiving pkt: len %u\r\n",
                rx_payload_len[port]);
                rx_pkt_len = rx_payload_len[port];
                // reset decoder
                rx_payload_len[port] = 0;
        }
        else if (rx_payload_len[port] == UARTLINK_MAX_PAYLOAD_SIZE) {
            LOG("uartlink: payload too long %u\r\n", rx_payload_len[port]);
            // reset decoder
            rx_payload_len[port] = 0;
        }

        uartlink_disable_interrupt(port); // classic: check-and-...-sleep pattern
    }
    uartlink_enable_interrupt(port);
    return rx_pkt_len;
}

//COMM UART most of the time
// Needs to look for transfer_active and transfer_done messages.
#if defined(LIBMSPUARTLINK0_UART_IDX) && !defined(CONSOLE)
__nv incoming_status_t progress;
__nv uint8_t incoming_cmd;
__nv char earth_msg[32] = {'a','b','c','d','e','f','g','h',
'i','j','k','l','m','n','o','p',' ','W','e','r','e','i','n',
's','p','a','c','e',' ','n','a','t'};
__nv char score_msg[32];
static uint16_t prog_len = 0;
static uint8_t prog_counter = 0;
static uint8_t prog_keys[16] = {0};

// Returns 1 if active transfer, 0 if not
static int handle_progress(uint8_t data) {
  uint8_t ascii_prog;
  ascii_prog = progress + 48;
    P1OUT |= BIT1;
    P1DIR |= BIT1;
    P1OUT &= ~BIT1;
  switch(progress) {
    case wait_esp0:
      prog_counter = 0;
      if (data == ESP_BYTE0) {
        P1OUT |= BIT1;
        P1DIR |= BIT1;
        P1OUT &= ~BIT1;
        //uartlink_send_basic(0,&progress,1);
        progress = wait_esp1;
      }
      break;
    case wait_esp1:
      prog_counter = 0;
      if (data == ESP_BYTE1) {
        P1OUT |= BIT1;
        P1DIR |= BIT1;
        P1OUT &= ~BIT1;
        prog_len = 0;
        progress = wait_len;
        //uartlink_send_basic(0,&progress,1);
      }
      break;
    case wait_len:
      //uartlink_send_basic(0,&progress,1);
      P1OUT |= BIT1;
      P1DIR |= BIT1;
      P1OUT &= ~BIT1;
      prog_len = data;
      prog_counter = 0;
      progress = wait_cmd;
      if (comm_expt_link.status == TRANSFER_ACTIVE) {
        comm_expt_link.transfer_len = prog_len + 3;
      }
      break;
    case wait_cmd:
      if (prog_counter == 5) {
        if (data == ASCII) {
          P1OUT |= BIT1;
          P1DIR |= BIT1;
          P1OUT &= ~BIT1;
          prog_counter = 0;
          progress = wait_sub_cmd;
        }
        else {
          progress = wait_esp0;
        }
          prog_counter = 0;
      }
      else {
        prog_counter++;
      }
      break;
    case wait_sub_cmd:
        P1OUT |= BIT1;
        P1DIR |= BIT1;
        P1OUT &= ~BIT1;
      if (data == SCORE) {
        prog_counter= 0;
        progress = wait_score;
      }
      else if (data == EXPT_WAKE || data == RF_KILL) {
          P1OUT |= BIT1;
          P1DIR |= BIT1;
          P1OUT &= ~BIT1;
          incoming_cmd = data;
          prog_counter = 0;
          progress = wait_keys;
      }
      else if (data == EXPT_DONE) {
        incoming_cmd = data;
        comm_expt_link.status == TRANSFER_DONE;
        progress = wait_esp0;
      }
      else {
        if (comm_expt_link.status == TRANSFER_ACTIVE) {
          prog_counter = 0;
          progress = wait_esp0;
        }
        else {
          prog_counter = 0;
          progress = wait_msg;
        }
      }
      break;
    case wait_keys:
      // process prog_keys into prog_keys array until we reach required number for
      // bootloader and kill
      P1OUT |= BIT1;
      P1DIR |= BIT1;
      P1OUT &= ~BIT1;
      prog_keys[prog_counter] = data;
      if (incoming_cmd == EXPT_WAKE && prog_counter == 7) {
        //check prog_keys
        int flag = 0;
        for (int i = 0; i < 8; i++) {
          if (prog_keys[i] != EXPT_WAKE_KEYS[i]) {
            flag = 1;
            break;
          }
        }
        if (flag) {
          progress = wait_esp0;
        }
        else {
      P1OUT |= BIT1;
      P1DIR |= BIT1;
      P1OUT &= ~BIT1;
      P1OUT |= BIT1;
      P1DIR |= BIT1;
      P1OUT &= ~BIT1;
      P1OUT |= BIT1;
      P1DIR |= BIT1;
      P1OUT &= ~BIT1;
      P1OUT |= BIT1;
      P1DIR |= BIT1;
      P1OUT &= ~BIT1;
          //EXPT_WAKE_OK
          comm_expt_link.status = TRANSFER_ACTIVE;
          // Zero out xfer_count
          xfer_count = 0;
          // Set to 5 until we overwrite it at length
          comm_expt_link.transfer_len = 4;
        }
      }
      else if (incoming_cmd == RF_KILL && prog_counter == 15) {
        int flag;
        flag = 0;
        for (int i = 0; i < 15; i++) {
          // check prog_keys
          if (prog_keys[i] != RF_KILL_KEYS[i]) {
            flag = 1;
            break;
          }
            /*P1OUT |= BIT1;
            P1DIR |= BIT1;
            P1OUT &= ~BIT1;*/
        }
        if (flag == 1) {
          progress = wait_esp0;
        }
        else {
          //RF_KILL_OK
          P1OUT |= BIT1;
          P1DIR |= BIT1;
          P1OUT &= ~BIT1;
          P1OUT |= BIT1;
          P1DIR |= BIT1;
          P1OUT &= ~BIT1;
          // Increment kill count
          rf_kill_count++;
          if (rf_kill_count >= 5) {
            P1OUT |= BIT1;
            P1DIR |= BIT1;
            P1OUT &= ~BIT1;
            rf_dead = 0xAB;
          }
          // Change progress first so we don't rerun this
          progress = wait_esp0;
        }
      }
      else {
        prog_counter++;
      }
      if (prog_counter > 16) {
        // This shouldn't happen!
        progress = wait_esp0;
      }
      break;
    case wait_msg:
      //uartlink_send_basic(0,&ascii_prog,1);
    // We do this directly so that if it gets garbled we'll know immediately
      earth_msg[prog_counter] = data;
      prog_counter++;
      if (prog_counter == prog_len - 6) {
        progress = wait_esp0;
      }
      break;
    case wait_score:
      score_msg[prog_counter] = data;
      prog_counter++;
      if (prog_counter == prog_len - 6) {
        progress = wait_esp0;
      }
      break;
    default:
      progress = wait_esp0;
      break;
  }
  return 0;
}
__attribute__ ((interrupt(UART_VECTOR(LIBMSPUARTLINK0_UART_IDX))))
void UART_ISR(LIBMSPUARTLINK0_UART_IDX) (void)
{
    switch(__even_in_range(UART(LIBMSPUARTLINK0_UART_IDX, IV),0x08)) {
        case UART_INTFLAG(TXIFG):
            if (tx_len[0]--) {
                UART(LIBMSPUARTLINK0_UART_IDX, TXBUF) = *tx_data[0]++;
            } else { // last byte got done
                UART(LIBMSPUARTLINK0_UART_IDX, IE) &= ~UCTXIE;
                __bic_SR_register_on_exit(LPM4_bits); // wakeup
            }
            break; // nothing to do, main thread is sleeping, so just wakeup
        case UART_INTFLAG(RXIFG):
        {
            // Put data at back of buffer
            uint8_t data = UART(LIBMSPUARTLINK0_UART_IDX, RXBUF);
            handle_progress(data);
            // If wormhole active, write to other uart 
            if (comm_expt_link.status == TRANSFER_ACTIVE) {
              xfer_buffer[xfer_count] = UART(LIBMSPUARTLINK1_UART_IDX, RXBUF);
              xfer_count++;
              if (xfer_count > XFER_BUFFER_SIZE ||
                xfer_count >= comm_expt_link.transfer_len) {
                // Set ready
                comm_expt_link.ready = 1;
              }
            }
            else {
              rx_fifo[0][rx_fifo_tail[0]] = data;
              // Increment buffer size
              rx_fifo_tail[0] = (rx_fifo_tail[0] + 1) & RX_FIFO_SIZE_MASK; //
              //wrap-around (assumes size is power of 2)

              if (rx_fifo_tail[0] == rx_fifo_head[0]) {
                  // overflow, throw away the received byte, by rolling back NOTE:
                  // tail == head happens both when full and empty, so can't use
                  // that as overflow check
                  rx_fifo_tail[0] = (rx_fifo_tail[0] - 1) & RX_FIFO_SIZE_MASK; //
                  //wrap-around (assumes size is power of 2)
              }
            }

            __bic_SR_register_on_exit(LPM4_bits); // wakeup
            break;
        }
        default:
            break;
    }
}
#endif

//EXPT UART most of the time
#ifdef LIBMSPUARTLINK1_UART_IDX
__attribute__ ((interrupt(UART_VECTOR(LIBMSPUARTLINK1_UART_IDX))))
void UART_ISR(LIBMSPUARTLINK1_UART_IDX) (void)
{
    switch(__even_in_range(UART(LIBMSPUARTLINK1_UART_IDX, IV),0x08)) {
        case UART_INTFLAG(TXIFG):
            if (tx_len[1]--) {
                UART(LIBMSPUARTLINK1_UART_IDX, TXBUF) = *tx_data[1]++;
            } else { // last byte got done
                UART(LIBMSPUARTLINK1_UART_IDX, IE) &= ~UCTXIE;
                __bic_SR_register_on_exit(LPM4_bits); // wakeup
            }
            break; // nothing to do, main thread is sleeping, so just wakeup
        case UART_INTFLAG(RXIFG):
        {

            rx_fifo[1][rx_fifo_tail[1]] = UART(LIBMSPUARTLINK1_UART_IDX, RXBUF);
            rx_fifo_tail[1] = (rx_fifo_tail[1] + 1) & RX_FIFO_SIZE_MASK; //
            //wrap-around (assumes size is power of 2)

            if (rx_fifo_tail[1] == rx_fifo_head[1]) {
                // overflow, throw away the received byte, by rolling back NOTE:
                // tail == head happens both when full and empty, so can't use
                // that as overflow check
                rx_fifo_tail[1] = (rx_fifo_tail[1] - 1) & RX_FIFO_SIZE_MASK; //
                //wrap-around (assumes size is power of 2)
            }

            __bic_SR_register_on_exit(LPM4_bits); // wakeup
            break;
        }
        default:
            break;
    }
}
#endif

//GNSS on artibeus (most of the time)
#ifdef LIBMSPUARTLINK2_UART_IDX
__attribute__ ((interrupt(UART_VECTOR(LIBMSPUARTLINK2_UART_IDX))))
void UART_ISR(LIBMSPUARTLINK2_UART_IDX) (void)
{
    switch(__even_in_range(UART(LIBMSPUARTLINK2_UART_IDX, IV),0x08)) {
        case UART_INTFLAG(TXIFG):
            if (tx_len[2]--) {
                UART(LIBMSPUARTLINK2_UART_IDX, TXBUF) = *tx_data[2]++;
            } else { // last byte got done
                UART(LIBMSPUARTLINK2_UART_IDX, IE) &= ~UCTXIE;
                __bic_SR_register_on_exit(LPM4_bits); // wakeup
            }
            break; // nothing to do, main thread is sleeping, so just wakeup
        case UART_INTFLAG(RXIFG):
        {   // Not failure safe?
            rx_fifo[2][rx_fifo_tail[2]] = UART(LIBMSPUARTLINK2_UART_IDX, RXBUF);
            rx_fifo_tail[2] = (rx_fifo_tail[2] + 1) & RX_FIFO_SIZE_MASK; //
            //wrap-around (assumes size is power of 2)

            if (rx_fifo_tail[2] == rx_fifo_head[2]) {
                // overflow, throw away the received byte, by rolling back NOTE:
                // tail == head happens both when full and empty, so can't use
                // that as overflow check
                rx_fifo_tail[2] = (rx_fifo_tail[2] - 1) & RX_FIFO_SIZE_MASK; //
                //wrap-around (assumes size is power of 2)
            }

            __bic_SR_register_on_exit(LPM4_bits); // wakeup
            break;
        }
        default:
            break;
    }
}
#endif


