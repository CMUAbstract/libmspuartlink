#include <msp430.h>

#include <libio/console.h>
#include <libmsp/periph.h>

#include "uartlink.h"

typedef enum {
    DECODER_STATE_HEADER  = 0x0,
    DECODER_STATE_PAYLOAD,
} decoder_state_t;

#define RX_FIFO_SIZE 32
#define RX_FIFO_SIZE_MASK 0x1f

static uint8_t rx_fifo[3][RX_FIFO_SIZE];
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

#if defined(LIBMSPUARTLINK_PIN_TX_PORT) && defined(LIBMSPUARTLINK_PIN_RX_PORT)
//TODO fix the EUSCI_A<x>_BASE thing..
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
#endif // LIBMSPUARTLINK_PIN_{TX && RX}_PORT

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

// Just like uartlinke_send but without the checksum and packetizing
void uartlink_send_basic(size_t port, uint8_t *payload, unsigned len)
{

    // Setup pointers for the ISR
    tx_data[port] = payload;
    tx_len[port] = len;
    switch (port) {
      case LIBMSPUARTLINK0_UART_IDX:
        UART(LIBMSPUARTLINK0_UART_IDX, IE) |= UCTXIE;
        UART(LIBMSPUARTLINK0_UART_IDX, TXBUF) = *tx_data[port]; // first byte, clears IFG

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

        LOG("uartlink: rcved: %02x\r\n", rx_byte);

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

#ifdef LIBMSPUARTLINK0_UART_IDX
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

            rx_fifo[0][rx_fifo_tail[0]] = UART(LIBMSPUARTLINK0_UART_IDX, RXBUF);
            rx_fifo_tail[0] = (rx_fifo_tail[0] + 1) & RX_FIFO_SIZE_MASK; //
            //wrap-around (assumes size is power of 2)

            if (rx_fifo_tail[0] == rx_fifo_head[0]) {
                // overflow, throw away the received byte, by rolling back NOTE:
                // tail == head happens both when full and empty, so can't use
                // that as overflow check
                rx_fifo_tail[0] = (rx_fifo_tail[0] - 1) & RX_FIFO_SIZE_MASK; //
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


