#include <msp430.h>

#include <libio/log.h>
#include <libmsp/periph.h>

#include "uartlink.h"

typedef enum {
    DECODER_STATE_HEADER  = 0x0,
    DECODER_STATE_PAYLOAD,
} decoder_state_t;

#define RX_FIFO_SIZE 32
#define RX_FIFO_SIZE_MASK 0x1f

static uint8_t rx_fifo[RX_FIFO_SIZE];
static unsigned rx_fifo_head = 0;
static unsigned rx_fifo_tail = 0;

static decoder_state_t decoder_state = DECODER_STATE_HEADER;
static unsigned rx_payload_len = 0;
static ul_header_t rx_header;

static void uartlink_configure()
{
    UART(LIBMSPUARTLINK_UART_IDX, CTL1) |= UCSWRST; // put state machine in reset
    UART(LIBMSPUARTLINK_UART_IDX, CTL1) |= UCSSEL__SMCLK;

    UART(LIBMSPUARTLINK_UART_IDX, BR0) = UART_BR0_LIBMSPUARTLINK_BAUDRATE;
    UART(LIBMSPUARTLINK_UART_IDX, BR1) = UART_BR1_LIBMSPUARTLINK_BAUDRATE;
    UART(LIBMSPUARTLINK_UART_IDX, MCTL) = 0;

    UART(LIBMSPUARTLINK_UART_IDX, CTL1) &= ~UCSWRST; // turn on
}

#ifdef LIBMSPUARTLINK_PIN_RX
void uartlink_open_rx()
{
    GPIO(LIBMSPUARTLINK_PIN_RX_PORT, SEL) |= BIT(LIBMSPUARTLINK_PIN_RX_PIN);
    uartlink_configure();
    UART(LIBMSPUARTLINK_UART_IDX, IE) |= UCRXIE;
}
#endif // LIBMSPUARTLINK_PIN_RX

#ifdef LIBMSPUARTLINK_PIN_TX
void uartlink_open_tx()
{
    GPIO(LIBMSPUARTLINK_PIN_TX_PORT, SEL) |= BIT(LIBMSPUARTLINK_PIN_TX_PIN);
    uartlink_configure();
    UART(LIBMSPUARTLINK_UART_IDX, IE) |= UCTXIE;
}
#endif // LIBMSPUARTLINK_PIN_TX

#if defined(LIBMSPUARTLINK_PIN_TX) && defined(LIBMSPUARTLINK_PIN_RX)
// bidirectional (not implemenented)
void uartlink_open()
{
    GPIO(LIBMSPUARTLINK_PIN_RX_PORT, SEL) |= BIT(LIBMSPUARTLINK_PIN_RX_PIN) | BIT(LIBMSPUARTLINK_PIN_TX_PIN);
    uartlink_configure();
    UART(LIBMSPUARTLINK_UART_IDX, IE) |= UCRXIE | UCTXIE;
}
#endif // LIBMSPUARTLINK_PIN_TX && RX

void uartlink_close()
{
    UART(LIBMSPUARTLINK_UART_IDX, CTL1) |= UCSWRST;
}

static inline void uartlink_disable_interrupt()
{
    UART(LIBMSPUARTLINK_UART_IDX, IE) &= ~UCRXIE;
}

static inline void uartlink_enable_interrupt()
{
    UART(LIBMSPUARTLINK_UART_IDX, IE) |= UCRXIE;
}

#ifdef LIBMSPUARTLINK_PIN_TX
static inline void send_byte(uint8_t b)
{
    // classic lock-check-sleep pattern
    __disable_interrupt();
    UART(LIBMSPUARTLINK_UART_IDX, TXBUF) = b;
    while (!(UART(LIBMSPUARTLINK_UART_IDX, IFG) & UCTXIFG)) {
        __bis_SR_register(LPM0_bits + GIE); // will wakeup on TX int
        __disable_interrupt();
    }
    __enable_interrupt();
}

void uartlink_send(uint8_t *payload, unsigned len)
{
    // Payload checksum
    CRCINIRES = 0xFFFF; // initialize checksumer
    for (int i = 0; i < len; ++i) {
        CRCDI = *(payload + i);
    }
    uint8_t pay_chksum = CRCINIRES & UARTLINK_PAYLOAD_CHKSUM_MASK;

    ul_header_ut header = { .typed = { .size = len,
                                       .pay_chksum = pay_chksum,
                                       .hdr_chksum = 0 /* to be filled in shortly */ } }

    CRCINIRES = 0xFFFF; // initialize checksumer
    CRCDI = header.raw;
    header.hdr_chksum = CRCINIRES & UARTLINK_HDR_CHKSUM_MASK;

    send_byte(header.raw);
    for (int i = 0; i < len; ++i)
        send_byte(*(payload + i));
}
#endif LIBMSPUARTLINK_PIN_TX

// Should be called whenever MCU wakes up, from the context of a main loop
// precondition: payload points to a buffer of at least UARTLINK_MAX_PAYLOAD_SIZE
unsigned uartlink_receive(uint8_t *payload)
{
    uartlink_disable_interrupt();
    while (rx_fifo_head != rx_fifo_tail) {
        uint8_t rx_byte = rx_fifo[rx_fifo_head++];
        rx_fifo_head &= RX_FIFO_SIZE_MASK; // wrap around
        uartlink_enable_interrupt();

        LOG("uartlink: rcved: %02x\r\n", rx_byte);

        // Attempt to interpret the incoming byte as a header,
        // even if we are parsing payload. Sender ensures that
        // header cannot appear inside a payload byte, using
        // an escaping scheme (TODO).
        ul_header_ut header = { .raw = rx_byte };

        CRCINIRES = 0xFFFF; // initialize checksum'er for header
        CRCDI = header.raw;
        if ((CRCINIRES & UARTLINK_HDR_CHKSUM_MASK) == header.typed.hdr_chksum) {

            rx_header = header.typed;
            rx_payload_len = 0; // init packet
            CRCINIRES = 0xFFFF; // initialize checksum'er for payload
            decoder_state = DECODER_STATE_PAYLOAD;
        }

        switch (decoder_state) {
            case DECODER_STATE_HEADER:
                // the byte was not a valid header, and we weren't parsing payload
                LOG("uartlink: rxed byted not a valid header: %02x\r\n", rx_byte);
                break;
            case DECODER_STATE_PAYLOAD:
                // assert: pkt.header.size < UARTLINK_MAX_PAYLOAD_SIZE
                payload[rx_payload_len++] = rx_byte;
                CRCDI = rx_byte;
                if (rx_payload_len == UARTLINK_MAX_PAYLOAD_SIZE) {
                    // check payload checksum
                    uint8_t rx_payload_chksum = CRCINIRES & UARTLINK_PAYLOAD_CHKSUM_MASK;
                    if (rx_payload_chksum != rx_header.pay_chksum) {
                        LOG("uartlink: payload chksum mismatch (0x%02x != 0x%02x)\r\n",
                            rx_payload_chksum, rx_header.pay_chksum);
                        // reset decoder
                        rx_payload_len = 0;
                        decoder_state = DECODER_STATE_HEADER;
                    }
                }
                break;
        }

        uartlink_disable_interrupt(); // classic: check-and-...-sleep pattern
    }
    uartlink_enable_interrupt();
    return rx_payload_len;
}

__attribute__ ((interrupt(UART_VECTOR(LIBMSPUARTLINK_UART_IDX))))
void UART_ISR(LIBMSPUARTLINK_UART_IDX) (void)
{
    switch(__even_in_range(UART(LIBMSPUARTLINK_UART_IDX, IV),USCI_UCTXIFG)) {
        case USCI_UCTXIFG:
            break; // nothing to do, main thread is sleeping, so just wakeup
        case USCI_UCRXIFG:                      // Vector 2 - RXIFG
        {
            P3OUT |= BIT2;

            rx_fifo[rx_fifo_tail] = UART(LIBMSPUARTLINK_UART_IDX, RXBUF);
            rx_fifo_tail = (rx_fifo_tail + 1) & RX_FIFO_SIZE_MASK; // wrap-around (assumes size is power of 2)

            if (rx_fifo_tail == rx_fifo_head) {
                // overflow, throw away the received byte, by rolling back
                // NOTE: tail == head happens both when full and empty, so can't use that as overflow check
                rx_fifo_tail = (rx_fifo_tail - 1) & RX_FIFO_SIZE_MASK; // wrap-around (assumes size is power of 2)
            }

            P3OUT &= ~BIT2;
            break;
        }
        default:
            break;
    }
    __bic_SR_register_on_exit(LPM4_bits); // wakeup
}
