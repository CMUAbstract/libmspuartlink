// File that has all of the packet handling procedures
#include <msp430.h>

#include <libio/console.h>
#include <libmsp/periph.h>
#include <libmsp/mem.h>
#include <libmspware/driverlib.h>
#include <libartibeus/comm.h>
#include <libartibeus/handle_uarts.h>

#include "uartlink.h"
#ifndef BIT_FLIP
#define BIT_FLIP(port,bit) \
	P##port##OUT |= BIT##bit; \
	P##port##DIR |= BIT##bit; \
	P##port##OUT &= ~BIT##bit;
#endif


//COMM UART most of the time
// Needs to look for transfer_active and transfer_done messages.
#if defined(LIBMSPUARTLINK0_UART_IDX) && !defined(CONSOLE) && !defined(LIBMSPUARTLINK_NO_PROCESS)
//TODO check that changing these to volatile works... We want this whole thing
//idempotent
static uint8_t prog_keys[16] = {0};

__attribute__ ((interrupt(UART_VECTOR(LIBMSPUARTLINK0_UART_IDX))))
void UART_ISR(LIBMSPUARTLINK0_UART_IDX) (void)
{
    BIT_FLIP(1,1);
    BIT_FLIP(1,2);
    BIT_FLIP(1,2);
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
            BIT_FLIP(1,2);
            uint8_t data = UART(LIBMSPUARTLINK0_UART_IDX, RXBUF);
            handle_progress_uart0(data);
            __bic_SR_register_on_exit(LPM4_bits); // wakeup
          break;
        }
        default:
            break;
    }
}
#elif defined(LIBMSPUARTLINK_NO_PROCESS) && defined(LIBMSPUARTLINK0_UART_IDX) \
&& !defined(CONSOLE)
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

          __bic_SR_register_on_exit(LPM4_bits); // wakeup
          break;
        }
        default:
            break;
    }
}
#endif

//EXPT UART most of the time
#if defined(LIBMSPUARTLINK1_UART_IDX) && !defined(LIBMSPUARTLINK_NO_PROCESS)
__attribute__ ((interrupt(UART_VECTOR(LIBMSPUARTLINK1_UART_IDX))))
void UART_ISR(LIBMSPUARTLINK1_UART_IDX) (void)
{
    BIT_FLIP(1,2);
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
            uint8_t data = UART(LIBMSPUARTLINK1_UART_IDX, RXBUF);
            handle_progress_uart1(data);

            __bic_SR_register_on_exit(LPM4_bits); // wakeup
            break;
        }
        default:
            break;
    }
}
#elif defined(LIBMSPUARTLINK1_UART_IDX)
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
    BIT_FLIP(1,2);
    BIT_FLIP(1,1);
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

