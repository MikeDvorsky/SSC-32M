/*
 * serial_cmd.h
 *
 * Serial command processing for the DeskPet servo controller.
 * Author : Mike Dvorsky
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "../Include/globals.h"

/**********************************************************************
* Receives and transmits serial strings using the SSC-32 format.
*
* Serial port: UART0 (alternate on pins PA4/PA5).
* Baud rate: 115200
*
* The serial port is interrupt driven for both transmit and receive.
*
* Incoming bytes are stored in a buffer by the ISR.  The ISR priority
* is low, in order to allow pulse generation to interrupt the serial
* ISR.
*
* Transmitted bytes are pulled from a buffer by the ISR.
**********************************************************************/

// RX and TX buffer Sizes.  Currently the sizes must be 255 or less, because
// the add/remove indexes are only 8 bits.
#define RXQ_NBYTES 255
#define TXQ_NBYTES 255

// RX and TX buffers
char rx_queue[RXQ_NBYTES];
typedef uint8_t rxq_index_t;
rxq_index_t rxq_add_idx;
rxq_index_t rxq_remove_idx;

char tx_queue[TXQ_NBYTES];
typedef uint8_t txq_index_t;
txq_index_t txq_add_idx;
txq_index_t txq_remove_idx;

#if (UNIT_TEST)
// Function to stuff the passed string into the RX buffer for unit testing.
// The passed string must be an ASCIIZ string, and must have length less
// than the RX queue size.
void uart_rx_stuff(const char * cmd_string)
{
	rxq_remove_idx = 0;
	for (rxq_add_idx = 0; cmd_string[rxq_add_idx] != 0; ++rxq_add_idx)
	{
		rx_queue[rxq_add_idx] = cmd_string[rxq_add_idx];
	}
}
#endif	// UNIT_TEST

/**********************************************************************
* UART RX ISR.  Places received bytes in RX queue.  This ISR does not
* check if the queue is full.
**********************************************************************/
ISR(USART0_RXC_vect)
{
	rx_queue[rxq_add_idx] = USART0_RXDATAL;
	++rxq_add_idx;
	if (rxq_add_idx >= RXQ_NBYTES)
	{
		rxq_add_idx = 0;
	}
}

/**********************************************************************
* UART TX ISR.  If a byte is in the TX queue, transmits it.
* Otherwise, disables the interrupt.
**********************************************************************/
ISR(USART0_DRE_vect)
{
	if (txq_remove_idx == txq_add_idx)
	{
		// Queue empty, disable the interrupt.  No need for a read-
		// modify-write.  All interrupt enables are known value.
		USART0_CTRLA = USART_RXCIE_bm;	// RXC interrupt is always enabled
	}
	else
	{
		// Queue not empty, transmit the byte.  Do not check for
		// empty queue after transmit.  Instead, allow another
		// interrupt after this byte is transmitted.  This is
		// consistent with the expectation of the function to add
		// bytes to the TX queue.
		USART0_TXDATAL = tx_queue[txq_remove_idx];
		++txq_remove_idx;
		if (txq_remove_idx >= TXQ_NBYTES)
		{
			txq_remove_idx = 0;
		}
	}
}


void uart_init(void)
{
	// Select the alternate pins (PA4/PA5) for UART0
	PORTMUX_USARTROUTEA = (PORTMUX_USARTROUTEA & ~PORTMUX_USART0_gm) | PORTMUX_USART0_ALT1_gc;
	
	// Set the Baud rate to 115200
	USART0_BAUD = 277;
	
	// Define the port pin directions.
	PORTA_DIRSET = _BV(4);
	PORTA_DIRCLR = _BV(5);
	
	// Enable TX and RX, set modes, etc.
	// Asynchronous, normal speed, no parity, 1 stop bit, 8 data bits
	USART0_CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc | USART_CHSIZE_8BIT_gc;
	USART0_CTRLB = USART_RXEN_bm | USART_TXEN_bm | USART_RXMODE_NORMAL_gc;
	
	// Enable the RX ISR.  The TX ISR will be enabled only if there is
	// data to transmit.
	USART0_CTRLA = USART_RXCIE_bm;
}

void uart_update(void)
{
	
}

//*********************************************************************
// If there is a value in the RX queue, remove it and place it in the
// location passed.  If the queue is empty return FALSE, else return
// TRUE.
//
// Do not disable interrupts.  Global interrupt disable would cause
// jitter in the pulse outputs.  We can't just disable the RX interrupt
// since the enable bit can't be written atomically.  Ths TX interrupt
// enable is in the same register and we don't know its state, so a
// critical section would be needed for a read-modify-write, which
// would mean disabling global interrupts.  (Technically it would
// probably be possible to deduce the desired state of the TX interrupt
// enable by examining the TX queue add/remove indexes, but that is
// an undesirable interaction between the TX/RX queues and should be
// avoided if possible.)
//*********************************************************************
bool uart_rx_get_char(uint8_t * rxByte)
{
	// Return FALSE if queue is empty.  No need to disable interrupts.
	// Currently the add index is 1 byte so it can be read in an
	// atomic operation.  If the size is increased to 2 bytes, it will
	// still work because the ISR adds bytes to the queue.  We have 2
	// cases if the add index is incoherent due to an ISR:
	// 1) The add and remove indexes are different even though the
	//    add index is incoherent.  This is OK because the ISR just
	//    put a byte in the queue, so the add/remove indexes would
	//    be different even without the incoherence.
	// 2) The add and remove indexes are the same due to the
	//    incoherence, so the function returns FALSE even though
	//    the queue is not empty.  This is essentially a race
	//    condition in which it doesn't matter the result.  It
	//    will look like the queue is empty this call, but the
	//    function will be called again, and the next time it
	//    will return the received byte.
	// In both of these cases, no harm is done due to the
	// incoherence.
	if (rxq_remove_idx == rxq_add_idx)
	{
		return false;
	}

	// Get the character from the queue and store in the passed location.
	*rxByte = rx_queue[rxq_remove_idx] ;

	// Update the remove pointer.  Do not disable interrupts.  The
	// remove index is not read or modified in the ISR.
	++rxq_remove_idx;
	if (rxq_remove_idx == RXQ_NBYTES)
	{
		rxq_remove_idx = 0;
	}

	// Return TRUE, indicating a valid character
	return true;

}


//*********************************************************************
// Put the passed character on the TX queue for later transmission.
// Do not check for full queue.  TX messages are only in response to
// received messages, which imposes a throttle on the number of bytes
// transmitted.
//
// In the present implementation, there are no delays between TX
// bytes.  The original SSC-32 inserted delays because it had to
// support slow microprocessors that lacked buffers and were prone
// to dropping bytes if they came too fast.  This is no longer needed.
//
// Disable the TX interrupt while modifying the add index.  Do NOT
// disable global interrupts, as this would cause jitter in the pulse
// outputs.
//*********************************************************************
void uart_tx_put_char(uint8_t txByte)
{
	// Disable the TX interrupt while adding to the queue
	USART0_CTRLA = USART_RXCIE_bm;	// RXC interrupt is always enabled
	
	// Store the character on the queue.
	tx_queue[txq_add_idx] = txByte;
	
	// Update the TX add index.
	++txq_add_idx;
	if (txq_add_idx >= TXQ_NBYTES)
	{
		txq_add_idx = 0;
	}
	
	// Enable the TX interrupt.
	USART0_CTRLA = USART_RXCIE_bm | USART_DREIE_bm;	// RXC interrupt is always enabled
}

//*********************************************************************
// Transmit an ASCIIZ string.  Max length = 20 bytes (arbitrary).
//*********************************************************************
void uart_tx_string(uint8_t * s)
{
	for (uint8_t i = 0; i < 20; ++i)
	{
		if (s[i] == 0)
		{
			// End of ASCIIZ string, leave without sending
			break;
		}
		uart_tx_put_char(s[i]);
	}
}

//*********************************************************************
// Transmit an unsigned integer.
//*********************************************************************
void uart_tx_uint16(uint16_t num)
{
	uint8_t digits[6];
	
	digits[5] = 0;	// ASCIIZ terminator
	int8_t i = 4;
	do 
	{
		uint8_t digit;
		digit = num % 10;
		digits[i] = digit + '0';
		num /= 10;
		--i;
	} while (num != 0);

	// Transmit the string of digits.  Variable 'i' is 1 less than the
	// index of the first character
	uart_tx_string(&digits[i+1]);
}