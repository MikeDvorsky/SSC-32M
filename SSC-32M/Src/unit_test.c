/*
 * unit_test.c
 *
 * Unit test cases for DeskPet project.
 * Author : Mike Dvorsky
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "../Include/globals.h"
#include "../Include/timer.h"
#include "../Include/uart.h"
#include "../Include/servo_pulse.h"
#include "../Include/servo_calculations.h"
#include "../Include/adc.h"
#include "../Include/servo_current.h"
#include "../Include/unit_test.h"

#if (UNIT_TEST)

void unit_test_verify(uint8_t servoNum, uint16_t pw, bool shouldMatch)
{
	bool match;
	static uint16_t errorCount = 0;
	
	match = ((ServoPulseDefs[servoNum].currentPW_l16 >> 16) == pw);
	if (match != shouldMatch)
	{
		++errorCount;
	}
}

void unit_test_driver(void)
{
	static uint64_t prevLoopCount = 0;
	
	// Only do this once per loop
	if (LoopCount == prevLoopCount)
	{
		return;
	}
	prevLoopCount = LoopCount;
	
	// Flash the LED (PF5) once per second
	if ((LoopCount % 25) == 0)
	{
		PORTF_DIRSET = _BV(5);
		PORTF_OUTTGL = _BV(5);
	}
	
	// Perform unit test operations
	switch (LoopCount)
	{
		case 10:
			// Send the version number
			uart_rx_stuff("VER\r");
			break;
		case 50:
			// Turn on servos 0 (1500) and 2 (2000) immediately
			uart_rx_stuff("#0P1500 #2 P2000 S1000 T1000 \r");
			break;
		case 51:
			// Verify
			unit_test_verify(0, 1500, true);
			unit_test_verify(2, 2000, true);
			break;
		case 100:
			// Turn on servo 5 (500) immediately; servos 0 (2000) and 2 (2100) over 1 second
			uart_rx_stuff(" #0P2000 #5P500 #2 P2100 T1000 \r");
			break;
		case 101:
			// Verify
			unit_test_verify(5, 500, true);
			break;
		case 148:
			// Verify
			unit_test_verify(0, 2000, false);
			unit_test_verify(2, 2100, false);
			break;
		case 152:
			// Verify
			unit_test_verify(0, 2000, true);
			unit_test_verify(2, 2100, true);
			break;
		case 200:
			// Set servos 0 (2100), 2 (1000), and 5 (1500) over 2 seconds
			uart_rx_stuff(" #0P2100 #5P1500S500 #2 P1000 \r");
			break;
		case 298:
			// Verify
			unit_test_verify(0, 2100, false);
			unit_test_verify(2, 1000, false);
			unit_test_verify(5, 1500, false);
			break;
		case 302:
			// Verify
			unit_test_verify(0, 2100, true);
			unit_test_verify(2, 1000, true);
			unit_test_verify(5, 1500, true);
			break;
	}
}
#endif	// UNIT_TEST
