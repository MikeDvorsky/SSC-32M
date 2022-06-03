/*
 * main.c
 *
 * Main C file for the DeskPet servo controller.
 * Author : Mike Dvorsky
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "../Include/globals.h"
#include "../Include/timer.h"
#include "../Include/uart.h"
#include "../Include/parse_commands.h"
#include "../Include/servo_pulse.h"
#include "../Include/servo_calculations.h"
#include "../Include/adc.h"
#include "../Include/servo_current.h"
#include "../Include/unit_test.h"


int main(void)
{
	/******************************************************************
	* Initializations
	******************************************************************/
	// Init the clock to EXTCLK, no prescaler
	//	_PROTECTED_WRITE(CLKCTRL_MCLKCTRLA, CLKCTRL_CLKSEL_EXTCLK_gc);
	//	_PROTECTED_WRITE(CLKCTRL_MCLKCTRLB, 0);
	// For testing, use internal clock set to 8MHz (note that the FREQSEL fuse must be set to 16MHz)
	_PROTECTED_WRITE(CLKCTRL_MCLKCTRLA, CLKCTRL_CLKSEL_OSC20M_gc);
	_PROTECTED_WRITE(CLKCTRL_MCLKCTRLB, CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm);
	
	// Init libraries
	uart_init();
	adc_init();
	// The following init functions must be called in the correct order
	// so that the servo outputs will be initialized to the correct
	// states (OFF = '0').
	parse_commands_init();		// (1) Init command array to all OFF
	servo_calculations_init();	// (2) Convert command array into pulse width array
	servo_pulse_init();			// (3) Convert pulse width array into edge array for the ISR

	// Do timer inits last so everything will be set up before first ISR
	timer_init();
	
	// Turn on global interrupts as last step in inits
	sei();
	
	// Turn on battery voltage monitor on pin PA2
	PORTA_DIRSET = _BV(2);
	PORTA_OUTTGL = _BV(2);

	
	/******************************************************************
	* Periodic updates
	******************************************************************/
    while (1) 
    {
		// Call unit test function if defined
		#if (UNIT_TEST)
		unit_test_driver();
		#endif
		// Call periodic update functions
		uart_update();
		adc_update();
		parse_commands_update();
		servo_calculations_update();
		servo_pulse_update();
    }
}
