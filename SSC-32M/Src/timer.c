/*
 * timer.c
 *
 * Timer functions (and ISR) for the DeskPet servo controller.
 * Author : Mike Dvorsky
 */ 

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "../Include/globals.h"

/**********************************************************************
* Timer ISR to output edges.  This takes about 10us with an 8MHz clock.
* The edges must therefore be > 10us apart.
**********************************************************************/
ISR(TCA0_CMP0_vect)
{
	TCA0_SINGLE_INTFLAGS = TCA_SINGLE_CMP0_bm;		// Clear the flag
	EdgeDef_t *edge = &ServoPulseEdges[EdgeIndex];	// Pointer to the current edge
	++EdgeIndex;									// Increment index
	*edge->regAddr = edge->bitMap;					// Set pin high/low
	TCA0_SINGLE_CMP0 = edge->nextEdge;				// Ready for next edge
}

/**********************************************************************
* Initialize the timer:
* - Set the compare value to 0 for the first interrupt
* - Set the timer prescaler and period
* - Enable the timer interrupt
**********************************************************************/
void timer_init(void)
{
	// Set the TCA count to 1 so the first interrupt won't happen right away
	TCA0_SINGLE_CNT = 1;
	// Set Compare Channel 0 register to 0 for the first interrupt
	TCA0_SINGLE_CMP0 = 0;
	// Init TCA to 1MHz timebase, TOP=20000 (for a 20ms period)
	// The peripheral clock is the 8MHz main clock
	TCA0_SINGLE_CTRLA = TCA_SINGLE_CLKSEL_DIV8_gc | TCA_SINGLE_ENABLE_bm;
	TCA0_SINGLE_PER = 20000;
	TCA0_SINGLE_CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc;
	// Set the TCA0 CMPO interrupt to level 1 so it can interrupt USART interrupts
	CPUINT_LVL1VEC = TCA0_CMP0_vect_num;
	// Clear interrupt flag and enable interrupt on Compare Channel 0
	TCA0_SINGLE_INTFLAGS = TCA_SINGLE_CMP0_bm;
	TCA0_SINGLE_INTCTRL = TCA_SINGLE_CMP0_bm;
}