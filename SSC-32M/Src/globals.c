/*
 * globals.c
 *
 * Global variables for the DeskPet servo controller.
 * Author : Mike Dvorsky
 */ 

#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>

#include "../Include/globals.h"


/**********************************************************************
* Pin definition array for servo output pins.  Each entry contains 
* pointers to the registers to set the pin, clear the pin, and
* set the pin to output; and the bitmask that needs to be written
* to the registers.  The array is indexed by servo number 0-11.
**********************************************************************/
const PinDef_t ServoPinDefs[NUM_SERVOS] =
{
	{&PORTA_OUTSET,&PORTA_OUTCLR,&PORTA_DIRSET,_BV(6)},	// Servo0 = PA6
	{&PORTA_OUTSET,&PORTA_OUTCLR,&PORTA_DIRSET,_BV(7)},	// Servo1 = PA7
	{&PORTB_OUTSET,&PORTB_OUTCLR,&PORTB_DIRSET,_BV(0)},	// Servo2 = PB0
	{&PORTB_OUTSET,&PORTB_OUTCLR,&PORTB_DIRSET,_BV(1)},	// Servo3 = PB1
	{&PORTB_OUTSET,&PORTB_OUTCLR,&PORTB_DIRSET,_BV(2)},	// Servo4 = PB2
	{&PORTB_OUTSET,&PORTB_OUTCLR,&PORTB_DIRSET,_BV(3)},	// Servo5 = PB3
	{&PORTB_OUTSET,&PORTB_OUTCLR,&PORTB_DIRSET,_BV(4)},	// Servo6 = PB4
	{&PORTB_OUTSET,&PORTB_OUTCLR,&PORTB_DIRSET,_BV(5)},	// Servo7 = PB5
	{&PORTC_OUTSET,&PORTC_OUTCLR,&PORTC_DIRSET,_BV(0)},	// Servo8 = PC0
	{&PORTC_OUTSET,&PORTC_OUTCLR,&PORTC_DIRSET,_BV(1)},	// Servo9 = PC1
	{&PORTC_OUTSET,&PORTC_OUTCLR,&PORTC_DIRSET,_BV(2)},	// Servo10 = PC2
	{&PORTC_OUTSET,&PORTC_OUTCLR,&PORTC_DIRSET,_BV(3)},	// Servo11 = PC3
};

/**********************************************************************
* Edge array for servo output pulses.  Defines the rising and falling
* edges for each pulse.
*
* Also define a global for the current index into the array.
**********************************************************************/
EdgeDef_t ServoPulseEdges[2 * NUM_SERVOS];
uint8_t EdgeIndex;

/**********************************************************************
* Pulse array for servo output pulses.  Defines the current and
* target pulse widths, and the delta value used for servo movement.
*
* Any targetPW less than the MINIMUM_PW is considered to indicate
* a servo output that should be logic '0'.
*
* Any targetPW greater than the MAXIMUM_PW is considered to indicate
* a servo output that should be logic '1'.
**********************************************************************/
PulseDef_t ServoPulseDefs[NUM_SERVOS];

/**********************************************************************
* Array of commanded servo moves, indexed by servo number.  Each
* contains a flag indicating whether the servo is part of the
* command, the commanded pulse width, and the commanded
* maximum speed.
**********************************************************************/
ServoCmd_t ServoCmdArray[NUM_SERVOS];
// The commanded (maximum) move time in milliseconds.
ServoCmdMoveTime_t ServoCmdMoveTime;
// A flag indicating whether there is a command waiting to be
// processed.
bool ServoCmdWaiting;


/**********************************************************************
* Counter of 20ms loops.  Incremented after all edges for a loop 
* are output.
**********************************************************************/
uint64_t LoopCount;


/**********************************************************************
* Number of milliseconds remaining in the latest command.
**********************************************************************/
int32_t MillisRemainingInCommand;