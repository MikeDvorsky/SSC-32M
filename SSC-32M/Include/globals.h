/*
 * globals.h
 *
 * Global variables and macros for the DeskPet servo controller.
 * Author : Mike Dvorsky
 */ 

/**********************************************************************
* DeskPet I/O
* - SERVO0 pulse = PA6
* - SERVO1 pulse = PA7
* - SERVO2 pulse = PB0
* - SERVO3 pulse = PB1
* - SERVO4 pulse = PB2
* - SERVO5 pulse = PB3
* - SERVO6 pulse = PB4
* - SERVO7 pulse = PB5
* - SERVO8 pulse = PC0
* - SERVO9 pulse = PC1
* - SERVO10 pulse = PC2
* - SERVO11 pulse = PC3
*
* - SERVO0 position = PD0
* - SERVO1 position = PD1
* - SERVO2 position = PD2
* - SERVO3 position = PD3
* - SERVO4 position = PD4
* - SERVO5 position = PD5
* - SERVO6 position = PD6
* - SERVO7 position = PD7
* - SERVO8 position = PE0
* - SERVO9 position = PE1
* - SERVO10 position = PE2
* - SERVO11 position = PE3
*
* - Servo Command RX = PA5 (UART0 Alternate)
* - Servo Command TX = PA4 (UART0 Alternate)
*
* - Servo Current RX = PC5 (UART1 Alternate)
* - Servo Current TX = PC4 (UART1 Alternate)
*
* - Battery Voltage Enable = PA2
* - Battery Voltage Input = PF2
**********************************************************************/

#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdint.h>
#include <stdbool.h>

// Version
#define VERSION (uint8_t *)"V0.1 ALPHA INTCLK\r"

// Flag indicating unit test
#define UNIT_TEST 0

// Number of servos supported
#define NUM_SERVOS 12

// Minimum and maximum pulse widths in microseconds
#define MINIMUM_PW 500
#define MAXIMUM_PW 2500

// Number of microseconds between rising edges in a group
#define RISING_EDGE_SPACING 20

// Period of servo pulses in milliseconds
#define SERVO_PULSE_PERIOD_MS 20

// Pin definition typedef
struct PinDef_s
{
	volatile uint8_t * outsetRegAddr;	// Pointer to the OUTSET register for a pin
	volatile uint8_t * outclrRegAddr;	// Pointer to the OUTCLR register for a pin
	volatile uint8_t * dirsetRegAddr;	// Pointer to the DIRSET register for a pin
	uint8_t bitMap;		// Bit map with '1' bit for the pin
};
typedef struct PinDef_s PinDef_t;

// Edge array typedef
struct EdgeDef_s
{
	volatile uint8_t * regAddr;	// Pointer to the OUTSET or OUTCLR register for the edge
	uint8_t bitMap;		// Bit map with '1' bit for the pin
	uint16_t nextEdge;	// Timer value for the next edge on the pin
};
typedef struct EdgeDef_s EdgeDef_t;

// Pulse array typedef
struct PulseDef_s
{
	uint16_t targetPW;			// The desired pulse width in timer ticks
	uint32_t currentPW_l16;		// The current pulse width, left shifted 16 bits
	int32_t deltaPW_l16;		// The delta pulse width, left shifted 16 bits
};
typedef struct PulseDef_s PulseDef_t;

// Servo Command typedef
struct ServoCmd_s
{
	bool isCommanded;			// TRUE if this servo is affected by the command
	uint16_t targetPW;			// The desired pulse width in microseconds
	uint16_t targetSpeed;		// The desired (max) move speed in microseconds/second
};
typedef struct ServoCmd_s ServoCmd_t;
typedef uint16_t ServoCmdMoveTime_t;		// The commanded (min) move time in milliseconds

// Pin definition array for servo output pins
extern const PinDef_t ServoPinDefs[NUM_SERVOS];

// Edge array for servo output pulses.  Defines the rising and falling edges for each pulse.
extern EdgeDef_t ServoPulseEdges[2 * NUM_SERVOS];
// Index of current edge in array
extern uint8_t EdgeIndex;

// Pulse array
extern PulseDef_t ServoPulseDefs[NUM_SERVOS];

// Command array, move time, and flag indicating command is waiting to be processed
extern ServoCmd_t ServoCmdArray[NUM_SERVOS];
extern ServoCmdMoveTime_t ServoCmdMoveTime;
extern bool ServoCmdWaiting;

// Counter of 20ms loops
extern uint64_t LoopCount;

// Number of milliseconds remaining in the current command
extern int32_t MillisRemainingInCommand;

#endif //GLOBALS_H
