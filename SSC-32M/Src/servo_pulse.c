/*
 * servo_pulse.c
 *
 * Servo pulse generation for the DeskPet servo controller.
 * Author : Mike Dvorsky
 */ 

#include <stdint.h>
#include <avr/io.h>

#include "../Include/globals.h"
#include "../Include/servo_pulse.h"

// Array of pulse widths for each servo
struct pulseWidth_s
{
	uint8_t servoNum;
	uint16_t pw;
};
typedef struct pulseWidth_s pulseWidth_t;
static pulseWidth_t pulseWidths[3];	// Hard coded for 3 servos per group

/**********************************************************************
* Initialize the servo output pins and the array of servo edges.
**********************************************************************/
void servo_pulse_init(void)
{
	// Init servo pulse output pins
	for (uint8_t servoNum = 0; servoNum < NUM_SERVOS; ++servoNum)
	{
		*ServoPinDefs[servoNum].outclrRegAddr = ServoPinDefs[servoNum].bitMap;	// Set the pin value to ''0'
		*ServoPinDefs[servoNum].dirsetRegAddr = ServoPinDefs[servoNum].bitMap;	// Configure the pin as output
	}

	// Init the ServoPulseEdges[] array.  This must be called after
	// the ServoPulseDefs[] array has been initialized to the
	// starting values.
	EdgeIndex = 2 * NUM_SERVOS;
	servo_pulse_update();
}

/**********************************************************************
* Update the pulse and edge arrays.
* NOTE: this function is hard coded for 12 servos total (4 groups of 3 servos
* per group).  If the number of servos changes, then the function must be changed.
* Inputs: EdgeIndex, ServoPulseDefs, ServoPinDefs
* Outputs: EdgeIndex, ServoPulseEdges
**********************************************************************/
void servo_pulse_update(void)
{
	// Test for all edges output.
	// When this happens, update the servo pulses and edges for next time.
	if ( EdgeIndex < (2 * NUM_SERVOS))
	{
		// Return if still outputting edges.
		return;
	}
	
	// Clear the edge index for next time
	EdgeIndex = 0;

	// Increment the loop counter every 20ms (i.e. every time all of
	// the edges have been output for a loop)
	++LoopCount;
	
	// Update the time remaining in the latest command
	MillisRemainingInCommand -= SERVO_PULSE_PERIOD_MS;
	if (MillisRemainingInCommand < 0)
	{
		MillisRemainingInCommand = 0;
	}

	// Update the global pulse array by adding the delta to each pulse
	// width, then clipping to the target.  Work one group at a time.
	for (uint8_t groupNum = 0; groupNum < 4; ++groupNum)
	{
		// Loop through the servos in the group, updating the pulse widths
		for (uint8_t offsetInGroup = 0; offsetInGroup < 3; ++offsetInGroup)
		{
			uint8_t servoNum = (groupNum * 3) + offsetInGroup;
			// Get a pointer to the pulse
			PulseDef_t *pulseDef = &ServoPulseDefs[servoNum];
			// Add the delta to the current PW.  If overshoot, then clip.
			pulseDef->currentPW_l16 += pulseDef->deltaPW_l16;
			if (((pulseDef->deltaPW_l16 > 0) && (pulseDef->currentPW_l16 > ((uint32_t)(pulseDef->targetPW) << 16)))
			|| ((pulseDef->deltaPW_l16 < 0) && (pulseDef->currentPW_l16 < ((uint32_t)(pulseDef->targetPW) << 16))))
			{
				pulseDef->currentPW_l16 = (uint32_t)(pulseDef->targetPW) << 16;
			}
			// Store the resulting pulse width in the pulseWidths array for later use.
			pulseWidths[offsetInGroup].servoNum = servoNum;
			pulseWidths[offsetInGroup].pw = pulseDef->currentPW_l16 >> 16;
			// If the pulse width is outside the range, then force it to 1 beyond the range.
			// This ensures that all such pulse widths sort correctly in the next step.
			if (pulseWidths[offsetInGroup].pw < MINIMUM_PW)
			{
				pulseWidths[offsetInGroup].pw = MINIMUM_PW - 1;
			}
			else if (pulseWidths[offsetInGroup].pw > MAXIMUM_PW)
			{
				pulseWidths[offsetInGroup].pw = MAXIMUM_PW + 1;
			}
		}
			
		// At this point, the pulseWidths array has been written with the servo
		// number and pulse width for each servo in the group.  Sort the
		// pulseWidths array by pulse width, from smallest to largest.
		// Insertion sort
		for (uint8_t i = 0; i < 2; ++i)	// Hard coded for 3 pulse per group
		{
			uint16_t minPW = pulseWidths[i].pw;
			uint8_t minIdx = i;
			for (uint8_t j = i+1; j < 3; ++j)
			{
				if (pulseWidths[j].pw < minPW)
				{
					minPW = pulseWidths[j].pw;
					minIdx = j;
				}
			}
			if (minIdx != i)
			{
				// Insert
				pulseWidth_t temp = pulseWidths[minIdx];
				for (uint8_t k = minIdx; k > i; --k)
				{
					pulseWidths[k] = pulseWidths[k-1];
				}
				pulseWidths[i] = temp;
			}
		}
		
		// Now that the pulseWidths array has the pulses in the group sorted,
		// write the edge array with the appropriate values.
		uint16_t risingEdgeTime = groupNum * 3000; // Each group starts 3000 us apart
		uint8_t risingEdgeNum = groupNum * 6;	// 6 edges per group (3 rising, 3 falling)
		for (uint8_t offsetInGroup = 0;	offsetInGroup < 3;	++offsetInGroup)
		{
			uint8_t servoNum = pulseWidths[offsetInGroup].servoNum;
			uint16_t pw = pulseWidths[offsetInGroup].pw;
			uint8_t bitMap = ServoPinDefs[servoNum].bitMap;

			// Store rising edge values.  Each rising edge is offset RISING_EDGE_SPACING from the previous.
			// For now, store the edge time for this edge.  It will adjusted later.
			ServoPulseEdges[risingEdgeNum].nextEdge = risingEdgeTime + (offsetInGroup * RISING_EDGE_SPACING);
			ServoPulseEdges[risingEdgeNum].bitMap = bitMap;
			// Set rising edge register address
			if (pw < MINIMUM_PW)
			{
				// Pulse width less than minimum indicates pin should stay at logic '0', so no rising edge.
				ServoPulseEdges[risingEdgeNum].regAddr = ServoPinDefs[servoNum].outclrRegAddr;
			}
			else
			{
				// Pulse width greater than minimum indicates rising edge (or constant '1') needed.
				ServoPulseEdges[risingEdgeNum].regAddr = ServoPinDefs[servoNum].outsetRegAddr;
			}

			// Store falling edge values.  The falling edge time is just PW later than the rising edge.
			uint8_t fallingEdgeNum = risingEdgeNum + 3;
			uint16_t fallingEdgeTime = ServoPulseEdges[risingEdgeNum].nextEdge + pw;
			// For now, store the edge time for this edge.  It will adjusted later.
			ServoPulseEdges[fallingEdgeNum].nextEdge = fallingEdgeTime;
			ServoPulseEdges[fallingEdgeNum].bitMap = bitMap;
			// Set falling edge register address
			if (pw > MAXIMUM_PW)
			{
				// Pulse width greater than maximum indicates pin should stay at logic '1', so no falling edge.
				ServoPulseEdges[fallingEdgeNum].regAddr = ServoPinDefs[servoNum].outsetRegAddr;
			}
			else
			{
				// Pulse width greater than minimum indicates falling edge (or constant '0') needed.
				ServoPulseEdges[fallingEdgeNum].regAddr = ServoPinDefs[servoNum].outclrRegAddr;
			}
			
			// Increment risingEdgeNum for next loop
			++risingEdgeNum;
		}
	
		// At this point, the "nextEdge" entry for ServoPulseEdges is set to the current edge.  Loop
		// through and adjust this.  Only loop through the first 5 edges.  The last one will be
		// set separately to the start of the next group.
		uint8_t edgeNum = groupNum * 6;
		for (uint8_t i = 0; i < 5; ++i)
		{
			// Set each nextEdge to the edge time for the next edge
			ServoPulseEdges[edgeNum + i].nextEdge = ServoPulseEdges[edgeNum + i + 1].nextEdge;
		}
		// Last entry should be the first edge of the next group
		if (risingEdgeTime < 9000)
		{
			// Last edge of group, next edge is the first edge of the next group
			ServoPulseEdges[edgeNum + 5].nextEdge = risingEdgeTime + 3000;
		}
		else
		{
			// Last edge of last group, next edge is at time 0 to start next cycle
			ServoPulseEdges[edgeNum + 5].nextEdge = 0;
		}
	}
}
