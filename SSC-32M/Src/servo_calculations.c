/*
 * servo_calculations.c
 *
 * Calculate servo pulse generation data for the DeskPet servo controller.
 * Author : Mike Dvorsky
 */ 

#include <stdbool.h>

#include "../Include/globals.h"
#include "../Include/servo_calculations.h"

void servo_calculations_init(void)
{
	// Must be called after the ServoCommandArray and related globals are
	// initialized.  This call to the servo_calculations_update()
	// function will initialize the ServoPulseDefs to the stating
	// condition of all servos OFF (logic '0').
	servo_calculations_update();
}

/**********************************************************************
* If there is a command waiting to be processed, then process it.
* Inputs: ServoCmdArray, ServoCmdMoveTime, ServoCmdWaiting, ServoPulseDefs
* Output: ServoCmdWaiting, ServoPulseDefs
**********************************************************************/
void servo_calculations_update(void)
{
	// If there is no command waiting, then leave
	if (!ServoCmdWaiting)
	{
		return;
	}
	
	// Clear the flag
	ServoCmdWaiting = false;
	
	// Find the total move time.  This is based on the commanded total
	// and the move time calculated for each servo based on the speed
	// and the change in pulse width.  The final move time will be
	// the MAXIMUM of all the times specified in the command.
	uint16_t moveTime_ms = ServoCmdMoveTime;	// Start with commanded time in milliseconds
	for (uint8_t servoNum = 0; servoNum < NUM_SERVOS; ++servoNum)
	{
		uint32_t servoMoveTime;
		int16_t servoPwDelta;
		
		// If this servo is not part of the command, then skip
		if (!ServoCmdArray[servoNum].isCommanded)
			continue;
			
		// If this servo is commanded to '0' or '1', then skip.  These commands
		// always take effect immediately, so should not be used in time/speed
		// calculations.
		// If this servo is currently at '0' or '1', then skip.  Timed move
		// does not make sense when going from solid '0' or '1' to a pulse
		// width, so don't take this into account for move time calculation.
		//  '0' and '1' are indicated by pulse widths of 0 and 0xFFFF.
		if ((ServoCmdArray[servoNum].targetPW == 0) || (ServoCmdArray[servoNum].targetPW == 0xFFFF)
			|| (ServoPulseDefs[servoNum].targetPW == 0) || (ServoPulseDefs[servoNum].targetPW == 0xFFFF))
			continue;
		
		// Calculate the magnitude of the change in pulse width in microseconds
		servoPwDelta = ServoCmdArray[servoNum].targetPW - (uint16_t)(ServoPulseDefs[servoNum].currentPW_l16 >> 16);
		if (servoPwDelta < 0)
			servoPwDelta = -servoPwDelta;
		// Calculate the move time for this servo in milliseconds
		if (ServoCmdArray[servoNum].targetSpeed == 0)	// Prevent divide by 0
			ServoCmdArray[servoNum].targetSpeed = 1;
		servoMoveTime = (1000UL * servoPwDelta) / ServoCmdArray[servoNum].targetSpeed;
		if (servoMoveTime > 0xFFFF)		// Clip to 16 bits
			servoMoveTime = 0xFFFF;
		if (servoMoveTime > moveTime_ms)	// New maximum?
			moveTime_ms = (uint16_t)servoMoveTime;
	}
	
	// Now that we have the move time, we can recalculate the speeds and
	// store in the ServoPulseDefs array
	for (uint8_t servoNum = 0; servoNum < NUM_SERVOS; ++servoNum)
	{
		int32_t servoPwDelta_L16;

		// If this servo is not part of the command, then skip
		if (!ServoCmdArray[servoNum].isCommanded)
			continue;
		
		// If this servo is commanded to or is currently '0' or '1', then store in
		// ServoPulseDefs with no speed.
		if ((ServoCmdArray[servoNum].targetPW == 0) || (ServoCmdArray[servoNum].targetPW == 0xFFFF)
			|| (ServoPulseDefs[servoNum].targetPW == 0) || (ServoPulseDefs[servoNum].targetPW == 0xFFFF))
		{
			ServoPulseDefs[servoNum].targetPW = ServoCmdArray[servoNum].targetPW;
			ServoPulseDefs[servoNum].currentPW_l16 = (uint32_t)ServoCmdArray[servoNum].targetPW << 16;
			ServoPulseDefs[servoNum].deltaPW_l16 = 0;
			continue;
		}

		// Store the target PW
		ServoPulseDefs[servoNum].targetPW = ServoCmdArray[servoNum].targetPW;

		// Store the calculated deltaPW per loop, left shifted 16
		servoPwDelta_L16 = ((uint32_t)ServoPulseDefs[servoNum].targetPW << 16) - ServoPulseDefs[servoNum].currentPW_l16;
		ServoPulseDefs[servoNum].deltaPW_l16 = SERVO_PULSE_PERIOD_MS * (servoPwDelta_L16 / moveTime_ms);
		
		// Save the move time in this command in the global so we can track when it is done
		MillisRemainingInCommand = moveTime_ms;
	}
	
	// Prepare for the next command by initializing all of the globals that
	// are used for command storage.
	// Clear servo command array to not commanded, and speed = max.
	for (uint8_t i = 0; i < NUM_SERVOS; ++i)
	{
		ServoCmdArray[i].isCommanded = false;
		ServoCmdArray[i].targetSpeed = 65535;
		ServoCmdArray[i].targetPW = 0;
	}
	// Clear the flag indicating that a command is waiting.
	ServoCmdWaiting = false;
	// Set the move time to 0 (default) for the next command
	ServoCmdMoveTime = 0;

}
