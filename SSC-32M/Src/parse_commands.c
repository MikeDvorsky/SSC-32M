/*
 * process_commands.h
 *
 * Command processing for the DeskPet servo controller.
 * Author : Mike Dvorsky
 */ 

#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>

#include "../Include/globals.h"
#include "../Include/uart.h"
#include "../Include/adc.h"

// Maximum token length.  Must be long enough to hold the longest
// command, as well as the longest argument (65535).
#define MAX_TOKEN_NBYTES 6
	
// Character type defines
#define CHAR_TYPE_WHITESPACE	0
#define CHAR_TYPE_DIGIT			1
#define CHAR_TYPE_ALPHAPUNC		2

// Prototpyes for the individual parsing functions
static void parseAlpha(uint8_t * token);
static void parseNumber(uint8_t * token);

static void ParseServoNum(uint16_t argument);
static void ParseServoHold(uint16_t argument);
static void ParseServoLimp(uint16_t argument);
static void ParseServoPW(uint16_t argument);
static void ParseQCurrent(uint16_t argument);
static void ParseQPos(uint16_t argument);
static void ParseQStatus(uint16_t argument);
static void ParseQVoltage(uint16_t argument);
static void ParseServoSpeed(uint16_t argument);
static void ParseMoveTime(uint16_t argument);
static void ParseVer(uint16_t argument);

// Structure for command parsing
struct ParseTable_s
{
	char * pCmdstr ;
	void (* pFunction) (uint16_t) ;
	bool argumentRequired;
};
typedef struct ParseTable_s ParseTable_t;


// Command parse table
static const ParseTable_t ParseTable[] =
{
	{"#", ParseServoNum, true},		// Set servo number
	{"H", ParseServoHold, false},	// Hold servo position
	{"L", ParseServoLimp, false},	// Turn off pulses for a servo, i.e. set output to logic '0'
	{"P", ParseServoPW, true},		// Set the Pulse Width in microseconds
	{"Q", ParseQStatus, false},		// Return servo status as an integer 0-10
	{"QC", ParseQCurrent, false},	// Returns servo current in milliamps
	{"QP", ParseQPos, false},		// Returns feedback voltage in millivolts
	{"QV", ParseQVoltage, false},	// Returns battery voltage in millivolts
	{"S", ParseServoSpeed, true},	// Set servo speed in us/sec
	{"T", ParseMoveTime, true},		// Set total move time in ms
	{"VER", ParseVer, false},		// Return firmware version
	{NULL, NULL},	// Sentinel must be last entry in table
};

// Pointer to function for parsed command
static void (* pCmdFunc)(uint16_t);
// Flag indicating whether the current function requires an argument
static bool argumentRequired;
// Servo number specified with '#'
static uint8_t servoNum = 255;	// Default to invalid servo number

/**********************************************************************
* Initialize the ServoCmdArray and related global data.
**********************************************************************/
void parse_commands_init(void)
{
	// Reset the command array to turn all servos off (output '0').
	// Set all servos to PW = 0 (servo OFF).
	for (uint8_t servoNum = 0; servoNum < NUM_SERVOS; ++servoNum)
	{
		ServoCmdArray[servoNum].isCommanded = true;
		ServoCmdArray[servoNum].targetPW = 0;
		ServoCmdArray[servoNum].targetSpeed = 0;
	}
	ServoCmdMoveTime = 0;
	ServoCmdWaiting = true;		// Trigger calculations
}


/**********************************************************************
* Parses command strings using the SSC-32 format.
* Input: RX bytes from UART
* Outputs: TX bytes to UART, ServoCmdArray[], ServoCmdMoveTime, ServoCmdWaiting
**********************************************************************/
void parse_commands_update(void)
{
	static uint8_t prevCharType = CHAR_TYPE_WHITESPACE;
	static uint8_t tokenIdx = 0;	// Index of next character to be added to the token
	static uint8_t token[MAX_TOKEN_NBYTES + 1];	// 1 extra for ASCIIZ zero byte
	uint8_t charType;	// Space, digit, or alpha/punctuation

	// Loop until no characters in the RX queue or a complete command
	// is parsed.  Read bytes from the serial port and collect into a "token".
	// When a token is complete, then look it up and call the handler.
	while (!ServoCmdWaiting)
	{
		uint8_t ch;
		bool charReturned = uart_rx_get_char(&ch);
		if (!charReturned) break;	// Nothing in queue? Quit loop
		
		// At this point, we have a character from the queue.
		// Determine which type of character it is.
		if (isspace(ch))
		{
			charType = CHAR_TYPE_WHITESPACE;
		}
		else if (isdigit(ch))
		{
			charType = CHAR_TYPE_DIGIT;
		}
		else
		{
			charType = CHAR_TYPE_ALPHAPUNC;
			ch = toupper(ch);
		}
		
		// Is it the same character type as the last character?
		if (charType != prevCharType)
		{
			// Add zero to end of token for ASCIIZ string
			token[tokenIdx] = 0;
			// Change in character type.  Process token.
			switch (prevCharType)
			{
				case CHAR_TYPE_ALPHAPUNC:
					// Token is alpha or punctuation
					parseAlpha(token);
					break;
				case CHAR_TYPE_DIGIT:
					// Token is a number
					parseNumber(token);
					break;
				// Otherwise must be white space; do nothing
			}
			tokenIdx = 0;	// Start new token
			prevCharType = charType;
		}

		// If not whitespace and there is space, then add to token
		if ((charType != CHAR_TYPE_WHITESPACE) && (tokenIdx < MAX_TOKEN_NBYTES))
		{
			token[tokenIdx] = ch;
			++tokenIdx;
		}
		
		// Special handling for carriage return.
		if (ch == 0x0D)
		{
			// After carriage return, set the flag indicating that there is
			// a command to process.
			ServoCmdWaiting = true;
			// Now reset the internal variables used for parsing commands
			// in prepration for parsing the next command.
			pCmdFunc = NULL;
			servoNum = 255;			// Invalid servo number
			argumentRequired = false;
		}
	}	
}

static void parseAlpha(uint8_t * token)
{
	// Search the table for the token.  Linear search for now, since
	// the table is small.
	for (uint8_t i = 0; ParseTable[i].pCmdstr != NULL; ++i)
	{
		if (strcmp((const char *)ParseTable[i].pCmdstr,(const char *)token) == 0)
		{
			// If token found, then save the function for later use.
			// Also save the flag indicating whether an argument is required.
			pCmdFunc = ParseTable[i].pFunction;
			argumentRequired = ParseTable[i].argumentRequired;
			// If no argument is required, then call the function now.
			if (!ParseTable[i].argumentRequired)
			{
				pCmdFunc(0);	// Dummy argument
			}
			break;	// No need to continue search
		}
	}
}

static void parseNumber(uint8_t * token)
{
	// Convert the token to integer and pass to the parsing
	// function for the command.
	uint16_t arg = atoi((const char *)token);
	// Call the function if valid and if an argument is required.
	// (If an argument is not required, then the function was already called.)
	if ((pCmdFunc != NULL) && argumentRequired)
	{
		pCmdFunc(arg);
	}
}

/**********************************************************************
* Parsing functions for the various commands.  The argument to these
* functions is a pointer to an unsigned int.  Some functions do not
* need/use the argument, depending on whether the command requires
* an argument.
**********************************************************************/
static void ParseServoNum(uint16_t argument)
{
	servoNum = argument;
}
static void ParseServoHold(uint16_t argument)
{
	// Set current pulse width to target pulse width
	if (servoNum < NUM_SERVOS)
	{
		ServoPulseDefs[servoNum].currentPW_l16 = (uint32_t)ServoPulseDefs[servoNum].targetPW << 16;
	}
}
static void ParseServoLimp(uint16_t argument)
{
	// Set pulse width to 0 to indicate solid '0'.
	if (servoNum < NUM_SERVOS)
	{
		ServoCmdArray[servoNum].isCommanded = true;
		ServoCmdArray[servoNum].targetPW = 0;
	}
}
static void ParseServoPW(uint16_t argument)
{
	// Test for valid servo number and PW argument
	if ((servoNum < NUM_SERVOS) && (argument >= MINIMUM_PW) && (argument <= MAXIMUM_PW))
	{
		// Set pulse width for servo.
		ServoCmdArray[servoNum].isCommanded = true;
		ServoCmdArray[servoNum].targetPW = argument;
	}
}
static void ParseQCurrent(uint16_t argument)
{
	
}
static void ParseQPos(uint16_t argument)
{
	// Return a string with position feedback voltage in millivolts
	uint16_t adcResult;

	if (servoNum < NUM_SERVOS)
	{
		// The ADC channel number matches the servo number, so no
		// conversion required.
		adcResult = adc_read_filtered(servoNum);
		// Write "*NQP", where N = servo number
		uart_tx_put_char('*');
		if (servoNum >= 10)
		{
			uart_tx_put_char((servoNum / 10) + '0');	// Tens digit;
		}
		uart_tx_put_char((servoNum % 10) + '0');		// Ones digit;
		uart_tx_put_char('Q');
		uart_tx_put_char('P');
		// Convert to voltage at pin in mV.  ADC of 1024 corresponds
		// to the supply voltage of 3.3V = 3300 mV.
		// 211200 = (3300/1024) * 65536
		uint32_t voltageMilliVolts = (uint32_t)adcResult * 211200UL / 65536UL;
		// Write voltage in millivolts.
		uart_tx_uint16(voltageMilliVolts);
		// Write final carriage return
		uart_tx_put_char('\r');
	}
}
static void ParseQStatus(uint16_t argument)
{
	// Write status value for a servo, one of:
	// - 1 = Limp (output to servo is logic '0')
	// - 4 = Traveling (moving at a stable speed)
	// - 6 = Holding (keeping current position)
	// These values are a subset of the status values supported by the LSS protocol
	
	if (servoNum < NUM_SERVOS)
	{
		// Write "*NQ", where N = servo number
		uart_tx_put_char('*');
		if (servoNum >= 10)
		{
			uart_tx_put_char((servoNum / 10) + '0');	// Tens digit;
		}
		uart_tx_put_char((servoNum % 10) + '0');		// Ones digit;
		uart_tx_put_char('Q');
		// Write the code corresponding to the servo status
		if (ServoPulseDefs[servoNum].currentPW_l16 == 0)
		{
			// PW of 0 indicates no pulse (output constant '0')
			uart_tx_put_char('1');
		}
		else if (ServoPulseDefs[servoNum].currentPW_l16 != ((uint32_t)ServoPulseDefs[servoNum].targetPW << 16))
		{
			// Current PW not equal to target PW indicates servo moving
			uart_tx_put_char('4');
		}
		else
		{
			// Otherwise servo holding
			uart_tx_put_char('6');
		}
		// Write final carriage return
		uart_tx_put_char('\r');
	}
}
static void ParseQVoltage(uint16_t argument)
{
	// Return a string with battery voltage in millivolts
	uint16_t adcResult;
	
	// The ADC channel for battery voltage is 12
	adcResult = adc_read_filtered(12);
	// Write "*QV"
	uart_tx_put_char('*');
	uart_tx_put_char('Q');
	uart_tx_put_char('V');
	// Convert to voltage at pin in mV.  ADC of 1024 corresponds
	// to a battery voltage of 12639mV.
	// 808896 = (12639/1024) * 65536
	uint32_t voltageMilliVolts = (uint32_t)adcResult * 808896UL / 65536UL;
	// Write voltage in millivolts.
	uart_tx_uint16(voltageMilliVolts);
	// Write final carriage return
	uart_tx_put_char('\r');
}
static void ParseServoSpeed(uint16_t argument)
{
	if (servoNum < NUM_SERVOS)
	{
		// Set speed for servo.  Do not set the
		// isCommanded to true since the speed by itself
		// does not make a valid command.  Needs PW.
		ServoCmdArray[servoNum].targetSpeed = argument;
	}
}
static void ParseMoveTime(uint16_t argument)
{
	ServoCmdMoveTime = argument;
}
static void ParseVer(uint16_t argument)
{
	uart_tx_string(VERSION);
}
