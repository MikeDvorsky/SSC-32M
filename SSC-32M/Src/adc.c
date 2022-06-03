/*
 * servo_fdbk.c
 *
 * Servo position feedback for the DeskPet servo controller.
 * Author : Mike Dvorsky
 */ 
#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>

#include "../Include/globals.h"
#include "../Include/adc.h"

// 12 servo feedback channels, plus battery voltage
#define NUM_ADC_CHANNELS 13

// Array of conversion results
static uint16_t filteredResults[NUM_ADC_CHANNELS];
// Pointer to the channel being converted
static uint8_t adcChannel;

void adc_init(void)
{
	// Disable the input buffer, disable pullup, disable
	// inversion for all ADC pins
	PORTD_PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;	// ADC0
	PORTD_PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;	// ADC1
	PORTD_PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;	// ADC2
	PORTD_PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;	// ADC3
	PORTD_PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc;	// ADC4
	PORTD_PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc;	// ADC5
	PORTD_PIN6CTRL = PORT_ISC_INPUT_DISABLE_gc;	// ADC6
	PORTD_PIN7CTRL = PORT_ISC_INPUT_DISABLE_gc;	// ADC7
	PORTE_PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;	// ADC8
	PORTE_PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;	// ADC9
	PORTE_PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;	// ADC10
	PORTE_PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;	// ADC11
	PORTF_PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;	// ADC12
	PORTF_PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;	// ADC13
	PORTF_PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc;	// ADC14
	PORTF_PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc;	// ADC15
	// Accumulate 1 sample
	ADC0_CTRLB = ADC_SAMPNUM_ACC1_gc;
	// Use reduced size sampling cap, VDD reference, and 1 MHz ADC clock
	ADC0_CTRLC = ADC_SAMPCAP_bm | ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV8_gc;
	// Delay 32 clocks before first sample after powerup
	ADC0_CTRLD = ADC_INITDLY_DLY32_gc;
	// No window comparator
	ADC0_CTRLE = ADC_WINCM_NONE_gc;
	// No additional sampling cycles
	ADC0_SAMPCTRL = 0;
	// Duty cycle of ADC clock = 25% (required for <= 1.5 MHz clock)
	ADC0_CALIB = ADC_DUTYCYC_DUTY25_gc;
	// Enable ADC
	ADC0_CTRLA = ADC_ENABLE_bm;
	// Clear result ready flag if set
	ADC0_INTFLAGS = ADC_RESRDY_bm;
	
	// Loop through all channels, taking the first reading
	for (adcChannel = 0; adcChannel < NUM_ADC_CHANNELS; ++adcChannel)
	{
		filteredResults[adcChannel] = adc_read_immediate(adcChannel);
	}
	// Start first conversion for periodic reading
	adcChannel = 0;
	ADC0_MUXPOS = adcChannel;
	ADC0_COMMAND = ADC_STCONV_bm;
}

void adc_update(void)
{
	uint16_t adcResult;
	
	// Cycle through the ADC pins, taking readings in sequence
	
	// If conversion in progress, return
	if ((ADC0_COMMAND & ADC_STCONV_bm) != 0)
	{
		return;
	}

	// Clear result ready flag from the previous conversion
	ADC0_INTFLAGS = ADC_RESRDY_bm;
	
	// Get result from the previous conversion, filter, and store.
	// Use filter equation
	//   newFiltered = 0.75 * prevFiltered + 0.25 * latestConversion
	//               = ((3 * prevFiltered) + latestConversion) / 4
	adcResult = ADC0_RES;
	filteredResults[adcChannel] = ((filteredResults[adcChannel] * 3) + adcResult) / 4;
	
	// Increment channel number, wrapping if needed
	++adcChannel;
	if (adcChannel >= NUM_ADC_CHANNELS)
	{
		adcChannel = 0;
	}
	
	// Start next conversion
	ADC0_MUXPOS = adcChannel;
	ADC0_COMMAND = ADC_STCONV_bm;
	
}

uint16_t adc_read_immediate(uint8_t channel)
{
	uint16_t adcVoltage;
	
	// Set channel (ADC0-ADC15)
	ADC0_MUXPOS = channel;
	// Start conversion
	ADC0_COMMAND = ADC_STCONV_bm;
	// Wait for conversion to complete
	while((ADC0_INTFLAGS & ADC_RESRDY_bm) == 0);
	// Clear result ready flag
	ADC0_INTFLAGS = ADC_RESRDY_bm;
	// Read result
	adcVoltage = ADC0_RES;
	// Return result
	return adcVoltage;
}


uint16_t adc_read_filtered(uint8_t channel)
{
	return filteredResults[channel];
}
