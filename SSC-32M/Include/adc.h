/*
 * servo_fdbk.h
 *
 * Servo position feedback for the DeskPet servo controller.
 * Author : Mike Dvorsky
 */ 

#ifndef SERVO_FDBK_H
#define SERVO_FDBK_H

#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>

#include "globals.h"

void adc_init(void);
void adc_update(void);
uint16_t adc_read_immediate(uint8_t channel);
uint16_t adc_read_filtered(uint8_t channel);

#endif // SERVO_FDBK_H