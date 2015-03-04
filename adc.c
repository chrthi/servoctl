/**
 * @file adc.c
 *
 */

/*
 * This file is part of servoctl.
 *
 * servoctl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * servoctl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with servoctl.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "adc.h"

volatile uint16_t ADC_LastResult;
volatile uint16_t ADC_LastResult_LP;

void ADC_Init(void) {
	ADMUX = /*_BV(REFS1) |*/ _BV(REFS0) | 6; //uncomment REFS1 to use 2.56V instead of 5V
	ADCSRA = _BV(ADEN) | _BV(ADSC) | ADC_PSCBITS;
	{
		loop_until_bit_is_clear(ADCSRA, ADSC);
		ADC_LastResult_LP = ADC_LastResult = ADC;
	}
	ADCSRA = _BV(ADEN) | _BV(ADIF) | ADC_PSCBITS;
}

uint16_t ADC_GetLastMove(uint16_t ref) {
	uint16_t res = ADC_LastResult;
	if(ref > 0x3ff) ref=ADC_LastResult_LP;
	return (ref > res) ? ref-res : res-ref;
}

ISR(ADC_vect) {
	uint16_t adc = ADC;
	ADC_LastResult_LP = (ADC_LastResult_LP + adc + 1) >> 1;
	ADC_LastResult = adc;
}
