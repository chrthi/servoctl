/**
 * @file main.c
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
#include <avr/sleep.h>
#include "uart.h"
#include "adc.h"
#include "servo.h"

int main(void) {
	// I/O ports
	DDRB = _BV(PB1) | _BV(PB4);
	PORTB = _BV(PB2) | _BV(4) | _BV(6) | _BV(7);
	DDRC = 0;
	PORTC = 0xff;
	DDRD = _BV(1);
	PORTD = _BV(1) | _BV(2) | _BV(3) | _BV(4) | _BV(5) | _BV(6) | _BV(7);

	UART_Init();
	ADC_Init();
	SRV_Init();

	// sleep mode = idle
	MCUCR = 0;

	sei();
	while(1) {
		sleep_enable();
		sleep_cpu();
	}
}
