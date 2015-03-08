/**
 * @file servo.c
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
#include <avr/eeprom.h>
#include "servo.h"
#include "adc.h"
#include "uart.h"

//Servo timing
#define PWM_PSC 8UL
#define PWM_FREQ 50UL
#define PWM_TOP (F_CPU/(PWM_PSC*PWM_FREQ)-1UL)
#define PWM_SPEED 200UL
#define PWM_TO_OCR(p) (((p) + (1<<2)) >> 3)
#define PWM_FROM_OCR(r) ((r) << 3)

#define SRV_PWR_PORT PORTB
#define SRV_PWR_DDR DDRB
#define SRV_PWR_P 0
#define SRV_PWR_ON() do {SRV_PWR_PORT &=~ _BV(SRV_PWR_P);} while(0)
#define SRV_PWR_OFF() do {SRV_PWR_PORT |= _BV(SRV_PWR_P);} while(0)
#define SRV_IS_PWR_ON() (!(PORTB & _BV(PB0)))
#define SRV_PWM_ON() do {TCCR1A |= _BV(COM1A1);} while(0)
#define SRV_PWM_OFF() do {TCCR1A |= _BV(COM1A1);} while(0)

volatile static uint16_t pwm_target;
volatile static uint16_t pwm_current;
static uint16_t pwm_targets_ee[SRV_NUM_POSITIONS] EEMEM;
static uint16_t pwm_targets[SRV_NUM_POSITIONS];
static uint16_t adc_vals_ee[SRV_NUM_POSITIONS] EEMEM;
static uint16_t adc_vals[SRV_NUM_POSITIONS];

static int8_t SRV_calcFromADC(uint16_t adcval, volatile uint16_t *target, volatile uint16_t *current);

void SRV_Init(void) {
	eeprom_read_block(pwm_targets, &pwm_targets_ee[0], sizeof(pwm_targets));
	for(int i=0; i<SRV_NUM_POSITIONS; ++i)
		if(pwm_targets[i] != 0xffff)
			pwm_targets[i] = PWM_FROM_OCR(pwm_targets[i]);
	eeprom_read_block(adc_vals, &adc_vals_ee[0], sizeof(adc_vals));
	ICR1 = PWM_TOP; // 50Hz = 18431

	/* The timer period is 20 ms.
	 * The ADC shall finish at 0.5 ms = 1/40 of the period.
	 * Adding half of the divisor (+20) is useful for correct rounding.
	 * The ADC takes 13 ADC clocks. */
	OCR1B = (PWM_TOP+1+20)/40 - 13*(ADC_PSC/PWM_PSC);

	TCCR1A = _BV(WGM11);
	TCCR1B = _BV(WGM13) | _BV(WGM12);
	if(SRV_calcFromADC(ADC_LastResult, &pwm_target, &pwm_current)) {
		/*If the eeprom is uninitialized, set the PWM to center (1 ms).
		 * That's 1/20 of the timer period.
		 * Adding half of the divisor (+10) is useful for correct rounding. */
		pwm_current = pwm_target = PWM_FROM_OCR((PWM_TOP+1UL+10UL)/20UL);
		OCR1A = (PWM_TOP+1UL+10UL)/20UL;
		SRV_PWR_OFF();
	} else {
		OCR1A = PWM_TO_OCR(pwm_current);
		if(pwm_target != pwm_current) {
			SRV_PWR_ON();
			SRV_PWM_ON();
		} else {
			SRV_PWR_OFF();
		}
	}
	TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
	TIMSK = _BV(OCIE1A) | _BV(OCIE1B);
}

static int8_t SRV_calcFromADC(uint16_t adcval, volatile uint16_t *target, volatile uint16_t *current) {
	uint8_t iLE=255, iGE=255;
	uint16_t mindiffLE=0xffff, mindiffGE=0xffff;
	for(uint8_t i=0; i<SRV_NUM_POSITIONS; ++i) {
		if(adc_vals[i] == 0xffff) continue;
		if(adc_vals[i]<=adcval && adcval-adc_vals[i] < mindiffLE) {
			iLE=i;
			mindiffLE=adcval-adc_vals[i];
		}
		if(adc_vals[i]>=adcval && adc_vals[i]-adcval < mindiffGE) {
			iGE=i;
			mindiffGE=adc_vals[i]-adcval;
		}
	}

	//handle corner cases:
	//No points are defined
	if(iLE==255 && iGE==255) return -1;
	else if(iLE==255 || iLE==iGE) {
		//We are left of all points, clamp to lowest
		//Or we've exactly hit a point, so just use that.
		if(target) *target=pwm_targets[iGE];
		if(current) *current=pwm_targets[iGE];
	} else if(iGE==255) {
		//We are right of all points, clamp to highest
		if(target) *target=pwm_targets[iLE];
		if(current) *current=pwm_targets[iLE];
	} else {
		//We are between two defined points. Interpolate linearly.
		if(target)
			*target = mindiffLE<mindiffGE ? pwm_targets[iLE] : pwm_targets[iGE];
		if(current) {
			uint16_t adcdiff = adc_vals[iGE]-adc_vals[iLE];
			*current = pwm_targets[iLE]+
					(((int32_t)pwm_targets[iGE]-pwm_targets[iLE])
							*(int32_t)mindiffLE + (int32_t)((adcdiff+1)>>1))
					/ adcdiff;
		}
	}
	return 0;
}

int8_t SRV_SetTarget(uint16_t pwm) {
	pwm = PWM_FROM_OCR(pwm);
	if(pwm_target == pwm) return -1;
	pwm_target=pwm;
	SRV_PWM_ON();
	SRV_PWR_ON();
	return 0;
}

int8_t SRV_SetTargetIndex(uint8_t index) {
	if(index >= SRV_NUM_POSITIONS) return -2;
	uint16_t pwm = pwm_targets[index];
	if(pwm >= PWM_TOP) return -3;
	if(pwm_target == pwm) return -1;
	pwm_target=pwm;
	SRV_PWM_ON();
	SRV_PWR_ON();
	return 0;
}

uint16_t SRV_GetTarget(void) {
	return PWM_TO_OCR(pwm_target);
}

int8_t SRV_SavePos(uint8_t index) {
	if(index >= SRV_NUM_POSITIONS) return -1;
	eeprom_write_word(&pwm_targets_ee[index], PWM_TO_OCR(pwm_targets[index] = pwm_current));
	eeprom_write_word(&adc_vals_ee[index], adc_vals[index] = ADC_LastResult);
	return 0;
}

void SRV_Clear(void) {
	for(int i=0; i<SRV_NUM_POSITIONS; ++i) {
		pwm_targets[i] = 0xffff;
		adc_vals[i] = 0xffff;
	}
	eeprom_write_block(pwm_targets, &pwm_targets_ee[0], sizeof(pwm_targets));
	eeprom_write_block(adc_vals, &adc_vals_ee[0], sizeof(adc_vals));
}

uint8_t SRV_GetCurrentIndex(void) {
	uint16_t pwm = pwm_target;
	if(pwm_current != pwm) return SRV_STATE_MOVING;
	for(uint8_t i=0; i<SRV_NUM_POSITIONS; ++i)
		if(pwm == pwm_targets[i]) return i;
	return SRV_STATE_CUSTOM;
}

ISR(TIMER1_COMPA_vect) {
	const uint16_t target = pwm_target;
	uint16_t current = pwm_current;
	static uint16_t final_adc_position=0xffff;
	static uint8_t count=PWM_FREQ;
	if(current == target) {

		if(count<PWM_FREQ-1) {
			UART_ReportProgress(PWM_TO_OCR(current), ADC_LastResult);
			/* This happens when the PWM is already the target value, but the
			 * servo is still busy moving to that point. */
			if(ADC_GetLastMove(0xffff)<3) {
				/* Servo is done moving */
				count = PWM_FREQ-1;
			}
		}

		switch(count) {
		case PWM_FREQ-1:
			/* Servo is just done moving.
			 * transition to the continuous loop */
			SRV_PWM_OFF();
			SRV_PWR_OFF();
			final_adc_position=ADC_LastResult;
			//UART_ReportDone();
			break;
		case 2*PWM_FREQ-1:
			/* Near the end of the continuous loop. Check the servo position.
			 * For this, we need to power the servo for one cycle. */
			SRV_PWR_ON();
			break;
		case 2*PWM_FREQ:
			/* End of the continuous loop. Check whether the servo has moved. */
			count = PWM_FREQ;
			if(final_adc_position!=0xffff && ADC_GetLastMove(final_adc_position)>10 && 0) {
				/*The servo has moved while powered off.
				 * We set the current PWM according to the measured position
				 * so it will be moved back in a controlled way.
				 * This ends the continuous loop. */
				SRV_calcFromADC(ADC_LastResult, NULL, &pwm_current);
				SRV_PWR_ON();
				SRV_PWM_ON();
			} else {
				/* Nothing happened, so stop the signals (we just needed one
				 * pulse for the measurement) and continue the loop.
				 */
				SRV_PWM_OFF();
				SRV_PWR_OFF();
			}
			break;
		default:
			break;
		}
		++count;
	} else {
		/* We are currently moving. Gradually change the PWM signal until we
		 * reach the target. */
		UART_ReportProgress(PWM_TO_OCR(current), ADC_LastResult);
		if(current > target) {
			if(current > target+PWM_SPEED) {
				current -= PWM_SPEED;
			} else {
				current = target;
				count = 0;
			}
		} else {
			if(current < target-PWM_SPEED) {
				current += PWM_SPEED;
			} else {
				current = target;
				count = 0;
			}
		}
		pwm_current = current;
		OCR1A = PWM_TO_OCR(current);
	}
}

ISR(TIMER1_COMPB_vect) {
	if(SRV_IS_PWR_ON())
		ADC_StartConversion();
}
