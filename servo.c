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
#define PWM_TOP (F_CPU/(PWM_PSC*PWM_FREQ)-1)
#define PWM_SPEED 64
#define PWM_TO_OCR(p) (((p) + (1<<3)) >> 4)
#define PWM_FROM_OCR(r) ((p) << 4)

volatile static uint16_t pwm_target;
volatile static uint16_t pwm_current;
static uint16_t pwm_targets_ee[SRV_NUM_POSITIONS] EEMEM;
static uint16_t pwm_targets[SRV_NUM_POSITIONS];
static uint16_t adc_vals_ee[SRV_NUM_POSITIONS] EEMEM;
static uint16_t adc_vals[SRV_NUM_POSITIONS];

static void SRV_calcFromADC(uint16_t adcval, volatile uint16_t *target, volatile uint16_t *current);

void SRV_Init(void) {
	eeprom_read_block(pwm_targets, &pwm_targets_ee[0], sizeof(pwm_targets));
	eeprom_read_block(adc_vals, &adc_vals_ee[0], sizeof(adc_vals));
	ICR1 = PWM_TOP; // 50Hz = 18431
	if(pwm_targets[0]==0xffff) {
		OCR1A = (PWM_TOP+1+10)/20; // 1ms = 921.6. Standard range = 461 - 1382
	} else {
		SRV_calcFromADC(ADC_LastResult, &pwm_target, &pwm_current);
		OCR1A = PWM_TO_OCR(pwm_current);
	}
	TCCR1A = _BV(COM1A1) | _BV(WGM11);
	TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
	TIMSK = _BV(TOIE1);
}

static void SRV_calcFromADC(uint16_t adcval, volatile uint16_t *target, volatile uint16_t *current) {
	uint8_t iLE=255, iGE=255;
	uint16_t mindiffLE=0xffff, mindiffGE=0xffff;
	for(uint8_t i=0; i<SRV_NUM_POSITIONS; ++i) {
		if(adc_vals[i]<=adcval && adcval-adc_vals[i] < mindiffLE) {
			iLE=i;
			mindiffLE=adcval-adc_vals[i];
		}
		if(adc_vals[i]>=adcval && adc_vals[i]-adcval < mindiffGE) {
			iGE=i;
			mindiffGE=adc_vals[i]-adcval;
		}
	}
	if(iLE==255) iLE=iGE;
	if(iGE==255) iGE=iLE;
	if(iLE==255) return;
	if(target)
		*target = mindiffLE<mindiffGE ? pwm_targets[iLE] : pwm_targets[iGE];
	if(current) {
		uint16_t adcdiff = (iGE==iLE) ? 1 : adc_vals[iGE]-adc_vals[iLE];
		*current = pwm_targets[iLE]+
				(((int32_t)pwm_targets[iGE]-pwm_targets[iLE])
						*(int32_t)mindiffLE + ((adcdiff+1)>>1))
				/ adcdiff;
	}
}

int8_t SRV_SetTarget(uint16_t pwm) {
	if(pwm_target == pwm) return -1;
	SRV_PWM_ON();
	SRV_PWR_ON();
	pwm_target=pwm;
	return 0;
}

int8_t SRV_SetTargetIndex(uint8_t index) {
	if(index >= SRV_NUM_POSITIONS) return -2;
	uint16_t pwm = pwm_targets[index];
	if(pwm >= PWM_TOP) return -3;
	return SRV_SetTarget(pwm);
}

uint16_t SRV_GetTarget(void) {
	return pwm_target;
}

int8_t SRV_SavePos(uint8_t index) {
	if(index >= SRV_NUM_POSITIONS) return -1;
	eeprom_write_word(&pwm_targets_ee[index], pwm_targets[index] = pwm_current);
	return 0;
}

uint8_t SRV_GetCurrentIndex(void) {
	uint16_t pwm = pwm_target;
	if(pwm_current != pwm) return SRV_STATE_MOVING;
	for(uint8_t i=0; i<SRV_NUM_POSITIONS; ++i)
		if(pwm == pwm_targets[i]) return i;
	return SRV_STATE_CUSTOM;
}

ISR(TIMER1_OVF_vect) {
	register uint16_t target = pwm_target;
	register uint16_t current = pwm_current;
	static uint16_t final_adc_position;
	if(current == target) {
		if(SRV_IS_PWR_ON()) {
			UART_ReportProgress(current, ADC_LastResult);
			if(ADC_GetLastMove(0xffff)<3) {
				//The servo is still powered on, but has stopped moving.
				SRV_PWM_OFF();
				///@todo: This won't work.
				SRV_PWR_OFF();
				final_adc_position=ADC_LastResult;
				UART_ReportDone();
			}
		} else {
			if(ADC_GetLastMove(final_adc_position)>10) {
				//The servo has moved while powered off. Move it back.
				SRV_calcFromADC(ADC_LastResult, NULL, &pwm_current);
				SRV_PWR_ON();
				SRV_PWM_ON();
			}
		}
	} else {
		UART_ReportProgress(current, ADC_LastResult);
		if(current > target) {
			if(current > target+PWM_SPEED)
				current -= PWM_SPEED;
			else
				current = target;
		} else {
			if(current < target-PWM_SPEED)
				current += PWM_SPEED;
			else
				current = target;
		}
		pwm_current = current;
		OCR1A = PWM_TO_OCR(current);
		ADC_StartConversion();
	}
}
