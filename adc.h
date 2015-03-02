/**
 * @file adc.h
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

#ifndef ADC_H_
#define ADC_H_

extern volatile uint16_t ADC_LastResult;
extern volatile uint16_t ADC_LastResult_LP;

void ADC_Init(void);
void ADC_StartConversion(void);
uint16_t ADC_GetLastMove(uint16_t ref);

#endif /* ADC_H_ */
