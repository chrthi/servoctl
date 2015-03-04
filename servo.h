/**
 * @file servo.h
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

#ifndef SERVO_H_
#define SERVO_H_

void SRV_Init(void);
int8_t SRV_SetTarget(uint16_t pwm);
int8_t SRV_SetTargetIndex(uint8_t index);
uint16_t SRV_GetTarget(void);
int8_t SRV_SavePos(uint8_t index);

#define SRV_STATE_MOVING 254
#define SRV_STATE_CUSTOM 255
uint8_t SRV_GetCurrentIndex(void);

#define SRV_NUM_POSITIONS 2
//extern volatile uint16_t SRV_Pwm0, SRV_Pwm1;

#endif /* SERVO_H_ */
