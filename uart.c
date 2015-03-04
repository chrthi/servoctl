/**
 * @file uart.c
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
#include <avr/pgmspace.h>
#include <ctype.h>
#include <stdio.h>
#include "servo.h"
#include "adc.h"

#define BAUDRATE 115200UL
#define UBRR_val ((F_CPU + (4UL * BAUDRATE)) / (8UL * BAUDRATE) - 1UL)
#define CASE_INSENSITIVE_CMDS
#define LEN(a) (sizeof(a)/sizeof(*a))

typedef struct uartcmd {
	void (*func)(const char *args);
	char firstchar;
	uint8_t cmdlen;
	PGM_P cmd;
} uartcmd_t;

/*
 * mv 0 #move to saved point 0
 * pos # print current position
 * stat # print extended status message
 * maint #unlock maintenance mode:
 * save 0 #save current position as point 0
 * pwm 922 #set pwm target
 * cal #auto-calibrate full servo range
 */

#define UART_CMD_FUNC(name) \
	static void cmd_ ## name (const char *args)
#define UART_CMD_DECL(name) \
		static const char _cmdstr_ ## name [] PROGMEM  = #name; \
		UART_CMD_FUNC(name)
#define UART_CMD_ENTRY(name) {cmd_ ## name, '\0', 0, _cmdstr_ ## name}
UART_CMD_DECL(mv);
UART_CMD_DECL(pos);
UART_CMD_DECL(stat);
UART_CMD_DECL(maint);
UART_CMD_DECL(save);
UART_CMD_DECL(pwm);
UART_CMD_DECL(cal);

static uartcmd_t uartcmds[] = {
		UART_CMD_ENTRY(mv),
		UART_CMD_ENTRY(pos),
		UART_CMD_ENTRY(stat),
		UART_CMD_ENTRY(maint),
		UART_CMD_ENTRY(save),
		UART_CMD_ENTRY(pwm),
		UART_CMD_ENTRY(cal)
};

#define uartprintf_P(fmt, ...) do { \
		sprintf_P(uart_txbuf, fmt, __VA_ARGS__); \
		uart_txpos = uart_txbuf; \
		UCSRB |= _BV(UDRIE); \
	} while(0)
//loop_until_bit_is_clear(UCSRB, UDRIE); \

#define uartputs_P(fmt) do { \
		strcpy_P(uart_txbuf, fmt); \
		uart_txpos = uart_txbuf; \
		UCSRB |= _BV(UDRIE); \
	} while(0)

char uart_rxbuf[16];
char uart_txbuf[64];
volatile char *uart_txpos;

volatile static struct {
	int isMaintMode:1;
} state;

UART_CMD_FUNC(mv) {
	if(args[1] != '\0' || args[0] < '0' || args[0] > '1') return;
	switch(SRV_SetTargetIndex(args[0]-'0')) {
	case 0: //success
		uartputs_P(PSTR("moving...\n"));
		break;
	case -1: //already there
		uartputs_P(PSTR("ok\n"));
		break;
	default:
		break;
	}
	return;
}

UART_CMD_FUNC(pos) {
	uint8_t idx = SRV_GetCurrentIndex();
	switch(idx) {
	case SRV_STATE_MOVING:
		uartputs_P(PSTR("moving\n"));
		break;
	case SRV_STATE_CUSTOM:
		uartputs_P(PSTR("custom\n"));
		break;
	default:
		uartprintf_P(PSTR("%u\n"), (unsigned int)idx);
		break;
	}
}

UART_CMD_FUNC(stat) {
	uartprintf_P(PSTR("PWM=%u, ADC=%u\n"), OCR1A /*(unsigned int)SRV_GetTarget()*/, (unsigned int)ADC_LastResult);
}

UART_CMD_FUNC(maint) {
	uint8_t m=state.isMaintMode ? 0: 1;
	state.isMaintMode = m;
	if(m) {
		uartputs_P(PSTR("unlocked\n"));
	} else {
		uartputs_P(PSTR("locked\n"));
	}
}

UART_CMD_FUNC(save) {
	if(args[1] != '\0' || args[0] < '0' || args[0] > '9') return;
	if(!SRV_SavePos(args[0]-'0')) uartputs_P(PSTR("ok\n"));
}

UART_CMD_FUNC(pwm) {
	uint16_t pwm=0;
	uint8_t numDigits=0;
	const char *p=args;
	while(*p >= '0' && *p <= '9' && numDigits<5) {
		pwm = pwm*10 + (*p-'0');
		++p;
		++numDigits;
	}
	if(numDigits == 0) return;
	if(SRV_SetTarget(pwm)) uartputs_P(PSTR("ok\n"));
	else uartputs_P(PSTR("moving...\n"));
}

UART_CMD_FUNC(cal) {

}

void UART_Init(void) {
	// Prepare commands list for easier searching
	for(uint8_t i=0; i<LEN(uartcmds); ++i) {
#ifdef CASE_INSENSITIVE_CMDS
		uartcmds[i].firstchar = tolower(pgm_read_byte(uartcmds[i].cmd));
#else
		uartcmds[i].firstchar = pgm_read_byte(uartcmds[i].cmd);
#endif
		uartcmds[i].cmdlen = strlen_P(uartcmds[i].cmd);
	}

	UCSRA = _BV(U2X);
	UCSRB = _BV(RXCIE) | _BV(RXEN) | _BV(TXEN);
	UCSRC = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0);
	UBRRH = UBRR_val >> 8;
	UBRRL = UBRR_val & 0xff;
}

void UART_ReportDone(void) {
	uartputs_P(PSTR("done.\n"));
}

void UART_ReportProgress(uint16_t pwm, uint16_t adc) {
	uint8_t c;
	char *p=uart_txbuf;
	//loop_until_bit_is_clear(UCSRB, UDRIE);
	c = pwm >>12;       *p++ = '0'+c;
	c = pwm >> 8 & 0xf; *p++ = (c<10) ? '0'+c : 'a'-10+c;
	c = pwm >> 4 & 0xf; *p++ = (c<10) ? '0'+c : 'a'-10+c;
	c = pwm      & 0xf; *p++ = (c<10) ? '0'+c : 'a'-10+c;
	*p++=';';
	c = adc >> 8;       *p++ = '0'+c;
	c = adc >> 4 & 0xf; *p++ = (c<10) ? '0'+c : 'a'-10+c;
	c = adc      & 0xf; *p++ = (c<10) ? '0'+c : 'a'-10+c;
	*p++ = '\n';
	*p = '\0';
	uart_txpos = uart_txbuf;
	UCSRB |= _BV(UDRIE);
}

ISR(USART_RXC_vect) {
	static char *rxpos = uart_rxbuf;
	register char c = UDR;
	if(c == '\n' || c == '\r') {
		if(rxpos==uart_rxbuf) return;
		if(rxpos) {
			*rxpos = '\0';
			for(uint8_t i=0; i<LEN(uartcmds); ++i) {
				if(uartcmds[i].func &&
#ifdef CASE_INSENSITIVE_CMDS
				tolower(uart_rxbuf[0]) == uartcmds[i].firstchar && !strncasecmp_P(uart_rxbuf, uartcmds[i].cmd, uartcmds[i].cmdlen)
#else
				uart_rxbuf[0] == uartcmds[i].firstchar && !strncmp_P(uart_rxbuf, uartcmds[i].cmd, uartcmds[i].cmdlen)
#endif
				&& (!uart_rxbuf[uartcmds[i].cmdlen] || isspace(uart_rxbuf[uartcmds[i].cmdlen]))) {
					char *c=uart_rxbuf+uartcmds[i].cmdlen;
					while(isspace(*c)) ++c;
					if(!*c) c = NULL;
					uartcmds[i].func(c);
					break;
				}
			}
		}
		rxpos = uart_rxbuf;
		return;
	}
	if(!rxpos) return;
	*rxpos++ = c;
	if(rxpos >= uart_rxbuf+(LEN(uart_rxbuf))) {
		rxpos = NULL;
	}
}

ISR(USART_UDRE_vect) {
	if(uart_txpos) {
		register char c = *uart_txpos++;
		if(c) {
			UDR = c;
		} else {
			uart_txpos = NULL;
			UCSRB &=~ _BV(UDRIE); \
		}
	}
}
