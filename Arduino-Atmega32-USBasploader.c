/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/
  Copyright (c) 2007 David A. Mellis
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

#define NUM_DIGITAL_PINS            32
#define NUM_ANALOG_INPUTS           8
#define analogInputToDigitalPin(p)  ((p < 8) ? (p) + 24 : -1)

#define digitalPinHasPWM(p)         ((p) == 3 || (p) == 12 || (p) == 13 || (p) == 15)

static const uint8_t SS   = 4;
static const uint8_t MOSI = 5;
static const uint8_t MISO = 6;
static const uint8_t SCK  = 7;

static const uint8_t SDA = 17;
static const uint8_t SCL = 16;
#define LED_BUILTIN 13

static const uint8_t A0 = 31;	//analog pins are reversely oriented 
static const uint8_t A1 = 30;
static const uint8_t A2 = 29;
static const uint8_t A3 = 28;
static const uint8_t A4 = 27;
static const uint8_t A5 = 26;
static const uint8_t A6 = 25;
static const uint8_t A7 = 24;

#define digitalPinToPCICR(p)    (((p) >= 0 && (p) <= 21) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) (((p) <= 7) ? 2 : (((p) <= 13) ? 0 : 1))
#define digitalPinToPCMSK(p)    (((p) <= 7) ? (&PCMSK2) : (((p) <= 13) ? (&PCMSK0) : (((p) <= 21) ? (&PCMSK1) : ((uint8_t *)0))))
#define digitalPinToPCMSKbit(p) (((p) <= 7) ? (p) : (((p) <= 13) ? ((p) - 8) : ((p) - 14)))

#define digitalPinToInterrupt(p)  ((p) == 2 ? 0 : ((p) == 3 ? 1 : NOT_AN_INTERRUPT))

#ifdef ARDUINO_MAIN

// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.

// ATMEL ATMEGA8 & 168 / ARDUINO
//
//                  +-----\/-----+
//            PB0  1|            |40  PC5 (AI 5)
//      (D 0) PB1  2|            |39  PC4 (AI 4)
//      (D 1) PB2  3|            |38  PC3 (AI 3)
//      (D 2) PB3  4|            |37  PC2 (AI 2)
// PWM+ (D 3) PB4  5|            |36  PC1 (AI 1)
//      (D 4) PB5  6|            |35  PC0 (AI 0)
//            PB6  7|            |34  GND
//            PB7  8|            |33  AREF
//          RESET  9|            |32  AVCC
//            VCC 10|            |31  PB5 (D 13)
//          XTAL2 11|            |30  PB4 (D 12)
//          XTAL1 12|            |29  PB3 (D 11) PWM
//      (D 7) PD7 13|            |28  PB2 (D 10) PWM
//      (D 8) PB0 14|            |27  PB1 (D 9) PWM
//            PB6 15|            |26  AVCC
//            PB7 16|            |25  PB5 (D 13)
// PWM+ (D 5) PD5 17|            |24  PB4 (D 12)
// PWM+ (D 6) PD6 18|            |23  PB3 (D 11) PWM
//      (D 7) PD7 19|            |22  PB2 (D 10) PWM
//      (D 8) PB0 20|            |21  PB1 (D 9) PWM
//                  +------------+
//
// (PWM+ indicates the additional PWM pins on the ATmega168.)

// ATMEL ATMEGA1280 / ARDUINO
//
// 0-7 PE0-PE7   works
// 8-13 PB0-PB5  works
// 14-21 PA0-PA7 works 
// 22-29 PH0-PH7 works
// 30-35 PG5-PG0 works
// 36-43 PC7-PC0 works
// 44-51 PJ7-PJ0 works
// 52-59 PL7-PL0 works
// 60-67 PD7-PD0 works
// A0-A7 PF0-PF7
// A8-A15 PK0-PK7


// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)BDCA

const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &DDRB,
	(uint16_t) &DDRD,
	(uint16_t) &DDRC,
	(uint16_t) &DDRA,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PORTB,
	(uint16_t) &PORTD,
	(uint16_t) &PORTC,
	(uint16_t) &PORTA,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PINB,
	(uint16_t) &PIND,
	(uint16_t) &PINC,
	(uint16_t) &PINA,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	PB,		//*	0	PB0
	PB,		//*	1	PB1
	PB,		//*	2	PB2
	PB,		//*	3	PB3
	PB,		//*	4	PB4
	PB,		//*	5	PB5
	PB,		//*	6	PB6
	PB,		//*	7	PB7
	PD,		//*	8	PD0 	RXD
	PD,		//*	9	PD1 	TXD
	PD,		//*	10	PD2
	PD,		//*	11	PD3
	PD,		//*	12	PD4
	PD,		//*	13	PD5
	PD,		//*	14	PD6
	PD,		//*	15	PD7
	PC,		//*	16	PC0
	PC,		//*	17	PC1
	PC,		//*	18	PC2
	PC,		//*	19	PC3
	PC,		//*	20	PC4
	PC,		//*	21	PC5
	PC,		//*	22	PC6
	PC,		//*	23	PC7
	PA,		//*	24	PA0		ANALOG 0
	PA,		//*	25	PA1		ANALOG 1
	PA,		//*	26	PA2		ANALOG 2
	PA,		//*	27	PA3		ANALOG 3
	PA,		//*	28	PA4		ANALOG 4
	PA,		//*	29	PA5		ANALOG 5
	PA,		//*	30	PA6		ANALOG 6
	PA,		//*	31	PA7		ANALOG 7
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	_BV(0), /* 0, port B */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6),
	_BV(7),
	_BV(0), /* 0, port D */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6),
	_BV(7),
	_BV(0), /* 0, port C */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6),
	_BV(7),
	_BV(7), /* 7, port A is reversely oriented*/
	_BV(6),
	_BV(5),
	_BV(4),
	_BV(3),
	_BV(2),
	_BV(1),
	_BV(0),
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	NOT_ON_TIMER,	//*	0 -  PB0
	NOT_ON_TIMER,	//*	1 -  PB1
	NOT_ON_TIMER,	//*	2 - PB2
	TIMER0A,		//*	3 - PB3 --- TIMER OC0
	NOT_ON_TIMER,	//*	4 - PB4
	NOT_ON_TIMER,	//*	5 - PB5
	NOT_ON_TIMER,	//*	6 - PB6
	NOT_ON_TIMER,	//*	7 - PB7
		
	//*	these arnt brought out but are needed for access to LED
	NOT_ON_TIMER,	//*	8 - PD0
	NOT_ON_TIMER,	//*	9 - PD1
	NOT_ON_TIMER,	//*	10 - PD2
	NOT_ON_TIMER,	//*	11 - PD3
	TIMER1B,		//*	12 - PD4-----------
	TIMER1A,		//*	13 - PD5-----------
	NOT_ON_TIMER,	//*	14 - PD6
	TIMER2,			//*	15 - PD7------------
	
	NOT_ON_TIMER,	//*	16 - PC0
	NOT_ON_TIMER,	//*	17 - PC1
	NOT_ON_TIMER,	//*	18 - PC2
	NOT_ON_TIMER,	//*	19 - PC3
	NOT_ON_TIMER,	//*	20 - PC4
	NOT_ON_TIMER,	//*	21 - PC5
	NOT_ON_TIMER,	//*	22 - PC6
	NOT_ON_TIMER,	//*	23 - PC7
	
	NOT_ON_TIMER,	//*	24 - PA0
	NOT_ON_TIMER,	//*	25 - PA1
	NOT_ON_TIMER,	//*	26 - PA2
	NOT_ON_TIMER,	//*	27 - PA3
	NOT_ON_TIMER,	//*	28 - PA4
	NOT_ON_TIMER,	//*	29 - PA5
	NOT_ON_TIMER,	//*	30 - PA6
	NOT_ON_TIMER,	//*	31 - PA7
};

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR   Serial
#define SERIAL_PORT_HARDWARE  Serial

#endif
