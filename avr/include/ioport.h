/**
* @file ioport.h
* @brief GPIO operations.
* @author Liao MY
* @date 2014-07-20
*
* This file define some macro used to operate pins. The macro
* that begin with PIN_ is used to represent one pin, and The macro
* that begin with PORT_ is used to represent one 8-bit port.
* Additionally, there are some macro that begin with pin_ and
* port_. These macros are used to operate the pins and ports, such
* as output, input, and set direction.
*/

#ifndef _IOCTRL_HEADER_
#define _IOCTRL_HEADER_

#include <avr/io.h>
#include <avr/cpufunc.h>

/* Copyright (C) 
* 2014 - Liao MY
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
* 
*/


/*----------------------------------------*/
/*	For ATmega16
 *----------------------------------------*/
#if defined( __AVR_ATmega16__ ) \
	|| defined( __AVR_ATmega16A__ )

/* ADC */
#define PIN_ADC0	PIN_A0
#define PIN_ADC1	PIN_A1
#define PIN_ADC2	PIN_A2
#define PIN_ADC3	PIN_A3
#define PIN_ADC4	PIN_A4
#define PIN_ADC5	PIN_A5
#define PIN_ADC6	PIN_A6
#define PIN_ADC7	PIN_A7

/* Extern counter input */
#define PIN_T0		PIN_B0
#define PIN_T1		PIN_B1

/* Extern interrup */
#define PIN_INT0	PIN_D2
#define PIN_INT1	PIN_D3
#define PIN_INT2	PIN_D4

/* Alnlog compare input */
#define PIN_AINT0	PIN_B2
#define PIN_AINT1	PIN_B3

/* Compare output */
#define PIN_OC0		PIN_B3
#define PIN_OC1B	PIN_D4
#define PIN_OC1A	PIN_D5
#define PIN_OC2		PIN_D7

/* SPI */
#define PIN_SS		PIN_B4
#define PIN_MOSI	PIN_B5
#define PIN_MISO	PIN_B6
#define PIN_SCK		PIN_B7

/* I2C */
#define PIN_SCL		PIN_C0
#define PIN_SDA		PIN_C1

/* GPIO */
#define PORT_A	A
#define PORT_B	B
#define PORT_C	C
#define PORT_D	D


/* In avr-libc, PA[0-7], PB[0-7]... are defined as index of 
   bits. but there, I need define some macros to represent
   these pins. These macro just is a symbol, because I need 
   map these macro to related register. */

#define PIN_A0	A0
#define PIN_A1	A1
#define PIN_A2	A2
#define PIN_A3	A3
#define PIN_A4	A4
#define PIN_A5	A5
#define PIN_A6	A6
#define PIN_A7	A7

#define PIN_B0	B0
#define PIN_B1	B1
#define PIN_B2	B2
#define PIN_B3	B3
#define PIN_B4	B4
#define PIN_B5	B5
#define PIN_B6	B6
#define PIN_B7	B7

#define PIN_C0	C0
#define PIN_C1	C1
#define PIN_C2	C2
#define PIN_C3	C3
#define PIN_C4	C4
#define PIN_C5	C5
#define PIN_C6	C6
#define PIN_C7	C7

#define PIN_D0	D0
#define PIN_D1	D1
#define PIN_D2	D2
#define PIN_D3	D3
#define PIN_D4	D4
#define PIN_D5	D5
#define PIN_D6	D6
#define PIN_D7	D7

/* Map PIN_* to DDR*( set direction ) */
#define PA0_DDR	DDRA
#define PA1_DDR	DDRA
#define PA2_DDR	DDRA
#define PA3_DDR	DDRA
#define PA4_DDR	DDRA
#define PA5_DDR	DDRA
#define PA6_DDR	DDRA
#define PA7_DDR	DDRA

#define PB0_DDR	DDRB
#define PB1_DDR	DDRB
#define PB2_DDR	DDRB
#define PB3_DDR	DDRB
#define PB4_DDR	DDRB
#define PB5_DDR	DDRB
#define PB6_DDR	DDRB
#define PB7_DDR	DDRB

#define PC0_DDR	DDRC
#define PC1_DDR	DDRC
#define PC2_DDR	DDRC
#define PC3_DDR	DDRC
#define PC4_DDR	DDRC
#define PC5_DDR	DDRC
#define PC6_DDR	DDRC
#define PC7_DDR	DDRC

#define PD0_DDR	DDRD
#define PD1_DDR	DDRD
#define PD2_DDR	DDRD
#define PD3_DDR	DDRD
#define PD4_DDR	DDRD
#define PD5_DDR	DDRD
#define PD6_DDR	DDRD
#define PD7_DDR	DDRD


/* Map PIN_* to PORT*( output data ) */
#define PA0_PORT PORTA
#define PA1_PORT PORTA
#define PA2_PORT PORTA
#define PA3_PORT PORTA
#define PA4_PORT PORTA
#define PA5_PORT PORTA
#define PA6_PORT PORTA
#define PA7_PORT PORTA

#define PB0_PORT PORTB
#define PB1_PORT PORTB
#define PB2_PORT PORTB
#define PB3_PORT PORTB
#define PB4_PORT PORTB
#define PB5_PORT PORTB
#define PB6_PORT PORTB
#define PB7_PORT PORTB

#define PC0_PORT PORTC
#define PC1_PORT PORTC
#define PC2_PORT PORTC
#define PC3_PORT PORTC
#define PC4_PORT PORTC
#define PC5_PORT PORTC
#define PC6_PORT PORTC
#define PC7_PORT PORTC

#define PD0_PORT PORTD
#define PD1_PORT PORTD
#define PD2_PORT PORTD
#define PD3_PORT PORTD
#define PD4_PORT PORTD
#define PD5_PORT PORTD
#define PD6_PORT PORTD
#define PD7_PORT PORTD

/* Map PIN_* to PIN*( read data ) */
#define PA0_PIN	PINA
#define PA1_PIN	PINA
#define PA2_PIN	PINA
#define PA3_PIN	PINA
#define PA4_PIN	PINA
#define PA5_PIN	PINA
#define PA6_PIN	PINA
#define PA7_PIN	PINA

#define PB0_PIN	PINB
#define PB1_PIN	PINB
#define PB2_PIN	PINB
#define PB3_PIN	PINB
#define PB4_PIN	PINB
#define PB5_PIN	PINB
#define PB6_PIN	PINB
#define PB7_PIN	PINB

#define PC0_PIN	PINC
#define PC1_PIN	PINC
#define PC2_PIN	PINC
#define PC3_PIN	PINC
#define PC4_PIN	PINC
#define PC5_PIN	PINC
#define PC6_PIN	PINC
#define PC7_PIN	PINC

#define PD0_PIN	PIND
#define PD1_PIN	PIND
#define PD2_PIN	PIND
#define PD3_PIN	PIND
#define PD4_PIN	PIND
#define PD5_PIN	PIND
#define PD6_PIN	PIND
#define PD7_PIN	PIND

#else /* Other type */
  #warning Device is not supported.
#endif 


#define _iopin_outmode( Xn ) \
		do{ P##Xn##_DDR |= _BV(P##Xn); _NOP(); }while(0)
#define _iopin_inmode( Xn ) \
		do{ P##Xn##_DDR &= ~_BV(P##Xn);_NOP(); }while(0)
#define _iopin_outhigh( Xn )\
		(P##Xn##_PORT |= _BV(P##Xn))
#define _iopin_outlow( Xn )\
		(P##Xn##_PORT &= ~_BV(P##Xn))
#define _iopin_input( Xn )\
		((P##Xn##_PIN & (1<<P##Xn))>>P##Xn )

#define _ioport_outmode( Px ) \
		do{ DDR##Px = 0xff; _NOP(); }while(0)
#define _ioport_inmode( Px )\
		do{ DDR##Px = 0x00; _NOP(); }while(0)
#define _ioport_output( Px, v )\
		( PORT##Px = v )
#define _ioport_input( Px )\
		( PIN##Px )

/*----------------------------------------*/
/*	Opearions
 *----------------------------------------*/
#define iopin_outmode( Xn )	_iopin_outmode( Xn )
#define iopin_inmode( Xn ) 	_iopin_inmode( Xn )
#define iopin_outhigh( Xn )	_iopin_outhigh( Xn )
#define iopin_outlow( Xn )	_iopin_outlow( Xn )
#define iopin_out( Xn, v )	((v)? _iopin_outhigh(Xn) : _iopin_outlow(Xn) )
#define iopin_input( Xn )		_iopin_input(Xn)
#define iopin_pullup( Xn )	_iopin_outhigh( Xn )
#define iopin_nopullup( Xn )	_iopin_outlow( Xn )

#define ioport_outmode( Px )	_ioport_outmode( Px )
#define ioport_inmode( Px )	_ioport_inmode( Px )
#define ioport_output( Px, v )	_ioport_output( Px, v )
#define ioport_input( Px )	_ioport_input( Px )
#define ioport_pullup( Px )	_ioport_output( Rx, 0xff )
#define ioport_nopullup( Px )	_ioport_output( Rx, 0x00 )


#endif

