/**
* @file serial.c
* @brief 
* @author Liao MY
* @version 
* @date 2014-07-21
*/

#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include "serial.h"


#if defined ( __AVR_ATmega16__ ) \
	|| defined( __AVR_ATmega16A__ )

#define SERIAL_PARITYBIT_NONE 0
#define SERIAL_PARITYBIT_EVEN 2
#define SERIAL_PARITYBIT_ODD  3
#define __SERIAL_PARITYBIT_VALUE(v) (SERIAL_PARITYBIT_##v)
#define SERIAL_PARITYBIT_VALUE(v) (__SERIAL_PARITYBIT_VALUE(v))

#ifndef BAUD_TOL
#  define BAUD_TOL 2
#endif

#define UBRR_VALUE (((FCPU) + 8UL * (SERIAL_BAUD)) / (16UL * (SERIAL_BAUD)) -1UL)

#if 100 * (FCPU) > \
  	(16 * ((UBRR_VALUE) + 1)) * (100 * (SERIAL_BAUD) + (SERIAL_BAUD) * (BAUD_TOL))
  #define USE_2X 1
#elif 100 * (FCPU) < \
	(16 * ((UBRR_VALUE) + 1)) * (100 * (SERIAL_BAUD) - (SERIAL_BAUD) * (BAUD_TOL))
  #define USE_2X 1
#else
  #define USE_2X 0
#endif

#if USE_2X
  #undef UBRR_VALUE
  #define UBRR_VALUE (((FCPU) + 4UL * (SERIAL_BAUD)) / (8UL * (SERIAL_BAUD)) -1UL)

  #if 100 * (FCPU) > \
  (8 * ((UBRR_VALUE) + 1)) * (100 * (SERIAL_BAUD) + (SERIAL_BAUD) * (BAUD_TOL))
    #warning "Baud rate achieved is higher than allowed"
  #endif

  #if 100 * (FCPU) < \
	(8 * ((UBRR_VALUE) + 1)) * (100 * (SERIAL_BAUD) - (SERIAL_BAUD) * (BAUD_TOL))
  #warning "Baud rate achieved is lower than allowed"
  #endif
#endif

#ifdef UBRR_VALUE
  #define UBRRL_VALUE (UBRR_VALUE & 0xff)
  #define UBRRH_VALUE (UBRR_VALUE >> 8)
#endif

void serial_init( void ){
	uint8_t temp;

	UCSRA = 0x00;

	/* set baudrate */
	UBRRH = UBRRH_VALUE;
	UBRRL = UBRRL_VALUE;
	#if USE_2X
		UCSRA |= (1<<U2X);
	#else
		UCSRA &= ~(1<<U2X);
	#endif

	/* rx & tx enable */
	UCSRB = (1<<RXEN) | (1<<TXEN);

	/* set frame format */
	/* data bits */
	temp = 1<<URSEL;
	temp |= ((SERIAL_DATABIT)-5)<<UCSZ0;
	/* stop bits */
	temp |= ((SERIAL_STOPBIT)-1)<<USBS;
	/* parity bit */
	temp |= ((SERIAL_PARITYBIT_VALUE(SERIAL_PARITYBIT)))<<UPM0;
	UCSRC = temp;
}

void serial_putc( uint8_t dat ){
	while( !(UCSRA & (1<<UDRE)) );
	UDR = dat;
}

uint8_t serial_getc( void ){
	while( !(UCSRA & (1<<RXC)) );
	return UDR;
}

static int serial_stream_getc( FILE *fd ){
	return serial_getc( );
}

static int serial_stream_putc( char c, FILE *fd ){
	if( c == '\n' )
		serial_putc( '\r');
	serial_putc( c );
	
	return 1;
}

FILE serial_file = FDEV_SETUP_STREAM( 
		serial_stream_putc,
		serial_stream_getc,
		_FDEV_SETUP_RW );

#endif /*  __AVR_ATmega16__, __AVR_ATmega16A__ */

