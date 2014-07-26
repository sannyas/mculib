#include <avr/io.h>
#include <ioport.h>

void nrf24l01p_pin_setup(){
	iopin_outlow( NRF24L01_PIN_CE );
	iopin_outhigh( NRF24L01_PIN_CSN );
	iopin_outlow( NRF24L01_PIN_SCK );

	iopin_outmode( NRF24L01_PIN_CE );
	iopin_outmode( NRF24L01_PIN_CSN );
	iopin_outmode( NRF24L01_PIN_SCK );
	iopin_outmode( NRF24L01_PIN_MOSI );
	iopin_inmode ( NRF24L01_PIN_MISO );
}

void nrf24l01p_pin_CE_setValue( uint8_t v ){

	iopin_out( NRF24L01_PIN_CE, v );
}

void nrf24l01p_pin_CSN_setValue( uint8_t v ){

	iopin_out( NRF24L01_PIN_CSN, v );
}

void nrf24l01p_pin_SCLK_setValue( uint8_t v ){

	iopin_out( NRF24L01_PIN_SCK, v );
}

void nrf24l01p_pin_MOSI_setValue( uint8_t v ){

	iopin_out( NRF24L01_PIN_MOSI, v );
}

uint8_t nrf24l01p_pin_MISO_getValue( void ){

    return iopin_input( NRF24L01_PIN_MISO );
}

static uint8_t _spi_rw( uint8_t data ){
	uint8_t i;
	for( i=0; i<8; ++i ){
		nrf24l01p_pin_MOSI_setValue( data&0x80 );
		data = data << 1;
		nrf24l01p_pin_SCLK_setValue( 1 );
		if( nrf24l01p_pin_MISO_getValue( ) ){
			data |= 0x01;
		}
		nrf24l01p_pin_SCLK_setValue( 0 );
	}
	return data;
}

void nrf24l01p_doCommand( uint8_t cmd, uint8_t const *data_in, uint8_t *data_out, uint8_t len ){
	uint8_t idx;
	uint8_t	temp; 

	nrf24l01p_pin_CSN_setValue( 0 );
	_spi_rw( cmd );
	for( idx=0; idx<len; ++idx ){
		temp = data_in ? data_in[idx] : 0xff;
		temp = _spi_rw( temp );
		if( data_out )
			data_out[idx] = temp;
	}
	nrf24l01p_pin_CSN_setValue( 1 );
}

void nrf24l01p_chipEnable( uint8_t en ){
	if( en ){
		nrf24l01p_pin_CE_setValue( 1 );
	}
	else{
		nrf24l01p_pin_CE_setValue( 0 );
	}
}

void nrf24l01p_platformInit( void ){
	nrf24l01p_pin_setup( );
}

#include <util/delay.h>
void  nrf24l01p_delay_us( uint16_t n ){

	while( n-- > 0 ){
		_delay_us( 1 );	
	}
}

void nrf24l01p_delay_ms( uint16_t n ){
	while( n-- > 0 ){
		_delay_ms( 1 );
	}
}

