#include <stdint.h>
#include <stdio.h>
#include <util/delay.h>
#include <serial.h>
#include <nrf24l01p.h>

int main( void ){

	// init uart
	stdout = &serial_file;
	serial_init();

	printf( "main start.\n" );

	nrf24l01p_test_dpl_tx();

	printf( "main end.\n" );	

	while( 1 ){
	}
	return 0;
}
