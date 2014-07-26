#include <stdint.h>
#include <stdio.h>
#include <nrf24l01p.h>


int main( void ){

	printf( "main start.\n" );

	//nrf24l01p_test_rx();
	nrf24l01p_test_dpl_rx();

	printf( "main end.\n" );	

	while( 1 ){
	}
	return 0;
}
