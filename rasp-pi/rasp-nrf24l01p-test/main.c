#include <stdint.h>
#include <stdio.h>
#include <nrf24l01p.h>
#include <string.h>
#include <pthread.h>

pthread_t tid;
pthread_mutex_t nrf_lock;

void *thread_fun( void *data );

int main( void ){
	char buff[200];
	int idx = 0;
	uint8_t addr[5] = { 0x19,0x92,0x02,0x17,0x01 };
	int len;
	
	//nrf24l01p_test_rx();

	printf( "main start.\n" );
	pthread_mutex_init( &nrf_lock, NULL );

	nrf24l01p_init();
	nrf24l01p_enableDynamicPayloadLen( 1 );
	nrf24l01p_setTxAddr( addr, 5 );
	nrf24l01p_setRxAddr( addr, 5, NRF24L01P_PIPE1, 0 );
	nrf24l01p_enterRxMode();
	pthread_create( &tid, NULL, thread_fun, NULL );

	while( 1 ){
		fgets( buff, 100, stdin );
		if( buff[strlen( buff )] == '\0' )
			buff[ strlen(buff)-1] = 0;
		len = strlen( buff );
		printf( "[%d] Send data: %s\n", idx, buff );
		idx ++;
		
		pthread_mutex_lock( &nrf_lock );
		nrf24l01p_standby();
		nrf24l01p_enterTxMode();
		while( len > 0 ){
			if( len > 32 ){
				nrf24l01p_transmit( buff, 32 );
				len -= 32;
			}
			else{
				nrf24l01p_transmit( buff, len );
				len =0;
			}
		}
		nrf24l01p_standby();
		nrf24l01p_enterRxMode();
		pthread_mutex_unlock( &nrf_lock );
				
	}	


	printf( "main end.\n" );	

	while( 1 ){
	}
	return 0;
}

void *thread_fun( void *data ){
	int pipe;
	int len;
	uint8_t buff[300];

	while( 1 ){
		pthread_mutex_lock( &nrf_lock );
		if( (pipe=nrf24l01p_dataPending() ) != NRF24L01P_PIPE_NONE ){
			len = nrf24l01p_receive( buff, 32 );
			buff[len] =0;
			printf( "recv %d: %s\n", len, buff );
		}
		pthread_mutex_unlock( &nrf_lock );
	}
}

