# NRF24L01+ Library

## Functions

Initialzation:
	void nrf24l01p_init( void );


Configurations:

	void nrf24l01p_enDynamicPayloadLen( uint8_t enable );
	void nrf24l01p_setChannel( uint8_t ch );
	void nrf24l01p_enableCRC( uint8_t en );
	void nrf24l01p_enAutoACK( uint8_t en );
	void nrf24l01p_setRetransmit( uint8_t delay,uint8_t count );
	void nrf24l01p_setDataRate( uint8_t rate );
	void nrf24l01p_setRFPower( uint8_t power );

Set send and receive address:

	void nrf24l01p_setTxAddr( uint8_t *addr, uint8_t len );
	void nrf24l01p_setRxAddr( uint8_t *addr, uint8_t len, uint8_t pipe, uint8_t payload_len );

Set mode:

	void nrf24l01p_enterTxMode( void );
	void nrf24l01p_enterRxMode( void );
	void nrf24l01p_standby( void );
	void nrf24l01p_powerDown( void );

receive and send data:

	uint8_t	nrf24l01p_transmit( uint8_t *data, uint8_t len );
	uint8_t nrf24l01p_dataPending( void );
	uint8_t nrf24l01p_receive( uint8_t *data, uint8_t maxlen );


Flow:

	+------------+                +-----------+       +----------+ 
	| initialize | -------------> | Configure | ----> | Set addr | --*
	+------------+   ^            +-----------+       +----------+   |
	                 |                                               |
	                 |                                               |
	                 |     *-------*                                 |
	                 |     |       |                                 |
	                 |     |       V                                 |
	                 |     |   +----------+      +---------+         |
	                 |     *<--| transmit | <--- | Tx Mode | <------/|
	      *----------*         +----------+      +---------+         |
	      |                                         |                |
	      |                                         |                |
	      |                  +----------------+     | +---------+    |
	      |               *--| Data pending ? | <---|-| Rx Mode | <--/
	      |           Yes |  +----------------+   ^ | +---------+         
	      |               |     +---------+       | |      |    
	      |               *---> | receive | ------* |      |  
	      |                     +---------+         |      |   
	      |                                         |      |    
	      |   +----------------------+              |      |
	      *-<-| Standby or powerdown | <------------/------/
	          +----------------------+
	

