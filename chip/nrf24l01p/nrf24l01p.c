/**
* @file nrf24l01p.c
* @brief Driver for NRF24L01+
* @author Liao MY
* @date 2014-07-25
*/

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

#include <stdio.h>
#include "nrf24l01p.h"


#ifdef NRF24L01P_DEBUG
 #define DEBUG_OUTPUT(fmt,args...)	printf( fmt,##args );
#else
 #define DEBUG_OUTPUT(fmt,args...)
#endif


/*-------------------------------------------*/
/*	Command
 *-------------------------------------------*/

/**
 * @brief SPI Commands.
 *
 * Every new commands must be started by a high to low transition
 * on CSN. The serial shifting SPI commands is in the following 
 * format:
 * 	<Command word>MSBit to LSBit; <Data bytes>LSByte to MSByte.
 * @{
 */
#define R_REGISTER(reg)		(reg)		// 5-bit addr
#define W_REGISTER(reg)		(0x20|(reg))	// 5-bit addr
#define R_RX_PAYLOAD		(0x61)
#define W_TX_PAYLOAD		(0xa0)
#define FLUSH_TX		(0xe1)
#define FLUSH_RX		(0xe2)
#define REUSE_TX_PL		(0xe3)
#define R_RX_PL_WID		(0x60)
#define W_ACK_PAYLOAD(pipe)	(0xa8|(pipe))
#define W_TX_PAYLOAD_NO_ACK	(0xb0)
#define NOP			(0xff)
/*@}*/


/*-------------------------------------------*/
/*	Registers
 *-------------------------------------------*/

/**
 * @brief Register map table.
 * @{
 */
#define CONFIG			0x00
#define EN_AA			0x01 // Enable auto ack.
#define EN_RXADDR		0x02 // Enable RX data pipe.
#define SETUP_AW		0x03 // Set addr width.
#define SETUP_RETR		0x04 // Setup of auto retransmission.
#define RF_CH			0x05 // RF channel.
#define RF_SETUP		0x06 // RF setup reg.
#define STATUS			0x07 // Status reg.
#define OBSERVE_TX		0x08
#define RPD			0x09 // Received power detector.
#define RX_ADDR_P0		0x0a 
#define RX_ADDR_P1		0x0b
#define RX_ADDR_P2		0x0c
#define RX_ADDR_P3		0x0d
#define RX_ADDR_P4		0x0e
#define RX_ADDR_P5		0x0f
#define TX_ADDR			0x10
#define RX_PW_P0		0x11
#define RX_PW_P1		0x12
#define RX_PW_P2		0x13
#define RX_PW_P3		0x14
#define RX_PW_P4		0x15
#define RX_PW_P5		0x16
#define FIFO_STATUS		0x17
#define DYNPD			0x1c // Enable dynamic payload len.
#define FEATURE			0x1d // Feature reg.
/* @} */


/*-------------------------------------------*/
/*	Bits of registers
 *-------------------------------------------*/

/**
 * @brief Bits in register.
 * @{
 */
/* configuratio nregister */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0

/* enable auto acknowledgment */
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0

/* enable rx addresses */
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0

/* setup of address width */
#define AW          0 /* 2 bits */

/* setup of auto re-transmission */
#define ARD         4 /* 4 bits */
#define ARC         0 /* 4 bits */

/* RF setup register */
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      1 /* 2 bits */   

/* general status register */
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1 /* 3 bits */
#define TX_FULL     0

/* transmit observe register */
#define PLOS_CNT    4 /* 4 bits */
#define ARC_CNT     0 /* 4 bits */

/* fifo status */
#define TX_REUSE    6
#define FIFO_TX_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0

/* dynamic length */
#define DPL_P0      0
#define DPL_P1      1
#define DPL_P2      2
#define DPL_P3      3
#define DPL_P4      4
#define DPL_P5      5

/* feature */
#define EN_DPL		2	
#define EN_ACK_PAY	1
#define EN_DNY_ACK	0
/* @} */


/*-------------------------------------------*/
/*	Some values of registers
 *-------------------------------------------*/

#define CONFIG_DEFAULT ((1<<EN_CRC)|(0<<CRCO) \
			|(1<<MASK_RX_DR)|(1<<MASK_TX_DS)|(1<<MASK_MAX_RT))

// Address width value.
#define AW_3			1
#define AW_4			2
#define AW_5			3

// Auto retransmission value.
#define AUTO_RETRANS_DELAY(n)	((n-1)<<4)	// unit 250us
#define AUTO_RETRANS_COUNT(n)	(n)

// FIFO status text.
#define fifo_tx_full(status)	(!!(status&(1<<FIFO_TX_FULL)))
#define fifo_tx_empty(status)	(!!(status&(1<<TX_EMPTY)))
#define fifo_rx_full(status)	(!!(status&(1<RX_FULL)))
#define fifo_rx_empty(status)	(status&(1<<RX_EMPTY))

/*-------------------------------------------*/
/*	Static data
 *-------------------------------------------*/
#define NRF24L01P_CONF_AUTO_ACK			(1<<0)
#define NRF24L01P_CONF_STATIC_PAYLOAD_LEN	(1<<1)
//#define NRF24L01P_CONF_CRC			(1<<2)

static struct struct_nrf24l01p_config{
	uint8_t config;
	uint8_t others;
} nrf24l01p_conf;



/*-------------------------------------------*/
/*	Low-level static functions
 *-------------------------------------------*/
static void nrf24l01p_writeReg( uint8_t reg, uint8_t *data, uint8_t len ){
	nrf24l01p_doCommand( W_REGISTER(reg), data, 0,  len );
}


static void nrf24l01p_singleWriteReg( uint8_t reg, uint8_t data ){
	nrf24l01p_doCommand( W_REGISTER(reg), &data, 0, 1 );
}

static void nrf24l01p_readReg( uint8_t reg, uint8_t *data, uint8_t len ){
	nrf24l01p_doCommand( R_REGISTER(reg), 0, data, len );
}

static uint8_t nrf24l01p_singleReadReg( uint8_t reg ){
	uint8_t data;
	nrf24l01p_doCommand( R_REGISTER(reg), 0, &data, 1 );
	return data;
}


/*-------------------------------------------*/
/*	API implemets
 *-------------------------------------------*/
void nrf24l01p_defaultConfig( void );


/**
* @brief Initialize chip.
*/
void nrf24l01p_init( void ){
	nrf24l01p_platformInit();	
	nrf24l01p_powerDown();
	nrf24l01p_defaultConfig();
}

/**
 * @brief Set to default configuration.
 * 
 * The default configurations are:
 *	CRC			enable, 8-bit
 * 	Auto ACK		enable
 * 	Payload len		static
 *	Rx pipe			All disable
 *	Address width		5
 *	Auto retransmit delay	250us
 *	Audo retransmit count	3
 *	RF channel		2
 *	Data rate		2Mbps
 *	RF output power		0dBm
 *	
 */
void nrf24l01p_defaultConfig( void ){ // TODO

	nrf24l01p_doCommand( FLUSH_RX, 0, 0, 0 );
	nrf24l01p_doCommand( FLUSH_TX, 0, 0, 0 );
	nrf24l01p_singleWriteReg( STATUS, 0xff );


	/**********************/
	nrf24l01p_singleWriteReg( EN_AA, 0x3f ); // enable all AUTO ACK.
	nrf24l01p_singleWriteReg( EN_RXADDR, 0x00 ); // disable all RX addr.
	nrf24l01p_singleWriteReg( SETUP_AW, 3 ); // address width: 5
	nrf24l01p_singleWriteReg( SETUP_RETR, 0x03 );
	nrf24l01p_singleWriteReg( RF_CH, 0x02 );
	nrf24l01p_singleWriteReg( RF_SETUP, 0x07 );

	/* disable dynamic payload len. */
	nrf24l01p_singleWriteReg( FEATURE, 0x00 ); 
	nrf24l01p_singleWriteReg( DYNPD, 0x00 );

	nrf24l01p_conf.others = (
			 NRF24L01P_CONF_AUTO_ACK 
			|NRF24L01P_CONF_STATIC_PAYLOAD_LEN
			);
	nrf24l01p_conf.config = CONFIG_DEFAULT;
}

void nrf24l01p_enableDynamicPayloadLen( uint8_t enable ){
	nrf24l01p_singleWriteReg( DYNPD, 0x00 );
	if( enable ){
		nrf24l01p_singleWriteReg( FEATURE, (1<<EN_DPL) );
		nrf24l01p_conf.others &= ~(NRF24L01P_CONF_STATIC_PAYLOAD_LEN);
	}
	else{
		nrf24l01p_singleWriteReg( FEATURE, 0x00 );
		nrf24l01p_conf.others |= NRF24L01P_CONF_STATIC_PAYLOAD_LEN;
	}
}

void nrf24l01p_setChannel( uint8_t ch ){
	nrf24l01p_singleWriteReg( RF_CH, ch );
}

void nrf24l01p_enableCRC( uint8_t en ){
	if( en ){
		nrf24l01p_conf.config |= EN_CRC;
	}
	else{
		nrf24l01p_conf.config &= ~EN_CRC;
	}
}

void nrf24l01p_enableAutoACK( uint8_t en ){
	if( en ){
		nrf24l01p_singleWriteReg( EN_AA, 0x3f );
		nrf24l01p_conf.others |= NRF24L01P_CONF_AUTO_ACK;
	}
	else{
		nrf24l01p_singleWriteReg( EN_AA, 0x00 );
		nrf24l01p_conf.others &= ~NRF24L01P_CONF_AUTO_ACK;
	}
}

void nrf24l01p_setRetransmit( uint8_t delay,uint8_t count ){
	
	nrf24l01p_singleWriteReg( SETUP_RETR, (delay<<4) | count );
}

void nrf24l01p_setDataRate( uint8_t rate ){
	uint8_t rf_setup;

	rf_setup = nrf24l01p_singleReadReg( RF_SETUP );
	rf_setup &= 0x07; // clear high 5 bits
	if( rate == NRF24L01P_DATA_RATE_250Kbps ){
		rf_setup |= 0x20;
	}
	else if( rate == NRF24L01P_DATA_RATE_1Mbps ){
		rf_setup |= 0x00;
	}
	else if( rate == NRF24L01P_DATA_RATE_2Mbps ){
		rf_setup |= 0x08;
	}
	
	nrf24l01p_singleWriteReg( RF_SETUP, rf_setup );
}

void nrf24l01p_setRFPower( uint8_t power ){
	uint8_t rf_setup;

	rf_setup = nrf24l01p_singleReadReg( RF_SETUP );
	rf_setup &= 0xf8; // clear low 3 bits
	if( power == NRF24L01P_RF_POWER_N18dBm )
		rf_setup |= 0x00;
	else if( power == NRF24L01P_RF_POWER_N12dBm )
		rf_setup |= 0x02;
	else if( power == NRF24L01P_RF_POWER_N6dBm )
		rf_setup |= 0x04;
	else if( power == NRF24L01P_RF_POWER_0dBm )
		rf_setup |= 0x06;

	nrf24l01p_singleWriteReg( RF_SETUP, rf_setup );
}

void nrf24l01p_enableInterupt( uint8_t inte, uint8_t enable ){

	if( inte & NRF24L01P_INT_RX_DR )
		nrf24l01p_conf.config &= ~(1<<MASK_RX_DR);
	else
		nrf24l01p_conf.config |= (1<<MASK_RX_DR);

	if( inte & NRF24L01P_INT_TX_DS )
		nrf24l01p_conf.config &= ~(1<<MASK_TX_DS);
	else
		nrf24l01p_conf.config |= (1<<MASK_TX_DS);

	if( inte & NRF24L01P_INT_MAX_RT )
		nrf24l01p_conf.config &= ~(1<<MASK_MAX_RT);
	else
		nrf24l01p_conf.config |= (1<<MASK_MAX_RT);
}

uint8_t nrf24l01p_getInterruptType( void ){
	uint8_t status = nrf24l01p_singleReadReg( STATUS );
	uint8_t int_type = 0;
	if( status & (1<<RX_DR) )
		int_type |= NRF24L01P_INT_RX_DR;
	if( status & (1<<TX_DS) )
		int_type |= NRF24L01P_INT_TX_DS;
	if( status & (1<<MAX_RT) )
		int_type |= NRF24L01P_INT_MAX_RT;
	return int_type;
}

void nrf24l01p_clearInterrupt( uint8_t inte ){
	uint8_t int_type = 0;

	if( inte & NRF24L01P_INT_RX_DR )
		int_type |= (1<<RX_DR);
	if( inte & NRF24L01P_INT_TX_DS )
		int_type |= (1<<TX_DS);
	if( inte & NRF24L01P_INT_MAX_RT )
		int_type |= (1<<MAX_RT);
	
	nrf24l01p_singleWriteReg( STATUS, int_type );

}

void nrf24l01p_setTxAddr( uint8_t *addr, uint8_t len ){
	/* if auto ack is enabled, must enable rx pipe 0. */
	if( nrf24l01p_conf.others & NRF24L01P_CONF_AUTO_ACK ){
		uint8_t en_rxaddr = nrf24l01p_singleReadReg( EN_RXADDR );
		en_rxaddr |= 0x01;
		nrf24l01p_singleWriteReg( EN_RXADDR, en_rxaddr );
		nrf24l01p_writeReg( RX_ADDR_P0, addr, len );
	}

	/* If dynamic payload len is enabled, DPL_P0 must be set. But why ?*/
	if( !(nrf24l01p_conf.others & NRF24L01P_CONF_STATIC_PAYLOAD_LEN) ){
		nrf24l01p_singleWriteReg( DYNPD, 0x01 );
	}

	nrf24l01p_writeReg( TX_ADDR, addr, 5 );
	/* TODO if using static payload len, this is unnecessary */
}

void nrf24l01p_setRxAddr( uint8_t *addr, uint8_t len, uint8_t pipe, uint8_t payload_len ){
	uint8_t val;

	val = nrf24l01p_singleReadReg( EN_RXADDR );
	if( addr == NULL ){
		val &= ~(1<<pipe);
		nrf24l01p_singleWriteReg( EN_RXADDR, val );
	}
		
	else{
		/* enable rx pipe */
		val |= (1<<pipe);
		nrf24l01p_singleWriteReg( EN_RXADDR, val );

		/* Static or dynamic payload len */
		if( nrf24l01p_conf.others & NRF24L01P_CONF_STATIC_PAYLOAD_LEN ){
			nrf24l01p_singleWriteReg( RX_PW_P0+pipe, payload_len );
		}
		else{
			val = nrf24l01p_singleReadReg( DYNPD );
			val |= (1<<pipe);
			nrf24l01p_singleWriteReg( DYNPD, val );
		}

		if( pipe >= 2 ){
			nrf24l01p_writeReg( RX_ADDR_P0+pipe, addr, 1 );
		}
		else{
			/* Set address width. */
			nrf24l01p_singleWriteReg( SETUP_AW, len-2 );
			nrf24l01p_writeReg( RX_ADDR_P0+pipe, addr, len );
		}
	}
}

void nrf24l01p_enterTxMode( void ){
	uint8_t config;

	nrf24l01p_chipEnable( 0 );
	nrf24l01p_doCommand( FLUSH_TX, 0, 0, 0 );
	nrf24l01p_doCommand( FLUSH_RX, 0, 0, 0 );
	nrf24l01p_singleWriteReg( STATUS, 0xff );

	//nrf24l01p_singleWriteReg( CONFIG, 0x06 );
	DEBUG_OUTPUT( "enter TxMode, config=0x%x\n", 
		nrf24l01p_conf.config|(1<<PWR_UP) );
	nrf24l01p_singleWriteReg( CONFIG, 
		nrf24l01p_conf.config|(1<<PWR_UP) );
	nrf24l01p_chipEnable( 1 );
}

void nrf24l01p_enterRxMode( void ){
	uint8_t config;

	nrf24l01p_chipEnable( 0 );
	nrf24l01p_doCommand( FLUSH_TX, 0, 0, 0 );
	nrf24l01p_doCommand( FLUSH_RX, 0, 0, 0 );
	nrf24l01p_singleWriteReg( STATUS, 0xff );

	//nrf24l01p_singleWriteReg( CONFIG, 0x07 );
	DEBUG_OUTPUT( "enter RxMode, config=0x%x\n", 
		nrf24l01p_conf.config|(1<<PRIM_RX)|(1<<PWR_UP));
	nrf24l01p_singleWriteReg( CONFIG, 
		nrf24l01p_conf.config|(1<<PRIM_RX)|(1<<PWR_UP) );
	nrf24l01p_chipEnable( 1 );
}

void nrf24l01p_standby( void ){
	nrf24l01p_chipEnable( 0 );
	//nrf24l01p_singleWriteReg( CONFIG, CONFIG_DEFAULT );
}

void nrf24l01p_powerDown( void ){
	nrf24l01p_chipEnable( 0 );
	nrf24l01p_singleWriteReg( CONFIG, CONFIG_DEFAULT ); 
}

uint8_t	nrf24l01p_transmit( uint8_t *data, uint8_t len ){

	uint8_t status;
	uint8_t ret;

	/* Check fifo full */
	status = nrf24l01p_singleReadReg( FIFO_STATUS );
	if( fifo_tx_full( status ) ){
		DEBUG_OUTPUT( "transmit error: fifo full\n" );
		return 0;
	}

	nrf24l01p_doCommand( W_TX_PAYLOAD, data, 0, len );
	if( nrf24l01p_conf.config & (1<<MASK_TX_DS) ){ // Interrupt disabled
		while( 1 ){
			status = nrf24l01p_singleReadReg( STATUS );
			if( status & (1<<TX_DS) ){
				ret = 1;
				break;
			}
			else if( status & (1<<MAX_RT) ){
				DEBUG_OUTPUT( "transmit error: timeout\r\n" );
				ret = 0;
				break;
			}
		}
	}
	
	//nrf24l01p_singleWriteReg( STATUS, 0xff );
	return ret;
}

uint8_t nrf24l01p_dataPending( ){
	uint8_t status;
	uint8_t fifo_status;
	uint8_t pipeno = NRF24L01P_PIPE_NONE;

	status = nrf24l01p_singleReadReg( STATUS );
	fifo_status = nrf24l01p_singleReadReg( FIFO_STATUS );
	if( status & (1<<RX_DR) ){
		nrf24l01p_singleWriteReg( STATUS, 0xff );
		pipeno = (status&0x0e)>>1;
		DEBUG_OUTPUT( "jjjj" );
	}
	else if( !fifo_rx_empty( fifo_status ) ){
		nrf24l01p_singleWriteReg( STATUS, 0xff );
		pipeno = (status&0x0e)>>1; // TODO Is this right pipe number ??
	}
	
	return pipeno;
}

uint8_t nrf24l01p_receive( uint8_t *data, uint8_t maxlen ){
	uint8_t fifo_status;
	uint8_t len;


	fifo_status = nrf24l01p_singleReadReg( FIFO_STATUS );	
	if( fifo_rx_empty( fifo_status ) ){
		return 0;
	}
	// TODO len
	nrf24l01p_doCommand( R_RX_PL_WID, 0, &len, 1 );
	if( maxlen < len ){
		DEBUG_OUTPUT( "revc buff not enough.\n" );
		len = maxlen;
	}
	nrf24l01p_doCommand( R_RX_PAYLOAD, 0, data, len );
	nrf24l01p_singleWriteReg( STATUS, 0xff );
	return len;

}




/************************************************
 *	Test
 ************************************************/
/**
 * @brief Test SPI read and write functions.
 */
void nrf24l01p_test_rw( void ){
	uint8_t data[5] = { 1,2,3,4,5 };

	DEBUG_OUTPUT( "write data (0x%x,0x%x,0x%x,0x%x,0x%x) to TX_ADDR\n",
			data[0], data[1], data[2], data[3], data[4] );
	nrf24l01p_doCommand( W_REGISTER(TX_ADDR), data, 0, 5 );
	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;
	data[4] = 0;
	nrf24l01p_doCommand( R_REGISTER(TX_ADDR), 0, data, 5 );
	DEBUG_OUTPUT( "Result is 0x%x,0x%x,0x%x,0x%x,0x%x\n:", 
			data[0], data[1], data[2], data[3], data[4] );
}

void nrf24l01p_test_tx( void ){
	uint8_t data[4] = { 0x0f,0xf0,0xaa,0 };
	uint8_t addr_p0[5] = {0xe7,0xe7,0xe7,0xe7,0xe7};
	uint8_t addr_p1[5] = {0xc2,0xc2,0xc2,0xc2,0xc2};
	uint8_t addr_p2[5] = {0xc3,0xc2,0xc2,0xc2,0xc2};
	uint8_t addr_p3[5] = {0xc4,0xc2,0xc2,0xc2,0xc2};
	uint8_t addr_p4[5] = {0xc5,0xc2,0xc2,0xc2,0xc2};
	uint8_t addr_p5[5] = {0xc6,0xc2,0xc2,0xc2,0xc2};
	uint8_t *addrs[6];
	uint8_t pipe = 0;

	addrs[0] = addr_p0;
	addrs[1] = addr_p1;
	addrs[2] = addr_p2;
	addrs[3] = addr_p3;
	addrs[4] = addr_p4;
	addrs[5] = addr_p5;

	//uint8_t addr_p0[5] = {0x11,0x11,0x11,0x11,0x11};
	//uint8_t addr_p1[5] = {0x11,0x11,0x11,0x11,0x12};


	nrf24l01p_init();
	nrf24l01p_test_rw();
	nrf24l01p_setTxAddr( addr_p1, 5 );
	nrf24l01p_enterTxMode();

	while( 1 ){
		//nrf24l01p_standby();
		//nrf24l01p_setTxAddr( addrs[pipe], 5 );
		//if( ++pipe>5 ) pipe = 0;
		//nrf24l01p_enterTxMode();

		DEBUG_OUTPUT( "send data: 0x%x,0x%x,0x%x,0x%x\n",
			data[0],data[1],data[2],data[3] );

		if( nrf24l01p_transmit( data, 4 ) ){
			DEBUG_OUTPUT( "send ok\n" );
			data[3] ++;
		}
		else{
			DEBUG_OUTPUT( "send falure\n" );
		}
		nrf24l01p_delay_ms( 500 );
	}
}

void nrf24l01p_test_rx( void ){
	uint8_t data[4];
	uint8_t addr_p0[5] = {0xe7,0xe7,0xe7,0xe7,0xe7};
	uint8_t addr_p1[5] = {0xc2,0xc2,0xc2,0xc2,0xc2};
	uint8_t addr_p2[1] = {0xc3};
	uint8_t addr_p3[1] = {0xc4};
	uint8_t addr_p4[1] = {0xc5};
	uint8_t addr_p5[1] = {0xc6};
	
	
	//uint8_t addr_p0[5] = {0x11,0x11,0x11,0x11,0x11};
	//uint8_t addr_p1[5] = {0x11,0x11,0x11,0x11,0x12};
	uint8_t len;
	uint8_t pipe;

	nrf24l01p_init();
	nrf24l01p_test_rw();

	nrf24l01p_setRxAddr( addr_p0, 5, NRF24L01P_PIPE0, 4 );
	nrf24l01p_setRxAddr( addr_p1, 5, NRF24L01P_PIPE1, 4 );
	nrf24l01p_setRxAddr( addr_p2, 1, NRF24L01P_PIPE2, 4 );
	nrf24l01p_setRxAddr( addr_p3, 1, NRF24L01P_PIPE3, 4 );
	nrf24l01p_setRxAddr( addr_p4, 1, NRF24L01P_PIPE4, 4 );
	nrf24l01p_setRxAddr( addr_p5, 1, NRF24L01P_PIPE5, 4 );

	nrf24l01p_enterRxMode();

	while( 1 ){
		if( (pipe=nrf24l01p_dataPending()) != NRF24L01P_PIPE_NONE ){
			len = nrf24l01p_receive( data, 4 );
			if( len == 0 ){
				DEBUG_OUTPUT( "error, no data\n" );
			}else
				DEBUG_OUTPUT( "pipe %u data: 0x%x,0x%x,0x%x,0x%x\n", 
					pipe, data[0],data[1],data[2],data[3] );
		}
	}
}

void nrf24l01p_test_dpl_tx( void ){
	uint8_t data[6] = { 0x00,0x00,0x00,0x00,0x00,0x00 };
	uint8_t pipe = 5;
	uint8_t len = 0;

	nrf24l01p_init();
	nrf24l01p_enableDynamicPayloadLen( 1 );

	while( 1 ){
		DEBUG_OUTPUT( "send data to pipe %d\n", pipe );

		if( ++len == 7 ) len = 1;
		if( nrf24l01p_transmit( data, len ) ){
			DEBUG_OUTPUT( "send ok\n" );
		}
		else{
			DEBUG_OUTPUT( "send falure\n" );
		}
		//nrf24l01p_delay_ms( 500 );
	}
}

void nrf24l01p_test_dpl_rx( void ){
	uint8_t data[10];
	uint8_t len;
	uint8_t pipe;
	uint8_t i;

	nrf24l01p_init();
	nrf24l01p_enableDynamicPayloadLen( 1 );
	nrf24l01p_enterRxMode();

	while( 1 ){
		if( (pipe=nrf24l01p_dataPending()) != NRF24L01P_PIPE_NONE ){
			len = nrf24l01p_receive( data, 10 );
			if( len == 0 ){
				DEBUG_OUTPUT( "error, no data\n" );
			}else{
				DEBUG_OUTPUT( "pipe %u data %u: ", pipe, len );
				for( i=0; i<len; ++i ){
					DEBUG_OUTPUT( "0x%x ", data[i] );	
				}
				DEBUG_OUTPUT( "\n" );
			}
		}
	}
}
void nrf24l01p_test( void ){
	nrf24l01p_init();
	nrf24l01p_test_rw();
}
