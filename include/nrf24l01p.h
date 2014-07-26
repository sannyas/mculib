/**
* @file nrf24l01p.h
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

#ifndef _H_NRF24L01PLUS_
#define _H_NRF24L01PLUS_

#include <stdint.h>


/* PIPE number */
#define PIPE0			0
#define PIPE1			1
#define PIPE2			2
#define PIPE3			3
#define PIPE4			4
#define PIPE5			5
#define PIPE_NONE		7

/* Data rate */
#define NRF24L01P_DATA_RATE_250Kbps	1
#define NRF24L01P_DATA_RATE_1Mbps	2
#define NRF24L01P_DATA_RATE_2Mbps	3

/* RF output power */
#define NRF24L01P_RF_POWER_N18dBm	1
#define NRF24L01P_RF_POWER_N12dBm	2
#define NRF24L01P_RF_POWER_N6dBm	3
#define NRF24L01P_RF_POWER_0dBm		4

/**
 * @brief operatins
 * @{
 */

/*-----------------------------------------*/
/*	Initialize
 *-----------------------------------------*/
void 	nrf24l01p_init( void );

/*-----------------------------------------*/
/*	Configurations.
 *-----------------------------------------*/
void	nrf24l01p_enDynamicPayloadLen( uint8_t enable );
void	nrf24l01p_setChannel( uint8_t ch );
void	nrf24l01p_enableCRC( uint8_t en );
void	nrf24l01p_enAutoACK( uint8_t en );
void	nrf24l01p_setRetransmit( uint8_t delay,uint8_t count );
void	nrf24l01p_setDataRate( uint8_t rate );
void	nrf24l01p_setRFPower( uint8_t power );

/*-----------------------------------------*/
/*	Set address.
 *-----------------------------------------*/
void	nrf24l01p_setTxAddr( uint8_t *addr, uint8_t len );
void	nrf24l01p_setRxAddr( uint8_t *addr, uint8_t len, uint8_t pipe, uint8_t payload_len );

/*-----------------------------------------*/
/*	Select mode.
 *-----------------------------------------*/
void	nrf24l01p_enterTxMode( void );
void	nrf24l01p_enterRxMode( void );
void	nrf24l01p_standby( void );
void	nrf24l01p_powerDown( void );

/*-----------------------------------------*/
/*	Send and receive data.
 *-----------------------------------------*/
uint8_t	nrf24l01p_transmit( uint8_t *data, uint8_t len );
uint8_t nrf24l01p_dataPending( void );
uint8_t nrf24l01p_receive( uint8_t *data, uint8_t maxlen );

/* @} */


// For test.
void nrf24l01p_test( void );
void nrf24l01p_test_tx();
void nrf24l01p_test_rx();
void nrf24l01p_test_dpl_tx();
void nrf24l01p_test_dpl_rx();



/**
 * These functions is implementation-dependent, which should
 * be rewrite in different platform.
 *
 * @{
 */

/**
* @brief SPI command.
*
* @param cmd Command code.
* @param data_in Input data.
* @param data_out Output data.
* @param len The length of data.
*/
extern void nrf24l01p_doCommand( uint8_t cmd, uint8_t const *data_in, uint8_t *data_out, uint8_t len );

/**
 * @brief Set Chip Enable.
 * 
 * @param en	0 or 1.
 */
extern void nrf24l01p_chipEnable( uint8_t enable );


/**
 * @brief Platform-dependent initialization, about SPI interface.
 */
extern void nrf24l01p_platformInit( void );
extern void nrf24l01p_delay_us( uint16_t n );
extern void nrf24l01p_delay_ms( uint16_t n );

#endif

