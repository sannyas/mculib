/**
* @file serial.h
* @brief 
* @author Liao MY
* @date 2014-07-21
*/

#ifndef _USART_HDR_
#define _USART_HDR_

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


#include <stdint.h>
#include <stdio.h>


/**
* @brief The FILE object used by avr-libc
*/
extern FILE serial_file;

/**
 * @brief Initialize serial.
 */
void serial_init( void );

/**
* @brief Output a charactor.
* @parm data The data to be sent.
*/
void serial_putc( uint8_t data );

/**
* @brief Get a charactor.
* @ret The received data.
*/
uint8_t	serial_getc( void );

#endif
