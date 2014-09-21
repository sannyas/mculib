/**
* @file debug.h
* @brief Debug for AVR
* @author Liao MY
* @date 2014-07-21
*/

#ifndef _DEBUG_H_
#define _DEBUG_H_

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

#define DEBUG_FUNC_ENTER_MSG	"--> "
#define DEBUG_FUNC_EXIT_MSG	"<-- "
#ifdef AVR_DEBUG
  extern uint8_t func_call_chain_cnt;
  #ifndef DEBUG_OUTPUT
    /* If don't define PDEBUG, define a default empty PDEBUG */
    #define DEBUG_OUTPUT(fmt,args...) printf(fmt,##args)
  #endif

  #define PDEBUG_SPACE() do{ \
	  		uint8_t __deep_cnt; \
	  		for( __deep_cnt=0; \
				__deep_cnt<func_call_chain_cnt; \
				++__deep_cnt ){ \
				DEBUG_OUTPUT( "  " );} \
	 		}while( 0 )
  #define PDEBUG(fmt,args...)  do{ PDEBUG_SPACE(); DEBUG_OUTPUT(fmt,##args); }while(0)

  #define DEBUG_FUNC_ENTER() do{ PDEBUG( DEBUG_FUNC_ENTER_MSG "%s\n", __FUNCTION__ );\
	  			 ++func_call_chain_cnt; \
 			     }while(0)
  #define DEBUG_FUNC_EXIT() do{ PDEBUG( DEBUG_FUNC_EXIT_MSG "\n");\
	  			--func_call_chain_cnt; \
  			    }while(0)

#else
  #undef PDEBUG
  #define DEBUG_INIT()		uint8_t func_call_chain_cnt
  #define PDEBUG(fmt,args...)  do{}while(0)
  #undef DEBUG_OUTPUT
  #define DEBUG_OUTPUT(fmt,args...) do{}while(0)
  #define DEBUG_FUNC_ENTER()  do{}while(0)
  #define DEBUG_FUNC_EXIT() do{}while(0)
#endif

#endif /* _DEBUG_H_ */
