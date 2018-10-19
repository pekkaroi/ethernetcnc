/*
 	bldc-drive Cheap and simple brushless DC motor driver designed for CNC applications using STM32 microcontroller.
	Copyright (C) 2015 Pekka Roivainen

	This program is free software; you can redistribute it and/or
	modify it under the terms of the GNU General Public License
	as published by the Free Software Foundation; either version 2
	of the License, or (at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/


#ifndef BOARD_CONFIGURATION_H_
#define BOARD_CONFIGURATION_H_

#include <stdint.h>
#include "lwip/ip_addr.h"
#include "board_configuration_def.h"



void getConfig(boardConfig *config);
//void setConfig(boardConfig *config, unsigned char* udp_packet);
void writeConfig(boardConfig *config);

#endif /* BOARD_CONFIGURATION_H_ */
