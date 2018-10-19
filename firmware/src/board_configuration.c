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


#include "board_configuration.h"
#include "eeprom.h"
#include <string.h>
#include <stdio.h>
#define EADDR_IS_INITIALIZED 0x0001
#define EADDR_CONFIG_START 0x0002

void writeConfig(boardConfig *config)
{

	FLASH_Unlock();


	uint16_t i;
	int16_t *ptr = (int16_t *)config;
	for(i=0; i<sizeof(boardConfig)/2;i++)
	{
		if(EE_WriteVariable(EADDR_CONFIG_START+i, *ptr)!=FLASH_COMPLETE)
		{
			//todo: Rise some error flag? Led maybe yes?
			while(1)
			{
				//die to infinite loop?
			}
		}
		ptr++;
	}

	FLASH_Lock();

}
/*
void getConfig(boardConfig *config)
{
	FLASH_Unlock();

	uint16_t i;

	uint16_t *ptr;
	EE_Init();
	EE_ReadVariable(EADDR_IS_INITIALIZED,&i);
	struct ip_addr ip;

	if(i != 0x5252)
	{
		//empty or corrupted EEPROM detected: write default config
		//EE_Format();
		EE_WriteVariable(EADDR_IS_INITIALIZED, 0x5252);
		config->useDHCP=1;
		IP4_ADDR(&ip, 0,0,0,0);
		config->ipaddr = ip;
		config->gw = ip;
		config->netmask = ip;
		config->local_port = 20202;
		config->remote_port = 20202;
		writeConfig(config);
		return;

	}
	ptr = (uint16_t *)config;
	for(i=0; i<sizeof(boardConfig)/2;i++)
	{

		if(EE_ReadVariable(EADDR_CONFIG_START+i, ptr)!=0)
		{
			//TODO: error handling. Led?
			while(1)
						{
							//die to infinite loop?
						}

		}
		ptr++;
	}
	FLASH_Lock();
	return;

}
*/

void getConfig(boardConfig *config)
{
   // FLASH_Unlock();

    uint16_t i;

    uint16_t *ptr;
    EE_Init();
    EE_ReadVariable(EADDR_IS_INITIALIZED,&i);
    struct ip_addr ip;

//    if(i != 0x5252)
//    {
        //empty or corrupted EEPROM detected: write default config
        //EE_Format();
 //       EE_WriteVariable(EADDR_IS_INITIALIZED, 0x5252);
        config->useDHCP=0;
        IP4_ADDR(&ip, 192,168,0,11);
        config->ipaddr = ip;
        IP4_ADDR(&ip, 255,255,255,0);
        config->gw = ip;
        IP4_ADDR(&ip, 192,168,0,1);
        config->netmask = ip;
        config->local_port = 20202;
        config->remote_port = 20202;
//        writeConfig(config);
        return;

 //   }
  /*  ptr = (uint16_t *)config;
    for(i=0; i<sizeof(boardConfig)/2;i++)
    {

        if(EE_ReadVariable(EADDR_CONFIG_START+i, ptr)!=0)
        {
            //TODO: error handling. Led?
            while(1)
                        {
                            //die to infinite loop?
                        }

        }
        ptr++;
    }
    FLASH_Lock();*/
    return;

}

void setConfig(boardConfig *config, unsigned char *udp_packet)
{
	uint8_t *ptr = (uint8_t *)config;
	uint16_t i;

	udp_packet++; //first byte contains the packet type info.
	//Rest is directly the struct content:
	for(i=0; i<sizeof(boardConfig);i++)
	{

		*ptr = (uint8_t)(udp_packet[0]);
		udp_packet++;
		ptr++;
	}
	writeConfig(config);


}
