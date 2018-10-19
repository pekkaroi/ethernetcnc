
#ifndef BOARD_CONFIGURATION_DEF_H_
#define BOARD_CONFIGURATION_DEF_H_

/* The struct is defined in a separate file, so that it can be included
*  to PC app that sends the config to the board
*/

//packed struct so that it can be nicely saved to Flash and sent via UDP.
typedef struct   __attribute__ ((__packed__)) {
	uint16_t useDHCP;
	struct ip_addr ipaddr;
	struct ip_addr netmask;
	struct ip_addr gw;
	uint16_t local_port;
	uint16_t remote_port;

} boardConfig;


#endif
