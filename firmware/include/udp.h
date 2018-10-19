/*
 * udp.h
 *
 *  Created on: Nov 22, 2015
 *      Author: pekka
 */

#ifndef UDP_H_
#define UDP_H_
#include "Configuration.h"
#include "Watchdog.h"
#include "board_configuration.h"
extern Configuration *config;
extern Watchdog *Wd;
extern "C" boardConfig c;
extern "C" void setConfig(boardConfig *config, unsigned char *udp_packet);
void udp_echoserver_init(boardConfig *conf);

#endif /* UDP_H_ */
