/*
 * udp.c
 *
 *  Created on: Nov 22, 2015
 *      Author: pekka
 */
#include "udp.h"
#include "lwip.h"
#include "lwip/udp.h"
#include "Configuration.h"
#include "PinDefinition.h"
#include "board_configuration.h"

boardConfig c;

void udp_echoserver_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, struct ip_addr *addr, u16_t port)
{

	unsigned char udp_out[100];
	struct pbuf *p2;


	unsigned char *udp_data = (unsigned char*)p->payload;
	uint16_t length;

	//Pet the watchdog at any packet received.
	Wd->pet();
	ledOn(1);
	if(udp_data[0] == 1 && config->configured)
	{
		//read request packet
		p2 = pbuf_alloc(PBUF_TRANSPORT,100,PBUF_POOL);
		if(p2==NULL)
		{
			pbuf_free(p);
			return;
		}

		length = config->generateMessage(udp_out,udp_data[1]);
		/* copy data to pbuf */
		pbuf_take(p2, (char *) udp_out, length);

		udp_sendto(upcb, p2,addr,c.remote_port);
		pbuf_free(p2);
		pbuf_free(p);
		return;

	}
	else if(udp_data[0] == 0 && config->configured)
	{
		//data packet
		length = config->readMessage(udp_data);
		pbuf_free(p);
		return;
	}

	else if(udp_data[0] == 2)
	{
		//configuration packet
		config->parseConfig(udp_data, 10);
		ledOn(0);
		pbuf_free(p);
		return;

	}
	else if(udp_data[0] == 3)
	{
		//This is a board configuration packet, containing information about IP address, gateway, UDP ports etc.
		//The packet should contain the struct defined in the board_configuration.h
		setConfig(&c, udp_data);
		udp_out[0] = 'a';
		udp_out[1] = 'c';
		udp_out[2] = 'k';
		p2 = pbuf_alloc(PBUF_TRANSPORT,100,PBUF_POOL);
		if(p2==NULL)
		{
			pbuf_free(p);
			return;
		}
		pbuf_take(p2,(char *) udp_out, 3);
		udp_sendto(upcb, p2,addr,c.remote_port);
		pbuf_free(p2);
		pbuf_free(p);
		return;
	}
	else if(!config->configured)
	{
		//not configured and non-configuration packet received - requesting for configuration packet.
		udp_out[0] = 4;

		p2 = pbuf_alloc(PBUF_TRANSPORT,100,PBUF_POOL);
		if(p2==NULL)
		{
			pbuf_free(p);
			return;
		}

		pbuf_take(p2,(char *) udp_out, 1);
		udp_sendto(upcb, p2,addr,c.remote_port);
		pbuf_free(p2);
		pbuf_free(p);
		return;

	}


}

void udp_echoserver_init(boardConfig *conf)
{
	c=*conf;

	struct udp_pcb *upcb;
	err_t err;

	/* Create a new UDP control block  */
	upcb = udp_new();

	if (upcb)
	{
		/* Bind the upcb to the UDP_PORT port */
		/* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
		err = udp_bind(upcb, IP_ADDR_ANY, conf->local_port);

		if(err == ERR_OK)
		{
			/* Set a receive callback for the upcb */
			udp_recv(upcb, udp_echoserver_receive_callback, NULL);
		}
	}
}
