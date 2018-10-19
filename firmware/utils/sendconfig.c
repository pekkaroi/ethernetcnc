
;
/*
    Simple udp client
    Silver Moon (m00n.silv3r@gmail.com)
*/
#include<stdio.h> //printf
#include<string.h> //memset
#include<stdlib.h> //exit(0);
#include<arpa/inet.h>
#include<sys/socket.h>


#define SERVER "10.0.0.97"
#define BUFLEN 512  //Max length of buffer
#define PORT 20202   //The port on which to send data
 /** Set an IP address given by the four byte-parts.
    Little-endian version that prevents the use of htonl. */

#define IP4_ADDR(ipaddr, a,b,c,d) \
        (ipaddr)->addr = ((uint32_t)((d) & 0xff) << 24) | \
                         ((uint32_t)((c) & 0xff) << 16) | \
                         ((uint32_t)((b) & 0xff) << 8)  | \
                          (uint32_t)((a) & 0xff)

struct ip_addr {
  uint32_t addr;
};
/*
 typedef struct  {
	uint16_t useDHCP;
	struct ip_addr ipaddr;
	struct ip_addr netmask;
	struct ip_addr gw;
	uint16_t local_port;
	uint16_t remote_port;

} boardConfig;
*/

#include "../include/board_configuration_def.h"
void die(char *s)
{
    perror(s);
    exit(1);
}

int main(void)
{
    struct sockaddr_in si_other;
    int s, i, slen=sizeof(si_other);
    char buf[BUFLEN];
    char message[BUFLEN];
	boardConfig c;
	struct ip_addr ip;
    struct ip_addr gw;
    struct ip_addr mask;
	IP4_ADDR(&ip, 10,0,0,97);
	c.ipaddr = ip;
	IP4_ADDR(&mask, 255,255,255,0);
	c.netmask = mask;
	IP4_ADDR(&gw, 10,0,0,1);
	c.gw = gw;
	c.useDHCP = 1;
	c.local_port = 20202;
	c.remote_port = 20202;

    if ( (s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        die("socket");
    }

    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT);

    if (inet_aton(SERVER , &si_other.sin_addr) == 0)
    {
        fprintf(stderr, "inet_aton() failed\n");
        exit(1);
    }

	//send the message
	message[0] = 3; //send configuration packet
	uint8_t *ptr = (uint8_t *)&c;

	for(i=0;i<sizeof(boardConfig);i++)
	{
		message[i+1] = *ptr;
		ptr++;
	}

	if (sendto(s, message, i+1 , 0 , (struct sockaddr *) &si_other, slen)==-1)
	{
		die("sendto()");
	}



    close(s);
    return 0;
}
