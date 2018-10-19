
#include <stdio.h>
//#include "diag/Trace.h"

#include "ethernetif.h"
#include "lwip.h"
#include "Configuration.h"
#include "PinDefinition.h"
#include "Watchdog.h"
#include "lwip/udp.h"
#include "eeprom.h"
#include "board_configuration.h"
#include <string.h>
#include "udp.h"

Configuration *config; //this config includes the runtime config received from ethernet
Watchdog *Wd;

void
main()
{

	HAL_Init();

	//load config from Flash, contains IP address & port information for stack
	boardConfig c;
	getConfig(&c);

	config = new Configuration();
	config->configured = 0;
	config->initBankGpio();

	//stack init
	LWIP_Init(&c);



	udp_echoserver_init(&c);

	//initialize watchdog to reset everything if no ethernet packets arrive in 10ms.
	Wd = new Watchdog();
	Wd->initialize(100);

	// infinite loop
	while (1)
	{
	  LWIP_Process();
	}

}


