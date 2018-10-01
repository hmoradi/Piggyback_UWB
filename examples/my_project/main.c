/*
 * Copyright (C) 2008, 2009, 2010  Kaspar Schleiser <kaspar@schleiser.de>
 * Copyright (C) 2013 INRIA
 * Copyright (C) 2013 Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Default application that shows a lot of functionality of RIOT
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 * @author      Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>

#include "thread.h"
#include "shell.h"
#include "shell_commands.h"
#include "periph/gpio.h"
#define ENABLE_DEBUG (1)
#include "debug.h"
#include "xtimer.h"

#define LED GPIO_PIN(PORT_B, 1)
#if FEATURE_PERIPH_RTC
#include "periph/rtc.h"
#endif

#ifdef MODULE_NETIF
#include "net/gnrc/pktdump.h"
#include "net/gnrc.h"
#endif

static int hello_world(int argc, char **argv){
	(void)argc;
	(void)argv;
	printf("hello world !\n");
	return 0;
}
const shell_command_t shell_commands[] = {
	{"hello","prints hello world",hello_world},
	{NULL,NULL,NULL}
};
int main(void)
{
#ifdef FEATURE_PERIPH_RTC
    rtc_init();
#endif

#ifdef MODULE_NETIF
    gnrc_netreg_entry_t dump = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL,
                                                          gnrc_pktdump_pid);
    gnrc_netreg_register(GNRC_NETTYPE_UNDEF, &dump);
#endif

    (void) puts("Welcome to RIOT!");
	if (gpio_init(LED, GPIO_OUT) < 0) {
		DEBUG("[ERROR] SDI GPIO initialization failed.");
	}
    //char line_buf[SHELL_DEFAULT_BUFSIZE];
    //shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
	while (1){
		gpio_write(LED, !gpio_read(LED));
		DEBUG("its working!\n");
		xtimer_usleep(500000U);
	}

    return 0;
}
