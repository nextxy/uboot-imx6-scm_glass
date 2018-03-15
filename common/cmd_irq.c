/*
 * Copyright 2008 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <config.h>
#include <command.h>
#include <linux/types.h>

#include <imx6dq/regssrc.h>
#include <imx6dq/interrupt.h>
#include <imx6dq/gic.h>


//globals used for gic_test
unsigned int gicTestDone;

void gic_sgi_test_handler(void)
{
    printf("In gic_sgi_test_handler()\n");

    gicTestDone = 0;            // test complete
}

void gic_test(void)
{
    uint32_t  timeout = 300000;  //3s

    // init enable interrupt
    gic_init_sdk();
	
    printf("Starting GIC SGI test\n");

    // register and enable sgi isr
    register_interrupt_routine(SW_INTERRUPT_3, gic_sgi_test_handler);
    enable_interrupt(SW_INTERRUPT_3, CPU_0, 0);

    gicTestDone = 1;

    printf("Sending SGI\n");
    gic_send_sgi(SW_INTERRUPT_3, 1, kGicSgiFilter_UseTargetList);

    printf("Waiting\n");
	while ((gicTestDone)) {
        udelay(10);
        if (timeout <= 0) {
			printf("sgi time out on interrupt.\n");
			break;
        }            
        timeout--;
    }

    printf("SGI was handled\n");
}



static int do_interrupts(cmd_tbl_t *cmdtp, int flag, int argc,
			 char * const argv[])
{

	if (argc != 2)
		return CMD_RET_USAGE;

	/* on */
	if (strncmp(argv[1], "on", 2) == 0)
	{
		enable_interrupts();
		gic_test();
	}
	else
		disable_interrupts();

	return 0;
}

U_BOOT_CMD(
	interrupts, 5, 0, do_interrupts,
	"enable or disable interrupts",
	"[on, off]"
);


/* Implemented in $(CPU)/interrupts.c */
int do_irqinfo (cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);

U_BOOT_CMD(
	irqinfo,    1,    1,     do_irqinfo,
	"print information about IRQs",
	""
);
