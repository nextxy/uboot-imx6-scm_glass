/*
 * (C) Copyright 2003
 * Texas Instruments <www.ti.com>
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Alex Zuepke <azu@sysgo.de>
 *
 * (C) Copyright 2002-2004
 * Gary Jennejohn, DENX Software Engineering, <garyj@denx.de>
 *
 * (C) Copyright 2004
 * Philippe Robin, ARM Ltd. <philippe.robin@arm.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/proc-armv/ptrace.h>
#include <asm/u-boot-arm.h>
#include "gic.h"
#include "interrupt.h"

DECLARE_GLOBAL_DATA_PTR;

//! @brief Array of handler functions assigned to each interrupt.
irq_hdlr_t g_interrupt_handlers[IMX_INTERRUPT_COUNT];

volatile uint32_t g_vectNum[4];


#ifdef CONFIG_USE_IRQ

int arch_interrupt_init(void)
{
	return 0;

}



int interrupt_init (void)
{
	unsigned long cpsr;

	/*
	 * setup up stacks if necessary
	 */
	IRQ_STACK_START = gd->irq_sp - 4;
	IRQ_STACK_START_IN = gd->irq_sp + 8;
	FIQ_STACK_START = IRQ_STACK_START - CONFIG_STACKSIZE_IRQ;


	__asm__ __volatile__("mrs %0, cpsr\n"
			     : "=r" (cpsr)
			     :
			     : "memory");

	__asm__ __volatile__("msr cpsr_c, %0\n"
			     "mov sp, %1\n"
			     :
			     : "r" (IRQ_MODE | I_BIT | F_BIT | (cpsr & ~FIQ_MODE)),
			       "r" (IRQ_STACK_START)
			     : "memory");

	__asm__ __volatile__("msr cpsr_c, %0\n"
			     "mov sp, %1\n"
			     :
			     : "r" (FIQ_MODE | I_BIT | F_BIT | (cpsr & ~IRQ_MODE)),
			       "r" (FIQ_STACK_START)
			     : "memory");

	__asm__ __volatile__("msr cpsr_c, %0"
			     :
			     : "r" (cpsr)
			     : "memory");

	return arch_interrupt_init();
}

/* enable IRQ interrupts */
void enable_interrupts (void)
{
	unsigned long temp;
	__asm__ __volatile__("mrs %0, cpsr\n"
			     "bic %0, %0, #0x80\n"
			     "msr cpsr_c, %0"
			     : "=r" (temp)
			     :
			     : "memory");
}


/*
 * disable IRQ/FIQ interrupts
 * returns true if interrupts had been enabled before we disabled them
 */
int disable_interrupts (void)
{
	unsigned long old,temp;
	__asm__ __volatile__("mrs %0, cpsr\n"
			     "orr %1, %0, #0xc0\n"
			     "msr cpsr_c, %1"
			     : "=r" (old), "=r" (temp)
			     :
			     : "memory");
	return (old & 0x80) == 0;
}
#else
int interrupt_init (void)
{
	/*
	 * setup up stacks if necessary
	 */
	IRQ_STACK_START_IN = gd->irq_sp + 8;

	return 0;
}

void enable_interrupts (void)
{
	return;
}
int disable_interrupts (void)
{
	return 0;
}
#endif


void bad_mode (void)
{
	panic ("Resetting CPU ...\n");
	reset_cpu (0);
}

void show_regs (struct pt_regs *regs)
{
	unsigned long flags;
	const char *processor_modes[] = {
	"USER_26",	"FIQ_26",	"IRQ_26",	"SVC_26",
	"UK4_26",	"UK5_26",	"UK6_26",	"UK7_26",
	"UK8_26",	"UK9_26",	"UK10_26",	"UK11_26",
	"UK12_26",	"UK13_26",	"UK14_26",	"UK15_26",
	"USER_32",	"FIQ_32",	"IRQ_32",	"SVC_32",
	"UK4_32",	"UK5_32",	"UK6_32",	"ABT_32",
	"UK8_32",	"UK9_32",	"HYP_32",	"UND_32",
	"UK12_32",	"UK13_32",	"UK14_32",	"SYS_32",
	};

	flags = condition_codes (regs);

	printf("pc : [<%08lx>]	   lr : [<%08lx>]\n",
	       instruction_pointer(regs), regs->ARM_lr);
	if (gd->flags & GD_FLG_RELOC) {
		printf("reloc pc : [<%08lx>]	   lr : [<%08lx>]\n",
		       instruction_pointer(regs) - gd->reloc_off,
		       regs->ARM_lr - gd->reloc_off);
	}
	printf("sp : %08lx  ip : %08lx	 fp : %08lx\n",
	       regs->ARM_sp, regs->ARM_ip, regs->ARM_fp);
	printf ("r10: %08lx  r9 : %08lx	 r8 : %08lx\n",
		regs->ARM_r10, regs->ARM_r9, regs->ARM_r8);
	printf ("r7 : %08lx  r6 : %08lx	 r5 : %08lx  r4 : %08lx\n",
		regs->ARM_r7, regs->ARM_r6, regs->ARM_r5, regs->ARM_r4);
	printf ("r3 : %08lx  r2 : %08lx	 r1 : %08lx  r0 : %08lx\n",
		regs->ARM_r3, regs->ARM_r2, regs->ARM_r1, regs->ARM_r0);
	printf ("Flags: %c%c%c%c",
		flags & CC_N_BIT ? 'N' : 'n',
		flags & CC_Z_BIT ? 'Z' : 'z',
		flags & CC_C_BIT ? 'C' : 'c', flags & CC_V_BIT ? 'V' : 'v');
	printf ("  IRQs %s  FIQs %s  Mode %s%s\n",
		interrupts_enabled (regs) ? "on" : "off",
		fast_interrupts_enabled (regs) ? "on" : "off",
		processor_modes[processor_mode (regs)],
		thumb_mode (regs) ? " (T)" : "");
}

void do_undefined_instruction (struct pt_regs *pt_regs)
{
	printf ("undefined instruction\n");
	show_regs (pt_regs);
	bad_mode ();
}

void do_software_interrupt (struct pt_regs *pt_regs)
{
	printf ("software interrupt\n");
	show_regs (pt_regs);
	bad_mode ();
}

void do_prefetch_abort (struct pt_regs *pt_regs)
{
	printf ("prefetch abort\n");
	show_regs (pt_regs);
	bad_mode ();
}

void do_data_abort (struct pt_regs *pt_regs)
{
	printf ("data abort\n");
	show_regs (pt_regs);
	bad_mode ();
}

void do_not_used (struct pt_regs *pt_regs)
{
	printf ("not used\n");
	show_regs (pt_regs);
	bad_mode ();
}

void do_fiq (struct pt_regs *pt_regs)
{
	printf ("fast interrupt request\n");
	show_regs (pt_regs);
	bad_mode ();
}

// wyb #ifndef CONFIG_USE_IRQ
#ifndef CONFIG_USE_IRQ
void do_irq (struct pt_regs *pt_regs)
{
	printf ("interrupt request\n");
	show_regs (pt_regs);
	bad_mode ();
}
#else

void do_irq (struct pt_regs *pt_regs)
{
    // vectNum = RESERVED[31:13] | CPUID[12:10] | INTERRUPT_ID[9:0] 
    // send ack and get ID source 
    uint32_t vectNum = gic_read_irq_ack();
    
    // Check that INT_ID isn't 1023 or 1022 (spurious interrupt) 
    if (vectNum & 0x0200)
    {
        gic_write_end_of_irq(vectNum);  // send end of irq 
    }
    else
    {
        // copy the local value to the global image of CPUID
        unsigned cpu = (vectNum >> 10) & 0x7;
        unsigned irq = vectNum & 0x1FF;
        
        // Store the current interrupt number.
        g_vectNum[cpu] = irq;
        
        // Call the service routine stored in the handlers array. If there isn't
        // one for this IRQ, then call the default handler.
        irq_hdlr_t isr = g_interrupt_handlers[irq];
        if (isr)
        {
            isr();
        }
        else
        {
            default_interrupt_routine();
        }
        
        // Clear current interrupt number.
        g_vectNum[cpu] = 0;
        
        // Signal the end of the irq.
        gic_write_end_of_irq(vectNum);
    }
}


#endif


/* Implemented in $(CPU)/interrupts.c */
int do_irqinfo (cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{

	printf ("interrupt request irqinfo\n");
	// show_regs (pt_regs);
	return 0;

}


void disable_interrupt(uint32_t irq_id, uint32_t cpu_id)
{
    gic_enable_irq(irq_id, false);
    gic_set_cpu_target(irq_id, cpu_id, false);
}

void enable_interrupt(uint32_t irq_id, uint32_t cpu_id, uint32_t priority)
{
    gic_set_irq_priority(irq_id, priority);
    gic_set_irq_security(irq_id, false);    // set IRQ as non-secure
    gic_set_cpu_target(irq_id, cpu_id, true);
    gic_enable_irq(irq_id, true);
}

// set funcISR as the ISR function for the source ID #
void register_interrupt_routine(uint32_t irq_id, irq_hdlr_t isr)
{
    g_interrupt_handlers[irq_id] = isr;
}

void default_interrupt_routine(void)
{
    // find a way to address an IRQ handled by another CPU. Assumes
    // here that CPU_0 is used.
    printf("Interrupt %d has been asserted\n", g_vectNum[0]);
}








