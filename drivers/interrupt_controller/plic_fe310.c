/*
 * Copyright (c) 2017 Jean-Paul Etienne <fractalclone@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Platform Level Interrupt Controller (PLIC) driver
 *        for the SiFive Freedom E310 processor
 */

#include <kernel.h>
#include <arch/cpu.h>
#include <init.h>
#include <soc.h>
#include <sw_isr_table.h>

#define PLIC_FE310_IRQS        (CONFIG_NUM_IRQS - RISCV_MAX_GENERIC_IRQ)
#define PLIC_FE310_EN_SIZE     ((PLIC_FE310_IRQS >> 5) + 1)

struct plic_fe310_regs_t {
	uint32_t threshold_prio;
	uint32_t claim_complete;
};

static int save_irq;

/**
 *
 * @brief Enable a riscv PLIC-specific interrupt line
 *
 * This routine enables a RISCV PLIC-specific interrupt line.
 * riscv_plic_irq_enable is called by SOC_FAMILY_RISCV_PRIVILEGE
 * _arch_irq_enable function to enable external interrupts for
 * IRQS > RISCV_MAX_GENERIC_IRQ, whenever CONFIG_RISCV_HAS_PLIC
 * variable is set.
 * @param irq IRQ number to enable
 *
 * @return N/A
 */
void riscv_plic_irq_enable(uint32_t irq)
{
	uint32_t key;
	uint32_t fe310_irq = irq - RISCV_MAX_GENERIC_IRQ;
	volatile uint32_t *en =
		(volatile uint32_t *)FE310_PLIC_IRQ_EN_BASE_ADDR;

	key = irq_lock();
	en += (fe310_irq >> 5);
	*en |= (1 << (fe310_irq & 31));
	irq_unlock(key);
}

/**
 *
 * @brief Disable a riscv PLIC-specific interrupt line
 *
 * This routine disables a RISCV PLIC-specific interrupt line.
 * riscv_plic_irq_disable is called by SOC_FAMILY_RISCV_PRIVILEGE
 * _arch_irq_disable function to disable external interrupts, for
 * IRQS > RISCV_MAX_GENERIC_IRQ, whenever CONFIG_RISCV_HAS_PLIC
 * variable is set.
 * @param irq IRQ number to disable
 *
 * @return N/A
 */
void riscv_plic_irq_disable(uint32_t irq)
{
	uint32_t key;
	uint32_t fe310_irq = irq - RISCV_MAX_GENERIC_IRQ;
	volatile uint32_t *en =
		(volatile uint32_t *)FE310_PLIC_IRQ_EN_BASE_ADDR;

	key = irq_lock();
	en += (fe310_irq >> 5);
	*en &= ~(1 << (fe310_irq & 31));
	irq_unlock(key);
}

/**
 *
 * @brief Check if a riscv PLIC-specific interrupt line is enabled
 *
 * This routine checks if a RISCV PLIC-specific interrupt line is enabled.
 * @param irq IRQ number to check
 *
 * @return 1 or 0
 */
int riscv_plic_irq_is_enabled(uint32_t irq)
{
	volatile uint32_t *en =
		(volatile uint32_t *)FE310_PLIC_IRQ_EN_BASE_ADDR;
	uint32_t fe310_irq = irq - RISCV_MAX_GENERIC_IRQ;

	en += (fe310_irq >> 5);
	return !!(*en & (1 << (fe310_irq & 31)));
}

/**
 *
 * @brief Set priority of a riscv PLIC-specific interrupt line
 *
 * This routine set the priority of a RISCV PLIC-specific interrupt line.
 * riscv_plic_irq_set_prio is called by riscv32 _ARCH_IRQ_CONNECT to set
 * the priority of an interrupt whenever CONFIG_RISCV_HAS_PLIC variable is set.
 * @param irq IRQ number for which to set priority
 *
 * @return N/A
 */
void riscv_plic_set_priority(uint32_t irq, uint32_t priority)
{
	volatile uint32_t *prio =
		(volatile uint32_t *)FE310_PLIC_PRIO_BASE_ADDR;

	/* Can set priority only for PLIC-specific interrupt line */
	if (irq <= RISCV_MAX_GENERIC_IRQ)
		return;

	if (priority > FE310_PLIC_MAX_PRIORITY)
		priority = FE310_PLIC_MAX_PRIORITY;

	prio += (irq - RISCV_MAX_GENERIC_IRQ);
	*prio = priority;
}

/**
 *
 * @brief Get riscv PLIC-specific interrupt line causing an interrupt
 *
 * This routine returns the RISCV PLIC-specific interrupt line causing an
 * interrupt.
 * @param irq IRQ number for which to set priority
 *
 * @return N/A
 */
int riscv_plic_get_irq(void)
{
	return save_irq;
}

static void plic_fe310_irq_handler(void *arg)
{
	volatile struct plic_fe310_regs_t *regs =
		(volatile struct plic_fe310_regs_t *)FE310_PLIC_REG_BASE_ADDR;

	uint32_t irq;
	struct _isr_table_entry *ite;

	/* Get the IRQ number generating the interrupt */
	irq = regs->claim_complete;

	/*
	 * Save IRQ in save_irq. To be used, if need be, by
	 * subsequent handlers registered in the _sw_isr_table table,
	 * as IRQ number held by the claim_complete register is
	 * cleared upon read.
	 */
	save_irq = irq;

	/*
	 * If the IRQ is out of range, call _irq_spurious.
	 * A call to _irq_spurious will not return.
	 */
	if (irq == 0 || irq >= PLIC_FE310_IRQS)
		_irq_spurious(NULL);

	irq += RISCV_MAX_GENERIC_IRQ;

	/* Call the corresponding IRQ handler in _sw_isr_table */
	ite = (struct _isr_table_entry *)&_sw_isr_table[irq];
	ite->isr(ite->arg);

	/*
	 * Write to claim_complete register to indicate to
	 * PLIC controller that the IRQ has been handled.
	 */
	regs->claim_complete = save_irq;
}

/**
 *
 * @brief Initialize the SiFive FE310 Platform Level Interrupt Controller
 * @return N/A
 */
static int plic_fe310_init(struct device *dev)
{
	ARG_UNUSED(dev);

	volatile uint32_t *en =
		(volatile uint32_t *)FE310_PLIC_IRQ_EN_BASE_ADDR;
	volatile uint32_t *prio =
		(volatile uint32_t *)FE310_PLIC_PRIO_BASE_ADDR;
	volatile struct plic_fe310_regs_t *regs =
		(volatile struct plic_fe310_regs_t *)FE310_PLIC_REG_BASE_ADDR;
	int i;

	/* Ensure that all interrupts are disabled initially */
	for (i = 0; i < PLIC_FE310_EN_SIZE; i++) {
		*en = 0;
		en++;
	}

	/* Set priority of each interrupt line to 0 initially */
	for (i = 0; i < PLIC_FE310_IRQS; i++) {
		*prio = 0;
		prio++;
	}

	/* Set threshold priority to 0 */
	regs->threshold_prio = 0;

	/* Setup IRQ handler for PLIC driver */
	IRQ_CONNECT(RISCV_MACHINE_EXT_IRQ,
		    0,
		    plic_fe310_irq_handler,
		    NULL,
		    0);

	/* Enable IRQ for PLIC driver */
	irq_enable(RISCV_MACHINE_EXT_IRQ);

	return 0;
}

SYS_INIT(plic_fe310_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
