/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Enter an AArch32 kernel from AArch64 EL2.
 *
 * On 64-bit sunxi SoCs, TF-A BL31 owns EL3 and U-Boot runs in AArch64
 * EL2, so armv8_switch_to_el2 never takes its EL3 path and the generic
 * weak armv8_el2_to_aarch32 falls through, entering 32-bit kernels in
 * the wrong execution state.
 *
 * Demote to AArch32 Supervisor mode at EL1 here. The generic
 * armv8_switch_to_el1_m macro is not reusable for this: it programs
 * SCTLR_EL1 with AArch64-view RES1 bits 28/29, which read as TRE/AFE
 * in the AArch32 view and scramble the memory attributes the moment
 * the OS enables its MMU (TEX remap with uninitialized PRRR/NMRR).
 * Use the AArch32 reset value TF-A programs for 32-bit lower ELs
 * instead (SCTLR_AARCH32_EL1_RES1 | CP15BEN | nTWI | nTWE), and the
 * same PSR the vendor firmware hands 32-bit kernels (0x1d3: SVC,
 * A/I/F masked, ARM state).
 *
 * Called from armv8_switch_to_el2 with x0 = 0, x1 = machine id,
 * x2 = devicetree address, x4 = kernel entry point,
 * x5 = ES_TO_AARCH32. x0..x3 carry through to r0..r3.
 */

#include <config.h>
#include <linux/linkage.h>
#include <asm/macro.h>
#include <asm/system.h>

#define SCTLR_AARCH32_EL1	0x00c50838	/* RES1 | CP15BEN | nTWI/nTWE */
#define SPSR_AARCH32_SVC	0x000001d3	/* A|I|F masked, SVC32, ARM */

ENTRY(armv8_el2_to_aarch32)
	/* Give EL1 access to the architectural timer and counter */
	mrs	x6, cnthctl_el2
	orr	x6, x6, #(CNTHCTL_EL2_EL1PCEN_EN | CNTHCTL_EL2_EL1PCTEN_EN)
	msr	cnthctl_el2, x6
	msr	cntvoff_el2, xzr

	/* Virtual ID registers mirror the hardware */
	mrs	x6, midr_el1
	msr	vpidr_el2, x6
	mrs	x6, mpidr_el1
	msr	vmpidr_el2, x6

	/* No coprocessor or FP/SIMD traps */
	mov	x6, #CPTR_EL2_RES1
	msr	cptr_el2, x6
	msr	hstr_el2, xzr
	mov	x6, #CPACR_EL1_FPEN_EN
	msr	cpacr_el1, x6

	/* AArch32-view SCTLR reset state: MMU/caches off, no TRE/AFE */
	ldr	x6, =SCTLR_AARCH32_EL1
	msr	sctlr_el1, x6

	/* Lower EL is AArch32, HVC disabled */
	mov	x6, #HCR_EL2_HCD_DIS
	msr	hcr_el2, x6

	ldr	x6, =SPSR_AARCH32_SVC
	msr	spsr_el2, x6
	msr	elr_el2, x4
	isb
	eret
ENDPROC(armv8_el2_to_aarch32)
