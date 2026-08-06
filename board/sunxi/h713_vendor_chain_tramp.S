/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * H713 vendor chainload trampoline.
 *
 * The vendor's boot0 is linked to run at 0x00104000 -- the address the BROM
 * loads a first stage to, which is also where our own SPL is executing from.
 * So the final copy cannot be done by the SPL itself: this routine is copied
 * into DRAM first and called there, overwrites the SPL, and hands off.
 *
 * The vendor chain starts in AArch32 (all 64-bit Allwinner SoCs reset in
 * AArch32 and run the BROM in it; see rmr_switch.S, which is this switch in
 * the other direction) -- but it does not stay there: boot0 loads an AArch64
 * BL31 as its "monitor" and switches execution state to enter it, which needs
 * RMR, an EL3-only register.
 *
 * So entering boot0 by exception return to AArch32 EL1 is not enough. Bench
 * 2026-08-06: it runs correctly that way all the way through DRAM init, eMMC
 * bring-up and "Loading boot-pkg Succeed(index=0)", enumerates every TOC1
 * entry, then goes silent at "Jump to second Boot." -- the RMR write faulting
 * to an EL1 vector nobody set up.
 *
 * Instead, do what the BROM effectively does: point the RVBAR alias at boot0
 * and warm-reset this core into AArch32, which enters at EL3 with RMR
 * available. Memory survives a core warm reset, so the copy below stays put.
 *
 * If the reset instead lands at the AArch32 reset vector (0x0, the BROM) --
 * the Cortex-A53 TRM says RVBAR governs the AArch64 reset address, and leaves
 * AArch32's implementation defined -- the BROM simply re-runs, loads our own
 * first stage from LBA 0x10 and boots it. The marker is already consumed by
 * then, so that failure costs one boot and nothing else.
 *
 * Must stay position-independent and touch no literal pool: it executes from
 * an address the linker knows nothing about.
 *
 *   x0 = source (staged boot0)   x1 = destination (0x00104000)
 *   x2 = byte count              x3 = AArch32 entry point
 *   x4 = RVBAR alias address (MMIO, chosen by the caller)
 */

#include <linux/linkage.h>

#define RMR_EL3_AA64		(1 << 0)	/* 1 = reset into AArch64 */
#define RMR_EL3_RR		(1 << 1)	/* request warm reset */

ENTRY(h713_chain_tramp)
1:	ldp	x5, x6, [x0], #16
	stp	x5, x6, [x1], #16
	subs	x2, x2, #16
	b.ne	1b

	/* The copy landed in memory the I-cache may still hold stale lines for */
	dsb	sy
	ic	ialluis
	dsb	sy
	isb

	/*
	 * Only EL3 can retarget the reset state.  The caller checks this too;
	 * if we somehow arrive here at another EL, stop rather than write
	 * registers that will trap -- the FEL button is the way out.
	 */
	mrs	x5, CurrentEL
	cmp	x5, #(3 << 2)
	b.ne	2f

	/* Start address for the reset, through Allwinner's writable alias */
	str	w3, [x4]
	dsb	sy
	isb

	mrs	x5, rmr_el3
	orr	x5, x5, #RMR_EL3_RR
	bic	x5, x5, #RMR_EL3_AA64		/* come back up in AArch32 */
	msr	rmr_el3, x5
	isb

2:	wfi
	b	2b
ENDPROC(h713_chain_tramp)

	.globl h713_chain_tramp_end
h713_chain_tramp_end:
