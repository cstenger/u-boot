// SPDX-License-Identifier: GPL-2.0+
/*
 * sun50iw12 (Allwinner H713) DRAM initialization.
 *
 * Vendor-sequence replay driver. Every block below is a 1:1 decode of the
 * stock HY310 BT0 (boot0, vendor libdram "V1.18"), image SHA-256
 * f8ac16e44a83869c8fa7193531bc3a8b228235428a99ac426daa0902026c9f0f,
 * loaded at 0x104000. BT0 function addresses are quoted per block so each
 * write can be cross-checked against the decode in:
 *   h713-lab/reports/h713-bt0-dram-init-sequence-20260703T004500Z/
 *
 * Register layout (proven by BT0 literal census; the H616 layout at
 * 0x047fa000/0x047fb000/0x04800000 and the A133 PHY at 0x04830000 do NOT
 * apply to sun50iw12):
 *   MCTL COM 0x04810000, combined controller/PHY ("DRAMC") 0x04820000,
 *   CCU 0x02001000, NSI 0x05700000, R_PRCM 0x07010000, RTC region
 *   0x07090000, sys-cfg ZQ-cal block 0x03000160.
 *
 * Scope note: BT0's board-level set_pll (BT0 0x105174, called from
 * 0x10506e before init_DRAM) programs CPU_AXI/PLL_CPU/PLL_PERI/AHB/APB,
 * which U-Boot's clock_init() already does (hardware-verified on H713 by
 * the after-clock FEL checkpoint). Those writes are intentionally NOT
 * replayed here: reprogramming PLL_PERI in a FEL session risks the BROM
 * USB clocks. Only the DRAM-relevant remainder of that function (bus
 * gates, NSI, MBUS enable) is replayed below.
 *
 * DRAM type: the same sun50iw12 BT0 code drives LPDDR3 (HY310, type 7)
 * and DDR3 (HY200, type 3); select via CONFIG_SUNXI_DRAM_H713_LPDDR3_STOCK
 * or CONFIG_SUNXI_DRAM_H713_DDR3_STOCK. Both were verified register-
 * identical to their stock BT0 by offline replay and confirmed on
 * hardware over FEL.
 */

#include <config.h>
#include <init.h>
#include <log.h>
#include <vsprintf.h>
#include <asm/io.h>
#include <asm/arch/cpu.h>
#include <asm/arch/dram.h>
#include <asm/arch/sys_proto.h>
#include <linux/bitops.h>
#include <linux/delay.h>

/* Register blocks proven by the BT0 literal census (see file header). */
#define IW12_CCU_BASE		0x02001000
#define IW12_MCTL_COM_BASE	0x04810000
#define IW12_DRAMC_BASE		0x04820000
#define IW12_NSI_BASE		0x05700000
#define IW12_R_CPUCFG_BASE	0x07000000	/* +0x5d4: vendor bit16 gate */
#define IW12_R_PRCM_BASE	0x07010000
#define IW12_RTC_REGION_BASE	0x07090000
#define IW12_SYS_CFG_BASE	0x03000000	/* 0x160/0x168/0x16c: ZQ cal */
#define IW12_SID_BASE		0x03006000	/* +0x200 used by vendor */

#define IW12_DRAM_TYPE_LPDDR2	6	/* BT0 compares this literal */

/* CCU offsets touched by the replayed subset of BT0 0x105174. */
#define IW12_CCU_MBUS_CFG	0x540
#define IW12_CCU_CLK_680	0x680	/* clock cfg: src/div, update, en */
#define IW12_CCU_BGR_68C	0x68c	/* gate bits 1:0, reset bits 17:16 */
#define IW12_CCU_BGR_70C	0x70c	/* reset bit 16, gate bit 0 */
#define IW12_CCU_BGR_DD8	0xdd8	/* gate bit 0, reset bit 16 */

/* CCU offsets from the core-init clock path (BT0 0x106604/0x106580). */
#define IW12_CCU_PLL_DDR	0x010	/* lock-en bit29, lock bit28,
					   output gate bit27, N at [15:8] */
#define IW12_CCU_PLL_DDR_PAT	0x110	/* sigma-delta, tpr13 bit23 only */
#define IW12_CCU_DRAM_CLK	0x800	/* en bit31, update bit30, key
					   bit27, src [26:24], div [1:0] */
#define IW12_CCU_DRAM_BGR	0x80c	/* gate bit 0, reset bit 16 */

/*
 * Pre-DRAM analog/pad configuration from BT0 board init (0x10506e,
 * before its set_pll call). U-Boot has no equivalent. Live FEL read on
 * this board showed 0x07090160 == 0x883f10f7 after BROM, i.e. the block
 * is alive at FEL time; this write clears bits [27:24].
 */
static void iw12_sys_cfg(void)
{
	clrbits_le32(IW12_RTC_REGION_BASE + 0x160, 0xf000000);
	clrsetbits_le32(IW12_RTC_REGION_BASE + 0x1f4, 0xf1, 0xc0);
}

/*
 * DRAM-relevant bus enables from BT0 0x105174 ("set pll" tail), verbatim
 * order and delays, plus the PLL_PERI/PSI writes from the same function
 * (the CPU_AXI/PLL_CPUX pair remains omitted: CPU clock rate only).
 * Vendor logs "nsi init ok 2020-7-12" for the NSI writes.
 * The final MBUS state after boot0 board init is 0xc3000002 (also written
 * directly by BT0 0x105040), matching this sequence applied to the mux
 * value 0x03000002 that U-Boot's clock_init() programs.
 */
static void iw12_bus_init(void)
{
	void *const ccu = (void *)IW12_CCU_BASE;
	void *const nsi = (void *)IW12_NSI_BASE;

	/*
	 * NOTE (2026-07-04): PLL_PERI must NOT be reconfigured here. Under
	 * FEL the SPL executes from SRAM whose fetch path rides on PLL_PERI;
	 * a single disabling write (0x08006301, bit31=0) starves the CPU and
	 * hangs it (proven live: wdog-timeline6/7/8 all rebooted at the bare
	 * 16 s watchdog, i.e. the reset probe right after the write never
	 * ran). BROM leaves PLL_PERI VCO at 1200 MHz; if the DRAM path needs
	 * a different MBUS rate, adjust the MBUS *mux/divider* below (safe -
	 * it does not disable PLL_PERI), never the PLL itself.
	 */
	clrsetbits_le32(ccu + IW12_CCU_CLK_680, 0x30f, 0x2);
	udelay(10);
	setbits_le32(ccu + IW12_CCU_CLK_680, 0x1000000);
	udelay(10);
	setbits_le32(ccu + IW12_CCU_CLK_680, BIT(31));

	setbits_le32(ccu + IW12_CCU_BGR_68C, 0x3);
	udelay(10);
	setbits_le32(ccu + IW12_CCU_BGR_68C, 0x30000);

	setbits_le32(ccu + IW12_CCU_BGR_70C, 0x10000);
	udelay(20);
	setbits_le32(ccu + IW12_CCU_BGR_70C, 0x1);

	setbits_le32(ccu + IW12_CCU_BGR_DD8, 0x1);
	udelay(1);
	setbits_le32(ccu + IW12_CCU_BGR_DD8, 0x10000);
	udelay(1);

	/*
	 * MBUS bring-up. DELIBERATE DEVIATION from strict vendor order,
	 * based on live bisect (2026-07-03):
	 * - boot0 writes plain ENABLE (bit31) early, with RESET (bit30)
	 *   still asserted and the mux cleared, and only deasserts RESET
	 *   at the very end. Returning to FEL in (or transiting) that
	 *   enabled-while-in-reset state wedged the session (rung BUS2).
	 * - The NSI register file below is clocked by the MBUS fabric:
	 *   with no MBUS clock at all, the first NSI read hangs the bus
	 *   (rung BUS4 with MBUS moved after NSI).
	 * So: bring MBUS fully up FIRST (mux -> RESET deassert -> ENABLE,
	 * never dwelling enabled-in-reset, never clearing the mux), ending
	 * at the vendor-final value 0xc3000002, THEN touch NSI.
	 */
	clrsetbits_le32(ccu + IW12_CCU_MBUS_CFG, 0x3000007, 0x3000002);
	udelay(1);
	setbits_le32(ccu + IW12_CCU_MBUS_CFG, BIT(30));
	udelay(1);
	setbits_le32(ccu + IW12_CCU_MBUS_CFG, BIT(31));
	udelay(1);

	/* NSI enable ("nsi init ok"). */
	setbits_le32(nsi + 0x40, 0x10);
	udelay(1);
	setbits_le32(nsi + 0x44, 0x10);
	udelay(1);
	setbits_le32(nsi + 0x80, 0x10);
	udelay(1);

}

/*
 * "ldob fix" analog calibration, BT0 0x10506e (after its set_pll call).
 * Vendor also ORs a SID-derived calibration byte (from BT0 0x1095dc,
 * byte [23:16]) into R_PRCM+0x340 when it is non-zero. The live FEL read
 * on this board showed 0x07010340 == 0x00002f0f, matching the cal==0
 * path, so the SID step is intentionally not replayed yet.
 */
static void iw12_ldob_fix(void)
{
	void *const prcm = (void *)IW12_R_PRCM_BASE;

	clrsetbits_le32(prcm + 0x340, 0xff00, 0x2f00);

	/* BT0 0x1050d8: set bit0 on three PRCM regs. */
	setbits_le32(prcm + 0x110, BIT(0));
	setbits_le32(prcm + 0x114, BIT(0));
	setbits_le32(prcm + 0x118, BIT(0));
}

/*
 * ZQ/RES240 calibration selection, first block of vendor init_DRAM
 * (BT0 0x107120). Path selection is tpr13/tpr9-gated:
 *  - tpr13 bit16: "DRAM only have internal ZQ!!" (this board:
 *    tpr13 = 0xb4036223 -> bit16 set)
 *  - tpr9 bit16: external value from tpr9[7:0]
 *  - else: full RES240 calibration via 0x03000160 + R_PRCM+0x254
 */
static void iw12_zq_cal(const struct dram_para *para)
{
	void *const syscfg = (void *)IW12_SYS_CFG_BASE;
	void *const prcm = (void *)IW12_R_PRCM_BASE;

	if (para->tpr13 & BIT(16)) {
		debug("DRAM only have internal ZQ\n");
		setbits_le32(syscfg + 0x160, BIT(8));
		writel(0, syscfg + 0x168);
		udelay(10);
	} else if (para->tpr9 & BIT(16)) {
		setbits_le32(syscfg + 0x160, BIT(8));
		writel(para->tpr9 & 0xff, syscfg + 0x168);
		debug("ZQ RES240_CTRL_REG value = 0x%x\n",
		      readl(syscfg + 0x168));
		udelay(20);
	} else {
		writel(0, prcm + 0x254);
		clrbits_le32(syscfg + 0x160, 3);
		udelay(10);
		clrbits_le32(syscfg + 0x160, 0x104);
		setbits_le32(syscfg + 0x160, 2);
		udelay(10);
		setbits_le32(syscfg + 0x160, 1);
		udelay(20);
		debug("ZQ cal value0 = 0x%x\n", readl(syscfg + 0x16c));
	}
}

/*
 * PLL_DDR setup, BT0 0x106580, verbatim. Returns the PLL rate in MHz.
 * N = clk*2/24 (720 MHz -> N=60 -> 1440 MHz PLL, controller runs at
 * PLL/2). The tpr13-bit23 sigma-delta path (PLL pattern reg 0x110) is
 * not taken on this board (tpr13 = 0xb4036223, bit23 clear) and is not
 * implemented yet.
 */
static u32 iw12_pll_ddr_init(const struct dram_para *para)
{
	void *const pll = (void *)IW12_CCU_BASE + IW12_CCU_PLL_DDR;
	void *const dram_clk = (void *)IW12_CCU_BASE + IW12_CCU_DRAM_CLK;
	u32 n = (para->clk * 2) / 24;

	if (para->tpr13 & BIT(23))
		panic("sun50iw12: tpr13 sigma-delta PLL path not decoded\n");

	writel((readl(pll) & 0xfff800fc) | 0xc0000000 | ((n - 1) << 8), pll);
	clrbits_le32(pll, BIT(29));
	setbits_le32(pll, BIT(29));
	while (!(readl(pll) & BIT(28)))
		;
	udelay(20);

	setbits_le32(pll, BIT(27));	/* PLL output gate */

	/* BT0 0x1065de: DRAM clk = src 0 (PLL_DDR), div 1, enable. */
	clrsetbits_le32(dram_clk, 0x3000303, BIT(31));

	return 24 * n;
}

/*
 * DRAM clock/reset bring-up, BT0 0x106604, verbatim order and delays.
 * First writes into the DRAMC/COM blocks happen at the end of this
 * function, after clock and reset are up — do not readl those blocks
 * before this point.
 */
static void iw12_dram_clk_init(const struct dram_para *para)
{
	void *const bgr = (void *)IW12_CCU_BASE + IW12_CCU_DRAM_BGR;
	void *const dram_clk = (void *)IW12_CCU_BASE + IW12_CCU_DRAM_CLK;

	clrbits_le32(bgr, BIT(0));		/* gate off */
	clrbits_le32(bgr, BIT(16));		/* reset assert */
	clrbits_le32(dram_clk, BIT(30));
	clrbits_le32(dram_clk, BIT(31));
	setbits_le32(dram_clk, BIT(27));	/* key/update */
	udelay(10);

	iw12_pll_ddr_init(para);		/* controller clk = rate/2 */
	udelay(100);

	clrbits_le32(dram_clk, 3);
	setbits_le32(dram_clk, BIT(27));
	udelay(5);
	setbits_le32(bgr, BIT(16));		/* reset deassert */
	setbits_le32(dram_clk, BIT(30));
	udelay(5);
	setbits_le32(bgr, BIT(0));		/* gate on */
	setbits_le32(dram_clk, BIT(31));

	/* First DRAMC/COM writes, BT0 0x10668c. */
	writel(0x8000, (void *)IW12_DRAMC_BASE + 0x00c);
	udelay(10);
	setbits_le32((void *)IW12_MCTL_COM_BASE + 0x020, 0x8000);
	udelay(10);
	setbits_le32((void *)IW12_MCTL_COM_BASE + 0x020, 0x100);
}

/*
 * Per-byte ODT/drive config from tpr5/tpr6, BT0 0x106d80.
 * Complete no-op on this board: tpr13 = 0xb4036223 has bit17 set (skip
 * everything) and bit16 set (skip the tpr6 half).
 */
static void iw12_odt_cfg(const struct dram_para *para)
{
	void *const dramc = (void *)IW12_DRAMC_BASE;

	if (para->tpr13 & BIT(17))
		return;

	clrsetbits_le32(dramc + 0x110, 0x7f7f7f7f, para->tpr5);
	if (para->tpr13 & BIT(16))
		return;

	clrsetbits_le32(dramc + 0x114, 0x7f, para->tpr6 & 0x7f);
}

/*
 * mctl_com_init, BT0 0x1066c0, verbatim algorithm. Geometry comes from
 * para1 nibbles; the column-field lookup ports the BT0 tbb jump table at
 * 0x106784 exactly. For this board (para1=0x10f410f4, para2=0x04000000,
 * type=7/LPDDR3, tpr4=0) the net effect is:
 *   COM+0x004: clr 0x3f0000 set 0x200000
 *   COM+0x000: config 0x4f1000, geometry low bits 0x9e4 (single rank)
 *   DRAMC+0x120 = 0x201
 */
static void iw12_com_init(const struct dram_para *para)
{
	/*
	 * BT0 tbb table: (para1&0xf)-1 -> column field. Entry values decoded
	 * from the tbb byte targets at 0x106784 and confirmed by executing
	 * BT0 under Unicorn: idx 3 (this board) lands on the |0x900 arm,
	 * giving the live COM+0x000 value 0x004f19e4.
	 */
	static const u16 col_field[8] = {
		0x700, 0x800, 0x600, 0x900, 0x600, 0x600, 0x600, 0xa00 };
	void *const com = (void *)IW12_MCTL_COM_BASE;
	void *const dramc = (void *)IW12_DRAMC_BASE;
	u32 val, geom, idx, ranks, i;

	clrsetbits_le32(com + 0x004, 0x3f0000, 0x200000);

	val = readl(com) & ~(0xff0000 | 0xf000);
	val |= 0x400000;
	val |= (para->type << 16) & 0x70000;
	if (!(para->para2 & 1))
		val |= 0x1000;
	if ((u32)para->type - 6 <= 1)
		val |= 0x80000;
	else
		val |= (para->tpr13 << 14) & 0x80000;
	writel(val, com);

	if (para->para2 & 0x100)
		ranks = (((para->para2 >> 12) & 0xf) == 1) ? 2 : 1;
	else
		ranks = 1;

	for (i = 0; i < ranks; i++) {
		u32 shift = i * 16;

		geom = readl(com + i * 8) & 0xfffff000;
		geom |= (para->para2 >> 12) & 3;
		geom |= ((para->para1 >> (shift + 12)) << 2) & 4;
		geom |= ((((para->para1 >> (shift + 4)) & 0xff) - 1) << 4)
			& 0xff;
		idx = ((para->para1 >> shift) & 0xf) - 1;
		geom |= (idx < 8) ? col_field[idx] : 0x600;
		writel(geom, com + i * 8);
	}

	writel((readl(com) & 1) ? 0x303 : 0x201, dramc + 0x120);
	if (para->para2 & 1) {
		writel(0, dramc + 0x444);
		writel(0, dramc + 0x4c4);
	}

	if (para->tpr4) {
		setbits_le32(com, (para->tpr4 << 25) & 0x6000000);
		setbits_le32(com + 0x008,
			     (para->tpr4 << 10) & 0xfffff000);
	}
}

/*
 * DRAM-type ODT/PHY parameter group at COM+0x500, BT0 0x106818.
 * The DDR3 paths select constants by a SID chip-bin read (SID+0x200);
 * the LPDDR3 path (this board) is unconditional. Final write sets bit0
 * as a latch strobe.
 */
static void iw12_com_type_params(const struct dram_para *para)
{
	void *const com = (void *)IW12_MCTL_COM_BASE;

	if (para->type == SUNXI_DRAM_TYPE_DDR3) {
		/*
		 * DDR3 COM+0x500 group, replayed from the Bench board's stock
		 * BT0 DDR3 arm (BT0 0x106818). Final write latches bit0.
		 * Clock-invariant (identical across a 312..1200 MHz Unicorn
		 * sweep); presumed geometry/para-derived, so a board with a
		 * different DDR3 configuration may need new values here.
		 */
		writel(0x071a0000, com + 0x500);
		writel(0x14c4920e, com + 0x504);
		writel(0x00592e66, com + 0x508);
		writel(0x01543ced, com + 0x50c);
		writel(0x071a0001, com + 0x500);
		return;
	}

	if (para->type != SUNXI_DRAM_TYPE_LPDDR3)
		panic("sun50iw12: only LPDDR3/DDR3 COM param sets are ported\n");

	writel(0x0023a400, com + 0x500);
	writel(0x008510c3, com + 0x504);
	writel(0, com + 0x508);
	writel(0, com + 0x50c);
	writel(0x0023a401, com + 0x500);
}

/*
 * Timing register write-out, BT0 0x105da6. Despite an intermediate decode
 * note that attributed these values to a RAM struct only, the forced-Thumb
 * disassembly writes this block directly to DRAMC+0x02c..0x094. This ports
 * the LPDDR3/tpr13-bit1 path used by the stock HY310 BT0.
 */
static void iw12_set_timing(const struct dram_para *para)
{
	void *const dramc = (void *)IW12_DRAMC_BASE;
	u32 clk = para->clk;
	u32 ip, lr, mr0, mr1, mr2, mr3;
	u32 wr_latency;
	u32 tpr0_b21_3, tpr0_b15_6, tpr0_b11_4, tpr0_b6_5, tpr0_b0_6;
	u32 tpr1_b23_5, tpr1_b20_3, tpr1_b15_5, tpr1_b11_4;
	u32 tpr1_b6_5, tpr1_b0_6;
	u32 tpr2_b12_9, tpr2_b0_12;

	if (para->type == SUNXI_DRAM_TYPE_DDR3) {
		/*
		 * DDR3 timing block DRAMC+0x02c..0x094, generalised from a
		 * 30-point Unicorn sweep (312..1200 MHz) of the Bench board's
		 * stock BT0 DDR3 arm (BT0 0x1062ea). The vendor code uses a
		 * frozen cycle-count table with a single speed-bin branch
		 * above 800 MHz (plus trd2wr above 912 MHz); only trasmax and
		 * the four init timers scale with the clock. Registers follow
		 * the D1 (sun20i) DRAMTMG/PITMG/PTR/RFSHTMG layout. The
		 * vendor computes from the PLL-rounded clock and overrides
		 * mr0/mr2 by speed bin (para->mr0/mr2 are NOT used here);
		 * emulation-verified bit-exact at all 30 swept clocks.
		 */
		u32 eclk = (clk * 2 / 24) * 12;	/* PLL_DDR-rounded clock */
		bool hs = eclk > 800;		/* high speed bin */
		u32 tccd = 2, tfaw = 16, trrd = 4, trcd = 5, trc = 17;
		u32 txp = 3, trp = 5, tras = 12, tmod = 12, tmrd = 4, tmrw = 0;
		u32 tcke = 3, tckesr = 4, tcksrx = 5, trefi = 77, trfc = 110;
		u32 trasmax = eclk / 30;
		u32 trtp = hs ? 4 : 3;
		u32 tcl = hs ? 7 : 6;		/* CAS latency / 2 */
		u32 tcwl = hs ? 5 : 4;
		u32 twtp = hs ? 12 : 11;	/* WL + BL/2 + tWTR */
		u32 twr2rd = hs ? 10 : 9;	/* WL + tWTR */
		u32 trd2wr = (eclk > 912) ? 6 : 5;
		u32 t_rdata_en = hs ? 5 : 4;
		u32 tdinit0 = 500 * eclk + 1;		/* 500 us */
		u32 tdinit1 = 360 * eclk / 1000 + 1;	/* 360 ns */
		u32 tdinit2 = 200 * eclk + 1;		/* 200 us */
		u32 tdinit3 = eclk + 1;			/*   1 us */

		wr_latency = hs ? 3 : 2;
		mr0 = hs ? 0x1e14 : 0x1c70;
		mr1 = para->mr1 & 0xffff;
		mr2 = hs ? 0x20 : 0x18;
		mr3 = 0;

		writel((para->odt_en >> 4) & 0x3, dramc + 0x02c);
		writel(mr0, dramc + 0x030);
		writel(mr1, dramc + 0x034);
		writel(mr2, dramc + 0x038);
		writel(mr3, dramc + 0x03c);
		writel(tdinit0 | (tdinit1 << 20), dramc + 0x050);
		writel(tdinit2 | (tdinit3 << 20), dramc + 0x054);
		writel((twtp << 24) | (tfaw << 16) | (trasmax << 8) | tras,
		       dramc + 0x058);
		writel((txp << 16) | (trtp << 8) | trc, dramc + 0x05c);
		writel((tcwl << 24) | (tcl << 16) | (trd2wr << 8) | twr2rd,
		       dramc + 0x060);
		writel((tmrw << 16) | (tmrd << 12) | tmod, dramc + 0x064);
		writel((trcd << 24) | (tccd << 16) | (trrd << 8) | trp,
		       dramc + 0x068);
		writel((tcksrx << 24) | (tcksrx << 16) | (tckesr << 8) | tcke,
		       dramc + 0x06c);
		writel((readl(dramc + 0x078) & 0x0fff0000) | 0xf0000010 |
		       (hs ? 0x7600 : 0x6600), dramc + 0x078);
		writel((2 << 24) | (t_rdata_en << 16) | BIT(8) | wr_latency,
		       dramc + 0x080);
		writel(0, dramc + 0x08c);
		writel((trefi << 16) | trfc, dramc + 0x090);
		writel((trefi << 15) & 0x0fff0000, dramc + 0x094);
		return;
	}

	if (para->type != SUNXI_DRAM_TYPE_LPDDR3 ||
	    !(para->tpr13 & BIT(1)))
		panic("sun50iw12: only stock LPDDR3 packed timing path is ported\n");

	tpr0_b21_3 = (para->tpr0 >> 21) & 0x7;
	tpr0_b15_6 = (para->tpr0 >> 15) & 0x3f;
	tpr0_b11_4 = (para->tpr0 >> 11) & 0xf;
	tpr0_b6_5 = (para->tpr0 >> 6) & 0x1f;
	tpr0_b0_6 = para->tpr0 & 0x3f;

	tpr1_b23_5 = (para->tpr1 >> 23) & 0x1f;
	tpr1_b20_3 = (para->tpr1 >> 20) & 0x7;
	tpr1_b15_5 = (para->tpr1 >> 15) & 0x1f;
	tpr1_b11_4 = (para->tpr1 >> 11) & 0xf;
	tpr1_b6_5 = (para->tpr1 >> 6) & 0x1f;
	tpr1_b0_6 = para->tpr1 & 0x3f;
	tpr2_b12_9 = (para->tpr2 >> 12) & 0x1ff;
	tpr2_b0_12 = para->tpr2 & 0xfff;

	ip = (clk > 800) ? 4 : 3;
	lr = (clk > 800) ? 6 : 5;
	wr_latency = (clk > 800) ? 3 : 2;
	tpr1_b11_4 += 5;
	tpr1_b20_3 += 5;

	if (tpr1_b15_5 + tpr1_b6_5 < ((clk > 800) ? 9 : 8))
		tpr1_b15_5 = ((clk > 800) ? 9 : 8) - tpr1_b6_5;

	/*
	 * BT0 stores the para->mrX low halves verbatim on this path; the
	 * high-half-selects-default arms in the static decode do not execute
	 * for the stock LPDDR3 parameters (Unicorn-executed BT0 writes
	 * 0 / 0xc3 / 0xa / 0x2). The old "mr3 default 0" sent a reserved
	 * LPDDR3 MR3 drive-strength code to the die.
	 */
	mr0 = para->mr0 & 0xffff;
	mr1 = para->mr1 & 0xffff;
	mr2 = para->mr2 & 0xffff;
	mr3 = para->mr3 & 0xffff;

	writel((para->odt_en >> 4) & 0x3, dramc + 0x02c);
	writel(mr0, dramc + 0x030);
	writel(mr1, dramc + 0x034);
	writel(mr2, dramc + 0x038);
	writel(mr3, dramc + 0x03c);
	/*
	 * clk/60, not clk/15: BT0's magic-multiply keeps an extra lsr #4
	 * (executed value 0x0c at 720 MHz, where clk/15 would give 0x30).
	 */
	writel(tpr1_b0_6 | (tpr0_b15_6 << 16) |
	       ((tpr1_b11_4 + ip) << 24) | ((clk / 60) << 8),
	       dramc + 0x058);
	writel(tpr0_b0_6 | (tpr1_b23_5 << 16) | (tpr1_b15_5 << 8),
	       dramc + 0x05c);
	writel((tpr1_b20_3 + ip) | (13 << 8) |
	       (((clk > 800) ? 7 : 6) << 16) | (ip << 24),
	       dramc + 0x060);
	writel((5 << 12) | (5 << 16) | 12, dramc + 0x064);
	writel(tpr1_b6_5 | (tpr0_b11_4 << 8) | (tpr0_b21_3 << 16) |
	       (tpr0_b6_5 << 24), dramc + 0x068);
	writel(0x05050503, dramc + 0x06c);
	writel((readl(dramc + 0x078) & 0x0fff0000) | 0xf0000010 |
	       ((clk > 800) ? 0x7600 : 0x6600), dramc + 0x078);
	writel(wr_latency | BIT(8) | (lr << 16) | BIT(25), dramc + 0x080);
	writel((200 * clk + 1) | (((100 * clk) / 1000 + 1) << 20),
	       dramc + 0x050);
	writel((11 * clk + 1) | ((clk + 1) << 20), dramc + 0x054);
	writel(tpr2_b12_9 | (tpr2_b0_12 << 16), dramc + 0x090);
	writel((tpr2_b0_12 << 15) & 0x0fff0000, dramc + 0x094);
}

/*
 * Per-byte/lane delay compensation, BT0 0x105b54. This is closely related
 * to mainline D1's eye_delay_compensation(), rebased from 0x03103000 to
 * the sun50iw12 combined DRAMC block, but the tpr10/tpr11/tpr12 nibble
 * mapping below follows H713 BT0 exactly.
 */
static void iw12_eye_delay_compensation(const struct dram_para *para)
{
	void *const dramc = (void *)IW12_DRAMC_BASE;
	u32 delay, off;

	delay = ((para->tpr11 & 0xf) << 9) | ((para->tpr12 & 0xf) << 1);
	for (off = 0x310; off < 0x334; off += 4)
		setbits_le32(dramc + off, delay);

	delay = ((para->tpr11 & 0xf0) << 5) |
		((para->tpr12 & 0xf0) >> 3);
	for (off = 0x390; off < 0x3b4; off += 4)
		setbits_le32(dramc + off, delay);

	delay = ((para->tpr11 & 0xf00) << 1) |
		((para->tpr12 & 0xf00) >> 7);
	for (off = 0x410; off < 0x434; off += 4)
		setbits_le32(dramc + off, delay);

	delay = ((para->tpr11 & 0xf000) >> 3) |
		((para->tpr12 & 0xf000) >> 11);
	for (off = 0x490; off < 0x4b4; off += 4)
		setbits_le32(dramc + off, delay);

	clrbits_le32(dramc + 0x100, BIT(26));

	delay = ((para->tpr11 & 0xf0000) >> 7) |
		((para->tpr12 & 0xf0000) >> 15);
	setbits_le32(dramc + 0x334, delay);
	setbits_le32(dramc + 0x338, delay);

	delay = ((para->tpr11 & 0xf00000) >> 11) |
		((para->tpr12 & 0xf00000) >> 19);
	setbits_le32(dramc + 0x3b4, delay);
	setbits_le32(dramc + 0x3b8, delay);

	delay = ((para->tpr11 & 0x0f000000) >> 15) |
		((para->tpr12 & 0x0f000000) >> 23);
	setbits_le32(dramc + 0x434, delay);
	setbits_le32(dramc + 0x438, delay);

	delay = ((para->tpr11 & 0xf0000000) >> 19) |
		((para->tpr12 & 0xf0000000) >> 27);
	setbits_le32(dramc + 0x4b4, delay);
	setbits_le32(dramc + 0x4b8, delay);

	setbits_le32(dramc + 0x33c,
			       (para->tpr11 & 0xf0000) << 9);
	setbits_le32(dramc + 0x3bc,
			       (para->tpr11 & 0xf00000) << 5);
	setbits_le32(dramc + 0x43c,
			       (para->tpr11 & 0x0f000000) << 1);
	setbits_le32(dramc + 0x4bc,
			       (para->tpr11 & 0xf0000000) >> 3);

	setbits_le32(dramc + 0x100, BIT(26));
	udelay(1);

	delay = (para->tpr10 & 0xf0) << 4;
	for (off = 0x240; off < 0x27c; off += 4)
		setbits_le32(dramc + off, delay);
	for (off = 0x228; off < 0x240; off += 4)
		setbits_le32(dramc + off, delay);

	setbits_le32(dramc + 0x218, (para->tpr10 << 8) & 0xf00);
	setbits_le32(dramc + 0x21c, para->tpr10 & 0xf00);
	setbits_le32(dramc + 0x280, (para->tpr10 >> 4) & 0xf00);
}

/*
 * PHY/DFI setup and PIR-style training trigger, BT0 0x1068d0. This is
 * the first block where the DRAM chips participate. The stock HY310
 * tpr13 DQS-gating mode is 0, so the normal trigger is 0x172 -> 0x173
 * when R_CPUCFG+0x5d4 bit16 is clear, or 0x62 -> 0x63 plus the extra
 * staged PHY toggles when that bit is set. The mode-1/2 arms are kept
 * because they are explicit in BT0 and match the D1-family driver.
 */
static int iw12_phy_train(const struct dram_para *para)
{
	void *const dramc = (void *)IW12_DRAMC_BASE;
	void *const com = (void *)IW12_MCTL_COM_BASE;
	u32 dqs_gating_mode = (para->tpr13 >> 2) & 0x3;
	u32 cpu_gate = readl(IW12_R_CPUCFG_BASE + 0x5d4) & BIT(16);
	u32 val, cmd, off;
	u32 status, ret;

	clrsetbits_le32(dramc + 0x108, 0xf00, 0x300);

	val = (para->odt_en & BIT(0)) ? 0 : BIT(5);
	for (off = 0x344; off != 0x544; off += 0x80) {
		u32 reg = readl(dramc + off) & ~0x30;

		reg |= val;
		if (para->clk > 672)
			reg = (reg & 0xffff09f1) | 0x400;
		else
			reg &= 0xffff0ff1;
		writel(reg, dramc + off);
	}

	setbits_le32(dramc + 0x208, BIT(1));
	iw12_eye_delay_compensation(para);

	if (dqs_gating_mode == 1) {
		clrbits_le32(dramc + 0x108, 0xc0);
		clrbits_le32(dramc + 0x0bc, 0x107);
	} else if (dqs_gating_mode == 2) {
		clrsetbits_le32(dramc + 0x108, 0xc0, 0x80);
		clrsetbits_le32(dramc + 0x0bc, 0x107,
				 (((para->tpr13 >> 16) & 0x1f) - 2) |
				 0x100);
		clrsetbits_le32(dramc + 0x11c, BIT(31), BIT(27));
	} else {
		clrbits_le32(dramc + 0x108, BIT(6));
		udelay(10);
		setbits_le32(dramc + 0x108, 0xc0);
	}

	if (para->type == SUNXI_DRAM_TYPE_LPDDR3) {
		if (dqs_gating_mode == 1) {
			clrsetbits_le32(dramc + 0x11c, 0x080000c0,
					 BIT(31));
		} else {
			clrsetbits_le32(dramc + 0x11c, 0x77000000,
					 0x22000000);
		}
	}

	clrsetbits_le32(dramc + 0x0c0, 0x0fffffff,
			 (para->para2 & BIT(12)) ? 0x03000001 :
						    0x01003087);

	if (cpu_gate) {
		clrbits_le32((void *)IW12_R_PRCM_BASE + 0x250, BIT(1));
		udelay(10);
	}

	clrsetbits_le32(dramc + 0x140, 0x03ffffff,
			 (para->zq & 0x00ffffff) | BIT(25));

	if (dqs_gating_mode == 1) {
		writel(0x52, dramc + 0x000);
		writel(0x53, dramc + 0x000);
		while (!(readl(dramc + 0x010) & BIT(0)))
			;
		udelay(10);
		cmd = (para->type == SUNXI_DRAM_TYPE_DDR3) ? 0x5a0 : 0x520;
	} else if (!cpu_gate) {
		cmd = (para->type == SUNXI_DRAM_TYPE_DDR3) ? 0x1f2 : 0x172;
	} else {
		cmd = 0x62;
	}

	writel(cmd, dramc + 0x000);
	setbits_le32(dramc + 0x000, BIT(0));
	udelay(10);
	while (!(readl(dramc + 0x010) & BIT(0)))
		;

	if (cpu_gate) {
		clrsetbits_le32(dramc + 0x10c, 0x06000000, 0x04000000);
		udelay(10);

		setbits_le32(dramc + 0x004, BIT(0));
		while ((readl(dramc + 0x018) & 0x7) != 0x3)
			;

		clrbits_le32((void *)IW12_R_PRCM_BASE + 0x250, BIT(0));
		udelay(10);

		clrbits_le32(dramc + 0x004, BIT(0));
		while ((readl(dramc + 0x018) & 0x7) != 0x1)
			;

		udelay(15);

		if (dqs_gating_mode == 1) {
			clrbits_le32(dramc + 0x108, 0xc0);
			clrsetbits_le32(dramc + 0x10c, 0x06000000,
					 0x02000000);
			udelay(1);
			writel(0x401, dramc + 0x000);
			while (!(readl(dramc + 0x010) & BIT(0)))
				;
		}
	}

	status = readl(dramc + 0x010);
	ret = (status & BIT(20)) ? 0 : 1;
	if (status & BIT(20)) {
		return 0;
	}

	while (!(readl(dramc + 0x018) & BIT(0)))
		;

	setbits_le32(dramc + 0x08c, BIT(31));
	udelay(10);
	clrbits_le32(dramc + 0x08c, BIT(31));
	udelay(10);
	setbits_le32(com + 0x014, BIT(31));
	udelay(10);

	clrbits_le32(dramc + 0x10c, 0x06000000);
	if (dqs_gating_mode == 1)
		clrsetbits_le32(dramc + 0x11c, 0xc0, 0x40);

	return ret;
}

static u32 iw12_rank_size_mb(u32 cr)
{
	u32 shift = ((cr >> 4) & 0xf) + ((cr >> 8) & 0xf) +
		    ((cr >> 2) & 0x3) - 14;

	return 1U << shift;
}

/* DRAM size calculator, BT0 0x106bdc. Returns MiB. */
static u32 iw12_get_dram_size(void)
{
	void *const com = (void *)IW12_MCTL_COM_BASE;
	u32 rank0 = readl(com + 0x000);
	u32 size0 = iw12_rank_size_mb(rank0);
	u32 rank1, size1;

	if (!(rank0 & 0x3))
		return size0;

	rank1 = readl(com + 0x008);
	if (rank1 & 0x3)
		size1 = iw12_rank_size_mb(rank1);
	else
		size1 = size0;

	return size0 + size1;
}

/*
 * SID-indexed maximum size clamp, BT0 0x10723a. The first table is for
 * non-DDR3 types (stock LPDDR3), the second for DDR3. The byte table maps
 * SID low16 values 0x40..0x43 into those capacity tables.
 */
static u32 iw12_sid_limit_mb(const struct dram_para *para)
{
	static const u32 limit_mb[2][4] = {
		{ 0x0c00, 0x0400, 0x0400, 0x0000 },
		{ 0x0c00, 0x0c00, 0x0400, 0x0c00 },
	};
	static const u8 sid_index[4] = { 3, 0, 1, 2 };
	u32 sid = readl(IW12_SID_BASE + 0x200);
	u32 idx = (sid & 0xffff) - 0x40;
	u32 variant = (idx <= 3) ? sid_index[idx] : 0;
	u32 type = (para->type == SUNXI_DRAM_TYPE_DDR3) ? 1 : 0;

	return limit_mb[type][variant];
}

/*
 * Post-training controller tail, BT0 0x10723a..0x107326, excluding the
 * optional simple memory test. This reports/clamps size, applies Auto-SR
 * policy, and finishes a handful of controller bits.
 */
static u32 iw12_finish_controller_tail(const struct dram_para *para)
{
	void *const dramc = (void *)IW12_DRAMC_BASE;
	u32 limit_mb = iw12_sid_limit_mb(para);
	u32 mem_size_mb;

	if (para->para2 & BIT(31))
		mem_size_mb = (para->para2 >> 16) & ~BIT(15);
	else
		mem_size_mb = iw12_get_dram_size();

	if (mem_size_mb > limit_mb)
		mem_size_mb = limit_mb;

	if (para->tpr13 & BIT(30)) {
		writel(para->tpr8 ? para->tpr8 : 0x10000200,
		       dramc + 0x0a0);
		writel(0x40a, dramc + 0x09c);
		setbits_le32(dramc + 0x004, BIT(0));
	} else {
		clrbits_le32(dramc + 0x0a0, 0xffff);
		clrbits_le32(dramc + 0x004, BIT(0));
	}

	if (para->tpr13 & BIT(9)) {
		clrsetbits_le32(dramc + 0x100, 0xf000, 0x5000);
	} else if (para->type != IW12_DRAM_TYPE_LPDDR2) {
		clrbits_le32(dramc + 0x100, 0xf000);
	}

	setbits_le32(dramc + 0x140, BIT(31));

	if (para->tpr13 & BIT(8))
		setbits_le32(dramc + 0x0b8, 0x300);

	if (para->tpr13 & BIT(26))
		clrbits_le32(dramc + 0x108, BIT(13));
	else
		setbits_le32(dramc + 0x108, BIT(13));

	if (para->type == SUNXI_DRAM_TYPE_LPDDR3)
		clrsetbits_le32(dramc + 0x07c, 0xf0000, 0x10000);

	return mem_size_mb;
}

/* Simple SDRAM write/read test, BT0 0x106cec. Returns 0 on success. */
static int iw12_simple_wr_test(u32 mem_size_mb, u32 len)
{
	u32 offs = (mem_size_mb / 2) << 18;	/* u32 words to half memory */
	volatile u32 *addr = (volatile u32 *)CFG_SYS_SDRAM_BASE;
	u32 i, val;

	for (i = 0; i != len; i++) {
		addr[i] = 0x01234567 + i;
		addr[offs + i] = 0xfedcba98 + i;
	}

	for (i = 0; i != len; i++) {
		val = addr[i];
		if (val != 0x01234567 + i)
			return 1;

		val = addr[offs + i];
		if (val != 0xfedcba98 + i)
			return 1;
	}

	return 0;
}

static const struct dram_para para = {
	.clk = CONFIG_DRAM_CLK,
#ifdef CONFIG_SUNXI_DRAM_H713_LPDDR3_STOCK
	.type = SUNXI_DRAM_TYPE_LPDDR3,
#endif
#ifdef CONFIG_SUNXI_DRAM_H713_DDR3_STOCK
	.type = SUNXI_DRAM_TYPE_DDR3,
#endif
	.zq = CONFIG_DRAM_SUNXI_ZQ,
	.odt_en = CONFIG_DRAM_SUNXI_ODT_EN,
	.para1 = CONFIG_DRAM_SUNXI_PARA1,
	.para2 = CONFIG_DRAM_SUNXI_PARA2,
	.mr0 = CONFIG_DRAM_SUNXI_MR0,
	.mr1 = CONFIG_DRAM_SUNXI_MR1,
	.mr2 = CONFIG_DRAM_SUNXI_MR2,
	.mr3 = CONFIG_DRAM_SUNXI_MR3,
	.tpr0 = CONFIG_DRAM_SUNXI_TPR0,
	.tpr1 = CONFIG_DRAM_SUNXI_TPR1,
	.tpr2 = CONFIG_DRAM_SUNXI_TPR2,
	.tpr3 = CONFIG_DRAM_SUNXI_TPR3,
	.tpr4 = CONFIG_DRAM_SUNXI_TPR4,
	.tpr5 = CONFIG_DRAM_SUNXI_TPR5,
	.tpr6 = CONFIG_DRAM_SUNXI_TPR6,
	.tpr7 = CONFIG_DRAM_SUNXI_TPR7,
	.tpr8 = CONFIG_DRAM_SUNXI_TPR8,
	.tpr9 = CONFIG_DRAM_SUNXI_TPR9,
	.tpr10 = CONFIG_DRAM_SUNXI_TPR10,
	.tpr11 = CONFIG_DRAM_SUNXI_TPR11,
	.tpr12 = CONFIG_DRAM_SUNXI_TPR12,
	.tpr13 = CONFIG_DRAM_SUNXI_TPR13,
};

unsigned long sunxi_dram_init(void)
{
	u32 mem_size_mb;

	iw12_sys_cfg();

	iw12_bus_init();

	iw12_ldob_fix();

	iw12_zq_cal(&para);

	/*
	 * auto_scan_dram (BT0 0x107090): with this board's tpr13
	 * (bit14 = rank/width known, bit0 = size known) both probe scans
	 * are skipped and the vendor goes straight to mctl_core_init.
	 * The probe paths (BT0 0x107044 rank/width with para1=0xb000b0,
	 * BT0 0x106dd8 size with para1=0x10e410e4) are not implemented.
	 */

	iw12_dram_clk_init(&para);

	iw12_odt_cfg(&para);	/* no-op on this board (tpr13 bit17) */
	iw12_com_init(&para);

	iw12_com_type_params(&para);

	iw12_set_timing(&para);

	if (!iw12_phy_train(&para))
		panic("sun50iw12: DRAM PHY training error\n");

	mem_size_mb = iw12_finish_controller_tail(&para);
	if (!mem_size_mb)
		panic("sun50iw12: DRAM size clamp returned zero\n");

	if (para.tpr13 & BIT(28)) {
		if (iw12_simple_wr_test(mem_size_mb, 4096))
			panic("sun50iw12: DRAM simple write/read test failed\n");
	}

	return (unsigned long)mem_size_mb * 1024UL * 1024;
}
