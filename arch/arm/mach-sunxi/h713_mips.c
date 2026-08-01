// SPDX-License-Identifier: GPL-2.0+
/*
 * Manual H713 display MIPS firmware loader.
 *
 * The register sequence is recovered from the HY200 factory U-Boot. Keep the
 * command manual until the firmware supplies a readiness witness independent
 * of the CPU status bit. Only the bench-proven display clock and routing
 * prerequisites belong here; LVDS, TVCAP, HDMI, and INCAP remain out of scope.
 */

#include <command.h>
#include <cpu_func.h>
#include <fs.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <vsprintf.h>
#include <sunxi_gpio.h>
#include <linux/string.h>
#include <asm/io.h>
#include <u-boot/sha256.h>

#define H713_MIPS_FW_ADDR		0x4b100000UL
#define H713_MIPS_FW_SIZE		0x00132910UL
#define H713_MIPS_FW_WINDOW_SIZE	0x00500000UL
#define H713_MIPS_BSS_START		0x4b232c00UL
#define H713_MIPS_BSS_END		0x4bac7c40UL
#define H713_MIPS_WITNESS_ADDR		(H713_MIPS_FW_ADDR + \
					 H713_MIPS_FW_WINDOW_SIZE)
#define H713_MIPS_WITNESS_SEED		0x4d495053

#define H713_MIPS_CLK_REG		0x02001600UL
#define H713_MIPS_RESET_REG		0x0200160cUL
#define H713_MIPS_STATUS_REG		0x0306101cUL
#define H713_MIPS_SHARE_ADDR_REG	0x03061024UL
#define H713_MIPS_SHARE_SIZE_REG	0x03061028UL
#define H713_MIPS_BOOTADDR_REG		0x03061030UL

#define H713_MIPS_CLK_VALUE		0x80000002
#define H713_MIPS_CLK_DISABLED		0x00000000
#define H713_MIPS_RESET_ASSERTED	0x00000000
#define H713_MIPS_RESET_STAGE1		0x00010000
#define H713_MIPS_RESET_STAGE2		0x00030000
#define H713_MIPS_RESET_STAGE3		0x00030001
#define H713_MIPS_RESET_RELEASED	0x00070001
#define H713_MIPS_STATUS_RELEASED	0x00000001

/*
 * Workspace layout, from the vendor display_cfg.xml header. The ARM stages the
 * config and TSE windows, so those are the only regions it must not clear.
 * Everything else above the firmware image is uninitialized DRAM that differs
 * per boot, and the firmware reads it — leaving it alone makes runs
 * irreproducible.
 */
#define H713_MIPS_DBG_ADDR		0x4bd01000UL
#define H713_MIPS_CFG_ADDR		0x4be01000UL
#define H713_MIPS_CFG_SIZE		0x00040000UL
#define H713_MIPS_TSE_ADDR		0x4be41000UL
#define H713_MIPS_TSE_SIZE		0x00100000UL
#define H713_MIPS_FB_ADDR		0x4bf41000UL
#define H713_MIPS_FB_SIZE		0x01a00000UL

#define H713_MIPS_SHMEM_ADDR		0x4e300000UL
#define H713_MIPS_SHMEM_SIZE		0x00500000UL
#define H713_MIPS_SHMEM_MAGIC		0xdeadbeef
#define H713_MIPS_SHMEM_MAGIC1_OFF	0x00000090UL
#define H713_MIPS_SHMEM_MAX_CPU_OFF	0x00004cd8UL
#define H713_MIPS_SHMEM_ARM_FLAG_OFF	0x00004cdcUL
#define H713_MIPS_SHMEM_MIPS_FLAG_OFF	0x00004ce0UL
#define H713_MIPS_SHMEM_MAGIC2_OFF	0x000075b8UL
#define H713_MIPS_SHMEM_ARM_READY	(BIT(0) | BIT(2))
#define H713_MIPS_SHMEM_MIPS_READY	BIT(0)
#define H713_MIPS_SHMEM_MIPS_APP_READY	BIT(2)
#define H713_MIPS_SHMEM_LOCK_COUNT	12
#define H713_MIPS_SHMEM_LOCK_SIZE	12
#define H713_MIPS_SHMEM_LOCK_FREE	2
#define H713_MIPS_SHMEM_LOCK_THREAD_NONE	0x000000ff
#define H713_MIPS_SHMEM_CALL_VERSION_OFF	0x000075c0UL
#define H713_MIPS_SHMEM_CALL_COUNT_OFF	0x000075c4UL
#define H713_MIPS_SHMEM_CALL_TABLE_OFF	0x000075c8UL
#define H713_MIPS_SHMEM_CALL_ENTRY_COUNT	1224
#define H713_MIPS_SHMEM_CALL_ENTRY_SIZE	96
#define H713_MIPS_SHMEM_CALL_NEXT_OFF	92
/*
 * One second was chosen when the firmware stalled in its first few hundred
 * milliseconds. With the config and TSE artifacts staged it now runs its whole
 * instrumented startup, so give an uninstrumented run the same budget as a
 * traced one before calling readiness a failure.
 */
#define H713_MIPS_READY_TIMEOUT_US	10000000
#define H713_MIPS_TRACE_TIMEOUT_US	10000000
#define H713_MIPS_DISP_READY_TIMEOUT_US	4000000
#define H713_MIPS_TRACE_OFF		0x00040000UL
#define H713_MIPS_TRACE_MARKER_COUNT	88
#define H713_MIPS_TRACE_DBG_ADDR	88
#define H713_MIPS_TRACE_DBG_SIZE	89
#define H713_MIPS_TRACE_REG_COUNT	90
#define H713_MIPS_TRACE_REG_OBJECT	91
#define H713_MIPS_TRACE_REG_CALLBACK	92
#define H713_MIPS_TRACE_COUNT		93
#define H713_MIPS_STABILITY_SECONDS	60
#define H713_MIPS_DIAG_OFF		0x00041000UL
#define H713_MIPS_DIAG_HEARTBEAT_OFF	(H713_MIPS_DIAG_OFF + 0x00)
#define H713_MIPS_DIAG_EXCEPTION_OFF	(H713_MIPS_DIAG_OFF + 0x04)
#define H713_MIPS_DIAG_STATUS_OFF	(H713_MIPS_DIAG_OFF + 0x08)
#define H713_MIPS_DIAG_CAUSE_OFF	(H713_MIPS_DIAG_OFF + 0x0c)
#define H713_MIPS_DIAG_EPC_OFF		(H713_MIPS_DIAG_OFF + 0x10)
#define H713_MIPS_DIAG_BADVADDR_OFF	(H713_MIPS_DIAG_OFF + 0x14)
#define H713_MIPS_DIAG_EXCEPTION_GENERAL	1
#define H713_MIPS_DIAG_EXCEPTION_CACHE	2

/*
 * Display-fabric prerequisites surrounding MIPS release. Several blocks wedge
 * the interconnect if accessed before the complete parent/module clock tree is
 * enabled. This exact subset was verified with MIPS running and across a Linux
 * handoff with Panfrost enabled.
 */
#define H713_DISPLAY_PLL_VIDEO2_REG	0x02001050UL
#define H713_DISPLAY_DEINT_CLK_REG	0x02001db0UL
#define H713_DISPLAY_PANEL_CLK_REG	0x02001db4UL
#define H713_DISPLAY_SVP_DTL_CLK_REG	0x02001db8UL
#define H713_DISPLAY_AFBD_CLK_REG	0x02001dc0UL
#define H713_DISPLAY_BGR_REG		0x02001dd8UL
#define H713_DISPLAY_TOP_REG		0x05700000UL
#define H713_DISPLAY_MIXER_CTRL_REG	0x0525c038UL

#define H713_DISPLAY_MIXER_CTRL_VALUE	0x00000100

#define H713_TVCAP_TCD3_CLK_REG		0x02001d6cUL
#define H713_TVCAP_VINCAP_DMA_CLK_REG	0x02001d74UL
#define H713_TVCAP_BUS_CLK_REG		0x02001d80UL
#define H713_TVCAP_HDMI_AUDIO_CLK_REG	0x02001d84UL
#define H713_TVCAP_BGR_REG		0x02001d88UL

/*
 * The board's own display.bin, read from its stock bootloader partition. An
 * earlier pin (16c74a28..., 0x132b18 bytes) came from a different firmware
 * revision in a captured dump and does not match this hardware; every run
 * against it was executing a mismatched image.
 */
static const u8 h713_mips_fw_sha256[SHA256_SUM_LEN] = {
	0x43, 0x80, 0xf1, 0xb3, 0xed, 0x7b, 0x62, 0xaa,
	0x50, 0x58, 0x2e, 0x7c, 0xb1, 0x6a, 0x87, 0xbd,
	0xfa, 0xce, 0x1b, 0x43, 0x00, 0x57, 0x8f, 0xe3,
	0x63, 0x1a, 0x41, 0x63, 0x54, 0xda, 0x30, 0xce,
};

static bool h713_display_prepared;

/* Previous value of every trace slot, so streaming reports only changes. */
static u32 trace_shadow[H713_MIPS_TRACE_COUNT];

struct h713_mips_patch {
	ulong addr;
	u32 expected;
	u32 replacement;
};

/*
 * Gate-2 stability instrumentation for the authenticated 4380f1b3... image.
 *
 * The timer trampoline preserves the displaced tick increment and store, then
 * publishes the new ThreadX tick through the MIPS uncached 0xae34xxxx alias.
 * The two exception trampolines record CP0 state before entering the stock
 * fatal handlers. EBase is set to 0x8b101000 at raw +0xb11ac..+0xb11b4, so
 * raw +0x1100 and +0x1180 are the live cache/general exception vectors.
 */
static const struct h713_mips_patch h713_mips_stability_patches[] = {
	/* Timer cave at raw +0x300. */
	{ 0x4b100300, 0x00000000, 0x26730001 }, /* addiu s3, s3, 1 */
	{ 0x4b100304, 0x00000000, 0x3c1aae34 }, /* lui k0, 0xae34 */
	{ 0x4b100308, 0x00000000, 0xaf531000 }, /* sw s3, 0x1000(k0) */
	{ 0x4b10030c, 0x00000000, 0x0ac412e1 }, /* j 0x8b104b84 */
	{ 0x4b100310, 0x00000000, 0xac532cc0 }, /* sw s3, 0x2cc0(v0) */
	{ 0x4b104b7c, 0x26730001, 0x0ac400c0 }, /* j 0x8b100300 */

	/* General-exception cave at raw +0x320; type 1 is stored last. */
	{ 0x4b100320, 0x00000000, 0x3c1aae34 }, /* lui k0, 0xae34 */
	{ 0x4b100324, 0x00000000, 0x401b6000 }, /* mfc0 k1, Status */
	{ 0x4b100328, 0x00000000, 0xaf5b1008 },
	{ 0x4b10032c, 0x00000000, 0x401b6800 }, /* mfc0 k1, Cause */
	{ 0x4b100330, 0x00000000, 0xaf5b100c },
	{ 0x4b100334, 0x00000000, 0x401b7000 }, /* mfc0 k1, EPC */
	{ 0x4b100338, 0x00000000, 0xaf5b1010 },
	{ 0x4b10033c, 0x00000000, 0x401b4000 }, /* mfc0 k1, BadVAddr */
	{ 0x4b100340, 0x00000000, 0xaf5b1014 },
	{ 0x4b100344, 0x00000000, 0x341b0001 },
	{ 0x4b100348, 0x00000000, 0xaf5b1004 },
	{ 0x4b10034c, 0x00000000, 0x0ac56f4a }, /* j 0x8b15bd28 */
	{ 0x4b100350, 0x00000000, 0x00000000 },
	{ 0x4b101180, 0x0ac56f4a, 0x0ac400c8 }, /* j 0x8b100320 */

	/* Cache-error cave at raw +0x360; EPC field receives ErrorEPC. */
	{ 0x4b100360, 0x00000000, 0x3c1aae34 }, /* lui k0, 0xae34 */
	{ 0x4b100364, 0x00000000, 0x401b6000 }, /* mfc0 k1, Status */
	{ 0x4b100368, 0x00000000, 0xaf5b1008 },
	{ 0x4b10036c, 0x00000000, 0x401b6800 }, /* mfc0 k1, Cause */
	{ 0x4b100370, 0x00000000, 0xaf5b100c },
	{ 0x4b100374, 0x00000000, 0x401bf000 }, /* mfc0 k1, ErrorEPC */
	{ 0x4b100378, 0x00000000, 0xaf5b1010 },
	{ 0x4b10037c, 0x00000000, 0x401b4000 }, /* mfc0 k1, BadVAddr */
	{ 0x4b100380, 0x00000000, 0xaf5b1014 },
	{ 0x4b100384, 0x00000000, 0x341b0002 },
	{ 0x4b100388, 0x00000000, 0xaf5b1004 },
	{ 0x4b10038c, 0x00000000, 0x0ac56f79 }, /* j 0x8b15bde4 */
	{ 0x4b100390, 0x00000000, 0x00000000 },
	{ 0x4b101100, 0x0ac56f79, 0x0ac400d8 }, /* j 0x8b100360 */
};

/*
 * Legacy trace for display.bin 16c74a28..., retained only as a reverse-
 * engineering record. The board's 4380f1b3... image moved the instrumented
 * functions, so none of these sites may be installed in executable DRAM.
 */
#if 0
/*
 * Volatile display.bin trace used only by "probe-trace". The pristine image is
 * authenticated before these words are installed in DRAM. Three tiny caves
 * and two inline stores mark:
 *
 *  1. share address absent (the polling path)
 *  2. share address and size accepted
 *  3. ARM CPU_READY accepted
 *  4. CPU_COMM hardware spinlock 0 acquired
 *  5. slave-side CPU_COMM initialization entered
 *  6. ThreadX application entry reached
 *  7. early application wrapper entered
 *  8. application byte-pool creation succeeded
 *  9. share-register reader called
 * 10. C runtime handed off to platform initialization
 * 11. constructor-table initialization completed
 * 12. early OS initialization completed
 * 13. platform service initialization completed
 * 14. application/thread construction entered
 * 15. application/thread construction completed
 * 16-25. successive calls within application/thread construction entered
 * 26. display object allocation entered
 * 27. display object construction entered
 * 28. display object resource discovery entered
 * 29-31. resource discovery's first three calls entered
 * 32-42. successive early system-initialization calls entered
 * 43-61. remaining early system-initialization calls entered
 * 62-67. device-manager allocation and factory calls entered
 * 68-83. HDMI receiver factory and constructor calls entered
 * 84-92. first HDMI receiver port's construction calls entered
 * 93-96. HDMI receiver MMIO address validation and first byte load
 * 97-102. HDMI receiver helper returns, tail calls, write, and timer entry
 * 103-106. polling-loop timer entry, return, current tick, and initial tick
 * 107-110. platform-registration callback targets and table positions
 * 111-112. marker-49 singleton allocation and construction
 * 130-132. the four-registration group's ids 0x130, 3, and 8 returning
 *
 * The marker stores target the firmware's uncached 0xae340000 alias, visible
 * to the ARM at H713_MIPS_SHMEM_ADDR + H713_MIPS_TRACE_OFF.
 */
static const struct h713_mips_patch h713_mips_trace_patches[] = {
	/* Cave 0x8b1002c4: marker 1, then return after the skipped log call. */
	{ 0x4b1002c4, 0x00000000, 0x3c1aae34 },
	{ 0x4b1002c8, 0x00000000, 0x341b0001 },
	{ 0x4b1002cc, 0x00000000, 0xaf5b0000 },
	{ 0x4b1002d0, 0x00000000, 0x0ac48e38 },
	{ 0x4b1002d4, 0x00000000, 0x00000000 },
	{ 0x4b1238d8, 0x0ec5401a, 0x0ac400b1 },

	/* Cave 0x8b100500: marker 2, then return after the skipped log call. */
	{ 0x4b100500, 0x00000000, 0x3c1aae34 },
	{ 0x4b100504, 0x00000000, 0x341b0002 },
	{ 0x4b100508, 0x00000000, 0xaf5b0004 },
	{ 0x4b10050c, 0x00000000, 0x0ac48e59 },
	{ 0x4b100510, 0x00000000, 0x00000000 },
	{ 0x4b12395c, 0x0ec5401a, 0x0ac40140 },

	/* Marker 3 in the disposable argument setup for the return log. */
	{ 0x4b12397c, 0x2687fd18, 0x3c1aae34 },
	{ 0x4b123980, 0x2666fa64, 0x341b0003 },
	{ 0x4b123988, 0x3c028b1f, 0xaf5b0008 },
	{ 0x4b12398c, 0x2442e040, 0x00000000 },
	{ 0x4b123990, 0xafa20014, 0x00000000 },

	/* Marker 4 immediately after CPU_COMM hardware spinlock 0 succeeds. */
	{ 0x4b11b07c, 0x8fc20090, 0x3c1aae34 },
	{ 0x4b11b080, 0x8fc475b8, 0x341b0004 },
	{ 0x4b11b084, 0x02e03825, 0xaf5b000c },

	/* Cave 0x8b100520: marker 5, then tail-call the original function. */
	{ 0x4b100520, 0x00000000, 0x3c1aae34 },
	{ 0x4b100524, 0x00000000, 0x341b0005 },
	{ 0x4b100528, 0x00000000, 0xaf5b0010 },
	{ 0x4b10052c, 0x00000000, 0x0ac466a2 },
	{ 0x4b100530, 0x00000000, 0x00000000 },
	{ 0x4b11b220, 0x0ec466a2, 0x0ec40148 },

	/* Cave 0x8b100540: marker 6, then tail-call the original timer read. */
	{ 0x4b100540, 0x00000000, 0x3c1aae34 },
	{ 0x4b100544, 0x00000000, 0x341b0006 },
	{ 0x4b100548, 0x00000000, 0xaf5b0014 },
	{ 0x4b10054c, 0x00000000, 0x0ac56f65 },
	{ 0x4b100550, 0x00000000, 0x00000000 },
	{ 0x4b1525a4, 0x0ec56f65, 0x0ec40150 },

	/* Cave 0x8b100560: marker 7, then enter the original wrapper. */
	{ 0x4b100560, 0x00000000, 0x3c1aae34 },
	{ 0x4b100564, 0x00000000, 0x341b0007 },
	{ 0x4b100568, 0x00000000, 0xaf5b0018 },
	{ 0x4b10056c, 0x00000000, 0x0ac4908f },
	{ 0x4b100570, 0x00000000, 0x00000000 },
	{ 0x4b1525ac, 0x0ec4908f, 0x0ec40158 },

	/* Cave 0x8b100590: marker 8 on the byte-pool success path. */
	{ 0x4b100590, 0x00000000, 0x3c1aae34 },
	{ 0x4b100594, 0x00000000, 0x341b0008 },
	{ 0x4b100598, 0x00000000, 0xaf5b001c },
	{ 0x4b10059c, 0x00000000, 0x0ac490c6 },
	{ 0x4b1005a0, 0x00000000, 0x00000000 },
	{ 0x4b124310, 0x0ec5401a, 0x0ac40164 },

	/* Cave 0x8b1005b0: marker 9, then call the share-register reader. */
	{ 0x4b1005b0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1005b4, 0x00000000, 0x341b0009 },
	{ 0x4b1005b8, 0x00000000, 0xaf5b0020 },
	{ 0x4b1005bc, 0x00000000, 0x0ac48e0a },
	{ 0x4b1005c0, 0x00000000, 0x00000000 },
	{ 0x4b12431c, 0x0ec48e0a, 0x0ec4016c },

	/* Caves 0x8b1005d0..0x8b100670 trace the pre-scheduler calls. */
	{ 0x4b1005d0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1005d4, 0x00000000, 0x341b000a },
	{ 0x4b1005d8, 0x00000000, 0xaf5b0024 },
	{ 0x4b1005dc, 0x00000000, 0x0ac407c1 },
	{ 0x4b1005e0, 0x00000000, 0x00000000 },
	{ 0x4b1b0940, 0x0100f809, 0x0ec40174 },

	{ 0x4b1005f0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1005f4, 0x00000000, 0x341b000b },
	{ 0x4b1005f8, 0x00000000, 0xaf5b0028 },
	{ 0x4b1005fc, 0x00000000, 0x0ac40b86 },
	{ 0x4b100600, 0x00000000, 0x00000000 },
	{ 0x4b101f24, 0x0ec40b86, 0x0ec4017c },

	{ 0x4b100610, 0x00000000, 0x3c1aae34 },
	{ 0x4b100614, 0x00000000, 0x341b000c },
	{ 0x4b100618, 0x00000000, 0xaf5b002c },
	{ 0x4b10061c, 0x00000000, 0x0ac51e54 },
	{ 0x4b100620, 0x00000000, 0x00000000 },
	{ 0x4b101f2c, 0x0ec51e54, 0x0ec40184 },

	{ 0x4b100630, 0x00000000, 0x3c1aae34 },
	{ 0x4b100634, 0x00000000, 0x341b000d },
	{ 0x4b100638, 0x00000000, 0xaf5b0030 },
	{ 0x4b10063c, 0x00000000, 0x0ac61091 },
	{ 0x4b100640, 0x00000000, 0x00000000 },
	{ 0x4b101f34, 0x0ec61091, 0x0ec4018c },

	{ 0x4b100650, 0x00000000, 0x3c1aae34 },
	{ 0x4b100654, 0x00000000, 0x341b000e },
	{ 0x4b100658, 0x00000000, 0xaf5b0034 },
	{ 0x4b10065c, 0x00000000, 0x0ac54b37 },
	{ 0x4b100660, 0x00000000, 0x00000000 },
	{ 0x4b101f3c, 0x0ec54b37, 0x0ec40194 },

	{ 0x4b100670, 0x00000000, 0x3c1aae34 },
	{ 0x4b100674, 0x00000000, 0x341b000f },
	{ 0x4b100678, 0x00000000, 0xaf5b0038 },
	{ 0x4b10067c, 0x00000000, 0x0ac415e4 },
	{ 0x4b100680, 0x00000000, 0x00000000 },
	{ 0x4b101f4c, 0x0ec415e4, 0x0ec4019c },

	/* Caves 0x8b100690..0x8b1007b0 trace construction's calls. */
	{ 0x4b100690, 0x00000000, 0x3c1aae34 },
	{ 0x4b100694, 0x00000000, 0x341b0010 },
	{ 0x4b100698, 0x00000000, 0xaf5b003c },
	{ 0x4b10069c, 0x00000000, 0x0ac672b9 },
	{ 0x4b1006a0, 0x00000000, 0x00000000 },
	{ 0x4b152ce4, 0x0ec672b9, 0x0ec401a4 },

	{ 0x4b1006b0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1006b4, 0x00000000, 0x341b0011 },
	{ 0x4b1006b8, 0x00000000, 0xaf5b0040 },
	{ 0x4b1006bc, 0x00000000, 0x0ac439dc },
	{ 0x4b1006c0, 0x00000000, 0x00000000 },
	{ 0x4b152cf0, 0x0ec439dc, 0x0ec401ac },

	{ 0x4b1006d0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1006d4, 0x00000000, 0x341b0012 },
	{ 0x4b1006d8, 0x00000000, 0xaf5b0044 },
	{ 0x4b1006dc, 0x00000000, 0x0ac43503 },
	{ 0x4b1006e0, 0x00000000, 0x00000000 },
	{ 0x4b152cf8, 0x0ec43503, 0x0ec401b4 },

	{ 0x4b1006f0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1006f4, 0x00000000, 0x341b0013 },
	{ 0x4b1006f8, 0x00000000, 0xaf5b0048 },
	{ 0x4b1006fc, 0x00000000, 0x0ac43511 },
	{ 0x4b100700, 0x00000000, 0x00000000 },
	{ 0x4b152d00, 0x0ec43511, 0x0ec401bc },

	{ 0x4b100710, 0x00000000, 0x3c1aae34 },
	{ 0x4b100714, 0x00000000, 0x341b0014 },
	{ 0x4b100718, 0x00000000, 0xaf5b004c },
	{ 0x4b10071c, 0x00000000, 0x0ac56698 },
	{ 0x4b100720, 0x00000000, 0x00000000 },
	{ 0x4b152d10, 0x0ec56698, 0x0ec401c4 },

	{ 0x4b100730, 0x00000000, 0x3c1aae34 },
	{ 0x4b100734, 0x00000000, 0x341b0015 },
	{ 0x4b100738, 0x00000000, 0xaf5b0050 },
	{ 0x4b10073c, 0x00000000, 0x0ac54e20 },
	{ 0x4b100740, 0x00000000, 0x00000000 },
	{ 0x4b152d18, 0x0ec54e20, 0x0ec401cc },

	{ 0x4b100750, 0x00000000, 0x3c1aae34 },
	{ 0x4b100754, 0x00000000, 0x341b0016 },
	{ 0x4b100758, 0x00000000, 0xaf5b0054 },
	{ 0x4b10075c, 0x00000000, 0x0ac5440b },
	{ 0x4b100760, 0x00000000, 0x00000000 },
	{ 0x4b152d20, 0x0ec5440b, 0x0ec401d4 },

	{ 0x4b100770, 0x00000000, 0x3c1aae34 },
	{ 0x4b100774, 0x00000000, 0x341b0017 },
	{ 0x4b100778, 0x00000000, 0xaf5b0058 },
	{ 0x4b10077c, 0x00000000, 0x0ac60af9 },
	{ 0x4b100780, 0x00000000, 0x00000000 },
	{ 0x4b152d28, 0x0ec60af9, 0x0ec401dc },

	{ 0x4b100790, 0x00000000, 0x3c1aae34 },
	{ 0x4b100794, 0x00000000, 0x341b0018 },
	{ 0x4b100798, 0x00000000, 0xaf5b005c },
	{ 0x4b10079c, 0x00000000, 0x0ac54acb },
	{ 0x4b1007a0, 0x00000000, 0x00000000 },
	{ 0x4b152d30, 0x0ec54acb, 0x0ec401e4 },

	{ 0x4b1007b0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1007b4, 0x00000000, 0x341b0019 },
	{ 0x4b1007b8, 0x00000000, 0xaf5b0060 },
	{ 0x4b1007bc, 0x00000000, 0x0ac56fed },
	{ 0x4b1007c0, 0x00000000, 0x00000000 },
	{ 0x4b152d54, 0x0ec56fed, 0x0ec401ec },

	/* Trace the allocation and construction calls below marker 17. */
	{ 0x4b1007d0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1007d4, 0x00000000, 0x341b001a },
	{ 0x4b1007d8, 0x00000000, 0xaf5b0064 },
	{ 0x4b1007dc, 0x00000000, 0x0ac6b54d },
	{ 0x4b1007e0, 0x00000000, 0x00000000 },
	{ 0x4b10e7b0, 0x0ec6b54d, 0x0ec401f4 },

	{ 0x4b1007f0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1007f4, 0x00000000, 0x341b001b },
	{ 0x4b1007f8, 0x00000000, 0xaf5b0068 },
	{ 0x4b1007fc, 0x00000000, 0x0ac439c1 },
	{ 0x4b100800, 0x00000000, 0x00000000 },
	{ 0x4b10e7c0, 0x0ec439c1, 0x0ec401fc },

	{ 0x4b100810, 0x00000000, 0x3c1aae34 },
	{ 0x4b100814, 0x00000000, 0x341b001c },
	{ 0x4b100818, 0x00000000, 0xaf5b006c },
	{ 0x4b10081c, 0x00000000, 0x0ac43554 },
	{ 0x4b100820, 0x00000000, 0x00000000 },
	{ 0x4b10e73c, 0x0ec43554, 0x0ec40204 },

	/* Trace the first resource-discovery calls below marker 28. */
	{ 0x4b100830, 0x00000000, 0x3c1aae34 },
	{ 0x4b100834, 0x00000000, 0x341b001d },
	{ 0x4b100838, 0x00000000, 0xaf5b0070 },
	{ 0x4b10083c, 0x00000000, 0x0ac452a3 },
	{ 0x4b100840, 0x00000000, 0x00000000 },
	{ 0x4b10d584, 0x0ec452a3, 0x0ec4020c },

	{ 0x4b100850, 0x00000000, 0x3c1aae34 },
	{ 0x4b100854, 0x00000000, 0x341b001e },
	{ 0x4b100858, 0x00000000, 0xaf5b0074 },
	{ 0x4b10085c, 0x00000000, 0x0ac4573b },
	{ 0x4b100860, 0x00000000, 0x00000000 },
	{ 0x4b10d598, 0x0ec4573b, 0x0ec40214 },

	{ 0x4b100870, 0x00000000, 0x3c1aae34 },
	{ 0x4b100874, 0x00000000, 0x341b001f },
	{ 0x4b100878, 0x00000000, 0xaf5b0078 },
	{ 0x4b10087c, 0x00000000, 0x0ac45030 },
	{ 0x4b100880, 0x00000000, 0x00000000 },
	{ 0x4b10d5a4, 0x0ec45030, 0x0ec4021c },

	/* Trace the early system-initialization calls below marker 24. */
	{ 0x4b100890, 0x00000000, 0x3c1aae34 },
	{ 0x4b100894, 0x00000000, 0x341b0020 },
	{ 0x4b100898, 0x00000000, 0xaf5b007c },
	{ 0x4b10089c, 0x00000000, 0x0ac5ff44 },
	{ 0x4b1008a0, 0x00000000, 0x00000000 },
	{ 0x4b152b64, 0x0ec5ff44, 0x0ec40224 },

	{ 0x4b1008b0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1008b4, 0x00000000, 0x341b0021 },
	{ 0x4b1008b8, 0x00000000, 0xaf5b0080 },
	{ 0x4b1008bc, 0x00000000, 0x0ac43500 },
	{ 0x4b1008c0, 0x00000000, 0x00000000 },
	{ 0x4b152b74, 0x0ec43500, 0x0ec4022c },

	{ 0x4b1008d0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1008d4, 0x00000000, 0x341b0022 },
	{ 0x4b1008d8, 0x00000000, 0xaf5b0084 },
	{ 0x4b1008dc, 0x00000000, 0x00400008 },
	{ 0x4b1008e0, 0x00000000, 0x00000000 },
	{ 0x4b152b90, 0x0040f809, 0x0ec40234 },

	{ 0x4b1008f0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1008f4, 0x00000000, 0x341b0023 },
	{ 0x4b1008f8, 0x00000000, 0xaf5b0088 },
	{ 0x4b1008fc, 0x00000000, 0x0ac43500 },
	{ 0x4b100900, 0x00000000, 0x00000000 },
	{ 0x4b152b98, 0x0ec43500, 0x0ec4023c },

	{ 0x4b100910, 0x00000000, 0x3c1aae34 },
	{ 0x4b100914, 0x00000000, 0x341b0024 },
	{ 0x4b100918, 0x00000000, 0xaf5b008c },
	{ 0x4b10091c, 0x00000000, 0x00400008 },
	{ 0x4b100920, 0x00000000, 0x00000000 },
	{ 0x4b152bb4, 0x0040f809, 0x0ec40244 },

	{ 0x4b100930, 0x00000000, 0x3c1aae34 },
	{ 0x4b100934, 0x00000000, 0x341b0025 },
	{ 0x4b100938, 0x00000000, 0xaf5b0090 },
	{ 0x4b10093c, 0x00000000, 0x0ac5ff56 },
	{ 0x4b100940, 0x00000000, 0x00000000 },
	{ 0x4b152bc4, 0x0ec5ff56, 0x0ec4024c },

	{ 0x4b100950, 0x00000000, 0x3c1aae34 },
	{ 0x4b100954, 0x00000000, 0x341b0026 },
	{ 0x4b100958, 0x00000000, 0xaf5b0094 },
	{ 0x4b10095c, 0x00000000, 0x0ac5ff56 },
	{ 0x4b100960, 0x00000000, 0x00000000 },
	{ 0x4b152bd4, 0x0ec5ff56, 0x0ec40254 },

	{ 0x4b100970, 0x00000000, 0x3c1aae34 },
	{ 0x4b100974, 0x00000000, 0x341b0027 },
	{ 0x4b100978, 0x00000000, 0xaf5b0098 },
	{ 0x4b10097c, 0x00000000, 0x0ac625f6 },
	{ 0x4b100980, 0x00000000, 0x00000000 },
	{ 0x4b152be0, 0x0ec625f6, 0x0ec4025c },

	{ 0x4b100990, 0x00000000, 0x3c1aae34 },
	{ 0x4b100994, 0x00000000, 0x341b0028 },
	{ 0x4b100998, 0x00000000, 0xaf5b009c },
	{ 0x4b10099c, 0x00000000, 0x0ac62345 },
	{ 0x4b1009a0, 0x00000000, 0x00000000 },
	{ 0x4b152be8, 0x0ec62345, 0x0ec40264 },

	{ 0x4b1009b0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1009b4, 0x00000000, 0x341b0029 },
	{ 0x4b1009b8, 0x00000000, 0xaf5b00a0 },
	{ 0x4b1009bc, 0x00000000, 0x0ac43500 },
	{ 0x4b1009c0, 0x00000000, 0x00000000 },
	{ 0x4b152bf0, 0x0ec43500, 0x0ec4026c },

	{ 0x4b1009d0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1009d4, 0x00000000, 0x341b002a },
	{ 0x4b1009d8, 0x00000000, 0xaf5b00a4 },
	{ 0x4b1009dc, 0x00000000, 0x00400008 },
	{ 0x4b1009e0, 0x00000000, 0x00000000 },
	{ 0x4b152c0c, 0x0040f809, 0x0ec40274 },

	/* Trace the remaining system-initialization calls below marker 42. */
	{ 0x4b1009f0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1009f4, 0x00000000, 0x341b002b },
	{ 0x4b1009f8, 0x00000000, 0xaf5b00a8 },
	{ 0x4b1009fc, 0x00000000, 0x0ac440d3 },
	{ 0x4b100a00, 0x00000000, 0x00000000 },
	{ 0x4b152c1c, 0x0ec440d3, 0x0ec4027c },

	{ 0x4b100a10, 0x00000000, 0x3c1aae34 },
	{ 0x4b100a14, 0x00000000, 0x341b002c },
	{ 0x4b100a18, 0x00000000, 0xaf5b00ac },
	{ 0x4b100a1c, 0x00000000, 0x0ac54a97 },
	{ 0x4b100a20, 0x00000000, 0x00000000 },
	{ 0x4b152c24, 0x0ec54a97, 0x0ec40284 },

	{ 0x4b100a30, 0x00000000, 0x3c1aae34 },
	{ 0x4b100a34, 0x00000000, 0x341b002d },
	{ 0x4b100a38, 0x00000000, 0xaf5b00b0 },
	{ 0x4b100a3c, 0x00000000, 0x0ac62571 },
	{ 0x4b100a40, 0x00000000, 0x00000000 },
	{ 0x4b152c2c, 0x0ec62571, 0x0ec4028c },

	{ 0x4b100a50, 0x00000000, 0x3c1aae34 },
	{ 0x4b100a54, 0x00000000, 0x341b002e },
	{ 0x4b100a58, 0x00000000, 0xaf5b00b4 },
	{ 0x4b100a5c, 0x00000000, 0x00400008 },
	{ 0x4b100a60, 0x00000000, 0x00000000 },
	{ 0x4b152c40, 0x0040f809, 0x0ec40294 },

	{ 0x4b100a70, 0x00000000, 0x3c1aae34 },
	{ 0x4b100a74, 0x00000000, 0x341b002f },
	{ 0x4b100a78, 0x00000000, 0xaf5b00b8 },
	{ 0x4b100a7c, 0x00000000, 0x0ac69cb7 },
	{ 0x4b100a80, 0x00000000, 0x00000000 },
	{ 0x4b152c48, 0x0ec69cb7, 0x0ec4029c },

	{ 0x4b100a90, 0x00000000, 0x3c1aae34 },
	{ 0x4b100a94, 0x00000000, 0x341b0030 },
	{ 0x4b100a98, 0x00000000, 0xaf5b00bc },
	{ 0x4b100a9c, 0x00000000, 0x0ac55f86 },
	{ 0x4b100aa0, 0x00000000, 0x00000000 },
	{ 0x4b152c50, 0x0ec55f86, 0x0ec402a4 },

	{ 0x4b100ab0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100ab4, 0x00000000, 0x341b0031 },
	{ 0x4b100ab8, 0x00000000, 0xaf5b00c0 },
	{ 0x4b100abc, 0x00000000, 0x0ac4a0d3 },
	{ 0x4b100ac0, 0x00000000, 0x00000000 },
	{ 0x4b152c58, 0x0ec4a0d3, 0x0ec402ac },

	{ 0x4b100ad0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100ad4, 0x00000000, 0x341b0032 },
	{ 0x4b100ad8, 0x00000000, 0xaf5b00c4 },
	{ 0x4b100adc, 0x00000000, 0x0ac6778c },
	{ 0x4b100ae0, 0x00000000, 0x00000000 },
	{ 0x4b152c60, 0x0ec6778c, 0x0ec402b4 },

	{ 0x4b100af0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100af4, 0x00000000, 0x341b0033 },
	{ 0x4b100af8, 0x00000000, 0xaf5b00c8 },
	{ 0x4b100afc, 0x00000000, 0x0ac5714f },
	{ 0x4b100b00, 0x00000000, 0x00000000 },
	{ 0x4b152c68, 0x0ec5714f, 0x0ec402bc },

	{ 0x4b100b10, 0x00000000, 0x3c1aae34 },
	{ 0x4b100b14, 0x00000000, 0x341b0034 },
	{ 0x4b100b18, 0x00000000, 0xaf5b00cc },
	{ 0x4b100b1c, 0x00000000, 0x0ac5eaab },
	{ 0x4b100b20, 0x00000000, 0x00000000 },
	{ 0x4b152c70, 0x0ec5eaab, 0x0ec402c4 },

	{ 0x4b100b30, 0x00000000, 0x3c1aae34 },
	{ 0x4b100b34, 0x00000000, 0x341b0035 },
	{ 0x4b100b38, 0x00000000, 0xaf5b00d0 },
	{ 0x4b100b3c, 0x00000000, 0x0ac60e77 },
	{ 0x4b100b40, 0x00000000, 0x00000000 },
	{ 0x4b152c78, 0x0ec60e77, 0x0ec402cc },

	{ 0x4b100b50, 0x00000000, 0x3c1aae34 },
	{ 0x4b100b54, 0x00000000, 0x341b0036 },
	{ 0x4b100b58, 0x00000000, 0xaf5b00d4 },
	{ 0x4b100b5c, 0x00000000, 0x0ac61003 },
	{ 0x4b100b60, 0x00000000, 0x00000000 },
	{ 0x4b152c80, 0x0ec61003, 0x0ec402d4 },

	{ 0x4b100b70, 0x00000000, 0x3c1aae34 },
	{ 0x4b100b74, 0x00000000, 0x341b0037 },
	{ 0x4b100b78, 0x00000000, 0xaf5b00d8 },
	{ 0x4b100b7c, 0x00000000, 0x0ac5f0d9 },
	{ 0x4b100b80, 0x00000000, 0x00000000 },
	{ 0x4b152c88, 0x0ec5f0d9, 0x0ec402dc },

	{ 0x4b100b90, 0x00000000, 0x3c1aae34 },
	{ 0x4b100b94, 0x00000000, 0x341b0038 },
	{ 0x4b100b98, 0x00000000, 0xaf5b00dc },
	{ 0x4b100b9c, 0x00000000, 0x0ac6768e },
	{ 0x4b100ba0, 0x00000000, 0x00000000 },
	{ 0x4b152c90, 0x0ec6768e, 0x0ec402e4 },

	{ 0x4b100bb0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100bb4, 0x00000000, 0x341b0039 },
	{ 0x4b100bb8, 0x00000000, 0xaf5b00e0 },
	{ 0x4b100bbc, 0x00000000, 0x0ac6af28 },
	{ 0x4b100bc0, 0x00000000, 0x00000000 },
	{ 0x4b152c98, 0x0ec6af28, 0x0ec402ec },

	{ 0x4b100bd0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100bd4, 0x00000000, 0x341b003a },
	{ 0x4b100bd8, 0x00000000, 0xaf5b00e4 },
	{ 0x4b100bdc, 0x00000000, 0x0ac54dad },
	{ 0x4b100be0, 0x00000000, 0x00000000 },
	{ 0x4b152ca0, 0x0ec54dad, 0x0ec402f4 },

	{ 0x4b100bf0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100bf4, 0x00000000, 0x341b003b },
	{ 0x4b100bf8, 0x00000000, 0xaf5b00e8 },
	{ 0x4b100bfc, 0x00000000, 0x0ac41bdf },
	{ 0x4b100c00, 0x00000000, 0x00000000 },
	{ 0x4b152cb0, 0x0ec41bdf, 0x0ec402fc },

	{ 0x4b100c10, 0x00000000, 0x3c1aae34 },
	{ 0x4b100c14, 0x00000000, 0x341b003c },
	{ 0x4b100c18, 0x00000000, 0xaf5b00ec },
	{ 0x4b100c1c, 0x00000000, 0x0ac41adf },
	{ 0x4b100c20, 0x00000000, 0x00000000 },
	{ 0x4b152cb8, 0x0ec41adf, 0x0ec40304 },

	{ 0x4b100c30, 0x00000000, 0x3c1aae34 },
	{ 0x4b100c34, 0x00000000, 0x341b003d },
	{ 0x4b100c38, 0x00000000, 0xaf5b00f0 },
	{ 0x4b100c3c, 0x00000000, 0x0ac423ed },
	{ 0x4b100c40, 0x00000000, 0x00000000 },
	{ 0x4b152cc0, 0x0ec423ed, 0x0ec4030c },

	/* Trace device-manager allocation and its four factory calls. */
	{ 0x4b100c50, 0x00000000, 0x3c1aae34 },
	{ 0x4b100c54, 0x00000000, 0x341b003e },
	{ 0x4b100c58, 0x00000000, 0xaf5b00f4 },
	{ 0x4b100c5c, 0x00000000, 0x0ac6b54d },
	{ 0x4b100c60, 0x00000000, 0x00000000 },
	{ 0x4b183a10, 0x0ec6b54d, 0x0ec40314 },

	{ 0x4b100c70, 0x00000000, 0x3c1aae34 },
	{ 0x4b100c74, 0x00000000, 0x341b003f },
	{ 0x4b100c78, 0x00000000, 0xaf5b00f8 },
	{ 0x4b100c7c, 0x00000000, 0x0ac60c33 },
	{ 0x4b100c80, 0x00000000, 0x00000000 },
	{ 0x4b183a1c, 0x0ec60c33, 0x0ec4031c },

	{ 0x4b100c90, 0x00000000, 0x3c1aae34 },
	{ 0x4b100c94, 0x00000000, 0x341b0040 },
	{ 0x4b100c98, 0x00000000, 0xaf5b00fc },
	{ 0x4b100c9c, 0x00000000, 0x0ac51818 },
	{ 0x4b100ca0, 0x00000000, 0x00000000 },
	{ 0x4b18310c, 0x0ec51818, 0x0ec40324 },

	{ 0x4b100cb0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100cb4, 0x00000000, 0x341b0041 },
	{ 0x4b100cb8, 0x00000000, 0xaf5b0100 },
	{ 0x4b100cbc, 0x00000000, 0x0ac51bba },
	{ 0x4b100cc0, 0x00000000, 0x00000000 },
	{ 0x4b18311c, 0x0ec51bba, 0x0ec4032c },

	{ 0x4b100cd0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100cd4, 0x00000000, 0x341b0042 },
	{ 0x4b100cd8, 0x00000000, 0xaf5b0104 },
	{ 0x4b100cdc, 0x00000000, 0x0ac4c6df },
	{ 0x4b100ce0, 0x00000000, 0x00000000 },
	{ 0x4b18312c, 0x0ec4c6df, 0x0ec40334 },

	{ 0x4b100cf0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100cf4, 0x00000000, 0x341b0043 },
	{ 0x4b100cf8, 0x00000000, 0xaf5b0108 },
	{ 0x4b100cfc, 0x00000000, 0x0ac4c22b },
	{ 0x4b100d00, 0x00000000, 0x00000000 },
	{ 0x4b18313c, 0x0ec4c22b, 0x0ec4033c },

	/* Trace the HDMI receiver factory and constructor calls. */
	{ 0x4b100d10, 0x00000000, 0x3c1aae34 },
	{ 0x4b100d14, 0x00000000, 0x341b0044 },
	{ 0x4b100d18, 0x00000000, 0xaf5b010c },
	{ 0x4b100d1c, 0x00000000, 0x0ac5401a },
	{ 0x4b100d20, 0x00000000, 0x00000000 },
	{ 0x4b131bb8, 0x0ec5401a, 0x0ec40344 },

	{ 0x4b100d30, 0x00000000, 0x3c1aae34 },
	{ 0x4b100d34, 0x00000000, 0x341b0045 },
	{ 0x4b100d38, 0x00000000, 0xaf5b0110 },
	{ 0x4b100d3c, 0x00000000, 0x0ac6b54d },
	{ 0x4b100d40, 0x00000000, 0x00000000 },
	{ 0x4b131bc0, 0x0ec6b54d, 0x0ec4034c },

	{ 0x4b100d50, 0x00000000, 0x3c1aae34 },
	{ 0x4b100d54, 0x00000000, 0x341b0046 },
	{ 0x4b100d58, 0x00000000, 0xaf5b0114 },
	{ 0x4b100d5c, 0x00000000, 0x0ac4c656 },
	{ 0x4b100d60, 0x00000000, 0x00000000 },
	{ 0x4b131bcc, 0x0ec4c656, 0x0ec40354 },

	{ 0x4b100d70, 0x00000000, 0x3c1aae34 },
	{ 0x4b100d74, 0x00000000, 0x341b0047 },
	{ 0x4b100d78, 0x00000000, 0xaf5b0118 },
	{ 0x4b100d7c, 0x00000000, 0x0ac60b3e },
	{ 0x4b100d80, 0x00000000, 0x00000000 },
	{ 0x4b13197c, 0x0ec60b3e, 0x0ec4035c },

	{ 0x4b100d90, 0x00000000, 0x3c1aae34 },
	{ 0x4b100d94, 0x00000000, 0x341b0048 },
	{ 0x4b100d98, 0x00000000, 0xaf5b011c },
	{ 0x4b100d9c, 0x00000000, 0x0ac5401a },
	{ 0x4b100da0, 0x00000000, 0x00000000 },
	{ 0x4b131a2c, 0x0ec5401a, 0x0ec40364 },

	{ 0x4b100db0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100db4, 0x00000000, 0x341b0049 },
	{ 0x4b100db8, 0x00000000, 0xaf5b0120 },
	{ 0x4b100dbc, 0x00000000, 0x0ac6b54d },
	{ 0x4b100dc0, 0x00000000, 0x00000000 },
	{ 0x4b131a38, 0x0ec6b54d, 0x0ec4036c },

	{ 0x4b100dd0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100dd4, 0x00000000, 0x341b004a },
	{ 0x4b100dd8, 0x00000000, 0xaf5b0124 },
	{ 0x4b100ddc, 0x00000000, 0x0ac4cc04 },
	{ 0x4b100de0, 0x00000000, 0x00000000 },
	{ 0x4b131a48, 0x0ec4cc04, 0x0ec40374 },

	{ 0x4b100df0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100df4, 0x00000000, 0x341b004b },
	{ 0x4b100df8, 0x00000000, 0xaf5b0128 },
	{ 0x4b100dfc, 0x00000000, 0x0ac6b54d },
	{ 0x4b100e00, 0x00000000, 0x00000000 },
	{ 0x4b131a54, 0x0ec6b54d, 0x0ec4037c },

	{ 0x4b100e10, 0x00000000, 0x3c1aae34 },
	{ 0x4b100e14, 0x00000000, 0x341b004c },
	{ 0x4b100e18, 0x00000000, 0xaf5b012c },
	{ 0x4b100e1c, 0x00000000, 0x0ac4c8fb },
	{ 0x4b100e20, 0x00000000, 0x00000000 },
	{ 0x4b131a64, 0x0ec4c8fb, 0x0ec40384 },

	{ 0x4b100e30, 0x00000000, 0x3c1aae34 },
	{ 0x4b100e34, 0x00000000, 0x341b004d },
	{ 0x4b100e38, 0x00000000, 0xaf5b0130 },
	{ 0x4b100e3c, 0x00000000, 0x0ac6b54d },
	{ 0x4b100e40, 0x00000000, 0x00000000 },
	{ 0x4b131a78, 0x0ec6b54d, 0x0ec4038c },

	{ 0x4b100e50, 0x00000000, 0x3c1aae34 },
	{ 0x4b100e54, 0x00000000, 0x341b004e },
	{ 0x4b100e58, 0x00000000, 0xaf5b0134 },
	{ 0x4b100e5c, 0x00000000, 0x0ac4e02d },
	{ 0x4b100e60, 0x00000000, 0x00000000 },
	{ 0x4b131a8c, 0x0ec4e02d, 0x0ec40394 },

	{ 0x4b100e70, 0x00000000, 0x3c1aae34 },
	{ 0x4b100e74, 0x00000000, 0x341b004f },
	{ 0x4b100e78, 0x00000000, 0xaf5b0138 },
	{ 0x4b100e7c, 0x00000000, 0x0ac4c958 },
	{ 0x4b100e80, 0x00000000, 0x00000000 },
	{ 0x4b131a9c, 0x0ec4c958, 0x0ec4039c },

	{ 0x4b100e90, 0x00000000, 0x3c1aae34 },
	{ 0x4b100e94, 0x00000000, 0x341b0050 },
	{ 0x4b100e98, 0x00000000, 0xaf5b013c },
	{ 0x4b100e9c, 0x00000000, 0x0ac56fed },
	{ 0x4b100ea0, 0x00000000, 0x00000000 },
	{ 0x4b131ac8, 0x0ec56fed, 0x0ec403a4 },

	{ 0x4b100eb0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100eb4, 0x00000000, 0x341b0051 },
	{ 0x4b100eb8, 0x00000000, 0xaf5b0140 },
	{ 0x4b100ebc, 0x00000000, 0x0ac56fed },
	{ 0x4b100ec0, 0x00000000, 0x00000000 },
	{ 0x4b131ae4, 0x0ec56fed, 0x0ec403ac },

	{ 0x4b100ed0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100ed4, 0x00000000, 0x341b0052 },
	{ 0x4b100ed8, 0x00000000, 0xaf5b0144 },
	{ 0x4b100edc, 0x00000000, 0x0ac56fed },
	{ 0x4b100ee0, 0x00000000, 0x00000000 },
	{ 0x4b131b00, 0x0ec56fed, 0x0ec403b4 },

	{ 0x4b100ef0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100ef4, 0x00000000, 0x341b0053 },
	{ 0x4b100ef8, 0x00000000, 0xaf5b0148 },
	{ 0x4b100efc, 0x00000000, 0x0ac5401a },
	{ 0x4b100f00, 0x00000000, 0x00000000 },
	{ 0x4b131b28, 0x0ec5401a, 0x0ec403bc },

	/* Trace construction of the first HDMI receiver port. */
	{ 0x4b100f10, 0x00000000, 0x3c1aae34 },
	{ 0x4b100f14, 0x00000000, 0x341b0054 },
	{ 0x4b100f18, 0x00000000, 0xaf5b014c },
	{ 0x4b100f1c, 0x00000000, 0x0ac4e781 },
	{ 0x4b100f20, 0x00000000, 0x00000000 },
	{ 0x4b1380e0, 0x0ec4e781, 0x0ec403c4 },

	{ 0x4b100f24, 0x00000000, 0x3c1aae34 },
	{ 0x4b100f28, 0x00000000, 0x341b0055 },
	{ 0x4b100f2c, 0x00000000, 0xaf5b0150 },
	{ 0x4b100f30, 0x00000000, 0x0ac6b54d },
	{ 0x4b100f34, 0x00000000, 0x00000000 },
	{ 0x4b13810c, 0x0ec6b54d, 0x0ec403c9 },

	{ 0x4b100f38, 0x00000000, 0x3c1aae34 },
	{ 0x4b100f3c, 0x00000000, 0x341b0056 },
	{ 0x4b100f40, 0x00000000, 0xaf5b0154 },
	{ 0x4b100f44, 0x00000000, 0x0ac4d495 },
	{ 0x4b100f48, 0x00000000, 0x00000000 },
	{ 0x4b138118, 0x0ec4d495, 0x0ec403ce },

	{ 0x4b100f4c, 0x00000000, 0x3c1aae34 },
	{ 0x4b100f50, 0x00000000, 0x341b0057 },
	{ 0x4b100f54, 0x00000000, 0xaf5b0158 },
	{ 0x4b100f58, 0x00000000, 0x00400008 },
	{ 0x4b100f5c, 0x00000000, 0x00000000 },
	{ 0x4b138224, 0x0040f809, 0x0ec403d3 },

	{ 0x4b100f60, 0x00000000, 0x3c1aae34 },
	{ 0x4b100f64, 0x00000000, 0x341b0058 },
	{ 0x4b100f68, 0x00000000, 0xaf5b015c },
	{ 0x4b100f6c, 0x00000000, 0x0ac6b54d },
	{ 0x4b100f70, 0x00000000, 0x00000000 },
	{ 0x4b138234, 0x0ec6b54d, 0x0ec403d8 },

	{ 0x4b100f74, 0x00000000, 0x3c1aae34 },
	{ 0x4b100f78, 0x00000000, 0x341b0059 },
	{ 0x4b100f7c, 0x00000000, 0xaf5b0160 },
	{ 0x4b100f80, 0x00000000, 0x0ac4e7c9 },
	{ 0x4b100f84, 0x00000000, 0x00000000 },
	{ 0x4b138244, 0x0ec4e7c9, 0x0ec403dd },

	{ 0x4b100f88, 0x00000000, 0x3c1aae34 },
	{ 0x4b100f8c, 0x00000000, 0x341b005a },
	{ 0x4b100f90, 0x00000000, 0xaf5b0164 },
	{ 0x4b100f94, 0x00000000, 0x0ac6b54d },
	{ 0x4b100f98, 0x00000000, 0x00000000 },
	{ 0x4b138250, 0x0ec6b54d, 0x0ec403e2 },

	{ 0x4b100f9c, 0x00000000, 0x3c1aae34 },
	{ 0x4b100fa0, 0x00000000, 0x341b005b },
	{ 0x4b100fa4, 0x00000000, 0xaf5b0168 },
	{ 0x4b100fa8, 0x00000000, 0x0ac4e9d9 },
	{ 0x4b100fac, 0x00000000, 0x00000000 },
	{ 0x4b138260, 0x0ec4e9d9, 0x0ec403e7 },

	{ 0x4b100fb0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100fb4, 0x00000000, 0x341b005c },
	{ 0x4b100fb8, 0x00000000, 0xaf5b016c },
	{ 0x4b100fbc, 0x00000000, 0x0ac5401a },
	{ 0x4b100fc0, 0x00000000, 0x00000000 },
	{ 0x4b138178, 0x0ec5401a, 0x0ec403ec },

	/*
	 * Marker 93 enters the address validator used by the first HDMI-RX
	 * register read, then tail-calls the original function.
	 */
	{ 0x4b100fd0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100fd4, 0x00000000, 0x341b005d },
	{ 0x4b100fd8, 0x00000000, 0xaf5b0170 },
	{ 0x4b100fdc, 0x00000000, 0x0ac5fe3b },
	{ 0x4b100fe0, 0x00000000, 0x00000000 },
	{ 0x4b17fa6c, 0x0ec5fe3b, 0x0ec403f4 },

	/*
	 * Marker 94 records a successful validator return and reproduces the
	 * original branch to the rejected-address or MMIO-read paths.
	 */
	{ 0x4b100ff0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100ff4, 0x00000000, 0x341b005e },
	{ 0x4b100ff8, 0x00000000, 0xaf5b0174 },
	{ 0x4b100ffc, 0x00000000, 0x10400003 },
	{ 0x4b101000, 0x00000000, 0x00000000 },
	{ 0x4b101004, 0x00000000, 0x0ac5fea8 },
	{ 0x4b101008, 0x00000000, 0x00000000 },
	{ 0x4b10100c, 0x00000000, 0x0ac5fe9f },
	{ 0x4b101010, 0x00000000, 0x00000000 },
	{ 0x4b17fa74, 0x1440000a, 0x0ac403fc },

	/*
	 * Markers 95 and 96 surround the byte load itself. If only marker 95
	 * appears, the access to physical HDMI-RX register 0x06840093 wedged.
	 */
	{ 0x4b101020, 0x00000000, 0x3c1aae34 },
	{ 0x4b101024, 0x00000000, 0x341b005f },
	{ 0x4b101028, 0x00000000, 0xaf5b0178 },
	{ 0x4b10102c, 0x00000000, 0x90820000 },
	{ 0x4b101030, 0x00000000, 0x341b0060 },
	{ 0x4b101034, 0x00000000, 0xaf5b017c },
	{ 0x4b101038, 0x00000000, 0x03e00008 },
	{ 0x4b10103c, 0x00000000, 0x00000000 },
	{ 0x4b17fa88, 0x90820000, 0x0ec40408 },

	/*
	 * Marker 97 records the return from the first indirect helper and
	 * reproduces its original zero/nonzero branch.
	 */
	{ 0x4b101040, 0x00000000, 0x3c1aae34 },
	{ 0x4b101044, 0x00000000, 0x341b0061 },
	{ 0x4b101048, 0x00000000, 0xaf5b0180 },
	{ 0x4b10104c, 0x00000000, 0x10400003 },
	{ 0x4b101050, 0x00000000, 0x00000000 },
	{ 0x4b101054, 0x00000000, 0x0ac4e7d5 },
	{ 0x4b101058, 0x00000000, 0x00000000 },
	{ 0x4b10105c, 0x00000000, 0x0ac4e7dd },
	{ 0x4b101060, 0x00000000, 0x00000000 },
	{ 0x4b139f4c, 0x10400009, 0x0ac40410 },

	/* Marker 98 enters the second indirect target. */
	{ 0x4b101070, 0x00000000, 0x3c1aae34 },
	{ 0x4b101074, 0x00000000, 0x341b0062 },
	{ 0x4b101078, 0x00000000, 0xaf5b0184 },
	{ 0x4b10107c, 0x00000000, 0x03200008 },
	{ 0x4b101080, 0x00000000, 0x00000000 },
	{ 0x4b139f6c, 0x03200008, 0x0ac4041c },
	{ 0x4b139fc4, 0x03200008, 0x0ac4041c },

	/*
	 * Marker 99 records the second target's initial byte-read return and
	 * reproduces its original bit-test branch.
	 */
	{ 0x4b101090, 0x00000000, 0x3c1aae34 },
	{ 0x4b101094, 0x00000000, 0x341b0063 },
	{ 0x4b101098, 0x00000000, 0xaf5b0188 },
	{ 0x4b10109c, 0x00000000, 0x10400003 },
	{ 0x4b1010a0, 0x00000000, 0x00000000 },
	{ 0x4b1010a4, 0x00000000, 0x0ac4f409 },
	{ 0x4b1010a8, 0x00000000, 0x00000000 },
	{ 0x4b1010ac, 0x00000000, 0x0ac4f40c },
	{ 0x4b1010b0, 0x00000000, 0x00000000 },
	{ 0x4b13d01c, 0x10400004, 0x0ac40424 },

	/* Marker 100 enters the second target's read/modify/write fallback. */
	{ 0x4b1010c0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1010c4, 0x00000000, 0x341b0064 },
	{ 0x4b1010c8, 0x00000000, 0xaf5b018c },
	{ 0x4b1010cc, 0x00000000, 0x0ac5febf },
	{ 0x4b1010d0, 0x00000000, 0x00000000 },
	{ 0x4b13d03c, 0x0ac5febf, 0x0ac40430 },

	/* Marker 101 enters the first target's byte-write helper. */
	{ 0x4b1010e0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1010e4, 0x00000000, 0x341b0065 },
	{ 0x4b1010e8, 0x00000000, 0xaf5b0190 },
	{ 0x4b1010ec, 0x00000000, 0x0ac5fead },
	{ 0x4b1010f0, 0x00000000, 0x00000000 },
	{ 0x4b13d06c, 0x0ec5fead, 0x0ec40438 },

	/* Marker 102 proves the first target's byte write returned. */
	{ 0x4b101110, 0x00000000, 0x3c1aae34 },
	{ 0x4b101114, 0x00000000, 0x341b0066 },
	{ 0x4b101118, 0x00000000, 0xaf5b0194 },
	{ 0x4b10111c, 0x00000000, 0x0ac56f65 },
	{ 0x4b101120, 0x00000000, 0x00000000 },
	{ 0x4b13d074, 0x0ec56f65, 0x0ec40444 },

	/* Marker 103 enters the polling loop's second timer read. */
	{ 0x4b101130, 0x00000000, 0x3c1aae34 },
	{ 0x4b101134, 0x00000000, 0x341b0067 },
	{ 0x4b101138, 0x00000000, 0xaf5b0198 },
	{ 0x4b10113c, 0x00000000, 0x0ac56f65 },
	{ 0x4b101140, 0x00000000, 0x00000000 },
	{ 0x4b13d098, 0x0ec56f65, 0x0ec4044c },

	/*
	 * Marker 104 proves that timer read returned. The next two trace words
	 * capture its current tick and the initial tick used by the wait loop.
	 */
	{ 0x4b101150, 0x00000000, 0x3c1aae34 },
	{ 0x4b101154, 0x00000000, 0x341b0068 },
	{ 0x4b101158, 0x00000000, 0xaf5b019c },
	{ 0x4b10115c, 0x00000000, 0xaf4201a0 },
	{ 0x4b101160, 0x00000000, 0xaf5101a4 },
	{ 0x4b101164, 0x00000000, 0x00511823 },
	{ 0x4b101168, 0x00000000, 0x2c630033 },
	{ 0x4b10116c, 0x00000000, 0x0ac4f42a },
	{ 0x4b101170, 0x00000000, 0x00000000 },
	{ 0x4b13d0a0, 0x00511823, 0x0ac40454 },
	{ 0x4b13d0a4, 0x2c630033, 0x00000000 },

	/*
	 * Record the current callback and table position before each indirect
	 * call in platform registration. A blocking callback leaves its target
	 * as the final value in the corresponding trace pair.
	 */
	{ 0x4b101190, 0x00000000, 0x3c1aae34 },
	{ 0x4b101194, 0x00000000, 0xaf4201a8 },
	{ 0x4b101198, 0x00000000, 0xaf5001ac },
	{ 0x4b10119c, 0x00000000, 0x00400008 },
	{ 0x4b1011a0, 0x00000000, 0x00000000 },
	{ 0x4b18427c, 0x0040f809, 0x0ec40464 },

	{ 0x4b1011b0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1011b4, 0x00000000, 0xaf4201b0 },
	{ 0x4b1011b8, 0x00000000, 0xaf5001b4 },
	{ 0x4b1011bc, 0x00000000, 0x00400008 },
	{ 0x4b1011c0, 0x00000000, 0x00000000 },
	{ 0x4b1842c0, 0x0040f809, 0x0ec4046c },

	/* Marker 111 enters the marker-49 singleton allocation. */
	{ 0x4b1011d0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1011d4, 0x00000000, 0x341b006f },
	{ 0x4b1011d8, 0x00000000, 0xaf5b01b8 },
	{ 0x4b1011dc, 0x00000000, 0x0ac6b54d },
	{ 0x4b1011e0, 0x00000000, 0x00000000 },
	{ 0x4b128380, 0x0ec6b54d, 0x0ec40474 },

	/*
	 * Markers 130..132 bisect the four-registration group at 0x8b128020.
	 * It is a virtual-call loop of the form
	 *
	 *	fn = (*obj)[0xc]; fn(obj, id, s2, 0)
	 *
	 * issued for ids 0x12e, 0x130, 3, and 8 in that order. Marker 130 used
	 * to end in a spin, which halted the run before id 3 ever executed; it
	 * now falls through so the last two calls are reached.
	 *
	 * Each marker sits immediately after its call returns, so the highest
	 * one reported names the last registration that completed and the next
	 * id in the list is the one that hangs. Marker 132 doubles as the
	 * probe's completion sentinel because it lands in the final slot.
	 */
	{ 0x4b1011e8, 0x00000000, 0x3c1aae34 },
	{ 0x4b1011ec, 0x00000000, 0x341b0082 },
	{ 0x4b1011f0, 0x00000000, 0xaf5b0250 },
	{ 0x4b1011f4, 0x00000000, 0x0ac4a017 },
	{ 0x4b1011f8, 0x00000000, 0x8e020000 },
	{ 0x4b128058, 0x8e020000, 0x0ac4047a },

	/* Marker 131: registration id 3 returned. */
	{ 0x4b1002d8, 0x00000000, 0x3c1aae34 },
	{ 0x4b1002dc, 0x00000000, 0x341b0083 },
	{ 0x4b1002e0, 0x00000000, 0xaf5b0254 },
	{ 0x4b1002e4, 0x00000000, 0x0ac4a01e },
	{ 0x4b1002e8, 0x00000000, 0x8e020000 },
	{ 0x4b128074, 0x8e020000, 0x0ac400b6 },

	/* Marker 132: registration id 8 returned; the whole group completed. */
	{ 0x4b1002ec, 0x00000000, 0x3c1aae34 },
	{ 0x4b1002f0, 0x00000000, 0x341b0084 },
	{ 0x4b1002f4, 0x00000000, 0xaf5b0258 },
	{ 0x4b1002f8, 0x00000000, 0x0ac4a025 },
	{ 0x4b1002fc, 0x00000000, 0x8fb00040 },
	{ 0x4b128090, 0x8fb00040, 0x0ac400bb },
};
#endif

/*
 * Minimal CPU_COMM readiness trace rebuilt against display.bin 4380f1b3....
 *
 * Each guarded call redirects through a zero-filled code cave, stores its
 * marker through the firmware's uncached 0xae340000 alias, then either resumes
 * after a disposable log call (markers 1 and 2) or tail-calls the original
 * function. Markers 1-9 cover the chain needed for MIPS READY:
 *
 *  1. share address absent (polling path)
 *  2. share address and size accepted
 *  3. ARM CPU_READY accepted
 *  4. CPU_COMM hardware spinlock 0 acquired
 *  5. slave-side CPU_COMM initialization entered
 *  6. ThreadX application entry reached
 *  7. CPU_COMM initialization entered
 *  8. application byte-pool creation succeeded
 *  9. share-register reader called
 *
 * Markers 10-16 cover reset handoff through the ThreadX scheduler:
 *
 * 10. reset code handed off to the C runtime
 * 11-14. successive pre-application initialization calls entered
 * 15. application/thread construction entered
 * 16. ThreadX scheduler entry called
 *
 * Markers 17-26 cover every direct call in application/thread construction;
 * marker 26 is the ThreadX thread-creation call whose entry argument is the
 * application function traced by marker 6.
 * Marker 21's extended cave also records the sys:dbg_buf address and size
 * passed to memset in trace slots 88 and 89.
 *
 * Markers 27-56 cover every call inside marker 25's early-system-
 * initialization function at 0x8b15340c.
 *
 * Markers 57-61 cover all five calls in marker 38's tse_init function at
 * 0x8b110478: the global enable write, InitTFDMemory virtual call,
 * tse_init_data, and its two fatal-log paths.
 *
 * Markers 62-66 cover setCPUReady(), its spinlock acquire/unlock calls, and
 * its return to InitCommMem. Markers 67-71 cover the five calls after
 * InitCommMem returns and before setCPUAppReady(). Marker 72 enters the first
 * registration group. Markers 73-77 trace its CPU_COMM request through call-
 * table insertion, result lookup, and the insertion spinlock. Markers 78-82
 * retain later registration groups and final calls in hal_adapter_init().
 * Its repeated registration helper records a call count plus its latest
 * object/callback pointers in trace slots 90-92. Markers 84-86 cover the
 * helper's formatting, request construction, and CPU_COMM request call.
 * Marker 87 covers the final hal-adapter registration lock; marker 88 covers
 * setCPUAppReady() entry.
 */
static const struct h713_mips_patch h713_mips_trace_patches[] = {
	/* Marker 1: share address absent; skip its log call and resume. */
	{ 0x4b1002c4, 0x00000000, 0x3c1aae34 },
	{ 0x4b1002c8, 0x00000000, 0x341b0001 },
	{ 0x4b1002cc, 0x00000000, 0xaf5b0000 },
	{ 0x4b1002d0, 0x00000000, 0x0ac48e83 },
	{ 0x4b1002d4, 0x00000000, 0x00000000 },
	{ 0x4b123a04, 0x0ec54252, 0x0ac400b1 },

	/* Marker 2: share registers accepted; skip its log call and resume. */
	{ 0x4b100500, 0x00000000, 0x3c1aae34 },
	{ 0x4b100504, 0x00000000, 0x341b0002 },
	{ 0x4b100508, 0x00000000, 0xaf5b0004 },
	{ 0x4b10050c, 0x00000000, 0x0ac48ea4 },
	{ 0x4b100510, 0x00000000, 0x00000000 },
	{ 0x4b123a88, 0x0ec54252, 0x0ac40140 },

	/* Marker 3: ARM CPU_READY accepted, then preserve the original log. */
	{ 0x4b1005d0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1005d4, 0x00000000, 0x341b0003 },
	{ 0x4b1005d8, 0x00000000, 0xaf5b0008 },
	{ 0x4b1005dc, 0x00000000, 0x0ac54252 },
	{ 0x4b1005e0, 0x00000000, 0x00000000 },
	{ 0x4b123ad4, 0x0ec54252, 0x0ec40174 },

	/* Marker 4: hardware spinlock acquired, then preserve the log. */
	{ 0x4b1005f0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1005f4, 0x00000000, 0x341b0004 },
	{ 0x4b1005f8, 0x00000000, 0xaf5b000c },
	{ 0x4b1005fc, 0x00000000, 0x0ac54252 },
	{ 0x4b100600, 0x00000000, 0x00000000 },
	{ 0x4b11b1e0, 0x0ec54252, 0x0ec4017c },

	/* Marker 5: enter slave-side CPU_COMM initialization. */
	{ 0x4b100520, 0x00000000, 0x3c1aae34 },
	{ 0x4b100524, 0x00000000, 0x341b0005 },
	{ 0x4b100528, 0x00000000, 0xaf5b0010 },
	{ 0x4b10052c, 0x00000000, 0x0ac466ed },
	{ 0x4b100530, 0x00000000, 0x00000000 },
	{ 0x4b11b34c, 0x0ec466ed, 0x0ec40148 },

	/* Marker 6: ThreadX application entry reached. */
	{ 0x4b100540, 0x00000000, 0x3c1aae34 },
	{ 0x4b100544, 0x00000000, 0x341b0006 },
	{ 0x4b100548, 0x00000000, 0xaf5b0014 },
	{ 0x4b10054c, 0x00000000, 0x0ac5719d },
	{ 0x4b100550, 0x00000000, 0x00000000 },
	{ 0x4b152e84, 0x0ec5719d, 0x0ec40150 },

	/* Marker 7: enter CPU_COMM initialization. */
	{ 0x4b100560, 0x00000000, 0x3c1aae34 },
	{ 0x4b100564, 0x00000000, 0x341b0007 },
	{ 0x4b100568, 0x00000000, 0xaf5b0018 },
	{ 0x4b10056c, 0x00000000, 0x0ac490da },
	{ 0x4b100570, 0x00000000, 0x00000000 },
	{ 0x4b152e8c, 0x0ec490da, 0x0ec40158 },

	/* Marker 8: CPU_COMM byte-pool creation succeeded. */
	{ 0x4b100590, 0x00000000, 0x3c1aae34 },
	{ 0x4b100594, 0x00000000, 0x341b0008 },
	{ 0x4b100598, 0x00000000, 0xaf5b001c },
	{ 0x4b10059c, 0x00000000, 0x0ac54252 },
	{ 0x4b1005a0, 0x00000000, 0x00000000 },
	{ 0x4b12443c, 0x0ec54252, 0x0ec40164 },

	/* Marker 9: enter the share-register reader. */
	{ 0x4b1005b0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1005b4, 0x00000000, 0x341b0009 },
	{ 0x4b1005b8, 0x00000000, 0xaf5b0020 },
	{ 0x4b1005bc, 0x00000000, 0x0ac48e55 },
	{ 0x4b1005c0, 0x00000000, 0x00000000 },
	{ 0x4b124448, 0x0ec48e55, 0x0ec4016c },

	/* Marker 10: reset code hands off to the C runtime at 0x8b101f04. */
	{ 0x4b100610, 0x00000000, 0x3c1aae34 },
	{ 0x4b100614, 0x00000000, 0x341b000a },
	{ 0x4b100618, 0x00000000, 0xaf5b0024 },
	{ 0x4b10061c, 0x00000000, 0x0ac407c1 },
	{ 0x4b100620, 0x00000000, 0x00000000 },
	{ 0x4b1b1220, 0x0100f809, 0x0ec40184 },

	/* Markers 11-16: every C-runtime call through scheduler entry. */
	{ 0x4b100630, 0x00000000, 0x3c1aae34 },
	{ 0x4b100634, 0x00000000, 0x341b000b },
	{ 0x4b100638, 0x00000000, 0xaf5b0028 },
	{ 0x4b10063c, 0x00000000, 0x0ac419a4 },
	{ 0x4b100640, 0x00000000, 0x00000000 },
	{ 0x4b101f1c, 0x0ec419a4, 0x0ec4018c },

	{ 0x4b100650, 0x00000000, 0x3c1aae34 },
	{ 0x4b100654, 0x00000000, 0x341b000c },
	{ 0x4b100658, 0x00000000, 0xaf5b002c },
	{ 0x4b10065c, 0x00000000, 0x0ac40b86 },
	{ 0x4b100660, 0x00000000, 0x00000000 },
	{ 0x4b101f24, 0x0ec40b86, 0x0ec40194 },

	{ 0x4b100670, 0x00000000, 0x3c1aae34 },
	{ 0x4b100674, 0x00000000, 0x341b000d },
	{ 0x4b100678, 0x00000000, 0xaf5b0030 },
	{ 0x4b10067c, 0x00000000, 0x0ac5203a },
	{ 0x4b100680, 0x00000000, 0x00000000 },
	{ 0x4b101f2c, 0x0ec5203a, 0x0ec4019c },

	{ 0x4b100690, 0x00000000, 0x3c1aae34 },
	{ 0x4b100694, 0x00000000, 0x341b000e },
	{ 0x4b100698, 0x00000000, 0xaf5b0034 },
	{ 0x4b10069c, 0x00000000, 0x0ac612c9 },
	{ 0x4b1006a0, 0x00000000, 0x00000000 },
	{ 0x4b101f34, 0x0ec612c9, 0x0ec401a4 },

	{ 0x4b1006b0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1006b4, 0x00000000, 0x341b000f },
	{ 0x4b1006b8, 0x00000000, 0xaf5b0038 },
	{ 0x4b1006bc, 0x00000000, 0x0ac54d6f },
	{ 0x4b1006c0, 0x00000000, 0x00000000 },
	{ 0x4b101f3c, 0x0ec54d6f, 0x0ec401ac },

	{ 0x4b1006d0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1006d4, 0x00000000, 0x341b0010 },
	{ 0x4b1006d8, 0x00000000, 0xaf5b003c },
	{ 0x4b1006dc, 0x00000000, 0x0ac415e4 },
	{ 0x4b1006e0, 0x00000000, 0x00000000 },
	{ 0x4b101f4c, 0x0ec415e4, 0x0ec401b4 },

	/* Markers 17-26: each application/thread construction call. */
	{ 0x4b1006f0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1006f4, 0x00000000, 0x341b0011 },
	{ 0x4b1006f8, 0x00000000, 0xaf5b0040 },
	{ 0x4b1006fc, 0x00000000, 0x0ac674f1 },
	{ 0x4b100700, 0x00000000, 0x00000000 },
	{ 0x4b1535c4, 0x0ec674f1, 0x0ec401bc },

	{ 0x4b100710, 0x00000000, 0x3c1aae34 },
	{ 0x4b100714, 0x00000000, 0x341b0012 },
	{ 0x4b100718, 0x00000000, 0xaf5b0044 },
	{ 0x4b10071c, 0x00000000, 0x0ac43a27 },
	{ 0x4b100720, 0x00000000, 0x00000000 },
	{ 0x4b1535d0, 0x0ec43a27, 0x0ec401c4 },

	{ 0x4b100730, 0x00000000, 0x3c1aae34 },
	{ 0x4b100734, 0x00000000, 0x341b0013 },
	{ 0x4b100738, 0x00000000, 0xaf5b0048 },
	{ 0x4b10073c, 0x00000000, 0x0ac4354e },
	{ 0x4b100740, 0x00000000, 0x00000000 },
	{ 0x4b1535d8, 0x0ec4354e, 0x0ec401cc },

	{ 0x4b100750, 0x00000000, 0x3c1aae34 },
	{ 0x4b100754, 0x00000000, 0x341b0014 },
	{ 0x4b100758, 0x00000000, 0xaf5b004c },
	{ 0x4b10075c, 0x00000000, 0x0ac4355c },
	{ 0x4b100760, 0x00000000, 0x00000000 },
	{ 0x4b1535e0, 0x0ec4355c, 0x0ec401d4 },

	{ 0x4b100770, 0x00000000, 0x3c1aae34 },
	{ 0x4b100774, 0x00000000, 0x341b0015 },
	{ 0x4b100778, 0x00000000, 0xaf5b0050 },
	{ 0x4b10077c, 0x00000000, 0xaf440160 },
	{ 0x4b100780, 0x00000000, 0xaf460164 },
	{ 0x4b100784, 0x00000000, 0x0ac568d0 },
	{ 0x4b100788, 0x00000000, 0x00000000 },
	{ 0x4b1535f0, 0x0ec568d0, 0x0ec401dc },

	{ 0x4b100790, 0x00000000, 0x3c1aae34 },
	{ 0x4b100794, 0x00000000, 0x341b0016 },
	{ 0x4b100798, 0x00000000, 0xaf5b0054 },
	{ 0x4b10079c, 0x00000000, 0x0ac55058 },
	{ 0x4b1007a0, 0x00000000, 0x00000000 },
	{ 0x4b1535f8, 0x0ec55058, 0x0ec401e4 },

	{ 0x4b1007b0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1007b4, 0x00000000, 0x341b0017 },
	{ 0x4b1007b8, 0x00000000, 0xaf5b0058 },
	{ 0x4b1007bc, 0x00000000, 0x0ac54643 },
	{ 0x4b1007c0, 0x00000000, 0x00000000 },
	{ 0x4b153600, 0x0ec54643, 0x0ec401ec },

	{ 0x4b1007d0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1007d4, 0x00000000, 0x341b0018 },
	{ 0x4b1007d8, 0x00000000, 0xaf5b005c },
	{ 0x4b1007dc, 0x00000000, 0x0ac60d31 },
	{ 0x4b1007e0, 0x00000000, 0x00000000 },
	{ 0x4b153608, 0x0ec60d31, 0x0ec401f4 },

	{ 0x4b1007f0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1007f4, 0x00000000, 0x341b0019 },
	{ 0x4b1007f8, 0x00000000, 0xaf5b0060 },
	{ 0x4b1007fc, 0x00000000, 0x0ac54d03 },
	{ 0x4b100800, 0x00000000, 0x00000000 },
	{ 0x4b153610, 0x0ec54d03, 0x0ec401fc },

	{ 0x4b100810, 0x00000000, 0x3c1aae34 },
	{ 0x4b100814, 0x00000000, 0x341b001a },
	{ 0x4b100818, 0x00000000, 0xaf5b0064 },
	{ 0x4b10081c, 0x00000000, 0x0ac57225 },
	{ 0x4b100820, 0x00000000, 0x00000000 },
	{ 0x4b153634, 0x0ec57225, 0x0ec40204 },

	/* Markers 27-56: every call inside 0x8b15340c. */
	{ 0x4b100830, 0x00000000, 0x3c1aae34 },
	{ 0x4b100834, 0x00000000, 0x341b001b },
	{ 0x4b100838, 0x00000000, 0xaf5b0068 },
	{ 0x4b10083c, 0x00000000, 0x0ac6017c },
	{ 0x4b100840, 0x00000000, 0x00000000 },
	{ 0x4b153444, 0x0ec6017c, 0x0ec4020c },

	{ 0x4b100850, 0x00000000, 0x3c1aae34 },
	{ 0x4b100854, 0x00000000, 0x341b001c },
	{ 0x4b100858, 0x00000000, 0xaf5b006c },
	{ 0x4b10085c, 0x00000000, 0x0ac4354b },
	{ 0x4b100860, 0x00000000, 0x00000000 },
	{ 0x4b153454, 0x0ec4354b, 0x0ec40214 },

	{ 0x4b100870, 0x00000000, 0x3c1aae34 },
	{ 0x4b100874, 0x00000000, 0x341b001d },
	{ 0x4b100878, 0x00000000, 0xaf5b0070 },
	{ 0x4b10087c, 0x00000000, 0x00400008 },
	{ 0x4b100880, 0x00000000, 0x00000000 },
	{ 0x4b153470, 0x0040f809, 0x0ec4021c },

	{ 0x4b100890, 0x00000000, 0x3c1aae34 },
	{ 0x4b100894, 0x00000000, 0x341b001e },
	{ 0x4b100898, 0x00000000, 0xaf5b0074 },
	{ 0x4b10089c, 0x00000000, 0x0ac4354b },
	{ 0x4b1008a0, 0x00000000, 0x00000000 },
	{ 0x4b153478, 0x0ec4354b, 0x0ec40224 },

	{ 0x4b1008b0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1008b4, 0x00000000, 0x341b001f },
	{ 0x4b1008b8, 0x00000000, 0xaf5b0078 },
	{ 0x4b1008bc, 0x00000000, 0x00400008 },
	{ 0x4b1008c0, 0x00000000, 0x00000000 },
	{ 0x4b153494, 0x0040f809, 0x0ec4022c },

	{ 0x4b1008d0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1008d4, 0x00000000, 0x341b0020 },
	{ 0x4b1008d8, 0x00000000, 0xaf5b007c },
	{ 0x4b1008dc, 0x00000000, 0x0ac6018e },
	{ 0x4b1008e0, 0x00000000, 0x00000000 },
	{ 0x4b1534a4, 0x0ec6018e, 0x0ec40234 },

	{ 0x4b1008f0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1008f4, 0x00000000, 0x341b0021 },
	{ 0x4b1008f8, 0x00000000, 0xaf5b0080 },
	{ 0x4b1008fc, 0x00000000, 0x0ac6018e },
	{ 0x4b100900, 0x00000000, 0x00000000 },
	{ 0x4b1534b4, 0x0ec6018e, 0x0ec4023c },

	{ 0x4b100910, 0x00000000, 0x3c1aae34 },
	{ 0x4b100914, 0x00000000, 0x341b0022 },
	{ 0x4b100918, 0x00000000, 0xaf5b0084 },
	{ 0x4b10091c, 0x00000000, 0x0ac6282e },
	{ 0x4b100920, 0x00000000, 0x00000000 },
	{ 0x4b1534c0, 0x0ec6282e, 0x0ec40244 },

	{ 0x4b100930, 0x00000000, 0x3c1aae34 },
	{ 0x4b100934, 0x00000000, 0x341b0023 },
	{ 0x4b100938, 0x00000000, 0xaf5b0088 },
	{ 0x4b10093c, 0x00000000, 0x0ac6257d },
	{ 0x4b100940, 0x00000000, 0x00000000 },
	{ 0x4b1534c8, 0x0ec6257d, 0x0ec4024c },

	{ 0x4b100950, 0x00000000, 0x3c1aae34 },
	{ 0x4b100954, 0x00000000, 0x341b0024 },
	{ 0x4b100958, 0x00000000, 0xaf5b008c },
	{ 0x4b10095c, 0x00000000, 0x0ac4354b },
	{ 0x4b100960, 0x00000000, 0x00000000 },
	{ 0x4b1534d0, 0x0ec4354b, 0x0ec40254 },

	{ 0x4b100970, 0x00000000, 0x3c1aae34 },
	{ 0x4b100974, 0x00000000, 0x341b0025 },
	{ 0x4b100978, 0x00000000, 0xaf5b0090 },
	{ 0x4b10097c, 0x00000000, 0x00400008 },
	{ 0x4b100980, 0x00000000, 0x00000000 },
	{ 0x4b1534ec, 0x0040f809, 0x0ec4025c },

	{ 0x4b100990, 0x00000000, 0x3c1aae34 },
	{ 0x4b100994, 0x00000000, 0x341b0026 },
	{ 0x4b100998, 0x00000000, 0xaf5b0094 },
	{ 0x4b10099c, 0x00000000, 0x0ac4411e },
	{ 0x4b1009a0, 0x00000000, 0x00000000 },
	{ 0x4b1534fc, 0x0ec4411e, 0x0ec40264 },

	{ 0x4b1009b0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1009b4, 0x00000000, 0x341b0027 },
	{ 0x4b1009b8, 0x00000000, 0xaf5b0098 },
	{ 0x4b1009bc, 0x00000000, 0x0ac54ccf },
	{ 0x4b1009c0, 0x00000000, 0x00000000 },
	{ 0x4b153504, 0x0ec54ccf, 0x0ec4026c },

	{ 0x4b1009d0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1009d4, 0x00000000, 0x341b0028 },
	{ 0x4b1009d8, 0x00000000, 0xaf5b009c },
	{ 0x4b1009dc, 0x00000000, 0x0ac627a9 },
	{ 0x4b1009e0, 0x00000000, 0x00000000 },
	{ 0x4b15350c, 0x0ec627a9, 0x0ec40274 },

	{ 0x4b1009f0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1009f4, 0x00000000, 0x341b0029 },
	{ 0x4b1009f8, 0x00000000, 0xaf5b00a0 },
	{ 0x4b1009fc, 0x00000000, 0x00400008 },
	{ 0x4b100a00, 0x00000000, 0x00000000 },
	{ 0x4b153520, 0x0040f809, 0x0ec4027c },

	{ 0x4b100a10, 0x00000000, 0x3c1aae34 },
	{ 0x4b100a14, 0x00000000, 0x341b002a },
	{ 0x4b100a18, 0x00000000, 0xaf5b00a4 },
	{ 0x4b100a1c, 0x00000000, 0x0ac69eef },
	{ 0x4b100a20, 0x00000000, 0x00000000 },
	{ 0x4b153528, 0x0ec69eef, 0x0ec40284 },

	{ 0x4b100a30, 0x00000000, 0x3c1aae34 },
	{ 0x4b100a34, 0x00000000, 0x341b002b },
	{ 0x4b100a38, 0x00000000, 0xaf5b00a8 },
	{ 0x4b100a3c, 0x00000000, 0x0ac561be },
	{ 0x4b100a40, 0x00000000, 0x00000000 },
	{ 0x4b153530, 0x0ec561be, 0x0ec4028c },

	{ 0x4b100a50, 0x00000000, 0x3c1aae34 },
	{ 0x4b100a54, 0x00000000, 0x341b002c },
	{ 0x4b100a58, 0x00000000, 0xaf5b00ac },
	{ 0x4b100a5c, 0x00000000, 0x0ac4a11e },
	{ 0x4b100a60, 0x00000000, 0x00000000 },
	{ 0x4b153538, 0x0ec4a11e, 0x0ec40294 },

	{ 0x4b100a70, 0x00000000, 0x3c1aae34 },
	{ 0x4b100a74, 0x00000000, 0x341b002d },
	{ 0x4b100a78, 0x00000000, 0xaf5b00b0 },
	{ 0x4b100a7c, 0x00000000, 0x0ac679c4 },
	{ 0x4b100a80, 0x00000000, 0x00000000 },
	{ 0x4b153540, 0x0ec679c4, 0x0ec4029c },

	{ 0x4b100a90, 0x00000000, 0x3c1aae34 },
	{ 0x4b100a94, 0x00000000, 0x341b002e },
	{ 0x4b100a98, 0x00000000, 0xaf5b00b4 },
	{ 0x4b100a9c, 0x00000000, 0x0ac57387 },
	{ 0x4b100aa0, 0x00000000, 0x00000000 },
	{ 0x4b153548, 0x0ec57387, 0x0ec402a4 },

	{ 0x4b100ab0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100ab4, 0x00000000, 0x341b002f },
	{ 0x4b100ab8, 0x00000000, 0xaf5b00b8 },
	{ 0x4b100abc, 0x00000000, 0x0ac5ece3 },
	{ 0x4b100ac0, 0x00000000, 0x00000000 },
	{ 0x4b153550, 0x0ec5ece3, 0x0ec402ac },

	{ 0x4b100ad0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100ad4, 0x00000000, 0x341b0030 },
	{ 0x4b100ad8, 0x00000000, 0xaf5b00bc },
	{ 0x4b100adc, 0x00000000, 0x0ac610af },
	{ 0x4b100ae0, 0x00000000, 0x00000000 },
	{ 0x4b153558, 0x0ec610af, 0x0ec402b4 },

	{ 0x4b100af0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100af4, 0x00000000, 0x341b0031 },
	{ 0x4b100af8, 0x00000000, 0xaf5b00c0 },
	{ 0x4b100afc, 0x00000000, 0x0ac6123b },
	{ 0x4b100b00, 0x00000000, 0x00000000 },
	{ 0x4b153560, 0x0ec6123b, 0x0ec402bc },

	{ 0x4b100b10, 0x00000000, 0x3c1aae34 },
	{ 0x4b100b14, 0x00000000, 0x341b0032 },
	{ 0x4b100b18, 0x00000000, 0xaf5b00c4 },
	{ 0x4b100b1c, 0x00000000, 0x0ac5f311 },
	{ 0x4b100b20, 0x00000000, 0x00000000 },
	{ 0x4b153568, 0x0ec5f311, 0x0ec402c4 },

	{ 0x4b100b30, 0x00000000, 0x3c1aae34 },
	{ 0x4b100b34, 0x00000000, 0x341b0033 },
	{ 0x4b100b38, 0x00000000, 0xaf5b00c8 },
	{ 0x4b100b3c, 0x00000000, 0x0ac678c6 },
	{ 0x4b100b40, 0x00000000, 0x00000000 },
	{ 0x4b153570, 0x0ec678c6, 0x0ec402cc },

	{ 0x4b100b50, 0x00000000, 0x3c1aae34 },
	{ 0x4b100b54, 0x00000000, 0x341b0034 },
	{ 0x4b100b58, 0x00000000, 0xaf5b00cc },
	{ 0x4b100b5c, 0x00000000, 0x0ac6b160 },
	{ 0x4b100b60, 0x00000000, 0x00000000 },
	{ 0x4b153578, 0x0ec6b160, 0x0ec402d4 },

	{ 0x4b100b70, 0x00000000, 0x3c1aae34 },
	{ 0x4b100b74, 0x00000000, 0x341b0035 },
	{ 0x4b100b78, 0x00000000, 0xaf5b00d0 },
	{ 0x4b100b7c, 0x00000000, 0x0ac54fe5 },
	{ 0x4b100b80, 0x00000000, 0x00000000 },
	{ 0x4b153580, 0x0ec54fe5, 0x0ec402dc },

	{ 0x4b100b90, 0x00000000, 0x3c1aae34 },
	{ 0x4b100b94, 0x00000000, 0x341b0036 },
	{ 0x4b100b98, 0x00000000, 0xaf5b00d4 },
	{ 0x4b100b9c, 0x00000000, 0x0ac41bdf },
	{ 0x4b100ba0, 0x00000000, 0x00000000 },
	{ 0x4b153590, 0x0ec41bdf, 0x0ec402e4 },

	{ 0x4b100bb0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100bb4, 0x00000000, 0x341b0037 },
	{ 0x4b100bb8, 0x00000000, 0xaf5b00d8 },
	{ 0x4b100bbc, 0x00000000, 0x0ac41adf },
	{ 0x4b100bc0, 0x00000000, 0x00000000 },
	{ 0x4b153598, 0x0ec41adf, 0x0ec402ec },

	{ 0x4b100bd0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100bd4, 0x00000000, 0x341b0038 },
	{ 0x4b100bd8, 0x00000000, 0xaf5b00dc },
	{ 0x4b100bdc, 0x00000000, 0x0ac423ed },
	{ 0x4b100be0, 0x00000000, 0x00000000 },
	{ 0x4b1535a0, 0x0ec423ed, 0x0ec402f4 },

	/* Markers 57-61: every call inside tse_init at 0x8b110478. */
	{ 0x4b100bf0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100bf4, 0x00000000, 0x341b0039 },
	{ 0x4b100bf8, 0x00000000, 0xaf5b00e0 },
	{ 0x4b100bfc, 0x00000000, 0x0ac627c9 },
	{ 0x4b100c00, 0x00000000, 0x00000000 },
	{ 0x4b1104a0, 0x0ec627c9, 0x0ec402fc },

	{ 0x4b100c10, 0x00000000, 0x3c1aae34 },
	{ 0x4b100c14, 0x00000000, 0x341b003a },
	{ 0x4b100c18, 0x00000000, 0xaf5b00e4 },
	{ 0x4b100c1c, 0x00000000, 0x00400008 },
	{ 0x4b100c20, 0x00000000, 0x00000000 },
	{ 0x4b1104b8, 0x0040f809, 0x0ec40304 },

	{ 0x4b100c30, 0x00000000, 0x3c1aae34 },
	{ 0x4b100c34, 0x00000000, 0x341b003b },
	{ 0x4b100c38, 0x00000000, 0xaf5b00e8 },
	{ 0x4b100c3c, 0x00000000, 0x0ac43aa4 },
	{ 0x4b100c40, 0x00000000, 0x00000000 },
	{ 0x4b1104c8, 0x0ec43aa4, 0x0ec4030c },

	{ 0x4b100c50, 0x00000000, 0x3c1aae34 },
	{ 0x4b100c54, 0x00000000, 0x341b003c },
	{ 0x4b100c58, 0x00000000, 0xaf5b00ec },
	{ 0x4b100c5c, 0x00000000, 0x0ac54252 },
	{ 0x4b100c60, 0x00000000, 0x00000000 },
	{ 0x4b110518, 0x0ec54252, 0x0ec40314 },

	{ 0x4b100c70, 0x00000000, 0x3c1aae34 },
	{ 0x4b100c74, 0x00000000, 0x341b003d },
	{ 0x4b100c78, 0x00000000, 0xaf5b00f0 },
	{ 0x4b100c7c, 0x00000000, 0x0ac54252 },
	{ 0x4b100c80, 0x00000000, 0x00000000 },
	{ 0x4b110568, 0x0ec54252, 0x0ec4031c },

	/* Marker 62: enter setCPUReady(getCurCPUID()). */
	{ 0x4b100c90, 0x00000000, 0x3c1aae34 },
	{ 0x4b100c94, 0x00000000, 0x341b003e },
	{ 0x4b100c98, 0x00000000, 0xaf5b00f4 },
	{ 0x4b100c9c, 0x00000000, 0x0ac468f7 },
	{ 0x4b100ca0, 0x00000000, 0x00000000 },
	{ 0x4b11b364, 0x0ec468f7, 0x0ec40324 },

	/* Marker 63: setCPUReady enters comm_SpinLock(3). */
	{ 0x4b100cb0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100cb4, 0x00000000, 0x341b003f },
	{ 0x4b100cb8, 0x00000000, 0xaf5b00f8 },
	{ 0x4b100cbc, 0x00000000, 0x0ac496f1 },
	{ 0x4b100cc0, 0x00000000, 0x00000000 },
	{ 0x4b11a56c, 0x0ec496f1, 0x0ec4032c },

	/* Marker 64: READY stores completed; enter comm_SpinUnlock(3). */
	{ 0x4b100cd0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100cd4, 0x00000000, 0x341b0040 },
	{ 0x4b100cd8, 0x00000000, 0xaf5b00fc },
	{ 0x4b100cdc, 0x00000000, 0x0ac496fa },
	{ 0x4b100ce0, 0x00000000, 0x00000000 },
	{ 0x4b11a5a4, 0x0ec496fa, 0x0ec40334 },

	/* Marker 65: unlock returned; enter setCPUReady's final log call. */
	{ 0x4b100cf0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100cf4, 0x00000000, 0x341b0041 },
	{ 0x4b100cf8, 0x00000000, 0xaf5b0100 },
	{ 0x4b100cfc, 0x00000000, 0x0ac54252 },
	{ 0x4b100d00, 0x00000000, 0x00000000 },
	{ 0x4b11a5cc, 0x0ec54252, 0x0ec4033c },

	/* Marker 66: setCPUReady returned to InitCommMem. */
	{ 0x4b100d10, 0x00000000, 0x3c1aae34 },
	{ 0x4b100d14, 0x00000000, 0x341b0042 },
	{ 0x4b100d18, 0x00000000, 0xaf5b0104 },
	{ 0x4b100d1c, 0x00000000, 0x0ac49318 },
	{ 0x4b100d20, 0x00000000, 0x00000000 },
	{ 0x4b11b36c, 0x0ec49318, 0x0ec40344 },

	/* Markers 67-71: calls after InitCommMem and before app-ready. */
	{ 0x4b100d30, 0x00000000, 0x3c1aae34 },
	{ 0x4b100d34, 0x00000000, 0x341b0043 },
	{ 0x4b100d38, 0x00000000, 0xaf5b0108 },
	{ 0x4b100d3c, 0x00000000, 0x0ac5719d },
	{ 0x4b100d40, 0x00000000, 0x00000000 },
	{ 0x4b152ec8, 0x0ec5719d, 0x0ec4034c },

	{ 0x4b100d50, 0x00000000, 0x3c1aae34 },
	{ 0x4b100d54, 0x00000000, 0x341b0044 },
	{ 0x4b100d58, 0x00000000, 0xaf5b010c },
	{ 0x4b100d5c, 0x00000000, 0x0ac42b6e },
	{ 0x4b100d60, 0x00000000, 0x00000000 },
	{ 0x4b152ed0, 0x0ec42b6e, 0x0ec40354 },

	{ 0x4b100d70, 0x00000000, 0x3c1aae34 },
	{ 0x4b100d74, 0x00000000, 0x341b0045 },
	{ 0x4b100d78, 0x00000000, 0xaf5b0110 },
	{ 0x4b100d7c, 0x00000000, 0x0ac5719d },
	{ 0x4b100d80, 0x00000000, 0x00000000 },
	{ 0x4b152ed8, 0x0ec5719d, 0x0ec4035c },

	{ 0x4b100d90, 0x00000000, 0x3c1aae34 },
	{ 0x4b100d94, 0x00000000, 0x341b0046 },
	{ 0x4b100d98, 0x00000000, 0xaf5b0114 },
	{ 0x4b100d9c, 0x00000000, 0x0ac52161 },
	{ 0x4b100da0, 0x00000000, 0x00000000 },
	{ 0x4b152ee4, 0x0ec52161, 0x0ec40364 },

	{ 0x4b100db0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100db4, 0x00000000, 0x341b0047 },
	{ 0x4b100db8, 0x00000000, 0xaf5b0118 },
	{ 0x4b100dbc, 0x00000000, 0x0ac52132 },
	{ 0x4b100dc0, 0x00000000, 0x00000000 },
	{ 0x4b152eec, 0x0ec52132, 0x0ec4036c },

	/* Marker 72: first registration group in hal_adapter_init(). */
	{ 0x4b100dd0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100dd4, 0x00000000, 0x341b0048 },
	{ 0x4b100dd8, 0x00000000, 0xaf5b011c },
	{ 0x4b100ddc, 0x00000000, 0x0ac49246 },
	{ 0x4b100de0, 0x00000000, 0x00000000 },
	{ 0x4b10ade4, 0x0ec49246, 0x0ec40374 },

	/* Marker 73: registration request enters the shared call-table insert. */
	{ 0x4b100df0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100df4, 0x00000000, 0x341b0049 },
	{ 0x4b100df8, 0x00000000, 0xaf5b0120 },
	{ 0x4b100dfc, 0x00000000, 0x0ac4717c },
	{ 0x4b100e00, 0x00000000, 0x00000000 },
	{ 0x4b1225dc, 0x0ec4717c, 0x0ec4037c },

	/* Marker 74: call-table insert returned; resolve its result object. */
	{ 0x4b100e10, 0x00000000, 0x3c1aae34 },
	{ 0x4b100e14, 0x00000000, 0x341b004a },
	{ 0x4b100e18, 0x00000000, 0xaf5b0124 },
	{ 0x4b100e1c, 0x00000000, 0x0ac47382 },
	{ 0x4b100e20, 0x00000000, 0x00000000 },
	{ 0x4b122614, 0x0ec47382, 0x0ec40384 },

	/* Marker 75: result absent; enter the alternate cleanup/removal call. */
	{ 0x4b100e30, 0x00000000, 0x3c1aae34 },
	{ 0x4b100e34, 0x00000000, 0x341b004b },
	{ 0x4b100e38, 0x00000000, 0xaf5b0128 },
	{ 0x4b100e3c, 0x00000000, 0x0ac4738d },
	{ 0x4b100e40, 0x00000000, 0x00000000 },
	{ 0x4b122658, 0x0ec4738d, 0x0ec4038c },

	/* Marker 76: call-table insert enters comm_SpinLock(2). */
	{ 0x4b100e50, 0x00000000, 0x3c1aae34 },
	{ 0x4b100e54, 0x00000000, 0x341b004c },
	{ 0x4b100e58, 0x00000000, 0xaf5b012c },
	{ 0x4b100e5c, 0x00000000, 0x0ac496f1 },
	{ 0x4b100e60, 0x00000000, 0x00000000 },
	{ 0x4b11c7a8, 0x0ec496f1, 0x0ec40394 },

	/* Marker 77: entry copied and counted; enter comm_SpinUnlock(2). */
	{ 0x4b100e70, 0x00000000, 0x3c1aae34 },
	{ 0x4b100e74, 0x00000000, 0x341b004d },
	{ 0x4b100e78, 0x00000000, 0xaf5b0130 },
	{ 0x4b100e7c, 0x00000000, 0x0ac496fa },
	{ 0x4b100e80, 0x00000000, 0x00000000 },
	{ 0x4b11c838, 0x0ec496fa, 0x0ec4039c },

	/* Markers 78-80: later registration groups in hal_adapter_init(). */
	{ 0x4b100e90, 0x00000000, 0x3c1aae34 },
	{ 0x4b100e94, 0x00000000, 0x341b004e },
	{ 0x4b100e98, 0x00000000, 0xaf5b0134 },
	{ 0x4b100e9c, 0x00000000, 0x0ac49246 },
	{ 0x4b100ea0, 0x00000000, 0x00000000 },
	{ 0x4b10aeb4, 0x0ec49246, 0x0ec403a4 },

	{ 0x4b100eb0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100eb4, 0x00000000, 0x341b004f },
	{ 0x4b100eb8, 0x00000000, 0xaf5b0138 },
	{ 0x4b100ebc, 0x00000000, 0x0ac49246 },
	{ 0x4b100ec0, 0x00000000, 0x00000000 },
	{ 0x4b10aee0, 0x0ec49246, 0x0ec403ac },

	{ 0x4b100ed0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100ed4, 0x00000000, 0x341b0050 },
	{ 0x4b100ed8, 0x00000000, 0xaf5b013c },
	{ 0x4b100edc, 0x00000000, 0x0ac49246 },
	{ 0x4b100ee0, 0x00000000, 0x00000000 },
	{ 0x4b10af0c, 0x0ec49246, 0x0ec403b4 },

	/* Markers 81-82: final registration and hal-adapter return log. */
	{ 0x4b100ef0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100ef4, 0x00000000, 0x341b0051 },
	{ 0x4b100ef8, 0x00000000, 0xaf5b0140 },
	{ 0x4b100efc, 0x00000000, 0x0ac48de2 },
	{ 0x4b100f00, 0x00000000, 0x00000000 },
	{ 0x4b10af1c, 0x0ec48de2, 0x0ec403bc },

	{ 0x4b100f10, 0x00000000, 0x3c1aae34 },
	{ 0x4b100f14, 0x00000000, 0x341b0052 },
	{ 0x4b100f18, 0x00000000, 0xaf5b0144 },
	{ 0x4b100f1c, 0x00000000, 0x0ac54252 },
	{ 0x4b100f20, 0x00000000, 0x00000000 },
	{ 0x4b10af5c, 0x0ec54252, 0x0ec403c4 },

	/* Repeated helper entry: count calls and retain its latest arguments. */
	{ 0x4b100f30, 0x00000000, 0x3c1aae34 },
	{ 0x4b100f34, 0x00000000, 0x8f5b0168 },
	{ 0x4b100f38, 0x00000000, 0x277b0001 },
	{ 0x4b100f3c, 0x00000000, 0xaf5b0168 },
	{ 0x4b100f40, 0x00000000, 0xaf44016c },
	{ 0x4b100f44, 0x00000000, 0xaf450170 },
	{ 0x4b100f48, 0x00000000, 0x0ac489ed },
	{ 0x4b100f4c, 0x00000000, 0x00000000 },
	{ 0x4b124984, 0x0ec489ed, 0x0ec403cc },

	/* Markers 84-86: format, construct, and submit a registration. */
	{ 0x4b100f50, 0x00000000, 0x3c1aae34 },
	{ 0x4b100f54, 0x00000000, 0x341b0054 },
	{ 0x4b100f58, 0x00000000, 0xaf5b014c },
	{ 0x4b100f5c, 0x00000000, 0x0ac56cf3 },
	{ 0x4b100f60, 0x00000000, 0x00000000 },
	{ 0x4b1249a4, 0x0ec56cf3, 0x0ec403d4 },

	{ 0x4b100f70, 0x00000000, 0x3c1aae34 },
	{ 0x4b100f74, 0x00000000, 0x341b0055 },
	{ 0x4b100f78, 0x00000000, 0xaf5b0150 },
	{ 0x4b100f7c, 0x00000000, 0x0ac491c1 },
	{ 0x4b100f80, 0x00000000, 0x00000000 },
	{ 0x4b1249b4, 0x0ec491c1, 0x0ec403dc },

	{ 0x4b100f90, 0x00000000, 0x3c1aae34 },
	{ 0x4b100f94, 0x00000000, 0x341b0056 },
	{ 0x4b100f98, 0x00000000, 0xaf5b0154 },
	{ 0x4b100f9c, 0x00000000, 0x0ac48955 },
	{ 0x4b100fa0, 0x00000000, 0x00000000 },
	{ 0x4b12486c, 0x0ec48955, 0x0ec403e4 },

	/* Marker 87: final hal-adapter registration enters spinlock 1. */
	{ 0x4b100fb0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100fb4, 0x00000000, 0x341b0057 },
	{ 0x4b100fb8, 0x00000000, 0xaf5b0158 },
	{ 0x4b100fbc, 0x00000000, 0x0ac496f1 },
	{ 0x4b100fc0, 0x00000000, 0x00000000 },
	{ 0x4b123828, 0x0ec496f1, 0x0ec403ec },

	/* Marker 88: enter the later setCPUAppReady() wrapper. */
	{ 0x4b100fd0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100fd4, 0x00000000, 0x341b0058 },
	{ 0x4b100fd8, 0x00000000, 0xaf5b015c },
	{ 0x4b100fdc, 0x00000000, 0x0ac4912c },
	{ 0x4b100fe0, 0x00000000, 0x00000000 },
	{ 0x4b152ef4, 0x0ec4912c, 0x0ec403f4 },
};

static void h713_mips_print_digest(const u8 *digest)
{
	int i;

	for (i = 0; i < SHA256_SUM_LEN; i++)
		printf("%02x", digest[i]);
}

static void h713_mips_stop(void)
{
	writel(H713_MIPS_RESET_ASSERTED, H713_MIPS_RESET_REG);
	mdelay(12);
	writel(H713_MIPS_CLK_DISABLED, H713_MIPS_CLK_REG);
	mdelay(12);
	h713_display_prepared = false;
}

static void h713_display_prepare(void)
{
	/*
	 * SPL leaves the display bus clock/reset enabled, but the module clocks
	 * and their video2 parent are gated. Accessing the mixer or LVDS blocks
	 * before enabling this complete tree wedges the interconnect.
	 *
	 * Preserve the cold-boot divider and mux fields. On the bench they yield
	 * deint/panel=150 MHz, svp-dtl=200 MHz, and afbd=600 MHz.
	 */
	setbits_le32((void *)H713_DISPLAY_PLL_VIDEO2_REG, BIT(31));
	mdelay(12);
	setbits_le32((void *)H713_DISPLAY_DEINT_CLK_REG, BIT(31));
	setbits_le32((void *)H713_DISPLAY_PANEL_CLK_REG, BIT(31));
	setbits_le32((void *)H713_DISPLAY_SVP_DTL_CLK_REG, BIT(31));
	setbits_le32((void *)H713_DISPLAY_AFBD_CLK_REG, BIT(31));
	setbits_le32((void *)H713_DISPLAY_BGR_REG, BIT(16) | BIT(0));
	mdelay(12);

	/*
	 * Recovered 1080p TVTOP routing. These values were bench-verified before
	 * the first mixer access; the initial SPL values open only part of the
	 * display fabric.
	 */
	writel(0x00000001, H713_DISPLAY_TOP_REG + 0x04);
	writel(0x11111111, H713_DISPLAY_TOP_REG + 0x44);
	writel(0x11111111, H713_DISPLAY_TOP_REG + 0x88);
	writel(0xfff11111, H713_DISPLAY_TOP_REG + 0x00);
	writel(0x00011111, H713_DISPLAY_TOP_REG + 0x40);
	writel(0x00001111, H713_DISPLAY_TOP_REG + 0x80);
	writel(0xfff000ef, H713_DISPLAY_TOP_REG + 0x84);
	mdelay(12);

	/*
	 * The factory state machine performs this mixer write in its earlier
	 * phase, before the final MIPS reset sequence.
	 */
	writel(H713_DISPLAY_MIXER_CTRL_VALUE, H713_DISPLAY_MIXER_CTRL_REG);
	mdelay(12);
	h713_display_prepared = true;
	printf("H713 MIPS: display clocks/routing prepared\n");
}

/*
 * Bring up the capture block before the MIPS leaves reset.
 *
 * The firmware writes 0xc0 to HDMI-RX register 0x06840093 and polls bit 0 for
 * a reset/lock acknowledgement (0x4b13d044). It never comes back, and the
 * timeout cannot rescue it: the tick is a software counter driven by the CP0
 * Compare ISR, and interrupts are still masked this early in startup, so the
 * wait loop spins forever and its caller never returns.
 *
 * An earlier attempt released TVCAP from the poll loop once marker 14 appeared
 * and wedged the interconnect. Doing it here instead matches the factory
 * ordering, where the capture clocks and reset are up before the coprocessor
 * runs at all.
 */
static void h713_tvcap_prepare(void)
{
	setbits_le32((void *)H713_TVCAP_TCD3_CLK_REG, BIT(31));
	setbits_le32((void *)H713_TVCAP_VINCAP_DMA_CLK_REG, BIT(31));
	setbits_le32((void *)H713_TVCAP_HDMI_AUDIO_CLK_REG, BIT(31));
	mdelay(12);
	setbits_le32((void *)H713_TVCAP_BUS_CLK_REG, BIT(1) | BIT(0));
	mdelay(12);
	setbits_le32((void *)H713_TVCAP_BGR_REG, BIT(16) | BIT(0));
	mdelay(12);
	printf("H713 MIPS: TVCAP clocks/reset prepared before release\n");
}

/*
 * Zero everything the firmware may read except the two staged windows, so a
 * run does not depend on what the previous boot left in DRAM. Identical cold
 * boots have otherwise diverged by more than ten markers.
 */
static void h713_mips_clear_workspace(void)
{
	ulong tail = H713_MIPS_FW_ADDR + H713_MIPS_FW_SIZE;

	memset((void *)tail, 0, H713_MIPS_CFG_ADDR - tail);
	memset((void *)H713_MIPS_FB_ADDR, 0, H713_MIPS_FB_SIZE);

	flush_cache(H713_MIPS_FW_ADDR, H713_MIPS_CFG_ADDR - H713_MIPS_FW_ADDR);
	flush_cache(H713_MIPS_FB_ADDR, H713_MIPS_FB_SIZE);

	printf("H713 MIPS: workspace cleared (cfg@0x%08lx, tse@0x%08lx kept)\n",
	       H713_MIPS_CFG_ADDR, H713_MIPS_TSE_ADDR);
}

static u32 h713_mips_read_witness(void)
{
	invalidate_dcache_range(H713_MIPS_WITNESS_ADDR,
				H713_MIPS_WITNESS_ADDR +
				CONFIG_SYS_CACHELINE_SIZE);

	return readl(H713_MIPS_WITNESS_ADDR);
}

static void h713_mips_seed_witness(void)
{
	writel(H713_MIPS_WITNESS_SEED, H713_MIPS_WITNESS_ADDR);
	flush_cache(H713_MIPS_WITNESS_ADDR, CONFIG_SYS_CACHELINE_SIZE);
}

static u32 h713_mips_read_shmem(ulong offset)
{
	ulong addr = H713_MIPS_SHMEM_ADDR + offset;
	ulong start = addr & ~(CONFIG_SYS_CACHELINE_SIZE - 1);

	invalidate_dcache_range(start, start + CONFIG_SYS_CACHELINE_SIZE);

	return readl(addr);
}

/*
 * The rest of the CPU_COMM master init, transcribed from the firmware.
 *
 * display.bin carries the ARM master's init as well as its own slave path.
 * getCurCPUID at 0x8b1227b4 hardcodes 1, so the master block at 0x8b11ace8 is
 * compiled in but never executed by the firmware -- it is the vendor's
 * specification of what the ARM must build. comm_InitSpinLock and the
 * call-table sentinels above came from the same block and were both correct on
 * hardware, which is the basis for trusting the rest of it.
 *
 * Three structures remain. Everything else the master path does either is
 * already published above or touches only firmware BSS (0x8b124ba4,
 * 0x8b119bb4) and imposes no obligation on the ARM.
 */
#define H713_MIPS_SEQ_BASE_OFF		0x00000098UL
#define H713_MIPS_SEQ_STRIDE		2440
#define H713_MIPS_SEQ_PER_DIR		4880
#define H713_MIPS_SEQ_PER_CPU		9760
#define H713_MIPS_SEQ_SLOTS		20
#define H713_MIPS_SEQ_SLOT_SIZE		104
#define H713_MIPS_SEQ_RING_OFF		0x0c0
#define H713_MIPS_SEQ_SLOTS_OFF		0x168
#define H713_MIPS_SEQ_FIFO_OFF		0x078
#define H713_MIPS_SEQ_FIFO_CAPACITY	21

/*
 * 0x8b1197d4(cpu, dir), addressed by 0x8b1193d8 as
 * shared + 0x98 + 9760*cpu + 4880*dir + 2440*idx. Eight structs of 0x988
 * bytes; the array ends exactly where max_cpu begins at 0x4cd8, which is the
 * check that the formula is right.
 *
 * The block at +0x78 is the vendor's comm_fifo, and the twenty 104-byte
 * message slots at +0x168 end exactly at 0x988. Each slot is stamped with its
 * index and an invalid session, then pushed onto the ring as its ARM-physical
 * address -- so the ring starts full of free slots, twenty in a capacity of
 * twenty-one, the spare being the full-detect slot.
 */
static void h713_mips_init_share_seq(uint cpu, uint dir, uint idx)
{
	ulong base = H713_MIPS_SHMEM_ADDR + H713_MIPS_SEQ_BASE_OFF +
		     cpu * H713_MIPS_SEQ_PER_CPU + dir * H713_MIPS_SEQ_PER_DIR +
		     idx * H713_MIPS_SEQ_STRIDE;
	ulong slots = base + H713_MIPS_SEQ_SLOTS_OFF;
	ulong ring = base + H713_MIPS_SEQ_RING_OFF;
	ulong fifo = base + H713_MIPS_SEQ_FIFO_OFF;
	uint i;

	writeb(cpu, base + 0x00);
	writeb(dir, base + 0x01);
	writeb(idx, base + 0x02);
	writeb(0,   base + 0x08);
	writeb(H713_MIPS_SEQ_SLOTS, base + 0x10);
	writel(~0U, base + 0x14);
	writeb(H713_MIPS_SEQ_SLOTS, base + 0x68);
	writeb(0,   base + 0x69);
	writel(~0U, base + 0x6c);

	writel(0, fifo + 0x00);				/* rd_idx     */
	writel(H713_MIPS_SEQ_SLOTS, fifo + 0x04);	/* wr_idx     */
	writel(0, fifo + 0x08);				/* peak_count */
	writel(1, fifo + 0x0c);				/* track      */
	writel(H713_MIPS_SEQ_FIFO_CAPACITY, fifo + 0x10);
	writel(4, fifo + 0x14);				/* item_size  */
	writel(ring, fifo + 0x18);			/* base_addr  */
	writel(0, fifo + 0x1c);

	/*
	 * The name is selected by idx, not dir: 0x8b1197d4 runs its body twice
	 * per (cpu, dir) -- idx 0 names the FIFO "FreeCall" at 0x8b119b84, then
	 * the tail at 0x8b119bac sets idx to 1 and repeats for "FreeReturn".
	 * That is why the addressing formula has an idx term and why all eight
	 * structures are live.
	 */
	strncpy((char *)(base + 0x98), idx ? "FreeReturn" : "FreeCall", 0x20);
	writel(0, base + 0xb8);

	for (i = 0; i < H713_MIPS_SEQ_SLOTS; i++) {
		ulong slot = slots + i * H713_MIPS_SEQ_SLOT_SIZE;

		writew(i, slot + 0x04);		/* comm_msg.slot_index */
		writel(~0U, slot + 0x0c);	/* comm_msg.session_id */
		writel(slot, ring + i * 4);	/* free slot, ARM-physical */
	}
}

static void h713_mips_init_share_seqs(void)
{
	uint cpu, dir, idx;

	for (cpu = 0; cpu < 2; cpu++)
		for (dir = 0; dir < 2; dir++)
			for (idx = 0; idx < 2; idx++)
				h713_mips_init_share_seq(cpu, dir, idx);
}

/*
 * Five empty circular lists, each {self, 0, self, 0}, written at 0x8b11af1c
 * onwards. The offsets are irregular because other state sits between them.
 */
static void h713_mips_init_lists(void)
{
	static const ulong off[] = {
		0x4d10, 0x4d28, 0x4d58, 0x4d70, 0x4d88,
	};
	uint i;

	for (i = 0; i < ARRAY_SIZE(off); i++) {
		ulong l = H713_MIPS_SHMEM_ADDR + off[i];

		writel(l, l + 0x00);
		writel(0, l + 0x04);
		writel(l, l + 0x08);
		writel(0, l + 0x0c);
	}
}

/*
 * 256 records of 0x28 bytes from 0x4db0 to 0x75b0 -- the loop at 0x8b11af70,
 * which lands exactly on magic2 at 0x75b8. Each gets the same self-referencing
 * head, then 0x8b1234c8(2, 0, record) appends it to the free list rooted at
 * 0x4d80: tail at +0x10, back link at record+0x20, anchor 0x4d88 at
 * record+0x18, and a u16 count at 0x4d82.
 */
#define H713_MIPS_REC_BASE_OFF	0x4db0UL
#define H713_MIPS_REC_END_OFF	0x75b0UL
#define H713_MIPS_REC_STRIDE	0x28
#define H713_MIPS_REC_LIST_OFF	0x4d80UL
#define H713_MIPS_REC_ANCHOR	0x4d88UL

static void h713_mips_init_record_pool(void)
{
	ulong list = H713_MIPS_SHMEM_ADDR + H713_MIPS_REC_LIST_OFF;
	ulong anchor = H713_MIPS_SHMEM_ADDR + H713_MIPS_REC_ANCHOR;
	ulong prev = 0;
	ulong off;
	u16 count = 0;

	for (off = H713_MIPS_REC_BASE_OFF; off < H713_MIPS_REC_END_OFF;
	     off += H713_MIPS_REC_STRIDE) {
		ulong r = H713_MIPS_SHMEM_ADDR + off;

		writel(r, r + 0x00);
		writel(0, r + 0x04);
		writel(r, r + 0x08);
		writel(0, r + 0x0c);

		writel(prev, r + 0x20);
		writel(anchor, r + 0x18);
		writel(0, r + 0x1c);
		writel(0, r + 0x24);
		if (prev)
			writel(0, prev + 0x04);

		prev = r + 0x18;
		writel(prev, list + 0x10);
		writel(0, list + 0x14);
		count++;
	}
	writew(count, list + 0x02);
}

static void h713_mips_prepare_ready_probe(void)
{
	int i;

	memset((void *)H713_MIPS_SHMEM_ADDR, 0, H713_MIPS_SHMEM_SIZE);

	/*
	 * InitCommMem takes the slave path when U-Boot publishes valid magic
	 * words. That path assumes the ARM master has already run
	 * comm_InitSpinLock(). Reproduce its exact 12-byte entry layout:
	 * type=free, status=free, mutex=0, owner=free, refcount=0, and no
	 * assigned thread. Leaving these bytes zero makes comm_SpinLock(3)
	 * wait forever before setCPUReady(1) can publish the MIPS flag.
	 */
	for (i = 0; i < H713_MIPS_SHMEM_LOCK_COUNT; i++) {
		u8 *lock = (u8 *)(H713_MIPS_SHMEM_ADDR +
				 i * H713_MIPS_SHMEM_LOCK_SIZE);

		lock[0] = H713_MIPS_SHMEM_LOCK_FREE;
		lock[1] = H713_MIPS_SHMEM_LOCK_FREE;
		lock[2] = 0;
		lock[3] = H713_MIPS_SHMEM_LOCK_FREE;
		*(u32 *)(lock + 4) = 0;
		*(u32 *)(lock + 8) = H713_MIPS_SHMEM_LOCK_THREAD_NONE;
	}

	/*
	 * The ARM master also clears the call-entry region, then seeds the
	 * per-entry link field with -1. The exact image does this at raw
	 * display.bin+0x1ae4c..0x1ae88: the table begins at shared+0x75c8,
	 * entries are 0x60 bytes, and the loop writes -1 at entry+0x5c until
	 * shared+0x240c4 (1224 entries). A zero link makes the MIPS insertion
	 * routine reject the first free slot before it can queue a request.
	 */
	writel(0, H713_MIPS_SHMEM_ADDR + H713_MIPS_SHMEM_CALL_VERSION_OFF);
	writel(0, H713_MIPS_SHMEM_ADDR + H713_MIPS_SHMEM_CALL_COUNT_OFF);
	for (i = 0; i < H713_MIPS_SHMEM_CALL_ENTRY_COUNT; i++)
		writel(~0U, H713_MIPS_SHMEM_ADDR +
		       H713_MIPS_SHMEM_CALL_TABLE_OFF +
		       i * H713_MIPS_SHMEM_CALL_ENTRY_SIZE +
		       H713_MIPS_SHMEM_CALL_NEXT_OFF);

	writel(3, H713_MIPS_SHMEM_ADDR + H713_MIPS_SHMEM_MAX_CPU_OFF);
	writel(H713_MIPS_SHMEM_ARM_READY,
	       H713_MIPS_SHMEM_ADDR + H713_MIPS_SHMEM_ARM_FLAG_OFF);
	writel(0, H713_MIPS_SHMEM_ADDR + H713_MIPS_SHMEM_MIPS_FLAG_OFF);

	h713_mips_init_share_seqs();
	h713_mips_init_lists();
	h713_mips_init_record_pool();

	/*
	 * Publish the magic words last. The MIPS firmware treats both markers,
	 * the ARM ready/app-ready flag, and an unlocked hardware spinlock 0 as
	 * the handoff from the ARM-side CPU_COMM master.
	 */
	writel(H713_MIPS_SHMEM_MAGIC,
	       H713_MIPS_SHMEM_ADDR + H713_MIPS_SHMEM_MAGIC1_OFF);
	writel(H713_MIPS_SHMEM_MAGIC,
	       H713_MIPS_SHMEM_ADDR + H713_MIPS_SHMEM_MAGIC2_OFF);
	flush_cache(H713_MIPS_SHMEM_ADDR, H713_MIPS_SHMEM_SIZE);

	printf("H713 MIPS: readiness probe shared memory prepared "
	       "(12 spinlocks, 1224 call entries, 8 share_seq, "
	       "5 lists, 256 records)\n");
}

static int h713_mips_apply_trace(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(h713_mips_trace_patches); i++) {
		const struct h713_mips_patch *patch =
			&h713_mips_trace_patches[i];

		if (readl(patch->addr) != patch->expected) {
			printf("H713 MIPS: trace site 0x%08lx is not pristine\n",
			       patch->addr);
			return -EINVAL;
		}
	}

	for (i = 0; i < ARRAY_SIZE(h713_mips_trace_patches); i++) {
		const struct h713_mips_patch *patch =
			&h713_mips_trace_patches[i];

		writel(patch->replacement, patch->addr);
	}
	flush_cache(H713_MIPS_FW_ADDR, H713_MIPS_FW_WINDOW_SIZE);
	printf("H713 MIPS: volatile handshake trace installed\n");

	return 0;
}

static int h713_mips_apply_stability(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(h713_mips_stability_patches); i++) {
		const struct h713_mips_patch *patch =
			&h713_mips_stability_patches[i];

		if (readl(patch->addr) != patch->expected) {
			printf("H713 MIPS: stability site 0x%08lx is not pristine\n",
			       patch->addr);
			return -EINVAL;
		}
	}

	for (i = 0; i < ARRAY_SIZE(h713_mips_stability_patches); i++) {
		const struct h713_mips_patch *patch =
			&h713_mips_stability_patches[i];

		writel(patch->replacement, patch->addr);
	}
	flush_cache(H713_MIPS_FW_ADDR, H713_MIPS_FW_WINDOW_SIZE);
	printf("H713 MIPS: uncached heartbeat/exception diagnostics installed\n");

	return 0;
}

/*
 * Report every trace slot that changed since the previous scan, as it changes.
 * The firmware's stalls wedge the interconnect often enough that a dump taken
 * after the poll loop is the one thing a failing run cannot produce. Streaming
 * costs one UART line per marker and makes the last line printed before a hang
 * the answer.
 */
static void h713_mips_stream_trace(u32 *shadow)
{
	ulong base = H713_MIPS_SHMEM_ADDR + H713_MIPS_TRACE_OFF;
	ulong start = base & ~(CONFIG_SYS_CACHELINE_SIZE - 1);
	ulong end = ALIGN(base + H713_MIPS_TRACE_COUNT * sizeof(u32),
			  CONFIG_SYS_CACHELINE_SIZE);
	int i;

	invalidate_dcache_range(start, end);

	for (i = 0; i < H713_MIPS_TRACE_COUNT; i++) {
		u32 value = readl(base + i * sizeof(u32));

		if (value == shadow[i])
			continue;

		shadow[i] = value;
		if (i == H713_MIPS_TRACE_DBG_ADDR)
			printf("H713 MIPS: sys:dbg_buf=0x%08x\n", value);
		else if (i == H713_MIPS_TRACE_DBG_SIZE)
			printf("H713 MIPS: sys:dbg_buf_size=0x%08x\n", value);
		else if (i == H713_MIPS_TRACE_REG_COUNT)
			printf("H713 MIPS: hal registration count=%u\n", value);
		else if (i == H713_MIPS_TRACE_REG_OBJECT)
			printf("H713 MIPS: hal registration object=0x%08x\n",
			       value);
		else if (i == H713_MIPS_TRACE_REG_CALLBACK)
			printf("H713 MIPS: hal registration callback=0x%08x\n",
			       value);
		else
			printf("H713 MIPS: trace[%d]=%u\n", i, value);
	}
}

static void h713_mips_print_trace(void)
{
	int i;

	printf("H713 MIPS: handshake trace");
	for (i = 0; i < H713_MIPS_TRACE_MARKER_COUNT; i++)
		printf(" %u", h713_mips_read_shmem(H713_MIPS_TRACE_OFF +
						   i * sizeof(u32)));
	printf("\n");
	printf("H713 MIPS: debug buffer addr=0x%08x size=0x%08x\n",
	       h713_mips_read_shmem(H713_MIPS_TRACE_OFF +
				   H713_MIPS_TRACE_DBG_ADDR * sizeof(u32)),
	       h713_mips_read_shmem(H713_MIPS_TRACE_OFF +
				   H713_MIPS_TRACE_DBG_SIZE * sizeof(u32)));
	printf("H713 MIPS: hal registrations=%u object=0x%08x "
	       "callback=0x%08x\n",
	       h713_mips_read_shmem(H713_MIPS_TRACE_OFF +
				   H713_MIPS_TRACE_REG_COUNT * sizeof(u32)),
	       h713_mips_read_shmem(H713_MIPS_TRACE_OFF +
				   H713_MIPS_TRACE_REG_OBJECT * sizeof(u32)),
	       h713_mips_read_shmem(H713_MIPS_TRACE_OFF +
				   H713_MIPS_TRACE_REG_CALLBACK * sizeof(u32)));
	printf("H713 MIPS: call table version=%u count=%u first-next=0x%08x\n",
	       h713_mips_read_shmem(H713_MIPS_SHMEM_CALL_VERSION_OFF),
	       h713_mips_read_shmem(H713_MIPS_SHMEM_CALL_COUNT_OFF),
	       h713_mips_read_shmem(H713_MIPS_SHMEM_CALL_TABLE_OFF +
				   H713_MIPS_SHMEM_CALL_NEXT_OFF));
}

static int h713_mips_release_reset(bool publish_shmem)
{
	writel(H713_MIPS_CLK_VALUE, H713_MIPS_CLK_REG);
	writel(H713_MIPS_RESET_ASSERTED, H713_MIPS_RESET_REG);
	mdelay(12);
	writel(H713_MIPS_RESET_STAGE1, H713_MIPS_RESET_REG);
	mdelay(12);
	writel(H713_MIPS_RESET_STAGE2, H713_MIPS_RESET_REG);
	mdelay(12);
	writel(H713_MIPS_RESET_STAGE3, H713_MIPS_RESET_REG);
	mdelay(12);

	if (publish_shmem) {
		writel(H713_MIPS_SHMEM_ADDR, H713_MIPS_SHARE_ADDR_REG);
		writel(H713_MIPS_SHMEM_SIZE, H713_MIPS_SHARE_SIZE_REG);
		if (readl(H713_MIPS_SHARE_ADDR_REG) != H713_MIPS_SHMEM_ADDR ||
		    readl(H713_MIPS_SHARE_SIZE_REG) != H713_MIPS_SHMEM_SIZE) {
			printf("H713 MIPS: share-register publication failed\n");
			return -EIO;
		}
	}

	writel(H713_MIPS_FW_ADDR, H713_MIPS_BOOTADDR_REG);
	writel(H713_MIPS_RESET_RELEASED, H713_MIPS_RESET_REG);

	return 0;
}

static int h713_mips_verify(void)
{
	u8 digest[SHA256_SUM_LEN];

	sha256_csum_wd((const u8 *)H713_MIPS_FW_ADDR, H713_MIPS_FW_SIZE,
		       digest, CHUNKSZ_SHA256);

	printf("H713 MIPS: display.bin SHA-256 ");
	h713_mips_print_digest(digest);
	printf("\n");

	if (memcmp(digest, h713_mips_fw_sha256, sizeof(digest))) {
		printf("H713 MIPS: firmware identity rejected\n");
		return -EPERM;
	}

	printf("H713 MIPS: firmware identity accepted\n");
	return 0;
}

static int h713_mips_load(const char *ifname, const char *dev,
			  const char *path)
{
	loff_t file_size;
	loff_t len_read;
	int ret;

	ret = fs_set_blk_dev(ifname, dev, FS_TYPE_ANY);
	if (ret) {
		printf("H713 MIPS: cannot select %s %s\n", ifname, dev);
		return ret;
	}

	ret = fs_size(path, &file_size);
	if (ret) {
		printf("H713 MIPS: cannot stat %s\n", path);
		return ret;
	}

	if (file_size != H713_MIPS_FW_SIZE) {
		printf("H713 MIPS: rejected size 0x%llx (expected 0x%lx)\n",
		       file_size, H713_MIPS_FW_SIZE);
		return -EINVAL;
	}

	h713_mips_stop();
	memset((void *)H713_MIPS_FW_ADDR, 0, H713_MIPS_FW_WINDOW_SIZE);

	ret = fs_set_blk_dev(ifname, dev, FS_TYPE_ANY);
	if (ret)
		return ret;

	len_read = file_size;
	ret = fs_read(path, H713_MIPS_FW_ADDR, 0, file_size, &len_read);
	if (ret || len_read != file_size) {
		printf("H713 MIPS: read failed (%d, 0x%llx/0x%llx bytes)\n",
		       ret, len_read, file_size);
		h713_mips_stop();
		return ret ? ret : -EIO;
	}

	flush_cache(H713_MIPS_FW_ADDR, H713_MIPS_FW_WINDOW_SIZE);
	printf("H713 MIPS: loaded %s to 0x%08lx (0x%llx bytes)\n",
	       path, H713_MIPS_FW_ADDR, len_read);

	ret = h713_mips_verify();
	if (ret)
		h713_mips_stop();

	return ret;
}

static int h713_mips_start(void)
{
	u32 status;
	int ret;

	ret = h713_mips_verify();
	if (ret) {
		h713_mips_stop();
		return ret;
	}

	/*
	 * loady and other generic loaders do not clear the firmware's runtime
	 * tail. Match the filesystem load path so BSS/heap never inherit stale
	 * DRAM from an earlier boot.
	 */
	memset((void *)(H713_MIPS_FW_ADDR + H713_MIPS_FW_SIZE), 0,
	       H713_MIPS_FW_WINDOW_SIZE - H713_MIPS_FW_SIZE);
	flush_cache(H713_MIPS_FW_ADDR, H713_MIPS_FW_WINDOW_SIZE);

	/*
	 * The firmware entry point clears BSS from 0x4b232c00 through
	 * 0x4bac7c28 before entering C code. Seed a word within that range but
	 * just beyond the ARM-cleared load window. Observing zero here proves
	 * that the MIPS executed a firmware-owned store; CPU status alone only
	 * proves reset release.
	 */
	h713_mips_seed_witness();
	h713_display_prepare();

	ret = h713_mips_release_reset(false);
	if (ret) {
		h713_mips_stop();
		return ret;
	}
	mdelay(300);

	status = readl(H713_MIPS_STATUS_REG);
	printf("H713 MIPS: reset released, CPU status=0x%08x\n", status);

	if (status != H713_MIPS_STATUS_RELEASED) {
		printf("H713 MIPS: unexpected CPU status; returning to reset\n");
		h713_mips_stop();
		return -EIO;
	}

	status = h713_mips_read_witness();
	printf("H713 MIPS: BSS witness@0x%08lx=0x%08x\n",
	       H713_MIPS_WITNESS_ADDR, status);
	if (status) {
		printf("H713 MIPS: no execution witness; returning to reset\n");
		h713_mips_stop();
		return -ETIMEDOUT;
	}

	printf("H713 MIPS: firmware execution proven by BSS clear\n");
	printf("H713 MIPS: higher-level firmware readiness is not yet proven\n");

	return 0;
}

/*
 * Release the MIPS the way stock fastlogo does, without touching the display
 * fabric.
 *
 * Stock configures the fabric from LogoRegData.bin, writes INCAP and the LVDS
 * enable, releases the coprocessor, waits 300 ms and finalises LVDS. Replaying
 * that needs a release step that leaves the vendor register state alone --
 * h713_mips_start() would re-apply h713_display_prepare() over the top of it.
 *
 * The cache flush is the part that is easy to miss: fastboot staging writes
 * through the ARM cache, so a verify reads back correct while DRAM still holds
 * stale bytes and the MIPS fetches garbage.
 */
/*
 * Rx_HDCP14_LoadKey polls HDMI-RX 0x06840093 and gives up after 0x33 ticks.
 * The tick is a ThreadX software counter driven by the CP0 Compare ISR, and
 * interrupts are masked this early, so the timeout cannot expire and a failure
 * the firmware is built to survive becomes a hang. Rewriting the loop bound to
 * zero makes the wait give up on its first pass, taking the same path a real
 * timeout would. Applied after the hash check, so the image is authenticated
 * before it is modified.
 *
 * The address is firmware-specific: the same loop sits at 0x4b13d0a4 in the
 * revision found in an earlier captured dump and at 0x4b13d6f8 in the image
 * this board actually carries. The guard below compares the instruction before
 * writing, so a wrong offset reports rather than corrupting the firmware.
 */
#define H713_MIPS_HDCP_WAIT_INSN	0x4b13d6f8UL
#define H713_MIPS_HDCP_WAIT_ORIG	0x2c630033
#define H713_MIPS_HDCP_WAIT_NONE	0x2c630000

static int h713_mips_release_raw(bool skip_hdcp_wait, bool publish_shmem,
				 bool trace, bool stability)
{
	u32 status, witness;
	int elapsed;
	int ret;

	ret = h713_mips_verify();
	if (ret)
		return ret;

	if ((trace || stability) && !publish_shmem)
		return -EINVAL;
	if (trace && stability)
		return -EINVAL;

	if (skip_hdcp_wait) {
		u32 insn = readl(H713_MIPS_HDCP_WAIT_INSN);

		if (insn != H713_MIPS_HDCP_WAIT_ORIG) {
			printf("H713 MIPS: HDCP wait site is 0x%08x, expected 0x%08x\n",
			       insn, H713_MIPS_HDCP_WAIT_ORIG);
			return -EINVAL;
		}
		writel(H713_MIPS_HDCP_WAIT_NONE, H713_MIPS_HDCP_WAIT_INSN);
		printf("H713 MIPS: HDCP key-load wait defeated\n");
	}

	if (publish_shmem)
		h713_mips_prepare_ready_probe();

	if (trace) {
		ret = h713_mips_apply_trace();
		if (ret)
			return ret;
		memset(trace_shadow, 0, sizeof(trace_shadow));
	}
	if (stability) {
		ret = h713_mips_apply_stability();
		if (ret)
			return ret;
	}

	flush_cache(H713_MIPS_FW_ADDR, H713_MIPS_FW_WINDOW_SIZE);
	h713_mips_seed_witness();

	ret = h713_mips_release_reset(publish_shmem);
	if (ret)
		return ret;
	if (trace) {
		for (elapsed = 0; elapsed < 300000; elapsed += 1000) {
			h713_mips_stream_trace(trace_shadow);
			udelay(1000);
		}
		h713_mips_stream_trace(trace_shadow);
	} else {
		mdelay(300);
	}

	status = readl(H713_MIPS_STATUS_REG);
	witness = h713_mips_read_witness();
	printf("H713 MIPS: released, status=0x%08x witness=0x%08x\n",
	       status, witness);

	if (status != H713_MIPS_STATUS_RELEASED) {
		printf("H713 MIPS: unexpected CPU status\n");
		return -EIO;
	}
	/*
	 * The seed sits inside the firmware's BSS (0x4b232c00..0x4bac7c40), so
	 * startup zeroes it -- but the firmware then *uses* that memory, and
	 * with elog buffering enabled it stores a pointer there. Demanding a
	 * zero read therefore reports a false failure on a perfectly healthy
	 * run. Any value other than the seed proves the MIPS wrote to it.
	 */
	if (witness == H713_MIPS_WITNESS_SEED) {
		printf("H713 MIPS: seed intact -- firmware not executing\n");
		return -ETIMEDOUT;
	}

	printf("H713 MIPS: firmware execution proven (witness overwritten)\n");
	return 0;
}

static int h713_mips_monitor_stability(void)
{
	u32 previous = h713_mips_read_shmem(H713_MIPS_DIAG_HEARTBEAT_OFF);
	bool advanced = true;
	int second;

	printf("H713 MIPS: starting %d-second stability window, tick=%u\n",
	       H713_MIPS_STABILITY_SECONDS, previous);
	for (second = 1; second <= H713_MIPS_STABILITY_SECONDS; second++) {
		u32 exception;
		u32 heartbeat;
		u32 mips_flag;
		u32 status;

		mdelay(1000);
		heartbeat =
			h713_mips_read_shmem(H713_MIPS_DIAG_HEARTBEAT_OFF);
		exception =
			h713_mips_read_shmem(H713_MIPS_DIAG_EXCEPTION_OFF);
		mips_flag =
			h713_mips_read_shmem(H713_MIPS_SHMEM_MIPS_FLAG_OFF);
		status = readl(H713_MIPS_STATUS_REG);

		printf("H713 MIPS: stability %2ds tick=%u delta=%u "
		       "status=%08x MIPS=%08x exception=%u\n",
		       second, heartbeat, heartbeat - previous, status,
		       mips_flag, exception);

		if (exception) {
			printf("H713 MIPS: %s exception status=%08x cause=%08x "
			       "epc=%08x badvaddr=%08x\n",
			       exception == H713_MIPS_DIAG_EXCEPTION_CACHE ?
			       "cache" : "general",
			       h713_mips_read_shmem(H713_MIPS_DIAG_STATUS_OFF),
			       h713_mips_read_shmem(H713_MIPS_DIAG_CAUSE_OFF),
			       h713_mips_read_shmem(H713_MIPS_DIAG_EPC_OFF),
			       h713_mips_read_shmem(H713_MIPS_DIAG_BADVADDR_OFF));
			return -EFAULT;
		}
		if (heartbeat == previous)
			advanced = false;
		if (status != H713_MIPS_STATUS_RELEASED ||
		    (mips_flag & (H713_MIPS_SHMEM_MIPS_READY |
				  H713_MIPS_SHMEM_MIPS_APP_READY)) !=
		    (H713_MIPS_SHMEM_MIPS_READY |
		     H713_MIPS_SHMEM_MIPS_APP_READY)) {
			printf("H713 MIPS: readiness/status changed during "
			       "stability window\n");
			return -EIO;
		}
		previous = heartbeat;
	}

	if (!advanced) {
		printf("H713 MIPS: stability failed: heartbeat stalled during "
		       "the observation window\n");
		return -ETIMEDOUT;
	}

	printf("H713 MIPS: stability passed: heartbeat advanced for %d seconds "
	       "with no recorded exception\n",
	       H713_MIPS_STABILITY_SECONDS);
	return 0;
}

static int h713_mips_wait_ready(int timeout_us, bool trace)
{
	u32 arm_flag;
	u32 magic1;
	u32 magic2;
	u32 mips_flag = 0;
	u32 status;
	u32 witness;
	int elapsed;

	for (elapsed = 0; elapsed < timeout_us; elapsed += 1000) {
		if (trace)
			h713_mips_stream_trace(trace_shadow);
		mips_flag =
			h713_mips_read_shmem(H713_MIPS_SHMEM_MIPS_FLAG_OFF);
		if ((mips_flag & (H713_MIPS_SHMEM_MIPS_READY |
				 H713_MIPS_SHMEM_MIPS_APP_READY)) ==
		    (H713_MIPS_SHMEM_MIPS_READY |
		     H713_MIPS_SHMEM_MIPS_APP_READY))
			break;
		udelay(1000);
	}

	if (trace) {
		h713_mips_stream_trace(trace_shadow);
		h713_mips_print_trace();
	}

	status = readl(H713_MIPS_STATUS_REG);
	witness = h713_mips_read_witness();
	magic1 = h713_mips_read_shmem(H713_MIPS_SHMEM_MAGIC1_OFF);
	magic2 = h713_mips_read_shmem(H713_MIPS_SHMEM_MAGIC2_OFF);
	arm_flag = h713_mips_read_shmem(H713_MIPS_SHMEM_ARM_FLAG_OFF);
	mips_flag = h713_mips_read_shmem(H713_MIPS_SHMEM_MIPS_FLAG_OFF);

	printf("H713 MIPS: readiness status=0x%08x witness=0x%08x\n",
	       status, witness);
	printf("H713 MIPS: CPU_COMM magic=%08x/%08x ARM=%08x MIPS=%08x\n",
	       magic1, magic2, arm_flag, mips_flag);

	if (status != H713_MIPS_STATUS_RELEASED ||
	    witness == H713_MIPS_WITNESS_SEED ||
	    magic1 != H713_MIPS_SHMEM_MAGIC ||
	    magic2 != H713_MIPS_SHMEM_MAGIC ||
	    arm_flag != H713_MIPS_SHMEM_ARM_READY ||
	    !(mips_flag & H713_MIPS_SHMEM_MIPS_READY)) {
		printf("H713 MIPS: firmware readiness not proven; core left running\n");
		return -ETIMEDOUT;
	}

	printf("H713 MIPS: firmware readiness proven by MIPS READY\n");
	if (!(mips_flag & H713_MIPS_SHMEM_MIPS_APP_READY)) {
		printf("H713 MIPS: application readiness not proven; "
		       "core left running\n");
		return -ETIMEDOUT;
	}

	printf("H713 MIPS: application readiness proven\n");
	return 0;
}

static int h713_mips_probe_ready(bool trace, bool release_tvcap,
				 bool skip_wait)
{
	u32 tvcap_saved_tcd3 = 0;
	u32 tvcap_saved_vincap = 0;
	u32 tvcap_saved_bus = 0;
	u32 tvcap_saved_hdmi_audio = 0;
	u32 tvcap_saved_bgr = 0;
	u32 arm_flag;
	u32 magic1;
	u32 magic2;
	u32 mips_flag = 0;
	u32 status;
	u32 witness;
	bool tvcap_released = false;
	int elapsed;
	int ret;
	int timeout = trace ? H713_MIPS_TRACE_TIMEOUT_US :
			      H713_MIPS_READY_TIMEOUT_US;

	ret = h713_mips_verify();
	if (ret) {
		h713_mips_stop();
		return ret;
	}

	h713_mips_clear_workspace();
	h713_mips_seed_witness();
	h713_mips_prepare_ready_probe();
	if (trace) {
		ret = h713_mips_apply_trace();
		if (ret)
			goto out_stop;
	}
	/*
	 * Rx_HDCP14_LoadKey polls HDMI-RX 0x06840093 bit 0 for a key-load
	 * acknowledgement and gives up after 0x33 ticks with "time out!". The
	 * tick is a software counter driven by the CP0 Compare ISR, so while
	 * interrupts are masked this early the timeout can never expire and a
	 * failure the firmware is designed to survive becomes a hang.
	 *
	 * Rewrite the loop bound inside marker 104's cave so the wait gives up
	 * on its first iteration, taking the same path a real timeout would.
	 */
	if (skip_wait) {
		writel(0x2c630000, 0x4b101168);
		flush_cache(H713_MIPS_FW_ADDR, H713_MIPS_FW_WINDOW_SIZE);
		printf("H713 MIPS: tick wait loops forced to expire at once\n");
	}

	h713_display_prepare();

	/* Capture the cold values before enabling, so stop() can undo this. */
	if (release_tvcap) {
		tvcap_saved_tcd3 = readl(H713_TVCAP_TCD3_CLK_REG);
		tvcap_saved_vincap = readl(H713_TVCAP_VINCAP_DMA_CLK_REG);
		tvcap_saved_bus = readl(H713_TVCAP_BUS_CLK_REG);
		tvcap_saved_hdmi_audio =
			readl(H713_TVCAP_HDMI_AUDIO_CLK_REG);
		tvcap_saved_bgr = readl(H713_TVCAP_BGR_REG);

		h713_tvcap_prepare();
		tvcap_released = true;
	}

	ret = h713_mips_release_reset(true);
	if (ret)
		goto out_stop;

	if (trace)
		memset(trace_shadow, 0, sizeof(trace_shadow));

	for (elapsed = 0; elapsed < timeout; elapsed += 1000) {
		if (trace)
			h713_mips_stream_trace(trace_shadow);


		mips_flag =
			h713_mips_read_shmem(H713_MIPS_SHMEM_MIPS_FLAG_OFF);
		if (mips_flag & H713_MIPS_SHMEM_MIPS_READY)
			break;
		udelay(1000);
	}

	if (trace)
		h713_mips_stream_trace(trace_shadow);

	status = readl(H713_MIPS_STATUS_REG);
	witness = h713_mips_read_witness();
	magic1 = h713_mips_read_shmem(H713_MIPS_SHMEM_MAGIC1_OFF);
	magic2 = h713_mips_read_shmem(H713_MIPS_SHMEM_MAGIC2_OFF);
	arm_flag = h713_mips_read_shmem(H713_MIPS_SHMEM_ARM_FLAG_OFF);
	mips_flag = h713_mips_read_shmem(H713_MIPS_SHMEM_MIPS_FLAG_OFF);

	if (trace)
		h713_mips_print_trace();
	printf("H713 MIPS: readiness probe status=0x%08x witness=0x%08x\n",
	       status, witness);
	printf("H713 MIPS: CPU_COMM magic=%08x/%08x ARM=%08x MIPS=%08x\n",
	       magic1, magic2, arm_flag, mips_flag);

	if (status != H713_MIPS_STATUS_RELEASED || witness ||
	    magic1 != H713_MIPS_SHMEM_MAGIC ||
	    magic2 != H713_MIPS_SHMEM_MAGIC ||
	    arm_flag != H713_MIPS_SHMEM_ARM_READY ||
	    !(mips_flag & H713_MIPS_SHMEM_MIPS_READY)) {
		printf("H713 MIPS: firmware readiness not proven\n");
		ret = -ETIMEDOUT;
	} else {
		printf("H713 MIPS: firmware readiness proven by MIPS READY\n");
		ret = 0;
	}

out_stop:
	h713_mips_stop();
	if (tvcap_released) {
		writel(tvcap_saved_bgr, H713_TVCAP_BGR_REG);
		writel(tvcap_saved_hdmi_audio,
		       H713_TVCAP_HDMI_AUDIO_CLK_REG);
		writel(tvcap_saved_bus, H713_TVCAP_BUS_CLK_REG);
		writel(tvcap_saved_vincap, H713_TVCAP_VINCAP_DMA_CLK_REG);
		writel(tvcap_saved_tcd3, H713_TVCAP_TCD3_CLK_REG);
		printf("H713 MIPS: TVCAP trace state restored\n");
	}
	printf("H713 MIPS: readiness probe returned the core to reset\n");

	return ret;
}


/*
 * Scan the coprocessor's workspace for its own log output.
 *
 * display.bin keeps an "elog" buffer whose location moves between firmware
 * revisions, so rather than hardcode an address that goes stale, walk the
 * region and print runs of printable text. After a run this surfaces whatever
 * the firmware said about its own startup, which beats inferring from the
 * outside.
 */
#define H713_LOG_MIN_RUN	12
#define H713_LOG_MAX_LINES	200

static int h713_mips_log(ulong start, ulong end)
{
	ulong a = start;
	uint printed = 0;

	if (end <= start)
		return -EINVAL;

	while (a < end && printed < H713_LOG_MAX_LINES) {
		ulong run = a;
		uint len = 0;

		while (run + len < end) {
			u8 c = readb(run + len);

			if (c == '\n' || c == '\t' || (c >= 0x20 && c < 0x7f))
				len++;
			else
				break;
		}

		if (len >= H713_LOG_MIN_RUN) {
			uint i;

			printf("0x%08lx: ", run);
			for (i = 0; i < len; i++) {
				u8 c = readb(run + i);

				putc(c == '\n' ? ' ' : c);
			}
			printf("\n");
			printed++;
		}

		a = run + (len ? len : 1);
	}

	printf("H713 MIPS: %u text run(s) in 0x%08lx..0x%08lx%s\n",
	       printed, start, end,
	       printed == H713_LOG_MAX_LINES ? " (truncated)" : "");

	return 0;
}

static void h713_mips_status(void)
{
	u32 witness = h713_mips_read_witness();

	printf("H713 MIPS: clk=0x%08x reset=0x%08x status=0x%08x ",
	       readl(H713_MIPS_CLK_REG), readl(H713_MIPS_RESET_REG),
	       readl(H713_MIPS_STATUS_REG));
	printf("bootaddr=0x%08x\n", readl(H713_MIPS_BOOTADDR_REG));
	printf("H713 MIPS: BSS=0x%08lx..0x%08lx witness@0x%08lx=0x%08x\n",
	       H713_MIPS_BSS_START, H713_MIPS_BSS_END,
	       H713_MIPS_WITNESS_ADDR, witness);
	printf("H713 MIPS: display prerequisites=%s\n",
	       h713_display_prepared ? "applied" : "not applied");
}

static int do_h713_mips(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[])
{
	int ret;

	if (argc < 2)
		return CMD_RET_USAGE;

	/* Default range covers the firmware's data and BSS working set. */
	if (!strcmp(argv[1], "log")) {
		ulong a = argc > 2 ? hextoul(argv[2], NULL) : 0x4b232000;
		ulong b = argc > 3 ? hextoul(argv[3], NULL) : 0x4bd00000;

		return h713_mips_log(a, b) ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	if (!strcmp(argv[1], "status")) {
		h713_mips_status();
		return CMD_RET_SUCCESS;
	}

	if (!strcmp(argv[1], "stop")) {
		h713_mips_stop();
		h713_mips_status();
		return CMD_RET_SUCCESS;
	}

	if (!strcmp(argv[1], "verify")) {
		ret = h713_mips_verify();
		return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	/*
	 * Prepare the display clock tree without releasing the MIPS. Several
	 * blocks are only ARM-accessible once this tree is up, so isolating a
	 * register probe from firmware activity needs prep on its own.
	 */
	if (!strcmp(argv[1], "prepare")) {
		h713_display_prepare();
		h713_mips_status();
		return CMD_RET_SUCCESS;
	}

	if (!strcmp(argv[1], "start")) {
		ret = h713_mips_start();
		return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	if (!strcmp(argv[1], "release")) {
		ret = h713_mips_release_raw(false, false, false, false);
		return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	if (!strcmp(argv[1], "probe-ready")) {
		ret = h713_mips_probe_ready(false, false, false);
		return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	/*
	 * The ARM leaves TVCAP alone by default; "probe-trace tvcap" opts into
	 * releasing it, which is known to wedge the board.
	 */
	if (!strcmp(argv[1], "probe-trace")) {
		bool tvcap = argc == 3 && !strcmp(argv[2], "tvcap");
		bool no_wait = argc == 3 && !strcmp(argv[2], "no-wait");

		if (argc > 3 || (argc == 3 && !tvcap && !no_wait))
			return CMD_RET_USAGE;

		ret = h713_mips_probe_ready(true, tvcap, no_wait);
		return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	if (!strcmp(argv[1], "load") || !strcmp(argv[1], "boot")) {
		if (argc != 5)
			return CMD_RET_USAGE;

		ret = h713_mips_load(argv[2], argv[3], argv[4]);
		if (!ret && !strcmp(argv[1], "boot"))
			ret = h713_mips_start();

		return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	return CMD_RET_USAGE;
}

U_BOOT_CMD(h713_mips, 5, 0, do_h713_mips,
	   "manually manage the H713 display MIPS firmware",
	   "status\n"
	   "h713_mips log [start] [end]\n"
	   "h713_mips stop\n"
	   "h713_mips verify\n"
	   "h713_mips prepare\n"
	   "h713_mips start\n"
	   "h713_mips release\n"
	   "h713_mips probe-ready\n"
	   "h713_mips probe-trace [tvcap|no-wait]\n"
	   "h713_mips load <interface> <dev[:part]> <path>\n"
	   "h713_mips boot <interface> <dev[:part]> <path>"
);

/*
 * LogoRegData.bin replay.
 *
 * Stock U-Boot drives the panel itself for its boot logo: it parses
 * mips/LogoRegData.bin and applies the register table, then blits a bitmap.
 * The file is a container of 16-byte {address, value, mask, type} records.
 *
 * The record semantics below are read off the vendor applier itself, at
 * 0x4a025164 in this board's stock U-Boot 2018.05 (Thumb-2, load base
 * 0x4a000000). It walks a {?, records, count} descriptor and switches on the
 * type word at +0xc:
 *
 *   type <= 4    masked read-modify-write: *addr = (*addr & ~mask) | value.
 *                Stock uses a 32-bit access for every type in that range --
 *                the width field is vestigial, and this container only ever
 *                uses 4 anyway.
 *   type 0xfe    pulse the masked bits: *addr = cur & ~mask, then
 *                *addr = cur | mask, where cur is one read taken up front.
 *                The value word is not used.
 *   type 0xff    delay, in MICROSECONDS -- it calls the udelay thunk at
 *                0x4a000d74, and the delay core busy-waits val * 24 arch-timer
 *                ticks at 24 MHz. The separate x1000 mdelay wrapper next to it
 *                is not what the walker uses.
 *
 * The container holds several alternative tables -- a CCU/TVTOP prologue, a
 * set of timing blocks, and seven DE/mixer/LVDS blocks for different modes --
 * and which combination this panel needs is not yet established. So this
 * command applies an explicit byte range rather than trying to pick for you;
 * bisecting on the bench is cheaper than guessing statically.
 *
 * h713_display_prepare() is, register for register, the opening of the first
 * table.
 */
/*
 * Stock imposes no ceiling; this only bounds a walk that has fallen into
 * garbage. The largest delay in the container is 15000 us, so 100 ms of
 * headroom cannot truncate a genuine record.
 */
#define H713_LOGO_MAX_DELAY_US	100000

struct h713_logo_rec {
	u32 addr;
	u32 val;
	u32 mask;
	u32 type;
};

static bool h713_logo_reg_sane(u32 addr)
{
	/* CCU, PIO/MIPS control, and the display/capture blocks only. */
	return (addr >= 0x02000000 && addr < 0x03100000) ||
	       (addr >= 0x04000000 && addr < 0x07000000);
}

static int h713_logo_walk(ulong base, ulong start, ulong end, bool apply)
{
	ulong off;
	int written = 0, skipped = 0, delayed = 0, pulsed = 0;

	/*
	 * Records are 16 bytes but the container only aligns them to 4 -- the
	 * first run starts at 0x17c -- so do not demand 16-byte offsets.
	 */
	if ((start | end) & 3 || end <= start) {
		printf("H713 logo: range must be 4-byte aligned and non-empty\n");
		return -EINVAL;
	}

	/*
	 * Records are 16 bytes but the container inserts other data between
	 * runs, so the stream shifts phase -- the run at 0xbdc is 0x1d8 from
	 * the one at 0xa04, which is not a multiple of 16. Resynchronise by
	 * stepping four bytes until a record parses, then consume sixteen.
	 * Walking a fixed lattice silently drops entries: it applied 167 of
	 * the 292 records in the 0xa04..0x1fdc section.
	 */
	for (off = start; off + sizeof(struct h713_logo_rec) <= end; ) {
		struct h713_logo_rec r;

		r.addr = readl(base + off);
		r.val  = readl(base + off + 4);
		r.mask = readl(base + off + 8);
		r.type = readl(base + off + 12);

		if (!r.addr && r.type == 0xff) {
			off += sizeof(struct h713_logo_rec);
			u32 us = min_t(u32, r.val, H713_LOGO_MAX_DELAY_US);

			if (apply)
				udelay(us);
			else
				printf("  +0x%04lx  delay %u us\n",
				       off - sizeof(struct h713_logo_rec), us);
			delayed++;
			continue;
		}

		if (h713_logo_reg_sane(r.addr) && r.type == 0xfe) {
			u32 cur;

			off += sizeof(struct h713_logo_rec);

			/*
			 * All four in this container pulse bit 31 of the
			 * display PLL at 0x058c0014 -- that is PLL_ENABLE, so
			 * this is a PLL restart, and the one inside timing
			 * block 6 sits between two 15 ms waits in the middle
			 * of a divider reprogram. Dropping it left everything
			 * after it in the block applied to a PLL that had
			 * never been re-locked.
			 */
			if (!apply) {
				printf("  +0x%04lx  0x%08x pulse mask 0x%08x\n",
				       off - sizeof(struct h713_logo_rec),
				       r.addr, r.mask);
				pulsed++;
				continue;
			}

			cur = readl(r.addr);
			writel(cur & ~r.mask, r.addr);
			writel(cur | r.mask, r.addr);
			pulsed++;
			continue;
		}

		if (!h713_logo_reg_sane(r.addr) ||
		    (r.type != 1 && r.type != 2 && r.type != 4)) {
			off += 4;
			skipped++;
			continue;
		}
		off += sizeof(struct h713_logo_rec);

		if (!apply) {
			printf("  +0x%04lx  0x%08x <- 0x%08x mask 0x%08x w%u\n",
			       off - sizeof(struct h713_logo_rec),
			       r.addr, r.val, r.mask, r.type);
			written++;
			continue;
		}

		/* Masked read-modify-write at the record's access width. */
		switch (r.type) {
		case 1:
			writeb((readb(r.addr) & ~(u8)r.mask) |
			       ((u8)r.val & (u8)r.mask), r.addr);
			break;
		case 2:
			writew((readw(r.addr) & ~(u16)r.mask) |
			       ((u16)r.val & (u16)r.mask), r.addr);
			break;
		default:
			writel((readl(r.addr) & ~r.mask) | (r.val & r.mask),
			       r.addr);
			break;
		}
		written++;
	}

	printf("H713 logo: %s %d record(s), %d pulse(s), %d delay(s), "
	       "%d resync step(s)\n", apply ? "applied" : "listed", written,
	       pulsed, delayed, skipped);

	return 0;
}

static int do_h713_logo(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[])
{
	ulong base, start, end;
	bool apply;

	if (argc != 5)
		return CMD_RET_USAGE;

	if (!strcmp(argv[1], "apply"))
		apply = true;
	else if (!strcmp(argv[1], "dump"))
		apply = false;
	else
		return CMD_RET_USAGE;

	base  = hextoul(argv[2], NULL);
	start = hextoul(argv[3], NULL);
	end   = hextoul(argv[4], NULL);

	return h713_logo_walk(base, start, end, apply) ?
	       CMD_RET_FAILURE : CMD_RET_SUCCESS;
}

U_BOOT_CMD(h713_logo, 5, 0, do_h713_logo,
	   "replay a range of the vendor LogoRegData.bin register table",
	   "dump  <blob-addr> <start-off> <end-off>\n"
	   "h713_logo apply <blob-addr> <start-off> <end-off>"
);

/*
 * Bit-banged I2C scan on TWI1's pins.
 *
 * The projector's output chain ends in a TI DLPC3435 DLP controller, and the
 * stock ge2d driver reaches it over I2C at 0x1b (its normal_i2c probe list).
 * TWI1 is the only I2C bus the vendor device tree enables: 0x02502400 at
 * 100 kHz on PH2/PH3, and it also carries an lsm6dsr accelerometer at 0x6a,
 * which makes a useful positive control -- seeing 0x6a proves the bus works.
 *
 * Bit-banging rather than bringing up mvtwsi keeps this self-contained: no
 * device-tree node, no CCU gate, nothing that can be silently wrong.
 */
/*
 * H713 uses CONFIG_SUNXI_NEW_PINCTRL, so banks are 0x30 apart and the pull
 * registers sit at +0x24 -- not the 0x24 stride and +0x1c pull of the older
 * layout. Take the stride from the header rather than restating it.
 */
#define H713_PIO_BASE		0x02000000UL
#define H713_PIO_BANK_B		1
#define H713_PIO_BANK_F		5
#define H713_PIO_BANK_H		7
#define H713_PB_CFG0		(H713_PIO_BASE + \
				 H713_PIO_BANK_B * SUNXI_PINCTRL_BANK_SIZE)
#define H713_PB_DATA		(H713_PB_CFG0 + 0x10)
#define H713_PF_CFG0		(H713_PIO_BASE + \
				 H713_PIO_BANK_F * SUNXI_PINCTRL_BANK_SIZE)
#define H713_PF_DATA		(H713_PF_CFG0 + 0x10)
#define H713_PH_CFG0		(H713_PIO_BASE + \
				 H713_PIO_BANK_H * SUNXI_PINCTRL_BANK_SIZE)
#define H713_PH_DATA		(H713_PH_CFG0 + 0x10)
#define H713_PH_PULL0		(H713_PH_CFG0 + 0x24)

#define H713_I2C_DELAY_US	5		/* ~100 kHz */

/* Selectable so a wrong guess costs a retype, not a rebuild. */
static uint h713_i2c_scl = 2;
static uint h713_i2c_sda = 3;
#define H713_I2C_SCL_PIN	h713_i2c_scl
#define H713_I2C_SDA_PIN	h713_i2c_sda

/* Drive low by becoming an output; release to high-Z and let the pull-up win. */
static void h713_i2c_set(uint pin, bool high)
{
	u32 cfg = readl(H713_PH_CFG0) & ~(0xfu << (pin * 4));

	if (high) {
		writel(cfg, H713_PH_CFG0);		/* input: high-Z */
	} else {
		clrbits_le32((void *)H713_PH_DATA, BIT(pin));
		writel(cfg | (1u << (pin * 4)), H713_PH_CFG0);	/* output low */
	}
	udelay(H713_I2C_DELAY_US);
}

static int h713_i2c_get_sda(void)
{
	return !!(readl(H713_PH_DATA) & BIT(H713_I2C_SDA_PIN));
}

static void h713_i2c_start(void)
{
	h713_i2c_set(H713_I2C_SDA_PIN, true);
	h713_i2c_set(H713_I2C_SCL_PIN, true);
	h713_i2c_set(H713_I2C_SDA_PIN, false);
	h713_i2c_set(H713_I2C_SCL_PIN, false);
}

static void h713_i2c_stop(void)
{
	h713_i2c_set(H713_I2C_SDA_PIN, false);
	h713_i2c_set(H713_I2C_SCL_PIN, true);
	h713_i2c_set(H713_I2C_SDA_PIN, true);
}

/* Returns true when the slave pulled SDA low for ACK. */
static bool h713_i2c_write_byte(u8 byte)
{
	bool ack;
	int i;

	for (i = 7; i >= 0; i--) {
		h713_i2c_set(H713_I2C_SDA_PIN, !!(byte & BIT(i)));
		h713_i2c_set(H713_I2C_SCL_PIN, true);
		h713_i2c_set(H713_I2C_SCL_PIN, false);
	}

	h713_i2c_set(H713_I2C_SDA_PIN, true);
	h713_i2c_set(H713_I2C_SCL_PIN, true);
	ack = !h713_i2c_get_sda();
	h713_i2c_set(H713_I2C_SCL_PIN, false);

	return ack;
}

static u8 h713_i2c_read_byte(bool ack)
{
	u8 v = 0;
	int i;

	h713_i2c_set(H713_I2C_SDA_PIN, true);		/* release for the slave */

	for (i = 7; i >= 0; i--) {
		h713_i2c_set(H713_I2C_SCL_PIN, true);
		if (h713_i2c_get_sda())
			v |= BIT(i);
		h713_i2c_set(H713_I2C_SCL_PIN, false);
	}

	/* ACK to continue, NACK to end the transfer. */
	h713_i2c_set(H713_I2C_SDA_PIN, !ack);
	h713_i2c_set(H713_I2C_SCL_PIN, true);
	h713_i2c_set(H713_I2C_SCL_PIN, false);
	h713_i2c_set(H713_I2C_SDA_PIN, true);

	return v;
}

/*
 * Plain read with no preceding register write: identifying an unknown chip
 * must not risk changing its state.
 */
static int h713_i2c_dump(uint addr, uint count)
{
	uint i;

	h713_i2c_start();
	if (!h713_i2c_write_byte((u8)((addr << 1) | 1))) {
		h713_i2c_stop();
		printf("H713 i2c: 0x%02x did not ACK its read address\n", addr);
		return -EIO;
	}

	printf("H713 i2c: 0x%02x read:", addr);
	for (i = 0; i < count; i++)
		printf(" %02x", h713_i2c_read_byte(i + 1 < count));
	printf("\n");

	h713_i2c_stop();
	return 0;
}

static int do_h713_i2c(struct cmd_tbl *cmdtp, int flag, int argc,
		       char *const argv[])
{
	int addr, found = 0;

	/* Enable the internal pull-ups the vendor pinmux asks for. */
	clrsetbits_le32((void *)H713_PH_PULL0,
			(3u << (H713_I2C_SCL_PIN * 2)) |
			(3u << (H713_I2C_SDA_PIN * 2)),
			(1u << (H713_I2C_SCL_PIN * 2)) |
			(1u << (H713_I2C_SDA_PIN * 2)));

	h713_i2c_set(H713_I2C_SCL_PIN, true);
	h713_i2c_set(H713_I2C_SDA_PIN, true);

	if (argc == 4 && !strcmp(argv[1], "read")) {
		uint a = hextoul(argv[2], NULL);
		uint n = dectoul(argv[3], NULL);

		if (a > 0x7f || !n || n > 32) {
			printf("H713 i2c: address 0..0x7f, count 1..32\n");
			return CMD_RET_FAILURE;
		}
		return h713_i2c_dump(a, n) ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	if (argc != 2 && argc != 4)
		return CMD_RET_USAGE;
	if (strcmp(argv[1], "scan"))
		return CMD_RET_USAGE;

	if (argc == 4) {
		h713_i2c_scl = dectoul(argv[2], NULL);
		h713_i2c_sda = dectoul(argv[3], NULL);
		if (h713_i2c_scl > 31 || h713_i2c_sda > 31 ||
		    h713_i2c_scl == h713_i2c_sda) {
			printf("H713 i2c: bad PH pin numbers\n");
			return CMD_RET_FAILURE;
		}
	}


	/*
	 * With SDA stuck low every ACK read returns zero and the scan reports
	 * a device at every address. Refuse to run rather than print 112 lies.
	 */
	if (!h713_i2c_get_sda()) {
		printf("H713 i2c: SDA (PH%d) reads low with the bus idle -- "
		       "wrong pins, no pull-up, or the line is held\n",
		       H713_I2C_SDA_PIN);
		return CMD_RET_FAILURE;
	}

	printf("H713 i2c: scanning PH%d/PH%d\n",
	       H713_I2C_SCL_PIN, H713_I2C_SDA_PIN);

	for (addr = 0x08; addr < 0x78; addr++) {
		bool ack;

		h713_i2c_start();
		ack = h713_i2c_write_byte((u8)(addr << 1));
		h713_i2c_stop();

		if (ack) {
			printf("  0x%02x ACK%s\n", addr,
			       addr == 0x1b ? "   <-- DLPC3435" :
			       addr == 0x6a ? "   <-- lsm6dsr (bus works)" : "");
			found++;
		}
	}

	printf("H713 i2c: %d device(s) responded\n", found);

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(h713_i2c, 4, 0, do_h713_i2c,
	   "bit-banged I2C scan on a pair of PH pins",
	   "scan            - scan using PH2/PH3 (vendor twi1 pins)\n"
	   "h713_i2c scan <scl> <sda> - scan using the given PH pin numbers\n"
	   "h713_i2c read <addr> <count> - read bytes, no register write"
);

/*
 * One-shot replay of stock U-Boot's fastlogo display bring-up.
 *
 * LogoRegData.bin is indexed: 13 descriptors of 0x18 bytes from offset 0x10,
 * each starting with a project ID that matches the ProjectID_*.TSE names. Two
 * of its words select which tables that project uses --
 *
 *   word3 & 0xffff  prologue variant, 1-based (three exist)
 *   word3 >> 16     timing variant,   0-based (eleven exist)
 *   word4 & 0xffff  DE/mixer variant, 0-based (seven exist)
 *
 * -- so a working configuration is a *consistent triple*, not three ranges
 * picked by eye. Doing it by hand produced combinations no project uses.
 *
 * Stock's order is: prologue, timing, LVDS FIFO reset, mixer write, DE table,
 * then clocks/INCAP/LVDS, the coprocessor release, and the LVDS finalise.
 */
struct h713_disp_block { u32 start, end; };

static const struct h713_disp_block h713_disp_prologue[] = {
	{ 0x01ac, 0x0484 }, { 0x0484, 0x075c }, { 0x075c, 0x0a34 },
};

static const struct h713_disp_block h713_disp_timing[] = {
	{ 0x0a34, 0x0c0c }, { 0x0c0c, 0x0de4 }, { 0x0de4, 0x100c },
	{ 0x100c, 0x11e4 }, { 0x11e4, 0x141c }, { 0x141c, 0x1654 },
	{ 0x1654, 0x18dc }, { 0x18dc, 0x1a04 }, { 0x1a04, 0x1c3c },
	{ 0x1c3c, 0x1ec4 }, { 0x1ec4, 0x214c },
};

static const struct h713_disp_block h713_disp_de[] = {
	{ 0x214c, 0x24c4 }, { 0x24c4, 0x283c }, { 0x283c, 0x2bb4 },
	{ 0x2bb4, 0x2f2c }, { 0x2f2c, 0x32a4 }, { 0x32a4, 0x361c },
	{ 0x361c, 0x39a4 }, { 0x39a4, 0x3d24 },
};

#define H713_DISP_DESC_OFF	0x10
#define H713_DISP_DESC_SIZE	0x18
#define H713_DISP_HDR_TABLE_LEN	8

struct h713_disp_sel { u32 project, prologue, timing, de; };

/*
 * Descriptor count is in the header, not fixed: this board's file carries 15
 * where an earlier revision had 13. Reading it keeps the command correct
 * across firmware versions.
 */
static uint h713_disp_desc_count(ulong blob)
{
	uint bytes = readw(blob + H713_DISP_HDR_TABLE_LEN);

	return bytes / H713_DISP_DESC_SIZE;
}

/* Set once a sequence has run, so the dump knows the blocks are clocked. */
static bool h713_disp_configured;

static int h713_disp_lookup(ulong blob, u32 project,
			    struct h713_disp_sel *sel)
{
	int i;

	for (i = 0; i < h713_disp_desc_count(blob); i++) {
		ulong d = blob + H713_DISP_DESC_OFF + i * H713_DISP_DESC_SIZE;
		u32 id = readl(d);
		u32 w3, w4;

		if (id != project)
			continue;

		w3 = readl(d + 8);
		w4 = readl(d + 12);
		sel->project  = id;
		sel->prologue = w3 & 0xffff;
		sel->timing   = w3 >> 16;
		sel->de       = w4 & 0xffff;

		if (!sel->prologue ||
		    sel->prologue > ARRAY_SIZE(h713_disp_prologue) ||
		    sel->timing >= ARRAY_SIZE(h713_disp_timing) ||
		    sel->de >= ARRAY_SIZE(h713_disp_de)) {
			printf("H713 disp: project 0x%02x selects out-of-range "
			       "tables (%u/%u/%u)\n", id, sel->prologue,
			       sel->timing, sel->de);
			return -EINVAL;
		}
		return 0;
	}

	printf("H713 disp: no descriptor for project 0x%02x\n", project);
	return -ENOENT;
}

static void h713_disp_list(ulong blob)
{
	int i;

	printf("H713 disp: project  prologue  timing  de\n");
	for (i = 0; i < h713_disp_desc_count(blob); i++) {
		ulong d = blob + H713_DISP_DESC_OFF + i * H713_DISP_DESC_SIZE;
		u32 w3 = readl(d + 8);

		printf("             0x%02x       %u       %2u   %u\n",
		       readl(d), w3 & 0xffff, w3 >> 16, readl(d + 12) & 0xffff);
	}
}

/* Clocks, capture block and INCAP, as stock issues them before LVDS. */
static void h713_disp_clocks(void)
{
	writel(0x22ffff22, 0x02000150);		/* PH mux; PH0/1 stay UART0 */
	mdelay(12);
	writel(0x00010001, 0x02001d88);
	mdelay(12);
	/*
	 * 0x02001020 is deliberately absent. Stock writes PLL_PERIPH0 there,
	 * but it is a no-op in stock's boot context and a live reconfiguration
	 * in ours -- MMC and the buses run from it. The other two PLLs are
	 * disabled in our cold state, so enabling them is safe.
	 */
	writel(0xb8003501, 0x02001040);
	writel(0x80000305, 0x02001d6c);
	mdelay(12);
	writel(0xb8002f01, 0x02001068);
	writel(0x81000001, 0x02001d74);
	mdelay(12);
	writel(0x80000000, 0x02001d84);
	mdelay(12);
	writel(0xc0000000, 0x02001d80);
	mdelay(12);

	writel(0x01111117, 0x06e00004);
	writel(0x00000404, 0x06e00008);
	writel(0x00111111, 0x06e00000);
	mdelay(12);
	writel(0, 0x06e00004);
	writel(0, 0x06e00008);
	writel(0, 0x06e00000);
	mdelay(12);
	writel(0x01111117, 0x06e00004);
	writel(0x00000404, 0x06e00008);
	writel(0x00111111, 0x06e00000);
	mdelay(12);

	/*
	 * No module-clock writes here on purpose. h713_display_prepare() sets
	 * PLL_VIDEO2 and the deint/panel/SVP-DTL/AFBD gates for the
	 * h713_mips start path, but the vendor prologue block already does the
	 * same work on this one -- prologue 3 writes 0x02001dc0 <- 0x80000005
	 * and enables PLL_VIDEO2 across four masked records. Calling the
	 * helper here would be redundant, not protective; the clock state is
	 * reported in the post-readiness dump instead, which is the only place
	 * it could still have changed.
	 */
}

/* Stock's reset: pulse bit 8 of the FIFO control, then re-latch the config. */
static void h713_disp_fifo_reset(void)
{
	u32 status = readl(0x05880fe0);
	u32 cfg;
	u32 ctl;

	if (!status) {
		printf("H713 disp: FIFO status clear; reset skipped\n");
		return;
	}

	cfg = readl(0x0588000c);
	ctl = readl(0x05700088);
	writel(ctl & ~0x100, 0x05700088);
	writel(ctl | 0x100, 0x05700088);
	writel(cfg, 0x0588000c);
	printf("H713 disp: FIFO status 0x%08x; reset applied\n", status);
}

/*
 * Stock fastlogo applies all LogoRegData groups, waits poweron_delay1
 * (550 ms), then invokes its panel-power method. The board-B runtime TOC1 DT
 * identifies panel_power_en as PF6, not the PH19 found in the older board-A
 * dump. It identifies panel_gpio_0 as PH16. Enable PF6 and pulse PH16 low for
 * 2 ms, then high for 5 ms. The state
 * machine waits poweron_delay0 (20 ms) before continuing into the display
 * clocks and MIPS release.
 *
 * Both GPIOs carry GPIO_PULL_DOWN in the stock DT. Set the data latch before
 * selecting output mode so the power line cannot glitch low as it is enabled.
 *
 * Do not use the legacy gpio_set_value() compatibility wrapper here. This
 * build uses DM_GPIO, whose compatibility wrapper reconstructs a descriptor
 * without its previous output-direction flags; the second value change then
 * does not reach this driver's set_flags() output path. Accessing the PF/PH
 * data latches directly also lets the diagnostic verify the exact registers.
 */
static int h713_disp_stock_panel_power(void)
{
	const uint power = SUNXI_GPF(6);
	const uint reset = SUNXI_GPH(16);
	u32 pf_dat;
	u32 ph_dat;

	printf("H713 panel: stock power pre-delay 550 ms\n");
	mdelay(550);

	sunxi_gpio_set_pull(power, SUNXI_GPIO_PULL_DOWN);
	sunxi_gpio_set_pull(reset, SUNXI_GPIO_PULL_DOWN);

	setbits_le32((void *)H713_PF_DATA, BIT(6));
	sunxi_gpio_set_cfgpin(power, SUNXI_GPIO_OUTPUT);

	clrbits_le32((void *)H713_PH_DATA, BIT(16));
	sunxi_gpio_set_cfgpin(reset, SUNXI_GPIO_OUTPUT);
	mdelay(2);
	setbits_le32((void *)H713_PH_DATA, BIT(16));
	mdelay(5);

	pf_dat = readl(H713_PF_DATA);
	ph_dat = readl(H713_PH_DATA);
	printf("H713 panel: stock GPIO phase complete: "
	       "PF6 cfg=%x latch=%d, PH16 cfg=%x latch=%d, "
	       "PF_DAT=%08x PH_DAT=%08x\n",
	       sunxi_gpio_get_cfgpin(power), !!(pf_dat & BIT(6)),
	       sunxi_gpio_get_cfgpin(reset), !!(ph_dat & BIT(16)),
	       pf_dat, ph_dat);
	mdelay(20);

	return 0;
}

/*
 * Board B's panel description, as stock assembles it.
 *
 * Stock parses its runtime DT into a flat 35-entry u32 array, then overwrites
 * the same array from /panel_config.ini -- the two name tables at stock
 * 0x4a05ae30 (DT) and 0x4a05a4fc (INI) are index-for-index parallel, which is
 * what makes the override work. The values below are that merged result:
 * panel_config.ini (Reserve0_a, sha256 7bffff88..., extracted to
 * local/mips-display/board-b-mips/panel_config.ini) over the runtime TOC1 DT.
 *
 * Only two fields actually differ between the two sources -- dual_port and
 * ssc_en -- and both are recorded here at their post-INI value.
 *
 * PanelLvds0Pol/PanelLvds1Pol are absent from *both* sources, so stock leaves
 * their array slots untouched and its patch helper skips them. They are
 * deliberately not modelled here.
 */
struct h713_panel_cfg {
	u32 mapping;		/* DT panel_protocol      */
	u32 odd_even;		/* DT panel_data_swap     */
	u32 dual_port;		/* DT 1 -> INI 0          */
	u32 mirror_mode;
	u32 inv_de, inv_hsync, inv_vsync, inv_dclk;
	u32 de_current, odd_current, even_current;
	u32 ssc_en;		/* DT 1 -> INI 0          */
	u32 htotal, vtotal, hsync, vsync, hbp, vbp, width;
};

static const struct h713_panel_cfg h713_panel_cfg_board_b = {
	.mapping = 0, .odd_even = 0, .dual_port = 0, .mirror_mode = 0,
	.inv_de = 0, .inv_hsync = 0, .inv_vsync = 0, .inv_dclk = 1,
	.de_current = 47, .odd_current = 7, .even_current = 7,
	.ssc_en = 0,
	.htotal = 1360, .vtotal = 760, .hsync = 20, .vsync = 2,
	.hbp = 40, .vbp = 20, .width = 1280,
};

struct h713_panel_patch {
	u32 reg;
	u8  shift;
	u32 fieldmask;
	u32 value;
};

/*
 * The vendor tables carry another panel's defaults; stock rewrites these
 * fields from the merged config before its applier touches hardware. Sites
 * and bit positions are transcribed from stock U-Boot's patch function at
 * 0x4a0248fc, whose helper at 0x4a024894 is a plain bitfield insert:
 *
 *     record.value = (record.value & ~(fieldmask << shift))
 *                  | ((value & fieldmask) << shift)
 *
 * guarded by (fieldmask << shift) being a subset of the record's own mask.
 *
 * Two sites are deliberately omitted. Stock's 0x05280084[31:16] and
 * 0x0528008c[15:0] both resolve to a literal zero in static analysis, which
 * may be a decode artefact rather than a real store; writing a zero we cannot
 * justify is worse than leaving the vendor default in place.
 */
static int h713_disp_panel_patch(ulong blob, const struct h713_disp_sel *sel)
{
	const struct h713_panel_cfg *c = &h713_panel_cfg_board_b;
	const struct h713_panel_patch tbl[] = {
		/* LVDS lane/map: two protocol fields plus the invert flags */
		{ 0x05800000,  6, 0x3,    c->mapping },
		{ 0x05800000,  3, 0x3,    c->mapping },
		{ 0x05800000, 14, 0x1,    c->odd_even },
		{ 0x05800000, 16, 0x1,    c->inv_hsync },
		{ 0x05800000, 17, 0x1,    c->inv_vsync },
		{ 0x05800000, 18, 0x1,    c->inv_de },
		{ 0x05800000, 24, 0x1,    c->inv_dclk },
		/* display PLL: LVDS drive currents and spread spectrum */
		{ 0x058c0020, 24, 0x3f,   c->de_current },
		{ 0x058c0020,  0, 0x7,    c->odd_current },
		{ 0x058c0024, 24, 0x3f,   c->de_current },
		{ 0x058c0024,  0, 0x7,    c->even_current },
		{ 0x058c0014, 24, 0x1,    c->ssc_en },
		/* TCON control: single/dual port */
		{ 0x0588000c, 12, 0x3,    c->dual_port },
		/* AFBD fetch: mirror mode */
		{ 0x05600140,  2, 0x3,    c->mirror_mode },
		/* mixer geometry and blanking */
		{ 0x0525c000, 16, 0xffff, c->vtotal },
		{ 0x0525c000,  0, 0xffff, c->htotal },
		{ 0x0525c004,  8, 0xff,   c->hsync },
		{ 0x0525c004,  0, 0xff,   c->vsync },
		{ 0x0525c01c,  0, 0xffff, c->hsync + c->hbp },
		{ 0x0525c020,  0, 0xffff, c->vsync + c->vbp },
		{ 0x0525c030,  0, 0xffff, c->vsync + c->vbp },
		{ 0x0525c034, 16, 0xffff, c->width },
		{ 0x0525c034,  0, 0xffff, c->hsync + c->hbp },
		/*
		 * Stock's compare at +0x24e12/+0x24e26 matches *either*
		 * 0x0524c010 or 0x0525c000 and patches whichever record it
		 * found, so the DE's copy of the geometry takes the same
		 * value as the mixer's. Omitting it left the DE composing
		 * 1440x741 beneath a mixer at 1360x760 -- visible in a live
		 * dump as de +0x10 holding the unpatched 0x02e4059f.
		 */
		{ 0x0524c010, 16, 0xffff, c->vtotal },
		{ 0x0524c010,  0, 0xffff, c->htotal },
		/* display engine */
		{ 0x0524c004, 16, 0xffff, c->width },
		{ 0x0524c004,  0, 0xffff, c->vsync + c->vbp },
		{ 0x0524c014,  8, 0xff,   c->vsync },
		{ 0x05280084,  0, 0xffff, c->width },
		{ 0x05280088,  0, 0xffff, c->vsync + c->vbp },
	};
	const struct h713_disp_block *ranges[] = {
		&h713_disp_prologue[sel->prologue - 1],
		&h713_disp_timing[sel->timing],
		&h713_disp_de[sel->de],
	};
	int patched = 0, guarded = 0;
	uint i, r;

	for (r = 0; r < ARRAY_SIZE(ranges); r++) {
		ulong off;

		for (off = ranges[r]->start;
		     off + sizeof(struct h713_logo_rec) <= ranges[r]->end; ) {
			struct h713_logo_rec rec;

			rec.addr = readl(blob + off);
			rec.val  = readl(blob + off + 4);
			rec.mask = readl(blob + off + 8);
			rec.type = readl(blob + off + 12);

			/*
			 * Record validity and the 4-byte resync step must
			 * mirror h713_logo_walk() exactly. If this pass
			 * framed the stream differently it would rewrite
			 * bytes the walker then reads as a different record.
			 */
			bool is_delay = !rec.addr && rec.type == 0xff;
			bool is_pulse = h713_logo_reg_sane(rec.addr) &&
					rec.type == 0xfe;
			bool is_write = h713_logo_reg_sane(rec.addr) &&
					(rec.type == 1 || rec.type == 2 ||
					 rec.type == 4);

			if (!is_delay && !is_pulse && !is_write) {
				off += 4;
				continue;
			}

			for (i = 0; is_write && i < ARRAY_SIZE(tbl); i++) {
				u32 window, updated;

				if (tbl[i].reg != rec.addr)
					continue;

				window = tbl[i].fieldmask << tbl[i].shift;
				if (window & ~rec.mask) {
					guarded++;
					continue;
				}

				updated = (rec.val & ~window) |
					  ((tbl[i].value & tbl[i].fieldmask)
					   << tbl[i].shift);
				if (updated == rec.val)
					continue;

				printf("  +0x%04lx  %08x  %08x -> %08x  "
				       "shift %u mask 0x%x\n", off, rec.addr,
				       rec.val, updated, tbl[i].shift,
				       tbl[i].fieldmask);
				writel(updated, blob + off + 4);
				rec.val = updated;
				patched++;
			}

			off += sizeof(struct h713_logo_rec);
		}
	}

	printf("H713 panel: config applied, %d record field(s) patched, "
	       "%d guarded by record mask\n", patched, guarded);
	return 0;
}

static int h713_disp_run(ulong blob, u32 project, bool skip_hdcp_wait,
			 bool prove_ready, bool trace, bool stability,
			 bool stock_panel_power, bool release_mips)
{
	struct h713_disp_sel sel;
	int ret;

	ret = h713_disp_lookup(blob, project, &sel);
	if (ret)
		return ret;

	printf("H713 disp: project 0x%02x -> prologue %u, timing %u, de %u\n",
	       sel.project, sel.prologue, sel.timing, sel.de);

	ret = h713_disp_panel_patch(blob, &sel);
	if (ret)
		return ret;

	ret = h713_logo_walk(blob, h713_disp_prologue[sel.prologue - 1].start,
			     h713_disp_prologue[sel.prologue - 1].end, true);
	if (ret)
		return ret;
	ret = h713_logo_walk(blob, h713_disp_timing[sel.timing].start,
			     h713_disp_timing[sel.timing].end, true);
	if (ret)
		return ret;

	h713_disp_fifo_reset();
	writel(H713_DISPLAY_MIXER_CTRL_VALUE, H713_DISPLAY_MIXER_CTRL_REG);

	ret = h713_logo_walk(blob, h713_disp_de[sel.de].start,
			     h713_disp_de[sel.de].end, true);
	if (ret)
		return ret;

	if (stock_panel_power) {
		ret = h713_disp_stock_panel_power();
		if (ret)
			return ret;
	}

	h713_disp_clocks();
	if (stock_panel_power)
		printf("H713 panel: post-stock GPIO mux: "
		       "PF6 cfg=%x latch=%d, PH16 cfg=%x latch=%d\n",
		       sunxi_gpio_get_cfgpin(SUNXI_GPF(6)),
		       !!(readl(H713_PF_DATA) & BIT(6)),
		       sunxi_gpio_get_cfgpin(SUNXI_GPH(16)),
		       !!(readl(H713_PH_DATA) & BIT(16)));

	writel(1, 0x06940000);			/* INCAP */
	mdelay(12);
	writel(0x01800045, 0x051c0010);		/* LVDS enable */
	mdelay(12);

	/*
	 * Everything the display path needs -- TCON timing, TVTOP routing, the
	 * display PLL, mixer, DE, AFBD, panel power, INCAP and LVDS -- comes
	 * from the vendor tables and the ARM sequence above. The coprocessor
	 * programs none of it; what it demonstrably does do is overwrite the
	 * TCON timing with 1080p once it runs.
	 *
	 * So holding it in reset is a real experiment, not a degraded run: if
	 * pixels appear without it, the firmware is what suppresses them and
	 * the fault is composition ownership rather than our register
	 * programming. It also leaves the panel on the tables' native 720p and
	 * lifts the one-launch-per-power-cycle rule, since no launch happens.
	 */
	if (release_mips) {
		ret = h713_mips_release_raw(skip_hdcp_wait, prove_ready, trace,
					    stability);
		if (ret)
			return ret;
	} else {
		printf("H713 disp: MIPS held in reset (noboot); display driven "
		       "by the ARM sequence alone\n");
	}

	writel(0x45, 0x051c0010);		/* LVDS finalise */

	/*
	 * The one register group stock's fastlogo writes and this replay never
	 * did. Enumerating every MMIO literal in stock's fastlogo function
	 * (0x4a0228d4) and diffing against our sequence leaves exactly one
	 * omission: 0x051c00d4..0x051c00e0, written as a barriered group at
	 * raw +0x22cca.
	 *
	 * Stock has two branches for it, selected on a config field at +0xf8.
	 * When that field is unset it writes the constants below; otherwise it
	 * read-modify-writes the same registers with values derived from
	 * config +0xec/+0xf0. The fields sit past the 35-entry panel array and
	 * neither the DT nor panel_config.ini supplies them, so the constant
	 * path is the one this board takes. Either way stock writes this
	 * group and we did not.
	 *
	 * These live in the LVDS PHY block alongside the firmware's
	 * hardware-blue-screen source at +0xb0/+0xb4/+0xb8 -- which does reach
	 * the panel -- so this is the right neighbourhood for a pixel path
	 * that is configured, clocked, enabled and still delivering nothing.
	 */
	dmb();
	writel(0, 0x051c00d4);
	dmb();
	writel(0, 0x051c00d8);
	dmb();
	writel(0x08000800, 0x051c00dc);
	dmb();
	writel(0x08000000, 0x051c00e0);
	dmb();
	printf("H713 disp: LVDS PHY tail applied: %08x %08x %08x %08x\n",
	       readl(0x051c00d4), readl(0x051c00d8),
	       readl(0x051c00dc), readl(0x051c00e0));
	h713_disp_configured = true;
	printf("H713 disp: sequence complete, LVDS FIFO status=0x%08x\n",
	       readl(0x05880fe0));

	/*
	 * Readiness is a property of a released coprocessor. With the MIPS
	 * held in reset there is nothing to wait for -- CPU_COMM was never
	 * published, so the magic words read as uninitialised DRAM -- and
	 * failing the run on that would abort the very test the flag exists
	 * to perform.
	 */
	if (prove_ready && release_mips) {
		ret = h713_mips_wait_ready(H713_MIPS_DISP_READY_TIMEOUT_US,
					   trace);
		if (ret)
			return ret;
		if (stability)
			return h713_mips_monitor_stability();
	}

	return 0;
}

static const struct { ulong base; uint words; const char *name; } h713_disp_regs[] = {
	{ 0x05700000, 16, "tvtop"  }, { 0x05800000, 12, "lvds-lane" },
	{ 0x05880000, 16, "lvds"   }, { 0x058c0000, 12, "disp-pll"  },
	{ 0x051c0000,  8, "lvds-phy" }, { 0x0525c000, 16, "mixer"   },
	/*
	 * The firmware's blue-screen source lives at +0xb0/+0xb4/+0xb8 and the
	 * group stock writes at the end of fastlogo at +0xd4..+0xe0. Neither
	 * was ever dumped.
	 */
	{ 0x051c00b0, 16, "lvds-phy2" },
	{ 0x0524c000, 32, "de"     }, { 0x05600140, 16, "afbd"      },
	/*
	 * DE block 5 writes 0x80000020 to 0x05600000, AFBD's top-level
	 * control, but nothing has ever read it back. If the fetch unit is
	 * globally disabled, that is where it shows.
	 */
	{ 0x05600000,  4, "afbd-top" },
	/*
	 * The vendor prologue enables PLL_VIDEO2 and the display module
	 * clocks, so they are not gated going in. Read them back after the
	 * coprocessor has run: it is the one actor that could since have
	 * changed them, and an unclocked fetch datapath behind a clocked
	 * register file would look exactly like the blank screen we have.
	 */
	{ 0x02001050,  4, "pll-video2" },
	{ 0x02001db0,  8, "disp-modclk" },
};

/*
 * Dump the blocks the vendor tables write. A register that does not hold what
 * was written to it means its block is gated or absent, which is far more
 * useful than guessing at semantics.
 */
static void h713_disp_dump(bool force)
{
	int i;

	/*
	 * These blocks are gated until the sequence runs; reading them cold
	 * stalls the interconnect and hangs the board. Refuse rather than
	 * wedge, unless the caller insists.
	 */
	if (!h713_disp_configured && !force) {
		printf("H713 disp: display not configured this boot -- reading "
		       "these blocks now would hang.\n"
		       "           run the sequence first, or 'dump force'\n");
		return;
	}

	for (i = 0; i < ARRAY_SIZE(h713_disp_regs); i++) {
		uint w;

		printf("%s @0x%08lx:", h713_disp_regs[i].name,
		       h713_disp_regs[i].base);
		for (w = 0; w < h713_disp_regs[i].words; w++) {
			if (!(w % 8))
				printf("\n  +0x%02x:", w * 4);
			printf(" %08x", readl(h713_disp_regs[i].base + w * 4));
		}
		printf("\n");
	}
}


/*
 * Load the vendor display artifacts straight off the board's own eMMC.
 *
 * They all live in the stock FAT bootloader partition, which is where stock
 * U-Boot reads them from -- the "bootloader" partition lookup in its fastlogo
 * path does exactly this. Reading them here removes the fastboot staging dance
 * entirely, survives reboots, and keeps proprietary blobs out of the U-Boot
 * image, which the project's rules require.
 */
#define H713_DISP_FS_IF		"mmc"
#define H713_DISP_FS_DEV	"1:2"
/*
 * Above the framebuffer window (which ends at 0x4d941000) and below the
 * CPU_COMM share region at 0x4e300000. Parking it inside the framebuffer
 * means h713_mips_clear_workspace() erases it the moment it is loaded.
 */
#define H713_DISP_LOGO_ADDR	0x4e000000UL

static int h713_disp_read(const char *path, ulong addr, loff_t *len)
{
	int ret;

	ret = fs_set_blk_dev(H713_DISP_FS_IF, H713_DISP_FS_DEV, FS_TYPE_ANY);
	if (ret) {
		printf("H713 disp: cannot select %s %s\n",
		       H713_DISP_FS_IF, H713_DISP_FS_DEV);
		return ret;
	}

	ret = fs_read(path, addr, 0, 0, len);
	if (ret) {
		printf("H713 disp: cannot read %s\n", path);
		return ret;
	}

	printf("  %-28s -> 0x%08lx  %llu bytes\n", path, addr, *len);
	return 0;
}

/* The TSE window takes the vendor databases concatenated, project file last. */
static int h713_disp_load_tse(u32 project)
{
	static const char *const fixed[] = {
		"mips/database.TSE", "mips/projecttable.TSE",
	};
	char pid[40];
	ulong addr = H713_MIPS_TSE_ADDR;
	loff_t len;
	int i, ret;

	memset((void *)H713_MIPS_TSE_ADDR, 0, H713_MIPS_TSE_SIZE);

	for (i = 0; i < ARRAY_SIZE(fixed); i++) {
		ret = h713_disp_read(fixed[i], addr, &len);
		if (ret)
			return ret;
		addr += len;
	}

	snprintf(pid, sizeof(pid), "mips/ProjectID_0x%04x.TSE", project);
	ret = h713_disp_read(pid, addr, &len);
	if (ret)
		return ret;
	addr += len;

	ret = h713_disp_read("mips/pq_custom.TSE", addr, &len);
	if (ret)
		return ret;
	addr += len;

	if (addr > H713_MIPS_TSE_ADDR + H713_MIPS_TSE_SIZE) {
		printf("H713 disp: TSE data overruns its window\n");
		return -ENOSPC;
	}

	/*
	 * The MIPS reads this through its own uncached mapping. Filesystem reads
	 * populate ARM cacheable DRAM, so publish both the payload and zero tail
	 * before reset release instead of depending on incidental eviction.
	 */
	flush_cache(H713_MIPS_TSE_ADDR, H713_MIPS_TSE_SIZE);

	return 0;
}

static int h713_disp_load(u32 project)
{
	loff_t len;
	int ret;

	printf("H713 disp: loading vendor artifacts from %s %s\n",
	       H713_DISP_FS_IF, H713_DISP_FS_DEV);

	/* Clear first: the workspace wipe must not run over what we load. */
	h713_mips_clear_workspace();
	memset((void *)H713_MIPS_CFG_ADDR, 0, H713_MIPS_CFG_SIZE);

	ret = h713_disp_read("mips/display.bin", H713_MIPS_FW_ADDR, &len);
	if (ret)
		return ret;
	if (len != H713_MIPS_FW_SIZE) {
		printf("H713 disp: display.bin is %llu bytes, expected 0x%lx\n",
		       len, H713_MIPS_FW_SIZE);
		return -EINVAL;
	}

	ret = h713_disp_read("mips/display_cfg.xml", H713_MIPS_CFG_ADDR, &len);
	if (ret)
		return ret;
	if (len > H713_MIPS_CFG_SIZE) {
		printf("H713 disp: display_cfg.xml overruns its window\n");
		return -ENOSPC;
	}
	/*
	 * The firmware's early sys:* lookups run before its scheduler. Make the
	 * XML and its zero-filled terminator visible to the non-coherent MIPS.
	 */
	flush_cache(H713_MIPS_CFG_ADDR, H713_MIPS_CFG_SIZE);

	ret = h713_disp_load_tse(project);
	if (ret)
		return ret;
	printf("H713 disp: config/TSE windows published for MIPS\n");

	ret = h713_disp_read("mips/LogoRegData.bin", H713_DISP_LOGO_ADDR, &len);
	if (ret)
		return ret;

	return 0;
}


/*
 * Single-command test run.
 *
 * The firmware wedges the interconnect some seconds after the sequence
 * completes, so anything typed by hand afterwards races that failure and the
 * result depends on how fast the operator types. Do the whole experiment --
 * load, config patches, sequence, timed sampling, log dump -- inside one
 * command so the timing is fixed.
 *
 * display_cfg.xml offsets are byte positions of single ASCII digits, so each
 * patch is one character and cannot change the document's length.
 */
#define H713_CFG_OFF_SOURCE_ID	0x0e48
#define H713_CFG_OFF_ELOG_MODE	0x1222
#define H713_CFG_OFF_ELOG_LEVEL	0x123c
#define H713_CFG_OFF_ELOG_ASYNC	0x125e

static int h713_cfg_set(ulong off, char want, const char *what)
{
	ulong a = H713_MIPS_CFG_ADDR + off;
	u8 cur = readb(a);

	if (cur < '0' || cur > '9') {
		printf("H713 disp: %s at +0x%04lx reads '%c', not a digit -- "
		       "config layout differs, skipping\n", what, off, cur);
		return -EINVAL;
	}

	writeb(want, a);
	printf("  %-14s '%c' -> '%c'\n", what, cur, want);

	return 0;
}

static void h713_disp_sample(void)
{
	static const uint at_ms[] = { 0, 100, 500, 1000, 2000, 4000 };
	uint i, elapsed = 0;

	printf("H713 disp: LVDS FIFO over time\n");
	for (i = 0; i < ARRAY_SIZE(at_ms); i++) {
		if (at_ms[i] > elapsed) {
			mdelay(at_ms[i] - elapsed);
			elapsed = at_ms[i];
		}
		printf("  t=%4u ms  fifo=0x%08x  status=0x%08x\n",
		       elapsed, readl(0x05880fe0),
		       readl(H713_MIPS_STATUS_REG));
	}
}

#define H713_DISP_OSD_FB_ADDR		0x6c100000UL
#define H713_DISP_OSD_WIDTH		1280
#define H713_DISP_OSD_HEIGHT		720
#define H713_DISP_OSD_STRIDE		(H713_DISP_OSD_WIDTH * sizeof(u32))
#define H713_DISP_OSD_SIZE		(H713_DISP_OSD_STRIDE * \
					 H713_DISP_OSD_HEIGHT)
#define H713_DISP_PANEL_BLUE_RGB0	0x051c00b0UL
#define H713_DISP_PANEL_BLUE_RGB1	0x051c00b4UL
#define H713_DISP_PANEL_BLUE_CTRL	0x051c00b8UL

/*
 * Every visible phase holds for this long. One second per frame is not enough
 * time to photograph by hand, and these runs are judged by eye: an image the
 * operator cannot capture is not evidence.
 *
 * The bar pattern cycles eight colours, so its rotation repeats with period
 * eight -- eight frames cover every distinct image and anything beyond that
 * is a duplicate. The previous 15x1s run was therefore seven redundant frames,
 * each too brief to catch.
 */
#define H713_DISP_OSD_FRAMES		8
#define H713_DISP_OSD_DWELL_MS		5000

/*
 * LogoRegData.bin DE block 5 writes 0x6c100000 to AFBD +0x38
 * (0x05600178), 0x1400 to +0x30, and 0x02cf04ff to +0x10. Together those
 * identify a linear 1280x720, 32-bit OSD buffer with a 5120-byte stride.
 * Stock U-Boot blits bootlogo.bmp into that buffer after applying the table;
 * our register-only replay must provide pixels explicitly.
 */
static void h713_disp_fill_pattern(uint phase)
{
	static const u32 colours[] = {
		0xffff0000,	/* red */
		0xff00ff00,	/* green */
		0xff0000ff,	/* blue */
		0xffffffff,	/* white */
		0xff00ffff,	/* cyan */
		0xffff00ff,	/* magenta */
		0xffffff00,	/* yellow */
		0xff000000,	/* black */
	};
	u32 *fb = (u32 *)H713_DISP_OSD_FB_ADDR;
	uint bar, x, y;

	for (y = 0; y < H713_DISP_OSD_HEIGHT; y++) {
		for (bar = 0; bar < ARRAY_SIZE(colours); bar++) {
			u32 colour = colours[(bar + phase) %
					     ARRAY_SIZE(colours)];

			for (x = 0; x < H713_DISP_OSD_WIDTH /
				     ARRAY_SIZE(colours); x++)
				*fb++ = colour;
		}
	}

	flush_cache(H713_DISP_OSD_FB_ADDR, H713_DISP_OSD_SIZE);
	printf("H713 panel: pattern %u published at 0x%08lx "
	       "(1280x720 ARGB8888, stride 0x%x)\n",
	       phase, H713_DISP_OSD_FB_ADDR, (uint)H713_DISP_OSD_STRIDE);
}

/*
 * The authenticated display.bin implements SetHWBlueScreenColorPanel at raw
 * +0x17a68. It writes the same three 10-bit components to panel registers
 * +0xb0 and +0xb4. EnableHWBlueScreenPanel at raw +0x16600 then sets
 * +0xb8[6:0] to 0x7f; its disable peer clears +0xb8[5:0].
 *
 * Reproduce that firmware-owned test source only after application readiness,
 * then restore all three registers exactly. This bypasses the OSD framebuffer,
 * AFBD and DE, giving a software-only split between scanout and the
 * panel/LVDS side of the pipeline.
 */
static u32 h713_disp_panel_blue_colour(u32 c0, u32 c1, u32 c2)
{
	return ((c0 & 0x3ff) << 20) |
	       ((c2 & 0x3ff) << 10) |
	       (c1 & 0x3ff);
}

static void h713_disp_panel_blue_test(void)
{
	static const u16 colours[][3] = {
		{ 0x3ff, 0x000, 0x000 },
		{ 0x000, 0x3ff, 0x000 },
		{ 0x000, 0x000, 0x3ff },
		{ 0x3ff, 0x3ff, 0x3ff },
	};
	u32 saved_rgb0 = readl(H713_DISP_PANEL_BLUE_RGB0);
	u32 saved_rgb1 = readl(H713_DISP_PANEL_BLUE_RGB1);
	u32 saved_ctrl = readl(H713_DISP_PANEL_BLUE_CTRL);
	uint i;

	printf("H713 panel: phase 1, 720p firmware hardware-blue-screen source, "
	       "%u colours x %u ms\n",
	       (uint)ARRAY_SIZE(colours), H713_DISP_OSD_DWELL_MS);
	printf("H713 panel: blue-screen baseline: %08x %08x %08x\n",
	       saved_rgb0, saved_rgb1, saved_ctrl);

	for (i = 0; i < ARRAY_SIZE(colours); i++) {
		u32 colour = h713_disp_panel_blue_colour(colours[i][0],
							 colours[i][1],
							 colours[i][2]);

		writel(colour, H713_DISP_PANEL_BLUE_RGB0);
		writel(colour, H713_DISP_PANEL_BLUE_RGB1);
		writel((readl(H713_DISP_PANEL_BLUE_CTRL) & ~0x7f) | 0x7f,
		       H713_DISP_PANEL_BLUE_CTRL);
		printf("H713 panel: hardware colour %u/4: %08x %08x %08x\n",
		       i + 1, readl(H713_DISP_PANEL_BLUE_RGB0),
		       readl(H713_DISP_PANEL_BLUE_RGB1),
		       readl(H713_DISP_PANEL_BLUE_CTRL));
		mdelay(H713_DISP_OSD_DWELL_MS);
	}

	writel(saved_rgb0, H713_DISP_PANEL_BLUE_RGB0);
	writel(saved_rgb1, H713_DISP_PANEL_BLUE_RGB1);
	writel(saved_ctrl, H713_DISP_PANEL_BLUE_CTRL);
	printf("H713 panel: blue-screen registers restored: %08x %08x %08x\n",
	       readl(H713_DISP_PANEL_BLUE_RGB0),
	       readl(H713_DISP_PANEL_BLUE_RGB1),
	       readl(H713_DISP_PANEL_BLUE_CTRL));
}

static void h713_disp_print_panel_gpio(const char *phase)
{
	u32 pb = readl(H713_PB_DATA);
	u32 pf = readl(H713_PF_DATA);
	u32 ph = readl(H713_PH_DATA);

	printf("H713 panel: %s: PB4 cfg=%x latch=%d, "
	       "PB5 cfg=%x latch=%d; PH16 cfg=%x latch=%d, "
	       "PF6 cfg=%x latch=%d\n",
	       phase,
	       sunxi_gpio_get_cfgpin(SUNXI_GPB(4)), !!(pb & BIT(4)),
	       sunxi_gpio_get_cfgpin(SUNXI_GPB(5)), !!(pb & BIT(5)),
	       sunxi_gpio_get_cfgpin(SUNXI_GPH(16)), !!(ph & BIT(16)),
	       sunxi_gpio_get_cfgpin(SUNXI_GPF(6)), !!(pf & BIT(6)));
}

/*
 * Exercise only controls identified by the stock DT, with the internal white
 * source active and the panel timing already at 1280x720.
 *
 * PB5 is shared by the LED backlight and cooling fan. It is forced high and
 * never toggled. PH16 is the active-high panel reset/enable and PF6 is panel
 * power. Do not exercise PB4 as brightness here: it is PWM2 from the
 * pre-override DT, while board B's panel_config.ini selects PWM5. The PWM5 pin
 * route is not proven.
 *
 * The PF6 power cycle is performed with PH16 asserted low, then the stock
 * low-to-high reset sequence is replayed after power returns. This makes a
 * visible opacity/flash change useful evidence that the named GPIOs reach live
 * panel hardware even if the LVDS image remains absent.
 */
static void h713_disp_panel_control_test(void)
{
	const uint fan_bl = SUNXI_GPB(5);
	const uint reset = SUNXI_GPH(16);
	const uint power = SUNXI_GPF(6);
	u32 saved_rgb0 = readl(H713_DISP_PANEL_BLUE_RGB0);
	u32 saved_rgb1 = readl(H713_DISP_PANEL_BLUE_RGB1);
	u32 saved_ctrl = readl(H713_DISP_PANEL_BLUE_CTRL);
	u32 white = h713_disp_panel_blue_colour(0x3ff, 0x3ff, 0x3ff);

	/* Preserve the fan/backlight safety interlock throughout this test. */
	setbits_le32((void *)H713_PB_DATA, BIT(5));
	sunxi_gpio_set_cfgpin(fan_bl, SUNXI_GPIO_OUTPUT);
	sunxi_gpio_set_cfgpin(reset, SUNXI_GPIO_OUTPUT);
	sunxi_gpio_set_cfgpin(power, SUNXI_GPIO_OUTPUT);

	writel(white, H713_DISP_PANEL_BLUE_RGB0);
	writel(white, H713_DISP_PANEL_BLUE_RGB1);
	writel((readl(H713_DISP_PANEL_BLUE_CTRL) & ~0x7f) | 0x7f,
	       H713_DISP_PANEL_BLUE_CTRL);

	printf("H713 panel: phase 2, power/enable readiness test; "
	       "internal white remains selected\n");
	h713_disp_print_panel_gpio("control baseline");
	mdelay(2000);

	printf("H713 panel: PH16 reset/enable LOW for 3 seconds\n");
	clrbits_le32((void *)H713_PH_DATA, BIT(16));
	h713_disp_print_panel_gpio("PH16 low");
	mdelay(3000);

	printf("H713 panel: PH16 reset/enable HIGH for 3 seconds\n");
	setbits_le32((void *)H713_PH_DATA, BIT(16));
	h713_disp_print_panel_gpio("PH16 high");
	mdelay(3000);

	printf("H713 panel: PF6 panel power OFF for 3 seconds; "
	       "PH16 held LOW, PB5 stays HIGH\n");
	clrbits_le32((void *)H713_PH_DATA, BIT(16));
	mdelay(100);
	clrbits_le32((void *)H713_PF_DATA, BIT(6));
	h713_disp_print_panel_gpio("PF6 low");
	mdelay(3000);

	printf("H713 panel: PF6 panel power ON; replaying stock PH16 release\n");
	setbits_le32((void *)H713_PF_DATA, BIT(6));
	mdelay(2);
	setbits_le32((void *)H713_PH_DATA, BIT(16));
	mdelay(5);
	h713_disp_print_panel_gpio("panel repowered");
	mdelay(5000);

	writel(saved_rgb0, H713_DISP_PANEL_BLUE_RGB0);
	writel(saved_rgb1, H713_DISP_PANEL_BLUE_RGB1);
	writel(saved_ctrl, H713_DISP_PANEL_BLUE_CTRL);
	printf("H713 panel: control test complete; blue-screen registers "
	       "restored: %08x %08x %08x\n",
	       readl(H713_DISP_PANEL_BLUE_RGB0),
	       readl(H713_DISP_PANEL_BLUE_RGB1),
	       readl(H713_DISP_PANEL_BLUE_CTRL));
}

/*
 * Restore the panel timing that project 0x33's timing block 6 programmed
 * before the MIPS replaced it with 1080p. Carry the live bit-31 enables into
 * +0x2c/+0x30, then pulse the same +0x0c latch used by the vendor table.
 */
static void h713_disp_latch_panel_timing(void)
{
	u32 ctl = readl(0x0588000c);
	u32 mode = readl(0x0588001c);

	writel((mode & ~0x7) | 0x4, 0x0588001c);
	writel(0x02f80550, 0x05880020);	/* 1360x760 total */
	writel(0x02d00500, 0x05880024);	/* 1280x720 active */
	writel(0x00140028, 0x05880028);
	writel(0x80000014, 0x0588002c);
	writel(0x80010003, 0x05880030);
	writel(ctl | BIT(0), 0x0588000c);
	udelay(1);
	writel(ctl & ~BIT(0), 0x0588000c);

	printf("H713 panel: 720p timing latched: %08x %08x %08x "
	       "%08x %08x %08x\n",
	       readl(0x0588001c), readl(0x05880020),
	       readl(0x05880024), readl(0x05880028),
	       readl(0x0588002c), readl(0x05880030));
}

static void h713_disp_animate_pattern(uint frames, uint dwell_ms, uint phase)
{
	uint i;

	for (i = 0; i < frames; i++) {
		h713_disp_fill_pattern(phase + i);
		printf("H713 panel: animation %u/%u, holding %u ms\n",
		       i + 1, frames, dwell_ms);
		mdelay(dwell_ms);
	}
}

/*
 * Re-apply the selected DE block after the coprocessor has settled.
 *
 * The firmware demonstrably reprograms display hardware after our replay: it
 * rewrites the TCON timing to 1080p, and 0x05600168 -- written 0x000003b2 by
 * DE block 5 -- reads back zero once it has run. Re-imposing the TCON timing
 * post-readiness is already known to stick (h713_disp_latch_panel_timing), so
 * the OSD path gets the same treatment: replay the block, which re-asserts
 * AFBD's format, size, stride, buffer address and enable bit together with the
 * mixer and DE registers the same block owns.
 *
 * The block is replayed rather than hand-written so it applies exactly what
 * the records say, including the panel-config patches already made to them.
 * DE block 5 touches only 0x0560xxxx, 0x0525cxxx, 0x0524cxxx and 0x05280xxx,
 * so it cannot disturb the TCON timing latched just before it.
 *
 * This is a diagnostic. If the OSD appears only after the re-assert, the
 * firmware owns the fetch path and the real work is in how ownership is handed
 * over -- not in another framebuffer pattern.
 */
static int h713_disp_reassert_osd(ulong blob, u32 project)
{
	struct h713_disp_sel sel;
	int ret;

	ret = h713_disp_lookup(blob, project, &sel);
	if (ret)
		return ret;

	printf("H713 panel: re-asserting DE block %u after MIPS readiness\n",
	       sel.de);

	/* Stock writes the mixer control ahead of the DE table; keep parity. */
	writel(H713_DISPLAY_MIXER_CTRL_VALUE, H713_DISPLAY_MIXER_CTRL_REG);

	ret = h713_logo_walk(blob, h713_disp_de[sel.de].start,
			     h713_disp_de[sel.de].end, true);
	if (ret)
		return ret;

	/* AFBD's buffer address was just rewritten; republish the pixels. */
	h713_disp_fill_pattern(0);
	return 0;
}

#define H713_DISP_AFBD_ENABLE_REG	0x05600144UL
#define H713_DISP_AFBD_STATUS_REG	0x05600168UL
#define H713_DISP_LVDS_SCAN_REG		0x05880000UL

/*
 * 0x05600144 is the last record DE block 5 applies -- a single-bit write under
 * mask 1 -- and it reads back zero afterwards. That is either a self-clearing
 * one-shot trigger, in which case the zero means nothing, or an enable that
 * refuses to latch. Those need different fixes, so separate them.
 *
 * Write the bit and read it straight back, repeatedly. Sample two neighbours
 * at the same time: 0x05600168, which holds a small changing value and behaves
 * like AFBD status rather than the configuration word the table treats it as,
 * and 0x05880000, whose two halves track the raster position within the
 * programmed 1360x760 and therefore prove the TCON is still scanning.
 *
 * If the enable is briefly observable set, or either neighbour reacts, the
 * write lands. If nothing anywhere moves, it is being swallowed.
 */
static void h713_disp_afbd_enable_probe(void)
{
	uint i;

	printf("H713 panel: AFBD enable probe\n");
	printf("  baseline    en=%08x status=%08x scan=%08x\n",
	       readl(H713_DISP_AFBD_ENABLE_REG),
	       readl(H713_DISP_AFBD_STATUS_REG),
	       readl(H713_DISP_LVDS_SCAN_REG));

	/* Masked read-modify-write, exactly as the record applier does it. */
	writel((readl(H713_DISP_AFBD_ENABLE_REG) & ~1UL) | 1UL,
	       H713_DISP_AFBD_ENABLE_REG);

	for (i = 0; i < 4; i++)
		printf("  read %u      en=%08x status=%08x scan=%08x\n", i,
		       readl(H713_DISP_AFBD_ENABLE_REG),
		       readl(H713_DISP_AFBD_STATUS_REG),
		       readl(H713_DISP_LVDS_SCAN_REG));

	mdelay(50);
	printf("  +50ms       en=%08x status=%08x scan=%08x\n",
	       readl(H713_DISP_AFBD_ENABLE_REG),
	       readl(H713_DISP_AFBD_STATUS_REG),
	       readl(H713_DISP_LVDS_SCAN_REG));
}

/*
 * CPU_COMM routine identities.
 *
 * The firmware registers its HAL routines by name and the transport addresses
 * them by a hashed id. The hash is an Allwinner CRC32 variant over
 * "<name>_<cpu_id>_<pid_low12>", seed 0x123456, cpu_id 1 for MIPS-side
 * routines. Reproducing it against ten independently documented ids matched
 * 10/10, and the 85 THal_Vp_ and MipsHalCallback_ name strings in the
 * authenticated display.bin match the 82 registrations counted on hardware.
 *
 * These are the ids worth recognising in a live table. THal_Vp_*BlackScreen is
 * first because a firmware that powers on with black-screen asserted would
 * explain every observation to date: the raster runs, the LVDS PHY is
 * configured, PHY-injected colours are visible because they inject downstream
 * of composition -- and the composited output is forced black.
 */
static const struct { u32 id; const char *name; } h713_comm_routines[] = {
	{ 0xb66041d8, "THal_Vp_DisableBlackScreen"  },
	{ 0xa30d4c6b, "THal_Vp_EnableBlackScreen"   },
	{ 0x143ffc87, "THal_Vp_DisableScreenCover"  },
	{ 0x0152f134, "THal_Vp_EnableScreenCover"   },
	{ 0x396f16bf, "THal_Vp_SetImageBufferAddr"  },
	{ 0x2f02f7dd, "THal_Vp_GetImageBufferAddr"  },
	{ 0xeaf13de5, "THal_Vp_SetSource"           },
	{ 0x24efc7c9, "THal_Vp_GetSource"           },
	{ 0x1c6ff747, "THal_Vp_Init"                },
	{ 0x3ab1d1dc, "THal_Vp_DisableVideoFreeze"  },
	{ 0x7bbd5772, "THal_Vp_Wce_GetActiveWindow" },
	{ 0x51ad877e, "Thal_Vp_SetBacklightLevel"   },
	{ 0xb46ce545, "Thal_Vp_SetBacklightPwmInfo" },
};

/*
 * Send one CPU_COMM CALL and poll for its reply.
 *
 * Every step below is transcribed from display.bin rather than inferred:
 *
 *   queue      0x8b1195ec/0x8b1195b8 are mirrors of the share_seq addressing,
 *              so a message from X to Y lives in share_seq(Y, X, idx). For
 *              ARM(0) -> MIPS(1) CALL that is share_seq(1,0,0), and the reply
 *              comes back in share_seq(0,1,1).
 *   allocate   0x8b118be8 takes share_seq+0x78 and pops via 0x8b1180ec, which
 *              returns base_addr + rd_idx*item_size and NULL when rd == wr.
 *              The popped entry holds the slot's ARM-physical address, and the
 *              slot carries its own index at +0x04, bounds-checked below 20.
 *   fill       memcpy(slot, msg, 104) at 0x8b1201b8, then slot[+0x04] = index
 *              and slot[+0x0A] = 2, then an assertion that the slot address is
 *              exactly share_seq + 0x168 + 104*index.
 *   publish    0x8b11f9ec writes share_seq[+0x10] = index, [+0x08] = flags,
 *              [+0x14] = session, [+0x18] = wait pointer, bumps [+0x04].
 *   doorbell   a single write to 0x03003874; the pulse workaround in the Linux
 *              tree belongs to the ARISC path, not this one.
 *
 * The wait pointer is published as zero deliberately. The receiver stores it
 * at share_seq[+0x70]/[+0x74] without dereferencing, and the signal primitive
 * at 0x8b15c27c null-checks and returns an error rather than writing. U-Boot
 * polls, so it does not need to be signalled. The residual risk is behavioural
 * -- the firmware may log an error path -- not memory corruption.
 *
 * This is the first traffic in either direction. It writes into a live
 * coprocessor's queues, so it is a separate opt-in command and never runs as
 * part of panel-test.
 */
#define H713_COMM_CALL_SEQ_OFF	0x000026b8UL	/* share_seq(1,0,0) FreeCall  */
#define H713_COMM_RET_SEQ_OFF	0x00001d30UL	/* share_seq(0,1,1) FreeReturn*/
#define H713_COMM_MSG_SIZE	104
#define H713_COMM_DOORBELL	0x03003874UL	/* User2 sub0 port1 MSG_DATA  */
#define H713_COMM_DOORBELL_CALL	0x00000002	/* msg_type 0, intr_type 2    */
#define H713_COMM_MSGBOX_VERSION 0x03003810UL	/* User2 sub0 +0x10           */
#define H713_COMM_MSGBOX_COUNT	0x03003864UL	/* User2 sub0 +0x60 + 4*port  */
#define H713_COMM_MSGBOX_BGR	0x0200171cUL	/* bit 0 gate, bit 16 reset   */
#define H713_COMM_MSGBOX_TX_IRQ_EN 0x03003830UL	/* User2 sub0 +0x30           */

static int h713_comm_call(u32 comp_id, const u32 *params, uint nparams)
{
	ulong seq = H713_MIPS_SHMEM_ADDR + H713_COMM_CALL_SEQ_OFF;
	ulong ret_seq = H713_MIPS_SHMEM_ADDR + H713_COMM_RET_SEQ_OFF;
	ulong fifo = seq + H713_MIPS_SEQ_FIFO_OFF;
	u32 rd, wr, cap, isz, base;
	ulong entry, slot, expect;
	u8 msg[H713_COMM_MSG_SIZE];
	uint index, i;
	int waited;
	bool drained = false;

	if (nparams > 10) {
		printf("H713 comm: at most 10 parameters\n");
		return -EINVAL;
	}
	if (h713_mips_read_shmem(H713_MIPS_SHMEM_MAGIC1_OFF) !=
	    H713_MIPS_SHMEM_MAGIC) {
		printf("H713 comm: shared memory not published this boot\n");
		return -ENODEV;
	}

	rd   = h713_mips_read_shmem(H713_COMM_CALL_SEQ_OFF +
				    H713_MIPS_SEQ_FIFO_OFF + 0x00);
	wr   = h713_mips_read_shmem(H713_COMM_CALL_SEQ_OFF +
				    H713_MIPS_SEQ_FIFO_OFF + 0x04);
	cap  = h713_mips_read_shmem(H713_COMM_CALL_SEQ_OFF +
				    H713_MIPS_SEQ_FIFO_OFF + 0x10);
	isz  = h713_mips_read_shmem(H713_COMM_CALL_SEQ_OFF +
				    H713_MIPS_SEQ_FIFO_OFF + 0x14);
	base = h713_mips_read_shmem(H713_COMM_CALL_SEQ_OFF +
				    H713_MIPS_SEQ_FIFO_OFF + 0x18);

	printf("H713 comm: FreeCall @+0x%05lx  rd=%u wr=%u cap=%u isz=%u "
	       "base=%08x\n", H713_COMM_CALL_SEQ_OFF, rd, wr, cap, isz, base);

	if (!cap || rd == wr) {
		printf("H713 comm: no free slot (ring empty)\n");
		return -EBUSY;
	}

	/* 0x8b1180ec: entry = base_addr + rd_idx * item_size */
	entry = base + rd * isz;
	invalidate_dcache_range(entry & ~(CONFIG_SYS_CACHELINE_SIZE - 1),
				(entry & ~(CONFIG_SYS_CACHELINE_SIZE - 1)) +
				CONFIG_SYS_CACHELINE_SIZE);
	slot = readl(entry);

	/* 0x8b118be8: the slot carries its own index, bounded by 20. */
	invalidate_dcache_range(slot & ~(CONFIG_SYS_CACHELINE_SIZE - 1),
				(slot & ~(CONFIG_SYS_CACHELINE_SIZE - 1)) +
				CONFIG_SYS_CACHELINE_SIZE);
	index = readw(slot + 0x04);

	expect = seq + H713_MIPS_SEQ_SLOTS_OFF + index * H713_COMM_MSG_SIZE;
	printf("H713 comm: slot %u @0x%08lx (index %u, expect 0x%08lx)\n",
	       rd, slot, index, expect);

	if (index >= H713_MIPS_SEQ_SLOTS || slot != expect) {
		printf("H713 comm: slot inconsistent -- refusing to send\n");
		return -EINVAL;
	}

	/*
	 * Build the message. Layout confirmed from the consumer, command_action
	 * at 0x8b120bc0: dst_cpu +0x02, slot_index +0x04, flags +0x06, parameter
	 * count in cmd_type +0x08, session +0x0C, comp_id +0x28, params +0x2C.
	 * flags stays zero -- the Linux driver clears its notify bit for a
	 * synchronous call, and command_action tests bit 2 of the published
	 * state to take an alternate path.
	 */
	memset(msg, 0, sizeof(msg));
	*(u16 *)(msg + 0x02) = 1;			/* dst_cpu = MIPS   */
	*(u16 *)(msg + 0x04) = (u16)index;
	*(u16 *)(msg + 0x06) = 0;			/* flags            */
	*(u16 *)(msg + 0x08) = (u16)nparams;		/* cmd_type = count */
	*(u32 *)(msg + 0x0c) = 0x00000001;		/* session id       */
	*(u32 *)(msg + 0x28) = comp_id;
	for (i = 0; i < nparams; i++)
		*(u32 *)(msg + 0x2c + i * 4) = params[i];

	memcpy((void *)slot, msg, sizeof(msg));
	writew((u16)index, slot + 0x04);
	writew(2, slot + 0x0a);				/* CALL marker      */
	flush_cache(slot & ~(CONFIG_SYS_CACHELINE_SIZE - 1),
		    H713_COMM_MSG_SIZE + CONFIG_SYS_CACHELINE_SIZE);

	/* Consume the ring entry. */
	writel((rd + 1) % cap, fifo + 0x00);

	/* 0x8b11f9ec, with the wait pointer deliberately zero. */
	writeb((u8)index, seq + 0x10);
	writeb(0, seq + 0x08);
	writel(0x00000001, seq + 0x14);			/* session          */
	writel(0, seq + 0x18);				/* wait ptr lo      */
	writel(0, seq + 0x1c);				/* wait ptr hi      */
	writel(h713_mips_read_shmem(H713_COMM_CALL_SEQ_OFF + 0x04) + 1,
	       seq + 0x04);
	flush_cache(seq, 0x80);

	printf("H713 comm: published index %u, comp_id 0x%08x, %u param(s)\n",
	       index, comp_id, nparams);

	/*
	 * The msgbox has its own bus gate and reset, which Linux takes via
	 * CLK_BUS_MSGBOX/RST_BUS_MSGBOX and U-Boot has never touched. An
	 * unclocked block swallows the doorbell silently, so enable it and
	 * prove it is alive before writing: the per-sub-block version register
	 * reads 0x00020000 on a live msgbox.
	 */
	printf("H713 comm: msgbox BGR before %08x\n",
	       readl(H713_COMM_MSGBOX_BGR));
	setbits_le32((void *)H713_COMM_MSGBOX_BGR, BIT(0) | BIT(16));
	udelay(20);
	printf("H713 comm: msgbox BGR after  %08x, version %08x (expect "
	       "00020000), fifo count %u\n",
	       readl(H713_COMM_MSGBOX_BGR), readl(H713_COMM_MSGBOX_VERSION),
	       readl(H713_COMM_MSGBOX_COUNT));

	/*
	 * H713's msgbox is edge-triggered, unlike H6's level-triggered one, so
	 * writing MSG_DATA alone leaves the message sitting in the FIFO without
	 * waking the receiver -- observed directly: the count went 0 -> 1 and
	 * the MIPS never drained it. The receiver needs a TX_IRQ_EN pulse.
	 *
	 * TX_IRQ_EN is at sub-block +0x30 and the bit is BIT(2*port + 1); the
	 * Linux tree's ARISC path pulses BIT(7) for port 3, which is the same
	 * rule. MIPS is port 1, so BIT(3).
	 */
	writel(H713_COMM_DOORBELL_CALL, H713_COMM_DOORBELL);
	writel(BIT(3), H713_COMM_MSGBOX_TX_IRQ_EN);
	udelay(10);
	writel(0, H713_COMM_MSGBOX_TX_IRQ_EN);
	udelay(100);
	printf("H713 comm: doorbell rung + IRQ pulsed; fifo count now %u\n",
	       readl(H713_COMM_MSGBOX_COUNT));

	/* Poll the return transport rather than waiting to be signalled. */
	for (waited = 0; waited < 2000; waited++) {
		u32 ridx = h713_mips_read_shmem(H713_COMM_RET_SEQ_OFF + 0x10) &
			   0xff;
		u32 fc = readl(H713_COMM_MSGBOX_COUNT);

		if (!fc && !drained) {
			printf("H713 comm: MIPS drained the FIFO after %d ms\n",
			       waited);
			drained = true;
		}

		if (ridx < H713_MIPS_SEQ_SLOTS) {
			ulong rslot = ret_seq + H713_MIPS_SEQ_SLOTS_OFF +
				      ridx * H713_COMM_MSG_SIZE;

			invalidate_dcache_range(rslot, rslot +
						H713_COMM_MSG_SIZE);
			printf("H713 comm: reply after %d ms, slot %u\n",
			       waited, ridx);
			printf("  session=%08x comp_id=%08x\n",
			       readl(rslot + 0x0c), readl(rslot + 0x28));
			for (i = 0; i < 6; i++)
				printf("  ret[%u]=%08x\n", i,
				       readl(rslot + 0x2c + i * 4));
			return 0;
		}
		mdelay(1);
	}

	printf("H713 comm: no reply within 2000 ms (fifo count %u, %s)\n",
	       readl(H713_COMM_MSGBOX_COUNT),
	       drained ? "was drained" : "never drained");
	printf("  FreeCall  rd=%u wr=%u  idx=%02x state=%02x\n",
	       h713_mips_read_shmem(H713_COMM_CALL_SEQ_OFF +
				    H713_MIPS_SEQ_FIFO_OFF + 0x00),
	       h713_mips_read_shmem(H713_COMM_CALL_SEQ_OFF +
				    H713_MIPS_SEQ_FIFO_OFF + 0x04),
	       h713_mips_read_shmem(H713_COMM_CALL_SEQ_OFF + 0x10) & 0xff,
	       h713_mips_read_shmem(H713_COMM_CALL_SEQ_OFF + 0x08) & 0xff);
	return -ETIMEDOUT;
}

/*
 * Read-only inspection of the eight share_seq transports.
 *
 * U-Boot builds each one with rd_idx=0 and wr_idx=20 -- a ring holding twenty
 * free slots in a capacity of twenty-one. If the firmware has allocated or
 * consumed any, those indices will have moved, which is the difference between
 * "the firmware tolerated our structures" and "the firmware is using them".
 *
 * Makes no writes and sends no messages, so it does not consume the
 * one-launch-per-power-cycle budget.
 */
static int h713_disp_comm_state(void)
{
	uint cpu, dir, idx, live = 0;

	if (h713_mips_read_shmem(H713_MIPS_SHMEM_MAGIC1_OFF) !=
	    H713_MIPS_SHMEM_MAGIC) {
		printf("H713 comm: shared memory not published this boot\n");
		return -ENODEV;
	}

	printf("H713 comm: share_seq transports "
	       "(as built: rd=0 wr=%u cap=%u)\n",
	       H713_MIPS_SEQ_SLOTS, H713_MIPS_SEQ_FIFO_CAPACITY);

	for (cpu = 0; cpu < 2; cpu++)
	for (dir = 0; dir < 2; dir++)
	for (idx = 0; idx < 2; idx++) {
		ulong off = H713_MIPS_SEQ_BASE_OFF +
			    cpu * H713_MIPS_SEQ_PER_CPU +
			    dir * H713_MIPS_SEQ_PER_DIR +
			    idx * H713_MIPS_SEQ_STRIDE;
		ulong f = off + H713_MIPS_SEQ_FIFO_OFF;
		u32 rd = h713_mips_read_shmem(f + 0x00);
		u32 wr = h713_mips_read_shmem(f + 0x04);
		u32 peak = h713_mips_read_shmem(f + 0x08);
		u32 cap = h713_mips_read_shmem(f + 0x10);
		u32 base = h713_mips_read_shmem(f + 0x18);
		char name[0x14];
		uint i;
		bool moved;

		for (i = 0; i < sizeof(name) - 1; i++)
			name[i] = (char)(h713_mips_read_shmem(off + 0x98 +
					 (i & ~3)) >> ((i & 3) * 8));
		name[sizeof(name) - 1] = 0;

		moved = rd != 0 || wr != H713_MIPS_SEQ_SLOTS;
		if (moved)
			live++;

		printf("  cpu=%u dir=%u idx=%u %-11s @+0x%05lx  "
		       "rd=%-3u wr=%-3u peak=%-3u cap=%-3u base=%08x%s\n",
		       cpu, dir, idx, name, off, rd, wr, peak, cap, base,
		       moved ? "  <== MOVED" : "");
	}

	printf("H713 comm: %u of 8 transport(s) show movement\n", live);
	if (!live)
		printf("H713 comm: firmware accepted the structures but has not "
		       "used them; a send would be the first traffic\n");
	return 0;
}

/*
 * Read-only inspection of the live call table. This makes no writes and sends
 * no messages, so it does not count as a second MIPS launch: run it at the
 * prompt after panel-test or mips-test, on the same boot, while the firmware
 * is still up.
 *
 * Two outputs. First, any entry word matching a known routine id, with the
 * offset it was found at -- that locates the id field within the 0x60-byte
 * entry and confirms the hash convention against this firmware. Second, raw
 * hex for the first few populated entries, so the layout can be worked out
 * offline even if nothing matches.
 */
static int h713_disp_call_table(uint raw_entries)
{
	u32 version, count;
	uint i, w, shown = 0, populated = 0, matched = 0;

	version = h713_mips_read_shmem(H713_MIPS_SHMEM_CALL_VERSION_OFF);
	count   = h713_mips_read_shmem(H713_MIPS_SHMEM_CALL_COUNT_OFF);

	printf("H713 comm: call table @0x%08lx  version=%u count=%u\n",
	       H713_MIPS_SHMEM_ADDR + H713_MIPS_SHMEM_CALL_TABLE_OFF,
	       version, count);

	if (!count || count > H713_MIPS_SHMEM_CALL_ENTRY_COUNT) {
		printf("H713 comm: table not populated -- run the firmware "
		       "first, on this boot\n");
		return -ENODEV;
	}

	for (i = 0; i < H713_MIPS_SHMEM_CALL_ENTRY_COUNT; i++) {
		ulong off = H713_MIPS_SHMEM_CALL_TABLE_OFF +
			    i * H713_MIPS_SHMEM_CALL_ENTRY_SIZE;
		u32 word[H713_MIPS_SHMEM_CALL_ENTRY_SIZE / 4];
		bool any = false;

		for (w = 0; w < ARRAY_SIZE(word); w++) {
			word[w] = h713_mips_read_shmem(off + w * 4);
			/* The free sentinel is not content. */
			if (word[w] && !(w * 4 == H713_MIPS_SHMEM_CALL_NEXT_OFF &&
					 word[w] == ~0U))
				any = true;
		}
		if (!any)
			continue;
		populated++;

		for (w = 0; w < ARRAY_SIZE(word); w++) {
			uint r;

			for (r = 0; r < ARRAY_SIZE(h713_comm_routines); r++) {
				if (word[w] != h713_comm_routines[r].id)
					continue;
				printf("  entry %4u  +0x%02x = %08x  %s\n",
				       i, w * 4, word[w],
				       h713_comm_routines[r].name);
				matched++;
			}
		}

		if (shown < raw_entries) {
			printf("  entry %4u raw:", i);
			for (w = 0; w < ARRAY_SIZE(word); w++) {
				if (!(w % 8))
					printf("\n    +0x%02x:", w * 4);
				printf(" %08x", word[w]);
			}
			printf("\n");
			shown++;
		}
	}

	printf("H713 comm: %u populated entr%s, %u known routine id(s) matched\n",
	       populated, populated == 1 ? "y" : "ies", matched);
	if (!matched)
		printf("H713 comm: no id matched -- the raw dumps above are the "
		       "input for working out the entry layout offline\n");
	return 0;
}

static int h713_disp_panel_test(u32 project, bool release_mips)
{
	int ret;

	ret = h713_disp_load(project);
	if (ret)
		return ret;

	/* Seed before AFBD is enabled, then republish after MIPS readiness. */
	h713_disp_fill_pattern(0);
	ret = h713_disp_run(H713_DISP_LOGO_ADDR, project, true, true, false,
			    false, true, release_mips);
	if (ret)
		return ret;

	h713_disp_latch_panel_timing();
	printf("H713 panel: phase 0, panel timing established before "
	       "downstream tests\n");
	h713_disp_panel_blue_test();
	h713_disp_panel_control_test();

	/*
	 * Capture the fetch path as the firmware left it, re-assert it, then
	 * capture it again. The pair of dumps is the evidence: if the AFBD
	 * words differ across the re-assert, the firmware had torn our OSD
	 * layer down and the blank screen was never a scanout question.
	 */
	printf("H713 panel: OSD state %s\n",
	       release_mips ? "as the firmware left it"
			    : "from the ARM sequence alone (MIPS in reset)");
	h713_disp_dump(false);

	ret = h713_disp_reassert_osd(H713_DISP_LOGO_ADDR, project);
	if (ret)
		return ret;

	printf("H713 panel: OSD state after re-assert\n");
	h713_disp_dump(false);

	h713_disp_afbd_enable_probe();

	printf("H713 panel: phase 3, moving OSD at panel 720p timing, "
	       "%u frames x %u ms\n",
	       H713_DISP_OSD_FRAMES, H713_DISP_OSD_DWELL_MS);
	h713_disp_animate_pattern(H713_DISP_OSD_FRAMES,
				  H713_DISP_OSD_DWELL_MS, 1);

	if (!release_mips) {
		printf("H713 panel: pattern test complete; MIPS never released, "
		       "so this run may be repeated without a power cycle\n");
		return 0;
	}

	printf("H713 panel: pattern test complete; MIPS left running, "
	       "power-cycle before another run\n");
	return 0;
}

static int h713_disp_test(u32 project, u32 source_id, u32 level)
{
	int ret;

	ret = h713_disp_load(project);
	if (ret)
		return ret;

	printf("H713 disp: config patches\n");
	h713_cfg_set(H713_CFG_OFF_SOURCE_ID, '0' + source_id, "source_id");
	h713_cfg_set(H713_CFG_OFF_ELOG_MODE, '2', "elog mode");
	h713_cfg_set(H713_CFG_OFF_ELOG_ASYNC, '0', "elog async");
	h713_cfg_set(H713_CFG_OFF_ELOG_LEVEL, '0' + level, "elog level");

	ret = h713_disp_run(H713_DISP_LOGO_ADDR, project, false, false, false,
			    false, false, true);
	if (ret)
		return ret;

	h713_disp_sample();

	printf("H713 disp: firmware log\n");
	h713_mips_log(0x4b232000, 0x4bd00000);

	return 0;
}

static int do_h713_disp(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[])
{
	ulong blob;

	if ((argc == 2 || argc == 3) && !strcmp(argv[1], "dump")) {
		bool force = argc == 3 && !strcmp(argv[2], "force");

		if (argc == 3 && !force)
			return CMD_RET_USAGE;
		h713_disp_dump(force);
		return CMD_RET_SUCCESS;
	}

	/* Everything in one command, so operator timing is not a variable. */
	if (argc >= 3 && !strcmp(argv[1], "test")) {
		u32 project = hextoul(argv[2], NULL);
		u32 src = argc > 3 ? dectoul(argv[3], NULL) : 2;
		u32 lvl = argc > 4 ? dectoul(argv[4], NULL) : 3;

		if (argc > 5 || src > 9 || lvl > 5)
			return CMD_RET_USAGE;
		return h713_disp_test(project, src, lvl) ?
		       CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	/*
	 * Reproduce stock's panel-power phase, publish cache-coherent pixels,
	 * prove full firmware readiness, then compare the firmware-selected
	 * timing with the panel's timing.
	 */
	/*
	 * Opt-in: this is the only command that writes into the live
	 * coprocessor's queues. GetImageBufferAddr (0x2f02f7dd) is the safest
	 * first target -- it takes no parameters and only reads firmware state.
	 */
	if (argc >= 3 && !strcmp(argv[1], "commcall")) {
		u32 comp_id = hextoul(argv[2], NULL);
		u32 p[10];
		uint n = argc - 3, k;

		if (n > ARRAY_SIZE(p))
			return CMD_RET_USAGE;
		for (k = 0; k < n; k++)
			p[k] = hextoul(argv[3 + k], NULL);
		return h713_comm_call(comp_id, p, n) ?
		       CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	if (argc == 2 && !strcmp(argv[1], "commstate"))
		return h713_disp_comm_state() ?
		       CMD_RET_FAILURE : CMD_RET_SUCCESS;

	if ((argc == 2 || argc == 3) && !strcmp(argv[1], "calltable")) {
		uint n = argc == 3 ? dectoul(argv[2], NULL) : 4;

		return h713_disp_call_table(n) ?
		       CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	if ((argc == 3 || argc == 4) && !strcmp(argv[1], "panel-test")) {
		bool noboot = argc == 4 && !strcmp(argv[3], "noboot");

		if (argc == 4 && !noboot)
			return CMD_RET_USAGE;
		return h713_disp_panel_test(hextoul(argv[2], NULL), !noboot) ?
		       CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	/* Load only, so display_cfg.xml can be patched before the run. */
	if (argc == 3 && !strcmp(argv[1], "load")) {
		if (h713_disp_load(hextoul(argv[2], NULL)))
			return CMD_RET_FAILURE;
		printf("H713 disp: loaded; run with 'h713_disp 0x%08lx <project>'\n",
		       H713_DISP_LOGO_ADDR);
		return CMD_RET_SUCCESS;
	}

	/*
	 * Full display launch plus the CPU_COMM handoff. The guarded HDCP wait
	 * override takes the firmware's own timeout path so its startup can
	 * reach the CPU_COMM thread while interrupts are still masked.
	 */
	if (argc == 3 && (!strcmp(argv[1], "mips-test") ||
			  !strcmp(argv[1], "mips-trace") ||
			  !strcmp(argv[1], "mips-stability"))) {
		u32 project = hextoul(argv[2], NULL);
		bool trace = !strcmp(argv[1], "mips-trace");
		bool stability = !strcmp(argv[1], "mips-stability");
		int ret;

		if (h713_disp_load(project))
			return CMD_RET_FAILURE;
		ret = h713_disp_run(H713_DISP_LOGO_ADDR, project, true, true,
				    trace, stability, false, true);
		printf("H713 disp: MIPS test complete; power-cycle before "
		       "another MIPS run\n");
		return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	/* Load everything from eMMC, then run: one command from power-on. */
	if (argc >= 3 && !strcmp(argv[1], "auto")) {
		u32 project = hextoul(argv[2], NULL);
		bool nowait = false;
		int i;

		for (i = 3; i < argc; i++) {
			if (!strcmp(argv[i], "nowait"))
				nowait = true;
			else
				return CMD_RET_USAGE;
		}

		if (h713_disp_load(project))
			return CMD_RET_FAILURE;
		return h713_disp_run(H713_DISP_LOGO_ADDR, project, nowait,
				     false, false, false, false, true) ?
		       CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	if (argc == 3 && !strcmp(argv[1], "list")) {
		h713_disp_list(hextoul(argv[2], NULL));
		return CMD_RET_SUCCESS;
	}

	if (argc != 3 && argc != 4)
		return CMD_RET_USAGE;
	if (argc == 4 && strcmp(argv[3], "nowait"))
		return CMD_RET_USAGE;

	blob = hextoul(argv[1], NULL);

	return h713_disp_run(blob, hextoul(argv[2], NULL), argc == 4, false,
			     false, false, false, true) ?
	       CMD_RET_FAILURE : CMD_RET_SUCCESS;
}

U_BOOT_CMD(h713_disp, 5, 0, do_h713_disp,
	   "run stock's fastlogo display sequence for a project ID",
	   "test <project-id> [source] [level]  - load, patch, run, sample, log\n"
	   "h713_disp mips-test <project-id>    - run with CPU_COMM readiness proof\n"
	   "h713_disp mips-trace <project-id>   - stream full-launch startup markers\n"
	   "h713_disp mips-stability <project-id> - run 60s heartbeat/exception test\n"
	   "h713_disp calltable [raw-entries]   - read the live CPU_COMM call table\n"
	   "h713_disp commstate                 - read the CPU_COMM transports\n"
	   "h713_disp commcall <id> [args..]    - send one CPU_COMM CALL (writes!)\n"
	   "h713_disp panel-test <project-id> [noboot]\n"
	   "                                    - 720p colours, power controls, OSD\n"
	   "                                      noboot: hold MIPS in reset, ARM only\n"
	   "h713_disp auto <project-id> [nowait] - load from eMMC and run\n"
	   "h713_disp load <project-id>         - load from eMMC only\n"
	   "h713_disp <blob-addr> <project-id> [nowait] - run against a staged blob\n"
	   "h713_disp list <blob-addr>          - show every project's tables\n"
	   "h713_disp dump [force]              - dump the display register blocks"
);
