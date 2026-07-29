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
/*
 * One second was chosen when the firmware stalled in its first few hundred
 * milliseconds. With the config and TSE artifacts staged it now runs its whole
 * instrumented startup, so give an uninstrumented run the same budget as a
 * traced one before calling readiness a failure.
 */
#define H713_MIPS_READY_TIMEOUT_US	10000000
#define H713_MIPS_TRACE_TIMEOUT_US	10000000
#define H713_MIPS_TRACE_OFF		0x00040000UL
#define H713_MIPS_TRACE_COUNT		151

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

static void h713_mips_prepare_ready_probe(void)
{
	memset((void *)H713_MIPS_SHMEM_ADDR, 0, H713_MIPS_SHMEM_SIZE);

	writel(3, H713_MIPS_SHMEM_ADDR + H713_MIPS_SHMEM_MAX_CPU_OFF);
	writel(H713_MIPS_SHMEM_ARM_READY,
	       H713_MIPS_SHMEM_ADDR + H713_MIPS_SHMEM_ARM_FLAG_OFF);
	writel(0, H713_MIPS_SHMEM_ADDR + H713_MIPS_SHMEM_MIPS_FLAG_OFF);

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

	printf("H713 MIPS: readiness probe shared memory prepared\n");
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
		printf("H713 MIPS: trace[%d]=%u\n", i, value);
	}
}

static void h713_mips_print_trace(void)
{
	int i;

	printf("H713 MIPS: handshake trace");
	for (i = 0; i < H713_MIPS_TRACE_COUNT; i++)
		printf(" %u", h713_mips_read_shmem(H713_MIPS_TRACE_OFF +
						   i * sizeof(u32)));
	printf("\n");
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

/*
 * The firmware's output timing is a hybrid: the geometry lives in the vendor
 * databases, but the selector that picks which record to use is compiled in.
 *
 * _LoadTFDPanelTiming builds the Output_Resolution selector 0x00060004 from an
 * immediate pair at file offset 0x88120 -- "lui v0, 6; addiu v0, v0, 4" --
 * where parameter 6 is Output_Resolution. Selector 0x00060004 resolves through
 * database.TSE's OUTPUT_TIMING_PROJECTOR to the 1080p record, whose fields are
 * copied verbatim into the LVDS timing registers: the u16 quad (2200, 1125,
 * 1920, 1080) is byte-identical to the 0x04650898 / 0x04380780 that
 * 0x05880020 / 0x05880024 read back on the bench. That overrides the 1280x720
 * the vendor's own register table writes for this panel.
 *
 * Rewriting the addiu's immediate to 3 asks for the adjacent resolution. The
 * mixer and DE already run 720p out of the same database -- both read
 * 0x02e4059f, which is the (1440, 741, 1280, 720) record in size-minus-one
 * form -- so the firmware is presently inconsistent with itself, and 3 should
 * bring the TCON into line with the rest of its own pipeline.
 *
 * Applied after the identity check, so the gate still verifies stock bytes.
 */
#define H713_MIPS_RES_SEL_BYTE		(H713_MIPS_FW_ADDR + 0x88124)
#define H713_MIPS_RES_SEL_1080P		0x04
#define H713_MIPS_RES_SEL_720P		0x03

static int h713_mips_release_raw(bool skip_hdcp_wait, bool force_720p)
{
	u32 status, witness;
	int ret;

	ret = h713_mips_verify();
	if (ret)
		return ret;

	if (force_720p) {
		u8 sel = readb(H713_MIPS_RES_SEL_BYTE);

		if (sel != H713_MIPS_RES_SEL_1080P) {
			printf("H713 MIPS: resolution selector is 0x%02x, expected 0x%02x\n",
			       sel, H713_MIPS_RES_SEL_1080P);
			return -EINVAL;
		}
		writeb(H713_MIPS_RES_SEL_720P, H713_MIPS_RES_SEL_BYTE);
		printf("H713 MIPS: Output_Resolution selector forced to 0x%02x\n",
		       H713_MIPS_RES_SEL_720P);
	}

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

	flush_cache(H713_MIPS_FW_ADDR, H713_MIPS_FW_WINDOW_SIZE);
	h713_mips_seed_witness();

	ret = h713_mips_release_reset(false);
	if (ret)
		return ret;
	mdelay(300);

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
		if (trace &&
		    h713_mips_read_shmem(H713_MIPS_TRACE_OFF +
					 150 * sizeof(u32)))
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
		ret = h713_mips_release_raw(false, false);
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
#define H713_PIO_BANK_H		7
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
}

/* Stock's reset: pulse bit 8 of the FIFO control, then re-latch the config. */
static void h713_disp_fifo_reset(void)
{
	u32 cfg = readl(0x0588000c);
	u32 ctl = readl(0x05700088);

	writel(ctl & ~0x100, 0x05700088);
	writel(ctl | 0x100, 0x05700088);
	writel(cfg, 0x0588000c);
}

static int h713_disp_run(ulong blob, u32 project, bool skip_hdcp_wait,
			 bool force_720p)
{
	struct h713_disp_sel sel;
	int ret;

	ret = h713_disp_lookup(blob, project, &sel);
	if (ret)
		return ret;

	printf("H713 disp: project 0x%02x -> prologue %u, timing %u, de %u\n",
	       sel.project, sel.prologue, sel.timing, sel.de);

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

	h713_disp_clocks();

	writel(1, 0x06940000);			/* INCAP */
	mdelay(12);
	writel(0x01800045, 0x051c0010);		/* LVDS enable */
	mdelay(12);

	ret = h713_mips_release_raw(skip_hdcp_wait, force_720p);
	if (ret)
		return ret;

	writel(0x45, 0x051c0010);		/* LVDS finalise */
	h713_disp_configured = true;
	printf("H713 disp: sequence complete, LVDS FIFO status=0x%08x\n",
	       readl(0x05880fe0));

	return 0;
}

static const struct { ulong base; uint words; const char *name; } h713_disp_regs[] = {
	{ 0x05700000, 16, "tvtop"  }, { 0x05800000, 12, "lvds-lane" },
	{ 0x05880000, 16, "lvds"   }, { 0x058c0000, 12, "disp-pll"  },
	{ 0x051c0000,  8, "lvds-phy" }, { 0x0525c000, 16, "mixer"   },
	{ 0x0524c000, 32, "de"     }, { 0x05600140, 16, "afbd"      },
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

	ret = h713_disp_load_tse(project);
	if (ret)
		return ret;

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

	ret = h713_disp_run(H713_DISP_LOGO_ADDR, project, false, false);
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

	/* Load only, so display_cfg.xml can be patched before the run. */
	if (argc == 3 && !strcmp(argv[1], "load")) {
		if (h713_disp_load(hextoul(argv[2], NULL)))
			return CMD_RET_FAILURE;
		printf("H713 disp: loaded; run with 'h713_disp 0x%08lx <project>'\n",
		       H713_DISP_LOGO_ADDR);
		return CMD_RET_SUCCESS;
	}

	/* Load everything from eMMC, then run: one command from power-on. */
	if (argc >= 3 && !strcmp(argv[1], "auto")) {
		u32 project = hextoul(argv[2], NULL);
		bool nowait = false, force_720p = false;
		int i;

		/* Modifiers in any order, so neither has to be remembered. */
		for (i = 3; i < argc; i++) {
			if (!strcmp(argv[i], "nowait"))
				nowait = true;
			else if (!strcmp(argv[i], "720p"))
				force_720p = true;
			else
				return CMD_RET_USAGE;
		}

		if (h713_disp_load(project))
			return CMD_RET_FAILURE;
		return h713_disp_run(H713_DISP_LOGO_ADDR, project, nowait,
				     force_720p) ?
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

	return h713_disp_run(blob, hextoul(argv[2], NULL), argc == 4, false) ?
	       CMD_RET_FAILURE : CMD_RET_SUCCESS;
}

U_BOOT_CMD(h713_disp, 5, 0, do_h713_disp,
	   "run stock's fastlogo display sequence for a project ID",
	   "test <project-id> [source] [level]  - load, patch, run, sample, log\n"
	   "h713_disp auto <project-id> [nowait] [720p] - load from eMMC and run\n"
	   "h713_disp load <project-id>         - load from eMMC only\n"
	   "h713_disp <blob-addr> <project-id> [nowait] - run against a staged blob\n"
	   "h713_disp list <blob-addr>          - show every project's tables\n"
	   "h713_disp dump [force]              - dump the display register blocks"
);
