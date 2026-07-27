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
#include <linux/string.h>
#include <asm/io.h>
#include <u-boot/sha256.h>

#define H713_MIPS_FW_ADDR		0x4b100000UL
#define H713_MIPS_FW_SIZE		0x00132b18UL
#define H713_MIPS_FW_WINDOW_SIZE	0x00500000UL
#define H713_MIPS_BSS_START		0x4b232c00UL
#define H713_MIPS_BSS_END		0x4bac7c28UL
#define H713_MIPS_WITNESS_ADDR		(H713_MIPS_FW_ADDR + \
					 H713_MIPS_FW_WINDOW_SIZE)
#define H713_MIPS_WITNESS_SEED		0x4d495053

#define H713_MIPS_CLK_REG		0x02001600UL
#define H713_MIPS_RESET_REG		0x0200160cUL
#define H713_MIPS_STATUS_REG		0x0306101cUL
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

static const u8 h713_mips_fw_sha256[SHA256_SUM_LEN] = {
	0x16, 0xc7, 0x4a, 0x28, 0x18, 0x7f, 0x34, 0x2d,
	0xe6, 0x57, 0x82, 0x8f, 0xab, 0x65, 0x14, 0x5b,
	0x14, 0x0a, 0xc9, 0x41, 0x1c, 0x40, 0xcc, 0xcc,
	0x02, 0xee, 0xd2, 0x50, 0x47, 0x47, 0x2e, 0xe9,
};

static bool h713_display_prepared;

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

	writel(H713_MIPS_CLK_VALUE, H713_MIPS_CLK_REG);
	writel(H713_MIPS_RESET_ASSERTED, H713_MIPS_RESET_REG);
	mdelay(12);
	writel(H713_MIPS_RESET_STAGE1, H713_MIPS_RESET_REG);
	mdelay(12);
	writel(H713_MIPS_RESET_STAGE2, H713_MIPS_RESET_REG);
	mdelay(12);
	writel(H713_MIPS_RESET_STAGE3, H713_MIPS_RESET_REG);
	mdelay(12);
	writel(H713_MIPS_FW_ADDR, H713_MIPS_BOOTADDR_REG);
	writel(H713_MIPS_RESET_RELEASED, H713_MIPS_RESET_REG);
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

	if (!strcmp(argv[1], "start")) {
		ret = h713_mips_start();
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
	   "h713_mips stop\n"
	   "h713_mips verify\n"
	   "h713_mips start\n"
	   "h713_mips load <interface> <dev[:part]> <path>\n"
	   "h713_mips boot <interface> <dev[:part]> <path>"
);
