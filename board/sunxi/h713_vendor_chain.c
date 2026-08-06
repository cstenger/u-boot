// SPDX-License-Identifier: GPL-2.0+
/*
 * H713 one-shot vendor chainload.
 *
 * The board carries two complete boot chains, but the BROM reads a first stage
 * from LBA 0x10 and nowhere else, so whichever one is installed there owns
 * every boot after it.  This makes ours the permanent default and the vendor's
 * an excursion that expires by itself: when the RTC marker says so, the SPL
 * loads the vendor's boot0 from LBA 0x100 -- the vendor's own second copy,
 * which tools/boot-switch.sh restores -- and hands the machine to it.
 *
 * Set the marker with "run boot_vendor" (or from Linux, whatever writes the
 * same RTC word), then reset.  The marker is consumed before anything else can
 * fail, so a chainload that hangs comes back to our own U-Boot on the next
 * power-on rather than repeating.  Any failure here just returns and boots us
 * normally; nothing is written to eMMC either way.
 *
 * See docs/flash.md "Switching stacks" for what is proven and what is not.
 */

#include <blk.h>
#include <log.h>
#include <mmc.h>
#include <asm/arch/cpu.h>
#include <asm/arch/spl.h>
#include <asm/cache.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <linux/string.h>

DECLARE_GLOBAL_DATA_PTR;

struct mmc *h713_chain_emmc(void);		/* board/sunxi/board.c */
void h713_chain_tramp(unsigned long src, unsigned long dst,
		      unsigned long len, unsigned long entry,
		      unsigned long rvbar);
extern char h713_chain_tramp_end[];

#define RTC_GP7			0x0709011cUL	/* survives reset; also carries
						 * the fastboot/prompt markers */
#define MARKER_VENDOR		0x001db007	/* "old boot" */

#define VENDOR_BOOT0_LBA	0x100		/* the vendor's own second copy */
#define BOOT0_BLOCKS		64		/* 32 KiB */
#define BOOT0_BYTES		(BOOT0_BLOCKS * 512)
#define BOOT0_LOAD		0x00104000UL	/* from the image header, +0x1c */

#define STAGE			0x4a000000UL	/* DRAM, clear of the SPL */
#define TRAMP			0x4a010000UL	/* must not be in the copy target */
#define MALLOC_BASE		0x48000000UL
#define MALLOC_LEN		0x00800000UL

#define EMMC_MIN_BLOCKS		0x200000UL	/* 1 GiB, to tell eMMC from SD */
#define EGON_STAMP		0x5f0a6c39

/*
 * Never hand off to a first stage that fails its own checksum: the vendor's
 * boot0 is 32 KiB of AArch32 that runs before anything can report a problem,
 * and a bad one costs a FEL trip.  This is the same check the BROM's header
 * describes -- sum every word with the checksum field replaced by the stamp.
 */
static int egon_valid(const u32 *img)
{
	u32 sum = 0;
	unsigned int i;

	if (memcmp(&img[1], "eGON.BT0", 8))
		return 0;
	if (img[4] != BOOT0_BYTES)
		return 0;

	for (i = 0; i < BOOT0_BYTES / 4; i++)
		sum += (i == 3) ? EGON_STAMP : img[i];

	return sum == img[3];
}

void h713_vendor_chainload(void)
{
	void (*tramp)(unsigned long, unsigned long, unsigned long,
		      unsigned long, unsigned long);
	unsigned long tramp_len, rvbar;
	struct blk_desc *bd;
	struct mmc *mmc;
	unsigned long n;
	u8 media;

	if (readl(RTC_GP7) != MARKER_VENDOR)
		return;

	/* One shot: consume the marker before anything here can fail. */
	writel(0, RTC_GP7);

	printf("\n=== H713 VENDOR CHAINLOAD ===\n");

	if (current_el() != 3) {
		printf("not at EL3, cannot enter AArch32 - booting ours\n");
		return;
	}

	/*
	 * We run in board_init_f, before the SPL sets up its heap, so the
	 * simple-malloc pool is still empty.  Point it at DRAM so that
	 * mmc_initialize() can allocate.  The SPL reinitialises this later,
	 * so returning from here is harmless.
	 */
	gd->malloc_base = MALLOC_BASE;
	gd->malloc_ptr = 0;
	gd->malloc_limit = MALLOC_LEN;

	if (mmc_initialize(NULL)) {
		printf("mmc_initialize fail - booting ours\n");
		return;
	}

	mmc = h713_chain_emmc();
	if (!mmc || mmc_init(mmc)) {
		printf("eMMC init fail - booting ours\n");
		return;
	}

	bd = mmc_get_blk_desc(mmc);
	if (bd->lba < EMMC_MIN_BLOCKS) {
		printf("device too small to be the eMMC - booting ours\n");
		return;
	}

	n = blk_dread(bd, VENDOR_BOOT0_LBA, BOOT0_BLOCKS, (void *)STAGE);
	if (n != BOOT0_BLOCKS) {
		printf("read %lu/%u blocks at LBA 0x%x - booting ours\n",
		       n, BOOT0_BLOCKS, VENDOR_BOOT0_LBA);
		return;
	}

	if (!egon_valid((const u32 *)STAGE)) {
		printf("no valid vendor boot0 at LBA 0x%x "
		       "(run boot-switch.sh stage) - booting ours\n",
		       VENDOR_BOOT0_LBA);
		return;
	}

	/*
	 * The BROM tells a first stage which device it came from by writing
	 * boot_media into the header of the copy it loaded into SRAM -- it is
	 * not part of the image on disk.  A pristine boot0 therefore reads it
	 * as 0 and hunts for its boot package on MMC0, the absent SD slot
	 * ("Wrong media type 0x0" / "Try SD card 0" / "Loading boot-pkg
	 * fail(error=2)").  Our own header still holds the byte the BROM wrote
	 * for us, so pass that on rather than hardcode a media code.
	 *
	 * After the checksum check, deliberately: the BROM patches this field
	 * the same way, after its own verification, and nothing re-checks it.
	 */
	media = readb(&((struct boot_file_head *)SPL_ADDR)->boot_media);
	writeb(media, &((struct boot_file_head *)STAGE)->boot_media);

	printf("vendor boot0 verified, boot_media 0x%x, entering AArch32 at 0x%x\n",
	       media, (unsigned int)BOOT0_LOAD);

	/*
	 * From here the SPL is about to overwrite itself, so there is no way
	 * back short of a power cycle.  Caches off first: the trampoline copy
	 * and the code it lands on must both be visible to an AArch32 core
	 * that starts with the MMU and caches disabled.
	 */
	tramp_len = (unsigned long)h713_chain_tramp_end -
		    (unsigned long)h713_chain_tramp;
	memcpy((void *)TRAMP, (void *)h713_chain_tramp, tramp_len);

	dcache_disable();
	invalidate_icache_all();

	/*
	 * Same alias selection rmr_switch.S uses on the way out to AArch64:
	 * a non-zero SRAM_VER_REG[7:0] means the SoC needs the alternative
	 * address.
	 */
	rvbar = (readl(SUNXI_SRAMC_BASE + 0x24) & 0xff) ?
		CONFIG_SUNXI_RVBAR_ALTERNATIVE : CONFIG_SUNXI_RVBAR_ADDRESS;

	tramp = (void *)TRAMP;
	tramp(STAGE, BOOT0_LOAD, BOOT0_BYTES, BOOT0_LOAD, rvbar);

	/* not reached */
	printf("chainload returned - booting ours\n");
}
