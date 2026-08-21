// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Zumapro camera back-end core bring-up driver
 *
 * The register sequences here are the common processor lifecycle observed on
 * the Pixel 9a vendor stack.  The debugfs diagnostic accepts a structurally
 * fixed, relocatable RGBP/YUVP program for offline bring-up; it is deliberately
 * not a camera ABI.  Separate GTNR-startup and MCSC recipes can be normalized
 * into dormant command lists while those downstream stages are brought up.
 * Powering a block down is safe only after every owned processor has accepted
 * a software reset.
 */

#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/scatterlist.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include <media/exynos-becore.h>
#include <media/media-device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

#include "exynos-becore-recipe.h"
#include "exynos-becore-gtnr-recipe.h"
#include "exynos-becore-mcsc-recipe.h"

#define BECORE_GLOBAL_ENABLE		0x0000
#define BECORE_GLOBAL_ENABLE_CLEAR	0x0008
#define BECORE_SW_RESET			0x0010
#define BECORE_SET_CTRL			0x0030
#define BECORE_COMMAND_Q_STOP_ON_FRAME	0x003c
#define BECORE_FRO_GLOBAL_ENABLE		0x006c

#define BECORE_CMDQ_QUE_CMD_L		0x0400
#define BECORE_CMDQ_QUE_CMD_M		0x0404
#define BECORE_CMDQ_QUE_CMD_H		0x0408
#define BECORE_CMDQ_QUE_CMD_START	0x040c
#define BECORE_CMDQ_ADD_TO_QUEUE_0	0x0484
#define BECORE_CMDQ_INT_STATUS		0x04f8
#define BECORE_CMDQ_INT_CLEAR		0x04fc
#define BECORE_CMDQ_INT_ENABLE		0x04f4
#define BECORE_CMDQ_ENABLE		0x0500

#define BECORE_INT0_ENABLE		0x0804
#define BECORE_INT0_STATUS		0x0808
#define BECORE_INT0_CLEAR		0x080c
#define BECORE_INT1_ENABLE		0x0814
#define BECORE_INT1_STATUS		0x0818
#define BECORE_INT1_CLEAR		0x081c

#define BECORE_C_LOADER_ENABLE		0x1000
#define BECORE_C_LOADER_MODE		0x1004
#define BECORE_STAT_RDMACL_EN		0x1600
#define BECORE_RGBP_C_LOADER_ENABLE	0x4000

#define BECORE_RESET_TIMEOUT_US		1000

#define BECORE_INT_FRAME_END		BIT(1)
#define BECORE_INT_CMDQ_HOLD		BIT(2)
#define BECORE_INT_EXPECTED		(BECORE_INT_FRAME_END | BECORE_INT_CMDQ_HOLD)
#define BECORE_YUVP_STAGE_BLOCKS	(BIT(BECORE_RGBP) | BIT(BECORE_YUVP))

#define BECORE_CMDQ_HEADER_BYTES		16
#define BECORE_CMDQ_PAYLOAD_BYTES	64
#define BECORE_CMDQ_MODE		0x9000

#define BECORE_GRID_SIZE			0x18000
#define BECORE_RUN_TIMEOUT_MS		1000

#define BECORE_RGBP_PHYS_BASE		0x1c440000
#define BECORE_YUVP_PHYS_BASE		0x1c840000
#define BECORE_GTNR_PHYS_BASE		0x1cc40000
#define BECORE_MCSC_PHYS_BASE		0x1d040000

#define BECORE_RGBP_INPUT_IMAGE_REG	(BECORE_RGBP_PHYS_BASE + 0x1c50)
#define BECORE_RGBP_INPUT_HEADER_REG	(BECORE_RGBP_PHYS_BASE + 0x1d10)
#define BECORE_RGBP_INPUT_ENABLE_REG	(BECORE_RGBP_PHYS_BASE + 0x1c00)
#define BECORE_RGBP_INPUT_COMP_REG	(BECORE_RGBP_PHYS_BASE + 0x1c04)
#define BECORE_RGBP_INPUT_FORMAT_REG	(BECORE_RGBP_PHYS_BASE + 0x1c10)
#define BECORE_RGBP_INPUT_WIDTH_REG	(BECORE_RGBP_PHYS_BASE + 0x1c20)
#define BECORE_RGBP_INPUT_HEIGHT_REG	(BECORE_RGBP_PHYS_BASE + 0x1c24)
#define BECORE_RGBP_INPUT_STRIDE_REG	(BECORE_RGBP_PHYS_BASE + 0x1c28)
#define BECORE_RGBP_INPUT_HEADER_STRIDE_REG \
	(BECORE_RGBP_PHYS_BASE + 0x1c34)
#define BECORE_RGBP_INPUT_BUSINFO_REG	(BECORE_RGBP_PHYS_BASE + 0x1c4c)
#define BECORE_YUVP_GRID_REG		(BECORE_YUVP_PHYS_BASE + 0x1c50)
#define BECORE_YUVP_OUTPUT_PLANE1_REG	(BECORE_YUVP_PHYS_BASE + 0x2450)
#define BECORE_YUVP_OUTPUT_PLANE2_REG	(BECORE_YUVP_PHYS_BASE + 0x2490)
#define BECORE_YUVP_OUTPUT_ENABLE_REG	(BECORE_YUVP_PHYS_BASE + 0x2400)
#define BECORE_YUVP_OUTPUT_MODE_REG	(BECORE_YUVP_PHYS_BASE + 0x2404)
#define BECORE_YUVP_OUTPUT_FORMAT_REG	(BECORE_YUVP_PHYS_BASE + 0x2410)
#define BECORE_YUVP_OUTPUT_LOSSY_REG	(BECORE_YUVP_PHYS_BASE + 0x2418)
#define BECORE_YUVP_OUTPUT_WIDTH_REG	(BECORE_YUVP_PHYS_BASE + 0x2420)
#define BECORE_YUVP_OUTPUT_HEIGHT_REG	(BECORE_YUVP_PHYS_BASE + 0x2424)
#define BECORE_YUVP_OUTPUT_STRIDE1_REG	(BECORE_YUVP_PHYS_BASE + 0x2428)
#define BECORE_YUVP_OUTPUT_STRIDE2_REG	(BECORE_YUVP_PHYS_BASE + 0x242c)
#define BECORE_YUVP_OUTPUT_VOTF_REG	(BECORE_YUVP_PHYS_BASE + 0x243c)
#define BECORE_YUVP_OUTPUT_BUSINFO_REG	(BECORE_YUVP_PHYS_BASE + 0x244c)

#define BECORE_GTNR_INPUT_PLANE1_REG	(BECORE_GTNR_PHYS_BASE + 0x1e50)
#define BECORE_GTNR_INPUT_PLANE2_REG	(BECORE_GTNR_PHYS_BASE + 0x1e90)
#define BECORE_GTNR_INPUT_VOTF_REG	(BECORE_GTNR_PHYS_BASE + 0x1e3c)
#define BECORE_GTNR_INPUT_FORMAT_REG	(BECORE_GTNR_PHYS_BASE + 0x1e10)
#define BECORE_GTNR_INPUT_LOSSY_REG	(BECORE_GTNR_PHYS_BASE + 0x1e18)
#define BECORE_GTNR_INPUT_COMP_REG	(BECORE_GTNR_PHYS_BASE + 0x1e04)
#define BECORE_GTNR_INPUT_WIDTH_REG	(BECORE_GTNR_PHYS_BASE + 0x1e20)
#define BECORE_GTNR_INPUT_HEIGHT_REG	(BECORE_GTNR_PHYS_BASE + 0x1e24)
#define BECORE_GTNR_INPUT_STRIDE1_REG	(BECORE_GTNR_PHYS_BASE + 0x1e28)
#define BECORE_GTNR_INPUT_STRIDE2_REG	(BECORE_GTNR_PHYS_BASE + 0x1e2c)
#define BECORE_GTNR_INPUT_BUSINFO_REG	(BECORE_GTNR_PHYS_BASE + 0x1e4c)
#define BECORE_GTNR_INPUT_MAX_MO_REG	(BECORE_GTNR_PHYS_BASE + 0x1e40)
#define BECORE_GTNR_INPUT_MAX_BL_REG	(BECORE_GTNR_PHYS_BASE + 0x1e48)
#define BECORE_GTNR_INPUT_ENABLE_REG	(BECORE_GTNR_PHYS_BASE + 0x1e00)
#define BECORE_GTNR_OUTPUT_PLANE1_REG	(BECORE_GTNR_PHYS_BASE + 0x3050)
#define BECORE_GTNR_OUTPUT_PLANE2_REG	(BECORE_GTNR_PHYS_BASE + 0x3090)
#define BECORE_GTNR_OUTPUT_FORMAT_REG	(BECORE_GTNR_PHYS_BASE + 0x3010)
#define BECORE_GTNR_OUTPUT_LOSSY_REG	(BECORE_GTNR_PHYS_BASE + 0x3018)
#define BECORE_GTNR_OUTPUT_COMP_REG	(BECORE_GTNR_PHYS_BASE + 0x3004)
#define BECORE_GTNR_OUTPUT_WIDTH_REG	(BECORE_GTNR_PHYS_BASE + 0x3020)
#define BECORE_GTNR_OUTPUT_HEIGHT_REG	(BECORE_GTNR_PHYS_BASE + 0x3024)
#define BECORE_GTNR_OUTPUT_STRIDE1_REG	(BECORE_GTNR_PHYS_BASE + 0x3028)
#define BECORE_GTNR_OUTPUT_STRIDE2_REG	(BECORE_GTNR_PHYS_BASE + 0x302c)
#define BECORE_GTNR_OUTPUT_BUSINFO_REG	(BECORE_GTNR_PHYS_BASE + 0x304c)
#define BECORE_GTNR_OUTPUT_MAX_MO_REG	(BECORE_GTNR_PHYS_BASE + 0x3040)
#define BECORE_GTNR_OUTPUT_MAX_BL_REG	(BECORE_GTNR_PHYS_BASE + 0x3048)
#define BECORE_GTNR_OUTPUT_ENABLE_REG	(BECORE_GTNR_PHYS_BASE + 0x3000)

#define BECORE_MCSC_INPUT_PLANE1_REG	(BECORE_MCSC_PHYS_BASE + 0x1850)
#define BECORE_MCSC_INPUT_PLANE2_REG	(BECORE_MCSC_PHYS_BASE + 0x1890)
#define BECORE_MCSC_INPUT_VOTF_REG	(BECORE_MCSC_PHYS_BASE + 0x183c)
#define BECORE_MCSC_INPUT_FORMAT_REG	(BECORE_MCSC_PHYS_BASE + 0x1810)
#define BECORE_MCSC_INPUT_LOSSY_REG	(BECORE_MCSC_PHYS_BASE + 0x1818)
#define BECORE_MCSC_INPUT_COMP_REG	(BECORE_MCSC_PHYS_BASE + 0x1804)
#define BECORE_MCSC_INPUT_WIDTH_REG	(BECORE_MCSC_PHYS_BASE + 0x1820)
#define BECORE_MCSC_INPUT_HEIGHT_REG	(BECORE_MCSC_PHYS_BASE + 0x1824)
#define BECORE_MCSC_INPUT_STRIDE1_REG	(BECORE_MCSC_PHYS_BASE + 0x1828)
#define BECORE_MCSC_INPUT_STRIDE2_REG	(BECORE_MCSC_PHYS_BASE + 0x182c)
#define BECORE_MCSC_INPUT_BUSINFO_REG	(BECORE_MCSC_PHYS_BASE + 0x184c)
#define BECORE_MCSC_INPUT_MAX_BL_REG	(BECORE_MCSC_PHYS_BASE + 0x1848)
#define BECORE_MCSC_INPUT_ENABLE_REG	(BECORE_MCSC_PHYS_BASE + 0x1800)
#define BECORE_MCSC_OUTPUT_PLANE1_REG	(BECORE_MCSC_PHYS_BASE + 0x2050)
#define BECORE_MCSC_OUTPUT_PLANE2_REG	(BECORE_MCSC_PHYS_BASE + 0x2090)
#define BECORE_MCSC_OUTPUT_FORMAT_REG	(BECORE_MCSC_PHYS_BASE + 0x2010)
#define BECORE_MCSC_OUTPUT_COMP_REG	(BECORE_MCSC_PHYS_BASE + 0x2004)
#define BECORE_MCSC_OUTPUT_WIDTH_REG	(BECORE_MCSC_PHYS_BASE + 0x2020)
#define BECORE_MCSC_OUTPUT_HEIGHT_REG	(BECORE_MCSC_PHYS_BASE + 0x2024)
#define BECORE_MCSC_OUTPUT_STRIDE1_REG	(BECORE_MCSC_PHYS_BASE + 0x2028)
#define BECORE_MCSC_OUTPUT_STRIDE2_REG	(BECORE_MCSC_PHYS_BASE + 0x202c)
#define BECORE_MCSC_OUTPUT_BUSINFO_REG	(BECORE_MCSC_PHYS_BASE + 0x204c)
#define BECORE_MCSC_OUTPUT_MAX_BL_REG	(BECORE_MCSC_PHYS_BASE + 0x2048)
#define BECORE_MCSC_OUTPUT_ENABLE_REG	(BECORE_MCSC_PHYS_BASE + 0x2000)
#define BECORE_MCSC_OUTPUT_DITHER_REG	(BECORE_MCSC_PHYS_BASE + 0x2f00)

enum becore_block_id {
	BECORE_RGBP,
	BECORE_MCFP,
	BECORE_YUVP,
	BECORE_MCSC,
	BECORE_NUM_BLOCKS,
};

struct becore_regval {
	u32 offset;
	u32 value;
};

struct becore_rgbp_input_profile {
	u32 width;
	u32 height;
	u32 data_format;
	u32 comp_control;
	u32 sbwc_block_width;
	u32 bytes_per_pixel;
	u32 header_stride;
	u32 businfo;
};

/*
 * The active dimensions and DMA fields are from the live ultrawide program.
 * As in Pablo's common DMA API, the payload and header geometry are derived
 * from the image profile.  Lyric additionally writes the 256-pixel-aligned
 * SBWC storage width after enabling the RDMA.
 */
static const struct becore_rgbp_input_profile becore_rgbp_input = {
	.width = 4208,
	.height = 3120,
	.data_format = 0x18,
	.comp_control = 0x9,
	.sbwc_block_width = 256,
	.bytes_per_pixel = 2,
	.header_stride = 0x40,
	.businfo = 0,
};

enum becore_rgbp_input_word {
	BECORE_RGBP_INPUT_FORMAT,
	BECORE_RGBP_INPUT_COMP,
	BECORE_RGBP_INPUT_ACTIVE_WIDTH,
	BECORE_RGBP_INPUT_HEIGHT,
	BECORE_RGBP_INPUT_STRIDE,
	BECORE_RGBP_INPUT_HEADER_STRIDE,
	BECORE_RGBP_INPUT_BUSINFO,
	BECORE_RGBP_INPUT_ENABLE,
	BECORE_RGBP_INPUT_STORAGE_WIDTH,
	BECORE_RGBP_INPUT_WORD_COUNT,
};

static const u32 becore_rgbp_input_regs[] = {
	[BECORE_RGBP_INPUT_FORMAT] = BECORE_RGBP_INPUT_FORMAT_REG,
	[BECORE_RGBP_INPUT_COMP] = BECORE_RGBP_INPUT_COMP_REG,
	[BECORE_RGBP_INPUT_ACTIVE_WIDTH] = BECORE_RGBP_INPUT_WIDTH_REG,
	[BECORE_RGBP_INPUT_HEIGHT] = BECORE_RGBP_INPUT_HEIGHT_REG,
	[BECORE_RGBP_INPUT_STRIDE] = BECORE_RGBP_INPUT_STRIDE_REG,
	[BECORE_RGBP_INPUT_HEADER_STRIDE] = BECORE_RGBP_INPUT_HEADER_STRIDE_REG,
	[BECORE_RGBP_INPUT_BUSINFO] = BECORE_RGBP_INPUT_BUSINFO_REG,
	[BECORE_RGBP_INPUT_ENABLE] = BECORE_RGBP_INPUT_ENABLE_REG,
	[BECORE_RGBP_INPUT_STORAGE_WIDTH] = BECORE_RGBP_INPUT_WIDTH_REG,
};

struct becore_yuvp_output_profile {
	u32 width;
	u32 height;
	u32 data_format;
	u32 mode;
	u32 lossy_byte32num;
	u32 votf_enable;
	u32 bytes_per_pixel;
	u32 block_height;
	u32 luma_height_align;
	u32 plane_gap;
	u32 businfo;
};

enum becore_yuvp_output_profile_id {
	BECORE_YUVP_OUTPUT_SBWCL,
	BECORE_YUVP_OUTPUT_P010,
	BECORE_YUVP_OUTPUT_PROFILE_COUNT,
};

/* Pixel uses a combined, two-plane WDMA where Pablo v1.1 uses split blocks. */
static const struct becore_yuvp_output_profile becore_yuvp_outputs[] = {
	[BECORE_YUVP_OUTPUT_SBWCL] = {
		.width = 4160,
		.height = 3120,
		.data_format = 0x2000,
		.mode = 0xa,
		.lossy_byte32num = 2,
		.votf_enable = 3,
		.bytes_per_pixel = 2,
		.block_height = 4,
		.luma_height_align = 16,
		.plane_gap = 0x40,
		.businfo = 0,
	},
	[BECORE_YUVP_OUTPUT_P010] = {
		.width = 4160,
		.height = 3120,
		.data_format = 0x2000,
		.mode = 0,
		.lossy_byte32num = 0,
		.votf_enable = 0,
		.bytes_per_pixel = 2,
		.businfo = 0,
	},
};

enum becore_yuvp_output_word {
	BECORE_YUVP_OUTPUT_VOTF,
	BECORE_YUVP_OUTPUT_FORMAT,
	BECORE_YUVP_OUTPUT_LOSSY,
	BECORE_YUVP_OUTPUT_MODE,
	BECORE_YUVP_OUTPUT_WIDTH,
	BECORE_YUVP_OUTPUT_HEIGHT,
	BECORE_YUVP_OUTPUT_STRIDE1,
	BECORE_YUVP_OUTPUT_STRIDE2,
	BECORE_YUVP_OUTPUT_BUSINFO,
	BECORE_YUVP_OUTPUT_ENABLE,
	BECORE_YUVP_OUTPUT_WORD_COUNT,
};

static const u32 becore_yuvp_output_regs[] = {
	[BECORE_YUVP_OUTPUT_VOTF] = BECORE_YUVP_OUTPUT_VOTF_REG,
	[BECORE_YUVP_OUTPUT_FORMAT] = BECORE_YUVP_OUTPUT_FORMAT_REG,
	[BECORE_YUVP_OUTPUT_LOSSY] = BECORE_YUVP_OUTPUT_LOSSY_REG,
	[BECORE_YUVP_OUTPUT_MODE] = BECORE_YUVP_OUTPUT_MODE_REG,
	[BECORE_YUVP_OUTPUT_WIDTH] = BECORE_YUVP_OUTPUT_WIDTH_REG,
	[BECORE_YUVP_OUTPUT_HEIGHT] = BECORE_YUVP_OUTPUT_HEIGHT_REG,
	[BECORE_YUVP_OUTPUT_STRIDE1] = BECORE_YUVP_OUTPUT_STRIDE1_REG,
	[BECORE_YUVP_OUTPUT_STRIDE2] = BECORE_YUVP_OUTPUT_STRIDE2_REG,
	[BECORE_YUVP_OUTPUT_BUSINFO] = BECORE_YUVP_OUTPUT_BUSINFO_REG,
	[BECORE_YUVP_OUTPUT_ENABLE] = BECORE_YUVP_OUTPUT_ENABLE_REG,
};

struct becore_gtnr_dma_profile {
	u32 width;
	u32 height;
	u32 data_format;
	u32 comp_control;
	u32 lossy_byte32num;
	u32 votf_enable;
	u32 stride;
	u32 businfo;
	u32 max_mo;
	u32 max_bl;
	u32 enable;
};

/*
 * The first physical-ultrawide GTNR frame has no temporal inputs or map DMAs.
 * It reads YUVP's 4160x3120 lossy-SBWC surface and writes a separate surface
 * with the same bounded layout.  Keep this startup shape dormant until GTNR's
 * power, reset, interrupt, and cross-block completion lifecycle is established.
 */
static const struct becore_gtnr_dma_profile becore_gtnr_input = {
	.width = 4160,
	.height = 3120,
	.data_format = 0x2000,
	.comp_control = 0xa,
	.lossy_byte32num = 2,
	.votf_enable = 1,
	.stride = 0x2080,
	.businfo = 1,
	.max_mo = 0x100,
	.max_bl = 0x10,
	.enable = 1,
};

static const struct becore_gtnr_dma_profile becore_gtnr_output = {
	.width = 4160,
	.height = 3120,
	.data_format = 0x2000,
	.comp_control = 0xa,
	.lossy_byte32num = 2,
	.stride = 0x2080,
	.businfo = 0,
	.max_mo = 0x100,
	.max_bl = 0x10,
	.enable = 1,
};

enum becore_gtnr_dma_word {
	BECORE_GTNR_INPUT_VOTF,
	BECORE_GTNR_INPUT_FORMAT,
	BECORE_GTNR_INPUT_LOSSY,
	BECORE_GTNR_INPUT_COMP,
	BECORE_GTNR_INPUT_WIDTH,
	BECORE_GTNR_INPUT_HEIGHT,
	BECORE_GTNR_INPUT_STRIDE1,
	BECORE_GTNR_INPUT_STRIDE2,
	BECORE_GTNR_INPUT_BUSINFO,
	BECORE_GTNR_INPUT_MAX_MO,
	BECORE_GTNR_INPUT_MAX_BL,
	BECORE_GTNR_INPUT_ENABLE,
	BECORE_GTNR_OUTPUT_FORMAT,
	BECORE_GTNR_OUTPUT_LOSSY,
	BECORE_GTNR_OUTPUT_COMP,
	BECORE_GTNR_OUTPUT_WIDTH,
	BECORE_GTNR_OUTPUT_HEIGHT,
	BECORE_GTNR_OUTPUT_STRIDE1,
	BECORE_GTNR_OUTPUT_STRIDE2,
	BECORE_GTNR_OUTPUT_BUSINFO,
	BECORE_GTNR_OUTPUT_MAX_MO,
	BECORE_GTNR_OUTPUT_MAX_BL,
	BECORE_GTNR_OUTPUT_ENABLE,
	BECORE_GTNR_DMA_WORD_COUNT,
};

static const u32 becore_gtnr_dma_regs[] = {
	[BECORE_GTNR_INPUT_VOTF] = BECORE_GTNR_INPUT_VOTF_REG,
	[BECORE_GTNR_INPUT_FORMAT] = BECORE_GTNR_INPUT_FORMAT_REG,
	[BECORE_GTNR_INPUT_LOSSY] = BECORE_GTNR_INPUT_LOSSY_REG,
	[BECORE_GTNR_INPUT_COMP] = BECORE_GTNR_INPUT_COMP_REG,
	[BECORE_GTNR_INPUT_WIDTH] = BECORE_GTNR_INPUT_WIDTH_REG,
	[BECORE_GTNR_INPUT_HEIGHT] = BECORE_GTNR_INPUT_HEIGHT_REG,
	[BECORE_GTNR_INPUT_STRIDE1] = BECORE_GTNR_INPUT_STRIDE1_REG,
	[BECORE_GTNR_INPUT_STRIDE2] = BECORE_GTNR_INPUT_STRIDE2_REG,
	[BECORE_GTNR_INPUT_BUSINFO] = BECORE_GTNR_INPUT_BUSINFO_REG,
	[BECORE_GTNR_INPUT_MAX_MO] = BECORE_GTNR_INPUT_MAX_MO_REG,
	[BECORE_GTNR_INPUT_MAX_BL] = BECORE_GTNR_INPUT_MAX_BL_REG,
	[BECORE_GTNR_INPUT_ENABLE] = BECORE_GTNR_INPUT_ENABLE_REG,
	[BECORE_GTNR_OUTPUT_FORMAT] = BECORE_GTNR_OUTPUT_FORMAT_REG,
	[BECORE_GTNR_OUTPUT_LOSSY] = BECORE_GTNR_OUTPUT_LOSSY_REG,
	[BECORE_GTNR_OUTPUT_COMP] = BECORE_GTNR_OUTPUT_COMP_REG,
	[BECORE_GTNR_OUTPUT_WIDTH] = BECORE_GTNR_OUTPUT_WIDTH_REG,
	[BECORE_GTNR_OUTPUT_HEIGHT] = BECORE_GTNR_OUTPUT_HEIGHT_REG,
	[BECORE_GTNR_OUTPUT_STRIDE1] = BECORE_GTNR_OUTPUT_STRIDE1_REG,
	[BECORE_GTNR_OUTPUT_STRIDE2] = BECORE_GTNR_OUTPUT_STRIDE2_REG,
	[BECORE_GTNR_OUTPUT_BUSINFO] = BECORE_GTNR_OUTPUT_BUSINFO_REG,
	[BECORE_GTNR_OUTPUT_MAX_MO] = BECORE_GTNR_OUTPUT_MAX_MO_REG,
	[BECORE_GTNR_OUTPUT_MAX_BL] = BECORE_GTNR_OUTPUT_MAX_BL_REG,
	[BECORE_GTNR_OUTPUT_ENABLE] = BECORE_GTNR_OUTPUT_ENABLE_REG,
};

struct becore_mcsc_dma_profile {
	u32 width;
	u32 height;
	u32 data_format;
	u32 comp_control;
	u32 lossy_byte32num;
	u32 votf_enable;
	u32 stride;
	u32 businfo;
	u32 max_bl;
	u32 enable;
	u32 dither;
};

/*
 * The physical-ultrawide 4000x3000 request carries YUVP's 4160x3120 lossy
 * SBWC intermediate into MCSC and writes output zero as linear NV21.  The
 * captured program used VOTF; a sequential run replaces only that transport
 * control so MCSC reads the completed driver-owned surface from memory.
 */
static const struct becore_mcsc_dma_profile becore_mcsc_input = {
	.width = 4160,
	.height = 3120,
	.data_format = 0x2000,
	.comp_control = 0xa,
	.lossy_byte32num = 2,
	.votf_enable = 0x00400001,
	.stride = 0x2080,
	.businfo = 2,
	.max_bl = 0x10,
	.enable = 1,
};

static const struct becore_mcsc_dma_profile becore_mcsc_output = {
	.width = 4000,
	.height = 3000,
	.data_format = 0x800,
	.comp_control = 0,
	.stride = 0xfc0,
	.businfo = 0,
	.max_bl = 4,
	.enable = 1,
	.dither = 0x10,
};

enum becore_mcsc_input_transport {
	BECORE_MCSC_INPUT_CAPTURED_VOTF,
	BECORE_MCSC_INPUT_MEMORY,
};

enum becore_mcsc_dma_word {
	BECORE_MCSC_INPUT_VOTF,
	BECORE_MCSC_INPUT_FORMAT,
	BECORE_MCSC_INPUT_LOSSY,
	BECORE_MCSC_INPUT_COMP,
	BECORE_MCSC_INPUT_WIDTH,
	BECORE_MCSC_INPUT_HEIGHT,
	BECORE_MCSC_INPUT_STRIDE1,
	BECORE_MCSC_INPUT_STRIDE2,
	BECORE_MCSC_INPUT_BUSINFO,
	BECORE_MCSC_INPUT_MAX_BL,
	BECORE_MCSC_INPUT_ENABLE,
	BECORE_MCSC_OUTPUT_FORMAT,
	BECORE_MCSC_OUTPUT_COMP,
	BECORE_MCSC_OUTPUT_WIDTH,
	BECORE_MCSC_OUTPUT_HEIGHT,
	BECORE_MCSC_OUTPUT_STRIDE1,
	BECORE_MCSC_OUTPUT_STRIDE2,
	BECORE_MCSC_OUTPUT_BUSINFO,
	BECORE_MCSC_OUTPUT_MAX_BL,
	BECORE_MCSC_OUTPUT_ENABLE,
	BECORE_MCSC_OUTPUT_DITHER,
	BECORE_MCSC_DMA_WORD_COUNT,
};

static const u32 becore_mcsc_dma_regs[] = {
	[BECORE_MCSC_INPUT_VOTF] = BECORE_MCSC_INPUT_VOTF_REG,
	[BECORE_MCSC_INPUT_FORMAT] = BECORE_MCSC_INPUT_FORMAT_REG,
	[BECORE_MCSC_INPUT_LOSSY] = BECORE_MCSC_INPUT_LOSSY_REG,
	[BECORE_MCSC_INPUT_COMP] = BECORE_MCSC_INPUT_COMP_REG,
	[BECORE_MCSC_INPUT_WIDTH] = BECORE_MCSC_INPUT_WIDTH_REG,
	[BECORE_MCSC_INPUT_HEIGHT] = BECORE_MCSC_INPUT_HEIGHT_REG,
	[BECORE_MCSC_INPUT_STRIDE1] = BECORE_MCSC_INPUT_STRIDE1_REG,
	[BECORE_MCSC_INPUT_STRIDE2] = BECORE_MCSC_INPUT_STRIDE2_REG,
	[BECORE_MCSC_INPUT_BUSINFO] = BECORE_MCSC_INPUT_BUSINFO_REG,
	[BECORE_MCSC_INPUT_MAX_BL] = BECORE_MCSC_INPUT_MAX_BL_REG,
	[BECORE_MCSC_INPUT_ENABLE] = BECORE_MCSC_INPUT_ENABLE_REG,
	[BECORE_MCSC_OUTPUT_FORMAT] = BECORE_MCSC_OUTPUT_FORMAT_REG,
	[BECORE_MCSC_OUTPUT_COMP] = BECORE_MCSC_OUTPUT_COMP_REG,
	[BECORE_MCSC_OUTPUT_WIDTH] = BECORE_MCSC_OUTPUT_WIDTH_REG,
	[BECORE_MCSC_OUTPUT_HEIGHT] = BECORE_MCSC_OUTPUT_HEIGHT_REG,
	[BECORE_MCSC_OUTPUT_STRIDE1] = BECORE_MCSC_OUTPUT_STRIDE1_REG,
	[BECORE_MCSC_OUTPUT_STRIDE2] = BECORE_MCSC_OUTPUT_STRIDE2_REG,
	[BECORE_MCSC_OUTPUT_BUSINFO] = BECORE_MCSC_OUTPUT_BUSINFO_REG,
	[BECORE_MCSC_OUTPUT_MAX_BL] = BECORE_MCSC_OUTPUT_MAX_BL_REG,
	[BECORE_MCSC_OUTPUT_ENABLE] = BECORE_MCSC_OUTPUT_ENABLE_REG,
	[BECORE_MCSC_OUTPUT_DITHER] = BECORE_MCSC_OUTPUT_DITHER_REG,
};

struct becore_device;

#define BECORE_INPUT_SLOT_COUNT	3

struct becore_dma_buffer {
	void *cpu;
	dma_addr_t dma;
	size_t size;
	size_t staged_bytes;
	struct sg_table *sgt;
};

enum becore_input_slot_state {
	BECORE_INPUT_FREE,
	BECORE_INPUT_PRODUCER,
	BECORE_INPUT_READY,
	BECORE_INPUT_BACKEND,
	BECORE_INPUT_QUARANTINED,
};

struct becore_input_slot {
	struct becore_dma_buffer buffer;
	enum becore_input_slot_state state;
	u64 producer_cookie;
	u64 ready_sequence;
};

struct becore_cmdq_program {
	void *cpu;
	dma_addr_t dma;
	size_t size;
	u32 header_count;
};

struct becore_block {
	struct becore_device *becore;
	const char *name;
	void __iomem *base;
	u32 int0_mask_prepare;
	u32 int0_mask;
	u32 int1_mask;
	u32 cmdq_int_mask;
	u32 last_int0;
	u32 last_int1;
	u32 last_cmdq_int;
	atomic64_t int0_count;
	atomic64_t int1_count;
};

struct becore_irq {
	struct becore_block *block;
	int irq;
	bool int1;
};

struct becore_video_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

static inline struct becore_video_buffer *
to_becore_video_buffer(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct becore_video_buffer, vb);
}

struct becore_device {
	struct device *dev;
	struct becore_block blocks[BECORE_NUM_BLOCKS];
	struct becore_irq irqs[BECORE_NUM_BLOCKS * 2];
	void __iomem *ssmt[14];
	void __iomem *sysreg_rgbp;
	void __iomem *sysreg_mcsc;
	struct dev_pm_domain_list *pm_domains;
	struct dentry *debugfs;
	struct media_device mdev;
	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct media_pad vdev_pad;
	struct vb2_queue queue;
	/* Serializes V4L2 ioctls and vb2 queue setup/teardown. */
	struct mutex video_lock;
	/* Protects the pending processed-output buffer list. */
	spinlock_t queue_lock;
	struct list_head queued_outputs;
	struct work_struct video_work;
	/* Serializes staging, input ownership, execution, and output inspection. */
	struct mutex lock;
	/* Protects IRQ-driven command-hold/frame completion state. */
	spinlock_t run_lock;
	struct completion run_completion;
	struct becore_input_slot inputs[BECORE_INPUT_SLOT_COUNT];
	struct becore_input_slot *run_input;
	struct becore_dma_buffer grid;
	struct becore_dma_buffer output;
	struct becore_dma_buffer gtnr_output;
	struct becore_dma_buffer mcsc_output;
	struct exynos_becore_input *input_producer;
	u64 producer_sequence;
	u64 input_sequence;
	struct becore_cmdq_program program[BECORE_NUM_BLOCKS];
	struct becore_cmdq_program gtnr_program;
	struct becore_cmdq_program mcsc_program;
	u8 *recipe;
	u8 *gtnr_recipe;
	u8 *mcsc_recipe;
	size_t recipe_staged_bytes;
	size_t gtnr_recipe_staged_bytes;
	size_t mcsc_recipe_staged_bytes;
	u32 recipe_generation;
	u32 gtnr_recipe_generation;
	u32 gtnr_encoded_generation;
	u32 mcsc_recipe_generation;
	u32 mcsc_encoded_generation;
	u32 run_generation;
	u32 completed_generation;
	u32 video_sequence;
	u32 cmdq_hold_mask;
	u32 frame_done_mask;
	u32 expected_mask;
	u32 output_changed_bytes;
	u32 output_first_changed;
	u32 mcsc_output_changed_bytes;
	u32 mcsc_output_first_changed;
	u32 output_profile;
	u32 active_output_profile;
	u32 mcsc_completed_generation;
	u32 mcsc_completed_output_size;
	enum becore_mcsc_input_transport mcsc_encoded_transport;
	dma_addr_t active_output_dma;
	size_t active_output_size;
	size_t completed_output_size;
	int last_run_result;
	bool running;
	bool start_issued;
	bool abort_run;
	bool irq_error;
	bool irqs_enabled;
	bool reset_failed;
	bool active_output_packed;
	bool active_mcsc;
	bool output_quarantined;
	bool video_streaming;
};

struct exynos_becore_input {
	struct becore_device *becore;
	struct device *producer;
	struct sg_table sgts[BECORE_INPUT_SLOT_COUNT];
	dma_addr_t dmas[BECORE_INPUT_SLOT_COUNT];
};

static const char * const becore_pm_domain_names[] = {
	"yuvp",
	"rgbp",
	"gdc",
	"mcsc",
};

static const char * const becore_irq_names[] = {
	"rgbp-int0", "rgbp-int1",
	"mcfp-int0", "mcfp-int1",
	"yuvp-int0", "yuvp-int1",
	"mcsc-int0", "mcsc-int1",
};

static const char * const becore_ssmt_names[] = {
	"ssmt-rgbp",
	"ssmt-yuvp",
	"ssmt-mcfp0",
	"ssmt-mcfp1",
	"ssmt-mcfp2",
	"ssmt-mcfp3",
	"ssmt-mcfp4",
	"ssmt-mcsc0",
	"ssmt-mcsc1",
	"ssmt-mcsc2",
	"ssmt-mcsc3",
	"ssmt-mcsc4",
	"ssmt-mcsc5",
	"ssmt-mcsc6",
};

static const struct becore_regval becore_rgbp_init[] = {
	{ BECORE_SET_CTRL, 0x1 },
	{ BECORE_GLOBAL_ENABLE_CLEAR, 0x1 },
	{ BECORE_FRO_GLOBAL_ENABLE, 0x3fffff },
	{ BECORE_COMMAND_Q_STOP_ON_FRAME, 0x40 },
	{ BECORE_CMDQ_ENABLE, 0x1 },
	{ BECORE_STAT_RDMACL_EN, 0x1 },
	{ BECORE_GLOBAL_ENABLE, 0x1 },
	{ BECORE_RGBP_C_LOADER_ENABLE, 0x1 },
	{ BECORE_SET_CTRL, 0x0 },
};

static const struct becore_regval becore_mcfp_init[] = {
	{ BECORE_SET_CTRL, 0x1 },
	{ BECORE_GLOBAL_ENABLE_CLEAR, 0x1 },
	{ BECORE_FRO_GLOBAL_ENABLE, 0xffff },
	{ BECORE_COMMAND_Q_STOP_ON_FRAME, 0x40 },
	{ BECORE_CMDQ_ENABLE, 0x1 },
	{ BECORE_STAT_RDMACL_EN, 0x1 },
	{ BECORE_C_LOADER_ENABLE, 0x1 },
	{ BECORE_C_LOADER_MODE, 0x3 },
	{ BECORE_GLOBAL_ENABLE, 0x1 },
	{ BECORE_SET_CTRL, 0x0 },
};

static const struct becore_regval becore_yuvp_init[] = {
	{ BECORE_SET_CTRL, 0x1 },
	{ BECORE_GLOBAL_ENABLE_CLEAR, 0x1 },
	{ BECORE_FRO_GLOBAL_ENABLE, 0xedff },
	{ BECORE_COMMAND_Q_STOP_ON_FRAME, 0x40 },
	{ BECORE_CMDQ_ENABLE, 0x1 },
	{ BECORE_C_LOADER_ENABLE, 0x1 },
	{ BECORE_C_LOADER_MODE, 0x1b },
	{ BECORE_STAT_RDMACL_EN, 0x1 },
	{ BECORE_GLOBAL_ENABLE, 0x1 },
	{ BECORE_SET_CTRL, 0x0 },
};

/* MCSC keeps SET_CTRL asserted while its command queue is active. */
static const struct becore_regval becore_mcsc_init[] = {
	{ BECORE_SET_CTRL, 0x1 },
	{ BECORE_GLOBAL_ENABLE_CLEAR, 0x1 },
	{ BECORE_FRO_GLOBAL_ENABLE, 0x3fffff },
	{ BECORE_CMDQ_ENABLE, 0x1 },
	{ BECORE_STAT_RDMACL_EN, 0x1 },
	{ BECORE_GLOBAL_ENABLE, 0x1 },
};

static void becore_write_table(struct becore_block *block,
			       const struct becore_regval *table,
			       size_t count)
{
	unsigned int i;

	for (i = 0; i < count; i++)
		writel_relaxed(table[i].value, block->base + table[i].offset);
}

static void becore_issue_reset(struct becore_block *block)
{
	if (block != &block->becore->blocks[BECORE_RGBP])
		writel_relaxed(0, block->base + BECORE_C_LOADER_ENABLE);

	writel_relaxed(1, block->base + BECORE_SW_RESET);
	writel_relaxed(0, block->base + BECORE_SET_CTRL);
}

static int becore_wait_reset(struct becore_block *block)
{
	u32 value;
	int ret;

	ret = readl_poll_timeout(block->base + BECORE_SW_RESET, value, !value,
				  1, BECORE_RESET_TIMEOUT_US);
	if (ret)
		dev_err(block->becore->dev, "%s reset timed out (0x%08x)\n",
			block->name, value);

	return ret;
}

static int becore_reset_all(struct becore_device *becore)
{
	int first_error = 0;
	unsigned int i;
	int ret;

	/* The OTF-connected chain has to receive reset as one hardware phase. */
	for (i = 0; i < BECORE_NUM_BLOCKS; i++)
		becore_issue_reset(&becore->blocks[i]);

	/* Wait for every block even if an earlier one failed. */
	for (i = 0; i < BECORE_NUM_BLOCKS; i++) {
		ret = becore_wait_reset(&becore->blocks[i]);
		if (ret && !first_error)
			first_error = ret;
	}

	return first_error;
}

static size_t becore_cmdq_program_size(u32 header_count)
{
	return ALIGN((size_t)header_count * BECORE_CMDQ_HEADER_BYTES,
		     BECORE_CMDQ_PAYLOAD_BYTES) +
	       (size_t)header_count * BECORE_CMDQ_PAYLOAD_BYTES;
}

static const u8 *becore_recipe_records(const struct becore_device *becore,
				       enum becore_block_id id)
{
	const u8 *records = becore->recipe + BECORE_RECIPE_HEADER_BYTES;

	if (id == BECORE_YUVP)
		records += BECORE_RGBP_HEADER_COUNT * BECORE_RECIPE_RECORD_BYTES;

	return records;
}

static u32 becore_rgbp_input_storage_width(void)
{
	return ALIGN(becore_rgbp_input.width,
		     becore_rgbp_input.sbwc_block_width);
}

static u32 becore_rgbp_input_stride(void)
{
	return becore_rgbp_input_storage_width() *
	       becore_rgbp_input.bytes_per_pixel;
}

static size_t becore_rgbp_input_image_offset(void)
{
	return (size_t)becore_rgbp_input.header_stride *
	       becore_rgbp_input.height;
}

static size_t becore_rgbp_input_size(void)
{
	size_t image_bytes = (size_t)becore_rgbp_input_stride() *
			     becore_rgbp_input.height;

	return ALIGN(becore_rgbp_input_image_offset() + image_bytes, SZ_4K);
}

static int becore_rgbp_input_value(u32 index, u32 reg, u32 *value)
{
	if (index >= BECORE_RGBP_INPUT_WORD_COUNT ||
	    reg != becore_rgbp_input_regs[index])
		return -EINVAL;
	if (!value)
		return 0;

	switch (index) {
	case BECORE_RGBP_INPUT_FORMAT:
		*value = becore_rgbp_input.data_format;
		break;
	case BECORE_RGBP_INPUT_COMP:
		*value = becore_rgbp_input.comp_control;
		break;
	case BECORE_RGBP_INPUT_ACTIVE_WIDTH:
		*value = becore_rgbp_input.width;
		break;
	case BECORE_RGBP_INPUT_HEIGHT:
		*value = becore_rgbp_input.height;
		break;
	case BECORE_RGBP_INPUT_STRIDE:
		*value = becore_rgbp_input_stride();
		break;
	case BECORE_RGBP_INPUT_HEADER_STRIDE:
		*value = becore_rgbp_input.header_stride;
		break;
	case BECORE_RGBP_INPUT_BUSINFO:
		*value = becore_rgbp_input.businfo;
		break;
	case BECORE_RGBP_INPUT_ENABLE:
		*value = 1;
		break;
	case BECORE_RGBP_INPUT_STORAGE_WIDTH:
		*value = becore_rgbp_input_storage_width();
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct becore_yuvp_output_profile *
becore_yuvp_output_profile(const struct becore_device *becore)
{
	return &becore_yuvp_outputs[becore->active_output_profile];
}

static u32
becore_yuvp_output_stride(const struct becore_yuvp_output_profile *profile)
{
	return profile->width * profile->bytes_per_pixel;
}

static size_t
becore_yuvp_output_plane2_offset(const struct becore_yuvp_output_profile *profile)
{
	u32 stride = becore_yuvp_output_stride(profile);
	u32 luma_height;
	u32 luma_rows;

	if (!profile->mode)
		return ALIGN((size_t)stride * profile->height, SZ_4K);

	luma_height = ALIGN(profile->height, profile->luma_height_align);
	luma_rows = DIV_ROUND_UP(luma_height, profile->block_height);

	return (size_t)stride * luma_rows + profile->plane_gap;
}

static size_t
becore_yuvp_output_size(const struct becore_yuvp_output_profile *profile)
{
	u32 stride = becore_yuvp_output_stride(profile);
	u32 chroma_height = DIV_ROUND_UP(profile->height, 2);
	u32 chroma_rows;
	size_t chroma_bytes;

	if (profile->mode)
		chroma_rows = DIV_ROUND_UP(chroma_height, profile->block_height);
	else
		chroma_rows = chroma_height;
	chroma_bytes = (size_t)stride * chroma_rows;

	return ALIGN(becore_yuvp_output_plane2_offset(profile) + chroma_bytes,
		     SZ_4K);
}

static size_t
becore_yuvp_packed_plane2_offset(const struct becore_yuvp_output_profile *profile)
{
	return (size_t)becore_yuvp_output_stride(profile) * profile->height;
}

static size_t
becore_yuvp_packed_output_size(const struct becore_yuvp_output_profile *profile)
{
	u32 chroma_height = DIV_ROUND_UP(profile->height, 2);

	return becore_yuvp_packed_plane2_offset(profile) +
	       (size_t)becore_yuvp_output_stride(profile) * chroma_height;
}

static size_t becore_active_output_plane2_offset(const struct becore_device *becore)
{
	const struct becore_yuvp_output_profile *profile =
		becore_yuvp_output_profile(becore);

	if (becore->active_output_packed)
		return becore_yuvp_packed_plane2_offset(profile);

	return becore_yuvp_output_plane2_offset(profile);
}

static size_t becore_active_output_size(const struct becore_device *becore)
{
	const struct becore_yuvp_output_profile *profile =
		becore_yuvp_output_profile(becore);

	if (becore->active_output_packed)
		return becore_yuvp_packed_output_size(profile);

	return becore_yuvp_output_size(profile);
}

static size_t becore_yuvp_output_allocation_size(void)
{
	const struct becore_yuvp_output_profile *sbwcl =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL];
	const struct becore_yuvp_output_profile *p010 =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_P010];

	return max(becore_yuvp_output_size(sbwcl),
		   becore_yuvp_output_size(p010));
}

static int becore_yuvp_output_value(struct becore_device *becore, u32 index,
				    u32 reg, u32 *value)
{
	const struct becore_yuvp_output_profile *profile =
		becore_yuvp_output_profile(becore);

	if (index >= BECORE_YUVP_OUTPUT_WORD_COUNT ||
	    reg != becore_yuvp_output_regs[index])
		return -EINVAL;
	if (!value)
		return 0;

	switch (index) {
	case BECORE_YUVP_OUTPUT_VOTF:
		*value = profile->votf_enable;
		break;
	case BECORE_YUVP_OUTPUT_FORMAT:
		*value = profile->data_format;
		break;
	case BECORE_YUVP_OUTPUT_LOSSY:
		*value = profile->lossy_byte32num;
		break;
	case BECORE_YUVP_OUTPUT_MODE:
		*value = profile->mode;
		break;
	case BECORE_YUVP_OUTPUT_WIDTH:
		*value = profile->width;
		break;
	case BECORE_YUVP_OUTPUT_HEIGHT:
		*value = profile->height;
		break;
	case BECORE_YUVP_OUTPUT_STRIDE1:
	case BECORE_YUVP_OUTPUT_STRIDE2:
		*value = becore_yuvp_output_stride(profile);
		break;
	case BECORE_YUVP_OUTPUT_BUSINFO:
		*value = profile->businfo;
		break;
	case BECORE_YUVP_OUTPUT_ENABLE:
		*value = 1;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int becore_gtnr_dma_value(u32 index, u32 reg, u32 *value)
{
	if (index >= BECORE_GTNR_DMA_WORD_COUNT ||
	    reg != becore_gtnr_dma_regs[index])
		return -EINVAL;
	if (!value)
		return 0;

	switch (index) {
	case BECORE_GTNR_INPUT_VOTF:
		*value = becore_gtnr_input.votf_enable;
		break;
	case BECORE_GTNR_INPUT_FORMAT:
		*value = becore_gtnr_input.data_format;
		break;
	case BECORE_GTNR_INPUT_LOSSY:
		*value = becore_gtnr_input.lossy_byte32num;
		break;
	case BECORE_GTNR_INPUT_COMP:
		*value = becore_gtnr_input.comp_control;
		break;
	case BECORE_GTNR_INPUT_WIDTH:
		*value = becore_gtnr_input.width;
		break;
	case BECORE_GTNR_INPUT_HEIGHT:
		*value = becore_gtnr_input.height;
		break;
	case BECORE_GTNR_INPUT_STRIDE1:
	case BECORE_GTNR_INPUT_STRIDE2:
		*value = becore_gtnr_input.stride;
		break;
	case BECORE_GTNR_INPUT_BUSINFO:
		*value = becore_gtnr_input.businfo;
		break;
	case BECORE_GTNR_INPUT_MAX_MO:
		*value = becore_gtnr_input.max_mo;
		break;
	case BECORE_GTNR_INPUT_MAX_BL:
		*value = becore_gtnr_input.max_bl;
		break;
	case BECORE_GTNR_INPUT_ENABLE:
		*value = becore_gtnr_input.enable;
		break;
	case BECORE_GTNR_OUTPUT_FORMAT:
		*value = becore_gtnr_output.data_format;
		break;
	case BECORE_GTNR_OUTPUT_LOSSY:
		*value = becore_gtnr_output.lossy_byte32num;
		break;
	case BECORE_GTNR_OUTPUT_COMP:
		*value = becore_gtnr_output.comp_control;
		break;
	case BECORE_GTNR_OUTPUT_WIDTH:
		*value = becore_gtnr_output.width;
		break;
	case BECORE_GTNR_OUTPUT_HEIGHT:
		*value = becore_gtnr_output.height;
		break;
	case BECORE_GTNR_OUTPUT_STRIDE1:
	case BECORE_GTNR_OUTPUT_STRIDE2:
		*value = becore_gtnr_output.stride;
		break;
	case BECORE_GTNR_OUTPUT_BUSINFO:
		*value = becore_gtnr_output.businfo;
		break;
	case BECORE_GTNR_OUTPUT_MAX_MO:
		*value = becore_gtnr_output.max_mo;
		break;
	case BECORE_GTNR_OUTPUT_MAX_BL:
		*value = becore_gtnr_output.max_bl;
		break;
	case BECORE_GTNR_OUTPUT_ENABLE:
		*value = becore_gtnr_output.enable;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static size_t becore_mcsc_output_plane2_offset(void)
{
	return (size_t)becore_mcsc_output.stride * becore_mcsc_output.height;
}

static size_t becore_mcsc_output_size(void)
{
	size_t chroma = (size_t)becore_mcsc_output.stride *
			DIV_ROUND_UP(becore_mcsc_output.height, 2);

	return ALIGN(becore_mcsc_output_plane2_offset() + chroma, SZ_4K);
}

static int
becore_mcsc_dma_value(u32 index, u32 reg,
		      enum becore_mcsc_input_transport transport, u32 *value)
{
	if (index >= BECORE_MCSC_DMA_WORD_COUNT ||
	    reg != becore_mcsc_dma_regs[index])
		return -EINVAL;
	if (!value)
		return 0;

	switch (index) {
	case BECORE_MCSC_INPUT_VOTF:
		*value = transport == BECORE_MCSC_INPUT_MEMORY ? 0 :
			 becore_mcsc_input.votf_enable;
		break;
	case BECORE_MCSC_INPUT_FORMAT:
		*value = becore_mcsc_input.data_format;
		break;
	case BECORE_MCSC_INPUT_LOSSY:
		*value = becore_mcsc_input.lossy_byte32num;
		break;
	case BECORE_MCSC_INPUT_COMP:
		*value = becore_mcsc_input.comp_control;
		break;
	case BECORE_MCSC_INPUT_WIDTH:
		*value = becore_mcsc_input.width;
		break;
	case BECORE_MCSC_INPUT_HEIGHT:
		*value = becore_mcsc_input.height;
		break;
	case BECORE_MCSC_INPUT_STRIDE1:
	case BECORE_MCSC_INPUT_STRIDE2:
		*value = becore_mcsc_input.stride;
		break;
	case BECORE_MCSC_INPUT_BUSINFO:
		*value = becore_mcsc_input.businfo;
		break;
	case BECORE_MCSC_INPUT_MAX_BL:
		*value = becore_mcsc_input.max_bl;
		break;
	case BECORE_MCSC_INPUT_ENABLE:
		*value = becore_mcsc_input.enable;
		break;
	case BECORE_MCSC_OUTPUT_FORMAT:
		*value = becore_mcsc_output.data_format;
		break;
	case BECORE_MCSC_OUTPUT_COMP:
		*value = becore_mcsc_output.comp_control;
		break;
	case BECORE_MCSC_OUTPUT_WIDTH:
		*value = becore_mcsc_output.width;
		break;
	case BECORE_MCSC_OUTPUT_HEIGHT:
		*value = becore_mcsc_output.height;
		break;
	case BECORE_MCSC_OUTPUT_STRIDE1:
	case BECORE_MCSC_OUTPUT_STRIDE2:
		*value = becore_mcsc_output.stride;
		break;
	case BECORE_MCSC_OUTPUT_BUSINFO:
		*value = becore_mcsc_output.businfo;
		break;
	case BECORE_MCSC_OUTPUT_MAX_BL:
		*value = becore_mcsc_output.max_bl;
		break;
	case BECORE_MCSC_OUTPUT_ENABLE:
		*value = becore_mcsc_output.enable;
		break;
	case BECORE_MCSC_OUTPUT_DITHER:
		*value = becore_mcsc_output.dither;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static size_t becore_gtnr_surface_plane2_offset(void)
{
	const struct becore_yuvp_output_profile *profile =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL];

	return becore_yuvp_output_plane2_offset(profile);
}

static size_t becore_gtnr_surface_size(void)
{
	const struct becore_yuvp_output_profile *profile =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL];

	return becore_yuvp_output_size(profile);
}

static dma_addr_t becore_gtnr_address_dma(struct becore_device *becore, u32 reg)
{
	switch (reg) {
	case BECORE_GTNR_INPUT_PLANE1_REG:
		return becore->output.dma;
	case BECORE_GTNR_INPUT_PLANE2_REG:
		return becore->output.dma + becore_gtnr_surface_plane2_offset();
	case BECORE_GTNR_OUTPUT_PLANE1_REG:
		return becore->gtnr_output.dma;
	case BECORE_GTNR_OUTPUT_PLANE2_REG:
		return becore->gtnr_output.dma +
		       becore_gtnr_surface_plane2_offset();
	default:
		return DMA_MAPPING_ERROR;
	}
}

static dma_addr_t becore_mcsc_address_dma(struct becore_device *becore, u32 reg)
{
	const struct becore_yuvp_output_profile *input =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL];

	switch (reg) {
	case BECORE_MCSC_INPUT_PLANE1_REG:
		return becore->output.dma;
	case BECORE_MCSC_INPUT_PLANE2_REG:
		return becore->output.dma + becore_yuvp_output_plane2_offset(input);
	case BECORE_MCSC_OUTPUT_PLANE1_REG:
		return becore->mcsc_output.dma;
	case BECORE_MCSC_OUTPUT_PLANE2_REG:
		return becore->mcsc_output.dma +
		       becore_mcsc_output_plane2_offset();
	default:
		return DMA_MAPPING_ERROR;
	}
}

static u32 becore_typed_word_count(enum becore_block_id id)
{
	if (id == BECORE_RGBP)
		return BECORE_RGBP_INPUT_WORD_COUNT;
	if (id == BECORE_YUVP)
		return BECORE_YUVP_OUTPUT_WORD_COUNT;

	return 0;
}

static int becore_typed_value(struct becore_device *becore,
			      enum becore_block_id id, u32 index, u32 reg,
			      u32 *value)
{
	if (id == BECORE_RGBP)
		return becore_rgbp_input_value(index, reg, value);
	if (id == BECORE_YUVP)
		return becore_yuvp_output_value(becore, index, reg, value);

	return -EINVAL;
}

static int becore_recipe_header_validate(const struct becore_device *becore)
{
	const u8 *header = becore->recipe;

	if (becore->recipe_staged_bytes != BECORE_RECIPE_BYTES)
		return -EINVAL;
	if (get_unaligned_le32(header) != BECORE_RECIPE_MAGIC ||
	    get_unaligned_le32(header + 4) != BECORE_RECIPE_VERSION ||
	    get_unaligned_le32(header + 8) != BECORE_RECIPE_HEADER_BYTES ||
	    get_unaligned_le32(header + 12) != BECORE_RECIPE_RECORD_BYTES ||
	    get_unaligned_le32(header + 16) != BECORE_RGBP_HEADER_COUNT ||
	    get_unaligned_le32(header + 20) != BECORE_YUVP_HEADER_COUNT ||
	    get_unaligned_le32(header + 24) != BECORE_RECIPE_BYTES ||
	    get_unaligned_le32(header + 28))
		return -EINVAL;

	return 0;
}

static dma_addr_t becore_address_dma(struct becore_device *becore, u32 reg)
{
	struct becore_dma_buffer *input = &becore->run_input->buffer;

	switch (reg) {
	case BECORE_RGBP_INPUT_IMAGE_REG:
		return input->dma + becore_rgbp_input_image_offset();
	case BECORE_RGBP_INPUT_HEADER_REG:
		return input->dma;
	case BECORE_YUVP_GRID_REG:
		return becore->grid.dma;
	case BECORE_YUVP_OUTPUT_PLANE1_REG:
		return becore->active_output_dma;
	case BECORE_YUVP_OUTPUT_PLANE2_REG:
		return becore->active_output_dma +
		       becore_active_output_plane2_offset(becore);
	default:
		return DMA_MAPPING_ERROR;
	}
}

static int becore_recipe_block_validate(struct becore_device *becore,
					enum becore_block_id id,
					const struct becore_cmdq_shape *shape,
					u32 header_count)
{
	const u8 *record = becore_recipe_records(becore, id);
	u32 address_count = 0;
	u32 typed_count = 0;
	u32 i;

	for (i = 0; i < header_count; i++, record += BECORE_RECIPE_RECORD_BYTES) {
		const u8 *words = record + 12;
		u16 used_mask;
		u32 word;

		if (!shape[i].valid_words || shape[i].valid_words > 16)
			return -EINVAL;
		used_mask = shape[i].valid_words == 16 ? U16_MAX :
			    GENMASK(shape[i].valid_words - 1, 0);
		if (get_unaligned_le32(record) != shape[i].mode ||
		    get_unaligned_le32(record + 4) != shape[i].target ||
		    get_unaligned_le32(record + 8) != shape[i].type_map ||
		    (shape[i].address_mask & ~used_mask) ||
		    (shape[i].typed_mask & ~used_mask) ||
		    (shape[i].fixed_mask & ~used_mask) ||
		    (shape[i].address_mask & shape[i].typed_mask) ||
		    (shape[i].address_mask & shape[i].fixed_mask) ||
		    (shape[i].typed_mask & shape[i].fixed_mask))
			return -EINVAL;

		for (word = 0; word < 16; word++) {
			u32 value = get_unaligned_le32(words + word * 4);

			if (word >= shape[i].valid_words) {
				if (value)
					return -EINVAL;
				continue;
			}

			if (shape[i].mode == 0x00090000 && !(word & 1)) {
				if (value != shape[i].pair_registers[word / 2])
					return -EINVAL;
				continue;
			}

			if (shape[i].fixed_mask & BIT(word)) {
				if (value != shape[i].fixed_values[word])
					return -EINVAL;
			}

			if (shape[i].typed_mask & BIT(word)) {
				u32 reg;

				if (shape[i].mode != 0x00090000 || !(word & 1))
					return -EINVAL;
				reg = shape[i].pair_registers[word / 2];
				if (becore_typed_value(becore, id, typed_count, reg,
						       NULL))
					return -EINVAL;
				typed_count++;
			}

			if (shape[i].address_mask & BIT(word)) {
				u32 reg;

				/* All five supported DMA fields are pair-mode values. */
				if (shape[i].mode != 0x00090000 || !(word & 1))
					return -EINVAL;
				reg = shape[i].pair_registers[word / 2];
				if (becore_address_dma(becore, reg) ==
				    DMA_MAPPING_ERROR)
					return -EINVAL;
				address_count++;
			}
		}
	}

	if ((id == BECORE_RGBP && address_count != 2) ||
	    (id == BECORE_YUVP && address_count != 3))
		return -EINVAL;
	if (typed_count != becore_typed_word_count(id))
		return -EINVAL;

	return 0;
}

static int becore_recipe_validate(struct becore_device *becore)
{
	struct becore_dma_buffer *input = &becore->run_input->buffer;
	int ret;

	ret = becore_recipe_header_validate(becore);
	if (ret)
		return ret;
	if (input->size != becore_rgbp_input_size() ||
	    input->staged_bytes != input->size ||
	    becore->grid.staged_bytes != BECORE_GRID_SIZE)
		return -EINVAL;

	ret = becore_recipe_block_validate(becore, BECORE_RGBP,
					   becore_rgbp_shape,
					   BECORE_RGBP_HEADER_COUNT);
	if (ret)
		return ret;

	return becore_recipe_block_validate(becore, BECORE_YUVP,
					    becore_yuvp_shape,
					    BECORE_YUVP_HEADER_COUNT);
}

static int becore_encode_block(struct becore_device *becore,
			       enum becore_block_id id,
			       const struct becore_cmdq_shape *shape,
			       u32 header_count)
{
	struct becore_cmdq_program *program = &becore->program[id];
	const u8 *record = becore_recipe_records(becore, id);
	size_t payload_offset = ALIGN((size_t)header_count *
				      BECORE_CMDQ_HEADER_BYTES,
				      BECORE_CMDQ_PAYLOAD_BYTES);
	u32 i;
	u32 typed_count = 0;

	if (!program->cpu || program->header_count != header_count ||
	    program->size != becore_cmdq_program_size(header_count) ||
	    upper_32_bits(program->dma) ||
	    upper_32_bits(program->dma + program->size - 1))
		return -EINVAL;

	memset(program->cpu, 0, program->size);
	for (i = 0; i < header_count;
	     i++, record += BECORE_RECIPE_RECORD_BYTES) {
		u8 *header = (u8 *)program->cpu + i * BECORE_CMDQ_HEADER_BYTES;
		u8 *payload = (u8 *)program->cpu + payload_offset +
			      i * BECORE_CMDQ_PAYLOAD_BYTES;
		dma_addr_t payload_dma = program->dma + payload_offset +
					 i * BECORE_CMDQ_PAYLOAD_BYTES;
		u32 word;

		put_unaligned_le32(shape[i].mode, header);
		put_unaligned_le32(lower_32_bits(payload_dma), header + 4);
		put_unaligned_le32(shape[i].target, header + 8);
		put_unaligned_le32(shape[i].type_map, header + 12);
		memcpy(payload, record + 12, BECORE_CMDQ_PAYLOAD_BYTES);

		for (word = 0; word < shape[i].valid_words; word++) {
			dma_addr_t dma;
			u32 reg;
			u32 value;

			if (shape[i].typed_mask & BIT(word)) {
				reg = shape[i].pair_registers[word / 2];
				if (becore_typed_value(becore, id, typed_count, reg,
						       &value))
					return -EINVAL;
				put_unaligned_le32(value, payload + word * 4);
				typed_count++;
				continue;
			}

			if (!(shape[i].address_mask & BIT(word)))
				continue;
			reg = shape[i].pair_registers[word / 2];
			dma = becore_address_dma(becore, reg);
			if (dma == DMA_MAPPING_ERROR || upper_32_bits(dma))
				return -EINVAL;
			put_unaligned_le32(lower_32_bits(dma), payload + word * 4);
		}
	}
	if (typed_count != becore_typed_word_count(id))
		return -EINVAL;

	return 0;
}

static int becore_encode_programs(struct becore_device *becore)
{
	int ret;

	ret = becore_encode_block(becore, BECORE_RGBP, becore_rgbp_shape,
				  BECORE_RGBP_HEADER_COUNT);
	if (ret)
		return ret;

	return becore_encode_block(becore, BECORE_YUVP, becore_yuvp_shape,
				   BECORE_YUVP_HEADER_COUNT);
}

static int becore_gtnr_recipe_validate(struct becore_device *becore)
{
	const u8 *header = becore->gtnr_recipe;
	const u8 *record = header + BECORE_GTNR_RECIPE_HEADER_BYTES;
	const struct becore_yuvp_output_profile *input =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL];
	u32 address_count = 0;
	u32 typed_count = 0;
	u32 i;

	if (becore->gtnr_recipe_staged_bytes != BECORE_GTNR_RECIPE_BYTES ||
	    get_unaligned_le32(header) != BECORE_GTNR_RECIPE_MAGIC ||
	    get_unaligned_le32(header + 4) != BECORE_GTNR_RECIPE_VERSION ||
	    get_unaligned_le32(header + 8) != BECORE_GTNR_RECIPE_HEADER_BYTES ||
	    get_unaligned_le32(header + 12) != BECORE_GTNR_RECIPE_RECORD_BYTES ||
	    get_unaligned_le32(header + 16) != BECORE_GTNR_HEADER_COUNT ||
	    get_unaligned_le32(header + 20) != BECORE_GTNR_RECIPE_BYTES ||
	    get_unaligned_le32(header + 24) || get_unaligned_le32(header + 28))
		return -EINVAL;
	if (becore->output.size < becore_gtnr_surface_size() ||
	    becore->gtnr_output.size != becore_gtnr_surface_size() ||
	    becore_gtnr_input.width != input->width ||
	    becore_gtnr_input.height != input->height ||
	    becore_gtnr_input.stride != becore_yuvp_output_stride(input) ||
	    becore_gtnr_output.width != input->width ||
	    becore_gtnr_output.height != input->height ||
	    becore_gtnr_output.stride != becore_yuvp_output_stride(input))
		return -EINVAL;

	for (i = 0; i < BECORE_GTNR_HEADER_COUNT;
	     i++, record += BECORE_GTNR_RECIPE_RECORD_BYTES) {
		const struct becore_cmdq_shape *shape = &becore_gtnr_shape[i];
		const u8 *words = record + 12;
		u16 used_mask;
		u32 word;

		if (!shape->valid_words || shape->valid_words > 16)
			return -EINVAL;
		used_mask = shape->valid_words == 16 ? U16_MAX :
			    GENMASK(shape->valid_words - 1, 0);
		if (get_unaligned_le32(record) != shape->mode ||
		    get_unaligned_le32(record + 4) != shape->target ||
		    get_unaligned_le32(record + 8) != shape->type_map ||
		    (shape->address_mask & ~used_mask) ||
		    (shape->typed_mask & ~used_mask) ||
		    (shape->fixed_mask & ~used_mask) ||
		    (shape->address_mask & shape->typed_mask) ||
		    (shape->address_mask & shape->fixed_mask) ||
		    (shape->typed_mask & shape->fixed_mask))
			return -EINVAL;

		for (word = 0; word < 16; word++) {
			u32 value = get_unaligned_le32(words + word * 4);

			if (word >= shape->valid_words) {
				if (value)
					return -EINVAL;
				continue;
			}
			if (shape->mode == 0x00090000 && !(word & 1)) {
				if (value != shape->pair_registers[word / 2])
					return -EINVAL;
				continue;
			}
			if ((shape->fixed_mask & BIT(word)) &&
			    value != shape->fixed_values[word])
				return -EINVAL;

			if (shape->typed_mask & BIT(word)) {
				u32 reg;

				if (shape->mode != 0x00090000 || !(word & 1))
					return -EINVAL;
				reg = shape->pair_registers[word / 2];
				if (becore_gtnr_dma_value(typed_count, reg, NULL))
					return -EINVAL;
				typed_count++;
			}
			if (shape->address_mask & BIT(word)) {
				u32 reg;

				if (shape->mode != 0x00090000 || !(word & 1))
					return -EINVAL;
				reg = shape->pair_registers[word / 2];
				if (becore_gtnr_address_dma(becore, reg) ==
				    DMA_MAPPING_ERROR)
					return -EINVAL;
				address_count++;
			}
		}
	}

	if (address_count != 4 || typed_count != BECORE_GTNR_DMA_WORD_COUNT)
		return -EINVAL;

	return 0;
}

static int becore_encode_gtnr(struct becore_device *becore)
{
	struct becore_cmdq_program *program = &becore->gtnr_program;
	const u8 *record = becore->gtnr_recipe +
			   BECORE_GTNR_RECIPE_HEADER_BYTES;
	size_t payload_offset = ALIGN((size_t)BECORE_GTNR_HEADER_COUNT *
				      BECORE_CMDQ_HEADER_BYTES,
				      BECORE_CMDQ_PAYLOAD_BYTES);
	u32 typed_count = 0;
	u32 i;

	if (!program->cpu || program->header_count != BECORE_GTNR_HEADER_COUNT ||
	    program->size != becore_cmdq_program_size(BECORE_GTNR_HEADER_COUNT) ||
	    upper_32_bits(program->dma) ||
	    upper_32_bits(program->dma + program->size - 1))
		return -EINVAL;

	memset(program->cpu, 0, program->size);
	for (i = 0; i < BECORE_GTNR_HEADER_COUNT;
	     i++, record += BECORE_GTNR_RECIPE_RECORD_BYTES) {
		const struct becore_cmdq_shape *shape = &becore_gtnr_shape[i];
		u8 *header = (u8 *)program->cpu + i * BECORE_CMDQ_HEADER_BYTES;
		u8 *payload = (u8 *)program->cpu + payload_offset +
			      i * BECORE_CMDQ_PAYLOAD_BYTES;
		dma_addr_t payload_dma = program->dma + payload_offset +
					 i * BECORE_CMDQ_PAYLOAD_BYTES;
		u32 word;

		put_unaligned_le32(shape->mode, header);
		put_unaligned_le32(lower_32_bits(payload_dma), header + 4);
		put_unaligned_le32(shape->target, header + 8);
		put_unaligned_le32(shape->type_map, header + 12);
		memcpy(payload, record + 12, BECORE_CMDQ_PAYLOAD_BYTES);

		for (word = 0; word < shape->valid_words; word++) {
			dma_addr_t dma;
			u32 reg;
			u32 value;

			if (shape->typed_mask & BIT(word)) {
				reg = shape->pair_registers[word / 2];
				if (becore_gtnr_dma_value(typed_count, reg, &value))
					return -EINVAL;
				put_unaligned_le32(value, payload + word * 4);
				typed_count++;
				continue;
			}
			if (!(shape->address_mask & BIT(word)))
				continue;
			reg = shape->pair_registers[word / 2];
			dma = becore_gtnr_address_dma(becore, reg);
			if (dma == DMA_MAPPING_ERROR || upper_32_bits(dma))
				return -EINVAL;
			put_unaligned_le32(lower_32_bits(dma), payload + word * 4);
		}
	}

	if (typed_count != BECORE_GTNR_DMA_WORD_COUNT)
		return -EINVAL;

	return 0;
}

static int becore_mcsc_recipe_validate(struct becore_device *becore)
{
	const u8 *header = becore->mcsc_recipe;
	const u8 *record = header + BECORE_MCSC_RECIPE_HEADER_BYTES;
	const struct becore_yuvp_output_profile *input =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL];
	u32 address_count = 0;
	u32 typed_count = 0;
	u32 i;

	if (becore->mcsc_recipe_staged_bytes != BECORE_MCSC_RECIPE_BYTES ||
	    get_unaligned_le32(header) != BECORE_MCSC_RECIPE_MAGIC ||
	    get_unaligned_le32(header + 4) != BECORE_MCSC_RECIPE_VERSION ||
	    get_unaligned_le32(header + 8) != BECORE_MCSC_RECIPE_HEADER_BYTES ||
	    get_unaligned_le32(header + 12) != BECORE_MCSC_RECIPE_RECORD_BYTES ||
	    get_unaligned_le32(header + 16) != BECORE_MCSC_HEADER_COUNT ||
	    get_unaligned_le32(header + 20) != BECORE_MCSC_RECIPE_BYTES ||
	    get_unaligned_le32(header + 24) || get_unaligned_le32(header + 28))
		return -EINVAL;
	if (becore->output.size < becore_yuvp_output_size(input) ||
	    becore->mcsc_output.size != becore_mcsc_output_size())
		return -EINVAL;

	for (i = 0; i < BECORE_MCSC_HEADER_COUNT;
	     i++, record += BECORE_MCSC_RECIPE_RECORD_BYTES) {
		const struct becore_cmdq_shape *shape = &becore_mcsc_shape[i];
		const u8 *words = record + 12;
		u16 used_mask;
		u32 word;

		if (!shape->valid_words || shape->valid_words > 16)
			return -EINVAL;
		used_mask = shape->valid_words == 16 ? U16_MAX :
			    GENMASK(shape->valid_words - 1, 0);
		if (get_unaligned_le32(record) != shape->mode ||
		    get_unaligned_le32(record + 4) != shape->target ||
		    get_unaligned_le32(record + 8) != shape->type_map ||
		    (shape->address_mask & ~used_mask) ||
		    (shape->typed_mask & ~used_mask) ||
		    (shape->fixed_mask & ~used_mask) ||
		    (shape->address_mask & shape->typed_mask) ||
		    (shape->address_mask & shape->fixed_mask) ||
		    (shape->typed_mask & shape->fixed_mask))
			return -EINVAL;

		for (word = 0; word < 16; word++) {
			u32 value = get_unaligned_le32(words + word * 4);

			if (word >= shape->valid_words) {
				if (value)
					return -EINVAL;
				continue;
			}
			if (shape->mode == 0x00090000 && !(word & 1)) {
				if (value != shape->pair_registers[word / 2])
					return -EINVAL;
				continue;
			}
			if ((shape->fixed_mask & BIT(word)) &&
			    value != shape->fixed_values[word])
				return -EINVAL;

			if (shape->typed_mask & BIT(word)) {
				u32 reg;

				if (shape->mode != 0x00090000 || !(word & 1))
					return -EINVAL;
				reg = shape->pair_registers[word / 2];
				if (becore_mcsc_dma_value(typed_count, reg,
							  BECORE_MCSC_INPUT_CAPTURED_VOTF,
							  NULL))
					return -EINVAL;
				typed_count++;
			}
			if (shape->address_mask & BIT(word)) {
				u32 reg;

				if (shape->mode != 0x00090000 || !(word & 1))
					return -EINVAL;
				reg = shape->pair_registers[word / 2];
				if (becore_mcsc_address_dma(becore, reg) ==
				    DMA_MAPPING_ERROR)
					return -EINVAL;
				address_count++;
			}
		}
	}

	if (address_count != 4 || typed_count != BECORE_MCSC_DMA_WORD_COUNT)
		return -EINVAL;

	return 0;
}

static int
becore_encode_mcsc(struct becore_device *becore,
		   enum becore_mcsc_input_transport transport)
{
	struct becore_cmdq_program *program = &becore->mcsc_program;
	const u8 *record = becore->mcsc_recipe +
			   BECORE_MCSC_RECIPE_HEADER_BYTES;
	size_t payload_offset = ALIGN((size_t)BECORE_MCSC_HEADER_COUNT *
				      BECORE_CMDQ_HEADER_BYTES,
				      BECORE_CMDQ_PAYLOAD_BYTES);
	u32 typed_count = 0;
	u32 i;

	if (!program->cpu || program->header_count != BECORE_MCSC_HEADER_COUNT ||
	    program->size != becore_cmdq_program_size(BECORE_MCSC_HEADER_COUNT) ||
	    upper_32_bits(program->dma) ||
	    upper_32_bits(program->dma + program->size - 1))
		return -EINVAL;

	memset(program->cpu, 0, program->size);
	for (i = 0; i < BECORE_MCSC_HEADER_COUNT;
	     i++, record += BECORE_MCSC_RECIPE_RECORD_BYTES) {
		const struct becore_cmdq_shape *shape = &becore_mcsc_shape[i];
		u8 *header = (u8 *)program->cpu + i * BECORE_CMDQ_HEADER_BYTES;
		u8 *payload = (u8 *)program->cpu + payload_offset +
			      i * BECORE_CMDQ_PAYLOAD_BYTES;
		dma_addr_t payload_dma = program->dma + payload_offset +
					 i * BECORE_CMDQ_PAYLOAD_BYTES;
		u32 word;

		put_unaligned_le32(shape->mode, header);
		put_unaligned_le32(lower_32_bits(payload_dma), header + 4);
		put_unaligned_le32(shape->target, header + 8);
		put_unaligned_le32(shape->type_map, header + 12);
		memcpy(payload, record + 12, BECORE_CMDQ_PAYLOAD_BYTES);

		for (word = 0; word < shape->valid_words; word++) {
			dma_addr_t dma;
			u32 reg;
			u32 value;

			if (shape->typed_mask & BIT(word)) {
				reg = shape->pair_registers[word / 2];
				if (becore_mcsc_dma_value(typed_count, reg, transport,
							  &value))
					return -EINVAL;
				put_unaligned_le32(value, payload + word * 4);
				typed_count++;
				continue;
			}
			if (!(shape->address_mask & BIT(word)))
				continue;
			reg = shape->pair_registers[word / 2];
			dma = becore_mcsc_address_dma(becore, reg);
			if (dma == DMA_MAPPING_ERROR || upper_32_bits(dma))
				return -EINVAL;
			put_unaligned_le32(lower_32_bits(dma), payload + word * 4);
		}
	}

	if (typed_count != BECORE_MCSC_DMA_WORD_COUNT)
		return -EINVAL;

	return 0;
}

static void becore_prepare_irqs(struct becore_block *block)
{
	writel_relaxed(block->int0_mask_prepare,
		       block->base + BECORE_INT0_ENABLE);
	if (block->int1_mask)
		writel_relaxed(block->int1_mask,
			       block->base + BECORE_INT1_ENABLE);
	writel_relaxed(block->cmdq_int_mask,
		       block->base + BECORE_CMDQ_INT_ENABLE);
}

static void becore_quiesce_irqs(struct becore_block *block)
{
	writel_relaxed(BIT(2), block->base + BECORE_INT0_ENABLE);
	if (block->int1_mask)
		writel_relaxed(0, block->base + BECORE_INT1_ENABLE);
	writel_relaxed(0, block->base + BECORE_CMDQ_INT_ENABLE);
}

static void becore_enable_linux_irqs(struct becore_device *becore)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(becore->irqs); i++)
		enable_irq(becore->irqs[i].irq);
	becore->irqs_enabled = true;
}

static void becore_disable_linux_irqs(struct becore_device *becore)
{
	unsigned int i;

	if (!becore->irqs_enabled)
		return;

	/* disable_irq() also waits for a handler already running on another CPU. */
	for (i = 0; i < ARRAY_SIZE(becore->irqs); i++)
		disable_irq(becore->irqs[i].irq);
	becore->irqs_enabled = false;
}

static int becore_runtime_resume(struct device *dev)
{
	struct becore_device *becore = dev_get_drvdata(dev);
	unsigned int i;
	int ret;

	ret = becore_reset_all(becore);
	if (ret) {
		/* Report success so runtime PM retains all supplier references. */
		becore->reset_failed = true;
		return 0;
	}
	becore->reset_failed = false;

	becore_write_table(&becore->blocks[BECORE_RGBP], becore_rgbp_init,
			   ARRAY_SIZE(becore_rgbp_init));
	becore_write_table(&becore->blocks[BECORE_MCFP], becore_mcfp_init,
			   ARRAY_SIZE(becore_mcfp_init));
	becore_write_table(&becore->blocks[BECORE_YUVP], becore_yuvp_init,
			   ARRAY_SIZE(becore_yuvp_init));
	becore_write_table(&becore->blocks[BECORE_MCSC], becore_mcsc_init,
			   ARRAY_SIZE(becore_mcsc_init));

	for (i = 0; i < BECORE_NUM_BLOCKS; i++)
		becore_prepare_irqs(&becore->blocks[i]);
	for (i = 0; i < BECORE_NUM_BLOCKS; i++)
		writel_relaxed(becore->blocks[i].int0_mask,
			       becore->blocks[i].base + BECORE_INT0_ENABLE);

	becore_enable_linux_irqs(becore);

	return 0;
}

static int becore_runtime_suspend(struct device *dev)
{
	struct becore_device *becore = dev_get_drvdata(dev);
	unsigned int i;
	int ret;

	becore_disable_linux_irqs(becore);
	for (i = 0; i < BECORE_NUM_BLOCKS; i++)
		becore_quiesce_irqs(&becore->blocks[i]);
	for (i = 0; i < BECORE_NUM_BLOCKS; i++)
		writel_relaxed(0, becore->blocks[i].base + BECORE_INT0_ENABLE);

	/* A failed reset must veto the following genpd power-down. */
	ret = becore_reset_all(becore);
	becore->reset_failed = !!ret;

	return ret;
}

static void becore_process_frame_irq(struct becore_block *block, u32 status,
				     bool error)
{
	struct becore_device *becore = block->becore;
	enum becore_block_id id = block - becore->blocks;
	unsigned long flags;
	unsigned int i;
	bool complete_run = false;

	if (id == BECORE_MCFP)
		return;

	spin_lock_irqsave(&becore->run_lock, flags);
	if (!becore->running)
		goto unlock;
	if (error) {
		becore->irq_error = true;
		becore->abort_run = true;
		complete_run = true;
		goto unlock;
	}
	if (!(becore->expected_mask & BIT(id)))
		goto unlock;

	if (status & BECORE_INT_CMDQ_HOLD) {
		becore->cmdq_hold_mask |= BIT(id);
		if ((becore->cmdq_hold_mask & becore->expected_mask) ==
		    becore->expected_mask &&
		    !becore->start_issued && !becore->abort_run) {
			becore->start_issued = true;
			/* Release the downstream end of the chain first. */
			for (i = BECORE_NUM_BLOCKS; i-- > 0;)
				if (becore->expected_mask & BIT(i))
					writel(1, becore->blocks[i].base +
					       BECORE_CMDQ_ADD_TO_QUEUE_0);
		}
	}
	if (status & BECORE_INT_FRAME_END) {
		becore->frame_done_mask |= BIT(id);
		if ((becore->frame_done_mask & becore->expected_mask) ==
		    becore->expected_mask)
			complete_run = true;
	}

unlock:
	spin_unlock_irqrestore(&becore->run_lock, flags);

	if (complete_run)
		complete(&becore->run_completion);
}

static irqreturn_t becore_irq_handler(int irq, void *data)
{
	struct becore_irq *irq_data = data;
	struct becore_block *block = irq_data->block;
	u32 cmdq_status;
	u32 status;

	if (irq_data->int1) {
		status = readl_relaxed(block->base + BECORE_INT1_STATUS);
		if (!status)
			return IRQ_NONE;

		writel_relaxed(status, block->base + BECORE_INT1_CLEAR);
		WRITE_ONCE(block->last_int1, status);
		atomic64_inc(&block->int1_count);
		becore_process_frame_irq(block, status, true);
		return IRQ_HANDLED;
	}

	status = readl_relaxed(block->base + BECORE_INT0_STATUS);
	cmdq_status = readl_relaxed(block->base + BECORE_CMDQ_INT_STATUS);
	if (!status && !cmdq_status)
		return IRQ_NONE;

	if (status)
		writel_relaxed(status, block->base + BECORE_INT0_CLEAR);
	if (cmdq_status)
		writel_relaxed(cmdq_status, block->base + BECORE_CMDQ_INT_CLEAR);

	WRITE_ONCE(block->last_int0, status);
	WRITE_ONCE(block->last_cmdq_int, cmdq_status);
	atomic64_inc(&block->int0_count);
	becore_process_frame_irq(block, status,
				 !!cmdq_status || !!(status & ~BECORE_INT_EXPECTED));

	return IRQ_HANDLED;
}

static int becore_map_resources(struct platform_device *pdev,
				struct becore_device *becore)
{
	static const char * const block_names[] = {
		"rgbp", "mcfp", "yuvp", "mcsc",
	};
	struct device *dev = &pdev->dev;
	unsigned int i;

	for (i = 0; i < BECORE_NUM_BLOCKS; i++) {
		becore->blocks[i].base =
			devm_platform_ioremap_resource_byname(pdev, block_names[i]);
		if (IS_ERR(becore->blocks[i].base))
			return dev_err_probe(dev, PTR_ERR(becore->blocks[i].base),
					     "cannot map %s\n", block_names[i]);
	}

	for (i = 0; i < ARRAY_SIZE(becore->ssmt); i++) {
		becore->ssmt[i] =
			devm_platform_ioremap_resource_byname(pdev,
						       becore_ssmt_names[i]);
		if (IS_ERR(becore->ssmt[i]))
			return dev_err_probe(dev, PTR_ERR(becore->ssmt[i]),
					     "cannot map %s\n", becore_ssmt_names[i]);
	}

	becore->sysreg_rgbp =
		devm_platform_ioremap_resource_byname(pdev, "sysreg-rgbp");
	if (IS_ERR(becore->sysreg_rgbp))
		return dev_err_probe(dev, PTR_ERR(becore->sysreg_rgbp),
				     "cannot map sysreg-rgbp\n");

	becore->sysreg_mcsc =
		devm_platform_ioremap_resource_byname(pdev, "sysreg-mcsc");
	if (IS_ERR(becore->sysreg_mcsc))
		return dev_err_probe(dev, PTR_ERR(becore->sysreg_mcsc),
				     "cannot map sysreg-mcsc\n");

	return 0;
}

static int becore_request_irqs(struct platform_device *pdev,
				struct becore_device *becore)
{
	struct device *dev = &pdev->dev;
	unsigned int i;
	int irq;
	int ret;

	for (i = 0; i < ARRAY_SIZE(becore->irqs); i++) {
		irq = platform_get_irq_byname(pdev, becore_irq_names[i]);
		if (irq < 0)
			return irq;

		becore->irqs[i].block = &becore->blocks[i / 2];
		becore->irqs[i].irq = irq;
		becore->irqs[i].int1 = i & 1;
		ret = devm_request_irq(dev, irq, becore_irq_handler,
				       IRQF_NO_AUTOEN,
				       becore_irq_names[i], &becore->irqs[i]);
		if (ret)
			return dev_err_probe(dev, ret, "cannot request %s\n",
					     becore_irq_names[i]);
	}

	return 0;
}

static int becore_alloc_dma_buffer(struct becore_device *becore,
				   struct becore_dma_buffer *buffer,
				   size_t size, const char *name)
{
	buffer->cpu = dmam_alloc_coherent(becore->dev, size, &buffer->dma,
					  GFP_KERNEL);
	if (!buffer->cpu)
		return dev_err_probe(becore->dev, -ENOMEM,
				     "cannot allocate %s buffer\n", name);
	buffer->size = size;
	if (upper_32_bits(buffer->dma) ||
	    upper_32_bits(buffer->dma + buffer->size - 1))
		return dev_err_probe(becore->dev, -ERANGE,
				     "%s buffer is outside 32-bit DMA\n", name);

	return 0;
}

static void becore_free_shared_input(void *data)
{
	struct becore_device *becore = data;
	unsigned int i;

	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		struct becore_dma_buffer *input = &becore->inputs[i].buffer;

		if (input->cpu)
			dma_vunmap_noncontiguous(becore->dev, input->cpu);
		if (input->sgt)
			dma_free_noncontiguous(becore->dev, input->size,
					       input->sgt, DMA_BIDIRECTIONAL);
		input->cpu = NULL;
		input->sgt = NULL;
	}
}

static int becore_alloc_shared_input(struct becore_device *becore)
{
	unsigned int i;
	int ret;

	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		struct becore_dma_buffer *input = &becore->inputs[i].buffer;

		input->size = becore_rgbp_input_size();
		input->sgt = dma_alloc_noncontiguous(becore->dev, input->size,
						     DMA_BIDIRECTIONAL,
						     GFP_KERNEL, 0);
		if (!input->sgt) {
			ret = -ENOMEM;
			goto err_free;
		}

		input->dma = sg_dma_address(input->sgt->sgl);
		input->cpu = dma_vmap_noncontiguous(becore->dev, input->size,
						    input->sgt);
		if (!input->cpu) {
			ret = -ENOMEM;
			goto err_free;
		}
		if (upper_32_bits(input->dma) ||
		    upper_32_bits(input->dma + input->size - 1)) {
			ret = -ERANGE;
			goto err_free;
		}
	}

	ret = devm_add_action_or_reset(becore->dev,
				       becore_free_shared_input, becore);
	return ret;

err_free:
	becore_free_shared_input(becore);
	return dev_err_probe(becore->dev, ret,
			     "cannot allocate Bayer input slot %u\n", i);
}

static int becore_alloc_cmdq_buffer(struct becore_device *becore,
				    struct becore_cmdq_program *program,
				    u32 header_count, const char *name)
{
	program->header_count = header_count;
	program->size = becore_cmdq_program_size(header_count);
	program->cpu = dmam_alloc_coherent(becore->dev, program->size,
					   &program->dma, GFP_KERNEL);
	if (!program->cpu)
		return dev_err_probe(becore->dev, -ENOMEM,
				     "cannot allocate %s CMDQ program\n",
				     name);
	if (upper_32_bits(program->dma) ||
	    upper_32_bits(program->dma + program->size - 1))
		return dev_err_probe(becore->dev, -ERANGE,
				     "%s CMDQ program is outside 32-bit DMA\n",
				     name);

	return 0;
}

static int becore_alloc_cmdq_program(struct becore_device *becore,
				     enum becore_block_id id,
				     u32 header_count)
{
	return becore_alloc_cmdq_buffer(becore, &becore->program[id],
					header_count, becore->blocks[id].name);
}

static int becore_alloc_diagnostic(struct becore_device *becore)
{
	size_t output_size = becore_yuvp_output_allocation_size();
	int ret;

	becore->recipe = devm_kzalloc(becore->dev, BECORE_RECIPE_BYTES,
				      GFP_KERNEL);
	if (!becore->recipe)
		return -ENOMEM;
	becore->gtnr_recipe = devm_kzalloc(becore->dev,
					   BECORE_GTNR_RECIPE_BYTES, GFP_KERNEL);
	if (!becore->gtnr_recipe)
		return -ENOMEM;
	becore->mcsc_recipe = devm_kzalloc(becore->dev,
					   BECORE_MCSC_RECIPE_BYTES, GFP_KERNEL);
	if (!becore->mcsc_recipe)
		return -ENOMEM;

	ret = becore_alloc_shared_input(becore);
	if (ret)
		return ret;
	ret = becore_alloc_dma_buffer(becore, &becore->grid,
				      BECORE_GRID_SIZE, "YUVP grid");
	if (ret)
		return ret;
	ret = becore_alloc_dma_buffer(becore, &becore->output,
				      output_size, "YUVP output");
	if (ret)
		return ret;
	ret = becore_alloc_dma_buffer(becore, &becore->gtnr_output,
				      becore_gtnr_surface_size(), "GTNR output");
	if (ret)
		return ret;
	ret = becore_alloc_dma_buffer(becore, &becore->mcsc_output,
				      becore_mcsc_output_size(), "MCSC output");
	if (ret)
		return ret;
	ret = becore_alloc_cmdq_program(becore, BECORE_RGBP,
					BECORE_RGBP_HEADER_COUNT);
	if (ret)
		return ret;

	ret = becore_alloc_cmdq_program(becore, BECORE_YUVP,
					BECORE_YUVP_HEADER_COUNT);
	if (ret)
		return ret;
	ret = becore_alloc_cmdq_buffer(becore, &becore->gtnr_program,
				       BECORE_GTNR_HEADER_COUNT, "GTNR startup");
	if (ret)
		return ret;

	return becore_alloc_cmdq_buffer(becore, &becore->mcsc_program,
					BECORE_MCSC_HEADER_COUNT, "MCSC");
}

static int becore_clone_sgtable(struct sg_table *dst,
				struct sg_table *src)
{
	struct scatterlist *src_sg;
	struct scatterlist *dst_sg;
	unsigned int i;
	int ret;

	ret = sg_alloc_table(dst, src->orig_nents, GFP_KERNEL);
	if (ret)
		return ret;

	dst_sg = dst->sgl;
	for_each_sg(src->sgl, src_sg, src->orig_nents, i) {
		sg_set_page(dst_sg, sg_page(src_sg), src_sg->length,
			    src_sg->offset);
		dst_sg = sg_next(dst_sg);
	}

	return 0;
}

/**
 * exynos_becore_input_map() - map the BE-core input into its producer domain
 * @backend: BE-core platform device
 * @producer: device which will write the compressed Bayer object
 *
 * Each producer_acquire() returns one generation-tagged slot and its address
 * in @producer's DMA domain.  BE-core retains distinct mappings of the same
 * pages and never exposes those IOVAs to the producer.
 */
struct exynos_becore_input *
exynos_becore_input_map(struct device *backend, struct device *producer)
{
	struct becore_device *becore;
	struct exynos_becore_input *input;
	unsigned int i;
	int ret;

	if (!backend || !producer)
		return ERR_PTR(-EINVAL);
	becore = dev_get_drvdata(backend);
	if (!becore || !becore->inputs[0].buffer.sgt)
		return ERR_PTR(-EPROBE_DEFER);

	input = kzalloc_obj(*input, GFP_KERNEL);
	if (!input)
		return ERR_PTR(-ENOMEM);
	input->becore = becore;
	input->producer = get_device(producer);

	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		struct becore_dma_buffer *slot = &becore->inputs[i].buffer;

		ret = becore_clone_sgtable(&input->sgts[i], slot->sgt);
		if (ret)
			goto err_mappings;
		ret = dma_map_sgtable(producer, &input->sgts[i],
				      DMA_FROM_DEVICE, 0);
		if (ret) {
			sg_free_table(&input->sgts[i]);
			goto err_mappings;
		}
		input->dmas[i] = sg_dma_address(input->sgts[i].sgl);
		if (input->sgts[i].nents != 1 ||
		    sg_dma_len(input->sgts[i].sgl) < slot->size ||
		    upper_32_bits(input->dmas[i]) ||
		    upper_32_bits(input->dmas[i] + slot->size - 1)) {
			ret = -ERANGE;
			dma_unmap_sgtable(producer, &input->sgts[i],
					  DMA_FROM_DEVICE, 0);
			sg_free_table(&input->sgts[i]);
			goto err_mappings;
		}
	}
	mutex_lock(&becore->lock);
	if (becore->reset_failed) {
		ret = -EIO;
		mutex_unlock(&becore->lock);
		goto err_mappings;
	}
	if (becore->input_producer) {
		ret = -EBUSY;
		mutex_unlock(&becore->lock);
		goto err_mappings;
	}
	becore->input_producer = input;
	mutex_unlock(&becore->lock);

	return input;

err_mappings:
	while (i--) {
		dma_unmap_sgtable(producer, &input->sgts[i],
				  DMA_FROM_DEVICE, 0);
		sg_free_table(&input->sgts[i]);
	}
	put_device(input->producer);
	kfree(input);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(exynos_becore_input_map);

void exynos_becore_input_unmap(struct exynos_becore_input *input)
{
	struct becore_device *becore;
	unsigned int i;

	if (!input)
		return;
	becore = input->becore;

	mutex_lock(&becore->lock);
	if (WARN_ON_ONCE(becore->input_producer != input)) {
		mutex_unlock(&becore->lock);
		return;
	}
	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		if (becore->inputs[i].state == BECORE_INPUT_BACKEND ||
		    becore->inputs[i].state == BECORE_INPUT_QUARANTINED) {
			dev_crit(becore->dev,
				 "retaining producer mappings for active/quarantined input slot %u\n",
				 i);
			mutex_unlock(&becore->lock);
			return;
		}
	}
	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		struct becore_input_slot *slot = &becore->inputs[i];

		if (slot->state == BECORE_INPUT_PRODUCER)
			dma_sync_sgtable_for_cpu(input->producer,
						 &input->sgts[i], DMA_FROM_DEVICE);
		if (slot->state != BECORE_INPUT_BACKEND) {
			slot->state = BECORE_INPUT_FREE;
			slot->buffer.staged_bytes = 0;
			slot->producer_cookie = 0;
			slot->ready_sequence = 0;
		}
	}
	becore->input_producer = NULL;
	mutex_unlock(&becore->lock);

	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		dma_unmap_sgtable(input->producer, &input->sgts[i],
				  DMA_FROM_DEVICE, 0);
		sg_free_table(&input->sgts[i]);
	}
	put_device(input->producer);
	kfree(input);
}
EXPORT_SYMBOL_GPL(exynos_becore_input_unmap);

size_t exynos_becore_input_size(struct exynos_becore_input *input)
{
	return input->becore->inputs[0].buffer.size;
}
EXPORT_SYMBOL_GPL(exynos_becore_input_size);

static struct becore_input_slot *
becore_input_ticket(struct exynos_becore_input *input,
		    const struct exynos_becore_input_buffer *buffer)
{
	struct becore_input_slot *slot;

	if (!buffer || buffer->slot >= BECORE_INPUT_SLOT_COUNT)
		return NULL;
	slot = &input->becore->inputs[buffer->slot];
	if (slot->state != BECORE_INPUT_PRODUCER ||
	    slot->producer_cookie != buffer->cookie ||
	    input->dmas[buffer->slot] != buffer->dma ||
	    slot->buffer.size != buffer->size)
		return NULL;

	return slot;
}

int exynos_becore_input_producer_acquire(struct exynos_becore_input *input,
					 struct exynos_becore_input_buffer *buffer)
{
	struct becore_device *becore = input->becore;
	struct becore_input_slot *slot = NULL;
	u64 cookie;
	unsigned int i;
	int ret = 0;

	if (!buffer)
		return -EINVAL;

	mutex_lock(&becore->lock);
	if (becore->input_producer != input) {
		ret = -EINVAL;
		goto unlock;
	}
	if (becore->reset_failed) {
		ret = -EIO;
		goto unlock;
	}
	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++)
		if (becore->inputs[i].state == BECORE_INPUT_FREE) {
			slot = &becore->inputs[i];
			break;
		}
	if (!slot) {
		ret = -EBUSY;
		goto unlock;
	}

	/* Discard any cached CPU copy before the front end overwrites it. */
	dma_sync_sgtable_for_device(becore->dev, slot->buffer.sgt,
				    DMA_BIDIRECTIONAL);
	dma_sync_sgtable_for_device(input->producer, &input->sgts[i],
				    DMA_FROM_DEVICE);
	slot->buffer.staged_bytes = 0;
	slot->ready_sequence = 0;
	cookie = ++becore->producer_sequence;
	if (!cookie)
		cookie = ++becore->producer_sequence;
	slot->producer_cookie = cookie;
	slot->state = BECORE_INPUT_PRODUCER;
	*buffer = (struct exynos_becore_input_buffer) {
		.dma = input->dmas[i],
		.size = slot->buffer.size,
		.cookie = cookie,
		.slot = i,
	};

unlock:
	mutex_unlock(&becore->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(exynos_becore_input_producer_acquire);

int exynos_becore_input_producer_complete(struct exynos_becore_input *input,
					  const struct exynos_becore_input_buffer *buffer)
{
	struct becore_device *becore = input->becore;
	struct becore_input_slot *slot;
	int ret = 0;

	mutex_lock(&becore->lock);
	if (becore->input_producer != input) {
		ret = -EINVAL;
		goto unlock;
	}
	slot = becore_input_ticket(input, buffer);
	if (!slot) {
		ret = -EINVAL;
		goto unlock;
	}

	/* The caller has quiesced the producer at a completed-frame boundary. */
	dma_sync_sgtable_for_cpu(input->producer, &input->sgts[buffer->slot],
				 DMA_FROM_DEVICE);
	dma_sync_sgtable_for_device(becore->dev, slot->buffer.sgt,
				    DMA_TO_DEVICE);
	slot->buffer.staged_bytes = slot->buffer.size;
	slot->producer_cookie = 0;
	slot->ready_sequence = ++becore->input_sequence;
	slot->state = BECORE_INPUT_READY;
	if (becore->video_streaming)
		schedule_work(&becore->video_work);

unlock:
	mutex_unlock(&becore->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(exynos_becore_input_producer_complete);

void exynos_becore_input_producer_abort(struct exynos_becore_input *input,
					const struct exynos_becore_input_buffer *buffer)
{
	struct becore_device *becore = input->becore;
	struct becore_input_slot *slot;

	mutex_lock(&becore->lock);
	if (becore->input_producer == input) {
		slot = becore_input_ticket(input, buffer);
		if (!slot)
			goto unlock;
		dma_sync_sgtable_for_cpu(input->producer,
					 &input->sgts[buffer->slot],
					 DMA_FROM_DEVICE);
		slot->buffer.staged_bytes = 0;
		slot->producer_cookie = 0;
		slot->ready_sequence = 0;
		slot->state = BECORE_INPUT_FREE;
	}

unlock:
	mutex_unlock(&becore->lock);
}
EXPORT_SYMBOL_GPL(exynos_becore_input_producer_abort);

static ssize_t becore_stage_write(struct becore_device *becore,
				  const char __user *buf, size_t count,
				  loff_t *ppos, void *staged, size_t capacity,
				  size_t *staged_bytes, u32 *generation)
{
	ssize_t ret = count;

	if (!count)
		return 0;

	mutex_lock(&becore->lock);
	if (becore->reset_failed) {
		ret = -EIO;
		goto unlock;
	}
	if (becore->video_streaming) {
		ret = -EBUSY;
		goto unlock;
	}
	if (becore->running) {
		ret = -EBUSY;
		goto unlock;
	}
	if (staged == becore->inputs[0].buffer.cpu &&
	    becore->inputs[0].state != BECORE_INPUT_FREE) {
		ret = -EBUSY;
		goto unlock;
	}
	if (*ppos < 0 || *ppos > capacity) {
		ret = -EINVAL;
		goto unlock;
	}
	if (count > capacity - *ppos) {
		ret = -EFBIG;
		goto unlock;
	}
	if (*ppos == 0)
		*staged_bytes = 0;
	if (*ppos != *staged_bytes) {
		ret = -ESPIPE;
		goto unlock;
	}
	if (copy_from_user((u8 *)staged + *staged_bytes, buf, count)) {
		/* A real partial replacement is never considered a valid stage. */
		*staged_bytes = 0;
		ret = -EFAULT;
		goto unlock;
	}

	*staged_bytes += count;
	*ppos += count;
	if (generation && *staged_bytes == capacity)
		(*generation)++;

unlock:
	mutex_unlock(&becore->lock);
	return ret;
}

static ssize_t becore_recipe_read(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;
	ssize_t ret;

	mutex_lock(&becore->lock);
	if (becore->reset_failed)
		ret = -EIO;
	else if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else
		ret = simple_read_from_buffer(buf, count, ppos, becore->recipe,
					      becore->recipe_staged_bytes);
	mutex_unlock(&becore->lock);

	return ret;
}

static ssize_t becore_recipe_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;

	return becore_stage_write(becore, buf, count, ppos, becore->recipe,
				  BECORE_RECIPE_BYTES,
				  &becore->recipe_staged_bytes,
				  &becore->recipe_generation);
}

static const struct file_operations becore_recipe_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_recipe_read,
	.write = becore_recipe_write,
	.llseek = default_llseek,
};

static ssize_t becore_gtnr_recipe_read(struct file *file, char __user *buf,
				       size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;
	ssize_t ret;

	mutex_lock(&becore->lock);
	if (becore->reset_failed)
		ret = -EIO;
	else if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else
		ret = simple_read_from_buffer(buf, count, ppos,
					      becore->gtnr_recipe,
					      becore->gtnr_recipe_staged_bytes);
	mutex_unlock(&becore->lock);

	return ret;
}

static ssize_t becore_gtnr_recipe_write(struct file *file,
					const char __user *buf, size_t count,
					loff_t *ppos)
{
	struct becore_device *becore = file->private_data;

	return becore_stage_write(becore, buf, count, ppos,
				  becore->gtnr_recipe, BECORE_GTNR_RECIPE_BYTES,
				  &becore->gtnr_recipe_staged_bytes,
				  &becore->gtnr_recipe_generation);
}

static const struct file_operations becore_gtnr_recipe_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_gtnr_recipe_read,
	.write = becore_gtnr_recipe_write,
	.llseek = default_llseek,
};

static ssize_t becore_mcsc_recipe_read(struct file *file, char __user *buf,
				       size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;
	ssize_t ret;

	mutex_lock(&becore->lock);
	if (becore->reset_failed)
		ret = -EIO;
	else if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else
		ret = simple_read_from_buffer(buf, count, ppos,
					      becore->mcsc_recipe,
					      becore->mcsc_recipe_staged_bytes);
	mutex_unlock(&becore->lock);

	return ret;
}

static ssize_t becore_mcsc_recipe_write(struct file *file,
					const char __user *buf, size_t count,
					loff_t *ppos)
{
	struct becore_device *becore = file->private_data;

	return becore_stage_write(becore, buf, count, ppos,
				  becore->mcsc_recipe, BECORE_MCSC_RECIPE_BYTES,
				  &becore->mcsc_recipe_staged_bytes,
				  &becore->mcsc_recipe_generation);
}

static const struct file_operations becore_mcsc_recipe_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_mcsc_recipe_read,
	.write = becore_mcsc_recipe_write,
	.llseek = default_llseek,
};

static ssize_t becore_input_write(struct file *file, const char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;
	struct becore_dma_buffer *input = &becore->inputs[0].buffer;

	return becore_stage_write(becore, buf, count, ppos,
				  input->cpu, input->size,
				  &input->staged_bytes, NULL);
}

static const struct file_operations becore_input_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = becore_input_write,
	.llseek = default_llseek,
};

static ssize_t becore_grid_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;

	return becore_stage_write(becore, buf, count, ppos,
				  becore->grid.cpu, becore->grid.size,
				  &becore->grid.staged_bytes, NULL);
}

static const struct file_operations becore_grid_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = becore_grid_write,
	.llseek = default_llseek,
};

static ssize_t becore_output_read(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;
	ssize_t ret;

	mutex_lock(&becore->lock);
	if (becore->reset_failed)
		ret = -EIO;
	else if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else if (!becore->completed_generation)
		ret = -ENODATA;
	else
		ret = simple_read_from_buffer(buf, count, ppos,
					      becore->output.cpu,
					      becore->completed_output_size);
	mutex_unlock(&becore->lock);

	return ret;
}

static const struct file_operations becore_output_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_output_read,
	.llseek = default_llseek,
};

static ssize_t becore_mcsc_output_read(struct file *file, char __user *buf,
				       size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;
	ssize_t ret;

	mutex_lock(&becore->lock);
	if (becore->reset_failed || becore->output_quarantined)
		ret = -EIO;
	else if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else if (!becore->mcsc_completed_generation)
		ret = -ENODATA;
	else
		ret = simple_read_from_buffer(buf, count, ppos,
					      becore->mcsc_output.cpu,
					      becore->mcsc_completed_output_size);
	mutex_unlock(&becore->lock);

	return ret;
}

static const struct file_operations becore_mcsc_output_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_mcsc_output_read,
	.llseek = default_llseek,
};

static ssize_t becore_encoded_read(struct file *file, char __user *buf,
				   size_t count, loff_t *ppos,
				   enum becore_block_id id)
{
	struct becore_device *becore = file->private_data;
	struct becore_cmdq_program *program = &becore->program[id];
	ssize_t ret;

	mutex_lock(&becore->lock);
	if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else if (!becore->completed_generation)
		ret = -ENODATA;
	else
		ret = simple_read_from_buffer(buf, count, ppos, program->cpu,
					      program->size);
	mutex_unlock(&becore->lock);

	return ret;
}

static ssize_t becore_rgbp_encoded_read(struct file *file, char __user *buf,
					size_t count, loff_t *ppos)
{
	return becore_encoded_read(file, buf, count, ppos, BECORE_RGBP);
}

static const struct file_operations becore_rgbp_encoded_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_rgbp_encoded_read,
	.llseek = default_llseek,
};

static ssize_t becore_yuvp_encoded_read(struct file *file, char __user *buf,
					size_t count, loff_t *ppos)
{
	return becore_encoded_read(file, buf, count, ppos, BECORE_YUVP);
}

static const struct file_operations becore_yuvp_encoded_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_yuvp_encoded_read,
	.llseek = default_llseek,
};

static ssize_t becore_gtnr_encoded_read(struct file *file, char __user *buf,
					size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;
	ssize_t ret;

	mutex_lock(&becore->lock);
	if (becore->reset_failed)
		ret = -EIO;
	else if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else if (becore->gtnr_recipe_staged_bytes != BECORE_GTNR_RECIPE_BYTES ||
		 becore->gtnr_encoded_generation != becore->gtnr_recipe_generation)
		ret = -ENODATA;
	else
		ret = simple_read_from_buffer(buf, count, ppos,
					      becore->gtnr_program.cpu,
					      becore->gtnr_program.size);
	mutex_unlock(&becore->lock);

	return ret;
}

static const struct file_operations becore_gtnr_encoded_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_gtnr_encoded_read,
	.llseek = default_llseek,
};

static int becore_gtnr_encode_set(void *data, u64 value)
{
	struct becore_device *becore = data;
	int ret;

	if (value != 1)
		return -EINVAL;

	mutex_lock(&becore->lock);
	if (becore->reset_failed) {
		ret = -EIO;
	} else if (becore->running || becore->video_streaming) {
		ret = -EBUSY;
	} else {
		ret = becore_gtnr_recipe_validate(becore);
		if (!ret)
			ret = becore_encode_gtnr(becore);
		if (!ret)
			becore->gtnr_encoded_generation =
				becore->gtnr_recipe_generation;
	}
	mutex_unlock(&becore->lock);

	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(becore_gtnr_encode_fops, NULL,
			 becore_gtnr_encode_set, "%llu\n");

static ssize_t becore_mcsc_encoded_read(struct file *file, char __user *buf,
					size_t count, loff_t *ppos)
{
	struct becore_device *becore = file->private_data;
	ssize_t ret;

	mutex_lock(&becore->lock);
	if (becore->reset_failed)
		ret = -EIO;
	else if (becore->running || becore->video_streaming)
		ret = -EBUSY;
	else if (becore->mcsc_recipe_staged_bytes != BECORE_MCSC_RECIPE_BYTES ||
		 becore->mcsc_encoded_generation != becore->mcsc_recipe_generation)
		ret = -ENODATA;
	else
		ret = simple_read_from_buffer(buf, count, ppos,
					      becore->mcsc_program.cpu,
					      becore->mcsc_program.size);
	mutex_unlock(&becore->lock);

	return ret;
}

static const struct file_operations becore_mcsc_encoded_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = becore_mcsc_encoded_read,
	.llseek = default_llseek,
};

static int becore_mcsc_encode_set(void *data, u64 value)
{
	struct becore_device *becore = data;
	int ret;

	if (value != 1)
		return -EINVAL;

	mutex_lock(&becore->lock);
	if (becore->reset_failed) {
		ret = -EIO;
	} else if (becore->running || becore->video_streaming) {
		ret = -EBUSY;
	} else {
		ret = becore_mcsc_recipe_validate(becore);
		if (!ret)
			ret = becore_encode_mcsc(becore,
						 BECORE_MCSC_INPUT_CAPTURED_VOTF);
		if (!ret)
			becore->mcsc_encoded_generation =
				becore->mcsc_recipe_generation;
		if (!ret)
			becore->mcsc_encoded_transport =
				BECORE_MCSC_INPUT_CAPTURED_VOTF;
	}
	mutex_unlock(&becore->lock);

	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(becore_mcsc_encode_fops, NULL,
			 becore_mcsc_encode_set, "%llu\n");

static void becore_clear_pending_irqs(struct becore_block *block)
{
	writel_relaxed(U32_MAX, block->base + BECORE_INT0_CLEAR);
	writel_relaxed(U32_MAX, block->base + BECORE_INT1_CLEAR);
	writel_relaxed(U32_MAX, block->base + BECORE_CMDQ_INT_CLEAR);
}

static void becore_publish_program(struct becore_device *becore,
				   enum becore_block_id id)
{
	struct becore_cmdq_program *program = id == BECORE_MCSC ?
		&becore->mcsc_program : &becore->program[id];
	struct becore_block *block = &becore->blocks[id];

	writel_relaxed(lower_32_bits(program->dma),
		       block->base + BECORE_CMDQ_QUE_CMD_L);
	writel_relaxed(BECORE_CMDQ_MODE | program->header_count,
		       block->base + BECORE_CMDQ_QUE_CMD_M);
	writel_relaxed(0xff, block->base + BECORE_CMDQ_QUE_CMD_H);
	writel_relaxed(1, block->base + BECORE_CMDQ_QUE_CMD_START);
}

static void becore_measure_buffer(const struct becore_dma_buffer *buffer,
				  u32 *changed_bytes, u32 *first_changed)
{
	const u8 *output = buffer->cpu;
	u32 changed = 0;
	u32 first = U32_MAX;
	u32 i;

	for (i = 0; i < buffer->size; i++) {
		if (output[i] == 0xa5)
			continue;
		if (first == U32_MAX)
			first = i;
		changed++;
	}
	*changed_bytes = changed;
	*first_changed = first;
}

static int becore_run_stage(struct becore_device *becore, u32 blocks)
{
	unsigned long flags;
	unsigned long waited;
	unsigned int i;
	int ret;

	for (i = 0; i < BECORE_NUM_BLOCKS; i++)
		if (blocks & BIT(i))
			becore_clear_pending_irqs(&becore->blocks[i]);
	reinit_completion(&becore->run_completion);

	spin_lock_irqsave(&becore->run_lock, flags);
	becore->expected_mask = blocks;
	becore->start_issued = false;
	if (becore->abort_run) {
		spin_unlock_irqrestore(&becore->run_lock, flags);
		return -ECANCELED;
	}
	spin_unlock_irqrestore(&becore->run_lock, flags);

	/* Publish every DMA ownership transition before starting the next stage. */
	dma_wmb();
	for (i = 0; i < BECORE_NUM_BLOCKS; i++)
		if (blocks & BIT(i))
			becore_publish_program(becore, i);
	mutex_unlock(&becore->lock);

	waited = wait_for_completion_timeout(&becore->run_completion,
					     msecs_to_jiffies(BECORE_RUN_TIMEOUT_MS));

	mutex_lock(&becore->lock);
	spin_lock_irqsave(&becore->run_lock, flags);
	if (!waited)
		ret = -ETIMEDOUT;
	else if (becore->irq_error)
		ret = -EIO;
	else if (becore->abort_run)
		ret = -ECANCELED;
	else if ((becore->frame_done_mask & blocks) != blocks)
		ret = -EIO;
	else
		ret = 0;
	spin_unlock_irqrestore(&becore->run_lock, flags);

	return ret;
}

static struct becore_input_slot *
becore_next_input(struct becore_device *becore, bool allow_staged)
{
	struct becore_input_slot *next = NULL;
	unsigned int i;

	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		struct becore_input_slot *slot = &becore->inputs[i];

		if (slot->state != BECORE_INPUT_READY)
			continue;
		if (!next || slot->ready_sequence < next->ready_sequence)
			next = slot;
	}
	if (next)
		return next;

	/* Slot zero remains the upload-and-repeat diagnostic oracle. */
	if (allow_staged && becore->inputs[0].state == BECORE_INPUT_FREE)
		return &becore->inputs[0];

	return NULL;
}

static int becore_run_frame(struct becore_device *becore, u32 output_profile,
			    bool ready_only, void *capture_output,
			    bool packed_output, bool run_mcsc)
{
	unsigned long flags;
	bool diagnostic_output = !capture_output;
	bool input_claimed = false;
	int pm_ret;
	int ret;
	u32 i;

	mutex_lock(&becore->lock);
	if (diagnostic_output && becore->video_streaming) {
		ret = -EBUSY;
		goto unlock;
	}
	if (!diagnostic_output && !becore->video_streaming) {
		ret = -ECANCELED;
		goto unlock;
	}
	if (becore->running) {
		ret = -EBUSY;
		goto unlock;
	}
	if (becore->reset_failed) {
		ret = -EIO;
		goto record_error;
	}
	if (output_profile >= BECORE_YUVP_OUTPUT_PROFILE_COUNT) {
		ret = -EINVAL;
		goto record_error;
	}
	if (packed_output && output_profile != BECORE_YUVP_OUTPUT_P010) {
		ret = -EINVAL;
		goto record_error;
	}
	if (run_mcsc && (output_profile != BECORE_YUVP_OUTPUT_SBWCL ||
			 packed_output)) {
		ret = -EINVAL;
		goto record_error;
	}
	becore->run_input = becore_next_input(becore, !ready_only);
	if (!becore->run_input) {
		if (ready_only) {
			ret = -ENODATA;
			goto unlock;
		}
		ret = -EBUSY;
		goto record_error;
	}
	becore->active_output_profile = output_profile;
	becore->active_output_packed = packed_output;
	becore->active_output_dma = becore->output.dma;
	becore->active_output_size = becore_active_output_size(becore);
	if (!becore->active_output_dma ||
	    upper_32_bits(becore->active_output_dma) ||
	    upper_32_bits(becore->active_output_dma +
			  becore->active_output_size - 1) ||
	    becore->active_output_size > becore->output.size) {
		ret = -EINVAL;
		goto record_error;
	}
	ret = becore_recipe_validate(becore);
	if (ret)
		goto record_error;
	if (run_mcsc) {
		ret = becore_mcsc_recipe_validate(becore);
		if (ret)
			goto record_error;
	}
	becore->run_input->state = BECORE_INPUT_BACKEND;
	input_claimed = true;

	ret = pm_runtime_resume_and_get(becore->dev);
	if (ret)
		goto record_error;
	if (becore->reset_failed) {
		/* The runtime callback deliberately retains every supplier. */
		ret = -EIO;
		goto record_error;
	}

	ret = becore_encode_programs(becore);
	if (ret)
		goto put_power;
	if (run_mcsc) {
		ret = becore_encode_mcsc(becore, BECORE_MCSC_INPUT_MEMORY);
		if (ret)
			goto put_power;
		becore->mcsc_encoded_generation = becore->mcsc_recipe_generation;
		becore->mcsc_encoded_transport = BECORE_MCSC_INPUT_MEMORY;
	}

	if (diagnostic_output) {
		memset(becore->output.cpu, 0xa5, becore->output.size);
		if (run_mcsc)
			memset(becore->mcsc_output.cpu, 0xa5,
			       becore->mcsc_output.size);
	}
	becore->output_changed_bytes = 0;
	becore->output_first_changed = U32_MAX;
	becore->mcsc_output_changed_bytes = 0;
	becore->mcsc_output_first_changed = U32_MAX;
	becore->mcsc_completed_generation = 0;
	becore->mcsc_completed_output_size = 0;
	for (i = 0; i < BECORE_NUM_BLOCKS; i++) {
		WRITE_ONCE(becore->blocks[i].last_int0, 0);
		WRITE_ONCE(becore->blocks[i].last_int1, 0);
		WRITE_ONCE(becore->blocks[i].last_cmdq_int, 0);
		atomic64_set(&becore->blocks[i].int0_count, 0);
		atomic64_set(&becore->blocks[i].int1_count, 0);
	}
	spin_lock_irqsave(&becore->run_lock, flags);
	becore->cmdq_hold_mask = 0;
	becore->frame_done_mask = 0;
	becore->expected_mask = 0;
	becore->abort_run = false;
	becore->irq_error = false;
	becore->running = true;
	becore->run_generation++;
	spin_unlock_irqrestore(&becore->run_lock, flags);
	becore->active_mcsc = run_mcsc;

	/* Move staged or producer-written Bayer pages into RGBP's DMA domain. */
	dma_sync_sgtable_for_device(becore->dev, becore->run_input->buffer.sgt,
				    DMA_TO_DEVICE);
	ret = becore_run_stage(becore, BECORE_YUVP_STAGE_BLOCKS);
	if (!ret && run_mcsc)
		ret = becore_run_stage(becore, BIT(BECORE_MCSC));

	spin_lock_irqsave(&becore->run_lock, flags);
	becore->running = false;
	spin_unlock_irqrestore(&becore->run_lock, flags);

	/* runtime_suspend synchronizes IRQs and resets all four processors. */
	pm_ret = pm_runtime_put_sync(becore->dev);
	if (pm_ret < 0) {
		/* Match probe: never leave a failed-reset device at usage zero. */
		pm_runtime_get_noresume(becore->dev);
		/*
		 * Neither DMA mapping may be returned after an unproven stop.
		 * The output is driver-owned, so userspace's vb2 buffer was never
		 * exposed to the processors and remains safe to return with ERROR.
		 */
		becore->run_input->state = BECORE_INPUT_QUARANTINED;
		becore->output_quarantined = true;
		becore->completed_generation = 0;
		becore->completed_output_size = 0;
		becore->mcsc_completed_generation = 0;
		becore->mcsc_completed_output_size = 0;
		if (!ret)
			ret = pm_ret;
	} else {
		dma_sync_sgtable_for_cpu(becore->dev,
					 becore->run_input->buffer.sgt,
					 DMA_TO_DEVICE);
		becore->run_input->state = BECORE_INPUT_FREE;
		becore->run_input->ready_sequence = 0;
	}
	becore->run_input = NULL;
	input_claimed = false;
	if (pm_ret >= 0)
		dma_rmb();
	if (capture_output && !ret && pm_ret >= 0)
		memcpy(capture_output, becore->output.cpu,
		       becore->active_output_size);
	if (diagnostic_output && pm_ret >= 0) {
		becore_measure_buffer(&becore->output,
				      &becore->output_changed_bytes,
				      &becore->output_first_changed);
		becore->completed_generation = becore->run_generation;
		becore->completed_output_size = becore->active_output_size;
		if (run_mcsc) {
			becore_measure_buffer(&becore->mcsc_output,
					      &becore->mcsc_output_changed_bytes,
					      &becore->mcsc_output_first_changed);
			if (!ret) {
				becore->mcsc_completed_generation =
					becore->run_generation;
				becore->mcsc_completed_output_size =
					becore_mcsc_output_size();
			}
		}
	}
	becore->last_run_result = ret;
	mutex_unlock(&becore->lock);

	return ret;

put_power:
	pm_ret = pm_runtime_put_sync(becore->dev);
	if (pm_ret < 0) {
		pm_runtime_get_noresume(becore->dev);
		if (!ret)
			ret = pm_ret;
	}
record_error:
	if (input_claimed) {
		becore->run_input->state = BECORE_INPUT_FREE;
		becore->run_input->ready_sequence = 0;
	}
	becore->run_input = NULL;
	becore->last_run_result = ret;
unlock:
	mutex_unlock(&becore->lock);
	return ret;
}

static int becore_run_set(void *data, u64 value)
{
	struct becore_device *becore = data;

	if (value != 1)
		return -EINVAL;
	if (READ_ONCE(becore->video_streaming))
		return -EBUSY;

	return becore_run_frame(becore, READ_ONCE(becore->output_profile),
				false, NULL, false, false);
}

static int becore_run_get(void *data, u64 *value)
{
	struct becore_device *becore = data;

	mutex_lock(&becore->lock);
	*value = becore->running;
	mutex_unlock(&becore->lock);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(becore_run_fops, becore_run_get, becore_run_set,
			 "%llu\n");

static int becore_mcsc_run_set(void *data, u64 value)
{
	struct becore_device *becore = data;

	if (value != 1)
		return -EINVAL;
	if (READ_ONCE(becore->video_streaming))
		return -EBUSY;

	return becore_run_frame(becore, BECORE_YUVP_OUTPUT_SBWCL,
				false, NULL, false, true);
}
DEFINE_DEBUGFS_ATTRIBUTE(becore_mcsc_run_fops, NULL, becore_mcsc_run_set,
			 "%llu\n");

static int becore_cancel_set(void *data, u64 value)
{
	struct becore_device *becore = data;
	unsigned long flags;
	bool cancelled = false;

	if (value != 1)
		return -EINVAL;

	mutex_lock(&becore->lock);
	if (becore->video_streaming) {
		mutex_unlock(&becore->lock);
		return -EBUSY;
	}
	spin_lock_irqsave(&becore->run_lock, flags);
	if (becore->running) {
		becore->abort_run = true;
		cancelled = true;
	}
	spin_unlock_irqrestore(&becore->run_lock, flags);
	if (cancelled)
		complete(&becore->run_completion);
	mutex_unlock(&becore->lock);

	return cancelled ? 0 : -EALREADY;
}
DEFINE_DEBUGFS_ATTRIBUTE(becore_cancel_fops, NULL, becore_cancel_set, "%llu\n");

/* ---- processed P010 capture queue -------------------------------------- */

static void becore_video_fill_pix(struct v4l2_pix_format *pix)
{
	const struct becore_yuvp_output_profile *profile =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_P010];

	pix->width = profile->width;
	pix->height = profile->height;
	pix->pixelformat = V4L2_PIX_FMT_P010;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = becore_yuvp_output_stride(profile);
	/* Single-planar P010 places UV immediately after the luma rows. */
	pix->sizeimage = becore_yuvp_packed_output_size(profile);
	/* The captured recipe does not describe its range or YCbCr matrix. */
	pix->colorspace = V4L2_COLORSPACE_RAW;
	pix->flags = 0;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_DEFAULT;
	pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static void becore_video_return_all(struct becore_device *becore,
				    enum vb2_buffer_state state)
{
	struct becore_video_buffer *buf, *tmp;
	LIST_HEAD(done);

	spin_lock_irq(&becore->queue_lock);
	list_splice_tail_init(&becore->queued_outputs, &done);
	spin_unlock_irq(&becore->queue_lock);

	list_for_each_entry_safe(buf, tmp, &done, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
}

static void becore_video_fail(struct becore_device *becore,
			      struct becore_video_buffer *buf)
{
	mutex_lock(&becore->lock);
	becore->video_streaming = false;
	mutex_unlock(&becore->lock);

	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	vb2_queue_error(&becore->queue);
	becore_video_return_all(becore, VB2_BUF_STATE_ERROR);
}

static void becore_video_work(struct work_struct *work)
{
	struct becore_device *becore =
		container_of(work, struct becore_device, video_work);

	for (;;) {
		struct becore_video_buffer *buf;
		void *vaddr;
		int ret;

		mutex_lock(&becore->lock);
		if (!becore->video_streaming) {
			mutex_unlock(&becore->lock);
			return;
		}
		mutex_unlock(&becore->lock);

		spin_lock_irq(&becore->queue_lock);
		if (list_empty(&becore->queued_outputs)) {
			spin_unlock_irq(&becore->queue_lock);
			return;
		}
		buf = list_first_entry(&becore->queued_outputs,
				       struct becore_video_buffer, list);
		list_del(&buf->list);
		spin_unlock_irq(&becore->queue_lock);

		vaddr = vb2_plane_vaddr(&buf->vb.vb2_buf, 0);
		if (WARN_ON_ONCE(!vaddr)) {
			becore_video_fail(becore, buf);
			return;
		}
		ret = becore_run_frame(becore, BECORE_YUVP_OUTPUT_P010,
				       true, vaddr, true, false);
		if (ret == -ENODATA || ret == -EBUSY) {
			spin_lock_irq(&becore->queue_lock);
			list_add(&buf->list, &becore->queued_outputs);
			spin_unlock_irq(&becore->queue_lock);
			return;
		}
		if (ret) {
			bool stopping;

			mutex_lock(&becore->lock);
			stopping = !becore->video_streaming;
			mutex_unlock(&becore->lock);
			if (ret == -ECANCELED && stopping) {
				vb2_buffer_done(&buf->vb.vb2_buf,
						VB2_BUF_STATE_ERROR);
				return;
			}
			becore_video_fail(becore, buf);
			return;
		}

		buf->vb.vb2_buf.timestamp = ktime_get_ns();
		mutex_lock(&becore->lock);
		buf->vb.sequence = becore->video_sequence++;
		mutex_unlock(&becore->lock);
		buf->vb.field = V4L2_FIELD_NONE;
		vb2_set_plane_payload(&buf->vb.vb2_buf, 0,
				      becore->active_output_size);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
	}
}

static int becore_queue_setup(struct vb2_queue *q, unsigned int *nbufs,
			      unsigned int *nplanes, unsigned int sizes[],
			      struct device *alloc_devs[])
{
	struct v4l2_pix_format pix;

	becore_video_fill_pix(&pix);
	if (*nplanes) {
		if (*nplanes != 1 || sizes[0] < pix.sizeimage)
			return -EINVAL;
		return 0;
	}

	*nplanes = 1;
	sizes[0] = pix.sizeimage;

	return 0;
}

static int becore_buf_prepare(struct vb2_buffer *vb)
{
	struct v4l2_pix_format pix;

	becore_video_fill_pix(&pix);
	if (vb2_plane_size(vb, 0) < pix.sizeimage ||
	    !vb2_plane_vaddr(vb, 0))
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, pix.sizeimage);

	return 0;
}

static void becore_buf_queue(struct vb2_buffer *vb)
{
	struct becore_device *becore = vb2_get_drv_priv(vb->vb2_queue);
	struct becore_video_buffer *buf =
		to_becore_video_buffer(to_vb2_v4l2_buffer(vb));

	spin_lock_irq(&becore->queue_lock);
	list_add_tail(&buf->list, &becore->queued_outputs);
	spin_unlock_irq(&becore->queue_lock);

	if (READ_ONCE(becore->video_streaming))
		schedule_work(&becore->video_work);
}

static int becore_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct becore_device *becore = vb2_get_drv_priv(q);
	int ret;

	mutex_lock(&becore->lock);
	if (becore->video_streaming || becore->running) {
		ret = -EBUSY;
		goto unlock;
	}
	ret = becore_recipe_header_validate(becore);
	if (ret)
		goto unlock;
	if (becore->grid.staged_bytes != BECORE_GRID_SIZE) {
		ret = -EINVAL;
		goto unlock;
	}
	if (becore->reset_failed) {
		ret = -EIO;
		goto unlock;
	}

	becore->video_sequence = 0;
	/* The shared driver-owned output now becomes the video bounce buffer. */
	becore->completed_generation = 0;
	becore->completed_output_size = 0;
	becore->video_streaming = true;
	mutex_unlock(&becore->lock);
	schedule_work(&becore->video_work);

	return 0;

unlock:
	mutex_unlock(&becore->lock);
	becore_video_return_all(becore, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void becore_stop_streaming(struct vb2_queue *q)
{
	struct becore_device *becore = vb2_get_drv_priv(q);
	unsigned long flags;
	bool cancel = false;

	mutex_lock(&becore->lock);
	becore->video_streaming = false;
	spin_lock_irqsave(&becore->run_lock, flags);
	if (becore->running) {
		becore->abort_run = true;
		cancel = true;
	}
	spin_unlock_irqrestore(&becore->run_lock, flags);
	if (cancel)
		complete(&becore->run_completion);
	mutex_unlock(&becore->lock);

	cancel_work_sync(&becore->video_work);
	becore_video_return_all(becore, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops becore_vb2_ops = {
	.queue_setup = becore_queue_setup,
	.buf_prepare = becore_buf_prepare,
	.buf_queue = becore_buf_queue,
	.start_streaming = becore_start_streaming,
	.stop_streaming = becore_stop_streaming,
};

static int becore_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strscpy(cap->driver, "exynos-becore", sizeof(cap->driver));
	strscpy(cap->card, "zumapro BE-core P010", sizeof(cap->card));

	return 0;
}

static int becore_enum_fmt(struct file *file, void *priv,
			   struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;

	f->pixelformat = V4L2_PIX_FMT_P010;

	return 0;
}

static int becore_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	becore_video_fill_pix(&f->fmt.pix);

	return 0;
}

static int becore_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct becore_device *becore = video_drvdata(file);

	if (vb2_is_busy(&becore->queue))
		return -EBUSY;

	becore_video_fill_pix(&f->fmt.pix);

	return 0;
}

static int becore_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fsize)
{
	const struct becore_yuvp_output_profile *profile =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_P010];

	if (fsize->index || fsize->pixel_format != V4L2_PIX_FMT_P010)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = profile->width;
	fsize->discrete.height = profile->height;

	return 0;
}

static const struct v4l2_ioctl_ops becore_ioctl_ops = {
	.vidioc_querycap = becore_querycap,
	.vidioc_enum_fmt_vid_cap = becore_enum_fmt,
	.vidioc_g_fmt_vid_cap = becore_g_fmt,
	.vidioc_s_fmt_vid_cap = becore_s_fmt,
	.vidioc_try_fmt_vid_cap = becore_g_fmt,
	.vidioc_enum_framesizes = becore_enum_framesizes,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations becore_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
	.unlocked_ioctl = video_ioctl2,
};

static const struct video_device becore_video_template = {
	.name = "exynos-becore P010 capture",
	.fops = &becore_fops,
	.ioctl_ops = &becore_ioctl_ops,
	.release = video_device_release_empty,
	.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING,
	.vfl_dir = VFL_DIR_RX,
};

static int becore_status_show(struct seq_file *s, void *unused)
{
	static const char * const input_state_names[] = {
		[BECORE_INPUT_FREE] = "free",
		[BECORE_INPUT_PRODUCER] = "producer",
		[BECORE_INPUT_READY] = "ready",
		[BECORE_INPUT_BACKEND] = "backend",
		[BECORE_INPUT_QUARANTINED] = "quarantined",
	};
	struct becore_device *becore = s->private;
	struct list_head *pos;
	unsigned long flags;
	unsigned int queued_outputs = 0;
	u32 cmdq_hold_mask;
	u32 frame_done_mask;
	u32 expected_mask;
	bool start_issued;
	bool irq_error;
	bool running;
	bool video_streaming;
	u32 i;

	mutex_lock(&becore->lock);
	spin_lock_irqsave(&becore->run_lock, flags);
	running = becore->running;
	cmdq_hold_mask = becore->cmdq_hold_mask;
	frame_done_mask = becore->frame_done_mask;
	expected_mask = becore->expected_mask;
	start_issued = becore->start_issued;
	irq_error = becore->irq_error;
	spin_unlock_irqrestore(&becore->run_lock, flags);
	spin_lock_irqsave(&becore->queue_lock, flags);
	list_for_each(pos, &becore->queued_outputs)
		queued_outputs++;
	spin_unlock_irqrestore(&becore->queue_lock, flags);
	video_streaming = becore->video_streaming;
	seq_printf(s, "running          %u\n", running);
	seq_printf(s, "video_queue      streaming %u, queued %u, sequence %u\n",
		   video_streaming, queued_outputs, becore->video_sequence);
	seq_printf(s, "runtime          %s\n",
		   pm_runtime_status_suspended(becore->dev) ? "suspended" : "active");
	seq_printf(s, "recipe           %zu/%u bytes, generation %u\n",
		   becore->recipe_staged_bytes, BECORE_RECIPE_BYTES,
		   becore->recipe_generation);
	seq_printf(s, "gtnr_recipe      %zu/%u bytes, generation %u, encoded %u\n",
		   becore->gtnr_recipe_staged_bytes, BECORE_GTNR_RECIPE_BYTES,
		   becore->gtnr_recipe_generation,
		   becore->gtnr_encoded_generation);
	seq_printf(s, "mcsc_recipe      %zu/%u bytes, generation %u, encoded %u\n",
		   becore->mcsc_recipe_staged_bytes, BECORE_MCSC_RECIPE_BYTES,
		   becore->mcsc_recipe_generation,
		   becore->mcsc_encoded_generation);
	seq_printf(s, "mcsc_transport   %s\n",
		   becore->mcsc_encoded_transport == BECORE_MCSC_INPUT_MEMORY ?
		   "memory" : "captured-votf");
	seq_printf(s, "input            %zu/%zu bytes, iova %pad\n",
		   becore->inputs[0].buffer.staged_bytes,
		   becore->inputs[0].buffer.size, &becore->inputs[0].buffer.dma);
	seq_printf(s, "input_state      %s\n",
		   input_state_names[becore->inputs[0].state]);
	seq_puts(s, "input_slots      ");
	for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++) {
		struct becore_input_slot *slot = &becore->inputs[i];

		seq_printf(s, "%s%u:%s",
			   i ? " " : "", i, input_state_names[slot->state]);
		if (slot->state == BECORE_INPUT_PRODUCER)
			seq_printf(s, "#%llu", slot->producer_cookie);
		else if (slot->state == BECORE_INPUT_READY)
			seq_printf(s, "@%llu", slot->ready_sequence);
	}
	seq_putc(s, '\n');
	if (becore->input_producer) {
		seq_printf(s, "input_producer   %s iovas",
			   dev_name(becore->input_producer->producer));
		for (i = 0; i < BECORE_INPUT_SLOT_COUNT; i++)
			seq_printf(s, " %u:%pad", i,
				   &becore->input_producer->dmas[i]);
		seq_putc(s, '\n');
	}
	seq_printf(s, "grid             %zu/%zu bytes, iova %pad\n",
		   becore->grid.staged_bytes, becore->grid.size,
		   &becore->grid.dma);
	seq_printf(s, "output           %zu active/%zu completed/%zu allocated bytes, iova %pad\n",
		   becore->active_output_size, becore->completed_output_size,
		   becore->output.size,
		   &becore->output.dma);
	seq_printf(s, "output_profile   %u requested, %u active\n",
		   READ_ONCE(becore->output_profile),
		   becore->active_output_profile);
	seq_printf(s, "active_path      %s\n",
		   becore->active_mcsc ? "YUVP-memory-to-MCSC" : "YUVP");
	seq_printf(s, "rgbp_cmdq        %zu bytes/%u headers, iova %pad\n",
		   becore->program[BECORE_RGBP].size,
		   becore->program[BECORE_RGBP].header_count,
		   &becore->program[BECORE_RGBP].dma);
	seq_printf(s, "yuvp_cmdq        %zu bytes/%u headers, iova %pad\n",
		   becore->program[BECORE_YUVP].size,
		   becore->program[BECORE_YUVP].header_count,
		   &becore->program[BECORE_YUVP].dma);
	seq_printf(s, "gtnr_output      %zu bytes, iova %pad\n",
		   becore->gtnr_output.size, &becore->gtnr_output.dma);
	seq_printf(s, "gtnr_cmdq        %zu bytes/%u headers, iova %pad\n",
		   becore->gtnr_program.size,
		   becore->gtnr_program.header_count,
		   &becore->gtnr_program.dma);
	seq_printf(s, "mcsc_output      %u completed/%zu allocated bytes, iova %pad\n",
		   becore->mcsc_completed_output_size, becore->mcsc_output.size,
		   &becore->mcsc_output.dma);
	seq_printf(s, "mcsc_cmdq        %zu bytes/%u headers, iova %pad\n",
		   becore->mcsc_program.size,
		   becore->mcsc_program.header_count,
		   &becore->mcsc_program.dma);
	seq_printf(s, "run_generation   %u\n", becore->run_generation);
	seq_printf(s, "completed         %u\n", becore->completed_generation);
	seq_printf(s, "last_result       %d\n", becore->last_run_result);
	seq_printf(s, "cmdq_hold         %#x\n", cmdq_hold_mask);
	seq_printf(s, "frame_done        %#x\n", frame_done_mask);
	seq_printf(s, "expected          %#x\n", expected_mask);
	seq_printf(s, "start_issued      %u\n", start_issued);
	seq_printf(s, "irq_error         %u\n", irq_error);
	seq_printf(s, "reset_failed      %u\n", becore->reset_failed);
	seq_printf(s, "output_quarantined %u\n", becore->output_quarantined);
	seq_printf(s, "output_changed    %u\n", becore->output_changed_bytes);
	seq_printf(s, "output_first      %#x\n", becore->output_first_changed);
	seq_printf(s, "mcsc_completed    %u\n",
		   becore->mcsc_completed_generation);
	seq_printf(s, "mcsc_changed      %u\n",
		   becore->mcsc_output_changed_bytes);
	seq_printf(s, "mcsc_first        %#x\n",
		   becore->mcsc_output_first_changed);
	for (i = 0; i < BECORE_NUM_BLOCKS; i++)
		seq_printf(s,
			   "%-4s irq0 %lld last %#010x cmdq %#010x; irq1 %lld last %#010x\n",
			   becore->blocks[i].name,
			   atomic64_read(&becore->blocks[i].int0_count),
			   READ_ONCE(becore->blocks[i].last_int0),
			   READ_ONCE(becore->blocks[i].last_cmdq_int),
			   atomic64_read(&becore->blocks[i].int1_count),
			   READ_ONCE(becore->blocks[i].last_int1));
	mutex_unlock(&becore->lock);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(becore_status);

static void becore_video_unregister(void *data)
{
	struct becore_device *becore = data;

	vb2_video_unregister_device(&becore->vdev);
	media_device_unregister(&becore->mdev);
	media_entity_cleanup(&becore->vdev.entity);
	v4l2_device_unregister(&becore->v4l2_dev);
	media_device_cleanup(&becore->mdev);
}

static int becore_video_register(struct becore_device *becore)
{
	struct vb2_queue *q = &becore->queue;
	int ret;

	becore->mdev.dev = becore->dev;
	strscpy(becore->mdev.model, "zumapro BE-core",
		sizeof(becore->mdev.model));
	media_device_init(&becore->mdev);
	becore->v4l2_dev.mdev = &becore->mdev;

	ret = v4l2_device_register(becore->dev, &becore->v4l2_dev);
	if (ret)
		goto err_mdev;

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP;
	q->dev = becore->dev;
	q->drv_priv = becore;
	q->ops = &becore_vb2_ops;
	q->mem_ops = &vb2_vmalloc_memops;
	q->buf_struct_size = sizeof(struct becore_video_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_queued_buffers = 1;
	q->lock = &becore->video_lock;
	ret = vb2_queue_init(q);
	if (ret)
		goto err_v4l2;

	becore->vdev = becore_video_template;
	becore->vdev.v4l2_dev = &becore->v4l2_dev;
	becore->vdev.queue = q;
	becore->vdev.lock = &becore->video_lock;
	becore->vdev.entity.function = MEDIA_ENT_F_IO_V4L;
	video_set_drvdata(&becore->vdev, becore);

	becore->vdev_pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&becore->vdev.entity, 1,
				     &becore->vdev_pad);
	if (ret)
		goto err_v4l2;

	ret = video_register_device(&becore->vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_entity;

	ret = media_device_register(&becore->mdev);
	if (ret)
		goto err_vdev;

	return devm_add_action_or_reset(becore->dev,
					becore_video_unregister, becore);

err_vdev:
	video_unregister_device(&becore->vdev);
err_entity:
	media_entity_cleanup(&becore->vdev.entity);
err_v4l2:
	v4l2_device_unregister(&becore->v4l2_dev);
err_mdev:
	media_device_cleanup(&becore->mdev);
	return ret;
}

static void becore_debugfs_remove(void *data)
{
	struct becore_device *becore = data;

	debugfs_remove_recursive(becore->debugfs);
}

static int becore_debugfs_init(struct becore_device *becore)
{
	struct dentry *dir;

	dir = debugfs_create_dir(dev_name(becore->dev), NULL);
	if (IS_ERR(dir))
		return PTR_ERR(dir);
	becore->debugfs = dir;
	debugfs_create_file("program", 0600, dir, becore, &becore_recipe_fops);
	debugfs_create_file("gtnr_program", 0600, dir, becore,
			    &becore_gtnr_recipe_fops);
	debugfs_create_file("mcsc_program", 0600, dir, becore,
			    &becore_mcsc_recipe_fops);
	debugfs_create_file("input", 0200, dir, becore, &becore_input_fops);
	debugfs_create_file("grid", 0200, dir, becore, &becore_grid_fops);
	debugfs_create_u32("output_profile", 0644, dir,
			   &becore->output_profile);
	debugfs_create_file("output", 0400, dir, becore, &becore_output_fops);
	debugfs_create_file("mcsc_output", 0400, dir, becore,
			    &becore_mcsc_output_fops);
	debugfs_create_file("rgbp_cmdq", 0400, dir, becore,
			    &becore_rgbp_encoded_fops);
	debugfs_create_file("yuvp_cmdq", 0400, dir, becore,
			    &becore_yuvp_encoded_fops);
	debugfs_create_file("gtnr_cmdq", 0400, dir, becore,
			    &becore_gtnr_encoded_fops);
	debugfs_create_file("gtnr_encode", 0200, dir, becore,
			    &becore_gtnr_encode_fops);
	debugfs_create_file("mcsc_cmdq", 0400, dir, becore,
			    &becore_mcsc_encoded_fops);
	debugfs_create_file("mcsc_encode", 0200, dir, becore,
			    &becore_mcsc_encode_fops);
	debugfs_create_file("run", 0600, dir, becore, &becore_run_fops);
	debugfs_create_file("mcsc_run", 0200, dir, becore,
			    &becore_mcsc_run_fops);
	debugfs_create_file("cancel", 0200, dir, becore, &becore_cancel_fops);
	debugfs_create_file("status", 0400, dir, becore, &becore_status_fops);

	return devm_add_action_or_reset(becore->dev, becore_debugfs_remove,
					becore);
}

static int becore_probe(struct platform_device *pdev)
{
	static const struct dev_pm_domain_attach_data pm_domain_data = {
		.pd_names = becore_pm_domain_names,
		.num_pd_names = ARRAY_SIZE(becore_pm_domain_names),
	};
	struct device *dev = &pdev->dev;
	struct becore_device *becore;
	int ret;

	becore = devm_kzalloc(dev, sizeof(*becore), GFP_KERNEL);
	if (!becore)
		return -ENOMEM;

	becore->dev = dev;
	mutex_init(&becore->lock);
	mutex_init(&becore->video_lock);
	spin_lock_init(&becore->run_lock);
	spin_lock_init(&becore->queue_lock);
	init_completion(&becore->run_completion);
	INIT_LIST_HEAD(&becore->queued_outputs);
	INIT_WORK(&becore->video_work, becore_video_work);
	becore->output_first_changed = U32_MAX;
	becore->mcsc_output_first_changed = U32_MAX;
	becore->active_output_profile = BECORE_YUVP_OUTPUT_SBWCL;
	becore->active_output_size = becore_active_output_size(becore);
	becore->blocks[BECORE_RGBP] = (struct becore_block) {
		.becore = becore,
		.name = "RGBP",
		.int0_mask_prepare = 0x18e1fc02,
		.int0_mask = 0x18e1fc06,
		.int1_mask = 0x7fff,
		.cmdq_int_mask = 0x7,
	};
	becore->blocks[BECORE_MCFP] = (struct becore_block) {
		.becore = becore,
		.name = "MCFP",
		.int0_mask_prepare = 0x3ffffc02,
		.int0_mask = 0x3ffffc06,
		.cmdq_int_mask = 0x7,
	};
	becore->blocks[BECORE_YUVP] = (struct becore_block) {
		.becore = becore,
		.name = "YUVP",
		.int0_mask_prepare = 0x3fe1fc02,
		.int0_mask = 0x3fe1fc06,
		.int1_mask = 0x1ffffff,
		.cmdq_int_mask = 0xff,
	};
	becore->blocks[BECORE_MCSC] = (struct becore_block) {
		.becore = becore,
		.name = "MCSC",
		.int0_mask_prepare = 0x3ffffc02,
		.int0_mask = 0x3ffffc06,
		.int1_mask = 0x501,
		.cmdq_int_mask = 0x1,
	};
	platform_set_drvdata(pdev, becore);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "no 32-bit DMA\n");

	ret = becore_map_resources(pdev, becore);
	if (ret)
		return ret;

	ret = devm_pm_domain_attach_list(dev, &pm_domain_data,
					 &becore->pm_domains);
	if (ret < 0)
		return dev_err_probe(dev, ret, "cannot attach power domains\n");
	if (ret != (int)ARRAY_SIZE(becore_pm_domain_names))
		return dev_err_probe(dev, -ENODEV,
				     "attached %d of %zu power domains\n", ret,
				     ARRAY_SIZE(becore_pm_domain_names));

	ret = becore_request_irqs(pdev, becore);
	if (ret)
		return ret;
	ret = becore_alloc_diagnostic(becore);
	if (ret)
		return ret;
	becore->active_output_dma = becore->output.dma;

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return dev_err_probe(dev, ret, "cannot power camera back end\n");
	if (becore->reset_failed) {
		dev_crit(dev, "processor reset failed; retaining power\n");
		return 0;
	}

	ret = pm_runtime_put_sync(dev);
	if (ret < 0) {
		/* Keep all suppliers active when the reset vetoes power-down. */
		pm_runtime_get_noresume(dev);
		dev_crit(dev, "cannot quiesce camera back end; retaining power\n");
		return 0;
	}

	ret = becore_video_register(becore);
	if (ret)
		return ret;

	return becore_debugfs_init(becore);
}

static const struct dev_pm_ops becore_pm_ops = {
	SET_RUNTIME_PM_OPS(becore_runtime_suspend, becore_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static const struct of_device_id becore_of_match[] = {
	{ .compatible = "google,zumapro-becore" },
	{ }
};
MODULE_DEVICE_TABLE(of, becore_of_match);

static struct platform_driver becore_driver = {
	.probe = becore_probe,
	.driver = {
		.name = "exynos-becore",
		.of_match_table = becore_of_match,
		.pm = pm_ptr(&becore_pm_ops),
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(becore_driver);

MODULE_DESCRIPTION("Google Zumapro camera back-end core");
MODULE_AUTHOR("Steffen Deusch <steffen@deusch.me>");
MODULE_LICENSE("GPL");
