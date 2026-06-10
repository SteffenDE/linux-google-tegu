// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Tensor G4 Zumapro DPU bring-up scaffold.
 *
 * DECON0 exposes an opt-in, trace-shaped color-map CRTC path for first
 * hardware-on validation.  It intentionally avoids DPP/RDMA programming.
 */

#include <linux/bitops.h>
#include <linux/component.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <drm/display/drm_dsc.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_modes.h>
#include <drm/drm_vblank.h>

#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include "exynos_drm_crtc.h"
#include "exynos_drm_drv.h"
#include "exynos_drm_plane.h"
#include "regs-zumapro-dpu.h"

static bool zumapro_enable_unsafe_modeset;
module_param_named(zumapro_enable_unsafe_modeset,
		   zumapro_enable_unsafe_modeset, bool, 0644);
MODULE_PARM_DESC(zumapro_enable_unsafe_modeset,
		 "Allow Zumapro DECON0 to touch display hardware on CRTC enable");

struct zumapro_dpp {
	struct device *dev;
	u32 id;
	u32 attributes;
	u32 axi_port;
	u32 scale_down;
	u32 scale_up;
	bool video_formats;
	const u32 *pixel_formats;
	unsigned int num_pixel_formats;
	const struct zumapro_dpp_restrictions *restrictions;
};

struct zumapro_decon {
	struct device *dev;
	struct drm_device *drm_dev;
	struct exynos_drm_crtc *crtc;
	struct exynos_drm_plane plane;
	struct exynos_drm_plane_config plane_config;
	void __iomem *main_regs;
	void __iomem *win_regs;
	void __iomem *sub_regs;
	void __iomem *wincon_regs;
	void __iomem *dqe_regs;
	u32 id;
	u32 cgc_dma_id;
	u32 max_windows;
	const struct zumapro_panel_pipeline *pipeline;
	int dpp_count;
	bool enabled;
	bool start_pending;
};

struct zumapro_decon_desc {
	u32 id;
	const char * const *reg_names;
	unsigned int num_reg_names;
	const char * const *irq_names;
	unsigned int num_irq_names;
	bool has_cgc_dma;
};

static const char * const zumapro_dpp_reg_names[] = {
	"dma",
	"dpp",
	"scl_coef",
	"sramc",
	"hdr_comm",
	"hdr",
};

static const char * const zumapro_dpp_irq_names[] = {
	"dma",
	"dpp",
};

static const char * const zumapro_decon_reg_names[] = {
	"main",
	"win",
	"sub",
	"wincon",
	"dqe",
	"dqe-cgc",
	"cgc-dma",
};

static const char * const zumapro_decon0_irq_names[] = {
	"frame_start",
	"frame_done",
	"extra",
	"dimming_start",
	"dimming_end",
	"cgc-dma",
};

static const char * const zumapro_decon1_irq_names[] = {
	"frame_start",
	"frame_done",
	"extra",
	"cgc-dma",
};

static const char * const zumapro_decon2_reg_names[] = {
	"main",
	"win",
	"sub",
	"wincon",
};

static const char * const zumapro_decon2_irq_names[] = {
	"frame_start",
	"frame_done",
	"extra",
};

static const struct zumapro_decon_desc zumapro_decon_descs[] = {
	{
		.id = 0,
		.reg_names = zumapro_decon_reg_names,
		.num_reg_names = ARRAY_SIZE(zumapro_decon_reg_names),
		.irq_names = zumapro_decon0_irq_names,
		.num_irq_names = ARRAY_SIZE(zumapro_decon0_irq_names),
		.has_cgc_dma = true,
	}, {
		.id = 1,
		.reg_names = zumapro_decon_reg_names,
		.num_reg_names = ARRAY_SIZE(zumapro_decon_reg_names),
		.irq_names = zumapro_decon1_irq_names,
		.num_irq_names = ARRAY_SIZE(zumapro_decon1_irq_names),
		.has_cgc_dma = true,
	}, {
		.id = 2,
		.reg_names = zumapro_decon2_reg_names,
		.num_reg_names = ARRAY_SIZE(zumapro_decon2_reg_names),
		.irq_names = zumapro_decon2_irq_names,
		.num_irq_names = ARRAY_SIZE(zumapro_decon2_irq_names),
	},
};

struct zumapro_dpp_format {
	u32 drm_format;
	enum zumapro_dpu_dma_format dma_format;
	enum zumapro_dpu_dpp_format dpp_format;
};

struct zumapro_dpp_size_range {
	u32 min;
	u32 max;
	u32 align;
};

struct zumapro_dpp_restrictions {
	struct zumapro_dpp_size_range src_f_w;
	struct zumapro_dpp_size_range src_f_h;
	struct zumapro_dpp_size_range src_w;
	struct zumapro_dpp_size_range src_h;
	u32 src_x_align;
	u32 src_y_align;

	struct zumapro_dpp_size_range dst_f_w;
	struct zumapro_dpp_size_range dst_f_h;
	struct zumapro_dpp_size_range dst_w;
	struct zumapro_dpp_size_range dst_h;
	u32 dst_x_align;
	u32 dst_y_align;

	struct zumapro_dpp_size_range blk_w;
	struct zumapro_dpp_size_range blk_h;
	u32 blk_x_align;
	u32 blk_y_align;

	u32 src_h_rot_max;
};

struct zumapro_panel_mode {
	const char *name;
	u32 clock_khz;
	u16 hdisplay;
	u16 hsync_start;
	u16 hsync_end;
	u16 htotal;
	u16 vdisplay;
	u16 vsync_start;
	u16 vsync_end;
	u16 vtotal;
	u16 width_mm;
	u16 height_mm;
	u16 vblank_usec;
	u16 te_usec;
	u8 refresh_hz;
	bool preferred;
	bool lp_mode;
};

struct zumapro_panel_pipeline {
	const struct zumapro_panel_mode *modes;
	unsigned int num_modes;
	const struct drm_dsc_config *dsc;
	u32 data_path;
	u32 out_type;
	enum zumapro_decon_fifo dsimif_fifo;
	u8 dsimif;
	u8 dsc_count;
	u8 data_lanes;
	u16 default_hs_clk_mbps;
	u16 alternate_hs_clk_mbps;
	u16 esc_clk_mhz;
	u32 pmsk[4];
	bool non_continuous_clock;
};

static const u32 zumapro_dpp_graphics_formats[] = {
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_BGRA8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBX8888,
	DRM_FORMAT_BGRX8888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_BGR565,
	DRM_FORMAT_ARGB2101010,
	DRM_FORMAT_ABGR2101010,
	DRM_FORMAT_RGBA1010102,
	DRM_FORMAT_BGRA1010102,
	DRM_FORMAT_ARGB16161616F,
	DRM_FORMAT_ABGR16161616F,
};

static const u32 zumapro_dpp_video_formats[] = {
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_BGRA8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBX8888,
	DRM_FORMAT_BGRX8888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_BGR565,
	DRM_FORMAT_ARGB2101010,
	DRM_FORMAT_ABGR2101010,
	DRM_FORMAT_RGBA1010102,
	DRM_FORMAT_BGRA1010102,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV21,
	DRM_FORMAT_NV16,
	DRM_FORMAT_NV61,
	DRM_FORMAT_P010,
	DRM_FORMAT_YUV420_8BIT,
	DRM_FORMAT_YUV420_10BIT,
	DRM_FORMAT_ARGB16161616F,
	DRM_FORMAT_ABGR16161616F,
};

static const struct zumapro_dpp_format zumapro_dpp_formats[] = {
	{ DRM_FORMAT_ARGB8888, ZUMAPRO_DMA_FORMAT_ARGB8888,
	  ZUMAPRO_DPP_FORMAT_ARGB8888 },
	{ DRM_FORMAT_ABGR8888, ZUMAPRO_DMA_FORMAT_ABGR8888,
	  ZUMAPRO_DPP_FORMAT_ARGB8888 },
	{ DRM_FORMAT_RGBA8888, ZUMAPRO_DMA_FORMAT_RGBA8888,
	  ZUMAPRO_DPP_FORMAT_ARGB8888 },
	{ DRM_FORMAT_BGRA8888, ZUMAPRO_DMA_FORMAT_BGRA8888,
	  ZUMAPRO_DPP_FORMAT_ARGB8888 },
	{ DRM_FORMAT_XRGB8888, ZUMAPRO_DMA_FORMAT_XRGB8888,
	  ZUMAPRO_DPP_FORMAT_ARGB8888 },
	{ DRM_FORMAT_XBGR8888, ZUMAPRO_DMA_FORMAT_XBGR8888,
	  ZUMAPRO_DPP_FORMAT_ARGB8888 },
	{ DRM_FORMAT_RGBX8888, ZUMAPRO_DMA_FORMAT_RGBX8888,
	  ZUMAPRO_DPP_FORMAT_ARGB8888 },
	{ DRM_FORMAT_BGRX8888, ZUMAPRO_DMA_FORMAT_BGRX8888,
	  ZUMAPRO_DPP_FORMAT_ARGB8888 },
	{ DRM_FORMAT_RGB565, ZUMAPRO_DMA_FORMAT_RGB565,
	  ZUMAPRO_DPP_FORMAT_ARGB8888 },
	{ DRM_FORMAT_BGR565, ZUMAPRO_DMA_FORMAT_BGR565,
	  ZUMAPRO_DPP_FORMAT_ARGB8888 },
	{ DRM_FORMAT_ARGB2101010, ZUMAPRO_DMA_FORMAT_ARGB2101010,
	  ZUMAPRO_DPP_FORMAT_ARGB8101010 },
	{ DRM_FORMAT_ABGR2101010, ZUMAPRO_DMA_FORMAT_ABGR2101010,
	  ZUMAPRO_DPP_FORMAT_ARGB8101010 },
	{ DRM_FORMAT_RGBA1010102, ZUMAPRO_DMA_FORMAT_RGBA1010102,
	  ZUMAPRO_DPP_FORMAT_ARGB8101010 },
	{ DRM_FORMAT_BGRA1010102, ZUMAPRO_DMA_FORMAT_BGRA1010102,
	  ZUMAPRO_DPP_FORMAT_ARGB8101010 },
	{ DRM_FORMAT_NV12, ZUMAPRO_DMA_FORMAT_NV12,
	  ZUMAPRO_DPP_FORMAT_YUV420_8P },
	{ DRM_FORMAT_NV21, ZUMAPRO_DMA_FORMAT_NV21,
	  ZUMAPRO_DPP_FORMAT_YUV420_8P },
	{ DRM_FORMAT_NV16, ZUMAPRO_DMA_FORMAT_NV16,
	  ZUMAPRO_DPP_FORMAT_YUV422_8P },
	{ DRM_FORMAT_NV61, ZUMAPRO_DMA_FORMAT_NV61,
	  ZUMAPRO_DPP_FORMAT_YUV422_8P },
	{ DRM_FORMAT_P010, ZUMAPRO_DMA_FORMAT_YUV420_P010,
	  ZUMAPRO_DPP_FORMAT_YUV420_P010 },
	{ DRM_FORMAT_YUV420_8BIT, ZUMAPRO_DMA_FORMAT_NV12,
	  ZUMAPRO_DPP_FORMAT_YUV420_8P },
	{ DRM_FORMAT_YUV420_10BIT, ZUMAPRO_DMA_FORMAT_YUV420_P010,
	  ZUMAPRO_DPP_FORMAT_YUV420_P010 },
	{ DRM_FORMAT_ARGB16161616F, ZUMAPRO_DMA_FORMAT_ARGB_FP16,
	  ZUMAPRO_DPP_FORMAT_ARGB8101010 },
	{ DRM_FORMAT_ABGR16161616F, ZUMAPRO_DMA_FORMAT_ABGR_FP16,
	  ZUMAPRO_DPP_FORMAT_ARGB8101010 },
};

static const struct zumapro_dpp_restrictions zumapro_dpp_restrictions = {
	.src_f_w = { 16, 65534, 1 },
	.src_f_h = { 16, 8190, 1 },
	.src_w = { 16, 4096, 1 },
	.src_h = { 16, 4096, 1 },
	.src_x_align = 1,
	.src_y_align = 1,

	.dst_f_w = { 16, 8190, 1 },
	.dst_f_h = { 16, 8190, 1 },
	.dst_w = { 16, 4096, 1 },
	.dst_h = { 16, 4096, 1 },
	.dst_x_align = 1,
	.dst_y_align = 1,

	.blk_w = { 4, 4096, 1 },
	.blk_h = { 4, 4096, 1 },
	.blk_x_align = 1,
	.blk_y_align = 1,

	.src_h_rot_max = 2160,
};

static const struct zumapro_dpp_format *
zumapro_dpp_find_format(u32 drm_format)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(zumapro_dpp_formats); i++)
		if (zumapro_dpp_formats[i].drm_format == drm_format)
			return &zumapro_dpp_formats[i];

	return NULL;
}

static int zumapro_dpp_validate_formats(struct device *dev,
					const u32 *formats,
					unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		if (!zumapro_dpp_find_format(formats[i]))
			return dev_err_probe(dev, -EINVAL,
					     "missing hardware mapping for DRM format %#x\n",
					     formats[i]);
	}

	return 0;
}

static int zumapro_dpp_select_formats(struct zumapro_dpp *dpp)
{
	if (dpp->video_formats) {
		dpp->pixel_formats = zumapro_dpp_video_formats;
		dpp->num_pixel_formats = ARRAY_SIZE(zumapro_dpp_video_formats);
	} else {
		dpp->pixel_formats = zumapro_dpp_graphics_formats;
		dpp->num_pixel_formats = ARRAY_SIZE(zumapro_dpp_graphics_formats);
	}

	return zumapro_dpp_validate_formats(dpp->dev, dpp->pixel_formats,
					    dpp->num_pixel_formats);
}

#define ZUMAPRO_DSC_6BIT_SIGNED(_v)	((_v) & 0x3f)

static const struct drm_dsc_config zumapro_tg4c_dsc = {
	.dsc_version_major = 1,
	.dsc_version_minor = 2,
	.line_buf_depth = 9,
	.bits_per_component = 8,
	.convert_rgb = true,
	.slice_count = 2,
	.slice_width = 540,
	.slice_height = 24,
	.simple_422 = false,
	.pic_width = 1080,
	.pic_height = 2424,
	.rc_tgt_offset_high = 3,
	.rc_tgt_offset_low = 3,
	.bits_per_pixel = 128,
	.rc_edge_factor = 6,
	.rc_quant_incr_limit1 = 11,
	.rc_quant_incr_limit0 = 11,
	.initial_xmit_delay = 512,
	.block_pred_enable = true,
	.first_line_bpg_offset = 12,
	.initial_offset = 6144,
	.rc_buf_thresh = {
		14, 28, 42, 56, 70, 84, 98, 105,
		112, 119, 121, 123, 125, 126,
	},
	.rc_range_params = {
		{ 0, 4, ZUMAPRO_DSC_6BIT_SIGNED(2) },
		{ 0, 4, ZUMAPRO_DSC_6BIT_SIGNED(0) },
		{ 1, 5, ZUMAPRO_DSC_6BIT_SIGNED(0) },
		{ 1, 6, ZUMAPRO_DSC_6BIT_SIGNED(-2) },
		{ 3, 7, ZUMAPRO_DSC_6BIT_SIGNED(-4) },
		{ 3, 7, ZUMAPRO_DSC_6BIT_SIGNED(-6) },
		{ 3, 7, ZUMAPRO_DSC_6BIT_SIGNED(-8) },
		{ 3, 8, ZUMAPRO_DSC_6BIT_SIGNED(-8) },
		{ 3, 9, ZUMAPRO_DSC_6BIT_SIGNED(-8) },
		{ 3, 10, ZUMAPRO_DSC_6BIT_SIGNED(-10) },
		{ 5, 10, ZUMAPRO_DSC_6BIT_SIGNED(-10) },
		{ 5, 11, ZUMAPRO_DSC_6BIT_SIGNED(-12) },
		{ 5, 11, ZUMAPRO_DSC_6BIT_SIGNED(-12) },
		{ 9, 12, ZUMAPRO_DSC_6BIT_SIGNED(-12) },
		{ 12, 13, ZUMAPRO_DSC_6BIT_SIGNED(-12) },
	},
	.rc_model_size = 8192,
	.flatness_min_qp = 3,
	.flatness_max_qp = 12,
	.initial_scale_value = 32,
	.scale_decrement_interval = 7,
	.scale_increment_interval = 588,
	.nfl_bpg_offset = 1069,
	.slice_bpg_offset = 1085,
	.final_offset = 4336,
	.vbr_enable = false,
	.slice_chunk_size = 540,
	.native_422 = false,
	.native_420 = false,
	.second_line_bpg_offset = 0,
	.nsl_bpg_offset = 0,
	.second_line_offset_adj = 0,
};

static const struct zumapro_panel_mode zumapro_tg4c_modes[] = {
	{
		.name = "1080x2424@60:60",
		.clock_khz = 167922,
		.hdisplay = 1080,
		.hsync_start = 1112,
		.hsync_end = 1124,
		.htotal = 1140,
		.vdisplay = 2424,
		.vsync_start = 2436,
		.vsync_end = 2440,
		.vtotal = 2455,
		.width_mm = 64,
		.height_mm = 145,
		.vblank_usec = 120,
		.te_usec = 8450,
		.refresh_hz = 60,
		.preferred = true,
	}, {
		.name = "1080x2424@120:120",
		.clock_khz = 335844,
		.hdisplay = 1080,
		.hsync_start = 1112,
		.hsync_end = 1124,
		.htotal = 1140,
		.vdisplay = 2424,
		.vsync_start = 2436,
		.vsync_end = 2440,
		.vtotal = 2455,
		.width_mm = 64,
		.height_mm = 145,
		.vblank_usec = 120,
		.te_usec = 276,
		.refresh_hz = 120,
	}, {
		.name = "1080x2424@30:30",
		.clock_khz = 83961,
		.hdisplay = 1080,
		.hsync_start = 1112,
		.hsync_end = 1124,
		.htotal = 1140,
		.vdisplay = 2424,
		.vsync_start = 2436,
		.vsync_end = 2440,
		.vtotal = 2455,
		.width_mm = 64,
		.height_mm = 145,
		.vblank_usec = 120,
		.refresh_hz = 30,
		.lp_mode = true,
	},
};

static const struct zumapro_panel_pipeline zumapro_decon0_tg4c_pipeline = {
	.modes = zumapro_tg4c_modes,
	.num_modes = ARRAY_SIZE(zumapro_tg4c_modes),
	.dsc = &zumapro_tg4c_dsc,
	.data_path = ZUMAPRO_DECON_ENHANCE_DQE_ON |
		     ZUMAPRO_DPATH_DSCC_DSCENC01_OUTFIFO01_DSIMIF0,
	.out_type = ZUMAPRO_DECON_OUT_DSI0,
	.dsimif_fifo = ZUMAPRO_DECON0_OFIFO0,
	.dsimif = 0,
	.dsc_count = 2,
	.data_lanes = 4,
	.default_hs_clk_mbps = 1102,
	.alternate_hs_clk_mbps = 1000,
	.esc_clk_mhz = 20,
	.pmsk = { 0x02, 0xb3, 0x02, 0x5cab },
	.non_continuous_clock = true,
};

static const struct zumapro_decon_desc *zumapro_decon_desc_by_id(u32 id)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(zumapro_decon_descs); i++)
		if (zumapro_decon_descs[i].id == id)
			return &zumapro_decon_descs[i];

	return NULL;
}

static void zumapro_decon_update_bits(void __iomem *regs, u32 offset, u32 mask,
				      u32 val)
{
	u32 tmp;

	tmp = readl(regs + offset);
	tmp &= ~mask;
	tmp |= val & mask;
	writel(tmp, regs + offset);
}

struct zumapro_decon_pps_word {
	u32 offset;
	u32 value;
};

/*
 * Final PPS register values from the downstream TG4C wake trace.  Downstream
 * reaches some of these by RMW, but the milestone-1 comparison is final state.
 */
static const struct zumapro_decon_pps_word zumapro_tg4c_dsc_pps[] = {
	{ 0x40, 0x12000089 },
	{ 0x44, 0x30800978 },
	{ 0x48, 0x04380018 },
	{ 0x4c, 0x021c021c },
	{ 0x50, 0x0200020e },
	{ 0x54, 0x0020024c },
	{ 0x58, 0x0007000c },
	{ 0x5c, 0x042d043d },
	{ 0x60, 0x180010f0 },
	{ 0x64, 0x030c2000 },
	{ 0x68, 0x060b0b33 },
	{ 0x6c, 0x0e1c2a38 },
	{ 0x70, 0x46546269 },
	{ 0x78, 0x7d7e0102 },
	{ 0x7c, 0x01000940 },
	{ 0x80, 0x09be19fc },
	{ 0x84, 0x19fa19f8 },
	{ 0x88, 0x1a381a78 },
	{ 0x8c, 0x1ab62ab6 },
	{ 0x90, 0x2af42af4 },
	{ 0x94, 0x4b346374 },
};

static void zumapro_decon_write_dsc_pps(struct zumapro_decon *decon, u8 dsc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(zumapro_tg4c_dsc_pps); i++)
		writel(zumapro_tg4c_dsc_pps[i].value,
		       decon->sub_regs + ZUMAPRO_DSC_OFFSET(dsc) +
		       zumapro_tg4c_dsc_pps[i].offset);
}

static u32 zumapro_dqe_size(u32 width, u32 height)
{
	return ZUMAPRO_DQE_IMG_VSIZE(height) |
	       ZUMAPRO_DQE_IMG_HSIZE(width);
}

static u32 zumapro_dqe_atc_ibsi(u32 width, u32 height, u32 div)
{
	u32 hori_grid = DIV_ROUND_UP(width, 8);
	u32 vert_grid = DIV_ROUND_UP(height, 16);
	u32 ibsi_x = (1 << 16) / (hori_grid * div);
	u32 ibsi_y = (1 << 16) / (vert_grid * div);

	return ZUMAPRO_DQE_ATC_IBSI_Y(ibsi_y) |
	       ZUMAPRO_DQE_ATC_IBSI_X(ibsi_x);
}

static u32 zumapro_dqe_atc_cdf(u32 width, u32 height)
{
	u32 pixels = width * height;
	u32 tmp;
	u32 shift;
	u32 denom;
	u32 div;

	if (!pixels)
		return 0;

	tmp = (481 * pixels) / (255 * (1 << 14));
	if (!tmp)
		return 0;

	if (tmp & (tmp - 1))
		shift = fls(tmp);
	else
		shift = fls(tmp) - 1;

	denom = pixels >> shift;
	if (!denom)
		return 0;

	div = ((1 << 14) / denom) * 255;

	return ZUMAPRO_DQE_ATC_CDF_SHIFT(shift) |
	       ZUMAPRO_DQE_ATC_CDF_DIV_VAL(div);
}

static void zumapro_decon_program_dqe(struct zumapro_decon *decon,
				      const struct drm_display_mode *mode)
{
	u32 width = mode->hdisplay;
	u32 height = mode->vdisplay;
	u32 size = zumapro_dqe_size(width, height);

	writel(size, decon->dqe_regs + ZUMAPRO_DQE_TOP_IMG_SIZE);
	writel(size, decon->dqe_regs + ZUMAPRO_DQE_TOP_FRM_SIZE);
	writel(ZUMAPRO_DQE_FULL_PXL_NUM(width * height),
	       decon->dqe_regs + ZUMAPRO_DQE_TOP_FRM_PXL_NUM);
	writel(zumapro_dqe_atc_ibsi(width, height, 4),
	       decon->dqe_regs + ZUMAPRO_DQE_ATC_PARTIAL_IBSI_P1);
	writel(zumapro_dqe_atc_ibsi(width, height, 2),
	       decon->dqe_regs + ZUMAPRO_DQE_ATC_PARTIAL_IBSI_P2);
	writel(zumapro_dqe_atc_cdf(width, height),
	       decon->dqe_regs + ZUMAPRO_DQE_ATC_CDF_DIV);
	writel(0, decon->dqe_regs + ZUMAPRO_DQE_ATC_CONTROL);
	writel(0, decon->dqe_regs + ZUMAPRO_DQE_DISP_DITHER_V4);
}

static void zumapro_decon_program_outfifo(struct zumapro_decon *decon)
{
	/* Downstream DECON0 primary OUTFIFO owns SRAM banks 0..10. */
	writel(0x11111111, decon->main_regs + ZUMAPRO_DECON_SRAM_EN_OF_PRI(0));
	writel(0x00000111, decon->main_regs + ZUMAPRO_DECON_SRAM_EN_OF_PRI(1));
	writel(0, decon->main_regs + ZUMAPRO_DECON_SRAM_EN_OF_PRI(2));
	writel(0, decon->main_regs + ZUMAPRO_DECON_SRAM_EN_OF_PRI(3));
	writel(0, decon->main_regs + ZUMAPRO_DECON_SRAM_EN_OF_SEC(0));
	writel(0, decon->main_regs + ZUMAPRO_DECON_SRAM_EN_OF_SEC(1));
	writel(0, decon->main_regs + ZUMAPRO_DECON_SRAM_EN_OF_SEC(2));
	writel(0, decon->main_regs + ZUMAPRO_DECON_SRAM_EN_OF_SEC(3));

	writel(0, decon->main_regs + ZUMAPRO_DECON_OF_PIXEL_ORDER);
	writel(0x1, decon->main_regs + ZUMAPRO_DECON_OF_URGENT_EN);
	writel(0x08000400, decon->main_regs + ZUMAPRO_DECON_OF_RD_URGENT_0);
	writel(0x10, decon->main_regs + ZUMAPRO_DECON_OF_RD_URGENT_1);
	writel(0, decon->main_regs + ZUMAPRO_DECON_OF_WR_URGENT_0);
	writel(0x1, decon->main_regs + ZUMAPRO_DECON_OF_DTA_CONTROL);
	writel(0x32000600, decon->main_regs + ZUMAPRO_DECON_OF_DTA_THRESHOLD);
}

static void zumapro_decon_program_dsc(struct zumapro_decon *decon,
				      const struct drm_display_mode *mode)
{
	const struct zumapro_panel_pipeline *pipeline = decon->pipeline;
	const struct drm_dsc_config *dsc = pipeline->dsc;
	u32 outfifo_width;
	u8 i;

	for (i = 0; i < pipeline->dsc_count; i++) {
		writel(0x222, decon->sub_regs + ZUMAPRO_DSC_CONTROL1(i));
		writel(0x30b4, decon->sub_regs + ZUMAPRO_DSC_CONTROL3(i));
		zumapro_decon_write_dsc_pps(decon, i);
	}

	outfifo_width = DIV_ROUND_UP(dsc->slice_width, 3);
	writel(ZUMAPRO_DECON_OF_HEIGHT(mode->vdisplay) |
	       ZUMAPRO_DECON_OF_WIDTH(outfifo_width),
	       decon->main_regs + ZUMAPRO_DECON_OF_SIZE_0);
	writel(ZUMAPRO_DECON_OF_TH_1H,
	       decon->main_regs + ZUMAPRO_DECON_OF_TH_TYPE);
	writel(ZUMAPRO_DECON_OF_WIDTH(outfifo_width),
	       decon->main_regs + ZUMAPRO_DECON_OF_SIZE_1);
	writel(ZUMAPRO_DECON_OF_HEIGHT(dsc->slice_height) |
	       ZUMAPRO_DECON_OF_WIDTH(outfifo_width),
	       decon->main_regs + ZUMAPRO_DECON_OF_SIZE_2);
}

static void zumapro_decon_program_lcd(struct zumapro_decon *decon,
				      const struct drm_display_mode *mode)
{
	const struct zumapro_panel_pipeline *pipeline = decon->pipeline;

	writel(ZUMAPRO_DECON_OF_HEIGHT(mode->vdisplay) |
	       ZUMAPRO_DECON_OF_WIDTH(mode->hdisplay),
	       decon->main_regs + ZUMAPRO_DECON_BLD_BG_IMG_SIZE_PRI);

	zumapro_decon_program_outfifo(decon);

	writel(ZUMAPRO_DSIMIF_SEL_DSIM(pipeline->dsimif_fifo),
	       decon->sub_regs + ZUMAPRO_DSIMIF_SEL(pipeline->dsimif));
	writel(pipeline->data_path,
	       decon->main_regs + ZUMAPRO_DECON_DATA_PATH_CON_0);

	zumapro_decon_program_dqe(decon, mode);
	zumapro_decon_program_dsc(decon, mode);

	zumapro_decon_update_bits(decon->main_regs, ZUMAPRO_DECON_GLOBAL_CON,
				  ZUMAPRO_DECON_GLOBAL_CON_CMD_MODE,
				  ZUMAPRO_DECON_GLOBAL_CON_CMD_MODE);
	zumapro_decon_update_bits(decon->main_regs, ZUMAPRO_DECON_GLOBAL_CON,
				  ZUMAPRO_DECON_GLOBAL_CON_EN_F, 0);
	zumapro_decon_update_bits(decon->main_regs, ZUMAPRO_DECON_TRIG_CON,
				  ZUMAPRO_DECON_HW_TRIG_SEL_MASK |
				  ZUMAPRO_DECON_HW_TRIG_EN |
				  ZUMAPRO_DECON_HW_TRIG_MASK,
				  ZUMAPRO_DECON_HW_TRIG_SEL_DDI0 |
				  ZUMAPRO_DECON_HW_TRIG_EN |
				  ZUMAPRO_DECON_HW_TRIG_MASK);
}

static void zumapro_decon_program_colormap_window(struct zumapro_decon *decon,
						 const struct drm_display_mode *mode)
{
	u32 blend_func;
	u32 blend_coeff;
	u32 end_pos;

	blend_func = ZUMAPRO_DECON_WIN_FUNC(ZUMAPRO_DECON_WIN_FUNC_USER_DEFINED) |
		     ZUMAPRO_DECON_WIN_ALPHA_MULT_SRC_SEL(
				ZUMAPRO_DECON_WIN_ALPHA_MULT_SRC_AF);
	blend_coeff =
		ZUMAPRO_DECON_WIN_FG_ALPHA_D_SEL(ZUMAPRO_DECON_WIN_BND_COEF_ONE) |
		ZUMAPRO_DECON_WIN_BG_ALPHA_D_SEL(ZUMAPRO_DECON_WIN_BND_COEF_ZERO) |
		ZUMAPRO_DECON_WIN_FG_ALPHA_A_SEL(ZUMAPRO_DECON_WIN_BND_COEF_ONE) |
		ZUMAPRO_DECON_WIN_BG_ALPHA_A_SEL(ZUMAPRO_DECON_WIN_BND_COEF_ZERO);
	end_pos = ZUMAPRO_DECON_WIN_POS_Y(mode->vdisplay - 1) |
		  ZUMAPRO_DECON_WIN_POS_X(mode->hdisplay - 1);

	writel(blend_func, decon->win_regs + ZUMAPRO_DECON_WIN_FUNC_CON_0(0));
	writel(blend_coeff, decon->win_regs + ZUMAPRO_DECON_WIN_FUNC_CON_1(0));
	writel(0, decon->win_regs + ZUMAPRO_DECON_WIN_START_POSITION(0));
	writel(end_pos, decon->win_regs + ZUMAPRO_DECON_WIN_END_POSITION(0));
	writel(0, decon->win_regs + ZUMAPRO_DECON_WIN_START_TIME_CON(0));
	writel(ZUMAPRO_DECON_WIN_MAPCOLOR_EN,
	       decon->wincon_regs + ZUMAPRO_DECON_CON_WIN(0));
	writel(0, decon->win_regs + ZUMAPRO_DECON_WIN_COLORMAP_0(0));
	writel(0, decon->win_regs + ZUMAPRO_DECON_WIN_COLORMAP_1(0));
	writel(ZUMAPRO_DECON_WIN_MAPCOLOR_EN | ZUMAPRO_DECON_WIN_EN,
	       decon->wincon_regs + ZUMAPRO_DECON_CON_WIN(0));
}

static int zumapro_decon_wait_run(struct zumapro_decon *decon)
{
	u32 val;

	return readl_poll_timeout(decon->main_regs + ZUMAPRO_DECON_GLOBAL_CON,
				  val, val & ZUMAPRO_DECON_GLOBAL_CON_RUN_STATUS,
				  10, 2000);
}

static void zumapro_decon_start(struct zumapro_decon *decon)
{
	int ret;

	zumapro_decon_update_bits(decon->main_regs, ZUMAPRO_DECON_SHD_REG_UP_REQ,
				  ZUMAPRO_DECON_SHD_DQE,
				  ZUMAPRO_DECON_SHD_DQE);
	zumapro_decon_update_bits(decon->main_regs, ZUMAPRO_DECON_SHD_REG_UP_REQ,
				  ZUMAPRO_DECON_SHD_ALL_WINDOWS,
				  ZUMAPRO_DECON_SHD_ALL_WINDOWS);

	zumapro_decon_update_bits(decon->main_regs, ZUMAPRO_DECON_GLOBAL_CON,
				  ZUMAPRO_DECON_GLOBAL_CON_EN |
				  ZUMAPRO_DECON_GLOBAL_CON_EN_F,
				  ZUMAPRO_DECON_GLOBAL_CON_EN |
				  ZUMAPRO_DECON_GLOBAL_CON_EN_F);
	zumapro_decon_update_bits(decon->main_regs, ZUMAPRO_DECON_SHD_REG_UP_REQ,
				  ZUMAPRO_DECON_SHD_GLOBAL |
				  ZUMAPRO_DECON_SHD_CMP,
				  ZUMAPRO_DECON_SHD_GLOBAL |
				  ZUMAPRO_DECON_SHD_CMP);

	ret = zumapro_decon_wait_run(decon);
	if (ret)
		dev_warn(decon->dev, "DECON%u did not enter run state: %d\n",
			 decon->id, ret);

	zumapro_decon_update_bits(decon->main_regs, ZUMAPRO_DECON_TRIG_CON,
				  ZUMAPRO_DECON_HW_TRIG_EN |
				  ZUMAPRO_DECON_HW_TRIG_MASK,
				  ZUMAPRO_DECON_HW_TRIG_EN);
}

static void zumapro_decon_stop(struct zumapro_decon *decon)
{
	u32 val;
	u32 win_count;
	u32 win;
	int ret;

	win_count = min_t(u32, decon->max_windows, ZUMAPRO_DPU_MAX_WINDOWS);
	for (win = 0; win < win_count; win++)
		writel(0, decon->wincon_regs + ZUMAPRO_DECON_CON_WIN(win));

	zumapro_decon_update_bits(decon->main_regs, ZUMAPRO_DECON_TRIG_CON,
				  ZUMAPRO_DECON_HW_TRIG_EN |
				  ZUMAPRO_DECON_HW_TRIG_MASK,
				  ZUMAPRO_DECON_HW_TRIG_MASK);
	zumapro_decon_update_bits(decon->main_regs, ZUMAPRO_DECON_GLOBAL_CON,
				  ZUMAPRO_DECON_GLOBAL_CON_EN_F, 0);
	zumapro_decon_update_bits(decon->main_regs, ZUMAPRO_DECON_SHD_REG_UP_REQ,
				  ZUMAPRO_DECON_SHD_GLOBAL |
				  ZUMAPRO_DECON_SHD_CMP,
				  ZUMAPRO_DECON_SHD_GLOBAL |
				  ZUMAPRO_DECON_SHD_CMP);

	/* Let a frame in flight drain before the reset, like downstream. */
	ret = readl_poll_timeout(decon->main_regs + ZUMAPRO_DECON_GLOBAL_CON,
				 val,
				 !(val & ZUMAPRO_DECON_GLOBAL_CON_RUN_STATUS),
				 10, 50000);
	if (ret)
		dev_warn(decon->dev, "DECON%u did not stop scanout: %d\n",
			 decon->id, ret);

	zumapro_decon_update_bits(decon->main_regs, ZUMAPRO_DECON_GLOBAL_CON,
				  ZUMAPRO_DECON_GLOBAL_CON_SRESET,
				  ZUMAPRO_DECON_GLOBAL_CON_SRESET);

	ret = readl_poll_timeout(decon->main_regs + ZUMAPRO_DECON_GLOBAL_CON,
				 val, !(val & ZUMAPRO_DECON_GLOBAL_CON_SRESET),
				 10, 2000);
	if (ret)
		dev_warn(decon->dev, "DECON%u reset did not complete: %d\n",
			 decon->id, ret);
}

static enum drm_mode_status
zumapro_decon_mode_valid(struct exynos_drm_crtc *crtc,
			 const struct drm_display_mode *mode)
{
	struct zumapro_decon *decon = crtc->ctx;
	const struct zumapro_panel_pipeline *pipeline = decon->pipeline;
	unsigned int i;

	for (i = 0; i < pipeline->num_modes; i++) {
		const struct zumapro_panel_mode *panel_mode = &pipeline->modes[i];

		if (mode->clock == panel_mode->clock_khz &&
		    mode->hdisplay == panel_mode->hdisplay &&
		    mode->hsync_start == panel_mode->hsync_start &&
		    mode->hsync_end == panel_mode->hsync_end &&
		    mode->htotal == panel_mode->htotal &&
		    mode->vdisplay == panel_mode->vdisplay &&
		    mode->vsync_start == panel_mode->vsync_start &&
		    mode->vsync_end == panel_mode->vsync_end &&
		    mode->vtotal == panel_mode->vtotal)
			return MODE_OK;
	}

	return MODE_BAD;
}

static int zumapro_decon_atomic_check(struct exynos_drm_crtc *crtc,
				      struct drm_crtc_state *state)
{
	struct zumapro_decon *decon = crtc->ctx;

	state->no_vblank = true;

	if (!state->active)
		return 0;

	if (!zumapro_enable_unsafe_modeset) {
		dev_warn_once(decon->dev,
			      "rejecting DECON%u modeset; pass exynosdrm.zumapro_enable_unsafe_modeset=1 for traced hardware-on validation\n",
			      decon->id);
		return -EPERM;
	}

	return 0;
}

static void zumapro_decon_atomic_enable(struct exynos_drm_crtc *crtc)
{
	struct zumapro_decon *decon = crtc->ctx;
	const struct drm_display_mode *mode = &crtc->base.state->adjusted_mode;
	int ret;

	if (decon->enabled)
		return;

	if (!mode->hdisplay || !mode->vdisplay)
		return;

	ret = pm_runtime_resume_and_get(decon->dev);
	if (ret < 0) {
		dev_err(decon->dev, "failed to resume DECON%u: %d\n",
			decon->id, ret);
		return;
	}

	/*
	 * The bootloader hands off a live, scanning DECON; downstream always
	 * stops and soft-resets the block before reprogramming it.
	 */
	zumapro_decon_stop(decon);

	zumapro_decon_program_lcd(decon, mode);
	zumapro_decon_program_colormap_window(decon, mode);

	/*
	 * Scanout must not start while the DSIM frame geometry is still
	 * unprogrammed and the panel uninitialized; both happen in the
	 * bridge-enable phase, after this hook.  Defer the start/unmask to
	 * atomic_flush, which runs after the bridge chain is enabled.
	 */
	decon->start_pending = true;
	decon->enabled = true;
}

static void zumapro_decon_atomic_disable(struct exynos_drm_crtc *crtc)
{
	struct zumapro_decon *decon = crtc->ctx;

	if (!decon->enabled)
		return;

	zumapro_decon_stop(decon);
	decon->enabled = false;
	decon->start_pending = false;
	pm_runtime_put_sync(decon->dev);
}

static void zumapro_decon_atomic_flush(struct exynos_drm_crtc *crtc)
{
	struct zumapro_decon *decon = crtc->ctx;

	if (!decon->enabled || !decon->start_pending)
		return;

	zumapro_decon_start(decon);
	decon->start_pending = false;
}

static int zumapro_decon_enable_vblank(struct exynos_drm_crtc *crtc)
{
	return -EINVAL;
}

static void zumapro_decon_disable_vblank(struct exynos_drm_crtc *crtc)
{
}

static void zumapro_decon_update_plane(struct exynos_drm_crtc *crtc,
				       struct exynos_drm_plane *plane)
{
}

static void zumapro_decon_disable_plane(struct exynos_drm_crtc *crtc,
					struct exynos_drm_plane *plane)
{
}

static const struct exynos_drm_crtc_ops zumapro_decon_crtc_ops = {
	.atomic_enable = zumapro_decon_atomic_enable,
	.atomic_disable = zumapro_decon_atomic_disable,
	.atomic_flush = zumapro_decon_atomic_flush,
	.enable_vblank = zumapro_decon_enable_vblank,
	.disable_vblank = zumapro_decon_disable_vblank,
	.mode_valid = zumapro_decon_mode_valid,
	.atomic_check = zumapro_decon_atomic_check,
	.update_plane = zumapro_decon_update_plane,
	.disable_plane = zumapro_decon_disable_plane,
};

static int zumapro_decon_bind(struct device *dev, struct device *master,
			      void *data)
{
	struct zumapro_decon *decon = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	int ret;

	if (!decon->pipeline)
		return dev_err_probe(dev, -ENODEV,
				     "DECON%u has no validated panel pipeline\n",
				     decon->id);

	decon->drm_dev = drm_dev;
	decon->plane_config.pixel_formats = zumapro_dpp_graphics_formats;
	decon->plane_config.num_pixel_formats =
		ARRAY_SIZE(zumapro_dpp_graphics_formats);
	decon->plane_config.zpos = 0;
	decon->plane_config.type = DRM_PLANE_TYPE_PRIMARY;

	ret = exynos_plane_init(drm_dev, &decon->plane, 0,
				&decon->plane_config);
	if (ret)
		return ret;

	decon->crtc = exynos_drm_crtc_create(drm_dev, &decon->plane.base,
					     EXYNOS_DISPLAY_TYPE_LCD,
					     &zumapro_decon_crtc_ops,
					     decon);
	if (IS_ERR(decon->crtc))
		return PTR_ERR(decon->crtc);

	return 0;
}

static void zumapro_decon_unbind(struct device *dev, struct device *master,
				 void *data)
{
	struct zumapro_decon *decon = dev_get_drvdata(dev);

	if (decon->crtc)
		zumapro_decon_atomic_disable(decon->crtc);
}

static const struct component_ops zumapro_decon_component_ops = {
	.bind = zumapro_decon_bind,
	.unbind = zumapro_decon_unbind,
};

static int zumapro_read_u32_compat(struct device *dev, const char *name,
				   const char *legacy_name, u32 *value)
{
	int ret;

	if (of_property_present(dev->of_node, name)) {
		ret = of_property_count_u32_elems(dev->of_node, name);
		if (ret != 1)
			return dev_err_probe(dev, -EINVAL,
					     "DT property %s must contain one u32\n",
					     name);
		ret = of_property_read_u32(dev->of_node, name, value);
		if (ret)
			return dev_err_probe(dev, ret,
					     "malformed DT property %s\n", name);
		return 0;
	}

	if (legacy_name) {
		if (!of_property_present(dev->of_node, legacy_name))
			return dev_err_probe(dev, -EINVAL,
					     "missing DT property %s\n", name);

		ret = of_property_count_u32_elems(dev->of_node, legacy_name);
		if (ret != 1)
			return dev_err_probe(dev, -EINVAL,
					     "DT property %s must contain one u32\n",
					     legacy_name);
		ret = of_property_read_u32(dev->of_node, legacy_name, value);
		if (ret)
			return dev_err_probe(dev, ret,
					     "malformed DT property %s\n",
					     legacy_name);

		dev_warn(dev, "using legacy DT property %s; prefer %s\n",
			 legacy_name, name);
		return 0;
	}

	return dev_err_probe(dev, -EINVAL, "missing DT property %s\n", name);
}

static int zumapro_read_u32_optional_compat(struct device *dev,
					    const char *name,
					    const char *legacy_name,
					    u32 *value)
{
	int ret;

	if (of_property_present(dev->of_node, name)) {
		ret = of_property_count_u32_elems(dev->of_node, name);
		if (ret != 1)
			return dev_err_probe(dev, -EINVAL,
					     "DT property %s must contain one u32\n",
					     name);
		of_property_read_u32(dev->of_node, name, value);
		return 0;
	}

	if (!legacy_name || !of_property_present(dev->of_node, legacy_name))
		return 0;

	ret = of_property_count_u32_elems(dev->of_node, legacy_name);
	if (ret != 1)
		return dev_err_probe(dev, -EINVAL,
				     "DT property %s must contain one u32\n",
				     legacy_name);

	of_property_read_u32(dev->of_node, legacy_name, value);

	if (legacy_name)
		dev_warn(dev, "using legacy DT property %s; prefer %s\n",
			 legacy_name, name);

	return 0;
}

static int zumapro_check_reg_names(struct platform_device *pdev,
				   const char * const *names, unsigned int count)
{
	struct device *dev = &pdev->dev;
	unsigned int i;

	for (i = 0; i < count; i++) {
		if (!platform_get_resource_byname(pdev, IORESOURCE_MEM,
						  names[i]))
			return dev_err_probe(dev, -EINVAL,
					     "missing %s MMIO resource\n",
					     names[i]);
	}

	return 0;
}

static int zumapro_check_irq_names(struct platform_device *pdev,
				   const char * const *names, unsigned int count)
{
	struct device *dev = &pdev->dev;
	unsigned int i;
	int irq;

	for (i = 0; i < count; i++) {
		irq = platform_get_irq_byname_optional(pdev, names[i]);
		if (irq < 0)
			return dev_err_probe(dev, irq, "missing %s IRQ\n",
					     names[i]);
	}

	return 0;
}

static int zumapro_dpp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct zumapro_dpp *dpp;
	int ret;

	dpp = devm_kzalloc(dev, sizeof(*dpp), GFP_KERNEL);
	if (!dpp)
		return -ENOMEM;

	dpp->dev = dev;

	ret = zumapro_read_u32_compat(dev, "google,dpp-id", "dpp,id",
				      &dpp->id);
	if (ret)
		return ret;

	if (dpp->id >= ZUMAPRO_DPU_FETCH_DPP_COUNT)
		return dev_err_probe(dev, -EINVAL,
				     "DPP%u is not a normal fetch DPP\n",
				     dpp->id);

	ret = zumapro_read_u32_optional_compat(dev, "google,dpp-attributes",
					       "attr", &dpp->attributes);
	if (ret)
		return ret;

	ret = zumapro_read_u32_optional_compat(dev, "google,axi-port",
					       "port", &dpp->axi_port);
	if (ret)
		return ret;

	ret = zumapro_read_u32_optional_compat(dev, "google,scale-down",
					       "scale_down", &dpp->scale_down);
	if (ret)
		return ret;

	ret = zumapro_read_u32_optional_compat(dev, "google,scale-up",
					       "scale_up", &dpp->scale_up);
	if (ret)
		return ret;

	dpp->video_formats = of_property_read_bool(dev->of_node,
						   "google,video-formats") ||
			     of_property_read_bool(dev->of_node, "dpp,video");

	ret = zumapro_dpp_select_formats(dpp);
	if (ret)
		return ret;
	dpp->restrictions = &zumapro_dpp_restrictions;

	ret = zumapro_check_reg_names(pdev, zumapro_dpp_reg_names,
				      ARRAY_SIZE(zumapro_dpp_reg_names));
	if (ret)
		return ret;

	ret = zumapro_check_irq_names(pdev, zumapro_dpp_irq_names,
				      ARRAY_SIZE(zumapro_dpp_irq_names));
	if (ret)
		return ret;

	platform_set_drvdata(pdev, dpp);
	dev_dbg(dev, "registered passive DPP%u topology\n", dpp->id);

	return 0;
}

static const struct of_device_id zumapro_dpp_of_match[] = {
	{ .compatible = "google,zumapro-dpp" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, zumapro_dpp_of_match);

struct platform_driver zumapro_dpp_driver = {
	.probe = zumapro_dpp_probe,
	.driver = {
		.name = "exynos-zumapro-dpp",
		.of_match_table = zumapro_dpp_of_match,
	},
};

static int zumapro_decon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct zumapro_decon_desc *desc;
	struct zumapro_decon *decon;
	int ret;

	decon = devm_kzalloc(dev, sizeof(*decon), GFP_KERNEL);
	if (!decon)
		return -ENOMEM;

	decon->dev = dev;

	ret = zumapro_read_u32_compat(dev, "google,decon-id", "decon,id",
				      &decon->id);
	if (ret)
		return ret;

	desc = zumapro_decon_desc_by_id(decon->id);
	if (!desc)
		return dev_err_probe(dev, -EINVAL, "unsupported DECON%u\n",
				     decon->id);

	if (decon->id == 0)
		decon->pipeline = &zumapro_decon0_tg4c_pipeline;

	if (desc->has_cgc_dma) {
		ret = zumapro_read_u32_optional_compat(dev,
						       "google,cgc-dma-id",
						       "cgc-dma,id",
						       &decon->cgc_dma_id);
		if (ret)
			return ret;
	}

	ret = zumapro_read_u32_optional_compat(dev, "google,max-windows",
					       "max_win", &decon->max_windows);
	if (ret)
		return ret;

	ret = zumapro_check_reg_names(pdev, desc->reg_names,
				      desc->num_reg_names);
	if (ret)
		return ret;

	ret = zumapro_check_irq_names(pdev, desc->irq_names,
				      desc->num_irq_names);
	if (ret)
		return ret;

	decon->main_regs = devm_platform_ioremap_resource_byname(pdev, "main");
	if (IS_ERR(decon->main_regs))
		return PTR_ERR(decon->main_regs);

	decon->win_regs = devm_platform_ioremap_resource_byname(pdev, "win");
	if (IS_ERR(decon->win_regs))
		return PTR_ERR(decon->win_regs);

	decon->sub_regs = devm_platform_ioremap_resource_byname(pdev, "sub");
	if (IS_ERR(decon->sub_regs))
		return PTR_ERR(decon->sub_regs);

	decon->wincon_regs =
		devm_platform_ioremap_resource_byname(pdev, "wincon");
	if (IS_ERR(decon->wincon_regs))
		return PTR_ERR(decon->wincon_regs);

	if (decon->pipeline) {
		decon->dqe_regs =
			devm_platform_ioremap_resource_byname(pdev, "dqe");
		if (IS_ERR(decon->dqe_regs))
			return PTR_ERR(decon->dqe_regs);
	}

	decon->dpp_count = of_count_phandle_with_args(dev->of_node, "dpps",
						      NULL);
	if (decon->dpp_count == -ENOENT)
		decon->dpp_count = 0;
	else if (decon->dpp_count < 0)
		return dev_err_probe(dev, decon->dpp_count,
				     "failed to parse dpps\n");

	platform_set_drvdata(pdev, decon);
	pm_runtime_enable(dev);

	ret = component_add(dev, &zumapro_decon_component_ops);
	if (ret) {
		pm_runtime_disable(dev);
		return ret;
	}

	dev_info(dev,
		 "registered DECON%u color-map CRTC with %d passive DPPs; modeset gated\n",
		 decon->id, decon->dpp_count);

	return 0;
}

static void zumapro_decon_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	component_del(dev, &zumapro_decon_component_ops);
	pm_runtime_disable(dev);
}

static const struct of_device_id zumapro_decon_of_match[] = {
	{ .compatible = "google,zumapro-decon" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, zumapro_decon_of_match);

struct platform_driver zumapro_decon_driver = {
	.probe = zumapro_decon_probe,
	.remove = zumapro_decon_remove,
	.driver = {
		.name = "exynos-zumapro-decon",
		.of_match_table = zumapro_decon_of_match,
	},
};

MODULE_DESCRIPTION("Google Tensor G4 Zumapro DPU bring-up scaffold");
MODULE_LICENSE("GPL");
