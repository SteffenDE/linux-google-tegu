// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Zumapro camera back-end core -- the surfaces and their geometry
 *
 * Three rasters describe a frame: the Bayer array the front end sends, the
 * chain raster RGBP hands YUVP and MCSC reads back, and the scaled picture
 * MCSC writes.  Everything here derives something from one or two of them --
 * a crop, a zoom ratio, a stride, a plane offset, an allocation size -- or
 * writes one of those derivations into the typed DMA word that carries it.
 *
 * The strides and offsets are the part of this driver with the least margin
 * for a plausible mistake: a formula that agrees with the vendor's at the one
 * raster the driver was written at is not the vendor's formula, and the
 * captures say so only where the two disagree.  tools/camera-becore-typed.py
 * checks the eight geometry-dependent typed words against every capture in
 * the corpus, and it is the only thing that does.
 */

#include <linux/align.h>
#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/build_bug.h>
#include <linux/dev_printk.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/types.h>

#include "exynos-becore-regs.h"
#include "exynos-becore-common.h"

/*
 * How the Bayer arrives, from the live ultrawide program: the DMA fields and
 * nothing about its size.  As in Pablo's common DMA API the payload and header
 * geometry are derived rather than carried, and Lyric additionally writes the
 * 256-pixel-aligned SBWC storage width after enabling the RDMA.
 *
 * The size lives in becore->array, because it is a property of the picture and
 * not of the format it is stored in -- the same format describes any of them.
 * Both are resolved once and passed to each derivation rather than read from
 * module scope, which is what lets the offline loop encode for a geometry the
 * hardware is not running.
 */
static const struct becore_rgbp_input_profile
becore_rgbp_inputs[BECORE_RGBP_INPUT_PROFILE_COUNT] = {
	[BECORE_RGBP_INPUT_SBWC] = {
		.data_format = 0x18,
		.comp_control = 0x9,
		.sbwc_block_width = 256,
		.bytes_per_pixel = 2,
		.header_stride = 0x40,
		.businfo = 0,
	},
	/*
	 * The same image without compression, so that a frame chosen rather
	 * than captured can be put through the offline loop.  Lyric's own
	 * BuildFormatConfig picks between the two formats and writes 0x1a when
	 * it is not compressing -- a three-instruction branch, `tst` on
	 * IsSbwcBufferCompression and a `csel` of 0x18 against 0x1a -- so this
	 * is the vendor's own counterpart of the profile above rather than a
	 * guess: same signedness, same depth, one 16-bit little-endian sample
	 * per pixel right-aligned in the low twelve bits.
	 *
	 * An SBWC block width of one makes the storage width the active width,
	 * which is what an uncompressed stride is, and there is no header
	 * plane.  Only the offline loop may select it: the live producer writes
	 * compressed Bayer, so reading its pages under this profile would
	 * decode payload as pixels.
	 */
	[BECORE_RGBP_INPUT_LINEAR] = {
		.data_format = 0x1a,
		.comp_control = 0,
		.sbwc_block_width = 1,
		.bytes_per_pixel = 2,
		.header_stride = 0,
		.businfo = 0,
	},
};

const u32 becore_rgbp_input_regs[] = {
	[BECORE_RGBP_CROP_SIZE] = BECORE_RGBP_CROP_SIZE_REG,
	[BECORE_RGBP_CROP_START] = BECORE_RGBP_CROP_START_REG,
	[BECORE_RGBP_SC_DST_SIZE] = BECORE_RGBP_SC_DST_SIZE_REG,
	[BECORE_RGBP_SC_H_RATIO] = BECORE_RGBP_SC_H_RATIO_REG,
	[BECORE_RGBP_SC_V_RATIO] = BECORE_RGBP_SC_V_RATIO_REG,
	[BECORE_RGBP_CHAIN_SRC_SIZE] = BECORE_RGBP_CHAIN_SRC_SIZE_REG,
	[BECORE_RGBP_CHAIN_DST_SIZE] = BECORE_RGBP_CHAIN_DST_SIZE_REG,
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

/* Pixel uses a combined, two-plane WDMA where Pablo v1.1 uses split blocks. */
const struct becore_yuvp_output_profile becore_yuvp_outputs[] = {
	[BECORE_YUVP_OUTPUT_SBWCL] = {
		.data_format = 0x2000,
		.mode = 0xa,
		.lossy_byte32num = 2,
		.votf_enable = 3,
		.block_width = 32,
		.block_height = 4,
		.luma_height_align = 16,
		.plane_gap = 0x40,
		.businfo = 0,
	},
	[BECORE_YUVP_OUTPUT_P010] = {
		.data_format = 0x2000,
		.mode = 0,
		.lossy_byte32num = 0,
		.votf_enable = 0,
		.bytes_per_pixel = 2,
		.businfo = 0,
	},
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

/*
 * The first physical-ultrawide GTNR frame has no temporal inputs or map DMAs.
 * It reads YUVP's 4160x3120 lossy-SBWC surface and writes a separate surface
 * with the same bounded layout.  Keep this startup shape dormant until GTNR's
 * power, reset, interrupt, and cross-block completion lifecycle is established.
 */
static const struct becore_gtnr_dma_profile becore_gtnr_input = {
	.votf_enable = 1,
	.businfo = 1,
	.max_mo = 0x100,
	.max_bl = 0x10,
	.enable = 1,
};

static const struct becore_gtnr_dma_profile becore_gtnr_output = {
	.businfo = 0,
	.max_mo = 0x100,
	.max_bl = 0x10,
	.enable = 1,
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

/*
 * The physical-ultrawide 4000x3000 request carries YUVP's 4160x3120 lossy
 * SBWC intermediate into MCSC and writes output zero as linear NV21.  The
 * captured program used VOTF; a sequential run replaces only that transport
 * control so MCSC reads the completed driver-owned surface from memory.
 */
static const struct becore_mcsc_dma_profile becore_mcsc_input = {
	.votf_enable = 0x00400001,
	.businfo = 2,
	.max_bl = 0x10,
	.enable = 1,
};


static const struct becore_mcsc_dma_profile becore_mcsc_output = {
	.data_format = 0x800,
	.comp_control = 0,
	.businfo = 0,
	.max_bl = 4,
	.enable = 1,
	.dither = 0x10,
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
	[BECORE_MCSC_DJAG_IMG_SIZE] = BECORE_MCSC_DJAG_IMG_SIZE_REG,
	[BECORE_MCSC_DJAG_PS_SRC_POS] = BECORE_MCSC_DJAG_PS_SRC_POS_REG,
	[BECORE_MCSC_DJAG_PS_SRC_SIZE] = BECORE_MCSC_DJAG_PS_SRC_SIZE_REG,
	[BECORE_MCSC_DJAG_PS_DST_SIZE] = BECORE_MCSC_DJAG_PS_DST_SIZE_REG,
	[BECORE_MCSC_DJAG_PS_H_RATIO] = BECORE_MCSC_DJAG_PS_H_RATIO_REG,
	[BECORE_MCSC_DJAG_PS_V_RATIO] = BECORE_MCSC_DJAG_PS_V_RATIO_REG,
};

static u32
becore_rgbp_input_storage_width(const struct becore_rgbp_input_profile *profile,
				const struct becore_raster *array)
{
	return ALIGN(array->width, profile->sbwc_block_width);
}

static u32
becore_rgbp_input_stride(const struct becore_rgbp_input_profile *profile,
			 const struct becore_raster *array)
{
	return becore_rgbp_input_storage_width(profile, array) *
	       profile->bytes_per_pixel;
}

size_t
becore_rgbp_input_image_offset(const struct becore_rgbp_input_profile *profile,
			       const struct becore_raster *array)
{
	return (size_t)profile->header_stride * array->height;
}

size_t
becore_rgbp_input_size(const struct becore_rgbp_input_profile *profile,
		       const struct becore_raster *array)
{
	size_t image_bytes = (size_t)becore_rgbp_input_stride(profile, array) *
			     array->height;

	return ALIGN(becore_rgbp_input_image_offset(profile, array) +
		     image_bytes, SZ_4K);
}

/*
 * Every input slot is sized for the live path, because that is what fills
 * them: ISPFE writes SBWC-compressed main Bayer straight into a slot, and its
 * own probe checks the size it is handed.  A harness profile therefore has to
 * fit inside that allocation rather than resize it, which is what
 * becore_input_profiles_validate() checks once at probe.
 */
size_t becore_input_allocation_size(const struct becore_raster *array)
{
	return becore_rgbp_input_size(&becore_rgbp_inputs[BECORE_RGBP_INPUT_SBWC],
				      array);
}

/* The array raster is becore_raster_validate()'s; this checks the profiles. */
int becore_input_profiles_validate(struct device *dev,
				   const struct becore_raster *array)
{
	unsigned int i;

	for (i = 0; i < BECORE_RGBP_INPUT_PROFILE_COUNT; i++) {
		const struct becore_rgbp_input_profile *profile =
			&becore_rgbp_inputs[i];

		if (!profile->bytes_per_pixel ||
		    !is_power_of_2(profile->sbwc_block_width))
			return dev_err_probe(dev, -EINVAL,
					     "input profile %u is degenerate\n",
					     i);
		if (becore_rgbp_input_size(profile, array) >
		    becore_input_allocation_size(array))
			return dev_err_probe(dev, -EINVAL,
					     "input profile %u wants %zu bytes, the slot holds %zu\n",
					     i, becore_rgbp_input_size(profile, array),
					     becore_input_allocation_size(array));
	}

	return 0;
}

/*
 * The same for the surfaces YUVP writes, and it is the divisors that make it
 * worth having: a compressed profile's stride and both plane sizes divide by
 * its block geometry, and its stride is only the expression written above
 * while the compressed rate is even.
 */
int becore_output_profiles_validate(struct device *dev)
{
	unsigned int i;

	for (i = 0; i < BECORE_YUVP_OUTPUT_PROFILE_COUNT; i++) {
		const struct becore_yuvp_output_profile *profile =
			&becore_yuvp_outputs[i];

		if (!profile->mode) {
			if (!profile->bytes_per_pixel)
				return dev_err_probe(dev, -EINVAL,
						     "output profile %u is uncompressed and has no pixel size\n",
						     i);
			continue;
		}
		if (!profile->block_width || !profile->block_height ||
		    !profile->luma_height_align)
			return dev_err_probe(dev, -EINVAL,
					     "output profile %u is compressed and has no block geometry\n",
					     i);
		if (!profile->lossy_byte32num || profile->lossy_byte32num & 1)
			return dev_err_probe(dev, -EINVAL,
					     "output profile %u has a compressed rate of %u, and becore_yuvp_output_stride() states only even ones\n",
					     i, profile->lossy_byte32num);
	}

	return 0;
}

/*
 * Two conventions the Exynos ISP blocks share, so they live here rather than
 * with any one of them.
 *
 * Geometry registers pack two 16-bit halves into one word with the width in
 * the high one. Scaling ratios are a 20-bit fixed-point fraction of the
 * destination, truncated -- Samsung spells this GET_ZOOM_RATIO(in, out),
 * ((in) << MCSC_PRECISION) / (out) with MCSC_PRECISION 20. The captured
 * programs are the check on the rounding: DJAG's 3536 << 20 over 4000 is
 * 926941.18 and the vendor writes 926941.
 */
u32 becore_pack_size(u32 high, u32 low)
{
	return (high << 16) | low;
}

/*
 * A ratio is a Q20 fraction in a u32, so it describes a downscale of just
 * under 4096 and no further.  Past that the shifted numerator leaves 32 bits
 * and the truncation lands somewhere small: an in/out of exactly 4096 reads
 * back as **zero**, a scaler told to step nothing per output pixel, and 4097
 * reads back as unity.  Either way becore_sc_coeff_set() then picks the x8/8
 * filter for it, because both are at or below its first band.  Neither the
 * register nor the filter is a failed write, so say so rather than write it.
 */
int becore_zoom_ratio(u32 in, u32 out, u32 *ratio)
{
	u64 scaled;

	if (!out)
		return -EINVAL;
	scaled = div_u64((u64)in << BECORE_RATIO_SHIFT, out);
	if (scaled > U32_MAX)
		return -ERANGE;
	*ratio = scaled;

	return 0;
}

/*
 * Where MCSC's scaler starts, which is a function of its ratio rather than a
 * constant: half a source step when it stretches and nothing when it shrinks.
 * Ten captured POLY_SC0 programs say so -- the six that shrink write zero, and
 * the four that stretch write half the ratio, odd values included, 0x0ffdf4
 * giving 0x0007fefa.
 *
 * **It is three scalers' rule and not all four's**, which is the thing not to
 * generalise from the register's name. DJAG's pre-scaler has the same field in
 * the same shape and does not follow it: seven captured programs run it as a
 * stretcher, at ratios from 889105 to 974302 against a unity of 1048576, and
 * every one of them writes an origin of zero where this would put 444552 to
 * 487151. So becore_mcsc_djag_value() states zero because that is what the
 * silicon is told, not because nothing has moved yet.
 *
 * Two sources say so and they agree.  Samsung's kernel driver carries
 * `if (hratio < RATIO_X8_8) h_phase_offset = hratio >> 1;` in
 * is_scaler_set_poly_scaler_coef() and is_scaler_set_post_scaler_coef(), while
 * every caller of is_scaler_set_djag_init_phase_offset() passes a literal 0 --
 * four of them, unconditionally, on the line after the ratio is set.  And
 * Lyric, which is what actually programs this silicon, has no ratio arithmetic
 * in its DJAG function at all: lyric::McscCropAndUpScaleBlock's writes two
 * literal zeros and then the round mode's 1.
 */
u32 becore_scaler_init_phase(u32 ratio)
{
	return ratio < BECORE_RATIO_UNITY ? ratio >> 1 : 0;
}

/* One bound, so the two fields it is the bound for have to be one width. */
static_assert(BECORE_YUVNR_CENTRE_MASK == BECORE_RGBP_DNS_CENTRE_MASK);

/* See becore_chain_validate(); the chroma plane is what asks for the four. */
#define BECORE_CHAIN_WIDTH_ALIGN	4U

int becore_raster_validate(struct device *dev, const char *name,
			   const struct becore_raster *raster, u32 max)
{
	if (!raster->width || !raster->height)
		return dev_err_probe(dev, -EINVAL,
				     "%s raster %ux%u has a zero extent\n",
				     name, raster->width, raster->height);
	if ((raster->width | raster->height) & 1)
		return dev_err_probe(dev, -EINVAL,
				     "%s raster %ux%u has an odd extent\n",
				     name, raster->width, raster->height);
	if (raster->width > max || raster->height > max)
		return dev_err_probe(dev, -EINVAL,
				     "%s raster %ux%u exceeds %u\n",
				     name, raster->width, raster->height, max);

	return 0;
}

/*
 * The chain raster is the one with a width alignment of its own, and it is
 * four rather than the two an even extent gives.  The chain surface carries
 * 4:2:0 chroma interleaved at half the luma width, and the DMA blocks that
 * plane the same way it blocks luma, so a merely even width leaves the chroma
 * with an odd sample count.
 *
 * MCSC's stage then never completes at all: not an error interrupt but a
 * timeout, which spends the run's whole budget and leaves the processors
 * unproven, so it is much better refused.  Measured across widths at every
 * residue -- 2578, 2582, 2586, 4158, 4162, 1506 and 1510 all hang where 2576,
 * 2580, 2584, 4160 and 1508 all run -- and it is the width alone: the chain's
 * height and both of the scaled output's extents are fine at merely even.
 * Every captured chain raster is a multiple of four.  [HW 2026-08-24]
 */
int becore_chain_validate(struct device *dev,
			  const struct becore_raster *chain)
{
	int ret = becore_raster_validate(dev, "chain", chain,
					 BECORE_RASTER_EXTENT_MAX);

	if (ret)
		return ret;
	if (chain->width % BECORE_CHAIN_WIDTH_ALIGN)
		return dev_err_probe(dev, -EINVAL,
				     "chain raster %ux%u has a width that is not a multiple of %u\n",
				     chain->width, chain->height,
				     BECORE_CHAIN_WIDTH_ALIGN);

	return 0;
}

/*
 * DMSCCROP's window: the largest centred rectangle of the Bayer array that
 * has the aspect ratio the chain hands downstream. Cropping is what makes the
 * two aspects agree, so only one axis is ever narrowed; the other keeps the
 * whole array.
 *
 * This is a policy choice rather than an arithmetic one, and the vendor makes
 * a different one on some readouts -- it takes the aspect fit on every 4:3
 * request, but keeps an eight-pixel margin on the rear camera's 16:9 modes.
 * What is *not* a choice is that everything below reads the crop rather than
 * the array: the scaler's ratios are the crop over the destination, and that
 * holds bit-exactly on all eighteen captured programs where taking the array
 * would be right only where the two coincide.
 *
 * Both sizes are kept even, and the margin is widened to a multiple of four so
 * that the centred origin lands on an even pixel: a window on the wrong Bayer
 * phase swaps the colours with no other symptom. The odd-origin test below is
 * therefore unreachable by construction and is kept as a guard on that
 * reasoning rather than on the arithmetic.
 */
int becore_rgbp_crop(const struct becore_raster *array,
		     const struct becore_raster *chain,
		     struct becore_rect *crop)
{
	u32 array_w = array->width;
	u32 array_h = array->height;
	u32 out_w = chain->width;
	u32 out_h = chain->height;
	u32 width;

	if (!out_w || !out_h || !array_w || !array_h)
		return -EINVAL;
	/*
	 * Widening the margin to a multiple of four keeps the crop's sizes
	 * even only while the array's own dimensions are, and a Bayer array
	 * with an odd dimension has no whole last quad in any case.
	 */
	if ((array_w | array_h) & 1)
		return -EINVAL;

	width = 2 * (u32)DIV_ROUND_CLOSEST_ULL((u64)array_h * out_w,
					       2 * out_h);
	if (width <= array_w) {
		crop->width = width;
		crop->height = array_h;
	} else {
		u64 fit = (u64)array_w * out_h;

		crop->width = array_w;
		crop->height = 2 * (u32)DIV_ROUND_CLOSEST_ULL(fit, 2 * out_w);
	}
	if (crop->height > array_h || !crop->width || !crop->height)
		return -ERANGE;
	/*
	 * A centred window is only on the right Bayer phase when its margin
	 * is a multiple of four, so widen it by the two pixels that are in
	 * the way rather than accept an off-phase origin. That costs at most
	 * a two-pixel aspect error, which is what the vendor accepts as well:
	 * on both 16:9 readouts where the exact fit lands here it stops
	 * cropping altogether, and widening reaches the same answer.
	 */
	crop->width += (array_w - crop->width) % 4;
	crop->height += (array_h - crop->height) % 4;
	crop->x = (array_w - crop->width) / 2;
	crop->y = (array_h - crop->height) / 2;
	if ((crop->x | crop->y) & 1)
		return -ERANGE;

	return 0;
}

/*
 * Every word RGBP's typed input profile supplies, computed whether or not the
 * caller wants the value: passing a null pointer is the probe-time validation
 * pass, and short-circuiting it there would mean the four crop-derived words
 * are the only ones never checked before the hardware sees them.
 */
int becore_rgbp_input_value(const struct becore_rgbp_input_profile *profile,
			    const struct becore_raster *array,
			    const struct becore_raster *chain,
			    u32 index, u32 reg, u32 *value)
{
	u32 result;

	if (index >= BECORE_RGBP_INPUT_WORD_COUNT ||
	    reg != becore_rgbp_input_regs[index])
		return -EINVAL;

	switch (index) {
	case BECORE_RGBP_CHAIN_SRC_SIZE:
		result = becore_pack_size(array->width, array->height);
		break;
	case BECORE_RGBP_CHAIN_DST_SIZE:
	case BECORE_RGBP_SC_DST_SIZE:
		result = becore_pack_size(chain->width, chain->height);
		break;
	case BECORE_RGBP_CROP_SIZE:
	case BECORE_RGBP_CROP_START:
	case BECORE_RGBP_SC_H_RATIO:
	case BECORE_RGBP_SC_V_RATIO: {
		struct becore_rect crop;
		int ret = becore_rgbp_crop(array, chain, &crop);

		if (ret)
			return ret;
		if (index == BECORE_RGBP_CROP_SIZE)
			result = becore_pack_size(crop.width, crop.height);
		else if (index == BECORE_RGBP_CROP_START)
			result = becore_pack_size(crop.x, crop.y);
		else if (index == BECORE_RGBP_SC_H_RATIO)
			ret = becore_zoom_ratio(crop.width, chain->width,
						&result);
		else
			ret = becore_zoom_ratio(crop.height, chain->height,
						&result);
		if (ret)
			return ret;
		break;
	}
	case BECORE_RGBP_INPUT_FORMAT:
		result = profile->data_format;
		break;
	case BECORE_RGBP_INPUT_COMP:
		result = profile->comp_control;
		break;
	case BECORE_RGBP_INPUT_ACTIVE_WIDTH:
		result = array->width;
		break;
	case BECORE_RGBP_INPUT_HEIGHT:
		result = array->height;
		break;
	case BECORE_RGBP_INPUT_STRIDE:
		result = becore_rgbp_input_stride(profile, array);
		break;
	case BECORE_RGBP_INPUT_HEADER_STRIDE:
		result = profile->header_stride;
		break;
	case BECORE_RGBP_INPUT_BUSINFO:
		result = profile->businfo;
		break;
	case BECORE_RGBP_INPUT_ENABLE:
		result = 1;
		break;
	case BECORE_RGBP_INPUT_STORAGE_WIDTH:
		result = becore_rgbp_input_storage_width(profile, array);
		break;
	default:
		return -EINVAL;
	}

	if (value)
		*value = result;

	return 0;
}

const struct becore_rgbp_input_profile *
becore_rgbp_input_profile(const struct becore_device *becore)
{
	return &becore_rgbp_inputs[becore->active_input_profile];
}

const struct becore_yuvp_output_profile *
becore_yuvp_output_profile(const struct becore_device *becore)
{
	return &becore_yuvp_outputs[becore->active_output_profile];
}

static u32
becore_yuvp_output_stride(const struct becore_yuvp_output_profile *profile,
			  const struct becore_raster *chain)
{
	if (!profile->mode)
		return chain->width * profile->bytes_per_pixel;

	/*
	 * A compressed surface's stride is a count of whole blocks, each of a
	 * fixed compressed size -- Samsung's is_hw_dma_get_payload_stride()
	 * for a lossy 32x4 surface.  A width that does not fill its last block
	 * still occupies one.
	 *
	 * That function branches on a 32- or 64-byte alignment, and the two
	 * branches collapse to this one expression while lossy_byte32num is
	 * even: the odd-rate correction term vanishes and both alignments
	 * become no-ops.  Ours is 2, so the alignment does not have to be
	 * known -- which is the only reason it is not carried here.  An odd
	 * rate would need both, and is refused where the profiles are checked.
	 */
	return DIV_ROUND_UP(chain->width, profile->block_width) *
	       profile->lossy_byte32num * profile->block_width;
}

/*
 * The chain surface: what YUVP writes and GTNR and MCSC read back.  One
 * surface, so one description, and the profile that writes it is the one that
 * knows it -- a reader that states the format again is a second answer to a
 * question already answered, and it is the answer that goes wrong silently,
 * because a reader that disagrees about a layout still reads a frame's worth
 * of bytes and produces a picture.
 *
 * It is always the lossy-SBWC profile, not whichever one is active: MCSC is
 * refused a run against any other (see becore_run_frame()), and GTNR's program
 * is written against this one.  So this names the surface rather than taking
 * the caller's, which also lets a value function with no device pointer ask.
 */
static const struct becore_yuvp_output_profile *becore_chain_surface(void)
{
	return &becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL];
}

static u32 becore_chain_stride(const struct becore_raster *chain)
{
	return becore_yuvp_output_stride(becore_chain_surface(), chain);
}

static size_t
becore_yuvp_output_plane2_offset(const struct becore_yuvp_output_profile *profile,
				 const struct becore_raster *chain)
{
	u32 stride = becore_yuvp_output_stride(profile, chain);
	u32 luma_height;
	u32 luma_rows;

	if (!profile->mode)
		return ALIGN((size_t)stride * chain->height, SZ_4K);

	luma_height = ALIGN(chain->height, profile->luma_height_align);
	luma_rows = DIV_ROUND_UP(luma_height, profile->block_height);

	return (size_t)stride * luma_rows + profile->plane_gap;
}

size_t
becore_yuvp_output_size(const struct becore_yuvp_output_profile *profile,
			const struct becore_raster *chain)
{
	u32 stride = becore_yuvp_output_stride(profile, chain);
	u32 chroma_height = DIV_ROUND_UP(chain->height, 2);
	u32 chroma_rows;
	size_t chroma_bytes;

	if (profile->mode)
		chroma_rows = DIV_ROUND_UP(chroma_height, profile->block_height);
	else
		chroma_rows = chroma_height;
	chroma_bytes = (size_t)stride * chroma_rows;

	return ALIGN(becore_yuvp_output_plane2_offset(profile, chain) +
		     chroma_bytes, SZ_4K);
}

static size_t
becore_yuvp_packed_plane2_offset(const struct becore_yuvp_output_profile *profile,
				 const struct becore_raster *chain)
{
	return (size_t)becore_yuvp_output_stride(profile, chain) * chain->height;
}

static size_t
becore_yuvp_packed_output_size(const struct becore_yuvp_output_profile *profile,
			       const struct becore_raster *chain)
{
	u32 chroma_height = DIV_ROUND_UP(chain->height, 2);

	return becore_yuvp_packed_plane2_offset(profile, chain) +
	       (size_t)becore_yuvp_output_stride(profile, chain) * chroma_height;
}

size_t becore_active_output_plane2_offset(const struct becore_device *becore)
{
	const struct becore_yuvp_output_profile *profile =
		becore_yuvp_output_profile(becore);

	if (becore->active_output_packed)
		return becore_yuvp_packed_plane2_offset(profile, &becore->chain);

	return becore_yuvp_output_plane2_offset(profile, &becore->chain);
}

size_t becore_active_output_size(const struct becore_device *becore)
{
	const struct becore_yuvp_output_profile *profile =
		becore_yuvp_output_profile(becore);

	if (becore->active_output_packed)
		return becore_yuvp_packed_output_size(profile, &becore->chain);

	return becore_yuvp_output_size(profile, &becore->chain);
}

size_t
becore_yuvp_output_allocation_size(const struct becore_raster *chain)
{
	const struct becore_yuvp_output_profile *sbwcl =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_SBWCL];
	const struct becore_yuvp_output_profile *p010 =
		&becore_yuvp_outputs[BECORE_YUVP_OUTPUT_P010];

	return max(becore_yuvp_output_size(sbwcl, chain),
		   becore_yuvp_output_size(p010, chain));
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
		*value = becore->chain.width;
		break;
	case BECORE_YUVP_OUTPUT_HEIGHT:
		*value = becore->chain.height;
		break;
	case BECORE_YUVP_OUTPUT_STRIDE1:
	case BECORE_YUVP_OUTPUT_STRIDE2:
		*value = becore_yuvp_output_stride(profile, &becore->chain);
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

int becore_gtnr_dma_value(const struct becore_raster *chain, u32 index,
			  u32 reg, u32 *value)
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
		*value = becore_chain_surface()->data_format;
		break;
	case BECORE_GTNR_INPUT_LOSSY:
		*value = becore_chain_surface()->lossy_byte32num;
		break;
	case BECORE_GTNR_INPUT_COMP:
		*value = becore_chain_surface()->mode;
		break;
	case BECORE_GTNR_INPUT_WIDTH:
		*value = chain->width;
		break;
	case BECORE_GTNR_INPUT_HEIGHT:
		*value = chain->height;
		break;
	case BECORE_GTNR_INPUT_STRIDE1:
	case BECORE_GTNR_INPUT_STRIDE2:
		*value = becore_chain_stride(chain);
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
		*value = becore_chain_surface()->data_format;
		break;
	case BECORE_GTNR_OUTPUT_LOSSY:
		*value = becore_chain_surface()->lossy_byte32num;
		break;
	case BECORE_GTNR_OUTPUT_COMP:
		*value = becore_chain_surface()->mode;
		break;
	case BECORE_GTNR_OUTPUT_WIDTH:
		*value = chain->width;
		break;
	case BECORE_GTNR_OUTPUT_HEIGHT:
		*value = chain->height;
		break;
	case BECORE_GTNR_OUTPUT_STRIDE1:
	case BECORE_GTNR_OUTPUT_STRIDE2:
		*value = becore_chain_stride(chain);
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

/*
 * The scaled output's stride: its width rounded up to 64 bytes, one byte to a
 * luma sample.  4000 becomes 4032, which is what the vendor's every captured
 * program writes and what the driver used to carry as 0xfc0.  It is the
 * node's `bytesperline` as well, so a stated one would be a promise to
 * userspace that the DMA had stopped keeping.
 */
#define BECORE_MCSC_OUTPUT_STRIDE_ALIGN	64

u32 becore_mcsc_output_stride(const struct becore_raster *output)
{
	return ALIGN(output->width, BECORE_MCSC_OUTPUT_STRIDE_ALIGN);
}

static size_t
becore_mcsc_output_plane2_offset(const struct becore_raster *output)
{
	return (size_t)becore_mcsc_output_stride(output) * output->height;
}

size_t
becore_mcsc_output_active_size(const struct becore_raster *output)
{
	size_t chroma = (size_t)becore_mcsc_output_stride(output) *
			DIV_ROUND_UP(output->height, 2);

	return becore_mcsc_output_plane2_offset(output) + chroma;
}

size_t becore_mcsc_output_size(const struct becore_raster *output)
{
	return ALIGN(becore_mcsc_output_active_size(output), SZ_4K);
}

/*
 * How much of DJAG's input the output is taken from. The captured program
 * crops 3536 x 2652 out of the 4160 x 3120 YUVP surface and scales that up to
 * fill 4000 x 3000, reserving a margin the vendor spends on electronic
 * stabilisation. This driver has no stabilisation to spend it on, so it takes
 * the whole raster and downscales instead, covering the full field.
 *
 * These say that against the raster rather than beside it. A margin carried
 * separately would have gone on describing the raster the driver was built
 * against, and DJAG would have cropped a window larger than its own input the
 * first time the raster moved.
 */
static u32 becore_mcsc_djag_crop_width(const struct becore_raster *chain)
{
	return chain->width;
}

static u32 becore_mcsc_djag_crop_height(const struct becore_raster *chain)
{
	return chain->height;
}

/*
 * The crop is centred in the scaler's input, so its origin is derived rather
 * than carried. An odd margin would land the window off a chroma boundary on a
 * 4:2:0 output, so refuse it instead of silently rounding.
 */
static int becore_mcsc_djag_origin(const struct becore_raster *chain,
				   u32 *x, u32 *y)
{
	if (becore_mcsc_djag_crop_width(chain) > chain->width ||
	    becore_mcsc_djag_crop_height(chain) > chain->height)
		return -ERANGE;
	*x = (chain->width - becore_mcsc_djag_crop_width(chain)) / 2;
	*y = (chain->height - becore_mcsc_djag_crop_height(chain)) / 2;
	if ((*x | *y) & 1)
		return -ERANGE;

	return 0;
}

int
becore_mcsc_dma_value(const struct becore_raster *chain,
		      const struct becore_raster *output, u32 index, u32 reg,
		      enum becore_mcsc_input_transport transport,
		      const u32 *requested_token, u32 *value)
{
	if (index >= BECORE_MCSC_DMA_WORD_COUNT ||
	    reg != becore_mcsc_dma_regs[index])
		return -EINVAL;
	if (!value)
		return 0;

	switch (index) {
	case BECORE_MCSC_INPUT_VOTF:
		/*
		 * Lyric builds this word two ways, and which one depends on
		 * the direction rather than on the block: a consumer gets its
		 * stall-line count in bits 29:16 with the enable in bit 0, and
		 * a producer gets a constant 2 with the enable in bit 0 -- the
		 * captured 0x00400001 here and 3 on YUVP's side.  The stall
		 * count has to be the same number as this endpoint's C2SERV
		 * token or the block and the fabric wait for different things,
		 * so generating it keeps them together when the token is
		 * swept.  There is one such register per DMA engine and none
		 * per plane.
		 */
		*value = transport == BECORE_MCSC_INPUT_MEMORY ? 0 :
			 becore_mcsc_votf_enable(requested_token);
		break;
	case BECORE_MCSC_INPUT_FORMAT:
		*value = becore_chain_surface()->data_format;
		break;
	case BECORE_MCSC_INPUT_LOSSY:
		*value = becore_chain_surface()->lossy_byte32num;
		break;
	case BECORE_MCSC_INPUT_COMP:
		*value = becore_chain_surface()->mode;
		break;
	case BECORE_MCSC_INPUT_WIDTH:
		*value = chain->width;
		break;
	case BECORE_MCSC_INPUT_HEIGHT:
		*value = chain->height;
		break;
	case BECORE_MCSC_INPUT_STRIDE1:
	case BECORE_MCSC_INPUT_STRIDE2:
		*value = becore_chain_stride(chain);
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
		*value = output->width;
		break;
	case BECORE_MCSC_OUTPUT_HEIGHT:
		*value = output->height;
		break;
	case BECORE_MCSC_OUTPUT_STRIDE1:
	case BECORE_MCSC_OUTPUT_STRIDE2:
		*value = becore_mcsc_output_stride(output);
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
	case BECORE_MCSC_DJAG_IMG_SIZE:
		*value = becore_pack_size(chain->width, chain->height);
		break;
	case BECORE_MCSC_DJAG_PS_SRC_POS: {
		u32 x, y;
		int ret = becore_mcsc_djag_origin(chain, &x, &y);

		if (ret)
			return ret;
		*value = becore_pack_size(x, y);
		break;
	}
	case BECORE_MCSC_DJAG_PS_SRC_SIZE:
		*value = becore_pack_size(becore_mcsc_djag_crop_width(chain),
					  becore_mcsc_djag_crop_height(chain));
		break;
	case BECORE_MCSC_DJAG_PS_DST_SIZE:
		*value = becore_pack_size(output->width, output->height);
		break;
	case BECORE_MCSC_DJAG_PS_H_RATIO:
		return becore_zoom_ratio(becore_mcsc_djag_crop_width(chain),
					 output->width, value);
	case BECORE_MCSC_DJAG_PS_V_RATIO:
		return becore_zoom_ratio(becore_mcsc_djag_crop_height(chain),
					 output->height, value);
	default:
		return -EINVAL;
	}

	return 0;
}

static size_t
becore_gtnr_surface_plane2_offset(const struct becore_raster *chain)
{
	return becore_yuvp_output_plane2_offset(becore_chain_surface(), chain);
}

size_t becore_gtnr_surface_size(const struct becore_raster *chain)
{
	return becore_yuvp_output_size(becore_chain_surface(), chain);
}

dma_addr_t becore_gtnr_address_dma(struct becore_device *becore, u32 reg)
{
	switch (reg) {
	case BECORE_GTNR_INPUT_PLANE1_REG:
		return becore->output.dma;
	case BECORE_GTNR_INPUT_PLANE2_REG:
		return becore->output.dma + becore_gtnr_surface_plane2_offset(&becore->chain);
	case BECORE_GTNR_OUTPUT_PLANE1_REG:
		return becore->gtnr_output.dma;
	case BECORE_GTNR_OUTPUT_PLANE2_REG:
		return becore->gtnr_output.dma +
		       becore_gtnr_surface_plane2_offset(&becore->chain);
	default:
		return DMA_MAPPING_ERROR;
	}
}

dma_addr_t becore_mcsc_address_dma(struct becore_device *becore, u32 reg)
{
	switch (reg) {
	case BECORE_MCSC_INPUT_PLANE1_REG:
		return becore->output.dma;
	case BECORE_MCSC_INPUT_PLANE2_REG:
		return becore->output.dma +
		       becore_yuvp_output_plane2_offset(becore_chain_surface(),
							&becore->chain);
	case BECORE_MCSC_OUTPUT_PLANE1_REG:
		return becore->mcsc_dest_dma;
	case BECORE_MCSC_OUTPUT_PLANE2_REG:
		return becore->mcsc_dest_dma +
		       becore_mcsc_output_plane2_offset(&becore->scaled);
	default:
		return DMA_MAPPING_ERROR;
	}
}

u32 becore_typed_word_count(enum becore_block_id id)
{
	if (id == BECORE_RGBP)
		return BECORE_RGBP_INPUT_WORD_COUNT;
	if (id == BECORE_YUVP)
		return BECORE_YUVP_OUTPUT_WORD_COUNT;

	return 0;
}

int becore_typed_value(struct becore_device *becore,
		       enum becore_block_id id, u32 index, u32 reg,
		       u32 *value)
{
	if (id == BECORE_RGBP)
		return becore_rgbp_input_value(becore_rgbp_input_profile(becore),
					       &becore->array, &becore->chain,
					       index, reg, value);
	if (id == BECORE_YUVP)
		return becore_yuvp_output_value(becore, index, reg, value);

	return -EINVAL;
}
