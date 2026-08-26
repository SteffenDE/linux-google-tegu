// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Zumapro camera back-end core -- the parameters node
 *
 * Under ADR 0009 the kernel owns the hardware description and the register
 * encoding, and userspace owns the per-frame image-quality values.  This file
 * is that boundary: the V4L2_BUF_TYPE_META_OUTPUT node a buffer arrives on,
 * the checks that admit one, and the per-block encoders that turn the values
 * in it into register words.
 *
 * Everything here reads a struct exynos_becore_params_* or the state one was
 * copied into.  A register the driver fills in with no buffer to fill it from
 * is a generated word and lives with the rest of them; the two meet in
 * becore_generated_value(), which calls the encoders below with a neutral
 * block so that a stream with no parameters runs the same arithmetic.
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/build_bug.h>
#include <linux/dev_printk.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/math.h>
#include <linux/minmax.h>
#include <linux/overflow.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

#include "exynos-becore-regs.h"
#include "exynos-becore-common.h"
#include "exynos-becore-sharpen.h"
#include "exynos-becore-yuvnr.h"
#include "exynos-becore-byrdns.h"
#include "exynos-becore-dmsc.h"

/*
 * The tuning a stream with no parameters buffer runs on: nothing, bar the one
 * value whose nothing is not a value.  Every other field is zero and the
 * block's own inverted bypass turns that into the noise reducer off, as
 * the sharpener's default does -- but the noise curve's domain is divided by,
 * so eight zeros there would be a division by nothing rather than a flat
 * curve, and becore_params_check_yuvnr() refuses the same eight from
 * userspace.  BECORE_YUVNR_DEFAULT_CURVE is the vendor's own answer to that,
 * read off SetDefaultTuningCommon rather than chosen here.
 *
 * Running the encode over this block rather than taking each entry's constant
 * -- which is how the sharpener produces its default -- is not a stylistic
 * difference.  This block has a field whose zero is *not* the constant's:
 * `enable` inverts, so the default has to go through the same arithmetic a
 * buffer does, and a default that read the constants would leave the noise
 * reducer running with no tuning at all.
 */
const struct exynos_becore_params_yuvnr becore_yuvnr_off = {
	BECORE_YUVNR_DEFAULT_CURVE
};

/*
 * The biquad filter's resolution ladder, each rung about root two times the
 * last, so a step is a factor of two in area.  `TranslateByrDns` runs
 * `lower_bound` over it with the shorter axis of the block's input.
 */
static const u32 becore_byr_dns_ladder[] = {
	486, 686, 972, 1374, 1944, 2748, 3888, 5498,
};

static_assert(ARRAY_SIZE(becore_byr_dns_ladder) ==
	      BECORE_RGBP_DNS_BIQUAD_MAX + 1);

/*
 * The literals GetDefaultDmsc(DmscRgbpOutput&) writes once the tuning path has
 * run, so they are the same whatever the scene was.  Generated from that
 * function beside the block's encode table, all 23 of them.
 */
static const struct becore_regval becore_dmsc_defaults[] = {
	BECORE_DMSC_DEFAULTS
};

static_assert(ARRAY_SIZE(becore_dmsc_defaults) == BECORE_DMSC_DEFAULT_COUNT);

int becore_rgbp_dmsc_value(u32 offset, u32 *value)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(becore_dmsc_defaults); i++) {
		if (becore_dmsc_defaults[i].offset != offset)
			continue;
		*value = becore_dmsc_defaults[i].value;
		return 0;
	}

	return -EINVAL;
}

/* One of the block's tuning registers, or NULL if it is not one. */
static const struct becore_sharpen_reg *becore_sharpen_lookup(u32 reg)
{
	u32 low = 0, high = ARRAY_SIZE(becore_sharpen_regs);
	u32 offset;

	if (reg < BECORE_YUVP_PHYS_BASE)
		return NULL;
	offset = reg - BECORE_YUVP_PHYS_BASE;
	if (offset > U16_MAX)
		return NULL;

	while (low < high) {
		u32 middle = low + (high - low) / 2;

		if (becore_sharpen_regs[middle].offset < offset)
			low = middle + 1;
		else
			high = middle;
	}

	if (low == ARRAY_SIZE(becore_sharpen_regs) ||
	    becore_sharpen_regs[low].offset != offset)
		return NULL;

	return &becore_sharpen_regs[low];
}

/*
 * The sharpener's tuning, encoded.
 *
 * The table is generated from the vendor's own translator, so this walks it
 * rather than knowing anything: field to bits, at the fixed point the UAPI
 * names.  A negative value is deposited two's complement in its own width,
 * which is the encode's one asymmetry -- 38 of the 344 fields are signed and
 * the vendor's clamp is what says which.
 *
 * `params` is what userspace most recently sent, or NULL for the driver's own
 * default -- which is not a table of values but the *absence* of them: every
 * field zero, so a register comes out as exactly the constant bits the
 * translator deposits whatever the tuning says.  Those words are then
 * irrelevant, because the block's bypass is asserted at the same time; what
 * makes them worth generating rather than replaying is that a program has to
 * carry a value for every word it writes, and a captured one would be some
 * Android frame's tuning frozen into the kernel.
 *
 * A value past the field saturates rather than being refused, which is what
 * `TranslateYuvSharpEnhancer` does and is not a shortcut: the phone's own
 * shipped tuning holds 2.0 for `noise_gain_lut[0]` at ordinary gains, which is
 * 256 at Q7 in an eight-bit field.  Refusing the buffer for that would mean
 * every caller had to carry a copy of the table below to send the calibration
 * the phone came with -- and where those field widths live is exactly what
 * this interface is for.
 */
int becore_sharpen_value(const struct exynos_becore_params_sharpen *params,
			 u32 reg, u32 *value)
{
	const struct becore_sharpen_reg *entry = becore_sharpen_lookup(reg);
	u32 word;
	u32 i;

	if (!entry)
		return -ENOENT;

	/* The driver's own default: no field contributes, so the constant is it. */
	if (!params) {
		*value = entry->constant;
		return 0;
	}

	word = entry->constant;
	for (i = 0; i < entry->count; i++) {
		const struct becore_sharpen_field *field =
			&becore_sharpen_fields[entry->first + i];
		s32 raw;

		raw = *(const __s32 *)((const u8 *)params + field->offset);
		raw = clamp(raw, field->min, field->max);
		word |= ((u32)raw & (BIT(field->width) - 1)) << field->shift;
	}

	*value = word;

	return 0;
}

/* One of the noise reducer's tuning registers, or NULL if it is not one. */
static const struct becore_yuvnr_reg *becore_yuvnr_lookup(u32 reg)
{
	u32 low = 0, high = ARRAY_SIZE(becore_yuvnr_regs);
	u16 offset;

	if (reg < BECORE_YUVP_PHYS_BASE)
		return NULL;
	if (reg - BECORE_YUVP_PHYS_BASE > U16_MAX)
		return NULL;
	offset = reg - BECORE_YUVP_PHYS_BASE;

	while (low < high) {
		u32 mid = low + (high - low) / 2;

		if (becore_yuvnr_regs[mid].offset < offset)
			low = mid + 1;
		else
			high = mid;
	}

	if (low == ARRAY_SIZE(becore_yuvnr_regs) ||
	    becore_yuvnr_regs[low].offset != offset)
		return NULL;

	return &becore_yuvnr_regs[low];
}

/*
 * Turning the low-frequency noise reducer off turns three other enables off
 * with it.  That is the vendor's own translators rather than a choice made
 * here, and the list is generated by measuring them -- so this reproduces a
 * gate rather than inventing one.  It is a gate and not a mode: nothing moves,
 * three bits stop being written.
 */
/*
 * Whether the block runs at all, and whether its low-frequency stage does.
 *
 * **The two take different truth tests**, and that is the register's doing
 * rather than a choice here. @enable is deposited *inverted* into `bypass` and
 * becore_yuvnr_value() inverts before it clamps, so any non-zero value --
 * negative included -- clears the bypass and runs the block. @lfnr_enable is a
 * plain field clamped to one bit, so a negative value reads as off there.
 * Anything that disagrees with these two writes a combination the vendor's
 * translators cannot produce.
 *
 * The low-frequency stage is `enable && lfnr_enable` and not @lfnr_enable
 * alone: `TranslateYuvNrCommon` clears **both** bits of `yuv_yuv_nr_top`
 * whenever @enable is zero, before any of the seventeen helpers runs, and
 * every downstream gate reads that register bit rather than the tuning field.
 */
static bool becore_yuvnr_running(const struct exynos_becore_params_yuvnr *params)
{
	return params->enable != 0;
}

static bool becore_yuvnr_lfnr(const struct exynos_becore_params_yuvnr *params)
{
	return becore_yuvnr_running(params) && params->lfnr_enable > 0;
}

/*
 * Which members a stage takes with it when it goes off.  Both lists are
 * generated: the generator runs the translators again with the enable clear
 * and sees which fields stop being written, so neither can drift from them.
 */
static bool becore_yuvnr_gated(const struct exynos_becore_params_yuvnr *params,
			       u32 offset)
{
	static const u32 enable_gated[] = BECORE_YUVNR_ENABLE_GATED;
	static const u32 lfnr_gated[] = BECORE_YUVNR_LFNR_GATED;
	u32 i;

	if (!becore_yuvnr_running(params))
		for (i = 0; i < ARRAY_SIZE(enable_gated); i++)
			if (enable_gated[i] == offset)
				return true;

	if (!becore_yuvnr_lfnr(params))
		for (i = 0; i < ARRAY_SIZE(lfnr_gated); i++)
			if (lfnr_gated[i] == offset)
				return true;

	return false;
}

/*
 * The noise reducer's tuning, encoded.
 *
 * The same walk as the sharpener's, over a table generated from the vendor's
 * own nineteen translators: field to bits, at the fixed point the UAPI names.
 * A value past the field saturates, which is what those translators do.
 *
 * One field is deposited inverted -- the block's `bypass`, which is @enable
 * the other way round -- and the flag on it is what says so, because a table
 * that needed the driver to know which register that was would be a table with
 * a special case in the reader.
 *
 * `params` is what userspace most recently sent.  There is no NULL case: with
 * no buffer in force the caller passes becore_yuvnr_off, so the default goes
 * through this same arithmetic rather than through a second copy of it.
 */
/*
 * The gain curve's last interval, which the hardware wants instead of its last
 * knot.
 *
 * `luma_gain_y` is 32 knots and the block carries 32 of them, but the register
 * table holds 31 and then the *width* of the interval between the last two,
 * with its sign in a bit of `luma_gain_config`.  That is `GAMMARGB`'s knot-axis
 * convention applied to a value axis, and it is why nothing reads
 * @luma_gain_y[31] through the generated table: no single member is the value
 * the register wants.
 *
 * The vendor takes the difference in floating point and converts; here both
 * knots have already been converted, so the subtraction is exact.
 *
 * Every shipped tuning this phone has ends its gain curve flat, so the width
 * is zero in all 441 captured programs and this reproduces them by computing
 * it rather than by assuming it.  What is carried here is the formula; the
 * value it takes today is the one a constant would also have given.
 */
#define BECORE_YUVNR_LUMA_GAIN_CONFIG	0x351c
#define BECORE_YUVNR_LUMA_GAIN_LAST	0x35b0

static void becore_yuvnr_last_interval(const struct exynos_becore_params_yuvnr *params,
				       u16 offset, u32 *word)
{
	s32 last = params->luma_gain_y[EXYNOS_BECORE_YUVNR_LUMA_GAIN_POINTS - 1];
	s32 previous = params->luma_gain_y[EXYNOS_BECORE_YUVNR_LUMA_GAIN_POINTS - 2];
	/*
	 * Both knots are userspace's and unbounded, so the difference is taken
	 * wide: at 32 bits `INT_MAX - -1` wraps to `INT_MIN`, whose `abs()` is
	 * itself, and the widest possible interval would then encode as no
	 * interval at all rather than saturating.
	 */
	s64 width = (s64)last - previous;

	if (offset == BECORE_YUVNR_LUMA_GAIN_LAST) {
		*word |= (u32)clamp(width < 0 ? -width : width, 0, 255) << 16;
		return;
	}

	/*
	 * The sign, and only where the curve is enabled at all -- the vendor
	 * writes this bit inside the same test that writes bit 0.
	 */
	if (width < 0 && (*word & BIT(0)))
		*word |= BIT(1);
}

/*
 * The temporal filter's gain curve, whose domain and slopes are both derived.
 *
 * `SetTnrLut` builds six register knots out of @mcfp_gain_lut_x's five and one
 * input.  That input is only ever read here, `GetDefaultYuvNr` builds its
 * default with four floats of 1.0, and the knot it gives is 64 in all 441
 * captured programs on three cameras and every geometry -- so the first knot
 * is stated rather than carried through the interface.
 *
 * Each of the other five is capped at a ceiling of its own and then raised to
 * its predecessor plus one if it does not exceed it.  The vendor tests the
 * *raw* value against the predecessor rather than the capped one, which cannot
 * differ: a ceiling is always above the knot before it.
 *
 * The sixth has no ceiling in the vendor's code -- it is *masked* into its
 * seven-bit field, so a knot past 127 wraps below its predecessor and the
 * curve stops rising.  Clamping instead is the rule this driver already takes
 * for the noise curve's knots, and it is what makes the rise unconditional:
 * the slopes below divide by the gaps, and with the rise guaranteed there is
 * nothing for a buffer to make degenerate and no check to put in front of
 * them.  No shipped tuning reaches the clamp -- every one of them ends this
 * curve at 127.
 */
#define BECORE_YUVNR_TNR_FIRST_KNOT	64
#define BECORE_YUVNR_TNR_CEILING(knot)	(122 + (knot))
#define BECORE_YUVNR_TNR_KNOT_MAX	127

static_assert(EXYNOS_BECORE_YUVNR_MCFP_LUT_X_POINTS ==
	      BECORE_YUVNR_TNR_KNOTS - 1);
/* Two knots to a register, so an odd count would drop the last one silently. */
static_assert(BECORE_YUVNR_TNR_KNOTS % 2 == 0);

static void becore_yuvnr_tnr_knots(const struct exynos_becore_params_yuvnr *params,
				   s32 knots[BECORE_YUVNR_TNR_KNOTS])
{
	u32 knot;

	knots[0] = BECORE_YUVNR_TNR_FIRST_KNOT;
	for (knot = 1; knot < BECORE_YUVNR_TNR_KNOTS; knot++) {
		s32 ceiling = knot < BECORE_YUVNR_TNR_KNOTS - 1 ?
			      BECORE_YUVNR_TNR_CEILING(knot) :
			      BECORE_YUVNR_TNR_KNOT_MAX;
		s32 sent = clamp(params->mcfp_gain_lut_x[knot - 1], 0, ceiling);

		knots[knot] = max(sent, knots[knot - 1] + 1);
	}
}

/*
 * Two seven-bit fields per register, low half first.  This wins over the field
 * table rather than adding to it, the way the slopes do.
 */
int becore_yuvnr_tnr_lut_x(const struct exynos_becore_params_yuvnr *params,
			   u32 reg, u32 *value)
{
	s32 knots[BECORE_YUVNR_TNR_KNOTS];
	u32 offset;
	u32 index;

	if (reg < BECORE_YUVP_PHYS_BASE)
		return -ENOENT;
	offset = reg - BECORE_YUVP_PHYS_BASE;
	if (offset < BECORE_YUVNR_TNR_KNOT_FIRST ||
	    (offset - BECORE_YUVNR_TNR_KNOT_FIRST) % 4)
		return -ENOENT;
	index = (offset - BECORE_YUVNR_TNR_KNOT_FIRST) / 4;
	if (index >= BECORE_YUVNR_TNR_KNOTS / 2)
		return -ENOENT;

	becore_yuvnr_tnr_knots(params, knots);
	*value = (u32)knots[index * 2] |
		 ((u32)knots[index * 2 + 1] << 16);

	return 0;
}

/*
 * Its slopes have the same standing: they describe the curve rather than
 * carrying values of their own, so a block that moved @mcfp_gain_lut_y or the
 * domain above while these stayed frozen would write segments that contradict
 * it.  `SetTnrSlope`'s quotient truncates toward zero, which is what C
 * division does.
 */
int becore_yuvnr_tnr_slope(const struct exynos_becore_params_yuvnr *params,
			   u32 reg, u32 *value)
{
	s32 knots[BECORE_YUVNR_TNR_KNOTS];
	s32 high, low;
	u32 offset;
	u32 index;

	if (reg < BECORE_YUVP_PHYS_BASE)
		return -ENOENT;
	offset = reg - BECORE_YUVP_PHYS_BASE;
	if (offset < BECORE_YUVNR_TNR_SLOPE_FIRST ||
	    (offset - BECORE_YUVNR_TNR_SLOPE_FIRST) % 4)
		return -ENOENT;
	index = (offset - BECORE_YUVNR_TNR_SLOPE_FIRST) / 4;
	if (index + 1 >= BECORE_YUVNR_TNR_KNOTS)
		return -ENOENT;

	becore_yuvnr_tnr_knots(params, knots);
	low = clamp(params->mcfp_gain_lut_y[index], 0,
		    BECORE_YUVNR_TNR_GAIN_MAX);
	high = clamp(params->mcfp_gain_lut_y[index + 1], 0,
		     BECORE_YUVNR_TNR_GAIN_MAX);
	*value = (u32)((high - low) / (knots[index + 1] - knots[index])) &
		 BECORE_YUVNR_TNR_SLOPE_MASK;

	return 0;
}

/*
 * The bits the low-frequency stage's own branch deposits.
 *
 * `TranslateYuvNrCommon` writes three words from a two-armed branch, and the
 * arm it takes is the one where @enable and @lfnr_enable are **both** set -- so
 * each of these bits stands for the two of them together and cannot be a field
 * of either. The generated table strips them out of those registers'
 * constants, because a constant there would be the enabled arm's answer and
 * every captured program has both enables on: nothing in the corpus could tell
 * the two apart, which is exactly why the branch had to be read.
 */
static void
becore_yuvnr_guarded_bits(const struct exynos_becore_params_yuvnr *params,
			  u32 offset, u32 *word)
{
	static const struct becore_yuvnr_guarded guarded[] =
		BECORE_YUVNR_GUARDED;
	bool off = !becore_yuvnr_lfnr(params);
	size_t i;

	for (i = 0; i < ARRAY_SIZE(guarded); i++) {
		if (guarded[i].offset != offset)
			continue;
		if (off == guarded[i].when_off)
			*word |= guarded[i].mask;
		return;
	}
}

int becore_yuvnr_value(const struct exynos_becore_params_yuvnr *params,
		       u32 reg, u32 *value)
{
	const struct becore_yuvnr_reg *entry = becore_yuvnr_lookup(reg);
	u32 word;
	u32 i;

	if (!entry)
		return -ENOENT;

	word = entry->constant;
	for (i = 0; i < entry->count; i++) {
		const struct becore_yuvnr_field *field =
			&becore_yuvnr_fields[entry->first + i];
		s32 raw;

		if (becore_yuvnr_gated(params, field->offset))
			raw = 0;
		else
			raw = *(const __s32 *)((const u8 *)params + field->offset);

		if (field->flags & BECORE_YUVNR_FIELD_INVERT)
			raw = raw ? 0 : 1;

		raw = clamp(raw, field->min, field->max);
		word |= ((u32)raw & (BIT(field->width) - 1)) << field->shift;
	}

	if (entry->offset == BECORE_YUVNR_LUMA_GAIN_LAST ||
	    entry->offset == BECORE_YUVNR_LUMA_GAIN_CONFIG)
		becore_yuvnr_last_interval(params, entry->offset, &word);

	becore_yuvnr_guarded_bits(params, entry->offset, &word);

	*value = word;

	return 0;
}

/*
 * What this block writes from outside the generated field table: bits ORed
 * into a register the table also writes, and whole registers the table does
 * not write at all.  Naming them is what puts them under the overlap check
 * below -- a regeneration that added a field where one of these deposits
 * lands would otherwise corrupt the word with nothing to say so.
 */
static const struct becore_regval becore_yuvnr_derived_bits[] = {
	{ BECORE_YUVNR_LUMA_GAIN_LAST, GENMASK(23, 16) },
	{ BECORE_YUVNR_LUMA_GAIN_CONFIG, BIT(1) },
};

/*
 * The same checks the sharpener's table gets, for the same reasons: the field
 * range bounds becore_yuvnr_fields[], the member offset bounds the read out of
 * the caller's block, and no two fields may claim the same bits or the loop
 * above could not OR.
 */
int becore_yuvnr_table_validate(struct device *dev)
{
	static const struct becore_yuvnr_guarded guarded[] =
		BECORE_YUVNR_GUARDED;
	u32 i, j;

	for (i = 0; i < ARRAY_SIZE(becore_yuvnr_regs); i++) {
		const struct becore_yuvnr_reg *entry = &becore_yuvnr_regs[i];
		u32 used = entry->constant;

		if (i && becore_yuvnr_regs[i - 1].offset >= entry->offset)
			return dev_err_probe(dev, -EINVAL,
					     "noise reducer register +%#06x is out of order\n",
					     entry->offset);
		if (entry->first + entry->count > ARRAY_SIZE(becore_yuvnr_fields))
			return dev_err_probe(dev, -EINVAL,
					     "noise reducer register +%#06x runs off the field table\n",
					     entry->offset);
		for (j = 0; j < entry->count; j++) {
			const struct becore_yuvnr_field *field =
				&becore_yuvnr_fields[entry->first + j];
			u32 mask;

			if (!field->width || field->width >= 32 ||
			    field->shift + field->width > 32)
				return dev_err_probe(dev, -EINVAL,
						     "noise reducer +%#06x field %u does not fit\n",
						     entry->offset, j);
			/*
			 * Widened, because `(s32)BIT(31)` is `INT_MIN` and
			 * would make this test pass for anything.  A 31-bit
			 * field is legal and the table has no reason never to
			 * grow one.
			 */
			if (field->max >= (s64)BIT_ULL(field->width) ||
			    field->min < -(s64)BIT_ULL(field->width - 1))
				return dev_err_probe(dev, -EINVAL,
						     "noise reducer +%#06x field %u has limits wider than itself\n",
						     entry->offset, j);
			if (field->offset % sizeof(__s32) ||
			    field->offset + sizeof(__s32) >
			    sizeof(struct exynos_becore_params_yuvnr))
				return dev_err_probe(dev, -EINVAL,
						     "noise reducer +%#06x field %u is outside the block\n",
						     entry->offset, j);
			mask = (BIT(field->width) - 1) << field->shift;
			if (used & mask)
				return dev_err_probe(dev, -EINVAL,
						     "noise reducer +%#06x field %u overlaps\n",
						     entry->offset, j);
			used |= mask;
		}

		for (j = 0; j < ARRAY_SIZE(becore_yuvnr_derived_bits); j++) {
			const struct becore_regval *derived =
				&becore_yuvnr_derived_bits[j];

			if (derived->offset != entry->offset)
				continue;
			if (used & derived->value)
				return dev_err_probe(dev, -EINVAL,
						     "noise reducer +%#06x collides with what it derives\n",
						     entry->offset);
		}

		/*
		 * The same for the branch's own bits: a field over one of them
		 * would mean the generator had decoded the branch after all,
		 * and the two answers would fight in the register.
		 */
		for (j = 0; j < ARRAY_SIZE(guarded); j++) {
			if (guarded[j].offset != entry->offset)
				continue;
			if (used & guarded[j].mask)
				return dev_err_probe(dev, -EINVAL,
						     "noise reducer +%#06x collides with its own branch\n",
						     entry->offset);
		}
	}

	/*
	 * And each guarded deposit needs a register in the table to land in,
	 * for the same reason the derived bits do: becore_yuvnr_value() only
	 * reaches them for a register it found.  It must also stay clear of
	 * what the derived bits claim -- the two depositors are checked
	 * against the field table but not against each other, and the order
	 * makes that worse in one direction: becore_yuvnr_last_interval()
	 * runs first and *reads* bit 0 of `luma_gain_config`, so a branch bit
	 * landing there would change an answer rather than only OR into it.
	 */
	for (i = 0; i < ARRAY_SIZE(guarded); i++) {
		if (!becore_yuvnr_lookup(BECORE_YUVP_PHYS_BASE +
					 guarded[i].offset))
			return dev_err_probe(dev, -EINVAL,
					     "noise reducer +%#06x deposits a branch bit into a register it does not write\n",
					     guarded[i].offset);
		for (j = 0; j < ARRAY_SIZE(becore_yuvnr_derived_bits); j++) {
			if (becore_yuvnr_derived_bits[j].offset !=
			    guarded[i].offset)
				continue;
			if (becore_yuvnr_derived_bits[j].value & guarded[i].mask)
				return dev_err_probe(dev, -EINVAL,
						     "noise reducer +%#06x has a branch bit over what it derives\n",
						     guarded[i].offset);
		}
	}

	/*
	 * Each derived deposit needs a register in the table to land in --
	 * `becore_yuvnr_value()` only reaches them for a register it found --
	 * and the temporal slopes need the opposite, because they replace a
	 * word rather than adding to one and the table would win over them.
	 */
	for (i = 0; i < ARRAY_SIZE(becore_yuvnr_derived_bits); i++) {
		u32 reg = BECORE_YUVP_PHYS_BASE +
			  becore_yuvnr_derived_bits[i].offset;

		if (!becore_yuvnr_lookup(reg))
			return dev_err_probe(dev, -EINVAL,
					     "noise reducer +%#06x derives bits into a register it does not write\n",
					     becore_yuvnr_derived_bits[i].offset);
	}
	for (i = 0; i < BECORE_YUVNR_TNR_KNOTS - 1; i++) {
		u32 reg = BECORE_YUVP_PHYS_BASE +
			  BECORE_YUVNR_TNR_SLOPE_FIRST + i * 4;

		if (becore_yuvnr_lookup(reg))
			return dev_err_probe(dev, -EINVAL,
					     "the temporal gain curve's slope %u is in the field table\n",
					     i);
	}
	for (i = 0; i < BECORE_YUVNR_TNR_KNOTS / 2; i++) {
		u32 reg = BECORE_YUVP_PHYS_BASE +
			  BECORE_YUVNR_TNR_KNOT_FIRST + i * 4;

		if (becore_yuvnr_lookup(reg))
			return dev_err_probe(dev, -EINVAL,
					     "the temporal gain curve's knots +%#06x are in the field table\n",
					     BECORE_YUVNR_TNR_KNOT_FIRST + i * 4);
	}

	/*
	 * The noise curve's slopes and shifts replace a word the same way, and
	 * are derived by `becore_noise_value()` rather than here -- so a
	 * regeneration that decoded one of them would clobber a searched shift
	 * with a field deposit, silently, because the field table wins.
	 */
	for (i = 0; i < ARRAY_SIZE(becore_noise_curves); i++) {
		const struct becore_noise_curve *curve = &becore_noise_curves[i];
		u32 reg;

		if (curve->block != BECORE_YUVP)
			continue;
		for (reg = curve->slope_first;
		     reg <= curve->slope_first + BECORE_NOISE_TABLE_LAST;
		     reg += 4)
			if (becore_yuvnr_lookup(reg))
				return dev_err_probe(dev, -EINVAL,
						     "a noise curve's slope +%#06x is in the field table\n",
						     reg - BECORE_YUVP_PHYS_BASE);
		if (becore_yuvnr_lookup(curve->shift_reg))
			return dev_err_probe(dev, -EINVAL,
					     "a noise curve's shift +%#06x is in the field table\n",
					     curve->shift_reg -
					     BECORE_YUVP_PHYS_BASE);
	}

	return 0;
}

/* One of the Bayer denoiser's tuning registers, or NULL if it is not one. */
static const struct becore_byr_dns_reg *becore_byr_dns_lookup(u32 reg)
{
	u32 low = 0, high = ARRAY_SIZE(becore_byr_dns_regs);
	u32 offset;

	if (reg < BECORE_RGBP_PHYS_BASE)
		return NULL;
	offset = reg - BECORE_RGBP_PHYS_BASE;
	if (offset > U16_MAX)
		return NULL;

	while (low < high) {
		u32 middle = low + (high - low) / 2;

		if (becore_byr_dns_regs[middle].offset < offset)
			low = middle + 1;
		else
			high = middle;
	}

	if (low == ARRAY_SIZE(becore_byr_dns_regs) ||
	    becore_byr_dns_regs[low].offset != offset)
		return NULL;

	return &becore_byr_dns_regs[low];
}

/*
 * Which white balance a frame is encoded against.
 *
 * The frame's own, whenever there is a frame: a slot carries the gains it was
 * taken through, so a moving AWB cannot scale one frame's noise by another
 * frame's gains.  The stream's seed stands in for the one caller that holds no
 * frame -- the record walk STREAMON runs to prove every word encodes -- where
 * it decides nothing, because that walk only asks whether a value exists.
 */
static struct exynos_becore_input_gains
becore_frame_gains(const struct becore_device *becore)
{
	if (becore->run_input)
		return becore->run_input->gains;

	return becore_stream_gains(becore);
}

/*
 * The three noise factors are the tuning times the white balance in front of
 * the block, which is the whole reason this block reads the AWB at all: photon
 * noise scales with the per-channel gain applied to it.  So the value that
 * travels in a parameters buffer is the tuning alone and this is where the
 * gain meets it -- Q12 against Q12, times the sixteen the field is expressed
 * at, rounded to nearest as `TranslateByrDns` does.
 *
 * Green's gain is the mean of the two green gains and not unity, which is what
 * the vendor's translator takes: `tuning * (wb[1] + wb[2]) * 0.5 * 16.0`
 * against `tuning * wb[0] * 16.0` for red.  Unity is right only where the AWB
 * normalises green -- true of every captured program, and not a property of
 * the block.  The mean is folded into the same rounding step rather than taken
 * first, so that the one rounding here is the one rounding the vendor has:
 * @sum is twice the gain and the shift is one bit longer to match.
 */
static s32 becore_byr_dns_balanced(const struct exynos_becore_input_gains *gains,
				   const struct becore_byr_dns_field *field,
				   s32 raw)
{
	u64 sum;
	u64 product;

	if (raw < 0)
		return 0;
	if (field->flags & BECORE_BYR_DNS_FIELD_RED)
		sum = 2ULL * gains->red;
	else if (field->flags & BECORE_BYR_DNS_FIELD_BLUE)
		sum = 2ULL * gains->blue;
	else
		sum = (u64)gains->green_red + gains->green_blue;

	product = (u64)raw * sum * 16 + (1ULL << 24);

	return (s32)min(product >> 25, (u64)S32_MAX);
}

/*
 * The Bayer denoiser's tuning, encoded.
 *
 * The table is generated from the vendor's own translator, so this walks it
 * rather than knowing anything: field to bits, at the fixed point the UAPI
 * names.  `params` is what userspace most recently sent, or NULL for the
 * driver's own default -- which is the absence of values rather than a table
 * of them, and because the block's `enable` reaches its register inverted,
 * that absence is the denoiser bypassed.
 *
 * A value past the field saturates rather than being refused, as it does for
 * the sharpener and for the same reason: the vendor's own translator clamps,
 * and the shipped tuning reaches the clamp.  Four fields are the exception in
 * the vendor's direction rather than ours -- `g_lpf`, `rb_lpf` and the two
 * `min_snr_pix_number_*` pairs are *masked* there rather than clamped, so an
 * out-of-range value wraps for the vendor and saturates here.  The generated
 * table's limits for those rows are the field's width, not a clamp the
 * translator performs.
 */
int becore_byrdns_value(const struct becore_device *becore,
			const struct exynos_becore_params_byr_dns *params,
			u32 reg, u32 *value)
{
	const struct becore_byr_dns_reg *entry = becore_byr_dns_lookup(reg);
	struct exynos_becore_input_gains gains;
	u32 word;
	u32 i;

	if (!entry)
		return -ENOENT;

	gains = becore_frame_gains(becore);
	word = entry->constant;
	for (i = 0; i < entry->count; i++) {
		const struct becore_byr_dns_field *field =
			&becore_byr_dns_fields[entry->first + i];
		s32 raw = 0;

		if (params)
			raw = *(const __s32 *)((const u8 *)params +
					       field->offset);
		if (field->flags & (BECORE_BYR_DNS_FIELD_RED |
				    BECORE_BYR_DNS_FIELD_GREEN |
				    BECORE_BYR_DNS_FIELD_BLUE))
			raw = becore_byr_dns_balanced(&gains, field, raw);
		if (field->flags & BECORE_BYR_DNS_FIELD_INVERT)
			raw = raw ? 0 : 1;

		raw = clamp(raw, field->min, field->max);
		word |= ((u32)raw & (BIT(field->width) - 1)) << field->shift;
	}

	*value = word;

	return 0;
}

/*
 * BIQUAD_SCALE_SHIFT_ADDER, which is a resolution octave rather than a number.
 *
 * `TranslateByrDns` takes `lower_bound` over a root-two ladder indexed by the
 * shorter axis of the block's input, caps the rung at seven, subtracts the
 * tuning's own offset and clamps what is left into the field's three bits --
 * truncating, where every value it *deposits* rounds.  Each rung is a factor
 * of two in area, so this is what tells the filter how much of the picture one
 * of its taps covers.
 *
 * It is the one word of this block that is neither tuning nor invariant: the
 * census finds four distinct values -- rungs 3 to 6 -- over the 574 captured
 * RGBP programs, and the captured 6 is right at eight of the eighteen captured
 * readouts and wrong at the other ten.
 */
int
becore_byr_dns_biquad_value(const struct becore_raster *array,
			    const struct exynos_becore_params_byr_dns *params,
			    u32 *value)
{
	s32 subtracter = params ? params->biquad_scale_shift_subtracter : 0;
	u32 shorter = min(array->width, array->height);
	s32 octaves;
	u32 rung;

	for (rung = 0; rung < ARRAY_SIZE(becore_byr_dns_ladder); rung++)
		if (becore_byr_dns_ladder[rung] >= shorter)
			break;
	rung = min_t(u32, rung, BECORE_RGBP_DNS_BIQUAD_MAX);

	octaves = (s32)(rung * BECORE_RGBP_DNS_BIQUAD_Q) - subtracter;
	octaves = clamp(octaves, 0,
			BECORE_RGBP_DNS_BIQUAD_MAX * BECORE_RGBP_DNS_BIQUAD_Q);
	*value = (u32)octaves / BECORE_RGBP_DNS_BIQUAD_Q;

	return 0;
}

/* One of the demosaic's tuning registers, or NULL if it is not one. */
static const struct becore_dmsc_reg *becore_dmsc_lookup(u32 reg)
{
	u32 low = 0, high = ARRAY_SIZE(becore_dmsc_regs);
	u32 offset;

	if (reg < BECORE_RGBP_PHYS_BASE)
		return NULL;
	offset = reg - BECORE_RGBP_PHYS_BASE;
	if (offset > U16_MAX)
		return NULL;

	while (low < high) {
		u32 middle = low + (high - low) / 2;

		if (becore_dmsc_regs[middle].offset < offset)
			low = middle + 1;
		else
			high = middle;
	}

	if (low == ARRAY_SIZE(becore_dmsc_regs) ||
	    becore_dmsc_regs[low].offset != offset)
		return NULL;

	return &becore_dmsc_regs[low];
}

/*
 * The demosaic's tuning, encoded.
 *
 * The Bayer denoiser's shape with nothing added: no white balance, no
 * geometry, no derivation -- every value in this block is a clamp and a
 * deposit, which is why this walks the generated table and knows nothing else.
 *
 * `params` is what userspace most recently sent, or becore_dmsc_neutral for
 * the driver's own default.
 *
 * That default is *not* the all-zero parameter set the sharpener's and the
 * Bayer denoiser's are, and the difference was measured rather than argued.
 * ADR 0009 asks for identity or bypass and says which one a block means is a
 * claim to settle; this block means neither.  Bypassing it leaves ten times
 * the Nyquist energy the running block does -- the Bayer mosaic itself, dot
 * for dot -- and the picture comes out entirely grayscale, because this is the
 * block that makes RGB from a mosaic and nothing downstream can do it instead.
 * What *is* neutral is the block running with every directional weight,
 * sharpening gain and desaturation limit at zero: a plain interpolation, with
 * colour, no mosaic, and less grain than the vendor's tuning, which restores
 * detail deliberately.  So the default is one field set and the rest zero
 * (research/data/camera-dmsc-default-2026-08-24/README.md).
 */
const struct exynos_becore_params_dmsc becore_dmsc_neutral = {
	.enable = 1,
};

int becore_dmsc_value(const struct exynos_becore_params_dmsc *params,
		      u32 reg, u32 *value)
{
	const struct becore_dmsc_reg *entry = becore_dmsc_lookup(reg);
	u32 word;
	u32 i;

	if (!entry)
		return -ENOENT;

	word = entry->constant;
	for (i = 0; i < entry->count; i++) {
		const struct becore_dmsc_field *field =
			&becore_dmsc_fields[entry->first + i];
		s32 raw = *(const __s32 *)((const u8 *)params + field->offset);

		if (field->flags & BECORE_DMSC_FIELD_INVERT)
			raw = raw ? 0 : 1;

		raw = clamp(raw, field->min, field->max);
		word |= ((u32)raw & (BIT(field->width) - 1)) << field->shift;
	}

	*value = word;

	return 0;
}

/*
 * The same checks the other generated tables get, plus one this block needs on
 * its own: its literals and its tuning come out of the same 43-register
 * context, so a register may be in one list or the other and never in both.
 */
int becore_dmsc_table_validate(struct device *dev)
{
	u32 i, j;

	for (i = 0; i < ARRAY_SIZE(becore_dmsc_regs); i++) {
		const struct becore_dmsc_reg *entry = &becore_dmsc_regs[i];
		u32 used = entry->constant;

		if (i && becore_dmsc_regs[i - 1].offset >= entry->offset)
			return dev_err_probe(dev, -EINVAL,
					     "demosaic register +%#06x is out of order\n",
					     entry->offset);
		if (entry->first + entry->count > ARRAY_SIZE(becore_dmsc_fields))
			return dev_err_probe(dev, -EINVAL,
					     "demosaic register +%#06x runs off the field table\n",
					     entry->offset);
		for (j = 0; j < entry->count; j++) {
			const struct becore_dmsc_field *field =
				&becore_dmsc_fields[entry->first + j];
			u32 mask;

			if (!field->width || field->width >= 32 ||
			    field->shift + field->width > 32)
				return dev_err_probe(dev, -EINVAL,
						     "demosaic +%#06x field %u does not fit\n",
						     entry->offset, j);
			if (field->max >= (s64)BIT_ULL(field->width) ||
			    field->min < -(s64)BIT_ULL(field->width - 1))
				return dev_err_probe(dev, -EINVAL,
						     "demosaic +%#06x field %u has limits wider than itself\n",
						     entry->offset, j);
			if (field->offset % sizeof(__s32) ||
			    field->offset + sizeof(__s32) >
			    sizeof(struct exynos_becore_params_dmsc))
				return dev_err_probe(dev, -EINVAL,
						     "demosaic +%#06x field %u is outside the block\n",
						     entry->offset, j);
			mask = (BIT(field->width) - 1) << field->shift;
			if (used & mask)
				return dev_err_probe(dev, -EINVAL,
						     "demosaic +%#06x field %u overlaps\n",
						     entry->offset, j);
			used |= mask;
		}
	}

	for (i = 0; i < ARRAY_SIZE(becore_dmsc_defaults); i++) {
		u32 reg = BECORE_RGBP_DMSC_BASE + becore_dmsc_defaults[i].offset;

		if (becore_dmsc_lookup(reg))
			return dev_err_probe(dev, -EINVAL,
					     "demosaic +%#06x is both a literal and a field\n",
					     becore_dmsc_defaults[i].offset);
	}

	return 0;
}

/*
 * The same checks the other two generated tables get, for the same reasons:
 * the field range bounds becore_byr_dns_fields[], the member offset bounds the
 * read out of the caller's block, and no two fields may claim the same bits or
 * the loop above could not OR.  The last one is this block's own: the noise
 * curves' slopes and shifts are derived by becore_noise_value() and replace a
 * word rather than adding to one, so a regeneration that decoded a slope would
 * clobber a searched shift with a field deposit and nothing would say so.
 */
int becore_byr_dns_table_validate(struct device *dev)
{
	u32 i, j;

	for (i = 0; i < ARRAY_SIZE(becore_byr_dns_regs); i++) {
		const struct becore_byr_dns_reg *entry = &becore_byr_dns_regs[i];
		u32 used = entry->constant;

		if (i && becore_byr_dns_regs[i - 1].offset >= entry->offset)
			return dev_err_probe(dev, -EINVAL,
					     "Bayer denoiser register +%#06x is out of order\n",
					     entry->offset);
		if (entry->first + entry->count >
		    ARRAY_SIZE(becore_byr_dns_fields))
			return dev_err_probe(dev, -EINVAL,
					     "Bayer denoiser register +%#06x runs off the field table\n",
					     entry->offset);
		for (j = 0; j < entry->count; j++) {
			const struct becore_byr_dns_field *field =
				&becore_byr_dns_fields[entry->first + j];
			u32 mask;

			if (!field->width || field->width >= 32 ||
			    field->shift + field->width > 32)
				return dev_err_probe(dev, -EINVAL,
						     "Bayer denoiser +%#06x field %u does not fit\n",
						     entry->offset, j);
			if (field->max >= (s64)BIT_ULL(field->width) ||
			    field->min < -(s64)BIT_ULL(field->width - 1))
				return dev_err_probe(dev, -EINVAL,
						     "Bayer denoiser +%#06x field %u has limits wider than itself\n",
						     entry->offset, j);
			if (field->offset % sizeof(__s32) ||
			    field->offset + sizeof(__s32) >
			    sizeof(struct exynos_becore_params_byr_dns))
				return dev_err_probe(dev, -EINVAL,
						     "Bayer denoiser +%#06x field %u is outside the block\n",
						     entry->offset, j);
			mask = (BIT(field->width) - 1) << field->shift;
			if (used & mask)
				return dev_err_probe(dev, -EINVAL,
						     "Bayer denoiser +%#06x field %u overlaps\n",
						     entry->offset, j);
			used |= mask;
		}
	}

	for (i = 0; i < ARRAY_SIZE(becore_noise_curves); i++) {
		const struct becore_noise_curve *curve = &becore_noise_curves[i];
		u32 reg;

		if (curve->block != BECORE_RGBP)
			continue;
		for (reg = curve->slope_first;
		     reg <= curve->slope_first + BECORE_NOISE_TABLE_LAST;
		     reg += 4)
			if (becore_byr_dns_lookup(reg))
				return dev_err_probe(dev, -EINVAL,
						     "a noise curve's slope +%#06x is in the field table\n",
						     reg - BECORE_RGBP_PHYS_BASE);
		if (becore_byr_dns_lookup(curve->shift_reg))
			return dev_err_probe(dev, -EINVAL,
					     "a noise curve's shift +%#06x is in the field table\n",
					     curve->shift_reg -
					     BECORE_RGBP_PHYS_BASE);
	}
	if (becore_byr_dns_lookup(BECORE_RGBP_DNS_BIQUAD_REG))
		return dev_err_probe(dev, -EINVAL,
				     "the biquad scale shift is both derived and a field\n");

	return 0;
}

/*
 * The generated table, checked once at probe, because a regeneration is
 * exactly when it could start being wrong.
 *
 * Two of these are load-bearing rather than tidy, and trimming them would put
 * an out-of-bounds read back into becore_sharpen_value(): the field range
 * bounds `becore_sharpen_fields[]`, and the member offset bounds the read out
 * of the caller's block.  The rest say what the encode assumes -- that no two
 * fields claim the same bits, so the loop can OR rather than read-modify-write,
 * and that each field's own limits fit the width it is deposited in.
 */
int becore_sharpen_table_validate(struct device *dev)
{
	u32 i, j;

	for (i = 0; i < ARRAY_SIZE(becore_sharpen_regs); i++) {
		const struct becore_sharpen_reg *entry = &becore_sharpen_regs[i];
		u32 used = entry->constant;

		if (i && becore_sharpen_regs[i - 1].offset >= entry->offset)
			return dev_err_probe(dev, -EINVAL,
					     "sharpener register +%#06x is out of order\n",
					     entry->offset);
		if (entry->first + entry->count >
		    ARRAY_SIZE(becore_sharpen_fields))
			return dev_err_probe(dev, -EINVAL,
					     "sharpener register +%#06x runs off the field table\n",
					     entry->offset);
		for (j = 0; j < entry->count; j++) {
			const struct becore_sharpen_field *field =
				&becore_sharpen_fields[entry->first + j];
			u32 mask;

			/*
			 * A 32-bit field would be a mask this cannot form --
			 * BIT(32) is undefined where a long is 32 bits -- and
			 * would need no mask in any case.
			 */
			if (!field->width || field->width >= 32 ||
			    field->shift + field->width > 32)
				return dev_err_probe(dev, -EINVAL,
						     "sharpener +%#06x field %u does not fit\n",
						     entry->offset, j);
			/* Widened for the same reason as the noise reducer's. */
			if (field->max >= (s64)BIT_ULL(field->width) ||
			    field->min < -(s64)BIT_ULL(field->width - 1))
				return dev_err_probe(dev, -EINVAL,
						     "sharpener +%#06x field %u has limits wider than itself\n",
						     entry->offset, j);
			if (field->offset % sizeof(__s32) ||
			    field->offset + sizeof(__s32) >
			    sizeof(struct exynos_becore_params_sharpen))
				return dev_err_probe(dev, -EINVAL,
						     "sharpener +%#06x field %u is outside the block\n",
						     entry->offset, j);
			mask = (BIT(field->width) - 1) << field->shift;
			if (used & mask)
				return dev_err_probe(dev, -EINVAL,
						     "sharpener +%#06x field %u overlaps\n",
						     entry->offset, j);
			used |= mask;
		}
	}

	return 0;
}

/*
 * One register of a parameters block, or -ENOENT if none of them reaches it.
 *
 * The blocks name values, not registers, so this is where the two meet: a
 * matrix coefficient is a 16-bit two's complement value in the low half of its
 * own word -- which is how every captured program writes one, negatives
 * included -- and the tone curve packs two samples per word with the
 * lower-numbered one low.  Anything
 * userspace never sent keeps whatever the recipe or the driver already put
 * there, which is what makes a parameters buffer additive rather than a
 * wholesale replacement of the program.
 */
int becore_params_value(const struct becore_device *becore,
			u32 reg, u32 *value)
{
	const struct becore_params_state *params = &becore->params;
	u32 index;

	if (params->ccm_valid &&
	    reg >= BECORE_YUVP_CCM_MATRIX_FIRST &&
	    reg <= BECORE_YUVP_CCM_MATRIX_LAST) {
		index = (reg - BECORE_YUVP_CCM_MATRIX_FIRST) / 4;
		*value = (u16)params->ccm[index];
		return 0;
	}
	if (params->ccm_valid &&
	    reg >= BECORE_YUVP_CCM_OFFSET_FIRST &&
	    reg <= BECORE_YUVP_CCM_OFFSET_LAST) {
		index = (reg - BECORE_YUVP_CCM_OFFSET_FIRST) / 4;
		*value = (u16)params->ccm_offsets[index];
		return 0;
	}
	if (params->ltm_curve_valid &&
	    reg >= BECORE_YUVP_LTM_GMAP_FIRST &&
	    reg <= BECORE_YUVP_LTM_GMAP_LAST) {
		index = (reg - BECORE_YUVP_LTM_GMAP_FIRST) / 4 *
			BECORE_LTM_CURVE_PER_REG;
		*value = params->ltm_curve[index] |
			 ((u32)params->ltm_curve[index + 1] << 16);
		return 0;
	}
	if (params->ltm_tone_adjust_valid &&
	    reg >= BECORE_YUVP_LTM_TONEADJ_FIRST &&
	    reg <= BECORE_YUVP_LTM_TONEADJ_LAST) {
		index = (reg - BECORE_YUVP_LTM_TONEADJ_FIRST) / 4 *
			BECORE_LTM_TONEADJ_PER_REG;
		/*
		 * An odd sample count, so the last register's high half has no
		 * sample to carry and stays zero -- as the driver's own default
		 * leaves it.
		 */
		*value = params->ltm_tone_adjust[index];
		if (index + 1 < EXYNOS_BECORE_LTM_TONE_ADJUST_POINTS)
			*value |= (u32)params->ltm_tone_adjust[index + 1] << 16;
		return 0;
	}
	/*
	 * No range test in front of this one: the table it walks is the only
	 * statement of where those registers are, and a second copy of their
	 * extent here would be a copy to keep in step.
	 */
	if (params->gamma_valid &&
	    !becore_yuvp_gamma_curve_value(params, reg, value))
		return 0;
	/*
	 * The colour LUT's lattice is not a register and is emitted separately,
	 * but the gate in front of it is one: the driver's own default asserts
	 * BYPASS because there is no lattice, and a block that brings one
	 * clears it.  The two cannot disagree -- the same flag decides both.
	 */
	if (params->clut_valid && reg == BECORE_YUVP_CLUT_BYPASS_REG) {
		*value = 0;
		return 0;
	}
	/*
	 * The sharpener the same way round: the driver's own default asserts
	 * the block's bypass because it has no tuning, and a block that brings
	 * tuning clears it.  Its 145 tuning words are generated from the same
	 * table underneath, so what a block changes is the values in them
	 * rather than whether they are written.
	 */
	if (params->sharpen_valid) {
		if (reg == BECORE_YUVP_SHARPEN_BYPASS_REG) {
			*value = 0;
			return 0;
		}
		if (!becore_sharpen_value(&params->sharpen, reg, value))
			return 0;
	}
	/*
	 * The noise reducer differs from both of those in one way that is
	 * worth stating: its bypass is a *field* of the block rather than a
	 * register beside it, so there is nothing to clear here.  A block that
	 * arrives with @enable zero bypasses the stage through the same table
	 * every other value goes through.
	 */
	if (params->yuvnr_valid) {
		if (!becore_yuvnr_value(&params->yuvnr, reg, value))
			return 0;
		if (!becore_yuvnr_tnr_lut_x(&params->yuvnr, reg, value))
			return 0;
		if (!becore_yuvnr_tnr_slope(&params->yuvnr, reg, value))
			return 0;
	}
	/*
	 * The Bayer denoiser is the noise reducer's shape with one addition:
	 * its scale shift is a *derivation* the tuning only offsets, so the
	 * block has a value that reaches a register the field table does not
	 * carry -- and it needs the geometry as well as the buffer, which is
	 * why this function takes the device.
	 */
	if (params->dmsc_valid && !becore_dmsc_value(&params->dmsc, reg, value))
		return 0;
	if (params->byr_dns_valid) {
		if (!becore_byrdns_value(becore, &params->byr_dns, reg, value))
			return 0;
		if (reg == BECORE_RGBP_DNS_BIQUAD_REG)
			return becore_byr_dns_biquad_value(&becore->array,
							   &params->byr_dns,
							   value);
	}

	return -ENOENT;
}

/* ---------------------------------------------------------------------------
 * The parameters node
 *
 * Under ADR 0009 the kernel owns the hardware description and the register
 * encoding; the per-frame image-quality *values* come from userspace through a
 * V4L2_BUF_TYPE_META_OUTPUT node, one typed block per hardware block.  Two of
 * them carry live policy -- the colour matrix is the white balance's own
 * output, the guide curve is the exposure estimate's, and both move frame to
 * frame in a moving scene -- and the fourth carries a calibration, which is a
 * different argument for the same boundary: the tone curve does not move at
 * all, and what makes it userspace's is that it is Google's image-quality work
 * rather than a description of the hardware.
 *
 * A buffer never carries a register, an address or a command -- only values,
 * in the units the block is specified in.
 */

static const struct v4l2_isp_params_block_type_info
becore_params_block_info[] = {
	[EXYNOS_BECORE_PARAM_BLOCK_CCM] = {
		.size = sizeof(struct exynos_becore_params_ccm),
	},
	[EXYNOS_BECORE_PARAM_BLOCK_LTM_CURVE] = {
		.size = sizeof(struct exynos_becore_params_ltm_curve),
	},
	[EXYNOS_BECORE_PARAM_BLOCK_CLUT] = {
		.size = sizeof(struct exynos_becore_params_clut),
	},
	[EXYNOS_BECORE_PARAM_BLOCK_GAMMA] = {
		.size = sizeof(struct exynos_becore_params_gamma),
	},
	[EXYNOS_BECORE_PARAM_BLOCK_SHARPEN] = {
		.size = sizeof(struct exynos_becore_params_sharpen),
	},
	[EXYNOS_BECORE_PARAM_BLOCK_YUVNR] = {
		.size = sizeof(struct exynos_becore_params_yuvnr),
	},
	[EXYNOS_BECORE_PARAM_BLOCK_BYR_DNS] = {
		.size = sizeof(struct exynos_becore_params_byr_dns),
	},
	[EXYNOS_BECORE_PARAM_BLOCK_DMSC] = {
		.size = sizeof(struct exynos_becore_params_dmsc),
	},
	[EXYNOS_BECORE_PARAM_BLOCK_LTM_TONE_ADJUST] = {
		.size = sizeof(struct exynos_becore_params_ltm_tone_adjust),
	},
};

static_assert(ARRAY_SIZE(becore_params_block_info) ==
	      EXYNOS_BECORE_PARAM_BLOCK_SENTINEL);

#define BECORE_PARAMS_BUFFER_SIZE \
	v4l2_isp_params_buffer_size(EXYNOS_BECORE_PARAMS_MAX_SIZE)

/*
 * Each row of the matrix has to sum to unity, which is what makes it preserve
 * neutrals; the vendor's own encoder guarantees it by renormalising the third
 * coefficient of each row.  A matrix that tints grey is far more likely to be
 * an arithmetic mistake upstream than an intention, so refuse it rather than
 * program it -- and refusing at buf_prepare tells userspace which buffer was
 * wrong, where a silently accepted one shows up as a colour cast three
 * abstraction layers away.
 */
static int becore_params_check_ccm(struct device *dev,
				   const struct exynos_becore_params_ccm *ccm)
{
	unsigned int row;

	for (row = 0; row < EXYNOS_BECORE_CCM_COEFFICIENTS / 3; row++) {
		s32 sum = ccm->matrix[row * 3] + ccm->matrix[row * 3 + 1] +
			  ccm->matrix[row * 3 + 2];

		if (sum != EXYNOS_BECORE_CCM_ONE) {
			dev_dbg(dev, "CCM row %u sums to %d, not %d\n", row,
				sum, EXYNOS_BECORE_CCM_ONE);
			return -EINVAL;
		}
	}

	return 0;
}

/*
 * A tone curve that goes backwards inverts contrast over that interval, which
 * no tone mapper wants and which is what a sign or ordering error looks like.
 */
static int
becore_params_check_ltm_curve(struct device *dev,
			      const struct exynos_becore_params_ltm_curve *ltm)
{
	unsigned int i;

	for (i = 0; i < EXYNOS_BECORE_LTM_CURVE_POINTS; i++) {
		if (ltm->curve[i] > EXYNOS_BECORE_LTM_CURVE_ONE) {
			dev_dbg(dev, "tone curve point %u exceeds unity\n", i);
			return -EINVAL;
		}
		if (i && ltm->curve[i] < ltm->curve[i - 1]) {
			dev_dbg(dev, "tone curve decreases at point %u\n", i);
			return -EINVAL;
		}
	}

	return 0;
}

/*
 * The same rule, in units the block's other curve does not share: this one is
 * Q14 where the guide curve is Q15, and both are 16-bit fields, so a curve
 * sent in the wrong one would fit and be twice as steep.
 */
static int
becore_params_check_ltm_tone_adjust(struct device *dev,
				    const struct exynos_becore_params_ltm_tone_adjust *ltm)
{
	unsigned int i;

	for (i = 0; i < EXYNOS_BECORE_LTM_TONE_ADJUST_POINTS; i++) {
		if (ltm->curve[i] > EXYNOS_BECORE_LTM_TONE_ADJUST_ONE) {
			dev_dbg(dev, "tone adjustment point %u exceeds unity\n",
				i);
			return -EINVAL;
		}
		if (i && ltm->curve[i] < ltm->curve[i - 1]) {
			dev_dbg(dev, "tone adjustment decreases at point %u\n",
				i);
			return -EINVAL;
		}
	}

	return 0;
}

/*
 * The lattice entries are the chroma the block outputs rather than an offset
 * to it, so there is no fill that leaves the picture alone and nothing here
 * can check that a table is *sensible*.  Two things it can check:
 *
 * The range, because the hardware field is ten bits and three of them share a
 * word -- a sample that did not fit would corrupt a neighbouring node rather
 * than itself.
 *
 * And the two ends of the grey axis.  Black and white have no hue, every
 * lattice the vendor ships is exactly neutral at both, and a lattice that is
 * not is what a stream written one sample out of step looks like.  The
 * interior of the diagonal is deliberately not checked: the vendor's own
 * tables are a count or two off neutral in the mid-greys, which is a tuning
 * choice rather than a mistake.
 */
static int becore_params_check_clut(struct device *dev,
				    const struct exynos_becore_params_clut *clut)
{
	static const unsigned int grey[] = { 0, EXYNOS_BECORE_CLUT_NODES - 1 };
	unsigned int i;

	for (i = 0; i < EXYNOS_BECORE_CLUT_NODES; i++) {
		if (clut->lut_u[i] > EXYNOS_BECORE_CLUT_MAX ||
		    clut->lut_v[i] > EXYNOS_BECORE_CLUT_MAX) {
			dev_dbg(dev, "colour LUT node %u exceeds %u\n", i,
				EXYNOS_BECORE_CLUT_MAX);
			return -EINVAL;
		}
	}

	for (i = 0; i < ARRAY_SIZE(grey); i++) {
		if (clut->lut_u[grey[i]] != EXYNOS_BECORE_CLUT_NEUTRAL ||
		    clut->lut_v[grey[i]] != EXYNOS_BECORE_CLUT_NEUTRAL) {
			dev_dbg(dev, "colour LUT node %u is not neutral\n",
				grey[i]);
			return -EINVAL;
		}
	}

	return 0;
}

/*
 * A tone curve that goes backwards is not just a strange picture: the block
 * stores its last knot as a distance from the one before it and puts the
 * direction in a _DELTA_SIGN register that this driver never writes, so a
 * falling curve would be encoded as a rising one.  The range is the other
 * half of the same encoding -- the field is fourteen bits, which is exactly
 * why the last knot is a distance -- so only that knot may reach unity.
 */
static int
becore_params_check_gamma(struct device *dev,
			  const struct exynos_becore_params_gamma *gamma)
{
	unsigned int channel;
	unsigned int i;

	for (channel = 0; channel < EXYNOS_BECORE_GAMMA_CHANNELS; channel++) {
		const __u16 *curve = gamma->curve[channel];

		u32 last = EXYNOS_BECORE_GAMMA_POINTS - 1;

		for (i = 0; i < EXYNOS_BECORE_GAMMA_POINTS; i++) {
			u32 limit = i == last ? EXYNOS_BECORE_GAMMA_ONE :
				    EXYNOS_BECORE_GAMMA_MAX;

			if (curve[i] > limit) {
				dev_dbg(dev,
					"gamma channel %u point %u exceeds %u\n",
					channel, i, limit);
				return -EINVAL;
			}
			if (i && curve[i] < curve[i - 1]) {
				dev_dbg(dev,
					"gamma channel %u decreases at point %u\n",
					channel, i);
				return -EINVAL;
			}
		}

		/*
		 * The last knot reaches the hardware as a distance rather than
		 * as a value, so it is the distance that has to fit the field:
		 * a curve sitting at zero and jumping to unity in its final
		 * segment satisfies every check above and would encode one bit
		 * past it.  The only curves this refuses are ones that do
		 * almost all of their rise between the last two knots.
		 */
		if (curve[last] - curve[last - 1] > EXYNOS_BECORE_GAMMA_MAX) {
			dev_dbg(dev,
				"gamma channel %u rises by %u in its last segment\n",
				channel, curve[last] - curve[last - 1]);
			return -EINVAL;
		}
	}

	return 0;
}

/*
 * The one thing this block cannot saturate its way out of.
 *
 * Every other value is clamped into its field and the worst a silly one can do
 * is look wrong, but the noise curve's domain is *divided by*: the driver takes
 * each segment's slope over the gap between two knots, and a gap of zero has no
 * slope.  The check is on the clamped knots rather than the sent ones, because
 * the clamp is what reaches the hardware -- two knots far past the field's top
 * arrive as the same knot.
 *
 * The temporal gain curve's domain is divided by too and needs no check of its
 * own: the vendor's own fix-up raises each knot to its predecessor plus one,
 * and clamping the last into its field rather than masking it is what makes
 * that rise unconditional.  See becore_yuvnr_tnr_knots().
 */
int __must_check
becore_params_check_yuvnr(struct device *dev,
			  const struct exynos_becore_params_yuvnr *yuvnr)
{
	unsigned int i;

	if (yuvnr->reserved) {
		dev_dbg(dev, "the noise reducer's reserved word is not zero\n");
		return -EINVAL;
	}

	for (i = 1; i < EXYNOS_BECORE_YUVNR_STD_LUT_POINTS; i++) {
		s32 previous = clamp(yuvnr->std_lut_x[i - 1], 0,
				     BECORE_NOISE_KNOT_MAX);
		s32 knot = clamp(yuvnr->std_lut_x[i], 0, BECORE_NOISE_KNOT_MAX);

		if (knot <= previous) {
			dev_dbg(dev,
				"noise curve domain does not rise at knot %u\n",
				i);
			return -EINVAL;
		}
	}

	return 0;
}

/*
 * Walk the blocks once.  `apply` distinguishes the buf_prepare pass, which
 * only says whether the buffer is acceptable, from the per-frame pass, which
 * installs it -- so that the two cannot drift apart into a buffer that
 * validates and then programs something else.
 */
static int becore_params_walk(struct becore_device *becore,
			      const struct v4l2_isp_params_buffer *config,
			      bool apply)
{
	size_t offset = 0;

	while (offset < config->data_size) {
		const struct v4l2_isp_params_block_header *header =
			(const void *)(config->data + offset);
		bool disable = header->flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE;
		int ret;

		offset += header->size;
		switch (header->type) {
		case EXYNOS_BECORE_PARAM_BLOCK_CCM: {
			const struct exynos_becore_params_ccm *ccm =
				(const void *)header;

			if (disable) {
				if (apply)
					becore->params.ccm_valid = false;
				break;
			}
			ret = becore_params_check_ccm(becore->dev, ccm);
			if (ret)
				return ret;
			if (!apply)
				break;
			memcpy(becore->params.ccm, ccm->matrix,
			       sizeof(becore->params.ccm));
			memcpy(becore->params.ccm_offsets, ccm->offsets,
			       sizeof(becore->params.ccm_offsets));
			becore->params.ccm_valid = true;
			break;
		}
		case EXYNOS_BECORE_PARAM_BLOCK_LTM_CURVE: {
			const struct exynos_becore_params_ltm_curve *ltm =
				(const void *)header;

			if (disable) {
				if (apply)
					becore->params.ltm_curve_valid = false;
				break;
			}
			ret = becore_params_check_ltm_curve(becore->dev, ltm);
			if (ret)
				return ret;
			if (!apply)
				break;
			memcpy(becore->params.ltm_curve, ltm->curve,
			       sizeof(becore->params.ltm_curve));
			becore->params.ltm_curve_valid = true;
			break;
		}
		case EXYNOS_BECORE_PARAM_BLOCK_LTM_TONE_ADJUST: {
			const struct exynos_becore_params_ltm_tone_adjust *ltm =
				(const void *)header;

			if (disable) {
				if (apply)
					becore->params.ltm_tone_adjust_valid = false;
				break;
			}
			ret = becore_params_check_ltm_tone_adjust(becore->dev,
								  ltm);
			if (ret)
				return ret;
			if (!apply)
				break;
			memcpy(becore->params.ltm_tone_adjust, ltm->curve,
			       sizeof(becore->params.ltm_tone_adjust));
			becore->params.ltm_tone_adjust_valid = true;
			break;
		}
		case EXYNOS_BECORE_PARAM_BLOCK_CLUT: {
			const struct exynos_becore_params_clut *clut =
				(const void *)header;

			/*
			 * Disabling this one really does switch the stage off,
			 * because bypass is what the driver has instead of a
			 * default lattice.
			 */
			if (disable) {
				if (apply)
					becore->params.clut_valid = false;
				break;
			}
			ret = becore_params_check_clut(becore->dev, clut);
			if (ret)
				return ret;
			if (!apply)
				break;
			memcpy(becore->params.clut_u, clut->lut_u,
			       sizeof(becore->params.clut_u));
			memcpy(becore->params.clut_v, clut->lut_v,
			       sizeof(becore->params.clut_v));
			becore->params.clut_valid = true;
			break;
		}
		case EXYNOS_BECORE_PARAM_BLOCK_GAMMA: {
			const struct exynos_becore_params_gamma *gamma =
				(const void *)header;

			if (disable) {
				if (apply)
					becore->params.gamma_valid = false;
				break;
			}
			ret = becore_params_check_gamma(becore->dev, gamma);
			if (ret)
				return ret;
			if (!apply)
				break;
			memcpy(becore->params.gamma, gamma->curve,
			       sizeof(becore->params.gamma));
			becore->params.gamma_valid = true;
			break;
		}
		case EXYNOS_BECORE_PARAM_BLOCK_SHARPEN: {
			const struct exynos_becore_params_sharpen *sharpen =
				(const void *)header;

			/*
			 * Disabling this one really does switch the stage off:
			 * the block has a bypass, and that bypass is what the
			 * driver has instead of a default tuning.
			 */
			if (disable) {
				if (apply)
					becore->params.sharpen_valid = false;
				break;
			}
			if (!apply)
				break;
			becore->params.sharpen = *sharpen;
			/*
			 * The header belongs to the buffer rather than to the
			 * block, and the encode reads offsets that start past
			 * it, so keep no copy of the caller's.
			 */
			memset(&becore->params.sharpen.header, 0,
			       sizeof(becore->params.sharpen.header));
			becore->params.sharpen_valid = true;
			break;
		}
		case EXYNOS_BECORE_PARAM_BLOCK_DMSC: {
			const struct exynos_becore_params_dmsc *dmsc =
				(const void *)header;

			/*
			 * Disabling puts the stage back on the driver's own
			 * default, which for this block is the demosaic
			 * *running* with no tuning rather than bypassed.
			 * @enable inside the block is how a buffer turns it
			 * off, and doing that leaves a mosaic in the picture.
			 */
			if (disable) {
				if (apply)
					becore->params.dmsc_valid = false;
				break;
			}
			if (!apply)
				break;
			becore->params.dmsc = *dmsc;
			/*
			 * The header belongs to the buffer rather than to the
			 * block, and the encode reads offsets that start past
			 * it, so keep no copy of the caller's.
			 */
			memset(&becore->params.dmsc.header, 0,
			       sizeof(becore->params.dmsc.header));
			becore->params.dmsc_valid = true;
			break;
		}
		case EXYNOS_BECORE_PARAM_BLOCK_BYR_DNS: {
			const struct exynos_becore_params_byr_dns *byr_dns =
				(const void *)header;

			/*
			 * Disabling the block puts the stage back on the words
			 * the driver's own default writes, which is the
			 * denoiser bypassed -- not the same thing as @enable
			 * inside the block, which is how a buffer says the
			 * same in the other direction.
			 */
			if (disable) {
				if (apply)
					becore->params.byr_dns_valid = false;
				break;
			}
			if (!apply)
				break;
			becore->params.byr_dns = *byr_dns;
			/*
			 * The header belongs to the buffer rather than to the
			 * block, and the encode reads offsets that start past
			 * it, so keep no copy of the caller's.
			 */
			memset(&becore->params.byr_dns.header, 0,
			       sizeof(becore->params.byr_dns.header));
			becore->params.byr_dns_valid = true;
			break;
		}
		case EXYNOS_BECORE_PARAM_BLOCK_YUVNR: {
			const struct exynos_becore_params_yuvnr *yuvnr =
				(const void *)header;

			/*
			 * Disabling the block puts the stage back on the words
			 * the recipe carries, which is not the same as turning
			 * the noise reducer off -- that is @enable, inside the
			 * block, and a buffer is how it is set either way.
			 */
			if (disable) {
				if (apply)
					becore->params.yuvnr_valid = false;
				break;
			}
			ret = becore_params_check_yuvnr(becore->dev, yuvnr);
			if (ret)
				return ret;
			if (!apply)
				break;
			becore->params.yuvnr = *yuvnr;
			/*
			 * The header belongs to the buffer rather than to the
			 * block, and the encode reads offsets that start past
			 * it, so keep no copy of the caller's.
			 */
			memset(&becore->params.yuvnr.header, 0,
			       sizeof(becore->params.yuvnr.header));
			becore->params.yuvnr_valid = true;
			break;
		}
		default:
			/* v4l2_isp_params_validate_buffer() rejects these. */
			return -EINVAL;
		}
	}

	return 0;
}

/* Make one buffer the current configuration and hand it back. */
static void becore_params_install(struct becore_device *becore,
				  struct becore_params_buffer *buf)
{
	/*
	 * The blocks were checked at buf_prepare against this same walk, over
	 * this same kernel copy, so this cannot fail -- and if it somehow did,
	 * the frame would run on the configuration it already had.
	 */
	if (becore_params_walk(becore, buf->config, true))
		dev_warn_once(becore->dev, "a validated params buffer did not apply\n");

	/*
	 * Both are the driver's to fill, even on an output queue: vb2 zeroes
	 * the sequence at prepare and never copies the application's, and the
	 * queue asks for a monotonic timestamp.  The counter is not reset by a
	 * session, because it must never go backwards -- a per-session one
	 * does, and a consumer that has already seen a higher number has no way
	 * to tell that from a buffer arriving out of order.
	 */
	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	buf->vb.sequence = becore->params_sequence++;
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
}

/*
 * Take the oldest queued buffer, if there is one, and make it the current
 * configuration.  A frame with nothing queued keeps the configuration it
 * already had, which is what lets userspace send only what changed.
 */
void becore_params_consume(struct becore_device *becore)
{
	struct becore_params_buffer *buf;

	spin_lock_irq(&becore->queue_lock);
	buf = list_first_entry_or_null(&becore->queued_params,
				       struct becore_params_buffer, list);
	if (buf)
		list_del(&buf->list);
	spin_unlock_irq(&becore->queue_lock);

	if (buf)
		becore_params_install(becore, buf);
}

static void becore_params_return_all(struct becore_device *becore,
				     enum vb2_buffer_state state)
{
	struct becore_params_buffer *buf;
	struct becore_params_buffer *tmp;

	spin_lock_irq(&becore->queue_lock);
	list_for_each_entry_safe(buf, tmp, &becore->queued_params, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	spin_unlock_irq(&becore->queue_lock);
}

static int becore_params_queue_setup(struct vb2_queue *q, unsigned int *nbufs,
				     unsigned int *nplanes, unsigned int sizes[],
				     struct device *alloc_devs[])
{
	if (*nplanes) {
		if (*nplanes != 1 || sizes[0] < BECORE_PARAMS_BUFFER_SIZE)
			return -EINVAL;
		return 0;
	}
	*nplanes = 1;
	sizes[0] = BECORE_PARAMS_BUFFER_SIZE;

	return 0;
}

static int becore_params_buf_init(struct vb2_buffer *vb)
{
	struct becore_params_buffer *buf =
		to_becore_params_buffer(to_vb2_v4l2_buffer(vb));

	buf->config = kvzalloc(BECORE_PARAMS_BUFFER_SIZE, GFP_KERNEL);

	return buf->config ? 0 : -ENOMEM;
}

static void becore_params_buf_cleanup(struct vb2_buffer *vb)
{
	struct becore_params_buffer *buf =
		to_becore_params_buffer(to_vb2_v4l2_buffer(vb));

	kvfree(buf->config);
	buf->config = NULL;
}

static int becore_params_buf_prepare(struct vb2_buffer *vb)
{
	struct becore_device *becore = vb2_get_drv_priv(vb->vb2_queue);
	struct becore_params_buffer *buf =
		to_becore_params_buffer(to_vb2_v4l2_buffer(vb));
	const struct v4l2_isp_params_buffer *user = vb2_plane_vaddr(vb, 0);
	int ret;

	ret = v4l2_isp_params_validate_buffer_size(becore->dev, vb,
						   BECORE_PARAMS_BUFFER_SIZE);
	if (ret)
		return ret;

	/* Validate what will be programmed, not what may change underneath. */
	memcpy(buf->config, user, BECORE_PARAMS_BUFFER_SIZE);

	/*
	 * A buffer carrying no blocks changes nothing, and there is nothing in
	 * it to check -- so take it rather than insisting its payload length
	 * agree with a block list that is not there.  That also makes an
	 * all-zero buffer legal, which is what a queue's own buffers are before
	 * anyone fills them and what v4l2-compliance queues.
	 */
	if (buf->config->version != V4L2_ISP_PARAMS_VERSION_V0 &&
	    buf->config->version != V4L2_ISP_PARAMS_VERSION_V1)
		return -EINVAL;
	if (!buf->config->data_size)
		return 0;

	ret = v4l2_isp_params_validate_buffer(becore->dev, vb, buf->config,
					      becore_params_block_info,
					      ARRAY_SIZE(becore_params_block_info));
	if (ret)
		return ret;

	return becore_params_walk(becore, buf->config, false);
}

/*
 * A buffer waits for a frame only while there is a frame coming.  When the
 * capture queue is streaming, frames follow one another and holding the buffer
 * until the next one is what makes a configuration frame-accurate.  When it is
 * not, "the next frame" is whenever somebody triggers the offline loop, or
 * never -- so apply it and hand it back instead of leaving userspace waiting
 * on a frame nobody is going to run.
 *
 * From a work item rather than from here, because a buffer that comes back
 * DONE inside the QBUF that queued it is not something a caller expects: vb2
 * allows it, and v4l2-compliance warns about it and then loses track of the
 * queue.  The work runs immediately afterwards and drains whatever is waiting.
 */
void becore_params_work(struct work_struct *work)
{
	struct becore_device *becore =
		container_of(work, struct becore_device, params_work);

	for (;;) {
		struct becore_params_buffer *buf;

		mutex_lock(&becore->lock);
		/*
		 * becore_run_stage() drops this lock while it waits, so a run
		 * can be in flight here -- and installing a configuration the
		 * frame in flight has already encoded past would return the
		 * buffer as though that frame had used it.  Leave it for the
		 * run to take, or for the next schedule if it does not.
		 */
		if (becore->video_streaming || becore->running) {
			mutex_unlock(&becore->lock);
			return;
		}
		spin_lock_irq(&becore->queue_lock);
		buf = list_first_entry_or_null(&becore->queued_params,
					       struct becore_params_buffer,
					       list);
		if (buf)
			list_del(&buf->list);
		spin_unlock_irq(&becore->queue_lock);
		if (buf)
			becore_params_install(becore, buf);
		mutex_unlock(&becore->lock);
		if (!buf)
			return;
	}
}

static void becore_params_buf_queue(struct vb2_buffer *vb)
{
	struct becore_device *becore = vb2_get_drv_priv(vb->vb2_queue);
	struct becore_params_buffer *buf =
		to_becore_params_buffer(to_vb2_v4l2_buffer(vb));

	spin_lock_irq(&becore->queue_lock);
	list_add_tail(&buf->list, &becore->queued_params);
	spin_unlock_irq(&becore->queue_lock);

	if (!READ_ONCE(becore->video_streaming))
		schedule_work(&becore->params_work);
}

/*
 * The end of a parameters session is where the configuration goes back to the
 * driver's own defaults -- not the start of one, because vb2 hands over
 * buffers queued before STREAMON before it calls start_streaming, so a reset
 * there would wipe what those buffers had just installed.  A capture session
 * does not reset it either: parameters belong to the parameters node.
 */
static void becore_params_stop_streaming(struct vb2_queue *q)
{
	struct becore_device *becore = vb2_get_drv_priv(q);

	/*
	 * Under the device lock, because becore_params_consume() takes a
	 * buffer off the list and completes it later in the same hold, and it
	 * is reached from a frame rather than from this queue.  Returning
	 * buffers without the lock can therefore find the list already empty
	 * and leave one active past this function, which is the one thing vb2
	 * forbids here.  The work item is cancelled afterwards: it takes the
	 * same lock, so it cannot be cancelled from under it.
	 */
	mutex_lock(&becore->lock);
	becore_params_return_all(becore, VB2_BUF_STATE_ERROR);
	memset(&becore->params, 0, sizeof(becore->params));
	mutex_unlock(&becore->lock);
	cancel_work_sync(&becore->params_work);
}

static const struct vb2_ops becore_params_vb2_ops = {
	.queue_setup = becore_params_queue_setup,
	.buf_init = becore_params_buf_init,
	.buf_cleanup = becore_params_buf_cleanup,
	.buf_prepare = becore_params_buf_prepare,
	.buf_queue = becore_params_buf_queue,
	.stop_streaming = becore_params_stop_streaming,
};

static int becore_params_querycap(struct file *file, void *priv,
				  struct v4l2_capability *cap)
{
	strscpy(cap->driver, "exynos-becore", sizeof(cap->driver));
	strscpy(cap->card, "zumapro BE-core parameters", sizeof(cap->card));

	return 0;
}

static void becore_params_fill_fmt(struct v4l2_meta_format *meta)
{
	memset(meta, 0, sizeof(*meta));
	meta->dataformat = V4L2_META_FMT_BECORE_PARAMS;
	meta->buffersize = BECORE_PARAMS_BUFFER_SIZE;
}

static int becore_params_g_fmt(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	becore_params_fill_fmt(&f->fmt.meta);

	return 0;
}

static int becore_params_enum_fmt(struct file *file, void *priv,
				  struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;

	f->pixelformat = V4L2_META_FMT_BECORE_PARAMS;

	return 0;
}

static const struct v4l2_ioctl_ops becore_params_ioctl_ops = {
	.vidioc_querycap = becore_params_querycap,
	.vidioc_enum_fmt_meta_out = becore_params_enum_fmt,
	.vidioc_g_fmt_meta_out = becore_params_g_fmt,
	.vidioc_s_fmt_meta_out = becore_params_g_fmt,
	.vidioc_try_fmt_meta_out = becore_params_g_fmt,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct video_device becore_params_template = {
	.name = "exynos-becore parameters",
	.fops = &becore_fops,
	.ioctl_ops = &becore_params_ioctl_ops,
	.release = video_device_release_empty,
	.device_caps = V4L2_CAP_META_OUTPUT | V4L2_CAP_STREAMING,
	.vfl_dir = VFL_DIR_TX,
};

int becore_params_init(struct becore_device *becore)
{
	struct vb2_queue *q = &becore->params_queue;
	size_t type;
	int ret;

	/*
	 * v4l2_isp_params_validate_buffer() walks the blocks by their declared
	 * size, so a block type whose size is zero would never terminate.  The
	 * sizes below are all sizeof() of a real struct, and this is what keeps
	 * that true if someone adds a type and leaves the entry out.
	 */
	for (type = 0; type < ARRAY_SIZE(becore_params_block_info); type++) {
		if (becore_params_block_info[type].size <
		    sizeof(struct v4l2_isp_params_block_header))
			return dev_err_probe(becore->dev, -EINVAL,
					     "params block type %zu has no size\n",
					     type);
	}

	q->type = V4L2_BUF_TYPE_META_OUTPUT;
	q->io_modes = VB2_MMAP;
	q->dev = becore->dev;
	q->drv_priv = becore;
	q->ops = &becore_params_vb2_ops;
	q->mem_ops = &vb2_vmalloc_memops;
	q->buf_struct_size = sizeof(struct becore_params_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &becore->params_lock;
	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	/* The producer's v4l2_device is filled in when it registers. */
	becore->params_vdev = becore_params_template;
	becore->params_vdev.queue = q;
	becore->params_vdev.lock = &becore->params_lock;
	becore->params_vdev.entity.function = MEDIA_ENT_F_IO_V4L;
	video_set_drvdata(&becore->params_vdev, becore);

	becore->params_pad.flags = MEDIA_PAD_FL_SOURCE;

	return media_entity_pads_init(&becore->params_vdev.entity, 1,
				      &becore->params_pad);
}
