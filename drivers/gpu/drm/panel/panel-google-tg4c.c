// SPDX-License-Identifier: GPL-2.0-only
/*
 * MIPI-DSI based driver for the Google "tg4c" display panel
 * (compatible "google,gs-tg4c"), as found in the Google Pixel 9a (tegu).
 *
 * All register sequences, timings, DSC parameters and supply/reset details
 * in this file are transcribed from the downstream Google panel driver
 * (private/google-modules/display/panels/tegu/panel-gs-tg4c.c, the gs_panel
 * framework). This is a WIP, upstream-style re-implementation built around the
 * standard drm_panel / mipi_dsi_multi_context APIs; it is NOT a port of the
 * downstream gs_panel framework.
 *
 * Copyright (c) 2024 Google LLC (original downstream values)
 * Copyright (c) 2026 Linden Maskat
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/kstrtox.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

/* MIPI DSI high-speed clock, see downstream MIPI_DSI_FREQ_MBPS_DEFAULT. */
#define TG4C_DSI_HS_CLK_MBPS	1102

/*
 * Brightness (DBV) range, from the downstream tg4c_btr_configs[] PVT entry:
 * normal range is DBV 1..3628 (2..1200 nits), default 1829. The HBM range
 * (3629..3939, up to 1800 nits) is gated behind a separate mode downstream
 * and is intentionally not exposed here.
 */
#define TG4C_BRIGHTNESS_MAX	3628
#define TG4C_BRIGHTNESS_DEFAULT	1829

/*
 * LHBM (local high-brightness mode): the panel drives a fixed circular region
 * -- 100 px radius, centred 540 px below the centre of the active area, i.e.
 * about (540, 1752) -- at elevated brightness, so an under-display fingerprint
 * sensor can see the finger.
 *
 * The region is driven white regardless of what is in the framebuffer beneath
 * it: enabling LHBM over an all-red screen produces a white disc, not a
 * brighter red one. Nothing has to be drawn there for it to work.
 *
 * DBV thresholds selecting the LHBM gamma set, from downstream
 * tg4c_set_local_hbm_gamma() (LHBM_GAMMASET1..3).
 */
#define TG4C_LHBM_GAMMA_SET1	3628
#define TG4C_LHBM_GAMMA_SET2	2774
#define TG4C_LHBM_GAMMA_SET3	2186

struct google_tg4c {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data *supplies;
	/*
	 * Serialises LHBM against the panel's own power transitions.
	 * @lhbm_usable tracks display-on precisely: drm_panel's ->enabled is
	 * cleared by the core only *after* ->disable returns, so it would still
	 * read true while the display-off sequence is running.
	 */
	struct mutex lhbm_lock;
	bool lhbm_usable;
	bool lhbm_on;
	unsigned int refresh_rate;
};

enum google_tg4c_supply {
	TG4C_SUPPLY_VDDI,
	TG4C_SUPPLY_VCI,
	TG4C_SUPPLY_VDDD,
};

/*
 * Supplies, transcribed from the DT panel@2 node (google,gs-tg4c):
 *   vci-supply, vddi-supply, vddd-supply.
 * Downstream reg_ctrl ordering (tg4c_reg_ctrl_desc):
 *   enable : VDDI, VCI, VDDD(+11ms)
 *   disable: VDDD, VCI, VDDI(+1ms)
 *
 * Keep this array in downstream enable order. The driver still toggles each
 * rail explicitly because regulator_bulk_enable() cannot interleave the
 * 11ms post-VDDD wait that downstream programs. Whether the panel actually
 * requires that delay is unverified; revisit once we can probe on hardware
 * and possibly drop or shorten it.
 */
static const struct regulator_bulk_data google_tg4c_supplies[] = {
	{ .supply = "vddi" },
	{ .supply = "vci" },
	{ .supply = "vddd" },
};

static inline struct google_tg4c *to_google_tg4c(struct drm_panel *panel)
{
	return container_of(panel, struct google_tg4c, panel);
}

static int google_tg4c_enable_supplies(struct google_tg4c *ctx)
{
	int ret;

	ret = regulator_enable(ctx->supplies[TG4C_SUPPLY_VDDI].consumer);
	if (ret)
		return ret;

	ret = regulator_enable(ctx->supplies[TG4C_SUPPLY_VCI].consumer);
	if (ret)
		goto disable_vddi;

	ret = regulator_enable(ctx->supplies[TG4C_SUPPLY_VDDD].consumer);
	if (ret)
		goto disable_vci;

	usleep_range(11000, 12000);

	return 0;

disable_vci:
	regulator_disable(ctx->supplies[TG4C_SUPPLY_VCI].consumer);
disable_vddi:
	regulator_disable(ctx->supplies[TG4C_SUPPLY_VDDI].consumer);
	usleep_range(1000, 2000);

	return ret;
}

static int google_tg4c_disable_supplies(struct google_tg4c *ctx)
{
	int ret;
	int first_ret = 0;

	ret = regulator_disable(ctx->supplies[TG4C_SUPPLY_VDDD].consumer);
	if (ret && !first_ret)
		first_ret = ret;

	ret = regulator_disable(ctx->supplies[TG4C_SUPPLY_VCI].consumer);
	if (ret && !first_ret)
		first_ret = ret;

	ret = regulator_disable(ctx->supplies[TG4C_SUPPLY_VDDI].consumer);
	if (ret && !first_ret)
		first_ret = ret;

	usleep_range(1000, 2000);

	return first_ret;
}

/*
 * Reset timing, transcribed from downstream .reset_timing_ms = {1, 1, 20}
 * combined with the gs_panel reset helper:
 * drive high for 1ms, low for 1ms, then high and wait 20ms for init.
 */
static void google_tg4c_reset(struct google_tg4c *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	msleep(20);
}

/*
 * Panel power-on / init sequence.
 *
 * Transcribed from downstream tg4c_init_cmds[] (DEFINE_GS_CMDSET(tg4c_init)).
 * The downstream array is heavily gated by panel silicon revision
 * (PANEL_REV_LT(EVT1_1), PANEL_REV_GE(DVT1), PANEL_REV_RANGE(...), etc). For
 * this WIP we emit the Pixel 9a bootloader-selected PVT path:
 *   - unconditional GS_DSI_CMD() entries are always emitted, and
 *   - for revision-gated entries we take the PANEL_REV_PVT filter selected by
 *     exynos_drm.panel_name=google-tg4c.2d008004 downstream.
 *
 * This includes PANEL_REV_GE(EVT1_1) and PANEL_REV_GE(DVT1), and skips
 * PANEL_REV_LT(EVT1_1), PANEL_REV_LT(PVT), and the max-exclusive
 * PANEL_REV_RANGE(EVT1_1, PVT) DBI block.
 *
 * TODO: this is a static snapshot. The downstream selects between these
 * variants at runtime from ctx->panel_rev (see tg4c_get_panel_rev() reading
 * DDIC register 0xDB). Re-introduce revision handling once panel ID read-back
 * is wired up.
 */
static int google_tg4c_on(struct google_tg4c *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	/* CMD2 Page 0x0A: BOIS_M on (PANEL_REV_GE(DVT1)) */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x0A);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xC6, 0x00, 0x03);

	/* Page Disable */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xF0, 0x55, 0xAA, 0x52, 0x00, 0x00);
	/* CMD3, Page0 */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xFF, 0xAA, 0x55, 0xA5, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6F, 0x16);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xF4, 0x02, 0x74);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6F, 0x31);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xF8, 0x01, 0x74);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6F, 0x15);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xF8, 0x01, 0x8D);

	/* CMD3, Page1 */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xFF, 0xAA, 0x55, 0xA5, 0x81);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6F, 0x0E);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xF5, 0x2A);

	/* BOIS Clk, idle vfp clk off */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6F, 0x0F);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xF5, 0x22);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5F, 0x00, 0x00);
	/* CMD3, Page2 */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xFF, 0xAA, 0x55, 0xA5, 0x82);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6F, 0x09);

	/* MIPI timing optimize (PANEL_REV_GE(EVT1_1)) */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xF2, 0x55);
	/* MIPI byte packet clk (PANEL_REV_GE(DVT1)) */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xF8, 0x0F);

	/* CMD Disable */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xFF, 0xAA, 0x55, 0xA5, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x35);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6F, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x35, 0x2D);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x55, 0x00);
	/* BC Dimming OFF */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x53, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2A, 0x00, 0x00, 0x04, 0x37);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2B, 0x00, 0x00, 0x09, 0x77);
	/* Normal GMA */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x26, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x51, 0x0E, 0x2C);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6F, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x51, 0x0F, 0xFE);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x81, 0x01, 0x19);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x88, 0x01, 0x02, 0x1C, 0x06, 0xDD, 0x00,
				     0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x90, 0x03, 0x43);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x91, 0x89, 0xA8, 0x00, 0x18, 0xC2, 0x00,
				     0x02, 0x0E, 0x02, 0x4C, 0x00, 0x07, 0x04, 0x2D, 0x04, 0x3D,
				     0x10, 0xF0);
	/* 60Hz -- see google_tg4c_set_refresh_rate() for the other value */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2F, 0x02);

	/* FFC off, then FFC setting for MIPI 1102 Mbps */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xC3, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xC3, 0x00, 0x06, 0x20, 0x11, 0xFF, 0x00,
				     0x06, 0x20, 0x11, 0xFF, 0x00, 0x05, 0xBD, 0x1F, 0x06, 0x4F,
				     0x19, 0x05, 0xBD, 0x1F, 0x06, 0x4F, 0x19, 0x05, 0xBD, 0x1F,
				     0x06, 0x4F, 0x19, 0x05, 0xBD, 0x1F, 0x06, 0x4F, 0x19, 0x05,
				     0xBD, 0x1F, 0x06, 0x4F, 0x19);

	/* Exit sleep, settle (PANEL_REV_GE(EVT1_1) uses 120ms) */
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	/*
	 * Re-issue the tear-effect configuration after display-on.  The init
	 * sequence above already sets TEAR_ON, but that happens before
	 * sleep-out, and the panel intermittently comes up with its tear output
	 * not running -- no TE at all until the next re-init, which stalls every
	 * command-mode frame behind it.  Downstream programs it a second time
	 * once the panel is up (tg4c_update_te2(), its .update_te2 op); do the
	 * same.  Parameters are downstream's defaults: scanline rising = 0, TE2
	 * width = 0x2d (45H).  0x35 takes (byte0, byte1) here exactly as the
	 * 0x6f/0x01 offset write in the init sequence does.
	 */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_TEAR_SCANLINE,
				     0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_TEAR_ON,
				     0x00, 0x2d);

	return dsi_ctx.accum_err;
}

/*
 * Panel power-off / sleep sequence.
 * Transcribed from downstream tg4c_off_cmds[]:
 *   display off (then 100ms), enter sleep (then 120ms).
 */
static int google_tg4c_off(struct google_tg4c *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 100);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

static int google_tg4c_prepare(struct drm_panel *panel)
{
	struct google_tg4c *ctx = to_google_tg4c(panel);
	int ret;

	ret = google_tg4c_enable_supplies(ctx);
	if (ret < 0)
		return ret;

	google_tg4c_reset(ctx);

	ret = google_tg4c_on(ctx);
	if (ret < 0)
		goto err;

	/* Display is on: LHBM writes are safe from here until ->disable. */
	mutex_lock(&ctx->lhbm_lock);
	ctx->lhbm_usable = true;
	/* google_tg4c_on() ends with 0x2F 0x02, and this tracks it. */
	ctx->refresh_rate = 60;
	mutex_unlock(&ctx->lhbm_lock);

	return 0;
err:
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	google_tg4c_disable_supplies(ctx);
	return ret;
}

static int google_tg4c_disable(struct drm_panel *panel)
{
	struct google_tg4c *ctx = to_google_tg4c(panel);

	/*
	 * Close the window before the display-off sequence starts, not after:
	 * a DCS write to a panel on its way down gets no reply and stalls every
	 * command-mode frame behind it. The panel forgets 0x87 along with the
	 * rest of its state, so the cached flag goes too.
	 */
	mutex_lock(&ctx->lhbm_lock);
	ctx->lhbm_usable = false;
	ctx->lhbm_on = false;
	ctx->refresh_rate = 0;
	mutex_unlock(&ctx->lhbm_lock);

	return google_tg4c_off(ctx);
}

static int google_tg4c_unprepare(struct drm_panel *panel)
{
	struct google_tg4c *ctx = to_google_tg4c(panel);

	gpiod_set_value_cansleep(ctx->reset_gpio, 0);

	return google_tg4c_disable_supplies(ctx);
}

/*
 * Display modes.
 *
 * Active area 1080x2424. Porches transcribed verbatim from the downstream
 * tg4c DRM_MODE_TIMING(): HFP/HSA/HBP = 32/12/16, VFP/VSA/VBP = 12/4/15
 *   => htotal = 1140, vtotal = 2455.
 * The 60Hz timing is "aligned to bootloader setting" downstream, so keep it
 * exact. width_mm/height_mm from downstream WIDTH_MM/HEIGHT_MM (64/145).
 */
#define TG4C_HFP	32
#define TG4C_HSA	12
#define TG4C_HBP	16
#define TG4C_VFP	12
#define TG4C_VSA	4
#define TG4C_VBP	15
#define TG4C_HTOTAL	(1080 + TG4C_HFP + TG4C_HSA + TG4C_HBP)
#define TG4C_VTOTAL	(2424 + TG4C_VFP + TG4C_VSA + TG4C_VBP)

static const struct drm_display_mode google_tg4c_modes[] = {
	{ /* 120Hz */
		.clock = TG4C_HTOTAL * TG4C_VTOTAL * 120 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + TG4C_HFP,
		.hsync_end = 1080 + TG4C_HFP + TG4C_HSA,
		.htotal = TG4C_HTOTAL,
		.vdisplay = 2424,
		.vsync_start = 2424 + TG4C_VFP,
		.vsync_end = 2424 + TG4C_VFP + TG4C_VSA,
		.vtotal = TG4C_VTOTAL,
		.width_mm = 64,
		.height_mm = 145,
		.type = DRM_MODE_TYPE_DRIVER,
	},
	{ /* 60Hz (downstream preferred / bootloader-aligned) */
		.clock = TG4C_HTOTAL * TG4C_VTOTAL * 60 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + TG4C_HFP,
		.hsync_end = 1080 + TG4C_HFP + TG4C_HSA,
		.htotal = TG4C_HTOTAL,
		.vdisplay = 2424,
		.vsync_start = 2424 + TG4C_VFP,
		.vsync_end = 2424 + TG4C_VFP + TG4C_VSA,
		.vtotal = TG4C_VTOTAL,
		.width_mm = 64,
		.height_mm = 145,
		.type = DRM_MODE_TYPE_PREFERRED | DRM_MODE_TYPE_DRIVER,
	},
};

static int google_tg4c_get_modes(struct drm_panel *panel,
				 struct drm_connector *connector)
{
	int count = 0;
	int i;

	/*
	 * Added by hand instead of via drm_connector_helper_get_modes_fixed(),
	 * which would mark every mode preferred; only the bootloader-aligned
	 * 60Hz mode carries DRM_MODE_TYPE_PREFERRED so mode sorting lists it
	 * first.
	 */
	for (i = 0; i < ARRAY_SIZE(google_tg4c_modes); i++) {
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev,
					  &google_tg4c_modes[i]);
		if (!mode)
			return count;

		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
		count++;
	}

	connector->display_info.width_mm = google_tg4c_modes[0].width_mm;
	connector->display_info.height_mm = google_tg4c_modes[0].height_mm;

	return count;
}

static const struct drm_panel_funcs google_tg4c_panel_funcs = {
	.prepare = google_tg4c_prepare,
	.unprepare = google_tg4c_unprepare,
	.disable = google_tg4c_disable,
	.get_modes = google_tg4c_get_modes,
};

/*
 * LHBM, transcribed from downstream tg4c_set_local_hbm_mode() and
 * tg4c_set_local_hbm_gamma().
 *
 * Downstream also reprograms the per-channel LHBM brightness (0xD1) on every
 * transition, picking an overdrive table from the current DBV and the reported
 * grey level of the surrounding content. That is deliberately not done here:
 * the values it starts from are the panel's own factory calibration, which
 * downstream *reads back* with a DCS read of 0xD1 at init
 * (tg4c_lhbm_brightness_init) before deriving the overdrive tables. Leaving
 * 0xD1 alone therefore leaves the panel at its calibrated brightness, which is
 * correct -- just not adaptive to the surround.
 *
 * Downstream force-disables LHBM whenever the refresh rate leaves 120 Hz
 * (tg4c_change_frequency). This driver advertises 60 Hz as preferred and does
 * no runtime switching, so a caller that wants LHBM has to have selected the
 * 120 Hz mode. Nothing here enforces that -- drm_panel is not told the current
 * mode -- so it is the caller's problem, and it is untested at 60 Hz.
 */
static int google_tg4c_set_lhbm(struct google_tg4c *ctx, bool on)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	u8 gamma_cmd[] = { 0x87, 0x00 };
	u16 br;

	/* CMD2 page 2, where 0x87 lives (as does the 0xD1 above). */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x02);

	if (!on) {
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x87, 0x00);
		return dsi_ctx.accum_err;
	}

	br = backlight_get_brightness(ctx->panel.backlight);
	if (br > TG4C_LHBM_GAMMA_SET1)
		gamma_cmd[1] = 0x01;
	else if (br > TG4C_LHBM_GAMMA_SET2)
		gamma_cmd[1] = 0x02;
	else if (br > TG4C_LHBM_GAMMA_SET3)
		gamma_cmd[1] = 0x03;
	else
		gamma_cmd[1] = 0x04;

	/*
	 * 0x6F sets the byte offset for the *next* write only, so this pair
	 * writes byte 7 of 0x87 (the gamma set) and the write after it writes
	 * byte 0 (the enable). The gamma value is not a compile-time constant,
	 * hence write_buffer rather than write_seq.
	 */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6F, 0x07);
	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, gamma_cmd, sizeof(gamma_cmd));
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x87, 0x05);

	return dsi_ctx.accum_err;
}

/*
 * The DDIC's own refresh rate, which is not the same thing as the mode DECON
 * drives. Downstream's tg4c_change_frequency() writes exactly this: 0x2F 0x00
 * for 120Hz and 0x2F 0x02 for 60Hz, and it force-disables LHBM on any switch
 * away from 120Hz -- so on this panel the two are coupled inside the DDIC.
 *
 * Our init sequence writes 0x2F 0x02, which pins the panel at 60Hz whatever
 * mode is selected, and there is no mode_set-driven switching here yet. This
 * attribute exists so that the coupling can be measured rather than assumed:
 * the fingerprint sensor's anti-spoof stage rejects every capture taken at
 * 60Hz, and whether that is because of this gate is exactly the open question.
 *
 * A DRM property driven from the mode is the right shape once the answer is
 * known. Until then this is a bring-up knob and is documented as one.
 */
static int google_tg4c_set_refresh_rate(struct google_tg4c *ctx, unsigned int hz)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	u8 cmd[] = { 0x2F, 0x02 };

	if (hz != 60 && hz != 120)
		return -EINVAL;

	/*
	 * write_buffer rather than write_seq: the payload is not a compile-time
	 * constant, and write_seq builds a static array from its arguments.
	 */
	if (hz == 120)
		cmd[1] = 0x00;
	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, cmd, sizeof(cmd));
	if (dsi_ctx.accum_err)
		return dsi_ctx.accum_err;

	ctx->refresh_rate = hz;
	return 0;
}

static ssize_t refresh_rate_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct google_tg4c *ctx = dev_get_drvdata(dev);
	unsigned int hz;

	mutex_lock(&ctx->lhbm_lock);
	hz = ctx->refresh_rate;
	mutex_unlock(&ctx->lhbm_lock);

	return sysfs_emit(buf, "%u\n", hz);
}

static ssize_t refresh_rate_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct google_tg4c *ctx = dev_get_drvdata(dev);
	unsigned int hz;
	int ret;

	ret = kstrtouint(buf, 10, &hz);
	if (ret)
		return ret;

	/*
	 * Shares lhbm_lock rather than taking one of its own: the DDIC couples
	 * the two, and downstream drops LHBM before leaving 120Hz. Serialising
	 * them here keeps that invariant expressible.
	 */
	mutex_lock(&ctx->lhbm_lock);
	if (!ctx->lhbm_usable)
		ret = -ENODEV;
	else if (ctx->lhbm_on && hz != 120)
		ret = -EBUSY;
	else
		ret = google_tg4c_set_refresh_rate(ctx, hz);
	mutex_unlock(&ctx->lhbm_lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(refresh_rate);

static ssize_t local_hbm_mode_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct google_tg4c *ctx = dev_get_drvdata(dev);
	bool on;

	mutex_lock(&ctx->lhbm_lock);
	on = ctx->lhbm_on;
	mutex_unlock(&ctx->lhbm_lock);

	return sysfs_emit(buf, "%d\n", on);
}

static ssize_t local_hbm_mode_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct google_tg4c *ctx = dev_get_drvdata(dev);
	bool on;
	int ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;

	mutex_lock(&ctx->lhbm_lock);
	if (!ctx->lhbm_usable) {
		ret = -ENODEV;
	} else if (on != ctx->lhbm_on) {
		ret = google_tg4c_set_lhbm(ctx, on);
		if (!ret)
			ctx->lhbm_on = on;
	}
	mutex_unlock(&ctx->lhbm_lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(local_hbm_mode);

static struct attribute *google_tg4c_attrs[] = {
	&dev_attr_local_hbm_mode.attr,
	&dev_attr_refresh_rate.attr,
	NULL,
};
ATTRIBUTE_GROUPS(google_tg4c);

/*
 * Brightness is a plain DCS set_display_brightness (0x51) write, transcribed
 * from downstream tg4c_set_brightness(). The value is sent MSB first (the
 * downstream "br >> 8, br & 0xff" order), hence the _large helper. The write
 * goes out in whatever LP/HS state the panel left the link in; the init
 * sequence already runs in LP mode over the adopted live link, so LP commands
 * during active scanout are the path proven on this hardware.
 */
static int google_tg4c_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);

	return mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
}

static const struct backlight_ops google_tg4c_bl_ops = {
	.update_status = google_tg4c_bl_update_status,
};

static struct backlight_device *
google_tg4c_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = TG4C_BRIGHTNESS_DEFAULT,
		.max_brightness = TG4C_BRIGHTNESS_MAX,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &google_tg4c_bl_ops, &props);
}

/* Truncate signed value to a 6-bit two's-complement field (downstream macro). */
#define TG4C_6BIT_SIGNED(v)	((v) & 0x3f)
#define TG4C_RC_RANGE(min, max, offset) \
	((struct drm_dsc_rc_range_parameters){ min, max, TG4C_6BIT_SIGNED(offset) })

/*
 * DSC configuration, transcribed verbatim from downstream tg4c_dsc_cfg
 * (struct drm_dsc_config). DSC v1.2, 2 horizontal slices of 540x24, 8bpc,
 * bits_per_pixel = 128 (i.e. 8.0 bpp in 4.4 fixed point).
 */
static void google_tg4c_dsc_init(struct drm_dsc_config *dsc)
{
	dsc->dsc_version_major = 1;
	dsc->dsc_version_minor = 2;
	dsc->line_buf_depth = 9;
	dsc->bits_per_component = 8;
	dsc->convert_rgb = true;
	dsc->slice_count = 2;
	dsc->slice_width = 540;
	dsc->slice_height = 24;
	dsc->simple_422 = false;
	dsc->pic_width = 1080;
	dsc->pic_height = 2424;
	dsc->rc_tgt_offset_high = 3;
	dsc->rc_tgt_offset_low = 3;
	dsc->bits_per_pixel = 128;
	dsc->rc_edge_factor = 6;
	dsc->rc_quant_incr_limit1 = 11;
	dsc->rc_quant_incr_limit0 = 11;
	dsc->initial_xmit_delay = 512;
	dsc->block_pred_enable = true;
	dsc->first_line_bpg_offset = 12;
	dsc->initial_offset = 6144;

	dsc->rc_buf_thresh[0] = 14;
	dsc->rc_buf_thresh[1] = 28;
	dsc->rc_buf_thresh[2] = 42;
	dsc->rc_buf_thresh[3] = 56;
	dsc->rc_buf_thresh[4] = 70;
	dsc->rc_buf_thresh[5] = 84;
	dsc->rc_buf_thresh[6] = 98;
	dsc->rc_buf_thresh[7] = 105;
	dsc->rc_buf_thresh[8] = 112;
	dsc->rc_buf_thresh[9] = 119;
	dsc->rc_buf_thresh[10] = 121;
	dsc->rc_buf_thresh[11] = 123;
	dsc->rc_buf_thresh[12] = 125;
	dsc->rc_buf_thresh[13] = 126;

	dsc->rc_range_params[0] = TG4C_RC_RANGE(0, 4, 2);
	dsc->rc_range_params[1] = TG4C_RC_RANGE(0, 4, 0);
	dsc->rc_range_params[2] = TG4C_RC_RANGE(1, 5, 0);
	dsc->rc_range_params[3] = TG4C_RC_RANGE(1, 6, -2);
	dsc->rc_range_params[4] = TG4C_RC_RANGE(3, 7, -4);
	dsc->rc_range_params[5] = TG4C_RC_RANGE(3, 7, -6);
	dsc->rc_range_params[6] = TG4C_RC_RANGE(3, 7, -8);
	dsc->rc_range_params[7] = TG4C_RC_RANGE(3, 8, -8);
	dsc->rc_range_params[8] = TG4C_RC_RANGE(3, 9, -8);
	dsc->rc_range_params[9] = TG4C_RC_RANGE(3, 10, -10);
	dsc->rc_range_params[10] = TG4C_RC_RANGE(5, 10, -10);
	dsc->rc_range_params[11] = TG4C_RC_RANGE(5, 11, -12);
	dsc->rc_range_params[12] = TG4C_RC_RANGE(5, 11, -12);
	dsc->rc_range_params[13] = TG4C_RC_RANGE(9, 12, -12);
	dsc->rc_range_params[14] = TG4C_RC_RANGE(12, 13, -12);

	dsc->rc_model_size = 8192;
	dsc->flatness_min_qp = 3;
	dsc->flatness_max_qp = 12;
	dsc->initial_scale_value = 32;
	dsc->scale_decrement_interval = 7;
	dsc->scale_increment_interval = 588;
	dsc->nfl_bpg_offset = 1069;
	dsc->slice_bpg_offset = 1085;
	dsc->final_offset = 4336;
	dsc->vbr_enable = false;
	dsc->slice_chunk_size = 540;
	dsc->native_422 = false;
	dsc->native_420 = false;
	dsc->second_line_bpg_offset = 0;
	dsc->nsl_bpg_offset = 0;
	dsc->second_line_offset_adj = 0;
}

static int google_tg4c_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct google_tg4c *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct google_tg4c, panel,
				   &google_tg4c_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ret = devm_mutex_init(dev, &ctx->lhbm_lock);
	if (ret)
		return ret;

	ret = devm_regulator_bulk_get_const(dev, ARRAY_SIZE(google_tg4c_supplies),
					    google_tg4c_supplies, &ctx->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	/*
	 * Command-mode panel with non-continuous HS clock; downstream gs_mode
	 * uses MIPI_DSI_CLOCK_NON_CONTINUOUS and the gs_panel framework runs it
	 * as a DSI command-mode (TE driven) panel.
	 */
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS;

	/* The panel must be powered/configured before the encoder enables. */
	ctx->panel.prepare_prev_first = true;

	google_tg4c_dsc_init(&ctx->dsc);
	dsi->dsc = &ctx->dsc;

	ctx->panel.backlight = google_tg4c_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	/*
	 * TODO: the downstream driver also implements LHBM *overdrive* (the
	 * 0xD1 reprogramming in tg4c_set_local_hbm_brightness; plain LHBM is
	 * implemented here), HBM (tg4c_set_hbm_mode), AOD/low-power modes
	 * (tg4c_lp_cmds, set_lp_mode), dynamic 60/120Hz switching
	 * (tg4c_change_frequency), TE2, FFC retuning and DDIC id/panel-rev
	 * read-back (tg4c_read_id / tg4c_get_panel_rev). None of that is
	 * implemented here.
	 */

	drm_panel_add(&ctx->panel);

	ret = devm_mipi_dsi_attach(dev, dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void google_tg4c_remove(struct mipi_dsi_device *dsi)
{
	struct google_tg4c *ctx = mipi_dsi_get_drvdata(dsi);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id google_tg4c_of_match[] = {
	{ .compatible = "google,gs-tg4c" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, google_tg4c_of_match);

static struct mipi_dsi_driver google_tg4c_driver = {
	.probe = google_tg4c_probe,
	.remove = google_tg4c_remove,
	.driver = {
		.name = "panel-google-tg4c",
		.of_match_table = google_tg4c_of_match,
		/*
		 * Exposes local_hbm_mode. A DRM connector property would be the
		 * upstream shape for this; sysfs is a local interface until
		 * there is one, and it keeps the name downstream userspace and
		 * the fingerprint daemon already expect.
		 */
		.dev_groups = google_tg4c_groups,
	},
};
module_mipi_dsi_driver(google_tg4c_driver);

MODULE_DESCRIPTION("DRM driver for Google tg4c (Pixel 9a) MIPI-DSI panel");
MODULE_LICENSE("GPL");
