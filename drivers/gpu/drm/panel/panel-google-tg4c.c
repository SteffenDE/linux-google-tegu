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

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

/* MIPI DSI high-speed clock, see downstream MIPI_DSI_FREQ_MBPS_DEFAULT. */
#define TG4C_DSI_HS_CLK_MBPS	1102

struct google_tg4c {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data *supplies;
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
 * this WIP we emit the *production* path:
 *   - unconditional GS_DSI_CMD() entries are always emitted, and
 *   - for revision-gated entries we take the latest-production variant
 *     (PANEL_REV_GE(EVT1_1) / PANEL_REV_GE(DVT1) and the EVT1_1..PVT range
 *     blocks), dropping the early-silicon LT(EVT1_1) alternatives.
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

	/* DBI block (PANEL_REV_RANGE(EVT1_1, PVT)) */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xC2, 0x33, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6F, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xC2, 0x77, 0x1F, 0x02, 0x00, 0x9E, 0xB8,
				     0xCA, 0xD8, 0xE4, 0xF0, 0xFA, 0x00, 0x1D, 0x00, 0x00, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xC3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
				     0x00, 0x01, 0x01, 0x00, 0x01, 0x03, 0x00, 0x03, 0x0A, 0x00,
				     0x0A, 0x3D, 0x10, 0x3D, 0xE1, 0x41, 0xE1, 0x00, 0x74, 0x00,
				     0xEF, 0x77, 0xEF, 0xEF, 0x97, 0xEF, 0xAE, 0x00, 0x00, 0x06,
				     0x90, 0x06, 0xAB);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xC5, 0x88, 0x00, 0x00, 0x88, 0x00, 0x00,
				     0x88, 0x00, 0x00, 0x08, 0x00, 0x88, 0x00, 0x00, 0x88, 0x00,
				     0x00, 0x88, 0x00, 0x00, 0x08, 0x00, 0x88, 0x00, 0x00, 0x88,
				     0x00, 0x00, 0x88, 0x00, 0x00, 0x08, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xC6, 0xF8, 0x00, 0xFF, 0x88, 0x00, 0x00,
				     0x88, 0x00, 0x00, 0xF8, 0x00, 0xFF, 0x88, 0x00, 0x00, 0x88,
				     0x00, 0x00, 0xF8, 0x00, 0xFF, 0x88, 0x00, 0x00, 0x88, 0x00,
				     0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xC7, 0x15);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xC8, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03,
				     0x06, 0x0D, 0x1A, 0x34);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xC9, 0x00, 0x01, 0x02, 0x04, 0x08, 0x10,
				     0x1F, 0x3F, 0x7D, 0xFA);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xCA, 0x00, 0x01, 0x01, 0x02, 0x04, 0x09,
				     0x12, 0x24, 0x47, 0x8E);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xCB, 0x21, 0x38, 0x55, 0x64, 0x2B, 0x92,
				     0xCA, 0x5F, 0xE8, 0xFE, 0x59, 0x4A, 0xFF, 0x37, 0x89, 0xFE,
				     0x2D, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
				     0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xCC, 0x00, 0x60, 0xB9, 0x21, 0x56, 0x42,
				     0x43, 0x1B, 0xD8, 0x86, 0x3A, 0xAC, 0xCA, 0x91, 0x86, 0xFD,
				     0xF4, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
				     0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xCE, 0x21, 0x1B, 0x16, 0x43, 0x81, 0xD3,
				     0xA7, 0xB2, 0x73, 0xAA, 0xAE, 0xF5, 0xBB, 0xCC, 0x3D, 0xF9,
				     0xA1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
				     0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xCF, 0x88, 0x78, 0x78, 0x78, 0x78, 0x7E,
				     0x77, 0x7E, 0x01, 0x67, 0x01, 0x51, 0x66, 0x51, 0x6C, 0x46,
				     0x6C, 0xB6, 0x24, 0xB6, 0xBE, 0x22, 0xBE, 0x00, 0x22, 0x00,
				     0x2E, 0x22, 0x2E, 0x2E, 0x12, 0x2E, 0x01, 0xFF, 0xFF, 0xFF,
				     0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
				     0xFF, 0xFF, 0xFF, 0xFF);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6F, 0x32);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xCF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
				     0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
				     0x68, 0x78, 0x61, 0x16, 0x61, 0x04, 0xFF, 0xFF, 0xFF, 0xFF,
				     0xFF, 0xFF, 0x33, 0x96, 0x96, 0x33, 0x96, 0x5A, 0x23, 0x5A,
				     0x8F, 0x32, 0x8F, 0x7D, 0x33, 0x7D, 0x3A, 0x23, 0x3A, 0xAB,
				     0x22, 0xAB, 0x38, 0x22);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6F, 0x64);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xCF, 0x38, 0x00, 0x12, 0x00, 0xDB, 0x11,
				     0xDB, 0xDB, 0x11, 0xDB, 0x93, 0x33, 0x96, 0x61, 0x13, 0x61,
				     0x94, 0x88, 0x5E, 0x5E, 0x48, 0x5E, 0x63, 0x44, 0x63, 0x35,
				     0x34, 0x35, 0x17, 0x33, 0x17, 0x56, 0x13, 0x56, 0xCB, 0x11,
				     0xCB, 0x55, 0x21, 0x55, 0x00, 0x12, 0x00, 0x7B, 0x11, 0x7B,
				     0x7B, 0x11, 0x7B, 0x2E);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6F, 0x96);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xCF, 0x38, 0x5E, 0x3C, 0x13, 0x3C, 0x2F);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xD0, 0x44, 0x00, 0x00, 0x44, 0x00, 0x00,
				     0x44, 0x00, 0x00, 0x44, 0x00, 0x00, 0x44, 0x00, 0x00, 0x44,
				     0x00, 0x00, 0x44, 0x00, 0x00, 0x44, 0x00, 0x00, 0x44, 0x00,
				     0x00, 0x44, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xD2, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xD3, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xFF, 0xAA, 0x55, 0xA5, 0x84);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6F, 0x26);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xF2, 0x00);

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
	/* 60Hz */
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
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	struct drm_dsc_picture_parameter_set pps;
	int ret;

	ret = google_tg4c_enable_supplies(ctx);
	if (ret < 0)
		return ret;

	google_tg4c_reset(ctx);

	ret = google_tg4c_on(ctx);
	if (ret < 0)
		goto err;

	/* Program the DSC PPS and enable compression. */
	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
	mipi_dsi_picture_parameter_set_multi(&dsi_ctx, &pps);
	mipi_dsi_compression_mode_ext_multi(&dsi_ctx, true, MIPI_DSI_COMPRESSION_DSC, 0);
	if (dsi_ctx.accum_err < 0) {
		ret = dsi_ctx.accum_err;
		goto err;
	}

	return 0;
err:
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	google_tg4c_disable_supplies(ctx);
	return ret;
}

static int google_tg4c_disable(struct drm_panel *panel)
{
	struct google_tg4c *ctx = to_google_tg4c(panel);

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

	for (i = 0; i < ARRAY_SIZE(google_tg4c_modes); i++)
		count += drm_connector_helper_get_modes_fixed(connector,
							      &google_tg4c_modes[i]);

	return count;
}

static const struct drm_panel_funcs google_tg4c_panel_funcs = {
	.prepare = google_tg4c_prepare,
	.unprepare = google_tg4c_unprepare,
	.disable = google_tg4c_disable,
	.get_modes = google_tg4c_get_modes,
};

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

	/*
	 * TODO: the downstream driver implements brightness control
	 * (tg4c_set_brightness), LHBM/local HBM overdrive (tg4c_set_local_hbm_*),
	 * HBM (tg4c_set_hbm_mode), AOD/low-power modes (tg4c_lp_cmds, set_lp_mode),
	 * dynamic 60/120Hz switching (tg4c_change_frequency), TE2, FFC retuning
	 * and DDIC id/panel-rev read-back (tg4c_read_id / tg4c_get_panel_rev).
	 * None of that is implemented here; a backlight device and the runtime
	 * command paths still need to be wired up.
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
	},
};
module_mipi_dsi_driver(google_tg4c_driver);

MODULE_DESCRIPTION("DRM driver for Google tg4c (Pixel 9a) MIPI-DSI panel");
MODULE_LICENSE("GPL");
