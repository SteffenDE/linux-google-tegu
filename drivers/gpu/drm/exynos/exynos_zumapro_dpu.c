// SPDX-License-Identifier: GPL-2.0-only
/*
 * Passive Google Tensor G4 Zumapro DPU scaffold.
 *
 * This driver intentionally validates DT topology without programming display
 * registers.  The real DECON/DPP enable path needs downstream trace parity
 * before the first MMIO write is allowed.
 */

#include <drm/drm_fourcc.h>

#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "exynos_drm_drv.h"
#include "regs-zumapro-dpu.h"

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
};

struct zumapro_decon {
	struct device *dev;
	u32 id;
	u32 cgc_dma_id;
	u32 max_windows;
	int dpp_count;
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

static const struct zumapro_decon_desc *zumapro_decon_desc_by_id(u32 id)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(zumapro_decon_descs); i++)
		if (zumapro_decon_descs[i].id == id)
			return &zumapro_decon_descs[i];

	return NULL;
}

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

	decon->dpp_count = of_count_phandle_with_args(dev->of_node, "dpps",
						      NULL);
	if (decon->dpp_count == -ENOENT)
		decon->dpp_count = 0;
	else if (decon->dpp_count < 0)
		return dev_err_probe(dev, decon->dpp_count,
				     "failed to parse dpps\n");

	platform_set_drvdata(pdev, decon);
	dev_info(dev,
		 "registered passive DECON%u topology with %d DPPs; DRM bind disabled\n",
		decon->id, decon->dpp_count);

	return 0;
}

static void zumapro_decon_remove(struct platform_device *pdev)
{
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

MODULE_DESCRIPTION("Passive Google Tensor G4 Zumapro DPU scaffold");
MODULE_LICENSE("GPL");
