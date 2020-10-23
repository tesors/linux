// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 Kévin L'hôpital <kevin.lhopital@bootlin.com>
 * Copyright 2020 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#include "sun8i_a83t_dphy.h"
#include "sun8i_a83t_mipi_csi2.h"

#define MODULE_NAME	"sun8i-a83t-mipi-csi2"

/* Core */

static void sun8i_a83t_mipi_csi2_init(struct sun8i_a83t_mipi_csi2_dev *cdev)
{
	struct regmap *regmap = cdev->regmap;

	/*
	 * The Allwinner BSP sets various magic values on a bunch of registers.
	 * This is apparently a necessary initialization process that will cause
	 * the capture to fail with unsolicited interrupts hitting if skipped.
	 *
	 * Most of the registers are set to proper values later, except for the
	 * two reserved registers. They are said to hold a "hardware lock"
	 * value, without more information available.
	 */

	regmap_write(regmap, SUN8I_A83T_MIPI_CSI2_CTRL_REG, 0);
	regmap_write(regmap, SUN8I_A83T_MIPI_CSI2_CTRL_REG,
		     SUN8I_A83T_MIPI_CSI2_CTRL_INIT_VALUE);

	regmap_write(regmap, SUN8I_A83T_MIPI_CSI2_RX_PKT_NUM_REG, 0);
	regmap_write(regmap, SUN8I_A83T_MIPI_CSI2_RX_PKT_NUM_REG,
		     SUN8I_A83T_MIPI_CSI2_RX_PKT_NUM_INIT_VALUE);

	regmap_write(regmap, SUN8I_A83T_DPHY_CTRL_REG, 0);
	regmap_write(regmap, SUN8I_A83T_DPHY_CTRL_REG,
		     SUN8I_A83T_DPHY_CTRL_INIT_VALUE);

	regmap_write(regmap, SUN8I_A83T_MIPI_CSI2_RSVD1_REG, 0);
	regmap_write(regmap, SUN8I_A83T_MIPI_CSI2_RSVD1_REG,
		     SUN8I_A83T_MIPI_CSI2_RSVD1_HW_LOCK_VALUE);

	regmap_write(regmap, SUN8I_A83T_MIPI_CSI2_RSVD2_REG, 0);
	regmap_write(regmap, SUN8I_A83T_MIPI_CSI2_RSVD2_REG,
		     SUN8I_A83T_MIPI_CSI2_RSVD2_HW_LOCK_VALUE);

	regmap_write(regmap, SUN8I_A83T_MIPI_CSI2_CFG_REG, 0);
	regmap_write(regmap, SUN8I_A83T_MIPI_CSI2_CFG_REG,
		     SUN8I_A83T_MIPI_CSI2_CFG_INIT_VALUE);
}

static int sun8i_a83t_mipi_csi2_s_power(struct v4l2_subdev *subdev, int on)
{
	struct sun8i_a83t_mipi_csi2_video *video =
		sun8i_a83t_mipi_csi2_subdev_video(subdev);
	struct sun8i_a83t_mipi_csi2_dev *cdev = sun8i_a83t_mipi_csi2_video_dev(video);
	int ret;

	if (!on) {
		clk_disable_unprepare(cdev->clk_mod);
		clk_disable_unprepare(cdev->clk_mipi);
		clk_disable_unprepare(cdev->clk_misc);
		reset_control_assert(cdev->reset);

		return 0;
	}

	ret = clk_prepare_enable(cdev->clk_mod);
	if (ret) {
		dev_err(cdev->dev, "failed to enable module clock\n");
		return ret;
	}

	ret = clk_prepare_enable(cdev->clk_mipi);
	if (ret) {
		dev_err(cdev->dev, "failed to enable MIPI clock\n");
		goto error_clk_mod;
	}

	ret = clk_prepare_enable(cdev->clk_misc);
	if (ret) {
		dev_err(cdev->dev, "failed to enable CSI misc clock\n");
		goto error_clk_mipi;
	}

	ret = reset_control_deassert(cdev->reset);
	if (ret) {
		dev_err(cdev->dev, "failed to deassert reset\n");
		goto error_clk_misc;
	}

	sun8i_a83t_mipi_csi2_init(cdev);

	return 0;

error_clk_misc:
	clk_disable_unprepare(cdev->clk_misc);

error_clk_mipi:
	clk_disable_unprepare(cdev->clk_mipi);

error_clk_mod:
	clk_disable_unprepare(cdev->clk_mod);

	return ret;
}

static const struct v4l2_subdev_core_ops sun8i_a83t_mipi_csi2_subdev_core_ops = {
	.s_power	= sun8i_a83t_mipi_csi2_s_power,
};

/* Video */

static int sun8i_a83t_mipi_csi2_s_stream(struct v4l2_subdev *subdev, int on)
{
	struct sun8i_a83t_mipi_csi2_video *video =
		sun8i_a83t_mipi_csi2_subdev_video(subdev);
	struct sun8i_a83t_mipi_csi2_dev *cdev = sun8i_a83t_mipi_csi2_video_dev(video);
	struct v4l2_subdev *remote_subdev = video->remote_subdev;
	struct v4l2_fwnode_bus_mipi_csi2 *bus_mipi_csi2 =
		&video->endpoint.bus.mipi_csi2;
	union phy_configure_opts dphy_opts = { 0 };
	struct phy_configure_opts_mipi_dphy *dphy_cfg = &dphy_opts.mipi_dphy;
	struct regmap *regmap = cdev->regmap;
	struct v4l2_ctrl *ctrl;
	unsigned int lanes_count;
	unsigned int bpp;
	unsigned long pixel_rate;
	u8 data_type = 0;
	u32 version = 0;
	/* Initialize to 0 to use both in disable label (ret != 0) and off. */
	int ret = 0;

	if (!remote_subdev)
		return -ENODEV;

	if (!on) {
		v4l2_subdev_call(remote_subdev, video, s_stream, 0);

disable:
		regmap_update_bits(regmap, SUN8I_A83T_MIPI_CSI2_CFG_REG,
				   SUN8I_A83T_MIPI_CSI2_CFG_SYNC_EN, 0);

		regmap_write(regmap, SUN8I_A83T_MIPI_CSI2_CTRL_REG, 0);

		phy_power_off(cdev->dphy);

		return ret;
	}

	switch (video->mbus_code) {
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		data_type = MIPI_CSI2_DATA_TYPE_RAW8;
		bpp = 8;
		break;
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		data_type = MIPI_CSI2_DATA_TYPE_RAW10;
		bpp = 10;
		break;
	default:
		return -EINVAL;
	}

	/* Sensor pixel rate */

	ctrl = v4l2_ctrl_find(remote_subdev->ctrl_handler, V4L2_CID_PIXEL_RATE);
	if (!ctrl) {
		dev_err(cdev->dev,
			"%s: no MIPI CSI-2 pixel rate from the sensor\n",
			__func__);
		return -ENODEV;
	}

	pixel_rate = (unsigned long)v4l2_ctrl_g_ctrl_int64(ctrl);
	if (!pixel_rate) {
		dev_err(cdev->dev,
			"%s: zero MIPI CSI-2 pixel rate from the sensor\n",
			__func__);
		return -ENODEV;
	}

	/* D-PHY configuration */

	lanes_count = bus_mipi_csi2->num_data_lanes;
	phy_mipi_dphy_get_default_config(pixel_rate, bpp, lanes_count,
					 dphy_cfg);

	/*
	 * Note that our hardware is using DDR, which is not taken in account by
	 * phy_mipi_dphy_get_default_config when calculating hs_clk_rate from
	 * the pixel rate, lanes count and bpp.
	 *
	 * The resulting clock rate is basically the symbol rate over the whole
	 * link. The actual clock rate is calculated with division by two since
	 * DDR samples both on rising and falling edges.
	 */

	dev_dbg(cdev->dev, "A83T MIPI CSI-2 config:\n");
	dev_dbg(cdev->dev,
		"%ld pixels/s, %u bits/pixel, %u lanes, %lu Hz clock\n",
		pixel_rate, bpp, lanes_count, dphy_cfg->hs_clk_rate / 2);

	ret = 0;
	ret |= phy_reset(cdev->dphy);
	ret |= phy_set_mode_ext(cdev->dphy, PHY_MODE_MIPI_DPHY,
				PHY_MIPI_DPHY_SUBMODE_RX);
	ret |= phy_configure(cdev->dphy, &dphy_opts);

	if (ret) {
		dev_err(cdev->dev, "Failed to setup MIPI D-PHY\n");
		return ret;
	}

	ret = phy_power_on(cdev->dphy);
	if (ret) {
		dev_err(cdev->dev, "Failed to power on MIPI D-PHY\n");
		return ret;
	}

	/* MIPI CSI-2 controller setup */

	regmap_write(regmap, SUN8I_A83T_MIPI_CSI2_CTRL_REG,
		     SUN8I_A83T_MIPI_CSI2_CTRL_RESET_N);

	regmap_read(regmap, SUN8I_A83T_MIPI_CSI2_VERSION_REG, &version);

	dev_dbg(cdev->dev, "A83T MIPI CSI-2 version: %04x\n", version);

	regmap_write(regmap, SUN8I_A83T_MIPI_CSI2_CFG_REG,
		     SUN8I_A83T_MIPI_CSI2_CFG_UNPKT_EN |
		     SUN8I_A83T_MIPI_CSI2_CFG_SYNC_DLY_CYCLE(8) |
		     SUN8I_A83T_MIPI_CSI2_CFG_N_CHANNEL(1) |
		     SUN8I_A83T_MIPI_CSI2_CFG_N_LANE(lanes_count));

	regmap_write(regmap, SUN8I_A83T_MIPI_CSI2_VCDT0_REG,
		     SUN8I_A83T_MIPI_CSI2_VCDT0_CH_VC(3, 3) |
		     SUN8I_A83T_MIPI_CSI2_VCDT0_CH_VC(2, 2) |
		     SUN8I_A83T_MIPI_CSI2_VCDT0_CH_VC(1, 1) |
		     SUN8I_A83T_MIPI_CSI2_VCDT0_CH_VC(0, 0) |
		     SUN8I_A83T_MIPI_CSI2_VCDT0_CH_DT(0, data_type));

	/* Start streaming. */
	regmap_update_bits(regmap, SUN8I_A83T_MIPI_CSI2_CFG_REG,
			   SUN8I_A83T_MIPI_CSI2_CFG_SYNC_EN,
			   SUN8I_A83T_MIPI_CSI2_CFG_SYNC_EN);

	ret = v4l2_subdev_call(remote_subdev, video, s_stream, 1);
	if (ret)
		goto disable;

	return 0;
}

static const struct v4l2_subdev_video_ops sun8i_a83t_mipi_csi2_subdev_video_ops = {
	.s_stream	= sun8i_a83t_mipi_csi2_s_stream,
};

/* Pad */

static int sun8i_a83t_mipi_csi2_enum_mbus_code(struct v4l2_subdev *subdev,
					  struct v4l2_subdev_pad_config *config,
					  struct v4l2_subdev_mbus_code_enum *code_enum)
{
	struct sun8i_a83t_mipi_csi2_video *video =
		sun8i_a83t_mipi_csi2_subdev_video(subdev);

	if (!video->remote_subdev)
		return -ENODEV;

	/* Forward to the sensor. */
	code_enum->pad = video->remote_pad_index;

	return v4l2_subdev_call(video->remote_subdev, pad, enum_mbus_code,
				config, code_enum);
}

static int sun8i_a83t_mipi_csi2_get_fmt(struct v4l2_subdev *subdev,
				   struct v4l2_subdev_pad_config *config,
				   struct v4l2_subdev_format *format)
{
	struct sun8i_a83t_mipi_csi2_video *video =
		sun8i_a83t_mipi_csi2_subdev_video(subdev);

	if (!video->remote_subdev)
		return -ENODEV;

	/* Forward to the sensor. */
	format->pad = video->remote_pad_index;

	return v4l2_subdev_call(video->remote_subdev, pad, get_fmt, config,
				format);
}

static int sun8i_a83t_mipi_csi2_set_fmt(struct v4l2_subdev *subdev,
				   struct v4l2_subdev_pad_config *config,
				   struct v4l2_subdev_format *format)
{
	struct sun8i_a83t_mipi_csi2_video *video =
		sun8i_a83t_mipi_csi2_subdev_video(subdev);

	if (!video->remote_subdev)
		return -ENODEV;

	/* Forward to the sensor. */
	format->pad = video->remote_pad_index;

	return v4l2_subdev_call(video->remote_subdev, pad, set_fmt, config,
				format);
}

static int sun8i_a83t_mipi_csi2_enum_frame_size(struct v4l2_subdev *subdev,
					   struct v4l2_subdev_pad_config *config,
					   struct v4l2_subdev_frame_size_enum *size_enum)
{
	struct sun8i_a83t_mipi_csi2_video *video =
		sun8i_a83t_mipi_csi2_subdev_video(subdev);

	if (!video->remote_subdev)
		return -ENODEV;

	/* Forward to the sensor. */
	size_enum->pad = video->remote_pad_index;

	return v4l2_subdev_call(video->remote_subdev, pad, enum_frame_size,
				config, size_enum);
}

static int sun8i_a83t_mipi_csi2_enum_frame_interval(struct v4l2_subdev *subdev,
					       struct v4l2_subdev_pad_config *config,
					       struct v4l2_subdev_frame_interval_enum *interval_enum)
{
	struct sun8i_a83t_mipi_csi2_video *video =
		sun8i_a83t_mipi_csi2_subdev_video(subdev);

	if (!video->remote_subdev)
		return -ENODEV;

	/* Forward to the sensor. */
	interval_enum->pad = video->remote_pad_index;

	return v4l2_subdev_call(video->remote_subdev, pad, enum_frame_interval,
				config, interval_enum);
}

static const struct v4l2_subdev_pad_ops sun8i_a83t_mipi_csi2_subdev_pad_ops = {
	.enum_mbus_code		= sun8i_a83t_mipi_csi2_enum_mbus_code,
	.get_fmt		= sun8i_a83t_mipi_csi2_get_fmt,
	.set_fmt		= sun8i_a83t_mipi_csi2_set_fmt,
	.enum_frame_size	= sun8i_a83t_mipi_csi2_enum_frame_size,
	.enum_frame_interval	= sun8i_a83t_mipi_csi2_enum_frame_interval,
};

/* Subdev */

static const struct v4l2_subdev_ops sun8i_a83t_mipi_csi2_subdev_ops = {
	.core		= &sun8i_a83t_mipi_csi2_subdev_core_ops,
	.video		= &sun8i_a83t_mipi_csi2_subdev_video_ops,
	.pad		= &sun8i_a83t_mipi_csi2_subdev_pad_ops,
};

/* Notifier */

static int sun8i_a83t_mipi_csi2_notifier_bound(struct v4l2_async_notifier *notifier,
					  struct v4l2_subdev *remote_subdev,
					  struct v4l2_async_subdev *remote_subdev_async)
{
	struct v4l2_subdev *subdev = notifier->sd;
	struct sun8i_a83t_mipi_csi2_video *video =
		sun8i_a83t_mipi_csi2_subdev_video(subdev);
	struct sun8i_a83t_mipi_csi2_dev *cdev = sun8i_a83t_mipi_csi2_video_dev(video);
	int source_pad;
	int ret;

	source_pad = media_entity_get_fwnode_pad(&remote_subdev->entity,
						 remote_subdev->fwnode,
						 MEDIA_PAD_FL_SOURCE);
	if (source_pad < 0)
		return source_pad;

	ret = media_create_pad_link(&remote_subdev->entity, source_pad,
				    &subdev->entity, 0,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		dev_err(cdev->dev, "failed to create %s:%u -> %s:%u link\n",
			remote_subdev->entity.name, source_pad,
			subdev->entity.name, 0);
		return ret;
	}

	video->remote_subdev = remote_subdev;
	video->remote_pad_index = source_pad;

	return 0;
}

static const struct v4l2_async_notifier_operations sun8i_a83t_mipi_csi2_notifier_ops = {
	.bound		= sun8i_a83t_mipi_csi2_notifier_bound,
};

/* Media Entity */

static int sun8i_a83t_mipi_csi2_link_validate(struct media_link *link)
{
	struct v4l2_subdev *subdev =
		container_of(link->sink->entity, struct v4l2_subdev, entity);
	struct sun8i_a83t_mipi_csi2_video *video =
		sun8i_a83t_mipi_csi2_subdev_video(subdev);
	struct v4l2_subdev *remote_subdev;
	struct v4l2_subdev_format format = { 0 };
	int ret;

	if (!is_media_entity_v4l2_subdev(link->source->entity))
		return -EINVAL;

	remote_subdev = media_entity_to_v4l2_subdev(link->source->entity);

	format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	format.pad = link->source->index;

	ret = v4l2_subdev_call(remote_subdev, pad, get_fmt, NULL, &format);
	if (ret)
		return ret;

	video->mbus_code = format.format.code;

	return 0;
}

static const struct media_entity_operations sun8i_a83t_mipi_csi2_entity_ops = {
	.link_validate	= sun8i_a83t_mipi_csi2_link_validate,
};

/* Base Driver */

static int sun8i_a83t_mipi_csi2_v4l2_setup(struct sun8i_a83t_mipi_csi2_dev *cdev)
{
	struct sun8i_a83t_mipi_csi2_video *video = &cdev->video;
	struct v4l2_subdev *subdev = &video->subdev;
	struct v4l2_async_notifier *notifier = &video->notifier;
	struct fwnode_handle *handle;
	struct v4l2_fwnode_endpoint *endpoint;
	int ret;

	/* Subdev */

	v4l2_subdev_init(subdev, &sun8i_a83t_mipi_csi2_subdev_ops);
	subdev->dev = cdev->dev;
	strscpy(subdev->name, MODULE_NAME, sizeof(subdev->name));
	v4l2_set_subdevdata(subdev, cdev);

	/* Entity */

	subdev->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	subdev->entity.ops = &sun8i_a83t_mipi_csi2_entity_ops;

	/* Pads */

	video->pads[0].flags = MEDIA_PAD_FL_SINK;
	video->pads[1].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&subdev->entity, 2, video->pads);
	if (ret)
		return ret;

	/* Endpoint */

	handle = fwnode_graph_get_endpoint_by_id(dev_fwnode(cdev->dev), 0, 0,
						 FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!handle)
		goto error_media_entity;

	endpoint = &video->endpoint;
	endpoint->bus_type = V4L2_MBUS_CSI2_DPHY;

	ret = v4l2_fwnode_endpoint_parse(handle, endpoint);
	fwnode_handle_put(handle);
	if (ret)
		goto error_media_entity;

	/* Notifier */

	v4l2_async_notifier_init(notifier);

	ret = v4l2_async_notifier_add_fwnode_remote_subdev(notifier, handle,
							   &video->subdev_async);
	if (ret)
		goto error_media_entity;

	video->notifier.ops = &sun8i_a83t_mipi_csi2_notifier_ops;

	ret = v4l2_async_subdev_notifier_register(subdev, notifier);
	if (ret < 0)
		goto error_notifier;

	/* Subdev */

	ret = v4l2_async_register_subdev(subdev);
	if (ret < 0)
		goto error_notifier_registered;

	return 0;

error_notifier_registered:
	v4l2_async_notifier_unregister(notifier);
error_notifier:
	v4l2_async_notifier_cleanup(notifier);
error_media_entity:
	media_entity_cleanup(&subdev->entity);

	return ret;
}

static int sun8i_a83t_mipi_csi2_v4l2_teardown(struct sun8i_a83t_mipi_csi2_dev *cdev)
{
	struct sun8i_a83t_mipi_csi2_video *video = &cdev->video;
	struct v4l2_subdev *subdev = &video->subdev;
	struct v4l2_async_notifier *notifier = &video->notifier;

	v4l2_async_unregister_subdev(subdev);
	v4l2_async_notifier_unregister(notifier);
	v4l2_async_notifier_cleanup(notifier);
	media_entity_cleanup(&subdev->entity);
	v4l2_device_unregister_subdev(subdev);

	return 0;
}

static const struct regmap_config sun8i_a83t_mipi_csi2_regmap_config = {
	.reg_bits       = 32,
	.reg_stride     = 4,
	.val_bits       = 32,
	.max_register	= 0x120,
};

static int sun8i_a83t_mipi_csi2_resource_request(struct sun8i_a83t_mipi_csi2_dev *cdev,
						 struct platform_device *pdev)
{
	struct resource *res;
	void __iomem *io_base;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	io_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(io_base))
		return PTR_ERR(io_base);

	cdev->regmap = devm_regmap_init_mmio_clk(&pdev->dev, "bus", io_base,
						 &sun8i_a83t_mipi_csi2_regmap_config);
	if (IS_ERR(cdev->regmap)) {
		dev_err(&pdev->dev, "failed to init register map\n");
		return PTR_ERR(cdev->regmap);
	}

	cdev->clk_mod = devm_clk_get(&pdev->dev, "mod");
	if (IS_ERR(cdev->clk_mod)) {
		dev_err(&pdev->dev, "failed to acquire csi clock\n");
		return PTR_ERR(cdev->clk_mod);
	}

	cdev->clk_mipi = devm_clk_get(&pdev->dev, "mipi");
	if (IS_ERR(cdev->clk_mod)) {
		dev_err(&pdev->dev, "failed to acquire mipi clock\n");
		return PTR_ERR(cdev->clk_mipi);
	}

	cdev->clk_misc = devm_clk_get(&pdev->dev, "misc");
	if (IS_ERR(cdev->clk_mod)) {
		dev_err(&pdev->dev, "failed to acquire misc clock\n");
		return PTR_ERR(cdev->clk_misc);
	}

	cdev->reset = devm_reset_control_get_shared(&pdev->dev, NULL);
	if (IS_ERR(cdev->reset)) {
		dev_err(&pdev->dev, "failed to get reset controller\n");
		return PTR_ERR(cdev->reset);
	}

	ret = sun8i_a83t_dphy_register(cdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to init MIPI D-PHY\n");
		return ret;
	}

	return 0;
}

static int sun8i_a83t_mipi_csi2_probe(struct platform_device *pdev)
{
	struct sun8i_a83t_mipi_csi2_dev *cdev;
	int ret;

	cdev = devm_kzalloc(&pdev->dev, sizeof(*cdev), GFP_KERNEL);
	if (!cdev)
		return -ENOMEM;

	cdev->dev = &pdev->dev;

	ret = sun8i_a83t_mipi_csi2_resource_request(cdev, pdev);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, cdev);

	ret = sun8i_a83t_mipi_csi2_v4l2_setup(cdev);
	if (ret)
		return ret;

	return 0;
}

static int sun8i_a83t_mipi_csi2_remove(struct platform_device *pdev)
{
	struct sun8i_a83t_mipi_csi2_dev *cdev = platform_get_drvdata(pdev);

	phy_exit(cdev->dphy);

	return sun8i_a83t_mipi_csi2_v4l2_teardown(cdev);
}

static const struct of_device_id sun8i_a83t_mipi_csi2_of_match[] = {
	{ .compatible = "allwinner,sun8i-a83t-mipi-csi2" },
	{},
};
MODULE_DEVICE_TABLE(of, sun8i_a83t_mipi_csi2_of_match);

static struct platform_driver sun8i_a83t_mipi_csi2_platform_driver = {
	.probe = sun8i_a83t_mipi_csi2_probe,
	.remove = sun8i_a83t_mipi_csi2_remove,
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(sun8i_a83t_mipi_csi2_of_match),
	},
};
module_platform_driver(sun8i_a83t_mipi_csi2_platform_driver);

MODULE_DESCRIPTION("Allwinner A83T MIPI CSI-2 and D-PHY Controller Driver");
MODULE_AUTHOR("Paul Kocialkowski <paul.kocialkowski@bootlin.com>");
MODULE_LICENSE("GPL");
