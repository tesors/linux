/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2020 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#ifndef __SUN6I_MIPI_CSI2_H__
#define __SUN6I_MIPI_CSI2_H__

#include <linux/phy/phy.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define SUN6I_MIPI_CSI2_CTL_REG				0x0
#define SUN6I_MIPI_CSI2_CTL_RESET_N			BIT(31)
#define SUN6I_MIPI_CSI2_CTL_VERSION_EN			BIT(30)
#define SUN6I_MIPI_CSI2_CTL_UNPK_EN			BIT(1)
#define SUN6I_MIPI_CSI2_CTL_EN				BIT(0)
#define SUN6I_MIPI_CSI2_CFG_REG				0x4
#define SUN6I_MIPI_CSI2_CFG_CHANNEL_MODE(v)		((((v) - 1) << 8) & \
							 GENMASK(9, 8))
#define SUN6I_MIPI_CSI2_CFG_LANE_COUNT(v)		(((v) - 1) & GENMASK(1, 0))
#define SUN6I_MIPI_CSI2_VCDT_RX_REG			0x8
#define SUN6I_MIPI_CSI2_VCDT_RX_CH_VC(ch, vc)		(((vc) & GENMASK(1, 0)) << \
							 ((ch) * 8 + 6))
#define SUN6I_MIPI_CSI2_VCDT_RX_CH_DT(ch, t)		(((t) & GENMASK(5, 0)) << \
							 ((ch) * 8))
#define SUN6I_MIPI_CSI2_RX_PKT_NUM_REG			0xc

#define SUN6I_MIPI_CSI2_VERSION_REG			0x3c

#define SUN6I_MIPI_CSI2_CH_BASE				0x1000
#define SUN6I_MIPI_CSI2_CH_OFFSET			0x100

#define SUN6I_MIPI_CSI2_CH_CFG_REG			0x40
#define SUN6I_MIPI_CSI2_CH_INT_EN_REG			0x50
#define SUN6I_MIPI_CSI2_CH_INT_EN_EOT_ERR		BIT(29)
#define SUN6I_MIPI_CSI2_CH_INT_EN_CHKSUM_ERR		BIT(28)
#define SUN6I_MIPI_CSI2_CH_INT_EN_ECC_WRN		BIT(27)
#define SUN6I_MIPI_CSI2_CH_INT_EN_ECC_ERR		BIT(26)
#define SUN6I_MIPI_CSI2_CH_INT_EN_LINE_SYNC_ERR		BIT(25)
#define SUN6I_MIPI_CSI2_CH_INT_EN_FRAME_SYNC_ERR	BIT(24)
#define SUN6I_MIPI_CSI2_CH_INT_EN_EMB_DATA		BIT(18)
#define SUN6I_MIPI_CSI2_CH_INT_EN_PF			BIT(17)
#define SUN6I_MIPI_CSI2_CH_INT_EN_PH_UPDATE		BIT(16)
#define SUN6I_MIPI_CSI2_CH_INT_EN_LINE_START_SYNC	BIT(11)
#define SUN6I_MIPI_CSI2_CH_INT_EN_LINE_END_SYNC	BIT(10)
#define SUN6I_MIPI_CSI2_CH_INT_EN_FRAME_START_SYNC	BIT(9)
#define SUN6I_MIPI_CSI2_CH_INT_EN_FRAME_END_SYNC	BIT(8)
#define SUN6I_MIPI_CSI2_CH_INT_EN_FIFO_OVER		BIT(0)

#define SUN6I_MIPI_CSI2_CH_INT_PD_REG			0x58
#define SUN6I_MIPI_CSI2_CH_INT_PD_EOT_ERR		BIT(29)
#define SUN6I_MIPI_CSI2_CH_INT_PD_CHKSUM_ERR		BIT(28)
#define SUN6I_MIPI_CSI2_CH_INT_PD_ECC_WRN		BIT(27)
#define SUN6I_MIPI_CSI2_CH_INT_PD_ECC_ERR		BIT(26)
#define SUN6I_MIPI_CSI2_CH_INT_PD_LINE_SYNC_ERR		BIT(25)
#define SUN6I_MIPI_CSI2_CH_INT_PD_FRAME_SYNC_ERR	BIT(24)
#define SUN6I_MIPI_CSI2_CH_INT_PD_EMB_DATA		BIT(18)
#define SUN6I_MIPI_CSI2_CH_INT_PD_PF			BIT(17)
#define SUN6I_MIPI_CSI2_CH_INT_PD_PH_UPDATE		BIT(16)
#define SUN6I_MIPI_CSI2_CH_INT_PD_LINE_START_SYNC	BIT(11)
#define SUN6I_MIPI_CSI2_CH_INT_PD_LINE_END_SYNC		BIT(10)
#define SUN6I_MIPI_CSI2_CH_INT_PD_FRAME_START_SYNC	BIT(9)
#define SUN6I_MIPI_CSI2_CH_INT_PD_FRAME_END_SYNC	BIT(8)
#define SUN6I_MIPI_CSI2_CH_INT_PD_FIFO_OVER		BIT(0)

#define SUN6I_MIPI_CSI2_CH_DT_TRIGGER_REG		0x60
#define SUN6I_MIPI_CSI2_CH_CUR_PH_REG			0x70
#define SUN6I_MIPI_CSI2_CH_ECC_REG			0x74
#define SUN6I_MIPI_CSI2_CH_CKS_REG			0x78
#define SUN6I_MIPI_CSI2_CH_FRAME_NUM_REG		0x7c
#define SUN6I_MIPI_CSI2_CH_LINE_NUM_REG			0x80

#define SUN6I_MIPI_CSI2_CH_REG(reg, ch) \
	(SUN6I_MIPI_CSI2_CH_BASE + SUN6I_MIPI_CSI2_CH_OFFSET * (ch) + (reg))

enum mipi_csi2_data_type {
    MIPI_CSI2_DATA_TYPE_RGB888  = 0X24,
	MIPI_CSI2_DATA_TYPE_RAW8	= 0x2a,
	MIPI_CSI2_DATA_TYPE_RAW10	= 0x2b,
	MIPI_CSI2_DATA_TYPE_RAW12	= 0x2c,
};

struct sun6i_mipi_csi2_video {
	struct v4l2_fwnode_endpoint endpoint;
	struct v4l2_subdev subdev;
	struct media_pad pads[2];

	struct v4l2_async_subdev subdev_async;
	struct v4l2_async_notifier notifier;

	struct v4l2_subdev *remote_subdev;
	u32 remote_pad_index;
	u32 mbus_code;
};

struct sun6i_mipi_csi2_dev {
	struct device *dev;

	struct regmap *regmap;
	struct clk *clk_mod;
	struct reset_control *reset;
	struct phy *dphy;

	struct sun6i_mipi_csi2_video video;
};

#define sun6i_mipi_csi2_subdev_video(subdev) \
	container_of(subdev, struct sun6i_mipi_csi2_video, subdev)

#define sun6i_mipi_csi2_video_dev(video) \
	container_of(video, struct sun6i_mipi_csi2_dev, video)

#endif /* __SUN6I_MIPI_CSI2_H__ */
