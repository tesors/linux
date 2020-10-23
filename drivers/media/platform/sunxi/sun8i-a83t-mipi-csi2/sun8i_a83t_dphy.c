// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#include <linux/phy/phy.h>
#include <linux/regmap.h>

#include "sun8i_a83t_dphy.h"
#include "sun8i_a83t_mipi_csi2.h"

static int sun8i_a83t_dphy_set_mode(struct phy *dphy, enum phy_mode mode,
				    int submode)
{
	if (mode != PHY_MODE_MIPI_DPHY ||
	    submode != PHY_MIPI_DPHY_SUBMODE_RX)
		return -EINVAL;

	return 0;
};

static int sun8i_a83t_dphy_configure(struct phy *dphy,
				     union phy_configure_opts *opts)
{
	struct sun8i_a83t_mipi_csi2_dev *cdev = phy_get_drvdata(dphy);
	int ret;

	ret = phy_mipi_dphy_config_validate(&opts->mipi_dphy);
	if (ret)
		return ret;

	memcpy(&cdev->dphy_config, opts, sizeof(cdev->dphy_config));

	return 0;
};

static int sun8i_a83t_dphy_power_on(struct phy *dphy)
{
	struct sun8i_a83t_mipi_csi2_dev *cdev = phy_get_drvdata(dphy);
	struct regmap *regmap = cdev->regmap;

	regmap_write(regmap, SUN8I_A83T_DPHY_CTRL_REG,
		     SUN8I_A83T_DPHY_CTRL_RESET_N |
		     SUN8I_A83T_DPHY_CTRL_SHUTDOWN_N);

	regmap_write(regmap, SUN8I_A83T_DPHY_ANA0_REG,
		     SUN8I_A83T_DPHY_ANA0_REXT_EN |
		     SUN8I_A83T_DPHY_ANA0_RINT(2) |
		     SUN8I_A83T_DPHY_ANA0_SNK(2));

	return 0;
};

static int sun8i_a83t_dphy_power_off(struct phy *dphy)
{
	struct sun8i_a83t_mipi_csi2_dev *cdev = phy_get_drvdata(dphy);
	struct regmap *regmap = cdev->regmap;

	regmap_write(regmap, SUN8I_A83T_DPHY_CTRL_REG, 0);

	return 0;
};

static struct phy_ops sun8i_a83t_dphy_ops = {
	.set_mode	= sun8i_a83t_dphy_set_mode,
	.configure	= sun8i_a83t_dphy_configure,
	.power_on	= sun8i_a83t_dphy_power_on,
	.power_off	= sun8i_a83t_dphy_power_off,
};

int sun8i_a83t_dphy_register(struct sun8i_a83t_mipi_csi2_dev *cdev)
{
	struct phy_provider *phy_provider;

	cdev->dphy = devm_phy_create(cdev->dev, NULL, &sun8i_a83t_dphy_ops);
	if (IS_ERR(cdev->dphy)) {
		dev_err(cdev->dev, "failed to create D-PHY\n");
		return PTR_ERR(cdev->dphy);
	}

	phy_set_drvdata(cdev->dphy, cdev);

	phy_provider = devm_of_phy_provider_register(cdev->dev,
						     of_phy_simple_xlate);
	if (IS_ERR(phy_provider)) {
		dev_err(cdev->dev, "failed to register D-PHY provider\n");
		return PTR_ERR(phy_provider);
	}

	return 0;
}
