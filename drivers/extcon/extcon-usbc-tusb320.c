// SPDX-License-Identifier: GPL-2.0
/**
 * drivers/extcon/extcon-tusb320.c - TUSB320 extcon driver
 *
 * Copyright (C) 2020 National Instruments Corporation
 * Author: Michael Auchter <michael.auchter@ni.com>
 */

#include <linux/extcon-provider.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/err.h>
#include <linux/usb/typec.h>

#define TUSB320_REG9				0x9
#define TUSB320_REGa				0xa
#define TUSB320_REG45               		0x45
#define TUSB320_REG9_ATTACHED_STATE_SHIFT	6
#define TUSB320_REG9_ATTACHED_STATE_MASK	0x3
#define TUSB320_REG9_CABLE_DIRECTION		BIT(5)
#define TUSB320_REG9_INTERRUPT_STATUS		BIT(4)
#define TUSB320_ATTACHED_STATE_NONE		0x0
#define TUSB320_ATTACHED_STATE_DFP		0x1
#define TUSB320_ATTACHED_STATE_UFP		0x2
#define TUSB320_ATTACHED_STATE_ACC		0x3

/* Register REG_MODE_SET 0a */
#define TUSB320_REG_SET_MODE			(BIT(5) | BIT(4))
#define TUSB320_REG_SET_BY_PORT			0x00
#define TUSB320_REG_SET_UFP			BIT(4)
#define TUSB320_REG_SET_DFP			BIT(5)
#define TUSB320_REG_SET_DRP			(BIT(5) | BIT(4))
#define TUSB320_REG_SET_SOFT_RESET		BIT(3)
#define TUSB320_REG_SET_DISABLE_RD_RP		BIT(2)

#define DISABLE_SET 0
#define DISABLE_CLEAR 1

int state;

struct tusb320_priv {
	struct device *dev;
	struct regmap *regmap;
	struct extcon_dev *edev;
    struct gpio_desc *irq_gpiod, *otg_vbus_gpiod;
    int id_irq;
};

struct tusb320_priv *priv;

static const char * const tusb_attached_states[] = {
	[TUSB320_ATTACHED_STATE_NONE] = "not attached",
	[TUSB320_ATTACHED_STATE_DFP]  = "downstream facing port",
	[TUSB320_ATTACHED_STATE_UFP]  = "upstream facing port",
	[TUSB320_ATTACHED_STATE_ACC]  = "accessory",
};

static const unsigned int tusb320_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

static void tusb320_disabled_state_exit(enum typec_port_data port_mode);
static int tusb320_port_mode_set(enum typec_port_data port_mode);

static int tusb320_check_signature(struct tusb320_priv *priv)
{
	static const char sig[] = { '\0', 'T', 'U', 'S', 'B', '3', '2', '0' };
	unsigned val;
	int i, ret;

	for (i = 0; i < sizeof(sig); i++) {
		ret = regmap_read(priv->regmap, sizeof(sig) - 1 - i, &val);
		if (ret < 0)
			return ret;
		if (val != sig[i]) {
			dev_err(priv->dev, "signature mismatch!\n");
			return -ENODEV;
		}
	}

	return 0;
}

static irqreturn_t tusb320_irq_handler(int irq, void *dev_id)
{
	int polarity;
	unsigned reg;

	if (regmap_read(priv->regmap, TUSB320_REG9, &reg)) {
		printk("error during i2c read!\n");
		return IRQ_NONE;
	}

// 	if (!(reg & TUSB320_REG9_INTERRUPT_STATUS))
// 		return IRQ_NONE;

	state = (reg >> TUSB320_REG9_ATTACHED_STATE_SHIFT) &
		TUSB320_REG9_ATTACHED_STATE_MASK;
	polarity = !!(reg & TUSB320_REG9_CABLE_DIRECTION);

	printk("attached state: %s, polarity: %d\n",
		tusb_attached_states[state], polarity);
    
    if (state == TUSB320_ATTACHED_STATE_DFP || state == TUSB320_ATTACHED_STATE_ACC) {
        gpiod_set_value(priv->otg_vbus_gpiod, 0);
    } else if (state == TUSB320_ATTACHED_STATE_UFP){
        gpiod_set_value(priv->otg_vbus_gpiod, 1);
    } else {
        tusb320_port_mode_set(TUSB320_REG_SET_BY_PORT);
        gpiod_set_value(priv->otg_vbus_gpiod, 1);
    }

	extcon_set_state(priv->edev, EXTCON_USB,
			 state == TUSB320_ATTACHED_STATE_UFP);
	extcon_set_state(priv->edev, EXTCON_USB_HOST,
			 state == TUSB320_ATTACHED_STATE_DFP || state == TUSB320_ATTACHED_STATE_ACC);
	extcon_set_property(priv->edev, EXTCON_USB,
			    EXTCON_PROP_USB_TYPEC_POLARITY,
			    (union extcon_property_value)polarity);
	extcon_set_property(priv->edev, EXTCON_USB_HOST,
			    EXTCON_PROP_USB_TYPEC_POLARITY,
			    (union extcon_property_value)polarity);
    
	extcon_sync(priv->edev, EXTCON_USB);
	extcon_sync(priv->edev, EXTCON_USB_HOST);

	regmap_write(priv->regmap, TUSB320_REG9, reg);

	return IRQ_HANDLED;
}

static const struct regmap_config tusb320_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int tusb320_port_mode_set(enum typec_port_data port_mode)
{
    unsigned reg, mask_val;
    
	regmap_read(priv->regmap, TUSB320_REGa, &reg);
    
	switch (port_mode) {
	case TYPEC_PORT_UFP:
		mask_val = TUSB320_REG_SET_UFP;
		break;
	case TYPEC_PORT_DFP:
		mask_val = TUSB320_REG_SET_DFP;
		break;
	case TYPEC_PORT_DRD:
		mask_val = TUSB320_REG_SET_DRP;
		break;
	default:
		mask_val = TUSB320_REG_SET_BY_PORT;
		break;
	}
	reg &= ~TUSB320_REG_SET_MODE;
	reg |= mask_val;
    
	regmap_write(priv->regmap, TUSB320_REGa, reg);
	return 0;
}

static void tusb320_rd_rp_disable(int set)
{
	unsigned reg;
    
    regmap_read(priv->regmap, TUSB320_REG45, &reg);

	if (set == DISABLE_SET) {
		reg |= TUSB320_REG_SET_DISABLE_RD_RP;
	} else {
		reg &= ~((unsigned) TUSB320_REG_SET_DISABLE_RD_RP);
	}
	
	regmap_write(priv->regmap, TUSB320_REG45, reg);
}

static void tusb320_soft_reset(void)
{
	unsigned reg;
    
	regmap_read(priv->regmap, TUSB320_REGa, &reg);
    
	reg |= TUSB320_REG_SET_SOFT_RESET;
    
	regmap_write(priv->regmap, TUSB320_REGa, reg);
}

static void tusb320_disabled_state_start(void)
{
	tusb320_port_mode_set(TYPEC_PORT_UFP);
	tusb320_rd_rp_disable(DISABLE_SET);
	tusb320_soft_reset();
	mdelay(25);
}

static void tusb320_disabled_state_exit(enum typec_port_data port_mode)
{
	tusb320_port_mode_set(port_mode);
	tusb320_rd_rp_disable(DISABLE_CLEAR);
}

int set_usb_to_host(void) {
    
    gpiod_set_value(priv->otg_vbus_gpiod, 1);
    disable_irq(priv->id_irq);
    tusb320_disabled_state_start();
    tusb320_disabled_state_exit(TYPEC_PORT_UFP);
    enable_irq(priv->id_irq);

    return 0;
}

static int tusb320_extcon_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->dev = &client->dev;
    
    priv->irq_gpiod = devm_gpiod_get(priv->dev, "int", GPIOD_IN);
    if (IS_ERR(priv->irq_gpiod)) {
		dev_err(priv->dev, "failed to get int gpio\n");
		return -ENODEV;
	}
	
    priv->otg_vbus_gpiod = devm_gpiod_get(priv->dev, "otg-vbus", GPIOD_OUT_LOW);
    if (IS_ERR(priv->otg_vbus_gpiod)) {
		dev_err(priv->dev, "failed to get otg-vbus gpio\n");
		return -ENODEV;
	}
    
    priv->id_irq = gpiod_to_irq(priv->irq_gpiod);
    if (priv->id_irq < 0) {
        dev_err(priv->dev, "failed to get irq\n");
        return priv->id_irq;
    }

	priv->regmap = devm_regmap_init_i2c(client, &tusb320_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	ret = tusb320_check_signature(priv);
	if (ret)
		return ret;

	priv->edev = devm_extcon_dev_allocate(priv->dev, tusb320_extcon_cable);
	if (IS_ERR(priv->edev)) {
		dev_err(priv->dev, "failed to allocate extcon device\n");
		return PTR_ERR(priv->edev);
	}

	ret = devm_extcon_dev_register(priv->dev, priv->edev);
	if (ret < 0) {
		dev_err(priv->dev, "failed to register extcon device\n");
		return ret;
	}

	extcon_set_property_capability(priv->edev, EXTCON_USB,
				       EXTCON_PROP_USB_TYPEC_POLARITY);
	extcon_set_property_capability(priv->edev, EXTCON_USB_HOST,
				       EXTCON_PROP_USB_TYPEC_POLARITY);

	/* update initial state */
	tusb320_irq_handler(client->irq, priv);

	ret = devm_request_threaded_irq(priv->dev, priv->id_irq, NULL,
					tusb320_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					client->name, priv);

	return ret;
}

static const struct of_device_id tusb320_extcon_dt_match[] = {
	{ .compatible = "ti,tusb320", },
	{ }
};
MODULE_DEVICE_TABLE(of, tusb320_extcon_dt_match);

static struct i2c_driver tusb320_extcon_driver = {
	.probe		= tusb320_extcon_probe,
	.driver		= {
		.name	= "extcon-tusb320",
		.of_match_table = tusb320_extcon_dt_match,
	},
};

static int __init tusb320_init(void)
{
	return i2c_add_driver(&tusb320_extcon_driver);
}
subsys_initcall(tusb320_init);

static void __exit tusb320_exit(void)
{
	i2c_del_driver(&tusb320_extcon_driver);
}
module_exit(tusb320_exit);

MODULE_AUTHOR("Michael Auchter <michael.auchter@ni.com>");
MODULE_DESCRIPTION("TI TUSB320 extcon driver");
MODULE_LICENSE("GPL v2");
